/*
 * vision_can_mqtt_final.cpp
 *
 * RPi1 자율주행 + MQTT 통합 최종 버전
 *
 * [RPi3 ↔ RPi1 토픽 구조]
 *   RPi3 → RPi1 수신:
 *     delivery/pin/{order_id}/3to1     ← PIN 정보
 *     delivery/command/{vehicle_id}    ← 출동 명령
 *     delivery/unlock/{vehicle_id}     ← 화물함 잠금 해제 허용
 *
 *   RPi1 → RPi3 송신:
 *     delivery/start/{vehicle_id}/1to3    ← 배달 시작
 *     delivery/vehicle/{vehicle_id}/status ← 차량 상태
 *     delivery/vehicle/{vehicle_id}/alert  ← 경보
 *     delivery/arrived/{vehicle_id}/1to3  ← 목적지 도착
 *     delivery/log/{vehicle_id}           ← 인증 실패 로그
 *     delivery/complete/{vehicle_id}/1to3 ← 배달 완료
 *
 * [주행 흐름]
 *   S_WAIT_CMD → S_FOLLOW → S_JUNC_STOP → S_JUNC_LEFT/RIGHT/STRAIGHT
 *   → S_REACQUIRE → S_DELIVER_WAIT → S_UTURN → S_FOLLOW(복귀) → S_FINISHED
 *
 * [컴파일]
 *   g++ vision_can_mqtt_final.cpp -o delivery_mqtt \
 *       $(pkg-config --cflags --libs opencv4) \
 *       -lmosquittopp -lpthread
 *
 * [실행]
 *   sudo ip link set can0 up type can bitrate 250000
 *   DISPLAY=:0 ./delivery_mqtt 10.42.0.161
 */

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <mosquittopp.h>
#include <nlohmann/json.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <stdint.h>
#include <time.h>
#include <cmath>
#include <fcntl.h>
#include <errno.h>
#include <vector>
#include <thread>
#include <atomic>
#include <string>
#include <sstream>
#include <iomanip>

using json = nlohmann::json;

// ===================== CAN 명령 코드 =====================
#define DIR_STOP      0
#define DIR_FORWARD   1
#define DIR_BACKWARD  2
#define DIR_LEFT      3
#define DIR_RIGHT     4
#define DIR_UTURN     5

// ===================== 주행 파라미터 =====================
#define DEFAULT_RPM       250.0f
#define STRAIGHT_DEADBAND 50

// ===================== 교차로 감지 파라미터 =====================
#define JUNC_STREAK      10
#define JUNC_COOLDOWN_MS 4000
#define HIGH_TH          0.40f

// ===================== 상태 지속 시간 =====================
#define DUR_JUNC_STOP_MS 1000
#define DUR_STRAIGHT_MS  500
#define DUR_TURN_MS      750
#define DUR_UTURN_MS     1300
#define DUR_REACQ_MS     400
#define DELIVER_WAIT_MS  10000

// ===================== 목적지 정의 =====================
#define DEST_A 1
#define DEST_B 2
#define DEST_C 3
#define DEST_D 4

// ===================== 맵 위치 인덱스 =====================
#define POS_START   0
#define POS_AB_JUNC 1
#define POS_A       2
#define POS_B       3
#define POS_CD_JUNC 4
#define POS_C       5
#define POS_D       6

// ===================== 상태머신 =====================
typedef enum {
    S_WAIT_CMD,         // MQTT 명령 대기
    S_FOLLOW,           // 라인트레이싱
    S_JUNC_STOP,        // 교차로 감지 후 정지
    S_JUNC_STRAIGHT,    // 교차로 직진
    S_JUNC_LEFT,        // 교차로 좌회전
    S_JUNC_RIGHT,       // 교차로 우회전
    S_REACQUIRE,        // 라인 재탐색
    S_DELIVER_WAIT,     // 목적지 도착 대기 (PIN 인증 대기)
    S_UTURN,            // U턴
    S_FINISHED          // 완료
} State;

typedef enum { J_NONE, J_PLUS, J_T } JuncType;

// ===================== 전역 변수 =====================
static float             g_rpm          = DEFAULT_RPM;
static std::atomic<int>  g_destination(0);
static std::atomic<bool> g_start_cmd(false);
static std::atomic<bool> g_unlock_received(false);
static std::atomic<bool> g_running(true);
static std::atomic<int>  g_map_position(POS_START);
static std::string       g_delivery_id  = "";
static std::string       g_vehicle_id   = "vehicle_001";
static std::string       g_pin          = "";
static uint8_t           g_next_action  = DIR_FORWARD;

// ===================== 유틸 함수 =====================
static uint64_t now_ms() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000ULL + ts.tv_nsec / 1000000ULL;
}

static std::string get_timestamp() {
    auto now = std::time(nullptr);
    auto tm  = *std::localtime(&now);
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y-%m-%d %H:%M:%S");
    return oss.str();
}

// ===================== CAN 송신 =====================
void send_cmd(int sock, uint8_t dir, float rpm) {
    struct can_frame frame;
    memset(&frame, 0, sizeof(frame));
    frame.can_id  = 0x010;
    frame.can_dlc = 3;
    uint16_t rpm_x10 = (uint16_t)(rpm * 10.0f);
    frame.data[0] = dir;
    frame.data[1] = (rpm_x10 >> 8) & 0xFF;
    frame.data[2] =  rpm_x10       & 0xFF;
    int flags = fcntl(sock, F_GETFL, 0);
    fcntl(sock, F_SETFL, flags | O_NONBLOCK);
    ssize_t ret = write(sock, &frame, sizeof(frame));
    if (ret < 0 && errno != EAGAIN && errno != EWOULDBLOCK)
        perror("CAN write");
}

void send_estop(int sock) {
    struct can_frame frame;
    memset(&frame, 0, sizeof(frame));
    frame.can_id  = 0x011;
    frame.can_dlc = 1;
    frame.data[0] = 0xFF;
    write(sock, &frame, sizeof(frame));
}

// MangoM32에 화물함 잠금 해제 대기 + PIN 전달 (CAN 0x012)
void send_cargo_unlock(int sock, const std::string& pin) {
    struct can_frame frame;
    memset(&frame, 0, sizeof(frame));
    frame.can_id  = 0x012;
    frame.can_dlc = 5;
    frame.data[0] = 0x01;
    for (int i = 0; i < 4 && i < (int)pin.size(); i++)
        frame.data[1 + i] = (uint8_t)pin[i];
    write(sock, &frame, sizeof(frame));
    printf("[CAN] 0x012 → MangoM32: 잠금 해제 대기 + PIN 전달\n");
}

// ===================== ArUco 감지 =====================
static int detect_aruco(const cv::Mat& frame) {
    static cv::aruco::Dictionary dict =
        cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    static cv::aruco::DetectorParameters params;
    static cv::aruco::ArucoDetector detector(dict, params);

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners, rejected;
    detector.detectMarkers(frame, corners, ids, rejected);
    if (ids.empty()) return -1;

    int best = 0; double best_area = 0;
    for (int i = 0; i < (int)ids.size(); i++) {
        double a = fabs(cv::contourArea(corners[i]));
        if (a > best_area) { best_area = a; best = i; }
    }
    return ids[best];
}

// ===================== 라인 추적 =====================
static uint8_t follow_line(const cv::Mat& frame, int& err_out, cv::Mat& vis) {
    int H = frame.rows, W = frame.cols;
    int y0 = (int)(H * 0.65);
    cv::Mat roi = frame(cv::Rect(0, y0, W, H - y0));

    cv::Mat gray, blur, bin;
    cv::cvtColor(roi, gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray, blur, cv::Size(7, 7), 0);
    cv::threshold(blur, bin, 0, 255, cv::THRESH_BINARY_INV | cv::THRESH_OTSU);
    cv::morphologyEx(bin, bin, cv::MORPH_OPEN,
        cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(bin, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    int best = -1; double best_area = 0;
    for (int i = 0; i < (int)contours.size(); i++) {
        double a = cv::contourArea(contours[i]);
        if (a > best_area) { best_area = a; best = i; }
    }

    if (best < 0 || best_area < 500) { err_out = 0; return DIR_FORWARD; }

    cv::Moments m = cv::moments(contours[best]);
    int cx = (m.m00 != 0) ? (int)(m.m10 / m.m00) : W / 2;
    err_out = cx - W / 2;

    cv::circle(vis, cv::Point(cx, y0 + roi.rows / 2), 8, cv::Scalar(0, 0, 255), -1);
    cv::rectangle(vis, cv::Point(0, y0), cv::Point(W, H), cv::Scalar(0, 255, 0), 2);
    cv::line(vis, cv::Point(W/2, y0), cv::Point(W/2, H), cv::Scalar(255, 0, 0), 2);

    if (abs(err_out) <= STRAIGHT_DEADBAND) return DIR_FORWARD;
    return (err_out < 0) ? DIR_LEFT : DIR_RIGHT;
}

// ===================== 교차로 감지 =====================
static JuncType classify_junction(const cv::Mat& bin_roi,
                                   float& dbg_L, float& dbg_M, float& dbg_R)
{
    int w = bin_roi.cols, h = bin_roi.rows;
    int lw = w / 3, mw = w / 3, rw = w - 2*(w/3);

    dbg_L = (float)cv::countNonZero(bin_roi(cv::Rect(0,     0, lw, h))) / (float)(lw * h);
    dbg_M = (float)cv::countNonZero(bin_roi(cv::Rect(lw,    0, mw, h))) / (float)(mw * h);
    dbg_R = (float)cv::countNonZero(bin_roi(cv::Rect(lw+mw, 0, rw, h))) / (float)(rw * h);

    if (dbg_L >= HIGH_TH && dbg_M >= HIGH_TH && dbg_R >= HIGH_TH) return J_PLUS;
    if (dbg_L >= HIGH_TH && dbg_R >= HIGH_TH) return J_T;
    return J_NONE;
}

// ===================== 목적지별 교차로 행동 결정 =====================
static uint8_t get_junction_action(int dest, int junc_count) {
    switch(dest) {
        case DEST_A: if (junc_count == 0) return DIR_LEFT;    break;
        case DEST_B: if (junc_count == 0) return DIR_RIGHT;   break;
        case DEST_C:
            if (junc_count == 0) return DIR_FORWARD;
            if (junc_count == 1) return DIR_LEFT;
            break;
        case DEST_D:
            if (junc_count == 0) return DIR_FORWARD;
            if (junc_count == 1) return DIR_RIGHT;
            break;
    }
    return DIR_FORWARD;
}

static void update_map_position(int dest, int junc_count) {
    if (junc_count == 0)      g_map_position = POS_AB_JUNC;
    else if (junc_count == 1) g_map_position = POS_CD_JUNC;
}

// ===================== MQTT 클라이언트 =====================
class VehicleMQTT : public mosqpp::mosquittopp {
public:
    bool connected = false;
    int  can_sock  = -1;

    VehicleMQTT() : mosqpp::mosquittopp("rpi1-vehicle") {
        username_pw_set("hoji", "1234");
    }

    bool start(const std::string& host, int port = 1883) {
        mosqpp::lib_init();
        int rc = connect(host.c_str(), port, 60);
        if (rc != MOSQ_ERR_SUCCESS) {
            fprintf(stderr, "[MQTT] connect failed: %d\n", rc);
            return false;
        }
        loop_start();
        for (int i = 0; i < 50 && !connected; i++)
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        return connected;
    }

    void publish_json(const std::string& topic, const json& data, int qos = 1) {
        if (!connected) return;
        std::string payload = data.dump();
        publish(nullptr, topic.c_str(), payload.size(), payload.c_str(), qos, false);
    }

    // ── RPi1 → RPi3 송신 함수들 ──────────────────────────────

    // delivery/vehicle/{id}/status (QoS 0, 주기적)
    void send_status(const std::string& status, int position) {
        json msg = {
            {"vehicle_id",  g_vehicle_id},
            {"delivery_id", g_delivery_id},
            {"status",      status},
            {"position",    position},
            {"timestamp",   get_timestamp()}
        };
        publish_json("delivery/vehicle/" + g_vehicle_id + "/status", msg, 0);
    }

    // delivery/start/{id}/1to3 (QoS 1)
    void send_start(int destination) {
        std::string dest_str = (destination==1)?"A":(destination==2)?"B":
                               (destination==3)?"C":"D";
        json msg = {
            {"vehicle_id",  g_vehicle_id},
            {"delivery_id", g_delivery_id},
            {"destination", dest_str},
            {"status",      "in_transit"},
            {"timestamp",   get_timestamp()}
        };
        publish_json("delivery/start/" + g_vehicle_id + "/1to3", msg, 1);
        printf("[MQTT] → RPi3: 배달 시작 (dest=%s)\n", dest_str.c_str());
    }

    // delivery/arrived/{id}/1to3 (QoS 1)
    void send_arrived(int destination) {
        std::string dest_str = (destination==1)?"A":(destination==2)?"B":
                               (destination==3)?"C":"D";
        json msg = {
            {"vehicle_id",  g_vehicle_id},
            {"delivery_id", g_delivery_id},
            {"destination", dest_str},
            {"status",      "arrived"},
            {"timestamp",   get_timestamp()}
        };
        publish_json("delivery/arrived/" + g_vehicle_id + "/1to3", msg, 1);
        printf("[MQTT] → RPi3: 목적지 도착 (dest=%s)\n", dest_str.c_str());
    }

    // delivery/complete/{id}/1to3 (QoS 1)
    void send_complete() {
        json msg = {
            {"vehicle_id",  g_vehicle_id},
            {"delivery_id", g_delivery_id},
            {"status",      "completed"},
            {"timestamp",   get_timestamp()}
        };
        publish_json("delivery/complete/" + g_vehicle_id + "/1to3", msg, 1);
        printf("[MQTT] → RPi3: 배달 완료\n");
    }

    // delivery/vehicle/{id}/alert (QoS 1)
    void send_alert(const std::string& alert_type) {
        json msg = {
            {"vehicle_id",  g_vehicle_id},
            {"delivery_id", g_delivery_id},
            {"type",        alert_type},
            {"timestamp",   get_timestamp()}
        };
        publish_json("delivery/vehicle/" + g_vehicle_id + "/alert", msg, 1);
        printf("[MQTT] → RPi3: 경보 (%s)\n", alert_type.c_str());
    }

    // delivery/log/{id} (MangoM32 인증 실패 로그 중계, QoS 1)
    void send_auth_log(const std::string& result, int attempt) {
        json msg = {
            {"vehicle_id",     g_vehicle_id},
            {"delivery_id",    g_delivery_id},
            {"result",         result},
            {"attempt_number", attempt},
            {"timestamp",      get_timestamp()}
        };
        publish_json("delivery/log/" + g_vehicle_id, msg, 1);
        printf("[MQTT] → RPi3: 인증 로그 (%s, %d회)\n", result.c_str(), attempt);
    }

protected:
    void on_connect(int rc) override {
        if (rc == 0) {
            connected = true;
            // ── RPi3 → RPi1 구독 ──
            subscribe(nullptr, "delivery/command/+",  1); // 출동 명령
            subscribe(nullptr, "delivery/pin/+/3to1", 2); // PIN 정보 (QoS 2)
            subscribe(nullptr, "delivery/unlock/+",   1); // 화물함 잠금 해제 허용
            printf("[MQTT] 연결 성공 & 구독 완료\n");
        } else {
            fprintf(stderr, "[MQTT] 연결 실패 rc=%d\n", rc);
        }
    }

    void on_disconnect(int rc) override {
        connected = false;
        printf("[MQTT] 연결 끊김 (rc=%d)\n", rc);
    }

    void on_message(const struct mosquitto_message* msg) override {
        std::string topic(msg->topic);
        std::string payload((char*)msg->payload, msg->payloadlen);

        try {
            auto data = json::parse(payload);

            // ── 출동 명령 (delivery/command/{vehicle_id}) ──
            if (topic.find("delivery/command/") != std::string::npos) {
                std::string dest_str = data["destination"].get<std::string>();
                int dest = (dest_str=="A")?1:(dest_str=="B")?2:
                           (dest_str=="C")?3:4;
                g_delivery_id = data.value("delivery_id", "");
                g_destination = dest;
                g_start_cmd   = true;
                printf("[MQTT] ← RPi3: 출동 명령 (dest=%s, id=%s)\n",
                       dest_str.c_str(), g_delivery_id.c_str());
            }

            // ── PIN 정보 (delivery/pin/{order_id}/3to1) ──
            else if (topic.find("delivery/pin/") != std::string::npos &&
                     topic.find("3to1") != std::string::npos) {
                g_pin         = data.value("pin", "");
                g_delivery_id = data.value("delivery_id", g_delivery_id);
                printf("[MQTT] ← RPi3: PIN 수신 (delivery=%s)\n",
                       g_delivery_id.c_str());
            }

            // ── 화물함 잠금 해제 허용 (delivery/unlock/{vehicle_id}) ──
            else if (topic.find("delivery/unlock/") != std::string::npos) {
                g_unlock_received = true;
                printf("[MQTT] ← RPi3: 화물함 잠금 해제 허용\n");
                // CAN 0x012 → MangoM32
                if (can_sock >= 0)
                    send_cargo_unlock(can_sock, g_pin);
            }

        } catch (const std::exception& e) {
            fprintf(stderr, "[MQTT] JSON 파싱 오류: %s\n", e.what());
        }
    }
};

// ===================== 메인 =====================
int main(int argc, char* argv[]) {
    std::string broker_host = "10.42.0.161";
    if (argc > 1) broker_host = argv[1];

    printf("=== RPi1 배달 차량 자율주행 + MQTT ===\n");
    printf("Broker: %s\n", broker_host.c_str());

    // ── CAN 초기화 ──
    int sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock < 0) { perror("socket"); return 1; }

    struct ifreq ifr;
    strcpy(ifr.ifr_name, "can0");
    if (ioctl(sock, SIOCGIFINDEX, &ifr) < 0) { perror("ioctl"); return 1; }

    struct sockaddr_can addr;
    memset(&addr, 0, sizeof(addr));
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(sock, (struct sockaddr*)&addr, sizeof(addr)) < 0)
    { perror("bind"); return 1; }

    // ── 카메라 초기화 ──
    cv::VideoCapture cap("/dev/video0");
    if (!cap.isOpened()) cap.open(0);
    if (!cap.isOpened()) { fprintf(stderr, "카메라 오픈 실패\n"); return 1; }
    cap.set(cv::CAP_PROP_FRAME_WIDTH,  640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

    // ── MQTT 초기화 ──
    VehicleMQTT mqtt;
    mqtt.can_sock = sock;
    bool mqtt_ok = mqtt.start(broker_host);
    if (mqtt_ok) printf("[MQTT] 브로커 연결 성공\n");
    else         printf("[MQTT] 브로커 연결 실패 - 단독 모드\n");

    // ── Heartbeat 스레드 (500ms마다 상태 전송) ──
    std::thread hb_thread([&mqtt]() {
        while (g_running) {
            std::string st = (g_map_position == POS_START) ? "idle" : "in_transit";
            mqtt.send_status(st, g_map_position.load());
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    });
    hb_thread.detach();

    printf("MQTT 출동 명령 대기 중... (키: 1=A 2=B 3=C 4=D e=E-Stop q=종료)\n");

    // ── 상태머신 초기화 ──
    State    state      = S_WAIT_CMD;
    uint64_t state_ts   = now_ms();
    uint64_t last_junc  = 0;
    uint64_t last_tx    = 0;
    int      streak     = 0;
    int      junc_count = 0;
    int      dest       = 0;
    bool     returning  = false;

    while (g_running) {
        cv::Mat frame;
        cap >> frame;
        if (frame.empty()) continue;

        uint64_t t   = now_ms();
        uint64_t dt  = t - state_ts;
        uint8_t  cmd = DIR_FORWARD;
        cv::Mat  vis = frame.clone();

        // ── 키 입력 ──
        int key = cv::waitKey(1) & 0xFF;
        if (key == 'q') break;
        if (key == 'e') {
            send_estop(sock);
            mqtt.send_alert("e_stop");
            printf("\n[E-Stop]\n");
        }
        if (key == '1') { g_destination = DEST_A; g_start_cmd = true; }
        if (key == '2') { g_destination = DEST_B; g_start_cmd = true; }
        if (key == '3') { g_destination = DEST_C; g_start_cmd = true; }
        if (key == '4') { g_destination = DEST_D; g_start_cmd = true; }

        // ── 상태머신 ──
        switch (state) {

        // ─ 명령 대기 ─
        case S_WAIT_CMD:
            cmd = DIR_STOP;
            if (g_start_cmd) {
                g_start_cmd       = false;
                g_unlock_received = false;
                dest              = g_destination.load();
                junc_count        = 0;
                streak            = 0;
                returning         = false;
                state             = S_FOLLOW;
                state_ts          = t;
                g_map_position    = POS_START;

                const char* dn = (dest==1)?"A":(dest==2)?"B":(dest==3)?"C":"D";
                printf("\n[START] 목적지: %s  delivery_id: %s\n",
                       dn, g_delivery_id.c_str());
                mqtt.send_start(dest);
            }
            break;

        // ─ 라인트레이싱 ─
        case S_FOLLOW: {
            int aid = detect_aruco(frame);

            // 목적지 ArUco 감지 (GO 모드)
            if (!returning && aid == dest) {
                int pos = (dest==1)?POS_A:(dest==2)?POS_B:(dest==3)?POS_C:POS_D;
                g_map_position = pos;
                state    = S_DELIVER_WAIT;
                state_ts = t;
                cmd      = DIR_STOP;
                printf("\n[ARRIVED] ArUco ID=%d → 목적지 도착!\n", aid);
                mqtt.send_arrived(dest);
                break;
            }

            // 출발지 ArUco ID=0 감지 (복귀 모드)
            if (returning && aid == 0) {
                g_map_position = POS_START;
                state    = S_FINISHED;
                state_ts = t;
                cmd      = DIR_STOP;
                printf("\n[RETURNED] ArUco ID=0 → 출발지 복귀 완료!\n");
                break;
            }

            // 교차로 감지
            int H = frame.rows, W = frame.cols;
            int yj0 = (int)(H * 0.40), yj1 = (int)(H * 0.62);
            cv::Mat jroi = frame(cv::Rect(0, yj0, W, yj1 - yj0));

            cv::Mat gray, blur, bin;
            cv::cvtColor(jroi, gray, cv::COLOR_BGR2GRAY);
            cv::GaussianBlur(gray, blur, cv::Size(5, 5), 0);
            cv::threshold(blur, bin, 0, 255, cv::THRESH_BINARY_INV | cv::THRESH_OTSU);
            cv::morphologyEx(bin, bin, cv::MORPH_OPEN,
                cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));

            float rL, rM, rR;
            JuncType jt = classify_junction(bin, rL, rM, rR);

            bool cool = (t - last_junc) > JUNC_COOLDOWN_MS;
            if (cool && jt != J_NONE) streak++;
            else if (!cool)           streak = 0;
            else                      streak = std::max(0, streak - 1);

            cv::rectangle(vis, cv::Point(0, yj0), cv::Point(W, yj1),
                          cv::Scalar(255, 0, 255), 2);

            char dbg[80];
            snprintf(dbg, sizeof(dbg), "S=%d/%d J=%d L=%.2f M=%.2f R=%.2f",
                     streak, JUNC_STREAK, junc_count, rL, rM, rR);
            cv::putText(vis, dbg, cv::Point(5, 90),
                        cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(255,255,0), 1);

            if (cool && streak >= JUNC_STREAK) {
                streak    = 0;
                last_junc = t;

                update_map_position(dest, junc_count);
                uint8_t action = get_junction_action(dest, junc_count);
                junc_count++;

                const char* an = (action==DIR_FORWARD)?"직진":
                                 (action==DIR_LEFT)?"좌회전":"우회전";
                printf("\n[교차로 %d번째] → %s (L=%.2f M=%.2f R=%.2f)\n",
                       junc_count, an, rL, rM, rR);

                mqtt.send_status("in_transit", g_map_position.load());

                g_next_action = action;
                state    = S_JUNC_STOP;
                state_ts = t;
                cmd      = DIR_STOP;
                break;
            }

            // 라인트레이싱
            int err = 0;
            cmd = follow_line(frame, err, vis);
            break;
        }

        // ─ 교차로 정지 (1초) ─
        case S_JUNC_STOP:
            cmd = DIR_STOP;
            if (dt >= DUR_JUNC_STOP_MS) {
                if      (g_next_action == DIR_FORWARD) state = S_JUNC_STRAIGHT;
                else if (g_next_action == DIR_LEFT)    state = S_JUNC_LEFT;
                else                                   state = S_JUNC_RIGHT;
                state_ts = t;
            }
            break;

        case S_JUNC_STRAIGHT:
            cmd = DIR_FORWARD;
            if (dt >= DUR_STRAIGHT_MS) { state = S_REACQUIRE; state_ts = t; }
            break;

        case S_JUNC_LEFT:
            cmd = DIR_LEFT;
            if (dt >= DUR_TURN_MS) { state = S_REACQUIRE; state_ts = t; }
            break;

        case S_JUNC_RIGHT:
            cmd = DIR_RIGHT;
            if (dt >= DUR_TURN_MS) { state = S_REACQUIRE; state_ts = t; }
            break;

        case S_REACQUIRE: {
            int err = 0;
            cmd = follow_line(frame, err, vis);
            if (dt >= DUR_REACQ_MS) { state = S_FOLLOW; state_ts = t; }
            break;
        }

        // ─ 목적지 도착 대기 (PIN 인증 대기) ─
        case S_DELIVER_WAIT:
            cmd = DIR_STOP;
            {
                uint64_t safe_dt = (t >= state_ts) ? (t - state_ts) : 0;
                int remain = (safe_dt < DELIVER_WAIT_MS) ?
                             (int)((DELIVER_WAIT_MS - safe_dt) / 1000) + 1 : 0;
                printf("\r[배달 대기] %d초  ", remain);
                fflush(stdout);

                char wait_str[32];
                snprintf(wait_str, sizeof(wait_str), "WAIT %ds", remain);
                cv::putText(vis, wait_str, cv::Point(10, 120),
                            cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0,255,255), 2);

                // 타임아웃 → 강제 복귀
                if (safe_dt >= DELIVER_WAIT_MS) {
                    printf("\n[TIMEOUT] 수령 없음 → 복귀 시작\n");
                    mqtt.send_complete();
                    state     = S_UTURN;
                    state_ts  = t;
                    returning = true;
                }
            }
            break;

        // ─ U턴 ─
        case S_UTURN:
            cmd = DIR_UTURN;
            if (dt >= DUR_UTURN_MS) {
                state    = S_REACQUIRE;
                state_ts = t;
                printf("\n[UTURN] 완료 → 복귀 라인트레이싱\n");
            }
            break;

        // ─ 완료 ─
        case S_FINISHED:
            cmd = DIR_STOP;
            send_cmd(sock, DIR_STOP, 0);
            mqtt.send_complete();
            printf("\n=============================\n");
            printf("      배달 완료! 출발지 복귀\n");
            printf("=============================\n");
            state         = S_WAIT_CMD;
            state_ts      = t;
            g_destination = 0;
            junc_count    = 0;
            returning     = false;
            break;

        default: break;
        }

        // ── CAN 송신 (50ms마다) ──
        if ((t - last_tx) >= 50) {
            float tx_rpm = (cmd == DIR_STOP) ? 0.0f : g_rpm;
            send_cmd(sock, cmd, tx_rpm);
            last_tx = t;
        }

        // ── 시각화 ──
        const char* stname =
            (state == S_WAIT_CMD)      ? "WAIT_CMD" :
            (state == S_FOLLOW)        ? "FOLLOW"   :
            (state == S_JUNC_STOP)     ? "JUNC_STP" :
            (state == S_JUNC_STRAIGHT) ? "JUNC_ST"  :
            (state == S_JUNC_LEFT)     ? "JUNC_L"   :
            (state == S_JUNC_RIGHT)    ? "JUNC_R"   :
            (state == S_REACQUIRE)     ? "REACQ"    :
            (state == S_DELIVER_WAIT)  ? "DELIVER"  :
            (state == S_UTURN)         ? "UTURN"    : "FINISH";

        const char* dest_name = (dest==1)?"A":(dest==2)?"B":(dest==3)?"C":(dest==4)?"D":"?";

        cv::putText(vis, stname, cv::Point(10, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 0.9, cv::Scalar(0,255,255), 2);
        cv::putText(vis, std::string(returning ? "RETURN:" : "GO:") + dest_name,
                    cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                    cv::Scalar(0,255,0), 2);

        char pos_str[32];
        snprintf(pos_str, sizeof(pos_str), "POS:%d", g_map_position.load());
        cv::putText(vis, pos_str, cv::Point(10, 90),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255,200,0), 2);

        cv::imshow("Delivery CAM", vis);
        usleep(5000);
    }

    g_running = false;
    send_estop(sock);
    cv::destroyAllWindows();
    close(sock);
    cap.release();
    printf("\n프로그램 종료\n");
    return 0;
}

