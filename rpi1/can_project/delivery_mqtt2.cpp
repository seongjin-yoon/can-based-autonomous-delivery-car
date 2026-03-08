/*
 * delivery_mqtt.cpp
 *
 * 라인트레이싱 + ArUco + MQTT 통합 버전
 *
 * [맵 구조]
 *        C ————————— D
 *              |
 *        A ————————— B
 *              |
 *            출발지
 *
 * [목적지별 경로]
 *   A: 1번교차로 좌회전 → ArUco ID=1 → 정지
 *   B: 1번교차로 우회전 → ArUco ID=2 → 정지
 *   C: 1번교차로 직진 → 2번교차로 좌회전 → ArUco ID=3 → 정지
 *   D: 1번교차로 직진 → 2번교차로 우회전 → ArUco ID=4 → 정지
 *
 * 컴파일:
 *   g++ delivery_mqtt.cpp -o delivery \
 *       $(pkg-config --cflags --libs opencv4) \
 *       -lmosquittopp -lpthread
 * 실행:
 *   ./delivery [broker_ip]
 */

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <mosquittopp.h>
#include <nlohmann/json.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <termios.h>
#include <net/if.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <stdint.h>
#include <sys/select.h>
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
#define DEFAULT_RPM       100.0f
#define TURN_RPM          150.0f
#define STRAIGHT_DEADBAND 50

// ===================== 교차로 감지 파라미터 =====================
// ★ STREAK 낮춤: 빠르게 지나가도 인식되게
//
#define JUNC_STREAK       3
// ★ 출발 후 교차로 무시 시간
#define JUNC_START_DELAY  4000
#define JUNC_COOLDOWN_MS  3000
// ★ HIGH_TH 낮춤: 한쪽만 높아도 인식되게
#define HIGH_TH           0.35f

// ===================== 상태 지속 시간 =====================
#define DUR_JUNC_STOP_MS  500     // 교차로 정지 (짧게)
#define DUR_STRAIGHT_MS   600     // 직진 통과
#define DUR_TURN_MS       3000    // 회전 시간 (넉넉하게)
#define DUR_UTURN_MS      2000    // 유턴
#define DUR_REACQ_MS      600     // 라인 재탐색
#define DELIVER_WAIT_MS   10000   // 배달 대기

// ===================== 상태머신 =====================
typedef enum {
    S_WAIT_CMD,
    S_FOLLOW,
    S_JUNC_STOP,
    S_JUNC_STRAIGHT,
    S_JUNC_LEFT,
    S_JUNC_RIGHT,
    S_REACQUIRE,
    S_DELIVER_WAIT,
    S_UTURN,
    S_FINISHED
} State;

typedef enum { J_NONE, J_PLUS, J_T } JuncType;

// ===================== 전역 변수 =====================
static struct termios orig_termios;
static float          g_rpm         = DEFAULT_RPM;
static uint8_t        g_next_action = DIR_FORWARD;

static std::atomic<int>  g_destination(0);
static std::atomic<bool> g_start_cmd(false);
static std::atomic<bool> g_running(true);
static std::string       g_delivery_id = "";
static std::string       g_vehicle_id  = "vehicle_001";

// ===================== 목적지별 교차로 행동 =====================
static uint8_t get_junction_action(int dest, int junc_count, bool returning) {
    if (!returning) {
        switch (dest) {
            case 1: if (junc_count == 0) return DIR_LEFT;    break;
            case 2: if (junc_count == 0) return DIR_RIGHT;   break;
            case 3:
                if (junc_count == 0) return DIR_FORWARD;
                if (junc_count == 1) return DIR_LEFT;
                break;
            case 4:
                if (junc_count == 0) return DIR_FORWARD;
                if (junc_count == 1) return DIR_RIGHT;
                break;
        }
    } else {
        switch (dest) {
            case 1: if (junc_count == 0) return DIR_RIGHT;   break;
            case 2: if (junc_count == 0) return DIR_LEFT;    break;
            case 3:
                if (junc_count == 0) return DIR_RIGHT;
                if (junc_count == 1) return DIR_FORWARD;
                break;
            case 4:
                if (junc_count == 0) return DIR_LEFT;
                if (junc_count == 1) return DIR_FORWARD;
                break;
        }
    }
    return DIR_FORWARD;
}

static int get_target_aruco(int dest) { return dest; }
static int get_max_junc(int dest) {
    if (dest == 1 || dest == 2) return 1;
    return 2;
}

// ===================== 유틸 =====================
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

void term_raw_on() {
    struct termios raw;
    tcgetattr(STDIN_FILENO, &orig_termios);
    raw = orig_termios;
    raw.c_lflag &= ~(ICANON | ECHO);
    raw.c_cc[VMIN] = 1; raw.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSAFLUSH, &raw);
}
void term_raw_off() { tcsetattr(STDIN_FILENO, TCSAFLUSH, &orig_termios); }

// ===================== CAN =====================
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
// ★ 핵심 수정: 한쪽(L 또는 R)만 높고 M도 높으면 교차로로 인식
static JuncType classify_junction(const cv::Mat& bin_roi,
                                   float& dbg_L, float& dbg_M, float& dbg_R)
{
    int w = bin_roi.cols, h = bin_roi.rows;
    int lw = w / 3, mw = w / 3, rw = w - 2*(w/3);

    dbg_L = (float)cv::countNonZero(bin_roi(cv::Rect(0,     0, lw, h))) / (float)(lw * h);
    dbg_M = (float)cv::countNonZero(bin_roi(cv::Rect(lw,    0, mw, h))) / (float)(mw * h);
    dbg_R = (float)cv::countNonZero(bin_roi(cv::Rect(lw+mw, 0, rw, h))) / (float)(rw * h);

    // 십자: L, M, R 모두 높음
    if (dbg_L >= HIGH_TH && dbg_M >= HIGH_TH && dbg_R >= HIGH_TH)
        return J_PLUS;

    // T자: L과 R 둘 다 높음
    if (dbg_L >= HIGH_TH && dbg_R >= HIGH_TH)
        return J_T;

    // ★ 추가: RC카가 치우쳐 있어도 인식되게
    // 한쪽만 높고 M도 높으면 교차로
    if ((dbg_L >= HIGH_TH || dbg_R >= HIGH_TH) && dbg_M >= HIGH_TH)
        return J_T;

    return J_NONE;
}

// ===================== MQTT 클라이언트 =====================
class VehicleMQTT : public mosqpp::mosquittopp {
public:
    bool connected = false;

    VehicleMQTT() : mosqpp::mosquittopp("rpi1-vehicle") {
        username_pw_set("hoji", "1234");
    }

    bool start(const std::string& host, int port = 1883) {
        mosqpp::lib_init();
        int rc = connect(host.c_str(), port, 60);
        if (rc != MOSQ_ERR_SUCCESS) {
            fprintf(stderr, "[MQTT] 연결 실패 rc=%d - 오프라인 모드\n", rc);
            return false;
        }
        loop_start();
        for (int i = 0; i < 50 && !connected; i++)
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        return connected;
    }

    void pub(const std::string& topic, const json& data, int qos = 1) {
        if (!connected) return;
        std::string payload = data.dump();
        publish(nullptr, topic.c_str(), payload.size(), payload.c_str(), qos, false);
    }

    void send_status(const std::string& status) {
        json msg = {
            {"vehicle_id",  g_vehicle_id},
            {"delivery_id", g_delivery_id},
            {"status",      status},
            {"timestamp",   get_timestamp()}
        };
        pub("delivery/vehicle/" + g_vehicle_id + "/status", msg, 0);
    }

    void send_start(int dest) {
        const char* dn = (dest==1)?"A":(dest==2)?"B":(dest==3)?"C":"D";
        json msg = {
            {"vehicle_id",  g_vehicle_id},
            {"delivery_id", g_delivery_id},
            {"destination", std::string(dn)},
            {"status",      "in_transit"},
            {"timestamp",   get_timestamp()}
        };
        pub("delivery/start/" + g_vehicle_id + "/1to3", msg, 1);
        printf("[MQTT] 출발 → 목적지 %s\n", dn);
    }

    void send_arrived(int dest) {
        const char* dn = (dest==1)?"A":(dest==2)?"B":(dest==3)?"C":"D";
        json msg = {
            {"vehicle_id",  g_vehicle_id},
            {"delivery_id", g_delivery_id},
            {"destination", std::string(dn)},
            {"status",      "arrived"},
            {"timestamp",   get_timestamp()}
        };
        pub("delivery/arrived/" + g_vehicle_id + "/1to3", msg, 1);
        printf("[MQTT] 도착! 목적지 %s\n", dn);
    }

    void send_complete() {
        json msg = {
            {"vehicle_id",  g_vehicle_id},
            {"delivery_id", g_delivery_id},
            {"status",      "completed"},
            {"timestamp",   get_timestamp()}
        };
        pub("delivery/complete/" + g_vehicle_id + "/1to3", msg, 1);
        printf("[MQTT] 배달 완료\n");
    }

protected:
    void on_connect(int rc) override {
        if (rc == 0) {
            connected = true;
            subscribe(nullptr, "delivery/command/+", 1);
            subscribe(nullptr, "delivery/pin/+/3to1", 2);
            printf("[MQTT] 연결됨 - 명령 대기 중\n");
        }
    }

    void on_disconnect(int rc) override {
        connected = false;
        printf("[MQTT] 연결 끊김 rc=%d\n", rc);
    }

    void on_message(const struct mosquitto_message* msg) override {
        std::string topic(msg->topic);
        std::string payload((char*)msg->payload, msg->payloadlen);
        try {
            auto data = json::parse(payload);
            if (topic.find("delivery/command/") != std::string::npos) {
                std::string dest_str = data["destination"].get<std::string>();
                int dest = (dest_str=="A")?1:(dest_str=="B")?2:
                           (dest_str=="C")?3:4;
                g_delivery_id = data.value("delivery_id", "");
                g_destination = dest;
                g_start_cmd   = true;
                printf("\n[MQTT] 출동 명령! 목적지=%s delivery_id=%s\n",
                       dest_str.c_str(), g_delivery_id.c_str());
            }
            if (topic.find("delivery/pin/") != std::string::npos &&
                topic.find("3to1") != std::string::npos) {
                g_delivery_id = data.value("delivery_id", "");
                printf("[MQTT] PIN 수신 delivery_id=%s\n", g_delivery_id.c_str());
            }
        } catch (...) {}
    }
};

// ===================== 메인 =====================
int main(int argc, char* argv[]) {
    std::string broker_host = "10.42.0.1";
    if (argc > 1) broker_host = argv[1];

    printf("=== 배달 차량 (라인트레이싱 + MQTT) ===\n");
    printf("목적지: A=1 B=2 C=3 D=4\n");
    printf("키: q=종료 e=E-Stop 1/2/3/4=수동목적지설정\n");
    printf("브로커: %s\n\n", broker_host.c_str());

    // CAN 초기화
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

    // 카메라 초기화
    cv::VideoCapture cap("/dev/video0");
    if (!cap.isOpened()) cap.open(0);
    if (!cap.isOpened()) { fprintf(stderr, "카메라 오픈 실패\n"); return 1; }
    cap.set(cv::CAP_PROP_FRAME_WIDTH,  320);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 240);
    cap.set(cv::CAP_PROP_FPS, 30);
    cap.set(cv::CAP_PROP_BUFFERSIZE, 1);

    // MQTT 초기화
    VehicleMQTT mqtt;
    bool mqtt_ok = mqtt.start(broker_host);
    if (!mqtt_ok) printf("[MQTT] 오프라인 모드 - 키보드로 목적지 설정\n");

    // Heartbeat 스레드
    std::thread hb([&mqtt]() {
        while (g_running) {
            std::string st = (g_destination == 0) ? "idle" : "in_transit";
            mqtt.send_status(st);
            std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        }
    });
    hb.detach();

    term_raw_on();

    State    state      = S_WAIT_CMD;
    uint64_t state_ts   = now_ms();
    uint64_t last_junc  = 0;
    uint64_t last_tx    = 0;
    uint64_t start_ts   = 0;
    int      streak     = 0;
    int      junc_count = 0;
    int      dest       = 0;
    bool     returning  = false;

    printf("MQTT 출동 명령 대기 중...\n");

    while (1) {
        cv::Mat frame;
        for (int i = 0; i < 2; i++) cap.grab();
        if (!cap.retrieve(frame) || frame.empty()) continue;

        uint64_t t  = now_ms();
        uint64_t dt = t - state_ts;
        uint8_t  cmd = DIR_STOP;
        cv::Mat  vis = frame.clone();

        // ── 키 입력 ──
        {
            fd_set fds; struct timeval tv = {0, 0};
            FD_ZERO(&fds); FD_SET(STDIN_FILENO, &fds);
            if (select(STDIN_FILENO+1, &fds, NULL, NULL, &tv) > 0) {
                char key = getchar();
                if (key == 'q') { send_cmd(sock, DIR_STOP, 0); goto exit_loop; }
                if (key == 'e') { send_estop(sock); printf("\n[E-Stop]\n"); }
                if (key=='1'||key=='2'||key=='3'||key=='4') {
                    g_destination = key - '0';
                    g_start_cmd   = true;
                    printf("\n[수동] 목적지 %c 설정\n", key);
                }
            }
        }

        // ── 상태머신 ──
        switch (state) {

        case S_WAIT_CMD:
            cmd = DIR_STOP;
            if (g_start_cmd) {
                g_start_cmd = false;
                dest        = g_destination.load();
                junc_count  = 0;
                streak      = 0;
                returning   = false;
                start_ts    = t;
                last_junc   = t;  // ★ 출발 직후 쿨다운 적용
                state       = S_FOLLOW;
                state_ts    = t;
                const char* dn = (dest==1)?"A":(dest==2)?"B":(dest==3)?"C":"D";
                printf("\n[출발] 목적지: %s (%dms 후 교차로 감지 시작)\n",
                       dn, JUNC_START_DELAY);
                mqtt.send_start(dest);
            }
            break;

        case S_FOLLOW: {
            // ArUco 감지
            int aid = detect_aruco(frame);
            if (!returning && dest > 0 && aid == get_target_aruco(dest)) {
                state    = S_DELIVER_WAIT;
                state_ts = t;
                cmd      = DIR_STOP;
                printf("\n[도착!] ArUco ID=%d 감지 → 정지\n", aid);
                mqtt.send_arrived(dest);
                break;
            }
            if (returning && aid == 0) {
                state    = S_FINISHED;
                state_ts = t;
                cmd      = DIR_STOP;
                printf("\n[복귀 완료] ArUco ID=0 감지!\n");
                break;
            }

            // 교차로 감지 ROI
            int H = frame.rows, W = frame.cols;
            // ★ ROI를 위로 올려서 더 일찍 교차로 감지
            int yj0 = (int)(H * 0.30);
            int yj1 = (int)(H * 0.55);
            cv::Mat jroi = frame(cv::Rect(0, yj0, W, yj1 - yj0));

            cv::Mat gray, blur, bin;
            cv::cvtColor(jroi, gray, cv::COLOR_BGR2GRAY);
            cv::GaussianBlur(gray, blur, cv::Size(5, 5), 0);
            cv::threshold(blur, bin, 0, 255, cv::THRESH_BINARY_INV | cv::THRESH_OTSU);
            cv::morphologyEx(bin, bin, cv::MORPH_OPEN,
                cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));

            float rL, rM, rR;
            JuncType jt = classify_junction(bin, rL, rM, rR);

            // ★ 출발 후 딜레이 + 쿨다운 둘 다 체크
            bool start_ok = (t - start_ts) >= JUNC_START_DELAY;
            bool cool     = (t - last_junc) > JUNC_COOLDOWN_MS;

            if (start_ok && cool && jt != J_NONE) streak++;
            else if (!cool || !start_ok)           streak = 0;
            else                                   streak = std::max(0, streak - 1);

            // 시각화
            cv::rectangle(vis, cv::Point(0, yj0), cv::Point(W, yj1),
                          cv::Scalar(255, 0, 255), 2);
            cv::line(vis, cv::Point(W/3,   yj0), cv::Point(W/3,   yj1), cv::Scalar(255,255,0), 1);
            cv::line(vis, cv::Point(2*W/3, yj0), cv::Point(2*W/3, yj1), cv::Scalar(255,255,0), 1);

            char dbg[80];
            if (!start_ok) {
                int remain = (int)((JUNC_START_DELAY - (t - start_ts)) / 1000) + 1;
                snprintf(dbg, sizeof(dbg), "S=%d/%d J=%d %s L=%.2f M=%.2f R=%.2f [딜레이 %ds]",
                         streak, JUNC_STREAK, junc_count,
                         returning?"RET":"GO", rL, rM, rR, remain);
            } else {
                snprintf(dbg, sizeof(dbg), "S=%d/%d J=%d %s L=%.2f M=%.2f R=%.2f",
                         streak, JUNC_STREAK, junc_count,
                         returning?"RET":"GO", rL, rM, rR);
            }
            cv::putText(vis, dbg, cv::Point(5, 90),
                        cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255,255,0), 1);

            // ★ 교차로 인식 조건
            if (start_ok && cool && streak >= JUNC_STREAK &&
                junc_count < get_max_junc(dest)) {

                streak    = 0;
                last_junc = t;

                uint8_t action = get_junction_action(dest, junc_count, returning);
                junc_count++;

                const char* an = (action==DIR_FORWARD)?"직진":
                                 (action==DIR_LEFT)?"좌회전":"우회전";
                printf("\n[교차로 %d번째] %s → %s (L=%.2f M=%.2f R=%.2f)\n",
                       junc_count, returning?"복귀":"GO", an, rL, rM, rR);

                g_next_action = action;
                state         = S_JUNC_STOP;
                state_ts      = t;
                cmd           = DIR_STOP;
                break;
            }

            // 라인트레이싱
            int err = 0;
            cmd = follow_line(frame, err, vis);
            if ((t - last_tx) >= 200) {
                const char* d = (cmd==DIR_FORWARD)?"ST":(cmd==DIR_LEFT)?"L":"R";
                printf("\r[%s err=%4d] DEST:%d %s JUNC:%d",
                       d, err, dest, returning?"RET":"GO", junc_count);
                fflush(stdout);
            }
            break;
        }

        case S_JUNC_STOP:
            cmd = DIR_STOP;
            if (dt >= DUR_JUNC_STOP_MS) {
                if      (g_next_action == DIR_FORWARD) state = S_JUNC_STRAIGHT;
                else if (g_next_action == DIR_LEFT)    state = S_JUNC_LEFT;
                else                                   state = S_JUNC_RIGHT;
                state_ts = t;
                const char* an = (g_next_action==DIR_FORWARD)?"직진":
                                 (g_next_action==DIR_LEFT)?"좌회전":"우회전";
                printf("\n[행동 시작] %s (%dms)\n", an, DUR_TURN_MS);
            }
            break;

        case S_JUNC_STRAIGHT:
            if ((t - last_tx) >= 50) {
                send_cmd(sock, DIR_FORWARD, DEFAULT_RPM);
                last_tx = t;
            }
            cmd = DIR_FORWARD;
            if (dt >= DUR_STRAIGHT_MS) { state = S_REACQUIRE; state_ts = t; }
            break;

        case S_JUNC_LEFT:
            if ((t - last_tx) >= 50) {
                send_cmd(sock, DIR_LEFT, TURN_RPM);
                last_tx = t;
            }
            cmd = DIR_LEFT;
            if (dt >= DUR_TURN_MS) {
                state    = S_REACQUIRE;
                state_ts = t;
                printf("\n[좌회전 완료] 라인 재탐색\n");
            }
            break;

        case S_JUNC_RIGHT:
            if ((t - last_tx) >= 50) {
                send_cmd(sock, DIR_RIGHT, TURN_RPM);
                last_tx = t;
            }
            cmd = DIR_RIGHT;
            if (dt >= DUR_TURN_MS) {
                state    = S_REACQUIRE;
                state_ts = t;
                printf("\n[우회전 완료] 라인 재탐색\n");
            }
            break;

        case S_REACQUIRE: {
            int err = 0;
            cmd = follow_line(frame, err, vis);
            if ((t - last_tx) >= 50) {
                send_cmd(sock, cmd, DEFAULT_RPM);
                last_tx = t;
            }
            if (dt >= DUR_REACQ_MS) {
                // ★ 복귀 출발도 딜레이 적용
                start_ts = t;
                last_junc = t;
                state    = S_FOLLOW;
                state_ts = t;
            }
            break;
        }

        case S_DELIVER_WAIT:
            cmd = DIR_STOP;
            {
                int remain = (dt < DELIVER_WAIT_MS) ?
                             (int)((DELIVER_WAIT_MS - dt) / 1000) + 1 : 0;
                printf("\r[배달 대기] %d초   ", remain);
                fflush(stdout);
                char ws[32];
                snprintf(ws, sizeof(ws), "WAIT %ds", remain);
                cv::putText(vis, ws, cv::Point(10, 120),
                            cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0,255,255), 2);
            }
            if (dt >= DELIVER_WAIT_MS) {
                state    = S_UTURN;
                state_ts = t;
                printf("\n[U턴] 복귀 시작\n");
                mqtt.send_arrived(dest);
            }
            break;

        case S_UTURN:
            if ((t - last_tx) >= 50) {
                send_cmd(sock, DIR_UTURN, TURN_RPM);
                last_tx = t;
            }
            cmd = DIR_UTURN;
            if (dt >= DUR_UTURN_MS) {
                returning  = true;
                junc_count = 0;
                start_ts   = t;
                last_junc  = t;
                state      = S_REACQUIRE;
                state_ts   = t;
                printf("\n[복귀 모드] 시작\n");
            }
            break;

        case S_FINISHED:
            send_cmd(sock, DIR_STOP, 0);
            mqtt.send_complete();
            printf("\n==============================\n");
            printf("        배달 완료!\n");
            printf("==============================\n");
            dest          = 0;
            junc_count    = 0;
            returning     = false;
            g_destination = 0;
            g_delivery_id = "";
            state         = S_WAIT_CMD;
            printf("다음 배달 명령 대기 중...\n");
            break;
        }

        // CAN 공통 송신 (회전/직진 상태는 위에서 직접 처리)
        if ((t - last_tx) >= 50) {
            float tx_rpm = (cmd == DIR_STOP) ? 0.0f : g_rpm;
            send_cmd(sock, cmd, tx_rpm);
            last_tx = t;
        }

        // ── 시각화 ──
        const char* stname =
            (state == S_WAIT_CMD)      ? "WAIT"     :
            (state == S_FOLLOW)        ? "FOLLOW"   :
            (state == S_JUNC_STOP)     ? "JUNC_STP" :
            (state == S_JUNC_STRAIGHT) ? "JUNC_ST"  :
            (state == S_JUNC_LEFT)     ? "JUNC_L"   :
            (state == S_JUNC_RIGHT)    ? "JUNC_R"   :
            (state == S_REACQUIRE)     ? "REACQ"    :
            (state == S_DELIVER_WAIT)  ? "DELIVER"  :
            (state == S_UTURN)         ? "UTURN"    : "FINISH";

        const char* dn = (dest==1)?"A":(dest==2)?"B":(dest==3)?"C":(dest==4)?"D":"?";

        cv::putText(vis, stname, cv::Point(10, 20),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0,255,255), 2);
        cv::putText(vis, std::string("DEST:")+dn+(returning?" RET":" GO"),
                    cv::Point(10, 45),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0,255,0), 2);

        char info[48];
        snprintf(info, sizeof(info), "J:%d TURN:%dms TH:%.2f",
                 junc_count, DUR_TURN_MS, HIGH_TH);
        cv::putText(vis, info, cv::Point(10, 68),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,200,0), 1);

        cv::imshow("Delivery", vis);
        if ((cv::waitKey(1) & 0xFF) == 'q') { send_estop(sock); break; }

        usleep(5000);
    }

exit_loop:
    g_running = false;
    send_estop(sock);
    cv::destroyAllWindows();
    term_raw_off();
    close(sock);
    cap.release();
    printf("\n프로그램 종료\n");
    return 0;
}
