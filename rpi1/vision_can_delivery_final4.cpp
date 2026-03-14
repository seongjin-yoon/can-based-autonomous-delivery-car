/*
 * vision_can_delivery_final.cpp
 *
 * 배달 차량 자율주행 최종 버전
 *
 * [GO]     십자(직진) → T자(좌회전) → ArUco ID=1(정지+10초) → U턴
 * [RETURN] T자(우회전) → 십자(직진) → ArUco ID=0(정지) → 완료
 *
 * 교차로 감지: L/M/R 흰색 비율 방식
 * 교차로 행동: 카운터 순서 기반 (십자/T자 구분 없음)
 *   1번째 → 직진
 *   2번째 → 좌회전
 *   3번째 → 우회전
 *   4번째 → 직진
 *
 * 컴파일:
 *   g++ vision_can_delivery_final.cpp -o delivery_final \
 *       $(pkg-config --cflags --libs opencv4)
 * 실행:
 *   sudo ip link set can0 up type can bitrate 250000
 *   DISPLAY=:0 ./delivery_final
 */

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
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

// ===================== CAN 명령 코드 =====================
#define DIR_STOP      0
#define DIR_FORWARD   1
#define DIR_BACKWARD  2
#define DIR_LEFT      3
#define DIR_RIGHT     4
#define DIR_UTURN     5

// ===================== 주행 파라미터 =====================
#define DEFAULT_RPM       80.0f
#define STRAIGHT_DEADBAND 50

// ===================== 교차로 감지 파라미터 =====================
#define JUNC_STREAK      10       // 연속 N프레임 감지해야 교차로 인정
#define JUNC_COOLDOWN_MS 4000    // 교차로 감지 후 무시 시간(ms)
#define LINE_TH          0.06f   // 라인 존재 판단 임계값
#define HIGH_TH          0.25f   // 확실한 라인 임계값

// ===================== 교차로 순서별 행동 =====================
static const uint8_t JUNC_ACTIONS[4] = {
    DIR_FORWARD,   // 1번째: 직진  (십자)
    DIR_LEFT,      // 2번째: 좌회전 (T자)
    DIR_RIGHT,     // 3번째: 우회전 (T자 복귀)
    DIR_FORWARD,   // 4번째: 직진  (십자 복귀)
};

// ===================== 상태 지속 시간 =====================
#define DUR_JUNC_STOP_MS 1000    // 교차로 정지 시간 (1초)
#define DUR_STRAIGHT_MS  500
#define DUR_TURN_MS      750
#define DUR_UTURN_MS     1300
#define DUR_REACQ_MS     400
#define DELIVER_WAIT_MS  10000

// ===================== 상태머신 =====================
typedef enum {
    S_FOLLOW,
    S_JUNC_STOP,        // 교차로 감지 후 1초 정지
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
static float g_rpm = DEFAULT_RPM;
static uint8_t g_next_action = DIR_FORWARD; // 교차로 정지 후 실행할 행동

// ===================== 유틸 함수 =====================
static uint64_t now_ms() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000ULL + ts.tv_nsec / 1000000ULL;
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

// ===================== CAN 송수신 =====================
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

    if (best < 0 || best_area < 500) {
        err_out = 0;
        return DIR_FORWARD;
    }

    cv::Moments m = cv::moments(contours[best]);
    int cx = (m.m00 != 0) ? (int)(m.m10 / m.m00) : W / 2;
    err_out = cx - W / 2;

    cv::circle(vis, cv::Point(cx, y0 + roi.rows / 2), 8, cv::Scalar(0, 0, 255), -1);
    cv::rectangle(vis, cv::Point(0, y0), cv::Point(W, H), cv::Scalar(0, 255, 0), 2);
    cv::line(vis, cv::Point(W/2, y0), cv::Point(W/2, H), cv::Scalar(255, 0, 0), 2);

    if (abs(err_out) <= STRAIGHT_DEADBAND) return DIR_FORWARD;
    return (err_out < 0) ? DIR_LEFT : DIR_RIGHT;
}

// ===================== 교차로 감지 (L/M/R 흰색 비율) =====================
static JuncType classify_junction(const cv::Mat& bin_roi,
                                   float& dbg_L, float& dbg_M, float& dbg_R)
{
    int w = bin_roi.cols, h = bin_roi.rows;
    int lw = w / 3, mw = w / 3, rw = w - 2*(w/3);

    dbg_L = (float)cv::countNonZero(bin_roi(cv::Rect(0,     0, lw, h))) / (float)(lw * h);
    dbg_M = (float)cv::countNonZero(bin_roi(cv::Rect(lw,    0, mw, h))) / (float)(mw * h);
    dbg_R = (float)cv::countNonZero(bin_roi(cv::Rect(lw+mw, 0, rw, h))) / (float)(rw * h);

    bool hasL = (dbg_L >= LINE_TH);
    bool hasM = (dbg_M >= LINE_TH);
    bool hasR = (dbg_R >= LINE_TH);

    // 십자: 좌우 모두 HIGH_TH 이상
    if (hasL && hasM && hasR && dbg_L >= HIGH_TH && dbg_R >= HIGH_TH)
        return J_PLUS;

    // T자: 한쪽 측면만 HIGH_TH 이상
    if (hasL && hasR && !hasM && (dbg_L >= HIGH_TH || dbg_R >= HIGH_TH))
        return J_T;
    if (hasL && hasM && !hasR && dbg_L >= HIGH_TH)
        return J_T;
    if (hasM && hasR && !hasL && dbg_R >= HIGH_TH)
        return J_T;

    return J_NONE;
}

// ===================== 메인 =====================
int main(void) {
    int sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock < 0) { perror("socket"); return 1; }

    struct ifreq ifr;
    strcpy(ifr.ifr_name, "can0");
    if (ioctl(sock, SIOCGIFINDEX, &ifr) < 0) { perror("ioctl"); return 1; }

    struct sockaddr_can addr;
    memset(&addr, 0, sizeof(addr));
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(sock, (struct sockaddr*)&addr, sizeof(addr)) < 0) { perror("bind"); return 1; }

    cv::VideoCapture cap("/dev/video0");
    if (!cap.isOpened()) {
        cap.open(0);
        if (!cap.isOpened()) { fprintf(stderr, "카메라 오픈 실패\n"); return 1; }
    }
    cap.set(cv::CAP_PROP_FRAME_WIDTH,  320);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 240);
    cap.set(cv::CAP_PROP_FPS, 30);
    cap.set(cv::CAP_PROP_BUFFERSIZE, 1);  // 버퍼 1프레임으로 최소화

    printf("=== 배달 차량 자율주행 최종 ===\n");
    printf("[GO]     십자(직진) -> T자(좌회전) -> ID=1(정지10초) -> U턴\n");
    printf("[RETURN] T자(우회전) -> 십자(직진) -> ID=0(정지) -> 완료\n");
    printf("키: q=종료  e=E-Stop  +/-=RPM 조절\n");
    printf("------------------------------------------\n");

    term_raw_on();

    bool     delivered  = false;
    bool     returning  = false;
    int      junc_count = 0;
    State    state      = S_FOLLOW;
    uint64_t state_ts   = now_ms();
    uint64_t last_junc  = 0;
    int      streak     = 0;
    uint64_t last_tx    = 0;

    while (1) {
        cv::Mat frame;
        for(int i=0; i< 2; i++) cap.grab();
	if (!cap.retrieve(frame) || frame.empty()) continue;

        uint64_t t  = now_ms();
        uint64_t dt = t - state_ts;
        uint8_t  cmd = DIR_FORWARD;
        cv::Mat  vis = frame.clone();

        // ── 키 입력 처리 ──
        {
            fd_set fds; struct timeval tv = {0, 0};
            FD_ZERO(&fds); FD_SET(STDIN_FILENO, &fds);
            if (select(STDIN_FILENO+1, &fds, NULL, NULL, &tv) > 0) {
                char key = getchar();
                if (key == 'q') {
                    send_cmd(sock, DIR_STOP, 0);
                    printf("\n종료\n");
                    goto exit_loop;
                }
                if (key == 'e') { send_estop(sock); printf("\n[E-Stop]\n"); }
                if (key == '+' && g_rpm < 120.0f) { g_rpm += 10.0f; printf("\n[RPM+] %.1f\n", g_rpm); }
                if (key == '-' && g_rpm > 10.0f)  { g_rpm -= 10.0f; printf("\n[RPM-] %.1f\n", g_rpm); }
            }
        }

        // ── 상태머신 ──
        if (state == S_FOLLOW) {

            // 1) ArUco 마커 감지
            int aid = detect_aruco(frame);

            if (!delivered && !returning && aid == 1) {
                delivered = true;
                state = S_DELIVER_WAIT; state_ts = t;
                cmd = DIR_STOP;
                printf("\n[EVENT] ArUco ID=1 감지 → 정지 & 10초 대기\n");
                goto tx;
            }
            if (returning && aid == 0) {
                state = S_FINISHED; state_ts = t;
                cmd = DIR_STOP;
                printf("\n[EVENT] ArUco ID=0 감지 → 원점 복귀 완료!\n");
                goto tx;
            }

            // 2) 교차로 감지 (junc_count < 4 일 때만)
            if (junc_count < 4) {
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

                // ROI 시각화
                cv::rectangle(vis, cv::Point(0, yj0), cv::Point(W, yj1),
                              cv::Scalar(255, 0, 255), 2);
                cv::line(vis, cv::Point(W/3,   yj0), cv::Point(W/3,   yj1), cv::Scalar(255,255,0), 1);
                cv::line(vis, cv::Point(2*W/3, yj0), cv::Point(2*W/3, yj1), cv::Scalar(255,255,0), 1);

                char dbg_str[64];
                snprintf(dbg_str, sizeof(dbg_str), "S=%d/%d J=%d L=%.2f M=%.2f R=%.2f",
                         streak, JUNC_STREAK, junc_count, rL, rM, rR);
                cv::putText(vis, dbg_str, cv::Point(5, 90),
                            cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(255,255,0), 1);

                if (cool && streak >= JUNC_STREAK) {
                    streak    = 0;
                    last_junc = t;

                    uint8_t action = JUNC_ACTIONS[junc_count];
                    junc_count++;

                    const char* an = (action == DIR_FORWARD) ? "직진" :
                                     (action == DIR_LEFT)    ? "좌회전" : "우회전";
                    printf("\n[교차로 %d번째] 정지 후 → %s (L=%.2f M=%.2f R=%.2f)\n",
                           junc_count, an, rL, rM, rR);

                    // 다음 행동 저장 후 먼저 1초 정지
                    state         = S_JUNC_STOP;
                    g_next_action = action;
                    state_ts      = t;
                    cmd           = DIR_STOP;
                    goto tx;
                }
            }

            // 3) 라인 추적
            {
                int err = 0;
                cmd = follow_line(frame, err, vis);
                if ((t - last_tx) >= 50) {
                    const char* d = (cmd == DIR_FORWARD) ? "STRAIGHT" :
                                    (cmd == DIR_LEFT)    ? "LEFT" : "RIGHT";
                    printf("\r[Line err=%4dpx] %-8s RPM=%.1f", err, d, g_rpm);
                    fflush(stdout);
                }
            }
        }

        else if (state == S_JUNC_STOP) {
            // 교차로 감지 후 1초 정지
            cmd = DIR_STOP;
            if (dt >= DUR_JUNC_STOP_MS) {
                // 1초 후 저장된 행동 실행
                if      (g_next_action == DIR_FORWARD) state = S_JUNC_STRAIGHT;
                else if (g_next_action == DIR_LEFT)    state = S_JUNC_LEFT;
                else                                   state = S_JUNC_RIGHT;
                state_ts = t;
            }
        }

        else if (state == S_JUNC_STRAIGHT) {
            cmd = DIR_FORWARD;
            if (dt >= DUR_STRAIGHT_MS) {
                state = S_REACQUIRE; state_ts = t;
                printf("\n[STATE] REACQUIRE\n");
            }
        }

        else if (state == S_JUNC_LEFT) {
            cmd = DIR_LEFT;
            if (dt >= DUR_TURN_MS) {
                state = S_REACQUIRE; state_ts = t;
                printf("\n[STATE] REACQUIRE\n");
            }
        }

        else if (state == S_JUNC_RIGHT) {
            cmd = DIR_RIGHT;
            if (dt >= DUR_TURN_MS) {
                state = S_REACQUIRE; state_ts = t;
                printf("\n[STATE] REACQUIRE\n");
            }
        }

        else if (state == S_REACQUIRE) {
            int err = 0;
            cmd = follow_line(frame, err, vis);
            if (dt >= DUR_REACQ_MS) {
                state = S_FOLLOW; state_ts = t;
            }
        }

        else if (state == S_DELIVER_WAIT) {
            cmd = DIR_STOP;
            uint64_t safe_dt = (t >= state_ts) ? (t - state_ts) : 0;
            int remain = (safe_dt < DELIVER_WAIT_MS)
                         ? (int)((DELIVER_WAIT_MS - safe_dt) / 1000) + 1 : 0;
            printf("\r[배달 대기] 남은시간: %d초   ", remain);
            fflush(stdout);
            if (safe_dt >= DELIVER_WAIT_MS) {
                state = S_UTURN; state_ts = t;
                printf("\n[STATE] U턴 시작\n");
            }
        }

        else if (state == S_UTURN) {
            cmd = DIR_UTURN;
            if (dt >= DUR_UTURN_MS) {
                returning = true;
                state = S_REACQUIRE; state_ts = t;
                printf("\n[MODE] 복귀 모드 시작\n");
            }
        }

        else if (state == S_FINISHED) {
            send_cmd(sock, DIR_STOP, 0);
            printf("\n=============================\n");
            printf("      배달 완료! 원점 복귀\n");
            printf("=============================\n");
            break;
        }

    tx:
        if ((t - last_tx) >= 50) {
            float tx_rpm = (cmd == DIR_STOP) ? 0.0f : g_rpm;
            send_cmd(sock, cmd, tx_rpm);
            last_tx = t;
        }

        // ── 시각화 ──
        {
            int W = vis.cols;
            const char* stname =
                (state == S_FOLLOW)        ? "FOLLOW"   :
                (state == S_JUNC_STOP)     ? "JUNC_STOP":
                (state == S_JUNC_STRAIGHT) ? "JUNC_ST"  :
                (state == S_JUNC_LEFT)     ? "JUNC_L"   :
                (state == S_JUNC_RIGHT)    ? "JUNC_R"   :
                (state == S_REACQUIRE)     ? "REACQ"    :
                (state == S_DELIVER_WAIT)  ? "DELIVER"  :
                (state == S_UTURN)         ? "UTURN"    : "FINISH";

            cv::putText(vis, stname,
                cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.9,
                cv::Scalar(0, 255, 255), 2);
            cv::putText(vis, returning ? "RETURN" : "GO",
                cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                cv::Scalar(0, 255, 0), 2);

            char rpm_str[32];
            snprintf(rpm_str, sizeof(rpm_str), "RPM:%.0f", g_rpm);
            cv::putText(vis, rpm_str,
                cv::Point(W - 110, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                cv::Scalar(255, 255, 0), 2);

            cv::imshow("Delivery CAM", vis);
            if ((cv::waitKey(1) & 0xFF) == 'q') { send_estop(sock); break; }
        }

        usleep(5000);
    }

exit_loop:
    send_estop(sock);
    cv::destroyAllWindows();
    term_raw_off();
    close(sock);
    cap.release();
    printf("\n프로그램 종료\n");
    return 0;
}

