/*
 * vision_can_delivery_final.cpp
 * 
 * 배달 차량 자율주행 최종 버전
 * 
 * [GO]    출발 → 십자(직진) → T자(좌회전) → ArUco ID=1(정지+10초) → 제자리 U턴
 * [RETURN] T자(우회전) → 십자(직진) → ArUco ID=0(정지) → 완료
 *
 * 교차로 감지: 수평 슬라이스 선 개수 카운팅 (조명 영향 최소화)
 * U턴: DIR_UTURN → STM32에서 좌전진 + 우후진 처리
 *
 * 컴파일:
 *   g++ vision_can_delivery_final.cpp -o delivery_final \
 *       $(pkg-config --cflags --libs opencv4)
 *
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
// STM32와 반드시 일치해야 함
#define DIR_STOP      0
#define DIR_FORWARD   1
#define DIR_BACKWARD  2
#define DIR_LEFT      3   // 좌회전 (양쪽 전진, 좌측 느리게)
#define DIR_RIGHT     4   // 우회전 (양쪽 전진, 우측 느리게)
#define DIR_UTURN     5   // 제자리 U턴 (좌전진 + 우후진)

// ===================== 주행 파라미터 =====================
#define DEFAULT_RPM       80.0f   // 기본 속도
#define STRAIGHT_DEADBAND 50      // ±50px 이내면 STRAIGHT (조정 가능)

// ===================== 교차로 감지 파라미터 =====================
#define JUNC_STREAK       6       // 연속 N프레임 감지해야 교차로 인정
#define JUNC_COOLDOWN_MS  4000    // 교차로 감지 후 무시 시간(ms)

// ===================== 교차로 순서별 행동 =====================
// GO:     1번째=직진(십자), 2번째=좌회전(T자)
// RETURN: 3번째=우회전(T자), 4번째=직진(십자)
static const uint8_t JUNC_ACTIONS[] = {
    DIR_FORWARD,   // 1번째 교차로: 직진  (십자)
    DIR_LEFT,      // 2번째 교차로: 좌회전 (T자)
    DIR_RIGHT,     // 3번째 교차로: 우회전 (T자 복귀)
    DIR_FORWARD,   // 4번째 교차로: 직진  (십자 복귀)
};

// ===================== 상태 지속 시간 =====================
#define DUR_STRAIGHT_MS   500     // 십자 직진 시간
#define DUR_TURN_MS       750     // T자 회전 시간 (실차에서 조정)
#define DUR_UTURN_MS      1300    // U턴 시간 (실차에서 조정)
#define DUR_REACQ_MS      400     // 교차로 후 라인 재탐색 시간
#define DELIVER_WAIT_MS   10000   // 배달 대기 시간(10초)

// ===================== 상태머신 =====================
typedef enum {
    S_FOLLOW,           // 라인 추적
    S_JUNC_STRAIGHT,    // 교차로 직진
    S_JUNC_LEFT,        // 교차로 좌회전
    S_JUNC_RIGHT,       // 교차로 우회전
    S_REACQUIRE,        // 교차로 후 라인 재탐색
    S_DELIVER_WAIT,     // 배달 대기(10초)
    S_UTURN,            // 제자리 U턴
    S_FINISHED          // 원점 복귀 완료
} State;

typedef enum { J_NONE, J_PLUS, J_T } JuncType;

// ===================== 전역 변수 =====================
static struct termios orig_termios;
static float g_rpm = DEFAULT_RPM;

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
    // 논블로킹
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

    // 가장 큰 마커 반환
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

    // ROI: 화면 하단 35%
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
        return DIR_FORWARD; // 라인 못 찾으면 직진 유지
    }

    cv::Moments m = cv::moments(contours[best]);
    int cx = (m.m00 != 0) ? (int)(m.m10 / m.m00) : W / 2;
    err_out = cx - W / 2;

    // 시각화
    cv::circle(vis, cv::Point(cx, y0 + roi.rows / 2), 8, cv::Scalar(0, 0, 255), -1);
    cv::rectangle(vis, cv::Point(0, y0), cv::Point(W, H), cv::Scalar(0, 255, 0), 2);
    cv::line(vis, cv::Point(W/2, y0), cv::Point(W/2, H), cv::Scalar(255, 0, 0), 2);

    if (abs(err_out) <= STRAIGHT_DEADBAND) return DIR_FORWARD;
    return (err_out < 0) ? DIR_LEFT : DIR_RIGHT;
}

// ===================== 교차로 감지 (컨투어 면적) =====================
/*
 * 라인 ROI에서 컨투어 면적이 갑자기 커지면 교차로
 * 일반 직선: 면적 작음 (테이프 하나)
 * 교차로:    면적 확 커짐 (테이프가 합쳐지면서)
 *
 * JUNC_AREA_MIN: 교차로 판단 최소 면적 → 실차 테스트 후 조정
 */
#define JUNC_AREA_MIN  8000.0   // 교차로 판단 면적 임계값 (실차에서 조정)

static JuncType detect_junction_area(const cv::Mat& frame, cv::Mat& vis,
                                      double& dbg_area)
{
    int H = frame.rows, W = frame.cols;

    // 교차로 ROI: 라인 ROI 바로 위쪽
    int yj0 = (int)(H * 0.40), yj1 = (int)(H * 0.65);
    cv::Mat roi = frame(cv::Rect(0, yj0, W, yj1 - yj0));

    cv::Mat gray, blur, bin;
    cv::cvtColor(roi, gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray, blur, cv::Size(7, 7), 0);
    cv::threshold(blur, bin, 0, 255, cv::THRESH_BINARY_INV | cv::THRESH_OTSU);
    cv::morphologyEx(bin, bin, cv::MORPH_OPEN,
        cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(bin, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    dbg_area = 0;
    for (auto& c : contours) {
        double a = cv::contourArea(c);
        if (a > dbg_area) dbg_area = a;
    }

    // ROI 시각화
    cv::rectangle(vis, cv::Point(0, yj0), cv::Point(W, yj1),
                  cv::Scalar(255, 0, 255), 2);

    if (dbg_area >= JUNC_AREA_MIN) return J_T; // 일단 교차로 감지 (순서로 판단)
    return J_NONE;
}

// ===================== 메인 =====================
int main(void) {
    // CAN 소켓 초기화
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

    // 카메라 초기화
    cv::VideoCapture cap("/dev/video0");
    if (!cap.isOpened()) {
        cap.open(0);
        if (!cap.isOpened()) { fprintf(stderr, "카메라 오픈 실패\n"); return 1; }
    }
    cap.set(cv::CAP_PROP_FRAME_WIDTH,  640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

    printf("=== 배달 차량 자율주행 최종 ===\n");
    printf("[GO]     십자(직진) -> T자(좌회전) -> ID=1(정지10초) -> U턴\n");
    printf("[RETURN] T자(우회전) -> 십자(직진) -> ID=0(정지) -> 완료\n");
    printf("키: q=종료  e=E-Stop  +/-=RPM 조절\n");
    printf("------------------------------------------\n");

    term_raw_on();

    bool    returning  = false;
    bool    delivered  = false;
    int     junc_count = 0;
    State   state      = S_FOLLOW;
    uint64_t state_ts  = now_ms();
    uint64_t last_junc = 0;
    int     streak     = 0;
    uint64_t last_tx   = 0;

    while (1) {
        cv::Mat frame;
        cap >> frame;
        if (frame.empty()) continue;

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

            // 목적지 도착 (ID=1)
            if (!delivered && !returning && aid == 1) {
                delivered = true;
                state = S_DELIVER_WAIT;
                state_ts = t;
                cmd = DIR_STOP;
                printf("\n[EVENT] ArUco ID=1 감지 → 정지 & 10초 대기\n");
                goto tx;
            }

            // 원점 복귀 완료 (ID=0)
            if (returning && aid == 0) {
                state = S_FINISHED;
                state_ts = t;
                cmd = DIR_STOP;
                printf("\n[EVENT] ArUco ID=0 감지 → 원점 복귀 완료!\n");
                goto tx;
            }

            // 2) 교차로 감지 (컨투어 면적)
            {
                double area = 0;
                JuncType jt = detect_junction_area(frame, vis, area);

                bool cool = (t - last_junc) > JUNC_COOLDOWN_MS;
                if (cool && jt != J_NONE) streak++;
                else if (!cool)           streak = 0;
                else                      streak = std::max(0, streak - 1);

                // 디버그 출력
                if ((t - last_tx) >= 50) {
                    printf("\r[Area=%.0f streak=%d/%d junc=%d] ",
                           area, streak, JUNC_STREAK, junc_count);
                    fflush(stdout);
                }

                // streak 진행도 표시
                char streak_str[48];
                snprintf(streak_str, sizeof(streak_str), "A=%.0f S=%d/%d J=%d",
                         area, streak, JUNC_STREAK, junc_count);
                cv::putText(vis, streak_str, cv::Point(10, 90),
                            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,255,0), 1);

                if (cool && streak >= JUNC_STREAK && junc_count < 4) {
                    streak    = 0;
                    last_junc = t;

                    uint8_t action = JUNC_ACTIONS[junc_count];
                    junc_count++;

                    const char* an = (action == DIR_FORWARD) ? "직진" :
                                     (action == DIR_LEFT)    ? "좌회전" : "우회전";
                    printf("\n[교차로 %d번째] → %s (area=%.0f)\n",
                           junc_count, an, area);

                    if      (action == DIR_FORWARD) state = S_JUNC_STRAIGHT;
                    else if (action == DIR_LEFT)    state = S_JUNC_LEFT;
                    else                            state = S_JUNC_RIGHT;
                    state_ts = t;
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
                    printf("\n[Line err=%4dpx] %-8s RPM=%.1f", err, d, g_rpm);
                    fflush(stdout);
                }
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
            // 라인 재탐색: 일정 시간 후 FOLLOW로 복귀
            int err = 0;
            cmd = follow_line(frame, err, vis);
            if (dt >= DUR_REACQ_MS) {
                state = S_FOLLOW; state_ts = t;
            }
        }

        else if (state == S_DELIVER_WAIT) {
            cmd = DIR_STOP;
            // dt 오버플로우 방지: state_ts가 미래면 0으로 처리
            uint64_t safe_dt = (t >= state_ts) ? (t - state_ts) : 0;
            int remain = (safe_dt < DELIVER_WAIT_MS)
                         ? (int)((DELIVER_WAIT_MS - safe_dt) / 1000) + 1
                         : 0;
            printf("\r[배달 대기] 남은시간: %d초   ", remain);
            fflush(stdout);
            if (safe_dt >= DELIVER_WAIT_MS) {
                state = S_UTURN; state_ts = t;
                printf("\n[STATE] U턴 시작\n");
            }
        }

        else if (state == S_UTURN) {
            // 제자리 U턴: STM32에서 DIR_UTURN(5) 수신 시
            // 왼쪽 바퀴 전진 + 오른쪽 바퀴 후진 → 제자리 180도 회전
            cmd = DIR_UTURN;
            if (dt >= DUR_UTURN_MS) {
                returning = true;
                state = S_REACQUIRE; state_ts = t;
                printf("\n[MODE] 복귀 모드 시작\n");
            }
        }

        else if (state == S_FINISHED) {
            send_cmd(sock, DIR_STOP, 0);
            printf("\n\n=============================\n");
            printf("      배달 완료! 원점 복귀\n");
            printf("=============================\n");
            break;
        }

    tx:
        // 50ms마다 CAN 전송
        if ((t - last_tx) >= 50) {
            float tx_rpm = (cmd == DIR_STOP) ? 0.0f : g_rpm;
            send_cmd(sock, cmd, tx_rpm);
            last_tx = t;
        }

        // ── 시각화 ──
        {
            int H = vis.rows, W = vis.cols;

            // 상태 표시
            const char* stname =
                (state == S_FOLLOW)        ? "FOLLOW"   :
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

            // RPM 표시
            char rpm_str[32];
            snprintf(rpm_str, sizeof(rpm_str), "RPM:%.0f", g_rpm);
            cv::putText(vis, rpm_str,
                cv::Point(W - 110, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                cv::Scalar(255, 255, 0), 2);

            cv::imshow("Delivery CAM", vis);
            int key = cv::waitKey(1) & 0xFF;
            if (key == 'q') { send_estop(sock); break; }
        }

        usleep(5000); // 5ms 대기
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

