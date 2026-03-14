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
#include <deque>

#define DIR_STOP     0
#define DIR_FORWARD  1
#define DIR_BACKWARD 2
#define DIR_LEFT     3
#define DIR_RIGHT    4
#define DEFAULT_RPM  80.0f

#define PLUS_SIDE_TH   0.45f
#define T_SIDE_TH      0.40f
#define T_MID_TH       0.10f
#define JUNC_STREAK    6
#define JUNC_COOLDOWN_MS 4000

#define DUR_STRAIGHT_MS   400
#define DUR_TURN_MS       700
#define DUR_REACQ_MS      500
#define DUR_UTURN_MS     1200
#define DELIVER_WAIT_MS 10000

typedef enum { S_FOLLOW, S_JUNC_STRAIGHT, S_JUNC_LEFT, S_JUNC_RIGHT,
               S_REACQUIRE, S_DELIVER_WAIT, S_UTURN, S_FINISHED } State;
typedef enum { J_NONE, J_PLUS, J_T } JuncType;

static struct termios orig_termios;
static float g_rpm = DEFAULT_RPM;

static uint64_t now_ms() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000ULL + ts.tv_nsec / 1000000ULL;
}
void term_raw_on(void) {
    struct termios raw;
    tcgetattr(STDIN_FILENO, &orig_termios);
    raw = orig_termios;
    raw.c_lflag &= ~(ICANON | ECHO);
    raw.c_cc[VMIN] = 1; raw.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSAFLUSH, &raw);
}
void term_raw_off(void) { tcsetattr(STDIN_FILENO, TCSAFLUSH, &orig_termios); }

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
    if (ret < 0 && errno != EAGAIN && errno != EWOULDBLOCK) perror("write");
}
void send_estop(int sock) {
    struct can_frame frame;
    memset(&frame, 0, sizeof(frame));
    frame.can_id = 0x011; frame.can_dlc = 1; frame.data[0] = 0xFF;
    write(sock, &frame, sizeof(frame));
}
void recv_feedback(int sock) {
    struct can_frame frame;
    struct timeval tv = {0,0};
    fd_set fds; FD_ZERO(&fds); FD_SET(sock, &fds);
    if (select(sock+1, &fds, NULL, NULL, &tv) > 0) {
        if (read(sock, &frame, sizeof(frame)) > 0) {
            if ((frame.can_id & CAN_SFF_MASK) == 0x100 && frame.can_dlc >= 8) {
                int16_t  l100 = (int16_t)((frame.data[0]<<8)|frame.data[1]);
                int16_t  r100 = (int16_t)((frame.data[2]<<8)|frame.data[3]);
                uint16_t pwmL = (uint16_t)((frame.data[4]<<8)|frame.data[5]);
                uint16_t pwmR = (uint16_t)((frame.data[6]<<8)|frame.data[7]);
                printf("\r[speed] L:%6.1f RPM PWM:%3u | R:%6.1f RPM PWM:%3u    ",
                       l100/100.0f, pwmL, r100/100.0f, pwmR);
                fflush(stdout);
            }
        }
    }
}

static float white_ratio(const cv::Mat& bin) {
    return (float)cv::countNonZero(bin) / (float)bin.total();
}
static JuncType classify_junction(const cv::Mat& roi, float& rL, float& rM, float& rR) {
    int w = roi.cols, h = roi.rows;
    rL = white_ratio(roi(cv::Rect(0,    0, w/3,       h)));
    rM = white_ratio(roi(cv::Rect(w/3,  0, w/3,       h)));
    rR = white_ratio(roi(cv::Rect(2*w/3,0, w-2*(w/3), h)));
    if (rL >= PLUS_SIDE_TH && rM >= PLUS_SIDE_TH && rR >= PLUS_SIDE_TH) return J_PLUS;
    if (rL >= T_SIDE_TH && rR >= T_SIDE_TH && rM <= T_MID_TH) return J_T;
    return J_NONE;
}
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
static uint8_t follow_line(const cv::Mat& frame, int& err_out) {
    int H = frame.rows, W = frame.cols;
    cv::Mat roi = frame(cv::Rect(0, (int)(H*0.65), W, H-(int)(H*0.65)));
    cv::Mat gray, blur, bin;
    cv::cvtColor(roi, gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray, blur, cv::Size(5,5), 0);
    cv::threshold(blur, bin, 0, 255, cv::THRESH_BINARY_INV | cv::THRESH_OTSU);
    cv::morphologyEx(bin, bin, cv::MORPH_OPEN,
        cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3)));
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(bin, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    int best = -1; double best_area = 0;
    for (int i = 0; i < (int)contours.size(); i++) {
        double a = cv::contourArea(contours[i]);
        if (a > best_area) { best_area = a; best = i; }
    }
    if (best < 0 || best_area < 500) { err_out = 0; return DIR_STOP; }
    cv::Moments m = cv::moments(contours[best]);
    int cx = (m.m00 != 0) ? (int)(m.m10/m.m00) : W/2;
    err_out = cx - W/2;
    if (abs(err_out) <= 40) return DIR_FORWARD;
    return (err_out < 0) ? DIR_LEFT : DIR_RIGHT;
}

int main(void) {
    int sock;
    struct sockaddr_can addr;
    struct ifreq ifr;
    sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock < 0) { perror("socket"); return 1; }
    strcpy(ifr.ifr_name, "can0");
    if (ioctl(sock, SIOCGIFINDEX, &ifr) < 0) { perror("ioctl"); return 1; }
    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(sock, (struct sockaddr*)&addr, sizeof(addr)) < 0) { perror("bind"); return 1; }

    cv::VideoCapture cap(0);
    if (!cap.isOpened()) { fprintf(stderr, "카메라 오픈 실패\n"); return 1; }
    cap.set(cv::CAP_PROP_FRAME_WIDTH,  640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

    printf("=== 배달 차량 자율주행 ===\n");
    printf("GO:     십자(직진) -> T자(좌회전) -> ID=1(10초대기) -> U턴\n");
    printf("RETURN: T자(우회전) -> 십자(직진) -> 원점\n");
    printf("'q'=종료  'e'=E-Stop  '+'/'-'=RPM\n");
    printf("------------------------------------------\n");

    term_raw_on();

    std::deque<uint8_t> go_plan     = { DIR_FORWARD, DIR_LEFT   };
    std::deque<uint8_t> return_plan = { DIR_RIGHT,   DIR_FORWARD };

    bool returning = false, delivered = false;
    State    state    = S_FOLLOW;
    uint64_t state_ts = now_ms();
    uint64_t last_junc = 0;
    int      streak    = 0;
    uint64_t last_tx   = 0;

    while (1) {
        recv_feedback(sock);
        cv::Mat frame;
        cap >> frame;
        if (frame.empty()) continue;

        uint64_t t  = now_ms();
        uint64_t dt = t - state_ts;
        uint8_t  cmd = DIR_FORWARD;

        // 키 입력
        {
            fd_set fds; struct timeval tv = {0,0};
            FD_ZERO(&fds); FD_SET(STDIN_FILENO, &fds);
            if (select(STDIN_FILENO+1, &fds, NULL, NULL, &tv) > 0) {
                char key = getchar();
                if (key=='q') { send_cmd(sock,DIR_STOP,0); printf("\n종료\n"); goto exit_loop; }
                if (key=='e') { send_estop(sock); printf("\n[E-Stop]\n"); }
                if (key=='+' && g_rpm<120.0f) { g_rpm+=10.0f; printf("\n[RPM] %.1f\n",g_rpm); }
                if (key=='-' && g_rpm>10.0f)  { g_rpm-=10.0f; printf("\n[RPM] %.1f\n",g_rpm); }
            }
        }

        if (state == S_FOLLOW) {
            // ArUco 목적지 ID=1
            if (!delivered && !returning) {
                int aid = detect_aruco(frame);
                if (aid == 1) {
                    delivered=true; state=S_DELIVER_WAIT; state_ts=t;
                    send_cmd(sock, DIR_STOP, 0);
                    printf("\n[EVENT] ArUco ID=1 -> 10초 대기\n");
                    goto tx;
                }
            }
            // 원점 복귀 ID=0
            if (returning) {
                int aid = detect_aruco(frame);
                if (aid == 0) {
                    state=S_FINISHED; state_ts=t;
                    send_cmd(sock, DIR_STOP, 0);
                    printf("\n[EVENT] 원점 복귀 완료!\n");
                    goto tx;
                }
            }
            // 교차로 감지
            {
                int H=frame.rows, W=frame.cols;
                int yj0=(int)(H*0.55), yj1=(int)(H*0.70);
                cv::Mat roi=frame(cv::Rect(0,yj0,W,yj1-yj0));
                cv::Mat gray,blur,bin;
                cv::cvtColor(roi,gray,cv::COLOR_BGR2GRAY);
                cv::GaussianBlur(gray,blur,cv::Size(5,5),0);
                cv::threshold(blur,bin,0,255,cv::THRESH_BINARY_INV|cv::THRESH_OTSU);
                float rL,rM,rR;
                JuncType jt = classify_junction(bin,rL,rM,rR);
                bool cool = (t-last_junc)>JUNC_COOLDOWN_MS;
                if (cool && jt!=J_NONE) streak++;
                else if (!cool) streak=0;
                else streak=std::max(0,streak-1);
                if (cool && streak>=JUNC_STREAK) {
                    streak=0; last_junc=t;
                    auto& plan = returning ? return_plan : go_plan;
                    uint8_t action = DIR_FORWARD;
                    if (!plan.empty()) { action=plan.front(); plan.pop_front(); }
                    const char* jn=(jt==J_PLUS)?"십자":"T자";
                    const char* an=(action==DIR_FORWARD)?"직진":(action==DIR_LEFT)?"좌회전":"우회전";
                    printf("\n[교차로] %s -> %s (L=%.2f M=%.2f R=%.2f)\n",jn,an,rL,rM,rR);
                    if      (action==DIR_FORWARD) state=S_JUNC_STRAIGHT;
                    else if (action==DIR_LEFT)    state=S_JUNC_LEFT;
                    else                          state=S_JUNC_RIGHT;
                    state_ts=t; goto tx;
                }
            }
            // 라인 추적
            {
                int err=0;
                cmd = follow_line(frame, err);
                if ((t-last_tx)>=100) {
                    const char* d=(cmd==DIR_FORWARD)?"STRAIGHT":(cmd==DIR_LEFT)?"LEFT":"RIGHT";
                    printf("\n[Line err=%dpx] %s RPM=%.1f", err, d, g_rpm);
                }
            }
        }
        else if (state==S_JUNC_STRAIGHT) {
            cmd=DIR_FORWARD;
            if (dt>=DUR_STRAIGHT_MS) { state=S_REACQUIRE; state_ts=t; printf("\n[STATE] REACQUIRE\n"); }
        }
        else if (state==S_JUNC_LEFT) {
            cmd=DIR_LEFT;
            if (dt>=DUR_TURN_MS) { state=S_REACQUIRE; state_ts=t; printf("\n[STATE] REACQUIRE\n"); }
        }
        else if (state==S_JUNC_RIGHT) {
            cmd=DIR_RIGHT;
            if (dt>=DUR_TURN_MS) { state=S_REACQUIRE; state_ts=t; printf("\n[STATE] REACQUIRE\n"); }
        }
        else if (state==S_REACQUIRE) {
            int err=0; cmd=follow_line(frame,err);
            if (dt>=DUR_REACQ_MS) { state=S_FOLLOW; state_ts=t; }
        }
        else if (state==S_DELIVER_WAIT) {
            cmd=DIR_STOP;
            printf("\r[배달 대기] 남은시간: %d초   ", (int)((DELIVER_WAIT_MS-dt)/1000)+1);
            fflush(stdout);
            if (dt>=DELIVER_WAIT_MS) { state=S_UTURN; state_ts=t; printf("\n[STATE] U턴\n"); }
        }
        else if (state==S_UTURN) {
            cmd=DIR_LEFT;
            if (dt>=DUR_UTURN_MS) {
                returning=true; state=S_REACQUIRE; state_ts=t;
                printf("\n[MODE] 복귀 모드\n");
            }
        }
        else if (state==S_FINISHED) {
            send_cmd(sock, DIR_STOP, 0);
            printf("\n=== 배달 완료! ===\n");
            break;
        }

        tx:
        if ((t-last_tx)>=100) {
            send_cmd(sock, cmd, (cmd==DIR_STOP)?0.0f:g_rpm);
            last_tx=t;
        }
        usleep(5000);
    }

exit_loop:
    send_estop(sock);
    term_raw_off();
    close(sock);
    cap.release();
    return 0;
}

