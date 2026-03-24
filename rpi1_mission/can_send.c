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
#include <fcntl.h>
#include <errno.h>

#define DIR_STOP     0
#define DIR_FORWARD  1
#define DIR_BACKWARD 2
#define DIR_LEFT     3
#define DIR_RIGHT    4
#define DIR_UTURN    5   // ★ 유턴 추가
#define DEFAULT_RPM  150.0f

static struct termios orig_termios;
static float target_rpm = DEFAULT_RPM;
static uint8_t last_dir = DIR_STOP;
static float   last_rpm = 0.0f;

static uint64_t now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000ULL + (uint64_t)ts.tv_nsec / 1000000ULL;
}

void term_raw_on(void)
{
    struct termios raw;
    tcgetattr(STDIN_FILENO, &orig_termios);
    raw = orig_termios;
    raw.c_lflag &= ~(ICANON | ECHO);
    raw.c_cc[VMIN]  = 1;
    raw.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSAFLUSH, &raw);
}

void term_raw_off(void)
{
    tcsetattr(STDIN_FILENO, TCSAFLUSH, &orig_termios);
}

void send_cmd(int sock, uint8_t dir, float rpm)
{
    struct can_frame frame;
    memset(&frame, 0, sizeof(frame));
    frame.can_id  = 0x010;
    frame.can_dlc = 3;
    uint16_t rpm_x10 = (uint16_t)(rpm * 10.0f);
    frame.data[0] = dir;
    frame.data[1] = (rpm_x10 >> 8) & 0xFF;
    frame.data[2] =  rpm_x10       & 0xFF;

    if (write(sock, &frame, sizeof(frame)) < 0 &&
        errno != EAGAIN && errno != EWOULDBLOCK)
        perror("write");
}

void send_estop(int sock)
{
    struct can_frame frame;
    memset(&frame, 0, sizeof(frame));
    frame.can_id  = 0x011;
    frame.can_dlc = 1;
    frame.data[0] = 0xFF;
    write(sock, &frame, sizeof(frame));
}

void recv_feedback(int sock)
{
    struct can_frame frame;
    struct timeval tv = {0, 0};
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(sock, &fds);
    if (select(sock + 1, &fds, NULL, NULL, &tv) > 0) {
        if (read(sock, &frame, sizeof(frame)) > 0) {
            if ((frame.can_id & 0x7FF) == 0x100 && frame.can_dlc >= 8) {
                int16_t  l100 = (int16_t)((frame.data[0] << 8) | frame.data[1]);
                int16_t  r100 = (int16_t)((frame.data[2] << 8) | frame.data[3]);
                uint16_t pwmL = (uint16_t)((frame.data[4] << 8) | frame.data[5]);
                uint16_t pwmR = (uint16_t)((frame.data[6] << 8) | frame.data[7]);
                printf("\r[speed] L:%6.1f RPM PWM:%3u | R:%6.1f RPM PWM:%3u    ",
                       l100 / 100.0f, pwmL, r100 / 100.0f, pwmR);
                fflush(stdout);
            }
        }
    }
}

int main(void)
{
    int sock;
    struct sockaddr_can addr;
    struct ifreq ifr;

    sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock < 0) { perror("socket"); return 1; }

    strcpy(ifr.ifr_name, "can0");
    if (ioctl(sock, SIOCGIFINDEX, &ifr) < 0) { perror("ioctl"); return 1; }

    memset(&addr, 0, sizeof(addr));
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    { perror("bind"); return 1; }

    printf("=== CAN Drive Control (PID) ===\n");
    printf("1:Left  2:Forward  3:Right  4:Backward  5:UTurn  0:Stop\n");
    printf("+/-: RPM up/down (current: %.1f)\n", target_rpm);
    printf("e: E-Stop  q: Exit\n");
    printf("------------------------------------------\n");

    term_raw_on();

    uint64_t last_tx_ms = 0;
    const uint64_t tx_period_ms = 100;

    while (1)
    {
        recv_feedback(sock);

        uint64_t t = now_ms();
        if (t - last_tx_ms >= tx_period_ms) {
            send_cmd(sock, last_dir, last_rpm);
            last_tx_ms = t;
        }

        fd_set fds;
        struct timeval tv = {0, 10000};
        FD_ZERO(&fds);
        FD_SET(STDIN_FILENO, &fds);

        if (select(STDIN_FILENO + 1, &fds, NULL, NULL, &tv) > 0)
        {
            char key = getchar();
            switch (key)
            {
                case '1':
                    last_dir = DIR_LEFT;
                    last_rpm = target_rpm;
                    send_cmd(sock, last_dir, last_rpm);
                    last_tx_ms = now_ms();
                    printf("\n[TX] Left\n");
                    break;
                case '2':
                    last_dir = DIR_FORWARD;
                    last_rpm = target_rpm;
                    send_cmd(sock, last_dir, last_rpm);
                    last_tx_ms = now_ms();
                    printf("\n[TX] Forward RPM:%.1f\n", target_rpm);
                    break;
                case '3':
                    last_dir = DIR_RIGHT;
                    last_rpm = target_rpm;
                    send_cmd(sock, last_dir, last_rpm);
                    last_tx_ms = now_ms();
                    printf("\n[TX] Right\n");
                    break;
                case '4':
                    last_dir = DIR_BACKWARD;
                    last_rpm = target_rpm;
                    send_cmd(sock, last_dir, last_rpm);
                    last_tx_ms = now_ms();
                    printf("\n[TX] Backward RPM:%.1f\n", target_rpm);
                    break;
                case '5':                          // ★ 유턴
                    last_dir = DIR_UTURN;
                    last_rpm = target_rpm;
                    send_cmd(sock, last_dir, last_rpm);
                    last_tx_ms = now_ms();
                    printf("\n[TX] UTurn\n");
                    break;
                case '0':
                    last_dir = DIR_STOP;
                    last_rpm = 0.0f;
                    send_cmd(sock, last_dir, last_rpm);
                    last_tx_ms = now_ms();
                    printf("\n[TX] Stop\n");
                    break;
                case '+':
                    if (target_rpm + 10.0f <= 300.0f) target_rpm += 10.0f;
                    last_rpm = target_rpm;
                    printf("\n[RPM] Target: %.1f\n", target_rpm);
                    break;
                case '-':
                    if (target_rpm - 10.0f >= 0.0f) target_rpm -= 10.0f;
                    last_rpm = target_rpm;
                    printf("\n[RPM] Target: %.1f\n", target_rpm);
                    break;
                case 'e':
                    last_dir = DIR_STOP;
                    last_rpm = 0.0f;
                    send_estop(sock);
                    printf("\n[E-Stop]!\n");
                    break;
                case 'q':
                    send_cmd(sock, DIR_STOP, 0);
                    printf("\nExit\n");
                    goto exit_loop;
                default:
                    break;
            }
        }
    }

exit_loop:
    term_raw_off();
    close(sock);
    return 0;
}
