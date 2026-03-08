#include <iostream>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <string>
#include <unistd.h>

// gpiomon 출력 한 줄 예:
// 200.307113600   rising  gpiochip0 24 "GPIO24"
static bool parse_line(const std::string& line, double& t_sec, std::string& edge) {
    // 앞의 시간(초) + 엣지(rising/falling)만 뽑기
    // 공백이 여러 개라서 sscanf로 처리
    char edge_buf[32] = {0};
    double t = 0.0;
    if (sscanf(line.c_str(), "%lf %31s", &t, edge_buf) == 2) {
        t_sec = t;
        edge = edge_buf;
        return true;
    }
    return false;
}

int main() {
    // gpiomon을 자식 프로세스로 실행해서 출력 읽기
    const char* cmd = "sudo gpiomon -c gpiochip0 -e both 24";
    FILE* fp = popen(cmd, "r");
    if (!fp) {
        std::cerr << "Failed to run gpiomon\n";
        return 1;
    }

    std::cout << "Reading ECHO edges... (Ctrl+C to stop)\n";

    char buf[512];
    bool have_rise = false;
    double t_rise = 0.0;

    while (fgets(buf, sizeof(buf), fp)) {
        std::string line(buf);

        double t_sec;
        std::string edge;
        if (!parse_line(line, t_sec, edge)) continue;

        if (edge == "rising") {
            // 상승 엣지 기록
            t_rise = t_sec;
            have_rise = true;
        } else if (edge == "falling") {
            if (!have_rise) {
                // 상승 없이 하강이 먼저 온 것은 무시(노이즈/초기상태 등)
                continue;
            }
            double t_fall = t_sec;

            // 펄스폭 (초 → 마이크로초)
            double pulse_s = t_fall - t_rise;
            if (pulse_s <= 0 || pulse_s > 0.05) { // 50ms 이상이면 이상치
                have_rise = false;
                continue;
            }

            double pulse_us = pulse_s * 1e6;
            double dist_cm = pulse_us / 58.0;

            std::cout << "pulse_us=" << pulse_us
                      << "  dist_cm=" << dist_cm << "\n";

            have_rise = false;
        }
    }

    pclose(fp);
    return 0;
}
