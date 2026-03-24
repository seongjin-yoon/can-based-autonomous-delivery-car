# RPi1 — 미션 ECU (Mission ECU)

> **CAN 기반 분산 ECU 무인 배달 차량 시스템** | CATNIP 팀

---

## 역할 및 개요

RC카에 탑재된 **두뇌 노드**. 모든 판단은 이 노드에서 이루어진다.

| 항목 | 내용 |
|------|------|
| 보드 | Raspberry Pi (온보드) |
| 언어 | C++ |
| 주요 기능 | OpenCV 라인트레이싱, ArUco 마커 목적지 인식, 미션 상태머신, MQTT 게이트웨이, CAN 통신 |

### 설계 원칙
- **모든 판단은 RPi1에서 한다.** 서버(RPi3)와의 MQTT 통신도 RPi1만 담당
- STM32는 명령 수행만, RPi1은 라인/마커 감지 → 명령 생성 → CAN 송신
- 외부 MQTT 연결 단일 진입점 역할

---

## 하드웨어 연결

```
RPi1
├── MCP2515 (SPI) ──────────────── CAN 버스 (250 kbps)
│    ├── 오실레이터: 8 MHz
│    ├── 전원: 5V (RPi 5V 핀 공급) — 3.3V 직결 금지
│    ├── 트랜시버: TJA1050 내장 (5V 전용)
│    └── J1 점퍼 제거 (중간 노드 — 내장 120Ω 비활성화)
├── Wi-Fi ──────────────────────── MQTT → RPi3 (브로커: 10.42.0.1)
└── USB 카메라 (/dev/video0) ────── 라인트레이싱 + ArUco 인식
```

### MCP2515 레벨시프터 적용 범위

| SPI 신호 | 방향 | 레벨시프터 필요 | 이유 |
|----------|------|:--------------:|------|
| MOSI | RPi → MCP2515 | 불필요 | MCP2515가 3.3V 입력 허용 |
| SCK | RPi → MCP2515 | 불필요 | 동일 |
| CS | RPi → MCP2515 | 불필요 | 동일 |
| MISO | MCP2515 → RPi | **필수** | 5V 출력이 RPi 3.3V 핀 손상 유발 |
| INT | MCP2515 → RPi | **필수** | 동일 |

---

## CAN 프로토콜

### 송신 (RPi1 → 타 노드)

| CAN ID | 수신 | 내용 | 주기 |
|--------|------|------|------|
| `0x010` | STM32 | 주행 명령 (방향 + RPM) | 50 ms |
| `0x011` | STM32 | E-Stop | 즉시 |
| `0x012` | STM103 | 배달정보 (PIN 4자리 + 목적지) | 이벤트 |
| `0x013` | STM103 | 도착 신호 | 이벤트 |

### 수신 (타 노드 → RPi1)

| CAN ID | 송신 | 내용 | 처리 |
|--------|------|------|------|
| `0x100` | STM32 | 속도 피드백 (RPM × 100) | 50 ms |
| `0x101` | STM32 | 전방 거리 (mm) | 50 ms |
| `0x200` | STM32 | ECU Heartbeat (0xAA) | 100 ms |
| `0x301` | STM103 | 도어 상태 (0x00=닫힘) | 이벤트 → U턴 트리거 |
| `0x302` | STM103 | 인증 결과 (0x01=성공, 0x02=오배달) | 이벤트 |
| `0x303` | STM103 | PIN 5회 실패 잠금 | 이벤트 → MQTT alert |

### 0x010 페이로드 (DLC=3)

| Byte | 내용 | 값 |
|------|------|----|
| [0] | 방향 | 0=정지, 1=전진, 2=후진, 3=좌, 4=우, 5=U턴 |
| [1] | RPM×10 상위 바이트 | uint8 |
| [2] | RPM×10 하위 바이트 | uint8 |

---

## MQTT 프로토콜

| Topic | 방향 | 내용 |
|-------|------|------|
| `delivery/vehicle/{id}/order` | RPi3 → RPi1 | 출동 명령 (order_id, destination, pin, menus) |
| `delivery/vehicle/{id}/1to3` | RPi1 → RPi3 | 출발/도착/완료 보고 |
| `delivery/vehicle/{id}/status` | RPi1 → RPi3 | Heartbeat (2초 주기) |
| `delivery/vehicle/{id}/alert` | RPi1 → RPi3 | pin_locked 이벤트 |

### 브로커 접속 정보

```
IP:       10.42.0.1  (Pi5_MQTT_AP 핫스팟)
포트:     1883
사용자명: hoji
비밀번호: 1234
```

---

## 상태머신 요약

```
S_WAIT_CMD → (MQTT 출동 명령) → S_FOLLOW
S_FOLLOW   → (교차로 감지)   → S_JUNC_STOP → S_JUNC_LEFT/RIGHT/STRAIGHT → S_REACQUIRE → S_FOLLOW
S_FOLLOW   → (ArUco 도착)    → S_DELIVER_WAIT
S_DELIVER_WAIT → (CAN 0x301/0x302) → S_UTURN
S_UTURN    → S_REACQUIRE_AFTER_UTURN → S_FOLLOW (returning=true)
S_FOLLOW   → (ArUco ID=0)   → S_FINISHED → S_WAIT_CMD
```

---

## 주요 파라미터

```cpp
#define DEFAULT_RPM        40.0f    // 라인트레이싱 기본 속도
#define TURN_RPM          150.0f    // 교차로 회전 속도
#define STRAIGHT_DEADBAND   50      // 직진 판단 오차 허용 (px)
#define JUNC_STREAK          2      // 교차로 연속 감지 임계값
#define DUR_JUNC_STOP_MS   800      // 교차로 정지 시간 (ms)
#define DUR_TURN_MS       4000      // 좌/우회전 시간 (ms)
#define DUR_UTURN_MS     12000      // U턴 시간 (ms)
```

---

## 빌드 및 실행

### 의존성 설치

```bash
sudo apt install libopencv-dev libmosquitto-dev libmosquittopp-dev
```

### CAN 인터페이스 설정

```bash
# /boot/firmware/config.txt에 추가 (최초 1회)
# dtparam=spi=on
# dtoverlay=mcp2515-can0,oscillator=8000000,interrupt=25

sudo ip link set can0 up type can bitrate 250000
```

### 빌드

```bash
g++ delivery40.cpp -o delivery \
    $(pkg-config --cflags --libs opencv4) \
    -lmosquittopp -lpthread
```

### 실행

```bash
./delivery             # 기본 브로커 10.42.0.1
./delivery 10.42.0.1   # 브로커 IP 명시
```

### Wi-Fi 재연결

```bash
sudo nmcli device wifi connect "Pi5_MQTT_AP" password "12345678"
```

---

## 주의사항

- `send_cmd()` 내 `O_NONBLOCK` 설정 제거 필수 — 명령 드롭 방지
- MCP2515 전원은 반드시 **5V** 공급 (3.3V 직결 시 CAN 불통)
- CAN 버스 중간 노드이므로 **J1 점퍼 제거** (종단저항 비활성화)
