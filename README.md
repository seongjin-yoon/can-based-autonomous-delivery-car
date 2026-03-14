# CATNIP — CAN 기반 분산 ECU 무인 배달 차량 시스템

> **Intel AI SW Academy 9기 1차 프로젝트**  
> STM32 × 2 + Raspberry Pi × 3 분산 ECU 아키텍처로 구현한 CAN 통신 기반 무인 배달 RC카


---

## 📌 프로젝트 개요

실제 차량의 ECU 분산 구조(주행 / 미션 / 화물함)를 RC카로 재현하여 CAN 통신 기반 임베디드 시스템 설계 역량을 검증하는 프로젝트입니다.

OpenCV 라인트레이싱 기반 자율 주행, PIN 인증 화물함 제어, Qt 기반 주문 클라이언트, MQTT 실시간 관제를 단일 시스템으로 통합합니다.

---

## 🏗️ 시스템 아키텍처

```
┌─────────────────────────────────────────────┐
│               RC카 (온보드)                 │
│                                             │
│  STM32 (B-L475E) ─────┐                     │
│  (주행 ECU)           │                     │
│                       CAN 버스 (250 kbps)   │
│  RPi1 ────────────────┤                     │
│  (미션 ECU)           │   Wi-Fi / MQTT      │
│                       │   └──────────────── ┼──→ RPi3 (서버)
│  STM103 ──────────────┘                     │         │
│  (화물함 ECU)                               │         │ MQTT
└─────────────────────────────────────────────┘     RPi4 (클라이언트)

온보드 CAN  : STM32 ↔ RPi1 ↔ STM103   단일 버스 250 kbps
오프보드 MQTT: RPi1  ↔ RPi3            무선 (차량당 연결 1개)
               RPi3  ↔ RPi4            무선
```

---

## 🖥️ 노드별 역할

| 노드 | 보드 | 위치 | 역할 |
|---|---|---|---|
| STM32 | B-L475E-IOT01A (STM32L4) | 온보드 | 주행 ECU — 모터 PWM / 엔코더 / 속도 PID / CAN |
| RPi1 | Raspberry Pi | 온보드 | 미션 ECU — 라인트레이싱 / ArUco / 미션 상태머신 / MQTT 게이트웨이 |
| STM103 | STM32F103 | 온보드 | 화물함 ECU — PIN 인증 / 서보 / LCD / 키패드 |
| RPi3 | Raspberry Pi | 오프보드 (서버) | MQTT 브로커 / 주문 중계 / SQLite DB |
| RPi4 | Raspberry Pi | 오프보드 (클라이언트) | Qt 주문 UI / 5인치 터치 디스플레이 |

**역할 분담 원칙**

- **STM32**: 판단하지 않는다. 명령 수행 + 센서 수집만 담당
- **RPi1**: 모든 판단은 여기서 한다. 외부 통신도 RPi1만 담당
- **STM103**: 주행 ECU와 완전 독립된 보안 영역. 외부 네트워크 직접 연결 없음
- **RPi3**: 실시간 제어에 절대 관여하지 않는다. 이벤트 기반 중계 및 저장

---

## 🚀 전체 배달 시나리오

```
① 고객 주문
   RPi4 Qt UI → 목적지(A~D) + PIN 4자리 설정 → MQTT → RPi3
   RPi3 → DB 저장 → RPi1 출동 명령 (destination + pin + order_id)
   RPi1 → CAN 0x012 → STM103: PIN + 목적지 전달

② 자율 주행
   RPi1: 카메라 라인트레이싱 → CAN 0x010 → STM32 모터 PID 제어
   교차로 감지 시 목적지별 행동 테이블에 따라 좌/우/직진 분기

③ 목적지 도착
   RPi1: ArUco 마커 ID(1~4) 감지 → 즉시 정지
   RPi1 → CAN 0x013 → STM103: 도착 신호
   RPi1 → MQTT → RPi3: arrived 보고

④ 화물함 인증
   소비자: LCD 확인 → 목적지 키(A~D) 입력 → PIN 4자리 입력(# 제출)
   성공 → 서보 열림(5초) → 닫힘 → RPi1 유턴 트리거
   실패 5회 → 10초 잠금 + MQTT alert
   오배달 / 미수령 30초 → CAN 0x302=0x02 → 즉시 유턴

⑤ 귀환
   RPi1: U턴 → 라인 재탐색 → 귀환 교차로 테이블 추종
   ArUco ID=0(출발지) 감지 → 완료
   RPi1 → MQTT → RPi3: completed
```

---

## 📡 CAN 프로토콜

| CAN ID | 내용 | 송신 | 수신 | 주기 |
|---|---|---|---|---|
| **0x010** | 주행 명령 (방향 + RPM) | RPi1 | STM32 | 50 ms |
| **0x011** | E-Stop | RPi1 | STM32 | 즉시 |
| **0x012** | 배달정보 (PIN + 목적지) | RPi1 | STM103 | 이벤트 |
| **0x013** | 도착 신호 | RPi1 | STM103 | 이벤트 |
| **0x100** | 속도 피드백 (RPM × 100) | STM32 | RPi1 | 50 ms |
| **0x101** | 전방 거리 (VL53L0X, mm) | STM32 | RPi1 | 50 ms |
| **0x200** | ECU Heartbeat (0xAA) | STM32 | RPi1 | 100 ms |
| **0x301** | 도어 상태 (0x00=닫힘) | STM103 | RPi1 | 이벤트 |
| **0x302** | 인증 결과 (0x01=성공 / 0x02=유턴) | STM103 | RPi1 | 이벤트 |
| **0x303** | PIN 5회 실패 잠금 | STM103 | RPi1 | 이벤트 |

통신 속도: **250 kbps** / 버스 점유율 1% 미만

---

## 🌐 MQTT 토픽

| 토픽 | 방향 | 내용 |
|---|---|---|
| `delivery/order/{order_id}/{destination}` | RPi4 → RPi3 | 주문 정보 (PIN + 메뉴 포함) |
| `delivery/order/{order_id}/3to4` | RPi3 → RPi4 | 주문 수신 ACK |
| `delivery/vehicle/{vehicle_id}/order` | RPi3 → RPi1 | 출동 명령 (PIN 포함) |
| `delivery/vehicle/{vehicle_id}/1to3` | RPi1 → RPi3 | 출발 / 도착 / 완료 보고 |
| `delivery/vehicle/{vehicle_id}/status` | RPi1 → RPi3 | Heartbeat (2초 주기) |
| `delivery/vehicle/{vehicle_id}/alert` | RPi1 → RPi3 | pin_locked 이벤트 |

브로커: `10.42.0.1:1883` (Pi5_MQTT_AP 핫스팟)

---

## ⚙️ 기술 스택

| 영역 | 기술 |
|---|---|
| STM32 / STM103 펌웨어 | C + STM32 HAL (CubeMX) |
| 라인트레이싱 / ArUco 인식 | C++ + OpenCV 4 |
| 미션 상태머신 / CAN 통신 | C++ + SocketCAN + mosquittopp |
| MQTT 브로커 / 서버 DB | Mosquitto + SQLite |
| 주문 클라이언트 UI | C++ + Qt Widgets |
| CAN 컨트롤러 | MCP2515 (RPi1) / bxCAN 내장 (STM32, STM103) |
| CAN 트랜시버 | SN65HVD230 (STM32, STM103) / TJA1050 내장 (MCP2515) |
| 빌드 | STM32CubeIDE / g++ |

---

## 🔌 주요 하드웨어

| 부품 | 수량 | 비고 |
|---|---|---|
| B-L475E-IOT01A (STM32L4) | 1 | 주행 ECU |
| STM32F103 보드 (STM103) | 2 | 화물함 ECU 1 + CAN 시뮬레이터 1 |
| Raspberry Pi | 3 | RPi1 / RPi3 / RPi4 |
| JGB37-520 엔코더 모터 | 4 | 좌우 각 2개 병렬 |
| L298N 모터 드라이버 | 1 | 스키드 스티어링 |
| SN65HVD230 CAN 트랜시버 | 2 | STM32 / STM103 각 1 |
| MCP2515 CAN 모듈 (5V) | 2 | RPi1 전용 1 + 예비 1 |
| 4채널 레벨시프터 | 2 | 엔코더 4채널 + MCP2515 MISO/INT |
| 220Ω 저항 | 4 | 종단저항 (양 끝단 2개 병렬) |
| USB 카메라 | 1 | 라인트레이싱 + ArUco |
| 소형 서보 모터 | 1 | 화물함 도어 |
| LCD 1602 (I2C, HD44780) | 1 | 화물함 안내 표시 |
| 4×4 키패드 | 1 | PIN 입력 |
| 5인치 터치 디스플레이 (XPT2046) | 1 | RPi4 주문 UI |
| 12V 배터리 | 1세트 | 구동 전원 |

---

## 🛠️ 빌드 및 실행

**RPi1 미션 ECU**

```bash
g++ delivery_mqtt.cpp -o delivery \
    $(pkg-config --cflags --libs opencv4) \
    -lmosquittopp -lpthread

./delivery             # 브로커 기본 10.42.0.1
./delivery 10.42.0.1   # 브로커 IP 직접 지정
```

**RPi3 서버**

```bash
g++ -std=c++17 delivery_server.cpp -o delivery_server \
    -lmosquitto -lmosquittopp -lsqlite3

./delivery_server [broker_ip]

# DB 조회
sqlite3 /home/pi/catnip/database/delivery_system.db \
    "SELECT order_id, destination, pin, status FROM order_table;"
```

**RPi1 CAN 인터페이스 설정** (`/boot/config.txt`)

```bash
dtoverlay=mcp2515-can0,oscillator=8000000,interrupt=25
```

```bash
sudo ip link set can0 up type can bitrate 250000
```

---

## 📂 브랜치 구조

```
main                           ← 최종 통합 (직접 push 금지)
dev                            ← 통합 테스트
├── feature/stm32
│    ├── feature/stm32-youngmo
│    └── feature/stm32-insumin
├── feature/stm103
├── feature/stm103-sim
├── feature/rpi1
│    ├── feature/rpi1-vision
│    └── feature/rpi1-system
├── feature/rpi3
└── feature/rpi4
```

**커밋 컨벤션**

```
[STM32]  CAN RX 0x010 주행 명령 수신 구현
[STM103] PIN 인증 상태머신 구현
[RPi1]   교차로 유턴 로직 수정
[RPi3]   alert 토픽 구독 및 로그 처리
[docs]   CAN ID 테이블 업데이트
```

---

## 🛡️ Fail-safe 설계

| 상황 | 감지 | 대응 |
|---|---|---|
| RPi1 다운 | STM32 IWDG 300ms 만료 | MCU 리셋 → 모터 즉시 정지 |
| CAN 명령 타임아웃 | 5초간 0x010 미수신 | Motor_Stop() 자동 실행 |
| STM32 Heartbeat 누락 | 300ms 이상 0x200 미수신 | E-Stop(0x011) + MQTT 보고 |
| PIN 5회 실패 | STM103 카운트 | 10초 잠금 + MQTT alert |
| 오배달 | STM103 목적지 불일치 | 0x302=0x02 → 즉시 유턴 |
| 미수령 타임아웃 | 도착 후 30초 초과 | 0x302=0x02 → 즉시 유턴 |

---

## 👥 팀 구성

| 이름 | 역할 | 담당 영역 |
|---|---|---|
| 구영모 | FW-Drive / PM | STM32 주행 ECU (모터 / 엔코더 / PID / CAN) + 전체 일정 / 문서 |
| 인수민 | FW-Drive | STM32 주행 ECU 공동 담당 |
| 윤성진 | Vision / System | RPi1 라인트레이싱 / ArUco / 미션 상태머신 / 일정 관리 |
| 김민우 | FW-Cargo | STM103 화물함 ECU (CAN / 키패드 / PIN / 서보 / LCD) |
| 최지호 | BE / Network | RPi3 서버 (Mosquitto / SQLite / MQTT / 클라이언트 연동) |

---

## ⚠️ 하드웨어 주의사항

- `main` 브랜치 직접 push 금지 — PR + 리뷰 후 머지
- CAN 버스 양 끝단 **종단저항(120Ω) 필수** — 미연결 시 ACK 에러 / BUS-OFF
- SN65HVD230 전원은 **3.3V** — GND와 핀 위치 혼동 주의 (오결선으로 CAN 불통 경험)
- MCP2515 MISO / INT 신호 5V → RPi 3.3V **레벨시프터 필수** (MOSI / SCK / CS는 불필요)
- JGB37-520 엔코더 신호 5V → STM32 3.3V **레벨시프터 필수**
- CAN 버스는 **직선(데이지체인)** 구조 필수 — 별형(Star) 배선 금지
