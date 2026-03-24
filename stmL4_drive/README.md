# stmL4 — 주행 ECU (Drive ECU)

> **CAN 기반 분산 ECU 무인 배달 차량 시스템** | CATNIP 팀

![stml4_board](/assets/stml4_board.png)

---

## 역할 및 개요

RC카에 탑재된 **모터 제어 전담 노드**.

| 항목 | 내용 |
|------|------|
| 보드 | B-L475E-IOT01A (STM32L476) |
| 언어 | C (STM32 HAL) |
| 주요 기능 | 모터 PWM 제어, 엔코더 RPM 측정, PID 속도 제어, CAN 통신, IWDG Watchdog |

### 설계 원칙
- **"판단하지 않는다."** — 명령 수행 + 센서 데이터 수집만 담당
- RPi1으로부터 CAN 0x010 주행 명령 수신 → L298N 제어
- RPi1 다운 시 IWDG 300ms 만료 → MCU 리셋 → 모터 즉시 정지

---

## 핀맵

| 기능 | 핀 | CubeMX 설정 | 연결 대상 |
|------|-----|------------|----------|
| USART1_TX | PB6 | USART1_TX | ST-LINK VCP (PC COM) |
| USART1_RX | PB7 | USART1_RX | ST-LINK VCP |
| CAN1_RX | PB8 | CAN1_RX | SN65HVD230 RXD |
| CAN1_TX | PB9 | CAN1_TX | SN65HVD230 TXD |
| TIM5_CH1 | PA0 | Encoder | 왼쪽 엔코더 A (레벨시프터 경유) |
| TIM5_CH2 | PA1 | Encoder | 왼쪽 엔코더 B (레벨시프터 경유) |
| TIM3_CH1 | PA6 | Encoder | 오른쪽 엔코더 A (레벨시프터 경유) |
| TIM3_CH2 | PA7 | Encoder | 오른쪽 엔코더 B (레벨시프터 경유) |
| TIM2_CH1 | PA15 | PWM | L298N ENA (왼쪽) |
| TIM2_CH3 | PA2 | PWM | L298N ENB (오른쪽) |
| GPIO_OUT | PC2 | GPIO_Output | L298N IN1 |
| GPIO_OUT | PC3 | GPIO_Output | L298N IN2 |
| GPIO_OUT | PC4 | GPIO_Output | L298N IN3 |
| GPIO_OUT | PC5 | GPIO_Output | L298N IN4 |
| I2C2_SCL | PB10 | I2C2_SCL | VL53L0X 전방 거리센서 |
| I2C2_SDA | PB11 | I2C2_SDA | VL53L0X |

> ⚠️ **PA15**: JTAG 핀 — CubeMX에서 Serial Wire(PA13/PA14)로 설정하고 PA15 JTAG 해제 필수

---

## CubeMX 설정

```
USART1: PB6(TX)/PB7(RX), 115200/8N1, NVIC Enable
CAN1:   PB8(RX)/PB9(TX), Prescaler=16, BS1=13TQ, BS2=2TQ → 250kbps
TIM2:   CH1(PA15)/CH3(PA2), Prescaler=79, Period=999 → 1kHz PWM
TIM5:   CH1(PA0)/CH2(PA1), Encoder Mode TI12, Period=65535 (왼쪽)
TIM3:   CH1(PA6)/CH2(PA7), Encoder Mode TI12, Period=65535 (오른쪽)
TIM6:   Prescaler=79, Period=9999 → 10ms PID 인터럽트
TIM7:   HAL Timebase (SysTick 대체)
IWDG:   Prescaler=/32, Reload=300 → 300ms Watchdog
Clock:  HSI 16MHz → PLL → SYSCLK 80MHz
SYS:    Debug: Serial Wire
```

---

## CAN 프로토콜

### 수신 (RPi1 → STM32)

| CAN ID | 내용 | 주기 |
|--------|------|------|
| `0x010` | 주행 명령 (방향 + RPM) | 50 ms |
| `0x011` | E-Stop | 즉시 |

### 0x010 페이로드 (DLC=3)

| Byte | 내용 | 값 |
|------|------|----|
| [0] | 방향 | 0=정지, 1=전진, 2=후진, 3=좌, 4=우, 5=U턴 |
| [1] | RPM×10 상위 바이트 | uint8 |
| [2] | RPM×10 하위 바이트 | uint8 |

### 송신 (STM32 → RPi1)

| CAN ID | 내용 | 주기 |
|--------|------|------|
| `0x100` | 속도 피드백 (좌/우 RPM×100, PWM 값) | 50 ms |
| `0x101` | 전방 거리 (VL53L0X, mm) | 50 ms |
| `0x200` | ECU Heartbeat (0xAA) | 100 ms |

### 0x100 페이로드 (DLC=8)

| Byte | 내용 | 타입 |
|------|------|------|
| [0~1] | 좌 RPM × 100 | int16 Big-Endian |
| [2~3] | 우 RPM × 100 | int16 Big-Endian |
| [4~5] | 좌 CCR (PWM 값) | uint16 Big-Endian |
| [6~7] | 우 CCR (PWM 값) | uint16 Big-Endian |

---

## 모터 제어 정책

| 방향 | PID | 동작 |
|------|:---:|------|
| 전진 (1) | ON | target_rpm 추종 |
| 후진 (2) | OFF | PWM 320 고정 |
| 좌회전 (3) | OFF | 왼쪽 후진(500) / 오른쪽 전진(900) |
| 우회전 (4) | OFF | 왼쪽 전진(900) / 오른쪽 후진(500) |
| U턴 (5) | OFF | 왼쪽 후진(500) / 오른쪽 전진(500) |
| 정지 (0) | OFF | PWM=0, IN1=IN2=HIGH / IN3=IN4=HIGH (브레이크 모드) |

### IN 핀 방향 제어

```
전진: IN1=H IN2=L / IN3=H IN4=L
후진: IN1=L IN2=H / IN3=L IN4=H
좌:   IN1=L IN2=H / IN3=H IN4=L
우:   IN1=H IN2=L / IN3=L IN4=H
정지: IN1=H IN2=H / IN3=H IN4=H  ← 브레이크 모드
```

---

## 핵심 파라미터

```c
#define PWM_MAX              999
#define PWM_OFFSET           300.0f
#define PULSE_PER_REV        5280        // 실측 확정값
#define PID_INTERVAL_MS      10

#define TARGET_RPM_DEFAULT   100.0f
#define TARGET_RPM_MAX       350.0f

#define KP                   0.5f
#define KI                   0.05f
#define KD                   0.0f
#define INTEGRAL_LIMIT       150.0f

#define PWM_FORWARD          400
#define PWM_TURN             900
#define PWM_BACKWARD         320

#define CAN_CMD_TIMEOUT_MS   5000        // 5초간 CAN 미수신 시 자동 정지
```

---

## 타이머 사용 현황

| 타이머 | 용도 | 핀 |
|--------|------|----|
| TIM2 | PWM — ENA(CH1), ENB(CH3) | PA15, PA2 |
| TIM3 | Encoder — 오른쪽 | PA6, PA7 |
| TIM5 | Encoder — 왼쪽 | PA0, PA1 |
| TIM6 | PID 인터럽트 (10ms) | — |
| TIM7 | HAL Timebase | — |
| IWDG | Watchdog 300ms | — |

---

## 타이밍 구조

```
TIM6 인터럽트 (10ms)
├── pid_flag = 1
└── can_timeout_cnt 증가

메인 루프
├── CAN RX 처리 (can_rx_flag)
├── CAN 타임아웃 체크 (5초 → Motor_Stop)
└── pid_flag 처리
    ├── Encoder_Update() → RPM 계산
    ├── PID_Compute() → PWM 출력 (전진 시만)
    ├── CAN TX: 속도 피드백 (50ms, 0x100)
    └── CAN TX: Heartbeat (100ms, 0x200)
```

---

## Fail-safe

| 항목 | 값 | 동작 |
|------|-----|------|
| IWDG Watchdog | 300 ms | RPi1 다운 시 MCU 리셋 → 모터 즉시 정지 |
| CAN 타임아웃 | 5000 ms | 0x010 미수신 시 Motor_Stop() 자동 실행 |

---

## 빌드 및 플래시

STM32CubeIDE에서 프로젝트 열기 → Build → Flash

### 링커 플래그 (float printf 출력 필요 시)

```
Project Properties → C/C++ Build → Settings → Linker flags
-u _printf_float 추가
```

### 주의사항

- **PA15 JTAG 해제** 필수 (CubeMX SYS → Debug: Serial Wire)
- SN65HVD230 VCC = **3.3V** (GND와 혼동 주의)
- 엔코더 신호는 **레벨시프터 경유** 필수 (5V 엔코더 → 3.3V STM32)
- L298N ENA/ENB는 PA15/PA2 **직결** 가능 (3.3V PWM으로 L298N 인식 확인)
