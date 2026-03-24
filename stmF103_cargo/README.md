# stmF103 — 화물함 ECU (Cargo ECU)

> **CAN 기반 분산 ECU 무인 배달 차량 시스템** | CATNIP 팀

---

## 역할 및 개요

RC카에 탑재된 **보안 화물함 제어 노드**.

| 항목 | 내용 |
|------|------|
| 보드 | STM32F103 독립 보드 (MangoM32) |
| 언어 | C (STM32 HAL) |
| 주요 기능 | PIN 인증, 서보 도어 제어, LCD 상태 표시, 4×4 키패드 입력, CAN 통신 |

### 설계 원칙
- **주행 ECU와 완전 독립된 보안 영역** — CAN만 사용, 외부 네트워크 직접 연결 없음
- PIN 인증 실패/타임아웃 시 RPi1에 즉시 신호 전송하여 유턴 트리거

---

## 핀맵

| 기능 | 핀 | CubeMX 설정 | 연결 대상 |
|------|-----|------------|----------|
| CAN1_RX | PB8 | CAN1_RX | SN65HVD230 RXD |
| CAN1_TX | PB9 | CAN1_TX | SN65HVD230 TXD |
| I2C2_SCL | PB10 | I2C2_SCL | LCD (HD44780, I2C) |
| I2C2_SDA | PB11 | I2C2_SDA | LCD |
| TIM3_CH1 | PA6 | PWM | 서보 모터 (50Hz) |
| LED | PA5 | GPIO_Output | 상태 표시 LED |
| USART1_TX | PA9 | USART1_TX | 디버그 115200 bps |
| USART1_RX | PA10 | USART1_RX | 디버그 |
| Keypad Row1~4 | PC0~PC3 | GPIO_Output | 4×4 키패드 행 |
| Keypad Col1~4 | PB12~PB15 | GPIO_Input Pull-Up | 4×4 키패드 열 |

---

## CubeMX 설정

```
CAN1:   PB8(RX)/PB9(TX), Prescaler=6, BS1=13TQ, BS2=2TQ → 250kbps
        AFIO Remap2 적용 필수: __HAL_AFIO_REMAP_CAN1_2()
I2C2:   PB10(SCL)/PB11(SDA), Standard Mode 100kHz
TIM3:   CH1(PA6), Prescaler=47, Period=19999 → 50Hz (서보 표준)
USART1: PA9(TX)/PA10(RX), 115200/8N1
GPIO:   PC0~PC3 Output / PB12~PB15 Input Pull-Up
Clock:  HSI 8MHz → PLL×12 → SYSCLK 48MHz
        APB1 /2 → PCLK1 24MHz
```

---

## CAN 프로토콜

### 수신 (RPi1 → STM103)

| CAN ID | 내용 | 처리 |
|--------|------|------|
| `0x012` | 배달정보 (PIN 4자리 + 목적지) | STATE_PKG_RECEIVED 진입 |
| `0x013` | 도착 신호 | STATE_VERIFY_DEST 진입 |

### 0x012 페이로드 (DLC=6)

| Byte | 내용 | 값 |
|------|------|----|
| [0] | 유효 패킷 식별 | 0x01 |
| [1] | PIN[0] | 숫자 0~9 |
| [2] | PIN[1] | 숫자 0~9 |
| [3] | PIN[2] | 숫자 0~9 |
| [4] | PIN[3] | 숫자 0~9 |
| [5] | 목적지 코드 | 0x01=A, 0x02=B, 0x03=C, 0x04=D |

### 송신 (STM103 → RPi1)

| CAN ID | 내용 | 트리거 |
|--------|------|--------|
| `0x301` | 도어 상태 (0x00=닫힘) | 서보 닫힘 완료 → RPi1 유턴 |
| `0x302` | 인증 결과 (0x01=성공, 0x02=오배달/미수령) | 인증 완료 또는 즉시 유턴 |
| `0x303` | PIN 5회 실패 잠금 (0x01) | MQTT alert 전송 트리거 |

---

## 보안 상태머신

```
STATE_IDLE
└→ CAN 0x012 수신 → STATE_PKG_RECEIVED

STATE_PKG_RECEIVED
└→ CAN 0x013 수신 → STATE_VERIFY_DEST

STATE_VERIFY_DEST
├→ 목적지 키(A~D) 일치 → STATE_WAIT_FOR_PIN
└→ 목적지 불일치 → STATE_WRONG_DEST_CONFIRM
    ├→ '1'(YES) → CAN 0x302=0x02 → STATE_IDLE (오배달 귀환)
    └→ '2'(NO)  → STATE_VERIFY_DEST

STATE_WAIT_FOR_PIN
├→ PIN 성공 → 서보 열림(5초) → 서보 닫힘
│             CAN 0x302=0x01 + CAN 0x301=0x00 → STATE_IDLE
├→ PIN 실패 1~4회 → 재입력
└→ PIN 실패 5회 → CAN 0x303=0x01 → STATE_LOCKED (10초)

미수령 타임아웃 (30초)
└→ CAN 0x302=0x02 → STATE_IDLE (귀환)
```

---

## 키패드 레이아웃

```
        Col1(PB12) Col2(PB13) Col3(PB14) Col4(PB15)
Row1(PC0)   1          2          3          A
Row2(PC1)   4          5          6          B
Row3(PC2)   7          8          9          C
Row4(PC3)   *          0          #          D

* = 한 자리 삭제 (백스페이스)
# = 확인 (제출)
A~D = 목적지 입력
```

---

## LCD 상태 표시

| 상태 | 1행 | 2행 |
|------|-----|-----|
| IDLE | `  Cargo  ECU  ` | `   Waiting...  ` |
| PKG_RECEIVED | `Destination: X` | `PIN received!` |
| VERIFY_DEST | `Destination?` | `Enter: A B C D` |
| WAIT_FOR_PIN | `Pkg arrived!!` | `PW: ****` |
| 성공 | `Delivery Done!` | `Enjoy ur day! :D` |
| LOCKED | `!! LOCKED !!` | `  Wait: Xs...` |
| 오배달/미수령 | `Wrong delivery!` | `Returning home` |

---

## 서보 파라미터

```c
#define SERVO_OPEN   2000   // 도어 열림 펄스폭 (μs)
#define SERVO_CLOSE  1000   // 도어 닫힘 펄스폭 (μs)
```

---

## 빌드 및 플래시

STM32CubeIDE에서 프로젝트 열기 → Build → Flash

### 주의사항

- **AFIO Remap2 필수**: `__HAL_AFIO_REMAP_CAN1_2()` 미적용 시 CAN 통신 불가
- **PB8 외부 Pull-up** 적용 (CAN Normal mode 진입 조건)
- SN65HVD230 VCC = **3.3V** (GND와 혼동 주의 — 오결선 시 CAN 불통)
- 플래시 실패(HardFault 잠금) 시: STM32CubeProgrammer → Under Reset + Full Erase 후 재플래시
