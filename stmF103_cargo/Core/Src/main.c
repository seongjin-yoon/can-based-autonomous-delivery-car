/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : CargoECU — STM32F103RBTx (MangoM32)
  *
  * [CAN 프로토콜 - 확정]
  *
  *  RPi1 → 모터ECU
  *   0x010 (DLC=3): 모터 명령 (dir + rpm)
  *   0x011 (DLC=1): E-Stop
  *
  *  RPi1 → 적재함ECU  <- 이 파일
  *   0x012 (DLC=6): 배달정보  Byte0=0x01, Byte1~4=PIN, Byte5=목적지
  *   0x013 (DLC=1): 도착신호  Byte0=0x01
  *
  *  적재함ECU → RPi1
  *   0x301 (DLC=1): 도어상태  0x00=닫힘 → RPi1 유턴 트리거
  *   0x302 (DLC=1): 인증결과  0x01=성공, 0x02=오배달/미수령
  *   0x303 (DLC=1): 잠금      0x01=5회 실패
  *
  * [핀맵]
  *  CAN    : PB8(RX) / PB9(TX)        AFIO Remap2
  *  I2C2   : PB10(SCL) / PB11(SDA)    → 16x2 LCD
  *  TIM3   : PA6(CH1)                 → Servo PWM 50Hz
  *  LED    : PA5                      → GPIO Output
  *  USART1 : PA9(TX) / PA10(RX)       → 디버그 115200
  *  Keypad Row : PC0~PC3              Output
  *  Keypad Col : PB12~PB15            Input Pull-Up
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include "can.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "lcd_i2c.h"
#include "keypad.h"
/* USER CODE END Includes */

/* USER CODE BEGIN PD */
/* 수정1: 30초로 수정 (300000U -> 30000U) */
#define AUTH_TIMEOUT_MS   30000U

/* 수정6: 서보 PWM 매크로화 */
#define SERVO_OPEN        2000
#define SERVO_CLOSE       1000

/* CAN ID 정의 */
#define CAN_CARGO_INFO    0x012
#define CAN_CARGO_ARRIVE  0x013
#define CAN_DOOR_STATUS   0x301
#define CAN_AUTH_RESULT   0x302
#define CAN_LOCKED        0x303
/* USER CODE END PD */

/* USER CODE BEGIN PV */
typedef enum {
    STATE_IDLE,
    STATE_PKG_RECEIVED,
    STATE_VERIFY_DEST,
    STATE_WRONG_DEST_CONFIRM,
    STATE_WAIT_FOR_PIN,
    STATE_LOCKED
} CargoState;

static const char* const STATE_NAME[] = {
    "IDLE", "PKG_RECEIVED", "VERIFY_DEST",
    "WRONG_DEST_CONFIRM", "WAIT_FOR_PIN", "LOCKED"
};

CargoState state       = STATE_IDLE;
char received_pin[5]   = "0000";
char input_pin[5]      = { 0 };
char dest_addr         = 'A';
char entered_dest      = '?';
uint8_t  pin_idx       = 0;
uint8_t  fail_cnt      = 0;
uint32_t lock_time     = 0;
uint32_t auth_start    = 0;

/* 수정3: last_lock_tick 전역화 */
uint32_t last_lock_tick = 0;

CAN_TxHeaderTypeDef TxHeader;
uint32_t TxMailbox;

volatile uint8_t pin_received_flag = 0;
volatile uint8_t arrival_flag      = 0;
/* USER CODE END PV */

void SystemClock_Config(void);

/* USER CODE BEGIN 0 */
int __io_putchar(int ch) {
    HAL_UART_Transmit(&huart1, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

static void LCD_PrintPad(uint8_t row, uint8_t col, const char *str)
{
    char buf[17];
    int len = (int)strlen(str);
    if (len > 16) len = 16;
    memcpy(buf, str, len);
    for (int i = len; i < 16; i++) buf[i] = ' ';
    buf[16] = '\0';
    LCD_SetCursor(row, col);
    LCD_Print(buf);
}

static void LCD_UpdatePinMask(void)
{
    char line[17] = "PW: ____        ";
    for (int i = 0; i < pin_idx; i++)
        line[4 + i] = '*';
    LCD_SetCursor(1, 0);
    LCD_Print(line);
}

static void CAN_Send(uint32_t id, uint8_t data)
{
    TxHeader.StdId              = id;
    TxHeader.DLC                = 1;
    TxHeader.IDE                = CAN_ID_STD;
    TxHeader.RTR                = CAN_RTR_DATA;
    TxHeader.TransmitGlobalTime = DISABLE;
    HAL_CAN_AddTxMessage(&hcan, &TxHeader, &data, &TxMailbox);
}

static void Log_StateChange(CargoState from, CargoState to)
{
    printf("  [STATE] %s -> %s\r\n", STATE_NAME[from], STATE_NAME[to]);
}

static void Reset_To_Idle(void)
{
    state        = STATE_IDLE;
    pin_idx      = 0;
    fail_cnt     = 0;
    dest_addr    = 'A';
    entered_dest = '?';
    auth_start   = 0;
    memset(input_pin, 0, sizeof(input_pin));
    memcpy(received_pin, "0000", 5);

    LCD_Clear();
    LCD_PrintPad(0, 0, "  Cargo  ECU  ");
    LCD_PrintPad(1, 0, "   Waiting...  ");

    printf("  [RESET] IDLE 복귀\r\n");
}
/* USER CODE END 0 */

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    __HAL_RCC_AFIO_CLK_ENABLE();
    __HAL_AFIO_REMAP_CAN1_2();

    MX_GPIO_Init();
    MX_CAN_Init();
    MX_USART1_UART_Init();
    MX_I2C2_Init();
    MX_TIM3_Init();

    /* CAN 필터 (전체 수신) */
    CAN_FilterTypeDef sFilterConfig = {0};
    sFilterConfig.FilterBank           = 0;
    sFilterConfig.FilterMode           = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale          = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig.FilterActivation     = ENABLE;
    HAL_CAN_ConfigFilter(&hcan, &sFilterConfig);
    HAL_CAN_Start(&hcan);
    HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

    /* 서보 닫힘 */
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, SERVO_CLOSE);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

    /* LED OFF */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

    /* LCD 초기화 */
    LCD_Init(&hi2c2);
    LCD_Clear();
    LCD_PrintPad(0, 0, "  Cargo  ECU  ");
    LCD_PrintPad(1, 0, "   Waiting...  ");

    printf("\r\n");
    printf("╔══════════════════════════════════════════╗\r\n");
    printf("║   CargoECU  STM32F103RBTx  MangoM32      ║\r\n");
    printf("║   CAN ID: RX=0x012,0x013 TX=0x301~0x303 ║\r\n");
    printf("╠══════════════════════════════════════════╣\r\n");
    printf("║  0x012: 배달정보(PIN+목적지) 수신         ║\r\n");
    printf("║  0x013: 도착신호 수신                     ║\r\n");
    printf("║  0x301: 도어닫힘 -> RPi1 유턴트리거       ║\r\n");
    printf("╚══════════════════════════════════════════╝\r\n\r\n");

    while (1)
    {
        /* ── [1] 0x012 배달정보 수신 처리 ── */
        if (pin_received_flag)
        {
            pin_received_flag = 0;
            printf("┌──────────────────────────────────────────┐\r\n");
            printf("│  [CAN RX 0x012] 배달정보 수신            │\r\n");
            printf("│  목적지: %c  PIN: %s                    │\r\n", dest_addr, received_pin);
            Log_StateChange(STATE_IDLE, STATE_PKG_RECEIVED);
            printf("│  0x013 도착신호 대기 중...               │\r\n");
            printf("└──────────────────────────────────────────┘\r\n\r\n");

            char dest_line[17];
            snprintf(dest_line, sizeof(dest_line), "Destination: %c  ", dest_addr);
            LCD_Clear();
            LCD_PrintPad(0, 0, dest_line);
            LCD_PrintPad(1, 0, "PIN received!   ");
        }

        /* ── [2] 0x013 도착신호 수신 처리 ── */
        if (arrival_flag)
        {
            arrival_flag = 0;
            auth_start   = HAL_GetTick();

            printf("┌──────────────────────────────────────────┐\r\n");
            printf("│  [CAN RX 0x013] 도착 신호!               │\r\n");
            Log_StateChange(STATE_PKG_RECEIVED, STATE_VERIFY_DEST);
            printf("└──────────────────────────────────────────┘\r\n\r\n");

            LCD_Clear();
            LCD_PrintPad(0, 0, "Destination?    ");
            LCD_PrintPad(1, 0, "Enter: A B C D  ");
        }

        /* ── [3] 잠금 상태 ── */
        if (state == STATE_LOCKED)
        {
            uint32_t elapsed = HAL_GetTick() - lock_time;
            if (elapsed >= 10000)
            {
                state          = STATE_WAIT_FOR_PIN;
                fail_cnt       = 0;
                pin_idx        = 0;
                last_lock_tick = 0;  /* 수정3: 잠금 해제 시 명시적 초기화 */
                memset(input_pin, 0, sizeof(input_pin));
                auth_start     = HAL_GetTick();

                LCD_Clear();
                LCD_PrintPad(0, 0, "Pkg arrived!!   ");
                LCD_UpdatePinMask();
                Log_StateChange(STATE_LOCKED, STATE_WAIT_FOR_PIN);
            }
            else
            {
                if (HAL_GetTick() - last_lock_tick >= 1000)
                {
                    last_lock_tick = HAL_GetTick();
                    uint32_t remain = (10000 - elapsed) / 1000;
                    char lock_line[17];
                    snprintf(lock_line, sizeof(lock_line), "  Wait: %lus...  ", remain);
                    LCD_PrintPad(1, 0, lock_line);
                    printf("[LOCKED] %lu sec remaining\r\n", remain);
                }
            }
            continue;
        }

        /* ── [4] 타임아웃 처리 ── */
        if ((state == STATE_VERIFY_DEST        ||
             state == STATE_WRONG_DEST_CONFIRM ||
             state == STATE_WAIT_FOR_PIN) && auth_start != 0)
        {
            if (HAL_GetTick() - auth_start >= AUTH_TIMEOUT_MS)
            {
                printf("[TIMEOUT] 미수령 -> RPi1에 0x302=0x02 귀환신호\r\n");
                CAN_Send(CAN_AUTH_RESULT, 0x02);
                LCD_Clear();
                LCD_PrintPad(0, 0, "No claim...     ");
                LCD_PrintPad(1, 0, "Returning home  ");
                HAL_Delay(1500);
                Reset_To_Idle();
                continue;
            }
        }

        /* ── [5] 키패드 스캔 ── */
        char key = Keypad_Scan();
        if (key != 0)
        {
            while (Keypad_Scan() != 0) HAL_Delay(10);
            HAL_Delay(50);
            printf("[KEY] '%c'  STATE=%s\r\n", key, STATE_NAME[state]);
        }

        /* ── [6] 목적지 확인 ── */
        if (state == STATE_VERIFY_DEST)
        {
            if (key != 'A' && key != 'B' && key != 'C' && key != 'D')
                continue;

            entered_dest = key;
            printf("[DEST] Input='%c'  Expected='%c'\r\n", entered_dest, dest_addr);

            if (entered_dest == dest_addr)
            {
                printf("[DEST] 일치! PIN 입력으로\r\n");
                Log_StateChange(STATE_VERIFY_DEST, STATE_WAIT_FOR_PIN);
                state   = STATE_WAIT_FOR_PIN;
                pin_idx = 0;
                memset(input_pin, 0, sizeof(input_pin));
                LCD_Clear();
                LCD_PrintPad(0, 0, "Pkg arrived!!   ");
                LCD_UpdatePinMask();
            }
            else
            {
                printf("[DEST] 불일치! 오배달 확인\r\n");
                Log_StateChange(STATE_VERIFY_DEST, STATE_WRONG_DEST_CONFIRM);
                state = STATE_WRONG_DEST_CONFIRM;
                char confirm_line[17];
                snprintf(confirm_line, sizeof(confirm_line), "Is dest %c right?", entered_dest);
                LCD_Clear();
                LCD_PrintPad(0, 0, confirm_line);
                LCD_PrintPad(1, 0, "1:YES  2:NO     ");
            }
            continue;
        }

        /* ── [7] 오배달 확인 ── */
        if (state == STATE_WRONG_DEST_CONFIRM)
        {
            if (key != '1' && key != '2') continue;

            if (key == '1')
            {
                printf("[오배달] 확정 -> RPi1에 0x302=0x02\r\n");
                CAN_Send(CAN_AUTH_RESULT, 0x02);
                LCD_Clear();
                LCD_PrintPad(0, 0, "Wrong delivery! ");
                LCD_PrintPad(1, 0, "Returning home  ");
                HAL_Delay(1500);
                Reset_To_Idle();
            }
            else
            {
                state = STATE_VERIFY_DEST;
                LCD_Clear();
                LCD_PrintPad(0, 0, "Destination?    ");
                LCD_PrintPad(1, 0, "Enter: A B C D  ");
            }
            continue;
        }

        /* ── [8] PIN 입력 ── */
        if (state != STATE_WAIT_FOR_PIN || key == 0)
            continue;

        if (key == '#')
        {
            /* 수정4: pin_idx 범위 방어 */
            input_pin[pin_idx < 4 ? pin_idx : 4] = '\0';
            printf("[AUTH] Input='%s'  Expected='%s'\r\n", input_pin, received_pin);

            if (strcmp(input_pin, received_pin) == 0)
            {
                fail_cnt = 0;
                printf("[AUTH] 성공!\r\n");

                LCD_Clear();
                LCD_PrintPad(0, 0, "Delivery Done!  ");
                LCD_PrintPad(1, 0, "Enjoy ur day! :D");

                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, SERVO_OPEN);

                CAN_Send(CAN_AUTH_RESULT, 0x01);
                printf("[CAN TX] 0x302=0x01 인증성공 -> RPi1\r\n");

                /* 수정7: HAL_GetTick() 기반 논블로킹 카운트다운 */
                uint32_t cd_start = HAL_GetTick();
                int last_cnt = 5;
                char cd_line[17];
                snprintf(cd_line, sizeof(cd_line), "Closing in %ds  ", last_cnt);
                LCD_PrintPad(1, 0, cd_line);
                printf("[SERVO] Closing in %d sec...\r\n", last_cnt);

                while (HAL_GetTick() - cd_start < 5000)
                {
                    int cur_cnt = 5 - (int)((HAL_GetTick() - cd_start) / 1000);
                    if (cur_cnt < 1) cur_cnt = 1;
                    if (cur_cnt != last_cnt)
                    {
                        last_cnt = cur_cnt;
                        snprintf(cd_line, sizeof(cd_line), "Closing in %ds  ", last_cnt);
                        LCD_PrintPad(1, 0, cd_line);
                        printf("[SERVO] Closing in %d sec...\r\n", last_cnt);
                    }
                }

                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, SERVO_CLOSE);
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

                CAN_Send(CAN_DOOR_STATUS, 0x00);
                printf("[CAN TX] 0x301=0x00 도어닫힘 -> RPi1 유턴트리거\r\n");

                Log_StateChange(STATE_WAIT_FOR_PIN, STATE_IDLE);
                Reset_To_Idle();
            }
            else
            {
                fail_cnt++;
                int remain_tries = 5 - (int)fail_cnt;
                printf("[AUTH] 실패 %d/5 (남은횟수: %d)\r\n", fail_cnt, remain_tries);

                pin_idx = 0;
                memset(input_pin, 0, sizeof(input_pin));

                LCD_Clear();
                LCD_PrintPad(0, 0, "Wrong PIN!      ");
                char try_line[17];
                if (remain_tries > 0)
                    snprintf(try_line, sizeof(try_line), "%d tries left   ", remain_tries);
                else
                    snprintf(try_line, sizeof(try_line), "No tries left!  ");
                LCD_PrintPad(1, 0, try_line);

                if (fail_cnt >= 5)
                {
                    CAN_Send(CAN_LOCKED, 0x01);
                    printf("[CAN TX] 0x303=0x01 잠금\r\n");

                    state          = STATE_LOCKED;
                    lock_time      = HAL_GetTick();
                    last_lock_tick = 0;  /* 수정3: 잠금 진입 시 명시적 초기화 */

                    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
                    HAL_Delay(600);
                    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

                    LCD_Clear();
                    LCD_PrintPad(0, 0, "!! LOCKED !!    ");
                    LCD_PrintPad(1, 0, "  Wait: 10s...  ");
                    Log_StateChange(STATE_WAIT_FOR_PIN, STATE_LOCKED);
                }
                else
                {
                    for (int i = 0; i < 6; i++) {
                        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
                        HAL_Delay(200);
                    }
                    HAL_Delay(600);
                    LCD_Clear();
                    LCD_PrintPad(0, 0, "Pkg arrived!!   ");
                    LCD_UpdatePinMask();
                }
            }
        }
        else if (key == '*')
        {
            if (pin_idx > 0) {
                pin_idx--;
                input_pin[pin_idx] = '\0';
                LCD_UpdatePinMask();
            }
        }
        else if (pin_idx < 4)
        {
            input_pin[pin_idx] = key;
            pin_idx++;
            LCD_UpdatePinMask();
        }
        else
        {
            printf("[PIN] 4자리 초과 — # 제출 또는 * 삭제\r\n");
        }
    }
}

/* ── CAN RX 인터럽트 콜백 ── */
/* 수정1: ISR 안 printf 전부 제거 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan_arg)
{
    CAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8] = {0};

    if (HAL_CAN_GetRxMessage(hcan_arg, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
        return;

    if (RxHeader.StdId == CAN_CARGO_INFO &&
        RxHeader.DLC >= 6 && RxData[0] == 0x01)
    {
        if (state != STATE_IDLE)
            return;

        received_pin[0] = '0' + RxData[1];
        received_pin[1] = '0' + RxData[2];
        received_pin[2] = '0' + RxData[3];
        received_pin[3] = '0' + RxData[4];
        received_pin[4] = '\0';

        switch (RxData[5]) {
            case 0x01: dest_addr = 'A'; break;
            case 0x02: dest_addr = 'B'; break;
            case 0x03: dest_addr = 'C'; break;
            case 0x04: dest_addr = 'D'; break;
            default:   dest_addr = 'A'; break;
        }
        state             = STATE_PKG_RECEIVED;
        fail_cnt          = 0;
        pin_idx           = 0;
        memset(input_pin, 0, sizeof(input_pin));
        pin_received_flag = 1;
        return;
    }

    if (RxHeader.StdId == CAN_CARGO_ARRIVE && RxData[0] == 0x01)
    {
        if (state == STATE_PKG_RECEIVED) {
            state        = STATE_VERIFY_DEST;
            arrival_flag = 1;
        }
        return;
    }
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI_DIV2;
    RCC_OscInitStruct.PLL.PLLMUL          = RCC_PLL_MUL12;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
        Error_Handler();

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
        Error_Handler();
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}
