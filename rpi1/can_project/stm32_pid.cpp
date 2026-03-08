/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : CAN 키보드 제어 + PID 속도 제어
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
#include <string.h>
#include <stdio.h>
#include <math.h>
/* USER CODE END Includes */

/* USER CODE BEGIN PD */
#define PWM_MAX             999
#define PWM_MIN             0
#define PWM_DEFAULT         500
#define PWM_OFFSET          300.0f
#define PULSE_PER_REV       1980
#define PID_INTERVAL_MS     10

#define TARGET_RPM_MAX      350.0f
#define TARGET_RPM_MIN      0.0f
#define TARGET_RPM_DEFAULT  100.0f

#define KP                  0.5f
#define KI                  0.05f
#define KD                  0.0f
#define INTEGRAL_LIMIT      150.0f

#define PWM_FORWARD         400
#define PWM_TURN            900

#define CAN_ID_CMD          0x010
#define CAN_ID_ESTOP        0x011
#define CAN_ID_SPEED        0x100
#define CAN_ID_HEARTBEAT    0x200

#define DIR_STOP            0
#define DIR_FORWARD         1
#define DIR_BACKWARD        2
#define DIR_LEFT            3
#define DIR_RIGHT           4
#define DIR_UTURN           5

#define CAN_SPEED_PERIOD_MS      50
#define CAN_HEARTBEAT_PERIOD_MS  100
#define CAN_CMD_TIMEOUT_MS       5000
/* USER CODE END PD */

/* USER CODE BEGIN PV */
uint8_t  rx_data;
uint8_t  cmd = 0;
char     uart_buf[160];

volatile int16_t enc_left_prev  = 0;
volatile int16_t enc_right_prev = 0;
volatile float   rpm_left       = 0.0f;
volatile float   rpm_right      = 0.0f;
volatile float   target_rpm     = TARGET_RPM_DEFAULT;

typedef struct {
    float kp, ki, kd;
    float integral;
    float prev_error;
    float output;
} PID_t;

PID_t pid_left  = { KP, KI, KD, 0.0f, 0.0f, 0.0f };
PID_t pid_right = { KP, KI, KD, 0.0f, 0.0f, 0.0f };

volatile uint16_t pwm_out_left  = 0;
volatile uint16_t pwm_out_right = 0;
volatile uint8_t  pid_flag      = 0;
volatile uint8_t  pid_enable    = 0;

volatile uint8_t current_dir = DIR_STOP;

CAN_TxHeaderTypeDef can_tx_header;
CAN_RxHeaderTypeDef can_rx_header;
uint8_t  can_tx_data[8];
uint8_t  can_rx_data[8];
uint32_t can_tx_mailbox;
volatile uint8_t can_rx_flag = 0;

volatile uint16_t can_speed_cnt     = 0;
volatile uint16_t can_heartbeat_cnt = 0;
volatile uint16_t can_timeout_cnt   = 0;
/* USER CODE END PV */

void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void CAN_Config(void);
void CAN_TX_Speed(void);
void CAN_TX_Heartbeat(void);
void CAN_RX_Process(void);
void Motor_Forward(void);
void Motor_Backward(void);
void Motor_Stop(void);
void Motor_Left(void);
void Motor_Right(void);
void Motor_UTurn(void);
void Motor_SetSpeed(uint16_t duty);
void Motor_SetPWM_LR(uint16_t left, uint16_t right);
void Encoder_Update(void);
float PID_Compute(PID_t *pid, float target, float current);
void PID_Reset(PID_t *pid);
void UART_Print(const char *msg);
static inline int16_t diff16(int16_t now, int16_t prev);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

static inline int16_t diff16(int16_t now, int16_t prev)
{
    return (int16_t)(now - prev);
}

float PID_Compute(PID_t *pid, float target, float current)
{
    float error = target - current;
    pid->integral += error * (PID_INTERVAL_MS / 1000.0f);
    if (pid->integral >  INTEGRAL_LIMIT) pid->integral =  INTEGRAL_LIMIT;
    if (pid->integral < -INTEGRAL_LIMIT) pid->integral = -INTEGRAL_LIMIT;
    float derivative = (error - pid->prev_error) / (PID_INTERVAL_MS / 1000.0f);
    pid->prev_error  = error;
    pid->output = PWM_OFFSET
                + (pid->kp * error)
                + (pid->ki * pid->integral)
                + (pid->kd * derivative);
    return pid->output;
}

void PID_Reset(PID_t *pid)
{
    pid->integral   = 0.0f;
    pid->prev_error = 0.0f;
    pid->output     = 0.0f;
}

void Encoder_Update(void)
{
    int16_t left_now  = (int16_t)__HAL_TIM_GET_COUNTER(&htim5);
    int16_t right_now = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
    int16_t left_diff  = diff16(left_now,  enc_left_prev);
    int16_t right_diff = diff16(right_now, enc_right_prev);
    enc_left_prev  = left_now;
    enc_right_prev = right_now;
    rpm_left  = -((float)left_diff  / PULSE_PER_REV) * (60000.0f / PID_INTERVAL_MS);
    rpm_right = -((float)right_diff / PULSE_PER_REV) * (60000.0f / PID_INTERVAL_MS);
}

void Motor_SetSpeed(uint16_t duty)
{
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, duty);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, duty);
}

void Motor_SetPWM_LR(uint16_t left, uint16_t right)
{
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, left);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, right);
}

// ★ PID 켜기 - 직진 시 target_rpm으로 속도 제어
void Motor_Forward(void)
{
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
    if (current_dir != DIR_FORWARD) {
        current_dir = DIR_FORWARD;
        PID_Reset(&pid_left);
        PID_Reset(&pid_right);
        pid_enable = 1;  // ★ PID 켜기
    }
}

void Motor_Backward(void)
{
    if (current_dir != DIR_BACKWARD) {
        current_dir = DIR_BACKWARD;
        pid_enable  = 0;
        PID_Reset(&pid_left);
        PID_Reset(&pid_right);
    }
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);
    Motor_SetPWM_LR(320, 320);
}

void Motor_Stop(void)
{
    pid_enable    = 0;
    current_dir   = DIR_STOP;
    pwm_out_left  = 0;
    pwm_out_right = 0;
    PID_Reset(&pid_left);
    PID_Reset(&pid_right);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
    HAL_GPIO_WritePin(GPIOC,
        GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);
    enc_left_prev  = (int16_t)__HAL_TIM_GET_COUNTER(&htim5);
    enc_right_prev = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
    rpm_left  = 0.0f;
    rpm_right = 0.0f;
}

// 회전은 PID 끄고 고정 PWM 사용
void Motor_Left(void)
{
    if (current_dir != DIR_LEFT) {
        current_dir = DIR_LEFT;
        pid_enable  = 0;
        PID_Reset(&pid_left);
        PID_Reset(&pid_right);
    }
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
    Motor_SetPWM_LR(500, PWM_TURN);
}

void Motor_Right(void)
{
    if (current_dir != DIR_RIGHT) {
        current_dir = DIR_RIGHT;
        pid_enable  = 0;
        PID_Reset(&pid_left);
        PID_Reset(&pid_right);
    }
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);
    Motor_SetPWM_LR(PWM_TURN, 500);
}

// ★ U턴 - 한쪽 전진, 반대쪽 후진으로 제자리 회전
void Motor_UTurn(void)
{
    if (current_dir != DIR_UTURN) {
        current_dir = DIR_UTURN;
        pid_enable  = 0;
        PID_Reset(&pid_left);
        PID_Reset(&pid_right);
    }
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET); // 왼쪽 후진
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);   // 오른쪽 전진
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
    Motor_SetPWM_LR(500, 500);
}

void UART_Print(const char *msg)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)msg, strlen(msg), 100);
}

void CAN_Config(void)
{
    CAN_FilterTypeDef filter;
    filter.FilterBank           = 0;
    filter.FilterMode           = CAN_FILTERMODE_IDMASK;
    filter.FilterScale          = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh         = 0x0000;
    filter.FilterIdLow          = 0x0000;
    filter.FilterMaskIdHigh     = 0x0000;
    filter.FilterMaskIdLow      = 0x0000;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation     = ENABLE;
    filter.SlaveStartFilterBank = 14;
    if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK) Error_Handler();
    if (HAL_CAN_Start(&hcan1) != HAL_OK)                 Error_Handler();
    if (HAL_CAN_ActivateNotification(&hcan1,
            CAN_IT_RX_FIFO0_MSG_PENDING |
            CAN_IT_ERROR | CAN_IT_BUSOFF) != HAL_OK)     Error_Handler();
    UART_Print("[CAN] Init OK\r\n");
}

void CAN_TX_Speed(void)
{
    int16_t  l100 = (int16_t)(rpm_left  * 100.0f);
    int16_t  r100 = (int16_t)(rpm_right * 100.0f);
    uint16_t ccrL = __HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_1);
    uint16_t ccrR = __HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_3);
    can_tx_data[0] = (uint8_t)((l100 >> 8) & 0xFF);
    can_tx_data[1] = (uint8_t)( l100       & 0xFF);
    can_tx_data[2] = (uint8_t)((r100 >> 8) & 0xFF);
    can_tx_data[3] = (uint8_t)( r100       & 0xFF);
    can_tx_data[4] = (uint8_t)((ccrL >> 8) & 0xFF);
    can_tx_data[5] = (uint8_t)( ccrL       & 0xFF);
    can_tx_data[6] = (uint8_t)((ccrR >> 8) & 0xFF);
    can_tx_data[7] = (uint8_t)( ccrR       & 0xFF);
    can_tx_header.StdId              = CAN_ID_SPEED;
    can_tx_header.ExtId              = 0;
    can_tx_header.RTR                = CAN_RTR_DATA;
    can_tx_header.IDE                = CAN_ID_STD;
    can_tx_header.DLC                = 8;
    can_tx_header.TransmitGlobalTime = DISABLE;
    HAL_CAN_AddTxMessage(&hcan1, &can_tx_header, can_tx_data, &can_tx_mailbox);
}

void CAN_TX_Heartbeat(void)
{
    can_tx_data[0] = 0xAA;
    can_tx_header.StdId              = CAN_ID_HEARTBEAT;
    can_tx_header.ExtId              = 0;
    can_tx_header.RTR                = CAN_RTR_DATA;
    can_tx_header.IDE                = CAN_ID_STD;
    can_tx_header.DLC                = 1;
    can_tx_header.TransmitGlobalTime = DISABLE;
    HAL_CAN_AddTxMessage(&hcan1, &can_tx_header, can_tx_data, &can_tx_mailbox);
}

void CAN_RX_Process(void)
{
    uint32_t id = can_rx_header.StdId;

    if (id == CAN_ID_ESTOP) {
        Motor_Stop();
        UART_Print("[CAN] E-Stop\r\n");
        return;
    }

    if (id == CAN_ID_CMD) {
        can_timeout_cnt = 0;
        uint8_t  dir     = can_rx_data[0];
        uint16_t rpm_x10 = ((uint16_t)can_rx_data[1] << 8) | can_rx_data[2];
        float    new_rpm = rpm_x10 / 10.0f;
        if (new_rpm < TARGET_RPM_MIN) new_rpm = TARGET_RPM_MIN;
        if (new_rpm > TARGET_RPM_MAX) new_rpm = TARGET_RPM_MAX;
        if (new_rpm > 0.0f) target_rpm = new_rpm;

        switch (dir) {
            case DIR_FORWARD:
                Motor_Forward();
                snprintf(uart_buf, sizeof(uart_buf),
                         "[CAN] FWD PID:%d TGT:%.1f\r\n", pid_enable, target_rpm);
                break;
            case DIR_BACKWARD:
                Motor_Backward();
                snprintf(uart_buf, sizeof(uart_buf), "[CAN] BWD\r\n");
                break;
            case DIR_LEFT:
                Motor_Left();
                snprintf(uart_buf, sizeof(uart_buf),
                         "[CAN] LEFT PWM:%d\r\n", PWM_TURN);
                break;
            case DIR_RIGHT:
                Motor_Right();
                snprintf(uart_buf, sizeof(uart_buf),
                         "[CAN] RIGHT PWM:%d\r\n", PWM_TURN);
                break;
            case DIR_UTURN:
                Motor_UTurn();
                snprintf(uart_buf, sizeof(uart_buf), "[CAN] UTURN\r\n");
                break;
            case DIR_STOP:
            default:
                Motor_Stop();
                snprintf(uart_buf, sizeof(uart_buf), "[CAN] STOP\r\n");
                break;
        }
        //UART_Print(uart_buf);
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance == CAN1) {
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0,
                                 &can_rx_header, can_rx_data) == HAL_OK) {
            can_rx_flag = 1;
        }
    }
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
    UART_Print("[CAN] ERROR\r\n");
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6) {
        pid_flag = 1;
        if (can_timeout_cnt < 0xFFFF) can_timeout_cnt++;
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        cmd = rx_data;
        HAL_UART_Receive_IT(&huart1, &rx_data, 1);
    }
}

/* USER CODE END 0 */

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_I2C2_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_TIM6_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    HAL_TIM_Base_Start_IT(&htim6);

    enc_left_prev  = (int16_t)__HAL_TIM_GET_COUNTER(&htim5);
    enc_right_prev = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);

    Motor_Stop();
    CAN_Config();
    HAL_UART_Receive_IT(&huart1, &rx_data, 1);

    UART_Print("\r\n=== CAN Motor Control + PID ===\r\n");
    snprintf(uart_buf, sizeof(uart_buf),
             "FWD PWM:%d  TURN PWM:%d  TGT RPM:%.1f  KP:%.2f KI:%.2f\r\n",
             PWM_FORWARD, PWM_TURN, TARGET_RPM_DEFAULT, KP, KI);
    UART_Print(uart_buf);
    UART_Print("Ready.\r\n");
  /* USER CODE END 2 */

    while (1)
    {
        if (can_rx_flag) {
            can_rx_flag = 0;
            CAN_RX_Process();
        }

        if (can_timeout_cnt >= (CAN_CMD_TIMEOUT_MS / PID_INTERVAL_MS)) {
            if (current_dir != DIR_STOP) {
                Motor_Stop();
                UART_Print("[TIMEOUT] STOP\r\n");
            }
        }

        if (pid_flag) {
            pid_flag = 0;
            Encoder_Update();

            if (pid_enable) {
                float out_left  = PID_Compute(&pid_left,  target_rpm, fabsf(rpm_left));
                float out_right = PID_Compute(&pid_right, target_rpm, fabsf(rpm_right));
                if (out_left  < PWM_MIN) out_left  = PWM_MIN;
                if (out_left  > PWM_MAX) out_left  = PWM_MAX;
                if (out_right < PWM_MIN) out_right = PWM_MIN;
                if (out_right > PWM_MAX) out_right = PWM_MAX;
                pwm_out_left  = (uint16_t)out_left;
                pwm_out_right = (uint16_t)out_right;
                Motor_SetPWM_LR(pwm_out_left, pwm_out_right);
            }

            can_speed_cnt += PID_INTERVAL_MS;
            if (can_speed_cnt >= CAN_SPEED_PERIOD_MS) {
                can_speed_cnt = 0;
                CAN_TX_Speed();
            }

            can_heartbeat_cnt += PID_INTERVAL_MS;
            if (can_heartbeat_cnt >= CAN_HEARTBEAT_PERIOD_MS) {
                can_heartbeat_cnt = 0;
                CAN_TX_Heartbeat();
            }

            static uint16_t print_cnt = 0;
            if (++print_cnt >= 100) {
                print_cnt = 0;
                uint16_t ccrL = __HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_1);
                uint16_t ccrR = __HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_3);
                snprintf(uart_buf, sizeof(uart_buf),
                         "DIR:%d PID:%d | L:%.1f RPM PWM:%u | R:%.1f RPM PWM:%u | TGT:%.1f\r\n",
                         current_dir, pid_enable,
                         rpm_left, ccrL, rpm_right, ccrR, target_rpm);
                UART_Print(uart_buf);
            }
        }
    }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK) Error_Handler();
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) Error_Handler();
}

void Error_Handler(void)
{
  __disable_irq();
  while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) {}
#endif
