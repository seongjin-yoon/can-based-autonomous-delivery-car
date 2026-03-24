/**
 * @file    keypad.c
 * @brief   4x4 Keypad driver — STM32F103RBTx (MangoM32)
 *
 * [핀 변경 from F429]
 *  Row (Output Push-Pull, init HIGH):
 *    R1: PE11 → PC0
 *    R2: PF14 → PC1
 *    R3: PE13 → PC2
 *    R4: PF15 → PC3
 *
 *  Col (Input Pull-Up):
 *    C1: PE9  → PB12
 *    C2: PF13 → PB13
 *    C3: PF12 → PB14
 *    C4: PD15 → PB15
 *
 * [동작 원리]
 *  Row를 하나씩 LOW로 구동 → Col에서 LOW 감지 → 해당 키 반환
 *  Col은 Pull-Up이므로 눌리지 않으면 HIGH
 */
#include "keypad.h"

/* Row: PC0~PC3 (Output) */
static GPIO_TypeDef* ROW_PORT[4] = {GPIOC, GPIOC, GPIOC, GPIOC};
static uint16_t      ROW_PIN[4]  = {GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_3};

/* Col: PB12~PB15 (Input Pull-Up) */
static GPIO_TypeDef* COL_PORT[4] = {GPIOB, GPIOB, GPIOB, GPIOB};
static uint16_t      COL_PIN[4]  = {GPIO_PIN_12, GPIO_PIN_13, GPIO_PIN_14, GPIO_PIN_15};

static const char keymap[4][4] = {
	{'3', '6', '9', '#'}, // row 2
    {'2', '5', '8', '0'}, // row 3
    {'1', '4', '7', '*'}, // row 4
	{'A', 'B', 'C', 'D'}, // row 1
};

char Keypad_Scan(void)
{
    for (int r = 0; r < 4; r++)
    {
        /* 모든 Row HIGH */
        for (int i = 0; i < 4; i++)
            HAL_GPIO_WritePin(ROW_PORT[i], ROW_PIN[i], GPIO_PIN_SET);

        /* 해당 Row만 LOW */
        HAL_GPIO_WritePin(ROW_PORT[r], ROW_PIN[r], GPIO_PIN_RESET);
        HAL_Delay(5);

        /* Col 스캔 */
        for (int c = 0; c < 4; c++)
        {
            if (HAL_GPIO_ReadPin(COL_PORT[c], COL_PIN[c]) == GPIO_PIN_RESET)
            {
                /* 모든 Row 복구 */
                for (int i = 0; i < 4; i++)
                    HAL_GPIO_WritePin(ROW_PORT[i], ROW_PIN[i], GPIO_PIN_SET);
                return keymap[r][c];
            }
        }
    }
    return 0;
}
