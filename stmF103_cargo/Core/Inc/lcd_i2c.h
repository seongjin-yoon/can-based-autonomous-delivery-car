#ifndef LCD_I2C_H
#define LCD_I2C_H

/* F103용 HAL 헤더로 변경 */
#include "stm32f1xx_hal.h"

#define LCD_ADDR (0x27 << 1)

void LCD_Init(I2C_HandleTypeDef *hi2c);
void LCD_Clear(void);
void LCD_SetCursor(uint8_t row, uint8_t col);
void LCD_Print(char *str);

#endif
