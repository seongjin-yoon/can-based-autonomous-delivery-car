/**
 * @file    lcd_i2c.c
 * @brief   16x2 LCD I2C driver (PCF8574 backpack)
 *
 * [변경 사항 from F429]
 *  - #include "stm32f4xx_hal.h" → lcd_i2c.h 내부에서 stm32f1xx_hal.h 사용
 *  - 로직 동일 (I2C 핀 PB10/PB11 동일)
 */
#include "lcd_i2c.h"

static I2C_HandleTypeDef *_hi2c;

#define LCD_BACKLIGHT 0x08
#define ENABLE        0x04

static void LCD_SendNibble(uint8_t nibble, uint8_t mode)
{
    uint8_t data = nibble | mode | LCD_BACKLIGHT;
    HAL_I2C_Master_Transmit(_hi2c, LCD_ADDR, &data, 1, 10);
    data |= ENABLE;
    HAL_I2C_Master_Transmit(_hi2c, LCD_ADDR, &data, 1, 10);
    HAL_Delay(1);
    data &= ~ENABLE;
    HAL_I2C_Master_Transmit(_hi2c, LCD_ADDR, &data, 1, 10);
}

static void LCD_SendByte(uint8_t byte, uint8_t mode)
{
    LCD_SendNibble(byte & 0xF0, mode);
    LCD_SendNibble((byte << 4) & 0xF0, mode);
}

void LCD_Init(I2C_HandleTypeDef *hi2c)
{
    _hi2c = hi2c;
    HAL_Delay(50);
    LCD_SendNibble(0x30, 0); HAL_Delay(5);
    LCD_SendNibble(0x30, 0); HAL_Delay(1);
    LCD_SendNibble(0x30, 0); HAL_Delay(1);
    LCD_SendNibble(0x20, 0);
    LCD_SendByte(0x28, 0);
    LCD_SendByte(0x0C, 0);
    LCD_SendByte(0x06, 0);
    LCD_SendByte(0x01, 0);
    HAL_Delay(2);
}

void LCD_Clear(void)
{
    LCD_SendByte(0x01, 0);
    HAL_Delay(5);
}

void LCD_SetCursor(uint8_t row, uint8_t col)
{
    uint8_t addr = (row == 0) ? 0x80 + col : 0xC0 + col;
    LCD_SendByte(addr, 0);
}

void LCD_Print(char *str)
{
    while (*str)
        LCD_SendByte(*str++, 0x01);
}
