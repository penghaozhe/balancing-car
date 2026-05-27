/*
 * lcd.c
 *
 *  Created on: 2026年5月26日
 *      Author: 35973
 */

#include"lcd.h"
#include "font.h"
#define LCD_CS_LOW()    HAL_GPIO_WritePin(TFT_CS_GPIO_Port, TFT_CS_Pin, GPIO_PIN_RESET)
#define LCD_CS_HIGH()   HAL_GPIO_WritePin(TFT_CS_GPIO_Port, TFT_CS_Pin, GPIO_PIN_SET)

#define LCD_DC_LOW()    HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_RESET)
#define LCD_DC_HIGH()   HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_SET)

#define LCD_RES_LOW()   HAL_GPIO_WritePin(TFT_RES_GPIO_Port, TFT_RES_Pin, GPIO_PIN_RESET)
#define LCD_RES_HIGH()  HAL_GPIO_WritePin(TFT_RES_GPIO_Port, TFT_RES_Pin, GPIO_PIN_SET)

#define LCD_BL_ON()     HAL_GPIO_WritePin(TFT_BL_GPIO_Port, TFT_BL_Pin, GPIO_PIN_SET)
#define LCD_BL_OFF()    HAL_GPIO_WritePin(TFT_BL_GPIO_Port, TFT_BL_Pin, GPIO_PIN_RESET)

static void LCD_WriteCmd(uint8_t cmd)
{
    LCD_DC_LOW();
    LCD_CS_LOW();
    HAL_SPI_Transmit(&hspi4, &cmd, 1, 100);
    LCD_CS_HIGH();
}

static void LCD_WriteData(uint8_t data)
{
    LCD_DC_HIGH();
    LCD_CS_LOW();
    HAL_SPI_Transmit(&hspi4, &data, 1, 100);
    LCD_CS_HIGH();
}

static void LCD_WriteData16(uint16_t data)
{
    uint8_t buf[2];

    buf[0] = data >> 8;
    buf[1] = data & 0xFF;

    LCD_DC_HIGH();
    LCD_CS_LOW();
    HAL_SPI_Transmit(&hspi4, buf, 2, 100);
    LCD_CS_HIGH();
}

static void LCD_SetAddress(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
    LCD_WriteCmd(0x2A);
    LCD_WriteData(0x00);
    LCD_WriteData(x1);
    LCD_WriteData(0x00);
    LCD_WriteData(x2);

    LCD_WriteCmd(0x2B);
    LCD_WriteData(0x00);
    LCD_WriteData(y1);
    LCD_WriteData(0x00);
    LCD_WriteData(y2);

    LCD_WriteCmd(0x2C);
}

void LCD_GPIO_Init(void)
{
    LCD_CS_HIGH();
    LCD_DC_HIGH();
    LCD_RES_HIGH();
    LCD_BL_ON();
}

void LCD_Init(void)
{
    LCD_GPIO_Init();

    LCD_RES_LOW();
    HAL_Delay(50);
    LCD_RES_HIGH();
    HAL_Delay(120);

    LCD_WriteCmd(0x11);
    HAL_Delay(120);

    LCD_WriteCmd(0x3A);
    LCD_WriteData(0x05);

    LCD_WriteCmd(0x36);
    LCD_WriteData(0xC8);

    LCD_WriteCmd(0x29);
    HAL_Delay(20);

    LCD_Clear(BLACK);
}

void LCD_Clear(uint16_t color)
{
    uint32_t i;
    uint8_t data[2];

    data[0] = color >> 8;
    data[1] = color & 0xFF;

    LCD_SetAddress(0, 0, LCD_W - 1, LCD_H - 1);

    LCD_DC_HIGH();
    LCD_CS_LOW();

    for(i = 0; i < LCD_W * LCD_H; i++)
    {
        HAL_SPI_Transmit(&hspi4, data, 2, 100);
    }

    LCD_CS_HIGH();
}

void LCD_DrawPoint(uint16_t x, uint16_t y, uint16_t color)
{
    if(x >= LCD_W || y >= LCD_H)
    {
        return;
    }

    LCD_SetAddress(x, y, x, y);
    LCD_WriteData16(color);
}

void LCD_ShowChar(uint16_t x, uint16_t y, char chr, uint16_t color, uint16_t backcolor)
{
    uint8_t i, j;
    uint8_t line;

    if(chr < ' ' || chr > '~')
    {
        chr = ' ';
    }

    for(i = 0; i < 5; i++)
    {
        line = Font5x7[chr - ' '][i];

        for(j = 0; j < 7; j++)
        {
            if(line & 0x01)
            {
                LCD_DrawPoint(x + i, y + j, color);
            }
            else
            {
                LCD_DrawPoint(x + i, y + j, backcolor);
            }

            line >>= 1;
        }
    }

    /* ??????? */
    for(j = 0; j < 7; j++)
    {
        LCD_DrawPoint(x + 5, y + j, backcolor);
    }
}

void LCD_ShowString(uint16_t x, uint16_t y, char *str, uint16_t color, uint16_t backcolor)
{
    while(*str != '\0')
    {
        LCD_ShowChar(x, y, *str, color, backcolor);

        x += 6;

        if(x + 6 > LCD_W)
        {
            x = 0;
            y += 8;
        }

        if(y + 8 > LCD_H)
        {
            break;
        }

        str++;
    }
}
