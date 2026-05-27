/*
 * lcd.h
 *
 *  Created on: 2026年5月26日
 *      Author: 35973
 */

#ifndef INC_LCD_H_
#define INC_LCD_H_


#include "main.h"
#include "spi.h"
#include "gpio.h"

#define LCD_W 128
#define LCD_H 160

#define WHITE   0xFFFF
#define BLACK   0x0000
#define BLUE    0x001F
#define RED     0xF800
#define GREEN   0x07E0
#define YELLOW  0xFFE0

void LCD_GPIO_Init(void);
void LCD_Init(void);
void LCD_Clear(uint16_t color);
void LCD_DrawPoint(uint16_t x, uint16_t y, uint16_t color);
void LCD_ShowChar(uint16_t x, uint16_t y, char chr, uint16_t color, uint16_t backcolor);
void LCD_ShowString(uint16_t x, uint16_t y, char *str, uint16_t color, uint16_t backcolor);

#endif


