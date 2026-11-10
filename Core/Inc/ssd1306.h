#ifndef SSD1306_DRV_H
#define SSD1306_DRV_H

#include "main.h"   /* stm32f1xx_hal.h */

#define SSD1306_W     128
#define SSD1306_H     64
#define SSD1306_ADDR  (0x3C << 1)   /* у некоторых модулей 0x3D */

HAL_StatusTypeDef ssd1306_init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef ssd1306_update(void);
void ssd1306_clear(void);
void ssd1306_pixel(int x, int y, int on);
void ssd1306_hline(int x, int y, int w);

/* Шрифт 5x7, scale = 1..4. Символ '\x7F' = знак градуса. */
int  ssd1306_text(int x, int y, const char *s, int scale);        /* возвращает x после текста */
int  ssd1306_text_width(const char *s, int scale);
void ssd1306_text_right(int x_right, int y, const char *s, int scale);
void ssd1306_text_center(int y, const char *s, int scale);

#endif /* SSD1306_DRV_H */
