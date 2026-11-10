/* Минимальный драйвер SSD1306 128x64 по I2C (HAL), кадровый буфер 1 КБ. */
#include "ssd1306.h"
#include "font5x7.h"
#include <string.h>

static I2C_HandleTypeDef *s_i2c;
static uint8_t s_fb[SSD1306_W * SSD1306_H / 8];

static HAL_StatusTypeDef send_cmds(const uint8_t *c, uint16_t n)
{
    /* Управляющий байт 0x00 = далее поток команд */
    return HAL_I2C_Mem_Write(s_i2c, SSD1306_ADDR, 0x00, I2C_MEMADD_SIZE_8BIT,
                             (uint8_t *)c, n, 50);
}

HAL_StatusTypeDef ssd1306_init(I2C_HandleTypeDef *hi2c)
{
    static const uint8_t init_seq[] = {
        0xAE,             /* display off */
        0xD5, 0x80,       /* clock divide */
        0xA8, 0x3F,       /* multiplex 64 */
        0xD3, 0x00,       /* display offset */
        0x40,             /* start line 0 */
        0x8D, 0x14,       /* charge pump on */
        0x20, 0x00,       /* horizontal addressing */
        0xA1,             /* segment remap */
        0xC8,             /* COM scan dec */
        0xDA, 0x12,       /* COM pins */
        0x81, 0xCF,       /* contrast */
        0xD9, 0xF1,       /* precharge */
        0xDB, 0x40,       /* VCOMH */
        0xA4,             /* resume to RAM */
        0xA6,             /* normal (не инверсия) */
        0x2E,             /* scroll off */
        0xAF              /* display on */
    };
    s_i2c = hi2c;
    HAL_Delay(100);       /* дисплею нужно время после подачи питания */
    HAL_StatusTypeDef st = send_cmds(init_seq, sizeof(init_seq));
    ssd1306_clear();
    if (st == HAL_OK) st = ssd1306_update();
    return st;
}

HAL_StatusTypeDef ssd1306_update(void)
{
    static const uint8_t window[] = { 0x21, 0, SSD1306_W - 1, 0x22, 0, (SSD1306_H / 8) - 1 };
    HAL_StatusTypeDef st = send_cmds(window, sizeof(window));
    if (st != HAL_OK) return st;
    /* Управляющий байт 0x40 = далее данные GRAM */
    return HAL_I2C_Mem_Write(s_i2c, SSD1306_ADDR, 0x40, I2C_MEMADD_SIZE_8BIT,
                             s_fb, sizeof(s_fb), 100);
}

void ssd1306_clear(void)
{
    memset(s_fb, 0, sizeof(s_fb));
}

void ssd1306_pixel(int x, int y, int on)
{
    if (x < 0 || x >= SSD1306_W || y < 0 || y >= SSD1306_H) return;
    uint8_t *p = &s_fb[x + (y >> 3) * SSD1306_W];
    uint8_t m = (uint8_t)(1u << (y & 7));
    if (on) *p |= m; else *p &= (uint8_t)~m;
}

void ssd1306_hline(int x, int y, int w)
{
    for (int i = 0; i < w; i++) ssd1306_pixel(x + i, y, 1);
}

static int draw_char(int x, int y, char ch, int scale)
{
    unsigned char c = (unsigned char)ch;
    if (c < 0x20 || c > 0x7F) c = '?';
    const uint8_t *g = font5x7[c - 0x20];
    for (int col = 0; col < 5; col++) {
        uint8_t bits = g[col];
        for (int row = 0; row < 7; row++) {
            if (bits & (1u << row)) {
                for (int dx = 0; dx < scale; dx++)
                    for (int dy = 0; dy < scale; dy++)
                        ssd1306_pixel(x + col * scale + dx, y + row * scale + dy, 1);
            }
        }
    }
    return x + 6 * scale;
}

int ssd1306_text(int x, int y, const char *s, int scale)
{
    while (*s) x = draw_char(x, y, *s++, scale);
    return x;
}

int ssd1306_text_width(const char *s, int scale)
{
    int n = (int)strlen(s);
    return n ? (n * 6 - 1) * scale : 0;
}

void ssd1306_text_right(int x_right, int y, const char *s, int scale)
{
    ssd1306_text(x_right - ssd1306_text_width(s, scale) + 1, y, s, scale);
}

void ssd1306_text_center(int y, const char *s, int scale)
{
    ssd1306_text((SSD1306_W - ssd1306_text_width(s, scale)) / 2, y, s, scale);
}
