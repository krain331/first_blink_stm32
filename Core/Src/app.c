/* =============================================================================
 *  Цифровой двойник преобразователя Micro Motion 2700 (STM32F103C8T6)
 *  Сенсор CMF300 эмулируется ESP32-S3: две синусоиды 80 Гц, сдвиг фаз 0..10°.
 *
 *  Обработка (как в реальном 2700):
 *    1. Оба сигнала оцифровываются СИНХРОННО (ADC1+ADC2, Dual regular
 *       simultaneous, запуск от TIM3, 16 кГц).
 *    2. Для каждого канала - комплексная амплитуда на 80 Гц (DFT с окном
 *       Ханна, блок 0,2 с = 16 периодов).
 *    3. Разность фаз dPhi = arg(X2 * conj(X1)), частота - по набегу фазы.
 *    4. dt = dPhi / (2*pi*f);  медиана по 5 блокам (отсев выбросов);
 *       демпфирование;  Qm = FCF * (dt - dt_zero).
 *
 *  Экран - две страницы:
 *    1/2 PROCESS : массовый расход, шкала % от Qmax, сумматор, статус
 *    2/2 SENSOR  : dPhi, dt, частота, амплитуды, размах АЦП, ноль, сбросы
 *
 *  Кнопка KEY (PA0):
 *    короткое нажатие          - следующая страница
 *    удержание 2 с на стр. 1   - сброс сумматора
 *    удержание 2 с на стр. 2   - калибровка нуля (потенциометр ESP32 в минимум!)
 * ===========================================================================*/
#include "app.h"
#include "main.h"
#include "ssd1306.h"
#include <math.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern TIM_HandleTypeDef htim3;
extern I2C_HandleTypeDef hi2c1;

/* ------------------------------ Настройки -------------------------------- */
/* Частота дискретизации: 16 кГц = 200 отсчётов на период 80 Гц */
#ifndef FS_HZ
#define FS_HZ           16000u
#endif
#define TIM3_ARR        (72000000u / FS_HZ - 1u)   /* 4499 */
#define F_REF_HZ        80u
#define SPP             (FS_HZ / F_REF_HZ)     /* 200 отсчётов на период */
#define BLOCK_N         (SPP * 16u)            /* 0,2 с = 16 периодов    */
#define HALF_N          (BLOCK_N / 8u)
#define T_BLOCK_S       ((double)BLOCK_N / (double)FS_HZ)

/* Шкала CMF300: диапазон по сертификату 250..4535 кг/мин.
 * Сдвиг 10° на 80 Гц  <=>  dt = 347,2 мкс  <=>  4535 кг/мин. */
#define QMAX_KG_MIN     4535.0
#define PHI_MAX_DEG     10.0
#define DT_MAX_US       (PHI_MAX_DEG / 360.0 / (double)F_REF_HZ * 1e6)
#define FCF_KGMIN_PER_US (QMAX_KG_MIN / DT_MAX_US)   /* ~13,06 кг/мин на мкс */

#define DAMPING_S       2.0     /* демпфирование расхода, с (в 2700 настраивается) */
#define MEDIAN_N        5       /* медиана по 5 блокам = 1 с: отсев одиночных выбросов */
#define CUTOFF_PCT      0.5     /* отсечка малого расхода, % от Qmax */
#define AMP_MIN_V       0.30    /* ниже - "NO SIGNAL" */
#define CLIP_LEVEL      4090u   /* размах >= этого - сигнал упирается в 0/3,3 В */
#define ZERO_TIME_S     10.0    /* время усреднения при калибровке нуля */
#define LONG_PRESS_MS   2000u
#define DEBOUNCE_MS     30u

/* Кнопка KEY на WeAct BluePill Plus (PA0) замыкает на +3,3 В -> активный HIGH.
 * В CubeMX для PA0 - Pull-down. Для кнопки на GND: 0 и Pull-up. */
#define BTN_ACTIVE_HIGH 1

/* Последняя страница 64-КБ флеша (1 КБ) - хранение нуля */
#define CFG_ADDR        0x0800FC00u
#define CFG_MAGIC       0xC0F1A300u

#define TWO_PI          6.283185307179586

/* ----------------------------- Обработка АЦП ----------------------------- */
/* Dual mode: каждое 32-битное слово = [ADC2 : ADC1] (старшие 16 бит - ADC2) */
static uint32_t adc_buf[2 * HALF_N];

static int16_t cos_t[SPP], sin_t[SPP];        /* Q15 */
static int16_t hann_t[BLOCK_N / 2 + 1];       /* Q15, симметричное окно */

typedef struct {
    int64_t  re1, im1, re2, im2;
    uint32_t sum1, sum2;                 /* для диагностики: среднее, размах */
    uint16_t min1, max1, min2, max2;
} dft_t;

static dft_t    acc;                 /* накопитель текущего блока (только ISR) */
static uint32_t n_idx;               /* номер отсчёта в блоке */
static uint32_t ph_idx;              /* индекс в таблице sin/cos */

static volatile dft_t    blk_out;    /* последний готовый блок */
static volatile uint32_t blk_seq;    /* счётчик готовых блоков */

static void build_tables(void)
{
    for (uint32_t k = 0; k < SPP; k++) {
        double a = TWO_PI * (double)k / (double)SPP;
        cos_t[k] = (int16_t)lrint(32767.0 * cos(a));
        sin_t[k] = (int16_t)lrint(32767.0 * sin(a));
    }
    for (uint32_t n = 0; n <= BLOCK_N / 2; n++) {
        double w = 0.5 - 0.5 * cos(TWO_PI * (double)n / (double)BLOCK_N);
        hann_t[n] = (int16_t)lrint(32767.0 * w);
    }
}

static void acc_reset(void)
{
    memset(&acc, 0, sizeof(acc));
    acc.min1 = acc.min2 = 0xFFFF;
}

/* Вызывается из прерывания DMA: накапливает DFT на частоте 80 Гц.
 * Окно Ханна на целом числе периодов полностью подавляет постоянную
 * составляющую (1,65 В), поэтому её не нужно вычитать. */
static void process_samples(const uint32_t *p, uint32_t cnt)
{
    for (uint32_t i = 0; i < cnt; i++) {
        uint32_t v  = p[i];
        int32_t  s1 = (int32_t)(v & 0x0FFFu);
        int32_t  s2 = (int32_t)((v >> 16) & 0x0FFFu);

        uint32_t wi = (n_idx <= BLOCK_N / 2) ? n_idx : (BLOCK_N - n_idx);
        int32_t  w  = hann_t[wi];
        int32_t  x1 = (s1 * w) >> 12;          /* <= 32767 */
        int32_t  x2 = (s2 * w) >> 12;
        int32_t  c  = cos_t[ph_idx];
        int32_t  s  = sin_t[ph_idx];

        acc.sum1 += (uint32_t)s1;  acc.sum2 += (uint32_t)s2;
        if (s1 < acc.min1) acc.min1 = (uint16_t)s1;
        if (s1 > acc.max1) acc.max1 = (uint16_t)s1;
        if (s2 < acc.min2) acc.min2 = (uint16_t)s2;
        if (s2 > acc.max2) acc.max2 = (uint16_t)s2;

        acc.re1 += (int64_t)(x1 * c);
        acc.im1 -= (int64_t)(x1 * s);
        acc.re2 += (int64_t)(x2 * c);
        acc.im2 -= (int64_t)(x2 * s);

        if (++ph_idx >= SPP) ph_idx = 0;
        if (++n_idx >= BLOCK_N) {
            n_idx = 0;
            ph_idx = 0;
            memcpy((void *)&blk_out, &acc, sizeof(acc));
            blk_seq++;
            acc_reset();
        }
    }
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1) process_samples(&adc_buf[0], HALF_N);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1) process_samples(&adc_buf[HALF_N], HALF_N);
}

/* ------------------------- Хранение нуля во флеше ------------------------ */
typedef struct { uint32_t magic; uint32_t zero_bits; uint32_t inv; } cfg_t;

static double zero_dt_us = 0.0;

static void cfg_load(void)
{
    const cfg_t *c = (const cfg_t *)CFG_ADDR;
    if (c->magic == CFG_MAGIC && c->inv == ~c->zero_bits) {
        float z;
        memcpy(&z, (const void *)&c->zero_bits, sizeof(z));
        if (isfinite(z) && fabsf(z) < 2000.0f) zero_dt_us = z;
    }
}

static bool cfg_save(void)
{
    float z = (float)zero_dt_us;
    uint32_t bits;
    memcpy(&bits, &z, sizeof(bits));
    uint32_t words[3] = { CFG_MAGIC, bits, ~bits };

    FLASH_EraseInitTypeDef er = {0};
    er.TypeErase   = FLASH_TYPEERASE_PAGES;
    er.PageAddress = CFG_ADDR;
    er.NbPages     = 1;
    uint32_t page_err = 0;
    bool ok = true;

    HAL_FLASH_Unlock();
    if (HAL_FLASHEx_Erase(&er, &page_err) != HAL_OK) ok = false;
    for (uint32_t i = 0; ok && i < 3; i++) {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, CFG_ADDR + 4u * i, words[i]) != HAL_OK)
            ok = false;
    }
    HAL_FLASH_Lock();
    return ok;
}

/* ------------------------- Форматирование чисел -------------------------- */
/* Своя функция, чтобы не тащить printf с float (экономия флеша) */
static char *fmt_fixed(char *out, double v, int dec)
{
    static const uint32_t p10[] = { 1, 10, 100, 1000, 10000 };
    if (dec < 0) dec = 0;
    if (dec > 4) dec = 4;
    bool neg = v < 0;
    if (neg) v = -v;
    if (v > 999999.0) v = 999999.0;
    uint32_t sc = (uint32_t)(v * p10[dec] + 0.5);
    if (neg && sc != 0) *out++ = '-';
    uint32_t ip = sc / p10[dec], fp = sc % p10[dec];
    char tmp[12];
    int n = 0;
    do { tmp[n++] = (char)('0' + ip % 10); ip /= 10; } while (ip);
    while (n) *out++ = tmp[--n];
    if (dec) {
        *out++ = '.';
        for (int k = dec - 1; k >= 0; k--) { out[k] = (char)('0' + fp % 10); fp /= 10; }
        out += dec;
    }
    *out = '\0';
    return out;
}

static char *str_add(char *out, const char *s)
{
    while (*s) *out++ = *s++;
    *out = '\0';
    return out;
}

/* ------------------------------ Состояние -------------------------------- */
static uint32_t seen_seq;
static uint32_t skip_blocks = 3;     /* первые блоки - переходный процесс */

static bool   have_prev;
static double prev_re1, prev_im1, prev_re2, prev_im2;

static double freq_hz = F_REF_HZ;
static double dt_raw_us;             /* dt текущего блока */
static double dt_us;                 /* после медианы и демпфирования */
static double dphi_deg;              /* dPhi, пересчитанная из dt_us */
static bool   dt_init;
static double med_buf[MEDIAN_N];
static uint32_t med_cnt, med_pos;
static double amp1_v, amp2_v;
static bool   sig_ok;
static double flow_kgmin;
static double total_kg;

static bool     zeroing;
static uint32_t zero_cnt;
static double   zero_sum;
#define ZERO_BLOCKS ((uint32_t)(ZERO_TIME_S / T_BLOCK_S + 0.5))
static uint32_t zero_msg_until;      /* показать "ZERO SAVED" до этого тика */

static uint8_t  page;                /* 0 - PROCESS, 1 - SENSOR */
#define PAGES 2

/* Диагностика */
static uint32_t avg1, avg2, pp1, pp2;   /* среднее и размах сырых отсчётов, 0..4095 */
static bool     dual_was_off;
static uint16_t boot_cnt;
static const char *reset_cause = "?";
static uint32_t i2c_errs;
static uint32_t sig_lost_cnt;           /* сколько раз пропадал сигнал */

static void start_zero(void)
{
    zeroing  = true;
    zero_cnt = 0;
    zero_sum = 0.0;
}

static void reset_filters(void)
{
    dt_init = false;
    med_cnt = med_pos = 0;
}

/* ------------------------------- Кнопка ---------------------------------- */
static void button_poll(void)
{
    static uint8_t  stable = 1, last_raw = 1;
    static uint32_t t_change, t_press;
    static bool     long_done;

    /* raw = 1 -> отпущена, 0 -> нажата */
    uint8_t  pin = (HAL_GPIO_ReadPin(BTN_GPIO_Port, BTN_Pin) == GPIO_PIN_SET);
    uint8_t  raw = BTN_ACTIVE_HIGH ? (uint8_t)!pin : pin;
    uint32_t now = HAL_GetTick();

    if (raw != last_raw) { last_raw = raw; t_change = now; }

    if ((now - t_change) >= DEBOUNCE_MS && raw != stable) {
        stable = raw;
        if (!stable) {                       /* нажата */
            t_press   = now;
            long_done = false;
        } else if (!long_done) {             /* короткое нажатие - следующая страница */
            page = (uint8_t)((page + 1) % PAGES);
        }
    }
    if (!stable && !long_done && (now - t_press) >= LONG_PRESS_MS) {
        long_done = true;
        if (page == 0) total_kg = 0.0;       /* стр. 1: сброс сумматора */
        else           start_zero();         /* стр. 2: калибровка нуля */
    }
}

/* -------------------------------- Экран ---------------------------------- */
/* Восстановление после ошибки I2C (помеха, плохой контакт на макетке) */
static void display_recover(void)
{
    i2c_errs++;
    HAL_I2C_DeInit(&hi2c1);
    HAL_I2C_Init(&hi2c1);
    ssd1306_init(&hi2c1);
}

static void draw_bar(int y, double frac)
{
    /* рамка 128x6, заполнение = доля от Qmax */
    ssd1306_hline(0, y, 128);
    ssd1306_hline(0, y + 5, 128);
    for (int j = 0; j < 6; j++) { ssd1306_pixel(0, y + j, 1); ssd1306_pixel(127, y + j, 1); }
    if (frac < 0) frac = 0;
    if (frac > 1) frac = 1;
    int w = (int)(frac * 124.0 + 0.5);
    for (int j = 2; j < 4; j++) ssd1306_hline(2, y + j, w);
}

static void draw_page_process(void)
{
    char line[32], *p;

    ssd1306_text(0, 0, "MASS FLOW", 1);
    ssd1306_text_right(127, 0, "kg/min", 1);

    if (zeroing) {
        ssd1306_text_center(11, "ZERO", 3);
    } else if (!sig_ok) {
        ssd1306_text_center(11, "NO SIG", 3);
    } else {
        fmt_fixed(line, flow_kgmin, 1);
        ssd1306_text_right(127, 11, line, 3);
    }

    draw_bar(35, sig_ok ? flow_kgmin / QMAX_KG_MIN : 0.0);

    ssd1306_text(0, 45, "TOTAL", 1);
    p = fmt_fixed(line, total_kg / 1000.0, 3);
    str_add(p, " t");
    ssd1306_text_right(127, 45, line, 1);

    /* строка статуса */
    if (zeroing) {
        p = str_add(line, "ZEROING ");
        p = fmt_fixed(p, (double)(ZERO_BLOCKS - zero_cnt) * T_BLOCK_S, 0);
        str_add(p, "s");
    } else if (HAL_GetTick() < zero_msg_until) {
        str_add(line, "ZERO SAVED");
    } else if (!sig_ok) {
        str_add(line, "SENSOR FAIL");
    } else if (pp1 >= CLIP_LEVEL || pp2 >= CLIP_LEVEL) {
        str_add(line, "SIGNAL CLIP");
    } else if (flow_kgmin == 0.0) {
        str_add(line, "LOW FLOW");
    } else {
        p = fmt_fixed(line, flow_kgmin / QMAX_KG_MIN * 100.0, 1);
        str_add(p, "% Qmax");
    }
    ssd1306_text(0, 56, line, 1);
    ssd1306_text_right(127, 56, "1/2", 1);
}

static void draw_page_sensor(void)
{
    char line[32], *p;

    ssd1306_text(0, 0, "SENSOR", 1);
    ssd1306_text_right(127, 0, "2/2", 1);

    p = str_add(line, "dPhi ");
    p = fmt_fixed(p, dphi_deg, 3);
    str_add(p, "\x7F");
    ssd1306_text(0, 9, line, 1);

    p = str_add(line, "dt   ");
    p = fmt_fixed(p, dt_us, 2);
    str_add(p, " us");
    ssd1306_text(0, 17, line, 1);

    p = str_add(line, "f    ");
    p = fmt_fixed(p, freq_hz, 2);
    str_add(p, " Hz");
    ssd1306_text(0, 25, line, 1);

    p = str_add(line, "A1 ");
    p = fmt_fixed(p, amp1_v, 2);
    p = str_add(p, "V  A2 ");
    p = fmt_fixed(p, amp2_v, 2);
    str_add(p, "V");
    ssd1306_text(0, 33, line, 1);

    p = str_add(line, "pp ");
    p = fmt_fixed(p, (double)pp1, 0);
    p = str_add(p, pp1 >= CLIP_LEVEL ? "! " : "  ");
    p = fmt_fixed(p, (double)pp2, 0);
    str_add(p, pp2 >= CLIP_LEVEL ? "!" : "");
    ssd1306_text(0, 41, line, 1);

    p = str_add(line, "Zero ");
    p = fmt_fixed(p, zero_dt_us, 2);
    str_add(p, " us");
    ssd1306_text(0, 49, line, 1);

    if (zeroing) {
        p = str_add(line, "ZEROING ");
        p = fmt_fixed(p, (double)(ZERO_BLOCKS - zero_cnt) * T_BLOCK_S, 0);
        str_add(p, "s pot=MIN");
    } else {
        p = str_add(line, dual_was_off ? "D* b" : "b");
        p = fmt_fixed(p, (double)boot_cnt, 0);
        p = str_add(p, " ");
        p = str_add(p, reset_cause);
        p = str_add(p, " lost");
        p = fmt_fixed(p, (double)sig_lost_cnt, 0);
        p = str_add(p, " e");
        fmt_fixed(p, (double)i2c_errs, 0);
    }
    ssd1306_text(0, 57, line, 1);
}

static void draw_screen(void)
{
    ssd1306_clear();
    if (page == 0) draw_page_process();
    else           draw_page_sensor();
    if (ssd1306_update() != HAL_OK) display_recover();
}

/* --------------------------- Обработка блока ----------------------------- */
static double median(const double *v, uint32_t n)
{
    double t[MEDIAN_N];
    memcpy(t, v, n * sizeof(double));
    for (uint32_t i = 1; i < n; i++) {            /* вставками, n <= 5 */
        double x = t[i];
        uint32_t j = i;
        while (j > 0 && t[j - 1] > x) { t[j] = t[j - 1]; j--; }
        t[j] = x;
    }
    return (n & 1u) ? t[n / 2] : 0.5 * (t[n / 2 - 1] + t[n / 2]);
}

static void process_block(const dft_t *b, uint32_t dseq)
{
    /* Амплитуда: |X| = A_counts * N/4 * (32767^2 / 4096) */
    const double k_amp = 4.0 * 4096.0 / (32767.0 * 32767.0 * (double)BLOCK_N)
                         * 3.3 / 4095.0;

    double r1 = (double)b->re1, i1 = (double)b->im1;
    double r2 = (double)b->re2, i2 = (double)b->im2;

    avg1 = b->sum1 / BLOCK_N;  avg2 = b->sum2 / BLOCK_N;
    pp1  = (uint32_t)(b->max1 - b->min1);
    pp2  = (uint32_t)(b->max2 - b->min2);

    amp1_v = sqrt(r1 * r1 + i1 * i1) * k_amp;
    amp2_v = sqrt(r2 * r2 + i2 * i2) * k_amp;
    bool was_ok = sig_ok;
    sig_ok = (amp1_v > AMP_MIN_V) && (amp2_v > AMP_MIN_V);
    if (was_ok && !sig_ok) sig_lost_cnt++;

    /* Частота по набегу фазы между соседними блоками */
    if (have_prev && dseq == 1 && sig_ok) {
        double fr = (r1 * prev_re1 + i1 * prev_im1) + (r2 * prev_re2 + i2 * prev_im2);
        double fi = (i1 * prev_re1 - r1 * prev_im1) + (i2 * prev_re2 - r2 * prev_im2);
        double f_new = (double)F_REF_HZ + atan2(fi, fr) * (double)FS_HZ / (TWO_PI * BLOCK_N);
        freq_hz += 0.2 * (f_new - freq_hz);
    }
    prev_re1 = r1; prev_im1 = i1; prev_re2 = r2; prev_im2 = i2;
    have_prev = true;

    if (!sig_ok) {
        flow_kgmin = 0.0;
        reset_filters();
        if (zeroing) zeroing = false;        /* без сигнала ноль не калибруем */
        return;
    }

    /* Разность фаз: канал 2 относительно канала 1 */
    double zr = r2 * r1 + i2 * i1;
    double zi = i2 * r1 - r2 * i1;
    dt_raw_us = atan2(zi, zr) / (TWO_PI * freq_hz) * 1e6;

    /* Калибровка нуля: простое среднее "сырых" dt */
    if (zeroing) {
        zero_sum += dt_raw_us;
        if (++zero_cnt >= ZERO_BLOCKS) {
            zero_dt_us = zero_sum / (double)zero_cnt;
            zeroing = false;
            cfg_save();
            zero_msg_until = HAL_GetTick() + 3000u;
            skip_blocks = 2;                 /* стирание флеша задержало прерывания */
            have_prev = false;
            reset_filters();
        }
        flow_kgmin = 0.0;
        return;
    }

    /* 1) медиана по последним MEDIAN_N блокам - убирает одиночные выбросы */
    med_buf[med_pos] = dt_raw_us;
    med_pos = (med_pos + 1) % MEDIAN_N;
    if (med_cnt < MEDIAN_N) med_cnt++;
    double dt_med = median(med_buf, med_cnt);

    /* 2) демпфирование (экспоненциальный фильтр, постоянная DAMPING_S).
     *    Скачок больше 10 % шкалы проходит сразу - быстрая реакция на ступеньку. */
    double a = T_BLOCK_S / (DAMPING_S + T_BLOCK_S);
    if (!dt_init || fabs(dt_med - dt_us) > 0.10 * DT_MAX_US) {
        dt_us = dt_med;
        dt_init = true;
    } else {
        dt_us += a * (dt_med - dt_us);
    }
    dphi_deg = dt_us * 1e-6 * freq_hz * 360.0;

    /* Массовый расход */
    flow_kgmin = FCF_KGMIN_PER_US * (dt_us - zero_dt_us);
    if (fabs(flow_kgmin) < QMAX_KG_MIN * CUTOFF_PCT / 100.0) flow_kgmin = 0.0;

    /* Сумматор */
    total_kg += flow_kgmin / 60.0 * T_BLOCK_S * (double)dseq;
}

/* ------------------------------ Запуск ----------------------------------- */
static void detect_reset(void)
{
    uint32_t csr = RCC->CSR;
    if      (csr & RCC_CSR_PORRSTF)  reset_cause = "POR";   /* подача питания */
    else if (csr & RCC_CSR_IWDGRSTF) reset_cause = "IWDG";
    else if (csr & RCC_CSR_WWDGRSTF) reset_cause = "WWDG";
    else if (csr & RCC_CSR_SFTRSTF)  reset_cause = "SFT";   /* программный/отладчик */
    else if (csr & RCC_CSR_LPWRRSTF) reset_cause = "LPWR";
    else if (csr & RCC_CSR_PINRSTF)  reset_cause = "PIN";   /* кнопка RST или просадка */
    RCC->CSR |= RCC_CSR_RMVF;

    /* Счётчик перезапусков в backup-регистре (сохраняется при сбросе, не при отключении питания) */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_RCC_BKP_CLK_ENABLE();
    HAL_PWR_EnableBkUpAccess();
    if (reset_cause[0] == 'P' && reset_cause[1] == 'O') BKP->DR1 = 0;
    BKP->DR1 = (uint16_t)(BKP->DR1 + 1);
    boot_cnt = (uint16_t)BKP->DR1;
}

static void show_error(const char *l1, const char *l2)
{
    ssd1306_clear();
    ssd1306_text_center(8, "ERROR", 2);
    ssd1306_text_center(32, l1, 1);
    ssd1306_text_center(44, l2, 1);
    ssd1306_update();
    while (1) {
        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
        HAL_Delay(100);
    }
}

void app_init(void)
{
    char line[24], *p;

    detect_reset();
    build_tables();
    cfg_load();
    acc_reset();

    ssd1306_init(&hi2c1);
    ssd1306_text_center(8,  "MICRO MOTION", 1);
    ssd1306_text_center(20, "2700 TWIN", 2);
    ssd1306_text_center(42, "CMF300  0-4535 kg/min", 1);
    p = str_add(line, "boot ");
    p = fmt_fixed(p, (double)boot_cnt, 0);
    p = str_add(p, "  reset ");
    str_add(p, reset_cause);
    ssd1306_text_center(54, line, 1);
    ssd1306_update();

    /* Проверка режима АЦП: без Dual regular simultaneous второй канал = 0 */
    if ((ADC1->CR1 & ADC_CR1_DUALMOD) != ADC_DUALMODE_REGSIMULT) {
        dual_was_off = true;
        MODIFY_REG(ADC1->CR1, ADC_CR1_DUALMOD, ADC_DUALMODE_REGSIMULT);
    }
    /* ADC2 обязан запускаться программно, иначе HAL откажется стартовать */
    if ((ADC2->CR2 & ADC_CR2_EXTSEL) != ADC_CR2_EXTSEL)
        show_error("ADC2 trigger must be", "Software start");

    if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK) show_error("ADC1 calibration", "failed");
    if (HAL_ADCEx_Calibration_Start(&hadc2) != HAL_OK) show_error("ADC2 calibration", "failed");

    /* ADC2 (ведомый) запускается отдельно: HAL_ADC_Start ставит ему бит
     * EXTTRIG, без которого ведомый не реагирует на запуск от ADC1. */
    if (HAL_ADC_Start(&hadc2) != HAL_OK) show_error("ADC2 start", "failed");
    SET_BIT(ADC2->CR2, ADC_CR2_EXTTRIG);

    /* Частота дискретизации задаётся здесь, независимо от значения в CubeMX */
    __HAL_TIM_SET_PRESCALER(&htim3, 0);
    __HAL_TIM_SET_AUTORELOAD(&htim3, TIM3_ARR);

    /* Затем ADC1 (ведущий) + DMA, они ждут триггера; последним - таймер */
    if (HAL_ADCEx_MultiModeStart_DMA(&hadc1, adc_buf, 2 * HALF_N) != HAL_OK)
        show_error("MultiModeStart_DMA", "failed (check DMA)");
    if (HAL_TIM_Base_Start(&htim3) != HAL_OK) show_error("TIM3 start", "failed");

    HAL_Delay(1500);
}

void app_loop(void)
{
    button_poll();

    if (blk_seq == seen_seq) return;

    dft_t b;
    uint32_t seq;
    __disable_irq();
    memcpy(&b, (const void *)&blk_out, sizeof(b));
    seq = blk_seq;
    __enable_irq();

    uint32_t dseq = seq - seen_seq;
    seen_seq = seq;

    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

    if (skip_blocks) { skip_blocks--; have_prev = false; return; }

    process_block(&b, dseq);
    draw_screen();
}
