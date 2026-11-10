#ifndef APP_H
#define APP_H

/* Цифровой двойник преобразователя Micro Motion 2700 на STM32F103C8T6.
 * Вызывать app_init() один раз после MX_..._Init(), app_loop() — в while(1). */
void app_init(void);
void app_loop(void);

#endif /* APP_H */
