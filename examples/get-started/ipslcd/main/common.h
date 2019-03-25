#ifndef __COMMON_H
#define __COMMON_H

/* @NOTE smart conifg */
extern void start_smart_config_main();
extern bool smart_config_over(void);

/* @NOTE lcd */
extern esp_err_t spilcd_init();
extern void lcd_clear32(uint32_t color);

extern uint8_t qrcode[];

#endif
