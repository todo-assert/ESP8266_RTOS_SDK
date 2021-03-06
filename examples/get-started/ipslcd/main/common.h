#ifndef __COMMON_H
#define __COMMON_H

/* @NOTE smart conifg */
extern void start_smart_config_main();
extern bool smart_config_over(void);

/* @NOTE lcd */
extern void start_sntp_main();

/* @NOTE lcd */
extern esp_err_t spilcd_init();
extern void lcd_clear32(uint32_t color);
extern void lcd_print(uint16_t x, uint16_t y, char *str, uint16_t color);
extern void draw_background(const uint8_t []);
extern void draw_background_anywhere(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, const uint8_t table[]);

extern const uint8_t qrcode[];
extern const uint8_t background[];
extern const uint8_t cloudy[];
extern const uint8_t overcast[];
extern const uint8_t rain[];
extern const uint8_t snow[];
extern const uint8_t sunny[];
extern const uint8_t thunder[];

#endif
