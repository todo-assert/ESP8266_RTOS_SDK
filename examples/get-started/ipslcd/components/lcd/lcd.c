
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp8266/gpio_struct.h"
#include "esp8266/spi_struct.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_libc.h"

#include "driver/gpio.h"
#include "driver/spi.h"
#include "esp8266/gpio_register.h"
#include "esp8266/pin_mux_register.h"

#include "driver/pwm.h"

extern uint8_t qrcode[];

#define LCD_HOR 240
#define LCD_VER 240


#define LCD_DC_PIN     12
#define LCD_RST_PIN    16
#define LCD_BACKLIGHT_PIN 15

#define LCD_PIN_SEL  (1ULL<<LCD_DC_PIN) | (1ULL<<LCD_RST_PIN) | (1ULL<<LCD_BACKLIGHT_PIN)
#define LCD_BACKLIGHT_PWM   15
#define LCD_BACKLIGHT_PWM_PERIOD    (500) // 500us

#define TAG "ips-lcd"

uint32_t lcd_backlight_duty = 180;

static uint32_t sendbuf;
spi_trans_t trans_color = {
    .mosi = &sendbuf,
    .bits.mosi = 32,
};

// PIN8 --- GPIO16 / Deep-Sleep Wakeup
// PIN9 --- GPIO14 / HSPICLK
// PIN10 --- GPIO12 / HSPIQ
// PIN12 --- GPIO13 / HSPID
// PIN13 --- GPIO15 / HSPICS / I2S_BCK
// PIN14 --- GPIO2 / UART TX during flash programming / I2S_WS
// PIN15 --- GPIO0 / SPI_CS2
// PIN16 --- GPIO4
// PIN18 --- SPIHD / HSPIHD / GPIO9
// PIN19 --- SPIWP / HSPIWP / GPIO10
// PIN20 --- SPI_CS0 / GPIO11
// PIN21 --- SPI_CLK / GPIO6 / SD_CLK
// PIN22 --- SPI_MSIO / GPIO7 / SD_D0
// PIN23 --- SPI_MOSI / GPIO8 / SD_D1
// PIN24 --- GPIO5
// PIN25 --- GPIO3 / UART RX during flash programming / U0RXD / I2S_DATA
// PIN26 --- GPIO1 / SPI_CS1 / U0TXD

// uint8_t framebuffer[LCD_HOR][LCD_VER];

static esp_err_t lcd_delay_ms(uint32_t time)
{
    vTaskDelay(time / portTICK_RATE_MS);
    return ESP_OK;
}

static esp_err_t lcd_set_dc(uint8_t dc)
{
    if (dc) {
        GPIO.out_w1ts |= (0x1 << LCD_DC_PIN);
    } else {
        GPIO.out_w1tc |= (0x1 << LCD_DC_PIN);
    }
    return ESP_OK;
}

static esp_err_t lcd_reset()
{
    gpio_set_level(LCD_RST_PIN, 0);
    lcd_delay_ms(100);
    gpio_set_level(LCD_RST_PIN, 1);
    lcd_delay_ms(100);
    return ESP_OK;
}

// Write an 8-bit cmd
static esp_err_t lcd_write_cmd(uint8_t cmd)
{
    uint32_t buf = cmd << 24;
    spi_trans_t trans = {0};
    trans.mosi = &buf;
    trans.bits.mosi = 8;
    lcd_set_dc(0);
    spi_trans(HSPI_HOST, trans);
    return ESP_OK;
}
// Write an 8-bit data
static esp_err_t lcd_write_byte(uint8_t data)
{
    uint32_t buf = data << 24;
    spi_trans_t trans = {0};
    trans.mosi = &buf;
    trans.bits.mosi = 8;
    lcd_set_dc(1);
    spi_trans(HSPI_HOST, trans);
    return ESP_OK;
}

static esp_err_t lcd_write_color(uint16_t data)
{
    sendbuf = data << 16;
    spi_trans(HSPI_HOST, trans_color);
    return ESP_OK;
    uint32_t buf = data << 16;
    spi_trans_t trans = {0};
    trans.mosi = &buf;
    trans.bits.mosi = 16;
    lcd_set_dc(1);
    spi_trans(HSPI_HOST, trans);
    return ESP_OK;
}

static esp_err_t lcd_write_32bit(uint32_t data)
{
    int x, y;

    // Waiting for an incomplete transfer
    while (SPI1.cmd.usr);

    // ENTER_CRITICAL();

    // Set the cmd length and transfer cmd
	SPI1.user.usr_command = 0;
    // Set addr length and transfer addr
	SPI1.user.usr_addr = 0;

    // Set mosi length and transmit mosi
	SPI1.user.usr_mosi = 1;
	SPI1.user1.usr_mosi_bitlen = 511; // 31;
	for(x=0;x<16;x++) {
		SPI1.data_buf[x] = data;
	}

    // Set the length of the miso
	SPI1.user.usr_miso = 0;

    // Call the event callback function to send a transfer start event

    // Start transmission
    SPI1.cmd.usr = 1;

    // Receive miso data
	while (SPI1.cmd.usr);

	for (x = 0; x < trans_color.bits.miso; x += 32) {
		y = x / 32;
		trans_color.miso[y] = SPI1.data_buf[y];
	}

    return ESP_OK;
}

__attribute__((unused)) static esp_err_t lcd_write_32bit_none(void)
{
    int x, y;

    // Waiting for an incomplete transfer
    while (SPI1.cmd.usr);

    // ENTER_CRITICAL();

    // Set the cmd length and transfer cmd
	SPI1.user.usr_command = 0;
    // Set addr length and transfer addr
	SPI1.user.usr_addr = 0;

    // Set mosi length and transmit mosi
	SPI1.user.usr_mosi = 1;
	SPI1.user1.usr_mosi_bitlen = 31; // 31;
	SPI1.data_buf[0] = trans_color.mosi[0];

    // Set the length of the miso
	SPI1.user.usr_miso = 0;

    // Call the event callback function to send a transfer start event

    // Start transmission
    SPI1.cmd.usr = 1;

    // Receive miso data
	while (SPI1.cmd.usr);

	for (x = 0; x < trans_color.bits.miso; x += 32) {
		y = x / 32;
		trans_color.miso[y] = SPI1.data_buf[y];
	}

    return ESP_OK;
}


void lcd_set_backlight(uint32_t light)
{
	lcd_backlight_duty = light;
	pwm_set_duty(1, light);
}
/*
static void backlight_listen_thread(void *pvParameters)
{
	while(1) {
		pwm_set_period(LCD_BACKLIGHT_PWM_PERIOD);
		// pwm_set_duty(1, lcd_backlight_duty);
		lcd_delay_ms(300);
	}
}
*/
void lcd_peripheral_init (void)
{
	/* init gpio */
#if 1
	gpio_config_t io_conf;
	io_conf.intr_type = GPIO_INTR_DISABLE;
	io_conf.mode = GPIO_MODE_OUTPUT;
	io_conf.pin_bit_mask = LCD_PIN_SEL; // (1ULL << LCD_DC_PIN) | (1ULL << LCD_RST_PIN); // |(1ULL << LCD_BACKLIGHT_PWM_0);
	io_conf.pull_down_en = 0;
	io_conf.pull_up_en = 1;
	gpio_config(&io_conf);
	gpio_set_level(LCD_BACKLIGHT_PIN, 0);
#endif
	// gpio_set_level(LCD_BACKLIGHT_PWM_0, 1);
	/* init pwm */
#if 0
	uint32_t lcd_backlight = LCD_BACKLIGHT_PWM;
	pwm_init(LCD_BACKLIGHT_PWM_PERIOD, &lcd_backlight_duty, 1, &lcd_backlight);
	pwm_set_channel_invert(0x1 << 0);
	pwm_start();
#endif
	/* init spi */
	spi_config_t spi_config;
	spi_config.interface.val = SPI_DEFAULT_INTERFACE;
	spi_config.intr_enable.val = SPI_MASTER_DEFAULT_INTR_ENABLE;
	spi_config.interface.cs_en = 0;
	spi_config.interface.miso_en = 0;
	spi_config.interface.cpol = 1;
	spi_config.interface.cpha = 1;
	spi_config.mode = SPI_MASTER_MODE;
	spi_config.clk_div = SPI_40MHz_DIV;
	spi_config.event_cb = NULL; // spi_event_callback;
	spi_init(HSPI_HOST, &spi_config);
}

void lcd_set_position(unsigned int x1,unsigned int y1,unsigned int x2,unsigned int y2)
{ 
	lcd_write_cmd(0x2a);
    lcd_set_dc(1);
	lcd_write_32bit(x1<<16|x2);
	// lcd_write_byte(x1>>8);
	// lcd_write_byte(x1);
	// lcd_write_byte(x2>>8);
	// lcd_write_byte(x2);

	lcd_write_cmd(0x2b);
    lcd_set_dc(1);
	lcd_write_32bit(y1<<16|y2);
	// lcd_write_byte(y1>>8);
	// lcd_write_byte(y1);
	// lcd_write_byte(y2>>8);
	// lcd_write_byte(y2);
	lcd_write_cmd(0x2c);					 						 
}

void lcd_clear(uint16_t color)
{
    uint16_t i,j;  	
    lcd_set_position(0,0,LCD_HOR-1,LCD_VER-1);
    lcd_set_dc(1);
    trans_color.bits.mosi = 16;
    for(i=0;i<LCD_HOR;i++)
	{
		for (j=0;j<LCD_VER;j++)
		{
			lcd_write_color(color);	 			 
		}
	}
}

void draw_qrcode(void)
#if 0
{
	uint32_t ofsbit = qrcode[0xd] << 24 | qrcode[0xc] << 16 | qrcode[0xb] << 8 | qrcode[0xa];
	size_t siz = (qrcode[5] << 24 | qrcode[4] << 16 | qrcode[3] << 8 | qrcode[2]) - ofsbit;
	uint8_t *start = (uint8_t *)&qrcode[ofsbit];
	int i, j;
	trans_color.bits.mosi = 16;
	for(i=0;i<siz;i++) {
		if( i%20 == 0 ) {
			lcd_set_position(40,200-i/20,199,200-i/20);
			// lcd_set_position(200-i/20,40,200,200);
			lcd_set_dc(1);
		}
		for(j=7;j>=0;j--) {
			if( start[i] & (1<<j) )
				lcd_write_color(0xffff);
			else
				lcd_write_color(0);
		}
	}
}
#else
{
	uint32_t ofsbit = qrcode[0xd] << 24 | qrcode[0xc] << 16 | qrcode[0xb] << 8 | qrcode[0xa];
	size_t siz = (qrcode[3] << 8 | qrcode[2]) - ofsbit;
	uint8_t *start = &qrcode[ofsbit];
	int i, j;
	trans_color.bits.mosi = 32;
	int x, y;
	for(i=0;i<siz;i+=4) {
		if( i%20 == 0 ) {
			lcd_set_position(40,200-i/20,199,200-i/20);
			lcd_set_dc(1);
		}
		while (SPI1.cmd.usr);
		SPI1.user.usr_command = 0;
		SPI1.user.usr_addr = 0;
		SPI1.user.usr_mosi = 1;
		SPI1.user1.usr_mosi_bitlen = 511;
		for(x=0;x<4;x++) {
			for(j=6;j>=0;j-=2) {
				SPI1.data_buf[4*x+(6-j)/2] = (start[i+x] & (1<<(j+1)) ? 0xffff<<16 : 0) | (start[i+x] & (1<<(j+0)) ? 0xffff : 0);
			}
			
		}
		SPI1.user.usr_miso = 0;
		SPI1.cmd.usr = 1;
		while (SPI1.cmd.usr);

		for (x = 0; x < trans_color.bits.miso; x += 32) {
			y = x / 32;
			trans_color.miso[y] = SPI1.data_buf[y];
		}
	}
}
#endif

extern const uint8_t background[];
void draw_background(const uint8_t table[])
#if 1
{
	uint32_t ofsbit = table[0xd] << 24 | table[0xc] << 16 | table[0xb] << 8 | table[0xa];
	size_t siz = (table[5] << 24 | table[4] << 16 | table[3] << 8 | table[2]) - ofsbit;
	uint8_t const *start = (uint8_t *)&table[ofsbit];
	int i;
	trans_color.bits.mosi = 32;
	union _double_color_t {
		uint32_t data;
		struct {
			uint32_t r0:5;
			uint32_t g0:6;
			uint32_t b0:5;
			uint32_t r1:5;
			uint32_t g1:6;
			uint32_t b1:5;
		};
	};
	int x, y;
	union _double_color_t *p;
	for(i=0;i<siz;i+=90) {
		if( i%720 == 0 ) { // 240*3
			lcd_set_position(0,239-i/720,239,239-i/720);
			lcd_set_dc(1);
		}
		// Waiting for an incomplete transfer
		while (SPI1.cmd.usr);
		SPI1.user.usr_command = 0;
		SPI1.user.usr_addr = 0;
		SPI1.user.usr_mosi = 1;
		SPI1.user1.usr_mosi_bitlen = 479; // 15*32; // 31;
		for(x=0;x<15;x++) {
			p = (union _double_color_t *)&SPI1.data_buf[x];
			p->r1 = start[i+6*x+0] >> 3;
			p->g1 = start[i+6*x+1] >> 2;
			p->b1 = start[i+6*x+2] >> 3;
			p->r0 = start[i+6*x+3] >> 3;
			p->g0 = start[i+6*x+4] >> 2;
			p->b0 = start[i+6*x+5] >> 3;
		}
		SPI1.user.usr_miso = 0;
		SPI1.cmd.usr = 1;
		while (SPI1.cmd.usr);

		for (x = 0; x < trans_color.bits.miso; x += 32) {
			y = x / 32;
			trans_color.miso[y] = SPI1.data_buf[y];
		}
		// lcd_write_32bit_none();
	}
}
#else
{
	uint32_t ofsbit = table[0xd] << 24 | table[0xc] << 16 | table[0xb] << 8 | table[0xa];
	size_t siz = (table[5] << 24 | table[4] << 16 | table[3] << 8 | table[2]) - ofsbit;
	uint8_t *start = (uint8_t *)&table[ofsbit];
	int i;
	trans_color.bits.mosi = 32;
	union _double_color_t {
		uint32_t data;
		struct {
			uint32_t r1:5;
			uint32_t g1:6;
			uint32_t b1:5;
			uint32_t r0:5;
			uint32_t g0:6;
			uint32_t b0:5;
		};
	};
	union _double_color_t *p = (union _double_color_t *)&sendbuf;
	for(i=0;i<siz;i+=6) {
		if( i%720 == 0 ) { // 240*3
			lcd_set_position(0,240-i/720,239,240-i/720);
			// lcd_set_position(200-i/20,40,200,200);
			lcd_set_dc(1);
		}
		p->r0 = start[i+0] >> 3;
		p->g0 = start[i+1] >> 2;
		p->b0 = start[i+2] >> 3;
		p->r1 = start[i+3] >> 3;
		p->g1 = start[i+4] >> 2;
		p->b1 = start[i+5] >> 3;
		lcd_write_32bit_none();
	}
}
#endif

#if 1
void draw_background_anywhere(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, const uint8_t table[])
{
	uint32_t ofsbit = table[0xd] << 24 | table[0xc] << 16 | table[0xb] << 8 | table[0xa];
	// size_t siz = (table[5] << 24 | table[4] << 16 | table[3] << 8 | table[2]) - ofsbit;
	uint8_t const *start = (uint8_t *)&table[ofsbit];
	union _double_color_t {
		uint32_t data;
		struct {
			uint32_t r0:5;
			uint32_t g0:6;
			uint32_t b0:5;
			uint32_t r1:5;
			uint32_t g1:6;
			uint32_t b1:5;
		};
	};
	int c,r;
	int x, y;
	lcd_set_position(x0, y0, x1, y1);
	lcd_set_dc(1);
#define SPI_TRANS16WORDS
#ifdef SPI_TRANS16WORDS
	trans_color.bits.mosi = 32;
#else
	trans_color.bits.mosi = 16;
#endif
	union _double_color_t dcolor;
	for(r=y0;r<=y1;r++) {
#ifdef SPI_TRANS16WORDS
		while (SPI1.cmd.usr);
		SPI1.user.usr_command = 0;
		SPI1.user.usr_addr = 0;
		SPI1.user.usr_mosi = 1; 
#endif
		x = 0;
		for(c=x0;c<=x1;c++) { // max 16 words
			y = ((LCD_VER-r-1)*240+c)*3;
#ifdef SPI_TRANS16WORDS
			if( x%2 != 0 ) {
				dcolor.r0 = start[y + 0] >> 3;
				dcolor.g0 = start[y + 1] >> 2;
				dcolor.b0 = start[y + 2] >> 3;
				SPI1.data_buf[x/2] = dcolor.data;
			} else {
				dcolor.r1 = start[y + 0] >> 3;
				dcolor.g1 = start[y + 1] >> 2;
				dcolor.b1 = start[y + 2] >> 3;
			}
			if(x == 0) {
				if( x1 - c < 31 ) {
					SPI1.user1.usr_mosi_bitlen = (x1-c+1)*16-1;
				} else {
					SPI1.user1.usr_mosi_bitlen = 511;
				}
			}
			
			if( x == 31 ) {
				x = 0;
				SPI1.user.usr_miso = 0;
				SPI1.cmd.usr = 1;
				while (SPI1.cmd.usr);
				for (x = 0; x < trans_color.bits.miso; x += 32) {
					y = x / 32;
					trans_color.miso[y] = SPI1.data_buf[y];
				}
			} else {
				x++;
			}
#else
			dcolor.r0 = start[y + 0] >> 3;
			dcolor.g0 = start[y + 1] >> 2;
			dcolor.b0 = start[y + 2] >> 3;
			dcolor.r1 = start[y + 0] >> 3;
			dcolor.g1 = start[y + 1] >> 2;
			dcolor.b1 = start[y + 2] >> 3;
			lcd_write_color(dcolor.data&0xffff);
#endif
		}
		if( x != 0 ) {
			// dcolor.r0 = start[y + 0] >> 3;
			// dcolor.g0 = start[y + 1] >> 2;
			// dcolor.b0 = start[y + 2] >> 3;
			SPI1.data_buf[(x-1)/2] = dcolor.data;
			x = 0;
			// SPI1.user1.usr_mosi_bitlen = x*16-1;
			SPI1.user.usr_miso = 0;
			SPI1.cmd.usr = 1;
			while (SPI1.cmd.usr);
			for (x = 0; x < trans_color.bits.miso; x += 32) {
				y = x / 32;
				trans_color.miso[y] = SPI1.data_buf[y];
			}
		}
	}
}
#endif

void lcd_clear32(uint32_t color)
{
	uint16_t i,j;  	
	lcd_set_position(0,0,LCD_HOR-1,LCD_VER-1);
	trans_color.bits.mosi = 32;
    lcd_set_dc(1);
    for(i=0;i<LCD_HOR/2;i++)
	{
		for (j=0;j<LCD_VER/16;j++)
		{
			lcd_write_32bit(color);	 			 
		}
	}
}


// esp_err_t lcd_update(void) 
// {
	// uint16_t i, j;
	// union {
		// uint16_t data;
		// struct {
			// uint16_t r:2;
			// uint16_t r_resv:3;
			// uint16_t g:4;
			// uint16_t g_resv:2;
			// uint16_t b:2;
			// uint16_t b_resv:3;
		// };
	// } color;
	// lcd_set_position(0,0,LCD_HOR-1,LCD_VER-1);
    // for(i=0;i<LCD_HOR;i++)
	// {
		// for (j=0;j<LCD_VER;j++)
		// {
			// color.r = framebuffer[i][j] >> 6;
			// color.g = (framebuffer[i][j] >> 2) & 0xf;
			// color.r = framebuffer[i][j] & 0x3;
			// lcd_write_color(color.data);	 			 
		// }
	// }
    // return ESP_OK;
// }

esp_err_t spilcd_init()
{
// extern void initspi(void);
	// initspi();
	lcd_peripheral_init();
	lcd_reset();
	
//************* Start Initial Sequence **********// 
	lcd_write_cmd(0x36); 
	lcd_write_byte(0x00);

	lcd_write_cmd(0x3A); 
	lcd_write_byte(0x05);

	lcd_write_cmd(0xB2);
	lcd_write_byte(0x0C);
	lcd_write_byte(0x0C);
	lcd_write_byte(0x00);
	lcd_write_byte(0x33);
	lcd_write_byte(0x33);

	lcd_write_cmd(0xB7); 
	lcd_write_byte(0x35);  

	lcd_write_cmd(0xBB);
	lcd_write_byte(0x19);

	lcd_write_cmd(0xC0);
	lcd_write_byte(0x2C);

	lcd_write_cmd(0xC2);
	lcd_write_byte(0x01);

	lcd_write_cmd(0xC3);
	lcd_write_byte(0x12);   

	lcd_write_cmd(0xC4);
	lcd_write_byte(0x20);  

	lcd_write_cmd(0xC6); 
	lcd_write_byte(0x0F);    

	lcd_write_cmd(0xD0); 
	lcd_write_byte(0xA4);
	lcd_write_byte(0xA1);

	lcd_write_cmd(0xE0);
	lcd_write_byte(0xD0);
	lcd_write_byte(0x04);
	lcd_write_byte(0x0D);
	lcd_write_byte(0x11);
	lcd_write_byte(0x13);
	lcd_write_byte(0x2B);
	lcd_write_byte(0x3F);
	lcd_write_byte(0x54);
	lcd_write_byte(0x4C);
	lcd_write_byte(0x18);
	lcd_write_byte(0x0D);
	lcd_write_byte(0x0B);
	lcd_write_byte(0x1F);
	lcd_write_byte(0x23);

	lcd_write_cmd(0xE1);
	lcd_write_byte(0xD0);
	lcd_write_byte(0x04);
	lcd_write_byte(0x0C);
	lcd_write_byte(0x11);
	lcd_write_byte(0x13);
	lcd_write_byte(0x2C);
	lcd_write_byte(0x3F);
	lcd_write_byte(0x44);
	lcd_write_byte(0x51);
	lcd_write_byte(0x2F);
	lcd_write_byte(0x1F);
	lcd_write_byte(0x1F);
	lcd_write_byte(0x20);
	lcd_write_byte(0x23);

	lcd_write_cmd(0x21); 
	lcd_write_cmd(0x11); 

	lcd_write_cmd(0x29); 
	draw_background(background);
	gpio_set_level(LCD_BACKLIGHT_PIN, 1);
	draw_qrcode();
	draw_background_anywhere(100, 100, 139, 139, background);
	// int i;
	// for(i=0;i<120;i++)
	// draw_background_anywhere(119-i, 119-i, i*2, i*2, background);
	// draw_background_anywhere(0, 0, 239, 239, background);
	// lcd_clear32(0x0);
extern void lcd_putch(uint16_t x, uint16_t y, uint8_t num, bool mode, uint16_t color);
extern void lcd_print(uint16_t x, uint16_t y, char *str, uint16_t color);
	lcd_print(0, 0, "connect WIFI by QR-code", 0xffff);
    return ESP_OK;
}

// extern const uint8_t font_1608[];
typedef struct _font_t {
	uint8_t width;
	uint8_t hight;
	uint32_t size;
	const uint8_t *font;
}font_t;
extern font_t ascii_1608;
void lcd_drawpoint(uint16_t x,uint16_t y, uint16_t color)
{
	lcd_set_position(x, y, x+1, y+1);
	lcd_set_dc(1);
	lcd_write_color(color); 
}
void lcd_putch(uint16_t x, uint16_t y, uint8_t num, bool mode, uint16_t color)
{
    uint8_t temp;
    uint8_t pos,t;
	uint16_t x0=x;
	uint16_t colortemp=color;      
    if(x>LCD_HOR-ascii_1608.width||y>LCD_VER-ascii_1608.hight) return;	       
	num = num - ' ';
	lcd_set_position(x, y, x+ascii_1608.width-1, y+ascii_1608.hight-1);
	union _double_color_t {
		uint32_t data;
		struct {
			uint32_t r0:5;
			uint32_t g0:6;
			uint32_t b0:5;
			uint32_t r1:5;
			uint32_t g1:6;
			uint32_t b1:5;
		};
	};
	trans_color.bits.mosi = 16;
	if(!mode) //非叠加方式
	{
		lcd_set_dc(1);
		for(pos=0; pos<ascii_1608.hight; pos++)
		{ 
			temp=ascii_1608.font[(uint16_t)num*ascii_1608.hight+pos];		 //调用1608字体
			for(t=0;t<ascii_1608.width;t++)
		    {                 
		        if(temp&0x01)color=colortemp;
				else {
#if 1
					uint8_t *s = (uint8_t *)&background[0x36+(((LCD_VER-(y+pos)-1)*240+x+t)*3)];
					union _double_color_t p;
					p.r0 = s[0] >> 3;
					p.g0 = s[1] >> 2;
					p.b0 = s[2] >> 3;
					p.r1 = s[0] >> 3;
					p.g1 = s[1] >> 2;
					p.b1 = s[2] >> 3;
					color = p.data & 0xffff;
#else
					color = 0x0; // p.data & 0xffff;
#endif
				}
				lcd_write_color(color);	
				temp>>=1; 
				x++;
		    }
			x=x0;
			y++;
		}	
	}
	else//叠加方式
	{
		for(pos=0;pos<ascii_1608.hight;pos++)
		{
		    temp=ascii_1608.font[(uint16_t)num*ascii_1608.hight+pos];		 //调用1608字体
			for(t=0;t<ascii_1608.width;t++)
		    {                 
		        if(temp&0x01)lcd_drawpoint(x+t,y+pos, color);//画一个点     
			else{
					uint8_t *s = (uint8_t *)&background[0x36+(((240-(y+pos))*240+x+t)*3)];
					union _double_color_t p;
					p.r0 = s[0] >> 3;
					p.g0 = s[1] >> 2;
					p.b0 = s[2] >> 3;
					p.r1 = s[0] >> 3;
					p.g1 = s[1] >> 2;
					p.b1 = s[2] >> 3;
					lcd_drawpoint(x+t,y+pos, p.data & 0xffff);
}
		        temp>>=1; 
		    }
		}
	}
	color=colortemp;	    	   	 	  
}  

void lcd_print(uint16_t x, uint16_t y, char *str, uint16_t color)
{
	uint16_t i = 0;
	while(str[i]) {
		lcd_putch(x + i*ascii_1608.width, y, str[i], false, color);
		i++;
	}
}
