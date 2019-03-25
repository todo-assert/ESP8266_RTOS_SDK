
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
{
	uint32_t ofsbit = 0x3e;// qrcode[0xd] << 24 | qrcode[0xc] << 16 | qrcode[0xb] << 8 | qrcode[0xa];
	size_t siz = 3200; // (qrcode[3] << 8 | qrcode[2]) - ofsbit;
	uint8_t *start = &qrcode[ofsbit];
	int i, j;
	// lcd_set_position(40,40,200,200);
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
	// for( int i=0;i<LCD_VER;i++ )
		// memset(framebuffer[i], 0xFFFF, LCD_HOR);
	// lcd_update();
	lcd_clear32(0xf00f);
	draw_qrcode();
	gpio_set_level(LCD_BACKLIGHT_PIN, 1);
	// backlight_listen_thread
	// xTaskCreate(&backlight_listen_thread, "backlight", 512, NULL, 8, NULL);
// uint32_t c = 0;
// uint32_t k;
	// for(c=0;c<0xffffffff;c+=0x10001) {
		// lcd_clear32(c);
		// lcd_delay_ms(100);
	// }
    return ESP_OK;
}


