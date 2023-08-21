#ifndef OLED_DISPLAY_H_
#define OLED_DISPLAY_H_

#include <pico/stdlib.h>
#include <string>
#include <stdio.h>
#include <algorithm>
#include "hardware/i2c.h"
#include "ascii_character.h"

/* 
display panel: 0.96 inch oled panel, resolution 128x64, driven by SSD1306, vcc= 3.3v
use i2c protocol
the default i2c device address is 0x78(with R/W bit) or 0x3C(without R/W bit)

character size 8 * 16 pixels, total characters 16 * 4 = 64, 16 characters per line
*/

#define ADDR _u(0x3C)                                 // chip address
#define OLED_WIDTH _u(128)                            // panel width
#define OLED_HEIGHT _u(64)                            // panel height
#define OLED_PAGE_HEIGHT _u(8)                        // ram page height
#define OLED_NUM_PAGES OLED_HEIGHT / OLED_PAGE_HEIGHT // nums of pages
#define OLED_BUF_LEN (OLED_NUM_PAGES * OLED_WIDTH)    // one seg in one page was represented by 1byte

// 1. Fundamental Command Table
#define OLED_SET_DISP _u(0xAE)      // display on/off
#define OLED_SET_CONTRAST _u(0x81)  // contrast
#define OLED_SET_NORM_INV _u(0xA6)  // A6 Normal display, A7 Inverse display
#define OLED_SET_ENTIRE_ON _u(0xA4) // A4 output follows RAM content, A5 output ignores RAM content

// 2. Scrolling Command Table
#define OLED_SET_SCROLL _u(0x2E)    // deactivate scroll

// 3. Addressing Setting Command Table
#define OLED_SET_MEM_ADDR _u(0x20)  // set memory address mode, A[1:0] = 00b, Horizontal Addressing Mode
#define OLED_SET_COL_ADDR _u(0x21)  // Column start address, range : 0-127d, Column end address, range : 0-127d
#define OLED_SET_PAGE_ADDR _u(0x22) // Setup page start and end address, Page start Address, range : 0-7d, Page end Address, range : 0-7d, (RESET = 7d)

// 4. Hardware Configuration (Panel resolution & layout related) Command Table
#define OLED_SET_DISP_START_LINE _u(0x40) // set display RAM display start line register from 0-63 using X5X3X2X1X0.
#define OLED_SET_SEG_REMAP _u(0xA0)       // A1h, X[0]=1b: column address 127 is mapped to SEG0
#define OLED_SET_MUX_RATIO _u(0xA8)       // set MUX ratio to N+1 MUX N=A[5:0] : from 16MUX to 64MUX, RESET=111111b (i.e. 63d, 64MUX)
#define OLED_SET_COM_OUT_DIR _u(0xC0)     // set scanning direction
#define OLED_SET_DISP_OFFSET _u(0xD3)     // set vertical shift by COM from 0d~63d The value is reset to 00h after RESET.

// 5. Timing & Driving Scheme Setting Command Table
#define OLED_SET_DISP_CLK_DIV _u(0xD5) // define the divide ratio (D) of the display clocks (DCLK): Divide ratio= A[3:0] + 1
#define OLED_SET_PRECHARGE _u(0xD9)    // pre-charge the stray capacitance
#define OLED_SET_VCOM_DESEL _u(0xDB)   // set VCOMH Deselect Level
#define OLED_SET_CHARGE_PUMP _u(0x8D)  // set charge pump


class oled_disp
{
public:
    oled_disp(i2c_inst_t* i2c_instance, uint8_t sda = PICO_DEFAULT_I2C_SDA_PIN, uint8_t scl = PICO_DEFAULT_I2C_SCL_PIN);
    ~oled_disp();

public:
    void init_dev();

    /*
    send a default image to the device
    send a default string to the device
    */
    void show_test_image();
    void show_test_string();


    /*
    write string to the device
    */
    oled_disp& operator<<(const std::string& str);

    /* 
    @param buf, image byte array
    @param buf_len, image byte array length
    @param x, start position x (width direction)
    @param x, start position y (height direction)
    */
    // void write_image(const uint8_t* buf, uint_t buf_len, uint8_t x = 0, uint8_t y = 0);
    // void write_image_128x64();
    // void write_string_16x32_en(const string& str, uint8_t x = 0, uint8_t y = 0);
    
    //double get_temp();
    //double get_humidity();
    //dht_reading& get_temp_and_humidity();

    // 方法一：需要使用定时器持续不断的读取数据，并进行存储，入队列，然后再计算出平均值，调用函数只是取出数据，并不从传感器处获得数据
    // 方法二：快速的读取几个数据，然后求平均值
    //double get_filtered_temp();
    //double get_filtered_humidity();
    //dht_reading& get_filtered_temp_and_humidity();

private:
    void init_oled_i2c();
    void init_oled_driver();

    void oled_send_cmd(uint8_t cmd);
    void oled_send_to_memory(uint8_t* buf, size_t buf_len);

    void write_text_to_dev(const std::string& text);    // 底层的写入string的函数
    //void read_from_dht();   // get error data three times, use the last read value

    /* 
    read data from dht11
    @param r store dht_reading result (temperature, humidity)
    @return true if reading is succeed, but not care whether the result is reasonable 
    */
    //bool do_read(dht_reading& r);
    //bool is_read_data_reasonable(dht_reading& r);

private:
    uint8_t sda_;
    uint8_t scl_;
    i2c_inst_t* i2c_instance_;
};


#endif