#include <pico/stdlib.h>
#include <stdio.h>
#include "hardware/adc.h"
#include "oled_disp.h"

const uint btn1 = 6;
const uint btn2 = 7;

const uint btn_up = 9;
const uint btn_down = 10;
const uint btn_left = 11;
const uint btn_right = 12;

// 使用GPIO0 和 GPIO1来作为UART PIN
#define UART_ID uart0
#define BAUD_RATE 115200
#define UART_TX_PIN 0
#define UART_RX_PIN 1


void send_msg_to_host(uint8_t* msg, uint8_t* data);
void check_btn_pressed(oled_disp& disp, const uint& gpio_pin);


// 
uint8_t joystick_msg[9] = {0xAA, 0xBB, 0x10, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t btn_msg[9] = {0xAA, 0xBB, 0x11, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00};

int main()
{
    stdio_init_all();
    const uint LED_PIN = PICO_DEFAULT_LED_PIN;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    bool led = true;

    adc_init();
    // Make sure GPIO is high-impedance, no pullups etc
    adc_gpio_init(26);
    adc_gpio_init(27);
    adc_gpio_init(28);
    adc_gpio_init(29);

    float adc_value[4];
    int adc_level[4];
    uint btns[6] = {btn1, btn2, btn_up, btn_down, btn_left, btn_right};

    // button
    gpio_init(btn1);
    gpio_set_dir(btn1, GPIO_IN);
    gpio_init(btn2);
    gpio_set_dir(btn2, GPIO_IN);
    gpio_init(btn_up);
    gpio_set_dir(btn_up, GPIO_IN);
    gpio_init(btn_down);
    gpio_set_dir(btn_down, GPIO_IN);
    gpio_init(btn_left);
    gpio_set_dir(btn_left, GPIO_IN);
    gpio_init(btn_right);
    gpio_set_dir(btn_right, GPIO_IN);

    // init diaplay
    i2c_init(i2c_default, 400 * 1000);
    gpio_set_function(4, GPIO_FUNC_I2C);
    gpio_set_function(5, GPIO_FUNC_I2C);
    gpio_pull_up(4);
    gpio_pull_up(5);

    oled_disp oled_one{i2c_default, 4, 5};
    oled_one.init_dev();


    sleep_ms(10000);
    // esp01s test
    uart_init(UART_ID, BAUD_RATE);
    // Set the TX and RX pins by using the function select on the GPIO
    // see datasheet for more information on function select
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    sleep_ms(100);    
    // 连接到服务器
    uart_puts(UART_ID, "AT+CIPSTART=\"TCP\",\"192.168.31.17\",12800\r\n");
    sleep_ms(1500);
    uart_puts(UART_ID, "AT+CIPSTART=\"TCP\",\"192.168.31.17\",12800\r\n");
    sleep_ms(1500);

    // 开启透传模式
    std::string target = "AT+CIPMODE=1\r\n";
    uart_puts(UART_ID, target.c_str());
    sleep_ms(500);
    uart_puts(UART_ID, target.c_str());
    sleep_ms(500);

    target = "AT+CIPSEND\r\n";
    uart_puts(UART_ID, target.c_str());
    sleep_ms(500);


    while (true)
    {
        // 1. blink LED
        gpio_put(LED_PIN, led = !led);

        // 2. read joystick value
        for (size_t i = 0; i < 4; i++)
        {
            adc_select_input(i);
            adc_value[i] = adc_read() * (3.3 / 4096);
            adc_level[i] = int(adc_value[i] / 0.3);
        }


        // printf("x1_value:%.2fv, y1_value%.2fv, x2_value:%.2fv, y2_value%.2fv\n", adc_value[0], adc_value[1], adc_value[2], adc_value[3]);
        // printf("{\"msg_type\":\"data\",\"dev_type\":\"joystick_panel\",\"source\":\"joystick\",\"value\":[%d,%d,%d,%d]}", 
        // adc_level[0], adc_level[1], adc_level[2], adc_level[3]);


        // send meg to server
        uint8_t joystick_data[4] = {0x00, 0x00, 0x00, 0x00};
        for (size_t i = 0; i < 4; i++)
        {
            joystick_data[i] = static_cast<uint8_t>(adc_level[i]);
        }
        send_msg_to_host(joystick_msg, joystick_data);
        
        
        
       
        // std::string result = std::string("x1_value=") + std::to_string(adc_value[0]);
        // uart_puts(UART_ID, result.c_str());

        // 3. check button
        for (size_t i = 0; i < 6; i++)
        {
            check_btn_pressed(oled_one, btns[i]);
        }

        // 4. check esp01s
        
        sleep_ms(200);
    }
}

void check_btn_pressed(oled_disp& disp, const uint& gpio_pin)
{
    if (gpio_get(gpio_pin) == false)
    {
        auto key = (std::to_string(gpio_pin) + std::string("pressed"));
        disp << key;
        std::string btn;


        uint8_t btn_data[4] = {0x00, 0x00, 0x00, 0x00};
        
        switch (gpio_pin)
        {
        case 9:btn = "btn_up"; btn_data[0] = 1; break;
        case 10:btn = "btn_down"; btn_data[1] = 1; break;
        case 11:btn = "btn_left"; btn_data[2] = 1; break;
        case 12:btn = "btn_right"; btn_data[3] = 1; break;
        default:break;
        }

        // printf("{\"msg_type\":\"data\",\"dev_type\":\"joystick_panel\",\"source\":\"%s\"}", btn.c_str());
        // uart_puts(UART_ID, key.c_str());
        // printf("btn=%d was pressed\n", gpio_pin);
        send_msg_to_host(btn_msg, btn_data);

    }
}

void send_msg_to_host(uint8_t* msg, uint8_t* data)
{
    for (size_t i = 0; i < 4; i++)
    {
        msg[i + 4] = data[i]; 
    }

    uint16_t check_sum = 0;
    for (size_t i = 0; i < 8; i++)
    {
        check_sum += static_cast<uint8_t>(msg[i]);
    }
    msg[8] = static_cast<uint8_t>(check_sum & 0xFF);

    for (size_t i = 0; i < 9; i++)
    {
        uart_putc(uart0, msg[i]);
    }
}