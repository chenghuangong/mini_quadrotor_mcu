#include <stdio.h>
#include <pico/stdlib.h>
#include <hardware/pwm.h>
#include <hardware/adc.h>
#include <pico.h>

/*
test half output thrust
*/

// define motor mosfet pin
const uint MOTOR_1 = 10;

// define duty cycle, half thrust
int duty = 200;
const int wrap_value = 4000;
const int clock_div = 10;

// adc pin
const uint adc_pin = 26;


// uart 
#define UART_ID uart0
#define BAUD_RATE 115200
// #define BAUD_RATE 9600

// 使用GPIO0 和 GPIO1来作为UART PIN
#define UART_TX_PIN 0
#define UART_RX_PIN 1


int main()
{
    stdio_init_all();

    // set pwm
    gpio_set_function(MOTOR_1, GPIO_FUNC_PWM);
    uint slice_num_1 = pwm_gpio_to_slice_num(MOTOR_1);

    // set adc
    adc_init();
    // Make sure GPIO is high-impedance, no pullups etc
    adc_gpio_init(26);


    // set uart
    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    // get pwm config
    pwm_config config = pwm_get_default_config();
    pwm_config_set_wrap(&config, wrap_value);
    pwm_config_set_clkdiv(&config, clock_div);
    
    // set duty cycle
    pwm_set_gpio_level(MOTOR_1, 0);
    pwm_init(slice_num_1, &config, true);

    while (true)
    {
        if (duty < 1000)
        {
            duty += 100;
        }

        // read adc channel0
        adc_select_input(0);
        uint adc1 = adc_read();
        printf("adc valve is %d\n", adc1);

        pwm_set_gpio_level(MOTOR_1, duty);
        sleep_ms(2000);
    }
    
    
    return 0;
}