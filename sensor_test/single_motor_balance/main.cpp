#include <stdio.h>
#include <pico/stdlib.h>
#include <hardware/pwm.h>
#include <hardware/adc.h>
#include <pico.h>
#include "MPU6050.hpp"

/*
test half output thrust
*/

// define motor mosfet pin
const uint MOTOR_1 = 10;

// define duty cycle, half thrust
int duty = 0;
const int wrap_value = 4000;
const int clock_div = 1;

// adc pin
const uint adc_pin = 26;

// uart 
#define UART_ID uart0
#define BAUD_RATE 115200
// #define BAUD_RATE 9600

// 使用GPIO0 和 GPIO1来作为UART PIN
#define UART_TX_PIN 0
#define UART_RX_PIN 1

// PID setting
int target_deg = -20;
double p_value = 50;
double i_value = 50;
double d_valve = -0.5;
//double i_value = 20;
//double d_valve = -0.01;
double angle_error = 0;

bool send_out_info_callback(repeating_timer_t *rt)
{
    // printf("deg error = %.2f, duty cycles = %d \n", angle_error, duty);
    printf("%.2f\t%d\n", angle_error, duty);
    return true;
}


int main()
{
    stdio_init_all();

    i2c_init(i2c_default, 100 * 1000);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

    mpu6050 mpu6050_dev;
    mpu6050_initialize(&mpu6050_dev, i2c_default);
    mpu6050_set_config(&mpu6050_dev, mpu6050_config::DRONE);

    // set pwm
    gpio_set_function(MOTOR_1, GPIO_FUNC_PWM);
    uint slice_num_1 = pwm_gpio_to_slice_num(MOTOR_1);

    // set adc
    adc_init();
    // Make sure GPIO is high-impedance, no pullups etc
    adc_gpio_init(26);
    adc_gpio_init(27);

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

    uint power_supply_adc = 0;
    uint current_sensor_adc = 0;

    // set pid variables
    double accumulate_err = 0;
    double differential_err = 0;

    repeating_timer timer;
    add_repeating_timer_ms(500, send_out_info_callback, nullptr, &timer);

    


    while (true)
    {
        mpu6050_read_kalman_data(&mpu6050_dev);

        // PID algorithm
        angle_error = target_deg - mpu6050_dev.angle_k[0];
        accumulate_err += angle_error * 0.005;
        differential_err = mpu6050_dev.angle_k[0] - mpu6050_dev.angle_kp[0];

        // calculate next duty cycles
        duty = angle_error * p_value + accumulate_err * i_value + (differential_err / 0.005) * d_valve;

        if (duty < 0)
        {
            duty = 0;
        }

        if (duty > 4000)
        {
            duty = 4000;
        }
        
        pwm_set_gpio_level(MOTOR_1, duty);
        sleep_ms(5);
    }

    return 0;
}