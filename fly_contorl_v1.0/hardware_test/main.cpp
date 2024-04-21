#include <stdio.h>
#include <pico/stdlib.h>
#include <hardware/pwm.h>
#include <pico.h>
#include "hw_config.h"
#include "BMP280.hpp"

bool led_status = false;

const int duty = 200;
const int wrap_value = 4000;
const int clock_div = 1;

void initialize_motor();
void turn_motor_on_sequence(uint motor1, uint motor2, uint motor3, uint motor4);

bool led_blink_callback(repeating_timer_t* t);
bool bmp280_read_callback(repeating_timer_t* t);

int main()
{
    stdio_init_all();


    // initialize LED
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    repeating_timer led_timer;
    add_repeating_timer_ms(200, led_blink_callback, nullptr, &led_timer);
    

    // initialize I2C
    i2c_init(I2C_INSTANCE, I2C_BAUDRATE);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);


    // initialize BMP280
    bmp280 bmp280_dev;
    bmp280_initialize(&bmp280_dev, I2C_INSTANCE);
    repeating_timer bmp280_timer;
    add_repeating_timer_ms(1000, bmp280_read_callback, &bmp280_dev, &bmp280_timer);
    
    
    // initialize BMI088


    // initialize motor
    initialize_motor();
    turn_motor_on_sequence(MOTOR_0, MOTOR_1, MOTOR_2, MOTOR_3);
    return 0;
}


void initialize_motor()
{
    // set pwm
    gpio_set_function(MOTOR_0, GPIO_FUNC_PWM);
    gpio_set_function(MOTOR_1, GPIO_FUNC_PWM);
    gpio_set_function(MOTOR_2, GPIO_FUNC_PWM);
    gpio_set_function(MOTOR_3, GPIO_FUNC_PWM);

    uint slice_num_1 = pwm_gpio_to_slice_num(MOTOR_0);
    uint slice_num_2 = pwm_gpio_to_slice_num(MOTOR_1);
    uint slice_num_3 = pwm_gpio_to_slice_num(MOTOR_2);
    uint slice_num_4 = pwm_gpio_to_slice_num(MOTOR_3);

    // get pwm config
    pwm_config config = pwm_get_default_config();
    pwm_config_set_wrap(&config, wrap_value);
    pwm_config_set_clkdiv(&config, clock_div);
    
    // set duty cycle
    pwm_set_gpio_level(MOTOR_0, 0);
    pwm_set_gpio_level(MOTOR_1, 0);
    pwm_set_gpio_level(MOTOR_2, 0);
    pwm_set_gpio_level(MOTOR_3, 0);

    pwm_init(slice_num_1, &config, true);
    pwm_init(slice_num_2, &config, true);
    pwm_init(slice_num_3, &config, true);
    pwm_init(slice_num_4, &config, true);
}


void turn_motor_on_sequence(uint motor0, uint motor1, uint motor2, uint motor3)
{
    while (1)
    {
        pwm_set_gpio_level(motor3, 0);
        pwm_set_gpio_level(motor0, duty);
        sleep_ms(1000);

        pwm_set_gpio_level(motor0, 0);
        pwm_set_gpio_level(motor1, duty);
        sleep_ms(1000);

        pwm_set_gpio_level(motor1, 0);
        pwm_set_gpio_level(motor2, duty);
        sleep_ms(1000);

        pwm_set_gpio_level(motor2, 0);
        pwm_set_gpio_level(motor3, duty);
        sleep_ms(1000);
    }    
}


bool led_blink_callback(repeating_timer_t* t)
{
    gpio_put(LED_PIN, led_status = !led_status);
    return true;
}


bool bmp280_read_callback(repeating_timer_t* t)
{
    bmp280* dev = static_cast<bmp280*>(t->user_data);
    bmp280_read_temperature_and_pressure(dev);
    printf("temperature is %.2f, pressure is %.3fkpa\n", dev->temperature / 100.0, dev->pressure / 1000.0);
    return true;
}