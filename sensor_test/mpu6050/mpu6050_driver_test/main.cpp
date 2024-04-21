/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "MPU6050.hpp"

mpu6050 mpu6050_dev;


bool print_timer_callback(repeating_timer_t* timer)
{
    // printf("roll = %.2f; pitch = %.2f\n", 
    printf("%.2f\t%.2f\n", 
    mpu6050_dev.angle_k[0], mpu6050_dev.angle_k[1]); 
    return true;
}

int main() 
{
    
    stdio_init_all();

    i2c_init(i2c_default, 400 * 1000);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

    
    mpu6050_initialize(&mpu6050_dev, i2c_default);
    mpu6050_set_config(&mpu6050_dev, mpu6050_config::DRONE);

    // while (true)
    // {
    //     // mpu6050_read_raw(&mpu6050_dev);
    //     mpu6050_read_converted_data(&mpu6050_dev);
    //     printf("roll = %.2f; pitch = %.2f; yaw = %.2f; gyro x = %.2f; gyro y = %.2f; gyro z = %.2f; temp = %.2f\n", 
    //     mpu6050_dev.angle_c[0], mpu6050_dev.angle_c[1], mpu6050_dev.angle_c[2], 
    //     mpu6050_dev.gyro_c[0], mpu6050_dev.gyro_c[1], mpu6050_dev.gyro_c[2], mpu6050_dev.temp_c);
    //     sleep_ms(500);
    // }


    repeating_timer_t timer;
    add_repeating_timer_ms(500, &print_timer_callback, nullptr, &timer);
    while (true)
    {
        mpu6050_read_kalman_data(&mpu6050_dev);
        sleep_ms(20);
    }
    
    

    return 0;
}


