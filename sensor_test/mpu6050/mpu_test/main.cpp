/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"

/* Example code to talk to a MPU6050 MEMS accelerometer and gyroscope

   This is taking to simple approach of simply reading registers. It's perfectly
   possible to link up an interrupt line and set things up to read from the
   inbuilt FIFO to make it more useful.

   NOTE: Ensure the device is capable of being driven at 3.3v NOT 5v. The Pico
   GPIO (and therefor I2C) cannot be used at 5v.

   You will need to use a level shifter on the I2C lines if you want to run the
   board at 5v.

   Connections on Raspberry Pi Pico board, other boards may vary.

   GPIO PICO_DEFAULT_I2C_SDA_PIN (On Pico this is GP4 (pin 6)) -> SDA on MPU6050 board
   GPIO PICO_DEFAULT_I2C_SCL_PIN (On Pico this is GP5 (pin 7)) -> SCL on MPU6050 board
   3.3v (pin 36) -> VCC on MPU6050 board
   GND (pin 38)  -> GND on MPU6050 board
*/

// By default these devices  are on bus address 0x68
static int addr = 0x68;


static void mpu6050_reset() {
    // Two byte reset. First byte register, second byte data
    // There are a load more options to set up the device in different ways that could be added here
    uint8_t buf[] = {0x6B, 0x80};
    i2c_write_blocking(i2c_default, addr, buf, 2, false);

    // after reset, the default value will bring mpu6050 into sleep mode,
    // so we need to bring sensor from sleep mode by set 0x6B register to 0x00
    sleep_ms(100);
    buf[1] = 0x00;
    i2c_write_blocking(i2c_default, addr, buf, 2, false);
}

static void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp) {
    // For this particular device, we send the device the register we want to read
    // first, then subsequently read from the device. The register is auto incrementing
    // so we don't need to keep sending the register we want, just the first.

    uint8_t buffer[6];

    // Start reading acceleration registers from register 0x3B for 6 bytes
    uint8_t val = 0x3B;
    i2c_write_blocking(i2c_default, addr, &val, 1, true); // true to keep master control of bus
    i2c_read_blocking(i2c_default, addr, buffer, 6, false);

    for (int i = 0; i < 3; i++) {
        accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }

    // Now gyro data from reg 0x43 for 6 bytes
    // The register is auto incrementing on each read
    val = 0x43;
    i2c_write_blocking(i2c_default, addr, &val, 1, true);
    i2c_read_blocking(i2c_default, addr, buffer, 6, false);  // False - finished with bus

    for (int i = 0; i < 3; i++) {
        gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);;
    }

    // Now temperature from reg 0x41 for 2 bytes
    // The register is auto incrementing on each read
    val = 0x41;
    i2c_write_blocking(i2c_default, addr, &val, 1, true);
    i2c_read_blocking(i2c_default, addr, buffer, 2, false);  // False - finished with bus

    *temp = buffer[0] << 8 | buffer[1];	
}

#define TIME_INTERVAL 20       // ms
#define ACC_SENSITIVITY 16384  // per g
#define GYRO_SENSITIVITY 131   // per deg/s
#define AVERAGE_OFFSET_TIME 60 // 计算roll pitch yaw角度偏差量平均值的时间，单位秒

double gyro_pitch = 0;
double gyro_roll = 0;
double gyro_yaw = 0;

double gyro_pitch_cali = 0;
double gyro_roll_cali = 0;
double gyro_yaw_cali = 0;

// 平均的角度误差量 单位deg/s，具有GYRO_SENSITIVITY
double average_pitch_offset = 0;
double average_roll_offset = 0;
double average_yaw_offset = 0;

int total_time = 0;

void output_plotter_data_type(int16_t* accel, int16_t* gyro, int16_t* temp);
void output_integral_data(int16_t* accel, int16_t* gyro);


int main() {
    stdio_init_all();

#if !defined(i2c_default) || !defined(PICO_DEFAULT_I2C_SDA_PIN) || !defined(PICO_DEFAULT_I2C_SCL_PIN)
    #warning i2c/mpu6050_i2c example requires a board with I2C pins
    puts("Default I2C pins were not defined");
#else
    printf("Hello, MPU6050! Reading raw data from registers...\n");

    // This example will use I2C0 on the default SDA and SCL pins (4, 5 on a Pico)
    i2c_init(i2c_default, 400 * 1000);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
    // Make the I2C pins available to picotool
    bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));

    mpu6050_reset();


    int16_t acceleration[3], gyro[3], temp;
    
    while (1) 
    {
        mpu6050_read_raw(acceleration, gyro, &temp);
        output_plotter_data_type(acceleration, gyro, &temp);
        // output_integral_data(acceleration, gyro);
        sleep_ms(TIME_INTERVAL);  
    }

#endif
    return 0;
}


// step1
// 1. convert acceleration value to roll and pitch data
// 2. output raw data to plotter
void output_plotter_data_type(int16_t* accel, int16_t* gyro, int16_t* temp)
{
    double roll_theta = atan(accel[1] / pow(accel[0] * accel[0] + accel[2] * accel[2], 0.5)) * (180 / M_PI);   // 沿着X方向旋转，x朝向前进方向
    double pitch_theta = atan(-accel[0] / pow(accel[1] * accel[1] + accel[2] * accel[2], 0.5)) * (180 / M_PI); // 沿着Y轴方向
    printf("%d\t%d\t%d\t%d\t%d\t%d\t%f\t%.2f\t%.2f\n", accel[0], accel[1], accel[2], gyro[0], gyro[1], gyro[2], (*temp / 340.0) + 36.53, roll_theta, pitch_theta);
}


// step2
// calculate pitch by gyro_y integral
// calculate roll by gyro_x integral
void output_integral_data(int16_t* accel, int16_t* gyro)
{
    gyro_roll = gyro_roll + TIME_INTERVAL * gyro[0] * 0.001 / GYRO_SENSITIVITY;
    gyro_pitch = gyro_pitch + TIME_INTERVAL * gyro[1] * 0.001 / GYRO_SENSITIVITY;
    gyro_yaw = gyro_yaw + TIME_INTERVAL * gyro[2] * 0.001 / GYRO_SENSITIVITY;
    total_time = total_time + TIME_INTERVAL;

    // calculate offset
    if (total_time == (AVERAGE_OFFSET_TIME * 1000))
    {
        average_roll_offset = (gyro_roll / AVERAGE_OFFSET_TIME) * GYRO_SENSITIVITY;
        average_pitch_offset = (gyro_pitch / AVERAGE_OFFSET_TIME) * GYRO_SENSITIVITY;
        average_yaw_offset = (gyro_yaw / AVERAGE_OFFSET_TIME) * GYRO_SENSITIVITY;
        //printf("offset_roll = %.2f, offset_pitch = %.2f, offset_yaw = %.2f", offset_roll, offset_pitch, offset_yaw);
    }

    if (total_time >= (AVERAGE_OFFSET_TIME * 1000))
    {
        gyro_roll_cali = gyro_roll_cali + TIME_INTERVAL * (gyro[0] - average_roll_offset) * 0.001 / GYRO_SENSITIVITY;
        gyro_pitch_cali = gyro_pitch_cali + TIME_INTERVAL * (gyro[1] - average_pitch_offset) * 0.001 / GYRO_SENSITIVITY;
        gyro_yaw_cali = gyro_yaw_cali + TIME_INTERVAL * (gyro[2] - average_yaw_offset) * 0.001 / GYRO_SENSITIVITY;
    }

    printf("%d\t%d\t%d\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%d\n", gyro[0], gyro[1], gyro[2], gyro_roll, gyro_pitch, gyro_yaw, gyro_roll_cali, gyro_pitch_cali, gyro_yaw_cali, total_time);
}


