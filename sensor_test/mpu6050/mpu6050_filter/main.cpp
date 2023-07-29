/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include "cmath"


#define SAMPLING_TIME 0.004           // 设定采样时间
#define UPLOAD_TIME 0.01              // 100 ms发送一次数据
#define ACC_SENSITIVITY 16384.0       // per g
#define GYRO_SENSITIVITY 131.0         // per °/s

int16_t acceleration[3], gyro[3], temp;


// =======================RC Low Pass Filter=======================

struct rc_data
{
    double rc_value =  1 / (2 * M_PI * 1);  // remain, 1 = cut off frequency 

    double output_data[3] = {0, 0, 0};

};

rc_data rc_filter;

// =======================RC Low Pass Filter=======================

// =======================moving average filter=======================

struct moving_average_data
{
    // using loop
    bool is_initialized = false;
    int average_buffer_length = 10;                            // 滞后严重

    double* average_data;
    uint8_t save_value_pos = average_buffer_length - 1;        // start at last position
    uint8_t delete_value_pos = 0;
    double sum = 0;

    double output_data = 0;
};

moving_average_data mova_filter;

// =======================moving average filter=======================



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

#ifdef i2c_default
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
#endif


bool read_sensor_callback(repeating_timer_t* rt);
bool output_data_callback(repeating_timer_t* rt);
void apply_rc_filter();
void apply_moving_average_filter();
void move_average_filter();
void mpu6050_enable_low_pass_filter();

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
    // mpu6050_enable_low_pass_filter();
    
    repeating_timer read_sensor_timer;
    add_repeating_timer_ms(SAMPLING_TIME * 1000, &read_sensor_callback, nullptr, &read_sensor_timer);

    repeating_timer output_data_timer;
    add_repeating_timer_ms(UPLOAD_TIME * 1000, &output_data_callback, nullptr, &output_data_timer);


    while (1) 
    {
        tight_loop_contents();
        
        // // These are the raw numbers from the chip, so will need tweaking to be really useful.
        // // See the datasheet for more information
        // printf("Acc. X = %d, Y = %d, Z = %d\n", acceleration[0], acceleration[1], acceleration[2]);
        // printf("Gyro. X = %d, Y = %d, Z = %d\n", gyro[0], gyro[1], gyro[2]);
        // // Temperature is simple so use the datasheet calculation to get deg C.
        // // Note this is chip temperature.
        // printf("Temp. = %f\n", (temp / 340.0) + 36.53);
    }

#endif
    return 0;
}


bool read_sensor_callback(repeating_timer_t* rt)
{
    // read sensor
    mpu6050_read_raw(acceleration, gyro, &temp);

    // apply filter
    apply_rc_filter();
    // apply_moving_average_filter();

    return true;
}


void apply_rc_filter()
{
    for (size_t i = 0; i < 3; i++)
    {
        rc_filter.output_data[i] =  (SAMPLING_TIME / (SAMPLING_TIME + rc_filter.rc_value)) * (gyro[i] / GYRO_SENSITIVITY) + 
                                    (rc_filter.rc_value / (SAMPLING_TIME + rc_filter.rc_value)) * rc_filter.output_data[i]; 
    }   
}

void apply_moving_average_filter()
{
    // initialize data buffer
    if (!mova_filter.is_initialized)
    {
        mova_filter.average_data = new double[mova_filter.average_buffer_length];
        for (size_t i = 0; i < mova_filter.average_buffer_length; i++)
        {
            mova_filter.average_data[i] = 0;
        }
        mova_filter.is_initialized = true;
    }

    mova_filter.sum = mova_filter.sum + gyro[0] / GYRO_SENSITIVITY - mova_filter.average_data[mova_filter.delete_value_pos];
    // save value
    mova_filter.average_data[mova_filter.save_value_pos] = gyro[0] / GYRO_SENSITIVITY;
    // update delete position and save value position
    mova_filter.delete_value_pos = ++mova_filter.delete_value_pos == mova_filter.average_buffer_length ? 0 : mova_filter.delete_value_pos;
    mova_filter.save_value_pos = ++mova_filter.save_value_pos == mova_filter.average_buffer_length ? 0 : mova_filter.save_value_pos;
    // calculate output value
    mova_filter.output_data = mova_filter.sum / mova_filter.average_buffer_length;
}


void mpu6050_enable_low_pass_filter()
{
    // enable low pass filter
    uint8_t buf1[] = {0x1A, 0x05};
    i2c_write_blocking(i2c_default, addr, buf1, 2, false);

    // set sensitivity to 65.5
    uint8_t buf2[] = {0x1B, 0x08};
    i2c_write_blocking(i2c_default, addr, buf2, 2, false);    
}


bool output_data_callback(repeating_timer_t* rt)
{
    // output raw data and rc filter data
    // printf("%f\t%f\t%f\t%f\t%f\t%f\n", gyro[0] / GYRO_SENSITIVITY, gyro[1] / GYRO_SENSITIVITY, gyro[2] / GYRO_SENSITIVITY, 
    //         rc_filter.output_data[0], rc_filter.output_data[1], rc_filter.output_data[2]);
    printf("%f\t%f\n", gyro[0] / GYRO_SENSITIVITY, rc_filter.output_data[0]);


    // output raw data and moving average filter data
    // printf("%f\t%f\n", gyro[0] / GYRO_SENSITIVITY, mova_filter.output_data);
    
    return true;
}

