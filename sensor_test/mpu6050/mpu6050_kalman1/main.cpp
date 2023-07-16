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

   [direction]
   x+ face to north, y+ face to west, so rotation on x direction is roll, rotation on y direction is pitch
*/

// By default these devices  are on bus address 0x68
static int addr = 0x68;

#define SAMPLING_TIME 0.02         // 设定采样时间
#define ACC_SENSITIVITY 16384      // per g
// #define ACC_STD_DEVIATION 3        // °
// #define GYRO_SENSITIVITY 131       // per °/s
// #define GYRO_STD_DEVIATION 4       // °/s

#define ACC_STD_DEVIATION 0.0824   // °
#define GYRO_SENSITIVITY 131       // per °/s
#define GYRO_STD_DEVIATION 0.1665  // °/s

#ifdef i2c_default
static void mpu6050_reset() {
    // Two byte reset. First byte register, second byte data
    // There are a load more options to set up the device in different ways that could be added here
    uint8_t buf[] = {0x6B, 0x00};
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

struct kalman_data
{
    double ctrl_matrix = SAMPLING_TIME;         // control matrix

    double acc_std_deviation = ACC_STD_DEVIATION;
    double gyro_std_deviation = GYRO_STD_DEVIATION;

    // [roll, pitch, yaw(optional)]
    double kalman_angle[2] = {0, 0};            // angle predicted by kalman fiter, use gyro to calculate
    double kalman_gain[2] = {0, 0};
    double prediction_uncertainty[2] = {0, 0};
    double acc_angle[2] = {0, 0};               // calculated by accelerometer
    double yaw_speed = 0;                       // show yaw speed, do not integrate yaw
    double yaw_angle = 0;

    // optional
    int sampling_count = 0;
    int raw_data_sum[6] = {0, 0, 0, 0, 0, 0};
    int raw_data_offset[6] = {0, 0, 0, 0, 0, 0};
};

void output_plotter_data_type(int16_t* accel, int16_t* gyro, int16_t *temp);
void output_kalman_filter(kalman_data& k_data, int16_t* accel, int16_t* gyro);
void output_1Dkalman_filter(kalman_data& kal, int16_t* accel, int16_t* gyro);
void output_human_read_data(int16_t* accel, int16_t* gyro, int16_t* temp);


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
    kalman_data kal_inst;

    while (1) 
    {
        mpu6050_read_raw(acceleration, gyro, &temp);
        // output_human_read_data(acceleration, gyro, &temp);
        output_1Dkalman_filter(kal_inst, acceleration, gyro);
        sleep_ms(1000 * SAMPLING_TIME);    
    }

#endif
    return 0;
}


void output_plotter_data_type(int16_t* accel, int16_t* gyro, int16_t* temp)
{
    int roll_theta = atan(accel[1] / pow(accel[0] * accel[0] + accel[2] * accel[2], 0.5)) * 57.2958;
    int pitch_theta = atan(-accel[0] / pow(accel[1] * accel[1] + accel[2] * accel[2], 0.5)) * 57.2958; // 沿着Y轴方向
    printf("%d\t%d\t%d\t%d\t%d\t%d\t%f\t%d\t%d\n", accel[0], accel[1], accel[2], gyro[0], gyro[1], gyro[2], (*temp / 340.0) + 36.53, roll_theta, pitch_theta);
}


void output_1Dkalman_filter(kalman_data& kal, int16_t* accel, int16_t* gyro)
{
    // TODO calibrate gyro value

    kal.sampling_count++;

    
    // 1. set zero point
    if (kal.sampling_count <= 1000)
    {
        for (size_t i = 0; i < 3; i++)
        {
            kal.raw_data_sum[i] = kal.raw_data_sum[i] + accel[i];
            kal.raw_data_sum[i + 3] = kal.raw_data_sum[i + 3] + gyro[i];
        }

        if (kal.sampling_count == 1000)
        {
            for (size_t i = 0; i < 6; i++)
            {
                kal.raw_data_offset[i] = int(kal.raw_data_sum[i] / 1000.0);
            }

            // acc_z is different, acc_z = g, 1g = 16384
            kal.raw_data_offset[2] = int(kal.raw_data_sum[2] / 1000.0) - ACC_SENSITIVITY;
        }
        printf("%d\n", kal.sampling_count);
        return;
    }

    // calibrate raw data
    for (size_t i = 0; i < 3; i++)
    {
        accel[i] = accel[i] - kal.raw_data_offset[i];
        gyro[i] = gyro[i] - kal.raw_data_offset[i + 3];
    }
    
    // use accelerometer calculate 
    kal.acc_angle[0] = atan(accel[1] / pow(accel[0] * accel[0] + accel[2] * accel[2], 0.5)) * (180 / M_PI);  // roll
    kal.acc_angle[1] = atan(-accel[0] / pow(accel[1] * accel[1] + accel[2] * accel[2], 0.5)) * (180 / M_PI); // pitch

    // 2. use kalman fiter to calculate roll and pitch
    for (size_t i = 0; i < 2; i++)
    {
        // 1. predict the current state of the system
        // kalman_angle[0] is roll, gyro[0] = X direction gyro 
        kal.kalman_angle[i] = kal.kalman_angle[i] + (gyro[i] / double(GYRO_SENSITIVITY)) * kal.ctrl_matrix;

        // 2. calculate the uncertainty of the prediction
        kal.prediction_uncertainty[i] = kal.prediction_uncertainty[i] + pow(SAMPLING_TIME * kal.gyro_std_deviation, 2);

        // 3. calculate kalman gain
        kal.kalman_gain[i] = kal.prediction_uncertainty[i] / (kal.prediction_uncertainty[i] + kal.acc_std_deviation * kal.acc_std_deviation);

        // 4. update the kalman angle
        kal.kalman_angle[i] = kal.kalman_angle[i] + kal.kalman_gain[i] * (kal.acc_angle[i] - kal.kalman_angle[i]);

        // 5. update uncertainty of the predicted state
        kal.prediction_uncertainty[i] = (1 - kal.kalman_gain[i]) * kal.prediction_uncertainty[i];
    }

    // get yaw speed
    kal.yaw_speed = gyro[2] / double(GYRO_SENSITIVITY);
    kal.yaw_angle = kal.yaw_angle + SAMPLING_TIME * kal.yaw_speed;
    printf("%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%d\n", kal.acc_angle[0] + 45, kal.acc_angle[1] + 45, kal.kalman_angle[0], kal.kalman_angle[1], kal.yaw_speed, kal.yaw_angle,
            kal.sampling_count);

}

void output_human_read_data(int16_t* accel, int16_t* gyro, int16_t* temp)
{
    // These are the raw numbers from the chip, so will need tweaking to be really useful.
    // See the datasheet for more information
    printf("Acc. X = %d, Y = %d, Z = %d ", accel[0], accel[1], accel[2]);
    printf("Gyro. X = %d, Y = %d, Z = %d ", gyro[0], gyro[1], gyro[2]);
    // Temperature is simple so use the datasheet calculation to get deg C.
    // Note this is chip temperature.
    printf("Temp. = %f ", (*temp / 340.0) + 36.53);
  
    // 测试pitch roll yaw angle
    int roll_theta = atan(accel[1] / pow(accel[0] * accel[0] + accel[2] * accel[2], 0.5)) * 57.2958;
    printf("roll = %d ", roll_theta);
}
