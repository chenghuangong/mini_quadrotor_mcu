#ifndef SENSOR_H_
#define SENSOR_H_
#include "rp2040_interface.h"
#include <cmath>

// #define MPU6050_SAMPLING_TIME 0.01  // 采样时间10ms
#define MPU6050_SAMPLING_TIME 0.004    // 采样时间4ms
#define ACC_SENSITIVITY 16384          // per g
#define ACC_STD_DEVIATION 3            // °
#define GYRO_SENSITIVITY 65.5          // per °/s
#define GYRO_STD_DEVIATION 4           // °/s



class sensor_bmp280
{
public:
    sensor_bmp280(i2c_communicator* communicator);

public:
    double* get_sensor_data();

private:
    void init_sensor();
    void init_bmp280();                      
    void get_calib_params();    // 用于对温度和气压进行校准
    int32_t convert(int32_t temp);
    int32_t convert_temp(int32_t temp);
    int32_t convert_pressure(int32_t pressure, int32_t temp);

private:
    i2c_communicator* communicator_;
    struct bmp280_calib_param 
    {
        // temperature params
        uint16_t dig_t1;
        int16_t dig_t2;
        int16_t dig_t3;
        // pressure params
        uint16_t dig_p1;
        int16_t dig_p2;
        int16_t dig_p3;
        int16_t dig_p4;
        int16_t dig_p5;
        int16_t dig_p6;
        int16_t dig_p7;
        int16_t dig_p8;
        int16_t dig_p9;
    };
    bmp280_calib_param calib_param_;

    double sensor_data[2] = {0, 0};
    bool is_initialized_ = false;
};

// ======================================MPU6050======================================
struct mpu6050_kalman_data
{
    double ctrl_matrix = MPU6050_SAMPLING_TIME;         // control matrix
    double acc_std_deviation = ACC_STD_DEVIATION;
    double gyro_std_deviation = GYRO_STD_DEVIATION;

    // [roll, pitch]
    double kalman_angle[2] = {0, 0};                    // angle predicted by kalman fiter, use gyro to calculate
    double kalman_gain[2] = {0, 0};
    double prediction_uncertainty[2] = {0, 0};
    double acc_angle[2] = {0, 0};                       // calculated by accelerometer

    // calculate roll and pitch by kalman filter
    double kalman_previous_angel[2] = {0, 0};
    double kalman_angel_speed[2] = {0, 0};

    double yaw_speed = 0;                               // show yaw speed, do not integrate yaw
    double yaw_angle = 0;

    // optional
    int sampling_count = 0;
    int raw_data_sum[6] = {0, 0, 0, 0, 0, 0};
    int raw_data_offset[6] = {0, 0, 0, 0, 0, 0};

    double temperature = 0;
};


class sensor_mpu6050
{
public:
    sensor_mpu6050(i2c_communicator* communicator);

public:
    void mpu6050_read_kalman();     // read raw and apply kalman filter
    double* get_sensor_raw_data();
    double* get_sensor_gyro_speed();
    double* get_sensor_kalman_data();
    
private:
    void init_sensor();
    void init_mpu6050();
    void mpu6050_set_low_pass_filter();
    void mpu6050_read_raw();        // read raw and save, time interval 10ms
    void apply_kalman_filter();
    bool perform_zero_point_calibration(); 
    
private:
    i2c_communicator* communicator_;
    i2c_inst_t* i2c_inst_;
    bool is_initialized_ = false;

    int16_t accel_[3], gyro_[3], temp_;
    int16_t accel_no_offset_[3], gyro_no_offset_[3];
    mpu6050_kalman_data kalman_filter_;
    double xyz_acc_gyro_data_[7];   // acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, temperature, raw data with offset
    double xyz_gyro_speed_[3];      // gyro_x, gyro_y, gyro_z, directly calculate xyz rotation speed, uint °/s
    double rpy_kalman_data_[7];     // roll, pitch, yaw(integrate gyro_z), temperature, roll_speed, pitch_speed, yaw_speed
};


#endif