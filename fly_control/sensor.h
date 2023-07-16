#ifndef SENSOR_H_
#define SENSOR_H_
#include "rp2040_interface.h"
#include <math.h>

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


class sensor_mpu6050
{
public:
    sensor_mpu6050(i2c_communicator* communicator);

public:
    double* get_sensor_raw_data();
    double* get_sensor_kalman_data();
    double* get_sensor_all_data() {return nullptr;}
    void mpu6050_read_raw();        // read raw and save, time interval 10ms 

private:
    void init_sensor();
    void init_mpu6050();
    void apply_kalman_filter(); 
    
private:
    struct kalman_data
    {
        double integral_time = 0.01;                // 10ms
        double ctrl_matrix = integral_time;         // control matrix
        double inter_matrix = 0;                    // intermediate matrix

        // first variable for roll, second variable for pitch
        double kal_theta[2] = {0, 0};               // theta after kalman fiter, kalman_data[0] = roll, kalman_data[1] = pitch
        double cal_theta[3] = {0, 0, 0};            // theta cal by acc
        double prediction_uncertainty[2] = {0, 0};  // predict uncertainty, 
        double kalman_gain[2] = {0, 0};

        double temperature = 0;
    };

    i2c_communicator* communicator_;
    i2c_inst_t* i2c_inst_;
    bool is_initialized_ = false;

    int16_t accel_[3], gyro_[3], temp_;
    kalman_data kalman_filter_;
    double raw_data_[7];
    double kal_data_[4]; // roll, pitch, yaw(暂时没有), temperature
};


#endif