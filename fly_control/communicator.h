#ifndef COMMUNICATOR_H_
#define COMMUNICATOR_H_
#include <string>
#include "rp2040_interface.h"
#include "sensor.h"


struct quad_motor
{
    bool motor_on = false;  // control motor on or off
    bool pid_on = false;

    uint motor1_;
    uint motor2_;
    uint motor3_;
    uint motor4_;

    int total_output_[4] = {0, 0, 0, 0};
    
    int throttle = 0;     // set by controller
    double roll_deg = 0;  // set by controller     
    double pitch_deg = 0; // set by controller
    double yaw_deg = 0;   // set by controller

    int roll_input = 0;   // controlled by communicator mpu6050
    int pitch_input = 0;  // controlled by communicator mpu6050
    int yaw_input = 0;    // controlled by communicator mpu6050

    // use throttle roll pitch yaw input value to calculate total output,
    // then apply total output to four motors 
    void convert_to_total_output()
    {
        total_output_[0] = throttle - roll_input - pitch_input - yaw_input;
        total_output_[1] = throttle - roll_input + pitch_input + yaw_input;
        total_output_[2] = throttle + roll_input + pitch_input - yaw_input;
        total_output_[3] = throttle + roll_input - pitch_input + yaw_input;
    }
};

/*
1. handle sensor data, transfer data to host by certain time interval,
2. receive command from host, and send command to corresponding components
*/

class communicator
{
friend bool push_sensor_data_callback(repeating_timer_t* rt);
friend bool collect_sensor_data_callback(repeating_timer_t* rt);

public:
    communicator(const std::string& addr, const uint16_t& port);

public:
    void start_push_sensor_data();        // send sensor data to host
    std::string get_sensor_data(); 

public:
    // motor
    quad_motor* motor_ = nullptr;

private:
    esp01s esp_;
    i2c_communicator i2c_communicator_;
    repeating_timer data_timer_;    // push sensor data
    repeating_timer sensor_timer_;  // read sensor data

    // sensor
    sensor_bmp280 sensor_pressure_;
    sensor_mpu6050 sensor_gyro_;
};


#endif
