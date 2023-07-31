#ifndef COMMUNICATOR_H_
#define COMMUNICATOR_H_
#include <string>
#include "rp2040_interface.h"
#include "sensor.h"

#define PWM_COUNTS 65535
// #define PWM_MOTOR_MAX 2000
#define PWM_MOTOR_MAX 65000


struct quad_motor
{
    bool motor_on = false;  // control motor on or off
    bool pid_on = false;

    uint motor1_;
    uint motor2_;
    uint motor3_;
    uint motor4_;

    int total_output_[4] = {0, 0, 0, 0};            // from 0 - PWM_MOTOR_MAX
    double total_output_pid_[4] = {0, 0, 0, 0};     // from 0 - PWM_MOTOR_MAX
    
    int throttle = 0;     // set by controller
    double roll_set_deg = 0;  // set by controller     
    double pitch_set_deg = 0; // set by controller
    double yaw_set_deg = 0;   // set by controller


    // rate controll
    int roll_input = 0;   // controlled by communicator mpu6050
    int pitch_input = 0;  // controlled by communicator mpu6050
    int yaw_input = 0;    // controlled by communicator mpu6050
    
    // PID value

    double p_roll = -0.1;
    double i_roll = 0;
    double d_roll = 0;

    double p_pitch = -0.1;
    double i_pitch = 0;
    double d_pitch = 0;

    double p_yaw = 0.05;
    // 加入yaw的积分的效果不好
    // double i_yaw = 0.001;
    double i_yaw = 0;
    double d_yaw = 0;

    double p_z = 0;
    double i_z = 0;
    double d_z = 0;

    
    // use throttle roll pitch yaw input value to calculate total output,
    // then apply total output to four motors 
    void convert_to_total_output()
    {
        total_output_[0] = throttle - roll_input - pitch_input + yaw_input;
        total_output_[1] = throttle - roll_input + pitch_input - yaw_input;
        total_output_[2] = throttle + roll_input + pitch_input + yaw_input;
        total_output_[3] = throttle + roll_input - pitch_input - yaw_input;

        // also convert to double
        for (size_t i = 0; i < 4; i++)
        {
            total_output_pid_[i] = total_output_[i];
        }
        
    }

    void update_input_error(double ethrottle, double eroll, double epitch, double eyaw)
    {
        // error is offset
        total_output_pid_[0] = total_output_pid_[0] + ethrottle - eroll - epitch + eyaw;
        total_output_pid_[1] = total_output_pid_[1] + ethrottle - eroll + epitch - eyaw;
        total_output_pid_[2] = total_output_pid_[2] + ethrottle + eroll + epitch + eyaw;
        total_output_pid_[3] = total_output_pid_[3] + ethrottle + eroll - epitch - eyaw;
        // convert double output to int
        for (size_t i = 0; i < 4; i++)
        {
            if (total_output_pid_[i] > 0 && total_output_pid_[i] < PWM_MOTOR_MAX)
            {
                total_output_[i] = static_cast<int>(total_output_pid_[i]);
            }
        }
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


private:
    esp01s esp_;
    i2c_communicator i2c_communicator_;
    repeating_timer data_timer_;    // push sensor data
    repeating_timer sensor_timer_;  // read sensor data

public:
    // motor
    quad_motor* motor_ = nullptr;
    // sensor
    // sensor使用了i2c_communicator进行初始化，这里需要修改掉，将sensor放到i2c_communicator之前无法运行
    // 虽然应该是没有关系的
    sensor_bmp280 sensor_pressure_;
    sensor_mpu6050 sensor_gyro_; 


    // only for test, test sampling time
    int sampling_count = 0; 
 
};


#endif
