#ifndef COMMUNICATOR_H_
#define COMMUNICATOR_H_
#include <string>
#include "rp2040_interface.h"
#include "sensor.h"

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

    // sensor
    sensor_bmp280 sensor_pressure_;
    sensor_mpu6050 sensor_gyro_;
};


#endif
