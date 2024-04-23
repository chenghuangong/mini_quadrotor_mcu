#ifndef SENSOR_CONTORLLER_H_
#define SENSOR_CONTORLLER_H_

// uint Hz
#define SENSOR_DATA_ACQUSITION_FREQUENCY 200

#include <pico/stdlib.h>
#include "BMI088.hpp"
#include "BMP280.hpp"

struct sensor_controller_t
{
    bmi088 dev;

    repeating_timer timer;

    bool is_data_rdy = false;
    double data[2];

    // only used to notify task handler data arrived
    task_handler_t* handler = nullptr;
};

bool sensor_contorller_callback(repeating_timer_t* t)
{
    sensor_controller_t* ctrl = static_cast<sensor_controller_t*>(t->user_data);

    // read data from devs
    bmi088_get_rpy_from_kalman(&ctrl->dev);
    ctrl->data[0] = ctrl->dev.rpy_k[0];
    ctrl->data[1] = ctrl->dev.rpy_k[1];
    
    // call task handler function to handle data
    if (ctrl->handler)
    {
        controller_callback_func(ctrl->handler, controller_t::SENSOR);
    }
    return true;
}

uint8_t run_sensor_controller(sensor_controller_t* ctrl)
{
    // initialize dev
    bmi088_initialize(&ctrl->dev, i2c_default);
    bmi088_start_acc_sensor(&ctrl->dev);
    sleep_ms(10);
    bmi088_config config = bmi088_get_drone_config();
    bmi088_write_config(&ctrl->dev, &config);
    bmi088_read_range(&ctrl->dev);
    bmi088_get_gyro_offset(&ctrl->dev);

    add_repeating_timer_ms(1000 / SENSOR_DATA_ACQUSITION_FREQUENCY , sensor_contorller_callback, ctrl, &ctrl->timer);
    return 0;
}

#endif
