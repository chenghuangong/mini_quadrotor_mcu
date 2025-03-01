#ifndef TASK_HANDLER_H_
#define TASK_HANDLER_H_

#include "config.hpp"
#include "public_func.hpp"
#include "sensor_controller.hpp"
#include "pid_contorller.hpp"
#include "comm_controller.hpp"
#include "status_controller.hpp"
#include "flash_controller.hpp"

// uint Hz
#define TASK_RUNNING_SPEED 1

struct task_handler_t
{
    repeating_timer timer;

    // task list
    task_ptr tasks[8];
    uint8_t task_count = 0;

    // sensor list
    sensor_controller_t sensor_ctrl;
    pid_contorller_t    pid_ctrl;
    comm_controller_t*  comm_ctrl = &comm_controller;
    config_controller_t config_ctrl;


    // set interlock
    bool is_protection_triggered = false;

    uint8_t temp = 0;
};


/*
* timer will running at 1kHz, run the tasks, such as interlock
*/
bool task_handler_callback(repeating_timer_t* t)
{
    task_handler_t* handler = static_cast<task_handler_t*>(t->user_data);
    
    for (size_t i = 0; i < handler->task_count; i++)
    {
        handler->tasks[i](handler);
    }
    return true;
}


/*
* copy data ptr to some read only modules
*/
uint8_t task_handler_copy_data_ptr(task_handler_t* handler)
{
    // copy data to communication controller
    handler->comm_ctrl->rpy_ptr = handler->sensor_ctrl.rpy;
    handler->comm_ctrl->rpy_rate_ptr = handler->sensor_ctrl.rpy_rate;
    handler->comm_ctrl->motor_thrust_ptr = handler->pid_ctrl.output_thrust;
    return 0;
}


/*
* this is sensor calibration data, not register config
* maybe should also add sensor register config
* load config data to sensor or calibrate by sensor
*/
uint8_t task_handler_load_sensor_config(task_handler_t* handler)
{
    // check config
    if (handler->config_ctrl.is_initialized)
    {
        // read gyro offset
        printf("read gyro zero point from flash...\n");
        for (size_t i = 0; i < 3; i++)
        {
            handler->sensor_ctrl.dev.gyro_offset[i] = handler->config_ctrl.config.bmi088_gyro_offset[i];
        }
    }
    else
    {
        printf("read gyro zero point from sensor...\n");
        bmi088_get_gyro_offset(&handler->sensor_ctrl.dev);
        for (size_t i = 0; i < 3; i++)
        {
            handler->config_ctrl.config.bmi088_gyro_offset[i] = handler->sensor_ctrl.dev.gyro_offset[i];
        }
    }
    return 0;
}


/*
* load pid value to pid controller
*/
uint8_t task_handler_load_pid_config(task_handler_t* handler)
{
    printf("read pid value from flash...\n");
    for (size_t i = 0; i < 3; i++)
    {
        for (size_t j = 0; j < 3; j++)
        {
            handler->pid_ctrl.angle_rpy_pid[i][j] = handler->config_ctrl.config.angle_rpy_pid[i][j];
            handler->pid_ctrl.rate_rpy_pid[i][j] = handler->config_ctrl.config.rate_rpy_pid[i][j];
        }
    }
    return 0;
}



uint8_t task_handler_initialize(task_handler_t* handler)
{
    // initialize I2C
    i2c_init(FLY_CTRL_I2C_INTS, FLY_CTRL_I2C_BAUDRATE);
    gpio_set_function(FLY_CTRL_I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(FLY_CTRL_I2C_SCL, GPIO_FUNC_I2C);

    // transfer handler pointer to controller
    // when controller receive data, automaticly notify handler
    handler->sensor_ctrl.handler = handler;
    handler->comm_ctrl->handler = handler;

    // run flash
    run_config_controller(&handler->config_ctrl);

    // just initialize sensor, run it later, for calibration purpose
    sensor_controller_initialize(&handler->sensor_ctrl);
    sleep_ms(10);
    
    // load config
    task_handler_load_sensor_config(handler);
    task_handler_load_pid_config(handler);
    config_controller_write_config(&handler->config_ctrl);
    config_controller_print_config(&handler->config_ctrl);



    
    // run sensor after read offset from flash or by sensor
    run_sensor_controller(&handler->sensor_ctrl);

    // run pid controller (should check)
    run_pid_controller(&handler->pid_ctrl);
    sleep_ms(10);

    // run communicator (should check)
    run_comm_controller(handler->comm_ctrl);
    sleep_ms(10);
    task_handler_copy_data_ptr(handler);

    return 0;
}


/*
* start timer to run the tasks
*/
uint8_t run_task_handler(task_handler_t* handler)
{
    add_repeating_timer_ms(1000 / TASK_RUNNING_SPEED, task_handler_callback, handler, &handler->timer);
    return 0;
}


/*
* add task to task handler
*/
uint8_t add_task(task_handler_t* handler, task_ptr task)
{
    if (handler->task_count > 7)
    {
        return 1;
    }
    handler->tasks[handler->task_count++] = task;
    return 0;
}


/*
* define tasks here
*/
uint8_t task_test(task_handler_t* handler)
{
    printf("task1\n");
    return 0;
}


/*
* running safety logic here
*/
uint8_t drone_guardian(task_handler_t* handler)
{
    // if sensor data out of limit
    // stop motor immediately
    // set is protection = true

    if (handler->is_protection_triggered)
    {
        // TODO
        // stop motor
        return 0;
    }
    return 0;
}


/* =====================================CALLBACK=====================================
* controller auto notify function
*/
uint8_t sensor_data_handler(task_handler_t* handler)
{
    // 1. stop drone if tilt more than 45deg
    drone_guardian(handler);

    // 2. transfer data to PID controller
    for (size_t i = 0; i < 3; i++)
    {
        handler->pid_ctrl.rpy[i] = handler->sensor_ctrl.rpy[i];
        // handler->pid_ctrl.previous_rpy_rate[i] = handler->pid_ctrl.current_rpy_rate[i];
        handler->pid_ctrl.current_rpy_rate[i] = handler->sensor_ctrl.rpy_rate[i];
    }

    // debug code
    // if(handler->temp++ == 20)
    // {
    //     printf("roll = %.2f, pitch = %.2f\n", handler->sensor_ctrl.rpy[0], handler->sensor_ctrl.rpy[1]);
    //     handler->temp = 0;
    // }
    return 0;
}


uint8_t comm_data_handler(task_handler_t* handler)
{
    // do something
    return 0;
}


uint8_t pid_data_handler(task_handler_t* handler)
{
    // do something
    return 0;
}


void controller_callback_func(task_handler_t* handler, controller_t ctrl)
{
    switch (ctrl)
    {
    case controller_t::SENSOR: sensor_data_handler(handler);break;
    case controller_t::PID: comm_data_handler(handler);break;
    case controller_t::COMM: pid_data_handler(handler);break;
    default:
        break;
    }
}


#endif





