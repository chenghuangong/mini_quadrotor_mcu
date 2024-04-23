#ifndef TASK_HANDLER_H_
#define TASK_HANDLER_H_

#include "config.hpp"
#include "public_func.hpp"
#include "sensor_controller.hpp"
#include "pid_contorller.hpp"
#include "comm_controller.hpp"
#include "status_controller.hpp"

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


uint8_t task_handler_initialize(task_handler_t* handler)
{
    // initialzie I2C
    i2c_init(FLY_CTRL_I2C_INTS, FLY_CTRL_I2C_BAUDRATE);
    gpio_set_function(FLY_CTRL_I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(FLY_CTRL_I2C_SCL, GPIO_FUNC_I2C);


    // transfer handler pointer to controller
    // when controller receive data, automaticly notify handler
    handler->sensor_ctrl.handler = handler;


    // run sensor
    run_sensor_controller(&handler->sensor_ctrl);
    sleep_ms(10);
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
    drone_guardian(handler);

    // transfer data to PID controller

    if(handler->temp++ == 20)
    {
        printf("roll = %.2f, pitch = %.2f\n", handler->sensor_ctrl.data[0], handler->sensor_ctrl.data[1]);
        handler->temp = 0;
    }
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





