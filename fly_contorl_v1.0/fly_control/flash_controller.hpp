#ifndef FLASH_CONTROLLER_H_
#define FLASH_CONTROLLER_H_

#include <stdio.h>
#include "config.hpp"
#include "RP2040_FLASH.hpp"

struct config_controller_t
{
    rp2040_flash_t flash;
    config_to_flash_t config;

    bool is_initialized = true;     // is config initialized in flash
    bool is_modified = false;       // is config modified by host
};


/*
* for debug 
*/
void config_controller_print_config(config_controller_t* ctrl)
{
    printf("token is: ");
    for (size_t i = 0; i < 8; i++)
    {
        printf("%c, ", ctrl->config.token[i]);
    }

    printf("\n");

    // print angle pid value
    for (size_t i = 0; i < 3; i++)
    {
        switch (i)
        {
        case 0: printf("angle roll : "); break;
        case 1: printf("angle pitch: "); break;
        case 2: printf("angle yaw  : "); break;
        default: break;
        }
        printf("p: %-*f, i: %-*f, d: %-*f\n", 15, ctrl->config.angle_rpy_pid[i][0], 15, ctrl->config.angle_rpy_pid[i][1], 15, ctrl->config.angle_rpy_pid[i][2]);
    }

    // print rate pid value
    for (size_t i = 0; i < 3; i++)
    {
        switch (i)
        {
        case 0: printf("rate roll  : "); break;
        case 1: printf("rate pitch : "); break;
        case 2: printf("rate yaw   : "); break;
        default: break;
        }
        printf("p: %-*f, i: %-*f, d: %-*f\n", 15, ctrl->config.rate_rpy_pid[i][0], 15, ctrl->config.rate_rpy_pid[i][1], 15, ctrl->config.rate_rpy_pid[i][2]);
    }

    // print gyro data
    for (size_t i = 0; i < 3; i++)
    {
        printf("gyro: %.6f, ", ctrl->config.bmi088_gyro_offset[i]);
    }
}


/*
* task handler will write config
*/
uint8_t config_controller_write_config(config_controller_t* ctrl)
{
    if (!ctrl->is_initialized | ctrl->is_modified)
    {
        rp2040_flash_write_pages(&ctrl->flash);
    }
    
    return 0;
}


uint8_t config_controller_initialize(config_controller_t* ctrl)
{
    rp2040_flash_set_data_struct(&ctrl->flash, (uint8_t*) &ctrl->config, 1);

    // read data and compare to the token
    rp2040_flash_read_pages(&ctrl->flash);


    for (size_t i = 0; i < 8; i++)
    {
        ctrl->is_initialized = ctrl->config.token[i] == config_token[i];
        if(!ctrl->is_initialized) break; 
    }


    if (ctrl->is_initialized)
    {
        // debug code
        printf("flash initialized...\n");

        // test code
        // ctrl->config.bmi088_gyro_offset[0] += 1;
        // rp2040_flash_write_pages(&ctrl->flash);

    } else 
    {
        // initialize buffer
        printf("initialize...\n");
        for (size_t i = 0; i < 8; i++)
        {
            ctrl->config.token[i] = config_token[i];
        }
        
        // write data to buffer
        ctrl->config.angle_rpy_pid[0][0] = PID_ANGLE_LOOP_ROLL_P;
        ctrl->config.angle_rpy_pid[0][1] = PID_ANGLE_LOOP_ROLL_I;
        ctrl->config.angle_rpy_pid[0][2] = PID_ANGLE_LOOP_ROLL_D;

        ctrl->config.angle_rpy_pid[1][0] = PID_ANGLE_LOOP_PITCH_P;
        ctrl->config.angle_rpy_pid[1][1] = PID_ANGLE_LOOP_PITCH_I;
        ctrl->config.angle_rpy_pid[1][2] = PID_ANGLE_LOOP_PITCH_D;

        ctrl->config.angle_rpy_pid[2][0] = PID_ANGLE_LOOP_YAW_P;
        ctrl->config.angle_rpy_pid[2][1] = PID_ANGLE_LOOP_YAW_I;
        ctrl->config.angle_rpy_pid[2][2] = PID_ANGLE_LOOP_YAW_D;

        ctrl->config.rate_rpy_pid[0][0] = PID_RATE_LOOP_ROLL_P;
        ctrl->config.rate_rpy_pid[0][1] = PID_RATE_LOOP_ROLL_I;
        ctrl->config.rate_rpy_pid[0][2] = PID_RATE_LOOP_ROLL_D;

        ctrl->config.rate_rpy_pid[1][0] = PID_RATE_LOOP_PITCH_P;
        ctrl->config.rate_rpy_pid[1][1] = PID_RATE_LOOP_PITCH_I;
        ctrl->config.rate_rpy_pid[1][2] = PID_RATE_LOOP_PITCH_D;

        ctrl->config.rate_rpy_pid[2][0] = PID_RATE_LOOP_YAW_P;
        ctrl->config.rate_rpy_pid[2][1] = PID_RATE_LOOP_YAW_I;
        ctrl->config.rate_rpy_pid[2][2] = PID_RATE_LOOP_YAW_D; 
    }

    return 0;
}


uint8_t run_config_controller(config_controller_t* ctrl)
{
    config_controller_initialize(ctrl);
    return 0;
}

#endif