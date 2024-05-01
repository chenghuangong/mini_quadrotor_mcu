#ifndef PID_CONTROLLER_H_
#define PID_CONTROLLER_H_

#include "PWM_MOTOR_CTRL.hpp"
#include "config.hpp"

/*
* define the pid object
*/
struct pid_obj
{
    double target;         //< set point
    double error;          //< error
    double previous_error; //< previous error
    double integ;          //< integral
    double deriv;          //< derivative

    double kp;             //< proportional gain
    double ki;             //< integral gain
    double kd;             //< derivative gain

    double outP;           //< proportional output (debugging)
    double outI;           //< integral output (debugging)
    double outD;           //< derivative output (debugging)

    double dt;             //< delta-time dt, sampling time

    double out;            // output
};

struct pid_contorller_t
{
    repeating_timer angle_controller_timer; 
    repeating_timer rate_controller_timer;

    // sensor feedback
    double rpy[3];                          // current angle, roll. pitch, yaw
    double current_rpy_rate[3];             // current rate, roll, pitch, yaw

    // pid value setting
    double angle_rpy_pid[3][3];             // angle_rpy_pid[0] = angle_roll_p, angle_roll_i, angle_roll_d
    double rate_rpy_pid[3][3];              // rate_rpy_pid[0] = rate_roll_p, rate_roll_i, rate_roll_d


    // angle target, set by user
    double rpy_angle_target[3];
    // rate target, set by angle controller, or set by user in rate mode
    double rpy_rate_target[3];
    double current_rpy_rate_target[3];
    double previous_rpy_rate_error[3];      // previous rate error
    double previous_rpy_rate_integral[3];
    // motor target, pid controller output, used to calculate output thrust
    double rpy_thrust_target[3];


    // test new function
    pid_obj rpy_angle_pid[3];
    pid_obj rpy_rate_pid[3];

    // motor thrust
    quad_pwm_motor_ctrl_t motors;               
    double basic_thrust[4];                 // controlled by remote controller or by height module (not added yet)
    double output_thrust[4];                // from 0 - 1, final thrust which to the motor directly 

    // temp
    uint16_t count = 0;
};


void pid_controller_pid_obj_initialize(pid_obj* pid, double target, double kp, double ki, double kd, double dt)
{  
    // initialize data
    pid->error = 0;
    pid->previous_error = 0;
    pid->integ = 0;
    pid->deriv = 0;
    pid->outP = 0;
    pid->outI = 0;
    pid->outD = 0;
    pid->out = 0;

    pid->target = target;
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->dt = dt;
}


void pid_controller_pid_update(pid_obj* pid, double* measured)
{
    // calculate error
    pid->error = pid->target - *measured;

    // calculate p
    pid->outP = pid->kp * pid->error;

    // calculate i, use average error value, or just use pid->error is also ok
    // TODO: need to constrain the integral value
    pid->integ += ((pid->error + pid->previous_error) / 2.0) * pid->dt;
    pid->outI = pid->ki * pid->integ;

    // calculate d
    // TODO: apply low pass filter to the deriv
    pid->deriv = (pid->error - pid->previous_error) / pid->dt;
    pid->outD = pid->kd * pid->deriv;

    // output
    pid->out = pid->outP + pid->outI + pid->outD;

    // update previous error
    pid->previous_error = pid->error;
}


uint8_t pid_controller_initialize(pid_contorller_t* ctrl)
{
    // initialize array
    for (size_t i = 0; i < 3; i++)
    {
        ctrl->rpy_angle_target[i] = 0;
        ctrl->rpy_rate_target[i] = 0;
        ctrl->current_rpy_rate_target[i] = 0;
        ctrl->previous_rpy_rate_error[i] = 0;
        ctrl->previous_rpy_rate_integral[i] = 0;

        ctrl->rpy[i] = 0;
        ctrl->current_rpy_rate[i] = 0;
        ctrl->rpy_thrust_target[i] = 0;                
    }

    // initialize pid obj
    for (size_t i = 0; i < 3; i++)
    {
        pid_controller_pid_obj_initialize(&ctrl->rpy_angle_pid[i], ctrl->rpy_angle_target[i], 
                                           ctrl->angle_rpy_pid[i][0], ctrl->angle_rpy_pid[i][1], ctrl->angle_rpy_pid[i][2], PID_ANGLE_SAMPLEING_TIME);
        pid_controller_pid_obj_initialize(&ctrl->rpy_rate_pid[i], ctrl->rpy_rate_target[i], 
                                           ctrl->rate_rpy_pid[i][0], ctrl->rate_rpy_pid[i][1], ctrl->rate_rpy_pid[i][2], PID_RATE_SAMPLEING_TIME);
    }
    
    // initialize motor thrust
    for (size_t i = 0; i < 4; i++)
    {
        ctrl->basic_thrust[i] = 0.05;
        ctrl->output_thrust[i] = 0;
    }

    // set motors
    uint8_t motor_pins[4] = { FLY_CTRL_MOTOR0_PIN, FLY_CTRL_MOTOR1_PIN, FLY_CTRL_MOTOR2_PIN, FLY_CTRL_MOTOR3_PIN };
    quad_pwm_motor_set_motor_pins(&ctrl->motors, motor_pins);
    quad_pwm_motor_set_pwm_config(&ctrl->motors, PID_PWM_MOTOR_COUNTS, PID_PWM_MOTOR_CLK_DIV);

    quad_pwm_motor_initialize(&ctrl->motors);
    quad_pwm_motor_enable(&ctrl->motors);

    // run motor self test
    quad_pwm_motor_self_test(&ctrl->motors);

    return 0;
}


void pid_controller_set_thrust(double* basic, double* rpy_thrust_target, double* result)
{
    result[0] = basic[0] - rpy_thrust_target[0] - rpy_thrust_target[1] + rpy_thrust_target[2];
    result[1] = basic[1] - rpy_thrust_target[0] + rpy_thrust_target[1] - rpy_thrust_target[2];
    result[2] = basic[2] + rpy_thrust_target[0] + rpy_thrust_target[1] + rpy_thrust_target[2];
    result[3] = basic[3] + rpy_thrust_target[0] - rpy_thrust_target[1] - rpy_thrust_target[2];

    for (size_t i = 0; i < 4; i++)
    {
        result[i] = result[i] < 0 ? 0 : (result[i] > 1 ? 1 : result[i]);
    }  
}


bool pid_controller_angle_controller(repeating_timer_t* t)
{
    pid_contorller_t* ctrl = static_cast<pid_contorller_t*>(t->user_data);

    // update pid, and output data to rate controller
    for (size_t i = 0; i < 3; i++)
    {
        pid_controller_pid_update(&ctrl->rpy_angle_pid[i], &ctrl->rpy[i]);
        ctrl->rpy_rate_target[i] = ctrl->rpy_angle_pid[i].out;
    }

    // code test
    // pid_controller_set_thrust(ctrl->basic_thrust, ctrl->rpy_rate_target, ctrl->output_thrust);
    // quad_pwm_motor_output(&ctrl->motors, ctrl->output_thrust);
    // if (ctrl->count++ == 50)
    // {
    //     ctrl->count = 0;
    //     printf("motor0 = %.3f, motor1 = %.3f, motor2 = %.3f, motor3 = %.3f\n", ctrl->output_thrust[0], ctrl->output_thrust[1], ctrl->output_thrust[2], ctrl->output_thrust[3]);
    // }
    
    return true;
}


bool pid_controller_rate_controller(repeating_timer_t* t)
{
    pid_contorller_t* ctrl = static_cast<pid_contorller_t*>(t->user_data);

    // update pid, and output data to rate controller
    for (size_t i = 0; i < 3; i++)
    {
        pid_controller_pid_update(&ctrl->rpy_rate_pid[i], &ctrl->current_rpy_rate[i]);
        ctrl->rpy_thrust_target[i] = ctrl->rpy_rate_pid[i].out;
    }

    // output thrust to motor
    // code test
    pid_controller_set_thrust(ctrl->basic_thrust, ctrl->rpy_thrust_target, ctrl->output_thrust);
    quad_pwm_motor_output(&ctrl->motors, ctrl->output_thrust);
    if (ctrl->count++ == 50)
    {
        ctrl->count = 0;
        //printf("motor0 = %.3f, motor1 = %.3f, motor2 = %.3f, motor3 = %.3f\n", ctrl->output_thrust[0], ctrl->output_thrust[1], ctrl->output_thrust[2], ctrl->output_thrust[3]);
    }

    return true;
}


uint8_t run_pid_controller(pid_contorller_t* ctrl)
{
    pid_controller_initialize(ctrl);
    // set out loop angle controller timer
    add_repeating_timer_ms(1000 / PID_ANGLE_MODE_FREQUENCY, &pid_controller_angle_controller, ctrl, &ctrl->angle_controller_timer);

    // set inner loop rate controller timer
    add_repeating_timer_ms(1000 / PID_RATE_MODE_FREQUENCY, &pid_controller_rate_controller, ctrl, &ctrl->rate_controller_timer);
    return 0;
}
#endif