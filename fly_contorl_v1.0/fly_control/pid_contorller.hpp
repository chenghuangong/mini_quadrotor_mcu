#ifndef PID_CONTROLLER_H_
#define PID_CONTROLLER_H_

#include "PWM_MOTOR_CTRL.hpp"


#define PID_ANGLE_MODE_FREQUENCY    100
#define PID_RATE_MODE_FREQUENCY     200
#define PID_PWM_MOTOR_COUNTS        4000
#define PID_PWM_MOTOR_CLK_DIV       1


struct pid_contorller_t
{
    repeating_timer outloop_timer;          // for angle mode
    repeating_timer innerloop_timer;        // for rate mode


    quad_pwm_motor_ctrl_t motors;
    double rpy_angle_mode_output[3];        // it is the angle mode output, rate mode input
    double final_thrust[4];                 // from 0 - 1, final thrust which to the motor directly 

};


uint8_t pid_controller_initialize(pid_contorller_t* ctrl)
{
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


bool pid_controller_angle_mode(repeating_timer_t* t)
{
    // receive roll pitch yaw angle value
    // and use P controller to output the 
    // update the 
    return true;
}


bool pid_controller_rate_mode(repeating_timer_t* t)
{
    return true;
}


uint8_t run_pid_controller(pid_contorller_t* ctrl)
{
    // set out loop angle mode timer
    add_repeating_timer_ms(1000 / PID_ANGLE_MODE_FREQUENCY, &pid_controller_angle_mode, ctrl, &ctrl->outloop_timer);

    // set inner loop rate mode timer
    add_repeating_timer_ms(1000 / PID_RATE_MODE_FREQUENCY, &pid_controller_rate_mode, ctrl, &ctrl->innerloop_timer);

}
#endif