#ifndef CONFIG_H_
#define CONFIG_H_

#include <hardware/i2c.h>

// =========================== hardware config ===========================
#define FLY_CTRL_I2C_INTS       i2c_default
#define FLY_CTRL_I2C_SDA        20
#define FLY_CTRL_I2C_SCL        21
#define FLY_CTRL_I2C_BAUDRATE   400 * 1000


#define FLY_CTRL_MOTOR0_PIN     29
#define FLY_CTRL_MOTOR1_PIN     22
#define FLY_CTRL_MOTOR2_PIN     7
#define FLY_CTRL_MOTOR3_PIN     0


// ===========================   pid setting   ===========================
// run angle loop at 100Hz
#define PID_ANGLE_MODE_FREQUENCY    100
// run rate loop at 200Hz
#define PID_RATE_MODE_FREQUENCY     200
// run pwm at 31.25kHz
#define PID_PWM_MOTOR_COUNTS        4000
#define PID_PWM_MOTOR_CLK_DIV       1

// angle loop PID value setup
#define PID_ANGLE_LOOP_ROLL_P       1
#define PID_ANGLE_LOOP_ROLL_I       2
#define PID_ANGLE_LOOP_ROLL_D       3

#define PID_ANGLE_LOOP_PITCH_P      4
#define PID_ANGLE_LOOP_PITCH_I      5
#define PID_ANGLE_LOOP_PITCH_D      6

#define PID_ANGLE_LOOP_YAW_P        7
#define PID_ANGLE_LOOP_YAW_I        8
#define PID_ANGLE_LOOP_YAW_D        9

// rate loop PID value setup
#define PID_RATE_LOOP_ROLL_P        11
#define PID_RATE_LOOP_ROLL_I        12
#define PID_RATE_LOOP_ROLL_D        13

#define PID_RATE_LOOP_PITCH_P       0.0005
#define PID_RATE_LOOP_PITCH_I       15
#define PID_RATE_LOOP_PITCH_D       16

#define PID_RATE_LOOP_YAW_P         17
#define PID_RATE_LOOP_YAW_I         18
#define PID_RATE_LOOP_YAW_D         19

#define PID_ANGLE_SAMPLEING_TIME    0.01
#define PID_RATE_SAMPLEING_TIME     0.005

/*
* used to compare with the saved value,
* if same than flash page is initialized, if not same than initialize flash
*/
char config_token[8] = {'a', 'c', 'c', 'd', 'e', 'f', 'g', 'h'};


/*
* be careful about memory alignment
*/
struct config_to_flash_t
{
    char token[8];                      // used to compare with saved value

    // sensor calibration area
    double bmi088_gyro_offset[3];       // bmi088 gyro offset

    // pid setting area
    double angle_rpy_pid[3][3];         // angle loop pid
    double rate_rpy_pid[3][3];          // rate loop pid
    
    double remaining_space[10];         // remaing space
};


#endif