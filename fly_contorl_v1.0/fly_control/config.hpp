#ifndef CONFIG_H_
#define CONFIG_H_

#include <hardware/i2c.h>

#define FLY_CTRL_I2C_INTS       i2c_default
#define FLY_CTRL_I2C_SDA        20
#define FLY_CTRL_I2C_SCL        21
#define FLY_CTRL_I2C_BAUDRATE   400 * 1000


#define FLY_CTRL_MOTOR0_PIN     29
#define FLY_CTRL_MOTOR1_PIN     22
#define FLY_CTRL_MOTOR2_PIN     7
#define FLY_CTRL_MOTOR3_PIN     0


#endif