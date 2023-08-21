#ifndef CONFIG_H_
#define CONFIG_H_

// =======================================================RP2040=======================================================
// for uart
#define UART_ID uart0
// #define UART_BAUD_RATE 115200
#define UART_BAUD_RATE 230400
#define UART_TX_PIN 0
#define UART_RX_PIN 1

// for i2c 
#define I2C_INST i2c_default
#define SDA_PIN 4
#define SCL_PIN 5

// status indicator led
#define LED_PIN PICO_DEFAULT_LED_PIN
// =======================================================RP2040======================================================= 



// =======================================================BMP280=======================================================
enum class BMP280_REG
{
    // I2C address
    ADDR = _u(0x76),        
    
    // hardware registers
    REG_CONFIG = _u(0xF5),   
    REG_CTRL_MEAS = _u(0xF4),
    REG_RESET = _u(0xE0),

    REG_TEMP_XLSB = _u(0xFC),
    REG_TEMP_LSB = _u(0xFB),
    REG_TEMP_MSB = _u(0xFA),

    REG_PRESSURE_XLSB = _u(0xF9),
    REG_PRESSURE_LSB = _u(0xF8),
    REG_PRESSURE_MSB = _u(0xF7),

    // calibration registers
    // LSB 和 MSB表示两种BIT到达的顺序
    // 例如0x12h, 00010010b，用MSB表示时，到达的顺序是00010010b，用LSB表示时，到达的顺序是01001000b
    // 注意下面的LSB和MSB仅仅表示数据的大头和小头部分，而不是数据的到达序列！！！
    REG_DIG_T1_LSB = _u(0x88),
    REG_DIG_T1_MSB = _u(0x89),
    REG_DIG_T2_LSB = _u(0x8A),
    REG_DIG_T2_MSB = _u(0x8B),
    REG_DIG_T3_LSB = _u(0x8C),
    REG_DIG_T3_MSB = _u(0x8D),
    REG_DIG_P1_LSB = _u(0x8E),
    REG_DIG_P1_MSB = _u(0x8F),
    REG_DIG_P2_LSB = _u(0x90),
    REG_DIG_P2_MSB = _u(0x91),
    REG_DIG_P3_LSB = _u(0x92),
    REG_DIG_P3_MSB = _u(0x93),
    REG_DIG_P4_LSB = _u(0x94),
    REG_DIG_P4_MSB = _u(0x95),
    REG_DIG_P5_LSB = _u(0x96),
    REG_DIG_P5_MSB = _u(0x97),
    REG_DIG_P6_LSB = _u(0x98),
    REG_DIG_P6_MSB = _u(0x99),
    REG_DIG_P7_LSB = _u(0x9A),
    REG_DIG_P7_MSB = _u(0x9B),
    REG_DIG_P8_LSB = _u(0x9C),
    REG_DIG_P8_MSB = _u(0x9D),
    REG_DIG_P9_LSB = _u(0x9E),
    REG_DIG_P9_MSB = _u(0x9F),
    NUM_CALIB_PARAMS = 24        // number of calibration registers to be read
};
// =======================================================BMP280=======================================================


// =======================================================MPU6050=======================================================
#define MPU6050_ADDR 0x68
// =======================================================MPU6050=======================================================

#endif