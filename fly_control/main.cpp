#include <stdio.h>
#include <pico/stdlib.h>
#include <hardware/uart.h>
#include <hardware/pwm.h>
#include <string>
#include "communicator.h"

// define motor as same as mpu6050 direction
#define MOTOR1_PIN 13
#define MOTOR2_PIN 10
#define MOTOR3_PIN 11
#define MOTOR4_PIN 12

#define SERVER_ADDR "192.168.31.17"
#define SERVER_PORT 12800


// define motor mosfet pin
const uint MOTOR_1 = MOTOR1_PIN;
const uint MOTOR_2 = MOTOR2_PIN;
const uint MOTOR_3 = MOTOR3_PIN;
const uint MOTOR_4 = MOTOR4_PIN;

quad_motor motors;
std::string cmd_buffer;

// only need call function below to set thrust, use offset
void set_motor_thrust_v2(quad_motor& motors, int* thrust);

void initialize_motor(quad_motor& motors);
void output_motor_thrust(quad_motor& motors);   // do set motor thrust to pwm
void on_uart_rx_v2();
void handle_cmd();

// timer callback
bool send_motor_data_callback(repeating_timer_t *rt)
{
    printf("{\"msg_type\":\"data\",\"dev_type\":\"drone\",\"source\":\"motor_thrust\",\"value\":[%d,%d,%d,%d]}", 
        motors.total_output_[0],
        motors.total_output_[1],
        motors.total_output_[2],
        motors.total_output_[3]);
    return true;    
}

bool heart_beat_callback(repeating_timer_t *rt)
{
    std::string msg = "{\"msg_type\":\"heart_beat\",\"dev_type\":\"drone\"}";
    uart_puts(uart0, msg.c_str());
    return true;    
}

int main()
{
    stdio_init_all();
    sleep_ms(10000);

    // start push gyro data to server
    // use pid controll motor
    communicator comm{SERVER_ADDR, SERVER_PORT};
    comm.start_push_sensor_data();

    // initialize motor
    motors.motor1_ = MOTOR_1;
    motors.motor2_ = MOTOR_2;
    motors.motor3_ = MOTOR_3;
    motors.motor4_ = MOTOR_4;
    initialize_motor(motors);
    comm.motor_ = &motors;

    // start data timer
    repeating_timer motor_data_timer;
    add_repeating_timer_ms(899, &send_motor_data_callback, nullptr, &motor_data_timer);
    repeating_timer heart_beat_timer;
    add_repeating_timer_ms(5123, &heart_beat_callback, nullptr, &heart_beat_timer);

    // set uart recv callback, uart0 has already initialized in communicator, so just set callback
    // irq_set_exclusive_handler(UART0_IRQ, on_uart_rx);
    irq_set_exclusive_handler(UART0_IRQ, on_uart_rx_v2);
    irq_set_enabled(UART0_IRQ, true);
    uart_set_irq_enables(uart0, true, false);


    while (1)
    {
        output_motor_thrust(motors);
        sleep_ms(100);
        tight_loop_contents();
    }

    return 0;
}


void initialize_motor(quad_motor& motors)
{
    gpio_set_function(motors.motor1_, GPIO_FUNC_PWM);
    gpio_set_function(motors.motor2_, GPIO_FUNC_PWM);
    gpio_set_function(motors.motor3_, GPIO_FUNC_PWM);
    gpio_set_function(motors.motor4_, GPIO_FUNC_PWM);

    uint slice_num_1 = pwm_gpio_to_slice_num(motors.motor1_);
    uint slice_num_2 = pwm_gpio_to_slice_num(motors.motor2_);
    uint slice_num_3 = pwm_gpio_to_slice_num(motors.motor3_);
    uint slice_num_4 = pwm_gpio_to_slice_num(motors.motor4_);

    // get pwm config
    pwm_config config = pwm_get_default_config();
    pwm_config_set_wrap(&config, 0xffff);
    pwm_config_set_clkdiv(&config, 40.f);
    
    // set duty cycle
    pwm_set_gpio_level(motors.motor1_, 0);
    pwm_set_gpio_level(motors.motor2_, 0);
    pwm_set_gpio_level(motors.motor3_, 0);
    pwm_set_gpio_level(motors.motor4_, 0);

    pwm_init(slice_num_1, &config, true);
    pwm_init(slice_num_2, &config, true);
    pwm_init(slice_num_3, &config, true);
    pwm_init(slice_num_4, &config, true);
}



void set_motor_thrust_v2(quad_motor& motors, int* thrust)
{
    // add thrust offset and original thrust value, save to temp thrust value
    for (size_t i = 0; i < 4; i++)
    {
        if (thrust[i] < 0 || thrust[i] > 255)
        {
            printf("error data\n");
            return;
        }
    }
    
    // if temp thrust value is valid, then write to motor thrust value
    for (size_t i = 0; i < 4; i++)
    {
        motors.total_output_[i] = thrust[i];
    }

    printf("set successful\n");
}

void output_motor_thrust(quad_motor& motors)
{
    pwm_set_gpio_level(motors.motor1_, motors.total_output_[0] * motors.total_output_[0]);
    pwm_set_gpio_level(motors.motor2_, motors.total_output_[1] * motors.total_output_[1]);
    pwm_set_gpio_level(motors.motor3_, motors.total_output_[2] * motors.total_output_[2]);
    pwm_set_gpio_level(motors.motor4_, motors.total_output_[3] * motors.total_output_[3]);
}


void on_uart_rx_v2()
{
    std::string value;
    while (uart_is_readable(uart0)) 
    {
        value += uart_getc(uart0);
    }

    // save to buffer
    cmd_buffer += value;

    // treat ';' as end of file
    if (value.find(';') != value.npos)
    {
        handle_cmd();
    }
}

void handle_cmd()
{
    if (cmd_buffer.length() > 33)
    {
        cmd_buffer.clear();
        return;
    }

    // printf(cmd_buffer.c_str());
    // printf("\n");

    std::string value = cmd_buffer;
    int thrust[4] = {0,0,0,0};
    int count = 0;
    size_t separator = 0;
    std::string number = "0";

    while ((separator = value.find(',')) != value.npos)
    {
        number = value.substr(0, separator);
        value = value.substr(separator + 1);
        thrust[count++] = std::stoi(number);
    }

    // printf("1:%d 2:%d 3:%d 4:%d\n", thrust[0], thrust[1], thrust[2], thrust[3]);
    set_motor_thrust_v2(motors, thrust);
    cmd_buffer.clear();
}