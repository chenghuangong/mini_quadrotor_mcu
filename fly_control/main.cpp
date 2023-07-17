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
#define CMD_BUFFER_SIZE 128


// define motor mosfet pin
const uint MOTOR_1 = MOTOR1_PIN;
const uint MOTOR_2 = MOTOR2_PIN;
const uint MOTOR_3 = MOTOR3_PIN;
const uint MOTOR_4 = MOTOR4_PIN;

quad_motor motors;
std::string cmd_buffer;
size_t cmd_char_buffer_length;
unsigned char cmd_char[8];
unsigned char cmd_char_buffer[CMD_BUFFER_SIZE];


// only need call function below to set thrust, use offset
void set_motor_thrust_v2(quad_motor& motors, int* thrust);

void initialize_motor(quad_motor& motors);
void output_motor_thrust(quad_motor& motors);   // do set motor thrust to pwm
void on_uart_rx_v2();
void check_buffer_cmd();
void handle_cmd();
void handle_cmd_v2();
void print_char_to_number(unsigned char* data, size_t length);

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
            return;
        }
    }
    
    // if temp thrust value is valid, then write to motor thrust value
    for (size_t i = 0; i < 4; i++)
    {
        motors.total_output_[i] = thrust[i];
    }

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
    // printf("uart_rx called\n");

    while (uart_is_readable(uart0) && cmd_char_buffer_length < (CMD_BUFFER_SIZE - 1)) 
    {
        cmd_char_buffer[cmd_char_buffer_length++] = uart_getc(uart0);
        printf("%d ", cmd_char_buffer[cmd_char_buffer_length - 1]);
    }

    // printf("\nuart_rx finished\n");

    // command length is 8
    if (cmd_char_buffer_length >= 8 && (cmd_char_buffer_length <= (CMD_BUFFER_SIZE - 1)))
    {
        // if there is a cmd in buffer, handle it
        // print_char_to_number(cmd_char_buffer, cmd_char_buffer_length);
        check_buffer_cmd();
    } else if (cmd_char_buffer_length >= CMD_BUFFER_SIZE)
    {
        // buffer is full, empty buffer
        cmd_char_buffer_length = 0;
    }
}


void check_buffer_cmd()
{
    // check frame head
    for (size_t i = 0; i < (cmd_char_buffer_length - 1); i++)
    {
        // find frame head or not
        if (cmd_char_buffer[i] == 0xAA && cmd_char_buffer[i + 1] == 0xBB)
        {   
            // whole data came in or not
            if ((i + 8) <= cmd_char_buffer_length)
            {
                // calculate check sum
                uint16_t sum = 0;
                for (size_t j = i; j < (i + 7); j++)
                {
                    sum += cmd_char_buffer[j];
                }

                // checksum correct or not
                if (uint8_t(sum) == cmd_char_buffer[i + 7])
                {
                    // move data to cmd
                    for (size_t k = 0; k < 8; k++)
                    {
                        cmd_char[k] = cmd_char_buffer[i + k];
                    }
                    handle_cmd_v2();          
                }
                cmd_char_buffer_length = 0;
                return;
            }
            // lack of data, break
            break;
        }
    }   
}

void handle_cmd()
{
    if (cmd_buffer.length() > 33)
    {
        cmd_buffer.clear();
        return;
    }

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


// command format should be changed to 
// format total 8 bytes
// cmd = 0x01: directly set all motor, the motor value is value1, value2, value3, value4
// cmd = 0x02: set throttle, only use value1
// cmd = 0x03: set roll, only use value1
// cmd = 0x04: set pitch, only use value1
// cmd = 0x05: set yaw, only use value1
// AA BB CMD VALUE1 VALUE2 VALUE3 VALUE4 CHECKSUM
void handle_cmd_v2()
{
    int thrust[4] = {0, 0, 0, 0};

    // printf("HANDLE CMD!!!!/n");

    switch (cmd_char[2])
    {
    case 0x01:
        // set all motor
        // first off pid
        motors.pid_on = false;
        for (size_t i = 0; i < 4; i++)
        {
            thrust[i] = cmd_char[3 + i];
        }
        set_motor_thrust_v2(motors, thrust);
        break;
    case 0x02:
        // set throttle
        motors.pid_on = true;
        motors.throttle = cmd_char[3];
        motors.convert_to_total_output();
    break;
    case 0x03:break;
    case 0x04:break;
    case 0x05:break;
    default:
        break;
    }
}

void print_char_to_number(unsigned char* data, size_t length)
{
    printf("data is: ");
    for (size_t i = 0; i < length; i++)
    {
        printf("%d ", data[i]);
    }
    printf("\n");
}