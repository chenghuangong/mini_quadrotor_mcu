#include <stdio.h>
#include <pico/stdlib.h>
#include <hardware/uart.h>
#include <hardware/pwm.h>
#include <string.h>
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

const double MOTOR_COE_1 = 0.96;
const double MOTOR_COE_2 = 1;
const double MOTOR_COE_3 = 1;
const double MOTOR_COE_4 = 0.96;

const uint PWM_STEP = PWM_COUNTS / PWM_MOTOR_MAX;

quad_motor motors;
std::string cmd_buffer;
size_t cmd_char_buffer_length;
unsigned char cmd_char[8];
unsigned char cmd_char_buffer[CMD_BUFFER_SIZE];

int motor_set_count = 0;


// only need call function below to set thrust, use offset
void set_motor_thrust_v2(quad_motor& motors, int* thrust);

void initialize_motor(quad_motor& motors);
void output_motor_thrust(quad_motor& motors);   // do set motor thrust to pwm
void on_uart_rx_v2();
float get_float_from_cmd(unsigned char*);
void check_buffer_cmd();
void handle_cmd();
void handle_cmd_v2();
void print_char_to_number(unsigned char* data, size_t length);

// timer callback
bool send_motor_data_callback(repeating_timer_t *rt)
{
    // printf("{\"msg_type\":\"data\",\"dev_type\":\"drone\",\"source\":\"motor_thrust\",\"value\":[%d,%d,%d,%d]}", 
    //     motors.total_output_[0],
    //     motors.total_output_[1],
    //     motors.total_output_[2],
    //     motors.total_output_[3]);
    std::string msg = "{\"msg_type\":\"data\",\"dev_type\":\"drone\",\"source\":\"motor_thrust\",\"value\":[" + 
        std::to_string(motors.total_output_[0]) + "," + 
        std::to_string(motors.total_output_[1]) + "," + 
        std::to_string(motors.total_output_[2]) + "," + 
        std::to_string(motors.total_output_[3]) + "]}";
    uart_puts(uart0, msg.c_str());
    return true;    
}

bool heart_beat_callback(repeating_timer_t *rt)
{
    auto comm = static_cast<communicator*>(rt->user_data);

    std::string msg = "{\"msg_type\":\"heart_beat\",\"dev_type\":\"drone\"}";
    uart_puts(uart0, msg.c_str());

    // printf("The sampling count is: %d\nThe motor count is: %d\n", comm->sampling_count, motor_set_count);

    return true;    
}

bool output_motor_thrust_callback(repeating_timer_t* rt)
{
    output_motor_thrust(motors);
    // motor_set_count++;
    return true;
}

int main()
{
    stdio_init_all();
    sleep_ms(10000);
    // printf("start\n");
    // start push gyro data to server
    // use pid controll motor
    communicator comm{SERVER_ADDR, SERVER_PORT};
    // printf("initialize communicator\n");
    comm.start_push_sensor_data();
    // printf("start communicator\n");

    // initialize motor
    motors.motor1_ = MOTOR_1;
    motors.motor2_ = MOTOR_2;
    motors.motor3_ = MOTOR_3;
    motors.motor4_ = MOTOR_4;
    initialize_motor(motors);
    // printf("initialize motor\n");
    comm.motor_ = &motors;

    // start data timer
    repeating_timer motor_data_timer;
    add_repeating_timer_ms(899, &send_motor_data_callback, nullptr, &motor_data_timer);
    repeating_timer heart_beat_timer;
    add_repeating_timer_ms(5001, &heart_beat_callback, &comm, &heart_beat_timer);
    // printf("start send data\n");

    // set uart recv callback, uart0 has already initialized in communicator, so just set callback
    // irq_set_exclusive_handler(UART0_IRQ, on_uart_rx);
    irq_set_exclusive_handler(UART0_IRQ, on_uart_rx_v2);
    irq_set_enabled(UART0_IRQ, true);
    uart_set_irq_enables(uart0, true, false);

    repeating_timer motor_output_timer;
    add_repeating_timer_ms(4, &output_motor_thrust_callback, nullptr, &motor_output_timer);

    while (1)
    {
        tight_loop_contents();
        // output_motor_thrust(motors);
        // sleep_ms(4);
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
    pwm_config_set_wrap(&config, 0xffff);   // set duty cycle
    pwm_config_set_clkdiv(&config, 40.f);   // set 125Mhz / 40 = 3.125MHz
    
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
        if (thrust[i] < 0 || thrust[i] > PWM_MOTOR_MAX)
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
    // convert duty cycle to linear, from 0 - PWM_MOTOR_MAX

    // pwm_set_gpio_level(motors.motor1_, motors.total_output_[0] * motors.total_output_[0]);
    // pwm_set_gpio_level(motors.motor2_, motors.total_output_[1] * motors.total_output_[1]);
    // pwm_set_gpio_level(motors.motor3_, motors.total_output_[2] * motors.total_output_[2]);
    // pwm_set_gpio_level(motors.motor4_, motors.total_output_[3] * motors.total_output_[3]);

    pwm_set_gpio_level(motors.motor1_, motors.total_output_[0] * PWM_STEP * MOTOR_COE_1);
    pwm_set_gpio_level(motors.motor2_, motors.total_output_[1] * PWM_STEP * MOTOR_COE_2);
    pwm_set_gpio_level(motors.motor3_, motors.total_output_[2] * PWM_STEP * MOTOR_COE_3);
    pwm_set_gpio_level(motors.motor4_, motors.total_output_[3] * PWM_STEP * MOTOR_COE_4);
}


void on_uart_rx_v2()
{
    // printf("uart_rx called\n");

    uint temp_length = cmd_char_buffer_length;

    while (uart_is_readable(uart0) && cmd_char_buffer_length < (CMD_BUFFER_SIZE - 1)) 
    {
        cmd_char_buffer[cmd_char_buffer_length++] = uart_getc(uart0);
        printf("%d ", cmd_char_buffer[cmd_char_buffer_length - 1]);
    }

    // no data received
    if (temp_length == cmd_char_buffer_length) return;

    // printf("\nuart_rx finished\n");

    // command length is 8
    if (cmd_char_buffer_length >= 8 && (cmd_char_buffer_length <= (CMD_BUFFER_SIZE - 1)))
    {
        // if there is a cmd in buffer, handle it
        // print_char_to_number(cmd_char_buffer, cmd_char_buffer_length);
        check_buffer_cmd();
        // printf("check buffer finished\n");
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

                // modified
                cmd_char_buffer_length = cmd_char_buffer_length - i - 8;
                if (cmd_char_buffer_length > 0)
                {

                    memcpy(cmd_char_buffer, cmd_char_buffer + i + 8, cmd_char_buffer_length);
                    // printf("after memcpy: \n");
                    // print_char_to_number(cmd_char_buffer, cmd_char_buffer_length);
                }
                // if there is a complete command, call function again, dangerous!
                if (cmd_char_buffer_length >= 8)
                {
                    check_buffer_cmd();
                }  
                return;
            }
            // lack of data, break
            break;
        }
    }   
}


float get_float_from_cmd(unsigned char* cmd)
{
    float f;
    memcpy(&f, cmd + 3, sizeof(f));
    return f;
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

// cmd = 0x06: set z p, 数据类型float
// cmd = 0x07: set z i, 数据类型float
// cmd = 0x08: set z d, 数据类型float

// cmd = 0x09: set roll p, 数据类型float
// cmd = 0x0A: set roll i, 数据类型float
// cmd = 0x0B: set roll d, 数据类型float

// cmd = 0x0C: set pitch p, 数据类型float
// cmd = 0x0D: set pitch i, 数据类型float
// cmd = 0x0E: set pitch d, 数据类型float

// cmd = 0x0F: set yaw p, 数据类型float
// cmd = 0x10: set yaw i, 数据类型float
// cmd = 0x11: set yaw d, 数据类型float

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
            thrust[i] = pow(cmd_char[3 + i], 2) / PWM_STEP;  // CONVERT TO 0 - PWM_MOTOR_MAX
        }
        set_motor_thrust_v2(motors, thrust);
        break;
    case 0x02:
        // set throttle
        motors.pid_on = true;
        motors.throttle = pow(cmd_char[3], 2) / PWM_STEP;
        motors.convert_to_total_output();
    break;
    case 0x03:break;
    case 0x04:break;
    case 0x05:break;
    case 0x06:break;
    case 0x07:break;
    case 0x08:break;

    case 0x09: motors.p_roll = get_float_from_cmd(cmd_char); break;
    case 0x0A: motors.i_roll = get_float_from_cmd(cmd_char); break;
    case 0x0B: motors.d_roll = get_float_from_cmd(cmd_char); break;

    case 0x0C: motors.p_pitch = get_float_from_cmd(cmd_char); break;
    case 0x0D: motors.i_pitch = get_float_from_cmd(cmd_char); break;
    case 0x0E: motors.d_pitch = get_float_from_cmd(cmd_char); break;
    
    case 0x0F: motors.p_yaw = get_float_from_cmd(cmd_char); break;
    case 0x10: motors.i_yaw = get_float_from_cmd(cmd_char); break;
    case 0x11: motors.d_yaw = get_float_from_cmd(cmd_char); break;
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