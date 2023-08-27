#include <stdio.h>
#include <pico/stdlib.h>
#include <hardware/uart.h>
#include <hardware/pwm.h>
#include <hardware/timer.h>
#include <hardware/adc.h>
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

const double MOTOR_COE_1 = 1;
const double MOTOR_COE_2 = 1;
const double MOTOR_COE_3 = 1;
const double MOTOR_COE_4 = 1;

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
void read_battery_level();


// PID function
void perform_motor_rate_control(communicator* comm, double roll_target = 0, double pitch_target = 0, double yaw_target = 0);
void perform_motor_angle_control(communicator* comm, double roll_target = 0, double pitch_target = 0, double yaw_target = 0);


// timer callback
bool send_motor_data_callback(repeating_timer_t *rt);
bool heart_beat_callback(repeating_timer_t *rt);
bool send_all_data_callback(repeating_timer_t *rt);
bool output_motor_thrust_callback(repeating_timer_t* rt);


int main()
{
    stdio_init_all();
    sleep_ms(10000);


    // initialize communicator and sensor
    communicator comm{SERVER_ADDR, SERVER_PORT};

    // initialize motor
    motors.motor1_ = MOTOR_1;
    motors.motor2_ = MOTOR_2;
    motors.motor3_ = MOTOR_3;
    motors.motor4_ = MOTOR_4;
    initialize_motor(motors);
    comm.motor_ = &motors;


    // initialize adc
    adc_init();
    adc_gpio_init(PICO_ADC0);


    // set uart recv callback, uart0 has already initialized in communicator, so just set callback
    // irq_set_exclusive_handler(UART0_IRQ, on_uart_rx);
    irq_set_exclusive_handler(UART0_IRQ, on_uart_rx_v2);
    irq_set_enabled(UART0_IRQ, true);
    uart_set_irq_enables(uart0, true, false);


    // read sensor data and apply pid to output
    repeating_timer motor_output_timer;
    add_repeating_timer_ms(4, &output_motor_thrust_callback, &comm, &motor_output_timer);
    // send data host
    repeating_timer all_data_timer;
    add_repeating_timer_ms(501, &send_all_data_callback, &comm, &all_data_timer);
    

    while (1)
    {
        tight_loop_contents();
    }

    return 0;
}

// =======================initialize motor=======================
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
    pwm_config_set_wrap(&config, PWM_COUNTS);                 // set duty cycle
    pwm_config_set_clkdiv(&config, PWM_CLOCK_DIV);            // set 125Mhz / 40 = 3.125MHz
    
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
    pwm_set_gpio_level(motors.motor1_, motors.total_output_[0] * MOTOR_COE_1);
    pwm_set_gpio_level(motors.motor2_, motors.total_output_[1] * MOTOR_COE_2);
    pwm_set_gpio_level(motors.motor3_, motors.total_output_[2] * MOTOR_COE_3);
    pwm_set_gpio_level(motors.motor4_, motors.total_output_[3] * MOTOR_COE_4);
}


// =======================handle command=======================
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

// cmd = 0x12: 设定电池电量

// cmd = 0x13: set roll angle p, 数据类型float
// cmd = 0x14: set pitch angle p, 数据类型float

// cmd = 0x15: set roll angle i, 数据类型float
// cmd = 0x16: set pitch angle i, 数据类型float

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
            thrust[i] = cmd_char[3 + i] * 15;  // CONVERT TO 0 - PWM_MOTOR_MAX
        }
        set_motor_thrust_v2(motors, thrust);
        break;
    case 0x02:
        // set throttle
        motors.pid_on = true;
        motors.throttle = cmd_char[3] * 15;
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
    case 0x13: motors.p_angle_roll = get_float_from_cmd(cmd_char); break;
    case 0x14: motors.p_angle_pitch = get_float_from_cmd(cmd_char); break;
    case 0x15: motors.i_angle_roll = get_float_from_cmd(cmd_char); break;
    case 0x16: motors.i_angle_pitch = get_float_from_cmd(cmd_char); break;

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


// =======================PID control=======================
void perform_motor_rate_control(communicator* comm, double roll_target, double pitch_target, double yaw_target)
{
    quad_motor* motor = comm->motor_;

    if (!motor->pid_on)
    {
        return;
    }

    auto rc = comm->sensor_gyro_.rate_ctrl;

    // save previous error
    rc.prev_err_roll = rc.err_roll;
    rc.prev_err_pitch = rc.err_pitch;
    rc.prev_err_yaw = rc.err_yaw;

    // set target rate speed is zero
    rc.err_roll = roll_target - rc.roll;
    rc.err_pitch = pitch_target - rc.pitch;
    rc.err_yaw = yaw_target - rc.yaw;


    // start PID controller, yaw_set_deg is offset
    double e_roll =  motor->p_roll * rc.err_roll + 
                     motor->i_roll * ((rc.err_roll + rc.prev_err_roll) / 2.0) * MPU6050_SAMPLING_TIME + rc.prev_integral_roll + 
                     motor->d_roll * (rc.err_roll - rc.prev_err_roll) / MPU6050_SAMPLING_TIME;
    
    double e_pitch = motor->p_pitch * rc.err_pitch + 
                     motor->i_pitch * ((rc.err_pitch + rc.prev_err_pitch) / 2.0) * MPU6050_SAMPLING_TIME + rc.prev_integral_pitch + 
                     motor->d_pitch * (rc.err_pitch - rc.prev_err_pitch) / MPU6050_SAMPLING_TIME;

    double e_yaw = motor->p_yaw * rc.err_yaw;

    motor->update_input_error(e_roll, e_pitch, e_yaw);


    // save previous integral value
    rc.prev_integral_roll += motor->i_roll * ((rc.err_roll + rc.prev_err_roll) / 2.0) * MPU6050_SAMPLING_TIME;
    rc.prev_integral_pitch += motor->i_pitch * ((rc.err_pitch + rc.prev_err_pitch) / 2.0) * MPU6050_SAMPLING_TIME;

    comm->sampling_count++;
}

void perform_motor_angle_control(communicator* comm, double roll_target, double pitch_target, double yaw_target)
{
    quad_motor* motor = comm->motor_;

    if (!motor->pid_on)
    {
        return;
    }

    auto gyro_data = comm->sensor_gyro_.get_sensor_kalman_data();

    auto ac = comm->sensor_gyro_.angle_ctrl;

    ac.prev_err_roll = ac.err_roll;
    ac.prev_err_pitch = ac.err_pitch;

    ac.err_roll = roll_target - gyro_data[0];
    ac.err_pitch = pitch_target - gyro_data[1];

    double roll_rate_target = motor->p_angle_roll * ac.err_roll + 
                              motor->i_angle_roll * ((ac.prev_err_roll + ac.err_roll) / 2.0) * MPU6050_SAMPLING_TIME + ac.prev_integral_roll;

    double pitch_rate_target = motor->p_angle_pitch * ac.err_pitch + 
                               motor->i_angle_pitch * ((ac.prev_err_pitch + ac.err_pitch) / 2.0) * MPU6050_SAMPLING_TIME + ac.prev_integral_pitch;

    ac.prev_integral_roll += motor->i_angle_roll * ((ac.err_roll + ac.prev_err_roll) / 2.0) * MPU6050_SAMPLING_TIME;
    ac.prev_integral_pitch += motor->i_angle_pitch * ((ac.err_pitch + ac.prev_err_pitch) / 2.0) * MPU6050_SAMPLING_TIME;

    perform_motor_rate_control(comm, roll_rate_target, pitch_rate_target);  
}


// =======================timer callback=======================
bool send_motor_data_callback(repeating_timer_t *rt)
{   
    // cost 4ms 
    // absolute_time_t start_time = get_absolute_time();
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

    // absolute_time_t end_time = get_absolute_time();
    // printf("time cost: %dus\n", static_cast<uint32_t>(absolute_time_diff_us(start_time, end_time)));
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

bool send_all_data_callback(repeating_timer_t *rt)
{
    // time cost 1.6ms, no motor data
    // time cost 3.0ms, with motor data
    // time cost 1.6ms, with motor data (baudrate = 230400), set data transfer frequency to 5Hz, time cost will be 10ms

    // 发送时间的速度决定步骤在uart上
    // absolute_time_t start_time = get_absolute_time();
    auto comm = static_cast<communicator*>(rt->user_data);

    auto gyro_data = comm->sensor_gyro_.get_sensor_kalman_data();       // kalman滤波的数据
    auto gyro_raw_data = comm->sensor_gyro_.get_sensor_raw_data();      // 6个原始数据
    auto pressure_data = comm->sensor_pressure_.get_sensor_data();      // 压力数据

    // read battery level
    read_battery_level();


    uint8_t frame[69];
    // head 4bytes
    frame[0] = 0xAA;
    frame[1] = 0xBB;
    frame[2] = 0x00; // cmd type
    frame[3] = 0x45; // length, 69bytes

    size_t insert_pos = 4;
    float temp = 0;

    // insert kalman data, 12bytes
    for (size_t i = 0; i < 3; i++)
    {
        temp = gyro_data[i];
        memcpy(frame + insert_pos, &temp, sizeof(temp));
        insert_pos += 4;
    }

    // insert gyro raw, 24bytes
    for (size_t i = 0; i < 6; i++)
    {
        temp = gyro_raw_data[i];
        memcpy(frame + insert_pos, &temp, sizeof(temp));
        insert_pos += 4;
    }

    // insert temperature, 4byte
    temp = gyro_data[3];
    memcpy(frame + insert_pos, &temp, sizeof(temp));
    insert_pos += 4;
    // insert pressure, 4byte
    temp = pressure_data[1];
    memcpy(frame + insert_pos, &temp, sizeof(temp));
    insert_pos += 4;

    // insert motor data, 16bytes
    for (size_t i = 0; i < 4; i++)
    {
        temp = motors.total_output_[i];
        memcpy(frame + insert_pos, &temp, sizeof(temp));
        insert_pos += 4;
    }

    // insert battery level, 4bytes
    temp = motors.battery_level;
    memcpy(frame + insert_pos, &temp, sizeof(temp));
    insert_pos += 4;

    // insert check sum and send to uart
    uint16_t check_sum = 0;
    for (size_t i = 0; i < insert_pos; i++)
    {
        // also write to uart
        uart_putc(uart0, frame[i]); 
        check_sum += frame[i];
    }

    frame[insert_pos] = static_cast<uint8_t>(check_sum & 0xFF);
    uart_putc(uart0, frame[insert_pos]);

    // absolute_time_t end_time = get_absolute_time();
    // printf("time cost: %dus\n", static_cast<uint32_t>(absolute_time_diff_us(start_time, end_time)));

    return true;
}

bool output_motor_thrust_callback(repeating_timer_t* rt)
{
    // read sensor data
    auto comm = static_cast<communicator*>(rt->user_data);
    comm->sensor_gyro_.mpu6050_read_kalman();

    // apply PID to motor output
    if (comm->motor_->pid_on)
    {
        // perform_motor_pid_control(comm);
        // perform_motor_rate_control(comm);
        perform_motor_angle_control(comm);
    }

    output_motor_thrust(motors);
    return true;
}

void read_battery_level()
{
    double adc_value = 0;
    double battery_value = 0;

    adc_select_input(0);
    adc_value = adc_read() * (3.3 / 4096);  // convert to voltage

    battery_value = adc_value / 0.6;
    motors.battery_level = battery_value;
}