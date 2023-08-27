#include <stdio.h>
#include <pico/stdlib.h>
#include <hardware/pwm.h>

/*
test half output thrust
*/

// define motor mosfet pin
const uint MOTOR_1 = 10;
const uint MOTOR_2 = 11;
const uint MOTOR_3 = 12;
const uint MOTOR_4 = 13;

// define duty cycle, half thrust
const int duty = 3000;
const int wrap_value = 4000;
const int clock_div = 1;

void turn_motor_on_sequence(uint motor1, uint motor2, uint motor3, uint motor4);
void turn_all_motor_on(uint motor1, uint motor2, uint motor3, uint motor4, uint thrust);
void turn_all_motor_on_changed_value(uint motor1, uint motor2, uint motor3, uint motor4, uint time_interval, uint max_thrust, uint step);

int main()
{
    stdio_init_all();

    // set pwm
    gpio_set_function(MOTOR_1, GPIO_FUNC_PWM);
    gpio_set_function(MOTOR_2, GPIO_FUNC_PWM);
    gpio_set_function(MOTOR_3, GPIO_FUNC_PWM);
    gpio_set_function(MOTOR_4, GPIO_FUNC_PWM);

    uint slice_num_1 = pwm_gpio_to_slice_num(MOTOR_1);
    uint slice_num_2 = pwm_gpio_to_slice_num(MOTOR_2);
    uint slice_num_3 = pwm_gpio_to_slice_num(MOTOR_3);
    uint slice_num_4 = pwm_gpio_to_slice_num(MOTOR_4);

    // get pwm config
    pwm_config config = pwm_get_default_config();
    pwm_config_set_wrap(&config, wrap_value);
    pwm_config_set_clkdiv(&config, clock_div);
    
    // set duty cycle
    pwm_set_gpio_level(MOTOR_1, 0);
    pwm_set_gpio_level(MOTOR_2, 0);
    pwm_set_gpio_level(MOTOR_3, 0);
    pwm_set_gpio_level(MOTOR_4, 0);

    pwm_init(slice_num_1, &config, true);
    pwm_init(slice_num_2, &config, true);
    pwm_init(slice_num_3, &config, true);
    pwm_init(slice_num_4, &config, true);
    

    // turn_motor_on_sequence(MOTOR_1, MOTOR_2, MOTOR_3, MOTOR_4);
    // turn_all_motor_on(MOTOR_1, MOTOR_2, MOTOR_3, MOTOR_4, duty);
    turn_all_motor_on_changed_value(MOTOR_1, MOTOR_2, MOTOR_3, MOTOR_4, 4, duty, 40);

    return 0;
}


void turn_motor_on_sequence(uint motor1, uint motor2, uint motor3, uint motor4)
{
    while (1)
    {
        pwm_set_gpio_level(motor4, 0);
        pwm_set_gpio_level(motor1, duty);
        sleep_ms(1000);

        pwm_set_gpio_level(motor1, 0);
        pwm_set_gpio_level(motor2, duty);
        sleep_ms(1000);

        pwm_set_gpio_level(motor2, 0);
        pwm_set_gpio_level(motor3, duty);
        sleep_ms(1000);

        pwm_set_gpio_level(motor3, 0);
        pwm_set_gpio_level(motor4, duty);
        sleep_ms(1000);
    }    
}


void turn_all_motor_on(uint motor1, uint motor2, uint motor3, uint motor4, uint thrust)
{
    pwm_set_gpio_level(motor1, thrust);
    pwm_set_gpio_level(motor2, thrust);
    pwm_set_gpio_level(motor3, thrust);
    pwm_set_gpio_level(motor4, thrust);

    while (true)
    {
        tight_loop_contents();
    }  
}


void turn_all_motor_on_changed_value(uint motor1, uint motor2, uint motor3, uint motor4, uint time_interval, uint max_thrust, uint step)
{
    bool decrease = true;
    int current_thrust = max_thrust;

    while (true)
    {
        if (decrease)
        {
            // 降低
            current_thrust = (current_thrust - step) < 0 ? 0 : (current_thrust - step);
            decrease = current_thrust == 0? false : true;
        } else 
        {
            // 升高
            current_thrust = (current_thrust + step) > max_thrust ? max_thrust : (current_thrust + step);
            decrease = current_thrust == max_thrust? true : false;
        }

        pwm_set_gpio_level(motor1, current_thrust);
        pwm_set_gpio_level(motor2, current_thrust);
        pwm_set_gpio_level(motor3, current_thrust);
        pwm_set_gpio_level(motor4, current_thrust);
        sleep_ms(time_interval);
    }  
}