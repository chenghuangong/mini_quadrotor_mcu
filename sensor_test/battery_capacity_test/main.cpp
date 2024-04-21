#include <stdio.h>
#include <pico/stdlib.h>
#include <hardware/pwm.h>
#include <hardware/adc.h>
#include <pico.h>


// uart 
#define UART_ID uart0
#define BAUD_RATE 115200

// 使用GPIO0 和 GPIO1来作为UART PIN
#define UART_TX_PIN 0
#define UART_RX_PIN 1

struct battery_info
{
    double c_voltage = 0;
    double c_current = 0;
    double p_voltage = 0;
    double p_current = 0;

    double capacity_mAh = 700;      // mAh
    double consume_joul = 0;
    double consume_mAh = 0;

    double current_offset = 0;      // voltage when no current flow 0A
    double voltage_offset = 0;
    double current_factor = 0.2;    // 0.4V per A
    double voltage_factor = 2;      // two 10k resistor
};


bool output_callback(repeating_timer_t* timer)
{
    auto data = static_cast<battery_info*>(timer->user_data);
    // printf("%.3f\t%.3f\t%.3f\t%.3f\t%.2f\n", bat.c_voltage, bat.c_current, bat.consume_joul, bat.consume_mAh, (1 - bat.consume_mAh / bat.capacity_mAh) * 100);
    printf("%.3f\t%.3f\t%.3f\t%.3f\t%.2f\n", data->c_voltage, data->c_current, data->consume_joul, data->consume_mAh, (1 - data->consume_mAh / data->capacity_mAh) * 100);
    return true;
}


int main()
{
    stdio_init_all();

    // set adc
    adc_init();
    // Make sure GPIO is high-impedance, no pullups etc
    adc_gpio_init(26);  // battery voltage
    adc_gpio_init(27);  // current sensor

    // set uart
    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    battery_info bat;

    double voffset_sum = 0;
    double coffset_sum = 0;

    printf("start calibration.\n");
    for (size_t i = 0; i < 100; i++)
    {
        adc_select_input(0);
        voffset_sum += (adc_read() * 3.3) / 4096.0;
        adc_select_input(1);
        coffset_sum += (adc_read() * 3.3) / 4096.0;
        sleep_ms(50);
    }
    bat.voltage_offset = voffset_sum / 100;
    bat.current_offset = coffset_sum / 100;
    printf("calibration finished.\n");
    
    repeating_timer timer;
    add_repeating_timer_ms(1000, output_callback, &bat, &timer);

    while (true)
    {
        // save data
        bat.p_voltage = bat.c_voltage;
        bat.p_current = bat.c_current;
        // read adc value and print
        adc_select_input(0);
        bat.c_voltage = ((adc_read() * 3.3) / 4096.0 - bat.voltage_offset) * 2;

        adc_select_input(1);
        bat.c_current = ((adc_read() * 3.3) / 4096.0 - bat.current_offset) / bat.current_factor;

        // calculate the power
        bat.consume_joul += ((bat.p_voltage + bat.c_voltage) / 2) * ((bat.p_current + bat.c_current) / 2) * 0.1;
        bat.consume_mAh += ((bat.c_current + bat.p_current) * 0.5 * 0.1 * 1000) / 3600;
        
        sleep_ms(100);
    }

    return 0;
}