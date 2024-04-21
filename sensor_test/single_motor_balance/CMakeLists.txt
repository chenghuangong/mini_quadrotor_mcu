add_executable(single_motor_balance main.cpp)

target_link_libraries(single_motor_balance pico_stdlib hardware_pwm hardware_adc hardware_i2c)

target_include_directories(single_motor_balance PRIVATE "E:/files/Project/raspberry_pico/rp2040-module-lib/modules/sensor")

pico_add_extra_outputs(single_motor_balance)

pico_enable_stdio_uart(single_motor_balance 1)
