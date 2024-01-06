add_executable(pwm_single_motor main.cpp)

target_link_libraries(pwm_single_motor pico_stdlib hardware_pwm hardware_adc)

pico_add_extra_outputs(pwm_single_motor)

pico_enable_stdio_uart(pwm_single_motor 1)
