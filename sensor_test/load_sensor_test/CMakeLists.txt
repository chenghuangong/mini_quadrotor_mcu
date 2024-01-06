add_executable(load_sensor_test main.cpp)

target_link_libraries(load_sensor_test pico_stdlib hardware_pwm hardware_adc)

pico_add_extra_outputs(load_sensor_test)

pico_enable_stdio_usb(load_sensor_test 1)
