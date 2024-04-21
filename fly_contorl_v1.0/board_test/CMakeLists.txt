add_executable(board_test_fc1 main.cpp)

target_link_libraries(board_test_fc1 pico_stdlib hardware_pwm hardware_adc hardware_i2c)

pico_add_extra_outputs(board_test_fc1)

pico_enable_stdio_uart(board_test_fc1 0)

pico_enable_stdio_usb(board_test_fc1 1)
