add_executable(remote_control main.cpp oled_disp.cpp)

target_link_libraries(remote_control pico_stdlib hardware_adc hardware_i2c)

pico_add_extra_outputs(remote_control)

# enable
pico_enable_stdio_usb(remote_control 0)
# disable
pico_enable_stdio_uart(remote_control 1)