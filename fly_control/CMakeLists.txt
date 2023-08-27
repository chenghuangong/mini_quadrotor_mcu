add_executable(fly_control main.cpp rp2040_interface.cpp communicator.cpp sensor.cpp)

target_link_libraries(fly_control pico_stdlib hardware_i2c hardware_pwm hardware_timer hardware_adc)

pico_add_extra_outputs(fly_control)

# enable
pico_enable_stdio_usb(fly_control 0)
# disable
pico_enable_stdio_uart(fly_control 0)