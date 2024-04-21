add_executable(hardware_testv1 main.cpp)

target_link_libraries(hardware_testv1 pico_stdlib hardware_pwm hardware_adc hardware_i2c)

target_include_directories(hardware_testv1 PRIVATE "E:/files/Project/raspberry_pico/rp2040-module-lib/modules/sensor")

pico_add_extra_outputs(hardware_testv1)

pico_enable_stdio_uart(hardware_testv1 0)

pico_enable_stdio_usb(hardware_testv1 1)
