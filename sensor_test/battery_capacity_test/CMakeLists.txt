add_executable(battery_capacity_test main.cpp)

target_link_libraries(battery_capacity_test pico_stdlib hardware_pwm hardware_adc hardware_i2c)

target_include_directories(battery_capacity_test PRIVATE "E:/files/Project/raspberry_pico/rp2040-module-lib/modules/sensor")

pico_add_extra_outputs(battery_capacity_test)

pico_enable_stdio_uart(battery_capacity_test 1)

add_definitions(-DPICO_STDIO_USB_CONNECTION_WITHOUT_DTR=1)
