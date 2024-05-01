add_executable(fly_control_v1.0 config.hpp 
public_func.hpp status_controller.hpp flash_controller.hpp comm_controller.hpp pid_contorller.hpp sensor_controller.hpp task_handler.hpp main.cpp)

target_include_directories(fly_control_v1.0 PRIVATE "E:/files/Project/raspberry_pico/rp2040-module-lib/modules/gpio")
target_include_directories(fly_control_v1.0 PRIVATE "E:/files/Project/raspberry_pico/rp2040-module-lib/modules/motion")
target_include_directories(fly_control_v1.0 PRIVATE "E:/files/Project/raspberry_pico/rp2040-module-lib/modules/sensor")
target_include_directories(fly_control_v1.0 PRIVATE "E:/files/Project/raspberry_pico/rp2040-module-lib/modules/wireless")
target_include_directories(fly_control_v1.0 PRIVATE "E:/files/Project/raspberry_pico/rp2040-module-lib/modules/flash")

target_link_libraries(fly_control_v1.0 pico_stdlib hardware_pwm hardware_i2c hardware_flash hardware_sync)

# enable
pico_enable_stdio_usb(fly_control_v1.0 1)
# disable
pico_enable_stdio_uart(fly_control_v1.0 0)

pico_add_extra_outputs(fly_control_v1.0)