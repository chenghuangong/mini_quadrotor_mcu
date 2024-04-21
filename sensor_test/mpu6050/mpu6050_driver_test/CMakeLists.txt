add_executable(mpu6050_driver_test
        main.cpp
        )

# pull in common dependencies and additional i2c hardware support
target_link_libraries(mpu6050_driver_test pico_stdlib hardware_i2c)
target_include_directories(mpu6050_driver_test PRIVATE "E:/files/Project/raspberry_pico/rp2040-module-lib/modules/sensor")

# create map/bin/hex file etc.
pico_add_extra_outputs(mpu6050_driver_test)

# enable
pico_enable_stdio_uart(mpu6050_driver_test 1)

