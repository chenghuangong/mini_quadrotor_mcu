add_executable(mpu6050_test
        main.cpp
        )

# pull in common dependencies and additional i2c hardware support
target_link_libraries(mpu6050_test pico_stdlib hardware_i2c)

# create map/bin/hex file etc.
pico_add_extra_outputs(mpu6050_test)

# enable
pico_enable_stdio_usb(mpu6050_test 1)

