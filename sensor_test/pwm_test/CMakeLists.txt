add_executable(pwm_test main.cpp)

target_link_libraries(pwm_test pico_stdlib hardware_pwm)

pico_add_extra_outputs(pwm_test)

