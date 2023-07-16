#ifndef RP2040_INTERFACE_H_
#define RP2040_INTERFACE_H_
#include <string>
#include <hardware/uart.h>
#include <hardware/i2c.h>
#include <pico/stdlib.h>
#include "config.h"

// rp2040 send data to esp01s by uart
class esp01s
{

public:
    esp01s(const std::string& addr, uint16_t port);

public:
    void connect_to_server();
    bool write(const std::string& msg);

private:
    void init_dev();

private:
    std::string addr_;
    uint16_t port_;
    bool connected_;
};


class i2c_communicator
{
public:
    i2c_communicator(i2c_inst_t* i2c, const uint sda, const uint scl) 
        : sda_(sda)
        , scl_(scl)
        , i2c_(i2c) {}

    void default_i2c_init()
    {
        perform_i2c_init(400 * 1000);
    }

    void baudrate_i2c_init(uint baudrate)
    {
        perform_i2c_init(baudrate);
    }

    i2c_inst_t* get_i2c_inst() { return i2c_; }

private:
    void perform_i2c_init(uint baudrate)
    {
        if (!is_initialized_)
        {
            i2c_init(i2c_, baudrate);
            gpio_set_function(sda_, GPIO_FUNC_I2C);
            gpio_set_function(scl_, GPIO_FUNC_I2C);
            gpio_pull_up(sda_);
            gpio_pull_up(scl_);
            is_initialized_ = true;
        }
    }

private:
    i2c_inst_t* i2c_;
    uint sda_;
    uint scl_;
    bool is_initialized_ = false;
};

#endif