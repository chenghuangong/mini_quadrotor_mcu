#include "rp2040_interface.h"

esp01s::esp01s(const std::string& addr, uint16_t port)
    : addr_(addr)
    , port_(port) 
    {
        init_dev();
    }


void esp01s::init_dev()
{
    uart_init(UART_ID, UART_BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
}


void esp01s::connect_to_server()
{
    // 使用AT指令, 延时一定要加够，不然会出错
    uart_puts(UART_ID, "AT\r\n");
    sleep_ms(500);
    std::string target = "AT+CIPSTART=\"TCP\",\"" + addr_ + "\"," + std::to_string(port_) + "\r\n";
    uart_puts(UART_ID, target.c_str());
    sleep_ms(500);
    // 开启透传模式
    target = "AT+CIPMODE=1\r\n";
    uart_puts(UART_ID, target.c_str());
    sleep_ms(500);
    target = "AT+CIPSEND\r\n";
    uart_puts(UART_ID, target.c_str());
    sleep_ms(500);
}

bool esp01s::write(const std::string& msg)
{
    uart_puts(UART_ID, msg.c_str());
    return true;
}