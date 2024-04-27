#ifndef COMM_CONTROLLER_H_
#define COMM_CONTROLLER_H_

#include "ESP8266.hpp"
#include "config.hpp"

struct comm_controller_t
{
    repeating_timer_t timer;

    esp8266_t esp8266;
    task_handler_t* handler;

    /*
    * tx msg format
    * tx_buf[0] = 0xAA;
    * tx_buf[1] = 0xBB;
    * tx_buf[2] = 0x00;        // cmd type
    * tx_buf[3] = 0x45;
    * tx_buf[-1] = check_sum; 
    */
    uint8_t tx_buf[256];
    uint8_t tx_len = 0;
    uint8_t rx_buf[256];
    uint8_t rx_len = 0;

    // data need to be transfered, directly copy the pointer other src
    // do not modify
    double* rpy_ptr;                // three values
    double* rpy_rate_ptr;           // three values
    double* motor_thrust_ptr;       // four values
};

// define a public comm controller to handle rx data
static comm_controller_t comm_controller;


void comm_controller_copy_float(uint8_t* dest, const void* src, size_t size)
{
    for (size_t j = 0; j < size; j++)
    {
        *(dest + j) = ((uint8_t*)src)[j];
    }
}


bool comm_controller_convert_rpy_to_msg(comm_controller_t* comm)
{
    comm->tx_buf[comm->tx_len++] = 0x00;    // length, include check sum, not add yet

    float temp = 0;

    // insert rpy data, 12bytes
    for (size_t i = 0; i < 3; i++)
    {
        temp = (float) comm->rpy_ptr[i];
        comm_controller_copy_float(comm->tx_buf + comm->tx_len, &temp, sizeof(float));
        comm->tx_len += sizeof(float);
    }

    
    return true;
}


void comm_controller_write_msg(comm_controller_t* comm)
{
    uint16_t check_sum = 0;
    comm->tx_buf[3] = comm->tx_len + 1;     // modify length infomation

    for (size_t i = 0; i < comm->tx_len; i++)
    {
        esp8266_write(&comm->esp8266, comm->tx_buf[i]);
        check_sum += comm->tx_buf[i];
    }

    esp8266_write(&comm->esp8266, (uint8_t) (check_sum & 0xFF));

    // debug code
    for (size_t i = 0; i < comm->tx_len; i++)
    {
        printf("0x%2X, ", comm->tx_buf[i]);
    }
    printf("0x%2X\n", (uint8_t) (check_sum & 0xFF));

    // reset the tx_buf length
    comm->tx_len = 3;
}


bool comm_controller_tx_timer_callback(repeating_timer_t* t)
{
    comm_controller_t* comm = static_cast<comm_controller_t*>(t->user_data);
    comm_controller_convert_rpy_to_msg(comm);
    comm_controller_write_msg(comm);
    return true;
}


/*
* extract cmd data from raw data
* then, transfer to functions
* if buffer length is 256, and no cmd found, then empty the buffer
* else move remain data to the buffer head
*/
void comm_controller_extract_command(comm_controller_t* comm)
{
    
}

void comm_controller_rx_callback()
{
    // only test code
    // while (uart_is_readable(comm_controller.esp8266.uart_handle)) 
    // {
    //     uint8_t data = uart_getc(comm_controller.esp8266.uart_handle);
    //     printf("%d ", data);
    // }

    uint8_t temp_rx_len = comm_controller.rx_len;

    while (uart_is_readable(comm_controller.esp8266.uart_handle) && comm_controller.rx_len < 256) 
    {
        comm_controller.rx_buf[comm_controller.rx_len++] = uart_getc(comm_controller.esp8266.uart_handle);
    }

    // no data received
    if (temp_rx_len == comm_controller.rx_len) return;

    // check buffer length, if more than 1byte, maybe contain a command
    if (comm_controller.rx_len >= 8)
    {
        comm_controller_extract_command(&comm_controller);
    }
}


uint8_t comm_controller_initialize(comm_controller_t* comm)
{
    // connect to the server
    esp8266_initialize(&comm->esp8266);
    esp8266_connect_to_server(&comm->esp8266);

    // add tx buffer head
    comm->tx_buf[comm->tx_len++] = 0xAA;
    comm->tx_buf[comm->tx_len++] = 0xBB;
    comm->tx_buf[comm->tx_len++] = 0x00; // cmd type

    return 0;
}


uint8_t run_comm_controller(comm_controller_t* comm)
{
    // initialize controller
    comm_controller_initialize(comm);

    // add timer callback
    add_repeating_timer_ms(1000, &comm_controller_tx_timer_callback, comm, &comm->timer);

    // add rx interrupt callbacks
    esp8266_set_rx_callback(&comm->esp8266, (uintptr_t) comm_controller_rx_callback);
    return 0;
}

#endif