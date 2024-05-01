#include <stdio.h>
#include <pico/stdlib.h>

#include "task_handler.hpp"

int main()
{
    stdio_init_all();
    sleep_ms(10000);

    task_handler_t handler;
    task_handler_initialize(&handler);

    // add_task(&handler, &task_test);
    // run_task_handler(&handler);

    while (true)
    {
        tight_loop_contents();
    }
    
    return 0;
}
