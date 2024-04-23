#ifndef PUBLIC_FUNC_H
#define PUBLIC_FUNC_H

#include <pico/stdlib.h>

enum class controller_t
{
    SENSOR,
    PID,
    COMM
};

struct task_handler_t;
typedef uint8_t (*task_ptr)(task_handler_t*);


void controller_callback_func(task_handler_t* handler, controller_t ctrl);
#endif