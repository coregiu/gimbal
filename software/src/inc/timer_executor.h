#ifndef __TIMER_H
#define __TIMER_H
#include "command.h"


void gimbal_task_callback(TimerHandle_t xTimer);

extern const struct module_command_executor timer_executor;

#endif
