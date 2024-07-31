/**
  ******************************************************************************
  *
  * command define and command receiver.
  * author: coregiu
  *
  *
  ******************************************************************************
**/

#ifndef CONTROLLER_DEFINE_H
#define CONTROLLER_DEFINE_H

#include "command.h"
#include "vehicle_executor.h"
#include "video_executor.h"
#include "timer_executor.h"
#include "audio_receiver.h"
#include "video_receiver.h"
#include "arm_roboot_executor.h"
#include "led_display.h"
#include "iic.h"

extern const char command_module_map[COMMANDS_LENGTH][2];

/**
 * init clock to 72MHz.
 */
void clock_init(void);

/**
 * init protocols such as iic, uart.
 */
void init_protocols();

/**
 * init all receive and executor modules
 *
 */
void init_modules();

/**
 * receive commands from clients.
 */
char* receive_commands();

/**
 * execute commands;
 *
 */
void execute_commands(char *commands, enum command_type type);

#endif