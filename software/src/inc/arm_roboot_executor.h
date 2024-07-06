/**
  ******************************************************************************
  *
  * Define of arm roboot executor.
  * author: coregiu
  *
  *
  ******************************************************************************
**/

#ifndef EXECUTOR_ARM_ROBOOT_H
#define EXECUTOR_ARM_ROBOOT_H

#include "command.h"

#define SERVO_1 0
#define SERVO_2 3
#define SERVO_3 4
#define SERVO_4 7
#define SERVO_5 8
#define SERVO_6 11

// ---------------servo define----------------
#define SERVO_RANGE_1_RIGHT   0
#define SERVO_RANGE_1_MIDDLE  90
#define SERVO_RANGE_1_LEFT    180
#define SERVO_RANGE_1_INIT    90

#define SERVO_RANGE_2_BACK    0
#define SERVO_RANGE_2_MIDDLE  90
#define SERVO_RANGE_2_FRONT   180
#define SERVO_RANGE_2_INIT    90

enum pwm_channel
{
    CHANNEL_BOTTOM = '1',
    CHANNEL_UP     = '2'
};

// angle: 0-180
void change_angle(enum pwm_channel channel, uchar angle);

extern const struct module_command_executor arm_roboot_executor;

#endif