/**
  ******************************************************************************
  *
  * Define of arm roboot executor.
  * author: coregiu
  *
  *
  ******************************************************************************
**/

#include <arm_roboot_executor.h>
#include "stm32_pca9685.h"

void init_roboot_state()
{

}

void update_roboot_state(struct command_context *command_context)
{

}

const struct module_command_executor arm_roboot_executor = {init_roboot_state, update_roboot_state};
