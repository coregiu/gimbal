/**
  ******************************************************************************
  *
  * Define of led display executor.
  * author: coregiu
  *
  *
  ******************************************************************************
**/

#include <led_display.h>

void init_led_display()
{
    OLED_Init();            //初始化OLED
    OLED_Clear();

    // OLED_ShowCHinese(54, 0, 3);  //电
    // OLED_ShowCHinese(72, 0, 4);  //子
    // OLED_ShowCHinese(90, 0, 5);  //科
    // OLED_ShowCHinese(108, 0, 6); //技
    OLED_ShowString(38, 3, "GIMBAL", 6);
    OLED_ShowString(20, 6, "Author:YOYO", 11);
}

void display_info(struct command_context *command_context)
{

}

const struct module_command_executor led_display_executor = {init_led_display, display_info};
