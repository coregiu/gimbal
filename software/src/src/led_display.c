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

struct gimbal_info gimbal_info = {0, 0, 0};

void init_led_display()
{
    OLED_Init();            //初始化OLED
    show_gimbal_info(&gimbal_info);
}

void display_info(struct command_context *command_context)
{

}

void show_gimbal_info(struct gimbal_info *gimbal_info)
{
    OLED_Clear();

    // OLED_ShowCHinese(54, 0, 3);  //电
    // OLED_ShowCHinese(72, 0, 4);  //子
    // OLED_ShowCHinese(90, 0, 5);  //科
    // OLED_ShowCHinese(108, 0, 6); //技
    OLED_ShowString(40, 0, "GIMBAL", 16);
    OLED_ShowString(20, 2, "Author:YOYO", 8);

    OLED_ShowString(20, 4, "X: 0", 8);
    OLED_ShowString(20, 5, "Y: 0", 8);
    OLED_ShowString(20, 6, "Z: 0", 8);
}

const struct module_command_executor led_display_executor = {init_led_display, display_info};
