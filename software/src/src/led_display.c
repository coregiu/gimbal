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

struct gimbal_info gimbal_info = {0, 0, -12300};

const uchar PIXEL_DISTANCE_INTERVAL = 6;

void init_led_display()
{
    OLED_Init();            //初始化OLED
    show_gimbal_info(&gimbal_info);
}

void display_info(struct command_context *command_context)
{

}

void show_number_by_order(u8 x, u8 y, short number)
{

}

void show_short_number(u8 x, u8 y, short number)
{
    if (number == 0)
    {
        OLED_ShowChar(x + PIXEL_DISTANCE_INTERVAL, y, 0x30, 8);
        return;
    }

    short loop_number = number;
    if (number < 0)
    {
        OLED_ShowChar(x + PIXEL_DISTANCE_INTERVAL, y, '-', 8);
        loop_number = ~(number - 1);
        x += PIXEL_DISTANCE_INTERVAL;
    }

    char show_numbers[5] = {0};
    char remainder = loop_number % 10;

    uchar loop_idx = 4;
    do
    {
        show_numbers[loop_idx] = remainder;
        loop_idx--;
        loop_number /= 10;
        remainder = loop_number % 10;
    } while(loop_number > 0 && loop_idx >= 0);

    for (uchar i = 0,flag = 0; i < 5; i++)
    {
        if (show_numbers[i] != 0 || flag > 0)
        {
            OLED_ShowChar(x + ((flag + 1) * PIXEL_DISTANCE_INTERVAL), y, show_numbers[i] + 0x30, 8);
            flag++;
        }
    }
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

    OLED_ShowString(20, 4, "X: ", 8);
    OLED_ShowString(20, 5, "Y: ", 8);
    OLED_ShowString(20, 6, "Z: ", 8);

    show_short_number(38, 4, gimbal_info->gyro_x);
    show_short_number(38, 5, gimbal_info->gyro_y);
    show_short_number(38, 6, gimbal_info->gyro_z);
}

const struct module_command_executor led_display_executor = {init_led_display, display_info};
