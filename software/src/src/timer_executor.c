#include <timer_executor.h>
#include "mpu6050.h"
#include "led_display.h"

struct gimbal_info pre_gimbal_info = {0, 0, 0};

void create_timer_executor()
{
    TimerHandle_t xTimer;
    const TickType_t xFrequency = pdMS_TO_TICKS(100); // 定时器频率100MS
    const BaseType_t xAutoReload = pdTRUE; // 设置为自动装载

    xTimer = xTimerCreate("GimbalTask", xFrequency, xAutoReload, NULL, gimbal_task_callback);
    if (xTimer != NULL)
    {
        xTimerStart(xTimer, 0); // 启动定时器
    }
}

void init_timer_module()
{
    create_timer_executor();
    uint8_t result = MPU_Init();
    if (result != 0)
    {
        uart_log_string_data("mpu init error");
    }
}

uchar compare_gimbal_info(struct gimbal_info *pre_gimbal_info, struct gimbal_info *gimbal_info)
{
    if (pre_gimbal_info->gyro_x != gimbal_info->gyro_x)
    {
        return 1;
    }
    if (pre_gimbal_info->gyro_y != gimbal_info->gyro_y)
    {
        return 1;
    }
    if (pre_gimbal_info->gyro_z != gimbal_info->gyro_z)
    {
        return 1;
    }
    return 0;
}

void set_gimbal_info(struct gimbal_info *pre_gimbal_info, struct gimbal_info *gimbal_info)
{
    pre_gimbal_info->gyro_x = gimbal_info->gyro_x;
    pre_gimbal_info->gyro_y = gimbal_info->gyro_y;
    pre_gimbal_info->gyro_z = gimbal_info->gyro_z;
}

void log_gyro_info(struct gimbal_info *gimbal_info)
{
    uart_log_data('X');
    uart_log_data(':');
    uart_log_number(gimbal_info->gyro_x);
    uart_log_data('Y');
    uart_log_data(':');
    uart_log_number(gimbal_info->gyro_y);
    uart_log_data('Z');
    uart_log_data(':');
    uart_log_number(gimbal_info->gyro_z);
    uart_log_enter_char();
}

void update_timer_state(struct command_context *command_context)
{

}

void gimbal_task_callback(TimerHandle_t xTimer)
{
    // LED = ~LED;
    // uint8_t result = MPU_Get_Gyroscope(&(gimbal_info.gyro_x), &(gimbal_info.gyro_x),&(gimbal_info.gyro_x));
    // if (result != 0)
    // {
    //     uart_log_string_data("mpu read error");
    //     return;
    // }
    // if (compare_gimbal_info(&pre_gimbal_info, &gimbal_info) == 0)
    // {
    //     // 位置没有变化
    //     return;
    // }
    // set_gimbal_info(&pre_gimbal_info, &gimbal_info);
    // log_gyro_info(&gimbal_info);
    // show_gimbal_info(&gimbal_info);
}

const struct module_command_executor timer_executor = {init_timer_module, update_timer_state};