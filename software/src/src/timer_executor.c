#include <timer_executor.h>
#include "mpu6050.h"
#include "led_display.h"

struct gimbal_info gimbal_info = {0, 0, 0, 0, 0, 0, 0.0, 0.0, 0.0, 0.0};
struct gimbal_info pre_gimbal_info = {0, 0, 0, 0, 0, 0, 0.0, 0.0, 0.0, 0.0};

void create_timer_executor()
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    // 使能TIM2时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    // 定时器参数配置
    // 假设系统时钟为72MHz，TIM2时钟源为APB1（经过2分频后为36MHz）
    // 设置预分频值为35999，则TIM2的计数频率为36MHz/(35999+1)=1KHz
    // 设置自动重载值为999，则中断周期为(999+1)*1ms=1s
    TIM_TimeBaseStructure.TIM_Period = 999;
    TIM_TimeBaseStructure.TIM_Prescaler = 35999;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    // 使能TIM2的更新中断
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

    // 中断优先级配置
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // 启动TIM2
    TIM_Cmd(TIM2, ENABLE);
}

void init_timer_module()
{
    create_timer_executor();
    MPU6050_initialize();     //=====MPU6050初始化
	DMP_Init();
    // uint8_t result = MPU_Init();
    // if (result != 0)
    // {
    //     uart_log_string_data("mpu init error");
    // }
    // else
    // {
    //     uart_log_string_data("mpu init success");
    // }
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

void log_gimbal_info(struct gimbal_info *gimbal_info)
{
    uart_log_string_no_enter("G_X:");
    uart_log_number(gimbal_info->gyro_x);
    uart_log_string_no_enter("|G_Y:");
    uart_log_number(gimbal_info->gyro_y);
    uart_log_string_no_enter("|G_Z:");
    uart_log_number(gimbal_info->gyro_z);
    uart_log_string_no_enter("|A_X:");
    uart_log_number(gimbal_info->accl_x);
    uart_log_string_no_enter("|A_Y:");
    uart_log_number(gimbal_info->accl_y);
    uart_log_string_no_enter("|A_Z:");
    uart_log_number(gimbal_info->accl_z);
    uart_log_string_no_enter("|roll:");
    uart_log_number(gimbal_info->roll);
    uart_log_string_no_enter("|pitch:");
    uart_log_number(gimbal_info->pitch);
    uart_log_string_no_enter("|yaw:");
    uart_log_number(gimbal_info->yaw);
    uart_log_string_no_enter("|Temp:");
    uart_log_number(gimbal_info->temperature);
    uart_log_enter_char();
}

void update_timer_state(struct command_context *command_context)
{

}

void TIM2_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {
        // 清除更新中断标志位
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
        LED = ~LED;
        // uint8_t result = MPU_Get_Gyroscope(&gimbal_info.gyro_x, &gimbal_info.gyro_y, &gimbal_info.gyro_z);
        // result |= MPU_Get_Accelerometer(&gimbal_info.accl_x, &gimbal_info.accl_y, &gimbal_info.accl_z);
        // gimbal_info.temperature = MPU_Get_Temperature();
        // if (result != 0)
        // {
        //     uart_log_string_data("mpu read error");
        //     return;
        // }
        Read_DMP(&gimbal_info.pitch, &gimbal_info.roll, &gimbal_info.yaw);
        gimbal_info.temperature = Read_Temperature();
        log_gimbal_info(&gimbal_info);
        if (compare_gimbal_info(&pre_gimbal_info, &gimbal_info) == 0)
        {
            // 位置没有变化
            return;
        }
        set_gimbal_info(&pre_gimbal_info, &gimbal_info);

        show_gimbal_info(&gimbal_info);
    }
}

const struct module_command_executor timer_executor = {init_timer_module, update_timer_state};