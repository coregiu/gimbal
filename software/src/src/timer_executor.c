#include <timer_executor.h>
#include <mpu6050.h>
#include <led_display.h>

struct gimbal_info pre_gimbal_info = {0, 0, 0};

void tim1_configuration()
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE); //使能TIM1时钟

    TIM_TimeBaseStructure.TIM_Period = 1000 - 1; // 自动重装载值, 100ms
    TIM_TimeBaseStructure.TIM_Prescaler = 7200 -1; // 时钟预分频值
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; // 向上计数模式
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; // 时钟分频因子
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0; // 重复计数器

    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

    TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE); // TIM1溢出中断允许

    TIM_Cmd(TIM1, ENABLE); // TIM1使能
}

void init_timer_module()
{
    tim1_configuration();
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

void TIM1_UP_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET) // TIM1溢出中断
    {
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update); // 清除TIM1溢出中断标志位
        // 开始监听mpu6050数据，计算偏转后反馈舵机反馈修正
        // uart_log_data('T');
        uint8_t result = MPU_Get_Gyroscope(&(gimbal_info.gyro_x), &(gimbal_info.gyro_x),&(gimbal_info.gyro_x));
        if (result != 0)
        {
            uart_log_string_data("mpu read error");
            return;
        }
        if (compare_gimbal_info(&pre_gimbal_info, &gimbal_info) == 0)
        {
            // 位置没有变化
            return;
        }
        set_gimbal_info(&pre_gimbal_info, &gimbal_info);
        log_gyro_info(&gimbal_info);
        show_gimbal_info(&gimbal_info);
    }
}

const struct module_command_executor timer_executor = {init_timer_module, update_timer_state};