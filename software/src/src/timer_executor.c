#include <timer_executor.h>
#include "mpu6050.h"
#include "led_display.h"
#include "controller.h"

struct gimbal_info gimbal_info = {0};
struct gimbal_info pre_gimbal_info = {0};
int32_t gimbale_data_buffer[6] = {0};

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
    TIM_TimeBaseStructure.TIM_Period = 499;
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

void bind_dma1_to_iic2()
{
    // 使能I2C2时钟
    RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
    // 使能DMA1时钟
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;

    // 设置DMA通道1为从外设到内存的传输
    DMA1_Channel1->CCR = DMA_CCR_DIR | DMA_CCR_MINC | DMA_CCR_PL_1; // Direction: PERIPH to MEMORY, Increment memory address, Priority level high

    // 设置外围设备地址为I2C数据寄存器
    DMA1_Channel1->CPAR = (uint32_t)(&(I2C2->DR));

    // 设置内存地址为数据缓冲区
    DMA1_Channel1->CMAR = (uint32_t)gimbale_data_buffer;

    // 设置数据长度
    DMA1_Channel1->CNDTR = 6; // Assuming we read 6 bytes from MPU6050

    // 使能DMA通道
    DMA1_Channel1->CCR |= DMA_CCR_EN;

    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void init_timer_module()
{
    create_timer_executor();
    bind_dma1_to_iic2();
    uint8_t result = mpu_init();
    if (result != 0)
    {
        uart_log_string_no_enter("mpu init error: ");
        uart_log_number(result);
        uart_log_enter_char();
    }
}

uchar compare_gimbal_info(struct gimbal_info *pre_gimbal_info, struct gimbal_info *gimbal_info)
{
    if ((int)pre_gimbal_info->roll != (int)gimbal_info->roll)
    {
        return 1;
    }
    if ((int)pre_gimbal_info->pitch != (int)gimbal_info->pitch)
    {
        return 1;
    }
    if ((int)pre_gimbal_info->yaw != (int)gimbal_info->yaw)
    {
        return 1;
    }
    return 0;
}

void set_gimbal_info(struct gimbal_info *pre_gimbal_info, struct gimbal_info *gimbal_info)
{
    pre_gimbal_info->gyro_x_raw = gimbal_info->gyro_x_raw;
    pre_gimbal_info->gyro_y_raw = gimbal_info->gyro_y_raw;
    pre_gimbal_info->gyro_z_raw = gimbal_info->gyro_z_raw;

    pre_gimbal_info->accl_x_raw = gimbal_info->accl_x_raw;
    pre_gimbal_info->accl_y_raw = gimbal_info->accl_y_raw;
    pre_gimbal_info->accl_z_raw = gimbal_info->accl_z_raw;

    pre_gimbal_info->gyro_x = gimbal_info->gyro_x;
    pre_gimbal_info->gyro_y = gimbal_info->gyro_y;
    pre_gimbal_info->gyro_z = gimbal_info->gyro_z;

    pre_gimbal_info->accl_x = gimbal_info->accl_x;
    pre_gimbal_info->accl_y = gimbal_info->accl_y;
    pre_gimbal_info->accl_z = gimbal_info->accl_z;

    pre_gimbal_info->roll = gimbal_info->roll;
    pre_gimbal_info->pitch = gimbal_info->pitch;
    pre_gimbal_info->yaw = gimbal_info->yaw;
    pre_gimbal_info->temperature = gimbal_info->temperature;
}

void log_gimbal_info(struct gimbal_info *gimbal_info)
{
    uart_log_string_no_enter("G_X:");
    uart_log_number(gimbal_info->gyro_x_raw);
    uart_log_string_no_enter("|G_Y:");
    uart_log_number(gimbal_info->gyro_y_raw);
    uart_log_string_no_enter("|G_Z:");
    uart_log_number(gimbal_info->gyro_z_raw);
    uart_log_string_no_enter("|A_X:");
    uart_log_number(gimbal_info->accl_x_raw);
    uart_log_string_no_enter("|A_Y:");
    uart_log_number(gimbal_info->accl_y_raw);
    uart_log_string_no_enter("|A_Z:");
    uart_log_number(gimbal_info->accl_z_raw);
    uart_log_string_no_enter("|roll:");
    uart_log_number(gimbal_info->roll * 100);
    uart_log_string_no_enter("|pitch:");
    uart_log_number(gimbal_info->pitch * 100);
    uart_log_string_no_enter("|yaw:");
    uart_log_number(gimbal_info->yaw * 100);
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
        // LED = ~LED;

	    uchar result = mpu_get_gyroscope(&gimbal_info.gyro_x_raw, &gimbal_info.gyro_y_raw, &gimbal_info.gyro_z_raw);
        result |= mpu_get_accelerometer(&gimbal_info.accl_x_raw, &gimbal_info.accl_y_raw, &gimbal_info.accl_z_raw);
        gimbal_info.temperature = mpu_get_temperature();

        if (result != 0)
        {
            uart_log_string_data("mpu read error");
            return;
        }

        if (mpu_type == MPU_6050 || mpu_type == MPU_6500)
        {
            compute_angle(&gimbal_info);
        }
        else if (mpu_type == MPU_9250)
        {
            result = mpu_get_magnetometer(&gimbal_info.magn_x_raw, &gimbal_info.magn_y_raw, &gimbal_info.magn_z_raw);
            if (result != 0)
            {
                uart_log_string_data("magn read error");
                return;
            }
            mpu_compute_mag(&gimbal_info.magn_x_raw, &gimbal_info.magn_y_raw, &gimbal_info.magn_z_raw, &gimbal_info.magn_x, &gimbal_info.magn_y, &gimbal_info.magn_z);
            ahrs_update(&gimbal_info);
        }
        else
        {
            return;
        }

        log_gimbal_info(&gimbal_info);
        if (compare_gimbal_info(&pre_gimbal_info, &gimbal_info) == 0)
        {
            // 位置没有变化
            return;
        }
        set_gimbal_info(&pre_gimbal_info, &gimbal_info);

        execute_commands("K", COMMAND_TYPE_AUTO);
        show_gimbal_info(&gimbal_info);
    }
}

const struct module_command_executor timer_executor = {init_timer_module, update_timer_state};