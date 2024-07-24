#include <timer_executor.h>
#include "mpu6050.h"
#include "led_display.h"
#include "controller.h"
#include "FreeRTOS.h"
#include "timers.h"

struct gimbal_info gimbal_info = {0};
struct gimbal_info pre_gimbal_info = {0};
int32_t gimbale_data_buffer[6] = {0};

static void vTimerCallback(TimerHandle_t xTimer);

static TimerHandle_t xMyTimer = NULL;

struct command_context gimbal_cmd = {0, DELAY_BEFOR_EXE, COMMAND_TYPE_AUTO, COMMAND_GIMBAL_INFO};
struct command_context display_cmd = {0, DELAY_BEFOR_EXE, COMMAND_TYPE_AUTO, COMMAND_LED_DISPLAY};
struct command_context servo_cmd = {0, DELAY_BEFOR_EXE, COMMAND_TYPE_AUTO, COMMAND_ADAPTE_SERVO};

static void vTimerCallback(TimerHandle_t xTimer)
{
    LED = ~LED;
    send_to_queue(&gimbal_cmd);
    send_to_queue(&display_cmd);
}

void create_timer_executor()
{
    const char *pcTimerName = "GimbalTimer";
    uint32_t ulPeriod = pdMS_TO_TICKS(1000); // Period in ticks, e.g., 1000ms
    uint8_t ucAutoReload = pdTRUE;           // Auto-reload after expiry
    BaseType_t xTimerCreated = pdFALSE;

    xMyTimer = xTimerCreate(
        pcTimerName,      // A text name for the timer (not used by the kernel)
        ulPeriod,         // The timer period in ticks
        ucAutoReload,     // Auto-reload setting
        (void *)NULL,     // The timer ID, can be NULL
        vTimerCallback    // The callback function that will be called on timer expiry
    );

    if(xMyTimer != NULL)
    {
        xTimerStart(xMyTimer, 0); // Start the timer
        xTimerCreated = pdTRUE;
    }

    configASSERT(xTimerCreated);
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
    // bind_dma1_to_iic2();
    uint8_t result = mpu_init();
    if (result != 0)
    {
        uart_log_string_no_enter("mpu init error: ");
        uart_log_number(result);
        uart_log_enter_char();
        return;
    }
    create_timer_executor();
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
    uart_log_string_no_enter("|M_X:");
    uart_log_number(gimbal_info->magn_x_raw);
    uart_log_string_no_enter("|M_Y:");
    uart_log_number(gimbal_info->magn_y_raw);
    uart_log_string_no_enter("|M_Z:");
    uart_log_number(gimbal_info->magn_z_raw);
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

    send_to_queue(&servo_cmd);
    // show_gimbal_info(&gimbal_info);
}

void TIM2_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {
        // 清除更新中断标志位
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);

    }
}

const struct module_command_executor timer_executor = {init_timer_module, update_timer_state};