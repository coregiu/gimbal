#include <timer_executor.h>
#include <mpu6050.h>
#include <led_display.h>

struct gimbal_info pre_gimbal_info = {0, 0, 0};

void tim1_configuration()
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE); //ʹ��TIM1ʱ��

    TIM_TimeBaseStructure.TIM_Period = 1000 - 1; // �Զ���װ��ֵ, 100ms
    TIM_TimeBaseStructure.TIM_Prescaler = 7200 -1; // ʱ��Ԥ��Ƶֵ
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; // ���ϼ���ģʽ
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; // ʱ�ӷ�Ƶ����
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0; // �ظ�������

    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

    TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE); // TIM1����ж�����

    TIM_Cmd(TIM1, ENABLE); // TIM1ʹ��
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
    if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET) // TIM1����ж�
    {
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update); // ���TIM1����жϱ�־λ
        // ��ʼ����mpu6050���ݣ�����ƫת���������������
        // uart_log_data('T');
        uint8_t result = MPU_Get_Gyroscope(&(gimbal_info.gyro_x), &(gimbal_info.gyro_x),&(gimbal_info.gyro_x));
        if (result != 0)
        {
            uart_log_string_data("mpu read error");
            return;
        }
        if (compare_gimbal_info(&pre_gimbal_info, &gimbal_info) == 0)
        {
            // λ��û�б仯
            return;
        }
        set_gimbal_info(&pre_gimbal_info, &gimbal_info);
        log_gyro_info(&gimbal_info);
        show_gimbal_info(&gimbal_info);
    }
}

const struct module_command_executor timer_executor = {init_timer_module, update_timer_state};