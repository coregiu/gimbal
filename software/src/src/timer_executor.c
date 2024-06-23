#include "timer_executor.h"

void tim1_configuration()
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE); // ʹ��TIM1ʱ��

    TIM_TimeBaseStructure.TIM_Period = 1000 - 1; // �Զ���װ��ֵ
    TIM_TimeBaseStructure.TIM_Prescaler = 72 - 1; // ʱ��Ԥ��Ƶֵ
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; // ���ϼ���ģʽ
    TIM_TimeBaseStructure.TIM_ClockDivision = 0; // ʱ�ӷ�Ƶ����
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0; // �ظ�������

    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

    TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE); // TIM1����ж�����

    NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn; // TIM1�ж�
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3; // ��ռ���ȼ�1
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3; // �����ȼ�0
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; // TIM1�ж�����
    NVIC_Init(&NVIC_InitStructure);

    TIM_Cmd(TIM1, ENABLE); // TIM1ʹ��
}

void update_timer_state(struct command_context *command_context)
{

}

void TIM1_UP_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET) // TIM1����ж�
    {
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update); // ���TIM1����жϱ�־λ
        // TODO: �ڴ˴�����жϴ������
        uart_log_data('T');
    }
}

const struct module_command_executor timer_executor = {tim1_configuration, update_timer_state};