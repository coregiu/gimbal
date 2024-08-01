#include <timer_manager.h>

struct command_context attitude_cmd = {COMMAND_ATTITUDE_INFO, MODULE_ATTITUDE, 0, DELAY_BEFOR_EXE, COMMAND_TYPE_AUTO};
struct command_context display_cmd = {COMMAND_LED_DISPLAY, MODULE_LED, 0, DELAY_BEFOR_EXE, COMMAND_TYPE_AUTO};

void create_timer_manager()
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    // 使能TIM2时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    // 定时器参数配置
    // 假设系统时钟为72MHz，TIM2时钟源为APB1（经过2分频后为36MHz）
    // 设置预分频值为35999，则TIM2的计数频率为36MHz/(35999+1)=1KHz
    // 设置自动重载值为999，则中断周期为(99+1)*1ms=100ms
    TIM_TimeBaseStructure.TIM_Period = 99;
    TIM_TimeBaseStructure.TIM_Prescaler = 35999;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    // 使能TIM2的更新中断
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

    // 中断优先级配置
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // 启动TIM2
    TIM_Cmd(TIM2, ENABLE);
}

void init_timer_module()
{
    create_timer_manager();
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
        execute_command(&attitude_cmd);
        execute_command(&display_cmd);
    }
}

const struct module_command_executor timer_manager = {init_timer_module, update_timer_state};
