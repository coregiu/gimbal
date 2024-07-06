/**
  ******************************************************************************
  *
  * Define of arm roboot executor.
  * author: coregiu
  *
  *
  ******************************************************************************
**/

#include <arm_roboot_executor.h>

const short ARR = 199;
const short PSC = 7199;

void init_roboot_state()
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    TIM_OCStructInit(&TIM_OCInitStructure); //配置默认项

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);                        //使能定时器3时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE); //使能GPIO外设和AFIO复用功能模块时钟

    GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, ENABLE); //Timer3部分重映射  TIM3_CH1->PB4,TIM3_CH2->PB5

    //设置该引脚为复用输出功能,输出TIM3 CH1|CH2的PWM脉冲波形
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5; //TIM_CH1|TIM_CH2
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;        //复用推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure); //初始化GPIO

    // 初始化TIM3
    // 定时器中断时间（ｍｓ）＝（TIM_Prescaler + 1）* (TIM_Period +1) * 1000 ／ 时钟频率
    // (7200*200)/72000000=0.02=20ms
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1 ;
    TIM_TimeBaseStructure.TIM_Period = ARR;                     //设置在下一个更新事件装入活动的自动重装载寄存器周期的值
    TIM_TimeBaseStructure.TIM_Prescaler = PSC;                  //设置用来作为TIMx时钟频率除数的预分频值
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;                //设置时钟分割:TDTS = Tck_tim
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM向上计数模式
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);             //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位

    //初始化TIM3 Channel2 PWM模式

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;             //选择定时器模式:TIM脉冲宽度调制模式1
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;     //输出极性:TIM输出比较极性高
    TIM_OC2Init(TIM3, &TIM_OCInitStructure);                      //根据T指定的参数初始化外设TIM3 OC2
    TIM_OC1Init(TIM3, &TIM_OCInitStructure);

    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable); //使能TIM3在CCR2上的预装载寄存器

    TIM_ARRPreloadConfig(TIM3, ENABLE); //使能TIMx在ARR上的预装载寄存器

    TIM_Cmd(TIM3, ENABLE); //使能TIM3

    // 设置舵机初始状态在90度
    change_angle(CHANNEL_BOTTOM, SERVO_RANGE_1_INIT);
    change_angle(CHANNEL_UP, SERVO_RANGE_2_INIT);
}

void update_roboot_state(struct command_context *command_context)
{
}

/**
 * 0度， 0.5ms， 在占空比20ms下，相当对于200次就是5次。
 * 45度， 10
 * 90度， 15
 * 135度， 20
 * 180度， 25
 */
void change_angle(enum pwm_channel channel, uchar angle)
{
    switch (channel)
    {
    case CHANNEL_BOTTOM:
        TIM_SetCompare1(TIM3, (angle / 9) + 5);
        break;
    case CHANNEL_UP:
        TIM_SetCompare2(TIM3, (angle / 9) + 5);
        break;
    default:
        break;
    }

}

const struct module_command_executor arm_roboot_executor = {init_roboot_state, update_roboot_state};
