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

const int SERVO_DOWN_RIGHT_POSITION  = 0;
const int SERVO_DOWN_MIDDLE_POSITION = 105;
const int SERVO_DOWN_LEFT_POSITION   = 176;

const int SERVO_TOP_BACK_POSITION    = 30;
const int SERVO_TOP_MIDDLE_POSITION  = 90;
const int SERVO_TOP_FRONT_POSITION   = 160;

void tim3_gpio_config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    // 输出比较通道1 GPIO 初始化
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

// 使用TIM功能的PWM模式
void tim3_mode_config(void)
{
    // 开启定时器时钟,即内部时钟CK_INT=72M
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    /*--------------------时基结构体初始化-------------------------*/
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    // 自动重装载寄存器的值，累计TIM_Period+1个频率后产生一个更新或者中断
    TIM_TimeBaseStructure.TIM_Period = ARR;
    // 驱动CNT计数器的时钟 = Fck_int/(psc+1)
    TIM_TimeBaseStructure.TIM_Prescaler = PSC;
    // 时钟分频因子 ，配置死区时间时需要用到
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    // 计数器计数模式，设置为向上计数
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    // 重复计数器的值，没用到不用管
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    // 初始化定时器
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

    /*--------------------输出比较结构体初始化-------------------*/
    TIM_OCInitTypeDef TIM_OCInitStructure;
    // 配置为PWM模式1
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    // 输出使能
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    // 输出通道电平极性配置
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

    // 输出比较通道 1
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OC1Init(TIM3, &TIM_OCInitStructure);
    TIM_OC2Init(TIM3, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable); //使能TIM3在CCR2上的预装载寄存器

    // 使能计数器
    TIM_Cmd(TIM3, ENABLE);
}

// 定义一个初始化函数，同时初始化时基与输出比较
void init_roboot_state(void)
{
    tim3_gpio_config();
    tim3_mode_config();
    // 设置舵机初始状态在90度
    change_angle(CHANNEL_BOTTOM, SERVO_DOWN_MIDDLE_POSITION);
    change_angle(CHANNEL_UP, SERVO_TOP_MIDDLE_POSITION);
}

void update_roboot_state(struct command_context *command_context)
{
    uchar bottom_angle = SERVO_DOWN_MIDDLE_POSITION - gimbal_info.yaw;
    bottom_angle = bottom_angle > SERVO_DOWN_LEFT_POSITION ? SERVO_DOWN_LEFT_POSITION : bottom_angle;
    bottom_angle = bottom_angle < SERVO_DOWN_RIGHT_POSITION ? SERVO_DOWN_RIGHT_POSITION : bottom_angle;
    change_angle(CHANNEL_BOTTOM, bottom_angle);

    uchar top_angle = SERVO_TOP_MIDDLE_POSITION + gimbal_info.roll;
    top_angle = top_angle > SERVO_TOP_FRONT_POSITION ? SERVO_TOP_FRONT_POSITION : top_angle;
    top_angle = top_angle < SERVO_TOP_BACK_POSITION ? SERVO_TOP_BACK_POSITION : top_angle;
    change_angle(CHANNEL_UP, top_angle);
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
