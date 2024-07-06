/**
  ******************************************************************************
  *
  * The main function of ago.
  * author: coregiu
  *
  *
  ******************************************************************************
**/

#include "stm32f10x_conf.h"
#include "delay.h"
#include "sys.h"

#define LED PCout(13)

static void TIM3_GPIO_Config(void)
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
static void TIM3_Mode_Config(void)
{
    // 开启定时器时钟,即内部时钟CK_INT=72M
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    /*--------------------时基结构体初始化-------------------------*/
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    // 自动重装载寄存器的值，累计TIM_Period+1个频率后产生一个更新或者中断
    TIM_TimeBaseStructure.TIM_Period = 199;
    // 驱动CNT计数器的时钟 = Fck_int/(psc+1)
    TIM_TimeBaseStructure.TIM_Prescaler = 7199;
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
void TIM3_Init(void)
{
    TIM3_GPIO_Config();
    TIM3_Mode_Config();
}

void Delay_T(__IO uint32_t nCount)	 //简单的延时函数
{
	for(; nCount != 0; nCount--);
}

#define SOFT_DELAY Delay_T(0x0FFFFF);

void init_led_module()
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE); //使能PA,PB, PC端口时钟

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;        //LED-->PC13 端口配置
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  //推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //IO口速度为50MHz
    GPIO_Init(GPIOC, &GPIO_InitStructure);            //根据设定参数初始化GPIOC.13
    GPIO_SetBits(GPIOC, GPIO_Pin_13);                 //PC.13 输出高
}

void init_e()
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
    TIM_TimeBaseStructure.TIM_Period = 199;                     //设置在下一个更新事件装入活动的自动重装载寄存器周期的值
    TIM_TimeBaseStructure.TIM_Prescaler = 7199;                 //设置用来作为TIMx时钟频率除数的预分频值
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;     //设置时钟分割:TDTS = Tck_tim
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
}

int main(void)
{
    delay_init();

    init_led_module();
    LED = 0;

    TIM3_Init();

	while(1)
	{
        LED = ~LED;
        TIM_SetCompare1(TIM3 , 5);
        TIM_SetCompare2(TIM3 , 5);
        delay_ms(1000);
        TIM_SetCompare1(TIM3 , 15);
        TIM_SetCompare2(TIM3 , 15);
        delay_ms(1000);
        TIM_SetCompare1(TIM3 , 25);
        TIM_SetCompare2(TIM3 , 25);
        delay_ms(1000);
		// for(int i = 5;i < 26;i++)//结果测试，设置CCR的值为5-6对应舵机旋转到0°-180°
		// {
		// 	TIM_SetCompare1(TIM3 , i);
		// 	SOFT_DELAY;
		// }
	}
    // delay_ms(500);

    // // LED = ~LED;
    // // vTaskStartScheduler();
    // while (1)
    // {
    //     LED = ~LED;
    //     TIM_SetCompare1(TIM3, 5);
    //     delay_ms(500);

    //     LED = ~LED;
    //     TIM_SetCompare1(TIM3, 10);
    //     delay_ms(500);

    //     LED = ~LED;
    //     TIM_SetCompare1(TIM3, 15);
    //     delay_ms(500);

    //     LED = ~LED;
    //     TIM_SetCompare1(TIM3, 20);
    //     delay_ms(500);

    //     LED = ~LED;
    //     TIM_SetCompare1(TIM3, 25);
    //     delay_ms(500);
    // }
}