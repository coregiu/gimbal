/**
  ******************************************************************************
  *
  * The main function of ago.
  * author: coregiu
  *
  *
  ******************************************************************************
**/

#include <controller.h>
#include <oled.h>

static void clock_init(void)
{
    /* Reset the RCC clock configuration to the default reset state. */
    /* HSI ON, PLL OFF, HSE OFF, system clock = 72 MHz, cpu_clock = 72 MHz */
    RCC_DeInit();
    /* BluePill board runs at 72 MHz */
    SystemCoreClockUpdate();

    if (SysTick_Config(SystemCoreClock / 1000))
    {
        /* Capture error */
        while (1)
            ;
    }
}

int main(void)
{
    clock_init();
    delay_init();

    init_modules();

    uart_log_start_info();

    LED = 0;
	delay_ms(500);

    //		delay_ms(8000);
    OLED_Init();			//初始化OLED
    OLED_Clear()  	;

    OLED_ShowCHinese(54,0,3);//电
    OLED_ShowCHinese(72,0,4);//子
    OLED_ShowCHinese(90,0,5);//科
    OLED_ShowCHinese(108,0,6);//技


    LED = 1;
	delay_ms(500);

    vTaskStartScheduler();
    while (1)
    {
    }
}