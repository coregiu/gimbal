/**
  ******************************************************************************
  *
  * The main function of ago.
  * author: coregiu
  *
  *
  * *****************************************************************************
  * 开启timer3定时器后不再需要clock_init了。
  * void clock_init(void)
  * {
  *     // Reset the RCC clock configuration to the default reset state.
  *     // HSI ON, PLL OFF, HSE OFF, system clock = 72 MHz, cpu_clock = 72 MHz
  *     RCC_DeInit();
  *     // BluePill board runs at 72 MHz
  *     SystemCoreClockUpdate();
  *
  *     if (SysTick_Config(SystemCoreClock / 1000))
  *     {
  *         // Capture error
  *         while (1)
  *             ;
  *     }
  * }
  */

#include <controller.h>

int main(void)
{
    delay_init();

    init_protocols();
    LED = 0;

    init_modules();
    uart_log_start_info();
    delay_ms(500);

    LED = ~LED;
    vTaskStartScheduler();
    while (1)
    {
    }
}