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

void clock_init(void)
{
    // Reset the RCC clock configuration to the default reset state.
    // HSI ON, PLL OFF, HSE OFF, system clock = 72 MHz, cpu_clock = 72 MHz
    // RCC_DeInit();

    // 1. Enable HSE
    RCC->CR |= RCC_CR_HSEON; // Enable HSE
    while (!(RCC->CR & RCC_CR_HSERDY)); // Wait until HSE is ready

    // 2. Configure PLL
    RCC->CFGR &= ~RCC_CFGR_PLLSRC; // Clear PLL source bit
    RCC->CFGR |= RCC_CFGR_PLLXTPRE_HSE_Div2; // Set HSE/2 as PLL source
    RCC->CFGR &= ~RCC_CFGR_PLLXTPRE; // No pre-scaling on HSE
    RCC->CFGR &= ~((0x7 << 18) & RCC->CFGR); // 清除原有的PLL乘法因子位
    RCC->CFGR |= (9 << 18); // 设置PLL乘法因子为9

    // 3. Enable PLL
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY)); // Wait until PLL is ready

    // 4. Switch to PLL
    RCC->CFGR &= ~RCC_CFGR_SW; // Clear active clock source
    RCC->CFGR |= RCC_CFGR_SW_PLL; // Set PLL as active clock source

    // 5. Wait until PLL is used as system clock
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);

    // 6. Configure AHB/APB prescalers
    RCC->CFGR &= ~RCC_CFGR_HPRE; // Set AHB prescaler to 1
    RCC->CFGR &= ~RCC_CFGR_PPRE1; // Set APB1 prescaler to 2
    RCC->CFGR &= ~RCC_CFGR_PPRE2; // Set APB2 prescaler to 1

    // 7. Update SystemCoreClock variable
    SystemCoreClockUpdate();

    // BluePill board runs at 72 MHz
    SystemCoreClockUpdate();
    if (SysTick_Config(SystemCoreClock / 1000))
    {
        // Capture error
        while (1)
            ;
    }
}

int main(void)
{
    clock_init();
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