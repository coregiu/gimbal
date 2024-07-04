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
#include "mpu6050.h"
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

    init_protocols();
    init_modules();

    uart_log_start_info();

    LED = 0;
    delay_ms(500);
    uint8_t result = MPU_Init();
    if (result == 0)
    {
    LED = 1;
    delay_ms(500);
    }

    // vTaskStartScheduler();
    while (1)
    {
        delay_ms(1000);
        result = MPU_Get_Gyroscope(&(gimbal_info.gyro_x), &(gimbal_info.gyro_x),&(gimbal_info.gyro_x));
        if (result == 0)
        {
            LED = !LED;
        }
        show_gimbal_info(&gimbal_info);

        TIM_SetCompare1(TIM3,5); //0度,1ms
        TIM_SetCompare2(TIM3,5); //0度,1ms
    	delay_ms(1000);
    	TIM_SetCompare1(TIM3,10); //45度,1ms
    	TIM_SetCompare2(TIM3,10); //45度,1ms
    	delay_ms(1000);
    	TIM_SetCompare1(TIM3,15); //90,1.5ms
    	TIM_SetCompare2(TIM3,15); //90,1.5ms
    	delay_ms(1000);
    	TIM_SetCompare1(TIM3,20); //135,2ms
    	TIM_SetCompare2(TIM3,20); //135,2ms
     	delay_ms(1000);
    	TIM_SetCompare1(TIM3,25); //180,2.5ms
    	TIM_SetCompare2(TIM3,25); //180,2.5ms
     	delay_ms(1000);
    }
}