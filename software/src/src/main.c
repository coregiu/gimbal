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
    LED = ~LED;
    delay_ms(500);
    }

    // vTaskStartScheduler();
    while (1)
    {
        result = MPU_Get_Gyroscope(&(gimbal_info.gyro_x), &(gimbal_info.gyro_x),&(gimbal_info.gyro_x));
        if (result == 0)
        {
            LED = ~LED;
        }
        show_gimbal_info(&gimbal_info);

        // for (char i = 0; i < 200; i++)
        // {
        //     TIM_SetCompare1(TIM3, i); //0度,1ms
        //     TIM_SetCompare2(TIM3, i); //0度,1ms
        //     delay_ms(10);
        // }
        // for (char i = 0; i < 200; i++)
        // {
        //     TIM_SetCompare1(TIM3, 199 - i); //0度,1ms
        //     TIM_SetCompare2(TIM3, 199 - i); //0度,1ms
        //     delay_ms(10);
        // }

        change_angle(0); //0度,0.5ms
        LED = ~LED;
    	delay_ms(1000);

    	change_angle(45); //45度,1ms
        LED = ~LED;
    	delay_ms(1000);

    	change_angle(90);  //90,1.5ms
        LED = ~LED;
    	delay_ms(1000);

    	change_angle(135); //135,2ms
        LED = ~LED;
     	delay_ms(1000);

    	change_angle(180); //180,2.5ms
        LED = ~LED;
     	delay_ms(1000);
    }
}