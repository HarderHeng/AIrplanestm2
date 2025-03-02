#include "includes.h"
#include "system_stm32f4xx.h"

int main()
{
	DelayInit();
	MPU_Init();
	Serial_Init();
	IC_Init();
	PWM_Init();
	//USART1_printf("start");
	PWM_Start();
	Delay_ms(8000);
	//int M1 =1100, M2 = 1100, M3 = 1100, M4 = 1100;
	// USART1_printf("test");
	Init_PID();
	// PWM_Set(M1,M2,M3,M4);
	while (1)
	{
		// Receiver_displaytarget();
		// TASK_HardwareTest(); // madgwick
		rate_pid(); // pid
					// Delay_ms(500);
	}
}
