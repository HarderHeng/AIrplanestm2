#include "includes.h"

// 任务优先级
#define TASK_START_PRIO 1
#define TASK_Test_Led_On_PRIO 3
#define TASK_Test_Led_Off_PRIO 4
#define TASK_Attitude_PRIO 5
#define TASK_Pid_PRIO 6

// 任务名定义
#define TASK_START_NAME "TASK_Start"
#define TASK_Test_Led_On_NAME "TASK_Test_Led_On"
#define TASK_Test_Led_Off_NAME "TASK_Test_Led_Off"
#define TASK_Attitude_NAME "TASK_Attitude"
#define TASK_Pid_NAME "TASK_Pid"

// 任务栈大小
#define TASK_START_STK_SIZE 128
#define TASK_Test_Led_On_STK_SIZE 64
#define TASK_Test_Led_Off_STK_SIZE 64
#define TASK_Attitude_STK_SIZE 128
#define TASK_Pid_STK_SIZE 128

// 任务栈
OS_STK TASK_START_STK[TASK_START_STK_SIZE];
OS_STK TASK_Test_Led_On_STK[TASK_Test_Led_On_STK_SIZE];
OS_STK TASK_Test_Led_Off_STK[TASK_Test_Led_Off_STK_SIZE];
OS_STK TASK_Attitude_STK[TASK_Attitude_STK_SIZE];
OS_STK TASK_Pid_STK[TASK_Pid_STK_SIZE];

// 任务定义
void TASK_Start(void *p_arg);
void TASK_Test_Led_On(void *p_arg);
void TASK_Test_Led_Off(void *p_arg);
void TASK_Attitude(void *p_arg);
void TASK_Pid(void *p_arg);

int main()
{
	DelayInit(); // 延时系统初始化
	// Delay_ms(5000);
	OSInit();																						// 操作系统初始化（包含SysTick的初始化）
	OS_TRACE_INIT();																				// 启动系统追踪
	OSTaskCreate(TASK_Start, (void *)0, &TASK_START_STK[TASK_START_STK_SIZE - 1], TASK_START_PRIO); // 创建START任务
	OSStart();																						// 启动系统
	return 0;
}
// 初始化任务
void TASK_Start(void *p_arg)
{
	OS_ERR err;
	Serial_Init(); // 串口发送和中断接受初始化
	IIC_Init();	   // 软件模拟IIC初始化
	PWM_Init();	   // PWM电机初始化以及调整电机上电启动
	MPU_Init();	   // MPU6050模块初始化
	HMC_Init();	   // HMC模块初始化
	Led_Init();	   // LED灯初始化
	IC_Init();	   //
	PWM_Start();
	Delay_ms(2000);
	Mutex_Init();
	Init_PID();
	// int a = OSSemCreate(1);
	// 进行硬件调试请屏蔽以下两条任务，并启用HardwareTest的任务。

	// OSTaskCreate(TASK_Test_Led_On, (void *)0, &TASK_Test_Led_On_STK[TASK_Test_Led_On_STK_SIZE - 1], TASK_Test_Led_On_PRIO);
	// OSTaskCreate(TASK_Test_Led_Off, (void *)0, &TASK_Test_Led_Off_STK[TASK_Test_Led_Off_STK_SIZE - 1], TASK_Test_Led_Off_PRIO);
	// OSTaskNameSet(TASK_Test_Led_On_PRIO, TASK_Test_Led_On_NAME, &err);
	// OSTaskNameSet(TASK_Test_Led_Off_PRIO, TASK_Test_Led_Off_NAME, &err);

	OSTaskCreate(TASK_Attitude, (void *)0, &TASK_Attitude_STK[TASK_Attitude_STK_SIZE - 1], TASK_Attitude_PRIO);
	OSTaskCreate(TASK_Pid, (void *)0, &TASK_Pid_STK[TASK_Pid_STK_SIZE - 1], TASK_Pid_PRIO);
	// OSTaskNameGet(TASK_Attitude_PRIO, TASK_Attitude_NAME, &err);
	// OSTaskNameGet(TASK_Pid_PRIO, TASK_Pid_NAME, &err);
	// USART1_printf("start");
}

void TASK_Test_Led_On(void *p_arg)
{
	while (1)
	{
		Led_On();
		OSTimeDly(100);
	}
} // 开灯循环函数

void TASK_Test_Led_Off(void *p_arg)
{
	while (1)
	{
		OSTimeDly(50);
		Led_Off();
		OSTimeDly(50);
	}
} // 关灯循环函数

// void TASK_HardwareTest(void *p_arg){
// 	while(1){
// 		// Serial_SendByte(OC1Get_Duty());
// 		// Serial_SendByte(OC2Get_Duty());
// 		// Serial_SendByte(OC3Get_Duty());
// 		// Serial_SendByte(OC4Get_Duty());
// 		// OSTimeDlyHMSM(0,0,1,0);

// 	}
// }