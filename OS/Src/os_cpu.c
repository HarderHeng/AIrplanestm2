#include "os_cpu.h"
#include  <ucos_ii.h>



/*Systemtick define*/
#define  OS_CPU_CM_SYST_CSR         (*((volatile INT32U *)0xE000E010uL)) /* SysTick Ctrl & Status Reg.                  */
#define  OS_CPU_CM_SYST_RVR         (*((volatile INT32U *)0xE000E014uL)) /* SysTick Reload  Value Reg.                  */
#define  OS_CPU_CM_SYST_CVR         (*((volatile INT32U *)0xE000E018uL)) /* SysTick Current Value Reg.                  */
#define  OS_CPU_CM_SYST_CALIB       (*((volatile INT32U *)0xE000E01CuL)) /* SysTick Cal     Value Reg.                  */
#define  OS_CPU_CM_SCB_SHPRI1       (*((volatile INT32U *)0xE000ED18uL)) /* System Handlers  4 to  7 Prio.              */
#define  OS_CPU_CM_SCB_SHPRI2       (*((volatile INT32U *)0xE000ED1CuL)) /* System Handlers  8 to 11 Prio.              */
#define  OS_CPU_CM_SCB_SHPRI3       (*((volatile INT32U *)0xE000ED20uL)) /* System Handlers 12 to 15 Prio.              */


#define  OS_CPU_CM_SYST_CSR_COUNTFLAG                     0x00010000uL   /* Count flag.                                 */
#define  OS_CPU_CM_SYST_CSR_CLKSOURCE                     0x00000004uL   /* Clock Source.                               */
#define  OS_CPU_CM_SYST_CSR_TICKINT                       0x00000002uL   /* Interrupt enable.                           */
#define  OS_CPU_CM_SYST_CSR_ENABLE                        0x00000001uL   /* Counter mode.                               */




void  OS_CPU_SysTickHandler (void)
{
#if OS_CRITICAL_METHOD == 3u                                    /* Allocate storage for CPU status register             */
    OS_CPU_SR  cpu_sr;
#endif


    OS_ENTER_CRITICAL();
    OSIntEnter();                                               /* Tell uC/OS-II that we are starting an ISR            */
    OS_EXIT_CRITICAL();

    OSTimeTick();                                               /* Call uC/OS-II's OSTimeTick()                         */

    OSIntExit();                                                /* Tell uC/OS-II that we are leaving the ISR            */
}


void  OS_CPU_SysTickInitFreq (INT32U  cpu_freq)
{
    INT32U  cnts;


    cnts = (cpu_freq / (INT32U)OS_TICKS_PER_SEC);               /* Determine nbr SysTick cnts between two OS tick intr. */

    OS_CPU_SysTickInit(cnts);
}

void  OS_CPU_SysTickInit (INT32U  cnts)
{
    INT32U  prio;
    INT32U  basepri;


                                                                /* Set BASEPRI boundary from the configuration.         */
    basepri               = (INT32U)(CPU_CFG_KA_IPL_BOUNDARY << (8u - CPU_CFG_NVIC_PRIO_BITS));
    OS_CPU_CM_SYST_RVR    = cnts - 1u;                          /* Set Reload register.                                 */

                                                                /* Set SysTick handler prio.                            */
    prio                  =  OS_CPU_CM_SCB_SHPRI3;
    prio                 &=  0x00FFFFFFu;
    prio                 |= (basepri << 24u);
    OS_CPU_CM_SCB_SHPRI3  = prio;

                                                                /* Enable timer.                                        */
    OS_CPU_CM_SYST_CSR   |= OS_CPU_CM_SYST_CSR_CLKSOURCE |
                            OS_CPU_CM_SYST_CSR_ENABLE;
                                                                /* Enable timer interrupt.                              */
    OS_CPU_CM_SYST_CSR   |= OS_CPU_CM_SYST_CSR_TICKINT;
}