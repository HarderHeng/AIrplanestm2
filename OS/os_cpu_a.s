    
    EXTERN OSRunning
    EXTERN OSPrioCur
    EXTERN OSPrioHighRdy
    EXTERN OSTCBCur
    EXTERN OSTCBHighRdy
    EXTERN OSTACKSwHook
    EXTERN OS_CPU_ExceptStkBase 

    EXPORT OS_CPU_PendSVHandler
    EXPORT OSCtxSW
    EXPORT OSIntCtxSw
    EXPORT OS_Start_HighRdy


    ;*********************************************************
    AREA switch_my,CODE,READONLY
OS_CPU_PendSVHandler
    ;push register
    MRS R0,PSP              ;Get the process SP读取此时任务栈的指针，把剩下的寄存器压入任务栈
    STMFD R0!,{R4-R11,R14}  ;Save remaining regs r4-r11 & r14 on process stack

    ;Save the process SP in its TCB, 把此时的栈指针放回TCB的顶部
    ;OSTCBCur->OSTCBStkPtr = SP,R0放的是栈顶
    LDR R1,=OSTCBCur    ;R1存放被切换的TCB的指针
    LDR R1,[R1]         ;R1为TCB的首地址
    STR R0,[R1]         ;把R0放入首地址

    ;保护现场完成
    ;Call OSTaskSwHook();
    ;调用hook，要先维护一下LR
    MOV R4,LR           ;Save LR exc_return value
    BL OSTACKSwHook

    ;Get current high priority
    ;OSPrioCur=OSPrioHighRdy
    LDR R0,=OSPrioCur
    LDR R1,=OSPrioHighRdy
    LDRB R3,[R1]
    STRB R3,[R0]

    ;Get current ready thread TCB
    ;OSTCBCur=OSTCBHighRdy
    LDR R1,=OSTCBHighRdy
    LDR R3,[R1];R3放的是最高优先级的地址
    STR R3,[R2];R2指向TCBcur

    ;set psp as sp
    ORR LR,R4,#0x04 ;?

    ;psp = OSTCBStkPtr
    MSR PSP,R0

    ;pop
    LDR R0, [R3]
    LDMFD R0!, {R4-R11, R14}
    MSR PSP, R0
    ;bx
    BX LR

;*******************************************************************

OSCtxSW
OSIntCtxSw//在32中这两种一致
    ;call PendSVHandler
    LDR R0,=ICSR
    MOV R1,1<<28
    STR R1,[R0]
    BX LR

;********************************************************************


SHCR EQU 0xE000ED20;system handler control register 
OS_Start_HighRdy;让操作系统第一个就绪的任务进行,半个上下文切换，不需要保存现场
    ;需要临界资源访问限制：开中断和关中断
    
    ;开中断
    CPSID I

    ; Setup PendSV exception priority to lowest:0xff
    LDR R0,=SHCR
    MOV R1,#0XFF
    STRB R1,[R0]


    ;init msp,psp
    ;给psp置0：初始化占位:Set initial PSP to 0, to tell context switcher this is first run;
    LDR R0,#0
    MSR PSP,R0
    
    ;msp:Set the main stack to OS_CPU_ExceptStkBase
    LDR     R0, =OS_CPU_ExceptStkBase                         ; Initialize the MSP to the OS_CPU_ExceptStkBase
    LDR     R1, [R0]
    MSR     MSP, R1


    ;hooK
    ;这个地方不能是0，32与2440的不同之处
    BL OSTaskSwHook

    ;Set OSRunning to TRUE
    LDR R0,=OSRunning
    MOVS R1,#1
    STRB R1,[R0]

    ; Get current high priority
    ;OSPrioCur=OSPrioHighRdy
    LDR R0,=OSPrioCur
    LDR R1,=OSPrioHighRdy
    LDRB R2,[R1];把priohighrdy指向的内容放入R2
    STRB R2,[R0];让priocur指向最高优先级
    ;R2中的内容即当前最高优先级

    ;Get current ready thread TCB,
    ;OSTCBCur=OSTCBHighRdy
    LDR R0,=OSTCBCur
    LDR R1,=OSTCBHighRdy
    LDR R3,[R1];R3放的是最高优先级任务的地址
    STR R3,[R0];R0放最高优先级任务的地址

    ;set psp：
    LDR     R0, [R2]      ;R0存放当前运行任务的栈顶指针                                      ; R0 is new process SP; SP = OSTCBHighRdy->OSTCBStkPtr;
    MSR     PSP, R0       ;PSP为任务指针
    
    ; Get new process SP from TCB, SP = OSTCBHighRdy->OSTCBStkPtr
    ;psp=StkPtr 不算异常，需要直接寄存器操作：修改控制寄存器的值
    MRS R0,CONTROL
    ORR R0,R0,#2    ;设置SPSEL
    MSR CONTROL,R0
    ISB

    ;Restore R0-R11 and R14 from new process stack
    ;pop
    ;出掉软件和硬件压的栈：首先是R4-R11和R14 接着出掉硬件帮我们压的R0-R3和R12（ip）和LR（链接）
    LDMFD SP!, {R4-R11, LR}                                 
    LDMFD SP!, {R0-R3}                                      
    LDMFD SP!, {R12, LR}                            
    LDMFD SP!, {R1, R2} 
    ;关中断
    CPSIE I
    ;BX
    BX LR;R1

    END