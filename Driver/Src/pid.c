#include "stm32f4xx.h"
#include "MPU6050.h"
#include "pid.h"
#include <math.h>
#include "Timer.h"
#include "attitude.h"
#include "task.h"
#include "USART.h"

#define DT 1000.0f // what is dt?
float roll = 0, pitch = 0, yaw = 0, thro = 0;

/**
 * 一个pid控制器的结构
 */
typedef struct
{
    float P;       // 参数P
    float I;       // 参数I
    float D;       // 参数D
    float Desired; // 期望值
    float Error;
    float preError;
    float PrepreError; //?
    float Increment;
    float Integ;
    float iLimit;
    float Deriv;
    float Output;
} PID;

PID pitch_angle_pid; // 角度
PID roll_angle_pid;
PID yaw_angle_pid;
PID pitch_rate_pid; // 角速度
PID roll_rate_pid;
PID yaw_rate_pid;
enum
{
    ROLL,
    PITCH,
    YAW,
    THROTTLE
};
enum
{
    X,
    Y,
    Z
};

float target_x, target_y, target_z;
/*y:俯仰（pitch），x:横滚（roll），z:偏航（yaw）*/

void threeAxisrot(double r11, double r12, double r21, double r31, double r32, double *x, double *y, double *z)
{
    *x = -atan2(r31, r32);
    *y = -asin(r21);
    *z = -atan2(r11, r21);
} // 用于将旋转矩阵转换为欧拉角

/**
 * @brief PID反馈控制器
 * @param pid pid控制器的名字（分为内外环和三个角度）
 * @param target pid反馈控制的目标值（由Reiver和外环pid控制器提供）
 * @param measure 当前值（由attitude和mpu提供）
 * @param dertT 单位时间
 */
void Cal_PID_Position(PID *pid, float target, float measure, int32_t dertT)
{
    float termI = 0;
    float dt = dertT / 1000000.0;

    pid->Error = target - measure; // 首先计算距离目标值的error
    // pid->Deriv = 1;
    pid->Deriv = (pid->Error - pid->preError) / dt;                                      // 根据误差计算出变化速率
    pid->Output = (pid->P * pid->Error) + (pid->I * pid->Integ) + (pid->D * pid->Deriv); // pid计算式
    // pid->Output = (pid->P * pid->Error) + (pid->I * pid->Integ);
    pid->preError = pid->Error;
    if (fabs(pid->Error) < thro - 1000) // 比油门还大时不积分
    {
        termI = (pid->Integ) + (pid->Error) * dt;
        if (termI > -pid->iLimit && termI < pid->iLimit && pid->Output < pid->iLimit) // 在-300~300时进行积分
            pid->Integ = termI;
    }
}
// 根据给定的目标值和当前的测量值，计算PID控制器的输出，pid结构体定义了一个pid控制器，target是目标值，measure是测量值，dertT是时间间隔

/**
 * @brief 控制飞行器姿态：角度pitch和roll,外环pid
 * 返回角速度，e=target角度-当前角度
 */
void CtrlAngle(void)
{
    float angTarget[3] = {0};
    // angTarget[PITCH] = (float)(target_y);
    // angTarget[ROLL] = (float)(target_x);
    /*test begin*/
    angTarget[PITCH] = 0;
    angTarget[ROLL] = 0;
    /*test end*/

    Cal_PID_Position(&roll_angle_pid, angTarget[ROLL], angle_roll, DT);
    Cal_PID_Position(&pitch_angle_pid, angTarget[PITCH], angle_pitch, DT);
    // pitch = pitch_angle_pid.Output;
    // roll = roll_angle_pid.Output;
    // angTarget[YAW]=(float)(target_z);
}

/**
 * @brief 控制飞行器姿态：控制飞行器的角速度
 * 返回角加速度 e=target角速度（来自外环pid）-当前角速度（gy-86）
 */
void CtrlRate(void)
{
    float yawRateTarget = 0;
    // yawRateTarget = (float)target_z;
    yawRateTarget = 0;

    Cal_PID_Position(&pitch_rate_pid, pitch_angle_pid.Output, attitude.gyro_y, DT);
    Cal_PID_Position(&roll_rate_pid, roll_angle_pid.Output, attitude.gyro_x, DT);
    Cal_PID_Position(&yaw_rate_pid, yawRateTarget, attitude.gyro_z, DT);

    pitch = pitch_rate_pid.Output;
    roll = roll_rate_pid.Output;
    yaw = yaw_rate_pid.Output;
} // 控制飞行器的角速度

void Init_PID()
{
    pitch_angle_pid.P = 5;
    pitch_angle_pid.I = 0; // 1.0;		//0
    pitch_angle_pid.D = 0;

    pitch_angle_pid.iLimit = 300; // or 1000

    pitch_rate_pid.P = 2.3;
    pitch_rate_pid.I = 0; // 0.5
    pitch_rate_pid.D = 0.0;

    pitch_rate_pid.iLimit = 300;
    ////////////////////////////////////////////
    roll_angle_pid.P = 5;
    roll_angle_pid.I = 0; // 1.0;
    roll_angle_pid.D = 0;
    roll_angle_pid.iLimit = 300; // or 1000

    roll_rate_pid.P = 1;
    roll_rate_pid.I = 0; // 0.5
    roll_rate_pid.D = 0.0;
    roll_rate_pid.iLimit = 300;
    ///////////////////////////////////////////
    yaw_angle_pid.P = 1;
    yaw_angle_pid.I = 0.2;
    yaw_angle_pid.D = 0;

    yaw_rate_pid.P = 20;
    yaw_rate_pid.I = 0;
    yaw_rate_pid.D = 0;
}

void pid_display(float roll, float pitch, float yaw)
{
    char ch1[10];
    char ch2[10];
    char ch4[10];

    float_to_string(roll, ch1, 1);
    float_to_string(pitch, ch2, 1);
    float_to_string(yaw, ch4, 1);
    USART1_printf("r:%s ", ch1);
    USART1_printf("p:%s ", ch2);
    USART1_printf("y:%s\n", ch4);
}
