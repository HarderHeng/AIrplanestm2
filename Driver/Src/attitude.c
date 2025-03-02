#include "attitude.h"
#include "madgwick.h"
#include "USART.h"
#include <math.h>

#define DEG_TO_RAD (3.14159265358979323846 / 180.0)
#define RAD_TO_DEG (180.0 / 3.14159265358979323846)
// 输出的姿态角（全局变量）

// 初始化函数
void AttitudeSolver_Init(float sample_frequency, float gain)
{
    beta = gain; // 设置 Madgwick 算法增益
    q0 = 1.0f;
    q1 = 0.0f; // 初始化四元数
    q2 = 0.0f;
    q3 = 0.0f;
}

// 使用加速度计和陀螺仪更新姿态
void AttitudeSolver_UpdateIMU(float gx, float gy, float gz, float ax, float ay, float az)
{
    MadgwickAHRSupdateIMU(gx, gy, gz, ax, ay, az);
}

// 使用加速度计、陀螺仪和磁力计更新姿态
void AttitudeSolver_Update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
{
    MadgwickAHRSupdate(gx, gy, gz, ax, ay, az, mx, my, mz);
}

// 获取姿态角（欧拉角形式,单位为rad）
void AttitudeSolver_GetEulerAngles(float *roll, float *pitch, float *yaw, int *count)
{
    *roll = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2)) * RAD_TO_DEG; // 转换为角度
    *pitch = asinf(2.0f * (q0 * q2 - q3 * q1)) * RAD_TO_DEG;
    *yaw = atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3)) * RAD_TO_DEG;
    *yaw = *yaw - (K * (*count) + B); // 线性回归矫正
    *count = *count + 1;

    // 转换回来
    *roll *= DEG_TO_RAD;
    *pitch *= DEG_TO_RAD;
    *yaw *= DEG_TO_RAD;
}

// 将四元数转换为欧拉角
void quat_to_angle(float *roll, float *pitch, float *yaw)
{
    *roll = 180.0 / acos(-1.0) * atan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2));
    *pitch = 180.0 / acos(-1.0) * asin(2 * (q0 * q2 - q3 * q1));
    *yaw = 180.0 / acos(-1.0) * atan2f(q0 * q3 + q1 * q2, 0.5f - q2 * q2 - q3 * q3);
}

// USART1打印到上位机
void madgwick_display1(float *roll, float *pitch, float *yaw)
{
    int16_t roll_send = *roll * 100;
    int16_t pitch_send = *pitch * 100;
    int16_t yaw_send = *yaw * 100;

    // anotc_sendangle(4500, 5600, 9230);

    anotc_sendangle(roll_send, pitch_send, yaw_send);
}

void madgwick_display2(float *q0, float *q1, float *q2, float *q3)
{
    int16_t send1 = *q0 * 10000;
    int16_t send2 = *q1 * 10000;
    int16_t send3 = *q2 * 10000;
    int16_t send4 = *q3 * 10000;

    anotc_sendquat(send1, send2, send3, send4);

    /*利用字符串打印*/
    // char send1[10];
    // char send2[10];
    // char send3[10];
    // char send4[10];
    // float_to_string(*q0, send1, 3);
    // float_to_string(*q1, send2, 3);
    // float_to_string(*q2, send3, 3);
    // float_to_string(*q3, send4, 3);
    // USART1_printf("q0:%s ", send1);
    // USART1_printf("q1:%s ", send2);
    // USART1_printf("q2:%s ", send3);
    // USART1_printf("q3:%s\n", send4);

    // anotc_sendquat(-7322, 0, 7322, 0);
}

void madgwick_displayraw(float *ax, float *ay, float *az, float *gx, float *gy, float *gz)
{
    char acc_x[10];
    char acc_y[10];
    char acc_z[10];
    char gyro_x[15];
    char gyro_y[15];
    char gyro_z[15];

    float_to_string(*ax, acc_x, 5);
    float_to_string(*ay, acc_y, 5);
    float_to_string(*az, acc_z, 5);
    float_to_string(*gx, gyro_x, 5);
    float_to_string(*gy, gyro_y, 5);
    float_to_string(*gz, gyro_z, 5);

    USART1_printf("ax:%s ", acc_x);
    USART1_printf("ay:%s ", acc_y);
    USART1_printf("az:%s\n", acc_z);
    // USART1_printf("gx:%s ", gyro_x);
    // USART1_printf("gy:%s ", gyro_y);
    // USART1_printf("gz:%s\n", gyro_z);
}

void madgwick_printangle(float *roll, float *pitch, float *yaw)
{
    char proll[10];
    char ppitch[10];
    char pyaw[10];
    float_to_string(*roll, proll, 2);
    float_to_string(*pitch, ppitch, 2);
    float_to_string(*yaw, pyaw, 2);
    USART1_printf("roll:%s", proll);
    USART1_printf("pitch:%s", ppitch);
    USART1_printf("yaw:%s\n", pyaw);
}
