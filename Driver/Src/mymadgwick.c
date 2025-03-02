#include <stdio.h>
#include "stm32f4xx.h" // Device header
#include "IICBySoftware.h"
#include "USART.h"
#include "Delay.h"
#include "madgwick.h"
#include "MPU6050.h"
#include "math.h"
#include "anotc.h"
#include "HMC5883L.h"
#define DEG2RAD 3.14f / 180.0f
#define MADGWICK_BETA 0.8f
// #define MADGWICK_BETA sqrt(3.0f / 4.0f) * 0.05f / 180 * 3.14
#define MADGWICK_SAMPLE_RATE 100.0f
#define PI 3.1415926535897932
// 定义结构体

// typedef struct quat_data
// {
//     float q1;
//     float q2;
//     float q3;
//     float q0;
// } quat_data_t;

// typedef struct madgwick_cfg
// {
//     float beta;
//     float freq;
// } madgwick_cfg_t;

// typedef struct madgwick
// {
//     float beta;
//     float freq;
//     float q1;
//     float q2;
//     float q3;
//     float q0;
// } madgwick_t;
// quat_data_t quat_answer; 移到h文件中
quat_data_t q_est_pre = {1.0f, 0.0f, 0.0f, 0.0f};
quat_data_t q_est_now = {1.0f, 0.0f, 0.0f, 0.0f};

// 定义了一个指针，指向结构体madgwick，该结构体定义了beta、采样频率、一个信号量锁、和q0-3

// 快速求平方根反比
static float invSqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long *)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float *)&i;
    y = y * (1.5f - (halfx * y * y));
    // float y = sqrt(x);
    // y = 1.000000 / y;
    return y;
}

// 初始化madgwick滤波器
madgwick_handle_t madgwick_init(madgwick_cfg_t *config)
{
    // 省略非裸机状态下对信号量互斥的操作（以下皆是
    madgwick_handle_t handle = (madgwick_handle_t)calloc(1, sizeof(madgwick_t));
    handle->beta = config->beta;
    handle->freq = config->freq;
    // handle->q1 = 0.0f;
    // handle->q2 = 0.0f;
    // handle->q3 = 0.0f;
    // handle->q0 = 1.0f;

    return handle;
}

// 已知gxyz和axyz，这里需要一个获取mpu6050的数据的操作，明确得到的数据是什么，转换为下面可计算的数据

// 计算得出数据融合后的姿态
void Update_Quat6dof(madgwick_handle_t handle, float gx, float gy, float gz, float ax, float ay, float az)
{
    q_est_pre = q_est_now;
    float q0 = q_est_pre.q0;
    float q1 = q_est_pre.q1;
    float q2 = q_est_pre.q2;
    float q3 = q_est_pre.q3;

    // float q0 = handle->q0; // 带入上一次滤波得到的值
    // float q1 = handle->q1;
    // float q2 = handle->q2;
    // float q3 = handle->q3;
    float beta = handle->beta; // 设定madgwick的工具值
    float sampleFreq = handle->freq;
    float recipNorm;                                                                    // 这个值用来取模
    float s0, s1, s2, s3;                                                               // 加速度计得到的方向参考值
    float qDot1, qDot2, qDot3, qDot4;                                                   // 梯度下降的方向
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2, q3q3; // 工具值

    // 陀螺仪部分，求dQ/dt
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    // 如果加速度计测出来的数据是合法的就继续下面的操作
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {

        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        _2q0 = 2.0f * q0;
        _2q1 = 2.0f * q1;
        _2q2 = 2.0f * q2;
        _2q3 = 2.0f * q3;
        _4q0 = 4.0f * q0;
        _4q1 = 4.0f * q1;
        _4q2 = 4.0f * q2;
        _8q1 = 8.0f * q1;
        _8q2 = 8.0f * q2;
        q0q0 = q0 * q0;
        q1q1 = q1 * q1;
        q2q2 = q2 * q2;
        q3q3 = q3 * q3;

        // 得到迭代的梯度
        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
        recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;
        // 得到梯度除以梯度的模的表达式s0-3

        qDot1 -= beta * s0;
        qDot2 -= beta * s1;
        qDot3 -= beta * s2;
        qDot4 -= beta * s3; // 得到融合后的迭代参数qDot1-4
    }

    q0 += qDot1 * (1.00f / sampleFreq);
    q1 += qDot2 * (1.00f / sampleFreq);
    q2 += qDot3 * (1.00f / sampleFreq);
    q3 += qDot4 * (1.00f / sampleFreq);
    // 得到校准后的q

    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm; // 化为单位四元数

    // handle->q0 = q0;
    // handle->q1 = q1;
    // handle->q2 = q2;
    // handle->q3 = q3; // 得到新的一组四元数,放入当下的结构体中
    q_est_now.q0 = q0;
    q_est_now.q1 = q1;
    q_est_now.q2 = q2;
    q_est_now.q3 = q3;

    return;
}

void Update_Quat9dof(madgwick_handle_t handle, float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
{
    if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f))
    {
        Update_Quat6dof(handle, gx, gy, gz, ax, ay, az);
    }

    float q0 = handle->q0;
    float q1 = handle->q1;
    float q2 = handle->q2;
    float q3 = handle->q3;
    float beta = handle->beta;
    float sampleFreq = handle->freq;
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float hx, hy;
    float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {

        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Normalise magnetometer measurement
        recipNorm = invSqrt(mx * mx + my * my + mz * mz);
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        _2q0mx = 2.0f * q0 * mx;
        _2q0my = 2.0f * q0 * my;
        _2q0mz = 2.0f * q0 * mz;
        _2q1mx = 2.0f * q1 * mx;
        _2q0 = 2.0f * q0;
        _2q1 = 2.0f * q1;
        _2q2 = 2.0f * q2;
        _2q3 = 2.0f * q3;
        _2q0q2 = 2.0f * q0 * q2;
        _2q2q3 = 2.0f * q2 * q3;
        q0q0 = q0 * q0;
        q0q1 = q0 * q1;
        q0q2 = q0 * q2;
        q0q3 = q0 * q3;
        q1q1 = q1 * q1;
        q1q2 = q1 * q2;
        q1q3 = q1 * q3;
        q2q2 = q2 * q2;
        q2q3 = q2 * q3;
        q3q3 = q3 * q3;

        // Reference direction of Earth's magnetic field
        hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
        hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
        _2bx = sqrt(hx * hx + hy * hy);
        _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
        _4bx = 2.0f * _2bx;
        _4bz = 2.0f * _2bz;

        // Gradient decent algorithm corrective step 得到迭代的梯度
        s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        // Apply feedback step 得到融合后的迭代参数
        qDot1 -= beta * s0;
        qDot2 -= beta * s1;
        qDot3 -= beta * s2;
        qDot4 -= beta * s3;

        return;
    }

    // Integrate rate of change of quaternion to yield quaternion
    q0 += qDot1 * (1.0f / sampleFreq);
    q1 += qDot2 * (1.0f / sampleFreq);
    q2 += qDot3 * (1.0f / sampleFreq);
    q3 += qDot4 * (1.0f / sampleFreq);

    // Normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    handle->q0 = q0;
    handle->q1 = q1;
    handle->q2 = q2;
    handle->q3 = q3; // 得到一组新的四元数

    return;
}

// 获得一组姿态
void madgwick_get_quaternion(madgwick_handle_t handle, quat_data_handle_t quat)
{

    // quat->q0 = handle->q0;
    // quat->q1 = handle->q1;
    // quat->q2 = handle->q2;
    // quat->q3 = handle->q3;
    quat->q0 = q_est_now.q0;
    quat->q1 = q_est_now.q1;
    quat->q2 = q_est_now.q2;
    quat->q3 = q_est_now.q3;
    return;
}

void madgwick_test()
{
    madgwick_handle_t madgwick_handle;

    madgwick_cfg_t madgwick_cfg;                    // 初始化madgwick滤波器
    madgwick_cfg.beta = MADGWICK_BETA;              // 设置beta参数（beta越大加速度计的收敛作用越大）
    madgwick_cfg.freq = MADGWICK_SAMPLE_RATE;       // 设置采样频率
    madgwick_handle = madgwick_init(&madgwick_cfg); // 初始化madgwick滤波器
    nodegg_handle_t ken = (nodegg_handle_t)calloc(1, sizeof(nodegg_t));
    while (1)
    {
        get_mpu1(ken);
        // get_mpu2(ken);

        // Update_Quat9dof(madgwick_handle, ken->gx, ken->gy, ken->gz, ken->ax, ken->ay, ken->az,ken->mx,ken->my,ken->mz);
        Update_Quat6dof(madgwick_handle, ken->gx, ken->gy, ken->gz, ken->ax, ken->ay, ken->az);

        madgwick_get_quaternion(madgwick_handle, &quat_answer);
        // float roll = 180.0 / acos(-1.0) * atan2(2 * (quat_answer.q0 * quat_answer.q1 + quat_answer.q2 * quat_answer.q3), 1 - 2 * (quat_answer.q1 * quat_answer.q1 + quat_answer.q2 * quat_answer.q2));
        // float pitch = 180.0 / acos(-1.0) * asin(2 * (quat_answer.q0 * quat_answer.q2 - quat_answer.q3 * quat_answer.q1));
        // float yaw = 180.0 / acos(-1.0) * atan2f(quat_answer.q0 * quat_answer.q3 + quat_answer.q1 * quat_answer.q2, 0.5f - quat_answer.q2 * quat_answer.q2 - quat_answer.q3 * quat_answer.q3);
        // 将四元数转换为欧拉角

        // Roll (x-axis rotation)
        float sinr_cosp = 2.0f * (quat_answer.q0 * quat_answer.q1 + quat_answer.q2 * quat_answer.q3);
        float cosr_cosp = 1.0f - 2.0f * (quat_answer.q1 * quat_answer.q1 + quat_answer.q2 * quat_answer.q2);
        float roll = atan2(sinr_cosp, cosr_cosp) * 180.0f / PI;
        // Pitch (y-axis rotation)
        float pitch = 0;
        float sinp = 2.0f * (quat_answer.q0 * quat_answer.q2 - quat_answer.q3 * quat_answer.q1);
        if (fabs(sinp) >= 1.0f)
        {
            pitch = copysign(90.0f, sinp); // 使用90度防止Pitch锁死
        }
        else
        {
            pitch = asin(sinp) * 180.0f / PI;
        }
        // Yaw (z-axis rotation)
        float siny_cosp = 2.0f * (quat_answer.q0 * quat_answer.q3 + quat_answer.q1 * quat_answer.q2);
        float cosy_cosp = 1.0f - 2.0f * (quat_answer.q2 * quat_answer.q2 + quat_answer.q3 * quat_answer.q3);
        float yaw = atan2(siny_cosp, cosy_cosp) * 180.0f / PI;

        /*上位机精度为两位小数*/
        angle_answer->roll = roll * 100;
        angle_answer->pitch = pitch * 100;
        angle_answer->yaw = yaw * 100;
        // angle_answer->roll = roll;
        // angle_answer->pitch = pitch;
        // angle_answer->yaw = yaw;

        /*输出数据*/
        // MPU6050_Display();
        // madgwick_display1(angle_answer);
        // madgwick_display2(quat_answer);
        madgwick_displayraw(ken);

        // Delay_ms(100);
    }
}

// USART1打印到上位机
void madgwick_display1(angle_handle_t angle_answer_it)
{
    int16_t roll_send = angle_answer_it->roll;
    int16_t pitch_send = angle_answer_it->pitch;
    int16_t yaw_send = angle_answer_it->yaw;

    // anotc_sendangle(4500, 5600, 9230);

    anotc_sendangle(roll_send, pitch_send, yaw_send);
    /*利用字符串打印*/
    // char roll[10];
    // char pitch[10];
    // char yaw[10];
    // float_to_string(angle_answer_it->roll, roll, 2);
    // float_to_string(angle_answer_it->pitch, pitch, 2);
    // float_to_string(angle_answer_it->yaw, yaw, 2);
    // USART1_printf("roll:%s", roll);
    // USART1_printf("pitch:%s", pitch);
    // USART1_printf("yaw:%s\n", yaw);
}

void madgwick_display2(quat_data_t quat_data_it)
{
    int16_t send1 = quat_data_it.q0 * 10000;
    int16_t send2 = quat_data_it.q1 * 10000;
    int16_t send3 = quat_data_it.q2 * 10000;
    int16_t send4 = quat_data_it.q3 * 10000;

    anotc_sendquat(send1, send2, send3, send4);

    /*利用字符串打印*/
    // char send1[10];
    // char send2[10];
    // char send3[10];
    // char send4[10];
    // float_to_string(quat_data_it.q0, send1, 2);
    // float_to_string(quat_data_it.q1, send2, 2);
    // float_to_string(quat_data_it.q2, send3, 2);
    // float_to_string(quat_data_it.q3, send4, 2);
    // USART1_printf("q0:%s ", send1);
    // USART1_printf("q1:%s ", send2);
    // USART1_printf("q2:%s ", send3);
    // USART1_printf("q3:%s\n", send4);

    // anotc_sendquat(-7322, 0, 7322, 0);
}

void madgwick_displayraw(nodegg_handle_t it)
{
    char ax[10];
    char ay[10];
    char az[10];
    char gx[10];
    char gy[10];
    char gz[10];

    float_to_string(it->ax, ax, 5);
    float_to_string(it->ay, ay, 5);
    float_to_string(it->az, az, 5);
    float_to_string(it->gx, gx, 5);
    float_to_string(it->gy, gy, 5);
    float_to_string(it->gz, gz, 5);

    USART1_printf("ax:%s ", ax);
    USART1_printf("ay:%s ", ay);
    USART1_printf("az:%s\n", az);
    // USART1_printf("gx:%s ", gx);
    // USART1_printf("gy:%s ", gy);
    // USART1_printf("gz:%s\n", gz);
}
