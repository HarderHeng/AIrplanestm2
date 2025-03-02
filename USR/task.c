#include "includes.h"
#include "os_cpu.h"
#include "task.h"
#include "mutex.h"

/*获取六个imu数据和三个姿态角数据，提供给taskpid*/

float angle_roll;
float angle_pitch;
float angle_yaw;
int count;
MPU6050Data attitude;

/**
 * @brief 获取六轴数据和欧拉角
 */
void TASK_Attitude(void *p_arg)
{

    while (1)
    {
        OSMutexPend(attitude_mutex, 10, &attitude_err);
        if (attitude_err == OS_ERR_NONE)
        {
            ACC_Read_MPU6050(&(attitude.acc_x), &(attitude.acc_y), &(attitude.acc_z));
            GYRO_Read_MPU6050(&(attitude.gyro_x), &(attitude.gyro_y), &(attitude.gyro_z));
            AttitudeSolver_UpdateIMU(attitude.gyro_x, attitude.gyro_y, attitude.gyro_z, attitude.acc_x, attitude.acc_y, attitude.acc_z);
            // AttitudeSolver_Update(attitude.gyro_x, attitude.gyro_y, attitude.gyro_z, attitude.acc_x, attitude.acc_y, attitude.acc_z,hmc.x,hmc.y,hmc.z);
            // AttitudeSolver_GetEulerAngles(&(angle_roll), &(angle_pitch), &(angle_yaw), &(count));
            quat_to_angle(&angle_roll, &angle_pitch, &angle_yaw);
            // madgwick_printangle(&angle_roll, &angle_pitch, &angle_yaw);
            // M_display(Motor[0], Motor[1], Motor[2], Motor[3]);
            // madgwick_display2(&q0, &q1, &q2, &q3);
            // madgwick_display1(&angle_roll, &angle_pitch, &angle_yaw);
            // pid_display(angle_roll, angle_pitch, angle_yaw);

            OSMutexPost(attitude_mutex);
            // USART1_printf("1");
        }
        else
        {
            // USART1_printf("EA\n");
        }
        OSTimeDly(2);
    }
}

/**
 * @brief 进行pid，并对电机进行赋值
 */
void TASK_Pid(void *p_arg)
{

    while (1)
    {
        OSMutexPend(attitude_mutex, 10, &attitude_err);
        if (attitude_err == OS_ERR_NONE)
        {
            rate_pid();
            // M_display(Motor[0], Motor[1], Motor[2], Motor[3]);
            // USART1_printf("2");
            OSMutexPost(attitude_mutex);
        }
        else
        {
            // USART1_printf("EP\n");
        }
        OSTimeDly(2);
    }
}
