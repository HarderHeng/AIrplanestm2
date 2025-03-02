#ifndef __MPU6050_H
#define __MPU6050_H
void MPU_Init(void);
// void MPU_GetArgulaData(uint8_t *array);
// void MPU_GetAccelorationData(uint8_t *array);
void MPU6050_Display(void);
// void get_mpu1();
void GYRO_Read_MPU6050(float *x, float *y, float *z);
void ACC_Read_MPU6050(float *x, float *y, float *z);
#endif