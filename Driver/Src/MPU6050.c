#include "stm32f4xx.h" // Device header
// #include "IICBySoftware.h"
#include "myiic.h"
#include "USART.h"
#include "Delay.h"
#include "madgwick.h"
#define MPU6050_Address 0xD0 // 从机8位地址

#define SampleRateDivider 0x19	   // 陀螺仪采样频率，值为0x07，125Hz
#define ConfigRegAddress 0x1A	   // 0x06 低通滤波频率5Hz
#define GYROConfigRegAddress 0x1B  // 0x18 陀螺仪自检及测量范围，不自检
#define AccelConfigRegAddress 0x1C // 0x01 加速计自检，不自检
#define MPU6050_INT_PIN_CFG 0x37

#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40

#define GYRO_XOUT_H 0x43
#define GYRO_XOUT_L 0x44
#define GYRO_YOUT_H 0x45
#define GYRO_YOUT_L 0x46
#define GYRO_ZOUT_H 0x47
#define GYRO_ZOUT_L 0x48

#define MPU6050_USER_CTRL 0x6A
#define PWR_MGMT_1 0x6B // 0x00 电源管理，正常启用
#define PWR_MGMT_2 0x6C

#define DEG2RAD 3.1415926f / 180.0f

#define gx_offset -1.70901
#define gy_offset 3.295
#define gz_offset 0.42725

#define P0 0.993145
#define P1 1.005377
#define P2 0.977883
#define P3 0.062463
#define P4 -0.016054
#define P5 -0.146175

// MPU写数据data到寄存器address
void MPU_WriteRegAdress(uint8_t address, uint8_t data)
{
	IIC_Start();
	IIC_SendByte(MPU6050_Address);
	IIC_ReceiveACK();
	IIC_SendByte(address);
	IIC_ReceiveACK();
	IIC_SendByte(data);
	IIC_ReceiveACK();
	IIC_Stop();
}

// mpu读取内部寄存器的值
uint8_t MPU_ReadRegAdress(uint8_t address)
{ // 从mpu中得到
	uint8_t byte;
	IIC_Start();
	IIC_SendByte(MPU6050_Address);
	IIC_ReceiveACK();
	IIC_SendByte(address);
	IIC_ReceiveACK();
	IIC_Stop();

	IIC_Start();
	IIC_SendByte(MPU6050_Address | 0x01); // 发送读模式设备地址
	IIC_ReceiveACK();
	byte = IIC_ReceiveByte();
	// IIC_SendACK(1);
	IIC_NACK();
	IIC_Stop();

	return byte;
}
/*
void MPU_GetAccelorationData(uint8_t *array){
	uint8_t NowAddress = ACCEL_XOUT_H;
	for(int i = 0;i<=2;i++){
		array[i]=((MPU_ReadRegAdress(NowAddress)<<8)|MPU_ReadRegAdress(NowAddress+1));
		NowAddress+=2;
		Delay_ms(100);
	}
}
void MPU_GetArgulaData(uint8_t *array){
	uint8_t NowAddress=GYRO_XOUT_H;
	for(int i = 0;i<=2;i++){
		array[i]=((MPU_ReadRegAdress(NowAddress)<<8)|MPU_ReadRegAdress(NowAddress+1));
		NowAddress+=2;
	}
}
*/

/**
 * @brief 根据寄存器的地址合成mpu6050的数据
 * @param regAddr
 */
int16_t MPU_GetData(uint8_t regAddr)
{
	uint8_t Data_H, Data_L;
	uint16_t data;

	Data_H = MPU_ReadRegAdress(regAddr);
	Data_L = MPU_ReadRegAdress(regAddr + 1);
	data = (Data_H << 8) | Data_L; // 合成数据
	return data;
}

// 先打印到串口,后续应该发送到蓝牙
void MPU6050_Display(void)
{
	/*
	printf("ACCEL_X:%d\t",MPU_GetData(ACCEL_XOUT_H));
	printf("ACCEL_Y:%d\t",MPU_GetData(ACCEL_YOUT_H));
	printf("ACCEL_Z:%d\t",MPU_GetData(ACCEL_ZOUT_H));

	printf("ACCEL_X:%d\t",MPU_GetData(GYRO_XOUT_H));
	printf("ACCEL_Y:%d\t",MPU_GetData(GYRO_YOUT_H));
	printf("ACCEL_Z:%d\t",MPU_GetData(GYRO_ZOUT_H));
*/
	uint8_t ax1 = MPU_ReadRegAdress(ACCEL_XOUT_H);
	Serial_SendByte(ax1);
	uint8_t ax2 = MPU_ReadRegAdress(ACCEL_XOUT_L);
	Serial_SendByte(ax2);
	uint8_t ay1 = MPU_ReadRegAdress(ACCEL_YOUT_H);
	Serial_SendByte(ay1);
	uint8_t ay2 = MPU_ReadRegAdress(ACCEL_YOUT_L);
	Serial_SendByte(ay2);
	uint8_t az1 = MPU_ReadRegAdress(ACCEL_ZOUT_H);
	Serial_SendByte(az1);
	uint8_t az2 = MPU_ReadRegAdress(ACCEL_ZOUT_L);
	Serial_SendByte(az2);

	uint8_t gx1 = MPU_ReadRegAdress(GYRO_XOUT_H);
	Serial_SendByte(gx1);
	uint8_t gx2 = MPU_ReadRegAdress(GYRO_XOUT_L);
	Serial_SendByte(gx2);
	uint8_t gy1 = MPU_ReadRegAdress(GYRO_YOUT_H);
	Serial_SendByte(gy1);
	uint8_t gy2 = MPU_ReadRegAdress(GYRO_YOUT_L);
	Serial_SendByte(gy2);
	uint8_t gz1 = MPU_ReadRegAdress(GYRO_ZOUT_H);
	Serial_SendByte(gz1);
	uint8_t gz2 = MPU_ReadRegAdress(GYRO_ZOUT_L);
	Serial_SendByte(gz2);
}

// void get_mpu1(nodegg_handle_t it)
// {
// 	it->gx = MPU_GetData(GYRO_XOUT_H) * 4000 / 65536.0;
// 	it->gy = MPU_GetData(GYRO_YOUT_H) * 4000 / 65536.0;
// 	it->gz = MPU_GetData(GYRO_ZOUT_H) * 4000 / 65536.0;

// 	it->ax = (float)MPU_GetData(ACCEL_XOUT_H) * 4 / 65536;
// 	it->ay = (float)MPU_GetData(ACCEL_YOUT_H) * 4 / 65536;
// 	it->az = (float)MPU_GetData(ACCEL_ZOUT_H) * 4 / 65536;

// 	it->gx = it->gx - gx_offset;
// 	it->gy = it->gy - gy_offset;
// 	it->gz = it->gz - gz_offset;
// 	it->gx = it->gx * DEG2RAD;
// 	it->gy = it->gy * DEG2RAD;
// 	it->gz = it->gz * DEG2RAD;
// 	// 使用高斯牛顿计算得到的校准参数
// 	it->ax = (it->ax - P3) * P0; // X轴校准：减去零偏误差，乘以比例误差
// 	it->ay = (it->ay - P4) * P1; // Y轴校准
// 	it->az = (it->az - P5) * P2; // Z轴校准
// }

void MPU_Init(void)
{
	IIC_Init();
	MPU_WriteRegAdress(PWR_MGMT_1, 0x80); // 先复位mpu6050再设定
	Delay_us(10);
	MPU_WriteRegAdress(PWR_MGMT_1, 0x00); // 唤醒mpu6050
	MPU_WriteRegAdress(PWR_MGMT_1, 0x01); // 原来写的是0x01，看一些资料感觉应该写0x00
	MPU_WriteRegAdress(PWR_MGMT_2, 0x00);

	MPU_WriteRegAdress(SampleRateDivider, 0x07); // 修改0x09为0x07
	MPU_WriteRegAdress(ConfigRegAddress, 0x06);	 // dplf为10hz
	MPU_WriteRegAdress(GYROConfigRegAddress, 0x18);
	MPU_WriteRegAdress(AccelConfigRegAddress, 0x00); // 修改量程为0x00，即2g

	MPU_WriteRegAdress(MPU6050_INT_PIN_CFG, 0x02); // 打开旁路模式
	MPU_WriteRegAdress(MPU6050_USER_CTRL, 0x00);   // 关闭IIC主模式
}

void ACC_Read_MPU6050(float *x, float *y, float *z)
{
	int16_t tempX, tempY, tempZ;
	tempX = MPU_GetData(ACCEL_XOUT_H);
	tempY = MPU_GetData(ACCEL_YOUT_H);
	tempZ = MPU_GetData(ACCEL_ZOUT_H);
	*x = (float)(tempX) * 4 / 65535.0;
	*y = (float)(tempY) * 4 / 65535.0;
	*z = (float)(tempZ) * 4 / 65535.0;
	*x = (*x - P3) * P0; // X轴校准：减去零偏误差，乘以比例误差
	*y = (*y - P4) * P1; // Y轴校准
	*z = (*z - P5) * P2; // Z轴校准
}

void GYRO_Read_MPU6050(float *x, float *y, float *z)
{
	int16_t tempX, tempY, tempZ;
	tempX = MPU_GetData(GYRO_XOUT_H);
	tempY = MPU_GetData(GYRO_YOUT_H);
	tempZ = MPU_GetData(GYRO_ZOUT_H);
	*x = (float)(tempX) * 4000 / 65535.0;
	*y = (float)(tempY) * 4000 / 65535.0;
	*z = (float)(tempZ) * 4000 / 65535.0;
	*x = *x - gx_offset;
	*y = *y - gy_offset;
	*z = *z - gz_offset;
	*x = *x * DEG2RAD;
	*y = *y * DEG2RAD;
	*z = *z * DEG2RAD;
}