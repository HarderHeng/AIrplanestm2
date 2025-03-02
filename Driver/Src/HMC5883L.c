#include "stm32f4xx.h"                  // Device header
#include "myiic.h"
#define HMC_5883LAddres 0x3C //MPU6050的八位地址
#define ConfigRegAddress_A 0x00 
#define ConfigRegAddress_B 0x01 
#define ModeRegAddress 0x02 

#define REG_X_MSB 0x03 //输出X寄存器A中存储测量结果中的MSB（高位数据）
#define REG_X_LSB 0x04 //输出X寄存器B中存储测量结果中的MSB（低位数据）
#define REG_Z_MSB 0x05 //输出Y寄存器A中存储测量结果中的MSB（高位数据）
#define REG_Z_LSB 0x06 //输出Y寄存器B中存储测量结果中的MSB（低位数据）
#define REG_Y_MSB 0x07 //输出Z寄存器A中存储测量结果中的MSB（高位数据） 
#define REG_Y_LSB 0x08 //输出Z寄存器B中存储测量结果中的MSB（低位数据）

void HMC_WriteRegAdress(uint8_t address, uint8_t data){
	IIC_Start();
	IIC_SendByte(HMC_5883LAddres);
	IIC_ReceiveACK();
	IIC_SendByte(address);
	IIC_ReceiveACK();
	IIC_SendByte(data);
	IIC_ReceiveACK();
	IIC_Stop();
}
uint8_t HMC_ReadRegAdress(uint8_t address){
	uint8_t byte;
	IIC_Start();
	IIC_SendByte(HMC_5883LAddres);
	IIC_ReceiveACK();
	IIC_SendByte(address);
	IIC_ReceiveACK();
	
	IIC_Start();
	IIC_SendByte(HMC_5883LAddres|0x01);
	IIC_ReceiveACK();
	byte=IIC_ReceiveByte();
	IIC_SendACK();
	IIC_Stop();
	
	return byte;
}
void HMC_Init(void){
	IIC_Init();
	HMC_WriteRegAdress(ConfigRegAddress_A,0x10);
	HMC_WriteRegAdress(ConfigRegAddress_B,0x20);
	HMC_WriteRegAdress(ModeRegAddress,0x00);
}