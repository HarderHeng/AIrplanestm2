#include "myiic.h"
#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"  // Device header
#include "stm32f4xx_gpio.h" // Device header
#include "Delay.h"

/*-----------------IIC状态相关函数--------------------------*/
static int iic_status = 0;
int getIICStatus() { return iic_status; }
void clearIICStatus() { iic_status = 0; }

/*-----------IIC位读写函数--------------------------------*/
/*向SCL写数值*/
void SCL_W(uint8_t x)
{
    GPIO_WriteBit(GPIOB, GPIO_Pin_8, (BitAction)x);
    Delay_us(10);
}
/*向SDA写数值*/
void SDA_W(uint8_t x)
{
    GPIO_WriteBit(GPIOB, GPIO_Pin_9, (BitAction)x);
    Delay_us(10);
}
/*读取SDA的数值*/
uint8_t SDA_R(void)
{
    uint8_t bit;
    bit = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_9);
    Delay_us(10);
    return bit;
}

/*--------------------------IIC初始化函数------------------------*/
void IIC_Init(void)
{
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); // 使能GPIOB的时钟
    GPIO_InitTypeDef GPIO_InitInstructure;
    GPIO_InitInstructure.GPIO_Mode = GPIO_Mode_OUT;  // 输出模式：配置为推挽输出
    GPIO_InitInstructure.GPIO_OType = GPIO_OType_OD; // 开漏输出可以不用切换输入输出模式
    GPIO_InitInstructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitInstructure.GPIO_PuPd = GPIO_PuPd_UP; // 上拉输入
    GPIO_InitInstructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitInstructure); // 初始化GPIOB的8和9口
    GPIO_SetBits(GPIOB, GPIO_Pin_8 | GPIO_Pin_9);
    // 初始化GPIOB的8和9口
}

/*----------------------------------IIC时序函数-----------------------*/
void IIC_Start(void)
{
    SDA_W(1); // 将SDA在SCL拉高之前就拉高，防止产生终止条件
    SCL_W(1);
    SDA_W(0); // 拉低SDA
    SCL_W(0); // 拉低SCL
}

void IIC_Stop(void)
{
    SDA_W(0); // 确保SDA低电平
    SCL_W(1); // 拉高SCL
    SDA_W(1); // 在SCL高电平时拉高SDA
}

/*iic应答相关时序函数*/

/*------------------------------------------------------------------------*/
/*iic发送ACK应答*/
/*SCL0->1表示应答
这个地方和前一版代码相比有较大的改动*/
void IIC_SendACK(void)
{             // byte=0
    SDA_W(0); // 发送一个应答位
    SCL_W(1); // 拉高SCL让从机读取
    SCL_W(0); // 拉低SCL进入下一个时序
    // SDA_W(1); // 主机释放SDA线
}

void IIC_NACK(void)
{
    SDA_W(1); // 主机发送1表示非应答
    SCL_W(1); // 从机读取应答
    SCL_W(0); // 读取完毕
}

/*iic接收一个应答*/
/*这一部分也和之前不一样，添加了一个等待和超时*/
uint8_t IIC_ReceiveACK(void)
{
    uint8_t bit = 0;
    uint8_t waittime = 0;
    SDA_W(1); // 主机释放SDA
    SCL_W(1); // 主机拉高SCL读取从机的应答
    while (SDA_R)
    {
        waittime++;
        if (waittime > 250)
        { 
            IIC_Stop();
            bit = 1;
            break;
        }
    }
    SCL_W(0);
    return bit;
} // 返回0表示收到应答，返回1表示没有收到应答

// uint8_t IIC_ReceiveACK(void)
// {
//     uint8_t bit;
//     SDA_W(1); // 主机释放SDA
//     SCL_W(1); // 主机拉高SCL读取从机的应答
//     bit = SDA_R();
//     SCL_W(0);
//     return bit;
// }
/*之前的返回值bit=SDA_R
如果读到的值是0，则直接返回bit=0
如果读到的值是1，那么表明bit=1，同时会激活stop*/

/*iic发送与接收数据*/

/*-------------------------------------------------------------------------*/
/*IIC发送一个字节（从高位发到低位）*/
void IIC_SendByte(uint8_t byte)
{
    uint8_t i = 0;
    for (i = 0; i < 8; i++)
    {
        SCL_W(0);                  // 新增
        SDA_W(byte & (0x80 >> i)); // 发送一个字节的一位
        SCL_W(1);
        // SCL_W(0); // 先拉高SCL再拉低，新注释
    }
    SCL_W(0);
}

// version2
// void IIC_SendByte(uint8_t byte)
// {
//     uint8_t i = 0;
//     for (i = 0; i < 8; i++)
//     {
//         SDA_W((byte & 0x80) >> 7);
//         SCL_W(1);
//         SCL_W(0);
//         byte << 1;
//     }
//     SDA_W(1);
// }

/*iic接收一个字节*/
uint8_t IIC_ReceiveByte(void)
{
    uint8_t byte = 0x00;
    uint8_t i = 0;
    SDA_W(1); // SDA拉高，从机把数据放到SDA
    for (i = 0; i < 8; i++)
    {
        SCL_W(0); // 新增
        SCL_W(1);
        if (SDA_R() == 1)
        {
            byte |= (0x80 >> i);
        }
        // SCL_W(0);
    }
    // for (int i = 7; i >= 0; i--)
    // {
    //     SCL_W(1);
    //     byte = byte | (SDA_R() << i);
    //     SCL_W(0);
    // }
    SCL_W(0);
    // IIC_SendACK(); // 参照下面同样的功能添加一个应答 新注释 这玩意不能加1127
    return byte;
}

// version2
// uint8_t IIC_ReceiveByte(uint8_t ack)
// {
//     uint8_t i, receive = 0;
//     for (i = 0; i < 8; i++) /* 接收1个字节数据 */
//     {
//         receive <<= 1; /* 高位先输出,所以先收到的数据位要左移 */
//         SCL_W(1);

//         if (SDA_R)
//         {
//             receive++;
//         }

//         SCL_W(0);
//     }

//     if (!ack)
//     {
//         IIC_NACK(); /* 发送nACK */
//     }
//     else
//     {
//         IIC_SendACK(0); /* 发送ACK */
//     }

//     return receive;
// }

/*发送一串数据*/

/*--------------------------------------------------------------------------*/
uint8_t iic_send_bytes(uint8_t *buffer, int size)
{
    for (int i = 0; i < size; i++)
    {
        IIC_SendByte(buffer[i]); // 发送数据
        if (IIC_ReceiveACK() == 1)
        { // 等待应答
            return 1;
        }
    }
    return 0;
}

// void iic_read_bytes(uint8_t *buffer, int size)
// {
//     // 连续读取size个字节
//     for (int i = 0; i < size; i++)
//     {
//         // 读出寄存器数据
//         if (i < size - 1)
//         {
//             buffer[i] = IIC_ReceiveByte(1); // 前size-1次发送ACK
//         }
//         else
//         {
//             buffer[i] = IIC_ReceiveByte(0); // 发送nACK
//         }
//     }
// }