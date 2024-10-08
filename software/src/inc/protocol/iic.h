/**
  ******************************************************************************
  *
  * Define protocol of iic.
  * author: coregiu
  *
  *
  ******************************************************************************
**/


#ifndef __PROTOCOL_IIC_H
#define __PROTOCOL_IIC_H

#include "stm32f10x_conf.h"
#include "sys.h"
#include "delay.h"

//IO操作函数
#define IIC_SCL PBout(10) //SCL
#define IIC_SDA PBout(11) //SDA
#define READ_SDA PBin(11) //输入SDA

//IO方向设置
#define SDA_IN()                    \
    {                               \
        GPIOB->CRL &= 0X0FFFFFFF;   \
        GPIOB->CRL |= (u32)8 << 28; \
    }
#define SDA_OUT()                   \
    {                               \
        GPIOB->CRL &= 0X0FFFFFFF;   \
        GPIOB->CRL |= (u32)3 << 28; \
    }

//IIC所有操作函数
void IIC_Init(void);                 //初始化IIC的IO口
void IIC_Start(void);                //发送IIC开始信号
void IIC_Stop(void);                 //发送IIC停止信号
void IIC_Send_Byte(u8 txd);          //IIC发送一个字节
u8 IIC_Read_Byte(unsigned char ack); //IIC读取一个字节
u8 IIC_Wait_Ack(void);               //IIC等待ACK信号
void IIC_Ack(void);                  //IIC发送ACK信号
void IIC_NAck(void);                 //IIC不发送ACK信号

int i2cWrite(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data);
int i2cRead(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf);

#endif