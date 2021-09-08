#ifndef __SOFT_I2C_H_
#define __SOFT_I2C_H_

#include "gd32e23x.h"

//IO操作函数         
#define SET_IIC_SCL() gpio_bit_set(GPIOA, GPIO_PIN_11)//输出1
#define CLR_IIC_SCL() gpio_bit_reset(GPIOA, GPIO_PIN_11)//输出0

#define SET_IIC_SDA() gpio_bit_set(GPIOA, GPIO_PIN_12)//输出1
#define CLR_IIC_SDA() gpio_bit_reset(GPIOA, GPIO_PIN_12)//输出0

#define READ_SDA   gpio_input_bit_get(GPIOA, GPIO_PIN_12)  //输入SDA

//IO方向设置
#define SDA_IN()      gpio_mode_set(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, GPIO_PIN_12)//配置为输入
#define SDA_OUT()     { gpio_mode_set(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_12);\
           gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_12);} //配置为输出
   
//IIC所有操作函数
      					 
void IIC_Init(void); 			/* 软件模拟IIC引脚初始化 */
void IIC_Start(void);			/* 启动I2C总线,即发送I2C起始条件 */
void IIC_Stop(void);			/* 结束I2C总线,即发送I2C结束条件 */
void IIC_ACK(void);				/* 发送应答 ACK */
void IIC_NACK(void);			/* 发送非应答 NACK */
uint8_t IIC_Wait_Ack(void);		/* 等待ACK */
void IIC_SendByte(uint8_t byte);/* 一个字节数据发送函数 */ 
uint8_t IIC_RcvByte(void); 		/* 一个字节数据接收函数 */


#endif
