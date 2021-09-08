#ifndef __BH1750_H__
#define __BH1750_H__
#include <stdint.h>

#define uchar unsigned char 
#define uint  unsigned int


//IO操作函数         
#define SET_IIC_BH1750_SCL() gpio_bit_set(GPIOB, GPIO_PIN_6)//输出1
#define CLR_IIC_BH1750_SCL() gpio_bit_reset(GPIOB, GPIO_PIN_6)//输出0

#define SET_IIC_BH1750_SDA() gpio_bit_set(GPIOB, GPIO_PIN_7)//输出1
#define CLR_IIC_BH1750_SDA() gpio_bit_reset(GPIOB, GPIO_PIN_7)//输出0

#define READ_BH1750_SDA      gpio_input_bit_get(GPIOB, GPIO_PIN_7)  //输入SDA

//IO方向设置
#define SDA_BH1750_IN()      gpio_mode_set(GPIOB, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, GPIO_PIN_7)//配置为输入
#define SDA_BH1750_OUT()     { gpio_mode_set(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_7);\
           gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_7);} //配置为输出

#define	  SlaveAddress   0x46   //定义器件在IIC总线中的从地址,根据ALT  ADDRESS地址引脚不同修改
                                //ALT  ADDRESS引脚接地时地址为0x46，接电源时地址为0xB8
															
extern uchar  BUF[8];           //接收数据缓存区      	
extern int   dis_data;          //变量		
extern int   mcy;               //表示进位标志位

void  Init_BH1750(void);
void  conversion(uint temp_data);
void  Single_Write_BH1750(uchar REG_Address);//单个写入数据
uchar Single_Read_BH1750(uchar REG_Address); //单个读取内部寄存器数据
void  mread(void);         //连续的读取内部寄存器数据
uint16_t read_BH1750(void);
#endif

