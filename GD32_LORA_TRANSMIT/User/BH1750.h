#ifndef __BH1750_H__
#define __BH1750_H__
#include <stdint.h>

#define uchar unsigned char 
#define uint  unsigned int


//IO��������         
#define SET_IIC_BH1750_SCL() gpio_bit_set(GPIOB, GPIO_PIN_6)//���1
#define CLR_IIC_BH1750_SCL() gpio_bit_reset(GPIOB, GPIO_PIN_6)//���0

#define SET_IIC_BH1750_SDA() gpio_bit_set(GPIOB, GPIO_PIN_7)//���1
#define CLR_IIC_BH1750_SDA() gpio_bit_reset(GPIOB, GPIO_PIN_7)//���0

#define READ_BH1750_SDA      gpio_input_bit_get(GPIOB, GPIO_PIN_7)  //����SDA

//IO��������
#define SDA_BH1750_IN()      gpio_mode_set(GPIOB, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, GPIO_PIN_7)//����Ϊ����
#define SDA_BH1750_OUT()     { gpio_mode_set(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_7);\
           gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_7);} //����Ϊ���

#define	  SlaveAddress   0x46   //����������IIC�����еĴӵ�ַ,����ALT  ADDRESS��ַ���Ų�ͬ�޸�
                                //ALT  ADDRESS���Žӵ�ʱ��ַΪ0x46���ӵ�Դʱ��ַΪ0xB8
															
extern uchar  BUF[8];           //�������ݻ�����      	
extern int   dis_data;          //����		
extern int   mcy;               //��ʾ��λ��־λ

void  Init_BH1750(void);
void  conversion(uint temp_data);
void  Single_Write_BH1750(uchar REG_Address);//����д������
uchar Single_Read_BH1750(uchar REG_Address); //������ȡ�ڲ��Ĵ�������
void  mread(void);         //�����Ķ�ȡ�ڲ��Ĵ�������
uint16_t read_BH1750(void);
#endif

