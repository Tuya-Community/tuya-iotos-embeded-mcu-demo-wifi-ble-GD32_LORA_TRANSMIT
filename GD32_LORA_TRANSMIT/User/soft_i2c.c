#include "soft_i2c.h"
#include "systick.h"
#include "delay.h"
/* 软件模拟IIC引脚初始化
 * IIC_SCL --> PA11
 * IIC_SDA --> PA12 */
 
//初始化IIC
void IIC_Init(void)
{					     	
	rcu_periph_clock_enable(RCU_GPIOA);   
	gpio_mode_set(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_11|GPIO_PIN_12); 
	gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_11|GPIO_PIN_12); 		
	gpio_bit_set(GPIOA,GPIO_PIN_11|GPIO_PIN_12);
}

/* 描述：启动I2C总线,即发送I2C起始条件. 
 * 参数：  无
 * 返回值：无						*/
void IIC_Start(void)
{
	SDA_OUT();     //sda线输出
	SET_IIC_SDA();	  	  
	SET_IIC_SCL();
	Delay_us(4);
 	CLR_IIC_SDA();//START:when CLK is high,DATA change form high to low 
	Delay_us(4);
	CLR_IIC_SCL();//钳住I2C总线，准备发送或接收数据 
}

/* 描述：结束I2C总线,即发送I2C结束条件.  
 * 参数：  无
 * 返回值：无						*/
void IIC_Stop(void)
{
	
	SDA_OUT();//sda线输出
	CLR_IIC_SCL();
	CLR_IIC_SDA();//STOP:when CLK is high DATA change form low to high
	Delay_us(4);
	SET_IIC_SCL(); 
	Delay_us(4);
	SET_IIC_SDA();//发送I2C总线结束信号
	Delay_us(4);
}
 
/* 描述：发送应答 ACK 
 * 参数：  无
 * 返回值：无		*/
void IIC_ACK(void)
{
	SDA_OUT();
	CLR_IIC_SCL();
	Delay_us(2);
	CLR_IIC_SDA();
	Delay_us(2);
	SET_IIC_SCL();
	Delay_us(2);
	CLR_IIC_SCL();
	Delay_us(1);    
}

/* 描述：发送非应答 NACK 
 * 参数：  无
 * 返回值：无		*/
void IIC_NACK(void)
{
  SDA_OUT();
	CLR_IIC_SCL();
	Delay_us(2); 
	SET_IIC_SDA();
	Delay_us(2);
	SET_IIC_SCL();
	Delay_us(2);
	CLR_IIC_SCL();
  Delay_us(1); 
	
}

/* 描述：等待ACK 
 * 参数：  无
 * 返回值：等待应答返回0，没有等待到应答返回1	*/
uint8_t IIC_Wait_Ack(void)
{
    uint8_t t = 200;
    SDA_OUT();
    SET_IIC_SDA();	
    Delay_us(1);
    CLR_IIC_SCL();//时钟输出0 	   
    Delay_us(1); 
    SDA_IN();		/* 数据发送完后释放数据线，准备接收应答位 */
    Delay_us(1); 
    while(READ_SDA)	/* 等待IIC应答*/
    {
		t--;
		Delay_us(1); 
		if(t==0)
		{
			CLR_IIC_SCL();
			return 1;
		}
		Delay_us(1); 
    }
    Delay_us(1);      
   	SET_IIC_SCL();
    Delay_us(1);
    CLR_IIC_SCL();            
    Delay_us(1);    
    return 0;	
}

/* 描述：一个字节数据发送函数               
 * 参数：  无
 * 返回值：无		*/
void IIC_SendByte(uint8_t byte)
{
	uint8_t BitCnt;
	SDA_OUT();
  CLR_IIC_SCL();//拉低时钟开始数据传输
	for(BitCnt=0;BitCnt<8;BitCnt++) /* 要传送的数据长度为8位 */
	{
		if(byte&0x80)	SET_IIC_SDA();	/* 判断发送位 */
		else 	CLR_IIC_SDA();
		byte<<=1;
		Delay_us(2); 
		SET_IIC_SCL();
		Delay_us(2);
		CLR_IIC_SCL();	
		Delay_us(2);
		
	}
}

/* 描述：一个字节数据接收函数               
 * 参数：  无
 * 返回值：接收到的字节数据		*/   
uint8_t IIC_RcvByte(void)
{
	uint8_t retc;
	uint8_t BitCnt;
	retc=0; 
	SDA_IN();			/* 设置数据线为输入方式 */
	Delay_us(1);                    
	for(BitCnt=0;BitCnt<8;BitCnt++)
	{  
		CLR_IIC_SCL();		/* 设置时钟线为低，准备接收数据位	*/
		Delay_us(2);               
		SET_IIC_SCL();		/* 设置时钟线为高使数据线上数据有效  */              
		retc=retc<<1;
		if(READ_SDA) retc |=1;	/* 读数据位,接收的数据位放入retc中 */
		Delay_us(1);
	}
  CLR_IIC_SCL();  
	return(retc);
}
