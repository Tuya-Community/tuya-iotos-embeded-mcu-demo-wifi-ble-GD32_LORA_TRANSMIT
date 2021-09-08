/**************************************************************************
 * 文件名  ：BH1750.c
 * 描述    ：光照度传感模块     
****************************************************************************/
#include "BH1750.h"
#include "delay.h"

uchar BUF[8];  //接收数据缓存区  
int   mcy;     //进位标志

/***开始信号***/
void BH1750_Start()
{                 
	SET_IIC_BH1750_SDA();  //拉高数据线	  
	SET_IIC_BH1750_SCL();  //拉高时钟线
  Delay_us(5);      //延时
	CLR_IIC_BH1750_SDA();//START:when CLK is high,DATA change form high to low 
  Delay_us(5);     //延时
  CLR_IIC_BH1750_SCL(); ;//钳住I2C总线，准备发送或接收数据 
}

/*****停止信号******/
void BH1750_Stop()
{                  
	CLR_IIC_BH1750_SDA();//STOP:when CLK is high DATA change form low to high
	SET_IIC_BH1750_SCL();   //拉高时钟线
  Delay_us(5);     //延时
	SET_IIC_BH1750_SDA();//发送I2C总线结束信号
  Delay_us(5);     //延时				
}

/**************************************
发送应答信号
入口参数:ack (0:ACK 1:NAK)
**************************************/
void BH1750_SendACK(int ack)
{

	SDA_BH1750_OUT();
	if(ack == 1)    //写应答信号
		SET_IIC_BH1750_SDA();//发送I2C总线结束信号
	else if(ack == 0)
	  CLR_IIC_BH1750_SDA(); 
	else
		return;			
	 SET_IIC_BH1750_SCL(); //拉高时钟线
   Delay_us(5);   //延时
	 CLR_IIC_BH1750_SCL();//拉低时钟线
   Delay_us(5);  //延时
}

/**************************************
接收应答信号
**************************************/
int BH1750_RecvACK()
{
	SDA_BH1750_IN(); /*这里一定要设成输入上拉，否则不能读出数据*/
	SET_IIC_BH1750_SCL(); //拉高时钟线
  Delay_us(5);   //延时	
	if(READ_BH1750_SDA == 1)//读应答信号
    mcy = 1 ;  
  else
    mcy = 0 ;				
 	CLR_IIC_BH1750_SCL(); //拉低时钟线
  Delay_us(5);                 //延时
  SDA_BH1750_OUT();
	return mcy;	
}

/**************************************
向IIC总线发送一个字节数据
**************************************/
void BH1750_SendByte(uchar dat)
{
  uchar i;
  for (i=0; i<8; i++)         //8位计数器
  {
		if( 0X80 & dat )
      SET_IIC_BH1750_SDA();
    else
      CLR_IIC_BH1750_SDA();
		dat <<= 1;
   	SET_IIC_BH1750_SCL();           //拉高时钟线
    Delay_us(5);             //延时
    CLR_IIC_BH1750_SCL();	         //拉低时钟线
    Delay_us(5);             //延时
  }
  BH1750_RecvACK();
}

uchar BH1750_RecvByte()
{
  uchar i;
  uchar dat = 0;
	uchar bit;
	SDA_BH1750_IN();/*这里一定要设成输入上拉，否则不能读出数据*/
 SET_IIC_BH1750_SDA();      //使能内部上拉,准备读取数据,
  for (i=0; i<8; i++)     //8位计数器
  {
    dat <<= 1;
    SET_IIC_BH1750_SCL();           //拉高时钟线
    Delay_us(5);             //延时
			
		if( SET == READ_BH1750_SDA)
      bit = 0X01;
    else
      bit = 0x00;  
		dat |= bit;             //读数据    
		CLR_IIC_BH1750_SCL();          //拉低时钟线
    Delay_us(5);            //延时
  }		
	 SDA_BH1750_OUT();
  return dat;
}

void Single_Write_BH1750(uchar REG_Address)
{
  BH1750_Start();                  //起始信号
  BH1750_SendByte(SlaveAddress);   //发送设备地址+写信号
  BH1750_SendByte(REG_Address);    //内部寄存器地址，请参考中文pdf22页 
  BH1750_Stop();                   //发送停止信号
}

//初始化BH1750，根据需要请参考pdf进行修改****
void Init_BH1750()
{
 
	 /*开启GPIOB的外设时钟*/ 
	rcu_periph_clock_enable(RCU_GPIOB);   
	gpio_mode_set(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_6|GPIO_PIN_7); 
	gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_6|GPIO_PIN_7); 	
	
  Single_Write_BH1750(0x01);  
	Delay_ms(180);            //延时180ms
}

//连续读出BH1750内部数据
void mread(void)
{   
	uchar i;	
  BH1750_Start();                          //起始信号
  BH1750_SendByte(SlaveAddress+1);         //发送设备地址+读信号
	
	for (i=0; i<3; i++)                      //连续读取6个地址数据，存储中BUF
  {
    BUF[i] = BH1750_RecvByte();          //BUF[0]存储0x32地址中的数据
    if (i == 3)
    {
      BH1750_SendACK(1);                //最后一个数据需要回NOACK
    }
    else
    {		
      BH1750_SendACK(0);                //回应ACK
    }
  }
  BH1750_Stop();                          //停止信号
  Delay_ms(5);
}

uint16_t read_BH1750(void)
{
  int dis_data;                       //变量	
	float temp1;
	float temp2;
	Single_Write_BH1750(0x01);   // power on
  Single_Write_BH1750(0x10);   // H- resolution mode
  Delay_ms(180);            //延时180ms
	mread();       //连续读出数据，存储在BUF中
  dis_data=BUF[0];
  dis_data=(dis_data<<8)+BUF[1]; //合成数据 
	temp1=dis_data/1.2;
	temp2=10*dis_data/1.2;	
	temp2=(int)temp2%10;	
	return (uint16_t)temp1;
}
