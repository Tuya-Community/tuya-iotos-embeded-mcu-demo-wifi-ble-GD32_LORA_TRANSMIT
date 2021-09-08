/**************************************************************************
 * �ļ���  ��BH1750.c
 * ����    �����նȴ���ģ��     
****************************************************************************/
#include "BH1750.h"
#include "delay.h"

uchar BUF[8];  //�������ݻ�����  
int   mcy;     //��λ��־

/***��ʼ�ź�***/
void BH1750_Start()
{                 
	SET_IIC_BH1750_SDA();  //����������	  
	SET_IIC_BH1750_SCL();  //����ʱ����
  Delay_us(5);      //��ʱ
	CLR_IIC_BH1750_SDA();//START:when CLK is high,DATA change form high to low 
  Delay_us(5);     //��ʱ
  CLR_IIC_BH1750_SCL(); ;//ǯסI2C���ߣ�׼�����ͻ�������� 
}

/*****ֹͣ�ź�******/
void BH1750_Stop()
{                  
	CLR_IIC_BH1750_SDA();//STOP:when CLK is high DATA change form low to high
	SET_IIC_BH1750_SCL();   //����ʱ����
  Delay_us(5);     //��ʱ
	SET_IIC_BH1750_SDA();//����I2C���߽����ź�
  Delay_us(5);     //��ʱ				
}

/**************************************
����Ӧ���ź�
��ڲ���:ack (0:ACK 1:NAK)
**************************************/
void BH1750_SendACK(int ack)
{

	SDA_BH1750_OUT();
	if(ack == 1)    //дӦ���ź�
		SET_IIC_BH1750_SDA();//����I2C���߽����ź�
	else if(ack == 0)
	  CLR_IIC_BH1750_SDA(); 
	else
		return;			
	 SET_IIC_BH1750_SCL(); //����ʱ����
   Delay_us(5);   //��ʱ
	 CLR_IIC_BH1750_SCL();//����ʱ����
   Delay_us(5);  //��ʱ
}

/**************************************
����Ӧ���ź�
**************************************/
int BH1750_RecvACK()
{
	SDA_BH1750_IN(); /*����һ��Ҫ������������������ܶ�������*/
	SET_IIC_BH1750_SCL(); //����ʱ����
  Delay_us(5);   //��ʱ	
	if(READ_BH1750_SDA == 1)//��Ӧ���ź�
    mcy = 1 ;  
  else
    mcy = 0 ;				
 	CLR_IIC_BH1750_SCL(); //����ʱ����
  Delay_us(5);                 //��ʱ
  SDA_BH1750_OUT();
	return mcy;	
}

/**************************************
��IIC���߷���һ���ֽ�����
**************************************/
void BH1750_SendByte(uchar dat)
{
  uchar i;
  for (i=0; i<8; i++)         //8λ������
  {
		if( 0X80 & dat )
      SET_IIC_BH1750_SDA();
    else
      CLR_IIC_BH1750_SDA();
		dat <<= 1;
   	SET_IIC_BH1750_SCL();           //����ʱ����
    Delay_us(5);             //��ʱ
    CLR_IIC_BH1750_SCL();	         //����ʱ����
    Delay_us(5);             //��ʱ
  }
  BH1750_RecvACK();
}

uchar BH1750_RecvByte()
{
  uchar i;
  uchar dat = 0;
	uchar bit;
	SDA_BH1750_IN();/*����һ��Ҫ������������������ܶ�������*/
 SET_IIC_BH1750_SDA();      //ʹ���ڲ�����,׼����ȡ����,
  for (i=0; i<8; i++)     //8λ������
  {
    dat <<= 1;
    SET_IIC_BH1750_SCL();           //����ʱ����
    Delay_us(5);             //��ʱ
			
		if( SET == READ_BH1750_SDA)
      bit = 0X01;
    else
      bit = 0x00;  
		dat |= bit;             //������    
		CLR_IIC_BH1750_SCL();          //����ʱ����
    Delay_us(5);            //��ʱ
  }		
	 SDA_BH1750_OUT();
  return dat;
}

void Single_Write_BH1750(uchar REG_Address)
{
  BH1750_Start();                  //��ʼ�ź�
  BH1750_SendByte(SlaveAddress);   //�����豸��ַ+д�ź�
  BH1750_SendByte(REG_Address);    //�ڲ��Ĵ�����ַ����ο�����pdf22ҳ 
  BH1750_Stop();                   //����ֹͣ�ź�
}

//��ʼ��BH1750��������Ҫ��ο�pdf�����޸�****
void Init_BH1750()
{
 
	 /*����GPIOB������ʱ��*/ 
	rcu_periph_clock_enable(RCU_GPIOB);   
	gpio_mode_set(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_6|GPIO_PIN_7); 
	gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_6|GPIO_PIN_7); 	
	
  Single_Write_BH1750(0x01);  
	Delay_ms(180);            //��ʱ180ms
}

//��������BH1750�ڲ�����
void mread(void)
{   
	uchar i;	
  BH1750_Start();                          //��ʼ�ź�
  BH1750_SendByte(SlaveAddress+1);         //�����豸��ַ+���ź�
	
	for (i=0; i<3; i++)                      //������ȡ6����ַ���ݣ��洢��BUF
  {
    BUF[i] = BH1750_RecvByte();          //BUF[0]�洢0x32��ַ�е�����
    if (i == 3)
    {
      BH1750_SendACK(1);                //���һ��������Ҫ��NOACK
    }
    else
    {		
      BH1750_SendACK(0);                //��ӦACK
    }
  }
  BH1750_Stop();                          //ֹͣ�ź�
  Delay_ms(5);
}

uint16_t read_BH1750(void)
{
  int dis_data;                       //����	
	float temp1;
	float temp2;
	Single_Write_BH1750(0x01);   // power on
  Single_Write_BH1750(0x10);   // H- resolution mode
  Delay_ms(180);            //��ʱ180ms
	mread();       //�����������ݣ��洢��BUF��
  dis_data=BUF[0];
  dis_data=(dis_data<<8)+BUF[1]; //�ϳ����� 
	temp1=dis_data/1.2;
	temp2=10*dis_data/1.2;	
	temp2=(int)temp2%10;	
	return (uint16_t)temp1;
}
