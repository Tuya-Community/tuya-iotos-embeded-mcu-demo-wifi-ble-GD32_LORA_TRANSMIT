#include "sht3x.h"
#include "usart.h"
#include "stdio.h"
#include "soft_i2c.h"

#define write 0
#define read  1

uint8_t addr = 0x44;	//IIC地址，ADDR脚接地时为0x44，接高电平时为0x45

/* 描述：向SHT30发送一条16bit指令 
 * 参数cmd：SHT30指令（在SHT30_MODE中枚举定义）
 * 返回值：发送成功返回0，发送失败返回1 		*/
static uint8_t SHT3x_Send_Cmd(SHT3X_CMD cmd)
{
  uint8_t cmd_buffer[2];
	uint8_t ret;
  cmd_buffer[0] = cmd >> 8;
  cmd_buffer[1] = cmd;
	
	IIC_SendByte(addr<<1 | write);	/* 写7位I2C设备地址加0作为写取位 */
	ret =  IIC_Wait_Ack();
	IIC_SendByte(cmd_buffer[0]);
	ret |= IIC_Wait_Ack();
	IIC_SendByte(cmd_buffer[1]);
	ret |= IIC_Wait_Ack();
	
	return ret;
}
/**
 * @brief   复位SHT30
 * @param   none
 * @retval  none
*/
void SHT3x_reset(void)
{
    SHT3x_Send_Cmd(SOFT_RESET_CMD);
    delay_1ms(20);
}
/* 描述：从SHT3x读取数据 
 * 参数data_len：读取多少个字节数据
 * 参数data_arr：读取的数据存放在一个数组里
 * 返回值：读取成功返回0，读取失败返回1 
*/
static uint8_t SHT3x_Recv_Data(uint8_t data_len, uint8_t* data_arr)
{
	uint8_t ret,i;
	IIC_SendByte(addr<<1 | read);	/* 写7位I2C设备地址加1为读取位 */
	ret = IIC_Wait_Ack();
	if(ret != 0) return 1;
	for(i = 0; i < (data_len - 1); i++)
	{
		data_arr[i]=IIC_RcvByte();
		IIC_ACK();
	}
	data_arr[i]=IIC_RcvByte();
	IIC_NACK();

	return 0;
}

/* 描述：读取传感器编号
 * 参数：存储编号数据的指针
 * 返回值：0-读取成功，1-读取失败 */
uint8_t SHT3x_ReadSerialNumber(uint32_t* serialNumber)
{ 
	uint8_t ret = 0; 
	uint8_t Num_buf[4] = {0xFF,0xFF,0xFF,0xFF};
	
	IIC_Start();
	SHT3x_Send_Cmd(READ_SERIAL_NUMBER);
	IIC_Stop();
	delay_1ms(10);	/* 有问题时需要适当延长！！！！！！*/
	IIC_Start();
	ret = SHT3x_Recv_Data(4,Num_buf);
	IIC_Stop();
	
	*serialNumber = ((Num_buf[0] << 24) | (Num_buf[1] << 16) |(Num_buf[2] << 8) |(Num_buf[3]));
	if(0xFF == *serialNumber) return 1; 
	return ret; 
}

/* 描述：SHT3x初始化函数，并将其设置为周期测量模式
 * 参数：无
 * 返回值：初始化成功返回0，初始化失败返回1 */
uint8_t SHT3x_Init(void)
{
	uint8_t ret;
  IIC_Init();
	IIC_Start();
  ret = SHT3x_Send_Cmd(MEDIUM_2_CMD);
	IIC_Stop();
	return ret;
}

/* 描述：数据CRC校验
 * 参数message：需要校验的数据
 * 参数initial_value：crc初始值
 * 返回值：计算得到的CRC码 */
#define CRC8_POLYNOMIAL 0x31
uint8_t CheckCrc8(uint8_t* const message, uint8_t initial_value)
{
	  uint8_t  remainder;	    //余数
    uint8_t  i = 0, j = 0;  //循环变量

    /* 初始化 */
    remainder = initial_value;
    for(j = 0; j < 2;j++)
    {
        remainder ^= message[j];
        /* 从最高位开始依次计算  */
        for (i = 0; i < 8; i++)
        {
            if (remainder & 0x80)
                remainder = (remainder << 1)^CRC8_POLYNOMIAL;
            else
                remainder = (remainder << 1);
        }
    }
    /* 返回计算的CRC码 */
    return remainder;
}

/* 描述：温湿度数据获取函数,周期读取，注意，需要提前设置周期模式   
 * 参数Tem_val：存储温度数据的指针, 温度单位为°C
 * 参数Hum_val：存储湿度数据的指针, 温度单位为%
 * 返回值：0-读取成功，1-读取失败
********************************************************************/
uint8_t SHT3x_Get_Humiture_periodic(double *Tem_val,double *Hum_val)
{
	uint8_t ret=0;
	uint8_t buff[6]={0};
	uint16_t tem,hum;
	double Temperature=0;
	double Humidity=0;

	IIC_Start();
	ret = SHT3x_Send_Cmd(READOUT_FOR_PERIODIC_MODE);	
	IIC_Start();
	ret = SHT3x_Recv_Data(6,buff);
	IIC_Stop();
	
	/* 校验温度数据和湿度数据是否接收正确 */
	if(CheckCrc8(buff, 0xFF) != buff[2] || CheckCrc8(&buff[3], 0xFF) != buff[5])
	{	
		printf("CRC_ERROR,ret = 0x%x\r\n",ret);
		return 1;
	}
		
	/* 转换温度数据 */
	tem = (((uint16_t)buff[0]<<8) | buff[1]);//温度数据拼接
	Temperature= (175.0*(double)tem/65535.0-45.0) ;	// T = -45 + 175 * tem / (2^16-1)
	
	/* 转换湿度数据 */
	hum = (((uint16_t)buff[3]<<8) | buff[4]);//湿度数据拼接
	Humidity= (100.0*(double)hum/65535.0);			// RH = hum*100 / (2^16-1)
	
	/* 过滤错误数据 */
	if((Temperature>=-20)&&(Temperature<=125)&&(Humidity>=0)&&(Humidity<=100))
	{
		*Tem_val = Temperature;
		*Hum_val = Humidity;
		
		return 0;
	}
	else
		return 1;
}

/* 描述：温湿度数据获取函数，单次获取
 * 参数Tem_val：存储温度数据的指针, 温度单位为°C
 * 参数Hum_val：存储湿度数据的指针, 温度单位为%
 * 返回值：0-读取成功，1-读取失败
********************************************************************/
uint8_t SHT3x_Get_Humiture_single(double *Tem_val,double *Hum_val)
{
	uint8_t ret=0;
	uint8_t buff[6];
	uint16_t tem,hum;
	double Temperature=0;
	double Humidity=0;

	IIC_Start();
	SHT3x_Send_Cmd(HIGH_ENABLED_CMD);
	IIC_Stop();
	
	delay_1ms(50);
	
	IIC_Start();
	ret = SHT3x_Recv_Data(6,buff);
	IIC_Stop();
	
	/* 校验温度数据和湿度数据是否接收正确 */
	if(CheckCrc8(buff, 0xFF) != buff[2] || CheckCrc8(&buff[3], 0xFF) != buff[5])
	{	
		printf("CRC_ERROR,ret = 0x%x\r\n",ret);
		return 1;
	}
		
	/* 转换温度数据 */
	tem = (((uint16_t)buff[0]<<8) | buff[1]);//温度数据拼接
	Temperature= (175.0*(double)tem/65535.0-45.0) ;	// T = -45 + 175 * tem / (2^16-1)
	
	/* 转换湿度数据 */
	hum = (((uint16_t)buff[3]<<8) | buff[4]);//湿度数据拼接
	Humidity= (100.0*(double)hum/65535.0);			// RH = hum*100 / (2^16-1)
	
	/* 过滤错误数据 */
	if((Temperature>=-20)&&(Temperature<=125)&&(Humidity>=0)&&(Humidity<=100))
	{
		*Tem_val = Temperature;
		*Hum_val = Humidity;
		return 0;
	}
	else
		return 1;
}





