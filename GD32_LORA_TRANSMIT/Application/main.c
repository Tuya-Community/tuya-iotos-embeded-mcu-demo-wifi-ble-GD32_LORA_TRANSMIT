/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "gd32e23x.h"
#include "delay.h"
#include "sx126x_v01.h"
#include <time.h>
#include <stdlib.h>
#include <string.h> 
#include <usart.h>
#include "BH1750.h"
#include "sht3x.h"
#include "SPI.h"

uint32_t USART0_RECEIVE_Buf[100]; 
uint32_t USART1_RECEIVE_Buf[100]; 

#define LORA_MODE	1
#define FSK_MODE  0


#define TRANSMITTER 1
#define RECEIVER   0

#define RX_CONTINOUS    1   //连续接收


#if (TRANSMITTER == RECEIVER)
    #error "Please define only Transmitter or receiver."
#endif

#define TEST_MODE	3  	//0-infinite preamble TX mode（暂时只是做了lora）
					          	//1-continous CW TX 
						          //其他值才能进入到收发分离模式
						


#if (LORA_MODE == FSK_MODE)
    #error "Please define only LoRa or FSK."
#endif

#define TX_OUTPUT_POWER                             22        // dBm  //测出来是18.536
#define RF_FREQUENCY                                490000000//480000000//915000000//470000000 // Hz


#if (FSK_MODE==1)

#define FSK_FDEV                                    10000//38400//25e3      // Hz 
#define FSK_DATARATE                                19200//40e3      // bps
#define FSK_BANDWIDTH                               93800//93800////58600  140e3     // Hz >> DSB in sx126x
#define FSK_AFC_BANDWIDTH                           100000     // Hz
#define FSK_PREAMBLE_LENGTH                         5         // Same for Tx and Rx
#define FSK_FIX_LENGTH_PAYLOAD_ON                   true
#define FSK_FIX_LENGTH_PAYLOAD 						          10
#define FSK_CRC										true
 
#elif (LORA_MODE==1)

#define LORA_BANDWIDTH                              1         // [0: 125 kHz,
																															//	1: 250 kHz, 														 
																															//	2: 500k
																															//	3 :20.83kHz
																															//	 4:31.25kHz
																															//	5:62.5kHz4
																															//6:41.67
#define LORA_SPREADING_FACTOR                       10        // [SF7..SF12]
#define LORA_CODINGRATE                             1         // [1: 4/5,
                                                              //  2: 4/6,
                                                              //  3: 4/7,
                                                              //  4: 4/8]
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx  SF5&6 will automatilly change to 12
#define LORA_SYMBOL_TIMEOUT                         0         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false

#endif


#define HAL_MAX_DELAY      0xFFFFFFFFU

#define RX_TIMEOUT_VALUE                            1000
#define TX_TIMEOUT                                  65535 
#define BUFFER_SIZE                                 250//10//250 // Define the payload size here

#define CADTIMEOUT_MS								2000                         //CAD timeout 时间  用ms表示

uint8_t dat;
uint8_t cnt=0x55;
uint8_t recdat=0;
uint8_t version=0;

uint16_t BufferSize = BUFFER_SIZE;
uint8_t Buffer[BUFFER_SIZE]={0};

int8_t RssiValue = 0;
int8_t SnrValue = 0;

PacketStatus_t RadioPktStatus;
uint8_t RadioRxPayload[255];
uint8_t RadioRxPacketSize;

uint8_t SendCnt=0;

volatile bool TXDone=false;
volatile bool RXDoneFlag=false;
volatile bool TimeOutFlag=false;
volatile bool CRCFail=false;
volatile int Cnt1=0;

const RadioLoRaBandwidths_t Bandwidths_copy[] = { LORA_BW_125, LORA_BW_250, LORA_BW_500,LORA_BW_020,LORA_BW_031,LORA_BW_062,LORA_BW_041 };

static void MX_GPIO_Init(void);

int main(void)
{
  uint8_t i=0;
  uint16_t  light;
	double Tem_val,Hum_val;
  bool DetectTruetable[100]={0};//CAD成功的分布
  bool RXTruetable[100]={0};//CAD后能接收正确的分布
  uint8_t CadDetectedTime=0;//检测到的cad的次数
  uint8_t RxCorrectTime=0;//RX 接收正确次数
  uint8_t TxTime=0;		//TX 次数
  int random_number=0;
  RadioStatus_t RadioStatus;
	//连续发送的时候用
	uint8_t ModulationParam[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  uint8_t PacketParam[9] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  /* Configure the system clock */
  systick_config(); 
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
	rcu_config();
	gpio_config();
	spi_config();
	USART0_Init();
  USART1_Init();
  Init_BH1750();
	SHT3x_reset();
	if( 0 == SHT3x_Init())
		printf("SHT3x_Init OK \r\n");
	else
		printf("SHT3x_Init ERR \r\n");
	
		
	
	
  gpio_bit_toggle(LED_GPIO_Port, LED_Pin);
	Delay_ms(250);
	
  SX126xReset();
  i=SX126xReadRegister(REG_LR_CRCSEEDBASEADDR);

  if(i==0x1D)
  {   
	printf("SPI SUCCESS!\n\r");
  }
  else
  {
	printf("SPI Fail! REG_LR_CRCSEEDBASEADDR=%x\n\r",i);
  }
  RadioInit();
  SX126xWriteRegister(0x889, SX126xReadRegister(0x889) & 0xfB);//SdCfg0 (0x889) sd_res (bit 2) = 0 
  printf("RadioInit Done!\n\r");


#if (TEST_MODE==0)   //infinite preamble TX mode
	//连续发送
	SX126xSetStandby( STDBY_RC );
	SX126xSetPacketType(PACKET_TYPE_LORA);//todo: 增加发射FSK模式下的改指令
	
	printf("set lora params\n");
	ModulationParam[0]=LORA_SPREADING_FACTOR;
	ModulationParam[1]=Bandwidths_copy[LORA_BANDWIDTH];
	ModulationParam[2]=LORA_CODINGRATE;
	ModulationParam[3]=0;//1:SF11 and SF12 0:其他 低速率优化  
	SX126xWriteCommand( RADIO_SET_MODULATIONPARAMS, ModulationParam, 4 );//lora发射参数配置


	//设置lora包参数
	PacketParam[0]=(LORA_PREAMBLE_LENGTH>>8)& 0xFF;
	PacketParam[1]=LORA_PREAMBLE_LENGTH;
	PacketParam[2]=LORA_FIX_LENGTH_PAYLOAD_ON;//head type
	PacketParam[3]=0xFF;//0Xff is MaxPayloadLength
	PacketParam[4]=true;//CRC on
	PacketParam[5]=LORA_IQ_INVERSION_ON;
	SX126xWriteCommand( RADIO_SET_PACKETPARAMS, PacketParam, 6 );

	//SX126xWriteBuffer( 0x00, SendData, 10 );

	//连续发送lora
	SX126xSetRfFrequency( RF_FREQUENCY );
    SX126xSetRfTxPower( TX_OUTPUT_POWER );
	SX126xSetTxInfinitePreamble();

	printf("TxContinuousWave Now--infinite preamble!\n\r");
	while(1);
#elif (TEST_MODE==1) //TX CW

	RadioSetTxContinuousWave( RF_FREQUENCY, TX_OUTPUT_POWER, TX_TIMEOUT );
	printf("TxContinuousWave Now---CW!\n\r");
	while(1);

#endif


#if (FSK_MODE==1)

	SX126xSetRfFrequency(RF_FREQUENCY);
	RadioSetTxConfig( MODEM_FSK, TX_OUTPUT_POWER, FSK_FDEV, FSK_BANDWIDTH,
						FSK_DATARATE, 0,
						FSK_PREAMBLE_LENGTH, FSK_FIX_LENGTH_PAYLOAD_ON,
						true, 0, 0, 0, 3000 );
	
	RadioSetRxConfig( MODEM_FSK, FSK_BANDWIDTH, FSK_DATARATE,
						0, FSK_AFC_BANDWIDTH, FSK_PREAMBLE_LENGTH,
						0, FSK_FIX_LENGTH_PAYLOAD_ON, FSK_FIX_LENGTH_PAYLOAD, FSK_CRC,0, 0,false, RX_CONTINOUS );
	
	printf("FSK:%d,Fdev=%ld,BitRate=%ld,BW=%ld,PWR=%d,PreLen=%d,PYLOAD=%d\n\r",RF_FREQUENCY,FSK_FDEV,FSK_DATARATE,FSK_BANDWIDTH,TX_OUTPUT_POWER,FSK_PREAMBLE_LENGTH,BUFFER_SIZE);
	printf("configure FSK parameters done\n!");

	

#elif (LORA_MODE==1)

			SX126xSetRfFrequency(RF_FREQUENCY);
			RadioSetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
									   LORA_SPREADING_FACTOR, LORA_CODINGRATE,
									   LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
									   true, 0, 0, LORA_IQ_INVERSION_ON, 3000 );


		  RadioSetRxConfig( MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
										 LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
										 LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
										 0, true, 0, 0, LORA_IQ_INVERSION_ON, RX_CONTINOUS );//最后一个参数设置是否是连续接收


		
		 printf("LORA:%d,SF=%d,codeRate=%d,BW=%d,PWR=%d,PreLen=%d,PYLOAD=%d\n\r",RF_FREQUENCY,LORA_SPREADING_FACTOR,LORA_CODINGRATE,LORA_BANDWIDTH,TX_OUTPUT_POWER,LORA_PREAMBLE_LENGTH,BUFFER_SIZE);
		 //{ true,false }--public,{ false,false }--private
		 if (RadioPublicNetwork.Previous==true && RadioPublicNetwork.Current==false)
		 printf("public\n\r");
		 else if (RadioPublicNetwork.Previous==false && RadioPublicNetwork.Current==false)
		 printf("private\n\r");
		 printf("configure LORA parameters done\n!");
		 //timeout=0代表单次接收，但是会先判断是否是连续接收，这个优先级最高
		//RadioRx(3000);//连续接收 只要在这里设置一次就可以  中间不能sleep 否则要重新执行RadioRx();		
#endif
	
  while (1)
  {
#if (TRANSMITTER==1)
	while(1)
	{ 
      light =read_BH1750();
			/* 采集温湿度数据 */
			if(SHT3x_Get_Humiture_periodic(&Tem_val,&Hum_val) == 0)
			{
				memcpy(Buffer,(double*)(&Tem_val),8);		
				memcpy(Buffer+8,(double*)(&Hum_val),8);
			}
			else
				printf("Get_Humiture ERR\r\n");
			
		memcpy(Buffer+16, (uint16_t*)&light, sizeof((uint16_t*)&light));
		RadioSend(Buffer ,18);
		printf("1=%d\n",read_BH1750());
			
		while(TXDone==false && TimeOutFlag==false);//一直等待tx done
		TXDone=false;
		TimeOutFlag=false;
		printf("TxTime=%d\n",TxTime);
		Delay_ms(1000); //1s

		//读取状态
		RadioStatus=SX126xGetStatus();
		printf("RadioStatus is(after TX_DONE) %d\n",(((RadioStatus.Value)>>4)&0x07));	
	}
#elif (RECEIVER==1) 
 while(1)
	{

#if (RX_CONTINOUS==1)
		//开始接收
		RadioRx(0xFFFFFF);//50MS(0XC80)超时  0-单次接收 无超时
		printf("continous RX...\n");
		while(1);//连续接收
#endif
		RadioRx(2000);//50MS(0XC80)超时  0-单次接收 无超时
		while(RXDoneFlag==false && TimeOutFlag==false && CRCFail==false);
		if(RXDoneFlag==true || TimeOutFlag==true || CRCFail==true)
		{
			if(CRCFail==false)	//CRC无错误
			{
				if(RXDoneFlag==true)
				{
					printf("\n%d:RxCorrect-PING\n",RxCorrectTime);
					RxCorrectTime++;
				}
			}
			
			CRCFail=false;
			RXDoneFlag=false;
			TimeOutFlag=false;
		}
		
	}
#endif		
 }
}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
	rcu_periph_clock_enable(RCU_GPIOA);
	rcu_periph_clock_enable(RCU_GPIOB);
  rcu_periph_clock_enable(RCU_CFGCMP);

  /*Configure GPIO pin Output Level */
	gpio_bit_reset(GPIOB,SW_CTL1_Pin|SW_CTL2_Pin|NRESET_Pin);
   
  /*Configure GPIO pin Output Level */
	gpio_bit_reset(GPIOA,SPI_CS_Pin|ANT_SWITCH_POWER_Pin);
	
  /*Configure GPIO pin Output Level */
  gpio_bit_set(LED_GPIO_Port,LED_Pin);

  /*Configure GPIO pins : SW_CTL1_Pin SW_CTL2_Pin TrigIO_Pin NRESET_Pin DeviceSel_Pin*/
  gpio_mode_set(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, SW_CTL1_Pin|SW_CTL2_Pin|NRESET_Pin);
  gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, SW_CTL1_Pin|SW_CTL2_Pin);
 
  /*Configure GPIO pins :  SPI_CS_Pin ANT_SWITCH_POWER_Pin */
  gpio_mode_set(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, SPI_CS_Pin|ANT_SWITCH_POWER_Pin);
  gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, SPI_CS_Pin|ANT_SWITCH_POWER_Pin);

  gpio_mode_set(GPIOB, GPIO_MODE_INPUT, GPIO_PUPD_NONE, BUSY_Pin);

	 /*Configure GPIO pin : DIO1_Pin */
	gpio_mode_set(DIO1_GPIO_Port, GPIO_MODE_INPUT, GPIO_PUPD_NONE, DIO1_Pin);

	/* enable and set key EXTI interrupt priority */
	nvic_irq_enable(EXTI2_3_IRQn,0U);
	 /* connect DIO1 EXTI line to DIO1 GPIO pin */
	syscfg_exti_line_config(EXTI_SOURCE_GPIOB, EXTI_SOURCE_PIN2);
	
	/* configure DIO1 EXTI line */
	exti_init(EXTI_2, EXTI_INTERRUPT, EXTI_TRIG_RISING);
	exti_interrupt_flag_clear(EXTI_2);
	


  /*Configure GPIO pin : LED_Pin */
  gpio_mode_set(LED_GPIO_Port, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED_Pin);
  gpio_output_options_set(LED_GPIO_Port, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, LED_Pin);


}

//DIO1的中断函数
void SX126xOnDio1Irq(void)
{
		 uint16_t irqRegs = SX126xGetIrqStatus( );
		 SX126xClearIrqStatus( IRQ_RADIO_ALL );//这里清掉中断标志
			 //发送结束
	 if( ( irqRegs & IRQ_TX_DONE ) == IRQ_TX_DONE )
				{
					 TXDone=true;
					 gpio_bit_toggle(LED_GPIO_Port, LED_Pin);
					 OnTxDone();
				}

		//在SX126xSetTx()设置了一个超时时间 可以检测改功能 --ok
		if( ( irqRegs & IRQ_RX_TX_TIMEOUT ) == IRQ_RX_TX_TIMEOUT )
				{
					 TimeOutFlag=true;
					 printf(" RX/TX timeout\n");
			
				}
		if( ( irqRegs & IRQ_RX_DONE ) == IRQ_RX_DONE )
		{
				SX126xGetPayload( RadioRxPayload, &RadioRxPacketSize , 255 );
				SX126xGetPacketStatus( &RadioPktStatus );
				gpio_bit_toggle(LED_GPIO_Port, LED_Pin);
				OnRxDone();
				RXDoneFlag=true;
		}

		 if( ( irqRegs & IRQ_CRC_ERROR ) == IRQ_CRC_ERROR )
			 {
					printf("CRC fail\n");
					CRCFail=true;
				 
			 }
		if( ( irqRegs & IRQ_CAD_DONE ) == IRQ_CAD_DONE )
				{
					if ( ( irqRegs & IRQ_CAD_ACTIVITY_DETECTED ) == IRQ_CAD_ACTIVITY_DETECTED ) 
					{
						//printf("IRQ_CAD_ACTIVITY_DETECTED\n");	
						//CadDetect=true;
					}      
				}
		if( ( irqRegs & IRQ_PREAMBLE_DETECTED ) == IRQ_PREAMBLE_DETECTED )
		{
				__NOP( );
		}

		if( ( irqRegs & IRQ_SYNCWORD_VALID ) == IRQ_SYNCWORD_VALID )
		{
				__NOP( );
		}

		if( ( irqRegs & IRQ_HEADER_VALID ) == IRQ_HEADER_VALID )
		{
				__NOP( );
		}
			
}

void OnTxDone(void)
{
	SleepParams_t params = { 0 };
  params.Fields.WarmStart = 1;//热启动
  //printf("params.value=%d\n",params.Value);
  //SX126xSetSleep( params );//热启动可以保存进入睡眠前的状态的相关寄存器  sleep可以清掉所有中断标志位
	printf("OnTxDone\n");
}
void OnRxDone()
{
	int i;
	printf("onRXDone\n");
  SleepParams_t params = { 0 };
  params.Fields.WarmStart = 1;//热启动

	//TODO:处理包的数据
	if (RadioPktStatus.packetType==PACKET_TYPE_LORA)
	printf("LoRa:PacketSIZE=%d,RSSI=%d,SNR=%d\n",
			RadioRxPacketSize,
					RadioPktStatus.Params.LoRa.RssiPkt, 
							RadioPktStatus.Params.LoRa.SnrPkt);
	else if (RadioPktStatus.packetType==PACKET_TYPE_GFSK)
	printf("FSK:PacketSIZE=%d,RssiAvg=%d,RssiSync=%d\n",
			RadioRxPacketSize,
					RadioPktStatus.Params.Gfsk.RssiAvg, 
							RadioPktStatus.Params.Gfsk.RssiSync);
	printf("Payload: ");
	for(i=0;i<RadioRxPacketSize;i++)
	{
		printf("0x%x ",RadioRxPayload[i]);
	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
