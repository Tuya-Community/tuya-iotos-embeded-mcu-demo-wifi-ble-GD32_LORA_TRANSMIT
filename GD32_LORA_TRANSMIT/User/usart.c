#include "usart.h"
#include "stdio.h" 

void USART0_Init(void)
{    
    rcu_periph_clock_enable(RCU_GPIOA);				//使能GPIOA时钟
    rcu_periph_clock_enable(RCU_USART0);			//使能USART0时钟
    
		//Usart0 TX GPIOA_9
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_9);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_10MHZ, GPIO_PIN_9);
		gpio_af_set(GPIOA, GPIO_AF_1, GPIO_PIN_9);	
		//Usart0 RX GPIOA_10
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_10);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_10MHZ, GPIO_PIN_10);
		gpio_af_set(GPIOA, GPIO_AF_1, GPIO_PIN_10);	
	
    usart_deinit(USART0);																			//初始化USART0
    usart_baudrate_set(USART0, 115200U);											  //波特率9600
    usart_word_length_set(USART0, USART_WL_8BIT);							//8个数据位
    usart_stop_bit_set(USART0, USART_STB_1BIT);								//1个停止位
    usart_parity_config(USART0, USART_PM_NONE);								//无奇偶校验
    usart_hardware_flow_rts_config(USART0, USART_RTS_DISABLE);//RTS流控关闭
    usart_hardware_flow_cts_config(USART0, USART_CTS_DISABLE);//CTS流控关闭
		usart_transmit_config(USART0, USART_TRANSMIT_ENABLE);			//使能USART0发送
    usart_receive_config(USART0, USART_RECEIVE_ENABLE);				//使能USART0接收	
		
		usart_interrupt_enable(USART0,USART_INT_RBNE);						//使能中断
		nvic_irq_enable(USART0_IRQn,2U);													//使能中断优先级
		usart_enable(USART0);																			//使能USART0
}
 
void USART0_Send_Byte(uint8_t Send_Byte)
{
    usart_data_transmit(USART0, Send_Byte);
    while ( usart_flag_get(USART0, USART_FLAG_TBE)== RESET) ;
}

void USART0_Send_Str(uint8_t *Str)
{
    while((*Str)!='\0')
    {
      USART0_Send_Byte(*Str);
      Str++;
    }
}

void USART0_IRQHandler(void)
{
	  uint8_t i;
    if (RESET != usart_interrupt_flag_get(USART0, USART_INT_FLAG_RBNE))   //接收中断
    {
       USART0_RECEIVE_Buf[i++] = (uint8_t)usart_data_receive(USART0);
    }
		else if(RESET != usart_interrupt_flag_get(USART0, USART_INT_FLAG_TBE))//发送中断
    {
			 usart_interrupt_flag_clear(USART0,USART_INT_FLAG_IDLE);    
    }
}

void USART1_Init(void)
{    
    rcu_periph_clock_enable(RCU_GPIOA);				//使能GPIOA时钟
    rcu_periph_clock_enable(RCU_USART1);			//使能USART0时钟
    //USART1 TX GPIOA_2 
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_2);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_10MHZ, GPIO_PIN_2);
		gpio_af_set(GPIOA, GPIO_AF_1, GPIO_PIN_2);	
		//USART1 RX GPIOA_3 
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_3);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_10MHZ, GPIO_PIN_3);
		gpio_af_set(GPIOA, GPIO_AF_1, GPIO_PIN_3);	
	
    usart_deinit(USART1);																			//USART1
    usart_baudrate_set(USART1, 9600U);										  	//波特率9600
    usart_word_length_set(USART1, USART_WL_8BIT);							//8位数据位
    usart_stop_bit_set(USART1, USART_STB_1BIT);								//1位停止位
    usart_parity_config(USART1, USART_PM_NONE);								//无奇偶校验位
    usart_hardware_flow_rts_config(USART1, USART_RTS_DISABLE);//RTS流控关闭
    usart_hardware_flow_cts_config(USART1, USART_CTS_DISABLE);//CTS流控关闭
		usart_transmit_config(USART1, USART_TRANSMIT_ENABLE);			//使能USART1发送
    usart_receive_config(USART1, USART_RECEIVE_ENABLE);				//使能USART1接收		
 
		usart_interrupt_enable(USART1,USART_INT_RBNE);						//使能USART1中断
		nvic_irq_enable(USART1_IRQn,2U);													//使能USART1中断优先级
		usart_enable(USART1);																			//使能USART1
}
 
void USART1_Send_Byte(uint8_t Send_Byte)
{
    usart_data_transmit(USART1, Send_Byte);
    while ( usart_flag_get(USART1, USART_FLAG_TBE)== RESET) ;
}

void USART1_Send_Str(uint8_t *Str)
{
    while((*Str)!='\0')
    {
      USART1_Send_Byte(*Str);
      Str++;
    }
}

void USART1_IRQHandler(void)
{
	  uint8_t i;
		if (RESET != usart_interrupt_flag_get(USART1, USART_INT_FLAG_RBNE))		 	//接收中断
    {
       USART1_RECEIVE_Buf[i++] = (uint8_t)usart_data_receive(USART1);
    }
		else if(RESET != usart_interrupt_flag_get(USART1, USART_INT_FLAG_TBE))  //发送中断
    {
			 usart_interrupt_flag_clear(USART1,USART_INT_FLAG_IDLE);  
    }
}

//*******************************************************************************
//函数名称：fputc(int ch, FILE *f)
//功    能：串口实现Printf()函数功能
//参    数：无
//返回值  ：
//********************************************************************************/
/* retarget the C library printf function to the USART */
int fputc(int ch, FILE *f)
{
	usart_data_transmit(USART0, (uint8_t)ch);
	while(RESET == usart_flag_get(USART0, USART_FLAG_TBE));

	return ch;
}
