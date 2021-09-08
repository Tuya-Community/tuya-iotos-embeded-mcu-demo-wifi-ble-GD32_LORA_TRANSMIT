#include "usart.h"
#include "stdio.h" 

void USART0_Init(void)
{    
    rcu_periph_clock_enable(RCU_GPIOA);				//ʹ��GPIOAʱ��
    rcu_periph_clock_enable(RCU_USART0);			//ʹ��USART0ʱ��
    
		//Usart0 TX GPIOA_9
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_9);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_10MHZ, GPIO_PIN_9);
		gpio_af_set(GPIOA, GPIO_AF_1, GPIO_PIN_9);	
		//Usart0 RX GPIOA_10
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_10);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_10MHZ, GPIO_PIN_10);
		gpio_af_set(GPIOA, GPIO_AF_1, GPIO_PIN_10);	
	
    usart_deinit(USART0);																			//��ʼ��USART0
    usart_baudrate_set(USART0, 115200U);											  //������9600
    usart_word_length_set(USART0, USART_WL_8BIT);							//8������λ
    usart_stop_bit_set(USART0, USART_STB_1BIT);								//1��ֹͣλ
    usart_parity_config(USART0, USART_PM_NONE);								//����żУ��
    usart_hardware_flow_rts_config(USART0, USART_RTS_DISABLE);//RTS���عر�
    usart_hardware_flow_cts_config(USART0, USART_CTS_DISABLE);//CTS���عر�
		usart_transmit_config(USART0, USART_TRANSMIT_ENABLE);			//ʹ��USART0����
    usart_receive_config(USART0, USART_RECEIVE_ENABLE);				//ʹ��USART0����	
		
		usart_interrupt_enable(USART0,USART_INT_RBNE);						//ʹ���ж�
		nvic_irq_enable(USART0_IRQn,2U);													//ʹ���ж����ȼ�
		usart_enable(USART0);																			//ʹ��USART0
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
    if (RESET != usart_interrupt_flag_get(USART0, USART_INT_FLAG_RBNE))   //�����ж�
    {
       USART0_RECEIVE_Buf[i++] = (uint8_t)usart_data_receive(USART0);
    }
		else if(RESET != usart_interrupt_flag_get(USART0, USART_INT_FLAG_TBE))//�����ж�
    {
			 usart_interrupt_flag_clear(USART0,USART_INT_FLAG_IDLE);    
    }
}

void USART1_Init(void)
{    
    rcu_periph_clock_enable(RCU_GPIOA);				//ʹ��GPIOAʱ��
    rcu_periph_clock_enable(RCU_USART1);			//ʹ��USART0ʱ��
    //USART1 TX GPIOA_2 
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_2);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_10MHZ, GPIO_PIN_2);
		gpio_af_set(GPIOA, GPIO_AF_1, GPIO_PIN_2);	
		//USART1 RX GPIOA_3 
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_3);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_10MHZ, GPIO_PIN_3);
		gpio_af_set(GPIOA, GPIO_AF_1, GPIO_PIN_3);	
	
    usart_deinit(USART1);																			//USART1
    usart_baudrate_set(USART1, 9600U);										  	//������9600
    usart_word_length_set(USART1, USART_WL_8BIT);							//8λ����λ
    usart_stop_bit_set(USART1, USART_STB_1BIT);								//1λֹͣλ
    usart_parity_config(USART1, USART_PM_NONE);								//����żУ��λ
    usart_hardware_flow_rts_config(USART1, USART_RTS_DISABLE);//RTS���عر�
    usart_hardware_flow_cts_config(USART1, USART_CTS_DISABLE);//CTS���عر�
		usart_transmit_config(USART1, USART_TRANSMIT_ENABLE);			//ʹ��USART1����
    usart_receive_config(USART1, USART_RECEIVE_ENABLE);				//ʹ��USART1����		
 
		usart_interrupt_enable(USART1,USART_INT_RBNE);						//ʹ��USART1�ж�
		nvic_irq_enable(USART1_IRQn,2U);													//ʹ��USART1�ж����ȼ�
		usart_enable(USART1);																			//ʹ��USART1
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
		if (RESET != usart_interrupt_flag_get(USART1, USART_INT_FLAG_RBNE))		 	//�����ж�
    {
       USART1_RECEIVE_Buf[i++] = (uint8_t)usart_data_receive(USART1);
    }
		else if(RESET != usart_interrupt_flag_get(USART1, USART_INT_FLAG_TBE))  //�����ж�
    {
			 usart_interrupt_flag_clear(USART1,USART_INT_FLAG_IDLE);  
    }
}

//*******************************************************************************
//�������ƣ�fputc(int ch, FILE *f)
//��    �ܣ�����ʵ��Printf()��������
//��    ������
//����ֵ  ��
//********************************************************************************/
/* retarget the C library printf function to the USART */
int fputc(int ch, FILE *f)
{
	usart_data_transmit(USART0, (uint8_t)ch);
	while(RESET == usart_flag_get(USART0, USART_FLAG_TBE));

	return ch;
}
