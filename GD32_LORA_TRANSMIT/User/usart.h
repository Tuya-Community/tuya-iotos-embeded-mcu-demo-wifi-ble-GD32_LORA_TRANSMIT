#ifndef USART_H
#define USART_H

#include "gd32e23x.h"

extern uint32_t USART0_RECEIVE_Buf[100];
extern uint32_t USART1_RECEIVE_Buf[100];

void USART0_Init();
void USART0_Send_Byte(uint8_t Send_Byte);
void USART0_Send_Str(uint8_t *Str);

void USART1_Init();
void USART1_Send_Byte(uint8_t Send_Byte);
void USART1_Send_Str(uint8_t *Str);


#endif