/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
#include "gd32e23x_gpio.h"

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define SW_CTL1_Pin GPIO_PIN_5
#define SW_CTL1_GPIO_Port GPIOB
#define SW_CTL2_Pin GPIO_PIN_4
#define SW_CTL2_GPIO_Port GPIOB
#define NRESET_Pin GPIO_PIN_0
#define NRESET_GPIO_Port GPIOB
#define SPI_CS_Pin GPIO_PIN_1
#define SPI_CS_GPIO_Port GPIOA
#define ANT_SWITCH_POWER_Pin GPIO_PIN_0
#define ANT_SWITCH_POWER_GPIO_Port GPIOA
#define BUSY_Pin GPIO_PIN_1
#define BUSY_GPIO_Port GPIOB
#define DIO1_Pin GPIO_PIN_2
#define DIO1_GPIO_Port GPIOB
#define DIO1_EXTI_IRQn EXTI2_IRQn
#define LED_Pin   GPIO_PIN_9
#define LED_GPIO_Port  GPIOB


/* USER CODE BEGIN Private defines */
void SX126xOnDio1Irq(void);
void OnTxDone(void);
void OnRxDone( void);
void GenTrig(void);


/* USER CODE END Private defines */

//void _Error_Handler(char *, int);

//#define Error_Handler() _Error_Handler(__FILE__, __LINE__)

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
