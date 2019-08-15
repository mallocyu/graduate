#ifndef __STM32_DRV_NRF24L01_H__
#define __STM32_DRV_NRF24L01_H__

#include "board.h"

/****************************************************************************
*结构体
****************************************************************************/
#ifndef RT_USING_GPIO_DESC
    typedef struct _gpio_desc
    {
        GPIO_TypeDef *gpio;
        uint32_t pin;
    }gpio_desc;
#endif

		
#define NRF_GPIO_CLK_ENABLE()                  __HAL_RCC_GPIOG_CLK_ENABLE()
#define NRF_CE_PORT                            GPIOG
#define NRF_CE_PIN                             GPIO_PIN_6
//#define NRF_CSN_PORT                           GPIOG
//#define NRF_CSN_PIN                            GPIO_PIN_7
#define NRF_IRQ_PORT                           GPIOG
#define NRF_IRQ_PIN                            GPIO_PIN_8

/***********************   DS18B20 函数宏定义  ****************************/
#define NRF_CE_LOW()                          HAL_GPIO_WritePin(NRF_CE_PORT,NRF_CE_PIN,GPIO_PIN_RESET) 
#define NRF_CE_HIGH()                         HAL_GPIO_WritePin(NRF_CE_PORT,NRF_CE_PIN,GPIO_PIN_SET)
		
//#define NRF_CS_LOW()                          HAL_GPIO_WritePin(NRF_CSN_PORT,NRF_CSN_PIN,GPIO_PIN_RESET) 
//#define NRF_CS_HIGH()                         HAL_GPIO_WritePin(NRF_CSN_PORT,NRF_CSN_PIN,GPIO_PIN_SET)

#define NRF_IRQ_INPUT()                        HAL_GPIO_ReadPin(NRF_IRQ_PORT,NRF_IRQ_PIN)                  //IRQ
    
    

#endif

