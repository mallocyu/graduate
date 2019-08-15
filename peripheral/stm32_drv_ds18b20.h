#ifndef __STM32_DRV_DS18B20_H_
#define __STM32_DRV_DS18B20_H_

#include "board.h"

/****************************************************************************
*结构体
****************************************************************************/
typedef struct _gpio_des
{
	GPIO_TypeDef *gpio;
	rt_uint32_t pin;
}gpio_des;

#define DS18B20_Dout_GPIO_CLK_ENABLE()              __HAL_RCC_GPIOG_CLK_ENABLE()
#define DS18B20_Dout_PORT                           GPIOG
#define DS18B20_Dout_PIN                            GPIO_PIN_9

/***********************   DS18B20 函数宏定义  ****************************/
#define DS18B20_Dout_LOW()                          HAL_GPIO_WritePin(DS18B20_Dout_PORT,DS18B20_Dout_PIN,GPIO_PIN_RESET) 
#define DS18B20_Dout_HIGH()                         HAL_GPIO_WritePin(DS18B20_Dout_PORT,DS18B20_Dout_PIN,GPIO_PIN_SET)
#define DS18B20_Data_IN()                           HAL_GPIO_ReadPin(DS18B20_Dout_PORT,DS18B20_Dout_PIN)

/****************************************************************************
*函数列表
****************************************************************************/
int ds18b20_drv_init(void);   /* 注册驱动 */


#endif


