#ifndef __STM32_DRV_BLUX_H_
#define __STM32_DRV_BLUX_H_

#include "board.h"

typedef struct _gpio_des
{
	GPIO_TypeDef *gpio;
	rt_uint32_t pin;
}gpio_des;

//#define BLUX_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOD_CLK_ENABLE()

////IO��������
//#define SDA_IN()  {GPIOB->MODER&=~(3<<(9*2));GPIOB->MODER|=0<<9*2;}	//PB9����ģʽ
//#define SDA_OUT() {GPIOB->MODER&=~(3<<(9*2));GPIOB->MODER|=1<<9*2;} //PB9���ģʽ

//#define IIC_SCL    PBout(8) //SCL

#endif

