#ifndef __STM32_DRV_MOTOR_H_
#define __STM32_DRV_MOTOR_H_

#include "board.h"

/****************************************************************************
*结构体
****************************************************************************/
typedef struct _gpio_des
{
	GPIO_TypeDef *gpio;
	rt_uint32_t pin;
}gpio_des;

#define MOTOR_GPIO_CLK_ENABLE()              __HAL_RCC_GPIOF_CLK_ENABLE()
#define MOTOR_Dout_PORT                      GPIOF
#define MOTOR_IN1_PIN                        GPIO_PIN_11
#define MOTOR_IN2_PIN                        GPIO_PIN_12
#define MOTOR_IN3_PIN                        GPIO_PIN_13
#define MOTOR_IN4_PIN                        GPIO_PIN_14

/***********************   DS18B20 函数宏定义  ****************************/
#define MOTOR_IN1_LOW()                          HAL_GPIO_WritePin(MOTOR_Dout_PORT,MOTOR_IN1_PIN,GPIO_PIN_RESET)
#define MOTOR_IN1_HIGH()                         HAL_GPIO_WritePin(MOTOR_Dout_PORT,MOTOR_IN1_PIN,GPIO_PIN_SET)

#define MOTOR_IN2_LOW()                          HAL_GPIO_WritePin(MOTOR_Dout_PORT,MOTOR_IN2_PIN,GPIO_PIN_RESET)
#define MOTOR_IN2_HIGH()                         HAL_GPIO_WritePin(MOTOR_Dout_PORT,MOTOR_IN2_PIN,GPIO_PIN_SET)

#define MOTOR_IN3_LOW()                          HAL_GPIO_WritePin(MOTOR_Dout_PORT,MOTOR_IN3_PIN,GPIO_PIN_RESET)
#define MOTOR_IN3_HIGH()                         HAL_GPIO_WritePin(MOTOR_Dout_PORT,MOTOR_IN3_PIN,GPIO_PIN_SET)

#define MOTOR_IN4_LOW()                          HAL_GPIO_WritePin(MOTOR_Dout_PORT,MOTOR_IN4_PIN,GPIO_PIN_RESET)
#define MOTOR_IN4_HIGH()                         HAL_GPIO_WritePin(MOTOR_Dout_PORT,MOTOR_IN4_PIN,GPIO_PIN_SET)

/****************************************************************************
*函数列表
****************************************************************************/
int ds18b20_drv_init(void);   /* 注册驱动 */

#endif
