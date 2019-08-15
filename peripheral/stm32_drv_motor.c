/****************************************************************************
* 版权信息：
* 系统名称：rt-thread-4.0.1 
* 文件名称：stm32_drv_motor.c
* 文件说明：motor驱动程序
* 作    者：YCG
* 版本信息：V1.0
* 设计日期：2019-05-26
* 修改记录：
* 日    期      版    本        修改人      修改摘要
  2019-05-26     V1.0            YCG       创建MOTOR驱动
****************************************************************************/
/***********************************************************
*注意：
*    1、本驱动未进行有效性测试，请谨慎使用！！
**********************************************************/
#include <rtdevice.h>
#include <rthw.h>
#include <stm32f4xx.h>
#include "stm32_drv_motor.h"

const static gpio_des _hard_desc[] =
{
		{GPIOF,11},
		{GPIOF,12},
		{GPIOF,13},
		{GPIOF,14},
};

/***************************************************************************
*                                分隔符                                    *
***************************************************************************/
#define  MOTOR_DBG

#ifdef  MOTOR_DBG
#define DBG_PRINT(fmt, ...)   rt_kprintf("MOTOR_DBG:" fmt, ##__VA_ARGS__)
#else
#define DBG_PRINT(fmt, ...)
#endif

/* 获得数组元素个数 */
#ifndef ARRAY_SIZE
#define ARRAY_SIZE(arr)     (sizeof(arr)/sizeof(arr[0]))
#endif

/* 获得PIN */
#define get_st_pin(gpio_pin) (0x01 << (gpio_pin&0xFF))

struct _motor_drv
{
    struct rt_device device;
	  const gpio_des * IN1;
	  const gpio_des * IN2;
	  const gpio_des * IN3;
	  const gpio_des * IN4;
	  rt_mutex_t mutex;
};

/*
****************************************************************************
*函 数 名: stm32_motor_init
*功能说明: 初始化驱动引脚
*形    参: 1. 电机设备句柄 2.打开标志
*返 回 值: 0 表示正常， 1 表示不正常
*其    他: 函数内部调用
****************************************************************************
*/
static rt_err_t stm32_motor_init(rt_device_t dev)
{
	  GPIO_InitTypeDef GPIO_InitStruct;
	  struct _motor_drv *drv = (struct _motor_drv *)dev;
	
		MOTOR_GPIO_CLK_ENABLE();
	  
	  GPIO_InitStruct.Pin = get_st_pin(drv->IN1->pin)| get_st_pin(drv->IN1->pin)|get_st_pin(drv->IN1->pin)|get_st_pin(drv->IN1->pin);
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	  HAL_GPIO_Init(GPIOF,&GPIO_InitStruct);
	
	  MOTOR_IN1_LOW();  //初始化低
	  MOTOR_IN2_LOW();
	  MOTOR_IN3_LOW();
	  MOTOR_IN4_LOW();
	  
//	  HAL_GPIO_WritePin(GPIOF,get_st_pin(drv->IN1->pin),GPIO_PIN_RESET);
//	  HAL_GPIO_WritePin(GPIOF,get_st_pin(drv->IN2->pin),GPIO_PIN_RESET);
//	  HAL_GPIO_WritePin(GPIOF,get_st_pin(drv->IN3->pin),GPIO_PIN_RESET);
//	  HAL_GPIO_WritePin(GPIOF,get_st_pin(drv->IN4->pin),GPIO_PIN_RESET);
    	  
	  return RT_EOK;
}
/*
****************************************************************************
*函 数 名: motor_gpio_def
*功能说明: 电机引脚重定义
*形    参: 1. 电机设备句柄 2.打开标志
*返 回 值: 0 表示正常， 1 表示不正常
*其    他: 函数内部调用
****************************************************************************
*/
static rt_err_t motor_gpio_def(struct _motor_drv *drv,unsigned char InputData)
{
    if(InputData&0x01)
    {
			 HAL_GPIO_WritePin(GPIOF,get_st_pin(drv->IN1->pin),GPIO_PIN_SET);
    }
    else
    {
			  HAL_GPIO_WritePin(GPIOF,get_st_pin(drv->IN1->pin),GPIO_PIN_RESET);
    }
    if(InputData&0x02)
    {
        HAL_GPIO_WritePin(GPIOF,get_st_pin(drv->IN2->pin),GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOF,get_st_pin(drv->IN2->pin),GPIO_PIN_RESET);
    }
    if(InputData&0x04)
    {
        HAL_GPIO_WritePin(GPIOF,get_st_pin(drv->IN3->pin),GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOF,get_st_pin(drv->IN3->pin),GPIO_PIN_RESET);
    }
    if(InputData&0x08)
    {
        HAL_GPIO_WritePin(GPIOF,get_st_pin(drv->IN4->pin),GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOF,get_st_pin(drv->IN4->pin),GPIO_PIN_RESET);
    }
		if(!InputData)
		{
			HAL_GPIO_WritePin(GPIOF,get_st_pin(drv->IN1->pin),GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOF,get_st_pin(drv->IN2->pin),GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOF,get_st_pin(drv->IN3->pin),GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOF,get_st_pin(drv->IN4->pin),GPIO_PIN_RESET);
		}
		
		DBG_PRINT("motor pin def success!!!\n");
		
		return RT_EOK;
}

/*
****************************************************************************
*函 数 名: stm32_motor_open
*功能说明: 打开驱动
*形    参: 1. 电机设备句柄 2.打开标志
*返 回 值: 0 表示正常， 1 表示不正常
*其    他: 函数内部调用
****************************************************************************
*/
static rt_err_t stm32_motor_open(rt_device_t dev,rt_uint16_t oflag)
{
		return RT_EOK;
}

/*
****************************************************************************
*函 数 名: stm32_motor_init
*功能说明: 初始化驱动引脚
*形    参: 1. 电机设备句柄 2.打开标志
*返 回 值: 0 表示正常， 1 表示不正常
*其    他: 函数内部调用
****************************************************************************
*/
static rt_err_t stm32_motor_write(rt_device_t dev,rt_off_t pos,const void *buffer,rt_size_t size)
{
	  int i = 0, j = 0;
	  rt_uint8_t phase_buff[4];
	  struct _motor_drv *drv = (struct _motor_drv *)dev;
	
	  rt_memcpy(phase_buff,buffer+1,size);
	  
		if(phase_buff[0] == 1)
    {
				for(i = 0;i<500;i++)
				{
						for(j = 0;j<8;j++)
						{
							 motor_gpio_def(drv,phase_buff[i]);
							 rt_thread_mdelay(1000);
						}
				}
		}
	
	  return RT_EOK;
}

/*
****************************************************************************
*函 数 名: stm32_motor_init
*功能说明: 初始化驱动引脚
*形    参: 1. 电机设备句柄 2.打开标志
*返 回 值: 0 表示正常， 1 表示不正常
*其    他: 函数内部调用
****************************************************************************
*/
static rt_err_t stm32_motor_control(rt_device_t dev,int cmd, void *arg)
{
		return RT_EOK;
}



int motor_drv_init(void)
{
	  int i = 0,count = 0;
		struct _motor_drv *dev = NULL;
    
	  for(i = 0;i<ARRAY_SIZE(_hard_desc);i++)
	  {
				dev = (struct _motor_drv *)rt_malloc(sizeof(struct _motor_drv));
				if(dev == RT_NULL)
				{
					 DBG_PRINT("malloc device Memory fail!!\r\n");
					 continue;
				}
				rt_memset(dev,RT_NULL,sizeof(struct _motor_drv));    //清空这片内存
				
				dev->mutex =  rt_mutex_create("motor_lock",RT_IPC_FLAG_FIFO); //创建互斥信号量
				if(RT_NULL == dev->mutex)
				{
						DBG_PRINT("mutex create fail\r\n");
						rt_free(dev);
						continue;
				}
				
				dev->IN1 = &(_hard_desc[i]);
				dev->IN2 = &(_hard_desc[i+1]);	
				dev->IN3 = &(_hard_desc[i+2]);	
				dev->IN4 = &(_hard_desc[i+3]);			

        dev->device.init    = stm32_motor_init;
				dev->device.open    = stm32_motor_open;
				dev->device.read    = RT_NULL;
				dev->device.write   = stm32_motor_write;
				dev->device.control = stm32_motor_control;
				
				dev->device.user_data = RT_NULL;
				
				rt_device_register(&(dev->device),"motor",RT_DEVICE_FLAG_WRONLY);

        count++;
				
    }
				return count;	
}

INIT_DEVICE_EXPORT(motor_drv_init);   //驱动自动调用



