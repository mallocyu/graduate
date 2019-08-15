/****************************************************************************
* ��Ȩ��Ϣ��
* ϵͳ���ƣ�rt-thread-4.0.1 
* �ļ����ƣ�stm32_drv_motor.c
* �ļ�˵����motor��������
* ��    �ߣ�YCG
* �汾��Ϣ��V1.0
* ������ڣ�2019-05-26
* �޸ļ�¼��
* ��    ��      ��    ��        �޸���      �޸�ժҪ
  2019-05-26     V1.0            YCG       ����MOTOR����
****************************************************************************/
/***********************************************************
*ע�⣺
*    1��������δ������Ч�Բ��ԣ������ʹ�ã���
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
*                                �ָ���                                    *
***************************************************************************/
#define  MOTOR_DBG

#ifdef  MOTOR_DBG
#define DBG_PRINT(fmt, ...)   rt_kprintf("MOTOR_DBG:" fmt, ##__VA_ARGS__)
#else
#define DBG_PRINT(fmt, ...)
#endif

/* �������Ԫ�ظ��� */
#ifndef ARRAY_SIZE
#define ARRAY_SIZE(arr)     (sizeof(arr)/sizeof(arr[0]))
#endif

/* ���PIN */
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
*�� �� ��: stm32_motor_init
*����˵��: ��ʼ����������
*��    ��: 1. ����豸��� 2.�򿪱�־
*�� �� ֵ: 0 ��ʾ������ 1 ��ʾ������
*��    ��: �����ڲ�����
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
	
	  MOTOR_IN1_LOW();  //��ʼ����
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
*�� �� ��: motor_gpio_def
*����˵��: ��������ض���
*��    ��: 1. ����豸��� 2.�򿪱�־
*�� �� ֵ: 0 ��ʾ������ 1 ��ʾ������
*��    ��: �����ڲ�����
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
*�� �� ��: stm32_motor_open
*����˵��: ������
*��    ��: 1. ����豸��� 2.�򿪱�־
*�� �� ֵ: 0 ��ʾ������ 1 ��ʾ������
*��    ��: �����ڲ�����
****************************************************************************
*/
static rt_err_t stm32_motor_open(rt_device_t dev,rt_uint16_t oflag)
{
		return RT_EOK;
}

/*
****************************************************************************
*�� �� ��: stm32_motor_init
*����˵��: ��ʼ����������
*��    ��: 1. ����豸��� 2.�򿪱�־
*�� �� ֵ: 0 ��ʾ������ 1 ��ʾ������
*��    ��: �����ڲ�����
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
*�� �� ��: stm32_motor_init
*����˵��: ��ʼ����������
*��    ��: 1. ����豸��� 2.�򿪱�־
*�� �� ֵ: 0 ��ʾ������ 1 ��ʾ������
*��    ��: �����ڲ�����
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
				rt_memset(dev,RT_NULL,sizeof(struct _motor_drv));    //�����Ƭ�ڴ�
				
				dev->mutex =  rt_mutex_create("motor_lock",RT_IPC_FLAG_FIFO); //���������ź���
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

INIT_DEVICE_EXPORT(motor_drv_init);   //�����Զ�����



