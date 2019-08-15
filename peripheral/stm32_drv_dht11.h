#ifndef __STM32_DRV_DHT11_H__
#define __STM32_DRV_DHT11_H__

#include <board.h>


/****************************************************************************
*�Զ����¶���������
****************************************************************************/
typedef struct dht11_data
{
    float humidity;
    float temperature;
}HT;


/****************************************************************************
*�ṹ��
****************************************************************************/
#ifndef RT_USING_GPIO_DESC
    typedef struct _gpio_desc
    {
        GPIO_TypeDef *gpio;
        uint32_t pin;
    }gpio_desc;
#endif
 
    
#define DHT11_Dout_GPIO_CLK_ENABLE()              __HAL_RCC_GPIOC_CLK_ENABLE()
#define DHT11_Dout_PORT                           GPIOC
#define DHT11_Dout_PIN                            GPIO_PIN_7

/***********************   DS18B20 �����궨��  ****************************/
#define DHT11_Dout_LOW()                          HAL_GPIO_WritePin(DHT11_Dout_PORT,DHT11_Dout_PIN,GPIO_PIN_RESET) 
#define DHT11_Dout_HIGH()                         HAL_GPIO_WritePin(DHT11_Dout_PORT,DHT11_Dout_PIN,GPIO_PIN_SET)
#define DHT11_Data_IN()                           HAL_GPIO_ReadPin(DHT11_Dout_PORT,DHT11_Dout_PIN)
    
/****************************************************************************
*�����б�
****************************************************************************/
int dht11_drv_init(void);   /* ע������ */

#endif

