#ifndef __STM32_DRV_DHT11_H__
#define __STM32_DRV_DHT11_H__

#include <board.h>


/****************************************************************************
*自定义温度数据类型
****************************************************************************/
typedef struct dht11_data
{
    float humidity;
    float temperature;
}HT;


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
 
    
#define DHT11_Dout_GPIO_CLK_ENABLE()              __HAL_RCC_GPIOC_CLK_ENABLE()
#define DHT11_Dout_PORT                           GPIOC
#define DHT11_Dout_PIN                            GPIO_PIN_7

/***********************   DS18B20 函数宏定义  ****************************/
#define DHT11_Dout_LOW()                          HAL_GPIO_WritePin(DHT11_Dout_PORT,DHT11_Dout_PIN,GPIO_PIN_RESET) 
#define DHT11_Dout_HIGH()                         HAL_GPIO_WritePin(DHT11_Dout_PORT,DHT11_Dout_PIN,GPIO_PIN_SET)
#define DHT11_Data_IN()                           HAL_GPIO_ReadPin(DHT11_Dout_PORT,DHT11_Dout_PIN)
    
/****************************************************************************
*函数列表
****************************************************************************/
int dht11_drv_init(void);   /* 注册驱动 */

#endif

