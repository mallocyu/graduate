/****************************************************************************
* ��Ȩ��Ϣ��
* ϵͳ���ƣ�rt-thread-2.1.0 
* �ļ����ƣ�stm32_drv_dht11.c
* �ļ�˵����dht11��ʪ����������
* ��    �ߣ�TYX
* �汾��Ϣ��V1.0
* ������ڣ�2017-03-26
* �޸ļ�¼��
* ��    ��      ��    ��        �޸���      �޸�ժҪ
  2017-03-26     V1.0            TYX       ����DHT11��������
  2017-04-21     V2.0            TYX       �޸Ķ�ȡIO��ƽ��ʽ��ʹ�ÿ�©�����ʽ��ȡ
****************************************************************************/
/****************************************************************************
*1.δ��ӵ�����Ϣ
****************************************************************************/

#include <rtdevice.h>
#include <rthw.h>
#include <stm32f4xx.h>
#include "stm32_drv_dht11.h"

/****************************************************************************
* �û���֪
* 1.����:   
*   1>�Զ������֣����������е������ظ�
*   2>���ж��������Ҫ���� %d ���б�ţ��������֧��99���豸
* 2.������Ϣ��
*   1>һ��������ֻ�ܽ�һ������
*   2>�����������Ӷ���������Ϣ
* 3.����ע�ắ�����Զ����ã������ֶ�����
****************************************************************************/
/* �������� */
#define HARD_DEVICE_NAME_FORMAT        "dht11_%d"
/* ������Ϣ */
const static gpio_desc _hard_desc[] =
{
    {GPIOC, 7},
};


/***************************************************************************
*                                �ָ���                                    *
***************************************************************************/
#define  DHT11_DBG


#ifdef  DHT11_DBG
#define DBG_PRINT(fmt, ...)   rt_kprintf("DHT11_DBG:" fmt, ##__VA_ARGS__)
#else
#define DBG_PRINT(fmt, ...)
#endif


//#if 0 /* �⺯����ʽ */
//	#define PIN_OUT_LOW(PORT_DQ,PIN_DQ)     GPIO_ResetBits(PORT_DQ, PIN_DQ)
//	#define PIN_OUT_HIGH(PORT_DQ,PIN_DQ)    GPIO_SetBits(PORT_DQ, PIN_DQ)
//	/* ��ȡ���ŵ�ƽ */
//	#define PIN_INPUT(PORT_DQ,PIN_DQ)       GP IO_ReadInputDataBit(PORT_DQ, PIN_DQ)
//#else	/* ֱ�Ӳ����Ĵ���������ٶ� */
//	#define PIN_OUT_LOW(PORT_DQ,PIN_DQ)     PORT_DQ->BSRR |= (PIN_DQ<<16)
//	#define PIN_OUT_HIGH(PORT_DQ,PIN_DQ)    PORT_DQ->BSRR |= (PIN_DQ)
//	/* ��ȡ���ŵ�ƽ */
//	#define PIN_INPUT(PORT_DQ,PIN_DQ)	   (PORT_DQ->IDR & PIN_DQ)
//#endif


/* �������Ԫ�ظ��� */
#ifndef ARRAY_SIZE
#define ARRAY_SIZE(arr)     (sizeof(arr)/sizeof(arr[0]))
#endif
/* ���PIN */
#define get_st_pin(gpio_pin) (0x01 << (gpio_pin&0xFF))

/* DHT11�����ṹ�� */
struct _dht11_drv
{
    struct rt_device device;        /* ͨ�������ṹ�� */
    const  gpio_desc * hard_desc;   /* ������Ϣ */
    rt_mutex_t mutex;               /* �����������̷߳��� */
};


static void rt_delay_us(int us)
{
	rt_uint32_t ticks;
	rt_uint32_t told,tnow,tcnt=0;
	rt_uint32_t reload=SysTick->LOAD;
	
    /* �����ʱ������tick�� */
    ticks = us * (reload / (1000000 / RT_TICK_PER_SECOND));
	
    /* ��һ�ζ�ʱ����ֵ */
    told = SysTick->VAL;
    /* ѭ����õ�ǰʱ�䣬ֱ���ﵽָ����ʱ����˳�ѭ�� */
	while(1)
	{
		tnow = SysTick->VAL;
		if(tnow != told)
		{
			if(tnow < told) tcnt += told - tnow;
			else tcnt += reload - tnow + told;
			told = tnow;
			if(tcnt >= ticks) break;
		}
	}
}

/*
****************************************************************************
*�� �� ��: _dht11_reset
*����˵��: ʹDHT11������λ
*��    ��: 1. DHT11�豸���
*�� �� ֵ: 0 ��ʾ������ 1 ��ʾ������
*��    ��: �����ڲ�����
****************************************************************************
*/
static rt_err_t _dht11_reset(struct _dht11_drv * drv)
{
    uint8_t retry = 0;
    
    DHT11_Dout_LOW();    //����������ʼ�ź�, ����ʱ�� > 18ms
    rt_thread_delay(20);//����20ms
    DHT11_Dout_HIGH();   //�������߲��ͷ�����
    rt_delay_us(30);
    
    DHT11_Data_IN();
    while(DHT11_Data_IN() && retry<100)  //�ȴ����߱�����
	{
		retry++;
		rt_delay_us(1);
	}
    
	if(retry>=100)             //��ʱ���ش���
        return RT_ERROR;

    while((!DHT11_Data_IN())&&retry<100)   //�ȴ����߱�����
	{
		retry++;
		rt_delay_us(1);
	}
    
	if(retry>=100)           //��ʱ���ش���
        return RT_ERROR;
    
	return RT_EOK;
}

/*
****************************************************************************
*�� �� ��: _dht11_read_byte
*����˵��: ��DHT11�����ж�ȡһ���ֽ�����
*��    ��: 1. DHT11�豸��� 2. ���ջ������׵�ַ
*�� �� ֵ: 0 ��ʾ������ 1 ��ʾ������
*��    ��: �����ڲ�����
****************************************************************************
*/
static rt_err_t _dht11_read_byte(struct _dht11_drv * drv, uint8_t * data)
{
    uint8_t bit = 0,result = 0;
    uint8_t i = 0, retry = 0;
    
    for(i = 0;i<8;i++)
    {
        while(DHT11_Data_IN() && retry<100)//�ȴ���Ϊ�͵�ƽ
        {
            rt_delay_us(1);
            retry++;
        } 
        if(retry>=100)          //��ʱ����
            return RT_ERROR;
        retry = 0;
        while((!DHT11_Data_IN())&&retry<100)//�ȴ���ߵ�ƽ
        {
            rt_delay_us(1);
            retry++;
        }
        if(retry>=100)          //��ʱ����
            return RT_ERROR;
        
        rt_delay_us(40);//�ȴ�40us
        if(DHT11_Data_IN()) bit=1;   //�������϶�ȡһλ����
        else bit=0;
        
        result = bit|(result<<1);
    }
    
    *data = result;
    
    return RT_EOK;
}

/*
****************************************************************************
*�� �� ��: _dht11_read_data
*����˵��: ��DHT11�����ж�ȡ����
*��    ��: 1. DHT11�豸��� 2. ���ջ������׵�ַ
*�� �� ֵ: 0 ��ʾ������ 1 ��ʾ������
*��    ��: �����ڲ�����
****************************************************************************
*/
static rt_err_t _dht11_read_data(struct _dht11_drv * _drv, HT * result)
{
    uint8_t i = 0;
    uint8_t temp_buf[5] = {0};
    
    if(_dht11_reset(_drv)==RT_EOK)              //��λ�ɹ�,��ʼ��ȡ����
    {
        for(i = 0;i<5;i++)
        {
            rt_enter_critical();
            _dht11_read_byte(_drv,&temp_buf[i]); //��ȡһ�ֽ�����
            rt_exit_critical();
        }
        if((temp_buf[0]+temp_buf[1]+temp_buf[2]+temp_buf[3])==temp_buf[4])  //����У���
		{
			result->humidity    = temp_buf[0];     //ֻȡ����ʪ����������
			result->temperature = temp_buf[2];     //ֻȡ�����¶���������
            
            return RT_EOK;
		}    
    }

    return RT_ERROR;        //��λʧ�ܣ����ش���
}

/*
****************************************************************************
*�� �� ��: stm32_dht11_init
*����˵��: ������ʼ��
*��    ��: 1. dht11�豸���
*�� �� ֵ: 0 ��ʾ������ 1 ��ʾ������
*��    ��: �����ڲ�����,��ʼ����Ӧ��IO����
****************************************************************************
*/
static rt_err_t stm32_dht11_init(rt_device_t dev)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    
    struct _dht11_drv * dht11 = (struct _dht11_drv *)dev;
    
    DHT11_Dout_GPIO_CLK_ENABLE();	      //ʹ��ʱ��
    
    GPIO_InitStruct.Pin = get_st_pin(dht11->hard_desc->pin);          //ѡ������
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;                       //��Ϊ��©���ģʽ
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;                //IO������ٶ�
    HAL_GPIO_Init(dht11->hard_desc->gpio, &GPIO_InitStruct);

	HAL_GPIO_WritePin(dht11->hard_desc->gpio, get_st_pin(dht11->hard_desc->pin), GPIO_PIN_SET);
    
    return RT_EOK;
}

/*
****************************************************************************
*�� �� ��: stm32_dht11_open
*����˵��: ������
*��    ��: 1. dht11�豸��� 2.�򿪱�־
*�� �� ֵ: 0 ��ʾ������ 1 ��ʾ������
*��    ��: �����ڲ�����
****************************************************************************
*/
static rt_err_t stm32_dht11_open(rt_device_t dev, rt_uint16_t oflag)
{   
    struct _dht11_drv * dht11 = (struct _dht11_drv *)dev;
    rt_err_t err = RT_EOK;
    
    rt_mutex_take(dht11->mutex,RT_WAITING_FOREVER);     //��ȡ�����ź���������
    err = _dht11_reset(dht11);
    rt_mutex_release(dht11->mutex);
    return err;
}

/*
****************************************************************************
*�� �� ��: stm32_dht11_close
*����˵��: �ر�����
*��    ��: 1. dht11�豸���
*�� �� ֵ: 0 ��ʾ������ 1 ��ʾ������
*��    ��: �����ڲ�����
****************************************************************************
*/
static rt_err_t stm32_dht11_close(rt_device_t dev)
{
    return RT_EOK;
}

/*
****************************************************************************
*�� �� ��: stm32_dht11_control
*����˵��: �����豸
*��    ��: 1. dht11�豸��� 2. ��������  3. ����
*�� �� ֵ: 0 ��ʾ������ 1 ��ʾ������
*��    ��: �����ڲ�����
****************************************************************************
*/
static rt_err_t stm32_dht11_control(rt_device_t dev, rt_uint8_t cmd, void *args)
{

    return RT_EOK;
}

/*
****************************************************************************
*�� �� ��: stm32_dht11_read
*����˵��: ����������
*��    ��: 1. dht11�豸��� 2. ƫ���� 3. ��������ַ 4. ��������С
*�� �� ֵ: 0 ��ʾ������ 1 ��ʾ������
*��    ��: �����ڲ�����
****************************************************************************
*/
static rt_size_t stm32_dht11_read(rt_device_t dev, rt_off_t pos, void* buffer, rt_size_t size)
{
    HT temp;
    int count = 0,i = 0;
    HT * data = (HT *)buffer;
    struct _dht11_drv * dht11 = (struct _dht11_drv *)dev;
    
    while(i<(size/sizeof(HT)))     //����������ݸ���
    {
        rt_mutex_take(dht11->mutex,RT_WAITING_FOREVER);   //���󻥳��ź���������
        if(_dht11_read_data(dht11,&temp) == RT_EOK)       //��ȡ���ݵ���ʱ��������
        {
            data[i].temperature   = temp.temperature;     //��ֵ��������
            data[i].humidity      = temp.humidity;
            count++;
        }
        rt_mutex_release(dht11->mutex);                   //�ͷŻ����ź���,����
        i++;
    }

    return count;
}

/*
****************************************************************************
*�� �� ��: dht11_drv_init
*����˵��: DHT11����ע��
*��    ��: ��
*�� �� ֵ: 0 ��ʾ������ 1 ��ʾ������
*��    ��: ��INIT_DEVICE_EXPORT���Զ�����
****************************************************************************
*/
int dht11_drv_init(void)
{
    int i = 0, count = 0;
    char dev_name[sizeof(HARD_DEVICE_NAME_FORMAT) + 3] = "";
    struct _dht11_drv * dev = RT_NULL;
    
    for(i=0; i<ARRAY_SIZE(_hard_desc);i++)
    {
        rt_sprintf(dev_name, HARD_DEVICE_NAME_FORMAT, i);  //���������������
        if(rt_device_find(dev_name))           //���ֱ�ռ��
        {
            DBG_PRINT("name repetition\r\n");
            continue;
        }
        
        dev = (struct _dht11_drv *)rt_malloc(sizeof(struct _dht11_drv));      //�����ڴ�
        if (RT_NULL == dev)            //����ʧ�ܣ���������ѭ��
        {
            continue;
        }
        rt_memset(dev,RT_NULL,sizeof(struct _dht11_drv));     //�����Ƭ�ڴ�
        
        dev->mutex =  rt_mutex_create(dev_name,RT_IPC_FLAG_FIFO); //�����ź���
        if(RT_NULL == dev->mutex)
        {
            rt_free(dev);
            continue;
        }
        
        dev->hard_desc        = &(_hard_desc[i]);
        dev->device.type      = RT_Device_Class_Miscellaneous;
        dev->device.init      = stm32_dht11_init;
        dev->device.open      = stm32_dht11_open;
        dev->device.close     = stm32_dht11_close;
        dev->device.read      = stm32_dht11_read;
        dev->device.write     = RT_NULL;
        dev->device.control   = stm32_dht11_control;
        dev->device.user_data = RT_NULL;
        rt_device_register(&(dev->device),dev_name,RT_DEVICE_FLAG_RDONLY);

        count++;
        
    }
    
    return count;
}


INIT_DEVICE_EXPORT(dht11_drv_init);  //�����

