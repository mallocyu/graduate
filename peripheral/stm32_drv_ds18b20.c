/****************************************************************************
* ��Ȩ��Ϣ��
* ϵͳ���ƣ�rt-thread-2.1.0 
* �ļ����ƣ�stm32_drv_ds18b20.c
* �ļ�˵����18b20�¶���������
* ��    �ߣ�TYX
* �汾��Ϣ��V1.0
* ������ڣ�2017-03-25
* �޸ļ�¼��
* ��    ��      ��    ��        �޸���      �޸�ժҪ
  2017-03-25     V1.0            TYX       ����18B20����
  2017-04-21     V2.0            TYX       �޸Ķ�ȡIO��ƽ��ʽ��ʹ�ÿ�©�����ʽ��ȡ
  2017-04-22     V2.1            TYX       ��ӵ�����Ϣ
****************************************************************************/
/****************************************************************************
*1.��֧���޸�����
****************************************************************************/

#include <rtdevice.h>
#include <rthw.h>
// #include "board.h"
#include <stm32f4xx.h>
#include "stm32_drv_ds18b20.h"

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
#define HARD_DEVICE_NAME_FORMAT        "18b20_%d"
/* ������Ϣ */
const static gpio_des _hard_desc[] =
{
    {GPIOG, 9},
};

/***************************************************************************
*                                �ָ���                                    *
***************************************************************************/
#define  DS18B20_DBG

#ifdef  DS18B20_DBG
#define DBG_PRINT(fmt, ...)   rt_kprintf("DS18B20_DBG:" fmt, ##__VA_ARGS__)
#else
#define DBG_PRINT(fmt, ...)
#endif

/* �������Ԫ�ظ��� */
#ifndef ARRAY_SIZE
#define ARRAY_SIZE(arr)     (sizeof(arr)/sizeof(arr[0]))
#endif

/* ���PIN */
#define get_st_pin(gpio_pin) (0x01 << (gpio_pin&0xFF))

/* DS18B20�����ṹ�� */
struct _ds18b20_drv
{
    struct rt_device  device;      /* ͨ�������ṹ�� */
    const gpio_des * hard_desc;   /* ������Ϣ */
    rt_mutex_t mutex;              /* �����������̷߳��� */
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
*�� �� ��: _ds18b20_reset
*����˵��: ʹDS18B20������λ
*��    ��: 1. 18b20�豸���
*�� �� ֵ: 0 ��ʾ������ 1 ��ʾ������
*��    ��: �����ڲ�����
****************************************************************************
*/
static rt_err_t _ds18b20_reset(struct _ds18b20_drv * drv)
{
    uint8_t retry = 0;

//    DS18B20_IO_OUT();
    DS18B20_Dout_LOW();   //������������650us
    rt_delay_us(750);   //����650us
    DS18B20_Dout_HIGH(); //�����ͷ�����
    rt_delay_us(15);   //�ȴ�10us
    
    DS18B20_Data_IN();
    while(DS18B20_Data_IN() && retry<200)  //�ȴ����߱��ӻ�����
	{
		retry++;
		rt_delay_us(1);  
	}
    
	if(retry >= 200)       //��ʱ���˳�����ʼ��ʧ��
    {
        DBG_PRINT("TimeOut :DS18B20 not detected\r\n");
        return RT_ERROR;
    }

    while(!DS18B20_Data_IN() && retry<240)  //�ȴ��ӻ��ͷ�����
	{
		retry++;
		rt_delay_us(1);
	}
    
	if(retry >= 240)     //��ʱ���˳�����ʼ��ʧ��
    {
        DBG_PRINT("TimeOut :DS18B20 not release bus\r\n");
        return RT_ERROR;
    }
        
	return RT_EOK;
}

/*
****************************************************************************
*�� �� ��: _ds18b20_read_byte
*����˵��: ��18B20���ȡһ���ֽ�
*��    ��: 1. 18b20�豸��� 2. ��������ַ
*�� �� ֵ: 0 ��ʾ������ 1 ��ʾ������
*��    ��: �����ڲ�����
****************************************************************************
*/
static rt_err_t _ds18b20_read_byte(struct _ds18b20_drv * drv, uint8_t * data)
{
    uint8_t i = 0;
    uint8_t bit = 0,result = 0;
    
    for(i = 0;i < 8;i++)
    {
        DS18B20_Dout_LOW();     //������������
        rt_delay_us(1);
        DS18B20_Dout_HIGH();    //�����ͷ�����
        rt_delay_us(12); 
        if(DS18B20_Data_IN()) bit=1;  //��ȡ���ߵ�ƽ
        else bit=0;	        
        result=(bit<<7) | (result>>1);
        
        rt_delay_us(50);
    }
    *data = result;
    
    return RT_EOK;
}


/*
****************************************************************************
*�� �� ��: _ds18b20_write_byte
*����˵��: ��18B20��дһ���ֽ�
*��    ��: 1. 18b20�豸��� 2. д�������׵�ַ
*�� �� ֵ: 0 ��ʾ������ 1 ��ʾ������
*��    ��: �����ڲ�����
****************************************************************************
*/
static uint8_t _ds18b20_write_byte(struct _ds18b20_drv * drv, uint8_t data)
{
    uint8_t i, testb;
    
    for (i=0;i<8;i++) 
	{
        testb=data&0x01;
        data=data>>1;
        if (testb)
        {
            DS18B20_Dout_LOW(); //������������,׼����������д����
            rt_delay_us(2);
            DS18B20_Dout_HIGH();     //����д��һ��λ
            rt_delay_us(60);
        }
        else
        {
            DS18B20_Dout_LOW(); //������������,׼����������д����
            rt_delay_us(60);                                       //����д��һ��λ
            DS18B20_Dout_HIGH();
            rt_delay_us(2);
        }
    }
   
    return RT_EOK;
}


/*
****************************************************************************
*�� �� ��: _ds18b20_start
*����˵��: ds18b20����һ���¶�ת��
*��    ��: 1. 18b20�豸���
*�� �� ֵ: 0 ��ʾ������ 1 ��ʾ������
*��    ��: �����ڲ�����
****************************************************************************
*/
static rt_err_t _ds18b20_start(struct _ds18b20_drv * drv)
{
    if(_ds18b20_reset(drv) == RT_EOK)
    {
        rt_enter_critical();                   //������ʱ��
        _ds18b20_write_byte(drv,0xcc);	       //������ ������ ROM
        _ds18b20_write_byte(drv,0x44);	       //������ ��ʼת��
        rt_exit_critical();
        
        return RT_EOK;
    }
    
    return RT_ERROR;
}


/*
****************************************************************************
*�� �� ��: _ds18b20_read_data
*����˵��: ��ds18b20�ж�ȡ�¶�����
*��    ��: 1. 18b20�豸��� 2.�������׵�ַ
*�� �� ֵ: 0 ��ʾ������ 1 ��ʾ������
*��    ��: �����ڲ�����
****************************************************************************
*/
static rt_err_t _ds18b20_read_data(struct _ds18b20_drv * drv, float * result)
{
    uint8_t  data_H, data_L;
    short data;
    
    _ds18b20_reset(drv);                 //��λ��׼������ͨ��
    rt_enter_critical();                 //�����ٽ���
    _ds18b20_write_byte(drv,0xcc);	     //������ ������ ROM
    _ds18b20_write_byte(drv,0xbe);	     //������ ��ʼ�����¶�ԭʼ����
    _ds18b20_read_byte(drv, &data_L); 	 //�¶ȵ�8λ  
    _ds18b20_read_byte(drv, &data_H); 	 //�¶ȸ�8λ
    rt_exit_critical();                  //�˳��ٽ���
    data = data_H;
    data <<= 8;
    data |=  data_L;
    if(data&0xF800)
        *result=(~data+1)*0.0625*10;     
    else
        *result=data*0.0625;
    
    if(*result>-55 && *result<125)
        return RT_EOK;
    else
    {
        DBG_PRINT("read_data error\r\n");
        return RT_ERROR;
    }
}


/*
****************************************************************************
*�� �� ��: stm32_ds18b20_init
*����˵��: ������ʼ��
*��    ��: 1. 18b20�豸���
*�� �� ֵ: 0 ��ʾ������ 1 ��ʾ������
*��    ��: �����ڲ�����,��ʼ����Ӧ��IO����
****************************************************************************
*/
static rt_err_t stm32_ds18b20_init(rt_device_t dev)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    struct _ds18b20_drv * ds18b20 = (struct _ds18b20_drv *)dev;

    /* GPIO Periph clock enable */
	  DS18B20_Dout_GPIO_CLK_ENABLE();
    /* Configure GPIO_InitStructure */
    GPIO_InitStruct.Pin = get_st_pin(ds18b20->hard_desc->pin);
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    /* output setting: od. */
//    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    HAL_GPIO_Init(ds18b20->hard_desc->gpio, &GPIO_InitStruct);

	HAL_GPIO_WritePin(ds18b20->hard_desc->gpio, get_st_pin(ds18b20->hard_desc->pin), GPIO_PIN_SET);

    return RT_EOK;
}

/*
****************************************************************************
*�� �� ��: stm32_ds18b20_open
*����˵��: ������
*��    ��: 1. 18b20�豸��� 2.�򿪱�־
*�� �� ֵ: 0 ��ʾ������ 1 ��ʾ������
*��    ��: �����ڲ�����
****************************************************************************
*/
static rt_err_t stm32_ds18b20_open(rt_device_t dev, rt_uint16_t oflag)
{   
    struct _ds18b20_drv * ds18b20 = (struct _ds18b20_drv *)dev;
    rt_err_t err = RT_EOK;
    
    rt_mutex_take(ds18b20->mutex,RT_WAITING_FOREVER);     //��ȡ�����ź���������
    err = _ds18b20_reset(ds18b20);
    rt_mutex_release(ds18b20->mutex);
    
    return err;
}

/*
****************************************************************************
*�� �� ��: stm32_ds18b20_close
*����˵��: �ر�����
*��    ��: 1. 18b20�豸���
*�� �� ֵ: 0 ��ʾ������ 1 ��ʾ������
*��    ��: �����ڲ�����
****************************************************************************
*/
static rt_err_t stm32_ds18b20_close(rt_device_t dev)
{
    return RT_EOK;
}

/*
****************************************************************************
*�� �� ��: stm32_ds18b20_read
*����˵��: ����������
*��    ��: 1. 18b20�豸��� 2. ƫ���� 3. ��������ַ 4. ��������С
*�� �� ֵ: 0 ��ʾ������ 1 ��ʾ������
*��    ��: �����ڲ�����
****************************************************************************
*/
static rt_size_t stm32_ds18b20_read(rt_device_t dev, rt_off_t pos, void* buffer, rt_size_t size)
{
    float temp;
    int count = 0,i = 0;
    struct _ds18b20_drv * ds18b20 = (struct _ds18b20_drv *)dev;
	float *result = buffer;

    while (i < (size / sizeof(float)))
    {
        rt_mutex_take(ds18b20->mutex,RT_WAITING_FOREVER);     //��ȡ�����ź���������
        if(_ds18b20_start(ds18b20) == RT_EOK)                 //��ʼһ��ת��
        {
            rt_thread_delay(800);                             //����ȴ�һ��ʱ��
            if(_ds18b20_read_data(ds18b20, &temp) == RT_EOK)  //��ȡ����
            {
                result[i] = temp;
                count++;
            }
            else
            {
                result[i] = 85;
            }
        }
        rt_mutex_release(ds18b20->mutex);
        i++;
    }

    return count;
}

/*
****************************************************************************
*�� �� ��: stm32_ds18b20_control
*����˵��: �����豸
*��    ��: 1. 18b20�豸��� 2. ��������  3. ����
*�� �� ֵ: 0 ��ʾ������ 1 ��ʾ������
*��    ��: �����ڲ�����,��������Ӹı侫������,ĿǰΪ��
****************************************************************************
*/
static rt_err_t stm32_ds18b20_control(rt_device_t dev, int cmd, void *args)
{

    return RT_EOK;
}

/*
****************************************************************************
*�� �� ��: ds18b20_drv_init
*����˵��: DS18B20����ע��
*��    ��: ��
*�� �� ֵ: 0 ��ʾ������ 1 ��ʾ������
*��    ��: ��INIT_DEVICE_EXPORT���Զ�����
****************************************************************************
*/
int ds18b20_drv_init(void)
{
    int i = 0, count = 0;
    char dev_name[sizeof(HARD_DEVICE_NAME_FORMAT) + 3] = "";
    struct _ds18b20_drv *dev = RT_NULL;

    for(i=0; i<ARRAY_SIZE(_hard_desc);i++)          //�������飬���DS18B20����
    {
        rt_sprintf(dev_name, HARD_DEVICE_NAME_FORMAT, i);  //���������������
        if(rt_device_find(dev_name))           //��ǰ�������ֱ�ʹ��
        {
            DBG_PRINT("name repetition\r\n");
            continue;
        }

        dev = (struct _ds18b20_drv *)rt_malloc(sizeof(struct _ds18b20_drv));  //�����ڴ�
        if (RT_NULL == dev)
        {
            DBG_PRINT("malloc device Memory fail\r\n");
            continue;
        }
        rt_memset(dev,RT_NULL,sizeof(struct _ds18b20_drv));    //�����Ƭ�ڴ�
        
        dev->mutex =  rt_mutex_create(dev_name,RT_IPC_FLAG_FIFO); //���������ź���
        if(RT_NULL == dev->mutex)
        {
            DBG_PRINT("mutex create fail\r\n");
            rt_free(dev);
            continue;
        }

        dev->hard_desc        = &(_hard_desc[i]);
        dev->device.type      = RT_Device_Class_Miscellaneous;
        dev->device.init      = stm32_ds18b20_init;
        dev->device.open      = stm32_ds18b20_open;
        dev->device.close     = stm32_ds18b20_close;
        dev->device.read      = stm32_ds18b20_read;
        dev->device.write     = RT_NULL;
        dev->device.control   = stm32_ds18b20_control;
        dev->device.user_data = RT_NULL;
        
        rt_device_register(&(dev->device),dev_name,RT_DEVICE_FLAG_RDONLY);   //������ע�ᵽϵͳ,���ϲ�Ӧ�ó������
        
        count++;     //�ɹ�������1
    }
    
    return count;
}


INIT_DEVICE_EXPORT(ds18b20_drv_init);   //�����Զ�����


