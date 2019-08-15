/***********************************************
1��һ��ѭ���߳�
2��һ�����ڶ�ʱ��
3����ʱ���ص�������̷߳��ʼ�
4���߳�����ȴ������ʼ����յ��ʼ��󣬶�һ��IO

***********************************************/
#include <rtthread.h>
#include <stdio.h>
#include <stdint.h>
#include <rtdevice.h>


#define EVENT_W_PIN      (0x01<<0)
#define DS18B20_PIN      (0x01<<1)
#define DHT11_PIN        (0x01<<2)
#define EVENT_UNKNOW     (0x00)

static rt_thread_t tid;
static rt_mq_t hcsr_mq;
static rt_timer_t hcs_timer;
static rt_uint8_t HCSR501_data1;
static rt_sem_t uartdata_sem = RT_NULL;

float ds18b20_buff[1];
float dht11_buff[2];
rt_uint16_t ds_buffer[1] = {0};
rt_uint16_t dht_buffer1[1] = {0};
rt_uint16_t dht_buffer2[1] = {0};
                                                       

/*********************************************
*��������HCSR501_timer_callback
*���ܣ�HCSR501�ص�����
*��ע��
**********************************************/
static void HCSR501_timer_callback(void *parameter)   //�ص����������ļ�̣���֪ͨ������
{
    rt_uint8_t HCSR501_data = EVENT_W_PIN;
    static int _tick = 0;
    
    if(_tick%2 == 0)                   
    {
        HCSR501_data |=EVENT_UNKNOW;
    } 
    if(_tick%4 == 0)                   
    {
        HCSR501_data |= EVENT_W_PIN;
    }
    if(_tick%6 == 0)
    {
        HCSR501_data |= DS18B20_PIN;
    }
    if(_tick%8 == 0)
    {
        HCSR501_data |= DHT11_PIN;
    }

    if(_tick ++ > 10000)
    {
        _tick = 0;
    }
     if(rt_mq_send(hcsr_mq, &HCSR501_data, sizeof(HCSR501_data)) != RT_EOK)
     {
         rt_kprintf("F:%s L:%d err!  message quene1 full!\n,",__FUNCTION__,__LINE__);
         return;
     }
}

/*************************************************
*��������HCSR501_thread_entry
*���ܣ�
*��ע��
*************************************************/
static void HCSR501_thread_entry(void *parameter)
{
    rt_device_t pin_dev;
    rt_device_t ds18b20_dev;
    rt_device_t dht11_dev;

    
    pin_dev = rt_device_find("pin");
    if(pin_dev)
    {
        if(rt_device_open(pin_dev,RT_DEVICE_OFLAG_RDONLY) == RT_EOK)   //��ֻ����ʽ��  IO
        {
            struct rt_device_pin_mode pin_mode;
            
            pin_mode.pin = 4;
            pin_mode.mode = PIN_MODE_INPUT_PULLUP;
            if(rt_device_control(pin_dev,0,&pin_mode) == RT_EOK)
            {
                rt_kprintf("Hong_wai pin open success!!\n");
            }
        }
    }
    
    ds18b20_dev = rt_device_find("18b20_0");
    if(ds18b20_dev)
    {
        if(rt_device_open(ds18b20_dev,RT_DEVICE_OFLAG_RDONLY) == RT_EOK)   //��ֻ����ʽ��   DS18B20
        {
            struct rt_device_pin_mode ds18b20_pin_mode;
            
            ds18b20_pin_mode.pin = 124;
            ds18b20_pin_mode.mode = PIN_MODE_INPUT_PULLUP;
            if(rt_device_control(ds18b20_dev,0,&ds18b20_pin_mode) != RT_EOK)
            {
                rt_kprintf("ds18b20 pin open fail!!\n");
            }
        }
    }
    
    dht11_dev = rt_device_find("dht11_0");                                //DHT11
    if(dht11_dev)
    {
        if(rt_device_open(dht11_dev,RT_DEVICE_OFLAG_RDONLY) == RT_EOK)
        {
            struct rt_device_pin_mode dht11_pin_mode;
            
            dht11_pin_mode.pin = 97;
            dht11_pin_mode.mode = PIN_MODE_INPUT_PULLDOWN;
            if(rt_device_control(dht11_dev,0,&dht11_pin_mode) != RT_EOK)
            {
                rt_kprintf("dht11 pin open fail!!\n");
            }
        }
    }
    
    
    
    while(1)
    {
         if(rt_mq_recv(hcsr_mq,&HCSR501_data1,sizeof(HCSR501_data1),RT_WAITING_FOREVER) == RT_EOK)
         {
             if(HCSR501_data1 & EVENT_W_PIN)
             {
                 if(pin_dev)
                 {
                     struct rt_device_pin_status pin_status;
                     
                     pin_status.pin = 4;
                     pin_status.status = 1;
                     if(rt_device_read(pin_dev,0,&pin_status,sizeof(pin_status)) != sizeof(pin_status))
                     {
                         rt_kprintf("pin read fail!!!!\n");
                     }
                     else
                     {
                         rt_kprintf("pin status is :%s \n",pin_status.status ? "H" : "L");
                     }
                 }
             }
             if(HCSR501_data1 & DS18B20_PIN)
             {
                 if(ds18b20_dev)
                 {
                     rt_device_read(ds18b20_dev,0,&ds18b20_buff[0],sizeof(ds18b20_buff[0]));    //��ds18b20����
									   ds_buffer[0] = (rt_uint16_t)ds18b20_buff[0];
									   rt_kprintf("this is %d \n",ds_buffer[0]);
                 }
             }
             if(HCSR501_data1 & DHT11_PIN)
             {
                 if(dht11_dev)
                 {
                     rt_device_read(dht11_dev,0,dht11_buff,sizeof(dht11_buff));
									   dht_buffer1[0] = (rt_uint16_t)dht11_buff[0];
									   dht_buffer2[0] = (rt_uint16_t)dht11_buff[1];
                 }
             }
         }
    }
}



int HCSR501_part_init(void)
{
	hcsr_mq = rt_mq_create("all_mq",1024,5,RT_IPC_FLAG_FIFO);
	if(hcsr_mq == RT_NULL)
	{
			rt_kprintf("F:%s L:%d err! mq create fail!\n,",__FUNCTION__,__LINE__);
			return -1;
	}

	hcs_timer = rt_timer_create("HCSR501_timer",HCSR501_timer_callback,RT_NULL,500,RT_TIMER_FLAG_PERIODIC);
	if(hcs_timer == RT_NULL)
	{
			rt_kprintf("F:%s L:%d err! hcs_timer create fail!\n,",__FUNCTION__,__LINE__);
			rt_mq_delete(hcsr_mq);
			return -1;
	}
	
	 tid = rt_thread_create("tid",HCSR501_thread_entry,RT_NULL,512,22,20);
	 if(tid == RT_NULL)
	 {
			rt_mq_delete(hcsr_mq);
			rt_timer_delete(hcs_timer);
			rt_kprintf("F:%s L:%d err! tid create fail!\n\n,",__FUNCTION__,__LINE__);
			return -1;
	 }
		 
	 
	 rt_thread_startup(tid);
	 rt_timer_start(hcs_timer);
	 return 0;
}

//INIT_APP_EXPORT(HCSR501_part_init);
/*�ܽ�һ�㲽�裺
  1�����Ȱ���Ҫ���̡߳���ʱ������Ϣ���д�����
  2���������ʧ�ܣ���ɾ�������ͷ���Դ
*/
