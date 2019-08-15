#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include "drv_spi.h"

#define SPI_BUS_NAME     "spi1"
#define SPI_DEVICE_NAME  "spi10"
#define ADC_NAME         "adc1"
#define ADC1_CHANNEL      3        //A3
#define ADC2_CHANNEL      4        //A4
#define ADC3_CHANNEL      5        //A5

#define CHANNEL   (0x28)
#define ADDR      (0X4A)

#define water_relay  GET_PIN(B,6)      //���ҹ��
#define light_relay  GET_PIN(B,7)      //���ҹ��
#define ven_relay    GET_PIN(B,8)      //���ҹ��

static rt_thread_t nrf_tid;
static rt_thread_t sensor;
static rt_thread_t deal;

static rt_device_t nrf;
static rt_device_t drv_ds,drv_dht;
static rt_adc_device_t  drv_adc;          //ʹ��ADC�豸���

typedef struct dht11_data
{
    float humidity;
    float temperature;
}HT;

//������ת��ת����
uint16_t phasecw[4] ={0x0200,0x0100,0x0080,0x0040};// D-C-B-A   
uint16_t phaseccw[4]={0x0040,0x0080,0x0100,0x0200};// A-B-C-D

//�ϱ����ݽṹ��
struct _data
{
    uint8_t addr;     //��ַ
    uint8_t channel;  //Ƶ��
    uint8_t sta;      //״̬
    uint8_t cmd;      //����
    
    uint16_t temp;    //�¶����� ��λ����  ��λС��
    uint16_t hum;     //ʪ������ ��λ����  ��λС��
    uint16_t ds_temp;   //�������� 
    uint16_t land_hum;     //ȼ������
    uint16_t light;   //��ǿ����
    uint16_t ul;      //����������
    uint16_t PM2_5;   //PM2.5����
    uint16_t COO;     //������̼����
    uint16_t TVOC;    //��ȩ����
    uint16_t noise;   //��������
    uint16_t IO_1;    //������1
    uint16_t IO_2;    //������2
    
    uint8_t data_1;   //��չ����1
    uint8_t data_2;   //��չ����1
    
};

struct _cmd
{
    uint8_t addr;      //��ַ
    uint8_t channel;   //Ƶ��
    uint8_t sta;       //״̬
    uint8_t cmd;       //����
    
    uint8_t data_1;    //����һ  �������Ⱥ�˳��
    uint8_t data_2;    //���ݶ�
    uint8_t data_3;    //������
    uint8_t data_4;    //������
    uint8_t data_5;    //������
    uint8_t data_6;    //������
		uint8_t data_7;    //������
    uint8_t data_8;    //������
    uint8_t data_9;    //������
};

/* �ӻ��ṹ�� */
struct _slave
{
    struct _data data;  //�ӻ��ϱ�����
    struct _cmd  cmd;   //�����·�����
    uint8_t link_sta;   //����״̬
    
};

/***********************************************
*�������ƣ�equip_status
*�������ܣ��̵������ÿ���
*��ע��
***********************************************/
void equip_status(rt_base_t pin,rt_uint8_t val)
{
	  rt_pin_mode(pin,PIN_MODE_OUTPUT);
		rt_pin_write(pin,val);
}

/******************************************************
*�������ƣ�nrf_thread_entry
*���ܣ�nrf���ݷ��ͽ����߳�
*��ע��
******************************************************/
void nrf_thread_entry(void *parameter)
{
	  rt_uint8_t result, i = 0;
	  uint32_t link_out = 0;
	  int link_sta = 0;
		char temp_buf[32];
	  struct _slave * slave = parameter;

		rt_hw_spi_device_attach(SPI_BUS_NAME,SPI_DEVICE_NAME,GPIOG,GPIO_PIN_7);   //bsp/stm32/�µ���������ô˺�������SPI����
	
	  nrf = rt_device_find("nrf_0");
	  if(nrf)

		rt_device_open(nrf,RT_DEVICE_FLAG_RDWR);
		
		while(1)
		{			
        if(rt_device_read(nrf,0,temp_buf,sizeof(temp_buf)))
				{
						if(link_out > 40)
            {
                link_sta = 0;
            }
              
            link_out++;   //���ӳ�ʱ����
            rt_thread_delay(50);
            continue;
				}
				
				if(temp_buf[0] != ADDR)  //�������������ַ
        {
            rt_thread_delay(100);
            continue;
        }
        
        if(temp_buf[1] != CHANNEL)  //�����������ͨ��
        {
            rt_thread_delay(100);
            continue;
        }
        
				rt_kprintf("%x\n",temp_buf[0]);
				
        rt_memcpy(&(slave->cmd),temp_buf,sizeof(struct _cmd));
        rt_thread_delay(10);
        /* �ظ��������� */
        if(rt_device_write(nrf,RT_NULL,&slave->data,sizeof(struct _data)))       //������Ҫ�޸ģ�����������������������������������������������������������������������
        {
            /* ����ʧ�ܣ�����û�н��յ����� */
        }

        link_out = 0;
        /* ����ͨ�� */
        slave->link_sta = 1;
        
        rt_thread_delay(200);
		}
}

void sensor_deal_entry(void *parameter)
{
    struct _slave * slave = parameter;
	  
	  /* �̵���1-- */
	  if(slave->cmd.data_1 == 1)  equip_status(water_relay,1);
	     else equip_status(water_relay,0);
		if(slave->cmd.data_2 == 1)  equip_status(light_relay,1);
	     else equip_status(light_relay,0);
		if(slave->cmd.data_3 == 1)  equip_status(ven_relay,1);
	     else equip_status(ven_relay,0);
	  
//		if(slave->cmd.data_4 == 1)  equip_status(water_relay,1);
//			 else equip_status(water_relay,0);
//		if(slave->cmd.data_5 == 1)  equip_status(water_relay,1);
//	     else equip_status(water_relay,0);
//		if(slave->cmd.data_6 == 1)  equip_status(water_relay,1);
//	     else equip_status(water_relay,0);
	
}



void sensor_collect_data(void *parameter)
{
	  int i = 0;
	  float ds_data = 0;
	  HT DHT;
	  struct _slave * slave = parameter;
		rt_uint32_t adc_temp_val1 = 0;
		rt_uint16_t temp_buff[1];
	
	  drv_ds = rt_device_find("18b20_0");
	  drv_dht = rt_device_find("dht11_0");
	
	  drv_adc = (rt_adc_device_t)rt_device_find(ADC_NAME);    //find adc
	/*******************************ADC�豸����*********************************/
	  if (drv_adc == RT_NULL)
    {
			  rt_kprintf("not find ADC!! F:%s , L:%d \n", __FUNCTION__,__LINE__);
        return;
    }
		rt_adc_enable(drv_adc,ADC1_CHANNEL);
	/***************************************************************************/
	  
	  rt_device_open(drv_ds,RT_DEVICE_FLAG_RDONLY);
		while(rt_device_open(drv_dht, RT_DEVICE_FLAG_RDONLY))
    {
        if(i++ > 10)
            break;
        
        rt_thread_delay(200);//200ms����һ������
    }

    while(1)
		{
			  rt_device_read(drv_dht,RT_NULL,&DHT,sizeof(DHT));
				rt_device_read(drv_ds,RT_NULL,&ds_data,sizeof(ds_data));
			
				slave->data.temp = (rt_uint16_t)(DHT.temperature);   //�¶�
			  slave->data.hum = (rt_uint16_t)(DHT.humidity);       //ʪ��
			  slave->data.ds_temp = (rt_uint16_t)ds_data;          //ds18B20�¶�
			
			  adc_temp_val1 = rt_adc_read(drv_adc,ADC1_CHANNEL);   //adc1�ɼ�  ������ʪ��
			  temp_buff[0] = (4092-adc_temp_val1)/3292*10000;
			  slave->data.land_hum = temp_buff[0];
			  rt_thread_delay(5);
			
			  rt_thread_delay(200);     //200ms������������
		}
}

int nrf_init(void)
{
	  struct _slave * slave;
	  
	  slave = rt_malloc(sizeof(struct _slave));
	  if(RT_NULL == slave)
      return RT_ERROR;
		
		rt_memset(slave,RT_NULL,sizeof(struct _slave));
		
		slave->data.addr    = ADDR;
    slave->data.channel = CHANNEL;	
	
	  nrf_tid = rt_thread_create("nrf_tid",nrf_thread_entry,slave,1024,10,10);
	  if(nrf_tid)
		{
				rt_thread_startup(nrf_tid);
		}
		
		sensor = rt_thread_create("sensor",sensor_collect_data,slave,1024,10,10);
		if(sensor)
		{
				rt_thread_startup(sensor);
		}
		
//		deal = rt_thread_create("deal",sensor_deal_entry,slave,1024,10,10);
//		if(deal)
//		{
//				rt_thread_startup(deal);
//		}
		
		return 0;
}

INIT_APP_EXPORT(nrf_init);

