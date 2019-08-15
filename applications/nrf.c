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

#define water_relay  GET_PIN(B,6)      //温室灌溉
#define light_relay  GET_PIN(B,7)      //温室灌溉
#define ven_relay    GET_PIN(B,8)      //温室灌溉

static rt_thread_t nrf_tid;
static rt_thread_t sensor;
static rt_thread_t deal;

static rt_device_t nrf;
static rt_device_t drv_ds,drv_dht;
static rt_adc_device_t  drv_adc;          //使用ADC设备句柄

typedef struct dht11_data
{
    float humidity;
    float temperature;
}HT;

//定义正转反转数组
uint16_t phasecw[4] ={0x0200,0x0100,0x0080,0x0040};// D-C-B-A   
uint16_t phaseccw[4]={0x0040,0x0080,0x0100,0x0200};// A-B-C-D

//上报数据结构体
struct _data
{
    uint8_t addr;     //地址
    uint8_t channel;  //频道
    uint8_t sta;      //状态
    uint8_t cmd;      //命令
    
    uint16_t temp;    //温度数据 高位整数  低位小数
    uint16_t hum;     //湿度数据 高位整数  低位小数
    uint16_t ds_temp;   //烟雾数据 
    uint16_t land_hum;     //燃气数据
    uint16_t light;   //光强数据
    uint16_t ul;      //紫外线数据
    uint16_t PM2_5;   //PM2.5数据
    uint16_t COO;     //二氧化碳数据
    uint16_t TVOC;    //甲醛数据
    uint16_t noise;   //噪音数据
    uint16_t IO_1;    //开关量1
    uint16_t IO_2;    //开关量2
    
    uint8_t data_1;   //扩展数据1
    uint8_t data_2;   //扩展数据1
    
};

struct _cmd
{
    uint8_t addr;      //地址
    uint8_t channel;   //频道
    uint8_t sta;       //状态
    uint8_t cmd;       //命令
    
    uint8_t data_1;    //数据一  排序无先后顺序
    uint8_t data_2;    //数据二
    uint8_t data_3;    //数据三
    uint8_t data_4;    //数据四
    uint8_t data_5;    //数据五
    uint8_t data_6;    //数据六
		uint8_t data_7;    //数据四
    uint8_t data_8;    //数据五
    uint8_t data_9;    //数据六
};

/* 从机结构体 */
struct _slave
{
    struct _data data;  //从机上报数据
    struct _cmd  cmd;   //主机下发命令
    uint8_t link_sta;   //连接状态
    
};

/***********************************************
*函数名称：equip_status
*函数功能：继电器设置开关
*备注：
***********************************************/
void equip_status(rt_base_t pin,rt_uint8_t val)
{
	  rt_pin_mode(pin,PIN_MODE_OUTPUT);
		rt_pin_write(pin,val);
}

/******************************************************
*函数名称：nrf_thread_entry
*功能：nrf数据发送接收线程
*备注：
******************************************************/
void nrf_thread_entry(void *parameter)
{
	  rt_uint8_t result, i = 0;
	  uint32_t link_out = 0;
	  int link_sta = 0;
		char temp_buf[32];
	  struct _slave * slave = parameter;

		rt_hw_spi_device_attach(SPI_BUS_NAME,SPI_DEVICE_NAME,GPIOG,GPIO_PIN_7);   //bsp/stm32/下的驱动需调用此函数进行SPI挂载
	
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
              
            link_out++;   //连接超时自增
            rt_thread_delay(50);
            continue;
				}
				
				if(temp_buf[0] != ADDR)  //如果不是主机地址
        {
            rt_thread_delay(100);
            continue;
        }
        
        if(temp_buf[1] != CHANNEL)  //如果不是主机通道
        {
            rt_thread_delay(100);
            continue;
        }
        
				rt_kprintf("%x\n",temp_buf[0]);
				
        rt_memcpy(&(slave->cmd),temp_buf,sizeof(struct _cmd));
        rt_thread_delay(10);
        /* 回复主机数据 */
        if(rt_device_write(nrf,RT_NULL,&slave->data,sizeof(struct _data)))       //数据需要修改！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！
        {
            /* 发送失败，主机没有接收到数据 */
        }

        link_out = 0;
        /* 正常通信 */
        slave->link_sta = 1;
        
        rt_thread_delay(200);
		}
}

void sensor_deal_entry(void *parameter)
{
    struct _slave * slave = parameter;
	  
	  /* 继电器1-- */
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
	/*******************************ADC设备测试*********************************/
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
        
        rt_thread_delay(200);//200ms处理一次数据
    }

    while(1)
		{
			  rt_device_read(drv_dht,RT_NULL,&DHT,sizeof(DHT));
				rt_device_read(drv_ds,RT_NULL,&ds_data,sizeof(ds_data));
			
				slave->data.temp = (rt_uint16_t)(DHT.temperature);   //温度
			  slave->data.hum = (rt_uint16_t)(DHT.humidity);       //湿度
			  slave->data.ds_temp = (rt_uint16_t)ds_data;          //ds18B20温度
			
			  adc_temp_val1 = rt_adc_read(drv_adc,ADC1_CHANNEL);   //adc1采集  土壤温湿度
			  temp_buff[0] = (4092-adc_temp_val1)/3292*10000;
			  slave->data.land_hum = temp_buff[0];
			  rt_thread_delay(5);
			
			  rt_thread_delay(200);     //200ms供数据有序处理
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

