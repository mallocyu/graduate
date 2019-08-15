#include <rtthread.h>
#include <stdio.h>
#include <stdint.h>
#include <rtdevice.h>

static rt_thread_t sensor;
static rt_device_t drv_ds,drv_dht;
rt_uint16_t data_buff[5] = {0};

typedef struct dht11_data
{
    float humidity;
    float temperature;
}HT;

void sensor_collect_data(void *parameter)
{
	  int i = 0;
	  float ds_data = 0;
	  HT DHT;
	
	  drv_ds = rt_device_find("18b20_0");
	  drv_dht = rt_device_find("dht11_0");
	  
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
			  data_buff[2] = (rt_uint16_t)(DHT.humidity);
			  data_buff[3] = (rt_uint16_t)(DHT.temperature);
			  
			  rt_device_read(drv_ds,RT_NULL,&ds_data,sizeof(ds_data));
			  data_buff[4] = (rt_uint16_t)ds_data;
			
			  rt_thread_delay(200);     //200ms供数据有序处理
		}
}

int sensor_data(void)
{
	sensor = rt_thread_create("sensor",sensor_collect_data,RT_NULL,1024,10,10);
	if(sensor)
	{
			rt_thread_startup(sensor);
	}
	
		return 0;
}

INIT_APP_EXPORT(sensor_data);

