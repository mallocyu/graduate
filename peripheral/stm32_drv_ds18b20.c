/****************************************************************************
* 版权信息：
* 系统名称：rt-thread-2.1.0 
* 文件名称：stm32_drv_ds18b20.c
* 文件说明：18b20温度驱动程序
* 作    者：TYX
* 版本信息：V1.0
* 设计日期：2017-03-25
* 修改记录：
* 日    期      版    本        修改人      修改摘要
  2017-03-25     V1.0            TYX       创建18B20驱动
  2017-04-21     V2.0            TYX       修改读取IO电平方式，使用开漏输出方式读取
  2017-04-22     V2.1            TYX       添加调试信息
****************************************************************************/
/****************************************************************************
*1.不支持修改量程
****************************************************************************/

#include <rtdevice.h>
#include <rthw.h>
// #include "board.h"
#include <stm32f4xx.h>
#include "stm32_drv_ds18b20.h"

/****************************************************************************
* 用户须知
* 1.名字:   
*   1>自定义名字，不能与已有的名字重复
*   2>如有多个器件，要带上 %d 进行编号，理论最大支持99个设备
* 2.引脚信息：
*   1>一个引脚上只能接一个器件
*   2>多个器件可添加多条引脚信息
* 3.驱动注册函数会自动调用，无需手动调用
****************************************************************************/

/* 驱动名字 */
#define HARD_DEVICE_NAME_FORMAT        "18b20_%d"
/* 引脚信息 */
const static gpio_des _hard_desc[] =
{
    {GPIOG, 9},
};

/***************************************************************************
*                                分隔符                                    *
***************************************************************************/
#define  DS18B20_DBG

#ifdef  DS18B20_DBG
#define DBG_PRINT(fmt, ...)   rt_kprintf("DS18B20_DBG:" fmt, ##__VA_ARGS__)
#else
#define DBG_PRINT(fmt, ...)
#endif

/* 获得数组元素个数 */
#ifndef ARRAY_SIZE
#define ARRAY_SIZE(arr)     (sizeof(arr)/sizeof(arr[0]))
#endif

/* 获得PIN */
#define get_st_pin(gpio_pin) (0x01 << (gpio_pin&0xFF))

/* DS18B20驱动结构体 */
struct _ds18b20_drv
{
    struct rt_device  device;      /* 通用驱动结构体 */
    const gpio_des * hard_desc;   /* 引脚信息 */
    rt_mutex_t mutex;              /* 互斥量，多线程访问 */
};

static void rt_delay_us(int us)
{
	rt_uint32_t ticks;
	rt_uint32_t told,tnow,tcnt=0;
	rt_uint32_t reload=SysTick->LOAD;
	
    /* 获得延时经过的tick数 */
    ticks = us * (reload / (1000000 / RT_TICK_PER_SECOND));
	
    /* 第一次定时器的值 */
    told = SysTick->VAL;
    /* 循环获得当前时间，直到达到指定的时间后退出循环 */
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
*函 数 名: _ds18b20_reset
*功能说明: 使DS18B20器件复位
*形    参: 1. 18b20设备句柄
*返 回 值: 0 表示正常， 1 表示不正常
*其    他: 函数内部调用
****************************************************************************
*/
static rt_err_t _ds18b20_reset(struct _ds18b20_drv * drv)
{
    uint8_t retry = 0;

//    DS18B20_IO_OUT();
    DS18B20_Dout_LOW();   //主机拉低总线650us
    rt_delay_us(750);   //拉低650us
    DS18B20_Dout_HIGH(); //主机释放总线
    rt_delay_us(15);   //等待10us
    
    DS18B20_Data_IN();
    while(DS18B20_Data_IN() && retry<200)  //等待总线被从机拉低
	{
		retry++;
		rt_delay_us(1);  
	}
    
	if(retry >= 200)       //超时则退出，初始化失败
    {
        DBG_PRINT("TimeOut :DS18B20 not detected\r\n");
        return RT_ERROR;
    }

    while(!DS18B20_Data_IN() && retry<240)  //等待从机释放总线
	{
		retry++;
		rt_delay_us(1);
	}
    
	if(retry >= 240)     //超时则退出，初始化失败
    {
        DBG_PRINT("TimeOut :DS18B20 not release bus\r\n");
        return RT_ERROR;
    }
        
	return RT_EOK;
}

/*
****************************************************************************
*函 数 名: _ds18b20_read_byte
*功能说明: 从18B20里读取一个字节
*形    参: 1. 18b20设备句柄 2. 缓存区地址
*返 回 值: 0 表示正常， 1 表示不正常
*其    他: 函数内部调用
****************************************************************************
*/
static rt_err_t _ds18b20_read_byte(struct _ds18b20_drv * drv, uint8_t * data)
{
    uint8_t i = 0;
    uint8_t bit = 0,result = 0;
    
    for(i = 0;i < 8;i++)
    {
        DS18B20_Dout_LOW();     //主机拉低总线
        rt_delay_us(1);
        DS18B20_Dout_HIGH();    //主机释放总线
        rt_delay_us(12); 
        if(DS18B20_Data_IN()) bit=1;  //读取总线电平
        else bit=0;	        
        result=(bit<<7) | (result>>1);
        
        rt_delay_us(50);
    }
    *data = result;
    
    return RT_EOK;
}


/*
****************************************************************************
*函 数 名: _ds18b20_write_byte
*功能说明: 往18B20里写一个字节
*形    参: 1. 18b20设备句柄 2. 写入数据首地址
*返 回 值: 0 表示正常， 1 表示不正常
*其    他: 函数内部调用
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
            DS18B20_Dout_LOW(); //主机拉低总线,准备往总线上写数据
            rt_delay_us(2);
            DS18B20_Dout_HIGH();     //主机写入一个位
            rt_delay_us(60);
        }
        else
        {
            DS18B20_Dout_LOW(); //主机拉低总线,准备往总线上写数据
            rt_delay_us(60);                                       //主机写入一个位
            DS18B20_Dout_HIGH();
            rt_delay_us(2);
        }
    }
   
    return RT_EOK;
}


/*
****************************************************************************
*函 数 名: _ds18b20_start
*功能说明: ds18b20进行一次温度转换
*形    参: 1. 18b20设备句柄
*返 回 值: 0 表示正常， 1 表示不正常
*其    他: 函数内部调用
****************************************************************************
*/
static rt_err_t _ds18b20_start(struct _ds18b20_drv * drv)
{
    if(_ds18b20_reset(drv) == RT_EOK)
    {
        rt_enter_critical();                   //进入临时区
        _ds18b20_write_byte(drv,0xcc);	       //发命令 跳过读 ROM
        _ds18b20_write_byte(drv,0x44);	       //发命令 开始转换
        rt_exit_critical();
        
        return RT_EOK;
    }
    
    return RT_ERROR;
}


/*
****************************************************************************
*函 数 名: _ds18b20_read_data
*功能说明: 从ds18b20中读取温度数据
*形    参: 1. 18b20设备句柄 2.缓冲区首地址
*返 回 值: 0 表示正常， 1 表示不正常
*其    他: 函数内部调用
****************************************************************************
*/
static rt_err_t _ds18b20_read_data(struct _ds18b20_drv * drv, float * result)
{
    uint8_t  data_H, data_L;
    short data;
    
    _ds18b20_reset(drv);                 //复位，准备进行通信
    rt_enter_critical();                 //进入临界区
    _ds18b20_write_byte(drv,0xcc);	     //发命令 跳过读 ROM
    _ds18b20_write_byte(drv,0xbe);	     //发命令 开始传送温度原始数据
    _ds18b20_read_byte(drv, &data_L); 	 //温度低8位  
    _ds18b20_read_byte(drv, &data_H); 	 //温度高8位
    rt_exit_critical();                  //退出临界区
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
*函 数 名: stm32_ds18b20_init
*功能说明: 驱动初始化
*形    参: 1. 18b20设备句柄
*返 回 值: 0 表示正常， 1 表示不正常
*其    他: 函数内部调用,初始化相应的IO引脚
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
*函 数 名: stm32_ds18b20_open
*功能说明: 打开驱动
*形    参: 1. 18b20设备句柄 2.打开标志
*返 回 值: 0 表示正常， 1 表示不正常
*其    他: 函数内部调用
****************************************************************************
*/
static rt_err_t stm32_ds18b20_open(rt_device_t dev, rt_uint16_t oflag)
{   
    struct _ds18b20_drv * ds18b20 = (struct _ds18b20_drv *)dev;
    rt_err_t err = RT_EOK;
    
    rt_mutex_take(ds18b20->mutex,RT_WAITING_FOREVER);     //获取互斥信号量，加锁
    err = _ds18b20_reset(ds18b20);
    rt_mutex_release(ds18b20->mutex);
    
    return err;
}

/*
****************************************************************************
*函 数 名: stm32_ds18b20_close
*功能说明: 关闭驱动
*形    参: 1. 18b20设备句柄
*返 回 值: 0 表示正常， 1 表示不正常
*其    他: 函数内部调用
****************************************************************************
*/
static rt_err_t stm32_ds18b20_close(rt_device_t dev)
{
    return RT_EOK;
}

/*
****************************************************************************
*函 数 名: stm32_ds18b20_read
*功能说明: 读驱动数据
*形    参: 1. 18b20设备句柄 2. 偏移量 3. 缓冲区地址 4. 缓冲区大小
*返 回 值: 0 表示正常， 1 表示不正常
*其    他: 函数内部调用
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
        rt_mutex_take(ds18b20->mutex,RT_WAITING_FOREVER);     //获取互斥信号量，加锁
        if(_ds18b20_start(ds18b20) == RT_EOK)                 //开始一次转换
        {
            rt_thread_delay(800);                             //挂起等待一段时间
            if(_ds18b20_read_data(ds18b20, &temp) == RT_EOK)  //读取数据
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
*函 数 名: stm32_ds18b20_control
*功能说明: 控制设备
*形    参: 1. 18b20设备句柄 2. 控制命令  3. 参数
*返 回 值: 0 表示正常， 1 表示不正常
*其    他: 函数内部调用,后面会增加改变精度命令,目前为空
****************************************************************************
*/
static rt_err_t stm32_ds18b20_control(rt_device_t dev, int cmd, void *args)
{

    return RT_EOK;
}

/*
****************************************************************************
*函 数 名: ds18b20_drv_init
*功能说明: DS18B20驱动注册
*形    参: 无
*返 回 值: 0 表示正常， 1 表示不正常
*其    他: 由INIT_DEVICE_EXPORT宏自动调用
****************************************************************************
*/
int ds18b20_drv_init(void)
{
    int i = 0, count = 0;
    char dev_name[sizeof(HARD_DEVICE_NAME_FORMAT) + 3] = "";
    struct _ds18b20_drv *dev = RT_NULL;

    for(i=0; i<ARRAY_SIZE(_hard_desc);i++)          //遍历数组，添加DS18B20驱动
    {
        rt_sprintf(dev_name, HARD_DEVICE_NAME_FORMAT, i);  //获得完整驱动名字
        if(rt_device_find(dev_name))           //当前驱动名字被使用
        {
            DBG_PRINT("name repetition\r\n");
            continue;
        }

        dev = (struct _ds18b20_drv *)rt_malloc(sizeof(struct _ds18b20_drv));  //申请内存
        if (RT_NULL == dev)
        {
            DBG_PRINT("malloc device Memory fail\r\n");
            continue;
        }
        rt_memset(dev,RT_NULL,sizeof(struct _ds18b20_drv));    //清空这片内存
        
        dev->mutex =  rt_mutex_create(dev_name,RT_IPC_FLAG_FIFO); //创建互斥信号量
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
        
        rt_device_register(&(dev->device),dev_name,RT_DEVICE_FLAG_RDONLY);   //将驱动注册到系统,供上层应用程序调用
        
        count++;     //成功数量加1
    }
    
    return count;
}


INIT_DEVICE_EXPORT(ds18b20_drv_init);   //驱动自动调用


