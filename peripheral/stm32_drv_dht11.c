/****************************************************************************
* 版权信息：
* 系统名称：rt-thread-2.1.0 
* 文件名称：stm32_drv_dht11.c
* 文件说明：dht11温湿度驱动程序
* 作    者：TYX
* 版本信息：V1.0
* 设计日期：2017-03-26
* 修改记录：
* 日    期      版    本        修改人      修改摘要
  2017-03-26     V1.0            TYX       创建DHT11驱动程序
  2017-04-21     V2.0            TYX       修改读取IO电平方式，使用开漏输出方式读取
****************************************************************************/
/****************************************************************************
*1.未添加调试信息
****************************************************************************/

#include <rtdevice.h>
#include <rthw.h>
#include <stm32f4xx.h>
#include "stm32_drv_dht11.h"

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
#define HARD_DEVICE_NAME_FORMAT        "dht11_%d"
/* 引脚信息 */
const static gpio_desc _hard_desc[] =
{
    {GPIOC, 7},
};


/***************************************************************************
*                                分隔符                                    *
***************************************************************************/
#define  DHT11_DBG


#ifdef  DHT11_DBG
#define DBG_PRINT(fmt, ...)   rt_kprintf("DHT11_DBG:" fmt, ##__VA_ARGS__)
#else
#define DBG_PRINT(fmt, ...)
#endif


//#if 0 /* 库函数方式 */
//	#define PIN_OUT_LOW(PORT_DQ,PIN_DQ)     GPIO_ResetBits(PORT_DQ, PIN_DQ)
//	#define PIN_OUT_HIGH(PORT_DQ,PIN_DQ)    GPIO_SetBits(PORT_DQ, PIN_DQ)
//	/* 读取引脚电平 */
//	#define PIN_INPUT(PORT_DQ,PIN_DQ)       GP IO_ReadInputDataBit(PORT_DQ, PIN_DQ)
//#else	/* 直接操作寄存器，提高速度 */
//	#define PIN_OUT_LOW(PORT_DQ,PIN_DQ)     PORT_DQ->BSRR |= (PIN_DQ<<16)
//	#define PIN_OUT_HIGH(PORT_DQ,PIN_DQ)    PORT_DQ->BSRR |= (PIN_DQ)
//	/* 读取引脚电平 */
//	#define PIN_INPUT(PORT_DQ,PIN_DQ)	   (PORT_DQ->IDR & PIN_DQ)
//#endif


/* 获得数组元素个数 */
#ifndef ARRAY_SIZE
#define ARRAY_SIZE(arr)     (sizeof(arr)/sizeof(arr[0]))
#endif
/* 获得PIN */
#define get_st_pin(gpio_pin) (0x01 << (gpio_pin&0xFF))

/* DHT11驱动结构体 */
struct _dht11_drv
{
    struct rt_device device;        /* 通用驱动结构体 */
    const  gpio_desc * hard_desc;   /* 引脚信息 */
    rt_mutex_t mutex;               /* 互斥量，多线程访问 */
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
*函 数 名: _dht11_reset
*功能说明: 使DHT11器件复位
*形    参: 1. DHT11设备句柄
*返 回 值: 0 表示正常， 1 表示不正常
*其    他: 函数内部调用
****************************************************************************
*/
static rt_err_t _dht11_reset(struct _dht11_drv * drv)
{
    uint8_t retry = 0;
    
    DHT11_Dout_LOW();    //主机发送起始信号, 拉低时间 > 18ms
    rt_thread_delay(20);//拉低20ms
    DHT11_Dout_HIGH();   //主机拉高并释放总线
    rt_delay_us(30);
    
    DHT11_Data_IN();
    while(DHT11_Data_IN() && retry<100)  //等待总线被拉低
	{
		retry++;
		rt_delay_us(1);
	}
    
	if(retry>=100)             //超时返回错误
        return RT_ERROR;

    while((!DHT11_Data_IN())&&retry<100)   //等待总线被拉高
	{
		retry++;
		rt_delay_us(1);
	}
    
	if(retry>=100)           //超时返回错误
        return RT_ERROR;
    
	return RT_EOK;
}

/*
****************************************************************************
*函 数 名: _dht11_read_byte
*功能说明: 从DHT11器件中读取一个字节数据
*形    参: 1. DHT11设备句柄 2. 接收缓冲区首地址
*返 回 值: 0 表示正常， 1 表示不正常
*其    他: 函数内部调用
****************************************************************************
*/
static rt_err_t _dht11_read_byte(struct _dht11_drv * drv, uint8_t * data)
{
    uint8_t bit = 0,result = 0;
    uint8_t i = 0, retry = 0;
    
    for(i = 0;i<8;i++)
    {
        while(DHT11_Data_IN() && retry<100)//等待变为低电平
        {
            rt_delay_us(1);
            retry++;
        } 
        if(retry>=100)          //超时返回
            return RT_ERROR;
        retry = 0;
        while((!DHT11_Data_IN())&&retry<100)//等待变高电平
        {
            rt_delay_us(1);
            retry++;
        }
        if(retry>=100)          //超时返回
            return RT_ERROR;
        
        rt_delay_us(40);//等待40us
        if(DHT11_Data_IN()) bit=1;   //从总线上读取一位数据
        else bit=0;
        
        result = bit|(result<<1);
    }
    
    *data = result;
    
    return RT_EOK;
}

/*
****************************************************************************
*函 数 名: _dht11_read_data
*功能说明: 从DHT11器件中读取数据
*形    参: 1. DHT11设备句柄 2. 接收缓冲区首地址
*返 回 值: 0 表示正常， 1 表示不正常
*其    他: 函数内部调用
****************************************************************************
*/
static rt_err_t _dht11_read_data(struct _dht11_drv * _drv, HT * result)
{
    uint8_t i = 0;
    uint8_t temp_buf[5] = {0};
    
    if(_dht11_reset(_drv)==RT_EOK)              //复位成功,开始读取数据
    {
        for(i = 0;i<5;i++)
        {
            rt_enter_critical();
            _dht11_read_byte(_drv,&temp_buf[i]); //读取一字节数据
            rt_exit_critical();
        }
        if((temp_buf[0]+temp_buf[1]+temp_buf[2]+temp_buf[3])==temp_buf[4])  //计算校验和
		{
			result->humidity    = temp_buf[0];     //只取出了湿度整数部分
			result->temperature = temp_buf[2];     //只取出了温度整数部分
            
            return RT_EOK;
		}    
    }

    return RT_ERROR;        //复位失败，返回错误
}

/*
****************************************************************************
*函 数 名: stm32_dht11_init
*功能说明: 驱动初始化
*形    参: 1. dht11设备句柄
*返 回 值: 0 表示正常， 1 表示不正常
*其    他: 函数内部调用,初始化相应的IO引脚
****************************************************************************
*/
static rt_err_t stm32_dht11_init(rt_device_t dev)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    
    struct _dht11_drv * dht11 = (struct _dht11_drv *)dev;
    
    DHT11_Dout_GPIO_CLK_ENABLE();	      //使能时钟
    
    GPIO_InitStruct.Pin = get_st_pin(dht11->hard_desc->pin);          //选定引脚
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;                       //设为开漏输出模式
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;                //IO口最大速度
    HAL_GPIO_Init(dht11->hard_desc->gpio, &GPIO_InitStruct);

	HAL_GPIO_WritePin(dht11->hard_desc->gpio, get_st_pin(dht11->hard_desc->pin), GPIO_PIN_SET);
    
    return RT_EOK;
}

/*
****************************************************************************
*函 数 名: stm32_dht11_open
*功能说明: 打开驱动
*形    参: 1. dht11设备句柄 2.打开标志
*返 回 值: 0 表示正常， 1 表示不正常
*其    他: 函数内部调用
****************************************************************************
*/
static rt_err_t stm32_dht11_open(rt_device_t dev, rt_uint16_t oflag)
{   
    struct _dht11_drv * dht11 = (struct _dht11_drv *)dev;
    rt_err_t err = RT_EOK;
    
    rt_mutex_take(dht11->mutex,RT_WAITING_FOREVER);     //获取互斥信号量，加锁
    err = _dht11_reset(dht11);
    rt_mutex_release(dht11->mutex);
    return err;
}

/*
****************************************************************************
*函 数 名: stm32_dht11_close
*功能说明: 关闭驱动
*形    参: 1. dht11设备句柄
*返 回 值: 0 表示正常， 1 表示不正常
*其    他: 函数内部调用
****************************************************************************
*/
static rt_err_t stm32_dht11_close(rt_device_t dev)
{
    return RT_EOK;
}

/*
****************************************************************************
*函 数 名: stm32_dht11_control
*功能说明: 控制设备
*形    参: 1. dht11设备句柄 2. 控制命令  3. 参数
*返 回 值: 0 表示正常， 1 表示不正常
*其    他: 函数内部调用
****************************************************************************
*/
static rt_err_t stm32_dht11_control(rt_device_t dev, rt_uint8_t cmd, void *args)
{

    return RT_EOK;
}

/*
****************************************************************************
*函 数 名: stm32_dht11_read
*功能说明: 读驱动数据
*形    参: 1. dht11设备句柄 2. 偏移量 3. 缓冲区地址 4. 缓冲区大小
*返 回 值: 0 表示正常， 1 表示不正常
*其    他: 函数内部调用
****************************************************************************
*/
static rt_size_t stm32_dht11_read(rt_device_t dev, rt_off_t pos, void* buffer, rt_size_t size)
{
    HT temp;
    int count = 0,i = 0;
    HT * data = (HT *)buffer;
    struct _dht11_drv * dht11 = (struct _dht11_drv *)dev;
    
    while(i<(size/sizeof(HT)))     //获得所需数据个数
    {
        rt_mutex_take(dht11->mutex,RT_WAITING_FOREVER);   //请求互斥信号量，锁定
        if(_dht11_read_data(dht11,&temp) == RT_EOK)       //读取数据到临时缓冲区中
        {
            data[i].temperature   = temp.temperature;     //赋值给缓冲区
            data[i].humidity      = temp.humidity;
            count++;
        }
        rt_mutex_release(dht11->mutex);                   //释放互斥信号量,解锁
        i++;
    }

    return count;
}

/*
****************************************************************************
*函 数 名: dht11_drv_init
*功能说明: DHT11驱动注册
*形    参: 无
*返 回 值: 0 表示正常， 1 表示不正常
*其    他: 由INIT_DEVICE_EXPORT宏自动调用
****************************************************************************
*/
int dht11_drv_init(void)
{
    int i = 0, count = 0;
    char dev_name[sizeof(HARD_DEVICE_NAME_FORMAT) + 3] = "";
    struct _dht11_drv * dev = RT_NULL;
    
    for(i=0; i<ARRAY_SIZE(_hard_desc);i++)
    {
        rt_sprintf(dev_name, HARD_DEVICE_NAME_FORMAT, i);  //获得完整驱动名字
        if(rt_device_find(dev_name))           //名字被占用
        {
            DBG_PRINT("name repetition\r\n");
            continue;
        }
        
        dev = (struct _dht11_drv *)rt_malloc(sizeof(struct _dht11_drv));      //申请内存
        if (RT_NULL == dev)            //申请失败，结束当次循环
        {
            continue;
        }
        rt_memset(dev,RT_NULL,sizeof(struct _dht11_drv));     //清空这片内存
        
        dev->mutex =  rt_mutex_create(dev_name,RT_IPC_FLAG_FIFO); //创建信号量
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


INIT_DEVICE_EXPORT(dht11_drv_init);  //宏调用

