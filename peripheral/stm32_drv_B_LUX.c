/****************************************************************************
* 版权信息：
* 系统名称：rt-thread-4.0.1 
* 文件名称：stm32_drv_blux.c
* 文件说明：blux驱动程序
* 作    者：YCG
* 版本信息：V1.0
* 设计日期：2019-05-26
* 修改记录：
* 日    期      版    本        修改人      修改摘要
  2019-05-26     V1.0            YCG       创建光照传感器驱动
****************************************************************************/
/***********************************************************
*注意：
*    1、本驱动未进行有效性测试，请谨慎使用！！
**********************************************************/
#include <rtdevice.h>
#include <rtthread.h>
#include <rthw.h>
#include "board.h"
#include "stm32_drv_b_lux.h"

#define BLUX_I2C_BUS_NAME          "i2c1"  /* 传感器连接的I2C总线设备名称 */
#define	B_LUX_V30B_SlaveAddr_w 	  0x94      //定义器件在IIC总线上的从机地址  写
#define	B_LUX_V30B_SlaveAddr_r 	  0x95      //定义器件在IIC总线上的从机地址  读

/***********************************************************
说明：
    0x00 - 0x03为读寄存器；(存储32位光照值)
		0x04 - 0x07为写寄存器；(0x04为配置寄存器，0x05 - 0x07备用)
		0x10 - 0x80为112字节的EEPROM
备注：B_LUX_V30B采用的是模拟的IIC，必须是SCL、SDA设置成推挽输出

************************************************************/

#define BLUX_REG_ADDR          0x04
#define BLUX_LTGHT_SET         0x00     //在 微光 模式下设置0x04寄存器
#define BLUX_BRIGHT_LIGHT_SET  0x03     //在 强光 模式下设置0x04寄存器

///* 引脚信息 */
//const static gpio_des _hard_desc[] =
//{
//	  {GPIOB, 8},      //SCL
//    {GPIOB, 9},      //SDA
//		{GPIOD, 8},      //EN    
//};


#ifdef  BLUX_DBG
#define DBG_PRINT(fmt, ...)   rt_kprintf("BLUX_DBG:" fmt, ##__VA_ARGS__)
#else
#define DBG_PRINT(fmt, ...)
#endif

struct _blux_device
{
		struct rt_device device;              /* 通用驱动结构体 */
	  struct rt_i2c_bus_device *iic_dev;    /* IIC设备句柄 */
	  rt_mutex_t  mutex;                    /* 互斥信号量 */
};

/****************************************************************************
*函 数 名: blux_write_reg
*功能说明: 写mpu6050单个寄存器
*形    参: 1. 寄存器地址  2.data
*返 回 值: 0 表示正常， 1 表示不正常
*其    他: 函数内部调用
****************************************************************************/
static rt_err_t blux_write_reg(rt_device_t dev,rt_uint8_t reg, rt_uint8_t data)
{
	  struct _blux_device * drv = (struct _blux_device *)dev;
	  
    struct rt_i2c_msg msgs;
    rt_uint8_t buf[2] = {reg, data};

    msgs.addr  = B_LUX_V30B_SlaveAddr_w;    /* 从机地址 */
    msgs.flags = RT_I2C_WR;                 /* 写标志 */
    msgs.buf   = buf;                       /* 发送数据指针 */
    msgs.len   = 2;

    if (rt_i2c_transfer(drv->iic_dev, &msgs, 1) == 1)
    {
        return RT_EOK;
    }
    else
    {
        return -RT_ERROR;
    }
}


/****************************************************************************
*函 数 名: blux_read_reg
*功能说明: 读取寄存器数据
*形    参: 1. 要读取的寄存器地址  2.要读取的数据字节数 3.v读取到的数据存储区
*返 回 值: 0 表示正常， 1 表示不正常
*其    他: 函数内部调用
****************************************************************************/
static rt_err_t blux_read_reg(rt_device_t dev,rt_uint8_t reg, rt_uint8_t *buf, rt_uint8_t len)
{   
	  struct _blux_device * drv = (struct _blux_device *)dev;
    struct rt_i2c_msg msgs[2];

    msgs[0].addr  = B_LUX_V30B_SlaveAddr_w;    /* 从机地址 */
    msgs[0].flags = RT_I2C_WR;      					 /* 写标志 */
    msgs[0].buf   = &reg;           					 /* 从机寄存器地址 */
    msgs[0].len   = 1;               					 /* 发送数据字节数 */

    msgs[1].addr  = B_LUX_V30B_SlaveAddr_w;    /* 从机地址 */
    msgs[1].flags = RT_I2C_RD;       					 /* 读标志 */
    msgs[1].buf   = buf;            					 /* 读取数据指针 */
    msgs[1].len   = len;           					   /* 读取数据字节数 */

    if (rt_i2c_transfer(drv->iic_dev, msgs, 2) == 2)
    {
        return RT_EOK;
    }
    else
    {
        return -RT_ERROR;
    }

}

/****************************************************************************
*函 数 名: blux_open
*功能说明: 使DHT11器件复位
*形    参: 1. DHT11设备句柄
*返 回 值: 0 表示正常， 1 表示不正常
*其    他: 函数内部调用
****************************************************************************/
static rt_err_t stm32_blux_open(rt_device_t dev, rt_uint16_t oflag)
{
	  /* 可向blux的0x05-0x0f的可读写寄存器写数据，读值 */
	  struct _blux_device * drv = (struct _blux_device *)dev;
	  rt_uint8_t result;
	
	  drv->iic_dev = (struct rt_i2c_bus_device *)rt_device_find(BLUX_I2C_BUS_NAME);
	  if(drv->iic_dev == RT_NULL)
		{
				DBG_PRINT("NOT found bus!!!\n");
			  return -1;
		}
		
		result = blux_write_reg(drv->iic_dev,BLUX_REG_ADDR,BLUX_LTGHT_SET);    //写0x04寄存器  设置0x00
		if(result != RT_EOK)
		{
				DBG_PRINT("write reg fail!!\n",__FUNCTION__,__LINE__);
			  return -1;
		}
	
		
		
		return RT_EOK;
}


/****************************************************************************
*函 数 名: _stm32_blux_read
*功能说明: 打开驱动
*形    参: 1. 设备句柄 2.打开标志
*返 回 值: 0 表示正常， 1 表示不正常
*其    他: 函数内部调用
****************************************************************************/
static rt_size_t stm32_blux_read(rt_device_t dev,rt_off_t pos, void* buffer, rt_size_t size)
{
	  struct _blux_device *drv = (struct _blux_device *)dev;
	
		rt_uint8_t vBuf[129];                       //接收数据缓存区 	
		rt_uint8_t val32 = 0;  
		float      temp;
	  rt_uint32_t *result = buffer;
	
	  blux_read_reg(drv->iic_dev,0x00,vBuf,sizeof(vBuf));
	  rt_thread_delay(rt_tick_from_millisecond(10));   //delay 10 ms
	  
		val32 = vBuf[3];
		val32 = val32<< 8;
		val32 |= vBuf[2];
		val32 = val32<< 8;
		val32 |= vBuf[1];
		val32 = val32<< 8;
		val32 |= vBuf[0];
	
	  temp = (float)(val32*1.4);			//半球透明度矫正值*1.4
	
    *result = (rt_uint32_t)(temp);

	
	  return 0;
}

int blux_drv_init(void)
{
    int i = 0, count = 0;
    struct _blux_device * dev = RT_NULL;
        
		dev = (struct _blux_device *)rt_malloc(sizeof(struct _blux_device));      //申请内存
		if (RT_NULL == dev)            //申请失败，结束当次循环
		{
				DBG_PRINT("rt_malloc fail\r\n");
        return -1;
		}
		rt_memset(dev,RT_NULL,sizeof(struct _blux_device));     //清空这片内存
		
		dev->mutex =  rt_mutex_create("blux_lock",RT_IPC_FLAG_FIFO); //创建信号量
		if(RT_NULL == dev->mutex)
		{
				rt_free(dev);
				return -1;
		}
		
//        dev->hard_desc        = &(_hard_desc[i]);
		dev->device.type      = RT_Device_Class_Miscellaneous;
//		dev->device.init      = stm32_blux_init;
		dev->device.open      = stm32_blux_open;
		dev->device.close     = RT_NULL;
		dev->device.read      = stm32_blux_read;
		dev->device.write     = RT_NULL;
		dev->device.control   = RT_NULL;
		dev->device.user_data = RT_NULL;
		
		rt_device_register(&(dev->device),"blux_dev",RT_DEVICE_FLAG_RDWR);

		count++;

    
    return count;
}

INIT_DEVICE_EXPORT(blux_drv_init);   //驱动自动调用
