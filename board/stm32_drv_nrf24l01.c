/****************************************************************************
* 版权信息：
* 系统名称：rt-thread-2.1.0 
* 文件名称：stm32_drv_nrf24l01.c
* 文件说明：NRF24L01驱动程序
* 作    者：TYX
* 版本信息：V1.0
* 设计日期：2017-04-26
* 修改记录：
* 日    期      版    本        修改人      修改摘要
  2017-04-26     V1.0            TYX       创建NRF驱动
	2019-05-05     V1.1            YCG       修改NRF总线引脚配置
****************************************************************************/
/****************************************************************************
*1.不支持读屏幕型号
*2.虚拟SPI总线没有编号,败笔
****************************************************************************/

#include "rtdevice.h"
#include <rtthread.h>
#include "stm32_drv_nrf24l01.h"

/****************************************************************************
* 用户须知
* 1.名字:   
*   1> 自定义名字，不能与已有的名字重复
* 2.信号线： NRF使用SPI总线通信
* 3.引脚信息：除去SPI三根总线,还需自定义两根信号线
*   1> 此显示屏会用到两个自定义引脚：
*      1 24L01片选信号
*　　  2 SPI片选信号线
*    两根线的顺序不能乱，一定是这样子的顺序
* 3.驱动注册函数会自动调用，无需手动调用
* 4.SPI总线设备,都要注册一个虚拟SPI总线,挂在实际总线上,虚拟总线共99跟
* 5.很烦RT SPI总线  搞个总线 还用注册个虚拟总线
****************************************************************************/


/* 驱动名字 */
#define HARD_DEVICE_NAME_FORMAT        "nrf_%d"

/* 引脚信息 */
const static gpio_desc _hard_desc[] =
{
    {GPIOG, 6},   //CE
    {GPIOG, 8},   //IRQ
};


/***************************************************************************
*                                分隔符                                    *
***************************************************************************/
/* 获得PIN */
#define get_st_pin(gpio_pin) (0x01 << (gpio_pin&0xFF))

#define SPI_DVICES_NAME                 "spi10"

#ifdef  NRF_DBG
#define DBG_PRINT(fmt, ...)   rt_kprintf("NRF_DBG:" fmt, ##__VA_ARGS__)
#else
#define DBG_PRINT(fmt, ...)
#endif


/* 获得数组元素个数 */
#ifndef ARRAY_SIZE
#define ARRAY_SIZE(arr)     (sizeof(arr)/sizeof(arr[0]))
#endif


/* 虚拟SPI总线名字 */
#define SPI_VR_NAME_FORMAT             "spi_vr%d"

//////////////////////////////////////////////////////////////////////////////////////////////////////////
//NRF24L01寄存器操作命令
#define NRF_READ_REG    0x00  //读配置寄存器,低5位为寄存器地址
#define NRF_WRITE_REG   0x20  //写配置寄存器,低5位为寄存器地址
#define RD_RX_PLOAD     0x61  //读RX有效数据,1~32字节
#define WR_TX_PLOAD     0xA0  //写TX有效数据,1~32字节
#define FLUSH_TX        0xE1  //清除TX FIFO寄存器.发射模式下用
#define FLUSH_RX        0xE2  //清除RX FIFO寄存器.接收模式下用
#define REUSE_TX_PL     0xE3  //重新使用上一包数据,CE为高,数据包被不断发送.
#define NOP             0xFF  //空操作,可以用来读状态寄存器	 
//SPI(NRF24L01)寄存器地址
#define CONFIG          0x00  //配置寄存器地址;bit0:1接收模式,0发射模式;bit1:电选择;bit2:CRC模式;bit3:CRC使能;
                              //bit4:中断MAX_RT(达到最大重发次数中断)使能;bit5:中断TX_DS使能;bit6:中断RX_DR使能
#define EN_AA           0x01  //使能自动应答功能  bit0~5,对应通道0~5
#define EN_RXADDR       0x02  //接收地址允许,bit0~5,对应通道0~5
#define SETUP_AW        0x03  //设置地址宽度(所有数据通道):bit1,0:00,3字节;01,4字节;02,5字节;
#define SETUP_RETR      0x04  //建立自动重发;bit3:0,自动重发计数器;bit7:4,自动重发延时 250*x+86us
#define RF_CH           0x05  //RF通道,bit6:0,工作通道频率;
#define RF_SETUP        0x06  //RF寄存器;bit3:传输速率(0:1Mbps,1:2Mbps);bit2:1,发射功率;bit0:低噪声放大器增益
#define STATUS          0x07  //状态寄存器;bit0:TX FIFO满标志;bit3:1,接收数据通道号(最大:6);bit4,达到最多次重发
                              //bit5:数据发送完成中断;bit6:接收数据中断;
#define MAX_TX  		0x10  //达到最大发送次数中断
#define TX_OK   		0x20  //TX发送完成中断
#define RX_OK   		0x40  //接收到数据中断

#define OBSERVE_TX      0x08  //发送检测寄存器,bit7:4,数据包丢失计数器;bit3:0,重发计数器
#define CD              0x09  //载波检测寄存器,bit0,载波检测;
#define RX_ADDR_P0      0x0A  //数据通道0接收地址,最大长度5个字节,低字节在前
#define RX_ADDR_P1      0x0B  //数据通道1接收地址,最大长度5个字节,低字节在前
#define RX_ADDR_P2      0x0C  //数据通道2接收地址,最低字节可设置,高字节,必须同RX_ADDR_P1[39:8]相等;
#define RX_ADDR_P3      0x0D  //数据通道3接收地址,最低字节可设置,高字节,必须同RX_ADDR_P1[39:8]相等;
#define RX_ADDR_P4      0x0E  //数据通道4接收地址,最低字节可设置,高字节,必须同RX_ADDR_P1[39:8]相等;
#define RX_ADDR_P5      0x0F  //数据通道5接收地址,最低字节可设置,高字节,必须同RX_ADDR_P1[39:8]相等;
#define TX_ADDR         0x10  //发送地址(低字节在前),ShockBurstTM模式下,RX_ADDR_P0与此地址相等
#define RX_PW_P0        0x11  //接收数据通道0有效数据宽度(1~32字节),设置为0则非法
#define RX_PW_P1        0x12  //接收数据通道1有效数据宽度(1~32字节),设置为0则非法
#define RX_PW_P2        0x13  //接收数据通道2有效数据宽度(1~32字节),设置为0则非法
#define RX_PW_P3        0x14  //接收数据通道3有效数据宽度(1~32字节),设置为0则非法
#define RX_PW_P4        0x15  //接收数据通道4有效数据宽度(1~32字节),设置为0则非法
#define RX_PW_P5        0x16  //接收数据通道5有效数据宽度(1~32字节),设置为0则非法
#define NRF_FIFO_STATUS 0x17  //FIFO状态寄存器;bit0,RX FIFO寄存器空标志;bit1,RX FIFO满标志;bit2,3,保留
                              //bit4,TX FIFO空标志;bit5,TX FIFO满标志;bit6,1,循环发送上一数据包.0,不循环;
//////////////////////////////////////////////////////////////////////////////////////////////////////////
//24L01发送接收数据宽度定义
#define TX_ADR_WIDTH    5   	//5字节的地址宽度
#define RX_ADR_WIDTH    5   	//5字节的地址宽度
#define TX_PLOAD_WIDTH  32  	//32字节的用户数据宽度
#define RX_PLOAD_WIDTH  32  	//32字节的用户数据宽度
//24L01发送接受地址定义
#define NRF_MAC_ADDRESS 0x4A

/* NRF信息结构体 */
struct _nrf_info
{
    uint8_t tx_address[TX_ADR_WIDTH];  /* 发送地址 */
    uint8_t rx_address[TX_ADR_WIDTH];  /* 接收地址 */
    
};

/* NRF驱动结构体 */
struct _nrf_drv
{
    struct rt_device device;      /* 通用驱动结构体 */
    struct rt_spi_device *spi_dev;/* SPI总线 */
    rt_mutex_t  mutex;            /* 互斥信号量 */
    const gpio_desc * gpio_ce;    /* CE引脚信息 */
    const gpio_desc * gpio_irq;   /* IRQ引脚信息 */
};

/*
****************************************************************************
*函 数 名: stm32_nrf_init
*功能说明: 驱动初始化
*形    参: 1. 设备句柄
*返 回 值: 0 表示正常， 1 表示不正常
*其    他: 函数内部调用,初始化相应的IO引脚
****************************************************************************
*/
static rt_err_t stm32_nrf_init(rt_device_t dev)
{
    GPIO_InitTypeDef  GPIO_InitStruct;
    struct _nrf_drv * drv = (struct _nrf_drv *)dev;
    
		NRF_GPIO_CLK_ENABLE();    //init GPIOG clock
    
    GPIO_InitStruct.Pin = get_st_pin(drv->gpio_ce->pin);	       //选定引脚
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; 		             //设为推挽输出模式
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;           //IO口最大速度
    HAL_GPIO_Init(drv->gpio_ce->gpio, &GPIO_InitStruct);         //初始化IO口
    
    GPIO_InitStruct.Pin = get_st_pin(drv->gpio_irq->pin);	       //选定引脚
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT; 	                   //设为下拉输入
		GPIO_InitStruct.Pull = GPIO_PULLUP;                          //上拉 输入
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;           //IO口最大速度
    HAL_GPIO_Init(drv->gpio_irq->gpio, &GPIO_InitStruct);        //初始化IO口
    
    HAL_GPIO_WritePin(drv->gpio_ce->gpio,get_st_pin(drv->gpio_ce->pin),GPIO_PIN_RESET);      //拉低

    return RT_EOK;
}

/*
****************************************************************************
*函 数 名: stm32_nrf_open
*功能说明: 打开驱动
*形    参: 1. 设备句柄 2.打开标志
*返 回 值: 0 表示正常， 1 表示不正常
*其    他: 函数内部调用
****************************************************************************
*/
static rt_err_t stm32_nrf_open(rt_device_t dev, rt_uint16_t oflag)
{
    int i = 0;
    uint8_t temp_buf[TX_ADR_WIDTH+1];
    struct rt_spi_configuration cfg;
    struct _nrf_drv * drv = (struct _nrf_drv *)dev;
	
		drv->spi_dev = (struct rt_spi_device *)rt_device_find(SPI_DVICES_NAME);
    if (drv->spi_dev == RT_NULL)
    {
        /* 注意！！！！！ 没有找到总线！！！！  */
        DBG_PRINT("not find spi device, name:%s\n", SPI_DVICES_NAME);
        return -1;
    }
	  NRF_CE_LOW();
    
    cfg.data_width = 8;             //设置虚拟SPI总线数据宽度
    cfg.mode = RT_SPI_MODE_0 | RT_SPI_MSB; //SPI模式等
    cfg.max_hz = 10 * 1000 * 1000;   //SPI速度  9M
    
    rt_spi_configure(drv->spi_dev, &cfg);
    
              /* 配置TX节点地址 */
    temp_buf[0] = NRF_WRITE_REG+TX_ADDR;  //写Tx节点命令
    rt_memset(temp_buf+1,NRF_MAC_ADDRESS,TX_ADR_WIDTH);    //填充地址数据
    rt_spi_send(drv->spi_dev,temp_buf,TX_ADR_WIDTH+1); //发送命令和数据
    rt_memset(temp_buf,RT_NULL,sizeof(temp_buf));          //清空缓冲区
    temp_buf[0] = NRF_READ_REG + TX_ADDR;                  //读Tx节点命令
    rt_spi_send_then_recv(drv->spi_dev, temp_buf, 1, temp_buf + 1, TX_ADR_WIDTH); //发送命令 读取数据

    for(i = 1;i < TX_ADR_WIDTH + 1;i++) //读取地址检测NRF是否存在
    {
        if(NRF_MAC_ADDRESS != temp_buf[i])
            return RT_ERROR;  //读出数据与写入数据不相等,写入数据有误
    }
           /* 配置TX节点地址结束 */


           /* 配置RX节点地址 */
    temp_buf[0] = NRF_WRITE_REG+RX_ADDR_P0;     //写Rx_0节点命令
    rt_memset(temp_buf+1,NRF_MAC_ADDRESS,RX_ADR_WIDTH); // 填充地址数据
    rt_spi_send(drv->spi_dev,temp_buf,RX_ADR_WIDTH+1);//发送命令和数据
    rt_memset(temp_buf,RT_NULL,sizeof(temp_buf));          //清空缓冲区
    temp_buf[0] = NRF_READ_REG + TX_ADDR;        //读Rx节点命令
    rt_spi_send_then_recv(drv->spi_dev, temp_buf, 1, temp_buf + 1, RX_ADR_WIDTH); //读Tx节点信息,判断是否写入正确

    for(i = 1;i < RX_ADR_WIDTH + 1;i++) //读取地址检测NRF是否存在
    {
        if(NRF_MAC_ADDRESS != temp_buf[i])
            return RT_ERROR;  //读出数据与写入数据不相等,写入数据有误
    }
         /* 配置RX节点地址结束 */

           /* 配置自动应答 */
    temp_buf[0] = NRF_WRITE_REG + EN_AA;           //配置自动应答命令
    temp_buf[1] = 0x01;                            //通道0开始自动应答
    rt_spi_send(drv->spi_dev,temp_buf,2);      //发送命令和数据
    rt_memset(temp_buf,RT_NULL,sizeof(temp_buf));  //清空缓冲区
    temp_buf[0] = NRF_READ_REG + EN_AA;            //读寄存器
    rt_spi_send_then_recv(drv->spi_dev, temp_buf, 1, temp_buf + 1, 1);
          /* 配置自动应答结束 */
    
          /* 使能通道接收数据 */
    temp_buf[0] = NRF_WRITE_REG+EN_RXADDR;           
    temp_buf[1] = 0x01;                             
    rt_spi_send(drv->spi_dev,temp_buf,2);        
    rt_memset(temp_buf,RT_NULL,sizeof(temp_buf));    
    temp_buf[0] = NRF_READ_REG + EN_RXADDR;            
    rt_spi_send_then_recv(drv->spi_dev, temp_buf, 1, temp_buf + 1, 1);
         /* 使能通道接收数据结束 */
    
           /* 配置最大重发次数与间隔时间 */
    temp_buf[0] = NRF_WRITE_REG+SETUP_RETR;           
    temp_buf[1] = 0x1A;                             
    rt_spi_send(drv->spi_dev,temp_buf,2);        
    rt_memset(temp_buf,RT_NULL,sizeof(temp_buf));    
    temp_buf[0] = NRF_READ_REG + SETUP_RETR;            
    rt_spi_send_then_recv(drv->spi_dev, temp_buf, 1, temp_buf + 1, 1);
         /* 配置最大重发次数与间隔时间结束 */
         
               /* 配置发射频率 */
    temp_buf[0] = NRF_WRITE_REG+RF_CH;           
    temp_buf[1] = 0x28;                             
    rt_spi_send(drv->spi_dev,temp_buf,2);        
    rt_memset(temp_buf,RT_NULL,sizeof(temp_buf));    
    temp_buf[0] = NRF_READ_REG + RF_CH;            
    rt_spi_send_then_recv(drv->spi_dev, temp_buf, 1, temp_buf + 1, 1);
             /* 配置发射频率结束 */

               /* 配置数据宽度 */
    temp_buf[0] = NRF_WRITE_REG+RX_PW_P0;           
    temp_buf[1] = RX_PLOAD_WIDTH;                             
    rt_spi_send(drv->spi_dev,temp_buf,2);        
    rt_memset(temp_buf,RT_NULL,sizeof(temp_buf));    
    temp_buf[0] = NRF_READ_REG + RX_PW_P0;            
    rt_spi_send_then_recv(drv->spi_dev, temp_buf, 1, temp_buf + 1, 1);
             /* 配置数据宽度结束 */
             
              /* 配置发射频率与增益 */
    temp_buf[0] = NRF_WRITE_REG+RF_SETUP;
    temp_buf[1] = 0x0f;
    rt_spi_send(drv->spi_dev,temp_buf,2);
    rt_memset(temp_buf,RT_NULL,sizeof(temp_buf));
    temp_buf[0] = NRF_READ_REG + RF_SETUP;
    rt_spi_send_then_recv(drv->spi_dev, temp_buf, 1, temp_buf + 1, 1);
             /* 配置发射频率结束 */
    
              /* 配置接收模式 */
    temp_buf[0] = NRF_WRITE_REG+CONFIG;           
    temp_buf[1] = 0x0f;
    rt_spi_send(drv->spi_dev,temp_buf,2);        
    rt_memset(temp_buf,RT_NULL,sizeof(temp_buf));    
    temp_buf[0] = NRF_READ_REG + CONFIG;
    rt_spi_send_then_recv(drv->spi_dev, temp_buf, 1, temp_buf + 1, 1);
             /* 配置接收模式结束 */
    
    NRF_CE_HIGH();

    return  RT_EOK; //完成参数配置
}


/*
****************************************************************************
*函 数 名: stm32_eeprom_write
*功能说明: 写EEPROM
*形    参: 1. 设备句柄 2. 偏移量 3. 缓冲区地址 4. 缓冲区大小
*返 回 值: 写数据长度
*其    他: 函数内部调用
****************************************************************************
*/
static rt_size_t stm32_nrf_write(rt_device_t dev, rt_off_t pos, const void* buffer, rt_size_t size)
{
       char temp_buf[33];
    struct _nrf_drv * drv = (struct _nrf_drv *)dev;

    // PIN_OUT_LOW(drv->gpio_ce->gpio,drv->gpio_ce->pin);  //拉低CE引脚
    NRF_CE_LOW();
	
     /* 配置发送模式 */
    temp_buf[0] = NRF_WRITE_REG+CONFIG;
    temp_buf[1] = 0x0e;
    rt_spi_send(drv->spi_dev,temp_buf,2);//配置发送模式
    
    temp_buf[0] = NRF_READ_REG+STATUS;
    rt_spi_send_then_recv(drv->spi_dev,temp_buf,1,temp_buf+1,1);
    temp_buf[0] = NRF_WRITE_REG+STATUS;
    rt_spi_send(drv->spi_dev,temp_buf+1,1);

    temp_buf[0] = NRF_READ_REG+STATUS;
    rt_spi_send_then_recv(drv->spi_dev,temp_buf,1,temp_buf+1,1); 


    temp_buf[0] = WR_TX_PLOAD;

    rt_memcpy(temp_buf+1,buffer,size);

    rt_spi_send(drv->spi_dev,temp_buf,33);



    // PIN_OUT_HIGH(drv->gpio_ce->gpio,drv->gpio_ce->pin);  //拉高CE引脚
		NRF_CE_HIGH();

    // while(PIN_INPUT(drv->gpio_irq->gpio,drv->gpio_irq->pin));
		while(NRF_IRQ_INPUT());

    // PIN_OUT_LOW(drv->gpio_ce->gpio,drv->gpio_ce->pin);  //拉高CE引脚
		NRF_CE_LOW();
		
    temp_buf[0] = NRF_READ_REG+STATUS;
    rt_spi_send_then_recv(drv->spi_dev,temp_buf,1,temp_buf+1,1);

    temp_buf[0] = NRF_WRITE_REG+STATUS;
    rt_spi_send(drv->spi_dev,temp_buf,2);  //清除标志

    if(temp_buf[1]&MAX_TX)//达到最大重发次数
    {
        temp_buf[0] = FLUSH_TX;
        rt_spi_send(drv->spi_dev,temp_buf,1);
        
        /* 配置发送模式 */
        temp_buf[0] = NRF_WRITE_REG+CONFIG;
        temp_buf[1] = 0x0f;
        rt_spi_send(drv->spi_dev,temp_buf,2);//配置发送模式
        
        // PIN_OUT_HIGH(drv->gpio_ce->gpio,drv->gpio_ce->pin);  //拉高CE引脚
			NRF_CE_HIGH();
        
        return MAX_TX; 
    }
    
    /* 配置发送模式 */
    temp_buf[0] = NRF_WRITE_REG+CONFIG;
    temp_buf[1] = 0x0f;
    rt_spi_send(drv->spi_dev,temp_buf,2);//配置发送模式
    // PIN_OUT_HIGH(drv->gpio_ce->gpio,drv->gpio_ce->pin);  //拉高CE引脚
		NRF_CE_HIGH();
    
    return 0;
}

static rt_size_t stm32_nrf_read(rt_device_t dev, rt_off_t pos, void* buffer, rt_size_t size)
{
    int count;
    char temp_buf[33];
    struct _nrf_drv * drv = (struct _nrf_drv *)dev;
	
    
    temp_buf[0] = NRF_READ_REG+STATUS;  //读取标志
    rt_spi_send_then_recv(drv->spi_dev,temp_buf,1,temp_buf+1,1);
    
    temp_buf[0] = NRF_WRITE_REG+STATUS;    //清楚标志
    rt_spi_send(drv->spi_dev,temp_buf,2);
    
    if(temp_buf[1]&RX_OK)//接收到数据
	  {
        temp_buf[0] = RD_RX_PLOAD;
        count = rt_spi_send_then_recv(drv->spi_dev,temp_buf,1,buffer,size);
        temp_buf[0] = FLUSH_RX;
        temp_buf[1] = 0xff;
        rt_spi_send(drv->spi_dev,temp_buf,2);
	  }

    return count;
}

/*
****************************************************************************
*函 数 名: lcd_drv_init
*功能说明: LCD驱动注册
*形    参: 无
*返 回 值: 注册成功的个数
*其    他: 由INIT_DEVICE_EXPORT宏自动调用
****************************************************************************
*/
int nrf_drv_init(void)
{
    int i = 0, count = 0;
    char dev_name[sizeof(HARD_DEVICE_NAME_FORMAT) + 2] = "";
    struct _nrf_drv *dev = RT_NULL;
    
    if(RT_NULL != (ARRAY_SIZE(_hard_desc) % 2))  //检测引脚是否为三个
    {
        DBG_PRINT("lcd pin num error\r\n");
        return RT_ERROR;
    }
    
    for(i=0; i<ARRAY_SIZE(_hard_desc); )             //遍历数组，添加LCD驱动
    {
        rt_sprintf(dev_name, HARD_DEVICE_NAME_FORMAT, i); //获得完整名字
        if(rt_device_find(HARD_DEVICE_NAME_FORMAT))       //判断是否重名
        {
            i += 2; 
            continue;
        }
        
        dev = rt_malloc(sizeof(struct _nrf_drv));    //分配驱动内存
        if(RT_NULL == dev)
        {
            i += 2;
            DBG_PRINT("rt_malloc fail\r\n");
            continue;
        }
        rt_memset(dev,RT_NULL,sizeof(struct _nrf_drv));
        
        dev->mutex = rt_mutex_create(dev_name,RT_IPC_FLAG_FIFO); //创建互斥信号量
        if(RT_NULL == dev->mutex)
        {
            DBG_PRINT("mutex create fail\r\n");
            rt_free(dev);
            i += 2;
            continue;
        }

        dev->gpio_ce = &(_hard_desc[i]);     //挂载LCD引脚信息
        dev->gpio_irq  = &(_hard_desc[i+1]);   //挂在RS引脚信息
        
        dev->device.type    = RT_Device_Class_Miscellaneous;  //其他设备
        dev->device.init    = stm32_nrf_init;
        dev->device.open    = stm32_nrf_open;
        dev->device.close   = RT_NULL;
        dev->device.control = RT_NULL;
        dev->device.read    = stm32_nrf_read;
        dev->device.write   = stm32_nrf_write;
        
        dev->device.user_data = RT_NULL;
        
        rt_device_register(&dev->device, dev_name,RT_DEVICE_FLAG_RDWR);      //注册驱动
        i += 2;
        count++;

    }

    return count;
}


INIT_DEVICE_EXPORT(nrf_drv_init);






