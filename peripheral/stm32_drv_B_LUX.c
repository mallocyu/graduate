/****************************************************************************
* ��Ȩ��Ϣ��
* ϵͳ���ƣ�rt-thread-4.0.1 
* �ļ����ƣ�stm32_drv_blux.c
* �ļ�˵����blux��������
* ��    �ߣ�YCG
* �汾��Ϣ��V1.0
* ������ڣ�2019-05-26
* �޸ļ�¼��
* ��    ��      ��    ��        �޸���      �޸�ժҪ
  2019-05-26     V1.0            YCG       �������մ���������
****************************************************************************/
/***********************************************************
*ע�⣺
*    1��������δ������Ч�Բ��ԣ������ʹ�ã���
**********************************************************/
#include <rtdevice.h>
#include <rtthread.h>
#include <rthw.h>
#include "board.h"
#include "stm32_drv_b_lux.h"

#define BLUX_I2C_BUS_NAME          "i2c1"  /* ���������ӵ�I2C�����豸���� */
#define	B_LUX_V30B_SlaveAddr_w 	  0x94      //����������IIC�����ϵĴӻ���ַ  д
#define	B_LUX_V30B_SlaveAddr_r 	  0x95      //����������IIC�����ϵĴӻ���ַ  ��

/***********************************************************
˵����
    0x00 - 0x03Ϊ���Ĵ�����(�洢32λ����ֵ)
		0x04 - 0x07Ϊд�Ĵ�����(0x04Ϊ���üĴ�����0x05 - 0x07����)
		0x10 - 0x80Ϊ112�ֽڵ�EEPROM
��ע��B_LUX_V30B���õ���ģ���IIC��������SCL��SDA���ó��������

************************************************************/

#define BLUX_REG_ADDR          0x04
#define BLUX_LTGHT_SET         0x00     //�� ΢�� ģʽ������0x04�Ĵ���
#define BLUX_BRIGHT_LIGHT_SET  0x03     //�� ǿ�� ģʽ������0x04�Ĵ���

///* ������Ϣ */
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
		struct rt_device device;              /* ͨ�������ṹ�� */
	  struct rt_i2c_bus_device *iic_dev;    /* IIC�豸��� */
	  rt_mutex_t  mutex;                    /* �����ź��� */
};

/****************************************************************************
*�� �� ��: blux_write_reg
*����˵��: дmpu6050�����Ĵ���
*��    ��: 1. �Ĵ�����ַ  2.data
*�� �� ֵ: 0 ��ʾ������ 1 ��ʾ������
*��    ��: �����ڲ�����
****************************************************************************/
static rt_err_t blux_write_reg(rt_device_t dev,rt_uint8_t reg, rt_uint8_t data)
{
	  struct _blux_device * drv = (struct _blux_device *)dev;
	  
    struct rt_i2c_msg msgs;
    rt_uint8_t buf[2] = {reg, data};

    msgs.addr  = B_LUX_V30B_SlaveAddr_w;    /* �ӻ���ַ */
    msgs.flags = RT_I2C_WR;                 /* д��־ */
    msgs.buf   = buf;                       /* ��������ָ�� */
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
*�� �� ��: blux_read_reg
*����˵��: ��ȡ�Ĵ�������
*��    ��: 1. Ҫ��ȡ�ļĴ�����ַ  2.Ҫ��ȡ�������ֽ��� 3.v��ȡ�������ݴ洢��
*�� �� ֵ: 0 ��ʾ������ 1 ��ʾ������
*��    ��: �����ڲ�����
****************************************************************************/
static rt_err_t blux_read_reg(rt_device_t dev,rt_uint8_t reg, rt_uint8_t *buf, rt_uint8_t len)
{   
	  struct _blux_device * drv = (struct _blux_device *)dev;
    struct rt_i2c_msg msgs[2];

    msgs[0].addr  = B_LUX_V30B_SlaveAddr_w;    /* �ӻ���ַ */
    msgs[0].flags = RT_I2C_WR;      					 /* д��־ */
    msgs[0].buf   = &reg;           					 /* �ӻ��Ĵ�����ַ */
    msgs[0].len   = 1;               					 /* ���������ֽ��� */

    msgs[1].addr  = B_LUX_V30B_SlaveAddr_w;    /* �ӻ���ַ */
    msgs[1].flags = RT_I2C_RD;       					 /* ����־ */
    msgs[1].buf   = buf;            					 /* ��ȡ����ָ�� */
    msgs[1].len   = len;           					   /* ��ȡ�����ֽ��� */

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
*�� �� ��: blux_open
*����˵��: ʹDHT11������λ
*��    ��: 1. DHT11�豸���
*�� �� ֵ: 0 ��ʾ������ 1 ��ʾ������
*��    ��: �����ڲ�����
****************************************************************************/
static rt_err_t stm32_blux_open(rt_device_t dev, rt_uint16_t oflag)
{
	  /* ����blux��0x05-0x0f�Ŀɶ�д�Ĵ���д���ݣ���ֵ */
	  struct _blux_device * drv = (struct _blux_device *)dev;
	  rt_uint8_t result;
	
	  drv->iic_dev = (struct rt_i2c_bus_device *)rt_device_find(BLUX_I2C_BUS_NAME);
	  if(drv->iic_dev == RT_NULL)
		{
				DBG_PRINT("NOT found bus!!!\n");
			  return -1;
		}
		
		result = blux_write_reg(drv->iic_dev,BLUX_REG_ADDR,BLUX_LTGHT_SET);    //д0x04�Ĵ���  ����0x00
		if(result != RT_EOK)
		{
				DBG_PRINT("write reg fail!!\n",__FUNCTION__,__LINE__);
			  return -1;
		}
	
		
		
		return RT_EOK;
}


/****************************************************************************
*�� �� ��: _stm32_blux_read
*����˵��: ������
*��    ��: 1. �豸��� 2.�򿪱�־
*�� �� ֵ: 0 ��ʾ������ 1 ��ʾ������
*��    ��: �����ڲ�����
****************************************************************************/
static rt_size_t stm32_blux_read(rt_device_t dev,rt_off_t pos, void* buffer, rt_size_t size)
{
	  struct _blux_device *drv = (struct _blux_device *)dev;
	
		rt_uint8_t vBuf[129];                       //�������ݻ����� 	
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
	
	  temp = (float)(val32*1.4);			//����͸���Ƚ���ֵ*1.4
	
    *result = (rt_uint32_t)(temp);

	
	  return 0;
}

int blux_drv_init(void)
{
    int i = 0, count = 0;
    struct _blux_device * dev = RT_NULL;
        
		dev = (struct _blux_device *)rt_malloc(sizeof(struct _blux_device));      //�����ڴ�
		if (RT_NULL == dev)            //����ʧ�ܣ���������ѭ��
		{
				DBG_PRINT("rt_malloc fail\r\n");
        return -1;
		}
		rt_memset(dev,RT_NULL,sizeof(struct _blux_device));     //�����Ƭ�ڴ�
		
		dev->mutex =  rt_mutex_create("blux_lock",RT_IPC_FLAG_FIFO); //�����ź���
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

INIT_DEVICE_EXPORT(blux_drv_init);   //�����Զ�����
