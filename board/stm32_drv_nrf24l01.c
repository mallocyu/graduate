/****************************************************************************
* ��Ȩ��Ϣ��
* ϵͳ���ƣ�rt-thread-2.1.0 
* �ļ����ƣ�stm32_drv_nrf24l01.c
* �ļ�˵����NRF24L01��������
* ��    �ߣ�TYX
* �汾��Ϣ��V1.0
* ������ڣ�2017-04-26
* �޸ļ�¼��
* ��    ��      ��    ��        �޸���      �޸�ժҪ
  2017-04-26     V1.0            TYX       ����NRF����
	2019-05-05     V1.1            YCG       �޸�NRF������������
****************************************************************************/
/****************************************************************************
*1.��֧�ֶ���Ļ�ͺ�
*2.����SPI����û�б��,�ܱ�
****************************************************************************/

#include "rtdevice.h"
#include <rtthread.h>
#include "stm32_drv_nrf24l01.h"

/****************************************************************************
* �û���֪
* 1.����:   
*   1> �Զ������֣����������е������ظ�
* 2.�ź��ߣ� NRFʹ��SPI����ͨ��
* 3.������Ϣ����ȥSPI��������,�����Զ��������ź���
*   1> ����ʾ�����õ������Զ������ţ�
*      1 24L01Ƭѡ�ź�
*����  2 SPIƬѡ�ź���
*    �����ߵ�˳�����ң�һ���������ӵ�˳��
* 3.����ע�ắ�����Զ����ã������ֶ�����
* 4.SPI�����豸,��Ҫע��һ������SPI����,����ʵ��������,�������߹�99��
* 5.�ܷ�RT SPI����  ������� ����ע�����������
****************************************************************************/


/* �������� */
#define HARD_DEVICE_NAME_FORMAT        "nrf_%d"

/* ������Ϣ */
const static gpio_desc _hard_desc[] =
{
    {GPIOG, 6},   //CE
    {GPIOG, 8},   //IRQ
};


/***************************************************************************
*                                �ָ���                                    *
***************************************************************************/
/* ���PIN */
#define get_st_pin(gpio_pin) (0x01 << (gpio_pin&0xFF))

#define SPI_DVICES_NAME                 "spi10"

#ifdef  NRF_DBG
#define DBG_PRINT(fmt, ...)   rt_kprintf("NRF_DBG:" fmt, ##__VA_ARGS__)
#else
#define DBG_PRINT(fmt, ...)
#endif


/* �������Ԫ�ظ��� */
#ifndef ARRAY_SIZE
#define ARRAY_SIZE(arr)     (sizeof(arr)/sizeof(arr[0]))
#endif


/* ����SPI�������� */
#define SPI_VR_NAME_FORMAT             "spi_vr%d"

//////////////////////////////////////////////////////////////////////////////////////////////////////////
//NRF24L01�Ĵ�����������
#define NRF_READ_REG    0x00  //�����üĴ���,��5λΪ�Ĵ�����ַ
#define NRF_WRITE_REG   0x20  //д���üĴ���,��5λΪ�Ĵ�����ַ
#define RD_RX_PLOAD     0x61  //��RX��Ч����,1~32�ֽ�
#define WR_TX_PLOAD     0xA0  //дTX��Ч����,1~32�ֽ�
#define FLUSH_TX        0xE1  //���TX FIFO�Ĵ���.����ģʽ����
#define FLUSH_RX        0xE2  //���RX FIFO�Ĵ���.����ģʽ����
#define REUSE_TX_PL     0xE3  //����ʹ����һ������,CEΪ��,���ݰ������Ϸ���.
#define NOP             0xFF  //�ղ���,����������״̬�Ĵ���	 
//SPI(NRF24L01)�Ĵ�����ַ
#define CONFIG          0x00  //���üĴ�����ַ;bit0:1����ģʽ,0����ģʽ;bit1:��ѡ��;bit2:CRCģʽ;bit3:CRCʹ��;
                              //bit4:�ж�MAX_RT(�ﵽ����ط������ж�)ʹ��;bit5:�ж�TX_DSʹ��;bit6:�ж�RX_DRʹ��
#define EN_AA           0x01  //ʹ���Զ�Ӧ����  bit0~5,��Ӧͨ��0~5
#define EN_RXADDR       0x02  //���յ�ַ����,bit0~5,��Ӧͨ��0~5
#define SETUP_AW        0x03  //���õ�ַ���(��������ͨ��):bit1,0:00,3�ֽ�;01,4�ֽ�;02,5�ֽ�;
#define SETUP_RETR      0x04  //�����Զ��ط�;bit3:0,�Զ��ط�������;bit7:4,�Զ��ط���ʱ 250*x+86us
#define RF_CH           0x05  //RFͨ��,bit6:0,����ͨ��Ƶ��;
#define RF_SETUP        0x06  //RF�Ĵ���;bit3:��������(0:1Mbps,1:2Mbps);bit2:1,���书��;bit0:�������Ŵ�������
#define STATUS          0x07  //״̬�Ĵ���;bit0:TX FIFO����־;bit3:1,��������ͨ����(���:6);bit4,�ﵽ�����ط�
                              //bit5:���ݷ�������ж�;bit6:���������ж�;
#define MAX_TX  		0x10  //�ﵽ����ʹ����ж�
#define TX_OK   		0x20  //TX��������ж�
#define RX_OK   		0x40  //���յ������ж�

#define OBSERVE_TX      0x08  //���ͼ��Ĵ���,bit7:4,���ݰ���ʧ������;bit3:0,�ط�������
#define CD              0x09  //�ز����Ĵ���,bit0,�ز����;
#define RX_ADDR_P0      0x0A  //����ͨ��0���յ�ַ,��󳤶�5���ֽ�,���ֽ���ǰ
#define RX_ADDR_P1      0x0B  //����ͨ��1���յ�ַ,��󳤶�5���ֽ�,���ֽ���ǰ
#define RX_ADDR_P2      0x0C  //����ͨ��2���յ�ַ,����ֽڿ�����,���ֽ�,����ͬRX_ADDR_P1[39:8]���;
#define RX_ADDR_P3      0x0D  //����ͨ��3���յ�ַ,����ֽڿ�����,���ֽ�,����ͬRX_ADDR_P1[39:8]���;
#define RX_ADDR_P4      0x0E  //����ͨ��4���յ�ַ,����ֽڿ�����,���ֽ�,����ͬRX_ADDR_P1[39:8]���;
#define RX_ADDR_P5      0x0F  //����ͨ��5���յ�ַ,����ֽڿ�����,���ֽ�,����ͬRX_ADDR_P1[39:8]���;
#define TX_ADDR         0x10  //���͵�ַ(���ֽ���ǰ),ShockBurstTMģʽ��,RX_ADDR_P0��˵�ַ���
#define RX_PW_P0        0x11  //��������ͨ��0��Ч���ݿ��(1~32�ֽ�),����Ϊ0��Ƿ�
#define RX_PW_P1        0x12  //��������ͨ��1��Ч���ݿ��(1~32�ֽ�),����Ϊ0��Ƿ�
#define RX_PW_P2        0x13  //��������ͨ��2��Ч���ݿ��(1~32�ֽ�),����Ϊ0��Ƿ�
#define RX_PW_P3        0x14  //��������ͨ��3��Ч���ݿ��(1~32�ֽ�),����Ϊ0��Ƿ�
#define RX_PW_P4        0x15  //��������ͨ��4��Ч���ݿ��(1~32�ֽ�),����Ϊ0��Ƿ�
#define RX_PW_P5        0x16  //��������ͨ��5��Ч���ݿ��(1~32�ֽ�),����Ϊ0��Ƿ�
#define NRF_FIFO_STATUS 0x17  //FIFO״̬�Ĵ���;bit0,RX FIFO�Ĵ����ձ�־;bit1,RX FIFO����־;bit2,3,����
                              //bit4,TX FIFO�ձ�־;bit5,TX FIFO����־;bit6,1,ѭ��������һ���ݰ�.0,��ѭ��;
//////////////////////////////////////////////////////////////////////////////////////////////////////////
//24L01���ͽ������ݿ�ȶ���
#define TX_ADR_WIDTH    5   	//5�ֽڵĵ�ַ���
#define RX_ADR_WIDTH    5   	//5�ֽڵĵ�ַ���
#define TX_PLOAD_WIDTH  32  	//32�ֽڵ��û����ݿ��
#define RX_PLOAD_WIDTH  32  	//32�ֽڵ��û����ݿ��
//24L01���ͽ��ܵ�ַ����
#define NRF_MAC_ADDRESS 0x4A

/* NRF��Ϣ�ṹ�� */
struct _nrf_info
{
    uint8_t tx_address[TX_ADR_WIDTH];  /* ���͵�ַ */
    uint8_t rx_address[TX_ADR_WIDTH];  /* ���յ�ַ */
    
};

/* NRF�����ṹ�� */
struct _nrf_drv
{
    struct rt_device device;      /* ͨ�������ṹ�� */
    struct rt_spi_device *spi_dev;/* SPI���� */
    rt_mutex_t  mutex;            /* �����ź��� */
    const gpio_desc * gpio_ce;    /* CE������Ϣ */
    const gpio_desc * gpio_irq;   /* IRQ������Ϣ */
};

/*
****************************************************************************
*�� �� ��: stm32_nrf_init
*����˵��: ������ʼ��
*��    ��: 1. �豸���
*�� �� ֵ: 0 ��ʾ������ 1 ��ʾ������
*��    ��: �����ڲ�����,��ʼ����Ӧ��IO����
****************************************************************************
*/
static rt_err_t stm32_nrf_init(rt_device_t dev)
{
    GPIO_InitTypeDef  GPIO_InitStruct;
    struct _nrf_drv * drv = (struct _nrf_drv *)dev;
    
		NRF_GPIO_CLK_ENABLE();    //init GPIOG clock
    
    GPIO_InitStruct.Pin = get_st_pin(drv->gpio_ce->pin);	       //ѡ������
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; 		             //��Ϊ�������ģʽ
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;           //IO������ٶ�
    HAL_GPIO_Init(drv->gpio_ce->gpio, &GPIO_InitStruct);         //��ʼ��IO��
    
    GPIO_InitStruct.Pin = get_st_pin(drv->gpio_irq->pin);	       //ѡ������
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT; 	                   //��Ϊ��������
		GPIO_InitStruct.Pull = GPIO_PULLUP;                          //���� ����
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;           //IO������ٶ�
    HAL_GPIO_Init(drv->gpio_irq->gpio, &GPIO_InitStruct);        //��ʼ��IO��
    
    HAL_GPIO_WritePin(drv->gpio_ce->gpio,get_st_pin(drv->gpio_ce->pin),GPIO_PIN_RESET);      //����

    return RT_EOK;
}

/*
****************************************************************************
*�� �� ��: stm32_nrf_open
*����˵��: ������
*��    ��: 1. �豸��� 2.�򿪱�־
*�� �� ֵ: 0 ��ʾ������ 1 ��ʾ������
*��    ��: �����ڲ�����
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
        /* ע�⣡�������� û���ҵ����ߣ�������  */
        DBG_PRINT("not find spi device, name:%s\n", SPI_DVICES_NAME);
        return -1;
    }
	  NRF_CE_LOW();
    
    cfg.data_width = 8;             //��������SPI�������ݿ��
    cfg.mode = RT_SPI_MODE_0 | RT_SPI_MSB; //SPIģʽ��
    cfg.max_hz = 10 * 1000 * 1000;   //SPI�ٶ�  9M
    
    rt_spi_configure(drv->spi_dev, &cfg);
    
              /* ����TX�ڵ��ַ */
    temp_buf[0] = NRF_WRITE_REG+TX_ADDR;  //дTx�ڵ�����
    rt_memset(temp_buf+1,NRF_MAC_ADDRESS,TX_ADR_WIDTH);    //����ַ����
    rt_spi_send(drv->spi_dev,temp_buf,TX_ADR_WIDTH+1); //�������������
    rt_memset(temp_buf,RT_NULL,sizeof(temp_buf));          //��ջ�����
    temp_buf[0] = NRF_READ_REG + TX_ADDR;                  //��Tx�ڵ�����
    rt_spi_send_then_recv(drv->spi_dev, temp_buf, 1, temp_buf + 1, TX_ADR_WIDTH); //�������� ��ȡ����

    for(i = 1;i < TX_ADR_WIDTH + 1;i++) //��ȡ��ַ���NRF�Ƿ����
    {
        if(NRF_MAC_ADDRESS != temp_buf[i])
            return RT_ERROR;  //����������д�����ݲ����,д����������
    }
           /* ����TX�ڵ��ַ���� */


           /* ����RX�ڵ��ַ */
    temp_buf[0] = NRF_WRITE_REG+RX_ADDR_P0;     //дRx_0�ڵ�����
    rt_memset(temp_buf+1,NRF_MAC_ADDRESS,RX_ADR_WIDTH); // ����ַ����
    rt_spi_send(drv->spi_dev,temp_buf,RX_ADR_WIDTH+1);//�������������
    rt_memset(temp_buf,RT_NULL,sizeof(temp_buf));          //��ջ�����
    temp_buf[0] = NRF_READ_REG + TX_ADDR;        //��Rx�ڵ�����
    rt_spi_send_then_recv(drv->spi_dev, temp_buf, 1, temp_buf + 1, RX_ADR_WIDTH); //��Tx�ڵ���Ϣ,�ж��Ƿ�д����ȷ

    for(i = 1;i < RX_ADR_WIDTH + 1;i++) //��ȡ��ַ���NRF�Ƿ����
    {
        if(NRF_MAC_ADDRESS != temp_buf[i])
            return RT_ERROR;  //����������д�����ݲ����,д����������
    }
         /* ����RX�ڵ��ַ���� */

           /* �����Զ�Ӧ�� */
    temp_buf[0] = NRF_WRITE_REG + EN_AA;           //�����Զ�Ӧ������
    temp_buf[1] = 0x01;                            //ͨ��0��ʼ�Զ�Ӧ��
    rt_spi_send(drv->spi_dev,temp_buf,2);      //�������������
    rt_memset(temp_buf,RT_NULL,sizeof(temp_buf));  //��ջ�����
    temp_buf[0] = NRF_READ_REG + EN_AA;            //���Ĵ���
    rt_spi_send_then_recv(drv->spi_dev, temp_buf, 1, temp_buf + 1, 1);
          /* �����Զ�Ӧ����� */
    
          /* ʹ��ͨ���������� */
    temp_buf[0] = NRF_WRITE_REG+EN_RXADDR;           
    temp_buf[1] = 0x01;                             
    rt_spi_send(drv->spi_dev,temp_buf,2);        
    rt_memset(temp_buf,RT_NULL,sizeof(temp_buf));    
    temp_buf[0] = NRF_READ_REG + EN_RXADDR;            
    rt_spi_send_then_recv(drv->spi_dev, temp_buf, 1, temp_buf + 1, 1);
         /* ʹ��ͨ���������ݽ��� */
    
           /* ��������ط���������ʱ�� */
    temp_buf[0] = NRF_WRITE_REG+SETUP_RETR;           
    temp_buf[1] = 0x1A;                             
    rt_spi_send(drv->spi_dev,temp_buf,2);        
    rt_memset(temp_buf,RT_NULL,sizeof(temp_buf));    
    temp_buf[0] = NRF_READ_REG + SETUP_RETR;            
    rt_spi_send_then_recv(drv->spi_dev, temp_buf, 1, temp_buf + 1, 1);
         /* ��������ط���������ʱ����� */
         
               /* ���÷���Ƶ�� */
    temp_buf[0] = NRF_WRITE_REG+RF_CH;           
    temp_buf[1] = 0x28;                             
    rt_spi_send(drv->spi_dev,temp_buf,2);        
    rt_memset(temp_buf,RT_NULL,sizeof(temp_buf));    
    temp_buf[0] = NRF_READ_REG + RF_CH;            
    rt_spi_send_then_recv(drv->spi_dev, temp_buf, 1, temp_buf + 1, 1);
             /* ���÷���Ƶ�ʽ��� */

               /* �������ݿ�� */
    temp_buf[0] = NRF_WRITE_REG+RX_PW_P0;           
    temp_buf[1] = RX_PLOAD_WIDTH;                             
    rt_spi_send(drv->spi_dev,temp_buf,2);        
    rt_memset(temp_buf,RT_NULL,sizeof(temp_buf));    
    temp_buf[0] = NRF_READ_REG + RX_PW_P0;            
    rt_spi_send_then_recv(drv->spi_dev, temp_buf, 1, temp_buf + 1, 1);
             /* �������ݿ�Ƚ��� */
             
              /* ���÷���Ƶ�������� */
    temp_buf[0] = NRF_WRITE_REG+RF_SETUP;
    temp_buf[1] = 0x0f;
    rt_spi_send(drv->spi_dev,temp_buf,2);
    rt_memset(temp_buf,RT_NULL,sizeof(temp_buf));
    temp_buf[0] = NRF_READ_REG + RF_SETUP;
    rt_spi_send_then_recv(drv->spi_dev, temp_buf, 1, temp_buf + 1, 1);
             /* ���÷���Ƶ�ʽ��� */
    
              /* ���ý���ģʽ */
    temp_buf[0] = NRF_WRITE_REG+CONFIG;           
    temp_buf[1] = 0x0f;
    rt_spi_send(drv->spi_dev,temp_buf,2);        
    rt_memset(temp_buf,RT_NULL,sizeof(temp_buf));    
    temp_buf[0] = NRF_READ_REG + CONFIG;
    rt_spi_send_then_recv(drv->spi_dev, temp_buf, 1, temp_buf + 1, 1);
             /* ���ý���ģʽ���� */
    
    NRF_CE_HIGH();

    return  RT_EOK; //��ɲ�������
}


/*
****************************************************************************
*�� �� ��: stm32_eeprom_write
*����˵��: дEEPROM
*��    ��: 1. �豸��� 2. ƫ���� 3. ��������ַ 4. ��������С
*�� �� ֵ: д���ݳ���
*��    ��: �����ڲ�����
****************************************************************************
*/
static rt_size_t stm32_nrf_write(rt_device_t dev, rt_off_t pos, const void* buffer, rt_size_t size)
{
       char temp_buf[33];
    struct _nrf_drv * drv = (struct _nrf_drv *)dev;

    // PIN_OUT_LOW(drv->gpio_ce->gpio,drv->gpio_ce->pin);  //����CE����
    NRF_CE_LOW();
	
     /* ���÷���ģʽ */
    temp_buf[0] = NRF_WRITE_REG+CONFIG;
    temp_buf[1] = 0x0e;
    rt_spi_send(drv->spi_dev,temp_buf,2);//���÷���ģʽ
    
    temp_buf[0] = NRF_READ_REG+STATUS;
    rt_spi_send_then_recv(drv->spi_dev,temp_buf,1,temp_buf+1,1);
    temp_buf[0] = NRF_WRITE_REG+STATUS;
    rt_spi_send(drv->spi_dev,temp_buf+1,1);

    temp_buf[0] = NRF_READ_REG+STATUS;
    rt_spi_send_then_recv(drv->spi_dev,temp_buf,1,temp_buf+1,1); 


    temp_buf[0] = WR_TX_PLOAD;

    rt_memcpy(temp_buf+1,buffer,size);

    rt_spi_send(drv->spi_dev,temp_buf,33);



    // PIN_OUT_HIGH(drv->gpio_ce->gpio,drv->gpio_ce->pin);  //����CE����
		NRF_CE_HIGH();

    // while(PIN_INPUT(drv->gpio_irq->gpio,drv->gpio_irq->pin));
		while(NRF_IRQ_INPUT());

    // PIN_OUT_LOW(drv->gpio_ce->gpio,drv->gpio_ce->pin);  //����CE����
		NRF_CE_LOW();
		
    temp_buf[0] = NRF_READ_REG+STATUS;
    rt_spi_send_then_recv(drv->spi_dev,temp_buf,1,temp_buf+1,1);

    temp_buf[0] = NRF_WRITE_REG+STATUS;
    rt_spi_send(drv->spi_dev,temp_buf,2);  //�����־

    if(temp_buf[1]&MAX_TX)//�ﵽ����ط�����
    {
        temp_buf[0] = FLUSH_TX;
        rt_spi_send(drv->spi_dev,temp_buf,1);
        
        /* ���÷���ģʽ */
        temp_buf[0] = NRF_WRITE_REG+CONFIG;
        temp_buf[1] = 0x0f;
        rt_spi_send(drv->spi_dev,temp_buf,2);//���÷���ģʽ
        
        // PIN_OUT_HIGH(drv->gpio_ce->gpio,drv->gpio_ce->pin);  //����CE����
			NRF_CE_HIGH();
        
        return MAX_TX; 
    }
    
    /* ���÷���ģʽ */
    temp_buf[0] = NRF_WRITE_REG+CONFIG;
    temp_buf[1] = 0x0f;
    rt_spi_send(drv->spi_dev,temp_buf,2);//���÷���ģʽ
    // PIN_OUT_HIGH(drv->gpio_ce->gpio,drv->gpio_ce->pin);  //����CE����
		NRF_CE_HIGH();
    
    return 0;
}

static rt_size_t stm32_nrf_read(rt_device_t dev, rt_off_t pos, void* buffer, rt_size_t size)
{
    int count;
    char temp_buf[33];
    struct _nrf_drv * drv = (struct _nrf_drv *)dev;
	
    
    temp_buf[0] = NRF_READ_REG+STATUS;  //��ȡ��־
    rt_spi_send_then_recv(drv->spi_dev,temp_buf,1,temp_buf+1,1);
    
    temp_buf[0] = NRF_WRITE_REG+STATUS;    //�����־
    rt_spi_send(drv->spi_dev,temp_buf,2);
    
    if(temp_buf[1]&RX_OK)//���յ�����
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
*�� �� ��: lcd_drv_init
*����˵��: LCD����ע��
*��    ��: ��
*�� �� ֵ: ע��ɹ��ĸ���
*��    ��: ��INIT_DEVICE_EXPORT���Զ�����
****************************************************************************
*/
int nrf_drv_init(void)
{
    int i = 0, count = 0;
    char dev_name[sizeof(HARD_DEVICE_NAME_FORMAT) + 2] = "";
    struct _nrf_drv *dev = RT_NULL;
    
    if(RT_NULL != (ARRAY_SIZE(_hard_desc) % 2))  //��������Ƿ�Ϊ����
    {
        DBG_PRINT("lcd pin num error\r\n");
        return RT_ERROR;
    }
    
    for(i=0; i<ARRAY_SIZE(_hard_desc); )             //�������飬���LCD����
    {
        rt_sprintf(dev_name, HARD_DEVICE_NAME_FORMAT, i); //�����������
        if(rt_device_find(HARD_DEVICE_NAME_FORMAT))       //�ж��Ƿ�����
        {
            i += 2; 
            continue;
        }
        
        dev = rt_malloc(sizeof(struct _nrf_drv));    //���������ڴ�
        if(RT_NULL == dev)
        {
            i += 2;
            DBG_PRINT("rt_malloc fail\r\n");
            continue;
        }
        rt_memset(dev,RT_NULL,sizeof(struct _nrf_drv));
        
        dev->mutex = rt_mutex_create(dev_name,RT_IPC_FLAG_FIFO); //���������ź���
        if(RT_NULL == dev->mutex)
        {
            DBG_PRINT("mutex create fail\r\n");
            rt_free(dev);
            i += 2;
            continue;
        }

        dev->gpio_ce = &(_hard_desc[i]);     //����LCD������Ϣ
        dev->gpio_irq  = &(_hard_desc[i+1]);   //����RS������Ϣ
        
        dev->device.type    = RT_Device_Class_Miscellaneous;  //�����豸
        dev->device.init    = stm32_nrf_init;
        dev->device.open    = stm32_nrf_open;
        dev->device.close   = RT_NULL;
        dev->device.control = RT_NULL;
        dev->device.read    = stm32_nrf_read;
        dev->device.write   = stm32_nrf_write;
        
        dev->device.user_data = RT_NULL;
        
        rt_device_register(&dev->device, dev_name,RT_DEVICE_FLAG_RDWR);      //ע������
        i += 2;
        count++;

    }

    return count;
}


INIT_DEVICE_EXPORT(nrf_drv_init);






