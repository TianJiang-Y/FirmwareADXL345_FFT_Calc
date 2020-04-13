 /**
  ******************************************************************************
  * @file    bsp_xxx.c
  * @author  STMicroelectronics
  * @version V1.0
  * @date    2013-xx-xx
  * @brief   spi flash �ײ�Ӧ�ú���bsp 
  ******************************************************************************
  * @attention
  *
  * ʵ��ƽ̨:���� F103-MINI STM32 ������ 
  * ��̳    :http://www.firebbs.cn
  * �Ա�    :https://fire-stm32.taobao.com
  *
  ******************************************************************************
  */
  
#include "bsp_spi_sram.h"

static __IO uint32_t  SPITimeout = SPIT_LONG_TIMEOUT;    
static uint16_t SPI_TIMEOUT_UserCallback(uint8_t errorCode);

/**
  * @brief  SPI_FLASH��ʼ��
  * @param  ��
  * @retval ��
  */
void SPI_SRAM_Init(void)
{
//  SPI_InitTypeDef  SPI_InitStructure;
//  GPIO_InitTypeDef GPIO_InitStructure;
//	
//	/* ʹ��SPIʱ�� */
//	SRAM_SPI_APBxClock_FUN ( SRAM_SPI_CLK, ENABLE );
//	
//	/* ʹ��SPI������ص�ʱ�� */
// 	SRAM_SPI_CS_APBxClock_FUN ( SRAM_SPI_CS_CLK|SRAM_SPI_SCK_CLK|
//																	SRAM_SPI_MISO_PIN|SRAM_SPI_MOSI_PIN, ENABLE );
//	
//  /* ����SPI�� CS���ţ���ͨIO���� */
//  GPIO_InitStructure.GPIO_Pin = SRAM_SPI_CS_PIN;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//  GPIO_Init(SRAM_SPI_CS_PORT, &GPIO_InitStructure);
//	
//  /* ����SPI�� SCK����*/
//  GPIO_InitStructure.GPIO_Pin = SRAM_SPI_SCK_PIN;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
//  GPIO_Init(SRAM_SPI_SCK_PORT, &GPIO_InitStructure);

//  /* ����SPI�� MISO����*/
//  GPIO_InitStructure.GPIO_Pin = SRAM_SPI_MISO_PIN;
//  GPIO_Init(SRAM_SPI_MISO_PORT, &GPIO_InitStructure);

//  /* ����SPI�� MOSI����*/
//  GPIO_InitStructure.GPIO_Pin = SRAM_SPI_MOSI_PIN;
//  GPIO_Init(SRAM_SPI_MOSI_PORT, &GPIO_InitStructure);

//  /* ֹͣ�ź� FLASH: CS���Ÿߵ�ƽ*/
//  SPI_SRAM_CS_HIGH();

//  /* SPI ģʽ���� */
//  // FLASHоƬ ֧��SPIģʽ0��ģʽ3���ݴ�����CPOL CPHA
//  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
//  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
//  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
//  SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
//  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
//  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
//  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
//  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
//  SPI_InitStructure.SPI_CRCPolynomial = 7;
//  SPI_Init(SRAM_SPIx , &SPI_InitStructure);

//  /* ʹ�� SPI  */
//  SPI_Cmd(SRAM_SPIx , ENABLE);
	
}

 /**
  * @brief  ��FLASH��ҳд�����ݣ����ñ�����д������ǰ��Ҫ�Ȳ�������
  * @param	pBuffer��Ҫд�����ݵ�ָ��
  * @param WriteAddr��д���ַ
  * @param  NumByteToWrite��д�����ݳ��ȣ�����С�ڵ���SPI_SRAM_PerWritePageSize
  * @retval ��
  */
void SPI_SRAM_PageWrite(u8* pBuffer, u32 WriteAddr, u16 NumByteToWrite)
{

//  /* ѡ��FLASH: CS�͵�ƽ */
//  SPI_SRAM_CS_LOW();
//  /* дҳдָ��*/
//  SPI_SRAM_SendByte(LC1024_WriteMode);
//	SPI_SRAM_SendByte(MD_Page);
//	SPI_SRAM_CS_HIGH();
//  /*����д��ַ�ĸ�λ*/
//	SPI_SRAM_CS_LOW();
//	SPI_SRAM_SendByte(LC1024_Write);
//  /* дҳдָ��*/
//  SPI_SRAM_SendByte((WriteAddr & 0xFF0000) >> 16);
//  /*����д��ַ����λ*/
//  SPI_SRAM_SendByte((WriteAddr & 0xFF00) >> 8);
//  /*����д��ַ�ĵ�λ*/
//  SPI_SRAM_SendByte(WriteAddr & 0xFF);

//  if(NumByteToWrite > SPI_SRAM_PerWritePageSize)
//  {
//     NumByteToWrite = SPI_SRAM_PerWritePageSize;
//     FLASH_ERROR("SPI_SRAM_PageWrite too large!"); 
//  }

//  /* д������*/
//  while (NumByteToWrite--)
//  {
//    /* ���͵�ǰҪд����ֽ����� */
//    SPI_SRAM_SendByte(*pBuffer);
//    /* ָ����һ�ֽ����� */
//    pBuffer++;
//  }

//  /* ֹͣ�ź� FLASH: CS �ߵ�ƽ */
//  SPI_SRAM_CS_HIGH();

}

 /**
  * @brief  ��FLASHд�����ݣ����ñ�����д������ǰ��Ҫ�Ȳ�������
  * @param	pBuffer��Ҫд�����ݵ�ָ��
  * @param  WriteAddr��д���ַ
  * @param  NumByteToWrite��д�����ݳ���
  * @retval ��
  */
void SPI_SRAM_SeqnWrite(u8* pBuffer, u32 WriteAddr, u32 NumByteToWrite)
{
  u8 NumOfPage = 0, NumOfSingle = 0, Addr = 0, count = 0, temp = 0;
	
//	/* ѡ��FLASH: CS�͵�ƽ */
//  SPI_SRAM_CS_LOW();
//  /* дҳдָ��*/
//  SPI_SRAM_SendByte(LC1024_WriteMode);
//	SPI_SRAM_SendByte(MD_Seqn);
//	SPI_SRAM_CS_HIGH();
//  /*����д��ַ�ĸ�λ*/
//	SPI_SRAM_CS_LOW();
//	SPI_SRAM_SendByte(LC1024_Write);
//  /* дҳдָ��*/
//  SPI_SRAM_SendByte((WriteAddr & 0xFF0000) >> 16);
//  /*����д��ַ����λ*/
//  SPI_SRAM_SendByte((WriteAddr & 0xFF00) >> 8);
//  /*����д��ַ�ĵ�λ*/
//  SPI_SRAM_SendByte(WriteAddr & 0xFF);

//  if(NumByteToWrite > 0x1FFFF)
//  {
//     NumByteToWrite = SPI_SRAM_PerWritePageSize;
//     FLASH_ERROR("SPI_SRAM_SeqnWrite too large!"); 
//  }

//  /* д������*/
//  while (NumByteToWrite--)
//  {
//    /* ���͵�ǰҪд����ֽ����� */
//    SPI_SRAM_SendByte(*pBuffer);
//    /* ָ����һ�ֽ����� */
//    pBuffer++;
//  }

//  /* ֹͣ�ź� FLASH: CS �ߵ�ƽ */
//  SPI_SRAM_CS_HIGH();
}

 /**
  * @brief  ��ȡFLASH����
  * @param 	pBuffer���洢�������ݵ�ָ��
  * @param   ReadAddr����ȡ��ַ
  * @param   NumByteToRead����ȡ���ݳ���
  * @retval ��
  */
void SPI_SRAM_SeqnRead(u8* pBuffer, u32 ReadAddr, u32 NumByteToRead)
{
	
//		/* ѡ��FLASH: CS�͵�ƽ */
//  SPI_SRAM_CS_LOW();
//  /* дҳдָ��*/
//  SPI_SRAM_SendByte(LC1024_WriteMode);
//	SPI_SRAM_SendByte(MD_Seqn);
//	SPI_SRAM_CS_HIGH();
//	
//  /* ѡ��FLASH: CS�͵�ƽ */
//  SPI_SRAM_CS_LOW();

//  /* ���� �� ָ�� */
//  SPI_SRAM_SendByte(LC1024_ReadData);

//  /* ���� �� ��ַ��λ */
//  SPI_SRAM_SendByte((ReadAddr & 0xFF0000) >> 16);
//  /* ���� �� ��ַ��λ */
//  SPI_SRAM_SendByte((ReadAddr& 0xFF00) >> 8);
//  /* ���� �� ��ַ��λ */
//  SPI_SRAM_SendByte(ReadAddr & 0xFF);
//	
//	/* ��ȡ���� */
//  while (NumByteToRead--) /* while there is data to be read */
//  {
//    /* ��ȡһ���ֽ�*/
//    *pBuffer = SPI_SRAM_SendByte(Dummy_Byte);
//    /* ָ����һ���ֽڻ����� */
//    pBuffer++;
//  }

//  /* ֹͣ�ź� FLASH: CS �ߵ�ƽ */
//  SPI_SRAM_CS_HIGH();
}



 /**
  * @brief  ʹ��SPI��ȡһ���ֽڵ�����
  * @param  ��
  * @retval ���ؽ��յ�������
  */
u8 SPI_SRAM_ReadByte(void)
{
  return (SPI_SRAM_SendByte(Dummy_Byte));
}

 /**
  * @brief  ʹ��SPI����һ���ֽڵ�����
  * @param  byte��Ҫ���͵�����
  * @retval ���ؽ��յ�������
  */
u8 SPI_SRAM_SendByte(u8 byte)
{
	 SPITimeout = SPIT_FLAG_TIMEOUT;
//  /* �ȴ����ͻ�����Ϊ�գ�TXE�¼� */
//  while (SPI_I2S_GetFlagStatus(SRAM_SPIx , SPI_I2S_FLAG_TXE) == RESET)
//	{
//    if((SPITimeout--) == 0) return SPI_TIMEOUT_UserCallback(0);
//   }

//  /* д�����ݼĴ�������Ҫд�������д�뷢�ͻ����� */
//  SPI_I2S_SendData(SRAM_SPIx , byte);

//	SPITimeout = SPIT_FLAG_TIMEOUT;
//  /* �ȴ����ջ������ǿգ�RXNE�¼� */
//  while (SPI_I2S_GetFlagStatus(SRAM_SPIx , SPI_I2S_FLAG_RXNE) == RESET)
//  {
//    if((SPITimeout--) == 0) return SPI_TIMEOUT_UserCallback(1);
//   }

//  /* ��ȡ���ݼĴ�������ȡ���ջ��������� */
//  return SPI_I2S_ReceiveData(SRAM_SPIx );
}


/**
  * @brief  �ȴ���ʱ�ص�����
  * @param  None.
  * @retval None.
  */
static  uint16_t SPI_TIMEOUT_UserCallback(uint8_t errorCode)
{
  /* �ȴ���ʱ��Ĵ���,���������Ϣ */
  FLASH_ERROR("SPI �ȴ���ʱ!errorCode = %d",errorCode);
  return 0;
}
   
/*********************************************END OF FILE**********************/
