 /**
  ******************************************************************************
  * @file    bsp_xxx.c
  * @author  STMicroelectronics
  * @version V1.0
  * @date    2013-xx-xx
  * @brief   spi flash 底层应用函数bsp 
  ******************************************************************************
  * @attention
  *
  * 实验平台:秉火 F103-MINI STM32 开发板 
  * 论坛    :http://www.firebbs.cn
  * 淘宝    :https://fire-stm32.taobao.com
  *
  ******************************************************************************
  */
  
#include "bsp_spi_sram.h"

static __IO uint32_t  SPITimeout = SPIT_LONG_TIMEOUT;    
static uint16_t SPI_TIMEOUT_UserCallback(uint8_t errorCode);

/**
  * @brief  SPI_FLASH初始化
  * @param  无
  * @retval 无
  */
void SPI_SRAM_Init(void)
{
//  SPI_InitTypeDef  SPI_InitStructure;
//  GPIO_InitTypeDef GPIO_InitStructure;
//	
//	/* 使能SPI时钟 */
//	SRAM_SPI_APBxClock_FUN ( SRAM_SPI_CLK, ENABLE );
//	
//	/* 使能SPI引脚相关的时钟 */
// 	SRAM_SPI_CS_APBxClock_FUN ( SRAM_SPI_CS_CLK|SRAM_SPI_SCK_CLK|
//																	SRAM_SPI_MISO_PIN|SRAM_SPI_MOSI_PIN, ENABLE );
//	
//  /* 配置SPI的 CS引脚，普通IO即可 */
//  GPIO_InitStructure.GPIO_Pin = SRAM_SPI_CS_PIN;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//  GPIO_Init(SRAM_SPI_CS_PORT, &GPIO_InitStructure);
//	
//  /* 配置SPI的 SCK引脚*/
//  GPIO_InitStructure.GPIO_Pin = SRAM_SPI_SCK_PIN;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
//  GPIO_Init(SRAM_SPI_SCK_PORT, &GPIO_InitStructure);

//  /* 配置SPI的 MISO引脚*/
//  GPIO_InitStructure.GPIO_Pin = SRAM_SPI_MISO_PIN;
//  GPIO_Init(SRAM_SPI_MISO_PORT, &GPIO_InitStructure);

//  /* 配置SPI的 MOSI引脚*/
//  GPIO_InitStructure.GPIO_Pin = SRAM_SPI_MOSI_PIN;
//  GPIO_Init(SRAM_SPI_MOSI_PORT, &GPIO_InitStructure);

//  /* 停止信号 FLASH: CS引脚高电平*/
//  SPI_SRAM_CS_HIGH();

//  /* SPI 模式配置 */
//  // FLASH芯片 支持SPI模式0及模式3，据此设置CPOL CPHA
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

//  /* 使能 SPI  */
//  SPI_Cmd(SRAM_SPIx , ENABLE);
	
}

 /**
  * @brief  对FLASH按页写入数据，调用本函数写入数据前需要先擦除扇区
  * @param	pBuffer，要写入数据的指针
  * @param WriteAddr，写入地址
  * @param  NumByteToWrite，写入数据长度，必须小于等于SPI_SRAM_PerWritePageSize
  * @retval 无
  */
void SPI_SRAM_PageWrite(u8* pBuffer, u32 WriteAddr, u16 NumByteToWrite)
{

//  /* 选择FLASH: CS低电平 */
//  SPI_SRAM_CS_LOW();
//  /* 写页写指令*/
//  SPI_SRAM_SendByte(LC1024_WriteMode);
//	SPI_SRAM_SendByte(MD_Page);
//	SPI_SRAM_CS_HIGH();
//  /*发送写地址的高位*/
//	SPI_SRAM_CS_LOW();
//	SPI_SRAM_SendByte(LC1024_Write);
//  /* 写页写指令*/
//  SPI_SRAM_SendByte((WriteAddr & 0xFF0000) >> 16);
//  /*发送写地址的中位*/
//  SPI_SRAM_SendByte((WriteAddr & 0xFF00) >> 8);
//  /*发送写地址的低位*/
//  SPI_SRAM_SendByte(WriteAddr & 0xFF);

//  if(NumByteToWrite > SPI_SRAM_PerWritePageSize)
//  {
//     NumByteToWrite = SPI_SRAM_PerWritePageSize;
//     FLASH_ERROR("SPI_SRAM_PageWrite too large!"); 
//  }

//  /* 写入数据*/
//  while (NumByteToWrite--)
//  {
//    /* 发送当前要写入的字节数据 */
//    SPI_SRAM_SendByte(*pBuffer);
//    /* 指向下一字节数据 */
//    pBuffer++;
//  }

//  /* 停止信号 FLASH: CS 高电平 */
//  SPI_SRAM_CS_HIGH();

}

 /**
  * @brief  对FLASH写入数据，调用本函数写入数据前需要先擦除扇区
  * @param	pBuffer，要写入数据的指针
  * @param  WriteAddr，写入地址
  * @param  NumByteToWrite，写入数据长度
  * @retval 无
  */
void SPI_SRAM_SeqnWrite(u8* pBuffer, u32 WriteAddr, u32 NumByteToWrite)
{
  u8 NumOfPage = 0, NumOfSingle = 0, Addr = 0, count = 0, temp = 0;
	
//	/* 选择FLASH: CS低电平 */
//  SPI_SRAM_CS_LOW();
//  /* 写页写指令*/
//  SPI_SRAM_SendByte(LC1024_WriteMode);
//	SPI_SRAM_SendByte(MD_Seqn);
//	SPI_SRAM_CS_HIGH();
//  /*发送写地址的高位*/
//	SPI_SRAM_CS_LOW();
//	SPI_SRAM_SendByte(LC1024_Write);
//  /* 写页写指令*/
//  SPI_SRAM_SendByte((WriteAddr & 0xFF0000) >> 16);
//  /*发送写地址的中位*/
//  SPI_SRAM_SendByte((WriteAddr & 0xFF00) >> 8);
//  /*发送写地址的低位*/
//  SPI_SRAM_SendByte(WriteAddr & 0xFF);

//  if(NumByteToWrite > 0x1FFFF)
//  {
//     NumByteToWrite = SPI_SRAM_PerWritePageSize;
//     FLASH_ERROR("SPI_SRAM_SeqnWrite too large!"); 
//  }

//  /* 写入数据*/
//  while (NumByteToWrite--)
//  {
//    /* 发送当前要写入的字节数据 */
//    SPI_SRAM_SendByte(*pBuffer);
//    /* 指向下一字节数据 */
//    pBuffer++;
//  }

//  /* 停止信号 FLASH: CS 高电平 */
//  SPI_SRAM_CS_HIGH();
}

 /**
  * @brief  读取FLASH数据
  * @param 	pBuffer，存储读出数据的指针
  * @param   ReadAddr，读取地址
  * @param   NumByteToRead，读取数据长度
  * @retval 无
  */
void SPI_SRAM_SeqnRead(u8* pBuffer, u32 ReadAddr, u32 NumByteToRead)
{
	
//		/* 选择FLASH: CS低电平 */
//  SPI_SRAM_CS_LOW();
//  /* 写页写指令*/
//  SPI_SRAM_SendByte(LC1024_WriteMode);
//	SPI_SRAM_SendByte(MD_Seqn);
//	SPI_SRAM_CS_HIGH();
//	
//  /* 选择FLASH: CS低电平 */
//  SPI_SRAM_CS_LOW();

//  /* 发送 读 指令 */
//  SPI_SRAM_SendByte(LC1024_ReadData);

//  /* 发送 读 地址高位 */
//  SPI_SRAM_SendByte((ReadAddr & 0xFF0000) >> 16);
//  /* 发送 读 地址中位 */
//  SPI_SRAM_SendByte((ReadAddr& 0xFF00) >> 8);
//  /* 发送 读 地址低位 */
//  SPI_SRAM_SendByte(ReadAddr & 0xFF);
//	
//	/* 读取数据 */
//  while (NumByteToRead--) /* while there is data to be read */
//  {
//    /* 读取一个字节*/
//    *pBuffer = SPI_SRAM_SendByte(Dummy_Byte);
//    /* 指向下一个字节缓冲区 */
//    pBuffer++;
//  }

//  /* 停止信号 FLASH: CS 高电平 */
//  SPI_SRAM_CS_HIGH();
}



 /**
  * @brief  使用SPI读取一个字节的数据
  * @param  无
  * @retval 返回接收到的数据
  */
u8 SPI_SRAM_ReadByte(void)
{
  return (SPI_SRAM_SendByte(Dummy_Byte));
}

 /**
  * @brief  使用SPI发送一个字节的数据
  * @param  byte：要发送的数据
  * @retval 返回接收到的数据
  */
u8 SPI_SRAM_SendByte(u8 byte)
{
	 SPITimeout = SPIT_FLAG_TIMEOUT;
//  /* 等待发送缓冲区为空，TXE事件 */
//  while (SPI_I2S_GetFlagStatus(SRAM_SPIx , SPI_I2S_FLAG_TXE) == RESET)
//	{
//    if((SPITimeout--) == 0) return SPI_TIMEOUT_UserCallback(0);
//   }

//  /* 写入数据寄存器，把要写入的数据写入发送缓冲区 */
//  SPI_I2S_SendData(SRAM_SPIx , byte);

//	SPITimeout = SPIT_FLAG_TIMEOUT;
//  /* 等待接收缓冲区非空，RXNE事件 */
//  while (SPI_I2S_GetFlagStatus(SRAM_SPIx , SPI_I2S_FLAG_RXNE) == RESET)
//  {
//    if((SPITimeout--) == 0) return SPI_TIMEOUT_UserCallback(1);
//   }

//  /* 读取数据寄存器，获取接收缓冲区数据 */
//  return SPI_I2S_ReceiveData(SRAM_SPIx );
}


/**
  * @brief  等待超时回调函数
  * @param  None.
  * @retval None.
  */
static  uint16_t SPI_TIMEOUT_UserCallback(uint8_t errorCode)
{
  /* 等待超时后的处理,输出错误信息 */
  FLASH_ERROR("SPI 等待超时!errorCode = %d",errorCode);
  return 0;
}
   
/*********************************************END OF FILE**********************/
