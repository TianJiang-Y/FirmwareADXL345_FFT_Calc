/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_tim.h"
//#include "stm32f1xx_hal_tim_ex.h"
#include "stm32f103xb.h"
#include "bsp_spi_sram.h"
//#include "app_vars.h"
#include "stdint.h"
//#include "stm32f1xx_hal_adc.h"
#include "xl345.h"
#include "math.h"

/*******************************************************************************/
/*                 Sampling frequency Select                                   */
/*******************************************************************************/

#define FS3200HZ_MODE

//#define FS1600HZ_MODE

/********************************************************************************/
TIM_HandleTypeDef htim4;
volatile uint8_t TIMcounter = 0;

void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart1;

#define DeselectSRAM		HAL_GPIO_WritePin(CS_RAM_GPIO_Port, CS_RAM_Pin, GPIO_PIN_SET)
#define SelectSRAM			HAL_GPIO_WritePin(CS_RAM_GPIO_Port, CS_RAM_Pin, GPIO_PIN_RESET)

float g_val[3];
unsigned char len;
unsigned char initf;	

/**************************************************************************************/

#ifdef FS1600HZ_MODE
	#define Fs              1600
	#define TIM_PERIOD      480
	#define RATE_VALUE      XL345_RATE_1600
#endif

#ifdef FS3200HZ_MODE
	#define Fs              3200
	#define TIM_PERIOD      240
	#define RATE_VALUE      XL345_RATE_3200
#endif
/**************************************************************************************/

#define PI              3.1415926
#define SAMPLENUMBER    512
#define FFT_BINS        256

#define FFT_Reso        (Fs / SAMPLENUMBER)   // Resoulation = Fs / N
#define DeltaTime       (1 / Fs)

#define	AccX       1
#define AccY       2
#define AccZ       3

#define LowFreq    10
#define HighFreq   1000

uint8_t CharBuf[10];

unsigned char ReadBuf[1024];  // 1024bytes = 1k

float INPUT[SAMPLENUMBER];    // 2k

float sin_tab[SAMPLENUMBER];  // 2k
float cos_tab[SAMPLENUMBER];  // 2k
	
float dataR[SAMPLENUMBER];    // 2k
float dataI[SAMPLENUMBER];    // 2k

float phase[FFT_BINS];     // 1k

float Vec[FFT_BINS];      // 1k

float RssV[FFT_BINS + 1];   // 1k

float ssm01[FFT_BINS];       // 1k
float ssm02[FFT_BINS];       // 1k

float w;
float max01;
float max02;
unsigned int Index01;
unsigned int Index02;

float freq01;
float freq02;

float veloc01;
float veloc02;

float displace;


float X_Freq = 0.0f;
float Y_Freq = 0.0f;
float Z_Freq = 0.0f;

float X_Vec = 0.0f;
float Y_Vec = 0.0f;
float Z_Vec = 0.0f;

float X_Displ = 0.0f;
float Y_Displ = 0.0f;
float Z_Displ = 0.0f;

unsigned int calc_len;
		
unsigned char sendbuf[105];
char StrBuf[10];
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART1_UART_Init(void);

// SRAM management
static void SetRAMMODE(unsigned char Md);

static void SeqnWriteRAM(unsigned long addr, unsigned long dtlen, unsigned char * Dt);
static void SeqnReadRAM(unsigned long addr, unsigned long dtlen, unsigned char * Dt);

static void ADXL_Init(void);
static void ADXL345_SetRegisterValue(unsigned char RegisterAddress, unsigned char RegisterData);


void MX_TIM1_Init(void);
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
#define DeselectADXL		HAL_GPIO_WritePin(CS_ADXL_GPIO_Port, CS_ADXL_Pin, GPIO_PIN_SET) // High
#define SelectADXL			HAL_GPIO_WritePin(CS_ADXL_GPIO_Port, CS_ADXL_Pin, GPIO_PIN_RESET) // Low

#define PowerOff				HAL_GPIO_WritePin(POWER_CTRL_GPIO_Port, POWER_CTRL_Pin, GPIO_PIN_SET)
#define PowerOn         HAL_GPIO_WritePin(POWER_CTRL_GPIO_Port, POWER_CTRL_Pin, GPIO_PIN_RESET)

#define DeselectSRAM		HAL_GPIO_WritePin(CS_RAM_GPIO_Port, CS_RAM_Pin, GPIO_PIN_SET)
#define SelectSRAM			HAL_GPIO_WritePin(CS_RAM_GPIO_Port, CS_RAM_Pin, GPIO_PIN_RESET)

/* USER CODE END PFP */
// For sampling parameters
#define Sample_Period  25      // millisecond
#define Sample_Time    160		 // millisecond, For 512-point FFT


/* USER CODE BEGIN 0 */
static void delay_ms (int ms) 
{
  ms *= (SystemCoreClock/11851);
  while (ms--) { __nop(); __nop(); __nop(); __nop(); __nop(); __nop(); }
}

void SPI1_SetSpeed(unsigned char SpeedSet)
{
	SpeedSet&=0X07;			//限制范围
	SPI1->CR1&=0XFFC7; 
	SPI1->CR1|=SpeedSet<<3;	//设置SPI5速度  
	SPI1->CR1|=1<<6; 		//SPI设备使能	  
} 

void SPI2_SetSpeed(unsigned char SpeedSet)
{
	SpeedSet&=0X07;			//限制范围
	SPI2->CR1&=0XFFC7; 
	SPI2->CR1|=SpeedSet<<3;	//设置SPI5速度  
	SPI2->CR1|=1<<6; 		//SPI设备使能	  
} 

//SPI1 读写一个字节
//TxData:要写入的字节
//返回值:读取到的字节
unsigned char  SPI1_ReadWriteByte(unsigned char  TxData)
{		 			 
	while((SPI1->SR&1<<1)==0);		//等待发送区空 
	SPI1->DR=TxData;	 	  		//发送一个byte  
	while((SPI1->SR&1<<0)==0);		//等待接收完一个byte  
	return SPI1->DR;          		//返回收到的数据				    
}

//SPI2 读写一个字节
//TxData:要写入的字节
//返回值:读取到的字节
unsigned char  SPI2_ReadWriteByte(unsigned char  TxData)
{		 			 
	while((SPI2->SR&1<<1)==0);		//等待发送区空 
	SPI2->DR=TxData;	 	  		//发送一个byte  
	while((SPI2->SR&1<<0)==0);		//等待接收完一个byte  
	return SPI2->DR;          		//返回收到的数据				    
}


void Blanking_Leds(void)
{
	HAL_GPIO_WritePin(GPIOC, LED_One_Pin, GPIO_PIN_RESET);delay_ms(250);
	HAL_GPIO_WritePin(GPIOC, LED_One_Pin, GPIO_PIN_SET);delay_ms(250);
	HAL_GPIO_WritePin(GPIOC, LED_TWO_Pin, GPIO_PIN_RESET);delay_ms(250);
	HAL_GPIO_WritePin(GPIOC, LED_TWO_Pin, GPIO_PIN_SET);delay_ms(250);
}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
	
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
	
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* SPI1 init function */
void MX_SPI1_Init(void)
{
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  HAL_SPI_Init(&hspi1);

}


/* SPI2 init function */
void MX_SPI2_Init(void)
{
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  HAL_SPI_Init(&hspi2);
}

/* USART1 init function */
void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart1);

}

void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED_One_Pin|LED_TWO_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, CS_RAM_Pin|CS_XBEE_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, Xbee_RX_Pin, GPIO_PIN_RESET);// Forcing Xbee module into SPI mode
	HAL_GPIO_WritePin(GPIOA, Xbee_TX_Pin, GPIO_PIN_RESET);// for Debugging

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, POWER_CTRL_Pin|RTC_SCK_Pin|RTC_SDA_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, Xbee_RST_Pin|CS_ADXL_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, SPI_MIX_MOSI_Pin | SPI_MIX_SCK_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : LED_One_Pin LED_TWO_Pin */
  GPIO_InitStruct.Pin = LED_One_Pin|LED_TWO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : CS_RAM_Pin CS_XBEE_Pin Xbee_RX_Pin*/

  GPIO_InitStruct.Pin = CS_RAM_Pin|CS_XBEE_Pin|Xbee_RX_Pin|Xbee_TX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : POWER_CTRL_Pin Xbee_RST_Pin CS_ADXL_Pin RTC_SCK_Pin 
                           RTC_SDA_Pin */
  GPIO_InitStruct.Pin = POWER_CTRL_Pin|Xbee_RST_Pin|CS_ADXL_Pin|RTC_SCK_Pin 
                          |RTC_SDA_Pin|SPI_MIX_SCK_Pin|SPI_MIX_MOSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : TempIn_Pin */
  GPIO_InitStruct.Pin = TempIn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(TempIn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Xbee_ATTN_Pin */
  GPIO_InitStruct.Pin = Xbee_ATTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Xbee_ATTN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Xbee_ATTN_Pin */
  GPIO_InitStruct.Pin = SPI_MIX_MISO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(SPI_MIX_MISO_GPIO_Port, &GPIO_InitStruct);
}


//void ByteWriteRAM(unsigned long addr, unsigned char Dt)
//{
//	 unsigned char tmpbyte;
//	// Select SRAM chip
//	 SelectSRAM;
//	// output Write Command
//	 SPI1_ReadWriteByte(SRAM_WRITE);
//	// output 24bit address
//	 tmpbyte = addr >> 16;
//	 SPI1_ReadWriteByte(tmpbyte);
//	 tmpbyte = (addr >> 8) & 0xff;
//	 SPI1_ReadWriteByte(tmpbyte);
//	 tmpbyte = addr & 0xff;
//	 SPI1_ReadWriteByte(tmpbyte);
//	 // Output Data
//	 SPI1_ReadWriteByte(Dt);
//	 // Deselect SRAM chip
//	DeselectSRAM;
//}

void SetRAMMODE(unsigned char Md)
{
//	 unsigned char tmpbyte;
	// Select SRAM chip
	 SelectSRAM;
	// output Write Mode Command
	 SPI1_ReadWriteByte(SRAM_WRMR);
	 // Output Mode Value
	 SPI1_ReadWriteByte(Md);
	 // Deselect SRAM chip
	 DeselectSRAM;
}

//unsigned char ByteReadRAM(unsigned long addr)
//{
//	 unsigned char tmpbyte;
//	// Select SRAM chip
//	 SelectSRAM;
//	// output Write Command
//	 SPI1_ReadWriteByte(SRAM_READ);
//	// output 24bit address
//	 tmpbyte = (addr >> 16) & 0xff;
//	 SPI1_ReadWriteByte(tmpbyte);
//	 tmpbyte = (addr >> 8) & 0xff;
//	 SPI1_ReadWriteByte(tmpbyte);
//	 tmpbyte = addr & 0xff;
//	 SPI1_ReadWriteByte(tmpbyte);
//	 // Output Data
//	 tmpbyte = SPI1_ReadWriteByte(Dummy_Byte);
//	 // Deselect SRAM chip
//	 DeselectSRAM;
//	 return tmpbyte;
//}

void SeqnWriteRAM(unsigned long addr, unsigned long dtlen, unsigned char * Dt)
{
	 unsigned char tmpbyte;
	 unsigned long i;
	// Select SRAM chip
	 SelectSRAM;
	// output Write Command
	 SPI1_ReadWriteByte(SRAM_WRITE);
	// output 24bit address
	 tmpbyte = (addr >> 16) & 0xff;
	 SPI1_ReadWriteByte(tmpbyte);
	 tmpbyte = (addr >> 8) & 0xff;
	 SPI1_ReadWriteByte(tmpbyte);
	 tmpbyte = addr & 0xff;
	 SPI1_ReadWriteByte(tmpbyte);
	 // Output Data
	 for(i=0;i<dtlen;i++)
		 SPI1_ReadWriteByte(*(Dt++));
	 // Deselect SRAM chip
	 DeselectSRAM;
}

void SeqnReadRAM(unsigned long addr, unsigned long dtlen, unsigned char * Dt)
{
	 unsigned char tmpbyte;
	 unsigned long i;
	// Select SRAM chip
	 SelectSRAM;
	// output Write Command
	 SPI1_ReadWriteByte(SRAM_READ);
	// output 24bit address
	 tmpbyte = (addr >> 16) & 0xff;
	 SPI1_ReadWriteByte(tmpbyte);
	 tmpbyte = (addr >> 8) & 0xff;
	 SPI1_ReadWriteByte(tmpbyte);
	 tmpbyte = addr & 0xff;
	 SPI1_ReadWriteByte(tmpbyte);
	 // Output Data
	 for(i=0;i<dtlen;i++)
		 *(Dt++) = SPI1_ReadWriteByte(Dummy_Byte);
	 // Deselect SRAM chip
	 DeselectSRAM;
}

//---------------------------------
//WriteToADXL345ViaSpi();
//---------------------------------
//Function that writes to the ADXL345 via the SPI port. It sends first the control
//word that includes the start address and then the data to write.
//--------------------------------------------------------------------------------
void WriteToADXL345ViaSpi(unsigned char RegisterAddress, unsigned char NumberofRegisters, unsigned char *RegisterData)
{
	unsigned	char	ControlValue = 0;
	unsigned	char	ValueToWrite = 0;
	signed		char	RegisterIndex = 0;

	//Create the 8-bit header
	ControlValue = (0x3F & RegisterAddress);

//	SET_SCK2;
//	adxl_delay(1);	
	DeselectADXL;
	HAL_GPIO_WritePin(GPIOB, SPI_MIX_MOSI_Pin | SPI_MIX_SCK_Pin, GPIO_PIN_SET);
	delay_ms(1);
  SelectADXL;
	SPI2_ReadWriteByte(ControlValue);

	//And then the data
	for (RegisterIndex=NumberofRegisters; RegisterIndex>0; RegisterIndex--)
	{
		ValueToWrite = *(RegisterData + RegisterIndex - 1);
		SPI2_ReadWriteByte(ValueToWrite);
	}
	DeselectADXL;	//bring CS high again
	HAL_GPIO_WritePin(GPIOB, SPI_MIX_MOSI_Pin | SPI_MIX_SCK_Pin, GPIO_PIN_SET);
}

//---------------------------------
//ADXL345_SetRegisterValue();
//---------------------------------
//Function that set to the ADXL345 via the SPI port.

void ADXL345_SetRegisterValue(unsigned char RegisterAddress, unsigned char RegisterData)
{
	unsigned	char	ControlValue = 0;
	unsigned	char	ValueToWrite = 0;

	//Create the 8-bit header
	ControlValue = (0x3F & RegisterAddress);

//	SET_SCK2;
//	adxl_delay(1);	
	DeselectADXL;
	HAL_GPIO_WritePin(GPIOB, SPI_MIX_MOSI_Pin | SPI_MIX_SCK_Pin, GPIO_PIN_SET);
	delay_ms(1);
  SelectADXL;
	SPI2_ReadWriteByte(ControlValue);
	ValueToWrite = RegisterData ;
	SPI2_ReadWriteByte(ValueToWrite);
	DeselectADXL;	//bring CS high again
	HAL_GPIO_WritePin(GPIOB, SPI_MIX_MOSI_Pin | SPI_MIX_SCK_Pin, GPIO_PIN_SET);
}

//---------------------------------
//ADXL345_ReadRegisterValue();
//---------------------------------
//Function that reads from the ADXL345 via the SPI port. It first send the control word
//that includes the start address and then 8 clocks for each register to read.
//--------------------------------------------------------------------------------
unsigned char ADXL345_ReadRegisterValue(unsigned char RegisterAddress)
{
	unsigned	char	ControlValue = 0;
	unsigned	char	ReceiveData = 0;

	//Create the 8-bit header
	ControlValue = (0x3F & RegisterAddress)+0x80;

//	SET_SCK2;
//	adxl_delay(1);	
	DeselectADXL;
	HAL_GPIO_WritePin(GPIOB, SPI_MIX_MOSI_Pin | SPI_MIX_SCK_Pin, GPIO_PIN_SET);
	__nop(); __nop();__nop(); __nop();
  SelectADXL;
	SPI2_ReadWriteByte(ControlValue);
	ReceiveData = SPI2_ReadWriteByte(0x00);
	DeselectADXL;	//bring CS high again
	HAL_GPIO_WritePin(GPIOB, SPI_MIX_MOSI_Pin | SPI_MIX_SCK_Pin, GPIO_PIN_SET);
	return ReceiveData;
} 

void ADXL_Init(void)
{
	 ADXL345_SetRegisterValue(XL345_OFSX,0x00);
   ADXL345_SetRegisterValue(XL345_OFSY,0x00);
   ADXL345_SetRegisterValue(XL345_OFSZ,0x00); 
   ADXL345_SetRegisterValue(XL345_DUR,0x00);  
   ADXL345_SetRegisterValue(XL345_LATENT,0x00);  
   ADXL345_SetRegisterValue(XL345_WINDOW,0x00);  

   ADXL345_SetRegisterValue(XL345_THRESH_ACT,0x01);  
   ADXL345_SetRegisterValue(XL345_THRESH_INACT,0x01); 
   ADXL345_SetRegisterValue(XL345_TIME_INACT,0x2B); 
   ADXL345_SetRegisterValue(XL345_ACT_INACT_CTL,0x00); // X,Y,Z Disable 
   ADXL345_SetRegisterValue(XL345_THRESH_FF,0x09);     // 600mg, Free-Fall
   ADXL345_SetRegisterValue(XL345_TIME_FF,0x46);       // 350mS, Free-Fall
   ADXL345_SetRegisterValue(XL345_TAP_AXES, XL345_TAP_SUPPRESS); 
   //ADXL345_read_byte(XL345_ACT_TAP_STATUS);  
   ADXL345_SetRegisterValue(XL345_BW_RATE, RATE_VALUE);  // Fs
   ADXL345_SetRegisterValue(XL345_POWER_CTL, XL345_MEASURE);
   ADXL345_SetRegisterValue(XL345_INT_ENABLE,0x00); 
   ADXL345_SetRegisterValue(XL345_INT_MAP,0x00);
   //ADXL345_read_byte(XL345_INT_SOURCE);   
   ADXL345_SetRegisterValue(XL345_DATA_FORMAT,0X0B); //self-test Disable,4-wire SPI mode,
   ADXL345_SetRegisterValue(XL345_FIFO_CTL,0x00);    // FIFO is bypassed. 
  //ADXL345_read_byte(XL345_FIFO_STATUS);     	
}

//---------------------------------
//ADXL345_ReadRegXyz();
//---------------------------------
//Function that reads acceleration data from the ADXL345 
//--------------------------------------------------------------------------------
void ADXL345_ReadRegXyz(unsigned char *intg)
{	
	uint8_t BUF[6]; 
	
	BUF[0] = ADXL345_ReadRegisterValue(XL345_DATAX0); *(intg++) = BUF[0]; // x-LSB
	BUF[1] = ADXL345_ReadRegisterValue(XL345_DATAX1); *(intg++) = BUF[1]; // x-MSB
	BUF[2] = ADXL345_ReadRegisterValue(XL345_DATAY0); *(intg++) = BUF[2]; // y-LSB
	BUF[3] = ADXL345_ReadRegisterValue(XL345_DATAY1); *(intg++) = BUF[3]; // y-MSB
	BUF[4] = ADXL345_ReadRegisterValue(XL345_DATAZ0); *(intg++) = BUF[4]; // z-LSB
	BUF[5] = ADXL345_ReadRegisterValue(XL345_DATAZ1); *(intg) = BUF[5];   // z-MSB
}

// sample_time's unit is millisecond.
// Reading data from ADXL345, write to the SRAM.....
unsigned int GetSamplesToRAM(unsigned int sample_time)
{
	unsigned int smpli;
	unsigned int smpln;	
		
	unsigned long addrx = 0x00000; // 0
	unsigned long addry = 0x01000; // 4096
	unsigned long addrz = 0x0A000; // 40960
	
  smpln = (sample_time*3200)/1000; // sample_time = 160mS, samples = 512-points.............
	
	if(smpln>3600000) 
		return 0; // This value is one of the case when we assume that period is 1 ms and time is one hour.
				      // In order to avoid error, this process is needed.
	
	HAL_TIM_Base_Start_IT(&htim4);      //start timer4 in interrupt mode.
	
	for(smpli = 0; smpli < smpln; smpli++)
			{		
    	    while(TIMcounter ==0);

					ADXL345_ReadRegXyz(&CharBuf[0]);
					HAL_GPIO_TogglePin(GPIOC, LED_One_Pin);
		
					SeqnWriteRAM(addrx,2,&CharBuf[0]);
					addrx += 2;
				
					SeqnWriteRAM(addry,2,&CharBuf[2]);
					addry += 2;
				
					SeqnWriteRAM(addrz,2,&CharBuf[4]);
					addrz += 2;			
				        
					TIMcounter = 0;
			}
			
	HAL_TIM_Base_Stop_IT(&htim4);        //stop timer4 in interrupt mode.
			
	return smpli;
}

void GetAccValueFromRAM(unsigned char axi)
{
	int16_t sndint;
	float sndflt;
	
	unsigned long addrx = 0x00000; // 0
	unsigned long addry = 0x01000; // 4096
	unsigned long addrz = 0x0A000; // 40960
	
	unsigned int j = 0;
	
	switch(axi)
	{
		case AccX:
					SeqnReadRAM(addrx,1024,&ReadBuf[0]);
		
					for(unsigned int i = 0; i< 1023; i+=2)
						{
							sndint = (((int16_t)ReadBuf[i+1]) << 8) | ReadBuf[i];
              
              // The unit of data obtained from the acceleration sensor is g.
              // If the value read from the sensor is multiplied by 9.8, 
              // the its unit  is obtained as the acceleration value as [m / s2]. 
              // Therefore, the final calculated value is also obtained as [m / s].
              // Unless multiplied by 9.8 as in the current code, all operations are computed in g unit.
              
							sndflt = (float) sndint * ADXL345_MG_LSB_FULL_RES / 1000;// * 9.80665;
              
							INPUT[j] = sndflt;
							j++;
						}
					break;
						
		case AccY:
					SeqnReadRAM(addry,1024,&ReadBuf[0]);
		
					for(unsigned int i = 0; i< 1023; i+=2)
						{
							sndint = (((int16_t)ReadBuf[i+1]) << 8) | ReadBuf[i];
              
							sndflt = (float) sndint * ADXL345_MG_LSB_FULL_RES / 1000;// * 9.80665;
              
							INPUT[j] = sndflt;
							j++;
						}
					break;
						
		case AccZ:
					SeqnReadRAM(addrz,1024,&ReadBuf[0]);
		
					for(unsigned int i = 0; i< 1023; i+=2)
						{
							sndint = (((int16_t)ReadBuf[i+1]) << 8) | ReadBuf[i];
              
							sndflt = (float) sndint * ADXL345_MG_LSB_FULL_RES / 1000;// * 9.80665;
              
							INPUT[j] = sndflt;
							j++;
						}
					break;
						
		default:break;
	}
	
	INPUT[SAMPLENUMBER - 1] = 0;
}

// For FFT calculation, sin and cos previously calculate....
void InitForFFT(void)
{
	int i;
	
	for ( i = 0; i < SAMPLENUMBER; i++ )
	{
		sin_tab[i] = sin(PI * 2 * i / SAMPLENUMBER);
		cos_tab[i] = cos(PI * 2 * i / SAMPLENUMBER);
	}
}

// 512-points FFT calculating Function...
void FFT(float InArr[SAMPLENUMBER])
{
	int x0,x1,x2,x3,x4,x5,x6,x7,x8,xx;
	int i,j,k,b,p,L;
	float TR,TI,temp;
	
	for ( unsigned int i = 0; i < SAMPLENUMBER; i++ )
	{
		dataR[i] = InArr[i];
//		dataR[SAMPLENUMBER-1] = 0;
		dataI[i] = 0.0f;
		InArr[i] = 0.0f;
	}
	
	/********** following code invert sequence ************/
	for ( i = 0; i < SAMPLENUMBER; i++ )
	{
		x0 = x1 = x2 = x3 = x4 = x5 = x6 = x7 = x8 = 0;
		x0 = i & 0x01; 
		x1 = (i / 2) & 0x01; 
		x2 = (i / 4) & 0x01; 
		x3 = (i / 8) & 0x01;
		x4 = (i / 16) & 0x01; 
		x5 = (i / 32) & 0x01; 
		x6 = (i / 64) & 0x01;
		x7 = (i / 128) & 0x01;
		x8 = (i / 256) & 0x01;
		
		xx = x0 * 256 + x1 * 128 + x2 * 64 + x3 * 32 + x4 * 16 + x5 * 8 + x6 * 4 + x7 * 2 + x8;
		dataI[xx] = dataR[i];
	}
	
	for ( i = 0; i < SAMPLENUMBER; i++ )
	{
		dataR[i] = dataI[i]; 
		dataI[i] = 0; 
	}

	/************** following code FFT *******************/
	
	for ( L = 1; L <= 9; L++ )  /* for(1) */
		{ 
			b = 1; 
			i = L - 1;
			
			while ( i > 0 ) 
				{
					b = b * 2; 
					i--;
				} /* b= 2^(L-1) */

			for ( j = 0; j <= b - 1; j++ )  /* for (2) */
				{
					p = 1; 
					i = 9 - L;

					while ( i > 0 ) /* p=pow(2,9-L)*j; */
						{
							p = p * 2;
							i--;
						}

					p = p * j;

					for ( k = j; k < SAMPLENUMBER; k = k + 2 * b ) /* for (3) */
						{
							TR = dataR[k];
							TI = dataI[k]; 
							temp = dataR[k + b];
							dataR[k] = dataR[k] + dataR[k+b] * cos_tab[p] + dataI[k+b] * sin_tab[p];
							dataI[k] = dataI[k] - dataR[k+b] * sin_tab[p] + dataI[k+b] * cos_tab[p];
							dataR[k + b] = TR - dataR[k + b] * cos_tab[p] - dataI[k+b] * sin_tab[p];
							dataI[k+b] = TI + temp * sin_tab[p] - dataI[k + b] * cos_tab[p];
						}
				}
		}
		
	for ( i = 1; i < FFT_BINS; i++ )
		{ 
		 /** Amplitude Spectrum **/
			InArr[i - 1] = sqrt(dataR[i+1] * dataR[i+1] + dataI[i+1] * dataI[i+1]);
			
		 /** Phase Spectrum **/
			phase[i - 1] = atan(dataI[i+1] / dataR[i+1]);
		}
} /* END FFT */

unsigned char BandPass_Filter(float FiInArr[FFT_BINS], unsigned int LowBand, unsigned int HighBand)
{
    float LowBin = 0.0f;
    float HighBin = 0.0f;

    float FreqL, FreqH;
    
    unsigned int binIndex01;
    unsigned int binIndex02;
  
  /**** Detected First First Maximum Value *****/
		for ( unsigned int i = 1; i < FFT_BINS; i++ )
		{		
			if(FiInArr[i] > HighBin)
			{
				HighBin = FiInArr[i];
				binIndex01 = i;
			}
		}
    
		FreqH = binIndex01 * FFT_Reso;
/**** Detected First Second Maximum Value *****/
		for ( unsigned int i = 1; i < (binIndex01 - 5); i++ )
		{		
			if((FiInArr[i] < HighBin) && (FiInArr[i] > LowBin))
			{
				LowBin = FiInArr[i];
				binIndex02 = i;
			}
		}
    
    FreqL = binIndex02 * FFT_Reso;
    
/***** Pass the Band Pass Filter **************/
    if((FreqL < LowBand) && (FreqH > HighBand))
      return 1;
    else
      return 0;
}

// Evaluate the velocity(in mm/s) of each frequency bin.
// Next, Running an RSS moving average on the velocity samples

void Calc_VelocRSS_Displace(float AccArr[FFT_BINS], float Inpha[FFT_BINS])
{		
		max01 = 0.0f;
		max02 = 0.0f;
		Index01 = 0;
		Index02 = 0;
	
		w = 0.0f;
	
		freq01 = 0.0f;
		freq02 = 0.0f;
	
		veloc01 = 0.0f;
		veloc02 = 0.0f;
	
		displace = 0.0f;
	
		for(unsigned int n = 0 ;n < FFT_BINS ;n++)
		{
			Vec[n] = 0.0f;
			ssm01[n] = 0.0f;
			ssm02[n] = 0.0f;
			RssV[n] = 0.0f;
		}
		
		RssV[FFT_BINS] = 0.0f;
		

		for(unsigned int j = 1; j < FFT_BINS; j++)
		{
			/**** w = 2 * PI * f, f = Delta f * n ****/
			w = 2 * PI * FFT_Reso * j;
      
      // If you have not multiplied 9.8 in the calculations of lines 731, 745, and 759, 
      // you must multiply them here.
      
      // Otherwise, you can implement by the following code.

      // If you multiply following formula by 1000 and 9.8, the unit is mm/s.
      // Unless multiplied by 1000, the unit is m/s.
      
			Vec[j] = (AccArr[j] / w) * 9.8 * 1000;   // when the unit is mm/s.
      
//      Vec[j] = (AccArr[j] / w) * 9.8;        // when the unit is m/s.
			
			/****  RSS Moving Average, Window Length  L = 2  ****/
			RssV[j] = sqrt(Vec[j-1] * Vec[j-1] + Vec[j] * Vec[j] + Vec[j+1] * Vec[j+1]);			
		}

// The equivalent vibration (in mm/s) and frequency
// of the measured samples is then evaluated based on
// the maximum value detected across the different frequencies		
		
/**** Detected First First Maximum Value *****/
		for ( unsigned int i = 2; i < FFT_BINS; i++ )
		{		
			if(RssV[i] > max01)
			{
				max01 = RssV[i];
				Index01 = i;
			}
		}
		
/**** Detected First Second Maximum Value *****/
		for ( unsigned int i = 2; i < FFT_BINS; i++ )
		{		
			if((RssV[i] < max01) && (RssV[i] > max02))
			{
				max02 = RssV[i];
				Index02 = i;
			}
		}
		
/**** calculate Max Frequency displacement according to time  ****/
		for(unsigned int j = 1;j < FFT_BINS; j++)
		{
			/****  Calculate the Displacement of each frequency bins  *****/
			ssm01[j] = ssm01[j - 1] + max01 * cos(w * j * DeltaTime + phase[j]) * DeltaTime;
			
			ssm02[j] = ssm02[j - 1] + max02 * cos(w * j * DeltaTime + phase[j]) * DeltaTime;
		}
	
/**** Velocity, Frequency, Displacement Return *****/
		freq01 = Index01 * FFT_Reso;
		freq02 = Index02 * FFT_Reso;
		
		veloc01 = max01;
		veloc02 = max02;
		
/**** displace unit is nm(*1000) ****/
//		displace = (RssV[Index - 1] * DeltaTime + 0.5 * AccArr[Index] * DeltaTime * DeltaTime) * 1000;
}

/**
  * Initializes the Global MSP.
  */

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base) 
{
 if(htim_base->Instance==TIM4) 
  {
    /* Peripheral clock enable */
    __HAL_RCC_TIM4_CLK_ENABLE();
    /* TIM4 interrupt Init */
    HAL_NVIC_SetPriority(TIM4_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM4_IRQn);
  }
}

/* TIM4 init function */ 
void MX_TIM4_Init(void) 
{
  TIM_ClockConfigTypeDef sClockSourceConfig; 
  TIM_MasterConfigTypeDef sMasterConfig;
	
  htim4.Instance = TIM4; 
  htim4.Init.Prescaler = 20; 
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP; 
  htim4.Init.Period = TIM_PERIOD;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1; 
  htim4.Init.RepetitionCounter = 0;
//  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

int main(void)
{
	TIMcounter = 0;
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
	MX_SPI1_Init();
  MX_SPI2_Init();
	
  MX_USART1_UART_Init();
	
	MX_TIM4_Init();
	
	Blanking_Leds();
	
	SPI1_SetSpeed(1);
	SPI2_SetSpeed(1);
	delay_ms(10);

	SetRAMMODE((MD_SEQN));
	ADXL_Init();	
	InitForFFT();
	
	HAL_TIM_Base_MspInit(&htim4);        //stop timer4 in interrupt mode.

	while (1)
		{
			calc_len = GetSamplesToRAM(Sample_Time);
			
			if(calc_len > 511)
				{
					HAL_GPIO_TogglePin(GPIOC, LED_TWO_Pin);
					/* X-axi */
					/*************************************************/
						GetAccValueFromRAM(AccX);	// Time Domain
					/*************************************************/
						FFT(INPUT);  // Frequency Domain
					/*************************************************/
            if(BandPass_Filter(INPUT, LowFreq, HighFreq))
              {
                  Calc_VelocRSS_Displace(INPUT, phase);

                  X_Freq = freq01;
                  X_Vec = veloc01;
                  X_Displ = displace;						
              }
            else
              {
                  X_Freq = (unsigned int)0xFF;
                  X_Vec = (unsigned int)0xFF;
                  X_Displ = (unsigned int)0xFF;		
              }
					/*************************************************/				
					
					/* Y-axi*/
					/*************************************************/
						GetAccValueFromRAM(AccY);	// Time Domain
					/*************************************************/
						FFT(INPUT); // Frequency Domain
					/*************************************************/
            if(BandPass_Filter(INPUT, LowFreq, HighFreq))
              {
                  Calc_VelocRSS_Displace(INPUT, phase);
                
                  Y_Freq = freq01;
                  Y_Vec = veloc01;
                  Y_Displ = displace;						
              }
            else
              {
                  Y_Freq = (unsigned int)0xFF;
                  Y_Vec = (unsigned int)0xFF;
                  Y_Displ = (unsigned int)0xFF;		
              }
					/*************************************************/
						
					/* Z-axi*/
					/*************************************************/
						GetAccValueFromRAM(AccZ);	// Time Domain						
					/*************************************************/
						FFT(INPUT); // Frequency Domain
					/*************************************************/
            if(BandPass_Filter(INPUT, LowFreq, HighFreq))
              {
                  Calc_VelocRSS_Displace(INPUT, phase);
                
                  Z_Freq = freq01;
                  Z_Vec = veloc01;
                  Z_Displ = displace;						
              }
            else
              {
                  Z_Freq = (unsigned int)0xFF;
                  Z_Vec = (unsigned int)0xFF;
                  Z_Displ = (unsigned int)0xFF;		
              }
					/************* Calculating End  *****************/
						calc_len = 0;
				}
		}
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
