#include <stdio.h>
#include <string.h>
#include "stm32f1xx_hal.h"
#include "isl1208.h"
#include "app_vars.h"


  GPIO_InitTypeDef GPIO_InitStruct;
	
void Delay(uint32_t nCount)
{
//    for(; nCount!=0; nCount--)
//        delay_count(200);
}
static const uint8_t daysOfMonth[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

uint8_t IsYunYear(int year)
{
	if (year % 4 != 0) return 0;
	if (year % 100 == 0) return 0;
	if (year % 400 != 0) return 0;
	return 1;
}

uint32_t DateToSecs(uint16_t syear, uint8_t smon, uint8_t sday, uint8_t hour, uint8_t min, uint8_t sec)
{
	uint16_t t;
	uint32_t seccount = 0;

	if(syear < 1970) return 0;
	if(smon < 1 || smon > 12)return 0;
	if(sday < 1 || sday > 31)return 0;	

	for (t = 1970; t < syear; t++)
	{
		if (IsYunYear(t)) seccount += 31622400;
		else seccount += 31536000;			 
	}
	smon -= 1;
	for (t = 0; t < smon; t++)	  
	{
		seccount += (uint32_t)daysOfMonth[t] * 86400;
		if(IsYunYear(syear) && t == 1)
			seccount += 86400;  
	}
	seccount += (uint32_t)(sday - 1) * 86400;
	seccount += (uint32_t)hour * 3600;
	seccount += (uint32_t)min * 60;
	seccount += sec;

	return seccount;	    
}

void SecsToDate(uint32_t usec, MSystemDate* pDate, MSystemTime *pTime)
{
	uint16_t daycnt = 0;
	uint32_t temp = 0;
	uint16_t temp1 = 0;	
	
 	temp = usec / 86400;
	if (daycnt != temp)
	{	  
		daycnt = temp;
		temp1 = 1970;	
		while (temp >= 365)
		{				 
			if (IsYunYear(temp1))
			{
				if(temp >= 366) temp -= 366;
				else { temp1++; break; }  
			}
			else temp -= 365;	  
			temp1++;  
		}   
		pDate->yy = temp1;
		temp1 = 0;
		while (temp >= 28)
		{
			if (IsYunYear(pDate->yy) && temp1 == 1)
			{
				if (temp >= 29)
					temp -= 29;
				else break; 
			}
			else 
			{
				if (temp >= daysOfMonth[temp1])
					temp -= daysOfMonth[temp1];
				else break;
			}
			temp1++;  
		}
		pDate->mm = temp1 + 1;
		pDate->dd  = temp + 1;  
	}
	else
	{
		pDate->yy  = 1970;
		pDate->mm = 1;
		pDate->dd  = 1;
	}
	temp = usec % 86400;     	  	   
	pTime->hh = temp / 3600;     	
	pTime->mm  = (temp % 3600) / 60; 
	pTime->ss  = (temp % 3600) % 60; 
}

void SecToDateString(uint32_t n, char *pStr)
{
	MSystemDate d;
	MSystemTime t;
	SecsToDate(n, &d, &t);

	sprintf(pStr, "%04d.%02d.%02d %02d:%02d:%02d", d.yy, d.mm, d.dd, t.hh, t.mm, t.ss);
}

uint8_t ConvToBCD(uint8_t v)
{
	return ((v / 10) << 8) | (v % 8);
}

uint8_t CalcDayOfWeek(int yy, int mm, int dd)
{
	uint16_t temp2;
	uint8_t yearH, yearL;

	const uint8_t table_week[12] = {0, 3, 3, 6, 1, 4, 6, 2, 5, 0, 3, 5};
	
	yearH = yy / 100;	
	yearL = yy % 100; 

	if(yearH > 19)
		yearL += 100;

	temp2 = yearL + yearL / 4;
	temp2 = temp2 % 7; 
	temp2 = temp2 + dd + table_week[mm - 1];
	if (yearL % 4 == 0 && mm < 3) temp2--;
	temp2 = temp2 % 7;
	if(temp2 == 0) temp2 = 7;
	return temp2;
}

uint32_t DecToBCD(uint32_t v)
{
	uint32_t ret = 0;
	ret <<= 4;
	ret += ((v % 10000) / 1000);
	ret <<= 4;
	ret += ((v % 1000) / 100);
	ret <<= 4;
	ret += ((v % 100) / 10);
	ret <<= 4;
	ret += (v % 10);
	
	return ret;
}

uint32_t BCDToDec(uint32_t v)
{
	uint32_t ret = 0;
	ret *= 10;
	ret += ((v & 0xf000) >> 12);
	ret *= 10;
	ret += ((v & 0x0f00) >> 8);
	ret *= 10;
	ret += ((v & 0x00f0) >> 4);
	ret *= 10;
	ret += (v & 0x000f);	
	
	return ret;
}
int I2C_Open(void)
{
	
		  GPIO_InitStruct.Pin = RTC_SCK_Pin | RTC_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

   /*Configure GPIO pin : RTC_SDA_Pin */
//  GPIO_InitStruct.Pin = RTC_SDA_Pin;
//  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
//  GPIO_InitStruct.Pull = GPIO_PULLUP;
//  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	  
		HAL_GPIO_WritePin(GPIOB, RTC_SCK_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, RTC_SDA_Pin, GPIO_PIN_SET);
    Delay(10);

		HAL_GPIO_WritePin(GPIOB, RTC_SCK_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, RTC_SDA_Pin, GPIO_PIN_RESET);
		Delay(10);

		HAL_GPIO_WritePin(GPIOB, RTC_SCK_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, RTC_SDA_Pin, GPIO_PIN_SET);
    Delay(20);

		return 0;
}

void RTC_I2C_SendStart(void)
{
    // Assert start bit
		GPIO_InitStruct.Pin = RTC_SCK_Pin | RTC_SDA_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	  
		HAL_GPIO_WritePin(GPIOB, RTC_SDA_Pin, GPIO_PIN_SET);
    Delay(5);
		HAL_GPIO_WritePin(GPIOB, RTC_SCK_Pin, GPIO_PIN_SET);
		Delay(5);
		HAL_GPIO_WritePin(GPIOB, RTC_SDA_Pin, GPIO_PIN_RESET);
    Delay(5);
		HAL_GPIO_WritePin(GPIOB, RTC_SCK_Pin, GPIO_PIN_RESET);
		Delay(5);
	
}

void RTC_I2C_Stop(void)
{
		GPIO_InitStruct.Pin = RTC_SCK_Pin | RTC_SDA_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
    Delay(5);
    HAL_GPIO_WritePin(GPIOB, RTC_SDA_Pin, GPIO_PIN_RESET);
	Delay(5);
    HAL_GPIO_WritePin(GPIOB, RTC_SCK_Pin, GPIO_PIN_SET);
	Delay(5);
    HAL_GPIO_WritePin(GPIOB, RTC_SDA_Pin, GPIO_PIN_SET);
	Delay(5);
}
//??ACK??
void RTC_I2C_Ack(void)
{
	GPIO_InitStruct.Pin = RTC_SCK_Pin | RTC_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	HAL_GPIO_WritePin(GPIOB, RTC_SCK_Pin, GPIO_PIN_RESET);
	Delay(5);
	HAL_GPIO_WritePin(GPIOB, RTC_SDA_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, RTC_SCK_Pin, GPIO_PIN_SET);
	Delay(5);
	HAL_GPIO_WritePin(GPIOB, RTC_SCK_Pin, GPIO_PIN_RESET);
	Delay(5);
}

void RTC_I2C_NAck(void)
{
	GPIO_InitStruct.Pin = RTC_SCK_Pin | RTC_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	HAL_GPIO_WritePin(GPIOB, RTC_SCK_Pin, GPIO_PIN_RESET);
	Delay(5);
	HAL_GPIO_WritePin(GPIOB, RTC_SDA_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, RTC_SCK_Pin, GPIO_PIN_SET);
	Delay(5);
	HAL_GPIO_WritePin(GPIOB, RTC_SCK_Pin, GPIO_PIN_RESET);
	Delay(5);
}

uint8_t RTC_I2C_Wait_Ack(void)
{
	uint8_t bit;
	uint8_t ucErrTime = 0;
		GPIO_InitStruct.Pin = RTC_SCK_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
		GPIO_InitStruct.Pin = RTC_SDA_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	

	HAL_GPIO_WritePin(GPIOB, RTC_SCK_Pin, GPIO_PIN_SET);
	Delay(5);
	bit = HAL_GPIO_ReadPin(GPIOB,		RTC_SDA_Pin);
	while (bit)
	{
		ucErrTime++;
		if(ucErrTime > 250)
		{
			RTC_I2C_Stop();
			return 1;
		} 
		Delay(5);
		bit = HAL_GPIO_ReadPin(GPIOB,		RTC_SDA_Pin);
	}
	HAL_GPIO_WritePin(GPIOB, RTC_SCK_Pin, GPIO_PIN_RESET);
	Delay(5);
	return 0;  
}

void RTC_I2C_Send_Byte(uint8_t data)
{
	uint8_t i;
		GPIO_InitStruct.Pin = RTC_SCK_Pin | RTC_SDA_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	
    HAL_GPIO_WritePin(GPIOB, RTC_SCK_Pin, GPIO_PIN_RESET);
	Delay(5);
	for (i = 0; i < 8; i++)
    {              
		if (data & 0x80)
			HAL_GPIO_WritePin(GPIOB, RTC_SDA_Pin, GPIO_PIN_SET);
		else
			HAL_GPIO_WritePin(GPIOB, RTC_SDA_Pin, GPIO_PIN_RESET);

		data <<= 1; 	      
		HAL_GPIO_WritePin(GPIOB, RTC_SCK_Pin, GPIO_PIN_SET);
		Delay(5);
		HAL_GPIO_WritePin(GPIOB, RTC_SCK_Pin, GPIO_PIN_RESET);
		Delay(5);
    }	 
	RTC_I2C_Wait_Ack();
}

uint8_t RTC_I2C_Read_Byte(uint8_t ack)
{
	uint8_t i, receive = 0;
	uint8_t bit;
		GPIO_InitStruct.Pin = RTC_SCK_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
		GPIO_InitStruct.Pin = RTC_SDA_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	Delay(5);
	for (i = 0; i < 8; i++)
	{ 
		HAL_GPIO_WritePin(GPIOB, RTC_SCK_Pin, GPIO_PIN_RESET);
		Delay(5);
		HAL_GPIO_WritePin(GPIOB, RTC_SCK_Pin, GPIO_PIN_SET);
		receive <<= 1;
		bit = HAL_GPIO_ReadPin(GPIOB,		RTC_SDA_Pin);
		Delay(5);
		if (bit) receive++;   
	}	  				 
	if (!ack) 
		RTC_I2C_NAck();//??nACK
	else 
		RTC_I2C_Ack(); //??ACK   
	return receive;
}

void RTC_WR_Reg(uint8_t uRegAddr, uint8_t uData)
{
    // 1-Phase(ID address, register address) write transmission
    RTC_I2C_SendStart();
    RTC_I2C_Send_Byte(I2C_ISL1208_ADDRESS << 1);        // Write ID address to sensor
    RTC_I2C_Send_Byte(uRegAddr);    // Write register address to sensor
	RTC_I2C_Send_Byte(uData);    // Write register address to sensor	
    RTC_I2C_Stop();
}

uint8_t RTC_RD_Reg(uint8_t uRegAddr)
{
    uint8_t u8Data;

    // 1-Phase(ID address, register address) write transmission
	RTC_I2C_SendStart();
    RTC_I2C_Send_Byte(I2C_ISL1208_ADDRESS << 1);        // Write ID address to sensor
    RTC_I2C_Send_Byte(uRegAddr);    // Write register address to sensor
	RTC_I2C_SendStart();
	RTC_I2C_Send_Byte((I2C_ISL1208_ADDRESS << 1)|0x01);        // Write ID address to sensor
    u8Data = RTC_I2C_Read_Byte(DrvI2C_Ack_No);        // Read data from sensor    
    RTC_I2C_Stop();

    return u8Data;
}

unsigned char RTC_Initialize(void)
{
	uint8_t yy, mm, dd;
	uint8_t hh, mmm, ss;

		GPIO_InitStruct.Pin = RTC_SCK_Pin | RTC_SDA_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    I2C_Open();

	RTC_WR_Reg(0x07, 0x91); 
//	RTC_WR_Reg(0x08, 0x80);  
//	RTC_WR_Reg(0x0C, 0x00);
	RTC_WR_Reg(0x0D, 0x00);
	RTC_WR_Reg(0x0E, 0x00);

	RTC_GetDate(&yy, &mm, &dd);
	RTC_GetTime(&hh, &mmm, &ss);
	if (yy == 0 && mm == 0 && dd == 0 &&
		hh == 0 && mmm == 0 && ss == 0)
	{
		RTC_SetDate(17, 1, 1);
		return 0;
	}
	return 1;
}

void RTC_SetTime(uint8_t hh, uint8_t mm, uint8_t ss)  
{  
	RTC_WR_Reg(0x00, DecToBCD(ss) & 0xff);
	RTC_WR_Reg(0x01, DecToBCD(mm) & 0xff);
	RTC_WR_Reg(0x02, (DecToBCD(hh) & 0xff) + 0x80);
} 
 
void RTC_SetDate(uint8_t yy, uint8_t mm, uint8_t dd)
{
	RTC_WR_Reg(0x03, DecToBCD(dd) & 0xff);
	RTC_WR_Reg(0x04, DecToBCD(mm) & 0xff);
	RTC_WR_Reg(0x05, DecToBCD(yy) & 0xff);	
}

void RTC_GetTime(uint8_t *hh, uint8_t *mm, uint8_t *ss)  
{  
	uint8_t temp;
	*ss = (uint8_t)(BCDToDec(RTC_RD_Reg(0x00)) & 0xff);
	*mm = (uint8_t)(BCDToDec(RTC_RD_Reg(0x01)) & 0xff);
	temp = RTC_RD_Reg(0x02);
	if(temp & 0x80)
	{
		*hh = (uint8_t)(BCDToDec(temp & 0x3F) & 0xff);
	}
	else if (temp & 0x20)
	{
		*hh = (uint8_t)(BCDToDec(temp & 0x1F + 0x12) & 0xff);
	}
	else
	{
		*hh = (uint8_t)(BCDToDec(temp & 0x1F) & 0xff);
	}		
}
  
void RTC_GetDate(uint8_t *yy, uint8_t *mm, uint8_t *dd) 
{
	*dd = (uint8_t)(BCDToDec(RTC_RD_Reg(0x03)) & 0xff);
	*mm = (uint8_t)(BCDToDec(RTC_RD_Reg(0x04)) & 0xff);
	*yy = (uint8_t)(BCDToDec(RTC_RD_Reg(0x05)) & 0xff);
}

void RTC_Int(uint8_t intrpt_hour, uint8_t intrpt_min, uint8_t intrpt_sec)
{			   
	uint8_t ret;
	// Set Second Alarm
	while (1)
	{
		RTC_WR_Reg(0x0C, 0x80 + (DecToBCD(intrpt_sec) & 0xff));
		ret = RTC_RD_Reg(0x0C);
		if (ret == (0x80 + (DecToBCD(intrpt_sec) & 0xff)))
			break;
	}
	// Set Minute Alarm 
  while (1)
	{
		if(intrpt_min<60)
			{
				RTC_WR_Reg(0x0D, 0x80 + (DecToBCD(intrpt_min) & 0xff));
				ret = RTC_RD_Reg(0x0D);
				if (ret == (0x80 + (DecToBCD(intrpt_min) & 0xff)))
					break;
			}
			else // If minute value is more than 59, then it is analyzed as "disable minute alarm".
				{
					RTC_WR_Reg(0x0D, 0x00);
					ret = RTC_RD_Reg(0x0D);
					if (ret == 0x00 )
							break;
				}
	}
	// Set Hour Alarm
  while (1)
	{
		if(intrpt_hour<24)
			{
				RTC_WR_Reg(0x0E, 0x80 + (DecToBCD(intrpt_hour) & 0xff));
				ret = RTC_RD_Reg(0x0E);
				if (ret == (0x80 + (DecToBCD(intrpt_hour) & 0xff)))
					break;
			}
			else  // If hour value is more than 23, then it is analyzed as "disable hour alarm".
				{
					RTC_WR_Reg(0x0E, 0x00);
					ret = RTC_RD_Reg(0x0E);
					if (ret == 0x00 )
							break;
				}
	}
	while (1)
	{
		RTC_WR_Reg(0x08, 0xC0);  
		ret = RTC_RD_Reg(0x08);
		if (ret == 0xC0)
			break;
	}
}

void RTC_IntDis(void)
{
	uint8_t ret;
	while (1)
	{
		RTC_WR_Reg(0x0C, 0x00);
		ret = RTC_RD_Reg(0x0C);
		if (ret == 0x00)
			break;
	}

	while (1)
	{
		RTC_WR_Reg(0x08, 0x80);
		ret = RTC_RD_Reg(0x08);
		if (ret == 0x80)
			break;
	}

}

uint8_t RTC_GetUserData(uint8_t idx)
{
	return RTC_RD_Reg(0x12 + idx);
}
									   
void RTC_SetUserData(uint8_t idx, uint8_t dat)
{
	RTC_WR_Reg(0x12 + idx, dat);
}
