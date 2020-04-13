/**************************************************************************//**
* @file     gpio.h
* @version  V1.00
* $Revision: 9 $
* $Date: 15/05/18 5:38p $
* @brief    NUC970 GPIO driver header file
*
* @note
* Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#ifndef __ISL1208_H__
#define __ISL1208_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define I2C_ISL1208_ADDRESS 0x6F
#define PIN_RTC_SDA			GPIOD, BIT6
#define PIN_RTC_SCK			GPIOD, BIT7
// Acknowledgment type
#define DrvI2C_Ack_No        0
#define DrvI2C_Ack_Have        1

extern void Delay(uint32_t nCount);
extern int I2C_Open(void);
extern void RTC_I2C_SendStart(void);
extern void RTC_I2C_Stop(void);
extern void RTC_I2C_Ack(void);
extern void RTC_I2C_NAck(void);
extern uint8_t RTC_I2C_Wait_Ack(void);
extern void RTC_I2C_Send_Byte(uint8_t data);
extern uint8_t RTC_I2C_Read_Byte(uint8_t ack);
extern void RTC_WR_Reg(uint8_t uRegAddr, uint8_t uData);
extern uint8_t RTC_RD_Reg(uint8_t uRegAddr);
extern unsigned char RTC_Initialize(void);
extern int BCD_To_Dec(int temp);
extern int Dec_To_BCD(int temp);
extern uint8_t IsLeapYear(uint8_t yy);
extern void RTC_SetTime(uint8_t hh, uint8_t mm, uint8_t ss);
extern void RTC_SetDate(uint8_t yy, uint8_t mm, uint8_t dd);
extern void RTC_GetTime(uint8_t *hh, uint8_t *mm, uint8_t *ss);
extern void RTC_GetDate(uint8_t *yy, uint8_t *mm, uint8_t *dd);
extern void RTC_Int(uint8_t intrpt_hour, uint8_t intrpt_min, uint8_t intrpt_sec);
extern void RTC_IntDis(void);
extern uint8_t RTC_GetUserData(uint8_t idx);
extern void RTC_SetUserData(uint8_t idx, uint8_t dat);
#ifndef M_PRODUCT_COVER
extern void SetPIREn(uint8_t v);
extern uint8_t	IsPIREn(void);
#endif
#ifdef __cplusplus
}
#endif

#endif //__GPIO_H__

/*** (C) COPYRIGHT 2015 Nuvoton Technology Corp. ***/
