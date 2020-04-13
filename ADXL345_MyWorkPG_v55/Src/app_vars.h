#ifndef _XR_APPVARS_H_
#define _XR_APPVARS_H_


#define     MaxMsgLen   128
/*等待超时时间*/

typedef struct
	{
		unsigned int yy;
		unsigned char mm;
		unsigned char dd;
	}  MSystemDate;
typedef struct 
	{
		unsigned char hh;
		unsigned char mm;
		unsigned char ss;
	} MSystemTime;	
typedef struct 
	{
		unsigned char SartDl;
		unsigned int  Len;
		unsigned char FrameTp;
		unsigned char FrameID;
		unsigned long addr_64H;
		unsigned long addr_64L;
		unsigned int	addr_16;
		unsigned char BroadR;
		unsigned char Options;
		unsigned char RF_Dt_Len;
		unsigned char RF_Dt[80];
		unsigned char CheckSum;
	} TP_XB_API_FRM;
#endif
