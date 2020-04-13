#ifndef __SPI_SRAM_H
#define __SPI_SRAM_H

#define SPI_SRAM_PageSize              32
#define SPI_SRAM_PerWritePageSize      32

/*命令定义-开头*******************************/
#define SRAM_WRMR					      	0x01 
#define SRAM_WRITE					      0x02 
#define SRAM_READ							    0x03 
#define SRAM_RDMR							    0x05 
#define SRAM_EDIO						      0x3B 
#define SRAM_RSTIO					      0xFF 

#define MD_BYTE											0x00
#define MD_PAGE											0x80
#define MD_SEQN											0x40
#define MD_RESV											0xC0


#define Dummy_Byte                0xFF

#endif /* __SPI_SRAM_H */

