#ifndef __ED037TC1_H__
#define __ED037TC1_H__
/*************************************************************************************
 * Copyright (C) 2019 E Ink Holdings Inc. (Ryan Liu, Eric CH Huang) All Rights Reserved *
 *************************************************************************************/
/**********************************
Constant set
**********************************/
#define DATA_MASK   0x0100
#define DCX_CMD     0x0000
#define DCX_DATA    0x0001

#define GATE_SET    0x01 // Gate Setting
#define VGH         0x03 // Gate Voltage Control
#define VSHL        0x04 // Source Voltage Control

#define GG_NOP      0x0B
#define BTST        0x0C 
#define GSP         0x0F //Gate Start position
#define DSP         0x10 //Deep leep mode
#define DEY         0x11 //Data Entry mode
#define SWRESET	    0x12 

#define TSC         0x18
#define WTM	    0x1A // Write Temp to IC
#define	RTM	    0x1B // Read Temp from IC
#define ETM	    0x1C // External Temp Sensor 

#define DSP_ACT		0x20 
#define DSP_SEQ		0x22
#define RAM_BW		0x24
#define RAM_RED		0x26
#define	VCOM_WT		0x2C //wirte Vcom   
#define	LUT_REG		0x32 //Write LUT   
#define	DIS_OPT		0x37 // Write Register for Display Option 
#define	BDW		0x3C //Border WF Control   
#define	X_ADDR		0x44 // Set RAM X address
#define Y_ADDR		0x45 // Set RAM Y address
#define AUTO_RAM_RED    0x46
#define AUTO_RAM_BW     0x47
#define	X_ADDRC		0x4E // Set RAM X address Counter
#define	Y_ADDRC		0x4F // Set RAM Y address Counter



#define Initial_23_16          0x00
#define Initial_15_0           0x0000 // 1K
#define Temperature0_23_16     0x00
#define Temperature0_15_0      0x0400 // 1K
#define Temperature1_23_16     0x00
#define Temperature1_15_0      0x0800
#define Temperature2_23_16     0x00
#define Temperature2_15_0      0x0C00
#define Temperature3_23_16     0x00
#define Temperature3_15_0      0x1000
#define Temperature4_23_16     0x00
#define Temperature4_15_0      0x1400
#define Temperature5_23_16     0x00
#define Temperature5_15_0      0x1800
#define Temperature6_23_16     0x00
#define Temperature6_15_0      0x1C00
#define Temperature7_23_16     0x00
#define Temperature7_15_0      0x2000
#define Temperature8_23_16     0x00
#define Temperature8_15_0      0x2400
#define Temperature9_23_16     0x00
#define Temperature9_15_0      0x2800
#define Temperature10_23_16    0x00
#define Temperature10_15_0     0x2C00

#define Initial_Counter            64
#define Temperature_LUT_Counter   1024


#endif 


#ifdef __ED037TC1_C__
#define __ED037TC1_EXTERN__
#else 
#define __ED037TC1_EXTERN__ extern
#endif

__ED037TC1_EXTERN__ void EPD_Init(void);
__ED037TC1_EXTERN__ void EPD_DTM1_Initial1(void);
__ED037TC1_EXTERN__ void spi_command(unsigned char dat);
__ED037TC1_EXTERN__ void spi_data(unsigned char dat);
__ED037TC1_EXTERN__ unsigned char spi_9b_get(void);
__ED037TC1_EXTERN__ void EPD_Load_LUT(unsigned int LUT, unsigned char const *LUT_Value, unsigned int LUT_Counter);

__ED037TC1_EXTERN__ void EPD_Display_White(void);
__ED037TC1_EXTERN__ void EPD_Display_Black(void);

__ED037TC1_EXTERN__ void check_busy_high(void);// If BUSYN=0 then waiting
__ED037TC1_EXTERN__ void check_busy_low(void);// If BUSYN=1 then waiting

__ED037TC1_EXTERN__ unsigned int byte_counter;
__ED037TC1_EXTERN__ unsigned int IMAGE_COUNTER;
__ED037TC1_EXTERN__ unsigned int SourceChannel;;
__ED037TC1_EXTERN__ unsigned int GateChannel;;