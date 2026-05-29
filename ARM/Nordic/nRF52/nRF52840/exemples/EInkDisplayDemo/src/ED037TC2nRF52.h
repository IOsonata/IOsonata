/**************************************************************************//*

  Copyright (C) 2019 E Ink Holdings Inc. (Ryan Liu, Eric CH Huang)
  All Rights Reserved

  Copyright (C) 2020 Quick Turn Designs LLC
  All Rights Reserved

  This file is Confidential and Proprietary
  Any reproduction or transfer of this information file is prohibited

*//************************************************************************//*

  File:        ED037TC2.h

  Description: Definitions, Function Prototypes, etc. for the ED037TC2 display

*//************************************************************************//*

Date      - Author             - Change Description
---------   ------------------   --------------------------------------------
unknown   - Ryan Liu           - Original file creation
            Eric CH Huang
22feb2020 - Robert Bennett     - Port to Quick Turn Designs Project (ST Com)

*//**************************************************************************/

#if !defined(__ED037TC2_H__)
#define __ED037TC2_H__


/**************************************************************************//*
  Definitions
*/
#define DATA_MASK   0x0100
#define DCX_CMD     0x0000
#define DCX_DATA    0x0001

#define PSR         0x0000
#define PWR         0x0001
#define POF         0x0002
#define PFS         0x0003
#define PON         0x0004
#define PMES        0x0005
#define BTST        0x0006
#define DSLP        0x0007
#define DTM1        0x0010
#define DSP         0x0011
#define DRF         0x0012
#define DTM2        0x0013
#define PTDM1       0x0014
#define PTDM2       0x0015
#define PDRF        0x0016
#define LUT_VCOM    0x0020
#define LUT_WW      0x0021
#define LUT_R       0x0022  // LUTR/LUTBW
#define LUT_W       0x0023  // LUTW/LUTWB
#define LUT_B       0x0024  // LUTB/LUTBB
#define LUTC_O      0x0025
#define SET_STG     0x0026
#define OSC         0x0030
#define TSC         0x0040
#define TSE         0x0041
#define TSW         0x0042
#define TSR         0x0043
#define CDI         0x0050
#define LPD         0x0051
#define TCON        0x0060
#define TRES        0x0061
#define TSGS        0x0062
#define REV         0x0070
#define FLG         0x0071
#define AMV         0x0080
#define VV          0x0081
#define VDCS        0x0082
#define PTL         0x0090
#define PTIN        0x0091
#define PTOUT       0x0092

#define PGM         0x00A0
#define APG         0x00A1
#define ROTP        0x00A2
#define CCSET       0x00E0
#define PWS         0x00E3
#define TSSET       0x00E5

#define Initial_23_16           0x00
#define Initial_15_0            0x0000  // 1K
#define Temperature0_23_16      0x00
#define Temperature0_15_0       0x0400  // 1K
#define Temperature1_23_16      0x00
#define Temperature1_15_0       0x0800
#define Temperature2_23_16      0x00
#define Temperature2_15_0       0x0C00
#define Temperature3_23_16      0x00
#define Temperature3_15_0       0x1000
#define Temperature4_23_16      0x00
#define Temperature4_15_0       0x1400
#define Temperature5_23_16      0x00
#define Temperature5_15_0       0x1800
#define Temperature6_23_16      0x00
#define Temperature6_15_0       0x1C00
#define Temperature7_23_16      0x00
#define Temperature7_15_0       0x2000
#define Temperature8_23_16      0x00
#define Temperature8_15_0       0x2400
#define Temperature9_23_16      0x00
#define Temperature9_15_0       0x2800
#define Temperature10_23_16     0x00
#define Temperature10_15_0      0x2C00


/**************************************************************************//*
  Function Prototypes
*/
void EPD_Init( void );
void spi_9b_send_9b( uint16_t dat );
void spi_9b_send( uint16_t dcx,unsigned char dat );
unsigned char spi_9b_get( void );

void EPD_Display_White( void );
void EPD_Display_Black( void );
void EPD_Display_Red( void );

void check_busy_high( void );   // If BUSYN=0 then waiting
void check_busy_low( void );    // If BUSYN=1 then waiting


#endif // !defined(__ED037TC2_H__)

