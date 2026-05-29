/**************************************************************************//*

  Copyright (C) 2019 E Ink Holdings Inc. (Ryan Liu, Eric CH Huang)
  All Rights Reserved

  Copyright (c) 2020 by Quick Turn Designs LLC
  All Rights Reserved

  This file is Confidential and Proprietary
  Any reproduction or transfer of this information file is prohibited

*//************************************************************************//*

  File:        ED037TC2.c

  Description: EPD Display interface code

*//************************************************************************//*

Date      - Author             - Change Description
---------   ------------------   --------------------------------------------
unknown   - Ryan Liu           - Original file creation
            Eric CH Huang
22feb2020 - Robert Bennett     - Port to Quick Turn Designs Project (ST Com)

*//**************************************************************************/


/**************************************************************************//*
  Include files
*/
#include <stdint.h>

#include "idelay.h"
#include "iopinctrl.h"

//#include "pindefine.h"
#include "ED037TC2nRF52.h"
//#include "DELAY.H"

// #include "LUT.h"
// #include "SPI.h"

#include "board.h"

/**************************************************************************//*
  External Data References (from LUT.c)
*/
extern uint8_t const lut_vcom[];
extern uint8_t const lut_ww[];
extern uint8_t const lut_w[];
extern uint8_t const lut_b[];
extern uint8_t const lut_r[];
extern uint8_t const lut_None[];
extern uint8_t const lut_vcom2[];
extern uint8_t const lut_w2[];
extern uint8_t const lut_b2[];
extern uint8_t const lut_r2[];


/**************************************************************************//*
  Global data
*/
uint16_t gByteCounter;


/**************************************************************************//*
  Function: check_busy_high()

    Detail: Dead-Loop, wait until BUSYN is non-zero

    Inputs: void
    Output: void
    Side Affects:
*/
void check_busy_high( void )
{
    usDelay( 2000 );

//    while(!(BUSYN));
    while (!IOPinRead(EINK_BUSY_PORT, EINK_BUSY_PIN));
}

/**************************************************************************//*
  Function: check_busy_low()

    Detail: Dead-Loop, wait until BUSYN is zero

    Inputs: void
    Output: void
    Side Affects:
*/
void check_busy_low( void )
{
    usDelay( 2000 );
    while( IOPinRead(EINK_BUSY_PORT, EINK_BUSY_PIN) );
}

//==============================================================================
//EPD initial
//==============================================================================

/**************************************************************************//*
  Function: spi_9b_init()

    Detail:Initialize SPI bus

    Inputs: void
    Output: void
    Side Affects:
*/
void spi_9b_init( void )
{
//    SCL_L;
	IOPinClear(EINK_SCK_PORT, EINK_SCK_PIN);
    //SDA_H;
	IOPinSet(EINK_SDA_PORT, EINK_SDA_PIN);
    //CSB_H;
	IOPinSet(EINK_CS_PORT, EINK_CS_PIN);
    //BS_H;
	IOPinClear(EINK_BSI_PORT, EINK_BSI_PIN);
    usDelay(10);  // 5+(1+1)*6+3 = 20 us
}

/**************************************************************************//*
  Function: EPD_Init()

    Detail:Initialize EPD display

    Inputs: void
    Output: void
    Side Affects:
*/
void EPD_Init( void )
{
    uint8_t temp;

    //spi_9b_init();      /* initialize spi interface */

    //RSTN_L;             // Reset
    IOPinClear(EINK_RESET_PORT, EINK_RESET_PIN);
    usDelay(100);
//    RSTN_H;
    IOPinSet(EINK_RESET_PORT, EINK_RESET_PIN);
    usDelay(1000);

    spi_9b_send_9b(PON);
    check_busy_high();

    spi_9b_send_9b(PWR);
    spi_9b_send(1,0x03);
    spi_9b_send(1,0x07);
    spi_9b_send(1,0x0D);
    spi_9b_send(1,0x0D);
    spi_9b_send(1,0x00);

    spi_9b_send_9b(BTST);
    spi_9b_send(1,0x57);
    spi_9b_send(1,0x63);
    spi_9b_send(1,0x31);

    spi_9b_send_9b(TCON);
    spi_9b_send(1,0x04);

    spi_9b_send_9b(PSR);
    spi_9b_send(1,0x6F);

    spi_9b_send_9b(OSC);
    spi_9b_send(1,0x3C);

    spi_9b_send_9b(TRES);
    spi_9b_send(1,0x01);
    spi_9b_send(1,0x18);
    spi_9b_send(1,0x01);
    spi_9b_send(1,0xE0);

    spi_9b_send_9b(CDI);
    spi_9b_send(1,0x37);

    spi_9b_send_9b(SET_STG);
    spi_9b_send(1,0x0F);

    spi_9b_send_9b(TSGS);
    spi_9b_send(1,0x00);
    spi_9b_send(1,0x00);
    spi_9b_send(1,0x00);
    spi_9b_send(1,0x00);

    spi_9b_send_9b(PON);
    check_busy_high();
    spi_9b_send_9b(AMV);
    spi_9b_send(1,0x11);
    check_busy_high();

    spi_9b_send_9b(VV);
    //temp = spi_9b_get();
    spi_9b_send_9b(VDCS);
    spi_9b_send(1,temp);

    spi_9b_send_9b(POF);
    check_busy_low();
    gByteCounter = 16800;   // (Pixel size)/(Pixel) = (280*480)/8
}


/**************************************************************************//*
  Function: EPD_Load_LUTC()

    Detail: load waveform

    Inputs: void
    Output: void
    Side Affects:
*/
void EPD_Load_LUTC( uint16_t LUT, uint8_t const *LUT_Value )
{
    int i;

    spi_9b_send_9b(LUT);

    for( i=0; i<60; i++ )
    {
        spi_9b_send (DCX_DATA, *LUT_Value );
        LUT_Value++;
    }
}

/**************************************************************************//*
  Function: EPD_Load_LUT()

    Detail: Load ???

    Inputs: void
    Output: void
    Side Affects:
*/
void EPD_Load_LUT( uint16_t LUT, uint8_t const *LUT_Value )
{
    int i;

    spi_9b_send_9b(LUT);

    for( i=0; i<42; i++ )
    {
        spi_9b_send( DCX_DATA, *LUT_Value );
        LUT_Value++;
    }
}

/**************************************************************************//*
  Function: CLEAR_LUT()

    Detail: Clear LUT

    Inputs: void
    Output: void
    Side Affects:
*/
void CLEAR_LUT( void )
{
    spi_9b_send_9b( PON );

    check_busy_high();

    EPD_Load_LUT( LUT_VCOM, lut_None );
    EPD_Load_LUT( LUT_WW, lut_None );
    EPD_Load_LUT( LUT_R, lut_None );
    EPD_Load_LUT( LUT_W, lut_None );
    EPD_Load_LUT( LUT_B, lut_None );

    spi_9b_send_9b( POF );

    check_busy_low();
}

/**************************************************************************//*
  Function: Upload_Temperature_LUT()

    Detail: Clear LUT

    Inputs: void
    Output: void
    Side Affects:
*/
void Upload_Temperature_LUT( void )
{

  EPD_Load_LUTC( LUT_VCOM, lut_vcom );

  EPD_Load_LUT( LUT_WW, lut_ww );
  EPD_Load_LUTC( LUT_R, lut_r );
  EPD_Load_LUT( LUT_W, lut_w );
  EPD_Load_LUT( LUT_B, lut_b );
}

/**************************************************************************//*
  Function: EPD_Display_White()

    Detail: EPD Display White

    Inputs: void
    Output: void
    Side Affects:
*/
void EPD_Display_White( void )
{
    uint16_t i;

    spi_9b_send_9b( PON );
    check_busy_high();

    spi_9b_send_9b( DTM1 );

    for( i=0; i < gByteCounter; i++ )
    {
        spi_9b_send( 1, 0 );
    }

    spi_9b_send_9b( DTM2 );

    for( i=0; i < gByteCounter; i++ )
    {
        spi_9b_send( 1, 0xA5 );
    }

    Upload_Temperature_LUT();
    spi_9b_send_9b( DRF );
    check_busy_high();

    //spi_9b_send_9b( POF );
    //check_busy_low();
}


/**************************************************************************//*
  Function: EPD_Display_Black()

    Detail: EPD Display Black

    Inputs: void
    Output: void
    Side Affects:
*/
void EPD_Display_Black( void )
{
    uint16_t i;

    spi_9b_send_9b( PON );
    check_busy_high();

    spi_9b_send_9b( DTM1 );

    for( i=0; i < gByteCounter; i++ )
    {
        spi_9b_send( 1, 0x00 );
    }
        spi_9b_send_9b( DTM2 );

        for( i=0; i < gByteCounter; i++ )
        {
            spi_9b_send( 1, 0 );
        }

        Upload_Temperature_LUT();

        spi_9b_send_9b( DRF );
        check_busy_high();

      //  spi_9b_send_9b( POF );
        //check_busy_low();
    //}
}



/**************************************************************************//*
  3-wire SPI transmission Protocol
*/

/**************************************************************************//*
  Function: spi_9b_send_9b()

    Detail: EPD Display Black

    Inputs: uint16_t dat -
    Output: void
    Side Affects:
*/
#if 0
void spi_9b_send_9b( uint16_t dat )
{
    if( (dat & DATA_MASK) == DATA_MASK )
    {
        spi_9b_send( DCX_DATA, (uint8_t)dat );
    }
    else
    {
        spi_9b_send( DCX_CMD, (uint8_t)dat );
    }
}
/**************************************************************************//*
  Function: spi_9b_send()

    Detail: EPD Display Black

    Inputs: uint16_t dcx -
            uint8_t dat  -
    Output: void
    Side Affects:
*/
void spi_9b_send( uint16_t dcx, uint8_t dat )
{
    uint8_t i;

    //CSB_L;

    IOPinClear(EINK_CS_PORT, EINK_CS_PIN);

    usDelay(1);

    if( dcx )
    {
//        SDA_H;  // 1 for DCX_DATA
    	IOPinSet(EINK_DC_PORT, EINK_DC_PIN);
    	//IOPinSet(EINK_SDA_PORT, EINK_SDA_PIN);
    }
    else
    {
//        SDA_L;  // 0 for DCX_CMD
    	IOPinClear(EINK_DC_PORT, EINK_DC_PIN);
    	//IOPinClear(EINK_SDA_PORT, EINK_SDA_PIN);
    }

//    SCL_H;
	//IOPinSet(EINK_SCK_PORT, EINK_SCK_PIN);


    usDelay(1);

//    SCL_L;
	//IOPinClear(EINK_SCK_PORT, EINK_SCK_PIN);

    usDelay(20);

    for( i=0; i < 8 ; i++ )
    {
        if( dat & 0x80 )
        {
//            SDA_H;
        	IOPinSet(EINK_SDA_PORT, EINK_SDA_PIN);
        }
        else
        {
//            SDA_L;
        	IOPinClear(EINK_SDA_PORT, EINK_SDA_PIN);
        }

        usDelay(1);

//        SCL_H;
    	IOPinSet(EINK_SCK_PORT, EINK_SCK_PIN);

        usDelay(10);

//        SCL_L;
    	IOPinClear(EINK_SCK_PORT, EINK_SCK_PIN);

        dat = dat << 1;
    }

//    SDA_L;
	IOPinClear(EINK_SDA_PORT, EINK_SDA_PIN);

    usDelay(20);

   // CSB_H;
    IOPinSet(EINK_CS_PORT,EINK_CS_PIN);

    usDelay(1);
}
#endif
/**************************************************************************//*
  Function: spi_9b_get()

    Detail: spi get

    Inputs: void
    Output: uint8_t -
    Side Affects:
*/
uint8_t spi_9b_get( void )
{
    int i;

    uint8_t DATA_BUF = 0x00;

//    CSB_L;
    IOPinClear(EINK_CS_PORT, EINK_CS_PIN);

    usDelay(1);
//    SDA_H;          //1 for data
	IOPinSet(EINK_DC_PORT, EINK_DC_PIN);
//	IOPinSet(EINK_SDA_PORT, EINK_SDA_PIN);
    usDelay(1);
//    SCL_H;
//	IOPinSet(EINK_SCK_PORT,EINK_SCK_PIN);
    usDelay(1);
//    SCL_L;
//	IOPinClear(EINK_SCK_PORT, EINK_SCK_PIN);
    usDelay(1);
    //P4DIR = 0x80;   // SDA input
    IOPinSetDir(EINK_SDA_PORT, EINK_SDA_PIN, IOPINDIR_INPUT);
    usDelay(3);

    for( i=0; i < 8; i++ )
    {
        DATA_BUF = DATA_BUF << 1;
//        SCL_H;
    	IOPinSet(EINK_SCK_PORT, EINK_SCK_PIN);

       // if(SDA_IN)
    	if (IOPinRead(EINK_SDA_PORT, EINK_SDA_PIN))
        {
            DATA_BUF|=0x01;
        }

//        SCL_L;
    	IOPinClear(EINK_SCK_PORT, EINK_SCK_PIN);
        usDelay(1);
    }

    //P4DIR = 0xC0;// SDA output
    IOPinSetDir(EINK_SDA_PORT, EINK_SDA_PIN, IOPINDIR_OUTPUT);
//    SDA_L;
	IOPinClear(EINK_SDA_PORT, EINK_SDA_PIN);

    usDelay(1);

//    CSB_H;
    IOPinSet(EINK_CS_PORT, EINK_CS_PIN);

    usDelay(1);

    return( DATA_BUF );
}


