#define __ED037TC1_C__
/*************************************************************************************
 * Copyright (C) 2019 E Ink Holdings Inc. (Ryan Liu, Eric CH Huang) All Rights Reserved *
 *************************************************************************************/
//#include "pindefine.h"
#include "ED037TC1.h"
//#include "delay.h"
#include "LUT.h"

#include "iopinctrl.h"
#include "idelay.h"
//#include "SPI.h"
#include "board.h"

void check_busy_high(void)// If BUSYN=0 then waiting
{
    usDelay( 2000 );
//while(!(BUSYN));

//    while(!(BUSYN));
    while (!IOPinRead(EINK_BUSY_PORT, EINK_BUSY_PIN));
}

void check_busy_low(void)// If BUSYN=1 then waiting
{
    usDelay( 2000 );
    while( IOPinRead(EINK_BUSY_PORT, EINK_BUSY_PIN) );
//while(BUSYN);
}
/*
void spi_9b_init(void)
{
  SCL_L;
  SDA_H;
  CSB_H;
  BS_H;
  delay(10);  // 5+(1+1)*6+3 = 20 us
}
*/
void EPD_Init(void)
{
  //spi_9b_init();
//  RSTN_L; // Reset
	IOPinClear(EINK_RESET_PORT, EINK_RESET_PIN);
    usDelay(100);
  //delay(3000);
//  RSTN_H;
//  delay(1000);
    IOPinSet(EINK_RESET_PORT, EINK_RESET_PIN);
    usDelay(1000);

  //CSB_L;
    IOPinClear(EINK_CS_PORT, EINK_CS_PIN);
    spi_command(SWRESET);
    //CSB_H;
    IOPinSet(EINK_CS_PORT, EINK_CS_PIN);
    check_busy_low();
  
    IMAGE_COUNTER = 16800;
  
    //CSB_L;
    IOPinClear(EINK_CS_PORT, EINK_CS_PIN);
    spi_command(0x46);
  	spi_data(0xF7);
  //CSB_H;
    IOPinSet(EINK_CS_PORT, EINK_CS_PIN);
  	check_busy_low();

  	//CSB_L;
  	IOPinClear(EINK_CS_PORT, EINK_CS_PIN);
  	spi_command(0x47);
  	spi_data(0xF7);
  //CSB_H;
    IOPinSet(EINK_CS_PORT, EINK_CS_PIN);
  	check_busy_low();
  
  
  //CSB_L;
	IOPinClear(EINK_CS_PORT, EINK_CS_PIN);
	spi_command(GATE_SET); // setting gaet number
	spi_data(0xDF);
	spi_data(0x01);
	spi_data(0x00);
	//CSB_H;
    IOPinSet(EINK_CS_PORT, EINK_CS_PIN);

	//CSB_L;
    IOPinClear(EINK_CS_PORT, EINK_CS_PIN);
	spi_command(VGH); // set gate voltage
	spi_data(0x00);
	//CSB_H;
    IOPinSet(EINK_CS_PORT, EINK_CS_PIN);

	//CSB_L;
    IOPinClear(EINK_CS_PORT, EINK_CS_PIN);
	spi_command(VSHL); // set source voltage
	spi_data(0x41);
	spi_data(0xA8);
	spi_data(0x32);
	//CSB_H;
    IOPinSet(EINK_CS_PORT, EINK_CS_PIN);

	//CSB_L;
    IOPinClear(EINK_CS_PORT, EINK_CS_PIN);
	spi_command(DEY); // set data entry sequence
	spi_data(0x03);
	//CSB_H;
    IOPinSet(EINK_CS_PORT, EINK_CS_PIN);

	//CSB_L;
    IOPinClear(EINK_CS_PORT, EINK_CS_PIN);
	spi_command(BDW); // set border
	spi_data(0x00);
	//CSB_H;
    IOPinSet(EINK_CS_PORT, EINK_CS_PIN);

	//CSB_L;
    IOPinClear(EINK_CS_PORT, EINK_CS_PIN);
	spi_command(BTST); // set booster strength
	spi_data(0xAE);
	spi_data(0xC7);
	spi_data(0xC3);
	spi_data(0xC0);
	spi_data(0xC0);
	//CSB_H;
    IOPinSet(EINK_CS_PORT, EINK_CS_PIN);

	//CSB_L;
    IOPinClear(EINK_CS_PORT, EINK_CS_PIN);
	spi_command(TSC); // set internal sensor on
	spi_data(0x80);
	//CSB_H;
    IOPinSet(EINK_CS_PORT, EINK_CS_PIN);

	//CSB_L;
    IOPinClear(EINK_CS_PORT, EINK_CS_PIN);
	spi_command(VCOM_WT); // set vcom value
	spi_data(0x44);
	//CSB_H;
    IOPinSet(EINK_CS_PORT, EINK_CS_PIN);

	//CSB_L;
    IOPinClear(EINK_CS_PORT, EINK_CS_PIN);
	spi_command(DIS_OPT); // set display option, these setting turn on previous function
	spi_data(0x00);
	spi_data(0xFF);
	spi_data(0xFF);
	spi_data(0xFF);
	spi_data(0xFF);
	spi_data(0x4F);
	spi_data(0xFF);
	spi_data(0xFF);
	spi_data(0xFF);
	spi_data(0xFF);
	//CSB_H;
    IOPinSet(EINK_CS_PORT, EINK_CS_PIN);



	//CSB_L;
    IOPinClear(EINK_CS_PORT, EINK_CS_PIN);
	spi_command(X_ADDR); // setting X direction start/end position of RAM
	spi_data(0x00);
	spi_data(0x00);
	spi_data(0x17);
	spi_data(0x01);
	//CSB_H;
    IOPinSet(EINK_CS_PORT, EINK_CS_PIN);

	//CSB_L;
    IOPinClear(EINK_CS_PORT, EINK_CS_PIN);
	spi_command(Y_ADDR); // setting Y direction start/end position of RAM
	spi_data(0x00);
	spi_data(0x00);
	spi_data(0xDF);
	spi_data(0x01);
	//CSB_H;
    IOPinSet(EINK_CS_PORT, EINK_CS_PIN);

	//CSB_L;
    IOPinClear(EINK_CS_PORT, EINK_CS_PIN);
	spi_command(DSP_SEQ);
	spi_data(0xCF);
	//CSB_H;
    IOPinSet(EINK_CS_PORT, EINK_CS_PIN);

	//CSB_L;
    IOPinClear(EINK_CS_PORT, EINK_CS_PIN);
	spi_command(X_ADDRC); // setting initial counter of X direction in RAM
	spi_data(0x00);
	spi_data(0x00);
	//CSB_H;
    IOPinSet(EINK_CS_PORT, EINK_CS_PIN);


	//CSB_L;
    IOPinClear(EINK_CS_PORT, EINK_CS_PIN);
	spi_command(Y_ADDRC);// setting initial counter of Y direction in RAM
	spi_data(0x00);
	spi_data(0x00);
	//CSB_H;
    IOPinSet(EINK_CS_PORT, EINK_CS_PIN);

    InitialDisplay();
}

void InitialDisplay(void)
{
	// Initializes the Display Memory RAM
	//CSB_L;
	IOPinClear(EINK_CS_PORT, EINK_CS_PIN);
	spi_command(0x46);
	spi_data(0xF7);
	//CSB_H;
    IOPinSet(EINK_CS_PORT, EINK_CS_PIN);
    check_busy_low();
	//CSB_L;
	IOPinClear(EINK_CS_PORT, EINK_CS_PIN);
	spi_command(0x47);
	spi_data(0xF7);
	//CSB_H;
	IOPinSet(EINK_CS_PORT, EINK_CS_PIN);
	check_busy_low();

	//CSB_L;
	IOPinClear(EINK_CS_PORT, EINK_CS_PIN);
	spi_command(X_ADDRC);
	spi_data(0);
	spi_data(0);
	//CSB_H;
	IOPinSet(EINK_CS_PORT, EINK_CS_PIN);

	//CSB_L;
	IOPinClear(EINK_CS_PORT, EINK_CS_PIN);
	spi_command(Y_ADDRC);
	spi_data(0);
	spi_data(0);
	//CSB_H;
	IOPinSet(EINK_CS_PORT, EINK_CS_PIN);

	//CSB_L;
	IOPinClear(EINK_CS_PORT, EINK_CS_PIN);
	// Sends White data to BW RAM
	spi_command(RAM_BW);
	for(int i = 0 ; i < IMAGE_COUNTER; i++)
	{
		spi_data(0xFF);
	}
	//CSB_H;
	IOPinSet(EINK_CS_PORT, EINK_CS_PIN);

	//CSB_L;
	IOPinClear(EINK_CS_PORT, EINK_CS_PIN);
	spi_command(X_ADDRC);
	spi_data(0);
	spi_data(0);
	//CSB_H;
	IOPinSet(EINK_CS_PORT, EINK_CS_PIN);

	//CSB_L;
	IOPinClear(EINK_CS_PORT, EINK_CS_PIN);
	spi_command(Y_ADDRC);
	spi_data(0);
	spi_data(0);
	//CSB_H;
	IOPinSet(EINK_CS_PORT, EINK_CS_PIN);

	//CSB_L;
	IOPinClear(EINK_CS_PORT, EINK_CS_PIN);
	// Sends Black data to RED RAM (aka Previous RAM)
	spi_command(RAM_RED);
	for(int i = 0 ; i < IMAGE_COUNTER; i++)
	{
		spi_data(0x00);
	}
	//CSB_H;
	IOPinSet(EINK_CS_PORT, EINK_CS_PIN);

	// Load the waveform type (flashing, non-flashing, etc.)
	Upload_Temperature_LUT();
	//CSB_L;
	IOPinClear(EINK_CS_PORT, EINK_CS_PIN);
	spi_command(DSP_SEQ);
	spi_data(0xcf);//DisplaySeq);
	//CSB_H;
	IOPinSet(EINK_CS_PORT, EINK_CS_PIN);

	//CSB_L;
	IOPinClear(EINK_CS_PORT, EINK_CS_PIN);
	spi_command(DSP_ACT);
	//CSB_H;
	IOPinSet(EINK_CS_PORT, EINK_CS_PIN);
	check_busy_low();
}

void EPD_Load_LUT(unsigned int LUT, unsigned char const *LUT_Value, unsigned int LUT_Counter)
{
  unsigned int i;
  
 // CSB_L;
	IOPinClear(EINK_CS_PORT, EINK_CS_PIN);
  spi_command(LUT);
  for (i = 0; i < LUT_Counter; i++)
  {
    spi_data(*LUT_Value);
    LUT_Value++;
  }
  //CSB_H;
	IOPinSet(EINK_CS_PORT, EINK_CS_PIN);
}
void Upload_Temperature_LUT()
{
  EPD_Load_LUT(LUT_REG, lut_vcom_gc, 105); 
}

//==============================================================================
//EPD Display
//==============================================================================

void EPD_Display_White(void)
{
  unsigned int i;

  //CSB_L;
	IOPinClear(EINK_CS_PORT, EINK_CS_PIN);
  spi_command(X_ADDRC);
  spi_data(0x00);
  spi_data(0x00);
  //CSB_H;
	IOPinSet(EINK_CS_PORT, EINK_CS_PIN);

  //CSB_L;
	IOPinClear(EINK_CS_PORT, EINK_CS_PIN);
  spi_command(Y_ADDRC);
  spi_data(0x00);
  spi_data(0x00);
  //CSB_H;
	IOPinSet(EINK_CS_PORT, EINK_CS_PIN);
  

  //CSB_L;
	IOPinClear(EINK_CS_PORT, EINK_CS_PIN);
  spi_command(RAM_BW);
  for (i = 0; i < IMAGE_COUNTER; i++)
  {
    spi_data(0xfF);
  }
  
  Upload_Temperature_LUT();
  
  //CSB_L;
	IOPinClear(EINK_CS_PORT, EINK_CS_PIN);
  spi_command(DSP_ACT);
  //CSB_H;
	IOPinSet(EINK_CS_PORT, EINK_CS_PIN);
  check_busy_low();    
  //CSB_H;
	IOPinSet(EINK_CS_PORT, EINK_CS_PIN);
}

void EPD_Display_Black(void)
{
 
	unsigned long i;

	//CSB_L;
	IOPinClear(EINK_CS_PORT, EINK_CS_PIN);
	spi_command(X_ADDRC);
	spi_data(0x00);
	spi_data(0x00);
	// CSB_H;
	IOPinSet(EINK_CS_PORT, EINK_CS_PIN);

	//CSB_L;
	IOPinClear(EINK_CS_PORT, EINK_CS_PIN);
	spi_command(Y_ADDRC);
	spi_data(0x00);
	spi_data(0x00);
	//CSB_H;
	IOPinSet(EINK_CS_PORT, EINK_CS_PIN);


	//CSB_L;
	IOPinClear(EINK_CS_PORT, EINK_CS_PIN);
	spi_command(RAM_BW);
	for (i = 0; i < IMAGE_COUNTER; i++)
	{
		spi_data(0);
	}

	Upload_Temperature_LUT();


	//CSB_L;
	IOPinClear(EINK_CS_PORT, EINK_CS_PIN);
	spi_command(DSP_ACT);
	// CSB_H;
	check_busy_low();
	// CSB_H;
	IOPinSet(EINK_CS_PORT, EINK_CS_PIN);
}


//==============================================================================
//3-wire SPI transmission Protocol
//==============================================================================
unsigned char spi_9b_get(void)
{
	int i;
	unsigned char DATA_BUF=0x00;

//  CSB_L;
	IOPinClear(EINK_CS_PORT, EINK_CS_PIN);

	usDelay(1);
  
//  SDA_H; //1 for data
	IOPinSet(EINK_DC_PORT, EINK_DC_PIN);
	usDelay(1);
  
  //SCL_H;
	IOPinSet(EINK_SCK_PORT,EINK_SCK_PIN);
	usDelay(1);

//  SCL_L;
	IOPinClear(EINK_SCK_PORT, EINK_SCK_PIN);
	usDelay(1);

//	P4DIR = 0x80;// SDA input
    IOPinSetDir(EINK_SDA_PORT, EINK_SDA_PIN, IOPINDIR_INPUT);
    usDelay(3);
	for(i=0;i<8;i++)
	{
		DATA_BUF=DATA_BUF<<1;
		//SCL_H;
    	IOPinSet(EINK_SCK_PORT, EINK_SCK_PIN);

    	//if(SDA_IN) DATA_BUF|=0x01;
    	if (IOPinRead(EINK_SDA_PORT, EINK_SDA_PIN))
        {
            DATA_BUF|=0x01;
        }
//		SCL_L;
    	IOPinClear(EINK_SCK_PORT, EINK_SCK_PIN);
        usDelay(1);
	}
//	P4DIR = 0xC0;// SDA output
    IOPinSetDir(EINK_SDA_PORT, EINK_SDA_PIN, IOPINDIR_OUTPUT);

    //SDA_L;
	IOPinClear(EINK_SDA_PORT, EINK_SDA_PIN);

	usDelay(1);

	//CSB_H;
    IOPinSet(EINK_CS_PORT, EINK_CS_PIN);

	usDelay(1);

	return DATA_BUF;
}
void spi_command(unsigned char dat)
{
#if 0
	spi_9b_send(0, dat);
#else
	unsigned char i;

	usDelay(1);
//	SDA_L; //0 for DCX_CMD
//	IOPinClear(EINK_SDA_PORT, EINK_SDA_PIN);
//	SCL_H;
//	IOPinSet(EINK_SCK_PORT, EINK_SCK_PIN);
//	usDelay(1);
//	SCL_L;
//	IOPinClear(EINK_SCK_PORT, EINK_SCK_PIN);

	IOPinClear(EINK_DC_PORT, EINK_DC_PIN);
	usDelay(1);
	for (i = 0; i < 8; i++)
	{
		if (dat & 0x80)
		{
//			SDA_H;
			IOPinSet(EINK_SDA_PORT, EINK_SDA_PIN);
		}
		else
		{
//			SDA_L;
			IOPinClear(EINK_SDA_PORT, EINK_SDA_PIN);
		}
		msDelay(1);
		//SCL_H;
		IOPinSet(EINK_SCK_PORT, EINK_SCK_PIN);
		usDelay(1);
//		SCL_L;
		IOPinClear(EINK_SCK_PORT, EINK_SCK_PIN);
		dat = dat << 1;
	}
#endif
}

void spi_data(unsigned char dat)
{
#if 0
	spi_9b_send(1, dat);
#else
  unsigned char i;
  
  usDelay(1);
  //SDA_H; //1 for DCX_DATA
  //SCL_H;
//  delay(1);
//  SCL_L;
	IOPinSet(EINK_DC_PORT, EINK_DC_PIN);
  usDelay(1);
  for (i = 0; i < 8; i++)
  {
    if (dat & 0x80)
    {
//      SDA_H;
		IOPinSet(EINK_SDA_PORT, EINK_SDA_PIN);
    }
    else
    {
//      SDA_L;
		IOPinClear(EINK_SDA_PORT, EINK_SDA_PIN);
    }
    usDelay(1);
//    SCL_H;
	IOPinSet(EINK_SCK_PORT, EINK_SCK_PIN);
    usDelay(1);
//    SCL_L;
	IOPinClear(EINK_SCK_PORT, EINK_SCK_PIN);
    dat = dat << 1;
  }
#endif
}

//==============================================================================
