/*--------------------------------------------------------------------------
File   : adc_nrf52_saadc.h

Author : Hoang Nguyen Hoan          June 16, 2017

Desc   : ADC implementation for Nordic nRF52 SAADC, also used on nRF5340
         and nRF91

AdcCfg_t.Rate and Rate() are in mHz. One conversion of the open channels is a
round, one result per channel, in channel order (scan mode with more than one
channel).

ADC_CONV_MODE_SINGLE : StartConversion converts one round. Without interrupt
Read waits for it and returns the results. With interrupt DEV_EVT_DATA_RDY
marks the end, the results go to the channel FIFOs (AdcChanCfg_t.pFifoMem).

ADC_CONV_MODE_CONTINUOUS : StartConversion starts the rounds until
StopConversion. The results are transferred in blocks of BlockRounds() rounds,
two blocks used in turn. At the end of a block the other one is started and
the full one goes to the channel FIFOs, then DEV_EVT_DATA_RDY. Without
interrupt Read and UpdateData take the full blocks. A channel without FIFO
gives the result of its last round. The rounds are started by:
	- the SAADC internal timer, with one open channel and Rate not 0. The rate
	  is 16 MHz / CC, CC 80 to 2047 (200 kHz to 7.8 kHz), the closest to Rate.
	- otherwise SAMPLE tasks from a PEI route the application connects with
	  ConnectTask(Pei, Evt, ADC_NRF52_PEI_TASK_SAMPLE) after StartConversion,
	  for example the compare event of a Timer. Rate is not used.
The START of the next block is triggered from the END interrupt, a round that
arrives before it is lost. Make the blocks long enough for the interrupt rate.

Oversampling (AdcCfg_t.OvrSample) is used with one open channel only. BURST is
then set on that channel, one SAMPLE task gives one result.

Single mode without interrupt can also run from PEI routes, see
PreparePeiSampling.

Limits (Limit) : a result at or above the high limit or at or below the low
limit gives a limit event, each time, and the limit handler is called from
the SAADC interrupt.

PowerOff resets all registers, Init is needed after it.

Copyright (c) 2017, I-SYST inc., all rights reserved

Permission to use, copy, modify, and distribute this software for any purpose
with or without fee is hereby granted, provided that the above copyright
notice and this permission notice appear in all copies, and none of the
names : I-SYST or its contributors may be used to endorse or
promote products derived from this software without specific prior written
permission.

For info or contributing contact : hnhoan at i-syst dot com

THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

----------------------------------------------------------------------------
Modified by          Date              Description

----------------------------------------------------------------------------*/
#ifndef __ADC_NRF52_H__
#define __ADC_NRF52_H__

#include <stdint.h>

#include "nrf.h"

#include "converters/adc_device.h"
#include "adc_nrf52_pei.h"

#define SAADC_NRF52_MAX_CHAN        8   // Max number of channels

class AdcnRF52;

/**
 * @brief	Limit event handler, called from the SAADC interrupt.
 *
 * @param	pAdc	: The ADC
 * @param	Chan	: Channel of the result
 * @param	bHigh	: true at or above the high limit, false at or below the low
 * 					  limit
 */
typedef void (*AdcnRF52LimitHandler_t)(AdcnRF52 * const pAdc, int Chan, bool bHigh);

class AdcnRF52 : public AdcDevice {
public:
	AdcnRF52();
	virtual ~AdcnRF52();
	AdcnRF52(AdcnRF52&);	// ctor not allowed

	/**
	 * @brief	ADC device initialization
	 *
	 * @param	Cfg 	: Configuration data
	 * 			pTimer	: Pointer to timer object for time stamping if available
	 * 			pIntrf	: Pointer to device interface instance
	 * 					  NULL, if self interface or internal SoC
	 * 					  such as MCU ADC pins
	 *
	 * @return	True - Success
	 */
	virtual bool Init(const AdcCfg_t &Cfg, Timer * const pTimer = NULL, DeviceIntrf * const pIntrf = NULL);

	/**
	 * @brief	Set the internal timer rate of the continuous mode.
	 *
	 * Used with one open channel only, taken at the next StartConversion.
	 *
	 * @param 	Val : Rate in mHz, 0 for no internal timer
	 *
	 * @return	Rate set in mHz, the closest the internal timer gives
	 */
	virtual uint32_t Rate(uint32_t Val);
	virtual uint32_t Rate() { return vRate; }

	/**
	 * @brief	Set conversion resolution
	 *
	 * @param 	Val : Resolution value in bits
	 *
	 * @return	Real resolution value set in bits
	 */
	virtual uint16_t Resolution(uint16_t Val);

	/**
	 * @brief	Configure channel for ADC conversion
	 *
	 * @param	pChanCfg : Array of channels to configure
	 * 			NbChan	 : Number of channels (array size)
	 *
	 * @return	True - Success
	 */
	virtual bool OpenChannel(const AdcChanCfg_t *pChanCfg, int NbChan);

	/**
	 * @brief	Close ADC channel
	 *
	 * @param 	Chan : Channel number
	 */
	virtual void CloseChannel(int Chan);

	/**
	 * @brief	Start ADC conversion process
	 *
	 * @return	True - Success
	 */
	virtual bool StartConversion();

	/**
	 * @brief	Stop ADC conversion
	 */
	virtual void StopConversion();

	/**
	 * @brief	Read converted data
	 *
	 * @param	pBuff : Buffer to receive converted data
	 * 			Len	  : Size of buffer array (total number of elements)
	 *
	 * @return	Number of ADC data in array.
	 */
	virtual int Read(AdcData_t *pBuff, int Len);

	/**
	 * @brief	Read ADC data of one channel
	 *
	 * @param 	Chan : Channel number
	 *  		pBuff : Pointer to buffer for returning data
	 *
	 * @return	true - data available
	 */
	virtual bool Read(int Chan, AdcData_t *pBuff);

	/**
	 * @brief	Execute auto calibration
	 *
	 * @return	true - success
	 */
	virtual bool Calibrate();

	virtual bool Enable();
	virtual void Disable();
	virtual void Reset();
	virtual void PowerOff();

	/**
	 * @brief	Polling continuous mode: take the full blocks.
	 *
	 * @return	true if a round is not read yet. Interrupt or single mode:
	 * 			always true.
	 */
	virtual bool UpdateData();

	/**
	 * @brief	Rounds per block of the continuous mode.
	 *
	 * Taken at the next StartConversion, after Init. The rounds used are the
	 * closest that fit the memory for two blocks of the open channels. The
	 * driver memory holds two blocks of 8 results.
	 *
	 * @param	NbRound	: Rounds per block, 1 or more
	 * @param	pMem	: Word aligned RAM for the two blocks, NULL for the driver
	 * 					  memory
	 * @param	MemSize	: Size of pMem in bytes
	 *
	 * @return	NbRound. While a conversion runs nothing is changed and the
	 * 			rounds in use are returned.
	 */
	int BlockRounds(int NbRound, int16_t *pMem = NULL, int MemSize = 0);

	/**
	 * @brief	Rounds per block, in use while a conversion runs, asked
	 * 			otherwise.
	 */
	int BlockRounds();

	/**
	 * @brief	Set the limits of an open channel.
	 *
	 * Interrupt mode only. The limits are converted to counts with the
	 * channel configuration, set them again after OpenChannel. OpenChannel
	 * and CloseChannel remove them.
	 *
	 * @param	Chan		: Channel number
	 * @param	LowVolt		: Low limit in volts
	 * @param	HighVolt	: High limit in volts, LowVolt or above
	 *
	 * @return	false if the channel is not open, no interrupt or the limits
	 * 			are not valid
	 */
	bool Limit(int Chan, float LowVolt, float HighVolt);

	/**
	 * @brief	Remove the limits of a channel.
	 */
	void LimitOff(int Chan);

	/**
	 * @brief	Set the limit event handler, NULL for none.
	 */
	void LimitHandler(AdcnRF52LimitHandler_t Handler);

	/**
	 * @brief Arm the existing channel configuration and internal
	 * EasyDMA buffer for hardware PEI sampling, without taking a sample.
	 * Only the single-conversion, noninterrupt mode is supported.
	 * Do not close channels or disable ADC while PEI routes are connected.
	 *
	 * Continuous mode does not use it: after StartConversion, without the
	 * internal timer, PeiSamplingReady is true and ConnectTask takes
	 * ADC_NRF52_PEI_TASK_SAMPLE only, the END interrupt starts the blocks.
	 */
	bool PreparePeiSampling();
	bool PeiSamplingReady() const;

	bool EnablePeiEvent(Pei &PeiDev, PeiEvent_t &Evt,
		ADC_NRF52_PEI_EVENT Event)
	{
		return PeiSamplingReady() &&
			AdcNrf52PeiBindEvent(PeiDev, &Evt, Event);
	}

	bool DisablePeiEvent(Pei &PeiDev, PeiEvent_t Evt)
	{
		return AdcNrf52PeiUnbindEvent(PeiDev, Evt);
	}

	bool ConnectTask(Pei &PeiDev, PeiEvent_t Evt, ADC_NRF52_PEI_TASK Task)
	{
		return PeiSamplingReady() &&
			AdcNrf52PeiConnectTask(PeiDev, Evt, Task);
	}

	bool DisconnectTask(Pei &PeiDev, PeiEvent_t Evt, ADC_NRF52_PEI_TASK Task)
	{
		return AdcNrf52PeiDisconnectTask(PeiDev, Evt, Task);
	}

private:
};

#endif // __ADC_NRF52_H__
