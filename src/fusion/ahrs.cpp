/**-------------------------------------------------------------------------
@file	ahrs.cpp

@brief	Generic AHRS (attitude and heading reference system)

This a generic abstraction layer for AHRS sensor fusion.  It is a mean to
provide a common interface to different sensor fusion library out there.

@author	Hoang Nguyen Hoan
@date	Aug. 1, 2018

@license

Copyright (c) 2018, I-SYST inc., all rights reserved

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

----------------------------------------------------------------------------*/

#include "fusion/ahrs.h"
/*
bool Ahrs::Init(const AhrsCfg_t &Cfg, uint32_t DevAddr, DeviceIntrf * const pIntrf, Timer * const pTimer)
{
	SetEvtHandler(Cfg.EvtHandler);
	vpTimer = pTimer;
	Interface(pIntrf);
	DeviceAddess(DevAddr);

	return true;
}
*/
bool Ahrs::Init(const AhrsCfg_t &Cfg, AccelSensor * const pAccel, GyroSensor * const pGyro, MagSensor * const pMag)
{
	vpAccel = pAccel;
	vpGyro = pGyro;
	vpMag = pMag;
	SetEvtHandler(Cfg.EvtHandler);

	return true;
}

AHRS_FEATURE Ahrs::Feature(AHRS_FEATURE FeatureBit, bool bEnDis)
{
	uint32_t bit = 0x8000;

	while (bit != 0)
	{
		switch (FeatureBit & bit)
		{
			case AHRS_FEATURE_EULER:
				Euler(bEnDis);
				break;
			case AHRS_FEATURE_QUATERNION:
				//Quaternion(bEnDis);
				break;
			case AHRS_FEATURE_COMPASS:
				Compass(bEnDis);
				break;
			case AHRS_FEATURE_GRAVITY:
				//Gravity(bEnDis);
				break;
			case AHRS_FEATURE_EXTERNAL_ACCEL:
				break;
			case AHRS_FEATURE_TAP:
				break;
			case AHRS_FEATURE_ROTATION:
				break;
			case AHRS_FEATURE_VIBRATION:
				break;
			case AHRS_FEATURE_PEDOMETER:
				break;
			case AHRS_FEATURE_CYCLING:
				break;
		}

		bit >>= 1;
	}
	if (bEnDis == true)
	{
		// Enable
		vActiveFeature |= FeatureBit;
	}
	else
	{
		// disable
		vActiveFeature &= ~FeatureBit;
	}

	return vActiveFeature;
}




