/**-------------------------------------------------------------------------
@file	gyro_sensor.cpp

@brief	Generic accelerometer sensor abstraction

@author	Hoang Nguyen Hoan
@date	Nov. 18, 2017

@license

MIT License

Copyright (c) 2019 I-SYST inc. All rights reserved.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

----------------------------------------------------------------------------*/
#include <string.h>

#include "sensors/gyro_sensor.h"

void GyroSensor::SetCalibration(float Gain[3][3], float Offset[3])
{
	float scale = (float)vData.Scale / (float)vData.Range;

	for (int i = 0; i < 3; i++)
	{
		vCalibGain[0][i] = Gain[0][i] * scale;
		vCalibGain[1][i] = Gain[1][i] * scale;
		vCalibGain[2][i] = Gain[2][i] * scale;
		vCalibOffset[i] = Offset[i] * scale;
	}
}

void GyroSensor::ClearCalibration()
{
	memset(vCalibOffset, 0, sizeof(float) * 3);
	memset(vCalibGain, 0, sizeof(float) * 9);

	vCalibGain[0][0] = (float)vData.Scale / (float)vData.Range;
	vCalibGain[1][1] = vCalibGain[0][0];
	vCalibGain[2][2] = vCalibGain[0][0];
}
