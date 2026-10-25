/** BH1745NUC RGBC sensor. SPDX-License-Identifier: MIT */
#ifndef __COLOR_BH1745_H__
#define __COLOR_BH1745_H__
#include "device.h"

#define BH1745_I2C_ADDR0 0x38
#define BH1745_I2C_ADDR1 0x39

struct Bh1745Data_t {
	uint16_t Red;
	uint16_t Green;
	uint16_t Blue;
	uint16_t Clear;
};

class ColorBh1745 : public Device {
public:
	ColorBh1745();
	bool Init(DeviceIntrf *pIntrf, uint32_t Addr = BH1745_I2C_ADDR0);
	bool Enable() override;
	void Disable() override;
	void Reset() override;
	bool SoftReset();
	// Time code 0..5: 160, 320, 640, 1280, 2560, 5120 ms; gain 1, 2, 16.
	bool Configure(uint8_t TimeCode, uint8_t Gain);
	// Returns false without modifying Data if not ready or transfer fails.
	bool Read(Bh1745Data_t &Data);
	// Source 0..3: R,G,B,C. Persistence 0..3: toggle, each, 4, 8 samples.
	bool ConfigureInterrupt(bool Enable, uint8_t Source, uint8_t Persistence,
		uint16_t Lower, uint16_t Upper, bool Latched = true);
	bool ClearInterrupt();
private:
	bool ReadReg(uint8_t Reg, uint8_t *pData, int Len);
	bool WriteReg(uint8_t Reg, const uint8_t *pData, int Len);
};
#endif
