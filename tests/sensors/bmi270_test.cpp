#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "sensors/ag_bmi270.h"

uint64_t g_DelayUs;

class MockIntrf : public DeviceIntrf {
public:
	MockIntrf(DEVINTRF_TYPE Type = DEVINTRF_TYPE_I2C)
	{
		vType = Type;
		vRate = Type == DEVINTRF_TYPE_SPI ? 8000000 : 400000;
		vFailReadReg = -1;
		vFailWriteReg = -1;
		vBadInitStatus = false;
		vConfigBytes = 0;
		vFifoLen = 0;
		vLastDevAddr = 0;
		vDummyReads = 0;
		ResetRegisters();
	}

	operator DevIntrf_t * () override { return NULL; }
	DEVINTRF_TYPE Type() override { return vType; }
	uint32_t Rate(uint32_t Rate) override { vRate = Rate; return Rate; }
	uint32_t Rate(void) override { return vRate; }
	void Disable(void) override {}
	void Enable(void) override {}
	int Rx(uint32_t, uint8_t *, int) override { return 0; }
	int Tx(uint32_t, const uint8_t *, int) override { return 0; }
	bool StartRx(uint32_t) override { return false; }
	int RxData(uint8_t *, int) override { return 0; }
	void StopRx(void) override {}
	bool StartTx(uint32_t) override { return false; }
	int TxData(const uint8_t *, int) override { return 0; }
	void StopTx(void) override {}
	bool RequestToSend(int) override { return true; }
	void Reset(void) override {}

	int Read(uint32_t DevAddr, const uint8_t *pCmd, int CmdLen,
		uint8_t *pData, int Len) override
	{
		uint8_t reg;
		int dataLen;
		int offset;
		int i;

		if (pCmd == NULL || CmdLen != 1 || pData == NULL || Len <= 0)
		{
			return 0;
		}

		vLastDevAddr = DevAddr;
		reg = pCmd[0] & 0x7F;
		if ((int)reg == vFailReadReg)
		{
			return 0;
		}

		offset = 0;
		dataLen = Len;
		if (vType == DEVINTRF_TYPE_SPI)
		{
			pData[0] = 0;
			offset = 1;
			dataLen--;
			vDummyReads++;
		}
		if (dataLen < 0)
		{
			return 0;
		}

		if (reg == BMI270_FIFO_DATA_REG)
		{
			if (dataLen > vFifoLen)
			{
				dataLen = vFifoLen;
			}
			memcpy(&pData[offset], vFifo, (size_t)dataLen);
			if (dataLen < vFifoLen)
			{
				memmove(vFifo, &vFifo[dataLen], (size_t)(vFifoLen - dataLen));
			}
			vFifoLen -= dataLen;
			return dataLen + offset;
		}

		if (reg == BMI270_FIFO_LENGTH_LSB_REG && dataLen >= 2)
		{
			pData[offset] = (uint8_t)vFifoLen;
			pData[offset + 1] = (uint8_t)(vFifoLen >> 8);
			for (i = 2; i < dataLen; i++)
			{
				pData[offset + i] = vReg[(uint8_t)(reg + i)];
			}
			return dataLen + offset;
		}

		for (i = 0; i < dataLen; i++)
		{
			pData[offset + i] = vReg[(uint8_t)(reg + i)];
		}
		return dataLen + offset;
	}

	int Write(uint32_t DevAddr, const uint8_t *pCmd, int CmdLen,
		const uint8_t *pData, int Len) override
	{
		uint8_t reg;
		int i;

		if (pCmd == NULL || CmdLen != 1 || pData == NULL || Len <= 0)
		{
			return 0;
		}

		vLastDevAddr = DevAddr;
		reg = pCmd[0] & 0x7F;
		if ((int)reg == vFailWriteReg)
		{
			return 0;
		}

		if (reg == BMI270_CMD_REG && Len == 1 && pData[0] == BMI270_CMD_SOFTRESET)
		{
			ResetRegisters();
			return 1;
		}
		if (reg == BMI270_CMD_REG && Len == 1 && pData[0] == BMI270_CMD_FIFO_FLUSH)
		{
			vFifoLen = 0;
			vReg[reg] = pData[0];
			return 1;
		}
		if (reg == BMI270_INIT_DATA_REG)
		{
			vConfigBytes += (unsigned)Len;
			return Len;
		}

		for (i = 0; i < Len; i++)
		{
			vReg[(uint8_t)(reg + i)] = pData[i];
		}
		if (reg == BMI270_INIT_CTRL_REG && Len == 1 && pData[0] == 1)
		{
			vReg[BMI270_INTERNAL_STATUS_REG] =
				vBadInitStatus ? 0 : BMI270_INTERNAL_STATUS_MESSAGE_INIT_OK;
		}
		return Len;
	}

	void ResetRegisters()
	{
		memset(vReg, 0, sizeof(vReg));
		vReg[BMI270_CHIP_ID_REG] = BMI270_CHIP_ID;
	}

	uint8_t vReg[256];
	uint8_t vFifo[256];
	DEVINTRF_TYPE vType;
	uint32_t vRate;
	int vFailReadReg;
	int vFailWriteReg;
	bool vBadInitStatus;
	unsigned vConfigBytes;
	int vFifoLen;
	uint32_t vLastDevAddr;
	unsigned vDummyReads;
};

static AccelSensorCfg_t AccelConfig(bool Fifo = false)
{
	AccelSensorCfg_t cfg = {};
	cfg.DevAddr = BMI270_I2C_7BITS_DEVADDR0;
	cfg.OpMode = SENSOR_OPMODE_CONTINUOUS;
	cfg.Freq = 100000;
	cfg.Scale = 2;
	cfg.FltrFreq = 0;
	cfg.Inter = 0;
	cfg.IntPol = DEVINTR_POL_HIGH;
	cfg.IntHandler = NULL;
	cfg.bFifoEn = Fifo;
	return cfg;
}

static GyroSensorCfg_t GyroConfig(bool Fifo = false)
{
	GyroSensorCfg_t cfg = {};
	cfg.DevAddr = BMI270_I2C_7BITS_DEVADDR0;
	cfg.OpMode = SENSOR_OPMODE_CONTINUOUS;
	cfg.Freq = 100000;
	cfg.Sensitivity = 2000;
	cfg.FltrFreq = 0;
	cfg.Inter = 0;
	cfg.IntPol = DEVINTR_POL_HIGH;
	cfg.bFifoEn = Fifo;
	return cfg;
}

static TempSensorCfg_t TempConfig()
{
	TempSensorCfg_t cfg = {};
	cfg.DevAddr = BMI270_I2C_7BITS_DEVADDR0;
	cfg.OpMode = SENSOR_OPMODE_CONTINUOUS;
	return cfg;
}

static void Put16(uint8_t *pData, int16_t Value)
{
	pData[0] = (uint8_t)Value;
	pData[1] = (uint8_t)((uint16_t)Value >> 8);
}

static void SetSensorTime(MockIntrf &Bus, uint32_t Ticks)
{
	Bus.vReg[BMI270_SENSOR_TIME0_REG] = (uint8_t)Ticks;
	Bus.vReg[BMI270_SENSOR_TIME1_REG] = (uint8_t)(Ticks >> 8);
	Bus.vReg[BMI270_SENSOR_TIME2_REG] = (uint8_t)(Ticks >> 16);
}

static void TestInitAndConfiguration()
{
	MockIntrf bus;
	AgBmi270 imu;
	AccelSensorCfg_t acfg;
	GyroSensorCfg_t gcfg;

	acfg = AccelConfig(false);
	assert(imu.Init(acfg, &bus, NULL));
	assert(bus.vConfigBytes > 0);
	assert((bus.vReg[BMI270_ACC_CONF_REG] & BMI270_ACC_CONF_ODR_MASK) == 8);
	assert((bus.vReg[BMI270_FIFO_CONFIG1_REG] & BMI270_FIFO_CONFIG1_FIFO_ACC_EN) == 0);
	assert((bus.vReg[BMI270_PWR_CTRL_REG] & BMI270_PWR_CTRL_ACC_EN) != 0);

	gcfg = GyroConfig(false);
	gcfg.Freq = 10000;
	assert(imu.Init(gcfg, &bus, NULL));
	assert((bus.vReg[BMI270_GYR_CONF_REG] & BMI270_GYR_CONF_ODR_MASK) == 6);
	assert((bus.vReg[BMI270_PWR_CTRL_REG] & BMI270_PWR_CTRL_GYR_EN) != 0);

	bus.vReg[BMI270_GYR_RANGE_REG + 1] = 0xA5;
	assert(imu.GyroBmi270::Sensitivity(125) == 125);
	assert((bus.vReg[BMI270_GYR_RANGE_REG] & BMI270_GYR_RANGE_GYR_RANGE_MASK) ==
		BMI270_GYR_RANGE_GYR_RANGE_125);
	assert(bus.vReg[BMI270_GYR_RANGE_REG + 1] == 0xA5);
}

static void TestFailedInitCanRetry()
{
	MockIntrf bus;
	AgBmi270 imu;
	AccelSensorCfg_t cfg;

	cfg = AccelConfig(false);
	bus.vFailWriteReg = BMI270_INIT_DATA_REG;
	assert(!imu.Init(cfg, &bus, NULL));
	bus.vFailWriteReg = -1;
	assert(imu.Init(cfg, &bus, NULL));

	MockIntrf badStatus;
	AgBmi270 retry;
	badStatus.vBadInitStatus = true;
	assert(!retry.Init(cfg, &badStatus, NULL));
	badStatus.vBadInitStatus = false;
	assert(retry.Init(cfg, &badStatus, NULL));
}

static void TestDirectDataAndTemperature()
{
	MockIntrf bus;
	AgBmi270 imu;
	AccelSensorCfg_t acfg;
	GyroSensorCfg_t gcfg;
	TempSensorCfg_t tcfg;
	AccelSensorRawData_t accel;
	GyroSensorRawData_t gyro;
	TempSensorData_t temp;

	acfg = AccelConfig(false);
	gcfg = GyroConfig(false);
	tcfg = TempConfig();
	assert(imu.Init(acfg, &bus, NULL));
	assert(imu.Init(gcfg, &bus, NULL));
	assert(imu.Init(tcfg, &bus, NULL));

	bus.vReg[BMI270_STATUS_REG] = BMI270_STATUS_DRDY_ACC | BMI270_STATUS_DRDY_GYR;
	Put16(&bus.vReg[BMI270_ACC_X_LSB_REG], 100);
	Put16(&bus.vReg[BMI270_ACC_X_LSB_REG + 2], -200);
	Put16(&bus.vReg[BMI270_ACC_X_LSB_REG + 4], 300);
	Put16(&bus.vReg[BMI270_GYR_X_LSB_REG], -400);
	Put16(&bus.vReg[BMI270_GYR_X_LSB_REG + 2], 500);
	Put16(&bus.vReg[BMI270_GYR_X_LSB_REG + 4], -600);
	Put16(&bus.vReg[BMI270_TEMPERATURE_LSB_REG], 512);
	SetSensorTime(bus, 160);

	assert(imu.UpdateData());
	assert(imu.Read(accel));
	assert(imu.Read(gyro));
	imu.TempBmi270::Read(temp);
	assert(accel.X == 100 && accel.Y == -200 && accel.Z == 300);
	assert(gyro.X == -400 && gyro.Y == 500 && gyro.Z == -600);
	assert(accel.Timestamp == 6250);
	assert(gyro.Timestamp == 6250);
	assert(temp.Timestamp == 6250);
	assert(temp.Temperature == 2400);
	assert(accel.Temp == 2400);
	assert(gyro.Temp == 2400);
}

static void TestHeaderlessFifoOrderAndLength()
{
	MockIntrf bus;
	AgBmi270 imu;
	AccelSensorCfg_t acfg;
	GyroSensorCfg_t gcfg;
	AccelSensorRawData_t accel;
	GyroSensorRawData_t gyro;

	acfg = AccelConfig(true);
	gcfg = GyroConfig(true);
	assert(imu.Init(acfg, &bus, NULL));
	assert(imu.Init(gcfg, &bus, NULL));
	assert((bus.vReg[BMI270_FIFO_CONFIG1_REG] &
		(BMI270_FIFO_CONFIG1_FIFO_ACC_EN | BMI270_FIFO_CONFIG1_FIFO_GYR_EN)) ==
		(BMI270_FIFO_CONFIG1_FIFO_ACC_EN | BMI270_FIFO_CONFIG1_FIFO_GYR_EN));

	Put16(&bus.vFifo[0], 10);
	Put16(&bus.vFifo[2], 20);
	Put16(&bus.vFifo[4], 30);
	Put16(&bus.vFifo[6], 40);
	Put16(&bus.vFifo[8], 50);
	Put16(&bus.vFifo[10], 60);
	bus.vFifoLen = 12;
	bus.vReg[BMI270_FIFO_WATERMARK_REG] = 0;
	SetSensorTime(bus, 320);

	assert(imu.UpdateData());
	assert(imu.Read(accel));
	assert(imu.Read(gyro));
	assert(gyro.X == 10 && gyro.Y == 20 && gyro.Z == 30);
	assert(accel.X == 40 && accel.Y == 50 && accel.Z == 60);
	assert(gyro.Timestamp == 12500);
	assert(accel.Timestamp == 12500);
	assert(bus.vFifoLen == 0);
}

static void TestTemperatureDoesNotForceGyro()
{
	MockIntrf bus;
	AgBmi270 imu;
	TempSensorCfg_t cfg;

	cfg = TempConfig();
	assert(imu.Init(cfg, &bus, NULL));
	assert((bus.vReg[BMI270_PWR_CTRL_REG] & BMI270_PWR_CTRL_TEMP_EN) != 0);
	assert((bus.vReg[BMI270_PWR_CTRL_REG] & BMI270_PWR_CTRL_GYR_EN) == 0);
}

static void TestSpiDummyAndChipSelect()
{
	MockIntrf bus(DEVINTRF_TYPE_SPI);
	AgBmi270 imu;
	AccelSensorCfg_t cfg;

	cfg = AccelConfig(false);
	cfg.DevAddr = 2;
	assert(imu.Init(cfg, &bus, NULL));
	assert(bus.vLastDevAddr == 2);
	assert(bus.vDummyReads > 0);
	assert((bus.vReg[BMI270_NV_CONF_REG] & BMI270_NV_CONF_SPI_EN) != 0);
}

static void TestReadFailure()
{
	MockIntrf bus;
	AgBmi270 imu;
	AccelSensorCfg_t cfg;

	cfg = AccelConfig(false);
	assert(imu.Init(cfg, &bus, NULL));
	bus.vReg[BMI270_STATUS_REG] = BMI270_STATUS_DRDY_ACC;
	SetSensorTime(bus, 1);
	bus.vFailReadReg = BMI270_ACC_X_LSB_REG;
	assert(!imu.UpdateData());
}

int main()
{
	static_assert(BMI270_SENSOR_TIME_REG == 0x18, "sensor-time register");
	static_assert(BMI270_FIFO_LENGTH_LSB_REG == 0x24, "FIFO length register");
	static_assert(BMI270_FIFO_DATA_REG == 0x26, "FIFO data register");
	static_assert(BMI270_FIFO_CONFIG1_FIFO_HEADER_EN == (1 << 4), "FIFO header bit");
	TestInitAndConfiguration();
	TestFailedInitCanRetry();
	TestDirectDataAndTemperature();
	TestTemperatureDoesNotForceGyro();
	TestHeaderlessFifoOrderAndLength();
	TestSpiDummyAndChipSelect();
	TestReadFailure();
	puts("BMI270 production-driver tests: PASS");
	return 0;
}
