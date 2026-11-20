#!/usr/bin/env python3
"""Exercise production Nordic TWI/TWIM/TWIS enable/disable functions.

The host layout assigns different ENABLE offsets to each peripheral mode.
This checks that the implementation addresses the right register view.
It is not an Arm MDK register-layout or silicon test.
"""
import os
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]

MODEL = r"""
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define TWIM_PRESENT 1
#define TWI_PRESENT 1
#define I2CMODE_SLAVE 2
#define I2CMODE_MASTER 1
#define TWIM_ENABLE_ENABLE_Enabled 6
#define TWIM_ENABLE_ENABLE_Disabled 0
#define TWIM_ENABLE_ENABLE_Pos 0
#define TWIS_ENABLE_ENABLE_Enabled 5
#define TWIS_ENABLE_ENABLE_Disabled 0
#define TWIS_ENABLE_ENABLE_Pos 0
#define TWI_ENABLE_ENABLE_Enabled 4
#define TWI_ENABLE_ENABLE_Disabled 0
#define TWI_ENABLE_ENABLE_Pos 0

typedef struct { uint32_t ENABLE; uint32_t Reserved[2]; } NRF_TWIM_Type;
typedef struct { uint32_t Reserved; uint32_t ENABLE; uint32_t Reserved2; } NRF_TWIS_Type;
typedef struct { uint32_t Reserved[2]; uint32_t ENABLE; } NRF_TWI_Type;
typedef struct {
	bool bDma;
	void *pDevData;
} DevIntrf_t;
typedef struct {
	int Mode;
} I2CCfg_t;
typedef struct {
	I2CCfg_t Cfg;
	DevIntrf_t DevIntrf;
} I2CDev_t;
typedef struct {
	I2CDev_t *pI2cDev;
	union {
		NRF_TWI_Type *pReg;
		NRF_TWIM_Type *pDmaReg;
		NRF_TWIS_Type *pDmaSReg;
	};
} nRFTwiDev_t;
"""

FIXTURE = r"""
int main(void)
{
	uint32_t registers[3];
	I2CDev_t device = {};
	nRFTwiDev_t driver = {};

	device.DevIntrf.pDevData = &driver;
	driver.pI2cDev = &device;
	driver.pDmaReg = (NRF_TWIM_Type *)registers;

	memset(registers, 0, sizeof(registers));
	device.DevIntrf.bDma = true;
	device.Cfg.Mode = I2CMODE_MASTER;
	nRFxI2CEnable(&device.DevIntrf);
	assert(registers[0] == TWIM_ENABLE_ENABLE_Enabled);
	assert(registers[1] == 0 && registers[2] == 0);
	nRFxI2CDisable(&device.DevIntrf);
	assert(registers[0] == 0);

	memset(registers, 0, sizeof(registers));
	device.DevIntrf.bDma = true;
	device.Cfg.Mode = I2CMODE_SLAVE;
	nRFxI2CEnable(&device.DevIntrf);
	assert(registers[1] == TWIS_ENABLE_ENABLE_Enabled);
	assert(registers[0] == 0 && registers[2] == 0);
	nRFxI2CDisable(&device.DevIntrf);
	assert(registers[1] == 0);

	memset(registers, 0, sizeof(registers));
	device.DevIntrf.bDma = false;
	device.Cfg.Mode = I2CMODE_MASTER;
	nRFxI2CEnable(&device.DevIntrf);
	assert(registers[2] == TWI_ENABLE_ENABLE_Enabled);
	assert(registers[0] == 0 && registers[1] == 0);
	nRFxI2CDisable(&device.DevIntrf);
	assert(registers[2] == 0);
	return 0;
}
"""


def function(source, name, next_name):
	begin = source.index(name)
	end = source.index(next_name, begin)
	return source[begin:end]


def main():
	source = (ROOT / "ARM/Nordic/src/i2c_nrfx.cpp").read_text()
	disable = function(source, "void nRFxI2CDisable(", "void nRFxI2CEnable(")
	enable = function(source, "void nRFxI2CEnable(", "void nRFxI2CPowerOff(")
	with tempfile.TemporaryDirectory(prefix="nordic-i2c-lifecycle-") as directory:
		test = Path(directory) / "lifecycle.cpp"
		executable = Path(directory) / "lifecycle"
		test.write_text(MODEL + disable + enable + FIXTURE)
		cmd = [os.environ.get("CXX", "g++"), "-std=c++17", "-Wall",
			"-Wextra", "-Werror", "-O2", str(test), "-o", str(executable)]
		subprocess.run(cmd, check=True)
		subprocess.run([str(executable)], check=True)
	print("Nordic TWI/TWIM/TWIS enable/disable host model: PASS")


if __name__ == "__main__":
	main()
