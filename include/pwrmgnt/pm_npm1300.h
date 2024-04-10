/**-------------------------------------------------------------------------
@file	pm_npm1300.h

@brief	Power management implementation of the Nordic nPM1300

This file contains generic definitions to implement power management drivers
such as a PMIC chip or MCU builtin power management

@author	Hoang Nguyen Hoan
@date	Apr. 10, 2024

@license

MIT License

Copyright (c) 2024 I-SYST inc. All rights reserved.

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
#indef __PM_NPM1300_H__
#define __PM_NPM1300_H__

#define NPM1300_VBUSIN_TASKUPDATE_ILIMSW_REG		0x200	// Select input current limit
#define NPM1300_VBUSIN_TASKUPDATE_ILIMSW_SELVBUSILIM0					(1<<0)	// Set to use vbusinilim0. Vbus removal results in switch 
																	// back to vbusinIlimStartup

#define NPM1300_VBUSIN_VBUSIN_ILIM0_REG				0x201	// Select Input Current limit for VBUS NOTE: Reset value from OTP, 
															// value listed in this table may not be correct.
#define NPM1300_VBUSIN_VBUSIN_ILIM0_500MA0								(1<<0)	// 500 mA
#define NPM1300_VBUSIN_VBUSIN_ILIM0_100MA								(1<<1)	// 100 mA
#define NPM1300_VBUSIN_VBUSIN_ILIM0_500MA								(1<<5)	// 500 mA
#define NPM1300_VBUSIN_VBUSIN_ILIM0_600MA								(1<<6)	// 600 mA
#define NPM1300_VBUSIN_VBUSIN_ILIM0_700MA								(1<<7)	// 700 mA
#define NPM1300_VBUSIN_VBUSIN_ILIM0_800MA								(1<<8)	// 800 mA
#define NPM1300_VBUSIN_VBUSIN_ILIM0_900MA								(1<<9)	// 900 mA
#define NPM1300_VBUSIN_VBUSIN_ILIM0_1000MA								(1<<10)	// 1000 mA
#define NPM1300_VBUSIN_VBUSIN_ILIM0_1100MA								(1<<11)	// 1100 mA
#define NPM1300_VBUSIN_VBUSIN_ILIM0_1200MA								(1<<12)	// 1200 mA
#define NPM1300_VBUSIN_VBUSIN_ILIM0_1300MA								(1<<13)	// 1300 mA
#define NPM1300_VBUSIN_VBUSIN_ILIM0_1400MA								(1<<14)	// 1400 mA
#define NPM1300_VBUSIN_VBUSIN_ILIM0_1500MA								(1<<15)	// 1500 mA

#define NPM1300_VBUSIN_VBUS_SUSPEND_REG				0x203	// Suspend mode enable
#define NPM1300_VBUSIN_VBUS_SUSPEND_SUSPENDMODE							(1<<0)	// Suspend mode

#define NPM1300_VBUSIN_USBC_DETECT_STATUS_REG		0x205	// VBUS CC comparator
#define NPM1300_VBUSIN_USBC_DETECT_STATUS_VBUSINCC1CMP_MASK				(3<<0)	
#define NPM1300_VBUSIN_USBC_DETECT_STATUS_VBUSINCC1CMP_DEFAULTUSB		(1<<0)	// Default USB 100/500mA
#define NPM1300_VBUSIN_USBC_DETECT_STATUS_VBUSINCC1CMP_1A5HIGHPOWER		(2<<0)	// 1.5A high power
#define NPM1300_VBUSIN_USBC_DETECT_STATUS_VBUSINCC1CMP_3AHIGHPOWER		(2<<0)	// 3A high power
#define NPM1300_VBUSIN_USBC_DETECT_STATUS_VBUSINCC2CMP_MASK				(3<<2)	
#define NPM1300_VBUSIN_USBC_DETECT_STATUS_VBUSINCC2CMP_DEFAULTUSB		(1<<2)	// Default USB 100/500mA
#define NPM1300_VBUSIN_USBC_DETECT_STATUS_VBUSINCC2CMP_1A5HIGHPOWER		(2<<2)	// 1.5A high power
#define NPM1300_VBUSIN_USBC_DETECT_STATUS_VBUSINCC2CMP_3AHIGHPOWER		(2<<2)	// 3A high power

#define NPM1300_VBUSIN_VBUSIN_STATUS_REG			0x207	// VBUS status flags
#define NPM1300_VBUSIN_VBUSIN_STATUS_VBUSINPRESENT_DETECTED				(1<<0)	// VBUS Detected
#define NPM1300_VBUSIN_VBUSIN_STATUS_VBUSINCURRLIMACTIVE_DETECTED		(1<<1)	// VBUS current limit Detected
#define NPM1300_VBUSIN_VBUSIN_STATUS_VBUSINOVRPROTACTIVE_ACTIVE			(1<<2)	// VBUS overvoltage protection active
#define NPM1300_VBUSIN_VBUSIN_STATUS_VBUSINUNDERVOLTAGE_DETECTED		(1<<3)	// VBUS undervoltage detected
#define NPM1300_VBUSIN_VBUSIN_STATUS_VBUSINSUSPENDMODEACTIVE_SUSPEND	(1<<4)	// VBUS suspended
#define NPM1300_VBUSIN_VBUSIN_STATUS_VBUSINVBUSOUTACTIVE_ACTIVE			(1<<5)	// VBUS out active

#define NPM1300_BCHARGER_TASK_RELEASE_ERR_REG		0x300	// Release charger from error
#define NPM1300_BCHARGER_TASK_CLEAR_CHGERR_REG		0x301	// Clear error register
#define NPM1300_BCHARGER_TASK_CLEAR_SAFTY_TIMER_REG	0x302	// Clear safty timers
#define NPM1300_BCHARGER_BCHGENABLE_SET_REG			0x304	// Charger enable set
#define NPM1300_BCHARGER_BCHGENABLE_CLR_REG			0x305	// Charger enable clear
#define NPM1300_BCHARGER_BCHGDISABLE_SET_REG		0x306	// Charger disable set
#define NPM1300_BCHARGER_BCHGDISABLE_CLR_REG		0x307	// Charger disable clear
#define NPM1300_BCHARGER_BCHGISET_MSB_REG			0x308	// Battery charger current config msb
#define NPM1300_BCHARGER_BCHGISET_LSB_REG			0x309	// Battery charger current config lsb
#define NPM1300_BCHARGER_BCHGISET_DISCHARGE_MSB_REG	0x30A	// Battery charger discharge config msb
#define NPM1300_BCHARGER_BCHGISET_DISCHARGE_LSB_REG	0x30B	// Battery charger discharge config lsb
#define NPM1300_BCHARGER_BCHGVTERM_REG				0x30C	// Battery charger termination voltage normal temp	
#define NPM1300_BCHARGER_BCHGVTERMR_REG				0x30D	// Battery charger termination voltage warm temp
#define NPM1300_BCHARGER_BCHGV_TRICKLE_SEL_REG		0x30E	// Battery charger trickle level select
#define NPM1300_BCHARGER_BCHGITERM_SEL_REG			0x30F	// Battery charger ITERM level select
#define NPM1300_BCHARGER_NTCCOLD_REG				0x310	// NTC thermistor threshold for cold temp region
#define NPM1300_BCHARGER_NTCCOLD_LSB_REG			0x311	// NTC thermistor threshold for cold temp region
#define NPM1300_BCHARGER_NTCCOOL_REG				0x312	// NTC thermistor threshold for cool temp region
#define NPM1300_BCHARGER_NTCCOOL_LSB_REG			0x313	// NTC thermistor threshold for cool temp region
#define NPM1300_BCHARGER_NTCWARM_REG				0x314	// NTC thermistor threshold for warm temp region
#define NPM1300_BCHARGER_NTCWARM_LSB_REG			0x315	// NTC thermistor threshold for warm temp region
#define NPM1300_BCHARGER_NTCHOT_REG					0x316	// NTC thermistor threshold for hot temp region
#define NPM1300_BCHARGER_NTCHOT_LSB_REG				0x317	// NTC thermistor threshold for hot temp region
#define NPM1300_BCHARGER_DIE_TEMP_STOP_REG			0x318	// Die temperature threshold for stop charging
#define NPM1300_BCHARGER_DIE_TEMP_STOP_LSB_REG		0x319	// Die temperature threshold for stop charging
#define NPM1300_BCHARGER_DIE_TEMP_RESUME_REG		0x31A	// Die temperature threshold for resume charging
#define NPM1300_BCHARGER_DIE_TEMP_RESUME_LSB_REG	0x31B	// Die temperature threshold for resume charging
#define NPM1300_BCHARGER_BCHGILIM_STATUS_REG		0x32D	// BCharger ilim status
#define NPM1300_BCHARGER_NTC_STATUS_REG				0x332	// NTC comparator status
#define NPM1300_BCHARGER_DIE_TEMP_STATUS			0x333	// Die temp comparator status
#define NPM1300_BCHARGER_BCHG_CHARGE_STATUS_REG		0x334	// Charging status
#define NPM1300_BCHARGER_BCHG_ERR_REASON_REG		0x336	// Charger FSM error : Latched error reasons
#define NPM1300_BCHARGER_BCHG_ERR_SENSOR_REG		0x337	// Charger FSM error : Latched sensor values
#define NPM1300_BCHARGER_BCHG_CONFIG_REG			0x33C	// Charger configuration

#define NMP1300_BUCK_BUCK1ENA_SET_REG				0x400	// Buck1 enable pulse
#define NMP1300_BUCK_BUCK1ENA_CLR_REG				0x401	// Buck1 disable pulse 
#define NMP1300_BUCK_BUCK2ENA_SET_REG				0x402	// Buck2 enable pulse
#define NMP1300_BUCK_BUCK2ENA_CLR_REG				0x403	// Buck2 disable pulse
#define NMP1300_BUCK_BUCK1PWM_SET_REG				0x404	// Buck1 PWM mode enable pulse
#define NMP1300_BUCK_BUCK1PWM_CLR_REG				0x405	// Buck1 PWM mode disable pulse
#define NMP1300_BUCK_BUCK2PWM_SET_REG				0x406	// Buck2 PWM mode enable pulse
#define NMP1300_BUCK_BUCK2PWM_CLR_REG				0x407	// Buck2 PWM mode diable pulse
#define NMP1300_BUCK_BUCK1NORM_VOUT_REG				0x408	// Buck1 output voltage normal mode
#define NMP1300_BUCK_BUCK1RET_VOUT_REG				0x409	// Buck1 output voltage retention mode
#define NMP1300_BUCK_BUCK2NORM_VOUT_REG				0x40A	// Buck2 output voltage normal mode
#define NMP1300_BUCK_BUCK2RET_VOUT_REG				0x40B	// Buck2 output voltage retention mode
#define NMP1300_BUCK_BUCKENCTRL_REG					0x40C	// Buck enable GPIO select
#define NMP1300_BUCK_BUCKVRETCTRL_REG				0x40D	// Buck retention voltage select
#define NMP1300_BUCK_BUCKPWMCTRLSEL_REG				0x40E	// Buck forced PWM mode GPIO select
#define NMP1300_BUCK_BUCKSWCTRLSEL_REG				0x40F	// Buck software control select
#define NMP1300_BUCK_BUCK1VOUT_STATUS_REG			0x410	// Buck1 VOUT status register
#define NMP1300_BUCK_BUCK2VOUT_STATUS_REG			0x411	// Buck2 VOUT status register
#define NMP1300_BUCK_BUCKCTRL0_REG					0x415	// Buck auto PFM to PWM control select
#define NMP1300_BUCK_BUCKSTATUS_REG 				0x416	// Buck status register

#define NMP1300_LDSW_TASKLDSW1_SET_REG				0x800	// Enable LDSW1
#define NMP1300_LDSW_TASKLDSW1_CLR_REG				0x801	// Disnable LDSW1
#define NMP1300_LDSW_TASKLDSW2_SET_REG				0x802	// Enable LDSW2
#define NMP1300_LDSW_TASKLDSW2_CLR_REG				0x803	// Disnable LDSW2
#define NMP1300_LDSW_LSDWSTATUS_REG					0x804	// Load switch status
#define NMP1300_LDSW_LDSW1GPISEL_REG				0x805	// Load switch1 GPIO control select
#define NMP1300_LDSW_LDSW2GPISEL_REG				0x806	// Load switch2 GPIO control select
#define NMP1300_LDSW_LDSWCONFIG_REG					0x807	// Load switch configuration
#define NMP1300_LDSW_LDSW1LDOSEL_REG				0x808	// Load swtich1 / LDO select
#define NMP1300_LDSW_LDSW2LDOSEL_REG				0x809	// Load swtich2 / LDO select
#define NMP1300_LDSW_LDSW1VOUTSEL_REG				0x80C	// LDO1 programmable output voltage
#define NMP1300_LDSW_LDSW2VOUTSEL_REG				0x80D	// LDO2 programmable output voltage

#define NMP1300_LEDDRV_LEDDRV0_MODESEL_REG			0xA00	// Select for LED 0 mode
#define NMP1300_LEDDRV_LEDDRV1_MODESEL_REG			0xA01	// Select for LED 1 mode
#define NMP1300_LEDDRV_LEDDRV2_MODESEL_REG			0xA02	// Select for LED 2 mode
#define NMP1300_LEDDRV_LEDDRV0_SET_REG				0xA03	// Set LED 0 On
#define NMP1300_LEDDRV_LEDDRV0_CLR_REG				0xA04	// Set LED 0 Off
#define NMP1300_LEDDRV_LEDDRV1_SET_REG				0xA05	// Set LED 1 On
#define NMP1300_LEDDRV_LEDDRV1_CLR_REG				0xA06	// Set LED 1 Off
#define NMP1300_LEDDRV_LEDDRV2_SET_REG				0xA07	// Set LED 2 On
#define NMP1300_LEDDRV_LEDDRV2_CLR_REG				0xA08	// Set LED 2 Off

#define NMP1300_GPIOS_GPIOMODE0_REG					0x600	// GPIO mode config
#define NMP1300_GPIOS_GPIOMODE1_REG					0x601	// GPIO mode config
#define NMP1300_GPIOS_GPIOMODE2_REG					0x602	// GPIO mode config
#define NMP1300_GPIOS_GPIOMODE3_REG					0x603	// GPIO mode config
#define NMP1300_GPIOS_GPIOMODE4_REG					0x604	// GPIO mode config
#define NMP1300_GPIOS_GPIODRIVE0_REG				0x605	// GPIO drive strength config
#define NMP1300_GPIOS_GPIODRIVE1_REG				0x606	// GPIO drive strength config
#define NMP1300_GPIOS_GPIODRIVE2_REG				0x607	// GPIO drive strength config
#define NMP1300_GPIOS_GPIODRIVE3_REG				0x608	// GPIO drive strength config
#define NMP1300_GPIOS_GPIODRIVE4_REG				0x609	// GPIO drive strength config
#define NMP1300_GPIOS_GPIOPUEN0_REG					0x60A	// GPIO pullup enable config
#define NMP1300_GPIOS_GPIOPUEN1_REG					0x60B	// GPIO pullup enable config
#define NMP1300_GPIOS_GPIOPUEN2_REG					0x60C	// GPIO pullup enable config
#define NMP1300_GPIOS_GPIOPUEN3_REG					0x60D	// GPIO pullup enable config
#define NMP1300_GPIOS_GPIOPUEN4_REG					0x60E	// GPIO pullup enable config
#define NMP1300_GPIOS_GPIOPDEN0_REG					0x60F	// GPIO pulldowm enable config
#define NMP1300_GPIOS_GPIOPDEN1_REG					0x610	// GPIO pulldown enable config
#define NMP1300_GPIOS_GPIOPDEN2_REG					0x611	// GPIO pulldown enable config
#define NMP1300_GPIOS_GPIOPDEN3_REG					0x612	// GPIO pulldown enable config
#define NMP1300_GPIOS_GPIOPDEN4_REG					0x613	// GPIO pulldown enable config
#define NMP1300_GPIOS_GPIOOPENDRAIN0_REG			0x614	// GPIO opendrain config
#define NMP1300_GPIOS_GPIOOPENDRAIN1_REG			0x615	// GPIO opendrain config
#define NMP1300_GPIOS_GPIOOPENDRAIN2_REG			0x616	// GPIO opendrain config
#define NMP1300_GPIOS_GPIOOPENDRAIN3_REG			0x617	// GPIO opendrain config
#define NMP1300_GPIOS_GPIOOPENDRAIN4_REG			0x618	// GPIO opendrain config
#define NMP1300_GPIOS_GPIODEBOUNCE0_REG				0x619	// GPIO debounce config
#define NMP1300_GPIOS_GPIODEBOUNCE1_REG				0x61A	// GPIO debounce config
#define NMP1300_GPIOS_GPIODEBOUNCE2_REG				0x61B	// GPIO debounce config
#define NMP1300_GPIOS_GPIODEBOUNCE3_REG				0x61C	// GPIO debounce config
#define NMP1300_GPIOS_GPIODEBOUNCE4_REG				0x61D	// GPIO debounce config
#define NMP1300_GPIOS_GPIOSTATUS_REG				0x61E	// GPIO status

#define NMP1300_ADC_TASKVBAT_MEASURE_REG			0x500	// Take VBAT measurement
#define NMP1300_ADC_TASKNTC_MEASURE_REG				0x501	// take NTC measurement
#define NMP1300_ADC_
#define NMP1300_ADC_
#define NMP1300_ADC_
#define NMP1300_ADC_
#define NMP1300_ADC_
#define NMP1300_ADC_
#define NMP1300_ADC_
#define NMP1300_ADC_
#define NMP1300_ADC_
#define NMP1300_ADC_
#define NMP1300_ADC_
#define NMP1300_ADC_
#define NMP1300_ADC_
#define NMP1300_ADC_



#endif	// __PM_NPM1300_H__
