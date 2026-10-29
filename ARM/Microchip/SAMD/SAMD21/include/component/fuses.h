/*
 * Component description for FUSES
 *
 * Copyright (c) 2022 Microchip Technology Inc. and its subsidiaries.
 *
 * Subject to your compliance with these terms, you may use Microchip software and any derivatives
 * exclusively with Microchip products. It is your responsibility to comply with third party license
 * terms applicable to your use of third party software (including open source software) that may
 * accompany Microchip software.
 *
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY,
 * APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND
 * FITNESS FOR A PARTICULAR PURPOSE.
 *
 * IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL OR CONSEQUENTIAL
 * LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF
 * MICROCHIP HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE FULLEST EXTENT
 * ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT
 * EXCEED THE AMOUNT OF FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 */

/* file generated from device description version 2019-11-25T06:52:33Z */
#ifndef _SAMD21_FUSES_COMPONENT_H_
#define _SAMD21_FUSES_COMPONENT_H_

/* ************************************************************************** */
/*   SOFTWARE API DEFINITION FOR FUSES                                        */
/* ************************************************************************** */

/* -------- FUSES_TEMP_LOG_WORD_0 : (FUSES Offset: 0x00) ( R/ 32) TEMP_LOG Page Word 0 -------- */
#define FUSES_TEMP_LOG_WORD_0_ROOM_TEMP_VAL_INT_Pos _UINT32_(0)                                          /* (FUSES_TEMP_LOG_WORD_0) Integer part of room temperature in oC Position */
#define FUSES_TEMP_LOG_WORD_0_ROOM_TEMP_VAL_INT_Msk (_UINT32_(0xFF) << FUSES_TEMP_LOG_WORD_0_ROOM_TEMP_VAL_INT_Pos) /* (FUSES_TEMP_LOG_WORD_0) Integer part of room temperature in oC Mask */
#define FUSES_TEMP_LOG_WORD_0_ROOM_TEMP_VAL_INT(value) (FUSES_TEMP_LOG_WORD_0_ROOM_TEMP_VAL_INT_Msk & (_UINT32_(value) << FUSES_TEMP_LOG_WORD_0_ROOM_TEMP_VAL_INT_Pos)) /* Assigment of value for ROOM_TEMP_VAL_INT in the FUSES_TEMP_LOG_WORD_0 register */
#define FUSES_TEMP_LOG_WORD_0_ROOM_TEMP_VAL_DEC_Pos _UINT32_(8)                                          /* (FUSES_TEMP_LOG_WORD_0) Decimal part of room temperature Position */
#define FUSES_TEMP_LOG_WORD_0_ROOM_TEMP_VAL_DEC_Msk (_UINT32_(0xF) << FUSES_TEMP_LOG_WORD_0_ROOM_TEMP_VAL_DEC_Pos) /* (FUSES_TEMP_LOG_WORD_0) Decimal part of room temperature Mask */
#define FUSES_TEMP_LOG_WORD_0_ROOM_TEMP_VAL_DEC(value) (FUSES_TEMP_LOG_WORD_0_ROOM_TEMP_VAL_DEC_Msk & (_UINT32_(value) << FUSES_TEMP_LOG_WORD_0_ROOM_TEMP_VAL_DEC_Pos)) /* Assigment of value for ROOM_TEMP_VAL_DEC in the FUSES_TEMP_LOG_WORD_0 register */
#define FUSES_TEMP_LOG_WORD_0_HOT_TEMP_VAL_INT_Pos _UINT32_(12)                                         /* (FUSES_TEMP_LOG_WORD_0) Integer part of hot temperature in oC Position */
#define FUSES_TEMP_LOG_WORD_0_HOT_TEMP_VAL_INT_Msk (_UINT32_(0xFF) << FUSES_TEMP_LOG_WORD_0_HOT_TEMP_VAL_INT_Pos) /* (FUSES_TEMP_LOG_WORD_0) Integer part of hot temperature in oC Mask */
#define FUSES_TEMP_LOG_WORD_0_HOT_TEMP_VAL_INT(value) (FUSES_TEMP_LOG_WORD_0_HOT_TEMP_VAL_INT_Msk & (_UINT32_(value) << FUSES_TEMP_LOG_WORD_0_HOT_TEMP_VAL_INT_Pos)) /* Assigment of value for HOT_TEMP_VAL_INT in the FUSES_TEMP_LOG_WORD_0 register */
#define FUSES_TEMP_LOG_WORD_0_HOT_TEMP_VAL_DEC_Pos _UINT32_(20)                                         /* (FUSES_TEMP_LOG_WORD_0) Decimal part of hot temperature Position */
#define FUSES_TEMP_LOG_WORD_0_HOT_TEMP_VAL_DEC_Msk (_UINT32_(0xF) << FUSES_TEMP_LOG_WORD_0_HOT_TEMP_VAL_DEC_Pos) /* (FUSES_TEMP_LOG_WORD_0) Decimal part of hot temperature Mask */
#define FUSES_TEMP_LOG_WORD_0_HOT_TEMP_VAL_DEC(value) (FUSES_TEMP_LOG_WORD_0_HOT_TEMP_VAL_DEC_Msk & (_UINT32_(value) << FUSES_TEMP_LOG_WORD_0_HOT_TEMP_VAL_DEC_Pos)) /* Assigment of value for HOT_TEMP_VAL_DEC in the FUSES_TEMP_LOG_WORD_0 register */
#define FUSES_TEMP_LOG_WORD_0_ROOM_INT1V_VAL_Pos _UINT32_(24)                                         /* (FUSES_TEMP_LOG_WORD_0) 2's complement of the internal 1V reference drift at room temperature (versus a 1.0 centered value) Position */
#define FUSES_TEMP_LOG_WORD_0_ROOM_INT1V_VAL_Msk (_UINT32_(0xFF) << FUSES_TEMP_LOG_WORD_0_ROOM_INT1V_VAL_Pos) /* (FUSES_TEMP_LOG_WORD_0) 2's complement of the internal 1V reference drift at room temperature (versus a 1.0 centered value) Mask */
#define FUSES_TEMP_LOG_WORD_0_ROOM_INT1V_VAL(value) (FUSES_TEMP_LOG_WORD_0_ROOM_INT1V_VAL_Msk & (_UINT32_(value) << FUSES_TEMP_LOG_WORD_0_ROOM_INT1V_VAL_Pos)) /* Assigment of value for ROOM_INT1V_VAL in the FUSES_TEMP_LOG_WORD_0 register */
#define FUSES_TEMP_LOG_WORD_0_Msk             _UINT32_(0xFFFFFFFF)                                 /* (FUSES_TEMP_LOG_WORD_0) Register Mask  */


/* -------- FUSES_TEMP_LOG_WORD_1 : (FUSES Offset: 0x04) ( R/ 32) TEMP_LOG Page Word 1 -------- */
#define FUSES_TEMP_LOG_WORD_1_HOT_INT1V_VAL_Pos _UINT32_(0)                                          /* (FUSES_TEMP_LOG_WORD_1) 2's complement of the internal 1V reference drift at hot temperature (versus a 1.0 centered value) Position */
#define FUSES_TEMP_LOG_WORD_1_HOT_INT1V_VAL_Msk (_UINT32_(0xFF) << FUSES_TEMP_LOG_WORD_1_HOT_INT1V_VAL_Pos) /* (FUSES_TEMP_LOG_WORD_1) 2's complement of the internal 1V reference drift at hot temperature (versus a 1.0 centered value) Mask */
#define FUSES_TEMP_LOG_WORD_1_HOT_INT1V_VAL(value) (FUSES_TEMP_LOG_WORD_1_HOT_INT1V_VAL_Msk & (_UINT32_(value) << FUSES_TEMP_LOG_WORD_1_HOT_INT1V_VAL_Pos)) /* Assigment of value for HOT_INT1V_VAL in the FUSES_TEMP_LOG_WORD_1 register */
#define FUSES_TEMP_LOG_WORD_1_ROOM_ADC_VAL_Pos _UINT32_(8)                                          /* (FUSES_TEMP_LOG_WORD_1) 12-bit ADC conversion at room temperature Position */
#define FUSES_TEMP_LOG_WORD_1_ROOM_ADC_VAL_Msk (_UINT32_(0xFFF) << FUSES_TEMP_LOG_WORD_1_ROOM_ADC_VAL_Pos) /* (FUSES_TEMP_LOG_WORD_1) 12-bit ADC conversion at room temperature Mask */
#define FUSES_TEMP_LOG_WORD_1_ROOM_ADC_VAL(value) (FUSES_TEMP_LOG_WORD_1_ROOM_ADC_VAL_Msk & (_UINT32_(value) << FUSES_TEMP_LOG_WORD_1_ROOM_ADC_VAL_Pos)) /* Assigment of value for ROOM_ADC_VAL in the FUSES_TEMP_LOG_WORD_1 register */
#define FUSES_TEMP_LOG_WORD_1_HOT_ADC_VAL_Pos _UINT32_(20)                                         /* (FUSES_TEMP_LOG_WORD_1) 12-bit ADC conversion at hot temperature Position */
#define FUSES_TEMP_LOG_WORD_1_HOT_ADC_VAL_Msk (_UINT32_(0xFFF) << FUSES_TEMP_LOG_WORD_1_HOT_ADC_VAL_Pos) /* (FUSES_TEMP_LOG_WORD_1) 12-bit ADC conversion at hot temperature Mask */
#define FUSES_TEMP_LOG_WORD_1_HOT_ADC_VAL(value) (FUSES_TEMP_LOG_WORD_1_HOT_ADC_VAL_Msk & (_UINT32_(value) << FUSES_TEMP_LOG_WORD_1_HOT_ADC_VAL_Pos)) /* Assigment of value for HOT_ADC_VAL in the FUSES_TEMP_LOG_WORD_1 register */
#define FUSES_TEMP_LOG_WORD_1_Msk             _UINT32_(0xFFFFFFFF)                                 /* (FUSES_TEMP_LOG_WORD_1) Register Mask  */


/* -------- FUSES_OTP4_WORD_0 : (FUSES Offset: 0x00) ( R/ 32) OTP4 Page Word 0 -------- */
#define FUSES_OTP4_WORD_0_ADC_LINEARITY_0_Pos _UINT32_(27)                                         /* (FUSES_OTP4_WORD_0) ADC Linearity bits 4:0 Position */
#define FUSES_OTP4_WORD_0_ADC_LINEARITY_0_Msk (_UINT32_(0x1F) << FUSES_OTP4_WORD_0_ADC_LINEARITY_0_Pos) /* (FUSES_OTP4_WORD_0) ADC Linearity bits 4:0 Mask */
#define FUSES_OTP4_WORD_0_ADC_LINEARITY_0(value) (FUSES_OTP4_WORD_0_ADC_LINEARITY_0_Msk & (_UINT32_(value) << FUSES_OTP4_WORD_0_ADC_LINEARITY_0_Pos)) /* Assigment of value for ADC_LINEARITY_0 in the FUSES_OTP4_WORD_0 register */
#define FUSES_OTP4_WORD_0_Msk                 _UINT32_(0xF8000000)                                 /* (FUSES_OTP4_WORD_0) Register Mask  */


/* -------- FUSES_OTP4_WORD_1 : (FUSES Offset: 0x04) ( R/ 32) OTP4 Page Word 1 -------- */
#define FUSES_OTP4_WORD_1_ADC_LINEARITY_1_Pos _UINT32_(0)                                          /* (FUSES_OTP4_WORD_1) ADC Linearity bits 7:5 Position */
#define FUSES_OTP4_WORD_1_ADC_LINEARITY_1_Msk (_UINT32_(0x7) << FUSES_OTP4_WORD_1_ADC_LINEARITY_1_Pos) /* (FUSES_OTP4_WORD_1) ADC Linearity bits 7:5 Mask */
#define FUSES_OTP4_WORD_1_ADC_LINEARITY_1(value) (FUSES_OTP4_WORD_1_ADC_LINEARITY_1_Msk & (_UINT32_(value) << FUSES_OTP4_WORD_1_ADC_LINEARITY_1_Pos)) /* Assigment of value for ADC_LINEARITY_1 in the FUSES_OTP4_WORD_1 register */
#define FUSES_OTP4_WORD_1_ADC_BIASCAL_Pos     _UINT32_(3)                                          /* (FUSES_OTP4_WORD_1) ADC Bias Calibration Position */
#define FUSES_OTP4_WORD_1_ADC_BIASCAL_Msk     (_UINT32_(0x7) << FUSES_OTP4_WORD_1_ADC_BIASCAL_Pos) /* (FUSES_OTP4_WORD_1) ADC Bias Calibration Mask */
#define FUSES_OTP4_WORD_1_ADC_BIASCAL(value)  (FUSES_OTP4_WORD_1_ADC_BIASCAL_Msk & (_UINT32_(value) << FUSES_OTP4_WORD_1_ADC_BIASCAL_Pos)) /* Assigment of value for ADC_BIASCAL in the FUSES_OTP4_WORD_1 register */
#define FUSES_OTP4_WORD_1_OSC32K_CAL_Pos      _UINT32_(6)                                          /* (FUSES_OTP4_WORD_1) OSC32K Calibration Position */
#define FUSES_OTP4_WORD_1_OSC32K_CAL_Msk      (_UINT32_(0x7F) << FUSES_OTP4_WORD_1_OSC32K_CAL_Pos) /* (FUSES_OTP4_WORD_1) OSC32K Calibration Mask */
#define FUSES_OTP4_WORD_1_OSC32K_CAL(value)   (FUSES_OTP4_WORD_1_OSC32K_CAL_Msk & (_UINT32_(value) << FUSES_OTP4_WORD_1_OSC32K_CAL_Pos)) /* Assigment of value for OSC32K_CAL in the FUSES_OTP4_WORD_1 register */
#define FUSES_OTP4_WORD_1_USB_TRANSN_Pos      _UINT32_(13)                                         /* (FUSES_OTP4_WORD_1) USB pad Transn calibration Position */
#define FUSES_OTP4_WORD_1_USB_TRANSN_Msk      (_UINT32_(0x1F) << FUSES_OTP4_WORD_1_USB_TRANSN_Pos) /* (FUSES_OTP4_WORD_1) USB pad Transn calibration Mask */
#define FUSES_OTP4_WORD_1_USB_TRANSN(value)   (FUSES_OTP4_WORD_1_USB_TRANSN_Msk & (_UINT32_(value) << FUSES_OTP4_WORD_1_USB_TRANSN_Pos)) /* Assigment of value for USB_TRANSN in the FUSES_OTP4_WORD_1 register */
#define FUSES_OTP4_WORD_1_USB_TRANSP_Pos      _UINT32_(18)                                         /* (FUSES_OTP4_WORD_1) USB pad Transp calibration Position */
#define FUSES_OTP4_WORD_1_USB_TRANSP_Msk      (_UINT32_(0x1F) << FUSES_OTP4_WORD_1_USB_TRANSP_Pos) /* (FUSES_OTP4_WORD_1) USB pad Transp calibration Mask */
#define FUSES_OTP4_WORD_1_USB_TRANSP(value)   (FUSES_OTP4_WORD_1_USB_TRANSP_Msk & (_UINT32_(value) << FUSES_OTP4_WORD_1_USB_TRANSP_Pos)) /* Assigment of value for USB_TRANSP in the FUSES_OTP4_WORD_1 register */
#define FUSES_OTP4_WORD_1_USB_TRIM_Pos        _UINT32_(23)                                         /* (FUSES_OTP4_WORD_1) USB pad Trim calibration Position */
#define FUSES_OTP4_WORD_1_USB_TRIM_Msk        (_UINT32_(0x7) << FUSES_OTP4_WORD_1_USB_TRIM_Pos)    /* (FUSES_OTP4_WORD_1) USB pad Trim calibration Mask */
#define FUSES_OTP4_WORD_1_USB_TRIM(value)     (FUSES_OTP4_WORD_1_USB_TRIM_Msk & (_UINT32_(value) << FUSES_OTP4_WORD_1_USB_TRIM_Pos)) /* Assigment of value for USB_TRIM in the FUSES_OTP4_WORD_1 register */
#define FUSES_OTP4_WORD_1_DFLL48M_COARSE_CAL_Pos _UINT32_(26)                                         /* (FUSES_OTP4_WORD_1) DFLL48M Coarse Calibration Position */
#define FUSES_OTP4_WORD_1_DFLL48M_COARSE_CAL_Msk (_UINT32_(0x3F) << FUSES_OTP4_WORD_1_DFLL48M_COARSE_CAL_Pos) /* (FUSES_OTP4_WORD_1) DFLL48M Coarse Calibration Mask */
#define FUSES_OTP4_WORD_1_DFLL48M_COARSE_CAL(value) (FUSES_OTP4_WORD_1_DFLL48M_COARSE_CAL_Msk & (_UINT32_(value) << FUSES_OTP4_WORD_1_DFLL48M_COARSE_CAL_Pos)) /* Assigment of value for DFLL48M_COARSE_CAL in the FUSES_OTP4_WORD_1 register */
#define FUSES_OTP4_WORD_1_Msk                 _UINT32_(0xFFFFFFFF)                                 /* (FUSES_OTP4_WORD_1) Register Mask  */


/* -------- FUSES_OTP4_WORD_2 : (FUSES Offset: 0x08) ( R/ 32) OTP4 Page Word 2 -------- */
#define FUSES_OTP4_WORD_2_DFLL48M_FINE_CAL_Pos _UINT32_(0)                                          /* (FUSES_OTP4_WORD_2) DFLL48M Fine Calibration Position */
#define FUSES_OTP4_WORD_2_DFLL48M_FINE_CAL_Msk (_UINT32_(0x3FF) << FUSES_OTP4_WORD_2_DFLL48M_FINE_CAL_Pos) /* (FUSES_OTP4_WORD_2) DFLL48M Fine Calibration Mask */
#define FUSES_OTP4_WORD_2_DFLL48M_FINE_CAL(value) (FUSES_OTP4_WORD_2_DFLL48M_FINE_CAL_Msk & (_UINT32_(value) << FUSES_OTP4_WORD_2_DFLL48M_FINE_CAL_Pos)) /* Assigment of value for DFLL48M_FINE_CAL in the FUSES_OTP4_WORD_2 register */
#define FUSES_OTP4_WORD_2_Msk                 _UINT32_(0x000003FF)                                 /* (FUSES_OTP4_WORD_2) Register Mask  */


/* -------- FUSES_USER_WORD_0 : (FUSES Offset: 0x00) (R/W 32) USER Page Word 0 -------- */
#define FUSES_USER_WORD_0_NVMCTRL_BOOTPROT_Pos _UINT32_(0)                                          /* (FUSES_USER_WORD_0) Bootloader Size Position */
#define FUSES_USER_WORD_0_NVMCTRL_BOOTPROT_Msk (_UINT32_(0x7) << FUSES_USER_WORD_0_NVMCTRL_BOOTPROT_Pos) /* (FUSES_USER_WORD_0) Bootloader Size Mask */
#define FUSES_USER_WORD_0_NVMCTRL_BOOTPROT(value) (FUSES_USER_WORD_0_NVMCTRL_BOOTPROT_Msk & (_UINT32_(value) << FUSES_USER_WORD_0_NVMCTRL_BOOTPROT_Pos)) /* Assigment of value for NVMCTRL_BOOTPROT in the FUSES_USER_WORD_0 register */
#define   FUSES_USER_WORD_0_NVMCTRL_BOOTPROT_SIZE_0BYTES_Val _UINT32_(0x7)                                        /* (FUSES_USER_WORD_0) 0 Bytes  */
#define   FUSES_USER_WORD_0_NVMCTRL_BOOTPROT_SIZE_512BYTES_Val _UINT32_(0x6)                                        /* (FUSES_USER_WORD_0) 512 Bytes  */
#define   FUSES_USER_WORD_0_NVMCTRL_BOOTPROT_SIZE_1024BYTES_Val _UINT32_(0x5)                                        /* (FUSES_USER_WORD_0) 1024 Bytes  */
#define   FUSES_USER_WORD_0_NVMCTRL_BOOTPROT_SIZE_2048BYTES_Val _UINT32_(0x4)                                        /* (FUSES_USER_WORD_0) 2048 Bytes  */
#define   FUSES_USER_WORD_0_NVMCTRL_BOOTPROT_SIZE_4096BYTES_Val _UINT32_(0x3)                                        /* (FUSES_USER_WORD_0) 4096 Bytes  */
#define   FUSES_USER_WORD_0_NVMCTRL_BOOTPROT_SIZE_8192BYTES_Val _UINT32_(0x2)                                        /* (FUSES_USER_WORD_0) 8192 Bytes  */
#define   FUSES_USER_WORD_0_NVMCTRL_BOOTPROT_SIZE_16384BYTES_Val _UINT32_(0x1)                                        /* (FUSES_USER_WORD_0) 16384 Bytes  */
#define   FUSES_USER_WORD_0_NVMCTRL_BOOTPROT_SIZE_32768BYTES_Val _UINT32_(0x0)                                        /* (FUSES_USER_WORD_0) 32768 Bytes  */
#define FUSES_USER_WORD_0_NVMCTRL_BOOTPROT_SIZE_0BYTES (FUSES_USER_WORD_0_NVMCTRL_BOOTPROT_SIZE_0BYTES_Val << FUSES_USER_WORD_0_NVMCTRL_BOOTPROT_Pos) /* (FUSES_USER_WORD_0) 0 Bytes Position  */
#define FUSES_USER_WORD_0_NVMCTRL_BOOTPROT_SIZE_512BYTES (FUSES_USER_WORD_0_NVMCTRL_BOOTPROT_SIZE_512BYTES_Val << FUSES_USER_WORD_0_NVMCTRL_BOOTPROT_Pos) /* (FUSES_USER_WORD_0) 512 Bytes Position  */
#define FUSES_USER_WORD_0_NVMCTRL_BOOTPROT_SIZE_1024BYTES (FUSES_USER_WORD_0_NVMCTRL_BOOTPROT_SIZE_1024BYTES_Val << FUSES_USER_WORD_0_NVMCTRL_BOOTPROT_Pos) /* (FUSES_USER_WORD_0) 1024 Bytes Position  */
#define FUSES_USER_WORD_0_NVMCTRL_BOOTPROT_SIZE_2048BYTES (FUSES_USER_WORD_0_NVMCTRL_BOOTPROT_SIZE_2048BYTES_Val << FUSES_USER_WORD_0_NVMCTRL_BOOTPROT_Pos) /* (FUSES_USER_WORD_0) 2048 Bytes Position  */
#define FUSES_USER_WORD_0_NVMCTRL_BOOTPROT_SIZE_4096BYTES (FUSES_USER_WORD_0_NVMCTRL_BOOTPROT_SIZE_4096BYTES_Val << FUSES_USER_WORD_0_NVMCTRL_BOOTPROT_Pos) /* (FUSES_USER_WORD_0) 4096 Bytes Position  */
#define FUSES_USER_WORD_0_NVMCTRL_BOOTPROT_SIZE_8192BYTES (FUSES_USER_WORD_0_NVMCTRL_BOOTPROT_SIZE_8192BYTES_Val << FUSES_USER_WORD_0_NVMCTRL_BOOTPROT_Pos) /* (FUSES_USER_WORD_0) 8192 Bytes Position  */
#define FUSES_USER_WORD_0_NVMCTRL_BOOTPROT_SIZE_16384BYTES (FUSES_USER_WORD_0_NVMCTRL_BOOTPROT_SIZE_16384BYTES_Val << FUSES_USER_WORD_0_NVMCTRL_BOOTPROT_Pos) /* (FUSES_USER_WORD_0) 16384 Bytes Position  */
#define FUSES_USER_WORD_0_NVMCTRL_BOOTPROT_SIZE_32768BYTES (FUSES_USER_WORD_0_NVMCTRL_BOOTPROT_SIZE_32768BYTES_Val << FUSES_USER_WORD_0_NVMCTRL_BOOTPROT_Pos) /* (FUSES_USER_WORD_0) 32768 Bytes Position  */
#define FUSES_USER_WORD_0_NVMCTRL_EEPROM_SIZE_Pos _UINT32_(4)                                          /* (FUSES_USER_WORD_0) EEPROM Size Position */
#define FUSES_USER_WORD_0_NVMCTRL_EEPROM_SIZE_Msk (_UINT32_(0x7) << FUSES_USER_WORD_0_NVMCTRL_EEPROM_SIZE_Pos) /* (FUSES_USER_WORD_0) EEPROM Size Mask */
#define FUSES_USER_WORD_0_NVMCTRL_EEPROM_SIZE(value) (FUSES_USER_WORD_0_NVMCTRL_EEPROM_SIZE_Msk & (_UINT32_(value) << FUSES_USER_WORD_0_NVMCTRL_EEPROM_SIZE_Pos)) /* Assigment of value for NVMCTRL_EEPROM_SIZE in the FUSES_USER_WORD_0 register */
#define   FUSES_USER_WORD_0_NVMCTRL_EEPROM_SIZE_SIZE_0BYTES_Val _UINT32_(0x7)                                        /* (FUSES_USER_WORD_0) 0 Bytes  */
#define   FUSES_USER_WORD_0_NVMCTRL_EEPROM_SIZE_SIZE_256BYTES_Val _UINT32_(0x6)                                        /* (FUSES_USER_WORD_0) 256 Bytes  */
#define   FUSES_USER_WORD_0_NVMCTRL_EEPROM_SIZE_SIZE_512BYTES_Val _UINT32_(0x5)                                        /* (FUSES_USER_WORD_0) 512 Bytes  */
#define   FUSES_USER_WORD_0_NVMCTRL_EEPROM_SIZE_SIZE_1024BYTES_Val _UINT32_(0x4)                                        /* (FUSES_USER_WORD_0) 1024 Bytes  */
#define   FUSES_USER_WORD_0_NVMCTRL_EEPROM_SIZE_SIZE_2048BYTES_Val _UINT32_(0x3)                                        /* (FUSES_USER_WORD_0) 2048 Bytes  */
#define   FUSES_USER_WORD_0_NVMCTRL_EEPROM_SIZE_SIZE_4096BYTES_Val _UINT32_(0x2)                                        /* (FUSES_USER_WORD_0) 4096 Bytes  */
#define   FUSES_USER_WORD_0_NVMCTRL_EEPROM_SIZE_SIZE_8192BYTES_Val _UINT32_(0x1)                                        /* (FUSES_USER_WORD_0) 8192 Bytes  */
#define   FUSES_USER_WORD_0_NVMCTRL_EEPROM_SIZE_SIZE_16384BYTES_Val _UINT32_(0x0)                                        /* (FUSES_USER_WORD_0) 16384 Bytes  */
#define FUSES_USER_WORD_0_NVMCTRL_EEPROM_SIZE_SIZE_0BYTES (FUSES_USER_WORD_0_NVMCTRL_EEPROM_SIZE_SIZE_0BYTES_Val << FUSES_USER_WORD_0_NVMCTRL_EEPROM_SIZE_Pos) /* (FUSES_USER_WORD_0) 0 Bytes Position  */
#define FUSES_USER_WORD_0_NVMCTRL_EEPROM_SIZE_SIZE_256BYTES (FUSES_USER_WORD_0_NVMCTRL_EEPROM_SIZE_SIZE_256BYTES_Val << FUSES_USER_WORD_0_NVMCTRL_EEPROM_SIZE_Pos) /* (FUSES_USER_WORD_0) 256 Bytes Position  */
#define FUSES_USER_WORD_0_NVMCTRL_EEPROM_SIZE_SIZE_512BYTES (FUSES_USER_WORD_0_NVMCTRL_EEPROM_SIZE_SIZE_512BYTES_Val << FUSES_USER_WORD_0_NVMCTRL_EEPROM_SIZE_Pos) /* (FUSES_USER_WORD_0) 512 Bytes Position  */
#define FUSES_USER_WORD_0_NVMCTRL_EEPROM_SIZE_SIZE_1024BYTES (FUSES_USER_WORD_0_NVMCTRL_EEPROM_SIZE_SIZE_1024BYTES_Val << FUSES_USER_WORD_0_NVMCTRL_EEPROM_SIZE_Pos) /* (FUSES_USER_WORD_0) 1024 Bytes Position  */
#define FUSES_USER_WORD_0_NVMCTRL_EEPROM_SIZE_SIZE_2048BYTES (FUSES_USER_WORD_0_NVMCTRL_EEPROM_SIZE_SIZE_2048BYTES_Val << FUSES_USER_WORD_0_NVMCTRL_EEPROM_SIZE_Pos) /* (FUSES_USER_WORD_0) 2048 Bytes Position  */
#define FUSES_USER_WORD_0_NVMCTRL_EEPROM_SIZE_SIZE_4096BYTES (FUSES_USER_WORD_0_NVMCTRL_EEPROM_SIZE_SIZE_4096BYTES_Val << FUSES_USER_WORD_0_NVMCTRL_EEPROM_SIZE_Pos) /* (FUSES_USER_WORD_0) 4096 Bytes Position  */
#define FUSES_USER_WORD_0_NVMCTRL_EEPROM_SIZE_SIZE_8192BYTES (FUSES_USER_WORD_0_NVMCTRL_EEPROM_SIZE_SIZE_8192BYTES_Val << FUSES_USER_WORD_0_NVMCTRL_EEPROM_SIZE_Pos) /* (FUSES_USER_WORD_0) 8192 Bytes Position  */
#define FUSES_USER_WORD_0_NVMCTRL_EEPROM_SIZE_SIZE_16384BYTES (FUSES_USER_WORD_0_NVMCTRL_EEPROM_SIZE_SIZE_16384BYTES_Val << FUSES_USER_WORD_0_NVMCTRL_EEPROM_SIZE_Pos) /* (FUSES_USER_WORD_0) 16384 Bytes Position  */
#define FUSES_USER_WORD_0_BOD33USERLEVEL_Pos  _UINT32_(8)                                          /* (FUSES_USER_WORD_0) BOD33 User Level Position */
#define FUSES_USER_WORD_0_BOD33USERLEVEL_Msk  (_UINT32_(0x3F) << FUSES_USER_WORD_0_BOD33USERLEVEL_Pos) /* (FUSES_USER_WORD_0) BOD33 User Level Mask */
#define FUSES_USER_WORD_0_BOD33USERLEVEL(value) (FUSES_USER_WORD_0_BOD33USERLEVEL_Msk & (_UINT32_(value) << FUSES_USER_WORD_0_BOD33USERLEVEL_Pos)) /* Assigment of value for BOD33USERLEVEL in the FUSES_USER_WORD_0 register */
#define FUSES_USER_WORD_0_BOD33_EN_Pos        _UINT32_(14)                                         /* (FUSES_USER_WORD_0) BOD33 Enable Position */
#define FUSES_USER_WORD_0_BOD33_EN_Msk        (_UINT32_(0x1) << FUSES_USER_WORD_0_BOD33_EN_Pos)    /* (FUSES_USER_WORD_0) BOD33 Enable Mask */
#define FUSES_USER_WORD_0_BOD33_EN(value)     (FUSES_USER_WORD_0_BOD33_EN_Msk & (_UINT32_(value) << FUSES_USER_WORD_0_BOD33_EN_Pos)) /* Assigment of value for BOD33_EN in the FUSES_USER_WORD_0 register */
#define   FUSES_USER_WORD_0_BOD33_EN_DISABLED_Val _UINT32_(0x0)                                        /* (FUSES_USER_WORD_0) BOD33 is disabled  */
#define   FUSES_USER_WORD_0_BOD33_EN_ENABLED_Val _UINT32_(0x1)                                        /* (FUSES_USER_WORD_0) BOD33 is enabled  */
#define FUSES_USER_WORD_0_BOD33_EN_DISABLED   (FUSES_USER_WORD_0_BOD33_EN_DISABLED_Val << FUSES_USER_WORD_0_BOD33_EN_Pos) /* (FUSES_USER_WORD_0) BOD33 is disabled Position  */
#define FUSES_USER_WORD_0_BOD33_EN_ENABLED    (FUSES_USER_WORD_0_BOD33_EN_ENABLED_Val << FUSES_USER_WORD_0_BOD33_EN_Pos) /* (FUSES_USER_WORD_0) BOD33 is enabled Position  */
#define FUSES_USER_WORD_0_BOD33_ACTION_Pos    _UINT32_(15)                                         /* (FUSES_USER_WORD_0) BOD33 Action Position */
#define FUSES_USER_WORD_0_BOD33_ACTION_Msk    (_UINT32_(0x3) << FUSES_USER_WORD_0_BOD33_ACTION_Pos) /* (FUSES_USER_WORD_0) BOD33 Action Mask */
#define FUSES_USER_WORD_0_BOD33_ACTION(value) (FUSES_USER_WORD_0_BOD33_ACTION_Msk & (_UINT32_(value) << FUSES_USER_WORD_0_BOD33_ACTION_Pos)) /* Assigment of value for BOD33_ACTION in the FUSES_USER_WORD_0 register */
#define   FUSES_USER_WORD_0_BOD33_ACTION_NONE_Val _UINT32_(0x0)                                        /* (FUSES_USER_WORD_0) No action  */
#define   FUSES_USER_WORD_0_BOD33_ACTION_RESET_Val _UINT32_(0x1)                                        /* (FUSES_USER_WORD_0) The BOD33 generates a reset  */
#define   FUSES_USER_WORD_0_BOD33_ACTION_INTERRUPT_Val _UINT32_(0x2)                                        /* (FUSES_USER_WORD_0) The BOD33 generates an interrupt  */
#define FUSES_USER_WORD_0_BOD33_ACTION_NONE   (FUSES_USER_WORD_0_BOD33_ACTION_NONE_Val << FUSES_USER_WORD_0_BOD33_ACTION_Pos) /* (FUSES_USER_WORD_0) No action Position  */
#define FUSES_USER_WORD_0_BOD33_ACTION_RESET  (FUSES_USER_WORD_0_BOD33_ACTION_RESET_Val << FUSES_USER_WORD_0_BOD33_ACTION_Pos) /* (FUSES_USER_WORD_0) The BOD33 generates a reset Position  */
#define FUSES_USER_WORD_0_BOD33_ACTION_INTERRUPT (FUSES_USER_WORD_0_BOD33_ACTION_INTERRUPT_Val << FUSES_USER_WORD_0_BOD33_ACTION_Pos) /* (FUSES_USER_WORD_0) The BOD33 generates an interrupt Position  */
#define FUSES_USER_WORD_0_WDT_ENABLE_Pos      _UINT32_(25)                                         /* (FUSES_USER_WORD_0) WDT Enable Position */
#define FUSES_USER_WORD_0_WDT_ENABLE_Msk      (_UINT32_(0x1) << FUSES_USER_WORD_0_WDT_ENABLE_Pos)  /* (FUSES_USER_WORD_0) WDT Enable Mask */
#define FUSES_USER_WORD_0_WDT_ENABLE(value)   (FUSES_USER_WORD_0_WDT_ENABLE_Msk & (_UINT32_(value) << FUSES_USER_WORD_0_WDT_ENABLE_Pos)) /* Assigment of value for WDT_ENABLE in the FUSES_USER_WORD_0 register */
#define   FUSES_USER_WORD_0_WDT_ENABLE_DISABLED_Val _UINT32_(0x0)                                        /* (FUSES_USER_WORD_0) WDT is disabled  */
#define   FUSES_USER_WORD_0_WDT_ENABLE_ENABLED_Val _UINT32_(0x1)                                        /* (FUSES_USER_WORD_0) WDT is enabled  */
#define FUSES_USER_WORD_0_WDT_ENABLE_DISABLED (FUSES_USER_WORD_0_WDT_ENABLE_DISABLED_Val << FUSES_USER_WORD_0_WDT_ENABLE_Pos) /* (FUSES_USER_WORD_0) WDT is disabled Position  */
#define FUSES_USER_WORD_0_WDT_ENABLE_ENABLED  (FUSES_USER_WORD_0_WDT_ENABLE_ENABLED_Val << FUSES_USER_WORD_0_WDT_ENABLE_Pos) /* (FUSES_USER_WORD_0) WDT is enabled Position  */
#define FUSES_USER_WORD_0_WDT_ALWAYSON_Pos    _UINT32_(26)                                         /* (FUSES_USER_WORD_0) WDT Always On Position */
#define FUSES_USER_WORD_0_WDT_ALWAYSON_Msk    (_UINT32_(0x1) << FUSES_USER_WORD_0_WDT_ALWAYSON_Pos) /* (FUSES_USER_WORD_0) WDT Always On Mask */
#define FUSES_USER_WORD_0_WDT_ALWAYSON(value) (FUSES_USER_WORD_0_WDT_ALWAYSON_Msk & (_UINT32_(value) << FUSES_USER_WORD_0_WDT_ALWAYSON_Pos)) /* Assigment of value for WDT_ALWAYSON in the FUSES_USER_WORD_0 register */
#define   FUSES_USER_WORD_0_WDT_ALWAYSON_DISABLED_Val _UINT32_(0x0)                                        /* (FUSES_USER_WORD_0) WDT is enabled and disabled through ENABLE bit  */
#define   FUSES_USER_WORD_0_WDT_ALWAYSON_ENABLED_Val _UINT32_(0x1)                                        /* (FUSES_USER_WORD_0) WDT is enabled and can only be disabled by a power-on reset  */
#define FUSES_USER_WORD_0_WDT_ALWAYSON_DISABLED (FUSES_USER_WORD_0_WDT_ALWAYSON_DISABLED_Val << FUSES_USER_WORD_0_WDT_ALWAYSON_Pos) /* (FUSES_USER_WORD_0) WDT is enabled and disabled through ENABLE bit Position  */
#define FUSES_USER_WORD_0_WDT_ALWAYSON_ENABLED (FUSES_USER_WORD_0_WDT_ALWAYSON_ENABLED_Val << FUSES_USER_WORD_0_WDT_ALWAYSON_Pos) /* (FUSES_USER_WORD_0) WDT is enabled and can only be disabled by a power-on reset Position  */
#define FUSES_USER_WORD_0_WDT_PER_Pos         _UINT32_(27)                                         /* (FUSES_USER_WORD_0) WDT Period Position */
#define FUSES_USER_WORD_0_WDT_PER_Msk         (_UINT32_(0xF) << FUSES_USER_WORD_0_WDT_PER_Pos)     /* (FUSES_USER_WORD_0) WDT Period Mask */
#define FUSES_USER_WORD_0_WDT_PER(value)      (FUSES_USER_WORD_0_WDT_PER_Msk & (_UINT32_(value) << FUSES_USER_WORD_0_WDT_PER_Pos)) /* Assigment of value for WDT_PER in the FUSES_USER_WORD_0 register */
#define   FUSES_USER_WORD_0_WDT_PER_CYC8_Val  _UINT32_(0x0)                                        /* (FUSES_USER_WORD_0) 8 clock cycles  */
#define   FUSES_USER_WORD_0_WDT_PER_CYC16_Val _UINT32_(0x1)                                        /* (FUSES_USER_WORD_0) 16 clock cycles  */
#define   FUSES_USER_WORD_0_WDT_PER_CYC32_Val _UINT32_(0x2)                                        /* (FUSES_USER_WORD_0) 32 clock cycles  */
#define   FUSES_USER_WORD_0_WDT_PER_CYC64_Val _UINT32_(0x3)                                        /* (FUSES_USER_WORD_0) 64 clock cycles  */
#define   FUSES_USER_WORD_0_WDT_PER_CYC128_Val _UINT32_(0x4)                                        /* (FUSES_USER_WORD_0) 128 clock cycles  */
#define   FUSES_USER_WORD_0_WDT_PER_CYC256_Val _UINT32_(0x5)                                        /* (FUSES_USER_WORD_0) 256 clock cycles  */
#define   FUSES_USER_WORD_0_WDT_PER_CYC512_Val _UINT32_(0x6)                                        /* (FUSES_USER_WORD_0) 512 clock cycles  */
#define   FUSES_USER_WORD_0_WDT_PER_CYC1024_Val _UINT32_(0x7)                                        /* (FUSES_USER_WORD_0) 1024 clock cycles  */
#define   FUSES_USER_WORD_0_WDT_PER_CYC2048_Val _UINT32_(0x8)                                        /* (FUSES_USER_WORD_0) 2048 clock cycles  */
#define   FUSES_USER_WORD_0_WDT_PER_CYC4096_Val _UINT32_(0x9)                                        /* (FUSES_USER_WORD_0) 4096 clock cycles  */
#define   FUSES_USER_WORD_0_WDT_PER_CYC8192_Val _UINT32_(0xA)                                        /* (FUSES_USER_WORD_0) 8192 clock cycles  */
#define   FUSES_USER_WORD_0_WDT_PER_CYC16384_Val _UINT32_(0xB)                                        /* (FUSES_USER_WORD_0) 16384 clock cycles  */
#define FUSES_USER_WORD_0_WDT_PER_CYC8        (FUSES_USER_WORD_0_WDT_PER_CYC8_Val << FUSES_USER_WORD_0_WDT_PER_Pos) /* (FUSES_USER_WORD_0) 8 clock cycles Position  */
#define FUSES_USER_WORD_0_WDT_PER_CYC16       (FUSES_USER_WORD_0_WDT_PER_CYC16_Val << FUSES_USER_WORD_0_WDT_PER_Pos) /* (FUSES_USER_WORD_0) 16 clock cycles Position  */
#define FUSES_USER_WORD_0_WDT_PER_CYC32       (FUSES_USER_WORD_0_WDT_PER_CYC32_Val << FUSES_USER_WORD_0_WDT_PER_Pos) /* (FUSES_USER_WORD_0) 32 clock cycles Position  */
#define FUSES_USER_WORD_0_WDT_PER_CYC64       (FUSES_USER_WORD_0_WDT_PER_CYC64_Val << FUSES_USER_WORD_0_WDT_PER_Pos) /* (FUSES_USER_WORD_0) 64 clock cycles Position  */
#define FUSES_USER_WORD_0_WDT_PER_CYC128      (FUSES_USER_WORD_0_WDT_PER_CYC128_Val << FUSES_USER_WORD_0_WDT_PER_Pos) /* (FUSES_USER_WORD_0) 128 clock cycles Position  */
#define FUSES_USER_WORD_0_WDT_PER_CYC256      (FUSES_USER_WORD_0_WDT_PER_CYC256_Val << FUSES_USER_WORD_0_WDT_PER_Pos) /* (FUSES_USER_WORD_0) 256 clock cycles Position  */
#define FUSES_USER_WORD_0_WDT_PER_CYC512      (FUSES_USER_WORD_0_WDT_PER_CYC512_Val << FUSES_USER_WORD_0_WDT_PER_Pos) /* (FUSES_USER_WORD_0) 512 clock cycles Position  */
#define FUSES_USER_WORD_0_WDT_PER_CYC1024     (FUSES_USER_WORD_0_WDT_PER_CYC1024_Val << FUSES_USER_WORD_0_WDT_PER_Pos) /* (FUSES_USER_WORD_0) 1024 clock cycles Position  */
#define FUSES_USER_WORD_0_WDT_PER_CYC2048     (FUSES_USER_WORD_0_WDT_PER_CYC2048_Val << FUSES_USER_WORD_0_WDT_PER_Pos) /* (FUSES_USER_WORD_0) 2048 clock cycles Position  */
#define FUSES_USER_WORD_0_WDT_PER_CYC4096     (FUSES_USER_WORD_0_WDT_PER_CYC4096_Val << FUSES_USER_WORD_0_WDT_PER_Pos) /* (FUSES_USER_WORD_0) 4096 clock cycles Position  */
#define FUSES_USER_WORD_0_WDT_PER_CYC8192     (FUSES_USER_WORD_0_WDT_PER_CYC8192_Val << FUSES_USER_WORD_0_WDT_PER_Pos) /* (FUSES_USER_WORD_0) 8192 clock cycles Position  */
#define FUSES_USER_WORD_0_WDT_PER_CYC16384    (FUSES_USER_WORD_0_WDT_PER_CYC16384_Val << FUSES_USER_WORD_0_WDT_PER_Pos) /* (FUSES_USER_WORD_0) 16384 clock cycles Position  */
#define FUSES_USER_WORD_0_WDT_WINDOW_0_Pos    _UINT32_(31)                                         /* (FUSES_USER_WORD_0) WDT Window bit 0 Position */
#define FUSES_USER_WORD_0_WDT_WINDOW_0_Msk    (_UINT32_(0x1) << FUSES_USER_WORD_0_WDT_WINDOW_0_Pos) /* (FUSES_USER_WORD_0) WDT Window bit 0 Mask */
#define FUSES_USER_WORD_0_WDT_WINDOW_0(value) (FUSES_USER_WORD_0_WDT_WINDOW_0_Msk & (_UINT32_(value) << FUSES_USER_WORD_0_WDT_WINDOW_0_Pos)) /* Assigment of value for WDT_WINDOW_0 in the FUSES_USER_WORD_0 register */
#define FUSES_USER_WORD_0_Msk                 _UINT32_(0xFE01FF77)                                 /* (FUSES_USER_WORD_0) Register Mask  */

#define FUSES_USER_WORD_0_WDT_WINDOW__Pos     _UINT32_(31)                                         /* (FUSES_USER_WORD_0 Position) WDT Window bit x */
#define FUSES_USER_WORD_0_WDT_WINDOW__Msk     (_UINT32_(0x1) << FUSES_USER_WORD_0_WDT_WINDOW__Pos) /* (FUSES_USER_WORD_0 Mask) WDT_WINDOW_ */
#define FUSES_USER_WORD_0_WDT_WINDOW_(value)  (FUSES_USER_WORD_0_WDT_WINDOW__Msk & (_UINT32_(value) << FUSES_USER_WORD_0_WDT_WINDOW__Pos)) 

/* -------- FUSES_USER_WORD_1 : (FUSES Offset: 0x04) (R/W 32) USER Page Word 1 -------- */
#define FUSES_USER_WORD_1_WDT_WINDOW_1_Pos    _UINT32_(0)                                          /* (FUSES_USER_WORD_1) WDT Window bits 3:1 Position */
#define FUSES_USER_WORD_1_WDT_WINDOW_1_Msk    (_UINT32_(0x7) << FUSES_USER_WORD_1_WDT_WINDOW_1_Pos) /* (FUSES_USER_WORD_1) WDT Window bits 3:1 Mask */
#define FUSES_USER_WORD_1_WDT_WINDOW_1(value) (FUSES_USER_WORD_1_WDT_WINDOW_1_Msk & (_UINT32_(value) << FUSES_USER_WORD_1_WDT_WINDOW_1_Pos)) /* Assigment of value for WDT_WINDOW_1 in the FUSES_USER_WORD_1 register */
#define FUSES_USER_WORD_1_WDT_EWOFFSET_Pos    _UINT32_(3)                                          /* (FUSES_USER_WORD_1) WDT Early Warning Offset Position */
#define FUSES_USER_WORD_1_WDT_EWOFFSET_Msk    (_UINT32_(0xF) << FUSES_USER_WORD_1_WDT_EWOFFSET_Pos) /* (FUSES_USER_WORD_1) WDT Early Warning Offset Mask */
#define FUSES_USER_WORD_1_WDT_EWOFFSET(value) (FUSES_USER_WORD_1_WDT_EWOFFSET_Msk & (_UINT32_(value) << FUSES_USER_WORD_1_WDT_EWOFFSET_Pos)) /* Assigment of value for WDT_EWOFFSET in the FUSES_USER_WORD_1 register */
#define   FUSES_USER_WORD_1_WDT_EWOFFSET_CYC8_Val _UINT32_(0x0)                                        /* (FUSES_USER_WORD_1) 8 clock cycles  */
#define   FUSES_USER_WORD_1_WDT_EWOFFSET_CYC16_Val _UINT32_(0x1)                                        /* (FUSES_USER_WORD_1) 16 clock cycles  */
#define   FUSES_USER_WORD_1_WDT_EWOFFSET_CYC32_Val _UINT32_(0x2)                                        /* (FUSES_USER_WORD_1) 32 clock cycles  */
#define   FUSES_USER_WORD_1_WDT_EWOFFSET_CYC64_Val _UINT32_(0x3)                                        /* (FUSES_USER_WORD_1) 64 clock cycles  */
#define   FUSES_USER_WORD_1_WDT_EWOFFSET_CYC128_Val _UINT32_(0x4)                                        /* (FUSES_USER_WORD_1) 128 clock cycles  */
#define   FUSES_USER_WORD_1_WDT_EWOFFSET_CYC256_Val _UINT32_(0x5)                                        /* (FUSES_USER_WORD_1) 256 clock cycles  */
#define   FUSES_USER_WORD_1_WDT_EWOFFSET_CYC512_Val _UINT32_(0x6)                                        /* (FUSES_USER_WORD_1) 512 clock cycles  */
#define   FUSES_USER_WORD_1_WDT_EWOFFSET_CYC1024_Val _UINT32_(0x7)                                        /* (FUSES_USER_WORD_1) 1024 clock cycles  */
#define   FUSES_USER_WORD_1_WDT_EWOFFSET_CYC2048_Val _UINT32_(0x8)                                        /* (FUSES_USER_WORD_1) 2048 clock cycles  */
#define   FUSES_USER_WORD_1_WDT_EWOFFSET_CYC4096_Val _UINT32_(0x9)                                        /* (FUSES_USER_WORD_1) 4096 clock cycles  */
#define   FUSES_USER_WORD_1_WDT_EWOFFSET_CYC8192_Val _UINT32_(0xA)                                        /* (FUSES_USER_WORD_1) 8192 clock cycles  */
#define   FUSES_USER_WORD_1_WDT_EWOFFSET_CYC16384_Val _UINT32_(0xB)                                        /* (FUSES_USER_WORD_1) 16384 clock cycles  */
#define FUSES_USER_WORD_1_WDT_EWOFFSET_CYC8   (FUSES_USER_WORD_1_WDT_EWOFFSET_CYC8_Val << FUSES_USER_WORD_1_WDT_EWOFFSET_Pos) /* (FUSES_USER_WORD_1) 8 clock cycles Position  */
#define FUSES_USER_WORD_1_WDT_EWOFFSET_CYC16  (FUSES_USER_WORD_1_WDT_EWOFFSET_CYC16_Val << FUSES_USER_WORD_1_WDT_EWOFFSET_Pos) /* (FUSES_USER_WORD_1) 16 clock cycles Position  */
#define FUSES_USER_WORD_1_WDT_EWOFFSET_CYC32  (FUSES_USER_WORD_1_WDT_EWOFFSET_CYC32_Val << FUSES_USER_WORD_1_WDT_EWOFFSET_Pos) /* (FUSES_USER_WORD_1) 32 clock cycles Position  */
#define FUSES_USER_WORD_1_WDT_EWOFFSET_CYC64  (FUSES_USER_WORD_1_WDT_EWOFFSET_CYC64_Val << FUSES_USER_WORD_1_WDT_EWOFFSET_Pos) /* (FUSES_USER_WORD_1) 64 clock cycles Position  */
#define FUSES_USER_WORD_1_WDT_EWOFFSET_CYC128 (FUSES_USER_WORD_1_WDT_EWOFFSET_CYC128_Val << FUSES_USER_WORD_1_WDT_EWOFFSET_Pos) /* (FUSES_USER_WORD_1) 128 clock cycles Position  */
#define FUSES_USER_WORD_1_WDT_EWOFFSET_CYC256 (FUSES_USER_WORD_1_WDT_EWOFFSET_CYC256_Val << FUSES_USER_WORD_1_WDT_EWOFFSET_Pos) /* (FUSES_USER_WORD_1) 256 clock cycles Position  */
#define FUSES_USER_WORD_1_WDT_EWOFFSET_CYC512 (FUSES_USER_WORD_1_WDT_EWOFFSET_CYC512_Val << FUSES_USER_WORD_1_WDT_EWOFFSET_Pos) /* (FUSES_USER_WORD_1) 512 clock cycles Position  */
#define FUSES_USER_WORD_1_WDT_EWOFFSET_CYC1024 (FUSES_USER_WORD_1_WDT_EWOFFSET_CYC1024_Val << FUSES_USER_WORD_1_WDT_EWOFFSET_Pos) /* (FUSES_USER_WORD_1) 1024 clock cycles Position  */
#define FUSES_USER_WORD_1_WDT_EWOFFSET_CYC2048 (FUSES_USER_WORD_1_WDT_EWOFFSET_CYC2048_Val << FUSES_USER_WORD_1_WDT_EWOFFSET_Pos) /* (FUSES_USER_WORD_1) 2048 clock cycles Position  */
#define FUSES_USER_WORD_1_WDT_EWOFFSET_CYC4096 (FUSES_USER_WORD_1_WDT_EWOFFSET_CYC4096_Val << FUSES_USER_WORD_1_WDT_EWOFFSET_Pos) /* (FUSES_USER_WORD_1) 4096 clock cycles Position  */
#define FUSES_USER_WORD_1_WDT_EWOFFSET_CYC8192 (FUSES_USER_WORD_1_WDT_EWOFFSET_CYC8192_Val << FUSES_USER_WORD_1_WDT_EWOFFSET_Pos) /* (FUSES_USER_WORD_1) 8192 clock cycles Position  */
#define FUSES_USER_WORD_1_WDT_EWOFFSET_CYC16384 (FUSES_USER_WORD_1_WDT_EWOFFSET_CYC16384_Val << FUSES_USER_WORD_1_WDT_EWOFFSET_Pos) /* (FUSES_USER_WORD_1) 16384 clock cycles Position  */
#define FUSES_USER_WORD_1_WDT_WEN_Pos         _UINT32_(7)                                          /* (FUSES_USER_WORD_1) WDT Window Mode Enable Position */
#define FUSES_USER_WORD_1_WDT_WEN_Msk         (_UINT32_(0x1) << FUSES_USER_WORD_1_WDT_WEN_Pos)     /* (FUSES_USER_WORD_1) WDT Window Mode Enable Mask */
#define FUSES_USER_WORD_1_WDT_WEN(value)      (FUSES_USER_WORD_1_WDT_WEN_Msk & (_UINT32_(value) << FUSES_USER_WORD_1_WDT_WEN_Pos)) /* Assigment of value for WDT_WEN in the FUSES_USER_WORD_1 register */
#define   FUSES_USER_WORD_1_WDT_WEN_DISABLED_Val _UINT32_(0x0)                                        /* (FUSES_USER_WORD_1) WDT is disabled  */
#define   FUSES_USER_WORD_1_WDT_WEN_ENABLED_Val _UINT32_(0x1)                                        /* (FUSES_USER_WORD_1) WDT is enabled  */
#define FUSES_USER_WORD_1_WDT_WEN_DISABLED    (FUSES_USER_WORD_1_WDT_WEN_DISABLED_Val << FUSES_USER_WORD_1_WDT_WEN_Pos) /* (FUSES_USER_WORD_1) WDT is disabled Position  */
#define FUSES_USER_WORD_1_WDT_WEN_ENABLED     (FUSES_USER_WORD_1_WDT_WEN_ENABLED_Val << FUSES_USER_WORD_1_WDT_WEN_Pos) /* (FUSES_USER_WORD_1) WDT is enabled Position  */
#define FUSES_USER_WORD_1_BOD33_HYST_Pos      _UINT32_(8)                                          /* (FUSES_USER_WORD_1) BOD33 Hysteresis Position */
#define FUSES_USER_WORD_1_BOD33_HYST_Msk      (_UINT32_(0x1) << FUSES_USER_WORD_1_BOD33_HYST_Pos)  /* (FUSES_USER_WORD_1) BOD33 Hysteresis Mask */
#define FUSES_USER_WORD_1_BOD33_HYST(value)   (FUSES_USER_WORD_1_BOD33_HYST_Msk & (_UINT32_(value) << FUSES_USER_WORD_1_BOD33_HYST_Pos)) /* Assigment of value for BOD33_HYST in the FUSES_USER_WORD_1 register */
#define   FUSES_USER_WORD_1_BOD33_HYST_DISABLED_Val _UINT32_(0x0)                                        /* (FUSES_USER_WORD_1) No Hysteresis  */
#define   FUSES_USER_WORD_1_BOD33_HYST_ENABLED_Val _UINT32_(0x1)                                        /* (FUSES_USER_WORD_1) Hysteresis Enabled  */
#define FUSES_USER_WORD_1_BOD33_HYST_DISABLED (FUSES_USER_WORD_1_BOD33_HYST_DISABLED_Val << FUSES_USER_WORD_1_BOD33_HYST_Pos) /* (FUSES_USER_WORD_1) No Hysteresis Position  */
#define FUSES_USER_WORD_1_BOD33_HYST_ENABLED  (FUSES_USER_WORD_1_BOD33_HYST_ENABLED_Val << FUSES_USER_WORD_1_BOD33_HYST_Pos) /* (FUSES_USER_WORD_1) Hysteresis Enabled Position  */
#define FUSES_USER_WORD_1_NVMCTRL_REGION_LOCKS_Pos _UINT32_(16)                                         /* (FUSES_USER_WORD_1) NVM Region Locks Position */
#define FUSES_USER_WORD_1_NVMCTRL_REGION_LOCKS_Msk (_UINT32_(0xFFFF) << FUSES_USER_WORD_1_NVMCTRL_REGION_LOCKS_Pos) /* (FUSES_USER_WORD_1) NVM Region Locks Mask */
#define FUSES_USER_WORD_1_NVMCTRL_REGION_LOCKS(value) (FUSES_USER_WORD_1_NVMCTRL_REGION_LOCKS_Msk & (_UINT32_(value) << FUSES_USER_WORD_1_NVMCTRL_REGION_LOCKS_Pos)) /* Assigment of value for NVMCTRL_REGION_LOCKS in the FUSES_USER_WORD_1 register */
#define FUSES_USER_WORD_1_Msk                 _UINT32_(0xFFFF01FF)                                 /* (FUSES_USER_WORD_1) Register Mask  */


/** \brief FUSES register offsets definitions */
#define FUSES_TEMP_LOG_WORD_0_REG_OFST _UINT32_(0x00)      /* (FUSES_TEMP_LOG_WORD_0) TEMP_LOG Page Word 0 Offset */
#define FUSES_TEMP_LOG_WORD_1_REG_OFST _UINT32_(0x04)      /* (FUSES_TEMP_LOG_WORD_1) TEMP_LOG Page Word 1 Offset */
#define FUSES_OTP4_WORD_0_REG_OFST     _UINT32_(0x00)      /* (FUSES_OTP4_WORD_0) OTP4 Page Word 0 Offset */
#define FUSES_OTP4_WORD_1_REG_OFST     _UINT32_(0x04)      /* (FUSES_OTP4_WORD_1) OTP4 Page Word 1 Offset */
#define FUSES_OTP4_WORD_2_REG_OFST     _UINT32_(0x08)      /* (FUSES_OTP4_WORD_2) OTP4 Page Word 2 Offset */
#define FUSES_USER_WORD_0_REG_OFST     _UINT32_(0x00)      /* (FUSES_USER_WORD_0) USER Page Word 0 Offset */
#define FUSES_USER_WORD_1_REG_OFST     _UINT32_(0x04)      /* (FUSES_USER_WORD_1) USER Page Word 1 Offset */

#if !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__))
/** \brief TEMP_LOG_FUSES register API structure */
typedef struct
{
  __I   uint32_t                       FUSES_TEMP_LOG_WORD_0; /**< Offset: 0x00 (R/   32) TEMP_LOG Page Word 0 */
  __I   uint32_t                       FUSES_TEMP_LOG_WORD_1; /**< Offset: 0x04 (R/   32) TEMP_LOG Page Word 1 */
} fuses_temp_log_fuses_registers_t;

/** \brief OTP4_FUSES register API structure */
typedef struct
{
  __I   uint32_t                       FUSES_OTP4_WORD_0;  /**< Offset: 0x00 (R/   32) OTP4 Page Word 0 */
  __I   uint32_t                       FUSES_OTP4_WORD_1;  /**< Offset: 0x04 (R/   32) OTP4 Page Word 1 */
  __I   uint32_t                       FUSES_OTP4_WORD_2;  /**< Offset: 0x08 (R/   32) OTP4 Page Word 2 */
} fuses_otp4_fuses_registers_t;

/** \brief USER_FUSES register API structure */
typedef struct
{
  __IO  uint32_t                       FUSES_USER_WORD_0;  /**< Offset: 0x00 (R/W  32) USER Page Word 0 */
  __IO  uint32_t                       FUSES_USER_WORD_1;  /**< Offset: 0x04 (R/W  32) USER Page Word 1 */
} fuses_user_fuses_registers_t;


#endif /* !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__)) */
#endif /* _SAMD21_FUSES_COMPONENT_H_ */
