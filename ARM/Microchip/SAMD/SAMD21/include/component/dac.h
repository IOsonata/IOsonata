/*
 * Component description for DAC
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
#ifndef _SAMD21_DAC_COMPONENT_H_
#define _SAMD21_DAC_COMPONENT_H_

/* ************************************************************************** */
/*   SOFTWARE API DEFINITION FOR DAC                                          */
/* ************************************************************************** */

/* -------- DAC_CTRLA : (DAC Offset: 0x00) (R/W 8) Control A -------- */
#define DAC_CTRLA_RESETVALUE                  _UINT8_(0x00)                                        /*  (DAC_CTRLA) Control A  Reset Value */

#define DAC_CTRLA_SWRST_Pos                   _UINT8_(0)                                           /* (DAC_CTRLA) Software Reset Position */
#define DAC_CTRLA_SWRST_Msk                   (_UINT8_(0x1) << DAC_CTRLA_SWRST_Pos)                /* (DAC_CTRLA) Software Reset Mask */
#define DAC_CTRLA_SWRST(value)                (DAC_CTRLA_SWRST_Msk & (_UINT8_(value) << DAC_CTRLA_SWRST_Pos)) /* Assigment of value for SWRST in the DAC_CTRLA register */
#define DAC_CTRLA_ENABLE_Pos                  _UINT8_(1)                                           /* (DAC_CTRLA) Enable Position */
#define DAC_CTRLA_ENABLE_Msk                  (_UINT8_(0x1) << DAC_CTRLA_ENABLE_Pos)               /* (DAC_CTRLA) Enable Mask */
#define DAC_CTRLA_ENABLE(value)               (DAC_CTRLA_ENABLE_Msk & (_UINT8_(value) << DAC_CTRLA_ENABLE_Pos)) /* Assigment of value for ENABLE in the DAC_CTRLA register */
#define DAC_CTRLA_RUNSTDBY_Pos                _UINT8_(2)                                           /* (DAC_CTRLA) Run in Standby Position */
#define DAC_CTRLA_RUNSTDBY_Msk                (_UINT8_(0x1) << DAC_CTRLA_RUNSTDBY_Pos)             /* (DAC_CTRLA) Run in Standby Mask */
#define DAC_CTRLA_RUNSTDBY(value)             (DAC_CTRLA_RUNSTDBY_Msk & (_UINT8_(value) << DAC_CTRLA_RUNSTDBY_Pos)) /* Assigment of value for RUNSTDBY in the DAC_CTRLA register */
#define DAC_CTRLA_Msk                         _UINT8_(0x07)                                        /* (DAC_CTRLA) Register Mask  */


/* -------- DAC_CTRLB : (DAC Offset: 0x01) (R/W 8) Control B -------- */
#define DAC_CTRLB_RESETVALUE                  _UINT8_(0x00)                                        /*  (DAC_CTRLB) Control B  Reset Value */

#define DAC_CTRLB_EOEN_Pos                    _UINT8_(0)                                           /* (DAC_CTRLB) External Output Enable Position */
#define DAC_CTRLB_EOEN_Msk                    (_UINT8_(0x1) << DAC_CTRLB_EOEN_Pos)                 /* (DAC_CTRLB) External Output Enable Mask */
#define DAC_CTRLB_EOEN(value)                 (DAC_CTRLB_EOEN_Msk & (_UINT8_(value) << DAC_CTRLB_EOEN_Pos)) /* Assigment of value for EOEN in the DAC_CTRLB register */
#define DAC_CTRLB_IOEN_Pos                    _UINT8_(1)                                           /* (DAC_CTRLB) Internal Output Enable Position */
#define DAC_CTRLB_IOEN_Msk                    (_UINT8_(0x1) << DAC_CTRLB_IOEN_Pos)                 /* (DAC_CTRLB) Internal Output Enable Mask */
#define DAC_CTRLB_IOEN(value)                 (DAC_CTRLB_IOEN_Msk & (_UINT8_(value) << DAC_CTRLB_IOEN_Pos)) /* Assigment of value for IOEN in the DAC_CTRLB register */
#define DAC_CTRLB_LEFTADJ_Pos                 _UINT8_(2)                                           /* (DAC_CTRLB) Left Adjusted Data Position */
#define DAC_CTRLB_LEFTADJ_Msk                 (_UINT8_(0x1) << DAC_CTRLB_LEFTADJ_Pos)              /* (DAC_CTRLB) Left Adjusted Data Mask */
#define DAC_CTRLB_LEFTADJ(value)              (DAC_CTRLB_LEFTADJ_Msk & (_UINT8_(value) << DAC_CTRLB_LEFTADJ_Pos)) /* Assigment of value for LEFTADJ in the DAC_CTRLB register */
#define DAC_CTRLB_VPD_Pos                     _UINT8_(3)                                           /* (DAC_CTRLB) Voltage Pump Disable Position */
#define DAC_CTRLB_VPD_Msk                     (_UINT8_(0x1) << DAC_CTRLB_VPD_Pos)                  /* (DAC_CTRLB) Voltage Pump Disable Mask */
#define DAC_CTRLB_VPD(value)                  (DAC_CTRLB_VPD_Msk & (_UINT8_(value) << DAC_CTRLB_VPD_Pos)) /* Assigment of value for VPD in the DAC_CTRLB register */
#define DAC_CTRLB_BDWP_Pos                    _UINT8_(4)                                           /* (DAC_CTRLB) Bypass DATABUF Write Protection Position */
#define DAC_CTRLB_BDWP_Msk                    (_UINT8_(0x1) << DAC_CTRLB_BDWP_Pos)                 /* (DAC_CTRLB) Bypass DATABUF Write Protection Mask */
#define DAC_CTRLB_BDWP(value)                 (DAC_CTRLB_BDWP_Msk & (_UINT8_(value) << DAC_CTRLB_BDWP_Pos)) /* Assigment of value for BDWP in the DAC_CTRLB register */
#define DAC_CTRLB_REFSEL_Pos                  _UINT8_(6)                                           /* (DAC_CTRLB) Reference Selection Position */
#define DAC_CTRLB_REFSEL_Msk                  (_UINT8_(0x3) << DAC_CTRLB_REFSEL_Pos)               /* (DAC_CTRLB) Reference Selection Mask */
#define DAC_CTRLB_REFSEL(value)               (DAC_CTRLB_REFSEL_Msk & (_UINT8_(value) << DAC_CTRLB_REFSEL_Pos)) /* Assigment of value for REFSEL in the DAC_CTRLB register */
#define   DAC_CTRLB_REFSEL_INT1V_Val          _UINT8_(0x0)                                         /* (DAC_CTRLB) Internal 1.0V reference  */
#define   DAC_CTRLB_REFSEL_AVCC_Val           _UINT8_(0x1)                                         /* (DAC_CTRLB) AVCC  */
#define   DAC_CTRLB_REFSEL_VREFP_Val          _UINT8_(0x2)                                         /* (DAC_CTRLB) External reference  */
#define DAC_CTRLB_REFSEL_INT1V                (DAC_CTRLB_REFSEL_INT1V_Val << DAC_CTRLB_REFSEL_Pos) /* (DAC_CTRLB) Internal 1.0V reference Position  */
#define DAC_CTRLB_REFSEL_AVCC                 (DAC_CTRLB_REFSEL_AVCC_Val << DAC_CTRLB_REFSEL_Pos)  /* (DAC_CTRLB) AVCC Position  */
#define DAC_CTRLB_REFSEL_VREFP                (DAC_CTRLB_REFSEL_VREFP_Val << DAC_CTRLB_REFSEL_Pos) /* (DAC_CTRLB) External reference Position  */
#define DAC_CTRLB_Msk                         _UINT8_(0xDF)                                        /* (DAC_CTRLB) Register Mask  */


/* -------- DAC_EVCTRL : (DAC Offset: 0x02) (R/W 8) Event Control -------- */
#define DAC_EVCTRL_RESETVALUE                 _UINT8_(0x00)                                        /*  (DAC_EVCTRL) Event Control  Reset Value */

#define DAC_EVCTRL_STARTEI_Pos                _UINT8_(0)                                           /* (DAC_EVCTRL) Start Conversion Event Input Position */
#define DAC_EVCTRL_STARTEI_Msk                (_UINT8_(0x1) << DAC_EVCTRL_STARTEI_Pos)             /* (DAC_EVCTRL) Start Conversion Event Input Mask */
#define DAC_EVCTRL_STARTEI(value)             (DAC_EVCTRL_STARTEI_Msk & (_UINT8_(value) << DAC_EVCTRL_STARTEI_Pos)) /* Assigment of value for STARTEI in the DAC_EVCTRL register */
#define DAC_EVCTRL_EMPTYEO_Pos                _UINT8_(1)                                           /* (DAC_EVCTRL) Data Buffer Empty Event Output Position */
#define DAC_EVCTRL_EMPTYEO_Msk                (_UINT8_(0x1) << DAC_EVCTRL_EMPTYEO_Pos)             /* (DAC_EVCTRL) Data Buffer Empty Event Output Mask */
#define DAC_EVCTRL_EMPTYEO(value)             (DAC_EVCTRL_EMPTYEO_Msk & (_UINT8_(value) << DAC_EVCTRL_EMPTYEO_Pos)) /* Assigment of value for EMPTYEO in the DAC_EVCTRL register */
#define DAC_EVCTRL_Msk                        _UINT8_(0x03)                                        /* (DAC_EVCTRL) Register Mask  */


/* -------- DAC_INTENCLR : (DAC Offset: 0x04) (R/W 8) Interrupt Enable Clear -------- */
#define DAC_INTENCLR_RESETVALUE               _UINT8_(0x00)                                        /*  (DAC_INTENCLR) Interrupt Enable Clear  Reset Value */

#define DAC_INTENCLR_UNDERRUN_Pos             _UINT8_(0)                                           /* (DAC_INTENCLR) Underrun Interrupt Enable Position */
#define DAC_INTENCLR_UNDERRUN_Msk             (_UINT8_(0x1) << DAC_INTENCLR_UNDERRUN_Pos)          /* (DAC_INTENCLR) Underrun Interrupt Enable Mask */
#define DAC_INTENCLR_UNDERRUN(value)          (DAC_INTENCLR_UNDERRUN_Msk & (_UINT8_(value) << DAC_INTENCLR_UNDERRUN_Pos)) /* Assigment of value for UNDERRUN in the DAC_INTENCLR register */
#define DAC_INTENCLR_EMPTY_Pos                _UINT8_(1)                                           /* (DAC_INTENCLR) Data Buffer Empty Interrupt Enable Position */
#define DAC_INTENCLR_EMPTY_Msk                (_UINT8_(0x1) << DAC_INTENCLR_EMPTY_Pos)             /* (DAC_INTENCLR) Data Buffer Empty Interrupt Enable Mask */
#define DAC_INTENCLR_EMPTY(value)             (DAC_INTENCLR_EMPTY_Msk & (_UINT8_(value) << DAC_INTENCLR_EMPTY_Pos)) /* Assigment of value for EMPTY in the DAC_INTENCLR register */
#define DAC_INTENCLR_SYNCRDY_Pos              _UINT8_(2)                                           /* (DAC_INTENCLR) Synchronization Ready Interrupt Enable Position */
#define DAC_INTENCLR_SYNCRDY_Msk              (_UINT8_(0x1) << DAC_INTENCLR_SYNCRDY_Pos)           /* (DAC_INTENCLR) Synchronization Ready Interrupt Enable Mask */
#define DAC_INTENCLR_SYNCRDY(value)           (DAC_INTENCLR_SYNCRDY_Msk & (_UINT8_(value) << DAC_INTENCLR_SYNCRDY_Pos)) /* Assigment of value for SYNCRDY in the DAC_INTENCLR register */
#define DAC_INTENCLR_Msk                      _UINT8_(0x07)                                        /* (DAC_INTENCLR) Register Mask  */


/* -------- DAC_INTENSET : (DAC Offset: 0x05) (R/W 8) Interrupt Enable Set -------- */
#define DAC_INTENSET_RESETVALUE               _UINT8_(0x00)                                        /*  (DAC_INTENSET) Interrupt Enable Set  Reset Value */

#define DAC_INTENSET_UNDERRUN_Pos             _UINT8_(0)                                           /* (DAC_INTENSET) Underrun Interrupt Enable Position */
#define DAC_INTENSET_UNDERRUN_Msk             (_UINT8_(0x1) << DAC_INTENSET_UNDERRUN_Pos)          /* (DAC_INTENSET) Underrun Interrupt Enable Mask */
#define DAC_INTENSET_UNDERRUN(value)          (DAC_INTENSET_UNDERRUN_Msk & (_UINT8_(value) << DAC_INTENSET_UNDERRUN_Pos)) /* Assigment of value for UNDERRUN in the DAC_INTENSET register */
#define DAC_INTENSET_EMPTY_Pos                _UINT8_(1)                                           /* (DAC_INTENSET) Data Buffer Empty Interrupt Enable Position */
#define DAC_INTENSET_EMPTY_Msk                (_UINT8_(0x1) << DAC_INTENSET_EMPTY_Pos)             /* (DAC_INTENSET) Data Buffer Empty Interrupt Enable Mask */
#define DAC_INTENSET_EMPTY(value)             (DAC_INTENSET_EMPTY_Msk & (_UINT8_(value) << DAC_INTENSET_EMPTY_Pos)) /* Assigment of value for EMPTY in the DAC_INTENSET register */
#define DAC_INTENSET_SYNCRDY_Pos              _UINT8_(2)                                           /* (DAC_INTENSET) Synchronization Ready Interrupt Enable Position */
#define DAC_INTENSET_SYNCRDY_Msk              (_UINT8_(0x1) << DAC_INTENSET_SYNCRDY_Pos)           /* (DAC_INTENSET) Synchronization Ready Interrupt Enable Mask */
#define DAC_INTENSET_SYNCRDY(value)           (DAC_INTENSET_SYNCRDY_Msk & (_UINT8_(value) << DAC_INTENSET_SYNCRDY_Pos)) /* Assigment of value for SYNCRDY in the DAC_INTENSET register */
#define DAC_INTENSET_Msk                      _UINT8_(0x07)                                        /* (DAC_INTENSET) Register Mask  */


/* -------- DAC_INTFLAG : (DAC Offset: 0x06) (R/W 8) Interrupt Flag Status and Clear -------- */
#define DAC_INTFLAG_RESETVALUE                _UINT8_(0x00)                                        /*  (DAC_INTFLAG) Interrupt Flag Status and Clear  Reset Value */

#define DAC_INTFLAG_UNDERRUN_Pos              _UINT8_(0)                                           /* (DAC_INTFLAG) Underrun Position */
#define DAC_INTFLAG_UNDERRUN_Msk              (_UINT8_(0x1) << DAC_INTFLAG_UNDERRUN_Pos)           /* (DAC_INTFLAG) Underrun Mask */
#define DAC_INTFLAG_UNDERRUN(value)           (DAC_INTFLAG_UNDERRUN_Msk & (_UINT8_(value) << DAC_INTFLAG_UNDERRUN_Pos)) /* Assigment of value for UNDERRUN in the DAC_INTFLAG register */
#define DAC_INTFLAG_EMPTY_Pos                 _UINT8_(1)                                           /* (DAC_INTFLAG) Data Buffer Empty Position */
#define DAC_INTFLAG_EMPTY_Msk                 (_UINT8_(0x1) << DAC_INTFLAG_EMPTY_Pos)              /* (DAC_INTFLAG) Data Buffer Empty Mask */
#define DAC_INTFLAG_EMPTY(value)              (DAC_INTFLAG_EMPTY_Msk & (_UINT8_(value) << DAC_INTFLAG_EMPTY_Pos)) /* Assigment of value for EMPTY in the DAC_INTFLAG register */
#define DAC_INTFLAG_SYNCRDY_Pos               _UINT8_(2)                                           /* (DAC_INTFLAG) Synchronization Ready Position */
#define DAC_INTFLAG_SYNCRDY_Msk               (_UINT8_(0x1) << DAC_INTFLAG_SYNCRDY_Pos)            /* (DAC_INTFLAG) Synchronization Ready Mask */
#define DAC_INTFLAG_SYNCRDY(value)            (DAC_INTFLAG_SYNCRDY_Msk & (_UINT8_(value) << DAC_INTFLAG_SYNCRDY_Pos)) /* Assigment of value for SYNCRDY in the DAC_INTFLAG register */
#define DAC_INTFLAG_Msk                       _UINT8_(0x07)                                        /* (DAC_INTFLAG) Register Mask  */


/* -------- DAC_STATUS : (DAC Offset: 0x07) ( R/ 8) Status -------- */
#define DAC_STATUS_RESETVALUE                 _UINT8_(0x00)                                        /*  (DAC_STATUS) Status  Reset Value */

#define DAC_STATUS_SYNCBUSY_Pos               _UINT8_(7)                                           /* (DAC_STATUS) Synchronization Busy Status Position */
#define DAC_STATUS_SYNCBUSY_Msk               (_UINT8_(0x1) << DAC_STATUS_SYNCBUSY_Pos)            /* (DAC_STATUS) Synchronization Busy Status Mask */
#define DAC_STATUS_SYNCBUSY(value)            (DAC_STATUS_SYNCBUSY_Msk & (_UINT8_(value) << DAC_STATUS_SYNCBUSY_Pos)) /* Assigment of value for SYNCBUSY in the DAC_STATUS register */
#define DAC_STATUS_Msk                        _UINT8_(0x80)                                        /* (DAC_STATUS) Register Mask  */


/* -------- DAC_DATA : (DAC Offset: 0x08) (R/W 16) Data -------- */
#define DAC_DATA_RESETVALUE                   _UINT16_(0x00)                                       /*  (DAC_DATA) Data  Reset Value */

#define DAC_DATA_DATA_Pos                     _UINT16_(0)                                          /* (DAC_DATA) Data value to be converted Position */
#define DAC_DATA_DATA_Msk                     (_UINT16_(0xFFFF) << DAC_DATA_DATA_Pos)              /* (DAC_DATA) Data value to be converted Mask */
#define DAC_DATA_DATA(value)                  (DAC_DATA_DATA_Msk & (_UINT16_(value) << DAC_DATA_DATA_Pos)) /* Assigment of value for DATA in the DAC_DATA register */
#define DAC_DATA_Msk                          _UINT16_(0xFFFF)                                     /* (DAC_DATA) Register Mask  */


/* -------- DAC_DATABUF : (DAC Offset: 0x0C) (R/W 16) Data Buffer -------- */
#define DAC_DATABUF_RESETVALUE                _UINT16_(0x00)                                       /*  (DAC_DATABUF) Data Buffer  Reset Value */

#define DAC_DATABUF_DATABUF_Pos               _UINT16_(0)                                          /* (DAC_DATABUF) Data Buffer Position */
#define DAC_DATABUF_DATABUF_Msk               (_UINT16_(0xFFFF) << DAC_DATABUF_DATABUF_Pos)        /* (DAC_DATABUF) Data Buffer Mask */
#define DAC_DATABUF_DATABUF(value)            (DAC_DATABUF_DATABUF_Msk & (_UINT16_(value) << DAC_DATABUF_DATABUF_Pos)) /* Assigment of value for DATABUF in the DAC_DATABUF register */
#define DAC_DATABUF_Msk                       _UINT16_(0xFFFF)                                     /* (DAC_DATABUF) Register Mask  */


/** \brief DAC register offsets definitions */
#define DAC_CTRLA_REG_OFST             _UINT32_(0x00)      /* (DAC_CTRLA) Control A Offset */
#define DAC_CTRLB_REG_OFST             _UINT32_(0x01)      /* (DAC_CTRLB) Control B Offset */
#define DAC_EVCTRL_REG_OFST            _UINT32_(0x02)      /* (DAC_EVCTRL) Event Control Offset */
#define DAC_INTENCLR_REG_OFST          _UINT32_(0x04)      /* (DAC_INTENCLR) Interrupt Enable Clear Offset */
#define DAC_INTENSET_REG_OFST          _UINT32_(0x05)      /* (DAC_INTENSET) Interrupt Enable Set Offset */
#define DAC_INTFLAG_REG_OFST           _UINT32_(0x06)      /* (DAC_INTFLAG) Interrupt Flag Status and Clear Offset */
#define DAC_STATUS_REG_OFST            _UINT32_(0x07)      /* (DAC_STATUS) Status Offset */
#define DAC_DATA_REG_OFST              _UINT32_(0x08)      /* (DAC_DATA) Data Offset */
#define DAC_DATABUF_REG_OFST           _UINT32_(0x0C)      /* (DAC_DATABUF) Data Buffer Offset */

#if !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__))
/** \brief DAC register API structure */
typedef struct
{  /* Digital Analog Converter */
  __IO  uint8_t                        DAC_CTRLA;          /**< Offset: 0x00 (R/W  8) Control A */
  __IO  uint8_t                        DAC_CTRLB;          /**< Offset: 0x01 (R/W  8) Control B */
  __IO  uint8_t                        DAC_EVCTRL;         /**< Offset: 0x02 (R/W  8) Event Control */
  __I   uint8_t                        Reserved1[0x01];
  __IO  uint8_t                        DAC_INTENCLR;       /**< Offset: 0x04 (R/W  8) Interrupt Enable Clear */
  __IO  uint8_t                        DAC_INTENSET;       /**< Offset: 0x05 (R/W  8) Interrupt Enable Set */
  __IO  uint8_t                        DAC_INTFLAG;        /**< Offset: 0x06 (R/W  8) Interrupt Flag Status and Clear */
  __I   uint8_t                        DAC_STATUS;         /**< Offset: 0x07 (R/   8) Status */
  __IO  uint16_t                       DAC_DATA;           /**< Offset: 0x08 (R/W  16) Data */
  __I   uint8_t                        Reserved2[0x02];
  __IO  uint16_t                       DAC_DATABUF;        /**< Offset: 0x0C (R/W  16) Data Buffer */
} dac_registers_t;


#endif /* !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__)) */
#endif /* _SAMD21_DAC_COMPONENT_H_ */
