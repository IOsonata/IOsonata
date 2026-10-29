/*
 * Component description for SYSCTRL
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
#ifndef _SAMD21_SYSCTRL_COMPONENT_H_
#define _SAMD21_SYSCTRL_COMPONENT_H_

/* ************************************************************************** */
/*   SOFTWARE API DEFINITION FOR SYSCTRL                                      */
/* ************************************************************************** */

/* -------- SYSCTRL_INTENCLR : (SYSCTRL Offset: 0x00) (R/W 32) Interrupt Enable Clear -------- */
#define SYSCTRL_INTENCLR_RESETVALUE           _UINT32_(0x00)                                       /*  (SYSCTRL_INTENCLR) Interrupt Enable Clear  Reset Value */

#define SYSCTRL_INTENCLR_XOSCRDY_Pos          _UINT32_(0)                                          /* (SYSCTRL_INTENCLR) XOSC Ready Interrupt Enable Position */
#define SYSCTRL_INTENCLR_XOSCRDY_Msk          (_UINT32_(0x1) << SYSCTRL_INTENCLR_XOSCRDY_Pos)      /* (SYSCTRL_INTENCLR) XOSC Ready Interrupt Enable Mask */
#define SYSCTRL_INTENCLR_XOSCRDY(value)       (SYSCTRL_INTENCLR_XOSCRDY_Msk & (_UINT32_(value) << SYSCTRL_INTENCLR_XOSCRDY_Pos)) /* Assigment of value for XOSCRDY in the SYSCTRL_INTENCLR register */
#define SYSCTRL_INTENCLR_XOSC32KRDY_Pos       _UINT32_(1)                                          /* (SYSCTRL_INTENCLR) XOSC32K Ready Interrupt Enable Position */
#define SYSCTRL_INTENCLR_XOSC32KRDY_Msk       (_UINT32_(0x1) << SYSCTRL_INTENCLR_XOSC32KRDY_Pos)   /* (SYSCTRL_INTENCLR) XOSC32K Ready Interrupt Enable Mask */
#define SYSCTRL_INTENCLR_XOSC32KRDY(value)    (SYSCTRL_INTENCLR_XOSC32KRDY_Msk & (_UINT32_(value) << SYSCTRL_INTENCLR_XOSC32KRDY_Pos)) /* Assigment of value for XOSC32KRDY in the SYSCTRL_INTENCLR register */
#define SYSCTRL_INTENCLR_OSC32KRDY_Pos        _UINT32_(2)                                          /* (SYSCTRL_INTENCLR) OSC32K Ready Interrupt Enable Position */
#define SYSCTRL_INTENCLR_OSC32KRDY_Msk        (_UINT32_(0x1) << SYSCTRL_INTENCLR_OSC32KRDY_Pos)    /* (SYSCTRL_INTENCLR) OSC32K Ready Interrupt Enable Mask */
#define SYSCTRL_INTENCLR_OSC32KRDY(value)     (SYSCTRL_INTENCLR_OSC32KRDY_Msk & (_UINT32_(value) << SYSCTRL_INTENCLR_OSC32KRDY_Pos)) /* Assigment of value for OSC32KRDY in the SYSCTRL_INTENCLR register */
#define SYSCTRL_INTENCLR_OSC8MRDY_Pos         _UINT32_(3)                                          /* (SYSCTRL_INTENCLR) OSC8M Ready Interrupt Enable Position */
#define SYSCTRL_INTENCLR_OSC8MRDY_Msk         (_UINT32_(0x1) << SYSCTRL_INTENCLR_OSC8MRDY_Pos)     /* (SYSCTRL_INTENCLR) OSC8M Ready Interrupt Enable Mask */
#define SYSCTRL_INTENCLR_OSC8MRDY(value)      (SYSCTRL_INTENCLR_OSC8MRDY_Msk & (_UINT32_(value) << SYSCTRL_INTENCLR_OSC8MRDY_Pos)) /* Assigment of value for OSC8MRDY in the SYSCTRL_INTENCLR register */
#define SYSCTRL_INTENCLR_DFLLRDY_Pos          _UINT32_(4)                                          /* (SYSCTRL_INTENCLR) DFLL Ready Interrupt Enable Position */
#define SYSCTRL_INTENCLR_DFLLRDY_Msk          (_UINT32_(0x1) << SYSCTRL_INTENCLR_DFLLRDY_Pos)      /* (SYSCTRL_INTENCLR) DFLL Ready Interrupt Enable Mask */
#define SYSCTRL_INTENCLR_DFLLRDY(value)       (SYSCTRL_INTENCLR_DFLLRDY_Msk & (_UINT32_(value) << SYSCTRL_INTENCLR_DFLLRDY_Pos)) /* Assigment of value for DFLLRDY in the SYSCTRL_INTENCLR register */
#define SYSCTRL_INTENCLR_DFLLOOB_Pos          _UINT32_(5)                                          /* (SYSCTRL_INTENCLR) DFLL Out Of Bounds Interrupt Enable Position */
#define SYSCTRL_INTENCLR_DFLLOOB_Msk          (_UINT32_(0x1) << SYSCTRL_INTENCLR_DFLLOOB_Pos)      /* (SYSCTRL_INTENCLR) DFLL Out Of Bounds Interrupt Enable Mask */
#define SYSCTRL_INTENCLR_DFLLOOB(value)       (SYSCTRL_INTENCLR_DFLLOOB_Msk & (_UINT32_(value) << SYSCTRL_INTENCLR_DFLLOOB_Pos)) /* Assigment of value for DFLLOOB in the SYSCTRL_INTENCLR register */
#define SYSCTRL_INTENCLR_DFLLLCKF_Pos         _UINT32_(6)                                          /* (SYSCTRL_INTENCLR) DFLL Lock Fine Interrupt Enable Position */
#define SYSCTRL_INTENCLR_DFLLLCKF_Msk         (_UINT32_(0x1) << SYSCTRL_INTENCLR_DFLLLCKF_Pos)     /* (SYSCTRL_INTENCLR) DFLL Lock Fine Interrupt Enable Mask */
#define SYSCTRL_INTENCLR_DFLLLCKF(value)      (SYSCTRL_INTENCLR_DFLLLCKF_Msk & (_UINT32_(value) << SYSCTRL_INTENCLR_DFLLLCKF_Pos)) /* Assigment of value for DFLLLCKF in the SYSCTRL_INTENCLR register */
#define SYSCTRL_INTENCLR_DFLLLCKC_Pos         _UINT32_(7)                                          /* (SYSCTRL_INTENCLR) DFLL Lock Coarse Interrupt Enable Position */
#define SYSCTRL_INTENCLR_DFLLLCKC_Msk         (_UINT32_(0x1) << SYSCTRL_INTENCLR_DFLLLCKC_Pos)     /* (SYSCTRL_INTENCLR) DFLL Lock Coarse Interrupt Enable Mask */
#define SYSCTRL_INTENCLR_DFLLLCKC(value)      (SYSCTRL_INTENCLR_DFLLLCKC_Msk & (_UINT32_(value) << SYSCTRL_INTENCLR_DFLLLCKC_Pos)) /* Assigment of value for DFLLLCKC in the SYSCTRL_INTENCLR register */
#define SYSCTRL_INTENCLR_DFLLRCS_Pos          _UINT32_(8)                                          /* (SYSCTRL_INTENCLR) DFLL Reference Clock Stopped Interrupt Enable Position */
#define SYSCTRL_INTENCLR_DFLLRCS_Msk          (_UINT32_(0x1) << SYSCTRL_INTENCLR_DFLLRCS_Pos)      /* (SYSCTRL_INTENCLR) DFLL Reference Clock Stopped Interrupt Enable Mask */
#define SYSCTRL_INTENCLR_DFLLRCS(value)       (SYSCTRL_INTENCLR_DFLLRCS_Msk & (_UINT32_(value) << SYSCTRL_INTENCLR_DFLLRCS_Pos)) /* Assigment of value for DFLLRCS in the SYSCTRL_INTENCLR register */
#define SYSCTRL_INTENCLR_BOD33RDY_Pos         _UINT32_(9)                                          /* (SYSCTRL_INTENCLR) BOD33 Ready Interrupt Enable Position */
#define SYSCTRL_INTENCLR_BOD33RDY_Msk         (_UINT32_(0x1) << SYSCTRL_INTENCLR_BOD33RDY_Pos)     /* (SYSCTRL_INTENCLR) BOD33 Ready Interrupt Enable Mask */
#define SYSCTRL_INTENCLR_BOD33RDY(value)      (SYSCTRL_INTENCLR_BOD33RDY_Msk & (_UINT32_(value) << SYSCTRL_INTENCLR_BOD33RDY_Pos)) /* Assigment of value for BOD33RDY in the SYSCTRL_INTENCLR register */
#define SYSCTRL_INTENCLR_BOD33DET_Pos         _UINT32_(10)                                         /* (SYSCTRL_INTENCLR) BOD33 Detection Interrupt Enable Position */
#define SYSCTRL_INTENCLR_BOD33DET_Msk         (_UINT32_(0x1) << SYSCTRL_INTENCLR_BOD33DET_Pos)     /* (SYSCTRL_INTENCLR) BOD33 Detection Interrupt Enable Mask */
#define SYSCTRL_INTENCLR_BOD33DET(value)      (SYSCTRL_INTENCLR_BOD33DET_Msk & (_UINT32_(value) << SYSCTRL_INTENCLR_BOD33DET_Pos)) /* Assigment of value for BOD33DET in the SYSCTRL_INTENCLR register */
#define SYSCTRL_INTENCLR_B33SRDY_Pos          _UINT32_(11)                                         /* (SYSCTRL_INTENCLR) BOD33 Synchronization Ready Interrupt Enable Position */
#define SYSCTRL_INTENCLR_B33SRDY_Msk          (_UINT32_(0x1) << SYSCTRL_INTENCLR_B33SRDY_Pos)      /* (SYSCTRL_INTENCLR) BOD33 Synchronization Ready Interrupt Enable Mask */
#define SYSCTRL_INTENCLR_B33SRDY(value)       (SYSCTRL_INTENCLR_B33SRDY_Msk & (_UINT32_(value) << SYSCTRL_INTENCLR_B33SRDY_Pos)) /* Assigment of value for B33SRDY in the SYSCTRL_INTENCLR register */
#define SYSCTRL_INTENCLR_DPLLLCKR_Pos         _UINT32_(15)                                         /* (SYSCTRL_INTENCLR) DPLL Lock Rise Interrupt Enable Position */
#define SYSCTRL_INTENCLR_DPLLLCKR_Msk         (_UINT32_(0x1) << SYSCTRL_INTENCLR_DPLLLCKR_Pos)     /* (SYSCTRL_INTENCLR) DPLL Lock Rise Interrupt Enable Mask */
#define SYSCTRL_INTENCLR_DPLLLCKR(value)      (SYSCTRL_INTENCLR_DPLLLCKR_Msk & (_UINT32_(value) << SYSCTRL_INTENCLR_DPLLLCKR_Pos)) /* Assigment of value for DPLLLCKR in the SYSCTRL_INTENCLR register */
#define SYSCTRL_INTENCLR_DPLLLCKF_Pos         _UINT32_(16)                                         /* (SYSCTRL_INTENCLR) DPLL Lock Fall Interrupt Enable Position */
#define SYSCTRL_INTENCLR_DPLLLCKF_Msk         (_UINT32_(0x1) << SYSCTRL_INTENCLR_DPLLLCKF_Pos)     /* (SYSCTRL_INTENCLR) DPLL Lock Fall Interrupt Enable Mask */
#define SYSCTRL_INTENCLR_DPLLLCKF(value)      (SYSCTRL_INTENCLR_DPLLLCKF_Msk & (_UINT32_(value) << SYSCTRL_INTENCLR_DPLLLCKF_Pos)) /* Assigment of value for DPLLLCKF in the SYSCTRL_INTENCLR register */
#define SYSCTRL_INTENCLR_DPLLLTO_Pos          _UINT32_(17)                                         /* (SYSCTRL_INTENCLR) DPLL Lock Timeout Interrupt Enable Position */
#define SYSCTRL_INTENCLR_DPLLLTO_Msk          (_UINT32_(0x1) << SYSCTRL_INTENCLR_DPLLLTO_Pos)      /* (SYSCTRL_INTENCLR) DPLL Lock Timeout Interrupt Enable Mask */
#define SYSCTRL_INTENCLR_DPLLLTO(value)       (SYSCTRL_INTENCLR_DPLLLTO_Msk & (_UINT32_(value) << SYSCTRL_INTENCLR_DPLLLTO_Pos)) /* Assigment of value for DPLLLTO in the SYSCTRL_INTENCLR register */
#define SYSCTRL_INTENCLR_Msk                  _UINT32_(0x00038FFF)                                 /* (SYSCTRL_INTENCLR) Register Mask  */


/* -------- SYSCTRL_INTENSET : (SYSCTRL Offset: 0x04) (R/W 32) Interrupt Enable Set -------- */
#define SYSCTRL_INTENSET_RESETVALUE           _UINT32_(0x00)                                       /*  (SYSCTRL_INTENSET) Interrupt Enable Set  Reset Value */

#define SYSCTRL_INTENSET_XOSCRDY_Pos          _UINT32_(0)                                          /* (SYSCTRL_INTENSET) XOSC Ready Interrupt Enable Position */
#define SYSCTRL_INTENSET_XOSCRDY_Msk          (_UINT32_(0x1) << SYSCTRL_INTENSET_XOSCRDY_Pos)      /* (SYSCTRL_INTENSET) XOSC Ready Interrupt Enable Mask */
#define SYSCTRL_INTENSET_XOSCRDY(value)       (SYSCTRL_INTENSET_XOSCRDY_Msk & (_UINT32_(value) << SYSCTRL_INTENSET_XOSCRDY_Pos)) /* Assigment of value for XOSCRDY in the SYSCTRL_INTENSET register */
#define SYSCTRL_INTENSET_XOSC32KRDY_Pos       _UINT32_(1)                                          /* (SYSCTRL_INTENSET) XOSC32K Ready Interrupt Enable Position */
#define SYSCTRL_INTENSET_XOSC32KRDY_Msk       (_UINT32_(0x1) << SYSCTRL_INTENSET_XOSC32KRDY_Pos)   /* (SYSCTRL_INTENSET) XOSC32K Ready Interrupt Enable Mask */
#define SYSCTRL_INTENSET_XOSC32KRDY(value)    (SYSCTRL_INTENSET_XOSC32KRDY_Msk & (_UINT32_(value) << SYSCTRL_INTENSET_XOSC32KRDY_Pos)) /* Assigment of value for XOSC32KRDY in the SYSCTRL_INTENSET register */
#define SYSCTRL_INTENSET_OSC32KRDY_Pos        _UINT32_(2)                                          /* (SYSCTRL_INTENSET) OSC32K Ready Interrupt Enable Position */
#define SYSCTRL_INTENSET_OSC32KRDY_Msk        (_UINT32_(0x1) << SYSCTRL_INTENSET_OSC32KRDY_Pos)    /* (SYSCTRL_INTENSET) OSC32K Ready Interrupt Enable Mask */
#define SYSCTRL_INTENSET_OSC32KRDY(value)     (SYSCTRL_INTENSET_OSC32KRDY_Msk & (_UINT32_(value) << SYSCTRL_INTENSET_OSC32KRDY_Pos)) /* Assigment of value for OSC32KRDY in the SYSCTRL_INTENSET register */
#define SYSCTRL_INTENSET_OSC8MRDY_Pos         _UINT32_(3)                                          /* (SYSCTRL_INTENSET) OSC8M Ready Interrupt Enable Position */
#define SYSCTRL_INTENSET_OSC8MRDY_Msk         (_UINT32_(0x1) << SYSCTRL_INTENSET_OSC8MRDY_Pos)     /* (SYSCTRL_INTENSET) OSC8M Ready Interrupt Enable Mask */
#define SYSCTRL_INTENSET_OSC8MRDY(value)      (SYSCTRL_INTENSET_OSC8MRDY_Msk & (_UINT32_(value) << SYSCTRL_INTENSET_OSC8MRDY_Pos)) /* Assigment of value for OSC8MRDY in the SYSCTRL_INTENSET register */
#define SYSCTRL_INTENSET_DFLLRDY_Pos          _UINT32_(4)                                          /* (SYSCTRL_INTENSET) DFLL Ready Interrupt Enable Position */
#define SYSCTRL_INTENSET_DFLLRDY_Msk          (_UINT32_(0x1) << SYSCTRL_INTENSET_DFLLRDY_Pos)      /* (SYSCTRL_INTENSET) DFLL Ready Interrupt Enable Mask */
#define SYSCTRL_INTENSET_DFLLRDY(value)       (SYSCTRL_INTENSET_DFLLRDY_Msk & (_UINT32_(value) << SYSCTRL_INTENSET_DFLLRDY_Pos)) /* Assigment of value for DFLLRDY in the SYSCTRL_INTENSET register */
#define SYSCTRL_INTENSET_DFLLOOB_Pos          _UINT32_(5)                                          /* (SYSCTRL_INTENSET) DFLL Out Of Bounds Interrupt Enable Position */
#define SYSCTRL_INTENSET_DFLLOOB_Msk          (_UINT32_(0x1) << SYSCTRL_INTENSET_DFLLOOB_Pos)      /* (SYSCTRL_INTENSET) DFLL Out Of Bounds Interrupt Enable Mask */
#define SYSCTRL_INTENSET_DFLLOOB(value)       (SYSCTRL_INTENSET_DFLLOOB_Msk & (_UINT32_(value) << SYSCTRL_INTENSET_DFLLOOB_Pos)) /* Assigment of value for DFLLOOB in the SYSCTRL_INTENSET register */
#define SYSCTRL_INTENSET_DFLLLCKF_Pos         _UINT32_(6)                                          /* (SYSCTRL_INTENSET) DFLL Lock Fine Interrupt Enable Position */
#define SYSCTRL_INTENSET_DFLLLCKF_Msk         (_UINT32_(0x1) << SYSCTRL_INTENSET_DFLLLCKF_Pos)     /* (SYSCTRL_INTENSET) DFLL Lock Fine Interrupt Enable Mask */
#define SYSCTRL_INTENSET_DFLLLCKF(value)      (SYSCTRL_INTENSET_DFLLLCKF_Msk & (_UINT32_(value) << SYSCTRL_INTENSET_DFLLLCKF_Pos)) /* Assigment of value for DFLLLCKF in the SYSCTRL_INTENSET register */
#define SYSCTRL_INTENSET_DFLLLCKC_Pos         _UINT32_(7)                                          /* (SYSCTRL_INTENSET) DFLL Lock Coarse Interrupt Enable Position */
#define SYSCTRL_INTENSET_DFLLLCKC_Msk         (_UINT32_(0x1) << SYSCTRL_INTENSET_DFLLLCKC_Pos)     /* (SYSCTRL_INTENSET) DFLL Lock Coarse Interrupt Enable Mask */
#define SYSCTRL_INTENSET_DFLLLCKC(value)      (SYSCTRL_INTENSET_DFLLLCKC_Msk & (_UINT32_(value) << SYSCTRL_INTENSET_DFLLLCKC_Pos)) /* Assigment of value for DFLLLCKC in the SYSCTRL_INTENSET register */
#define SYSCTRL_INTENSET_DFLLRCS_Pos          _UINT32_(8)                                          /* (SYSCTRL_INTENSET) DFLL Reference Clock Stopped Interrupt Enable Position */
#define SYSCTRL_INTENSET_DFLLRCS_Msk          (_UINT32_(0x1) << SYSCTRL_INTENSET_DFLLRCS_Pos)      /* (SYSCTRL_INTENSET) DFLL Reference Clock Stopped Interrupt Enable Mask */
#define SYSCTRL_INTENSET_DFLLRCS(value)       (SYSCTRL_INTENSET_DFLLRCS_Msk & (_UINT32_(value) << SYSCTRL_INTENSET_DFLLRCS_Pos)) /* Assigment of value for DFLLRCS in the SYSCTRL_INTENSET register */
#define SYSCTRL_INTENSET_BOD33RDY_Pos         _UINT32_(9)                                          /* (SYSCTRL_INTENSET) BOD33 Ready Interrupt Enable Position */
#define SYSCTRL_INTENSET_BOD33RDY_Msk         (_UINT32_(0x1) << SYSCTRL_INTENSET_BOD33RDY_Pos)     /* (SYSCTRL_INTENSET) BOD33 Ready Interrupt Enable Mask */
#define SYSCTRL_INTENSET_BOD33RDY(value)      (SYSCTRL_INTENSET_BOD33RDY_Msk & (_UINT32_(value) << SYSCTRL_INTENSET_BOD33RDY_Pos)) /* Assigment of value for BOD33RDY in the SYSCTRL_INTENSET register */
#define SYSCTRL_INTENSET_BOD33DET_Pos         _UINT32_(10)                                         /* (SYSCTRL_INTENSET) BOD33 Detection Interrupt Enable Position */
#define SYSCTRL_INTENSET_BOD33DET_Msk         (_UINT32_(0x1) << SYSCTRL_INTENSET_BOD33DET_Pos)     /* (SYSCTRL_INTENSET) BOD33 Detection Interrupt Enable Mask */
#define SYSCTRL_INTENSET_BOD33DET(value)      (SYSCTRL_INTENSET_BOD33DET_Msk & (_UINT32_(value) << SYSCTRL_INTENSET_BOD33DET_Pos)) /* Assigment of value for BOD33DET in the SYSCTRL_INTENSET register */
#define SYSCTRL_INTENSET_B33SRDY_Pos          _UINT32_(11)                                         /* (SYSCTRL_INTENSET) BOD33 Synchronization Ready Interrupt Enable Position */
#define SYSCTRL_INTENSET_B33SRDY_Msk          (_UINT32_(0x1) << SYSCTRL_INTENSET_B33SRDY_Pos)      /* (SYSCTRL_INTENSET) BOD33 Synchronization Ready Interrupt Enable Mask */
#define SYSCTRL_INTENSET_B33SRDY(value)       (SYSCTRL_INTENSET_B33SRDY_Msk & (_UINT32_(value) << SYSCTRL_INTENSET_B33SRDY_Pos)) /* Assigment of value for B33SRDY in the SYSCTRL_INTENSET register */
#define SYSCTRL_INTENSET_DPLLLCKR_Pos         _UINT32_(15)                                         /* (SYSCTRL_INTENSET) DPLL Lock Rise Interrupt Enable Position */
#define SYSCTRL_INTENSET_DPLLLCKR_Msk         (_UINT32_(0x1) << SYSCTRL_INTENSET_DPLLLCKR_Pos)     /* (SYSCTRL_INTENSET) DPLL Lock Rise Interrupt Enable Mask */
#define SYSCTRL_INTENSET_DPLLLCKR(value)      (SYSCTRL_INTENSET_DPLLLCKR_Msk & (_UINT32_(value) << SYSCTRL_INTENSET_DPLLLCKR_Pos)) /* Assigment of value for DPLLLCKR in the SYSCTRL_INTENSET register */
#define SYSCTRL_INTENSET_DPLLLCKF_Pos         _UINT32_(16)                                         /* (SYSCTRL_INTENSET) DPLL Lock Fall Interrupt Enable Position */
#define SYSCTRL_INTENSET_DPLLLCKF_Msk         (_UINT32_(0x1) << SYSCTRL_INTENSET_DPLLLCKF_Pos)     /* (SYSCTRL_INTENSET) DPLL Lock Fall Interrupt Enable Mask */
#define SYSCTRL_INTENSET_DPLLLCKF(value)      (SYSCTRL_INTENSET_DPLLLCKF_Msk & (_UINT32_(value) << SYSCTRL_INTENSET_DPLLLCKF_Pos)) /* Assigment of value for DPLLLCKF in the SYSCTRL_INTENSET register */
#define SYSCTRL_INTENSET_DPLLLTO_Pos          _UINT32_(17)                                         /* (SYSCTRL_INTENSET) DPLL Lock Timeout Interrupt Enable Position */
#define SYSCTRL_INTENSET_DPLLLTO_Msk          (_UINT32_(0x1) << SYSCTRL_INTENSET_DPLLLTO_Pos)      /* (SYSCTRL_INTENSET) DPLL Lock Timeout Interrupt Enable Mask */
#define SYSCTRL_INTENSET_DPLLLTO(value)       (SYSCTRL_INTENSET_DPLLLTO_Msk & (_UINT32_(value) << SYSCTRL_INTENSET_DPLLLTO_Pos)) /* Assigment of value for DPLLLTO in the SYSCTRL_INTENSET register */
#define SYSCTRL_INTENSET_Msk                  _UINT32_(0x00038FFF)                                 /* (SYSCTRL_INTENSET) Register Mask  */


/* -------- SYSCTRL_INTFLAG : (SYSCTRL Offset: 0x08) (R/W 32) Interrupt Flag Status and Clear -------- */
#define SYSCTRL_INTFLAG_RESETVALUE            _UINT32_(0x00)                                       /*  (SYSCTRL_INTFLAG) Interrupt Flag Status and Clear  Reset Value */

#define SYSCTRL_INTFLAG_XOSCRDY_Pos           _UINT32_(0)                                          /* (SYSCTRL_INTFLAG) XOSC Ready Position */
#define SYSCTRL_INTFLAG_XOSCRDY_Msk           (_UINT32_(0x1) << SYSCTRL_INTFLAG_XOSCRDY_Pos)       /* (SYSCTRL_INTFLAG) XOSC Ready Mask */
#define SYSCTRL_INTFLAG_XOSCRDY(value)        (SYSCTRL_INTFLAG_XOSCRDY_Msk & (_UINT32_(value) << SYSCTRL_INTFLAG_XOSCRDY_Pos)) /* Assigment of value for XOSCRDY in the SYSCTRL_INTFLAG register */
#define SYSCTRL_INTFLAG_XOSC32KRDY_Pos        _UINT32_(1)                                          /* (SYSCTRL_INTFLAG) XOSC32K Ready Position */
#define SYSCTRL_INTFLAG_XOSC32KRDY_Msk        (_UINT32_(0x1) << SYSCTRL_INTFLAG_XOSC32KRDY_Pos)    /* (SYSCTRL_INTFLAG) XOSC32K Ready Mask */
#define SYSCTRL_INTFLAG_XOSC32KRDY(value)     (SYSCTRL_INTFLAG_XOSC32KRDY_Msk & (_UINT32_(value) << SYSCTRL_INTFLAG_XOSC32KRDY_Pos)) /* Assigment of value for XOSC32KRDY in the SYSCTRL_INTFLAG register */
#define SYSCTRL_INTFLAG_OSC32KRDY_Pos         _UINT32_(2)                                          /* (SYSCTRL_INTFLAG) OSC32K Ready Position */
#define SYSCTRL_INTFLAG_OSC32KRDY_Msk         (_UINT32_(0x1) << SYSCTRL_INTFLAG_OSC32KRDY_Pos)     /* (SYSCTRL_INTFLAG) OSC32K Ready Mask */
#define SYSCTRL_INTFLAG_OSC32KRDY(value)      (SYSCTRL_INTFLAG_OSC32KRDY_Msk & (_UINT32_(value) << SYSCTRL_INTFLAG_OSC32KRDY_Pos)) /* Assigment of value for OSC32KRDY in the SYSCTRL_INTFLAG register */
#define SYSCTRL_INTFLAG_OSC8MRDY_Pos          _UINT32_(3)                                          /* (SYSCTRL_INTFLAG) OSC8M Ready Position */
#define SYSCTRL_INTFLAG_OSC8MRDY_Msk          (_UINT32_(0x1) << SYSCTRL_INTFLAG_OSC8MRDY_Pos)      /* (SYSCTRL_INTFLAG) OSC8M Ready Mask */
#define SYSCTRL_INTFLAG_OSC8MRDY(value)       (SYSCTRL_INTFLAG_OSC8MRDY_Msk & (_UINT32_(value) << SYSCTRL_INTFLAG_OSC8MRDY_Pos)) /* Assigment of value for OSC8MRDY in the SYSCTRL_INTFLAG register */
#define SYSCTRL_INTFLAG_DFLLRDY_Pos           _UINT32_(4)                                          /* (SYSCTRL_INTFLAG) DFLL Ready Position */
#define SYSCTRL_INTFLAG_DFLLRDY_Msk           (_UINT32_(0x1) << SYSCTRL_INTFLAG_DFLLRDY_Pos)       /* (SYSCTRL_INTFLAG) DFLL Ready Mask */
#define SYSCTRL_INTFLAG_DFLLRDY(value)        (SYSCTRL_INTFLAG_DFLLRDY_Msk & (_UINT32_(value) << SYSCTRL_INTFLAG_DFLLRDY_Pos)) /* Assigment of value for DFLLRDY in the SYSCTRL_INTFLAG register */
#define SYSCTRL_INTFLAG_DFLLOOB_Pos           _UINT32_(5)                                          /* (SYSCTRL_INTFLAG) DFLL Out Of Bounds Position */
#define SYSCTRL_INTFLAG_DFLLOOB_Msk           (_UINT32_(0x1) << SYSCTRL_INTFLAG_DFLLOOB_Pos)       /* (SYSCTRL_INTFLAG) DFLL Out Of Bounds Mask */
#define SYSCTRL_INTFLAG_DFLLOOB(value)        (SYSCTRL_INTFLAG_DFLLOOB_Msk & (_UINT32_(value) << SYSCTRL_INTFLAG_DFLLOOB_Pos)) /* Assigment of value for DFLLOOB in the SYSCTRL_INTFLAG register */
#define SYSCTRL_INTFLAG_DFLLLCKF_Pos          _UINT32_(6)                                          /* (SYSCTRL_INTFLAG) DFLL Lock Fine Position */
#define SYSCTRL_INTFLAG_DFLLLCKF_Msk          (_UINT32_(0x1) << SYSCTRL_INTFLAG_DFLLLCKF_Pos)      /* (SYSCTRL_INTFLAG) DFLL Lock Fine Mask */
#define SYSCTRL_INTFLAG_DFLLLCKF(value)       (SYSCTRL_INTFLAG_DFLLLCKF_Msk & (_UINT32_(value) << SYSCTRL_INTFLAG_DFLLLCKF_Pos)) /* Assigment of value for DFLLLCKF in the SYSCTRL_INTFLAG register */
#define SYSCTRL_INTFLAG_DFLLLCKC_Pos          _UINT32_(7)                                          /* (SYSCTRL_INTFLAG) DFLL Lock Coarse Position */
#define SYSCTRL_INTFLAG_DFLLLCKC_Msk          (_UINT32_(0x1) << SYSCTRL_INTFLAG_DFLLLCKC_Pos)      /* (SYSCTRL_INTFLAG) DFLL Lock Coarse Mask */
#define SYSCTRL_INTFLAG_DFLLLCKC(value)       (SYSCTRL_INTFLAG_DFLLLCKC_Msk & (_UINT32_(value) << SYSCTRL_INTFLAG_DFLLLCKC_Pos)) /* Assigment of value for DFLLLCKC in the SYSCTRL_INTFLAG register */
#define SYSCTRL_INTFLAG_DFLLRCS_Pos           _UINT32_(8)                                          /* (SYSCTRL_INTFLAG) DFLL Reference Clock Stopped Position */
#define SYSCTRL_INTFLAG_DFLLRCS_Msk           (_UINT32_(0x1) << SYSCTRL_INTFLAG_DFLLRCS_Pos)       /* (SYSCTRL_INTFLAG) DFLL Reference Clock Stopped Mask */
#define SYSCTRL_INTFLAG_DFLLRCS(value)        (SYSCTRL_INTFLAG_DFLLRCS_Msk & (_UINT32_(value) << SYSCTRL_INTFLAG_DFLLRCS_Pos)) /* Assigment of value for DFLLRCS in the SYSCTRL_INTFLAG register */
#define SYSCTRL_INTFLAG_BOD33RDY_Pos          _UINT32_(9)                                          /* (SYSCTRL_INTFLAG) BOD33 Ready Position */
#define SYSCTRL_INTFLAG_BOD33RDY_Msk          (_UINT32_(0x1) << SYSCTRL_INTFLAG_BOD33RDY_Pos)      /* (SYSCTRL_INTFLAG) BOD33 Ready Mask */
#define SYSCTRL_INTFLAG_BOD33RDY(value)       (SYSCTRL_INTFLAG_BOD33RDY_Msk & (_UINT32_(value) << SYSCTRL_INTFLAG_BOD33RDY_Pos)) /* Assigment of value for BOD33RDY in the SYSCTRL_INTFLAG register */
#define SYSCTRL_INTFLAG_BOD33DET_Pos          _UINT32_(10)                                         /* (SYSCTRL_INTFLAG) BOD33 Detection Position */
#define SYSCTRL_INTFLAG_BOD33DET_Msk          (_UINT32_(0x1) << SYSCTRL_INTFLAG_BOD33DET_Pos)      /* (SYSCTRL_INTFLAG) BOD33 Detection Mask */
#define SYSCTRL_INTFLAG_BOD33DET(value)       (SYSCTRL_INTFLAG_BOD33DET_Msk & (_UINT32_(value) << SYSCTRL_INTFLAG_BOD33DET_Pos)) /* Assigment of value for BOD33DET in the SYSCTRL_INTFLAG register */
#define SYSCTRL_INTFLAG_B33SRDY_Pos           _UINT32_(11)                                         /* (SYSCTRL_INTFLAG) BOD33 Synchronization Ready Position */
#define SYSCTRL_INTFLAG_B33SRDY_Msk           (_UINT32_(0x1) << SYSCTRL_INTFLAG_B33SRDY_Pos)       /* (SYSCTRL_INTFLAG) BOD33 Synchronization Ready Mask */
#define SYSCTRL_INTFLAG_B33SRDY(value)        (SYSCTRL_INTFLAG_B33SRDY_Msk & (_UINT32_(value) << SYSCTRL_INTFLAG_B33SRDY_Pos)) /* Assigment of value for B33SRDY in the SYSCTRL_INTFLAG register */
#define SYSCTRL_INTFLAG_DPLLLCKR_Pos          _UINT32_(15)                                         /* (SYSCTRL_INTFLAG) DPLL Lock Rise Position */
#define SYSCTRL_INTFLAG_DPLLLCKR_Msk          (_UINT32_(0x1) << SYSCTRL_INTFLAG_DPLLLCKR_Pos)      /* (SYSCTRL_INTFLAG) DPLL Lock Rise Mask */
#define SYSCTRL_INTFLAG_DPLLLCKR(value)       (SYSCTRL_INTFLAG_DPLLLCKR_Msk & (_UINT32_(value) << SYSCTRL_INTFLAG_DPLLLCKR_Pos)) /* Assigment of value for DPLLLCKR in the SYSCTRL_INTFLAG register */
#define SYSCTRL_INTFLAG_DPLLLCKF_Pos          _UINT32_(16)                                         /* (SYSCTRL_INTFLAG) DPLL Lock Fall Position */
#define SYSCTRL_INTFLAG_DPLLLCKF_Msk          (_UINT32_(0x1) << SYSCTRL_INTFLAG_DPLLLCKF_Pos)      /* (SYSCTRL_INTFLAG) DPLL Lock Fall Mask */
#define SYSCTRL_INTFLAG_DPLLLCKF(value)       (SYSCTRL_INTFLAG_DPLLLCKF_Msk & (_UINT32_(value) << SYSCTRL_INTFLAG_DPLLLCKF_Pos)) /* Assigment of value for DPLLLCKF in the SYSCTRL_INTFLAG register */
#define SYSCTRL_INTFLAG_DPLLLTO_Pos           _UINT32_(17)                                         /* (SYSCTRL_INTFLAG) DPLL Lock Timeout Position */
#define SYSCTRL_INTFLAG_DPLLLTO_Msk           (_UINT32_(0x1) << SYSCTRL_INTFLAG_DPLLLTO_Pos)       /* (SYSCTRL_INTFLAG) DPLL Lock Timeout Mask */
#define SYSCTRL_INTFLAG_DPLLLTO(value)        (SYSCTRL_INTFLAG_DPLLLTO_Msk & (_UINT32_(value) << SYSCTRL_INTFLAG_DPLLLTO_Pos)) /* Assigment of value for DPLLLTO in the SYSCTRL_INTFLAG register */
#define SYSCTRL_INTFLAG_Msk                   _UINT32_(0x00038FFF)                                 /* (SYSCTRL_INTFLAG) Register Mask  */


/* -------- SYSCTRL_PCLKSR : (SYSCTRL Offset: 0x0C) ( R/ 32) Power and Clocks Status -------- */
#define SYSCTRL_PCLKSR_RESETVALUE             _UINT32_(0x00)                                       /*  (SYSCTRL_PCLKSR) Power and Clocks Status  Reset Value */

#define SYSCTRL_PCLKSR_XOSCRDY_Pos            _UINT32_(0)                                          /* (SYSCTRL_PCLKSR) XOSC Ready Position */
#define SYSCTRL_PCLKSR_XOSCRDY_Msk            (_UINT32_(0x1) << SYSCTRL_PCLKSR_XOSCRDY_Pos)        /* (SYSCTRL_PCLKSR) XOSC Ready Mask */
#define SYSCTRL_PCLKSR_XOSCRDY(value)         (SYSCTRL_PCLKSR_XOSCRDY_Msk & (_UINT32_(value) << SYSCTRL_PCLKSR_XOSCRDY_Pos)) /* Assigment of value for XOSCRDY in the SYSCTRL_PCLKSR register */
#define SYSCTRL_PCLKSR_XOSC32KRDY_Pos         _UINT32_(1)                                          /* (SYSCTRL_PCLKSR) XOSC32K Ready Position */
#define SYSCTRL_PCLKSR_XOSC32KRDY_Msk         (_UINT32_(0x1) << SYSCTRL_PCLKSR_XOSC32KRDY_Pos)     /* (SYSCTRL_PCLKSR) XOSC32K Ready Mask */
#define SYSCTRL_PCLKSR_XOSC32KRDY(value)      (SYSCTRL_PCLKSR_XOSC32KRDY_Msk & (_UINT32_(value) << SYSCTRL_PCLKSR_XOSC32KRDY_Pos)) /* Assigment of value for XOSC32KRDY in the SYSCTRL_PCLKSR register */
#define SYSCTRL_PCLKSR_OSC32KRDY_Pos          _UINT32_(2)                                          /* (SYSCTRL_PCLKSR) OSC32K Ready Position */
#define SYSCTRL_PCLKSR_OSC32KRDY_Msk          (_UINT32_(0x1) << SYSCTRL_PCLKSR_OSC32KRDY_Pos)      /* (SYSCTRL_PCLKSR) OSC32K Ready Mask */
#define SYSCTRL_PCLKSR_OSC32KRDY(value)       (SYSCTRL_PCLKSR_OSC32KRDY_Msk & (_UINT32_(value) << SYSCTRL_PCLKSR_OSC32KRDY_Pos)) /* Assigment of value for OSC32KRDY in the SYSCTRL_PCLKSR register */
#define SYSCTRL_PCLKSR_OSC8MRDY_Pos           _UINT32_(3)                                          /* (SYSCTRL_PCLKSR) OSC8M Ready Position */
#define SYSCTRL_PCLKSR_OSC8MRDY_Msk           (_UINT32_(0x1) << SYSCTRL_PCLKSR_OSC8MRDY_Pos)       /* (SYSCTRL_PCLKSR) OSC8M Ready Mask */
#define SYSCTRL_PCLKSR_OSC8MRDY(value)        (SYSCTRL_PCLKSR_OSC8MRDY_Msk & (_UINT32_(value) << SYSCTRL_PCLKSR_OSC8MRDY_Pos)) /* Assigment of value for OSC8MRDY in the SYSCTRL_PCLKSR register */
#define SYSCTRL_PCLKSR_DFLLRDY_Pos            _UINT32_(4)                                          /* (SYSCTRL_PCLKSR) DFLL Ready Position */
#define SYSCTRL_PCLKSR_DFLLRDY_Msk            (_UINT32_(0x1) << SYSCTRL_PCLKSR_DFLLRDY_Pos)        /* (SYSCTRL_PCLKSR) DFLL Ready Mask */
#define SYSCTRL_PCLKSR_DFLLRDY(value)         (SYSCTRL_PCLKSR_DFLLRDY_Msk & (_UINT32_(value) << SYSCTRL_PCLKSR_DFLLRDY_Pos)) /* Assigment of value for DFLLRDY in the SYSCTRL_PCLKSR register */
#define SYSCTRL_PCLKSR_DFLLOOB_Pos            _UINT32_(5)                                          /* (SYSCTRL_PCLKSR) DFLL Out Of Bounds Position */
#define SYSCTRL_PCLKSR_DFLLOOB_Msk            (_UINT32_(0x1) << SYSCTRL_PCLKSR_DFLLOOB_Pos)        /* (SYSCTRL_PCLKSR) DFLL Out Of Bounds Mask */
#define SYSCTRL_PCLKSR_DFLLOOB(value)         (SYSCTRL_PCLKSR_DFLLOOB_Msk & (_UINT32_(value) << SYSCTRL_PCLKSR_DFLLOOB_Pos)) /* Assigment of value for DFLLOOB in the SYSCTRL_PCLKSR register */
#define SYSCTRL_PCLKSR_DFLLLCKF_Pos           _UINT32_(6)                                          /* (SYSCTRL_PCLKSR) DFLL Lock Fine Position */
#define SYSCTRL_PCLKSR_DFLLLCKF_Msk           (_UINT32_(0x1) << SYSCTRL_PCLKSR_DFLLLCKF_Pos)       /* (SYSCTRL_PCLKSR) DFLL Lock Fine Mask */
#define SYSCTRL_PCLKSR_DFLLLCKF(value)        (SYSCTRL_PCLKSR_DFLLLCKF_Msk & (_UINT32_(value) << SYSCTRL_PCLKSR_DFLLLCKF_Pos)) /* Assigment of value for DFLLLCKF in the SYSCTRL_PCLKSR register */
#define SYSCTRL_PCLKSR_DFLLLCKC_Pos           _UINT32_(7)                                          /* (SYSCTRL_PCLKSR) DFLL Lock Coarse Position */
#define SYSCTRL_PCLKSR_DFLLLCKC_Msk           (_UINT32_(0x1) << SYSCTRL_PCLKSR_DFLLLCKC_Pos)       /* (SYSCTRL_PCLKSR) DFLL Lock Coarse Mask */
#define SYSCTRL_PCLKSR_DFLLLCKC(value)        (SYSCTRL_PCLKSR_DFLLLCKC_Msk & (_UINT32_(value) << SYSCTRL_PCLKSR_DFLLLCKC_Pos)) /* Assigment of value for DFLLLCKC in the SYSCTRL_PCLKSR register */
#define SYSCTRL_PCLKSR_DFLLRCS_Pos            _UINT32_(8)                                          /* (SYSCTRL_PCLKSR) DFLL Reference Clock Stopped Position */
#define SYSCTRL_PCLKSR_DFLLRCS_Msk            (_UINT32_(0x1) << SYSCTRL_PCLKSR_DFLLRCS_Pos)        /* (SYSCTRL_PCLKSR) DFLL Reference Clock Stopped Mask */
#define SYSCTRL_PCLKSR_DFLLRCS(value)         (SYSCTRL_PCLKSR_DFLLRCS_Msk & (_UINT32_(value) << SYSCTRL_PCLKSR_DFLLRCS_Pos)) /* Assigment of value for DFLLRCS in the SYSCTRL_PCLKSR register */
#define SYSCTRL_PCLKSR_BOD33RDY_Pos           _UINT32_(9)                                          /* (SYSCTRL_PCLKSR) BOD33 Ready Position */
#define SYSCTRL_PCLKSR_BOD33RDY_Msk           (_UINT32_(0x1) << SYSCTRL_PCLKSR_BOD33RDY_Pos)       /* (SYSCTRL_PCLKSR) BOD33 Ready Mask */
#define SYSCTRL_PCLKSR_BOD33RDY(value)        (SYSCTRL_PCLKSR_BOD33RDY_Msk & (_UINT32_(value) << SYSCTRL_PCLKSR_BOD33RDY_Pos)) /* Assigment of value for BOD33RDY in the SYSCTRL_PCLKSR register */
#define SYSCTRL_PCLKSR_BOD33DET_Pos           _UINT32_(10)                                         /* (SYSCTRL_PCLKSR) BOD33 Detection Position */
#define SYSCTRL_PCLKSR_BOD33DET_Msk           (_UINT32_(0x1) << SYSCTRL_PCLKSR_BOD33DET_Pos)       /* (SYSCTRL_PCLKSR) BOD33 Detection Mask */
#define SYSCTRL_PCLKSR_BOD33DET(value)        (SYSCTRL_PCLKSR_BOD33DET_Msk & (_UINT32_(value) << SYSCTRL_PCLKSR_BOD33DET_Pos)) /* Assigment of value for BOD33DET in the SYSCTRL_PCLKSR register */
#define SYSCTRL_PCLKSR_B33SRDY_Pos            _UINT32_(11)                                         /* (SYSCTRL_PCLKSR) BOD33 Synchronization Ready Position */
#define SYSCTRL_PCLKSR_B33SRDY_Msk            (_UINT32_(0x1) << SYSCTRL_PCLKSR_B33SRDY_Pos)        /* (SYSCTRL_PCLKSR) BOD33 Synchronization Ready Mask */
#define SYSCTRL_PCLKSR_B33SRDY(value)         (SYSCTRL_PCLKSR_B33SRDY_Msk & (_UINT32_(value) << SYSCTRL_PCLKSR_B33SRDY_Pos)) /* Assigment of value for B33SRDY in the SYSCTRL_PCLKSR register */
#define SYSCTRL_PCLKSR_DPLLLCKR_Pos           _UINT32_(15)                                         /* (SYSCTRL_PCLKSR) DPLL Lock Rise Position */
#define SYSCTRL_PCLKSR_DPLLLCKR_Msk           (_UINT32_(0x1) << SYSCTRL_PCLKSR_DPLLLCKR_Pos)       /* (SYSCTRL_PCLKSR) DPLL Lock Rise Mask */
#define SYSCTRL_PCLKSR_DPLLLCKR(value)        (SYSCTRL_PCLKSR_DPLLLCKR_Msk & (_UINT32_(value) << SYSCTRL_PCLKSR_DPLLLCKR_Pos)) /* Assigment of value for DPLLLCKR in the SYSCTRL_PCLKSR register */
#define SYSCTRL_PCLKSR_DPLLLCKF_Pos           _UINT32_(16)                                         /* (SYSCTRL_PCLKSR) DPLL Lock Fall Position */
#define SYSCTRL_PCLKSR_DPLLLCKF_Msk           (_UINT32_(0x1) << SYSCTRL_PCLKSR_DPLLLCKF_Pos)       /* (SYSCTRL_PCLKSR) DPLL Lock Fall Mask */
#define SYSCTRL_PCLKSR_DPLLLCKF(value)        (SYSCTRL_PCLKSR_DPLLLCKF_Msk & (_UINT32_(value) << SYSCTRL_PCLKSR_DPLLLCKF_Pos)) /* Assigment of value for DPLLLCKF in the SYSCTRL_PCLKSR register */
#define SYSCTRL_PCLKSR_DPLLLTO_Pos            _UINT32_(17)                                         /* (SYSCTRL_PCLKSR) DPLL Lock Timeout Position */
#define SYSCTRL_PCLKSR_DPLLLTO_Msk            (_UINT32_(0x1) << SYSCTRL_PCLKSR_DPLLLTO_Pos)        /* (SYSCTRL_PCLKSR) DPLL Lock Timeout Mask */
#define SYSCTRL_PCLKSR_DPLLLTO(value)         (SYSCTRL_PCLKSR_DPLLLTO_Msk & (_UINT32_(value) << SYSCTRL_PCLKSR_DPLLLTO_Pos)) /* Assigment of value for DPLLLTO in the SYSCTRL_PCLKSR register */
#define SYSCTRL_PCLKSR_Msk                    _UINT32_(0x00038FFF)                                 /* (SYSCTRL_PCLKSR) Register Mask  */


/* -------- SYSCTRL_XOSC : (SYSCTRL Offset: 0x10) (R/W 16) External Multipurpose Crystal Oscillator (XOSC) Control -------- */
#define SYSCTRL_XOSC_RESETVALUE               _UINT16_(0x80)                                       /*  (SYSCTRL_XOSC) External Multipurpose Crystal Oscillator (XOSC) Control  Reset Value */

#define SYSCTRL_XOSC_ENABLE_Pos               _UINT16_(1)                                          /* (SYSCTRL_XOSC) Oscillator Enable Position */
#define SYSCTRL_XOSC_ENABLE_Msk               (_UINT16_(0x1) << SYSCTRL_XOSC_ENABLE_Pos)           /* (SYSCTRL_XOSC) Oscillator Enable Mask */
#define SYSCTRL_XOSC_ENABLE(value)            (SYSCTRL_XOSC_ENABLE_Msk & (_UINT16_(value) << SYSCTRL_XOSC_ENABLE_Pos)) /* Assigment of value for ENABLE in the SYSCTRL_XOSC register */
#define SYSCTRL_XOSC_XTALEN_Pos               _UINT16_(2)                                          /* (SYSCTRL_XOSC) Crystal Oscillator Enable Position */
#define SYSCTRL_XOSC_XTALEN_Msk               (_UINT16_(0x1) << SYSCTRL_XOSC_XTALEN_Pos)           /* (SYSCTRL_XOSC) Crystal Oscillator Enable Mask */
#define SYSCTRL_XOSC_XTALEN(value)            (SYSCTRL_XOSC_XTALEN_Msk & (_UINT16_(value) << SYSCTRL_XOSC_XTALEN_Pos)) /* Assigment of value for XTALEN in the SYSCTRL_XOSC register */
#define SYSCTRL_XOSC_RUNSTDBY_Pos             _UINT16_(6)                                          /* (SYSCTRL_XOSC) Run in Standby Position */
#define SYSCTRL_XOSC_RUNSTDBY_Msk             (_UINT16_(0x1) << SYSCTRL_XOSC_RUNSTDBY_Pos)         /* (SYSCTRL_XOSC) Run in Standby Mask */
#define SYSCTRL_XOSC_RUNSTDBY(value)          (SYSCTRL_XOSC_RUNSTDBY_Msk & (_UINT16_(value) << SYSCTRL_XOSC_RUNSTDBY_Pos)) /* Assigment of value for RUNSTDBY in the SYSCTRL_XOSC register */
#define SYSCTRL_XOSC_ONDEMAND_Pos             _UINT16_(7)                                          /* (SYSCTRL_XOSC) On Demand Control Position */
#define SYSCTRL_XOSC_ONDEMAND_Msk             (_UINT16_(0x1) << SYSCTRL_XOSC_ONDEMAND_Pos)         /* (SYSCTRL_XOSC) On Demand Control Mask */
#define SYSCTRL_XOSC_ONDEMAND(value)          (SYSCTRL_XOSC_ONDEMAND_Msk & (_UINT16_(value) << SYSCTRL_XOSC_ONDEMAND_Pos)) /* Assigment of value for ONDEMAND in the SYSCTRL_XOSC register */
#define SYSCTRL_XOSC_GAIN_Pos                 _UINT16_(8)                                          /* (SYSCTRL_XOSC) Oscillator Gain Position */
#define SYSCTRL_XOSC_GAIN_Msk                 (_UINT16_(0x7) << SYSCTRL_XOSC_GAIN_Pos)             /* (SYSCTRL_XOSC) Oscillator Gain Mask */
#define SYSCTRL_XOSC_GAIN(value)              (SYSCTRL_XOSC_GAIN_Msk & (_UINT16_(value) << SYSCTRL_XOSC_GAIN_Pos)) /* Assigment of value for GAIN in the SYSCTRL_XOSC register */
#define   SYSCTRL_XOSC_GAIN_0_Val             _UINT16_(0x0)                                        /* (SYSCTRL_XOSC) 2MHz  */
#define   SYSCTRL_XOSC_GAIN_1_Val             _UINT16_(0x1)                                        /* (SYSCTRL_XOSC) 4MHz  */
#define   SYSCTRL_XOSC_GAIN_2_Val             _UINT16_(0x2)                                        /* (SYSCTRL_XOSC) 8MHz  */
#define   SYSCTRL_XOSC_GAIN_3_Val             _UINT16_(0x3)                                        /* (SYSCTRL_XOSC) 16MHz  */
#define   SYSCTRL_XOSC_GAIN_4_Val             _UINT16_(0x4)                                        /* (SYSCTRL_XOSC) 30MHz  */
#define SYSCTRL_XOSC_GAIN_0                   (SYSCTRL_XOSC_GAIN_0_Val << SYSCTRL_XOSC_GAIN_Pos)   /* (SYSCTRL_XOSC) 2MHz Position  */
#define SYSCTRL_XOSC_GAIN_1                   (SYSCTRL_XOSC_GAIN_1_Val << SYSCTRL_XOSC_GAIN_Pos)   /* (SYSCTRL_XOSC) 4MHz Position  */
#define SYSCTRL_XOSC_GAIN_2                   (SYSCTRL_XOSC_GAIN_2_Val << SYSCTRL_XOSC_GAIN_Pos)   /* (SYSCTRL_XOSC) 8MHz Position  */
#define SYSCTRL_XOSC_GAIN_3                   (SYSCTRL_XOSC_GAIN_3_Val << SYSCTRL_XOSC_GAIN_Pos)   /* (SYSCTRL_XOSC) 16MHz Position  */
#define SYSCTRL_XOSC_GAIN_4                   (SYSCTRL_XOSC_GAIN_4_Val << SYSCTRL_XOSC_GAIN_Pos)   /* (SYSCTRL_XOSC) 30MHz Position  */
#define SYSCTRL_XOSC_AMPGC_Pos                _UINT16_(11)                                         /* (SYSCTRL_XOSC) Automatic Amplitude Gain Control Position */
#define SYSCTRL_XOSC_AMPGC_Msk                (_UINT16_(0x1) << SYSCTRL_XOSC_AMPGC_Pos)            /* (SYSCTRL_XOSC) Automatic Amplitude Gain Control Mask */
#define SYSCTRL_XOSC_AMPGC(value)             (SYSCTRL_XOSC_AMPGC_Msk & (_UINT16_(value) << SYSCTRL_XOSC_AMPGC_Pos)) /* Assigment of value for AMPGC in the SYSCTRL_XOSC register */
#define SYSCTRL_XOSC_STARTUP_Pos              _UINT16_(12)                                         /* (SYSCTRL_XOSC) Start-Up Time Position */
#define SYSCTRL_XOSC_STARTUP_Msk              (_UINT16_(0xF) << SYSCTRL_XOSC_STARTUP_Pos)          /* (SYSCTRL_XOSC) Start-Up Time Mask */
#define SYSCTRL_XOSC_STARTUP(value)           (SYSCTRL_XOSC_STARTUP_Msk & (_UINT16_(value) << SYSCTRL_XOSC_STARTUP_Pos)) /* Assigment of value for STARTUP in the SYSCTRL_XOSC register */
#define   SYSCTRL_XOSC_STARTUP_CYCLE1_Val     _UINT16_(0x0)                                        /* (SYSCTRL_XOSC) 31 us  */
#define   SYSCTRL_XOSC_STARTUP_CYCLE2_Val     _UINT16_(0x1)                                        /* (SYSCTRL_XOSC) 61 us  */
#define   SYSCTRL_XOSC_STARTUP_CYCLE4_Val     _UINT16_(0x2)                                        /* (SYSCTRL_XOSC) 122 us  */
#define   SYSCTRL_XOSC_STARTUP_CYCLE8_Val     _UINT16_(0x3)                                        /* (SYSCTRL_XOSC) 244 us  */
#define   SYSCTRL_XOSC_STARTUP_CYCLE16_Val    _UINT16_(0x4)                                        /* (SYSCTRL_XOSC) 488 us  */
#define   SYSCTRL_XOSC_STARTUP_CYCLE32_Val    _UINT16_(0x5)                                        /* (SYSCTRL_XOSC) 977 us  */
#define   SYSCTRL_XOSC_STARTUP_CYCLE64_Val    _UINT16_(0x6)                                        /* (SYSCTRL_XOSC) 1953 us  */
#define   SYSCTRL_XOSC_STARTUP_CYCLE128_Val   _UINT16_(0x7)                                        /* (SYSCTRL_XOSC) 3906 us  */
#define   SYSCTRL_XOSC_STARTUP_CYCLE256_Val   _UINT16_(0x8)                                        /* (SYSCTRL_XOSC) 7813 us  */
#define   SYSCTRL_XOSC_STARTUP_CYCLE512_Val   _UINT16_(0x9)                                        /* (SYSCTRL_XOSC) 15625 us  */
#define   SYSCTRL_XOSC_STARTUP_CYCLE1024_Val  _UINT16_(0xA)                                        /* (SYSCTRL_XOSC) 31250 us  */
#define   SYSCTRL_XOSC_STARTUP_CYCLE2048_Val  _UINT16_(0xB)                                        /* (SYSCTRL_XOSC) 62500 us  */
#define   SYSCTRL_XOSC_STARTUP_CYCLE4096_Val  _UINT16_(0xC)                                        /* (SYSCTRL_XOSC) 125000 us  */
#define   SYSCTRL_XOSC_STARTUP_CYCLE8192_Val  _UINT16_(0xD)                                        /* (SYSCTRL_XOSC) 250000 us  */
#define   SYSCTRL_XOSC_STARTUP_CYCLE16384_Val _UINT16_(0xE)                                        /* (SYSCTRL_XOSC) 500000 us  */
#define   SYSCTRL_XOSC_STARTUP_CYCLE32768_Val _UINT16_(0xF)                                        /* (SYSCTRL_XOSC) 1000000 us  */
#define SYSCTRL_XOSC_STARTUP_CYCLE1           (SYSCTRL_XOSC_STARTUP_CYCLE1_Val << SYSCTRL_XOSC_STARTUP_Pos) /* (SYSCTRL_XOSC) 31 us Position  */
#define SYSCTRL_XOSC_STARTUP_CYCLE2           (SYSCTRL_XOSC_STARTUP_CYCLE2_Val << SYSCTRL_XOSC_STARTUP_Pos) /* (SYSCTRL_XOSC) 61 us Position  */
#define SYSCTRL_XOSC_STARTUP_CYCLE4           (SYSCTRL_XOSC_STARTUP_CYCLE4_Val << SYSCTRL_XOSC_STARTUP_Pos) /* (SYSCTRL_XOSC) 122 us Position  */
#define SYSCTRL_XOSC_STARTUP_CYCLE8           (SYSCTRL_XOSC_STARTUP_CYCLE8_Val << SYSCTRL_XOSC_STARTUP_Pos) /* (SYSCTRL_XOSC) 244 us Position  */
#define SYSCTRL_XOSC_STARTUP_CYCLE16          (SYSCTRL_XOSC_STARTUP_CYCLE16_Val << SYSCTRL_XOSC_STARTUP_Pos) /* (SYSCTRL_XOSC) 488 us Position  */
#define SYSCTRL_XOSC_STARTUP_CYCLE32          (SYSCTRL_XOSC_STARTUP_CYCLE32_Val << SYSCTRL_XOSC_STARTUP_Pos) /* (SYSCTRL_XOSC) 977 us Position  */
#define SYSCTRL_XOSC_STARTUP_CYCLE64          (SYSCTRL_XOSC_STARTUP_CYCLE64_Val << SYSCTRL_XOSC_STARTUP_Pos) /* (SYSCTRL_XOSC) 1953 us Position  */
#define SYSCTRL_XOSC_STARTUP_CYCLE128         (SYSCTRL_XOSC_STARTUP_CYCLE128_Val << SYSCTRL_XOSC_STARTUP_Pos) /* (SYSCTRL_XOSC) 3906 us Position  */
#define SYSCTRL_XOSC_STARTUP_CYCLE256         (SYSCTRL_XOSC_STARTUP_CYCLE256_Val << SYSCTRL_XOSC_STARTUP_Pos) /* (SYSCTRL_XOSC) 7813 us Position  */
#define SYSCTRL_XOSC_STARTUP_CYCLE512         (SYSCTRL_XOSC_STARTUP_CYCLE512_Val << SYSCTRL_XOSC_STARTUP_Pos) /* (SYSCTRL_XOSC) 15625 us Position  */
#define SYSCTRL_XOSC_STARTUP_CYCLE1024        (SYSCTRL_XOSC_STARTUP_CYCLE1024_Val << SYSCTRL_XOSC_STARTUP_Pos) /* (SYSCTRL_XOSC) 31250 us Position  */
#define SYSCTRL_XOSC_STARTUP_CYCLE2048        (SYSCTRL_XOSC_STARTUP_CYCLE2048_Val << SYSCTRL_XOSC_STARTUP_Pos) /* (SYSCTRL_XOSC) 62500 us Position  */
#define SYSCTRL_XOSC_STARTUP_CYCLE4096        (SYSCTRL_XOSC_STARTUP_CYCLE4096_Val << SYSCTRL_XOSC_STARTUP_Pos) /* (SYSCTRL_XOSC) 125000 us Position  */
#define SYSCTRL_XOSC_STARTUP_CYCLE8192        (SYSCTRL_XOSC_STARTUP_CYCLE8192_Val << SYSCTRL_XOSC_STARTUP_Pos) /* (SYSCTRL_XOSC) 250000 us Position  */
#define SYSCTRL_XOSC_STARTUP_CYCLE16384       (SYSCTRL_XOSC_STARTUP_CYCLE16384_Val << SYSCTRL_XOSC_STARTUP_Pos) /* (SYSCTRL_XOSC) 500000 us Position  */
#define SYSCTRL_XOSC_STARTUP_CYCLE32768       (SYSCTRL_XOSC_STARTUP_CYCLE32768_Val << SYSCTRL_XOSC_STARTUP_Pos) /* (SYSCTRL_XOSC) 1000000 us Position  */
#define SYSCTRL_XOSC_Msk                      _UINT16_(0xFFC6)                                     /* (SYSCTRL_XOSC) Register Mask  */


/* -------- SYSCTRL_XOSC32K : (SYSCTRL Offset: 0x14) (R/W 16) 32kHz External Crystal Oscillator (XOSC32K) Control -------- */
#define SYSCTRL_XOSC32K_RESETVALUE            _UINT16_(0x80)                                       /*  (SYSCTRL_XOSC32K) 32kHz External Crystal Oscillator (XOSC32K) Control  Reset Value */

#define SYSCTRL_XOSC32K_ENABLE_Pos            _UINT16_(1)                                          /* (SYSCTRL_XOSC32K) Oscillator Enable Position */
#define SYSCTRL_XOSC32K_ENABLE_Msk            (_UINT16_(0x1) << SYSCTRL_XOSC32K_ENABLE_Pos)        /* (SYSCTRL_XOSC32K) Oscillator Enable Mask */
#define SYSCTRL_XOSC32K_ENABLE(value)         (SYSCTRL_XOSC32K_ENABLE_Msk & (_UINT16_(value) << SYSCTRL_XOSC32K_ENABLE_Pos)) /* Assigment of value for ENABLE in the SYSCTRL_XOSC32K register */
#define SYSCTRL_XOSC32K_XTALEN_Pos            _UINT16_(2)                                          /* (SYSCTRL_XOSC32K) Crystal Oscillator Enable Position */
#define SYSCTRL_XOSC32K_XTALEN_Msk            (_UINT16_(0x1) << SYSCTRL_XOSC32K_XTALEN_Pos)        /* (SYSCTRL_XOSC32K) Crystal Oscillator Enable Mask */
#define SYSCTRL_XOSC32K_XTALEN(value)         (SYSCTRL_XOSC32K_XTALEN_Msk & (_UINT16_(value) << SYSCTRL_XOSC32K_XTALEN_Pos)) /* Assigment of value for XTALEN in the SYSCTRL_XOSC32K register */
#define SYSCTRL_XOSC32K_EN32K_Pos             _UINT16_(3)                                          /* (SYSCTRL_XOSC32K) 32kHz Output Enable Position */
#define SYSCTRL_XOSC32K_EN32K_Msk             (_UINT16_(0x1) << SYSCTRL_XOSC32K_EN32K_Pos)         /* (SYSCTRL_XOSC32K) 32kHz Output Enable Mask */
#define SYSCTRL_XOSC32K_EN32K(value)          (SYSCTRL_XOSC32K_EN32K_Msk & (_UINT16_(value) << SYSCTRL_XOSC32K_EN32K_Pos)) /* Assigment of value for EN32K in the SYSCTRL_XOSC32K register */
#define SYSCTRL_XOSC32K_EN1K_Pos              _UINT16_(4)                                          /* (SYSCTRL_XOSC32K) 1kHz Output Enable Position */
#define SYSCTRL_XOSC32K_EN1K_Msk              (_UINT16_(0x1) << SYSCTRL_XOSC32K_EN1K_Pos)          /* (SYSCTRL_XOSC32K) 1kHz Output Enable Mask */
#define SYSCTRL_XOSC32K_EN1K(value)           (SYSCTRL_XOSC32K_EN1K_Msk & (_UINT16_(value) << SYSCTRL_XOSC32K_EN1K_Pos)) /* Assigment of value for EN1K in the SYSCTRL_XOSC32K register */
#define SYSCTRL_XOSC32K_AAMPEN_Pos            _UINT16_(5)                                          /* (SYSCTRL_XOSC32K) Automatic Amplitude Control Enable Position */
#define SYSCTRL_XOSC32K_AAMPEN_Msk            (_UINT16_(0x1) << SYSCTRL_XOSC32K_AAMPEN_Pos)        /* (SYSCTRL_XOSC32K) Automatic Amplitude Control Enable Mask */
#define SYSCTRL_XOSC32K_AAMPEN(value)         (SYSCTRL_XOSC32K_AAMPEN_Msk & (_UINT16_(value) << SYSCTRL_XOSC32K_AAMPEN_Pos)) /* Assigment of value for AAMPEN in the SYSCTRL_XOSC32K register */
#define SYSCTRL_XOSC32K_RUNSTDBY_Pos          _UINT16_(6)                                          /* (SYSCTRL_XOSC32K) Run in Standby Position */
#define SYSCTRL_XOSC32K_RUNSTDBY_Msk          (_UINT16_(0x1) << SYSCTRL_XOSC32K_RUNSTDBY_Pos)      /* (SYSCTRL_XOSC32K) Run in Standby Mask */
#define SYSCTRL_XOSC32K_RUNSTDBY(value)       (SYSCTRL_XOSC32K_RUNSTDBY_Msk & (_UINT16_(value) << SYSCTRL_XOSC32K_RUNSTDBY_Pos)) /* Assigment of value for RUNSTDBY in the SYSCTRL_XOSC32K register */
#define SYSCTRL_XOSC32K_ONDEMAND_Pos          _UINT16_(7)                                          /* (SYSCTRL_XOSC32K) On Demand Control Position */
#define SYSCTRL_XOSC32K_ONDEMAND_Msk          (_UINT16_(0x1) << SYSCTRL_XOSC32K_ONDEMAND_Pos)      /* (SYSCTRL_XOSC32K) On Demand Control Mask */
#define SYSCTRL_XOSC32K_ONDEMAND(value)       (SYSCTRL_XOSC32K_ONDEMAND_Msk & (_UINT16_(value) << SYSCTRL_XOSC32K_ONDEMAND_Pos)) /* Assigment of value for ONDEMAND in the SYSCTRL_XOSC32K register */
#define SYSCTRL_XOSC32K_STARTUP_Pos           _UINT16_(8)                                          /* (SYSCTRL_XOSC32K) Oscillator Start-Up Time Position */
#define SYSCTRL_XOSC32K_STARTUP_Msk           (_UINT16_(0x7) << SYSCTRL_XOSC32K_STARTUP_Pos)       /* (SYSCTRL_XOSC32K) Oscillator Start-Up Time Mask */
#define SYSCTRL_XOSC32K_STARTUP(value)        (SYSCTRL_XOSC32K_STARTUP_Msk & (_UINT16_(value) << SYSCTRL_XOSC32K_STARTUP_Pos)) /* Assigment of value for STARTUP in the SYSCTRL_XOSC32K register */
#define   SYSCTRL_XOSC32K_STARTUP_CYCLE1_Val  _UINT16_(0x0)                                        /* (SYSCTRL_XOSC32K) 0.122 ms  */
#define   SYSCTRL_XOSC32K_STARTUP_CYCLE32_Val _UINT16_(0x1)                                        /* (SYSCTRL_XOSC32K) 1.068 ms  */
#define   SYSCTRL_XOSC32K_STARTUP_CYCLE2048_Val _UINT16_(0x2)                                        /* (SYSCTRL_XOSC32K) 62.592 ms  */
#define   SYSCTRL_XOSC32K_STARTUP_CYCLE4096_Val _UINT16_(0x3)                                        /* (SYSCTRL_XOSC32K) 125.092 ms  */
#define   SYSCTRL_XOSC32K_STARTUP_CYCLE16384_Val _UINT16_(0x4)                                        /* (SYSCTRL_XOSC32K) 500.092 ms  */
#define   SYSCTRL_XOSC32K_STARTUP_CYCLE32768_Val _UINT16_(0x5)                                        /* (SYSCTRL_XOSC32K) 1000.092 ms  */
#define   SYSCTRL_XOSC32K_STARTUP_CYCLE65536_Val _UINT16_(0x6)                                        /* (SYSCTRL_XOSC32K) 2000.092 ms  */
#define   SYSCTRL_XOSC32K_STARTUP_CYCLE131072_Val _UINT16_(0x7)                                        /* (SYSCTRL_XOSC32K) 4000.092 ms  */
#define SYSCTRL_XOSC32K_STARTUP_CYCLE1        (SYSCTRL_XOSC32K_STARTUP_CYCLE1_Val << SYSCTRL_XOSC32K_STARTUP_Pos) /* (SYSCTRL_XOSC32K) 0.122 ms Position  */
#define SYSCTRL_XOSC32K_STARTUP_CYCLE32       (SYSCTRL_XOSC32K_STARTUP_CYCLE32_Val << SYSCTRL_XOSC32K_STARTUP_Pos) /* (SYSCTRL_XOSC32K) 1.068 ms Position  */
#define SYSCTRL_XOSC32K_STARTUP_CYCLE2048     (SYSCTRL_XOSC32K_STARTUP_CYCLE2048_Val << SYSCTRL_XOSC32K_STARTUP_Pos) /* (SYSCTRL_XOSC32K) 62.592 ms Position  */
#define SYSCTRL_XOSC32K_STARTUP_CYCLE4096     (SYSCTRL_XOSC32K_STARTUP_CYCLE4096_Val << SYSCTRL_XOSC32K_STARTUP_Pos) /* (SYSCTRL_XOSC32K) 125.092 ms Position  */
#define SYSCTRL_XOSC32K_STARTUP_CYCLE16384    (SYSCTRL_XOSC32K_STARTUP_CYCLE16384_Val << SYSCTRL_XOSC32K_STARTUP_Pos) /* (SYSCTRL_XOSC32K) 500.092 ms Position  */
#define SYSCTRL_XOSC32K_STARTUP_CYCLE32768    (SYSCTRL_XOSC32K_STARTUP_CYCLE32768_Val << SYSCTRL_XOSC32K_STARTUP_Pos) /* (SYSCTRL_XOSC32K) 1000.092 ms Position  */
#define SYSCTRL_XOSC32K_STARTUP_CYCLE65536    (SYSCTRL_XOSC32K_STARTUP_CYCLE65536_Val << SYSCTRL_XOSC32K_STARTUP_Pos) /* (SYSCTRL_XOSC32K) 2000.092 ms Position  */
#define SYSCTRL_XOSC32K_STARTUP_CYCLE131072   (SYSCTRL_XOSC32K_STARTUP_CYCLE131072_Val << SYSCTRL_XOSC32K_STARTUP_Pos) /* (SYSCTRL_XOSC32K) 4000.092 ms Position  */
#define SYSCTRL_XOSC32K_WRTLOCK_Pos           _UINT16_(12)                                         /* (SYSCTRL_XOSC32K) Write Lock Position */
#define SYSCTRL_XOSC32K_WRTLOCK_Msk           (_UINT16_(0x1) << SYSCTRL_XOSC32K_WRTLOCK_Pos)       /* (SYSCTRL_XOSC32K) Write Lock Mask */
#define SYSCTRL_XOSC32K_WRTLOCK(value)        (SYSCTRL_XOSC32K_WRTLOCK_Msk & (_UINT16_(value) << SYSCTRL_XOSC32K_WRTLOCK_Pos)) /* Assigment of value for WRTLOCK in the SYSCTRL_XOSC32K register */
#define SYSCTRL_XOSC32K_Msk                   _UINT16_(0x17FE)                                     /* (SYSCTRL_XOSC32K) Register Mask  */


/* -------- SYSCTRL_OSC32K : (SYSCTRL Offset: 0x18) (R/W 32) 32kHz Internal Oscillator (OSC32K) Control -------- */
#define SYSCTRL_OSC32K_RESETVALUE             _UINT32_(0x3F0080)                                   /*  (SYSCTRL_OSC32K) 32kHz Internal Oscillator (OSC32K) Control  Reset Value */

#define SYSCTRL_OSC32K_ENABLE_Pos             _UINT32_(1)                                          /* (SYSCTRL_OSC32K) Oscillator Enable Position */
#define SYSCTRL_OSC32K_ENABLE_Msk             (_UINT32_(0x1) << SYSCTRL_OSC32K_ENABLE_Pos)         /* (SYSCTRL_OSC32K) Oscillator Enable Mask */
#define SYSCTRL_OSC32K_ENABLE(value)          (SYSCTRL_OSC32K_ENABLE_Msk & (_UINT32_(value) << SYSCTRL_OSC32K_ENABLE_Pos)) /* Assigment of value for ENABLE in the SYSCTRL_OSC32K register */
#define SYSCTRL_OSC32K_EN32K_Pos              _UINT32_(2)                                          /* (SYSCTRL_OSC32K) 32kHz Output Enable Position */
#define SYSCTRL_OSC32K_EN32K_Msk              (_UINT32_(0x1) << SYSCTRL_OSC32K_EN32K_Pos)          /* (SYSCTRL_OSC32K) 32kHz Output Enable Mask */
#define SYSCTRL_OSC32K_EN32K(value)           (SYSCTRL_OSC32K_EN32K_Msk & (_UINT32_(value) << SYSCTRL_OSC32K_EN32K_Pos)) /* Assigment of value for EN32K in the SYSCTRL_OSC32K register */
#define SYSCTRL_OSC32K_EN1K_Pos               _UINT32_(3)                                          /* (SYSCTRL_OSC32K) 1kHz Output Enable Position */
#define SYSCTRL_OSC32K_EN1K_Msk               (_UINT32_(0x1) << SYSCTRL_OSC32K_EN1K_Pos)           /* (SYSCTRL_OSC32K) 1kHz Output Enable Mask */
#define SYSCTRL_OSC32K_EN1K(value)            (SYSCTRL_OSC32K_EN1K_Msk & (_UINT32_(value) << SYSCTRL_OSC32K_EN1K_Pos)) /* Assigment of value for EN1K in the SYSCTRL_OSC32K register */
#define SYSCTRL_OSC32K_RUNSTDBY_Pos           _UINT32_(6)                                          /* (SYSCTRL_OSC32K) Run in Standby Position */
#define SYSCTRL_OSC32K_RUNSTDBY_Msk           (_UINT32_(0x1) << SYSCTRL_OSC32K_RUNSTDBY_Pos)       /* (SYSCTRL_OSC32K) Run in Standby Mask */
#define SYSCTRL_OSC32K_RUNSTDBY(value)        (SYSCTRL_OSC32K_RUNSTDBY_Msk & (_UINT32_(value) << SYSCTRL_OSC32K_RUNSTDBY_Pos)) /* Assigment of value for RUNSTDBY in the SYSCTRL_OSC32K register */
#define SYSCTRL_OSC32K_ONDEMAND_Pos           _UINT32_(7)                                          /* (SYSCTRL_OSC32K) On Demand Control Position */
#define SYSCTRL_OSC32K_ONDEMAND_Msk           (_UINT32_(0x1) << SYSCTRL_OSC32K_ONDEMAND_Pos)       /* (SYSCTRL_OSC32K) On Demand Control Mask */
#define SYSCTRL_OSC32K_ONDEMAND(value)        (SYSCTRL_OSC32K_ONDEMAND_Msk & (_UINT32_(value) << SYSCTRL_OSC32K_ONDEMAND_Pos)) /* Assigment of value for ONDEMAND in the SYSCTRL_OSC32K register */
#define SYSCTRL_OSC32K_STARTUP_Pos            _UINT32_(8)                                          /* (SYSCTRL_OSC32K) Oscillator Start-Up Time Position */
#define SYSCTRL_OSC32K_STARTUP_Msk            (_UINT32_(0x7) << SYSCTRL_OSC32K_STARTUP_Pos)        /* (SYSCTRL_OSC32K) Oscillator Start-Up Time Mask */
#define SYSCTRL_OSC32K_STARTUP(value)         (SYSCTRL_OSC32K_STARTUP_Msk & (_UINT32_(value) << SYSCTRL_OSC32K_STARTUP_Pos)) /* Assigment of value for STARTUP in the SYSCTRL_OSC32K register */
#define   SYSCTRL_OSC32K_STARTUP_CYCLE3_Val   _UINT32_(0x0)                                        /* (SYSCTRL_OSC32K) 0.092 ms  */
#define   SYSCTRL_OSC32K_STARTUP_CYCLE4_Val   _UINT32_(0x1)                                        /* (SYSCTRL_OSC32K) 0.122 ms  */
#define   SYSCTRL_OSC32K_STARTUP_CYCLE6_Val   _UINT32_(0x2)                                        /* (SYSCTRL_OSC32K) 0.183 ms  */
#define   SYSCTRL_OSC32K_STARTUP_CYCLE10_Val  _UINT32_(0x3)                                        /* (SYSCTRL_OSC32K) 0.305 ms  */
#define   SYSCTRL_OSC32K_STARTUP_CYCLE18_Val  _UINT32_(0x4)                                        /* (SYSCTRL_OSC32K) 0.549 ms  */
#define   SYSCTRL_OSC32K_STARTUP_CYCLE34_Val  _UINT32_(0x5)                                        /* (SYSCTRL_OSC32K) 1.038 ms  */
#define   SYSCTRL_OSC32K_STARTUP_CYCLE66_Val  _UINT32_(0x6)                                        /* (SYSCTRL_OSC32K) 2.014 ms  */
#define   SYSCTRL_OSC32K_STARTUP_CYCLE130_Val _UINT32_(0x7)                                        /* (SYSCTRL_OSC32K) 3.967 ms  */
#define SYSCTRL_OSC32K_STARTUP_CYCLE3         (SYSCTRL_OSC32K_STARTUP_CYCLE3_Val << SYSCTRL_OSC32K_STARTUP_Pos) /* (SYSCTRL_OSC32K) 0.092 ms Position  */
#define SYSCTRL_OSC32K_STARTUP_CYCLE4         (SYSCTRL_OSC32K_STARTUP_CYCLE4_Val << SYSCTRL_OSC32K_STARTUP_Pos) /* (SYSCTRL_OSC32K) 0.122 ms Position  */
#define SYSCTRL_OSC32K_STARTUP_CYCLE6         (SYSCTRL_OSC32K_STARTUP_CYCLE6_Val << SYSCTRL_OSC32K_STARTUP_Pos) /* (SYSCTRL_OSC32K) 0.183 ms Position  */
#define SYSCTRL_OSC32K_STARTUP_CYCLE10        (SYSCTRL_OSC32K_STARTUP_CYCLE10_Val << SYSCTRL_OSC32K_STARTUP_Pos) /* (SYSCTRL_OSC32K) 0.305 ms Position  */
#define SYSCTRL_OSC32K_STARTUP_CYCLE18        (SYSCTRL_OSC32K_STARTUP_CYCLE18_Val << SYSCTRL_OSC32K_STARTUP_Pos) /* (SYSCTRL_OSC32K) 0.549 ms Position  */
#define SYSCTRL_OSC32K_STARTUP_CYCLE34        (SYSCTRL_OSC32K_STARTUP_CYCLE34_Val << SYSCTRL_OSC32K_STARTUP_Pos) /* (SYSCTRL_OSC32K) 1.038 ms Position  */
#define SYSCTRL_OSC32K_STARTUP_CYCLE66        (SYSCTRL_OSC32K_STARTUP_CYCLE66_Val << SYSCTRL_OSC32K_STARTUP_Pos) /* (SYSCTRL_OSC32K) 2.014 ms Position  */
#define SYSCTRL_OSC32K_STARTUP_CYCLE130       (SYSCTRL_OSC32K_STARTUP_CYCLE130_Val << SYSCTRL_OSC32K_STARTUP_Pos) /* (SYSCTRL_OSC32K) 3.967 ms Position  */
#define SYSCTRL_OSC32K_WRTLOCK_Pos            _UINT32_(12)                                         /* (SYSCTRL_OSC32K) Write Lock Position */
#define SYSCTRL_OSC32K_WRTLOCK_Msk            (_UINT32_(0x1) << SYSCTRL_OSC32K_WRTLOCK_Pos)        /* (SYSCTRL_OSC32K) Write Lock Mask */
#define SYSCTRL_OSC32K_WRTLOCK(value)         (SYSCTRL_OSC32K_WRTLOCK_Msk & (_UINT32_(value) << SYSCTRL_OSC32K_WRTLOCK_Pos)) /* Assigment of value for WRTLOCK in the SYSCTRL_OSC32K register */
#define SYSCTRL_OSC32K_CALIB_Pos              _UINT32_(16)                                         /* (SYSCTRL_OSC32K) Oscillator Calibration Position */
#define SYSCTRL_OSC32K_CALIB_Msk              (_UINT32_(0x7F) << SYSCTRL_OSC32K_CALIB_Pos)         /* (SYSCTRL_OSC32K) Oscillator Calibration Mask */
#define SYSCTRL_OSC32K_CALIB(value)           (SYSCTRL_OSC32K_CALIB_Msk & (_UINT32_(value) << SYSCTRL_OSC32K_CALIB_Pos)) /* Assigment of value for CALIB in the SYSCTRL_OSC32K register */
#define SYSCTRL_OSC32K_Msk                    _UINT32_(0x007F17CE)                                 /* (SYSCTRL_OSC32K) Register Mask  */


/* -------- SYSCTRL_OSCULP32K : (SYSCTRL Offset: 0x1C) (R/W 8) 32kHz Ultra Low Power Internal Oscillator (OSCULP32K) Control -------- */
#define SYSCTRL_OSCULP32K_RESETVALUE          _UINT8_(0x1F)                                        /*  (SYSCTRL_OSCULP32K) 32kHz Ultra Low Power Internal Oscillator (OSCULP32K) Control  Reset Value */

#define SYSCTRL_OSCULP32K_CALIB_Pos           _UINT8_(0)                                           /* (SYSCTRL_OSCULP32K) Oscillator Calibration Position */
#define SYSCTRL_OSCULP32K_CALIB_Msk           (_UINT8_(0x1F) << SYSCTRL_OSCULP32K_CALIB_Pos)       /* (SYSCTRL_OSCULP32K) Oscillator Calibration Mask */
#define SYSCTRL_OSCULP32K_CALIB(value)        (SYSCTRL_OSCULP32K_CALIB_Msk & (_UINT8_(value) << SYSCTRL_OSCULP32K_CALIB_Pos)) /* Assigment of value for CALIB in the SYSCTRL_OSCULP32K register */
#define SYSCTRL_OSCULP32K_WRTLOCK_Pos         _UINT8_(7)                                           /* (SYSCTRL_OSCULP32K) Write Lock Position */
#define SYSCTRL_OSCULP32K_WRTLOCK_Msk         (_UINT8_(0x1) << SYSCTRL_OSCULP32K_WRTLOCK_Pos)      /* (SYSCTRL_OSCULP32K) Write Lock Mask */
#define SYSCTRL_OSCULP32K_WRTLOCK(value)      (SYSCTRL_OSCULP32K_WRTLOCK_Msk & (_UINT8_(value) << SYSCTRL_OSCULP32K_WRTLOCK_Pos)) /* Assigment of value for WRTLOCK in the SYSCTRL_OSCULP32K register */
#define SYSCTRL_OSCULP32K_Msk                 _UINT8_(0x9F)                                        /* (SYSCTRL_OSCULP32K) Register Mask  */


/* -------- SYSCTRL_OSC8M : (SYSCTRL Offset: 0x20) (R/W 32) 8MHz Internal Oscillator (OSC8M) Control -------- */
#define SYSCTRL_OSC8M_RESETVALUE              _UINT32_(0x87070382)                                 /*  (SYSCTRL_OSC8M) 8MHz Internal Oscillator (OSC8M) Control  Reset Value */

#define SYSCTRL_OSC8M_ENABLE_Pos              _UINT32_(1)                                          /* (SYSCTRL_OSC8M) Oscillator Enable Position */
#define SYSCTRL_OSC8M_ENABLE_Msk              (_UINT32_(0x1) << SYSCTRL_OSC8M_ENABLE_Pos)          /* (SYSCTRL_OSC8M) Oscillator Enable Mask */
#define SYSCTRL_OSC8M_ENABLE(value)           (SYSCTRL_OSC8M_ENABLE_Msk & (_UINT32_(value) << SYSCTRL_OSC8M_ENABLE_Pos)) /* Assigment of value for ENABLE in the SYSCTRL_OSC8M register */
#define SYSCTRL_OSC8M_RUNSTDBY_Pos            _UINT32_(6)                                          /* (SYSCTRL_OSC8M) Run in Standby Position */
#define SYSCTRL_OSC8M_RUNSTDBY_Msk            (_UINT32_(0x1) << SYSCTRL_OSC8M_RUNSTDBY_Pos)        /* (SYSCTRL_OSC8M) Run in Standby Mask */
#define SYSCTRL_OSC8M_RUNSTDBY(value)         (SYSCTRL_OSC8M_RUNSTDBY_Msk & (_UINT32_(value) << SYSCTRL_OSC8M_RUNSTDBY_Pos)) /* Assigment of value for RUNSTDBY in the SYSCTRL_OSC8M register */
#define SYSCTRL_OSC8M_ONDEMAND_Pos            _UINT32_(7)                                          /* (SYSCTRL_OSC8M) On Demand Control Position */
#define SYSCTRL_OSC8M_ONDEMAND_Msk            (_UINT32_(0x1) << SYSCTRL_OSC8M_ONDEMAND_Pos)        /* (SYSCTRL_OSC8M) On Demand Control Mask */
#define SYSCTRL_OSC8M_ONDEMAND(value)         (SYSCTRL_OSC8M_ONDEMAND_Msk & (_UINT32_(value) << SYSCTRL_OSC8M_ONDEMAND_Pos)) /* Assigment of value for ONDEMAND in the SYSCTRL_OSC8M register */
#define SYSCTRL_OSC8M_PRESC_Pos               _UINT32_(8)                                          /* (SYSCTRL_OSC8M) Oscillator Prescaler Position */
#define SYSCTRL_OSC8M_PRESC_Msk               (_UINT32_(0x3) << SYSCTRL_OSC8M_PRESC_Pos)           /* (SYSCTRL_OSC8M) Oscillator Prescaler Mask */
#define SYSCTRL_OSC8M_PRESC(value)            (SYSCTRL_OSC8M_PRESC_Msk & (_UINT32_(value) << SYSCTRL_OSC8M_PRESC_Pos)) /* Assigment of value for PRESC in the SYSCTRL_OSC8M register */
#define   SYSCTRL_OSC8M_PRESC_0_Val           _UINT32_(0x0)                                        /* (SYSCTRL_OSC8M) 1  */
#define   SYSCTRL_OSC8M_PRESC_1_Val           _UINT32_(0x1)                                        /* (SYSCTRL_OSC8M) 2  */
#define   SYSCTRL_OSC8M_PRESC_2_Val           _UINT32_(0x2)                                        /* (SYSCTRL_OSC8M) 4  */
#define   SYSCTRL_OSC8M_PRESC_3_Val           _UINT32_(0x3)                                        /* (SYSCTRL_OSC8M) 8  */
#define SYSCTRL_OSC8M_PRESC_0                 (SYSCTRL_OSC8M_PRESC_0_Val << SYSCTRL_OSC8M_PRESC_Pos) /* (SYSCTRL_OSC8M) 1 Position  */
#define SYSCTRL_OSC8M_PRESC_1                 (SYSCTRL_OSC8M_PRESC_1_Val << SYSCTRL_OSC8M_PRESC_Pos) /* (SYSCTRL_OSC8M) 2 Position  */
#define SYSCTRL_OSC8M_PRESC_2                 (SYSCTRL_OSC8M_PRESC_2_Val << SYSCTRL_OSC8M_PRESC_Pos) /* (SYSCTRL_OSC8M) 4 Position  */
#define SYSCTRL_OSC8M_PRESC_3                 (SYSCTRL_OSC8M_PRESC_3_Val << SYSCTRL_OSC8M_PRESC_Pos) /* (SYSCTRL_OSC8M) 8 Position  */
#define SYSCTRL_OSC8M_CALIB_Pos               _UINT32_(16)                                         /* (SYSCTRL_OSC8M) Oscillator Calibration Position */
#define SYSCTRL_OSC8M_CALIB_Msk               (_UINT32_(0xFFF) << SYSCTRL_OSC8M_CALIB_Pos)         /* (SYSCTRL_OSC8M) Oscillator Calibration Mask */
#define SYSCTRL_OSC8M_CALIB(value)            (SYSCTRL_OSC8M_CALIB_Msk & (_UINT32_(value) << SYSCTRL_OSC8M_CALIB_Pos)) /* Assigment of value for CALIB in the SYSCTRL_OSC8M register */
#define SYSCTRL_OSC8M_FRANGE_Pos              _UINT32_(30)                                         /* (SYSCTRL_OSC8M) Oscillator Frequency Range Position */
#define SYSCTRL_OSC8M_FRANGE_Msk              (_UINT32_(0x3) << SYSCTRL_OSC8M_FRANGE_Pos)          /* (SYSCTRL_OSC8M) Oscillator Frequency Range Mask */
#define SYSCTRL_OSC8M_FRANGE(value)           (SYSCTRL_OSC8M_FRANGE_Msk & (_UINT32_(value) << SYSCTRL_OSC8M_FRANGE_Pos)) /* Assigment of value for FRANGE in the SYSCTRL_OSC8M register */
#define   SYSCTRL_OSC8M_FRANGE_0_Val          _UINT32_(0x0)                                        /* (SYSCTRL_OSC8M) 4 to 6MHz  */
#define   SYSCTRL_OSC8M_FRANGE_1_Val          _UINT32_(0x1)                                        /* (SYSCTRL_OSC8M) 6 to 8MHz  */
#define   SYSCTRL_OSC8M_FRANGE_2_Val          _UINT32_(0x2)                                        /* (SYSCTRL_OSC8M) 8 to 11MHz  */
#define   SYSCTRL_OSC8M_FRANGE_3_Val          _UINT32_(0x3)                                        /* (SYSCTRL_OSC8M) 11 to 15MHz  */
#define SYSCTRL_OSC8M_FRANGE_0                (SYSCTRL_OSC8M_FRANGE_0_Val << SYSCTRL_OSC8M_FRANGE_Pos) /* (SYSCTRL_OSC8M) 4 to 6MHz Position  */
#define SYSCTRL_OSC8M_FRANGE_1                (SYSCTRL_OSC8M_FRANGE_1_Val << SYSCTRL_OSC8M_FRANGE_Pos) /* (SYSCTRL_OSC8M) 6 to 8MHz Position  */
#define SYSCTRL_OSC8M_FRANGE_2                (SYSCTRL_OSC8M_FRANGE_2_Val << SYSCTRL_OSC8M_FRANGE_Pos) /* (SYSCTRL_OSC8M) 8 to 11MHz Position  */
#define SYSCTRL_OSC8M_FRANGE_3                (SYSCTRL_OSC8M_FRANGE_3_Val << SYSCTRL_OSC8M_FRANGE_Pos) /* (SYSCTRL_OSC8M) 11 to 15MHz Position  */
#define SYSCTRL_OSC8M_Msk                     _UINT32_(0xCFFF03C2)                                 /* (SYSCTRL_OSC8M) Register Mask  */


/* -------- SYSCTRL_DFLLCTRL : (SYSCTRL Offset: 0x24) (R/W 16) DFLL48M Control -------- */
#define SYSCTRL_DFLLCTRL_RESETVALUE           _UINT16_(0x80)                                       /*  (SYSCTRL_DFLLCTRL) DFLL48M Control  Reset Value */

#define SYSCTRL_DFLLCTRL_ENABLE_Pos           _UINT16_(1)                                          /* (SYSCTRL_DFLLCTRL) DFLL Enable Position */
#define SYSCTRL_DFLLCTRL_ENABLE_Msk           (_UINT16_(0x1) << SYSCTRL_DFLLCTRL_ENABLE_Pos)       /* (SYSCTRL_DFLLCTRL) DFLL Enable Mask */
#define SYSCTRL_DFLLCTRL_ENABLE(value)        (SYSCTRL_DFLLCTRL_ENABLE_Msk & (_UINT16_(value) << SYSCTRL_DFLLCTRL_ENABLE_Pos)) /* Assigment of value for ENABLE in the SYSCTRL_DFLLCTRL register */
#define SYSCTRL_DFLLCTRL_MODE_Pos             _UINT16_(2)                                          /* (SYSCTRL_DFLLCTRL) Operating Mode Selection Position */
#define SYSCTRL_DFLLCTRL_MODE_Msk             (_UINT16_(0x1) << SYSCTRL_DFLLCTRL_MODE_Pos)         /* (SYSCTRL_DFLLCTRL) Operating Mode Selection Mask */
#define SYSCTRL_DFLLCTRL_MODE(value)          (SYSCTRL_DFLLCTRL_MODE_Msk & (_UINT16_(value) << SYSCTRL_DFLLCTRL_MODE_Pos)) /* Assigment of value for MODE in the SYSCTRL_DFLLCTRL register */
#define SYSCTRL_DFLLCTRL_STABLE_Pos           _UINT16_(3)                                          /* (SYSCTRL_DFLLCTRL) Stable DFLL Frequency Position */
#define SYSCTRL_DFLLCTRL_STABLE_Msk           (_UINT16_(0x1) << SYSCTRL_DFLLCTRL_STABLE_Pos)       /* (SYSCTRL_DFLLCTRL) Stable DFLL Frequency Mask */
#define SYSCTRL_DFLLCTRL_STABLE(value)        (SYSCTRL_DFLLCTRL_STABLE_Msk & (_UINT16_(value) << SYSCTRL_DFLLCTRL_STABLE_Pos)) /* Assigment of value for STABLE in the SYSCTRL_DFLLCTRL register */
#define SYSCTRL_DFLLCTRL_LLAW_Pos             _UINT16_(4)                                          /* (SYSCTRL_DFLLCTRL) Lose Lock After Wake Position */
#define SYSCTRL_DFLLCTRL_LLAW_Msk             (_UINT16_(0x1) << SYSCTRL_DFLLCTRL_LLAW_Pos)         /* (SYSCTRL_DFLLCTRL) Lose Lock After Wake Mask */
#define SYSCTRL_DFLLCTRL_LLAW(value)          (SYSCTRL_DFLLCTRL_LLAW_Msk & (_UINT16_(value) << SYSCTRL_DFLLCTRL_LLAW_Pos)) /* Assigment of value for LLAW in the SYSCTRL_DFLLCTRL register */
#define SYSCTRL_DFLLCTRL_USBCRM_Pos           _UINT16_(5)                                          /* (SYSCTRL_DFLLCTRL) USB Clock Recovery Mode Position */
#define SYSCTRL_DFLLCTRL_USBCRM_Msk           (_UINT16_(0x1) << SYSCTRL_DFLLCTRL_USBCRM_Pos)       /* (SYSCTRL_DFLLCTRL) USB Clock Recovery Mode Mask */
#define SYSCTRL_DFLLCTRL_USBCRM(value)        (SYSCTRL_DFLLCTRL_USBCRM_Msk & (_UINT16_(value) << SYSCTRL_DFLLCTRL_USBCRM_Pos)) /* Assigment of value for USBCRM in the SYSCTRL_DFLLCTRL register */
#define SYSCTRL_DFLLCTRL_RUNSTDBY_Pos         _UINT16_(6)                                          /* (SYSCTRL_DFLLCTRL) Run in Standby Position */
#define SYSCTRL_DFLLCTRL_RUNSTDBY_Msk         (_UINT16_(0x1) << SYSCTRL_DFLLCTRL_RUNSTDBY_Pos)     /* (SYSCTRL_DFLLCTRL) Run in Standby Mask */
#define SYSCTRL_DFLLCTRL_RUNSTDBY(value)      (SYSCTRL_DFLLCTRL_RUNSTDBY_Msk & (_UINT16_(value) << SYSCTRL_DFLLCTRL_RUNSTDBY_Pos)) /* Assigment of value for RUNSTDBY in the SYSCTRL_DFLLCTRL register */
#define SYSCTRL_DFLLCTRL_ONDEMAND_Pos         _UINT16_(7)                                          /* (SYSCTRL_DFLLCTRL) On Demand Control Position */
#define SYSCTRL_DFLLCTRL_ONDEMAND_Msk         (_UINT16_(0x1) << SYSCTRL_DFLLCTRL_ONDEMAND_Pos)     /* (SYSCTRL_DFLLCTRL) On Demand Control Mask */
#define SYSCTRL_DFLLCTRL_ONDEMAND(value)      (SYSCTRL_DFLLCTRL_ONDEMAND_Msk & (_UINT16_(value) << SYSCTRL_DFLLCTRL_ONDEMAND_Pos)) /* Assigment of value for ONDEMAND in the SYSCTRL_DFLLCTRL register */
#define SYSCTRL_DFLLCTRL_CCDIS_Pos            _UINT16_(8)                                          /* (SYSCTRL_DFLLCTRL) Chill Cycle Disable Position */
#define SYSCTRL_DFLLCTRL_CCDIS_Msk            (_UINT16_(0x1) << SYSCTRL_DFLLCTRL_CCDIS_Pos)        /* (SYSCTRL_DFLLCTRL) Chill Cycle Disable Mask */
#define SYSCTRL_DFLLCTRL_CCDIS(value)         (SYSCTRL_DFLLCTRL_CCDIS_Msk & (_UINT16_(value) << SYSCTRL_DFLLCTRL_CCDIS_Pos)) /* Assigment of value for CCDIS in the SYSCTRL_DFLLCTRL register */
#define SYSCTRL_DFLLCTRL_QLDIS_Pos            _UINT16_(9)                                          /* (SYSCTRL_DFLLCTRL) Quick Lock Disable Position */
#define SYSCTRL_DFLLCTRL_QLDIS_Msk            (_UINT16_(0x1) << SYSCTRL_DFLLCTRL_QLDIS_Pos)        /* (SYSCTRL_DFLLCTRL) Quick Lock Disable Mask */
#define SYSCTRL_DFLLCTRL_QLDIS(value)         (SYSCTRL_DFLLCTRL_QLDIS_Msk & (_UINT16_(value) << SYSCTRL_DFLLCTRL_QLDIS_Pos)) /* Assigment of value for QLDIS in the SYSCTRL_DFLLCTRL register */
#define SYSCTRL_DFLLCTRL_BPLCKC_Pos           _UINT16_(10)                                         /* (SYSCTRL_DFLLCTRL) Bypass Coarse Lock Position */
#define SYSCTRL_DFLLCTRL_BPLCKC_Msk           (_UINT16_(0x1) << SYSCTRL_DFLLCTRL_BPLCKC_Pos)       /* (SYSCTRL_DFLLCTRL) Bypass Coarse Lock Mask */
#define SYSCTRL_DFLLCTRL_BPLCKC(value)        (SYSCTRL_DFLLCTRL_BPLCKC_Msk & (_UINT16_(value) << SYSCTRL_DFLLCTRL_BPLCKC_Pos)) /* Assigment of value for BPLCKC in the SYSCTRL_DFLLCTRL register */
#define SYSCTRL_DFLLCTRL_WAITLOCK_Pos         _UINT16_(11)                                         /* (SYSCTRL_DFLLCTRL) Wait Lock Position */
#define SYSCTRL_DFLLCTRL_WAITLOCK_Msk         (_UINT16_(0x1) << SYSCTRL_DFLLCTRL_WAITLOCK_Pos)     /* (SYSCTRL_DFLLCTRL) Wait Lock Mask */
#define SYSCTRL_DFLLCTRL_WAITLOCK(value)      (SYSCTRL_DFLLCTRL_WAITLOCK_Msk & (_UINT16_(value) << SYSCTRL_DFLLCTRL_WAITLOCK_Pos)) /* Assigment of value for WAITLOCK in the SYSCTRL_DFLLCTRL register */
#define SYSCTRL_DFLLCTRL_Msk                  _UINT16_(0x0FFE)                                     /* (SYSCTRL_DFLLCTRL) Register Mask  */


/* -------- SYSCTRL_DFLLVAL : (SYSCTRL Offset: 0x28) (R/W 32) DFLL48M Value -------- */
#define SYSCTRL_DFLLVAL_RESETVALUE            _UINT32_(0x00)                                       /*  (SYSCTRL_DFLLVAL) DFLL48M Value  Reset Value */

#define SYSCTRL_DFLLVAL_FINE_Pos              _UINT32_(0)                                          /* (SYSCTRL_DFLLVAL) Fine Value Position */
#define SYSCTRL_DFLLVAL_FINE_Msk              (_UINT32_(0x3FF) << SYSCTRL_DFLLVAL_FINE_Pos)        /* (SYSCTRL_DFLLVAL) Fine Value Mask */
#define SYSCTRL_DFLLVAL_FINE(value)           (SYSCTRL_DFLLVAL_FINE_Msk & (_UINT32_(value) << SYSCTRL_DFLLVAL_FINE_Pos)) /* Assigment of value for FINE in the SYSCTRL_DFLLVAL register */
#define SYSCTRL_DFLLVAL_COARSE_Pos            _UINT32_(10)                                         /* (SYSCTRL_DFLLVAL) Coarse Value Position */
#define SYSCTRL_DFLLVAL_COARSE_Msk            (_UINT32_(0x3F) << SYSCTRL_DFLLVAL_COARSE_Pos)       /* (SYSCTRL_DFLLVAL) Coarse Value Mask */
#define SYSCTRL_DFLLVAL_COARSE(value)         (SYSCTRL_DFLLVAL_COARSE_Msk & (_UINT32_(value) << SYSCTRL_DFLLVAL_COARSE_Pos)) /* Assigment of value for COARSE in the SYSCTRL_DFLLVAL register */
#define SYSCTRL_DFLLVAL_DIFF_Pos              _UINT32_(16)                                         /* (SYSCTRL_DFLLVAL) Multiplication Ratio Difference Position */
#define SYSCTRL_DFLLVAL_DIFF_Msk              (_UINT32_(0xFFFF) << SYSCTRL_DFLLVAL_DIFF_Pos)       /* (SYSCTRL_DFLLVAL) Multiplication Ratio Difference Mask */
#define SYSCTRL_DFLLVAL_DIFF(value)           (SYSCTRL_DFLLVAL_DIFF_Msk & (_UINT32_(value) << SYSCTRL_DFLLVAL_DIFF_Pos)) /* Assigment of value for DIFF in the SYSCTRL_DFLLVAL register */
#define SYSCTRL_DFLLVAL_Msk                   _UINT32_(0xFFFFFFFF)                                 /* (SYSCTRL_DFLLVAL) Register Mask  */


/* -------- SYSCTRL_DFLLMUL : (SYSCTRL Offset: 0x2C) (R/W 32) DFLL48M Multiplier -------- */
#define SYSCTRL_DFLLMUL_RESETVALUE            _UINT32_(0x00)                                       /*  (SYSCTRL_DFLLMUL) DFLL48M Multiplier  Reset Value */

#define SYSCTRL_DFLLMUL_MUL_Pos               _UINT32_(0)                                          /* (SYSCTRL_DFLLMUL) DFLL Multiply Factor Position */
#define SYSCTRL_DFLLMUL_MUL_Msk               (_UINT32_(0xFFFF) << SYSCTRL_DFLLMUL_MUL_Pos)        /* (SYSCTRL_DFLLMUL) DFLL Multiply Factor Mask */
#define SYSCTRL_DFLLMUL_MUL(value)            (SYSCTRL_DFLLMUL_MUL_Msk & (_UINT32_(value) << SYSCTRL_DFLLMUL_MUL_Pos)) /* Assigment of value for MUL in the SYSCTRL_DFLLMUL register */
#define SYSCTRL_DFLLMUL_FSTEP_Pos             _UINT32_(16)                                         /* (SYSCTRL_DFLLMUL) Fine Maximum Step Position */
#define SYSCTRL_DFLLMUL_FSTEP_Msk             (_UINT32_(0x3FF) << SYSCTRL_DFLLMUL_FSTEP_Pos)       /* (SYSCTRL_DFLLMUL) Fine Maximum Step Mask */
#define SYSCTRL_DFLLMUL_FSTEP(value)          (SYSCTRL_DFLLMUL_FSTEP_Msk & (_UINT32_(value) << SYSCTRL_DFLLMUL_FSTEP_Pos)) /* Assigment of value for FSTEP in the SYSCTRL_DFLLMUL register */
#define SYSCTRL_DFLLMUL_CSTEP_Pos             _UINT32_(26)                                         /* (SYSCTRL_DFLLMUL) Coarse Maximum Step Position */
#define SYSCTRL_DFLLMUL_CSTEP_Msk             (_UINT32_(0x3F) << SYSCTRL_DFLLMUL_CSTEP_Pos)        /* (SYSCTRL_DFLLMUL) Coarse Maximum Step Mask */
#define SYSCTRL_DFLLMUL_CSTEP(value)          (SYSCTRL_DFLLMUL_CSTEP_Msk & (_UINT32_(value) << SYSCTRL_DFLLMUL_CSTEP_Pos)) /* Assigment of value for CSTEP in the SYSCTRL_DFLLMUL register */
#define SYSCTRL_DFLLMUL_Msk                   _UINT32_(0xFFFFFFFF)                                 /* (SYSCTRL_DFLLMUL) Register Mask  */


/* -------- SYSCTRL_DFLLSYNC : (SYSCTRL Offset: 0x30) (R/W 8) DFLL48M Synchronization -------- */
#define SYSCTRL_DFLLSYNC_RESETVALUE           _UINT8_(0x00)                                        /*  (SYSCTRL_DFLLSYNC) DFLL48M Synchronization  Reset Value */

#define SYSCTRL_DFLLSYNC_READREQ_Pos          _UINT8_(7)                                           /* (SYSCTRL_DFLLSYNC) Read Request Position */
#define SYSCTRL_DFLLSYNC_READREQ_Msk          (_UINT8_(0x1) << SYSCTRL_DFLLSYNC_READREQ_Pos)       /* (SYSCTRL_DFLLSYNC) Read Request Mask */
#define SYSCTRL_DFLLSYNC_READREQ(value)       (SYSCTRL_DFLLSYNC_READREQ_Msk & (_UINT8_(value) << SYSCTRL_DFLLSYNC_READREQ_Pos)) /* Assigment of value for READREQ in the SYSCTRL_DFLLSYNC register */
#define SYSCTRL_DFLLSYNC_Msk                  _UINT8_(0x80)                                        /* (SYSCTRL_DFLLSYNC) Register Mask  */


/* -------- SYSCTRL_BOD33 : (SYSCTRL Offset: 0x34) (R/W 32) 3.3V Brown-Out Detector (BOD33) Control -------- */
#define SYSCTRL_BOD33_RESETVALUE              _UINT32_(0x00)                                       /*  (SYSCTRL_BOD33) 3.3V Brown-Out Detector (BOD33) Control  Reset Value */

#define SYSCTRL_BOD33_ENABLE_Pos              _UINT32_(1)                                          /* (SYSCTRL_BOD33) Enable Position */
#define SYSCTRL_BOD33_ENABLE_Msk              (_UINT32_(0x1) << SYSCTRL_BOD33_ENABLE_Pos)          /* (SYSCTRL_BOD33) Enable Mask */
#define SYSCTRL_BOD33_ENABLE(value)           (SYSCTRL_BOD33_ENABLE_Msk & (_UINT32_(value) << SYSCTRL_BOD33_ENABLE_Pos)) /* Assigment of value for ENABLE in the SYSCTRL_BOD33 register */
#define SYSCTRL_BOD33_HYST_Pos                _UINT32_(2)                                          /* (SYSCTRL_BOD33) Hysteresis Position */
#define SYSCTRL_BOD33_HYST_Msk                (_UINT32_(0x1) << SYSCTRL_BOD33_HYST_Pos)            /* (SYSCTRL_BOD33) Hysteresis Mask */
#define SYSCTRL_BOD33_HYST(value)             (SYSCTRL_BOD33_HYST_Msk & (_UINT32_(value) << SYSCTRL_BOD33_HYST_Pos)) /* Assigment of value for HYST in the SYSCTRL_BOD33 register */
#define SYSCTRL_BOD33_ACTION_Pos              _UINT32_(3)                                          /* (SYSCTRL_BOD33) BOD33 Action Position */
#define SYSCTRL_BOD33_ACTION_Msk              (_UINT32_(0x3) << SYSCTRL_BOD33_ACTION_Pos)          /* (SYSCTRL_BOD33) BOD33 Action Mask */
#define SYSCTRL_BOD33_ACTION(value)           (SYSCTRL_BOD33_ACTION_Msk & (_UINT32_(value) << SYSCTRL_BOD33_ACTION_Pos)) /* Assigment of value for ACTION in the SYSCTRL_BOD33 register */
#define   SYSCTRL_BOD33_ACTION_NONE_Val       _UINT32_(0x0)                                        /* (SYSCTRL_BOD33) No action  */
#define   SYSCTRL_BOD33_ACTION_RESET_Val      _UINT32_(0x1)                                        /* (SYSCTRL_BOD33) The BOD33 generates a reset  */
#define   SYSCTRL_BOD33_ACTION_INTERRUPT_Val  _UINT32_(0x2)                                        /* (SYSCTRL_BOD33) The BOD33 generates an interrupt  */
#define SYSCTRL_BOD33_ACTION_NONE             (SYSCTRL_BOD33_ACTION_NONE_Val << SYSCTRL_BOD33_ACTION_Pos) /* (SYSCTRL_BOD33) No action Position  */
#define SYSCTRL_BOD33_ACTION_RESET            (SYSCTRL_BOD33_ACTION_RESET_Val << SYSCTRL_BOD33_ACTION_Pos) /* (SYSCTRL_BOD33) The BOD33 generates a reset Position  */
#define SYSCTRL_BOD33_ACTION_INTERRUPT        (SYSCTRL_BOD33_ACTION_INTERRUPT_Val << SYSCTRL_BOD33_ACTION_Pos) /* (SYSCTRL_BOD33) The BOD33 generates an interrupt Position  */
#define SYSCTRL_BOD33_RUNSTDBY_Pos            _UINT32_(6)                                          /* (SYSCTRL_BOD33) Run in Standby Position */
#define SYSCTRL_BOD33_RUNSTDBY_Msk            (_UINT32_(0x1) << SYSCTRL_BOD33_RUNSTDBY_Pos)        /* (SYSCTRL_BOD33) Run in Standby Mask */
#define SYSCTRL_BOD33_RUNSTDBY(value)         (SYSCTRL_BOD33_RUNSTDBY_Msk & (_UINT32_(value) << SYSCTRL_BOD33_RUNSTDBY_Pos)) /* Assigment of value for RUNSTDBY in the SYSCTRL_BOD33 register */
#define SYSCTRL_BOD33_MODE_Pos                _UINT32_(8)                                          /* (SYSCTRL_BOD33) Operation Mode Position */
#define SYSCTRL_BOD33_MODE_Msk                (_UINT32_(0x1) << SYSCTRL_BOD33_MODE_Pos)            /* (SYSCTRL_BOD33) Operation Mode Mask */
#define SYSCTRL_BOD33_MODE(value)             (SYSCTRL_BOD33_MODE_Msk & (_UINT32_(value) << SYSCTRL_BOD33_MODE_Pos)) /* Assigment of value for MODE in the SYSCTRL_BOD33 register */
#define SYSCTRL_BOD33_CEN_Pos                 _UINT32_(9)                                          /* (SYSCTRL_BOD33) Clock Enable Position */
#define SYSCTRL_BOD33_CEN_Msk                 (_UINT32_(0x1) << SYSCTRL_BOD33_CEN_Pos)             /* (SYSCTRL_BOD33) Clock Enable Mask */
#define SYSCTRL_BOD33_CEN(value)              (SYSCTRL_BOD33_CEN_Msk & (_UINT32_(value) << SYSCTRL_BOD33_CEN_Pos)) /* Assigment of value for CEN in the SYSCTRL_BOD33 register */
#define SYSCTRL_BOD33_PSEL_Pos                _UINT32_(12)                                         /* (SYSCTRL_BOD33) Prescaler Select Position */
#define SYSCTRL_BOD33_PSEL_Msk                (_UINT32_(0xF) << SYSCTRL_BOD33_PSEL_Pos)            /* (SYSCTRL_BOD33) Prescaler Select Mask */
#define SYSCTRL_BOD33_PSEL(value)             (SYSCTRL_BOD33_PSEL_Msk & (_UINT32_(value) << SYSCTRL_BOD33_PSEL_Pos)) /* Assigment of value for PSEL in the SYSCTRL_BOD33 register */
#define   SYSCTRL_BOD33_PSEL_DIV2_Val         _UINT32_(0x0)                                        /* (SYSCTRL_BOD33) Divide clock by 2  */
#define   SYSCTRL_BOD33_PSEL_DIV4_Val         _UINT32_(0x1)                                        /* (SYSCTRL_BOD33) Divide clock by 4  */
#define   SYSCTRL_BOD33_PSEL_DIV8_Val         _UINT32_(0x2)                                        /* (SYSCTRL_BOD33) Divide clock by 8  */
#define   SYSCTRL_BOD33_PSEL_DIV16_Val        _UINT32_(0x3)                                        /* (SYSCTRL_BOD33) Divide clock by 16  */
#define   SYSCTRL_BOD33_PSEL_DIV32_Val        _UINT32_(0x4)                                        /* (SYSCTRL_BOD33) Divide clock by 32  */
#define   SYSCTRL_BOD33_PSEL_DIV64_Val        _UINT32_(0x5)                                        /* (SYSCTRL_BOD33) Divide clock by 64  */
#define   SYSCTRL_BOD33_PSEL_DIV128_Val       _UINT32_(0x6)                                        /* (SYSCTRL_BOD33) Divide clock by 128  */
#define   SYSCTRL_BOD33_PSEL_DIV256_Val       _UINT32_(0x7)                                        /* (SYSCTRL_BOD33) Divide clock by 256  */
#define   SYSCTRL_BOD33_PSEL_DIV512_Val       _UINT32_(0x8)                                        /* (SYSCTRL_BOD33) Divide clock by 512  */
#define   SYSCTRL_BOD33_PSEL_DIV1K_Val        _UINT32_(0x9)                                        /* (SYSCTRL_BOD33) Divide clock by 1024  */
#define   SYSCTRL_BOD33_PSEL_DIV2K_Val        _UINT32_(0xA)                                        /* (SYSCTRL_BOD33) Divide clock by 2048  */
#define   SYSCTRL_BOD33_PSEL_DIV4K_Val        _UINT32_(0xB)                                        /* (SYSCTRL_BOD33) Divide clock by 4096  */
#define   SYSCTRL_BOD33_PSEL_DIV8K_Val        _UINT32_(0xC)                                        /* (SYSCTRL_BOD33) Divide clock by 8192  */
#define   SYSCTRL_BOD33_PSEL_DIV16K_Val       _UINT32_(0xD)                                        /* (SYSCTRL_BOD33) Divide clock by 16384  */
#define   SYSCTRL_BOD33_PSEL_DIV32K_Val       _UINT32_(0xE)                                        /* (SYSCTRL_BOD33) Divide clock by 32768  */
#define   SYSCTRL_BOD33_PSEL_DIV64K_Val       _UINT32_(0xF)                                        /* (SYSCTRL_BOD33) Divide clock by 65536  */
#define SYSCTRL_BOD33_PSEL_DIV2               (SYSCTRL_BOD33_PSEL_DIV2_Val << SYSCTRL_BOD33_PSEL_Pos) /* (SYSCTRL_BOD33) Divide clock by 2 Position  */
#define SYSCTRL_BOD33_PSEL_DIV4               (SYSCTRL_BOD33_PSEL_DIV4_Val << SYSCTRL_BOD33_PSEL_Pos) /* (SYSCTRL_BOD33) Divide clock by 4 Position  */
#define SYSCTRL_BOD33_PSEL_DIV8               (SYSCTRL_BOD33_PSEL_DIV8_Val << SYSCTRL_BOD33_PSEL_Pos) /* (SYSCTRL_BOD33) Divide clock by 8 Position  */
#define SYSCTRL_BOD33_PSEL_DIV16              (SYSCTRL_BOD33_PSEL_DIV16_Val << SYSCTRL_BOD33_PSEL_Pos) /* (SYSCTRL_BOD33) Divide clock by 16 Position  */
#define SYSCTRL_BOD33_PSEL_DIV32              (SYSCTRL_BOD33_PSEL_DIV32_Val << SYSCTRL_BOD33_PSEL_Pos) /* (SYSCTRL_BOD33) Divide clock by 32 Position  */
#define SYSCTRL_BOD33_PSEL_DIV64              (SYSCTRL_BOD33_PSEL_DIV64_Val << SYSCTRL_BOD33_PSEL_Pos) /* (SYSCTRL_BOD33) Divide clock by 64 Position  */
#define SYSCTRL_BOD33_PSEL_DIV128             (SYSCTRL_BOD33_PSEL_DIV128_Val << SYSCTRL_BOD33_PSEL_Pos) /* (SYSCTRL_BOD33) Divide clock by 128 Position  */
#define SYSCTRL_BOD33_PSEL_DIV256             (SYSCTRL_BOD33_PSEL_DIV256_Val << SYSCTRL_BOD33_PSEL_Pos) /* (SYSCTRL_BOD33) Divide clock by 256 Position  */
#define SYSCTRL_BOD33_PSEL_DIV512             (SYSCTRL_BOD33_PSEL_DIV512_Val << SYSCTRL_BOD33_PSEL_Pos) /* (SYSCTRL_BOD33) Divide clock by 512 Position  */
#define SYSCTRL_BOD33_PSEL_DIV1K              (SYSCTRL_BOD33_PSEL_DIV1K_Val << SYSCTRL_BOD33_PSEL_Pos) /* (SYSCTRL_BOD33) Divide clock by 1024 Position  */
#define SYSCTRL_BOD33_PSEL_DIV2K              (SYSCTRL_BOD33_PSEL_DIV2K_Val << SYSCTRL_BOD33_PSEL_Pos) /* (SYSCTRL_BOD33) Divide clock by 2048 Position  */
#define SYSCTRL_BOD33_PSEL_DIV4K              (SYSCTRL_BOD33_PSEL_DIV4K_Val << SYSCTRL_BOD33_PSEL_Pos) /* (SYSCTRL_BOD33) Divide clock by 4096 Position  */
#define SYSCTRL_BOD33_PSEL_DIV8K              (SYSCTRL_BOD33_PSEL_DIV8K_Val << SYSCTRL_BOD33_PSEL_Pos) /* (SYSCTRL_BOD33) Divide clock by 8192 Position  */
#define SYSCTRL_BOD33_PSEL_DIV16K             (SYSCTRL_BOD33_PSEL_DIV16K_Val << SYSCTRL_BOD33_PSEL_Pos) /* (SYSCTRL_BOD33) Divide clock by 16384 Position  */
#define SYSCTRL_BOD33_PSEL_DIV32K             (SYSCTRL_BOD33_PSEL_DIV32K_Val << SYSCTRL_BOD33_PSEL_Pos) /* (SYSCTRL_BOD33) Divide clock by 32768 Position  */
#define SYSCTRL_BOD33_PSEL_DIV64K             (SYSCTRL_BOD33_PSEL_DIV64K_Val << SYSCTRL_BOD33_PSEL_Pos) /* (SYSCTRL_BOD33) Divide clock by 65536 Position  */
#define SYSCTRL_BOD33_LEVEL_Pos               _UINT32_(16)                                         /* (SYSCTRL_BOD33) BOD33 Threshold Level Position */
#define SYSCTRL_BOD33_LEVEL_Msk               (_UINT32_(0x3F) << SYSCTRL_BOD33_LEVEL_Pos)          /* (SYSCTRL_BOD33) BOD33 Threshold Level Mask */
#define SYSCTRL_BOD33_LEVEL(value)            (SYSCTRL_BOD33_LEVEL_Msk & (_UINT32_(value) << SYSCTRL_BOD33_LEVEL_Pos)) /* Assigment of value for LEVEL in the SYSCTRL_BOD33 register */
#define SYSCTRL_BOD33_Msk                     _UINT32_(0x003FF35E)                                 /* (SYSCTRL_BOD33) Register Mask  */


/* -------- SYSCTRL_VREG : (SYSCTRL Offset: 0x3C) (R/W 16) Voltage Regulator System (VREG) Control -------- */
#define SYSCTRL_VREG_RESETVALUE               _UINT16_(0x00)                                       /*  (SYSCTRL_VREG) Voltage Regulator System (VREG) Control  Reset Value */

#define SYSCTRL_VREG_RUNSTDBY_Pos             _UINT16_(6)                                          /* (SYSCTRL_VREG) Run in Standby Position */
#define SYSCTRL_VREG_RUNSTDBY_Msk             (_UINT16_(0x1) << SYSCTRL_VREG_RUNSTDBY_Pos)         /* (SYSCTRL_VREG) Run in Standby Mask */
#define SYSCTRL_VREG_RUNSTDBY(value)          (SYSCTRL_VREG_RUNSTDBY_Msk & (_UINT16_(value) << SYSCTRL_VREG_RUNSTDBY_Pos)) /* Assigment of value for RUNSTDBY in the SYSCTRL_VREG register */
#define SYSCTRL_VREG_FORCELDO_Pos             _UINT16_(13)                                         /* (SYSCTRL_VREG) Force LDO Voltage Regulator Position */
#define SYSCTRL_VREG_FORCELDO_Msk             (_UINT16_(0x1) << SYSCTRL_VREG_FORCELDO_Pos)         /* (SYSCTRL_VREG) Force LDO Voltage Regulator Mask */
#define SYSCTRL_VREG_FORCELDO(value)          (SYSCTRL_VREG_FORCELDO_Msk & (_UINT16_(value) << SYSCTRL_VREG_FORCELDO_Pos)) /* Assigment of value for FORCELDO in the SYSCTRL_VREG register */
#define SYSCTRL_VREG_Msk                      _UINT16_(0x2040)                                     /* (SYSCTRL_VREG) Register Mask  */


/* -------- SYSCTRL_VREF : (SYSCTRL Offset: 0x40) (R/W 32) Voltage References System (VREF) Control -------- */
#define SYSCTRL_VREF_RESETVALUE               _UINT32_(0x00)                                       /*  (SYSCTRL_VREF) Voltage References System (VREF) Control  Reset Value */

#define SYSCTRL_VREF_TSEN_Pos                 _UINT32_(1)                                          /* (SYSCTRL_VREF) Temperature Sensor Enable Position */
#define SYSCTRL_VREF_TSEN_Msk                 (_UINT32_(0x1) << SYSCTRL_VREF_TSEN_Pos)             /* (SYSCTRL_VREF) Temperature Sensor Enable Mask */
#define SYSCTRL_VREF_TSEN(value)              (SYSCTRL_VREF_TSEN_Msk & (_UINT32_(value) << SYSCTRL_VREF_TSEN_Pos)) /* Assigment of value for TSEN in the SYSCTRL_VREF register */
#define SYSCTRL_VREF_BGOUTEN_Pos              _UINT32_(2)                                          /* (SYSCTRL_VREF) Bandgap Output Enable Position */
#define SYSCTRL_VREF_BGOUTEN_Msk              (_UINT32_(0x1) << SYSCTRL_VREF_BGOUTEN_Pos)          /* (SYSCTRL_VREF) Bandgap Output Enable Mask */
#define SYSCTRL_VREF_BGOUTEN(value)           (SYSCTRL_VREF_BGOUTEN_Msk & (_UINT32_(value) << SYSCTRL_VREF_BGOUTEN_Pos)) /* Assigment of value for BGOUTEN in the SYSCTRL_VREF register */
#define SYSCTRL_VREF_CALIB_Pos                _UINT32_(16)                                         /* (SYSCTRL_VREF) Bandgap Voltage Generator Calibration Position */
#define SYSCTRL_VREF_CALIB_Msk                (_UINT32_(0x7FF) << SYSCTRL_VREF_CALIB_Pos)          /* (SYSCTRL_VREF) Bandgap Voltage Generator Calibration Mask */
#define SYSCTRL_VREF_CALIB(value)             (SYSCTRL_VREF_CALIB_Msk & (_UINT32_(value) << SYSCTRL_VREF_CALIB_Pos)) /* Assigment of value for CALIB in the SYSCTRL_VREF register */
#define SYSCTRL_VREF_Msk                      _UINT32_(0x07FF0006)                                 /* (SYSCTRL_VREF) Register Mask  */


/* -------- SYSCTRL_DPLLCTRLA : (SYSCTRL Offset: 0x44) (R/W 8) DPLL Control A -------- */
#define SYSCTRL_DPLLCTRLA_RESETVALUE          _UINT8_(0x80)                                        /*  (SYSCTRL_DPLLCTRLA) DPLL Control A  Reset Value */

#define SYSCTRL_DPLLCTRLA_ENABLE_Pos          _UINT8_(1)                                           /* (SYSCTRL_DPLLCTRLA) DPLL Enable Position */
#define SYSCTRL_DPLLCTRLA_ENABLE_Msk          (_UINT8_(0x1) << SYSCTRL_DPLLCTRLA_ENABLE_Pos)       /* (SYSCTRL_DPLLCTRLA) DPLL Enable Mask */
#define SYSCTRL_DPLLCTRLA_ENABLE(value)       (SYSCTRL_DPLLCTRLA_ENABLE_Msk & (_UINT8_(value) << SYSCTRL_DPLLCTRLA_ENABLE_Pos)) /* Assigment of value for ENABLE in the SYSCTRL_DPLLCTRLA register */
#define SYSCTRL_DPLLCTRLA_RUNSTDBY_Pos        _UINT8_(6)                                           /* (SYSCTRL_DPLLCTRLA) Run in Standby Position */
#define SYSCTRL_DPLLCTRLA_RUNSTDBY_Msk        (_UINT8_(0x1) << SYSCTRL_DPLLCTRLA_RUNSTDBY_Pos)     /* (SYSCTRL_DPLLCTRLA) Run in Standby Mask */
#define SYSCTRL_DPLLCTRLA_RUNSTDBY(value)     (SYSCTRL_DPLLCTRLA_RUNSTDBY_Msk & (_UINT8_(value) << SYSCTRL_DPLLCTRLA_RUNSTDBY_Pos)) /* Assigment of value for RUNSTDBY in the SYSCTRL_DPLLCTRLA register */
#define SYSCTRL_DPLLCTRLA_ONDEMAND_Pos        _UINT8_(7)                                           /* (SYSCTRL_DPLLCTRLA) On Demand Clock Activation Position */
#define SYSCTRL_DPLLCTRLA_ONDEMAND_Msk        (_UINT8_(0x1) << SYSCTRL_DPLLCTRLA_ONDEMAND_Pos)     /* (SYSCTRL_DPLLCTRLA) On Demand Clock Activation Mask */
#define SYSCTRL_DPLLCTRLA_ONDEMAND(value)     (SYSCTRL_DPLLCTRLA_ONDEMAND_Msk & (_UINT8_(value) << SYSCTRL_DPLLCTRLA_ONDEMAND_Pos)) /* Assigment of value for ONDEMAND in the SYSCTRL_DPLLCTRLA register */
#define SYSCTRL_DPLLCTRLA_Msk                 _UINT8_(0xC2)                                        /* (SYSCTRL_DPLLCTRLA) Register Mask  */


/* -------- SYSCTRL_DPLLRATIO : (SYSCTRL Offset: 0x48) (R/W 32) DPLL Ratio Control -------- */
#define SYSCTRL_DPLLRATIO_RESETVALUE          _UINT32_(0x00)                                       /*  (SYSCTRL_DPLLRATIO) DPLL Ratio Control  Reset Value */

#define SYSCTRL_DPLLRATIO_LDR_Pos             _UINT32_(0)                                          /* (SYSCTRL_DPLLRATIO) Loop Divider Ratio Position */
#define SYSCTRL_DPLLRATIO_LDR_Msk             (_UINT32_(0xFFF) << SYSCTRL_DPLLRATIO_LDR_Pos)       /* (SYSCTRL_DPLLRATIO) Loop Divider Ratio Mask */
#define SYSCTRL_DPLLRATIO_LDR(value)          (SYSCTRL_DPLLRATIO_LDR_Msk & (_UINT32_(value) << SYSCTRL_DPLLRATIO_LDR_Pos)) /* Assigment of value for LDR in the SYSCTRL_DPLLRATIO register */
#define SYSCTRL_DPLLRATIO_LDRFRAC_Pos         _UINT32_(16)                                         /* (SYSCTRL_DPLLRATIO) Loop Divider Ratio Fractional Part Position */
#define SYSCTRL_DPLLRATIO_LDRFRAC_Msk         (_UINT32_(0xF) << SYSCTRL_DPLLRATIO_LDRFRAC_Pos)     /* (SYSCTRL_DPLLRATIO) Loop Divider Ratio Fractional Part Mask */
#define SYSCTRL_DPLLRATIO_LDRFRAC(value)      (SYSCTRL_DPLLRATIO_LDRFRAC_Msk & (_UINT32_(value) << SYSCTRL_DPLLRATIO_LDRFRAC_Pos)) /* Assigment of value for LDRFRAC in the SYSCTRL_DPLLRATIO register */
#define SYSCTRL_DPLLRATIO_Msk                 _UINT32_(0x000F0FFF)                                 /* (SYSCTRL_DPLLRATIO) Register Mask  */


/* -------- SYSCTRL_DPLLCTRLB : (SYSCTRL Offset: 0x4C) (R/W 32) DPLL Control B -------- */
#define SYSCTRL_DPLLCTRLB_RESETVALUE          _UINT32_(0x00)                                       /*  (SYSCTRL_DPLLCTRLB) DPLL Control B  Reset Value */

#define SYSCTRL_DPLLCTRLB_FILTER_Pos          _UINT32_(0)                                          /* (SYSCTRL_DPLLCTRLB) Proportional Integral Filter Selection Position */
#define SYSCTRL_DPLLCTRLB_FILTER_Msk          (_UINT32_(0x3) << SYSCTRL_DPLLCTRLB_FILTER_Pos)      /* (SYSCTRL_DPLLCTRLB) Proportional Integral Filter Selection Mask */
#define SYSCTRL_DPLLCTRLB_FILTER(value)       (SYSCTRL_DPLLCTRLB_FILTER_Msk & (_UINT32_(value) << SYSCTRL_DPLLCTRLB_FILTER_Pos)) /* Assigment of value for FILTER in the SYSCTRL_DPLLCTRLB register */
#define   SYSCTRL_DPLLCTRLB_FILTER_DEFAULT_Val _UINT32_(0x0)                                        /* (SYSCTRL_DPLLCTRLB) Default filter mode  */
#define   SYSCTRL_DPLLCTRLB_FILTER_LBFILT_Val _UINT32_(0x1)                                        /* (SYSCTRL_DPLLCTRLB) Low bandwidth filter  */
#define   SYSCTRL_DPLLCTRLB_FILTER_HBFILT_Val _UINT32_(0x2)                                        /* (SYSCTRL_DPLLCTRLB) High bandwidth filter  */
#define   SYSCTRL_DPLLCTRLB_FILTER_HDFILT_Val _UINT32_(0x3)                                        /* (SYSCTRL_DPLLCTRLB) High damping filter  */
#define SYSCTRL_DPLLCTRLB_FILTER_DEFAULT      (SYSCTRL_DPLLCTRLB_FILTER_DEFAULT_Val << SYSCTRL_DPLLCTRLB_FILTER_Pos) /* (SYSCTRL_DPLLCTRLB) Default filter mode Position  */
#define SYSCTRL_DPLLCTRLB_FILTER_LBFILT       (SYSCTRL_DPLLCTRLB_FILTER_LBFILT_Val << SYSCTRL_DPLLCTRLB_FILTER_Pos) /* (SYSCTRL_DPLLCTRLB) Low bandwidth filter Position  */
#define SYSCTRL_DPLLCTRLB_FILTER_HBFILT       (SYSCTRL_DPLLCTRLB_FILTER_HBFILT_Val << SYSCTRL_DPLLCTRLB_FILTER_Pos) /* (SYSCTRL_DPLLCTRLB) High bandwidth filter Position  */
#define SYSCTRL_DPLLCTRLB_FILTER_HDFILT       (SYSCTRL_DPLLCTRLB_FILTER_HDFILT_Val << SYSCTRL_DPLLCTRLB_FILTER_Pos) /* (SYSCTRL_DPLLCTRLB) High damping filter Position  */
#define SYSCTRL_DPLLCTRLB_LPEN_Pos            _UINT32_(2)                                          /* (SYSCTRL_DPLLCTRLB) Low-Power Enable Position */
#define SYSCTRL_DPLLCTRLB_LPEN_Msk            (_UINT32_(0x1) << SYSCTRL_DPLLCTRLB_LPEN_Pos)        /* (SYSCTRL_DPLLCTRLB) Low-Power Enable Mask */
#define SYSCTRL_DPLLCTRLB_LPEN(value)         (SYSCTRL_DPLLCTRLB_LPEN_Msk & (_UINT32_(value) << SYSCTRL_DPLLCTRLB_LPEN_Pos)) /* Assigment of value for LPEN in the SYSCTRL_DPLLCTRLB register */
#define SYSCTRL_DPLLCTRLB_WUF_Pos             _UINT32_(3)                                          /* (SYSCTRL_DPLLCTRLB) Wake Up Fast Position */
#define SYSCTRL_DPLLCTRLB_WUF_Msk             (_UINT32_(0x1) << SYSCTRL_DPLLCTRLB_WUF_Pos)         /* (SYSCTRL_DPLLCTRLB) Wake Up Fast Mask */
#define SYSCTRL_DPLLCTRLB_WUF(value)          (SYSCTRL_DPLLCTRLB_WUF_Msk & (_UINT32_(value) << SYSCTRL_DPLLCTRLB_WUF_Pos)) /* Assigment of value for WUF in the SYSCTRL_DPLLCTRLB register */
#define SYSCTRL_DPLLCTRLB_REFCLK_Pos          _UINT32_(4)                                          /* (SYSCTRL_DPLLCTRLB) Reference Clock Selection Position */
#define SYSCTRL_DPLLCTRLB_REFCLK_Msk          (_UINT32_(0x3) << SYSCTRL_DPLLCTRLB_REFCLK_Pos)      /* (SYSCTRL_DPLLCTRLB) Reference Clock Selection Mask */
#define SYSCTRL_DPLLCTRLB_REFCLK(value)       (SYSCTRL_DPLLCTRLB_REFCLK_Msk & (_UINT32_(value) << SYSCTRL_DPLLCTRLB_REFCLK_Pos)) /* Assigment of value for REFCLK in the SYSCTRL_DPLLCTRLB register */
#define   SYSCTRL_DPLLCTRLB_REFCLK_REF0_Val   _UINT32_(0x0)                                        /* (SYSCTRL_DPLLCTRLB) CLK_DPLL_REF0 clock reference  */
#define   SYSCTRL_DPLLCTRLB_REFCLK_REF1_Val   _UINT32_(0x1)                                        /* (SYSCTRL_DPLLCTRLB) CLK_DPLL_REF1 clock reference  */
#define   SYSCTRL_DPLLCTRLB_REFCLK_GCLK_Val   _UINT32_(0x2)                                        /* (SYSCTRL_DPLLCTRLB) GCLK_DPLL clock reference  */
#define SYSCTRL_DPLLCTRLB_REFCLK_REF0         (SYSCTRL_DPLLCTRLB_REFCLK_REF0_Val << SYSCTRL_DPLLCTRLB_REFCLK_Pos) /* (SYSCTRL_DPLLCTRLB) CLK_DPLL_REF0 clock reference Position  */
#define SYSCTRL_DPLLCTRLB_REFCLK_REF1         (SYSCTRL_DPLLCTRLB_REFCLK_REF1_Val << SYSCTRL_DPLLCTRLB_REFCLK_Pos) /* (SYSCTRL_DPLLCTRLB) CLK_DPLL_REF1 clock reference Position  */
#define SYSCTRL_DPLLCTRLB_REFCLK_GCLK         (SYSCTRL_DPLLCTRLB_REFCLK_GCLK_Val << SYSCTRL_DPLLCTRLB_REFCLK_Pos) /* (SYSCTRL_DPLLCTRLB) GCLK_DPLL clock reference Position  */
#define SYSCTRL_DPLLCTRLB_LTIME_Pos           _UINT32_(8)                                          /* (SYSCTRL_DPLLCTRLB) Lock Time Position */
#define SYSCTRL_DPLLCTRLB_LTIME_Msk           (_UINT32_(0x7) << SYSCTRL_DPLLCTRLB_LTIME_Pos)       /* (SYSCTRL_DPLLCTRLB) Lock Time Mask */
#define SYSCTRL_DPLLCTRLB_LTIME(value)        (SYSCTRL_DPLLCTRLB_LTIME_Msk & (_UINT32_(value) << SYSCTRL_DPLLCTRLB_LTIME_Pos)) /* Assigment of value for LTIME in the SYSCTRL_DPLLCTRLB register */
#define   SYSCTRL_DPLLCTRLB_LTIME_DEFAULT_Val _UINT32_(0x0)                                        /* (SYSCTRL_DPLLCTRLB) No time-out  */
#define   SYSCTRL_DPLLCTRLB_LTIME_8MS_Val     _UINT32_(0x4)                                        /* (SYSCTRL_DPLLCTRLB) Time-out if no lock within 8 ms  */
#define   SYSCTRL_DPLLCTRLB_LTIME_9MS_Val     _UINT32_(0x5)                                        /* (SYSCTRL_DPLLCTRLB) Time-out if no lock within 9 ms  */
#define   SYSCTRL_DPLLCTRLB_LTIME_10MS_Val    _UINT32_(0x6)                                        /* (SYSCTRL_DPLLCTRLB) Time-out if no lock within 10 ms  */
#define   SYSCTRL_DPLLCTRLB_LTIME_11MS_Val    _UINT32_(0x7)                                        /* (SYSCTRL_DPLLCTRLB) Time-out if no lock within 11 ms  */
#define SYSCTRL_DPLLCTRLB_LTIME_DEFAULT       (SYSCTRL_DPLLCTRLB_LTIME_DEFAULT_Val << SYSCTRL_DPLLCTRLB_LTIME_Pos) /* (SYSCTRL_DPLLCTRLB) No time-out Position  */
#define SYSCTRL_DPLLCTRLB_LTIME_8MS           (SYSCTRL_DPLLCTRLB_LTIME_8MS_Val << SYSCTRL_DPLLCTRLB_LTIME_Pos) /* (SYSCTRL_DPLLCTRLB) Time-out if no lock within 8 ms Position  */
#define SYSCTRL_DPLLCTRLB_LTIME_9MS           (SYSCTRL_DPLLCTRLB_LTIME_9MS_Val << SYSCTRL_DPLLCTRLB_LTIME_Pos) /* (SYSCTRL_DPLLCTRLB) Time-out if no lock within 9 ms Position  */
#define SYSCTRL_DPLLCTRLB_LTIME_10MS          (SYSCTRL_DPLLCTRLB_LTIME_10MS_Val << SYSCTRL_DPLLCTRLB_LTIME_Pos) /* (SYSCTRL_DPLLCTRLB) Time-out if no lock within 10 ms Position  */
#define SYSCTRL_DPLLCTRLB_LTIME_11MS          (SYSCTRL_DPLLCTRLB_LTIME_11MS_Val << SYSCTRL_DPLLCTRLB_LTIME_Pos) /* (SYSCTRL_DPLLCTRLB) Time-out if no lock within 11 ms Position  */
#define SYSCTRL_DPLLCTRLB_LBYPASS_Pos         _UINT32_(12)                                         /* (SYSCTRL_DPLLCTRLB) Lock Bypass Position */
#define SYSCTRL_DPLLCTRLB_LBYPASS_Msk         (_UINT32_(0x1) << SYSCTRL_DPLLCTRLB_LBYPASS_Pos)     /* (SYSCTRL_DPLLCTRLB) Lock Bypass Mask */
#define SYSCTRL_DPLLCTRLB_LBYPASS(value)      (SYSCTRL_DPLLCTRLB_LBYPASS_Msk & (_UINT32_(value) << SYSCTRL_DPLLCTRLB_LBYPASS_Pos)) /* Assigment of value for LBYPASS in the SYSCTRL_DPLLCTRLB register */
#define SYSCTRL_DPLLCTRLB_DIV_Pos             _UINT32_(16)                                         /* (SYSCTRL_DPLLCTRLB) Clock Divider Position */
#define SYSCTRL_DPLLCTRLB_DIV_Msk             (_UINT32_(0x7FF) << SYSCTRL_DPLLCTRLB_DIV_Pos)       /* (SYSCTRL_DPLLCTRLB) Clock Divider Mask */
#define SYSCTRL_DPLLCTRLB_DIV(value)          (SYSCTRL_DPLLCTRLB_DIV_Msk & (_UINT32_(value) << SYSCTRL_DPLLCTRLB_DIV_Pos)) /* Assigment of value for DIV in the SYSCTRL_DPLLCTRLB register */
#define SYSCTRL_DPLLCTRLB_Msk                 _UINT32_(0x07FF173F)                                 /* (SYSCTRL_DPLLCTRLB) Register Mask  */


/* -------- SYSCTRL_DPLLSTATUS : (SYSCTRL Offset: 0x50) ( R/ 8) DPLL Status -------- */
#define SYSCTRL_DPLLSTATUS_RESETVALUE         _UINT8_(0x00)                                        /*  (SYSCTRL_DPLLSTATUS) DPLL Status  Reset Value */

#define SYSCTRL_DPLLSTATUS_LOCK_Pos           _UINT8_(0)                                           /* (SYSCTRL_DPLLSTATUS) DPLL Lock Status Position */
#define SYSCTRL_DPLLSTATUS_LOCK_Msk           (_UINT8_(0x1) << SYSCTRL_DPLLSTATUS_LOCK_Pos)        /* (SYSCTRL_DPLLSTATUS) DPLL Lock Status Mask */
#define SYSCTRL_DPLLSTATUS_LOCK(value)        (SYSCTRL_DPLLSTATUS_LOCK_Msk & (_UINT8_(value) << SYSCTRL_DPLLSTATUS_LOCK_Pos)) /* Assigment of value for LOCK in the SYSCTRL_DPLLSTATUS register */
#define SYSCTRL_DPLLSTATUS_CLKRDY_Pos         _UINT8_(1)                                           /* (SYSCTRL_DPLLSTATUS) Output Clock Ready Position */
#define SYSCTRL_DPLLSTATUS_CLKRDY_Msk         (_UINT8_(0x1) << SYSCTRL_DPLLSTATUS_CLKRDY_Pos)      /* (SYSCTRL_DPLLSTATUS) Output Clock Ready Mask */
#define SYSCTRL_DPLLSTATUS_CLKRDY(value)      (SYSCTRL_DPLLSTATUS_CLKRDY_Msk & (_UINT8_(value) << SYSCTRL_DPLLSTATUS_CLKRDY_Pos)) /* Assigment of value for CLKRDY in the SYSCTRL_DPLLSTATUS register */
#define SYSCTRL_DPLLSTATUS_ENABLE_Pos         _UINT8_(2)                                           /* (SYSCTRL_DPLLSTATUS) DPLL Enable Position */
#define SYSCTRL_DPLLSTATUS_ENABLE_Msk         (_UINT8_(0x1) << SYSCTRL_DPLLSTATUS_ENABLE_Pos)      /* (SYSCTRL_DPLLSTATUS) DPLL Enable Mask */
#define SYSCTRL_DPLLSTATUS_ENABLE(value)      (SYSCTRL_DPLLSTATUS_ENABLE_Msk & (_UINT8_(value) << SYSCTRL_DPLLSTATUS_ENABLE_Pos)) /* Assigment of value for ENABLE in the SYSCTRL_DPLLSTATUS register */
#define SYSCTRL_DPLLSTATUS_DIV_Pos            _UINT8_(3)                                           /* (SYSCTRL_DPLLSTATUS) Divider Enable Position */
#define SYSCTRL_DPLLSTATUS_DIV_Msk            (_UINT8_(0x1) << SYSCTRL_DPLLSTATUS_DIV_Pos)         /* (SYSCTRL_DPLLSTATUS) Divider Enable Mask */
#define SYSCTRL_DPLLSTATUS_DIV(value)         (SYSCTRL_DPLLSTATUS_DIV_Msk & (_UINT8_(value) << SYSCTRL_DPLLSTATUS_DIV_Pos)) /* Assigment of value for DIV in the SYSCTRL_DPLLSTATUS register */
#define SYSCTRL_DPLLSTATUS_Msk                _UINT8_(0x0F)                                        /* (SYSCTRL_DPLLSTATUS) Register Mask  */


/** \brief SYSCTRL register offsets definitions */
#define SYSCTRL_INTENCLR_REG_OFST      _UINT32_(0x00)      /* (SYSCTRL_INTENCLR) Interrupt Enable Clear Offset */
#define SYSCTRL_INTENSET_REG_OFST      _UINT32_(0x04)      /* (SYSCTRL_INTENSET) Interrupt Enable Set Offset */
#define SYSCTRL_INTFLAG_REG_OFST       _UINT32_(0x08)      /* (SYSCTRL_INTFLAG) Interrupt Flag Status and Clear Offset */
#define SYSCTRL_PCLKSR_REG_OFST        _UINT32_(0x0C)      /* (SYSCTRL_PCLKSR) Power and Clocks Status Offset */
#define SYSCTRL_XOSC_REG_OFST          _UINT32_(0x10)      /* (SYSCTRL_XOSC) External Multipurpose Crystal Oscillator (XOSC) Control Offset */
#define SYSCTRL_XOSC32K_REG_OFST       _UINT32_(0x14)      /* (SYSCTRL_XOSC32K) 32kHz External Crystal Oscillator (XOSC32K) Control Offset */
#define SYSCTRL_OSC32K_REG_OFST        _UINT32_(0x18)      /* (SYSCTRL_OSC32K) 32kHz Internal Oscillator (OSC32K) Control Offset */
#define SYSCTRL_OSCULP32K_REG_OFST     _UINT32_(0x1C)      /* (SYSCTRL_OSCULP32K) 32kHz Ultra Low Power Internal Oscillator (OSCULP32K) Control Offset */
#define SYSCTRL_OSC8M_REG_OFST         _UINT32_(0x20)      /* (SYSCTRL_OSC8M) 8MHz Internal Oscillator (OSC8M) Control Offset */
#define SYSCTRL_DFLLCTRL_REG_OFST      _UINT32_(0x24)      /* (SYSCTRL_DFLLCTRL) DFLL48M Control Offset */
#define SYSCTRL_DFLLVAL_REG_OFST       _UINT32_(0x28)      /* (SYSCTRL_DFLLVAL) DFLL48M Value Offset */
#define SYSCTRL_DFLLMUL_REG_OFST       _UINT32_(0x2C)      /* (SYSCTRL_DFLLMUL) DFLL48M Multiplier Offset */
#define SYSCTRL_DFLLSYNC_REG_OFST      _UINT32_(0x30)      /* (SYSCTRL_DFLLSYNC) DFLL48M Synchronization Offset */
#define SYSCTRL_BOD33_REG_OFST         _UINT32_(0x34)      /* (SYSCTRL_BOD33) 3.3V Brown-Out Detector (BOD33) Control Offset */
#define SYSCTRL_VREG_REG_OFST          _UINT32_(0x3C)      /* (SYSCTRL_VREG) Voltage Regulator System (VREG) Control Offset */
#define SYSCTRL_VREF_REG_OFST          _UINT32_(0x40)      /* (SYSCTRL_VREF) Voltage References System (VREF) Control Offset */
#define SYSCTRL_DPLLCTRLA_REG_OFST     _UINT32_(0x44)      /* (SYSCTRL_DPLLCTRLA) DPLL Control A Offset */
#define SYSCTRL_DPLLRATIO_REG_OFST     _UINT32_(0x48)      /* (SYSCTRL_DPLLRATIO) DPLL Ratio Control Offset */
#define SYSCTRL_DPLLCTRLB_REG_OFST     _UINT32_(0x4C)      /* (SYSCTRL_DPLLCTRLB) DPLL Control B Offset */
#define SYSCTRL_DPLLSTATUS_REG_OFST    _UINT32_(0x50)      /* (SYSCTRL_DPLLSTATUS) DPLL Status Offset */

#if !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__))
/** \brief SYSCTRL register API structure */
typedef struct
{  /* System Control */
  __IO  uint32_t                       SYSCTRL_INTENCLR;   /**< Offset: 0x00 (R/W  32) Interrupt Enable Clear */
  __IO  uint32_t                       SYSCTRL_INTENSET;   /**< Offset: 0x04 (R/W  32) Interrupt Enable Set */
  __IO  uint32_t                       SYSCTRL_INTFLAG;    /**< Offset: 0x08 (R/W  32) Interrupt Flag Status and Clear */
  __I   uint32_t                       SYSCTRL_PCLKSR;     /**< Offset: 0x0C (R/   32) Power and Clocks Status */
  __IO  uint16_t                       SYSCTRL_XOSC;       /**< Offset: 0x10 (R/W  16) External Multipurpose Crystal Oscillator (XOSC) Control */
  __I   uint8_t                        Reserved1[0x02];
  __IO  uint16_t                       SYSCTRL_XOSC32K;    /**< Offset: 0x14 (R/W  16) 32kHz External Crystal Oscillator (XOSC32K) Control */
  __I   uint8_t                        Reserved2[0x02];
  __IO  uint32_t                       SYSCTRL_OSC32K;     /**< Offset: 0x18 (R/W  32) 32kHz Internal Oscillator (OSC32K) Control */
  __IO  uint8_t                        SYSCTRL_OSCULP32K;  /**< Offset: 0x1C (R/W  8) 32kHz Ultra Low Power Internal Oscillator (OSCULP32K) Control */
  __I   uint8_t                        Reserved3[0x03];
  __IO  uint32_t                       SYSCTRL_OSC8M;      /**< Offset: 0x20 (R/W  32) 8MHz Internal Oscillator (OSC8M) Control */
  __IO  uint16_t                       SYSCTRL_DFLLCTRL;   /**< Offset: 0x24 (R/W  16) DFLL48M Control */
  __I   uint8_t                        Reserved4[0x02];
  __IO  uint32_t                       SYSCTRL_DFLLVAL;    /**< Offset: 0x28 (R/W  32) DFLL48M Value */
  __IO  uint32_t                       SYSCTRL_DFLLMUL;    /**< Offset: 0x2C (R/W  32) DFLL48M Multiplier */
  __IO  uint8_t                        SYSCTRL_DFLLSYNC;   /**< Offset: 0x30 (R/W  8) DFLL48M Synchronization */
  __I   uint8_t                        Reserved5[0x03];
  __IO  uint32_t                       SYSCTRL_BOD33;      /**< Offset: 0x34 (R/W  32) 3.3V Brown-Out Detector (BOD33) Control */
  __I   uint8_t                        Reserved6[0x04];
  __IO  uint16_t                       SYSCTRL_VREG;       /**< Offset: 0x3C (R/W  16) Voltage Regulator System (VREG) Control */
  __I   uint8_t                        Reserved7[0x02];
  __IO  uint32_t                       SYSCTRL_VREF;       /**< Offset: 0x40 (R/W  32) Voltage References System (VREF) Control */
  __IO  uint8_t                        SYSCTRL_DPLLCTRLA;  /**< Offset: 0x44 (R/W  8) DPLL Control A */
  __I   uint8_t                        Reserved8[0x03];
  __IO  uint32_t                       SYSCTRL_DPLLRATIO;  /**< Offset: 0x48 (R/W  32) DPLL Ratio Control */
  __IO  uint32_t                       SYSCTRL_DPLLCTRLB;  /**< Offset: 0x4C (R/W  32) DPLL Control B */
  __I   uint8_t                        SYSCTRL_DPLLSTATUS; /**< Offset: 0x50 (R/   8) DPLL Status */
} sysctrl_registers_t;


#endif /* !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__)) */
#endif /* _SAMD21_SYSCTRL_COMPONENT_H_ */
