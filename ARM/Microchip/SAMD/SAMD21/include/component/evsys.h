/*
 * Component description for EVSYS
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
#ifndef _SAMD21_EVSYS_COMPONENT_H_
#define _SAMD21_EVSYS_COMPONENT_H_

/* ************************************************************************** */
/*   SOFTWARE API DEFINITION FOR EVSYS                                        */
/* ************************************************************************** */

/* -------- EVSYS_CTRL : (EVSYS Offset: 0x00) ( /W 8) Control -------- */
#define EVSYS_CTRL_RESETVALUE                 _UINT8_(0x00)                                        /*  (EVSYS_CTRL) Control  Reset Value */

#define EVSYS_CTRL_SWRST_Pos                  _UINT8_(0)                                           /* (EVSYS_CTRL) Software Reset Position */
#define EVSYS_CTRL_SWRST_Msk                  (_UINT8_(0x1) << EVSYS_CTRL_SWRST_Pos)               /* (EVSYS_CTRL) Software Reset Mask */
#define EVSYS_CTRL_SWRST(value)               (EVSYS_CTRL_SWRST_Msk & (_UINT8_(value) << EVSYS_CTRL_SWRST_Pos)) /* Assigment of value for SWRST in the EVSYS_CTRL register */
#define EVSYS_CTRL_GCLKREQ_Pos                _UINT8_(4)                                           /* (EVSYS_CTRL) Generic Clock Requests Position */
#define EVSYS_CTRL_GCLKREQ_Msk                (_UINT8_(0x1) << EVSYS_CTRL_GCLKREQ_Pos)             /* (EVSYS_CTRL) Generic Clock Requests Mask */
#define EVSYS_CTRL_GCLKREQ(value)             (EVSYS_CTRL_GCLKREQ_Msk & (_UINT8_(value) << EVSYS_CTRL_GCLKREQ_Pos)) /* Assigment of value for GCLKREQ in the EVSYS_CTRL register */
#define EVSYS_CTRL_Msk                        _UINT8_(0x11)                                        /* (EVSYS_CTRL) Register Mask  */


/* -------- EVSYS_CHANNEL : (EVSYS Offset: 0x04) (R/W 32) Channel -------- */
#define EVSYS_CHANNEL_RESETVALUE              _UINT32_(0x00)                                       /*  (EVSYS_CHANNEL) Channel  Reset Value */

#define EVSYS_CHANNEL_CHANNEL_Pos             _UINT32_(0)                                          /* (EVSYS_CHANNEL) Channel Selection Position */
#define EVSYS_CHANNEL_CHANNEL_Msk             (_UINT32_(0xF) << EVSYS_CHANNEL_CHANNEL_Pos)         /* (EVSYS_CHANNEL) Channel Selection Mask */
#define EVSYS_CHANNEL_CHANNEL(value)          (EVSYS_CHANNEL_CHANNEL_Msk & (_UINT32_(value) << EVSYS_CHANNEL_CHANNEL_Pos)) /* Assigment of value for CHANNEL in the EVSYS_CHANNEL register */
#define EVSYS_CHANNEL_SWEVT_Pos               _UINT32_(8)                                          /* (EVSYS_CHANNEL) Software Event Position */
#define EVSYS_CHANNEL_SWEVT_Msk               (_UINT32_(0x1) << EVSYS_CHANNEL_SWEVT_Pos)           /* (EVSYS_CHANNEL) Software Event Mask */
#define EVSYS_CHANNEL_SWEVT(value)            (EVSYS_CHANNEL_SWEVT_Msk & (_UINT32_(value) << EVSYS_CHANNEL_SWEVT_Pos)) /* Assigment of value for SWEVT in the EVSYS_CHANNEL register */
#define EVSYS_CHANNEL_EVGEN_Pos               _UINT32_(16)                                         /* (EVSYS_CHANNEL) Event Generator Selection Position */
#define EVSYS_CHANNEL_EVGEN_Msk               (_UINT32_(0x7F) << EVSYS_CHANNEL_EVGEN_Pos)          /* (EVSYS_CHANNEL) Event Generator Selection Mask */
#define EVSYS_CHANNEL_EVGEN(value)            (EVSYS_CHANNEL_EVGEN_Msk & (_UINT32_(value) << EVSYS_CHANNEL_EVGEN_Pos)) /* Assigment of value for EVGEN in the EVSYS_CHANNEL register */
#define EVSYS_CHANNEL_PATH_Pos                _UINT32_(24)                                         /* (EVSYS_CHANNEL) Path Selection Position */
#define EVSYS_CHANNEL_PATH_Msk                (_UINT32_(0x3) << EVSYS_CHANNEL_PATH_Pos)            /* (EVSYS_CHANNEL) Path Selection Mask */
#define EVSYS_CHANNEL_PATH(value)             (EVSYS_CHANNEL_PATH_Msk & (_UINT32_(value) << EVSYS_CHANNEL_PATH_Pos)) /* Assigment of value for PATH in the EVSYS_CHANNEL register */
#define   EVSYS_CHANNEL_PATH_SYNCHRONOUS_Val  _UINT32_(0x0)                                        /* (EVSYS_CHANNEL) Synchronous path  */
#define   EVSYS_CHANNEL_PATH_RESYNCHRONIZED_Val _UINT32_(0x1)                                        /* (EVSYS_CHANNEL) Resynchronized path  */
#define   EVSYS_CHANNEL_PATH_ASYNCHRONOUS_Val _UINT32_(0x2)                                        /* (EVSYS_CHANNEL) Asynchronous path  */
#define EVSYS_CHANNEL_PATH_SYNCHRONOUS        (EVSYS_CHANNEL_PATH_SYNCHRONOUS_Val << EVSYS_CHANNEL_PATH_Pos) /* (EVSYS_CHANNEL) Synchronous path Position  */
#define EVSYS_CHANNEL_PATH_RESYNCHRONIZED     (EVSYS_CHANNEL_PATH_RESYNCHRONIZED_Val << EVSYS_CHANNEL_PATH_Pos) /* (EVSYS_CHANNEL) Resynchronized path Position  */
#define EVSYS_CHANNEL_PATH_ASYNCHRONOUS       (EVSYS_CHANNEL_PATH_ASYNCHRONOUS_Val << EVSYS_CHANNEL_PATH_Pos) /* (EVSYS_CHANNEL) Asynchronous path Position  */
#define EVSYS_CHANNEL_EDGSEL_Pos              _UINT32_(26)                                         /* (EVSYS_CHANNEL) Edge Detection Selection Position */
#define EVSYS_CHANNEL_EDGSEL_Msk              (_UINT32_(0x3) << EVSYS_CHANNEL_EDGSEL_Pos)          /* (EVSYS_CHANNEL) Edge Detection Selection Mask */
#define EVSYS_CHANNEL_EDGSEL(value)           (EVSYS_CHANNEL_EDGSEL_Msk & (_UINT32_(value) << EVSYS_CHANNEL_EDGSEL_Pos)) /* Assigment of value for EDGSEL in the EVSYS_CHANNEL register */
#define   EVSYS_CHANNEL_EDGSEL_NO_EVT_OUTPUT_Val _UINT32_(0x0)                                        /* (EVSYS_CHANNEL) No event output when using the resynchronized or synchronous path  */
#define   EVSYS_CHANNEL_EDGSEL_RISING_EDGE_Val _UINT32_(0x1)                                        /* (EVSYS_CHANNEL) Event detection only on the rising edge of the signal from the event generator when using the resynchronized or synchronous path  */
#define   EVSYS_CHANNEL_EDGSEL_FALLING_EDGE_Val _UINT32_(0x2)                                        /* (EVSYS_CHANNEL) Event detection only on the falling edge of the signal from the event generator when using the resynchronized or synchronous path  */
#define   EVSYS_CHANNEL_EDGSEL_BOTH_EDGES_Val _UINT32_(0x3)                                        /* (EVSYS_CHANNEL) Event detection on rising and falling edges of the signal from the event generator when using the resynchronized or synchronous path  */
#define EVSYS_CHANNEL_EDGSEL_NO_EVT_OUTPUT    (EVSYS_CHANNEL_EDGSEL_NO_EVT_OUTPUT_Val << EVSYS_CHANNEL_EDGSEL_Pos) /* (EVSYS_CHANNEL) No event output when using the resynchronized or synchronous path Position  */
#define EVSYS_CHANNEL_EDGSEL_RISING_EDGE      (EVSYS_CHANNEL_EDGSEL_RISING_EDGE_Val << EVSYS_CHANNEL_EDGSEL_Pos) /* (EVSYS_CHANNEL) Event detection only on the rising edge of the signal from the event generator when using the resynchronized or synchronous path Position  */
#define EVSYS_CHANNEL_EDGSEL_FALLING_EDGE     (EVSYS_CHANNEL_EDGSEL_FALLING_EDGE_Val << EVSYS_CHANNEL_EDGSEL_Pos) /* (EVSYS_CHANNEL) Event detection only on the falling edge of the signal from the event generator when using the resynchronized or synchronous path Position  */
#define EVSYS_CHANNEL_EDGSEL_BOTH_EDGES       (EVSYS_CHANNEL_EDGSEL_BOTH_EDGES_Val << EVSYS_CHANNEL_EDGSEL_Pos) /* (EVSYS_CHANNEL) Event detection on rising and falling edges of the signal from the event generator when using the resynchronized or synchronous path Position  */
#define EVSYS_CHANNEL_Msk                     _UINT32_(0x0F7F010F)                                 /* (EVSYS_CHANNEL) Register Mask  */


/* -------- EVSYS_USER : (EVSYS Offset: 0x08) (R/W 16) User Multiplexer -------- */
#define EVSYS_USER_RESETVALUE                 _UINT16_(0x00)                                       /*  (EVSYS_USER) User Multiplexer  Reset Value */

#define EVSYS_USER_USER_Pos                   _UINT16_(0)                                          /* (EVSYS_USER) User Multiplexer Selection Position */
#define EVSYS_USER_USER_Msk                   (_UINT16_(0x1F) << EVSYS_USER_USER_Pos)              /* (EVSYS_USER) User Multiplexer Selection Mask */
#define EVSYS_USER_USER(value)                (EVSYS_USER_USER_Msk & (_UINT16_(value) << EVSYS_USER_USER_Pos)) /* Assigment of value for USER in the EVSYS_USER register */
#define EVSYS_USER_CHANNEL_Pos                _UINT16_(8)                                          /* (EVSYS_USER) Channel Event Selection Position */
#define EVSYS_USER_CHANNEL_Msk                (_UINT16_(0x1F) << EVSYS_USER_CHANNEL_Pos)           /* (EVSYS_USER) Channel Event Selection Mask */
#define EVSYS_USER_CHANNEL(value)             (EVSYS_USER_CHANNEL_Msk & (_UINT16_(value) << EVSYS_USER_CHANNEL_Pos)) /* Assigment of value for CHANNEL in the EVSYS_USER register */
#define   EVSYS_USER_CHANNEL_0_Val            _UINT16_(0x0)                                        /* (EVSYS_USER) No Channel Output Selected  */
#define EVSYS_USER_CHANNEL_0                  (EVSYS_USER_CHANNEL_0_Val << EVSYS_USER_CHANNEL_Pos) /* (EVSYS_USER) No Channel Output Selected Position  */
#define EVSYS_USER_Msk                        _UINT16_(0x1F1F)                                     /* (EVSYS_USER) Register Mask  */


/* -------- EVSYS_CHSTATUS : (EVSYS Offset: 0x0C) ( R/ 32) Channel Status -------- */
#define EVSYS_CHSTATUS_RESETVALUE             _UINT32_(0xF00FF)                                    /*  (EVSYS_CHSTATUS) Channel Status  Reset Value */

#define EVSYS_CHSTATUS_USRRDY0_Pos            _UINT32_(0)                                          /* (EVSYS_CHSTATUS) Channel 0 User Ready Position */
#define EVSYS_CHSTATUS_USRRDY0_Msk            (_UINT32_(0x1) << EVSYS_CHSTATUS_USRRDY0_Pos)        /* (EVSYS_CHSTATUS) Channel 0 User Ready Mask */
#define EVSYS_CHSTATUS_USRRDY0(value)         (EVSYS_CHSTATUS_USRRDY0_Msk & (_UINT32_(value) << EVSYS_CHSTATUS_USRRDY0_Pos)) /* Assigment of value for USRRDY0 in the EVSYS_CHSTATUS register */
#define EVSYS_CHSTATUS_USRRDY1_Pos            _UINT32_(1)                                          /* (EVSYS_CHSTATUS) Channel 1 User Ready Position */
#define EVSYS_CHSTATUS_USRRDY1_Msk            (_UINT32_(0x1) << EVSYS_CHSTATUS_USRRDY1_Pos)        /* (EVSYS_CHSTATUS) Channel 1 User Ready Mask */
#define EVSYS_CHSTATUS_USRRDY1(value)         (EVSYS_CHSTATUS_USRRDY1_Msk & (_UINT32_(value) << EVSYS_CHSTATUS_USRRDY1_Pos)) /* Assigment of value for USRRDY1 in the EVSYS_CHSTATUS register */
#define EVSYS_CHSTATUS_USRRDY2_Pos            _UINT32_(2)                                          /* (EVSYS_CHSTATUS) Channel 2 User Ready Position */
#define EVSYS_CHSTATUS_USRRDY2_Msk            (_UINT32_(0x1) << EVSYS_CHSTATUS_USRRDY2_Pos)        /* (EVSYS_CHSTATUS) Channel 2 User Ready Mask */
#define EVSYS_CHSTATUS_USRRDY2(value)         (EVSYS_CHSTATUS_USRRDY2_Msk & (_UINT32_(value) << EVSYS_CHSTATUS_USRRDY2_Pos)) /* Assigment of value for USRRDY2 in the EVSYS_CHSTATUS register */
#define EVSYS_CHSTATUS_USRRDY3_Pos            _UINT32_(3)                                          /* (EVSYS_CHSTATUS) Channel 3 User Ready Position */
#define EVSYS_CHSTATUS_USRRDY3_Msk            (_UINT32_(0x1) << EVSYS_CHSTATUS_USRRDY3_Pos)        /* (EVSYS_CHSTATUS) Channel 3 User Ready Mask */
#define EVSYS_CHSTATUS_USRRDY3(value)         (EVSYS_CHSTATUS_USRRDY3_Msk & (_UINT32_(value) << EVSYS_CHSTATUS_USRRDY3_Pos)) /* Assigment of value for USRRDY3 in the EVSYS_CHSTATUS register */
#define EVSYS_CHSTATUS_USRRDY4_Pos            _UINT32_(4)                                          /* (EVSYS_CHSTATUS) Channel 4 User Ready Position */
#define EVSYS_CHSTATUS_USRRDY4_Msk            (_UINT32_(0x1) << EVSYS_CHSTATUS_USRRDY4_Pos)        /* (EVSYS_CHSTATUS) Channel 4 User Ready Mask */
#define EVSYS_CHSTATUS_USRRDY4(value)         (EVSYS_CHSTATUS_USRRDY4_Msk & (_UINT32_(value) << EVSYS_CHSTATUS_USRRDY4_Pos)) /* Assigment of value for USRRDY4 in the EVSYS_CHSTATUS register */
#define EVSYS_CHSTATUS_USRRDY5_Pos            _UINT32_(5)                                          /* (EVSYS_CHSTATUS) Channel 5 User Ready Position */
#define EVSYS_CHSTATUS_USRRDY5_Msk            (_UINT32_(0x1) << EVSYS_CHSTATUS_USRRDY5_Pos)        /* (EVSYS_CHSTATUS) Channel 5 User Ready Mask */
#define EVSYS_CHSTATUS_USRRDY5(value)         (EVSYS_CHSTATUS_USRRDY5_Msk & (_UINT32_(value) << EVSYS_CHSTATUS_USRRDY5_Pos)) /* Assigment of value for USRRDY5 in the EVSYS_CHSTATUS register */
#define EVSYS_CHSTATUS_USRRDY6_Pos            _UINT32_(6)                                          /* (EVSYS_CHSTATUS) Channel 6 User Ready Position */
#define EVSYS_CHSTATUS_USRRDY6_Msk            (_UINT32_(0x1) << EVSYS_CHSTATUS_USRRDY6_Pos)        /* (EVSYS_CHSTATUS) Channel 6 User Ready Mask */
#define EVSYS_CHSTATUS_USRRDY6(value)         (EVSYS_CHSTATUS_USRRDY6_Msk & (_UINT32_(value) << EVSYS_CHSTATUS_USRRDY6_Pos)) /* Assigment of value for USRRDY6 in the EVSYS_CHSTATUS register */
#define EVSYS_CHSTATUS_USRRDY7_Pos            _UINT32_(7)                                          /* (EVSYS_CHSTATUS) Channel 7 User Ready Position */
#define EVSYS_CHSTATUS_USRRDY7_Msk            (_UINT32_(0x1) << EVSYS_CHSTATUS_USRRDY7_Pos)        /* (EVSYS_CHSTATUS) Channel 7 User Ready Mask */
#define EVSYS_CHSTATUS_USRRDY7(value)         (EVSYS_CHSTATUS_USRRDY7_Msk & (_UINT32_(value) << EVSYS_CHSTATUS_USRRDY7_Pos)) /* Assigment of value for USRRDY7 in the EVSYS_CHSTATUS register */
#define EVSYS_CHSTATUS_CHBUSY0_Pos            _UINT32_(8)                                          /* (EVSYS_CHSTATUS) Channel 0 Busy Position */
#define EVSYS_CHSTATUS_CHBUSY0_Msk            (_UINT32_(0x1) << EVSYS_CHSTATUS_CHBUSY0_Pos)        /* (EVSYS_CHSTATUS) Channel 0 Busy Mask */
#define EVSYS_CHSTATUS_CHBUSY0(value)         (EVSYS_CHSTATUS_CHBUSY0_Msk & (_UINT32_(value) << EVSYS_CHSTATUS_CHBUSY0_Pos)) /* Assigment of value for CHBUSY0 in the EVSYS_CHSTATUS register */
#define EVSYS_CHSTATUS_CHBUSY1_Pos            _UINT32_(9)                                          /* (EVSYS_CHSTATUS) Channel 1 Busy Position */
#define EVSYS_CHSTATUS_CHBUSY1_Msk            (_UINT32_(0x1) << EVSYS_CHSTATUS_CHBUSY1_Pos)        /* (EVSYS_CHSTATUS) Channel 1 Busy Mask */
#define EVSYS_CHSTATUS_CHBUSY1(value)         (EVSYS_CHSTATUS_CHBUSY1_Msk & (_UINT32_(value) << EVSYS_CHSTATUS_CHBUSY1_Pos)) /* Assigment of value for CHBUSY1 in the EVSYS_CHSTATUS register */
#define EVSYS_CHSTATUS_CHBUSY2_Pos            _UINT32_(10)                                         /* (EVSYS_CHSTATUS) Channel 2 Busy Position */
#define EVSYS_CHSTATUS_CHBUSY2_Msk            (_UINT32_(0x1) << EVSYS_CHSTATUS_CHBUSY2_Pos)        /* (EVSYS_CHSTATUS) Channel 2 Busy Mask */
#define EVSYS_CHSTATUS_CHBUSY2(value)         (EVSYS_CHSTATUS_CHBUSY2_Msk & (_UINT32_(value) << EVSYS_CHSTATUS_CHBUSY2_Pos)) /* Assigment of value for CHBUSY2 in the EVSYS_CHSTATUS register */
#define EVSYS_CHSTATUS_CHBUSY3_Pos            _UINT32_(11)                                         /* (EVSYS_CHSTATUS) Channel 3 Busy Position */
#define EVSYS_CHSTATUS_CHBUSY3_Msk            (_UINT32_(0x1) << EVSYS_CHSTATUS_CHBUSY3_Pos)        /* (EVSYS_CHSTATUS) Channel 3 Busy Mask */
#define EVSYS_CHSTATUS_CHBUSY3(value)         (EVSYS_CHSTATUS_CHBUSY3_Msk & (_UINT32_(value) << EVSYS_CHSTATUS_CHBUSY3_Pos)) /* Assigment of value for CHBUSY3 in the EVSYS_CHSTATUS register */
#define EVSYS_CHSTATUS_CHBUSY4_Pos            _UINT32_(12)                                         /* (EVSYS_CHSTATUS) Channel 4 Busy Position */
#define EVSYS_CHSTATUS_CHBUSY4_Msk            (_UINT32_(0x1) << EVSYS_CHSTATUS_CHBUSY4_Pos)        /* (EVSYS_CHSTATUS) Channel 4 Busy Mask */
#define EVSYS_CHSTATUS_CHBUSY4(value)         (EVSYS_CHSTATUS_CHBUSY4_Msk & (_UINT32_(value) << EVSYS_CHSTATUS_CHBUSY4_Pos)) /* Assigment of value for CHBUSY4 in the EVSYS_CHSTATUS register */
#define EVSYS_CHSTATUS_CHBUSY5_Pos            _UINT32_(13)                                         /* (EVSYS_CHSTATUS) Channel 5 Busy Position */
#define EVSYS_CHSTATUS_CHBUSY5_Msk            (_UINT32_(0x1) << EVSYS_CHSTATUS_CHBUSY5_Pos)        /* (EVSYS_CHSTATUS) Channel 5 Busy Mask */
#define EVSYS_CHSTATUS_CHBUSY5(value)         (EVSYS_CHSTATUS_CHBUSY5_Msk & (_UINT32_(value) << EVSYS_CHSTATUS_CHBUSY5_Pos)) /* Assigment of value for CHBUSY5 in the EVSYS_CHSTATUS register */
#define EVSYS_CHSTATUS_CHBUSY6_Pos            _UINT32_(14)                                         /* (EVSYS_CHSTATUS) Channel 6 Busy Position */
#define EVSYS_CHSTATUS_CHBUSY6_Msk            (_UINT32_(0x1) << EVSYS_CHSTATUS_CHBUSY6_Pos)        /* (EVSYS_CHSTATUS) Channel 6 Busy Mask */
#define EVSYS_CHSTATUS_CHBUSY6(value)         (EVSYS_CHSTATUS_CHBUSY6_Msk & (_UINT32_(value) << EVSYS_CHSTATUS_CHBUSY6_Pos)) /* Assigment of value for CHBUSY6 in the EVSYS_CHSTATUS register */
#define EVSYS_CHSTATUS_CHBUSY7_Pos            _UINT32_(15)                                         /* (EVSYS_CHSTATUS) Channel 7 Busy Position */
#define EVSYS_CHSTATUS_CHBUSY7_Msk            (_UINT32_(0x1) << EVSYS_CHSTATUS_CHBUSY7_Pos)        /* (EVSYS_CHSTATUS) Channel 7 Busy Mask */
#define EVSYS_CHSTATUS_CHBUSY7(value)         (EVSYS_CHSTATUS_CHBUSY7_Msk & (_UINT32_(value) << EVSYS_CHSTATUS_CHBUSY7_Pos)) /* Assigment of value for CHBUSY7 in the EVSYS_CHSTATUS register */
#define EVSYS_CHSTATUS_USRRDY8_Pos            _UINT32_(16)                                         /* (EVSYS_CHSTATUS) Channel 8 User Ready Position */
#define EVSYS_CHSTATUS_USRRDY8_Msk            (_UINT32_(0x1) << EVSYS_CHSTATUS_USRRDY8_Pos)        /* (EVSYS_CHSTATUS) Channel 8 User Ready Mask */
#define EVSYS_CHSTATUS_USRRDY8(value)         (EVSYS_CHSTATUS_USRRDY8_Msk & (_UINT32_(value) << EVSYS_CHSTATUS_USRRDY8_Pos)) /* Assigment of value for USRRDY8 in the EVSYS_CHSTATUS register */
#define EVSYS_CHSTATUS_USRRDY9_Pos            _UINT32_(17)                                         /* (EVSYS_CHSTATUS) Channel 9 User Ready Position */
#define EVSYS_CHSTATUS_USRRDY9_Msk            (_UINT32_(0x1) << EVSYS_CHSTATUS_USRRDY9_Pos)        /* (EVSYS_CHSTATUS) Channel 9 User Ready Mask */
#define EVSYS_CHSTATUS_USRRDY9(value)         (EVSYS_CHSTATUS_USRRDY9_Msk & (_UINT32_(value) << EVSYS_CHSTATUS_USRRDY9_Pos)) /* Assigment of value for USRRDY9 in the EVSYS_CHSTATUS register */
#define EVSYS_CHSTATUS_USRRDY10_Pos           _UINT32_(18)                                         /* (EVSYS_CHSTATUS) Channel 10 User Ready Position */
#define EVSYS_CHSTATUS_USRRDY10_Msk           (_UINT32_(0x1) << EVSYS_CHSTATUS_USRRDY10_Pos)       /* (EVSYS_CHSTATUS) Channel 10 User Ready Mask */
#define EVSYS_CHSTATUS_USRRDY10(value)        (EVSYS_CHSTATUS_USRRDY10_Msk & (_UINT32_(value) << EVSYS_CHSTATUS_USRRDY10_Pos)) /* Assigment of value for USRRDY10 in the EVSYS_CHSTATUS register */
#define EVSYS_CHSTATUS_USRRDY11_Pos           _UINT32_(19)                                         /* (EVSYS_CHSTATUS) Channel 11 User Ready Position */
#define EVSYS_CHSTATUS_USRRDY11_Msk           (_UINT32_(0x1) << EVSYS_CHSTATUS_USRRDY11_Pos)       /* (EVSYS_CHSTATUS) Channel 11 User Ready Mask */
#define EVSYS_CHSTATUS_USRRDY11(value)        (EVSYS_CHSTATUS_USRRDY11_Msk & (_UINT32_(value) << EVSYS_CHSTATUS_USRRDY11_Pos)) /* Assigment of value for USRRDY11 in the EVSYS_CHSTATUS register */
#define EVSYS_CHSTATUS_CHBUSY8_Pos            _UINT32_(24)                                         /* (EVSYS_CHSTATUS) Channel 8 Busy Position */
#define EVSYS_CHSTATUS_CHBUSY8_Msk            (_UINT32_(0x1) << EVSYS_CHSTATUS_CHBUSY8_Pos)        /* (EVSYS_CHSTATUS) Channel 8 Busy Mask */
#define EVSYS_CHSTATUS_CHBUSY8(value)         (EVSYS_CHSTATUS_CHBUSY8_Msk & (_UINT32_(value) << EVSYS_CHSTATUS_CHBUSY8_Pos)) /* Assigment of value for CHBUSY8 in the EVSYS_CHSTATUS register */
#define EVSYS_CHSTATUS_CHBUSY9_Pos            _UINT32_(25)                                         /* (EVSYS_CHSTATUS) Channel 9 Busy Position */
#define EVSYS_CHSTATUS_CHBUSY9_Msk            (_UINT32_(0x1) << EVSYS_CHSTATUS_CHBUSY9_Pos)        /* (EVSYS_CHSTATUS) Channel 9 Busy Mask */
#define EVSYS_CHSTATUS_CHBUSY9(value)         (EVSYS_CHSTATUS_CHBUSY9_Msk & (_UINT32_(value) << EVSYS_CHSTATUS_CHBUSY9_Pos)) /* Assigment of value for CHBUSY9 in the EVSYS_CHSTATUS register */
#define EVSYS_CHSTATUS_CHBUSY10_Pos           _UINT32_(26)                                         /* (EVSYS_CHSTATUS) Channel 10 Busy Position */
#define EVSYS_CHSTATUS_CHBUSY10_Msk           (_UINT32_(0x1) << EVSYS_CHSTATUS_CHBUSY10_Pos)       /* (EVSYS_CHSTATUS) Channel 10 Busy Mask */
#define EVSYS_CHSTATUS_CHBUSY10(value)        (EVSYS_CHSTATUS_CHBUSY10_Msk & (_UINT32_(value) << EVSYS_CHSTATUS_CHBUSY10_Pos)) /* Assigment of value for CHBUSY10 in the EVSYS_CHSTATUS register */
#define EVSYS_CHSTATUS_CHBUSY11_Pos           _UINT32_(27)                                         /* (EVSYS_CHSTATUS) Channel 11 Busy Position */
#define EVSYS_CHSTATUS_CHBUSY11_Msk           (_UINT32_(0x1) << EVSYS_CHSTATUS_CHBUSY11_Pos)       /* (EVSYS_CHSTATUS) Channel 11 Busy Mask */
#define EVSYS_CHSTATUS_CHBUSY11(value)        (EVSYS_CHSTATUS_CHBUSY11_Msk & (_UINT32_(value) << EVSYS_CHSTATUS_CHBUSY11_Pos)) /* Assigment of value for CHBUSY11 in the EVSYS_CHSTATUS register */
#define EVSYS_CHSTATUS_Msk                    _UINT32_(0x0F0FFFFF)                                 /* (EVSYS_CHSTATUS) Register Mask  */


/* -------- EVSYS_INTENCLR : (EVSYS Offset: 0x10) (R/W 32) Interrupt Enable Clear -------- */
#define EVSYS_INTENCLR_RESETVALUE             _UINT32_(0x00)                                       /*  (EVSYS_INTENCLR) Interrupt Enable Clear  Reset Value */

#define EVSYS_INTENCLR_OVR0_Pos               _UINT32_(0)                                          /* (EVSYS_INTENCLR) Channel 0 Overrun Interrupt Enable Position */
#define EVSYS_INTENCLR_OVR0_Msk               (_UINT32_(0x1) << EVSYS_INTENCLR_OVR0_Pos)           /* (EVSYS_INTENCLR) Channel 0 Overrun Interrupt Enable Mask */
#define EVSYS_INTENCLR_OVR0(value)            (EVSYS_INTENCLR_OVR0_Msk & (_UINT32_(value) << EVSYS_INTENCLR_OVR0_Pos)) /* Assigment of value for OVR0 in the EVSYS_INTENCLR register */
#define EVSYS_INTENCLR_OVR1_Pos               _UINT32_(1)                                          /* (EVSYS_INTENCLR) Channel 1 Overrun Interrupt Enable Position */
#define EVSYS_INTENCLR_OVR1_Msk               (_UINT32_(0x1) << EVSYS_INTENCLR_OVR1_Pos)           /* (EVSYS_INTENCLR) Channel 1 Overrun Interrupt Enable Mask */
#define EVSYS_INTENCLR_OVR1(value)            (EVSYS_INTENCLR_OVR1_Msk & (_UINT32_(value) << EVSYS_INTENCLR_OVR1_Pos)) /* Assigment of value for OVR1 in the EVSYS_INTENCLR register */
#define EVSYS_INTENCLR_OVR2_Pos               _UINT32_(2)                                          /* (EVSYS_INTENCLR) Channel 2 Overrun Interrupt Enable Position */
#define EVSYS_INTENCLR_OVR2_Msk               (_UINT32_(0x1) << EVSYS_INTENCLR_OVR2_Pos)           /* (EVSYS_INTENCLR) Channel 2 Overrun Interrupt Enable Mask */
#define EVSYS_INTENCLR_OVR2(value)            (EVSYS_INTENCLR_OVR2_Msk & (_UINT32_(value) << EVSYS_INTENCLR_OVR2_Pos)) /* Assigment of value for OVR2 in the EVSYS_INTENCLR register */
#define EVSYS_INTENCLR_OVR3_Pos               _UINT32_(3)                                          /* (EVSYS_INTENCLR) Channel 3 Overrun Interrupt Enable Position */
#define EVSYS_INTENCLR_OVR3_Msk               (_UINT32_(0x1) << EVSYS_INTENCLR_OVR3_Pos)           /* (EVSYS_INTENCLR) Channel 3 Overrun Interrupt Enable Mask */
#define EVSYS_INTENCLR_OVR3(value)            (EVSYS_INTENCLR_OVR3_Msk & (_UINT32_(value) << EVSYS_INTENCLR_OVR3_Pos)) /* Assigment of value for OVR3 in the EVSYS_INTENCLR register */
#define EVSYS_INTENCLR_OVR4_Pos               _UINT32_(4)                                          /* (EVSYS_INTENCLR) Channel 4 Overrun Interrupt Enable Position */
#define EVSYS_INTENCLR_OVR4_Msk               (_UINT32_(0x1) << EVSYS_INTENCLR_OVR4_Pos)           /* (EVSYS_INTENCLR) Channel 4 Overrun Interrupt Enable Mask */
#define EVSYS_INTENCLR_OVR4(value)            (EVSYS_INTENCLR_OVR4_Msk & (_UINT32_(value) << EVSYS_INTENCLR_OVR4_Pos)) /* Assigment of value for OVR4 in the EVSYS_INTENCLR register */
#define EVSYS_INTENCLR_OVR5_Pos               _UINT32_(5)                                          /* (EVSYS_INTENCLR) Channel 5 Overrun Interrupt Enable Position */
#define EVSYS_INTENCLR_OVR5_Msk               (_UINT32_(0x1) << EVSYS_INTENCLR_OVR5_Pos)           /* (EVSYS_INTENCLR) Channel 5 Overrun Interrupt Enable Mask */
#define EVSYS_INTENCLR_OVR5(value)            (EVSYS_INTENCLR_OVR5_Msk & (_UINT32_(value) << EVSYS_INTENCLR_OVR5_Pos)) /* Assigment of value for OVR5 in the EVSYS_INTENCLR register */
#define EVSYS_INTENCLR_OVR6_Pos               _UINT32_(6)                                          /* (EVSYS_INTENCLR) Channel 6 Overrun Interrupt Enable Position */
#define EVSYS_INTENCLR_OVR6_Msk               (_UINT32_(0x1) << EVSYS_INTENCLR_OVR6_Pos)           /* (EVSYS_INTENCLR) Channel 6 Overrun Interrupt Enable Mask */
#define EVSYS_INTENCLR_OVR6(value)            (EVSYS_INTENCLR_OVR6_Msk & (_UINT32_(value) << EVSYS_INTENCLR_OVR6_Pos)) /* Assigment of value for OVR6 in the EVSYS_INTENCLR register */
#define EVSYS_INTENCLR_OVR7_Pos               _UINT32_(7)                                          /* (EVSYS_INTENCLR) Channel 7 Overrun Interrupt Enable Position */
#define EVSYS_INTENCLR_OVR7_Msk               (_UINT32_(0x1) << EVSYS_INTENCLR_OVR7_Pos)           /* (EVSYS_INTENCLR) Channel 7 Overrun Interrupt Enable Mask */
#define EVSYS_INTENCLR_OVR7(value)            (EVSYS_INTENCLR_OVR7_Msk & (_UINT32_(value) << EVSYS_INTENCLR_OVR7_Pos)) /* Assigment of value for OVR7 in the EVSYS_INTENCLR register */
#define EVSYS_INTENCLR_EVD0_Pos               _UINT32_(8)                                          /* (EVSYS_INTENCLR) Channel 0 Event Detection Interrupt Enable Position */
#define EVSYS_INTENCLR_EVD0_Msk               (_UINT32_(0x1) << EVSYS_INTENCLR_EVD0_Pos)           /* (EVSYS_INTENCLR) Channel 0 Event Detection Interrupt Enable Mask */
#define EVSYS_INTENCLR_EVD0(value)            (EVSYS_INTENCLR_EVD0_Msk & (_UINT32_(value) << EVSYS_INTENCLR_EVD0_Pos)) /* Assigment of value for EVD0 in the EVSYS_INTENCLR register */
#define EVSYS_INTENCLR_EVD1_Pos               _UINT32_(9)                                          /* (EVSYS_INTENCLR) Channel 1 Event Detection Interrupt Enable Position */
#define EVSYS_INTENCLR_EVD1_Msk               (_UINT32_(0x1) << EVSYS_INTENCLR_EVD1_Pos)           /* (EVSYS_INTENCLR) Channel 1 Event Detection Interrupt Enable Mask */
#define EVSYS_INTENCLR_EVD1(value)            (EVSYS_INTENCLR_EVD1_Msk & (_UINT32_(value) << EVSYS_INTENCLR_EVD1_Pos)) /* Assigment of value for EVD1 in the EVSYS_INTENCLR register */
#define EVSYS_INTENCLR_EVD2_Pos               _UINT32_(10)                                         /* (EVSYS_INTENCLR) Channel 2 Event Detection Interrupt Enable Position */
#define EVSYS_INTENCLR_EVD2_Msk               (_UINT32_(0x1) << EVSYS_INTENCLR_EVD2_Pos)           /* (EVSYS_INTENCLR) Channel 2 Event Detection Interrupt Enable Mask */
#define EVSYS_INTENCLR_EVD2(value)            (EVSYS_INTENCLR_EVD2_Msk & (_UINT32_(value) << EVSYS_INTENCLR_EVD2_Pos)) /* Assigment of value for EVD2 in the EVSYS_INTENCLR register */
#define EVSYS_INTENCLR_EVD3_Pos               _UINT32_(11)                                         /* (EVSYS_INTENCLR) Channel 3 Event Detection Interrupt Enable Position */
#define EVSYS_INTENCLR_EVD3_Msk               (_UINT32_(0x1) << EVSYS_INTENCLR_EVD3_Pos)           /* (EVSYS_INTENCLR) Channel 3 Event Detection Interrupt Enable Mask */
#define EVSYS_INTENCLR_EVD3(value)            (EVSYS_INTENCLR_EVD3_Msk & (_UINT32_(value) << EVSYS_INTENCLR_EVD3_Pos)) /* Assigment of value for EVD3 in the EVSYS_INTENCLR register */
#define EVSYS_INTENCLR_EVD4_Pos               _UINT32_(12)                                         /* (EVSYS_INTENCLR) Channel 4 Event Detection Interrupt Enable Position */
#define EVSYS_INTENCLR_EVD4_Msk               (_UINT32_(0x1) << EVSYS_INTENCLR_EVD4_Pos)           /* (EVSYS_INTENCLR) Channel 4 Event Detection Interrupt Enable Mask */
#define EVSYS_INTENCLR_EVD4(value)            (EVSYS_INTENCLR_EVD4_Msk & (_UINT32_(value) << EVSYS_INTENCLR_EVD4_Pos)) /* Assigment of value for EVD4 in the EVSYS_INTENCLR register */
#define EVSYS_INTENCLR_EVD5_Pos               _UINT32_(13)                                         /* (EVSYS_INTENCLR) Channel 5 Event Detection Interrupt Enable Position */
#define EVSYS_INTENCLR_EVD5_Msk               (_UINT32_(0x1) << EVSYS_INTENCLR_EVD5_Pos)           /* (EVSYS_INTENCLR) Channel 5 Event Detection Interrupt Enable Mask */
#define EVSYS_INTENCLR_EVD5(value)            (EVSYS_INTENCLR_EVD5_Msk & (_UINT32_(value) << EVSYS_INTENCLR_EVD5_Pos)) /* Assigment of value for EVD5 in the EVSYS_INTENCLR register */
#define EVSYS_INTENCLR_EVD6_Pos               _UINT32_(14)                                         /* (EVSYS_INTENCLR) Channel 6 Event Detection Interrupt Enable Position */
#define EVSYS_INTENCLR_EVD6_Msk               (_UINT32_(0x1) << EVSYS_INTENCLR_EVD6_Pos)           /* (EVSYS_INTENCLR) Channel 6 Event Detection Interrupt Enable Mask */
#define EVSYS_INTENCLR_EVD6(value)            (EVSYS_INTENCLR_EVD6_Msk & (_UINT32_(value) << EVSYS_INTENCLR_EVD6_Pos)) /* Assigment of value for EVD6 in the EVSYS_INTENCLR register */
#define EVSYS_INTENCLR_EVD7_Pos               _UINT32_(15)                                         /* (EVSYS_INTENCLR) Channel 7 Event Detection Interrupt Enable Position */
#define EVSYS_INTENCLR_EVD7_Msk               (_UINT32_(0x1) << EVSYS_INTENCLR_EVD7_Pos)           /* (EVSYS_INTENCLR) Channel 7 Event Detection Interrupt Enable Mask */
#define EVSYS_INTENCLR_EVD7(value)            (EVSYS_INTENCLR_EVD7_Msk & (_UINT32_(value) << EVSYS_INTENCLR_EVD7_Pos)) /* Assigment of value for EVD7 in the EVSYS_INTENCLR register */
#define EVSYS_INTENCLR_OVR8_Pos               _UINT32_(16)                                         /* (EVSYS_INTENCLR) Channel 8 Overrun Interrupt Enable Position */
#define EVSYS_INTENCLR_OVR8_Msk               (_UINT32_(0x1) << EVSYS_INTENCLR_OVR8_Pos)           /* (EVSYS_INTENCLR) Channel 8 Overrun Interrupt Enable Mask */
#define EVSYS_INTENCLR_OVR8(value)            (EVSYS_INTENCLR_OVR8_Msk & (_UINT32_(value) << EVSYS_INTENCLR_OVR8_Pos)) /* Assigment of value for OVR8 in the EVSYS_INTENCLR register */
#define EVSYS_INTENCLR_OVR9_Pos               _UINT32_(17)                                         /* (EVSYS_INTENCLR) Channel 9 Overrun Interrupt Enable Position */
#define EVSYS_INTENCLR_OVR9_Msk               (_UINT32_(0x1) << EVSYS_INTENCLR_OVR9_Pos)           /* (EVSYS_INTENCLR) Channel 9 Overrun Interrupt Enable Mask */
#define EVSYS_INTENCLR_OVR9(value)            (EVSYS_INTENCLR_OVR9_Msk & (_UINT32_(value) << EVSYS_INTENCLR_OVR9_Pos)) /* Assigment of value for OVR9 in the EVSYS_INTENCLR register */
#define EVSYS_INTENCLR_OVR10_Pos              _UINT32_(18)                                         /* (EVSYS_INTENCLR) Channel 10 Overrun Interrupt Enable Position */
#define EVSYS_INTENCLR_OVR10_Msk              (_UINT32_(0x1) << EVSYS_INTENCLR_OVR10_Pos)          /* (EVSYS_INTENCLR) Channel 10 Overrun Interrupt Enable Mask */
#define EVSYS_INTENCLR_OVR10(value)           (EVSYS_INTENCLR_OVR10_Msk & (_UINT32_(value) << EVSYS_INTENCLR_OVR10_Pos)) /* Assigment of value for OVR10 in the EVSYS_INTENCLR register */
#define EVSYS_INTENCLR_OVR11_Pos              _UINT32_(19)                                         /* (EVSYS_INTENCLR) Channel 11 Overrun Interrupt Enable Position */
#define EVSYS_INTENCLR_OVR11_Msk              (_UINT32_(0x1) << EVSYS_INTENCLR_OVR11_Pos)          /* (EVSYS_INTENCLR) Channel 11 Overrun Interrupt Enable Mask */
#define EVSYS_INTENCLR_OVR11(value)           (EVSYS_INTENCLR_OVR11_Msk & (_UINT32_(value) << EVSYS_INTENCLR_OVR11_Pos)) /* Assigment of value for OVR11 in the EVSYS_INTENCLR register */
#define EVSYS_INTENCLR_EVD8_Pos               _UINT32_(24)                                         /* (EVSYS_INTENCLR) Channel 8 Event Detection Interrupt Enable Position */
#define EVSYS_INTENCLR_EVD8_Msk               (_UINT32_(0x1) << EVSYS_INTENCLR_EVD8_Pos)           /* (EVSYS_INTENCLR) Channel 8 Event Detection Interrupt Enable Mask */
#define EVSYS_INTENCLR_EVD8(value)            (EVSYS_INTENCLR_EVD8_Msk & (_UINT32_(value) << EVSYS_INTENCLR_EVD8_Pos)) /* Assigment of value for EVD8 in the EVSYS_INTENCLR register */
#define EVSYS_INTENCLR_EVD9_Pos               _UINT32_(25)                                         /* (EVSYS_INTENCLR) Channel 9 Event Detection Interrupt Enable Position */
#define EVSYS_INTENCLR_EVD9_Msk               (_UINT32_(0x1) << EVSYS_INTENCLR_EVD9_Pos)           /* (EVSYS_INTENCLR) Channel 9 Event Detection Interrupt Enable Mask */
#define EVSYS_INTENCLR_EVD9(value)            (EVSYS_INTENCLR_EVD9_Msk & (_UINT32_(value) << EVSYS_INTENCLR_EVD9_Pos)) /* Assigment of value for EVD9 in the EVSYS_INTENCLR register */
#define EVSYS_INTENCLR_EVD10_Pos              _UINT32_(26)                                         /* (EVSYS_INTENCLR) Channel 10 Event Detection Interrupt Enable Position */
#define EVSYS_INTENCLR_EVD10_Msk              (_UINT32_(0x1) << EVSYS_INTENCLR_EVD10_Pos)          /* (EVSYS_INTENCLR) Channel 10 Event Detection Interrupt Enable Mask */
#define EVSYS_INTENCLR_EVD10(value)           (EVSYS_INTENCLR_EVD10_Msk & (_UINT32_(value) << EVSYS_INTENCLR_EVD10_Pos)) /* Assigment of value for EVD10 in the EVSYS_INTENCLR register */
#define EVSYS_INTENCLR_EVD11_Pos              _UINT32_(27)                                         /* (EVSYS_INTENCLR) Channel 11 Event Detection Interrupt Enable Position */
#define EVSYS_INTENCLR_EVD11_Msk              (_UINT32_(0x1) << EVSYS_INTENCLR_EVD11_Pos)          /* (EVSYS_INTENCLR) Channel 11 Event Detection Interrupt Enable Mask */
#define EVSYS_INTENCLR_EVD11(value)           (EVSYS_INTENCLR_EVD11_Msk & (_UINT32_(value) << EVSYS_INTENCLR_EVD11_Pos)) /* Assigment of value for EVD11 in the EVSYS_INTENCLR register */
#define EVSYS_INTENCLR_Msk                    _UINT32_(0x0F0FFFFF)                                 /* (EVSYS_INTENCLR) Register Mask  */


/* -------- EVSYS_INTENSET : (EVSYS Offset: 0x14) (R/W 32) Interrupt Enable Set -------- */
#define EVSYS_INTENSET_RESETVALUE             _UINT32_(0x00)                                       /*  (EVSYS_INTENSET) Interrupt Enable Set  Reset Value */

#define EVSYS_INTENSET_OVR0_Pos               _UINT32_(0)                                          /* (EVSYS_INTENSET) Channel 0 Overrun Interrupt Enable Position */
#define EVSYS_INTENSET_OVR0_Msk               (_UINT32_(0x1) << EVSYS_INTENSET_OVR0_Pos)           /* (EVSYS_INTENSET) Channel 0 Overrun Interrupt Enable Mask */
#define EVSYS_INTENSET_OVR0(value)            (EVSYS_INTENSET_OVR0_Msk & (_UINT32_(value) << EVSYS_INTENSET_OVR0_Pos)) /* Assigment of value for OVR0 in the EVSYS_INTENSET register */
#define EVSYS_INTENSET_OVR1_Pos               _UINT32_(1)                                          /* (EVSYS_INTENSET) Channel 1 Overrun Interrupt Enable Position */
#define EVSYS_INTENSET_OVR1_Msk               (_UINT32_(0x1) << EVSYS_INTENSET_OVR1_Pos)           /* (EVSYS_INTENSET) Channel 1 Overrun Interrupt Enable Mask */
#define EVSYS_INTENSET_OVR1(value)            (EVSYS_INTENSET_OVR1_Msk & (_UINT32_(value) << EVSYS_INTENSET_OVR1_Pos)) /* Assigment of value for OVR1 in the EVSYS_INTENSET register */
#define EVSYS_INTENSET_OVR2_Pos               _UINT32_(2)                                          /* (EVSYS_INTENSET) Channel 2 Overrun Interrupt Enable Position */
#define EVSYS_INTENSET_OVR2_Msk               (_UINT32_(0x1) << EVSYS_INTENSET_OVR2_Pos)           /* (EVSYS_INTENSET) Channel 2 Overrun Interrupt Enable Mask */
#define EVSYS_INTENSET_OVR2(value)            (EVSYS_INTENSET_OVR2_Msk & (_UINT32_(value) << EVSYS_INTENSET_OVR2_Pos)) /* Assigment of value for OVR2 in the EVSYS_INTENSET register */
#define EVSYS_INTENSET_OVR3_Pos               _UINT32_(3)                                          /* (EVSYS_INTENSET) Channel 3 Overrun Interrupt Enable Position */
#define EVSYS_INTENSET_OVR3_Msk               (_UINT32_(0x1) << EVSYS_INTENSET_OVR3_Pos)           /* (EVSYS_INTENSET) Channel 3 Overrun Interrupt Enable Mask */
#define EVSYS_INTENSET_OVR3(value)            (EVSYS_INTENSET_OVR3_Msk & (_UINT32_(value) << EVSYS_INTENSET_OVR3_Pos)) /* Assigment of value for OVR3 in the EVSYS_INTENSET register */
#define EVSYS_INTENSET_OVR4_Pos               _UINT32_(4)                                          /* (EVSYS_INTENSET) Channel 4 Overrun Interrupt Enable Position */
#define EVSYS_INTENSET_OVR4_Msk               (_UINT32_(0x1) << EVSYS_INTENSET_OVR4_Pos)           /* (EVSYS_INTENSET) Channel 4 Overrun Interrupt Enable Mask */
#define EVSYS_INTENSET_OVR4(value)            (EVSYS_INTENSET_OVR4_Msk & (_UINT32_(value) << EVSYS_INTENSET_OVR4_Pos)) /* Assigment of value for OVR4 in the EVSYS_INTENSET register */
#define EVSYS_INTENSET_OVR5_Pos               _UINT32_(5)                                          /* (EVSYS_INTENSET) Channel 5 Overrun Interrupt Enable Position */
#define EVSYS_INTENSET_OVR5_Msk               (_UINT32_(0x1) << EVSYS_INTENSET_OVR5_Pos)           /* (EVSYS_INTENSET) Channel 5 Overrun Interrupt Enable Mask */
#define EVSYS_INTENSET_OVR5(value)            (EVSYS_INTENSET_OVR5_Msk & (_UINT32_(value) << EVSYS_INTENSET_OVR5_Pos)) /* Assigment of value for OVR5 in the EVSYS_INTENSET register */
#define EVSYS_INTENSET_OVR6_Pos               _UINT32_(6)                                          /* (EVSYS_INTENSET) Channel 6 Overrun Interrupt Enable Position */
#define EVSYS_INTENSET_OVR6_Msk               (_UINT32_(0x1) << EVSYS_INTENSET_OVR6_Pos)           /* (EVSYS_INTENSET) Channel 6 Overrun Interrupt Enable Mask */
#define EVSYS_INTENSET_OVR6(value)            (EVSYS_INTENSET_OVR6_Msk & (_UINT32_(value) << EVSYS_INTENSET_OVR6_Pos)) /* Assigment of value for OVR6 in the EVSYS_INTENSET register */
#define EVSYS_INTENSET_OVR7_Pos               _UINT32_(7)                                          /* (EVSYS_INTENSET) Channel 7 Overrun Interrupt Enable Position */
#define EVSYS_INTENSET_OVR7_Msk               (_UINT32_(0x1) << EVSYS_INTENSET_OVR7_Pos)           /* (EVSYS_INTENSET) Channel 7 Overrun Interrupt Enable Mask */
#define EVSYS_INTENSET_OVR7(value)            (EVSYS_INTENSET_OVR7_Msk & (_UINT32_(value) << EVSYS_INTENSET_OVR7_Pos)) /* Assigment of value for OVR7 in the EVSYS_INTENSET register */
#define EVSYS_INTENSET_EVD0_Pos               _UINT32_(8)                                          /* (EVSYS_INTENSET) Channel 0 Event Detection Interrupt Enable Position */
#define EVSYS_INTENSET_EVD0_Msk               (_UINT32_(0x1) << EVSYS_INTENSET_EVD0_Pos)           /* (EVSYS_INTENSET) Channel 0 Event Detection Interrupt Enable Mask */
#define EVSYS_INTENSET_EVD0(value)            (EVSYS_INTENSET_EVD0_Msk & (_UINT32_(value) << EVSYS_INTENSET_EVD0_Pos)) /* Assigment of value for EVD0 in the EVSYS_INTENSET register */
#define EVSYS_INTENSET_EVD1_Pos               _UINT32_(9)                                          /* (EVSYS_INTENSET) Channel 1 Event Detection Interrupt Enable Position */
#define EVSYS_INTENSET_EVD1_Msk               (_UINT32_(0x1) << EVSYS_INTENSET_EVD1_Pos)           /* (EVSYS_INTENSET) Channel 1 Event Detection Interrupt Enable Mask */
#define EVSYS_INTENSET_EVD1(value)            (EVSYS_INTENSET_EVD1_Msk & (_UINT32_(value) << EVSYS_INTENSET_EVD1_Pos)) /* Assigment of value for EVD1 in the EVSYS_INTENSET register */
#define EVSYS_INTENSET_EVD2_Pos               _UINT32_(10)                                         /* (EVSYS_INTENSET) Channel 2 Event Detection Interrupt Enable Position */
#define EVSYS_INTENSET_EVD2_Msk               (_UINT32_(0x1) << EVSYS_INTENSET_EVD2_Pos)           /* (EVSYS_INTENSET) Channel 2 Event Detection Interrupt Enable Mask */
#define EVSYS_INTENSET_EVD2(value)            (EVSYS_INTENSET_EVD2_Msk & (_UINT32_(value) << EVSYS_INTENSET_EVD2_Pos)) /* Assigment of value for EVD2 in the EVSYS_INTENSET register */
#define EVSYS_INTENSET_EVD3_Pos               _UINT32_(11)                                         /* (EVSYS_INTENSET) Channel 3 Event Detection Interrupt Enable Position */
#define EVSYS_INTENSET_EVD3_Msk               (_UINT32_(0x1) << EVSYS_INTENSET_EVD3_Pos)           /* (EVSYS_INTENSET) Channel 3 Event Detection Interrupt Enable Mask */
#define EVSYS_INTENSET_EVD3(value)            (EVSYS_INTENSET_EVD3_Msk & (_UINT32_(value) << EVSYS_INTENSET_EVD3_Pos)) /* Assigment of value for EVD3 in the EVSYS_INTENSET register */
#define EVSYS_INTENSET_EVD4_Pos               _UINT32_(12)                                         /* (EVSYS_INTENSET) Channel 4 Event Detection Interrupt Enable Position */
#define EVSYS_INTENSET_EVD4_Msk               (_UINT32_(0x1) << EVSYS_INTENSET_EVD4_Pos)           /* (EVSYS_INTENSET) Channel 4 Event Detection Interrupt Enable Mask */
#define EVSYS_INTENSET_EVD4(value)            (EVSYS_INTENSET_EVD4_Msk & (_UINT32_(value) << EVSYS_INTENSET_EVD4_Pos)) /* Assigment of value for EVD4 in the EVSYS_INTENSET register */
#define EVSYS_INTENSET_EVD5_Pos               _UINT32_(13)                                         /* (EVSYS_INTENSET) Channel 5 Event Detection Interrupt Enable Position */
#define EVSYS_INTENSET_EVD5_Msk               (_UINT32_(0x1) << EVSYS_INTENSET_EVD5_Pos)           /* (EVSYS_INTENSET) Channel 5 Event Detection Interrupt Enable Mask */
#define EVSYS_INTENSET_EVD5(value)            (EVSYS_INTENSET_EVD5_Msk & (_UINT32_(value) << EVSYS_INTENSET_EVD5_Pos)) /* Assigment of value for EVD5 in the EVSYS_INTENSET register */
#define EVSYS_INTENSET_EVD6_Pos               _UINT32_(14)                                         /* (EVSYS_INTENSET) Channel 6 Event Detection Interrupt Enable Position */
#define EVSYS_INTENSET_EVD6_Msk               (_UINT32_(0x1) << EVSYS_INTENSET_EVD6_Pos)           /* (EVSYS_INTENSET) Channel 6 Event Detection Interrupt Enable Mask */
#define EVSYS_INTENSET_EVD6(value)            (EVSYS_INTENSET_EVD6_Msk & (_UINT32_(value) << EVSYS_INTENSET_EVD6_Pos)) /* Assigment of value for EVD6 in the EVSYS_INTENSET register */
#define EVSYS_INTENSET_EVD7_Pos               _UINT32_(15)                                         /* (EVSYS_INTENSET) Channel 7 Event Detection Interrupt Enable Position */
#define EVSYS_INTENSET_EVD7_Msk               (_UINT32_(0x1) << EVSYS_INTENSET_EVD7_Pos)           /* (EVSYS_INTENSET) Channel 7 Event Detection Interrupt Enable Mask */
#define EVSYS_INTENSET_EVD7(value)            (EVSYS_INTENSET_EVD7_Msk & (_UINT32_(value) << EVSYS_INTENSET_EVD7_Pos)) /* Assigment of value for EVD7 in the EVSYS_INTENSET register */
#define EVSYS_INTENSET_OVR8_Pos               _UINT32_(16)                                         /* (EVSYS_INTENSET) Channel 8 Overrun Interrupt Enable Position */
#define EVSYS_INTENSET_OVR8_Msk               (_UINT32_(0x1) << EVSYS_INTENSET_OVR8_Pos)           /* (EVSYS_INTENSET) Channel 8 Overrun Interrupt Enable Mask */
#define EVSYS_INTENSET_OVR8(value)            (EVSYS_INTENSET_OVR8_Msk & (_UINT32_(value) << EVSYS_INTENSET_OVR8_Pos)) /* Assigment of value for OVR8 in the EVSYS_INTENSET register */
#define EVSYS_INTENSET_OVR9_Pos               _UINT32_(17)                                         /* (EVSYS_INTENSET) Channel 9 Overrun Interrupt Enable Position */
#define EVSYS_INTENSET_OVR9_Msk               (_UINT32_(0x1) << EVSYS_INTENSET_OVR9_Pos)           /* (EVSYS_INTENSET) Channel 9 Overrun Interrupt Enable Mask */
#define EVSYS_INTENSET_OVR9(value)            (EVSYS_INTENSET_OVR9_Msk & (_UINT32_(value) << EVSYS_INTENSET_OVR9_Pos)) /* Assigment of value for OVR9 in the EVSYS_INTENSET register */
#define EVSYS_INTENSET_OVR10_Pos              _UINT32_(18)                                         /* (EVSYS_INTENSET) Channel 10 Overrun Interrupt Enable Position */
#define EVSYS_INTENSET_OVR10_Msk              (_UINT32_(0x1) << EVSYS_INTENSET_OVR10_Pos)          /* (EVSYS_INTENSET) Channel 10 Overrun Interrupt Enable Mask */
#define EVSYS_INTENSET_OVR10(value)           (EVSYS_INTENSET_OVR10_Msk & (_UINT32_(value) << EVSYS_INTENSET_OVR10_Pos)) /* Assigment of value for OVR10 in the EVSYS_INTENSET register */
#define EVSYS_INTENSET_OVR11_Pos              _UINT32_(19)                                         /* (EVSYS_INTENSET) Channel 11 Overrun Interrupt Enable Position */
#define EVSYS_INTENSET_OVR11_Msk              (_UINT32_(0x1) << EVSYS_INTENSET_OVR11_Pos)          /* (EVSYS_INTENSET) Channel 11 Overrun Interrupt Enable Mask */
#define EVSYS_INTENSET_OVR11(value)           (EVSYS_INTENSET_OVR11_Msk & (_UINT32_(value) << EVSYS_INTENSET_OVR11_Pos)) /* Assigment of value for OVR11 in the EVSYS_INTENSET register */
#define EVSYS_INTENSET_EVD8_Pos               _UINT32_(24)                                         /* (EVSYS_INTENSET) Channel 8 Event Detection Interrupt Enable Position */
#define EVSYS_INTENSET_EVD8_Msk               (_UINT32_(0x1) << EVSYS_INTENSET_EVD8_Pos)           /* (EVSYS_INTENSET) Channel 8 Event Detection Interrupt Enable Mask */
#define EVSYS_INTENSET_EVD8(value)            (EVSYS_INTENSET_EVD8_Msk & (_UINT32_(value) << EVSYS_INTENSET_EVD8_Pos)) /* Assigment of value for EVD8 in the EVSYS_INTENSET register */
#define EVSYS_INTENSET_EVD9_Pos               _UINT32_(25)                                         /* (EVSYS_INTENSET) Channel 9 Event Detection Interrupt Enable Position */
#define EVSYS_INTENSET_EVD9_Msk               (_UINT32_(0x1) << EVSYS_INTENSET_EVD9_Pos)           /* (EVSYS_INTENSET) Channel 9 Event Detection Interrupt Enable Mask */
#define EVSYS_INTENSET_EVD9(value)            (EVSYS_INTENSET_EVD9_Msk & (_UINT32_(value) << EVSYS_INTENSET_EVD9_Pos)) /* Assigment of value for EVD9 in the EVSYS_INTENSET register */
#define EVSYS_INTENSET_EVD10_Pos              _UINT32_(26)                                         /* (EVSYS_INTENSET) Channel 10 Event Detection Interrupt Enable Position */
#define EVSYS_INTENSET_EVD10_Msk              (_UINT32_(0x1) << EVSYS_INTENSET_EVD10_Pos)          /* (EVSYS_INTENSET) Channel 10 Event Detection Interrupt Enable Mask */
#define EVSYS_INTENSET_EVD10(value)           (EVSYS_INTENSET_EVD10_Msk & (_UINT32_(value) << EVSYS_INTENSET_EVD10_Pos)) /* Assigment of value for EVD10 in the EVSYS_INTENSET register */
#define EVSYS_INTENSET_EVD11_Pos              _UINT32_(27)                                         /* (EVSYS_INTENSET) Channel 11 Event Detection Interrupt Enable Position */
#define EVSYS_INTENSET_EVD11_Msk              (_UINT32_(0x1) << EVSYS_INTENSET_EVD11_Pos)          /* (EVSYS_INTENSET) Channel 11 Event Detection Interrupt Enable Mask */
#define EVSYS_INTENSET_EVD11(value)           (EVSYS_INTENSET_EVD11_Msk & (_UINT32_(value) << EVSYS_INTENSET_EVD11_Pos)) /* Assigment of value for EVD11 in the EVSYS_INTENSET register */
#define EVSYS_INTENSET_Msk                    _UINT32_(0x0F0FFFFF)                                 /* (EVSYS_INTENSET) Register Mask  */


/* -------- EVSYS_INTFLAG : (EVSYS Offset: 0x18) (R/W 32) Interrupt Flag Status and Clear -------- */
#define EVSYS_INTFLAG_RESETVALUE              _UINT32_(0x00)                                       /*  (EVSYS_INTFLAG) Interrupt Flag Status and Clear  Reset Value */

#define EVSYS_INTFLAG_OVR0_Pos                _UINT32_(0)                                          /* (EVSYS_INTFLAG) Channel 0 Overrun Position */
#define EVSYS_INTFLAG_OVR0_Msk                (_UINT32_(0x1) << EVSYS_INTFLAG_OVR0_Pos)            /* (EVSYS_INTFLAG) Channel 0 Overrun Mask */
#define EVSYS_INTFLAG_OVR0(value)             (EVSYS_INTFLAG_OVR0_Msk & (_UINT32_(value) << EVSYS_INTFLAG_OVR0_Pos)) /* Assigment of value for OVR0 in the EVSYS_INTFLAG register */
#define EVSYS_INTFLAG_OVR1_Pos                _UINT32_(1)                                          /* (EVSYS_INTFLAG) Channel 1 Overrun Position */
#define EVSYS_INTFLAG_OVR1_Msk                (_UINT32_(0x1) << EVSYS_INTFLAG_OVR1_Pos)            /* (EVSYS_INTFLAG) Channel 1 Overrun Mask */
#define EVSYS_INTFLAG_OVR1(value)             (EVSYS_INTFLAG_OVR1_Msk & (_UINT32_(value) << EVSYS_INTFLAG_OVR1_Pos)) /* Assigment of value for OVR1 in the EVSYS_INTFLAG register */
#define EVSYS_INTFLAG_OVR2_Pos                _UINT32_(2)                                          /* (EVSYS_INTFLAG) Channel 2 Overrun Position */
#define EVSYS_INTFLAG_OVR2_Msk                (_UINT32_(0x1) << EVSYS_INTFLAG_OVR2_Pos)            /* (EVSYS_INTFLAG) Channel 2 Overrun Mask */
#define EVSYS_INTFLAG_OVR2(value)             (EVSYS_INTFLAG_OVR2_Msk & (_UINT32_(value) << EVSYS_INTFLAG_OVR2_Pos)) /* Assigment of value for OVR2 in the EVSYS_INTFLAG register */
#define EVSYS_INTFLAG_OVR3_Pos                _UINT32_(3)                                          /* (EVSYS_INTFLAG) Channel 3 Overrun Position */
#define EVSYS_INTFLAG_OVR3_Msk                (_UINT32_(0x1) << EVSYS_INTFLAG_OVR3_Pos)            /* (EVSYS_INTFLAG) Channel 3 Overrun Mask */
#define EVSYS_INTFLAG_OVR3(value)             (EVSYS_INTFLAG_OVR3_Msk & (_UINT32_(value) << EVSYS_INTFLAG_OVR3_Pos)) /* Assigment of value for OVR3 in the EVSYS_INTFLAG register */
#define EVSYS_INTFLAG_OVR4_Pos                _UINT32_(4)                                          /* (EVSYS_INTFLAG) Channel 4 Overrun Position */
#define EVSYS_INTFLAG_OVR4_Msk                (_UINT32_(0x1) << EVSYS_INTFLAG_OVR4_Pos)            /* (EVSYS_INTFLAG) Channel 4 Overrun Mask */
#define EVSYS_INTFLAG_OVR4(value)             (EVSYS_INTFLAG_OVR4_Msk & (_UINT32_(value) << EVSYS_INTFLAG_OVR4_Pos)) /* Assigment of value for OVR4 in the EVSYS_INTFLAG register */
#define EVSYS_INTFLAG_OVR5_Pos                _UINT32_(5)                                          /* (EVSYS_INTFLAG) Channel 5 Overrun Position */
#define EVSYS_INTFLAG_OVR5_Msk                (_UINT32_(0x1) << EVSYS_INTFLAG_OVR5_Pos)            /* (EVSYS_INTFLAG) Channel 5 Overrun Mask */
#define EVSYS_INTFLAG_OVR5(value)             (EVSYS_INTFLAG_OVR5_Msk & (_UINT32_(value) << EVSYS_INTFLAG_OVR5_Pos)) /* Assigment of value for OVR5 in the EVSYS_INTFLAG register */
#define EVSYS_INTFLAG_OVR6_Pos                _UINT32_(6)                                          /* (EVSYS_INTFLAG) Channel 6 Overrun Position */
#define EVSYS_INTFLAG_OVR6_Msk                (_UINT32_(0x1) << EVSYS_INTFLAG_OVR6_Pos)            /* (EVSYS_INTFLAG) Channel 6 Overrun Mask */
#define EVSYS_INTFLAG_OVR6(value)             (EVSYS_INTFLAG_OVR6_Msk & (_UINT32_(value) << EVSYS_INTFLAG_OVR6_Pos)) /* Assigment of value for OVR6 in the EVSYS_INTFLAG register */
#define EVSYS_INTFLAG_OVR7_Pos                _UINT32_(7)                                          /* (EVSYS_INTFLAG) Channel 7 Overrun Position */
#define EVSYS_INTFLAG_OVR7_Msk                (_UINT32_(0x1) << EVSYS_INTFLAG_OVR7_Pos)            /* (EVSYS_INTFLAG) Channel 7 Overrun Mask */
#define EVSYS_INTFLAG_OVR7(value)             (EVSYS_INTFLAG_OVR7_Msk & (_UINT32_(value) << EVSYS_INTFLAG_OVR7_Pos)) /* Assigment of value for OVR7 in the EVSYS_INTFLAG register */
#define EVSYS_INTFLAG_EVD0_Pos                _UINT32_(8)                                          /* (EVSYS_INTFLAG) Channel 0 Event Detection Position */
#define EVSYS_INTFLAG_EVD0_Msk                (_UINT32_(0x1) << EVSYS_INTFLAG_EVD0_Pos)            /* (EVSYS_INTFLAG) Channel 0 Event Detection Mask */
#define EVSYS_INTFLAG_EVD0(value)             (EVSYS_INTFLAG_EVD0_Msk & (_UINT32_(value) << EVSYS_INTFLAG_EVD0_Pos)) /* Assigment of value for EVD0 in the EVSYS_INTFLAG register */
#define EVSYS_INTFLAG_EVD1_Pos                _UINT32_(9)                                          /* (EVSYS_INTFLAG) Channel 1 Event Detection Position */
#define EVSYS_INTFLAG_EVD1_Msk                (_UINT32_(0x1) << EVSYS_INTFLAG_EVD1_Pos)            /* (EVSYS_INTFLAG) Channel 1 Event Detection Mask */
#define EVSYS_INTFLAG_EVD1(value)             (EVSYS_INTFLAG_EVD1_Msk & (_UINT32_(value) << EVSYS_INTFLAG_EVD1_Pos)) /* Assigment of value for EVD1 in the EVSYS_INTFLAG register */
#define EVSYS_INTFLAG_EVD2_Pos                _UINT32_(10)                                         /* (EVSYS_INTFLAG) Channel 2 Event Detection Position */
#define EVSYS_INTFLAG_EVD2_Msk                (_UINT32_(0x1) << EVSYS_INTFLAG_EVD2_Pos)            /* (EVSYS_INTFLAG) Channel 2 Event Detection Mask */
#define EVSYS_INTFLAG_EVD2(value)             (EVSYS_INTFLAG_EVD2_Msk & (_UINT32_(value) << EVSYS_INTFLAG_EVD2_Pos)) /* Assigment of value for EVD2 in the EVSYS_INTFLAG register */
#define EVSYS_INTFLAG_EVD3_Pos                _UINT32_(11)                                         /* (EVSYS_INTFLAG) Channel 3 Event Detection Position */
#define EVSYS_INTFLAG_EVD3_Msk                (_UINT32_(0x1) << EVSYS_INTFLAG_EVD3_Pos)            /* (EVSYS_INTFLAG) Channel 3 Event Detection Mask */
#define EVSYS_INTFLAG_EVD3(value)             (EVSYS_INTFLAG_EVD3_Msk & (_UINT32_(value) << EVSYS_INTFLAG_EVD3_Pos)) /* Assigment of value for EVD3 in the EVSYS_INTFLAG register */
#define EVSYS_INTFLAG_EVD4_Pos                _UINT32_(12)                                         /* (EVSYS_INTFLAG) Channel 4 Event Detection Position */
#define EVSYS_INTFLAG_EVD4_Msk                (_UINT32_(0x1) << EVSYS_INTFLAG_EVD4_Pos)            /* (EVSYS_INTFLAG) Channel 4 Event Detection Mask */
#define EVSYS_INTFLAG_EVD4(value)             (EVSYS_INTFLAG_EVD4_Msk & (_UINT32_(value) << EVSYS_INTFLAG_EVD4_Pos)) /* Assigment of value for EVD4 in the EVSYS_INTFLAG register */
#define EVSYS_INTFLAG_EVD5_Pos                _UINT32_(13)                                         /* (EVSYS_INTFLAG) Channel 5 Event Detection Position */
#define EVSYS_INTFLAG_EVD5_Msk                (_UINT32_(0x1) << EVSYS_INTFLAG_EVD5_Pos)            /* (EVSYS_INTFLAG) Channel 5 Event Detection Mask */
#define EVSYS_INTFLAG_EVD5(value)             (EVSYS_INTFLAG_EVD5_Msk & (_UINT32_(value) << EVSYS_INTFLAG_EVD5_Pos)) /* Assigment of value for EVD5 in the EVSYS_INTFLAG register */
#define EVSYS_INTFLAG_EVD6_Pos                _UINT32_(14)                                         /* (EVSYS_INTFLAG) Channel 6 Event Detection Position */
#define EVSYS_INTFLAG_EVD6_Msk                (_UINT32_(0x1) << EVSYS_INTFLAG_EVD6_Pos)            /* (EVSYS_INTFLAG) Channel 6 Event Detection Mask */
#define EVSYS_INTFLAG_EVD6(value)             (EVSYS_INTFLAG_EVD6_Msk & (_UINT32_(value) << EVSYS_INTFLAG_EVD6_Pos)) /* Assigment of value for EVD6 in the EVSYS_INTFLAG register */
#define EVSYS_INTFLAG_EVD7_Pos                _UINT32_(15)                                         /* (EVSYS_INTFLAG) Channel 7 Event Detection Position */
#define EVSYS_INTFLAG_EVD7_Msk                (_UINT32_(0x1) << EVSYS_INTFLAG_EVD7_Pos)            /* (EVSYS_INTFLAG) Channel 7 Event Detection Mask */
#define EVSYS_INTFLAG_EVD7(value)             (EVSYS_INTFLAG_EVD7_Msk & (_UINT32_(value) << EVSYS_INTFLAG_EVD7_Pos)) /* Assigment of value for EVD7 in the EVSYS_INTFLAG register */
#define EVSYS_INTFLAG_OVR8_Pos                _UINT32_(16)                                         /* (EVSYS_INTFLAG) Channel 8 Overrun Position */
#define EVSYS_INTFLAG_OVR8_Msk                (_UINT32_(0x1) << EVSYS_INTFLAG_OVR8_Pos)            /* (EVSYS_INTFLAG) Channel 8 Overrun Mask */
#define EVSYS_INTFLAG_OVR8(value)             (EVSYS_INTFLAG_OVR8_Msk & (_UINT32_(value) << EVSYS_INTFLAG_OVR8_Pos)) /* Assigment of value for OVR8 in the EVSYS_INTFLAG register */
#define EVSYS_INTFLAG_OVR9_Pos                _UINT32_(17)                                         /* (EVSYS_INTFLAG) Channel 9 Overrun Position */
#define EVSYS_INTFLAG_OVR9_Msk                (_UINT32_(0x1) << EVSYS_INTFLAG_OVR9_Pos)            /* (EVSYS_INTFLAG) Channel 9 Overrun Mask */
#define EVSYS_INTFLAG_OVR9(value)             (EVSYS_INTFLAG_OVR9_Msk & (_UINT32_(value) << EVSYS_INTFLAG_OVR9_Pos)) /* Assigment of value for OVR9 in the EVSYS_INTFLAG register */
#define EVSYS_INTFLAG_OVR10_Pos               _UINT32_(18)                                         /* (EVSYS_INTFLAG) Channel 10 Overrun Position */
#define EVSYS_INTFLAG_OVR10_Msk               (_UINT32_(0x1) << EVSYS_INTFLAG_OVR10_Pos)           /* (EVSYS_INTFLAG) Channel 10 Overrun Mask */
#define EVSYS_INTFLAG_OVR10(value)            (EVSYS_INTFLAG_OVR10_Msk & (_UINT32_(value) << EVSYS_INTFLAG_OVR10_Pos)) /* Assigment of value for OVR10 in the EVSYS_INTFLAG register */
#define EVSYS_INTFLAG_OVR11_Pos               _UINT32_(19)                                         /* (EVSYS_INTFLAG) Channel 11 Overrun Position */
#define EVSYS_INTFLAG_OVR11_Msk               (_UINT32_(0x1) << EVSYS_INTFLAG_OVR11_Pos)           /* (EVSYS_INTFLAG) Channel 11 Overrun Mask */
#define EVSYS_INTFLAG_OVR11(value)            (EVSYS_INTFLAG_OVR11_Msk & (_UINT32_(value) << EVSYS_INTFLAG_OVR11_Pos)) /* Assigment of value for OVR11 in the EVSYS_INTFLAG register */
#define EVSYS_INTFLAG_EVD8_Pos                _UINT32_(24)                                         /* (EVSYS_INTFLAG) Channel 8 Event Detection Position */
#define EVSYS_INTFLAG_EVD8_Msk                (_UINT32_(0x1) << EVSYS_INTFLAG_EVD8_Pos)            /* (EVSYS_INTFLAG) Channel 8 Event Detection Mask */
#define EVSYS_INTFLAG_EVD8(value)             (EVSYS_INTFLAG_EVD8_Msk & (_UINT32_(value) << EVSYS_INTFLAG_EVD8_Pos)) /* Assigment of value for EVD8 in the EVSYS_INTFLAG register */
#define EVSYS_INTFLAG_EVD9_Pos                _UINT32_(25)                                         /* (EVSYS_INTFLAG) Channel 9 Event Detection Position */
#define EVSYS_INTFLAG_EVD9_Msk                (_UINT32_(0x1) << EVSYS_INTFLAG_EVD9_Pos)            /* (EVSYS_INTFLAG) Channel 9 Event Detection Mask */
#define EVSYS_INTFLAG_EVD9(value)             (EVSYS_INTFLAG_EVD9_Msk & (_UINT32_(value) << EVSYS_INTFLAG_EVD9_Pos)) /* Assigment of value for EVD9 in the EVSYS_INTFLAG register */
#define EVSYS_INTFLAG_EVD10_Pos               _UINT32_(26)                                         /* (EVSYS_INTFLAG) Channel 10 Event Detection Position */
#define EVSYS_INTFLAG_EVD10_Msk               (_UINT32_(0x1) << EVSYS_INTFLAG_EVD10_Pos)           /* (EVSYS_INTFLAG) Channel 10 Event Detection Mask */
#define EVSYS_INTFLAG_EVD10(value)            (EVSYS_INTFLAG_EVD10_Msk & (_UINT32_(value) << EVSYS_INTFLAG_EVD10_Pos)) /* Assigment of value for EVD10 in the EVSYS_INTFLAG register */
#define EVSYS_INTFLAG_EVD11_Pos               _UINT32_(27)                                         /* (EVSYS_INTFLAG) Channel 11 Event Detection Position */
#define EVSYS_INTFLAG_EVD11_Msk               (_UINT32_(0x1) << EVSYS_INTFLAG_EVD11_Pos)           /* (EVSYS_INTFLAG) Channel 11 Event Detection Mask */
#define EVSYS_INTFLAG_EVD11(value)            (EVSYS_INTFLAG_EVD11_Msk & (_UINT32_(value) << EVSYS_INTFLAG_EVD11_Pos)) /* Assigment of value for EVD11 in the EVSYS_INTFLAG register */
#define EVSYS_INTFLAG_Msk                     _UINT32_(0x0F0FFFFF)                                 /* (EVSYS_INTFLAG) Register Mask  */


/** \brief EVSYS register offsets definitions */
#define EVSYS_CTRL_REG_OFST            _UINT32_(0x00)      /* (EVSYS_CTRL) Control Offset */
#define EVSYS_CHANNEL_REG_OFST         _UINT32_(0x04)      /* (EVSYS_CHANNEL) Channel Offset */
#define EVSYS_USER_REG_OFST            _UINT32_(0x08)      /* (EVSYS_USER) User Multiplexer Offset */
#define EVSYS_CHSTATUS_REG_OFST        _UINT32_(0x0C)      /* (EVSYS_CHSTATUS) Channel Status Offset */
#define EVSYS_INTENCLR_REG_OFST        _UINT32_(0x10)      /* (EVSYS_INTENCLR) Interrupt Enable Clear Offset */
#define EVSYS_INTENSET_REG_OFST        _UINT32_(0x14)      /* (EVSYS_INTENSET) Interrupt Enable Set Offset */
#define EVSYS_INTFLAG_REG_OFST         _UINT32_(0x18)      /* (EVSYS_INTFLAG) Interrupt Flag Status and Clear Offset */

#if !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__))
/** \brief EVSYS register API structure */
typedef struct
{  /* Event System Interface */
  __O   uint8_t                        EVSYS_CTRL;         /**< Offset: 0x00 ( /W  8) Control */
  __I   uint8_t                        Reserved1[0x03];
  __IO  uint32_t                       EVSYS_CHANNEL;      /**< Offset: 0x04 (R/W  32) Channel */
  __IO  uint16_t                       EVSYS_USER;         /**< Offset: 0x08 (R/W  16) User Multiplexer */
  __I   uint8_t                        Reserved2[0x02];
  __I   uint32_t                       EVSYS_CHSTATUS;     /**< Offset: 0x0C (R/   32) Channel Status */
  __IO  uint32_t                       EVSYS_INTENCLR;     /**< Offset: 0x10 (R/W  32) Interrupt Enable Clear */
  __IO  uint32_t                       EVSYS_INTENSET;     /**< Offset: 0x14 (R/W  32) Interrupt Enable Set */
  __IO  uint32_t                       EVSYS_INTFLAG;      /**< Offset: 0x18 (R/W  32) Interrupt Flag Status and Clear */
} evsys_registers_t;


#endif /* !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__)) */
#endif /* _SAMD21_EVSYS_COMPONENT_H_ */
