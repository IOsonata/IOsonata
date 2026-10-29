/*
 * Component description for USB
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

/* file generated from device description version 2019-11-25T07:00:47Z */
#ifndef _SAMD11_USB_COMPONENT_H_
#define _SAMD11_USB_COMPONENT_H_

/* ************************************************************************** */
/*   SOFTWARE API DEFINITION FOR USB                                          */
/* ************************************************************************** */

/* -------- USB_DEVICE_ADDR : (USB Offset: 0x00) (R/W 32) DEVICE_DESC_BANK Endpoint Bank, Adress of Data Buffer -------- */
#define USB_DEVICE_ADDR_ADDR_Pos              _UINT32_(0)                                          /* (USB_DEVICE_ADDR) Adress of data buffer Position */
#define USB_DEVICE_ADDR_ADDR_Msk              (_UINT32_(0xFFFFFFFF) << USB_DEVICE_ADDR_ADDR_Pos)   /* (USB_DEVICE_ADDR) Adress of data buffer Mask */
#define USB_DEVICE_ADDR_ADDR(value)           (USB_DEVICE_ADDR_ADDR_Msk & (_UINT32_(value) << USB_DEVICE_ADDR_ADDR_Pos)) /* Assigment of value for ADDR in the USB_DEVICE_ADDR register */
#define USB_DEVICE_ADDR_Msk                   _UINT32_(0xFFFFFFFF)                                 /* (USB_DEVICE_ADDR) Register Mask  */


/* -------- USB_DEVICE_PCKSIZE : (USB Offset: 0x04) (R/W 32) DEVICE_DESC_BANK Endpoint Bank, Packet Size -------- */
#define USB_DEVICE_PCKSIZE_BYTE_COUNT_Pos     _UINT32_(0)                                          /* (USB_DEVICE_PCKSIZE) Byte Count Position */
#define USB_DEVICE_PCKSIZE_BYTE_COUNT_Msk     (_UINT32_(0x3FFF) << USB_DEVICE_PCKSIZE_BYTE_COUNT_Pos) /* (USB_DEVICE_PCKSIZE) Byte Count Mask */
#define USB_DEVICE_PCKSIZE_BYTE_COUNT(value)  (USB_DEVICE_PCKSIZE_BYTE_COUNT_Msk & (_UINT32_(value) << USB_DEVICE_PCKSIZE_BYTE_COUNT_Pos)) /* Assigment of value for BYTE_COUNT in the USB_DEVICE_PCKSIZE register */
#define USB_DEVICE_PCKSIZE_MULTI_PACKET_SIZE_Pos _UINT32_(14)                                         /* (USB_DEVICE_PCKSIZE) Multi Packet In or Out size Position */
#define USB_DEVICE_PCKSIZE_MULTI_PACKET_SIZE_Msk (_UINT32_(0x3FFF) << USB_DEVICE_PCKSIZE_MULTI_PACKET_SIZE_Pos) /* (USB_DEVICE_PCKSIZE) Multi Packet In or Out size Mask */
#define USB_DEVICE_PCKSIZE_MULTI_PACKET_SIZE(value) (USB_DEVICE_PCKSIZE_MULTI_PACKET_SIZE_Msk & (_UINT32_(value) << USB_DEVICE_PCKSIZE_MULTI_PACKET_SIZE_Pos)) /* Assigment of value for MULTI_PACKET_SIZE in the USB_DEVICE_PCKSIZE register */
#define USB_DEVICE_PCKSIZE_SIZE_Pos           _UINT32_(28)                                         /* (USB_DEVICE_PCKSIZE) Enpoint size Position */
#define USB_DEVICE_PCKSIZE_SIZE_Msk           (_UINT32_(0x7) << USB_DEVICE_PCKSIZE_SIZE_Pos)       /* (USB_DEVICE_PCKSIZE) Enpoint size Mask */
#define USB_DEVICE_PCKSIZE_SIZE(value)        (USB_DEVICE_PCKSIZE_SIZE_Msk & (_UINT32_(value) << USB_DEVICE_PCKSIZE_SIZE_Pos)) /* Assigment of value for SIZE in the USB_DEVICE_PCKSIZE register */
#define USB_DEVICE_PCKSIZE_AUTO_ZLP_Pos       _UINT32_(31)                                         /* (USB_DEVICE_PCKSIZE) Automatic Zero Length Packet Position */
#define USB_DEVICE_PCKSIZE_AUTO_ZLP_Msk       (_UINT32_(0x1) << USB_DEVICE_PCKSIZE_AUTO_ZLP_Pos)   /* (USB_DEVICE_PCKSIZE) Automatic Zero Length Packet Mask */
#define USB_DEVICE_PCKSIZE_AUTO_ZLP(value)    (USB_DEVICE_PCKSIZE_AUTO_ZLP_Msk & (_UINT32_(value) << USB_DEVICE_PCKSIZE_AUTO_ZLP_Pos)) /* Assigment of value for AUTO_ZLP in the USB_DEVICE_PCKSIZE register */
#define USB_DEVICE_PCKSIZE_Msk                _UINT32_(0xFFFFFFFF)                                 /* (USB_DEVICE_PCKSIZE) Register Mask  */


/* -------- USB_DEVICE_EXTREG : (USB Offset: 0x08) (R/W 16) DEVICE_DESC_BANK Endpoint Bank, Extended -------- */
#define USB_DEVICE_EXTREG_SUBPID_Pos          _UINT16_(0)                                          /* (USB_DEVICE_EXTREG) SUBPID field send with extended token Position */
#define USB_DEVICE_EXTREG_SUBPID_Msk          (_UINT16_(0xF) << USB_DEVICE_EXTREG_SUBPID_Pos)      /* (USB_DEVICE_EXTREG) SUBPID field send with extended token Mask */
#define USB_DEVICE_EXTREG_SUBPID(value)       (USB_DEVICE_EXTREG_SUBPID_Msk & (_UINT16_(value) << USB_DEVICE_EXTREG_SUBPID_Pos)) /* Assigment of value for SUBPID in the USB_DEVICE_EXTREG register */
#define USB_DEVICE_EXTREG_VARIABLE_Pos        _UINT16_(4)                                          /* (USB_DEVICE_EXTREG) Variable field send with extended token Position */
#define USB_DEVICE_EXTREG_VARIABLE_Msk        (_UINT16_(0x7FF) << USB_DEVICE_EXTREG_VARIABLE_Pos)  /* (USB_DEVICE_EXTREG) Variable field send with extended token Mask */
#define USB_DEVICE_EXTREG_VARIABLE(value)     (USB_DEVICE_EXTREG_VARIABLE_Msk & (_UINT16_(value) << USB_DEVICE_EXTREG_VARIABLE_Pos)) /* Assigment of value for VARIABLE in the USB_DEVICE_EXTREG register */
#define USB_DEVICE_EXTREG_Msk                 _UINT16_(0x7FFF)                                     /* (USB_DEVICE_EXTREG) Register Mask  */


/* -------- USB_DEVICE_STATUS_BK : (USB Offset: 0x0A) (R/W 8) DEVICE_DESC_BANK Enpoint Bank, Status of Bank -------- */
#define USB_DEVICE_STATUS_BK_CRCERR_Pos       _UINT8_(0)                                           /* (USB_DEVICE_STATUS_BK) CRC Error Status Position */
#define USB_DEVICE_STATUS_BK_CRCERR_Msk       (_UINT8_(0x1) << USB_DEVICE_STATUS_BK_CRCERR_Pos)    /* (USB_DEVICE_STATUS_BK) CRC Error Status Mask */
#define USB_DEVICE_STATUS_BK_CRCERR(value)    (USB_DEVICE_STATUS_BK_CRCERR_Msk & (_UINT8_(value) << USB_DEVICE_STATUS_BK_CRCERR_Pos)) /* Assigment of value for CRCERR in the USB_DEVICE_STATUS_BK register */
#define USB_DEVICE_STATUS_BK_ERRORFLOW_Pos    _UINT8_(1)                                           /* (USB_DEVICE_STATUS_BK) Error Flow Status Position */
#define USB_DEVICE_STATUS_BK_ERRORFLOW_Msk    (_UINT8_(0x1) << USB_DEVICE_STATUS_BK_ERRORFLOW_Pos) /* (USB_DEVICE_STATUS_BK) Error Flow Status Mask */
#define USB_DEVICE_STATUS_BK_ERRORFLOW(value) (USB_DEVICE_STATUS_BK_ERRORFLOW_Msk & (_UINT8_(value) << USB_DEVICE_STATUS_BK_ERRORFLOW_Pos)) /* Assigment of value for ERRORFLOW in the USB_DEVICE_STATUS_BK register */
#define USB_DEVICE_STATUS_BK_Msk              _UINT8_(0x03)                                        /* (USB_DEVICE_STATUS_BK) Register Mask  */


/* -------- USB_DEVICE_EPCFG : (USB Offset: 0x00) (R/W 8) DEVICE_ENDPOINT End Point Configuration -------- */
#define USB_DEVICE_EPCFG_RESETVALUE           _UINT8_(0x00)                                        /*  (USB_DEVICE_EPCFG) DEVICE_ENDPOINT End Point Configuration  Reset Value */

#define USB_DEVICE_EPCFG_EPTYPE0_Pos          _UINT8_(0)                                           /* (USB_DEVICE_EPCFG) End Point Type0 Position */
#define USB_DEVICE_EPCFG_EPTYPE0_Msk          (_UINT8_(0x7) << USB_DEVICE_EPCFG_EPTYPE0_Pos)       /* (USB_DEVICE_EPCFG) End Point Type0 Mask */
#define USB_DEVICE_EPCFG_EPTYPE0(value)       (USB_DEVICE_EPCFG_EPTYPE0_Msk & (_UINT8_(value) << USB_DEVICE_EPCFG_EPTYPE0_Pos)) /* Assigment of value for EPTYPE0 in the USB_DEVICE_EPCFG register */
#define USB_DEVICE_EPCFG_EPTYPE1_Pos          _UINT8_(4)                                           /* (USB_DEVICE_EPCFG) End Point Type1 Position */
#define USB_DEVICE_EPCFG_EPTYPE1_Msk          (_UINT8_(0x7) << USB_DEVICE_EPCFG_EPTYPE1_Pos)       /* (USB_DEVICE_EPCFG) End Point Type1 Mask */
#define USB_DEVICE_EPCFG_EPTYPE1(value)       (USB_DEVICE_EPCFG_EPTYPE1_Msk & (_UINT8_(value) << USB_DEVICE_EPCFG_EPTYPE1_Pos)) /* Assigment of value for EPTYPE1 in the USB_DEVICE_EPCFG register */
#define USB_DEVICE_EPCFG_NYETDIS_Pos          _UINT8_(7)                                           /* (USB_DEVICE_EPCFG) NYET Token Disable Position */
#define USB_DEVICE_EPCFG_NYETDIS_Msk          (_UINT8_(0x1) << USB_DEVICE_EPCFG_NYETDIS_Pos)       /* (USB_DEVICE_EPCFG) NYET Token Disable Mask */
#define USB_DEVICE_EPCFG_NYETDIS(value)       (USB_DEVICE_EPCFG_NYETDIS_Msk & (_UINT8_(value) << USB_DEVICE_EPCFG_NYETDIS_Pos)) /* Assigment of value for NYETDIS in the USB_DEVICE_EPCFG register */
#define USB_DEVICE_EPCFG_Msk                  _UINT8_(0xF7)                                        /* (USB_DEVICE_EPCFG) Register Mask  */


/* -------- USB_DEVICE_EPSTATUSCLR : (USB Offset: 0x04) ( /W 8) DEVICE_ENDPOINT End Point Pipe Status Clear -------- */
#define USB_DEVICE_EPSTATUSCLR_RESETVALUE     _UINT8_(0x00)                                        /*  (USB_DEVICE_EPSTATUSCLR) DEVICE_ENDPOINT End Point Pipe Status Clear  Reset Value */

#define USB_DEVICE_EPSTATUSCLR_DTGLOUT_Pos    _UINT8_(0)                                           /* (USB_DEVICE_EPSTATUSCLR) Data Toggle OUT Clear Position */
#define USB_DEVICE_EPSTATUSCLR_DTGLOUT_Msk    (_UINT8_(0x1) << USB_DEVICE_EPSTATUSCLR_DTGLOUT_Pos) /* (USB_DEVICE_EPSTATUSCLR) Data Toggle OUT Clear Mask */
#define USB_DEVICE_EPSTATUSCLR_DTGLOUT(value) (USB_DEVICE_EPSTATUSCLR_DTGLOUT_Msk & (_UINT8_(value) << USB_DEVICE_EPSTATUSCLR_DTGLOUT_Pos)) /* Assigment of value for DTGLOUT in the USB_DEVICE_EPSTATUSCLR register */
#define USB_DEVICE_EPSTATUSCLR_DTGLIN_Pos     _UINT8_(1)                                           /* (USB_DEVICE_EPSTATUSCLR) Data Toggle IN Clear Position */
#define USB_DEVICE_EPSTATUSCLR_DTGLIN_Msk     (_UINT8_(0x1) << USB_DEVICE_EPSTATUSCLR_DTGLIN_Pos)  /* (USB_DEVICE_EPSTATUSCLR) Data Toggle IN Clear Mask */
#define USB_DEVICE_EPSTATUSCLR_DTGLIN(value)  (USB_DEVICE_EPSTATUSCLR_DTGLIN_Msk & (_UINT8_(value) << USB_DEVICE_EPSTATUSCLR_DTGLIN_Pos)) /* Assigment of value for DTGLIN in the USB_DEVICE_EPSTATUSCLR register */
#define USB_DEVICE_EPSTATUSCLR_CURBK_Pos      _UINT8_(2)                                           /* (USB_DEVICE_EPSTATUSCLR) Current Bank Clear Position */
#define USB_DEVICE_EPSTATUSCLR_CURBK_Msk      (_UINT8_(0x1) << USB_DEVICE_EPSTATUSCLR_CURBK_Pos)   /* (USB_DEVICE_EPSTATUSCLR) Current Bank Clear Mask */
#define USB_DEVICE_EPSTATUSCLR_CURBK(value)   (USB_DEVICE_EPSTATUSCLR_CURBK_Msk & (_UINT8_(value) << USB_DEVICE_EPSTATUSCLR_CURBK_Pos)) /* Assigment of value for CURBK in the USB_DEVICE_EPSTATUSCLR register */
#define USB_DEVICE_EPSTATUSCLR_STALLRQ0_Pos   _UINT8_(4)                                           /* (USB_DEVICE_EPSTATUSCLR) Stall 0 Request Clear Position */
#define USB_DEVICE_EPSTATUSCLR_STALLRQ0_Msk   (_UINT8_(0x1) << USB_DEVICE_EPSTATUSCLR_STALLRQ0_Pos) /* (USB_DEVICE_EPSTATUSCLR) Stall 0 Request Clear Mask */
#define USB_DEVICE_EPSTATUSCLR_STALLRQ0(value) (USB_DEVICE_EPSTATUSCLR_STALLRQ0_Msk & (_UINT8_(value) << USB_DEVICE_EPSTATUSCLR_STALLRQ0_Pos)) /* Assigment of value for STALLRQ0 in the USB_DEVICE_EPSTATUSCLR register */
#define USB_DEVICE_EPSTATUSCLR_STALLRQ1_Pos   _UINT8_(5)                                           /* (USB_DEVICE_EPSTATUSCLR) Stall 1 Request Clear Position */
#define USB_DEVICE_EPSTATUSCLR_STALLRQ1_Msk   (_UINT8_(0x1) << USB_DEVICE_EPSTATUSCLR_STALLRQ1_Pos) /* (USB_DEVICE_EPSTATUSCLR) Stall 1 Request Clear Mask */
#define USB_DEVICE_EPSTATUSCLR_STALLRQ1(value) (USB_DEVICE_EPSTATUSCLR_STALLRQ1_Msk & (_UINT8_(value) << USB_DEVICE_EPSTATUSCLR_STALLRQ1_Pos)) /* Assigment of value for STALLRQ1 in the USB_DEVICE_EPSTATUSCLR register */
#define USB_DEVICE_EPSTATUSCLR_BK0RDY_Pos     _UINT8_(6)                                           /* (USB_DEVICE_EPSTATUSCLR) Bank 0 Ready Clear Position */
#define USB_DEVICE_EPSTATUSCLR_BK0RDY_Msk     (_UINT8_(0x1) << USB_DEVICE_EPSTATUSCLR_BK0RDY_Pos)  /* (USB_DEVICE_EPSTATUSCLR) Bank 0 Ready Clear Mask */
#define USB_DEVICE_EPSTATUSCLR_BK0RDY(value)  (USB_DEVICE_EPSTATUSCLR_BK0RDY_Msk & (_UINT8_(value) << USB_DEVICE_EPSTATUSCLR_BK0RDY_Pos)) /* Assigment of value for BK0RDY in the USB_DEVICE_EPSTATUSCLR register */
#define USB_DEVICE_EPSTATUSCLR_BK1RDY_Pos     _UINT8_(7)                                           /* (USB_DEVICE_EPSTATUSCLR) Bank 1 Ready Clear Position */
#define USB_DEVICE_EPSTATUSCLR_BK1RDY_Msk     (_UINT8_(0x1) << USB_DEVICE_EPSTATUSCLR_BK1RDY_Pos)  /* (USB_DEVICE_EPSTATUSCLR) Bank 1 Ready Clear Mask */
#define USB_DEVICE_EPSTATUSCLR_BK1RDY(value)  (USB_DEVICE_EPSTATUSCLR_BK1RDY_Msk & (_UINT8_(value) << USB_DEVICE_EPSTATUSCLR_BK1RDY_Pos)) /* Assigment of value for BK1RDY in the USB_DEVICE_EPSTATUSCLR register */
#define USB_DEVICE_EPSTATUSCLR_Msk            _UINT8_(0xF7)                                        /* (USB_DEVICE_EPSTATUSCLR) Register Mask  */

#define USB_DEVICE_EPSTATUSCLR_STALLRQ_Pos    _UINT8_(4)                                           /* (USB_DEVICE_EPSTATUSCLR Position) Stall x Request Clear */
#define USB_DEVICE_EPSTATUSCLR_STALLRQ_Msk    (_UINT8_(0x3) << USB_DEVICE_EPSTATUSCLR_STALLRQ_Pos) /* (USB_DEVICE_EPSTATUSCLR Mask) STALLRQ */
#define USB_DEVICE_EPSTATUSCLR_STALLRQ(value) (USB_DEVICE_EPSTATUSCLR_STALLRQ_Msk & (_UINT8_(value) << USB_DEVICE_EPSTATUSCLR_STALLRQ_Pos)) 

/* -------- USB_DEVICE_EPSTATUSSET : (USB Offset: 0x05) ( /W 8) DEVICE_ENDPOINT End Point Pipe Status Set -------- */
#define USB_DEVICE_EPSTATUSSET_RESETVALUE     _UINT8_(0x00)                                        /*  (USB_DEVICE_EPSTATUSSET) DEVICE_ENDPOINT End Point Pipe Status Set  Reset Value */

#define USB_DEVICE_EPSTATUSSET_DTGLOUT_Pos    _UINT8_(0)                                           /* (USB_DEVICE_EPSTATUSSET) Data Toggle OUT Set Position */
#define USB_DEVICE_EPSTATUSSET_DTGLOUT_Msk    (_UINT8_(0x1) << USB_DEVICE_EPSTATUSSET_DTGLOUT_Pos) /* (USB_DEVICE_EPSTATUSSET) Data Toggle OUT Set Mask */
#define USB_DEVICE_EPSTATUSSET_DTGLOUT(value) (USB_DEVICE_EPSTATUSSET_DTGLOUT_Msk & (_UINT8_(value) << USB_DEVICE_EPSTATUSSET_DTGLOUT_Pos)) /* Assigment of value for DTGLOUT in the USB_DEVICE_EPSTATUSSET register */
#define USB_DEVICE_EPSTATUSSET_DTGLIN_Pos     _UINT8_(1)                                           /* (USB_DEVICE_EPSTATUSSET) Data Toggle IN Set Position */
#define USB_DEVICE_EPSTATUSSET_DTGLIN_Msk     (_UINT8_(0x1) << USB_DEVICE_EPSTATUSSET_DTGLIN_Pos)  /* (USB_DEVICE_EPSTATUSSET) Data Toggle IN Set Mask */
#define USB_DEVICE_EPSTATUSSET_DTGLIN(value)  (USB_DEVICE_EPSTATUSSET_DTGLIN_Msk & (_UINT8_(value) << USB_DEVICE_EPSTATUSSET_DTGLIN_Pos)) /* Assigment of value for DTGLIN in the USB_DEVICE_EPSTATUSSET register */
#define USB_DEVICE_EPSTATUSSET_CURBK_Pos      _UINT8_(2)                                           /* (USB_DEVICE_EPSTATUSSET) Current Bank Set Position */
#define USB_DEVICE_EPSTATUSSET_CURBK_Msk      (_UINT8_(0x1) << USB_DEVICE_EPSTATUSSET_CURBK_Pos)   /* (USB_DEVICE_EPSTATUSSET) Current Bank Set Mask */
#define USB_DEVICE_EPSTATUSSET_CURBK(value)   (USB_DEVICE_EPSTATUSSET_CURBK_Msk & (_UINT8_(value) << USB_DEVICE_EPSTATUSSET_CURBK_Pos)) /* Assigment of value for CURBK in the USB_DEVICE_EPSTATUSSET register */
#define USB_DEVICE_EPSTATUSSET_STALLRQ0_Pos   _UINT8_(4)                                           /* (USB_DEVICE_EPSTATUSSET) Stall 0 Request Set Position */
#define USB_DEVICE_EPSTATUSSET_STALLRQ0_Msk   (_UINT8_(0x1) << USB_DEVICE_EPSTATUSSET_STALLRQ0_Pos) /* (USB_DEVICE_EPSTATUSSET) Stall 0 Request Set Mask */
#define USB_DEVICE_EPSTATUSSET_STALLRQ0(value) (USB_DEVICE_EPSTATUSSET_STALLRQ0_Msk & (_UINT8_(value) << USB_DEVICE_EPSTATUSSET_STALLRQ0_Pos)) /* Assigment of value for STALLRQ0 in the USB_DEVICE_EPSTATUSSET register */
#define USB_DEVICE_EPSTATUSSET_STALLRQ1_Pos   _UINT8_(5)                                           /* (USB_DEVICE_EPSTATUSSET) Stall 1 Request Set Position */
#define USB_DEVICE_EPSTATUSSET_STALLRQ1_Msk   (_UINT8_(0x1) << USB_DEVICE_EPSTATUSSET_STALLRQ1_Pos) /* (USB_DEVICE_EPSTATUSSET) Stall 1 Request Set Mask */
#define USB_DEVICE_EPSTATUSSET_STALLRQ1(value) (USB_DEVICE_EPSTATUSSET_STALLRQ1_Msk & (_UINT8_(value) << USB_DEVICE_EPSTATUSSET_STALLRQ1_Pos)) /* Assigment of value for STALLRQ1 in the USB_DEVICE_EPSTATUSSET register */
#define USB_DEVICE_EPSTATUSSET_BK0RDY_Pos     _UINT8_(6)                                           /* (USB_DEVICE_EPSTATUSSET) Bank 0 Ready Set Position */
#define USB_DEVICE_EPSTATUSSET_BK0RDY_Msk     (_UINT8_(0x1) << USB_DEVICE_EPSTATUSSET_BK0RDY_Pos)  /* (USB_DEVICE_EPSTATUSSET) Bank 0 Ready Set Mask */
#define USB_DEVICE_EPSTATUSSET_BK0RDY(value)  (USB_DEVICE_EPSTATUSSET_BK0RDY_Msk & (_UINT8_(value) << USB_DEVICE_EPSTATUSSET_BK0RDY_Pos)) /* Assigment of value for BK0RDY in the USB_DEVICE_EPSTATUSSET register */
#define USB_DEVICE_EPSTATUSSET_BK1RDY_Pos     _UINT8_(7)                                           /* (USB_DEVICE_EPSTATUSSET) Bank 1 Ready Set Position */
#define USB_DEVICE_EPSTATUSSET_BK1RDY_Msk     (_UINT8_(0x1) << USB_DEVICE_EPSTATUSSET_BK1RDY_Pos)  /* (USB_DEVICE_EPSTATUSSET) Bank 1 Ready Set Mask */
#define USB_DEVICE_EPSTATUSSET_BK1RDY(value)  (USB_DEVICE_EPSTATUSSET_BK1RDY_Msk & (_UINT8_(value) << USB_DEVICE_EPSTATUSSET_BK1RDY_Pos)) /* Assigment of value for BK1RDY in the USB_DEVICE_EPSTATUSSET register */
#define USB_DEVICE_EPSTATUSSET_Msk            _UINT8_(0xF7)                                        /* (USB_DEVICE_EPSTATUSSET) Register Mask  */

#define USB_DEVICE_EPSTATUSSET_STALLRQ_Pos    _UINT8_(4)                                           /* (USB_DEVICE_EPSTATUSSET Position) Stall x Request Set */
#define USB_DEVICE_EPSTATUSSET_STALLRQ_Msk    (_UINT8_(0x3) << USB_DEVICE_EPSTATUSSET_STALLRQ_Pos) /* (USB_DEVICE_EPSTATUSSET Mask) STALLRQ */
#define USB_DEVICE_EPSTATUSSET_STALLRQ(value) (USB_DEVICE_EPSTATUSSET_STALLRQ_Msk & (_UINT8_(value) << USB_DEVICE_EPSTATUSSET_STALLRQ_Pos)) 

/* -------- USB_DEVICE_EPSTATUS : (USB Offset: 0x06) ( R/ 8) DEVICE_ENDPOINT End Point Pipe Status -------- */
#define USB_DEVICE_EPSTATUS_RESETVALUE        _UINT8_(0x00)                                        /*  (USB_DEVICE_EPSTATUS) DEVICE_ENDPOINT End Point Pipe Status  Reset Value */

#define USB_DEVICE_EPSTATUS_DTGLOUT_Pos       _UINT8_(0)                                           /* (USB_DEVICE_EPSTATUS) Data Toggle Out Position */
#define USB_DEVICE_EPSTATUS_DTGLOUT_Msk       (_UINT8_(0x1) << USB_DEVICE_EPSTATUS_DTGLOUT_Pos)    /* (USB_DEVICE_EPSTATUS) Data Toggle Out Mask */
#define USB_DEVICE_EPSTATUS_DTGLOUT(value)    (USB_DEVICE_EPSTATUS_DTGLOUT_Msk & (_UINT8_(value) << USB_DEVICE_EPSTATUS_DTGLOUT_Pos)) /* Assigment of value for DTGLOUT in the USB_DEVICE_EPSTATUS register */
#define USB_DEVICE_EPSTATUS_DTGLIN_Pos        _UINT8_(1)                                           /* (USB_DEVICE_EPSTATUS) Data Toggle In Position */
#define USB_DEVICE_EPSTATUS_DTGLIN_Msk        (_UINT8_(0x1) << USB_DEVICE_EPSTATUS_DTGLIN_Pos)     /* (USB_DEVICE_EPSTATUS) Data Toggle In Mask */
#define USB_DEVICE_EPSTATUS_DTGLIN(value)     (USB_DEVICE_EPSTATUS_DTGLIN_Msk & (_UINT8_(value) << USB_DEVICE_EPSTATUS_DTGLIN_Pos)) /* Assigment of value for DTGLIN in the USB_DEVICE_EPSTATUS register */
#define USB_DEVICE_EPSTATUS_CURBK_Pos         _UINT8_(2)                                           /* (USB_DEVICE_EPSTATUS) Current Bank Position */
#define USB_DEVICE_EPSTATUS_CURBK_Msk         (_UINT8_(0x1) << USB_DEVICE_EPSTATUS_CURBK_Pos)      /* (USB_DEVICE_EPSTATUS) Current Bank Mask */
#define USB_DEVICE_EPSTATUS_CURBK(value)      (USB_DEVICE_EPSTATUS_CURBK_Msk & (_UINT8_(value) << USB_DEVICE_EPSTATUS_CURBK_Pos)) /* Assigment of value for CURBK in the USB_DEVICE_EPSTATUS register */
#define USB_DEVICE_EPSTATUS_STALLRQ0_Pos      _UINT8_(4)                                           /* (USB_DEVICE_EPSTATUS) Stall 0 Request Position */
#define USB_DEVICE_EPSTATUS_STALLRQ0_Msk      (_UINT8_(0x1) << USB_DEVICE_EPSTATUS_STALLRQ0_Pos)   /* (USB_DEVICE_EPSTATUS) Stall 0 Request Mask */
#define USB_DEVICE_EPSTATUS_STALLRQ0(value)   (USB_DEVICE_EPSTATUS_STALLRQ0_Msk & (_UINT8_(value) << USB_DEVICE_EPSTATUS_STALLRQ0_Pos)) /* Assigment of value for STALLRQ0 in the USB_DEVICE_EPSTATUS register */
#define USB_DEVICE_EPSTATUS_STALLRQ1_Pos      _UINT8_(5)                                           /* (USB_DEVICE_EPSTATUS) Stall 1 Request Position */
#define USB_DEVICE_EPSTATUS_STALLRQ1_Msk      (_UINT8_(0x1) << USB_DEVICE_EPSTATUS_STALLRQ1_Pos)   /* (USB_DEVICE_EPSTATUS) Stall 1 Request Mask */
#define USB_DEVICE_EPSTATUS_STALLRQ1(value)   (USB_DEVICE_EPSTATUS_STALLRQ1_Msk & (_UINT8_(value) << USB_DEVICE_EPSTATUS_STALLRQ1_Pos)) /* Assigment of value for STALLRQ1 in the USB_DEVICE_EPSTATUS register */
#define USB_DEVICE_EPSTATUS_BK0RDY_Pos        _UINT8_(6)                                           /* (USB_DEVICE_EPSTATUS) Bank 0 ready Position */
#define USB_DEVICE_EPSTATUS_BK0RDY_Msk        (_UINT8_(0x1) << USB_DEVICE_EPSTATUS_BK0RDY_Pos)     /* (USB_DEVICE_EPSTATUS) Bank 0 ready Mask */
#define USB_DEVICE_EPSTATUS_BK0RDY(value)     (USB_DEVICE_EPSTATUS_BK0RDY_Msk & (_UINT8_(value) << USB_DEVICE_EPSTATUS_BK0RDY_Pos)) /* Assigment of value for BK0RDY in the USB_DEVICE_EPSTATUS register */
#define USB_DEVICE_EPSTATUS_BK1RDY_Pos        _UINT8_(7)                                           /* (USB_DEVICE_EPSTATUS) Bank 1 ready Position */
#define USB_DEVICE_EPSTATUS_BK1RDY_Msk        (_UINT8_(0x1) << USB_DEVICE_EPSTATUS_BK1RDY_Pos)     /* (USB_DEVICE_EPSTATUS) Bank 1 ready Mask */
#define USB_DEVICE_EPSTATUS_BK1RDY(value)     (USB_DEVICE_EPSTATUS_BK1RDY_Msk & (_UINT8_(value) << USB_DEVICE_EPSTATUS_BK1RDY_Pos)) /* Assigment of value for BK1RDY in the USB_DEVICE_EPSTATUS register */
#define USB_DEVICE_EPSTATUS_Msk               _UINT8_(0xF7)                                        /* (USB_DEVICE_EPSTATUS) Register Mask  */

#define USB_DEVICE_EPSTATUS_STALLRQ_Pos       _UINT8_(4)                                           /* (USB_DEVICE_EPSTATUS Position) Stall x Request */
#define USB_DEVICE_EPSTATUS_STALLRQ_Msk       (_UINT8_(0x3) << USB_DEVICE_EPSTATUS_STALLRQ_Pos)    /* (USB_DEVICE_EPSTATUS Mask) STALLRQ */
#define USB_DEVICE_EPSTATUS_STALLRQ(value)    (USB_DEVICE_EPSTATUS_STALLRQ_Msk & (_UINT8_(value) << USB_DEVICE_EPSTATUS_STALLRQ_Pos)) 

/* -------- USB_DEVICE_EPINTFLAG : (USB Offset: 0x07) (R/W 8) DEVICE_ENDPOINT End Point Interrupt Flag -------- */
#define USB_DEVICE_EPINTFLAG_RESETVALUE       _UINT8_(0x00)                                        /*  (USB_DEVICE_EPINTFLAG) DEVICE_ENDPOINT End Point Interrupt Flag  Reset Value */

#define USB_DEVICE_EPINTFLAG_TRCPT0_Pos       _UINT8_(0)                                           /* (USB_DEVICE_EPINTFLAG) Transfer Complete 0 Position */
#define USB_DEVICE_EPINTFLAG_TRCPT0_Msk       (_UINT8_(0x1) << USB_DEVICE_EPINTFLAG_TRCPT0_Pos)    /* (USB_DEVICE_EPINTFLAG) Transfer Complete 0 Mask */
#define USB_DEVICE_EPINTFLAG_TRCPT0(value)    (USB_DEVICE_EPINTFLAG_TRCPT0_Msk & (_UINT8_(value) << USB_DEVICE_EPINTFLAG_TRCPT0_Pos)) /* Assigment of value for TRCPT0 in the USB_DEVICE_EPINTFLAG register */
#define USB_DEVICE_EPINTFLAG_TRCPT1_Pos       _UINT8_(1)                                           /* (USB_DEVICE_EPINTFLAG) Transfer Complete 1 Position */
#define USB_DEVICE_EPINTFLAG_TRCPT1_Msk       (_UINT8_(0x1) << USB_DEVICE_EPINTFLAG_TRCPT1_Pos)    /* (USB_DEVICE_EPINTFLAG) Transfer Complete 1 Mask */
#define USB_DEVICE_EPINTFLAG_TRCPT1(value)    (USB_DEVICE_EPINTFLAG_TRCPT1_Msk & (_UINT8_(value) << USB_DEVICE_EPINTFLAG_TRCPT1_Pos)) /* Assigment of value for TRCPT1 in the USB_DEVICE_EPINTFLAG register */
#define USB_DEVICE_EPINTFLAG_TRFAIL0_Pos      _UINT8_(2)                                           /* (USB_DEVICE_EPINTFLAG) Error Flow 0 Position */
#define USB_DEVICE_EPINTFLAG_TRFAIL0_Msk      (_UINT8_(0x1) << USB_DEVICE_EPINTFLAG_TRFAIL0_Pos)   /* (USB_DEVICE_EPINTFLAG) Error Flow 0 Mask */
#define USB_DEVICE_EPINTFLAG_TRFAIL0(value)   (USB_DEVICE_EPINTFLAG_TRFAIL0_Msk & (_UINT8_(value) << USB_DEVICE_EPINTFLAG_TRFAIL0_Pos)) /* Assigment of value for TRFAIL0 in the USB_DEVICE_EPINTFLAG register */
#define USB_DEVICE_EPINTFLAG_TRFAIL1_Pos      _UINT8_(3)                                           /* (USB_DEVICE_EPINTFLAG) Error Flow 1 Position */
#define USB_DEVICE_EPINTFLAG_TRFAIL1_Msk      (_UINT8_(0x1) << USB_DEVICE_EPINTFLAG_TRFAIL1_Pos)   /* (USB_DEVICE_EPINTFLAG) Error Flow 1 Mask */
#define USB_DEVICE_EPINTFLAG_TRFAIL1(value)   (USB_DEVICE_EPINTFLAG_TRFAIL1_Msk & (_UINT8_(value) << USB_DEVICE_EPINTFLAG_TRFAIL1_Pos)) /* Assigment of value for TRFAIL1 in the USB_DEVICE_EPINTFLAG register */
#define USB_DEVICE_EPINTFLAG_RXSTP_Pos        _UINT8_(4)                                           /* (USB_DEVICE_EPINTFLAG) Received Setup Position */
#define USB_DEVICE_EPINTFLAG_RXSTP_Msk        (_UINT8_(0x1) << USB_DEVICE_EPINTFLAG_RXSTP_Pos)     /* (USB_DEVICE_EPINTFLAG) Received Setup Mask */
#define USB_DEVICE_EPINTFLAG_RXSTP(value)     (USB_DEVICE_EPINTFLAG_RXSTP_Msk & (_UINT8_(value) << USB_DEVICE_EPINTFLAG_RXSTP_Pos)) /* Assigment of value for RXSTP in the USB_DEVICE_EPINTFLAG register */
#define USB_DEVICE_EPINTFLAG_STALL0_Pos       _UINT8_(5)                                           /* (USB_DEVICE_EPINTFLAG) Stall 0 In/out Position */
#define USB_DEVICE_EPINTFLAG_STALL0_Msk       (_UINT8_(0x1) << USB_DEVICE_EPINTFLAG_STALL0_Pos)    /* (USB_DEVICE_EPINTFLAG) Stall 0 In/out Mask */
#define USB_DEVICE_EPINTFLAG_STALL0(value)    (USB_DEVICE_EPINTFLAG_STALL0_Msk & (_UINT8_(value) << USB_DEVICE_EPINTFLAG_STALL0_Pos)) /* Assigment of value for STALL0 in the USB_DEVICE_EPINTFLAG register */
#define USB_DEVICE_EPINTFLAG_STALL1_Pos       _UINT8_(6)                                           /* (USB_DEVICE_EPINTFLAG) Stall 1 In/out Position */
#define USB_DEVICE_EPINTFLAG_STALL1_Msk       (_UINT8_(0x1) << USB_DEVICE_EPINTFLAG_STALL1_Pos)    /* (USB_DEVICE_EPINTFLAG) Stall 1 In/out Mask */
#define USB_DEVICE_EPINTFLAG_STALL1(value)    (USB_DEVICE_EPINTFLAG_STALL1_Msk & (_UINT8_(value) << USB_DEVICE_EPINTFLAG_STALL1_Pos)) /* Assigment of value for STALL1 in the USB_DEVICE_EPINTFLAG register */
#define USB_DEVICE_EPINTFLAG_Msk              _UINT8_(0x7F)                                        /* (USB_DEVICE_EPINTFLAG) Register Mask  */

#define USB_DEVICE_EPINTFLAG_TRCPT_Pos        _UINT8_(0)                                           /* (USB_DEVICE_EPINTFLAG Position) Transfer Complete x */
#define USB_DEVICE_EPINTFLAG_TRCPT_Msk        (_UINT8_(0x3) << USB_DEVICE_EPINTFLAG_TRCPT_Pos)     /* (USB_DEVICE_EPINTFLAG Mask) TRCPT */
#define USB_DEVICE_EPINTFLAG_TRCPT(value)     (USB_DEVICE_EPINTFLAG_TRCPT_Msk & (_UINT8_(value) << USB_DEVICE_EPINTFLAG_TRCPT_Pos)) 
#define USB_DEVICE_EPINTFLAG_TRFAIL_Pos       _UINT8_(2)                                           /* (USB_DEVICE_EPINTFLAG Position) Error Flow x */
#define USB_DEVICE_EPINTFLAG_TRFAIL_Msk       (_UINT8_(0x3) << USB_DEVICE_EPINTFLAG_TRFAIL_Pos)    /* (USB_DEVICE_EPINTFLAG Mask) TRFAIL */
#define USB_DEVICE_EPINTFLAG_TRFAIL(value)    (USB_DEVICE_EPINTFLAG_TRFAIL_Msk & (_UINT8_(value) << USB_DEVICE_EPINTFLAG_TRFAIL_Pos)) 
#define USB_DEVICE_EPINTFLAG_STALL_Pos        _UINT8_(5)                                           /* (USB_DEVICE_EPINTFLAG Position) Stall x In/out */
#define USB_DEVICE_EPINTFLAG_STALL_Msk        (_UINT8_(0x3) << USB_DEVICE_EPINTFLAG_STALL_Pos)     /* (USB_DEVICE_EPINTFLAG Mask) STALL */
#define USB_DEVICE_EPINTFLAG_STALL(value)     (USB_DEVICE_EPINTFLAG_STALL_Msk & (_UINT8_(value) << USB_DEVICE_EPINTFLAG_STALL_Pos)) 

/* -------- USB_DEVICE_EPINTENCLR : (USB Offset: 0x08) (R/W 8) DEVICE_ENDPOINT End Point Interrupt Clear Flag -------- */
#define USB_DEVICE_EPINTENCLR_RESETVALUE      _UINT8_(0x00)                                        /*  (USB_DEVICE_EPINTENCLR) DEVICE_ENDPOINT End Point Interrupt Clear Flag  Reset Value */

#define USB_DEVICE_EPINTENCLR_TRCPT0_Pos      _UINT8_(0)                                           /* (USB_DEVICE_EPINTENCLR) Transfer Complete 0 Interrupt Disable Position */
#define USB_DEVICE_EPINTENCLR_TRCPT0_Msk      (_UINT8_(0x1) << USB_DEVICE_EPINTENCLR_TRCPT0_Pos)   /* (USB_DEVICE_EPINTENCLR) Transfer Complete 0 Interrupt Disable Mask */
#define USB_DEVICE_EPINTENCLR_TRCPT0(value)   (USB_DEVICE_EPINTENCLR_TRCPT0_Msk & (_UINT8_(value) << USB_DEVICE_EPINTENCLR_TRCPT0_Pos)) /* Assigment of value for TRCPT0 in the USB_DEVICE_EPINTENCLR register */
#define USB_DEVICE_EPINTENCLR_TRCPT1_Pos      _UINT8_(1)                                           /* (USB_DEVICE_EPINTENCLR) Transfer Complete 1 Interrupt Disable Position */
#define USB_DEVICE_EPINTENCLR_TRCPT1_Msk      (_UINT8_(0x1) << USB_DEVICE_EPINTENCLR_TRCPT1_Pos)   /* (USB_DEVICE_EPINTENCLR) Transfer Complete 1 Interrupt Disable Mask */
#define USB_DEVICE_EPINTENCLR_TRCPT1(value)   (USB_DEVICE_EPINTENCLR_TRCPT1_Msk & (_UINT8_(value) << USB_DEVICE_EPINTENCLR_TRCPT1_Pos)) /* Assigment of value for TRCPT1 in the USB_DEVICE_EPINTENCLR register */
#define USB_DEVICE_EPINTENCLR_TRFAIL0_Pos     _UINT8_(2)                                           /* (USB_DEVICE_EPINTENCLR) Error Flow 0 Interrupt Disable Position */
#define USB_DEVICE_EPINTENCLR_TRFAIL0_Msk     (_UINT8_(0x1) << USB_DEVICE_EPINTENCLR_TRFAIL0_Pos)  /* (USB_DEVICE_EPINTENCLR) Error Flow 0 Interrupt Disable Mask */
#define USB_DEVICE_EPINTENCLR_TRFAIL0(value)  (USB_DEVICE_EPINTENCLR_TRFAIL0_Msk & (_UINT8_(value) << USB_DEVICE_EPINTENCLR_TRFAIL0_Pos)) /* Assigment of value for TRFAIL0 in the USB_DEVICE_EPINTENCLR register */
#define USB_DEVICE_EPINTENCLR_TRFAIL1_Pos     _UINT8_(3)                                           /* (USB_DEVICE_EPINTENCLR) Error Flow 1 Interrupt Disable Position */
#define USB_DEVICE_EPINTENCLR_TRFAIL1_Msk     (_UINT8_(0x1) << USB_DEVICE_EPINTENCLR_TRFAIL1_Pos)  /* (USB_DEVICE_EPINTENCLR) Error Flow 1 Interrupt Disable Mask */
#define USB_DEVICE_EPINTENCLR_TRFAIL1(value)  (USB_DEVICE_EPINTENCLR_TRFAIL1_Msk & (_UINT8_(value) << USB_DEVICE_EPINTENCLR_TRFAIL1_Pos)) /* Assigment of value for TRFAIL1 in the USB_DEVICE_EPINTENCLR register */
#define USB_DEVICE_EPINTENCLR_RXSTP_Pos       _UINT8_(4)                                           /* (USB_DEVICE_EPINTENCLR) Received Setup Interrupt Disable Position */
#define USB_DEVICE_EPINTENCLR_RXSTP_Msk       (_UINT8_(0x1) << USB_DEVICE_EPINTENCLR_RXSTP_Pos)    /* (USB_DEVICE_EPINTENCLR) Received Setup Interrupt Disable Mask */
#define USB_DEVICE_EPINTENCLR_RXSTP(value)    (USB_DEVICE_EPINTENCLR_RXSTP_Msk & (_UINT8_(value) << USB_DEVICE_EPINTENCLR_RXSTP_Pos)) /* Assigment of value for RXSTP in the USB_DEVICE_EPINTENCLR register */
#define USB_DEVICE_EPINTENCLR_STALL0_Pos      _UINT8_(5)                                           /* (USB_DEVICE_EPINTENCLR) Stall 0 In/Out Interrupt Disable Position */
#define USB_DEVICE_EPINTENCLR_STALL0_Msk      (_UINT8_(0x1) << USB_DEVICE_EPINTENCLR_STALL0_Pos)   /* (USB_DEVICE_EPINTENCLR) Stall 0 In/Out Interrupt Disable Mask */
#define USB_DEVICE_EPINTENCLR_STALL0(value)   (USB_DEVICE_EPINTENCLR_STALL0_Msk & (_UINT8_(value) << USB_DEVICE_EPINTENCLR_STALL0_Pos)) /* Assigment of value for STALL0 in the USB_DEVICE_EPINTENCLR register */
#define USB_DEVICE_EPINTENCLR_STALL1_Pos      _UINT8_(6)                                           /* (USB_DEVICE_EPINTENCLR) Stall 1 In/Out Interrupt Disable Position */
#define USB_DEVICE_EPINTENCLR_STALL1_Msk      (_UINT8_(0x1) << USB_DEVICE_EPINTENCLR_STALL1_Pos)   /* (USB_DEVICE_EPINTENCLR) Stall 1 In/Out Interrupt Disable Mask */
#define USB_DEVICE_EPINTENCLR_STALL1(value)   (USB_DEVICE_EPINTENCLR_STALL1_Msk & (_UINT8_(value) << USB_DEVICE_EPINTENCLR_STALL1_Pos)) /* Assigment of value for STALL1 in the USB_DEVICE_EPINTENCLR register */
#define USB_DEVICE_EPINTENCLR_Msk             _UINT8_(0x7F)                                        /* (USB_DEVICE_EPINTENCLR) Register Mask  */

#define USB_DEVICE_EPINTENCLR_TRCPT_Pos       _UINT8_(0)                                           /* (USB_DEVICE_EPINTENCLR Position) Transfer Complete x Interrupt Disable */
#define USB_DEVICE_EPINTENCLR_TRCPT_Msk       (_UINT8_(0x3) << USB_DEVICE_EPINTENCLR_TRCPT_Pos)    /* (USB_DEVICE_EPINTENCLR Mask) TRCPT */
#define USB_DEVICE_EPINTENCLR_TRCPT(value)    (USB_DEVICE_EPINTENCLR_TRCPT_Msk & (_UINT8_(value) << USB_DEVICE_EPINTENCLR_TRCPT_Pos)) 
#define USB_DEVICE_EPINTENCLR_TRFAIL_Pos      _UINT8_(2)                                           /* (USB_DEVICE_EPINTENCLR Position) Error Flow x Interrupt Disable */
#define USB_DEVICE_EPINTENCLR_TRFAIL_Msk      (_UINT8_(0x3) << USB_DEVICE_EPINTENCLR_TRFAIL_Pos)   /* (USB_DEVICE_EPINTENCLR Mask) TRFAIL */
#define USB_DEVICE_EPINTENCLR_TRFAIL(value)   (USB_DEVICE_EPINTENCLR_TRFAIL_Msk & (_UINT8_(value) << USB_DEVICE_EPINTENCLR_TRFAIL_Pos)) 
#define USB_DEVICE_EPINTENCLR_STALL_Pos       _UINT8_(5)                                           /* (USB_DEVICE_EPINTENCLR Position) Stall x In/Out Interrupt Disable */
#define USB_DEVICE_EPINTENCLR_STALL_Msk       (_UINT8_(0x3) << USB_DEVICE_EPINTENCLR_STALL_Pos)    /* (USB_DEVICE_EPINTENCLR Mask) STALL */
#define USB_DEVICE_EPINTENCLR_STALL(value)    (USB_DEVICE_EPINTENCLR_STALL_Msk & (_UINT8_(value) << USB_DEVICE_EPINTENCLR_STALL_Pos)) 

/* -------- USB_DEVICE_EPINTENSET : (USB Offset: 0x09) (R/W 8) DEVICE_ENDPOINT End Point Interrupt Set Flag -------- */
#define USB_DEVICE_EPINTENSET_RESETVALUE      _UINT8_(0x00)                                        /*  (USB_DEVICE_EPINTENSET) DEVICE_ENDPOINT End Point Interrupt Set Flag  Reset Value */

#define USB_DEVICE_EPINTENSET_TRCPT0_Pos      _UINT8_(0)                                           /* (USB_DEVICE_EPINTENSET) Transfer Complete 0 Interrupt Enable Position */
#define USB_DEVICE_EPINTENSET_TRCPT0_Msk      (_UINT8_(0x1) << USB_DEVICE_EPINTENSET_TRCPT0_Pos)   /* (USB_DEVICE_EPINTENSET) Transfer Complete 0 Interrupt Enable Mask */
#define USB_DEVICE_EPINTENSET_TRCPT0(value)   (USB_DEVICE_EPINTENSET_TRCPT0_Msk & (_UINT8_(value) << USB_DEVICE_EPINTENSET_TRCPT0_Pos)) /* Assigment of value for TRCPT0 in the USB_DEVICE_EPINTENSET register */
#define USB_DEVICE_EPINTENSET_TRCPT1_Pos      _UINT8_(1)                                           /* (USB_DEVICE_EPINTENSET) Transfer Complete 1 Interrupt Enable Position */
#define USB_DEVICE_EPINTENSET_TRCPT1_Msk      (_UINT8_(0x1) << USB_DEVICE_EPINTENSET_TRCPT1_Pos)   /* (USB_DEVICE_EPINTENSET) Transfer Complete 1 Interrupt Enable Mask */
#define USB_DEVICE_EPINTENSET_TRCPT1(value)   (USB_DEVICE_EPINTENSET_TRCPT1_Msk & (_UINT8_(value) << USB_DEVICE_EPINTENSET_TRCPT1_Pos)) /* Assigment of value for TRCPT1 in the USB_DEVICE_EPINTENSET register */
#define USB_DEVICE_EPINTENSET_TRFAIL0_Pos     _UINT8_(2)                                           /* (USB_DEVICE_EPINTENSET) Error Flow 0 Interrupt Enable Position */
#define USB_DEVICE_EPINTENSET_TRFAIL0_Msk     (_UINT8_(0x1) << USB_DEVICE_EPINTENSET_TRFAIL0_Pos)  /* (USB_DEVICE_EPINTENSET) Error Flow 0 Interrupt Enable Mask */
#define USB_DEVICE_EPINTENSET_TRFAIL0(value)  (USB_DEVICE_EPINTENSET_TRFAIL0_Msk & (_UINT8_(value) << USB_DEVICE_EPINTENSET_TRFAIL0_Pos)) /* Assigment of value for TRFAIL0 in the USB_DEVICE_EPINTENSET register */
#define USB_DEVICE_EPINTENSET_TRFAIL1_Pos     _UINT8_(3)                                           /* (USB_DEVICE_EPINTENSET) Error Flow 1 Interrupt Enable Position */
#define USB_DEVICE_EPINTENSET_TRFAIL1_Msk     (_UINT8_(0x1) << USB_DEVICE_EPINTENSET_TRFAIL1_Pos)  /* (USB_DEVICE_EPINTENSET) Error Flow 1 Interrupt Enable Mask */
#define USB_DEVICE_EPINTENSET_TRFAIL1(value)  (USB_DEVICE_EPINTENSET_TRFAIL1_Msk & (_UINT8_(value) << USB_DEVICE_EPINTENSET_TRFAIL1_Pos)) /* Assigment of value for TRFAIL1 in the USB_DEVICE_EPINTENSET register */
#define USB_DEVICE_EPINTENSET_RXSTP_Pos       _UINT8_(4)                                           /* (USB_DEVICE_EPINTENSET) Received Setup Interrupt Enable Position */
#define USB_DEVICE_EPINTENSET_RXSTP_Msk       (_UINT8_(0x1) << USB_DEVICE_EPINTENSET_RXSTP_Pos)    /* (USB_DEVICE_EPINTENSET) Received Setup Interrupt Enable Mask */
#define USB_DEVICE_EPINTENSET_RXSTP(value)    (USB_DEVICE_EPINTENSET_RXSTP_Msk & (_UINT8_(value) << USB_DEVICE_EPINTENSET_RXSTP_Pos)) /* Assigment of value for RXSTP in the USB_DEVICE_EPINTENSET register */
#define USB_DEVICE_EPINTENSET_STALL0_Pos      _UINT8_(5)                                           /* (USB_DEVICE_EPINTENSET) Stall 0 In/out Interrupt enable Position */
#define USB_DEVICE_EPINTENSET_STALL0_Msk      (_UINT8_(0x1) << USB_DEVICE_EPINTENSET_STALL0_Pos)   /* (USB_DEVICE_EPINTENSET) Stall 0 In/out Interrupt enable Mask */
#define USB_DEVICE_EPINTENSET_STALL0(value)   (USB_DEVICE_EPINTENSET_STALL0_Msk & (_UINT8_(value) << USB_DEVICE_EPINTENSET_STALL0_Pos)) /* Assigment of value for STALL0 in the USB_DEVICE_EPINTENSET register */
#define USB_DEVICE_EPINTENSET_STALL1_Pos      _UINT8_(6)                                           /* (USB_DEVICE_EPINTENSET) Stall 1 In/out Interrupt enable Position */
#define USB_DEVICE_EPINTENSET_STALL1_Msk      (_UINT8_(0x1) << USB_DEVICE_EPINTENSET_STALL1_Pos)   /* (USB_DEVICE_EPINTENSET) Stall 1 In/out Interrupt enable Mask */
#define USB_DEVICE_EPINTENSET_STALL1(value)   (USB_DEVICE_EPINTENSET_STALL1_Msk & (_UINT8_(value) << USB_DEVICE_EPINTENSET_STALL1_Pos)) /* Assigment of value for STALL1 in the USB_DEVICE_EPINTENSET register */
#define USB_DEVICE_EPINTENSET_Msk             _UINT8_(0x7F)                                        /* (USB_DEVICE_EPINTENSET) Register Mask  */

#define USB_DEVICE_EPINTENSET_TRCPT_Pos       _UINT8_(0)                                           /* (USB_DEVICE_EPINTENSET Position) Transfer Complete x Interrupt Enable */
#define USB_DEVICE_EPINTENSET_TRCPT_Msk       (_UINT8_(0x3) << USB_DEVICE_EPINTENSET_TRCPT_Pos)    /* (USB_DEVICE_EPINTENSET Mask) TRCPT */
#define USB_DEVICE_EPINTENSET_TRCPT(value)    (USB_DEVICE_EPINTENSET_TRCPT_Msk & (_UINT8_(value) << USB_DEVICE_EPINTENSET_TRCPT_Pos)) 
#define USB_DEVICE_EPINTENSET_TRFAIL_Pos      _UINT8_(2)                                           /* (USB_DEVICE_EPINTENSET Position) Error Flow x Interrupt Enable */
#define USB_DEVICE_EPINTENSET_TRFAIL_Msk      (_UINT8_(0x3) << USB_DEVICE_EPINTENSET_TRFAIL_Pos)   /* (USB_DEVICE_EPINTENSET Mask) TRFAIL */
#define USB_DEVICE_EPINTENSET_TRFAIL(value)   (USB_DEVICE_EPINTENSET_TRFAIL_Msk & (_UINT8_(value) << USB_DEVICE_EPINTENSET_TRFAIL_Pos)) 
#define USB_DEVICE_EPINTENSET_STALL_Pos       _UINT8_(5)                                           /* (USB_DEVICE_EPINTENSET Position) Stall x In/out Interrupt enable */
#define USB_DEVICE_EPINTENSET_STALL_Msk       (_UINT8_(0x3) << USB_DEVICE_EPINTENSET_STALL_Pos)    /* (USB_DEVICE_EPINTENSET Mask) STALL */
#define USB_DEVICE_EPINTENSET_STALL(value)    (USB_DEVICE_EPINTENSET_STALL_Msk & (_UINT8_(value) << USB_DEVICE_EPINTENSET_STALL_Pos)) 

/* -------- USB_CTRLA : (USB Offset: 0x00) (R/W 8) Control A -------- */
#define USB_CTRLA_RESETVALUE                  _UINT8_(0x00)                                        /*  (USB_CTRLA) Control A  Reset Value */

#define USB_CTRLA_SWRST_Pos                   _UINT8_(0)                                           /* (USB_CTRLA) Software Reset Position */
#define USB_CTRLA_SWRST_Msk                   (_UINT8_(0x1) << USB_CTRLA_SWRST_Pos)                /* (USB_CTRLA) Software Reset Mask */
#define USB_CTRLA_SWRST(value)                (USB_CTRLA_SWRST_Msk & (_UINT8_(value) << USB_CTRLA_SWRST_Pos)) /* Assigment of value for SWRST in the USB_CTRLA register */
#define USB_CTRLA_ENABLE_Pos                  _UINT8_(1)                                           /* (USB_CTRLA) Enable Position */
#define USB_CTRLA_ENABLE_Msk                  (_UINT8_(0x1) << USB_CTRLA_ENABLE_Pos)               /* (USB_CTRLA) Enable Mask */
#define USB_CTRLA_ENABLE(value)               (USB_CTRLA_ENABLE_Msk & (_UINT8_(value) << USB_CTRLA_ENABLE_Pos)) /* Assigment of value for ENABLE in the USB_CTRLA register */
#define USB_CTRLA_RUNSTDBY_Pos                _UINT8_(2)                                           /* (USB_CTRLA) Run in Standby Mode Position */
#define USB_CTRLA_RUNSTDBY_Msk                (_UINT8_(0x1) << USB_CTRLA_RUNSTDBY_Pos)             /* (USB_CTRLA) Run in Standby Mode Mask */
#define USB_CTRLA_RUNSTDBY(value)             (USB_CTRLA_RUNSTDBY_Msk & (_UINT8_(value) << USB_CTRLA_RUNSTDBY_Pos)) /* Assigment of value for RUNSTDBY in the USB_CTRLA register */
#define USB_CTRLA_MODE_Pos                    _UINT8_(7)                                           /* (USB_CTRLA) Operating Mode Position */
#define USB_CTRLA_MODE_Msk                    (_UINT8_(0x1) << USB_CTRLA_MODE_Pos)                 /* (USB_CTRLA) Operating Mode Mask */
#define USB_CTRLA_MODE(value)                 (USB_CTRLA_MODE_Msk & (_UINT8_(value) << USB_CTRLA_MODE_Pos)) /* Assigment of value for MODE in the USB_CTRLA register */
#define   USB_CTRLA_MODE_DEVICE_Val           _UINT8_(0x0)                                         /* (USB_CTRLA) Device Mode  */
#define USB_CTRLA_MODE_DEVICE                 (USB_CTRLA_MODE_DEVICE_Val << USB_CTRLA_MODE_Pos)    /* (USB_CTRLA) Device Mode Position  */
#define USB_CTRLA_Msk                         _UINT8_(0x87)                                        /* (USB_CTRLA) Register Mask  */


/* -------- USB_SYNCBUSY : (USB Offset: 0x02) ( R/ 8) Synchronization Busy -------- */
#define USB_SYNCBUSY_RESETVALUE               _UINT8_(0x00)                                        /*  (USB_SYNCBUSY) Synchronization Busy  Reset Value */

#define USB_SYNCBUSY_SWRST_Pos                _UINT8_(0)                                           /* (USB_SYNCBUSY) Software Reset Synchronization Busy Position */
#define USB_SYNCBUSY_SWRST_Msk                (_UINT8_(0x1) << USB_SYNCBUSY_SWRST_Pos)             /* (USB_SYNCBUSY) Software Reset Synchronization Busy Mask */
#define USB_SYNCBUSY_SWRST(value)             (USB_SYNCBUSY_SWRST_Msk & (_UINT8_(value) << USB_SYNCBUSY_SWRST_Pos)) /* Assigment of value for SWRST in the USB_SYNCBUSY register */
#define USB_SYNCBUSY_ENABLE_Pos               _UINT8_(1)                                           /* (USB_SYNCBUSY) Enable Synchronization Busy Position */
#define USB_SYNCBUSY_ENABLE_Msk               (_UINT8_(0x1) << USB_SYNCBUSY_ENABLE_Pos)            /* (USB_SYNCBUSY) Enable Synchronization Busy Mask */
#define USB_SYNCBUSY_ENABLE(value)            (USB_SYNCBUSY_ENABLE_Msk & (_UINT8_(value) << USB_SYNCBUSY_ENABLE_Pos)) /* Assigment of value for ENABLE in the USB_SYNCBUSY register */
#define USB_SYNCBUSY_Msk                      _UINT8_(0x03)                                        /* (USB_SYNCBUSY) Register Mask  */


/* -------- USB_QOSCTRL : (USB Offset: 0x03) (R/W 8) USB Quality Of Service -------- */
#define USB_QOSCTRL_RESETVALUE                _UINT8_(0x05)                                        /*  (USB_QOSCTRL) USB Quality Of Service  Reset Value */

#define USB_QOSCTRL_CQOS_Pos                  _UINT8_(0)                                           /* (USB_QOSCTRL) Configuration Quality of Service Position */
#define USB_QOSCTRL_CQOS_Msk                  (_UINT8_(0x3) << USB_QOSCTRL_CQOS_Pos)               /* (USB_QOSCTRL) Configuration Quality of Service Mask */
#define USB_QOSCTRL_CQOS(value)               (USB_QOSCTRL_CQOS_Msk & (_UINT8_(value) << USB_QOSCTRL_CQOS_Pos)) /* Assigment of value for CQOS in the USB_QOSCTRL register */
#define USB_QOSCTRL_DQOS_Pos                  _UINT8_(2)                                           /* (USB_QOSCTRL) Data Quality of Service Position */
#define USB_QOSCTRL_DQOS_Msk                  (_UINT8_(0x3) << USB_QOSCTRL_DQOS_Pos)               /* (USB_QOSCTRL) Data Quality of Service Mask */
#define USB_QOSCTRL_DQOS(value)               (USB_QOSCTRL_DQOS_Msk & (_UINT8_(value) << USB_QOSCTRL_DQOS_Pos)) /* Assigment of value for DQOS in the USB_QOSCTRL register */
#define USB_QOSCTRL_Msk                       _UINT8_(0x0F)                                        /* (USB_QOSCTRL) Register Mask  */


/* -------- USB_DEVICE_CTRLB : (USB Offset: 0x08) (R/W 16) DEVICE Control B -------- */
#define USB_DEVICE_CTRLB_RESETVALUE           _UINT16_(0x01)                                       /*  (USB_DEVICE_CTRLB) DEVICE Control B  Reset Value */

#define USB_DEVICE_CTRLB_DETACH_Pos           _UINT16_(0)                                          /* (USB_DEVICE_CTRLB) Detach Position */
#define USB_DEVICE_CTRLB_DETACH_Msk           (_UINT16_(0x1) << USB_DEVICE_CTRLB_DETACH_Pos)       /* (USB_DEVICE_CTRLB) Detach Mask */
#define USB_DEVICE_CTRLB_DETACH(value)        (USB_DEVICE_CTRLB_DETACH_Msk & (_UINT16_(value) << USB_DEVICE_CTRLB_DETACH_Pos)) /* Assigment of value for DETACH in the USB_DEVICE_CTRLB register */
#define USB_DEVICE_CTRLB_UPRSM_Pos            _UINT16_(1)                                          /* (USB_DEVICE_CTRLB) Upstream Resume Position */
#define USB_DEVICE_CTRLB_UPRSM_Msk            (_UINT16_(0x1) << USB_DEVICE_CTRLB_UPRSM_Pos)        /* (USB_DEVICE_CTRLB) Upstream Resume Mask */
#define USB_DEVICE_CTRLB_UPRSM(value)         (USB_DEVICE_CTRLB_UPRSM_Msk & (_UINT16_(value) << USB_DEVICE_CTRLB_UPRSM_Pos)) /* Assigment of value for UPRSM in the USB_DEVICE_CTRLB register */
#define USB_DEVICE_CTRLB_SPDCONF_Pos          _UINT16_(2)                                          /* (USB_DEVICE_CTRLB) Speed Configuration Position */
#define USB_DEVICE_CTRLB_SPDCONF_Msk          (_UINT16_(0x3) << USB_DEVICE_CTRLB_SPDCONF_Pos)      /* (USB_DEVICE_CTRLB) Speed Configuration Mask */
#define USB_DEVICE_CTRLB_SPDCONF(value)       (USB_DEVICE_CTRLB_SPDCONF_Msk & (_UINT16_(value) << USB_DEVICE_CTRLB_SPDCONF_Pos)) /* Assigment of value for SPDCONF in the USB_DEVICE_CTRLB register */
#define   USB_DEVICE_CTRLB_SPDCONF_FS_Val     _UINT16_(0x0)                                        /* (USB_DEVICE_CTRLB) FS : Full Speed  */
#define   USB_DEVICE_CTRLB_SPDCONF_LS_Val     _UINT16_(0x1)                                        /* (USB_DEVICE_CTRLB) LS : Low Speed  */
#define   USB_DEVICE_CTRLB_SPDCONF_HS_Val     _UINT16_(0x2)                                        /* (USB_DEVICE_CTRLB) HS : High Speed capable  */
#define   USB_DEVICE_CTRLB_SPDCONF_HSTM_Val   _UINT16_(0x3)                                        /* (USB_DEVICE_CTRLB) HSTM: High Speed Test Mode (force high-speed mode for test mode)  */
#define USB_DEVICE_CTRLB_SPDCONF_FS           (USB_DEVICE_CTRLB_SPDCONF_FS_Val << USB_DEVICE_CTRLB_SPDCONF_Pos) /* (USB_DEVICE_CTRLB) FS : Full Speed Position  */
#define USB_DEVICE_CTRLB_SPDCONF_LS           (USB_DEVICE_CTRLB_SPDCONF_LS_Val << USB_DEVICE_CTRLB_SPDCONF_Pos) /* (USB_DEVICE_CTRLB) LS : Low Speed Position  */
#define USB_DEVICE_CTRLB_SPDCONF_HS           (USB_DEVICE_CTRLB_SPDCONF_HS_Val << USB_DEVICE_CTRLB_SPDCONF_Pos) /* (USB_DEVICE_CTRLB) HS : High Speed capable Position  */
#define USB_DEVICE_CTRLB_SPDCONF_HSTM         (USB_DEVICE_CTRLB_SPDCONF_HSTM_Val << USB_DEVICE_CTRLB_SPDCONF_Pos) /* (USB_DEVICE_CTRLB) HSTM: High Speed Test Mode (force high-speed mode for test mode) Position  */
#define USB_DEVICE_CTRLB_NREPLY_Pos           _UINT16_(4)                                          /* (USB_DEVICE_CTRLB) No Reply Position */
#define USB_DEVICE_CTRLB_NREPLY_Msk           (_UINT16_(0x1) << USB_DEVICE_CTRLB_NREPLY_Pos)       /* (USB_DEVICE_CTRLB) No Reply Mask */
#define USB_DEVICE_CTRLB_NREPLY(value)        (USB_DEVICE_CTRLB_NREPLY_Msk & (_UINT16_(value) << USB_DEVICE_CTRLB_NREPLY_Pos)) /* Assigment of value for NREPLY in the USB_DEVICE_CTRLB register */
#define USB_DEVICE_CTRLB_TSTJ_Pos             _UINT16_(5)                                          /* (USB_DEVICE_CTRLB) Test mode J Position */
#define USB_DEVICE_CTRLB_TSTJ_Msk             (_UINT16_(0x1) << USB_DEVICE_CTRLB_TSTJ_Pos)         /* (USB_DEVICE_CTRLB) Test mode J Mask */
#define USB_DEVICE_CTRLB_TSTJ(value)          (USB_DEVICE_CTRLB_TSTJ_Msk & (_UINT16_(value) << USB_DEVICE_CTRLB_TSTJ_Pos)) /* Assigment of value for TSTJ in the USB_DEVICE_CTRLB register */
#define USB_DEVICE_CTRLB_TSTK_Pos             _UINT16_(6)                                          /* (USB_DEVICE_CTRLB) Test mode K Position */
#define USB_DEVICE_CTRLB_TSTK_Msk             (_UINT16_(0x1) << USB_DEVICE_CTRLB_TSTK_Pos)         /* (USB_DEVICE_CTRLB) Test mode K Mask */
#define USB_DEVICE_CTRLB_TSTK(value)          (USB_DEVICE_CTRLB_TSTK_Msk & (_UINT16_(value) << USB_DEVICE_CTRLB_TSTK_Pos)) /* Assigment of value for TSTK in the USB_DEVICE_CTRLB register */
#define USB_DEVICE_CTRLB_TSTPCKT_Pos          _UINT16_(7)                                          /* (USB_DEVICE_CTRLB) Test packet mode Position */
#define USB_DEVICE_CTRLB_TSTPCKT_Msk          (_UINT16_(0x1) << USB_DEVICE_CTRLB_TSTPCKT_Pos)      /* (USB_DEVICE_CTRLB) Test packet mode Mask */
#define USB_DEVICE_CTRLB_TSTPCKT(value)       (USB_DEVICE_CTRLB_TSTPCKT_Msk & (_UINT16_(value) << USB_DEVICE_CTRLB_TSTPCKT_Pos)) /* Assigment of value for TSTPCKT in the USB_DEVICE_CTRLB register */
#define USB_DEVICE_CTRLB_OPMODE2_Pos          _UINT16_(8)                                          /* (USB_DEVICE_CTRLB) Specific Operational Mode Position */
#define USB_DEVICE_CTRLB_OPMODE2_Msk          (_UINT16_(0x1) << USB_DEVICE_CTRLB_OPMODE2_Pos)      /* (USB_DEVICE_CTRLB) Specific Operational Mode Mask */
#define USB_DEVICE_CTRLB_OPMODE2(value)       (USB_DEVICE_CTRLB_OPMODE2_Msk & (_UINT16_(value) << USB_DEVICE_CTRLB_OPMODE2_Pos)) /* Assigment of value for OPMODE2 in the USB_DEVICE_CTRLB register */
#define USB_DEVICE_CTRLB_GNAK_Pos             _UINT16_(9)                                          /* (USB_DEVICE_CTRLB) Global NAK Position */
#define USB_DEVICE_CTRLB_GNAK_Msk             (_UINT16_(0x1) << USB_DEVICE_CTRLB_GNAK_Pos)         /* (USB_DEVICE_CTRLB) Global NAK Mask */
#define USB_DEVICE_CTRLB_GNAK(value)          (USB_DEVICE_CTRLB_GNAK_Msk & (_UINT16_(value) << USB_DEVICE_CTRLB_GNAK_Pos)) /* Assigment of value for GNAK in the USB_DEVICE_CTRLB register */
#define USB_DEVICE_CTRLB_LPMHDSK_Pos          _UINT16_(10)                                         /* (USB_DEVICE_CTRLB) Link Power Management Handshake Position */
#define USB_DEVICE_CTRLB_LPMHDSK_Msk          (_UINT16_(0x3) << USB_DEVICE_CTRLB_LPMHDSK_Pos)      /* (USB_DEVICE_CTRLB) Link Power Management Handshake Mask */
#define USB_DEVICE_CTRLB_LPMHDSK(value)       (USB_DEVICE_CTRLB_LPMHDSK_Msk & (_UINT16_(value) << USB_DEVICE_CTRLB_LPMHDSK_Pos)) /* Assigment of value for LPMHDSK in the USB_DEVICE_CTRLB register */
#define   USB_DEVICE_CTRLB_LPMHDSK_NO_Val     _UINT16_(0x0)                                        /* (USB_DEVICE_CTRLB) No handshake. LPM is not supported  */
#define   USB_DEVICE_CTRLB_LPMHDSK_ACK_Val    _UINT16_(0x1)                                        /* (USB_DEVICE_CTRLB) ACK  */
#define   USB_DEVICE_CTRLB_LPMHDSK_NYET_Val   _UINT16_(0x2)                                        /* (USB_DEVICE_CTRLB) NYET  */
#define   USB_DEVICE_CTRLB_LPMHDSK_STALL_Val  _UINT16_(0x3)                                        /* (USB_DEVICE_CTRLB) STALL  */
#define USB_DEVICE_CTRLB_LPMHDSK_NO           (USB_DEVICE_CTRLB_LPMHDSK_NO_Val << USB_DEVICE_CTRLB_LPMHDSK_Pos) /* (USB_DEVICE_CTRLB) No handshake. LPM is not supported Position  */
#define USB_DEVICE_CTRLB_LPMHDSK_ACK          (USB_DEVICE_CTRLB_LPMHDSK_ACK_Val << USB_DEVICE_CTRLB_LPMHDSK_Pos) /* (USB_DEVICE_CTRLB) ACK Position  */
#define USB_DEVICE_CTRLB_LPMHDSK_NYET         (USB_DEVICE_CTRLB_LPMHDSK_NYET_Val << USB_DEVICE_CTRLB_LPMHDSK_Pos) /* (USB_DEVICE_CTRLB) NYET Position  */
#define USB_DEVICE_CTRLB_LPMHDSK_STALL        (USB_DEVICE_CTRLB_LPMHDSK_STALL_Val << USB_DEVICE_CTRLB_LPMHDSK_Pos) /* (USB_DEVICE_CTRLB) STALL Position  */
#define USB_DEVICE_CTRLB_Msk                  _UINT16_(0x0FFF)                                     /* (USB_DEVICE_CTRLB) Register Mask  */

#define USB_DEVICE_CTRLB_OPMODE_Pos           _UINT16_(8)                                          /* (USB_DEVICE_CTRLB Position) Specific Operational Mode */
#define USB_DEVICE_CTRLB_OPMODE_Msk           (_UINT16_(0x1) << USB_DEVICE_CTRLB_OPMODE_Pos)       /* (USB_DEVICE_CTRLB Mask) OPMODE */
#define USB_DEVICE_CTRLB_OPMODE(value)        (USB_DEVICE_CTRLB_OPMODE_Msk & (_UINT16_(value) << USB_DEVICE_CTRLB_OPMODE_Pos)) 

/* -------- USB_DEVICE_DADD : (USB Offset: 0x0A) (R/W 8) DEVICE Device Address -------- */
#define USB_DEVICE_DADD_RESETVALUE            _UINT8_(0x00)                                        /*  (USB_DEVICE_DADD) DEVICE Device Address  Reset Value */

#define USB_DEVICE_DADD_DADD_Pos              _UINT8_(0)                                           /* (USB_DEVICE_DADD) Device Address Position */
#define USB_DEVICE_DADD_DADD_Msk              (_UINT8_(0x7F) << USB_DEVICE_DADD_DADD_Pos)          /* (USB_DEVICE_DADD) Device Address Mask */
#define USB_DEVICE_DADD_DADD(value)           (USB_DEVICE_DADD_DADD_Msk & (_UINT8_(value) << USB_DEVICE_DADD_DADD_Pos)) /* Assigment of value for DADD in the USB_DEVICE_DADD register */
#define USB_DEVICE_DADD_ADDEN_Pos             _UINT8_(7)                                           /* (USB_DEVICE_DADD) Device Address Enable Position */
#define USB_DEVICE_DADD_ADDEN_Msk             (_UINT8_(0x1) << USB_DEVICE_DADD_ADDEN_Pos)          /* (USB_DEVICE_DADD) Device Address Enable Mask */
#define USB_DEVICE_DADD_ADDEN(value)          (USB_DEVICE_DADD_ADDEN_Msk & (_UINT8_(value) << USB_DEVICE_DADD_ADDEN_Pos)) /* Assigment of value for ADDEN in the USB_DEVICE_DADD register */
#define USB_DEVICE_DADD_Msk                   _UINT8_(0xFF)                                        /* (USB_DEVICE_DADD) Register Mask  */


/* -------- USB_DEVICE_STATUS : (USB Offset: 0x0C) ( R/ 8) DEVICE Status -------- */
#define USB_DEVICE_STATUS_RESETVALUE          _UINT8_(0x40)                                        /*  (USB_DEVICE_STATUS) DEVICE Status  Reset Value */

#define USB_DEVICE_STATUS_SPEED_Pos           _UINT8_(2)                                           /* (USB_DEVICE_STATUS) Speed Status Position */
#define USB_DEVICE_STATUS_SPEED_Msk           (_UINT8_(0x3) << USB_DEVICE_STATUS_SPEED_Pos)        /* (USB_DEVICE_STATUS) Speed Status Mask */
#define USB_DEVICE_STATUS_SPEED(value)        (USB_DEVICE_STATUS_SPEED_Msk & (_UINT8_(value) << USB_DEVICE_STATUS_SPEED_Pos)) /* Assigment of value for SPEED in the USB_DEVICE_STATUS register */
#define   USB_DEVICE_STATUS_SPEED_FS_Val      _UINT8_(0x0)                                         /* (USB_DEVICE_STATUS) Full-speed mode  */
#define   USB_DEVICE_STATUS_SPEED_HS_Val      _UINT8_(0x1)                                         /* (USB_DEVICE_STATUS) High-speed mode  */
#define   USB_DEVICE_STATUS_SPEED_LS_Val      _UINT8_(0x2)                                         /* (USB_DEVICE_STATUS) Low-speed mode  */
#define USB_DEVICE_STATUS_SPEED_FS            (USB_DEVICE_STATUS_SPEED_FS_Val << USB_DEVICE_STATUS_SPEED_Pos) /* (USB_DEVICE_STATUS) Full-speed mode Position  */
#define USB_DEVICE_STATUS_SPEED_HS            (USB_DEVICE_STATUS_SPEED_HS_Val << USB_DEVICE_STATUS_SPEED_Pos) /* (USB_DEVICE_STATUS) High-speed mode Position  */
#define USB_DEVICE_STATUS_SPEED_LS            (USB_DEVICE_STATUS_SPEED_LS_Val << USB_DEVICE_STATUS_SPEED_Pos) /* (USB_DEVICE_STATUS) Low-speed mode Position  */
#define USB_DEVICE_STATUS_LINESTATE_Pos       _UINT8_(6)                                           /* (USB_DEVICE_STATUS) USB Line State Status Position */
#define USB_DEVICE_STATUS_LINESTATE_Msk       (_UINT8_(0x3) << USB_DEVICE_STATUS_LINESTATE_Pos)    /* (USB_DEVICE_STATUS) USB Line State Status Mask */
#define USB_DEVICE_STATUS_LINESTATE(value)    (USB_DEVICE_STATUS_LINESTATE_Msk & (_UINT8_(value) << USB_DEVICE_STATUS_LINESTATE_Pos)) /* Assigment of value for LINESTATE in the USB_DEVICE_STATUS register */
#define   USB_DEVICE_STATUS_LINESTATE_0_Val   _UINT8_(0x0)                                         /* (USB_DEVICE_STATUS) SE0/RESET  */
#define   USB_DEVICE_STATUS_LINESTATE_1_Val   _UINT8_(0x1)                                         /* (USB_DEVICE_STATUS) FS-J or LS-K State  */
#define   USB_DEVICE_STATUS_LINESTATE_2_Val   _UINT8_(0x2)                                         /* (USB_DEVICE_STATUS) FS-K or LS-J State  */
#define USB_DEVICE_STATUS_LINESTATE_0         (USB_DEVICE_STATUS_LINESTATE_0_Val << USB_DEVICE_STATUS_LINESTATE_Pos) /* (USB_DEVICE_STATUS) SE0/RESET Position  */
#define USB_DEVICE_STATUS_LINESTATE_1         (USB_DEVICE_STATUS_LINESTATE_1_Val << USB_DEVICE_STATUS_LINESTATE_Pos) /* (USB_DEVICE_STATUS) FS-J or LS-K State Position  */
#define USB_DEVICE_STATUS_LINESTATE_2         (USB_DEVICE_STATUS_LINESTATE_2_Val << USB_DEVICE_STATUS_LINESTATE_Pos) /* (USB_DEVICE_STATUS) FS-K or LS-J State Position  */
#define USB_DEVICE_STATUS_Msk                 _UINT8_(0xCC)                                        /* (USB_DEVICE_STATUS) Register Mask  */


/* -------- USB_FSMSTATUS : (USB Offset: 0x0D) ( R/ 8) Finite State Machine Status -------- */
#define USB_FSMSTATUS_RESETVALUE              _UINT8_(0x01)                                        /*  (USB_FSMSTATUS) Finite State Machine Status  Reset Value */

#define USB_FSMSTATUS_FSMSTATE_Pos            _UINT8_(0)                                           /* (USB_FSMSTATUS) Fine State Machine Status Position */
#define USB_FSMSTATUS_FSMSTATE_Msk            (_UINT8_(0x3F) << USB_FSMSTATUS_FSMSTATE_Pos)        /* (USB_FSMSTATUS) Fine State Machine Status Mask */
#define USB_FSMSTATUS_FSMSTATE(value)         (USB_FSMSTATUS_FSMSTATE_Msk & (_UINT8_(value) << USB_FSMSTATUS_FSMSTATE_Pos)) /* Assigment of value for FSMSTATE in the USB_FSMSTATUS register */
#define   USB_FSMSTATUS_FSMSTATE_OFF_Val      _UINT8_(0x1)                                         /* (USB_FSMSTATUS) OFF (L3). It corresponds to the powered-off, disconnected, and disabled state  */
#define   USB_FSMSTATUS_FSMSTATE_ON_Val       _UINT8_(0x2)                                         /* (USB_FSMSTATUS) ON (L0). It corresponds to the Idle and Active states  */
#define   USB_FSMSTATUS_FSMSTATE_SUSPEND_Val  _UINT8_(0x4)                                         /* (USB_FSMSTATUS) SUSPEND (L2)  */
#define   USB_FSMSTATUS_FSMSTATE_SLEEP_Val    _UINT8_(0x8)                                         /* (USB_FSMSTATUS) SLEEP (L1)  */
#define   USB_FSMSTATUS_FSMSTATE_DNRESUME_Val _UINT8_(0x10)                                        /* (USB_FSMSTATUS) DNRESUME. Down Stream Resume.  */
#define   USB_FSMSTATUS_FSMSTATE_UPRESUME_Val _UINT8_(0x20)                                        /* (USB_FSMSTATUS) UPRESUME. Up Stream Resume.  */
#define   USB_FSMSTATUS_FSMSTATE_RESET_Val    _UINT8_(0x40)                                        /* (USB_FSMSTATUS) RESET. USB lines Reset.  */
#define USB_FSMSTATUS_FSMSTATE_OFF            (USB_FSMSTATUS_FSMSTATE_OFF_Val << USB_FSMSTATUS_FSMSTATE_Pos) /* (USB_FSMSTATUS) OFF (L3). It corresponds to the powered-off, disconnected, and disabled state Position  */
#define USB_FSMSTATUS_FSMSTATE_ON             (USB_FSMSTATUS_FSMSTATE_ON_Val << USB_FSMSTATUS_FSMSTATE_Pos) /* (USB_FSMSTATUS) ON (L0). It corresponds to the Idle and Active states Position  */
#define USB_FSMSTATUS_FSMSTATE_SUSPEND        (USB_FSMSTATUS_FSMSTATE_SUSPEND_Val << USB_FSMSTATUS_FSMSTATE_Pos) /* (USB_FSMSTATUS) SUSPEND (L2) Position  */
#define USB_FSMSTATUS_FSMSTATE_SLEEP          (USB_FSMSTATUS_FSMSTATE_SLEEP_Val << USB_FSMSTATUS_FSMSTATE_Pos) /* (USB_FSMSTATUS) SLEEP (L1) Position  */
#define USB_FSMSTATUS_FSMSTATE_DNRESUME       (USB_FSMSTATUS_FSMSTATE_DNRESUME_Val << USB_FSMSTATUS_FSMSTATE_Pos) /* (USB_FSMSTATUS) DNRESUME. Down Stream Resume. Position  */
#define USB_FSMSTATUS_FSMSTATE_UPRESUME       (USB_FSMSTATUS_FSMSTATE_UPRESUME_Val << USB_FSMSTATUS_FSMSTATE_Pos) /* (USB_FSMSTATUS) UPRESUME. Up Stream Resume. Position  */
#define USB_FSMSTATUS_FSMSTATE_RESET          (USB_FSMSTATUS_FSMSTATE_RESET_Val << USB_FSMSTATUS_FSMSTATE_Pos) /* (USB_FSMSTATUS) RESET. USB lines Reset. Position  */
#define USB_FSMSTATUS_Msk                     _UINT8_(0x3F)                                        /* (USB_FSMSTATUS) Register Mask  */


/* -------- USB_DEVICE_FNUM : (USB Offset: 0x10) ( R/ 16) DEVICE Device Frame Number -------- */
#define USB_DEVICE_FNUM_RESETVALUE            _UINT16_(0x00)                                       /*  (USB_DEVICE_FNUM) DEVICE Device Frame Number  Reset Value */

#define USB_DEVICE_FNUM_MFNUM_Pos             _UINT16_(0)                                          /* (USB_DEVICE_FNUM) Micro Frame Number Position */
#define USB_DEVICE_FNUM_MFNUM_Msk             (_UINT16_(0x7) << USB_DEVICE_FNUM_MFNUM_Pos)         /* (USB_DEVICE_FNUM) Micro Frame Number Mask */
#define USB_DEVICE_FNUM_MFNUM(value)          (USB_DEVICE_FNUM_MFNUM_Msk & (_UINT16_(value) << USB_DEVICE_FNUM_MFNUM_Pos)) /* Assigment of value for MFNUM in the USB_DEVICE_FNUM register */
#define USB_DEVICE_FNUM_FNUM_Pos              _UINT16_(3)                                          /* (USB_DEVICE_FNUM) Frame Number Position */
#define USB_DEVICE_FNUM_FNUM_Msk              (_UINT16_(0x7FF) << USB_DEVICE_FNUM_FNUM_Pos)        /* (USB_DEVICE_FNUM) Frame Number Mask */
#define USB_DEVICE_FNUM_FNUM(value)           (USB_DEVICE_FNUM_FNUM_Msk & (_UINT16_(value) << USB_DEVICE_FNUM_FNUM_Pos)) /* Assigment of value for FNUM in the USB_DEVICE_FNUM register */
#define USB_DEVICE_FNUM_FNCERR_Pos            _UINT16_(15)                                         /* (USB_DEVICE_FNUM) Frame Number CRC Error Position */
#define USB_DEVICE_FNUM_FNCERR_Msk            (_UINT16_(0x1) << USB_DEVICE_FNUM_FNCERR_Pos)        /* (USB_DEVICE_FNUM) Frame Number CRC Error Mask */
#define USB_DEVICE_FNUM_FNCERR(value)         (USB_DEVICE_FNUM_FNCERR_Msk & (_UINT16_(value) << USB_DEVICE_FNUM_FNCERR_Pos)) /* Assigment of value for FNCERR in the USB_DEVICE_FNUM register */
#define USB_DEVICE_FNUM_Msk                   _UINT16_(0xBFFF)                                     /* (USB_DEVICE_FNUM) Register Mask  */


/* -------- USB_DEVICE_INTENCLR : (USB Offset: 0x14) (R/W 16) DEVICE Device Interrupt Enable Clear -------- */
#define USB_DEVICE_INTENCLR_RESETVALUE        _UINT16_(0x00)                                       /*  (USB_DEVICE_INTENCLR) DEVICE Device Interrupt Enable Clear  Reset Value */

#define USB_DEVICE_INTENCLR_SUSPEND_Pos       _UINT16_(0)                                          /* (USB_DEVICE_INTENCLR) Suspend Interrupt Enable Position */
#define USB_DEVICE_INTENCLR_SUSPEND_Msk       (_UINT16_(0x1) << USB_DEVICE_INTENCLR_SUSPEND_Pos)   /* (USB_DEVICE_INTENCLR) Suspend Interrupt Enable Mask */
#define USB_DEVICE_INTENCLR_SUSPEND(value)    (USB_DEVICE_INTENCLR_SUSPEND_Msk & (_UINT16_(value) << USB_DEVICE_INTENCLR_SUSPEND_Pos)) /* Assigment of value for SUSPEND in the USB_DEVICE_INTENCLR register */
#define USB_DEVICE_INTENCLR_MSOF_Pos          _UINT16_(1)                                          /* (USB_DEVICE_INTENCLR) Micro Start of Frame Interrupt Enable in High Speed Mode Position */
#define USB_DEVICE_INTENCLR_MSOF_Msk          (_UINT16_(0x1) << USB_DEVICE_INTENCLR_MSOF_Pos)      /* (USB_DEVICE_INTENCLR) Micro Start of Frame Interrupt Enable in High Speed Mode Mask */
#define USB_DEVICE_INTENCLR_MSOF(value)       (USB_DEVICE_INTENCLR_MSOF_Msk & (_UINT16_(value) << USB_DEVICE_INTENCLR_MSOF_Pos)) /* Assigment of value for MSOF in the USB_DEVICE_INTENCLR register */
#define USB_DEVICE_INTENCLR_SOF_Pos           _UINT16_(2)                                          /* (USB_DEVICE_INTENCLR) Start Of Frame Interrupt Enable Position */
#define USB_DEVICE_INTENCLR_SOF_Msk           (_UINT16_(0x1) << USB_DEVICE_INTENCLR_SOF_Pos)       /* (USB_DEVICE_INTENCLR) Start Of Frame Interrupt Enable Mask */
#define USB_DEVICE_INTENCLR_SOF(value)        (USB_DEVICE_INTENCLR_SOF_Msk & (_UINT16_(value) << USB_DEVICE_INTENCLR_SOF_Pos)) /* Assigment of value for SOF in the USB_DEVICE_INTENCLR register */
#define USB_DEVICE_INTENCLR_EORST_Pos         _UINT16_(3)                                          /* (USB_DEVICE_INTENCLR) End of Reset Interrupt Enable Position */
#define USB_DEVICE_INTENCLR_EORST_Msk         (_UINT16_(0x1) << USB_DEVICE_INTENCLR_EORST_Pos)     /* (USB_DEVICE_INTENCLR) End of Reset Interrupt Enable Mask */
#define USB_DEVICE_INTENCLR_EORST(value)      (USB_DEVICE_INTENCLR_EORST_Msk & (_UINT16_(value) << USB_DEVICE_INTENCLR_EORST_Pos)) /* Assigment of value for EORST in the USB_DEVICE_INTENCLR register */
#define USB_DEVICE_INTENCLR_WAKEUP_Pos        _UINT16_(4)                                          /* (USB_DEVICE_INTENCLR) Wake Up Interrupt Enable Position */
#define USB_DEVICE_INTENCLR_WAKEUP_Msk        (_UINT16_(0x1) << USB_DEVICE_INTENCLR_WAKEUP_Pos)    /* (USB_DEVICE_INTENCLR) Wake Up Interrupt Enable Mask */
#define USB_DEVICE_INTENCLR_WAKEUP(value)     (USB_DEVICE_INTENCLR_WAKEUP_Msk & (_UINT16_(value) << USB_DEVICE_INTENCLR_WAKEUP_Pos)) /* Assigment of value for WAKEUP in the USB_DEVICE_INTENCLR register */
#define USB_DEVICE_INTENCLR_EORSM_Pos         _UINT16_(5)                                          /* (USB_DEVICE_INTENCLR) End Of Resume Interrupt Enable Position */
#define USB_DEVICE_INTENCLR_EORSM_Msk         (_UINT16_(0x1) << USB_DEVICE_INTENCLR_EORSM_Pos)     /* (USB_DEVICE_INTENCLR) End Of Resume Interrupt Enable Mask */
#define USB_DEVICE_INTENCLR_EORSM(value)      (USB_DEVICE_INTENCLR_EORSM_Msk & (_UINT16_(value) << USB_DEVICE_INTENCLR_EORSM_Pos)) /* Assigment of value for EORSM in the USB_DEVICE_INTENCLR register */
#define USB_DEVICE_INTENCLR_UPRSM_Pos         _UINT16_(6)                                          /* (USB_DEVICE_INTENCLR) Upstream Resume Interrupt Enable Position */
#define USB_DEVICE_INTENCLR_UPRSM_Msk         (_UINT16_(0x1) << USB_DEVICE_INTENCLR_UPRSM_Pos)     /* (USB_DEVICE_INTENCLR) Upstream Resume Interrupt Enable Mask */
#define USB_DEVICE_INTENCLR_UPRSM(value)      (USB_DEVICE_INTENCLR_UPRSM_Msk & (_UINT16_(value) << USB_DEVICE_INTENCLR_UPRSM_Pos)) /* Assigment of value for UPRSM in the USB_DEVICE_INTENCLR register */
#define USB_DEVICE_INTENCLR_RAMACER_Pos       _UINT16_(7)                                          /* (USB_DEVICE_INTENCLR) Ram Access Interrupt Enable Position */
#define USB_DEVICE_INTENCLR_RAMACER_Msk       (_UINT16_(0x1) << USB_DEVICE_INTENCLR_RAMACER_Pos)   /* (USB_DEVICE_INTENCLR) Ram Access Interrupt Enable Mask */
#define USB_DEVICE_INTENCLR_RAMACER(value)    (USB_DEVICE_INTENCLR_RAMACER_Msk & (_UINT16_(value) << USB_DEVICE_INTENCLR_RAMACER_Pos)) /* Assigment of value for RAMACER in the USB_DEVICE_INTENCLR register */
#define USB_DEVICE_INTENCLR_LPMNYET_Pos       _UINT16_(8)                                          /* (USB_DEVICE_INTENCLR) Link Power Management Not Yet Interrupt Enable Position */
#define USB_DEVICE_INTENCLR_LPMNYET_Msk       (_UINT16_(0x1) << USB_DEVICE_INTENCLR_LPMNYET_Pos)   /* (USB_DEVICE_INTENCLR) Link Power Management Not Yet Interrupt Enable Mask */
#define USB_DEVICE_INTENCLR_LPMNYET(value)    (USB_DEVICE_INTENCLR_LPMNYET_Msk & (_UINT16_(value) << USB_DEVICE_INTENCLR_LPMNYET_Pos)) /* Assigment of value for LPMNYET in the USB_DEVICE_INTENCLR register */
#define USB_DEVICE_INTENCLR_LPMSUSP_Pos       _UINT16_(9)                                          /* (USB_DEVICE_INTENCLR) Link Power Management Suspend Interrupt Enable Position */
#define USB_DEVICE_INTENCLR_LPMSUSP_Msk       (_UINT16_(0x1) << USB_DEVICE_INTENCLR_LPMSUSP_Pos)   /* (USB_DEVICE_INTENCLR) Link Power Management Suspend Interrupt Enable Mask */
#define USB_DEVICE_INTENCLR_LPMSUSP(value)    (USB_DEVICE_INTENCLR_LPMSUSP_Msk & (_UINT16_(value) << USB_DEVICE_INTENCLR_LPMSUSP_Pos)) /* Assigment of value for LPMSUSP in the USB_DEVICE_INTENCLR register */
#define USB_DEVICE_INTENCLR_Msk               _UINT16_(0x03FF)                                     /* (USB_DEVICE_INTENCLR) Register Mask  */


/* -------- USB_DEVICE_INTENSET : (USB Offset: 0x18) (R/W 16) DEVICE Device Interrupt Enable Set -------- */
#define USB_DEVICE_INTENSET_RESETVALUE        _UINT16_(0x00)                                       /*  (USB_DEVICE_INTENSET) DEVICE Device Interrupt Enable Set  Reset Value */

#define USB_DEVICE_INTENSET_SUSPEND_Pos       _UINT16_(0)                                          /* (USB_DEVICE_INTENSET) Suspend Interrupt Enable Position */
#define USB_DEVICE_INTENSET_SUSPEND_Msk       (_UINT16_(0x1) << USB_DEVICE_INTENSET_SUSPEND_Pos)   /* (USB_DEVICE_INTENSET) Suspend Interrupt Enable Mask */
#define USB_DEVICE_INTENSET_SUSPEND(value)    (USB_DEVICE_INTENSET_SUSPEND_Msk & (_UINT16_(value) << USB_DEVICE_INTENSET_SUSPEND_Pos)) /* Assigment of value for SUSPEND in the USB_DEVICE_INTENSET register */
#define USB_DEVICE_INTENSET_MSOF_Pos          _UINT16_(1)                                          /* (USB_DEVICE_INTENSET) Micro Start of Frame Interrupt Enable in High Speed Mode Position */
#define USB_DEVICE_INTENSET_MSOF_Msk          (_UINT16_(0x1) << USB_DEVICE_INTENSET_MSOF_Pos)      /* (USB_DEVICE_INTENSET) Micro Start of Frame Interrupt Enable in High Speed Mode Mask */
#define USB_DEVICE_INTENSET_MSOF(value)       (USB_DEVICE_INTENSET_MSOF_Msk & (_UINT16_(value) << USB_DEVICE_INTENSET_MSOF_Pos)) /* Assigment of value for MSOF in the USB_DEVICE_INTENSET register */
#define USB_DEVICE_INTENSET_SOF_Pos           _UINT16_(2)                                          /* (USB_DEVICE_INTENSET) Start Of Frame Interrupt Enable Position */
#define USB_DEVICE_INTENSET_SOF_Msk           (_UINT16_(0x1) << USB_DEVICE_INTENSET_SOF_Pos)       /* (USB_DEVICE_INTENSET) Start Of Frame Interrupt Enable Mask */
#define USB_DEVICE_INTENSET_SOF(value)        (USB_DEVICE_INTENSET_SOF_Msk & (_UINT16_(value) << USB_DEVICE_INTENSET_SOF_Pos)) /* Assigment of value for SOF in the USB_DEVICE_INTENSET register */
#define USB_DEVICE_INTENSET_EORST_Pos         _UINT16_(3)                                          /* (USB_DEVICE_INTENSET) End of Reset Interrupt Enable Position */
#define USB_DEVICE_INTENSET_EORST_Msk         (_UINT16_(0x1) << USB_DEVICE_INTENSET_EORST_Pos)     /* (USB_DEVICE_INTENSET) End of Reset Interrupt Enable Mask */
#define USB_DEVICE_INTENSET_EORST(value)      (USB_DEVICE_INTENSET_EORST_Msk & (_UINT16_(value) << USB_DEVICE_INTENSET_EORST_Pos)) /* Assigment of value for EORST in the USB_DEVICE_INTENSET register */
#define USB_DEVICE_INTENSET_WAKEUP_Pos        _UINT16_(4)                                          /* (USB_DEVICE_INTENSET) Wake Up Interrupt Enable Position */
#define USB_DEVICE_INTENSET_WAKEUP_Msk        (_UINT16_(0x1) << USB_DEVICE_INTENSET_WAKEUP_Pos)    /* (USB_DEVICE_INTENSET) Wake Up Interrupt Enable Mask */
#define USB_DEVICE_INTENSET_WAKEUP(value)     (USB_DEVICE_INTENSET_WAKEUP_Msk & (_UINT16_(value) << USB_DEVICE_INTENSET_WAKEUP_Pos)) /* Assigment of value for WAKEUP in the USB_DEVICE_INTENSET register */
#define USB_DEVICE_INTENSET_EORSM_Pos         _UINT16_(5)                                          /* (USB_DEVICE_INTENSET) End Of Resume Interrupt Enable Position */
#define USB_DEVICE_INTENSET_EORSM_Msk         (_UINT16_(0x1) << USB_DEVICE_INTENSET_EORSM_Pos)     /* (USB_DEVICE_INTENSET) End Of Resume Interrupt Enable Mask */
#define USB_DEVICE_INTENSET_EORSM(value)      (USB_DEVICE_INTENSET_EORSM_Msk & (_UINT16_(value) << USB_DEVICE_INTENSET_EORSM_Pos)) /* Assigment of value for EORSM in the USB_DEVICE_INTENSET register */
#define USB_DEVICE_INTENSET_UPRSM_Pos         _UINT16_(6)                                          /* (USB_DEVICE_INTENSET) Upstream Resume Interrupt Enable Position */
#define USB_DEVICE_INTENSET_UPRSM_Msk         (_UINT16_(0x1) << USB_DEVICE_INTENSET_UPRSM_Pos)     /* (USB_DEVICE_INTENSET) Upstream Resume Interrupt Enable Mask */
#define USB_DEVICE_INTENSET_UPRSM(value)      (USB_DEVICE_INTENSET_UPRSM_Msk & (_UINT16_(value) << USB_DEVICE_INTENSET_UPRSM_Pos)) /* Assigment of value for UPRSM in the USB_DEVICE_INTENSET register */
#define USB_DEVICE_INTENSET_RAMACER_Pos       _UINT16_(7)                                          /* (USB_DEVICE_INTENSET) Ram Access Interrupt Enable Position */
#define USB_DEVICE_INTENSET_RAMACER_Msk       (_UINT16_(0x1) << USB_DEVICE_INTENSET_RAMACER_Pos)   /* (USB_DEVICE_INTENSET) Ram Access Interrupt Enable Mask */
#define USB_DEVICE_INTENSET_RAMACER(value)    (USB_DEVICE_INTENSET_RAMACER_Msk & (_UINT16_(value) << USB_DEVICE_INTENSET_RAMACER_Pos)) /* Assigment of value for RAMACER in the USB_DEVICE_INTENSET register */
#define USB_DEVICE_INTENSET_LPMNYET_Pos       _UINT16_(8)                                          /* (USB_DEVICE_INTENSET) Link Power Management Not Yet Interrupt Enable Position */
#define USB_DEVICE_INTENSET_LPMNYET_Msk       (_UINT16_(0x1) << USB_DEVICE_INTENSET_LPMNYET_Pos)   /* (USB_DEVICE_INTENSET) Link Power Management Not Yet Interrupt Enable Mask */
#define USB_DEVICE_INTENSET_LPMNYET(value)    (USB_DEVICE_INTENSET_LPMNYET_Msk & (_UINT16_(value) << USB_DEVICE_INTENSET_LPMNYET_Pos)) /* Assigment of value for LPMNYET in the USB_DEVICE_INTENSET register */
#define USB_DEVICE_INTENSET_LPMSUSP_Pos       _UINT16_(9)                                          /* (USB_DEVICE_INTENSET) Link Power Management Suspend Interrupt Enable Position */
#define USB_DEVICE_INTENSET_LPMSUSP_Msk       (_UINT16_(0x1) << USB_DEVICE_INTENSET_LPMSUSP_Pos)   /* (USB_DEVICE_INTENSET) Link Power Management Suspend Interrupt Enable Mask */
#define USB_DEVICE_INTENSET_LPMSUSP(value)    (USB_DEVICE_INTENSET_LPMSUSP_Msk & (_UINT16_(value) << USB_DEVICE_INTENSET_LPMSUSP_Pos)) /* Assigment of value for LPMSUSP in the USB_DEVICE_INTENSET register */
#define USB_DEVICE_INTENSET_Msk               _UINT16_(0x03FF)                                     /* (USB_DEVICE_INTENSET) Register Mask  */


/* -------- USB_DEVICE_INTFLAG : (USB Offset: 0x1C) (R/W 16) DEVICE Device Interrupt Flag -------- */
#define USB_DEVICE_INTFLAG_RESETVALUE         _UINT16_(0x00)                                       /*  (USB_DEVICE_INTFLAG) DEVICE Device Interrupt Flag  Reset Value */

#define USB_DEVICE_INTFLAG_SUSPEND_Pos        _UINT16_(0)                                          /* (USB_DEVICE_INTFLAG) Suspend Position */
#define USB_DEVICE_INTFLAG_SUSPEND_Msk        (_UINT16_(0x1) << USB_DEVICE_INTFLAG_SUSPEND_Pos)    /* (USB_DEVICE_INTFLAG) Suspend Mask */
#define USB_DEVICE_INTFLAG_SUSPEND(value)     (USB_DEVICE_INTFLAG_SUSPEND_Msk & (_UINT16_(value) << USB_DEVICE_INTFLAG_SUSPEND_Pos)) /* Assigment of value for SUSPEND in the USB_DEVICE_INTFLAG register */
#define USB_DEVICE_INTFLAG_MSOF_Pos           _UINT16_(1)                                          /* (USB_DEVICE_INTFLAG) Micro Start of Frame in High Speed Mode Position */
#define USB_DEVICE_INTFLAG_MSOF_Msk           (_UINT16_(0x1) << USB_DEVICE_INTFLAG_MSOF_Pos)       /* (USB_DEVICE_INTFLAG) Micro Start of Frame in High Speed Mode Mask */
#define USB_DEVICE_INTFLAG_MSOF(value)        (USB_DEVICE_INTFLAG_MSOF_Msk & (_UINT16_(value) << USB_DEVICE_INTFLAG_MSOF_Pos)) /* Assigment of value for MSOF in the USB_DEVICE_INTFLAG register */
#define USB_DEVICE_INTFLAG_SOF_Pos            _UINT16_(2)                                          /* (USB_DEVICE_INTFLAG) Start Of Frame Position */
#define USB_DEVICE_INTFLAG_SOF_Msk            (_UINT16_(0x1) << USB_DEVICE_INTFLAG_SOF_Pos)        /* (USB_DEVICE_INTFLAG) Start Of Frame Mask */
#define USB_DEVICE_INTFLAG_SOF(value)         (USB_DEVICE_INTFLAG_SOF_Msk & (_UINT16_(value) << USB_DEVICE_INTFLAG_SOF_Pos)) /* Assigment of value for SOF in the USB_DEVICE_INTFLAG register */
#define USB_DEVICE_INTFLAG_EORST_Pos          _UINT16_(3)                                          /* (USB_DEVICE_INTFLAG) End of Reset Position */
#define USB_DEVICE_INTFLAG_EORST_Msk          (_UINT16_(0x1) << USB_DEVICE_INTFLAG_EORST_Pos)      /* (USB_DEVICE_INTFLAG) End of Reset Mask */
#define USB_DEVICE_INTFLAG_EORST(value)       (USB_DEVICE_INTFLAG_EORST_Msk & (_UINT16_(value) << USB_DEVICE_INTFLAG_EORST_Pos)) /* Assigment of value for EORST in the USB_DEVICE_INTFLAG register */
#define USB_DEVICE_INTFLAG_WAKEUP_Pos         _UINT16_(4)                                          /* (USB_DEVICE_INTFLAG) Wake Up Position */
#define USB_DEVICE_INTFLAG_WAKEUP_Msk         (_UINT16_(0x1) << USB_DEVICE_INTFLAG_WAKEUP_Pos)     /* (USB_DEVICE_INTFLAG) Wake Up Mask */
#define USB_DEVICE_INTFLAG_WAKEUP(value)      (USB_DEVICE_INTFLAG_WAKEUP_Msk & (_UINT16_(value) << USB_DEVICE_INTFLAG_WAKEUP_Pos)) /* Assigment of value for WAKEUP in the USB_DEVICE_INTFLAG register */
#define USB_DEVICE_INTFLAG_EORSM_Pos          _UINT16_(5)                                          /* (USB_DEVICE_INTFLAG) End Of Resume Position */
#define USB_DEVICE_INTFLAG_EORSM_Msk          (_UINT16_(0x1) << USB_DEVICE_INTFLAG_EORSM_Pos)      /* (USB_DEVICE_INTFLAG) End Of Resume Mask */
#define USB_DEVICE_INTFLAG_EORSM(value)       (USB_DEVICE_INTFLAG_EORSM_Msk & (_UINT16_(value) << USB_DEVICE_INTFLAG_EORSM_Pos)) /* Assigment of value for EORSM in the USB_DEVICE_INTFLAG register */
#define USB_DEVICE_INTFLAG_UPRSM_Pos          _UINT16_(6)                                          /* (USB_DEVICE_INTFLAG) Upstream Resume Position */
#define USB_DEVICE_INTFLAG_UPRSM_Msk          (_UINT16_(0x1) << USB_DEVICE_INTFLAG_UPRSM_Pos)      /* (USB_DEVICE_INTFLAG) Upstream Resume Mask */
#define USB_DEVICE_INTFLAG_UPRSM(value)       (USB_DEVICE_INTFLAG_UPRSM_Msk & (_UINT16_(value) << USB_DEVICE_INTFLAG_UPRSM_Pos)) /* Assigment of value for UPRSM in the USB_DEVICE_INTFLAG register */
#define USB_DEVICE_INTFLAG_RAMACER_Pos        _UINT16_(7)                                          /* (USB_DEVICE_INTFLAG) Ram Access Position */
#define USB_DEVICE_INTFLAG_RAMACER_Msk        (_UINT16_(0x1) << USB_DEVICE_INTFLAG_RAMACER_Pos)    /* (USB_DEVICE_INTFLAG) Ram Access Mask */
#define USB_DEVICE_INTFLAG_RAMACER(value)     (USB_DEVICE_INTFLAG_RAMACER_Msk & (_UINT16_(value) << USB_DEVICE_INTFLAG_RAMACER_Pos)) /* Assigment of value for RAMACER in the USB_DEVICE_INTFLAG register */
#define USB_DEVICE_INTFLAG_LPMNYET_Pos        _UINT16_(8)                                          /* (USB_DEVICE_INTFLAG) Link Power Management Not Yet Position */
#define USB_DEVICE_INTFLAG_LPMNYET_Msk        (_UINT16_(0x1) << USB_DEVICE_INTFLAG_LPMNYET_Pos)    /* (USB_DEVICE_INTFLAG) Link Power Management Not Yet Mask */
#define USB_DEVICE_INTFLAG_LPMNYET(value)     (USB_DEVICE_INTFLAG_LPMNYET_Msk & (_UINT16_(value) << USB_DEVICE_INTFLAG_LPMNYET_Pos)) /* Assigment of value for LPMNYET in the USB_DEVICE_INTFLAG register */
#define USB_DEVICE_INTFLAG_LPMSUSP_Pos        _UINT16_(9)                                          /* (USB_DEVICE_INTFLAG) Link Power Management Suspend Position */
#define USB_DEVICE_INTFLAG_LPMSUSP_Msk        (_UINT16_(0x1) << USB_DEVICE_INTFLAG_LPMSUSP_Pos)    /* (USB_DEVICE_INTFLAG) Link Power Management Suspend Mask */
#define USB_DEVICE_INTFLAG_LPMSUSP(value)     (USB_DEVICE_INTFLAG_LPMSUSP_Msk & (_UINT16_(value) << USB_DEVICE_INTFLAG_LPMSUSP_Pos)) /* Assigment of value for LPMSUSP in the USB_DEVICE_INTFLAG register */
#define USB_DEVICE_INTFLAG_Msk                _UINT16_(0x03FF)                                     /* (USB_DEVICE_INTFLAG) Register Mask  */


/* -------- USB_DEVICE_EPINTSMRY : (USB Offset: 0x20) ( R/ 16) DEVICE End Point Interrupt Summary -------- */
#define USB_DEVICE_EPINTSMRY_RESETVALUE       _UINT16_(0x00)                                       /*  (USB_DEVICE_EPINTSMRY) DEVICE End Point Interrupt Summary  Reset Value */

#define USB_DEVICE_EPINTSMRY_EPINT0_Pos       _UINT16_(0)                                          /* (USB_DEVICE_EPINTSMRY) End Point 0 Interrupt Position */
#define USB_DEVICE_EPINTSMRY_EPINT0_Msk       (_UINT16_(0x1) << USB_DEVICE_EPINTSMRY_EPINT0_Pos)   /* (USB_DEVICE_EPINTSMRY) End Point 0 Interrupt Mask */
#define USB_DEVICE_EPINTSMRY_EPINT0(value)    (USB_DEVICE_EPINTSMRY_EPINT0_Msk & (_UINT16_(value) << USB_DEVICE_EPINTSMRY_EPINT0_Pos)) /* Assigment of value for EPINT0 in the USB_DEVICE_EPINTSMRY register */
#define USB_DEVICE_EPINTSMRY_EPINT1_Pos       _UINT16_(1)                                          /* (USB_DEVICE_EPINTSMRY) End Point 1 Interrupt Position */
#define USB_DEVICE_EPINTSMRY_EPINT1_Msk       (_UINT16_(0x1) << USB_DEVICE_EPINTSMRY_EPINT1_Pos)   /* (USB_DEVICE_EPINTSMRY) End Point 1 Interrupt Mask */
#define USB_DEVICE_EPINTSMRY_EPINT1(value)    (USB_DEVICE_EPINTSMRY_EPINT1_Msk & (_UINT16_(value) << USB_DEVICE_EPINTSMRY_EPINT1_Pos)) /* Assigment of value for EPINT1 in the USB_DEVICE_EPINTSMRY register */
#define USB_DEVICE_EPINTSMRY_EPINT2_Pos       _UINT16_(2)                                          /* (USB_DEVICE_EPINTSMRY) End Point 2 Interrupt Position */
#define USB_DEVICE_EPINTSMRY_EPINT2_Msk       (_UINT16_(0x1) << USB_DEVICE_EPINTSMRY_EPINT2_Pos)   /* (USB_DEVICE_EPINTSMRY) End Point 2 Interrupt Mask */
#define USB_DEVICE_EPINTSMRY_EPINT2(value)    (USB_DEVICE_EPINTSMRY_EPINT2_Msk & (_UINT16_(value) << USB_DEVICE_EPINTSMRY_EPINT2_Pos)) /* Assigment of value for EPINT2 in the USB_DEVICE_EPINTSMRY register */
#define USB_DEVICE_EPINTSMRY_EPINT3_Pos       _UINT16_(3)                                          /* (USB_DEVICE_EPINTSMRY) End Point 3 Interrupt Position */
#define USB_DEVICE_EPINTSMRY_EPINT3_Msk       (_UINT16_(0x1) << USB_DEVICE_EPINTSMRY_EPINT3_Pos)   /* (USB_DEVICE_EPINTSMRY) End Point 3 Interrupt Mask */
#define USB_DEVICE_EPINTSMRY_EPINT3(value)    (USB_DEVICE_EPINTSMRY_EPINT3_Msk & (_UINT16_(value) << USB_DEVICE_EPINTSMRY_EPINT3_Pos)) /* Assigment of value for EPINT3 in the USB_DEVICE_EPINTSMRY register */
#define USB_DEVICE_EPINTSMRY_EPINT4_Pos       _UINT16_(4)                                          /* (USB_DEVICE_EPINTSMRY) End Point 4 Interrupt Position */
#define USB_DEVICE_EPINTSMRY_EPINT4_Msk       (_UINT16_(0x1) << USB_DEVICE_EPINTSMRY_EPINT4_Pos)   /* (USB_DEVICE_EPINTSMRY) End Point 4 Interrupt Mask */
#define USB_DEVICE_EPINTSMRY_EPINT4(value)    (USB_DEVICE_EPINTSMRY_EPINT4_Msk & (_UINT16_(value) << USB_DEVICE_EPINTSMRY_EPINT4_Pos)) /* Assigment of value for EPINT4 in the USB_DEVICE_EPINTSMRY register */
#define USB_DEVICE_EPINTSMRY_EPINT5_Pos       _UINT16_(5)                                          /* (USB_DEVICE_EPINTSMRY) End Point 5 Interrupt Position */
#define USB_DEVICE_EPINTSMRY_EPINT5_Msk       (_UINT16_(0x1) << USB_DEVICE_EPINTSMRY_EPINT5_Pos)   /* (USB_DEVICE_EPINTSMRY) End Point 5 Interrupt Mask */
#define USB_DEVICE_EPINTSMRY_EPINT5(value)    (USB_DEVICE_EPINTSMRY_EPINT5_Msk & (_UINT16_(value) << USB_DEVICE_EPINTSMRY_EPINT5_Pos)) /* Assigment of value for EPINT5 in the USB_DEVICE_EPINTSMRY register */
#define USB_DEVICE_EPINTSMRY_EPINT6_Pos       _UINT16_(6)                                          /* (USB_DEVICE_EPINTSMRY) End Point 6 Interrupt Position */
#define USB_DEVICE_EPINTSMRY_EPINT6_Msk       (_UINT16_(0x1) << USB_DEVICE_EPINTSMRY_EPINT6_Pos)   /* (USB_DEVICE_EPINTSMRY) End Point 6 Interrupt Mask */
#define USB_DEVICE_EPINTSMRY_EPINT6(value)    (USB_DEVICE_EPINTSMRY_EPINT6_Msk & (_UINT16_(value) << USB_DEVICE_EPINTSMRY_EPINT6_Pos)) /* Assigment of value for EPINT6 in the USB_DEVICE_EPINTSMRY register */
#define USB_DEVICE_EPINTSMRY_EPINT7_Pos       _UINT16_(7)                                          /* (USB_DEVICE_EPINTSMRY) End Point 7 Interrupt Position */
#define USB_DEVICE_EPINTSMRY_EPINT7_Msk       (_UINT16_(0x1) << USB_DEVICE_EPINTSMRY_EPINT7_Pos)   /* (USB_DEVICE_EPINTSMRY) End Point 7 Interrupt Mask */
#define USB_DEVICE_EPINTSMRY_EPINT7(value)    (USB_DEVICE_EPINTSMRY_EPINT7_Msk & (_UINT16_(value) << USB_DEVICE_EPINTSMRY_EPINT7_Pos)) /* Assigment of value for EPINT7 in the USB_DEVICE_EPINTSMRY register */
#define USB_DEVICE_EPINTSMRY_Msk              _UINT16_(0x00FF)                                     /* (USB_DEVICE_EPINTSMRY) Register Mask  */

#define USB_DEVICE_EPINTSMRY_EPINT_Pos        _UINT16_(0)                                          /* (USB_DEVICE_EPINTSMRY Position) End Point 7 Interrupt */
#define USB_DEVICE_EPINTSMRY_EPINT_Msk        (_UINT16_(0xFF) << USB_DEVICE_EPINTSMRY_EPINT_Pos)   /* (USB_DEVICE_EPINTSMRY Mask) EPINT */
#define USB_DEVICE_EPINTSMRY_EPINT(value)     (USB_DEVICE_EPINTSMRY_EPINT_Msk & (_UINT16_(value) << USB_DEVICE_EPINTSMRY_EPINT_Pos)) 

/* -------- USB_DESCADD : (USB Offset: 0x24) (R/W 32) Descriptor Address -------- */
#define USB_DESCADD_RESETVALUE                _UINT32_(0x00)                                       /*  (USB_DESCADD) Descriptor Address  Reset Value */

#define USB_DESCADD_DESCADD_Pos               _UINT32_(0)                                          /* (USB_DESCADD) Descriptor Address Value Position */
#define USB_DESCADD_DESCADD_Msk               (_UINT32_(0xFFFFFFFF) << USB_DESCADD_DESCADD_Pos)    /* (USB_DESCADD) Descriptor Address Value Mask */
#define USB_DESCADD_DESCADD(value)            (USB_DESCADD_DESCADD_Msk & (_UINT32_(value) << USB_DESCADD_DESCADD_Pos)) /* Assigment of value for DESCADD in the USB_DESCADD register */
#define USB_DESCADD_Msk                       _UINT32_(0xFFFFFFFF)                                 /* (USB_DESCADD) Register Mask  */


/* -------- USB_PADCAL : (USB Offset: 0x28) (R/W 16) USB PAD Calibration -------- */
#define USB_PADCAL_RESETVALUE                 _UINT16_(0x00)                                       /*  (USB_PADCAL) USB PAD Calibration  Reset Value */

#define USB_PADCAL_TRANSP_Pos                 _UINT16_(0)                                          /* (USB_PADCAL) USB Pad Transp calibration Position */
#define USB_PADCAL_TRANSP_Msk                 (_UINT16_(0x1F) << USB_PADCAL_TRANSP_Pos)            /* (USB_PADCAL) USB Pad Transp calibration Mask */
#define USB_PADCAL_TRANSP(value)              (USB_PADCAL_TRANSP_Msk & (_UINT16_(value) << USB_PADCAL_TRANSP_Pos)) /* Assigment of value for TRANSP in the USB_PADCAL register */
#define USB_PADCAL_TRANSN_Pos                 _UINT16_(6)                                          /* (USB_PADCAL) USB Pad Transn calibration Position */
#define USB_PADCAL_TRANSN_Msk                 (_UINT16_(0x1F) << USB_PADCAL_TRANSN_Pos)            /* (USB_PADCAL) USB Pad Transn calibration Mask */
#define USB_PADCAL_TRANSN(value)              (USB_PADCAL_TRANSN_Msk & (_UINT16_(value) << USB_PADCAL_TRANSN_Pos)) /* Assigment of value for TRANSN in the USB_PADCAL register */
#define USB_PADCAL_TRIM_Pos                   _UINT16_(12)                                         /* (USB_PADCAL) USB Pad Trim calibration Position */
#define USB_PADCAL_TRIM_Msk                   (_UINT16_(0x7) << USB_PADCAL_TRIM_Pos)               /* (USB_PADCAL) USB Pad Trim calibration Mask */
#define USB_PADCAL_TRIM(value)                (USB_PADCAL_TRIM_Msk & (_UINT16_(value) << USB_PADCAL_TRIM_Pos)) /* Assigment of value for TRIM in the USB_PADCAL register */
#define USB_PADCAL_Msk                        _UINT16_(0x77DF)                                     /* (USB_PADCAL) Register Mask  */


/** \brief USB register offsets definitions */
#define USB_DEVICE_ADDR_REG_OFST       _UINT32_(0x00)      /* (USB_DEVICE_ADDR) DEVICE_DESC_BANK Endpoint Bank, Adress of Data Buffer Offset */
#define USB_DEVICE_PCKSIZE_REG_OFST    _UINT32_(0x04)      /* (USB_DEVICE_PCKSIZE) DEVICE_DESC_BANK Endpoint Bank, Packet Size Offset */
#define USB_DEVICE_EXTREG_REG_OFST     _UINT32_(0x08)      /* (USB_DEVICE_EXTREG) DEVICE_DESC_BANK Endpoint Bank, Extended Offset */
#define USB_DEVICE_STATUS_BK_REG_OFST  _UINT32_(0x0A)      /* (USB_DEVICE_STATUS_BK) DEVICE_DESC_BANK Enpoint Bank, Status of Bank Offset */
#define USB_DEVICE_EPCFG_REG_OFST      _UINT32_(0x00)      /* (USB_DEVICE_EPCFG) DEVICE_ENDPOINT End Point Configuration Offset */
#define USB_DEVICE_EPSTATUSCLR_REG_OFST _UINT32_(0x04)      /* (USB_DEVICE_EPSTATUSCLR) DEVICE_ENDPOINT End Point Pipe Status Clear Offset */
#define USB_DEVICE_EPSTATUSSET_REG_OFST _UINT32_(0x05)      /* (USB_DEVICE_EPSTATUSSET) DEVICE_ENDPOINT End Point Pipe Status Set Offset */
#define USB_DEVICE_EPSTATUS_REG_OFST   _UINT32_(0x06)      /* (USB_DEVICE_EPSTATUS) DEVICE_ENDPOINT End Point Pipe Status Offset */
#define USB_DEVICE_EPINTFLAG_REG_OFST  _UINT32_(0x07)      /* (USB_DEVICE_EPINTFLAG) DEVICE_ENDPOINT End Point Interrupt Flag Offset */
#define USB_DEVICE_EPINTENCLR_REG_OFST _UINT32_(0x08)      /* (USB_DEVICE_EPINTENCLR) DEVICE_ENDPOINT End Point Interrupt Clear Flag Offset */
#define USB_DEVICE_EPINTENSET_REG_OFST _UINT32_(0x09)      /* (USB_DEVICE_EPINTENSET) DEVICE_ENDPOINT End Point Interrupt Set Flag Offset */
#define USB_CTRLA_REG_OFST             _UINT32_(0x00)      /* (USB_CTRLA) Control A Offset */
#define USB_SYNCBUSY_REG_OFST          _UINT32_(0x02)      /* (USB_SYNCBUSY) Synchronization Busy Offset */
#define USB_QOSCTRL_REG_OFST           _UINT32_(0x03)      /* (USB_QOSCTRL) USB Quality Of Service Offset */
#define USB_DEVICE_CTRLB_REG_OFST      _UINT32_(0x08)      /* (USB_DEVICE_CTRLB) DEVICE Control B Offset */
#define USB_DEVICE_DADD_REG_OFST       _UINT32_(0x0A)      /* (USB_DEVICE_DADD) DEVICE Device Address Offset */
#define USB_DEVICE_STATUS_REG_OFST     _UINT32_(0x0C)      /* (USB_DEVICE_STATUS) DEVICE Status Offset */
#define USB_FSMSTATUS_REG_OFST         _UINT32_(0x0D)      /* (USB_FSMSTATUS) Finite State Machine Status Offset */
#define USB_DEVICE_FNUM_REG_OFST       _UINT32_(0x10)      /* (USB_DEVICE_FNUM) DEVICE Device Frame Number Offset */
#define USB_DEVICE_INTENCLR_REG_OFST   _UINT32_(0x14)      /* (USB_DEVICE_INTENCLR) DEVICE Device Interrupt Enable Clear Offset */
#define USB_DEVICE_INTENSET_REG_OFST   _UINT32_(0x18)      /* (USB_DEVICE_INTENSET) DEVICE Device Interrupt Enable Set Offset */
#define USB_DEVICE_INTFLAG_REG_OFST    _UINT32_(0x1C)      /* (USB_DEVICE_INTFLAG) DEVICE Device Interrupt Flag Offset */
#define USB_DEVICE_EPINTSMRY_REG_OFST  _UINT32_(0x20)      /* (USB_DEVICE_EPINTSMRY) DEVICE End Point Interrupt Summary Offset */
#define USB_DESCADD_REG_OFST           _UINT32_(0x24)      /* (USB_DESCADD) Descriptor Address Offset */
#define USB_PADCAL_REG_OFST            _UINT32_(0x28)      /* (USB_PADCAL) USB PAD Calibration Offset */

#if !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__))
/** \brief DEVICE_DESC_BANK register API structure */
typedef struct
{
  __IO  uint32_t                       USB_ADDR;           /**< Offset: 0x00 (R/W  32) DEVICE_DESC_BANK Endpoint Bank, Adress of Data Buffer */
  __IO  uint32_t                       USB_PCKSIZE;        /**< Offset: 0x04 (R/W  32) DEVICE_DESC_BANK Endpoint Bank, Packet Size */
  __IO  uint16_t                       USB_EXTREG;         /**< Offset: 0x08 (R/W  16) DEVICE_DESC_BANK Endpoint Bank, Extended */
  __IO  uint8_t                        USB_STATUS_BK;      /**< Offset: 0x0A (R/W  8) DEVICE_DESC_BANK Enpoint Bank, Status of Bank */
  __I   uint8_t                        Reserved1[0x05];
} usb_device_desc_bank_registers_t;

/** \brief DEVICE_ENDPOINT register API structure */
typedef struct
{
  __IO  uint8_t                        USB_EPCFG;          /**< Offset: 0x00 (R/W  8) DEVICE_ENDPOINT End Point Configuration */
  __I   uint8_t                        Reserved1[0x03];
  __O   uint8_t                        USB_EPSTATUSCLR;    /**< Offset: 0x04 ( /W  8) DEVICE_ENDPOINT End Point Pipe Status Clear */
  __O   uint8_t                        USB_EPSTATUSSET;    /**< Offset: 0x05 ( /W  8) DEVICE_ENDPOINT End Point Pipe Status Set */
  __I   uint8_t                        USB_EPSTATUS;       /**< Offset: 0x06 (R/   8) DEVICE_ENDPOINT End Point Pipe Status */
  __IO  uint8_t                        USB_EPINTFLAG;      /**< Offset: 0x07 (R/W  8) DEVICE_ENDPOINT End Point Interrupt Flag */
  __IO  uint8_t                        USB_EPINTENCLR;     /**< Offset: 0x08 (R/W  8) DEVICE_ENDPOINT End Point Interrupt Clear Flag */
  __IO  uint8_t                        USB_EPINTENSET;     /**< Offset: 0x09 (R/W  8) DEVICE_ENDPOINT End Point Interrupt Set Flag */
  __I   uint8_t                        Reserved2[0x16];
} usb_device_endpoint_registers_t;

#define USB_DEVICE_DESC_BANK_NUMBER 2

/** \brief USB_DESCRIPTOR register API structure */
typedef struct
{  /* Universal Serial Bus */
        usb_device_desc_bank_registers_t DEVICE_DESC_BANK[USB_DEVICE_DESC_BANK_NUMBER]; /**< Offset: 0x00  */
} usb_descriptor_registers_t;

#define USB_DEVICE_ENDPOINT_NUMBER 8

/** \brief USB register API structure */
typedef struct
{  /* Universal Serial Bus */
  __IO  uint8_t                        USB_CTRLA;          /**< Offset: 0x00 (R/W  8) Control A */
  __I   uint8_t                        Reserved1[0x01];
  __I   uint8_t                        USB_SYNCBUSY;       /**< Offset: 0x02 (R/   8) Synchronization Busy */
  __IO  uint8_t                        USB_QOSCTRL;        /**< Offset: 0x03 (R/W  8) USB Quality Of Service */
  __I   uint8_t                        Reserved2[0x04];
  __IO  uint16_t                       USB_CTRLB;          /**< Offset: 0x08 (R/W  16) DEVICE Control B */
  __IO  uint8_t                        USB_DADD;           /**< Offset: 0x0A (R/W  8) DEVICE Device Address */
  __I   uint8_t                        Reserved3[0x01];
  __I   uint8_t                        USB_STATUS;         /**< Offset: 0x0C (R/   8) DEVICE Status */
  __I   uint8_t                        USB_FSMSTATUS;      /**< Offset: 0x0D (R/   8) Finite State Machine Status */
  __I   uint8_t                        Reserved4[0x02];
  __I   uint16_t                       USB_FNUM;           /**< Offset: 0x10 (R/   16) DEVICE Device Frame Number */
  __I   uint8_t                        Reserved5[0x02];
  __IO  uint16_t                       USB_INTENCLR;       /**< Offset: 0x14 (R/W  16) DEVICE Device Interrupt Enable Clear */
  __I   uint8_t                        Reserved6[0x02];
  __IO  uint16_t                       USB_INTENSET;       /**< Offset: 0x18 (R/W  16) DEVICE Device Interrupt Enable Set */
  __I   uint8_t                        Reserved7[0x02];
  __IO  uint16_t                       USB_INTFLAG;        /**< Offset: 0x1C (R/W  16) DEVICE Device Interrupt Flag */
  __I   uint8_t                        Reserved8[0x02];
  __I   uint16_t                       USB_EPINTSMRY;      /**< Offset: 0x20 (R/   16) DEVICE End Point Interrupt Summary */
  __I   uint8_t                        Reserved9[0x02];
  __IO  uint32_t                       USB_DESCADD;        /**< Offset: 0x24 (R/W  32) Descriptor Address */
  __IO  uint16_t                       USB_PADCAL;         /**< Offset: 0x28 (R/W  16) USB PAD Calibration */
  __I   uint8_t                        Reserved10[0xD6];
        usb_device_endpoint_registers_t DEVICE_ENDPOINT[USB_DEVICE_ENDPOINT_NUMBER]; /**< Offset: 0x100  */
} usb_registers_t;


#endif /* !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__)) */
/** \brief USB_DESCRIPTOR memory section attribute */
#define SECTION_USB_DESCRIPTOR

#endif /* _SAMD11_USB_COMPONENT_H_ */
