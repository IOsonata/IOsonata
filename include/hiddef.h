/**-------------------------------------------------------------------------
@file	hiddef.h

@brief	Generic HID Class definitions

@author	Hoang Nguyen Hoan
@date	Nov. 11, 2014

@license

MIT License

Copyright (c) 2014, I-SYST inc., all rights reserved

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
#ifndef __HIDDEF_H__
#define __HIDDEF_H__

#include <stdint.h>
#include <string.h>

#ifndef __cplusplus
#include <stdbool.h>
#endif

/** @addtogroup HID
  * @{
  */

typedef enum __HID_Subclass {
	HID_SUBCLASS_NONE = 0,
	HID_SUBCLASS_BOOT = 1,		//!< Boot Interface Subclass
} HID_SUBCLASS;

typedef enum __HID_Protocol {
	HID_PROT_NONE		= 0,
	HID_PROT_KEYBOARD	= 1,
	HID_PROT_MOUSE		= 2
} HID_PROT;

typedef enum __HID_Request_Codes {
	HID_REQ_GET_REPORT 		= 1,
	HID_REQ_GET_IDLE 		= 2,
	HID_REQ_GET_PROTOCOL 	= 3,
	HID_REQ_SET_REPORT 		= 9,
	HID_REQ_SET_IDLE 		= 10,
	HID_REQ_SET_PROTOCOL 	= 11
} HID_REQ;

// The wValue field in the SETUP_DATA specifies the Report Type in the high
// byte and the Report ID in the low byte. Set Report ID to 0 (zero) if Report IDs
// are not used.
#define HID_REPID_MASK		0xFF
#define HID_REPTYPE_MASK	0xFF00

typedef enum __HID_Report_type {
	HID_REPTYPE_INPUT 	= (1 << 8),
	HID_REPTYPE_OUTPUT 	= (2 << 8),
	HID_REPTYPE_FEATURE = (3 << 8)
} HID_REPTYPE;

typedef enum __HID_Usage_Page {
	HID_USAGEPAGE_UNDEF				= 0,
	HID_USAGEPAGE_DESKTOP			= 1,		//!< Generic Desktop Controls
	HID_USAGEPAGE_SIM_CTRL			= 2,		//!< Simulation Controls
	HID_USAGEPAGE_VR_CTRL			= 3,		//!< VR Controls
	HID_USAGEPAGE_SPORT_CTRL		= 4,		//!< Sport Controls
	HID_USAGEPAGE_GAME_CTRL			= 5,		//!< Game Controls
	HID_USAGEPAGE_GEN_DEV_CTRL		= 6,		//!< Generic Device Controls
	HID_USAGEPAGE_KEYBOARD			= 7,		//!< Keyboard/Keypad
	HID_USAGEPAGE_LED				= 8,		//!< LEDs
	HID_USAGEPAGE_BUTTON			= 9,		//!< Button
	HID_USAGEPAGE_ORDINAL			= 0xA,		//!< Ordinal
	HID_USAGEPAGE_TEL				= 0xB,		//!< Telephony
	HID_USAGEPAGE_CONSUMER			= 0xC,		//!< Consumer
	HID_USAGEPAGE_DIGITIZER			= 0xD,		//!< Digitizer
	HID_USAGEPAGE_HAPTIC			= 0xE,		//!< Haptic
	HID_USAGEPAGE_PID				= 0xF,		//!< PID Page Physical Interface Device
												//!< definitions for force feedback and related devices.
	HID_USAGEPAGE_UNICODE			= 0x10,		//!< Unicode
	HID_USAGEPAGE_SOC				= 0x11,		//!< SoC
	HID_USAGEPAGE_EYE_HEAD_TRACK	= 0x12,		//!< Eye and Head tracker
	HID_USAGEPAGE_AUXDISPLAY		= 0x14,		//!<  Auxiliary Display
	HID_USAGEPAGE_SENSOR			= 0x20,		//!< Sensors
	HID_USAGEPAGE_MEDICAL			= 0x40,		//!< Medical Instruments
	HID_USAGEPAGE_BRAILLE_DISPLAY	= 0x41,		//!< Braille display
	HID_USAGEPAGE_LIGHT_ILLUM		= 0x59,		//!< Light and illuminztion
	HID_USAGEPAGE_MONITOR			= 0x80,		//!< 0x80-0x83 Monitor pages Device
												//!< Class Definition for Monitor Devices
	HID_USAGEPAGE_MONITOR_ENUM		= 0x81,		//!< Monitor enumerated
	HID_USAGEPAGE_VESA_VIRT_CTRL	= 0x82,		//!< VESA virtual control
	HID_USAGEPAGE_POWER				= 0x84,		//!< 0x84-0x87 Power pages Device Class
												//!< Definition for Power Devices
	HID_USAGEPAGE_BATT_SYSTEM		= 0x85,		//!< Battery system
	HID_USAGEPAGE_BARCODE			= 0x8C,		//!< Bar Code Scanner page
	HID_USAGEPAGE_SCALE				= 0x8D,		//!< Scale page
	HID_USAGEPAGE_MSR				= 0x8E,		//!< Magnetic Stripe Reading (MSR) Devices
	HID_USAGEPAGE_POS				= 0x8F,		//!< Reserved Point of Sale pages
	HID_USAGEPAGE_CAM				= 0x90,		//!< Camera Control Page Device Class Definition
	HID_USAGEPAGE_ARCADE			= 0x91,		//!< Arcade Page
												//!< OAAF Definitions for arcade and coinop related Devices
	HID_USAGEPAGE_FIDO_ALLIANCE		= 0xF1D0,	//!< FIDO alliance
	HID_USAGEPAGE_VENDOR			= 0xFF00	//!< 0xFF00-0xFFFF Vendor-defined
} HID_USAGEPAGE;

// Generic Desktop Page
typedef enum __HID_Page_Desktop {
	HID_PAGE_DESKTOP_UNDEF					= 0,
	HID_PAGE_DESKTOP_POINTER				= 1,
	HID_PAGE_DESKTOP_MOUSER					= 2,
	HID_PAGE_DESKTOP_JOYSTICK				= 4,
	HID_PAGE_DESKTOP_GAMEPAD				= 5,
	HID_PAGE_DESKTOP_KEYBOARD				= 6,
	HID_PAGE_DESKTOP_KEYPAD					= 7,
	HID_PAGE_DESKTOP_MULTI_AXIS_CTRL		= 8,
	HID_PAGE_DESKTOP_TABLET_PCSYST_CTRL		= 9,
	HID_PAGE_DESKTOP_WATER_COOL_DEV			= 0xA,
	HID_PAGE_DESKTOP_COMPUTER_CHASSIS_DEV	= 0xB,
	HID_PAGE_DESKTOP_WIRELESS_RADIO_CTRL	= 0xC,
	HID_PAGE_DESKTOP_PORTABLE_DEV_CTRL		= 0xD,
	HID_PAGE_DESKTOP_SYST_MULTIAXIS_CTRL	= 0xE,
	HID_PAGE_DESKTOP_SPATIAL_CTRL			= 0xF,
	HID_PAGE_DESKTOP_ASSISTIVE_CTRL			= 0x10,
	HID_PAGE_DESKTOP_DEVICE_DOCK			= 0x11,
	HID_PAGE_DESKTOP_DOCKABLE_DEV			= 0x12,
	HID_PAGE_DESKTOP_CALLSTATE_MNGR_CTRL	= 0x13,
} HID_PAGE_DESKTOP;

typedef enum __HID_Page_VR_Control {
	HID_PAGE_VR_CTRL_UNDEF					= 0,
	HID_PAGE_VR_CTRL_BELT					= 1,
	HID_PAGE_VR_CTRL_BODY_SUIT				= 2,
	HID_PAGE_VR_CTRL_FLEXOR					= 3,
	HID_PAGE_VR_CTRL_GLOVE					= 4,
	HID_PAGE_VR_CTRL_HEAD_TRACKER			= 5,
	HID_PAGE_VR_CTRL_HEAD_MOUNT_DISPLAY		= 6,
	HID_PAGE_VR_CTRL_HAND_TRACKER			= 7,
	HID_PAGE_VR_CTRL_OCULOMETER				= 8,
	HID_PAGE_VR_CTRL_VEST					= 9,
	HID_PAGE_VR_CTRL_ANIMATRONIC_DEV		= 0xA,
	HID_PAGE_VR_CTRL_STEREO_ENABLE			= 0x20,
	HID_PAGE_VR_CTRL_DISPLAY_ENABLE			= 0x21
} HID_PAGE_VR_CTRL;

typedef enum __HID_Page_Game_Control {
	HID_PAGE_GAME_CTRL_UNDEF					= 0,
	HID_PAGE_GAME_CTRL_3DGAME_CTRLR				= 1,
	HID_PAGE_GAME_CTRL_PINBALL_DEV				= 2,
	HID_PAGE_GAME_CTRL_GUN_DEV					= 3,
	HID_PAGE_GAME_CTRL_POINT_OF_VIEW			= 0x20,
	HID_PAGE_GAME_CTRL_TURN_RIGHT_LEFT			= 0x21,
	HID_PAGE_GAME_CTRL_PITCH_FORWARD_BACKWARD	= 0x22,
	HID_PAGE_GAME_CTRL_ROLL_RIGHT_LEFT			= 0x23,
	HID_PAGE_GAME_CTRL_MOVE_RIGHT_LEFT			= 0x24,
	HID_PAGE_GAME_CTRL_MOVE_FORWARD_BACKWARD	= 0x25,
	HID_PAGE_GAME_CTRL_MOVE_UP_DOWN				= 0x26,
	HID_PAGE_GAME_CTRL_LEAN_RIGHT_LEFT			= 0x27,
	HID_PAGE_GAME_CTRL_LEAN_FORWARD_BACKWARD	= 0x28,
	HID_PAGE_GAME_CTRL_HEIGHT_POV				= 0x29,
	HID_PAGE_GAME_CTRL_FLIPPER					= 0x2A,
	HID_PAGE_GAME_CTRL_SECONDARY_FLIPPER		= 0x2B,
	HID_PAGE_GAME_CTRL_BUMP						= 0x2C,
	HID_PAGE_GAME_CTRL_NEW_GAME					= 0x2D,
	HID_PAGE_GAME_CTRL_SHOOT_BALL				= 0x2E,
	HID_PAGE_GAME_CTRL_PLAYER					= 0x2F,
	HID_PAGE_GAME_CTRL_GUN_BOLT					= 0x30,
	HID_PAGE_GAME_CTRL_GUN_CLIP					= 0x31,
	HID_PAGE_GAME_CTRL_GUN_SELECTOR				= 0x32,
	HID_PAGE_GAME_CTRL_GUN_SINGLE_SHOT			= 0x33,
	HID_PAGE_GAME_CTRL_GUN_BURST				= 0x34,
	HID_PAGE_GAME_CTRL_GUN_AUTOMATIC			= 0x35,
	HID_PAGE_GAME_CTRL_GUN_SAFETY				= 0x36,
	HID_PAGE_GAME_CTRL_GAMEPAD_FIRE_JUMP		= 0x37,
	HID_PAGE_GAME_CTRL_GAMEPAD_TRIGGER			= 0x39,
	HID_PAGE_GAME_CTRL_FORM_FITTING_GAMEPAD		= 0x3A
} HID_PAGE_GAME_CTRL;

#pragma pack(push, 1)

typedef struct __HID_Report_Descriptor {
	uint8_t bType;					//!< type of class descriptor. See Section 7.1.2:
									//!< Set_Descriptor Request for a table of class
									//!< descriptor constants.
	uint16_t Length;				//!< total size of the Report descriptor.
} HIDReportDesc_t;

typedef struct __HID_Physical_Descriptor {

} HIDPhysicalDesc_t;

typedef struct __HID_Descriptor {
	uint8_t bLength;				//!< Total size of the HID descriptor.
	uint8_t bDescriptorType;		//!< type of HID descriptor.
	uint16_t bcdHID;				//!< HID Class Specification release.
	uint8_t bCountryCode;			//!< country code of the localized hardware.
	uint8_t bNumDescriptors;		//!< number of class descriptors (always at
									//!< least one i.e. Report descriptor.)
	HIDReportDesc_t Descriptor[1];	//!< Array of HID report descriptor class
} HIDDesc_t;


#pragma pack(pop)

/** @} end group HID */

#endif	// __HIDDEF_H__
