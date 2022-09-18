/**-------------------------------------------------------------------------
@file	ble_hcidef.h

@brief	BLE standard HCI definitions

@author	Hoang Nguyen Hoan
@date	Sep. 12, 2022

@license

MIT License

Copyright (c) 2022, I-SYST, all rights reserved

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
#ifndef __BLE_HCIDEF_H__
#define __BLE_HCIDEF_H__

#include <stdint.h>

/** @addtogroup Bluetooth
 * @{ */

// Link Control Commands
// OGF = 1
#define BLE_HCI_CMD_LINKCTRL			1

#define BLE_HCI_CMD_LINKCTRL_INQUIRY						((1<<10) | 1)		//!< Inquiry command
#define BLE_HCI_CMD_LINKCTRL_INQUIRY_CANCEL					((1<<10) | 2)		//!< Inquiry Cancel command
#define BLE_HCI_CMD_LINKCTRL_PERIODIC_INQUIRY_MODE			((1<<10) | 3)		//!< Periodic Inquiry Mode command
#define BLE_HCI_CMD_LINKCTRL_PERIODIC_INQUIRY_MODE_EXIT		((1<<10) | 4)		//!< Exit Periodic Inquiry Mode command
#define BLE_HCI_CMD_LINKCTRL_CREATE_CONN					((1<<10) | 5)		//!< Create Connection command
#define BLE_HCI_CMD_LINKCTRL_DISCONNECT						((1<<10) | 6)		//!< Disconnect command
#define BLE_HCI_CMD_LINKCTRL_ADD_SCO_CONNECTION				((1<<10) | 7)		//!< Create an SCO connection defined by the connection handle parameters
#define BLE_HCI_CMD_LINKCTRL_CREATE_CONN_CANCEL				((1<<10) | 8)		//!< Create Connection Cancel command
#define BLE_HCI_CMD_LINKCTRL_ACCEPT_CONN_RQST				((1<<10) | 9)		//!< Accept Connection Request command
#define BLE_HCI_CMD_LINKCTRL_REJECT_CONN_RQST				((1<<10) | 0xA)		//!< Reject Connection Request command
#define BLE_HCI_CMD_LINKCTRL_LINK_KEY_RQST_REPLY			((1<<10) | 0xB)		//!< Link Key Request Reply command
#define BLE_HCI_CMD_LINKCTRL_LINK_KEY_RQST_NEG_REPLY		((1<<10) | 0xC)		//!< Link Key Request Negative Reply command
#define BLE_HCI_CMD_LINKCTRL_PINCODE_RQST_REPLY				((1<<10) | 0xD)		//!< PIN Code Request Reply command
#define BLE_HCI_CMD_LINKCTRL_PINCODE_RQST_NEG_REPLY			((1<<10) | 0xE)		//!< PIN Code Request Negative Reply command
#define BLE_HCI_CMD_LINKCTRL_CHANGE_CONN_PACKET_TYPE		((1<<10) | 0xF)		//!< Change Connection Packet Type command
#define BLE_HCI_CMD_LINKCTRL_AUTHEN_RQST					((1<<10) | 0x11)	//!< Authentication Requested command
#define BLE_HCI_CMD_LINKCTRL_SET_CONN_ENCRYPTION			((1<<10) | 0x13)	//!< Set Connection Encryption command
#define BLE_HCI_CMD_LINKCTRL_CHANGE_CONN_LINK_KEY			((1<<10) | 0x15)	//!< Change Connection Link Key command
#define BLE_HCI_CMD_LINKCTRL_LINK_KEY_SELECTION				((1<<10) | 0x17)	//!< Link Key Selection command
#define BLE_HCI_CMD_LINKCTRL_REMOTE_NAME_RQST				((1<<10) | 0x19)	//!< Remote Name Request command
#define BLE_HCI_CMD_LINKCTRL_REMOTE_NAME_RQST_CANCEL		((1<<10) | 0x1A)	//!< Remote Name Request Cancel command
#define BLE_HCI_CMD_LINKCTRL_READ_REMOTE_SUPPORTED_FEATURES	((1<<10) | 0x1B)	//!< Read Remote Supported Features command
#define BLE_HCI_CMD_LINKCTRL_READ_REMOTE_EXT_FEATURES		((1<<10) | 0x1C)	//!< Read Remote Extended Features command
#define BLE_HCI_CMD_LINKCTRL_READ_REMOTE_VERS_INFO			((1<<10) | 0x1D)	//!< Read Remote Version Information command
#define BLE_HCI_CMD_LINKCTRL_READ_CLOCK_OFFSET				((1<<10) | 0x1F)	//!< Read Clock Offset command
#define BLE_HCI_CMD_LINKCTRL_READ_LMP_HANDLE				((1<<10) | 0x20)	//!< Read LMP Handle command
#define BLE_HCI_CMD_LINKCTRL_SETUP_SYNCHRONOUS_CONN			((1<<10) | 0x28)	//!< Setup Synchronous Connection command
#define BLE_HCI_CMD_LINKCTRL_ACCEPT_SYNCHRONOUS_CONN_RQST	((1<<10) | 0x29)	//!< Accept Synchronous Connection Request command
#define BLE_HCI_CMD_LINKCTRL_REJECT_SYNCHRONOUS_CONN_RQST	((1<<10) | 0x2A)	//!< Reject Synchronous Connection Request command
#define BLE_HCI_CMD_LINKCTRL_IO_CAPABILITY_RQST_REPLY		((1<<10) | 0x2B)	//!< IO Capability Request Reply command
#define BLE_HCI_CMD_LINKCTRL_USER_INFO_RQST_REPLY			((1<<10) | 0x2C)	//!< User Confirmation Request Reply command
#define BLE_HCI_CMD_LINKCTRL_USER_INFO_RQST_NEG_REPLY		((1<<10) | 0x2D)	//!< User Confirmation Request Negative Reply command
#define BLE_HCI_CMD_LINKCTRL_USER_PASSKEY_RQST_REPLY		((1<<10) | 0x2E)	//!< User Passkey Request Reply command
#define BLE_HCI_CMD_LINKCTRL_USER_PASSKEY_RQST_NEG_REPLY	((1<<10) | 0x2F)	//!< User Passkey Request Negative Reply command
#define BLE_HCI_CMD_LINKCTRL_REMOTE_OOB_DATA_RQST_REPLY		((1<<10) | 0x30)	//!< Remote OOB Data Request Reply command
#define BLE_HCI_CMD_LINKCTRL_REMOTE_OOB_DATA_RQST_NEG_REPLY	((1<<10) | 0x33)	//!< Remote OOB Data Request Negative Reply command
#define BLE_HCI_CMD_LINKCTRL_IO_CAPABILITY_RQST_NEG_REPLY	((1<<10) | 0x34)	//!< IO Capability Request Negative Reply command
#define BLE_HCI_CMD_LINKCTRL_ENHANCED_SETUP_SYNCHRONOUS_CONN	((1<<10) | 0x3D)	//!< Enhanced Setup Synchronous Connection command
#define BLE_HCI_CMD_LINKCTRL_ENHANCED_ACCEPT_SYNCHRONOUS_CONN_RQST	((1<<10) | 0x3E)	//!< Enhanced Accept Synchronous Connection Request command
#define BLE_HCI_CMD_LINKCTRL_TRUNCATED_PAGE					((1<<10) | 0x3F)	//!< Truncated Page command
#define BLE_HCI_CMD_LINKCTRL_TRUNCATED_PAGE_CANCEL			((1<<10) | 0x40)	//!< Truncated Page Cancel command
#define BLE_HCI_CMD_LINKCTRL_SET_CONNLESS_PERIPH_BROADCAST	((1<<10) | 0x41)	//!< Set Connectionless Peripheral Broadcast command
#define BLE_HCI_CMD_LINKCTRL_SET_CONNLESS_PERIPH_BROADCAST_RECEIVE	((1<<10) | 0x42)	//!< Set Connectionless Peripheral Broadcast Receive command
#define BLE_HCI_CMD_LINKCTRL_START_SYNC_TRAIN				((1<<10) | 0x43)	//!< Start Synchronization Train command
#define BLE_HCI_CMD_LINKCTRL_RECEIVE_SYNC_TRAIN				((1<<10) | 0x44)	//!< Receive Synchronization Train command
#define BLE_HCI_CMD_LINKCTRL_REMOTE_OOB_EXT_DATA_RQST_REPLY	((1<<10) | 0x45)	//!< Remote OOB Extended Data Request Reply command


// HCI Policy Command
// OGF = 2
#define BLE_HCI_CMD_POLICY			2

#define BLE_HCI_CMD_POLICY_HOLD_MODE						((2<<10) | 1)		//!< Hold Mode command
#define BLE_HCI_CMD_POLICY_SNIFF_MODE						((2<<10) | 3)		//!< Sniff Mode command
#define BLE_HCI_CMD_POLICY_EXIT_SNIFF_MODE					((2<<10) | 4)		//!< Exit Sniff Mode command
#define BLE_HCI_CMD_POLICY_QOS_SETUP						((2<<10) | 7)		//!< QoS Setup command
#define BLE_HCI_CMD_POLICY_ROLE_DISCOVERY					((2<<10) | 9)		//!< Role Discovery command
#define BLE_HCI_CMD_POLICY_SWITCH_ROLE						((2<<10) | 0xB)		//!< Switch Role command
#define BLE_HCI_CMD_POLICY_READ_LINK_POLICY_SETTINGS		((2<<10) | 0xC)		//!< Read Link Policy Settings command
#define BLE_HCI_CMD_POLICY_WRITE_LINK_POLICY_SETTINGS		((2<<10) | 0xD)		//!< Write Link Policy Settings command
#define BLE_HCI_CMD_POLICY_READ_DEFAULT_LINK_POLICY_SETTINGS	((2<<10) | 0xE)		//!< Read Default Link Policy Settings command
#define BLE_HCI_CMD_POLICY_WRITE_DEFAULT_LINK_POLICY_SETTINGS	((2<<10) | 0xF)		//!< Write Default Link Policy Settings command
#define BLE_HCI_CMD_POLICY_FLOW_SPECIFICATION				((2<<10) | 0x10)	//!< Flow Specification command
#define BLE_HCI_CMD_POLICY_SNIFF_SUBRATING					((2<<10) | 0x11)	//!< Sniff Subrating command


// Host Controller and Baseband Command
// OGF = 3
#define BLE_HCI_CMD_BASEBAND			3

#define BLE_HCI_CMD_BASEBAND_SET_EVENT_MASK					((3<<10) | 1)		//!< Set Event Mask command
#define BLE_HCI_CMD_BASEBAND_RESET							((3<<10) | 3)		//!< Reset command
#define BLE_HCI_CMD_BASEBAND_SET_EVENT_FILTER				((3<<10) | 5)		//!< Set Event Filter command
#define BLE_HCI_CMD_BASEBAND_FLUSH							((3<<10) | 8)		//!< Flush command
#define BLE_HCI_CMD_BASEBAND_READ_PIN_TYPE					((3<<10) | 9)		//!< Read PIN Type command
#define BLE_HCI_CMD_BASEBAND_WRITE_PIN_TYPE					((3<<10) | 0xA)		//!< Write PIN Type command
//#define BLE_HCI_CMD_BASEBAND_CREATE_NEW_UNIT_KEY			((3<<10) | 0xB)		//!< Create a new unit key.
#define BLE_HCI_CMD_BASEBAND_READ_STORED_LINK_KEY			((3<<10) | 0xD)		//!< Read Stored Link Key command
#define BLE_HCI_CMD_BASEBAND_WRITE_STORED_LINK_KEY			((3<<10) | 0x11)	//!< Write Stored Link Key command
#define BLE_HCI_CMD_BASEBAND_DELETE_STORED_LINK_KEY			((3<<10) | 0x12)	//!< Delete Stored Link Key command
#define BLE_HCI_CMD_BASEBAND_WRITE_LOCAL_NAME				((3<<10) | 0x13)	//!< Write Local Name command
#define BLE_HCI_CMD_BASEBAND_READ_LOCAL_NAME				((3<<10) | 0x14)	//!< Read Local Name command
#define BLE_HCI_CMD_BASEBAND_READ_CONN_ACCEPT_TIMEOUT		((3<<10) | 0x15)	//!< Read Connection Accept Timeout command
#define BLE_HCI_CMD_BASEBAND_WRITE_CONN_ACCEPT_TIMEOPUT		((3<<10) | 0x16)	//!< Write Connection Accept Timeout command
#define BLE_HCI_CMD_BASEBAND_READ_PAGE_TIMEOUT				((3<<10) | 0x17)	//!< Read Page Timeout command
#define BLE_HCI_CMD_BASEBAND_WRITE_PAGE_TIMEOUT				((3<<10) | 0x18)	//!< Write Page Timeout command
#define BLE_HCI_CMD_BASEBAND_READ_SCAN_ENABLE				((3<<10) | 0x19)	//!< Read Scan Enable command
#define BLE_HCI_CMD_BASEBAND_WRITE_SCAN_ENABLE				((3<<10) | 0x1A)	//!< Write Scan Enable command
#define BLE_HCI_CMD_BASEBAND_READ_PAGE_SCAN_ACTIVITY		((3<<10) | 0x1B)	//!< Read Page Scan Activity command
#define BLE_HCI_CMD_BASEBAND_WRITE_PAGE_SCAN_ACTIVITY		((3<<10) | 0x1C)	//!< Write Page Scan Activity command
#define BLE_HCI_CMD_BASEBAND_READ_INQUIRY_SCAN_ACTIVITY		((3<<10) | 0x1D)	//!< Read Inquiry Scan Activity command
#define BLE_HCI_CMD_BASEBAND_WRITE_INQUIRY_SCAN_ACTIVITY	((3<<10) | 0x1E)	//!< Write Inquiry Scan Activity command
#define BLE_HCI_CMD_BASEBAND_READ_AUTHEN_ENABLE				((3<<10) | 0x1F)	//!< Read Authentication Enable command
#define BLE_HCI_CMD_BASEBAND_WRITE_AUTHEN_ENABLE			((3<<10) | 0x20)	//!< Write Authentication Enable command
#define BLE_HCI_CMD_BASEBAND_READ_ENCRYPTION_MODE			((3<<10) | 0x21)	//!<
#define BLE_HCI_CMD_BASEBAND_WRITE_ENCRYPTION_MODE			((3<<10) | 0x22)	//!<
#define BLE_HCI_CMD_BASEBAND_READ_CLASS_OF_DEVICE			((3<<10) | 0x23)	//!< Read Class of Device command
#define BLE_HCI_CMD_BASEBAND_WRITE_CLASS_OF_DEVICE			((3<<10) | 0x24)	//!< Write Class of Device command
#define BLE_HCI_CMD_BASEBAND_READ_VOICE_SETTING				((3<<10) | 0x25)	//!< Read Voice Setting command
#define BLE_HCI_CMD_BASEBAND_WRITE_VOICE_SETTING			((3<<10) | 0x26)	//!< Write Voice Setting command
#define BLE_HCI_CMD_BASEBAND_READ_AUTO_FLUSH_TIMEOUT		((3<<10) | 0x27)	//!< Read Automatic Flush Timeout command
#define BLE_HCI_CMD_BASEBAND_WRITE_AUTO_FLUSH_TIMEOUT		((3<<10) | 0x28)	//!< Write Automatic Flush Timeout command
#define BLE_HCI_CMD_BASEBAND_READ_NB_BROADCAST_RETRANS		((3<<10) | 0x29)	//!< Read Num Broadcast Retransmissions command
#define BLE_HCI_CMD_BASEBAND_WRITE_NB_BROADCAST_RETRANS		((3<<10) | 0x2A)	//!< Write Num Broadcast Retransmissions command
#define BLE_HCI_CMD_BASEBAND_READ_HOLD_MODE_ACTIVITY		((3<<10) | 0x2B)	//!< Read Hold Mode Activity command
#define BLE_HCI_CMD_BASEBAND_WRITE_HOLD_MODE_ACTIVITY		((3<<10) | 0x2C)	//!< Write Hold Mode Activity command
#define BLE_HCI_CMD_BASEBAND_READ_TRANSMIT_POWER_LEVEL		((3<<10) | 0x2D)	//!< Read Transmit Power Level command
#define BLE_HCI_CMD_BASEBAND_READ_SYNCHRONOUS_FLOWCTRL_ENABLE	((3<<10) | 0x2E)	//!< Read Synchronous Flow Control Enable command
#define BLE_HCI_CMD_BASEBAND_WRITE_SYNCHRONOUS_FLOWCTRL_ENABLE	((3<<10) | 0x2F)	//!< Write Synchronous Flow Control Enable command
#define BLE_HCI_CMD_BASEBAND_SET_FLOWCTRL					((3<<10) | 0x31)	//!< Set Controller To Host Flow Control command
#define BLE_HCI_CMD_BASEBAND_HOST_BUFFER_SIZE				((3<<10) | 0x33)	//!< Host Buffer Size command
#define BLE_HCI_CMD_BASEBAND_HOST_NB_COMPLETE_PACKETS		((3<<10) | 0x35)	//!< Host Number Of Completed Packets command
#define BLE_HCI_CMD_BASEBAND_READ_LINK_SUPERV_TIMEOUT		((3<<10) | 0x36)	//!< Read Link Supervision Timeout command
#define BLE_HCI_CMD_BASEBAND_WRITE_LINK_SUPERV_TIMEOUT		((3<<10) | 0x37)	//!< Write Link Supervision Timeout command
#define BLE_HCI_CMD_BASEBAND_READ_NB_SUPPORTED_IAC			((3<<10) | 0x38)	//!< Read Number Of Supported IAC command
#define BLE_HCI_CMD_BASEBAND_READ_CURRENT_IAC_LAP			((3<<10) | 0x39)	//!< Read Current IAC LAP command
#define BLE_HCI_CMD_BASEBAND_WRITE_CURRENT_IAC_LAP			((3<<10) | 0x3A)	//!< Write Current IAC LAP command
#define BLE_HCI_CMD_BASEBAND_READ_PAGE_SCAN_PERIOD_MODE		((3<<10) | 0x3B)	//!<
#define BLE_HCI_CMD_BASEBAND_WRITE_PAGE_SCAN_PERIOD_MODE	((3<<10) | 0x3C)	//!< Set the timeout session of a page scan.
#define BLE_HCI_CMD_BASEBAND_READ_PAGE_SCAN_MODE			((3<<10) | 0x3D)	//!< Read the default Page scan mode.
#define BLE_HCI_CMD_BASEBAND_WRITE_PAGE_SCAN_MODE			((3<<10) | 0x3E)	//!< Set the default page scan mode.
#define BLE_HCI_CMD_BASEBAND_SET_AFH_CHAN_CLASS				((3<<10) | 0x3F)	//!< Set AFH Host Channel Classification command
#define BLE_HCI_CMD_BASEBAND_READ_INQUIRY_SCAN_TYPE			((3<<10) | 0x42)	//!< Read Inquiry Scan Type command
#define BLE_HCI_CMD_BASEBAND_WRITE_INQUIRY_SCAN_TYPE		((3<<10) | 0x43)	//!< Write Inquiry Scan Type command
#define BLE_HCI_CMD_BASEBAND_READ_INQUIRY_MODE				((3<<10) | 0x44)	//!< Read Inquiry Mode command
#define BLE_HCI_CMD_BASEBAND_WRITE_INQUIRY_MODE				((3<<10) | 0x45)	//!< Write Inquiry Mode command
#define BLE_HCI_CMD_BASEBAND_READ_PAGE_SCAN_TYPE			((3<<10) | 0x46)	//!< Read Page Scan Type command
#define BLE_HCI_CMD_BASEBAND_WRITE_PAGE_SCAN_TYPE			((3<<10) | 0x47)	//!< Write Page Scan Type command
#define BLE_HCI_CMD_BASEBAND_READ_AFH_CHAN_ASSESS_MODE		((3<<10) | 0x48)	//!< Read AFH Channel Assessment Mode command
#define BLE_HCI_CMD_BASEBAND_WRITE_AFH_CHAN_ASSESS_MODE		((3<<10) | 0x49)	//!< Write AFH Channel Assessment Mode command
#define BLE_HCI_CMD_BASEBAND_READ_EXT_INQUIRY_RESPONSE		((3<<10) | 0x51)	//!< Read Extended Inquiry Response command
#define BLE_HCI_CMD_BASEBAND_WRITE_EXT_INQUIRY_RESPONSE		((3<<10) | 0x52)	//!< Write Extended Inquiry Response command
#define BLE_HCI_CMD_BASEBAND_REFRESH_ENCRYPTION_KEY			((3<<10) | 0x53)	//!< Refresh Encryption Key command
#define BLE_HCI_CMD_BASEBAND_READ_SIMPLE_PAIRING_MODE		((3<<10) | 0x55)	//!< Read Simple Pairing Mode command
#define BLE_HCI_CMD_BASEBAND_WRITE_SIMPLE_PAIRING_MODE		((3<<10) | 0x56)	//!< Write Simple Pairing Mode command
#define BLE_HCI_CMD_BASEBAND_READ_LOCAL_OOB_DATA			((3<<10) | 0x57)	//!< Read Local OOB Data command
#define BLE_HCI_CMD_BASEBAND_READ_INQUIRY_RESP_TRANSMIT_PWR_LEVEL	((3<<10) | 0x58)	//!< Read Inquiry Response Transmit Power Level command
#define BLE_HCI_CMD_BASEBAND_WRITE_INQUIRY_TRANSMIT_PWR_LEVEL	((3<<10) | 0x59)	//!< Write Inquiry Transmit Power Level command
#define BLE_HCI_CMD_BASEBAND_READ_DEFAULT_ERR_DATA_REPORT	((3<<10) | 0x5A)	//!< Read Default Erroneous Data Reporting command
#define BLE_HCI_CMD_BASEBAND_WRITE_DEFAULT_ERR_DATA_REPORT	((3<<10) | 0x5B)	//!< Write Default Erroneous Data Reporting command
#define BLE_HCI_CMD_BASEBAND_ENHANCED_FLUSH					((3<<10) | 0x5F)	//!< Enhanced Flush command
#define BLE_HCI_CMD_BASEBAND_SEND_KEYPRESS_NOTIF			((3<<10) | 0x60)	//!< Send Keypress Notification command
#define BLE_HCI_CMD_BASEBAND_SET_EVT_MASK_PAGE_2			((3<<10) | 0x63)	//!< Set Event Mask Page 2 command
#define BLE_HCI_CMD_BASEBAND_READ_FLOWCTRL_MODE				((3<<10) | 0x66)	//!< Read Flow Control Mode command
#define BLE_HCI_CMD_BASEBAND_WRITE_FLOWCTRL_MODE			((3<<10) | 0x67)	//!< Write Flow Control Mode command
#define BLE_HCI_CMD_BASEBAND_READ_ENHANCED_TRANSMIT_PWR_LEVEL	((3<<10) | 0x68)	//!< Read Enhanced Transmit Power Level command
#define BLE_HCI_CMD_BASEBAND_READ_LE_HOST_SUPPORT			((3<<10) | 0x6C)	//!< Read LE Host Support command
#define BLE_HCI_CMD_BASEBAND_WRITE_LE_HOST_SUPPORT			((3<<10) | 0x6D)	//!< Write LE Host Support command
#define BLE_HCI_CMD_BASEBAND_SET_MWS_CHAN_PARAM				((3<<10) | 0x6E)	//!< Set MWS Channel Parameters command
#define BLE_HCI_CMD_BASEBAND_SET_EXTERNAL_FRAME_CONF		((3<<10) | 0x6F)	//!< Set External Frame Configuration command
#define BLE_HCI_CMD_BASEBAND_SET_MWS_SIGNALING				((3<<10) | 0x70)	//!< Set MWS Signaling command
#define BLE_HCI_CMD_BASEBAND_SET_MWS_TRANSPORT_LAYER		((3<<10) | 0x71)	//!< Set MWS Transport Layer command
#define BLE_HCI_CMD_BASEBAND_SET_MWS_SCAN_FrEQ_TABLE		((3<<10) | 0x72)	//!< Set MWS Scan Frequency Table command
#define BLE_HCI_CMD_BASEBAND_SET_MWS_PATTERN_CONFIG			((3<<10) | 0x73)	//!< Set MWS_PATTERN Configuration command
#define BLE_HCI_CMD_BASEBAND_SET_RESERVED_LT_ADDR			((3<<10) | 0x74)	//!< Set Reserved LT_ADDR command
#define BLE_HCI_CMD_BASEBAND_DELETE_RESERVED_LT_ADDR		((3<<10) | 0x75)	//!< Delete Reserved LT_ADDR command
#define BLE_HCI_CMD_BASEBAND_SET_CONNLESS_PERIPH_BROADCAST_DATA	((3<<10) | 0x76)	//!< Set Connectionless Peripheral Broadcast Data command
#define BLE_HCI_CMD_BASEBAND_READ_SYNC_TRAIN_PARAM			((3<<10) | 0x77)	//!< Read Synchronization Train Parameters command
#define BLE_HCI_CMD_BASEBAND_WRITE_SYNC_TRAIN_PARAM			((3<<10) | 0x78)	//!< Write Synchronization Train Parameters command
#define BLE_HCI_CMD_BASEBAND_READ_SECURE_CONN_HOST_SUPPORT	((3<<10) | 0x79)	//!< Read Secure Connections Host Support command
#define BLE_HCI_CMD_BASEBAND_WRITE_SECURE_CONN_HOST_SUPPORT	((3<<10) | 0x7A)	//!< Write Secure Connections Host Support command
#define BLE_HCI_CMD_BASEBAND_READ_AUTH_PAYLOAD_TIMEOUT		((3<<10) | 0x7B)	//!< Read Authenticated Payload Timeout command
#define BLE_HCI_CMD_BASEBAND_WRITE_AUTH_PAYLOAD_TIMEOUT		((3<<10) | 0x7C)	//!< Write Authenticated Payload Timeout command
#define BLE_HCI_CMD_BASEBAND_READ_LOCAL_OOB_EXT_DATA		((3<<10) | 0x7D)	//!< Read Local OOB Extended Data command
#define BLE_HCI_CMD_BASEBAND_READ_EXT_PAGE_TIMEOUT			((3<<10) | 0x7E)	//!< Read Extended Page Timeout command
#define BLE_HCI_CMD_BASEBAND_WRITE_EXT_PAGE_TIMEOUT			((3<<10) | 0x7F)	//!< Write Extended Page Timeout command
#define BLE_HCI_CMD_BASEBAND_READ_EXT_INQUIRY_LEN			((3<<10) | 0x80)	//!< Read Extended Inquiry Length command
#define BLE_HCI_CMD_BASEBAND_WRITE_EXT_INQUIRY_LEN			((3<<10) | 0x81)	//!< Write Extended Inquiry Length command
#define BLE_HCI_CMD_BASEBAND_SET_ECOSYSTEM_BAS_INTERVAL		((3<<10) | 0x82)	//!< Set Ecosystem Base Interval command
#define BLE_HCI_CMD_BASEBAND_CONFIG_DATA_PATH				((3<<10) | 0x83)	//!< Configure Data Path command
#define BLE_HCI_CMD_BASEBAND_SET_MIN_ENCRYPTION_KEY_SIZE	((3<<10) | 0x84)	//!< Set Min Encryption Key Size command


// INFORMATIONAL PARAMETERS
// OGF = 4
#define BLE_HCI_CMD_INFO				4

#define BLE_HCI_CMD_INFO_READ_LOCAL_VERS_INFO				((4<<10) | 1)		//!< Read Local Version Information command
#define BLE_HCI_CMD_INFO_READ_LOCAL_SUPPORTED_COMMANDS		((4<<10) | 2)		//!< Read Local Supported Commands command
#define BLE_HCI_CMD_INFO_READ_LOCAL_SUPPORTED_FEATURES		((4<<10) | 3)		//!< Read Local Supported Features command
#define BLE_HCI_CMD_INFO_READ_LOCAL_EXT_FEATURES			((4<<10) | 4)		//!< Read Local Extended Features command
#define BLE_HCI_CMD_INFO_READ_BUFFER_SIZE					((4<<10) | 5)		//!< Read Buffer Size command
#define BLE_HCI_CMD_INFO_READ_BD_ADDR						((4<<10) | 9)		//!< Read BD_ADDR command
#define BLE_HCI_CMD_INFO_READ_DATA_BLOCK_SIZE				((4<<10) | 0xA)		//!< Read Data Block Size command
#define BLE_HCI_CMD_INFO_READ_LOCAL_SUPPORTED_CODeCS_V1		((4<<10) | 0xB)		//!< Read Local Supported Codecs command V1
#define BLE_HCI_CMD_INFO_READ_LOCAL_SUPPORTED_CODeCS_V2		((4<<10) | 0xD)		//!< Read Local Supported Codecs command V2
#define BLE_HCI_CMD_INFO_READ_LOCAL_SIMPLE_PAIRING_OPTIONS	((4<<10) | 0xC)		//!< Read Local Simple Pairing Options command
#define BLE_HCI_CMD_INFO_READ_LOCAL_SUPPORTED_CODEC_CAPABILITIES	((4<<10) | 0xE)	//!< Read Local Supported Codec Capabilities command
#define BLE_HCI_CMD_INFO_READ_LOCAL_SUPPORTED_CTLR_DELAY	((4<<10) | 0xF)		//!< Read Local Supported Controller Delay command

// STATUS PARAMETERS
// OGF = 5
#define BLE_HCI_CMD_STATUS				5

#define BLE_HCI_CMD_STATUS_READ_FAILED_CONTACT_COUNTER		((5<<10) | 1)		//!< Read Failed Contact Counter command
#define BLE_HCI_CMD_STATUS_RESET_FAILED_CONTACT_COUNTER		((5<<10) | 2)		//!< Reset Failed Contact Counter command
#define BLE_HCI_CMD_STATUS_READ_LINK_QUALITY				((5<<10) | 3)		//!< Read Link Quality command
#define BLE_HCI_CMD_STATUS_READ_RSSI						((5<<10) | 5)		//!< Read RSSI command
#define BLE_HCI_CMD_STATUS_READ_AFH_CHAN_MAP				((5<<10) | 6)		//!< Read AFH Channel Map command
#define BLE_HCI_CMD_STATUS_READ_CLOCK						((5<<10) | 7)		//!< Read Clock command
#define BLE_HCI_CMD_STATUS_READ_ENCRYPTION_KEY_SIZE			((5<<10) | 8)		//!< Read Encryption Key Size command
#define BLE_HCI_CMD_STATUS_GET_MWS_TRANSPORT_LAYER_CONFIG	((5<<10) | 0xC)		//!< Get MWS Transport Layer Configuration command
#define BLE_HCI_CMD_STATUS_SET_TRIGGERED_CLOCK_CAPTURE		((5<<10) | 0xD)		//!< Set Triggered Clock Capture command


// TESTING COMMANDS
// OGF = 6
#define BLE_HCI_CMD_TEST				6

#define BLE_HCI_CMD_TEST_READ_LOOPBACK_MODE					((6<<10) | 1)		//!< Read Loopback Mode command
#define BLE_HCI_CMD_TEST_WRITE_LOOPBACK_MODE				((6<<10) | 2)		//!< Write Loopback Mode command
#define BLE_HCI_CMD_TEST_ENABLE_DUT_MODE					((6<<10) | 3)		//!< Enable Device Under Test Mode command
#define BLE_HCI_CMD_TEST_WRITE_SIMPLE_PAIRING_DEBUG_MODE	((6<<10) | 4)		//!< Write Simple Pairing Debug Mode command
#define BLE_HCI_CMD_TEST_WRITE_SECURE_CONN_TEST_MODE		((6<<10) | 0xA)		//!< Write Secure Connections Test Mode command

// LE Controller Commands
// OGF = 8

#define BLE_HCI_CMD_CTLR				8

#define BLE_HCI_CMD_CTLR_SET_EVT_MASK						((8<<10) | 1)		//!< LE Set Event Mask command
#define BLE_HCI_CMD_CTLR_READ_BUFF_SIZE						((8<<10) | 2)		//!< LE Read Buffer Size command V1
#define BLE_HCI_CMD_CTLR_READ_BUFF_SIZE_EXT					((8<<10) | 0x60)	//!< LE Read Buffer Size command V2
#define BLE_HCI_CMD_CTLR_READ_LOCAL_SUPP_FEATURES			((8<<10) | 3)		//!< LE Read Local Supported Features command
#define BLE_HCI_CMD_CTLR_SET_RAND_ADDR						((8<<10) | 5)		//!< LE Set Random Address command
#define BLE_HCI_CMD_CTLR_SET_ADV_PARAM						((8<<10) | 6)		//!< LE Set Advertising Parameters command
#define BLE_HCI_CMD_CTLR_READ_ADV_CHAN_TX_PWR				((8<<10) | 7)		//!< LE Read Advertising Physical Channel Tx Power command
#define BLE_HCI_CMD_CTLR_SET_ADV_DATA						((8<<10) | 8)		//!< LE Set Advertising Data command
#define BLE_HCI_CMD_CTLR_SET_SCAN_RESP_DATA					((8<<10) | 9)		//!< LE Set Scan Response Data command
#define BLE_HCI_CMD_CTLR_SET_ADV_ENABLE						((8<<10) | 0xA)		//!< LE Set Advertising Enable command
#define BLE_HCI_CMD_CTLR_SET_SCAN_PARAM						((8<<10) | 0xB)		//!< LE Set Scan Parameters command
#define BLE_HCI_CMD_CTLR_SET_SCAN_ENABLE					((8<<10) | 0xC)		//!< LE Set Scan Enable command
#define BLE_HCI_CMD_CTLR_CREATE_CONN						((8<<10) | 0xD)		//!< LE Create Connection command
#define BLE_HCI_CMD_CTLR_CREATE_CONN_CANCEL					((8<<10) | 0xE)		//!< LE Create Connection Cancel command
#define BLE_HCI_CMD_CTLR_READ_FILTER_ACCEPT_LIST_SIZE		((8<<10) | 0xF)		//!< LE Read Filter Accept List Size command
#define BLE_HCI_CMD_CTLR_CLEAR_FILTER_ACCEPT_LIST			((8<<10) | 0x10)	//!< LE Clear Filter Accept List command
#define BLE_HCI_CMD_CTLR_FILTER_ACCEPT_LIST_ADD_DEV			((8<<10) | 0x11)	//!< LE Add Device To Filter Accept List command
#define BLE_HCI_CMD_CTLR_FILTER_ACCEPT_LIST_REMOVE_DEV		((8<<10) | 0x12)	//!< LE Remove Device From Filter Accept List command
#define BLE_HCI_CMD_CTLR_CONN_UPDATE						((8<<10) | 0x13)	//!< LE Connection Update command
#define BLE_HCI_CMD_CTLR_SET_HOST_CHAN_CLASS				((8<<10) | 0x14)	//!< LE Set Host Channel Classification command
#define BLE_HCI_CMD_CTLR_READ_CHAN_MAP						((8<<10) | 0x15)	//!< LE Read Channel Map command
#define BLE_HCI_CMD_CTLR_READ_REMOTE_FEATURES				((8<<10) | 0x16)	//!< LE Read Remote Features command
#define BLE_HCI_CMD_CTLR_ENCRYPT							((8<<10) | 0x17)	//!< LE Encrypt command
#define BLE_HCI_CMD_CTLR_RAND								((8<<10) | 0x18)	//!< LE Rand command
#define BLE_HCI_CMD_CTLR_ENABLE_ENCRYPTION					((8<<10) | 0x19)	//!< LE Enable Encryption command
#define BLE_HCI_CMD_CTLR_LONGTERM_KEY_REQUEST_REPLY			((8<<10) | 0x1A)	//!< LE Long Term Key Request Reply command
#define BLE_HCI_CMD_CTLR_LONGTERM_KEY_REQUEST_NEG_REPLY		((8<<10) | 0X1B)	//!< LE Long Term Key Request Negative Reply command
#define BLE_HCI_CMD_CTLR_READ_SUPPORTED_STATES				((8<<10) | 0x1C)	//!< LE Read Supported States command
#define BLE_HCI_CMD_CTLR_RECEIVER_TEST_V1					((8<<10) | 0x1D)	//!< LE Receiver Test command V1
#define BLE_HCI_CMD_CTLR_RECEIVER_TEST_V2					((8<<10) | 0x33)	//!< LE Receiver Test command V2
#define BLE_HCI_CMD_CTLR_RECEIVER_TEST_V3					((8<<10) | 0x4F)	//!< LE Receiver Test command V3
#define BLE_HCI_CMD_CTLR_TRANSMITTER_TEST_V1				((8<<10) | 0x1E)	//!< LE Transmitter Test command V1
#define BLE_HCI_CMD_CTLR_TRANSMITTER_TEST_V2				((8<<10) | 0x34)	//!< LE Transmitter Test command V2
#define BLE_HCI_CMD_CTLR_TRANSMITTER_TEST_V3				((8<<10) | 0x50)	//!< LE Transmitter Test command V3
#define BLE_HCI_CMD_CTLR_TRANSMITTER_TEST_V4				((8<<10) | 0x17B)	//!< LE Transmitter Test command V4
#define BLE_HCI_CMD_CTLR_TEST_END							((8<<10) | 0x1F)	//!< LE Test End command
#define BLE_HCI_CMD_CTLR_REMOTE_CONN_PARAM_REQUEST_REPLY	((8<<10) | 0x20)	//!< LE Remote Connection Parameter Request Reply command
#define BLE_HCI_CMD_CTLR_REMOTE_CONN_PARAM_RQST_NEG_REPLY	((8<<10) | 0x21)	//!< LE Remote Connection Parameter Request Negative Reply command
#define BLE_HCI_CMD_CTLR_SET_DATA_LEN						((8<<10) | 0x22)	//!< LE Set Data Length command
#define BLE_HCI_CMD_CTLR_READ_SUGG_DEFAULT_DATA_LEN			((8<<10) | 0x23)	//!< LE Read Suggested Default Data Length command
#define BLE_HCI_CMD_CTLR_WRITE_SUGG_DEFAULT_DATA_LEN		((8<<10) | 0x24)	//!< LE Write Suggested Default Data Length command
#define BLE_HCI_CMD_CTLR_READ_LOCAL_P256_PUBLIC_KEY			((8<<10) | 0x25)	//!< LE Read Local P-256 Public Key command
#define BLE_HCI_CMD_CTLR_GENERATE_DHKEY_V1					((8<<10) | 0x26)	//!< LE Generate DHKey command V1
#define BLE_HCI_CMD_CTLR_GENERATE_DHKEY_V2					((8<<10) | 0x5E)	//!< LE Generate DHKey command V2
#define BLE_HCI_CMD_CTLR_RESOLVING_LIST_ADD_DEV				((8<<10) | 0x27)	//!< LE Add Device To Resolving List command
#define BLE_HCI_CMD_CTLR_RESOLVING_LIST_REMOVE_DEV			((8<<10) | 0x28)	//!< LE Remove Device From Resolving List command
#define BLE_HCI_CMD_CTLR_RESOLVING_LIST_CLEAR				((8<<10) | 0x29)	//!< LE Clear Resolving List command
#define BLE_HCI_CMD_CTLR_RESOLVING_LIST_READ_SIZE			((8<<10) | 0x2A)	//!< LE Read Resolving List Size command
#define BLE_HCI_CMD_CTLR_READ_PEER_RESOLVABLE_ADDR			((8<<10) | 0x2B)	//!< LE Read Peer Resolvable Address command
#define BLE_HCI_CMD_CTLR_READ_LOCAL_RESOLVABLE_ADDR			((8<<10) | 0x2C)	//!< LE Read Local Resolvable Address command
#define BLE_HCI_CMD_CTLR_SET_ADDR_RESOLUTION_ENABLE			((8<<10) | 0x2D)	//!< LE Set Address Resolution Enable command
#define BLE_HCI_CMD_CTLR_SET_RESOLVABLE_PRIVATE_ADDR_TIMEOUT	((8<<10) | 0x2E)	//!< LE Set Resolvable Private Address Timeout command
#define BLE_HCI_CMD_CTLR_READ_MAX_DATA_LEN					((8<<10) | 0x2F)	//!< LE Read Maximum Data Length command
#define BLE_HCI_CMD_CTLR_READ_PHY							((8<<10) | 0x30)	//!< LE Read PHY command
#define BLE_HCI_CMD_CTLR_SET_DEFAULT_PHY					((8<<10) | 0x31)	//!< LE Set Default PHY command
#define BLE_HCI_CMD_CTLR_SET_PHY							((8<<10) | 0x32)	//!< LE Set PHY command
#define BLE_HCI_CMD_CTLR_SET_ADV_SET_RAND_ADDR				((8<<10) | 0x35)	//!< LE Set Advertising Set Random Address command
#define BLE_HCI_CMD_CTLR_SET_EXT_ADV_PARAM					((8<<10) | 0x36)	//!< LE Set Extended Advertising Parameters command
#define BLE_HCI_CMD_CTLR_SET_EXT_ADV_DATA					((8<<10) | 0x37)	//!< LE Set Extended Advertising Data command
#define BLE_HCI_CMD_CTLR_SET_EXT_SCAN_RESP_DATA				((8<<10) | 0x38)	//!< LE Set Extended Scan Response Data command
#define BLE_HCI_CMD_CTLR_SET_EXT_ADV_ENABLE					((8<<10) | 0x39)	//!< LE Set Extended Advertising Enable command
#define BLE_HCI_CMD_CTLR_READ_MAX_ADV_DATA_LEN				((8<<10) | 0x3A)	//!< LE Read Maximum Advertising Data Length command
#define BLE_HCI_CMD_CTLR_READ_NB_SUPPORTED_ADV_SETS			((8<<10) | 0x3B)	//!< LE Read Number of Supported Advertising Sets command
#define BLE_HCI_CMD_CTLR_REMOVE_ADV_SET						((8<<10) | 0x3C)	//!< LE Remove Advertising Set command
#define BLE_HCI_CMD_CTLR_CLEAR_ADV_SETS						((8<<10) | 0x3D)	//!< LE Clear Advertising Sets command
#define BLE_HCI_CMD_CTLR_SET_PERIODIC_ADV_PARAM				((8<<10) | 0x3E)	//!< LE Set Periodic Advertising Parameters command
#define BLE_HCI_CMD_CTLR_SET_PERIODIC_ADV_DATA				((8<<10) | 0x3F)	//!< LE Set Periodic Advertising Data command
#define BLE_HCI_CMD_CTLR_SET_PERIODIC_ADV_ENABLE			((8<<10) | 0x40)	//!< LE Set Periodic Advertising Enable command
#define BLE_HCI_CMD_CTLR_SET_EXT_SCAN_PARAM					((8<<10) | 0x41)	//!< LE Set Extended Scan Parameters command
#define BLE_HCI_CMD_CTLR_SET_EXT_SCAN_ENABLE				((8<<10) | 0x42)	//!< LE Set Extended Scan Enable command
#define BLE_HCI_CMD_CTLR_EXT_CREATE_CONN					((8<<10) | 0x43)	//!< LE Extended Create Connection command
#define BLE_HCI_CMD_CTLR_PERIODIC_ADV_CREATE_SYNC			((8<<10) | 0x44)	//!< LE Periodic Advertising Create Sync command
#define BLE_HCI_CMD_CTLR_PERIODIC_ADV_CREATE_SYNC_CANCEL	((8<<10) | 0x45)	//!< LE Periodic Advertising Create Sync Cancel command
#define BLE_HCI_CMD_CTLR_PERIODIC_ADV_TERMINATE_SYNC		((8<<10) | 0x46)	//!< LE Periodic Advertising Terminate Sync command
#define BLE_HCI_CMD_CTLR_PERIODIC_ADV_LIST_ADD_DEV			((8<<10) | 0x47)	//!< LE Add Device To Periodic Advertiser List command
#define BLE_HCI_CMD_CTLR_PERIODIC_ADV_LIST_REMOVE_DEV		((8<<10) | 0x48)	//!< LE Remove Device From Periodic Advertiser List command
#define BLE_HCI_CMD_CTLR_PERIODIC_ADV_LIST_CLEAR			((8<<10) | 0x49)	//!< LE Clear Periodic Advertiser List command
#define BLE_HCI_CMD_CTLR_PERIODIC_ADV_LIST_READ_SIZE		((8<<10) | 0x4A)	//!< LE Read Periodic Advertiser List Size command
#define BLE_HCI_CMD_CTLR_READ_TRANSMIT_PWR					((8<<10) | 0x4B)	//!< LE Read Transmit Power command
#define BLE_HCI_CMD_CTLR_READ_RF_PATH_COMPENSATION			((8<<10) | 0x4C)	//!< LE Read RF Path Compensation command
#define BLE_HCI_CMD_CTLR_WRITE_RF_PATH_COMPENSATION			((8<<10) | 0x4D)	//!< LE Write RF Path Compensation command
#define BLE_HCI_CMD_CTLR_SET_PRIVACY_MODE					((8<<10) | 0x4E)	//!< LE Set Privacy Mode command
#define BLE_HCI_CMD_CTLR_SET_CONNLESS_CTE_TRANSMIT_PARAM	((8<<10) | 0x51)	//!< LE Set Connectionless CTE Transmit Parameters command
#define BLE_HCI_CMD_CTLR_SET_CONNLESS_CTE_TRANSMIT_ENABLE	((8<<10) | 0x52)	//!< LE Set Connectionless CTE Transmit Enable command
#define BLE_HCI_CMD_CTLR_SET_CONNLESS_IQ_SAMPLING_ENABLE	((8<<10) | 0x53)	//!< LE Set Connectionless IQ Sampling Enable command
#define BLE_HCI_CMD_CTLR_SET_CONN_CTE_RECEIVE_PARAM			((8<<10) | 0x54)	//!< LE Set Connection CTE Receive Parameters command
#define BLE_HCI_CMD_CTLR_SET_CONN_CTE_TRANSMIT_PARAM		((8<<10) | 0x55)	//!< LE Set Connection CTE Transmit Parameters command
#define BLE_HCI_CMD_CTLR_CONN_CTE_RQST_ENABLE				((8<<10) | 0x56)	//!< LE Connection CTE Request Enable command
#define BLE_HCI_CMD_CTLR_CONN_CTE_RESPONSE_ENABLE			((8<<10) | 0x57)	//!< LE Connection CTE Response Enable command
#define BLE_HCI_CMD_CTLR_READ_ANT_INFO						((8<<10) | 0x58)	//!< LE Read Antenna Information command
#define BLE_HCI_CMD_CTLR_SET_PERIODIC_ADV_RECEIVE_ENABLE	((8<<10) | 0x59)	//!< LE Set Periodic Advertising Receive Enable command
#define BLE_HCI_CMD_CTLR_PERIODIC_ADV_SYNC_TRANSFER			((8<<10) | 0x5A)	//!< LE Periodic Advertising Sync Transfer command
#define BLE_HCI_CMD_CTLR_PERIODIC_ADV_SET_INFO_TRANSFER		((8<<10) | 0x5B)	//!< LE Periodic Advertising Set Info Transfer command
#define BLE_HCI_CMD_CTLR_SET_PERIODIC_ADV_SYNC_TRANSFER_PARAM	((8<<10) | 0x5C)	//!< LE Set Periodic Advertising Sync Transfer Parameters command
#define BLE_HCI_CMD_CTLR_SET_DEFAULT_PERIODIC_ADV_SYNC_TRANSFER_PARAM	((8<<10) | 0x5D)	//!< LE Set Default Periodic Advertising Sync Transfer Parameters command
#define BLE_HCI_CMD_CTLR_MODIFY_SLEEP_CLOCK_ACCURACY		((8<<10) | 0x5F)	//!< LE Modify Sleep Clock Accuracy command
#define BLE_HCI_CMD_CTLR_READ_ISO_TX_SYNC					((8<<10) | 0x61)	//!< LE Read ISO TX Sync command
#define BLE_HCI_CMD_CTLR_SET_CIG_PARAM						((8<<10) | 0x62)	//!< LE Set CIG Parameters command
#define BLE_HCI_CMD_CTLR_SET_CIG_PARAM_TEST					((8<<10) | 0x63)	//!< LE Set CIG Parameters Test command
#define BLE_HCI_CMD_CTLR_CREATE_CIS							((8<<10) | 0x64)	//!< LE Create CIS command
#define BLE_HCI_CMD_CTLR_REMOVE_CIG							((8<<10) | 0x65)	//!< LE Remove CIG command
#define BLE_HCI_CMD_CTLR_ACCEPT_CIS_RQST					((8<<10) | 0x66)	//!< LE Accept CIS Request command
#define BLE_HCI_CMD_CTLR_REJECT_CIS_RQST					((8<<10) | 0x67)	//!< LE Reject CIS Request command
#define BLE_HCI_CMD_CTLR_CREATE_BIG							((8<<10) | 0x68)	//!< LE Create BIG command
#define BLE_HCI_CMD_CTLR_CREATE_BIG_TEST					((8<<10) | 0x69)	//!< LE Create BIG Test command
#define BLE_HCI_CMD_CTLR_TERMINATE_BIG						((8<<10) | 0x6A)	//!< LE Terminate BIG command
#define BLE_HCI_CMD_CTLR_BIG_CREATE_SYNC					((8<<10) | 0x6B)	//!< LE BIG Create Sync command
#define BLE_HCI_CMD_CTLR_BIG_TERMINATE_SYNC					((8<<10) | 0x6C)	//!< LE BIG Terminate Sync command
#define BLE_HCI_CMD_CTLR_RQST_PEER_SCA						((8<<10) | 0x6D)	//!< LE Request Peer SCA command
#define BLE_HCI_CMD_CTLR_SETUP_ISO_DATA_PATH				((8<<10) | 0x6E)	//!< LE Setup ISO Data Path command
#define BLE_HCI_CMD_CTLR_REMOVE_ISO_DATA_PATH				((8<<10) | 0x6F)	//!< LE Remove ISO Data Path command
#define BLE_HCI_CMD_CTLR_ISO_TRANSMIT_TEST					((8<<10) | 0x70)	//!< LE ISO Transmit Test command
#define BLE_HCI_CMD_CTLR_ISO_RECEIVE_TEST					((8<<10) | 0x71)	//!< LE ISO Receive Test command
#define BLE_HCI_CMD_CTLR_ISO_READ_TEST_COUNTERS				((8<<10) | 0x72)	//!< LE ISO Read Test Counters command
#define BLE_HCI_CMD_CTLR_ISO_TEST_END						((8<<10) | 0x73)	//!< LE ISO Test End command
#define BLE_HCI_CMD_CTLR_SET_HOST_FEATURE					((8<<10) | 0x74)	//!< LE Set Host Feature command
#define BLE_HCI_CMD_CTLR_READ_ISO_LINK_QUALITY				((8<<10) | 0x75)	//!< LE Read ISO Link Quality command
#define BLE_HCI_CMD_CTLR_ENHANCED_READ_TRANSMIT_PWR_LEVEL	((8<<10) | 0x76)	//!< LE Enhanced Read Transmit Power Level command
#define BLE_HCI_CMD_CTLR_READ_REMOTE_TRANSMIT_PWR_LEVEL		((8<<10) | 0x77)	//!< LE Read Remote Transmit Power Level command
#define BLE_HCI_CMD_CTLR_SET_PATH_LOSS_REPORTING_PARAM		((8<<10) | 0x78)	//!< LE Set Path Loss Reporting Parameters command
#define BLE_HCI_CMD_CTLR_SET_PATH_LOSS_REPORTING_ENABLE		((8<<10) | 0x79)	//!< LE Set Path Loss Reporting Enable command
#define BLE_HCI_CMD_CTLR_SET_TRANSMIT_PWR_REPORTING_ENABLE	((8<<10) | 0x7A)	//!< LE Set Transmit Power Reporting Enable command
#define BLE_HCI_CMD_CTLR_SET_DATA_RELATED_ADDR_CHANGE		((8<<10) | 0x7C)	//!< LE Set Data Related Address Changes command
#define BLE_HCI_CMD_CTLR_SET_DEFAULT_SUBRATE				((8<<10) | 0x7D)	//!< LE Set Default Subrate command
#define BLE_HCI_CMD_CTLR_SUBRATE_RQST						((8<<10) | 0x7E)	//!< LE Subrate Request command



// HCI events
#define BLE_HCI_EVT_INQUERY_COMPLETE						1	//!< Indicates the Inquiry has finished
#define BLE_HCI_EVT_INQUERY_RESULT							2	//!< Indicates that Bluetooth device(s) have responded for the inquiry.
#define BLE_HCI_EVT_CONN_COMPLETE							3	//!< Indicates to both hosts that the new connection has been formed.
#define BLE_HCI_EVT_CONN_REQUEST							4	//!< Indicates that a new connection is trying to be established
#define BLE_HCI_EVT_DISCONN_COMPLETE						5	//!< Occurs when a connection has been disconnected.
#define BLE_HCI_EVT_AUTHEN_COMPLETE							6	//!< Occurs when an authentication has been completed.
#define BLE_HCI_EVT_REMOTE_NAME_REQUEST_COMPLETE			7	//!< Indicates that the request for the remote name has been completed.
#define BLE_HCI_EVT_ENCRYPTION_CHANGE						8	//!< Indicates that a change in the encryption has been completed.
#define BLE_HCI_EVT_CHANGE_CONN_LINK_KEY_COMPLETE			9	//!< Indicates that the change in the link key has been completed.
#define BLE_HCI_EVT_MASTER_LINK_KEY_COMPLETE				0xA	//!< Indicates that the change in the temporary link key or semi permanent link key on the master device is complete.
#define BLE_HCI_EVT_READ_REMOTE_SUPP_FEATURES_COMPLETE		0xB	//!< Indicates that the reading of the supported features on the remote device is complete.
#define BLE_HCI_EVT_READ_REMOTE_VERS_COMPLETE				0xC	//!< Indicates that the version number on the remote device has been read and completed.
#define BLE_HCI_EVT_QOS_SETTUP_COMPLETE						0xD	//!< Indicates that the Quality of Service setup has been complete.
#define BLE_HCI_EVT_COMMAND_COMPLETE						0xE	//!< Used by controller to send status and event parameters to the host for the particular command.
#define BLE_HCI_EVT_COMMAND_STATUS							0xF	//!< Indicates that the command has been received and is being processed in the host controller.
#define BLE_HCI_EVT_HARDWARE_ERROR							0x10	//!< Indicates a hardware failure of the Bluetooth device.
#define BLE_HCI_EVT_FLUSH_OCCURED							0x11	//!< Indicates that the data has been flushed for a particular connection.
#define BLE_HCI_EVT_ROLE_CHANGED							0x12	//!< Indicates that the current bluetooth role for a connection has been changed.
#define BLE_HCI_EVT_NB_COMPLETED_PACKET						0x13	//!< Indicates to the host the number of data packets sent compared to the last time the same event was sent.
#define BLE_HCI_EVT_MODE_CHANGED							0x14	//!< Indicates the change in mode from hold, sniff, park or active to another mode.
#define BLE_HCI_EVT_RETURN_LINK_KEYS						0x15	//!< Used to return stored link keys after a Read_Stored_Link_Key command was issued.
#define BLE_HCI_EVT_PIN_CODE_REQUEST						0x16	//!< Indicates the a PIN code is required for a new connection.
#define BLE_HCI_EVT_LINK_KEY_REQUEST						0x17	//!< Indicates that a link key is required for the connection.
#define BLE_HCI_EVT_LINK_KEY_NOTIFICATION					0x18	//!< Indicates to the host that a new link key has been created.
#define BLE_HCI_EVT_LOOPBACK_COMMAND						0x19	//!< Indicates that command sent from the host will be looped back.
#define BLE_HCI_EVT_DATA_BUFFER_OVERFLOW					0x1A	//!< Indicates that the data buffers on the host has overflowed.
#define BLE_HCI_EVT_MAX_SLOT_CHANGED						0x1B	//!< Informs the host when the LMP_Max_Slots parameter changes.
#define BLE_HCI_EVT_READ_CLOCK_OFFSET_COMPLETE				0x1C	//!< Indicates the completion of reading the clock offset information.
#define BLE_HCI_EVT_CONN_PACKET_TYPE_CHANGED				0x1D	//!< Indicate the completion of the packet type change for a connection.
#define BLE_HCI_EVT_QOS_VIOLATION							0x1E	//!< Indicates that the link manager is unable to provide the required Quality of Service.
#define BLE_HCI_EVT_PAGE_SCAN_MODE_CHANGED					0x1F	//!< Indicates that the remote device has successfully changed the Page Scan mode.
#define BLE_HCI_EVT_PAGE_SCAN_REPETITION_MODE_CHANGED		0x20	//!< Indicates that the remote device has successfully changed the Page Scan Repetition mode.

// HCI error codes
#define BLE_HCI_ERR_UNKNOWN_COMMAND							1
#define BLE_HCI_ERR_NO_CONNECTION							2
#define BLE_HCI_ERR_HARDWARE_FAILURE						3
#define BLE_HCI_ERR_PAGE_TIMEOUT							4
#define BLE_HCI_ERR_AUTHEN_FAILURE							5
#define BLE_HCI_ERR_KEY_MISSING								6
#define BLE_HCI_ERR_MEMORY_FULL								7
#define BLE_HCI_ERR_CONN_TIMEOUT							8
#define BLE_HCI_ERR_MAX_NB_CONN								9
#define BLE_HCI_ERR_MAX_NB_SCO_CONN							0xA
#define BLE_HCI_ERR_ACL_CONN_EXISTS							0xB
#define BLE_HCI_ERR_COMMAND_DISALLOWED						0xC
#define BLE_HCI_ERR_HOST_REJECT_RESOURCE_LIMIT				0xD
#define BLE_HCI_ERR_HOST_REJECT_SECURITY					0xE
#define BLE_HCI_ERR_HOST_REJECT_PERSONAL_DEVICE				0xF
#define BLE_HCI_ERR_HOST_TIMEOUT							0x10
#define BLE_HCI_ERR_UNSUPPORTED_FEATURE_PARAM				0x11
#define BLE_HCI_ERR_INVALID_COMMAND_PARAM					0x12
#define BLE_HCI_ERR_CONN_TERMINATED_USER					0x13
#define BLE_HCI_ERR_CONN_TERMINATED_LOW_RESOURCE			0x14
#define BLE_HCI_ERR_CONN_TERMINATED_POWER_OFF				0x15
#define BLE_HCI_ERR_CONN_TERMINATED_LOCAL_HOST				0x16
#define BLE_HCI_ERR_REPEATED_ATTEMPTS						0x17
#define BLE_HCI_ERR_PAIRING_NOT_ALLOWED						0x18
#define BLE_HCI_ERR_UNKNOWN_LMP_PDU							0x19
#define BLE_HCI_ERR_UNSUPPORTED_REMOTE_FEATURE				0x1A
#define BLE_HCI_ERR_SCO_OFFSET_REJECTED						0x1B
#define BLE_HCI_ERR_SCO_INTERVAL_REJECTED					0x1C
#define BLE_HCI_ERR_SCO_AIR_MODE_REJECTED					0x1D
#define BLE_HCI_ERR_INVALID_LMP_PARAM						0x1E
#define BLE_HCI_ERR_UNSPECIFIED								0x1F
#define BLE_HCI_ERR_UNSUPPORTED_LMP_PARAM					0x20
#define BLE_HCI_ERR_ROLE_CHANGE_NOT_ALLOWED					0x21
#define BLE_HCI_ERR_LMP_RESPONSE_TIMEOUT					0x22
#define BLE_HCI_ERR_LMP_TRANSACTION_COLLISION				0x23
#define BLE_HCI_ERR_LMP_PDU_NOT_ALLOWED						0x24
#define BLE_HCI_ERR_ENCRYPTION_MODE_NOT_ACCEPTABLE			0x25
#define BLE_HCI_ERR_UNIT_KEY_USED							0x26
#define BLE_HCI_ERR_QOS_NOT_SUPPORTED						0x27
#define BLE_HCI_ERR_INSTANT_PASSED							0x28
#define BLE_HCI_ERR_PAIRING_WITH_UNIT_KEY_NOT_SUPPORTED		0x29



#pragma pack(push, 1)

/// HCI Command packet header
typedef struct __Ble_Hci_Cmd_Packet_Header {
	union {
		uint16_t OpCode;		//!< HCI OpCode
		struct {
			uint16_t Ocf:10;
			uint16_t Ogf:6;
		};
	};
	uint8_t ParamLen;			//!< HCI parameter length in bytes
	//uint8_t ParamStart;			//!< HCI parameter
} BleHciCmdPacketHdr_t;

/// HCI Command packet
/// NOTE: This structure is variable length
typedef struct __Ble_Hci_Cmd_Packet {
	BleHciCmdPacketHdr_t Hdr;	//!< Command packet header
	uint8_t Param[1];
} BleHciCmd_Packet_t;

#define BLE_STDHCI_PBFLAG_START_NONFLUSHABLE		0
#define BLE_STDHCI_PBFLAG_CONTINUING_FRAGMENT		1
#define BLE_STDHCI_PBFLAG_START_FLUSHABLE			2
#define BLE_STDHCI_PBFLAG_COMPLETE_L2CAP_PDU		3

#define BLE_STDHCI_BCFLAG_POINT_TO_POINT			0
#define BLE_STDHCI_BCFLAG_BR_EDR_BROADCAST			1

/// HCI ACL data packet header
typedef struct __Ble_Hci_Data_Packet_Header {
	uint32_t Handle:12;			//!< Connection Handle
	uint32_t PBFlag:2;			//!< Packet boundary flag
	uint32_t BCFlag:2;			//!< Broadcast flag
	uint32_t Len:16;			//!< Data length in bytes
} BleHciDataPacketHdr_t;

/// HCI ACL data packet
/// NOTE: This structure is variable length
typedef struct __Ble_Hci_Data_Packet {
	BleHciDataPacketHdr_t Hdr;	//!< Data packet header
	uint8_t Data[1];
} BleHciDataPacke_t;


#pragma pack(pop)

#ifdef __cplusplus
extern "C" {
#endif


#ifdef __cplusplus
}
#endif

/** @} */

#endif // __BLE_HCIDEF_H__
