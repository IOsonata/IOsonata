/**-------------------------------------------------------------------------
@file	bt_hci.h

@brief	Bluetooth standard HCI

Generic implementation & definitions of Bluetooth HCI (Host Controller Interface)

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
#ifndef __BT_HCI_H__
#define __BT_HCI_H__

#include <stdint.h>

#include "bluetooth/bt_dev.h"

/** @addtogroup Bluetooth
 * @{ */

// Link Control Commands
// OGF = 1
#define BT_HCI_CMD_LINKCTRL			1

#define BT_HCI_CMD_LINKCTRL_INQUIRY							((1<<10) | 1)		//!< Inquiry command
#define BT_HCI_CMD_LINKCTRL_INQUIRY_CANCEL					((1<<10) | 2)		//!< Inquiry Cancel command
#define BT_HCI_CMD_LINKCTRL_PERIODIC_INQUIRY_MODE			((1<<10) | 3)		//!< Periodic Inquiry Mode command
#define BT_HCI_CMD_LINKCTRL_PERIODIC_INQUIRY_MODE_EXIT		((1<<10) | 4)		//!< Exit Periodic Inquiry Mode command
#define BT_HCI_CMD_LINKCTRL_CREATE_CONN						((1<<10) | 5)		//!< Create Connection command
#define BT_HCI_CMD_LINKCTRL_DISCONNECT						((1<<10) | 6)		//!< Disconnect command
#define BT_HCI_CMD_LINKCTRL_CREATE_CONN_CANCEL				((1<<10) | 8)		//!< Create Connection Cancel command
#define BT_HCI_CMD_LINKCTRL_ACCEPT_CONN_RQST				((1<<10) | 9)		//!< Accept Connection Request command
#define BT_HCI_CMD_LINKCTRL_REJECT_CONN_RQST				((1<<10) | 0xA)		//!< Reject Connection Request command
#define BT_HCI_CMD_LINKCTRL_LINK_KEY_RQST_REPLY				((1<<10) | 0xB)		//!< Link Key Request Reply command
#define BT_HCI_CMD_LINKCTRL_LINK_KEY_RQST_NEG_REPLY			((1<<10) | 0xC)		//!< Link Key Request Negative Reply command
#define BT_HCI_CMD_LINKCTRL_PINCODE_RQST_REPLY				((1<<10) | 0xD)		//!< PIN Code Request Reply command
#define BT_HCI_CMD_LINKCTRL_PINCODE_RQST_NEG_REPLY			((1<<10) | 0xE)		//!< PIN Code Request Negative Reply command
#define BT_HCI_CMD_LINKCTRL_CHANGE_CONN_PACKET_TYPE			((1<<10) | 0xF)		//!< Change Connection Packet Type command
#define BT_HCI_CMD_LINKCTRL_AUTHEN_RQST						((1<<10) | 0x11)	//!< Authentication Requested command
#define BT_HCI_CMD_LINKCTRL_SET_CONN_ENCRYPTION				((1<<10) | 0x13)	//!< Set Connection Encryption command
#define BT_HCI_CMD_LINKCTRL_CHANGE_CONN_LINK_KEY			((1<<10) | 0x15)	//!< Change Connection Link Key command
#define BT_HCI_CMD_LINKCTRL_LINK_KEY_SELECTION				((1<<10) | 0x17)	//!< Link Key Selection command
#define BT_HCI_CMD_LINKCTRL_REMOTE_NAME_RQST				((1<<10) | 0x19)	//!< Remote Name Request command
#define BT_HCI_CMD_LINKCTRL_REMOTE_NAME_RQST_CANCEL			((1<<10) | 0x1A)	//!< Remote Name Request Cancel command
#define BT_HCI_CMD_LINKCTRL_READ_REMOTE_SUPPORTED_FEATURES	((1<<10) | 0x1B)	//!< Read Remote Supported Features command
#define BT_HCI_CMD_LINKCTRL_READ_REMOTE_EXT_FEATURES		((1<<10) | 0x1C)	//!< Read Remote Extended Features command
#define BT_HCI_CMD_LINKCTRL_READ_REMOTE_VERS_INFO			((1<<10) | 0x1D)	//!< Read Remote Version Information command
#define BT_HCI_CMD_LINKCTRL_READ_CLOCK_OFFSET				((1<<10) | 0x1F)	//!< Read Clock Offset command
#define BT_HCI_CMD_LINKCTRL_READ_LMP_HANDLE					((1<<10) | 0x20)	//!< Read LMP Handle command
#define BT_HCI_CMD_LINKCTRL_SETUP_SYNCHRONOUS_CONN			((1<<10) | 0x28)	//!< Setup Synchronous Connection command
#define BT_HCI_CMD_LINKCTRL_ACCEPT_SYNCHRONOUS_CONN_RQST	((1<<10) | 0x29)	//!< Accept Synchronous Connection Request command
#define BT_HCI_CMD_LINKCTRL_REJECT_SYNCHRONOUS_CONN_RQST	((1<<10) | 0x2A)	//!< Reject Synchronous Connection Request command
#define BT_HCI_CMD_LINKCTRL_IO_CAPABILITY_RQST_REPLY		((1<<10) | 0x2B)	//!< IO Capability Request Reply command
#define BT_HCI_CMD_LINKCTRL_USER_INFO_RQST_REPLY			((1<<10) | 0x2C)	//!< User Confirmation Request Reply command
#define BT_HCI_CMD_LINKCTRL_USER_INFO_RQST_NEG_REPLY		((1<<10) | 0x2D)	//!< User Confirmation Request Negative Reply command
#define BT_HCI_CMD_LINKCTRL_USER_PASSKEY_RQST_REPLY			((1<<10) | 0x2E)	//!< User Passkey Request Reply command
#define BT_HCI_CMD_LINKCTRL_USER_PASSKEY_RQST_NEG_REPLY		((1<<10) | 0x2F)	//!< User Passkey Request Negative Reply command
#define BT_HCI_CMD_LINKCTRL_REMOTE_OOB_DATA_RQST_REPLY		((1<<10) | 0x30)	//!< Remote OOB Data Request Reply command
#define BT_HCI_CMD_LINKCTRL_REMOTE_OOB_DATA_RQST_NEG_REPLY	((1<<10) | 0x33)	//!< Remote OOB Data Request Negative Reply command
#define BT_HCI_CMD_LINKCTRL_IO_CAPABILITY_RQST_NEG_REPLY	((1<<10) | 0x34)	//!< IO Capability Request Negative Reply command
#define BT_HCI_CMD_LINKCTRL_ENHANCED_SETUP_SYNCHRONOUS_CONN	((1<<10) | 0x3D)	//!< Enhanced Setup Synchronous Connection command
#define BT_HCI_CMD_LINKCTRL_ENHANCED_ACCEPT_SYNCHRONOUS_CONN_RQST	((1<<10) | 0x3E)	//!< Enhanced Accept Synchronous Connection Request command
#define BT_HCI_CMD_LINKCTRL_TRUNCATED_PAGE					((1<<10) | 0x3F)	//!< Truncated Page command
#define BT_HCI_CMD_LINKCTRL_TRUNCATED_PAGE_CANCEL			((1<<10) | 0x40)	//!< Truncated Page Cancel command
#define BT_HCI_CMD_LINKCTRL_SET_CONNLESS_PERIPH_BROADCAST	((1<<10) | 0x41)	//!< Set Connectionless Peripheral Broadcast command
#define BT_HCI_CMD_LINKCTRL_SET_CONNLESS_PERIPH_BROADCAST_RECEIVE	((1<<10) | 0x42)	//!< Set Connectionless Peripheral Broadcast Receive command
#define BT_HCI_CMD_LINKCTRL_START_SYNC_TRAIN				((1<<10) | 0x43)	//!< Start Synchronization Train command
#define BT_HCI_CMD_LINKCTRL_RECEIVE_SYNC_TRAIN				((1<<10) | 0x44)	//!< Receive Synchronization Train command
#define BT_HCI_CMD_LINKCTRL_REMOTE_OOB_EXT_DATA_RQST_REPLY	((1<<10) | 0x45)	//!< Remote OOB Extended Data Request Reply command


// HCI Policy Command
// OGF = 2
#define BT_HCI_CMD_POLICY			2

#define BT_HCI_CMD_POLICY_HOLD_MODE							((2<<10) | 1)		//!< Hold Mode command
#define BT_HCI_CMD_POLICY_SNIFF_MODE						((2<<10) | 3)		//!< Sniff Mode command
#define BT_HCI_CMD_POLICY_EXIT_SNIFF_MODE					((2<<10) | 4)		//!< Exit Sniff Mode command
#define BT_HCI_CMD_POLICY_QOS_SETUP							((2<<10) | 7)		//!< QoS Setup command
#define BT_HCI_CMD_POLICY_ROLE_DISCOVERY					((2<<10) | 9)		//!< Role Discovery command
#define BT_HCI_CMD_POLICY_SWITCH_ROLE						((2<<10) | 0xB)		//!< Switch Role command
#define BT_HCI_CMD_POLICY_READ_LINK_POLICY_SETTINGS			((2<<10) | 0xC)		//!< Read Link Policy Settings command
#define BT_HCI_CMD_POLICY_WRITE_LINK_POLICY_SETTINGS		((2<<10) | 0xD)		//!< Write Link Policy Settings command
#define BT_HCI_CMD_POLICY_READ_DEFAULT_LINK_POLICY_SETTINGS	((2<<10) | 0xE)		//!< Read Default Link Policy Settings command
#define BT_HCI_CMD_POLICY_WRITE_DEFAULT_LINK_POLICY_SETTINGS	((2<<10) | 0xF)		//!< Write Default Link Policy Settings command
#define BT_HCI_CMD_POLICY_FLOW_SPECIFICATION				((2<<10) | 0x10)	//!< Flow Specification command
#define BT_HCI_CMD_POLICY_SNIFF_SUBRATING					((2<<10) | 0x11)	//!< Sniff Subrating command


// Host Controller and Baseband Command
// OGF = 3
#define BT_HCI_CMD_BASEBAND			3

#define BT_HCI_CMD_BASEBAND_SET_EVENT_MASK					((3<<10) | 1)		//!< Set Event Mask command
#define BT_HCI_CMD_BASEBAND_RESET							((3<<10) | 3)		//!< Reset command
#define BT_HCI_CMD_BASEBAND_SET_EVENT_FILTER				((3<<10) | 5)		//!< Set Event Filter command
#define BT_HCI_CMD_BASEBAND_FLUSH							((3<<10) | 8)		//!< Flush command
#define BT_HCI_CMD_BASEBAND_READ_PIN_TYPE					((3<<10) | 9)		//!< Read PIN Type command
#define BT_HCI_CMD_BASEBAND_WRITE_PIN_TYPE					((3<<10) | 0xA)		//!< Write PIN Type command
#define BT_HCI_CMD_BASEBAND_READ_STORED_LINK_KEY			((3<<10) | 0xD)		//!< Read Stored Link Key command
#define BT_HCI_CMD_BASEBAND_WRITE_STORED_LINK_KEY			((3<<10) | 0x11)	//!< Write Stored Link Key command
#define BT_HCI_CMD_BASEBAND_DELETE_STORED_LINK_KEY			((3<<10) | 0x12)	//!< Delete Stored Link Key command
#define BT_HCI_CMD_BASEBAND_WRITE_LOCAL_NAME				((3<<10) | 0x13)	//!< Write Local Name command
#define BT_HCI_CMD_BASEBAND_READ_LOCAL_NAME					((3<<10) | 0x14)	//!< Read Local Name command
#define BT_HCI_CMD_BASEBAND_READ_CONN_ACCEPT_TIMEOUT		((3<<10) | 0x15)	//!< Read Connection Accept Timeout command
#define BT_HCI_CMD_BASEBAND_WRITE_CONN_ACCEPT_TIMEOPUT		((3<<10) | 0x16)	//!< Write Connection Accept Timeout command
#define BT_HCI_CMD_BASEBAND_READ_PAGE_TIMEOUT				((3<<10) | 0x17)	//!< Read Page Timeout command
#define BT_HCI_CMD_BASEBAND_WRITE_PAGE_TIMEOUT				((3<<10) | 0x18)	//!< Write Page Timeout command
#define BT_HCI_CMD_BASEBAND_READ_SCAN_ENABLE				((3<<10) | 0x19)	//!< Read Scan Enable command
#define BT_HCI_CMD_BASEBAND_WRITE_SCAN_ENABLE				((3<<10) | 0x1A)	//!< Write Scan Enable command
#define BT_HCI_CMD_BASEBAND_READ_PAGE_SCAN_ACTIVITY			((3<<10) | 0x1B)	//!< Read Page Scan Activity command
#define BT_HCI_CMD_BASEBAND_WRITE_PAGE_SCAN_ACTIVITY		((3<<10) | 0x1C)	//!< Write Page Scan Activity command
#define BT_HCI_CMD_BASEBAND_READ_INQUIRY_SCAN_ACTIVITY		((3<<10) | 0x1D)	//!< Read Inquiry Scan Activity command
#define BT_HCI_CMD_BASEBAND_WRITE_INQUIRY_SCAN_ACTIVITY		((3<<10) | 0x1E)	//!< Write Inquiry Scan Activity command
#define BT_HCI_CMD_BASEBAND_READ_AUTHEN_ENABLE				((3<<10) | 0x1F)	//!< Read Authentication Enable command
#define BT_HCI_CMD_BASEBAND_WRITE_AUTHEN_ENABLE				((3<<10) | 0x20)	//!< Write Authentication Enable command
#define BT_HCI_CMD_BASEBAND_READ_CLASS_OF_DEVICE			((3<<10) | 0x23)	//!< Read Class of Device command
#define BT_HCI_CMD_BASEBAND_WRITE_CLASS_OF_DEVICE			((3<<10) | 0x24)	//!< Write Class of Device command
#define BT_HCI_CMD_BASEBAND_READ_VOICE_SETTING				((3<<10) | 0x25)	//!< Read Voice Setting command
#define BT_HCI_CMD_BASEBAND_WRITE_VOICE_SETTING				((3<<10) | 0x26)	//!< Write Voice Setting command
#define BT_HCI_CMD_BASEBAND_READ_AUTO_FLUSH_TIMEOUT			((3<<10) | 0x27)	//!< Read Automatic Flush Timeout command
#define BT_HCI_CMD_BASEBAND_WRITE_AUTO_FLUSH_TIMEOUT		((3<<10) | 0x28)	//!< Write Automatic Flush Timeout command
#define BT_HCI_CMD_BASEBAND_READ_NB_BROADCAST_RETRANS		((3<<10) | 0x29)	//!< Read Num Broadcast Retransmissions command
#define BT_HCI_CMD_BASEBAND_WRITE_NB_BROADCAST_RETRANS		((3<<10) | 0x2A)	//!< Write Num Broadcast Retransmissions command
#define BT_HCI_CMD_BASEBAND_READ_HOLD_MODE_ACTIVITY			((3<<10) | 0x2B)	//!< Read Hold Mode Activity command
#define BT_HCI_CMD_BASEBAND_WRITE_HOLD_MODE_ACTIVITY		((3<<10) | 0x2C)	//!< Write Hold Mode Activity command
#define BT_HCI_CMD_BASEBAND_READ_TRANSMIT_POWER_LEVEL		((3<<10) | 0x2D)	//!< Read Transmit Power Level command
#define BT_HCI_CMD_BASEBAND_READ_SYNCHRONOUS_FLOWCTRL_ENABLE	((3<<10) | 0x2E)	//!< Read Synchronous Flow Control Enable command
#define BT_HCI_CMD_BASEBAND_WRITE_SYNCHRONOUS_FLOWCTRL_ENABLE	((3<<10) | 0x2F)	//!< Write Synchronous Flow Control Enable command
#define BT_HCI_CMD_BASEBAND_SET_FLOWCTRL					((3<<10) | 0x31)	//!< Set Controller To Host Flow Control command
#define BT_HCI_CMD_BASEBAND_HOST_BUFFER_SIZE				((3<<10) | 0x33)	//!< Host Buffer Size command
#define BT_HCI_CMD_BASEBAND_HOST_NB_COMPLETE_PACKETS		((3<<10) | 0x35)	//!< Host Number Of Completed Packets command
#define BT_HCI_CMD_BASEBAND_READ_LINK_SUPERV_TIMEOUT		((3<<10) | 0x36)	//!< Read Link Supervision Timeout command
#define BT_HCI_CMD_BASEBAND_WRITE_LINK_SUPERV_TIMEOUT		((3<<10) | 0x37)	//!< Write Link Supervision Timeout command
#define BT_HCI_CMD_BASEBAND_READ_NB_SUPPORTED_IAC			((3<<10) | 0x38)	//!< Read Number Of Supported IAC command
#define BT_HCI_CMD_BASEBAND_READ_CURRENT_IAC_LAP			((3<<10) | 0x39)	//!< Read Current IAC LAP command
#define BT_HCI_CMD_BASEBAND_WRITE_CURRENT_IAC_LAP			((3<<10) | 0x3A)	//!< Write Current IAC LAP command
#define BT_HCI_CMD_BASEBAND_SET_AFH_CHAN_CLASS				((3<<10) | 0x3F)	//!< Set AFH Host Channel Classification command
#define BT_HCI_CMD_BASEBAND_READ_INQUIRY_SCAN_TYPE			((3<<10) | 0x42)	//!< Read Inquiry Scan Type command
#define BT_HCI_CMD_BASEBAND_WRITE_INQUIRY_SCAN_TYPE			((3<<10) | 0x43)	//!< Write Inquiry Scan Type command
#define BT_HCI_CMD_BASEBAND_READ_INQUIRY_MODE				((3<<10) | 0x44)	//!< Read Inquiry Mode command
#define BT_HCI_CMD_BASEBAND_WRITE_INQUIRY_MODE				((3<<10) | 0x45)	//!< Write Inquiry Mode command
#define BT_HCI_CMD_BASEBAND_READ_PAGE_SCAN_TYPE				((3<<10) | 0x46)	//!< Read Page Scan Type command
#define BT_HCI_CMD_BASEBAND_WRITE_PAGE_SCAN_TYPE			((3<<10) | 0x47)	//!< Write Page Scan Type command
#define BT_HCI_CMD_BASEBAND_READ_AFH_CHAN_ASSESS_MODE		((3<<10) | 0x48)	//!< Read AFH Channel Assessment Mode command
#define BT_HCI_CMD_BASEBAND_WRITE_AFH_CHAN_ASSESS_MODE		((3<<10) | 0x49)	//!< Write AFH Channel Assessment Mode command
#define BT_HCI_CMD_BASEBAND_READ_EXT_INQUIRY_RESPONSE		((3<<10) | 0x51)	//!< Read Extended Inquiry Response command
#define BT_HCI_CMD_BASEBAND_WRITE_EXT_INQUIRY_RESPONSE		((3<<10) | 0x52)	//!< Write Extended Inquiry Response command
#define BT_HCI_CMD_BASEBAND_REFRESH_ENCRYPTION_KEY			((3<<10) | 0x53)	//!< Refresh Encryption Key command
#define BT_HCI_CMD_BASEBAND_READ_SIMPLE_PAIRING_MODE		((3<<10) | 0x55)	//!< Read Simple Pairing Mode command
#define BT_HCI_CMD_BASEBAND_WRITE_SIMPLE_PAIRING_MODE		((3<<10) | 0x56)	//!< Write Simple Pairing Mode command
#define BT_HCI_CMD_BASEBAND_READ_LOCAL_OOB_DATA				((3<<10) | 0x57)	//!< Read Local OOB Data command
#define BT_HCI_CMD_BASEBAND_READ_INQUIRY_RESP_TRANSMIT_PWR_LEVEL	((3<<10) | 0x58)	//!< Read Inquiry Response Transmit Power Level command
#define BT_HCI_CMD_BASEBAND_WRITE_INQUIRY_TRANSMIT_PWR_LEVEL		((3<<10) | 0x59)	//!< Write Inquiry Transmit Power Level command
#define BT_HCI_CMD_BASEBAND_READ_DEFAULT_ERR_DATA_REPORT	((3<<10) | 0x5A)	//!< Read Default Erroneous Data Reporting command
#define BT_HCI_CMD_BASEBAND_WRITE_DEFAULT_ERR_DATA_REPORT	((3<<10) | 0x5B)	//!< Write Default Erroneous Data Reporting command
#define BT_HCI_CMD_BASEBAND_ENHANCED_FLUSH					((3<<10) | 0x5F)	//!< Enhanced Flush command
#define BT_HCI_CMD_BASEBAND_SEND_KEYPRESS_NOTIF				((3<<10) | 0x60)	//!< Send Keypress Notification command
#define BT_HCI_CMD_BASEBAND_SET_EVT_MASK_PAGE_2				((3<<10) | 0x63)	//!< Set Event Mask Page 2 command
#define BT_HCI_CMD_BASEBAND_READ_FLOWCTRL_MODE				((3<<10) | 0x66)	//!< Read Flow Control Mode command
#define BT_HCI_CMD_BASEBAND_WRITE_FLOWCTRL_MODE				((3<<10) | 0x67)	//!< Write Flow Control Mode command
#define BT_HCI_CMD_BASEBAND_READ_ENHANCED_TRANSMIT_PWR_LEVEL	((3<<10) | 0x68)	//!< Read Enhanced Transmit Power Level command
#define BT_HCI_CMD_BASEBAND_READ_LE_HOST_SUPPORT			((3<<10) | 0x6C)	//!< Read LE Host Support command
#define BT_HCI_CMD_BASEBAND_WRITE_LE_HOST_SUPPORT			((3<<10) | 0x6D)	//!< Write LE Host Support command
#define BT_HCI_CMD_BASEBAND_SET_MWS_CHAN_PARAM				((3<<10) | 0x6E)	//!< Set MWS Channel Parameters command
#define BT_HCI_CMD_BASEBAND_SET_EXTERNAL_FRAME_CONF			((3<<10) | 0x6F)	//!< Set External Frame Configuration command
#define BT_HCI_CMD_BASEBAND_SET_MWS_SIGNALING				((3<<10) | 0x70)	//!< Set MWS Signaling command
#define BT_HCI_CMD_BASEBAND_SET_MWS_TRANSPORT_LAYER			((3<<10) | 0x71)	//!< Set MWS Transport Layer command
#define BT_HCI_CMD_BASEBAND_SET_MWS_SCAN_FrEQ_TABLE			((3<<10) | 0x72)	//!< Set MWS Scan Frequency Table command
#define BT_HCI_CMD_BASEBAND_SET_MWS_PATTERN_CONFIG			((3<<10) | 0x73)	//!< Set MWS_PATTERN Configuration command
#define BT_HCI_CMD_BASEBAND_SET_RESERVED_LT_ADDR			((3<<10) | 0x74)	//!< Set Reserved LT_ADDR command
#define BT_HCI_CMD_BASEBAND_DELETE_RESERVED_LT_ADDR			((3<<10) | 0x75)	//!< Delete Reserved LT_ADDR command
#define BT_HCI_CMD_BASEBAND_SET_CONNLESS_PERIPH_BROADCAST_DATA	((3<<10) | 0x76)	//!< Set Connectionless Peripheral Broadcast Data command
#define BT_HCI_CMD_BASEBAND_READ_SYNC_TRAIN_PARAM			((3<<10) | 0x77)	//!< Read Synchronization Train Parameters command
#define BT_HCI_CMD_BASEBAND_WRITE_SYNC_TRAIN_PARAM			((3<<10) | 0x78)	//!< Write Synchronization Train Parameters command
#define BT_HCI_CMD_BASEBAND_READ_SECURE_CONN_HOST_SUPPORT	((3<<10) | 0x79)	//!< Read Secure Connections Host Support command
#define BT_HCI_CMD_BASEBAND_WRITE_SECURE_CONN_HOST_SUPPORT	((3<<10) | 0x7A)	//!< Write Secure Connections Host Support command
#define BT_HCI_CMD_BASEBAND_READ_AUTH_PAYLOAD_TIMEOUT		((3<<10) | 0x7B)	//!< Read Authenticated Payload Timeout command
#define BT_HCI_CMD_BASEBAND_WRITE_AUTH_PAYLOAD_TIMEOUT		((3<<10) | 0x7C)	//!< Write Authenticated Payload Timeout command
#define BT_HCI_CMD_BASEBAND_READ_LOCAL_OOB_EXT_DATA			((3<<10) | 0x7D)	//!< Read Local OOB Extended Data command
#define BT_HCI_CMD_BASEBAND_READ_EXT_PAGE_TIMEOUT			((3<<10) | 0x7E)	//!< Read Extended Page Timeout command
#define BT_HCI_CMD_BASEBAND_WRITE_EXT_PAGE_TIMEOUT			((3<<10) | 0x7F)	//!< Write Extended Page Timeout command
#define BT_HCI_CMD_BASEBAND_READ_EXT_INQUIRY_LEN			((3<<10) | 0x80)	//!< Read Extended Inquiry Length command
#define BT_HCI_CMD_BASEBAND_WRITE_EXT_INQUIRY_LEN			((3<<10) | 0x81)	//!< Write Extended Inquiry Length command
#define BT_HCI_CMD_BASEBAND_SET_ECOSYSTEM_BAS_INTERVAL		((3<<10) | 0x82)	//!< Set Ecosystem Base Interval command
#define BT_HCI_CMD_BASEBAND_CONFIG_DATA_PATH				((3<<10) | 0x83)	//!< Configure Data Path command
#define BT_HCI_CMD_BASEBAND_SET_MIN_ENCRYPTION_KEY_SIZE		((3<<10) | 0x84)	//!< Set Min Encryption Key Size command


// INFORMATIONAL PARAMETERS
// OGF = 4
#define BT_HCI_CMD_INFO				4

#define BT_HCI_CMD_INFO_READ_LOCAL_VERS_INFO				((4<<10) | 1)		//!< Read Local Version Information command
#define BT_HCI_CMD_INFO_READ_LOCAL_SUPPORTED_COMMANDS		((4<<10) | 2)		//!< Read Local Supported Commands command
#define BT_HCI_CMD_INFO_READ_LOCAL_SUPPORTED_FEATURES		((4<<10) | 3)		//!< Read Local Supported Features command
#define BT_HCI_CMD_INFO_READ_LOCAL_EXT_FEATURES				((4<<10) | 4)		//!< Read Local Extended Features command
#define BT_HCI_CMD_INFO_READ_BUFFER_SIZE					((4<<10) | 5)		//!< Read Buffer Size command
#define BT_HCI_CMD_INFO_READ_BD_ADDR						((4<<10) | 9)		//!< Read BD_ADDR command
#define BT_HCI_CMD_INFO_READ_DATA_BLOCK_SIZE				((4<<10) | 0xA)		//!< Read Data Block Size command
#define BT_HCI_CMD_INFO_READ_LOCAL_SUPPORTED_CODeCS_V1		((4<<10) | 0xB)		//!< Read Local Supported Codecs command V1
#define BT_HCI_CMD_INFO_READ_LOCAL_SUPPORTED_CODeCS_V2		((4<<10) | 0xD)		//!< Read Local Supported Codecs command V2
#define BT_HCI_CMD_INFO_READ_LOCAL_SIMPLE_PAIRING_OPTIONS	((4<<10) | 0xC)		//!< Read Local Simple Pairing Options command
#define BT_HCI_CMD_INFO_READ_LOCAL_SUPPORTED_CODEC_CAPABILITIES		((4<<10) | 0xE)	//!< Read Local Supported Codec Capabilities command
#define BT_HCI_CMD_INFO_READ_LOCAL_SUPPORTED_CTLR_DELAY		((4<<10) | 0xF)		//!< Read Local Supported Controller Delay command

// STATUS PARAMETERS
// OGF = 5
#define BT_HCI_CMD_STATUS				5

#define BT_HCI_CMD_STATUS_READ_FAILED_CONTACT_COUNTER		((5<<10) | 1)		//!< Read Failed Contact Counter command
#define BT_HCI_CMD_STATUS_RESET_FAILED_CONTACT_COUNTER		((5<<10) | 2)		//!< Reset Failed Contact Counter command
#define BT_HCI_CMD_STATUS_READ_LINK_QUALITY					((5<<10) | 3)		//!< Read Link Quality command
#define BT_HCI_CMD_STATUS_READ_RSSI							((5<<10) | 5)		//!< Read RSSI command
#define BT_HCI_CMD_STATUS_READ_AFH_CHAN_MAP					((5<<10) | 6)		//!< Read AFH Channel Map command
#define BT_HCI_CMD_STATUS_READ_CLOCK						((5<<10) | 7)		//!< Read Clock command
#define BT_HCI_CMD_STATUS_READ_ENCRYPTION_KEY_SIZE			((5<<10) | 8)		//!< Read Encryption Key Size command
#define BT_HCI_CMD_STATUS_GET_MWS_TRANSPORT_LAYER_CONFIG	((5<<10) | 0xC)		//!< Get MWS Transport Layer Configuration command
#define BT_HCI_CMD_STATUS_SET_TRIGGERED_CLOCK_CAPTURE		((5<<10) | 0xD)		//!< Set Triggered Clock Capture command


// TESTING COMMANDS
// OGF = 6
#define BT_HCI_CMD_TEST				6

#define BT_HCI_CMD_TEST_READ_LOOPBACK_MODE					((6<<10) | 1)		//!< Read Loopback Mode command
#define BT_HCI_CMD_TEST_WRITE_LOOPBACK_MODE					((6<<10) | 2)		//!< Write Loopback Mode command
#define BT_HCI_CMD_TEST_ENABLE_DUT_MODE						((6<<10) | 3)		//!< Enable Device Under Test Mode command
#define BT_HCI_CMD_TEST_WRITE_SIMPLE_PAIRING_DEBUG_MODE		((6<<10) | 4)		//!< Write Simple Pairing Debug Mode command
#define BT_HCI_CMD_TEST_WRITE_SECURE_CONN_TEST_MODE			((6<<10) | 0xA)		//!< Write Secure Connections Test Mode command

// LE Controller Commands
// OGF = 8

#define BT_HCI_CMD_CTLR				8

#define BT_HCI_CMD_CTLR_SET_EVT_MASK						((8<<10) | 1)		//!< LE Set Event Mask command
#define BT_HCI_CMD_CTLR_READ_BUFF_SIZE						((8<<10) | 2)		//!< LE Read Buffer Size command V1
#define BT_HCI_CMD_CTLR_READ_BUFF_SIZE_EXT					((8<<10) | 0x60)	//!< LE Read Buffer Size command V2
#define BT_HCI_CMD_CTLR_READ_LOCAL_SUPP_FEATURES			((8<<10) | 3)		//!< LE Read Local Supported Features command
#define BT_HCI_CMD_CTLR_SET_RAND_ADDR						((8<<10) | 5)		//!< LE Set Random Address command
#define BT_HCI_CMD_CTLR_SET_ADV_PARAM						((8<<10) | 6)		//!< LE Set Advertising Parameters command
#define BT_HCI_CMD_CTLR_READ_ADV_CHAN_TX_PWR				((8<<10) | 7)		//!< LE Read Advertising Physical Channel Tx Power command
#define BT_HCI_CMD_CTLR_SET_ADV_DATA						((8<<10) | 8)		//!< LE Set Advertising Data command
#define BT_HCI_CMD_CTLR_SET_SCAN_RESP_DATA					((8<<10) | 9)		//!< LE Set Scan Response Data command
#define BT_HCI_CMD_CTLR_SET_ADV_ENABLE						((8<<10) | 0xA)		//!< LE Set Advertising Enable command
#define BT_HCI_CMD_CTLR_SET_SCAN_PARAM						((8<<10) | 0xB)		//!< LE Set Scan Parameters command
#define BT_HCI_CMD_CTLR_SET_SCAN_ENABLE						((8<<10) | 0xC)		//!< LE Set Scan Enable command
#define BT_HCI_CMD_CTLR_CREATE_CONN							((8<<10) | 0xD)		//!< LE Create Connection command
#define BT_HCI_CMD_CTLR_CREATE_CONN_CANCEL					((8<<10) | 0xE)		//!< LE Create Connection Cancel command
#define BT_HCI_CMD_CTLR_READ_FILTER_ACCEPT_LIST_SIZE		((8<<10) | 0xF)		//!< LE Read Filter Accept List Size command
#define BT_HCI_CMD_CTLR_CLEAR_FILTER_ACCEPT_LIST			((8<<10) | 0x10)	//!< LE Clear Filter Accept List command
#define BT_HCI_CMD_CTLR_FILTER_ACCEPT_LIST_ADD_DEV			((8<<10) | 0x11)	//!< LE Add Device To Filter Accept List command
#define BT_HCI_CMD_CTLR_FILTER_ACCEPT_LIST_REMOVE_DEV		((8<<10) | 0x12)	//!< LE Remove Device From Filter Accept List command
#define BT_HCI_CMD_CTLR_CONN_UPDATE							((8<<10) | 0x13)	//!< LE Connection Update command
#define BT_HCI_CMD_CTLR_SET_HOST_CHAN_CLASS					((8<<10) | 0x14)	//!< LE Set Host Channel Classification command
#define BT_HCI_CMD_CTLR_READ_CHAN_MAP						((8<<10) | 0x15)	//!< LE Read Channel Map command
#define BT_HCI_CMD_CTLR_READ_REMOTE_FEATURES				((8<<10) | 0x16)	//!< LE Read Remote Features command
#define BT_HCI_CMD_CTLR_ENCRYPT								((8<<10) | 0x17)	//!< LE Encrypt command
#define BT_HCI_CMD_CTLR_RAND								((8<<10) | 0x18)	//!< LE Rand command
#define BT_HCI_CMD_CTLR_ENABLE_ENCRYPTION					((8<<10) | 0x19)	//!< LE Enable Encryption command
#define BT_HCI_CMD_CTLR_LONGTERM_KEY_REQUEST_REPLY			((8<<10) | 0x1A)	//!< LE Long Term Key Request Reply command
#define BT_HCI_CMD_CTLR_LONGTERM_KEY_REQUEST_NEG_REPLY		((8<<10) | 0X1B)	//!< LE Long Term Key Request Negative Reply command
#define BT_HCI_CMD_CTLR_READ_SUPPORTED_STATES				((8<<10) | 0x1C)	//!< LE Read Supported States command
#define BT_HCI_CMD_CTLR_RECEIVER_TEST_V1					((8<<10) | 0x1D)	//!< LE Receiver Test command V1
#define BT_HCI_CMD_CTLR_RECEIVER_TEST_V2					((8<<10) | 0x33)	//!< LE Receiver Test command V2
#define BT_HCI_CMD_CTLR_RECEIVER_TEST_V3					((8<<10) | 0x4F)	//!< LE Receiver Test command V3
#define BT_HCI_CMD_CTLR_TRANSMITTER_TEST_V1					((8<<10) | 0x1E)	//!< LE Transmitter Test command V1
#define BT_HCI_CMD_CTLR_TRANSMITTER_TEST_V2					((8<<10) | 0x34)	//!< LE Transmitter Test command V2
#define BT_HCI_CMD_CTLR_TRANSMITTER_TEST_V3					((8<<10) | 0x50)	//!< LE Transmitter Test command V3
#define BT_HCI_CMD_CTLR_TRANSMITTER_TEST_V4					((8<<10) | 0x17B)	//!< LE Transmitter Test command V4
#define BT_HCI_CMD_CTLR_TEST_END							((8<<10) | 0x1F)	//!< LE Test End command
#define BT_HCI_CMD_CTLR_REMOTE_CONN_PARAM_REQUEST_REPLY		((8<<10) | 0x20)	//!< LE Remote Connection Parameter Request Reply command
#define BT_HCI_CMD_CTLR_REMOTE_CONN_PARAM_RQST_NEG_REPLY	((8<<10) | 0x21)	//!< LE Remote Connection Parameter Request Negative Reply command
#define BT_HCI_CMD_CTLR_SET_DATA_LEN						((8<<10) | 0x22)	//!< LE Set Data Length command
#define BT_HCI_CMD_CTLR_READ_SUGG_DEFAULT_DATA_LEN			((8<<10) | 0x23)	//!< LE Read Suggested Default Data Length command
#define BT_HCI_CMD_CTLR_WRITE_SUGG_DEFAULT_DATA_LEN			((8<<10) | 0x24)	//!< LE Write Suggested Default Data Length command
#define BT_HCI_CMD_CTLR_READ_LOCAL_P256_PUBLIC_KEY			((8<<10) | 0x25)	//!< LE Read Local P-256 Public Key command
#define BT_HCI_CMD_CTLR_GENERATE_DHKEY_V1					((8<<10) | 0x26)	//!< LE Generate DHKey command V1
#define BT_HCI_CMD_CTLR_GENERATE_DHKEY_V2					((8<<10) | 0x5E)	//!< LE Generate DHKey command V2
#define BT_HCI_CMD_CTLR_RESOLVING_LIST_ADD_DEV				((8<<10) | 0x27)	//!< LE Add Device To Resolving List command
#define BT_HCI_CMD_CTLR_RESOLVING_LIST_REMOVE_DEV			((8<<10) | 0x28)	//!< LE Remove Device From Resolving List command
#define BT_HCI_CMD_CTLR_RESOLVING_LIST_CLEAR				((8<<10) | 0x29)	//!< LE Clear Resolving List command
#define BT_HCI_CMD_CTLR_RESOLVING_LIST_READ_SIZE			((8<<10) | 0x2A)	//!< LE Read Resolving List Size command
#define BT_HCI_CMD_CTLR_READ_PEER_RESOLVABLE_ADDR			((8<<10) | 0x2B)	//!< LE Read Peer Resolvable Address command
#define BT_HCI_CMD_CTLR_READ_LOCAL_RESOLVABLE_ADDR			((8<<10) | 0x2C)	//!< LE Read Local Resolvable Address command
#define BT_HCI_CMD_CTLR_SET_ADDR_RESOLUTION_ENABLE			((8<<10) | 0x2D)	//!< LE Set Address Resolution Enable command
#define BT_HCI_CMD_CTLR_SET_RESOLVABLE_PRIVATE_ADDR_TIMEOUT	((8<<10) | 0x2E)	//!< LE Set Resolvable Private Address Timeout command
#define BT_HCI_CMD_CTLR_READ_MAX_DATA_LEN					((8<<10) | 0x2F)	//!< LE Read Maximum Data Length command
#define BT_HCI_CMD_CTLR_READ_PHY							((8<<10) | 0x30)	//!< LE Read PHY command
#define BT_HCI_CMD_CTLR_SET_DEFAULT_PHY						((8<<10) | 0x31)	//!< LE Set Default PHY command
#define BT_HCI_CMD_CTLR_SET_PHY								((8<<10) | 0x32)	//!< LE Set PHY command
#define BT_HCI_CMD_CTLR_SET_ADV_SET_RAND_ADDR				((8<<10) | 0x35)	//!< LE Set Advertising Set Random Address command
#define BT_HCI_CMD_CTLR_SET_EXT_ADV_PARAM					((8<<10) | 0x36)	//!< LE Set Extended Advertising Parameters command
#define BT_HCI_CMD_CTLR_SET_EXT_ADV_DATA					((8<<10) | 0x37)	//!< LE Set Extended Advertising Data command
#define BT_HCI_CMD_CTLR_SET_EXT_SCAN_RESP_DATA				((8<<10) | 0x38)	//!< LE Set Extended Scan Response Data command
#define BT_HCI_CMD_CTLR_SET_EXT_ADV_ENABLE					((8<<10) | 0x39)	//!< LE Set Extended Advertising Enable command
#define BT_HCI_CMD_CTLR_READ_MAX_ADV_DATA_LEN				((8<<10) | 0x3A)	//!< LE Read Maximum Advertising Data Length command
#define BT_HCI_CMD_CTLR_READ_NB_SUPPORTED_ADV_SETS			((8<<10) | 0x3B)	//!< LE Read Number of Supported Advertising Sets command
#define BT_HCI_CMD_CTLR_REMOVE_ADV_SET						((8<<10) | 0x3C)	//!< LE Remove Advertising Set command
#define BT_HCI_CMD_CTLR_CLEAR_ADV_SETS						((8<<10) | 0x3D)	//!< LE Clear Advertising Sets command
#define BT_HCI_CMD_CTLR_SET_PERIODIC_ADV_PARAM				((8<<10) | 0x3E)	//!< LE Set Periodic Advertising Parameters command
#define BT_HCI_CMD_CTLR_SET_PERIODIC_ADV_DATA				((8<<10) | 0x3F)	//!< LE Set Periodic Advertising Data command
#define BT_HCI_CMD_CTLR_SET_PERIODIC_ADV_ENABLE				((8<<10) | 0x40)	//!< LE Set Periodic Advertising Enable command
#define BT_HCI_CMD_CTLR_SET_EXT_SCAN_PARAM					((8<<10) | 0x41)	//!< LE Set Extended Scan Parameters command
#define BT_HCI_CMD_CTLR_SET_EXT_SCAN_ENABLE					((8<<10) | 0x42)	//!< LE Set Extended Scan Enable command
#define BT_HCI_CMD_CTLR_EXT_CREATE_CONN						((8<<10) | 0x43)	//!< LE Extended Create Connection command
#define BT_HCI_CMD_CTLR_PERIODIC_ADV_CREATE_SYNC			((8<<10) | 0x44)	//!< LE Periodic Advertising Create Sync command
#define BT_HCI_CMD_CTLR_PERIODIC_ADV_CREATE_SYNC_CANCEL		((8<<10) | 0x45)	//!< LE Periodic Advertising Create Sync Cancel command
#define BT_HCI_CMD_CTLR_PERIODIC_ADV_TERMINATE_SYNC			((8<<10) | 0x46)	//!< LE Periodic Advertising Terminate Sync command
#define BT_HCI_CMD_CTLR_PERIODIC_ADV_LIST_ADD_DEV			((8<<10) | 0x47)	//!< LE Add Device To Periodic Advertiser List command
#define BT_HCI_CMD_CTLR_PERIODIC_ADV_LIST_REMOVE_DEV		((8<<10) | 0x48)	//!< LE Remove Device From Periodic Advertiser List command
#define BT_HCI_CMD_CTLR_PERIODIC_ADV_LIST_CLEAR				((8<<10) | 0x49)	//!< LE Clear Periodic Advertiser List command
#define BT_HCI_CMD_CTLR_PERIODIC_ADV_LIST_READ_SIZE			((8<<10) | 0x4A)	//!< LE Read Periodic Advertiser List Size command
#define BT_HCI_CMD_CTLR_READ_TRANSMIT_PWR					((8<<10) | 0x4B)	//!< LE Read Transmit Power command
#define BT_HCI_CMD_CTLR_READ_RF_PATH_COMPENSATION			((8<<10) | 0x4C)	//!< LE Read RF Path Compensation command
#define BT_HCI_CMD_CTLR_WRITE_RF_PATH_COMPENSATION			((8<<10) | 0x4D)	//!< LE Write RF Path Compensation command
#define BT_HCI_CMD_CTLR_SET_PRIVACY_MODE					((8<<10) | 0x4E)	//!< LE Set Privacy Mode command
#define BT_HCI_CMD_CTLR_SET_CONNLESS_CTE_TRANSMIT_PARAM		((8<<10) | 0x51)	//!< LE Set Connectionless CTE Transmit Parameters command
#define BT_HCI_CMD_CTLR_SET_CONNLESS_CTE_TRANSMIT_ENABLE	((8<<10) | 0x52)	//!< LE Set Connectionless CTE Transmit Enable command
#define BT_HCI_CMD_CTLR_SET_CONNLESS_IQ_SAMPLING_ENABLE		((8<<10) | 0x53)	//!< LE Set Connectionless IQ Sampling Enable command
#define BT_HCI_CMD_CTLR_SET_CONN_CTE_RECEIVE_PARAM			((8<<10) | 0x54)	//!< LE Set Connection CTE Receive Parameters command
#define BT_HCI_CMD_CTLR_SET_CONN_CTE_TRANSMIT_PARAM			((8<<10) | 0x55)	//!< LE Set Connection CTE Transmit Parameters command
#define BT_HCI_CMD_CTLR_CONN_CTE_RQST_ENABLE				((8<<10) | 0x56)	//!< LE Connection CTE Request Enable command
#define BT_HCI_CMD_CTLR_CONN_CTE_RESPONSE_ENABLE			((8<<10) | 0x57)	//!< LE Connection CTE Response Enable command
#define BT_HCI_CMD_CTLR_READ_ANT_INFO						((8<<10) | 0x58)	//!< LE Read Antenna Information command
#define BT_HCI_CMD_CTLR_SET_PERIODIC_ADV_RECEIVE_ENABLE		((8<<10) | 0x59)	//!< LE Set Periodic Advertising Receive Enable command
#define BT_HCI_CMD_CTLR_PERIODIC_ADV_SYNC_TRANSFER			((8<<10) | 0x5A)	//!< LE Periodic Advertising Sync Transfer command
#define BT_HCI_CMD_CTLR_PERIODIC_ADV_SET_INFO_TRANSFER		((8<<10) | 0x5B)	//!< LE Periodic Advertising Set Info Transfer command
#define BT_HCI_CMD_CTLR_SET_PERIODIC_ADV_SYNC_TRANSFER_PARAM			((8<<10) | 0x5C)	//!< LE Set Periodic Advertising Sync Transfer Parameters command
#define BT_HCI_CMD_CTLR_SET_DEFAULT_PERIODIC_ADV_SYNC_TRANSFER_PARAM	((8<<10) | 0x5D)	//!< LE Set Default Periodic Advertising Sync Transfer Parameters command
#define BT_HCI_CMD_CTLR_MODIFY_SLEEP_CLOCK_ACCURACY			((8<<10) | 0x5F)	//!< LE Modify Sleep Clock Accuracy command
#define BT_HCI_CMD_CTLR_READ_ISO_TX_SYNC					((8<<10) | 0x61)	//!< LE Read ISO TX Sync command
#define BT_HCI_CMD_CTLR_SET_CIG_PARAM						((8<<10) | 0x62)	//!< LE Set CIG Parameters command
#define BT_HCI_CMD_CTLR_SET_CIG_PARAM_TEST					((8<<10) | 0x63)	//!< LE Set CIG Parameters Test command
#define BT_HCI_CMD_CTLR_CREATE_CIS							((8<<10) | 0x64)	//!< LE Create CIS command
#define BT_HCI_CMD_CTLR_REMOVE_CIG							((8<<10) | 0x65)	//!< LE Remove CIG command
#define BT_HCI_CMD_CTLR_ACCEPT_CIS_RQST						((8<<10) | 0x66)	//!< LE Accept CIS Request command
#define BT_HCI_CMD_CTLR_REJECT_CIS_RQST						((8<<10) | 0x67)	//!< LE Reject CIS Request command
#define BT_HCI_CMD_CTLR_CREATE_BIG							((8<<10) | 0x68)	//!< LE Create BIG command
#define BT_HCI_CMD_CTLR_CREATE_BIG_TEST						((8<<10) | 0x69)	//!< LE Create BIG Test command
#define BT_HCI_CMD_CTLR_TERMINATE_BIG						((8<<10) | 0x6A)	//!< LE Terminate BIG command
#define BT_HCI_CMD_CTLR_BIG_CREATE_SYNC						((8<<10) | 0x6B)	//!< LE BIG Create Sync command
#define BT_HCI_CMD_CTLR_BIG_TERMINATE_SYNC					((8<<10) | 0x6C)	//!< LE BIG Terminate Sync command
#define BT_HCI_CMD_CTLR_RQST_PEER_SCA						((8<<10) | 0x6D)	//!< LE Request Peer SCA command
#define BT_HCI_CMD_CTLR_SETUP_ISO_DATA_PATH					((8<<10) | 0x6E)	//!< LE Setup ISO Data Path command
#define BT_HCI_CMD_CTLR_REMOVE_ISO_DATA_PATH				((8<<10) | 0x6F)	//!< LE Remove ISO Data Path command
#define BT_HCI_CMD_CTLR_ISO_TRANSMIT_TEST					((8<<10) | 0x70)	//!< LE ISO Transmit Test command
#define BT_HCI_CMD_CTLR_ISO_RECEIVE_TEST					((8<<10) | 0x71)	//!< LE ISO Receive Test command
#define BT_HCI_CMD_CTLR_ISO_READ_TEST_COUNTERS				((8<<10) | 0x72)	//!< LE ISO Read Test Counters command
#define BT_HCI_CMD_CTLR_ISO_TEST_END						((8<<10) | 0x73)	//!< LE ISO Test End command
#define BT_HCI_CMD_CTLR_SET_HOST_FEATURE					((8<<10) | 0x74)	//!< LE Set Host Feature command
#define BT_HCI_CMD_CTLR_READ_ISO_LINK_QUALITY				((8<<10) | 0x75)	//!< LE Read ISO Link Quality command
#define BT_HCI_CMD_CTLR_ENHANCED_READ_TRANSMIT_PWR_LEVEL	((8<<10) | 0x76)	//!< LE Enhanced Read Transmit Power Level command
#define BT_HCI_CMD_CTLR_READ_REMOTE_TRANSMIT_PWR_LEVEL		((8<<10) | 0x77)	//!< LE Read Remote Transmit Power Level command
#define BT_HCI_CMD_CTLR_SET_PATH_LOSS_REPORTING_PARAM		((8<<10) | 0x78)	//!< LE Set Path Loss Reporting Parameters command
#define BT_HCI_CMD_CTLR_SET_PATH_LOSS_REPORTING_ENABLE		((8<<10) | 0x79)	//!< LE Set Path Loss Reporting Enable command
#define BT_HCI_CMD_CTLR_SET_TRANSMIT_PWR_REPORTING_ENABLE	((8<<10) | 0x7A)	//!< LE Set Transmit Power Reporting Enable command
#define BT_HCI_CMD_CTLR_SET_DATA_RELATED_ADDR_CHANGE		((8<<10) | 0x7C)	//!< LE Set Data Related Address Changes command
#define BT_HCI_CMD_CTLR_SET_DEFAULT_SUBRATE					((8<<10) | 0x7D)	//!< LE Set Default Subrate command
#define BT_HCI_CMD_CTLR_SUBRATE_RQST						((8<<10) | 0x7E)	//!< LE Subrate Request command



// HCI events
#define BT_HCI_EVT_INQUERY_COMPLETE							1		//!< Inquiry Complete event
#define BT_HCI_EVT_INQUERY_RESULT							2		//!< Inquiry Result event
#define BT_HCI_EVT_CONN_COMPLETE							3		//!< Connection Complete event
#define BT_HCI_EVT_CONN_REQUEST								4		//!< Connection Request event
#define BT_HCI_EVT_DISCONN_COMPLETE							5		//!< Disconnection Complete event
#define BT_HCI_EVT_AUTHEN_COMPLETE							6		//!< Authentication Complete event
#define BT_HCI_EVT_REMOTE_NAME_RQST_COMPLETE				7		//!< Remote Name Request Complete event
#define BT_HCI_EVT_ENCRYPTION_CHANGE						8		//!< Encryption Change event V1
#define BT_HCI_EVT_ENCRYPTION_CHANGE_V2						0x59	//!< Encryption Change event V2
#define BT_HCI_EVT_CHANGE_CONN_LINK_KEY_COMPLETE			9		//!< Change Connection Link Key Complete event
#define BT_HCI_EVT_LINK_KEY_TYPE_CHANGED					0xA		//!< Link Key Type Changed event
#define BT_HCI_EVT_READ_REMOTE_SUPPORTED_FEATURES_COMPLETE	0xB		//!< Read Remote Supported Features Complete event
#define BT_HCI_EVT_READ_REMOTE_VERS_INFO_COMPLETE			0xC		//!< Read Remote Version Information Complete event
#define BT_HCI_EVT_QOS_SETTUP_COMPLETE						0xD		//!< QoS Setup Complete event
#define BT_HCI_EVT_COMMAND_COMPLETE							0xE		//!< Command Complete event
#define BT_HCI_EVT_COMMAND_STATUS							0xF		//!< Command Status event
#define BT_HCI_EVT_HARDWARE_ERROR							0x10	//!< Hardware Error event
#define BT_HCI_EVT_FLUSH_OCCURED							0x11	//!< Flush Occurred event
#define BT_HCI_EVT_ROLE_CHANGE								0x12	//!< Role Change event
#define BT_HCI_EVT_NB_COMPLETED_PACKET						0x13	//!< Number Of Completed Packets event
#define BT_HCI_EVT_MODE_CHANGE								0x14	//!< Mode Change event
#define BT_HCI_EVT_RETURN_LINK_KEYS							0x15	//!< Return Link Keys event
#define BT_HCI_EVT_PIN_CODE_RQST							0x16	//!< PIN Code Request event
#define BT_HCI_EVT_LINK_KEY_RQST							0x17	//!< Link Key Request event
#define BT_HCI_EVT_LINK_KEY_NOTIF							0x18	//!< Link Key Notification event
#define BT_HCI_EVT_LOOPBACK_COMMAND							0x19	//!< Loopback Command event
#define BT_HCI_EVT_DATA_BUFFER_OVERFLOW						0x1A	//!< Data Buffer Overflow event
#define BT_HCI_EVT_MAX_SLOT_CHANGE							0x1B	//!< Max Slots Change event
#define BT_HCI_EVT_READ_CLOCK_OFFSET_COMPLETE				0x1C	//!< Read Clock Offset Complete event
#define BT_HCI_EVT_CONN_PACKET_TYPE_CHANGED					0x1D	//!< Connection Packet Type Changed event
#define BT_HCI_EVT_QOS_VIOLATION							0x1E	//!< QoS Violation event
#define BT_HCI_EVT_PAGE_SCAN_REPETITION_MODE_CHANGE			0x20	//!< Page Scan Repetition Mode Change event
#define BT_HCI_EVT_FLOW_SPECS_COMPLETE						0x21	//!< Flow Specification Complete event
#define BT_HCI_EVT_INQUIRY_RESULT_WITH_RSSI					0x22	//!< Inquiry Result with RSSI event
#define BT_HCI_EVT_READ_REMOTE_EXT_FEATURES_COMPLETE		0x23	//!< Read Remote Extended Features Complete event
#define BT_HCI_EVT_SYNCHRONOUS_CONN_COMPLETE				0x2C	//!< Synchronous Connection Complete event
#define BT_HCI_EVT_SYNCHRONOUS_CONN_CHANGED					0x2D	//!< Synchronous Connection Changed event
#define BT_HCI_EVT_SNIFF_SUBRATING							0x2E	//!< Sniff Subrating event
#define BT_HCI_EVT_EXT_INQUIRY_RESULT						0x2F	//!< Extended Inquiry Result event
#define BT_HCI_EVT_ENCRYPTION_KEY_REFRESH_COMPLETE			0x30	//!< Encryption Key Refresh Complete event
#define BT_HCI_EVT_IO_CAPABILITY_RQST						0x31	//!< IO Capability Request event
#define BT_HCI_EVT_IO_CAPABILITY_RESPONSE					0x32	//!< IO Capability Response event
#define BT_HCI_EVT_USER_CONFIRM_RQST						0x33	//!< User Confirmation Request event
#define BT_HCI_EVT_USER_PASSKEY_RQST						0x34	//!< User Passkey Request event
#define BT_HCI_EVT_REMOTE_OOB_DATA_RQST						0x35	//!< Remote OOB Data Request event
#define BT_HCI_EVT_SIMPLE_PAIRING_COMPLETE					0x36	//!< Simple Pairing Complete event
#define BT_HCI_EVT_LINK_SUPERVISION_TIMEOUT_CHANGED			0x38	//!< Link Supervision Timeout Changed event
#define BT_HCI_EVT_ENHANCED_FLUSH_COMPLETE					0x39	//!< Enhanced Flush Complete event
#define BT_HCI_EVT_USER_PASSKEY_NOTIF						0x3B	//!< User Passkey Notification event
#define BT_HCI_EVT_KEYPRESS_NOTIF							0x3C	//!< Keypress Notification event
#define BT_HCI_EVT_REMOTE_HOST_SUPPORTED_FEATURES_NOTIF		0x3D	//!< Remote Host Supported Features Notification event
#define BT_HCI_EVT_NB_COMPLETED_DATA_BLOCKS					0x48	//!< Number Of Completed Data Blocks event
#define BT_HCI_EVT_TRIGGERED_CLOCK_CAPTURE					0x4E	//!< Triggered Clock Capture event
#define BT_HCI_EVT_SYNC_TRAIN_COMPLETE						0x4F	//!< Synchronization Train Complete event
#define BT_HCI_EVT_SYNC_TRAIN_RECEIVED						0x50	//!< Synchronization Train Received event
#define BT_HCI_EVT_CONNLESS_PERIPH_BROADCAST_RECEIVE		0x51	//!< Connectionless Peripheral Broadcast Receive event
#define BT_HCI_EVT_CONNLESS_PERIPH_BROADCAST_TIMEOUT		0x52	//!< Connectionless Peripheral Broadcast Timeout event
#define BT_HCI_EVT_TRUNCATED_PAGE_COMPLETE					0x53	//!< Truncated Page Complete event
#define BT_HCI_EVT_PERIPH_PAGE_RESPONSE_TIMNEOUT			0x54	//!< Peripheral Page Response Timeout event
#define BT_HCI_EVT_CONNLESS_PERIPH_BROADCAST_CHAN_MAP_CHANGE	0x55	//!< Connectionless Peripheral Broadcast Channel Map Change event
#define BT_HCI_EVT_INQUIRY_RESPONSE_NOTIF					0x56	//!< Inquiry Response Notification event
#define BT_HCI_EVT_AUTHEN_PAYLOAD_TIMEOUT_EXPIRED			0x57	//!< Authenticated Payload Timeout Expired event
#define BT_HCI_EVT_SAM_STATUS_CHANGE						0x58	//!< SAM Status Change event
#define BT_HCI_EVT_LE										0x3E	//!< LE Meta event
#define BT_HCI_EVT_LE_CONN_COMPLETE							1		//!< LE Connection Complete event
#define BT_HCI_EVT_LE_ADV_REPORT								2		//!< LE Advertising Report event
#define BT_HCI_EVT_LE_CONN_UPDATE_COMPLETE						3		//!< LE Connection Update Complete event
#define BT_HCI_EVT_LE_READ_REMOTE_FEATURES_COMPLETE			4		//!< LE Read Remote Features Complete event
#define BT_HCI_EVT_LE_LONGTERM_KEY_RQST						5		//!< LE Long Term Key Request event
#define BT_HCI_EVT_LE_REMOTE_CONN_PARAM_RQST					6		//!< LE Remote Connection Parameter Request event
#define BT_HCI_EVT_LE_DATA_LEN_CHANGE							7		//!< LE Data Length Change event
#define BT_HCI_EVT_LE_READ_LOCAL_P256_PUBLIC_KEY_COMPLETE		8		//!< LE Read Local P-256 Public Key Complete event
#define BT_HCI_EVT_LE_GENERATE_DHKEY_COMPLETE					9		//!< LE Generate DHKey Complete event
#define BT_HCI_EVT_LE_ENHANCED_CONN_COMPLETE					0xA		//!< LE Enhanced Connection Complete event
#define BT_HCI_EVT_LE_DIRECTED_ADV_REPORT						0xB		//!< LE Directed Advertising Report event
#define BT_HCI_EVT_LE_PHY_UPDATE_COMPLETE						0xC		//!< LE PHY Update Complete event
#define BT_HCI_EVT_LE_EXT_ADV_REPORT							0xD		//!< LE Extended Advertising Report event
#define BT_HCI_EVT_LE_PERIODIC_ADV_SYNC_ESTABLISHED			0xE		//!< LE Periodic Advertising Sync Established event
#define BT_HCI_EVT_LE_PERIODIC_ADV_REPORT						0xF		//!< LE Periodic Advertising Report event
#define BT_HCI_EVT_LE_PERIODIC_ADV_SYNC_LOST					0x10	//!< LE Periodic Advertising Sync Lost event
#define BT_HCI_EVT_LE_SCAN_TIMEOUT								0x11	//!< LE Scan Timeout event
#define BT_HCI_EVT_LE_ADV_SET_TERMINATED						0x12	//!< LE Advertising Set Terminated event
#define BT_HCI_EVT_LE_SCAN_RQST_RECEIVED						0x13	//!< LE Scan Request Received event
#define BT_HCI_EVT_LE_CHAN_SELECTION_ALGO						0x14	//!< LE Channel Selection Algorithm event
#define BT_HCI_EVT_LE_CONNLESS_IQ_REPORT						0x15	//!< LE Connectionless IQ Report event
#define BT_HCI_EVT_LE_CONN_IQ_REPORT							0x16	//!< LE Connection IQ Report event
#define BT_HCI_EVT_LE_CTE_RQST_FAILED							0x17	//!< LE CTE Request Failed event
#define BT_HCI_EVT_LE_PERIODIC_ADV_SYNC_TRANSFER_RECEIVED		0x18	//!< LE Periodic Advertising Sync Transfer Received event
#define BT_HCI_EVT_LE_CIS_ESTABLISHED							0x19	//!< LE CIS Established event
#define BT_HCI_EVT_LE_CIS_RQST									0x1A	//!< LE CIS Request event
#define BT_HCI_EVT_LE_CREATE_BIG_COMPLETE						0x1B	//!< LE Create BIG Complete event
#define BT_HCI_EVT_LE_TERMINATE_BIG_COMPLETE					0x1C	//!< LE Terminate BIG Complete event
#define BT_HCI_EVT_LE_BIG_SYNC_ESTABLISHED						0x1D	//!< LE BIG Sync Established event
#define BT_HCI_EVT_LE_BIG_SYNC_LOST							0x1E	//!< LE BIG Sync Lost event
#define BT_HCI_EVT_LE_RQST_PEER_SCA_COMPLETE					0x1F	//!< LE Request Peer SCA Complete event
#define BT_HCI_EVT_LE_PATH_LOSS_THREESHOLD						0x20	//!< LE Path Loss Threshold event
#define BT_HCI_EVT_LE_TRANSMIT_PWR_REPORTING					0x21	//!< LE Transmit Power Reporting event
#define BT_HCI_EVT_LE_BIGINFO_ADV_REPORT						0x22	//!< LE BIGInfo Advertising Report event
#define BT_HCI_EVT_LE_SUBRATE_CHANGE							0x23	//!< LE Subrate Change event


// HCI error codes
#define BT_HCI_SUCCESS										0
#define BT_HCI_ERR_NONE										BT_HCI_SUCCESS
#define BT_HCI_ERR_UNKNOWN_COMMAND							1			//!< Unknown HCI Command
#define BT_HCI_ERR_UNKNOWN_CONN_ID							2			//!< Unknown Connection Identifier
#define BT_HCI_ERR_HARDWARE_FAILURE							3			//!< Hardware Failure
#define BT_HCI_ERR_PAGE_TIMEOUT								4			//!< Page Timeout
#define BT_HCI_ERR_AUTHEN_FAILURE							5			//!< Authentication Failure
#define BT_HCI_ERR_KEY_MISSING								6			//!< PIN or Key Missing
#define BT_HCI_ERR_MEMORY_FULL								7			//!< Memory Capacity Exceeded
#define BT_HCI_ERR_CONN_TIMEOUT								8			//!< Connection Timeout
#define BT_HCI_ERR_CONN_LIMIT_EXCEEDED						9			//!< Connection Limit Exceeded
#define BT_HCI_ERR_SYNCHRONOUS_CONN_LIMIT_EXCEEDED			0xA			//!< Synchronous Connection Limit To A Device Exceeded
#define BT_HCI_ERR_CONN_EXISTS								0xB			//!< Connection Already Exists
#define BT_HCI_ERR_COMMAND_DISALLOWED						0xC			//!< Command Disallowed
#define BT_HCI_ERR_CONN_REJECT_RESOURCE_LIMIT				0xD			//!< Connection Rejected due to Limited Resources
#define BT_HCI_ERR_CONN_REJECT_SECURITY						0xE			//!< Connection Rejected Due To Security Reasons
#define BT_HCI_ERR_CONN_REJECT_BD_ADDR						0xF			//!< Connection Rejected due to Unacceptable BD_ADDR
#define BT_HCI_ERR_CONN_ACCEPT_TIMEOUT						0x10		//!< Connection Accept Timeout Exceeded
#define BT_HCI_ERR_UNSUPPORTED_FEATURE_PARAM				0x11		//!< Unsupported Feature or Parameter Value
#define BT_HCI_ERR_INVALID_COMMAND_PARAM					0x12		//!< Invalid HCI Command Parameters
#define BT_HCI_ERR_CONN_TERMINATED_USER						0x13		//!< Remote User Terminated Connection
#define BT_HCI_ERR_CONN_TERMINATED_LOW_RESOURCE				0x14		//!< Remote Device Terminated Connection due to Low Resources
#define BT_HCI_ERR_CONN_TERMINATED_POWER_OFF				0x15		//!< Remote Device Terminated Connection due to Power Off
#define BT_HCI_ERR_CONN_TERMINATED_LOCAL_HOST				0x16		//!< Connection Terminated By Local Host
#define BT_HCI_ERR_REPEATED_ATTEMPTS						0x17		//!< Repeated Attempts
#define BT_HCI_ERR_PAIRING_NOT_ALLOWED						0x18		//!< Pairing Not Allowed
#define BT_HCI_ERR_UNKNOWN_LMP_PDU							0x19		//!< Unknown LMP PDU
#define BT_HCI_ERR_UNSUPPORTED_REMOTE_FEATURE				0x1A		//!< Unsupported Remote Feature
#define BT_HCI_ERR_SCO_OFFSET_REJECTED						0x1B		//!< SCO Offset Rejected
#define BT_HCI_ERR_SCO_INTERVAL_REJECTED					0x1C		//!< SCO Interval Rejected
#define BT_HCI_ERR_SCO_AIR_MODE_REJECTED					0x1D		//!< SCO Air Mode Rejected
#define BT_HCI_ERR_INVALID_LMP_LL_PARAM						0x1E		//!< Invalid LMP Parameters / Invalid LL Parameters
#define BT_HCI_ERR_UNSPECIFIED								0x1F		//!< Unspecified Error
#define BT_HCI_ERR_UNSUPPORTED_LMP_LL_PARAM					0x20		//!< Unsupported LMP Parameter Value / Unsupported LL Parameter Value
#define BT_HCI_ERR_ROLE_CHANGE_NOT_ALLOWED					0x21		//!< Role Change Not Allowed
#define BT_HCI_ERR_LMP_LL_RESPONSE_TIMEOUT					0x22		//!< LMP Response Timeout / LL Response Timeout
#define BT_HCI_ERR_LMP_LL_TRANSACTION_COLLISION				0x23		//!< LMP Error Transaction Collision / LL Procedure Collision
#define BT_HCI_ERR_LMP_PDU_NOT_ALLOWED						0x24		//!< LMP PDU Not Allowed
#define BT_HCI_ERR_ENCRYPTION_MODE_NOT_ACCEPTABLE			0x25		//!< Encryption Mode Not Acceptable
#define BT_HCI_ERR_LINK_KEY_NO_CHANGE						0x26		//!< Link Key cannot be Changed
#define BT_HCI_ERR_QOS_NOT_SUPPORTED						0x27		//!< Requested QoS Not Supported
#define BT_HCI_ERR_INSTANT_PASSED							0x28		//!< Instant Passed
#define BT_HCI_ERR_PAIRING_WITH_UNIT_KEY_NOT_SUPPORTED		0x29		//!< Pairing With Unit Key Not Supported
#define BT_HCI_ERR_DIFF_TRANSACTION_COLLISION				0x2A		//!< Different Transaction Collision
#define BT_HCI_ERR_RESERVED_2B								0x2B		//!< Reserved for future use
#define BT_HCI_ERR_QOS_UNACCEPTABLE_PARAM					0x2C		//!< QoS Unacceptable Parameter
#define BT_HCI_ERR_QOS_REJECTED								0x2D		//!< QoS Rejected
#define BT_HCI_ERR_CHAN_CLASS_NOT_SUPPORTED					0x2E		//!< Channel Classification Not Supported
#define BT_HCI_ERR_INSUFFICIENT_SECURITY					0x2F		//!< Insufficient Security
#define BT_HCI_ERR_PARAM_RANGE								0x30		//!< Parameter Out Of Mandatory Range
#define BT_HCI_ERR_RESERVED_31								0x31		//!< Reserved for future use
#define BT_HCI_ERR_ROLE_SWITCH_PENDING						0x32		//!< Role Switch Pending
#define BT_HCI_ERR_RESERVED_33								0x33		//!< Reserved for future use
#define BT_HCI_ERR_RESERVED_SLOT_VIOLATION					0x34		//!< Reserved Slot Violation
#define BT_HCI_ERR_ROLE_SWITCH_FAILED						0x35		//!< Role Switch Failed
#define BT_HCI_ERR_EXT_INQUIRY_RESPONSE_TOO_LARGE			0x36		//!< Extended Inquiry Response Too Large
#define BT_HCI_ERR_SECURE_SIMPLE_PAIRING_NOT_SUPPORTED		0x37		//!< Secure Simple Pairing Not Supported By Host
#define BT_HCI_ERR_HOST_BUSY_PAIRING						0x38		//!< Host Busy - Pairing
#define BT_HCI_ERR_CONN_REJECT_NO_CHAN						0x39		//!< Connection Rejected due to No Suitable Channel Found
#define BT_HCI_ERR_CONTROLER_BUSY							0x3A		//!< Controller Busy
#define BT_HCI_ERR_UNACCEPTABLE_CONN_PARAM					0x3B		//!< Unacceptable Connection Parameters
#define BT_HCI_ERR_ADV_TIMEOUT								0x3C		//!< Advertising Timeout
#define BT_HCI_ERR_CONN_TERMINATED_MIC_FAILURE				0x3D		//!< Connection Terminated due to MIC Failure
#define BT_HCI_ERR_CONN_FAILED_SYNC_TIMEOUT					0x3E		//!< Connection Failed to be Established / Synchronization Timeout
//#define BT_HCI_ERR_	0x3F		//!<
#define BT_HCI_ERR_COARSE_CLOCK_REJECTED					0x40		//!< Coarse Clock Adjustment Rejected but Will Try to Adjust Using Clock Dragging
#define BT_HCI_ERR_TYPE0_SUBMAP_NOT_DEFINED					0x41		//!< Type0 Submap Not Defined
#define BT_HCI_ERR_UNKNOWN_ADV_ID							0x42		//!< Unknown Advertising Identifier
#define BT_HCI_ERR_LIMIT_REACHED							0x43		//!< Limit Reached
#define BT_HCI_ERR_OPERATION_CANCELED_HOST					0x44		//!< Operation Cancelled by Host
#define BT_HCI_ERR_PACKET_TOO_LONG							0x45		//!< Packet Too Long

#ifndef BT_HCI_BUFFER_MAX_SIZE
#define BT_HCI_BUFFER_MAX_SIZE				260
#endif

typedef uint8_t	BtHciErr_t;

#pragma pack(push, 1)


/// HCI Command packet header
typedef struct __Bt_Hci_Cmd_Packet_Header {
	union {
		uint16_t OpCode;		//!< HCI OpCode
		struct {
			uint16_t Ocf:10;
			uint16_t Ogf:6;
		};
	};
	uint8_t ParamLen;			//!< HCI parameter length in bytes
} BtHciCmdPacketHdr_t;

/// HCI Command packet
/// NOTE: This structure is variable length
typedef struct __Bt_Hci_Cmd_Packet {
	BtHciCmdPacketHdr_t Hdr;	//!< Command packet header
	uint8_t Param[1];
} BtHciCmd_Packet_t;

#define BT_HCI_PBFLAG_START_NONFLUSHABLE		0
#define BT_HCI_PBFLAG_CONTINUING_FRAGMENT		1
#define BT_HCI_PBFLAG_START_FLUSHABLE			2
#define BT_HCI_PBFLAG_COMPLETE_L2CAP_PDU		3

#define BT_HCI_BCFLAG_POINT_TO_POINT			0
#define BT_HCI_BCFLAG_BR_EDR_BROADCAST			1


/// HCI ACL data packet header
typedef struct __Bt_Hci_ACL_Data_Packet_Header {
	uint32_t ConnHdl:12;		//!< Connection Handle
	uint32_t PBFlag:2;			//!< Packet boundary flag
	uint32_t BCFlag:2;			//!< Broadcast flag
	uint32_t Len:16;			//!< Data length in bytes
} BtHciACLDataPacketHdr_t;

/// HCI ACL data packet
/// NOTE: This structure is variable length
typedef struct __Bt_Hci_ACL_Data_Packet {
	BtHciACLDataPacketHdr_t Hdr;	//!< Data packet header
	uint8_t Data[1];
} BtHciACLDataPacket_t;

/// HCI Synchronous data packet header
typedef struct __Bt_Hci_Sync_Data_Packet_Header {
	uint32_t ConnHdl:12;		//!< Connection Handle
	uint32_t Status:2;			//!< Status flag
	uint32_t Reserved:2;		//!<
	uint32_t Len:8;				//!< Data length in bytes
} BtHciSyncDataPacketHdr_t;

/// HCI Synchronous data packet
/// NOTE: This structure is variable length
typedef struct __Bt_Hci_Sync_Data_Packet {
	BtHciSyncDataPacketHdr_t Hdr;
	uint8_t Data[1];
} BtHciSyncDataPacket_t;

/// HCI ISO data packet header
typedef struct __Bt_Hci_ISO_Data_Packet_Header {
	uint32_t ConnHdl:12;		//!< Connection handle
	uint32_t PBFlag:2;			//!< Packet boundary flag
	uint32_t TSFlag:1;			//!< Broadcast flag
	uint32_t Reserved:1;		//!<
	uint32_t Len:14;			//!< Data length in bytes
	uint32_t Reserved1:2;
} BtHciISODataPacketHdr_t;

/// HCI ISO data packet
/// NOTE: This structure is variable length
typedef struct __Bt_Hci_ISO_Data_Packet {
	BtHciISODataPacketHdr_t Hdr;
	uint8_t Data[1];
} BtHciISODataPacket_t;

/// HCI ISO data load
/// NOTE: This structure is variable length
typedef struct __Bt_Hci_ISO_Data_Load {
	uint32_t Timestamp;			//!< Timestamp in usec
	uint16_t SeqNo;				//!< Sequence number
	uint32_t Len:12;			//!< SDU length
	uint32_t Reserved:2;
	uint32_t Status:2;
	uint8_t SduFrag[1];			//!< ISO SDU fragment
} BtHciISODataLoad_t;

#pragma pack(pop)

typedef struct __Bt_Hci_Device		BtHciDevice_t;

#if 0

typedef uint32_t (*BtHciSendDataFct_t)(void *pData, uint32_t Len);
typedef void (*BtEvtHandler_t)(BtHciDevice_t * const pDev, uint32_t Evt);
typedef void (*BtEvtConnected_t)(uint16_t ConnHdl, uint8_t Role, uint8_t AddrType, uint8_t PerrAddr[6]);

/*
typedef struct __Bt_Hci_Dev_Config {
	BtHciSendDataFct_t SendData;
	BtEvtHandler_t EvtHandler;
	BtEvtConnected_t ConnectedHandler;
} BtHciDevCfg_t;
*/

#endif

struct __Bt_Hci_Device {
	void *pCtx;
	uint32_t RxDataLen;
	uint32_t TxDataLen;
	uint32_t (*SendData)(void *pData, uint32_t Len);
	void (*EvtHandler)(BtHciDevice_t * const pDev, uint32_t Evt);
	void (*Connected)(uint16_t ConnHdl, uint8_t Role, uint8_t AddrType, uint8_t PerrAddr[6]);
	void (*Disconnected)(uint16_t ConnHdl, uint8_t Reason);
	void (*SendCompleted)(uint16_t ConnHdl, uint16_t NbPktSent);
};

#ifdef __cplusplus
extern "C" {
#endif
/*
#define USEC_TO_1250(Val)	((uint16_t)(((Val) + 500UL) / 1250UL))
#define MSEC_TO_1_25(Val)	((uint16_t)((Val) / 1.250F))

static inline uint16_t uSecTo1250(uint32_t Val) {
	return (uint16_t)(Val / 1250UL);
};

static inline uint16_t mSecTo1_25(float Val) {
	return (uint16_t)(Val / 1.250F);
};
*/

//bool BtHciInit(BtHciDevCfg_t const *pCfg);
void BtHciProcessData(BtDev_t * const pDev, BtHciACLDataPacket_t * const pPkt);

static inline int BtHciSendData(BtDev_t * const pDev, void * const pData, int Len) {
	return pDev->SendData(pData, Len);
}
void BtHciNotify(BtDev_t * const pDev, uint16_t ConnHdl, uint16_t ValHdl, void * const pData, size_t Len);


#ifdef __cplusplus
}
#endif

/** @} */

#endif // __BT_HCI_H__
