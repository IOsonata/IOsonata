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

// Link Control Command
// OGF = 1
#define BLE_STDHCI_CMD_LINKCTRL			1

#define BLE_STDHCI_CMD_LINKCTRL_INQUIRY						((1<<10) | 1)		//!< Enter Inquiry mode where it discovers other Bluetooth devices.
#define BLE_STDHCI_CMD_LINKCTRL_INQUIRY_CANCEL				((1<<10) | 2)		//!< Cancel the Inquiry mode in which the Bluetooth device is in.
#define BLE_STDHCI_CMD_LINKCTRL_PERIODIC_INQUIRY_MODE		((1<<10) | 3)		//!< Set the device to enter Inquiry modes periodically according to the time interval set.
#define BLE_STDHCI_CMD_LINKCTRL_EXIT_PERIODIC_INQUIRY_MODE	((1<<10) | 4)		//!< Exit the periodic Inquiry mode
#define BLE_STDHCI_CMD_LINKCTRL_CREATE_CONNECTION			((1<<10) | 5)		//!< Create an ACL connection to the device specified by the BD_ADDR in the parameters.
#define BLE_STDHCI_CMD_LINKCTRL_DISCONNECT					((1<<10) | 6)		//!< Terminate the existing connection to a device
#define BLE_STDHCI_CMD_LINKCTRL_ADD_SCO_CONNECTION			((1<<10) | 7)		//!< Create an SCO connection defined by the connection handle parameters
#define BLE_STDHCI_CMD_LINKCTRL_ACCEPT_CONNECTION_REQUEST	((1<<10) | 9)		//!< Accept a new connection request
#define BLE_STDHCI_CMD_LINKCTRL_REJECT_CONNECTION_REQUEST	((1<<10) | 0xA)		//!< Reject a new connection request
#define BLE_STDHCI_CMD_LINKCTRL_KEY_REQUEST_REPLY			((1<<10) | 0xB)		//!< Key request event sent from controller to the host
#define BLE_STDHCI_CMD_LINKCTRL_KEY_REQUEST_NEG_REPLY		((1<<10) | 0xC)		//!< Key request event from the controller to the host if there is no link key associated with the connection.
#define BLE_STDHCI_CMD_LINKCTRL_PINCODE_REQUEST_REPLY		((1<<10) | 0xD)		//!< PIN code request event sent from a controller to the host.
#define BLE_STDHCI_CMD_LINKCTRL_PINCODE_REQUEST_NEG_REPLY	((1<<10) | 0xE)		//!< PIN code request event sent from the controller to the host if there is no PIN associated with the connection.
#define BLE_STDHCI_CMD_LINKCTRL_CHANGE_CONN_PACKET_TYPE		((1<<10) | 0xF)		//!< Change the type of packets to be sent for an existing connection.
#define BLE_STDHCI_CMD_LINKCTRL_AUTHEN_REQUEST				((1<<10) | 0x11)	//!< Establish authentication between two devices specified by the connection handle.
#define BLE_STDHCI_CMD_LINKCTRL_SET_CONN_ENCRYPTION			((1<<10) | 0x13)	//!< Enable or disable the link level encryption.
#define BLE_STDHCI_CMD_LINKCTRL_CHANGE_CONN_LINK_KEY		((1<<10) | 0x15)	//!< Force the change of a link key to a new one between two connected devices.
#define BLE_STDHCI_CMD_LINKCTRL_MASTER_LINK_KEY				((1<<10) | 0x17)	//!< Force two devices to use the master's link key temporarily.
#define BLE_STDHCI_CMD_LINKCTRL_REMOTE_NAME_REQUEST			((1<<10) | 0x19)	//!< Determine the user friendly name of the connected device.
#define BLE_STDHCI_CMD_LINKCTRL_READ_REMOTE_SUPP_FEATURES	((1<<10) | 0x1B)	//!< Determine the features supported by the connected device.
#define BLE_STDHCI_CMD_LINKCTRL_READ_REMOTE_VERS_INFO		((1<<10) | 0x1D)	//!< Determine the version information of the connected device.
#define BLE_STDHCI_CMD_LINKCTRL_READ_CLOCK_OFFSET			((1<<10) | 0x1F)	//!< read the clock offset of the remote device.

// HCI Policy Command
// OGF = 2
#define BLE_STDHCI_CMD_POLICY			2

#define BLE_STDHCI_CMD_POLICY_HOLD_MODE						((2<<10) | 1)		//!< Place the current or remote device into the Hold mode state.
#define BLE_STDHCI_CMD_POLICY_SNIFF_MODE					((2<<10) | 3)		//!< Place the current or remote device into the Sniff mode state.
#define BLE_STDHCI_CMD_POLICY_EXIT_SNIFF_MODE				((2<<10) | 4)		//!< Exit the current or remote device from the Sniff mode state.
#define BLE_STDHCI_CMD_POLICY_PARK_MODE						((2<<10) | 5)		//!< Place the current or remote device into the Park mode state.
#define BLE_STDHCI_CMD_POLICY_EXIT_PARK_MODE				((2<<10) | 6)		//!< Exit the current or remote device from the Park mode state.
#define BLE_STDHCI_CMD_POLICY_QOS_SETUP						((2<<10) | 7)		//!< Setup the Quality of Service parameters of the device.
#define BLE_STDHCI_CMD_POLICY_ROLE_DISCOVERY				((2<<10) | 9)		//!< Determine the role of the device for a particular connection.
#define BLE_STDHCI_CMD_POLICY_SWITCH_ROLE					((2<<10) | 0xB)		//!< Allow the device to switch roles for a particular connection.
#define BLE_STDHCI_CMD_POLICY_READ_LINK_POLICY_SETTINGS		((2<<10) | 0xC)		//!< Determine the link policy that the LM can use to establish connections.
#define BLE_STDHCI_CMD_POLICY_WRITE_LINK_POLICY_SETTINGS	((2<<10) | 0xD)		//!< Set the link policy that the LM can use for a particular connection.


// Host Controller and Baseband Command
// OGF = 3
#define BLE_STDHCI_CMD_BASEBAND			3

#define BLE_STDHCI_CMD_BASEBAND_SET_EVENT_MASK				((3<<10) | 1)		//!< Set which events are generated by the HCI for the host.
#define BLE_STDHCI_CMD_BASEBAND_RESET						((3<<10) | 3)		//!< Reset the host controller, link manager and the radio module.
#define BLE_STDHCI_CMD_BASEBAND_SET_EVENT_FILTER			((3<<10) | 5)		//!< Set the different types of event filters that the host needs to receive.
#define BLE_STDHCI_CMD_BASEBAND_FLUSH						((3<<10) | 8)		//!< Flush all pending data packets for transmission for a particular connection handle.
#define BLE_STDHCI_CMD_BASEBAND_READ_PIN_TYPE				((3<<10) | 9)		//!< Determine if the link manager assumes that the host requires a variable PIN type or fixed PIN code. PIN is used during pairing.
#define BLE_STDHCI_CMD_BASEBAND_WRITE_PIN_TYPE				((3<<10) | 0xA)		//!< Write to the host controller on the PIN type supported by the host.
#define BLE_STDHCI_CMD_BASEBAND_CREATE_NEW_UNIT_KEY			((3<<10) | 0xB)		//!< Create a new unit key.
#define BLE_STDHCI_CMD_BASEBAND_READ_LINK_KEY				((3<<10) | 0xD)		//!< Read the link key stored in the host controller.
#define BLE_STDHCI_CMD_BASEBAND_WRITE_LINK_KEY				((3<<10) | 0x11)	//!< Write the link key to the host controller.
#define BLE_STDHCI_CMD_BASEBAND_DELETE_LINK_KEY				((3<<10) | 0x12)	//!< Delete a stored link key in the host controller.
#define BLE_STDHCI_CMD_BASEBAND_CHANGE_LOCAL_NAME			((3<<10) | 0x13)	//!< Modify the user friendly name of the device.
#define BLE_STDHCI_CMD_BASEBAND_READ_LOCAL_NAME				((3<<10) | 0x14)	//!< Read the user friendly name of the device.
#define BLE_STDHCI_CMD_BASEBAND_READ_CONN_ACCEPT_TIMEOUT	((3<<10) | 0x15)	//!< Determine the timeout session before the host denies and rejects a new connection request.
#define BLE_STDHCI_CMD_BASEBAND_WRITE_CONN_ACCEPT_TIMEOPUT	((3<<10) | 0x16)	//!< Set the timeout session before a device can deny or reject a connection request.
#define BLE_STDHCI_CMD_BASEBAND_READ_PAGE_TIMEOUT			((3<<10) | 0x17)	//!< Read the timeout value where a device will wait for a connection acceptance before sending a connection failure is returned.
#define BLE_STDHCI_CMD_BASEBAND_WRITE_PAGE_TIMEOUT			((3<<10) | 0x18)	//!< Write the timeout value where a device will wait for a connection acceptance before sending a connection failure is returned.
#define BLE_STDHCI_CMD_BASEBAND_READ_SCAN_ENABLE			((3<<10) | 0x19)	//!< Read the status of the Scan_Enable configuration.
#define BLE_STDHCI_CMD_BASEBAND_WRITE_SCAN_ENABLE			((3<<10) | 0x1A)	//!< Set the status of the Scan_Enable configuration.
#define BLE_STDHCI_CMD_BASEBAND_READ_PAGE_SCAN_ACTIVITY		((3<<10) | 0x1B)	//!< Read the value of the Page_Scan_Interval and Page_Scan_Window configurations.
#define BLE_STDHCI_CMD_BASEBAND_WRITE_PAGE_SCAN_ACTIVITY	((3<<10) | 0x1C)	//!< Write the value of the Page_Scan_Interval and Page_Scan_Window configurations.
#define BLE_STDHCI_CMD_BASEBAND_READ_INQUIRY_SCAN_ACTIVITY	((3<<10) | 0x1D)	//!< Read the value of the Inquiry_Scan_Interval and Inquiry_Scan_Window configurations.
#define BLE_STDHCI_CMD_BASEBAND_WRITE_INQUIRY_SCAN_ACTIVITY	((3<<10) | 0x1E)	//!< Set the value of the Inquiry_Scan_Interval and Inquiry_Scan_Window configurations.
#define BLE_STDHCI_CMD_BASEBAND_READ_AUTHEN_ENABLE			((3<<10) | 0x1F)	//!< Read the Authentication_Enable parameter.
#define BLE_STDHCI_CMD_BASEBAND_WRITE_AUTHEN_ENABLE			((3<<10) | 0x20)	//!< Set the Authentication_Enable parameter.
#define BLE_STDHCI_CMD_BASEBAND_READ_ENCRYPTION_MODE		((3<<10) | 0x21)	//!< Read the Encryption_Mode parameter.
#define BLE_STDHCI_CMD_BASEBAND_WRITE_ENCRYPTION_MODE		((3<<10) | 0x22)	//!< Write the Encryption_Mode parameter.
#define BLE_STDHCI_CMD_BASEBAND_READ_CLASS_OF_DEVICE		((3<<10) | 0x23)	//!< Read the Class_Of_Device parameter.
#define BLE_STDHCI_CMD_BASEBAND_WRITE_CLASS_OF_DEVICE		((3<<10) | 0x24)	//!< Set the Class_Of_Device parameter.
#define BLE_STDHCI_CMD_BASEBAND_READ_VOICE_SETTINGS			((3<<10) | 0x25)	//!< Read the Voice_Setting parameter. Used for voice connections.
#define BLE_STDHCI_CMD_BASEBAND_WRITE_VOICE_SETTINGS		((3<<10) | 0x26)	//!< Set the Voice_Setting parameter. Used for voice connections.
#define BLE_STDHCI_CMD_BASEBAND_READ_AUTO_FLUSH_TIMEOUT		((3<<10) | 0x27)	//!< Read the Flush_Timeout parameter. Used for ACL connections only.
#define BLE_STDHCI_CMD_BASEBAND_WRITE_AUTO_FLUSH_TIMEOUT	((3<<10) | 0x28)	//!< Set the Flush_Timeout parameter. Used for ACL connections only.
#define BLE_STDHCI_CMD_BASEBAND_READ_NB_BROADCAST_RETRANS	((3<<10) | 0x29)	//!< Read the number of time a broadcast message is retransmitted.
#define BLE_STDHCI_CMD_BASEBAND_WRITE_NB_BROADCAST_RETRANS	((3<<10) | 0x2A)	//!< Set the number of time a broadcast message is retransmitted.
#define BLE_STDHCI_CMD_BASEBAND_READ_HOLD_MODE_ACTIVITY		((3<<10) | 0x2B)	//!< Read the Hold_Mode_Activity parameter.
#define BLE_STDHCI_CMD_BASEBAND_WRITE_HOLD_MODE_ACTIVITY	((3<<10) | 0x2C)	//!< Set the Hold_Mode_Activity parameter.
#define BLE_STDHCI_CMD_BASEBAND_READ_TRANSMIT_POWER_LEVEL	((3<<10) | 0x2D)	//!< Read the power level required for transmission for a connection handle.
#define BLE_STDHCI_CMD_BASEBAND_READ_SCO_FLOWCTRL_ENABLE	((3<<10) | 0x2E)	//!< Check the current status of the flow control for the SCO connection.
#define BLE_STDHCI_CMD_BASEBAND_WRITE_SCO_FLOWCTRL_ENABLE	((3<<10) | 0x2F)	//!< Set the status of the flow control for a connection handle.
#define BLE_STDHCI_CMD_BASEBAND_SET_HOST_FLOWCTRL			((3<<10) | 0x31)	//!< Set the flow control from the host controller to host in on or off state.
#define BLE_STDHCI_CMD_BASEBAND_HOST_BUFFER_SIZE			((3<<10) | 0x33)	//!< Inform the host controller of the buffer size of the host for ACL and SCO connections.
#define BLE_STDHCI_CMD_BASEBAND_HOST_NB_COMPLETE_PACKET		((3<<10) | 0x35)	//!< Host controller when it is ready to receive more data packets.
#define BLE_STDHCI_CMD_BASEBAND_READ_LINK_SUPERV_TIMEOUT	((3<<10) | 0x36)	//!< Read the timeout for monitoring link losses.
#define BLE_STDHCI_CMD_BASEBAND_WRITE_LINK_SUPERV_TIMEOUT	((3<<10) | 0x37)	//!< Set the timeout for monitoring link losses.
#define BLE_STDHCI_CMD_BASEBAND_READ_NB_SUPPORTED_IAC		((3<<10) | 0x38)	//!< Read the number of IACs that the device can listen on during Inquiry access.
#define BLE_STDHCI_CMD_BASEBAND_READ_CURRENT_IAC_LAP		((3<<10) | 0x39)	//!< Read the LAP for the current IAC.
#define BLE_STDHCI_CMD_BASEBAND_WRITE_CURRENT_IAC_LAP		((3<<10) | 0x3A)	//!< Set the LAP for the current IAC.
#define BLE_STDHCI_CMD_BASEBAND_READ_PAGE_SCAN_PERIOD_MODE	((3<<10) | 0x3B)	//!< Read the timeout session of a page scan.
#define BLE_STDHCI_CMD_BASEBAND_WRITE_PAGE_SCAN_PERIOD_MODE	((3<<10) | 0x3C)	//!< Set the timeout session of a page scan.
#define BLE_STDHCI_CMD_BASEBAND_READ_PAGE_SCAN_MODE			((3<<10) | 0x3D)	//!< Read the default Page scan mode.
#define BLE_STDHCI_CMD_BASEBAND_WRITE_PAGE_SCAN_MODE		((3<<10) | 0x3D)	//!< Set the default page scan mode.

// HCI events
#define BLE_STDHCI_EVT_INQUERY_COMPLETE						1	//!< Indicates the Inquiry has finished
#define BLE_STDHCI_EVT_INQUERY_RESULT						2	//!< Indicates that Bluetooth device(s) have responded for the inquiry.
#define BLE_STDHCI_EVT_CONN_COMPLETE						3	//!< Indicates to both hosts that the new connection has been formed.
#define BLE_STDHCI_EVT_CONN_REQUEST							4	//!< Indicates that a new connection is trying to be established
#define BLE_STDHCI_EVT_DISCONN_COMPLETE						5	//!< Occurs when a connection has been disconnected.
#define BLE_STDHCI_EVT_AUTHEN_COMPLETE						6	//!< Occurs when an authentication has been completed.
#define BLE_STDHCI_EVT_REMOTE_NAME_REQUEST_COMPLETE			7	//!< Indicates that the request for the remote name has been completed.
#define BLE_STDHCI_EVT_ENCRYPTION_CHANGE					8	//!< Indicates that a change in the encryption has been completed.
#define BLE_STDHCI_EVT_CHANGE_CONN_LINK_KEY_COMPLETE		9	//!< Indicates that the change in the link key has been completed.
#define BLE_STDHCI_EVT_MASTER_LINK_KEY_COMPLETE				0xA	//!< Indicates that the change in the temporary link key or semi permanent link key on the master device is complete.
#define BLE_STDHCI_EVT_READ_REMOTE_SUPP_FEATURES_COMPLETE	0xB	//!< Indicates that the reading of the supported features on the remote device is complete.
#define BLE_STDHCI_EVT_READ_REMOTE_VERS_COMPLETE			0xC	//!< Indicates that the version number on the remote device has been read and completed.
#define BLE_STDHCI_EVT_QOS_SETTUP_COMPLETE					0xD	//!< Indicates that the Quality of Service setup has been complete.
#define BLE_STDHCI_EVT_COMMAND_COMPLETE						0xE	//!< Used by controller to send status and event parameters to the host for the particular command.
#define BLE_STDHCI_EVT_COMMAND_STATUS						0xF	//!< Indicates that the command has been received and is being processed in the host controller.
#define BLE_STDHCI_EVT_HARDWARE_ERROR						0x10	//!< Indicates a hardware failure of the Bluetooth device.
#define BLE_STDHCI_EVT_FLUSH_OCCURED						0x11	//!< Indicates that the data has been flushed for a particular connection.
#define BLE_STDHCI_EVT_ROLE_CHANGED							0x12	//!< Indicates that the current bluetooth role for a connection has been changed.
#define BLE_STDHCI_EVT_NB_COMPLETED_PACKET					0x13	//!< Indicates to the host the number of data packets sent compared to the last time the same event was sent.
#define BLE_STDHCI_EVT_MODE_CHANGED							0x14	//!< Indicates the change in mode from hold, sniff, park or active to another mode.
#define BLE_STDHCI_EVT_RETURN_LINK_KEYS						0x15	//!< Used to return stored link keys after a Read_Stored_Link_Key command was issued.
#define BLE_STDHCI_EVT_PIN_CODE_REQUEST						0x16	//!< Indicates the a PIN code is required for a new connection.
#define BLE_STDHCI_EVT_LINK_KEY_REQUEST						0x17	//!< Indicates that a link key is required for the connection.
#define BLE_STDHCI_EVT_LINK_KEY_NOTIFICATION				0x18	//!< Indicates to the host that a new link key has been created.
#define BLE_STDHCI_EVT_LOOPBACK_COMMAND						0x19	//!< Indicates that command sent from the host will be looped back.
#define BLE_STDHCI_EVT_DATA_BUFFER_OVERFLOW					0x1A	//!< Indicates that the data buffers on the host has overflowed.
#define BLE_STDHCI_EVT_MAX_SLOT_CHANGED						0x1B	//!< Informs the host when the LMP_Max_Slots parameter changes.
#define BLE_STDHCI_EVT_READ_CLOCK_OFFSET_COMPLETE			0x1C	//!< Indicates the completion of reading the clock offset information.
#define BLE_STDHCI_EVT_CONN_PACKET_TYPE_CHANGED				0x1D	//!< Indicate the completion of the packet type change for a connection.
#define BLE_STDHCI_EVT_QOS_VIOLATION						0x1E	//!< Indicates that the link manager is unable to provide the required Quality of Service.
#define BLE_STDHCI_EVT_PAGE_SCAN_MODE_CHANGED				0x1F	//!< Indicates that the remote device has successfully changed the Page Scan mode.
#define BLE_STDHCI_EVT_PAGE_SCAN_REPETITION_MODE_CHANGED	0x20	//!< Indicates that the remote device has successfully changed the Page Scan Repetition mode.

// HCI error codes
#define BLE_STDHCI_ERR_UNKNOWN_COMMAND						1
#define BLE_STDHCI_ERR_NO_CONNECTION						2
#define BLE_STDHCI_ERR_HARDWARE_FAILURE						3
#define BLE_STDHCI_ERR_PAGE_TIMEOUT							4
#define BLE_STDHCI_ERR_AUTHEN_FAILURE						5
#define BLE_STDHCI_ERR_KEY_MISSING							6
#define BLE_STDHCI_ERR_MEMORY_FULL							7
#define BLE_STDHCI_ERR_CONN_TIMEOUT							8
#define BLE_STDHCI_ERR_MAX_NB_CONN							9
#define BLE_STDHCI_ERR_MAX_NB_SCO_CONN						0xA
#define BLE_STDHCI_ERR_ACL_CONN_EXISTS						0xB
#define BLE_STDHCI_ERR_COMMAND_DISALLOWED					0xC
#define BLE_STDHCI_ERR_HOST_REJECT_RESOURCE_LIMIT			0xD
#define BLE_STDHCI_ERR_HOST_REJECT_SECURITY					0xE
#define BLE_STDHCI_ERR_HOST_REJECT_PERSONAL_DEVICE			0xF
#define BLE_STDHCI_ERR_HOST_TIMEOUT							0x10
#define BLE_STDHCI_ERR_UNSUPPORTED_FEATURE_PARAM			0x11
#define BLE_STDHCI_ERR_INVALID_COMMAND_PARAM				0x12
#define BLE_STDHCI_ERR_CONN_TERMINATED_USER					0x13
#define BLE_STDHCI_ERR_CONN_TERMINATED_LOW_RESOURCE			0x14
#define BLE_STDHCI_ERR_CONN_TERMINATED_POWER_OFF			0x15
#define BLE_STDHCI_ERR_CONN_TERMINATED_LOCAL_HOST			0x16
#define BLE_STDHCI_ERR_REPEATED_ATTEMPTS					0x17
#define BLE_STDHCI_ERR_PAIRING_NOT_ALLOWED					0x18
#define BLE_STDHCI_ERR_UNKNOWN_LMP_PDU						0x19
#define BLE_STDHCI_ERR_UNSUPPORTED_REMOTE_FEATURE			0x1A
#define BLE_STDHCI_ERR_SCO_OFFSET_REJECTED					0x1B
#define BLE_STDHCI_ERR_SCO_INTERVAL_REJECTED				0x1C
#define BLE_STDHCI_ERR_SCO_AIR_MODE_REJECTED				0x1D
#define BLE_STDHCI_ERR_INVALID_LMP_PARAM					0x1E
#define BLE_STDHCI_ERR_UNSPECIFIED							0x1F
#define BLE_STDHCI_ERR_UNSUPPORTED_LMP_PARAM				0x20
#define BLE_STDHCI_ERR_ROLE_CHANGE_NOT_ALLOWED				0x21
#define BLE_STDHCI_ERR_LMP_RESPONSE_TIMEOUT					0x22
#define BLE_STDHCI_ERR_LMP_TRANSACTION_COLLISION			0x23
#define BLE_STDHCI_ERR_LMP_PDU_NOT_ALLOWED					0x24
#define BLE_STDHCI_ERR_ENCRYPTION_MODE_NOT_ACCEPTABLE		0x25
#define BLE_STDHCI_ERR_UNIT_KEY_USED						0x26
#define BLE_STDHCI_ERR_QOS_NOT_SUPPORTED					0x27
#define BLE_STDHCI_ERR_INSTANT_PASSED						0x28
#define BLE_STDHCI_ERR_PAIRING_WITH_UNIT_KEY_NOT_SUPPORTED	0x29



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
