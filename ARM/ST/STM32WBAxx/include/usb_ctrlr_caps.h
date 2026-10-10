/* STM32WBA65 USB controller capabilities.
 * STM32 targets with distinct USB hardware must provide their own
 * usb_ctrlr_caps.h rather than changing the shared USB stack API.
 */
#ifndef __STM32WBA65_USB_CTRLR_CAPS_H__
#define __STM32WBA65_USB_CTRLR_CAPS_H__
typedef enum __Usb_Ctrlr_Trans_Type {
	CONTROL = USB_ENDPATT_TRANS_CONTROL,
	ISO = USB_ENDPATT_TRANS_ISO,
	BULK = USB_ENDPATT_TRANS_BULK,
	INT = USB_ENDPATT_TRANS_INT,
} UsbCtrlrTransType_t;

// Nine bidirectional endpoint numbers, including EP0. HS integrated PHY.
enum {
	USB_CTRLR_CNT = 1,
	USB_HIGHSPEED_CAPABLE_0 = 1,
	USB_EPIN_CNT_0 = 9,
	USB_EPOUT_CNT_0 = 9,
	USB_CTRLR0_CONTROL_PKT_LEN_MAX = 64,
	USB_CTRLR0_BULK_PKT_LEN_MAX = 512,
	USB_CTRLR0_INT_PKT_LEN_MAX = 1024,
	USB_CTRLR0_ISO_PKT_LEN_MAX = 1024,
	USB_ISO_SUPPORTED_0 = 1,
	USB_ISO_EPIN_MASK_0 = 0x01FEU,
	USB_ISO_EPOUT_MASK_0 = 0x01FEU,
};

#ifndef USB_CONFIG_DESC_MAXLEN
#define USB_CONFIG_DESC_MAXLEN		768U
#endif

#define USB_EPIN_CNT(CtrlrNo) \
	((CtrlrNo) == 0 ? USB_EPIN_CNT_0 : 0)
#define USB_EPOUT_CNT(CtrlrNo) \
	((CtrlrNo) == 0 ? USB_EPOUT_CNT_0 : 0)
#define USB_HIGHSPEED_CAPABLE(CtrlrNo) \
	((CtrlrNo) == 0 ? USB_HIGHSPEED_CAPABLE_0 : 0)
#define USB_ISO_SUPPORTED(CtrlrNo) \
	((CtrlrNo) == 0 ? USB_ISO_SUPPORTED_0 : 0)
#define USB_ISO_EPIN_MASK(CtrlrNo) \
	((CtrlrNo) == 0 ? (uint32_t)USB_ISO_EPIN_MASK_0 : 0U)
#define USB_ISO_EPOUT_MASK(CtrlrNo) \
	((CtrlrNo) == 0 ? (uint32_t)USB_ISO_EPOUT_MASK_0 : 0U)
#define USB_CTRLR_PKT_LEN_MAX(CtrlrNo, TransType) \
	((CtrlrNo) != 0 ? 0 : \
	 (TransType) == CONTROL ? USB_CTRLR0_CONTROL_PKT_LEN_MAX : \
	 (TransType) == ISO ? USB_CTRLR0_ISO_PKT_LEN_MAX : \
	 (TransType) == BULK ? USB_CTRLR0_BULK_PKT_LEN_MAX : \
	 (TransType) == INT ? USB_CTRLR0_INT_PKT_LEN_MAX : 0)

#define USB_CTRLR_ISO_INIT(DevNo) UsbCtrlrIsoInit(DevNo)



#endif
