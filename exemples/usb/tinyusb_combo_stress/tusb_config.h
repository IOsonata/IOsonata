#ifndef TUSB_CONFIG_H_
#define TUSB_CONFIG_H_

#define CFG_TUSB_MCU				OPT_MCU_NRF5X
#define CFG_TUSB_OS					OPT_OS_NONE
#define CFG_TUSB_DEBUG				0

#define CFG_TUSB_RHPORT0_MODE		(OPT_MODE_DEVICE | OPT_MODE_FULL_SPEED)
#define CFG_TUD_ENABLED				1
#define CFG_TUH_ENABLED				0
#define CFG_TUD_MAX_SPEED			OPT_MODE_FULL_SPEED

#define CFG_TUSB_MEM_SECTION
#define CFG_TUSB_MEM_ALIGN			__attribute__((aligned(4)))

#define CFG_TUD_ENDPOINT0_SIZE		64

#define CFG_TUD_CDC					2
#define CFG_TUD_HID					1
#define CFG_TUD_VENDOR				1
#define CFG_TUD_MSC					0
#define CFG_TUD_MIDI				0

#define CFG_TUD_CDC_NOTIFY			1
#define CFG_TUD_CDC_RX_BUFSIZE		256
#define CFG_TUD_CDC_TX_BUFSIZE		2048
#define CFG_TUD_CDC_RX_EPSIZE		64
#define CFG_TUD_CDC_TX_EPSIZE		64

#define CFG_TUD_HID_EP_BUFSIZE		64

// Raw interrupt interface. Alternate settings require direct/non-buffered mode.
#define CFG_TUD_VENDOR_RX_BUFSIZE	0
#define CFG_TUD_VENDOR_TX_BUFSIZE	0
#define CFG_TUD_VENDOR_ALT_SETTINGS	1
#define CFG_TUD_VENDOR_EP_INT_OUT	1
#define CFG_TUD_VENDOR_EP_INT_IN	1
#define CFG_TUD_VENDOR_EP_INT_OUT_BUFSIZE	64
#define CFG_TUD_VENDOR_EP_INT_IN_BUFSIZE	64

// EP8 ISO is handled by the application's tiny class driver so the same
// endpoint address can change MPS across alt 1..6.
#define CFG_TUD_VENDOR_EP_ISO_OUT	0
#define CFG_TUD_VENDOR_EP_ISO_IN	0

#endif	// TUSB_CONFIG_H_
