/*
 * usbd_hid.cpp
 *
 *  Created on: Nov 22, 2024
 *      Author: hoan
 */

#include "nrf.h"
#include "app_util_platform.h"
#include "nrf_drv_usbd.h"
#include "nrf_drv_clock.h"
#include "nrf_gpio.h"
#include "nrf_drv_power.h"

#include "app_usbd.h"
#include "app_usbd_core.h"
#include "app_usbd_string_desc.h"
#include "app_usbd_cdc_acm.h"
#include "app_usbd_serial_num.h"
#include "generic/app_usbd_hid_generic.h"
#include "mouse/app_usbd_hid_mouse.h"
#include "nrf_cli.h"
#include "nrf_cli_cdc_acm.h"
//#include "app_usbd_hid_kbd.h"
#include "app_error.h"
//#include "bsp.h"

#include "usb/usb_hiddef.h"

#include "SlimeVRDongle.h"

static void usbd_user_ev_handler(app_usbd_event_type_t event);
static void cdc_acm_user_ev_handler(app_usbd_class_inst_t const * p_inst,
                                    app_usbd_cdc_acm_user_event_t event);

#if NRF_CLI_ENABLED
//NRF_CLI_CDC_ACM_DEF(m_cli_cdc_acm_transport);
//#define NRF_CLI_CDC_ACM_DEF(_name_)
static nrf_cli_cdc_acm_internal_cb_t m_cli_cdc_acm_transport_cb;
static const nrf_cli_cdc_acm_internal_t m_cli_cdc_acm_transport = {
	.transport = {.p_api = &nrf_cli_cdc_acm_transport_api},
	.p_cb = &m_cli_cdc_acm_transport_cb,
};

NRF_CLI_DEF(m_cli_cdc_acm,
            "usb_cli:~$ ",
            &m_cli_cdc_acm_transport.transport,
            '\r',
            CLI_EXAMPLE_LOG_QUEUE_SIZE);
#endif


/**
 * @brief Enable USB power detection
 */
#ifndef USBD_POWER_DETECTION
#define USBD_POWER_DETECTION true
#endif

// Nordic CLI uses interface 1 & 2 already
#define USBD_INTERFACE_HID				3		// Interface 3 - HID
#define USBD_EPIN_HID					NRFX_USBD_EPIN3
#define USBD_EPOUT_HID					NRFX_USBD_EPOUT3

#define APP_USBD_CORE_DEVICE_DESCRIPTOR  {                                                               \
   .bLength = sizeof(app_usbd_descriptor_device_t),    /* descriptor size */                             \
   .bDescriptorType = APP_USBD_DESCRIPTOR_DEVICE,      /* descriptor type */                             \
   .bcdUSB = APP_USBD_BCD_VER_MAKE(2,0,0),             /* USB BCD version: 2.0 */                        \
   .bDeviceClass = 0,                                  /* device class: 0 - specified by interface */    \
   .bDeviceSubClass = 0,                               /* device subclass: 0 - specified by interface */ \
   .bDeviceProtocol = 0,                               /* device protocol: 0 - specified by interface */ \
   .bMaxPacketSize0 = NRF_DRV_USBD_EPSIZE,             /* endpoint size: fixed to: NRF_DRV_USBD_EPSIZE*/ \
   .idVendor = APP_USBD_VID,                           /* Vendor ID*/                                    \
   .idProduct = APP_USBD_PID,                          /* Product ID*/                                   \
   .bcdDevice = APP_USBD_BCD_VER_MAKE(                 /* Device version BCD */                          \
       APP_USBD_DEVICE_VER_MAJOR,                                                                        \
       APP_USBD_DEVICE_VER_MINOR,                                                                        \
       APP_USBD_DEVICE_VER_SUB),                                                                         \
   .iManufacturer = APP_USBD_STRING_ID_MANUFACTURER,   /* String ID: manufacturer */                     \
   .iProduct = APP_USBD_STRING_ID_PRODUCT,             /* String ID: product */                          \
   .iSerialNumber = APP_USBD_STRING_ID_SERIAL,         /* String ID: serial */                           \
   .bNumConfigurations = 1                             /* Fixed value: only one configuration supported*/\
}

extern "C"
const app_usbd_descriptor_device_t m_device_dsc =
    APP_USBD_CORE_DEVICE_DESCRIPTOR;

/**
 * @brief Mouse speed (value sent via HID when board button is pressed).
 * */
#define CONFIG_MOUSE_MOVE_SPEED (3)

/**
 * @brief Mouse move repeat time in milliseconds
 */
#define CONFIG_MOUSE_MOVE_TIME_MS (5)


/* GPIO used as LED & buttons in this example */
//#define LED_USB_START    (BSP_BOARD_LED_0)
//#define LED_HID_REP_IN   (BSP_BOARD_LED_2)

#define BTN_MOUSE_X_POS  0
#define BTN_MOUSE_Y_POS  1
#define BTN_MOUSE_LEFT   2
#define BTN_MOUSE_RIGHT  3

/**
 * @brief Left button mask in buttons report
 */
#define HID_BTN_LEFT_MASK  (1U << 0)

/**
 * @brief Right button mask in buttons report
 */
#define HID_BTN_RIGHT_MASK (1U << 1)

/* HID report layout */
#define HID_BTN_IDX   0 /**< Button bit mask position */
#define HID_X_IDX     1 /**< X offset position */
#define HID_Y_IDX     2 /**< Y offset position */
#define HID_W_IDX     3 /**< Wheel position  */
#define HID_REP_SIZE  4 /**< The size of the report */

/**
 * @brief Number of reports defined in report descriptor.
 */
#define REPORT_IN_QUEUE_SIZE    1

/**
 * @brief Size of maximum output report. HID generic class will reserve
 *        this buffer size + 1 memory space.
 *
 * Maximum value of this define is 63 bytes. Library automatically adds
 * one byte for report ID. This means that output report size is limited
 * to 64 bytes.
 */
#define REPORT_OUT_MAXSIZE  0

/**
 * @brief Feature report maximum size. HID generic class will reserve
 *        this buffer size + 1 memory space.
 */
#define REPORT_FEATURE_MAXSIZE  31

/**
 * @brief HID generic class endpoints count.
 * */
#define HID_GENERIC_EP_COUNT  1

/**
 * @brief List of HID generic class endpoints.
 * */
#define ENDPOINT_LIST()                                      \
(                                                            \
		USBD_EPIN_HID                                     \
)
/**
 * @brief HID generic mouse action types
 */
typedef enum {
    HID_GENERIC_MOUSE_X,
    HID_GENERIC_MOUSE_Y,
    HID_GENERIC_MOUSE_BTN_LEFT,
    HID_GENERIC_MOUSE_BTN_RIGHT,
} hid_generic_mouse_action_t;

/**
 * @brief User event handler.
 * */
static void hid_user_ev_handler(app_usbd_class_inst_t const * p_inst,
                                app_usbd_hid_user_event_t event);
#if 0
/**
 * @brief USB HID instance initializer @ref app_usbd_hid_inst_t.
 *
 * @param subclass_dsc          HID subclass descriptors.
 * @param sub_boot              Subclass boot. (@ref app_usbd_hid_subclass_t)
 * @param protocl               HID protocol. (@ref app_usbd_hid_protocol_t)
 * @param report_buff_in        Input report buffer list.
 * @param report_buff_out       Output report buffer.
 * @param report_buff_feature   Feature report buffer.
 * @param user_ev_handler       @ref app_usbd_hid_user_ev_handler_t.
 * @param hid_methods           @ref app_usbd_hid_methods_t.
 * @param ep_list               List of endpoints and intervals
 * */

#define APP_USBD_HID_INST_CONFIG(subclass_dsc,               \
                                 sub_boot,                   \
                                 protocl,                    \
                                 report_buff_in,             \
                                 report_buff_out,            \
                                 report_buff_feature,        \
                                 user_ev_handler,            \
                                 hid_methods,                \
                                 ep_list)                    \
    {                                                        \
        .p_subclass_desc = subclass_dsc,                     \
        .subclass_desc_count = ARRAY_SIZE(subclass_dsc),     \
        .subclass_boot = sub_boot,                           \
        .protocol = protocl,                                 \
        .p_rep_buffer_in = report_buff_in,                   \
        .p_rep_buffer_out = report_buff_out,                 \
        .p_rep_buffer_feature = report_buff_feature,         \
        .p_hid_methods = hid_methods,                        \
        .user_event_handler = user_ev_handler,               \
        .p_ep_interval = ep_list                             \
    }
#endif
/**
 * @brief Reuse HID mouse report descriptor for HID generic class
 */
//APP_USBD_HID_GENERIC_SUBCLASS_REPORT_DESC(mouse_desc,APP_USBD_HID_MOUSE_REPORT_DSC_BUTTON(2));

#if 1
#define SLIMEVR_HID_REPORT_DESC		{\
	0x05, 0x01,      \
	0x09, 0x00,      \
	0xA1, 0x01,      \
	0x09, 0x00,      \
	0x09, 0x00,      \
	0x09, 0x00,      \
	0x09, 0x00,      \
	0x15, 0x00,      \
	0x25, 0xFF,      \
	0x75, 0x08,      \
	0x95, 0x04,      \
	0x81, 0x02,      \
	0x09, 0x00,      \
	0x09, 0x00,      \
	0x09, 0x00,      \
	0x09, 0x00,      \
	0x09, 0x00,      \
	0x09, 0x00,      \
	0x09, 0x00,      \
	0x09, 0x00,      \
	0x15, 0x00,      \
	0x26, 0xFF, 0xFF,\
	0x75, 0x10,      \
	0x95, 0x08,      \
	0x81, 0x02,      \
	0xC0,            \
}
APP_USBD_HID_GENERIC_SUBCLASS_REPORT_DESC(s_SlimeHIDReportDesc, SLIMEVR_HID_REPORT_DESC);
/*
0x05, 0x01,        // Usage Page (Generic Desktop Ctrls)
0x09, 0x00,        // Usage (Undefined)
0xA1, 0x01,        // Collection (Application)
0x09, 0x00,        //   Usage (Undefined)
0x09, 0x00,        //   Usage (Undefined)
0x09, 0x00,        //   Usage (Undefined)
0x09, 0x00,        //   Usage (Undefined)
0x15, 0x00,        //   Logical Minimum (0)
0x25, 0xFF,        //   Logical Maximum (-1)
0x75, 0x08,        //   Report Size (8)
0x95, 0x04,        //   Report Count (4)
0x81, 0x02,        //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
0x09, 0x00,        //   Usage (Undefined)
0x09, 0x00,        //   Usage (Undefined)
0x09, 0x00,        //   Usage (Undefined)
0x09, 0x00,        //   Usage (Undefined)
0x09, 0x00,        //   Usage (Undefined)
0x09, 0x00,        //   Usage (Undefined)
0x09, 0x00,        //   Usage (Undefined)
0x09, 0x00,        //   Usage (Undefined)
0x15, 0x00,        //   Logical Minimum (0)
0x26, 0xFF, 0xFF,  //   Logical Maximum (-1)
0x75, 0x10,        //   Report Size (16)
0x95, 0x08,        //   Report Count (8)
0x81, 0x02,        //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
0xC0,              // End Collection
*/
#else
static const uint8_t s_SlimeHIDReportDesc_data[] = {
	0x05, 0x01,        // Usage Page (Generic Desktop Ctrls)
	0x09, 0x00,        // Usage (Undefined)
	0xA1, 0x01,        // Collection (Application)
	0x09, 0x00,        //   Usage (Undefined)
	0x09, 0x00,        //   Usage (Undefined)
	0x09, 0x00,        //   Usage (Undefined)
	0x09, 0x00,        //   Usage (Undefined)
	0x15, 0x00,        //   Logical Minimum (0)
	0x25, 0xFF,        //   Logical Maximum (-1)
	0x75, 0x08,        //   Report Size (8)
	0x95, 0x04,        //   Report Count (4)
	0x81, 0x02,        //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
	0x09, 0x00,        //   Usage (Undefined)
	0x09, 0x00,        //   Usage (Undefined)
	0x09, 0x00,        //   Usage (Undefined)
	0x09, 0x00,        //   Usage (Undefined)
	0x09, 0x00,        //   Usage (Undefined)
	0x09, 0x00,        //   Usage (Undefined)
	0x09, 0x00,        //   Usage (Undefined)
	0x09, 0x00,        //   Usage (Undefined)
	0x15, 0x00,        //   Logical Minimum (0)
	0x26, 0xFF, 0xFF,  //   Logical Maximum (-1)
	0x75, 0x10,        //   Report Size (16)
	0x95, 0x08,        //   Report Count (8)
	0x81, 0x02,        //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
	0xC0,              // End Collection
};
#endif

static const app_usbd_hid_subclass_desc_t * reps[] = { &s_SlimeHIDReportDesc };
//		sizeof(s_SlimeHIDReportDesc), APP_USBD_DESCRIPTOR_REPORT, s_SlimeHIDReportDesc};

/*lint -save -e26 -e64 -e123 -e505 -e651*/
#if 0
/**
 * @brief Global HID generic instance
 */
APP_USBD_HID_GENERIC_GLOBAL_DEF_INTERNALCPP(m_app_hid_generic,
                                HID_GENERIC_INTERFACE,
                                hid_user_ev_handler,
                                ENDPOINT_LIST(),
                                reps,
                                REPORT_IN_QUEUE_SIZE,
                                REPORT_OUT_MAXSIZE,
                                REPORT_FEATURE_MAXSIZE,
                                APP_USBD_HID_SUBCLASS_BOOT,
                                APP_USBD_HID_PROTO_MOUSE);
#else
#if 0
static app_usbd_hid_report_buffer_t CONCAT_2(m_app_hid_generic, _in);
APP_USBD_HID_GENERIC_GLOBAL_OUT_REP_DEF(CONCAT_2(m_app_hid_generic, _out),
		REPORT_OUT_MAXSIZE + 1);
APP_USBD_HID_GENERIC_GLOBAL_FEATURE_REP_DEF(CONCAT_2(m_app_hid_generic, _feature),
		REPORT_FEATURE_MAXSIZE + 1);
static uint8_t CONCAT_2(m_app_hid_generic, _ep)[]=
    {MACRO_MAP(APP_USBD_HID_GENERIC_INTERVAL,BRACKET_EXTRACT(ENDPOINT_LIST()))};
NRF_QUEUE_DEF(app_usbd_hid_report_buffer_t,
		m_app_hid_generic_queue,
		REPORT_IN_QUEUE_SIZE,
              NRF_QUEUE_MODE_OVERFLOW);
#endif
#if 0
APP_USBD_CLASS_INST_GLOBAL_DEFCPP(
		m_app_hid_generic,
    app_usbd_hid_generic,
    &app_usbd_generic_class_methods,
    APP_USBD_HID_GENERIC_CONFIG(HID_GENERIC_INTERFACE, ENDPOINT_LIST()),
    (APP_USBD_HID_GENERIC_INST_CONFIG(&CONCAT_2(m_app_hid_generic, _in),
                                      &CONCAT_2(m_app_hid_generic, _out),
                                      &CONCAT_2(m_app_hid_generic, _feature),
									  hid_user_ev_handler,
                                      &m_app_hid_generic_queue,
									  reps,
									  APP_USBD_HID_SUBCLASS_BOOT,
									  APP_USBD_HID_PROTO_MOUSE,
                                      CONCAT_2(m_app_hid_generic, _ep)))
);
#endif
#if 0
#define APP_USBD_CLASS_INST_GLOBAL_DEFCPP(instance_name,                           \
                                       type_name,                               \
                                       class_methods,                           \
                                       interfaces_configs,                      \
                                       class_config_part)                       \
    static APP_USBD_CLASS_DATA_TYPE(type_name) CONCAT_2(instance_name, _data);  \
    const APP_USBD_CLASS_INSTANCE_TYPE(type_name) instance_name =               \
        APP_USBD_CLASS_INSTANCE_INITVALCPP(                                        \
            &CONCAT_2(instance_name, _data),                                    \
            class_methods,                                                      \
            interfaces_configs,                                                 \
            class_config_part)
#endif

#define INTERFACE_CONFIGS APP_USBD_HID_GENERIC_CONFIG(USBD_INTERFACE_HID, ENDPOINT_LIST())


//static APP_USBD_CLASS_DATA_TYPE(app_usbd_hid_generic) CONCAT_2(m_app_hid_generic, _data);
//const APP_USBD_CLASS_INSTANCE_TYPE(app_usbd_hid_generic) m_app_hid_generic =
#if 0
		{
	.specific = {
		.p_data = m_app_hid_generic_data,
		.p_class_methods = APP_USBD_HID_GENERIC_CONFIG(HID_GENERIC_INTERFACE, ENDPOINT_LIST()),                                           \
		.iface = {
			.cnt    = NUM_VA_ARGS(BRACKET_EXTRACT(interfaces_configs)),
			.config = { APP_USBD_CLASS_IFACES_CONFIG_EXTRACT(interfaces_configs) },
			.ep     = { APP_USBD_CLASS_IFACES_EP_EXTRACT(interfaces_configs) }
		},
		BRACKET_EXTRACT(class_config_part)

};
#elif 0
		APP_USBD_CLASS_INSTANCE_INITVAL(
				&CONCAT_2(m_app_hid_generic, _data),
				&app_usbd_generic_class_methods,
		//APP_USBD_HID_GENERIC_CONFIG(USBD_INTERFACE_HID, ENDPOINT_LIST()),
				INTERFACE_CONFIGS,
	    (APP_USBD_HID_GENERIC_INST_CONFIG(&CONCAT_2(m_app_hid_generic, _in),
	                                      &CONCAT_2(m_app_hid_generic, _out),
	                                      &CONCAT_2(m_app_hid_generic, _feature),
	                                      hid_user_ev_handler,
	                                      &m_app_hid_generic_queue,
										  reps,
										  APP_USBD_HID_SUBCLASS_BOOT,
										  APP_USBD_HID_PROTO_MOUSE,
	                                      CONCAT_2(m_app_hid_generic, _ep)))
	);

#endif
#endif
#if 1
/**
 * @brief Reuse HID mouse report descriptor for HID generic class
 */
//APP_USBD_HID_GENERIC_SUBCLASS_REPORT_DESC(mouse_desc,APP_USBD_HID_MOUSE_REPORT_DSC_BUTTON(2));

//static const app_usbd_hid_subclass_desc_t * reps[] = {&mouse_desc};

/*lint -save -e26 -e64 -e123 -e505 -e651*/

/**
 * @brief Global HID generic instance
 */
APP_USBD_HID_GENERIC_GLOBAL_DEF(m_app_hid_generic,
		USBD_INTERFACE_HID,
                                hid_user_ev_handler,
                                ENDPOINT_LIST(),
                                reps,
                                REPORT_IN_QUEUE_SIZE,
                                REPORT_OUT_MAXSIZE,
                                REPORT_FEATURE_MAXSIZE,
                                APP_USBD_HID_SUBCLASS_BOOT,
                                APP_USBD_HID_PROTO_MOUSE);

/*lint -restore*/
#endif


/**
 * @brief Mouse state
 *
 * Current mouse status
 */
struct
{
    int16_t acc_x;    /**< Accumulated x state */
    int16_t acc_y;    /**< Accumulated y state */
    uint8_t btn;      /**< Current btn state */
    uint8_t last_btn; /**< Last transfered button state */
}m_mouse_state;

/**
 * @brief Mark the ongoing transmission
 *
 * Marks that the report buffer is busy and cannot be used until transmission finishes
 * or invalidates (by USB reset or suspend event).
 */
static bool m_report_pending;

/**
 * @brief Timer to repeat mouse move
 */
//APP_TIMER_DEF(m_mouse_move_timer);

/**
 * @brief Get maximal allowed accumulated value
 *
 * Function gets maximal value from the accumulated input.
 * @sa m_mouse_state::acc_x, m_mouse_state::acc_y
 */
static int8_t hid_acc_for_report_get(int16_t acc)
{
    if(acc > INT8_MAX)
    {
        return INT8_MAX;
    }
    else if(acc < INT8_MIN)
    {
        return INT8_MIN;
    }
    else
    {
        return (int8_t)(acc);
    }
}

/**
 * @brief Internal function that process mouse state
 *
 * This function checks current mouse state and tries to send
 * new report if required.
 * If report sending was successful it clears accumulated positions
 * and mark last button state that was transfered.
 */
static void hid_generic_mouse_process_state(void)
{
    if (m_report_pending)
        return;
    if ((m_mouse_state.acc_x != 0) ||
        (m_mouse_state.acc_y != 0) ||
        (m_mouse_state.btn != m_mouse_state.last_btn))
    {
        ret_code_t ret;
        static uint8_t report[HID_REP_SIZE];
        /* We have some status changed that we need to transfer */
        report[HID_BTN_IDX] = m_mouse_state.btn;
        report[HID_X_IDX]   = (uint8_t)hid_acc_for_report_get(m_mouse_state.acc_x);
        report[HID_Y_IDX]   = (uint8_t)hid_acc_for_report_get(m_mouse_state.acc_y);
        /* Start the transfer */
        ret = app_usbd_hid_generic_in_report_set(
            &m_app_hid_generic,
            report,
            sizeof(report));
        if (ret == NRF_SUCCESS)
        {
            m_report_pending = true;
            m_mouse_state.last_btn = report[HID_BTN_IDX];
            CRITICAL_REGION_ENTER();
            /* This part of the code can fail if interrupted by BSP keys processing.
             * Lock interrupts to be safe */
            m_mouse_state.acc_x   -= (int8_t)report[HID_X_IDX];
            m_mouse_state.acc_y   -= (int8_t)report[HID_Y_IDX];
            CRITICAL_REGION_EXIT();
        }
    }
}

/**
 * @brief HID generic IN report send handling
 * */
static void hid_generic_mouse_action(hid_generic_mouse_action_t action, int8_t param)
{
    CRITICAL_REGION_ENTER();
    /*
     * Update mouse state
     */
    switch (action)
    {
        case HID_GENERIC_MOUSE_X:
            m_mouse_state.acc_x += param;
            break;
        case HID_GENERIC_MOUSE_Y:
            m_mouse_state.acc_y += param;
            break;
        case HID_GENERIC_MOUSE_BTN_RIGHT:
            if(param == 1)
            {
                m_mouse_state.btn |= HID_BTN_RIGHT_MASK;
            }
            else
            {
                m_mouse_state.btn &= ~HID_BTN_RIGHT_MASK;
            }
            break;
        case HID_GENERIC_MOUSE_BTN_LEFT:
            if(param == 1)
            {
                m_mouse_state.btn |= HID_BTN_LEFT_MASK;
            }
            else
            {
                m_mouse_state.btn &= ~HID_BTN_LEFT_MASK;
            }
            break;
    }
    CRITICAL_REGION_EXIT();
}

/**
 * @brief Class specific event handler.
 *
 * @param p_inst    Class instance.
 * @param event     Class specific event.
 * */
static void hid_user_ev_handler(app_usbd_class_inst_t const * p_inst,
                                app_usbd_hid_user_event_t event)
{
    switch (event)
    {
        case APP_USBD_HID_USER_EVT_OUT_REPORT_READY:
        {
            /* No output report defined for this example.*/
            ASSERT(0);
            break;
        }
        case APP_USBD_HID_USER_EVT_IN_REPORT_DONE:
        {
            m_report_pending = false;
            hid_generic_mouse_process_state();
           // bsp_board_led_invert(LED_HID_REP_IN);
            break;
        }
        case APP_USBD_HID_USER_EVT_SET_BOOT_PROTO:
        {
            UNUSED_RETURN_VALUE(hid_generic_clear_buffer(p_inst));
           // NRF_LOG_INFO("SET_BOOT_PROTO");
            break;
        }
        case APP_USBD_HID_USER_EVT_SET_REPORT_PROTO:
        {
            UNUSED_RETURN_VALUE(hid_generic_clear_buffer(p_inst));
           // NRF_LOG_INFO("SET_REPORT_PROTO");
            break;
        }
        default:
            break;
    }
}

/**
 * @brief USBD library specific event handler.
 *
 * @param event     USBD library event.
 * */
static void usbd_user_ev_handler(app_usbd_event_type_t event)
{
    switch (event)
    {
        case APP_USBD_EVT_DRV_SOF:
            break;
        case APP_USBD_EVT_DRV_RESET:
            m_report_pending = false;
            break;
        case APP_USBD_EVT_DRV_SUSPEND:
            m_report_pending = false;
            app_usbd_suspend_req(); // Allow the library to put the peripheral into sleep mode
           // bsp_board_leds_off();
            break;
        case APP_USBD_EVT_DRV_RESUME:
            m_report_pending = false;
           // bsp_board_led_on(LED_USB_START);
            break;
        case APP_USBD_EVT_STARTED:
            m_report_pending = false;
           // bsp_board_led_on(LED_USB_START);
            break;
        case APP_USBD_EVT_STOPPED:
            app_usbd_disable();
           // bsp_board_leds_off();
            break;
        case APP_USBD_EVT_POWER_DETECTED:
          //  NRF_LOG_INFO("USB power detected");
            if (!nrf_drv_usbd_is_enabled())
            {
                app_usbd_enable();
            }
            break;
        case APP_USBD_EVT_POWER_REMOVED:
           // NRF_LOG_INFO("USB power removed");
            app_usbd_stop();
            break;
        case APP_USBD_EVT_POWER_READY:
           // NRF_LOG_INFO("USB ready");
            app_usbd_start();
            break;
        default:
            break;
    }
}

static void mouse_move_timer_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    bool used = false;

   // if (bsp_button_is_pressed(BTN_MOUSE_X_POS))
    {
        hid_generic_mouse_action(HID_GENERIC_MOUSE_X, CONFIG_MOUSE_MOVE_SPEED);
        used = true;
    }
  //  if (bsp_button_is_pressed(BTN_MOUSE_Y_POS))
    {
        hid_generic_mouse_action(HID_GENERIC_MOUSE_Y, CONFIG_MOUSE_MOVE_SPEED);
        used = true;
    }

    if(!used)
    {
  //      UNUSED_RETURN_VALUE(app_timer_stop(m_mouse_move_timer));
    }
}
#if 0
static void bsp_event_callback(bsp_event_t ev)
{
    switch ((unsigned int)ev)
    {
        case CONCAT_2(BSP_EVENT_KEY_, BTN_MOUSE_X_POS):
            hid_generic_mouse_action(HID_GENERIC_MOUSE_X, CONFIG_MOUSE_MOVE_SPEED);
            UNUSED_RETURN_VALUE(app_timer_start(m_mouse_move_timer, APP_TIMER_TICKS(CONFIG_MOUSE_MOVE_TIME_MS), NULL));
            break;

        case CONCAT_2(BSP_EVENT_KEY_, BTN_MOUSE_Y_POS):
            hid_generic_mouse_action(HID_GENERIC_MOUSE_Y, CONFIG_MOUSE_MOVE_SPEED);
            UNUSED_RETURN_VALUE(app_timer_start(m_mouse_move_timer, APP_TIMER_TICKS(CONFIG_MOUSE_MOVE_TIME_MS), NULL));
            break;

        case CONCAT_2(BSP_EVENT_KEY_, BTN_MOUSE_RIGHT):
            hid_generic_mouse_action(HID_GENERIC_MOUSE_BTN_RIGHT, 1);
            break;
        case CONCAT_2(BSP_USER_EVENT_RELEASE_, BTN_MOUSE_RIGHT):
            hid_generic_mouse_action(HID_GENERIC_MOUSE_BTN_RIGHT, -1);
            break;

        case CONCAT_2(BSP_EVENT_KEY_, BTN_MOUSE_LEFT):
            hid_generic_mouse_action(HID_GENERIC_MOUSE_BTN_LEFT, 1);
            break;
        case CONCAT_2(BSP_USER_EVENT_RELEASE_, BTN_MOUSE_LEFT):
            hid_generic_mouse_action(HID_GENERIC_MOUSE_BTN_LEFT, -1);
            break;

        default:
            return; // no implementation needed
    }
}
#endif
#if NRF_CLI_ENABLED
void init_cli(void)
{
    ret_code_t ret;
    ret = nrf_cli_init(&m_cli_cdc_acm, nullptr, true, true, NRF_LOG_SEVERITY_INFO);
    APP_ERROR_CHECK(ret);
    ret = nrf_cli_start(&m_cli_cdc_acm);
    APP_ERROR_CHECK(ret);
}
#endif

static ret_code_t idle_handle(app_usbd_class_inst_t const * p_inst, uint8_t report_id)
{
    switch (report_id)
    {
        case 0:
        {
            uint8_t report[] = {0xBE, 0xEF};
            return app_usbd_hid_generic_idle_report_set(
              &m_app_hid_generic,
              report,
              sizeof(report));
        }
        default:
            return NRF_ERROR_NOT_SUPPORTED;
    }

}

void UsbInit()
{
    ret_code_t ret;
    static const app_usbd_config_t usbd_config = {
        .ev_state_proc = usbd_user_ev_handler
    };

    ret = app_usbd_init(&usbd_config);
    APP_ERROR_CHECK(ret);

	app_usbd_serial_num_generate();

    app_usbd_class_inst_t const * class_cdc_acm =
            app_usbd_cdc_acm_class_inst_get(&nrf_cli_cdc_acm);
    ret = app_usbd_class_append(class_cdc_acm);
    APP_ERROR_CHECK(ret);

    app_usbd_class_inst_t const * class_inst_generic;
    class_inst_generic = app_usbd_hid_generic_class_inst_get(&m_app_hid_generic);

    ret = hid_generic_idle_handler_set(class_inst_generic, idle_handle);
    APP_ERROR_CHECK(ret);

    ret = app_usbd_class_append(class_inst_generic);
    APP_ERROR_CHECK(ret);

    if (USBD_POWER_DETECTION)
    {
        ret = app_usbd_power_events_enable();
        APP_ERROR_CHECK(ret);
    }
    else
    {
        app_usbd_enable();
        app_usbd_start();
    }
}