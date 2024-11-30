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

// This is to overside hard coded VID/PID in Nordic Lib
extern "C" const app_usbd_descriptor_device_t m_device_dsc = APP_USBD_CORE_DEVICE_DESCRIPTOR;


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
 * @brief User event handler.
 * */
static void hid_user_ev_handler(app_usbd_class_inst_t const * p_inst,
                                app_usbd_hid_user_event_t event);


#if 0
/*
static const uint8_t hid_report_desc[] = {
	HID_USAGE_PAGE(HID_USAGE_GEN_DESKTOP),
	HID_USAGE(HID_USAGE_GEN_DESKTOP_UNDEFINED),
	HID_COLLECTION(HID_COLLECTION_APPLICATION),
		HID_USAGE(HID_USAGE_GEN_DESKTOP_UNDEFINED),
		HID_REPORT_SIZE(8),
		HID_REPORT_COUNT(64),
		HID_INPUT(0x02),
	HID_END_COLLECTION,
};
*/

#define SLIMEVR_HID_REPORT_DESC		{\
		0x05, 0x01,        \
		0x09, 0x00,        \
		0xA1, 0x01,        \
		0x09, 0x00,        \
		0x75, 0x08,        \
		0x95, 0x40,        \
		0x81, 0x02,        \
		0xC0,              \
}

APP_USBD_HID_GENERIC_SUBCLASS_REPORT_DESC(s_SlimeHIDReportDesc, SLIMEVR_HID_REPORT_DESC);
#else
// not using Nordic macro
static const uint8_t s_SlimeHIDReportDescData[] = {
	0x05, 0x01,        // Usage Page (Generic Desktop Ctrls)
	0x09, 0x00,        // Usage (Undefined)
	0xA1, 0x01,        // Collection (Application)
	0x09, 0x00,        //   Usage (Undefined)
	0x75, 0x08,        //   Report Size (8)
	0x95, 0x40,        //   Report Count (64)
	0x81, 0x02,        //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
	0xC0,              // End Collection
};
static const app_usbd_hid_subclass_desc_t s_SlimeHIDReportDesc = { sizeof(s_SlimeHIDReportDescData), APP_USBD_DESCRIPTOR_REPORT, s_SlimeHIDReportDescData};
#endif

static const app_usbd_hid_subclass_desc_t * reps[] = { &s_SlimeHIDReportDesc };


#define INTERFACE_CONFIGS APP_USBD_HID_GENERIC_CONFIG(USBD_INTERFACE_HID, ENDPOINT_LIST())

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

#if 1

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

#if 0
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
#endif
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
            //hid_generic_mouse_process_state();
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
