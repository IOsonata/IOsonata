// Definitions for the HCI command entry points bt_hci_ctlr_sdc.cpp dispatches
// to. Each records its own name, so a test can ask which vendor call an
// opcode reached. That mapping is the thing BtHciCmdSdc gets wrong quietly:
// its switch answers 0xFF in the default case, so an opcode with no case
// looks exactly like a controller refusing the command.

#include <cstddef>
#include <cstring>

#include "sdc_hci_cmd_stub.h"

const char *g_SdcHciCmdLast;
int g_SdcHciCmdCount;
uint8_t g_SdcHciCmdStatus;

void SdcHciCmdStubReset(void)
{
	g_SdcHciCmdLast = nullptr;
	g_SdcHciCmdCount = 0;
	g_SdcHciCmdStatus = 0;
}

extern "C" {

#define SDC_CMD_RECORD(name) \
	g_SdcHciCmdLast = #name; \
	g_SdcHciCmdCount++; \
	return g_SdcHciCmdStatus

uint8_t sdc_hci_cmd_cb_reset(void)
{
	SDC_CMD_RECORD(cb_reset);
}

uint8_t sdc_hci_cmd_le_clear_periodic_adv_list(void)
{
	SDC_CMD_RECORD(le_clear_periodic_adv_list);
}

uint8_t sdc_hci_cmd_le_periodic_adv_create_sync_cancel(void)
{
	SDC_CMD_RECORD(le_periodic_adv_create_sync_cancel);
}

uint8_t sdc_hci_cmd_le_read_max_data_length(sdc_hci_cmd_le_read_max_data_length_return_t *)
{
	SDC_CMD_RECORD(le_read_max_data_length);
}

uint8_t sdc_hci_cmd_le_read_periodic_adv_list_size(sdc_hci_cmd_le_read_periodic_adv_list_size_return_t *)
{
	SDC_CMD_RECORD(le_read_periodic_adv_list_size);
}

uint8_t sdc_hci_cmd_cb_set_event_mask(const sdc_hci_cmd_cb_set_event_mask_t *)
{
	SDC_CMD_RECORD(cb_set_event_mask);
}

uint8_t sdc_hci_cmd_cb_set_event_mask_page_2(const sdc_hci_cmd_cb_set_event_mask_page_2_t *)
{
	SDC_CMD_RECORD(cb_set_event_mask_page_2);
}

uint8_t sdc_hci_cmd_lc_disconnect(const sdc_hci_cmd_lc_disconnect_t *)
{
	SDC_CMD_RECORD(lc_disconnect);
}

uint8_t sdc_hci_cmd_le_add_device_to_periodic_adv_list(const sdc_hci_cmd_le_add_device_to_periodic_adv_list_t *)
{
	SDC_CMD_RECORD(le_add_device_to_periodic_adv_list);
}

uint8_t sdc_hci_cmd_le_create_conn(const sdc_hci_cmd_le_create_conn_t *)
{
	SDC_CMD_RECORD(le_create_conn);
}

uint8_t sdc_hci_cmd_le_enable_encryption(const sdc_hci_cmd_le_enable_encryption_t *)
{
	SDC_CMD_RECORD(le_enable_encryption);
}

uint8_t sdc_hci_cmd_le_periodic_adv_create_sync(const sdc_hci_cmd_le_periodic_adv_create_sync_t *)
{
	SDC_CMD_RECORD(le_periodic_adv_create_sync);
}

uint8_t sdc_hci_cmd_le_periodic_adv_terminate_sync(const sdc_hci_cmd_le_periodic_adv_terminate_sync_t *)
{
	SDC_CMD_RECORD(le_periodic_adv_terminate_sync);
}

uint8_t sdc_hci_cmd_le_remove_device_from_periodic_adv_list(const sdc_hci_cmd_le_remove_device_from_periodic_adv_list_t *)
{
	SDC_CMD_RECORD(le_remove_device_from_periodic_adv_list);
}

uint8_t sdc_hci_cmd_le_set_adv_set_random_address(const sdc_hci_cmd_le_set_adv_set_random_address_t *)
{
	SDC_CMD_RECORD(le_set_adv_set_random_address);
}

uint8_t sdc_hci_cmd_le_set_event_mask(const sdc_hci_cmd_le_set_event_mask_t *)
{
	SDC_CMD_RECORD(le_set_event_mask);
}

uint8_t sdc_hci_cmd_le_set_ext_adv_data(const sdc_hci_cmd_le_set_ext_adv_data_t *)
{
	SDC_CMD_RECORD(le_set_ext_adv_data);
}

uint8_t sdc_hci_cmd_le_set_ext_adv_enable(const sdc_hci_cmd_le_set_ext_adv_enable_t *)
{
	SDC_CMD_RECORD(le_set_ext_adv_enable);
}

uint8_t sdc_hci_cmd_le_set_ext_scan_enable(const sdc_hci_cmd_le_set_ext_scan_enable_t *)
{
	SDC_CMD_RECORD(le_set_ext_scan_enable);
}

uint8_t sdc_hci_cmd_le_set_ext_scan_params(const sdc_hci_cmd_le_set_ext_scan_params_t *)
{
	SDC_CMD_RECORD(le_set_ext_scan_params);
}

uint8_t sdc_hci_cmd_le_set_ext_scan_response_data(const sdc_hci_cmd_le_set_ext_scan_response_data_t *)
{
	SDC_CMD_RECORD(le_set_ext_scan_response_data);
}

uint8_t sdc_hci_cmd_le_set_host_feature(const sdc_hci_cmd_le_set_host_feature_t *)
{
	SDC_CMD_RECORD(le_set_host_feature);
}

uint8_t sdc_hci_cmd_le_set_periodic_adv_data(const sdc_hci_cmd_le_set_periodic_adv_data_t *)
{
	SDC_CMD_RECORD(le_set_periodic_adv_data);
}

uint8_t sdc_hci_cmd_le_set_periodic_adv_enable(const sdc_hci_cmd_le_set_periodic_adv_enable_t *)
{
	SDC_CMD_RECORD(le_set_periodic_adv_enable);
}

uint8_t sdc_hci_cmd_le_set_periodic_adv_params(const sdc_hci_cmd_le_set_periodic_adv_params_t *)
{
	SDC_CMD_RECORD(le_set_periodic_adv_params);
}

uint8_t sdc_hci_cmd_le_set_periodic_adv_receive_enable(const sdc_hci_cmd_le_set_periodic_adv_receive_enable_t *)
{
	SDC_CMD_RECORD(le_set_periodic_adv_receive_enable);
}

uint8_t sdc_hci_cmd_le_set_phy(const sdc_hci_cmd_le_set_phy_t *)
{
	SDC_CMD_RECORD(le_set_phy);
}

uint8_t sdc_hci_cmd_le_set_random_address(const sdc_hci_cmd_le_set_random_address_t *)
{
	SDC_CMD_RECORD(le_set_random_address);
}

uint8_t sdc_hci_cmd_le_write_suggested_default_data_length(const sdc_hci_cmd_le_write_suggested_default_data_length_t *)
{
	SDC_CMD_RECORD(le_write_suggested_default_data_length);
}

uint8_t sdc_hci_cmd_le_encrypt(const sdc_hci_cmd_le_encrypt_t *, sdc_hci_cmd_le_encrypt_return_t *)
{
	SDC_CMD_RECORD(le_encrypt);
}

uint8_t sdc_hci_cmd_le_long_term_key_request_negative_reply(const sdc_hci_cmd_le_long_term_key_request_negative_reply_t *, sdc_hci_cmd_le_long_term_key_request_negative_reply_return_t *)
{
	SDC_CMD_RECORD(le_long_term_key_request_negative_reply);
}

uint8_t sdc_hci_cmd_le_long_term_key_request_reply(const sdc_hci_cmd_le_long_term_key_request_reply_t *, sdc_hci_cmd_le_long_term_key_request_reply_return_t *)
{
	SDC_CMD_RECORD(le_long_term_key_request_reply);
}

uint8_t sdc_hci_cmd_le_read_phy(const sdc_hci_cmd_le_read_phy_t *, sdc_hci_cmd_le_read_phy_return_t *)
{
	SDC_CMD_RECORD(le_read_phy);
}

uint8_t sdc_hci_cmd_le_set_data_length(const sdc_hci_cmd_le_set_data_length_t *, sdc_hci_cmd_le_set_data_length_return_t *)
{
	SDC_CMD_RECORD(le_set_data_length);
}

uint8_t sdc_hci_cmd_le_set_ext_adv_params(const sdc_hci_cmd_le_set_ext_adv_params_t *, sdc_hci_cmd_le_set_ext_adv_params_return_t *)
{
	SDC_CMD_RECORD(le_set_ext_adv_params);
}

uint8_t sdc_hci_cmd_le_set_ext_adv_params_v2(const sdc_hci_cmd_le_set_ext_adv_params_v2_t *, sdc_hci_cmd_le_set_ext_adv_params_v2_return_t *)
{
	SDC_CMD_RECORD(le_set_ext_adv_params_v2);
}

uint8_t sdc_hci_cmd_le_set_periodic_adv_params_v2(const sdc_hci_cmd_le_set_periodic_adv_params_v2_t *, sdc_hci_cmd_le_set_periodic_adv_params_v2_return_t *)
{
	SDC_CMD_RECORD(le_set_periodic_adv_params_v2);
}

uint8_t sdc_hci_cmd_le_set_periodic_adv_response_data(const sdc_hci_cmd_le_set_periodic_adv_response_data_t *, sdc_hci_cmd_le_set_periodic_adv_response_data_return_t *)
{
	SDC_CMD_RECORD(le_set_periodic_adv_response_data);
}

uint8_t sdc_hci_cmd_le_set_periodic_adv_subevent_data(const sdc_hci_cmd_le_set_periodic_adv_subevent_data_t *, sdc_hci_cmd_le_set_periodic_adv_subevent_data_return_t *)
{
	SDC_CMD_RECORD(le_set_periodic_adv_subevent_data);
}

uint8_t sdc_hci_cmd_le_set_periodic_sync_subevent(const sdc_hci_cmd_le_set_periodic_sync_subevent_t *, sdc_hci_cmd_le_set_periodic_sync_subevent_return_t *)
{
	SDC_CMD_RECORD(le_set_periodic_sync_subevent);
}

}	// extern "C"
