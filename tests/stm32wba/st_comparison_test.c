/* Included after the production client and command mocks by run_st_comparison.py. */
static uint32_t clockMs;
static unsigned reports;
static uint8_t pattern = 0x7f;

uint32_t UTIL_TIMER_GetCurrentTime(void)
{
	return clockMs;
}

void UTIL_SEQ_SetTask(uint32_t Mask, unsigned Priority)
{
	(void)Mask;
	(void)Priority;
}

void UTIL_SEQ_RegTask(uint32_t Mask, unsigned Flags, void (*Function)(void))
{
	(void)Mask;
	(void)Flags;
	(void)Function;
}

int UTIL_TIMER_Create(UTIL_TIMER_Object_t *Timer, unsigned Period, unsigned Mode,
	void (*Callback)(void *), void *Context)
{
	assert(Period == 20 && Mode == UTIL_TIMER_PERIODIC);
	return 0;
}

int UTIL_TIMER_Start(UTIL_TIMER_Object_t *Timer)
{
	return 0;
}

void TestLog(const char *Format, ...)
{
	if (strstr(Format, "Bytes/sec="))
	{
		reports++;
	}
}

SVCCTL_UserEvtFlowStatus_t SVCCTL_App_Notification(void *Event)
{
	/* Any fallback to the original interactive application is a regression. */
	assert(0);
	return SVCCTL_UserEvtFlowEnable;
}

static void Event(uint8_t Code, const uint8_t *Data, unsigned Len)
{
	uint8_t packet[258];

	assert(Len <= 255);
	packet[0] = 4;
	packet[1] = Code;
	packet[2] = Len;
	memcpy(packet + 3, Data, Len);
	inEvent = 1;
	assert(SVCCTL_UserEvtRx(packet) == SVCCTL_UserEvtFlowEnable);
	inEvent = 0;
}

static void Vendor(uint16_t Code, const uint8_t *Data, unsigned Len)
{
	uint8_t payload[255];

	payload[0] = Code;
	payload[1] = Code >> 8;
	memcpy(payload + 2, Data, Len);
	Event(0xff, payload, Len + 2);
}

static void Complete(void)
{
	const uint8_t done[] = {0, 0, 0};

	Vendor(ACI_GATT_PROC_COMPLETE_VSEVT_CODE, done, sizeof(done));
}

static void Connect(void)
{
	uint8_t adv[] = {2, 1, 4, 0, 1, 2, 3, 4, 5, 6, 13,
		12, 9, 'B', 'l', 'e', 'P', 'r', 'b', 's', '_', 'S', 'D', 'C', 220};
	const uint8_t stopped[] = {GAP_GENERAL_DISCOVERY_PROC, 0, 0};
	uint8_t connected[31] = {10, 0, 0, 0, 0};
	uint8_t phy[] = {12, 0, 0, 0, 1, 1};
	const uint8_t dle[] = {7, 0, 0, 251, 0, 0x48, 8, 251, 0, 0x48, 8};
	const uint8_t mtu[] = {0, 0, 247, 0};
	const uint8_t service[] = {0, 0, 1, 1, 0, 20, 0};
	uint8_t chars[46] = {0, 0, 21, 42, 2, 0, 0x10, 3, 0};
	const uint8_t descriptors[] = {0, 0, 1, 4, 4, 0, 2, 0x29};

	PrbsWork();
	assert(s_Phase == PRBS_SCANNING);
	assert(strcmp(lastCommand, "aci_gap_start_scan") == 0);
	/* Truncated advertisements must not read outside their payload. */
	Event(0x3e, adv, 8);
	assert(s_Phase == PRBS_SCANNING);
	Event(0x3e, adv, sizeof(adv));
	assert(s_Phase == PRBS_STOP_SCAN);
	PrbsWork();
	assert(s_Phase == PRBS_SCAN_STOPPING);
	Vendor(ACI_GAP_PROC_COMPLETE_VSEVT_CODE, stopped, sizeof(stopped));
	PrbsWork();
	assert(s_Phase == PRBS_CONNECTING);
	connected[24] = 12;
	connected[28] = 0x90;
	connected[29] = 1;
	Event(0x3e, connected, sizeof(connected));
	assert(s_Phase == PRBS_PHY);
	PrbsWork();
	assert(s_Phase == PRBS_PHY_WAIT);
	assert(strcmp(lastCommand, "hci_le_set_phy") == 0);
	PrbsWork();
	assert(strcmp(lastCommand, "hci_le_set_phy") == 0);
	phy[4] = s_Phy;
	phy[5] = s_Phy;
	Event(0x3e, phy, sizeof(phy));
	PrbsWork();
	assert(s_Phase == PRBS_DLE_WAIT);
	Event(0x3e, dle, sizeof(dle));
	PrbsWork();
	assert(s_Phase == PRBS_MTU_WAIT);
	Vendor(ACI_ATT_EXCHANGE_MTU_RESP_VSEVT_CODE, mtu, sizeof(mtu));
	Complete();
	PrbsWork();
	assert(s_Phase == PRBS_SERVICE_WAIT);
	Vendor(ACI_ATT_FIND_BY_TYPE_VALUE_RESP_VSEVT_CODE, service, sizeof(service));
	Complete();
	PrbsWork();
	assert(s_Phase == PRBS_CHARS_WAIT);
	memcpy(chars + 9, s_ValueUuid, 16);
	chars[25] = 10;
	chars[27] = 2;
	chars[28] = 11;
	Vendor(ACI_ATT_READ_BY_TYPE_RESP_VSEVT_CODE, chars, sizeof(chars));
	Complete();
	assert(s_Value == 3 && s_DescEnd == 9);
	PrbsWork();
	assert(s_Phase == PRBS_DESCRIPTORS_WAIT);
	Vendor(ACI_ATT_FIND_INFO_RESP_VSEVT_CODE, descriptors, sizeof(descriptors));
	Complete();
	PrbsWork();
	assert(s_Phase == PRBS_SUBSCRIBE_WAIT && s_Cccd == 4);
	Complete();
	assert(s_Phase == PRBS_RECEIVE && !s_Failed);
}

static void Notification(void)
{
	uint8_t payload[249] = {0, 0, 3, 0, 244};
	unsigned i;

	for (i = 5; i < sizeof(payload); i++)
	{
		payload[i] = pattern;
		pattern = ((pattern << 1) | (((pattern >> 6) ^ (pattern >> 5)) & 1)) & 0x7f;
	}
	Vendor(ACI_GATT_NOTIFICATION_VSEVT_CODE, payload, sizeof(payload));
	memset(payload, 0xaa, sizeof(payload));
}

int main(void)
{
	const uint8_t disconnected[] = {0, 0, 0, 0x16};
	uint32_t start;
	unsigned i;
	unsigned before;

	SVCCTL_Init();
	PrbsClientInit();
	Connect();
	for (i = 0; i < 2; i++)
	{
		start = clockMs;
		Notification();
		PrbsWork();
		assert(s_Stats.Bytes == 244 && s_Stats.Breaks == 0);
		clockMs += 1000;
		Notification();
		PrbsWork();
		assert(s_Stats.Synced && s_Stats.Breaks == 0);
		clockMs = start + 59999;
		Notification();
		PrbsWork();
		assert(!s_Final);
		clockMs++;
		Notification();
		PrbsWork();
		assert(s_Final && s_Complete && s_Stats.Bytes == 732);
		assert(s_Phase == PRBS_DISCONNECT_WAIT);
		Event(5, disconnected, sizeof(disconnected));
		if (i == 0)
		{
			assert(s_Phy == 2 && s_Phase == PRBS_SCAN && s_Stats.Bytes == 0);
			Connect();
		}
	}
	assert(s_Phase == PRBS_DONE && reports == 6);
	/* Abort must never advance to 2M or report a complete trial. */
	s_Phy = 1;
	PrbsReset();
	Connect();
	Notification();
	PrbsWork();
	Event(5, disconnected, sizeof(disconnected));
	PrbsWork();
	assert(s_Phy == 1 && !s_Complete && s_Final && s_Phase == PRBS_DONE);
	/* Fixed ownership queue accepts 12 and detects the thirteenth. */
	PrbsReset();
	Connect();
	for (i = 0; i < 13; i++)
	{
		Notification();
	}
	assert(s_Count == 12 && s_Dropped == 1 && s_Failed);
	PrbsWork();
	assert(!s_Complete);
	/* Temporary command refusal is retried; timer callback never calls BLE. */
	s_Connection = PRBS_INVALID;
	PrbsReset();
	reply = BLE_STATUS_BUSY;
	PrbsWork();
	assert(s_Phase == PRBS_SCAN && !s_Failed);
	before = calls;
	PrbsTick(NULL);
	assert(calls == before);
	reply = 0;
	PrbsWork();
	assert(s_Phase == PRBS_SCANNING);
	clockMs += 30000;
	PrbsWork();
	assert(s_Final && !s_Complete && s_Phase == PRBS_DONE);
	puts("automatic 1M/2M, discovery, PRBS ownership, duration, abort and retries passed");
	return 0;
}
