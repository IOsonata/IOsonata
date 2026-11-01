/* ST callback dependencies; runner inserts the complete production callback. */
#include <assert.h>
#include <stdint.h>
#include <string.h>
#include <stddef.h>

typedef uint8_t tBleStatus;
#define BLE_STATUS_FAILED 1
#define BLE_STATUS_SUCCESS 0
#define HCI_EVENT_PKT_TYPE 4
#define HCI_ACLDATA_PKT_TYPE 2
#define CFG_AMM_VIRTUAL_BLE_EVENTS 0
#define CFG_TASK_HCI_ASYNCH_EVT_ID 0
#define CFG_SEQ_PRIO_0 0
#define AMM_ERROR_OK 0
#define MEMCPY memcpy
#define DIVC(a, b) (((a) + (b) - 1) / (b))
#define LOG_INFO_APP(...) ((void)0)
typedef struct { uint32_t words[2]; } BleEvtPacketHeader_t;
typedef struct { uint8_t unused; } tListNode;
typedef struct {
	BleEvtPacketHeader_t header;
	struct {
		uint8_t type;
		struct { uint8_t evtcode; uint8_t plen; uint8_t payload[255]; } evt;
	} evtserial;
} BleEvtPacket_t;
static BleEvtPacket_t storage;
static struct { void (*Callback)(void); } BLE_EVENTS_ResumeFlowProcessCb;
static tListNode BleAsynchEventQueue;
static unsigned inserted;
static unsigned allocationFails;
static void BLE_ResumeFlowProcessCallback(void) {}
static int AMM_Alloc(unsigned Pool, unsigned Words, uint32_t **Out, void *Resume)
{
	assert(Words * 4 <= sizeof(storage));
	*Out = (uint32_t *)&storage;
	return allocationFails;
}
static void LST_insert_tail(tListNode *Head, tListNode *Node) { inserted++; }
static void UTIL_SEQ_SetTask(uint32_t Mask, unsigned Priority) {}

/* CALLBACK */

int main(void)
{
	uint8_t header[] = {4, 255, 248, 0x0c, 0x1b, 0, 0};
	uint8_t payload[244];
	uint8_t shortEvent[] = {4, 5, 4, 0, 0, 0, 0x16};

	memset(payload, 0x55, sizeof(payload));
	assert(BLECB_Indication(header, sizeof(header), payload, sizeof(payload)) == 0);
	assert(inserted == 1 && storage.evtserial.evt.plen == 248);
	assert(memcmp(storage.evtserial.evt.payload, header + 3, 4) == 0);
	assert(memcmp(storage.evtserial.evt.payload + 4, payload, 244) == 0);
	memset(payload, 0xaa, sizeof(payload));
	assert(storage.evtserial.evt.payload[247] == 0x55);
	assert(BLECB_Indication(shortEvent, sizeof(shortEvent), NULL, 0) == 0);
	assert(BLECB_Indication(header, sizeof(header), NULL, 244) != 0);
	assert(BLECB_Indication(header, sizeof(header), payload, 243) != 0);
	assert(BLECB_Indication(header, 2, payload, 244) != 0);
	assert(BLECB_Indication(NULL, 0, NULL, 0) != 0);
	allocationFails = 1;
	assert(BLECB_Indication(shortEvent, sizeof(shortEvent), NULL, 0) != 0);
	assert(inserted == 2);
	return 0;
}
