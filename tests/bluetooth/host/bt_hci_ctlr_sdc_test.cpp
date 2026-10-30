// The SoftDevice Controller setup, built against stand-in vendor headers.
//
// This is the layer four successive reviews found unfinished, each time one
// step below where the last one stopped: the command opcodes were not
// dispatched, then the sources were in no project, then a support call sat in
// the wrong role branch, then the response support calls were missing, then
// no periodic advertising resource was reserved at all. Every one of those is
// invisible to a test that drives an encoder and compares bytes, because none
// of them is in the bytes. They are in which vendor calls the setup makes.
//
// So that is what this checks. nordic_stub/sdc supplies the vendor entry points with
// the names and the configuration tag values the real header has, and the
// definitions below record every call. A missing sdc_support_*, a resource
// left unreserved, or a call in the wrong role branch is then a failing check
// rather than a device that comes up and refuses the command later.

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "sdc.h"
#include "nrf.h"

extern void (*g_SdcHciCmdHook)(void);
#include "sdc_hci_vs.h"
#include "sdc_hci_cmd_stub.h"

#include "bluetooth/bt_gap.h"
#include "bluetooth/bt_hci.h"
#include "bluetooth/bt_hci_ctlr.h"
#include "crypto_rng_nrf.h"
#include "nrf_mpsl.h"

#ifdef SDC_TEST_NO_FEATURE_MODULES
// This build runs one test only, see main. The others stay compiled.
#pragma GCC diagnostic ignored "-Wunused-function"
#endif

namespace {

uint32_t s_Basepri;
uint32_t s_Exception;
uint32_t s_LowPriority = 7;
void (*s_PendingIrq)(void);
int s_IrqCalls;
int s_DataPutCalls;
int s_GetCalls;
int32_t s_DataPutResult;
int s_RxRemaining;
bool s_InCommand;

int s_Failures = 0;
int s_Checks = 0;

#define CHECK(expr) do { \
	++s_Checks; \
	if (!(expr)) { \
		std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #expr); \
		++s_Failures; \
	} \
} while (0)

// --- what the setup did -----------------------------------------------------

// One sdc_cfg_set call, kept in order so a tag set twice or set in the wrong
// role branch is visible.
struct CfgCall {
	uint8_t Type;
	sdc_cfg_t Cfg;
};

const int kMaxCfg = 32;
const int kMaxSupport = 64;

CfgCall s_Cfg[kMaxCfg];
int s_CfgCount;
const char *s_Support[kMaxSupport];
int s_SupportCount;
int s_InitCalls;
int s_EnableCalls;
bool s_Enabled;
int s_DisableCalls;
uint8_t s_FailEventConfig;
uint32_t s_EventLength;
bool s_EventExtended;
int s_RandSourceCalls;
int s_MpslInitCalls;
int s_ArbiterStartCalls;
int s_ArbiterStopCalls;

// What a refused vendor call answers. sdc_init and sdc_cfg_set answer a
// negative nrfxlib errno; nothing in the setup reads which one, only the sign,
// so one value stands for all of them rather than pulling nrf_errno.h in.
const int32_t kVendorRefused = -1;

// Failure injection. Each is the call that should fail, or 0 for none.
uint8_t s_FailCfgType;
bool s_FailMpslInit;
bool s_FailInit;
bool s_FailEnable;
bool s_FailArbiter;
bool s_FailRandSource;
// What sdc_cfg_set reports as the pool size the configuration needs.
int32_t s_CfgRam;

void Reset(void)
{
	s_Basepri = 0;
	s_Exception = 0;
	s_LowPriority = 7;
	s_PendingIrq = nullptr;
	g_SdcHciCmdHook = nullptr;
	s_IrqCalls = 0;
	s_DataPutCalls = 0;
	s_GetCalls = 0;
	s_DataPutResult = 0;
	s_RxRemaining = 0;
	s_InCommand = false;
	std::memset(s_Cfg, 0, sizeof(s_Cfg));
	std::memset(s_Support, 0, sizeof(s_Support));
	s_CfgCount = 0;
	s_SupportCount = 0;
	s_InitCalls = 0;
	s_EnableCalls = 0;
	s_Enabled = false;
	s_DisableCalls = 0;
	s_FailEventConfig = 0;
	s_EventLength = 0;
	s_EventExtended = false;
	s_RandSourceCalls = 0;
	s_MpslInitCalls = 0;
	s_ArbiterStartCalls = 0;
	s_ArbiterStopCalls = 0;
	s_FailCfgType = 0;
	s_FailMpslInit = false;
	s_FailInit = false;
	s_FailEnable = false;
	s_FailArbiter = false;
	s_FailRandSource = false;
	s_CfgRam = 0;
}

bool Supported(const char *pName)
{
	for (int i = 0; i < s_SupportCount; i++)
	{
		if (std::strcmp(s_Support[i], pName) == 0)
		{
			return true;
		}
	}

	return false;
}

int SupportIndex(const char *pName)
{
	for (int i = 0; i < s_SupportCount; i++)
	{
		if (std::strcmp(s_Support[i], pName) == 0)
		{
			return i;
		}
	}

	return -1;
}

const CfgCall *FindCfg(uint8_t Type)
{
	for (int i = 0; i < s_CfgCount; i++)
	{
		if (s_Cfg[i].Type == Type)
		{
			return &s_Cfg[i];
		}
	}

	return nullptr;
}

int CountCfg(uint8_t Type)
{
	int n = 0;

	for (int i = 0; i < s_CfgCount; i++)
	{
		if (s_Cfg[i].Type == Type)
		{
			n++;
		}
	}

	return n;
}

// --- driving the setup ------------------------------------------------------

uint8_t s_RxFifoMem[4096];

BtHciCtlrCfg_t MakeCfg(uint16_t Role)
{
	BtHciCtlrCfg_t cfg;

	std::memset(&cfg, 0, sizeof(cfg));
	cfg.Role = Role;
	cfg.CentralDevMax = 1;
	cfg.PeriphDevMax = 1;
	cfg.RxPktCount = 4;
	cfg.TxPktCount = 4;
	cfg.MaxDataLen = 251;
	cfg.PacketSize = 255;
	cfg.pRxFifoMem = s_RxFifoMem;
	cfg.RxFifoMemSize = sizeof(s_RxFifoMem);

	return cfg;
}

// Bring up without connection support having asked for the link features.
bool StartNoConn(BtHciCtlrCfg_t *pCfg)
{
	BtHciCtlrDev_t dev = {};

	if (BtHciCtlrInit(&dev, pCfg) == false)
	{
		return false;
	}

	return BtHciCtlrStart(&dev, pCfg);
}

// Bring up the way an application with links does: connection support asks
// for the peripheral link features before the controller is enabled.
bool Start(BtHciCtlrCfg_t *pCfg)
{
	if (pCfg != nullptr && (pCfg->Role & BT_GAP_ROLE_PERIPHERAL))
	{
		BtHciCtlrPeripheralSupport();
	}

	return StartNoConn(pCfg);
}

// An application without connection support never asks for the peripheral
// link features. A broadcaster comes up without them and a peripheral role
// stops the bring up. Runs first: once asked, the request stays.
void TestPeripheralSupportIsOnRequest(void)
{
	Reset();

	BtHciCtlrCfg_t cfg = MakeCfg(BT_GAP_ROLE_BROADCASTER);
	CHECK(StartNoConn(&cfg));
	CHECK(Supported("adv"));
	CHECK(Supported("ext_adv"));
	CHECK(Supported("peripheral") == false);
	CHECK(s_EnableCalls == 1);

	Reset();
	cfg = MakeCfg(BT_GAP_ROLE_PERIPHERAL);
	CHECK(StartNoConn(&cfg) == false);
	CHECK(BtHciCtlrErrorGet() == BT_HCI_CTLR_ERROR_PERIPHERAL_SUPPORT);
	CHECK(Supported("peripheral") == false);
	CHECK(s_EnableCalls == 0);

	Reset();
	CHECK(Start(&cfg));
	CHECK(Supported("peripheral"));

	// Asked for, a broadcaster still does not enable them
	Reset();
	cfg = MakeCfg(BT_GAP_ROLE_BROADCASTER);
	CHECK(Start(&cfg));
	CHECK(Supported("peripheral") == false);
}

// --- the checks -------------------------------------------------------------

// A broadcaster or peripheral enables the advertising states, an observer or
// central the scanning ones. The periodic advertising calls follow the same
// split, and getting that wrong is not a compile error and not a wire
// difference: sdc_support_le_periodic_sync() sat in the advertising branch,
// where its documented prerequisite sdc_support_ext_scan() is never called, so
// a scanner build enabled nothing and Create Sync was refused.
void TestSupportCallsFollowTheRole(void)
{
	Reset();

	BtHciCtlrCfg_t cfg = MakeCfg(BT_GAP_ROLE_PERIPHERAL);
	CHECK(Start(&cfg));

	CHECK(Supported("adv"));
	CHECK(Supported("ext_adv"));
	CHECK(Supported("peripheral"));
	CHECK(Supported("le_periodic_adv"));
	CHECK(Supported("le_periodic_adv_with_rsp"));
	// Nothing from the scanning side.
	CHECK(Supported("scan") == false);
	CHECK(Supported("ext_scan") == false);
	CHECK(Supported("le_periodic_sync") == false);
	CHECK(Supported("le_periodic_sync_with_rsp") == false);
	CHECK(Supported("central") == false);

	Reset();
	cfg = MakeCfg(BT_GAP_ROLE_OBSERVER);
	CHECK(Start(&cfg));

	CHECK(Supported("scan"));
	CHECK(Supported("ext_scan"));
	CHECK(Supported("le_periodic_sync"));
	CHECK(Supported("le_periodic_sync_with_rsp"));
	CHECK(Supported("adv") == false);
	CHECK(Supported("ext_adv") == false);
	CHECK(Supported("le_periodic_adv") == false);
	CHECK(Supported("le_periodic_adv_with_rsp") == false);

	// A device doing both gets both sets.
	Reset();
	cfg = MakeCfg(BT_GAP_ROLE_PERIPHERAL | BT_GAP_ROLE_CENTRAL);
	CHECK(Start(&cfg));

	CHECK(Supported("le_periodic_adv"));
	CHECK(Supported("le_periodic_adv_with_rsp"));
	CHECK(Supported("le_periodic_sync"));
	CHECK(Supported("le_periodic_sync_with_rsp"));
}

// sdc.h states the ordering prerequisites: extended advertising before
// periodic advertising, extended scanning before periodic sync. A call made
// out of order is accepted and does nothing.
void TestSupportOrderMeetsThePrerequisites(void)
{
	Reset();

	BtHciCtlrCfg_t cfg = MakeCfg(BT_GAP_ROLE_PERIPHERAL | BT_GAP_ROLE_CENTRAL);
	CHECK(Start(&cfg));

	CHECK(SupportIndex("ext_adv") >= 0);
	CHECK(SupportIndex("le_periodic_adv") > SupportIndex("ext_adv"));
	CHECK(SupportIndex("le_periodic_adv_with_rsp") > SupportIndex("ext_adv"));
	CHECK(SupportIndex("ext_scan") >= 0);
	CHECK(SupportIndex("le_periodic_sync") > SupportIndex("ext_scan"));
	CHECK(SupportIndex("le_periodic_sync_with_rsp") > SupportIndex("ext_scan"));
}

// The defect that made periodic advertising impossible on air. Enabling a
// feature is not reserving anything for it: with no set count the controller
// has nothing for the commands to act on, and every one of them is refused
// however well formed it is.
void TestPeriodicResourcesAreReservedOnRequest(void)
{
	Reset();

	// Nothing asked for, nothing reserved. This is what every build that does
	// not use the feature does, and it must not start paying for it.
	BtHciCtlrCfg_t cfg = MakeCfg(BT_GAP_ROLE_PERIPHERAL | BT_GAP_ROLE_CENTRAL);
	CHECK(Start(&cfg));

	CHECK(FindCfg(SDC_CFG_TYPE_PERIODIC_ADV_COUNT) == nullptr);
	CHECK(FindCfg(SDC_CFG_TYPE_PERIODIC_SYNC_COUNT) == nullptr);
	CHECK(FindCfg(SDC_CFG_TYPE_PERIODIC_SYNC_BUFFER_CFG) == nullptr);
	CHECK(FindCfg(SDC_CFG_TYPE_PERIODIC_ADV_RSP_COUNT) == nullptr);
	CHECK(FindCfg(SDC_CFG_TYPE_PERIODIC_ADV_RSP_BUFFER_CFG) == nullptr);
	CHECK(FindCfg(SDC_CFG_TYPE_PERIODIC_SYNC_RSP_TX_BUFFER_CFG) == nullptr);

	// An advertiser asking for one set gets exactly that, and nothing from the
	// receiving side.
	Reset();
	cfg = MakeCfg(BT_GAP_ROLE_PERIPHERAL);
	cfg.PeriodicAdvCount = 2;
	CHECK(Start(&cfg));

	const CfgCall *c = FindCfg(SDC_CFG_TYPE_PERIODIC_ADV_COUNT);
	CHECK(c != nullptr);
	if (c != nullptr)
	{
		CHECK(c->Cfg.periodic_adv_count.count == 2);
	}
	CHECK(FindCfg(SDC_CFG_TYPE_PERIODIC_SYNC_COUNT) == nullptr);
	CHECK(FindCfg(SDC_CFG_TYPE_PERIODIC_ADV_RSP_COUNT) == nullptr);

	// An observer asking to synchronize gets the count and the report buffers,
	// which are separate reservations.
	Reset();
	cfg = MakeCfg(BT_GAP_ROLE_OBSERVER);
	cfg.PeriodicSyncCount = 3;
	CHECK(Start(&cfg));

	c = FindCfg(SDC_CFG_TYPE_PERIODIC_SYNC_COUNT);
	CHECK(c != nullptr);
	if (c != nullptr)
	{
		CHECK(c->Cfg.periodic_sync_count.count == 3);
	}
	CHECK(FindCfg(SDC_CFG_TYPE_PERIODIC_SYNC_BUFFER_CFG) != nullptr);
	CHECK(FindCfg(SDC_CFG_TYPE_PERIODIC_ADV_COUNT) == nullptr);
}

// Responses need their own reservation on both sides. An advertiser gets the
// response set count and the response buffers; a responder gets the transmit
// buffers it answers a subevent out of.
void TestPawrResourcesAreReservedOnRequest(void)
{
	Reset();

	BtHciCtlrCfg_t cfg = MakeCfg(BT_GAP_ROLE_PERIPHERAL);
	cfg.PeriodicAdvCount = 1;
	cfg.PawrAdvCount = 1;
	CHECK(Start(&cfg));

	const CfgCall *c = FindCfg(SDC_CFG_TYPE_PERIODIC_ADV_RSP_COUNT);
	CHECK(c != nullptr);
	if (c != nullptr)
	{
		CHECK(c->Cfg.periodic_adv_rsp_count.count == 1);
	}

	c = FindCfg(SDC_CFG_TYPE_PERIODIC_ADV_RSP_BUFFER_CFG);
	CHECK(c != nullptr);
	if (c != nullptr)
	{
		// Two transmit buffers so one can be filled while the other is on air,
		// and a data size that holds the largest single subevent response.
		CHECK(c->Cfg.periodic_adv_rsp_buffer_cfg.tx_buffer_count == 2);
		CHECK(c->Cfg.periodic_adv_rsp_buffer_cfg.max_tx_data_size == 251);
		CHECK(c->Cfg.periodic_adv_rsp_buffer_cfg.rx_buffer_count == 2);
	}

	// The responder side is reserved from the scanning branch, so an advertiser
	// only build never gets it.
	CHECK(FindCfg(SDC_CFG_TYPE_PERIODIC_SYNC_RSP_TX_BUFFER_CFG) == nullptr);

	Reset();
	cfg = MakeCfg(BT_GAP_ROLE_OBSERVER);
	cfg.PeriodicSyncCount = 1;
	cfg.PawrSyncCount = 2;
	CHECK(Start(&cfg));

	c = FindCfg(SDC_CFG_TYPE_PERIODIC_SYNC_RSP_TX_BUFFER_CFG);
	CHECK(c != nullptr);
	if (c != nullptr)
	{
		CHECK(c->Cfg.periodic_sync_rsp_tx_buffer_cfg.count == 2);
	}
	CHECK(FindCfg(SDC_CFG_TYPE_PERIODIC_ADV_RSP_COUNT) == nullptr);
}

// A count asked for in a role the device does not have reserves nothing. The
// reservation sits inside the role branch, so a peripheral asking to
// synchronize is asking for something it has no scanning state for.
void TestCountsOutsideTheirRoleReserveNothing(void)
{
	Reset();

	BtHciCtlrCfg_t cfg = MakeCfg(BT_GAP_ROLE_PERIPHERAL);
	cfg.PeriodicSyncCount = 4;
	cfg.PawrSyncCount = 4;
	CHECK(Start(&cfg));

	CHECK(FindCfg(SDC_CFG_TYPE_PERIODIC_SYNC_COUNT) == nullptr);
	CHECK(FindCfg(SDC_CFG_TYPE_PERIODIC_SYNC_RSP_TX_BUFFER_CFG) == nullptr);

	Reset();
	cfg = MakeCfg(BT_GAP_ROLE_OBSERVER);
	cfg.PeriodicAdvCount = 4;
	cfg.PawrAdvCount = 4;
	CHECK(Start(&cfg));

	CHECK(FindCfg(SDC_CFG_TYPE_PERIODIC_ADV_COUNT) == nullptr);
	CHECK(FindCfg(SDC_CFG_TYPE_PERIODIC_ADV_RSP_COUNT) == nullptr);
}

// Each tag is set once. A tag set twice would mean the second write silently
// replacing the first, which is how a reservation disappears without any call
// failing.
void TestEachTagIsSetOnce(void)
{
	Reset();

	BtHciCtlrCfg_t cfg = MakeCfg(BT_GAP_ROLE_PERIPHERAL | BT_GAP_ROLE_CENTRAL);
	cfg.PeriodicAdvCount = 1;
	cfg.PeriodicSyncCount = 1;
	cfg.PawrAdvCount = 1;
	cfg.PawrSyncCount = 1;
	CHECK(Start(&cfg));

	const uint8_t tags[] = {
		SDC_CFG_TYPE_BUFFER_CFG,
		SDC_CFG_TYPE_PERIPHERAL_COUNT,
		SDC_CFG_TYPE_CENTRAL_COUNT,
		SDC_CFG_TYPE_ADV_COUNT,
		SDC_CFG_TYPE_ADV_BUFFER_CFG,
		SDC_CFG_TYPE_SCAN_BUFFER_CFG,
		SDC_CFG_TYPE_PERIODIC_ADV_COUNT,
		SDC_CFG_TYPE_PERIODIC_SYNC_COUNT,
		SDC_CFG_TYPE_PERIODIC_SYNC_BUFFER_CFG,
		SDC_CFG_TYPE_PERIODIC_ADV_RSP_COUNT,
		SDC_CFG_TYPE_PERIODIC_ADV_RSP_BUFFER_CFG,
		SDC_CFG_TYPE_PERIODIC_SYNC_RSP_TX_BUFFER_CFG,
	};

	for (size_t i = 0; i < sizeof(tags) / sizeof(tags[0]); i++)
	{
		CHECK(CountCfg(tags[i]) == 1);
	}
}

// Every reservation is checked, so a controller that refuses one stops the
// bring-up rather than running on a configuration it did not get. The
// periodic ones are the newest and the easiest to leave unchecked.
void TestARefusedReservationStopsTheBringUp(void)
{
	const uint8_t tags[] = {
		SDC_CFG_TYPE_BUFFER_CFG,
		SDC_CFG_TYPE_PERIPHERAL_COUNT,
		SDC_CFG_TYPE_ADV_COUNT,
		SDC_CFG_TYPE_ADV_BUFFER_CFG,
		SDC_CFG_TYPE_PERIODIC_ADV_COUNT,
		SDC_CFG_TYPE_PERIODIC_ADV_RSP_COUNT,
		SDC_CFG_TYPE_PERIODIC_ADV_RSP_BUFFER_CFG,
	};

	for (size_t i = 0; i < sizeof(tags) / sizeof(tags[0]); i++)
	{
		Reset();
		s_FailCfgType = tags[i];

		BtHciCtlrCfg_t cfg = MakeCfg(BT_GAP_ROLE_PERIPHERAL);
		cfg.PeriodicAdvCount = 1;
		cfg.PawrAdvCount = 1;

		CHECK(Start(&cfg) == false);
		// The radio never came up, so nothing may have been enabled.
		CHECK(s_EnableCalls == 0);
	}

	const uint8_t scanTags[] = {
		SDC_CFG_TYPE_CENTRAL_COUNT,
		SDC_CFG_TYPE_SCAN_BUFFER_CFG,
		SDC_CFG_TYPE_PERIODIC_SYNC_COUNT,
		SDC_CFG_TYPE_PERIODIC_SYNC_BUFFER_CFG,
		SDC_CFG_TYPE_PERIODIC_SYNC_RSP_TX_BUFFER_CFG,
	};

	for (size_t i = 0; i < sizeof(scanTags) / sizeof(scanTags[0]); i++)
	{
		Reset();
		s_FailCfgType = scanTags[i];

		BtHciCtlrCfg_t cfg = MakeCfg(BT_GAP_ROLE_OBSERVER);
		cfg.PeriodicSyncCount = 1;
		cfg.PawrSyncCount = 1;

		CHECK(Start(&cfg) == false);
		CHECK(s_EnableCalls == 0);
	}
}

// The reservations grow what the controller needs out of the memory pool, and
// the last sdc_cfg_set reports that number. Turning periodic advertising on is
// the case that makes a pool that was large enough stop being large enough, so
// the comparison has to stop the bring-up rather than hand sdc_enable a pool
// it will run past.
void TestAnUndersizedPoolStopsTheBringUp(void)
{
	Reset();
	// One octet more than the pool holds.
	s_CfgRam = 10001;

	BtHciCtlrCfg_t cfg = MakeCfg(BT_GAP_ROLE_PERIPHERAL);
	cfg.PeriodicAdvCount = 1;

	CHECK(Start(&cfg) == false);
	CHECK(s_EnableCalls == 0);
	// The arbiter is started after the pool check, so a refusal here leaves no
	// timeslot session open.
	CHECK(s_ArbiterStartCalls == 0);

	// The refusal names itself. A target build has no other way to learn the
	// size to set the pool from, since the traces here are compiled out unless
	// the build defines DEBUG_ENABLE and no project does.
	CHECK(BtHciCtlrErrorGet() == BT_HCI_CTLR_ERROR_MEM_POOL);
	CHECK(BtHciCtlrErrorValueGet() == 10001);

	// Exactly the pool size is enough.
	Reset();
	s_CfgRam = 10000;
	cfg = MakeCfg(BT_GAP_ROLE_PERIPHERAL);
	cfg.PeriodicAdvCount = 1;

	CHECK(Start(&cfg));
	CHECK(s_EnableCalls == 1);
}

// The ordering the bring-up depends on, each step checked. sdc_init answers
// Operation Not Permitted when MPSL is not up, which is what it had been doing
// on every boot while the result went unread.
void TestBringUpOrderAndFailures(void)
{
	Reset();

	BtHciCtlrCfg_t cfg = MakeCfg(BT_GAP_ROLE_PERIPHERAL);
	CHECK(Start(&cfg));
	CHECK(s_MpslInitCalls == 1);
	CHECK(s_InitCalls == 1);
	CHECK(s_RandSourceCalls == 1);
	CHECK(s_ArbiterStartCalls == 1);
	CHECK(s_EnableCalls == 1);
	CHECK(s_ArbiterStopCalls == 0);
	CHECK(s_EventLength == 11250);
	CHECK(s_EventExtended);

	// MPSL first, and nothing after it if it fails.
	Reset();
	s_FailMpslInit = true;
	cfg = MakeCfg(BT_GAP_ROLE_PERIPHERAL);
	CHECK(Start(&cfg) == false);
	CHECK(s_InitCalls == 0);
	CHECK(s_EnableCalls == 0);

	Reset();
	s_FailInit = true;
	cfg = MakeCfg(BT_GAP_ROLE_PERIPHERAL);
	CHECK(Start(&cfg) == false);
	CHECK(s_CfgCount == 0);
	CHECK(s_EnableCalls == 0);

	// Without an entropy source the controller cannot produce the random
	// numbers pairing and private addresses need.
	Reset();
	s_FailRandSource = true;
	cfg = MakeCfg(BT_GAP_ROLE_PERIPHERAL);
	CHECK(Start(&cfg) == false);
	CHECK(s_CfgCount == 0);
	CHECK(s_EnableCalls == 0);

	Reset();
	s_FailArbiter = true;
	cfg = MakeCfg(BT_GAP_ROLE_PERIPHERAL);
	CHECK(Start(&cfg) == false);
	CHECK(s_EnableCalls == 0);

	// A radio that never started leaves the timeslot session with nothing to
	// arbitrate against, so the session is closed again.
	Reset();
	s_FailEnable = true;
	cfg = MakeCfg(BT_GAP_ROLE_PERIPHERAL);
	CHECK(Start(&cfg) == false);
	CHECK(s_ArbiterStartCalls == 1);
	CHECK(s_ArbiterStopCalls == 1);
	// Failed post-enable HCI configuration must tear down the live controller.
	for (uint8_t failure = 1; failure <= 2; failure++)
	{
		Reset();
		s_FailEventConfig = failure;
		CHECK(Start(&cfg) == false);
		CHECK(s_EnableCalls == 1);
		CHECK(s_DisableCalls == 1);
		CHECK(!s_Enabled);
		CHECK(s_ArbiterStopCalls == 1);
		CHECK(BtHciCtlrErrorGet() == BT_HCI_CTLR_ERROR_CTLR_INIT);
	}

}

// The deliberate negative, so the checks above are known to be able to fail.
void TestNullArgumentsAreRefused(void)
{
	Reset();

	BtHciCtlrCfg_t cfg = MakeCfg(BT_GAP_ROLE_PERIPHERAL);
	BtHciCtlrDev_t dev = {};

	CHECK(BtHciCtlrStart(nullptr, &cfg) == false);
	CHECK(BtHciCtlrStart(&dev, nullptr) == false);
	CHECK(s_MpslInitCalls == 0);
	CHECK(s_InitCalls == 0);
}

// The commands the generic host issues at start on every controller:
// the ACL buffer sizes, the address the controller identifies itself with
// and random numbers. Each has to reach the SDC entry point of the same
// meaning, and Read BD_ADDR has to answer the factory address of the FICR,
// static random marker included, so an nRF5x keeps the address it has always
// advertised with.
void TestHostStartCommandsAreAnswered(void)
{
	Reset();

	uint8_t ret[8] = {0};
	g_SdcHciCmdLast = nullptr;
	g_SdcHciCmdCount = 0;

	CHECK(BtHciCtlrCommand(nullptr, BT_HCI_CMD_CTLR_READ_BUFF_SIZE, nullptr, 0, ret, 3) == 0);
	CHECK(g_SdcHciCmdLast != nullptr && std::strcmp(g_SdcHciCmdLast, "le_read_buffer_size") == 0);

	CHECK(BtHciCtlrCommand(nullptr, BT_HCI_CMD_CTLR_RAND, nullptr, 0, ret, sizeof(ret)) == 0);
	CHECK(g_SdcHciCmdLast != nullptr && std::strcmp(g_SdcHciCmdLast, "le_rand") == 0);

	int before = g_SdcHciCmdCount;
	uint8_t addr[6] = {0};
	CHECK(BtHciCtlrCommand(nullptr, BT_HCI_CMD_VS_ISYST_READ_STATIC_ADDR, nullptr, 0, addr, sizeof(addr)) == 0);
	// The I-SYST static address is answered from the FICR, not from a
	// controller entry point
	CHECK(g_SdcHciCmdCount == before);
	CHECK(addr[0] == 0xA5 && addr[4] == 0xA5);
	CHECK((addr[5] & 0xC0) == 0xC0);

	// A short return buffer takes what fits and nothing past it
	uint8_t one[2] = {0x11, 0x22};
	CHECK(BtHciCtlrCommand(nullptr, BT_HCI_CMD_VS_ISYST_READ_STATIC_ADDR, nullptr, 0, one, 1) == 0);
	CHECK(one[0] == 0xA5 && one[1] == 0x22);

	// The public address is the controller's to answer
	CHECK(BtHciCtlrCommand(nullptr, BT_HCI_CMD_INFO_READ_BD_ADDR, nullptr, 0, addr, sizeof(addr)) == 0);
	CHECK(g_SdcHciCmdLast != nullptr && std::strcmp(g_SdcHciCmdLast, "ip_read_bd_addr") == 0);

	// The crypto commands are not the SDC port's, the host has its engines
	CHECK(BtHciCtlrCommand(nullptr, BT_HCI_CMD_VS_ISYST_CRYPTO_SUPPORTED, nullptr, 0, addr, 4) != 0);

	// A command the controller does not have is answered 0xFF
	CHECK(BtHciCtlrCommand(nullptr, BT_HCI_CMD_BASEBAND_RESET, nullptr, 0, nullptr, 0) == 0xFF);
}

void LowIrq(void)
{
	CHECK(!s_InCommand);
	s_IrqCalls++;
}

void CommandInterrupt(void)
{
	s_InCommand = true;
	// A pending MPSL software interrupt must wait until the vendor call exits.
	CHECK(s_Basepri != 0 && s_Basepri <= (s_LowPriority << 5));
	if (s_Basepri == 0 || s_Basepri > (s_LowPriority << 5))
	{
		LowIrq();
	}
	else
	{
		s_PendingIrq = LowIrq;
	}
	s_InCommand = false;
}

void ReconnectRx(BtHciCtlrDev_t *, bool bEvent, uint8_t *pPacket)
{
	uint8_t *p = (uint8_t *)pPacket;
	uint16_t opcode = p[0] == 5 ? BT_HCI_CMD_CTLR_SET_EXT_ADV_ENABLE :
		BT_HCI_CMD_CTLR_SET_PHY;

	CHECK(bEvent);
	CHECK(s_Basepri == 0);
	CHECK(BtHciCtlrCommand(nullptr, opcode, nullptr, 0, nullptr, 0) == 0);
}

void TestControllerSerialization(void)
{
	BtHciCtlrCfg_t cfg = MakeCfg(BT_GAP_ROLE_PERIPHERAL);
	BtHciCtlrDev_t dev = {};
	uint8_t data[4] = {};
	int before;

	Reset();
	BtHciCtlrPeripheralSupport();
	BtHciCtlrLinkSupport();
	CHECK(BtHciCtlrStart(&dev, &cfg));
	g_SdcHciCmdHook = CommandInterrupt;
	CHECK(BtHciCtlrCommand(nullptr, BT_HCI_CMD_CTLR_SET_PHY, data, sizeof(data), nullptr, 0) == 0);
	CHECK(s_IrqCalls == 1);
	CHECK(s_Basepri == 0);
	g_SdcHciCmdStatus = 0x0c;
	CHECK(BtHciCtlrCommand(nullptr, BT_HCI_CMD_CTLR_SET_PHY, data, sizeof(data), nullptr, 0) == 0x0c);
	CHECK(s_Basepri == 0);
	g_SdcHciCmdStatus = 0;

	// A stronger caller mask must survive success and unsupported commands.
	s_Basepri = 3 << 5;
	CHECK(BtHciCtlrCommand(nullptr, BT_HCI_CMD_CTLR_SET_PHY, data, sizeof(data), nullptr, 0) == 0);
	CHECK(s_Basepri == (3 << 5));
	CHECK(BtHciCtlrCommand(nullptr, 0xffff, nullptr, 0, nullptr, 0) == 0xff);
	CHECK(s_Basepri == (3 << 5));
	__set_BASEPRI(0);
	g_SdcHciCmdHook = nullptr;

	CHECK(dev.Send(&dev, data, sizeof(data)) == sizeof(data));
	s_DataPutResult = -1;
	CHECK(dev.Send(&dev, data, sizeof(data)) == 0);
	CHECK(s_Basepri == 0);

	// Higher priority application IRQs cannot reenter a suspended MPSL call.
	before = g_SdcHciCmdCount;
	s_Exception = 16 + 2;
	CHECK(BtHciCtlrCommand(nullptr, BT_HCI_CMD_CTLR_SET_PHY, data, sizeof(data), nullptr, 0) == BT_HCI_ERR_COMMAND_DISALLOWED);
	CHECK(g_SdcHciCmdCount == before);
	before = s_DataPutCalls;
	CHECK(dev.Send(&dev, data, sizeof(data)) == 0);
	CHECK(s_DataPutCalls == before);
	s_Exception = 16 + SWI5_EGU5_IRQn;
	CHECK(BtHciCtlrCommand(nullptr, BT_HCI_CMD_CTLR_SET_PHY, data, sizeof(data), nullptr, 0) == 0);
	s_Exception = 0;

	// Deliver disconnect/reconnect events; host callbacks can restart
	// advertising and request PHY without retaining the receive-call mask.
	dev.RxHandler = ReconnectRx;
	s_RxRemaining = 2;
	BtHciCtlrProcess(&dev);
	CHECK(s_RxRemaining == 0);
	CHECK(s_GetCalls == 3);
	CHECK(s_Basepri == 0);

	// The guard follows the configured priority, including the nRF54L value.
	s_LowPriority = 4;
	g_SdcHciCmdHook = CommandInterrupt;
	CHECK(BtHciCtlrCommand(nullptr, BT_HCI_CMD_CTLR_SET_PHY, data, sizeof(data), nullptr, 0) == 0);
	CHECK(s_Basepri == 0);
	g_SdcHciCmdHook = nullptr;
}

#ifdef SDC_TEST_NO_FEATURE_MODULES
// The same setup in an application that links neither the scan module nor the
// periodic advertising module. The controller port finds no reference to
// them, so it must not enable those controller features, and a role that
// needs scanning must stop the bring up instead of running without it.
void TestFeatureModulesNotLinked(void)
{
	Reset();

	BtHciCtlrCfg_t cfg = MakeCfg(BT_GAP_ROLE_PERIPHERAL);
	CHECK(Start(&cfg));

	CHECK(Supported("adv"));
	CHECK(Supported("ext_adv"));
	CHECK(Supported("peripheral"));
	CHECK(Supported("le_periodic_adv") == false);
	CHECK(Supported("le_periodic_adv_with_rsp") == false);
	CHECK(Supported("scan") == false);
	CHECK(Supported("central") == false);

	Reset();
	cfg = MakeCfg(BT_GAP_ROLE_OBSERVER);
	CHECK(Start(&cfg) == false);
	CHECK(BtHciCtlrErrorGet() == BT_HCI_CTLR_ERROR_CENTRAL_SUPPORT);
	CHECK(Supported("scan") == false);
	CHECK(s_EnableCalls == 0);

	Reset();
	cfg = MakeCfg(BT_GAP_ROLE_PERIPHERAL | BT_GAP_ROLE_CENTRAL);
	CHECK(Start(&cfg) == false);
	CHECK(BtHciCtlrErrorGet() == BT_HCI_CTLR_ERROR_CENTRAL_SUPPORT);
	CHECK(s_EnableCalls == 0);
}
#endif

}	// namespace

#ifndef SDC_TEST_NO_FEATURE_MODULES
// The controller port enables the scan side and periodic advertising only in
// an application that links the scan module (BtAppScanInit) and the periodic
// advertising module (BtPadvInit). Each module defines one of these
// references. This test stands in for an application that uses both. The
// build with SDC_TEST_NO_FEATURE_MODULES is the application that uses neither.
extern "C" void (* const g_pBtHciCtlrCentralSupport)(void) = BtHciCtlrCentralSupport;
extern "C" void (* const g_pBtHciCtlrPeriodicAdvSupport)(void) = BtHciCtlrPeriodicAdvSupport;
#endif

// --- the vendor entry points ------------------------------------------------


extern "C" int32_t sdc_init(sdc_fault_handler_t)
{
	s_InitCalls++;

	return s_FailInit ? kVendorRefused : 0;
}

extern "C" int32_t sdc_rand_source_register(const sdc_rand_source_t *)
{
	s_RandSourceCalls++;

	return s_FailRandSource ? kVendorRefused : 0;
}

extern "C" int32_t sdc_cfg_set(uint8_t, uint8_t Type, const sdc_cfg_t *pCfg)
{
	if (s_CfgCount < kMaxCfg)
	{
		s_Cfg[s_CfgCount].Type = Type;
		if (pCfg != nullptr)
		{
			s_Cfg[s_CfgCount].Cfg = *pCfg;
		}
		s_CfgCount++;
	}

	if (s_FailCfgType != 0 && Type == s_FailCfgType)
	{
		return kVendorRefused;
	}

	return s_CfgRam;
}

extern "C" int32_t sdc_enable(sdc_callback_t, uint8_t *)
{
	s_EnableCalls++;
	s_Enabled = !s_FailEnable;

	return s_FailEnable ? kVendorRefused : 0;
}

extern "C" int32_t sdc_disable(void)
{
	s_Enabled = false;
	s_DisableCalls++;
	return 0;
}

extern "C" int32_t sdc_default_tx_power_set(int8_t)
{
	return 0;
}

// The low frequency timer of the link timeouts. Not started on a host.
extern "C" bool nRFxLFTimerInit(TimerDev_t * const, const TimerCfg_t * const)
{
	return false;
}

extern "C" int32_t sdc_hci_get(uint8_t *pData, uint8_t *pType)
{
	s_GetCalls++;
	CHECK(s_Basepri != 0 && s_Basepri <= (s_LowPriority << 5));
	if (s_RxRemaining == 0)
	{
		return kVendorRefused;
	}
	*pType = 1;
	pData[0] = s_RxRemaining == 2 ? 5 : 0x3e;
	s_RxRemaining--;
	return 0;
}

extern "C" int32_t sdc_hci_data_put(uint8_t *)
{
	s_DataPutCalls++;
	CHECK(s_Basepri != 0 && s_Basepri <= (s_LowPriority << 5));
	return s_DataPutResult;
}

extern "C" uint8_t sdc_hci_cmd_vs_conn_event_extend(const sdc_hci_cmd_vs_conn_event_extend_t *pParams)
{
	if (!s_Enabled || s_FailEventConfig == 1)
	{
		return 0x0c;
	}
	s_EventExtended = pParams->enable == 1;
	return s_EventExtended ? 0 : 0x12;
}

extern "C" uint8_t sdc_hci_cmd_vs_event_length_set(const sdc_hci_cmd_vs_event_length_set_t *pParams)
{
	if (!s_Enabled || s_FailEventConfig == 2)
	{
		return 0x0c;
	}
	s_EventLength = pParams->event_length_us;
	return 0;
}

// Each support call records its own name, so the test reads the same way the
// vendor header does.
#define SDC_SUPPORT_STUB(name) \
	void sdc_support_##name(void) \
	{ \
		if (s_SupportCount < kMaxSupport) \
		{ \
			s_Support[s_SupportCount++] = #name; \
		} \
	}

extern "C" SDC_SUPPORT_STUB(adv)
SDC_SUPPORT_STUB(ext_adv)
SDC_SUPPORT_STUB(scan)
SDC_SUPPORT_STUB(ext_scan)
SDC_SUPPORT_STUB(peripheral)
SDC_SUPPORT_STUB(central)
SDC_SUPPORT_STUB(ext_central)
SDC_SUPPORT_STUB(dle_peripheral)
SDC_SUPPORT_STUB(dle_central)
SDC_SUPPORT_STUB(phy_update_peripheral)
SDC_SUPPORT_STUB(phy_update_central)
SDC_SUPPORT_STUB(le_2m_phy)
SDC_SUPPORT_STUB(le_coded_phy)
SDC_SUPPORT_STUB(le_power_control)
SDC_SUPPORT_STUB(le_power_control_peripheral)
SDC_SUPPORT_STUB(le_power_control_central)
SDC_SUPPORT_STUB(le_conn_cte_rsp_peripheral)
SDC_SUPPORT_STUB(le_conn_cte_rsp_central)
SDC_SUPPORT_STUB(le_periodic_adv)
SDC_SUPPORT_STUB(le_periodic_sync)
SDC_SUPPORT_STUB(le_periodic_adv_with_rsp)
SDC_SUPPORT_STUB(le_periodic_sync_with_rsp)

bool MpslInit(void)
{
	s_MpslInitCalls++;

	return s_FailMpslInit == false;
}

extern "C" int MpslNvmArbiterStart(void)
{
	s_ArbiterStartCalls++;

	return s_FailArbiter ? -1 : 0;
}

extern "C" void MpslNvmArbiterStop(void)
{
	s_ArbiterStopCalls++;
}


// The controller reaches the entropy source through the crypto engine facet.
// Nothing in the bring-up draws from it, so these only have to exist. Defining
// the out of line virtuals here is what puts the class vtable in this build
// instead of pulling the nrfx driver in for it.
bool CryptoRngNrf::Init(DeviceIntrf * const)
{
	return false;
}

bool CryptoRngNrf::Enable()
{
	// Written because this is a member definition and can reach it.
	// rng_nrfx.cpp is what really uses the field; without a write here it
	// reads as a dead private field in this build alone.
	vbIntrfEnabled = false;

	return false;
}

void CryptoRngNrf::Disable()
{
}

void CryptoRngNrf::Reset()
{
}

CRYPTO_STATUS CryptoRngNrf::Random(uint8_t *, size_t)
{
	return CRYPTO_STATUS_FAIL;
}

CryptoRngNrf *CryptoRngNrfInstance(void)
{
	static CryptoRngNrf s_Rng;

	return &s_Rng;
}

int main(void)
{
	TestPeripheralSupportIsOnRequest();

#ifdef SDC_TEST_NO_FEATURE_MODULES
	TestFeatureModulesNotLinked();

	if (s_Failures != 0)
	{
		std::printf("SDC controller setup without feature modules: %d failure(s), %d checks\n",
					s_Failures, s_Checks);
		return 1;
	}

	std::printf("SDC controller setup without feature modules: PASS (%d checks)\n", s_Checks);

	return 0;
#else
	TestSupportCallsFollowTheRole();
	TestSupportOrderMeetsThePrerequisites();
	TestPeriodicResourcesAreReservedOnRequest();
	TestPawrResourcesAreReservedOnRequest();
	TestCountsOutsideTheirRoleReserveNothing();
	TestEachTagIsSetOnce();
	TestARefusedReservationStopsTheBringUp();
	TestAnUndersizedPoolStopsTheBringUp();
	TestBringUpOrderAndFailures();
	TestNullArgumentsAreRefused();
	TestHostStartCommandsAreAnswered();
	TestControllerSerialization();

	if (s_Failures != 0)
	{
		std::printf("SDC controller setup tests: %d failure(s), %d checks\n",
					s_Failures, s_Checks);
		return 1;
	}

	std::printf("SDC controller setup tests: PASS (%d checks)\n", s_Checks);

	return 0;
#endif
}

// Startup trace is enabled in the diagnostic build; this test has no sink.
#include "syslog.h"
extern "C" SysLog_t *SysLogGet(void) { return nullptr; }
extern "C" int SysLogPrintf(SysLog_t * const, const char *, ...) { return 0; }

uint32_t NVIC_GetPriority(IRQn_Type Irq)
{
	return Irq == SWI5_EGU5_IRQn ? s_LowPriority : 2;
}

uint32_t __get_IPSR(void) { return s_Exception; }
uint32_t __get_BASEPRI(void) { return s_Basepri; }
void __set_BASEPRI_MAX(uint32_t Value)
{
	if (s_Basepri == 0 || (Value != 0 && Value < s_Basepri))
	{
		s_Basepri = Value;
	}
}
void __set_BASEPRI(uint32_t Value)
{
	s_Basepri = Value;
	if (s_PendingIrq != nullptr &&
		(Value == 0 || Value > (s_LowPriority << 5)))
	{
		void (*irq)(void) = s_PendingIrq;
		s_PendingIrq = nullptr;
		irq();
	}
}
