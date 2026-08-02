#include <errno.h>
#include <poll.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include "bluetooth/bt_gap.h"
#include "bluetooth/bt_hci.h"
#include "bluetooth/bt_l2cap.h"
#include "bluetooth/bt_peer.h"
#include "bluetooth/bt_smp.h"
#include "bt_compliance.h"
#include "bt_virtual_le_link.h"
#include "bt_virtual_oob.h"

namespace {

using btcompliance::Context;
using btcompliance::LeConnectionCase;
using btcompliance::OobFault;
using btcompliance::Requirement;
using btcompliance::ScOobData;
using btcompliance::VirtualLeLink;
using btcompliance::VirtualOobChannel;

constexpr uint16_t TEST_CONN_HDL = 1;
constexpr size_t WIRE_DATA_SIZE = 96;
constexpr int POLL_TIMEOUT_MS = 2000;
constexpr int MAX_EVENT_STEPS = 20000;

enum WireType : uint8_t {
	WIRE_CMD_INIT = 1,
	WIRE_CMD_GENERATE_OOB,
	WIRE_CMD_SET_PEER_OOB,
	WIRE_CMD_START_PAIRING,
	WIRE_CMD_REQUEST_SECURITY,
	WIRE_CMD_RX_SMP,
	WIRE_CMD_LTK_REQUEST,
	WIRE_CMD_ENCRYPTED,
	WIRE_CMD_NUMERIC_REPLY,
	WIRE_CMD_PASSKEY_REPLY,
	WIRE_CMD_SET_TX_BLOCK,
	WIRE_CMD_PUMP_TX,
	WIRE_CMD_STOP,

	WIRE_EVT_READY = 64,
	WIRE_EVT_OOB_DATA,
	WIRE_EVT_TX_SMP,
	WIRE_EVT_ENABLE_ENCRYPTION,
	WIRE_EVT_LTK_REPLY,
	WIRE_EVT_LTK_NEG_REPLY,
	WIRE_EVT_NUMERIC,
	WIRE_EVT_PASSKEY_DISPLAY,
	WIRE_EVT_PASSKEY_REQUEST,
	WIRE_EVT_PAIRING_COMPLETE,
	WIRE_EVT_ERROR,
};

struct WireMessage {
	uint8_t Type;
	uint8_t Length;
	uint16_t Code;
	uint32_t Value;
	uint8_t Data[WIRE_DATA_SIZE];
};

static bool WriteAll(int Fd, const void *pData, size_t Len)
{
	const uint8_t *p = static_cast<const uint8_t *>(pData);
	while (Len != 0)
	{
		ssize_t written = write(Fd, p, Len);
		if (written < 0 && errno == EINTR)
		{
			continue;
		}
		if (written <= 0)
		{
			return false;
		}
		p += written;
		Len -= static_cast<size_t>(written);
	}
	return true;
}

static bool ReadAll(int Fd, void *pData, size_t Len)
{
	uint8_t *p = static_cast<uint8_t *>(pData);
	while (Len != 0)
	{
		ssize_t count = read(Fd, p, Len);
		if (count < 0 && errno == EINTR)
		{
			continue;
		}
		if (count <= 0)
		{
			return false;
		}
		p += count;
		Len -= static_cast<size_t>(count);
	}
	return true;
}

static bool SendWire(int Fd, const WireMessage &Message)
{
	return WriteAll(Fd, &Message, sizeof(Message));
}

static bool ReceiveWire(int Fd, WireMessage *pMessage)
{
	return pMessage != nullptr && ReadAll(Fd, pMessage, sizeof(*pMessage));
}


static uint64_t LoadLe64(const uint8_t *p)
{
	uint64_t value = 0;
	for (int i = 7; i >= 0; i--)
	{
		value = (value << 8) | p[i];
	}
	return value;
}

static void StoreLe64(uint8_t *p, uint64_t Value)
{
	for (int i = 0; i < 8; i++)
	{
		p[i] = static_cast<uint8_t>(Value >> (i * 8));
	}
}

struct TestKeyContext {
	uint8_t PublicKey[64];
};

class TestCrypto final : public KeyAgreeEngine, public CipherEngine, public RngEngine {
public:
	TestCrypto() : vState(0xC001D00DU) {}

	void Seed(uint32_t SeedValue)
	{
		vState = SeedValue != 0 ? SeedValue : 1;
	}

	bool Enable() override { return true; }
	void Disable() override {}
	void Reset() override {}
	int SelfTest() override { return 0; }

	size_t KeyCtxSize() const override { return sizeof(TestKeyContext); }
	size_t KeyCtxAlign() const override { return alignof(TestKeyContext); }

	void KeyReset(void *pKeyCtx) override
	{
		if (pKeyCtx != nullptr)
		{
			volatile uint8_t *p = static_cast<volatile uint8_t *>(pKeyCtx);
			for (size_t i = 0; i < sizeof(TestKeyContext); i++)
			{
				p[i] = 0;
			}
		}
	}

	CRYPTO_STATUS KeyGen(CRYPTO_CURVE Curve, void *pKeyCtx,
		uint8_t *pPublicKey) override
	{
		if (Curve != CRYPTO_CURVE_P256 || pKeyCtx == nullptr || pPublicKey == nullptr)
		{
			return CRYPTO_STATUS_FAIL;
		}
		TestKeyContext *pCtx = static_cast<TestKeyContext *>(pKeyCtx);
		for (size_t i = 0; i < sizeof(pCtx->PublicKey); i++)
		{
			pCtx->PublicKey[i] = static_cast<uint8_t>(Next() | 1U);
		}
		memcpy(pPublicKey, pCtx->PublicKey, sizeof(pCtx->PublicKey));
		return CRYPTO_STATUS_OK;
	}

	CRYPTO_STATUS Agree(CRYPTO_CURVE Curve, void *pKeyCtx,
		const uint8_t *pPeerPublicKey, uint8_t *pSharedX,
		bool bKeepKey = false) override
	{
		if (Curve != CRYPTO_CURVE_P256 || pKeyCtx == nullptr ||
			pPeerPublicKey == nullptr || pSharedX == nullptr)
		{
			return CRYPTO_STATUS_FAIL;
		}
		TestKeyContext *pCtx = static_cast<TestKeyContext *>(pKeyCtx);
		for (size_t i = 0; i < 32; i++)
		{
			pSharedX[i] = static_cast<uint8_t>(
				pCtx->PublicKey[i] ^ pPeerPublicKey[i] ^
				pCtx->PublicKey[32 + i] ^ pPeerPublicKey[32 + i] ^ 0x5AU);
		}
		if (!bKeepKey)
		{
			KeyReset(pKeyCtx);
		}
		return CRYPTO_STATUS_OK;
	}

	CRYPTO_STATUS Cipher(CRYPTO_CIPHER_ALG Alg, int bEncrypt,
		const CryptoKey &Key, const uint8_t *pIv, size_t IvLen,
		const uint8_t *pInput, size_t Len, uint8_t *pOutput) override
	{
		(void)bEncrypt;
		(void)pIv;
		(void)IvLen;
		if (Alg != CRYPTO_CIPHER_ECB || Key.Type != CRYPTO_KEY_AES_128 ||
			Key.Loc != CRYPTO_KEY_LOC_PLAIN || Key.Plain.pData == nullptr ||
			Key.Plain.Len != 16 || pInput == nullptr || pOutput == nullptr || Len != 16)
		{
			return CRYPTO_STATUS_FAIL;
		}

		uint8_t state[16];
		uint8_t next[16];
		memcpy(state, pInput, sizeof(state));
		for (uint8_t round = 0; round < 6; round++)
		{
			for (uint8_t i = 0; i < 16; i++)
			{
				uint8_t a = state[(i + 1U) & 15U];
				uint8_t b = state[(i + 5U) & 15U];
				uint8_t k = Key.Plain.pData[(i + round) & 15U];
				uint8_t shift = static_cast<uint8_t>((i % 7U) + 1U);
				uint8_t mixed = static_cast<uint8_t>(a + k + round + i);
				next[i] = static_cast<uint8_t>((mixed << shift) |
					(mixed >> (8U - shift)));
				next[i] ^= b;
			}
			memcpy(state, next, sizeof(state));
		}
		memcpy(pOutput, state, sizeof(state));
		return CRYPTO_STATUS_OK;
	}

	CRYPTO_STATUS Random(uint8_t *pOutput, size_t Len) override
	{
		if (pOutput == nullptr && Len != 0)
		{
			return CRYPTO_STATUS_FAIL;
		}
		for (size_t i = 0; i < Len; i++)
		{
			pOutput[i] = static_cast<uint8_t>(Next() >> 24);
		}
		return CRYPTO_STATUS_OK;
	}

	bool IsSecure() const override { return true; }

private:
	uint32_t Next()
	{
		uint32_t x = vState;
		x ^= x << 13;
		x ^= x >> 17;
		x ^= x << 5;
		vState = x;
		return x;
	}

	uint32_t vState;
};

struct WorkerState {
	int CommandFd;
	int EventFd;
	uint8_t Role;
	uint8_t IoCaps;
	uint8_t AuthReq;
	uint8_t LocalAddrType;
	uint8_t LocalAddr[6];
	BtHciDevice_t Hci;
	BtDevice_t Peer;
	TestCrypto Crypto;
	uint8_t LocalIrk[16];
	bool TxBlocked;
};

static WorkerState *s_pWorker = nullptr;

static bool WorkerEmit(uint8_t Type, uint16_t Code, uint32_t Value,
	const void *pData, size_t Len)
{
	if (s_pWorker == nullptr || Len > WIRE_DATA_SIZE || (Len != 0 && pData == nullptr))
	{
		return false;
	}
	WireMessage message = {};
	message.Type = Type;
	message.Length = static_cast<uint8_t>(Len);
	message.Code = Code;
	message.Value = Value;
	if (Len != 0)
	{
		memcpy(message.Data, pData, Len);
	}
	return SendWire(s_pWorker->EventFd, message);
}

static uint8_t WorkerHciCommand(BtHciDevice_t * const, uint16_t,
	const void *, uint8_t, void *, uint8_t)
{
	return 0;
}

static bool WorkerInit(WorkerState *pWorker, const WireMessage &Message)
{
	if (pWorker == nullptr || Message.Length < 17)
	{
		return false;
	}

	pWorker->Role = Message.Data[0];
	pWorker->IoCaps = Message.Data[1];
	pWorker->AuthReq = Message.Data[2];
	pWorker->LocalAddrType = Message.Data[3];
	uint8_t peerAddrType = Message.Data[4];
	memcpy(pWorker->LocalAddr, &Message.Data[5], 6);

	memset(&pWorker->Hci, 0, sizeof(pWorker->Hci));
	pWorker->Hci.pCtx = pWorker;
	pWorker->Hci.Command = WorkerHciCommand;

	memset(&pWorker->Peer, 0, sizeof(pWorker->Peer));
	pWorker->Peer.Conn.Hdl = TEST_CONN_HDL;
	pWorker->Peer.Conn.Role = pWorker->Role;
	pWorker->Peer.Conn.PeerAddrType = peerAddrType;
	memcpy(pWorker->Peer.Conn.PeerAddr, &Message.Data[11], 6);
	pWorker->Peer.Conn.OwnAddrType = pWorker->LocalAddrType;
	memcpy(pWorker->Peer.Conn.OwnAddr, pWorker->LocalAddr, 6);
	pWorker->Peer.pHciDev = &pWorker->Hci;

	for (size_t i = 0; i < sizeof(pWorker->LocalIrk); i++)
	{
		pWorker->LocalIrk[i] = static_cast<uint8_t>(0x80U + i + pWorker->Role);
	}
	pWorker->Crypto.Seed(pWorker->Role == BT_CONN_ROLE_CENTRAL ?
		0x13579BDFU : 0x2468ACE1U);
	if (!BtSmpInit(&pWorker->Crypto, &pWorker->Crypto, &pWorker->Crypto))
	{
		return false;
	}
	BtSmpAuthConfig(pWorker->IoCaps, pWorker->AuthReq);
	return true;
}

static int WorkerMain(int CommandFd, int EventFd)
{
	WorkerState worker = {};
	worker.CommandFd = CommandFd;
	worker.EventFd = EventFd;
	s_pWorker = &worker;

	for (;;)
	{
		WireMessage message = {};
		if (!ReceiveWire(CommandFd, &message))
		{
			break;
		}

		switch (message.Type)
		{
			case WIRE_CMD_INIT:
				if (!WorkerInit(&worker, message))
				{
					WorkerEmit(WIRE_EVT_ERROR, 1, 0, nullptr, 0);
					return 2;
				}
				WorkerEmit(WIRE_EVT_READY, 0, 0, nullptr, 0);
				break;

			case WIRE_CMD_GENERATE_OOB:
			{
				ScOobData data = {};
				int rc = BtSmpOobLocalDataGen(&worker.Hci, data.Random, data.Confirm);
				if (rc != 0)
				{
					WorkerEmit(WIRE_EVT_ERROR, 2, static_cast<uint32_t>(rc), nullptr, 0);
				}
				else
				{
					WorkerEmit(WIRE_EVT_OOB_DATA, 0, 0, &data, sizeof(data));
				}
				break;
			}

			case WIRE_CMD_SET_PEER_OOB:
				if (message.Length == sizeof(ScOobData))
				{
					const ScOobData *pData = reinterpret_cast<const ScOobData *>(message.Data);
					BtSmpOobPeerDataSet(pData->Random, pData->Confirm);
				}
				else
				{
					WorkerEmit(WIRE_EVT_ERROR, 3, message.Length, nullptr, 0);
				}
				break;

			case WIRE_CMD_START_PAIRING:
				BtSmpStartPairing(TEST_CONN_HDL);
				break;

			case WIRE_CMD_REQUEST_SECURITY:
				BtSmpRequestSecurity(TEST_CONN_HDL);
				break;

			case WIRE_CMD_RX_SMP:
			{
				if (message.Length == 0 || message.Length > sizeof(BtL2CapSmp_t))
				{
					WorkerEmit(WIRE_EVT_ERROR, 4, message.Length, nullptr, 0);
					break;
				}
				BtL2CapSmp_t smp = {};
				memcpy(&smp, message.Data, message.Length);
				BtProcessSmpData(&worker.Hci, TEST_CONN_HDL, &smp, message.Length);
				break;
			}

			case WIRE_CMD_LTK_REQUEST:
				if (message.Length >= 10)
				{
					uint64_t rand = LoadLe64(message.Data);
					uint16_t ediv = static_cast<uint16_t>(message.Data[8] |
						(static_cast<uint16_t>(message.Data[9]) << 8));
					BtSmpProcessLtkRequest(&worker.Hci, TEST_CONN_HDL, rand, ediv);
				}
				break;

			case WIRE_CMD_ENCRYPTED:
				BtSmpEncryptionChanged(&worker.Hci, TEST_CONN_HDL,
					static_cast<uint8_t>(message.Code), static_cast<uint8_t>(message.Value));
				break;

			case WIRE_CMD_NUMERIC_REPLY:
				BtSmpNumericComparisonReply(TEST_CONN_HDL, message.Value != 0);
				break;

			case WIRE_CMD_PASSKEY_REPLY:
				BtSmpPasskeyReply(TEST_CONN_HDL, message.Value);
				break;

			case WIRE_CMD_SET_TX_BLOCK:
				worker.TxBlocked = message.Value != 0;
				break;

			case WIRE_CMD_PUMP_TX:
				BtSmpTimeoutCheck();
				break;

			case WIRE_CMD_STOP:
				BtSmpDisconnected(TEST_CONN_HDL);
				return 0;

			default:
				WorkerEmit(WIRE_EVT_ERROR, 5, message.Type, nullptr, 0);
				break;
		}
	}
	return 0;
}

struct ChildPeer {
	pid_t Pid;
	int CommandFd;
	int EventFd;
	bool Complete;
	bool Success;
	bool Authenticated;
	bool SecureConnections;
	bool KeyValid;
	bool NumericSeen;
	uint32_t NumericValue;
	bool PasskeyDisplaySeen;
	uint32_t DisplayedPasskey;
	bool PasskeyRequestSeen;
	uint8_t FailureReason;
};

static bool SpawnPeer(ChildPeer *pPeer)
{
	int commandPipe[2];
	int eventPipe[2];
	if (pPeer == nullptr || pipe(commandPipe) != 0 || pipe(eventPipe) != 0)
	{
		return false;
	}

	pid_t pid = fork();
	if (pid < 0)
	{
		close(commandPipe[0]);
		close(commandPipe[1]);
		close(eventPipe[0]);
		close(eventPipe[1]);
		return false;
	}
	if (pid == 0)
	{
		close(commandPipe[1]);
		close(eventPipe[0]);
		int rc = WorkerMain(commandPipe[0], eventPipe[1]);
		close(commandPipe[0]);
		close(eventPipe[1]);
		_exit(rc);
	}

	close(commandPipe[0]);
	close(eventPipe[1]);
	memset(pPeer, 0, sizeof(*pPeer));
	pPeer->Pid = pid;
	pPeer->CommandFd = commandPipe[1];
	pPeer->EventFd = eventPipe[0];
	return true;
}

static void StopPeer(ChildPeer *pPeer)
{
	if (pPeer == nullptr || pPeer->Pid <= 0)
	{
		return;
	}
	WireMessage stop = {};
	stop.Type = WIRE_CMD_STOP;
	SendWire(pPeer->CommandFd, stop);
	close(pPeer->CommandFd);
	close(pPeer->EventFd);
	int status = 0;
	if (waitpid(pPeer->Pid, &status, 0) < 0)
	{
		kill(pPeer->Pid, SIGKILL);
		waitpid(pPeer->Pid, &status, 0);
	}
	pPeer->Pid = 0;
}

static bool SendInit(ChildPeer *pPeer, uint8_t Role, uint8_t IoCaps,
	uint8_t AuthReq, uint8_t LocalAddrType, const uint8_t LocalAddr[6],
	uint8_t PeerAddrType, const uint8_t PeerAddr[6])
{
	WireMessage message = {};
	message.Type = WIRE_CMD_INIT;
	message.Length = 17;
	message.Data[0] = Role;
	message.Data[1] = IoCaps;
	message.Data[2] = AuthReq;
	message.Data[3] = LocalAddrType;
	message.Data[4] = PeerAddrType;
	memcpy(&message.Data[5], LocalAddr, 6);
	memcpy(&message.Data[11], PeerAddr, 6);
	if (!SendWire(pPeer->CommandFd, message))
	{
		return false;
	}
	WireMessage ready = {};
	return ReceiveWire(pPeer->EventFd, &ready) && ready.Type == WIRE_EVT_READY;
}

static bool RequestOob(ChildPeer *pPeer, ScOobData *pData)
{
	WireMessage request = {};
	request.Type = WIRE_CMD_GENERATE_OOB;
	if (!SendWire(pPeer->CommandFd, request))
	{
		return false;
	}
	WireMessage response = {};
	if (!ReceiveWire(pPeer->EventFd, &response) ||
		response.Type != WIRE_EVT_OOB_DATA || response.Length != sizeof(*pData))
	{
		return false;
	}
	memcpy(pData, response.Data, sizeof(*pData));
	return true;
}

static bool SetPeerOob(ChildPeer *pPeer, const ScOobData &Data)
{
	WireMessage message = {};
	message.Type = WIRE_CMD_SET_PEER_OOB;
	message.Length = sizeof(Data);
	memcpy(message.Data, &Data, sizeof(Data));
	return SendWire(pPeer->CommandFd, message);
}

struct PairingRunConfig {
	uint8_t CentralIo;
	uint8_t PeripheralIo;
	bool Mitm;
	bool Oob;
	bool PeripheralStarts;
	OobFault CentralReceives;
	OobFault PeripheralReceives;
};

struct PairingRunResult {
	bool TransportOk;
	bool Complete;
	bool Success;
	bool Authenticated;
	bool Sc;
	bool Numeric;
	bool Passkey;
	bool Oob;
	uint8_t FailureReason;
	int Steps;
};

static uint8_t ExpectedModel(uint8_t CentralIo, uint8_t PeripheralIo,
	bool Mitm, bool Oob)
{
	static const uint8_t map[5][5] = {
		{ 0, 0, 2, 0, 2 },
		{ 0, 1, 2, 0, 1 },
		{ 2, 2, 2, 0, 2 },
		{ 0, 0, 0, 0, 0 },
		{ 2, 1, 2, 0, 1 },
	};
	if (Oob)
	{
		return BT_SMP_MODEL_OOB;
	}
	if (!Mitm)
	{
		return BT_SMP_MODEL_JUST_WORKS;
	}
	return map[CentralIo][PeripheralIo];
}

static bool ForwardSmp(ChildPeer *pDestination, const WireMessage &Event)
{
	WireMessage command = {};
	command.Type = WIRE_CMD_RX_SMP;
	command.Length = Event.Length;
	memcpy(command.Data, Event.Data, Event.Length);
	return SendWire(pDestination->CommandFd, command);
}

static bool SendNumericReply(ChildPeer *pPeer, bool Confirm)
{
	WireMessage message = {};
	message.Type = WIRE_CMD_NUMERIC_REPLY;
	message.Value = Confirm ? 1U : 0U;
	return SendWire(pPeer->CommandFd, message);
}

static bool SendPasskeyReply(ChildPeer *pPeer, uint32_t Passkey)
{
	WireMessage message = {};
	message.Type = WIRE_CMD_PASSKEY_REPLY;
	message.Value = Passkey;
	return SendWire(pPeer->CommandFd, message);
}

static void HandlePairingComplete(ChildPeer *pPeer, const WireMessage &Event)
{
	pPeer->Complete = true;
	pPeer->Success = Event.Value != 0;
	if (Event.Length >= 3)
	{
		pPeer->Authenticated = Event.Data[0] != 0;
		pPeer->SecureConnections = Event.Data[1] != 0;
		pPeer->KeyValid = Event.Data[2] != 0;
	}
}

static PairingRunResult RunPairing(const PairingRunConfig &Config)
{
	PairingRunResult result = {};
	ChildPeer central = {};
	ChildPeer peripheral = {};
	if (!SpawnPeer(&central) || !SpawnPeer(&peripheral))
	{
		StopPeer(&central);
		StopPeer(&peripheral);
		return result;
	}

	const uint8_t centralAddr[6] = { 0xC0, 0x10, 0x20, 0x30, 0x40, 0x50 };
	const uint8_t peripheralAddr[6] = { 0xD0, 0x11, 0x21, 0x31, 0x41, 0x51 };
	uint8_t auth = static_cast<uint8_t>(BT_SMP_AUTHREQ_BONDING_FLAG_BONDING |
		BT_SMP_AUTHREQ_SC | (Config.Mitm ? BT_SMP_AUTHREQ_MITM : 0));

	bool setup = SendInit(&central, BT_CONN_ROLE_CENTRAL, Config.CentralIo,
		auth, 0, centralAddr, 0, peripheralAddr) &&
		SendInit(&peripheral, BT_CONN_ROLE_PERIPHERAL, Config.PeripheralIo,
			auth, 0, peripheralAddr, 0, centralAddr);

	VirtualOobChannel oob;
	if (setup && Config.Oob)
	{
		ScOobData centralData = {};
		ScOobData peripheralData = {};
		setup = RequestOob(&central, &centralData) &&
			RequestOob(&peripheral, &peripheralData) &&
			oob.Publish(0, centralData) && oob.Publish(1, peripheralData);
		ScOobData delivered = {};
		if (setup && oob.Deliver(1, &delivered, Config.CentralReceives))
		{
			setup = SetPeerOob(&central, delivered);
		}
		else if (Config.CentralReceives != OobFault::Drop)
		{
			setup = false;
		}
		if (setup && oob.Deliver(0, &delivered, Config.PeripheralReceives))
		{
			setup = SetPeerOob(&peripheral, delivered);
		}
		else if (Config.PeripheralReceives != OobFault::Drop)
		{
			setup = false;
		}
	}

	WireMessage start = {};
	start.Type = Config.PeripheralStarts ? WIRE_CMD_REQUEST_SECURITY : WIRE_CMD_START_PAIRING;
	if (setup)
	{
		setup = SendWire(Config.PeripheralStarts ? peripheral.CommandFd : central.CommandFd,
			start);
	}

	bool encryptionPending = false;
	uint8_t centralLtk[16] = {};
	bool numericRepliesSent = false;
	bool passkeyReplySent = false;
	int idlePolls = 0;

	for (int step = 0; setup && step < MAX_EVENT_STEPS; step++)
	{
		result.Steps = step + 1;
		if (central.Complete && peripheral.Complete)
		{
			break;
		}
		if (central.FailureReason != 0 || peripheral.FailureReason != 0)
		{
			if (++idlePolls > 2)
			{
				break;
			}
		}

		pollfd fds[2] = {
			{ central.EventFd, POLLIN, 0 },
			{ peripheral.EventFd, POLLIN, 0 },
		};
		int ready = poll(fds, 2, POLL_TIMEOUT_MS);
		if (ready <= 0)
		{
			setup = false;
			break;
		}

		for (int side = 0; side < 2; side++)
		{
			if ((fds[side].revents & POLLIN) == 0)
			{
				continue;
			}
			ChildPeer *pSource = side == 0 ? &central : &peripheral;
			ChildPeer *pDestination = side == 0 ? &peripheral : &central;
			WireMessage event = {};
			if (!ReceiveWire(pSource->EventFd, &event))
			{
				setup = false;
				break;
			}

			switch (event.Type)
			{
				case WIRE_EVT_TX_SMP:
					if (event.Length >= 2 && event.Data[0] == BT_SMP_CODE_PAIRING_FAILED)
					{
						pSource->FailureReason = event.Data[1];
					}
					setup = ForwardSmp(pDestination, event);
					break;

				case WIRE_EVT_ENABLE_ENCRYPTION:
				{
					if (event.Length != 26)
					{
						setup = false;
						break;
					}
					memcpy(centralLtk, &event.Data[10], 16);
					WireMessage ltk = {};
					ltk.Type = WIRE_CMD_LTK_REQUEST;
					ltk.Length = 10;
					memcpy(ltk.Data, event.Data, 10);
					setup = SendWire(peripheral.CommandFd, ltk);
					encryptionPending = true;
					break;
				}

				case WIRE_EVT_LTK_REPLY:
				{
					if (!encryptionPending || event.Length != 16 ||
						memcmp(event.Data, centralLtk, 16) != 0)
					{
						setup = false;
						break;
					}
					WireMessage encrypted = {};
					encrypted.Type = WIRE_CMD_ENCRYPTED;
					encrypted.Code = 0;
					encrypted.Value = 1;
					setup = SendWire(central.CommandFd, encrypted) &&
						SendWire(peripheral.CommandFd, encrypted);
					encryptionPending = false;
					break;
				}

				case WIRE_EVT_LTK_NEG_REPLY:
					setup = false;
					break;

				case WIRE_EVT_NUMERIC:
					pSource->NumericSeen = true;
					pSource->NumericValue = event.Value;
					break;

				case WIRE_EVT_PASSKEY_DISPLAY:
					pSource->PasskeyDisplaySeen = true;
					pSource->DisplayedPasskey = event.Value;
					break;

				case WIRE_EVT_PASSKEY_REQUEST:
					pSource->PasskeyRequestSeen = true;
					break;

				case WIRE_EVT_PAIRING_COMPLETE:
					HandlePairingComplete(pSource, event);
					break;

				case WIRE_EVT_ERROR:
					setup = false;
					break;

				default:
					setup = false;
					break;
			}
		}

		if (setup && central.NumericSeen && peripheral.NumericSeen &&
			!numericRepliesSent)
		{
			bool match = central.NumericValue == peripheral.NumericValue;
			setup = SendNumericReply(&central, match) &&
				SendNumericReply(&peripheral, match);
			numericRepliesSent = true;
		}

		if (setup && !passkeyReplySent)
		{
			if (central.PasskeyRequestSeen && peripheral.PasskeyRequestSeen)
			{
				// KeyboardOnly/KeyboardOnly: the user supplies one external
				// passkey and enters the same value on both devices.
				setup = SendPasskeyReply(&central, 123456U) &&
					SendPasskeyReply(&peripheral, 123456U);
				passkeyReplySent = true;
			}
			else if (central.PasskeyDisplaySeen && peripheral.PasskeyRequestSeen)
			{
				setup = SendPasskeyReply(&peripheral, central.DisplayedPasskey);
				passkeyReplySent = true;
			}
			else if (peripheral.PasskeyDisplaySeen && central.PasskeyRequestSeen)
			{
				setup = SendPasskeyReply(&central, peripheral.DisplayedPasskey);
				passkeyReplySent = true;
			}
		}
	}

	result.TransportOk = setup;
	result.Complete = central.Complete && peripheral.Complete;
	result.Success = result.Complete && central.Success && peripheral.Success;
	result.Authenticated = result.Success && central.Authenticated && peripheral.Authenticated;
	result.Sc = result.Success && central.SecureConnections && peripheral.SecureConnections;
	result.Numeric = central.NumericSeen && peripheral.NumericSeen;
	result.Passkey = (central.PasskeyDisplaySeen && peripheral.PasskeyRequestSeen) ||
		(peripheral.PasskeyDisplaySeen && central.PasskeyRequestSeen) ||
		(central.PasskeyRequestSeen && peripheral.PasskeyRequestSeen);
	result.Oob = Config.Oob;
	result.FailureReason = central.FailureReason != 0 ?
		central.FailureReason : peripheral.FailureReason;

	StopPeer(&central);
	StopPeer(&peripheral);
	return result;
}

static void TestConnectionMatrix(Context &ctx)
{
	static const Requirement req = {
		"GAP-CONN-MATRIX-BV-01", "Core Vol 3 Part C", "9.3",
		"Central and Peripheral role, address, connection-procedure and security-initiator combinations are enumerated"
	};
	ctx.Begin(req);

	bool seen[4][4][3][2] = {};
	uint8_t payload[7] = { 1, 2, 3, 4, 5, 6, 7 };
	for (size_t i = 0; i < VirtualLeLink::CASE_COUNT; i++)
	{
		LeConnectionCase c = VirtualLeLink::CaseAt(i);
		BT_CHECK(ctx, VirtualLeLink::Valid(c));
		unsigned ca = static_cast<unsigned>(c.CentralAddress);
		unsigned pa = static_cast<unsigned>(c.PeripheralAddress);
		unsigned ct = static_cast<unsigned>(c.Type);
		unsigned si = static_cast<unsigned>(c.SecurityStart);
		BT_CHECK(ctx, !seen[ca][pa][ct][si]);
		seen[ca][pa][ct][si] = true;

		VirtualLeLink link;
		BT_CHECK(ctx, link.Connect(c, 1, 2));
		BT_CHECK(ctx, link.Connected());
		BT_CHECK(ctx, link.CentralHandle() == 1);
		BT_CHECK(ctx, link.PeripheralHandle() == 2);
		BT_CHECK(ctx, link.ForwardFromCentral(payload, sizeof(payload)));
		BT_CHECK(ctx, link.ForwardFromPeripheral(payload, sizeof(payload) - 2));
		BT_CHECK(ctx, link.CentralToPeripheralBytes() == sizeof(payload));
		BT_CHECK(ctx, link.PeripheralToCentralBytes() == sizeof(payload) - 2);
		link.Disconnect();
		BT_CHECK(ctx, !link.Connected());
	}
	ctx.End();
}

static void TestOobChannelIsolation(Context &ctx)
{
	static const Requirement req = {
		"SMP-OOB-CHANNEL-BV-01", "Core Vol 3 Part H", "2.3.5.6.4",
		"SC OOB material moves through a separate single-use channel with drop, corruption and replay controls"
	};
	ctx.Begin(req);

	VirtualOobChannel channel;
	ScOobData a = {};
	ScOobData b = {};
	for (size_t i = 0; i < 16; i++)
	{
		a.Random[i] = static_cast<uint8_t>(i);
		a.Confirm[i] = static_cast<uint8_t>(0x20 + i);
		b.Random[i] = static_cast<uint8_t>(0x40 + i);
		b.Confirm[i] = static_cast<uint8_t>(0x60 + i);
	}
	BT_CHECK(ctx, channel.Publish(0, a));
	BT_CHECK(ctx, channel.Publish(1, b));
	ScOobData out = {};
	BT_CHECK(ctx, channel.Deliver(0, &out, OobFault::None));
	BT_CHECK(ctx, memcmp(&out, &a, sizeof(out)) == 0);
	BT_CHECK(ctx, !channel.Deliver(0, &out, OobFault::None));
	BT_CHECK(ctx, channel.Deliver(0, &out, OobFault::Replay));
	BT_CHECK(ctx, channel.Deliver(1, &out, OobFault::CorruptRandom));
	BT_CHECK(ctx, out.Random[0] == static_cast<uint8_t>(b.Random[0] ^ 0x80));
	channel.Reset();
	BT_CHECK(ctx, !channel.Deliver(0, &out, OobFault::None));
	ctx.End();
}

static void TestAssociationMatrix(Context &ctx)
{
	static const Requirement req = {
		"SMP-SC-ASSOC-BV-01", "Core Vol 3 Part H", "2.3.5.1 and 2.3.5.6",
		"all IO-capability pairs complete through the selected SC association model, including OOB"
	};
	ctx.Begin(req);

	for (uint8_t centralIo = 0; centralIo <= BT_SMP_IOCAPS_KEYBOARD_DISPLAY; centralIo++)
	{
		for (uint8_t peripheralIo = 0; peripheralIo <= BT_SMP_IOCAPS_KEYBOARD_DISPLAY; peripheralIo++)
		{
			for (int mode = 0; mode < 3; mode++)
			{
				PairingRunConfig cfg = {};
				cfg.CentralIo = centralIo;
				cfg.PeripheralIo = peripheralIo;
				cfg.Mitm = mode != 0;
				cfg.Oob = mode == 2;
				cfg.CentralReceives = OobFault::None;
				cfg.PeripheralReceives = OobFault::None;
				PairingRunResult result = RunPairing(cfg);
				uint8_t model = ExpectedModel(centralIo, peripheralIo, cfg.Mitm, cfg.Oob);

				BT_CHECK(ctx, result.TransportOk);
				BT_CHECK(ctx, result.Complete);
				BT_CHECK(ctx, result.Success);
				BT_CHECK(ctx, result.Sc);
				BT_CHECK(ctx, result.Authenticated ==
					(model != BT_SMP_MODEL_JUST_WORKS));
				BT_CHECK(ctx, result.Numeric ==
					(model == BT_SMP_MODEL_NUMERIC_COMPARISON));
				BT_CHECK(ctx, result.Passkey ==
					(model == BT_SMP_MODEL_PASSKEY_ENTRY));
			}
		}
	}
	ctx.End();
}

static void TestPeripheralSecurityRequest(Context &ctx)
{
	static const Requirement req = {
		"SMP-SEC-REQ-BV-01", "Core Vol 3 Part H", "2.4.6 and 3.6.7",
		"a Peripheral Security Request causes the Central to initiate each supported SC association model"
	};
	ctx.Begin(req);

	const PairingRunConfig cases[] = {
		{ BT_SMP_IOCAPS_NO_INPUT_NO_OUTPUT, BT_SMP_IOCAPS_NO_INPUT_NO_OUTPUT,
			false, false, true, OobFault::None, OobFault::None },
		{ BT_SMP_IOCAPS_DISPLAY_YESNO, BT_SMP_IOCAPS_DISPLAY_YESNO,
			true, false, true, OobFault::None, OobFault::None },
		{ BT_SMP_IOCAPS_DISPLAY_ONLY, BT_SMP_IOCAPS_KEYBOARD_ONLY,
			true, false, true, OobFault::None, OobFault::None },
		{ BT_SMP_IOCAPS_NO_INPUT_NO_OUTPUT, BT_SMP_IOCAPS_NO_INPUT_NO_OUTPUT,
			true, true, true, OobFault::None, OobFault::None },
	};

	for (const PairingRunConfig &cfg : cases)
	{
		PairingRunResult result = RunPairing(cfg);
		BT_CHECK(ctx, result.TransportOk);
		BT_CHECK(ctx, result.Complete);
		BT_CHECK(ctx, result.Success);
		BT_CHECK(ctx, result.Sc);
	}
	ctx.End();
}

static void TestTxFailureQueuesAndRetries(Context &ctx)
{
	static const Requirement req = {
		"SMP-ACL-FLOW-BI-02", "Core Vol 3 Part H / Vol 4 Part E", "3.4 and 5.4.2",
		"an SMP PDU refused by the ACL transport is retained and sent after credits return"
	};
	ctx.Begin(req);

	ChildPeer peer = {};
	BT_CHECK(ctx, SpawnPeer(&peer));
	const uint8_t local[6] = { 1, 2, 3, 4, 5, 0xC0 };
	const uint8_t remote[6] = { 6, 7, 8, 9, 10, 0xD0 };
	BT_CHECK(ctx, SendInit(&peer, BT_CONN_ROLE_CENTRAL,
		BT_SMP_IOCAPS_NO_INPUT_NO_OUTPUT,
		BT_SMP_AUTHREQ_BONDING_FLAG_BONDING | BT_SMP_AUTHREQ_SC,
		BTADDR_TYPE_RAND, local, BTADDR_TYPE_RAND, remote));

	WireMessage block = {};
	block.Type = WIRE_CMD_SET_TX_BLOCK;
	block.Value = 1;
	BT_CHECK(ctx, SendWire(peer.CommandFd, block));

	WireMessage start = {};
	start.Type = WIRE_CMD_START_PAIRING;
	BT_CHECK(ctx, SendWire(peer.CommandFd, start));

	pollfd pfd = { peer.EventFd, POLLIN, 0 };
	BT_CHECK(ctx, poll(&pfd, 1, 50) == 0);

	block.Value = 0;
	BT_CHECK(ctx, SendWire(peer.CommandFd, block));
	WireMessage pump = {};
	pump.Type = WIRE_CMD_PUMP_TX;
	BT_CHECK(ctx, SendWire(peer.CommandFd, pump));

	WireMessage event = {};
	BT_CHECK(ctx, ReceiveWire(peer.EventFd, &event));
	BT_CHECK(ctx, event.Type == WIRE_EVT_TX_SMP);
	BT_CHECK(ctx, event.Length > 0);
	BT_CHECK(ctx, event.Data[0] == BT_SMP_CODE_PAIRING_REQ);

	StopPeer(&peer);
	ctx.End();
}

static void TestOobDirectionsAndFailures(Context &ctx)
{
	static const Requirement req = {
		"SMP-SC-OOB-BV-BI-01", "Core Vol 3 Part H", "2.3.3 and 2.3.5.6.4",
		"one-direction and bidirectional SC OOB complete, while corrupted authentication data fails"
	};
	ctx.Begin(req);

	PairingRunConfig centralToPeripheral = {
		BT_SMP_IOCAPS_NO_INPUT_NO_OUTPUT,
		BT_SMP_IOCAPS_NO_INPUT_NO_OUTPUT,
		true, true, false, OobFault::Drop, OobFault::None
	};
	PairingRunResult centralToPeripheralResult = RunPairing(centralToPeripheral);
	BT_CHECK(ctx, centralToPeripheralResult.TransportOk);
	BT_CHECK(ctx, centralToPeripheralResult.Complete);
	BT_CHECK(ctx, centralToPeripheralResult.Success);
	BT_CHECK(ctx, centralToPeripheralResult.Authenticated);
	BT_CHECK(ctx, centralToPeripheralResult.Sc);

	PairingRunConfig peripheralToCentral = centralToPeripheral;
	peripheralToCentral.CentralReceives = OobFault::None;
	peripheralToCentral.PeripheralReceives = OobFault::Drop;
	PairingRunResult peripheralToCentralResult = RunPairing(peripheralToCentral);
	BT_CHECK(ctx, peripheralToCentralResult.TransportOk);
	BT_CHECK(ctx, peripheralToCentralResult.Complete);
	BT_CHECK(ctx, peripheralToCentralResult.Success);
	BT_CHECK(ctx, peripheralToCentralResult.Authenticated);
	BT_CHECK(ctx, peripheralToCentralResult.Sc);

	PairingRunConfig corrupt = centralToPeripheral;
	corrupt.CentralReceives = OobFault::CorruptConfirm;
	PairingRunResult corruptResult = RunPairing(corrupt);
	BT_CHECK(ctx, !corruptResult.Success);
	BT_CHECK(ctx, corruptResult.FailureReason != 0);
	ctx.End();
}

} // namespace

extern "C" {

void CryptoSecureWipe(void *pData, size_t Len)
{
	volatile uint8_t *p = static_cast<volatile uint8_t *>(pData);
	while (Len-- != 0)
	{
		*p++ = 0;
	}
}

BtDevice_t *BtPeerFindByHdl(uint16_t ConnHdl)
{
	return s_pWorker != nullptr && s_pWorker->Peer.Conn.Hdl == ConnHdl ?
		&s_pWorker->Peer : nullptr;
}

uint8_t BtPeerRole(uint16_t ConnHdl)
{
	BtDevice_t *pPeer = BtPeerFindByHdl(ConnHdl);
	return pPeer != nullptr ? pPeer->Conn.Role : BT_CONN_ROLE_UNKNOWN;
}

uint16_t BtPeerActiveHdl(void)
{
	return s_pWorker != nullptr ? s_pWorker->Peer.Conn.Hdl : BT_CONN_HDL_INVALID;
}

bool BtGapConnSecGet(uint16_t ConnHdl, BtConnSec_t *pSec)
{
	BtDevice_t *pPeer = BtPeerFindByHdl(ConnHdl);
	if (pPeer == nullptr || pSec == nullptr)
	{
		return false;
	}
	*pSec = pPeer->Conn.Sec;
	return true;
}

void BtGapConnSecSet(uint16_t ConnHdl, const BtConnSec_t *pSec)
{
	BtDevice_t *pPeer = BtPeerFindByHdl(ConnHdl);
	if (pPeer != nullptr && pSec != nullptr)
	{
		pPeer->Conn.Sec = *pSec;
		pPeer->bSecure = pSec->Level != BT_GAP_SEC_LEVEL_NONE;
	}
}

uint32_t BtHciSendAcl(BtHciDevice_t * const, BtHciACLDataPacket_t * const pAcl)
{
	if (s_pWorker != nullptr && s_pWorker->TxBlocked)
	{
		return 0;
	}
	if (pAcl == nullptr || pAcl->Hdr.Len < sizeof(BtL2CapHdr_t))
	{
		return 0;
	}
	BtL2CapPdu_t *pL2 = reinterpret_cast<BtL2CapPdu_t *>(pAcl->Data);
	if (pL2->Hdr.Cid != BT_L2CAP_CID_SEC_MNGR || pL2->Hdr.Len > WIRE_DATA_SIZE)
	{
		return 0;
	}
	const uint8_t *pSmp = reinterpret_cast<const uint8_t *>(&pL2->Smp);
	if (!WorkerEmit(WIRE_EVT_TX_SMP, pSmp[0], 0, pSmp, pL2->Hdr.Len))
	{
		return 0;
	}
	return static_cast<uint32_t>(sizeof(BtHciACLDataPacketHdr_t) + pAcl->Hdr.Len);
}

void BtSmpHciEnableEncryption(BtHciDevice_t * const, uint16_t,
	uint64_t Rand, uint16_t Ediv, const uint8_t Ltk[16])
{
	uint8_t data[26] = {};
	StoreLe64(data, Rand);
	data[8] = static_cast<uint8_t>(Ediv);
	data[9] = static_cast<uint8_t>(Ediv >> 8);
	memcpy(&data[10], Ltk, 16);
	WorkerEmit(WIRE_EVT_ENABLE_ENCRYPTION, 0, 0, data, sizeof(data));
}

void BtSmpHciLtkReply(BtHciDevice_t * const, uint16_t, const uint8_t Ltk[16])
{
	WorkerEmit(WIRE_EVT_LTK_REPLY, 0, 0, Ltk, 16);
}

void BtSmpHciLtkNegReply(BtHciDevice_t * const, uint16_t)
{
	WorkerEmit(WIRE_EVT_LTK_NEG_REPLY, 0, 0, nullptr, 0);
}

void BtSmpNumericComparison(uint16_t, uint32_t Value)
{
	WorkerEmit(WIRE_EVT_NUMERIC, 0, Value, nullptr, 0);
}

void BtSmpPasskeyDisplay(uint16_t, uint32_t Passkey)
{
	WorkerEmit(WIRE_EVT_PASSKEY_DISPLAY, 0, Passkey, nullptr, 0);
}

void BtSmpPasskeyRequest(uint16_t)
{
	WorkerEmit(WIRE_EVT_PASSKEY_REQUEST, 0, 0, nullptr, 0);
}

void BtSmpPairingComplete(uint16_t, bool Success, const BtSmpKeys_t *pKeys)
{
	uint8_t data[3] = {};
	if (pKeys != nullptr)
	{
		data[0] = pKeys->bAuthenticated ? 1 : 0;
		data[1] = pKeys->bSc ? 1 : 0;
		data[2] = pKeys->bValid ? 1 : 0;
	}
	WorkerEmit(WIRE_EVT_PAIRING_COMPLETE, 0, Success ? 1U : 0U,
		data, sizeof(data));
}

void BtSmpLocalAddrGet(uint8_t *pType, uint8_t pAddr[6])
{
	if (s_pWorker != nullptr)
	{
		*pType = s_pWorker->LocalAddrType;
		memcpy(pAddr, s_pWorker->LocalAddr, 6);
	}
	else
	{
		*pType = 0;
		memset(pAddr, 0, 6);
	}
}

bool BtSmpLocalIrkGet(uint8_t Irk[16])
{
	if (s_pWorker == nullptr || Irk == nullptr)
	{
		return false;
	}
	memcpy(Irk, s_pWorker->LocalIrk, 16);
	return true;
}

void BtSmpBondLoad(void) {}
void BtSmpBondSave(int, const void *, size_t) {}
void BtSmpBondErase(void) {}
void BtSmpLocalIdSave(void) {}
size_t BtSmpLocalIdRecordSize(void) { return 0; }
size_t BtSmpLocalIdSerialize(void *, size_t) { return 0; }
bool BtSmpLocalIdRestore(const void *, size_t) { return false; }

bool BtSmpBondKeysLookup(uint16_t, uint64_t, uint16_t, BtSmpKeys_t *)
{
	return false;
}

bool BtSmpBonded(uint16_t)
{
	return false;
}

bool BtGattTxPendUntracked(uint16_t, uint16_t) { return true; }
void BtGattTxPendRelease(uint16_t) {}

void BtSmpBondCccdSave(uint16_t, uint16_t, uint16_t) {}
uint8_t BtSmpBondCccdGet(uint16_t, uint16_t *, uint16_t *, uint8_t)
{
	return 0;
}

} // extern "C"

int main()
{
	Context ctx("Bluetooth dual-host connection and SMP compliance");
	TestConnectionMatrix(ctx);
	TestOobChannelIsolation(ctx);
	TestAssociationMatrix(ctx);
	TestPeripheralSecurityRequest(ctx);
	TestTxFailureQueuesAndRetries(ctx);
	TestOobDirectionsAndFailures(ctx);
	return ctx.Finish(false);
}
