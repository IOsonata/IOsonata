// Dual-host ATT long-write compliance.
//
// The client and server run in separate processes. ATT SDUs cross a pair of
// pipes inside bounded L2CAP Basic Mode frames, so the production ATT server
// is exercised through the same request/response byte stream a controller
// transport carries rather than through direct test calls.

#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include <signal.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include "bluetooth/bt_att.h"
#include "bluetooth/bt_gap.h"
#include "bluetooth/bt_gatt.h"
#include "bluetooth/bt_l2cap.h"
#include "bluetooth/bt_peer.h"

#include "bt_compliance.h"

using btcompliance::Context;
using btcompliance::Requirement;

namespace {

constexpr uint16_t kCentralConn = 0x0001;
constexpr uint16_t kObserverConn = 0x0002;
constexpr uint16_t kControlCid = 0xFFFF;
constexpr uint16_t kOpenValueHandle = 0x0001;
constexpr uint16_t kSecureValueHandle = 0x0002;
constexpr uint16_t kOpenUuid = 0xFFF1;
constexpr uint16_t kSecureUuid = 0xFFF2;
constexpr uint16_t kMaxValueLen = 512;
constexpr uint16_t kPrepareQueueSize = 1024;
constexpr uint16_t kWirePayloadMax = BT_ATT_MTU_MAX + 16;

#pragma pack(push, 1)
struct WireHeader {
	uint16_t ConnHdl;
	uint16_t Len;
	uint16_t Cid;
};
#pragma pack(pop)

struct Pdu {
	uint16_t Len;
	uint8_t Data[kWirePayloadMax];
};

struct ServerPeer {
	BtDevice_t Dev;
	uint8_t Queue[kPrepareQueueSize];
};

static ServerPeer s_ServerPeers[2];
static BtChar_t s_OpenChar;
static BtChar_t s_SecureChar;
static uint8_t s_OpenValue[kMaxValueLen];
static uint8_t s_SecureValue[kMaxValueLen];
static uint8_t s_CapturedAcl[BT_HCI_BUFFER_MAX_SIZE];
static uint32_t s_CapturedAclLen;
static bool s_RefuseAcl;

static uint16_t GetLe16(const uint8_t *p)
{
	return (uint16_t)(p[0] | ((uint16_t)p[1] << 8));
}

static void PutLe16(uint8_t *p, uint16_t Value)
{
	p[0] = (uint8_t)(Value & 0xFF);
	p[1] = (uint8_t)(Value >> 8);
}

static uint32_t CaptureAcl(void *pData, uint32_t Len)
{
	if (pData == nullptr || Len > sizeof(s_CapturedAcl))
	{
		return 0;
	}
	std::memcpy(s_CapturedAcl, pData, Len);
	s_CapturedAclLen = Len;
	return Len;
}

static bool WriteAll(int Fd, const void *pData, size_t Len)
{
	const uint8_t *p = (const uint8_t *)pData;
	while (Len > 0)
	{
		ssize_t n = write(Fd, p, Len);
		if (n < 0)
		{
			if (errno == EINTR)
			{
				continue;
			}
			return false;
		}
		if (n == 0)
		{
			return false;
		}
		p += (size_t)n;
		Len -= (size_t)n;
	}
	return true;
}

static bool ReadAll(int Fd, void *pData, size_t Len)
{
	uint8_t *p = (uint8_t *)pData;
	while (Len > 0)
	{
		ssize_t n = read(Fd, p, Len);
		if (n < 0)
		{
			if (errno == EINTR)
			{
				continue;
			}
			return false;
		}
		if (n == 0)
		{
			return false;
		}
		p += (size_t)n;
		Len -= (size_t)n;
	}
	return true;
}

static BtDevice_t *FindServerPeer(uint16_t ConnHdl)
{
	for (size_t i = 0; i < sizeof(s_ServerPeers) / sizeof(s_ServerPeers[0]); ++i)
	{
		if (s_ServerPeers[i].Dev.Conn.Hdl == ConnHdl)
		{
			return &s_ServerPeers[i].Dev;
		}
	}
	return nullptr;
}

static BtAttDBEntry_t *AddCharValue(BtChar_t *pChar, uint8_t *pValue,
									 uint16_t MaxLen, uint16_t Uuid,
									 uint32_t Permission)
{
	std::memset(pChar, 0, sizeof(*pChar));
	pChar->Uuid = Uuid;
	pChar->MaxDataLen = MaxLen;
	pChar->Property = BT_GATT_CHAR_PROP_READ | BT_GATT_CHAR_PROP_WRITE;
	pChar->pValue = pValue;
	pChar->ValueLen = 0;

	BtUuid16_t uuid = { 0, BT_UUID_TYPE_16, Uuid };
	BtAttDBEntry_t *pEntry =
		BtAttDBAddEntry(&uuid, (int)(sizeof(BtAttCharValue_t) + MaxLen));
	if (pEntry == nullptr)
	{
		return nullptr;
	}

	((BtAttCharValue_t *)pEntry->Data)->pChar = pChar;
	BtAttDBEntrySetPermission(pEntry, Permission);
	return pEntry;
}

static bool ServerInit()
{
	std::memset(s_ServerPeers, 0, sizeof(s_ServerPeers));
	std::memset(s_OpenValue, 0, sizeof(s_OpenValue));
	std::memset(s_SecureValue, 0, sizeof(s_SecureValue));

	for (size_t i = 0; i < sizeof(s_ServerPeers) / sizeof(s_ServerPeers[0]); ++i)
	{
		ServerPeer &peer = s_ServerPeers[i];
		peer.Dev.Conn.Hdl = (i == 0) ? kCentralConn : kObserverConn;
		peer.Dev.Conn.MaxMtu = BT_ATT_MTU_MIN;
		peer.Dev.Conn.pLongWrBuff = peer.Queue;
		peer.Dev.Conn.LongWrBuffSize = sizeof(peer.Queue);
		peer.Dev.Conn.LongWrLen = 0;
		peer.Dev.Conn.Sec.Level = BT_GAP_SEC_LEVEL_NONE;
		peer.Dev.Conn.Sec.KeySize = 0;
		peer.Dev.Conn.Sec.Flags = 0;
	}

	BtAttDBInit(2048);
	BtAttSetMtu(BT_ATT_MTU_MAX);

	BtAttDBEntry_t *pOpen = AddCharValue(
		&s_OpenChar, s_OpenValue, sizeof(s_OpenValue), kOpenUuid,
		BT_ATT_PERMISSION_READ | BT_ATT_PERMISSION_WRITE);
	BtAttDBEntry_t *pSecure = AddCharValue(
		&s_SecureChar, s_SecureValue, sizeof(s_SecureValue), kSecureUuid,
		BT_ATT_PERMISSION_READ | BT_ATT_PERMISSION_WRITE |
		BT_ATT_PERMISSION_READ_ENCRYPT | BT_ATT_PERMISSION_WRITE_ENCRYPT |
		BT_ATT_PERMISSION_READ_AUTHEN | BT_ATT_PERMISSION_WRITE_AUTHEN |
		BT_ATT_PERMISSION_READ_KEY_SIZE | BT_ATT_PERMISSION_WRITE_KEY_SIZE);

	return pOpen != nullptr && pOpen->Hdl == kOpenValueHandle &&
		   pSecure != nullptr && pSecure->Hdl == kSecureValueHandle;
}

static bool ServerControl(uint16_t ConnHdl, const uint8_t *pReq, uint16_t ReqLen,
						  Pdu *pRsp)
{
	pRsp->Len = 1;
	pRsp->Data[0] = 1;
	if (ReqLen < 1)
	{
		return true;
	}

	if (pReq[0] == 1)
	{
		BtDevice_t *pPeer = FindServerPeer(ConnHdl);
		if (pPeer == nullptr || ReqLen != 4)
		{
			return true;
		}
		pPeer->Conn.Sec.Level = pReq[1];
		pPeer->Conn.Sec.KeySize = pReq[2];
		pPeer->Conn.Sec.Flags = pReq[3];
		pRsp->Data[0] = 0;
		return true;
	}

	if (pReq[0] == 2)
	{
		pRsp->Data[0] = 0;
		return false;
	}

	return true;
}

static int ServerMain(int RxFd, int TxFd)
{
	if (!ServerInit())
	{
		return 2;
	}

	bool running = true;
	while (running)
	{
		WireHeader hdr;
		if (!ReadAll(RxFd, &hdr, sizeof(hdr)))
		{
			break;
		}
		if (hdr.Len > kWirePayloadMax)
		{
			return 3;
		}

		Pdu req = {};
		req.Len = hdr.Len;
		if (req.Len > 0 && !ReadAll(RxFd, req.Data, req.Len))
		{
			return 4;
		}

		Pdu rsp = {};
		uint16_t rspCid = hdr.Cid;
		if (hdr.Cid == BT_L2CAP_CID_ATT)
		{
			if (req.Len == 0)
			{
				rsp.Len = 0;
			}
			else
			{
				rsp.Len = (uint16_t)BtAttProcessReq(
					hdr.ConnHdl, (BtAttReqRsp_t *)req.Data, req.Len,
					(BtAttReqRsp_t *)rsp.Data);
			}
		}
		else if (hdr.Cid == kControlCid)
		{
			running = ServerControl(hdr.ConnHdl, req.Data, req.Len, &rsp);
		}
		else
		{
			rspCid = kControlCid;
			rsp.Len = 1;
			rsp.Data[0] = BT_L2CAP_CMD_REJECT_REASON_INVALID_CID;
		}

		WireHeader out = { hdr.ConnHdl, rsp.Len, rspCid };
		if (!WriteAll(TxFd, &out, sizeof(out)) ||
			(rsp.Len > 0 && !WriteAll(TxFd, rsp.Data, rsp.Len)))
		{
			return 5;
		}
	}

	return 0;
}

class DualHostLink {
public:
	DualHostLink() : vTx(-1), vRx(-1), vPid(-1) {}

	bool Start()
	{
		int c2s[2];
		int s2c[2];
		if (pipe(c2s) != 0 || pipe(s2c) != 0)
		{
			return false;
		}

		pid_t pid = fork();
		if (pid < 0)
		{
			close(c2s[0]);
			close(c2s[1]);
			close(s2c[0]);
			close(s2c[1]);
			return false;
		}

		if (pid == 0)
		{
			close(c2s[1]);
			close(s2c[0]);
			int rc = ServerMain(c2s[0], s2c[1]);
			close(c2s[0]);
			close(s2c[1]);
			_exit(rc);
		}

		close(c2s[0]);
		close(s2c[1]);
		vTx = c2s[1];
		vRx = s2c[0];
		vPid = pid;
		return true;
	}

	bool Exchange(uint16_t ConnHdl, uint16_t Cid, const uint8_t *pReq,
				  uint16_t ReqLen, Pdu *pRsp)
	{
		if (vTx < 0 || vRx < 0 || pRsp == nullptr || ReqLen > kWirePayloadMax)
		{
			return false;
		}

		WireHeader hdr = { ConnHdl, ReqLen, Cid };
		if (!WriteAll(vTx, &hdr, sizeof(hdr)) ||
			(ReqLen > 0 && !WriteAll(vTx, pReq, ReqLen)))
		{
			return false;
		}

		WireHeader rspHdr;
		if (!ReadAll(vRx, &rspHdr, sizeof(rspHdr)) ||
			rspHdr.ConnHdl != ConnHdl || rspHdr.Cid != Cid ||
			rspHdr.Len > kWirePayloadMax)
		{
			return false;
		}

		pRsp->Len = rspHdr.Len;
		return pRsp->Len == 0 || ReadAll(vRx, pRsp->Data, pRsp->Len);
	}

	bool SetSecurity(uint16_t ConnHdl, uint8_t Level, uint8_t KeySize,
					 uint8_t Flags)
	{
		uint8_t req[4] = { 1, Level, KeySize, Flags };
		Pdu rsp = {};
		return Exchange(ConnHdl, kControlCid, req, sizeof(req), &rsp) &&
			   rsp.Len == 1 && rsp.Data[0] == 0;
	}

	bool Stop()
	{
		bool ok = true;
		if (vTx >= 0 && vRx >= 0)
		{
			uint8_t req = 2;
			Pdu rsp = {};
			ok = Exchange(kCentralConn, kControlCid, &req, 1, &rsp) &&
				 rsp.Len == 1 && rsp.Data[0] == 0;
		}

		if (vTx >= 0)
		{
			close(vTx);
			vTx = -1;
		}
		if (vRx >= 0)
		{
			close(vRx);
			vRx = -1;
		}

		if (vPid > 0)
		{
			int status = 0;
			while (waitpid(vPid, &status, 0) < 0)
			{
				if (errno != EINTR)
				{
					ok = false;
					break;
				}
			}
			if (!WIFEXITED(status) || WEXITSTATUS(status) != 0)
			{
				ok = false;
			}
			vPid = -1;
		}
		return ok;
	}

	~DualHostLink()
	{
		if (vTx >= 0 || vRx >= 0 || vPid > 0)
		{
			Stop();
		}
	}

private:
	int vTx;
	int vRx;
	pid_t vPid;
};

static bool IsError(const Pdu &Rsp, uint8_t ReqOp, uint8_t Error)
{
	return Rsp.Len == 5 &&
		   Rsp.Data[0] == BT_ATT_OPCODE_ATT_ERROR_RSP &&
		   Rsp.Data[1] == ReqOp &&
		   Rsp.Data[4] == Error;
}

static bool ExchangeMtu(DualHostLink &Link, uint16_t ConnHdl, uint16_t Offered,
						uint16_t *pNegotiated)
{
	uint8_t req[3] = { BT_ATT_OPCODE_ATT_EXCHANGE_MTU_REQ, 0, 0 };
	PutLe16(req + 1, Offered);
	Pdu rsp = {};
	if (!Link.Exchange(ConnHdl, BT_L2CAP_CID_ATT, req, sizeof(req), &rsp) ||
		rsp.Len != 3 || rsp.Data[0] != BT_ATT_OPCODE_ATT_EXCHANGE_MTU_RSP)
	{
		return false;
	}

	uint16_t serverMtu = GetLe16(rsp.Data + 1);
	uint16_t negotiated = Offered < serverMtu ? Offered : serverMtu;
	if (negotiated < BT_ATT_MTU_MIN)
	{
		negotiated = BT_ATT_MTU_MIN;
	}
	if (pNegotiated != nullptr)
	{
		*pNegotiated = negotiated;
	}
	return true;
}

static bool PrepareWrite(DualHostLink &Link, uint16_t ConnHdl, uint16_t Hdl,
						 uint16_t Offset, const uint8_t *pData, uint16_t Len,
						 Pdu *pRsp)
{
	uint8_t req[kWirePayloadMax] = {};
	req[0] = BT_ATT_OPCODE_ATT_PREPARE_WRITE_REQ;
	PutLe16(req + 1, Hdl);
	PutLe16(req + 3, Offset);
	if (Len > 0)
	{
		std::memcpy(req + 5, pData, Len);
	}

	Pdu localRsp = {};
	Pdu *rsp = pRsp != nullptr ? pRsp : &localRsp;
	if (!Link.Exchange(ConnHdl, BT_L2CAP_CID_ATT, req, (uint16_t)(5 + Len), rsp))
	{
		return false;
	}

	if (rsp->Len != 5 + Len ||
		rsp->Data[0] != BT_ATT_OPCODE_ATT_PREPARE_WRITE_RSP)
	{
		return false;
	}
	return std::memcmp(rsp->Data + 1, req + 1, 4 + Len) == 0;
}

static bool ExecuteWrite(DualHostLink &Link, uint16_t ConnHdl, bool Commit,
						 Pdu *pRsp)
{
	uint8_t req[2] = {
		BT_ATT_OPCODE_ATT_EXECUTE_WRITE_REQ,
		(uint8_t)(Commit ? 1 : 0)
	};
	Pdu localRsp = {};
	Pdu *rsp = pRsp != nullptr ? pRsp : &localRsp;
	if (!Link.Exchange(ConnHdl, BT_L2CAP_CID_ATT, req, sizeof(req), rsp))
	{
		return false;
	}
	return rsp->Len == 1 &&
		   rsp->Data[0] == BT_ATT_OPCODE_ATT_EXECUTE_WRITE_RSP;
}

static bool PrepareLongWrite(DualHostLink &Link, uint16_t ConnHdl, uint16_t Hdl,
							 const uint8_t *pData, uint16_t Len, uint16_t Mtu)
{
	uint16_t fragment = (uint16_t)(Mtu - 5);
	uint16_t offset = 0;
	while (offset < Len)
	{
		uint16_t n = (uint16_t)(Len - offset);
		if (n > fragment)
		{
			n = fragment;
		}
		if (!PrepareWrite(Link, ConnHdl, Hdl, offset, pData + offset, n, nullptr))
		{
			return false;
		}
		offset = (uint16_t)(offset + n);
	}
	return true;
}

static bool ReadLong(DualHostLink &Link, uint16_t ConnHdl, uint16_t Hdl,
					 uint16_t Mtu, uint8_t *pOut, uint16_t OutSize,
					 uint16_t *pLen)
{
	uint16_t offset = 0;
	bool first = true;
	for (;;)
	{
		uint8_t req[5] = {};
		uint16_t reqLen;
		if (first)
		{
			req[0] = BT_ATT_OPCODE_ATT_READ_REQ;
			PutLe16(req + 1, Hdl);
			reqLen = 3;
		}
		else
		{
			req[0] = BT_ATT_OPCODE_ATT_READ_BLOB_REQ;
			PutLe16(req + 1, Hdl);
			PutLe16(req + 3, offset);
			reqLen = 5;
		}

		Pdu rsp = {};
		if (!Link.Exchange(ConnHdl, BT_L2CAP_CID_ATT, req, reqLen, &rsp))
		{
			return false;
		}

		uint8_t expected = first ? BT_ATT_OPCODE_ATT_READ_RSP :
								  BT_ATT_OPCODE_ATT_READ_BLOB_RSP;
		if (rsp.Len < 1 || rsp.Data[0] != expected)
		{
			return false;
		}

		uint16_t n = (uint16_t)(rsp.Len - 1);
		if ((uint32_t)offset + n > OutSize)
		{
			return false;
		}
		if (n > 0)
		{
			std::memcpy(pOut + offset, rsp.Data + 1, n);
		}
		offset = (uint16_t)(offset + n);
		first = false;

		if (n < Mtu - 1)
		{
			break;
		}
	}

	if (pLen != nullptr)
	{
		*pLen = offset;
	}
	return true;
}

static void FillPattern(uint8_t *pData, uint16_t Len, uint8_t Seed)
{
	for (uint16_t i = 0; i < Len; ++i)
	{
		pData[i] = (uint8_t)(Seed + i * 29U + (i >> 2));
	}
}

static bool ReadEquals(DualHostLink &Link, uint16_t ConnHdl, uint16_t Hdl,
					   uint16_t Mtu, const uint8_t *pExpected,
					   uint16_t ExpectedLen)
{
	uint8_t value[kMaxValueLen] = {};
	uint16_t len = 0;
	return ReadLong(Link, ConnHdl, Hdl, Mtu, value, sizeof(value), &len) &&
		   len == ExpectedLen &&
		   std::memcmp(value, pExpected, ExpectedLen) == 0;
}

static bool RunLongWriteMtuCase(uint16_t OfferedMtu)
{
	DualHostLink link;
	if (!link.Start())
	{
		return false;
	}

	uint16_t mtu1 = 0;
	uint16_t mtu2 = 0;
	bool ok = ExchangeMtu(link, kCentralConn, OfferedMtu, &mtu1) &&
			  ExchangeMtu(link, kObserverConn, OfferedMtu, &mtu2) &&
			  mtu1 == OfferedMtu && mtu2 == OfferedMtu;

	uint16_t len = (uint16_t)(2U * OfferedMtu + 7U);
	if (len >= kMaxValueLen)
	{
		len = kMaxValueLen - 1;
	}

	uint8_t valueA[kMaxValueLen] = {};
	uint8_t valueB[kMaxValueLen] = {};
	FillPattern(valueA, len, 0x11);
	FillPattern(valueB, len, 0x83);

	uint8_t empty[1] = {};
	ok = ok && ReadEquals(link, kObserverConn, kOpenValueHandle, mtu2, empty, 0);
	ok = ok && PrepareLongWrite(link, kCentralConn, kOpenValueHandle,
							   valueA, len, mtu1);
	ok = ok && ReadEquals(link, kObserverConn, kOpenValueHandle, mtu2, empty, 0);
	ok = ok && ExecuteWrite(link, kCentralConn, true, nullptr);
	ok = ok && ReadEquals(link, kObserverConn, kOpenValueHandle, mtu2,
						 valueA, len);

	ok = ok && PrepareLongWrite(link, kCentralConn, kOpenValueHandle,
							   valueB, len, mtu1);
	ok = ok && ExecuteWrite(link, kCentralConn, false, nullptr);
	ok = ok && ReadEquals(link, kObserverConn, kOpenValueHandle, mtu2,
						 valueA, len);

	return link.Stop() && ok;
}

static void TestClientRequestBuilders(Context &ctx)
{
	static const Requirement req = {
		"ATT-LONG-WRITE-CLIENT-BV-05", "Core Vol 3 Part F", "3.4.6.1 and 3.4.6.3",
		"production ATT client builders emit exact Prepare Write and Execute Write ACL/L2CAP packets"
	};
	ctx.Begin(req);

	BtHciDevice_t dev = {};
	dev.SendData = CaptureAcl;
	uint8_t value[18];
	FillPattern(value, sizeof(value), 0x39);

	BtAttSetMtu(BT_ATT_MTU_MIN);
	s_RefuseAcl = false;
	s_CapturedAclLen = 0;
	BT_CHECK(ctx, BtAttPrepareWriteRequest(&dev, 0x0123, 0x4567, 0x0089,
									 value, sizeof(value)));
	BtHciACLDataPacket_t *acl = (BtHciACLDataPacket_t *)s_CapturedAcl;
	BtL2CapPdu_t *l2 = (BtL2CapPdu_t *)acl->Data;
	BT_CHECK(ctx, s_CapturedAclLen == sizeof(acl->Hdr) + sizeof(BtL2CapHdr_t) + 5 + sizeof(value));
	BT_CHECK(ctx, acl->Hdr.ConnHdl == 0x0123);
	BT_CHECK(ctx, acl->Hdr.PBFlag == BT_HCI_PBFLAG_START_NONFLUSHABLE);
	BT_CHECK(ctx, l2->Hdr.Cid == BT_L2CAP_CID_ATT);
	BT_CHECK(ctx, l2->Hdr.Len == 5 + sizeof(value));
	BT_CHECK(ctx, l2->Att.OpCode == BT_ATT_OPCODE_ATT_PREPARE_WRITE_REQ);
	BT_CHECK(ctx, l2->Att.PrepareWriteReq.Hdl == 0x4567);
	BT_CHECK(ctx, l2->Att.PrepareWriteReq.Offset == 0x0089);
	BT_CHECK(ctx, std::memcmp(l2->Att.PrepareWriteReq.Data, value, sizeof(value)) == 0);

	s_CapturedAclLen = 0;
	BT_CHECK(ctx, BtAttExecuteWriteRequest(&dev, 0x0123, true));
	acl = (BtHciACLDataPacket_t *)s_CapturedAcl;
	l2 = (BtL2CapPdu_t *)acl->Data;
	BT_CHECK(ctx, l2->Att.OpCode == BT_ATT_OPCODE_ATT_EXECUTE_WRITE_REQ);
	BT_CHECK(ctx, l2->Att.ExecuteWriteReq.Flag == 0x01);
	BT_CHECK(ctx, BtAttExecuteWriteRequest(&dev, 0x0123, false));
	acl = (BtHciACLDataPacket_t *)s_CapturedAcl;
	l2 = (BtL2CapPdu_t *)acl->Data;
	BT_CHECK(ctx, l2->Att.ExecuteWriteReq.Flag == 0x00);

	uint8_t tooLarge[BT_HCI_BUFFER_MAX_SIZE] = {};
	s_CapturedAclLen = 0;
	BT_CHECK(ctx, !BtAttPrepareWriteRequest(&dev, 1, 1, 0,
									  tooLarge, sizeof(tooLarge)));
	BT_CHECK(ctx, !BtAttPrepareWriteRequest(&dev, 1, 1, 0, tooLarge, 19));
	s_RefuseAcl = true;
	BT_CHECK(ctx, !BtAttPrepareWriteRequest(&dev, 1, 1, 0, value, 1));
	BT_CHECK(ctx, !BtAttExecuteWriteRequest(&dev, 1, true));
	s_RefuseAcl = false;
	BT_CHECK(ctx, s_CapturedAclLen == 0);
	BT_CHECK(ctx, !BtAttPrepareWriteRequest(nullptr, 1, 1, 0, value, 1));
	BT_CHECK(ctx, !BtAttExecuteWriteRequest(nullptr, 1, true));
	ctx.End();
}

static void TestL2capAttTransport(Context &ctx)
{
	static const Requirement req = {
		"HARNESS-L2CAP-ATT-BV-01", "Core Vol 3 Part A/F", "3 and 3.4.2",
		"ATT requests and responses cross isolated hosts in bounded L2CAP Basic Mode frames"
	};
	ctx.Begin(req);

	DualHostLink link;
	BT_CHECK(ctx, link.Start());

	uint16_t mtu = 0;
	BT_CHECK(ctx, ExchangeMtu(link, kCentralConn, BT_ATT_MTU_MIN, &mtu));
	BT_CHECK(ctx, mtu == BT_ATT_MTU_MIN);

	uint8_t readReq[3] = { BT_ATT_OPCODE_ATT_READ_REQ, 1, 0 };
	Pdu rsp = {};
	BT_CHECK(ctx, link.Exchange(kCentralConn, BT_L2CAP_CID_ATT,
							   readReq, sizeof(readReq), &rsp));
	BT_CHECK(ctx, rsp.Len == 1);
	BT_CHECK(ctx, rsp.Data[0] == BT_ATT_OPCODE_ATT_READ_RSP);

	BT_CHECK(ctx, link.Stop());
	ctx.End();
}

static void TestLongWriteMtuMatrix(Context &ctx)
{
	static const Requirement req = {
		"ATT-LONG-WRITE-BV-01", "Core Vol 3 Part F", "3.4.6",
		"Prepare Write and Execute Write commit atomically at MTU 23, 64, 128 and 247"
	};
	ctx.Begin(req);

	static const uint16_t mtus[] = { 23, 64, 128, 247 };
	for (size_t i = 0; i < sizeof(mtus) / sizeof(mtus[0]); ++i)
	{
		BT_CHECK(ctx, RunLongWriteMtuCase(mtus[i]));
	}
	ctx.End();
}

static void TestQueueBoundsAndAtomicFailure(Context &ctx)
{
	static const Requirement req = {
		"ATT-LONG-WRITE-BI-02", "Core Vol 3 Part F", "3.4.6.1 and 3.4.6.3",
		"prepare queue exhaustion and invalid final length fail without modifying the attribute"
	};
	ctx.Begin(req);

	DualHostLink link;
	BT_CHECK(ctx, link.Start());
	uint16_t mtu = 0;
	BT_CHECK(ctx, ExchangeMtu(link, kCentralConn, 247, &mtu));
	BT_CHECK(ctx, ExchangeMtu(link, kObserverConn, 247, nullptr));

	uint8_t baseline[32];
	FillPattern(baseline, sizeof(baseline), 0x41);
	BT_CHECK(ctx, PrepareLongWrite(link, kCentralConn, kOpenValueHandle,
								  baseline, sizeof(baseline), mtu));
	BT_CHECK(ctx, ExecuteWrite(link, kCentralConn, true, nullptr));
	BT_CHECK(ctx, ReadEquals(link, kObserverConn, kOpenValueHandle, mtu,
							baseline, sizeof(baseline)));

	uint8_t fragment[BT_ATT_MTU_MAX - 5];
	FillPattern(fragment, sizeof(fragment), 0x22);
	bool fullSeen = false;
	for (unsigned i = 0; i < 32; ++i)
	{
		Pdu rsp = {};
		if (!PrepareWrite(link, kCentralConn, kOpenValueHandle, 0,
						  fragment, (uint16_t)sizeof(fragment), &rsp))
		{
			fullSeen = IsError(rsp, BT_ATT_OPCODE_ATT_PREPARE_WRITE_REQ,
							   BT_ATT_ERROR_PREPARE_QUE_FULL);
			break;
		}
	}
	BT_CHECK(ctx, fullSeen);
	BT_CHECK(ctx, ExecuteWrite(link, kCentralConn, false, nullptr));
	BT_CHECK(ctx, ReadEquals(link, kObserverConn, kOpenValueHandle, mtu,
							baseline, sizeof(baseline)));

	uint8_t maxValue[kMaxValueLen];
	FillPattern(maxValue, sizeof(maxValue), 0x67);
	BT_CHECK(ctx, PrepareLongWrite(link, kCentralConn, kOpenValueHandle,
								  maxValue, sizeof(maxValue), mtu));
	BT_CHECK(ctx, ExecuteWrite(link, kCentralConn, true, nullptr));
	BT_CHECK(ctx, ReadEquals(link, kObserverConn, kOpenValueHandle, mtu,
							maxValue, sizeof(maxValue)));

	// A contiguous 513-byte value is accepted into the prepare queue but must
	// fail as one transaction at Execute Write. The prior 512-byte value stays.
	uint8_t tooLong[kMaxValueLen + 1];
	FillPattern(tooLong, sizeof(tooLong), 0xA5);
	BT_CHECK(ctx, PrepareLongWrite(link, kCentralConn, kOpenValueHandle,
								  tooLong, sizeof(tooLong), mtu));
	Pdu executeRsp = {};
	uint8_t executeReq[2] = { BT_ATT_OPCODE_ATT_EXECUTE_WRITE_REQ, 1 };
	BT_CHECK(ctx, link.Exchange(kCentralConn, BT_L2CAP_CID_ATT, executeReq,
							   sizeof(executeReq), &executeRsp));
	BT_CHECK(ctx, IsError(executeRsp, BT_ATT_OPCODE_ATT_EXECUTE_WRITE_REQ,
						 BT_ATT_ERROR_INVALID_ATT_VALUE));
	BT_CHECK(ctx, ReadEquals(link, kObserverConn, kOpenValueHandle, mtu,
							maxValue, sizeof(maxValue)));

	BT_CHECK(ctx, link.Stop());
	ctx.End();
}

static void TestPerConnectionQueues(Context &ctx)
{
	static const Requirement req = {
		"ATT-LONG-WRITE-BV-03", "Core Vol 3 Part F", "3.4.6",
		"prepared-write queues are isolated per connection and execute independently"
	};
	ctx.Begin(req);

	DualHostLink link;
	BT_CHECK(ctx, link.Start());
	uint16_t mtu1 = 0;
	uint16_t mtu2 = 0;
	BT_CHECK(ctx, ExchangeMtu(link, kCentralConn, 64, &mtu1));
	BT_CHECK(ctx, ExchangeMtu(link, kObserverConn, 64, &mtu2));

	uint8_t valueA[100];
	uint8_t valueB[100];
	FillPattern(valueA, sizeof(valueA), 0x19);
	FillPattern(valueB, sizeof(valueB), 0xD1);

	BT_CHECK(ctx, PrepareLongWrite(link, kCentralConn, kOpenValueHandle,
								  valueA, sizeof(valueA), mtu1));
	BT_CHECK(ctx, PrepareLongWrite(link, kObserverConn, kOpenValueHandle,
								  valueB, sizeof(valueB), mtu2));

	BT_CHECK(ctx, ExecuteWrite(link, kCentralConn, true, nullptr));
	BT_CHECK(ctx, ReadEquals(link, kCentralConn, kOpenValueHandle, mtu1,
							valueA, sizeof(valueA)));

	BT_CHECK(ctx, ExecuteWrite(link, kObserverConn, true, nullptr));
	BT_CHECK(ctx, ReadEquals(link, kCentralConn, kOpenValueHandle, mtu1,
							valueB, sizeof(valueB)));

	BT_CHECK(ctx, link.Stop());
	ctx.End();
}

static void TestSecurityTransition(Context &ctx)
{
	static const Requirement req = {
		"ATT-LONG-WRITE-BI-04", "Core Vol 3 Part F/C", "3.4.6 and 10.3",
		"authenticated long writes are rejected before security and accepted after authenticated encryption"
	};
	ctx.Begin(req);

	DualHostLink link;
	BT_CHECK(ctx, link.Start());
	uint16_t mtu = 0;
	BT_CHECK(ctx, ExchangeMtu(link, kCentralConn, 64, &mtu));

	uint8_t value[96];
	FillPattern(value, sizeof(value), 0x56);
	Pdu rsp = {};

	uint8_t prep[6] = {
		BT_ATT_OPCODE_ATT_PREPARE_WRITE_REQ,
		(uint8_t)(kSecureValueHandle & 0xFF),
		(uint8_t)(kSecureValueHandle >> 8),
		0, 0, value[0]
	};
	BT_CHECK(ctx, link.Exchange(kCentralConn, BT_L2CAP_CID_ATT,
							   prep, sizeof(prep), &rsp));
	BT_CHECK(ctx, IsError(rsp, BT_ATT_OPCODE_ATT_PREPARE_WRITE_REQ,
						 BT_ATT_ERROR_INSUF_AUTHEN));

	BT_CHECK(ctx, link.SetSecurity(kCentralConn,
								  BT_GAP_SEC_LEVEL_ENC_UNAUTH, 16,
								  BT_GAP_SEC_FLAG_SC));
	rsp = {};
	BT_CHECK(ctx, link.Exchange(kCentralConn, BT_L2CAP_CID_ATT,
							   prep, sizeof(prep), &rsp));
	BT_CHECK(ctx, IsError(rsp, BT_ATT_OPCODE_ATT_PREPARE_WRITE_REQ,
						 BT_ATT_ERROR_INSUF_AUTHEN));

	BT_CHECK(ctx, link.SetSecurity(kCentralConn,
								  BT_GAP_SEC_LEVEL_LESC_AUTH, 16,
								  BT_GAP_SEC_FLAG_SC));
	BT_CHECK(ctx, PrepareLongWrite(link, kCentralConn, kSecureValueHandle,
								  value, sizeof(value), mtu));
	BT_CHECK(ctx, ExecuteWrite(link, kCentralConn, true, nullptr));
	BT_CHECK(ctx, ReadEquals(link, kCentralConn, kSecureValueHandle, mtu,
							value, sizeof(value)));

	BT_CHECK(ctx, link.SetSecurity(kCentralConn,
								  BT_GAP_SEC_LEVEL_LESC_AUTH, 7,
								  BT_GAP_SEC_FLAG_SC));
	rsp = {};
	BT_CHECK(ctx, link.Exchange(kCentralConn, BT_L2CAP_CID_ATT,
							   prep, sizeof(prep), &rsp));
	BT_CHECK(ctx, IsError(rsp, BT_ATT_OPCODE_ATT_PREPARE_WRITE_REQ,
						 BT_ATT_ERROR_ENCRYPT_KEY_TOO_SHORT));

	BT_CHECK(ctx, link.Stop());
	ctx.End();
}

} // namespace

extern "C" {

uint32_t BtHciSendAcl(BtHciDevice_t * const pDev,
					  BtHciACLDataPacket_t * const pAcl)
{
	if (s_RefuseAcl || pDev == nullptr || pDev->SendData == nullptr ||
		pAcl == nullptr)
	{
		return 0;
	}
	uint32_t total = (uint32_t)pAcl->Hdr.Len + sizeof(pAcl->Hdr);
	return pDev->SendData((uint8_t*)pAcl, total);
}

BtDevice_t *BtPeerFindByHdl(uint16_t Hdl)
{
	return FindServerPeer(Hdl);
}

bool BtGapConnSecGet(uint16_t ConnHdl, BtConnSec_t *pSec)
{
	BtDevice_t *pPeer = FindServerPeer(ConnHdl);
	if (pPeer == nullptr || pSec == nullptr)
	{
		return false;
	}
	*pSec = pPeer->Conn.Sec;
	return true;
}

uint16_t BtGattCccdGet(uint16_t, uint16_t)
{
	return 0;
}

uint8_t BtGattCccdValueError(BtGattChar_t *, uint16_t)
{
	return 0;
}

bool BtGattCccdCanStore(uint16_t, uint16_t, uint16_t)
{
	return true;
}

bool BtGattCccdSet(uint16_t, uint16_t, uint16_t)
{
	return true;
}

void BtGattClientNotified(uint16_t, uint16_t, uint8_t *, uint16_t)
{
}

void BtGattHandleValueConfirm(uint16_t)
{
}

bool BtSmpSignVerify(uint16_t, const uint8_t *, size_t, const uint8_t *)
{
	return false;
}

} // extern "C"

int main()
{
	signal(SIGPIPE, SIG_IGN);

	Context ctx("Bluetooth dual-host ATT long-write compliance");
	TestClientRequestBuilders(ctx);
	TestL2capAttTransport(ctx);
	TestLongWriteMtuMatrix(ctx);
	TestQueueBoundsAndAtomicFailure(ctx);
	TestPerConnectionQueues(ctx);
	TestSecurityTransition(ctx);
	return ctx.Finish(false);
}
