from pathlib import Path


def read(path):
    return Path(path).read_text()


def write(path, text):
    Path(path).write_text(text)


def replace_once(path, old, new):
    text = read(path)
    count = text.count(old)
    if count != 1:
        raise RuntimeError(f"{path}: expected one match, found {count}")
    write(path, text.replace(old, new, 1))


# Execute Write: validate first, compact the original prepare queue into a
# canonical record list, install all final values, then issue callbacks.
path = "src/bluetooth/bt_att.cpp"
text = read(path)
start = text.index("static uint8_t BtAttExecLongWrite(")
end = text.index("// Resolve the attribute type", start)
new_func = r'''static uint8_t BtAttExecLongWrite(BtDevice_t *pConn, uint16_t *pFailHdl)
{
	if (pConn == nullptr || pConn->Conn.pLongWrBuff == nullptr)
	{
		return 0;
	}

	uint8_t *buf = pConn->Conn.pLongWrBuff;
	uint16_t total = pConn->Conn.LongWrLen;
	uint16_t pos = 0;
	uint8_t err = 0;

	BtGattCccdState_t oldCccd[BT_GATT_CCCD_STATE_MAX];
	BtGattCccdState_t planCccd[BT_GATT_CCCD_STATE_MAX];
	uint8_t oldCccdCount = pConn->Conn.NbCccd;
	uint8_t planCccdCount = oldCccdCount;
	memcpy(oldCccd, pConn->Conn.Cccd,
		   oldCccdCount * sizeof(BtGattCccdState_t));
	memcpy(planCccd, oldCccd,
		   oldCccdCount * sizeof(BtGattCccdState_t));

	// Pass 1: validate every run and project the final CCCD table. Nothing
	// in the attribute database or peer state is changed here.
	while (pos + 6 <= total)
	{
		uint16_t hdl, off, totLen, next;
		err = BtAttLongWrRun(buf, total, pos, false,
							&hdl, &off, &totLen, &next);
		if (err != 0)
		{
			*pFailHdl = hdl;
			break;
		}

		BtAttDBEntry_t *entry = BtAttDBFindHandle(hdl);
		if (entry == nullptr)
		{
			*pFailHdl = hdl;
			err = BT_ATT_ERROR_INVALID_HANDLE;
			break;
		}
		if (off > BtAttLongWrProjLen(buf, total, pos, hdl,
									BtAttCurValueLen(entry)))
		{
			*pFailHdl = hdl;
			err = BT_ATT_ERROR_INVALID_OFFSET;
			break;
		}

		BtGattChar_t *pChar =
				BtAttEntryIsCharValue(entry) ? BtAttEntryChar(entry) : nullptr;
		if (pChar != nullptr &&
			(uint32_t)off + totLen > pChar->MaxDataLen)
		{
			*pFailHdl = hdl;
			err = BT_ATT_ERROR_INVALID_ATT_VALUE;
			break;
		}

		if (BtAttEntryIsCccd(entry))
		{
			uint16_t reclen;
			memcpy(&reclen, buf + pos + 4, 2);
			if (off != 0 || totLen != 2 || reclen != 2 ||
				(uint32_t)pos + 8UL > total)
			{
				*pFailHdl = hdl;
				err = BT_ATT_ERROR_INVALID_ATT_VALUE;
				break;
			}
			const uint8_t *val = buf + pos + 6;
			uint16_t cccd = (uint16_t)(val[0] | (val[1] << 8));
			uint8_t cerr = BtGattCccdValueError(BtAttEntryChar(entry), cccd);
			if (cerr != 0)
			{
				*pFailHdl = hdl;
				err = cerr;
				break;
			}
			if (BtAttCccdPlanSet(planCccd, &planCccdCount, hdl, cccd) == false)
			{
				*pFailHdl = hdl;
				err = BT_ATT_ERROR_INSUF_RESOURCE;
				break;
			}
		}
		pos = next;
	}

	if (err == 0 && pos != total)
	{
		err = BT_ATT_ERROR_INVALID_PDU;
		*pFailHdl = 0;
	}
	if (err != 0)
	{
		pConn->Conn.LongWrLen = 0;
		return err;
	}

	// Pass 2: turn each contiguous run into one canonical queue record. The
	// destination never extends beyond the source run, so unread records are
	// not overwritten. Later passes walk only this canonical layout.
	uint16_t readPos = 0;
	uint16_t compactedTotal = 0;
	while (readPos + 6 <= total)
	{
		uint16_t hdl, off, totLen, next;
		err = BtAttLongWrRun(buf, total, readPos, true,
							&hdl, &off, &totLen, &next);
		if (err != 0)
		{
			*pFailHdl = hdl;
			pConn->Conn.LongWrLen = 0;
			return err;
		}

		uint8_t *dst = buf + compactedTotal;
		if (compactedTotal != readPos && totLen > 0)
		{
			memmove(dst + 6, buf + readPos + 6, totLen);
		}
		memcpy(dst, &hdl, 2);
		memcpy(dst + 2, &off, 2);
		memcpy(dst + 4, &totLen, 2);

		compactedTotal = (uint16_t)(compactedTotal + 6 + totLen);
		readPos = next;
	}

	// Pass 3: install every characteristic value without callbacks.
	pos = 0;
	while (pos + 6 <= compactedTotal)
	{
		uint16_t hdl, off, len;
		memcpy(&hdl, buf + pos, 2);
		memcpy(&off, buf + pos + 2, 2);
		memcpy(&len, buf + pos + 4, 2);

		BtAttDBEntry_t *entry = BtAttDBFindHandle(hdl);
		if (entry != nullptr && BtAttEntryIsCharValue(entry))
		{
			BtGattChar_t *pChar = BtAttEntryChar(entry);
			if (len > 0)
			{
				memcpy((uint8_t*)pChar->pValue + off, buf + pos + 6, len);
			}
			uint16_t valueEnd = (uint16_t)(off + len);
			if (valueEnd > pChar->ValueLen)
			{
				pChar->ValueLen = valueEnd;
			}
		}
		pos = (uint16_t)(pos + 6 + len);
	}

	// The CCCD plan was fully validated, so this final-state copy cannot fail.
	pConn->Conn.NbCccd = planCccdCount;
	memcpy(pConn->Conn.Cccd, planCccd,
		   planCccdCount * sizeof(BtGattCccdState_t));

	// Pass 4: all values are final before the first callback or persistence
	// hook. Each canonical characteristic run produces exactly one callback.
	uint16_t doneCccd[BT_GATT_CCCD_STATE_MAX];
	uint8_t doneCccdCount = 0;
	pos = 0;
	while (pos + 6 <= compactedTotal)
	{
		uint16_t hdl, off, len;
		memcpy(&hdl, buf + pos, 2);
		memcpy(&off, buf + pos + 2, 2);
		memcpy(&len, buf + pos + 4, 2);
		BtAttDBEntry_t *entry = BtAttDBFindHandle(hdl);

		if (entry != nullptr && BtAttEntryIsCharValue(entry))
		{
			BtGattChar_t *pChar = BtAttEntryChar(entry);
			if (pChar != nullptr && pChar->WrCB != nullptr)
			{
				pChar->WrCB(pChar, buf + pos + 6, (int)off, (int)len);
			}
		}
		else if (entry != nullptr && BtAttEntryIsCccd(entry))
		{
			bool seen = false;
			for (uint8_t i = 0; i < doneCccdCount; i++)
			{
				seen = seen || doneCccd[i] == hdl;
			}
			if (!seen)
			{
				doneCccd[doneCccdCount++] = hdl;
				BtGattCccdChanged(pConn->Conn.Hdl, hdl,
					BtAttCccdListGet(oldCccd, oldCccdCount, hdl));
			}
		}
		pos = (uint16_t)(pos + 6 + len);
	}

	pConn->Conn.LongWrLen = 0;
	return 0;
}

'''
write(path, text[:start] + new_func + text[end:])


# Consecutive null-owner ACL groups are equivalent and may share one ring
# entry. They are never merged across a tracked operation, so ordering remains
# exact while ordinary ATT/SMP traffic cannot consume the whole ring as fast.
replace_once(
    "src/bluetooth/bt_gatt.cpp",
    r'''// Account for HCI packets this link sent that carry no GATT completion: ATT
// responses, SMP PDUs, L2CAP signaling. They own no callback but the controller
// still reports them, and without this their completions drain the ring and
// fire someone else's callback early.
//
// They are counted rather than given ring entries, so ordinary request and
// response traffic cannot exhaust the ring and refuse a notification.
bool BtGattTxPendUntracked(uint16_t ConnHdl, uint16_t NbPkt)
{
	return BtGattTxPendReserve(ConnHdl, nullptr, NbPkt);
}''',
    r'''// Account for HCI packets this link sent that carry no GATT completion: ATT
// responses, SMP PDUs and L2CAP signaling. A null owner is still an ordered
// ring entry, so its completion cannot be charged to a notification sent
// before or after it. Adjacent null-owner groups may be coalesced because no
// callback boundary exists between them.
bool BtGattTxPendUntracked(uint16_t ConnHdl, uint16_t NbPkt)
{
	BtDevice_t *pConn = BtPeerFindByHdl(ConnHdl);
	if (pConn == nullptr || NbPkt == 0)
	{
		return false;
	}

	if (pConn->TxPendCount > 0)
	{
		uint8_t idx = (uint8_t)((pConn->TxPendHead +
								 pConn->TxPendCount - 1) % BT_DEV_TXPEND_MAX);
		if (pConn->TxPend[idx].pChar == nullptr &&
			(uint32_t)pConn->TxPend[idx].Remain + NbPkt <= UINT16_MAX)
		{
			pConn->TxPend[idx].Remain =
					(uint16_t)(pConn->TxPend[idx].Remain + NbPkt);
			return true;
		}
	}

	return BtGattTxPendReserve(ConnHdl, nullptr, NbPkt);
}'''
)

replace_once(
    "tests/bluetooth/host/bt_gatt_txcomplete_test.cpp",
    r'''// Packets with no owner accumulate in the counter, never in the ring.
void TestUntrackedCoalesces()
{
	Setup(64);

	BtGattTxPendUntracked(kConnHdl, 1);
	BtGattTxPendUntracked(kConnHdl, 1);
	BtGattTxPendUntracked(kConnHdl, 1);
	CHECK(Peer()->TxPendCount == 3);

	CHECK(BtGattCharNotify(kConnHdl, &s_Char[0], s_Value, 8));
	CHECK(Peer()->TxPendCount == 4);

	BtGattSendCompleted(kConnHdl, 3);
	CHECK(s_TxCompleteCount == 0);
	CHECK(Peer()->TxPendCount == 1);
}''',
    r'''// Consecutive packets with no callback owner share one ordered group. A
// tracked packet still creates a boundary that later untracked traffic cannot
// cross.
void TestUntrackedCoalesces()
{
	Setup(64);

	CHECK(BtGattTxPendUntracked(kConnHdl, 1));
	CHECK(BtGattTxPendUntracked(kConnHdl, 1));
	CHECK(BtGattTxPendUntracked(kConnHdl, 1));
	CHECK(Peer()->TxPendCount == 1);
	CHECK(Peer()->TxPend[Peer()->TxPendHead].Remain == 3);

	CHECK(BtGattCharNotify(kConnHdl, &s_Char[0], s_Value, 8));
	CHECK(Peer()->TxPendCount == 2);
	CHECK(BtGattTxPendUntracked(kConnHdl, 2));
	CHECK(Peer()->TxPendCount == 3);

	BtGattSendCompleted(kConnHdl, 3);
	CHECK(s_TxCompleteCount == 0);
	CHECK(Peer()->TxPendCount == 2);
	BtGattSendCompleted(kConnHdl, 1);
	CHECK(s_TxCompleteCount == 1);
	CHECK(Peer()->TxPendCount == 1);
}'''
)


# Callback-level Execute Write regression: two fragments for the first value,
# a second attribute in the same transaction, and one callback only after both
# final values have been installed.
replace_once(
    "tests/bluetooth/host/bt_gatt_adversarial_test.cpp",
    r'''bool s_LastNotifyState = false;
bool s_LastIndicateState = false;
''',
    r'''bool s_LastNotifyState = false;
bool s_LastIndicateState = false;
int s_LongWriteCallbacks = 0;
int s_LongWriteOffset = -1;
int s_LongWriteLen = -1;
uint8_t s_LongWriteData[16];
bool s_LongWriteSawPeerFinal = false;
BtGattChar_t *s_LongWritePeerChar = nullptr;
'''
)

replace_once(
    "tests/bluetooth/host/bt_gatt_adversarial_test.cpp",
    r'''void IndicateChanged(BtGattChar_t *, bool Enabled, uint16_t)
{
	s_IndicateTransitions++;
	s_LastIndicateState = Enabled;
}
''',
    r'''void IndicateChanged(BtGattChar_t *, bool Enabled, uint16_t)
{
	s_IndicateTransitions++;
	s_LastIndicateState = Enabled;
}

void LongWriteChanged(BtGattChar_t *, uint8_t *pData, int Offset, int Len)
{
	s_LongWriteCallbacks++;
	s_LongWriteOffset = Offset;
	s_LongWriteLen = Len;
	if (Len > 0 && Len <= (int)sizeof(s_LongWriteData))
	{
		std::memcpy(s_LongWriteData, pData, (size_t)Len);
	}
	static const uint8_t expected[] = { 0x91, 0x92, 0x93 };
	s_LongWriteSawPeerFinal = s_LongWritePeerChar != nullptr &&
		s_LongWritePeerChar->ValueLen == sizeof(expected) &&
		std::memcmp(s_LongWritePeerChar->pValue,
					expected, sizeof(expected)) == 0;
}
'''
)

replace_once(
    "tests/bluetooth/host/bt_gatt_adversarial_test.cpp",
    r'''void TestExecuteWriteCccdCapacityIsAtomic()
{''',
    r'''void TestExecuteWriteCallbacksUseCanonicalRuns()
{
	static BtGattChar_t chars[2];
	static BtGattSrvc_t service;
	static bool registered = false;
	static uint8_t queue[96];

	if (!registered)
	{
		std::memset(chars, 0, sizeof(chars));
		std::memset(&service, 0, sizeof(service));
		for (int i = 0; i < 2; i++)
		{
			chars[i].Uuid = (uint16_t)(0xFFC0 + i);
			chars[i].MaxDataLen = 16;
			chars[i].Property = BT_GATT_CHAR_PROP_READ |
							 BT_GATT_CHAR_PROP_WRITE;
		}
		chars[0].WrCB = LongWriteChanged;
		service.UuidSrvc = 0xFFBF;
		service.NbChar = 2;
		service.pCharArray = chars;
		BT_CHECK(s_Test, BtGattSrvcAdd(&service));
		registered = true;
	}

	s_Peer.Conn.pLongWrBuff = queue;
	s_Peer.Conn.LongWrBuffSize = sizeof(queue);
	s_Peer.Conn.LongWrLen = 0;
	s_LongWriteCallbacks = 0;
	s_LongWriteOffset = -1;
	s_LongWriteLen = -1;
	std::memset(s_LongWriteData, 0, sizeof(s_LongWriteData));
	s_LongWriteSawPeerFinal = false;
	s_LongWritePeerChar = &chars[1];

	static const uint8_t first[] = { 1, 2, 3 };
	static const uint8_t second[] = { 4, 5, 6, 7 };
	static const uint8_t peer[] = { 0x91, 0x92, 0x93 };
	struct Fragment {
		uint16_t Hdl;
		uint16_t Offset;
		const uint8_t *pData;
		uint16_t Len;
	};
	const Fragment fragments[] = {
		{ chars[0].ValHdl, 0, first, sizeof(first) },
		{ chars[0].ValHdl, sizeof(first), second, sizeof(second) },
		{ chars[1].ValHdl, 0, peer, sizeof(peer) },
	};

	for (const Fragment &f : fragments)
	{
		uint8_t request[16] = {};
		uint8_t response[16] = {};
		request[0] = BT_ATT_OPCODE_ATT_PREPARE_WRITE_REQ;
		request[1] = (uint8_t)(f.Hdl & 0xFF);
		request[2] = (uint8_t)(f.Hdl >> 8);
		request[3] = (uint8_t)(f.Offset & 0xFF);
		request[4] = (uint8_t)(f.Offset >> 8);
		std::memcpy(request + 5, f.pData, f.Len);
		uint32_t len = BtAttProcessReq(kConnHdl,
			(BtAttReqRsp_t *)request, (uint16_t)(5 + f.Len),
			(BtAttReqRsp_t *)response);
		BT_CHECK(s_Test, len == 5 + f.Len);
		BT_CHECK(s_Test,
			response[0] == BT_ATT_OPCODE_ATT_PREPARE_WRITE_RSP);
	}

	BtAttReqRsp_t execute = {};
	BtAttReqRsp_t response = {};
	execute.OpCode = BT_ATT_OPCODE_ATT_EXECUTE_WRITE_REQ;
	execute.ExecuteWriteReq.Flag = 1;
	uint32_t len = BtAttProcessReq(kConnHdl, &execute, 2, &response);

	static const uint8_t expected[] = { 1, 2, 3, 4, 5, 6, 7 };
	BT_CHECK(s_Test, len == 1);
	BT_CHECK(s_Test,
		response.OpCode == BT_ATT_OPCODE_ATT_EXECUTE_WRITE_RSP);
	BT_CHECK(s_Test, s_LongWriteCallbacks == 1);
	BT_CHECK(s_Test, s_LongWriteOffset == 0);
	BT_CHECK(s_Test, s_LongWriteLen == (int)sizeof(expected));
	BT_CHECK(s_Test,
		std::memcmp(s_LongWriteData, expected, sizeof(expected)) == 0);
	BT_CHECK(s_Test, chars[0].ValueLen == sizeof(expected));
	BT_CHECK(s_Test,
		std::memcmp(chars[0].pValue, expected, sizeof(expected)) == 0);
	BT_CHECK(s_Test, s_LongWriteSawPeerFinal);
}

void TestExecuteWriteCccdCapacityIsAtomic()
{'''
)

replace_once(
    "tests/bluetooth/host/bt_gatt_adversarial_test.cpp",
    r'''	s_Test.Run("execute write CCCD capacity atomicity",
			   TestExecuteWriteCccdCapacityIsAtomic);
''',
    r'''	s_Test.Run("execute write canonical callback",
			   TestExecuteWriteCallbacksUseCanonicalRuns);
	s_Test.Run("execute write CCCD capacity atomicity",
			   TestExecuteWriteCccdCapacityIsAtomic);
'''
)

print("final ATT and completion review fixes applied")
