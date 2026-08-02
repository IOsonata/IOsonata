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


path = "src/bluetooth/bt_smp.cpp"

replace_once(
    path,
    '''\tstruct {\n\t\tuint8_t Len;\n\t\tuint8_t Data[BT_SMP_TX_PDU_MAX];\n\t} TxQueue[BT_SMP_TX_QUEUE_DEPTH];\n\tuint8_t TxHead;\n\tuint8_t TxCount;\n\tBtHciDevice_t *pTxDev;''',
    '''\tstruct {\n\t\tuint8_t Len;\n\t\tbool BlocksRx;\n\t\tuint8_t Data[BT_SMP_TX_PDU_MAX];\n\t} TxQueue[BT_SMP_TX_QUEUE_DEPTH];\n\tuint8_t TxHead;\n\tuint8_t TxCount;\n\tuint8_t TxBarrierCount;\n\tBtHciDevice_t *pTxDev;''',
)

replace_once(
    path,
    '''static inline bool SmpPairingTimedOut(const BtSmpLink_t *pLink)\n{\n\treturn SmpPairingActive(pLink->Ctx.State) &&\n\t\t   (uint32_t)(BtSmpMsTick() - pLink->Ctx.TmrStart) >= BT_SMP_TIMEOUT_MS;\n}''',
    '''static inline bool SmpPairingTimedOut(const BtSmpLink_t *pLink)\n{\n\t// A queued handshake PDU has not crossed HCI yet. Its peer response cannot\n\t// legitimately exist and the SMP response timer must not run until the\n\t// prerequisite packet has actually been accepted by the controller.\n\treturn pLink->TxBarrierCount == 0 &&\n\t\t   SmpPairingActive(pLink->Ctx.State) &&\n\t\t   (uint32_t)(BtSmpMsTick() - pLink->Ctx.TmrStart) >= BT_SMP_TIMEOUT_MS;\n}''',
)

replace_once(
    path,
    '''static void SmpTxPump(BtSmpLink_t *pLink)\n{''',
    '''// These phase-1/2 packets establish the state in which the next peer PDU\n// is meaningful. If one is queued but not yet accepted by HCI, an inbound PDU\n// that appears to answer it is early and must not drive the state machine. Key\n// distribution is intentionally excluded: both sides may distribute keys once\n// encryption is enabled and those directions are independent.\nstatic bool SmpTxPduBlocksRx(uint8_t Code)\n{\n\tswitch (Code)\n\t{\n\t\tcase BT_SMP_CODE_PAIRING_REQ:\n\t\tcase BT_SMP_CODE_PAIRING_RSP:\n\t\tcase BT_SMP_CODE_PAIRING_CONFIRM:\n\t\tcase BT_SMP_CODE_PAIRING_RANDOM:\n\t\tcase BT_SMP_CODE_PAIRING_PUBLIC_KEY:\n\t\tcase BT_SMP_CODE_PAIRING_DHKEY_CHECK:\n\t\t\treturn true;\n\t\tdefault:\n\t\t\treturn false;\n\t}\n}\n\nstatic void SmpTxPump(BtSmpLink_t *pLink)\n{''',
)

replace_once(
    path,
    '''\t\tpLink->Ctx.TmrStart = BtSmpMsTick();\n\t\tSMP_TRACE_PDU("TX", item.Data[0], (int)pLink->Ctx.State);\n\t\tpLink->TxHead = (uint8_t)((pLink->TxHead + 1) % BT_SMP_TX_QUEUE_DEPTH);\n\t\tpLink->TxCount--;''',
    '''\t\tpLink->Ctx.TmrStart = BtSmpMsTick();\n\t\tSMP_TRACE_PDU("TX", item.Data[0], (int)pLink->Ctx.State);\n\t\tif (item.BlocksRx && pLink->TxBarrierCount > 0)\n\t\t{\n\t\t\tpLink->TxBarrierCount--;\n\t\t}\n\t\tpLink->TxHead = (uint8_t)((pLink->TxHead + 1) % BT_SMP_TX_QUEUE_DEPTH);\n\t\tpLink->TxCount--;''',
)

replace_once(
    path,
    '''\tpLink->TxQueue[idx].Len = (uint8_t)Len;\n\tmemcpy(pLink->TxQueue[idx].Data, pData, Len);\n\tpLink->TxCount++;\n\tpLink->pTxDev = pDev;''',
    '''\tpLink->TxQueue[idx].Len = (uint8_t)Len;\n\tmemcpy(pLink->TxQueue[idx].Data, pData, Len);\n\tpLink->TxQueue[idx].BlocksRx =\n\t\tSmpTxPduBlocksRx(pLink->TxQueue[idx].Data[0]);\n\tif (pLink->TxQueue[idx].BlocksRx)\n\t{\n\t\tpLink->TxBarrierCount++;\n\t}\n\tpLink->TxCount++;\n\tpLink->pTxDev = pDev;''',
)

replace_once(
    path,
    '''\tif (pLink != nullptr)\n\t{\n\t\tpLink->TxHead = 0;\n\t\tpLink->TxCount = 0;\n\t\tpLink->pTxDev = pDev;\n\t}''',
    '''\tif (pLink != nullptr)\n\t{\n\t\tpLink->TxHead = 0;\n\t\tpLink->TxCount = 0;\n\t\tpLink->TxBarrierCount = 0;\n\t\tpLink->pTxDev = pDev;\n\t}''',
)

replace_once(
    path,
    '''\tif (pLink->Ctx.bLocked)\n\t{\n\t\tSmpSendFailed(pDev, ConnHdl, BT_SMP_ERR_REPEATED_ATTEMPTS);\n\t\treturn;\n\t}\n\n\t// Anchor the pairing timer when the exchange begins (state still IDLE),''',
    '''\tif (pLink->Ctx.bLocked)\n\t{\n\t\tSmpSendFailed(pDev, ConnHdl, BT_SMP_ERR_REPEATED_ATTEMPTS);\n\t\treturn;\n\t}\n\n\t// The local state may already describe the response expected after a PDU\n\t// that is still queued behind HCI flow control. Do not accept a peer PDU\n\t// until every prerequisite phase-1/2 packet has actually left this host.\n\tif (pLink->TxBarrierCount > 0)\n\t{\n\t\treturn;\n\t}\n\n\t// Anchor the pairing timer when the exchange begins (state still IDLE),''',
)


path = "tests/bluetooth/compliance/smp/bt_smp_dual_host_test.cpp"
old = '''static void TestTxFailureQueuesAndRetries(Context &ctx)\n{\n\tstatic const Requirement req = {\n\t\t"SMP-ACL-FLOW-BI-02", "Core Vol 3 Part H / Vol 4 Part E", "3.4 and 5.4.2",\n\t\t"an SMP PDU refused by the ACL transport is retained and sent after credits return"\n\t};\n\tctx.Begin(req);\n\n\tChildPeer peer = {};\n\tBT_CHECK(ctx, SpawnPeer(&peer));\n\tconst uint8_t local[6] = { 1, 2, 3, 4, 5, 0xC0 };\n\tconst uint8_t remote[6] = { 6, 7, 8, 9, 10, 0xD0 };\n\tBT_CHECK(ctx, SendInit(&peer, BT_CONN_ROLE_CENTRAL,\n\t\tBT_SMP_IOCAPS_NO_INPUT_NO_OUTPUT,\n\t\tBT_SMP_AUTHREQ_BONDING_FLAG_BONDING | BT_SMP_AUTHREQ_SC,\n\t\tBTADDR_TYPE_RAND, local, BTADDR_TYPE_RAND, remote));\n\n\tWireMessage block = {};\n\tblock.Type = WIRE_CMD_SET_TX_BLOCK;\n\tblock.Value = 1;\n\tBT_CHECK(ctx, SendWire(peer.CommandFd, block));\n\n\tWireMessage start = {};\n\tstart.Type = WIRE_CMD_START_PAIRING;\n\tBT_CHECK(ctx, SendWire(peer.CommandFd, start));\n\n\tpollfd pfd = { peer.EventFd, POLLIN, 0 };\n\tBT_CHECK(ctx, poll(&pfd, 1, 50) == 0);\n\n\tblock.Value = 0;\n\tBT_CHECK(ctx, SendWire(peer.CommandFd, block));\n\tWireMessage pump = {};\n\tpump.Type = WIRE_CMD_PUMP_TX;\n\tBT_CHECK(ctx, SendWire(peer.CommandFd, pump));\n\n\tWireMessage event = {};\n\tBT_CHECK(ctx, ReceiveWire(peer.EventFd, &event));\n\tBT_CHECK(ctx, event.Type == WIRE_EVT_TX_SMP);\n\tBT_CHECK(ctx, event.Length > 0);\n\tBT_CHECK(ctx, event.Data[0] == BT_SMP_CODE_PAIRING_REQ);\n\n\tStopPeer(&peer);\n\tctx.End();\n}'''
new = '''static void TestTxFailureQueuesAndRetries(Context &ctx)\n{\n\tstatic const Requirement req = {\n\t\t"SMP-ACL-FLOW-BI-02", "Core Vol 3 Part H / Vol 4 Part E", "3.4 and 5.4.2",\n\t\t"a refused SMP PDU is retained, and an early peer response cannot advance the state before transmission"\n\t};\n\tctx.Begin(req);\n\n\tChildPeer peer = {};\n\tBT_CHECK(ctx, SpawnPeer(&peer));\n\tconst uint8_t local[6] = { 1, 2, 3, 4, 5, 0xC0 };\n\tconst uint8_t remote[6] = { 6, 7, 8, 9, 10, 0xD0 };\n\tBT_CHECK(ctx, SendInit(&peer, BT_CONN_ROLE_CENTRAL,\n\t\tBT_SMP_IOCAPS_NO_INPUT_NO_OUTPUT,\n\t\tBT_SMP_AUTHREQ_BONDING_FLAG_BONDING | BT_SMP_AUTHREQ_SC,\n\t\tBTADDR_TYPE_RAND, local, BTADDR_TYPE_RAND, remote));\n\n\tWireMessage block = {};\n\tblock.Type = WIRE_CMD_SET_TX_BLOCK;\n\tblock.Value = 1;\n\tBT_CHECK(ctx, SendWire(peer.CommandFd, block));\n\n\tWireMessage start = {};\n\tstart.Type = WIRE_CMD_START_PAIRING;\n\tBT_CHECK(ctx, SendWire(peer.CommandFd, start));\n\n\tpollfd pfd = { peer.EventFd, POLLIN, 0 };\n\tBT_CHECK(ctx, poll(&pfd, 1, 50) == 0);\n\n\t// This looks like a valid answer to the queued Pairing Request, but that\n\t// request has not crossed HCI. It must be ignored rather than producing a\n\t// Public Key behind the still-unsent request.\n\tconst uint8_t pairingRsp[] = {\n\t\tBT_SMP_CODE_PAIRING_RSP,\n\t\tBT_SMP_IOCAPS_NO_INPUT_NO_OUTPUT,\n\t\t0,\n\t\t(uint8_t)(BT_SMP_AUTHREQ_BONDING_FLAG_BONDING | BT_SMP_AUTHREQ_SC),\n\t\t16, 0, 0\n\t};\n\tWireMessage response = {};\n\tresponse.Type = WIRE_CMD_RX_SMP;\n\tresponse.Length = sizeof(pairingRsp);\n\tmemcpy(response.Data, pairingRsp, sizeof(pairingRsp));\n\tBT_CHECK(ctx, SendWire(peer.CommandFd, response));\n\tBT_CHECK(ctx, poll(&pfd, 1, 50) == 0);\n\n\tblock.Value = 0;\n\tBT_CHECK(ctx, SendWire(peer.CommandFd, block));\n\tWireMessage pump = {};\n\tpump.Type = WIRE_CMD_PUMP_TX;\n\tBT_CHECK(ctx, SendWire(peer.CommandFd, pump));\n\n\tWireMessage event = {};\n\tBT_CHECK(ctx, ReceiveWire(peer.EventFd, &event));\n\tBT_CHECK(ctx, event.Type == WIRE_EVT_TX_SMP);\n\tBT_CHECK(ctx, event.Length > 0);\n\tBT_CHECK(ctx, event.Data[0] == BT_SMP_CODE_PAIRING_REQ);\n\n\t// The early response was discarded, not deferred. No Public Key may be\n\t// waiting after the Pairing Request finally leaves.\n\tpfd.revents = 0;\n\tBT_CHECK(ctx, poll(&pfd, 1, 50) == 0);\n\n\t// The same response is valid now that its prerequisite packet was sent.\n\tBT_CHECK(ctx, SendWire(peer.CommandFd, response));\n\tBT_CHECK(ctx, ReceiveWire(peer.EventFd, &event));\n\tBT_CHECK(ctx, event.Type == WIRE_EVT_TX_SMP);\n\tBT_CHECK(ctx, event.Length > 0);\n\tBT_CHECK(ctx, event.Data[0] == BT_SMP_CODE_PAIRING_PUBLIC_KEY);\n\n\tStopPeer(&peer);\n\tctx.End();\n}'''
text = read(path)
if text.count(old) != 1:
    raise RuntimeError(f"{path}: expected blocked-TX test once, found {text.count(old)}")
write(path, text.replace(old, new, 1))

print("SMP transmission barrier and early-response test applied")
