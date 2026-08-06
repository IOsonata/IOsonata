from pathlib import Path

source_path = Path("src/bluetooth/bt_hci_host.cpp")
source = source_path.read_text()
start = source.index("uint32_t BtHciSendAcl(")
end = source.index("\nvoid BtHciProcessEvent", start)
replacement = r'''uint32_t BtHciSendAcl(BtHciDevice_t * const pDev, BtHciACLDataPacket_t * const pAcl)
{
	if (pDev == nullptr || pDev->SendData == nullptr || pAcl == nullptr)
	{
		return 0;
	}

	uint16_t l2Len = (uint16_t)pAcl->Hdr.Len;	// L2CAP PDU bytes (header + payload)
	const BtL2CapHdr_t *pL2Hdr = l2Len >= sizeof(BtL2CapHdr_t) ?
			(const BtL2CapHdr_t*)pAcl->Data : nullptr;
	bool bSmp = pL2Hdr != nullptr &&
			pL2Hdr->Cid == BT_L2CAP_CID_SEC_MNGR &&
			(uint32_t)pL2Hdr->Len + sizeof(BtL2CapHdr_t) == l2Len;
	uint16_t nFrag = (pDev->AclMaxLen == 0 || l2Len <= pDev->AclMaxLen) ? 1 :
			(uint16_t)((l2Len + pDev->AclMaxLen - 1) / pDev->AclMaxLen);

	// Every controller packet must occupy the same per-connection completion
	// order used by GATT. SMP previously bypassed this queue, so its completion
	// could consume a later notification entry and fire that callback early.
	// Reserve the whole PDU before any fragment can enter the controller.
	if (pDev->AclCreditMax > 0 && pDev->AclCredit < (int16_t)nFrag)
	{
		if (bSmp)
		{
			// SMP callers advance their state immediately after this void-style
			// send path. With no packet sent, terminate the link instead of
			// letting the pairing state describe a PDU the peer never received.
			BtHciDropLink(pDev, pAcl->Hdr.ConnHdl);
		}
		return 0;
	}
	if (bSmp && BtGattTxPendUntracked(pAcl->Hdr.ConnHdl, nFrag) == false)
	{
		// The completion ring is part of ACL ordering. Sending without a slot
		// would corrupt callback attribution, so fail the security procedure
		// closed before handing any bytes to the controller.
		BtHciDropLink(pDev, pAcl->Hdr.ConnHdl);
		return 0;
	}

	// Single-packet path: used when fragmentation is not configured or the PDU
	// already fits one ACL data packet.
	if (nFrag == 1)
	{
		uint32_t txLen = (uint32_t)l2Len + sizeof(pAcl->Hdr);
		uint32_t sent = pDev->SendData((uint8_t*)pAcl, txLen);
		if (sent != txLen)
		{
			if (bSmp)
			{
				// No controller packet was accepted, so remove the reservation
				// and terminate the pairing link before its caller advances.
				BtGattTxPendRelease(pAcl->Hdr.ConnHdl);
				BtHciDropLink(pDev, pAcl->Hdr.ConnHdl);
			}
			return 0;
		}
		if (pDev->AclCreditMax > 0)
		{
			pDev->AclCredit--;
		}
		return sent;
	}

	// Fragmentation path: take the credits for the whole PDU before the first
	// fragment goes out. This prevents another sender from spending credits
	// needed to finish a PDU whose START is already in the controller.
	if (pDev->AclCreditMax > 0)
	{
		pDev->AclCredit = (int16_t)(pDev->AclCredit - (int16_t)nFrag);
	}

	uint8_t fbuf[BT_HCI_BUFFER_MAX_SIZE];
	BtHciACLDataPacket_t *frag = (BtHciACLDataPacket_t*)fbuf;
	const uint8_t *src = pAcl->Data;
	uint16_t off = 0;
	uint16_t nSent = 0;

	while (off < l2Len)
	{
		uint16_t chunk = (uint16_t)(l2Len - off);
		if (chunk > pDev->AclMaxLen)
		{
			chunk = pDev->AclMaxLen;
		}

		frag->Hdr.ConnHdl = pAcl->Hdr.ConnHdl;
		frag->Hdr.PBFlag = (off == 0) ? BT_HCI_PBFLAG_START_NONFLUSHABLE
									  : BT_HCI_PBFLAG_CONTINUING_FRAGMENT;
		frag->Hdr.BCFlag = 0;
		frag->Hdr.Len = chunk;
		memcpy(frag->Data, src + off, chunk);

		uint32_t txLen = (uint32_t)chunk + sizeof(frag->Hdr);
		uint32_t sent = pDev->SendData((uint8_t*)frag, txLen);
		if (sent != txLen)
		{
			// Hand back credits for fragments that never went out. Fragments
			// already accepted stay spent until the controller completes them.
			if (pDev->AclCreditMax > 0)
			{
				pDev->AclCredit = (int16_t)(pDev->AclCredit +
						(int16_t)(nFrag - nSent));
			}

			if (bSmp && nSent == 0)
			{
				// No fragment can complete this reservation.
				BtGattTxPendRelease(pAcl->Hdr.ConnHdl);
			}
			if (nSent > 0 || bSmp)
			{
				// A partial L2CAP PDU cannot be withdrawn, while an unsent SMP
				// PDU would leave the pairing state ahead of the wire. Both
				// cases require terminating the link.
				BtHciDropLink(pDev, pAcl->Hdr.ConnHdl);
			}
			return 0;
		}

		nSent++;
		off = (uint16_t)(off + chunk);
	}

	return (uint32_t)l2Len + sizeof(pAcl->Hdr);
}
'''
source_path.write_text(source[:start] + replacement + source[end:])

makefile_path = Path("tests/bluetooth/host/Makefile")
makefile = makefile_path.read_text()
name_marker = "\tbt_hci_flow_test \\\n"
assert makefile.count(name_marker) == 1
makefile = makefile.replace(
    name_marker,
    name_marker + "\tbt_hci_smp_tx_test \\\n",
)
source_marker = """bt_hci_flow_test_SOURCES := \\
\tbt_hci_flow_test.cpp \\
\t$(HCI_GATT_TEST_STUB) \\
\t$(ROOT)/src/bluetooth/bt_hci_host.cpp \\
\t$(ROOT)/src/bluetooth/bt_hci_ctlr.cpp

"""
assert makefile.count(source_marker) == 1
makefile = makefile.replace(
    source_marker,
    source_marker + """bt_hci_smp_tx_test_SOURCES := \\
\tbt_hci_smp_tx_test.cpp \\
\t$(ROOT)/src/bluetooth/bt_hci_host.cpp

""",
)
makefile_path.write_text(makefile)

Path(".tmp_smp_tx_patch.py").unlink()
Path(".github/workflows/smp_tx_fix.yml").unlink()
