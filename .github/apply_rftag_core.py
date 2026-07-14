from pathlib import Path


def replace_one(path: str, old: str, new: str) -> None:
	file = Path(path)
	text = file.read_text(encoding="ascii")
	count = text.count(old)
	if count != 1:
		raise RuntimeError(f"{path}: expected one match, found {count}")
	file.write_text(text.replace(old, new, 1), encoding="ascii", newline="\n")


replace_one(
	"include/rftag/rftag.h",
	"\tvirtual int OnFrame(const uint8_t *pRx, int RxLen, uint8_t *pTx, int TxCap) = 0;\n\nprotected:\n",
	"\tvirtual int OnFrame(const uint8_t *pRx, int RxLen, uint8_t *pTx, int TxCap) = 0;\n\n"
	"\t/**\n"
	"\t * @brief\tNumber of response bits from the last OnFrame call.\n"
	"\t *\n"
	"\t * Byte framed protocols use TxLen * 8. A protocol with a short\n"
	"\t * response overrides this method.\n"
	"\t */\n"
	"\tvirtual int ResponseBits(int TxLen) const { return TxLen > 0 ? TxLen * 8 : 0; }\n\n"
	"protected:\n",
)

replace_one(
	"include/rftag/rftag.h",
	"\tvirtual int ProcessFrame(const uint8_t *pRx, int RxLen, uint8_t *pTx, int TxCap);\n\n\t/**\n\t * @brief\tRead tag memory. The base reads the local image.\n",
	"\tvirtual int ProcessFrame(const uint8_t *pRx, int RxLen, uint8_t *pTx, int TxCap);\n\n"
	"\t/**\n"
	"\t * @brief\tNumber of response bits from the last ProcessFrame call.\n"
	"\t */\n"
	"\tint ResponseBits(int TxLen) const {\n"
	"\t\treturn vpProto != nullptr ? vpProto->ResponseBits(TxLen) : 0;\n"
	"\t}\n\n"
	"\t/**\n"
	"\t * @brief\tRead tag memory. The base reads the local image.\n",
)

replace_one(
	"src/rftag/rftag.cpp",
	'''bool RFTag::Init(const RFTagCfg_t &Cfg, DeviceIntrf * const pIntrf)
{
	// A failed Init must not leave a previously initialized object usable.
	Valid(false);

	vCfg = Cfg;
	vpProto = nullptr;

	Interface(pIntrf);

	// The base is a local memory tag. It requires the image and a NDEF area
	// inside it. A bus tag subclass overrides Init and lifts this.
	if (vCfg.pMem == nullptr || vCfg.MemSize == 0)
	{
		return false;
	}

	if (vCfg.NdefAddr >= vCfg.MemSize)
	{
		return false;
	}

	// NdefMaxLen of 0 selects the remainder of the tag memory.
	uint32_t avail = vCfg.MemSize - vCfg.NdefAddr;

	if (vCfg.NdefMaxLen == 0)
	{
		vCfg.NdefMaxLen = avail;
	}

	// Subtraction form, the additive form can wrap on large offsets.
	if (vCfg.NdefMaxLen > avail)
	{
		return false;
	}

	Valid(true);

	return true;
}
''',
	'''bool RFTag::Init(const RFTagCfg_t &Cfg, DeviceIntrf * const pIntrf)
{
	Valid(false);
	Interface(nullptr);
	vpProto = nullptr;
	memset(&vCfg, 0, sizeof(vCfg));

	RFTagCfg_t cfg = Cfg;

	// The base is a local memory tag. It requires the image and a NDEF area
	// inside it. A bus tag subclass validates its own memory access.
	if (cfg.pMem == nullptr || cfg.MemSize == 0 || cfg.NdefAddr >= cfg.MemSize)
	{
		return false;
	}

	uint32_t avail = cfg.MemSize - cfg.NdefAddr;

	if (cfg.NdefMaxLen == 0)
	{
		cfg.NdefMaxLen = avail;
	}

	if (cfg.NdefMaxLen > avail)
	{
		return false;
	}

	vCfg = cfg;
	Interface(pIntrf);
	Valid(true);

	return true;
}
''',
)

replace_one(
	"src/rftag/rftag.cpp",
	'''void RFTag::Reset()
{
	if (vpProto)
	{
		vpProto->Init(this);
	}
}
''',
	'''void RFTag::Reset()
{
	if (vpProto != nullptr && vpProto->Init(this) == false)
	{
		vpProto = nullptr;
		EvtHandler(RFTAG_EVT_ERROR, 0, 0);
	}
}
''',
)

replace_one(
	"src/rftag/rftag.cpp",
	'''int RFTag::ProcessFrame(const uint8_t *pRx, int RxLen, uint8_t *pTx, int TxCap)
{
	if (vpProto == nullptr)
	{
		return 0;
	}

	return vpProto->OnFrame(pRx, RxLen, pTx, TxCap);
}
''',
	'''int RFTag::ProcessFrame(const uint8_t *pRx, int RxLen, uint8_t *pTx, int TxCap)
{
	if (Valid() == false || vpProto == nullptr || pRx == nullptr || RxLen <= 0 ||
		TxCap < 0 || (TxCap > 0 && pTx == nullptr))
	{
		return 0;
	}

	return vpProto->OnFrame(pRx, RxLen, pTx, TxCap);
}
''',
)

replace_one(
	"src/rftag/rftag.cpp",
	'''int RFTag::MemRead(uint32_t Addr, uint8_t *pBuff, int Len)
{
	if (pBuff == nullptr || Len <= 0 || Addr >= vCfg.MemSize)
''',
	'''int RFTag::MemRead(uint32_t Addr, uint8_t *pBuff, int Len)
{
	if (Valid() == false || vCfg.pMem == nullptr || pBuff == nullptr ||
		Len <= 0 || Addr >= vCfg.MemSize)
''',
)

replace_one(
	"src/rftag/rftag.cpp",
	'''int RFTag::MemWrite(uint32_t Addr, const uint8_t *pData, int Len)
{
	if (pData == nullptr || Len <= 0 || Addr >= vCfg.MemSize)
''',
	'''int RFTag::MemWrite(uint32_t Addr, const uint8_t *pData, int Len)
{
	if (Valid() == false || vCfg.pMem == nullptr || pData == nullptr ||
		Len <= 0 || Addr >= vCfg.MemSize)
''',
)

replace_one(
	"src/rftag/rftag.cpp",
	'''bool RFTag::SetNdefNLen16(const uint8_t *pNdef, uint16_t Len)
{
	uint8_t hdr[2];

	// Length prefix plus payload must fit the NDEF area
	if ((uint32_t)Len + sizeof(hdr) > vCfg.NdefMaxLen)
	{
		return false;
	}

	hdr[0] = (uint8_t)(Len >> 8);
	hdr[1] = (uint8_t)Len;

	if (MemWrite(vCfg.NdefAddr, hdr, sizeof(hdr)) != (int)sizeof(hdr))
	{
		return false;
	}

	return MemWrite(vCfg.NdefAddr + sizeof(hdr), pNdef, Len) == Len;
}
''',
	'''bool RFTag::SetNdefNLen16(const uint8_t *pNdef, uint16_t Len)
{
	uint8_t hdr[2] = { 0, 0 };

	if ((uint32_t)Len + sizeof(hdr) > vCfg.NdefMaxLen)
	{
		return false;
	}

	// Keep the message empty until the complete payload is stored.
	if (MemWrite(vCfg.NdefAddr, hdr, sizeof(hdr)) != (int)sizeof(hdr))
	{
		return false;
	}

	if (Len > 0 && MemWrite(vCfg.NdefAddr + sizeof(hdr), pNdef, Len) != Len)
	{
		return false;
	}

	hdr[0] = (uint8_t)(Len >> 8);
	hdr[1] = (uint8_t)Len;

	return MemWrite(vCfg.NdefAddr, hdr, sizeof(hdr)) == (int)sizeof(hdr);
}
''',
)

replace_one(
	"src/rftag/rftag.cpp",
	'''bool RFTag::SetNdefTlv(const uint8_t *pNdef, uint16_t Len)
{
	uint8_t hdr[4];
	uint32_t addr = vCfg.NdefAddr;
	int hlen = 0;

	// Short form framing is 0x03, length, payload, 0xFE terminator
	if ((uint32_t)(Len + 3) > vCfg.NdefMaxLen)
	{
		return false;
	}

	hdr[hlen++] = 0x03;

	if (Len < 0xFF)
	{
		hdr[hlen++] = (uint8_t)Len;
	}
	else
	{
		// Long form framing adds 0xFF and a 2 byte length ahead of payload
		if ((uint32_t)(Len + 5) > vCfg.NdefMaxLen)
		{
			return false;
		}
		hdr[hlen++] = 0xFF;
		hdr[hlen++] = (uint8_t)(Len >> 8);
		hdr[hlen++] = (uint8_t)Len;
	}

	if (MemWrite(addr, hdr, hlen) != hlen)
	{
		return false;
	}

	addr += hlen;

	if (MemWrite(addr, pNdef, Len) != Len)
	{
		return false;
	}

	addr += Len;

	uint8_t term = 0xFE;

	return MemWrite(addr, &term, 1) == 1;
}
''',
	'''bool RFTag::SetNdefTlv(const uint8_t *pNdef, uint16_t Len)
{
	uint8_t hdr[4];
	uint8_t pending[4];
	uint32_t addr = vCfg.NdefAddr;
	int hlen = 0;

	// Short form framing is 0x03, length, payload, 0xFE terminator
	if ((uint32_t)(Len + 3) > vCfg.NdefMaxLen)
	{
		return false;
	}

	hdr[hlen++] = 0x03;

	if (Len < 0xFF)
	{
		hdr[hlen++] = (uint8_t)Len;
	}
	else
	{
		// Long form framing adds 0xFF and a 2 byte length ahead of payload
		if ((uint32_t)(Len + 5) > vCfg.NdefMaxLen)
		{
			return false;
		}
		hdr[hlen++] = 0xFF;
		hdr[hlen++] = (uint8_t)(Len >> 8);
		hdr[hlen++] = (uint8_t)Len;
	}

	memcpy(pending, hdr, hlen);
	if (hlen == 2)
	{
		pending[1] = 0;
	}
	else
	{
		pending[2] = 0;
		pending[3] = 0;
	}

	if (MemWrite(addr, pending, hlen) != hlen)
	{
		return false;
	}

	addr += hlen;

	if (Len > 0 && MemWrite(addr, pNdef, Len) != Len)
	{
		return false;
	}

	addr += Len;

	uint8_t term = 0xFE;

	if (MemWrite(addr, &term, 1) != 1)
	{
		return false;
	}

	return MemWrite(vCfg.NdefAddr, hdr, hlen) == hlen;
}
''',
)

replace_one(
	"src/rftag/rftag.cpp",
	'''bool RFTag::SetNdef(const uint8_t *pNdef, uint16_t Len)
{
	if (Len > 0 && pNdef == nullptr)
''',
	'''bool RFTag::SetNdef(const uint8_t *pNdef, uint16_t Len)
{
	if (Valid() == false || (Len > 0 && pNdef == nullptr))
''',
)

replace_one(
	"src/rftag/rftag.cpp",
	'''int RFTag::GetNdef(uint8_t *pNdef, uint16_t Len)
{
	if (pNdef == nullptr || Len == 0)
''',
	'''int RFTag::GetNdef(uint8_t *pNdef, uint16_t Len)
{
	if (Valid() == false || pNdef == nullptr || Len == 0)
''',
)

for path in ["include/rftag/rftag.h", "src/rftag/rftag.cpp"]:
	text = Path(path).read_text(encoding="ascii")
	if text.count("{") != text.count("}"):
		raise RuntimeError(f"{path}: brace imbalance")
