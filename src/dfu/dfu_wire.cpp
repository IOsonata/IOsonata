/**-------------------------------------------------------------------------
@file	dfu_wire.cpp

@brief	DFU wire protocol over any byte stream. See dfu_wire.h and
		docs/architecture/dfu_wire.md.

@author	Hoang Nguyen Hoan
@date	Sep. 21, 2026

@license

MIT License

Copyright (c) 2026, I-SYST inc., all rights reserved

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

----------------------------------------------------------------------------*/
#include <string.h>

#include "crc.h"
#include "dfu/dfu_wire.h"

/** @addtogroup DFU
  * @{
  */

static inline void DfuWirePut16(uint8_t *p, uint32_t v)
{
	p[0] = (uint8_t)v;
	p[1] = (uint8_t)(v >> 8);
}

static inline void DfuWirePut32(uint8_t *p, uint32_t v)
{
	p[0] = (uint8_t)v;
	p[1] = (uint8_t)(v >> 8);
	p[2] = (uint8_t)(v >> 16);
	p[3] = (uint8_t)(v >> 24);
}

static inline uint32_t DfuWireGet16(const uint8_t *p)
{
	return (uint32_t)p[0] | ((uint32_t)p[1] << 8);
}

static inline uint32_t DfuWireGet32(const uint8_t *p)
{
	return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) |
		   ((uint32_t)p[3] << 24);
}

int DfuWireStatus(int Res)
{
	switch (Res)
	{
		case DFU_IMG_OK:			return DFU_WIRE_OK;
		case DFU_MGR_ERR_STATE:		return DFU_WIRE_ERR_STATE;
		case DFU_MGR_ERR_LEN:		return DFU_WIRE_ERR_LEN;
		case DFU_IMG_ERR_MAGIC:
		case DFU_IMG_ERR_HDR:		return DFU_WIRE_ERR_HDR;
		case DFU_IMG_ERR_SIZE:		return DFU_WIRE_ERR_SIZE;
		case DFU_IMG_ERR_TLV:
		case DFU_IMG_ERR_NOHASH:	return DFU_WIRE_ERR_TLV;
		case DFU_IMG_ERR_KEY:		return DFU_WIRE_ERR_KEY;
		case DFU_IMG_ERR_SIG:
		case DFU_IMG_ERR_CRYPTO:	return DFU_WIRE_ERR_SIG;
		case DFU_MGR_ERR_VERSION:	return DFU_WIRE_ERR_VERSION;
		case DFU_MGR_ERR_OFFSET:	return DFU_WIRE_ERR_OFFSET;
		case DFU_IMG_ERR_HASH:		return DFU_WIRE_ERR_HASH;
		case DFU_IMG_ERR_VECTOR:	return DFU_WIRE_ERR_ENTRY;
		default:					return DFU_WIRE_ERR_FLASH;
	}
}

DfuWire::DfuWire()
{
	memset(&vCfg, 0, sizeof(vCfg));
	vbInit = false;
	vRxLen = 0;
	vbSkip = false;
	vDropped = 0;
	vSeq = 0;
	vbDone = false;
	vManLen = 0;
	vManTotal = 0;
}

bool DfuWire::Init(const DfuWireCfg_t &Cfg)
{
	vbInit = false;

	if (Cfg.pMgr == nullptr || Cfg.pIntrf == nullptr || Cfg.pRxBuf == nullptr ||
		Cfg.MaxBody < 16 || (Cfg.MaxBody & (Cfg.MaxBody - 1)) != 0 ||
		Cfg.RxBufSize < DFU_WIRE_RXBUF_SIZE(Cfg.MaxBody))
	{
		return false;
	}

	vCfg = Cfg;
	vDropped = 0;
	vbDone = false;
	vManLen = 0;
	vManTotal = 0;
	vbInit = SetIntrf(Cfg.pIntrf);

	return vbInit;
}

bool DfuWire::SetIntrf(DeviceIntrf *pIntrf)
{
	if (pIntrf == nullptr || vSlip.Init(pIntrf, false) == false)
	{
		return false;
	}

	vCfg.pIntrf = pIntrf;
	vRxLen = 0;
	vbSkip = false;
	vCfg.pRxBuf[0] = 0;

	return true;
}

// One response frame: Op | 0x80, the request Seq, the body, the CRC. Slip
// escapes it and ends it; an END first closes whatever noise the host had
// on the line.
void DfuWire::Rsp(uint8_t Op, const uint8_t *pBody, uint32_t Len)
{
	uint32_t n = 2 + Len;

	vTx[0] = (uint8_t)(Op | DFU_WIRE_OP_RSP);
	vTx[1] = vSeq;
	memcpy(&vTx[2], pBody, Len);
	DfuWirePut32(&vTx[n], crc32_ieee_cont(0, vTx, (int)n));
	n += 4;

	uint8_t end = DFU_WIRE_END;

	(void)vCfg.pIntrf->Tx(0, &end, 1);
	(void)vSlip.Tx(0, vTx, (int)n);
}

void DfuWire::RspStatus(uint8_t Op, int Status)
{
	uint8_t b = (uint8_t)Status;

	Rsp(Op, &b, 1);
}

// Status, then where the upload is: offset and CRC-32 of what was taken.
void DfuWire::RspOffset(uint8_t Op, int Status)
{
	uint8_t b[9];

	b[0] = (uint8_t)Status;
	DfuWirePut32(&b[1], vCfg.pMgr->Offset());
	DfuWirePut32(&b[5], vCfg.pMgr->Crc());
	Rsp(Op, b, sizeof(b));
}

void DfuWire::Serve(uint8_t Op, const uint8_t *p, uint32_t Len)
{
	DfuMgr *m = vCfg.pMgr;

	switch (Op)
	{
		case DFU_WIRE_OP_INFO:
		{
			uint8_t b[DFU_WIRE_INFO_LEN];
			const DfuImgVer_t &v = m->CurVer();

			if (Len != 0)
			{
				RspStatus(Op, DFU_WIRE_ERR_LEN);
				break;
			}
			memset(b, 0, sizeof(b));
			b[0] = DFU_WIRE_OK;
			b[1] = DFU_WIRE_PROTO_VER;
			b[2] = m->Active() ? DFU_WIRE_STATE_RECV :
				   vbDone ? DFU_WIRE_STATE_DONE : DFU_WIRE_STATE_IDLE;
			b[3] = m->AllowDowngrade() ? DFU_WIRE_F_DOWNGRADE : 0;
			DfuWirePut16(&b[4], vCfg.MaxBody);
			DfuWirePut16(&b[6], DfuTgtWriteUnit());
			DfuWirePut16(&b[8], vCfg.EraseMaxMs);
			DfuWirePut16(&b[10], DFU_MGR_MANIFEST_MAX);
			DfuWirePut32(&b[12], m->MaxLen());
			DfuWirePut32(&b[16], m->Offset());
			b[20] = v.Major;
			b[21] = v.Minor;
			DfuWirePut16(&b[22], v.Rev);
			DfuWirePut32(&b[24], v.Build);
			if (vCfg.pDevId != nullptr)
			{
				memcpy(&b[28], vCfg.pDevId, 8);
			}
			DfuWirePut32(&b[36], vCfg.BootVer);
			Rsp(Op, b, sizeof(b));
			break;
		}

		case DFU_WIRE_OP_BEGIN:
		{
			// Total, offset, then manifest bytes, gathered in the manager's
			// record body until the total is in.
			if (Len < DFU_WIRE_BEGIN_HDR + 1)
			{
				RspStatus(Op, DFU_WIRE_ERR_LEN);
				break;
			}

			uint32_t total = DfuWireGet16(p);
			uint32_t off = DfuWireGet16(p + 2);
			uint32_t l = Len - DFU_WIRE_BEGIN_HDR;

			if (off == 0)
			{
				// A new upload: whatever was under way ends here.
				m->Abort();
				vbDone = false;
				vManLen = 0;
				vManTotal = (uint16_t)total;
			}
			if (vManTotal == 0 || total != vManTotal ||
				total > DFU_MGR_MANIFEST_MAX || off > vManLen ||
				l > total - off)
			{
				vManTotal = 0;
				RspStatus(Op, DFU_WIRE_ERR_LEN);
				break;
			}

			// A resend of a piece already taken copies the same bytes.
			memcpy(DfuMgr::ManifestBuf() + off, p + DFU_WIRE_BEGIN_HDR, l);
			if (off + l > vManLen)
			{
				vManLen = (uint16_t)(off + l);
			}
			if (vManLen < total)
			{
				// More to come.
				RspOffset(Op, DFU_WIRE_OK);
				break;
			}

			vManTotal = 0;
			RspOffset(Op, DfuWireStatus(m->BeginManifest(DfuMgr::ManifestBuf(),
														 total)));
			break;
		}

		case DFU_WIRE_OP_WRITE:
		{
			if (Len < DFU_WIRE_WRITE_HDR + 1 ||
				Len > (uint32_t)DFU_WIRE_WRITE_HDR + vCfg.MaxBody)
			{
				RspOffset(Op, DFU_WIRE_ERR_LEN);
				break;
			}
			int res = m->Write(DfuWireGet32(p), p + DFU_WIRE_WRITE_HDR,
							   Len - DFU_WIRE_WRITE_HDR);
			RspOffset(Op, DfuWireStatus(res));
			break;
		}

		case DFU_WIRE_OP_FINISH:
		{
			if (Len != 0)
			{
				RspStatus(Op, DFU_WIRE_ERR_LEN);
				break;
			}
			if (vbDone)
			{
				// The response was lost; it is done already.
				RspStatus(Op, DFU_WIRE_OK);
				break;
			}

			int res = m->Finish(nullptr);
			if (res == DFU_IMG_OK && m->Commit() == false)
			{
				res = DFU_IMG_ERR_READ;
			}
			vbDone = res == DFU_IMG_OK;
			RspStatus(Op, DfuWireStatus(res));
			break;
		}

		case DFU_WIRE_OP_RESET:
		case DFU_WIRE_OP_ENTER:
		{
			bool rec = Op == DFU_WIRE_OP_ENTER || (Len == 1 && p[0] == 0);

			if (Op == DFU_WIRE_OP_RESET && Len != 1)
			{
				RspStatus(Op, DFU_WIRE_ERR_LEN);
				break;
			}
			if (Op == DFU_WIRE_OP_ENTER && Len != 0)
			{
				RspStatus(Op, DFU_WIRE_ERR_LEN);
				break;
			}
			RspStatus(Op, DFU_WIRE_OK);
			if (vCfg.Reset != nullptr)
			{
				vCfg.Reset(rec);
			}
			else
			{
				m->Reset(rec);
			}
			break;
		}

		case DFU_WIRE_OP_ABORT:
			m->Abort();
			vbDone = false;
			vManTotal = 0;
			RspStatus(Op, DFU_WIRE_OK);
			break;
	}
}

// A complete decoded frame in pRxBuf: check it, serve it.
bool DfuWire::Frame(void)
{
	uint8_t *f = vCfg.pRxBuf;
	uint32_t n = vRxLen;

	vRxLen = 0;
	if (n == 0)
	{
		// Back to back ENDs.
		return false;
	}
	if (n < DFU_WIRE_FRAME_OVH ||
		crc32_ieee_cont(0, f, (int)(n - 4)) != DfuWireGet32(f + n - 4) ||
		f[0] < DFU_WIRE_OP_INFO || f[0] > DFU_WIRE_OP_ENTER)
	{
		vDropped++;
		return false;
	}

	vSeq = f[1];
	Serve(f[0], f + 2, n - DFU_WIRE_FRAME_OVH);

	return true;
}

bool DfuWire::Poll(void)
{
	bool served = false;

	if (vbInit == false)
	{
		return false;
	}

	// Slip non blocking receive: it decodes into the buffer given and, when
	// it returns short of the length, leaves its state in the byte after the
	// last one counted: 0xFF nothing more, the escape code an escape still
	// to be paired, the END code a frame ended. Reading goes on from there.
	// A frame starts at pRxBuf[0], set to 0 first so that a byte left from
	// the last frame is not taken for an escape. A frame too long is read
	// over at pRxBuf[0] up to its end, the state moved down each time.
	for (;;)
	{
		uint8_t *p = vbSkip ? vCfg.pRxBuf : vCfg.pRxBuf + vRxLen;
		int room = vbSkip ? (int)vCfg.RxBufSize :
				   (int)(vCfg.RxBufSize - vRxLen);
		int n = vSlip.Rx(0, p, room);

		if (n < 0)
		{
			n = 0;
		}
		if (vbSkip)
		{
			p[0] = n < room ? p[n] : 0;
		}
		else
		{
			vRxLen += (uint32_t)n;
		}

		if (vSlip.RxCompleted())
		{
			// The end of a frame, of a dropped one, or of line noise.
			if (vbSkip)
			{
				vbSkip = false;
				vRxLen = 0;
			}
			else if (Frame())
			{
				served = true;
			}
			vCfg.pRxBuf[0] = 0;
			continue;
		}

		if (vbSkip == false && vRxLen >= vCfg.RxBufSize)
		{
			// Longer than the longest frame: dropped up to its end.
			vDropped++;
			vbSkip = true;
			vRxLen = 0;
			vCfg.pRxBuf[0] = 0;
			continue;
		}

		if (n == 0)
		{
			break;
		}
	}

	return served;
}

/** @} */
