from pathlib import Path
import xml.etree.ElementTree as ET

cp = Path('ARM/Nordic/nRF54/nRF54L15/lib/Eclipse/.cproject')
text = cp.read_text()
needle = 'src/bluetooth/bt_attrsp.cpp|src/bluetooth/bt_attreq.cpp|src/bluetooth/bt_hci_host.cpp'
replacement = 'src/bluetooth/bt_attrsp.cpp|src/bluetooth/bt_attreq.cpp|src/bluetooth/bt_att.cpp|src/bluetooth/bt_l2cap.cpp|src/bluetooth/bt_hci_host.cpp'
count = text.count(needle)
if count != 2:
    raise SystemExit(f'expected 2 source exclusion anchors, found {count}')
text = text.replace(needle, replacement)
cp.write_text(text)
ET.parse(cp)

att = Path('src/bluetooth/bt_att.cpp')
text = att.read_text()
old = '''uint32_t BtAttProcessReq(uint16_t ConnHdl, BtAttReqRsp_t * const pReqAtt, int ReqLen, BtAttReqRsp_t * const pRspAtt)
{
\tuint32_t retval = 0;

\tDEBUG_PRINTF("ATT OpCode %x, L2Cap len %d\\n", pReqAtt->OpCode, ReqLen);
'''
new = '''uint32_t BtAttProcessReq(uint16_t ConnHdl, BtAttReqRsp_t * const pReqAtt, int ReqLen, BtAttReqRsp_t * const pRspAtt)
{
\tif (pReqAtt == nullptr || pRspAtt == nullptr || ReqLen < 1)
\t{
\t\treturn 0;
\t}

\tuint32_t retval = 0;

\tDEBUG_PRINTF("ATT OpCode %x, L2Cap len %d\\n", pReqAtt->OpCode, ReqLen);
'''
if old not in text:
    raise SystemExit('BtAttProcessReq anchor not found')
att.write_text(text.replace(old, new, 1))
