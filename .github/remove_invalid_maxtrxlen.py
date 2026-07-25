from pathlib import Path
import subprocess

ROOT = Path('.')
HEADER = ROOT / 'include/device_intrf.h'
SPI = ROOT / 'ARM/Nordic/src/spi_nrfx.cpp'
WORKFLOW = ROOT / '.github/workflows/remove-invalid-maxtrxlen.yml'
SCRIPT = ROOT / '.github/remove_invalid_maxtrxlen.py'
TOKENS = (b'MaxTrxLen', b'GetMaxTransferLen', b'DeviceIntrfGetMaxTransferLen')


def replace_once(text: str, old: str, new: str, name: str) -> str:
    count = text.count(old)
    if count != 1:
        raise RuntimeError(f'{name}: expected one match, found {count}')
    return text.replace(old, new, 1)


# Remove the unauthorized DeviceIntrf member and both accessors.
lines = HEADER.read_text().splitlines(keepends=True)
out = []
skip_c_getter = False
counts = {'member': 0, 'c_getter': 0, 'cpp_getter': 0}

for line in lines:
    if skip_c_getter:
        if line.strip() == '}':
            skip_c_getter = False
        continue
    if 'size_t GetMaxTransferLen(' in line:
        counts['cpp_getter'] += 1
        continue
    if line.startswith('static inline size_t DeviceIntrfGetMaxTransferLen('):
        counts['c_getter'] += 1
        skip_c_getter = True
        continue
    if 'MaxTrxLen;' in line:
        counts['member'] += 1
        continue
    out.append(line)

if counts != {'member': 1, 'c_getter': 1, 'cpp_getter': 1}:
    raise RuntimeError(f'unexpected DeviceIntrf removal counts: {counts}')

header_text = ''.join(out)
if 'size_t' not in header_text:
    header_text = replace_once(
        header_text, '#include <stddef.h>\n', '', 'stddef include')
HEADER.write_text(header_text)

# Remove all later assignments, uses and documentation that depended on it.
grep = subprocess.run(
    ['git', 'grep', '-Il', '-e', 'MaxTrxLen', '-e', 'GetMaxTransferLen',
     '-e', 'DeviceIntrfGetMaxTransferLen', '--', '.'],
    check=False, text=True, capture_output=True)
for name in grep.stdout.splitlines():
    path = Path(name)
    if path in (HEADER, WORKFLOW, SCRIPT):
        continue
    raw_lines = path.read_bytes().splitlines(keepends=True)
    path.write_bytes(b''.join(
        line for line in raw_lines
        if not any(token in line for token in TOKENS)))

# Restore the Nordic SPI DMA chunking changed in the same bad commit.
spi = SPI.read_text()
spi = replace_once(spi, '''\tdev->pDmaReg->RXD.PTR = (uint32_t)pBuff;
\tif (BuffLen >= NRFX_SPI_DMA_MAXCNT)
\t{
\t\tdev->pDmaReg->RXD.LIST = SPIM_RXD_LIST_LIST_ArrayList << SPIM_RXD_LIST_LIST_Pos;
\t\tdev->pDmaReg->RXD.MAXCNT = NRFX_SPI_DMA_MAXCNT;
\t}
''', '''\tdev->pDmaReg->RXD.PTR = (uint32_t)pBuff;
\tdev->pDmaReg->RXD.LIST = SPIM_RXD_LIST_LIST_ArrayList << SPIM_RXD_LIST_LIST_Pos;
''', 'SPI RX list setup')
spi = replace_once(spi, '''\t\tif (BuffLen < NRFX_SPI_DMA_MAXCNT)
\t\t{
\t\t\tdev->pDmaReg->RXD.MAXCNT = BuffLen;
\t\t}
''', '''\t\tint l = min(BuffLen, NRFX_SPI_DMA_MAXCNT);

\t\tdev->pDmaReg->RXD.MAXCNT = l;
''', 'SPI RX chunk size')
spi = replace_once(spi, '''\t\tsize_t l = dev->pDmaReg->RXD.AMOUNT;
\t\tBuffLen -= l;
\t\tcnt += l;
''', '''\t\tl = dev->pDmaReg->RXD.AMOUNT;
\t\tBuffLen -= l;
\t\tpBuff += l;
\t\tcnt += l;
''', 'SPI RX advance')
spi = replace_once(spi, '''\tdev->pDmaReg->TXD.PTR = (uint32_t)pData;

\tif (DataLen >= NRFX_SPI_DMA_MAXCNT)
\t{
\t\tdev->pDmaReg->TXD.LIST = SPIM_TXD_LIST_LIST_ArrayList << SPIM_TXD_LIST_LIST_Pos;
\t\tdev->pDmaReg->TXD.MAXCNT = NRFX_SPI_DMA_MAXCNT;
\t}
''', '''\tdev->pDmaReg->TXD.PTR = (uint32_t)pData;
\tdev->pDmaReg->TXD.LIST = SPIM_TXD_LIST_LIST_ArrayList << SPIM_TXD_LIST_LIST_Pos;
''', 'SPI TX list setup')
spi = replace_once(spi, '''\t\tif (DataLen < NRFX_SPI_DMA_MAXCNT)
\t\t{
\t\t\tdev->pDmaReg->TXD.MAXCNT = DataLen;
\t\t}

''', '''\t\tint l = min(DataLen, NRFX_SPI_DMA_MAXCNT);
\t\tdev->pDmaReg->TXD.MAXCNT = l;
''', 'SPI TX chunk size')
spi = replace_once(spi, '''\t\tsize_t l = dev->pDmaReg->TXD.AMOUNT;
\t\tDataLen -= l;
\t\tcnt += l;

''', '''\t\tl = dev->pDmaReg->TXD.AMOUNT;
\t\tDataLen -= l;
\t\tpData += l;
\t\tcnt += l;
''', 'SPI TX advance')
SPI.write_text(spi)

WORKFLOW.unlink()
SCRIPT.unlink()
