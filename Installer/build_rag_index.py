#!/usr/bin/env python3
"""
IOsonata RAG Index Builder v6 - Unified Schema

Indexes IOsonata source code into a SQLite database for RAG retrieval.
Uses unified schema shared with build_external_index.py and build_knowledge_db.py.

Usage:
  python3 build_rag_index.py                    # Build index
  python3 build_rag_index.py --update-embeddings --provider openai --api-key $KEY
"""

from __future__ import annotations

import argparse
import functools
import glob
import hashlib
import os
import re
import sqlite3
import struct
import subprocess
import time
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Dict, Iterator, List, Optional, Tuple

# Import unified schema
from rag_schema import (
    SCHEMA_VERSION, COMPRESS_LEVEL,
    KIND_FUNCTION, KIND_TYPE, KIND_EXAMPLE, KIND_ASSEMBLY, KIND_HEADER,
    PERIPH_NONE, PERIPH_BLE, PERIPH_UART, PERIPH_SPI, PERIPH_I2C, PERIPH_USB,
    PERIPH_TIMER, PERIPH_PWM, PERIPH_ADC, PERIPH_GPIO, PERIPH_FLASH,
    PERIPH_I2S, PERIPH_PDM, PERIPH_QSPI, PERIPH_ESB, PERIPH_SENSOR,
    PERIPH_RTC, PERIPH_WDT, PERIPH_DMA, PERIPH_CRYPTO, PERIPH_MAP,
    PROVIDER_MAP, DB_TYPE_CODE,
    db_connect, ensure_schema, fts_rebuild, set_standard_meta,
    compress, decompress, sha256, StringCache,
    build_line_index, idx_to_line,
)

print = functools.partial(print, flush=True)

# =============================================================================
# CONFIG
# =============================================================================

DEFAULT_IGNORE_DIRS = {
    ".git", ".github", ".iosonata", ".metadata", ".settings", ".vscode", ".idea",
    "build", "out", "dist", "Debug", "Release", "OSX", "linux", "win32", "OSC",
    "node_modules", "__pycache__", ".pytest_cache",
}
IGNORE_DIR_PREFIXES = ("Debug", "Release", "cmake-build-")
SOURCE_SUFFIXES = {".h", ".hpp", ".hh", ".c", ".cc", ".cpp", ".cxx", ".inc", ".inl"}
EXAMPLE_DIR_HINTS = frozenset(("example", "examples", "sample", "samples", "demo", "demos", "test", "tests", "exemples"))

# =============================================================================
# BLE IMPLEMENTATION TYPES
# =============================================================================

BLE_IMPL_NONE = 0
BLE_IMPL_SOFTDEVICE = 1
BLE_IMPL_SDC = 2

# =============================================================================
# DEVICE CATEGORIES
# =============================================================================

DEVCAT_NONE = 0
DEVCAT_SENSOR = 1
DEVCAT_DISPLAY = 2
DEVCAT_MISCDEV = 3
DEVCAT_PMIC = 4
DEVCAT_IMU = 5
DEVCAT_AUDIO = 6
DEVCAT_STORAGE = 7
DEVCAT_CONVERTER = 8

DEVCAT_MAP = {
    "sensors": DEVCAT_SENSOR, "display": DEVCAT_DISPLAY, "miscdev": DEVCAT_MISCDEV,
    "pwrmgnt": DEVCAT_PMIC, "imu": DEVCAT_IMU, "audio": DEVCAT_AUDIO,
    "storage": DEVCAT_STORAGE, "converters": DEVCAT_CONVERTER,
}

# =============================================================================
# PERIPHERAL DETECTION PATTERNS
# =============================================================================

IMPL_PATTERNS = [
    # Nordic BLE
    (r'bt_app_nrf52\.cpp$', PERIPH_BLE, BLE_IMPL_SOFTDEVICE),
    (r'bt_gap_nrf52\.cpp$', PERIPH_BLE, BLE_IMPL_SOFTDEVICE),
    (r'bt_gatt_nrf52\.cpp$', PERIPH_BLE, BLE_IMPL_SOFTDEVICE),
    (r'bt_app_sdc\.cpp$', PERIPH_BLE, BLE_IMPL_SDC),
    (r'bt_gap_sdc\.cpp$', PERIPH_BLE, BLE_IMPL_SDC),
    (r'bt_gatt_sdc\.cpp$', PERIPH_BLE, BLE_IMPL_SDC),
    (r'ble_dev\.cpp$', PERIPH_BLE, BLE_IMPL_NONE),
    # Nordic peripherals
    (r'uart_nrf.*\.cpp$', PERIPH_UART, 0),
    (r'spi_nrf.*\.cpp$', PERIPH_SPI, 0),
    (r'i2c_nrf.*\.cpp$', PERIPH_I2C, 0),
    (r'usbd?_nrf.*\.cpp$', PERIPH_USB, 0),
    (r'timer_.*nrf.*\.cpp$', PERIPH_TIMER, 0),
    (r'pwm_nrf.*\.cpp$', PERIPH_PWM, 0),
    (r'adc_nrf.*\.cpp$', PERIPH_ADC, 0),
    (r'i2s_nrf.*\.cpp$', PERIPH_I2S, 0),
    (r'pdm_nrf.*\.cpp$', PERIPH_PDM, 0),
    (r'qspi_nrf.*\.cpp$', PERIPH_QSPI, 0),
    (r'esb_.*\.cpp$', PERIPH_ESB, 0),
    # STM32
    (r'uart_stm32.*\.cpp$', PERIPH_UART, 0),
    (r'spi_stm32.*\.cpp$', PERIPH_SPI, 0),
    (r'i2c_stm32.*\.cpp$', PERIPH_I2C, 0),
    (r'usb_stm32.*\.cpp$', PERIPH_USB, 0),
    (r'timer_stm32.*\.cpp$', PERIPH_TIMER, 0),
    (r'pwm_stm32.*\.cpp$', PERIPH_PWM, 0),
    (r'adc_stm32.*\.cpp$', PERIPH_ADC, 0),
    # SAM
    (r'uart_sam.*\.cpp$', PERIPH_UART, 0),
    (r'spi_sam.*\.cpp$', PERIPH_SPI, 0),
    (r'i2c_sam.*\.cpp$', PERIPH_I2C, 0),
    # LPC
    (r'uart_lpc.*\.cpp$', PERIPH_UART, 0),
    (r'spi_lpc.*\.cpp$', PERIPH_SPI, 0),
    (r'i2c_lpc.*\.cpp$', PERIPH_I2C, 0),
    # RISC-V
    (r'uart_esp32c.*\.cpp$', PERIPH_UART, 0),
    (r'spi_esp32c.*\.cpp$', PERIPH_SPI, 0),
    (r'i2c_esp32c.*\.cpp$', PERIPH_I2C, 0),
    (r'uart_ch32v.*\.cpp$', PERIPH_UART, 0),
    (r'spi_ch32v.*\.cpp$', PERIPH_SPI, 0),
    (r'i2c_ch32v.*\.cpp$', PERIPH_I2C, 0),
]


def detect_peripheral_from_impl(filename: str) -> Tuple[int, int]:
    """Returns (peripheral_id, ble_impl_type)"""
    for pattern, periph, impl in IMPL_PATTERNS:
        if re.search(pattern, filename, re.I):
            return (periph, impl)
    return (PERIPH_NONE, 0)


def detect_peripheral_from_path(path: str) -> int:
    """Detect peripheral ID from file path."""
    p = path.lower()
    if 'bluetooth' in p or '/bt_' in p or 'ble' in p:
        return PERIPH_BLE
    for keyword, periph_id in PERIPH_MAP.items():
        if keyword in p:
            return periph_id
    return PERIPH_NONE


# =============================================================================
# CODE PARSING
# =============================================================================

def _safe_read(path: Path, max_bytes: int) -> str:
    try:
        raw = path.read_bytes()[:max_bytes]
        for enc in ("utf-8", "latin-1"):
            try:
                return raw.decode(enc)
            except UnicodeDecodeError:
                continue
        return raw.decode("utf-8", errors="replace")
    except:
        return ""


def _mask_comments(src: str) -> str:
    out = list(src)
    n = len(src)
    i = 0
    def repl(s, e):
        for k in range(s, min(e, n)):
            if out[k] not in "\n\r":
                out[k] = " "
    while i < n:
        c = src[i]
        if c == "/" and i + 1 < n:
            if src[i+1] == "/":
                j = i
                while i < n and src[i] != "\n":
                    i += 1
                repl(j, i)
                continue
            if src[i+1] == "*":
                j = i
                i += 2
                while i + 1 < n and not (src[i] == "*" and src[i+1] == "/"):
                    i += 1
                i = min(n, i + 2)
                repl(j, i)
                continue
        if c in "\"'":
            q, j = c, i
            i += 1
            while i < n:
                if out[i] == "\\":
                    i += 2
                    continue
                if out[i] == q:
                    i += 1
                    break
                i += 1
            repl(j, i)
            continue
        i += 1
    return "".join(out)


def _find_brace(masked: str, start: int) -> Optional[int]:
    depth = 0
    for i in range(start, len(masked)):
        if masked[i] == "{":
            depth += 1
        elif masked[i] == "}":
            depth -= 1
            if depth == 0:
                return i
    return None


_TYPE_RE = re.compile(r"\b(class|struct|enum)\s+([A-Za-z_]\w*)[^;{]*\{", re.M)
_FUNC_RE = re.compile(
    r"^[ \t]*(?:static\s+|inline\s+|extern\s+|virtual\s+|__STATIC_INLINE\s+)*"
    r"([A-Za-z_][\w:<>\s\*&]+?)\s+([A-Za-z_]\w*)\s*\(([^;\)]{0,400})\)\s*([{;])",
    re.M
)


def iter_types(src: str):
    masked = _mask_comments(src)
    for m in _TYPE_RE.finditer(masked):
        kind, name = m.group(1), m.group(2)
        brace = masked.find("{", m.start())
        if brace < 0:
            continue
        end = _find_brace(masked, brace)
        if end is None:
            continue
        j = end + 1
        while j < len(masked) and masked[j].isspace():
            j += 1
        if j < len(masked) and masked[j] == ";":
            end = j
        yield kind, name, m.start(), end


def iter_funcs(src: str):
    masked = _mask_comments(src)
    for m in _FUNC_RE.finditer(masked):
        ret, name, params, tail = m.group(1), m.group(2), m.group(3), m.group(4)
        if name.startswith("_"):
            continue
        ret = " ".join(ret.split())
        params = " ".join(params.split())
        sig = f"{ret} {name}({params})"
        has_body = tail == "{"
        yield name, sig, ret, m.start(), m.end(), has_body


def _brief_comment(src: str, idx: int) -> str:
    window = src[max(0, idx - 2000):idx]
    m = re.search(r"/\*\*([\s\S]*?)\*/\s*$", window)
    if m:
        txt = re.sub(r"^\s*\*\s?", "", m.group(1), flags=re.M).strip()
        return re.sub(r"\s+", " ", txt)[:400]
    lines = window.splitlines()
    brief = []
    for line in reversed(lines[-10:]):
        s = line.strip()
        if s.startswith("//"):
            brief.append(s[2:].strip())
        elif s == "":
            continue
        else:
            break
    if brief:
        brief.reverse()
        return re.sub(r"\s+", " ", " ".join(brief))[:400]
    return ""


# =============================================================================
# MCU SUPPORT MATRIX
# =============================================================================

def parse_eclipse_project(project_file: Path) -> List[str]:
    sources = []
    try:
        tree = ET.parse(project_file)
        for link in tree.getroot().findall('.//linkedResources/link'):
            name_elem = link.find('n') or link.find('name')
            type_elem = link.find('type')
            if name_elem is None or type_elem is None:
                continue
            if type_elem.text != '1':
                continue
            name = name_elem.text or ''
            if any(name.endswith(ext) for ext in ['.c', '.cpp', '.cc']):
                sources.append(os.path.basename(name))
    except Exception:
        pass
    return sources


def find_eclipse_lib_projects(root: Path) -> List[Path]:
    projects = []
    for arch_dir in ['ARM', 'RISCV']:
        pattern = str(root / arch_dir / '**' / 'lib' / 'Eclipse' / '.project')
        projects.extend(Path(p) for p in glob.glob(pattern, recursive=True))
    return sorted(projects)


def extract_mcu(project_path: Path) -> Optional[str]:
    parts = project_path.parts
    for i, part in enumerate(parts):
        if part.lower() == 'lib' and i > 0:
            return parts[i - 1].upper().replace('-', '').replace('_', '')
    return None


def build_mcu_support(conn: sqlite3.Connection, root: Path, file_cache: StringCache, verbose: bool) -> int:
    projects = find_eclipse_lib_projects(root)
    count = 0
    conn.execute("DELETE FROM mcu_support")
    
    for project_file in projects:
        mcu = extract_mcu(project_file)
        if not mcu:
            continue
        sources = parse_eclipse_project(project_file)
        seen = set()
        for filename in sources:
            periph, impl_type = detect_peripheral_from_impl(filename)
            if periph == PERIPH_NONE:
                continue
            key = (mcu, periph, impl_type)
            if key in seen:
                continue
            seen.add(key)
            file_id = file_cache.get_id(filename)
            conn.execute(
                "INSERT OR REPLACE INTO mcu_support(mcu, periph, impl_type, file_id) VALUES(?,?,?,?)",
                (mcu, periph, impl_type, file_id)
            )
            count += 1
    conn.commit()
    return count


# =============================================================================
# DEVICE SUPPORT
# =============================================================================

DEVICE_NAME_FIXES = {
    "bme280": "BME280", "bme680": "BME680", "bme688": "BME688",
    "bmp280": "BMP280", "bmp388": "BMP388", "sht31": "SHT31", "sht40": "SHT40",
    "mpu6050": "MPU6050", "mpu9250": "MPU9250", "icm20948": "ICM20948",
    "lis2dh": "LIS2DH", "lis3dh": "LIS3DH", "lsm6dso": "LSM6DSO",
    "bmi160": "BMI160", "bmi270": "BMI270", "bno055": "BNO055",
    "ssd1306": "SSD1306", "ssd1351": "SSD1351", "ili9341": "ILI9341",
    "st7735": "ST7735", "st7789": "ST7789", "sh1106": "SH1106",
    "bq24295": "BQ24295", "max17048": "MAX17048", "apa102": "APA102",
    "ws2812": "WS2812", "vs1053": "VS1053",
}


def extract_device_name(filename: str) -> Optional[str]:
    stem = Path(filename).stem.lower()
    for prefix in ('tph_', 'tphg_', 'agm_', 'ag_', 'accel_', 'gyro_', 'mag_',
                   'led_', 'lcd_', 'oled_', 'disp_', 'pmic_', 'charger_'):
        if stem.startswith(prefix):
            stem = stem[len(prefix):]
            break
    if stem in DEVICE_NAME_FIXES:
        return DEVICE_NAME_FIXES[stem]
    match = re.search(r'([a-z]{2,})[-_]?(\d{2,}[a-z]*)', stem)
    if match:
        candidate = (match.group(1) + match.group(2)).lower()
        if candidate in DEVICE_NAME_FIXES:
            return DEVICE_NAME_FIXES[candidate]
        return candidate.upper()
    return None


def build_device_support(conn: sqlite3.Connection, root: Path, file_cache: StringCache, verbose: bool) -> int:
    count = 0
    seen = set()
    conn.execute("DELETE FROM devices")
    
    device_dirs = ['sensors', 'display', 'miscdev', 'pwrmgnt', 'imu', 'audio', 'storage', 'converters']
    for base in ['include', 'src']:
        for dev_dir in device_dirs:
            dir_path = root / base / dev_dir
            if not dir_path.exists():
                continue
            category = DEVCAT_MAP.get(dev_dir, DEVCAT_NONE)
            for ext in ['.h', '.hpp', '.cpp', '.c']:
                for filepath in dir_path.glob(f'*{ext}'):
                    if not filepath.is_file():
                        continue
                    device_name = extract_device_name(filepath.name)
                    if not device_name:
                        continue
                    key = (device_name, category)
                    if key in seen:
                        continue
                    seen.add(key)
                    rel_path = filepath.relative_to(root) if filepath.is_relative_to(root) else filepath
                    file_id = file_cache.get_id(str(rel_path))
                    conn.execute(
                        "INSERT OR REPLACE INTO devices(name, category, file_id) VALUES(?,?,?)",
                        (device_name, category, file_id)
                    )
                    count += 1
    conn.commit()
    return count


# =============================================================================
# EMBEDDING
# =============================================================================

class Embedder:
    BATCH = {"voyage": 72, "openai": 100, "hash": 500}
    RATE = {"voyage": 100, "openai": 500, "hash": 10000}

    def __init__(self, provider: str, api_key: Optional[str], model: str):
        self.provider = provider.lower()
        self.provider_id = PROVIDER_MAP.get(self.provider, 3)
        self.api_key = api_key
        self.model = model or {"voyage": "voyage-code-2", "openai": "text-embedding-3-small", "hash": "hash-512"}.get(self.provider, "hash-512")
        self._last = 0.0
        if self.provider in ("voyage", "openai") and not api_key:
            raise ValueError(f"{provider} requires API key")

    @property
    def batch_size(self) -> int:
        return self.BATCH.get(self.provider, 32)

    def _wait(self):
        if self.provider == "hash":
            return
        interval = 60.0 / self.RATE.get(self.provider, 100)
        elapsed = time.time() - self._last
        if elapsed < interval:
            time.sleep(interval - elapsed)
        self._last = time.time()

    def _post_json(self, url: str, headers: dict, payload: dict) -> dict:
        import urllib.request
        import json
        req = urllib.request.Request(url, json.dumps(payload).encode('utf-8'), headers=headers, method='POST')
        with urllib.request.urlopen(req, timeout=90) as resp:
            return json.loads(resp.read().decode('utf-8'))

    def embed_batch(self, texts: List[str]) -> List[bytes]:
        if not texts:
            return []
        if self.provider == "hash":
            return [self._hash(t) for t in texts]
        self._wait()
        if self.provider == "voyage":
            return self._voyage(texts)
        if self.provider == "openai":
            return self._openai(texts)
        return [b'' for _ in texts]

    def _hash(self, text: str) -> bytes:
        dim = 512
        vec = [0.0] * dim
        for tok in re.findall(r"[A-Za-z_]\w+", (text or "").lower())[:20000]:
            h = hashlib.blake2b(tok.encode(errors="ignore"), digest_size=8).digest()
            idx = int.from_bytes(h[:4], "little") % dim
            vec[idx] += 1.0 if (h[4] & 1) == 0 else -1.0
        norm = sum(x * x for x in vec) ** 0.5
        if norm > 0:
            vec = [x / norm for x in vec]
        return struct.pack(f"<{dim}f", *vec)

    def _voyage(self, texts: List[str]) -> List[bytes]:
        result = self._post_json(
            "https://api.voyageai.com/v1/embeddings",
            {"Authorization": f"Bearer {self.api_key}", "Content-Type": "application/json"},
            {"model": self.model, "input": [t[:8000] for t in texts], "input_type": "document"}
        )
        return [struct.pack(f"<{len(e)}f", *e) if e else b'' 
                for e in [item.get("embedding", []) for item in sorted(result.get("data", []), key=lambda x: x.get("index", 0))]]

    def _openai(self, texts: List[str]) -> List[bytes]:
        result = self._post_json(
            "https://api.openai.com/v1/embeddings",
            {"Authorization": f"Bearer {self.api_key}", "Content-Type": "application/json"},
            {"model": self.model, "input": [t[:8000] for t in texts]}
        )
        return [struct.pack(f"<{len(e)}f", *e) if e else b''
                for e in [item.get("embedding", []) for item in sorted(result.get("data", []), key=lambda x: x.get("index", 0))]]


# =============================================================================
# INDEX BUILDER
# =============================================================================

class IndexBuilder:
    def __init__(self, source: Path, output: Path, enable_fts: bool = True,
                 max_file_kb: int = 1024, max_chunk: int = 8000, example_cap: int = 400, verbose: bool = False):
        self.source = source
        self.output = output
        self.enable_fts = enable_fts
        self.max_bytes = max(64 * 1024, max_file_kb * 1024)
        self.max_chunk = max(1000, max_chunk)
        self.example_cap = example_cap
        self.verbose = verbose

    def _should_ignore(self, d: str) -> bool:
        return d in DEFAULT_IGNORE_DIRS or d.startswith(".") or d.startswith(IGNORE_DIR_PREFIXES)

    def _iter_files(self):
        for dp, dn, fn in os.walk(self.source):
            dn[:] = [d for d in dn if not self._should_ignore(d)]
            dn.sort()
            fn.sort()
            for f in fn:
                p = Path(dp) / f
                if p.suffix.lower() in SOURCE_SUFFIXES:
                    yield p

    def _relpath(self, p: Path) -> str:
        try:
            return str(p.resolve().relative_to(self.source.resolve())).replace("\\", "/")
        except:
            return str(p).replace("\\", "/")

    def _git_commit(self) -> Optional[bytes]:
        try:
            r = subprocess.run(["git", "rev-parse", "HEAD"], cwd=str(self.source),
                               stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
            if r.returncode == 0:
                return bytes.fromhex(r.stdout.decode().strip())
        except:
            pass
        return None

    def build(self, version: str) -> Path:
        self.output.mkdir(parents=True, exist_ok=True)
        db_path = self.output / "index.db"
        if db_path.exists():
            db_path.unlink()

        t0 = time.time()
        print(f"[00:00] Building index v{SCHEMA_VERSION}")

        conn = db_connect(db_path, baseline=True)
        ensure_schema(conn, enable_fts=self.enable_fts, include_mcu=True, include_devices=True)

        # Standardized metadata
        commit = self._git_commit()
        set_standard_meta(conn, DB_TYPE_CODE, version, commit=commit)

        # String caches
        file_cache = StringCache(conn, "files")
        module_cache = StringCache(conn, "modules")

        # MCU support
        print(f"[{_fmt(time.time()-t0)}] Building MCU support matrix...")
        mcu_count = build_mcu_support(conn, self.source, file_cache, self.verbose)

        # Device support
        print(f"[{_fmt(time.time()-t0)}] Building device support matrix...")
        device_count = build_device_support(conn, self.source, file_cache, self.verbose)

        # Index files
        print(f"[{_fmt(time.time()-t0)}] Scanning files...")
        stats = {"files": 0, "chunks": 0, "functions": 0, "types": 0, "examples": 0}

        for path in self._iter_files():
            stats["files"] += 1
            rel = self._relpath(path)
            file_id = file_cache.get_id(rel)
            module_id = module_cache.get_id(rel.split("/")[0] if "/" in rel else "core")
            periph = detect_peripheral_from_path(rel)

            try:
                src = _safe_read(path, self.max_bytes)
            except:
                continue

            # Precompute line index for O(1) line lookups
            nl_idx = build_line_index(src)

            # Examples
            is_example = any(h in rel.lower().split("/") for h in EXAMPLE_DIR_HINTS)
            if self.example_cap and is_example and stats["examples"] < self.example_cap:
                lines = src.splitlines()
                if len(lines) > 240:
                    src_chunk = "\n".join(lines[:180]) + "\n/* snip */\n" + "\n".join(lines[-40:])
                else:
                    src_chunk = src
                content = src_chunk[:self.max_chunk]
                conn.execute(
                    "INSERT INTO chunks(kind,periph,file_id,module_id,line_start,line_end,title,content,hash) VALUES(?,?,?,?,?,?,?,?,?)",
                    (KIND_EXAMPLE, periph, file_id, module_id, 1, len(lines), f"Example: {path.stem}",
                     compress(content), sha256(content))
                )
                stats["examples"] += 1
                stats["chunks"] += 1

            # Types
            for kind, name, s_idx, e_idx in iter_types(src):
                block = src[s_idx:e_idx+1]
                line = idx_to_line(nl_idx, s_idx)
                desc = _brief_comment(src, s_idx)
                content = f"{kind} {name}\n{desc}\n{block}"[:self.max_chunk]
                conn.execute(
                    "INSERT INTO chunks(kind,periph,file_id,module_id,line_start,line_end,title,content,hash) VALUES(?,?,?,?,?,?,?,?,?)",
                    (KIND_TYPE, periph, file_id, module_id, line, line + block.count("\n"),
                     f"{kind} {name}", compress(content), sha256(content))
                )
                stats["types"] += 1
                stats["chunks"] += 1

            # Functions
            for fname, sig, ret, s_idx, e_idx, has_body in iter_funcs(src):
                line = idx_to_line(nl_idx, s_idx)
                desc = _brief_comment(src, s_idx)
                body = ""
                if has_body:
                    masked = _mask_comments(src)
                    brace = masked.find("{", s_idx)
                    if brace >= 0:
                        end = _find_brace(masked, brace)
                        if end:
                            body = src[brace:end+1]
                content = f"{sig}\n{desc}\n{body}"[:self.max_chunk]
                conn.execute(
                    "INSERT INTO chunks(kind,periph,file_id,module_id,line_start,line_end,title,signature,content,hash) VALUES(?,?,?,?,?,?,?,?,?,?)",
                    (KIND_FUNCTION, periph, file_id, module_id, line, line + content.count("\n"),
                     fname, compress(sig), compress(content), sha256(content))
                )
                conn.execute(
                    "INSERT INTO api(name,periph,file_id,line,signature,ret_type) VALUES(?,?,?,?,?,?)",
                    (fname, periph, file_id, line, compress(sig), compress(ret) if ret else None)
                )
                stats["functions"] += 1
                stats["chunks"] += 1

        conn.commit()

        if self.enable_fts:
            print(f"[{_fmt(time.time()-t0)}] Building FTS...")
            fts_rebuild(conn)

        conn.execute("VACUUM")
        conn.close()

        elapsed = time.time() - t0
        size_kb = db_path.stat().st_size / 1024
        print(f"\n[{_fmt(elapsed)}] ✓ Complete")
        print(f"  Files:     {stats['files']}")
        print(f"  Chunks:    {stats['chunks']}")
        print(f"  Functions: {stats['functions']}")
        print(f"  Types:     {stats['types']}")
        print(f"  Examples:  {stats['examples']}")
        print(f"  MCU:       {mcu_count}")
        print(f"  Devices:   {device_count}")
        print(f"  DB size:   {size_kb:.1f} KB")
        return db_path

    def update_embeddings(self, db_path: Path, provider: str, api_key: Optional[str],
                          model: str, batch_size: int = 0, max_new: int = 0) -> int:
        kinds = [KIND_FUNCTION, KIND_TYPE, KIND_EXAMPLE]
        embedder = Embedder(provider, api_key, model)
        if batch_size <= 0:
            batch_size = embedder.batch_size

        conn = db_connect(db_path, baseline=False)
        conn.execute("INSERT OR IGNORE INTO models(name) VALUES(?)", (embedder.model,))
        row = conn.execute("SELECT id FROM models WHERE name=?", (embedder.model,)).fetchone()
        model_id = int(row[0])

        placeholders = ",".join("?" * len(kinds))
        rows = conn.execute(f"""
            SELECT c.id, c.content FROM chunks c
            WHERE c.kind IN ({placeholders})
            AND NOT EXISTS (SELECT 1 FROM embeddings e WHERE e.chunk_id=c.id AND e.provider=? AND e.model_id=?)
        """, (*kinds, embedder.provider_id, model_id)).fetchall()
        
        if max_new > 0:
            rows = rows[:max_new]

        total = len(rows)
        print(f"Embedding {total} chunks: {provider}/{embedder.model}")
        if total == 0:
            conn.close()
            return 0

        t0, done, now = time.time(), 0, int(time.time())
        for i in range(0, total, batch_size):
            batch = rows[i:i + batch_size]
            ids = [r[0] for r in batch]
            texts = [decompress(r[1]) for r in batch]
            try:
                embeddings = embedder.embed_batch(texts)
                for chunk_id, emb in zip(ids, embeddings):
                    if emb:
                        conn.execute("INSERT OR REPLACE INTO embeddings VALUES(?,?,?,?,?)",
                                     (chunk_id, embedder.provider_id, model_id, emb, now))
                        done += 1
                conn.commit()
                print(f"  [{_fmt(time.time()-t0)}] {done}/{total}")
            except Exception as e:
                print(f"  [error] {e}")

        conn.close()
        print(f"✓ {done} embeddings ({done/(time.time()-t0):.1f}/s)")
        return done


def _fmt(s: float) -> str:
    m, s = divmod(int(s), 60)
    return f"{m:02d}:{s:02d}"


def _detect_root() -> Optional[Path]:
    cwd = Path.cwd()
    for p in [cwd] + list(cwd.parents)[:3]:
        if (p / "include/iopinctrl.h").exists():
            return p
    return None


def main():
    p = argparse.ArgumentParser(description="Build IOsonata RAG index v6 (unified schema)")
    p.add_argument("--source-dir", default="")
    p.add_argument("--output-dir", default="")
    p.add_argument("--version", default="dev")
    p.add_argument("--no-fts", action="store_true")
    p.add_argument("--verbose", action="store_true")
    p.add_argument("--update-embeddings", action="store_true")
    p.add_argument("--provider", default="openai")
    p.add_argument("--api-key", default="")
    p.add_argument("--model", default="")
    p.add_argument("--batch-size", type=int, default=0)
    p.add_argument("--max-new", type=int, default=0)
    args = p.parse_args()

    if not args.api_key:
        args.api_key = os.environ.get('OPENAI_API_KEY') or os.environ.get('VOYAGE_API_KEY', '')

    source = Path(args.source_dir) if args.source_dir else (_detect_root() or Path("."))
    output = Path(args.output_dir) if args.output_dir else (source / ".iosonata")

    builder = IndexBuilder(source, output, enable_fts=not args.no_fts, verbose=args.verbose)

    if args.update_embeddings:
        builder.update_embeddings(output / "index.db", args.provider, args.api_key or None, args.model, args.batch_size, args.max_new)
    else:
        print(f"Source: {source.resolve()}\nOutput: {output.resolve()}\n")
        builder.build(args.version)


if __name__ == "__main__":
    main()
