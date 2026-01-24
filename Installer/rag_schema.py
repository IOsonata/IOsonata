#!/usr/bin/env python3
"""
IOcomposer RAG Unified Schema v7

This module defines the common schema used by all three indexers:
- build_rag_index.py (IOsonata code)
- build_external_index.py (External SDKs)
- build_knowledge_db.py (Documentation/Knowledge)

Design principles:
1. Binary-first: Integer enums, compressed BLOBs, binary hashes
2. Consistent: Same table names, column names, types across all DBs
3. Machine-optimized: No JSON stored, FTS5 title-only (contentless)
4. Simple: Java only needs one schema handler

v7 additions:
- Manifest table for static data (MCU list, device list)
- Pre-computed JSON for system prompt injection
- build_manifest() and generate_manifest_context() functions

All DBs output to:
- IOsonata: .iosonata/index.db
- External: external/.extsdk/{sdk}.db
- Knowledge: .iosonata/knowledge.db
"""

import bisect
import hashlib
import json
import os
import sqlite3
import struct
import time
import zlib
from pathlib import Path
from typing import Dict, Iterator, Optional

# =============================================================================
# CONSTANTS
# =============================================================================

SCHEMA_VERSION = 7  # Bumped for manifest support
COMPRESS_LEVEL = 6

# =============================================================================
# SOURCE FILE SUFFIXES
# =============================================================================

SOURCE_SUFFIXES_CODE = {".h", ".hpp", ".hh", ".c", ".cc", ".cpp", ".cxx", ".inc", ".inl"}
SOURCE_SUFFIXES_ASM = {".s", ".S"}
SOURCE_SUFFIXES_ALL = SOURCE_SUFFIXES_CODE | SOURCE_SUFFIXES_ASM

# =============================================================================
# CHUNK KINDS (unified across all indexers)
# =============================================================================

KIND_UNKNOWN = 0
KIND_FUNCTION = 1
KIND_TYPE = 2
KIND_EXAMPLE = 3
KIND_ASSEMBLY = 4
KIND_HEADER = 5
# Knowledge-specific kinds (10+)
KIND_CONCEPT = 10
KIND_ENTITY = 11
KIND_PATTERN = 12
KIND_RECIPE = 13
KIND_BOOK_EXTRACT = 14

KIND_MAP = {
    "unknown": KIND_UNKNOWN,
    "function": KIND_FUNCTION,
    "type": KIND_TYPE,
    "example": KIND_EXAMPLE,
    "assembly": KIND_ASSEMBLY,
    "header": KIND_HEADER,
    "concept": KIND_CONCEPT,
    "entity": KIND_ENTITY,
    "pattern": KIND_PATTERN,
    "recipe": KIND_RECIPE,
    "book": KIND_BOOK_EXTRACT,
    "book_extract": KIND_BOOK_EXTRACT,
}

# =============================================================================
# PERIPHERAL IDS
# =============================================================================

PERIPH_NONE = 0
PERIPH_BLE = 1
PERIPH_UART = 2
PERIPH_SPI = 3
PERIPH_I2C = 4
PERIPH_USB = 5
PERIPH_TIMER = 6
PERIPH_PWM = 7
PERIPH_ADC = 8
PERIPH_GPIO = 9
PERIPH_FLASH = 10
PERIPH_I2S = 11
PERIPH_PDM = 12
PERIPH_QSPI = 13
PERIPH_ESB = 14
PERIPH_SENSOR = 15
PERIPH_RTC = 16
PERIPH_WDT = 17
PERIPH_DMA = 18
PERIPH_CRYPTO = 19

PERIPH_MAP = {
    "ble": PERIPH_BLE, "bluetooth": PERIPH_BLE,
    "uart": PERIPH_UART, "serial": PERIPH_UART,
    "spi": PERIPH_SPI,
    "i2c": PERIPH_I2C, "twi": PERIPH_I2C,
    "usb": PERIPH_USB,
    "timer": PERIPH_TIMER,
    "pwm": PERIPH_PWM,
    "adc": PERIPH_ADC, "saadc": PERIPH_ADC,
    "gpio": PERIPH_GPIO, "iopin": PERIPH_GPIO,
    "flash": PERIPH_FLASH, "nvmc": PERIPH_FLASH,
    "i2s": PERIPH_I2S,
    "pdm": PERIPH_PDM,
    "qspi": PERIPH_QSPI,
    "esb": PERIPH_ESB,
    "sensor": PERIPH_SENSOR, "imu": PERIPH_SENSOR,
    "rtc": PERIPH_RTC,
    "wdt": PERIPH_WDT, "watchdog": PERIPH_WDT,
    "dma": PERIPH_DMA,
    "crypto": PERIPH_CRYPTO, "aes": PERIPH_CRYPTO,
}

# Reverse mapping for manifest generation (v7)
PERIPHERAL_NAMES = {
    PERIPH_NONE: "none",
    PERIPH_BLE: "ble",
    PERIPH_UART: "uart",
    PERIPH_SPI: "spi",
    PERIPH_I2C: "i2c",
    PERIPH_USB: "usb",
    PERIPH_TIMER: "timer",
    PERIPH_PWM: "pwm",
    PERIPH_ADC: "adc",
    PERIPH_GPIO: "gpio",
    PERIPH_FLASH: "flash",
    PERIPH_I2S: "i2s",
    PERIPH_PDM: "pdm",
    PERIPH_QSPI: "qspi",
    PERIPH_ESB: "esb",
    PERIPH_SENSOR: "sensor",
    PERIPH_RTC: "rtc",
    PERIPH_WDT: "wdt",
    PERIPH_DMA: "dma",
    PERIPH_CRYPTO: "crypto",
}

# Device category names for manifest (v7)
DEVICE_CATEGORY_NAMES = {
    0: "unknown",
    1: "sensor",
    2: "display",
    3: "miscdev",
    4: "pmic",
    5: "imu",
    6: "audio",
    7: "storage",
    8: "converter",
}

# =============================================================================
# PROVIDER IDS
# =============================================================================

PROVIDER_VOYAGE = 1
PROVIDER_OPENAI = 2
PROVIDER_HASH = 3

PROVIDER_MAP = {
    "voyage": PROVIDER_VOYAGE,
    "openai": PROVIDER_OPENAI,
    "hash": PROVIDER_HASH,
}

# =============================================================================
# DATABASE CONNECTION
# =============================================================================

def db_connect(path: Path, baseline: bool = True) -> sqlite3.Connection:
    """Connect to database with standard settings.
    
    Args:
        path: Database file path
        baseline: If True, use DELETE journal (single file).
                  If False, use WAL (working copy mode).
    """
    conn = sqlite3.connect(str(path), timeout=30)
    if baseline:
        conn.execute("PRAGMA journal_mode=DELETE")
        conn.execute("PRAGMA synchronous=FULL")
    else:
        conn.execute("PRAGMA journal_mode=WAL")
        conn.execute("PRAGMA synchronous=NORMAL")
    conn.execute("PRAGMA cache_size=-65536")  # 64MB
    conn.execute("PRAGMA page_size=4096")
    conn.execute("PRAGMA temp_store=MEMORY")
    conn.execute("PRAGMA foreign_keys=ON")
    return conn


# =============================================================================
# UNIFIED SCHEMA
# =============================================================================

SCHEMA_SQL = """
-- Metadata (key-value, binary values)
CREATE TABLE IF NOT EXISTS meta (
    k TEXT PRIMARY KEY,
    v BLOB NOT NULL
);

-- String interning: file paths
CREATE TABLE IF NOT EXISTS files (
    id INTEGER PRIMARY KEY,
    path TEXT UNIQUE NOT NULL
);

-- String interning: module/category names
CREATE TABLE IF NOT EXISTS modules (
    id INTEGER PRIMARY KEY,
    name TEXT UNIQUE NOT NULL
);

-- Main chunks table (unified for code + knowledge)
CREATE TABLE IF NOT EXISTS chunks (
    id INTEGER PRIMARY KEY,
    kind INTEGER NOT NULL,          -- KIND_* constants
    periph INTEGER DEFAULT 0,       -- PERIPH_* constants (0 for knowledge)
    file_id INTEGER NOT NULL,       -- FK to files
    module_id INTEGER NOT NULL,     -- FK to modules
    line_start INTEGER DEFAULT 0,   -- 0 for knowledge chunks
    line_end INTEGER DEFAULT 0,
    title TEXT NOT NULL,            -- function name, type name, or doc title
    signature BLOB,                 -- compressed signature (code) or NULL
    content BLOB NOT NULL,          -- zlib compressed content
    hash BLOB NOT NULL,             -- 32-byte SHA256
    created INTEGER DEFAULT 0,      -- unix timestamp (for knowledge)
    FOREIGN KEY (file_id) REFERENCES files(id),
    FOREIGN KEY (module_id) REFERENCES modules(id)
);
CREATE INDEX IF NOT EXISTS idx_chunks_kind ON chunks(kind);
CREATE INDEX IF NOT EXISTS idx_chunks_periph ON chunks(periph);
CREATE INDEX IF NOT EXISTS idx_chunks_file ON chunks(file_id);
CREATE INDEX IF NOT EXISTS idx_chunks_module ON chunks(module_id);
CREATE INDEX IF NOT EXISTS idx_chunks_title ON chunks(title);
CREATE INDEX IF NOT EXISTS idx_chunks_hash ON chunks(hash);

-- Embedding models lookup
CREATE TABLE IF NOT EXISTS models (
    id INTEGER PRIMARY KEY,
    name TEXT UNIQUE NOT NULL
);

-- Embeddings (unified)
CREATE TABLE IF NOT EXISTS embeddings (
    chunk_id INTEGER NOT NULL,
    provider INTEGER NOT NULL,      -- PROVIDER_* constants
    model_id INTEGER NOT NULL,      -- FK to models
    embedding BLOB NOT NULL,        -- packed float32
    updated INTEGER NOT NULL,       -- unix timestamp
    PRIMARY KEY (chunk_id, provider, model_id),
    FOREIGN KEY (chunk_id) REFERENCES chunks(id) ON DELETE CASCADE,
    FOREIGN KEY (model_id) REFERENCES models(id)
);
CREATE INDEX IF NOT EXISTS idx_emb_provider ON embeddings(provider);

-- API functions (code indexers only, optional for knowledge)
CREATE TABLE IF NOT EXISTS api (
    id INTEGER PRIMARY KEY,
    name TEXT NOT NULL,
    periph INTEGER DEFAULT 0,
    file_id INTEGER,
    line INTEGER,
    signature BLOB,
    ret_type BLOB
);
CREATE INDEX IF NOT EXISTS idx_api_name ON api(name);
CREATE INDEX IF NOT EXISTS idx_api_periph ON api(periph);
"""

# MCU support table (IOsonata only)
MCU_SUPPORT_SQL = """
CREATE TABLE IF NOT EXISTS mcu_support (
    id INTEGER PRIMARY KEY,
    mcu TEXT NOT NULL,
    periph INTEGER NOT NULL,
    impl_type INTEGER DEFAULT 0,
    file_id INTEGER,
    UNIQUE(mcu, periph, impl_type)
);
CREATE INDEX IF NOT EXISTS idx_mcu ON mcu_support(mcu);
CREATE INDEX IF NOT EXISTS idx_mcu_periph ON mcu_support(mcu, periph);
"""

# Device support table (IOsonata only)
DEVICES_SQL = """
CREATE TABLE IF NOT EXISTS devices (
    id INTEGER PRIMARY KEY,
    name TEXT NOT NULL,
    category INTEGER NOT NULL,
    file_id INTEGER,
    class_name TEXT,
    description TEXT,
    UNIQUE(name, category)
);
CREATE INDEX IF NOT EXISTS idx_dev_name ON devices(name);
CREATE INDEX IF NOT EXISTS idx_dev_cat ON devices(category);
"""

# Manifest table (IOsonata only) - pre-computed static data for system prompt (v7)
MANIFEST_SQL = """
CREATE TABLE IF NOT EXISTS manifest (
    key TEXT PRIMARY KEY,
    value TEXT NOT NULL,
    updated INTEGER NOT NULL
);
"""

# FTS5 virtual table - title-only, contentless
# Design: indexes chunk titles for fast keyword search without duplicating content
# Content stays compressed in chunks.content (binary-first principle)
FTS_SQL = """
CREATE VIRTUAL TABLE IF NOT EXISTS fts USING fts5(
    title,
    content='',
    tokenize='porter unicode61'
);

CREATE TRIGGER IF NOT EXISTS chunks_ai AFTER INSERT ON chunks BEGIN
    INSERT INTO fts(rowid, title) VALUES (new.id, new.title);
END;

CREATE TRIGGER IF NOT EXISTS chunks_ad AFTER DELETE ON chunks BEGIN
    INSERT INTO fts(fts, rowid, title) VALUES('delete', old.id, old.title);
END;

CREATE TRIGGER IF NOT EXISTS chunks_au AFTER UPDATE ON chunks BEGIN
    INSERT INTO fts(fts, rowid, title) VALUES('delete', old.id, old.title);
    INSERT INTO fts(rowid, title) VALUES (new.id, new.title);
END;
"""


def ensure_schema(conn: sqlite3.Connection, enable_fts: bool = True,
                  include_mcu: bool = False, include_devices: bool = False,
                  include_manifest: bool = False) -> None:
    """Create unified schema.
    
    Args:
        conn: Database connection
        enable_fts: Create FTS5 virtual table
        include_mcu: Create mcu_support table (IOsonata only)
        include_devices: Create devices table (IOsonata only)
        include_manifest: Create manifest table (IOsonata only) - v7
    """
    conn.executescript(SCHEMA_SQL)
    
    if include_mcu:
        conn.executescript(MCU_SUPPORT_SQL)
    
    if include_devices:
        conn.executescript(DEVICES_SQL)
    
    if include_manifest:
        conn.executescript(MANIFEST_SQL)
    
    if enable_fts:
        conn.executescript(FTS_SQL)
    
    conn.commit()


def fts_rebuild(conn: sqlite3.Connection) -> None:
    """Rebuild FTS index from chunks (title-only, contentless)."""
    # For contentless FTS5, use 'delete-all' command then repopulate
    conn.execute("INSERT INTO fts(fts) VALUES('delete-all')")
    conn.execute("INSERT INTO fts(rowid, title) SELECT id, title FROM chunks")
    conn.commit()


# =============================================================================
# STANDARDIZED METADATA CONTRACT
# =============================================================================
# All DBs must have these meta keys:
#   schema     - u32 schema version
#   built      - i64 unix timestamp
#   type       - bytes: b'code', b'external', or b'knowledge'
#   version    - bytes: user-provided version string
# Optional:
#   commit     - 20-byte git SHA1 (if in git repo)
#   fingerprint - 32-byte SHA256 of source content (for skip-if-unchanged)

DB_TYPE_CODE = b'code'
DB_TYPE_EXTERNAL = b'external'
DB_TYPE_KNOWLEDGE = b'knowledge'


def set_standard_meta(conn: sqlite3.Connection, db_type: bytes, version: str,
                      commit: Optional[bytes] = None, fingerprint: Optional[bytes] = None) -> None:
    """Set standardized metadata for all DB types."""
    set_meta(conn, 'schema', pack_u32(SCHEMA_VERSION))
    set_meta(conn, 'built', pack_i64(int(time.time())))
    set_meta(conn, 'type', db_type)
    set_meta(conn, 'version', version.encode('utf-8'))
    if commit:
        set_meta(conn, 'commit', commit)
    if fingerprint:
        set_meta(conn, 'fingerprint', fingerprint)
    conn.commit()


# =============================================================================
# COMPRESSION HELPERS
# =============================================================================

def compress(text: str) -> bytes:
    """Compress text to bytes using zlib."""
    if not text:
        return b''
    return zlib.compress(text.encode('utf-8', errors='replace'), COMPRESS_LEVEL)


def decompress(data: bytes) -> str:
    """Decompress bytes to text."""
    if not data:
        return ''
    return zlib.decompress(data).decode('utf-8', errors='replace')


def sha256(text: str) -> bytes:
    """Return 32-byte SHA256 hash of text."""
    return hashlib.sha256(text.encode('utf-8', errors='ignore')).digest()


def sha256_bytes(data: bytes) -> bytes:
    """Return 32-byte SHA256 hash of bytes."""
    return hashlib.sha256(data).digest()


# =============================================================================
# STRING INTERNING
# =============================================================================

class StringCache:
    """Intern strings into files or modules table."""
    
    def __init__(self, conn: sqlite3.Connection, table: str):
        self.conn = conn
        self.table = table
        # Detect column name (path for files, name for modules)
        cols = [r[1] for r in conn.execute(f"PRAGMA table_info({table})")]
        self.col = 'path' if 'path' in cols else 'name'
        # Pre-load existing entries
        self._cache: Dict[str, int] = {}
        for row in conn.execute(f"SELECT id, {self.col} FROM {table}"):
            self._cache[row[1]] = row[0]
    
    def get_id(self, value: str) -> int:
        """Get or create ID for a string value."""
        if value in self._cache:
            return self._cache[value]
        
        self.conn.execute(
            f"INSERT OR IGNORE INTO {self.table}({self.col}) VALUES(?)",
            (value,)
        )
        row = self.conn.execute(
            f"SELECT id FROM {self.table} WHERE {self.col}=?",
            (value,)
        ).fetchone()
        
        if not row:
            raise RuntimeError(f"Failed to intern {value!r} into {self.table}")
        
        self._cache[value] = row[0]
        return row[0]


# =============================================================================
# METADATA HELPERS
# =============================================================================

def set_meta(conn: sqlite3.Connection, key: str, value: bytes) -> None:
    """Set a metadata key-value pair."""
    conn.execute("INSERT OR REPLACE INTO meta(k, v) VALUES(?, ?)", (key, value))


def get_meta(conn: sqlite3.Connection, key: str) -> Optional[bytes]:
    """Get a metadata value by key."""
    row = conn.execute("SELECT v FROM meta WHERE k=?", (key,)).fetchone()
    return row[0] if row else None


def pack_u32(x: int) -> bytes:
    return struct.pack("<I", x & 0xFFFFFFFF)


def pack_i64(x: int) -> bytes:
    return struct.pack("<q", x)


def unpack_u32(data: bytes) -> int:
    return struct.unpack("<I", data)[0]


def unpack_i64(data: bytes) -> int:
    return struct.unpack("<q", data)[0]


# =============================================================================
# SOURCE FINGERPRINTING (for skip-if-unchanged)
# =============================================================================

def compute_source_fingerprint(root: Path, suffixes: set, ignore_dirs: set,
                               ignore_prefixes: tuple = ()) -> bytes:
    """Compute SHA256 fingerprint from file stats (path + size + mtime_ns).
    
    Deterministic and properly prunes ignored directories.
    
    Args:
        root: Directory to fingerprint
        suffixes: File extensions to include (e.g. {'.c', '.h'})
        ignore_dirs: Exact directory names to skip
        ignore_prefixes: Directory name prefixes to skip (e.g. ('Debug', 'cmake-build-'))
    """
    h = hashlib.sha256()

    def should_ignore_dir(name: str) -> bool:
        if name in ignore_dirs or name.startswith('.'):
            return True
        if ignore_prefixes and name.startswith(ignore_prefixes):
            return True
        return False

    for dp, dn, fn in os.walk(root):
        # Prune + deterministic traversal (must modify dn in-place, not wrap os.walk)
        dn[:] = sorted(d for d in dn if not should_ignore_dir(d))
        fn = sorted(fn)

        for f in fn:
            p = Path(dp) / f
            if p.suffix.lower() not in suffixes:
                continue
            try:
                st = p.stat()
                rel = p.relative_to(root).as_posix()  # stable across OS
                h.update(rel.encode('utf-8'))
                h.update(struct.pack('<QQ', st.st_size, st.st_mtime_ns))
            except OSError:
                continue

    return h.digest()


# =============================================================================
# LINE NUMBER HELPERS (O(n) instead of O(n²))
# =============================================================================


def build_line_index(src: str) -> list:
    """Build newline index for O(1) line lookups.
    
    Returns list of newline positions. Use with idx_to_line().
    """
    return [i for i, c in enumerate(src) if c == '\n']


def idx_to_line(nl_index: list, char_idx: int) -> int:
    """Convert character index to 1-based line number using precomputed index."""
    return bisect.bisect_right(nl_index, char_idx) + 1


# =============================================================================
# MANIFEST FUNCTIONS (v7)
# =============================================================================

def detect_mcu_family(mcu: str) -> str:
    """Detect MCU family from name."""
    u = mcu.upper()
    if u.startswith("NRF52"): return "nrf52"
    if u.startswith("NRF53"): return "nrf53"
    if u.startswith("NRF54"): return "nrf54"
    if u.startswith("NRF91"): return "nrf91"
    if u.startswith("STM32"): return "stm32"
    if u.startswith("SAM"): return "sam"
    if u.startswith("LPC"): return "lpc"
    if u.startswith("ESP32"): return "esp32"
    if u.startswith("CH32"): return "ch32"
    return "other"


def build_manifest(conn: sqlite3.Connection) -> int:
    """Build manifest table from mcu_support and devices tables.
    
    Call this AFTER build_mcu_support() and build_device_support().
    Returns number of manifest entries created.
    """
    now = int(time.time())
    count = 0
    
    # === MCU MANIFEST ===
    rows = conn.execute("""
        SELECT DISTINCT mcu, periph FROM mcu_support ORDER BY mcu, periph
    """).fetchall()
    
    mcus = set()
    mcu_by_family = {}
    mcu_peripherals = {}
    
    for mcu, periph in rows:
        mcus.add(mcu)
        periph_name = PERIPHERAL_NAMES.get(periph, f"periph_{periph}")
        
        if mcu not in mcu_peripherals:
            mcu_peripherals[mcu] = []
        if periph_name != "none" and periph_name not in mcu_peripherals[mcu]:
            mcu_peripherals[mcu].append(periph_name)
        
        family = detect_mcu_family(mcu)
        if family not in mcu_by_family:
            mcu_by_family[family] = []
        if mcu not in mcu_by_family[family]:
            mcu_by_family[family].append(mcu)
    
    conn.execute("INSERT OR REPLACE INTO manifest(key,value,updated) VALUES(?,?,?)",
                 ("mcu_list", json.dumps(sorted(mcus)), now))
    count += 1
    
    conn.execute("INSERT OR REPLACE INTO manifest(key,value,updated) VALUES(?,?,?)",
                 ("mcu_by_family", json.dumps({k: sorted(v) for k, v in sorted(mcu_by_family.items())}), now))
    count += 1
    
    conn.execute("INSERT OR REPLACE INTO manifest(key,value,updated) VALUES(?,?,?)",
                 ("mcu_periph_matrix", json.dumps({k: sorted(v) for k, v in mcu_peripherals.items()}), now))
    count += 1
    
    # === DEVICE MANIFEST ===
    rows = conn.execute("SELECT name, category FROM devices ORDER BY category, name").fetchall()
    
    devices = []
    device_by_category = {}
    
    for name, category in rows:
        devices.append(name)
        cat_name = DEVICE_CATEGORY_NAMES.get(category, "unknown")
        if cat_name not in device_by_category:
            device_by_category[cat_name] = []
        if name not in device_by_category[cat_name]:
            device_by_category[cat_name].append(name)
    
    conn.execute("INSERT OR REPLACE INTO manifest(key,value,updated) VALUES(?,?,?)",
                 ("device_list", json.dumps(sorted(set(devices))), now))
    count += 1
    
    conn.execute("INSERT OR REPLACE INTO manifest(key,value,updated) VALUES(?,?,?)",
                 ("device_by_category", json.dumps({k: sorted(v) for k, v in sorted(device_by_category.items())}), now))
    count += 1
    
    conn.commit()
    return count


def get_manifest_json(conn: sqlite3.Connection, key: str):
    """Get manifest value as parsed JSON."""
    row = conn.execute("SELECT value FROM manifest WHERE key=?", (key,)).fetchone()
    return json.loads(row[0]) if row and row[0] else None


def generate_manifest_context(conn: sqlite3.Connection) -> str:
    """Generate static context fragment for system prompt injection.
    
    This should be loaded ONCE at startup and cached in the system prompt.
    """
    mcu_by_family = get_manifest_json(conn, "mcu_by_family") or {}
    device_by_cat = get_manifest_json(conn, "device_by_category") or {}
    
    lines = []
    lines.append("=== IOsonata Supported Hardware (Authoritative) ===")
    lines.append("")
    lines.append("## Supported MCUs")
    for family, mcus in sorted(mcu_by_family.items()):
        if mcus:
            lines.append(f"- **{family.upper()}**: {', '.join(mcus)}")
    lines.append("")
    lines.append("## Supported Device Drivers")
    for category, devices in sorted(device_by_cat.items()):
        if devices and category != "unknown":
            lines.append(f"### {category.title()}")
            lines.append(", ".join(devices))
            lines.append("")
    
    lines.append("---")
    lines.append("⚠️ AUTHORITATIVE LIST: If a device/MCU is NOT listed above, IOsonata does NOT support it.")
    lines.append("Do NOT fabricate driver names. Ask user for part number if needed.")
    lines.append("")
    
    return "\n".join(lines)
