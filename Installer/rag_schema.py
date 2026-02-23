#!/usr/bin/env python3
"""
IOcomposer RAG Unified Schema v9

This module defines the common schema used by all three indexers:
- build_rag_index.py (IOsonata code)
- build_external_index.py (External SDKs)
- build_knowledge_db.py (Documentation/Knowledge)

Design principles:
1. Binary-first: Integer enums, compressed BLOBs, binary hashes
2. Consistent: Same table names, column names, types across all DBs
3. Machine-optimized: No JSON stored, FTS5 title-only (contentless)
4. Simple: Java only needs one schema handler

Exported constants:
- SCHEMA_VERSION: Current schema version (9)
- KIND_*: Chunk type constants (0-14)
- PERIPH_*: Peripheral type constants (0-19)
- DEVCAT_*: Device category constants (0-8)
- PROVIDER_*: Embedding provider constants (1-3)
- DB_TYPE_*: Database type identifiers

v7 additions:
- Manifest table for static data (MCU list, device list)
- Pre-computed JSON for system prompt injection
- build_manifest() and generate_manifest_context() functions

v8 additions:
- Base class hierarchy scanning (sensor types, device types, interfaces)
- BASE_CLASSES_SQL table definition in unified schema
- DEVCAT_* constants moved here from build_rag_index.py
- base_classes_by_category and base_class_details in manifest

v9 additions:
- Added 'header' column to devices table for header file paths
- Manifest includes device_details with header paths for LLM tool fetching
- generate_manifest_context() outputs device headers for fetch_iosonata_header tool

Schema tables (controlled by ensure_schema flags):
- Core: meta, files, modules, chunks, models, embeddings, api, fts
- IOsonata only: mcu_support, devices, manifest, base_classes

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

SCHEMA_VERSION = 9  # Bumped for device header path support
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

# =============================================================================
# DEVICE CATEGORY IDS
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

# Directory name -> category ID mapping (for indexer)
DEVCAT_MAP = {
    "sensors": DEVCAT_SENSOR,
    "display": DEVCAT_DISPLAY,
    "miscdev": DEVCAT_MISCDEV,
    "pwrmgnt": DEVCAT_PMIC,
    "imu": DEVCAT_IMU,
    "audio": DEVCAT_AUDIO,
    "storage": DEVCAT_STORAGE,
    "converters": DEVCAT_CONVERTER,
}

# Category ID -> name mapping (for manifest)
DEVCAT_NAMES = {
    DEVCAT_NONE: "unknown",
    DEVCAT_SENSOR: "sensor",
    DEVCAT_DISPLAY: "display",
    DEVCAT_MISCDEV: "miscdev",
    DEVCAT_PMIC: "pmic",
    DEVCAT_IMU: "imu",
    DEVCAT_AUDIO: "audio",
    DEVCAT_STORAGE: "storage",
    DEVCAT_CONVERTER: "converter",
}

# Alias for backward compatibility
DEVICE_CATEGORY_NAMES = DEVCAT_NAMES

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

# Device support table (IOsonata only) - v9: added header column
DEVICES_SQL = """
CREATE TABLE IF NOT EXISTS devices (
    id INTEGER PRIMARY KEY,
    name TEXT NOT NULL,
    category INTEGER NOT NULL,
    file_id INTEGER,
    header TEXT,                    -- v9: relative path to header file for LLM fetching
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

# Base class hierarchy table (IOsonata only) - for sensor/device/interface inheritance (v8)
BASE_CLASSES_SQL = """
CREATE TABLE IF NOT EXISTS base_classes (
    id INTEGER PRIMARY KEY,
    name TEXT UNIQUE NOT NULL,      -- e.g., "TempSensor", "AccelSensor"
    category TEXT NOT NULL,         -- "sensor", "interface", "converter", etc.
    header TEXT NOT NULL,           -- relative path: "include/sensors/temp_sensor.h"
    parent TEXT,                    -- parent class name or NULL
    config_struct TEXT              -- e.g., "TempSensorCfg_t" or NULL
);
CREATE INDEX IF NOT EXISTS idx_baseclass_name ON base_classes(name);
CREATE INDEX IF NOT EXISTS idx_baseclass_cat ON base_classes(category);
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
                  include_manifest: bool = False, include_base_classes: bool = False) -> None:
    """Create unified schema.
    
    Args:
        conn: Database connection
        enable_fts: Create FTS5 virtual table
        include_mcu: Create mcu_support table (IOsonata only)
        include_devices: Create devices table (IOsonata only)
        include_manifest: Create manifest table (IOsonata only) - v7
        include_base_classes: Create base_classes table (IOsonata only) - v8
    """
    conn.executescript(SCHEMA_SQL)
    
    if include_mcu:
        conn.executescript(MCU_SUPPORT_SQL)
    
    if include_devices:
        conn.executescript(DEVICES_SQL)
    
    if include_manifest:
        conn.executescript(MANIFEST_SQL)
    
    if include_base_classes:
        conn.executescript(BASE_CLASSES_SQL)
    
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
    """Build manifest table from mcu_support, devices, and base_classes tables.
    
    Call this AFTER build_mcu_support(), build_device_support(), and build_base_class_hierarchy().
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
    
    # === DEVICE MANIFEST (v9: now includes header paths) ===
    rows = conn.execute("SELECT name, category, header FROM devices ORDER BY category, name").fetchall()
    
    devices = []
    device_by_category = {}
    device_details = {}  # v9: Store device details including header path
    
    for name, category, header in rows:
        devices.append(name)
        cat_name = DEVICE_CATEGORY_NAMES.get(category, "unknown")
        if cat_name not in device_by_category:
            device_by_category[cat_name] = []
        if name not in device_by_category[cat_name]:
            device_by_category[cat_name].append(name)
        
        # v9: Store device details with header path
        device_details[name] = {
            'category': cat_name,
            'header': header or ''
        }
    
    conn.execute("INSERT OR REPLACE INTO manifest(key,value,updated) VALUES(?,?,?)",
                 ("device_list", json.dumps(sorted(set(devices))), now))
    count += 1
    
    conn.execute("INSERT OR REPLACE INTO manifest(key,value,updated) VALUES(?,?,?)",
                 ("device_by_category", json.dumps({k: sorted(v) for k, v in sorted(device_by_category.items())}), now))
    count += 1
    
    # v9: Store device details with header paths
    conn.execute("INSERT OR REPLACE INTO manifest(key,value,updated) VALUES(?,?,?)",
                 ("device_details", json.dumps(device_details), now))
    count += 1
    
    # === BASE CLASS HIERARCHY (v8) ===
    try:
        rows = conn.execute("""
            SELECT name, category, header, parent, config_struct 
            FROM base_classes 
            ORDER BY category, name
        """).fetchall()
        
        base_classes_by_category = {}
        base_class_details = {}
        
        for name, category, header, parent, config_struct in rows:
            # Group by category
            if category not in base_classes_by_category:
                base_classes_by_category[category] = []
            base_classes_by_category[category].append(name)
            
            # Store details for each base class
            base_class_details[name] = {
                'header': header,
                'parent': parent,
                'config_struct': config_struct,
                'category': category
            }
        
        conn.execute("INSERT OR REPLACE INTO manifest(key,value,updated) VALUES(?,?,?)",
                     ("base_classes_by_category", json.dumps({k: sorted(v) for k, v in sorted(base_classes_by_category.items())}), now))
        count += 1
        
        conn.execute("INSERT OR REPLACE INTO manifest(key,value,updated) VALUES(?,?,?)",
                     ("base_class_details", json.dumps(base_class_details), now))
        count += 1
    except sqlite3.OperationalError:
        # base_classes table doesn't exist yet (older index)
        pass
    
    conn.commit()
    return count


def get_manifest_json(conn: sqlite3.Connection, key: str):
    """Get manifest value as parsed JSON."""
    row = conn.execute("SELECT value FROM manifest WHERE key=?", (key,)).fetchone()
    return json.loads(row[0]) if row and row[0] else None


def generate_manifest_context(conn: sqlite3.Connection) -> str:
    """Generate static context fragment for system prompt injection.
    
    This should be loaded ONCE at startup and cached in the system prompt.
    v9: Now includes device header paths for LLM tool fetching.
    """
    mcu_by_family = get_manifest_json(conn, "mcu_by_family") or {}
    mcu_periph_matrix = get_manifest_json(conn, "mcu_periph_matrix") or {}
    device_by_cat = get_manifest_json(conn, "device_by_category") or {}
    device_details = get_manifest_json(conn, "device_details") or {}  # v9
    base_classes_by_cat = get_manifest_json(conn, "base_classes_by_category") or {}
    base_class_details = get_manifest_json(conn, "base_class_details") or {}
    
    lines = []
    lines.append("=== IOsonata Supported Hardware (Authoritative) ===")
    lines.append("")
    lines.append("## Supported MCUs")
    for family, mcus in sorted(mcu_by_family.items()):
        if mcus:
            lines.append(f"- **{family.upper()}**: {', '.join(mcus)}")
    lines.append("")
    
    # MCU Peripheral Support Matrix — only peripherals IOsonata has implemented
    if mcu_periph_matrix:
        lines.append("## IOsonata Peripheral Implementation by MCU")
        lines.append("Peripherals listed below have IOsonata implementations. Use IOsonata API for these.")
        lines.append("For peripherals NOT listed, use vendor SDK directly.")
        lines.append("")
        for mcu, periphs in sorted(mcu_periph_matrix.items()):
            if periphs:
                lines.append(f"**{mcu}**: {', '.join(sorted(periphs))}")
        lines.append("")
    
    # v9: Device drivers with header paths for LLM tool fetching
    lines.append("## Supported Device Drivers")
    lines.append("Use `fetch_iosonata_header` tool with the header path to get struct definitions.")
    lines.append("")
    for category, devices in sorted(device_by_cat.items()):
        if devices and category != "unknown":
            lines.append(f"### {category.title()}")
            for dev_name in sorted(devices):
                info = device_details.get(dev_name, {})
                header = info.get('header', '')
                if header:
                    lines.append(f"- **{dev_name}**: `{header}`")
                else:
                    lines.append(f"- {dev_name}")
            lines.append("")
    
    # === BASE CLASS HIERARCHY (v8) ===
    if base_classes_by_cat:
        lines.append("## Base Class Hierarchy (for implementing new drivers)")
        lines.append("")
        
        # Sensor base classes
        sensor_bases = base_classes_by_cat.get('sensor', [])
        if sensor_bases:
            lines.append("### Sensor Base Classes")
            lines.append("Use these when implementing sensor drivers:")
            lines.append("")
            lines.append("| Sensor Type | Base Class | Header | Config Struct |")
            lines.append("|-------------|------------|--------|---------------|")
            for name in sorted(sensor_bases):
                info = base_class_details.get(name, {})
                header = info.get('header', '')
                config = info.get('config_struct', '') or '—'
                lines.append(f"| {name.replace('Sensor', '')} | {name} | `{header}` | {config} |")
            lines.append("")
            lines.append("**If sensor type NOT listed above:** inherit directly from `Sensor` (sensors/sensor.h)")
            lines.append("")
        
        # Converter base classes (ADC/DAC)
        converter_bases = base_classes_by_cat.get('converter', [])
        if converter_bases:
            lines.append("### Converter Base Classes (ADC/DAC)")
            lines.append("Use these for analog-to-digital and digital-to-analog converters:")
            lines.append("")
            for name in sorted(converter_bases):
                info = base_class_details.get(name, {})
                header = info.get('header', '')
                config = info.get('config_struct', '') or ''
                config_str = f" (Config: {config})" if config else ""
                lines.append(f"- **{name}**: `{header}`{config_str}")
            lines.append("")
        
        # Core base classes
        core_bases = base_classes_by_cat.get('core', [])
        if core_bases:
            lines.append("### Core Base Classes")
            for name in sorted(core_bases):
                info = base_class_details.get(name, {})
                header = info.get('header', '')
                lines.append(f"- **{name}**: `{header}`")
            lines.append("")
        
        # Interface base classes
        iface_bases = base_classes_by_cat.get('interface', [])
        if iface_bases:
            lines.append("### Interface Base Classes")
            for name in sorted(iface_bases):
                info = base_class_details.get(name, {})
                header = info.get('header', '')
                lines.append(f"- **{name}**: `{header}`")
            lines.append("")
        
        # Display base classes
        display_bases = base_classes_by_cat.get('display', [])
        if display_bases:
            lines.append("### Display Base Classes")
            for name in sorted(display_bases):
                info = base_class_details.get(name, {})
                header = info.get('header', '')
                lines.append(f"- **{name}**: `{header}`")
            lines.append("")
        
        # PMIC/Power management base classes
        pmic_bases = base_classes_by_cat.get('pmic', [])
        if pmic_bases:
            lines.append("### Power Management Base Classes")
            for name in sorted(pmic_bases):
                info = base_class_details.get(name, {})
                header = info.get('header', '')
                lines.append(f"- **{name}**: `{header}`")
            lines.append("")
        
        # Audio base classes
        audio_bases = base_classes_by_cat.get('audio', [])
        if audio_bases:
            lines.append("### Audio Base Classes")
            for name in sorted(audio_bases):
                info = base_class_details.get(name, {})
                header = info.get('header', '')
                lines.append(f"- **{name}**: `{header}`")
            lines.append("")
    
    lines.append("---")
    lines.append("⚠️ AUTHORITATIVE LIST: If a device/MCU is NOT listed above, IOsonata does NOT support it.")
    lines.append("⚠️ BASE CLASSES: If a sensor type is NOT listed, inherit from Sensor directly.")
    lines.append("⚠️ HEADER PATHS: Use the paths shown above with fetch_iosonata_header tool.")
    lines.append("Do NOT fabricate driver names or base class names. Ask user for part number if needed.")
    lines.append("")
    
    return "\n".join(lines)
