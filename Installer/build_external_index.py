#!/usr/bin/env python3
"""
External SDK Index Builder v3 - Binary Optimized

Same optimizations as IOsonata indexer:
1. Integer IDs for enums
2. zlib compressed content
3. Lookup tables for strings
4. No JSON/XML/YAML

Usage:
  python3 build_external_index.py              # Index all SDKs
  python3 build_external_index.py --update-embeddings --api-key $KEY
"""

from __future__ import annotations

import argparse
import functools
import hashlib
import os
import re
import sqlite3
import struct
import time
import zlib
from datetime import datetime, timezone
from pathlib import Path
from typing import Dict, Iterator, List, Optional, Sequence, Tuple

print = functools.partial(print, flush=True)

SCHEMA_VERSION = 5
COMPRESS_LEVEL = 6

DEFAULT_IGNORE_DIRS = {
    ".git", ".github", ".metadata", ".settings", ".vscode", ".idea",
    "build", "out", "dist", "Debug", "Release", "OSX", "linux", "win32",
    "node_modules", "__pycache__", ".pytest_cache",
    "doc", "docs", "documentation", "doxygen",
}
IGNORE_DIR_PREFIXES = ("Debug", "Release", "cmake-build-")
SOURCE_SUFFIXES = {".h", ".hpp", ".hh", ".c", ".cc", ".cpp", ".cxx", ".inc", ".inl", ".s", ".S"}
EXAMPLE_DIR_HINTS = frozenset(("example", "examples", "sample", "samples", "demo", "demos", "test", "tests"))

# =============================================================================
# ENUM MAPPINGS
# =============================================================================

KIND_FUNCTION = 1
KIND_TYPE = 2
KIND_EXAMPLE = 3
KIND_ASSEMBLY = 4

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
PERIPH_DMA = 14
PERIPH_CRYPTO = 15

PERIPH_MAP = {
    "ble": PERIPH_BLE, "bluetooth": PERIPH_BLE, "softdevice": PERIPH_BLE, "nrf_sdh": PERIPH_BLE,
    "uart": PERIPH_UART, "serial": PERIPH_UART,
    "spi": PERIPH_SPI, "i2c": PERIPH_I2C, "twi": PERIPH_I2C,
    "usb": PERIPH_USB, "timer": PERIPH_TIMER, "pwm": PERIPH_PWM,
    "adc": PERIPH_ADC, "saadc": PERIPH_ADC, "gpio": PERIPH_GPIO, "pin": PERIPH_GPIO,
    "flash": PERIPH_FLASH, "nvmc": PERIPH_FLASH, "qspi": PERIPH_QSPI,
    "i2s": PERIPH_I2S, "pdm": PERIPH_PDM, "dma": PERIPH_DMA,
    "crypto": PERIPH_CRYPTO, "aes": PERIPH_CRYPTO, "sha": PERIPH_CRYPTO,
}

PROVIDER_VOYAGE = 1
PROVIDER_OPENAI = 2
PROVIDER_HASH = 3
PROVIDER_MAP = {"voyage": PROVIDER_VOYAGE, "openai": PROVIDER_OPENAI, "hash": PROVIDER_HASH}


def detect_peripheral(path: str) -> int:
    p = path.lower()
    for kw, periph_id in PERIPH_MAP.items():
        if kw in p:
            return periph_id
    return PERIPH_NONE


# =============================================================================
# COMPRESSION
# =============================================================================

def compress(text: str) -> bytes:
    if not text:
        return b''
    return zlib.compress(text.encode('utf-8', errors='replace'), COMPRESS_LEVEL)


def decompress(data: bytes) -> str:
    if not data:
        return ''
    return zlib.decompress(data).decode('utf-8', errors='replace')


def sha1_bytes(text: str) -> bytes:
    return hashlib.sha1(text.encode('utf-8', errors='ignore')).digest()


# =============================================================================
# DATABASE SCHEMA
# =============================================================================

def _db_connect(path: Path) -> sqlite3.Connection:
    conn = sqlite3.connect(str(path), timeout=30)
    conn.execute("PRAGMA journal_mode=WAL")
    conn.execute("PRAGMA synchronous=NORMAL")
    conn.execute("PRAGMA cache_size=-65536")
    conn.execute("PRAGMA page_size=4096")
    conn.execute("PRAGMA temp_store=MEMORY")
    return conn


def _ensure_schema(conn: sqlite3.Connection, enable_fts: bool) -> None:
    conn.executescript("""
    CREATE TABLE IF NOT EXISTS meta (k TEXT PRIMARY KEY, v BLOB);

    CREATE TABLE IF NOT EXISTS modules (id INTEGER PRIMARY KEY, name TEXT UNIQUE NOT NULL);
    CREATE TABLE IF NOT EXISTS files (id INTEGER PRIMARY KEY, path TEXT UNIQUE NOT NULL);

    CREATE TABLE IF NOT EXISTS chunks (
        id INTEGER PRIMARY KEY,
        kind INTEGER NOT NULL,
        periph INTEGER DEFAULT 0,
        file_id INTEGER NOT NULL,
        module_id INTEGER NOT NULL,
        line_start INTEGER,
        line_end INTEGER,
        title TEXT NOT NULL,
        signature BLOB,
        content BLOB NOT NULL,
        hash BLOB NOT NULL
    );
    CREATE INDEX IF NOT EXISTS idx_c_kind ON chunks(kind);
    CREATE INDEX IF NOT EXISTS idx_c_periph ON chunks(periph);
    CREATE INDEX IF NOT EXISTS idx_c_file ON chunks(file_id);
    CREATE INDEX IF NOT EXISTS idx_c_title ON chunks(title);

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

    CREATE TABLE IF NOT EXISTS embeddings (
        chunk_id INTEGER NOT NULL,
        provider INTEGER NOT NULL,
        model_id INTEGER NOT NULL,
        embedding BLOB NOT NULL,
        updated INTEGER NOT NULL,
        PRIMARY KEY (chunk_id, provider, model_id)
    );

    CREATE TABLE IF NOT EXISTS models (id INTEGER PRIMARY KEY, name TEXT UNIQUE NOT NULL);
    """)

    if enable_fts:
        conn.executescript("""
        CREATE VIRTUAL TABLE IF NOT EXISTS fts USING fts5(
            title, content, content='chunks', content_rowid='id', tokenize='porter unicode61'
        );
        CREATE TRIGGER IF NOT EXISTS chunks_ai AFTER INSERT ON chunks BEGIN
            INSERT INTO fts(rowid, title, content) VALUES (new.id, new.title, '');
        END;
        CREATE TRIGGER IF NOT EXISTS chunks_ad AFTER DELETE ON chunks BEGIN
            INSERT INTO fts(fts, rowid, title, content) VALUES('delete', old.id, old.title, '');
        END;
        """)
    conn.commit()


def _fts_rebuild(conn: sqlite3.Connection) -> None:
    conn.execute("INSERT INTO fts(fts) VALUES('rebuild')")
    conn.commit()


# =============================================================================
# STRING INTERNING
# =============================================================================

class StringCache:
    """Intern strings into a table with a UNIQUE text column.

    Compatibility note: macOS/Python can ship SQLite versions that
    do not support `RETURNING` (SQLite < 3.35). We therefore use
    INSERT OR IGNORE + SELECT, which works broadly.
    """

    def __init__(self, conn: sqlite3.Connection, table: str):
        self.conn = conn
        self.table = table
        self.value_col = self._detect_value_column(conn, table)
        self._cache: Dict[str, int] = {}
        for row in conn.execute(f"SELECT id, {self.value_col} FROM {table}"): 
            self._cache[row[1]] = row[0]

    @staticmethod
    def _detect_value_column(conn: sqlite3.Connection, table: str) -> str:
        cols = [r[1] for r in conn.execute(f"PRAGMA table_info({table})")]
        if 'name' in cols:
            return 'name'
        if 'path' in cols:
            return 'path'
        for c in cols:
            if c != 'id':
                return c
        raise RuntimeError(f"Cannot detect value column for table {table}")

    def get_id(self, value: str) -> int:
        if value in self._cache:
            return self._cache[value]
        self.conn.execute(
            f"INSERT OR IGNORE INTO {self.table}({self.value_col}) VALUES(?)",
            (value,)
        )
        row = self.conn.execute(
            f"SELECT id FROM {self.table} WHERE {self.value_col}=?",
            (value,)
        ).fetchone()
        if not row:
            raise RuntimeError(f"Failed to intern value into {self.table}: {value!r}")
        id_ = int(row[0])
        self._cache[value] = id_
        return id_


# =============================================================================
# CODE PARSING
# =============================================================================

def _safe_read(path: Path, max_bytes: int) -> str:
    try:
        raw = path.read_bytes()[:max_bytes]
        for enc in ("utf-8", "latin-1"):
            try:
                return raw.decode(enc)
            except:
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
    r"^[ \t]*(?:static\s+|inline\s+|extern\s+|__STATIC_INLINE\s+|IRAM_ATTR\s+)*"
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
        yield kind, name, m.start(), end


def iter_funcs(src: str):
    masked = _mask_comments(src)
    for m in _FUNC_RE.finditer(masked):
        ret, name, params, tail = m.group(1), m.group(2), m.group(3), m.group(4)
        if name.startswith("_"):
            continue
        sig = f"{' '.join(ret.split())} {name}({' '.join(params.split())})"
        yield name, sig, ret.strip(), m.start(), m.end(), tail == "{"


def _brief_comment(src: str, idx: int) -> str:
    window = src[max(0, idx - 2000):idx]
    m = re.search(r"/\*\*([\s\S]*?)\*/\s*$", window)
    if m:
        txt = re.sub(r"^\s*\*\s?", "", m.group(1), flags=re.M).strip()
        return re.sub(r"\s+", " ", txt)[:400]
    
    # Check for // single-line comments
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
# EMBEDDING
# =============================================================================

class Embedder:
    BATCH = {"voyage": 72, "openai": 100, "hash": 500}
    RATE = {"voyage": 100, "openai": 500, "hash": 10000}

    def __init__(self, provider: str, api_key: Optional[str], model: str):
        self.provider = provider.lower()
        self.provider_id = PROVIDER_MAP.get(self.provider, PROVIDER_HASH)
        self.api_key = api_key
        self.model = model or {"voyage": "voyage-code-2", "openai": "text-embedding-3-small"}.get(self.provider, "hash-512")
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
        import urllib.request
        import json
        url = "https://api.voyageai.com/v1/embeddings"
        headers = {"Authorization": f"Bearer {self.api_key}", "Content-Type": "application/json"}
        payload = {"model": self.model, "input": [t[:8000] for t in texts], "input_type": "document"}
        req = urllib.request.Request(url, json.dumps(payload).encode(), headers=headers, method="POST")
        with urllib.request.urlopen(req, timeout=90) as resp:
            result = json.loads(resp.read().decode())
        return [struct.pack(f"<{len(e)}f", *e) if (e := item.get("embedding")) else b''
                for item in sorted(result.get("data", []), key=lambda x: x.get("index", 0))]

    def _openai(self, texts: List[str]) -> List[bytes]:
        import urllib.request
        import json
        url = "https://api.openai.com/v1/embeddings"
        headers = {"Authorization": f"Bearer {self.api_key}", "Content-Type": "application/json"}
        payload = {"model": self.model, "input": [t[:8000] for t in texts]}
        req = urllib.request.Request(url, json.dumps(payload).encode(), headers=headers, method="POST")
        with urllib.request.urlopen(req, timeout=90) as resp:
            result = json.loads(resp.read().decode())
        return [struct.pack(f"<{len(e)}f", *e) if (e := item.get("embedding")) else b''
                for item in sorted(result.get("data", []), key=lambda x: x.get("index", 0))]


# =============================================================================
# SDK INDEXER
# =============================================================================

class SDKIndexer:
    def __init__(self, enable_fts: bool = True, max_file_kb: int = 1024,
                 max_chunk: int = 8000, example_cap: int = 250, verbose: bool = False):
        self.enable_fts = enable_fts
        self.max_bytes = max(64 * 1024, max_file_kb * 1024)
        self.max_chunk = max(1000, max_chunk)
        self.example_cap = example_cap
        self.verbose = verbose

    def _should_ignore(self, d: str) -> bool:
        return d in DEFAULT_IGNORE_DIRS or d.startswith(".") or d.startswith(IGNORE_DIR_PREFIXES)

    def _iter_files(self, root: Path):
        for dp, dn, fn in os.walk(root):
            dn[:] = [d for d in dn if not self._should_ignore(d)]
            dn.sort()
            fn.sort()
            for f in fn:
                p = Path(dp) / f
                if p.suffix.lower() in SOURCE_SUFFIXES:
                    yield p

    def _relpath(self, path: Path, root: Path) -> str:
        try:
            return str(path.resolve().relative_to(root.resolve())).replace("\\", "/")
        except:
            return str(path).replace("\\", "/")

    def _fingerprint(self, root: Path) -> bytes:
        h = hashlib.sha1()
        h.update(str(root.resolve()).encode())
        count, newest = 0, 0
        for p in self._iter_files(root):
            count += 1
            try:
                mt = int(p.stat().st_mtime)
                if mt > newest:
                    newest = mt
            except:
                pass
            if count > 5000:
                break
        h.update(struct.pack("<IQ", count, newest))
        return h.digest()

    def build_sdk_db(self, sdk_path: Path, name: str, out_db: Path, version: str, skip_unchanged: bool) -> Path:
        out_db.parent.mkdir(parents=True, exist_ok=True)
        fingerprint = self._fingerprint(sdk_path)

        if skip_unchanged and out_db.exists():
            try:
                conn = _db_connect(out_db)
                row = conn.execute("SELECT v FROM meta WHERE k='fingerprint'").fetchone()
                conn.close()
                if row and row[0] == fingerprint:
                    print(f"  - {name}: unchanged")
                    return out_db
            except:
                pass

        if out_db.exists():
            out_db.unlink()

        t0 = time.time()
        print(f"  - {name}: indexing...")

        conn = _db_connect(out_db)
        _ensure_schema(conn, self.enable_fts)

        conn.execute("INSERT INTO meta VALUES('schema', ?)", (struct.pack("<I", SCHEMA_VERSION),))
        conn.execute("INSERT INTO meta VALUES('version', ?)", (version.encode(),))
        conn.execute("INSERT INTO meta VALUES('built', ?)", (struct.pack("<Q", int(time.time())),))
        conn.execute("INSERT INTO meta VALUES('sdk_name', ?)", (name.encode(),))
        conn.execute("INSERT INTO meta VALUES('fingerprint', ?)", (fingerprint,))

        file_cache = StringCache(conn, "files")
        module_cache = StringCache(conn, "modules")

        stats = {"files": 0, "chunks": 0, "funcs": 0, "types": 0, "examples": 0, "asm": 0}
        last_prog = time.time()

        for path in self._iter_files(sdk_path):
            stats["files"] += 1
            rel = self._relpath(path, sdk_path)
            file_id = file_cache.get_id(rel)
            module_id = module_cache.get_id(rel.split("/")[0] if "/" in rel else "root")
            periph = detect_peripheral(rel)

            if stats["files"] % 50 == 0 or time.time() - last_prog > 3:
                print(f"    [{_fmt(time.time()-t0)}] {name}: files={stats['files']} chunks={stats['chunks']}")
                last_prog = time.time()

            src = _safe_read(path, self.max_bytes)
            if not src:
                continue

            # Assembly
            if path.suffix.lower() in (".s", ".S"):
                content = src[:self.max_chunk]
                conn.execute(
                    "INSERT INTO chunks(kind,periph,file_id,module_id,line_start,line_end,title,content,hash) VALUES(?,?,?,?,?,?,?,?,?)",
                    (KIND_ASSEMBLY, periph, file_id, module_id, 1, content.count("\n")+1,
                     f"asm:{path.stem}", compress(content), sha1_bytes(content))
                )
                stats["asm"] += 1
                stats["chunks"] += 1
                continue

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
                    (KIND_EXAMPLE, periph, file_id, module_id, 1, len(lines), f"Example:{path.stem}",
                     compress(content), sha1_bytes(content))
                )
                stats["examples"] += 1
                stats["chunks"] += 1

            # Types
            for kind, name_, s_idx, e_idx in iter_types(src):
                block = src[s_idx:e_idx+1]
                line = src[:s_idx].count("\n") + 1
                content = f"{kind} {name_}\n{block}"[:self.max_chunk]
                conn.execute(
                    "INSERT INTO chunks(kind,periph,file_id,module_id,line_start,line_end,title,content,hash) VALUES(?,?,?,?,?,?,?,?,?)",
                    (KIND_TYPE, periph, file_id, module_id, line, line + block.count("\n"),
                     f"{kind} {name_}", compress(content), sha1_bytes(content))
                )
                stats["types"] += 1
                stats["chunks"] += 1

            # Functions
            for fname, sig, ret, s_idx, e_idx, has_body in iter_funcs(src):
                line = src[:s_idx].count("\n") + 1
                body = ""
                if has_body:
                    masked = _mask_comments(src)
                    brace = masked.find("{", s_idx)
                    if brace >= 0:
                        end = _find_brace(masked, brace)
                        if end:
                            body = src[brace:end+1]
                content = f"{sig}\n{body}"[:self.max_chunk]
                conn.execute(
                    "INSERT INTO chunks(kind,periph,file_id,module_id,line_start,line_end,title,signature,content,hash) VALUES(?,?,?,?,?,?,?,?,?,?)",
                    (KIND_FUNCTION, periph, file_id, module_id, line, line + content.count("\n"),
                     fname, compress(sig), compress(content), sha1_bytes(content))
                )
                conn.execute(
                    "INSERT INTO api(name,periph,file_id,line,signature,ret_type) VALUES(?,?,?,?,?,?)",
                    (fname, periph, file_id, line, compress(sig), compress(ret) if ret else None)
                )
                stats["funcs"] += 1
                stats["chunks"] += 1

        conn.commit()

        if self.enable_fts:
            _fts_rebuild(conn)

        conn.execute("VACUUM")
        conn.close()

        size_kb = out_db.stat().st_size / 1024
        print(f"    [{_fmt(time.time()-t0)}] {name}: {stats['files']} files, {stats['chunks']} chunks ({size_kb:.0f}KB)")
        return out_db


def update_embeddings(db_path: Path, provider: str, api_key: Optional[str], model: str,
                      batch_size: int = 0, kinds: List[int] = None, max_new: int = 0) -> int:
    kinds = kinds or [KIND_FUNCTION, KIND_TYPE, KIND_EXAMPLE]
    embedder = Embedder(provider, api_key, model)
    if batch_size <= 0:
        batch_size = embedder.batch_size

    conn = _db_connect(db_path)
    conn.execute("INSERT OR IGNORE INTO models(name) VALUES(?)", (embedder.model,))
    row = conn.execute("SELECT id FROM models WHERE name=?", (embedder.model,)).fetchone()
    if not row:
        raise RuntimeError(f"Failed to resolve model id for {embedder.model!r}")
    model_id = int(row[0])

    placeholders = ",".join("?" * len(kinds))
    cursor = conn.execute(f"""
        SELECT c.id, c.content FROM chunks c
        WHERE c.kind IN ({placeholders})
        AND NOT EXISTS (SELECT 1 FROM embeddings e WHERE e.chunk_id=c.id AND e.provider=? AND e.model_id=?)
    """, (*kinds, embedder.provider_id, model_id))
    rows = cursor.fetchall()
    if max_new > 0:
        rows = rows[:max_new]

    total = len(rows)
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
        except Exception as e:
            print(f"  [error] {e}")

    conn.close()
    return done


def _fmt(s: float) -> str:
    m, s = divmod(int(s), 60)
    return f"{m:02d}:{s:02d}"


def sanitize(name: str) -> str:
    s = re.sub(r"[^a-z0-9]+", "_", name.strip().lower())
    return re.sub(r"_+", "_", s).strip("_") or "sdk"


def discover_sdks(root: Path, include: Sequence[str], exclude: Sequence[str]) -> List[Tuple[str, Path]]:
    if not root.exists():
        return []
    inc_re = [re.compile(p, re.I) for p in include] if include else []
    exc_re = [re.compile(p, re.I) for p in exclude] if exclude else []
    out = []
    for child in sorted(root.iterdir()):
        if not child.is_dir() or child.name.startswith("."):
            continue
        name = sanitize(child.name)
        if exc_re and any(r.search(name) for r in exc_re):
            continue
        if inc_re and not any(r.search(name) for r in inc_re):
            continue
        out.append((name, child))
    return out


def _detect_root() -> Optional[Path]:
    cwd = Path.cwd()
    if cwd.name == "external":
        return cwd
    for p in [cwd] + list(cwd.parents)[:4]:
        ext = p / "external"
        if ext.exists() and ext.is_dir():
            return ext
    return None


def main():
    p = argparse.ArgumentParser(description="Build SDK indexes v3 (binary)")
    p.add_argument("--sdk", default="")
    p.add_argument("--name", default="")
    p.add_argument("--output", default="")
    p.add_argument("--sdk-root", default="")
    p.add_argument("--output-dir", default="")
    p.add_argument("--include", action="append", default=[])
    p.add_argument("--exclude", action="append", default=[])
    p.add_argument("--skip-if-unchanged", action="store_true")
    p.add_argument("--version", default="dev")
    p.add_argument("--no-fts", action="store_true")
    p.add_argument("--verbose", action="store_true")
    p.add_argument("--update-embeddings", action="store_true")
    p.add_argument("--provider", default="voyage")
    p.add_argument("--api-key", default=os.environ.get("VOYAGE_API_KEY", ""))
    p.add_argument("--model", default="")
    p.add_argument("--batch-size", type=int, default=0)
    p.add_argument("--max-new", type=int, default=0)
    args = p.parse_args()


    # Provider-aware default API key
    if not args.api_key:
        prov = (args.provider or '').strip().lower()
        if prov == 'openai':
            args.api_key = os.environ.get('OPENAI_API_KEY', '')
        elif prov == 'voyage':
            args.api_key = os.environ.get('VOYAGE_API_KEY', '')
    indexer = SDKIndexer(enable_fts=not args.no_fts, verbose=args.verbose)

    if args.sdk:
        sdk_path = Path(args.sdk)
        name = args.name or sanitize(sdk_path.name)
        out_db = Path(args.output) if args.output else (sdk_path.parent / ".extsdk" / f"{name}.db")
        if args.update_embeddings:
            n = update_embeddings(out_db, args.provider, args.api_key or None, args.model, args.batch_size, max_new=args.max_new)
            print(f"✓ {name}: {n} embeddings")
        else:
            indexer.build_sdk_db(sdk_path, name, out_db, args.version, args.skip_if_unchanged)
        return

    sdk_root = Path(args.sdk_root) if args.sdk_root else _detect_root()
    if not sdk_root:
        print("Error: SDK root not found. Use --sdk-root")
        return

    output_dir = Path(args.output_dir) if args.output_dir else (sdk_root / ".extsdk")
    sdks = discover_sdks(sdk_root, args.include, args.exclude)

    if not sdks:
        print(f"No SDKs in {sdk_root}")
        return

    print(f"SDK root: {sdk_root}\nOutput: {output_dir}\nSDKs: {len(sdks)}\n")

    if args.update_embeddings:
        total = 0
        for name, _ in sdks:
            db = output_dir / f"{name}.db"
            if db.exists():
                n = update_embeddings(db, args.provider, args.api_key or None, args.model, args.batch_size, max_new=args.max_new)
                print(f"  {name}: {n}")
                total += n
        print(f"\n✓ Total: {total} embeddings")
    else:
        for name, sdk_path in sdks:
            indexer.build_sdk_db(sdk_path, name, output_dir / f"{name}.db", args.version, args.skip_if_unchanged)
        print(f"\n✓ Indexed {len(sdks)} SDKs")


if __name__ == "__main__":
    main()
