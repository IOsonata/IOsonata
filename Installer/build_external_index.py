#!/usr/bin/env python3
"""
External SDK Index Builder (installer baseline) + Startup Embedding Updater

Intent:
- Installer builds baseline per-SDK DBs (no API keys required).
- Application startup (has API key) updates embeddings in those DBs.

One DB per SDK (traceable), with the same DB contract as the IOsonata repo indexer:
- chunks(uid TEXT UNIQUE, title, content, kind, module, file_path, start_line, end_line, content_hash)
- embeddings(chunk_uid, provider, model, dim, embedding, updated_utc) PK(chunk_uid, provider, model)
- optional FTS5: fts_chunks(rowid=chunks.id, title, content, file_path, kind, module)

Usage:
  # Index all SDKs under auto-detected external/ folder (no embeddings)
  python3 build_external_index.py

  # Index all SDKs under a specific root
  python3 build_external_index.py --sdk-root /path/to/external

  # Index a single SDK
  python3 build_external_index.py --sdk /path/to/nrfx

  # Update embeddings later (requires API key)
  python3 build_external_index.py --update-embeddings --api-key $VOYAGE_API_KEY
"""

from __future__ import annotations

import argparse
import functools
import hashlib
import json
import os
import re
import sqlite3
import struct
import sys
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Dict, Iterator, List, Optional, Sequence, Tuple

# Force unbuffered output for better progress feedback
print = functools.partial(print, flush=True)


# -----------------------------
# Config
# -----------------------------

SCHEMA_VERSION = "3.4"

DEFAULT_IGNORE_DIRS = {
    # Git/IDE
    ".git", ".github", ".iosonata", ".metadata", ".settings", ".vscode", ".idea",
    # Build artifacts
    "build", "out", "dist",
    # Non-embedded platforms
    "OSX", "linux", "win32", "OSC",
    # Python/Node
    "node_modules", "__pycache__", ".pytest_cache",
    # Documentation (large, not code)
    "doc", "docs", "documentation", "doxygen",
}

# Directory name prefixes to ignore
IGNORE_DIR_PREFIXES = ("Debug", "Release", "cmake-build-")

# Include assembly for SDK indexing
SOURCE_SUFFIXES = {".h", ".hpp", ".hh", ".c", ".cc", ".cpp", ".cxx", ".inc", ".inl", ".s", ".S"}

EXAMPLE_DIR_HINTS = ("example", "examples", "sample", "samples", "demo", "demos", "test", "tests", "exemples")


# -----------------------------
# Embedding client with batch support
# -----------------------------

class EmbeddingClient:
    """
    Embedding client supporting batch operations for:
    - voyage (https://api.voyageai.com/v1/embeddings)
    - openai (https://api.openai.com/v1/embeddings)
    - hash  (offline deterministic hashing vectors; low semantic quality but always available)
    """

    BATCH_SIZES = {"voyage": 72, "openai": 100, "hash": 500}
    RATE_LIMITS = {"voyage": 100, "openai": 500, "hash": 10000}

    def __init__(self, provider: str, api_key: Optional[str], model: Optional[str]):
        self.provider = (provider or "voyage").lower().strip()
        self.api_key = api_key
        self.model = model or self._default_model(self.provider)
        self._cache: Dict[str, List[float]] = {}
        self._last_request_time: float = 0

        if self.provider in ("anthropic",):
            self.provider = "voyage"

        if self.provider in ("voyage", "openai") and not self.api_key:
            raise ValueError(f"provider={self.provider} requires --api-key")

    @staticmethod
    def _default_model(provider: str) -> str:
        defaults = {
            "openai": "text-embedding-3-small",
            "voyage": "voyage-code-2",
            "hash": "hash-512",
        }
        return defaults.get(provider, "voyage-code-2")

    @property
    def batch_size(self) -> int:
        return self.BATCH_SIZES.get(self.provider, 32)

    def _rate_limit_wait(self) -> None:
        if self.provider == "hash":
            return
        min_interval = 60.0 / self.RATE_LIMITS.get(self.provider, 100)
        elapsed = time.time() - self._last_request_time
        if elapsed < min_interval:
            time.sleep(min_interval - elapsed)
        self._last_request_time = time.time()

    def embed(self, text: str) -> List[float]:
        results = self.embed_batch([text])
        return results[0] if results else []

    def embed_batch(self, texts: List[str], max_retries: int = 3) -> List[List[float]]:
        if not texts:
            return []

        results: List[Optional[List[float]]] = [None] * len(texts)
        uncached_indices: List[int] = []

        for i, text in enumerate(texts):
            cache_key = self._cache_key(text)
            if cache_key in self._cache:
                results[i] = self._cache[cache_key]
            else:
                uncached_indices.append(i)

        if not uncached_indices:
            return [r if r is not None else [] for r in results]

        uncached_texts = [texts[i] for i in uncached_indices]

        for batch_start in range(0, len(uncached_texts), self.batch_size):
            batch_end = min(batch_start + self.batch_size, len(uncached_texts))
            batch_texts = uncached_texts[batch_start:batch_end]
            batch_indices = uncached_indices[batch_start:batch_end]

            for attempt in range(max_retries):
                try:
                    self._rate_limit_wait()

                    if self.provider == "hash":
                        batch_vecs = [self._embed_hash(t) for t in batch_texts]
                    elif self.provider == "voyage":
                        batch_vecs = self._embed_voyage_batch(batch_texts)
                    elif self.provider == "openai":
                        batch_vecs = self._embed_openai_batch(batch_texts)
                    else:
                        raise ValueError(f"Unknown provider: {self.provider}")

                    for idx, vec, text in zip(batch_indices, batch_vecs, batch_texts):
                        results[idx] = vec
                        if vec:
                            self._cache[self._cache_key(text)] = vec
                    break

                except Exception as e:
                    if attempt < max_retries - 1:
                        wait_time = (2 ** attempt) + 1
                        print(f"  [warn] API error (attempt {attempt + 1}): {e}, retrying in {wait_time}s...")
                        time.sleep(wait_time)
                    else:
                        print(f"  [error] API failed after {max_retries} attempts: {e}")
                        for idx in batch_indices:
                            if results[idx] is None:
                                results[idx] = []

        return [r if r is not None else [] for r in results]

    def _cache_key(self, text: str) -> str:
        return hashlib.md5(
            (self.provider + "|" + self.model + "|" + (text or "")).encode("utf-8", errors="ignore")
        ).hexdigest()

    def _hash_dim(self) -> int:
        m = re.search(r"hash-(\d+)", self.model)
        if m:
            return max(32, min(int(m.group(1)), 4096))
        return 512

    @staticmethod
    def _tokenize(text: str) -> List[str]:
        return re.findall(r"[A-Za-z_]\w+|::|->|==|!=|<=|>=|&&|\|\||[{}()[\];,]", text)

    def _embed_hash(self, text: str) -> List[float]:
        dim = self._hash_dim()
        vec = [0.0] * dim
        for tok in self._tokenize((text or "").lower())[:20000]:
            h = hashlib.blake2b(tok.encode("utf-8", errors="ignore"), digest_size=8).digest()
            idx = int.from_bytes(h[:4], "little") % dim
            sign = 1.0 if (h[4] & 1) == 0 else -1.0
            vec[idx] += sign
        norm = sum(x * x for x in vec) ** 0.5
        if norm > 0:
            vec = [x / norm for x in vec]
        return vec

    def _embed_voyage_batch(self, texts: List[str]) -> List[List[float]]:
        import urllib.request
        url = "https://api.voyageai.com/v1/embeddings"
        headers = {"Authorization": f"Bearer {self.api_key}", "Content-Type": "application/json"}
        truncated = [t[:8000] if t else "" for t in texts]
        payload = {"model": self.model, "input": truncated, "input_type": "document"}
        data = json.dumps(payload).encode("utf-8")
        req = urllib.request.Request(url, data=data, headers=headers, method="POST")
        with urllib.request.urlopen(req, timeout=90) as resp:
            result = json.loads(resp.read().decode("utf-8", errors="replace"))
        embeddings_data = sorted(result.get("data", []), key=lambda x: x.get("index", 0))
        return [item.get("embedding", []) for item in embeddings_data]

    def _embed_openai_batch(self, texts: List[str]) -> List[List[float]]:
        import urllib.request
        url = "https://api.openai.com/v1/embeddings"
        headers = {"Authorization": f"Bearer {self.api_key}", "Content-Type": "application/json"}
        truncated = [t[:8000] if t else "" for t in texts]
        payload = {"model": self.model, "input": truncated}
        data = json.dumps(payload).encode("utf-8")
        req = urllib.request.Request(url, data=data, headers=headers, method="POST")
        with urllib.request.urlopen(req, timeout=90) as resp:
            result = json.loads(resp.read().decode("utf-8", errors="replace"))
        embeddings_data = sorted(result.get("data", []), key=lambda x: x.get("index", 0))
        return [item.get("embedding", []) for item in embeddings_data]


# -----------------------------
# DB layer
# -----------------------------

def _db_connect(db_path: Path) -> sqlite3.Connection:
    conn = sqlite3.connect(str(db_path))
    conn.execute("PRAGMA foreign_keys=ON;")
    conn.execute("PRAGMA journal_mode=DELETE;")
    conn.execute("PRAGMA synchronous=NORMAL;")
    conn.execute("PRAGMA temp_store=MEMORY;")
    return conn


def _ensure_schema(conn: sqlite3.Connection, enable_fts: bool) -> None:
    conn.executescript("""
    CREATE TABLE IF NOT EXISTS metadata_kv (
      k TEXT PRIMARY KEY,
      v TEXT
    );

    CREATE TABLE IF NOT EXISTS chunks (
      id INTEGER PRIMARY KEY,
      uid TEXT NOT NULL UNIQUE,
      kind TEXT NOT NULL,
      title TEXT,
      signature TEXT,
      module TEXT,
      file_path TEXT,
      start_line INTEGER,
      end_line INTEGER,
      content TEXT NOT NULL,
      content_hash TEXT NOT NULL
    );

    CREATE INDEX IF NOT EXISTS idx_chunks_kind ON chunks(kind);
    CREATE INDEX IF NOT EXISTS idx_chunks_file_path ON chunks(file_path);
    CREATE INDEX IF NOT EXISTS idx_chunks_module ON chunks(module);

    CREATE TABLE IF NOT EXISTS embeddings (
      chunk_uid TEXT NOT NULL,
      provider TEXT NOT NULL,
      model TEXT NOT NULL,
      dim INTEGER NOT NULL,
      embedding BLOB NOT NULL,
      updated_utc INTEGER NOT NULL,
      PRIMARY KEY (chunk_uid, provider, model)
    );
    CREATE INDEX IF NOT EXISTS idx_embeddings_chunk_uid ON embeddings(chunk_uid);
    """)

    if enable_fts:
        conn.executescript("""
        CREATE VIRTUAL TABLE IF NOT EXISTS fts_chunks USING fts5(
          title, content, file_path, kind, module
        );
        """)
    conn.commit()


def _set_kv(conn: sqlite3.Connection, key: str, value: str) -> None:
    conn.execute("INSERT OR REPLACE INTO metadata_kv(k,v) VALUES (?,?)", (key, value))


def _get_kv(conn: sqlite3.Connection, key: str) -> str:
    row = conn.execute("SELECT v FROM metadata_kv WHERE k=?", (key,)).fetchone()
    return row[0] if row else ""


def _has_fts(conn: sqlite3.Connection) -> bool:
    row = conn.execute("SELECT name FROM sqlite_master WHERE type='table' AND name='fts_chunks'").fetchone()
    return row is not None


def _fts_rebuild(conn: sqlite3.Connection) -> None:
    conn.execute("DELETE FROM fts_chunks;")
    conn.execute("""
      INSERT INTO fts_chunks(rowid, title, content, file_path, kind, module)
      SELECT id, COALESCE(title,''), content, COALESCE(file_path,''), COALESCE(kind,''), COALESCE(module,'')
      FROM chunks;
    """)
    conn.commit()


def _pack_f32_le(vec: Sequence[float]) -> bytes:
    return struct.pack("<" + "f" * len(vec), *vec)


def _stable_chunk_uid(file: str, kind: str, start_line: int, end_line: int, content_hash: str) -> str:
    base = f"{file}|{kind}|{start_line}|{end_line}|{content_hash}"
    return hashlib.sha1(base.encode("utf-8", errors="ignore")).hexdigest()


# -----------------------------
# Parsing helpers
# -----------------------------

def _safe_read_text(path: Path, max_bytes: int) -> str:
    data = path.read_bytes()
    if len(data) > max_bytes:
        data = data[:max_bytes]
    try:
        return data.decode("utf-8")
    except UnicodeDecodeError:
        return data.decode("latin-1", errors="replace")


def _truncate(s: str, max_chars: int) -> str:
    if len(s) <= max_chars:
        return s
    return s[:max_chars] + "\n/* … truncated … */\n"


def _line_count(s: str) -> int:
    return max(1, s.count("\n") + 1)


def _is_example_path(rel_path: str) -> bool:
    parts = rel_path.lower().split("/")
    return any(h in parts for h in EXAMPLE_DIR_HINTS)


def _is_assembly_file(path: Path) -> bool:
    return path.suffix.lower() in (".s",)


def _should_ignore_dir(dirname: str, ignore_dirs: set, ignore_prefixes: tuple) -> bool:
    """Check if directory should be ignored."""
    if dirname in ignore_dirs:
        return True
    if dirname.startswith("."):
        return True
    if dirname.startswith(ignore_prefixes):
        return True
    return False


def _mask_comments_and_strings(src: str) -> str:
    out = list(src)
    i, n = 0, len(out)

    def repl(j: int, k: int):
        for t in range(j, k):
            if out[t] != "\n":
                out[t] = " "

    while i < n:
        c = out[i]
        if c == "/" and i + 1 < n and out[i + 1] == "/":
            j = i
            i += 2
            while i < n and out[i] != "\n":
                i += 1
            repl(j, i)
            continue
        if c == "/" and i + 1 < n and out[i + 1] == "*":
            j = i
            i += 2
            while i + 1 < n and not (out[i] == "*" and out[i + 1] == "/"):
                i += 1
            i = min(n, i + 2)
            repl(j, i)
            continue
        if c in ("'", '"'):
            quote = c
            j = i
            i += 1
            while i < n:
                if out[i] == "\\":
                    i += 2
                    continue
                if out[i] == quote:
                    i += 1
                    break
                i += 1
            repl(j, i)
            continue
        i += 1
    return "".join(out)


def _find_matching_brace(masked: str, open_index: int) -> Optional[int]:
    depth = 0
    for i in range(open_index, len(masked)):
        ch = masked[i]
        if ch == "{":
            depth += 1
        elif ch == "}":
            depth -= 1
            if depth == 0:
                return i
    return None


_TYPE_HEAD_RE = re.compile(r"\b(class|struct|enum)\s+([A-Za-z_]\w*)[^;{]*\{", re.M)

# SDK-friendly function regex (handles __STATIC_INLINE, IRAM_ATTR, etc.)
_FUNC_HEAD_RE = re.compile(
    r"""(?P<prefix>^[ \t]*(?:static\s+|inline\s+|extern\s+|__STATIC_INLINE\s+|IRAM_ATTR\s+|__attribute__\(\([^\)]*\)\)\s*)*)
        (?P<ret>[A-Za-z_][\w:<>\s\*&]+?)\s+
        (?P<n>[A-Za-z_]\w*)\s*
        \((?P<params>[^;\)]{0,400})\)\s*
        (?P<tail>\{|\;)
    """,
    re.X | re.M,
)


def iter_type_blocks(src: str) -> Iterator[Tuple[str, str, int, int]]:
    masked = _mask_comments_and_strings(src)
    for m in _TYPE_HEAD_RE.finditer(masked):
        kind = m.group(1)
        name = m.group(2)
        brace_open = masked.find("{", m.start())
        if brace_open < 0:
            continue
        brace_close = _find_matching_brace(masked, brace_open)
        if brace_close is None:
            continue
        end = brace_close
        j = end + 1
        while j < len(masked) and masked[j].isspace():
            j += 1
        if j < len(masked) and masked[j] == ";":
            end = j
        yield kind, name, m.start(), end


def iter_function_heads(src: str) -> Iterator[Tuple[str, str, int, int, bool]]:
    masked = _mask_comments_and_strings(src)
    for m in _FUNC_HEAD_RE.finditer(masked):
        name = m.group("n")
        # Skip internal/private functions
        if name.startswith("_"):
            continue
        ret = " ".join(m.group("ret").split())
        params = " ".join(m.group("params").split())
        sig = f"{ret} {name}({params})"
        has_body = m.group("tail") == "{"
        yield name, sig, m.start(), m.end(), has_body


def _brief_comment_before(src: str, start_idx: int) -> str:
    window = src[max(0, start_idx - 2000):start_idx]
    m = re.search(r"/\*\*([\s\S]*?)\*/\s*$", window)
    if m:
        txt = re.sub(r"^\s*\*\s?", "", m.group(1), flags=re.M).strip()
        return re.sub(r"\s+", " ", txt)[:400]
    lines = window.splitlines()
    brief_lines = []
    for line in reversed(lines[-10:]):
        s = line.strip()
        if s.startswith("//"):
            brief_lines.append(s[2:].strip())
            continue
        if s == "":
            continue
        break
    if brief_lines:
        brief_lines.reverse()
        return re.sub(r"\s+", " ", " ".join(brief_lines))[:400]
    return ""


def _relpath(path: Path, root: Path) -> str:
    try:
        return str(path.resolve().relative_to(root.resolve())).replace("\\", "/")
    except Exception:
        return str(path).replace("\\", "/")


def _sdk_fingerprint(root: Path, ignore_dirs: set, ignore_prefixes: tuple) -> str:
    """Fast fingerprint for skip-if-unchanged check."""
    h = hashlib.sha1()
    h.update(str(root.resolve()).encode())
    count = 0
    newest = 0
    for dirpath, dirnames, filenames in os.walk(root):
        dirnames[:] = [d for d in dirnames if not _should_ignore_dir(d, ignore_dirs, ignore_prefixes)]
        for fn in filenames:
            p = Path(dirpath) / fn
            if p.suffix.lower() in SOURCE_SUFFIXES:
                count += 1
                try:
                    mtime = int(p.stat().st_mtime)
                    if mtime > newest:
                        newest = mtime
                except Exception:
                    pass
        if count > 5000:
            break
    h.update(str(count).encode())
    h.update(str(newest).encode())
    return h.hexdigest()


def _fmt_time(seconds: float) -> str:
    m, s = divmod(int(seconds), 60)
    return f"{m:02d}:{s:02d}"


# -----------------------------
# Chunk text builders
# -----------------------------

def _infer_module(rel_path: str) -> str:
    parts = rel_path.split("/")
    return parts[0] if len(parts) >= 2 else "root"


def _type_chunk_text(kind: str, name: str, desc: str, file: str, block: str) -> str:
    parts = [f"{kind} {name}"]
    if desc:
        parts.append(desc)
    parts.append(f"File: {file}")
    parts.append(block)
    return "\n".join(parts)


def _func_chunk_text(sig: str, desc: str, file: str, body: str) -> str:
    parts = [sig]
    if desc:
        parts.append(desc)
    parts.append(f"File: {file}")
    if body:
        parts.append(body)
    return "\n".join(parts)


def _example_chunk_text(file: str, src: str) -> str:
    lines = src.splitlines()
    if len(lines) <= 240:
        return f"Example file: {file}\n" + src
    head = "\n".join(lines[:180])
    tail = "\n".join(lines[-40:])
    return f"Example file: {file}\n" + head + "\n/* … snip … */\n" + tail


def _asm_chunk_text(file: str, src: str) -> str:
    """Create a chunk for assembly files (just store first N lines for search)."""
    lines = src.splitlines()[:100]
    return f"Assembly file: {file}\n" + "\n".join(lines)


def _insert_chunk(
    conn: sqlite3.Connection,
    uid: str,
    kind: str,
    title: str,
    signature: str,
    module: str,
    file_path: str,
    start_line: int,
    end_line: int,
    content: str,
    content_hash: str,
) -> None:
    conn.execute(
        """INSERT OR REPLACE INTO chunks
           (uid,kind,title,signature,module,file_path,start_line,end_line,content,content_hash)
           VALUES (?,?,?,?,?,?,?,?,?,?)""",
        (uid, kind, title, signature, module, file_path, start_line, end_line, content, content_hash),
    )


# -----------------------------
# SDK Indexer
# -----------------------------

@dataclass
class BuildStats:
    files: int = 0
    chunks: int = 0
    examples: int = 0
    types: int = 0
    functions: int = 0
    assembly: int = 0


class SDKIndexer:
    def __init__(
        self,
        sdk_root: Path,
        *,
        enable_fts: bool = True,
        ignore_dirs: Sequence[str] = (),
        max_file_kb: int = 1024,
        max_chunk_chars: int = 8000,
        example_cap: int = 250,
        verbose: bool = False,
    ):
        self.sdk_root = sdk_root
        self.enable_fts = enable_fts
        self.ignore_dirs = set(DEFAULT_IGNORE_DIRS) | set(ignore_dirs)
        self.ignore_prefixes = IGNORE_DIR_PREFIXES
        self.max_file_bytes = max(64 * 1024, max_file_kb * 1024)
        self.max_chunk_chars = max(1000, max_chunk_chars)
        self.example_cap = max(0, example_cap)
        self.verbose = verbose

    def _iter_source_files(self, root: Path) -> Iterator[Path]:
        for dirpath, dirnames, filenames in os.walk(root):
            dirnames[:] = [d for d in dirnames if not _should_ignore_dir(d, self.ignore_dirs, self.ignore_prefixes)]
            for fn in filenames:
                p = Path(dirpath) / fn
                if p.suffix.lower() in SOURCE_SUFFIXES:
                    yield p

    def build_sdk_db(self, sdk_path: Path, name: str, out_db: Path, version: str, skip_if_unchanged: bool) -> Path:
        out_db.parent.mkdir(parents=True, exist_ok=True)

        fingerprint = _sdk_fingerprint(sdk_path, self.ignore_dirs, self.ignore_prefixes)
        cfg = {
            "schema_version": SCHEMA_VERSION,
            "sdk_name": name,
            "sdk_path": str(sdk_path.resolve()),
            "enable_fts": self.enable_fts,
            "max_file_bytes": self.max_file_bytes,
            "max_chunk_chars": self.max_chunk_chars,
            "example_cap": self.example_cap,
        }
        cfg_hash = hashlib.sha1(json.dumps(cfg, sort_keys=True).encode("utf-8")).hexdigest()

        if skip_if_unchanged and out_db.exists():
            try:
                conn = _db_connect(out_db)
                _ensure_schema(conn, enable_fts=self.enable_fts)
                prev_fp = _get_kv(conn, "sdk_fingerprint")
                prev_ch = _get_kv(conn, "config_hash")
                conn.close()
                if prev_fp == fingerprint and prev_ch == cfg_hash:
                    print(f"  - {name}: unchanged; skipping")
                    return out_db
            except Exception:
                pass

        if out_db.exists():
            out_db.unlink()

        t0 = time.time()
        print(f"  - {name}: indexing...")

        conn = _db_connect(out_db)
        _ensure_schema(conn, enable_fts=self.enable_fts)

        _set_kv(conn, "schema_version", SCHEMA_VERSION)
        _set_kv(conn, "version", version)
        _set_kv(conn, "built_utc", datetime.now(timezone.utc).isoformat())
        _set_kv(conn, "sdk_name", name)
        _set_kv(conn, "sdk_path", str(sdk_path.resolve()))
        _set_kv(conn, "sdk_fingerprint", fingerprint)
        _set_kv(conn, "config_hash", cfg_hash)
        _set_kv(conn, "config_json", json.dumps(cfg, sort_keys=True))

        stats = BuildStats()
        last_progress = time.time()

        for path in self._iter_source_files(sdk_path):
            stats.files += 1
            rel = _relpath(path, sdk_path)
            module = _infer_module(rel)

            # Progress every 50 files or every 3 seconds
            now = time.time()
            if stats.files % 50 == 0 or (now - last_progress) > 3:
                elapsed = now - t0
                print(f"    [{_fmt_time(elapsed)}] {name}: files={stats.files} chunks={stats.chunks}")
                last_progress = now

            try:
                src = _safe_read_text(path, self.max_file_bytes)
            except Exception:
                continue

            # Handle assembly files separately
            if _is_assembly_file(path):
                content = _truncate(_asm_chunk_text(rel, src), self.max_chunk_chars)
                chash = hashlib.sha1(content.encode("utf-8", errors="ignore")).hexdigest()
                uid = _stable_chunk_uid(rel, "assembly", 1, _line_count(content), chash)
                _insert_chunk(conn, uid, "assembly", f"Assembly: {path.stem}", "", module, rel, 1, _line_count(content), content, chash)
                stats.assembly += 1
                stats.chunks += 1
                continue

            # Examples
            if self.example_cap and _is_example_path(rel):
                if stats.examples < self.example_cap:
                    content = _truncate(_example_chunk_text(rel, src), self.max_chunk_chars)
                    chash = hashlib.sha1(content.encode("utf-8", errors="ignore")).hexdigest()
                    uid = _stable_chunk_uid(rel, "example", 1, _line_count(content), chash)
                    _insert_chunk(conn, uid, "example", f"Example: {Path(rel).stem}", "", module, rel, 1, _line_count(content), content, chash)
                    stats.examples += 1
                    stats.chunks += 1

            # Types
            for kind, tname, s_idx, e_idx in iter_type_blocks(src):
                block = src[s_idx:e_idx+1]
                line = src[:s_idx].count("\n") + 1
                desc = _brief_comment_before(src, s_idx)
                content = _truncate(_type_chunk_text(kind, tname, desc, rel, block), self.max_chunk_chars)
                chash = hashlib.sha1(content.encode("utf-8", errors="ignore")).hexdigest()
                uid = _stable_chunk_uid(rel, "type", line, line + content.count("\n"), chash)
                _insert_chunk(conn, uid, "type", f"{kind} {tname}", "", module, rel, line, line + content.count("\n"), content, chash)
                stats.types += 1
                stats.chunks += 1

            # Functions
            for fname, sig, s_idx, e_idx, has_body in iter_function_heads(src):
                line = src[:s_idx].count("\n") + 1
                desc = _brief_comment_before(src, s_idx)
                body_text = ""
                if has_body:
                    masked = _mask_comments_and_strings(src)
                    brace_open = masked.find("{", e_idx - 1)
                    if brace_open != -1:
                        brace_close = _find_matching_brace(masked, brace_open)
                        if brace_close:
                            body_text = src[brace_open:brace_close+1]
                content = _truncate(_func_chunk_text(sig, desc, rel, body_text), self.max_chunk_chars)
                chash = hashlib.sha1(content.encode("utf-8", errors="ignore")).hexdigest()
                uid = _stable_chunk_uid(rel, "function", line, line + content.count("\n"), chash)
                _insert_chunk(conn, uid, "function", fname, sig, module, rel, line, line + content.count("\n"), content, chash)
                stats.functions += 1
                stats.chunks += 1

            if stats.files % 100 == 0:
                conn.commit()

        conn.commit()

        if self.enable_fts:
            _fts_rebuild(conn)

        _set_kv(conn, "files_indexed", str(stats.files))
        _set_kv(conn, "chunks_indexed", str(stats.chunks))
        _set_kv(conn, "functions_indexed", str(stats.functions))
        _set_kv(conn, "types_indexed", str(stats.types))
        _set_kv(conn, "examples_indexed", str(stats.examples))
        _set_kv(conn, "assembly_indexed", str(stats.assembly))
        conn.commit()
        conn.close()

        elapsed = time.time() - t0
        size_kb = out_db.stat().st_size // 1024
        print(f"    ✓ {name}: {stats.files} files, {stats.chunks} chunks, {size_kb} KB ({elapsed:.1f}s)")
        return out_db

    def update_embeddings_all(
        self,
        output_dir: Path,
        provider: str,
        api_key: Optional[str],
        model: Optional[str],
        *,
        batch_size: int = 0,
        kinds: Sequence[str] = ("function", "type", "example"),
        max_new_per_db: int = 0,
        verbose: bool = False,
    ) -> int:
        client = EmbeddingClient(provider=provider, api_key=api_key, model=model)

        dbs = sorted(output_dir.glob("*.db"))
        if not dbs:
            print(f"No .db files found in: {output_dir}")
            return 0

        total_done = 0
        for db_path in dbs:
            done = update_embeddings_db(
                db_path=db_path,
                client=client,
                batch_size=batch_size,
                kinds=kinds,
                max_new=max_new_per_db,
                verbose=verbose,
            )
            total_done += done
        return total_done


def update_embeddings_db(
    *,
    db_path: Path,
    client: EmbeddingClient,
    batch_size: int,
    kinds: Sequence[str],
    max_new: int,
    verbose: bool,
) -> int:
    """Update embeddings in a single DB using batch API calls."""
    conn = _db_connect(db_path)
    _ensure_schema(conn, enable_fts=_has_fts(conn))

    provider = client.provider
    model = client.model

    if batch_size <= 0:
        batch_size = client.batch_size

    kind_placeholders = ",".join("?" for _ in kinds)
    limit_clause = f"LIMIT {int(max_new)}" if max_new and max_new > 0 else ""
    sql = f"""
      SELECT uid, COALESCE(title,''), COALESCE(signature,''), COALESCE(module,''), COALESCE(file_path,''), content
      FROM chunks
      WHERE kind IN ({kind_placeholders})
        AND uid NOT IN (SELECT chunk_uid FROM embeddings WHERE provider=? AND model=?)
      {limit_clause}
    """
    rows = conn.execute(sql, tuple(kinds) + (provider, model)).fetchall()
    total = len(rows)

    if total == 0:
        conn.close()
        return 0

    print(f"[embed] {db_path.name}: {total} missing")

    def build_text(title: str, sig: str, module: str, file_path: str, content: str) -> str:
        parts = []
        if title:
            parts.append(title)
        if sig:
            parts.append(sig)
        if module or file_path:
            parts.append(f"Module: {module}  File: {file_path}")
        parts.append(content)
        return "\n".join(parts)[:8000]

    t0 = time.time()
    done = 0
    failed = 0

    for batch_start in range(0, total, batch_size):
        batch_end = min(batch_start + batch_size, total)
        batch_rows = rows[batch_start:batch_end]

        batch_uids = [row[0] for row in batch_rows]
        batch_texts = [build_text(row[1], row[2], row[3], row[4], row[5]) for row in batch_rows]

        batch_vecs = client.embed_batch(batch_texts)

        now_utc = int(time.time())
        for uid, vec in zip(batch_uids, batch_vecs):
            if vec:
                blob = _pack_f32_le(vec)
                conn.execute(
                    "INSERT OR REPLACE INTO embeddings(chunk_uid,provider,model,dim,embedding,updated_utc) VALUES (?,?,?,?,?,?)",
                    (uid, provider, model, len(vec), blob, now_utc),
                )
                done += 1
            else:
                failed += 1

        conn.commit()

        elapsed = time.time() - t0
        pct = 100 * batch_end / total
        print(f"  [{_fmt_time(elapsed)}] {db_path.name}: {batch_end}/{total} ({pct:.0f}%)")

    _set_kv(conn, f"embeddings_{provider}_{model}_updated_utc", str(int(time.time())))
    conn.commit()
    conn.close()

    elapsed = time.time() - t0
    rate = done / elapsed if elapsed > 0 else 0
    print(f"  ✓ {db_path.name}: {done} embeddings ({rate:.1f}/s)")
    if failed and verbose:
        print(f"    [warn] failed: {failed}")

    return done


# -----------------------------
# CLI helpers
# -----------------------------

def _default_sdk_root() -> Optional[str]:
    for k in ("IOCOMPOSER_SDK_ROOT", "SDK_ROOT"):
        v = os.environ.get(k)
        if v:
            return v
    return None


def _default_index_dir() -> Optional[str]:
    for k in ("IOCOMPOSER_INDEX_DIR", "INDEX_DIR"):
        v = os.environ.get(k)
        if v:
            return v
    return None


def sanitize_name(name: str) -> str:
    s = name.strip().lower()
    s = re.sub(r"[^a-z0-9]+", "_", s)
    s = re.sub(r"_+", "_", s).strip("_")
    return s or "sdk"


def discover_sdks(sdk_root: Path, include: Sequence[str], exclude: Sequence[str]) -> List[Tuple[str, Path]]:
    if not sdk_root.exists():
        return []
    inc_re = [re.compile(p, re.I) for p in include] if include else []
    exc_re = [re.compile(p, re.I) for p in exclude] if exclude else []

    out: List[Tuple[str, Path]] = []
    for child in sorted(sdk_root.iterdir()):
        if not child.is_dir():
            continue
        if child.name.startswith("."):
            continue
        name = sanitize_name(child.name)
        if exc_re and any(r.search(name) for r in exc_re):
            continue
        if inc_re and not any(r.search(name) for r in inc_re):
            continue
        out.append((name, child))
    return out


# -----------------------------
# CLI
# -----------------------------

def _detect_external_sdk_root() -> Optional[Path]:
    """Auto-detect external SDK root by looking for common patterns."""
    cwd = Path.cwd()
    
    # Check if current directory is named 'external' and contains SDK-like subdirs
    if cwd.name == "external":
        subdirs = [d for d in cwd.iterdir() if d.is_dir() and not d.name.startswith(".")]
        if subdirs:
            return cwd
    
    # Check for IOcomposer/external pattern
    for parent in [cwd] + list(cwd.parents)[:4]:
        external_dir = parent / "external"
        if external_dir.exists() and external_dir.is_dir():
            subdirs = [d for d in external_dir.iterdir() if d.is_dir() and not d.name.startswith(".")]
            if subdirs:
                return external_dir
        
        # Also check if parent is IOcomposer-like (has IOsonata sibling)
        iosonata_dir = parent / "IOsonata"
        if iosonata_dir.exists() and external_dir.exists():
            return external_dir
    
    # Check current directory for SDK-like subdirectories
    subdirs = [d for d in cwd.iterdir() if d.is_dir() and not d.name.startswith(".")]
    sdk_hints = ["nrf", "sdk", "hal", "driver", "cmsis", "freertos", "stm"]
    if any(any(hint in d.name.lower() for hint in sdk_hints) for d in subdirs):
        return cwd
    
    return None


def main() -> None:
    p = argparse.ArgumentParser(
        description="Build per-SDK RAG indexes (base index without embeddings).",
        epilog="""
Examples:
  # Index all SDKs under auto-detected external/ folder
  python3 build_external_index.py

  # Index all SDKs under a specific root
  python3 build_external_index.py --sdk-root /path/to/external

  # Index a single SDK
  python3 build_external_index.py --sdk /path/to/nrfx

  # Update embeddings later (requires API key)
  python3 build_external_index.py --update-embeddings --api-key $VOYAGE_API_KEY
        """,
        formatter_class=argparse.RawDescriptionHelpFormatter
    )

    # Build mode: one SDK or all SDKs
    p.add_argument("--sdk", default="", help="Path to a single SDK to index")
    p.add_argument("--name", default="", help="SDK name (auto-detect from folder name if not specified)")
    p.add_argument("--output", default="", help="Output DB path (single SDK mode)")

    p.add_argument("--sdk-root", default="", help="Root folder containing SDK folders (auto-detect if not specified)")
    p.add_argument("--output-dir", default="", help="Output directory for .db files (default: <sdk-root>/.extsdk)")
    p.add_argument("--include", action="append", default=[], help="Regex filter for SDK names to include (repeatable)")
    p.add_argument("--exclude", action="append", default=[], help="Regex filter for SDK names to exclude (repeatable)")
    p.add_argument("--skip-if-unchanged", action="store_true", help="Skip indexing if fingerprint unchanged")

    # Shared options
    p.add_argument("--version", default="dev", help="Version string")
    p.add_argument("--no-fts", action="store_true", help="Disable FTS5 index")
    p.add_argument("--ignore-dir", action="append", default=[], help="Additional directory name to ignore (repeatable)")
    p.add_argument("--max-file-kb", type=int, default=1024, help="Max KB per file to index (default: 1024)")
    p.add_argument("--max-chunk-chars", type=int, default=8000, help="Max chars per chunk (default: 8000)")
    p.add_argument("--example-cap", type=int, default=250, help="Max example files per SDK, 0=disable (default: 250)")
    p.add_argument("--verbose", action="store_true", help="Verbose logs")

    # Embedding update mode (separate operation, requires API key)
    p.add_argument("--update-embeddings", action="store_true", help="Update embeddings in existing DB(s) (requires --api-key)")
    p.add_argument("--provider", default="voyage", help="Embedding provider: voyage|openai|hash (default: voyage)")
    p.add_argument("--api-key", default=os.environ.get("VOYAGE_API_KEY") or os.environ.get("OPENAI_API_KEY") or "", help="API key for provider")
    p.add_argument("--model", default="", help="Embedding model (provider default if empty)")
    p.add_argument("--batch-size", type=int, default=0, help="Embedding batch size (0=provider default)")
    p.add_argument("--max-new", type=int, default=0, help="Max new embeddings per DB (0=all)")
    p.add_argument("--kinds", default="function,type,example", help="Comma list of chunk kinds to embed")

    args = p.parse_args()

    # Auto-detect SDK root if not specified and not in single-SDK mode
    if not args.sdk and not args.sdk_root:
        detected = _detect_external_sdk_root()
        if detected:
            sdk_root = detected
            print(f"[auto-detect] Found external SDK root: {sdk_root}")
        else:
            # Check environment variable as fallback
            env_root = _default_sdk_root()
            if env_root:
                sdk_root = Path(env_root)
                print(f"[env] Using SDK root from environment: {sdk_root}")
            else:
                print("ERROR: Could not auto-detect SDK root.", file=sys.stderr)
                print("  Specify --sdk-root or --sdk, or set IOCOMPOSER_SDK_ROOT environment variable.", file=sys.stderr)
                sys.exit(2)
    elif args.sdk_root:
        sdk_root = Path(args.sdk_root)
    else:
        sdk_root = Path(".")  # Will be overridden in single-SDK mode

    enable_fts = not args.no_fts
    indexer = SDKIndexer(
        sdk_root=sdk_root,
        enable_fts=enable_fts,
        ignore_dirs=args.ignore_dir,
        max_file_kb=args.max_file_kb,
        max_chunk_chars=args.max_chunk_chars,
        example_cap=args.example_cap,
        verbose=args.verbose,
    )

    # Handle embedding update mode
    if args.update_embeddings:
        kinds = [k.strip() for k in args.kinds.split(",") if k.strip()]
        if args.output:
            client = EmbeddingClient(provider=args.provider, api_key=args.api_key or None, model=args.model or None)
            done = update_embeddings_db(
                db_path=Path(args.output),
                client=client,
                batch_size=args.batch_size,
                kinds=kinds,
                max_new=args.max_new,
                verbose=args.verbose,
            )
            print(f"✓ updated embeddings: {done}")
            return

        out_dir = Path(args.output_dir) if args.output_dir else (sdk_root / ".extsdk")
        done = indexer.update_embeddings_all(
            output_dir=out_dir,
            provider=args.provider,
            api_key=args.api_key or None,
            model=args.model or None,
            batch_size=args.batch_size,
            kinds=kinds,
            max_new_per_db=args.max_new,
            verbose=args.verbose,
        )
        print(f"✓ updated embeddings across SDK DBs: {done}")
        return

    # Single SDK build mode
    if args.sdk:
        sdk_path = Path(args.sdk)
        name = args.name or sanitize_name(sdk_path.name)
        if args.output:
            out_db = Path(args.output)
        elif args.output_dir:
            out_db = Path(args.output_dir) / f"{name}.db"
        else:
            out_db = sdk_path.parent / ".extsdk" / f"{name}.db"
        out_db.parent.mkdir(parents=True, exist_ok=True)
        
        print(f"Building external SDK index:")
        print(f"  SDK:       {sdk_path.resolve()}")
        print(f"  Name:      {name}")
        print(f"  Output:    {out_db.resolve()}")
        print(f"  FTS5:      {'enabled' if enable_fts else 'disabled'}")
        print(f"  Examples:  {args.example_cap if args.example_cap > 0 else 'disabled'}")
        print(f"  Embedding: skipped (use --update-embeddings with --api-key to add later)")
        print()
        
        indexer.build_sdk_db(sdk_path, name, out_db, args.version, args.skip_if_unchanged)
        return

    # Multi-SDK build mode
    out_dir = Path(args.output_dir) if args.output_dir else (sdk_root / ".extsdk")
    out_dir.mkdir(parents=True, exist_ok=True)

    sdks = discover_sdks(sdk_root, include=args.include, exclude=args.exclude)
    if not sdks:
        print(f"No SDK folders found under: {sdk_root}", file=sys.stderr)
        sys.exit(1)

    print(f"Building external SDK indexes:")
    print(f"  SDK root:  {sdk_root.resolve()}")
    print(f"  Output:    {out_dir.resolve()}")
    print(f"  SDKs:      {len(sdks)} found ({', '.join(name for name, _ in sdks)})")
    print(f"  FTS5:      {'enabled' if enable_fts else 'disabled'}")
    print(f"  Examples:  {args.example_cap if args.example_cap > 0 else 'disabled'} per SDK")
    print(f"  Embedding: skipped (use --update-embeddings with --api-key to add later)")
    print()

    for name, path in sdks:
        out_db = out_dir / f"{name}.db"
        indexer.build_sdk_db(path, name, out_db, args.version, args.skip_if_unchanged)


if __name__ == "__main__":
    main()
