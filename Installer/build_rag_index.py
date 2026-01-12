#!/usr/bin/env python3
"""
IOsonata Repo Index Builder (base index) + Startup Embedding Updater

Design intent:
- GitHub Actions builds the *base* index for the IOsonata repo (no API keys required).
- Application startup (has API key) runs *embedding update* against the existing DB.

This script supports both:
1) Build base index: parses repo, creates searchable chunks + FTS.
2) Update embeddings only: embeds any chunks missing embeddings for (provider, model).

DB contract (shared with SDK indexer):
- chunks(uid TEXT UNIQUE, title, content, kind, module, file, start_line, end_line, content_hash)
- embeddings(chunk_uid, provider, model, dim, embedding, updated_utc) PK(chunk_uid, provider, model)
- optional FTS5: fts_chunks(rowid=chunks.id, title, content, file, kind, module)

Usage:
  # Base build (GitHub)
  python3 build_rag_index.py --source-dir . --output-dir .iosonata --version 1.0.0

  # Embedding update at app start
  python3 build_rag_index.py --update-embeddings --db .iosonata/index.db --provider voyage --api-key $VOYAGE_API_KEY
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
import subprocess
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

# Directories to ignore completely
DEFAULT_IGNORE_DIRS = {
    # Git/IDE
    ".git", ".github", ".iosonata", ".metadata", ".settings", ".vscode", ".idea",
    # Build artifacts
    "build", "out", "dist",
    # Non-embedded platforms (not relevant for IOsonata embedded)
    "OSX", "linux", "win32", "OSC",
    # Python/Node
    "node_modules", "__pycache__", ".pytest_cache",
}

# Directory name prefixes to ignore (Debug*, Release*, cmake-build-*, etc.)
IGNORE_DIR_PREFIXES = ("Debug", "Release", "cmake-build-")

SOURCE_SUFFIXES = {".h", ".hpp", ".hh", ".c", ".cc", ".cpp", ".cxx", ".inc", ".inl"}

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

    # Batch sizes per provider (API limits)
    BATCH_SIZES = {"voyage": 72, "openai": 100, "hash": 500}
    
    # Rate limits (requests per minute) - conservative
    RATE_LIMITS = {"voyage": 100, "openai": 500, "hash": 10000}

    def __init__(self, provider: str, api_key: Optional[str], model: Optional[str]):
        self.provider = (provider or "voyage").lower().strip()
        self.api_key = api_key
        self.model = model or self._default_model(self.provider)
        self._cache: Dict[str, List[float]] = {}
        self._last_request_time: float = 0

        # backward compatibility aliases
        if self.provider in ("anthropic",):
            self.provider = "voyage"

        if self.provider in ("voyage", "openai") and not self.api_key:
            raise ValueError(f"provider={self.provider} requires --api-key (or env var)")

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
        """Enforce rate limiting between API calls."""
        if self.provider == "hash":
            return
        min_interval = 60.0 / self.RATE_LIMITS.get(self.provider, 100)
        elapsed = time.time() - self._last_request_time
        if elapsed < min_interval:
            time.sleep(min_interval - elapsed)
        self._last_request_time = time.time()

    def embed(self, text: str) -> List[float]:
        """Embed a single text. Use embed_batch for multiple texts."""
        results = self.embed_batch([text])
        return results[0] if results else []

    def embed_batch(self, texts: List[str], max_retries: int = 3) -> List[List[float]]:
        """
        Embed multiple texts in a single API call (or multiple calls if exceeds batch size).
        Returns list of embeddings in same order as input texts.
        Failed embeddings return empty lists.
        """
        if not texts:
            return []

        # Check cache first
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

        # Process uncached texts in batches
        uncached_texts = [texts[i] for i in uncached_indices]
        
        for batch_start in range(0, len(uncached_texts), self.batch_size):
            batch_end = min(batch_start + self.batch_size, len(uncached_texts))
            batch_texts = uncached_texts[batch_start:batch_end]
            batch_indices = uncached_indices[batch_start:batch_end]

            # Retry logic for API calls
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

                    # Store results and cache
                    for idx, vec, text in zip(batch_indices, batch_vecs, batch_texts):
                        results[idx] = vec
                        if vec:
                            self._cache[self._cache_key(text)] = vec
                    break

                except Exception as e:
                    if attempt < max_retries - 1:
                        wait_time = (2 ** attempt) + 1  # Exponential backoff
                        print(f"  [warn] API error (attempt {attempt + 1}): {e}, retrying in {wait_time}s...")
                        time.sleep(wait_time)
                    else:
                        print(f"  [error] API failed after {max_retries} attempts: {e}")
                        # Mark batch as failed (empty vectors)
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
            d = int(m.group(1))
            return max(32, min(d, 4096))
        return 512

    @staticmethod
    def _tokenize(text: str) -> List[str]:
        return re.findall(r"[A-Za-z_]\w+|::|->|==|!=|<=|>=|&&|\|\||[{}()[\];,]", text)

    def _embed_hash(self, text: str) -> List[float]:
        """Feature hashing - deterministic, not semantic, but always available."""
        dim = self._hash_dim()
        vec = [0.0] * dim
        for tok in self._tokenize((text or "").lower())[:20000]:
            h = hashlib.blake2b(tok.encode("utf-8", errors="ignore"), digest_size=8).digest()
            idx = int.from_bytes(h[:4], "little") % dim
            sign = 1.0 if (h[4] & 1) == 0 else -1.0
            vec[idx] += sign
        # L2 normalize
        norm = sum(x * x for x in vec) ** 0.5
        if norm > 0:
            vec = [x / norm for x in vec]
        return vec

    def _embed_voyage_batch(self, texts: List[str]) -> List[List[float]]:
        """Batch embedding via Voyage API."""
        import urllib.request
        import urllib.error

        url = "https://api.voyageai.com/v1/embeddings"
        headers = {
            "Authorization": f"Bearer {self.api_key}",
            "Content-Type": "application/json",
        }
        # Truncate each text and prepare batch
        truncated = [t[:8000] if t else "" for t in texts]
        payload = {
            "model": self.model,
            "input": truncated,
            "input_type": "document",
        }
        data = json.dumps(payload).encode("utf-8")
        req = urllib.request.Request(url, data=data, headers=headers, method="POST")
        
        with urllib.request.urlopen(req, timeout=90) as resp:
            result = json.loads(resp.read().decode("utf-8", errors="replace"))
        
        # Sort by index to maintain order
        embeddings_data = sorted(result.get("data", []), key=lambda x: x.get("index", 0))
        return [item.get("embedding", []) for item in embeddings_data]

    def _embed_openai_batch(self, texts: List[str]) -> List[List[float]]:
        """Batch embedding via OpenAI API."""
        import urllib.request
        import urllib.error

        url = "https://api.openai.com/v1/embeddings"
        headers = {
            "Authorization": f"Bearer {self.api_key}",
            "Content-Type": "application/json",
        }
        truncated = [t[:8000] if t else "" for t in texts]
        payload = {
            "model": self.model,
            "input": truncated,
        }
        data = json.dumps(payload).encode("utf-8")
        req = urllib.request.Request(url, data=data, headers=headers, method="POST")
        
        with urllib.request.urlopen(req, timeout=90) as resp:
            result = json.loads(resp.read().decode("utf-8", errors="replace"))
        
        embeddings_data = sorted(result.get("data", []), key=lambda x: x.get("index", 0))
        return [item.get("embedding", []) for item in embeddings_data]


# -----------------------------
# Parsing helpers
# -----------------------------

def _relpath(path: Path, root: Path) -> str:
    try:
        return str(path.resolve().relative_to(root.resolve())).replace("\\", "/")
    except Exception:
        return str(path).replace("\\", "/")


def _safe_read_text(path: Path, max_bytes: int) -> str:
    data = path.read_bytes()
    if len(data) > max_bytes:
        data = data[:max_bytes]
    try:
        return data.decode("utf-8")
    except UnicodeDecodeError:
        return data.decode("latin-1", errors="replace")


def _mask_comments_and_strings(src: str) -> str:
    """Replace comments and string literals with spaces (preserve newlines and length)."""
    out = list(src)
    i, n = 0, len(out)

    def repl(j: int, k: int):
        for t in range(j, k):
            if out[t] != "\n":
                out[t] = " "

    while i < n:
        c = out[i]
        # line comment
        if c == "/" and i + 1 < n and out[i + 1] == "/":
            j = i
            i += 2
            while i < n and out[i] != "\n":
                i += 1
            repl(j, i)
            continue
        # block comment
        if c == "/" and i + 1 < n and out[i + 1] == "*":
            j = i
            i += 2
            while i + 1 < n and not (out[i] == "*" and out[i + 1] == "/"):
                i += 1
            i = min(n, i + 2)
            repl(j, i)
            continue
        # strings/chars
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

_FUNC_HEAD_RE = re.compile(
    r"""(?P<prefix>^[ \t]*(?:template\s*<[^>]*>\s*)?(?:static\s+|inline\s+|extern\s+|constexpr\s+|virtual\s+)?)
        (?P<ret>[A-Za-z_][\w:<>\s\*&]+?)\s+
        (?P<n>(?:[A-Za-z_]\w*::)*[A-Za-z_]\w*)\s*
        \((?P<params>[^;\)]{0,400})\)\s*
        (?P<tail>\{|\;)
    """,
    re.X | re.M,
)


def iter_type_blocks(src: str) -> Iterator[Tuple[str, str, int, int]]:
    """Yields: (kind, name, start_idx, end_idx_inclusive)"""
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
    """Yields: (name, signature, start_idx, end_idx, has_body)"""
    masked = _mask_comments_and_strings(src)
    for m in _FUNC_HEAD_RE.finditer(masked):
        name = m.group("n")
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
      key TEXT PRIMARY KEY,
      value TEXT NOT NULL
    );

    CREATE TABLE IF NOT EXISTS functions (
      id INTEGER PRIMARY KEY,
      name TEXT,
      signature TEXT,
      module TEXT,
      file TEXT,
      line INTEGER,
      description TEXT
    );

    CREATE TABLE IF NOT EXISTS types (
      id INTEGER PRIMARY KEY,
      kind TEXT,
      name TEXT,
      module TEXT,
      file TEXT,
      line INTEGER,
      description TEXT
    );

    CREATE TABLE IF NOT EXISTS examples (
      id INTEGER PRIMARY KEY,
      name TEXT,
      module TEXT,
      file TEXT,
      line INTEGER,
      description TEXT
    );

    CREATE TABLE IF NOT EXISTS chunks (
      id INTEGER PRIMARY KEY,
      uid TEXT NOT NULL UNIQUE,
      kind TEXT NOT NULL,
      title TEXT,
      signature TEXT,
      module TEXT,
      file TEXT,
      start_line INTEGER,
      end_line INTEGER,
      content TEXT NOT NULL,
      content_hash TEXT NOT NULL,
      entity_type TEXT,
      entity_id INTEGER
    );

    CREATE INDEX IF NOT EXISTS idx_chunks_kind ON chunks(kind);
    CREATE INDEX IF NOT EXISTS idx_chunks_file ON chunks(file);
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
          title, content, file, kind, module
        );
        """)
    conn.commit()


def _fts_rebuild(conn: sqlite3.Connection) -> None:
    conn.execute("DELETE FROM fts_chunks;")
    conn.execute("""
      INSERT INTO fts_chunks(rowid, title, content, file, kind, module)
      SELECT id, COALESCE(title,''), content, COALESCE(file,''), COALESCE(kind,''), COALESCE(module,'')
      FROM chunks;
    """)
    conn.commit()


def _stable_chunk_uid(file: str, kind: str, start_line: int, end_line: int, content_hash: str) -> str:
    base = f"{file}|{kind}|{start_line}|{end_line}|{content_hash}"
    return hashlib.sha1(base.encode("utf-8", errors="ignore")).hexdigest()


def _pack_f32_le(vec: Sequence[float]) -> bytes:
    return struct.pack("<" + "f" * len(vec), *vec)


def _set_kv(conn: sqlite3.Connection, key: str, value: str) -> None:
    conn.execute("INSERT OR REPLACE INTO metadata_kv(key,value) VALUES (?,?)", (key, value))


def _get_kv(conn: sqlite3.Connection, key: str) -> str:
    row = conn.execute("SELECT value FROM metadata_kv WHERE key=?", (key,)).fetchone()
    return row[0] if row else ""


def _has_fts(conn: sqlite3.Connection) -> bool:
    row = conn.execute("SELECT name FROM sqlite_master WHERE type='table' AND name='fts_chunks'").fetchone()
    return row is not None


# -----------------------------
# Helpers
# -----------------------------

def _infer_module(rel_path: str) -> str:
    """Infer module from file path - uses first meaningful directory."""
    parts = rel_path.split("/")
    if len(parts) >= 2:
        # Skip common prefixes, return meaningful module
        return parts[0]
    return "core"


def _is_example_path(rel_path: str) -> bool:
    low = rel_path.lower()
    return any(h in low.split("/") for h in EXAMPLE_DIR_HINTS)


def _truncate(s: str, max_chars: int) -> str:
    if len(s) <= max_chars:
        return s
    return s[:max_chars] + "\n/* … truncated … */\n"


def _line_count(s: str) -> int:
    return max(1, s.count("\n") + 1)


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


def _insert_chunk(
    conn: sqlite3.Connection,
    uid: str,
    kind: str,
    title: str,
    signature: str,
    module: str,
    file: str,
    start_line: int,
    end_line: int,
    content: str,
    content_hash: str,
    entity_type: str,
    entity_id: int,
) -> None:
    conn.execute(
        """INSERT OR REPLACE INTO chunks
           (uid,kind,title,signature,module,file,start_line,end_line,content,content_hash,entity_type,entity_id)
           VALUES (?,?,?,?,?,?,?,?,?,?,?,?)""",
        (uid, kind, title, signature, module, file, start_line, end_line, content, content_hash, entity_type, entity_id),
    )


def _git_commit(root: Path) -> Optional[str]:
    try:
        res = subprocess.run(
            ["git", "rev-parse", "HEAD"],
            cwd=str(root),
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
            check=False,
        )
        if res.returncode == 0:
            return res.stdout.strip()
    except Exception:
        pass
    return None


def _fmt_time(seconds: float) -> str:
    m, s = divmod(int(seconds), 60)
    return f"{m:02d}:{s:02d}"


def _should_ignore_dir(dirname: str, ignore_dirs: set, ignore_prefixes: tuple) -> bool:
    """Check if directory should be ignored."""
    if dirname in ignore_dirs:
        return True
    if dirname.startswith("."):
        return True
    if dirname.startswith(ignore_prefixes):
        return True
    return False


# -----------------------------
# Builder
# -----------------------------

@dataclass
class BuildStats:
    files: int = 0
    functions: int = 0
    types: int = 0
    examples: int = 0
    chunks: int = 0


class IndexBuilder:
    def __init__(
        self,
        source_dir: Path,
        output_dir: Path,
        *,
        enable_fts: bool = True,
        ignore_dirs: Sequence[str] = (),
        max_file_kb: int = 1024,
        max_chunk_chars: int = 8000,
        example_cap: int = 400,
        verbose: bool = False,
    ):
        self.source_dir = source_dir
        self.output_dir = output_dir
        self.enable_fts = enable_fts
        self.ignore_dirs = set(DEFAULT_IGNORE_DIRS) | set(ignore_dirs)
        self.ignore_prefixes = IGNORE_DIR_PREFIXES
        self.max_file_bytes = max(64 * 1024, max_file_kb * 1024)
        self.max_chunk_chars = max(1000, max_chunk_chars)
        self.example_cap = max(0, example_cap)
        self.verbose = verbose

    def _iter_source_files(self) -> Iterator[Path]:
        root = self.source_dir
        for dirpath, dirnames, filenames in os.walk(root):
            # Prune directories - modifies dirnames in-place
            dirnames[:] = [
                d for d in dirnames 
                if not _should_ignore_dir(d, self.ignore_dirs, self.ignore_prefixes)
            ]
            for fn in filenames:
                p = Path(dirpath) / fn
                if p.suffix.lower() in SOURCE_SUFFIXES:
                    yield p

    def build_base(self, version: str) -> Path:
        self.output_dir.mkdir(parents=True, exist_ok=True)
        db_path = self.output_dir / "index.db"
        if db_path.exists():
            db_path.unlink()

        t0 = time.time()
        print(f"[{_fmt_time(0)}] index build start (schema={SCHEMA_VERSION})")
        commit = _git_commit(self.source_dir)
        if commit:
            print(f"[{_fmt_time(0)}] git commit: {commit}")

        conn = _db_connect(db_path)
        _ensure_schema(conn, enable_fts=self.enable_fts)

        # metadata
        _set_kv(conn, "schema_version", SCHEMA_VERSION)
        _set_kv(conn, "version", version)
        _set_kv(conn, "built_utc", datetime.now(timezone.utc).isoformat())
        _set_kv(conn, "source", "iosonata")
        if commit:
            _set_kv(conn, "git_commit", commit)

        stats = BuildStats()

        print(f"[{_fmt_time(0)}] scanning source files...")
        last_progress = time.time()
        
        for path in self._iter_source_files():
            stats.files += 1
            rel = _relpath(path, self.source_dir)
            
            # Progress every 100 files or every 5 seconds
            now = time.time()
            if stats.files % 100 == 0 or (now - last_progress) > 5:
                elapsed = now - t0
                print(f"[{_fmt_time(elapsed)}] files={stats.files} chunks={stats.chunks} (last={rel})")
                last_progress = now

            try:
                src = _safe_read_text(path, self.max_file_bytes)
            except Exception:
                continue

            module = _infer_module(rel)

            # examples
            if self.example_cap and _is_example_path(rel):
                if stats.examples < self.example_cap:
                    ex_id = conn.execute(
                        "INSERT INTO examples(name,module,file,line,description) VALUES (?,?,?,?,?)",
                        (path.stem, module, rel, 1, ""),
                    ).lastrowid
                    content = _truncate(_example_chunk_text(rel, src), self.max_chunk_chars)
                    chash = hashlib.sha1(content.encode("utf-8", errors="ignore")).hexdigest()
                    uid = _stable_chunk_uid(rel, "example", 1, _line_count(content), chash)
                    _insert_chunk(conn, uid, "example", f"Example: {path.stem}", "", module, rel, 1, _line_count(content), content, chash, "example", ex_id)
                    stats.examples += 1
                    stats.chunks += 1

            # types
            for kind, name, s_idx, e_idx in iter_type_blocks(src):
                block = src[s_idx:e_idx+1]
                line = src[:s_idx].count("\n") + 1
                desc = _brief_comment_before(src, s_idx)
                type_id = conn.execute(
                    "INSERT INTO types(kind,name,module,file,line,description) VALUES (?,?,?,?,?,?)",
                    (kind, name, module, rel, line, desc),
                ).lastrowid
                content = _truncate(_type_chunk_text(kind, name, desc, rel, block), self.max_chunk_chars)
                chash = hashlib.sha1(content.encode("utf-8", errors="ignore")).hexdigest()
                uid = _stable_chunk_uid(rel, "type", line, line + content.count("\n"), chash)
                _insert_chunk(conn, uid, "type", f"{kind} {name}", "", module, rel, line, line + content.count("\n"), content, chash, "type", type_id)
                stats.types += 1
                stats.chunks += 1

            # functions
            for name, sig, s_idx, e_idx, has_body in iter_function_heads(src):
                if name in ("if", "for", "while", "switch"):
                    continue
                line = src[:s_idx].count("\n") + 1
                desc = _brief_comment_before(src, s_idx)
                func_id = conn.execute(
                    "INSERT INTO functions(name,signature,module,file,line,description) VALUES (?,?,?,?,?,?)",
                    (name, sig, module, rel, line, desc),
                ).lastrowid
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
                _insert_chunk(conn, uid, "function", name, sig, module, rel, line, line + content.count("\n"), content, chash, "function", func_id)
                stats.functions += 1
                stats.chunks += 1

            if stats.files % 200 == 0:
                conn.commit()

        conn.commit()

        if self.enable_fts:
            elapsed = time.time() - t0
            print(f"[{_fmt_time(elapsed)}] building FTS index...")
            _fts_rebuild(conn)

        # final metadata
        _set_kv(conn, "files_indexed", str(stats.files))
        _set_kv(conn, "functions_indexed", str(stats.functions))
        _set_kv(conn, "types_indexed", str(stats.types))
        _set_kv(conn, "examples_indexed", str(stats.examples))
        _set_kv(conn, "chunks_indexed", str(stats.chunks))
        conn.commit()
        conn.close()

        elapsed = time.time() - t0
        size_kb = db_path.stat().st_size // 1024
        print(f"[{_fmt_time(elapsed)}] ✓ built {db_path} ({size_kb} KB)")
        print(f"  files={stats.files} functions={stats.functions} types={stats.types} examples={stats.examples} chunks={stats.chunks}")
        return db_path

    def update_embeddings(
        self,
        db_path: Path,
        provider: str,
        api_key: Optional[str],
        model: Optional[str],
        *,
        batch_size: int = 0,  # 0 = use provider default
        kinds: Sequence[str] = ("function", "type", "example"),
        max_new: int = 0,
        verbose: bool = False,
    ) -> int:
        """
        Update embeddings in-place for chunks missing embeddings for (provider, model).
        Uses batch API calls for 10-20x faster embedding generation.
        """
        if not db_path.exists():
            raise FileNotFoundError(db_path)

        client = EmbeddingClient(provider=provider, api_key=api_key, model=model)
        provider = client.provider
        model = client.model
        
        # Use provider's optimal batch size if not specified
        if batch_size <= 0:
            batch_size = client.batch_size

        conn = _db_connect(db_path)
        _ensure_schema(conn, enable_fts=_has_fts(conn))

        t0 = time.time()
        print(f"[{_fmt_time(0)}] embedding update start (provider={provider}, model={model}, batch={batch_size})")

        # Find missing chunk_uids
        kind_placeholders = ",".join("?" for _ in kinds)
        limit_clause = f"LIMIT {int(max_new)}" if max_new and max_new > 0 else ""
        sql = f"""
          SELECT uid, COALESCE(title,''), COALESCE(signature,''), COALESCE(module,''), COALESCE(file,''), content
          FROM chunks
          WHERE kind IN ({kind_placeholders})
            AND uid NOT IN (SELECT chunk_uid FROM embeddings WHERE provider=? AND model=?)
          {limit_clause}
        """
        rows = conn.execute(sql, tuple(kinds) + (provider, model)).fetchall()
        total = len(rows)
        
        if total == 0:
            print(f"[{_fmt_time(0)}] no missing embeddings; done")
            conn.close()
            return 0

        print(f"[{_fmt_time(0)}] missing embeddings: {total} chunks")

        def build_text(title: str, sig: str, module: str, file: str, content: str) -> str:
            parts = []
            if title:
                parts.append(title)
            if sig:
                parts.append(sig)
            if module or file:
                parts.append(f"Module: {module}  File: {file}")
            parts.append(content)
            return "\n".join(parts)[:8000]

        done = 0
        failed = 0

        for batch_start in range(0, total, batch_size):
            batch_end = min(batch_start + batch_size, total)
            batch_rows = rows[batch_start:batch_end]

            # Prepare texts for batch embedding
            batch_uids = [row[0] for row in batch_rows]
            batch_texts = [build_text(row[1], row[2], row[3], row[4], row[5]) for row in batch_rows]

            # Batch embed
            batch_vecs = client.embed_batch(batch_texts)

            # Store results
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

            # Progress
            elapsed = time.time() - t0
            pct = 100 * batch_end / total
            print(f"[{_fmt_time(elapsed)}] embedded {batch_end}/{total} ({pct:.0f}%)")

        _set_kv(conn, f"embeddings_{provider}_{model}_updated_utc", str(int(time.time())))
        conn.commit()
        conn.close()

        elapsed = time.time() - t0
        rate = done / elapsed if elapsed > 0 else 0
        print(f"[{_fmt_time(elapsed)}] ✓ embeddings updated: {done} ({rate:.1f}/s)")
        if failed:
            print(f"  [warn] failed: {failed}")
        return done


# -----------------------------
# CLI
# -----------------------------

def main() -> None:
    p = argparse.ArgumentParser(description="Build IOsonata base index (GitHub) and/or update embeddings (startup).")
    p.add_argument("--source-dir", default=".", help="IOsonata repo root")
    p.add_argument("--output-dir", default=".iosonata", help="Output directory containing index.db")
    p.add_argument("--db", default="", help="Explicit path to index.db (for --update-embeddings)")
    p.add_argument("--version", default="dev", help="Version string")
    p.add_argument("--no-fts", action="store_true", help="Disable FTS5 index")
    p.add_argument("--ignore-dir", action="append", default=[], help="Additional directory name to ignore (repeatable)")
    p.add_argument("--max-file-kb", type=int, default=1024, help="Max bytes per file to index (KB)")
    p.add_argument("--max-chunk-chars", type=int, default=8000, help="Max chars per chunk")
    p.add_argument("--example-cap", type=int, default=400, help="Max example files to index (0 disables examples)")
    p.add_argument("--verbose", action="store_true", help="Verbose progress")

    # embedding update mode
    p.add_argument("--update-embeddings", action="store_true", help="Do not rebuild; only update embeddings in existing DB")
    p.add_argument("--provider", default="voyage", help="Embedding provider: voyage|openai|hash")
    p.add_argument("--api-key", default=os.environ.get("VOYAGE_API_KEY") or os.environ.get("OPENAI_API_KEY") or "", help="API key for provider")
    p.add_argument("--model", default="", help="Embedding model (provider default if empty)")
    p.add_argument("--batch-size", type=int, default=0, help="Embedding batch size (0=provider default)")
    p.add_argument("--max-new", type=int, default=0, help="Max new embeddings to create (0=all)")
    p.add_argument("--kinds", default="function,type,example", help="Comma list of chunk kinds to embed")
    args = p.parse_args()

    builder = IndexBuilder(
        source_dir=Path(args.source_dir),
        output_dir=Path(args.output_dir),
        enable_fts=not args.no_fts,
        ignore_dirs=args.ignore_dir,
        max_file_kb=args.max_file_kb,
        max_chunk_chars=args.max_chunk_chars,
        example_cap=args.example_cap,
        verbose=args.verbose,
    )

    if args.update_embeddings:
        db_path = Path(args.db) if args.db else (Path(args.output_dir) / "index.db")
        kinds = [k.strip() for k in args.kinds.split(",") if k.strip()]
        builder.update_embeddings(
            db_path=db_path,
            provider=args.provider,
            api_key=args.api_key or None,
            model=args.model or None,
            batch_size=args.batch_size,
            kinds=kinds,
            max_new=args.max_new,
            verbose=args.verbose,
        )
        return

    builder.build_base(args.version)


if __name__ == "__main__":
    main()
