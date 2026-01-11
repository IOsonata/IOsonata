#!/usr/bin/env python3
"""
IOsonata Repository Index Builder (CI-friendly, robust, hybrid-ready)

Designed for GitHub Actions (or any CI) to rebuild the repo index on change.

Key properties:
- Robust extraction: masks comments/strings and uses brace matching (no fragile body regex).
- Hybrid-ready retrieval: FTS5 (BM25) + optional embeddings + stable "chunks" for RAG/LLM.
- CI-friendly determinism: optional deterministic mode to avoid DB diffs on identical commits.
- Optional skip-if-unchanged: fast no-op when the index already matches the current commit + config.

Typical CI usage (deterministic + skip):
  python build_rag_index_v3.py --source-dir . --output-dir .iosonata --version 1.0.0 \\
    --deterministic --skip-if-unchanged

Optional embeddings (only if you have a key in CI secrets):
  python build_rag_index_v3.py --source-dir . --output-dir .iosonata --version 1.0.0 \\
    --deterministic --skip-if-unchanged --embeddings --provider voyage --api-key "$VOYAGE_API_KEY"
"""

from __future__ import annotations

import argparse
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
from typing import Dict, List, Optional, Sequence, Tuple

try:
    import requests  # type: ignore
except Exception:
    requests = None  # pragma: no cover


SCHEMA_VERSION = "3.3"
INDEXER_NAME = "build_rag_index.py"


# ----------------------------
# Low-level text utilities
# ----------------------------

def _utc_now_iso() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat().replace("+00:00", "Z")

def _md5(s: str) -> str:
    return hashlib.md5(s.encode("utf-8", errors="ignore")).hexdigest()

def _short_hash(s: str, n: int = 12) -> str:
    return _md5(s)[:n]

def _blank_preserve_newlines(buf: List[str], start: int, end: int) -> None:
    for i in range(start, end):
        if buf[i] != "\n":
            buf[i] = " "

def mask_comments_and_strings(src: str) -> str:
    """
    Return a same-length string where comments and string/char literals are replaced with spaces.
    Newlines are preserved so line-number mapping remains stable.
    """
    s = src
    out = list(s)
    i = 0
    n = len(s)

    while i < n:
        ch = s[i]

        # line comment //
        if ch == "/" and i + 1 < n and s[i + 1] == "/":
            j = s.find("\n", i + 2)
            if j == -1:
                j = n
            _blank_preserve_newlines(out, i, j)
            i = j
            continue

        # block comment /* ... */
        if ch == "/" and i + 1 < n and s[i + 1] == "*":
            j = s.find("*/", i + 2)
            if j == -1:
                j = n - 2
            _blank_preserve_newlines(out, i, j + 2)
            i = j + 2
            continue

        # string literal "..."
        if ch == '"':
            j = i + 1
            while j < n:
                if s[j] == "\\":
                    j += 2
                    continue
                if s[j] == '"':
                    j += 1
                    break
                j += 1
            _blank_preserve_newlines(out, i, min(j, n))
            i = j
            continue

        # char literal 'x'
        if ch == "'":
            j = i + 1
            while j < n:
                if s[j] == "\\":
                    j += 2
                    continue
                if s[j] == "'":
                    j += 1
                    break
                j += 1
            _blank_preserve_newlines(out, i, min(j, n))
            i = j
            continue

        i += 1

    return "".join(out)

def find_matching_brace(masked: str, open_brace_idx: int) -> Optional[int]:
    """
    Find matching '}' for masked[open_brace_idx] == '{' using brace depth.
    'masked' should already have comments/strings blanked out.
    """
    if open_brace_idx < 0 or open_brace_idx >= len(masked) or masked[open_brace_idx] != "{":
        return None
    depth = 0
    for i in range(open_brace_idx, len(masked)):
        c = masked[i]
        if c == "{":
            depth += 1
        elif c == "}":
            depth -= 1
            if depth == 0:
                return i
    return None

def extract_doc_block_before(src: str, pos: int, max_back: int = 3000) -> str:
    start = max(0, pos - max_back)
    prefix = src[start:pos]

    m = re.search(r"/\*\*[\s\S]*?\*/\s*$", prefix)
    if m:
        return m.group(0)

    lines = prefix.splitlines()
    doc_lines = []
    i = len(lines) - 1
    while i >= 0:
        ln = lines[i].rstrip()
        if ln.strip().startswith("//"):
            doc_lines.append(ln)
            i -= 1
            continue
        if ln.strip() == "":
            i -= 1
            continue
        break
    if doc_lines:
        return "\n".join(reversed(doc_lines)) + "\n"
    return ""

def brief_from_comment(comment: str) -> str:
    if not comment:
        return ""
    c = re.sub(r"/\*\*|\*/", "", comment)
    c = re.sub(r"^\s*//+ ?", "", c, flags=re.M)
    c = re.sub(r"\s+\*\s?", " ", c)
    c = " ".join(c.split())
    for sep in [". ", ".\n"]:
        if sep in c:
            return c.split(sep, 1)[0].strip()
    return c.strip()

def try_git_commit(repo_dir: Path) -> Optional[str]:
    """
    Full 40-hex commit id (deterministic). Returns None outside git.
    """
    try:
        r = subprocess.check_output(["git", "-C", str(repo_dir), "rev-parse", "HEAD"], stderr=subprocess.DEVNULL)
        v = r.decode("utf-8", errors="ignore").strip()
        return v or None
    except Exception:
        return None

def try_git_commit_time_iso(repo_dir: Path) -> Optional[str]:
    """
    Commit timestamp in ISO-8601 (deterministic for a given commit).
    """
    try:
        r = subprocess.check_output(["git", "-C", str(repo_dir), "show", "-s", "--format=%cI", "HEAD"], stderr=subprocess.DEVNULL)
        v = r.decode("utf-8", errors="ignore").strip()
        return v or None
    except Exception:
        return None

def _config_hash(config: dict) -> str:
    return _md5(json.dumps(config, sort_keys=True, separators=(",", ":")))

def _read_metadata_kv(db_path: Path) -> Dict[str, str]:
    if not db_path.exists():
        return {}
    try:
        conn = sqlite3.connect(str(db_path))
        cur = conn.execute("SELECT name FROM sqlite_master WHERE type='table' AND name='metadata_kv';")
        if not cur.fetchone():
            conn.close()
            return {}
        kv = {k: v for k, v in conn.execute("SELECT k, v FROM metadata_kv;").fetchall()}
        conn.close()
        return kv
    except Exception:
        return {}

def _tree_fingerprint(root: Path, files: List[Path]) -> str:
    """
    Fast fingerprint for non-git environments:
    - relative path (POSIX)
    - size
    - mtime_ns
    """
    h = hashlib.md5()
    for f in files:
        try:
            st = f.stat()
            rel = str(f.relative_to(root)).replace("\\", "/")
            h.update(rel.encode("utf-8", errors="ignore"))
            h.update(b"\0")
            h.update(str(int(st.st_size)).encode())
            h.update(b"\0")
            h.update(str(int(getattr(st, "st_mtime_ns", int(st.st_mtime * 1e9)))).encode())
            h.update(b"\n")
        except Exception:
            continue
    return h.hexdigest()



# ----------------------------
# Embeddings
# ----------------------------

class EmbeddingClient:
    """Embedding backends for hybrid retrieval.

    provider:
      - 'voyage'    : VoyageAI embeddings API (requires --api-key and 'requests')
      - 'openai'    : OpenAI embeddings API (requires --api-key and 'requests')
      - 'hash'      : dependency-free feature-hash embedding (no API key; deterministic; lexical)
      - 'fastembed' : local ONNX embeddings via 'fastembed' (optional dependency; no API key)

    Notes:
      - 'hash' embeddings are not semantic; they mainly encode token overlap and are best used as
        a lightweight vector signal alongside FTS/BM25.
      - 'fastembed' provides higher-quality local embeddings but increases install size/time.
    """

    def __init__(self, provider: str, api_key: str = "", model: Optional[str] = None, timeout_s: int = 30):
        self.provider = (provider or "").lower().strip()
        self.api_key = api_key or ""
        self.timeout_s = int(timeout_s)

        if self.provider == "voyage":
            self.model = model or "voyage-code-2"
        elif self.provider == "openai":
            self.model = model or "text-embedding-3-small"
        elif self.provider == "hash":
            self.model = model or "hash-512"
        elif self.provider == "fastembed":
            self.model = model or "BAAI/bge-small-en-v1.5"
        else:
            raise ValueError("provider must be one of: 'voyage', 'openai', 'hash', 'fastembed'")

        self._cache: Dict[str, List[float]] = {}
        self._hash_dim: int = 512
        self._fe = None  # fastembed model instance (lazy)

        if self.provider in ("voyage", "openai"):
            if not self.api_key:
                raise RuntimeError(f"--api-key is required for provider '{self.provider}'")
            if requests is None:
                raise RuntimeError("requests is required for API embeddings. Install with: pip install requests")

        if self.provider == "hash":
            # Parse dimension from model string like 'hash-256'/'hash-512'
            m = re.search(r"(\d+)", self.model)
            if m:
                try:
                    self._hash_dim = max(64, min(4096, int(m.group(1))))
                except Exception:
                    self._hash_dim = 512

    def embed(self, text: str) -> Optional[List[float]]:
        if not text:
            return None
        key = _md5(f"{self.provider}:{self.model}:{text}")
        if key in self._cache:
            return self._cache[key]
        try:
            if self.provider == "voyage":
                vec = self._embed_voyage(text)
            elif self.provider == "openai":
                vec = self._embed_openai(text)
            elif self.provider == "fastembed":
                vec = self._embed_fastembed(text)
            else:
                vec = self._embed_hash(text)

            if vec:
                self._cache[key] = vec
            return vec
        except Exception:
            return None

    def _embed_voyage(self, text: str) -> Optional[List[float]]:
        url = "https://api.voyageai.com/v1/embeddings"
        headers = {"Authorization": f"Bearer {self.api_key}", "Content-Type": "application/json"}
        payload = {"model": self.model, "input": [text]}
        r = requests.post(url, headers=headers, json=payload, timeout=self.timeout_s)
        r.raise_for_status()
        data = r.json()
        emb = data["data"][0]["embedding"]
        return emb if isinstance(emb, list) else None

    def _embed_openai(self, text: str) -> Optional[List[float]]:
        url = "https://api.openai.com/v1/embeddings"
        headers = {"Authorization": f"Bearer {self.api_key}", "Content-Type": "application/json"}
        payload = {"model": self.model, "input": text}
        r = requests.post(url, headers=headers, json=payload, timeout=self.timeout_s)
        r.raise_for_status()
        data = r.json()
        emb = data["data"][0]["embedding"]
        return emb if isinstance(emb, list) else None

    def _embed_fastembed(self, text: str) -> Optional[List[float]]:
        if self._fe is None:
            try:
                from fastembed import TextEmbedding  # type: ignore
            except Exception as e:
                raise RuntimeError("fastembed is not installed. Install with: pip install fastembed") from e
            # fastembed downloads model weights on first use (cacheable in CI/installer)
            self._fe = TextEmbedding(model_name=self.model)

        # fastembed returns an iterator/generator of vectors
        vecs = list(self._fe.embed([text]))
        if not vecs:
            return None
        v0 = vecs[0]
        # v0 may be numpy array or list-like
        return [float(x) for x in v0]

    def _embed_hash(self, text: str) -> Optional[List[float]]:
        # Feature hashing over identifier-like tokens.
        # Deterministic, dependency-free, and fast.
        tokens = re.findall(r"[A-Za-z_][A-Za-z0-9_]*", text)
        if not tokens:
            return None

        dim = self._hash_dim
        vec = [0.0] * dim
        # Count tokens; signed hashing to reduce collisions bias.
        for tok in tokens:
            h = int(hashlib.md5(tok.encode("utf-8", "ignore")).hexdigest(), 16)
            idx = h % dim
            sign = 1.0 if ((h >> 1) & 1) else -1.0
            vec[idx] += sign

        # L2 normalize
        norm2 = 0.0
        for x in vec:
            norm2 += x * x
        if norm2 <= 0.0:
            return None
        inv = 1.0 / (norm2 ** 0.5)
        return [x * inv for x in vec]


def pack_embedding(vec: Sequence[float]) -> bytes:
    return struct.pack("<" + "f" * len(vec), *[float(x) for x in vec])


# ----------------------------
# Index model
# ----------------------------

@dataclass
class FunctionInfo:
    id: int
    name: str
    qualified_name: str
    signature: str
    module: str
    file: str
    line: int
    description: str
    kind: str
    body: str

@dataclass
class TypeInfo:
    id: int
    name: str
    kind: str
    module: str
    file: str
    line: int
    description: str
    body: str


class IndexBuilder:
    """
    Robust index builder for a C/C++ codebase (optimized for IOsonata conventions, but generic).
    """

    MODULE_PATTERNS = {
        "ble": ["ble", "gatt", "gap", "smp", "att", "hci", "softdevice"],
        "uart": ["uart"],
        "i2c": ["i2c", "twi", "twim"],
        "spi": ["spi", "spim"],
        "gpio": ["gpio", "pin", "port"],
        "timer": ["timer", "rtc", "systick"],
        "pwm": ["pwm"],
        "adc": ["adc", "saadc"],
        "usb": ["usb"],
        "net": ["net", "tcp", "udp", "ip", "lwip"],
        "sensor": ["sensor", "imu", "accel", "gyro", "mag", "baro"],
        "core": ["core", "device", "board", "intrf", "i_syst", "isyst"],
    }

    SKIP_DIR_PATTERNS = [
        ".git", ".svn", ".hg",
        ".iosonata", ".metadata", ".vscode", ".idea", "__pycache__", "node_modules",
        "build", ".build", "out", "dist", "debug", "release",
        "cmake", "docs", "doc", "generated", "gen", "third_party", "vendor"
    ]

    SOURCE_EXTS = {".c", ".h", ".hpp", ".cpp", ".cc", ".cxx", ".inc"}

    def __init__(
        self,
        source_dir: str,
        output_dir: str,
        enable_embeddings: bool = False,
        api_key: str = "",
        embedding_provider: str = "voyage",
        embedding_model: Optional[str] = None,
        enable_fts: bool = True,
        max_file_kb: int = 1024,
        max_chunk_chars: int = 7000,
        include_examples: bool = True,
        examples_max_files: int = 300,
        examples_step_lines: int = 220,
        examples_markers: Optional[str] = None,
        deterministic: bool = False,
        vacuum: bool = False,
        skip_if_unchanged: bool = False,
        journal_mode: str = "WAL",
        progress_every: int = 250,
        verbose: bool = False,
        quiet: bool = False,
        compute_callgraph: bool = False,
        compute_type_usage: bool = False,
        wal_checkpoint: bool = True,
    ):
        self.source_dir = Path(source_dir).resolve()
        self.output_dir = Path(output_dir).resolve()
        self.output_db = self.output_dir / "index.db"

        self.enable_embeddings = enable_embeddings
        self.enable_fts = enable_fts
        self.max_file_kb = max_file_kb
        self.max_chunk_chars = max_chunk_chars
        self.include_examples = include_examples
        self.examples_max_files = int(examples_max_files)
        self.examples_step_lines = int(examples_step_lines)
        self.examples_markers = (examples_markers or '').strip()

        self.deterministic = deterministic
        self.vacuum = bool(vacuum)
        self.skip_if_unchanged = skip_if_unchanged
        self.journal_mode = "DELETE" if deterministic else journal_mode.upper()

        self.progress_every = max(1, int(progress_every))
        self.verbose = bool(verbose)
        self.quiet = bool(quiet)
        self.compute_callgraph = bool(compute_callgraph)
        self.compute_type_usage = bool(compute_type_usage)
        self.wal_checkpoint = bool(wal_checkpoint)
        self._log = None  # will be set in build()

        self.embedding_client: Optional[EmbeddingClient] = None
        # Normalize provider name early for consistent metadata + behavior.
        self.embedding_provider = (embedding_provider or "voyage").lower().strip()
        self.embedding_model = embedding_model

        if self.enable_embeddings:
            # API providers require keys; local providers do not.
            provider = self.embedding_provider
            if provider in ("voyage", "openai"):
                if not api_key:
                    raise ValueError(
                        "API key required for provider '%s' when --embeddings is enabled" % provider
                    )
                self.embedding_client = EmbeddingClient(provider, api_key, model=embedding_model)
            elif provider in ("hash", "fastembed"):
                # Local embedding backends (offline)
                self.embedding_client = EmbeddingClient(provider, api_key, model=embedding_model)
            else:
                raise ValueError("Unknown embedding provider: %s" % provider)

        self.functions_by_id: Dict[int, FunctionInfo] = {}
        self.types_by_id: Dict[int, TypeInfo] = {}
        self._type_spans: Dict[str, List[Tuple[int, int]]] = {}
        self._masked_cache: Dict[str, str] = {}


    def build(self, version: str) -> None:
        self.output_dir.mkdir(parents=True, exist_ok=True)

        t0 = time.monotonic()

        def _fmt_elapsed() -> str:
            s = int(time.monotonic() - t0)
            mm = s // 60
            ss = s % 60
            return f"{mm:02d}:{ss:02d}"

        def log(msg: str) -> None:
            if self.quiet:
                return
            print(f"[{_fmt_elapsed()}] {msg}", flush=True)

        self._log = log
        log(f"index build start (schema={SCHEMA_VERSION})")

        git_commit = try_git_commit(self.source_dir)
        git_time = try_git_commit_time_iso(self.source_dir)
        if git_commit:
            log(f"git commit: {git_commit}")
        else:
            log("git commit: <none> (non-git environment)")

        config = {
            "schema_version": SCHEMA_VERSION,
            "enable_fts": bool(self.enable_fts),
            "enable_embeddings": bool(self.embedding_client is not None),
            "embedding_provider": self.embedding_provider if self.embedding_client else None,
            "embedding_model": self.embedding_client.model if self.embedding_client else None,
            "max_file_kb": int(self.max_file_kb),
            "max_chunk_chars": int(self.max_chunk_chars),
            "include_examples": bool(self.include_examples),
            "vacuum": bool(self.vacuum),
            "examples_max_files": int(self.examples_max_files),
            "examples_step_lines": int(self.examples_step_lines),
            "examples_markers": self.examples_markers,
        }
        cfg_hash = _config_hash(config)

        if self.skip_if_unchanged and self.output_db.exists():
            kv = _read_metadata_kv(self.output_db)
            if git_commit:
                if kv.get("git_commit") == git_commit and kv.get("config_hash") == cfg_hash and kv.get("schema_version") == SCHEMA_VERSION:
                    print(f"Index up-to-date: {self.output_db} (commit={git_commit[:12]})")
                    return
            else:
                # Outside git (rare in CI, but possible in local installer-like use).
                # Fall back to a fast tree fingerprint so skip-if-unchanged is still correct.
                sources_for_fp = self._find_sources()
                fp = _tree_fingerprint(self.source_dir, sources_for_fp)
                if kv.get("tree_fingerprint") == fp and kv.get("config_hash") == cfg_hash and kv.get("schema_version") == SCHEMA_VERSION:
                    print(f"Index up-to-date: {self.output_db} (commit=n/a)")
                    return

        if self.output_db.exists():
            self.output_db.unlink()

        conn = sqlite3.connect(str(self.output_db))
        conn.execute(f"PRAGMA journal_mode={self.journal_mode};")
        conn.execute("PRAGMA synchronous=NORMAL;")
        conn.execute("PRAGMA temp_store=MEMORY;")
        conn.execute("PRAGMA foreign_keys=ON;")

        self._create_schema(conn)

        search_mode = "hybrid" if (self.enable_fts and self.enable_embeddings) else ("fts" if self.enable_fts else "keywords")

        built = _utc_now_iso()
        if self.deterministic and git_time:
            built = git_time  # stable for a given commit

        conn.execute(
            "INSERT INTO metadata VALUES (?,?,?,?,?,?,?,?,?,?,?)",
            (
                version,
                INDEXER_NAME,
                built,
                "IOsonata",
                str(self.source_dir),
                git_commit,
                None,  # sdk
                search_mode,
                (self.embedding_provider if self.embedding_client else None),
                (self.embedding_client.model if self.embedding_client else None),
                None,  # embedding_dim (filled after embeddings)
            ),
        )

        # extra deterministic/provenance fields (do NOT force DB churn unless commit/config changes)
        conn.execute("INSERT INTO metadata_kv(k, v) VALUES(?,?)", ("schema_version", SCHEMA_VERSION))
        conn.execute("INSERT INTO metadata_kv(k, v) VALUES(?,?)", ("git_commit", git_commit or ""))
        conn.execute("INSERT INTO metadata_kv(k, v) VALUES(?,?)", ("built_mode", "deterministic" if self.deterministic else "now"))
        conn.execute("INSERT INTO metadata_kv(k, v) VALUES(?,?)", ("config_hash", cfg_hash))
        conn.execute("INSERT INTO metadata_kv(k, v) VALUES(?,?)", ("config_json", json.dumps(config, sort_keys=True)))
        conn.commit()

        log("scanning source files...")
        sources = self._find_sources()
        log(f"source files: {len(sources)}")

        if not git_commit:
            conn.execute("INSERT OR REPLACE INTO metadata_kv(k, v) VALUES(?,?)", ("tree_fingerprint", _tree_fingerprint(self.source_dir, sources)))
            conn.commit()

        conn.execute("BEGIN;")
        func_count = type_count = 0

        parsed_files = 0
        last_parse_log = time.monotonic()
        for fp in sources:
            try:
                if fp.stat().st_size > self.max_file_kb * 1024:
                    continue
                content = fp.read_text(encoding="utf-8", errors="ignore")
            except Exception:
                continue

            rel = str(fp.relative_to(self.source_dir)).replace("\\", "/")
            module = self._detect_module(fp)
            parsed_files += 1
            if (parsed_files % self.progress_every == 0) or (time.monotonic() - last_parse_log) > 8.0:
                last_parse_log = time.monotonic()
                if self.verbose:
                    log(f"parse: {parsed_files}/{len(sources)} files (last: {rel})")
                else:
                    log(f"parse: {parsed_files}/{len(sources)} files")

            type_count += self._parse_types(conn, content, rel, module)
            func_count += self._parse_functions(conn, content, rel, module)

        conn.execute("COMMIT;")

        # Example chunks (optional)
        chunk_count = 0
        if self.include_examples:
            log("indexing examples as chunks...")
        conn.execute("BEGIN;")
        if self.include_examples:
            chunk_count += self._index_examples_as_chunks(conn, sources)
        conn.execute("COMMIT;")
        if self.include_examples:
            log(f"example chunks: {chunk_count}")

        # Code graph
        if self.compute_callgraph or self.compute_type_usage:
            log("building code graph...")
            conn.execute("BEGIN;")
            if self.compute_callgraph:
                self._build_call_graph(conn)
            if self.compute_type_usage:
                self._build_type_usage(conn)
            conn.execute("COMMIT;")
            log("code graph: done")
        else:
            log("code graph: skipped")

        # Chunks for functions/types
        log("creating chunks (functions/types)...")
        conn.execute("BEGIN;")
        chunk_count += self._create_chunks(conn)
        conn.execute("COMMIT;")
        log(f"chunks total: {chunk_count}")

        # FTS
        if self.enable_fts:
            log("building FTS5 index...")
            self._build_fts(conn)
            log("FTS5: done")

        # Embeddings (optional)
        if self.embedding_client:
            emb_dim = self._build_embeddings(conn)
            conn.execute("UPDATE metadata SET embedding_dim=? WHERE rowid=1;", (emb_dim,))
            conn.commit()

        log("building module schemas...")
        self._build_module_schemas(conn)
        log("module schemas: done")

        if self.vacuum:
            # compact into a stable single-file form (can be expensive)
            try:
                conn.execute("VACUUM;")
            except Exception:
                pass

        conn.commit()
        try:
            if self.wal_checkpoint and self.journal_mode.upper() == "WAL":
                log("checkpointing WAL...")
                conn.execute("PRAGMA wal_checkpoint(TRUNCATE);")
                conn.commit()
        except Exception:
            pass
        conn.close()
        log("index build complete")


        print(f"Built repo index: {self.output_db}")
        print(f"  functions: {func_count}")
        print(f"  types:     {type_count}")
        print(f"  chunks:    {chunk_count}")
        print(f"  fts:       {'on' if self.enable_fts else 'off'}")
        print(f"  embeddings:{'on' if self.embedding_client else 'off'}")
        if git_commit:
            print(f"  git:       {git_commit[:12]}")

    # ----------------------------
    # Schema
    # ----------------------------

    def _create_schema(self, conn: sqlite3.Connection) -> None:
        conn.execute("""
            CREATE TABLE metadata (
                version TEXT,
                indexer TEXT,
                built TEXT,
                source_name TEXT,
                source_path TEXT,
                git_commit TEXT,
                sdk TEXT,
                search_mode TEXT,
                embedding_provider TEXT,
                embedding_model TEXT,
                embedding_dim INTEGER
            )
        """)
        conn.execute("""
            CREATE TABLE metadata_kv (
                k TEXT PRIMARY KEY,
                v TEXT
            )
        """)

        conn.execute("""
            CREATE TABLE functions (
                id INTEGER PRIMARY KEY,
                name TEXT,
                qualified_name TEXT,
                signature TEXT,
                module TEXT,
                file TEXT,
                line INTEGER,
                description TEXT,
                kind TEXT,
                body_hash TEXT
            )
        """)
        conn.execute("CREATE INDEX idx_functions_name ON functions(name);")
        conn.execute("CREATE INDEX idx_functions_qname ON functions(qualified_name);")
        conn.execute("CREATE INDEX idx_functions_module ON functions(module);")

        conn.execute("""
            CREATE TABLE types (
                id INTEGER PRIMARY KEY,
                name TEXT,
                kind TEXT,
                module TEXT,
                file TEXT,
                line INTEGER,
                description TEXT,
                body_hash TEXT
            )
        """)
        conn.execute("CREATE INDEX idx_types_name ON types(name);")

        conn.execute("""
            CREATE TABLE call_graph (
                caller_id INTEGER,
                callee TEXT,
                callee_kind TEXT,
                FOREIGN KEY (caller_id) REFERENCES functions(id)
            )
        """)
        conn.execute("CREATE INDEX idx_call_graph_caller ON call_graph(caller_id);")

        conn.execute("""
            CREATE TABLE type_usage (
                function_id INTEGER,
                type_name TEXT,
                FOREIGN KEY (function_id) REFERENCES functions(id)
            )
        """)
        conn.execute("CREATE INDEX idx_type_usage_fn ON type_usage(function_id);")

        conn.execute("""
            CREATE TABLE chunks (
                id INTEGER PRIMARY KEY,
                uid TEXT NOT NULL,
                entity_type TEXT,
                entity_id INTEGER,
                title TEXT,
                module TEXT,
                file TEXT,
                start_line INTEGER,
                end_line INTEGER,
                kind TEXT,
                content TEXT,
                content_hash TEXT
            )
        """)
        conn.execute("CREATE INDEX idx_chunks_entity ON chunks(entity_type, entity_id);")
        conn.execute("CREATE UNIQUE INDEX idx_chunks_uid ON chunks(uid);")

        conn.execute("""
            CREATE TABLE embeddings (
                chunk_uid TEXT NOT NULL,
                provider  TEXT NOT NULL,
                model     TEXT NOT NULL,
                dim       INTEGER NOT NULL,
                embedding BLOB NOT NULL,
                updated_utc INTEGER NOT NULL,
                PRIMARY KEY (chunk_uid, provider, model),
                FOREIGN KEY (chunk_uid) REFERENCES chunks(uid)
            )
        """)
        conn.execute("CREATE INDEX idx_embeddings_chunk_uid ON embeddings(chunk_uid);")

        conn.execute("""
            CREATE TABLE module_schemas (
                module TEXT PRIMARY KEY,
                schema_json TEXT
            )
        """)

        conn.commit()

    # ----------------------------
    # Discovery
    # ----------------------------

    def _find_sources(self) -> List[Path]:
        """
        Discover source files quickly.

        Uses os.walk with directory pruning (much faster than Path.rglob on large repos).
        """
        files: List[Path] = []
        skip = {x.lower() for x in self.SKIP_DIR_PATTERNS}

        scanned_files = 0
        kept = 0
        last_log = time.monotonic()

        for root, dirs, filenames in os.walk(self.source_dir, topdown=True):
            # Prune directories early
            dirs[:] = [d for d in dirs if d.lower() not in skip]

            for fn in filenames:
                scanned_files += 1
                ext = Path(fn).suffix.lower()
                if ext not in self.SOURCE_EXTS:
                    continue
                p = Path(root) / fn

                # Safety net: skip any file whose relative path contains a skipped component
                try:
                    rel_parts = [part.lower() for part in p.relative_to(self.source_dir).parts]
                except Exception:
                    continue
                if any(part in skip for part in rel_parts):
                    continue

                files.append(p)
                kept += 1

                if self._log and (kept % self.progress_every == 0 or (time.monotonic() - last_log) > 5.0):
                    last_log = time.monotonic()
                    self._log(f"scan: kept {kept} source files (scanned {scanned_files})")

        files.sort(key=lambda x: str(x.relative_to(self.source_dir)).replace("\\", "/"))
        return files


    def _detect_module(self, filepath: Path) -> str:
        name = filepath.name.lower()
        path = str(filepath).replace("\\", "/").lower()
        for mod, pats in self.MODULE_PATTERNS.items():
            if any(p in name or f"/{p}/" in path for p in pats):
                return mod
        return "core"

    def _masked(self, content: str) -> str:
        k = _short_hash(content, 16)
        if k not in self._masked_cache:
            self._masked_cache[k] = mask_comments_and_strings(content)
        return self._masked_cache[k]

    # ----------------------------
    # Parsing (types)
    # ----------------------------

    def _parse_types(self, conn: sqlite3.Connection, content: str, filepath: str, module: str) -> int:
        masked = self._masked(content)
        count = 0

        type_re = re.compile(r"\b(class|struct|union|enum(?:\s+class)?)\s+(?P<name>[A-Za-z_]\w*)[^{;]*\{")
        for m in type_re.finditer(masked):
            kind = m.group(1).replace("  ", " ").strip()
            name = m.group("name")

            open_idx = masked.find("{", m.end() - 1)
            if open_idx == -1:
                continue
            close_idx = find_matching_brace(masked, open_idx)
            if close_idx is None:
                continue

            # require trailing ';' to reduce false positives
            semi = masked.find(";", close_idx)
            if semi == -1 or semi - close_idx > 15:
                continue

            start = m.start()
            end = semi + 1
            block = content[start:end]

            self._type_spans.setdefault(filepath, []).append((start, end))

            line = content[:start].count("\n") + 1
            comment = extract_doc_block_before(content, start)
            desc = brief_from_comment(comment)
            body_hash = _short_hash(block, 12)

            cur = conn.execute(
                "INSERT INTO types(name, kind, module, file, line, description, body_hash) VALUES(?,?,?,?,?,?,?)",
                (name, kind, module, filepath, line, desc, body_hash),
            )
            tid = int(cur.lastrowid)
            self.types_by_id[tid] = TypeInfo(tid, name, kind, module, filepath, line, desc, block)
            count += 1

        return count

    # ----------------------------
    # Parsing (functions)
    # ----------------------------

    def _parse_functions(self, conn: sqlite3.Connection, content: str, filepath: str, module: str) -> int:
        masked = self._masked(content)
        spans = sorted(self._type_spans.get(filepath, []))

        def _in_type_span(pos: int) -> bool:
            # spans per file are small; linear scan is fine and deterministic
            for a, b in spans:
                if a <= pos < b:
                    return True
            return False

        header_re = re.compile(
            r'(?P<comment>/\*\*[\s\S]*?\*/\s*)?'
            r'(?P<prefix>(?:^|[\n;{}]))\s*'
            r'(?:template\s*<[^>]*>\s*)?'
            r'(?:extern\s+|static\s+|inline\s+|constexpr\s+|virtual\s+|__attribute__\s*\(\([^)]*\)\)\s+)*'
            r'(?P<ret>[\w:\<\>\s\*&]+?)\s+'
            r'(?P<name>[A-Za-z_~]\w*(?:::[A-Za-z_~]\w*)*)\s*'
            r'\((?P<params>[^)]*)\)\s*'
            r'(?P<trailer>(?:const\s+)?(?:noexcept\s+)?(?:override\s+)?(?:final\s+)?)(?P<end>[{;])'
        )

        count = 0
        for m in header_re.finditer(masked):
            if _in_type_span(m.start('name')):
                continue

            qname = m.group("name")
            if qname.startswith("_"):
                continue
            if qname in {"if", "for", "while", "switch", "return", "sizeof", "alignof", "catch", "static_assert"}:
                continue

            end_ch = m.group("end")
            body = ""
            kind = "declaration"

            if end_ch == "{":
                open_idx = m.end("end") - 1
                close_idx = find_matching_brace(masked, open_idx)
                if close_idx is None:
                    continue
                body = content[open_idx + 1 : close_idx]
                kind = "definition"

            ret = " ".join(content[m.start("ret") : m.end("ret")].split())
            params = " ".join(content[m.start("params") : m.end("params")].split())
            sig = f"{ret} {qname}({params})"

            line = content[: m.start()].count("\n") + 1
            comment = m.group("comment") or extract_doc_block_before(content, m.start())
            desc = brief_from_comment(comment)
            body_hash = _short_hash(body, 12) if body else ""

            simple_name = qname.split("::")[-1]

            cur = conn.execute(
                """INSERT INTO functions(name, qualified_name, signature, module, file, line, description, kind, body_hash)
                   VALUES(?,?,?,?,?,?,?,?,?)""",
                (simple_name, qname, sig, module, filepath, line, desc, kind, body_hash),
            )
            fid = int(cur.lastrowid)
            self.functions_by_id[fid] = FunctionInfo(fid, simple_name, qname, sig, module, filepath, line, desc, kind, body)
            count += 1

        return count

    # ----------------------------
    # Examples as chunks
    # ----------------------------

    def _index_examples_as_chunks(self, conn: sqlite3.Connection, sources: List[Path]) -> int:
        """
        Index example-like source files as chunks.

        Performance notes:
        - Reuses the already-discovered `sources` list (no second filesystem walk).
        - Caps the number of example files to keep CI runtimes bounded.
        """
        default_markers = ["example", "examples", "demo", "sample", "samples", "tutorial"]
        markers = []
        if self.examples_markers:
            markers = [m.strip().lower() for m in self.examples_markers.split(",") if m.strip()]
        if not markers:
            markers = default_markers

        files: List[Path] = []
        for f in sources:
            try:
                rel = str(f.relative_to(self.source_dir)).replace("\\", "/")
            except Exception:
                continue
            rel_l = rel.lower()
            name_l = f.name.lower()
            if any(f"/{mk}/" in rel_l for mk in markers) or any(name_l.startswith(mk) for mk in markers):
                files.append(f)

        files.sort(key=lambda x: str(x.relative_to(self.source_dir)).replace("\\", "/"))

        if self.examples_max_files > 0 and len(files) > self.examples_max_files:
            files = files[: self.examples_max_files]

        step = max(40, int(self.examples_step_lines))
        rows = []
        count = 0

        for f in files:
            try:
                if f.stat().st_size > self.max_file_kb * 1024:
                    continue
                content = f.read_text(encoding="utf-8", errors="ignore")
            except Exception:
                continue

            rel = str(f.relative_to(self.source_dir)).replace("\\", "/")
            module = self._detect_module(f)

            lines = content.splitlines()
            for i in range(0, len(lines), step):
                chunk_lines = lines[i : i + step]
                if not chunk_lines:
                    continue
                start_line = i + 1
                end_line = i + len(chunk_lines)
                chunk_text = "\n".join(chunk_lines)
                if not chunk_text.strip():
                    continue
                title = f"{rel}:{start_line}-{end_line}"
                h = hashlib.sha1(chunk_text.encode('utf-8', errors='ignore')).hexdigest()
                uid = hashlib.sha1(f"{rel}|example|{start_line}|{end_line}|{h}".encode("utf-8", errors="ignore")).hexdigest()
                rows.append((
                    uid,
                    "example_file", None, title, module, rel,
                    start_line, end_line, "example",
                    chunk_text[: self.max_chunk_chars], h
                ))
                count += 1

        if rows:
            conn.executemany(
                """INSERT INTO chunks(uid, entity_type, entity_id, title, module, file, start_line, end_line, kind, content, content_hash)
                   VALUES(?,?,?,?,?,?,?,?,?,?,?)""",
                rows,
            )
        return count

    # ----------------------------
    # Code graph
    # ----------------------------

    def _build_call_graph(self, conn: sqlite3.Connection) -> None:
        conn.execute("DELETE FROM call_graph;")
        if not self.functions_by_id:
            return

        call_re = re.compile(r"\b([A-Za-z_]\w*)\s*\(")
        member_re = re.compile(r"(?:\.|->)\s*([A-Za-z_]\w*)\s*\(")
        keywords = {"if", "for", "while", "switch", "return", "sizeof", "alignof", "catch", "static_assert", "new", "delete"}

        for fid, fi in self.functions_by_id.items():
            if not fi.body:
                continue
            masked_body = mask_comments_and_strings(fi.body)
            calls = set(call_re.findall(masked_body)) | set(member_re.findall(masked_body))
            calls = {c for c in calls if c not in keywords and c != fi.name}
            rows = [(fid, c, "call") for c in sorted(calls)]
            if rows:
                conn.executemany("INSERT INTO call_graph VALUES(?,?,?)", rows)

    def _build_type_usage(self, conn: sqlite3.Connection) -> None:
        conn.execute("DELETE FROM type_usage;")
        if not self.functions_by_id:
            return

        t_typedef = re.compile(r"\b([A-Za-z_]\w*_t)\b")
        t_camel = re.compile(r"\b([A-Z][A-Za-z0-9_]{2,})\b")

        blacklist = {"NULL", "TRUE", "FALSE", "UINT32", "UINT16", "UINT8", "INT32", "INT16", "INT8", "SIZE_T"}
        for fid, fi in self.functions_by_id.items():
            if not fi.body:
                continue
            masked_body = mask_comments_and_strings(fi.body)
            types = set(t_typedef.findall(masked_body)) | set(t_camel.findall(masked_body))
            types = {t for t in types if t not in blacklist}
            rows = [(fid, t) for t in sorted(types)]
            if rows:
                conn.executemany("INSERT INTO type_usage VALUES(?,?)", rows)

    # ----------------------------
    # Chunks
    # ----------------------------

    def _create_chunks(self, conn: sqlite3.Connection) -> int:
        rows = []
        count = 0

        for fid, fi in self.functions_by_id.items():
            if not fi.body and not fi.description:
                continue
            content = fi.signature
            if fi.body:
                content = fi.signature + "\n{\n" + fi.body.strip() + "\n}"
            content = content.strip()
            if not content:
                continue
            h = hashlib.sha1(content.encode('utf-8', errors='ignore')).hexdigest()
            start_line = fi.line
            end_line = fi.line + max(0, content.count("\n"))
            uid = hashlib.sha1(f"{fi.file}|function|{start_line}|{end_line}|{h}".encode("utf-8", errors="ignore")).hexdigest()
            rows.append((
                uid,
                "function", fid, fi.signature, fi.module, fi.file,
                start_line, end_line, "function",
                content[: self.max_chunk_chars], h
            ))
            count += 1

        for tid, ti in self.types_by_id.items():
            content = ti.body.strip()
            if not content:
                continue
            h = hashlib.sha1(content.encode('utf-8', errors='ignore')).hexdigest()
            start_line = ti.line
            end_line = ti.line + max(0, content.count("\n"))
            uid = hashlib.sha1(f"{ti.file}|type|{start_line}|{end_line}|{h}".encode("utf-8", errors="ignore")).hexdigest()
            rows.append((
                uid,
                "type", tid, f"{ti.kind} {ti.name}", ti.module, ti.file,
                start_line, end_line, "type",
                content[: self.max_chunk_chars], h
            ))
            count += 1

        if rows:
            conn.executemany(
                """INSERT INTO chunks(uid, entity_type, entity_id, title, module, file, start_line, end_line, kind, content, content_hash)
                   VALUES(?,?,?,?,?,?,?,?,?,?,?)""",
                rows,
            )
        return count


    # ----------------------------
    # FTS
    # ----------------------------

    def _build_fts(self, conn: sqlite3.Connection) -> None:
        try:
            conn.execute("DROP TABLE IF EXISTS fts_chunks;")
            conn.execute("""
                CREATE VIRTUAL TABLE fts_chunks USING fts5(
                    title, content, module, file, kind,
                    content='chunks', content_rowid='id'
                )
            """)
            conn.execute("INSERT INTO fts_chunks(fts_chunks) VALUES('rebuild');")
            conn.commit()
        except sqlite3.OperationalError as e:
            print(f"[warn] FTS5 unavailable: {e}. Continuing without FTS.", file=sys.stderr)
            self.enable_fts = False

    # ----------------------------
    # Embeddings
    # ----------------------------

    def _build_embeddings(self, conn: sqlite3.Connection) -> Optional[int]:
        """Populate (or incrementally update) the embeddings table for the configured provider/model."""
        assert self.embedding_client is not None

        provider = self.embedding_client.provider
        model = self.embedding_client.model
        now = int(time.time())

        cur = conn.execute(
            """SELECT c.uid, c.title, c.content
                   FROM chunks c
              LEFT JOIN embeddings e
                     ON e.chunk_uid = c.uid
                    AND e.provider = ?
                    AND e.model = ?
                  WHERE e.chunk_uid IS NULL
               ORDER BY c.id;""",
            (provider, model),
        )
        rows = cur.fetchall()

        inserted = 0
        emb_dim: Optional[int] = None

        for uid, title, content in rows:
            text = (str(title or "") + "\n\n" + str(content or "")).strip()
            if len(text) > 9000:
                text = text[:9000]
            vec = self.embedding_client.embed(text)
            if not vec:
                continue
            if emb_dim is None:
                emb_dim = len(vec)

            conn.execute(
                """INSERT OR REPLACE INTO embeddings(chunk_uid, provider, model, dim, embedding, updated_utc)
                       VALUES(?,?,?,?,?,?)""",
                (uid, provider, model, len(vec), pack_embedding(vec), now),
            )
            inserted += 1

        conn.commit()
        print(f"  embeddings stored: {inserted}")
        return emb_dim


    # ----------------------------
    # Module schemas
    # ----------------------------

    def _build_module_schemas(self, conn: sqlite3.Connection) -> None:
        conn.execute("DELETE FROM module_schemas;")

        fcounts = {m: c for m, c in conn.execute("SELECT module, COUNT(*) FROM functions GROUP BY module;").fetchall()}
        tcounts = {m: c for m, c in conn.execute("SELECT module, COUNT(*) FROM types GROUP BY module;").fetchall()}
        ecounts = {m: c for m, c in conn.execute("SELECT module, COUNT(*) FROM chunks WHERE kind='example' GROUP BY module;").fetchall()}

        modules = set(fcounts) | set(tcounts) | set(ecounts)
        for mod in sorted(modules):
            schema = {
                "module": mod,
                "functions": int(fcounts.get(mod, 0)),
                "types": int(tcounts.get(mod, 0)),
                "example_chunks": int(ecounts.get(mod, 0)),
                "source": "IOsonata",
            }
            conn.execute("INSERT INTO module_schemas VALUES(?,?)", (mod, json.dumps(schema)))

        conn.commit()


# ----------------------------
# CLI
# ----------------------------

def main() -> None:
    p = argparse.ArgumentParser(
        description="Build repo index (robust, hybrid-ready, CI-friendly)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument("--source-dir", default=".", help="Source directory (repo root)")
    p.add_argument("--output-dir", default=".iosonata", help="Output directory (index.db will be created here)")
    p.add_argument("--version", default="0.0.0", help="Index version string")

    p.add_argument("--profile", default="ci", choices=["ci", "deep"],
                   help="Index profile: 'ci' (fast defaults for GitHub) or 'deep' (slower: graphs + unlimited examples)")

    p.add_argument("--deterministic", action="store_true",
                   help="Prefer stable output for a given commit/config (built timestamp from git, sorted inputs, journal=DELETE).")
    p.add_argument("--vacuum", action="store_true",
                   help="Run VACUUM at end (slow). Enable only if you commit index.db and want minimal diffs.")
    p.add_argument("--skip-if-unchanged", action="store_true",
                   help="No-op if existing index matches current git commit + config hash")
    p.add_argument("--verbose", action="store_true", help="Verbose progress logging (prints last processed file)")
    p.add_argument("--quiet", action="store_true", help="Suppress progress logging")
    p.add_argument("--progress-every", type=int, default=250, help="Progress log interval (files/chunks)")

    # Deep analysis knobs
    p.add_argument("--callgraph", action="store_true", help="Enable call graph extraction (slow)")
    p.add_argument("--type-usage", action="store_true", help="Enable type-usage extraction (slow)")
    p.add_argument("--no-callgraph", action="store_true", help="Force-disable call graph extraction (faster)")
    p.add_argument("--no-type-usage", action="store_true", help="Force-disable type-usage extraction (faster)")

    p.add_argument("--no-checkpoint", action="store_true", help="Do not WAL-checkpoint at end (leave -wal/-shm)")
    p.add_argument("--journal", default="WAL", choices=["WAL", "DELETE"],
                   help="SQLite journal mode (ignored when --deterministic; deterministic forces DELETE)")

    # Hybrid retrieval
    p.add_argument("--embeddings", action="store_true", help="Enable vector embeddings (hybrid search)")
    p.add_argument("--provider", default="hash", choices=["voyage", "openai", "hash", "fastembed"],
                   help="Embedding backend (API: voyage/openai; local: hash/fastembed). Default is 'hash' (offline).")
    p.add_argument("--embedding-model", default=None, help="Embedding model name (provider-specific)")
    p.add_argument("--api-key", default="", help="API key for embedding provider (voyage/openai)")

    # Chunking / search
    p.add_argument("--no-fts", action="store_true", help="Disable SQLite FTS5 index (BM25)")
    p.add_argument("--max-file-kb", type=int, default=1024, help="Skip files larger than this size (KB)")
    p.add_argument("--max-chunk-chars", type=int, default=7000, help="Max chars stored per chunk")
    p.add_argument("--no-examples", action="store_true", help="Disable example chunk indexing")
    p.add_argument("--examples-max-files", type=int, default=300, help="Max number of example files to chunk (0=unlimited)")
    p.add_argument("--examples-step-lines", type=int, default=220, help="Lines per example chunk block")
    p.add_argument("--examples-markers", default="",
                   help="Comma-separated directory/name markers treated as examples (default: example,examples,demo,sample,samples,tutorial)")

    args = p.parse_args()

    # Resolve API key from environment for convenience (CI/installer).
    api_key = args.api_key or ""
    if not api_key:
        if args.provider == "voyage":
            api_key = os.getenv("VOYAGE_API_KEY", "")
        elif args.provider == "openai":
            api_key = os.getenv("OPENAI_API_KEY", "")

    # Profile-driven defaults (important for speed in CI).
    callgraph = (args.profile == "deep") or bool(args.callgraph)
    type_usage = (args.profile == "deep") or bool(args.type_usage)
    if args.no_callgraph:
        callgraph = False
    if args.no_type_usage:
        type_usage = False

    examples_max_files = int(args.examples_max_files)
    # In deep mode, default to unlimited examples unless explicitly capped.
    if args.profile == "deep" and args.examples_max_files == 300:
        examples_max_files = 0

    # In GitHub Actions CI, it's usually safe to skip rebuild when commit/config is unchanged.
    if os.getenv("GITHUB_ACTIONS") == "true" and not args.skip_if_unchanged:
        args.skip_if_unchanged = True

    builder = IndexBuilder(
        source_dir=args.source_dir,
        output_dir=args.output_dir,
        enable_embeddings=args.embeddings,
        api_key=api_key,
        embedding_provider=args.provider,
        embedding_model=args.embedding_model,
        enable_fts=(not args.no_fts),
        max_file_kb=args.max_file_kb,
        max_chunk_chars=args.max_chunk_chars,
        include_examples=(not args.no_examples),
        examples_max_files=examples_max_files,
        examples_step_lines=args.examples_step_lines,
        examples_markers=args.examples_markers,
        deterministic=args.deterministic,
        vacuum=args.vacuum,
        skip_if_unchanged=args.skip_if_unchanged,
        journal_mode=args.journal,
        progress_every=args.progress_every,
        verbose=args.verbose,
        quiet=args.quiet,
        compute_callgraph=callgraph,
        compute_type_usage=type_usage,
        wal_checkpoint=(not args.no_checkpoint),
    )
    builder.build(args.version)

if __name__ == "__main__":
    main()
