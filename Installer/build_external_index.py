#!/usr/bin/env python3
"""
External SDK Index Builder (Installer-friendly, robust, hybrid-ready)

Designed to be called by an installer script to build local indexes for third-party SDKs
(nrfx, nRF5 SDK, FreeRTOS, lwIP, TinyUSB, LVGL, etc.).

Key properties:
- Robust extraction: masks comments/strings and uses brace matching for type blocks and function bodies.
- Hybrid-ready retrieval: FTS5 (BM25) + optional embeddings + stable "chunks" for RAG/LLM.
- Installer-friendly: optional skip-if-unchanged based on a fast SDK tree fingerprint (path+size+mtime).

Typical installer usage (no network, fast keyword/FTS):
  python build_external_index_v3.py --sdk /path/to/nrfx --name nrfx --output /path/to/index/nrfx.db --version 1.0.0 \\
    --skip-if-unchanged

Optional embeddings (only if you have a key):
  python build_external_index_v3.py --sdk /path/to/nrfx --name nrfx --output nrfx.db --version 1.0.0 \\
    --embeddings --provider voyage --api-key "$VOYAGE_API_KEY"
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import sqlite3
import struct
import sys
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

try:
    import requests  # type: ignore
except Exception:
    requests = None  # pragma: no cover


SCHEMA_VERSION = "3.0"
INDEXER_NAME = "build_external_index_v3.py"


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

def extract_doc_block_before(src: str, pos: int, max_back: int = 2000) -> str:
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
    Fast fingerprint for skip-if-unchanged:
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


class ExternalIndexBuilder:
    SDK_PREFIXES = {
        "nrfx": ["nrfx_", "nrf_", "NRF_"],
        "freertos": ["vTask", "xTask", "xQueue", "xSemaphore", "pvPort", "uxTask", "eTask", "pcTask"],
        "lvgl": ["lv_"],
        "lwip": ["lwip_", "netif_", "ip4_", "ip6_", "pbuf_", "tcp_", "udp_", "dhcp_", "dns_"],
        "tinyusb": ["tud_", "tuh_", "tu_", "tusb_"],
        # generic fallback: no prefix filtering (index all)
        "generic": [],
    }

    SKIP_DIR_PATTERNS = [
        "debug", "release", "build", ".build", "obj", ".obj",
        ".git", ".svn", "docs", "doc", "generated", "gen",
    ]

    SEARCH_DIRS = ["", "src", "include", "source", "inc", "drivers", "hal", "portable", "core", "api", "middleware"]

    SOURCE_EXTS = {".c", ".h", ".hpp", ".cpp", ".cc", ".cxx", ".inc"}

    def __init__(
        self,
        sdk_path: str,
        sdk_name: str,
        output_db: str,
        enable_embeddings: bool = False,
        api_key: str = "",
        embedding_provider: str = "voyage",
        embedding_model: Optional[str] = None,
        enable_fts: bool = True,
        max_file_kb: int = 1024,
        max_chunk_chars: int = 6000,
        skip_if_unchanged: bool = False,
        journal_mode: str = "DELETE",
    ):
        self.sdk_path = Path(sdk_path).resolve()
        self.sdk_name = sdk_name.lower().strip() or "generic"
        self.output_db = Path(output_db).resolve()
        self.enable_embeddings = enable_embeddings
        self.enable_fts = enable_fts
        self.max_file_kb = max_file_kb
        self.max_chunk_chars = max_chunk_chars
        self.skip_if_unchanged = skip_if_unchanged
        self.journal_mode = journal_mode.upper()

        self.prefixes = self.SDK_PREFIXES.get(self.sdk_name, self.SDK_PREFIXES["generic"])

        self.embedding_client: Optional[EmbeddingClient] = None
        self.embedding_provider = embedding_provider
        self.embedding_model = embedding_model

        if self.enable_embeddings:
            if not api_key:
                # installer convenience: try env defaults
                if embedding_provider == "voyage":
                    api_key = os.getenv("VOYAGE_API_KEY", "")
                elif embedding_provider == "openai":
                    api_key = os.getenv("OPENAI_API_KEY", "")
            if not api_key:
                raise ValueError("API key required when --embeddings is enabled")
            self.embedding_client = EmbeddingClient(embedding_provider, api_key, model=embedding_model)

        self.functions_by_id: Dict[int, FunctionInfo] = {}
        self.types_by_id: Dict[int, TypeInfo] = {}
        self._masked_cache: Dict[str, str] = {}

    # -------------
    # Build
    # -------------

    def build(self, version: str) -> None:
        if not self.sdk_path.exists():
            raise FileNotFoundError(f"SDK path not found: {self.sdk_path}")

        self.output_db.parent.mkdir(parents=True, exist_ok=True)

        sources = self._find_sources()
        sdk_fingerprint = _tree_fingerprint(self.sdk_path, sources)

        config = {
            "schema_version": SCHEMA_VERSION,
            "sdk_name": self.sdk_name,
            "enable_fts": bool(self.enable_fts),
            "enable_embeddings": bool(self.embedding_client is not None),
            "embedding_provider": self.embedding_provider if self.embedding_client else None,
            "embedding_model": self.embedding_client.model if self.embedding_client else None,
            "max_file_kb": int(self.max_file_kb),
            "max_chunk_chars": int(self.max_chunk_chars),
        }
        cfg_hash = _config_hash(config)

        if self.skip_if_unchanged and self.output_db.exists():
            kv = _read_metadata_kv(self.output_db)
            if kv.get("sdk_fingerprint") == sdk_fingerprint and kv.get("config_hash") == cfg_hash and kv.get("schema_version") == SCHEMA_VERSION:
                print(f"Index up-to-date: {self.output_db} (sdk={self.sdk_name})")
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

        conn.execute(
            "INSERT INTO metadata VALUES (?,?,?,?,?,?,?,?,?,?,?)",
            (
                version,
                INDEXER_NAME,
                _utc_now_iso(),
                self.sdk_name,
                str(self.sdk_path),
                None,  # git_commit
                self.sdk_name,
                search_mode,
                (self.embedding_provider if self.embedding_client else None),
                (self.embedding_client.model if self.embedding_client else None),
                None,  # embedding_dim (filled after embeddings)
            ),
        )

        conn.execute("INSERT INTO metadata_kv(k, v) VALUES(?,?)", ("schema_version", SCHEMA_VERSION))
        conn.execute("INSERT INTO metadata_kv(k, v) VALUES(?,?)", ("sdk_fingerprint", sdk_fingerprint))
        conn.execute("INSERT INTO metadata_kv(k, v) VALUES(?,?)", ("config_hash", cfg_hash))
        conn.execute("INSERT INTO metadata_kv(k, v) VALUES(?,?)", ("config_json", json.dumps(config, sort_keys=True)))
        conn.commit()

        # Phase 1: parse
        conn.execute("BEGIN;")
        func_count = type_count = chunk_count = 0

        for fp in sources:
            try:
                if fp.stat().st_size > self.max_file_kb * 1024:
                    continue
                content = fp.read_text(encoding="utf-8", errors="ignore")
            except Exception:
                continue

            rel = str(fp.relative_to(self.sdk_path)).replace("\\", "/")
            module = self._detect_module(fp)

            type_count += self._parse_types(conn, content, rel, module)
            func_count += self._parse_functions(conn, content, rel, module)

        # examples as chunks (stable, LLM-friendly)
        chunk_count += self._index_examples_as_chunks(conn)

        conn.execute("COMMIT;")

        # Phase 2: code graph
        conn.execute("BEGIN;")
        self._build_call_graph(conn)
        self._build_type_usage(conn)
        conn.execute("COMMIT;")

        # Phase 3: chunks from functions/types
        conn.execute("BEGIN;")
        chunk_count += self._create_chunks(conn)
        conn.execute("COMMIT;")

        # Phase 4: FTS
        if self.enable_fts:
            self._build_fts(conn)

        # Phase 5: embeddings
        if self.embedding_client:
            emb_dim = self._build_embeddings(conn)
            conn.execute("UPDATE metadata SET embedding_dim=? WHERE rowid=1;", (emb_dim,))
            conn.commit()

        # Phase 6: module schemas
        self._build_module_schemas(conn)

        conn.commit()
        conn.close()

        print(f"Built external index: {self.output_db}")
        print(f"  sdk:      {self.sdk_name}")
        print(f"  functions:{func_count}")
        print(f"  types:    {type_count}")
        print(f"  chunks:   {chunk_count}")
        print(f"  fts:      {'on' if self.enable_fts else 'off'}")
        print(f"  embeddings:{'on' if self.embedding_client else 'off'}")

    # -------------
    # Schema
    # -------------

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

        conn.execute("""
            CREATE TABLE embeddings (
                id INTEGER PRIMARY KEY,
                chunk_id INTEGER,
                embedding BLOB,
                FOREIGN KEY (chunk_id) REFERENCES chunks(id)
            )
        """)
        conn.execute("CREATE INDEX idx_embeddings_chunk ON embeddings(chunk_id);")

        conn.execute("""
            CREATE TABLE module_schemas (
                module TEXT PRIMARY KEY,
                schema_json TEXT
            )
        """)

        conn.commit()

    # -------------
    # Discovery
    # -------------

    def _find_sources(self) -> List[Path]:
        roots = []
        for d in self.SEARCH_DIRS:
            p = (self.sdk_path / d).resolve()
            if p.exists() and p.is_dir():
                roots.append(p)

        files: List[Path] = []
        for root in roots:
            for f in root.rglob("*"):
                if not f.is_file():
                    continue
                if f.suffix.lower() not in self.SOURCE_EXTS:
                    continue
                p = str(f).replace("\\", "/").lower()
                if any(x in p for x in self.SKIP_DIR_PATTERNS):
                    continue
                files.append(f)

        # de-duplicate and sort deterministically
        files = sorted(set(files), key=lambda x: str(x.relative_to(self.sdk_path)).replace("\\", "/"))
        return files

    def _detect_module(self, filepath: Path) -> str:
        n = filepath.name.lower()
        p = str(filepath).replace("\\", "/").lower()
        # SDK-centric modules: keep it basic to avoid misleading labels
        if "ble" in p or "softdevice" in p:
            return "ble"
        if "uart" in p:
            return "uart"
        if "i2c" in p or "twi" in p:
            return "i2c"
        if "spi" in p:
            return "spi"
        if "gpio" in p or "gpiote" in p:
            return "gpio"
        if "usb" in p:
            return "usb"
        if "net" in p or "lwip" in p:
            return "net"
        return self.sdk_name

    def _masked(self, content: str) -> str:
        k = _short_hash(content, 16)
        if k not in self._masked_cache:
            self._masked_cache[k] = mask_comments_and_strings(content)
        return self._masked_cache[k]

    # -------------
    # Parsing (types)
    # -------------

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

            semi = masked.find(";", close_idx)
            if semi == -1 or semi - close_idx > 15:
                continue

            start = m.start()
            end = semi + 1
            block = content[start:end]
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

    # -------------
    # Parsing (functions)
    # -------------

    def _parse_functions(self, conn: sqlite3.Connection, content: str, filepath: str, module: str) -> int:
        masked = self._masked(content)

        header_re = re.compile(
            r'(?P<comment>/\*\*[\s\S]*?\*/\s*)?'
            r'(?P<prefix>(?:^|[\n;{}]))\s*'
            r'(?:template\s*<[^>]*>\s*)?'
            r'(?:extern\s+|static\s+|inline\s+|constexpr\s+|__attribute__\s*\(\([^)]*\)\)\s+)*'
            r'(?P<ret>[\w:\<\>\s\*&]+?)\s+'
            r'(?P<name>[A-Za-z_~]\w*(?:::[A-Za-z_~]\w*)*)\s*'
            r'\((?P<params>[^)]*)\)\s*'
            r'(?P<trailer>(?:const\s+)?(?:noexcept\s+)?)(?P<end>[{;])'
        )

        prefixes = self.prefixes
        count = 0

        for m in header_re.finditer(masked):
            qname = m.group("name")

            # prefix filter for known SDKs; for "generic" we index everything
            if prefixes:
                if not any(qname.startswith(px) for px in prefixes):
                    continue

            # skip keywords / false positives
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

    # -------------
    # Examples as chunks
    # -------------

    def _index_examples_as_chunks(self, conn: sqlite3.Connection) -> int:
        patterns = ("example", "examples", "demo", "sample", "samples", "tutorial")
        count = 0

        files: List[Path] = []
        for f in self.sdk_path.rglob("*"):
            if not f.is_file():
                continue
            if f.suffix.lower() not in self.SOURCE_EXTS:
                continue
            p = str(f).replace("\\", "/").lower()
            if any(x in p for x in self.SKIP_DIR_PATTERNS):
                continue
            if not any(f"/{pat}/" in p or f.name.lower().startswith(pat) for pat in patterns):
                continue
            files.append(f)

        files.sort(key=lambda x: str(x.relative_to(self.sdk_path)).replace("\\", "/"))

        step = 240
        for f in files:
            try:
                if f.stat().st_size > self.max_file_kb * 1024:
                    continue
                content = f.read_text(encoding="utf-8", errors="ignore")
            except Exception:
                continue

            rel = str(f.relative_to(self.sdk_path)).replace("\\", "/")
            module = self._detect_module(f)

            lines = content.splitlines()
            for i in range(0, len(lines), step):
                chunk_lines = lines[i : i + step]
                if not chunk_lines:
                    continue
                start_line = i + 1
                end_line = i + len(chunk_lines)
                chunk_text = "\n".join(chunk_lines)
                title = f"{rel}:{start_line}-{end_line}"
                h = _short_hash(chunk_text, 12)
                conn.execute(
                    """INSERT INTO chunks(entity_type, entity_id, title, module, file, start_line, end_line, kind, content, content_hash)
                       VALUES(?,?,?,?,?,?,?,?,?,?)""",
                    ("example_file", None, title, module, rel, start_line, end_line, "example", chunk_text[: self.max_chunk_chars], h),
                )
                count += 1

        return count

    # -------------
    # Code graph
    # -------------

    def _build_call_graph(self, conn: sqlite3.Connection) -> None:
        conn.execute("DELETE FROM call_graph;")
        call_re = re.compile(r"\b([A-Za-z_]\w*)\s*\(")
        member_re = re.compile(r"(?:\.|->)\s*([A-Za-z_]\w*)\s*\(")
        keywords = {"if", "for", "while", "switch", "return", "sizeof", "alignof", "catch", "static_assert", "new", "delete"}

        for fid, fi in self.functions_by_id.items():
            if not fi.body:
                continue
            masked_body = mask_comments_and_strings(fi.body)
            calls = set(call_re.findall(masked_body)) | set(member_re.findall(masked_body))
            calls = {c for c in calls if c not in keywords and c != fi.name}
            for c in sorted(calls):
                conn.execute("INSERT INTO call_graph VALUES(?,?,?)", (fid, c, "call"))

    def _build_type_usage(self, conn: sqlite3.Connection) -> None:
        conn.execute("DELETE FROM type_usage;")
        t_typedef = re.compile(r"\b([A-Za-z_]\w*_t)\b")
        t_camel = re.compile(r"\b([A-Z][A-Za-z0-9_]{2,})\b")
        blacklist = {"NULL", "TRUE", "FALSE"}

        for fid, fi in self.functions_by_id.items():
            if not fi.body:
                continue
            masked_body = mask_comments_and_strings(fi.body)
            types = set(t_typedef.findall(masked_body)) | set(t_camel.findall(masked_body))
            types = {t for t in types if t not in blacklist}
            for t in sorted(types):
                conn.execute("INSERT INTO type_usage VALUES(?,?)", (fid, t))

    # -------------
    # Chunks
    # -------------

    def _create_chunks(self, conn: sqlite3.Connection) -> int:
        count = 0

        for fid, fi in self.functions_by_id.items():
            if not fi.body and not fi.description:
                continue
            content = fi.signature
            if fi.body:
                content = fi.signature + "\n{\n" + fi.body.strip() + "\n}"
            content = content.strip()
            h = _short_hash(content, 12)
            start_line = fi.line
            end_line = fi.line + max(0, content.count("\n"))
            conn.execute(
                """INSERT INTO chunks(entity_type, entity_id, title, module, file, start_line, end_line, kind, content, content_hash)
                   VALUES(?,?,?,?,?,?,?,?,?,?)""",
                ("function", fid, fi.signature, fi.module, fi.file, start_line, end_line, "function", content[: self.max_chunk_chars], h),
            )
            count += 1

        for tid, ti in self.types_by_id.items():
            content = ti.body.strip()
            if not content:
                continue
            h = _short_hash(content, 12)
            start_line = ti.line
            end_line = ti.line + max(0, content.count("\n"))
            conn.execute(
                """INSERT INTO chunks(entity_type, entity_id, title, module, file, start_line, end_line, kind, content, content_hash)
                   VALUES(?,?,?,?,?,?,?,?,?,?)""",
                ("type", tid, f"{ti.kind} {ti.name}", ti.module, ti.file, start_line, end_line, "type", content[: self.max_chunk_chars], h),
            )
            count += 1

        return count

    # -------------
    # FTS
    # -------------

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

    # -------------
    # Embeddings
    # -------------

    def _build_embeddings(self, conn: sqlite3.Connection) -> Optional[int]:
        assert self.embedding_client is not None
        conn.execute("DELETE FROM embeddings;")

        rows = conn.execute("SELECT id, title, content FROM chunks ORDER BY id;").fetchall()
        inserted = 0
        emb_dim: Optional[int] = None

        for cid, title, content in rows:
            text = (title + "\n\n" + content).strip()
            if len(text) > 9000:
                text = text[:9000]
            vec = self.embedding_client.embed(text)
            if not vec:
                continue
            if emb_dim is None:
                emb_dim = len(vec)
            conn.execute("INSERT INTO embeddings(chunk_id, embedding) VALUES(?,?)", (cid, pack_embedding(vec)))
            inserted += 1

        conn.commit()
        print(f"  embeddings stored: {inserted}")
        return emb_dim

    # -------------
    # Module schemas
    # -------------

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
                "source": self.sdk_name,
            }
            conn.execute("INSERT INTO module_schemas VALUES(?,?)", (mod, json.dumps(schema)))

        conn.commit()


# ----------------------------
# CLI
# ----------------------------

def main() -> None:
    p = argparse.ArgumentParser(description="Build external SDK index (robust, hybrid-ready)")
    p.add_argument("--sdk", required=True, help="SDK root path")
    p.add_argument("--name", required=True, help="SDK name (nrfx, freertos, lvgl, lwip, tinyusb, generic)")
    p.add_argument("--output", required=True, help="Output SQLite DB path")
    p.add_argument("--version", default="0.0.0", help="Index version string")

    p.add_argument("--skip-if-unchanged", action="store_true", help="Skip building if existing index matches current SDK fingerprint + config")

    p.add_argument("--embeddings", action="store_true", help="Enable vector embeddings (hybrid search)")
    p.add_argument("--provider", default="voyage", choices=["voyage","openai","hash","fastembed"], help="Embedding backend (API: voyage/openai; local: hash/fastembed)")
    p.add_argument("--embedding-model", default=None, help="Embedding model name (provider-specific)")
    p.add_argument("--api-key", default="", help="API key (optional; if empty and embeddings enabled, env VOYAGE_API_KEY/OPENAI_API_KEY is used)")

    p.add_argument("--no-fts", action="store_true", help="Disable SQLite FTS5 index (BM25)")
    p.add_argument("--journal", default="DELETE", choices=["WAL", "DELETE"], help="SQLite journal mode")
    p.add_argument("--max-file-kb", type=int, default=1024, help="Skip files larger than this size (KB)")
    p.add_argument("--max-chunk-chars", type=int, default=6000, help="Max chars stored per chunk")

    args = p.parse_args()

    builder = ExternalIndexBuilder(
        sdk_path=args.sdk,
        sdk_name=args.name,
        output_db=args.output,
        enable_embeddings=args.embeddings,
        api_key=args.api_key,
        embedding_provider=args.provider,
        embedding_model=args.embedding_model,
        enable_fts=(not args.no_fts),
        max_file_kb=args.max_file_kb,
        max_chunk_chars=args.max_chunk_chars,
        skip_if_unchanged=args.skip_if_unchanged,
        journal_mode=args.journal,
    )
    builder.build(args.version)

if __name__ == "__main__":
    main()
