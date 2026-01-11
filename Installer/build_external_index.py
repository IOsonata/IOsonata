#!/usr/bin/env python3
"""
build_external_index.py (SDK indexer)

Purpose
- Run at install time to build a baseline, hybrid-ready index for ONE SDK root into ONE SQLite DB.
- GitHub cannot build this because SDKs are not available there.
- Embeddings are intentionally OPTIONAL at install time. Runtime can add/update embeddings using
  update_index_embeddings_v2.py (same updater used for IOsonata repo index).

Key design points
- Produces the same "chunks.uid" + (chunk_uid, provider, model) embeddings contract as IOsonata index.
- Creates FTS5 table (bm25) over chunks for lexical retrieval.
- Optional deep passes (call graph / type usage) are OFF by default for installer speed.

Schema expectations for runtime embedding updater (update_index_embeddings_v2.py)
- chunks.uid exists (TEXT UNIQUE)
- embeddings PK: (chunk_uid, provider, model)
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
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, Iterator, List, Optional, Sequence, Set, Tuple

try:
    import requests  # optional; only needed if embeddings enabled with provider voyage/openai
except Exception:
    requests = None  # type: ignore


SCHEMA_VERSION = "3.3"

DEFAULT_MAX_FILE_KB = 512
DEFAULT_MAX_CHUNK_CHARS = 6000
DEFAULT_EXAMPLE_CAP = 250

SOURCE_SUFFIXES = {
    ".c", ".h", ".hpp", ".hh", ".hxx",
    ".cpp", ".cc", ".cxx",
    ".inc", ".inl",
    ".S", ".s",
}

SKIP_DIR_NAMES = {
    ".git", ".github", ".svn",
    ".hg", ".idea", ".vscode",
    ".metadata", ".settings",
    "build", "out", "dist",
    "__pycache__", "node_modules",
    ".iosonata",
}

# Common "not code" extensions we skip aggressively even if large trees contain them.
SKIP_FILE_SUFFIXES = {
    ".png", ".jpg", ".jpeg", ".gif", ".bmp", ".tiff", ".ico",
    ".pdf", ".zip", ".7z", ".rar", ".gz", ".bz2", ".xz",
    ".bin", ".hex", ".elf", ".map", ".o", ".a", ".so", ".dylib", ".dll",
    ".mp3", ".mp4", ".wav", ".avi",
}

EXAMPLE_PATH_HINTS = ("example", "examples", "sample", "samples", "demo", "demos", "tutorial")


def _now_tag() -> str:
    return time.strftime("%H:%M:%S", time.gmtime())


class Logger:
    def __init__(self) -> None:
        self.t0 = time.monotonic()

    def log(self, msg: str) -> None:
        dt = time.monotonic() - self.t0
        mm = int(dt // 60)
        ss = int(dt % 60)
        print(f"[{mm:02d}:{ss:02d}] {msg}", flush=True)


def sha1_text(s: str) -> str:
    return hashlib.sha1(s.encode("utf-8", errors="ignore")).hexdigest()


def sha1_bytes(b: bytes) -> str:
    return hashlib.sha1(b).hexdigest()


def pack_f32le(vec: Sequence[float]) -> bytes:
    return struct.pack("<" + "f" * len(vec), *vec)


def is_probable_source_file(path: Path) -> bool:
    suf = path.suffix
    if suf in SKIP_FILE_SUFFIXES:
        return False
    if suf in SOURCE_SUFFIXES:
        return True
    return False


def read_text_file(path: Path, max_kb: int) -> Optional[str]:
    try:
        st = path.stat()
        if st.st_size > max_kb * 1024:
            return None
        data = path.read_bytes()
        # Try utf-8; fall back to latin-1 with replacement.
        try:
            return data.decode("utf-8")
        except Exception:
            return data.decode("latin-1", errors="replace")
    except Exception:
        return None


_COMMENT_RE = re.compile(
    r"""
    //.*?$                       |  # C++ single-line
    /\*[\s\S]*?\*/               |  # C multi-line
    "(?:\\.|[^"\\])*"            |  # double-quoted string
    '(?:\\.|[^'\\])*'               # single-quoted char/string
    """,
    re.MULTILINE | re.VERBOSE,
)


def mask_comments_and_strings(text: str) -> str:
    """
    Replace comments and string/char literals with spaces, preserving newlines and text length.
    This keeps line/column mapping stable while preventing brace/paren confusion.
    """

    def repl(m: re.Match) -> str:
        s = m.group(0)
        # Preserve line breaks to keep line numbers stable
        return re.sub(r"[^\n]", " ", s)

    return _COMMENT_RE.sub(repl, text)


def line_of_offset(text: str, offset: int) -> int:
    # 1-based line number
    return text.count("\n", 0, max(0, offset)) + 1


def brace_match(text: str, open_pos: int) -> Optional[int]:
    """
    Given text and position of '{', return index of matching '}' (inclusive), or None.
    Assumes comments/strings already masked.
    """
    depth = 0
    n = len(text)
    i = open_pos
    while i < n:
        c = text[i]
        if c == "{":
            depth += 1
        elif c == "}":
            depth -= 1
            if depth == 0:
                return i
        i += 1
    return None


# Heuristic function header finder:
# - captures function name before '('
# - avoids control keywords
# FAST regex - avoids catastrophic backtracking
_FUNC_HEADER_RE = re.compile(
    r"""
    (?:^|[;\n\}])\s*                  # boundary + whitespace
    (?:(?:extern|static|inline|const|volatile|unsigned|signed)\s+)*  # qualifiers
    ([A-Za-z_]\w*(?:\s*\*)*)\s+      # return type (group 1)
    ([A-Za-z_]\w*)\s*                  # function name (group 2)
    \(                                  # args begin
    """,
    re.VERBOSE | re.MULTILINE,
)

_TYPE_START_RE = re.compile(
    r"""
    \b(?:
        typedef\s+(?P<tdef>(struct|enum|union))\s*(?P<tag1>[A-Za-z_]\w*)?\s*\{ |
        (?P<kw>(struct|enum|union|class))\s+(?P<tag2>[A-Za-z_]\w*)\s*\{
    )
    """,
    re.VERBOSE | re.MULTILINE,
)


@dataclass
class Chunk:
    uid: str
    kind: str
    title: str
    signature: str
    module: str
    file: str
    start_line: int
    end_line: int
    content: str
    content_hash: str


class EmbeddingClient:
    def __init__(self, provider: str, model: str, api_key: Optional[str], timeout_s: int = 60) -> None:
        self.provider = provider
        self.model = model
        self.api_key = api_key
        self.timeout_s = timeout_s

    def embed_batch(self, texts: Sequence[str]) -> List[Optional[List[float]]]:
        if self.provider == "hash":
            return self._embed_hash_batch(texts)
        if self.provider == "fastembed":
            return self._embed_fastembed_batch(texts)
        if self.provider == "voyage":
            return self._embed_voyage_batch(texts)
        if self.provider == "openai":
            return self._embed_openai_batch(texts)
        raise ValueError(f"Unknown provider: {self.provider}")

    def _embed_hash_batch(self, texts: Sequence[str], dim: int = 512) -> List[Optional[List[float]]]:
        out: List[Optional[List[float]]] = []
        token_re = re.compile(r"[A-Za-z_]\w+")
        for t in texts:
            vec = [0.0] * dim
            for tok in token_re.findall(t):
                h = hashlib.sha1(tok.encode("utf-8", errors="ignore")).digest()
                idx = int.from_bytes(h[:4], "little") % dim
                sign = 1.0 if (h[4] & 1) == 0 else -1.0
                vec[idx] += sign
            # L2 normalize
            norm = sum(v * v for v in vec) ** 0.5
            if norm > 0:
                vec = [v / norm for v in vec]
            out.append(vec)
        return out

    def _embed_fastembed_batch(self, texts: Sequence[str]) -> List[Optional[List[float]]]:
        try:
            from fastembed import TextEmbedding  # type: ignore
        except Exception as e:
            raise RuntimeError("provider=fastembed requires `pip install fastembed`") from e
        model_name = self.model or "BAAI/bge-small-en-v1.5"
        emb = TextEmbedding(model_name=model_name)
        vectors = list(emb.embed(list(texts)))
        return [list(v) for v in vectors]

    def _embed_voyage_batch(self, texts: Sequence[str]) -> List[Optional[List[float]]]:
        if requests is None:
            raise RuntimeError("provider=voyage requires the 'requests' package")
        if not self.api_key:
            raise RuntimeError("provider=voyage requires --api-key or VOYAGE_API_KEY")
        url = "https://api.voyageai.com/v1/embeddings"
        headers = {"Authorization": f"Bearer {self.api_key}", "Content-Type": "application/json"}
        payload = {"model": self.model, "input": list(texts)}
        
        # Retry with exponential backoff for rate limits
        for attempt in range(5):
            try:
                r = requests.post(url, headers=headers, json=payload, timeout=self.timeout_s)
                if r.status_code == 429:
                    import time
                    wait = min(60, (2 ** attempt) + 1)
                    print(f"[rate-limit] waiting {wait}s...", file=sys.stderr)
                    time.sleep(wait)
                    continue
                r.raise_for_status()
                break
            except requests.exceptions.Timeout:
                if attempt < 4:
                    import time
                    time.sleep(2 ** attempt)
                    continue
                raise
        
        data = r.json()
        items = sorted(data.get("data") or [], key=lambda x: x.get("index", 0))
        out: List[Optional[List[float]]] = [item.get("embedding") for item in items]
        while len(out) < len(texts):
            out.append(None)
        return out

    def _embed_openai_batch(self, texts: Sequence[str]) -> List[Optional[List[float]]]:
        if requests is None:
            raise RuntimeError("provider=openai requires the 'requests' package")
        if not self.api_key:
            raise RuntimeError("provider=openai requires --api-key or OPENAI_API_KEY")
        url = "https://api.openai.com/v1/embeddings"
        headers = {"Authorization": f"Bearer {self.api_key}", "Content-Type": "application/json"}
        payload = {"model": self.model, "input": list(texts)}
        
        # Retry with exponential backoff for rate limits
        for attempt in range(5):
            try:
                r = requests.post(url, headers=headers, json=payload, timeout=self.timeout_s)
                if r.status_code == 429:
                    wait = min(60, (2 ** attempt) + 1)
                    print(f"[rate-limit] waiting {wait}s...", file=sys.stderr)
                    time.sleep(wait)
                    continue
                r.raise_for_status()
                break
            except requests.exceptions.Timeout:
                if attempt < 4:
                    time.sleep(2 ** attempt)
                    continue
                raise
        
        data = r.json()
        items = sorted(data.get("data") or [], key=lambda x: x.get("index", 0))
        out: List[Optional[List[float]]] = [item.get("embedding") for item in items]
        while len(out) < len(texts):
            out.append(None)
        return out


class SDKIndexer:
    def __init__(
        self,
        sdk_root: Path,
        sdk_name: str,
        out_db: Path,
        max_file_kb: int,
        max_chunk_chars: int,
        include_examples: bool,
        example_cap: int,
        enable_fts: bool,
        enable_callgraph: bool,
        enable_type_usage: bool,
        embeddings: bool,
        embed_provider: str,
        embed_model: str,
        api_key: Optional[str],
        skip_if_unchanged: bool,
        deterministic: bool,
        verbose: bool,
    ) -> None:
        self.sdk_root = sdk_root
        self.sdk_name = sdk_name
        self.out_db = out_db
        self.max_file_kb = max_file_kb
        self.max_chunk_chars = max_chunk_chars
        self.include_examples = include_examples
        self.example_cap = example_cap
        self.enable_fts = enable_fts
        self.enable_callgraph = enable_callgraph
        self.enable_type_usage = enable_type_usage
        self.embeddings = embeddings
        self.embed_provider = embed_provider
        self.embed_model = embed_model
        self.api_key = api_key
        self.skip_if_unchanged = skip_if_unchanged
        self.deterministic = deterministic
        self.verbose = verbose

        self.log = Logger()
        self._sources: List[Path] = []
        self._sdk_fingerprint: str = ""

    def _connect(self, path: Path) -> sqlite3.Connection:
        conn = sqlite3.connect(str(path))
        conn.execute("PRAGMA foreign_keys=ON;")
        # Installer-friendly: avoid -wal/-shm by default.
        if self.deterministic:
            conn.execute("PRAGMA journal_mode=DELETE;")
            conn.execute("PRAGMA synchronous=FULL;")
        else:
            conn.execute("PRAGMA journal_mode=DELETE;")
            conn.execute("PRAGMA synchronous=NORMAL;")
        conn.execute("PRAGMA temp_store=MEMORY;")
        conn.execute("PRAGMA cache_size=-20000;")  # ~20MB
        return conn

    def _create_schema(self, conn: sqlite3.Connection) -> None:
        conn.executescript(
            """
            CREATE TABLE IF NOT EXISTS metadata_kv (
              k TEXT PRIMARY KEY,
              v TEXT NOT NULL
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
              content TEXT,
              content_hash TEXT
            );
            CREATE INDEX IF NOT EXISTS idx_chunks_kind ON chunks(kind);
            CREATE INDEX IF NOT EXISTS idx_chunks_file ON chunks(file);

            CREATE TABLE IF NOT EXISTS functions (
              id INTEGER PRIMARY KEY,
              name TEXT,
              signature TEXT,
              file TEXT,
              start_line INTEGER,
              end_line INTEGER,
              body_hash TEXT
            );
            CREATE INDEX IF NOT EXISTS idx_functions_name ON functions(name);

            CREATE TABLE IF NOT EXISTS types (
              id INTEGER PRIMARY KEY,
              kind TEXT,
              name TEXT,
              signature TEXT,
              file TEXT,
              start_line INTEGER,
              end_line INTEGER,
              body_hash TEXT
            );
            CREATE INDEX IF NOT EXISTS idx_types_name ON types(name);

            CREATE TABLE IF NOT EXISTS call_graph (
              caller TEXT,
              callee TEXT,
              file TEXT,
              caller_line INTEGER
            );

            CREATE TABLE IF NOT EXISTS type_usage (
              type_name TEXT,
              context TEXT,
              file TEXT,
              line INTEGER
            );

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
            """
        )
        # FTS created later if enabled.

    def _metadata_get(self, conn: sqlite3.Connection, k: str) -> Optional[str]:
        row = conn.execute("SELECT v FROM metadata_kv WHERE k=?", (k,)).fetchone()
        return row[0] if row else None

    def _metadata_set(self, conn: sqlite3.Connection, k: str, v: str) -> None:
        conn.execute(
            "INSERT INTO metadata_kv(k,v) VALUES(?,?) ON CONFLICT(k) DO UPDATE SET v=excluded.v",
            (k, v),
        )

    def _config_json(self, version: str) -> str:
        cfg = {
            "schema_version": SCHEMA_VERSION,
            "sdk_name": self.sdk_name,
            "version": version,
            "max_file_kb": self.max_file_kb,
            "max_chunk_chars": self.max_chunk_chars,
            "include_examples": self.include_examples,
            "example_cap": self.example_cap,
            "enable_fts": self.enable_fts,
            "enable_callgraph": self.enable_callgraph,
            "enable_type_usage": self.enable_type_usage,
            # embeddings config does NOT affect baseline content; store for reference only.
        }
        return json.dumps(cfg, sort_keys=True)

    def _find_sources_and_fingerprint(self) -> Tuple[List[Path], str]:
        """
        One walk: collect candidate source files and compute a stable SDK fingerprint over them.
        Fingerprint covers: relative path + size + mtime_ns.
        """
        h = hashlib.sha1()
        sources: List[Path] = []

        root = self.sdk_root.resolve()
        for dirpath, dirnames, filenames in os.walk(root):
            # prune directories in-place
            dirnames[:] = [d for d in dirnames if d not in SKIP_DIR_NAMES]
            # quick skip of hidden dirs
            dirnames[:] = [d for d in dirnames if not d.startswith(".") or d in (".",)]
            for fn in filenames:
                p = Path(dirpath) / fn
                if not is_probable_source_file(p):
                    continue
                try:
                    st = p.stat()
                except Exception:
                    continue
                if st.st_size > self.max_file_kb * 1024:
                    continue
                rel = str(p.relative_to(root)).replace("\\", "/")
                sources.append(p)
                # update fingerprint
                h.update(rel.encode("utf-8"))
                h.update(b"\0")
                h.update(str(st.st_size).encode("ascii"))
                h.update(b"\0")
                h.update(str(getattr(st, "st_mtime_ns", int(st.st_mtime * 1e9))).encode("ascii"))
                h.update(b"\n")
        sources.sort()
        return sources, h.hexdigest()

    def _is_example_path(self, rel_path: str) -> bool:
        lower = rel_path.lower()
        return any(h in lower for h in EXAMPLE_PATH_HINTS)

    def _extract_types(self, masked: str, original: str) -> List[Tuple[str, str, int, int, str]]:
        """
        Return list of (kind, name, start_off, end_off, signature_text)
        where end_off is inclusive offset of closing '}' or ';' if present.
        """
        out: List[Tuple[str, str, int, int, str]] = []
        for m in _TYPE_START_RE.finditer(masked):
            start = m.start()
            kw = (m.group("tdef") or m.group("kw") or "").strip()
            tag = (m.group("tag1") or m.group("tag2") or "").strip()
            # find '{' position
            brace_pos = masked.find("{", m.start())
            if brace_pos < 0:
                continue
            close = brace_match(masked, brace_pos)
            if close is None:
                continue
            # include trailing ';' if present
            end = close
            j = close + 1
            while j < len(masked) and masked[j].isspace():
                j += 1
            if j < len(masked) and masked[j] == ";":
                end = j
            # signature line guess
            sig_head = original[m.start():brace_pos].strip().splitlines()[-1].strip()
            # Determine name for typedef-anon blocks
            name = tag
            if not name:
                # try after close brace: typedef struct { } NAME;
                tail = original[close + 1 : min(len(original), close + 200)]
                m2 = re.search(r"\b([A-Za-z_]\w+)\s*;", tail)
                if m2:
                    name = m2.group(1)
                else:
                    name = "<anonymous>"
            sig = sig_head if sig_head else f"{kw} {name}".strip()
            out.append((kw or "type", name, start, end, sig))
        return out

    def _extract_functions(self, masked: str, original: str, type_spans: List[Tuple[int, int]]) -> List[Tuple[str, int, int, str]]:
        """
        Return list of (name, start_off, end_off, signature_text)
        For SDKs we focus on top-level style functions. We skip matches inside type spans
        (inline methods) to avoid duplicate/churn.
        """
        out: List[Tuple[str, int, int, str]] = []

        def inside_type(pos: int) -> bool:
            for a, b in type_spans:
                if a <= pos <= b:
                    return True
            return False

        for m in _FUNC_HEADER_RE.finditer(masked):
            # signature start is after boundary char; keep close to real header
            hdr_start = m.start(1)  # return type group
            name = m.group(2)  # function name group
            
            # Skip ALL_CAPS macros
            if name.isupper() and len(name) > 2:
                continue
            # Skip keywords
            if name in {"if", "for", "while", "switch", "return", "sizeof", "catch", "defined"}:
                continue
            if inside_type(m.start()):
                continue
            # find '{' after header
            # We search from end of match for '{' but ensure we see ')'
            scan_start = m.end()
            # find closing paren first to reduce false positives
            close_paren = masked.find(")", scan_start)
            if close_paren < 0:
                continue
            brace_pos = masked.find("{", close_paren)
            if brace_pos < 0:
                continue
            close = brace_match(masked, brace_pos)
            if close is None:
                continue
            sig_text = original[hdr_start:brace_pos].strip()
            # normalize signature single-line
            sig_one = " ".join(sig_text.split())
            out.append((name, hdr_start, close, sig_one))
        return out

    def _extract_calls(self, body_masked: str) -> Set[str]:
        calls: Set[str] = set()
        # capture tokens followed by '(' but not keywords
        for m in re.finditer(r"\b([A-Za-z_]\w+)\s*\(", body_masked):
            tok = m.group(1)
            if tok in {"if", "for", "while", "switch", "return", "sizeof", "catch"}:
                continue
            calls.add(tok)
        return calls

    def _extract_type_usage(self, body_masked: str, type_names: Set[str]) -> Set[str]:
        used: Set[str] = set()
        # quick token scan
        for m in re.finditer(r"\b([A-Za-z_]\w+)\b", body_masked):
            tok = m.group(1)
            if tok in type_names:
                used.add(tok)
        return used

    def _mk_chunk_uid(self, rel_file: str, kind: str, start_line: int, end_line: int, content_hash: str) -> str:
        return sha1_text(f"{rel_file}|{kind}|{start_line}|{end_line}|{content_hash}")

    def build(self, version: str) -> None:
        self.out_db.parent.mkdir(parents=True, exist_ok=True)

        self.log.log(f"index build start (schema={SCHEMA_VERSION})")
        self.log.log(f"sdk: {self.sdk_name} root={self.sdk_root}")

        sources, fp = self._find_sources_and_fingerprint()
        self._sources = sources
        self._sdk_fingerprint = fp

        self.log.log(f"scanning source files... ({len(sources)} candidates)")

        cfg_json = self._config_json(version)
        cfg_hash = sha1_text(cfg_json)

        # Skip-if-unchanged check against existing DB
        if self.skip_if_unchanged and self.out_db.exists():
            try:
                conn_old = self._connect(self.out_db)
                sv = self._metadata_get(conn_old, "schema_version")
                old_fp = self._metadata_get(conn_old, "sdk_fingerprint")
                old_cfg = self._metadata_get(conn_old, "config_hash")
                conn_old.close()
                if sv == SCHEMA_VERSION and old_fp == fp and old_cfg == cfg_hash:
                    self.log.log("no changes detected; skipping rebuild.")
                    return
            except Exception:
                # fall through to rebuild
                pass

        tmp = self.out_db.with_suffix(self.out_db.suffix + ".tmp")
        if tmp.exists():
            try:
                tmp.unlink()
            except Exception:
                pass

        conn = self._connect(tmp)
        try:
            self._create_schema(conn)
            self._metadata_set(conn, "schema_version", SCHEMA_VERSION)
            self._metadata_set(conn, "sdk_name", self.sdk_name)
            self._metadata_set(conn, "built_utc", str(int(time.time())))
            self._metadata_set(conn, "sdk_fingerprint", fp)
            self._metadata_set(conn, "config_json", cfg_json)
            self._metadata_set(conn, "config_hash", cfg_hash)
            self._metadata_set(conn, "version", version)

            # Clean tables for rebuild
            conn.execute("DELETE FROM functions;")
            conn.execute("DELETE FROM types;")
            conn.execute("DELETE FROM call_graph;")
            conn.execute("DELETE FROM type_usage;")
            conn.execute("DELETE FROM chunks;")
            if self.enable_fts:
                conn.execute("DROP TABLE IF EXISTS fts_chunks;")

            chunks: List[Chunk] = []
            all_type_names: Set[str] = set()

            # Pass 1: parse and stage chunks
            for idx, path in enumerate(sources, start=1):
                rel = str(path.relative_to(self.sdk_root)).replace("\\", "/")
                txt = read_text_file(path, self.max_file_kb)
                if txt is None:
                    continue
                # Skip very large files that cause regex slowdown
                if len(txt) > 150_000:
                    continue
                masked = mask_comments_and_strings(txt)

                types = self._extract_types(masked, txt)
                type_spans: List[Tuple[int, int]] = []
                for kind, name, a, b, sig in types:
                    type_spans.append((a, b))
                    all_type_names.add(name)
                    start_line = line_of_offset(txt, a)
                    end_line = line_of_offset(txt, b)
                    block = txt[a : b + 1]
                    content = (sig + "\n" + block).strip()
                    if len(content) > self.max_chunk_chars:
                        content = content[: self.max_chunk_chars] + "\n/* …truncated… */"
                    chash = sha1_text(content)
                    uid = self._mk_chunk_uid(rel, "type", start_line, end_line, chash)
                    chunks.append(
                        Chunk(
                            uid=uid,
                            kind="type",
                            title=name,
                            signature=sig,
                            module=self.sdk_name,
                            file=rel,
                            start_line=start_line,
                            end_line=end_line,
                            content=content,
                            content_hash=chash,
                        )
                    )
                    conn.execute(
                        "INSERT INTO types(kind,name,signature,file,start_line,end_line,body_hash) VALUES(?,?,?,?,?,?,?)",
                        (kind, name, sig, rel, start_line, end_line, chash),
                    )

                funcs = self._extract_functions(masked, txt, type_spans)
                for name, a, b, sig in funcs:
                    start_line = line_of_offset(txt, a)
                    end_line = line_of_offset(txt, b)
                    body = txt[a : b + 1]
                    content = (sig + "\n" + body).strip()
                    if len(content) > self.max_chunk_chars:
                        content = content[: self.max_chunk_chars] + "\n/* …truncated… */"
                    chash = sha1_text(content)
                    uid = self._mk_chunk_uid(rel, "function", start_line, end_line, chash)
                    chunks.append(
                        Chunk(
                            uid=uid,
                            kind="function",
                            title=name,
                            signature=sig,
                            module=self.sdk_name,
                            file=rel,
                            start_line=start_line,
                            end_line=end_line,
                            content=content,
                            content_hash=chash,
                        )
                    )
                    conn.execute(
                        "INSERT INTO functions(name,signature,file,start_line,end_line,body_hash) VALUES(?,?,?,?,?,?)",
                        (name, sig, rel, start_line, end_line, chash),
                    )

                    if self.enable_callgraph or self.enable_type_usage:
                        # deeper analysis uses masked body
                        body_masked = masked[a : b + 1]
                        if self.enable_callgraph:
                            for callee in self._extract_calls(body_masked):
                                conn.execute(
                                    "INSERT INTO call_graph(caller,callee,file,caller_line) VALUES(?,?,?,?)",
                                    (name, callee, rel, start_line),
                                )
                        if self.enable_type_usage and all_type_names:
                            for tname in self._extract_type_usage(body_masked, all_type_names):
                                conn.execute(
                                    "INSERT INTO type_usage(type_name,context,file,line) VALUES(?,?,?,?)",
                                    (tname, name, rel, start_line),
                                )

                if self.verbose and (idx % 200 == 0):
                    self.log.log(f"parsed {idx}/{len(sources)} files (chunks={len(chunks)})")

            # Examples: index a limited set of example-like files as chunks
            if self.include_examples:
                ex_count = 0
                for path in sources:
                    rel = str(path.relative_to(self.sdk_root)).replace("\\", "/")
                    if not self._is_example_path(rel):
                        continue
                    txt = read_text_file(path, self.max_file_kb)
                    if txt is None:
                        continue
                    start_line, end_line = 1, txt.count("\n") + 1
                    title = rel.split("/")[-1]
                    content = txt.strip()
                    if len(content) > self.max_chunk_chars:
                        content = content[: self.max_chunk_chars] + "\n/* …truncated… */"
                    chash = sha1_text(content)
                    uid = self._mk_chunk_uid(rel, "example", start_line, end_line, chash)
                    chunks.append(
                        Chunk(
                            uid=uid,
                            kind="example",
                            title=title,
                            signature="",
                            module=self.sdk_name,
                            file=rel,
                            start_line=start_line,
                            end_line=end_line,
                            content=content,
                            content_hash=chash,
                        )
                    )
                    ex_count += 1
                    if ex_count >= self.example_cap:
                        break
                self.log.log(f"examples indexed: {ex_count} (cap={self.example_cap})")

            # Bulk insert chunks
            conn.executemany(
                """
                INSERT INTO chunks(uid,kind,title,signature,module,file,start_line,end_line,content,content_hash)
                VALUES(?,?,?,?,?,?,?,?,?,?)
                """,
                [
                    (
                        c.uid,
                        c.kind,
                        c.title,
                        c.signature,
                        c.module,
                        c.file,
                        c.start_line,
                        c.end_line,
                        c.content,
                        c.content_hash,
                    )
                    for c in chunks
                ],
            )
            conn.commit()
            self.log.log(f"chunks written: {len(chunks)}")

            # FTS build
            if self.enable_fts:
                self.log.log("building FTS5 index...")
                try:
                    conn.execute(
                        "CREATE VIRTUAL TABLE fts_chunks USING fts5(title, content, file, kind, module);"
                    )
                    conn.execute(
                        """
                        INSERT INTO fts_chunks(rowid, title, content, file, kind, module)
                        SELECT id, title, content, file, kind, module FROM chunks;
                        """
                    )
                except sqlite3.OperationalError as e:
                    # FTS5 not available
                    self.log.log(f"FTS5 unavailable: {e}. Proceeding without fts_chunks.")
            else:
                self.log.log("FTS disabled.")

            # Optional embeddings at install time (usually OFF)
            if self.embeddings:
                self.log.log(f"embedding chunks (provider={self.embed_provider}, model={self.embed_model}) ...")
                client = EmbeddingClient(
                    provider=self.embed_provider,
                    model=self.embed_model,
                    api_key=self.api_key,
                    timeout_s=60,
                )
                # Embed only function/type by default; include examples too if you want (kinds list can be adjusted)
                kinds_to_embed = {"function", "type"}
                rows = conn.execute(
                    "SELECT uid, content FROM chunks WHERE kind IN ('function','type') ORDER BY id"
                ).fetchall()
                batch_size = 64
                utc_now = int(time.time())
                for i in range(0, len(rows), batch_size):
                    batch = rows[i : i + batch_size]
                    uids = [r[0] for r in batch]
                    texts = [r[1] for r in batch]
                    vecs = client.embed_batch(texts)
                    # determine dim
                    dim = 0
                    for v in vecs:
                        if v:
                            dim = len(v)
                            break
                    ins = []
                    for uid, v in zip(uids, vecs):
                        if not v:
                            continue
                        ins.append((uid, self.embed_provider, self.embed_model, len(v), pack_f32le(v), utc_now))
                    if ins:
                        conn.executemany(
                            """
                            INSERT INTO embeddings(chunk_uid, provider, model, dim, embedding, updated_utc)
                            VALUES(?,?,?,?,?,?)
                            ON CONFLICT(chunk_uid, provider, model) DO UPDATE SET
                              dim=excluded.dim,
                              embedding=excluded.embedding,
                              updated_utc=excluded.updated_utc
                            """,
                            ins,
                        )
                    if self.verbose:
                        self.log.log(f"embedded {min(i+batch_size, len(rows))}/{len(rows)}")

            conn.commit()
        finally:
            try:
                conn.close()
            except Exception:
                pass

        # Atomic replace
        if self.out_db.exists():
            try:
                self.out_db.unlink()
            except Exception:
                pass
        tmp.replace(self.out_db)
        self.log.log(f"done: {self.out_db}")


def parse_args(argv: Sequence[str]) -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Build SDK code index DBs. Scans a root folder for SDK subdirectories and indexes each one.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Index all SDKs in ./external folder:
  python3 build_external_index.py

  # Index all SDKs in custom folder:
  python3 build_external_index.py --root ~/my_sdks

  # Index a single SDK:
  python3 build_external_index.py --sdk ~/external/nRF5_SDK

  # Index specific SDKs by name:
  python3 build_external_index.py --only nRF5_SDK,ESP32_SDK
"""
    )
    p.add_argument("--root", default="external", help="Root folder containing SDK subdirectories (default: external)")
    p.add_argument("--sdk", default="", help="Index a single SDK at this path (overrides --root)")
    p.add_argument("--only", default="", help="Comma-separated list of SDK folder names to index (default: all)")
    p.add_argument("--exclude", default="", help="Comma-separated list of SDK folder names to skip")
    p.add_argument("--version", default="1.0.0", help="Index version string")
    p.add_argument("--max-file-kb", type=int, default=DEFAULT_MAX_FILE_KB, help="Skip files larger than this (KB)")
    p.add_argument("--max-chunk-chars", type=int, default=DEFAULT_MAX_CHUNK_CHARS, help="Truncate chunk content to this length")
    p.add_argument("--no-examples", action="store_true", help="Do not index example-like files as chunks")
    p.add_argument("--example-cap", type=int, default=DEFAULT_EXAMPLE_CAP, help="Max example files to index")
    p.add_argument("--no-fts", action="store_true", help="Do not build FTS5 index")
    p.add_argument("--callgraph", action="store_true", help="Enable call graph extraction (slower)")
    p.add_argument("--type-usage", action="store_true", help="Enable type usage extraction (slower)")
    p.add_argument("--skip-if-unchanged", action="store_true", help="Skip rebuild if SDK fingerprint + config unchanged")
    p.add_argument("--deterministic", action="store_true", help="Favor deterministic SQLite settings (slower)")
    p.add_argument("--verbose", action="store_true", help="Verbose progress logging")

    # Optional embeddings at install time (normally OFF)
    p.add_argument("--embeddings", action="store_true", help="Compute embeddings during install (optional; runtime updater can do this instead)")
    p.add_argument("--provider", choices=["voyage", "openai", "hash", "fastembed"], default="hash", help="Embedding provider")
    p.add_argument("--embedding-model", default="", help="Embedding model (provider-specific). For hash, ignore; for fastembed set model name.")
    p.add_argument("--api-key", default="", help="API key (voyage/openai). Can also use VOYAGE_API_KEY / OPENAI_API_KEY env vars.")
    return p.parse_args(list(argv))


def find_sdk_directories(root: Path, only: List[str], exclude: List[str]) -> List[Path]:
    """Find SDK subdirectories in root folder."""
    sdks = []
    if not root.exists() or not root.is_dir():
        return sdks
    
    for item in sorted(root.iterdir()):
        if not item.is_dir():
            continue
        name = item.name
        # Skip hidden folders and common non-SDK folders
        if name.startswith("."):
            continue
        if name.lower() in {"build", "out", "dist", "__pycache__", "node_modules"}:
            continue
        # Apply filters
        if only and name not in only:
            continue
        if name in exclude:
            continue
        sdks.append(item)
    
    return sdks


def index_single_sdk(
    sdk_path: Path,
    sdk_name: str,
    out_db: Path,
    args: argparse.Namespace,
    api_key: Optional[str],
) -> bool:
    """Index a single SDK. Returns True on success."""
    try:
        idx = SDKIndexer(
            sdk_root=sdk_path,
            sdk_name=sdk_name,
            out_db=out_db,
            max_file_kb=args.max_file_kb,
            max_chunk_chars=args.max_chunk_chars,
            include_examples=not args.no_examples,
            example_cap=args.example_cap,
            enable_fts=not args.no_fts,
            enable_callgraph=bool(args.callgraph),
            enable_type_usage=bool(args.type_usage),
            embeddings=bool(args.embeddings),
            embed_provider=args.provider,
            embed_model=args.embedding_model or (
                "voyage-code-2" if args.provider == "voyage" else "text-embedding-3-small" if args.provider == "openai" else ""
            ),
            api_key=api_key,
            skip_if_unchanged=bool(args.skip_if_unchanged),
            deterministic=bool(args.deterministic),
            verbose=bool(args.verbose),
        )
        idx.build(args.version)
        return True
    except Exception as e:
        print(f"ERROR indexing {sdk_name}: {e}", file=sys.stderr)
        return False


def main(argv: Optional[Sequence[str]] = None) -> None:
    args = parse_args(argv or sys.argv[1:])

    # API key selection (if needed)
    api_key = args.api_key.strip() or None
    if args.provider == "voyage":
        api_key = api_key or os.getenv("VOYAGE_API_KEY")
    elif args.provider == "openai":
        api_key = api_key or os.getenv("OPENAI_API_KEY")

    # Single SDK mode
    if args.sdk.strip():
        sdk_path = Path(args.sdk).expanduser().resolve()
        if not sdk_path.exists() or not sdk_path.is_dir():
            print(f"ERROR: SDK path not found: {sdk_path}", file=sys.stderr)
            sys.exit(2)
        
        sdk_name = sdk_path.name
        # Output in parent's .extsdk folder
        out_db = sdk_path.parent / ".extsdk" / f"{sdk_name}.db"
        out_db.parent.mkdir(parents=True, exist_ok=True)
        
        print(f"Indexing single SDK: {sdk_name}")
        print(f"  Path: {sdk_path}")
        print(f"  Output: {out_db}")
        
        success = index_single_sdk(sdk_path, sdk_name, out_db, args, api_key)
        sys.exit(0 if success else 1)

    # Multi-SDK mode: scan root folder
    root = Path(args.root).expanduser().resolve()
    if not root.exists() or not root.is_dir():
        print(f"ERROR: Root folder not found: {root}", file=sys.stderr)
        sys.exit(2)

    # Parse filters
    only = [x.strip() for x in args.only.split(",") if x.strip()] if args.only else []
    exclude = [x.strip() for x in args.exclude.split(",") if x.strip()] if args.exclude else []

    # Find SDK directories
    sdks = find_sdk_directories(root, only, exclude)
    if not sdks:
        print(f"No SDK directories found in: {root}", file=sys.stderr)
        if only:
            print(f"  Filter (--only): {only}", file=sys.stderr)
        sys.exit(1)

    # Create .extsdk folder in root
    extsdk_dir = root / ".extsdk"
    extsdk_dir.mkdir(parents=True, exist_ok=True)

    print(f"Found {len(sdks)} SDK(s) in: {root}")
    for sdk in sdks:
        print(f"  - {sdk.name}")
    print(f"Output folder: {extsdk_dir}")
    print()

    # Index each SDK
    success_count = 0
    fail_count = 0
    for sdk_path in sdks:
        sdk_name = sdk_path.name
        out_db = extsdk_dir / f"{sdk_name}.db"
        
        print(f"=" * 60)
        print(f"Indexing: {sdk_name}")
        print(f"=" * 60)
        
        if index_single_sdk(sdk_path, sdk_name, out_db, args, api_key):
            success_count += 1
        else:
            fail_count += 1
        print()

    # Summary
    print("=" * 60)
    print(f"SUMMARY: {success_count} succeeded, {fail_count} failed")
    print(f"Index DBs stored in: {extsdk_dir}")
    print("=" * 60)
    
    sys.exit(0 if fail_count == 0 else 1)


if __name__ == "__main__":
    main()
