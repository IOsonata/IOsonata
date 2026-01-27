#!/usr/bin/env python3
"""
External SDK Index Builder v8 - Unified Schema

Indexes external SDKs (nRF5_SDK, FreeRTOS, LVGL, etc.) into SQLite databases.
Uses unified schema shared with build_rag_index.py and build_knowledge_db.py.

v6: Initial unified schema
v7: Manifest support (not used by external SDKs)
v8: Base class hierarchy support (not used by external SDKs, but schema compatible)

Usage:
  python3 build_external_index.py              # Index all SDKs
  python3 build_external_index.py --update-embeddings --provider openai --api-key $KEY
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
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Iterator, List, Optional, Tuple

# Import unified schema
from rag_schema import (
    SCHEMA_VERSION, COMPRESS_LEVEL,
    KIND_FUNCTION, KIND_TYPE, KIND_EXAMPLE, KIND_ASSEMBLY,
    PERIPH_NONE, PERIPH_MAP, PROVIDER_MAP,
    DB_TYPE_EXTERNAL, SOURCE_SUFFIXES_ALL,
    db_connect, ensure_schema, fts_rebuild, set_standard_meta,
    compress, decompress, sha256, StringCache,
    compute_source_fingerprint, build_line_index, idx_to_line,
    get_meta,
)

print = functools.partial(print, flush=True)

# =============================================================================
# BENCHMARK DATA
# =============================================================================

@dataclass
class BuildStats:
    """Statistics for a single SDK build."""
    name: str
    elapsed: float = 0.0
    files: int = 0
    chunks: int = 0
    funcs: int = 0
    types: int = 0
    examples: int = 0
    size_kb: float = 0.0
    skipped: bool = False
    error: str = ""


@dataclass 
class BenchmarkReport:
    """Collects timing and stats across all SDK builds."""
    start_time: float = field(default_factory=time.time)
    sdk_stats: List[BuildStats] = field(default_factory=list)
    
    def add(self, stats: BuildStats):
        self.sdk_stats.append(stats)
    
    def total_elapsed(self) -> float:
        return time.time() - self.start_time
    
    def print_summary(self):
        """Print a formatted benchmark summary."""
        print("\n" + "=" * 70)
        print("BENCHMARK SUMMARY")
        print("=" * 70)
        
        # Header
        print(f"{'SDK':<25} {'Time':>8} {'Files':>7} {'Chunks':>8} {'Size':>10} {'Status':<10}")
        print("-" * 70)
        
        total_files = 0
        total_chunks = 0
        total_size = 0.0
        build_time = 0.0
        
        for s in self.sdk_stats:
            if s.skipped:
                status = "skipped"
                time_str = "-"
            elif s.error:
                status = "ERROR"
                time_str = "-"
            else:
                status = "OK"
                time_str = f"{s.elapsed:.1f}s"
                build_time += s.elapsed
                total_files += s.files
                total_chunks += s.chunks
                total_size += s.size_kb
            
            size_str = f"{s.size_kb:.0f} KB" if s.size_kb > 0 else "-"
            print(f"{s.name:<25} {time_str:>8} {s.files:>7} {s.chunks:>8} {size_str:>10} {status:<10}")
        
        print("-" * 70)
        
        # Totals
        built = sum(1 for s in self.sdk_stats if not s.skipped and not s.error)
        skipped = sum(1 for s in self.sdk_stats if s.skipped)
        errors = sum(1 for s in self.sdk_stats if s.error)
        
        print(f"{'TOTAL':<25} {build_time:>7.1f}s {total_files:>7} {total_chunks:>8} {total_size:>7.0f} KB")
        print(f"\nSDKs: {len(self.sdk_stats)} total, {built} built, {skipped} skipped, {errors} errors")
        print(f"Wall time: {self.total_elapsed():.1f}s (build time: {build_time:.1f}s)")
        print("=" * 70)

# =============================================================================
# CONFIG
# =============================================================================

DEFAULT_IGNORE_DIRS = {
    ".git", ".github", ".metadata", ".settings", ".vscode", ".idea",
    "build", "out", "dist", "Debug", "Release", "OSX", "linux", "win32",
    "node_modules", "__pycache__", ".pytest_cache",
    "doc", "docs", "documentation", "doxygen",
}
IGNORE_DIR_PREFIXES = ("Debug", "Release", "cmake-build-")
EXAMPLE_DIR_HINTS = frozenset(("example", "examples", "sample", "samples", "demo", "demos", "test", "tests", "exemples"))


def detect_peripheral(path: str) -> int:
    p = path.lower()
    for kw, periph_id in PERIPH_MAP.items():
        if kw in p:
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
    """Iterate over function definitions (with body only, skip declarations)."""
    masked = _mask_comments(src)
    for m in _FUNC_RE.finditer(masked):
        ret, name, params, tail = m.group(1), m.group(2), m.group(3), m.group(4)
        if name.startswith("_"):
            continue
        # Skip declarations (no body) - only index definitions
        has_body = tail == "{"
        if not has_body:
            continue
        sig = f"{' '.join(ret.split())} {name}({' '.join(params.split())})"
        yield name, sig, ret.strip(), m.start(), m.end(), has_body


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
# SDK INDEXER
# =============================================================================

class SDKIndexer:
    def __init__(self, enable_fts: bool = True, max_file_kb: int = 256, max_chunk: int = 6000,
                 example_cap: int = 500, verbose: bool = False):
        self.enable_fts = enable_fts
        self.max_bytes = max(32 * 1024, max_file_kb * 1024)
        self.max_chunk = max(1000, max_chunk)
        self.example_cap = example_cap
        self.verbose = verbose

    def _should_ignore(self, d: str) -> bool:
        return d in DEFAULT_IGNORE_DIRS or d.startswith(".") or d.startswith(IGNORE_DIR_PREFIXES)

    def _iter_files(self, sdk_path: Path) -> Iterator[Path]:
        for dp, dn, fn in os.walk(sdk_path):
            dn[:] = [d for d in dn if not self._should_ignore(d)]
            dn.sort()
            fn.sort()
            for f in fn:
                p = Path(dp) / f
                if p.suffix.lower() in SOURCE_SUFFIXES_ALL:
                    yield p

    def build_sdk_db(self, sdk_path: Path, name: str, out_db: Path, version: str, skip_unchanged: bool) -> BuildStats:
        """Build SDK database and return build statistics."""
        out_db.parent.mkdir(parents=True, exist_ok=True)
        
        # Initialize stats
        build_stats = BuildStats(name=name)
        
        # Compute source fingerprint for reliable skip-if-unchanged
        current_fp = compute_source_fingerprint(sdk_path, SOURCE_SUFFIXES_ALL, DEFAULT_IGNORE_DIRS, IGNORE_DIR_PREFIXES)
        
        if skip_unchanged and out_db.exists():
            try:
                # Read-only check to avoid creating WAL sidecar files
                conn = sqlite3.connect(f"file:{out_db}?mode=ro", uri=True, timeout=5)
                stored_fp = get_meta(conn, 'fingerprint')
                conn.close()
                if stored_fp == current_fp:
                    print(f"  [skip] {name} (unchanged)")
                    build_stats.skipped = True
                    build_stats.size_kb = out_db.stat().st_size / 1024
                    return build_stats
            except:
                pass  # DB corrupt or no fingerprint, rebuild

        if out_db.exists():
            out_db.unlink()

        t0 = time.time()
        conn = db_connect(out_db, baseline=True)
        ensure_schema(conn, enable_fts=self.enable_fts, include_mcu=False, include_devices=False)

        # Standardized metadata
        set_standard_meta(conn, DB_TYPE_EXTERNAL, version, fingerprint=current_fp)

        file_cache = StringCache(conn, "files")
        module_cache = StringCache(conn, "modules")

        stats = {"files": 0, "chunks": 0, "funcs": 0, "types": 0, "examples": 0}

        for path in self._iter_files(sdk_path):
            stats["files"] += 1
            try:
                rel = str(path.relative_to(sdk_path)).replace("\\", "/")
            except:
                rel = str(path).replace("\\", "/")

            file_id = file_cache.get_id(rel)
            module_id = module_cache.get_id(rel.split("/")[0] if "/" in rel else "root")
            periph = detect_peripheral(rel)

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
                if len(lines) > 200:
                    src_chunk = "\n".join(lines[:150]) + "\n/* snip */\n" + "\n".join(lines[-30:])
                else:
                    src_chunk = src
                content = src_chunk[:self.max_chunk]
                conn.execute(
                    "INSERT INTO chunks(kind,periph,file_id,module_id,line_start,line_end,title,content,hash) VALUES(?,?,?,?,?,?,?,?,?)",
                    (KIND_EXAMPLE, periph, file_id, module_id, 1, len(lines), f"Example:{path.stem}",
                     compress(content), sha256(content))
                )
                stats["examples"] += 1
                stats["chunks"] += 1

            # Types
            for kind, name_, s_idx, e_idx in iter_types(src):
                block = src[s_idx:e_idx+1]
                line = idx_to_line(nl_idx, s_idx)
                content = f"{kind} {name_}\n{block}"[:self.max_chunk]
                conn.execute(
                    "INSERT INTO chunks(kind,periph,file_id,module_id,line_start,line_end,title,content,hash) VALUES(?,?,?,?,?,?,?,?,?)",
                    (KIND_TYPE, periph, file_id, module_id, line, line + block.count("\n"),
                     f"{kind} {name_}", compress(content), sha256(content))
                )
                stats["types"] += 1
                stats["chunks"] += 1

            # Functions - signature only (no body needed for API reference)
            for fname, sig, ret, s_idx, e_idx, has_body in iter_funcs(src):
                line = idx_to_line(nl_idx, s_idx)
                # API functions: signature is enough for RAG
                # Implementation details are in examples
                content = sig[:self.max_chunk]
                conn.execute(
                    "INSERT INTO chunks(kind,periph,file_id,module_id,line_start,line_end,title,signature,content,hash) VALUES(?,?,?,?,?,?,?,?,?,?)",
                    (KIND_FUNCTION, periph, file_id, module_id, line, line,
                     fname, compress(sig), compress(content), sha256(content))
                )
                conn.execute(
                    "INSERT INTO api(name,periph,file_id,line,signature,ret_type) VALUES(?,?,?,?,?,?)",
                    (fname, periph, file_id, line, compress(sig), compress(ret) if ret else None)
                )
                stats["funcs"] += 1
                stats["chunks"] += 1

        conn.commit()

        if self.enable_fts:
            fts_rebuild(conn)

        conn.execute("VACUUM")
        conn.close()

        elapsed = time.time() - t0
        size_kb = out_db.stat().st_size / 1024
        
        # Update build stats
        build_stats.elapsed = elapsed
        build_stats.files = stats["files"]
        build_stats.chunks = stats["chunks"]
        build_stats.funcs = stats["funcs"]
        build_stats.types = stats["types"]
        build_stats.examples = stats["examples"]
        build_stats.size_kb = size_kb
        
        print(f"    [{_fmt(elapsed)}] {name}: {stats['files']} files, {stats['chunks']} chunks ({size_kb:.0f}KB)")
        return build_stats


def update_embeddings(db_path: Path, provider: str, api_key: Optional[str], model: str,
                      batch_size: int = 0, max_new: int = 0) -> int:
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


def discover_sdks(root: Path, include: List[str], exclude: List[str]) -> List[Tuple[str, Path]]:
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
    p = argparse.ArgumentParser(description="Build SDK indexes v8 (unified schema)")
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
    p.add_argument("--provider", default="openai")
    p.add_argument("--api-key", default="")
    p.add_argument("--model", default="")
    p.add_argument("--batch-size", type=int, default=0)
    p.add_argument("--max-new", type=int, default=0)
    p.add_argument("--no-benchmark", action="store_true", help="Disable benchmark summary")
    args = p.parse_args()

    if not args.api_key:
        args.api_key = os.environ.get('OPENAI_API_KEY') or os.environ.get('VOYAGE_API_KEY', '')

    indexer = SDKIndexer(enable_fts=not args.no_fts, verbose=args.verbose)

    # Single SDK mode
    if args.sdk:
        sdk_path = Path(args.sdk)
        name = args.name or sanitize(sdk_path.name)
        out_db = Path(args.output) if args.output else (sdk_path.parent / ".extsdk" / f"{name}.db")
        if args.update_embeddings:
            n = update_embeddings(out_db, args.provider, args.api_key or None, args.model, args.batch_size, args.max_new)
            print(f"✓ {name}: {n} embeddings")
        else:
            report = BenchmarkReport()
            stats = indexer.build_sdk_db(sdk_path, name, out_db, args.version, args.skip_if_unchanged)
            report.add(stats)
            if not args.no_benchmark:
                report.print_summary()
        return

    # Multi-SDK mode
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
                n = update_embeddings(db, args.provider, args.api_key or None, args.model, args.batch_size, args.max_new)
                print(f"  {name}: {n}")
                total += n
        print(f"\n✓ Total: {total} embeddings")
    else:
        report = BenchmarkReport()
        for name, sdk_path in sdks:
            try:
                stats = indexer.build_sdk_db(sdk_path, name, output_dir / f"{name}.db", args.version, args.skip_if_unchanged)
                report.add(stats)
            except Exception as e:
                print(f"    [ERROR] {name}: {e}")
                error_stats = BuildStats(name=name, error=str(e))
                report.add(error_stats)
        
        if not args.no_benchmark:
            report.print_summary()
        else:
            print(f"\n✓ Indexed {len(sdks)} SDKs")


if __name__ == "__main__":
    main()
