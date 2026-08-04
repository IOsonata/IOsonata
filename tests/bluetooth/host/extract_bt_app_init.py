#!/usr/bin/env python3
"""Extract the production nRF54 BtAppInit function for its host test."""

from __future__ import annotations

import argparse
import pathlib
import re
import sys


def extract_function(source: str, signature: str) -> str:
	match = re.search(signature, source, re.MULTILINE)
	if match is None:
		raise ValueError("function signature not found")

	start = match.start()
	brace = source.find("{", match.end())
	if brace < 0:
		raise ValueError("function opening brace not found")

	depth = 0
	index = brace
	state = "code"
	escaped = False

	while index < len(source):
		ch = source[index]
		nxt = source[index + 1] if index + 1 < len(source) else ""

		if state == "line_comment":
			if ch == "\n":
				state = "code"
		elif state == "block_comment":
			if ch == "*" and nxt == "/":
				state = "code"
				index += 1
		elif state == "string":
			if escaped:
				escaped = False
			elif ch == "\\":
				escaped = True
			elif ch == '"':
				state = "code"
		elif state == "char":
			if escaped:
				escaped = False
			elif ch == "\\":
				escaped = True
			elif ch == "'":
				state = "code"
		else:
			if ch == "/" and nxt == "/":
				state = "line_comment"
				index += 1
			elif ch == "/" and nxt == "*":
				state = "block_comment"
				index += 1
			elif ch == '"':
				state = "string"
			elif ch == "'":
				state = "char"
			elif ch == "{":
				depth += 1
			elif ch == "}":
				depth -= 1
				if depth == 0:
					return source[start:index + 1]

		index += 1

	raise ValueError("function closing brace not found")


def host_compat(function: str) -> str:
	"""Apply strict-host adaptations without changing production control flow."""
	old = "\t\t.Role = pCfg->Role,"
	new = "\t\t.Role = static_cast<uint8_t>(pCfg->Role),"
	if old not in function:
		raise ValueError("BtGapCfg_t Role initializer not found")
	return function.replace(old, new, 1)


def main() -> int:
	parser = argparse.ArgumentParser()
	parser.add_argument("source", type=pathlib.Path)
	parser.add_argument("output", type=pathlib.Path)
	args = parser.parse_args()

	try:
		source = args.source.read_text(encoding="utf-8")
		function = extract_function(
			source,
			r"^bool\s+BtAppInit\s*\(const\s+BtAppCfg_t\s*\*\s*pCfg\s*\)",
		)
		function = host_compat(function)
	except (OSError, ValueError) as exc:
		print(f"extract_bt_app_init.py: {exc}", file=sys.stderr)
		return 1

	# BtAppInit resets two nRF54 translation-unit-local state machines before
	# doing any externally visible initialization. Their behavior is tested in
	# the production port; the isolated host test only needs no-op definitions
	# so it can execute the complete production BtAppInit control flow.
	preamble = (
		"// Generated from the production nRF54 BM port. Do not edit.\n"
		"static void SecurePendingReset() {}\n"
		"static void BtSmpOobDataClearInternal() {}\n\n"
	)

	args.output.parent.mkdir(parents=True, exist_ok=True)
	args.output.write_text(preamble + function + "\n", encoding="utf-8")
	return 0


if __name__ == "__main__":
	raise SystemExit(main())
