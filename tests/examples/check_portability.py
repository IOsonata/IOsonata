#!/usr/bin/env python3
"""Check target separation and local Eclipse example source links."""
from pathlib import Path
import re
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]
SUFFIXES = {'.c', '.cpp', '.h', '.hpp', '.S'}
RULES = {
    'vendor include': r'#\s*include\s*[<"](?:nrf|nrfx|stm32|cracen|crypto_rng_nrf|modem_nrf|blueio_board|custom_board|cmsis_|core_cm)[^>"\n]*[>"]',
    'target conditional': r'#\s*(?:if|elif|ifdef|ifndef)\b[^\n]*\b(?:S132|S140|NRFXLIB_SDC|IOSONATA_STM32_WBA|NRF\w*|STM32\w*|__arm__|__riscv)\b',
    'controller register': r'\b(?:NRF_\w+|SCB|SysTick|NVIC|GPIO[A-Z]|RCC|PWR)\s*->',
    'direct CPU operation': r'\b(?:__asm(?:__)?|__disable_irq|__enable_irq|NVIC_\w+)\b',
    'CMSIS clock variable': r'\bSystemCoreClock\b',
    'TinyUSB MCU selection': r'#\s*define\s+CFG_TUSB_MCU\b',
}
errors = []
files = sorted(p for p in (ROOT / 'exemples').rglob('*') if p.suffix in SUFFIXES)
for path in files:
    source = path.read_text()
    source = re.sub(r'/\*.*?\*/|//[^\n]*', '', source, flags=re.S)
    for name, pattern in RULES.items():
        if re.search(pattern, source):
            errors.append(f'{path.relative_to(ROOT)}: {name}')

links = 0
for path in ROOT.rglob('.project'):
    if '.git' in path.parts:
        continue
    for element in ET.parse(path).iter('locationURI'):
        uri = element.text or ''
        match = re.fullmatch(r'PARENT-(\d+)-PROJECT_LOC/(.+)', uri)
        if not match:
            continue
        relative = match[2]
        if not ('exemples/' in relative and Path(relative).suffix in SUFFIXES):
            continue
        base = path.parent
        for _ in range(int(match[1])):
            base = base.parent
        if not (base / relative).is_file():
            errors.append(f'{path.relative_to(ROOT)}: missing {relative}')
        links += 1

if errors:
    raise SystemExit('\n'.join(errors))
print(f'PASS: {len(files)} shared source files; {links} Eclipse example links')
