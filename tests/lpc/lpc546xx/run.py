#!/usr/bin/env python3
"""Run the LPC546xx host register regressions with the repository's NXP headers."""
from pathlib import Path
import subprocess
import tempfile
root = Path(__file__).resolve().parents[3]
includes = ['tests/lpc/lpc546xx/shim', 'include', 'ARM/NXP/include', 'ARM/NXP/LPC546xx/include',
            'ARM/NXP/LPC546xx/LPC54628/lib/include', 'micro-ecc', 'tests/crypto']
crypto = ['src/device.cpp', 'src/device_intrf.cpp', 'src/coredev/timer.cpp', 'tests/crypto/timer_fixture.cpp',
          'src/crypto/crypto.cpp', 'src/crypto/crypto_aes_modes.cpp', 'src/crypto/crypto_aes_soft.cpp',
          'src/crypto/crypto_softsha256.cpp', 'src/crypto/crypto_softrng.cpp', 'src/crypto/crypto_uecc.cpp',
          'src/crypto/crypto_p256.cpp', 'micro-ecc/uECC.c', 'exemples/crypto/crypto_engine_test.cpp']
# Each test, with the sources it links as their own objects and its defines
tests = [('timer_test.cpp', [], []), ('iopin_test.cpp', [], []), ('i2c_spi_test.cpp', [], []),
         ('uart_dma_test.cpp', [], []), ('i2c_bus_test.cpp', [], []), ('spi_bus_test.cpp', [], []),
         ('wdt_test.cpp', [], []), ('rtc_test.cpp', [], []), ('adc_test.cpp', [], []), ('pwm_test.cpp', [], []),
         ('nvm_test.cpp', ['src/storage/nvm.cpp', 'src/device.cpp', 'src/device_intrf.cpp'], []),
         ('emc_test.cpp', [], []),
         ('sdif_test.cpp', ['src/storage/diskio_impl.cpp', 'src/storage/sdcard_impl.cpp', 'src/device_intrf.cpp',
                            'src/crc.c'], []),
         ('i2s_test.cpp', ['src/device_intrf.cpp', 'src/cfifo.c'], []),
         ('audpll_test.c', [], []),
         ('ethernet_test.cpp', ['src/device_intrf.cpp'], []),
         ('lcd_test.cpp', ['src/display/display.cpp', 'src/display/fonts/syst5x7.c', 'src/device.cpp',
                           'src/device_intrf.cpp'], []),
         ('spifi_test.cpp', ['src/storage/nvm.cpp', 'src/storage/flash.cpp', 'src/device.cpp',
                             'src/device_intrf.cpp', 'src/coredev/spi.cpp'], []),
         ('crypto_test.cpp', crypto, ['-DLPC546XX_SHA_POLL_MAX=64', '-DuECC_ENABLE_VLI_API=1',
                                      '-DCRYPTO_TEST_NO_MAIN', '-DCRYPTO_TEST_SOAK_AES=20',
                                      '-DCRYPTO_TEST_SOAK_PKA=2'])]
san = ['-fsanitize=undefined', '-fno-sanitize-recover=all']
for test, extra, defines in tests:
    with tempfile.TemporaryDirectory() as tmp:
        objects = []
        for n, source in enumerate([str(root / 'tests/lpc/lpc546xx' / test), *[str(root / e) for e in extra]]):
            obj = str(Path(tmp) / f'{n}.o')
            compiler = ['gcc', '-std=gnu17'] if source.endswith('.c') else ['g++', '-std=c++17']
            subprocess.run([*compiler, '-O2', '-Wall', *san, '-fno-pie', '-DLPC54628_SERIES', *defines,
                            *['-I' + str(root / p) for p in includes], '-c', source, '-o', obj], check=True)
            objects.append(obj)
        exe = str(Path(tmp) / 'regression')
        subprocess.run(['g++', *san, '-no-pie', *objects, '-o', exe, '-lpthread'], check=True)
        subprocess.run([exe], check=True)
