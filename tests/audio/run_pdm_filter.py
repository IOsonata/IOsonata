#!/usr/bin/env python3
"""Compare the production CIC3 decimator with an independent finite FIR sum."""
from collections import deque
import os
from pathlib import Path
import random
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]


def oracle(data):
    # Convolve three length-64 boxcar FIRs, without CIC integrators/combs.
    kernel = [1]
    for stage in range(3):
        next_kernel = [0] * (len(kernel) + 63)
        for index, weight in enumerate(kernel):
            for offset in range(64):
                next_kernel[index + offset] += weight
        kernel = next_kernel
    history = deque([0] * len(kernel), maxlen=len(kernel))
    result = []
    count = 0
    for byte in data:
        for bit in range(7, -1, -1):
            history.appendleft(1 if byte & (1 << bit) else -1)
            count += 1
            if count % 64 == 0:
                value = sum(weight * sample for weight, sample in zip(kernel, history))
                value = value // 8 if value >= 0 else -((-value) // 8)
                result.append(max(-32768, min(32767, value)))
    return result


def main():
    rng = random.Random(0x50444D)
    cases = [bytes(4096), bytes([255]) * 4096, bytes([0x55, 0xAA]) * 2048,
             bytes(rng.randrange(256) for _ in range(8192))]
    with tempfile.TemporaryDirectory(prefix='pdm-filter-') as temp:
        temp = Path(temp)
        fixture = []
        for index, data in enumerate(cases):
            expected = oracle(data)
            fixture += [f'static const uint8_t s_Data{index}[] = {{' + ','.join(map(str, data)) + '};',
                        f'static const int16_t s_Expected{index}[] = {{' + ','.join(map(str, expected)) + '};']
        source = '#include <cassert>\n#include <cstdio>\n#include "audio/pdm_filter.h"\n'
        source += '\n'.join(fixture)
        source += '''
static void Check(const uint8_t *pData, unsigned Count, const int16_t *pExpected)
{
	PdmFilter_t filter;
	unsigned output = 0;
	int16_t sample = 12345;

	PdmFilterReset(&filter);
	for (unsigned i = 0; i < Count; i++)
	{
		sample = 12345;
		bool ready = PdmFilterByte(&filter, pData[i], &sample);
		assert(ready == (i % 8 == 7));
		if (ready)
		{
			assert(sample == pExpected[output++]);
		}
		else
		{
			assert(sample == 12345);
		}
	}
	assert(output == Count / 8);
	// Reset at a partial-byte-block boundary must discard the old history.
	for (unsigned offset = 1; offset < 8; offset++)
	{
		PdmFilterReset(&filter);
		for (unsigned i = 0; i < offset; i++)
		{
			PdmFilterByte(&filter, 0xFF, &sample);
		}
		PdmFilterReset(&filter);
		for (unsigned i = 0; i < 8; i++)
		{
			PdmFilterByte(&filter, pData[i], &sample);
		}
		assert(sample == pExpected[0]);
	}
}
int main()
{
'''
        for index in range(len(cases)):
            source += f'\tCheck(s_Data{index}, sizeof(s_Data{index}), s_Expected{index});\n'
        source += '\tstd::puts("PDM CIC3: independent FIR oracle, saturation, modulo wrap and reset PASS");\n}\n'
        source_file = temp / 'filter_test.cpp'
        source_file.write_text(source)
        for opt in ('-O0', '-Os'):
            exe = temp / 'filter_test'
            subprocess.run([os.environ.get('CXX', 'g++'), '-std=gnu++23', opt,
                            '-Wall', '-Wextra', '-Werror', '-fsanitize=address,undefined',
                            '-fno-omit-frame-pointer', '-I' + str(ROOT / 'include'),
                            str(source_file), str(ROOT / 'src/audio/pdm_filter.cpp'),
                            '-o', str(exe)], check=True)
            subprocess.run([str(exe)], check=True)
            print(f'{opt}: {sum(len(data) // 8 for data in cases)} PCM samples match FIR reference', flush=True)


if __name__ == '__main__':
    main()
