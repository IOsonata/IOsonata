#!/usr/bin/env python3
"""Build a host test from C fixtures and C++ production sources.

The LPC USB controller sources are C++ files. The fixtures and register
models are C. run() takes one gcc command with both kinds of sources,
compiles each C file with gcc and each C++ file with g++, then links with
g++. Headers given with -include are wrapped in extern "C" for the C++
files, so the test hooks they declare keep C linkage.

Copyright (c) 2026, I-SYST inc. MIT License.
"""
from pathlib import Path
import subprocess
import tempfile

# C options that g++ reports as invalid for C++
C_ONLY = ('-Wno-pointer-to-int-cast', '-Wno-int-to-pointer-cast',
          '-Wno-incompatible-pointer-types', '-Wno-discarded-qualifiers',
          '-Wno-missing-prototypes', '-Wno-implicit-function-declaration')
VALUE_OPTIONS = ('-include', '-o', '-x', '-MF', '-MT')


def is_cxx(path):
    return path.endswith('.cpp') or path.endswith('.cc')


def run(cmd, cxx_std='-std=gnu++20', check=True):
    """Compile and link cmd, a gcc command line, by source language."""
    sources = []
    compile_opts = []
    link_opts = []
    includes = []
    output = None
    args = list(cmd[1:])
    index = 0
    while index < len(args):
        arg = args[index]
        if arg in VALUE_OPTIONS:
            value = args[index + 1]
            if arg == '-o':
                output = value
            elif arg == '-include':
                includes.append(value)
            index += 2
            continue
        if not arg.startswith('-') and (arg.endswith('.c') or is_cxx(arg)):
            sources.append(arg)
        elif arg.startswith('-Wl,') or arg in ('-no-pie', '-static'):
            link_opts.append(arg)
        else:
            compile_opts.append(arg)
            if arg.startswith('-fsanitize') or arg.startswith('-fno-sanitize'):
                link_opts.append(arg)
        index += 1

    with tempfile.TemporaryDirectory() as work:
        work = Path(work)
        wrapped = []
        for number, header in enumerate(includes):
            path = work / ('c_linkage_' + str(number) + '.h')
            path.write_text('extern "C" {\n#include "' + header + '"\n}\n')
            wrapped.append(str(path))
        objects = []
        for number, source in enumerate(sources):
            obj = str(work / (str(number) + '.o'))
            if is_cxx(source):
                opts = [cxx_std if opt.startswith('-std=') else opt
                        for opt in compile_opts if opt not in C_ONLY]
                forced = [item for header in wrapped for item in ('-include', header)]
                compiler = ['g++', *opts, '-D_Static_assert=static_assert', *forced]
            else:
                forced = [item for header in includes for item in ('-include', header)]
                compiler = ['gcc', *compile_opts, *forced]
            result = subprocess.run([*compiler, '-c', source, '-o', obj], check=check)
            if result.returncode != 0:
                return result
            objects.append(obj)
        return subprocess.run(['g++', *link_opts, *objects, '-o', output], check=check)
