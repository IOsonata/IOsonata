#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 I-SYST inc.
"""Shared TLS certificate tests using local OpenSSL and production crypto.

All peers and generated credentials are local test fixtures. No network
downloads or ARM toolchain are needed. The complete suite runs at O0/O2
with AddressSanitizer and UndefinedBehaviorSanitizer.
"""

import base64
import os
from pathlib import Path
import select
import socket
import subprocess
import tempfile
import time


ROOT = Path(__file__).resolve().parents[2]
UECC = Path(os.environ.get('UECC_ROOT', ROOT / 'micro-ecc'))
SOURCES = ['src/net/tls.cpp', 'src/crypto/x509.cpp',
           'src/crypto/crypto_softaes.cpp', 'src/crypto/crypto_softsha256.cpp',
           'src/crypto/crypto_p256.cpp', 'src/crypto/crypto_uecc.cpp',
           'src/device.cpp', 'src/device_intrf.cpp']
ENV = dict(os.environ, ASAN_OPTIONS='detect_leaks=0:abort_on_error=1',
           UBSAN_OPTIONS='halt_on_error=1:print_stacktrace=1')


def openssl(*arguments):
    result = subprocess.run(['openssl', *map(str, arguments)], check=True,
                            stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    return result.stdout


def der_item(data, offset):
    tag = data[offset]
    length = data[offset + 1]
    start = offset + 2
    if length & 0x80:
        count = length & 0x7f
        assert 0 < count <= 4
        length = int.from_bytes(data[start:start + count], 'big')
        start += count
    assert start + length <= len(data)
    return tag, start, start + length


def der_encode(tag, data):
    length = len(data)
    if length < 128:
        header = bytes([tag, length])
    else:
        count = (length.bit_length() + 7) // 8
        header = bytes([tag, 0x80 | count]) + length.to_bytes(count, 'big')
    return header + data


def der_children(data):
    _, start, end = der_item(data, 0)
    assert end == len(data)
    children = []
    while start < end:
        _, _, child_end = der_item(data, start)
        children.append(data[start:child_end])
        start = child_end
    return children


def write_certificate(pem, data):
    encoded = base64.b64encode(data).decode('ascii')
    pem.write_text('-----BEGIN CERTIFICATE-----\n' +
                   '\n'.join(encoded[n:n + 64] for n in range(0, len(encoded), 64)) +
                   '\n-----END CERTIFICATE-----\n', encoding='ascii')


def make_certificates(directory):
    """Return locally generated credential paths for TLS and X.509 checks."""
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    paths = {}
    serial = 0

    def certificate(name, subject, extensions, issuer=None, days=30):
        nonlocal serial
        serial += 1
        key = directory / (name + '.key')
        pem = directory / (name + '.pem')
        csr = directory / (name + '.csr')
        config = directory / (name + '.cnf')
        der = directory / (name + '.der')
        config.write_text('[req]\ndistinguished_name = dn\n[dn]\n'
                          '[v3]\n' + extensions + '\n', encoding='ascii')
        openssl('ecparam', '-name', 'prime256v1', '-genkey', '-noout', '-out', key)
        if issuer is None:
            openssl('req', '-new', '-x509', '-sha256', '-key', key, '-out', pem,
                    '-subj', '/CN=' + subject, '-days', days,
                    '-set_serial', serial, '-config', config, '-extensions', 'v3')
        else:
            openssl('req', '-new', '-key', key, '-out', csr, '-subj', '/CN=' + subject)
            openssl('x509', '-req', '-in', csr, '-CA', paths[issuer]['pem'],
                    '-CAkey', paths[issuer]['key'], '-set_serial', serial,
                    '-days', days, '-sha256', '-extfile', config,
                    '-extensions', 'v3', '-out', pem)
        der.write_bytes(openssl('x509', '-in', pem, '-outform', 'DER'))
        paths[name] = {'pem': pem, 'der': der, 'key': key}
        return paths[name]

    ca = ('basicConstraints = critical,CA:TRUE,pathlen:2\n'
          'keyUsage = critical,keyCertSign,cRLSign\n'
          'subjectKeyIdentifier = hash\n'
          'authorityKeyIdentifier = keyid:always')
    intermediate = ca.replace('pathlen:2', 'pathlen:0')
    leaf = ('basicConstraints = critical,CA:FALSE\n'
            'keyUsage = critical,digitalSignature\n'
            'extendedKeyUsage = serverAuth\n'
            'subjectAltName = DNS:localhost,DNS:server.iosonata.test\n'
            'subjectKeyIdentifier = hash\n'
            'authorityKeyIdentifier = keyid:always')
    certificate('root', 'IOsonata TLS test root', ca, days=3650)
    certificate('other_root', 'IOsonata unrelated test root', ca, days=3650)
    certificate('intermediate', 'IOsonata TLS test intermediate', intermediate,
                'root', days=365)
    certificate('server', 'localhost', leaf, 'intermediate')
    certificate('direct', 'localhost', leaf, 'root')
    certificate('wildcard', '*.mqtt.example.test',
                leaf.replace('DNS:localhost,DNS:server.iosonata.test',
                             'DNS:*.mqtt.example.test'), 'intermediate')
    certificate('wrong_name', 'localhost',
                leaf.replace('DNS:localhost,DNS:server.iosonata.test', 'DNS:wrong.test'),
                'intermediate')
    certificate('no_san', 'localhost',
                leaf.replace('subjectAltName = DNS:localhost,DNS:server.iosonata.test\n', ''),
                'intermediate')
    certificate('wrong_eku', 'localhost', leaf.replace('serverAuth', 'clientAuth'),
                'intermediate')
    certificate('wrong_usage', 'localhost',
                leaf.replace('digitalSignature', 'keyAgreement'), 'intermediate')
    certificate('critical_extension', 'localhost',
                leaf + '\n1.2.3.4 = critical,DER:05:00', 'intermediate')
    certificate('noncritical_extension', 'localhost',
                leaf + '\n1.2.3.4 = DER:05:00', 'intermediate')
    certificate('large_server', 'localhost',
                leaf + '\n1.2.3.5 = DER:04:82:09:C4:' + ':'.join(['00'] * 2500),
                'intermediate')
    certificate('not_ca', 'IOsonata invalid intermediate',
                intermediate.replace('CA:TRUE,pathlen:0', 'CA:FALSE'), 'root')
    certificate('non_ca_server', 'localhost', leaf, 'not_ca')
    certificate('no_ca_usage', 'IOsonata unsigned intermediate',
                intermediate.replace('keyCertSign,cRLSign', 'digitalSignature'), 'root')
    certificate('no_ca_usage_server', 'localhost', leaf, 'no_ca_usage')
    certificate('client_ca', 'IOsonata client-only intermediate',
                intermediate + '\nextendedKeyUsage = clientAuth', 'root')
    certificate('client_ca_server', 'localhost', leaf, 'client_ca')
    certificate('short_ca', 'IOsonata short-lived intermediate', intermediate,
                'root', days=1)
    certificate('short_ca_server', 'localhost', leaf, 'short_ca')
    certificate('short_root', 'IOsonata short-lived root', ca, days=1)
    certificate('short_root_server', 'localhost', leaf, 'short_root')
    certificate('sub_intermediate', 'IOsonata excess intermediate', intermediate,
                'intermediate')
    certificate('pathlen_server', 'localhost', leaf, 'sub_intermediate')
    certificate('name_constrained', 'IOsonata constrained intermediate',
                intermediate + '\nnameConstraints = critical,permitted;DNS:.example.test',
                'root')
    certificate('constrained_server', 'localhost', leaf, 'name_constrained')
    certificate('aki_issuer', 'localhost',
                leaf.replace('keyid:always', 'keyid,issuer:always'), 'intermediate')
    certificate('invalid_wildcard', 'localhost',
                leaf.replace('DNS:localhost,DNS:server.iosonata.test',
                             'DNS:f*.mqtt.example.test'), 'intermediate')
    certificate('noncritical_ca', 'IOsonata noncritical CA',
                intermediate.replace('basicConstraints = critical,', 'basicConstraints = '),
                'root')
    certificate('noncritical_ca_server', 'localhost', leaf, 'noncritical_ca')
    certificate('noncritical_constraint', 'IOsonata noncritical constraint',
                intermediate + '\nnameConstraints = permitted;DNS:.example.test', 'root')
    certificate('noncritical_constraint_server', 'localhost', leaf, 'noncritical_constraint')
    certificate('pathlen_no_usage', 'IOsonata pathlen without usage',
                intermediate.replace('keyUsage = critical,keyCertSign,cRLSign\n', ''), 'root')
    certificate('pathlen_no_usage_server', 'localhost', leaf, 'pathlen_no_usage')

    def modified_server(name, fields):
        original = der_children(paths['server']['der'].read_bytes())
        tbs = der_encode(0x30, b''.join(fields))
        tbs_path = directory / (name + '.tbs')
        tbs_path.write_bytes(tbs)
        signature = openssl('dgst', '-sha256', '-sign',
                            paths['intermediate']['key'], tbs_path)
        data = der_encode(0x30, tbs + original[1] + der_encode(0x03, b'\x00' + signature))
        der = directory / (name + '.der')
        pem = directory / (name + '.pem')
        der.write_bytes(data)
        write_certificate(pem, data)
        paths[name] = {'der': der, 'pem': pem}

    original_fields = der_children(der_children(paths['server']['der'].read_bytes())[0])
    assert original_fields[-1][0] == 0xa3
    original_extensions = der_children(der_children(original_fields[-1])[0])
    fields = list(original_fields)
    fields[-1] = der_encode(0xa3, der_encode(0x30,
                           b''.join(original_extensions) + original_extensions[0]))
    modified_server('duplicate_extension', fields)
    fields = list(original_fields)
    validity = der_children(fields[4])
    validity[0] = der_encode(0x17, b'260230000000Z')
    fields[4] = der_encode(0x30, b''.join(validity))
    modified_server('invalid_calendar', fields)
    fields = list(original_fields)
    fields[4] = der_encode(0x30, der_encode(0x17, b'600101000000Z') +
                           der_encode(0x18, b'20510101000000Z'))
    modified_server('wide_validity', fields)
    fields = list(original_fields)
    fields[5] = der_encode(0x30, b'')
    modified_server('empty_subject_noncritical_san', fields)
    extensions = []
    for extension in original_extensions:
        children = der_children(extension)
        if children[0] == b'\x06\x03\x55\x1d\x11':
            children.insert(1, b'\x01\x01\xff')
        extensions.append(der_encode(0x30, b''.join(children)))
    fields[-1] = der_encode(0xa3, der_encode(0x30, b''.join(extensions)))
    modified_server('empty_subject_critical_san', fields)
    fields = list(original_fields)
    fields[-1] = fields[-1].replace(b'localhost', b'local\x00ost')
    modified_server('nul_san', fields)

    # An issuer-signed certificate carrying an invalid affine P-256 point.
    # Re-sign the altered TBSCertificate so signature corruption cannot hide
    # a missing public-key validity check.
    original = paths['server']['der'].read_bytes()
    _, outer_start, _ = der_item(original, 0)
    _, _, tbs_end = der_item(original, outer_start)
    _, _, algorithm_end = der_item(original, tbs_end)
    tbs = bytearray(original[outer_start:tbs_end])
    point = tbs.index(b'\x03\x42\x00\x04') + 4
    tbs[point:point + 64] = bytes(64)
    tbs_path = directory / 'invalid_point.tbs'
    tbs_path.write_bytes(tbs)
    signature = openssl('dgst', '-sha256', '-sign', paths['intermediate']['key'], tbs_path)
    invalid = der_encode(0x30, bytes(tbs) + original[tbs_end:algorithm_end] +
                         der_encode(0x03, b'\x00' + signature))
    invalid_der = directory / 'invalid_point.der'
    invalid_pem = directory / 'invalid_point.pem'
    invalid_der.write_bytes(invalid)
    write_certificate(invalid_pem, invalid)
    paths['invalid_point'] = {'pem': invalid_pem, 'der': invalid_der}

    damaged = bytearray(paths['server']['der'].read_bytes())
    damaged[-1] ^= 1
    damaged_der = directory / 'bad_signature.der'
    damaged_pem = directory / 'bad_signature.pem'
    damaged_der.write_bytes(damaged)
    write_certificate(damaged_pem, damaged)
    paths['bad_signature'] = {'pem': damaged_pem, 'der': damaged_der,
                              'key': paths['server']['key']}

    # SEC1 ECPrivateKey: version, then the 32-byte private scalar. The raw
    # scalar is used only by the host peer which signs CertificateVerify.
    sec1 = openssl('ec', '-in', paths['server']['key'], '-outform', 'DER')
    tag, start, end = der_item(sec1, 0)
    assert tag == 0x30 and end == len(sec1)
    tag, _, end = der_item(sec1, start)
    assert tag == 0x02
    tag, start, end = der_item(sec1, end)
    assert tag == 0x04 and end - start == 32
    scalar = directory / 'server.scalar'
    scalar.write_bytes(sec1[start:end])
    paths['server']['scalar'] = scalar

    for name, certificates in {
            'chain': ['intermediate'],
            'chain_root': ['intermediate', 'root'],
            'non_ca_chain': ['not_ca'],
            'no_ca_usage_chain': ['no_ca_usage'],
            'client_ca_chain': ['client_ca'],
            'short_ca_chain': ['short_ca'],
            'pathlen_chain': ['sub_intermediate', 'intermediate'],
            'constrained_chain': ['name_constrained']}.items():
        chain = directory / (name + '.pem')
        chain.write_bytes(b''.join(paths[c]['pem'].read_bytes() for c in certificates))
        paths[name] = chain
    return paths


def interop(binary, paths, now, *, chunk=65536, certificate='server',
            anchor='root', hostname='localhost', chain='chain', expected='ok',
            connections=1, buffers='internal'):
    with socket.socket() as probe:
        probe.bind(('127.0.0.1', 0))
        port = probe.getsockname()[1]
    command = ['openssl', 's_server', '-accept', f'127.0.0.1:{port}',
               '-cert', str(paths[certificate]['pem']),
               '-key', str(paths[certificate]['key']),
               '-tls1_3', '-groups', 'P-256',
               '-ciphersuites', 'TLS_AES_128_GCM_SHA256',
               '-rev', '-naccept', str(connections)]
    if chain is not None:
        command += ['-cert_chain', str(paths[chain])]
    print(f'OpenSSL: certificate={certificate} anchor={anchor} '
          f'host={hostname} chunk={chunk} expected={expected} '
          f'connections={connections} buffers={buffers}', flush=True)
    with tempfile.TemporaryFile() as errors:
        server = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=errors, bufsize=0)
        try:
            # Do not consume an accepted connection while waiting for startup.
            deadline = time.monotonic() + 5
            while True:
                remaining = deadline - time.monotonic()
                if remaining <= 0 or not select.select([server.stdout], [], [], remaining)[0]:
                    raise RuntimeError('OpenSSL did not start')
                line = server.stdout.readline()
                if b'ACCEPT' in line:
                    break
                if not line:
                    errors.seek(0)
                    raise RuntimeError(errors.read().decode())
            subprocess.run([str(binary), str(port), str(chunk),
                            str(paths[anchor]['der']), hostname, str(now),
                            expected, str(connections), buffers],
                           check=True, env=ENV, timeout=max(60, connections * 20))
        except Exception:
            errors.seek(0)
            print(errors.read().decode(errors='replace'), flush=True)
            raise
        finally:
            server.terminate()
            server.wait(timeout=5)


def main():
    with tempfile.TemporaryDirectory(prefix='iosonata-tls-cert-') as temp:
        directory = Path(temp)
        credentials = directory / 'credentials'
        paths = make_certificates(credentials)
        now = int(time.time()) + 60
        for optimization in ['-O0', '-O2']:
            ecc = directory / 'uecc.o'
            subprocess.run(['gcc', optimization, '-g', '-fsanitize=address,undefined',
                            '-fno-pie', '-DuECC_ENABLE_VLI_API=1', '-I' + str(UECC),
                            '-c', str(UECC / 'uECC.c'), '-o', str(ecc)], check=True)
            flags = ['-std=gnu++17', '-DuECC_ENABLE_VLI_API=1', optimization,
                     '-g', '-Wall', '-Wextra', '-Werror', '-fno-exceptions',
                     '-fno-rtti', '-ffunction-sections', '-fdata-sections',
                     '-fsanitize=address,undefined', '-fno-omit-frame-pointer',
                     '-fno-pie', '-no-pie', '-I' + str(ROOT / 'include'),
                     '-I' + str(UECC)]
            for name in ['tls_cert_test', 'tls_cert_host']:
                binary = directory / name
                subprocess.run(['g++', *flags, str(ROOT / f'tests/net/{name}.cpp'),
                                *[str(ROOT / p) for p in SOURCES], str(ecc),
                                '-Wl,--gc-sections', '-o', str(binary)], check=True)
                if name == 'tls_cert_test':
                    subprocess.run([str(binary), str(credentials), str(now)],
                                   check=True, env=ENV, timeout=60)
                    continue
                for chunk in [65536, 1, 7]:
                    interop(binary, paths, now, chunk=chunk, connections=2)
                interop(binary, paths, now, chain='chain_root', buffers='external')
                interop(binary, paths, now, certificate='direct', chain=None)
                interop(binary, paths, now, hostname='LOCALHOST')
                interop(binary, paths, now, certificate='noncritical_extension')
                interop(binary, paths, now, certificate='large_server',
                        buffers='external', chunk=7)
                interop(binary, paths, now, certificate='large_server', expected='limit')
                for options in [
                        {'anchor': 'other_root'},
                        {'hostname': 'wrong.test'},
                        {'certificate': 'wrong_name'},
                        {'certificate': 'no_san'},
                        {'certificate': 'bad_signature'},
                        {'certificate': 'wrong_eku'},
                        {'certificate': 'wrong_usage'},
                        {'certificate': 'critical_extension'},
                        {'certificate': 'non_ca_server', 'chain': 'non_ca_chain'}]:
                    interop(binary, paths, now, expected='certificate', **options)
                interop(binary, paths, now - 3600, expected='certificate')
                interop(binary, paths, now + 31 * 86400, expected='certificate')
                if optimization == '-O2':
                    interop(binary, paths, now, connections=20, buffers='external')
            print(f'{optimization}: certificate TLS/X.509 ASan/UBSan PASS', flush=True)
        subprocess.run(['gcc', '-std=c11', '-Werror', '-fsyntax-only',
                        '-I' + str(ROOT / 'include'), '-x', 'c', '-'],
                       input='#include "net/tls.h"\n#include "crypto/x509.h"\n'
                             'TlsDev_t dev; TlsCertCfg_t cfg; X509VerifyCfg_t verify;\n',
                       text=True, check=True)
    print('Certificate TLS C header and OpenSSL interoperability PASS', flush=True)


if __name__ == '__main__':
    main()
