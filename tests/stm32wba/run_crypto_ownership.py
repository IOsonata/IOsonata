#!/usr/bin/env python3
"""Host-check production WBA crypto ownership and scalar-only import."""
from pathlib import Path
import re
import subprocess
import tempfile
ROOT = Path(__file__).resolve().parents[2]
source = (ROOT / 'ARM/ST/STM32WBAxx/src/crypto_stm32wba.cpp').read_text()
header = (ROOT / 'include/crypto/crypto_stm32wba.h').read_text()
def function(name):
    match = re.search(r'^(?:bool|void|CRYPTO_STATUS) ' + re.escape(name) + r'\([^;]*?\)\s*\{', source, re.M)
    end, depth = match.end(), 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[match.start():end]
code = r'''
#include <atomic>
#include <cassert>
#include <cstdint>
#include <cstring>
struct Reg { unsigned CR, AHB2ENR, AHB2RSTR; } rcc, aes, pka;
#define RCC (&rcc)
#define AES (&aes)
#define PKA (&pka)
#define RCC_AHB2ENR_AESEN 1
#define RCC_AHB2ENR_PKAEN 2
#define RCC_AHB2RSTR_AESRST 1
#define SET_BIT(r,b) ((r) |= (b))
#define CLEAR_BIT(r,b) ((r) &= ~(b))
static unsigned starts, multiplies;
static bool startOk = true;
static void WbaAesWipeRegs() {}
static void WbaPkaWipeRam() {}
static bool WbaPkaStart() { starts++; RCC->AHB2ENR |= 2; return startOk; }
static void CryptoSecureWipe(void *p, size_t n) { memset(p, 0, n); }
static bool P256ScalarInRange(const uint8_t *p) { return p[31] != 0; }
static const uint8_t s_Gx[32] = {}, s_Gy[32] = {};
static bool WbaPkaScalarMul(const uint8_t *, const uint8_t *, const uint8_t *, uint8_t *, uint8_t *) { multiplies++; return true; }
enum CRYPTO_CURVE { CRYPTO_CURVE_P256 };
enum CRYPTO_STATUS { CRYPTO_STATUS_OK, CRYPTO_STATUS_BUSY, CRYPTO_STATUS_FAIL, CRYPTO_STATUS_UNSUPPORTED };
'''
for name in ['CryptoWbaAes', 'CryptoWbaPka']:
    start = header.index('class ' + name)
    body = header[start:header.index('\n};', start)]
    private = body[body.index('private:') + len('private:'):]
    code += 'struct ' + name + ' { ' + name + '() = default; bool vbValid = false;\n' + private
    code += 'bool Enable(); void Disable(); void Reset();\n'
    if name.endswith('Pka'):
        code += 'struct KeyCtx { uint8_t PrivKey[32]; bool bKeyValid; }; void KeyReset(void *); CRYPTO_STATUS KeyImport(CRYPTO_CURVE, void *, const uint8_t *, uint8_t *);\n'
    code += '};\n'
    code += 'std::atomic_flag ' + name + '::vOpBusy = ATOMIC_FLAG_INIT;\nunsigned ' + name + '::s_EnabledCount;\n'
    for method in ['Enable', 'Disable', 'Reset']:
        code += function(name + '::' + method) + '\n'
code += function('CryptoWbaPka::KeyReset') + '\n' + function('CryptoWbaPka::KeyImport')
code += r'''
static uint32_t s_PkaBuf[24];
static int s_PkaErr, s_PkaOp;
enum { BT_WBA_PKA_NONE, BT_WBA_PKA_KEYGEN, BT_WBA_PKA_DHKEY };
static unsigned completed;
static bool BtWbaPkaKeyGen() { return false; }
static bool BtWbaPkaDhKey() { return false; }
static void PKACTRL_CB_Complete() { completed++; }
'''
saved = source
source = (ROOT / 'ARM/ST/STM32WBAxx/src/bt_wba_pka.cpp').read_text().replace('static void BtWbaPkaTask', 'void BtWbaPkaTask')
code += function('BtWbaPkaTask')
source = saved
code += r'''
int main() {
 for (int op = BT_WBA_PKA_KEYGEN; op <= BT_WBA_PKA_DHKEY; op++) {
  memset(s_PkaBuf, 0xa5, sizeof(s_PkaBuf));
  s_PkaOp = op;
  BtWbaPkaTask();
  for (auto word : s_PkaBuf) assert(word == 0);
  assert(s_PkaErr == 1 && s_PkaOp == BT_WBA_PKA_NONE);
 }
 assert(completed == 2);
 CryptoWbaAes a, b;
 assert(a.Enable() && b.Enable());
 assert(a.OpAcquire());
 assert(!b.OpAcquire() && !b.Enable());
 b.Disable();
 assert(b.vbValid);
 a.OpRelease();
 a.Disable();
 assert(RCC->AHB2ENR & 1);
 b.Disable();
 assert(!(RCC->AHB2ENR & 1));
 b.Disable();
 assert(CryptoWbaAes::s_EnabledCount == 0);
 CryptoWbaPka p, q;
 assert(p.Enable() && q.Enable() && starts == 1);
 assert(p.OpAcquire() && !q.OpAcquire());
 p.OpRelease();
 p.Disable();
 assert(RCC->AHB2ENR & 2);
 uint8_t priv[32] = {}, pub[64] = {};
 priv[31] = 1;
 CryptoWbaPka::KeyCtx ctx;
 assert(q.KeyImport(CRYPTO_CURVE_P256, &ctx, priv, nullptr) == CRYPTO_STATUS_OK);
 assert(ctx.bKeyValid && ctx.PrivKey[31] == 1 && multiplies == 0);
 assert(q.KeyImport(CRYPTO_CURVE_P256, &ctx, priv, pub) == CRYPTO_STATUS_OK && multiplies == 1);
 q.Disable();
 assert(!(RCC->AHB2ENR & 2));
 startOk = false;
 assert(!q.Enable() && CryptoWbaPka::s_EnabledCount == 0);
}
'''
with tempfile.TemporaryDirectory() as temp:
    path = Path(temp)
    (path / 'test.cpp').write_text(code)
    for opt in ['-O0', '-O2']:
        subprocess.run(['g++', '-std=c++17', opt, '-fsanitize=undefined', '-fno-sanitize-recover=all', str(path / 'test.cpp'), '-o', str(path / 'test')], check=True)
        subprocess.run([str(path / 'test')], check=True)
        print('PASS WBA crypto:', opt, 'shared ownership, lifecycle, private-only import')
