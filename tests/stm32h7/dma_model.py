"""DMA1/2 streams and BDMA single-transfer registers; no timing claims."""
class Dma:
    def __init__(self, uc):
        self.uc = uc
        self.ch = [dict(cr=0, count=0, par=0, mem=0, cursor=0, req=0, flags=0, fcr=0) for _ in range(24)]
        self.moved = [0] * 24

    def decode(self, a):
        for base, first in ((0x40020000, 0), (0x40020400, 8)):
            if base <= a < base + 0xd0:
                off = a - base
                if off < 16: return ('status', first, off)
                ch, off = divmod(off - 16, 24)
                return ('stream', first + ch, off)
        if 0x58025400 <= a < 0x580254a8:
            off = a - 0x58025400
            if off < 8: return ('status', 16, off)
            ch, off = divmod(off - 8, 20)
            return ('bdma', 16 + ch, off)
        for base, first, size in ((0x40020800, 0, 64), (0x58025800, 16, 32)):
            if base <= a < base + size: return ('mux', first + (a-base)//4, 0)
        return None

    def read(self, a):
        kind, n, off = self.decode(a)
        c = self.ch[n]
        if kind == 'mux': return c['req']
        if kind == 'status':
            if n == 16:
                return sum(self.ch[n+i]['flags'] << (4*i) for i in range(8)) if off == 0 else 0
            if off >= 8: return 0
            n += 4 if off else 0
            return sum(self.ch[n+i]['flags'] << (0,6,16,22)[i] for i in range(4))
        return c[{0:'cr',4:'count',8:'par',12:'mem',16:'mem',20:'fcr'}[off]]

    def write(self, a, val):
        kind, n, off = self.decode(a)
        c = self.ch[n]
        if kind == 'mux': c['req'] = val; return
        if kind == 'status':
            if n == 16:
                if off == 4:
                    for i in range(8): self.ch[n+i]['flags'] &= ~(val >> (4*i) & 15)
            elif off >= 8:
                n += 4 if off == 12 else 0
                for i in range(4): self.ch[n+i]['flags'] &= ~(val >> (0,6,16,22)[i] & 0x3d)
            return
        key = {0:'cr',4:'count',8:'par',12:'mem',16:'mem',20:'fcr'}[off]
        if key in ('count', 'par', 'mem'): assert not c['cr'] & 1, ('DMA reprogrammed enabled', n, off)
        if key == 'cr' and val & 1 and not c['cr'] & 1:
            c['cursor'] = c['mem']
            assert c['count'] > 0
            assert 0x38000000 <= c['mem'] < 0x38010000, 'bus staging not in SRAM4'
        c[key] = val

    def find(self, req):
        return next((n for n,c in enumerate(self.ch) if c['cr'] & 1 and c['count'] and c['req'] == req), None)

    def move(self, req, width, value=None):
        n = self.find(req)
        if n is None: return None
        c = self.ch[n]
        routes = {33:0x40005424,34:0x40005428,35:0x40005824,36:0x40005828,
                  73:0x40005c24,74:0x40005c28,13:0x58001c24,14:0x58001c28,
                  37:0x40013030,38:0x40013020,39:0x40003830,40:0x40003820,
                  61:0x40003c30,62:0x40003c20,83:0x40013430,84:0x40013420,
                  85:0x40015030,86:0x40015020,11:0x58001430,12:0x58001420}
        assert c['par'] == routes[req], ('wrong peripheral address',n,req,c['par'])
        assert (n >= 16) == (req < 15), ('wrong DMA controller',n,req)
        w = (c['cr'] >> (8 if n >= 16 else 11)) & 3
        assert width == 1 << w, ('DMA width',n,width,w)
        tx = bool(c['cr'] & (16 if n >= 16 else 64))
        assert tx == (value is None)
        if value is None: value = int.from_bytes(self.uc.mem_read(c['cursor'], width), 'little')
        else: self.uc.mem_write(c['cursor'], int(value).to_bytes(width, 'little'))
        if c['cr'] & (128 if n >= 16 else 1024): c['cursor'] += width
        c['count'] -= 1
        self.moved[n] += width
        if not c['count']:
            c['cr'] &= ~1
            c['flags'] |= 2 if n >= 16 else 32
        return value

    def take(self, req, n=1): return self.move(req, n)
    def put(self, req, value, n=1): return self.move(req, n, value)

    def pending(self):
        for n,c in enumerate(self.ch):
            tc = (c['flags'] & (2 if n >= 16 else 32)) and (c['cr'] & (2 if n >= 16 else 16))
            err = (c['flags'] & (8 if n >= 16 else 13)) and (c['cr'] & (8 if n >= 16 else 6))
            if tc or err:
                return f'BDMA_Channel{n-16}_IRQHandler' if n >= 16 else f'DMA{n//8+1}_Stream{n%8}_IRQHandler'
