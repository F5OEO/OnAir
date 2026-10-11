// VDL Mode 2 bit layer, AVLC and the protocols above it: see vdl2_proto.h.
#include "dect2/vdl2_proto.h"
#include "dect2/acars_proto.h"
#include "dect2/aero_pos.h"
#include "dect2/vdl2_app.h"
#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstring>

namespace dect2 {

const int kVdl2PreamblePhase[kVdl2PreambleSyms] = {0, 3, -3, 1, 1, 2, 0, 4, -3, 4, -2, 3, 1, -2, -3, 0};
const uint8_t kVdl2GrayBits[8] = {0, 1, 3, 2, 6, 7, 5, 4};

int vdl2StepForBits(int b) {
    for (int i = 0; i < 8; i++) if (kVdl2GrayBits[i] == (b & 7)) return i;
    return 0;
}

// ---------------------------------------------------------------- scrambler, header

void vdl2Scramble(std::vector<uint8_t>& bits, size_t from) {
    uint16_t s = kVdl2LfsrSeed;
    for (size_t i = from; i < bits.size(); i++) {
        const int b = (s ^ (s >> 14)) & 1;
        s = (uint16_t)((s >> 1) | (b << 14));
        bits[i] ^= (uint8_t)b;
    }
}

namespace {
// parity rows of the header code over the 25-bit word (bit 24 = first sent); the low five bits are the parity bits
uint32_t hrow(int i) {
    static const char* rows[5] = {"0000000011111111111110000", "0011111100001111111101000", "1100011100110000111100100", "1101101101010011001100010", "0110100111100101010100001"};
    uint32_t v = 0;
    for (int k = 0; k < 25; k++) v = (v << 1) | (uint32_t)(rows[i][k] - '0');
    return v;
}
// error pattern for each syndrome (bit 4 of the syndrome = row 0)
uint32_t synd(int s) {
    static const char* t[32] = {
        "0000000000000000000000000", "0000000000000000000000001", "0000000000000000000000010", "0100000000000000000000100",
        "0000000000000000000000100", "0100000000000000000000010", "1000000000000000000000000", "0100000000000000000000000",
        "0000000000000000000001000", "0010000000000000000000000", "0001000000000000000000000", "0000100000000000000000000",
        "0000010000000000000000000", "1000100000000000000000000", "0000001000000000000000000", "0000000100000000000000000",
        "0000000000000000000010000", "0000000010000000000000000", "0100000000100000000000000", "0000000001000000000000000",
        "0100000001000000000000000", "0000000000100000000000000", "0000000000010000000000000", "1000000010000000000000000",
        "0000000000001000000000000", "0000000000000100000000000", "0000000000000010000000000", "0000000000000001000000000",
        "0000000000000000100000000", "0000000000000000010000000", "0000000000000000001000000", "0000000000000000000100000"};
    uint32_t v = 0;
    for (int k = 0; k < 25; k++) v = (v << 1) | (uint32_t)(t[s][k] - '0');
    return v;
}
uint32_t rev(uint32_t v, int n) { uint32_t r = 0; for (int i = 0; i < n; i++) { r = (r << 1) | (v & 1); v >>= 1; } return r; }
int par(uint32_t v) { return __builtin_popcount(v) & 1; }
int syndrome(uint32_t w) { int s = 0; for (int i = 0; i < 5; i++) s |= par(w & hrow(i)) << (4 - i); return s; }
} // namespace

uint32_t vdl2HeaderWord(uint32_t lenBits) {
    uint32_t w = rev(lenBits & 0x1FFFF, 17) << 5;
    for (int i = 0; i < 5; i++) w |= (uint32_t)par(w & hrow(i)) << (4 - i);
    return w;
}

bool vdl2HeaderDecode(uint32_t word, uint32_t& lenBits, bool& corrected) {
    word &= (1u << 22) - 1;                 // the reserved bits are not checked
    const int s = syndrome(word);
    corrected = s != 0;
    word ^= synd(s);
    if (word >> 22) return false;
    lenBits = rev((word >> 5) & 0x1FFFF, 17);
    if (lenBits == 0) return false;
    return lenBits <= (corrected ? 0x1FFFu : 0x3FFFu);
}

int vdl2LastFec(int n) { return n < 3 ? 0 : n <= 30 ? 2 : n <= 67 ? 4 : 6; }

bool vdl2Layout(uint32_t lenBits, Vdl2Layout& l) {
    l = {};
    l.dataOct = (int)((lenBits + 7) / 8);
    l.blocks = l.dataOct / kVdl2RsK;
    l.fecOct = l.blocks * (kVdl2RsN - kVdl2RsK);
    l.lastLen = l.dataOct % kVdl2RsK;
    l.lastFec = 6;
    if (l.lastLen) { l.blocks++; l.lastFec = vdl2LastFec(l.lastLen); l.fecOct += l.lastFec; }
    else l.lastLen = kVdl2RsK;
    return l.dataOct > 0 && l.fecOct > 0;
}

// ---------------------------------------------------------------- Reed-Solomon

namespace {
struct Gf {
    uint8_t exp[512], log[256];
    Gf() {
        int x = 1;
        for (int i = 0; i < 255; i++) { exp[i] = (uint8_t)x; log[x] = (uint8_t)i; x <<= 1; if (x & 0x100) x ^= 0x187; }
        for (int i = 255; i < 512; i++) exp[i] = exp[i - 255];
        log[0] = 0;
    }
    uint8_t mul(uint8_t a, uint8_t b) const { return (a && b) ? exp[log[a] + log[b]] : 0; }
    uint8_t div(uint8_t a, uint8_t b) const { return a ? exp[(log[a] + 255 - log[b]) % 255] : 0; }
    uint8_t pw(int e) const { e %= 255; if (e < 0) e += 255; return exp[e]; }
};
const Gf& gf() { static const Gf g; return g; }
constexpr int kFcr = 120, kRoots = 6;

// generator polynomial, highest power first: g[0] = 1
const uint8_t* genPoly() {
    static uint8_t g[kRoots + 1];
    static bool done = false;
    if (!done) {
        const Gf& G = gf();
        uint8_t p[kRoots + 1] = {1};
        int deg = 0;
        for (int i = 0; i < kRoots; i++) {      // p(x) *= (x + a^(fcr+i))
            const uint8_t r = G.pw(kFcr + i);
            uint8_t q[kRoots + 1] = {};
            for (int k = 0; k <= deg; k++) { q[k] ^= p[k]; q[k + 1] ^= G.mul(p[k], r); }
            deg++;
            memcpy(p, q, sizeof p);
        }
        memcpy(g, p, sizeof g);
        done = true;
    }
    return g;
}
} // namespace

void vdl2RsEncode(const uint8_t data[kVdl2RsK], uint8_t parity[6]) {
    const Gf& G = gf();
    const uint8_t* g = genPoly();
    uint8_t r[kRoots] = {};
    for (int i = 0; i < kVdl2RsK; i++) {
        const uint8_t fb = data[i] ^ r[0];
        for (int k = 0; k < kRoots - 1; k++) r[k] = r[k + 1] ^ G.mul(fb, g[k + 1]);
        r[kRoots - 1] = G.mul(fb, g[kRoots]);
    }
    memcpy(parity, r, kRoots);
}

int vdl2RsDecode(uint8_t cw[kVdl2RsN], int nParity) {
    const Gf& G = gf();
    const int N = kVdl2RsN;
    // syndromes: S_i = c(a^(fcr+i)), c[0] the highest power
    uint8_t S[kRoots];
    bool zero = true;
    for (int i = 0; i < kRoots; i++) {
        uint8_t s = 0;
        const uint8_t a = G.pw(kFcr + i);
        for (int j = 0; j < N; j++) s = G.mul(s, a) ^ cw[j];
        S[i] = s;
        zero &= s == 0;
    }
    const int nEr = kRoots - nParity;
    if (zero && nEr == 0) return 0;
    // erasure locator: positions kVdl2RsK + nParity .. N - 1, locator a^(N-1-j)
    uint8_t lam[kRoots + 1] = {1};
    for (int e = 0; e < nEr; e++) {
        const int j = kVdl2RsK + nParity + e;
        const uint8_t X = G.pw(N - 1 - j);
        for (int k = kRoots; k > 0; k--) lam[k] ^= G.mul(X, lam[k - 1]);
    }
    uint8_t B[kRoots + 1];
    memcpy(B, lam, sizeof B);
    int L = nEr;
    for (int r = nEr + 1; r <= kRoots; r++) {
        uint8_t d = 0;
        for (int i = 0; i < r; i++) d ^= G.mul(lam[i], S[r - 1 - i]);
        uint8_t xB[kRoots + 1] = {};
        for (int k = kRoots; k > 0; k--) xB[k] = B[k - 1];
        if (d == 0) { memcpy(B, xB, sizeof B); continue; }
        uint8_t T[kRoots + 1];
        for (int k = 0; k <= kRoots; k++) T[k] = lam[k] ^ G.mul(d, xB[k]);
        if (2 * L <= r + nEr - 1) {
            L = r + nEr - L;
            for (int k = 0; k <= kRoots; k++) B[k] = G.div(lam[k], d);
        } else memcpy(B, xB, sizeof B);
        memcpy(lam, T, sizeof lam);
    }
    int deg = 0;
    for (int k = 0; k <= kRoots; k++) if (lam[k]) deg = k;
    if (deg == 0) return zero ? 0 : -1;
    // Chien search over the 255 positions
    int pos[kRoots], nr = 0;
    for (int j = 0; j < N; j++) {
        const int e = N - 1 - j;                 // locator exponent of position j
        uint8_t v = 0;
        for (int k = 0; k <= deg; k++) v ^= G.mul(lam[k], G.pw(-e * k));
        if (v == 0) { if (nr == kRoots) return -1; pos[nr++] = j; }
    }
    if (nr != deg) return -1;
    // evaluator omega = S(x) lambda(x) mod x^roots
    uint8_t om[kRoots] = {};
    for (int i = 0; i < kRoots; i++) for (int k = 0; k <= i && k <= deg; k++) om[i] ^= G.mul(lam[k], S[i - k]);
    int fixed = 0;
    for (int q = 0; q < nr; q++) {
        const int j = pos[q], e = N - 1 - j;
        const uint8_t xinv = G.pw(-e);
        uint8_t o = 0, xp = 1;
        for (int i = 0; i < kRoots; i++) { o ^= G.mul(om[i], xp); xp = G.mul(xp, xinv); }
        uint8_t dl = 0;
        for (int k = 1; k <= deg; k += 2) dl ^= G.mul(lam[k], G.pw(-e * (k - 1)));
        if (!dl) return -1;
        const uint8_t val = G.mul(G.div(o, dl), G.pw(e * (1 - kFcr)));
        if (val && j < kVdl2RsK + nParity) fixed++;
        cw[j] ^= val;
    }
    if (2 * fixed + nEr > kRoots) return -1;
    return fixed;
}

// ---------------------------------------------------------------- burst body

namespace {
void putOctet(std::vector<uint8_t>& bits, uint8_t v) { for (int b = 0; b < 8; b++) bits.push_back((v >> b) & 1); }
uint8_t getOctet(const uint8_t* bits) { uint8_t v = 0; for (int b = 0; b < 8; b++) v |= (uint8_t)((bits[b] & 1) << b); return v; }
} // namespace

std::vector<uint8_t> vdl2BurstBits(const std::vector<uint8_t>& dataBits) {
    const uint32_t len = (uint32_t)dataBits.size();
    Vdl2Layout l;
    std::vector<uint8_t> out;
    if (!vdl2Layout(len, l)) return out;
    std::vector<uint8_t> data(l.dataOct, 0);
    for (size_t i = 0; i < dataBits.size(); i++) data[i / 8] |= (uint8_t)((dataBits[i] & 1) << (i % 8));
    // blocks (rows), data row by row
    std::vector<std::vector<uint8_t>> rows(l.blocks, std::vector<uint8_t>(kVdl2RsN, 0));
    for (int i = 0; i < l.dataOct; i++) rows[i / kVdl2RsK][i % kVdl2RsK] = data[i];
    for (int r = 0; r < l.blocks; r++) vdl2RsEncode(rows[r].data(), &rows[r][kVdl2RsK]);
    const uint32_t hw = vdl2HeaderWord(len);
    for (int b = 24; b >= 0; b--) out.push_back((hw >> b) & 1);
    for (int c = 0; c < kVdl2RsK; c++)
        for (int r = 0; r < l.blocks; r++) {
            if (r == l.blocks - 1 && c >= l.lastLen) continue;
            putOctet(out, rows[r][c]);
        }
    const int fecRows = l.lastFec ? l.blocks : l.blocks - 1;
    for (int c = 0; c < kVdl2RsN - kVdl2RsK; c++)
        for (int r = 0; r < fecRows; r++) {
            if (r == l.blocks - 1 && c >= l.lastFec) continue;
            putOctet(out, rows[r][kVdl2RsK + c]);
        }
    vdl2Scramble(out, 0);
    return out;
}

bool vdl2BurstData(const uint8_t* bits, const Vdl2Layout& l, uint32_t lenBits, std::vector<uint8_t>& dataBits, int& rsFixed) {
    rsFixed = 0;
    std::vector<std::vector<uint8_t>> rows(l.blocks, std::vector<uint8_t>(kVdl2RsN, 0));
    size_t k = 0;
    for (int c = 0; c < kVdl2RsK; c++)
        for (int r = 0; r < l.blocks; r++) {
            if (r == l.blocks - 1 && c >= l.lastLen) continue;
            rows[r][c] = getOctet(bits + 8 * k++);
        }
    const int fecRows = l.lastFec ? l.blocks : l.blocks - 1;
    for (int c = 0; c < kVdl2RsN - kVdl2RsK; c++)
        for (int r = 0; r < fecRows; r++) {
            if (r == l.blocks - 1 && c >= l.lastFec) continue;
            rows[r][kVdl2RsK + c] = getOctet(bits + 8 * k++);
        }
    for (int r = 0; r < l.blocks; r++) {
        const int np = r == l.blocks - 1 ? l.lastFec : 6;
        if (!np) continue;
        const int f = vdl2RsDecode(rows[r].data(), np);
        if (f < 0) return false;
        rsFixed += f;
    }
    dataBits.clear();
    dataBits.reserve(lenBits);
    for (int r = 0; r < l.blocks; r++) {
        const int n = r == l.blocks - 1 ? l.lastLen : kVdl2RsK;
        for (int c = 0; c < n; c++) for (int b = 0; b < 8 && dataBits.size() < lenBits; b++) dataBits.push_back((rows[r][c] >> b) & 1);
    }
    return true;
}

// ---------------------------------------------------------------- HDLC

namespace {
uint16_t crcReg(const uint8_t* p, size_t n) {
    uint16_t c = 0xFFFF;
    for (size_t i = 0; i < n; i++) { c ^= p[i]; for (int b = 0; b < 8; b++) c = (c & 1) ? (uint16_t)((c >> 1) ^ 0x8408) : (uint16_t)(c >> 1); }
    return c;
}
} // namespace

uint16_t vdl2Fcs(const uint8_t* p, size_t n) { return (uint16_t)~crcReg(p, n); }
bool vdl2FcsOk(const uint8_t* p, size_t n) { return n >= 2 && crcReg(p, n) == 0xF0B8; }

std::vector<uint8_t> vdl2HdlcBits(const std::vector<std::vector<uint8_t>>& frames) {
    std::vector<uint8_t> out;
    auto flag = [&] { putOctet(out, 0x7E); };
    flag();
    for (const auto& f : frames) {
        int ones = 0;
        for (uint8_t v : f)
            for (int b = 0; b < 8; b++) {
                const int bit = (v >> b) & 1;
                out.push_back((uint8_t)bit);
                if (bit) { if (++ones == 5) { out.push_back(0); ones = 0; } } else ones = 0;
            }
        flag();
    }
    return out;
}

std::vector<std::vector<uint8_t>> vdl2HdlcFrames(const std::vector<uint8_t>& bits) {
    std::vector<std::vector<uint8_t>> out;
    std::vector<uint8_t> cur;
    int ones = 0;
    bool sync = false, abort = false;
    auto finish = [&] {
        if (sync && !abort && cur.size() >= 16 && cur.size() % 8 == 0) {
            std::vector<uint8_t> f(cur.size() / 8);
            for (size_t i = 0; i < f.size(); i++) f[i] = getOctet(&cur[8 * i]);
            out.push_back(std::move(f));
        }
        cur.clear(); abort = false;
    };
    for (uint8_t b : bits) {
        if (b) {
            if (++ones > 6) { abort = true; cur.clear(); }
            else cur.push_back(1);
            continue;
        }
        if (ones == 5) { ones = 0; continue; }       // inserted zero
        if (ones == 6) {                             // flag: drop its leading zero and six ones
            cur.resize(cur.size() >= 7 ? cur.size() - 7 : 0);
            finish();
            sync = true; ones = 0;
            continue;
        }
        cur.push_back(0); ones = 0;
    }
    return out;
}

// ---------------------------------------------------------------- AVLC

namespace {
uint32_t rev28(uint32_t v) { return rev(v, 28); }
} // namespace

void vdl2PutAddr(uint8_t* o, const Vdl2Addr& a, bool last) {
    const uint32_t V = ((uint32_t)a.status << 27) | ((uint32_t)(a.type & 7) << 24) | (a.addr & 0xFFFFFF);
    const uint32_t t = rev28(V);
    for (int i = 0; i < 4; i++) o[i] = (uint8_t)(((t >> (7 * i)) & 0x7F) << 1);
    if (last) o[3] |= 1;
}

Vdl2Addr vdl2GetAddr(const uint8_t* o) {
    uint32_t t = 0;
    for (int i = 0; i < 4; i++) t |= (uint32_t)((o[i] >> 1) & 0x7F) << (7 * i);
    const uint32_t V = rev28(t);
    Vdl2Addr a;
    a.addr = V & 0xFFFFFF; a.type = (V >> 24) & 7; a.status = (V >> 27) & 1;
    return a;
}

const char* vdl2AddrTypeName(int t) {
    switch (t) { case 1: return "aircraft"; case 4: return "ground station (ADM)"; case 5: return "ground station (DEL)"; case 7: return "all stations"; default: return "reserved"; }
}

uint8_t vdl2ControlI(int ns, int nr, bool p) { return (uint8_t)(((ns & 7) << 1) | (p ? 0x10 : 0) | ((nr & 7) << 5)); }

std::vector<uint8_t> vdl2Frame(const Vdl2Addr& dst, const Vdl2Addr& src, uint8_t control, const std::vector<uint8_t>& info) {
    std::vector<uint8_t> f(9);
    vdl2PutAddr(&f[0], dst, false);
    vdl2PutAddr(&f[4], src, true);
    f[8] = control;
    f.insert(f.end(), info.begin(), info.end());
    const uint16_t c = vdl2Fcs(f.data(), f.size());
    f.push_back((uint8_t)(c & 0xFF)); f.push_back((uint8_t)(c >> 8));
    return f;
}

// ---- protocol readers ----
namespace {

std::string hexs(const uint8_t* p, size_t n, size_t max = 24) {
    std::string s;
    char b[4];
    for (size_t i = 0; i < n && i < max; i++) { snprintf(b, sizeof b, "%02X", p[i]); s += b; }
    if (n > max) s += "...";
    return s;
}

std::string bcd(const uint8_t* p, int digits, int startNibble) {
    std::string s;
    for (int i = 0; i < digits; i++) {
        const int nib = startNibble + i;
        const uint8_t v = (p[nib / 2] >> (nib % 2 ? 0 : 4)) & 0xF;
        s += (char)(v < 10 ? '0' + v : '?');
    }
    return s;
}

void addLine(std::string& d, const char* fmt, ...) __attribute__((format(printf, 2, 3)));
void addLine(std::string& d, const char* fmt, ...) {
    char b[400];
    va_list ap; va_start(ap, fmt); vsnprintf(b, sizeof b, fmt, ap); va_end(ap);
    d += b; d += '\n';
}

double signed12(int v) { return (v & 0x800) ? v - 0x1000 : v; }

std::string nsap(const uint8_t* p, int n) { return hexs(p, (size_t)n, 20); }

struct Dict { int id; const char* s; };
const char* dict(const Dict* d, int id) { for (; d->s; d++) if (d->id == id) return d->s; return nullptr; }

uint32_t be16(const uint8_t* p) { return ((uint32_t)p[0] << 8) | p[1]; }
uint32_t be32(const uint8_t* p) { return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3]; }
uint32_t beN(const uint8_t* p, size_t n) { uint32_t v = 0; for (size_t i = 0; i < n && i < 4; i++) v = (v << 8) | p[i]; return v; }

std::string asciiOf(const uint8_t* p, size_t n) {
    std::string s;
    for (size_t i = 0; i < n; i++) s += (p[i] >= 32 && p[i] < 127) ? (char)p[i] : '.';
    return s;
}
std::string hexAscii(const uint8_t* p, size_t n, size_t max = 24) {
    bool text = n > 1;
    for (size_t i = 0; i < n; i++) if (p[i] < 32 || p[i] > 126) text = false;
    return hexs(p, n, max) + (text ? " \"" + asciiOf(p, n) + "\"" : "");
}

// What the layers above AVLC need to know while a frame is read.
struct Ctx { Vdl2Reasm* rs = nullptr; double now = 0; uint32_t src = 0, dst = 0; };
uint64_t pairKey(const Ctx& c) { return ((uint64_t)c.src << 24) | c.dst; }

void userData(const uint8_t* p, size_t n, Vdl2Message& m, const Ctx& c);
void clnpPayload(const uint8_t* p, size_t n, Vdl2Message& m, const Ctx& c);

// One more piece of a numbered sequence (X.25 packets, COTP data TPDUs). True when `last` ended it: `out` has all the octets.
bool seqAdd(Vdl2Reasm::Seq& s, uint32_t seq, uint32_t mask, const uint8_t* p, size_t n, bool last, double now, std::vector<uint8_t>& out, int& pieces) {
    if (s.pieces && (seq != s.next || std::fabs(now - s.last) > 60)) { s.data.clear(); s.pieces = 0; }
    s.data.insert(s.data.end(), p, p + n);
    s.pieces++;
    s.next = (seq + 1) & mask;
    s.last = now;
    if (!last) return false;
    out = std::move(s.data);
    pieces = s.pieces;
    return true;
}

// ---- ES-IS (ISO 9542), the hello of an end system or intermediate system
void readEsis(const uint8_t* p, size_t n, Vdl2Message& m) {
    m.proto = "ES-IS";
    if (n < 10 || p[2] != 1 || p[1] > n) { m.summary = "ES-IS"; addLine(m.detail, "ES-IS (ISO 9542): %zu octets, header not valid: %s", n, hexs(p, n).c_str()); return; }
    const int type = p[4] & 0x1F;
    const char* tn = type == 2 ? "ES hello" : type == 4 ? "IS hello" : type == 6 ? "redirect" : "unknown PDU";
    m.summary = std::string("ES-IS ") + tn;
    addLine(m.detail, "ES-IS (ISO 9542): %s, hold time %u s", tn, be16(p + 5));
    size_t o = 9;
    const size_t al = o < n ? p[o++] : 0;
    if (o + al > n) { addLine(m.detail, "  address cut short"); return; }
    addLine(m.detail, "  %s %s", type == 2 ? "source address (SA)" : "network entity title (NET)", hexAscii(p + o, al, 24).c_str());
    o += al;
    while (o + 2 <= n) {
        const uint8_t code = p[o], len = p[o + 1];
        const uint8_t* v = p + o + 2;
        if (o + 2 + len > n) break;
        switch (code) {
        case 0x81: {
            std::string s;
            if (len >= 1) {
                if ((v[0] & 0x1F) == 0x1F) s = "all traffic types"; else {
                    static const char* tt[5] = {"ATS", "AOC", "ATN administrative", "general communications", "ATN system management"};
                    for (int i = 0; i < 5; i++) if (v[0] & (1 << i)) s += (s.empty() ? "" : ", ") + std::string(tt[i]);
                    if (s.empty()) s = "none";
                }
                if ((v[0] & 1) && len > 1) {
                    s += "; ATSC classes ";
                    if (v[1] == 0xFF) s += "all"; else for (int i = 0; i < 8; i++) if (v[1] & (1 << i)) s += (char)('A' + i);
                }
            }
            addLine(m.detail, "  mobile subnetwork capabilities: permitted traffic %s", s.c_str());
            break;
        }
        case 0x88: addLine(m.detail, "  ATN data link capabilities: %s", hexs(v, len).c_str()); break;
        case 0xC5: addLine(m.detail, "  security: %s", hexs(v, len).c_str()); break;
        case 0xCF: addLine(m.detail, "  priority: %s", hexs(v, len).c_str()); break;
        default: addLine(m.detail, "  option %02X: %s", code, hexs(v, len).c_str()); break;
        }
        o += 2u + len;
    }
}

// ---- IDRP (ISO 10747), BISPDU header and the simple PDUs
void readIdrp(const uint8_t* p, size_t n, Vdl2Message& m) {
    m.proto = "IDRP";
    if (n < 30) { m.summary = "IDRP"; addLine(m.detail, "IDRP (ISO 10747): %zu octets, header cut short", n); return; }
    static const Dict kTypes[] = {{1, "OPEN"}, {2, "UPDATE"}, {3, "ERROR"}, {4, "KEEPALIVE"}, {5, "CEASE"}, {6, "RIB refresh"}, {0, nullptr}};
    const int type = p[3];
    const char* tn = dict(kTypes, type);
    m.summary = std::string("IDRP ") + (tn ? tn : "unknown");
    addLine(m.detail, "IDRP (ISO 10747) BISPDU: %s, length %u, seq %u, ack %u, credit offered %u, available %u", tn ? tn : "unknown type", be16(p + 1), be32(p + 4), be32(p + 8), p[12], p[13]);
    const uint8_t* b = p + 30;
    size_t bn = std::min<size_t>(n, be16(p + 1) >= 30 ? be16(p + 1) : n) - 30;
    if (type == 1 && bn >= 6) {
        addLine(m.detail, "  version %d, hold time %u s, max PDU %u octets, source RDI %s", b[0], be16(b + 1), be16(b + 3), bn > 5 && 6u + b[5] <= bn ? hexs(b + 6, b[5]).c_str() : "-");
    } else if (type == 3 && bn >= 2) {
        static const Dict kOpen[] = {{1, "unsupported version"}, {2, "bad max PDU size"}, {3, "bad peer RD"}, {4, "unsupported auth code"}, {5, "auth failure"}, {6, "bad RIB-AttsSet"}, {7, "RDC mismatch"}, {0, nullptr}};
        static const Dict kUpd[] = {{1, "malformed attribute list"}, {2, "unrecognized well-known attribute"}, {3, "missing well-known attribute"}, {4, "attribute flags error"},
            {5, "attribute length error"}, {6, "RD routing loop"}, {7, "invalid NEXT_HOP attribute"}, {8, "optional attribute error"}, {9, "invalid reachability information"},
            {10, "misconfigured RDCs"}, {11, "malformed NLRI"}, {12, "duplicated attributes"}, {13, "illegal RD path segment"}, {0, nullptr}};
        const int code = b[0], sub = b[1];
        const char* cn = code == 1 ? "OPEN PDU error" : code == 2 ? "UPDATE PDU error" : code == 3 ? "hold timer expired" : code == 4 ? "FSM error" : code == 5 ? "RIB refresh PDU error" : "unknown error";
        const char* sn = code == 1 ? dict(kOpen, sub) : code == 2 ? dict(kUpd, sub) : nullptr;
        addLine(m.detail, "  %s (%d), subcode %d%s%s%s", cn, code, sub, sn ? " (" : "", sn ? sn : "", sn ? ")" : "");
        m.summary += std::string(": ") + cn;
    } else if (bn) {
        addLine(m.detail, "  %zu octets of %s data (path attributes and reachability are not decoded)", bn, tn ? tn : "BISPDU");
    }
}

// ---- COTP (ISO 8073 / X.224 class 4) TPDUs
const char* cotpName(uint8_t c) {
    switch (c & 0xF0) {
    case 0xE0: return "CR (connection request)"; case 0xD0: return "CC (connection confirm)"; case 0xF0: return "DT (data)";
    case 0x80: return "DR (disconnect request)"; case 0xC0: return "DC (disconnect confirm)"; case 0x60: return "AK (acknowledgement)";
    case 0x10: return "ED (expedited data)"; case 0x20: return "EA (expedited ack)"; case 0x70: return "ER (error)"; case 0x50: return "RJ (reject)";
    default: return "unknown TPDU";
    }
}

std::string cotpParam(uint8_t code, const uint8_t* v, size_t len, bool er) {
    char b[160];
    if (er) {
        if (code == 0xC1) return "invalid TPDU header " + hexs(v, len);
        if (code == 0xC3) return "checksum " + hexs(v, len);
        snprintf(b, sizeof b, "parameter %02X %s", code, hexs(v, len).c_str());
        return b;
    }
    switch (code) {
    case 0x08: return "ATN checksum " + hexs(v, len);
    case 0x85: snprintf(b, sizeof b, "ack time %u ms", beN(v, len)); return b;
    case 0x86: return "residual error rate " + hexs(v, len);
    case 0x87: snprintf(b, sizeof b, "priority %u", beN(v, len)); return b;
    case 0x88: return "transit delay " + hexs(v, len);
    case 0x89: return "throughput " + hexs(v, len);
    case 0x8A: snprintf(b, sizeof b, "subsequence number %u", beN(v, len)); return b;
    case 0x8B: snprintf(b, sizeof b, "reassignment time %u s", beN(v, len)); return b;
    case 0x8C:
        if (len == 8) { snprintf(b, sizeof b, "flow control: acked TPDU %u, subsequence %u, credit %u", be32(v) & 0x7FFFFFFFu, be16(v + 4), be16(v + 6)); return b; }
        return "flow control " + hexs(v, len);
    case 0x8F: return "selective ack " + hexs(v, len);
    case 0xC0:
        if (len == 1 && v[0] >= 7 && v[0] <= 13) { snprintf(b, sizeof b, "TPDU size %u octets", 1u << v[0]); return b; }
        return "TPDU size " + hexs(v, len);
    case 0xC1: return "calling transport selector " + hexAscii(v, len);
    case 0xC2: return "called / responding transport selector " + hexAscii(v, len);
    case 0xC3: return "checksum " + hexs(v, len);
    case 0xC4: snprintf(b, sizeof b, "version %u", beN(v, len)); return b;
    case 0xC5: return "protection " + hexs(v, len);
    case 0xC6: return "additional options " + hexs(v, len);
    case 0xC7: return "additional protocol classes " + hexs(v, len);
    case 0xE0: return "additional info " + hexs(v, len);
    case 0xF0: return "preferred max TPDU size " + hexs(v, len);
    case 0xF2: snprintf(b, sizeof b, "inactivity timer %u ms", beN(v, len)); return b;
    default: snprintf(b, sizeof b, "parameter %02X %s", code, hexs(v, len).c_str()); return b;
    }
}

// What a transport user sends: the ULCS (session, presentation, ACSE) wrapping ASN.1 PER for CPDLC or context management (vdl2_app.h).
// The hex of the octets stays under whatever was read; ADS-C and the rest of the ICAO messages are only named by their headers.
void atnApplication(const uint8_t* p, size_t n, Vdl2Message& m) {
    const Vdl2AppResult r = vdl2AtnApp(p, n, m.fromAir);
    addLine(m.detail, "  ATN application data, %zu octets:", n);
    m.detail += r.detail;
    if (!r.decoded) addLine(m.detail, "    ASN.1 PER body not decoded");
    addLine(m.detail, "    raw %s", hexs(p, n, 64).c_str());
    m.summary += ", ATN application data";
    if (!r.summary.empty()) m.summary += " (" + r.summary + ")";
}

void readCotp(const uint8_t* p, size_t n, Vdl2Message& m, const Ctx& c) {
    m.proto = "COTP";
    bool firstTpdu = true;
    while (n) {
        const size_t li = p[0];
        if (n < 4 || li == 0 || li == 255 || li + 1 > n) { addLine(m.detail, "COTP (ISO 8073): %zu octets that do not form a TPDU: %s", n, hexs(p, n).c_str()); return; }
        const uint8_t* q = p + 1;                        // the TPDU code .. the end of the header
        const uint8_t top = q[0] & 0xF0;
        std::string head;
        char b[200];
        size_t var = 0;                                  // offset of the variable part in q
        bool last = false, dataTpdu = false, eot = false, ext = false;
        uint32_t seq = 0, ref = (uint32_t)be16(q + 1);
        bool ok = true;
        switch (top) {
        case 0xE0: case 0xD0: case 0x80:
            ok = li >= 6;
            if (ok) {
                if (top == 0x80) snprintf(b, sizeof b, "dst ref %04X, src ref %04X, reason %d", ref, be16(q + 3), q[5]);
                else snprintf(b, sizeof b, "dst ref %04X, src ref %04X, class %d, options %X, credit %d", ref, be16(q + 3), q[5] >> 4, q[5] & 0xF, q[0] & 0xF);
                head = b; var = 6; last = true;
            }
            break;
        case 0xF0: case 0x10:
            ext = li & 1;
            ok = li >= (ext ? 7u : 4u);
            if (ok) {
                eot = q[3] & 0x80;
                seq = ext ? be32(q + 3) & 0x7FFFFFFFu : (q[3] & 0x7Fu);
                snprintf(b, sizeof b, "dst ref %04X, TPDU %u%s%s", ref, seq, eot ? ", last of the message" : ", more follows", ext ? ", extended" : "");
                head = b; var = ext ? 7 : 4; last = true; dataTpdu = true;
            }
            break;
        case 0xC0:
            ok = li >= 5;
            if (ok) { snprintf(b, sizeof b, "dst ref %04X, src ref %04X", ref, be16(q + 3)); head = b; var = 5; }
            break;
        case 0x60: case 0x20: case 0x50:
            ext = li & 1;
            ok = li >= (ext ? 7u : 4u);
            if (ok) {
                seq = ext ? be32(q + 3) & 0x7FFFFFFFu : (q[3] & 0x7Fu);
                if (top == 0x60 && ext && li >= 9) { snprintf(b, sizeof b, "dst ref %04X, next TPDU %u, credit %u", ref, seq, be16(q + 7)); var = 9; }
                else if (top == 0x50 && ext && li >= 9) { snprintf(b, sizeof b, "dst ref %04X, next TPDU %u, credit %u", ref, seq, be16(q + 7)); var = 0; }
                else { snprintf(b, sizeof b, "dst ref %04X, TPDU %u", ref, seq); var = top == 0x50 ? 0 : (ext ? 7 : 4); }
                head = b;
            }
            break;
        case 0x70:
            ok = li >= 4;
            if (ok) {
                static const char* er[] = {"not specified", "invalid parameter code", "invalid TPDU type", "invalid parameter value"};
                snprintf(b, sizeof b, "dst ref %04X, cause %d (%s)", ref, q[3], q[3] < 4 ? er[q[3]] : "unknown");
                head = b; var = 4;
            }
            break;
        default: ok = false;
        }
        if (!ok) { addLine(m.detail, "COTP (ISO 8073): not a TPDU I can read: %s", hexs(p, n).c_str()); return; }
        addLine(m.detail, "COTP (ISO 8073): %s, %s", cotpName(q[0]), head.c_str());
        if (firstTpdu) m.summary += std::string(", COTP ") + std::string(cotpName(q[0])).substr(0, 2);
        else m.summary += std::string(" + ") + std::string(cotpName(q[0])).substr(0, 2);
        firstTpdu = false;
        for (size_t o = var; var && o + 2 <= li + 0u; ) {
            const uint8_t code = q[o], len = q[o + 1];
            if (o + 2 + len > li) break;
            addLine(m.detail, "  %s", cotpParam(code, q + o + 2, len, top == 0x70).c_str());
            o += 2u + len;
        }
        if (!last) { p += li + 1; n -= li + 1; continue; }
        const uint8_t* u = p + li + 1;
        size_t un = n - li - 1;
        if (!un) return;
        if (top == 0x80 && un == 1) { static const char* sp[] = {"protocol error", "normal, no reuse", "normal, reuse not possible"}; addLine(m.detail, "  session disconnect reason: %s", u[0] < 3 ? sp[u[0]] : "unknown"); return; }
        if (!dataTpdu) { atnApplication(u, un, m); return; }
        if (!c.rs) {
            if (!eot) { addLine(m.detail, "  %zu octets of a message that continues in the next TPDU", un); return; }
            atnApplication(u, un, m);
            return;
        }
        std::vector<uint8_t> whole;
        int pieces = 0;
        const uint64_t key = pairKey(c) | ((uint64_t)ref << 48);
        if (seqAdd(c.rs->cotp[key], seq, ext ? 0x7FFFFFFFu : 0x7Fu, u, un, eot, c.now, whole, pieces)) {
            c.rs->cotp.erase(key);
            if (pieces > 1) addLine(m.detail, "  message joined from %d TPDUs, %zu octets", pieces, whole.size());
            atnApplication(whole.data(), whole.size(), m);
        } else {
            addLine(m.detail, "  piece %d of a message, %zu octets so far", c.rs->cotp[key].pieces, c.rs->cotp[key].data.size());
            m.summary += ", piece of a message";
        }
        return;
    }
}

// ---- CLNP (ISO 8473) and its ICAO compressed form
void joinSegments(const uint8_t* data, size_t n, uint32_t pduId, uint32_t offset, bool more, uint32_t total, Vdl2Message& m, const Ctx& c) {
    // whole PDU in one piece, or no memory to join with
    if ((offset == 0 && !more) || !c.rs) {
        if (offset == 0 && !more) { clnpPayload(data, n, m, c); return; }
        addLine(m.detail, "  segment at offset %u of %u, %zu octets (not joined)", offset, total, n);
        m.summary += ", segment";
        return;
    }
    const uint64_t key = pairKey(c) | ((uint64_t)pduId << 48);
    Vdl2Reasm::Segs& g = c.rs->clnp[key];
    g.last = c.now;
    g.total = total;
    g.part[offset].assign(data, data + n);
    if (!more) { g.final = true; g.finalEnd = offset + (uint32_t)n; }
    bool complete = false;
    if (g.final) {
        uint32_t reach = 0;
        for (const auto& kv : g.part) { if (kv.first > reach) break; reach = std::max(reach, kv.first + (uint32_t)kv.second.size()); }
        complete = reach >= g.finalEnd;
    }
    if (!complete) {
        addLine(m.detail, "  segment at offset %u of %u, %zu octets; %zu pieces held, waiting for the rest", offset, total, n, g.part.size());
        m.summary += ", segment";
        return;
    }
    std::vector<uint8_t> whole(g.finalEnd);
    const size_t pieces = g.part.size();
    for (const auto& kv : g.part) for (size_t i = 0; i < kv.second.size() && kv.first + i < whole.size(); i++) whole[kv.first + i] = kv.second[i];
    c.rs->clnp.erase(key);
    addLine(m.detail, "  PDU joined from %zu segments, %zu octets", pieces, whole.size());
    clnpPayload(whole.data(), whole.size(), m, c);
}

void clnpPayload(const uint8_t* p, size_t n, Vdl2Message& m, const Ctx& c) {
    if (!n) return;
    if (p[0] == 0x82) readEsis(p, n, m);
    else if (p[0] == 0x85) readIdrp(p, n, m);
    else if (p[0] == 0x81) addLine(m.detail, "CLNP inside CLNP: %s", hexs(p, n).c_str());
    else readCotp(p, n, m, c);
}

void readClnp(const uint8_t* p, size_t n, Vdl2Message& m, const Ctx& c, int depth = 0) {
    m.proto = "CLNP";
    const size_t li = n > 1 ? p[1] : 0;
    if (n < 9 || li == 255 || li < 9 || li > n || p[2] != 1) {
        addLine(m.detail, "CLNP: %zu octets, header not valid: %s", n, hexs(p, n).c_str());
        m.summary = "CLNP (not valid)";
        return;
    }
    const int ver = p[2], life = p[3], type = p[4] & 0x1F;
    const bool sp = p[4] & 0x80, ms = p[4] & 0x40;
    const char* tn = type == 0x1C ? "DT (data)" : type == 0x01 ? "ER (error report)" : type == 0x1E ? "ERQ (echo request)" : type == 0x1F ? "ERP (echo reply)" : "unknown";
    addLine(m.detail, "CLNP: %s, version %d, lifetime %.1f s, segment %u octets, header %zu octets, checksum %04X%s%s", tn, ver, life * 0.5, be16(p + 5), li, be16(p + 7),
            sp ? ", segmentation permitted" : "", (p[4] & 0x20) ? ", error reports wanted" : "");
    size_t o = 9;
    std::string dst, src;
    if (o < li) { const size_t dl = p[o++]; if (o + dl <= li) { dst = nsap(p + o, (int)dl); o += dl; } }
    if (o < li) { const size_t sl = p[o++]; if (o + sl <= li) { src = nsap(p + o, (int)sl); o += sl; } }
    addLine(m.detail, "  destination NSAP %s", dst.empty() ? "-" : dst.c_str());
    addLine(m.detail, "  source NSAP      %s", src.empty() ? "-" : src.c_str());
    uint32_t pduId = 0, offset = 0, total = 0;
    if (sp && o + 6 <= li) { pduId = be16(p + o); offset = be16(p + o + 2); total = be16(p + o + 4); o += 6; }
    if (sp) addLine(m.detail, "  segmentation: PDU id %04X, offset %u, total length %u%s", pduId, offset, total, ms ? ", more segments" : "");
    while (o + 2 <= li) {
        const uint8_t code = p[o], len = p[o + 1];
        const uint8_t* v = p + o + 2;
        if (o + 2 + len > li) break;
        switch (code) {
        case 0x05: addLine(m.detail, "  local reference %s", hexs(v, len).c_str()); break;
        case 0xC3: addLine(m.detail, "  QoS maintenance %s", hexs(v, len).c_str()); break;
        case 0xC1:
            if (len == 2) {
                static const Dict kReason[] = {{0, "reason not specified"}, {1, "protocol procedure error"}, {2, "incorrect checksum"}, {3, "PDU discarded due to congestion"},
                    {4, "header syntax error"}, {5, "segmentation needed but not permitted"}, {6, "incomplete PDU received"}, {7, "duplicate option"}, {8, "unknown PDU type"},
                    {0x80, "destination address unreachable"}, {0x81, "destination address unknown"}, {0x90, "unspecified source routing error"}, {0x91, "syntax error in source routing field"},
                    {0x92, "unknown address in source routing field"}, {0x93, "path not acceptable"}, {0xA0, "lifetime expired in transit"}, {0xA1, "lifetime expired during reassembly"},
                    {0xB0, "unsupported option"}, {0xB1, "unsupported protocol version"}, {0xB2, "unsupported security option"}, {0xB3, "unsupported source routing option"},
                    {0xB4, "unsupported record route option"}, {0xB5, "unsupported or unavailable QoS"}, {0xC0, "reassembly interference"}, {0, nullptr}};
                const char* r = dict(kReason, v[0]);
                addLine(m.detail, "  discard reason %02X (%s)%s", v[0], r ? r : "unknown", v[1] ? (", erroneous octet " + std::to_string(v[1])).c_str() : "");
            } else addLine(m.detail, "  discard reason %s", hexs(v, len).c_str());
            break;
        case 0xC4: addLine(m.detail, "  prefix-based scope control %s", hexs(v, len).c_str()); break;
        case 0xC5: addLine(m.detail, "  security %s", hexs(v, len).c_str()); break;
        case 0xC6: addLine(m.detail, "  radius scope control %s", hexs(v, len).c_str()); break;
        case 0xC8: addLine(m.detail, "  source routing %s", hexs(v, len).c_str()); break;
        case 0xCB: addLine(m.detail, "  record route %s", hexs(v, len).c_str()); break;
        case 0xCC: addLine(m.detail, "  padding, %d octets", len); break;
        case 0xCD: addLine(m.detail, "  priority %s", hexs(v, len).c_str()); break;
        default: addLine(m.detail, "  option %02X %s", code, hexs(v, len).c_str()); break;
        }
        o += 2u + len;
    }
    m.summary = std::string("CLNP ") + (type == 0x1C ? "DT" : tn);
    if (li >= n) return;
    const uint8_t* d = p + li;
    const size_t dn = n - li;
    if (type == 0x01) {
        // an error report carries the header of the PDU that failed
        if (depth < 2 && dn >= 9 && d[0] == 0x81) { addLine(m.detail, "  the PDU that caused it:"); Vdl2Message inner; readClnp(d, dn, inner, c, depth + 1); m.detail += inner.detail; }
        return;
    }
    if (sp) joinSegments(d, dn, pduId, offset, ms, total, m, c); else clnpPayload(d, dn, m, c);
}

void readClnpCompressed(const uint8_t* p, size_t n, Vdl2Message& m, const Ctx& c) {
    m.proto = "CLNP";
    const int type = p[0] >> 4, prio = p[0] & 0xF;
    const bool derived = type == 6 || type == 7 || type == 9 || type == 10;
    const bool sp = type == 1 || type == 3 || derived;
    const bool more = type == 7 || type == 10;
    const bool exp = p[3] & 0x80;
    const size_t hl = 4 + (exp ? 1 : 0) + (sp ? 2 : 0) + (derived ? 4 : 0);
    if (n < hl) { addLine(m.detail, "CLNP (compressed header): cut short: %s", hexs(p, n).c_str()); m.summary = "CLNP (compressed, cut short)"; return; }
    size_t o = 4;
    uint32_t lref = p[3] & 0x7F;
    if (exp) lref = (lref << 8) | p[o++];
    uint32_t pduId = 0, offset = 0, total = 0;
    if (sp) { pduId = be16(p + o); o += 2; }
    if (derived) { offset = be16(p + o); total = be16(p + o + 2); o += 4; }
    addLine(m.detail, "CLNP (ICAO SNDCF compressed header): type %d, local reference %X, priority %d, lifetime %.1f s, flags %02X", type, lref, prio, p[1] * 0.5, p[2]);
    if (sp) addLine(m.detail, "  PDU id %04X%s", pduId, derived ? "" : ", not segmented");
    if (derived) addLine(m.detail, "  segment at offset %u of %u octets%s", offset, total, more ? ", more follows" : ", last");
    m.summary = "CLNP compressed header";
    if (o >= n) return;
    if (derived) joinSegments(p + o, n - o, pduId, offset, more, total, m, c); else clnpPayload(p + o, n - o, m, c);
}

// ---- X.25 (ISO 8208) with the ICAO SNDCF
const Dict kClearCauses[] = {{0x00, "DTE originated"}, {0x01, "number busy"}, {0x03, "invalid facility request"}, {0x05, "network congestion"}, {0x09, "remote procedure error"},
    {0x0D, "not obtainable"}, {0x13, "local procedure error"}, {0x15, "ROA out of order"}, {0x19, "reverse charging acceptance not subscribed"}, {0x21, "incompatible destination"},
    {0x29, "fast select acceptance not subscribed"}, {0x39, "ship absent"}, {0, nullptr}};
const Dict kResetCauses[] = {{0x00, "DTE originated"}, {0x01, "out of order"}, {0x03, "remote procedure error"}, {0x05, "local procedure error"}, {0x07, "network congestion"},
    {0x09, "remote DTE operational"}, {0x0F, "network operational"}, {0x11, "incompatible destination"}, {0x1D, "network out of order"}, {0, nullptr}};
const Dict kRestartCauses[] = {{0x01, "local procedure error"}, {0x03, "network congestion"}, {0x07, "network operational"}, {0, nullptr}};
const Dict kDiag[] = {
    {0x00, "cleared by system management"}, {0x01, "invalid P(S)"}, {0x02, "invalid P(R)"}, {0x10, "packet type invalid"}, {0x11, "packet type invalid for state r1"},
    {0x12, "packet type invalid for state r2"}, {0x13, "packet type invalid for state r3"}, {0x14, "packet type invalid for state p1"}, {0x15, "packet type invalid for state p2"},
    {0x16, "packet type invalid for state p3"}, {0x17, "packet type invalid for state p4"}, {0x18, "packet type invalid for state p5"}, {0x19, "packet type invalid for state p6"},
    {0x1A, "packet type invalid for state p7"}, {0x1B, "packet type invalid for state d1"}, {0x1C, "packet type invalid for state d2"}, {0x1D, "packet type invalid for state d3"},
    {0x20, "packet not allowed"}, {0x21, "unidentifiable packet"}, {0x22, "call on one-way logical channel"}, {0x23, "invalid packet type on a PVC"},
    {0x24, "packet on unassigned logical channel"}, {0x25, "reject not subscribed to"}, {0x26, "packet too short"}, {0x27, "packet too long"},
    {0x28, "invalid general format identifier"}, {0x29, "restart packet with non-zero reserved bits"}, {0x2A, "packet type not compatible with facility"},
    {0x2B, "unauthorized interrupt confirmation"}, {0x2C, "unauthorized interrupt"}, {0x2D, "unauthorized reject"}, {0x30, "time expired"},
    {0x31, "time expired for incoming call"}, {0x32, "time expired for clear indication"}, {0x33, "time expired for reset indication"}, {0x34, "time expired for restart indication"},
    {0x35, "time expired for call deflection"}, {0x40, "call setup or call clearing problem"}, {0x41, "facility code not allowed"}, {0x42, "facility parameter not allowed"},
    {0x43, "invalid called DTE address"}, {0x44, "invalid calling DTE address"}, {0x45, "invalid facility length"}, {0x46, "incoming call barred"},
    {0x47, "no logical channel available"}, {0x48, "call collision"}, {0x49, "duplicate facility requested"}, {0x4A, "non-zero address length"},
    {0x4B, "non-zero facility length"}, {0x4C, "facility not provided when expected"}, {0x4D, "invalid ITU-T specified DTE facility"},
    {0x4E, "max number of call redirections or deflections exceeded"}, {0x50, "miscellaneous"}, {0x51, "improper cause code from DTE"}, {0x52, "not aligned octet"},
    {0x53, "inconsistent Q-bit setting"}, {0x54, "NUI problem"}, {0x55, "ICRD problem"}, {0x70, "international problem"}, {0x71, "remote network problem"},
    {0x72, "international protocol problem"}, {0x73, "international link out of order"}, {0x74, "international link busy"}, {0x75, "transit network facility problem"},
    {0x76, "remote network facility problem"}, {0x77, "international routing problem"}, {0x78, "temporary routing problem"}, {0x79, "unknown called DNIC"},
    {0x7A, "maintenance action"}, {0x80, "version number not supported"}, {0x81, "invalid length field"}, {0x82, "call collision resolution"},
    {0x83, "proposed directory size too large"}, {0x84, "LREF cancellation not supported"}, {0x85, "received DTE refused, received NET refused or invalid NET selector"},
    {0x86, "invalid SNCR field"}, {0x87, "ACA compression not supported"}, {0x88, "LREF compression not supported"}, {0x8F, "deflate compression not supported"},
    {0x90, "idle timer expired"}, {0x91, "need to reuse the circuit"}, {0x92, "system local error"}, {0x93, "invalid SEL field value in received NET"},
    {0xE1, "OSI network disconnect (transient)"}, {0xE2, "OSI network disconnect (permanent)"}, {0xE3, "OSI network reject, reason unspecified (transient)"},
    {0xE4, "OSI network reject, reason unspecified (permanent)"}, {0xE5, "OSI network reject, QoS not available (transient)"}, {0xE6, "OSI network reject, QoS not available (permanent)"},
    {0xE7, "OSI network reject, NSAP unreachable (transient)"}, {0xE8, "OSI network reject, NSAP unreachable (permanent)"}, {0xE9, "OSI network reset, no reason given"},
    {0xEA, "OSI network reset, congestion"}, {0xEB, "OSI network reject, NSAP address unknown (permanent)"}, {0xF0, "system lack of resources"},
    {0xF1, "higher level initiated disconnect (normal)"}, {0xF2, "incompatible information in user data"}, {0xF3, "higher level initiated disconnect, incompatible data"},
    {0xF4, "higher level initiated reject, no reason given (transient)"}, {0xF5, "higher level initiated reject, no reason given (permanent)"},
    {0xF6, "higher level initiated reject, QoS not available (transient)"}, {0xF7, "higher level initiated reject, QoS not available (permanent)"},
    {0xF8, "higher level initiated reject, incompatible data"}, {0xF9, "unrecognized protocol ID"}, {0xFA, "higher level initiated reset, user resync"}, {0, nullptr}};

std::string compressionName(int c) {
    std::string s;
    if (c & 0x40) s += "ACA ";
    if (c & 0x20) s += "DEFLATE ";
    if (c & 0x02) s += "LREF ";
    if (c & 0x01) s += "LREF-CAN ";
    if (s.empty()) return "none";
    s.pop_back();
    return s;
}

// The facilities of a call packet: returns the octets used (the length octet included), or -1
int readFacilities(const uint8_t* p, size_t n, Vdl2Message& m) {
    if (!n) return -1;
    const size_t fl = p[0];
    if (1 + fl > n) return -1;
    size_t i = 1;
    const size_t end = 1 + fl;
    while (i < end) {
        const uint8_t code = p[i++];
        size_t len = ((code >> 6) & 3) + 1;
        if (len == 4) { if (i >= end) break; len = p[i++]; }
        if (i + len > end) break;
        const uint8_t* v = p + i;
        char b[120];
        switch (code) {
        case 0x00: break;
        case 0x01: addLine(m.detail, "  facility: fast select %s", len && (v[0] & 0x80) ? "requested" : "not requested"); break;
        case 0x08: addLine(m.detail, "  facility: called line address modified %s", hexs(v, len).c_str()); break;
        case 0x42:
            if (len >= 2 && v[0] <= 15 && v[1] <= 15) { snprintf(b, sizeof b, "max packet size: %u octets from the calling DTE, %u from the called DTE", 1u << v[1], 1u << v[0]); addLine(m.detail, "  facility: %s", b); }
            else addLine(m.detail, "  facility: max packet size %s", hexs(v, len).c_str());
            break;
        case 0x43:
            if (len >= 2) addLine(m.detail, "  facility: window size: %u packets from the calling DTE, %u from the called DTE", v[1], v[0]);
            else addLine(m.detail, "  facility: window size %s", hexs(v, len).c_str());
            break;
        case 0xC9: addLine(m.detail, "  facility: called address extension %s", hexAscii(v, len).c_str()); break;
        default: addLine(m.detail, "  facility %02X: %s", code, hexs(v, len).c_str()); break;
        }
        i += len;
    }
    return (int)(1 + fl);
}

void readSndcfError(const uint8_t* p, size_t n, Vdl2Message& m, const Ctx& c) {
    static const char* kErr[] = {"compressed NPDU with unrecognized local reference", "creation of directory entry outside of sender's permitted range", "directory entry exists",
        "local reference greater than maximum value accepted", "data unit identifier missing when SP=1", "reserved", "reserved", "compressed CLNP PDU with unrecognized type",
        "local reference cancellation error"};
    if (n < 3) { m.summary += ": SNDCF error report"; addLine(m.detail, "SNDCF error report cut short: %s", hexs(p, n).c_str()); return; }
    addLine(m.detail, "ICAO SNDCF error report: cause %02X (%s), local reference %02X", p[1], p[1] <= 8 ? kErr[p[1]] : "unknown", p[2]);
    m.summary += std::string(": SNDCF error: ") + (p[1] <= 8 ? kErr[p[1]] : "unknown");
    if (n > 3) { addLine(m.detail, "the PDU that caused it:"); const std::string s = m.summary; userData(p + 3, n - 3, m, c); m.summary = s; }
}

void userData(const uint8_t* p, size_t n, Vdl2Message& m, const Ctx& c) {
    if (!n) return;
    const std::string head = m.summary;
    const int top = p[0] >> 4;
    if (p[0] == 0x81) readClnp(p, n, m, c);
    else if (p[0] == 0x82) readEsis(p, n, m);
    else if (p[0] == 0x85) readIdrp(p, n, m);
    else if (p[0] == 0xE0) { readSndcfError(p, n, m, c); return; }
    else if (top < 4 || top == 6 || top == 7 || top == 9 || top == 10) readClnpCompressed(p, n, m, c);
    else { m.summary = "unknown network protocol"; addLine(m.detail, "network protocol %02X not known, %zu octets: %s", p[0], n, hexs(p, n).c_str()); }
    if (!head.empty()) m.summary = head + ": " + m.summary;
}

void readX25(const uint8_t* p, size_t n, Vdl2Message& m, const Ctx& c) {
    m.proto = "X.25";
    if (n < 3) { m.summary = "X.25: cut short"; addLine(m.detail, "X.25 (ISO 8208): %zu octets, too short", n); return; }
    const int gfi = p[0] >> 4, lcn = ((p[0] & 0xF) << 8) | p[1];
    const uint8_t pt = p[2];
    char b[160];
    if (!(pt & 1)) {
        const int pr = pt >> 5, ps = (pt >> 1) & 7;
        const bool more = (pt >> 4) & 1;
        snprintf(b, sizeof b, "X.25 data, LCN %d, P(S) %d P(R) %d%s", lcn, ps, pr, more ? ", more follows" : "");
        m.summary = b;
        addLine(m.detail, "X.25 (ISO 8208): data packet, GFI %X, logical channel %d, P(S) %d, P(R) %d%s, %zu octets of user data", gfi, lcn, ps, pr, more ? ", M bit set" : "", n - 3);
        const uint8_t* u = p + 3;
        const size_t un = n - 3;
        if (!un) return;
        if (!c.rs) {
            if (more) { m.summary += ": piece of a longer message"; addLine(m.detail, "  the message continues in the next packet"); return; }
            userData(u, un, m, c);
            return;
        }
        std::vector<uint8_t> whole;
        int pieces = 0;
        const uint64_t key = pairKey(c);
        if (seqAdd(c.rs->x25[key], (uint32_t)ps, 7, u, un, !more, c.now, whole, pieces)) {
            c.rs->x25.erase(key);
            if (pieces > 1) addLine(m.detail, "  user data joined from %d packets, %zu octets", pieces, whole.size());
            userData(whole.data(), whole.size(), m, c);
        } else {
            const auto& e = c.rs->x25[key];
            m.summary += ": piece of a longer message";
            addLine(m.detail, "  piece %d of a longer message, %zu octets so far, the rest is in the next packets", e.pieces, e.data.size());
        }
        return;
    }
    switch (pt) {
    case 0x0B: case 0x0F: {
        const bool req = pt == 0x0B;
        std::string called, calling;
        size_t o = 3;
        if (n > 3) {
            const int lcalling = p[3] >> 4, lcalled = p[3] & 0xF;
            o = 4;
            const size_t bytes = (size_t)(lcalling + lcalled + 1) / 2;
            if (o + bytes <= n) { called = bcd(p + o, lcalled, 0); calling = bcd(p + o, lcalling, lcalled); o += bytes; }
        }
        snprintf(b, sizeof b, "X.25 %s, LCN %d", req ? "call request" : "call accepted", lcn);
        m.summary = b;
        addLine(m.detail, "X.25 (ISO 8208): %s, logical channel %d", req ? "call request" : "call accepted", lcn);
        addLine(m.detail, "  called DTE %s, calling DTE %s", called.empty() ? "-" : called.c_str(), calling.empty() ? "-" : calling.c_str());
        const int fl = o < n ? readFacilities(p + o, n - o, m) : -1;
        if (fl < 0) { if (o < n) addLine(m.detail, "  facilities cut short: %s", hexs(p + o, n - o).c_str()); return; }
        o += (size_t)fl;
        if (o >= n) return;
        if (req) {
            if (p[o] == 0xC1 && o + 1 < n && p[o + 1] >= 4 && o + 2 + p[o + 1] <= n && p[o + 2] == 1) {
                const uint8_t* s = p + o + 2;
                addLine(m.detail, "  ICAO SNDCF: version %d, compression %s", s[0], compressionName(s[3]).c_str());
                o += 2u + p[o + 1];
            } else if (p[o] == 0xC1 && o + 1 < n && o + 2 + p[o + 1] <= n) {
                addLine(m.detail, "  ICAO SNDCF, %d octets: %s", p[o + 1], hexs(p + o + 2, p[o + 1]).c_str());
                o += 2u + p[o + 1];
            }
        } else {
            addLine(m.detail, "  compression chosen: %s", compressionName(p[o]).c_str());
            o++;
        }
        if (o < n) userData(p + o, n - o, m, c);        // fast select: a data PDU rides on the call
        return;
    }
    case 0x13: case 0x17: {
        snprintf(b, sizeof b, "X.25 clear %s, LCN %d", pt == 0x13 ? "request" : "confirm", lcn);
        m.summary = b;
        addLine(m.detail, "X.25 (ISO 8208): clear %s, logical channel %d", pt == 0x13 ? "request" : "confirmation", lcn);
        if (pt == 0x13 && n >= 4) {
            const int cause = p[3] & 0x80 ? 0 : p[3];
            const char* cn = dict(kClearCauses, cause);
            addLine(m.detail, "  cause %02X (%s)", p[3], p[3] & 0x80 ? "DTE originated" : cn ? cn : "unknown");
            if (n >= 5) { const char* dn = dict(kDiag, p[4]); addLine(m.detail, "  diagnostic %02X (%s)", p[4], dn ? dn : "unknown"); m.summary += std::string(": ") + (dn ? dn : "diagnostic"); }
        }
        return;
    }
    case 0x1B: case 0x1F: case 0xFB: case 0xFF: case 0x23: case 0x27: case 0xF1: {
        const char* nm = pt == 0x1B ? "reset request" : pt == 0x1F ? "reset confirmation" : pt == 0xFB ? "restart request" : pt == 0xFF ? "restart confirmation" : pt == 0x23 ? "interrupt" : pt == 0x27 ? "interrupt confirmation" : "diagnostic";
        snprintf(b, sizeof b, "X.25 %s, LCN %d", nm, lcn);
        m.summary = b;
        addLine(m.detail, "X.25 (ISO 8208): %s, logical channel %d", nm, lcn);
        if ((pt == 0x1B || pt == 0xFB) && n >= 4) {
            const char* cn = p[3] & 0x80 ? "DTE originated" : dict(pt == 0x1B ? kResetCauses : kRestartCauses, p[3]);
            addLine(m.detail, "  cause %02X (%s)", p[3], cn ? cn : "unknown");
            if (n >= 5) { const char* dn = dict(kDiag, p[4]); addLine(m.detail, "  diagnostic %02X (%s)", p[4], dn ? dn : "unknown"); }
        } else if (pt == 0xF1 && n >= 4) {
            const char* dn = dict(kDiag, p[3]);
            addLine(m.detail, "  diagnostic %02X (%s)", p[3], dn ? dn : "unknown");
        }
        return;
    }
    default:
        if ((pt & 0x1F) == 0x01 || (pt & 0x1F) == 0x05 || (pt & 0x1F) == 0x09) {
            const char* nm = (pt & 0x1F) == 0x01 ? "RR" : (pt & 0x1F) == 0x05 ? "RNR" : "REJ";
            snprintf(b, sizeof b, "X.25 %s, LCN %d, P(R) %d", nm, lcn, pt >> 5);
            m.summary = b;
            addLine(m.detail, "X.25 (ISO 8208): %s, logical channel %d, P(R) %d", nm, lcn, pt >> 5);
            return;
        }
        snprintf(b, sizeof b, "X.25 packet type %02X, LCN %d", pt, lcn);
        m.summary = b;
        addLine(m.detail, "X.25 (ISO 8208): packet type %02X, logical channel %d, %zu octets", pt, lcn, n);
    }
}

// a VDL2 frequency: 12 bits of 10 kHz steps above 100 MHz (rounded to the 25 kHz raster), four bits of modulation support
std::string xidFreq(const uint8_t* v) {
    uint32_t khz = ((be16(v) & 0x0FFF) + 10000) * 10;
    if (khz % 25) khz += 25 - khz % 25;
    const int mod = v[0] >> 4;
    char b[80];
    snprintf(b, sizeof b, "%.3f MHz (%s)", khz / 1000.0, mod & 2 ? (mod & 4 ? "VDL Mode 2 and 3" : "VDL Mode 2") : mod & 4 ? "VDL Mode 3" : "no modulation listed");
    return b;
}

void readXid(const uint8_t* p, size_t n, Vdl2Message& m) {
    m.proto = "XID";
    int h = 1, r = 1;
    std::string lines;
    bool any = false;
    if (n >= 3 && p[0] == 0x82) {
        size_t o = 1;
        while (o + 3 <= n) {
            const uint8_t gid = p[o];
            const size_t glen = ((size_t)p[o + 1] << 8) | p[o + 2];
            o += 3;
            if (o + glen > n) break;
            const uint8_t* g = p + o;
            size_t k = 0;
            while (k + 2 <= glen) {
                const uint8_t id = g[k], len = g[k + 1];
                const uint8_t* v = g + k + 2;
                if (k + 2 + len > glen) break;
                any = true;
                if (gid == 0xF0) {
                    switch (id) {
                    case 0x00: addLine(lines, "  parameter set %.*s", (int)len, (const char*)v); break;
                    case 0x01: if (len >= 1) { r = v[0] & 1; h = (v[0] >> 1) & 1; addLine(lines, "  connection management: h %d, r %d", h, r); } break;
                    case 0x03: if (len >= 1) addLine(lines, "  XID sequencing: seq %d, retry %d", v[0] & 7, v[0] >> 4); break;
                    case 0x83: addLine(lines, "  destination airport %.*s", (int)len, (const char*)v); break;
                    case 0xC1: addLine(lines, "  airport coverage %.*s", (int)len, (const char*)v); break;
                    case 0xC3: addLine(lines, "  nearest airport %.*s", (int)len, (const char*)v); break;
                    case 0x84: case 0xC8:
                        if (len >= 3) {
                            const double la = signed12((v[0] << 4) | (v[1] >> 4)) / 10.0, lo = signed12(((v[1] & 0xF) << 8) | v[2]) / 10.0;
                            m.hasPos = true; m.lat = la; m.lon = lo;
                            if (id == 0x84 && len >= 4) { m.hasAlt = true; m.altFt = v[3] * 1000; }
                            m.posKind = id == 0x84 ? "XID aircraft location" : "XID ground station location";
                            addLine(lines, "  %s %.1f%c %.1f%c%s", id == 0x84 ? "aircraft location" : "ground station location", std::fabs(la), la < 0 ? 'S' : 'N',
                                    std::fabs(lo), lo < 0 ? 'W' : 'E', id == 0x84 && len >= 4 ? (" " + std::to_string(v[3] * 1000) + " ft").c_str() : "");
                        }
                        break;
                    case 0xC0:
                        if (len % 6 == 0) {
                            addLine(lines, "  frequency support list:");
                            for (size_t e = 0; e < len; e += 6) addLine(lines, "    ground station %06X: %s", vdl2GetAddr(v + e + 2).addr, xidFreq(v + e).c_str());
                        } else addLine(lines, "  frequency support list: %s", hexs(v, len).c_str());
                        break;
                    case 0x82: case 0x41: case 0x48: case 0xC5:
                        if (len % 4 == 0) {
                            std::string a;
                            for (size_t e = 0; e < len; e += 4) { char t[12]; snprintf(t, sizeof t, " %06X", vdl2GetAddr(v + e).addr); a += t; }
                            addLine(lines, "  %s:%s", id == 0x82 ? "alternate ground stations" : id == 0x41 ? "replacement ground stations" : id == 0x48 ? "ground station address filter" : "system mask", a.c_str());
                        } else addLine(lines, "  parameter %02X: %s", id, hexs(v, len).c_str());
                        break;
                    case 0x81: {
                        std::string s2;
                        if (len >= 1) { if (v[0] & 2) s2 += "VDL Mode 2 (D8PSK, 31.5 kbit/s)"; if (v[0] & 4) s2 += std::string(s2.empty() ? "" : ", ") + "VDL Mode 3 (D8PSK, 31.5 kbit/s)"; }
                        addLine(lines, "  modulation support: %s", s2.empty() ? hexs(v, len).c_str() : s2.c_str());
                        break;
                    }
                    case 0x40: if (len >= 2) addLine(lines, "  autotune frequency %s", xidFreq(v).c_str()); break;
                    case 0x04: addLine(lines, "  AVLC specific options %s", hexs(v, len).c_str()); break;
                    case 0x05: addLine(lines, "  expedited SN connection %s", hexs(v, len).c_str()); break;
                    case 0x06:
                        if (len >= 3) {
                            static const char* lcr[] = {"bad local parameter", "out of link layer resources", "out of packet layer resources", "terrestrial network not available",
                                "terrestrial network congestion", "cannot support autotune", "station cannot support initiating handoff"};
                            const char* cn = v[0] < 7 ? lcr[v[0]] : v[0] == 0x7F ? "other unspecified local reason" : v[0] == 0x80 ? "bad global parameter" : v[0] == 0x81 ? "protocol violation"
                                           : v[0] == 0x82 ? "ground system out of resources" : v[0] == 0xFF ? "other unspecified system reason" : "unknown";
                            addLine(lines, "  link connection refused: %s (%02X), delay %u%s%s", cn, v[0], (v[1] << 8) | v[2], len > 3 ? ", additional data " : "", len > 3 ? hexs(v + 3, len - 3).c_str() : "");
                        } else addLine(lines, "  LCR cause %s", hexs(v, len).c_str());
                        break;
                    case 0x02: addLine(lines, "  SQP %s", hexs(v, len).c_str()); break;
                    case 0x42: addLine(lines, "  timer T4 %s", hexs(v, len).c_str()); break;
                    case 0x43: addLine(lines, "  MAC persistence %s", hexs(v, len).c_str()); break;
                    case 0x44: addLine(lines, "  counter M1 %s", hexs(v, len).c_str()); break;
                    case 0x45: addLine(lines, "  timer TM2 %s", hexs(v, len).c_str()); break;
                    case 0x46: addLine(lines, "  timer TG5 %s", hexs(v, len).c_str()); break;
                    case 0x47: addLine(lines, "  timer T3min %s", hexs(v, len).c_str()); break;
                    case 0x49: addLine(lines, "  broadcast connection %s", hexs(v, len).c_str()); break;
                    case 0xC4: addLine(lines, "  ATN router NETs %s", hexAscii(v, len, 48).c_str()); break;
                    case 0xC6: addLine(lines, "  timer TG3 %s", hexs(v, len).c_str()); break;
                    case 0xC7: addLine(lines, "  timer TG4 %s", hexs(v, len).c_str()); break;
                    default: addLine(lines, "  VDL parameter %02X: %s", id, hexs(v, len).c_str()); break;
                    }
                } else if (gid == 0x80) {
                    static const char* pub[] = {"parameter set ID", "procedure classes", "HDLC options", "", "N1 downlink", "N1 uplink", "k downlink", "k uplink", "timer T1 downlink", "counter N2", "timer T2"};
                    if (id < 11 && *pub[id]) addLine(lines, "  public parameter %s: %s", pub[id], id == 0 ? asciiOf(v, len).c_str() : hexs(v, len).c_str());
                    else addLine(lines, "  public parameter %d: %s", id, hexs(v, len).c_str());
                }
                k += 2u + len;
            }
            o += glen;
        }
    }
    const int type = ((int)m.response << 3) | ((int)m.pf << 2) | (h << 1) | r;
    static const char* names[16] = {"XID", "XID_CMD_LCR", "XID_CMD_HO", "GSIF", "XID_CMD_LE", "XID", "XID_CMD_HO", "XID_CMD_LPM",
                                    "XID", "XID", "XID", "XID", "XID_RSP_LE", "XID_RSP_LCR", "XID_RSP_HO", "XID_RSP_LPM"};
    m.frameName = names[type & 15];
    const char* desc = m.frameName == "GSIF" ? "ground station information" : m.frameName == "XID_CMD_LE" ? "link establishment" : m.frameName == "XID_RSP_LE" ? "link establishment response"
                     : m.frameName == "XID_CMD_HO" ? "handoff" : m.frameName == "XID_RSP_HO" ? "handoff response" : m.frameName == "XID_CMD_LCR" ? "link connection refused" : "link parameters";
    m.summary = m.frameName + " (" + desc + ")";
    addLine(m.detail, "XID: %s%s", desc, any ? "" : ", no parameters read");
    m.detail += lines;
}

void readAcars(const uint8_t* p, size_t n, Vdl2Message& m) {
    // p: from the SOH; mode .. suffix, check bytes, an optional DEL
    m.proto = "ACARS";
    m.acars = true;
    size_t end = n;
    if (end && p[end - 1] == 0x7F) end--;
    if (end < 1 + 13 + 2) { m.summary = "ACARS: cut short"; addLine(m.detail, "ACARS: %zu octets, too short", n); return; }
    const uint8_t* blk = p + 1;
    const size_t bl = end - 1 - 2;                     // mode .. suffix
    m.acarsCrcOk = acarsCrc(blk, bl + 2) == 0;
    std::vector<uint8_t> t7(bl);
    for (size_t i = 0; i < bl; i++) t7[i] = blk[i] & 0x7F;
    AcarsMessage a;
    if (!acarsParseBlock(t7.data(), (int)bl, a)) { m.summary = "ACARS: not a valid block"; addLine(m.detail, "ACARS: the block layout is not valid"); return; }
    m.reg = a.reg; m.flight = a.flightId; m.label = a.label; m.labelText = a.labelText; m.msgNum = a.msgNum; m.acarsText = a.text; m.blockId = a.blockId;
    std::string s = "ACARS " + a.label;
    if (!a.labelText.empty()) s += " (" + a.labelText + ")";
    if (!a.flightId.empty()) s += " " + a.flightId;
    std::string one;
    for (char c : a.text) one += (c == '\n' || c == '\r') ? ' ' : c;
    if (!one.empty()) s += ": " + (one.size() > 80 ? one.substr(0, 80) + "..." : one);
    m.summary = s;
    addLine(m.detail, "ACARS: mode %c, registration %s, label %s%s%s, block id %c%s", a.mode ? a.mode : '-', a.reg.empty() ? "-" : a.reg.c_str(), a.label.c_str(),
            a.labelText.empty() ? "" : " ", a.labelText.c_str(), a.blockId ? a.blockId : '-', m.acarsCrcOk ? "" : ", check sequence FAILED");
    if (!a.msgNum.empty() || !a.flightId.empty()) addLine(m.detail, "  message %s, flight %s", a.msgNum.empty() ? "-" : a.msgNum.c_str(), a.flightId.empty() ? "-" : a.flightId.c_str());
    if (!a.decoded.empty()) addLine(m.detail, "  reading: %s", a.decoded.c_str());
    AeroPosition pos;
    std::string adsc;
    const bool uplink = !m.fromAir;
    if (aeroPositionFromMessage(a.label, a.text, uplink, pos, &adsc) && pos.valid) {
        m.hasPos = true; m.lat = pos.lat; m.lon = pos.lon; m.hasAlt = pos.hasAlt; m.altFt = pos.altFt; m.posKind = pos.kind;
        if (!pos.flightId.empty() && m.flight.empty()) m.flight = pos.flightId;
    }
    if (!adsc.empty()) { addLine(m.detail, "ADS-C (FANS-1/A):"); m.detail += adsc; if (m.detail.back() != '\n') m.detail += '\n'; m.summary = "ACARS " + a.label + " ADS-C" + (m.hasPos ? " position report" : ""); }
    // FANS-1/A CPDLC (ARINC 622 IMI AT1 / CR1 / CC1 / DR1): found and named, the ASN.1 PER body is not decoded
    for (const char* imi : {".AT1", ".CR1", ".CC1", ".DR1"}) {
        const size_t at = a.text.find(imi);
        if (at != std::string::npos) {
            const std::string what = imi[1] == 'A' ? "CPDLC message" : imi[1] == 'C' && imi[2] == 'R' ? "CPDLC connect request" : imi[2] == 'C' ? "CPDLC connect confirm" : "CPDLC disconnect";
            addLine(m.detail, "FANS-1/A %s (IMI %s), ground facility %s, %zu hex digits of ASN.1 PER (not decoded)", what.c_str(), imi + 1, a.text.substr(at > 7 ? at - 7 : 0, at > 7 ? 7 : at).c_str(),
                    a.text.size() - at - 4);
            m.summary = "ACARS " + a.label + " FANS-1/A " + what;
            break;
        }
    }
    if (m.hasPos) addLine(m.detail, "  position %.4f %.4f%s (%s)", m.lat, m.lon, m.hasAlt ? (" " + std::to_string(m.altFt) + " ft").c_str() : "", m.posKind.c_str());
}

} // namespace

void Vdl2Reasm::prune(double now) {
    auto old = [&](double last) { return std::fabs(now - last) > 120; };
    for (auto it = x25.begin(); it != x25.end();) it = old(it->second.last) ? x25.erase(it) : std::next(it);
    for (auto it = cotp.begin(); it != cotp.end();) it = old(it->second.last) ? cotp.erase(it) : std::next(it);
    for (auto it = clnp.begin(); it != clnp.end();) it = old(it->second.last) ? clnp.erase(it) : std::next(it);
}

bool vdl2ParseFrame(const uint8_t* f, size_t n, Vdl2Message& m, Vdl2Reasm* rs, double now) {
    if (n < 9) return false;
    if (rs) rs->prune(now);
    const Vdl2Addr d = vdl2GetAddr(f), s = vdl2GetAddr(f + 4);
    m.dst = d.addr; m.dstType = d.type; m.src = s.addr; m.srcType = s.type;
    m.onGround = d.status; m.response = s.status;
    m.fromAir = s.type == 1; m.fromGround = s.type == 4 || s.type == 5;
    const uint8_t c = f[8];
    const uint8_t* info = f + 9;
    const size_t in = n - 9;
    m.detail.clear();
    Ctx cx;
    cx.rs = rs; cx.now = now; cx.src = s.addr; cx.dst = d.addr;
    addLine(m.detail, "AVLC: %06X (%s) to %06X (%s), %s, %s", s.addr, vdl2AddrTypeName(s.type), d.addr, vdl2AddrTypeName(d.type), m.onGround ? "on ground" : "airborne", m.response ? "response" : "command");
    if (!(c & 1)) {
        m.frameKind = 'I'; m.frameName = "I"; m.ns = (c >> 1) & 7; m.nr = (c >> 5) & 7; m.pf = (c >> 4) & 1;
        addLine(m.detail, "  I frame, N(S) %d, N(R) %d%s, %zu octets of information", m.ns, m.nr, m.pf ? ", poll" : "", in);
        if (in > 3 && info[0] == 0xFF && info[1] == 0xFF && info[2] == 0x01) readAcars(info + 2, in - 2, m);
        else if (in) readX25(info, in, m, cx);
        else m.summary = "I frame, no information";
    } else if ((c & 3) == 1) {
        static const char* sn[4] = {"RR", "RNR", "REJ", "SREJ"};
        m.frameKind = 'S'; m.frameName = sn[(c >> 2) & 3]; m.nr = (c >> 5) & 7; m.pf = (c >> 4) & 1;
        m.summary = m.frameName + " N(R) " + std::to_string(m.nr);
        addLine(m.detail, "  S frame %s, N(R) %d%s", m.frameName.c_str(), m.nr, m.pf ? ", poll/final" : "");
    } else {
        m.frameKind = 'U'; m.pf = (c >> 4) & 1;
        const int mf = (c >> 2) & 0x3B;
        switch (mf) {
        case 0x00: m.frameName = "UI"; break; case 0x03: m.frameName = "DM"; break; case 0x10: m.frameName = "DISC"; break;
        case 0x18: m.frameName = "UA"; break; case 0x21: m.frameName = "FRMR"; break; case 0x2B: m.frameName = "XID"; break;
        case 0x38: m.frameName = "TEST"; break; default: m.frameName = "U?"; break;
        }
        addLine(m.detail, "  U frame %s%s, %zu octets of information", m.frameName.c_str(), m.pf ? ", poll/final" : "", in);
        if (mf == 0x2B) readXid(info, in, m);
        else if (mf == 0x00 && in > 3 && info[0] == 0xFF && info[1] == 0xFF && info[2] == 0x01) readAcars(info + 2, in - 2, m);
        else m.summary = m.frameName + (in ? " (" + std::to_string(in) + " octets)" : "");
    }
    return true;
}

// ---------------------------------------------------------------- builders

std::vector<uint8_t> vdl2AcarsInfo(const std::string& reg, const std::string& label, char blockId, const std::string& text, char ack, char mode) {
    AcarsBlockSpec s;
    s.mode = mode;
    s.reg = reg;
    s.ack = (uint8_t)ack;
    s.label[0] = label.size() > 0 ? label[0] : 'H';
    s.label[1] = label.size() > 1 ? label[1] : '1';
    s.blockId = blockId;
    s.text = text;
    s.hasText = !text.empty();
    std::vector<uint8_t> f = acarsBuildFrame(s, 0);    // SYN SYN SOH ... DEL
    std::vector<uint8_t> out = {0xFF, 0xFF};
    out.insert(out.end(), f.begin() + 2, f.end());
    return out;
}

std::vector<uint8_t> vdl2X25ClnpData(int lcn, int ps, int pr, const std::vector<uint8_t>& src, const std::vector<uint8_t>& dst, const std::vector<uint8_t>& user) {
    std::vector<uint8_t> o = {(uint8_t)(0x10 | ((lcn >> 8) & 0xF)), (uint8_t)(lcn & 0xFF), (uint8_t)(((pr & 7) << 5) | ((ps & 7) << 1))};
    std::vector<uint8_t> h = {0x81, 0, 0x01, 60, 0x1C, 0, 0, 0, 0};
    h.push_back((uint8_t)dst.size()); h.insert(h.end(), dst.begin(), dst.end());
    h.push_back((uint8_t)src.size()); h.insert(h.end(), src.begin(), src.end());
    h[1] = (uint8_t)h.size();
    std::vector<uint8_t> tp = {4, 0xF0, 0x00, 0x00, 0x80};   // COTP DT: destination reference 0, end of TSDU, sequence 0
    tp.insert(tp.end(), user.begin(), user.end());
    const size_t seg = h.size() + tp.size();
    h[5] = (uint8_t)(seg >> 8); h[6] = (uint8_t)seg;
    o.insert(o.end(), h.begin(), h.end());
    o.insert(o.end(), tp.begin(), tp.end());
    return o;
}

std::vector<uint8_t> vdl2X25CallRequest(int lcn, const std::string& called, const std::string& calling) {
    std::vector<uint8_t> o = {(uint8_t)(0x10 | ((lcn >> 8) & 0xF)), (uint8_t)(lcn & 0xFF), 0x0B, (uint8_t)((calling.size() << 4) | (called.size() & 0xF))};
    const std::string all = called + calling;
    for (size_t i = 0; i < all.size(); i += 2) {
        const int hi = all[i] - '0', lo = i + 1 < all.size() ? all[i + 1] - '0' : 0;
        o.push_back((uint8_t)((hi << 4) | lo));
    }
    o.push_back(0);                                     // no facilities
    o.push_back(0xC1); o.push_back(0x04); o.push_back(0x01); o.push_back(0x00); o.push_back(0x00); o.push_back(0x00);   // ICAO SNDCF version 1, no compression
    return o;
}

namespace {
void putLoc(std::vector<uint8_t>& g, double lat, double lon) {
    const int la = (int)std::lround(lat * 10) & 0xFFF, lo = (int)std::lround(lon * 10) & 0xFFF;
    g.push_back((uint8_t)(la >> 4)); g.push_back((uint8_t)(((la & 0xF) << 4) | (lo >> 8))); g.push_back((uint8_t)(lo & 0xFF));
}
std::vector<uint8_t> xid(const std::vector<uint8_t>& vdlParams) {
    std::vector<uint8_t> o = {0x82, 0x80, 0x00, 0x03, 0x01, 0x01, 0x00};   // public group: parameter set id
    o.push_back(0xF0); o.push_back((uint8_t)(vdlParams.size() >> 8)); o.push_back((uint8_t)vdlParams.size());
    o.insert(o.end(), vdlParams.begin(), vdlParams.end());
    return o;
}
void param(std::vector<uint8_t>& g, uint8_t id, const std::vector<uint8_t>& v) { g.push_back(id); g.push_back((uint8_t)v.size()); g.insert(g.end(), v.begin(), v.end()); }
} // namespace

std::vector<uint8_t> vdl2XidGsif(double lat, double lon, const std::string& airports) {
    std::vector<uint8_t> g;
    param(g, 0x00, {'V', '2'});
    std::vector<uint8_t> loc; putLoc(loc, lat, lon);
    param(g, 0xC8, loc);
    param(g, 0xC1, std::vector<uint8_t>(airports.begin(), airports.end()));
    return xid(g);
}

std::vector<uint8_t> vdl2XidAircraftLe(double lat, double lon, int altFt, const std::string& dest) {
    std::vector<uint8_t> g;
    param(g, 0x00, {'V', '2'});
    param(g, 0x01, {0x00});
    std::vector<uint8_t> loc; putLoc(loc, lat, lon); loc.push_back((uint8_t)std::max(0, std::min(255, (altFt + 500) / 1000)));
    param(g, 0x84, loc);
    param(g, 0x83, std::vector<uint8_t>(dest.begin(), dest.end()));
    return xid(g);
}

} // namespace dect2
