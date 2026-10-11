// ATN application data on VDL Mode 2: see vdl2_app.h.
#include "dect2/vdl2_app.h"
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <vector>

namespace dect2 {
namespace {

// ---- unaligned PER bit reader: any failure sets err and later reads give 0 ----
struct Bits {
    std::vector<uint8_t> d;     // one bit per octet, first bit first
    size_t pos = 0;
    bool err = false;
    Bits() = default;
    Bits(const uint8_t* p, size_t n) { for (size_t i = 0; i < n; i++) for (int b = 7; b >= 0; b--) d.push_back((p[i] >> b) & 1); }
    size_t left() const { return d.size() - pos; }
    uint32_t u(int n) {
        uint32_t v = 0;
        if (n < 0 || left() < (size_t)n) { err = true; pos = d.size(); return 0; }
        for (int i = 0; i < n; i++) v = (v << 1) | d[pos++];
        return v;
    }
    bool bit() { return u(1) != 0; }
    // constrained whole number lo..hi
    long range(long lo, long hi) {
        long n = hi - lo + 1;
        int bits = 0;
        while (((long)1 << bits) < n) bits++;
        return lo + (long)u(bits);
    }
    // length determinant of an unconstrained size (no fragments)
    size_t len() {
        const uint32_t a = u(8);
        if (!(a & 0x80)) return a;
        if ((a & 0xC0) == 0x80) return ((size_t)(a & 0x3F) << 8) | u(8);
        err = true;
        return 0;
    }
    Bits sub(size_t n) {
        Bits r;
        if (left() < n) { err = true; return r; }
        r.d.assign(d.begin() + (long)pos, d.begin() + (long)(pos + n));
        pos += n;
        return r;
    }
    std::vector<uint8_t> octets(size_t n) { std::vector<uint8_t> o; for (size_t i = 0; i < n && !err; i++) o.push_back((uint8_t)u(8)); return o; }
    bool bitString(Bits& out) { const size_t n = len(); if (err) return false; out = sub(n); return !err; }
    // the PER of a whole open payload ends inside its last octet
    bool complete(size_t totalBits) const { return !err && (pos + 7) / 8 == (totalBits + 7) / 8; }
};

void add(std::string& s, const char* fmt, ...) __attribute__((format(printf, 2, 3)));
void add(std::string& s, const char* fmt, ...) {
    char b[400];
    va_list ap; va_start(ap, fmt); vsnprintf(b, sizeof b, fmt, ap); va_end(ap);
    s += b; s += '\n';
}

struct Dict { int id; const char* s; };
const char* dictOf(const Dict* d, int id) { for (; d->s; d++) if (d->id == id) return d->s; return nullptr; }

std::string charsOf(Bits& b, int n) { std::string s; for (int i = 0; i < n; i++) { const char c = (char)b.u(7); s += (c >= 32 && c < 127) ? c : '.'; } return s; }

std::string hexOf(const std::vector<uint8_t>& v) { std::string s; char t[4]; for (uint8_t c : v) { snprintf(t, sizeof t, "%02X", c); s += t; } return s; }

std::string oidText(const std::vector<uint8_t>& v) {
    std::string s;
    unsigned long long acc = 0;
    bool first = true;
    char t[32];
    for (uint8_t c : v) {
        acc = (acc << 7) | (c & 0x7F);
        if (c & 0x80) continue;
        if (first) { snprintf(t, sizeof t, "%llu.%llu", acc < 80 ? acc / 40 : 2, acc < 80 ? acc % 40 : acc - 80); first = false; }
        else snprintf(t, sizeof t, ".%llu", acc);
        s += t;
        acc = 0;
    }
    return s;
}

const char* appName(long q) { return q == 0 ? "ADS-C" : q == 1 ? "CM" : q == 22 ? "CPDLC" : "application"; }

// ---------------------------------------------------------------- CPDLC message elements

struct CpdlcElement { int id; const char* text; const char* args; };
#include "vdl2_cpdlc_elements.inc"

std::string levelText(Bits& b) {
    char t[48];
    if (!b.bit()) {                                    // single level
        switch (b.u(2)) {
        case 0: snprintf(t, sizeof t, "%ld ft", b.range(-60, 7000) * 10); break;
        case 1: snprintf(t, sizeof t, "%ld m", b.range(-30, 25000)); break;
        case 2: snprintf(t, sizeof t, "FL%ld", b.range(30, 700)); break;
        default: snprintf(t, sizeof t, "%ld m", b.range(100, 2500) * 10); break;
        }
        return t;
    }
    std::string s = "block ";                          // block level: two single levels
    for (int i = 0; i < 2; i++) {
        switch (b.u(2)) {
        case 0: snprintf(t, sizeof t, "%ld ft", b.range(-60, 7000) * 10); break;
        case 1: snprintf(t, sizeof t, "%ld m", b.range(-30, 25000)); break;
        case 2: snprintf(t, sizeof t, "FL%ld", b.range(30, 700)); break;
        default: snprintf(t, sizeof t, "%ld m", b.range(100, 2500) * 10); break;
        }
        s += (i ? " to " : "") + std::string(t);
    }
    return s;
}

std::string timeText(Bits& b) { char t[16]; const long h = b.range(0, 23), m = b.range(0, 59); snprintf(t, sizeof t, "%02ld:%02ld", h, m); return t; }

std::string speedText(Bits& b) {
    char t[32];
    switch (b.u(3)) {
    case 0: snprintf(t, sizeof t, "%ld kt IAS", b.range(0, 400)); break;
    case 1: snprintf(t, sizeof t, "%ld km/h IAS", b.range(0, 800)); break;
    case 2: snprintf(t, sizeof t, "%ld kt TAS", b.range(0, 2000)); break;
    case 3: snprintf(t, sizeof t, "%ld km/h TAS", b.range(0, 4000)); break;
    case 4: snprintf(t, sizeof t, "%ld kt GS", b.range(-50, 2000)); break;
    case 5: snprintf(t, sizeof t, "%ld km/h GS", b.range(-100, 4000)); break;
    case 6: snprintf(t, sizeof t, "Mach %.3f", b.range(500, 4000) / 1000.0); break;
    default: b.err = true; return "";
    }
    return t;
}

std::string degreesText(Bits& b) { const bool tr = b.bit(); char t[24]; snprintf(t, sizeof t, "%ld° %s", b.range(1, 360), tr ? "true" : "magnetic"); return t; }

std::string latText(Bits& b, bool lat) {
    char t[48];
    std::string v;
    switch (b.u(2)) {
    case 0: snprintf(t, sizeof t, "%.3f°", (lat ? b.range(0, 90000) : b.range(0, 180000)) / 1000.0); v = t; break;
    case 1: { const long d = lat ? b.range(0, 89) : b.range(0, 179); const long m = b.range(0, 5999); snprintf(t, sizeof t, "%ld°%05.2f'", d, m / 100.0); v = t; break; }
    case 2: { const long d = lat ? b.range(0, 89) : b.range(0, 179); const long m = b.range(0, 59), s = b.range(0, 59); snprintf(t, sizeof t, "%ld°%02ld'%02ld\"", d, m, s); v = t; break; }
    default: b.err = true; return "";
    }
    const bool neg = b.bit();
    return v + (lat ? (neg ? "S" : "N") : (neg ? "W" : "E"));
}

std::string latLonText(Bits& b) {
    const bool hasLat = b.bit(), hasLon = b.bit();
    std::string s;
    if (hasLat) s += latText(b, true);
    if (hasLon) s += (s.empty() ? "" : " ") + latText(b, false);
    return s;
}

std::string fixText(Bits& b) {           // Fix name with an optional latitude / longitude
    const bool ll = b.bit();
    const int n = (int)b.range(1, 5);
    std::string s = charsOf(b, n);
    if (ll) s += " (" + latLonText(b) + ")";
    return s;
}
std::string navaidText(Bits& b) {
    const bool ll = b.bit();
    const int n = (int)b.range(1, 4);
    std::string s = charsOf(b, n);
    if (ll) s += " (" + latLonText(b) + ")";
    return s;
}

std::string positionText(Bits& b) {
    char t[48];
    switch (b.u(3)) {
    case 0: return fixText(b);
    case 1: return navaidText(b);
    case 2: return charsOf(b, 4);
    case 3: return latLonText(b);
    case 4: {
        std::string s = b.bit() ? navaidText(b) : fixText(b);
        s += " " + degreesText(b);
        if (!b.bit()) snprintf(t, sizeof t, " %.1f nm", b.range(0, 9999) / 10.0); else snprintf(t, sizeof t, " %.2f km", b.range(0, 8000) * 0.25);
        return s + t;
    }
    default: b.err = true; return "";
    }
}

std::string freqText(Bits& b) {
    char t[40];
    switch (b.u(2)) {
    case 0: snprintf(t, sizeof t, "%ld kHz", b.range(2850, 28000)); break;
    case 1: snprintf(t, sizeof t, "%.3f MHz", b.range(23600, 27398) * 0.005); break;
    case 2: snprintf(t, sizeof t, "%.3f MHz", b.range(9000, 15999) * 0.025); break;
    default: { const long ch = b.range(32, 57); (void)ch; snprintf(t, sizeof t, "SATCOM channel %s", hexOf(b.octets(12)).c_str()); break; }
    }
    return t;
}

std::string freeText(Bits& b) { const int n = (int)b.range(1, 256); return charsOf(b, n); }
std::string codeText(Bits& b) { std::string s; for (int i = 0; i < 4; i++) s += (char)('0' + b.u(3)); return s; }
std::string vrateText(Bits& b) { char t[32]; if (!b.bit()) snprintf(t, sizeof t, "%ld ft/min", b.range(0, 3000) * 10); else snprintf(t, sizeof t, "%ld m/min", b.range(0, 1000) * 10); return t; }

std::string argText(Bits& b, char kind) {
    switch (kind) {
    case 'L': return levelText(b);
    case 'T': return timeText(b);
    case 'P': return positionText(b);
    case 'S': return speedText(b);
    case 'D': return degreesText(b);
    case 'F': return freeText(b);
    case 'Q': return freqText(b);
    case 'C': return codeText(b);
    case 'V': return vrateText(b);
    default: b.err = true; return "";
    }
}

// one message element: the text with its arguments put in. False when the arguments cannot be read (the rest of the message with them).
bool element(Bits& b, bool down, std::string& out) {
    if (b.bit()) { b.err = true; return false; }                   // an extension element (not in this message set version)
    const int idx = (int)b.range(0, down ? 113 : 237);
    const CpdlcElement* tab = down ? kDownlinkElements : kUplinkElements;
    const size_t cnt = down ? sizeof kDownlinkElements / sizeof kDownlinkElements[0] : sizeof kUplinkElements / sizeof kUplinkElements[0];
    if (b.err || (size_t)idx >= cnt) return false;
    const CpdlcElement& e = tab[idx];
    char head[24];
    snprintf(head, sizeof head, "%cM%d", down ? 'D' : 'U', idx);
    if (strchr(e.args, '?')) { out = std::string(head) + " " + e.text + " (arguments not decoded)"; return false; }
    std::string txt = e.text;
    for (const char* k = e.args; *k; k++) {
        const std::string a = argText(b, *k);
        if (b.err) return false;
        const size_t o = txt.find('[');
        const size_t c = txt.find(']', o);
        if (o == std::string::npos || c == std::string::npos) break;
        txt.replace(o, c - o + 1, a);
    }
    out = std::string(head) + " " + txt;
    return true;
}

const char* kLogicalAck[] = {"required", "not required"};

// ATCDownlinkMessage / ATCUplinkMessage
bool cpdlcMessage(Bits& b, bool down, Vdl2AppResult& r) {
    const bool hasRef = b.bit(), hasAck = b.bit();
    const long id = b.range(0, 63);
    const long ref = hasRef ? b.range(0, 63) : -1;
    const long yr = b.range(1996, 2095), mo = b.range(1, 12), dy = b.range(1, 31);
    const long hh = b.range(0, 23), mm = b.range(0, 59), ss = b.range(0, 59);
    const int ack = hasAck ? (int)b.u(1) : -1;
    if (b.err) return false;
    add(r.detail, "  CPDLC %s message: id %ld%s, %04ld-%02ld-%02ld %02ld:%02ld:%02ld%s%s", down ? "downlink" : "uplink", id,
        ref >= 0 ? (", reference " + std::to_string(ref)).c_str() : "", yr, mo, dy, hh, mm, ss, ack >= 0 ? ", logical acknowledgement " : "", ack >= 0 ? kLogicalAck[ack] : "");
    const bool constrained = b.bit();
    const int count = (int)b.range(1, 7);
    std::string first;
    for (int i = 0; i < count && !b.err; i++) {
        std::string t;
        const bool ok = element(b, down, t);
        if (!t.empty()) { add(r.detail, "    %s", t.c_str()); if (first.empty()) { first = t.substr(t.find(' ') + 1); } }
        if (!ok) { r.summary = "CPDLC " + (first.empty() ? std::string("message") : first); return true; }
    }
    if (b.err) return false;
    if (constrained) add(r.detail, "    route clearance data follows (not decoded)");
    r.summary = "CPDLC " + first + (count > 1 ? " +" + std::to_string(count - 1) : "");
    return true;
}

const char* kUserAbort[] = {"undefined", "no message identification numbers available", "duplicate message identification numbers", "no longer next data authority",
    "current data authority abort", "commanded termination", "invalid response", "time out of synchronisation", "unknown integrity check", "validation failure",
    "unable to decode message", "invalid PDU", "invalid CPDLC message"};
const char* kProviderAbort[] = {"timer expired", "undefined error", "invalid PDU", "protocol error", "communication service error", "communication service failure",
    "invalid QOS parameter", "expected PDU missing"};

// ProtectedDownlinkMessage / ProtectedUplinkMessage: the ATC message inside is another PER encoding in a bit string
bool protectedMsg(Bits& b, bool down, Vdl2AppResult& r) {
    if (b.bit()) { b.err = true; return false; }                   // extension
    const bool hasAlg = b.bit(), hasMsg = b.bit();
    if (hasAlg) { b.err = true; return false; }                    // an algorithm identifier: not read
    Bits in;
    if (hasMsg && !b.bitString(in)) return false;
    Bits chk;
    if (!b.bitString(chk)) return false;
    if (!hasMsg) { add(r.detail, "  CPDLC %s message without content", down ? "downlink" : "uplink"); r.summary = "CPDLC (empty)"; return true; }
    const size_t total = in.d.size();
    return cpdlcMessage(in, down, r) && in.complete(total);
}

bool cpdlcPdu(Bits& b, bool fromAir, Vdl2AppResult& r) {
    if (b.bit()) { b.err = true; return false; }                   // extension
    if (fromAir) {
        switch (b.u(2)) {
        case 0: { if (b.bit()) return false; const int v = (int)b.u(4); if (v > 12) return false; add(r.detail, "  CPDLC user abort: %s", kUserAbort[v]); r.summary = "CPDLC abort"; return true; }
        case 1: { if (b.bit()) return false; const int v = (int)b.u(3); add(r.detail, "  CPDLC provider abort: %s", kProviderAbort[v]); r.summary = "CPDLC provider abort"; return true; }
        case 2: { const bool dsc = b.bit(); if (dsc) add(r.detail, "  (digital signature mode)"); return protectedMsg(b, true, r); }
        default: return protectedMsg(b, true, r);
        }
    }
    const int idx = (int)b.u(3);
    switch (idx) {
    case 0: { if (b.bit()) return false; const int v = (int)b.u(4); if (v > 12) return false; add(r.detail, "  CPDLC user abort: %s", kUserAbort[v]); r.summary = "CPDLC abort"; return true; }
    case 1: { if (b.bit()) return false; const int v = (int)b.u(3); add(r.detail, "  CPDLC provider abort: %s", kProviderAbort[v]); r.summary = "CPDLC provider abort"; return true; }
    case 2: case 3: return protectedMsg(b, false, r);
    default: return false;                                         // forward, forward response: not read
    }
}

// ---------------------------------------------------------------- context management

std::string longTsap(Bits& b) {
    const std::vector<uint8_t> rdp = b.octets(5);
    const bool hasArs = b.bit();
    std::vector<uint8_t> ars;
    if (hasArs) ars = b.octets(3);
    const size_t nsel = (size_t)b.range(10, 11);
    const std::vector<uint8_t> loc = b.octets(nsel);
    return "RDP " + hexOf(rdp) + (hasArs ? " ARS " + hexOf(ars) : "") + " NSEL/TSEL " + hexOf(loc);
}

bool appList(Bits& b, std::string& out) {
    const int n = (int)b.range(1, 256);
    for (int i = 0; i < n && !b.err; i++) {
        const long q = b.range(0, 255), ver = b.range(1, 255);
        std::string addr;
        if (!b.bit()) addr = longTsap(b);
        else { const bool hasArs = b.bit(); std::vector<uint8_t> ars; if (hasArs) ars = b.octets(3); const size_t nsel = (size_t)b.range(10, 11); addr = (hasArs ? "ARS " + hexOf(ars) + " " : "") + "NSEL/TSEL " + hexOf(b.octets(nsel)); }
        char t[64];
        snprintf(t, sizeof t, "    %s (AE qualifier %ld), version %ld, ", appName(q), q, ver);
        out += t + addr + "\n";
    }
    return !b.err;
}

const char* kCmAbort[] = {"timer expired", "undefined error", "invalid PDU", "protocol error", "dialogue acceptance not permitted", "dialogue end not accepted",
    "communication service error", "communication service failure", "invalid QOS parameter", "expected PDU missing"};

bool cmPdu(Bits& b, bool fromAir, Vdl2AppResult& r) {
    if (b.bit()) { b.err = true; return false; }                   // extension
    if (fromAir) {
        switch (b.u(2)) {
        case 0: {
            const bool g = b.bit(), a = b.bit(), fd = b.bit(), dep = b.bit(), dst = b.bit(), etd = b.bit();
            const int fn = (int)b.range(2, 8);
            const std::string flight = charsOf(b, fn);
            const std::string tsap = longTsap(b);
            if (b.err) return false;
            add(r.detail, "  CM logon request: flight %s", flight.c_str());
            add(r.detail, "    aircraft address %s", tsap.c_str());
            std::string apps;
            if (g) { add(r.detail, "    ground initiated applications:"); if (!appList(b, apps)) return false; r.detail += apps; apps.clear(); }
            if (a) { add(r.detail, "    air only initiated applications:"); if (!appList(b, apps)) return false; r.detail += apps; }
            if (fd) { const int n = (int)b.range(4, 8); add(r.detail, "    facility %s", charsOf(b, n).c_str()); }
            if (dep) { add(r.detail, "    departure airport %s", charsOf(b, 4).c_str()); }
            if (dst) { add(r.detail, "    destination airport %s", charsOf(b, 4).c_str()); }
            if (etd) { const long y = b.range(1996, 2095), mo = b.range(1, 12), dy = b.range(1, 31), h = b.range(0, 23), mi = b.range(0, 59); add(r.detail, "    departure time %04ld-%02ld-%02ld %02ld:%02ld", y, mo, dy, h, mi); }
            r.summary = "CM logon request " + flight;
            return !b.err;
        }
        case 1: { const long v = b.range(0, 1); add(r.detail, "  CM contact response: %s", v ? "not successful" : "success"); r.summary = v ? "CM contact failed" : "CM contact response"; return true; }
        default: { if (b.bit()) return false; const int v = (int)b.u(4); if (v > 9) return false; add(r.detail, "  CM abort: %s", kCmAbort[v]); r.summary = "CM abort"; return true; }
        }
    }
    switch (b.u(3)) {
    case 0: {
        const bool air = b.bit(), gnd = b.bit();
        add(r.detail, "  CM logon response");
        std::string apps;
        if (air) { add(r.detail, "    air initiated applications:"); if (!appList(b, apps)) return false; r.detail += apps; apps.clear(); }
        if (gnd) { add(r.detail, "    ground only initiated applications:"); if (!appList(b, apps)) return false; r.detail += apps; }
        r.summary = "CM logon response";
        return !b.err;
    }
    case 2: {
        const int n = (int)b.range(4, 8);
        const std::string fac = charsOf(b, n);
        const std::string tsap = longTsap(b);
        if (b.err) return false;
        add(r.detail, "  CM contact request: facility %s", fac.c_str());
        add(r.detail, "    address %s", tsap.c_str());
        r.summary = "CM contact request " + fac;
        return true;
    }
    case 4: { if (b.bit()) return false; const int v = (int)b.u(4); if (v > 9) return false; add(r.detail, "  CM abort: %s", kCmAbort[v]); r.summary = "CM abort"; return true; }
    default: return false;                                         // update, forward request / response: not read
    }
}

// ---------------------------------------------------------------- ACSE and the presentation layer

bool appPayload(const Bits& payload, bool fromAir, Vdl2AppResult& r) {
    // CPDLC first, then CM (the same order as the ground / air direction decides); each must use the data exactly
    {
        Bits b = payload; Vdl2AppResult t;
        if (cpdlcPdu(b, fromAir, t) && b.complete(payload.d.size())) { r.detail += t.detail; r.summary = t.summary; r.decoded = true; return true; }
    }
    {
        Bits b = payload; Vdl2AppResult t;
        if (cmPdu(b, fromAir, t) && b.complete(payload.d.size())) { r.detail += t.detail; r.summary = t.summary; r.decoded = true; return true; }
    }
    return false;
}

// Association-information: an EXTERNAL whose data is a bit string
bool userInfo(Bits& b, Bits& out) {
    b.bit();                                                       // spare
    const bool dr = b.bit(), ir = b.bit(), dd = b.bit();
    if (dr || ir || dd) { b.err = true; return false; }
    if (b.u(2) != 2) { b.err = true; return false; }               // arbitrary
    return b.bitString(out);
}

bool acse(Bits& b, bool fromAir, Vdl2AppResult& r) {
    if (b.bit()) return false;
    const int kind = (int)b.u(3);
    Bits ui;
    bool hasUi = false;
    switch (kind) {
    case 2: case 3: {                                              // release request / response
        if (b.bit()) return false;
        const bool reason = b.bit(); hasUi = b.bit();
        static const char* rs[] = {"normal", "urgent / not finished", "user defined"};
        long v = -1;
        if (reason) { if (b.bit()) return false; v = (long)b.u(5); }
        add(r.detail, "  X.227 ACSE: release %s%s%s", kind == 2 ? "request" : "response", v >= 0 ? ", reason " : "", v >= 0 ? (v < 3 ? (kind == 2 ? (v == 0 ? "normal" : v == 1 ? "urgent" : "user defined") : (v == 0 ? "normal" : v == 1 ? "not finished" : "user defined")) : "other") : "");
        (void)rs;
        r.summary = kind == 2 ? "ACSE release request" : "ACSE release response";
        if (hasUi && !userInfo(b, ui)) return false;
        break;
    }
    case 4: {                                                      // abort
        if (b.bit()) return false;
        const bool diag = b.bit(); hasUi = b.bit();
        if (b.bit()) return false;
        const int src = (int)b.u(1);
        static const char* dg[] = {"no reason given", "protocol error", "authentication mechanism name not recognised", "authentication mechanism name required", "authentication failure", "authentication required"};
        int d = -1;
        if (diag) { if (b.bit()) return false; d = (int)b.u(3); }
        add(r.detail, "  X.227 ACSE: abort by %s%s%s", src ? "provider" : "user", d >= 0 ? ", " : "", d >= 0 ? (d < 6 ? dg[d] : "unknown diagnostic") : "");
        r.summary = "ACSE abort";
        if (hasUi && !userInfo(b, ui)) return false;
        break;
    }
    case 0: case 1: {                                              // association request / response: the leading optional fields must be absent
        if (b.bit()) return false;
        const int nOpt = kind == 0 ? 15 : 11;
        std::vector<bool> has;
        for (int i = 0; i < nOpt; i++) has.push_back(b.bit());
        if (b.err || has[0]) return false;                         // a protocol version is not expected
        const size_t ol = b.len();
        const std::string ctx = oidText(b.octets(ol));
        std::string res;
        if (kind == 1) {
            if (b.bit()) return false;
            static const char* rn[] = {"accepted", "rejected (permanent)", "rejected (transient)"};
            const int v = (int)b.u(2);
            res = v < 3 ? rn[v] : "unknown";
            if (!b.bit()) { if (b.bit()) return false; b.u(4); } else { if (b.bit()) return false; b.u(2); }
        }
        long calling = -1;
        if (kind == 0) {
            // called AP title (1), called AE qualifier (2), called AP inv (3), called AE inv (4), calling AP title (5), calling AE qualifier (6), ...
            for (int i = 1; i < 14; i++) {
                if (!has[(size_t)i]) continue;
                if (i == 6) {                                      // calling AE qualifier: form 2 is an integer
                    if (b.bit() || b.bit()) return false;
                    const size_t n = b.len();
                    calling = 0;
                    for (uint8_t c : b.octets(n)) calling = (calling << 8) | c;
                } else return false;
            }
        } else {
            for (int i = 1; i < 10; i++) if (has[(size_t)i]) return false;
        }
        hasUi = has[(size_t)nOpt - 1];
        add(r.detail, "  X.227 ACSE: association %s, application context %s%s%s%s%s", kind == 0 ? "request" : "response", ctx.c_str(), res.empty() ? "" : ", ", res.c_str(),
            calling >= 0 ? ", calling " : "", calling >= 0 ? appName(calling) : "");
        r.summary = kind == 0 ? "ACSE association request" : "ACSE association response";
        if (hasUi && !userInfo(b, ui)) return false;
        break;
    }
    default: return false;
    }
    if (b.err) return false;
    if (hasUi) {
        if (appPayload(ui, fromAir, r)) return true;
        add(r.detail, "  user information, %zu bits, not read", ui.d.size());
    }
    return true;
}

} // namespace

Vdl2AppResult vdl2AtnApp(const uint8_t* p, size_t n, bool fromAir) {
    Vdl2AppResult r;
    if (!n) return r;
    if (p[0] & 0x80) {
        static const Dict kSpdu[] = {{0xE8, "short connect (CN)"}, {0xF0, "short accept (AC)"}, {0xD8, "short accept continue"}, {0xE0, "short refuse (RF)"}, {0xA0, "short refuse continue"}, {0, nullptr}};
        const char* sn = (p[0] & 4) ? nullptr : dictOf(kSpdu, p[0] & 0xF8);
        if (!sn) return r;
        add(r.detail, "  X.225 session SPDU: %s%s", sn, (p[0] & 0xF8) == 0xE0 ? ((p[0] & 1) ? ", persistent refusal" : ", transient refusal") : "");
        r.summary = sn;
        if (n >= 2 && (p[1] & 3) == 2) {
            add(r.detail, "  X.226 presentation: ASN.1 unaligned PER");
            if (n >= 3) {
                Bits b(p + 2, n - 2);
                Vdl2AppResult t;
                if (acse(b, fromAir, t) && b.complete(b.d.size())) { r.detail += t.detail; r.summary += ", " + t.summary; r.decoded = t.decoded; }
            }
        }
        return r;
    }
    // no session header: Fully-encoded-data (presentation data), or a bare ACSE APDU
    {
        Bits b(p, n);
        const bool spare = b.bit();
        const bool tsn = b.bit();
        if (!tsn) {
            const bool ext = b.bit();
            if (!ext) {
                const long pc = b.range(1, 127);
                if (b.u(2) == 2) {
                    Bits pay;
                    if (b.bitString(pay) && b.complete(b.d.size())) {
                        (void)spare;
                        Vdl2AppResult t;
                        if (pc == 1) {
                            Bits a = pay;
                            if (acse(a, fromAir, t) && a.complete(pay.d.size())) { r.detail += t.detail; r.summary = t.summary; r.decoded = t.decoded; return r; }
                        } else if (appPayload(pay, fromAir, t)) { r.detail += t.detail; r.summary = t.summary; r.decoded = true; return r; }
                    }
                }
            }
        }
    }
    {
        Bits b(p, n);
        Vdl2AppResult t;
        if (acse(b, fromAir, t) && b.complete(b.d.size())) { r.detail += t.detail; r.summary = t.summary; r.decoded = t.decoded; }
    }
    return r;
}

} // namespace dect2
