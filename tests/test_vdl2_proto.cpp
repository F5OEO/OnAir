// VDL Mode 2 layers without a radio signal: header code, scrambler, Reed-Solomon code with shortened blocks and erasures, interleaving,
// HDLC framing, AVLC addresses, and the decoders of what the frames carry (ACARS with ADS-C, X.25 / CLNP / COTP, XID).
#include "dect2/vdl2_proto.h"
#include "dect2/aero_adsc.h"
#include "dect2/vdl2_app.h"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <random>
using namespace dect2;
static int fails = 0;
#define CHECK(c, ...) do { if (!(c)) { printf("FAIL line %d: ", __LINE__); printf(__VA_ARGS__); printf("\n"); fails++; } } while (0)

static bool bodyRoundTrip(const std::vector<uint8_t>& data, int flips, std::mt19937& rng, int& fixedOut) {
    std::vector<uint8_t> b = vdl2BurstBits(data);
    // errors in whole octets of the body, at most one octet per block per flip round
    for (int f = 0; f < flips; f++) {
        const size_t pos = kVdl2HeaderBits + 8 * (rng() % ((b.size() - kVdl2HeaderBits) / 8));
        b[pos + rng() % 8] ^= 1;
    }
    vdl2Scramble(b, 0);
    uint32_t w = 0, len = 0;
    bool corr = false;
    for (int i = 0; i < kVdl2HeaderBits; i++) w = (w << 1) | b[(size_t)i];
    Vdl2Layout l;
    if (!vdl2HeaderDecode(w, len, corr) || len != data.size() || !vdl2Layout(len, l)) return false;
    std::vector<uint8_t> out;
    if (!vdl2BurstData(b.data() + kVdl2HeaderBits, l, len, out, fixedOut)) return false;
    return out == data;
}

int main() {
    std::mt19937 rng(5);
    // header: every single-bit error in the 22 protected bits is corrected
    for (uint32_t len : {1u, 77u, 800u, 1992u, 8191u, 12000u}) {
        const uint32_t w = vdl2HeaderWord(len);
        uint32_t got = 0; bool corr = true;
        CHECK(vdl2HeaderDecode(w, got, corr) && got == len && !corr, "header %u", len);
        for (int b = 0; b < 22; b++) {
            const bool ok = vdl2HeaderDecode(w ^ (1u << b), got, corr);
            if (len <= 0x1FFF) CHECK(ok && got == len && corr, "header %u bit %d -> %u", len, b, got);
        }
        CHECK(vdl2HeaderDecode(w | (7u << 22), got, corr) && got == len, "reserved bits are ignored");
    }
    // layout rules
    Vdl2Layout l;
    CHECK(!vdl2Layout(16, l), "two octets carry no FEC: not a burst");
    CHECK(vdl2Layout(8 * 30, l) && l.lastFec == 2 && l.fecOct == 2, "30 octets: 2 FEC");
    CHECK(vdl2Layout(8 * 31, l) && l.lastFec == 4, "31 octets: 4 FEC");
    CHECK(vdl2Layout(8 * 68, l) && l.lastFec == 6, "68 octets: 6 FEC");
    CHECK(vdl2Layout(8 * 249, l) && l.blocks == 1 && l.fecOct == 6 && l.lastLen == 249, "one full block");
    CHECK(vdl2Layout(8 * 300, l) && l.blocks == 2 && l.fecOct == 10 && l.lastLen == 51, "300 octets");
    // Reed-Solomon: up to three errors in a full block, one error with four erasures
    {
        uint8_t d[255];
        for (int t = 0; t < 200; t++) {
            for (int i = 0; i < 249; i++) d[i] = (uint8_t)rng();
            vdl2RsEncode(d, d + 249);
            uint8_t c[255];
            memcpy(c, d, 255);
            const int ne = t % 4;
            for (int e = 0; e < ne; e++) c[(e * 83 + t) % 255] ^= (uint8_t)(1 + rng() % 255);
            const int f = vdl2RsDecode(c, 6);
            CHECK(f == ne && !memcmp(c, d, 255), "RS %d errors: %d", ne, f);
        }
        for (int t = 0; t < 100; t++) {
            for (int i = 0; i < 249; i++) d[i] = i < 20 ? (uint8_t)rng() : 0;
            vdl2RsEncode(d, d + 249);
            uint8_t c[255];
            memcpy(c, d, 255);
            for (int i = 251; i < 255; i++) c[i] = 0;              // only two parity octets sent
            if (t % 2) c[t % 20] ^= 0x5A;
            const int f = vdl2RsDecode(c, 2);
            CHECK(f == t % 2 && !memcmp(c, d, 251), "shortened RS: %d", f);
        }
        for (int i = 0; i < 249; i++) d[i] = (uint8_t)rng();
        vdl2RsEncode(d, d + 249);
        for (int e = 0; e < 4; e++) d[e * 50] ^= 0x33;
        CHECK(vdl2RsDecode(d, 6) < 0, "four errors are refused");
    }
    // burst body: interleaving and FEC for many lengths, with errors spread over the blocks
    int fixed = 0;
    for (int n : {17, 23, 24, 100, 250, 600, 1500, 1992, 4000, 8000}) {
        std::vector<uint8_t> data((size_t)n);
        for (auto& v : data) v = (uint8_t)(rng() & 1);
        CHECK(bodyRoundTrip(data, 0, rng, fixed) && fixed == 0, "body %d bits", n);
        if (n >= 8 * 68) CHECK(bodyRoundTrip(data, 1, rng, fixed) && fixed == 1, "body %d bits, one error", n);
    }
    // HDLC: stuffing of long runs of ones, two frames between shared flags
    {
        std::vector<uint8_t> a = {0xFF, 0xFF, 0x7E, 0x7E, 0x3F, 0x01}, b = {0x55, 0xFE, 0xFF, 0xFF, 0xFF, 0x00, 0x12};
        const auto bits = vdl2HdlcBits({a, b});
        const auto fr = vdl2HdlcFrames(bits);
        CHECK(fr.size() == 2 && fr[0] == a && fr[1] == b, "hdlc %zu frames", fr.size());
    }
    // AVLC addresses
    for (uint32_t a : {0x000001u, 0x4CA2B1u, 0xFFFFFFu, 0x10A123u})
        for (int ty : {1, 4, 5, 7}) {
            uint8_t o[4];
            vdl2PutAddr(o, {a, ty, true}, true);
            const Vdl2Addr r = vdl2GetAddr(o);
            CHECK(r.addr == a && r.type == ty && r.status && (o[3] & 1), "address %06X type %d", a, ty);
        }
    // ACARS with ADS-C over AVLC
    {
        std::vector<uint8_t> g = aeroAdscBasicGroup(51.5, -0.4, 36000, 1234.5);
        const auto f = aeroAdscFlightIdGroup("BA0117");
        g.insert(g.end(), f.begin(), f.end());
        const std::string text = "M01ABA0117" + aeroBuildAdscText("LHRCAYA", "G-EUUA", g);
        const auto fr = vdl2Frame({0x10A000, 4, false}, {0x4CA2B1, 1, false}, vdl2ControlI(2, 5, false), vdl2AcarsInfo("G-EUUA", "B6", '3', text));
        CHECK(vdl2FcsOk(fr.data(), fr.size()), "fcs");
        Vdl2Message m;
        CHECK(vdl2ParseFrame(fr.data(), fr.size() - 2, m), "parse");
        printf("  ACARS: %s\n", m.summary.c_str());
        CHECK(m.src == 0x4CA2B1 && m.fromAir && m.dst == 0x10A000 && m.dstType == 4 && m.frameKind == 'I' && m.ns == 2 && m.nr == 5, "AVLC fields");
        CHECK(m.acars && m.acarsCrcOk && m.reg == "G-EUUA" && m.label == "B6" && m.flight == "BA0117", "ACARS fields %s %s %s", m.reg.c_str(), m.label.c_str(), m.flight.c_str());
        CHECK(m.hasPos && std::fabs(m.lat - 51.5) < 0.01 && std::fabs(m.lon + 0.4) < 0.01 && m.hasAlt && std::abs(m.altFt - 36000) < 10, "ADS-C position %.3f %.3f %d", m.lat, m.lon, m.altFt);
    }
    // X.25 data with CLNP and COTP
    {
        std::vector<uint8_t> src(20, 0x11), dst(20, 0x22), user = {1, 2, 3, 4, 5};
        const auto fr = vdl2Frame({0x10A000, 4, false}, {0x4CA2B1, 1, false}, vdl2ControlI(0, 0, false), vdl2X25ClnpData(1, 3, 4, src, dst, user));
        Vdl2Message m;
        CHECK(vdl2ParseFrame(fr.data(), fr.size() - 2, m), "parse x25");
        printf("  X.25: %s\n", m.summary.c_str());
        CHECK(m.proto == "COTP" && m.summary.find("P(S) 3 P(R) 4") != std::string::npos && m.detail.find("CLNP: DT") != std::string::npos && m.detail.find("COTP") != std::string::npos, "%s", m.detail.c_str());
        const auto cr = vdl2Frame({0x10A000, 4, false}, {0x4CA2B1, 1, false}, vdl2ControlI(1, 0, false), vdl2X25CallRequest(1, "1234", "56789"));
        Vdl2Message c;
        CHECK(vdl2ParseFrame(cr.data(), cr.size() - 2, c) && c.summary.find("call request") != std::string::npos && c.detail.find("called DTE 1234, calling DTE 56789") != std::string::npos, "%s", c.detail.c_str());
    }
    // XID: GSIF with the ground station's location, an aircraft's link establishment
    {
        const auto fr = vdl2Frame({0xFFFFFF, 7, false}, {0x10A000, 4, false}, kVdl2CtrlXID, vdl2XidGsif(51.4, -0.5, "EGLL EGKK"));
        Vdl2Message m;
        CHECK(vdl2ParseFrame(fr.data(), fr.size() - 2, m) && m.frameName == "GSIF" && m.hasPos && std::fabs(m.lat - 51.4) < 0.01 && std::fabs(m.lon + 0.5) < 0.01, "GSIF %s %.2f %.2f", m.frameName.c_str(), m.lat, m.lon);
        CHECK(m.detail.find("airport coverage EGLL EGKK") != std::string::npos && m.fromGround, "%s", m.detail.c_str());
        const auto le = vdl2Frame({0x10A000, 4, false}, {0x4CA2B1, 1, false}, kVdl2CtrlXID | 0x10, vdl2XidAircraftLe(-33.9, 151.2, 37000, "YSSY"));
        Vdl2Message a;
        CHECK(vdl2ParseFrame(le.data(), le.size() - 2, a) && a.frameName == "XID_CMD_LE" && a.hasPos && std::fabs(a.lat + 33.9) < 0.01 && std::fabs(a.lon - 151.2) < 0.01 && a.altFt == 37000, "LE %s", a.frameName.c_str());
        printf("  XID: %s / %s\n", m.summary.c_str(), a.summary.c_str());
    }
    // the layers dumpvdl2 names: compressed CLNP, ES-IS, IDRP, COTP connect, SNDCF, X.25 clear, pieces joined
    {
        const Vdl2Addr gs{0x10A000, 4, false}, ac{0x4CA2B1, 1, false};
        auto parse = [&](const std::vector<uint8_t>& info, Vdl2Reasm* rs, double t, Vdl2Message& m, bool air = true) {
            const auto fr = vdl2Frame(air ? gs : ac, air ? ac : gs, vdl2ControlI(0, 0, false), info);
            return vdl2ParseFrame(fr.data(), fr.size() - 2, m, rs, t);
        };
        auto x25 = [](int ps, bool more, const std::vector<uint8_t>& user) {
            std::vector<uint8_t> o = {0x10, 0x01, (uint8_t)(((ps & 7) << 1) | (more ? 0x10 : 0))};
            o.insert(o.end(), user.begin(), user.end());
            return o;
        };
        // compressed header (type 0, no segmentation) carrying a COTP CR with a TPDU size and a transport selector
        const std::vector<uint8_t> cr = {13, 0xE0, 0x00, 0x00, 0x00, 0x01, 0x40, 0xC0, 0x01, 0x0B, 0xC2, 0x02, 0x00, 0x01};
        std::vector<uint8_t> comp = {0x01, 120, 0x00, 0x05};   // type 0, priority 1, lifetime 60 s, local reference 5
        comp[0] = 0x01;
        comp.insert(comp.end(), cr.begin(), cr.end());
        Vdl2Message m;
        CHECK(parse(x25(0, false, comp), nullptr, 0, m), "parse compressed");
        CHECK(m.detail.find("compressed header") != std::string::npos && m.detail.find("local reference 5") != std::string::npos && m.detail.find("CR (connection request)") != std::string::npos &&
              m.detail.find("TPDU size 2048 octets") != std::string::npos && m.detail.find("called / responding transport selector 0001") != std::string::npos, "%s", m.detail.c_str());
        // CPDLC from the air, octets as received on 136.775 / 136.825 MHz and read the same by dumpvdl2
        {
            auto hex = [](const char* h) { std::vector<uint8_t> v; for (size_t i = 0; h[i]; i += 2) v.push_back((uint8_t)strtol(std::string(h + i, 2).c_str(), nullptr, 16)); return v; };
            const Vdl2AppResult w = vdl2AtnApp(hex("00A7264F31443D2A88EE00008286024F54").data(), 17, true);
            CHECK(w.decoded && w.summary == "CPDLC WILCO" && w.detail.find("id 5, reference 4, 2026-10-11 17:07:28, logical acknowledgement required") != std::string::npos, "%s", w.detail.c_str());
            const Vdl2AppResult l = vdl2AtnApp(hex("00A7164EE2493D2A88EB832103CB921EA8").data(), 17, true);
            CHECK(l.decoded && l.summary == "CPDLC LOGICAL ACKNOWLEDGEMENT" && l.detail.find("id 9, reference 9, 2026-10-11 17:07:23") != std::string::npos, "%s", l.detail.c_str());
            const Vdl2AppResult c = vdl2AtnApp(hex("00A6C64D900F4AA23D006320BB33D298").data(), 16, true);
            CHECK(c.decoded && c.summary == "CPDLC CURRENT DATA AUTHORITY" && c.detail.find("id 0, 2026-10-11 17:07:40") != std::string::npos, "%s", c.detail.c_str());
            const Vdl2AppResult r = vdl2AtnApp(hex("002808C36004E2C9DC5127A5511E700020A05C197B").data(), 21, true);
            CHECK(r.decoded && r.detail.find("release response, reason normal") != std::string::npos && r.detail.find("DM0 WILCO") != std::string::npos, "%s", r.detail.c_str());
            // octets cut short are not read
            CHECK(!vdl2AtnApp(hex("00A7264F31443D2A88EE0000828602").data(), 15, true).decoded, "cut short");
        }
        // ES-IS hello with the mobile subnetwork capabilities
        std::vector<uint8_t> ish = {0x82, 0, 1, 0, 4, 0, 30, 0, 0, 3, 0x47, 0x00, 0x27, 0x81, 0x01, 0x03};
        ish[1] = (uint8_t)ish.size();
        Vdl2Message e;
        CHECK(parse(x25(1, false, ish), nullptr, 0, e) && e.proto == "ES-IS" && e.summary.find("IS hello") != std::string::npos && e.detail.find("hold time 30 s") != std::string::npos &&
              e.detail.find("ATS, AOC") != std::string::npos, "%s", e.detail.c_str());
        // IDRP keepalive
        std::vector<uint8_t> idrp(30, 0);
        idrp[0] = 0x85; idrp[2] = 30; idrp[3] = 4; idrp[7] = 9;
        Vdl2Message k;
        CHECK(parse(x25(2, false, idrp), nullptr, 0, k) && k.proto == "IDRP" && k.summary.find("KEEPALIVE") != std::string::npos && k.detail.find("seq 9") != std::string::npos, "%s", k.detail.c_str());
        // X.25 call request with the SNDCF, clear request with its cause
        Vdl2Message q;
        CHECK(parse(vdl2X25CallRequest(1, "1234", "56789"), nullptr, 0, q) && q.detail.find("ICAO SNDCF: version 1, compression none") != std::string::npos, "%s", q.detail.c_str());
        Vdl2Message cl;
        CHECK(parse({0x10, 0x01, 0x13, 0x09, 0xF1}, nullptr, 0, cl) && cl.detail.find("remote procedure error") != std::string::npos && cl.detail.find("higher level initiated disconnect (normal)") != std::string::npos, "%s", cl.detail.c_str());
        // one CLNP DT cut in two X.25 packets (M bit): the first shows nothing decoded, the second the whole
        std::vector<uint8_t> src(20, 0x11), dst(20, 0x22), user(40);
        for (size_t i = 0; i < user.size(); i++) user[i] = (uint8_t)i;
        const auto whole = vdl2X25ClnpData(1, 0, 0, src, dst, user);
        const std::vector<uint8_t> body(whole.begin() + 3, whole.end());
        const size_t cut = body.size() / 2;
        Vdl2Reasm rs;
        Vdl2Message p1, p2;
        CHECK(parse(x25(3, true, std::vector<uint8_t>(body.begin(), body.begin() + (long)cut)), &rs, 1.0, p1) && p1.detail.find("piece 1 of a longer message") != std::string::npos && p1.proto == "X.25", "%s", p1.detail.c_str());
        CHECK(parse(x25(4, false, std::vector<uint8_t>(body.begin() + (long)cut, body.end())), &rs, 1.5, p2) && p2.detail.find("joined from 2 packets") != std::string::npos &&
              p2.detail.find("CLNP: DT") != std::string::npos && p2.detail.find("ATN application data, 40 octets") != std::string::npos, "%s", p2.detail.c_str());
        // the same pieces much later do not join
        Vdl2Message p3, p4;
        CHECK(parse(x25(3, true, std::vector<uint8_t>(body.begin(), body.begin() + (long)cut)), &rs, 10.0, p3), "p3");
        CHECK(parse(x25(4, false, std::vector<uint8_t>(body.begin() + (long)cut, body.end())), &rs, 400.0, p4) && p4.detail.find("joined") == std::string::npos, "%s", p4.detail.c_str());
        // CLNP segments (compressed header, derived type 7 then 6) joined by offset
        std::vector<uint8_t> payload = cr;
        auto seg = [&](int type, uint16_t off, size_t from, size_t to) {
            std::vector<uint8_t> o = {(uint8_t)((type << 4) | 1), 120, 0x00, 0x05, 0x00, 0x42, (uint8_t)(off >> 8), (uint8_t)off, 0x00, (uint8_t)payload.size()};
            o.insert(o.end(), payload.begin() + (long)from, payload.begin() + (long)to);
            return o;
        };
        Vdl2Reasm r2;
        Vdl2Message s1, s2;
        CHECK(parse(x25(0, false, seg(7, 0, 0, 6)), &r2, 2.0, s1) && s1.detail.find("waiting for the rest") != std::string::npos, "%s", s1.detail.c_str());
        CHECK(parse(x25(1, false, seg(6, 6, 6, payload.size())), &r2, 2.2, s2) && s2.detail.find("joined from 2 segments") != std::string::npos && s2.detail.find("CR (connection request)") != std::string::npos, "%s", s2.detail.c_str());
        printf("  layers: %s | %s | %s\n", m.summary.c_str(), e.summary.c_str(), k.summary.c_str());
    }
    printf(fails ? "vdl2 proto: %d FAILED\n" : "vdl2 proto: all passed\n", fails);
    return fails ? 1 : 0;
}
