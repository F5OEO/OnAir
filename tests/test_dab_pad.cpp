// DAB+ slide show: the PAD in the data stream element, X-PAD (variable and short, with and without content indicators), data groups, MOT segments.
// The generator's signal (dab_gen_pad.cpp, written from the standards without the receiver's code) goes through the PAD decoder directly, through
// DabAudio (Reed-Solomon, access units, libavcodec) and through the whole receiver; hand made PAD covers the other X-PAD forms and bad data.
#include "dect2/dab.h"
#include "dect2/dab_gen.h"
#include "dect2/dab_pad.h"
#include <cstdio>
#include <cstring>
#include <vector>
using namespace dect2;

static int fails = 0;
#define CHECK(c, ...) do { if (!(c)) { printf("FAIL line %d: ", __LINE__); printf(__VA_ARGS__); printf("\n"); fails++; } } while (0)

static void expectedImage(int& w, int& h, std::vector<uint8_t>& px) { dabgen::slideImage(w, h, px); }

static bool sameImage(const DabSlide& s) {
    int w, h; std::vector<uint8_t> px;
    expectedImage(w, h, px);
    return s.w == w && s.h == h && s.rgba == px;
}

static dabgen::TxConfig config() {
    dabgen::TxConfig tc;
    tc.services = dabgen::defaultServices();
    tc.services[2].slide = true;     // "OnAir Melody", 64 kbit/s
    tc.utcSeconds = 1700000000;
    return tc;
}

static void testDirect() {
    dabgen::Transmitter tx(config());
    const auto& aus = tx.accessUnits(2);
    CHECK(aus.size() >= 400, "access units: %zu", aus.size());
    DabPadDecoder pd;
    for (const auto& au : aus) pd.accessUnit(au.data(), (int)au.size());
    CHECK(pd.seq() == 1, "one picture expected, seq %llu", (unsigned long long)pd.seq());
    CHECK(sameImage(pd.slide()), "pixels differ");
    CHECK(pd.slide().name == "OnAir test slide", "name '%s'", pd.slide().name.c_str());
    const uint64_t s1 = pd.seq();
    for (const auto& au : aus) pd.accessUnit(au.data(), (int)au.size());   // the carousel: the same object again is not a new picture
    CHECK(pd.seq() == s1, "repeat counted as new");
    // the services without PAD
    DabPadDecoder q;
    for (const auto& au : tx.accessUnits(0)) q.accessUnit(au.data(), (int)au.size());
    CHECK(q.seq() == 0, "picture from a service without PAD");
}

// a MOT object as a data group, built here
static std::vector<uint8_t> group(int type, const std::vector<uint8_t>& seg, bool goodCrc = true) {
    std::vector<uint8_t> g = {(uint8_t)(0x40 | 0x20 | 0x10 | type), 0x00, 0x80, 0x00, 0x12, 0x00, 0x07, (uint8_t)(seg.size() >> 8), (uint8_t)seg.size()};
    g.insert(g.end(), seg.begin(), seg.end());
    const uint16_t c = dabgen::crc16(g.data(), (int)g.size());
    g.push_back((uint8_t)(c >> 8)); g.push_back((uint8_t)c ^ (goodCrc ? 0 : 1));
    return g;
}
static std::vector<uint8_t> motHeader(size_t bodySize) {
    const size_t hs = 7;
    return {(uint8_t)(bodySize >> 20), (uint8_t)(bodySize >> 12), (uint8_t)(bodySize >> 4), (uint8_t)((bodySize << 4) | (hs >> 9)), (uint8_t)(hs >> 1), (uint8_t)(((hs & 1) << 7) | 4), 3};
}
static void dgli(DabPadDecoder& pd, size_t len) {
    uint8_t l[4] = {(uint8_t)((len >> 8) & 0x3F), (uint8_t)len, 0, 0};
    const uint16_t c = dabgen::crc16(l, 2);
    l[2] = (uint8_t)(c >> 8); l[3] = (uint8_t)c;
    const uint8_t xp[6] = {0x01, 0x00, l[0], l[1], l[2], l[3]};   // CI: type 1, length index 0; end marker
    const uint8_t fp[2] = {0x20, 0x02};
    pd.pad(xp, 6, fp);
}
// the data group in pieces of `piece` bytes: `how` 0 = every piece with a content indicator, 1 = indicator on the first only (variable X-PAD),
// 2 = short X-PAD (3 bytes after an indicator, then 4 without)
static void send(DabPadDecoder& pd, const std::vector<uint8_t>& g, int how) {
    dgli(pd, g.size());
    size_t p = 0;
    bool first = true;
    while (p < g.size()) {
        if (how == 2) {
            const size_t n = first ? 3 : 4;
            std::vector<uint8_t> xp;
            if (first) xp.push_back(12);
            for (size_t i = 0; i < n; i++) xp.push_back(p + i < g.size() ? g[p + i] : 0);
            const uint8_t fp[2] = {0x10, (uint8_t)(first ? 0x02 : 0x00)};
            pd.pad(xp.data(), (int)xp.size(), fp);
            p += n;
        } else if (how == 1 && !first) {
            std::vector<uint8_t> xp(16, 0);
            for (size_t i = 0; i < 16 && p + i < g.size(); i++) xp[i] = g[p + i];
            const uint8_t fp[2] = {0x20, 0x00};
            pd.pad(xp.data(), 16, fp);
            p += 16;
        } else {
            std::vector<uint8_t> xp = {(uint8_t)((4 << 5) | (first ? 12 : 13)), 0x00};
            for (size_t i = 0; i < 16; i++) xp.push_back(p + i < g.size() ? g[p + i] : 0);
            const uint8_t fp[2] = {0x20, 0x02};
            pd.pad(xp.data(), 18, fp);
            p += 16;
        }
        first = false;
    }
}
static void testForms() {
    const std::vector<uint8_t> png = dabgen::slidePng();
    for (int how = 0; how < 3; how++) {
        DabPadDecoder pd;
        send(pd, group(3, motHeader(png.size())), how);
        send(pd, group(4, png), how);
        CHECK(pd.seq() == 1 && sameImage(pd.slide()), "X-PAD form %d: seq %llu", how, (unsigned long long)pd.seq());
    }
    {   // a bad CRC in the body: no picture; the good one after it: the picture
        DabPadDecoder pd;
        send(pd, group(3, motHeader(png.size())), 0);
        send(pd, group(4, png, false), 0);
        CHECK(pd.seq() == 0, "picture from a data group with a bad CRC");
        send(pd, group(4, png), 0);
        CHECK(pd.seq() == 1 && sameImage(pd.slide()), "recovery");
    }
    {   // a body of the wrong size, and not an image
        DabPadDecoder pd;
        send(pd, group(3, motHeader(png.size() + 1)), 0);
        send(pd, group(4, png), 0);
        CHECK(pd.seq() == 0, "body size not checked");
        pd.pad(nullptr, 0, std::vector<uint8_t>{0x20, 0x02}.data());
        const uint8_t junk[5] = {0x80, 0xFF, 0xFF, 0x00, 0x00};
        pd.accessUnit(junk, 5);
        const uint8_t junk2[4] = {0x80, 0x02, 0x20, 0x02};
        pd.accessUnit(junk2, 4);
    }
}

static void testAudio() {
    dabgen::Transmitter tx(config());
    DabAudio au;
    au.setSilent(true);
    au.select(3, true, 64);
    const auto& lay = tx.layout();
    int svc = -1;
    for (size_t i = 0; i < lay.size(); i++) if (lay[i].subId == 3) svc = (int)i;
    CHECK(svc >= 0, "sub-channel 3");
    std::vector<uint8_t> f;
    for (int64_t k = 0; k < 600; k++) { tx.logicalFrame(svc, k, f); au.push(f.data(), (int)f.size()); }
    DabSlide s;
    const bool got = au.slide(s, 0);
    CHECK(got && s.seq > 0 && sameImage(s), "DabAudio: slide %d", (int)got);
    const uint64_t seq1 = s.seq;
    CHECK(!au.slide(s, seq1), "no new picture expected");
    const DabAudioStats st = au.stats();
    CHECK(st.auBad == 0 && st.superframesBad == 0 && st.pcmFrames > 0, "audio with PAD: %llu bad, %llu pcm frames", (unsigned long long)st.auBad, (unsigned long long)st.pcmFrames);
    au.select(3, true, 64);
    CHECK(au.slide(s, seq1) && s.seq > seq1 && s.w == 0 && s.rgba.empty(), "slide kept after select()");
}

int main() {
    testDirect();
    testForms();
    testAudio();
    if (fails) { printf("%d FAILED\n", fails); return 1; }
    printf("test_dab_pad: all passed\n");
    return 0;
}
