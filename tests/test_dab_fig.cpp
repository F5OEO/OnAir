// FIC analysis: the FIG decoders on hand-made FIGs (the layouts of EN 300 401 clause 8), then the receiver against the test transmitter: the
// analysis holds the FIGs the transmitter sends, counted, and they decode to the ensemble id, the sub-channels and the labels it was given.
#include "dect2/dab.h"
#include "dect2/dab_fig.h"
#include "dect2/dab_announce.h"
#include "dect2/dab_gen.h"
#include "jobs.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>
using namespace dect2;
using testjobs::jprintf;

static int fails = 0;
#define CHECK(c, ...) do { if (!(c)) { jprintf("FAIL line %d: ", __LINE__); jprintf(__VA_ARGS__); jprintf("\n"); fails++; } } while (0)

static DabFigRecord rec(int type, int ext, int cn, int oe, int pd, std::vector<uint8_t> data) {
    DabFigRecord r;
    r.type = type; r.ext = ext; r.cn = cn; r.oe = oe; r.pd = pd; r.data = std::move(data); r.count = 1;
    return r;
}

static std::string pf4(uint32_t sid) { char b[32]; snprintf(b, sizeof b, "Service ID=0x%04X", sid); return b; }

static bool has(const DabFigDecoded& d, const std::string& s) {
    for (const auto& l : d.lines) if (l.text.find(s) != std::string::npos) return true;
    return false;
}

static void dump(const DabFigDecoded& d) {
    for (const auto& l : d.lines) jprintf("   %*s%s\n", 2 * l.level, "", l.text.c_str());
    for (const auto& e : d.errors) jprintf("   ERROR %s\n", e.c_str());
}

static void testDecoders() {
    DabFigContext ctx;
    {   // FIG 0/0: EId 0xCE15, change flag 0, CIF count 5/200
        const auto d = dabFigDecode(rec(0, 0, 0, 0, 0, {0x00, 0xCE, 0x15, 0x05, 0x00 | 0x00, 200}), ctx);
        CHECK(has(d, "Ensemble ID=0xCE15"), "0/0 ensemble id");
        CHECK(has(d, "Country ID=12"), "0/0 country");
        CHECK(has(d, "CIF Count=5/0"), "0/0 cif count %s", d.lines.size() > 5 ? d.lines[5].text.c_str() : "?");
    }
    {   // FIG 0/1: sub-channel 3, start 100, long form EEP 3-A, 36 CUs
        const auto d = dabFigDecode(rec(0, 1, 0, 0, 0, {0x01, (3 << 2) | 0, 100, 0x80 | (0 << 4) | (2 << 2) | 0, 36}), ctx);
        CHECK(has(d, "Subch=0x3") && has(d, "start_addr=100") && has(d, "EEP=3-A") && has(d, "subch size=36"), "0/1 long form");
        CHECK(d.errors.empty(), "0/1 errors");
    }
    {   // FIG 0/2: programme service 0xCE01, one audio component (DAB+, sub-channel 3, primary)
        const auto d = dabFigDecode(rec(0, 2, 0, 0, 0, {0x02, 0xCE, 0x01, 0x01, (63), (3 << 2) | 0}), ctx);
        CHECK(has(d, "Service ID=0xCE01") && has(d, "ASCTy=AAC sound (63)") && has(d, "SubChannel ID=0x03") && has(d, "primary=true"), "0/2");
    }
    {   // FIG 0/9 with an ensemble LTO of +1:00 (2 half hours) and ECC 0xE1, international table 1
        const auto d = dabFigDecode(rec(0, 9, 0, 0, 0, {0x09, 0x00 | 2, 0xE1, 0x01}), ctx);
        CHECK(has(d, "Ensemble LTO=0x2 UTC+1:00") && has(d, "Ensemble ECC=0xE1"), "0/9");
    }
    {   // FIG 0/10: MJD 60000 (2023-02-25), 12:34 UTC long form with seconds 56
        const uint32_t mjd = 60000;
        const auto d = dabFigDecode(rec(0, 10, 0, 0, 0, {0x0A, (uint8_t)((mjd >> 10) & 0x7F), (uint8_t)((mjd >> 2) & 0xFF),
                                                           (uint8_t)(((mjd & 3) << 6) | 0x08 | (12 >> 2)), (uint8_t)(((12 & 3) << 6) | 34), (uint8_t)(56 << 2), 0}), ctx);
        CHECK(has(d, "Sat Feb 25 2023") && has(d, "UTC Time=12:34:56.0"), "0/10 %s", d.lines.size() > 1 ? d.lines[1].text.c_str() : "?");
    }
    {   // FIG 0/17: programme type Rock Music (11) of service 0xCE01 with a language (English 9)
        const auto d = dabFigDecode(rec(0, 17, 0, 0, 0, {0x11, 0xCE, 0x01, 0x80 | 0x20, 9, 11}), ctx);
        CHECK(has(d, "Language=0x9 English") && has(d, "Rock Music"), "0/17");
    }
    {   // FIG 0/21: region 0, one DAB ensemble 0xCE15 at 225.648 MHz (225648 kHz = 16 * 14103)
        const uint32_t f16 = 225648 / 16;
        const auto d = dabFigDecode(rec(0, 21, 0, 0, 0, {0x15, 0x00, 6 + 0, 0xCE, 0x15, 0x03, (uint8_t)((f16 >> 16) & 7), (uint8_t)(f16 >> 8), (uint8_t)f16}), ctx);
        CHECK(has(d, "225648 kHz") && has(d, "ID field=0xCE15 EId"), "0/21");
    }
    {   // FIG 1/1: the service 0xCE01 labelled "Test Radio" (EBU Latin), short label "Test" from the mask 0xF000
        std::vector<uint8_t> v = {0x01, 0xCE, 0x01};
        const char* t = "Test Radio      ";
        v.insert(v.end(), t, t + 16);
        v.push_back(0xF0); v.push_back(0x00);
        const auto d = dabFigDecode(rec(1, 1, 0, 0, 0, v), ctx);
        CHECK(has(d, "Label=\"Test Radio\"") && has(d, "Short label=\"Test\"") && has(d, "Service ID=0xCE01"), "1/1");
        v[0] = 0x00;   // FIG 1/0
        CHECK(has(dabFigDecode(rec(1, 0, 0, 0, 0, v), ctx), "Ensemble ID=0xCE01"), "1/0");
    }
    {   // FIG 2/1 in two segments (UTF-8): "Hello " and "World", the label is put together from both
        std::vector<DabFigRecord> all;
        std::vector<uint8_t> s0 = {(uint8_t)(0x01 | (0 << 4) | 0x08 * 0), 0xCE, 0x01, (uint8_t)((0 << 7) | (1 << 4)), 0, 0, 'H', 'e', 'l', 'l', 'o', ' '};
        std::vector<uint8_t> s1 = {(uint8_t)(0x01 | (1 << 4)), 0xCE, 0x01, 'W', 'o', 'r', 'l', 'd'};
        all.push_back(rec(2, 1, 0, 0, 0, s0));
        all.push_back(rec(2, 1, 0, 0, 0, s1));
        DabFigContext c2;
        c2.all = &all;
        const auto d = dabFigDecode(all[1], c2);
        CHECK(has(d, "Label=\"Hello World\""), "2/1 segments");
        if (!has(d, "Label=\"Hello World\"")) dump(d);
    }
    {   // a truncated FIG and one with no decoder never crash, and the unknown one shows its bytes
        const auto a = dabFigDecode(rec(0, 2, 0, 0, 0, {0x02, 0xCE}), ctx);
        CHECK(!a.errors.empty(), "truncated 0/2 is reported");
        const auto b = dabFigDecode(rec(0, 29, 0, 0, 0, {0x1D, 1, 2, 3}), ctx);
        CHECK(has(b, "data: 1D 01 02 03"), "unknown FIG as hex");
        for (int t = 0; t < 7; t++)
            for (int e = 0; e < 32; e++)
                for (int len = 1; len < 31; len++) {   // any bytes: nothing outside the buffer is read
                    std::vector<uint8_t> v((size_t)len);
                    for (int i = 0; i < len; i++) v[(size_t)i] = (uint8_t)(i * 37 + e * 11 + t * 5 + len);
                    int ext = 0;
                    if (t == 0) { ext = e; v[0] = (uint8_t)((v[0] & 0xE0) | e); }
                    else if (t == 1 || t == 2 || t == 5) { ext = e & 7; v[0] = (uint8_t)((v[0] & 0xF8) | ext); }
                    const int oe = t == 0 ? (v[0] >> 6) & 1 : t == 1 ? (v[0] >> 3) & 1 : 0;
                    (void)dabFigDecode(rec(t, ext, t == 0 ? v[0] >> 7 : 0, oe, t == 0 ? (v[0] >> 5) & 1 : 0, v), ctx);
                }
    }
}

static void testInfo() {
    DabFicAnalysis a;
    a.figs.push_back(rec(0, 9, 0, 0, 0, {0x09, 0x02, 0xE1, 0x01}));                       // LTO +1:00, ECC E1, table 1
    a.figs.push_back(rec(0, 17, 0, 0, 0, {0x11, 0xCE, 0x01, 0xA0, 9, 11}));              // service CE01: English, Rock Music
    a.figs.push_back(rec(0, 18, 0, 0, 0, {0x12, 0xCE, 0x01, 0x00, 0x03, 0x00}));          // service CE01 supports Alarm and Road Traffic
    a.figs.push_back(rec(0, 5, 0, 0, 0, {0x05, 0x03, 15}));                               // sub-channel 3: French
    const uint32_t f16 = 225648 / 16;
    a.figs.push_back(rec(0, 21, 0, 0, 0, {0x15, 0x00, 6, 0xCE, 0x15, 0x03, (uint8_t)((f16 >> 16) & 7), (uint8_t)(f16 >> 8), (uint8_t)f16}));
    a.figs.push_back(rec(0, 21, 0, 0, 0, {0x15, 0x00, 4, 0x20, 0x01, 0x82, 0x0A}));        // FM with RDS 0x2001: 87.5 + 10 * 0.1 = 88.5 MHz  (rm 8, one frequency)
    const DabFicInfo i = dabFicInfo(a);
    CHECK(i.hasEcc && i.ecc == 0xE1 && i.hasLto && i.ltoHalfHours == 2, "0/9 info");
    CHECK(i.pty.count(0xCE01) && i.pty.at(0xCE01) == 11 && i.language.at(0xCE01) == 9, "0/17 info");
    CHECK(i.announce.at(0xCE01) == 0x0003, "0/18 info");
    CHECK(i.subLanguage.at(3) == 15, "0/5 info");
    CHECK(i.freqs.size() == 2 && i.freqs[0].id == 0xCE15 && i.freqs[0].kHz.size() == 1 && std::fabs(i.freqs[0].kHz[0] - 225648) < 1, "0/21 DAB");
    CHECK(i.freqs.size() == 2 && i.freqs[1].rm == 8 && i.freqs[1].kHz.size() == 1 && std::fabs(i.freqs[1].kHz[0] - 88500) < 1, "0/21 FM");
}

static void testReceiver() {
    dabgen::TxConfig tc;
    tc.utcSeconds = 1700000000;
    SynthConfig sc;
    const double rate = 2.048e6;
    auto syn = makeDabSynth(tc, sc, rate);
    CHECK(syn != nullptr, "no generator");
    if (!syn) return;
    DabReceiver rx;
    rx.configure(rate);
    rx.audio().setSilent(true);
    std::vector<cf32> x(65536);
    const size_t total = (size_t)(6.0 * rate);
    for (size_t done = 0; done < total; done += x.size()) {
        const size_t n = std::min(x.size(), total - done);
        syn->generate(x.data(), n);
        rx.feed(x.data(), n);
    }
    const DabFicAnalysis fic = rx.ficAnalysis();
    CHECK(fic.frames > 40, "only %llu frames", (unsigned long long)fic.frames);
    CHECK(fic.fibOk > 100 && fic.fibBad == 0, "FIBs %llu ok %llu bad", (unsigned long long)fic.fibOk, (unsigned long long)fic.fibBad);
    CHECK(fic.fibCount == fic.fibOk, "fill is counted over the good FIBs");
    CHECK(fic.fillBytes > 0, "fill bytes");
    const DabFigContext ctx = dabFigContext(fic);
    const DabEnsemble ens = rx.ensemble();
    int n00 = 0, n01 = 0, n02 = 0, n10 = 0, n11 = 0, n010 = 0;
    std::string labels;
    bool eid = false;
    size_t subs = 0;
    for (const DabFigRecord& r : fic.figs) {
        const DabFigDecoded d = dabFigDecode(r, ctx);
        if (!d.errors.empty()) { CHECK(false, "decode errors in %s", dabFigHeader(r).c_str()); dump(d); }
        CHECK(r.count >= 1 && r.lastFrame >= r.firstFrame && r.lastFib >= 0 && r.lastFib < 12, "record bookkeeping");
        if (r.type == 0 && r.ext == 0) { n00++; eid |= has(d, "Ensemble ID=0xCE15"); }
        if (r.type == 0 && r.ext == 1) { n01++; for (const auto& l : d.lines) if (l.text.rfind("Subch=", 0) == 0) subs++; }
        if (r.type == 0 && r.ext == 2) n02++;
        if (r.type == 0 && r.ext == 10) n010++;
        if (r.type == 1 && r.ext == 0) { n10++; CHECK(has(d, "Label=\"" + ens.label + "\""), "1/0 label vs ensemble '%s'", ens.label.c_str()); }
        if (r.type == 1 && r.ext == 1) {
            n11++;
            for (const auto& kv : ens.services) if (has(d, pf4(kv.first))) labels += kv.second.label + ";";
        }
    }
    CHECK(n00 == 1, "FIG 0/0 keeps one record (%d)", n00);
    CHECK(n010 == 1, "FIG 0/10 keeps one record (%d)", n010);
    CHECK(eid, "FIG 0/0 decodes to the ensemble id");
    CHECK(n01 >= 1 && subs == ens.subs.size(), "0/1 describes %zu of %zu sub-channels", subs, ens.subs.size());
    CHECK(n02 >= 1, "FIG 0/2 present");
    CHECK(n10 == 1, "FIG 1/0 present");
    CHECK(n11 == (int)ens.services.size(), "one FIG 1/1 per service (%d of %zu)", n11, ens.services.size());
    CHECK(!ens.services.empty(), "no services");
    const std::string text = dabFicText(fic);
    CHECK(text.find("FIG 0/0  Ensemble information") != std::string::npos && text.find("Ensemble ID=0xCE15") != std::string::npos, "text dump");
    uint64_t total00 = 0;
    for (const DabFicStat& s : fic.stats) if (s.type == 0 && s.ext == 0) total00 = s.count;
    CHECK(total00 + 2 >= fic.frames, "0/0 seen %llu times in %llu frames", (unsigned long long)total00, (unsigned long long)fic.frames);
    // a reset empties the analysis
    rx.reset();
    CHECK(rx.ficAnalysis().figs.empty(), "reset clears the FIGs");
    jprintf("receiver: %zu distinct FIGs, %llu frames, %.0f %% of the FIC used\n", fic.figs.size(), (unsigned long long)fic.frames,
            100.0 * (double)fic.fillBytes / (30.0 * (double)fic.fibCount));
}

// FIG 0/17, 0/18 and 0/19 sent by the test transmitter, read back by the receiver
static void testAnnounce() {
    dabgen::TxConfig tc;
    tc.utcSeconds = 1700000000;
    tc.annCluster = 3; tc.annSupport = 0x0003; tc.annActive = 0x0002; tc.annSubId = 2;
    tc.pty = 11; tc.ptyDynamic = true;
    SynthConfig sc;
    const double rate = 2.048e6;
    auto syn = makeDabSynth(tc, sc, rate);
    if (!syn) { CHECK(false, "no generator"); return; }
    DabReceiver rx;
    rx.configure(rate);
    rx.audio().setSilent(true);
    std::vector<cf32> x(65536);
    const size_t total = (size_t)(5.0 * rate);
    for (size_t done = 0; done < total; done += x.size()) {
        const size_t n = std::min(x.size(), total - done);
        syn->generate(x.data(), n);
        rx.feed(x.data(), n);
    }
    DabEnsemble e = rx.ensemble();
    CHECK(!e.services.empty(), "no services");
    int withPty = 0;
    for (const auto& kv : e.services) if (kv.second.pty == 11 && kv.second.ptyDynamic && kv.second.language == 9) withPty++;
    CHECK(withPty == (int)e.services.size(), "programme type on %d of %zu services", withPty, e.services.size());
    CHECK(e.annSupport.size() == e.services.size() && e.annSupport.begin()->second.asu == 0x0003 && e.annSupport.begin()->second.clusters == std::vector<int>{3}, "announcement support");
    CHECK(e.annActive.size() == 1 && e.annActive[0].cluster == 3 && e.annActive[0].asw == 0x0002 && e.annActive[0].subId == 2, "announcement active: %zu", e.annActive.size());
    DabAnnouncement an;
    CHECK(dabActiveAnnouncement(e, 1, 0x0002, an) && an.subId == 2 && an.type == 1, "listener of sub-channel 1 is sent to 2");
    CHECK(!dabActiveAnnouncement(e, 1, 0x0004, an), "not enabled");
    // the announcement ends: no more FIG 0/19, it goes out of the ensemble after a few seconds
    tc.annActive = 0;
    auto syn2 = makeDabSynth(tc, sc, rate);
    for (size_t done = 0; done < (size_t)(6.0 * rate); done += x.size()) { syn2->generate(x.data(), x.size()); rx.feed(x.data(), x.size()); }
    e = rx.ensemble();
    CHECK(e.annActive.empty(), "announcement still on: %zu", e.annActive.size());
}

// FIG 2 labels in UTF-8 (in segments) instead of FIG 1: the receiver shows them
static void testUtf8Labels() {
    dabgen::TxConfig tc;
    tc.utcSeconds = 1700000000;
    tc.utf8Labels = true;
    tc.ensembleLabel = "R\xC3\xA9seau \xC3\x9F";           // Réseau ß
    tc.services = dabgen::defaultServices();
    tc.services.resize(2);
    tc.services[0].label = "Caf\xC3\xA9 \xC3\xB1 Radio";    // Café ñ Radio
    tc.services[1].label = "\xC3\x9C" "ber Mix";               // Über Mix
    SynthConfig sc;
    const double rate = 2.048e6;
    auto syn = makeDabSynth(tc, sc, rate);
    if (!syn) { CHECK(false, "no generator"); return; }
    DabReceiver rx;
    rx.configure(rate);
    rx.audio().setSilent(true);
    std::vector<cf32> x(65536);
    for (size_t done = 0; done < (size_t)(6.0 * rate); done += x.size()) { syn->generate(x.data(), x.size()); rx.feed(x.data(), x.size()); }
    const DabEnsemble e = rx.ensemble();
    CHECK(e.label == tc.ensembleLabel, "ensemble label '%s'", e.label.c_str());
    CHECK(e.services.size() == 2, "%zu services", e.services.size());
    for (size_t i = 0; i < 2; i++) {
        const auto it = e.services.find(tc.services[i].sid);
        CHECK(it != e.services.end() && it->second.label == tc.services[i].label, "service %zu label '%s'", i, it == e.services.end() ? "-" : it->second.label.c_str());
    }
    // the FIC tab decodes them too
    const DabFicAnalysis fic = rx.ficAnalysis();
    const DabFigContext ctx = dabFigContext(fic);
    bool seen = false;
    for (const DabFigRecord& r : fic.figs) if (r.type == 2 && r.ext == 1) { const auto d = dabFigDecode(r, ctx); for (const auto& l : d.lines) if (l.text == "Label=\"" + tc.services[0].label + "\"") seen = true; }
    CHECK(seen, "assembled label in the FIC tab");
}

int main() {
    testDecoders();
    testInfo();
    testReceiver();
    testAnnounce();
    testUtf8Labels();
    if (fails) { jprintf("%d FAILED\n", fails); return 1; }
    jprintf("test_dab_fig: all passed\n");
    return 0;
}
