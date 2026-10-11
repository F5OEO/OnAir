// GSM-R through the engine (setStandard(35)): the synthetic source plays the two railway cells at 5 Msps; the receiver finds them and reports
// through latestRx() with their ARFCNs.
#include "dect2/engine.h"
#include "dect2/gsmr_gen.h"
#include "dect2/gsmr_rx.h"
#include "dect2/gsmr_tel.h"
#include "dect2/modes.h"
#include <chrono>
#include <cmath>
#include <cstdio>
#include <thread>
using namespace dect2;
static int fails = 0;
#define CHECK(c, ...) do { if (!(c)) { printf("FAIL line %d: ", __LINE__); printf(__VA_ARGS__); printf("\n"); fails++; } } while (0)

int main() {
    const ModeTuning mt = gsmrTuning();
    Engine e;
    DeviceInfo dev;
    TuneSettings t;
    t.centerHz = 922.8e6;
    t.bandwidthMhz = mt.bandwidthMhz;
    t.sampleRate = mt.sampleRate;
    t.synth.mode = 35;
    t.synth.snrDb = 22;
    FileOptions fo;
    e.setStandard(35);
    CHECK(e.start(dev, t, fo), "engine start");
    CHECK(e.activeStandard() == 34, "active standard %d", e.activeStandard());
    RxTelemetry rx;
    uint64_t seq = 0;
    int reports = 0, wrongStd = 0;
    bool ok = false;
    const auto t0 = std::chrono::steady_clock::now();
    while (std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() < 60.0) {
        if (e.latestRx(rx, seq)) {
            seq = rx.seq; reports++;
            if (rx.standard != 34) wrongStd++;
            bool a = false, b = false;
            for (const auto& c : rx.gsmr.cells) { if (c.arfcn == 960 && c.hasSi3) a = true; if (c.arfcn == 968 && c.hasSi3) b = true; }
            if (a && b) ok = true;
            if (ok) break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    printf("  %d reports: %s\n", reports, gsmrSummary(rx.gsmr).c_str());
    CHECK(ok, "test cells not identified");
    for (const auto& c : rx.gsmr.cells)
        if (c.arfcn == 960) CHECK(c.mcc == 1 && c.mnc == 1 && c.lac == 4711 && c.cellId == 12001 && c.bsic == 10, "cell fields: LAC %u cell %u BSIC %d", c.lac, c.cellId, c.bsic);
    CHECK(wrongStd == 0 && rx.rateOk, "standard / rate");
    CHECK(!modeSummary(rx).empty(), "summary");
    e.stop();
    printf(fails ? "gsmr engine: %d FAILED\n" : "gsmr engine: all passed\n", fails);
    return fails ? 1 : 0;
}
