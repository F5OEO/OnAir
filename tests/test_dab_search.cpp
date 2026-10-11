// DAB sync search on channels with no DAB: silence and signals with deep power dips look like a null symbol, and the phase reference search that
// follows is the costly part. It must not run on every block that comes in (that took ten times real time), and a DAB signal that starts after
// them must still be found.
#include "dect2/dab.h"
#include "dect2/dab_gen.h"
#include <chrono>
#include <cmath>
#include <cstdio>
#include <random>
#include <vector>
using namespace dect2;

static int fails = 0;
#define CHECK(c, ...) do { if (!(c)) { printf("FAIL line %d: ", __LINE__); printf(__VA_ARGS__); printf("\n"); fails++; } } while (0)

static double feedAll(DabReceiver& rx, const std::vector<cf32>& x, double rate) {
    const auto t0 = std::chrono::steady_clock::now();
    for (size_t i = 0; i < x.size(); i += 16384) rx.feed(x.data() + i, std::min<size_t>(16384, x.size() - i));
    return 100.0 * std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() / ((double)x.size() / rate);
}

int main() {
    const double rate = 2.1e6;
    const size_t N = (size_t)(4 * rate);
    std::mt19937 g(1);
    std::normal_distribution<float> nd(0, 1.f);
    std::vector<cf32> x(N);
    {
        DabReceiver rx;
        rx.configure(rate);
        rx.audio().setSilent(true);
        const double zeros = feedAll(rx, x, rate);
        CHECK(zeros < 60, "silence: load %.0f %%", zeros);
        for (size_t period : {5000, 50000}) {
            for (size_t i = 0; i < N; i++) x[i] = cf32(nd(g), nd(g)) * (((i / period) & 1) ? 1.f : 0.001f);
            const double l = feedAll(rx, x, rate);
            CHECK(l < 60, "power dips every %zu samples: load %.0f %%", period, l);
        }
    }
    {   // the DAB signal after a channel with nothing: found within a second or two
        DabReceiver rx;
        rx.configure(rate);
        rx.audio().setSilent(true);
        for (size_t i = 0; i < N; i++) x[i] = cf32(nd(g), nd(g)) * (((i / 5000) & 1) ? 1.f : 0.001f);
        feedAll(rx, x, rate);
        dabgen::TxConfig tc;
        tc.utcSeconds = 1700000000;
        SynthConfig sc;
        sc.snrDb = 15;
        auto syn = makeDabSynth(tc, sc, rate);
        std::vector<cf32> y((size_t)(3 * rate));
        syn->generate(y.data(), y.size());
        feedAll(rx, y, rate);
        DabTelemetry t;
        rx.telemetry(t, 0);
        CHECK(t.state == 2 && rx.ensemble().valid, "no lock after the DAB signal started: state %d", t.state);
    }
    if (fails) { printf("%d FAILED\n", fails); return 1; }
    printf("test_dab_search: all passed\n");
    return 0;
}
