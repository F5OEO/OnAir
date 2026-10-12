#include "dect2/engine.h"
#include "dect2/hfdig_ftx.h"
#include "dect2/rate_choice.h"
#include "dect2/dvbt.h"
#include "dect2/isdbt.h"
#include "dect2/modes.h"
#include "dect2/t2rx.h"
#include <chrono>
#include "dect2/platform.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <algorithm>
#include <cstdint>
#include <map>
#include <mutex>
#include <atomic>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace dect2 {

// A radio that delivers floating-point samples (LimeSDR, SoapySDR, a cf32 recording) can hand over NaN or infinite values after a
// glitch or with mismatched firmware. They would poison every filter, FFT and gain loop after them, so they become silence here.
static void dropNonFinite(cf32* x, size_t n) {
    // looks at the exponent bits of every float: all ones means NaN or infinity. One branch per block of 256 values, the loop vectorises.
    static_assert(sizeof(cf32) == 2 * sizeof(float), "cf32 layout");
    uint32_t* w = reinterpret_cast<uint32_t*>(x);
    const size_t total = 2 * n;
    for (size_t b = 0; b < total; b += 256) {
        const size_t e = std::min<size_t>(b + 256, total);
        uint32_t bad = 0;
        for (size_t i = b; i < e; i++) bad |= (uint32_t)((w[i] & 0x7F800000u) == 0x7F800000u);
        if (bad)
            for (size_t i = b & ~(size_t)1; i < e; i += 2)
                if ((w[i] & 0x7F800000u) == 0x7F800000u || (w[i + 1] & 0x7F800000u) == 0x7F800000u) { w[i] = 0; w[i + 1] = 0; }
    }
}

// The receiver that runs for an engine standard code (setStandard): 0 and 1 are DVB-T2, 2 DVB-T, ... so it is the code minus one
static int activeStdFor(int stdMode) { return stdMode >= 2 ? stdMode - 1 : 0; }

// "Remove DC spike" and "IQ correction" of the samples (iq_correct.h), with the part skipped that would harm the active mode (tests/test_iq_modes.cpp):
// the DC removal where the wanted carrier can sit on the centre (FM, and the modes whose ModeTuning says carrierAtCentre), the IQ correction
// where the signal is not circular around the centre (ModeTuning::notCircular)
static void cleanSamples(IqCorrector& fix, OffsetMixer& mix, bool forceDc, cf32* x, size_t n, int activeStd) {
    const ModeTuning* mt = modeTuning(activeStd + 1);
    fix.process(x, n, activeStd == 6 || (mt && mt->carrierAtCentre), mt && mt->notCircular, forceDc);
    mix.mix(x, n);   // offset tuning: the channel back to the centre, after the IQ correction (the imbalance mirrors around the radio's centre)
}

// Offset tuning only where one channel sits on the centre: the TV standards, DAB, FM, DVB-S, DTMB, DMR, DRM, ADS-B, GNSS and radiosondes.
// Not analog TV (its own DC handling, carriers both sides), and not the modes that already tune off the channel or watch several channels.
bool Engine::offsetAllowed() const {
    const int m = stdMode_.load();
    if (m == 10 || m == 16 || m == 18 || m == 21) return false;   // analog TV, AIS, ACARS, Iridium
    if (const ModeTuning* mt = modeTuning(m)) if (mt->tuneOffsetHz != 0) return false;
    return m >= 0 && m <= 15;
}

void Engine::ingest(cf32* x, size_t n) {
    rec_.push(x, n);   // the raw samples, before anything touches them
    dropNonFinite(x, n);
    cleanSamples(iqFix_, offMix_, offsetDc_.load(), x, n, activeStd_.load());
    const auto a0 = std::chrono::steady_clock::now();
    lastSamples_ = a0;   // the radio is alive, whether or not the resampler gives a sample out of this block
    if (offResample_) { offBuf_.clear(); offRs_.process(x, n, offBuf_); x = offBuf_.data(); n = offBuf_.size(); }
    if (!n) return;
    if (sigid_.armed()) sigid_.push(x, n);
    feedSpectrum(x, n);
    if (const int s = secAct_.load(); s >= 0 && s != activeStd_.load()) pushSecondary(x, n);   // the second mode: a copy, before the first receiver
    feedRx(x, n);
    tRx_ += std::chrono::duration<double>(std::chrono::steady_clock::now() - a0).count();
    nSamp_ += n;
}

Engine::Engine() : t0_(std::chrono::steady_clock::now()) {}
Engine::~Engine() { stop(); }

void Engine::log(const std::string& line) {
    double t = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0_).count();
    char buf[32];
    snprintf(buf, sizeof buf, "%7.2fs ", t);
    std::lock_guard<std::mutex> lk(logMu_);
    log_.push_back(buf + line);
    if (log_.size() > 5000) log_.erase(log_.begin(), log_.begin() + 1000);
}

std::string Engine::loadProfile() const {
    char b[200];
    snprintf(b, sizeof b, "analysis thread: receiver %.2f s for %.2f s of signal processed (the spectrum thread used %.2f s)", tRx_, nSamp_ / std::max(1.0, rate_.load()), tSpec_);
    return b;
}

std::vector<std::string> Engine::logSnapshot(size_t& total) {
    std::lock_guard<std::mutex> lk(logMu_);
    total = log_.size();
    return log_;
}

void Engine::clearLog() {
    std::lock_guard<std::mutex> lk(logMu_);
    log_.clear();
}

bool Engine::start(const DeviceInfo& dev, const TuneSettings& tune, const FileOptions& file) {
    stop();
    ring_.clear();
    ring_.resetDropped();
    radioMissing_ = 0; radioMissEvents_ = 0; cpuDropEvents_ = 0; ringDroppedSeen_ = 0;
    lastRadioMissNs_ = 0; lastCpuDropNs_ = 0; loadPct_ = 0; loadT_ = 0;
    holdDelivery(1000);   // a radio often stutters while its stream starts
    std::string err;
    if (dev.kind == DeviceInfo::File)
        src_ = makeFileSource(file.path, file.format, file.sampleRate, file.loop);
    else
        src_ = makeSource(dev);
    if (!src_) { log("no source"); return false; }
    offHz_ = 0; offRate_ = 0; offsetDc_ = false;
    std::string offNote;
    if (autoOffset_ && dev.isRadio() && offsetAllowed()) {
        // a HackRF has no listed maximum (20 Msps); the HackRF Pro tunes exactly only at up to 10 Msps and at 20, so it gets 20 above 10
        double maxRate = dev.maxRateHz > 0 ? dev.maxRateHz : dev.kind == DeviceInfo::HackRF ? 20e6 : 0;
        // a link slower than the converter (a PlutoSDR on its USB cable): no faster than it carries, or than the mode already runs above it
        if (dev.steadyRateHz > 0) maxRate = std::max(tune.sampleRate, linkCapHz(dev, tune));
        OffsetPlan p = planOffset(tune.bandwidthMhz * 1e6, tune.sampleRate, maxRate, dev.minRateHz);
        if (p.ok && dev.kind == DeviceInfo::HackRF && p.rateHz > 10e6) p.rateHz = 20e6;
        if (p.ok) { offHz_ = p.offsetHz; offRate_ = p.rateHz; }
        else { offsetDc_ = true; offNote = "offset tuning not possible (" + p.why + "): removing the DC spike instead"; }
    }
    // the spectrum analyser on a HackRF: the radio's own sweep (DECT2_SPECAN_STEPPED=1 steps it like any other radio)
    specanSweep_ = stdMode_.load() == 44 && dev.kind == DeviceInfo::HackRF && !getenv("DECT2_SPECAN_STEPPED");
    if (!src_->start(radioTune(tune), ring_, err)) {
        log("source start failed: " + err);
        src_.reset();
        return false;
    }
    if (offHz_ > 0 && src_->sampleRate() / 2 < offHz_ + tune.bandwidthMhz * 0.5e6) {
        // the radio did not take the rate the offset needs: back on the channel, with the DC removal
        offNote = "offset tuning not possible (the radio runs at " + std::to_string(src_->sampleRate() / 1e6).substr(0, 5) + " Msps): removing the DC spike instead";
        offHz_ = 0; offRate_ = 0; offsetDc_ = true;
        std::string e2;
        if (!src_->retune(radioTune(tune), e2)) log("retune failed: " + e2);
    }
    { std::lock_guard<std::mutex> lk(noteMu_); note_.clear(); retuneNote_.clear(); }
    setNote(offNote.empty() ? err : err.empty() ? offNote : err + "; " + offNote, false);
    srcRate_ = src_->sampleRate();
    rate_ = srcRate_;
    live_ = src_->realtimeHardware();
    // FT8 / WSPR slots: a live radio follows the system clock; a recording or the test signal starts anywhere in a slot, so the
    // slots are found from the signals
    hfdig().ftx().setSlotSearch(!live_.load());
    offMix_.set(offHz_, srcRate_);
    offResample_ = false;
    if (offHz_ > 0) {
        // back to the mode's rate after the shift (if the radio runs faster than the mode asked for)
        if (tune.sampleRate > 0 && srcRate_ > tune.sampleRate * 1.001 && offRs_.configure(srcRate_, tune.sampleRate)) { offRs_.reset(); offResample_ = true; rate_ = tune.sampleRate; }
        char b[200];
        snprintf(b, sizeof b, "offset tuning: the radio sits %.3f MHz below the channel at %.2f Msps, its DC spike outside the channel; the receiver gets %.3f Msps",
                 offHz_.load() / 1e6, srcRate_ / 1e6, rate_.load() / 1e6);
        log(b);
    }
    {
        std::lock_guard<std::mutex> lk(tuneMu_);
        lastDev_ = dev; lastTune_ = tune; lastFile_ = file;
    }
    radioLost_ = false; lastSamples_ = std::chrono::steady_clock::now(); reconnectErr_.clear();
    logP1Count_ = 0; logGi_ = -2; logState_ = -1; logFrameSyms_ = 0;
    bwMhz_ = tune.bandwidthMhz; bwActive_ = bwMhz_;
    bwDetected_ = 0; bwReset_ = true; bwReq_ = false;
    rx_.configure(rate_, tune.bandwidthMhz);
    {   // the resampler on its own thread: live radios on Windows by default (not yet tried elsewhere); DECT2_PIPELINE=0/1 forces it off/on
#ifdef _WIN32
        bool pipe = src_->realtimeHardware();
#else
        bool pipe = false;
#endif
        if (const char* e = getenv("DECT2_PIPELINE")) pipe = atoi(e) != 0;
        rx_.setPipelined(pipe);
    }
    rxT_.configure(rate_, tune.bandwidthMhz);
    rxT_.setPacketCallback([this](const uint8_t* pk, size_t n, double secs) { onTsPackets(pk, n, secs); });
    rxA_.configure(rate_);
    rxA_.setBlocking(!src_->realtimeHardware());
    rxA_.setPacketCallback([this](const uint8_t* pk, size_t n, double secs) { onTsPackets(pk, n, secs); });   // runs on the ATSC worker thread
    rxD_.configure(rate_);
    rxD_.setPacketCallback([this](const uint8_t* pk, size_t n, double secs) { onTsPackets(pk, n, secs); });   // DMB video, on the receiver thread
    rxI_.configure(rate_, tune.isdbtBwMhz);
    rxI_.setPacketCallback([this](const uint8_t* pk, size_t n, double secs) { onTsPackets(pk, n, secs); });
    rxFm_.configure(rate_);
    rxFm_.setStationFinder(lastDev_.kind == DeviceInfo::File || lastDev_.kind == DeviceInfo::Synthetic);   // a live radio stays on the frequency tuned: a weak station is not traded for a strong neighbour
    if (stdMode_.load() == 7 && !rxFm_.ready()) log("FM needs a sample rate of at least 500 ksps");
    rxDvbs_.configure(rate_);
    rxDvbs_.setPacketCallback([this](const uint8_t* pk, size_t n, double secs) { onTsPackets(pk, n, secs); });   // runs on the receiver thread
    rxDvbs_.setLogCallback([this](const std::string& s) { log(s); });
    rxDtmb_.configure(rate_, tune.bandwidthMhz);
    rxDtmb_.setPacketCallback([this](const uint8_t* pk, size_t n, double secs) { onTsPackets(pk, n, secs); });   // runs on the receiver thread
    rxDtmb_.setLogCallback([this](const std::string& s) { log(s); });
    rxAtv_.configure(rate_);
    rxAtv_.setLogCallback([this](const std::string& s) { log(s); });
    rxDmr_.configure(rate_);
    rxDmr_.setLogCallback([this](const std::string& s) { log(s); });
    rxDrm_.configure(rate_);
    rxDrm_.setLogCallback([this](const std::string& s) { log(s); });
    rxAdsb_.configure(rate_);
    rxAdsb_.setLogCallback([this](const std::string& s) { log(s); });
    rxGnss_.configure(rate_);
    rxGnss_.setLogCallback([this](const std::string& s) { log(s); });
    {   // a narrow channel sits tuneOffsetHz away from the radio's centre (a file holds the channel as recorded: no offset)
        const ModeTuning* mto = modeTuning(stdMode_.load());
        const double o = mto && dev.kind != DeviceInfo::File ? -mto->tuneOffsetHz : 0.0;
        rxSonde_.setSignalOffset(o);
        rxAis_.setSignalOffset(o);
        rxMarine_.setSignalOffset(o);
        rxAcars_.setSignalOffset(o);
        if (dev.kind != DeviceInfo::File) rxAcars_.setCenterHz(radioTune(tune).centerHz);   // channel frequencies and the 25 kHz grid come from it
        rxInmc_.setSignalOffset(o);
        rxAero_.setSignalOffset(o);
        rxIridium_.setSignalOffset(o);
        rxMesh_.setSignalOffset(o);
        rxPager_.setSignalOffset(o);
        rxPacket_.setSignalOffset(o);
        rxHfdig_.setSignalOffset(o);
        rxAirband_.setSignalOffset(o);
        rxVdl2_.setSignalOffset(o);
        rxVdl2_.setCenterHz(dev.kind != DeviceInfo::File ? radioTune(tune).centerHz : 0.0);
        rxAirband_.setCenterHz(dev.kind != DeviceInfo::File ? radioTune(tune).centerHz : 0.0);   // the channels are placed from it
        rxIsm_.setSignalOffset(o);
        rxIsm_.setCenterHz(dev.kind != DeviceInfo::File ? radioTune(tune).centerHz : 0.0);   // the devices' frequencies
        rxVor_.setSignalOffset(o);
        rxVor_.setCenterHz(dev.kind != DeviceInfo::File ? radioTune(tune).centerHz : 0.0);   // VOR, localizer, glide slope or marker by the frequency
        rxM17_.setSignalOffset(o);
        rxHfdl_.setSignalOffset(o);
        rxHfdl_.setCenterHz(dev.kind != DeviceInfo::File ? radioTune(tune).centerHz : 0.0);   // the channels are placed from it
        rxCell_.setSignalOffset(o);
        rxCell_.setCenterHz(dev.kind != DeviceInfo::File ? radioTune(tune).centerHz : 0.0);   // the cells' EARFCNs
        rxCw_.setSignalOffset(o);
        rxDstar_.setSignalOffset(o);
        rxNxdn_.setSignalOffset(o);
        rxDpmr_.setSignalOffset(o);
        rxMac_.setSignalOffset(o);
        rxP25_.setSignalOffset(o);
        rxGsmr_.setSignalOffset(o);
        rxGsmr_.setCenterHz(dev.kind != DeviceInfo::File ? radioTune(tune).centerHz : 0.0);   // the carriers' ARFCNs
        rxTetra_.setSignalOffset(o);
        rxTetra_.setCenterHz(dev.kind != DeviceInfo::File ? radioTune(tune).centerHz : 0.0);   // the carriers' frequencies
        rxPsk_.setSignalOffset(o);
        rxJs8_.setSignalOffset(o);
        rxAle_.setSignalOffset(o);
        rxAle_.setDialHz(dev.kind != DeviceInfo::File ? tune.centerHz : 0.0);   // the dial, logged with each call
        rxOrb_.setCenterHz(dev.kind != DeviceInfo::File ? radioTune(tune).centerHz : 0.0);   // absolute frequencies and Doppler
        rxBle_.setSignalOffset(o);
        rxBle_.setCenterHz(dev.kind != DeviceInfo::File ? radioTune(tune).centerHz : 0.0);   // the advertising channel tried first
        rxQo100_.setCenterHz(dev.kind != DeviceInfo::File ? radioTune(tune).centerHz : 0.0);   // the IF: where the beacons and transponders lie
        rxTime_.setSignalOffset(o);
        rxCube_.setSignalOffset(o);
        rxCube_.setDialHz(dev.kind != DeviceInfo::File ? tune.centerHz : 0.0);   // names frames after the satellite table
        rxTime_.setDialHz(dev.kind != DeviceInfo::File ? tune.centerHz : 0.0);   // the station when none is chosen
        rxWifi_.setSignalOffset(o);
        rxGeneral_.setSignalOffset(o);
        rxRid_.setSignalOffset(o);
        rxRid_.setCenterHz(dev.kind != DeviceInfo::File ? radioTune(tune).centerHz : 0.0);   // which BLE and Wi-Fi channels lie in the band
        rxMeteor_.setSignalOffset(o);
        rxNr_.setSignalOffset(o);
        rxNr_.setCenterHz(dev.kind != DeviceInfo::File ? radioTune(tune).centerHz : 0.0);   // the sync raster points searched
        rxGoes_.setSignalOffset(o);
        rxWifi_.setCenterHz(dev.kind != DeviceInfo::File ? radioTune(tune).centerHz : 0.0);   // the channel noted with each network
    }
    rxSonde_.configure(rate_);
    rxSonde_.setLogCallback([this](const std::string& s) { log(s); });
    rxAis_.configure(rate_);
    rxAis_.setLogCallback([this](const std::string& s) { log(s); });
    rxMarine_.configure(rate_);
    rxMarine_.setLogCallback([this](const std::string& s) { log(s); });
    rxAcars_.configure(rate_);
    rxAcars_.setLogCallback([this](const std::string& s) { log(s); });
    rxInmc_.configure(rate_);
    rxInmc_.setLogCallback([this](const std::string& s) { log(s); });
    rxAero_.configure(rate_);
    rxAero_.setLogCallback([this](const std::string& s) { log(s); });
    rxIridium_.configure(rate_);
    rxIridium_.setLogCallback([this](const std::string& s) { log(s); });
    rxMesh_.configure(rate_);
    rxMesh_.setLogCallback([this](const std::string& s) { log(s); });
    rxHdr_.configure(rate_);
    rxHdr_.setLogCallback([this](const std::string& s) { log(s); });
    rxCdr_.configure(rate_);
    rxCdr_.setLogCallback([this](const std::string& s) { log(s); });
    rxPager_.configure(rate_);
    rxPager_.setLogCallback([this](const std::string& s) { log(s); });
    rxPacket_.configure(rate_);
    rxPacket_.setLogCallback([this](const std::string& s) { log(s); });
    rxHfdig_.configure(rate_);
    rxHfdig_.setLogCallback([this](const std::string& s) { log(s); });
    rxAirband_.configure(rate_);
    rxVdl2_.configure(rate_);
    rxVdl2_.setLogCallback([this](const std::string& s) { log(s); });
    rxAirband_.setLogCallback([this](const std::string& s) { log(s); });
    rxIsm_.configure(rate_);
    rxIsm_.setLogCallback([this](const std::string& s) { log(s); });
    rxVor_.configure(rate_);
    rxVor_.setLogCallback([this](const std::string& s) { log(s); });
    rxM17_.configure(rate_);
    rxHfdl_.configure(rate_);
    rxHfdl_.setLogCallback([this](const std::string& s) { log(s); });
    rxM17_.setLogCallback([this](const std::string& s) { log(s); });
    rxCell_.configure(rate_);
    rxCell_.setLogCallback([this](const std::string& s) { log(s); });
    rxCw_.configure(rate_);
    rxCw_.setLogCallback([this](const std::string& s) { log(s); });
    rxDstar_.configure(rate_);
    rxDstar_.setLogCallback([this](const std::string& s) { log(s); });
    rxNxdn_.configure(rate_);
    rxDpmr_.configure(rate_);
    rxDpmr_.setChannel(tune.centerHz / 1e6);   // the PMR446 channel it sits on (the scan steps from there)
    rxDpmr_.setLogCallback([this](const std::string& s) { log(s); });
    rxMac_.configure(rate_);
    rxMac_.setLogCallback([this](const std::string& s) { log(s); });
    rxNxdn_.setLogCallback([this](const std::string& s) { log(s); });
    rxP25_.configure(rate_);
    rxP25_.setLogCallback([this](const std::string& s) { log(s); });
    rxGsmr_.configure(rate_);
    rxGsmr_.setLogCallback([this](const std::string& s) { log(s); });
    rxTetra_.configure(rate_);
    rxTetra_.setLogCallback([this](const std::string& s) { log(s); });
    // the other receivers say what they listen for when configured, and the last of these stays on the bottom line: name the active one last
    if (stdMode_.load() == 34) log("Cell towers: searching for LTE cells");
    if (stdMode_.load() == 41) log("GSM-R: searching for base stations (BCCH carriers)");
    if (stdMode_.load() == 48) log("TETRA: searching for base stations (receive only, nothing is decrypted)");
    rxPsk_.configure(rate_);
    rxPsk_.setLogCallback([this](const std::string& s) { log(s); });
    rxJs8_.configure(rate_);
    rxSpecx_.configure(rate_);
    rxSpecx_.setLogCallback([this](const std::string& s) { log(s); });
    rxSpecan_.configure(rate_);
    rxSpecan_.setLogCallback([this](const std::string& s) { log(s); });
    rxSpecan_.setCenterHz(dev.kind != DeviceInfo::File ? radioTune(tune).centerHz : 0.0);
    // stepped sweeps retune the radio straight from the receiver (no hold, no reset: the receiver skips the settling samples itself);
    // a file, the test signal and the HackRF's own sweep do not step
    if (stdMode_.load() == 44 && dev.isRadio() && !specanSweep_)
        rxSpecan_.setRetuneCallback([this](double hz) {
            if (!src_) return false;
            TuneSettings t; { std::lock_guard<std::mutex> lk(tuneMu_); t = lastTune_; }
            t.centerHz = hz;
            std::string err;
            return src_->retune(radioTune(t), err);
        });
    else rxSpecan_.setRetuneCallback(nullptr);
    rxAstro_.configure(rate_);
    rxAstro_.setLogCallback([this](const std::string& s) { log(s); });
    // frequency switching retunes the radio straight from the receiver (it skips the settling samples itself); a file or the test signal cannot
    if (stdMode_.load() == 50 && dev.isRadio())
        rxAstro_.setRetuneCallback([this](double offsetHz) {
            if (!src_) return false;
            TuneSettings t; { std::lock_guard<std::mutex> lk(tuneMu_); t = lastTune_; }
            t.centerHz += offsetHz;
            std::string err;
            return src_->retune(radioTune(t), err);
        });
    else rxAstro_.setRetuneCallback(nullptr);
    rxDvbc_.configure(rate_);
    rxDvbc_.setBlocking(dev.kind == DeviceInfo::File);
    rxDvbc_.setPacketCallback([this](const uint8_t* pk, size_t n, double secs) { onTsPackets(pk, n, secs); });   // runs on the receiver thread
    rxDvbc_.setLogCallback([this](const std::string& s) { log(s); });
    rxOrb_.configure(rate_);
    rxOrb_.setLogCallback([this](const std::string& s) { log(s); });
    rxUplink_.configure(rate_);
    rxUplink_.setLogCallback([this](const std::string& s) { log(s); });
    rxAle_.configure(rate_);
    rxAle_.setLogCallback([this](const std::string& s) { log(s); });
    rxBle_.configure(rate_);
    rxBle_.setLogCallback([this](const std::string& s) { log(s); });
    rxIono_.configure(rate_);
    rxIono_.setLogCallback([this](const std::string& s) { log(s); });
    rxIono_.setCenterHz(radioTune(tune).centerHz);   // 0 .. 30 MHz is placed from it (a file: the frequency it was set to)
    // sweep starts are whole seconds of UTC with a radio (the computer clock at the first sample); a file or the test signal counts from 0
    rxIono_.setTimeOrigin(dev.isRadio() ? std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count() : 0.0);
    rxQo100_.configure(rate_);
    rxTime_.configure(rate_);
    rxCube_.configure(rate_);
    rxCube_.setWallClock(dev.kind != DeviceInfo::File);
    rxCube_.setLogCallback([this](const std::string& s) { log(s); });
    rxTime_.setLogCallback([this](const std::string& s) { log(s); });
    rxTime_.setClockMode(dev.kind == DeviceInfo::File ? 0 : 1);   // a file has no clock to compare; a radio and the test signal run on the computer's
    rxWifi_.configure(rate_);
    rxGeneral_.configure(rate_);
    rxGeneral_.setLogCallback([this](const std::string& s) { log(s); });
    rxRid_.configure(rate_);
    rxRid_.setLogCallback([this](const std::string& s) { log(s); });
    rxNr_.configure(rate_);
    rxNr_.setLogCallback([this](const std::string& s) { log(s); });
    rxWifi_.setLogCallback([this](const std::string& s) { log(s); });
    rxMeteor_.configure(rate_);
    rxMeteor_.setLogCallback([this](const std::string& s) { log(s); });
    rxGoes_.configure(rate_);
    rxGoes_.setLogCallback([this](const std::string& s) { log(s); });
    rxQo100_.setLogCallback([this](const std::string& s) { log(s); });
    rxJs8_.setLogCallback([this](const std::string& s) { log(s); });
    if (const ModeTuning* mt = modeTuning(stdMode_.load()))
        if (rate_.load() < mt->minSampleRate - 1) {
            char m[200];
            snprintf(m, sizeof m, "sample rate %.2f Msps is too low for %s (at least %.2f Msps is needed) - the receiver cannot work with this radio", rate_.load() / 1e6, mt->name, mt->minSampleRate / 1e6);
            log(m);
        }
    logIMode_ = logIGi_ = logITmcc_ = -1; logISync_ = false;
    rxA3_.setPaced(true);
    rxA3_.configure(rate_);
    rxA3_.setBlocking(!src_->realtimeHardware());
    rxA3_.setPacketCallback([this](const uint8_t* pk, size_t n, double secs) { onTsPackets(pk, n, secs); });   // runs on the ATSC 3.0 worker thread
    atsc3Seq_ = 0; logA3State_ = -1; logA3Svc_ = false;
    { std::lock_guard<std::mutex> lk(atsc3Mu_); atsc3Tel_ = Atsc3Telemetry(); }
    dabSeq_ = 0; logDState_ = -1; logDEns_ = false; dmbSub_ = -1;
    for (auto& s : modeSeq_) s = 0;
    atscSeq_ = 0; logAState_ = -1;
    activeStd_ = activeStdFor(stdMode_.load());
    autoMark_ = 0; lastLockSec_ = 0; logTMode_ = logTGi_ = logTTps_ = -1; logTFec_ = false;
    {
        std::lock_guard<std::mutex> lk(tsMu_);
        demux_.reset();
        unpack_ = BbUnpacker();
        tsSnap_ = TsSnapshot();
        lastT2Frame_ = -1;
        unpack_.setSink([this](const uint8_t* pkt) { demux_.feed(pkt); ttx_.feedTs(pkt); frameBuf_.insert(frameBuf_.end(), pkt, pkt + 188); });
    }
    rx_.setPlpCallback([this](const PlpResult& r) { if (plpDump_) plpDump_(r); onPlp(r); });
    {   // DVB-T2 / DVB-T, ATSC and DAB at a rate their receiver cannot use (the telemetry test that stood here never fired: the receiver
        // publishes nothing before its first samples, so the user got no word at all, e.g. an RTL-SDR at 2.56 Msps on an 8 MHz channel)
        const int sm = stdMode_.load();
        const double need = sm <= 4 ? minSampleRateFor(sm, tune.bandwidthMhz) : 0;
        if (need > 0 && rate_.load() < need - 1) {
            char what[64], m[240];
            if (sm == 3) snprintf(what, sizeof what, "an ATSC channel");
            else if (sm == 4) snprintf(what, sizeof what, "a DAB ensemble");
            else snprintf(what, sizeof what, "the %g MHz %s channel", tune.bandwidthMhz, sm == 1 ? "DVB-T2" : sm == 2 ? "DVB-T" : "DVB-T2 / DVB-T");
            snprintf(m, sizeof m, "sample rate %.2f Msps is too low for %s (at least %.2f Msps is needed) - %s", rate_.load() / 1e6, what, need / 1e6,
                     sm <= 2 ? "only the spectrum is shown" : "the receiver cannot lock with this radio");
            log(m);
        }
    }
    if (stdMode_ == 5 && rate_.load() < 6.5e6) {
        char m[200];
        snprintf(m, sizeof m, "sample rate %.2f Msps is too low for an ATSC 3.0 channel (about 6.5 Msps or more is needed) - the receiver cannot lock with this radio", rate_.load() / 1e6);
        log(m);
    }
    if (stdMode_ == 6 && rate_.load() < minSampleRateFor(6, tune.isdbtBwMhz ? tune.isdbtBwMhz : 6)) {
        char m[200];
        snprintf(m, sizeof m, "sample rate %.2f Msps is too low for a %d MHz ISDB-T channel (about %.0f Msps or more is needed) - the receiver cannot lock with this radio", rate_.load() / 1e6,
                 tune.isdbtBwMhz ? tune.isdbtBwMhz : 6, minSampleRateFor(6, tune.isdbtBwMhz ? tune.isdbtBwMhz : 6) / 1e6);
        log(m);
    }
    char b[160];
    snprintf(b, sizeof b, "source started: %s  fs=%.4f Msps", dev.name.c_str(), rate_.load() / 1e6);
    log(b);
    if (dev.isGeneric()) {
        snprintf(b, sizeof b, "tuned %.3f MHz  gain %.0f dB", tune.centerHz / 1e6, tune.gainDb);
        log(b);
    }
    if (dev.kind == DeviceInfo::HackRF) {
        snprintf(b, sizeof b, "tuned %.3f MHz  LNA %d dB  VGA %d dB  amp %s", tune.centerHz / 1e6, tune.lnaDb, tune.vgaDb, tune.ampOn ? "on" : "off");
        log(b);
    }
    tSpec_ = tRx_ = 0; nSamp_ = 0;
    stopReq_ = false;
    running_ = true;
    specStop_ = false;
    specQ_.clear();
    specTh_ = std::thread([this] { spectrumLoop(); });
    th_ = std::thread([this] { analysisLoop(); });
    startSecondary();   // the second mode asked for before (or kept from the last start)
    return true;
}

void Engine::stop() {
    stopSecondary();   // stays asked for: the next start() brings it back
    player_.select(-1);
    if (!running_ && !src_) return;
    stopReq_ = true;
    if (th_.joinable()) th_.join();
    rec_.stop();   // a recording ends with the radio (the analysis thread is gone, nothing pushes any more)
    {
        std::lock_guard<std::mutex> lk(specQMu_);
        specStop_ = true;
    }
    specQCv_.notify_all();
    if (specTh_.joinable()) specTh_.join();
    live_ = false;
    if (src_) {
        src_->stop(); src_.reset();
        char b[160];
        double dur = nSamp_ / std::max(1.0, rate_.load());
        snprintf(b, sizeof b, "source stopped (CPU load: spectrum %.0f%%, receiver %.0f%% of real time)", 100 * tSpec_ / std::max(1e-9, dur), 100 * tRx_ / std::max(1e-9, dur));
        log(b);
        log(t2rxProfile());
    }
    rxA3_.stop();
    running_ = false;
}

bool Engine::startRecording(const std::string& path, FileFormat fmt, std::string& err) {
    DeviceInfo::Kind kind;
    { std::lock_guard<std::mutex> lk(tuneMu_); kind = lastDev_.kind; }
    if (!running_ || !src_) { err = "start the radio first"; return false; }
    if (kind == DeviceInfo::File) { err = "recording a file playback is not needed: the file is already there"; return false; }
    return rec_.start(path, fmt, srcRate_, err);
}

std::string Engine::recordingName(FileFormat fmt) const {
    TuneSettings t;
    { std::lock_guard<std::mutex> lk(tuneMu_); t = lastTune_; }
    return recordingFileName(recordingModeId(stdMode_.load()), radioTune(t).centerHz, srcRate_, fmt);
}

// The radio of a narrow-channel mode is tuned tuneOffsetHz above the user's frequency, so the channel stays off the DC spike.
TuneSettings Engine::radioTune(const TuneSettings& t) const {
    TuneSettings r = t;
    if (const ModeTuning* mt = modeTuning(stdMode_.load())) r.centerHz += mt->tuneOffsetHz;
    if (offHz_ > 0) { r.centerHz -= offHz_; r.sampleRate = offRate_; r.basebandFilterHz = 0; }   // offset tuning: the radio below the channel
    if (specanSweep_) {   // HackrfSource sweeps this range itself (MHz)
        double lo = 0, hi = 0;
        rxSpecan_.range(lo, hi);
        r.radio["specan_sweep"] = std::to_string(lo / 1e6) + ":" + std::to_string(hi / 1e6);
        r.sampleRate = 20e6; r.basebandFilterHz = 15e6;
    }
    return r;
}

bool Engine::retune(const TuneSettings& tune) {
    if (!src_) return false;
    std::string err;
    holdDelivery(300);
    if (!src_->retune(radioTune(tune), err)) { log("retune failed: " + err); return false; }
    setNote(err, true);
    if (restartIfRateChanged(tune)) return running_;
    double oldCentre = 0;
    { std::lock_guard<std::mutex> lk(tuneMu_); oldCentre = lastTune_.centerHz; lastTune_ = tune; }
    // a retune that does not move (the gain, the AGC) leaves a second mode's receiver alone: placing it again would restart it
    const bool moved = std::fabs(oldCentre - tune.centerHz) > 0.5;
    if (moved || secAct_.load() < 0) {
        if (lastDev_.kind != DeviceInfo::File) { rxIono_.setCenterHz(radioTune(tune).centerHz); rxAcars_.setCenterHz(radioTune(tune).centerHz); rxAirband_.setCenterHz(radioTune(tune).centerHz); rxVdl2_.setCenterHz(radioTune(tune).centerHz); rxIsm_.setCenterHz(radioTune(tune).centerHz); rxVor_.setCenterHz(radioTune(tune).centerHz); rxCell_.setCenterHz(radioTune(tune).centerHz); rxHfdl_.setCenterHz(radioTune(tune).centerHz); rxGsmr_.setCenterHz(radioTune(tune).centerHz); rxTetra_.setCenterHz(radioTune(tune).centerHz); rxOrb_.setCenterHz(radioTune(tune).centerHz); rxAle_.setDialHz(tune.centerHz); rxBle_.setCenterHz(radioTune(tune).centerHz); }
        if (lastDev_.kind != DeviceInfo::File) { rxCube_.setDialHz(tune.centerHz); rxTime_.setDialHz(tune.centerHz); rxWifi_.setCenterHz(radioTune(tune).centerHz); rxAcars_.setCenterHz(radioTune(tune).centerHz); rxAirband_.setCenterHz(radioTune(tune).centerHz); rxVdl2_.setCenterHz(radioTune(tune).centerHz); rxIsm_.setCenterHz(radioTune(tune).centerHz); rxVor_.setCenterHz(radioTune(tune).centerHz); rxCell_.setCenterHz(radioTune(tune).centerHz); rxHfdl_.setCenterHz(radioTune(tune).centerHz); rxGsmr_.setCenterHz(radioTune(tune).centerHz); rxTetra_.setCenterHz(radioTune(tune).centerHz); rxOrb_.setCenterHz(radioTune(tune).centerHz); rxAle_.setDialHz(tune.centerHz); rxBle_.setCenterHz(radioTune(tune).centerHz); rxQo100_.setCenterHz(radioTune(tune).centerHz); }
        if (lastDev_.kind != DeviceInfo::File) { rxRid_.setCenterHz(radioTune(tune).centerHz); rxWifi_.setCenterHz(radioTune(tune).centerHz); rxAcars_.setCenterHz(radioTune(tune).centerHz); rxAirband_.setCenterHz(radioTune(tune).centerHz); rxVdl2_.setCenterHz(radioTune(tune).centerHz); rxIsm_.setCenterHz(radioTune(tune).centerHz); rxVor_.setCenterHz(radioTune(tune).centerHz); rxCell_.setCenterHz(radioTune(tune).centerHz); rxHfdl_.setCenterHz(radioTune(tune).centerHz); rxGsmr_.setCenterHz(radioTune(tune).centerHz); rxTetra_.setCenterHz(radioTune(tune).centerHz); rxOrb_.setCenterHz(radioTune(tune).centerHz); rxAle_.setDialHz(tune.centerHz); rxBle_.setCenterHz(radioTune(tune).centerHz); rxQo100_.setCenterHz(radioTune(tune).centerHz); }
        if (lastDev_.kind != DeviceInfo::File) { rxWifi_.setCenterHz(radioTune(tune).centerHz); rxNr_.setCenterHz(radioTune(tune).centerHz); rxAcars_.setCenterHz(radioTune(tune).centerHz); rxAirband_.setCenterHz(radioTune(tune).centerHz); rxVdl2_.setCenterHz(radioTune(tune).centerHz); rxIsm_.setCenterHz(radioTune(tune).centerHz); rxVor_.setCenterHz(radioTune(tune).centerHz); rxCell_.setCenterHz(radioTune(tune).centerHz); rxHfdl_.setCenterHz(radioTune(tune).centerHz); rxGsmr_.setCenterHz(radioTune(tune).centerHz); rxTetra_.setCenterHz(radioTune(tune).centerHz); rxOrb_.setCenterHz(radioTune(tune).centerHz); rxAle_.setDialHz(tune.centerHz); rxBle_.setCenterHz(radioTune(tune).centerHz); rxQo100_.setCenterHz(radioTune(tune).centerHz); }
    }
    refreshSecondary(moved);
    return true;
}

// Every receiver was set up for the rate the radio had at start(); a retune that changes it (another rate asked for, or the radio
// picking a different one) would leave them decoding at the wrong rate, so the engine starts again with the new settings
// A driver note is logged when it is new (a retune that brings the same one again, as the AGC does, does not fill the log)
void Engine::setNote(const std::string& n, bool retune) {
    {
        std::lock_guard<std::mutex> lk(noteMu_);
        std::string& slot = retune ? retuneNote_ : note_;
        if (n == slot) return;
        slot = n;
    }
    if (!n.empty()) log("note: " + n);
}

bool Engine::restartIfRateChanged(const TuneSettings& tune) {
    if (!src_ || std::fabs(src_->sampleRate() - srcRate_) < 0.5) return false;
    char b[160];
    snprintf(b, sizeof b, "the radio now runs at %.3f Msps (was %.3f): restarting the receivers", src_->sampleRate() / 1e6, srcRate_ / 1e6);
    log(b);
    const DeviceInfo dev = lastDev_;
    const FileOptions file = lastFile_;
    stop();
    start(dev, tune, file);
    return true;
}

bool Engine::retuneReset(const TuneSettings& tune) {
    if (!src_) return false;
    std::string err;
    holdDelivery(300);
    if (!src_->retune(radioTune(tune), err)) { log("retune failed: " + err); return false; }
    setNote(err, true);
    if (restartIfRateChanged(tune)) return running_;
    double oldCentre = 0;
    { std::lock_guard<std::mutex> lk(tuneMu_); oldCentre = lastTune_.centerHz; lastTune_ = tune; }
    // a retune that does not move (the gain, the AGC) leaves a second mode's receiver alone: placing it again would restart it
    const bool moved = std::fabs(oldCentre - tune.centerHz) > 0.5;
    if (moved || secAct_.load() < 0) {
        if (lastDev_.kind != DeviceInfo::File) { rxIono_.setCenterHz(radioTune(tune).centerHz); rxAcars_.setCenterHz(radioTune(tune).centerHz); rxAirband_.setCenterHz(radioTune(tune).centerHz); rxVdl2_.setCenterHz(radioTune(tune).centerHz); rxIsm_.setCenterHz(radioTune(tune).centerHz); rxVor_.setCenterHz(radioTune(tune).centerHz); rxCell_.setCenterHz(radioTune(tune).centerHz); rxHfdl_.setCenterHz(radioTune(tune).centerHz); rxGsmr_.setCenterHz(radioTune(tune).centerHz); rxTetra_.setCenterHz(radioTune(tune).centerHz); rxOrb_.setCenterHz(radioTune(tune).centerHz); rxAle_.setDialHz(tune.centerHz); rxBle_.setCenterHz(radioTune(tune).centerHz); }
        if (lastDev_.kind != DeviceInfo::File) { rxCube_.setDialHz(tune.centerHz); rxTime_.setDialHz(tune.centerHz); rxWifi_.setCenterHz(radioTune(tune).centerHz); rxAcars_.setCenterHz(radioTune(tune).centerHz); rxAirband_.setCenterHz(radioTune(tune).centerHz); rxVdl2_.setCenterHz(radioTune(tune).centerHz); rxIsm_.setCenterHz(radioTune(tune).centerHz); rxVor_.setCenterHz(radioTune(tune).centerHz); rxCell_.setCenterHz(radioTune(tune).centerHz); rxHfdl_.setCenterHz(radioTune(tune).centerHz); rxGsmr_.setCenterHz(radioTune(tune).centerHz); rxTetra_.setCenterHz(radioTune(tune).centerHz); rxOrb_.setCenterHz(radioTune(tune).centerHz); rxAle_.setDialHz(tune.centerHz); rxBle_.setCenterHz(radioTune(tune).centerHz); rxQo100_.setCenterHz(radioTune(tune).centerHz); }
        if (lastDev_.kind != DeviceInfo::File) { rxRid_.setCenterHz(radioTune(tune).centerHz); rxWifi_.setCenterHz(radioTune(tune).centerHz); rxAcars_.setCenterHz(radioTune(tune).centerHz); rxAirband_.setCenterHz(radioTune(tune).centerHz); rxVdl2_.setCenterHz(radioTune(tune).centerHz); rxIsm_.setCenterHz(radioTune(tune).centerHz); rxVor_.setCenterHz(radioTune(tune).centerHz); rxCell_.setCenterHz(radioTune(tune).centerHz); rxHfdl_.setCenterHz(radioTune(tune).centerHz); rxGsmr_.setCenterHz(radioTune(tune).centerHz); rxTetra_.setCenterHz(radioTune(tune).centerHz); rxOrb_.setCenterHz(radioTune(tune).centerHz); rxAle_.setDialHz(tune.centerHz); rxBle_.setCenterHz(radioTune(tune).centerHz); rxQo100_.setCenterHz(radioTune(tune).centerHz); }
        if (lastDev_.kind != DeviceInfo::File) { rxWifi_.setCenterHz(radioTune(tune).centerHz); rxNr_.setCenterHz(radioTune(tune).centerHz); rxAcars_.setCenterHz(radioTune(tune).centerHz); rxAirband_.setCenterHz(radioTune(tune).centerHz); rxVdl2_.setCenterHz(radioTune(tune).centerHz); rxIsm_.setCenterHz(radioTune(tune).centerHz); rxVor_.setCenterHz(radioTune(tune).centerHz); rxCell_.setCenterHz(radioTune(tune).centerHz); rxHfdl_.setCenterHz(radioTune(tune).centerHz); rxGsmr_.setCenterHz(radioTune(tune).centerHz); rxTetra_.setCenterHz(radioTune(tune).centerHz); rxOrb_.setCenterHz(radioTune(tune).centerHz); rxAle_.setDialHz(tune.centerHz); rxBle_.setCenterHz(radioTune(tune).centerHz); rxQo100_.setCenterHz(radioTune(tune).centerHz); }
    }
    refreshSecondary(moved);
    resetReq_ = true;
    return true;
}

void Engine::applyReset() {
    std::lock_guard<std::mutex> secLk(secFeedMu_);   // the second mode's receiver is reset with the rest, never while it takes a block
    {
        std::lock_guard<std::mutex> lk(secQMu_);
        secQ_.clear(); secQN_ = 0;
    }
    secQCv_.notify_all();
    secFlush_ = true;
    ring_.clear();
    holdDelivery(300);
    rx_.reset();
    rxT_.reset();
    rxA_.reset();
    rxD_.reset();
    rxA3_.reset();
    rxI_.reset();
    rxFm_.reset();
    rxDvbs_.reset();
    rxDtmb_.reset();
    rxAtv_.reset();
    rxDmr_.reset();
    rxDrm_.reset();
    rxAdsb_.reset();
    rxGnss_.reset();
    rxSonde_.reset();
    rxAis_.reset();
    rxMarine_.reset();
    rxAcars_.reset();
    rxInmc_.reset();
    rxAero_.reset();
    rxIridium_.reset();
    rxMesh_.reset();
    rxHdr_.reset();
    rxCdr_.reset();
    rxPager_.reset();
    rxPacket_.reset();
    rxHfdig_.reset();
    rxAirband_.reset();
    rxVdl2_.reset();
    rxIsm_.reset();
    rxVor_.reset();
    rxM17_.reset();
    rxHfdl_.reset();
    rxCell_.reset();
    rxCw_.reset();
    rxDstar_.reset();
    rxNxdn_.reset();
    rxDpmr_.reset();
    rxMac_.reset();
    rxP25_.reset();
    rxGsmr_.reset();
    rxTetra_.reset();
    rxPsk_.reset();
    rxJs8_.reset();
    rxSpecx_.reset();
    rxSpecan_.reset();
    rxAstro_.reset();
    rxDvbc_.reset();
    rxOrb_.reset();
    rxUplink_.reset();
    rxAle_.reset();
    rxBle_.reset();
    rxIono_.reset();
    rxQo100_.reset();
    rxWifi_.reset();
    rxGeneral_.reset();
    rxTime_.reset();
    rxCube_.reset();
    rxRid_.reset();
    rxMeteor_.reset();
    rxNr_.reset();
    rxGoes_.reset();
    autoMark_ = nSamp_ / std::max(1.0, rate_.load()); lastLockSec_ = autoMark_;
    analyzer_.reset();
    {
        std::lock_guard<std::mutex> lk(tsMu_);
        demux_.reset();
        unpack_ = BbUnpacker();
        unpack_.setSink([this](const uint8_t* pkt) { demux_.feed(pkt); ttx_.feedTs(pkt); frameBuf_.insert(frameBuf_.end(), pkt, pkt + 188); });
        tsSnap_ = TsSnapshot();
        lastT2Frame_ = -1;
        frameBuf_.clear();
    }
    {
        std::lock_guard<std::mutex> lk(specMu_);
        spec_ = SpectrumFrame();
    }
    {
        std::lock_guard<std::mutex> lk(rxMu_);
        rxTel_ = RxTelemetry();
    }
    logP1Count_ = 0; logGi_ = -2; logState_ = -1; logFrameSyms_ = 0;
    resets_++;
}

void Engine::onTsPackets(const uint8_t* pk, size_t n, double secs) {
    std::lock_guard<std::mutex> lk(tsMu_);
    demux_.setGbDefault(stdMode_.load() == 8);   // DTMB
    for (size_t i = 0; i < n; i++) {
        const uint8_t* q = pk + i * 188;
        demux_.feed(q);
        ttx_.feedTs(q);
    }
    demux_.advance(secs);
    tsSnap_ = demux_.snapshot();
    outputs_.packets(pk, n, &tsSnap_);
    if (tap_) tap_(pk, n, &tsSnap_);
    if (player_.selected() >= 0) player_.push(pk, n, tsSnap_);
    outputs_.burstDone(secs);
}

// Auto mode alternates between the two receivers until one locks, then stays with it. Only the active receiver sees samples,
// so idle searching costs one receiver, not two.
void Engine::feedRx(const cf32* x, size_t n) { feedReceiver(activeStd_.load(), x, n); }

void Engine::feedReceiver(int a, const cf32* x, size_t n) {
    if (a == 5) rxI_.feed(x, n); else if (a == 4) rxA3_.feed(x, n); else if (a == 3) rxD_.feed(x, n); else if (a == 2) rxA_.feed(x, n); else if (a == 1) rxT_.feed(x, n); else if (a == 6) rxFm_.feed(x, n); else if (a == 7) rxDvbs_.feed(x, n); else if (a == 8) rxDtmb_.feed(x, n); else if (a == 9) rxAtv_.feed(x, n); else if (a == 10) rxDmr_.feed(x, n); else if (a == 11) rxDrm_.feed(x, n); else if (a == 12) rxAdsb_.feed(x, n); else if (a == 13) rxGnss_.feed(x, n); else if (a == 14) rxSonde_.feed(x, n); else if (a == 15) rxAis_.feed(x, n); else if (a == 16) rxMarine_.feed(x, n); else if (a == 17) rxAcars_.feed(x, n); else if (a == 18) rxInmc_.feed(x, n); else if (a == 19) rxAero_.feed(x, n); else if (a == 20) rxIridium_.feed(x, n); else if (a == 21) rxMesh_.feed(x, n); else if (a == 22) rxHdr_.feed(x, n); else if (a == 23) rxCdr_.feed(x, n); else if (a == 24) rxPager_.feed(x, n); else if (a == 25) rxPacket_.feed(x, n); else if (a == 26) rxHfdig_.feed(x, n); else if (a == 27) rxAirband_.feed(x, n); else if (a == 28) rxVdl2_.feed(x, n); else if (a == 29) rxIsm_.feed(x, n); else if (a == 31) rxVor_.feed(x, n); else if (a == 32) rxM17_.feed(x, n); else if (a == 33) rxCell_.feed(x, n); else if (a == 35) rxCw_.feed(x, n); else if (a == 34) rxDstar_.feed(x, n); else if (a == 36) rxHfdl_.feed(x, n); else if (a == 39) rxNxdn_.feed(x, n); else if (a == 46) rxDpmr_.feed(x, n); else if (a == 40) rxGsmr_.feed(x, n); else if (a == 37) rxPsk_.feed(x, n); else if (a == 38) rxJs8_.feed(x, n); else if (a == 42) rxSpecx_.feed(x, n); else if (a == 43) rxSpecan_.feed(x, n); else if (a == 41) rxDvbc_.feed(x, n); else if (a == 45) rxOrb_.feed(x, n); else if (a == 44) rxUplink_.feed(x, n); else if (a == 47) rxTetra_.feed(x, n); else if (a == 50) rxAle_.feed(x, n); else if (a == 48) rxMac_.feed(x, n); else if (a == 49) rxAstro_.feed(x, n); else if (a == 51) rxP25_.feed(x, n); else if (a == 52) rxBle_.feed(x, n); else if (a == 53) rxIono_.feed(x, n); else if (a == 54) rxQo100_.feed(x, n); else if (a == 55) rxWifi_.feed(x, n); else if (a == 56) rxGeneral_.feed(x, n); else if (a == 59) rxTime_.feed(x, n); else if (a == 57) rxRid_.feed(x, n); else if (a == 60) rxMeteor_.feed(x, n); else if (a == 58) rxNr_.feed(x, n); else if (a == 61) rxGoes_.feed(x, n); else if (a == 62) rxCube_.feed(x, n); else rx_.feed(x, n);
}

void Engine::changeBandwidth(double mhz) {
    if (mhz == bwMhz_) return;
    char b[160];
    snprintf(b, sizeof b, "bandwidth: %.4g MHz -> %.4g MHz", bwMhz_, mhz);
    log(b);
    bwMhz_ = mhz; bwActive_ = mhz;
    rx_.configure(rate_, mhz);
    rxT_.configure(rate_, mhz);
    applyReset();
}

// While nothing is locked: average the spectrum, measure the width of the flat top and switch to that channel bandwidth
void Engine::bandwidthStep(const SpectrumFrame& f, const RxTelemetry& t, bool tLocked) {
    const double now = nSamp_ / std::max(1.0, rate_.load());
    if (bwReset_.exchange(false)) { bwDet_.reset(); bwMark_ = now; bwVote_ = 0; }
    if (bwReq_.exchange(false)) { changeBandwidth(bwReqVal_); bwDet_.reset(); bwMark_ = now; }
    if (!bwAuto_ || activeStd_ >= 2) return;   // an ATSC channel is always 6 MHz, a DAB ensemble 1.536 MHz
    const bool locked = activeStd_ == 0 ? t.state == 2 : tLocked;
    if (locked) { bwDet_.reset(); bwMark_ = now; bwVote_ = 0; return; }
    bwDet_.add(f.dbfs, rate_ / 1e6);
    if (bwDet_.frames() < 12) return;
    const BandwidthEstimate e = bwDet_.estimate();
    if (e.valid) {
        bwVote_ = e.bwMhz == bwVote_ ? bwVote_ : e.bwMhz;
        if (e.bwMhz == bwVote_ && bwDet_.frames() >= 20) {
            bwDetected_ = e.bwMhz;
            if (e.bwMhz != bwMhz_) { char b[160]; snprintf(b, sizeof b, "signal is %.2f MHz wide: %.4g MHz channel", e.occupiedMhz, e.bwMhz); log(b); changeBandwidth(e.bwMhz); }
            bwDet_.reset(); bwMark_ = now;
        }
    }
    if (now - bwMark_ > 6.0) { bwDet_.reset(); bwMark_ = now; }   // keep the average fresh
}

// A fresh start of the receiver of active standard a (the first mode switching to it, the second mode starting)
void Engine::resetReceiver(int a) {
    switch (a) {
    case 0: rx_.reset(); break;
    case 1: rxT_.reset(); break;
    case 2: rxA_.reset(); break;
    case 3: rxD_.reset(); break;
    case 4: rxA3_.reset(); break;
    case 5: rxI_.reset(); break;
    case 6: rxFm_.reset(); break;
    case 7: rxDvbs_.reset(); break;
    case 8: rxDtmb_.reset(); break;
    case 9: rxAtv_.reset(); break;
    case 10: rxDmr_.reset(); break;
    case 11: rxDrm_.reset(); break;
    case 12: rxAdsb_.reset(); break;
    case 13: rxGnss_.reset(); break;
    case 14: rxSonde_.reset(); break;
    case 15: rxAis_.reset(); break;
    case 16: rxMarine_.reset(); break;
    case 17: rxAcars_.reset(); break;
    case 18: rxInmc_.reset(); break;
    case 19: rxAero_.reset(); break;
    case 20: rxIridium_.reset(); break;
    case 21: rxMesh_.reset(); break;
    case 22: rxHdr_.reset(); break;
    case 23: rxCdr_.reset(); break;
    case 24: rxPager_.reset(); break;
    case 25: rxPacket_.reset(); break;
    case 26: rxHfdig_.reset(); break;
    case 27: rxAirband_.reset(); break;
    case 28: rxVdl2_.reset(); break;
    case 29: rxIsm_.reset(); break;
    case 31: rxVor_.reset(); break;
    case 32: rxM17_.reset(); break;
    case 36: rxHfdl_.reset(); break;
    case 33: rxCell_.reset(); break;
    case 35: rxCw_.reset(); break;
    case 34: rxDstar_.reset(); break;
    case 39: rxNxdn_.reset(); break;
    case 46: rxDpmr_.reset(); break;
    case 48: rxMac_.reset(); break;
    case 51: rxP25_.reset(); break;
    case 40: rxGsmr_.reset(); break;
    case 47: rxTetra_.reset(); break;
    case 37: rxPsk_.reset(); break;
    case 38: rxJs8_.reset(); break;
    case 42: rxSpecx_.reset(); break;
    case 43: rxSpecan_.reset(); break;
    case 49: rxAstro_.reset(); break;
    case 41: rxDvbc_.reset(); break;
    case 45: rxOrb_.reset(); break;
    case 44: rxUplink_.reset(); break;
    case 50: rxAle_.reset(); break;
    case 52: rxBle_.reset(); break;
    case 53: rxIono_.reset(); break;
    case 54: rxQo100_.reset(); break;
    case 55: rxWifi_.reset(); break;
    case 56: rxGeneral_.reset(); break;
    case 59: rxTime_.reset(); break;
    case 62: rxCube_.reset(); break;
    case 61: rxGoes_.reset(); break;
    case 57: rxRid_.reset(); break;
    case 60: rxMeteor_.reset(); break;
    case 58: rxNr_.reset(); break;
    default: break;
    }
}

void Engine::autoSelect(const RxTelemetry& t, bool tLocked) {
    const double now = nSamp_ / std::max(1.0, rate_.load());
    if (stdReq_.exchange(false)) {
        const int m = stdMode_.load();
        if (m >= 1 && activeStd_ != activeStdFor(m)) {
            if (dmbSub_ >= 0) { player_.select(-1); dmbSub_ = -1; }   // leaving DAB: the DMB picture stops with it
            activeStd_ = activeStdFor(m);
            // the second mode on this receiver stops first (secondaryLoop() checks secAct_ under secFeedMu_)
            if (secAct_.load() == activeStd_.load()) {
                std::lock_guard<std::mutex> fl(secFeedMu_);
                secAct_ = -1;
                setSecNote("suspended: the first mode is now the same receiver");
                configureReceiver(activeStd_.load(), rate_.load());
                placeReceiverAsPrimary(activeStd_.load());
            }
            resetReceiver(activeStd_.load());
        }
        autoMark_ = lastLockSec_ = now;
        return;
    }
    if (stdMode_.load() != 0) return;
    const bool locked = activeStd_ == 0 ? t.state == 2 : tLocked;
    if (locked) { lastLockSec_ = now; autoMark_ = now; return; }
    const double slice = activeStd_ == 0 ? 1.6 : 1.3;
    if (now - autoMark_ < slice) return;
    if (activeStd_ == 0) { activeStd_ = 1; rxT_.reset(); log("auto: trying DVB-T"); }
    else { activeStd_ = 0; rx_.reset(); log("auto: trying DVB-T2"); }
    autoMark_ = now;
}

void Engine::logDabEvents(const RxTelemetry& t) {
    const DabTelemetry& d = t.dab;
    if (d.state != logDState_) {
        if (d.state == 2) { char b[120]; snprintf(b, sizeof b, "DAB: frame sync, carrier offset %+.0f Hz, SNR %.1f dB", d.cfoHz, d.snrDb); log(b); }
        else if (logDState_ == 2) log("DAB: sync lost");
        logDState_ = d.state;
    }
    if (d.ensemble && !logDEns_) {
        char b[160];
        snprintf(b, sizeof b, "DAB: ensemble \"%s\", %d services", d.ensembleLabel.c_str(), d.services);
        log(b);
        logDEns_ = true;
    } else if (!d.ensemble) logDEns_ = false;
}

void Engine::logAtsc3Events(const Atsc3Telemetry& a) {
    const int st = a.locked ? 2 : a.bootstraps > 0 ? 1 : 0;
    if (st != logA3State_) {
        char b[200];
        if (st == 2) snprintf(b, sizeof b, "ATSC 3.0: frames decoding, carrier offset %.0f Hz", a.cfoHz);
        else if (st == 1) snprintf(b, sizeof b, "ATSC 3.0: bootstrap found, waiting for a frame that decodes");
        else snprintf(b, sizeof b, "ATSC 3.0: searching for a bootstrap");
        log(b);
        logA3State_ = st;
    }
    const bool svc = a.serviceReady;
    if (svc != logA3Svc_) {
        logA3Svc_ = svc;
        if (svc) { char b[160]; snprintf(b, sizeof b, "ATSC 3.0: signaling of service %d received", a.selected); log(b); }
    }
}

void Engine::logAtscEvents(const RxTelemetry& t) {
    const AtscTelemetry& a = t.atsc;
    const int level = a.tsOk ? 4 : a.fieldSync ? 3 : a.segSync ? 2 : a.pilot ? 1 : 0;
    if (level != logAState_) {
        static const char* nm[] = {"ATSC: searching for the pilot and segment sync", "ATSC: pilot locked", "ATSC: segment sync, symbol clock locked", "ATSC: field sync found", "ATSC: transport stream flowing"};
        log(nm[level]);
        if (level >= 3) { char b[120]; snprintf(b, sizeof b, "ATSC: carrier offset %+.0f Hz, symbol clock %+.1f ppm, SNR %.1f dB", a.cfoHz, a.sroPpm, a.snrDb); log(b); }
        logAState_ = level;
    }
}

void Engine::logIsdbtEvents(const RxTelemetry& t) {
    char b[240];
    if (t.fftN && (t.fftN != logIMode_ || t.giIdx != logIGi_)) {
        snprintf(b, sizeof b, "ISDB-T found: mode %d (%s carrier spacing), guard interval %s, CFO %+.1f Hz", t.isdbt.mode ? t.isdbt.mode : (t.fftN == 2048 ? 1 : t.fftN == 4096 ? 2 : 3),
                 t.fftN == 2048 ? "4 kHz" : t.fftN == 4096 ? "2 kHz" : "1 kHz", isdbt::guardName(t.giIdx), t.cfoHz);
        log(b);
        logIMode_ = t.fftN; logIGi_ = t.giIdx;
    }
    const int tm = t.isdbt.tmccOk ? 1 : 0;
    if (tm != logITmcc_) {
        if (tm) {
            std::string s;
            for (int i = 0; i < 3; i++) {
                const auto& L = t.isdbt.layer[i];
                if (!L.segments) continue;
                snprintf(b, sizeof b, "%s%c: %d segment%s %s %s", s.empty() ? "" : ", ", 'A' + i, L.segments, L.segments == 1 ? "" : "s", isdbt::modName(L.mod), isdbt::rateName(L.rate));
                s += b;
            }
            snprintf(b, sizeof b, "TMCC decoded: %s%s", s.c_str(), t.isdbt.partial ? " (with a one-segment layer)" : "");
            log(b);
        } else if (logITmcc_ == 1) log("TMCC lost");
        logITmcc_ = tm;
    }
    bool sync = false;
    for (int i = 0; i < 3; i++) sync |= t.isdbt.layer[i].synced;
    if (sync != logISync_) {
        log(sync ? "transport stream locked (Viterbi + Reed-Solomon)" : "transport stream sync lost");
        logISync_ = sync;
    }
}

void Engine::logDvbtEvents(const RxTelemetry& t) {
    char b[200];
    if (t.fftN && (t.fftN != logTMode_ || t.giIdx != logTGi_)) {
        snprintf(b, sizeof b, "DVB-T found: %s, guard interval %s, CFO %+.1f Hz", t.fftN == 8192 ? "8K" : "2K", dvbt::guardName(t.giIdx), t.cfoHz);
        log(b);
        logTMode_ = t.fftN; logTGi_ = t.giIdx;
    }
    const int tps = t.dvbt.tpsOk ? 1 : 0;
    if (tps != logTTps_) {
        if (tps) {
            snprintf(b, sizeof b, "TPS decoded: %s, code rate %s, cell id %d", dvbt::modName(t.dvbt.mod), dvbt::rateName(t.dvbt.crHp), t.dvbt.cellId);
            log(b);
        } else if (logTTps_ == 1) log("TPS lost");
        logTTps_ = tps;
    }
    if (t.dvbt.fecSync != logTFec_) {
        log(t.dvbt.fecSync ? "transport stream locked (Viterbi + Reed-Solomon)" : "transport stream sync lost");
        logTFec_ = t.dvbt.fecSync;
    }
}

namespace {
// How long the analysis thread waits for, and holds, the transport-stream lock while it hands a decoded frame on (see engineWaitProfile()).
std::atomic<uint64_t> gPlpFrames{0}, gPlpWaitUs{0}, gPlpHoldUs{0}, gPlpPushUs{0};
inline uint64_t usSince(std::chrono::steady_clock::time_point t) { return (uint64_t)std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - t).count(); }
}

std::string engineWaitProfile() {
    char b[200];
    snprintf(b, sizeof b, "analysis thread, per decoded frame: lock wait %.2f ms, lock held %.2f ms (player push %.2f ms); %llu frames", 1e-3 * gPlpWaitUs / std::max<uint64_t>(1, gPlpFrames), 1e-3 * gPlpHoldUs / std::max<uint64_t>(1, gPlpFrames), 1e-3 * gPlpPushUs / std::max<uint64_t>(1, gPlpFrames), (unsigned long long)gPlpFrames.load());
    return b;
}

void Engine::onPlp(const PlpResult& r) {
    if (getenv("DECT2_FRAMESTATS")) fprintf(stderr, "[frame] t2 %d: blocks %d ok %d bad %d MER %.1f preBER %.3f iters %.1f\n", r.t2Frame, r.blocks, r.blocksOk, r.bchFailed, r.merDb, r.preBer, r.avgLdpcIters);
    const auto tWait0 = std::chrono::steady_clock::now();
    std::lock_guard<std::mutex> lk(tsMu_);
    gPlpWaitUs += usSince(tWait0);
    struct HoldTimer { std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now(); ~HoldTimer() { gPlpHoldUs += usSince(t0); gPlpFrames++; } } holdTimer;
    frameBuf_.clear();
    // FRAME_IDX restarts at 0 every super-frame (NUM_T2_FRAMES frames), and a PLP with FRAME_INTERVAL > 1 skips frames: compare the
    // index with the one expected, not with last + 1, or every super-frame boundary (every second frame on a 2-frame super-frame) looks
    // like lost data and drops the half-assembled SDT/NIT/EIT sections.
    if (lastT2Frame_ >= 0) {
        const int mod = r.numT2Frames > 0 ? r.numT2Frames : 256;
        if (r.t2Frame != (lastT2Frame_ + std::max(1, r.frameInterval)) % mod) { unpack_.lost(); demux_.markLoss(); }
    }
    lastT2Frame_ = r.t2Frame;
    for (const auto& f : r.frames) {
        if (f.bits.empty()) { unpack_.lost(); demux_.markLoss(); continue; }
        unpack_.push(f.bits, f.header);
    }
    demux_.advance(r.frameSec);
    tsSnap_ = demux_.snapshot();
    outputs_.packets(frameBuf_.data(), frameBuf_.size() / 188, &tsSnap_);
    if (tap_) tap_(frameBuf_.data(), frameBuf_.size() / 188, &tsSnap_);
    if (player_.selected() >= 0) { const auto tp = std::chrono::steady_clock::now(); player_.push(frameBuf_.data(), frameBuf_.size() / 188, tsSnap_); gPlpPushUs += usSince(tp); }
    frameBuf_.clear();
    outputs_.burstDone(r.frameSec);
}

void Engine::setPacketTap(std::function<void(const uint8_t*, size_t, const TsSnapshot*)> f) { std::lock_guard<std::mutex> lk(tsMu_); tap_ = std::move(f); }
TsSnapshot Engine::tsSnapshot() { std::lock_guard<std::mutex> lk(tsMu_); return tsSnap_; }
std::map<int, std::vector<EpgEvent>> Engine::epg() {
    std::lock_guard<std::mutex> lk(tsMu_);
    return demux_.epg();
}

BbStats Engine::bbStats() { std::lock_guard<std::mutex> lk(tsMu_); return unpack_.stats(); }

bool Engine::latestRx(RxTelemetry& out, uint64_t lastSeq) {
    std::lock_guard<std::mutex> lk(rxMu_);
    if (rxTel_.seq <= lastSeq) return false;
    out = rxTel_;
    return true;
}

void Engine::logRxEvents(const RxTelemetry& t) {
    char b[200];
    if (t.p1Count != logP1Count_ && t.p1Count > 0) {
        bool first = logP1Count_ == 0 || t.p1.s2field1 != logS2_ || t.p1.s1 != logS1_;
        if (first) {
            snprintf(b, sizeof b, "P1 found: %s, S2 FFT %s%s, CFO %+.1f Hz, conf %.2f", s1Name(t.p1.s1),
                     fftModeFromS2(t.p1.s2field1) ? fftModeFromS2(t.p1.s2field1)->name : "?", t.p1.mixed ? " (mixed)" : "",
                     t.p1.cfoHz, t.p1.conf);
            log(b);
        }
        logP1Count_ = t.p1Count;
        logS2_ = t.p1.s2field1;
        logS1_ = t.p1.s1;
    }
    if (t.giIdx != logGi_) {
        if (t.giIdx >= 0) {
            snprintf(b, sizeof b, "guard interval %s detected (margin %.2fx), %d symbols locked", guardName(t.giIdx), t.giMargin, (int)t.symbols);
            log(b);
        }
        logGi_ = t.giIdx;
    }
    if (t.state != logState_) {
        static const char* nm[] = {"searching for P1", "waiting for guard interval", "locked"};
        log(std::string("sync: ") + nm[t.state]);
        logState_ = t.state;
    }
    if (t.symbolsPerFrame != logFrameSyms_ && t.symbolsPerFrame > 0) {
        snprintf(b, sizeof b, "frame length %.2f ms = %d symbols after P1, SRO %+.1f ppm", t.frameMs, t.symbolsPerFrame, t.sroPpm);
        log(b);
        logFrameSyms_ = t.symbolsPerFrame;
    }
}

bool Engine::latestSpectrum(SpectrumFrame& out, uint64_t lastSeq) {
    std::lock_guard<std::mutex> lk(specMu_);
    if (spec_.seq <= lastSeq || spec_.dbfs.empty()) return false;
    out = spec_;
    return true;
}

// A radio that was delivering samples and stops (the cable was pulled) is reported, and a HackRF is opened again with the settings in use
// as soon as it is back, so that the receiver carries on by itself.
void Engine::watchRadio() {
    if (!src_ || !src_->realtimeHardware()) return;
    using namespace std::chrono;
    const auto now = steady_clock::now();
    if (!radioLost_) {
        if (now - lastSamples_ > seconds(2)) {
            radioLost_ = true;
            nextReconnect_ = now + seconds(1);
            log(lastDev_.kind == DeviceInfo::HackRF ? "radio stopped sending samples (unplugged?) - waiting for it to come back" : "radio stopped sending samples - press Stop and Start to open it again");
        }
        return;
    }
    if (lastDev_.kind != DeviceInfo::HackRF || now < nextReconnect_) return;
    nextReconnect_ = now + seconds(2);
    TuneSettings tune;
    { std::lock_guard<std::mutex> lk(tuneMu_); tune = lastTune_; }
    src_->stop();
    std::string err;
    if (src_->start(radioTune(tune), ring_, err)) {
        radioLost_ = false;
        lastSamples_ = steady_clock::now();
        reconnectErr_.clear();
        resetReq_ = true;   // the receiver starts again from the new samples
        log("radio reconnected");
    } else if (err != reconnectErr_) {
        reconnectErr_ = err;
        log("radio not available yet: " + err);
    }
}

// The receiver cannot keep up (the ring is more than half backlog): jump to the newest samples in one go. One clean gap that the receiver
// resynchronises after is far better than the radio thread dropping a few samples out of every chunk, which ruins every frame.
// Only for live radios: a recording or the synthetic signal waits for a slow receiver instead (see the paced source), and must never lose samples.
void Engine::catchUp() {
    if (!src_ || !src_->realtimeHardware()) return;
    const size_t skipped = ring_.dropBacklog();
    if (!skipped) return;
    if (activeStd_.load() == 0) rx_.markGap(skipped);
    skippedSamples_ += skipped;
    skipEvents_++;
    const auto now = std::chrono::steady_clock::now();
    if (now - lastSkipLog_ > std::chrono::seconds(10)) {
        lastSkipLog_ = now;
        char b[200];
        snprintf(b, sizeof b, "the receiver cannot keep up with the sample rate: skipped %.2f s of signal (%llu skips so far). Close other programs, use a narrower channel or a lower sample rate", skipped / std::max(1.0, srcRate_), (unsigned long long)skipEvents_);
        log(b);
    }
}

static int64_t steadyNs() { return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count(); }

void Engine::holdDelivery(int ms) {
    delivHoldNs_ = steadyNs() + (int64_t)ms * 1000000;
    delivReanchor_ = true;
}

// Samples that never came out of the radio (delivery_check.h)
void Engine::checkDelivery() {
    if (!src_ || !src_->realtimeHardware() || srcRate_ <= 0) return;
    if (radioLost_) { holdDelivery(500); return; }   // unplugged: that is reported as such, and the gap is not a loss of the USB link
    uint64_t off; int64_t ns; size_t block;
    if (!ring_.lastWrite(off, ns, block)) return;
    if (delivReanchor_.exchange(false)) deliv_.restart(delivHoldNs_.load());
    const double step = deliv_.observe(off, ns, srcRate_, (double)block);
    if (step > 0) {
        radioMissing_ = radioMissing_.load() + step;
        radioMissEvents_++;
        const int64_t now = steadyNs();
        lastRadioMissNs_ = now;
        if (now - lastRadioLogNs_.load() > 10000000000LL) {
            lastRadioLogNs_ = now;
            char b[240];
            snprintf(b, sizeof b, "the radio did not deliver %.0f ms of signal (%.2f s in all): its USB connection or driver could not keep up. Try another USB port (not a hub), a shorter cable or a lower sample rate",
                     1000 * step / srcRate_, radioMissing_.load() / srcRate_);
            log(b);
        }
    }
}

SampleLoss Engine::sampleLoss() const {
    SampleLoss l;
    l.live = live_.load();
    const double r = std::max(1.0, srcRate_);   // the ring holds the radio's samples (before any resampling)
    l.radioSec = radioMissing_.load() / r;
    l.cpuSec = (double)ring_.dropped() / r;
    l.radioEvents = radioMissEvents_.load();
    l.cpuEvents = cpuDropEvents_.load();
    const int64_t now = steadyNs();
    if (lastRadioMissNs_.load()) l.sinceRadioSec = (double)(now - lastRadioMissNs_.load()) * 1e-9;
    if (lastCpuDropNs_.load()) l.sinceCpuSec = (double)(now - lastCpuDropNs_.load()) * 1e-9;
    l.loadPct = loadPct_.load();
    return l;
}

void Engine::feedSpectrum(const cf32* x, size_t n) {
    {
        std::lock_guard<std::mutex> lk(specQMu_);
        if (specQ_.size() >= 6) return;   // the display is statistical: when the spectrum thread is behind, skip a piece
        specQ_.emplace_back(x, x + n);
    }
    specQCv_.notify_one();
}

void Engine::spectrumLoop() {
    std::vector<cf32> chunk;
    for (;;) {
        {
            std::unique_lock<std::mutex> lk(specQMu_);
            specQCv_.wait(lk, [this] { return specStop_ || !specQ_.empty(); });
            if (specQ_.empty()) return;   // stopping
            chunk = std::move(specQ_.front());
            specQ_.pop_front();
        }
        const auto a0 = std::chrono::steady_clock::now();
        analyzer_.feed(chunk.data(), chunk.size());
        tSpec_ += std::chrono::duration<double>(std::chrono::steady_clock::now() - a0).count();
    }
}

// The report of the receivers from DVB-S on (the first mode and the second, each with its own report numbers): 1 = a new one in t,
// 0 = none new, -1 = a is not one of them
int Engine::pollMode(int a, RxTelemetry& t, uint64_t* seqs) {
    if (a == 7) {
        DvbsTelemetry mt;
        if (rxDvbs_.telemetry(mt, seqs[0])) {
            seqs[0] = mt.seq;
            t.standard = 7;
            t.seq = mt.seq;
            t.state = mt.state;
            t.cfoHz = mt.cfoHz;
            t.dataValid = mt.dataValid;
            t.dataSnrDb = mt.snrDb;
            t.blocksOk = mt.blocksOk; t.blocksBad = mt.blocksBad;
            t.rateOk = rxDvbs_.ready();
            t.dvbs = std::move(mt);
            return 1;
        }
    } else if (a == 8) {
        DtmbTelemetry mt;
        if (rxDtmb_.telemetry(mt, seqs[1])) {
            seqs[1] = mt.seq;
            t.standard = 8;
            t.seq = mt.seq;
            t.state = mt.state;
            t.cfoHz = mt.cfoHz;
            t.dataValid = mt.dataValid;
            t.dataSnrDb = mt.snrDb;
            t.blocksOk = mt.blocksOk; t.blocksBad = mt.blocksBad;
            t.rateOk = rxDtmb_.ready();
            t.dtmb = std::move(mt);
            return 1;
        }
    } else if (a == 9) {
        AtvTelemetry mt;
        if (rxAtv_.telemetry(mt, seqs[2])) {
            seqs[2] = mt.seq;
            t.standard = 9;
            t.seq = mt.seq;
            t.state = mt.state;
            t.cfoHz = mt.cfoHz;
            t.dataValid = mt.dataValid;
            t.dataSnrDb = mt.snrDb;
            t.blocksOk = mt.blocksOk; t.blocksBad = mt.blocksBad;
            t.rateOk = rxAtv_.ready();
            t.atv = std::move(mt);
            return 1;
        }
    } else if (a == 10) {
        DmrTelemetry mt;
        if (rxDmr_.telemetry(mt, seqs[3])) {
            seqs[3] = mt.seq;
            t.standard = 10;
            t.seq = mt.seq;
            t.state = mt.state;
            t.cfoHz = mt.cfoHz;
            t.dataValid = mt.dataValid;
            t.dataSnrDb = mt.snrDb;
            t.blocksOk = mt.blocksOk; t.blocksBad = mt.blocksBad;
            t.rateOk = rxDmr_.ready();
            t.dmr = std::move(mt);
            return 1;
        }
    } else if (a == 11) {
        DrmTelemetry mt;
        if (rxDrm_.telemetry(mt, seqs[4])) {
            seqs[4] = mt.seq;
            t.standard = 11;
            t.seq = mt.seq;
            t.state = mt.state;
            t.cfoHz = mt.cfoHz;
            t.dataValid = mt.dataValid;
            t.dataSnrDb = mt.snrDb;
            t.blocksOk = mt.blocksOk; t.blocksBad = mt.blocksBad;
            t.rateOk = rxDrm_.ready();
            t.drm = std::move(mt);
            return 1;
        }
    } else if (a == 12) {
        AdsbTelemetry mt;
        if (rxAdsb_.telemetry(mt, seqs[5])) {
            seqs[5] = mt.seq;
            t.standard = 12;
            t.seq = mt.seq;
            t.state = mt.state;
            t.cfoHz = mt.cfoHz;
            t.dataValid = mt.dataValid;
            t.dataSnrDb = mt.snrDb;
            t.blocksOk = mt.blocksOk; t.blocksBad = mt.blocksBad;
            t.rateOk = rxAdsb_.ready();
            t.adsb = std::move(mt);
            return 1;
        }
    } else if (a == 13) {
        GnssTelemetry mt;
        if (rxGnss_.telemetry(mt, seqs[6])) {
            seqs[6] = mt.seq;
            t.standard = 13;
            t.seq = mt.seq;
            t.state = mt.state;
            t.cfoHz = mt.cfoHz;
            t.dataValid = mt.dataValid;
            t.dataSnrDb = mt.snrDb;
            t.blocksOk = mt.blocksOk; t.blocksBad = mt.blocksBad;
            t.rateOk = rxGnss_.ready();
            t.gnss = std::move(mt);
            return 1;
        }
    } else if (a == 14) {
        SondeTelemetry mt;
        if (rxSonde_.telemetry(mt, seqs[7])) {
            seqs[7] = mt.seq;
            t.standard = 14;
            t.seq = mt.seq;
            t.state = mt.state;
            t.cfoHz = mt.cfoHz;
            t.dataValid = mt.dataValid;
            t.dataSnrDb = mt.snrDb;
            t.blocksOk = mt.blocksOk; t.blocksBad = mt.blocksBad;
            t.rateOk = rxSonde_.ready();
            t.sonde = std::move(mt);
            return 1;
        }
    } else if (a == 15) {
        AisTelemetry mt;
        if (rxAis_.telemetry(mt, seqs[8])) {
            seqs[8] = mt.seq;
            t.standard = 15;
            t.seq = mt.seq;
            t.state = mt.state;
            t.cfoHz = mt.cfoHz;
            t.dataValid = mt.dataValid;
            t.dataSnrDb = mt.snrDb;
            t.blocksOk = mt.blocksOk; t.blocksBad = mt.blocksBad;
            t.rateOk = rxAis_.ready();
            t.ais = std::move(mt);
            return 1;
        }
    } else if (a == 16) {
        MarineTelemetry mt;
        if (rxMarine_.telemetry(mt, seqs[9])) {
            seqs[9] = mt.seq;
            t.standard = 16;
            t.seq = mt.seq;
            t.state = mt.state;
            t.cfoHz = mt.cfoHz;
            t.dataValid = mt.dataValid;
            t.dataSnrDb = mt.snrDb;
            t.blocksOk = mt.blocksOk; t.blocksBad = mt.blocksBad;
            t.rateOk = rxMarine_.ready();
            t.marine = std::move(mt);
            return 1;
        }
    } else if (a == 17) {
        AcarsTelemetry mt;
        if (rxAcars_.telemetry(mt, seqs[10])) {
            seqs[10] = mt.seq;
            t.standard = 17;
            t.seq = mt.seq;
            t.state = mt.state;
            t.cfoHz = mt.cfoHz;
            t.dataValid = mt.dataValid;
            t.dataSnrDb = mt.snrDb;
            t.blocksOk = mt.blocksOk; t.blocksBad = mt.blocksBad;
            t.rateOk = rxAcars_.ready();
            t.acars = std::move(mt);
            return 1;
        }
    } else if (a == 18) {
        InmcTelemetry mt;
        if (rxInmc_.telemetry(mt, seqs[11])) {
            seqs[11] = mt.seq;
            t.standard = 18;
            t.seq = mt.seq;
            t.state = mt.state;
            t.cfoHz = mt.cfoHz;
            t.dataValid = mt.dataValid;
            t.dataSnrDb = mt.snrDb;
            t.blocksOk = mt.blocksOk; t.blocksBad = mt.blocksBad;
            t.rateOk = rxInmc_.ready();
            t.inmc = std::move(mt);
            return 1;
        }
    } else if (a == 19) {
        AeroTelemetry mt;
        if (rxAero_.telemetry(mt, seqs[12])) {
            seqs[12] = mt.seq;
            t.standard = 19;
            t.seq = mt.seq;
            t.state = mt.state;
            t.cfoHz = mt.cfoHz;
            t.dataValid = mt.dataValid;
            t.dataSnrDb = mt.snrDb;
            t.blocksOk = mt.blocksOk; t.blocksBad = mt.blocksBad;
            t.rateOk = rxAero_.ready();
            t.aero = std::move(mt);
            return 1;
        }
    } else if (a == 20) {
        IridiumTelemetry mt;
        if (rxIridium_.telemetry(mt, seqs[13])) {
            seqs[13] = mt.seq;
            t.standard = 20;
            t.seq = mt.seq;
            t.state = mt.state;
            t.cfoHz = mt.cfoHz;
            t.dataValid = mt.dataValid;
            t.dataSnrDb = mt.snrDb;
            t.blocksOk = mt.blocksOk; t.blocksBad = mt.blocksBad;
            t.rateOk = rxIridium_.ready();
            t.iridium = std::move(mt);
            return 1;
        }
    } else if (a == 21) {
        MeshTelemetry mt;
        if (rxMesh_.telemetry(mt, seqs[14])) {
            seqs[14] = mt.seq;
            t.standard = 21;
            t.seq = mt.seq;
            t.state = mt.state;
            t.cfoHz = mt.cfoHz;
            t.dataValid = mt.dataValid;
            t.dataSnrDb = mt.snrDb;
            t.blocksOk = mt.blocksOk; t.blocksBad = mt.blocksBad;
            t.rateOk = rxMesh_.ready();
            t.mesh = std::move(mt);
            return 1;
        }
    } else if (a == 22) {
        HdrTelemetry mt;
        if (rxHdr_.telemetry(mt, seqs[15])) {
            seqs[15] = mt.seq;
            t.standard = 22;
            t.seq = mt.seq;
            t.state = mt.state;
            t.cfoHz = mt.cfoHz;
            t.dataValid = mt.dataValid;
            t.dataSnrDb = mt.snrDb;
            t.blocksOk = mt.blocksOk; t.blocksBad = mt.blocksBad;
            t.rateOk = rxHdr_.ready();
            t.hdr = std::move(mt);
            return 1;
        }
    } else if (a == 23) {
        CdrTelemetry mt;
        if (rxCdr_.telemetry(mt, seqs[16])) {
            seqs[16] = mt.seq;
            t.standard = 23;
            t.seq = mt.seq;
            t.state = mt.state;
            t.cfoHz = mt.cfoHz;
            t.dataValid = mt.dataValid;
            t.dataSnrDb = mt.snrDb;
            t.blocksOk = mt.blocksOk; t.blocksBad = mt.blocksBad;
            t.rateOk = rxCdr_.ready();
            t.cdr = std::move(mt);
            return 1;
        }
    } else if (a == 24) {
        PagerTelemetry mt;
        if (rxPager_.telemetry(mt, seqs[17])) {
            seqs[17] = mt.seq;
            t.standard = 24;
            t.seq = mt.seq;
            t.state = mt.state;
            t.cfoHz = mt.cfoHz;
            t.dataValid = mt.dataValid;
            t.dataSnrDb = mt.snrDb;
            t.blocksOk = mt.blocksOk; t.blocksBad = mt.blocksBad;
            t.rateOk = rxPager_.ready();
            t.pager = std::move(mt);
            return 1;
        }
    } else if (a == 25) {
        PacketTelemetry mt;
        if (rxPacket_.telemetry(mt, seqs[18])) {
            seqs[18] = mt.seq;
            t.standard = 25;
            t.seq = mt.seq;
            t.state = mt.state;
            t.cfoHz = mt.cfoHz;
            t.dataValid = mt.dataValid;
            t.dataSnrDb = mt.snrDb;
            t.blocksOk = mt.blocksOk; t.blocksBad = mt.blocksBad;
            t.rateOk = rxPacket_.ready();
            t.packet = std::move(mt);
            return 1;
        }
    } else if (a == 26) {
        HfdigTelemetry mt;
        if (rxHfdig_.telemetry(mt, seqs[19])) {
            seqs[19] = mt.seq;
            t.standard = 26;
            t.seq = mt.seq;
            t.state = mt.state;
            t.cfoHz = mt.cfoHz;
            t.dataValid = mt.dataValid;
            t.dataSnrDb = mt.snrDb;
            t.blocksOk = mt.blocksOk; t.blocksBad = mt.blocksBad;
            t.rateOk = rxHfdig_.ready();
            t.hfdig = std::move(mt);
            return 1;
        }
    } else if (a == 27) {
        AirbandTelemetry mt;
        if (rxAirband_.telemetry(mt, seqs[20])) {
            seqs[20] = mt.seq;
            t.standard = 27;
            t.seq = mt.seq;
            t.state = mt.state;
            t.cfoHz = mt.cfoHz;
            t.dataValid = mt.dataValid;
            t.dataSnrDb = mt.snrDb;
            t.blocksOk = mt.blocksOk; t.blocksBad = mt.blocksBad;
            t.rateOk = rxAirband_.ready();
            t.airband = std::move(mt);
            return 1;
        }
    } else if (a == 28) {
        Vdl2Telemetry mt;
        if (rxVdl2_.telemetry(mt, seqs[21])) {
            seqs[21] = mt.seq;
            t.standard = 28;
            t.seq = mt.seq;
            t.state = mt.state;
            t.cfoHz = mt.cfoHz;
            t.dataValid = mt.dataValid;
            t.dataSnrDb = mt.snrDb;
            t.blocksOk = mt.blocksOk; t.blocksBad = mt.blocksBad;
            t.rateOk = rxVdl2_.ready();
            t.vdl2 = std::move(mt);
            return 1;
        }
    } else if (a == 29) {
        IsmTelemetry mt;
        if (rxIsm_.telemetry(mt, seqs[22])) {
            seqs[22] = mt.seq;
            t.standard = 29;
            t.seq = mt.seq;
            t.state = mt.state;
            t.cfoHz = mt.cfoHz;
            t.dataValid = mt.dataValid;
            t.dataSnrDb = mt.snrDb;
            t.blocksOk = mt.blocksOk; t.blocksBad = mt.blocksBad;
            t.rateOk = rxIsm_.ready();
            t.ism = std::move(mt);
            return 1;
        }
    } else if (a == 31) {
        VorTelemetry mt;
        if (rxVor_.telemetry(mt, seqs[24])) {
            seqs[24] = mt.seq;
            t.standard = 31;
            t.seq = mt.seq;
            t.state = mt.state;
            t.cfoHz = mt.cfoHz;
            t.dataValid = mt.dataValid;
            t.dataSnrDb = mt.snrDb;
            t.blocksOk = mt.blocksOk; t.blocksBad = mt.blocksBad;
            t.rateOk = rxVor_.ready();
            t.vor = std::move(mt);
            return 1;
        }
    } else if (a == 32) {
        M17Telemetry mt;
        if (rxM17_.telemetry(mt, seqs[25])) {
            seqs[25] = mt.seq;
            t.standard = 32;
            t.seq = mt.seq;
            t.state = mt.state;
            t.cfoHz = mt.cfoHz;
            t.dataValid = mt.dataValid;
            t.dataSnrDb = mt.snrDb;
            t.blocksOk = mt.blocksOk; t.blocksBad = mt.blocksBad;
            t.rateOk = rxM17_.ready();
            t.m17 = std::move(mt);
            return 1;
        }
    } else if (a == 33) {
        CellTelemetry mt;
        if (rxCell_.telemetry(mt, seqs[26])) {
            seqs[26] = mt.seq;
            t.standard = 33;
            t.seq = mt.seq;
            t.state = mt.state;
            t.cfoHz = mt.cfoHz;
            t.dataValid = mt.dataValid;
            t.dataSnrDb = mt.snrDb;
            t.blocksOk = mt.blocksOk; t.blocksBad = mt.blocksBad;
            t.rateOk = rxCell_.ready();
            t.cell = std::move(mt);
            return 1;
        }
    } else if (a == 35) {
        CwTelemetry mt;
        if (rxCw_.telemetry(mt, seqs[28])) {
            seqs[28] = mt.seq;
            t.standard = 35;
            t.seq = mt.seq;
            t.state = mt.state;
            t.cfoHz = mt.cfoHz;
            t.dataValid = mt.dataValid;
            t.dataSnrDb = mt.snrDb;
            t.blocksOk = mt.blocksOk; t.blocksBad = mt.blocksBad;
            t.rateOk = rxCw_.ready();
            t.cw = std::move(mt);
            return 1;
        }
    } else if (a == 39) {
        NxdnTelemetry mt;
        if (rxNxdn_.telemetry(mt, seqs[32])) {
            seqs[32] = mt.seq;
            t.standard = 39;
            t.seq = mt.seq;
            t.state = mt.state;
            t.cfoHz = mt.cfoHz;
            t.dataValid = mt.dataValid;
            t.dataSnrDb = mt.snrDb;
            t.blocksOk = mt.blocksOk; t.blocksBad = mt.blocksBad;
            t.rateOk = rxNxdn_.ready();
            t.nxdn = std::move(mt);
            return 1;
        }
    } else if (a == 34) {
        DstarTelemetry mt;
        if (rxDstar_.telemetry(mt, seqs[27])) {
            seqs[27] = mt.seq;
            t.standard = 34;
            t.seq = mt.seq;
            t.state = mt.state;
            t.cfoHz = mt.cfoHz;
            t.dataValid = mt.dataValid;
            t.dataSnrDb = mt.snrDb;
            t.blocksOk = mt.blocksOk; t.blocksBad = mt.blocksBad;
            t.rateOk = rxDstar_.ready();
            t.dstar = std::move(mt);
            return 1;
        }
    } else if (a == 36) {
        HfdlTelemetry mt;
        if (rxHfdl_.telemetry(mt, seqs[29])) {
            seqs[29] = mt.seq;
            t.standard = 36;
            t.seq = mt.seq;
            t.state = mt.state;
            t.cfoHz = mt.cfoHz;
            t.dataValid = mt.dataValid;
            t.dataSnrDb = mt.snrDb;
            t.blocksOk = mt.blocksOk; t.blocksBad = mt.blocksBad;
            t.rateOk = rxHfdl_.ready();
            t.hfdl = std::move(mt);
            return 1;
        }
    } else if (a == 40) {
        GsmrTelemetry mt;
        if (rxGsmr_.telemetry(mt, seqs[34])) {
            seqs[34] = mt.seq;
            t.standard = 40;
            t.seq = mt.seq;
            t.state = mt.state;
            t.cfoHz = mt.cfoHz;
            t.dataValid = mt.dataValid;
            t.dataSnrDb = mt.snrDb;
            t.blocksOk = mt.blocksOk; t.blocksBad = mt.blocksBad;
            t.rateOk = rxGsmr_.ready();
            t.gsmr = std::move(mt);
            return 1;
        }
    } else if (a == 37) {
        PskTelemetry mt;
        if (rxPsk_.telemetry(mt, seqs[30])) {
            seqs[30] = mt.seq;
            t.standard = 37;
            t.seq = mt.seq;
            t.state = mt.state;
            t.cfoHz = mt.cfoHz;
            t.dataValid = mt.dataValid;
            t.dataSnrDb = mt.snrDb;
            t.blocksOk = mt.blocksOk; t.blocksBad = mt.blocksBad;
            t.rateOk = rxPsk_.ready();
            t.psk = std::move(mt);
            return 1;
        }
    } else if (a == 42) {
        SpecxTelemetry mt;
        if (rxSpecx_.telemetry(mt, seqs[36])) {
            seqs[36] = mt.seq;
            t.standard = 42;
            t.seq = mt.seq;
            t.state = mt.state;
            t.rateOk = rxSpecx_.ready();
            t.specx = std::move(mt);
            return 1;
        }
    } else if (a == 38) {
        Js8Telemetry mt;
        if (rxJs8_.telemetry(mt, seqs[31])) {
            seqs[31] = mt.seq;
            t.standard = 38;
            t.seq = mt.seq;
            t.state = mt.state;
            t.cfoHz = mt.cfoHz;
            t.dataValid = mt.dataValid;
            t.dataSnrDb = mt.snrDb;
            t.blocksOk = mt.blocksOk; t.blocksBad = mt.blocksBad;
            t.rateOk = rxJs8_.ready();
            t.js8 = std::move(mt);
            return 1;
        }
    } else if (a == 49) {
        AstroTelemetry mt;
        if (rxAstro_.telemetry(mt, seqs[43])) {
            seqs[43] = mt.seq;
            t.standard = 49;
            t.seq = mt.seq;
            t.state = mt.state;
            t.cfoHz = 0;
            t.rateOk = rxAstro_.ready();
            t.astro = std::move(mt);
            return 1;
        }
    } else if (a == 43) {
        SpecanTelemetry mt;
        if (rxSpecan_.telemetry(mt, seqs[37])) {
            seqs[37] = mt.seq;
            t.standard = 43;
            t.seq = mt.seq;
            t.state = mt.state;
            t.cfoHz = 0;
            t.dataValid = mt.dataValid;
            t.blocksOk = mt.blocksOk; t.blocksBad = mt.blocksBad;
            t.rateOk = rxSpecan_.ready();
            t.specan = std::move(mt);
            return 1;
        }
    } else if (a == 48) {
        MacTelemetry mt;
        if (rxMac_.telemetry(mt, seqs[42])) {
            seqs[42] = mt.seq;
            t.standard = 48;
            t.seq = mt.seq;
            t.state = mt.state;
            t.cfoHz = mt.cfoHz;
            t.dataValid = mt.dataValid;
            t.dataSnrDb = mt.snrDb;
            t.blocksOk = mt.blocksOk; t.blocksBad = mt.blocksBad;
            t.rateOk = rxMac_.ready();
            t.mac = std::move(mt);
            return 1;
        }
    } else if (a == 46) {
        DpmrTelemetry mt;
        if (rxDpmr_.telemetry(mt, seqs[40])) {
            seqs[40] = mt.seq;
            t.standard = 46;
            t.seq = mt.seq;
            t.state = mt.state;
            t.cfoHz = mt.cfoHz;
            t.dataValid = mt.dataValid;
            t.dataSnrDb = mt.snrDb;
            t.blocksOk = mt.blocksOk; t.blocksBad = mt.blocksBad;
            t.rateOk = rxDpmr_.ready();
            t.dpmr = std::move(mt);
            return 1;
        }
    } else if (a == 41) {
        DvbcTelemetry mt;
        if (rxDvbc_.telemetry(mt, seqs[35])) {
            seqs[35] = mt.seq;
            t.standard = 41;
            t.seq = mt.seq;
            t.state = mt.state;
            t.cfoHz = mt.cfoHz;
            t.dataValid = mt.dataValid;
            t.dataSnrDb = mt.snrDb;
            t.blocksOk = mt.blocksOk; t.blocksBad = mt.blocksBad;
            t.rateOk = rxDvbc_.ready();
            t.dvbc = std::move(mt);
            return 1;
        }
    } else if (a == 45) {
        OrbcommTelemetry mt;
        if (rxOrb_.telemetry(mt, seqs[39])) {
            seqs[39] = mt.seq;
            t.standard = 45;
            t.seq = mt.seq;
            t.state = mt.state;
            t.cfoHz = mt.cfoHz;
            t.dataValid = mt.dataValid;
            t.dataSnrDb = mt.snrDb;
            t.blocksOk = mt.blocksOk; t.blocksBad = mt.blocksBad;
            t.rateOk = rxOrb_.ready();
            t.orbcomm = std::move(mt);
            return 1;
        }
    } else if (a == 47) {
        TetraTelemetry mt;
        if (rxTetra_.telemetry(mt, seqs[41])) {
            seqs[41] = mt.seq;
            t.standard = 47;
            t.seq = mt.seq;
            t.state = mt.state;
            t.cfoHz = mt.cfoHz;
            t.dataValid = mt.dataValid;
            t.dataSnrDb = mt.snrDb;
            t.blocksOk = mt.blocksOk; t.blocksBad = mt.blocksBad;
            t.rateOk = rxTetra_.ready();
            t.tetra = std::move(mt);
            return 1;
        }
    } else if (a == 44) {
        UplinkTelemetry mt;
        if (rxUplink_.telemetry(mt, seqs[38])) {
            seqs[38] = mt.seq;
            t.standard = 44;
            t.seq = mt.seq;
            t.state = mt.state;
            t.cfoHz = mt.cfoHz;
            t.dataValid = mt.dataValid;
            t.dataSnrDb = mt.snrDb;
            t.blocksOk = mt.blocksOk; t.blocksBad = mt.blocksBad;
            t.rateOk = rxUplink_.ready();
            t.uplink = std::move(mt);
            return 1;
        }
    } else if (a == 50) {
        AleTelemetry mt;
        if (rxAle_.telemetry(mt, seqs[44])) {
            seqs[44] = mt.seq;
            t.standard = 50;
            t.seq = mt.seq;
            t.state = mt.state;
            t.cfoHz = mt.cfoHz;
            t.dataValid = mt.dataValid;
            t.dataSnrDb = mt.snrDb;
            t.blocksOk = mt.blocksOk; t.blocksBad = mt.blocksBad;
            t.rateOk = rxAle_.ready();
            t.ale = std::move(mt);
            return 1;
        }
    } else if (a == 51) {
        P25Telemetry mt;
        if (rxP25_.telemetry(mt, seqs[45])) {
            seqs[45] = mt.seq;
            t.standard = 51;
            t.seq = mt.seq;
            t.state = mt.state;
            t.cfoHz = mt.cfoHz;
            t.dataValid = mt.dataValid;
            t.dataSnrDb = mt.snrDb;
            t.blocksOk = mt.blocksOk; t.blocksBad = mt.blocksBad;
            t.rateOk = rxP25_.ready();
            t.p25 = std::move(mt);
            return 1;
        }
    } else if (a == 52) {
        BleTelemetry mt;
        if (rxBle_.telemetry(mt, seqs[46])) {
            seqs[46] = mt.seq;
            t.standard = 52;
            t.seq = mt.seq;
            t.state = mt.state;
            t.cfoHz = mt.cfoHz;
            t.dataValid = mt.dataValid;
            t.dataSnrDb = mt.snrDb;
            t.blocksOk = mt.blocksOk; t.blocksBad = mt.blocksBad;
            t.rateOk = rxBle_.ready();
            t.ble = std::move(mt);
            return 1;
        }
    } else if (a == 53) {
        IonoTelemetry mt;
        if (rxIono_.telemetry(mt, seqs[47])) {
            seqs[47] = mt.seq;
            t.standard = 53;
            t.seq = mt.seq;
            t.state = mt.state;
            t.cfoHz = 0;
            t.rateOk = rxIono_.ready();
            t.iono = std::move(mt);
            return 1;
        }
    } else if (a == 54) {
        Qo100Telemetry mt;
        if (rxQo100_.telemetry(mt, seqs[48])) {
            seqs[48] = mt.seq;
            t.standard = 54;
            t.seq = mt.seq;
            t.state = mt.state;
            t.cfoHz = mt.corrValid ? mt.lnbErrHz : 0;
            t.rateOk = rxQo100_.ready();
            t.qo100 = std::move(mt);
            return 1;
        }
    } else if (a == 55) {
        WifiTelemetry mt;
        if (rxWifi_.telemetry(mt, seqs[49])) {
            seqs[49] = mt.seq;
            t.standard = 55;
            t.seq = mt.seq;
            t.state = mt.state;
            t.cfoHz = mt.cfoHz;
            t.dataValid = mt.dataValid;
            t.dataSnrDb = mt.snrDb;
            t.blocksOk = mt.blocksOk; t.blocksBad = mt.blocksBad;
            t.rateOk = rxWifi_.ready();
            t.wifi = std::move(mt);
            return 1;
        }
    } else if (a == 56) {
        GeneralTelemetry mt;
        if (rxGeneral_.telemetry(mt, seqs[50])) {
            seqs[50] = mt.seq;
            t.standard = 56;
            t.seq = mt.seq;
            t.state = mt.state;
            t.cfoHz = mt.cfoHz;
            t.dataValid = mt.dataValid;
            t.dataSnrDb = mt.snrDb;
            t.blocksOk = mt.blocksOk; t.blocksBad = mt.blocksBad;
            t.rateOk = rxGeneral_.ready();
            t.general = std::move(mt);
            return 1;
        }
    } else if (a == 57) {
        RidTelemetry mt;
        if (rxRid_.telemetry(mt, seqs[51])) {
            seqs[51] = mt.seq;
            t.standard = 57;
            t.seq = mt.seq;
            t.state = mt.state;
            t.cfoHz = mt.cfoHz;
            t.dataValid = mt.dataValid;
            t.dataSnrDb = mt.snrDb;
            t.blocksOk = mt.blocksOk; t.blocksBad = mt.blocksBad;
            t.rateOk = rxRid_.ready();
            t.rid = std::move(mt);
            return 1;
        }
    } else if (a == 58) {
        NrTelemetry mt;
        if (rxNr_.telemetry(mt, seqs[52])) {
            seqs[52] = mt.seq;
            t.standard = 58;
            t.seq = mt.seq;
            t.state = mt.state;
            t.cfoHz = mt.cfoHz;
            t.dataValid = mt.dataValid;
            t.dataSnrDb = mt.snrDb;
            t.blocksOk = mt.blocksOk; t.blocksBad = mt.blocksBad;
            t.rateOk = rxNr_.ready();
            t.nr = std::move(mt);
            return 1;
        }
    } else if (a == 59) {
        TimeTelemetry mt;
        if (rxTime_.telemetry(mt, seqs[53])) {
            seqs[53] = mt.seq;
            t.standard = 59;
            t.seq = mt.seq;
            t.state = mt.state;
            t.cfoHz = mt.cfoHz;
            t.dataValid = mt.dataValid;
            t.dataSnrDb = mt.snrDb;
            t.blocksOk = mt.blocksOk; t.blocksBad = mt.blocksBad;
            t.rateOk = rxTime_.ready();
            t.timesig = std::move(mt);
            return 1;
        }
    } else if (a == 60) {
        MeteorTelemetry mt;
        if (rxMeteor_.telemetry(mt, seqs[54])) {
            seqs[54] = mt.seq;
            t.standard = 60;
            t.seq = mt.seq;
            t.state = mt.state;
            t.cfoHz = mt.cfoHz;
            t.dataValid = mt.dataValid;
            t.dataSnrDb = mt.snrDb;
            t.blocksOk = mt.blocksOk; t.blocksBad = mt.blocksBad;
            t.rateOk = rxMeteor_.ready();
            t.meteor = std::move(mt);
            return 1;
        }
    } else if (a == 61) {
        GoesTelemetry mt;
        if (rxGoes_.telemetry(mt, seqs[55])) {
            seqs[55] = mt.seq;
            t.standard = 61;
            t.seq = mt.seq;
            t.state = mt.state;
            t.cfoHz = mt.cfoHz;
            t.dataValid = mt.dataValid;
            t.dataSnrDb = mt.snrDb;
            t.blocksOk = mt.blocksOk; t.blocksBad = mt.blocksBad;
            t.rateOk = rxGoes_.ready();
            t.goes = std::move(mt);
            return 1;
        }
    } else if (a == 62) {
        CubeTelemetry mt;
        if (rxCube_.telemetry(mt, seqs[56])) {
            seqs[56] = mt.seq;
            t.standard = 62;
            t.seq = mt.seq;
            t.state = mt.state;
            t.cfoHz = mt.cfoHz;
            t.dataValid = mt.dataValid;
            t.dataSnrDb = mt.snrDb;
            t.blocksOk = mt.blocksOk; t.blocksBad = mt.blocksBad;
            t.rateOk = rxCube_.ready();
            t.cube = std::move(mt);
            return 1;
        }
    } else return -1;
    return 0;
}

void Engine::analysisLoop() {
    setThreadPriority(ThreadPriority::Realtime); // the sample path must never wait for decoders or the UI
    std::vector<cf32> buf(1 << 16);
    auto next = std::chrono::steady_clock::now();
    SpectrumFrame f;
    uint64_t rxSeq = 0;
    while (!stopReq_) {
        if (resetReq_.exchange(false)) { applyReset(); rxSeq = 0; }
        size_t n;
        catchUp();
        while ((n = ring_.read(buf.data(), buf.size())) > 0) { ingest(buf.data(), n); if (std::chrono::steady_clock::now() > next + std::chrono::milliseconds(250)) break; } // keep publishing spectrum/telemetry even when the receiver is behind
        next += std::chrono::milliseconds(33);
        watchRadio();
        checkDelivery();
        if (const uint64_t d = ring_.dropped(); d != ringDroppedSeen_.load()) { ringDroppedSeen_ = d; cpuDropEvents_++; lastCpuDropNs_ = steadyNs(); }
        {   // the receiver's load: its time on this thread against the signal time it handled, over about two seconds
            const double now = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
            if (loadT_ == 0 || nSamp_ < loadSamp_) { loadT_ = now; loadRx_ = tRx_; loadSamp_ = nSamp_; }
            else if (now - loadT_ >= 2.0) {
                const double sig = (nSamp_ - loadSamp_) / std::max(1.0, rate_.load());
                if (sig > 0.05) loadPct_ = (float)(100.0 * (tRx_ - loadRx_) / sig);
                loadT_ = now; loadRx_ = tRx_; loadSamp_ = nSamp_;
            }
        }
        {
            RxTelemetry t;
            const bool dvbt = activeStd_.load() == 1;
            if (activeStd_.load() == 2) {
                AtscTelemetry at;
                if (rxA_.telemetry(at, atscSeq_)) {
                    atscSeq_ = at.seq;
                    t.standard = 2;
                    t.atsc = at;
                    t.seq = at.seq;
                    t.state = at.fieldSync ? 2 : at.segSync ? 1 : 0;
                    t.cfoHz = at.cfoHz; t.sroPpm = at.sroPpm;
                    t.dataValid = at.fieldSync && at.eqTrained;
                    t.dataSnrDb = (float)at.snrDb;
                    t.blocksOk = at.rsClean + at.rsCorrected;
                    t.blocksBad = at.rsFailed;
                    t.rateOk = rxA_.rateOk();
                    logAtscEvents(t);
                    std::lock_guard<std::mutex> lk(rxMu_);
                    publishRx(std::move(t));
                }
            } else if (activeStd_.load() == 5) {
                if (rxI_.telemetry(t, rxSeq)) {
                    rxSeq = t.seq;
                    logIsdbtEvents(t);
                    std::lock_guard<std::mutex> lk(rxMu_);
                    publishRx(std::move(t));
                }
            } else if (activeStd_.load() == 4) {
                Atsc3Telemetry at;
                if (rxA3_.telemetry(at, atsc3Seq_)) {
                    atsc3Seq_ = at.seq;
                    t.standard = 4;
                    t.seq = at.seq;
                    t.state = at.locked ? 2 : at.bootstraps > 0 ? 1 : 0;
                    t.cfoHz = at.cfoHz;
                    t.dataValid = at.locked && at.serviceReady;
                    t.blocksOk = at.bbPackets - at.bbBad; t.blocksBad = at.bbBad;
                    t.rateOk = true;
                    logAtsc3Events(at);
                    {
                        std::lock_guard<std::mutex> lk(atsc3Mu_);
                        atsc3Tel_ = at;
                    }
                    std::lock_guard<std::mutex> lk(rxMu_);
                    publishRx(std::move(t));
                }
            } else if (activeStd_.load() == 3) {
                DabTelemetry dt;
                if (rxD_.telemetry(dt, dabSeq_)) {
                    dabSeq_ = dt.seq;
                    // a DMB video service plays like a TV service: the player follows the selected DMB sub-channel (a new one starts afresh)
                    if (dt.dmb.sub != dmbSub_) {
                        if (dmbSub_ >= 0) player_.select(-1);
                        dmbSub_ = dt.dmb.sub;
                        if (dmbSub_ >= 0) {
                            { std::lock_guard<std::mutex> lk(tsMu_); demux_.reset(); tsSnap_ = TsSnapshot(); }
                            player_.select(DmbRemux::kProgram);
                        }
                    }
                    t.standard = 3;
                    t.seq = dt.seq;
                    t.state = dt.state;
                    t.cfoHz = dt.cfoHz;
                    t.dataValid = dt.ensemble && dt.ficRecentOk > 0;
                    t.dataSnrDb = (float)dt.snrDb;
                    t.blocksOk = dt.fibOk; t.blocksBad = dt.fibBad;
                    t.rateOk = true;
                    t.dab = std::move(dt);
                    logDabEvents(t);
                    std::lock_guard<std::mutex> lk(rxMu_);
                    publishRx(std::move(t));
                }
            } else if (activeStd_.load() == 6) {
                FmTelemetry ft;
                if (rxFm_.telemetry(ft, fmSeq_)) {
                    fmSeq_ = ft.seq;
                    t.standard = 6;
                    t.seq = ft.seq;
                    t.state = ft.state;
                    t.cfoHz = ft.cfoHz;
                    t.dataValid = ft.state >= 1;
                    t.dataSnrDb = ft.snrDb;
                    t.rateOk = true;
                    t.fm = std::move(ft);
                    std::lock_guard<std::mutex> lk(rxMu_);
                    publishRx(std::move(t));
                }
            } else if (const int pm = pollMode(activeStd_.load(), t, modeSeq_); pm >= 0) {
                if (pm == 1) { std::lock_guard<std::mutex> lk(rxMu_); publishRx(std::move(t)); }
            } else if (dvbt ? rxT_.telemetry(t, rxSeq) : rx_.telemetry(t, rxSeq)) {
                rxSeq = t.seq;
                if (dvbt) logDvbtEvents(t); else logRxEvents(t);
                autoSelect(t, dvbt && (t.dvbt.tpsOk || t.dvbt.fecSync));
                std::lock_guard<std::mutex> lk(rxMu_);
                publishRx(std::move(t));
            } else autoSelect(rxTel_, dvbt && rxTel_.dvbt.tpsOk);
        }
        if (analyzer_.takeFrame(f)) {
            {
                std::lock_guard<std::mutex> lk(specMu_);
                spec_ = f;
            }
            bandwidthStep(f, rxTel_, activeStd_.load() == 1 && rxTel_.dvbt.tpsOk);
        }
        // keep reading while we wait so the ring never fills
        while (std::chrono::steady_clock::now() < next && !stopReq_) {
            catchUp();
            size_t m = ring_.read(buf.data(), buf.size());
            if (m) ingest(buf.data(), m);
            else std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }
}

// ---------------------------------------------------------------- two modes at once (setSecondary)

// The second mode's shift of the whole capture (a few Msps, so it must be cheap): a table of one period of 4096 phase steps, turned
// by the phase at the start of each period, in plain float arithmetic that the compiler vectorises (std::complex products are not)
namespace {
struct CaptureShifter {
    static constexpr size_t L = 4096;
    double shift = 0, rate = 0;
    std::vector<float> re, im;
    std::complex<double> ph{1, 0}, stepL{1, 0};
    size_t pos = 0;
    void set(double s, double r) {
        if (s == shift && r == rate && !re.empty()) return;
        shift = s; rate = r; pos = 0; ph = 1;
        re.resize(L); im.resize(L);
        const double w = r > 0 ? -2 * M_PI * s / r : 0.0;
        for (size_t k = 0; k < L; k++) { re[k] = (float)std::cos(w * (double)k); im[k] = (float)std::sin(w * (double)k); }
        stepL = std::polar(1.0, w * (double)L);
    }
    void mix(cf32* x, size_t n) {
        if (shift == 0 || rate <= 0) return;
        float* v = reinterpret_cast<float*>(x);
        size_t i = 0;
        while (i < n) {
            const size_t seg = std::min(n - i, L - pos);
            const float pr = (float)ph.real(), pi = (float)ph.imag();
            const float* tr = re.data() + pos;
            const float* ti = im.data() + pos;
            float* o = v + 2 * i;
            for (size_t k = 0; k < seg; k++) {
                const float lr = tr[k] * pr - ti[k] * pi, li = tr[k] * pi + ti[k] * pr;
                const float xr = o[2 * k], xi = o[2 * k + 1];
                o[2 * k] = xr * lr - xi * li;
                o[2 * k + 1] = xr * li + xi * lr;
            }
            i += seg; pos += seg;
            if (pos == L) { pos = 0; ph *= stepL; ph /= std::abs(ph); }
        }
    }
};
}   // namespace

bool Engine::secondaryAllowed(int m) {
    switch (m) {
    case 11: case 13: case 15: case 16: case 17: case 18: case 19: case 20: case 22: case 25: case 26: case 27: case 28: case 29: case 30:
    case 32: case 33: case 35: case 36: case 37: case 38: case 39: case 40: case 45: case 47: case 48: case 51: case 52: case 57: case 60:
    case 61: case 63:
        return true;
    default:
        return false;
    }
}

std::string Engine::secondaryCheck(int pri, int sec, double centreHz, double rateHz, double secHz) {
    const ModeTuning* st = modeTuning(sec);
    const ModeTuning* pt = modeTuning(pri);
    const std::string name = st ? st->name : "This mode";
    if (!secondaryAllowed(sec) || !st) {
        const char* why;
        switch (sec) {
        case 14: case 34: case 41: case 43: case 44: case 50: case 53: case 54: case 59:
            why = "it tunes the radio, scans or takes the whole capture by itself"; break;
        case 7: case 12: case 23: case 24:
            why = "its station list and sound belong to the first mode"; break;
        case 21: case 46: case 55: case 56: case 58:
            why = "it needs the radio's whole band for itself"; break;
        default:
            why = "it makes a transport stream or a picture, which only the first mode can show"; break;
        }
        return name + " cannot run as a second mode: " + why;
    }
    if (pri == sec || activeStdFor(pri) == activeStdFor(sec)) return name + " is already the first mode";
    if (pri == 44 || pri == 50) return std::string("no second mode beside ") + (pt ? pt->name : "this mode") + ": it retunes the radio itself";
    char b[400];
    if (rateHz > 0 && rateHz < st->minSampleRate - 1) {
        snprintf(b, sizeof b, "%s needs at least %.2f Msps (the radio runs at %.2f Msps)", name.c_str(), st->minSampleRate / 1e6, rateHz / 1e6);
        return b;
    }
    // the channel (its width around its frequency) inside the part of the capture the filters keep
    const double off = std::fabs(secHz - centreHz);
    const double need = off + st->bandwidthMhz * 0.5e6;
    if (rateHz > 0 && need > 0.45 * rateHz) {
        snprintf(b, sizeof b, "%s at %.3f MHz lies %.3f MHz from the centre of the capture (%.3f MHz): it needs %.1f Msps or more, the radio runs at %.2f Msps. "
                 "Raise the sample rate or tune the first mode nearer", name.c_str(), secHz / 1e6, off / 1e6, centreHz / 1e6, need / 0.45 / 1e6, rateHz / 1e6);
        return b;
    }
    return "";
}

double Engine::captureCentreHz() const {
    TuneSettings t;
    DeviceInfo::Kind k;
    { std::lock_guard<std::mutex> lk(tuneMu_); t = lastTune_; k = lastDev_.kind; }
    if (k == DeviceInfo::File) return t.centerHz;   // a recording: the frequency it was set to is its centre
    const ModeTuning* mt = modeTuning(stdMode_.load());
    return t.centerHz + (mt ? mt->tuneOffsetHz : 0);   // the radio's centre (with offset tuning the samples are shifted back to it)
}

// The receivers that can run as the second mode, set up for a sample rate (start() does it for all of them at the capture's)
void Engine::configureReceiver(int a, double r) {
    switch (a) {
    case 10: rxDmr_.configure(r); break;
    case 12: rxAdsb_.configure(r); break;
    case 14: rxSonde_.configure(r); break;
    case 15: rxAis_.configure(r); break;
    case 16: rxMarine_.configure(r); break;
    case 17: rxAcars_.configure(r); break;
    case 18: rxInmc_.configure(r); break;
    case 19: rxAero_.configure(r); break;
    case 21: rxMesh_.configure(r); break;
    case 24: rxPager_.configure(r); break;
    case 25: rxPacket_.configure(r); break;
    case 26: rxHfdig_.configure(r); break;
    case 27: rxAirband_.configure(r); break;
    case 28: rxVdl2_.configure(r); break;
    case 29: rxIsm_.configure(r); break;
    case 31: rxVor_.configure(r); break;
    case 32: rxM17_.configure(r); break;
    case 34: rxDstar_.configure(r); break;
    case 35: rxCw_.configure(r); break;
    case 36: rxHfdl_.configure(r); break;
    case 37: rxPsk_.configure(r); break;
    case 38: rxJs8_.configure(r); break;
    case 39: rxNxdn_.configure(r); break;
    case 44: rxUplink_.configure(r); break;
    case 46: rxDpmr_.configure(r); break;
    case 47: rxTetra_.configure(r); break;
    case 50: rxAle_.configure(r); break;
    case 51: rxP25_.configure(r); break;
    case 56: rxGeneral_.configure(r); break;
    case 59: rxTime_.configure(r); break;
    case 60: rxMeteor_.configure(r); break;
    case 62: rxCube_.configure(r); break;
    default: break;
    }
}

void Engine::muteReceiver(int a, bool m) {
    switch (a) {
    case 10: rxDmr_.setMuted(m); break;
    case 26: rxHfdig_.setMuted(m); break;
    case 27: rxAirband_.setMuted(m); break;
    case 31: rxVor_.setMuted(m); break;
    case 32: rxM17_.setMuted(m); break;
    case 34: rxDstar_.setMuted(m); break;
    case 35: rxCw_.setMuted(m); break;
    case 37: rxPsk_.setMuted(m); break;
    case 38: rxJs8_.setMuted(m); break;
    case 39: rxNxdn_.setMuted(m); break;
    case 46: rxDpmr_.setMuted(m); break;
    case 50: rxAle_.setMuted(m); break;
    case 51: rxP25_.setMuted(m); break;
    case 56: rxGeneral_.setMuted(m); break;
    default: break;   // no sound
    }
}

void Engine::setSecondarySound(bool on) {
    secSound_ = on;
    std::lock_guard<std::mutex> ck(secCtlMu_);
    if (secTh_.joinable()) muteReceiver(activeStdFor(secStd_.load()), !on);
}

// Where the receiver of active standard a finds its channel: as start() does for the first mode, one receiver at a time
void Engine::placeReceiver(int a, double o, double centreHz, double dialHz, double chanHz) {
    switch (a) {
    case 14: rxSonde_.setSignalOffset(o); break;
    case 15: rxAis_.setSignalOffset(o); break;
    case 16: rxMarine_.setSignalOffset(o); break;
    case 17: rxAcars_.setSignalOffset(o); if (centreHz > 0) rxAcars_.setCenterHz(centreHz); break;
    case 18: rxInmc_.setSignalOffset(o); break;
    case 19: rxAero_.setSignalOffset(o); break;
    case 21: rxMesh_.setSignalOffset(o); break;
    case 24: rxPager_.setSignalOffset(o); break;
    case 25: rxPacket_.setSignalOffset(o); break;
    case 26: rxHfdig_.setSignalOffset(o); break;
    case 27: rxAirband_.setSignalOffset(o); rxAirband_.setCenterHz(centreHz); break;
    case 28: rxVdl2_.setSignalOffset(o); rxVdl2_.setCenterHz(centreHz); break;
    case 29: rxIsm_.setSignalOffset(o); rxIsm_.setCenterHz(centreHz); break;
    case 31: rxVor_.setSignalOffset(o); rxVor_.setCenterHz(centreHz); break;
    case 32: rxM17_.setSignalOffset(o); break;
    case 34: rxDstar_.setSignalOffset(o); break;
    case 35: rxCw_.setSignalOffset(o); break;
    case 36: rxHfdl_.setSignalOffset(o); rxHfdl_.setCenterHz(centreHz); break;
    case 37: rxPsk_.setSignalOffset(o); break;
    case 38: rxJs8_.setSignalOffset(o); break;
    case 39: rxNxdn_.setSignalOffset(o); break;
    case 46: rxDpmr_.setSignalOffset(o); rxDpmr_.setChannel(chanHz / 1e6); break;
    case 47: rxTetra_.setSignalOffset(o); rxTetra_.setCenterHz(centreHz); break;
    case 50: rxAle_.setSignalOffset(o); rxAle_.setDialHz(dialHz); break;
    case 51: rxP25_.setSignalOffset(o); break;
    case 56: rxGeneral_.setSignalOffset(o); break;
    case 59: rxTime_.setSignalOffset(o); rxTime_.setDialHz(dialHz); break;
    case 60: rxMeteor_.setSignalOffset(o); break;
    case 62: rxCube_.setSignalOffset(o); rxCube_.setDialHz(dialHz); break;
    default: break;   // DMR, ADS-B, Mode S uplink: the channel on the centre, nothing to place
    }
}

void Engine::placeReceiverAsPrimary(int a) {
    TuneSettings t;
    DeviceInfo::Kind k;
    { std::lock_guard<std::mutex> lk(tuneMu_); t = lastTune_; k = lastDev_.kind; }
    const bool file = k == DeviceInfo::File;
    const ModeTuning* mt = modeTuning(stdMode_.load());
    placeReceiver(a, mt && !file ? -mt->tuneOffsetHz : 0.0, file ? 0.0 : radioTune(t).centerHz, file ? 0.0 : t.centerHz, t.centerHz);
}

// Shifts the second mode's frequency to where its receiver expects the channel: the centre of a radio tuned for that mode alone (its
// frequency plus its tuneOffsetHz, the receiver mixing the offset out as for the first mode); a recording has the channel on the centre
bool Engine::placeSecondary(std::string& why) {
    const int sm = secStd_.load();
    const double hz = secHz_.load(), centre = captureCentreHz();
    why = secondaryCheck(stdMode_.load(), sm, centre, rate_.load(), hz);
    if (why.empty() && activeStdFor(sm) == activeStd_.load()) why = "the first mode runs the same receiver";
    if (!why.empty()) return false;
    const ModeTuning* mt = modeTuning(sm);
    DeviceInfo::Kind k;
    { std::lock_guard<std::mutex> lk(tuneMu_); k = lastDev_.kind; }
    const bool file = k == DeviceInfo::File;
    const double toff = file || !mt ? 0.0 : mt->tuneOffsetHz;
    const double vc = hz + toff;
    secShift_ = vc - centre;
    placeReceiver(activeStdFor(sm), -toff, file ? 0.0 : vc, file ? 0.0 : hz, hz);
    return true;
}

bool Engine::setSecondary(int stdMode, double offsetHz) { return setSecondaryHz(stdMode, stdMode ? captureCentreHz() + offsetHz : 0.0); }

bool Engine::setSecondaryHz(int sm, double hz) {
    std::lock_guard<std::mutex> ck(secCtlMu_);
    stopSecondaryLocked();
    secStd_ = sm; secHz_ = hz;
    { std::lock_guard<std::mutex> lk(rx2Mu_); rxTel2_ = RxTelemetry(); }
    if (!sm) { setSecNote(""); return true; }
    if (!running_) {   // nothing to measure against yet: the mode now, the fit when the radio starts
        if (!secondaryAllowed(sm) || sm == stdMode_.load()) { setSecNote(secondaryCheck(stdMode_.load(), sm, 0, 0, hz)); secStd_ = 0; return false; }
        setSecNote("starts with the radio");
        return true;
    }
    return startSecondaryLocked(false);
}

void Engine::startSecondary() {
    std::lock_guard<std::mutex> ck(secCtlMu_);
    if (secStd_.load()) startSecondaryLocked(true);
}

void Engine::stopSecondary() {
    std::lock_guard<std::mutex> ck(secCtlMu_);
    stopSecondaryLocked();
}

bool Engine::startSecondaryLocked(bool keep) {
    const int sm = secStd_.load();
    if (!sm || !running_ || secTh_.joinable()) return false;
    const int a = activeStdFor(sm);
    std::string why;
    bool ok;
    {
        std::lock_guard<std::mutex> fl(secFeedMu_);
        ok = placeSecondary(why);
        if (ok) {   // as a radio tuned for this mode alone: its own sample rate (or the capture's when that is lower), fresh tables
            const ModeTuning* mt = modeTuning(sm);
            const double want = mt && mt->sampleRate > 0 ? mt->sampleRate : rate_.load();
            secRate_ = rate_.load() > want * 1.001 ? want : rate_.load();
            configureReceiver(a, secRate_.load());
            muteReceiver(a, !secSound_.load());   // the first mode keeps the speakers unless the second is asked to play
            resetReceiver(a);   // the first mode does not use it, the second thread is not running yet
        }
    }
    if (!ok && !keep) { secStd_ = 0; setSecNote(why); log("second mode refused: " + why); return false; }
    for (auto& s : modeSeq2_) s = 0;
    {
        std::lock_guard<std::mutex> lk(secQMu_);
        secQ_.clear(); secQN_ = 0; secStop_ = false;
    }
    secDropped_ = 0;
    secFlush_ = true;
    secAct_ = ok ? a : -1;
    setSecNote(ok ? "" : "suspended: " + why);
    secTh_ = std::thread([this, a] { secondaryLoop(a); });
    char b[240];
    const ModeTuning* mt = modeTuning(sm);
    if (ok) snprintf(b, sizeof b, "second mode: %s at %.4f MHz, %.3f MHz from the centre of the capture", mt ? mt->name : "?", secHz_.load() / 1e6, (secHz_.load() - captureCentreHz()) / 1e6);
    else snprintf(b, sizeof b, "second mode suspended: %s", why.c_str());
    log(b);
    return ok;
}

void Engine::stopSecondaryLocked() {
    const bool had = secTh_.joinable();
    const int a = activeStdFor(secStd_.load());
    secAct_ = -1;
    {
        std::lock_guard<std::mutex> lk(secQMu_);
        secStop_ = true;
    }
    secQCv_.notify_all();
    if (secTh_.joinable()) secTh_.join();
    {
        std::lock_guard<std::mutex> lk(secQMu_);
        secQ_.clear(); secQN_ = 0; secStop_ = false;
    }
    // the receiver back where start() put it, should it become the first mode without a restart (autoSelect does it for the first mode itself)
    if (had && a != activeStd_.load()) { configureReceiver(a, rate_.load()); placeReceiverAsPrimary(a); muteReceiver(a, false); }
}

void Engine::refreshSecondary(bool force) {
    std::lock_guard<std::mutex> ck(secCtlMu_);
    if (!secTh_.joinable()) return;
    const int a = activeStdFor(secStd_.load());
    std::lock_guard<std::mutex> fl(secFeedMu_);
    const bool was = secAct_.load() == a;
    if (was && !force) return;
    std::string why;
    if (placeSecondary(why)) {
        if (!was) { secFlush_ = true; setSecNote(""); log("second mode resumed"); }
        secAct_ = a;
    } else {
        if (was) log("second mode suspended: " + why);
        secAct_ = -1;
        setSecNote("suspended: " + why);
        secQCv_.notify_all();
    }
}

// The first mode's thread hands a copy of every block over; a live radio cannot wait, so a second receiver that falls behind loses
// blocks (counted); a recording or the test signal waits for it, as the source waits for the first receiver
void Engine::pushSecondary(const cf32* x, size_t n) {
    const size_t cap = std::max<size_t>(1u << 18, (size_t)(rate_.load() * 0.5));
    std::unique_lock<std::mutex> lk(secQMu_);
    if (live_.load()) {
        if (secQN_ + n > cap) { secDropped_ += n; return; }
    } else {
        while (!secStop_ && secAct_.load() >= 0 && secQN_ + n > cap && !stopReq_) secQCv_.wait_for(lk, std::chrono::milliseconds(50));
        if (secStop_ || secAct_.load() < 0 || stopReq_) return;
    }
    secQ_.emplace_back(x, x + n);
    secQN_ += n;
    lk.unlock();
    secQCv_.notify_all();
}

void Engine::secondaryLoop(int a) {
    setThreadPriority(ThreadPriority::Background);   // below the first mode's sample path
    CaptureShifter mix;
    ExactResampler rs;   // the capture's rate down to the second mode's own
    double rsIn = 0, rsOut = 0;
    std::vector<cf32> dec;
    std::vector<cf32> b;
    auto nextPoll = std::chrono::steady_clock::now();
    for (;;) {
        b.clear();
        {
            std::unique_lock<std::mutex> lk(secQMu_);
            secQCv_.wait_for(lk, std::chrono::milliseconds(30), [&] { return secStop_ || !secQ_.empty(); });
            if (secStop_) break;
            if (!secQ_.empty()) { b.swap(secQ_.front()); secQ_.pop_front(); secQN_ -= b.size(); }
        }
        if (!b.empty()) {
            secQCv_.notify_all();   // room for the next block
            std::lock_guard<std::mutex> fl(secFeedMu_);
            if (secFlush_.exchange(false)) { mix = CaptureShifter(); rs.reset(); for (auto& s : modeSeq2_) s = 0; }
            if (secAct_.load() == a) {
                mix.set(secShift_.load(), rate_.load());
                mix.mix(b.data(), b.size());
                if (rsIn != rate_.load() || rsOut != secRate_.load()) { rsIn = rate_.load(); rsOut = secRate_.load(); if (!rs.configure(rsIn, rsOut)) rs.configure(1, 1); }
                if (rs.passthrough()) feedReceiver(a, b.data(), b.size());
                else { dec.clear(); rs.process(b.data(), b.size(), dec); if (!dec.empty()) feedReceiver(a, dec.data(), dec.size()); }
            }
        }
        const auto now = std::chrono::steady_clock::now();
        if (now >= nextPoll) {
            nextPoll = now + std::chrono::milliseconds(33);
            RxTelemetry t;
            int pm = 0;
            { std::lock_guard<std::mutex> fl(secFeedMu_); if (secAct_.load() == a) pm = pollMode(a, t, modeSeq2_); }
            if (pm == 1) { std::lock_guard<std::mutex> lk(rx2Mu_); t.seq = ++rx2Pub_; rxTel2_ = std::move(t); }
        }
    }
}

bool Engine::latestRx2(RxTelemetry& out, uint64_t lastSeq) {
    std::lock_guard<std::mutex> lk(rx2Mu_);
    if (rxTel2_.seq <= lastSeq) return false;
    out = rxTel2_;
    return true;
}

void applySubMode(Engine& e, const SubMode* s) {
    if (!s) { e.hfdig().setDecoders(31); e.pager().setProtocols(3); e.vor().setFunction(0); return; }
    switch (s->stdMode) {
    case 27:
        e.hfdig().setDecoders(s->decoders);
        if (s->sel >= 0) for (int m = 0; m < kFtxModes; m++) e.hfdig().ftx().setEnabled(m, m == s->sel);
        break;
    case 22: e.mesh().setProtocols(s->decoders); break;
    case 32: e.vor().setFunction(s->sel); break;
    case 25: e.pager().setProtocols(s->decoders); break;
    case 17: e.marine().setService(s->sel); break;
    default: break;
    }
}

} // namespace dect2
