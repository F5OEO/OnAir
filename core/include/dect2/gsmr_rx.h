// GSM-R receiver: the railway GSM base stations around you (GSM 900 band, railway allocation 921 - 925 MHz), from what every base station
// broadcasts in the clear to all mobiles on its BCCH (3GPP TS 44.018 / 45.002 / 45.003 / 45.004): its base station identity code (SCH),
// and from System Information 1 to 4 its cell identity, location area and operator code, access parameters and channel lists.
// Receive only. Nothing addressed to a mobile is decoded: paging, assignments, traffic channels, the uplink, and the railway applications
// above the radio layer (voice group calls, ETCS data) are out of scope. No subscriber or device identity exists in these messages, and
// the BCCH is not ciphered.
//
// The input is taken in blocks of 2.2 s (a full cycle of the eight System Information schedule positions). Each block's spectrum is
// searched for 200 kHz carriers; each carrier is brought to baseband and searched for the frequency correction bursts (FCCH), the
// synchronisation burst that follows (BSIC and frame number), and then every BCCH block of the block is equalised and decoded. A block
// is captured only while the previous one is not being searched, so a slow computer just searches less often.
//
// Written to the engine's contract: configure() once the sample rate is known, feed() from the analysis thread (never blocks),
// telemetry() and the setters from the interface thread.
#pragma once
#include "mode_tuning.h"
#include "ring.h"
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

namespace dect2 {

struct GsmrTelemetry;   // gsmr_tel.h

class GsmrReceiver {
public:
    GsmrReceiver();
    ~GsmrReceiver();

    void configure(double inputRateHz);
    void setSignalOffset(double hz);   // where the user's (dial) frequency sits relative to 0 Hz in the input
    void setCenterHz(double hz);       // the radio's centre frequency (0: unknown, for files)
    bool ready() const;                // false when the input rate is too low for this mode
    void reset();                      // after a retune: drop the block being captured (the cell table is kept)
    void clearCells();                 // the user's Clear button
    void feed(const cf32* x, size_t n);
    bool telemetry(GsmrTelemetry& out, uint64_t lastSeq);
    void setLogCallback(std::function<void(const std::string&)> cb);
    bool busy() const;                 // a block is captured or being searched (tools: feed a file only as fast as it is searched)

private:
    struct Impl;
    std::unique_ptr<Impl> p_;
};

ModeTuning gsmrTuning();

// ---------------------------------------------------------------- the 900 MHz channel raster
double gsmrArfcnToHz(int arfcn);               // downlink; 0 when not a GSM 900 channel (0 - 124, 955 - 1023)
int gsmrHzToArfcn(double hz);                  // -1 when outside GSM 900
bool gsmrIsRailway(int arfcn);                 // 955 - 973 (921.2 - 924.8 MHz)
// Operator of an MCC-MNC from a short built-in list of public network codes ("" unknown), and the country of an MCC
const char* gsmrOperatorName(int mcc, int mnc);
const char* gsmrCountry(int mcc);

} // namespace dect2
