// DAB / DAB+ data structures shared by the receiver and the UI: ensemble contents and live telemetry.
#pragma once
#include "ring.h"
#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace dect2 {

struct DabSubchannel {
    int id = 0;
    int start = 0;        // first capacity unit (CU) inside the CIF
    int size = 0;         // number of CUs
    bool eep = true;      // equal error protection (false: unequal, short form)
    int option = 0;       // EEP: 0 = A profiles, 1 = B profiles
    int level = 0;        // EEP: 0..3 = protection level 1..4; UEP: protection level 1..5 (stored as 0..4)
    int uepIndex = 0;     // UEP table index (short form)
    int bitrate = 0;      // kbit/s, 0 = unknown
};

struct DabComponent {
    int tmid = 0;         // transport mechanism: 0 MSC stream audio, 1 MSC stream data, 3 MSC packet data
    int ascty = -1;       // audio service component type: 0 = MPEG-1/2 Layer II (DAB), 63 = HE-AAC (DAB+)
    int dscty = -1;       // data service component type: 24 = MPEG-2 transport stream (TS 102 427), the carrier of DMB video
    int subId = -1;
    bool primary = true;
    bool dmb() const { return tmid == 1 && dscty == 24; }
};

// One DL Plus tag (ETSI TS 102 980): a part of the dynamic label with its meaning (1 title, 4 artist, 2 album ... see dabDlpTypeName)
struct DabDlpTag {
    int type = 0;
    std::string text;
};

struct DabService {
    uint32_t sid = 0;
    std::string label;
    std::vector<DabComponent> comps;
    std::string dls;      // dynamic label ("now playing"), when the station sends one
    std::vector<DabDlpTag> dlPlus;   // the DL Plus tags of the label of the selected service, when it sends them
    bool dlpRunning = false;         // DL Plus: the item (programme) is running
    int pty = -1;                    // programme type (FIG 0/17), -1 = none signalled
    bool ptyDynamic = false;         // the programme type describes the current programme (FIG 0/17 S/D flag), not the station in general
    int language = -1;               // FIG 0/17
    std::map<int, std::string> compLabels;   // service component labels by SCIdS (FIG 1/4, 2/4)
    // FIG 0/13 user applications of the primary component (TS 101 756 table 16: 0x009 = DMB), -1 = none signalled
    int userApp = -1;
    int dmbProfile = 0;   // DMB: the VideoServiceObjectProfileId (TS 102 428 table 8: 1 = BSAC audio, 2 = HE-AAC v2 audio)
    // The first audio stream component, or null
    const DabComponent* audio() const {
        for (const auto& c : comps) if (c.tmid == 0) return &c;
        return nullptr;
    }
    bool dabPlus() const { const DabComponent* a = audio(); return a && a->ascty == 63; }
    // The DMB video stream component (stream data carrying a transport stream, not signalled as another user application), or null
    const DabComponent* dmb() const {
        if (userApp >= 0 && userApp != 0x009) return nullptr;
        for (const auto& c : comps) if (c.dmb()) return &c;
        return nullptr;
    }
    // What a click plays: the audio, else the DMB video
    const DabComponent* playable() const { const DabComponent* a = audio(); return a ? a : dmb(); }
};

// Announcements (EN 300 401 8.1.6): the types a service supports and the clusters it belongs to (FIG 0/18), and the announcements being made
// now (FIG 0/19): which sub-channel carries the announcement for a cluster
struct DabAnnSupport {
    unsigned asu = 0;             // bit n: announcement type n (0 alarm, 1 road traffic, 2 transport, 3 warning, 4 news, 5 weather, 6 event, 7 special event, 8 programme information, 9 sport, 10 finance)
    std::vector<int> clusters;
};
struct DabAnnSwitch {
    int cluster = 0;
    unsigned asw = 0;             // the types being announced
    int subId = -1;               // the sub-channel that carries the announcement
    bool newFlag = false;         // a new announcement (false: a repetition)
    bool region = false;          // the announcement is for a region only (not evaluated)
};

struct DabEnsemble {
    bool valid = false;           // ensemble id and at least one service seen
    uint16_t eid = 0;
    std::string label;
    int64_t utc = 0;              // broadcast time (seconds since 1970), 0 = not received
    std::map<int, DabSubchannel> subs;
    std::map<uint32_t, DabService> services;
    std::map<uint32_t, DabAnnSupport> annSupport;   // service id -> announcement support
    std::vector<DabAnnSwitch> annActive;            // announcements heard in the last few seconds
};

// The audio parameters in the header byte of a DAB+ superframe (ETSI TS 102 563 clause 5.2): b6 DAC rate, b5 SBR, b4 AAC channel mode, b3 PS, b2..0 MPEG Surround
struct DabAacParams {
    bool valid = false;
    int outRate = 0;              // sampling rate of the output: 32000 or 48000
    int coreRate = 0;             // sampling rate of the AAC-LC core: the output rate, or half of it with SBR
    bool sbr = false;             // spectral band replication (HE-AAC)
    bool ps = false;              // parametric stereo (HE-AAC v2): a mono core made stereo
    bool stereo = false;          // the AAC channel mode: two channels (with PS: the output)
    int surround = 0;             // MPEG Surround configuration, 0 = none
    int auPerSuperframe = 0;      // access units in a superframe (120 ms): 2, 3, 4 or 6
    int coreChannels = 0;         // channels the AAC-LC core codes
    int outChannels = 0;
    std::string profile;          // "AAC-LC", "HE-AAC (AAC-LC + SBR)", "HE-AAC v2 (AAC-LC + SBR + PS)"
};

struct DabAudioStats {
    DabAacParams aac;             // DAB+: from the superframe header
    std::string decoder;          // "FFmpeg" or "faad2"
    bool sbrDecoded = false;      // the decoder applied SBR / PS (faad2 reports it: FFmpeg cannot with DAB+'s 960-sample frames)
    bool psDecoded = false;
    int sub = -1;                 // selected sub-channel, -1 = none
    bool dabPlus = false;
    int bitrate = 0;
    std::string codec;            // "HE-AAC v2", "AAC-LC", "MP2" ...
    int sampleRate = 0, channels = 0;
    bool decoding = false;        // audio is being produced
    uint64_t frames = 0;          // logical frames received
    uint64_t superframesOk = 0, superframesBad = 0;
    uint64_t rsCorrected = 0;     // bytes fixed by Reed-Solomon
    uint64_t auOk = 0, auBad = 0; // access units with a good / bad CRC
    uint64_t pcmFrames = 0;
    int bufferedMs = 0;
    int underruns = 0;
};

// The DMB video service being decoded (ETSI TS 102 427 outer code, TS 102 428 MPEG-4 Systems in the transport stream)
struct DmbStats {
    bool active = false;          // a DMB sub-channel is selected
    int sub = -1;                 // which
    bool sync = false;            // the packet sync of the outer interleaver was found
    uint64_t rsOk = 0, rsFailed = 0;    // Reed-Solomon (204,188) blocks decoded / beyond repair
    uint64_t rsCorrected = 0;     // bytes fixed by Reed-Solomon
    uint64_t tsOut = 0;           // transport stream packets handed to the player
    std::string video, audio;     // "H.264", "HE-AAC", "AAC-LC", "BSAC" ...
    int sampleRate = 0, channels = 0;
    std::string note;             // what cannot be played yet, for the service info
};

struct DabTelemetry {
    uint64_t seq = 0;
    bool sync = false;            // frame sync
    int state = 0;                // 0 searching, 1 syncing (found the null symbol), 2 locked
    float cirPeak = 0;            // peak / median of the phase reference correlation (about 100+ on a good signal)
    double cfoHz = 0;             // carrier frequency offset
    double snrDb = 0;             // estimated from the DQPSK clusters
    double secSinceSync = 0;
    uint64_t frames = 0;          // DAB frames (96 ms) processed
    uint64_t fibOk = 0, fibBad = 0;     // FIB CRC results since start
    int ficRecentOk = 0;          // FIBs with good CRC in the last frame (out of 12)
    bool ensemble = false;
    std::string ensembleLabel;
    int services = 0;
    std::vector<cf32> constellation;    // differential cells of one data symbol (clusters at the 4 diagonals)
    std::vector<float> cir;             // channel impulse response magnitude (phase reference correlation), 2048 points
    DabAudioStats audio;
    DmbStats dmb;
    // TII (EN 300 401 clause 14.8): the transmitters of the network heard in the null symbols, strongest first
    struct Tii { int mainId = 0, subId = 0; float levelDb = 0, marginDb = 0; };
    std::vector<Tii> tii;
    int tiiFrames = 0;                  // null symbols analysed since the lock
};

} // namespace dect2
