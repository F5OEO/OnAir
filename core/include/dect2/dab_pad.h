// DAB+ programme associated data: the PAD carried in the data stream element of every access unit (ETSI TS 102 563 clause 5.2, EN 300 401
// clause 7.4), the MSC data groups sent in its X-PAD (EN 300 401 clause 5.3.3, X-PAD application types 1, 12 and 13) and the MOT objects in those
// (ETSI EN 301 234, header mode). A MOT object whose content type is an image is the station's slide show picture.
#pragma once
#include <cstddef>
#include <cstdint>
#include <deque>
#include <map>
#include <string>
#include <vector>

namespace dect2 {

struct DabSlide {
    uint64_t seq = 0;                // counts the pictures received; 0 = none yet
    int w = 0, h = 0;
    std::vector<uint8_t> rgba;       // w * h pixels, R G B A
    std::string name;                // the MOT ContentName, when sent
};

class DabPadDecoder {
public:
    void reset();
    // One DAB+ access unit (raw AAC data block, CRC removed). Looks for the data stream element that carries the PAD.
    // The PAD bytes in the element are in the order of EN 300 401 figure 29 (as at the end of an MP2 frame): the two F-PAD bytes last, the X-PAD
    // before them with the byte sent first right next to the F-PAD.
    void accessUnit(const uint8_t* au, int n);
    // The PAD of one audio frame: xpad = the X-PAD bytes in the order they were sent, fpad = the last two bytes of the PAD.
    void pad(const uint8_t* xpad, int xlen, const uint8_t fpad[2]);
    uint64_t seq() const { return slide_.seq; }
    const std::string& label() const { return label_; }   // the dynamic label (X-PAD application types 2 and 3), UTF-8
    const DabSlide& slide() const { return slide_; }

private:
    struct Obj {
        std::map<int, std::vector<uint8_t>> head, body;
        int headLast = -1, bodyLast = -1;
        bool done = false;
    };
    void subfield(int type, const uint8_t* d, int n);
    void dataGroup(const uint8_t* d, int n);
    void segment(uint16_t tid, int type, int segNo, bool last, const uint8_t* d, int n);
    void tryComplete(Obj& o);
    void labelSegment(const uint8_t* d, int n);

    int lastType_ = 0;                   // application type of the last content indicator, for X-PAD without one
    std::vector<uint8_t> group_;         // the data group being collected
    int groupLen_ = 0;                   // its length from the data group length indicator, 0 = none seen
    bool collecting_ = false;
    std::vector<uint8_t> dls_;           // the dynamic label segment being collected
    bool dlsOn_ = false;
    int dlsToggle_ = -1;                 // toggle bit of the label being assembled
    std::map<int, std::vector<uint8_t>> dlsParts_;
    int dlsLast_ = -1, dlsCharset_ = 0;
    std::string label_;
    std::map<uint16_t, Obj> objs_;
    std::deque<uint16_t> order_;
    DabSlide slide_;
};

// Decodes JPEG, PNG, GIF or BMP (MOT content subtype 1, 3, 0, 2) to RGBA.
bool decodeSlideImage(const uint8_t* data, size_t n, int subtype, int& w, int& h, std::vector<uint8_t>& rgba);

} // namespace dect2
