// DAB+ PAD: data stream element -> F-PAD / X-PAD -> MSC data groups -> MOT objects -> slide show picture.
#include "dect2/dab_pad.h"
#include "dect2/dab.h"
#include "dect2/png.h"
extern "C" {
#include <libavcodec/avcodec.h>
#include <libswscale/swscale.h>
}
#include <algorithm>
#include <cstring>

namespace dect2 {

namespace {
constexpr int kSubfieldLen[8] = {4, 6, 8, 12, 16, 24, 32, 48};   // EN 300 401 table 30: the length index of a content indicator
constexpr size_t kMaxObjects = 6;
constexpr size_t kMaxObjectBytes = 1 << 20;
}

bool decodeSlideImage(const uint8_t* data, size_t n, int subtype, int& w, int& h, std::vector<uint8_t>& rgba) {
    if (!data || n < 8) return false;
    if (subtype == 3) return decodePng(data, n, w, h, rgba);
    const AVCodecID id = subtype == 1 ? AV_CODEC_ID_MJPEG : subtype == 0 ? AV_CODEC_ID_GIF : subtype == 2 ? AV_CODEC_ID_BMP : AV_CODEC_ID_NONE;
    const AVCodec* codec = id == AV_CODEC_ID_NONE ? nullptr : avcodec_find_decoder(id);
    if (!codec) return false;
    AVCodecContext* ctx = avcodec_alloc_context3(codec);
    AVPacket* pkt = av_packet_alloc();
    AVFrame* fr = av_frame_alloc();
    bool ok = false;
    if (ctx && pkt && fr && avcodec_open2(ctx, codec, nullptr) >= 0 && av_new_packet(pkt, (int)n) >= 0) {
        std::memcpy(pkt->data, data, n);
        int r = avcodec_send_packet(ctx, pkt);
        if (r >= 0) { r = avcodec_receive_frame(ctx, fr); if (r == AVERROR(EAGAIN)) { avcodec_send_packet(ctx, nullptr); r = avcodec_receive_frame(ctx, fr); } }
        if (r >= 0 && fr->width > 0 && fr->height > 0 && fr->width <= 4096 && fr->height <= 4096) {
            SwsContext* sws = sws_getContext(fr->width, fr->height, (AVPixelFormat)fr->format, fr->width, fr->height, AV_PIX_FMT_RGBA, SWS_BILINEAR, nullptr, nullptr, nullptr);
            if (sws) {
                w = fr->width; h = fr->height;
                rgba.assign((size_t)w * (size_t)h * 4, 0);
                uint8_t* dst[1] = {rgba.data()};
                const int ls[1] = {w * 4};
                ok = sws_scale(sws, fr->data, fr->linesize, 0, h, dst, ls) == h;
                sws_freeContext(sws);
            }
        }
    }
    if (fr) av_frame_free(&fr);
    if (pkt) av_packet_free(&pkt);
    if (ctx) avcodec_free_context(&ctx);
    return ok;
}

void DabPadDecoder::reset() {
    lastType_ = 0; group_.clear(); groupLen_ = 0; collecting_ = false;
    objs_.clear(); order_.clear();
    slide_ = DabSlide();
}

void DabPadDecoder::accessUnit(const uint8_t* au, int n) {
    if (n < 4 || (au[0] >> 5) != 4) return;                 // the data stream element (id_syn_ele 4) comes first when there is PAD
    int pos = 2, cnt = au[1];
    if (cnt == 255) { cnt += au[2]; pos = 3; }
    if (cnt < 2 || pos + cnt > n) return;
    const uint8_t* d = au + pos;
    const uint8_t fpad[2] = {d[cnt - 2], d[cnt - 1]};
    std::vector<uint8_t> x((size_t)cnt - 2);
    for (int k = 0; k < cnt - 2; k++) x[(size_t)k] = d[cnt - 3 - k];
    pad(x.data(), cnt - 2, fpad);
}

void DabPadDecoder::pad(const uint8_t* xp, int xlen, const uint8_t fpad[2]) {
    if ((fpad[0] >> 6) != 0) return;                       // F-PAD type 0 only
    const int ind = (fpad[0] >> 4) & 3;
    const bool ci = (fpad[1] & 2) != 0;
    if ((ind != 1 && ind != 2) || xlen <= 0) return;
    if (ind == 1) xlen = std::min(xlen, 4);                // short X-PAD: 4 bytes
    struct Sub { int type, len; };
    std::vector<Sub> subs;
    int pos = 0;
    if (ci) {
        if (ind == 1) { subs.push_back({xp[0] & 0x1F, 3}); pos = 1; }
        else {
            int i = 0;
            for (; i < 4 && i < xlen; i++) {
                const int t = xp[i] & 0x1F;
                if (t == 0) { i++; break; }                 // end marker (also uses up a byte)
                subs.push_back({t, kSubfieldLen[xp[i] >> 5]});
            }
            pos = i;
        }
        if (!subs.empty()) lastType_ = subs.back().type;
    } else {
        const int t = lastType_ == 12 ? 13 : lastType_ == 2 ? 3 : lastType_;   // the data continue the application of the last indicator
        if (t == 0) return;
        subs.push_back({t, xlen});
    }
    for (const Sub& s : subs) {
        const int len = std::min(s.len, xlen - pos);
        if (len <= 0) break;
        subfield(s.type, xp + pos, len);
        pos += len;
    }
}

void DabPadDecoder::subfield(int type, const uint8_t* d, int n) {
    if (type == 1) {                                        // data group length indicator: 14 bit length, CRC-16
        if (n < 2) return;
        if (n >= 4 && dab::crc16(d, 2) != (uint16_t)((d[2] << 8) | d[3])) { groupLen_ = 0; collecting_ = false; return; }
        groupLen_ = ((d[0] & 0x3F) << 8) | d[1];
        collecting_ = false;
        return;
    }
    if (type != 12 && type != 13) return;                   // 2 / 3 (dynamic label) and the rest are not used here
    if (type == 12) { group_.clear(); collecting_ = groupLen_ > 0; }
    if (!collecting_) return;
    const size_t take = std::min((size_t)n, (size_t)groupLen_ - group_.size());
    group_.insert(group_.end(), d, d + take);
    if ((int)group_.size() >= groupLen_) {
        std::vector<uint8_t> g;
        g.swap(group_);
        collecting_ = false; groupLen_ = 0;
        dataGroup(g.data(), (int)g.size());
    }
}

void DabPadDecoder::dataGroup(const uint8_t* d, int n) {
    if (n < 3) return;
    const bool ext = d[0] & 0x80, crc = d[0] & 0x40, seg = d[0] & 0x20, ua = d[0] & 0x10;
    const int type = d[0] & 0x0F;
    if (crc) {
        if (n < 5 || dab::crc16(d, n - 2) != (uint16_t)((d[n - 2] << 8) | d[n - 1])) return;
        n -= 2;
    }
    int p = 2 + (ext ? 2 : 0);
    bool last = true;
    int segNo = 0;
    if (seg) {
        if (p + 2 > n) return;
        last = (d[p] & 0x80) != 0;
        segNo = ((d[p] & 0x7F) << 8) | d[p + 1];
        p += 2;
    }
    uint16_t tid = 0;
    if (ua) {
        if (p + 1 > n) return;
        const bool hasTid = d[p] & 0x10;
        const int li = d[p] & 0x0F;
        p++;
        if (p + li > n) return;
        if (hasTid && li >= 2) tid = (uint16_t)((d[p] << 8) | d[p + 1]);
        p += li;
    }
    if (type != 3 && type != 4) return;                     // MOT header and (unscrambled) body; the directory mode is not used by slide shows
    if (p + 2 > n) return;
    const int size = ((d[p] & 0x1F) << 8) | d[p + 1];      // the MOT segmentation header: repetition count (3 bits), segment size (13 bits)
    p += 2;
    segment(tid, type, segNo, last, d + p, std::min(size, n - p));
}

void DabPadDecoder::segment(uint16_t tid, int type, int segNo, bool last, const uint8_t* d, int n) {
    if (n < 0 || segNo > 4095) return;
    auto it = objs_.find(tid);
    if (it == objs_.end()) {
        if (order_.size() >= kMaxObjects) { objs_.erase(order_.front()); order_.pop_front(); }
        it = objs_.emplace(tid, Obj()).first;
        order_.push_back(tid);
    }
    Obj& o = it->second;
    auto& parts = type == 3 ? o.head : o.body;
    int& lastNo = type == 3 ? o.headLast : o.bodyLast;
    if (type == 3 && segNo == 0) {                          // a different header under the same transport id: a new object
        auto h0 = o.head.find(0);
        if (h0 != o.head.end() && (h0->second.size() != (size_t)n || std::memcmp(h0->second.data(), d, (size_t)n) != 0)) o = Obj();
    }
    if (o.done) return;
    parts[segNo].assign(d, d + n);
    if (last) lastNo = segNo;
    if (lastNo >= 0 && parts.rbegin()->first > lastNo) { parts.clear(); lastNo = -1; return; }
    tryComplete(o);
}

void DabPadDecoder::tryComplete(Obj& o) {
    if (o.headLast < 0 || o.bodyLast < 0) return;
    if ((int)o.head.size() != o.headLast + 1 || (int)o.body.size() != o.bodyLast + 1) return;
    std::vector<uint8_t> h, b;
    for (auto& kv : o.head) h.insert(h.end(), kv.second.begin(), kv.second.end());
    for (auto& kv : o.body) b.insert(b.end(), kv.second.begin(), kv.second.end());
    if (h.size() < 7 || b.size() > kMaxObjectBytes) return;
    o.done = true;
    const size_t bodySize = ((size_t)h[0] << 20) | ((size_t)h[1] << 12) | ((size_t)h[2] << 4) | (h[3] >> 4);
    const size_t headSize = (((size_t)h[3] & 0x0F) << 9) | ((size_t)h[4] << 1) | (h[5] >> 7);
    const int ctype = (h[5] >> 1) & 0x3F;
    const int subtype = ((h[5] & 1) << 8) | h[6];
    if (ctype != 2 || (bodySize && bodySize != b.size())) return;     // not a picture, or the body is not what the header says
    std::string name;
    for (size_t p = 7; p < std::min(headSize, h.size());) {            // the header extension: parameters with a 2 bit length class
        const int pli = h[p] >> 6, id = h[p] & 0x3F;
        p++;
        size_t len = pli == 0 ? 0 : pli == 1 ? 1 : pli == 2 ? 4 : 0;
        if (pli == 3) {
            if (p >= h.size()) break;
            if (h[p] & 0x80) { if (p + 1 >= h.size()) break; len = ((size_t)(h[p] & 0x7F) << 8) | h[p + 1]; p += 2; }
            else len = h[p++] & 0x7F;
        }
        if (p + len > h.size()) break;
        if (id == 0x0C && len >= 1) name.assign((const char*)&h[p + 1], len - 1);   // ContentName: character set byte, then the text
        p += len;
    }
    DabSlide s;
    if (!decodeSlideImage(b.data(), b.size(), subtype, s.w, s.h, s.rgba)) return;
    s.name = name;
    s.seq = slide_.seq + 1;
    slide_ = std::move(s);
}

} // namespace dect2
