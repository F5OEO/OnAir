// FIG decoders. Ported from the FIG analyser of etisnoop (https://github.com/Opendigitalradio/etisnoop), GPL-3.0-or-later:
//   Copyright (C) 2014 CSP Innovazione nelle ICT s.c.a r.l. (http://www.csp.it/)
//   Copyright (C) 2018 Matthias P. Braendli (http://www.opendigitalradio.org)
//   Copyright (C) 2015 Data Path
// Each decoder works on one FIG data field copied into a zero padded buffer, so a truncated FIG reads zeros and ends its loops instead
// of reading outside it; the global databases of etisnoop (international table, main identifier positions, label segments) come in
// through DabFigContext.
#include "dect2/dab_fig.h"
#include "dect2/text_charset.h"
#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>

namespace dect2 {

namespace {

std::string pf(const char* fmt, ...) __attribute__((format(printf, 1, 2)));
std::string pf(const char* fmt, ...) {
    char b[512];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(b, sizeof b, fmt, ap);
    va_end(ap);
    return b;
}

const char* const kLanguage[] = {
    "Unknown/not applicable", "Albanian", "Breton", "Catalan", "Croatian", "Welsh", "Czech", "Danish", "German", "English", "Spanish",
    "Esperanto", "Estonian", "Basque", "Faroese", "French", "Frisian", "Irish", "Gaelic", "Galician", "Icelandic", "Italian", "Lappish",
    "Latin", "Latvian", "Luxembourgian", "Lithuanian", "Hungarian", "Maltese", "Dutch", "Norwegian", "Occitan", "Polish", "Portuguese",
    "Romanian", "Romansh", "Serbian", "Slovak", "Slovene", "Finnish", "Swedish", "Turkish", "Flemish", "Walloon", "rfu", "rfu", "rfu",
    "rfu", "Reserved for national assignment", "Reserved for national assignment", "Reserved for national assignment",
    "Reserved for national assignment", "Reserved for national assignment", "Reserved for national assignment",
    "Reserved for national assignment", "Reserved for national assignment", "Reserved for national assignment",
    "Reserved for national assignment", "Reserved for national assignment", "Reserved for national assignment",
    "Reserved for national assignment", "Reserved for national assignment", "Reserved for national assignment",
    "Reserved for national assignment", "Background sound/clean feed", "rfu", "rfu", "rfu", "rfu", "Zulu", "Vietnamese", "Uzbek", "Urdu",
    "Ukranian", "Thai", "Telugu", "Tatar", "Tamil", "Tadzhik", "Swahili", "Sranan Tongo", "Somali", "Sinhalese", "Shona", "Serbo-Croat",
    "Rusyn", "Russian", "Quechua", "Pushtu", "Punjabi", "Persian", "Papiamento", "Oriya", "Nepali", "Ndebele", "Marathi", "Moldavian",
    "Malaysian", "Malagasay", "Macedonian", "Laotian", "Korean", "Khmer", "Kazakh", "Kannada", "Japanese", "Indonesian", "Hindi", "Hebrew",
    "Hausa", "Gurani", "Gujurati", "Greek", "Georgian", "Fulani", "Dari", "Chuvash", "Chinese", "Burmese", "Bulgarian", "Bengali",
    "Belorussian", "Bambora", "Azerbaijani", "Assamese", "Armenian", "Arabic", "Amharic"};
const char* language(unsigned c) { return c < sizeof kLanguage / sizeof *kLanguage ? kLanguage[c] : "invalid language code"; }

// TS 101 756 tables 14 and 15
const char* const kAnnounce[16] = {"Alarm", "Road Traffic flash", "Transport flash", "Warning/Service", "News flash", "Area weather flash",
    "Event announcement", "Special event", "Programme Information", "Sport report", "Financial report", "Reserved for future definition",
    "Reserved for future definition", "Reserved for future definition", "Reserved for future definition", "Reserved for future definition"};

// TS 101 756 tables 12 and 13
const char* const kPty[2][32] = {
    {"No programme type", "News", "Current Affairs", "Information", "Sport", "Education", "Drama", "Culture", "Science", "Varied",
     "Pop Music", "Rock Music", "Easy Listening Music", "Light Classical", "Serious Classical", "Other Music", "Weather/meteorology",
     "Finance/Business", "Children's programmes", "Social Affairs", "Religion", "Phone In", "Travel", "Leisure", "Jazz Music",
     "Country Music", "National Music", "Oldies Music", "Folk Music", "Documentary", "Not used", "Not used"},
    {"No program type", "News", "Information", "Sports", "Talk", "Rock", "Classic Rock", "Adult Hits", "Soft Rock", "Top 40", "Country",
     "Oldies", "Soft", "Nostalgia", "Jazz", "Classical", "Rhythm and Blues", "Soft Rhythm and Blues", "Foreign Language",
     "Religious Music", "Religious Talk", "Personality", "Public", "College", "rfu", "rfu", "rfu", "rfu", "rfu", "Weather", "Not used",
     "Not used"}};
const char* ptyName(int table, unsigned pty) {
    if (table < 1 || table > 2) return "unknown international table Id";
    return pty < 32 ? kPty[table - 1][pty] : "invalid programme type";
}

// TS 101 756 table 2
const char* dscty(int t) {
    switch (t) {
        case 0: return "Unspecified data";
        case 1: return "Traffic Message Channel (TMC)";
        case 2: return "Emergency Warning System (EWS)";
        case 3: return "Interactive Text Transmission System (ITTS)";
        case 4: return "Paging";
        case 5: return "Transparent Data Channel (TDC)";
        case 24: return "MPEG-2 Transport Stream, see ETSI TS 102 427";
        case 59: return "Embedded IP packets";
        case 60: return "Multimedia Object Transfer (MOT)";
        case 61: return "Proprietary service: no DSCTy signalled";
        case 62: case 63: return "Not used";
        default: return "Rfu";
    }
}

const char* const kCaMode[8] = {"Sub-channel CA", "Data Group CA", "MOT CA", "proprietary CA", "reserved", "reserved", "reserved", "reserved"};

const char* userApp(int t) {   // TS 101 756 table 16
    switch (t) {
        case 0x000: return "Reserved for future definition";
        case 0x001: return "Not used";
        case 0x002: return "MOT Slideshow";
        case 0x003: return "MOT Broadcast Web Site";
        case 0x004: return "TPEG";
        case 0x005: return "DGPS";
        case 0x006: return "TMC";
        case 0x007: return "EPG";
        case 0x008: return "DAB Java";
        case 0x009: return "DMB";
        case 0x44a: return "Journaline";
        default: return "Reserved for future applications";
    }
}

std::string pnum(unsigned p) {
    const unsigned minute = p & 0x3F, hour = (p >> 6) & 0x1F, day = (p >> 11) & 0x1F;
    if (day) return pf("day of month=%u time=%02u:%02u", day, hour, minute);
    if (hour == 0 && minute == 0) return "Status code: no meaningful PNum is currently provided";
    if (hour == 0 && minute == 1) return "Blank code: the current programme is not worth recording";
    if (hour == 0 && minute == 2) return "Interrupt code: the interrupt is unplanned (for example a traffic announcement)";
    return "invalid value";
}

std::string mjdDate(int mjd) {   // EN 62106 annex G, valid 1900-03-01 .. 2100-02-28
    static const char* wd[] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};
    static const char* mo[] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};
    const int y = (int)(((double)mjd - 15078.2) / 365.25);
    const int m = (int)(((double)mjd - 14956.1 - (int)((double)y * 365.25)) / 30.6001);
    const int d = mjd - 14956 - (int)((double)y * 365.25) - (int)((double)m * 30.6001);
    const int k = (m == 14 || m == 15) ? 1 : 0;
    const int year = y + k + 1900, mon = m - 1 - k * 12 - 1;
    if (d < 0 || mon < 0 || mon > 11) return "invalid MJD";
    return pf("%s %s %02d %d", wd[((mjd + 2) % 7 + 1) % 7], mo[mon], d, year);
}

std::string hexOf(const uint8_t* p, size_t n) {
    std::string s;
    for (size_t i = 0; i < n; i++) s += pf(i ? " %02X" : "%02X", p[i]);
    return s;
}

std::string ltoText(int lto) {   // 6 bit two's complement, half hours
    if (lto & 0x20) lto |= ~0x3F;
    return pf("%s%d:%02d", lto < 0 ? "-" : "+", std::abs(lto) >> 1, (std::abs(lto) & 1) * 30);
}

struct Fig {
    const uint8_t* f;      // the data field, zero padded after len
    int len;
    int cn, oe, pd;
    const DabFigContext* ctx;
    DabFigDecoded* out;
    void add(int lvl, const std::string& s) { out->lines.push_back({lvl, s}); }
    void err(const std::string& s) { out->errors.push_back(s); }
    uint32_t u16(int i) const { return (uint32_t)(f[i] << 8) | f[i + 1]; }
    uint32_t u32(int i) const { return ((uint32_t)f[i] << 24) | ((uint32_t)f[i + 1] << 16) | ((uint32_t)f[i + 2] << 8) | f[i + 3]; }
    void announce(int lvl, unsigned flags) { for (int j = 0; j < 16; j++) if (flags & (1u << j)) add(lvl, pf("- %s", kAnnounce[j])); }
};

// ------------------------------------------------------------------ FIG 0
void fig0_0(Fig& g) {   // EN 300 401 6.4
    const uint8_t* f = g.f;
    g.add(0, pf("Ensemble ID=0x%04X", g.u16(1)));
    g.add(0, pf("Country ID=%d", f[1] >> 4));
    g.add(0, pf("Ensemble reference=%d", (f[1] & 0x0F) * 256 + f[2]));
    const int ch = f[3] >> 6;
    g.add(0, pf("Change flag=%d", ch));
    g.add(0, pf("Alarm flag=%d", (f[3] >> 5) & 1));
    g.add(0, pf("CIF Count=%d/%d", f[3] & 0x1F, f[4]));
    if (ch) g.add(0, pf("Occurrence change=%d", f[5]));
}

void fig0_1(Fig& g) {   // 6.2.1
    const uint8_t* f = g.f;
    for (int i = 1; i <= g.len - 3;) {
        const int id = f[i] >> 2, start = ((f[i] & 3) << 8) | f[i + 1];
        g.add(0, pf("Sub-channel 0x%02X", id));
        g.add(1, pf("Subch=0x%x", id));
        g.add(1, pf("start_addr=%d", start));
        if (f[i + 2] & 0x80) {
            const int opt = (f[i + 2] >> 4) & 7, lvl = (f[i + 2] >> 2) & 3, size = ((f[i + 2] & 3) << 8) | f[i + 3];
            g.add(1, "form=long");
            if (opt == 0) g.add(1, pf("EEP=%d-A", lvl + 1));
            else if (opt == 1) g.add(1, pf("EEP=%d-B", lvl + 1));
            else g.err(pf("Invalid option %d protection %d", opt, lvl));
            g.add(1, pf("subch size=%d", size));
            i += 4;
        } else {
            const int sw = (f[i + 2] >> 6) & 1;
            g.add(1, "form=short");
            if (sw) g.err(pf("Invalid table_switch %d", sw));
            g.add(1, pf("table index=%d", f[i + 2] & 0x3F));
            i += 3;
        }
    }
}

void fig0_2(Fig& g) {   // 6.3.1
    const uint8_t* f = g.f;
    for (int k = 1; k < g.len;) {
        uint32_t sid;
        int ecc = 0, cid, sref;
        if (!g.pd) { sid = g.u16(k); cid = f[k] >> 4; sref = (f[k] & 0x0F) * 256 + f[k + 1]; k += 2; }
        else { sid = g.u32(k); ecc = f[k]; cid = f[k + 1] >> 4; sref = (f[k + 1] & 0x0F) * 65536 + f[k + 2] * 256 + f[k + 3]; k += 4; }
        const int local = f[k] >> 7, caid = (f[k] >> 4) & 7, ncomp = f[k] & 15;
        g.add(0, pf("Service 0x%X", sid));
        g.add(1, pf("Service ID=0x%X", sid));
        if (g.pd) g.add(1, pf("ECC=%d", ecc));
        g.add(1, pf("Country id=%d", cid));
        g.add(1, pf("Service reference=%d", sref));
        g.add(1, pf("Number of components=%d", ncomp));
        g.add(1, pf("Local flag=%d", local));
        g.add(1, pf("CAID=%d", caid));
        k++;
        g.add(1, "Components:");
        for (int i = 0; i < ncomp; i++, k += 2) {
            const int timd = f[k] >> 6, ps = (f[k + 1] >> 1) & 1, ca = f[k + 1] & 1, scty = f[k] & 0x3F, sub = f[k + 1] >> 2;
            g.add(2, pf("Component %d", i));
            g.add(3, pf("ID=%d", i));
            g.add(3, ps ? "primary=false" : "primary=true");
            if (timd == 0) {
                g.add(3, "Mode=audio stream");
                const char* n = scty == 0 ? "MPEG Foreground sound" : scty == 1 ? "MPEG Background sound" : scty == 2 ? "Multi Channel sound" :
                                scty == 63 ? "AAC sound" : "Unknown ASCTy";
                g.add(3, pf("ASCTy=%s (%d)", n, scty));
                g.add(3, pf("SubChannel ID=0x%02X", sub));
            } else if (timd == 1) {
                g.add(3, "Mode=data stream");
                g.add(3, pf("DSCTy=%d %s", scty, dscty(scty)));
                g.add(3, pf("SubChannel ID=0x%02X", sub));
            } else if (timd == 2) {
                g.add(3, "Mode=FIDC");
                g.add(3, pf("DSCTy=%d %s", scty, dscty(scty)));
                g.add(3, pf("Fast Information Data Channel ID=0x%02X", sub));
            } else {
                g.add(3, "Mode=MSC Packet");
                g.add(3, pf("SubChannel ID=0x%02X", sub));
            }
            g.add(3, pf("CA=%d", ca));
        }
    }
}

void fig0_3(Fig& g) {   // 6.3.2
    const uint8_t* f = g.f;
    for (int i = 1; i < g.len - 4;) {
        const int scid = (f[i] << 4) | (f[i + 1] >> 4), rfa = (f[i + 1] >> 1) & 7, caorg = f[i + 1] & 1;
        const int dg = (f[i + 2] >> 7) & 1, rfu = (f[i + 2] >> 6) & 1, ty = f[i + 2] & 0x3F;
        g.add(0, pf("Packet component SCId 0x%X", scid));
        g.add(1, pf("SCId=0x%X", scid));
        g.add(1, pf("CAOrg flag=%d CAOrg field %s", caorg, caorg ? "present" : "absent"));
        g.add(1, pf("DG flag=%d", dg));
        g.add(1, pf("DSCTy=%d %s", ty, dscty(ty)));
        g.add(1, pf("SubChId=0x%X", f[i + 3] >> 2));
        g.add(1, pf("Packet address=0x%X", ((f[i + 3] & 3) << 8) | f[i + 4]));
        if (rfa) g.err(pf("Rfa=%d invalid value", rfa));
        if (rfu) g.err(pf("Rfu=%d invalid value", rfu));
        i += 5;
        if (caorg) {
            if (i < g.len - 1) g.add(1, pf("CAOrg=0x%X CAMode=%d \"%s\" SharedFlag=0x%X%s", g.u16(i), f[i] >> 5, kCaMode[f[i] >> 5], f[i + 1], f[i + 1] ? "" : " invalid"));
            else g.err("Invalid figlen");
            i += 2;
        }
    }
}

void fig0_5(Fig& g) {   // 8.1.2
    const uint8_t* f = g.f;
    for (int i = 1; i < g.len - 1;) {
        if (!(f[i] >> 7)) {
            const int fic = (f[i] >> 6) & 1;
            g.add(0, pf("Component language %s 0x%X", fic ? "FIDCId" : "SubChId", f[i] & 0x3F));
            g.add(1, "form=short");
            g.add(1, pf("MSC/FIC flag=%d %s", fic, fic ? "FIC" : "MSC"));
            g.add(1, pf(fic ? "FIDCId=0x%X" : "SubChId=0x%X", f[i] & 0x3F));
            g.add(1, pf("Language=0x%X %s", f[i + 1], language(f[i + 1])));
            i += 2;
        } else {
            g.add(0, "Component language, long form");
            g.add(1, "form=long");
            if (i < g.len - 2) {
                if ((f[i] >> 4) & 7) g.err(pf("Rfa=%d invalid value", (f[i] >> 4) & 7));
                g.add(1, pf("SCId=0x%X", ((f[i] & 0x0F) << 8) | f[i + 1]));
                g.add(1, pf("Language=0x%X %s", f[i + 2], language(f[i + 2])));
            } else g.err("Long form FIG is too short");
            i += 3;
        }
    }
}

void fig0_6(Fig& g) {   // 8.1.15
    const uint8_t* f = g.f;
    for (int i = 1; i < g.len - 1;) {
        const int idl = (f[i] >> 7) & 1, la = (f[i] >> 6) & 1, sh = (f[i] >> 5) & 1, ils = (f[i] >> 4) & 1, lsn = ((f[i] & 0x0F) << 8) | f[i + 1];
        const int key = (g.oe << 15) | (g.pd << 14) | (sh << 13) | (ils << 12) | lsn;
        g.add(0, pf("Link LSN %d", lsn));
        g.add(1, pf("Id list flag=%d", idl));
        g.add(1, pf("LA=%d %s", la, la ? "active" : "inactive"));
        g.add(1, pf("S/H=%d %s", sh, sh ? "Hard" : "Soft"));
        g.add(1, pf("ILS=%d %s", ils, ils ? "international" : "national"));
        g.add(1, pf("LSN=%d", lsn));
        g.add(1, pf("database key=0x%04x", key));
        i += 2;
        if (!idl) {
            if (g.cn == 0) g.add(1, "CEI=true");
            continue;
        }
        if (i >= g.len) continue;
        const int n = f[i] & 15;
        if (!g.pd) {
            const int idlq = (f[i] >> 5) & 3, shd = (f[i] >> 4) & 1;
            g.add(1, pf("IdLQ=%d", idlq));
            g.add(1, pf("Shd=%d %s", shd, shd ? "b11-8 in 4-F are different services" : "single service"));
            g.add(1, "Id List:");
            const int w = ils ? 3 : 2;
            for (int j = 0; j < n && i + 1 + w * j + (w - 1) < g.len; j++) {
                const int o = i + 1 + w * j;
                const bool first = (j == 0 && g.oe == 0 && g.cn == 0) || idlq == 0;
                const char* kind = first ? "DAB SId" : idlq == 1 ? "RDS PI" : idlq == 2 ? "(AM-FM legacy)" : "DRM-AMSS service";
                g.add(2, "-");
                if (!ils) g.add(3, pf("%s=0x%X", kind, g.u16(o)));
                else g.add(3, pf("%s=ecc 0x%02X Id 0x%04X", kind, f[o], g.u16(o + 1)));
            }
            if (n == 0 && idlq == 1) g.err("deadlink");
            i += n * w + 1;
        } else {
            g.add(1, "Id List:");
            for (int j = 0; j < n && i + 4 + 4 * j < g.len; j++) g.add(2, pf("- 0x%X", g.u32(i + 1 + 4 * j)));
            i += n * 4 + 1;
        }
    }
}

void fig0_7(Fig& g) {   // 6.4.2
    if (g.len != 3) { g.err("FIG0/7 has incorrect length"); return; }
    const unsigned v = g.u16(1);
    g.add(0, pf("Services=%u", v >> 10));
    g.add(0, pf("Count=%u", v & 0x3FF));
}

void fig0_8(Fig& g) {   // 6.3.5
    const uint8_t* f = g.f;
    for (int i = 1; i < g.len - (2 + 2 * g.pd);) {
        uint32_t sid;
        if (!g.pd) { sid = g.u16(i); i += 2; } else { sid = g.u32(i); i += 4; }
        const int ext = f[i] >> 7, rfa = (f[i] >> 4) & 7, scids = f[i] & 15;
        g.add(0, pf("Component SId 0x%X SCIdS %d", sid, scids));
        g.add(1, pf("SId=0x%X", sid));
        g.add(1, pf("Ext flag=%d 8-bit Rfa %s", ext, ext ? "present" : "absent"));
        if (rfa) g.err(pf("Rfa=%d invalid value", rfa));
        g.add(1, pf("SCIdS=0x%X", scids));
        i++;
        if (i >= g.len) break;
        const int ls = f[i] >> 7;
        g.add(1, pf("L/S flag=%d %s", ls, ls ? "Long form" : "Short form"));
        if (!ls) {
            if (i < g.len - ext) {
                const int fic = (f[i] >> 6) & 1;
                if (!fic) g.add(1, pf("MSC/FIC flag=0 MSC, SubChId=0x%X", f[i] & 0x3F));
                else g.add(1, pf("MSC/FIC flag=1 FIC, FIDCId=0x%X", f[i] & 0x3F));
                if (ext && f[i + 1]) g.err(pf("Rfa=0x%X invalid value", f[i + 1]));
            }
            i += 1 + ext;
        } else {
            if (i < g.len - 1) {
                if ((f[i] >> 4) & 7) g.err(pf("Rfa=%d invalid value", (f[i] >> 4) & 7));
                g.add(1, pf("SCId=0x%X", ((f[i] & 0x0F) << 8) | f[i + 1]));
            }
            i += 2;
        }
    }
}

void fig0_9(Fig& g) {   // 8.1.3.2
    const uint8_t* f = g.f;
    int i = 1;
    if (i >= g.len - 2) return;
    const int ext = f[i] >> 7, uniq = (f[i] >> 6) & 1;
    int lto = f[i] & 0x3F;
    g.add(0, pf("Ext flag=%d extended field %s", ext, ext ? "present" : "absent"));
    g.add(0, pf("LTO uniq=%d %s", uniq, uniq ? "several time zones" : "one time zone (time specified by Ensemble LTO)"));
    g.add(0, pf("Ensemble LTO=0x%X UTC%s", lto, ltoText(lto).c_str()));
    if (lto & 0x20) lto |= ~0x3F;
    if (std::abs(lto) > 24) g.err("LTO out of range -12 hours to +12 hours");
    g.add(0, pf("Ensemble ECC=0x%X", f[i + 1]));
    g.add(0, pf("International Table Id=0x%X", f[i + 2]));
    g.add(0, pf("database key=0x%x", (g.oe << 1) | g.pd));
    i += 3;
    if (!ext) return;
    g.add(0, "Subfields:");
    while (i < g.len) {
        const int n = f[i] >> 6, l = f[i] & 0x3F;
        g.add(1, "Subfield");
        g.add(2, pf("Number of services=%d", n));
        g.add(2, pf("LTO=0x%X UTC%s", l, ltoText(l).c_str()));
        int sl = l;
        if (sl & 0x20) sl |= ~0x3F;
        if (std::abs(sl) > 24) g.err("LTO in extended field out of range -12 hours to +12 hours");
        if (n == 0 && l == 0) g.add(2, "CEI=true");
        i++;
        if (!g.pd) {
            if (i < g.len) {
                g.add(2, pf("ECC=0x%X", f[i]));
                i++;
                for (int j = i; j < i + n * 2 && j < g.len; j += 2) g.add(2, pf("SId=0x%X", g.u16(j)));
                i += n * 2;
            }
        } else {
            for (int j = i; j < i + n * 4 && j < g.len; j += 4) g.add(2, pf("SId=0x%X", g.u32(j)));
            i += n * 4;
        }
    }
}

void fig0_10(Fig& g) {   // 8.1.3.1
    const uint8_t* f = g.f;
    const uint32_t mjd = ((uint32_t)(f[1] & 0x7F) << 10) | ((uint32_t)f[2] << 2) | (f[3] >> 6);
    const bool utc = f[3] & 8;
    const int hh = ((f[3] & 7) << 2) | (f[4] >> 6), mm = f[4] & 0x3F;
    g.add(0, utc ? "form=long" : "form=short");
    g.add(0, pf("MJD=0x%X %s", mjd, mjdDate((int)mjd).c_str()));
    g.add(0, pf("LSI=%u", (f[3] >> 5) & 1));
    g.add(0, pf("ConfInd=%u", (f[3] >> 4) & 1));
    if (utc) g.add(0, pf("UTC Time=%02d:%02d:%02d.%d", hh, mm, f[5] >> 2, ((f[5] & 3) << 8) | f[6]));
    else g.add(0, pf("UTC Time=%02d:%02d", hh, mm));
}

void mainIdCheck(Fig& g, int mainId) {
    const int mode = g.ctx->mode;
    if ((mode == 1 || mode == 2 || mode == 4) && mainId > 69) g.err(pf("invalid value for transmission mode %d", mode));
    else if (mode == 3 && mainId > 5) g.err(pf("invalid value for transmission mode %d", mode));
}

void fig0_11(Fig& g) {   // 8.1.16.1
    const uint8_t* f = g.f;
    for (int i = 1; i < g.len - 1;) {
        const int gaty = f[i] >> 4, ge = (f[i] >> 3) & 1, region = ((f[i] & 7) << 8) | f[i + 1];
        const int key = (g.oe << 12) | (g.pd << 11) | region;
        i += 2;
        g.add(0, pf("Region 0x%X", region));
        g.add(1, pf("GATy=%d", gaty));
        g.add(1, gaty == 0 ? "Geographical area=defined by TII list" :
                 gaty == 1 ? "Geographical area=defined as a spherical rectangle by the co-ordinates of one corner and its extents" :
                             "Geographical area=reserved for future use");
        g.add(1, pf("G/E flag=%d %s coverage area", ge, ge ? "Global" : "Ensemble"));
        g.add(1, pf("RegionId=0x%X", region));
        g.add(1, pf("database key=0x%X", key));
        if (gaty == 0) {
            if (i >= g.len) continue;
            if (f[i] >> 5) g.err(pf("Rfu=%d invalid value", f[i] >> 5));
            const int ntii = f[i] & 0x1F;
            g.add(1, pf("Length of TII list=%d", ntii));
            if (!ntii) g.add(1, "CEI=true");
            i++;
            int j = 0;
            for (; i < g.len - 1 && j < ntii; j++) {
                const int main = f[i] & 0x7F;
                if (f[i] >> 7) g.err(pf("Rfa=%d invalid value, MainId=0x%X", f[i] >> 7, main));
                else g.add(1, pf("MainId=0x%X", main));
                mainIdCheck(g, main);
                if (f[i + 1] >> 5) g.err(pf("Rfa=%d invalid value", f[i + 1] >> 5));
                const int nsub = f[i + 1] & 0x1F;
                g.add(1, pf("Length of SubId=%d", nsub));
                i += 2;
                int bit = 3, sub = 0, k = 0;
                for (; i < g.len && k < nsub; k++) {
                    if (bit >= 0) {
                        sub |= (f[i] >> bit) & 0x1F;
                        g.add(2, pf("SubId=0x%X", sub));
                        if (sub == 0 || sub > 23) g.err(pf("Invalid SubId=0x%X", sub));
                        bit -= 5;
                        sub = 0;
                    }
                    if (bit < 0) { sub = (f[i] << std::abs(bit)) & 0x1F; bit += 8; i++; }
                }
                if (bit > 3) i++;
                if (k < nsub) g.err(pf("%d SubId missing, fig length too short !", nsub - k));
            }
            if (j < ntii) g.err(pf("%d Transmitter group missing, fig length too short !", ntii - j));
        } else if (gaty == 1) {
            if (i < g.len - 6) {
                const int16_t la = (int16_t)g.u16(i), lo = (int16_t)g.u16(i + 2);
                const double lat = la * 90.0 / 32768, lng = lo * 180.0 / 32768;
                g.add(1, pf("Lat Lng coarse=0x%X 0x%X => %f, %f", (unsigned)(uint16_t)la, (unsigned)(uint16_t)lo, lat, lng));
                const int elat = (f[i + 4] << 4) | (f[i + 5] >> 4), elng = ((f[i + 5] & 0x0F) << 8) | f[i + 6];
                g.add(1, pf("Extent Lat Lng=0x%X 0x%X => %f, %f", elat, elng, lat + elat * 90.0 / 32768, lng + elng * 180.0 / 32768));
            } else g.err("Coordinates missing, fig length too short !");
            i += 7;
        } else {
            g.add(1, pf("stop Region definition iteration %d/%d", i, g.len));
            g.err("Stopping iteration because Rfu encountered");
            break;
        }
    }
}

void fig0_13(Fig& g) {   // 8.1.20
    const uint8_t* f = g.f;
    int k = 1;
    uint32_t sid;
    if (!g.pd) { sid = g.u16(k); k += 2; } else { sid = g.u32(k); k += 4; }
    const int scids = f[k] >> 4, no = f[k] & 15;
    k++;
    g.add(0, pf("SId=0x%X", sid));
    g.add(0, pf("SCIdS=%u", scids));
    g.add(0, pf("User applications(%d):", no));
    for (int n = 0; n < no && k + 2 <= g.len; n++) {
        const int type = (f[k] << 3) | (f[k + 1] >> 5), ulen = f[k + 1] & 0x1F;
        k += 2;
        g.add(1, pf("User application %d", n));
        g.add(2, pf("User Application=%d '%s'", type, userApp(type)));
        g.add(2, pf("length=%u", ulen));
        if (ulen < 2) { k += ulen; continue; }
        int eff = ulen;
        if (!g.pd) {   // programme services carry the X-PAD data field first
            const int caorg = (f[k] >> 6) & 1;
            g.add(2, pf("CAflag=%d", (f[k] >> 7) & 1));
            g.add(2, pf("CAOrgflag=%d", caorg));
            g.add(2, pf("XPAD_AppTy=%d", f[k] & 0x1F));
            g.add(2, pf("DGflag=%d", (f[k + 1] >> 7) & 1));
            g.add(2, pf("DSCTy=%d", f[k + 1] & 0x3F));
            k += 2;
            eff -= 2;
            if (caorg) {
                if (ulen < 4) g.err("User Application Data Length too short to contain CAOrg!");
                else { g.add(2, pf("ca_org=%u", g.u16(k))); k += 2; eff -= 2; }
            }
        }
        std::string ua = "UA Data=[";
        for (int i = 0; i < eff && k + i < g.len; i++) ua += pf(i ? ", 0x%02x" : "0x%02x", f[k + i]);
        g.add(2, ua + "]");
        k += eff;
    }
}

void fig0_14(Fig& g) {   // 6.2.2
    static const char* const fec[4] = {"no FEC scheme applied", "FEC scheme applied according to ETSI EN 300 401 clause 5.3.5",
        "reserved for future definition", "reserved for future definition"};
    for (int i = 1; i < g.len; i++) {
        g.add(0, pf("Sub-channel 0x%X", g.f[i] >> 2));
        g.add(1, pf("SubChId=0x%X", g.f[i] >> 2));
        g.add(1, pf("FEC scheme=%d %s", g.f[i] & 3, fec[g.f[i] & 3]));
    }
}

void fig0_16(Fig& g) {   // 8.1.4 and 8.1.10.3
    const uint8_t* f = g.f;
    for (int i = 1; i < g.len - 4;) {
        const int rfa = f[i + 4] >> 6, rfu = (f[i + 4] >> 2) & 15, cont = (f[i + 4] >> 1) & 1, upd = f[i + 4] & 1;
        g.add(0, pf("Programme number of SId 0x%X", g.u16(i)));
        g.add(1, pf("SId=0x%X", g.u16(i)));
        g.add(1, pf("PNum=0x%X ", g.u16(i + 2)) + pnum(g.u16(i + 2)));
        if (rfa) g.err(pf("Rfa=%d invalid value", rfa));
        if (rfu) g.err(pf("Rfu=0x%X invalid value", rfu));
        g.add(1, pf("Continuation flag=%d, the programme will %s", cont, cont ? "be interrupted but continued later" : "not be subject to a planned interruption"));
        g.add(1, pf("Update flag=%d %sre-direction", upd, upd ? "" : "no "));
        i += 5;
        if (upd) {
            if (i < g.len - 1) {
                g.add(1, pf("New SId=0x%X", g.u16(i)));
                if (i < g.len - 3) g.add(1, pf("New PNum=0x%X ", g.u16(i + 2)) + pnum(g.u16(i + 2)));
                else g.err("missing New PNum !");
            } else g.err("missing New SId and New PNum !");
            i += 4;
        }
    }
}

void fig0_17(Fig& g) {   // 8.1.5
    const uint8_t* f = g.f;
    for (int i = 1; i < g.len - 3;) {
        const int sd = f[i + 2] >> 7, ps = (f[i + 2] >> 6) & 1, l = (f[i + 2] >> 5) & 1, cc = (f[i + 2] >> 4) & 1;
        g.add(0, pf("Programme type of SId 0x%X", g.u16(i)));
        g.add(1, pf("SId=0x%X", g.u16(i)));
        g.add(1, pf("S/D=%d Programme Type codes and language (when present), %srepresent the current programme contents", sd, sd ? "" : "may not "));
        g.add(1, pf("P/S=%d %s service component", ps, ps ? "secondary" : "primary"));
        g.add(1, pf("L flag=%d language field %s", l, l ? "present" : "absent"));
        g.add(1, pf("CC flag=%d complementary code and preceding Rfa and Rfu fields %s", cc, cc ? "present" : "absent"));
        if (f[i + 2] & 15) g.err(pf("Rfa=0x%X invalid value", f[i + 2] & 15));
        i += 3;
        if (l) {
            if (i < g.len) g.add(1, pf("Language=0x%X %s", f[i], language(f[i])));
            else g.err("Language= invalid FIG length");
            i++;
        }
        if (i < g.len) {
            if (f[i] >> 6) g.err(pf("Rfa=0x%X invalid value", f[i] >> 6));
            if ((f[i] >> 5) & 1) g.err("Rfu=1 invalid value");
            g.add(1, pf("Int code=0x%X %s", f[i] & 0x1F, ptyName(g.ctx->intlTable, f[i] & 0x1F)));
            i++;
        } else g.err("Int code: invalid FIG length");
        if (cc) {
            if (i < g.len) {
                if (f[i] >> 6) g.err(pf("Rfa=0x%X invalid value", f[i] >> 6));
                if ((f[i] >> 5) & 1) g.err("Rfu=1 invalid value");
                g.add(1, pf("Comp code=0x%X %s", f[i] & 0x1F, ptyName(g.ctx->intlTable, f[i] & 0x1F)));
                i++;
            } else g.err("Comp code= invalid FIG length");
        }
    }
}

void fig0_18(Fig& g) {   // 8.1.6.1
    const uint8_t* f = g.f;
    for (int i = 1; i < g.len - 4;) {
        const unsigned asu = g.u16(i + 2);
        const int ncl = f[i + 4] & 0x1F;
        g.add(0, pf("Announcement support of SId 0x%X", g.u16(i)));
        g.add(1, pf("SId=0x%X", g.u16(i)));
        g.add(1, pf("Asu flags=0x%04x", asu));
        if (f[i + 4] >> 5) g.err(pf("Rfa=%d invalid value", f[i + 4] >> 5));
        g.add(1, pf("Number of clusters=%d", ncl));
        g.add(1, pf("database key=0x%05x", (g.oe << 17) | (g.pd << 16) | g.u16(i)));
        if (!ncl && !asu) g.add(1, "CEI=true");
        i += 5;
        std::string cl;
        int j = 0;
        for (; j < ncl && i < g.len; j++, i++) cl += pf(j ? ", 0x%X" : "0x%X", f[i]);
        g.add(1, "Cluster Ids: [" + cl + "]");
        if (j < ncl) g.err("missing Cluster Id, fig length too short !");
        g.add(1, "Announcements:");
        g.announce(2, asu);
    }
}

void fig0_19(Fig& g) {   // 8.1.6.2
    const uint8_t* f = g.f;
    for (int i = 1; i < g.len - 3;) {
        const unsigned asw = g.u16(i + 1);
        const int nw = f[i + 3] >> 7, rg = (f[i + 3] >> 6) & 1;
        g.add(0, pf("Announcement switching, cluster 0x%02x", f[i]));
        g.add(1, pf("Cluster Id=0x%02x", f[i]));
        g.add(1, pf("Asw flags=0x%04x", asw));
        g.add(1, pf("New flag=%d %s", nw, nw ? "new" : "repeat"));
        g.add(1, pf("Region flag=%d last byte %s", rg, rg ? "present" : "absent"));
        g.add(1, pf("SubChId=%d", f[i + 3] & 0x3F));
        if (rg) {
            if (i < g.len - 4) {
                if (f[i + 4] >> 6) g.err(pf("Rfa=%d invalid value", f[i + 4] >> 6));
                g.add(1, pf("Region Lower Part=0x%02x", f[i + 4] & 0x3F));
            } else g.err("missing Region Lower Part, fig length too short !");
        }
        g.add(1, "Announcement switching:");
        g.announce(2, asw);
        i += 4 + rg;
    }
}

void fig0_21(Fig& g) {   // 8.1.8
    const uint8_t* f = g.f;
    int i = 1;
    while (i < g.len) {
        const int region = (f[i] << 3) | (f[i + 1] >> 5), flen = f[i + 1] & 0x1F;
        g.add(0, pf("Frequency information, region 0x%03x", region));
        g.add(1, pf("RegionId=0x%03x", region));
        g.add(1, pf("Len=%d Bytes", flen));
        i += 2;
        const int start = i;
        g.add(1, "FIs:");
        for (int fi = 0; i < start + flen; fi++) {
            if (i + 3 > g.len) { g.err("FIG0/21 too small!"); break; }
            const unsigned id = g.u16(i);
            const int rm = f[i + 2] >> 4, cont = (f[i + 2] >> 3) & 1, nfl = f[i + 2] & 7;
            g.add(2, pf("Frequency list %d", fi));
            g.add(3, pf("Length Freq list=%d", nfl));
            i += 3;
            const char* idf = "invalid";
            switch (rm) {
                case 0: case 1: idf = "EId"; break;
                case 6: idf = "DRM Service Id"; break;
                case 8: idf = "RDS PI"; break;
                case 9: case 10: case 12: idf = "Dummy"; break;
                case 14: idf = "AMSS Service Id"; break;
                default: g.err("R&M invalid"); break;
            }
            g.add(3, pf("ID field=0x%X %s", id, idf));
            const char* rms = "Rfu";
            switch (rm) {
                case 0: rms = "DAB ensemble, no local windows"; break;
                case 6: rms = "DRM"; break;
                case 8: rms = "FM with RDS"; break;
                case 9: rms = "FM without RDS"; break;
                case 10: rms = "AM (MW in 9 kHz steps & LW)"; break;
                case 12: rms = "AM (MW in 5 kHz steps & SW)"; break;
                case 14: rms = "AMSS"; break;
                default: g.err("R&M is Rfu"); break;
            }
            g.add(3, pf("R&M=0x%1x %s", rm, rms));
            std::string cs;
            if (g.oe == 0 || (rm != 6 && (rm < 8 || rm > 10) && rm != 12 && rm != 14)) {
                if (rm == 0 || rm == 1) cs = cont ? ", continuous output possible" : ", continuous output not expected";
                else if (rm == 6) cs = cont ? ", compensating time delay on DRM audio signal" : ", no compensating time delay on DRM audio signal";
                else if (rm == 8 || rm == 9) cs = cont ? ", compensating time delay on FM audio signal" : ", no compensating time delay on FM audio signal";
                else if (rm == 10 || rm == 12 || rm == 14) cs = cont ? ", compensating time delay on AM audio signal" : ", no compensating time delay on AM audio signal";
                else cs = ", Rfu";
            } else { cs = ", reserved for future addition"; g.err("Rfu"); }
            g.add(3, pf("Continuity flag=%d", cont) + cs);
            g.add(3, pf("database key=0x%09llX", ((unsigned long long)g.oe << 32) | ((unsigned long long)g.pd << 31) | ((unsigned long long)region << 20) | ((unsigned long long)id << 4) | (unsigned)rm));
            if (!nfl) g.add(3, "CEI=true");
            g.add(3, "Frequency Information:");
            switch (rm) {
                case 0: case 1: {
                    if (nfl % 3) g.err("Length of freq list incorrect size");
                    for (int k = 0; k < nfl / 3; k++) {
                        g.add(4, "-");
                        if (i + 3 > g.len) { g.err(pf("FIG 0/21 too small for FI %d, freq %d", fi, k)); break; }
                        const int ctl = f[i] >> 3;
                        const unsigned freq = 16u * (((unsigned)(f[i] & 7) << 16) | ((unsigned)f[i + 1] << 8) | f[i + 2]);
                        i += 3;
                        if (!freq) { g.err(pf("Frequency not to be used (0) in FI %d, freq %d", fi, k)); continue; }
                        const int tm = (ctl >> 1) & 7;
                        if (!(ctl & 0x10)) {
                            g.add(5, pf("%u kHz", freq));
                            g.add(5, (ctl & 1) ? "no geographically adjacent area" : "geographically adjacent area");
                            if (tm == 0) g.add(5, "no transmission mode signalled");
                            else if (tm <= 4) g.add(5, pf("transmission mode %d", tm));
                            else g.add(5, pf("invalid transmission mode 0x%x", tm));
                        } else g.add(5, pf("%u kHz, invalid Control field b23 0x%x", freq, ctl));
                    }
                    break;
                }
                case 8: case 9: case 10:
                    for (int k = 0; k < nfl; k++, i++) {
                        g.add(4, "-");
                        if (i + 1 > g.len) { g.err(pf("FIG 0/21 too small for FI %d, freq %d", fi, k)); break; }
                        const unsigned fr = f[i];
                        if (!fr) { g.err("Frequency not to be used (0)"); continue; }
                        if (rm == 10) g.add(5, pf("%u kHz", fr < 16 ? 144 + fr * 9 : 387 + fr * 9));
                        else g.add(5, pf("%.1f MHz", 87.5 + fr * 0.1));
                    }
                    break;
                case 12:
                    if (nfl % 2) g.err("Length of freq list incorrect size");
                    for (int k = 0; k < nfl / 2; k++, i += 2) {
                        g.add(4, "-");
                        if (i + 2 > g.len) { g.err(pf("FIG 0/21 too small for FI %d, freq %d", fi, k)); break; }
                        const unsigned fr = 5 * g.u16(i);
                        if (fr) g.add(5, pf("%u kHz", fr)); else g.err("Frequency not to be used (0)");
                    }
                    break;
                case 6: case 14: {
                    if (i + 1 > g.len) { g.err(pf("FIG 0/21 too small for control of FI %d", fi)); break; }
                    const int nf = std::max(0, nfl - 1) / 2;
                    if (nfl < 1 || (nfl - 1) % 2) g.err("Length of freq list incorrect size");
                    const uint32_t id2 = f[i++];
                    for (int k = 0; k < nf; k++, i += 2) {
                        g.add(4, "-");
                        if (i + 2 > g.len) { g.err(pf("FIG 0/21 too small for FI %d, freq %d", fi, k)); break; }
                        const unsigned fr = ((f[i] & 0x7F) << 8) | f[i + 1];
                        if (fr) g.add(5, pf("%u kHz", fr)); else g.err("Frequency not to be used (0)");
                        g.add(5, pf("%s Service Id 0x%X", rm == 6 ? "DRM" : "AMSS", (id2 << 16) | id));
                    }
                    break;
                }
                default:
                    g.err("Invalid R&M");
                    i += nfl;   // skip the list: its layout is unknown
                    break;
            }
        }
    }
}

void fig0_22(Fig& g) {   // 8.1.9
    const uint8_t* f = g.f;
    for (int i = 1; i < g.len;) {
        const int ms = f[i] >> 7, main = f[i] & 0x7F;
        const int key = (g.oe << 8) | (g.pd << 7) | main;
        g.add(0, pf("%s identifier 0x%X", ms ? "Sub" : "Main", main));
        g.add(1, pf("M/S=%d %sidentifier", ms, ms ? "Sub-" : "Main "));
        g.add(1, pf("MainId=0x%X", main));
        mainIdCheck(g, main);
        g.add(1, pf("database key=0x%X", key));
        i++;
        if (!ms) {
            if (i < g.len - 4) {
                const int16_t lac = (int16_t)g.u16(i), loc = (int16_t)g.u16(i + 2);
                const int laf = f[i + 4] >> 4, lof = f[i + 4] & 15;
                const double lat = (double)(((int32_t)lac << 4) | laf) * 90 / 524288, lng = (double)(((int32_t)loc << 4) | lof) * 180 / 524288;
                g.add(1, pf("Lat Lng coarse=0x%X 0x%X, Lat Lng fine=0x%X 0x%X => Lat Lng=%f, %f", (unsigned)(uint16_t)lac, (unsigned)(uint16_t)loc, laf, lof, lat, lng));
                i += 5;
            } else { g.err("invalid length of Latitude Longitude coarse fine"); break; }
        } else {
            if (i >= g.len) { g.err("invalid fig length or Number of SubId fields length"); break; }
            if (f[i] >> 3) g.err(pf("Rfu=%d invalid value", f[i] >> 3));
            const int n = f[i] & 7;
            g.add(1, pf("Number of SubId fields=%d%s", n, n ? "" : ", CEI"));
            i++;
            g.add(1, "SubId Fields:");
            const auto pos = g.ctx->mainPos.find(key);
            for (int j = i; j < i + n * 6 && j < g.len - 5; j += 6) {
                const int sub = f[j] >> 3, td = ((f[j] & 3) << 8) | f[j + 1];
                const int16_t dla = (int16_t)g.u16(j + 2), dlo = (int16_t)g.u16(j + 4);
                g.add(2, pf("Transmitter SubId 0x%X", sub));
                g.add(3, pf("SubId=0x%X", sub));
                if (sub == 0 || sub > 23) g.err("invalid value");
                g.add(3, pf("TD=%d us", td));
                g.add(3, pf("Lat Lng offset=0x%X 0x%X", (unsigned)(uint16_t)dla, (unsigned)(uint16_t)dlo));
                double lat = 90.0 * dla / 524288, lng = 180.0 * dlo / 524288;
                if (pos != g.ctx->mainPos.end()) {
                    g.add(3, pf("Lat Lng=%f, %f", lat + pos->second.first, lng + pos->second.second));
                } else {
                    g.add(3, pf("Lat Lng=%f, %f wrong value because Main identifier latitude/longitude not available", lat, lng));
                }
            }
            i += n * 6;
        }
    }
}

void fig0_24(Fig& g) {   // 8.1.10.2
    const uint8_t* f = g.f;
    for (int i = 1; i < g.len - (g.pd + 1) * 2;) {
        uint32_t sid;
        if (!g.pd) { sid = g.u16(i); i += 2; } else { sid = g.u32(i); i += 4; }
        const int n = f[i] & 15;
        g.add(0, pf("Other ensembles of service 0x%X", sid));
        g.add(1, pf("PD=%d", g.pd));
        g.add(1, pf("SId=0x%X", sid));
        g.add(1, pf("CAId=%d", (f[i] >> 4) & 7));
        g.add(1, pf("Number of EId=%d", n));
        g.add(1, pf("database key=%09llu", ((unsigned long long)g.oe << 33) | ((unsigned long long)g.pd << 32) | sid));
        if (f[i] >> 7) g.err("Rfa=1 invalid value");
        if (!n) g.add(1, "CEI=true");
        i++;
        std::string e;
        for (int j = i, k = 0; j < i + n * 2 && j < g.len; j += 2, k++) e += pf(k ? ", 0x%04x" : "0x%04x", g.u16(j));
        g.add(1, "EIds: [" + e + "]");
        i += n * 2;
    }
}

void fig0_25(Fig& g) {   // 8.1.10.5.1
    const uint8_t* f = g.f;
    for (int i = 1; i < g.len - 4;) {
        const unsigned asu = g.u16(i + 2);
        const int n = f[i + 4] & 15;
        g.add(0, pf("Other ensemble announcement support of SId 0x%X", g.u16(i)));
        g.add(1, pf("SId=0x%X", g.u16(i)));
        g.add(1, pf("Asu flags=0x%X", asu));
        g.add(1, pf("Number of EIds=%d", n));
        if (f[i + 4] >> 4) g.err(pf("Rfu=%d invalid value", f[i + 4] >> 4));
        g.add(1, pf("database key=0x%05x", (g.oe << 17) | (g.pd << 16) | g.u16(i)));
        if (!n) g.add(1, "CEI=true");
        i += 5;
        std::string e;
        int j = 0;
        for (; j < n && i < g.len - 1; j++, i += 2) e += pf(j ? ", 0x%04x" : "0x%04x", g.u16(i));
        g.add(1, "EIds: [" + e + "]");
        if (j < n) g.err("missing EId, fig length too short !");
        g.add(1, "OE Announcement support:");
        g.announce(2, asu);
    }
}

void fig0_26(Fig& g) {   // 8.1.10.5.2
    const uint8_t* f = g.f;
    for (int i = 1; i < g.len - 6;) {
        const unsigned asw = g.u16(i + 1);
        const int nw = f[i + 3] >> 7, rg = (f[i + 3] >> 6) & 1;
        g.add(0, pf("Other ensemble announcement switching, cluster 0x%X", f[i]));
        g.add(1, pf("Cluster Id Current Ensemble=0x%X", f[i]));
        g.add(1, pf("Asw flags=0x%X", asw));
        g.add(1, pf("New flag=%d %s announcement", nw, nw ? "newly introduced" : "repeated"));
        g.add(1, pf("Region flag=%d last byte %s", rg, rg ? "present" : "absent. The announcement concerns the whole service area"));
        g.add(1, pf("Region Id Current Ensemble=0x%X", f[i + 3] & 0x3F));
        g.add(1, pf("EId Other Ensemble=0x%X", g.u16(i + 4)));
        g.add(1, pf("Cluster Id Other Ensemble=0x%X", f[i + 6]));
        i += 7;
        if (rg) {
            if (i < g.len) {
                if (f[i] >> 6) g.err(pf("Rfa=%d invalid value", f[i] >> 6));
                g.add(1, pf("Region Id Other Ensemble=0x%X", f[i] & 0x3F));
            } else g.err("missing Region Id Other Ensemble, fig length too short !");
            i++;
        }
        g.add(1, "Announcement switching:");
        g.announce(2, asw);
    }
}

void fig0_27(Fig& g) {   // 8.1.11.2.1
    const uint8_t* f = g.f;
    for (int i = 1; i < g.len - 2;) {
        const int n = f[i + 2] & 15;
        g.add(0, pf("FM announcement support of SId 0x%X", g.u16(i)));
        g.add(1, pf("SId=0x%X", g.u16(i)));
        if (f[i + 2] >> 4) g.err(pf("Rfu=%d invalid value", f[i + 2] >> 4));
        g.add(1, pf("Number of PI codes=%d", n));
        if (n > 12) g.err(pf("Number of PI codes=%d > 12 (maximum value)", n));
        g.add(1, pf("database key=0x%02X", (g.oe << 5) | (g.pd << 4) | n));
        if (!n) g.add(1, "CEI=true");
        i += 3;
        g.add(1, "PI Codes:");
        int j = 0;
        for (; j < n && i < g.len - 1; j++, i += 2) g.add(2, pf("- 0x%X", g.u16(i)));
        if (j != n) g.err("fig length too short !");
    }
}

void fig0_28(Fig& g) {   // 8.1.11.2.2
    const uint8_t* f = g.f;
    for (int i = 1; i < g.len - 3; i += 4) {
        const int nw = f[i + 1] >> 7;
        g.add(0, pf("FM announcement switching, cluster 0x%X", f[i]));
        g.add(1, pf("Cluster Id Current Ensemble=0x%X", f[i]));
        if (!f[i]) g.err("Cluster Id Current Ensemble invalid value 0");
        g.add(1, pf("New flag=%d %s announcement", nw, nw ? "newly introduced" : "repeated"));
        if ((f[i + 1] >> 6) & 1) g.err("Rfa=1 invalid value");
        g.add(1, pf("Region Id Current Ensemble=0x%X", f[i + 1] & 0x3F));
        g.add(1, pf("PI=0x%X", g.u16(i + 2)));
    }
}

void fig0_31(Fig& g) {   // 8.1.12
    const uint8_t* f = g.f;
    if (g.len != 7) g.err(pf("invalid length %d, expecting 7", g.len));
    const uint32_t t0 = g.u32(1);
    g.add(0, pf("FIG type 0 flag field=0x%X", t0));
    g.add(0, pf("FIG type 1 flag field=0x%X", f[5]));
    g.add(0, pf("FIG type 2 flag field=0x%X", f[6]));
    for (int j = 0; j < 32; j++) {
        if (!(t0 & (1u << j))) continue;
        if (j <= 5 || j == 8 || j == 10 || j == 13 || j == 14 || j == 19 || j == 26 || j == 28)
            g.err(pf("OE %d FIG 0/%d=carried in AIC, invalid configuration, shall always be carried entirely in the FIC", g.oe, j));
        else if (j == 21 || j == 24 || g.oe == 0) g.add(1, pf("OE %d FIG 0/%d=carried in AIC, same shall be carried in FIC", g.oe, j));
        else g.add(1, pf("OE %d FIG 0/%d=carried in AIC, may be carried entirely in AIC", g.oe, j));
    }
    for (int t = 1; t <= 2; t++)
        for (int j = 0; j < 8; j++)
            if (f[4 + t] & (1u << j))
                g.add(1, pf("OE %d FIG %d/%d=carried in AIC, %s", g.oe, t, j, g.oe == 0 ? "same shall be carried in FIC" : "may be carried entirely in AIC"));
}

// ------------------------------------------------------------------ FIG 1 and 2
std::string labelUtf8(int charset, const uint8_t* p, size_t n) { return text::clean(text::dabCharset(charset, p, n)); }

const char* xpadApp(int t) { return t == 2 ? "DLS" : t == 12 ? "MOT" : "?"; }

void fig1(Fig& g, int cs, int ext) {   // 8.1.13, 8.1.14 (16 character labels with a short label mask)
    const uint8_t* f = g.f;
    g.add(0, pf("Charset=%d", cs));
    if (g.len < 19) { g.err("FIG 1 too short for a label"); return; }
    const uint8_t* lab = f + g.len - 18;
    const unsigned mask = g.u16(g.len - 2);
    switch (ext) {
        case 0: g.add(0, pf("Ensemble ID=0x%04X", g.u16(1))); break;
        case 1: g.add(0, pf("Service ID=0x%04X", g.u16(1))); break;
        case 4: {
            const int pd = f[1] >> 7;
            g.add(0, pf(pd ? "Service ID=0x%08X" : "Service ID=0x%04X", pd ? g.u32(2) : g.u16(2)));
            g.add(0, pf("Service Component ID=0x%04X", f[1] & 15));
            break;
        }
        case 5: g.add(0, pf("Service ID=0x%08X", g.u32(1))); break;
        case 6: {
            const int pd = f[1] >> 7;
            g.add(0, pf(pd ? "Service ID=0x%08X" : "Service ID=0x%04X", pd ? g.u32(2) : g.u16(2)));
            g.add(0, pf("Service Component ID=0x%04X", f[1] & 15));
            const int app = f[pd ? 6 : 4] & 0x1F;
            g.add(0, pf("X-PAD App=%02X (%s)", app, xpadApp(app)));
            break;
        }
        default: g.add(0, "data: " + hexOf(f + 1, (size_t)(g.len - 1))); return;
    }
    g.add(0, "Label=\"" + labelUtf8(cs, lab, 16) + "\"");
    g.add(0, pf("Short label mask=0x%04X", mask));
    std::string sel;
    for (int i = 0; i < 16; i++) if (mask & (0x8000u >> i)) sel += (char)lab[i];
    g.add(0, "Short label=\"" + labelUtf8(cs, (const uint8_t*)sel.data(), sel.size()) + "\"");
}

int fig2IdLen(const uint8_t* f, int ext) {
    switch (ext) {
        case 0: case 1: return 2;
        case 4: return (f[1] >> 7) ? 5 : 3;
        case 5: return 4;
        case 6: return (f[1] >> 7) ? 6 : 4;
        default: return 0;
    }
}

std::string ucsOrUtf8(bool ucs2, const std::vector<uint8_t>& b) {
    return text::clean(ucs2 ? text::ucs2be(b.data(), b.size()) : std::string(b.begin(), b.end()));
}

void fig2(Fig& g, int toggle, int seg, int rfu, int ext) {   // 8.1.13, 8.1.14 and TS 103 176: UTF-8 / UCS-2 labels in segments
    const uint8_t* f = g.f;
    g.add(0, pf("toggle flag=%d", toggle));
    g.add(0, pf("segment index=%d", seg));
    g.add(0, pf("rfu=%d", rfu));
    const int idl = fig2IdLen(f, ext);
    if (g.len <= 1 + idl || ext == 2 || ext == 3 || ext > 6) {
        if (ext > 6 || ext == 2 || ext == 3) g.add(0, "data: " + hexOf(f + 1, (size_t)(g.len - 1)));
        else g.err("FIG2 length error");
        return;
    }
    switch (ext) {
        case 0: g.add(0, pf("Ensemble ID=0x%04X", g.u16(1))); break;
        case 1: g.add(0, pf("Service ID=0x%04X", g.u16(1))); break;
        case 4: {
            const int pd = f[1] >> 7;
            g.add(0, pf(pd ? "Service ID=0x%08X" : "Service ID=0x%04X", pd ? g.u32(2) : g.u16(2)));
            g.add(0, pf("Service Component ID=0x%04X", f[1] & 15));
            break;
        }
        case 5: g.add(0, pf("Service ID=0x%08X", g.u32(1))); break;
        case 6: {
            const int pd = f[1] >> 7;
            g.add(0, pf(pd ? "Service ID=0x%08X" : "Service ID=0x%04X", pd ? g.u32(2) : g.u16(2)));
            g.add(0, pf("Service Component ID=0x%04X", f[1] & 15));
            const int app = f[pd ? 6 : 4] & 0x1F;
            g.add(0, pf("X-PAD App=%02X (%s)", app, xpadApp(app)));
            break;
        }
    }
    const uint8_t* d = f + 1 + idl;
    int n = g.len - 1 - idl;
    bool ucs2 = false;
    int total = 0;
    if (seg == 0) {
        ucs2 = d[0] >> 7;
        total = ((d[0] >> 4) & 7) + 1;
        g.add(0, pf("encoding=%s", ucs2 ? "UCS-2" : "UTF-8"));
        g.add(0, pf("Total number of segments=%d", total));
        if (!rfu) {
            g.add(0, pf("rfa=%d", d[0] & 15));
            g.add(0, pf("character flag=%04x", g.u16((int)(d - f) + 1)));
            if (n <= 3) { g.err("FIG2 label length too short"); return; }
            d += 3; n -= 3;
        } else {
            g.add(0, pf("text control=0x%02x", d[0] & 15));
            if (n <= 1) { g.err("FIG2 label length too short"); return; }
            d += 1; n -= 1;
        }
    }
    g.add(0, "Segment text=\"" + ucsOrUtf8(ucs2, std::vector<uint8_t>(d, d + n)) + "\"");
    // put the label together from the segments of the same label and toggle value
    if (!g.ctx->all) return;
    std::map<int, std::vector<uint8_t>> segs;
    std::map<int, uint64_t> seen;
    bool u = false;
    int cnt = 0;
    for (const DabFigRecord& r : *g.ctx->all) {
        if (r.type != 2 || r.data.size() < 2 + (size_t)idl || (r.data[0] & 7) != ext || (r.data[0] >> 7) != toggle) continue;
        if (!std::equal(f + 1, f + 1 + idl, r.data.begin() + 1)) continue;
        const int s = (r.data[0] >> 4) & 7, rf = (r.data[0] >> 3) & 1;
        const uint8_t* rd = r.data.data() + 1 + idl;
        int rn = (int)r.data.size() - 1 - idl;
        if (s == 0) {
            if (rn < (rf ? 2 : 4)) continue;
            u = rd[0] >> 7;
            cnt = ((rd[0] >> 4) & 7) + 1;
            rd += rf ? 1 : 3; rn -= rf ? 1 : 3;
        }
        if (rn < 0 || (seen.count(s) && seen[s] > r.lastFrame)) continue;
        seen[s] = r.lastFrame;
        segs[s].assign(rd, rd + rn);
    }
    std::string state = "(";
    for (const auto& s : segs) state += pf("%d,", s.first);
    state += pf("count=%d,charset=%s)", cnt, cnt ? (u ? "UCS-2" : "UTF-8") : "undefined");
    g.add(0, "Label segments=\"" + state + "\"");
    if (!cnt) return;
    std::vector<uint8_t> all;
    for (int s = 0; s < cnt; s++) {
        const auto it = segs.find(s);
        if (it == segs.end()) return;
        all.insert(all.end(), it->second.begin(), it->second.end());
    }
    g.add(0, "Label=\"" + ucsOrUtf8(u, all) + "\"");
}

} // namespace

// ------------------------------------------------------------------ public
const char* dabFigName(int type, int ext) {
    if (type == 0) {
        switch (ext) {
            case 0: return "Ensemble information";
            case 1: return "Basic sub-channel organization";
            case 2: return "Basic service and service component definition";
            case 3: return "Service component in packet mode";
            case 4: return "Service component with conditional access";
            case 5: return "Service component language";
            case 6: return "Service linking information";
            case 7: return "Configuration information";
            case 8: return "Service component global definition";
            case 9: return "Country, LTO and international table";
            case 10: return "Date and time";
            case 11: return "Region definition";
            case 13: return "User application information";
            case 14: return "FEC sub-channel organization";
            case 16: return "Programme number";
            case 17: return "Programme type";
            case 18: return "Announcement support";
            case 19: return "Announcement switching";
            case 20: return "Service component information";
            case 21: return "Frequency information";
            case 22: return "Transmitter identification information (TII) database";
            case 24: return "Other ensembles services";
            case 25: return "Other ensembles announcement support";
            case 26: return "Other ensembles announcement switching";
            case 27: return "FM announcement support";
            case 28: return "FM announcement switching";
            case 31: return "FIC re-direction";
            default: return "Reserved";
        }
    }
    if (type == 1 || type == 2) {
        switch (ext) {
            case 0: return "Ensemble label";
            case 1: return "Programme service label";
            case 4: return "Service component label";
            case 5: return "Data service label";
            case 6: return "X-PAD user application label";
            default: return "Reserved";
        }
    }
    if (type == 5) return "Fast information data channel (FIDC)";
    if (type == 6) return "Conditional access";
    return "Reserved";
}

std::string dabFigHeader(const DabFigRecord& r) {
    if (r.type == 0) return pf("0/%d  C/N %d  OE %d  P/D %d", r.ext, r.cn, r.oe, r.pd);
    if (r.type == 1 || r.type == 2) return pf("%d/%d  OE %d", r.type, r.ext, r.oe);
    return pf("%d", r.type);
}

DabFigContext dabFigContext(const DabFicAnalysis& a) {
    DabFigContext c;
    c.all = &a.figs;
    for (const DabFigRecord& r : a.figs) {
        if (r.type != 0 || r.data.size() < 4) continue;
        if (r.ext == 9 && r.data.size() >= 4) c.intlTable = r.data[3];
        else if (r.ext == 22) {   // main identifiers: latitude and longitude, for the offsets of the sub identifiers
            const uint8_t* f = r.data.data();
            const int n = (int)r.data.size();
            for (int i = 1; i < n;) {
                const int ms = f[i] >> 7, main = f[i] & 0x7F;
                i++;
                if (!ms) {
                    if (i + 5 > n) break;
                    const int16_t lac = (int16_t)((f[i] << 8) | f[i + 1]), loc = (int16_t)((f[i + 2] << 8) | f[i + 3]);
                    c.mainPos[(r.oe << 8) | (r.pd << 7) | main] = {(double)(((int32_t)lac << 4) | (f[i + 4] >> 4)) * 90 / 524288,
                                                                    (double)(((int32_t)loc << 4) | (f[i + 4] & 15)) * 180 / 524288};
                    i += 5;
                } else {
                    if (i >= n) break;
                    i += 1 + (f[i] & 7) * 6;
                }
            }
        }
    }
    return c;
}

DabFigDecoded dabFigDecode(const DabFigRecord& r, const DabFigContext& ctx) {
    DabFigDecoded out;
    uint8_t buf[96] = {};
    const int len = (int)std::min<size_t>(r.data.size(), 32);
    std::copy(r.data.begin(), r.data.begin() + len, buf);
    if (len < 1) { out.errors.push_back("empty FIG"); return out; }
    Fig g{buf, len, r.cn, r.oe, r.pd, &ctx, &out};
    // the shortest data field each decoder can read without running past the padding
    auto need = [&](int n) { if (len >= n) return true; g.err(pf("FIG too short (%d bytes, %d expected)", len, n)); return false; };
    bool done = true;
    if (r.type == 0) {
        switch (r.ext) {
            case 0: if (need(5)) fig0_0(g); break;
            case 1: fig0_1(g); break;
            case 2: if (need(3)) fig0_2(g); break;
            case 3: fig0_3(g); break;
            case 5: fig0_5(g); break;
            case 6: fig0_6(g); break;
            case 7: fig0_7(g); break;
            case 8: fig0_8(g); break;
            case 9: fig0_9(g); break;
            case 10: if (need(5)) fig0_10(g); break;
            case 11: fig0_11(g); break;
            case 13: if (need(g.pd ? 6 : 4)) fig0_13(g); break;
            case 14: fig0_14(g); break;
            case 16: fig0_16(g); break;
            case 17: fig0_17(g); break;
            case 18: fig0_18(g); break;
            case 19: fig0_19(g); break;
            case 21: fig0_21(g); break;
            case 22: fig0_22(g); break;
            case 24: fig0_24(g); break;
            case 25: fig0_25(g); break;
            case 26: fig0_26(g); break;
            case 27: fig0_27(g); break;
            case 28: fig0_28(g); break;
            case 31: fig0_31(g); break;
            default: done = false; out.errors.push_back(pf("FIG 0/%d: no decoder", r.ext)); break;
        }
    } else if (r.type == 1) {
        fig1(g, buf[0] >> 4, r.ext);
    } else if (r.type == 2) {
        fig2(g, buf[0] >> 7, (buf[0] >> 4) & 7, (buf[0] >> 3) & 1, r.ext);
    } else {
        done = false;
    }
    if (!done || (out.lines.empty() && r.type != 1 && r.type != 2)) out.lines.push_back({0, "data: " + hexOf(r.data.data(), r.data.size())});
    return out;
}

const char* dabLanguageName(int code) { return language((unsigned)code); }
const char* dabPtyName(int table, int pty) { return ptyName(table, (unsigned)pty); }
const char* dabAnnouncementName(int bit) { return bit >= 0 && bit < 16 ? kAnnounce[bit] : "?"; }
std::string dabUserAppName(int type) { return userApp(type); }

DabFicInfo dabFicInfo(const DabFicAnalysis& a) {
    DabFicInfo c;
    auto sid16 = [](const uint8_t* p) { return (uint32_t)(p[0] << 8) | p[1]; };
    // 0/9 first: the international table names the programme types
    for (const DabFigRecord& r : a.figs) if (r.type == 0 && r.ext == 9 && r.data.size() >= 4) c.intlTable = r.data[3];
    for (const DabFigRecord& r : a.figs) {
        if (r.type != 0) continue;
        uint8_t f[96] = {};
        const int len = (int)std::min<size_t>(r.data.size(), 32);
        std::copy(r.data.begin(), r.data.begin() + len, f);
        const bool pd = r.pd;
        auto u32 = [&](int i) { return ((uint32_t)f[i] << 24) | ((uint32_t)f[i + 1] << 16) | ((uint32_t)f[i + 2] << 8) | f[i + 3]; };
        switch (r.ext) {
            case 0:
                if (len >= 5) { c.country = f[1] >> 4; c.eref = (f[1] & 15) * 256 + f[2]; c.alarm = (f[3] >> 5) & 1; c.cifHigh = f[3] & 0x1F; c.cifLow = f[4]; }
                break;
            case 5:
                for (int i = 1; i < len - 1;) {
                    if (!(f[i] >> 7)) { if (!((f[i] >> 6) & 1)) c.subLanguage[f[i] & 0x3F] = f[i + 1]; i += 2; } else i += 3;
                }
                break;
            case 6:
                for (int i = 1; i < len - 1;) {
                    DabFicInfo::Link l;
                    const int idl = f[i] >> 7;
                    l.active = (f[i] >> 6) & 1; l.hard = (f[i] >> 5) & 1; l.intl = (f[i] >> 4) & 1; l.lsn = ((f[i] & 15) << 8) | f[i + 1];
                    i += 2;
                    if (idl && i < len) {
                        const int n = f[i] & 15;
                        const int w = pd ? 4 : l.intl ? 3 : 2;
                        for (int j = 0; j < n && i + 1 + w * (j + 1) <= len; j++) {
                            const int o = i + 1 + w * j;
                            l.ids.push_back(w == 2 ? sid16(f + o) : w == 4 ? u32(o) : ((uint32_t)f[o] << 16) | sid16(f + o + 1));
                        }
                        i += n * w + 1;
                    }
                    c.links.push_back(l);
                }
                break;
            case 7: if (len == 3) { c.services = (f[1] << 8 | f[2]) >> 10; c.reconfCount = (f[1] << 8 | f[2]) & 0x3FF; } break;
            case 9:
                if (len >= 4) { c.hasLto = true; c.ltoHalfHours = f[1] & 0x3F; if (c.ltoHalfHours & 0x20) c.ltoHalfHours -= 64; c.hasEcc = true; c.ecc = f[2]; }
                break;
            case 10:
                if (len >= 5) {
                    c.hasTime = true;
                    c.mjd = ((f[1] & 0x7F) << 10) | (f[2] << 2) | (f[3] >> 6);
                    c.hour = ((f[3] & 7) << 2) | (f[4] >> 6); c.minute = f[4] & 0x3F;
                    c.second = (f[3] & 8) && len >= 6 ? f[5] >> 2 : -1;
                }
                break;
            case 13: {
                int k = 1;
                if (len < (pd ? 6 : 4)) break;
                uint32_t sid = pd ? u32(k) : sid16(f + k);
                k += pd ? 4 : 2;
                const int no = f[k] & 15;
                k++;
                for (int n = 0; n < no && k + 2 <= len; n++) {
                    c.userApps[sid].push_back((f[k] << 3) | (f[k + 1] >> 5));
                    k += 2 + (f[k + 1] & 0x1F);
                }
                break;
            }
            case 14: for (int i = 1; i < len; i++) c.subFec[f[i] >> 2] = f[i] & 3; break;
            case 17:
                for (int i = 1; i < len - 3;) {
                    const uint32_t sid = sid16(f + i);
                    const int l = (f[i + 2] >> 5) & 1, cc = (f[i + 2] >> 4) & 1;
                    i += 3;
                    if (l) { c.language[sid] = f[i]; i++; }
                    if (i < len) { c.pty[sid] = f[i] & 0x1F; i++; }
                    if (cc) i++;
                }
                break;
            case 18: for (int i = 1; i < len - 4; i += 5 + (f[i + 4] & 0x1F)) c.announce[sid16(f + i)] = sid16(f + i + 2); break;
            case 21:
                for (int i = 1; i < len;) {
                    const int flen = f[i + 1] & 0x1F;
                    i += 2;
                    const int end = std::min(len, i + flen);
                    while (i + 3 <= end) {
                        DabFicInfo::Freq q;
                        q.id = sid16(f + i); q.rm = f[i + 2] >> 4;
                        const int nfl = f[i + 2] & 7;
                        i += 3;
                        const int fe = std::min(end, i + nfl);
                        if (q.rm == 0 || q.rm == 1) for (; i + 3 <= fe; i += 3) { const unsigned v = 16u * (((unsigned)(f[i] & 7) << 16) | (f[i + 1] << 8) | f[i + 2]); if (v) q.kHz.push_back(v); }
                        else if (q.rm == 8 || q.rm == 9) for (; i < fe; i++) { if (f[i]) q.kHz.push_back(87500.0 + f[i] * 100.0); }
                        else if (q.rm == 10) for (; i < fe; i++) { if (f[i]) q.kHz.push_back(f[i] < 16 ? 144.0 + f[i] * 9 : 387.0 + f[i] * 9); }
                        else if (q.rm == 12) for (; i + 2 <= fe; i += 2) { const unsigned v = 5 * sid16(f + i); if (v) q.kHz.push_back(v); }
                        i = fe;
                        c.freqs.push_back(q);
                    }
                    i = end;
                }
                break;
            case 22:
                for (int i = 1; i < len;) {
                    const int ms = f[i] >> 7, main = f[i] & 0x7F;
                    i++;
                    if (!ms) {
                        if (i + 5 > len) break;
                        const int16_t lac = (int16_t)(f[i] << 8 | f[i + 1]), loc = (int16_t)(f[i + 2] << 8 | f[i + 3]);
                        c.transmitters.push_back({main, (double)(((int32_t)lac << 4) | (f[i + 4] >> 4)) * 90 / 524288, (double)(((int32_t)loc << 4) | (f[i + 4] & 15)) * 180 / 524288});
                        i += 5;
                    } else { if (i >= len) break; i += 1 + (f[i] & 7) * 6; }
                }
                break;
            case 24:
                for (int i = 1; i < len - (pd + 1) * 2;) {
                    const uint32_t sid = pd ? u32(i) : sid16(f + i);
                    i += pd ? 4 : 2;
                    const int n = f[i] & 15;
                    i++;
                    auto& v = c.otherEnsembles[sid];
                    for (int j = 0; j < n && i + 2 <= len; j++, i += 2) v.push_back((uint16_t)sid16(f + i));
                }
                break;
            default: break;
        }
    }
    return c;
}

std::string dabFicText(const DabFicAnalysis& a) {
    std::string s = pf("FIC: %llu frames, %llu FIBs ok, %llu bad\n\n", (unsigned long long)a.frames, (unsigned long long)a.fibOk, (unsigned long long)a.fibBad);
    const DabFigContext ctx = dabFigContext(a);
    int lt = -1, le = -1;
    for (const DabFigRecord& r : a.figs) {
        if (r.type != lt || r.ext != le) {
            s += pf("FIG %d/%d  %s\n", r.type, r.ext, dabFigName(r.type, r.ext));
            lt = r.type; le = r.ext;
        }
        s += pf("  %s  x%llu  frames %llu..%llu  FIB %d\n", dabFigHeader(r).c_str(), (unsigned long long)r.count, (unsigned long long)r.firstFrame,
                (unsigned long long)r.lastFrame, r.lastFib);
        const DabFigDecoded d = dabFigDecode(r, ctx);
        for (const DabFigLine& l : d.lines) s += std::string((size_t)(4 + 2 * l.level), ' ') + l.text + "\n";
        for (const std::string& e : d.errors) s += "    ERROR: " + e + "\n";
    }
    return s;
}

} // namespace dect2
