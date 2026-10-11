// VDL Mode 2 screens: the message log with a filter and a detail pane, a map of the positions the messages carry, the aircraft and
// ground stations, and the channel list with its activity.
#include "adsb_map.h"
#include "app.h"
#include "dect2/adsb_tel.h"
#include "dect2/vdl2_tel.h"
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <map>

// What ADS-B heard in this session, by ICAO address: VDL2 shows it next to its own aircraft (adsb_ui.cpp feeds it while ADS-B runs)
struct AdsbSeen { std::string callsign; bool hasPos = false; double lat = 0, lon = 0; bool hasAlt = false; int altFt = 0; int64_t wall = 0; };
static std::map<uint32_t, AdsbSeen>& adsbSeen() { static std::map<uint32_t, AdsbSeen> m; return m; }

void vdl2NoteAdsb(const dect2::AdsbTelemetry& t) {
    auto& m = adsbSeen();
    const int64_t now = (int64_t)time(nullptr);
    for (const auto& c : t.aircraft) {
        if (c.icao > 0xFFFFFF) continue;
        AdsbSeen& s = m[c.icao];
        if (!c.callsign.empty()) s.callsign = c.callsign;
        if (c.hasPos) { s.hasPos = true; s.lat = c.lat; s.lon = c.lon; }
        if (c.hasAlt) { s.hasAlt = true; s.altFt = c.altFt; }
        s.wall = now;
    }
    if (m.size() > 5000) m.clear();
}

namespace {

struct State {
    bool loaded = false;
    int viewMode = 0;                    // 0 messages, 1 map, 2 aircraft, 3 ground stations, 4 channels
    char fAir[24] = "";                  // filter: address, registration or flight contains this
    char fText[32] = "";                 // filter: the summary or detail contains this
    int fKind = 0;                       // 0 all, 1 ACARS, 2 ATN (X.25 / CLNP), 3 link (XID, S, U)
    double fChan = 0;                    // channel filter: the channel's frequency in Hz, 0 = any
    int fFrame = 0;                      // frame type: 0 any, 1 I, 2 S (RR, RNR, REJ), 3 UI, 4 XID (all), 5 GSIF, 6 link establishment, 7 handoff, 8 other U
    uint64_t selected = 0;
    char chans[160] = "";                // channel list in MHz, empty = the European six
    char pushedChans[160] = "\x01";
    bool wasRunning = false;
    adsbmap::View map;
    bool mapFollow = true;
    uint32_t selAir = 0;
};
State S;

bool live(const App& a) { return a.engine.running() && a.rx.standard == 28; }

void loadState() {
    if (S.loaded) return;
    S.loaded = true;
    plat::Prefs& d = plat::prefs();
    snprintf(S.chans, sizeof S.chans, "%s", d.getS("vdl2Chans", "").c_str());
    S.map.zoom = (int)d.getI("vdl2Zoom", 7);
    S.map.lat = 51.47; S.map.lon = -0.45;
    if (const char* e = getenv("DECT2_VDL2_VIEW")) S.viewMode = std::max(0, std::min(4, atoi(e)));   // dev: open on a view (screenshots)
}

std::string lower(std::string s) { for (auto& c : s) c = (char)std::tolower((unsigned char)c); return s; }
bool contains(const std::string& hay, const char* needle) { return !*needle || lower(hay).find(lower(needle)) != std::string::npos; }
std::string hex6(uint32_t a) { char b[8]; snprintf(b, sizeof b, "%06X", a & 0xFFFFFF); return b; }

std::string clock(int64_t t) {
    if (t <= 0) return "-";
    const time_t tt = (time_t)t;
    struct tm g;
#ifdef _WIN32
    gmtime_s(&g, &tt);
#else
    gmtime_r(&tt, &g);
#endif
    char b[16];
    snprintf(b, sizeof b, "%02d:%02d:%02d", g.tm_hour, g.tm_min, g.tm_sec);
    return b;
}

std::string ago(double now, double then) {
    if (then <= 0) return "-";
    const double s = std::max(0.0, now - then);
    char b[24];
    if (s < 90) snprintf(b, sizeof b, "%.0f s", s); else if (s < 5400) snprintf(b, sizeof b, "%.0f min", s / 60); else snprintf(b, sizeof b, "%.1f h", s / 3600);
    return b;
}

const char* typeShort(int t) { return t == 1 ? "air" : t == 4 || t == 5 ? "gnd" : t == 7 ? "all" : "?"; }

const dect2::Vdl2Aircraft* findAir(const dect2::Vdl2Telemetry& t, uint32_t icao) {
    for (const auto& c : t.aircraft) if (c.icao == icao) return &c;
    return nullptr;
}

// the name of an address: aircraft registration / flight, or the ground station's airports
std::string who(const dect2::Vdl2Telemetry& t, uint32_t addr, int type) {
    std::string s = hex6(addr);
    if (type == 1) {
        if (const auto* c = findAir(t, addr)) { if (!c->reg.empty()) s += " " + c->reg; if (!c->flight.empty()) s += " " + c->flight; }
        auto it = adsbSeen().find(addr);
        if (it != adsbSeen().end() && !it->second.callsign.empty()) s += " (" + it->second.callsign + ")";
    }
    return s;
}

void tick(App& a) {
    loadState();
    if (!a.engine.running()) { S.wasRunning = false; S.pushedChans[0] = '\x01'; S.pushedChans[1] = 0; return; }
    if (std::strcmp(S.pushedChans, S.chans) != 0) {
        std::vector<double> hz;
        const char* p = S.chans;
        while (*p) {
            char* e = nullptr;
            const double mhz = std::strtod(p, &e);
            if (e == p) { p++; continue; }
            if (mhz >= 118 && mhz <= 140) hz.push_back(mhz * 1e6);
            p = e;
        }
        a.engine.vdl2().setChannels(hz);
        std::snprintf(S.pushedChans, sizeof S.pushedChans, "%s", S.chans);
    }
    S.wasRunning = true;
}

// ---------------------------------------------------------------- messages

bool passes(const dect2::Vdl2Telemetry& t, const dect2::Vdl2Message& m) {
    if (S.fKind == 1 && !m.acars) return false;
    if (S.fKind == 2 && !(m.proto == "X.25" || m.proto == "CLNP" || m.proto == "COTP" || m.proto == "ES-IS" || m.proto == "IDRP")) return false;
    if (S.fKind == 3 && (m.acars || m.frameKind == 'I')) return false;
    if (S.fChan > 0 && std::fabs(m.freqHz - S.fChan) > 1000) return false;
    if (S.fFrame) {
        const std::string& f = m.frameName;
        const bool xid = f == "GSIF" || f.compare(0, 3, "XID") == 0;
        bool ok = false;
        switch (S.fFrame) {
        case 1: ok = m.frameKind == 'I'; break;
        case 2: ok = m.frameKind == 'S'; break;
        case 3: ok = f == "UI"; break;
        case 4: ok = xid; break;
        case 5: ok = f == "GSIF"; break;
        case 6: ok = f == "XID_CMD_LE" || f == "XID_RSP_LE"; break;
        case 7: ok = f == "XID_CMD_HO" || f == "XID_RSP_HO"; break;
        case 8: ok = m.frameKind == 'U' && f != "UI" && !xid; break;
        }
        if (!ok) return false;
    }
    if (S.fAir[0]) {
        const std::string s = who(t, m.src, m.srcType) + " " + who(t, m.dst, m.dstType) + " " + m.reg + " " + m.flight;
        if (!contains(s, S.fAir)) return false;
    }
    if (S.fText[0] && !contains(m.summary, S.fText) && !contains(m.detail, S.fText)) return false;
    return true;
}

void filterRow(const dect2::Vdl2Telemetry& t) {
    ImGui::TextDisabled("Aircraft / flight"); ImGui::SameLine(0, 5 * gUi);
    ImGui::SetNextItemWidth(150 * gUi);
    ImGui::InputTextWithHint("##vdfa", "ICAO, reg, flight", S.fAir, sizeof S.fAir);
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("Show only frames to or from this aircraft: its 24-bit address (4CA2B1), registration or flight.");
    flowNext(12 * gUi);
    ImGui::TextDisabled("Text"); ImGui::SameLine(0, 5 * gUi);
    ImGui::SetNextItemWidth(100 * gUi);
    ImGui::InputText("##vdft", S.fText, sizeof S.fText);
    flowNext(12 * gUi);
    ImGui::SetNextItemWidth(110 * gUi);
    static const char* kinds[] = {"all frames", "ACARS", "ATN (X.25)", "link (XID, S, U)"};
    ImGui::Combo("##vdfk", &S.fKind, kinds, 4);
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("What the frame carries.");
    flowNext(8 * gUi);
    ImGui::SetNextItemWidth(150 * gUi);
    static const char* frames[] = {"any frame type", "I (information)", "S (RR, RNR, REJ)", "UI", "XID (all)", "GSIF", "XID link establishment", "XID handoff", "other U (DISC, UA, DM, TEST)"};
    ImGui::Combo("##vdff", &S.fFrame, frames, 9);
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("Show only this kind of AVLC frame.");
    flowNext(8 * gUi);
    ImGui::SetNextItemWidth(120 * gUi);
    {
        char cur[24];
        if (S.fChan > 0) snprintf(cur, sizeof cur, "%.3f MHz", S.fChan / 1e6); else snprintf(cur, sizeof cur, "any channel");
        if (ImGui::BeginCombo("##vdfc", cur)) {
            if (ImGui::Selectable("any channel", S.fChan <= 0)) S.fChan = 0;
            for (const auto& c : t.channels) {
                char l[24]; snprintf(l, sizeof l, "%.3f MHz", c.freqHz / 1e6);
                if (ImGui::Selectable(l, std::fabs(S.fChan - c.freqHz) < 1000)) S.fChan = c.freqHz;
            }
            ImGui::EndCombo();
        }
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("Show only the frames received on this channel.");
    }
    flowEnd();
}

void detailPane(const App& a, const dect2::Vdl2Telemetry& t, const dect2::Vdl2Message* m) {
    if (!m) { ImGui::PushTextWrapPos(0); ImGui::TextDisabled("Click a message to see every layer of it."); ImGui::PopTextWrapPos(); return; }
    auto kv = [&](const char* k, const char* fmt, auto... v) { ImGui::TextDisabled("%s", k); kvColumn(100 * gUi); ImGui::PushFont(a.mono, 0); ImGui::PushTextWrapPos(0); ImGui::Text(fmt, v...); ImGui::PopTextWrapPos(); ImGui::PopFont(); };
    kv("time", "%s UTC  %.3f MHz  %.0f dBFS  SNR %.0f dB  %+.0f Hz", clock(m->wallTime).c_str(), m->freqHz / 1e6, m->levelDb, m->snrDb, m->cfoHz);
    kv("from", "%s  (%s)", who(t, m->src, m->srcType).c_str(), m->srcType == 1 ? "aircraft" : m->srcType == 4 || m->srcType == 5 ? "ground station" : m->srcType == 7 ? "all stations" : "reserved");
    kv("to", "%s  (%s)", who(t, m->dst, m->dstType).c_str(), m->dstType == 1 ? "aircraft" : m->dstType == 4 || m->dstType == 5 ? "ground station" : m->dstType == 7 ? "all stations" : "reserved");
    if (m->rsFixed) kv("repaired", "%d octet%s by the Reed-Solomon code", m->rsFixed, m->rsFixed == 1 ? "" : "s");
    const uint32_t air = m->srcType == 1 ? m->src : m->dstType == 1 ? m->dst : 0;
    if (air) {
        auto it = adsbSeen().find(air);
        if (it != adsbSeen().end()) {
            const AdsbSeen& s = it->second;
            std::string pos = s.hasPos ? (std::to_string(s.lat).substr(0, 7) + " " + std::to_string(s.lon).substr(0, 7)) : "no position";
            kv("ADS-B", "%s, %s%s, heard %s ago", s.callsign.empty() ? "no callsign" : s.callsign.c_str(), pos.c_str(), s.hasAlt ? (", " + std::to_string(s.altFt) + " ft").c_str() : "",
               ago((double)time(nullptr), (double)s.wall).c_str());
        } else kv("ADS-B", "%s", "not heard in this session (run the ADS-B mode to link the address)");
    }
    ImGui::Spacing();
    ImGui::TextDisabled("Decoded");
    ImGui::BeginChild("##vddet", ImVec2(0, 0), ImGuiChildFlags_Borders);
    ImGui::PushFont(a.mono, 0);
    ImGui::PushTextWrapPos(0);
    ImGui::TextUnformatted(m->detail.c_str());
    if (m->acars && !m->acarsText.empty()) { ImGui::Spacing(); ImGui::TextDisabled("ACARS text"); ImGui::TextUnformatted(m->acarsText.c_str()); }
    ImGui::PopTextWrapPos();
    ImGui::PopFont();
    ImGui::EndChild();
}

void messageView(App& a) {
    const dect2::Vdl2Telemetry& t = a.rx.vdl2;
    const bool on = live(a);
    filterRow(t);
    const float H = ImGui::GetContentRegionAvail().y;
    const float listH = std::max(120.f * gUi, H * 0.55f);
    const dect2::Vdl2Message* sel = nullptr;
    if (ImGui::BeginTable("##vdmsg", 6, ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY | ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_BordersInnerV, ImVec2(0, listH))) {
        ImGui::TableSetupScrollFreeze(0, 1);
        ImGui::TableSetupColumn("Time UTC"); ImGui::TableSetupColumn("MHz"); ImGui::TableSetupColumn("From"); ImGui::TableSetupColumn("To");
        ImGui::TableSetupColumn("Frame"); ImGui::TableSetupColumn("Content", ImGuiTableColumnFlags_WidthStretch);
        ImGui::TableHeadersRow();
        int shown = 0;
        if (on) {
            for (const auto& m : t.messages) {
                if (m.serial == S.selected) sel = &m;
                if (!passes(t, m)) continue;
                shown++;
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                char id[24]; snprintf(id, sizeof id, "##v%llu", (unsigned long long)m.serial);
                if (ImGui::Selectable((clock(m.wallTime) + id).c_str(), S.selected == m.serial, ImGuiSelectableFlags_SpanAllColumns)) { S.selected = m.serial; sel = &m; }
                ImGui::TableNextColumn(); ImGui::Text("%.3f", m.freqHz / 1e6);
                ImGui::TableNextColumn(); ImGui::Text("%s %s", typeShort(m.srcType), hex6(m.src).c_str());
                ImGui::TableNextColumn(); ImGui::Text("%s %s", typeShort(m.dstType), hex6(m.dst).c_str());
                ImGui::TableNextColumn(); ImGui::PushStyleColor(ImGuiCol_Text, pal::accent()); ImGui::TextUnformatted(m.frameName.c_str()); ImGui::PopStyleColor();
                ImGui::TableNextColumn(); ImGui::TextUnformatted(ellipsize(m.summary, ImGui::GetContentRegionAvail().x).c_str());
            }
        }
        ImGui::EndTable();
        if (!on) { ImGui::PushTextWrapPos(0); ImGui::TextDisabled("%s", a.engine.running() ? "starting" : "start the receiver to see messages"); ImGui::PopTextWrapPos(); }
        else if (!shown) { ImGui::PushTextWrapPos(0); ImGui::TextDisabled(t.messages.empty() ? "no frame yet" : "no frame matches the filter"); ImGui::PopTextWrapPos(); }
    }
    ImGui::Spacing();
    ImGui::BeginChild("##vdpane", ImVec2(0, 0));
    detailPane(a, t, on ? sel : nullptr);
    ImGui::EndChild();
}

// ---------------------------------------------------------------- map

ImU32 planeColour(double now, double posTime) { return now - posTime > 1800 ? IM_COL32(150, 154, 158, 220) : IM_COL32(205, 214, 120, 255); }

void mapCanvas(App& a, ImVec2 size) {
    const dect2::Vdl2Telemetry& t = a.rx.vdl2;
    const bool on = live(a);
    const ImVec2 p0 = ImGui::GetCursorScreenPos();
    if (S.mapFollow && on) {
        double la0 = 90, la1 = -90, lo0 = 180, lo1 = -180; int n = 0;
        auto add = [&](double la, double lo) { la0 = std::min(la0, la); la1 = std::max(la1, la); lo0 = std::min(lo0, lo); lo1 = std::max(lo1, lo); n++; };
        for (const auto& c : t.aircraft) if (c.hasPos) add(c.lat, c.lon);
        for (const auto& g : t.stations) if (g.hasPos) add(g.lat, g.lon);
        if (n) {
            S.map.lat = (la0 + la1) / 2; S.map.lon = (lo0 + lo1) / 2;
            const double span = std::max(0.5, std::max(lo1 - lo0, (la1 - la0) * 1.4));
            S.map.zoom = std::max(2, std::min(11, (int)std::floor(std::log2(0.7 * size.x * 360.0 / (256.0 * span)))));
        }
    }
    const double lat0 = S.map.lat, lon0 = S.map.lon;
    adsbmap::draw(S.map, size);
    if (S.map.lat != lat0 || S.map.lon != lon0) S.mapFollow = false;
    ImDrawList* dl = ImGui::GetWindowDrawList();
    dl->PushClipRect(p0, ImVec2(p0.x + size.x, p0.y + size.y), true);
    ImVec2 click; const bool clicked = adsbmap::clickedAt(click);
    float best = 18 * gUi; uint32_t hit = 0;
    int placed = 0;
    if (on) {
        for (const auto& g : t.stations) {   // ground stations: squares
            if (!g.hasPos) continue;
            const ImVec2 q = adsbmap::project(g.lat, g.lon);
            dl->AddRectFilled(ImVec2(q.x - 5 * gUi, q.y - 5 * gUi), ImVec2(q.x + 5 * gUi, q.y + 5 * gUi), IM_COL32(120, 170, 230, 255));
            dl->AddRect(ImVec2(q.x - 5 * gUi, q.y - 5 * gUi), ImVec2(q.x + 5 * gUi, q.y + 5 * gUi), IM_COL32(0, 0, 0, 160));
            const std::string s = "GS " + hex6(g.addr) + (g.airports.empty() ? "" : " " + g.airports);
            dl->AddText(ImVec2(q.x + 8 * gUi, q.y - 7 * gUi), IM_COL32(200, 220, 245, 255), s.c_str());
        }
        for (const auto& c : t.aircraft) {
            if (!c.hasPos) continue;
            placed++;
            const ImU32 col = planeColour(t.nowSec, c.posTime);
            for (size_t i = 1; i < c.track.size(); i++) dl->AddLine(adsbmap::project(c.track[i - 1].lat, c.track[i - 1].lon), adsbmap::project(c.track[i].lat, c.track[i].lon), (col & 0x00FFFFFF) | 0x90000000, 1.5f);
            const ImVec2 q = adsbmap::project(c.lat, c.lon);
            const bool sel = c.icao == S.selAir;
            dl->AddCircleFilled(q, (sel ? 6.f : 5.f) * gUi, col);
            dl->AddCircle(q, (sel ? 6.f : 5.f) * gUi, sel ? IM_COL32(255, 255, 255, 255) : IM_COL32(0, 0, 0, 160), 16, sel ? 1.6f : 1.f);
            std::string s = c.reg.empty() ? hex6(c.icao) : c.reg;
            if (!c.flight.empty()) s += " " + c.flight;
            if (c.hasAlt) { char b[16]; snprintf(b, sizeof b, " FL%03d", (c.altFt + 50) / 100); s += b; }
            const ImVec2 ts = ImGui::CalcTextSize(s.c_str());
            const ImVec2 lp(q.x + 9 * gUi, q.y - ts.y * 0.5f - 2 * gUi);
            dl->AddRectFilled(lp, ImVec2(lp.x + ts.x + 8 * gUi, lp.y + ts.y + 4 * gUi), IM_COL32(10, 12, 14, 210), 2.f);
            dl->AddText(ImVec2(lp.x + 4 * gUi, lp.y + 2 * gUi), IM_COL32(235, 238, 240, 255), s.c_str());
            const float d = std::hypot(q.x - click.x, q.y - click.y);
            if (clicked && d < best) { best = d; hit = c.icao; }
        }
    }
    if (clicked) S.selAir = hit;
    dl->PopClipRect();
    ImGui::SetCursorScreenPos(ImVec2(p0.x + 8 * gUi, p0.y + 8 * gUi));
    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.06f, 0.07f, 0.08f, 0.85f));
    if (ImGui::Button("+", ImVec2(26 * gUi, 0))) { S.map.zoom = std::min(adsbmap::kMaxZoom, S.map.zoom + 1); S.mapFollow = false; }
    ImGui::SameLine(0, 2 * gUi);
    if (ImGui::Button("-", ImVec2(26 * gUi, 0))) { S.map.zoom = std::max(adsbmap::kMinZoom, S.map.zoom - 1); S.mapFollow = false; }
    ImGui::SameLine(0, 6 * gUi);
    if (ImGui::Button("fit")) S.mapFollow = true;
    ImGui::PopStyleColor();
    ImGui::SameLine(0, 6 * gUi);
    adsbmap::tileControls("never the position itself");
    plat::prefs().setI("vdl2Zoom", S.map.zoom);
    ImGui::SetCursorScreenPos(ImVec2(p0.x + 8 * gUi, p0.y + size.y - ImGui::GetTextLineHeight() - 6 * gUi));
    if (!on) adsbmap::legend(size.x, "start the receiver");
    else if (!placed) adsbmap::legend(size.x, "no position report yet");
    else adsbmap::legend(size.x, "%d of %zu aircraft placed", placed, t.aircraft.size());
    ImGui::SetCursorScreenPos(ImVec2(p0.x, p0.y + size.y));
}

void mapSide(App& a) {
    const dect2::Vdl2Telemetry& t = a.rx.vdl2;
    const bool on = live(a);
    const dect2::Vdl2Aircraft* c = on ? findAir(t, S.selAir) : nullptr;
    if (!c) {
        ImGui::PushTextWrapPos(0);
        ImGui::TextDisabled("Dots: aircraft positions from ADS-C, ACARS position reports and XID locations. Squares: ground stations (their GSIF).");
        ImGui::PopTextWrapPos();
        if (!on) return;
        int n = 0;
        for (const auto& x : t.aircraft) {
            if (!x.hasPos) continue;
            char id[16]; snprintf(id, sizeof id, "##vp%d", ++n);
            const std::string row = (x.reg.empty() ? hex6(x.icao) : x.reg) + "  " + ago(t.nowSec, x.posTime) + " ago" + id;
            if (ImGui::Selectable(row.c_str(), false)) S.selAir = x.icao;
        }
        return;
    }
    auto kv = [&](const char* k, const char* fmt, auto... v) { ImGui::TextDisabled("%s", k); kvColumn(84 * gUi); ImGui::PushFont(a.mono, 0); ImGui::PushTextWrapPos(0); ImGui::Text(fmt, v...); ImGui::PopTextWrapPos(); ImGui::PopFont(); };
    ImGui::PushStyleColor(ImGuiCol_Text, pal::heading());
    ImGui::Text("%s %s", hex6(c->icao).c_str(), c->reg.c_str());
    ImGui::PopStyleColor();
    ImGui::SameLine();
    if (ImGui::SmallButton("clear")) S.selAir = 0;
    if (!c->flight.empty()) kv("flight", "%s", c->flight.c_str());
    if (c->hasPos) kv("position", "%.4f %.4f", c->lat, c->lon);
    if (c->hasAlt) kv("altitude", "%d ft", c->altFt);
    if (!c->posKind.empty()) kv("source", "%s, %s ago", c->posKind.c_str(), ago(t.nowSec, c->posTime).c_str());
    if (c->groundStation) kv("ground st.", "%s", hex6(c->groundStation).c_str());
    kv("frames", "%u, last %s ago on %.3f MHz", c->messages, ago(t.nowSec, c->lastHeardSec).c_str(), c->freqHz / 1e6);
    auto it = adsbSeen().find(c->icao);
    if (it != adsbSeen().end()) kv("ADS-B", "%s%s", it->second.callsign.c_str(), it->second.hasPos ? " (position known)" : "");
}

void mapView(App& a) {
    const ImVec2 av = ImGui::GetContentRegionAvail();
    const float sideW = std::max(220.f * gUi, std::min(340.f * gUi, av.x * 0.3f));
    const ImVec2 size(std::max(200.f, av.x - sideW - 8 * gUi), std::max(160.f, av.y));
    const ImVec2 p0 = ImGui::GetCursorScreenPos();
    mapCanvas(a, size);
    ImGui::SetCursorScreenPos(ImVec2(p0.x + size.x + 8 * gUi, p0.y));
    ImGui::BeginChild("##vdmapside", ImVec2(sideW, size.y));
    mapSide(a);
    ImGui::EndChild();
}

// ---------------------------------------------------------------- tables

void aircraftView(const App& a) {
    const dect2::Vdl2Telemetry& t = a.rx.vdl2;
    if (!ImGui::BeginTable("##vdair", 9, ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY | ImGuiTableFlags_ScrollX | ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_BordersInnerV, ImVec2(0, ImGui::GetContentRegionAvail().y))) return;
    ImGui::TableSetupScrollFreeze(0, 1);
    ImGui::TableSetupColumn("ICAO"); ImGui::TableSetupColumn("Reg"); ImGui::TableSetupColumn("Flight"); ImGui::TableSetupColumn("ADS-B");
    ImGui::TableSetupColumn("Frames"); ImGui::TableSetupColumn("MHz"); ImGui::TableSetupColumn("Last heard"); ImGui::TableSetupColumn("Ground st."); ImGui::TableSetupColumn("Position");
    ImGui::TableHeadersRow();
    if (live(a))
        for (const auto& c : t.aircraft) {
            ImGui::TableNextRow();
            ImGui::TableNextColumn(); ImGui::TextUnformatted(hex6(c.icao).c_str());
            ImGui::TableNextColumn(); ImGui::TextUnformatted(c.reg.c_str());
            ImGui::TableNextColumn(); ImGui::TextUnformatted(c.flight.c_str());
            ImGui::TableNextColumn(); { auto it = adsbSeen().find(c.icao); ImGui::TextUnformatted(it != adsbSeen().end() ? (it->second.callsign.empty() ? "seen" : it->second.callsign.c_str()) : "-"); }
            ImGui::TableNextColumn(); ImGui::Text("%u", c.messages);
            ImGui::TableNextColumn(); ImGui::Text("%.3f", c.freqHz / 1e6);
            ImGui::TableNextColumn(); ImGui::TextUnformatted(ago(t.nowSec, c.lastHeardSec).c_str());
            ImGui::TableNextColumn(); ImGui::TextUnformatted(c.groundStation ? hex6(c.groundStation).c_str() : "-");
            ImGui::TableNextColumn(); if (c.hasPos) ImGui::Text("%.3f %.3f%s", c.lat, c.lon, c.hasAlt ? (" " + std::to_string(c.altFt) + " ft").c_str() : ""); else ImGui::TextUnformatted("-");
        }
    ImGui::EndTable();
}

void stationView(const App& a) {
    const dect2::Vdl2Telemetry& t = a.rx.vdl2;
    if (!ImGui::BeginTable("##vdgs", 7, ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY | ImGuiTableFlags_ScrollX | ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_BordersInnerV, ImVec2(0, ImGui::GetContentRegionAvail().y))) return;
    ImGui::TableSetupScrollFreeze(0, 1);
    ImGui::TableSetupColumn("Address"); ImGui::TableSetupColumn("Kind"); ImGui::TableSetupColumn("Frames"); ImGui::TableSetupColumn("MHz");
    ImGui::TableSetupColumn("Last heard"); ImGui::TableSetupColumn("Airports"); ImGui::TableSetupColumn("Location");
    ImGui::TableHeadersRow();
    if (live(a))
        for (const auto& g : t.stations) {
            ImGui::TableNextRow();
            ImGui::TableNextColumn(); ImGui::TextUnformatted(hex6(g.addr).c_str());
            ImGui::TableNextColumn(); ImGui::TextUnformatted(g.type == 5 ? "DEL" : "ADM");
            ImGui::TableNextColumn(); ImGui::Text("%u", g.messages);
            ImGui::TableNextColumn(); if (g.freqHz > 0) ImGui::Text("%.3f", g.freqHz / 1e6); else ImGui::TextUnformatted("-");
            ImGui::TableNextColumn(); ImGui::TextUnformatted(ago(t.nowSec, g.lastHeardSec).c_str());
            ImGui::TableNextColumn(); ImGui::TextUnformatted(g.airports.empty() ? "-" : g.airports.c_str());
            ImGui::TableNextColumn(); if (g.hasPos) ImGui::Text("%.1f %.1f", g.lat, g.lon); else ImGui::TextUnformatted("-");
        }
    ImGui::EndTable();
}

// one bar per channel: level against the noise floor, counts, activity lamp
void channelView(const App& a) {
    const dect2::Vdl2Telemetry& t = a.rx.vdl2;
    const bool on = live(a);
    if (!on || t.channels.empty()) { ImGui::PushTextWrapPos(0); ImGui::TextDisabled("%s", on ? "no channel" : a.engine.running() ? "starting" : "start the receiver to see the channels"); ImGui::PopTextWrapPos(); return; }
    ImDrawList* dl = ImGui::GetWindowDrawList();
    const float W = ImGui::GetContentRegionAvail().x;
    const float rowH = ImGui::GetTextLineHeightWithSpacing() * 1.5f;
    const float labW = 110 * gUi, txtW = std::min(330.f * gUi, W * 0.5f), barW = std::max(60.f, W - labW - txtW - 12 * gUi);
    ImGui::BeginChild("##vdch", ImVec2(0, 0));
    for (const auto& c : t.channels) {
        const ImVec2 p = ImGui::GetCursorScreenPos();
        ImGui::Dummy(ImVec2(W, rowH));
        const float y0 = p.y + 2, y1 = p.y + rowH - 2, ty = p.y + (rowH - ImGui::GetTextLineHeight()) * 0.5f;
        char b[96];
        snprintf(b, sizeof b, "%.3f%s", c.freqHz / 1e6, std::fabs(c.freqHz - 136.975e6) < 1 ? " CSC" : "");
        dl->AddCircleFilled(ImVec2(p.x + 5 * gUi, p.y + rowH * 0.5f), 4 * gUi, !c.inBand ? IM_COL32(70, 70, 70, 255) : c.active ? IM_COL32(110, 210, 120, 255) : IM_COL32(120, 124, 128, 255));
        dl->AddText(ImVec2(p.x + 14 * gUi, ty), c.inBand ? IM_COL32(220, 222, 225, 255) : IM_COL32(110, 114, 118, 255), b);
        const float bx = p.x + labW;
        dl->AddRectFilled(ImVec2(bx, y0), ImVec2(bx + barW, y1), IM_COL32(8, 9, 10, 255));
        auto xOf = [&](float db) { return bx + barW * std::max(0.f, std::min(1.f, (db + 110.f) / 90.f)); };
        if (c.inBand && c.levelDb > -119) dl->AddRectFilled(ImVec2(bx, y0), ImVec2(xOf(c.levelDb), y1), c.active ? IM_COL32(110, 190, 110, 230) : IM_COL32(110, 120, 110, 110));
        if (c.inBand && c.noiseDb > -119) dl->AddLine(ImVec2(xOf(c.noiseDb), y0), ImVec2(xOf(c.noiseDb), y1), IM_COL32(230, 160, 70, 255), 2.f);
        if (!c.inBand) snprintf(b, sizeof b, "outside the sample band");
        else snprintf(b, sizeof b, "%llu bursts  %llu good  %llu bad  %llu frames  SNR %.0f  %s", (unsigned long long)c.bursts, (unsigned long long)c.good, (unsigned long long)c.bad,
                      (unsigned long long)c.frames, c.snrDb, c.lastHeardSec > 0 ? (ago(t.nowSec, c.lastHeardSec) + " ago").c_str() : "-");
        dl->AddText(ImVec2(bx + barW + 10 * gUi, ty), c.active ? IM_COL32(200, 204, 208, 255) : IM_COL32(130, 134, 138, 255), b);
    }
    ImGui::Spacing();
    ImGui::PushTextWrapPos(0);
    ImGui::TextDisabled("Bar: level of the last burst (-110 to -20 dBFS); orange line: the channel's noise floor. Green dot: a good burst in the last 2 s.");
    ImGui::PopTextWrapPos();
    ImGui::EndChild();
}

// ---------------------------------------------------------------- hooks

void tab(App& a) {
    loadState();
    subNav("vdl2v", S.viewMode, {"Messages", "Map", "Aircraft", "Ground stations", "Channels"});
    if (S.viewMode == 0) messageView(a);
    else if (S.viewMode == 1) mapView(a);
    else if (S.viewMode == 2) aircraftView(a);
    else if (S.viewMode == 3) stationView(a);
    else channelView(a);
}

void list(App& a) {
    if (!live(a)) { ImGui::PushTextWrapPos(0); ImGui::TextDisabled(a.engine.running() ? "starting" : "start the receiver to see aircraft"); ImGui::PopTextWrapPos(); return; }
    const dect2::Vdl2Telemetry& t = a.rx.vdl2;
    ImGui::PushTextWrapPos(0); ImGui::TextDisabled("%zu aircraft, %zu ground stations", t.aircraft.size(), t.stations.size()); ImGui::PopTextWrapPos();
    if (!ImGui::BeginTable("##vdl_s", 3, ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY | ImGuiTableFlags_ScrollX | ImGuiTableFlags_SizingFixedFit, ImVec2(0, ImGui::GetContentRegionAvail().y))) return;
    ImGui::TableSetupScrollFreeze(0, 1);
    ImGui::TableSetupColumn("ICAO"); ImGui::TableSetupColumn("Reg / flight"); ImGui::TableSetupColumn("Frames");
    ImGui::TableHeadersRow();
    for (const auto& c : t.aircraft) {
        ImGui::TableNextRow();
        ImGui::TableNextColumn();
        if (ImGui::Selectable((hex6(c.icao) + "##vl" + hex6(c.icao)).c_str(), S.selAir == c.icao, ImGuiSelectableFlags_SpanAllColumns)) { S.selAir = c.icao; snprintf(S.fAir, sizeof S.fAir, "%s", hex6(c.icao).c_str()); }
        ImGui::TableNextColumn(); ImGui::Text("%s %s", c.reg.c_str(), c.flight.c_str());
        ImGui::TableNextColumn(); ImGui::Text("%u", c.messages);
    }
    ImGui::EndTable();
}

void receiver(App& a) {
    if (!live(a)) { ImGui::TextDisabled("%s", a.engine.running() ? "starting" : "stopped"); return; }
    const dect2::Vdl2Telemetry& t = a.rx.vdl2;
    auto kv = [&](const char* k, const char* fmt, auto... v) { ImGui::TextDisabled("%s", k); kvColumn(130 * gUi); ImGui::PushFont(a.mono, 0); ImGui::PushTextWrapPos(0); ImGui::Text(fmt, v...); ImGui::PopTextWrapPos(); ImGui::PopFont(); };
    int inBand = 0, act = 0;
    uint64_t bursts = 0;
    for (const auto& c : t.channels) { inBand += c.inBand; act += c.active; bursts += c.bursts; }
    kv("centre", "%.4f MHz, %d of %zu channels in the band, %d active", t.centerHz / 1e6, inBand, t.channels.size(), act);
    kv("rates", "%.0f kS/s in, %.1f kS/s per channel (%.2f samples per symbol)", t.inputRate / 1e3, t.channelRate / 1e3, t.channelRate / 10500);
    kv("bursts", "%llu training sequences, %llu failed", (unsigned long long)bursts, (unsigned long long)t.blocksBad);
    kv("frames", "%llu good, %llu failed their check sequence", (unsigned long long)t.blocksOk, (unsigned long long)t.fcsBad);
    kv("FEC", "%llu octets corrected by Reed-Solomon, %llu headers repaired", (unsigned long long)t.rsFixedTotal, (unsigned long long)t.headerFixed);
    kv("tuning error", "%+.0f Hz (measured from the bursts)", t.cfoHz);
    kv("last SNR", "%.1f dB", t.snrDb);
    kv("positions", "%llu frames carried a position", (unsigned long long)t.positionsTotal);
}

void status(App& a) {
    const bool run = a.engine.running(), on = live(a);
    const dect2::Vdl2Telemetry& t = a.rx.vdl2;
    const SignalStats& st = a.spec.stats;
    const AdcStatus adc = classifyAdc(st.rmsDbfs, st.peak, st.clipFraction);
    StatusPanel panel;
    int act = 0;
    for (const auto& c : t.channels) act += c.active;
    lamp("IQ", run ? (adc == AdcStatus::Overload ? 3 : (adc == AdcStatus::Good ? 1 : 2)) : 0, (int)Ic::Wave); flowNext(12 * gUi);
    lamp("Bursts", !on ? 0 : t.state >= 1 ? 1 : 0); flowNext(12 * gUi);
    lamp("Frames", !on ? 0 : t.state == 2 ? 1 : t.blocksOk > 0 ? 2 : 0); flowNext(10 * gUi);
    ImGui::TextDisabled("|"); flowNext(10 * gUi);
    auto ro = [&](const char* label, const std::string& val) {
        ImGui::AlignTextToFramePadding();
        ImGui::TextDisabled("%s", label); ImGui::SameLine(0, 5 * gUi);
        ImGui::PushFont(a.mono, 0); ImGui::TextUnformatted(val.c_str()); ImGui::PopFont();
        flowNext(15 * gUi);
    };
    char b[64];
    if (!on) { ro("State", run ? "starting" : "stopped"); return; }
    ro("State", t.state == 2 ? "Decoding" : t.state == 1 ? "Bursts" : "Listening");
    snprintf(b, sizeof b, "%d", act); ro("Active", b);
    snprintf(b, sizeof b, "%llu / %llu", (unsigned long long)t.blocksOk, (unsigned long long)t.blocksBad); ro("Frames / bad", b);
    snprintf(b, sizeof b, "%zu", t.aircraft.size()); ro("Aircraft", b);
}

void summary(const App& a, std::string& l1, std::string& l2) {
    l1 = "VDL Mode 2";
    if (live(a)) l2 = dect2::vdl2Summary(a.rx.vdl2);
}

void tuner(App& a, bool&) {
    loadState();
    (void)a;
    ImGui::TextDisabled("Channels MHz");
    ImGui::SetNextItemWidth(std::min(230 * gUi, ImGui::GetContentRegionAvail().x));
    if (ImGui::InputTextWithHint("##vdch", "Europe: 136.725 - 136.975", S.chans, sizeof S.chans)) plat::prefs().setS("vdl2Chans", S.chans);
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("The VDL2 channels to decode, separated by spaces. Empty: the European set,\n136.725 136.775 136.825 136.875 136.900 and 136.975 (the common signalling channel).\nNorth America uses 136.650 136.700 136.800 136.975. Tune between the channels: every channel inside the sample band is decoded.");
}

void synth(App& a, bool& changed) {
    SynthConfig& sc = a.tune.synth;
    ImGui::PushTextWrapPos(0); ImGui::TextDisabled("simulated aircraft"); ImGui::PopTextWrapPos();
    flowNext(); ImGui::SetNextItemWidth(90 * gUi);
    int n = sc.modeOpt[0] > 0 ? sc.modeOpt[0] : 6;
    if (ImGui::SliderInt("##vdn", &n, 1, 24, "%d aircraft")) { sc.modeOpt[0] = n; changed = true; }
    flowNext(); ImGui::SetNextItemWidth(110 * gUi);
    static const char* sets[] = {"Europe (6)", "136.975 only"};
    int cs = std::max(0, std::min(1, sc.modeOpt[2]));
    if (ImGui::Combo("##vdset", &cs, sets, 2)) { sc.modeOpt[2] = cs; changed = true; }
    flowNext(); ImGui::TextDisabled("rate"); ImGui::SameLine(); ImGui::SetNextItemWidth(80 * gUi);
    float rate = sc.modeVal[0] > 0 ? (float)sc.modeVal[0] : 1.f;
    if (ImGui::SliderFloat("##vdrate", &rate, 0.5f, 4.f, "x%.1f")) { sc.modeVal[0] = rate; changed = true; }
    flowNext(); ImGui::TextDisabled("weak ch."); ImGui::SameLine(); ImGui::SetNextItemWidth(80 * gUi);
    float att = (float)sc.modeVal[1];
    if (ImGui::SliderFloat("##vdatt", &att, 0.f, 30.f, "-%.0f dB")) { sc.modeVal[1] = att; changed = true; }
    flowNext(); ImGui::TextDisabled("CFO"); ImGui::SameLine(); ImGui::SetNextItemWidth(90 * gUi);
    float cfo = (float)(sc.cfoHz / 1e3);
    if (ImGui::SliderFloat("##vdcfo", &cfo, -5, 5, "%.2f kHz")) { sc.cfoHz = cfo * 1e3; changed = true; }
}

void meters(const App& a, std::vector<ModeMeter>& out) {
    const dect2::Vdl2Telemetry& t = a.rx.vdl2;
    int act = 0;
    for (const auto& c : t.channels) act += c.active;
    out.push_back({"ACTIVE CH.", "%.0f", (double)act, 0, 6, act > 0 ? 1 : 0});
    out.push_back({"SNR  dB", "%.0f", t.snrDb, 0, 35, t.snrDb >= 20 ? 1 : t.snrDb >= 14 ? 2 : 3});
    out.push_back({"AIRCRAFT", "%.0f", (double)t.aircraft.size(), 0, 30, t.aircraft.empty() ? 0 : 1});
    const double tot = (double)(t.blocksOk + t.blocksBad);
    const double good = tot > 0 ? 100.0 * (double)t.blocksOk / tot : 0.0;
    out.push_back({"GOOD  %", "%.0f", good, 0, 100, tot == 0 ? 0 : good >= 90 ? 1 : good >= 60 ? 2 : 3});
}

} // namespace

extern const ModeUi kVdl2Ui;
const ModeUi kVdl2Ui = {
    .sideTitle = "AIRCRAFT",
    .tabName = "Messages",
    .tabIcon = Ic::Doc,
    .tab = tab,
    .receiver = receiver,
    .list = list,
    .status = status,
    .summary = summary,
    .tuner = tuner,
    .synth = synth,
    .tick = tick,
    .meters = meters,
};
