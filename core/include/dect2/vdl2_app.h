// ATN application data on VDL Mode 2: the ULCS wrapper (X.225 session SPDU, X.226 presentation, X.227 ACSE), and the ASN.1 unaligned PER
// payloads it carries: context management (CM), CPDLC. Facts from ICAO Doc 9705 / 9880 and the ATN B1 CPDLC message set as dumpvdl2
// reads them (reference only, nothing copied). A decode is accepted only when it uses the data up to its last octet; anything the
// reader does not know leaves the octets as hex.
#pragma once
#include <cstddef>
#include <cstdint>
#include <string>

namespace dect2 {

struct Vdl2AppResult {
    std::string detail;      // indented lines for the detail pane (without the hex of the octets)
    std::string summary;     // a short phrase for the list: "CPDLC WILCO", "CM logon request"
    bool decoded = false;    // an application message was read (not only the session / ACSE headers)
};

// `p` / `n`: the transport user data of a COTP DT. `fromAir`: sent by the aircraft.
Vdl2AppResult vdl2AtnApp(const uint8_t* p, size_t n, bool fromAir);

} // namespace dect2
