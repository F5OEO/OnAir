# Voice and audio codecs

OnAir does not ship some voice and audio codecs: either they are patented, or they cannot be bundled. You can install the ones that
exist as free software yourself, and OnAir will use them. For the others, this page says what OnAir does and which separate program
you can use.

In OnAir, each mode that needs such a codec has a small **?** button next to its sound or voice controls. It shows whether the codec
was found (and where), how to install it on your computer, where to put it, and a **Look again** button that searches again without a
restart.

| Mode | Codec | What OnAir does | Can OnAir load it? |
|---|---|---|---|
| HF digital, FreeDV | codec2 | plays speech when codec2 is installed | **yes** |
| DMR | AMBE+2 | finds, counts and logs voice calls; no sound | no |
| D-STAR | AMBE (3600x2400) | call log, messages, positions; plays voice when mbelib is installed | **yes** (mbelib) |
| HD Radio | HDC | stations, data and pictures; no sound | no |
| CDR | DRA+ | service list; no sound | no |
| Inmarsat Aero | AMBE (C channel voice) | P channel data only; voice channels are not tuned | no |
| Iridium | AMBE variant | counts voice frames; saves `.bits` for iridium-toolkit | no |
| DAB/DAB+, DRM, TV modes | MP2, AAC, xHE-AAC, MPEG-2, H.264, HEVC, AC-3 | plays (FFmpeg is built into OnAir) | nothing to install |

## codec2 (FreeDV)

codec2 is the open voice codec of FreeDV (LGPL-2.1). It is free software; OnAir simply does not ship it yet.

### macOS (Homebrew)

1. Install Homebrew: <https://brew.sh>
2. In Terminal: `brew install codec2` (formula page: <https://formulae.brew.sh/formula/codec2>)

Homebrew puts it in `/opt/homebrew/lib` (Apple silicon) or `/usr/local/lib` (Intel). OnAir looks there; nothing to copy.

### Linux

- Debian / Ubuntu: `sudo apt install libcodec2-1.2` (Ubuntu 22.04: `libcodec2-1.0`).
  Package list: <https://packages.ubuntu.com/search?keywords=libcodec2>
- Fedora: `sudo dnf install codec2`. Package page: <https://packages.fedoraproject.org/pkgs/codec2/codec2/>

The package puts it in the system library folder; OnAir looks there. A portable OnAir also finds it there.

### Windows

The codec2 project (<https://github.com/drowe67/codec2>) has no ready-made Windows download, but the FreeDV program carries it:

1. Download and install FreeDV from <https://freedv.org/download/> (`FreeDV-<version>-windows-x86_64.exe` for most PCs,
   `...-aarch64.exe` for Windows on ARM).
2. Open its `bin` folder (for example `C:\Program Files\FreeDV <version>\bin`) and copy `libcodec2.dll`.
3. Paste it into an OnAir codecs folder (below). Make the folder if it is not there.

If OnAir still says "not found", the DLL may need another DLL from the same `bin` folder; copy that too. The DLL must be for the same
kind of Windows as OnAir (x86_64).

### The OnAir codecs folders

OnAir looks in these folders first, then in the system's usual places:

| OS | Next to the program | In your user folder |
|---|---|---|
| Windows | `codecs\` next to `OnAir.exe` | `%LOCALAPPDATA%\OnAir\codecs\` |
| macOS | `codecs/` next to `OnAir.app` | `~/Library/Application Support/OnAir/codecs/` |
| Linux | `codecs/` next to the `dect2` program | `~/.local/share/onair/codecs/` (or `$XDG_DATA_HOME/onair/codecs/`) |

File names: `libcodec2.dll` or `codec2.dll` (Windows), `libcodec2.dylib` or `libcodec2.1.2.dylib` (macOS), `libcodec2.so` or
`libcodec2.so.1.2` (Linux).

### Check that OnAir found it

Open the HF digital mode and pick FreeDV. The **Library** lamp turns green, and the **?** button says
`codec2 found: <path>`. If you installed it while OnAir was open, press **Look again** in that window; no restart is needed.

## AMBE+2 (DMR voice)

DMR voice uses the AMBE+2 vocoder (DVSI), which is patented. OnAir finds, counts and logs voice calls but plays no sound, and it cannot
load an AMBE library. The open mbelib library (<https://github.com/szechyjs/mbelib>) implements these vocoders; its own notice says
that using it may need a patent licence. Wiring it into OnAir's DMR receiver is not a small change, so it is not done.

To hear DMR voice, use a separate program such as DSD-FME (<https://github.com/lwvmobile/dsd-fme>).

## AMBE (D-STAR voice) through mbelib

D-STAR voice uses AMBE (the 3600x2400 variant), which is patented. Without a codec OnAir shows every call (callsigns, repeaters,
flags), the slow data message, the GPS and D-PRS positions and the voice frame count, but plays no sound. If you install the open
mbelib library (<https://github.com/szechyjs/mbelib>, ISC licence) yourself, OnAir loads it at run time and plays D-STAR voice. mbelib's
own notice says that using it may need a patent licence; check the rules where you live first.

mbelib has no ready-made downloads, so it is built from source (it needs CMake and a C compiler):

### macOS

1. Install Homebrew (<https://brew.sh>), then `brew install cmake`.
2. `git clone https://github.com/szechyjs/mbelib`, then `cd mbelib && mkdir build && cd build && cmake .. && make`.
3. Copy `libmbe.dylib` (or `libmbe.1.3.dylib`) into an OnAir codecs folder (above), or run `sudo make install` (`/usr/local/lib`).

### Linux

1. Install `git`, `cmake` and a compiler (Debian / Ubuntu: `sudo apt install git cmake build-essential`; Fedora:
   `sudo dnf install git cmake gcc`).
2. `git clone https://github.com/szechyjs/mbelib`, then `cd mbelib && mkdir build && cd build && cmake .. && make && sudo make install`
   (then `sudo ldconfig`), or copy `libmbe.so*` into an OnAir codecs folder.

### Windows

Build mbelib with CMake and Visual Studio (x64), or take `mbe.dll` / `libmbe.dll` from a Windows build of DSD-FME
(<https://github.com/lwvmobile/dsd-fme>), and put it into an OnAir codecs folder (above).

File names OnAir looks for: `mbe.dll`, `libmbe.dll` or `mbelib.dll` (Windows), `libmbe.dylib` or `libmbe.1.3.dylib` (macOS),
`libmbe.so`, `libmbe.so.1` or `libmbe.so.1.3` (Linux). Open D-STAR: the **?** next to the voice says `mbelib found: <path>`;
**Look again** searches again without a restart.

## AMBE+2 (NXDN voice) through mbelib

NXDN voice (NXDN48 and NXDN96) uses AMBE+2 (the 3600x2450 variant), which is patented. Without a codec OnAir shows every call (unit
IDs, talkgroups, call type, encryption), status and short data messages, the site information and the voice frame count, but plays
no sound. If you install the open mbelib library (<https://github.com/szechyjs/mbelib>, ISC licence) yourself, OnAir loads it at run
time and plays NXDN voice; encrypted calls stay silent. mbelib's own notice says that using it may need a patent licence; check the
rules where you live first.

It is the same library as for D-STAR: build and install it as described there (macOS, Linux, Windows), with the same file names. Open
NXDN: the **?** next to the voice says `mbelib found: <path>`; **Look again** searches again without a restart.

## HDC (HD Radio sound)

HD Radio sound uses HDC, a patented codec. OnAir shows the stations, the data and the pictures, but plays no sound. To hear HD Radio,
use nrsc5 (<https://github.com/theori-io/nrsc5>) with an RTL-SDR or an IQ recording.

## DRA+ (CDR sound)

CDR sound uses DRA+, a patented codec that is not openly described. OnAir lists the services but plays no sound. There is no free
DRA+ decoder to install at this time.

## AMBE (Inmarsat Aero voice)

Aero C channel voice uses an AMBE vocoder. OnAir decodes the P channel data only and does not tune voice channels. Separate programs,
for example JAERO (<https://github.com/jontio/JAERO>), work on C channels.

## AMBE variant (Iridium voice)

OnAir counts Iridium voice frames but does not decode them. It hands the data off instead: **Save for iridium-toolkit** writes a
`.bits` file. Then:

1. Get iridium-toolkit: <https://github.com/muccc/iridium-toolkit>
2. `iridium-parser.py output.bits > output.parsed`
3. Install the `ir77_ambe_decode` program from osmo-ir77 (<http://git.osmocom.org/osmo-ir77>), as the toolkit's README explains, and
   use its `stats-voc.py` or `voc-cluster.py` to play the voice.

## AAC, xHE-AAC and the other FFmpeg codecs

DAB/DAB+ (MP2, AAC), DRM (AAC, xHE-AAC) and the TV modes decode with FFmpeg, which is built into every OnAir download (FFmpeg 8.0).
Nothing to install, with one exception: the SBR and parametric stereo of DAB+ HE-AAC (HE-AAC and HE-AAC v2 stations). FFmpeg cannot apply
them to the 960-sample frames of DAB+ ("SBR with 960 frame length"), so without help those stations play the core only: band-limited, and
HE-AAC v2 in mono. OnAir uses faad2 for them when it is installed (Linux: the `libfaad2` package; macOS: `brew install faad2`; Windows: a
`libfaad2.dll` beside the program). It is found at run time and not shipped; the DAB player panel says which decoder is working and whether SBR
and PS are applied. Some xHE-AAC tools (harmonic SBR, PVC, inter-TES, MPEG Surround 212) are not in FFmpeg yet; DRM stations that use
them stay silent and the DRM view says so.

## Legal note

Some of these codecs are covered by patents in some countries. Whether you install and use them is your choice and your
responsibility; check the rules where you live. OnAir does not ship them, does not download them, and contains no code from them.
