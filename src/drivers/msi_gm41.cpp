// MSI Clutch GM41 Lightweight Wireless (and its Stalker edition). Battery is pushed, never queried.
//
// The mouse sends an unsolicited 64-byte input report on its vendor collection FF10:0006
// (interface 1) every few seconds while it is being used:
//   A0 <raw> 07 <flags> 94 00 94      (bytes after the dropped report-ID 0)
// <raw> falls as the battery drains; bytes 4..5 repeat the firmware version (0x0094 here, the
// same value the F5 "firmware version" command returns). <flags> was seen as 00 and 04; its
// meaning is not confirmed, so it is only shown in the tooltip.
//
// Sources:
//  * Product ids from MSI Center (MysticLight_AllDevice.dll, Class_GM41W.SupportList_PID):
//    0D4C wireless receiver, 0D4B cable; Stalker 4518 receiver, 04E2 cable. The same class shows
//    the 0D <cmd> framing on this collection (writes 0x31.., reads with bit 7 set).
//  * Report layout and calibration samples: liamgibsonn/mouse-protocol PR #1 notes
//    (raw 0x4D = 57 %, 0x48 = 54 %, 0x45 = 53 %, from MSI's own percentage).
//  * Own capture on a GM41 receiver 0DB0:0D4C, firmware 0x0094: "A0 75 07 00 94 00 94" every
//    1-11 s while the mouse moved, nothing while it rested.
//
// The percentage is a linear fit through the three published samples (about 0.51 % per step)
// and is approximate, especially near empty and full; the raw value is shown in the tooltip so
// the mapping can be refined.
//
// Because reports only arrive while the mouse moves, the driver keeps the collection open
// between polls (Windows queues input reports for an open handle) and remembers the last value.
// Nothing is ever written to the device.
#include "drivers.h"

#include <map>
#include <memory>
#include <mutex>

namespace {

struct Model { USHORT pid; bool cable; const wchar_t* name; };
const Model kModels[] = {
    { 0x0D4C, false, L"MSI Clutch GM41 Lightweight Wireless" },
    { 0x0D4B, true,  L"MSI Clutch GM41 Lightweight Wireless" },
    { 0x4518, false, L"MSI Clutch GM41 Lightweight Wireless Stalker" },
    { 0x04E2, true,  L"MSI Clutch GM41 Lightweight Wireless Stalker" },
};

const Model* FindModel(USHORT pid) {
    for (const auto& m : kModels)
        if (m.pid == pid) return &m;
    return nullptr;
}

std::wstring Hex2(BYTE v) {
    const wchar_t* d = L"0123456789ABCDEF";
    return { d[v >> 4], d[v & 15] };
}

// Linear fit through (0x4D, 57), (0x48, 54), (0x45, 53): percent ~= 0.51 * raw + 17.6.
int RawToPercent(int raw) { return Clamp100((raw * 51 + 1760 + 50) / 100); }

struct Listener {
    std::wstring path;
    std::unique_ptr<HidDev> dev;
    int raw = -1;
    BYTE flags = 0;
    ULONGLONG at = 0;  // tick of the last battery report
};

std::mutex g_lock;
std::map<USHORT, Listener> g_listeners;  // by product id

bool Decode(const std::vector<BYTE>& r, Listener& l) {
    if (r.size() < 7 || r[0] != 0xA0 || r[2] != 0x07) return false;
    l.raw = r[1];
    l.flags = r[3];
    l.at = GetTickCount64();
    return true;
}

const ULONGLONG kFreshMs = 10 * 60 * 1000;   // a report this recent is shown as current
const ULONGLONG kReopenMs = 2 * 60 * 1000;   // reopen a silent handle (receiver may have been replugged)

}  // namespace

void PollMsiGm41(PollContext& ctx) {
    std::lock_guard<std::mutex> guard(g_lock);
    for (auto [vid, pid] : ctx.Products(0x0DB0)) {
        const Model* model = FindModel(pid);
        if (!model) continue;
        auto colls = ctx.Find(vid, pid, [](const HidColl& c) {
            return c.page() == 0xFF10 && c.usage() == 0x0006 && c.inLen() >= 8;
        });
        if (colls.empty()) continue;
        const HidColl* c = colls.front();

        Listener& l = g_listeners[pid];
        if (!l.dev || !l.dev->ok() || l.path != c->path) {
            l.dev = std::make_unique<HidDev>();
            if (!l.dev->OpenReadOnly(*c)) { l.dev.reset(); continue; }
            l.path = c->path;
        }

        // Take everything Windows queued since the last poll; keep the newest battery report.
        bool fresh = false;
        std::vector<BYTE> r;
        for (int i = 0; i < 128 && l.dev->Read(r, 1) >= 0; i++) fresh |= Decode(r, l);

        // Nothing recent (first poll, or the mouse rested): give a moving mouse a moment to report.
        if (!fresh && (l.raw < 0 || GetTickCount64() - l.at > kFreshMs)) {
            ULONGLONG end = GetTickCount64() + 3000;
            while (!fresh) {
                int left = (int)((LONGLONG)end - (LONGLONG)GetTickCount64());
                if (left <= 0 || l.dev->Read(r, left) < 0) break;
                fresh = Decode(r, l);
            }
        }
        if (!fresh && l.at && GetTickCount64() - l.at > kReopenMs) l.dev.reset();

        if (l.raw < 0 || GetTickCount64() - l.at > kFreshMs) continue;  // grace keeps the last value, dimmed

        DeviceStatus s;
        s.key = L"msi:" + Hex4(model->cable ? (USHORT)(pid == 0x04E2 ? 0x4518 : 0x0D4C) : pid);
        s.name = model->name;
        s.kind = DeviceKind::Mouse;
        s.via = model->cable ? L"USB cable" : L"2.4 GHz receiver";
        s.percent = RawToPercent(l.raw);
        s.detail = L"approximate (raw 0x" + Hex2((BYTE)l.raw) + L", flags 0x" + Hex2(l.flags) + L")";
        s.graceSec = 1800;
        ctx.Add(s, vid, pid);
    }
}
