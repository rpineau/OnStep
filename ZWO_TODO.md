# ZWO AMx Mount Support — Feature Status

## X2 Interface Comparison

| Interface | OnStep | RST | SkyWatcher | iOptronV3 | This Plugin |
|-----------|--------|-----|------------|-----------|-------------|
| MountDriverInterface | ✅ | ✅ | ✅ | ✅ | ✅ |
| SyncMountInterface | ✅ | ✅ | ✅ | ✅ | ✅ |
| SlewToInterface | ✅ | ✅ | ✅ | ✅ | ✅ |
| AsymmetricalEquatorialInterface | ✅ | ✅ | ✅ | ✅ | ✅ |
| OpenLoopMoveInterface | ✅ | ✅ | ✅ | ✅ | ✅ |
| TrackingRatesInterface | ✅ | ✅ | ✅ | ✅ | ✅ |
| ParkInterface | ✅ | ✅ | ✅ | ✅ | ✅ |
| UnparkInterface | ✅ | ✅ | ✅ | ✅ | ✅ |
| ModalSettingsDialogInterface | ✅ | ✅ | ✅ | ✅ | ✅ |
| X2GUIEventInterface | ✅ | ✅ | ✅ | ✅ | ✅ |
| SerialPortParams2Interface | ✅ | ✅ | ✅ | ✅ | ✅ |
| DriverSlewsToParkPositionInterface | ✅ | ✅ | ❌ | ✅ | ✅ |
| PulseGuideInterface2 | ✅ | ❌ | ✅ | ❌ | ✅ |
| NeedsRefractionInterface | ✅ | ✅ | ✅ | ✅ | ✅ |
| FindHomeInterface | ❌ | ❌ | ❌ | ❌ | ✅ |

We implement every interface that any of the comparison plugins implement, plus FindHomeInterface (undocumented).

## ZWO Feature Implementation Status

| Feature | ZWO Protocol Cmd | Implemented | ZWO Override | Tested | Result |
|---------|-----------------|-------------|--------------|--------|--------|
| **Connection** | | | | | |
| Serial connect (9600 baud) | — | ✅ | ✅ | ✅ | Pass |
| WiFi connect (192.168.4.1:4030) | — | ✅ (via TSX) | — | ❌ | — |
| Set lat/long on connect | `:SMGE` | ✅ | ✅ | ✅ | Pass |
| Set date/time/tz on connect | `:SMTI` | ✅ | ✅ | ✅ | Pass |
| Auto-home on connect | `:hC#` | ❌ removed | — | — | Removed: mount must not move on connect. Homing via TSX Startup → Find Home (FindHomeInterface). |
| Vendor detection (mountlist) | `pszDriverSelection` | ✅ | — | ✅ | Pass |
| **Coordinates** | | | | | |
| Get RA/DEC | `:GR#` `:GD#` | ✅ | ❌ (base) | ✅ | Pass |
| Get Alt/Az | `:GA#` `:GZ#` | ✅ | ❌ (base) | ❌ | — |
| Get pier side | `:Gm#` | ✅ | ❌ (base) | ✅ | Pass |
| **Slewing** | | | | | |
| Set target RA/DEC | `:Sr` `:Sd` | ✅ | ❌ (base) | ✅ | Pass |
| GOTO (RA/DEC) | `:MS#` | ✅ | ❌ (base†) | ✅ | Pass |
| GOTO (Alt/Az) | `:MA#` | ⚠️ | ❌ (base†) | ❌ | `:MA#` does NOT exist in ZWO protocol v2.1. ZWO has no AltAz GOTO. Command times out and is silently ignored. `gotoPark()` override avoids this path entirely. |
| Abort | `:Q#` | ✅ | ❌ (base) | ❌ | — |
| Slew complete check | `:GU#` | ✅ | ❌ (base) | ✅ | Pass |
| **Tracking** | | | | | |
| Start tracking | `:Te#` | ✅ | ❌ (base) | ✅ | Pass |
| Stop tracking | `:Td#` | ✅ | ❌ (base) | ✅ | Pass |
| Sidereal rate | `:TQ#` | ✅ | ❌ (base) | ❌ | — |
| Lunar rate | `:TL#` | ✅ | ❌ (base) | ❌ | — |
| Solar rate | `:TS#` | ✅ | ❌ (base) | ❌ | — |
| Get tracking rate | `:GT#` | ✅ | ❌ (base) | ❌ | — |
| Get tracking status | `:GAT#` | ✅ | ✅ | ❌ | ZWO override uses `:GAT#`; response `0`=off, non-zero=on. Falls back to cached `m_bIsTracking` on error. |
| **Open Loop Move** | | | | | |
| Set speed | `:Rn#` | ✅ | ❌ (base) | ❌ | — |
| Move N/S/E/W | `:Mn#` `:Ms#` `:Me#` `:Mw#` | ✅ | ❌ (base) | ❌ | — |
| Stop N/S/E/W | `:Qn#` `:Qs#` `:Qe#` `:Qw#` | ✅ | ❌ (base) | ❌ | — |
| **Guiding** | | | | | |
| Pulse guide (via OLM) | `:Me#`/`:Mw#` etc | ✅ | ❌ (base) | ❌ | Needs nighttime |
| Native pulse guide | `:Mgdnnnn#` | ✅ | ✅ | ❌ | Via DirectGuideInterface |
| Set guide rate | `:Rg0.nn#` | ✅ | ✅ | ❌ | `setGuideRate()` sends `:Rg%.2f#`. Called on settings dialog save when connected. |
| Get guide rate | `:Ggr#` | ✅ | ✅ | ❌ | `getGuideRate()` called at end of `Connect()`; result synced into `X2Mount::m_dZWOGuideRate` so DirectGuide math and settings dialog both reflect actual mount state. |
| **Sync** | | | | | |
| Sync position | `:CM#` | ✅ | ❌ (base) | ❌ | — |
| Clear multi-star cal | `:NSC#` | ❌ | — | ❌ | — |
| **Homing** | | | | | |
| Go to zero position | `:hC#` | ✅ | ✅ | ✅ | Pass |
| Check homing done | `:GU#` (H flag) | ✅ | ❌ (base) | ✅ | Pass |
| Query homing success | `:Gh#` | ✅ | ✅ | ✅ | Returns 0 even after homing (firmware issue?) |
| **Parking** | | | | | |
| Park (default position) | `:hP#` | ✅ | ✅ | ✅ | Falls back to :hC# when no park pos set |
| Park status | `:Gps#` | ✅ | ✅ | ✅ | Returns empty when no park pos — fallback works |
| Park status (isParked) | `:Gps#` | ✅ | ✅ | ❌ | ZWO override of `getAtPark()` uses `:Gps#`; `2`=parked. Was broken: base used `:GU#` P-flag which ZWO doesn't set. |
| Set custom park position | `:Sp01#` | ✅ | ✅ | ❌ | `setCurentPosAsPark()` callable via "Set Current Position as Park" button in `groupBox_zwoAdvanced`. TSX "Set Park Position" has no driver callback — it stores position internally and passes it via `startPark(dAz, dAlt)`, which we cannot honor (no AltAz slew on ZWO). |
| Unpark | `:Spu#` | ✅ | ✅ | ❌ | ZWO override sends `:Spu#` instead of `:hR#` |
| **Meridian Limits** | | | | | |
| Get limits | `:GTa#` | ✅ | ✅ | ✅ | Pass (returns 0 — mount default) |
| Set meridian behavior | `:STannsnn#` | ✅ | ✅ | ❌ | Via settings dialog; applied on OK. Was broken: sent `:STA…` (uppercase A) — fixed to `:STa…`. Response format: nn=two flag bits (flip+continue-tracking), snn=signed limit angle. |
| Get flip hour angle | `:GTa#` | ✅ | ✅ | ✅ | Pass |
| Beyond the pole | `:Gm#` | ✅ | ❌ (base) | ✅ | Pass |
| **Height Limits** | | | | | |
| Enable/disable | `:SLE#` `:SLD#` | ✅ | ✅ | ❌ | Via settings dialog; enabled state persisted in INI (mount has no read-back). |
| Set upper/lower limit | `:SLHnn#` `:SLLnn#` | ✅ | ✅ | ❌ | Via settings dialog; applied on OK. Was broken: `:SLL` was sent with a leading `+` sign (`:SLL+15#`) — fixed to unsigned `%02d` (`:SLL15#`). |
| Get limits | `:GLH#` `:GLL#` | ✅ | ✅ | ❌ | Read from mount when dialog opens (if connected). |
| **Alignment** | | | | | |
| isAligned (homed check) | `:Gh#` + internal flag | ✅ | ✅ | ✅ | Pass (uses m_bHasBeenHomed fallback) |
| **Site Data** | | | | | |
| Set longitude | `:Sg` | ✅ | ❌ (base) | ✅ | Pass |
| Set latitude | `:St` | ✅ | ❌ (base) | ✅ | Pass |
| Set timezone | `:SG` | ✅ | ✅ (via SMTI) | ✅ | Pass |
| Set date | `:SC` | ✅ | ✅ (via SMTI) | ✅ | Pass |
| Set time | `:SL` | ✅ | ✅ (via SMTI) | ✅ | Pass |
| Get sidereal time | `:GS#` | ✅ | ❌ (base) | ❌ | — |
| Daylight saving | `:GH#` `:SHn#` | ❌ | — | ❌ | — |
| **Device Info** | | | | | |
| Firmware version | `:GVN#` | ✅ | ❌ (base) | ❌ | — |
| Device name | `:GVP#` | ✅ | ❌ (base) | ❌ | — |
| **Mode** | | | | | |
| EQ/AZ mode switch | `:AP#` `:AA#` | ❌ | — | ❌ | Not needed (always EQ mode) |

† Base class `slewTargetRaDecEpochNow` / `slewTargetAltAszEpochNow` modified to handle ZWO `e` + error code response format and error code 7.

## Priority TODO Items

### Must Fix
- [x] **Unpark**: ZWO override sends `:Spu#` instead of base `:hR#`. ✅
- [x] **Set custom park position**: `:Sp01#` callable via "Set Current Position as Park" button in ZWO Advanced settings group. TSX's "Set Park Position" menu has no driver callback; our button is the ZWO equivalent. ✅

### Should Implement
- [x] **Native pulse guide** (`:Mgdnnnn#`): Implemented via `DirectGuideInterface` — more precise than OLM-based guiding. ✅
- [x] **Guide rate get/set** (`:Rg0.nn#` / `:Ggr#`): Exposed in settings dialog, persisted via INI (`m_dZWOGuideRate`). ✅
- [x] **Dynamic UI Hiding**: ZWO shows only relevant controls; standard OnStep shows only its controls. For ZWO: time/location, home, and park sections hidden (TSX owns these via interfaces). For OnStep: ZWO guide rate and advanced limits hidden. ✅
- [x] **Get tracking status** (`:GAT#`): `ZWOMount::isTrackingOn()` override uses `:GAT#` (response `0`=off, non-zero=on) instead of `:GU#` bitmask. ✅
- [x] **isParked() fix**: `ZWOMount::getAtPark()` override uses `:Gps#` instead of `:GU#` P-flag, which ZWO firmware doesn't set. ✅ Both `getAtPark` and `isTrackingOn` required `virtual` added to base class. ✅
- [x] **Height limits** (`:SLE#`/`:SLD#`/`:SLHnn#`/`:SLLnn#`/`:GLH#`/`:GLL#`): Exposed in settings dialog (`groupBox_zwoAdvanced`), ZWO-only. ✅
- [x] **Meridian behavior config** (`:STannsnn#`): Exposed in settings dialog (track + slew past meridian), applied on OK. ✅

### Nice to Have
- [ ] **Daylight saving** (`:GH#`/`:SHn#`): The SMTI compound command handles timezone; DST might need explicit handling.
- [ ] **Clear multi-star calibration** (`:NSC#`): Add button to settings dialog.
- [ ] **WiFi connection**: Test over WiFi (192.168.4.1:4030) in addition to USB serial.

### Not Needed
- EQ/AZ mode switch — always equatorial mode for this use case
- Bluetooth commands — not applicable to X2 serial/TCP

## Testing Checklist

| Test | Status | Notes |
|------|--------|-------|
| Connect (USB serial) | ✅ Pass | Auto-detects ZWO from mountlist |
| Auto-home on connect | N/A — removed | Homing is user-initiated via TSX Find Home |
| Site data sync (SMGE+SMTI) | ✅ Pass | Always runs on connect; 3-digit longitude padding required |
| Basic slew (GOTO) | ✅ Pass | :MS# with e-prefix error handling |
| Slew complete detection | ✅ Pass | Via :GU# status polling |
| Park (home fallback) | ✅ Pass | Falls back to :hC# when no park pos |
| Start/stop tracking | ✅ Pass | :Te# / :Td# |
| Tracking status readback | ❌ Untested | Implemented via :GAT# override; needs on-mount confirmation of response format |
| Park / isParked() | ❌ Untested | getAtPark() now uses :Gps# override; Unpark button should enable after parking |
| Meridian limits | ✅ Pass | :GTa# returns mount config (0° default) |
| Flip hour angle | ✅ Pass | Derived from :GTa# |
| Beyond the pole | ✅ Pass | :Gm# returns E/W/N |
| Pier side reporting | ✅ Pass | |
| Abort slew | ❌ Untested | |
| Sync position | ❌ Untested | |
| Open loop move (jog buttons) | ❌ Untested | |
| Pulse guide (autoguiding) | ❌ Untested | Implemented (DirectGuideInterface + `:Mgdnnnn#`), needs nighttime test |
| Unpark | ❌ Untested | Implemented (`:Spu#` override), needs on-mount test |
| Set custom park position | ❌ Untested | "Set Current Position as Park" button in ZWO Advanced settings (enabled only when connected). Calls `:Sp01#`. Needs on-mount test. |
| Tracking rate changes (lunar/solar) | ❌ Untested | |
| WiFi connection | ❌ Untested | |
| Disconnect/reconnect | ❌ Untested | |
| Settings dialog | ❌ Untested | ZWO dialog shows: mount settings, ZWO limits, debug. Time/location and park sections hidden. |

## Undocumented FindHomeInterface ✅ Implemented

This interface enables the "Startup" -> "Find Home" menu option in TSX. If your plugin implements this, TSX queries it upon connection. **Now implemented in this plugin.**

```cpp
class FindHomeInterface {
public:
    virtual ~FindHomeInterface() {}
    virtual int startFindHome() = 0;
    virtual int isCompleteFindHome(bool& bComplete) const = 0;
    virtual int endFindHome() = 0;
};
```

TSX expects `startSlewTo` to return `ERR_MOUNTNOTHOMED` (231) if the mount needs homing but hasn't done it yet.

### TSX "Set Park Position" / "Clear Park Position" — No Driver Interface

Investigated via string search of `TheSkyX` binary, all licensed interface headers, and comparison plugin `.dylib` files. **Conclusion: there is no `SetParkPositionInterface` or `ClearParkPositionInterface`.** TSX handles these menu actions internally:

- **"Set Park Position"**: TSX records the current telescope position in its own state (`m_dSoftwareParkAlt` / `m_dSoftwareParkAz`). No driver callback.
- **"Clear Park Position"**: Clears TSX's internally stored park coordinates. No driver callback.
- **"Park"**: TSX calls `startPark(dAz, dAlt)` with the stored coordinates. Since we implement `DriverSlewsToParkPositionInterface`, TSX expects us to slew there.

For standard OnStep, `startPark(dAz, dAlt)` → `gotoParkPos()` → `:MA#` (AltAz slew). For ZWO, `:MA#` doesn't exist. Our ZWO `gotoPark()` override sends `:hP#` (go to mount's stored park position), ignoring the TSX-provided coordinates.

**ZWO workaround**: Use the "Set Current Position as Park" button in the settings dialog (ZWO Advanced section). Move the mount to the desired park position, then open settings and click the button — this sends `:Sp01#` to store that position in the mount. Subsequent `:hP#` park commands will return to that position.

### ZWO Protocol Implementation Notes

The `FindHomeInterface` is fully implementable for the ZWO protocol. Using the `:hC#` command stops the mount at the zero position. You can check the state with `:GU#` (returns `H` in the bitmask if at home position) or via Park error code 5 (`PARK_NOT_GO_HOME`).
