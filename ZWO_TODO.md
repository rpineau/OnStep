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
| Auto-home on connect | `:hC#` | ✅ | ✅ | ✅ | Pass |
| Vendor detection (mountlist) | `pszDriverSelection` | ✅ | — | ✅ | Pass |
| **Coordinates** | | | | | |
| Get RA/DEC | `:GR#` `:GD#` | ✅ | ❌ (base) | ✅ | Pass |
| Get Alt/Az | `:GA#` `:GZ#` | ✅ | ❌ (base) | ❌ | — |
| Get pier side | `:Gm#` | ✅ | ❌ (base) | ✅ | Pass |
| **Slewing** | | | | | |
| Set target RA/DEC | `:Sr` `:Sd` | ✅ | ❌ (base) | ✅ | Pass |
| GOTO (RA/DEC) | `:MS#` | ✅ | ❌ (base†) | ✅ | Pass |
| GOTO (Alt/Az) | `:MA#` | ✅ | ❌ (base†) | ❌ | — |
| Abort | `:Q#` | ✅ | ❌ (base) | ❌ | — |
| Slew complete check | `:GU#` | ✅ | ❌ (base) | ✅ | Pass |
| **Tracking** | | | | | |
| Start tracking | `:Te#` | ✅ | ❌ (base) | ✅ | Pass |
| Stop tracking | `:Td#` | ✅ | ❌ (base) | ✅ | Pass |
| Sidereal rate | `:TQ#` | ✅ | ❌ (base) | ❌ | — |
| Lunar rate | `:TL#` | ✅ | ❌ (base) | ❌ | — |
| Solar rate | `:TS#` | ✅ | ❌ (base) | ❌ | — |
| Get tracking rate | `:GT#` | ✅ | ❌ (base) | ❌ | — |
| Get tracking status | `:GAT#` | ❌ | — | ❌ | — |
| **Open Loop Move** | | | | | |
| Set speed | `:Rn#` | ✅ | ❌ (base) | ❌ | — |
| Move N/S/E/W | `:Mn#` `:Ms#` `:Me#` `:Mw#` | ✅ | ❌ (base) | ❌ | — |
| Stop N/S/E/W | `:Qn#` `:Qs#` `:Qe#` `:Qw#` | ✅ | ❌ (base) | ❌ | — |
| **Guiding** | | | | | |
| Pulse guide (via OLM) | `:Me#`/`:Mw#` etc | ✅ | ❌ (base) | ❌ | Needs nighttime |
| Native pulse guide | `:Mgdnnnn#` | ✅ | ✅ | ❌ | Via DirectGuideInterface |
| Set guide rate | `:Rg0.nn#` | ✅ | ✅ | ❌ | Via UI + INI (m_dZWOGuideRate) |
| Get guide rate | `:Ggr#` | ✅ | ✅ | ❌ | Via UI + INI (m_dZWOGuideRate) |
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
| Set custom park position | `:Sp01#` | ✅ (via UI) | ✅ | ❌ | Sends `:Sp01#` to set current pos as park |
| Unpark | `:Spu#` | ✅ | ✅ | ❌ | ZWO override sends `:Spu#` instead of `:hR#` |
| **Meridian Limits** | | | | | |
| Get limits | `:GTa#` | ✅ | ✅ | ✅ | Pass (returns 0 — mount default) |
| Set meridian behavior | `:STannsnn#` | ❌ | — | ❌ | — |
| Get flip hour angle | `:GTa#` | ✅ | ✅ | ✅ | Pass |
| Beyond the pole | `:Gm#` | ✅ | ❌ (base) | ✅ | Pass |
| **Height Limits** | | | | | |
| Enable/disable | `:SLE#` `:SLD#` | ❌ | — | ❌ | — |
| Set upper/lower limit | `:SLHnn#` `:SLLnn#` | ❌ | — | ❌ | — |
| Get limits | `:GLH#` `:GLL#` | ❌ | — | ❌ | — |
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
- [x] **Set custom park position**: `:Sp01#` implemented via UI button with ZWO override. ✅

### Should Implement
- [x] **Native pulse guide** (`:Mgdnnnn#`): Implemented via `DirectGuideInterface` — more precise than OLM-based guiding. ✅
- [x] **Guide rate get/set** (`:Rg0.nn#` / `:Ggr#`): Exposed in settings dialog, persisted via INI (`m_dZWOGuideRate`). ✅
- [ ] **Get tracking status** (`:GAT#`): Better error reporting when tracking fails.
- [ ] **Height limits** (`:SLE#`/`:SLD#`/`:SLHnn#`/`:SLLnn#`/`:GLH#`/`:GLL#`): Expose in settings dialog for ZWO users.
- [ ] **Meridian behavior config** (`:STannsnn#`): Expose in settings dialog for ZWO users.

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
| Auto-home on connect | ✅ Pass | Sends :hC#, waits for completion |
| Site data sync (SMGE+SMTI) | ✅ Pass | 3-digit longitude padding required |
| Basic slew (GOTO) | ✅ Pass | :MS# with e-prefix error handling |
| Slew complete detection | ✅ Pass | Via :GU# status polling |
| Park (home fallback) | ✅ Pass | Falls back to :hC# when no park pos |
| Start/stop tracking | ✅ Pass | :Te# / :Td# |
| Meridian limits | ✅ Pass | :GTa# returns mount config (0° default) |
| Flip hour angle | ✅ Pass | Derived from :GTa# |
| Beyond the pole | ✅ Pass | :Gm# returns E/W/N |
| Pier side reporting | ✅ Pass | |
| Abort slew | ❌ Untested | |
| Sync position | ❌ Untested | |
| Open loop move (jog buttons) | ❌ Untested | |
| Pulse guide (autoguiding) | ❌ Untested | Implemented (DirectGuideInterface + `:Mgdnnnn#`), needs nighttime test |
| Unpark | ❌ Untested | Implemented (`:Spu#` override), needs on-mount test |
| Set custom park position | ❌ Untested | Implemented (`:Sp01#` override), needs on-mount test |
| Tracking rate changes (lunar/solar) | ❌ Untested | |
| WiFi connection | ❌ Untested | |
| Disconnect/reconnect | ❌ Untested | |
| Settings dialog | ❌ Untested | Partially tested (Home button works) |

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

### ZWO Protocol Implementation Notes

The `FindHomeInterface` is fully implementable for the ZWO protocol. Using the `:hC#` command stops the mount at the zero position. You can check the state with `:GU#` (returns `H` in the bitmask if at home position) or via Park error code 5 (`PARK_NOT_GO_HOME`).
