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
| DriverSlewsToParkPositionInterface | ✅ | ✅ | ❌ | ✅ | ✅ OnStep / ❌ ZWO |
| PulseGuideInterface2 | ✅ | ❌ | ✅ | ❌ | ✅ |
| NeedsRefractionInterface | ✅ | ✅ | ✅ | ✅ | ✅ |
| FindHomeInterface | ❌ | ❌ | ❌ | ❌ | ✅ ZWO only |
| MotorStatusInterface | ❌ | ❌ | ❌ | ❌ | ✅ ZWO only |

We implement every interface that any comparison plugin implements. `FindHomeInterface` and `MotorStatusInterface` are undocumented TSX interfaces reverse-engineered from the binary; implemented for ZWO homing support. `DriverSlewsToParkPositionInterface` is returned for OnStep (driver owns the park slew) but not ZWO (TSX owns the slew via `SlewToInterface`).

## ZWO Feature Implementation Status

| Feature | ZWO Protocol Cmd | Implemented | ZWO Override | Tested | Result |
|---------|-----------------|-------------|--------------|--------|--------|
| **Connection** | | | | | |
| Serial connect (9600 baud) | — | ✅ | ✅ | ✅ | Pass |
| WiFi connect (192.168.4.1:4030) | — | ✅ (via TSX) | — | ❌ | — |
| Set lat/long on connect | `:SMGE` | ✅ | ✅ | ✅ | Pass |
| Set date/time/tz on connect | `:SMTI` | ✅ | ✅ | ✅ | Pass |
| Auto-home on connect | `:hC#` | ❌ removed | — | — | Removed: mount must not move on connect. Homing via TSX Startup → Find Home. |
| Vendor detection (mountlist) | `pszDriverSelection` | ✅ | — | ✅ | Pass |
| **Coordinates** | | | | | |
| Get RA/DEC | `:GR#` `:GD#` | ✅ | ❌ (base) | ✅ | Pass |
| Get Alt/Az | `:GA#` `:GZ#` | ✅ | ❌ (base) | ❌ | — |
| Get pier side | `:Gm#` | ✅ | ❌ (base) | ✅ | Pass |
| **Slewing** | | | | | |
| Set target RA/DEC | `:Sr` `:Sd` | ✅ | ❌ (base) | ✅ | Pass |
| GOTO (RA/DEC) | `:MS#` | ✅ | ❌ (base†) | ✅ | Pass |
| GOTO (Alt/Az) | `:MA#` | ⚠️ | ❌ (base†) | — | `:MA#` does not exist in ZWO protocol v2.1. `gotoPark()` override avoids this path. |
| Abort | `:Q#` | ✅ | ❌ (base) | ✅ | Pass |
| Slew complete check | `:GU#` | ✅ | ❌ (base) | ✅ | Pass |
| **Tracking** | | | | | |
| Start tracking | `:Te#` | ✅ | ❌ (base) | ✅ | Pass |
| Stop tracking | `:Td#` | ✅ | ❌ (base) | ✅ | Pass |
| Sidereal rate | `:TQ#` | ✅ | ❌ (base) | ❌ | — |
| Lunar rate | `:TL#` | ✅ | ❌ (base) | ❌ | — |
| Solar rate | `:TS#` | ✅ | ❌ (base) | ❌ | — |
| Get tracking rate | `:GT#` | ✅ | ❌ (base) | ❌ | — |
| Get tracking status | `:GAT#` | ✅ | ✅ | ✅ | Pass. ZWO override uses `:GAT#`; `0`=off, non-zero=on. |
| **Open Loop Move** | | | | | |
| Set speed | `:Rn#` | ✅ | ❌ (base) | ✅ | Pass |
| Move N/S/E/W | `:Mn#` `:Ms#` `:Me#` `:Mw#` | ✅ | ❌ (base) | ✅ | Pass |
| Stop N/S/E/W | `:Qn#` `:Qs#` `:Qe#` `:Qw#` | ✅ | ❌ (base) | ✅ | Pass. Diagonal moves fixed: base now tracks active dirs as a bitmask (`m_nOpenLoopDirMask`); all active axes stopped on `endOpenLoopMove`. |
| **Guiding** | | | | | |
| Pulse guide (via OLM) | `:Me#`/`:Mw#` etc | ✅ | ❌ (base) | ❌ | Needs nighttime |
| Native pulse guide | `:Mgdnnnn#` | ✅ | ✅ | ❌ | Via DirectGuideInterface; needs nighttime |
| Set guide rate | `:Rg0.nn#` | ✅ | ✅ | ❌ | `setGuideRate()` sends `:Rg%.2f#`. UI spinner clamped 0.10–0.90 per spec. |
| Get guide rate | `:Ggr#` | ✅ | ✅ | ❌ | Synced from mount at connect into `X2Mount::m_dZWOGuideRate`. |
| **Sync** | | | | | |
| Sync position | `:CM#` | ✅ | ❌ (base) | ❌ | — |
| **Homing** | | | | | |
| Go to zero position | `:hC#` | ✅ | ✅ | ✅ | Pass |
| Check homing done | `:GU#` (H flag) | ✅ | ❌ (base) | ✅ | Pass |
| Query homing success | `:Gh#` | ✅ | ✅ | ✅ | Returns 0 even after homing (firmware quirk); `m_bHasBeenHomed` used as fallback. |
| **Parking** | | | | | |
| Park (goto + finalize) | `:Sp01#` `:hP#` | ✅ | ✅ | ✅ | Pass. TSX slews mount to park position (SlewToInterface), then calls `startPark` → `gotoPark`. `gotoPark` sends `:Sp01#` (register current position as park 1) + `:hP#`. `isParkingComplete` returns true immediately. |
| Park status (isParked) | `m_bZWOParked` | ✅ | ✅ | ✅ | Pass. `:Gps#` always returns empty on this firmware; `:GU#` never sets 'P'. `getAtPark()` falls back to `m_bZWOParked`, set by `finalizepark()`, cleared by `gotoPark()`/`unPark()`. |
| Set custom park position | `:Sp01#` | ✅ | ✅ | ✅ | Auto-sent in `gotoPark()`. Also callable via "Set Current Position as Park" button. |
| Unpark | `:Spu#` | ✅ | ✅ | ❌ | ZWO override sends `:Spu#`. Clears `m_bZWOParked`. Needs on-mount test. |
| **Meridian Limits** | | | | | |
| Get limits | `:GTa#` | ✅ | ✅ | ✅ | Pass |
| Set meridian behavior | `:STannsnn#` | ✅ | ✅ | ✅ | Pass. Was broken: sent `:STA…` (uppercase) — fixed to `:STa…`. |
| Get flip hour angle | `:GTa#` | ✅ | ✅ | ✅ | Pass |
| Beyond the pole | `:Gm#` | ✅ | ❌ (base) | ✅ | Pass |
| **Height Limits** | | | | | |
| Enable/disable | `:SLE#` `:SLD#` | ✅ | ✅ | ✅ | Pass. Enabled state persisted in INI. |
| Set upper/lower limit | `:SLHnn#` `:SLLnn#` | ✅ | ✅ | ✅ | Pass. Was broken: `:SLL` sent with leading `+` — fixed to unsigned `%02d`. |
| Get limits | `:GLH#` `:GLL#` | ✅ | ✅ | ✅ | Pass. Read from mount when dialog opens. |
| **Alignment** | | | | | |
| isAligned (homed check) | `:Gh#` + internal flag | ✅ | ✅ | ✅ | Pass |
| **Site Data** | | | | | |
| Set longitude | `:Sg` | ✅ | ❌ (base) | ✅ | Pass |
| Set latitude | `:St` | ✅ | ❌ (base) | ✅ | Pass |
| Set timezone | `:SG` | ✅ | ✅ (via SMTI) | ✅ | Pass |
| Set date | `:SC` | ✅ | ✅ (via SMTI) | ✅ | Pass |
| Set time | `:SL` | ✅ | ✅ (via SMTI) | ✅ | Pass |
| Daylight saving | `:GH#` `:SHn#` | ❌ | — | ❌ | SMTI handles timezone offset; DST may need explicit handling. |
| **Device Info** | | | | | |
| Firmware version | `:GVN#` | ✅ | ❌ (base) | ✅ | Pass |
| Device name | `:GVP#` | ✅ | ❌ (base) | ✅ | Pass |
| **Mode** | | | | | |
| EQ/AZ mode switch | `:AP#` `:AA#` | ❌ | — | — | Not needed (always EQ mode) |

† Base class `slewTargetRaDecEpochNow` / `slewTargetAltAszEpochNow` modified to handle ZWO `e` + error code response format and error code 7.

## Priority TODO Items

### Nice to Have
- [ ] **Unpark**: Test `:Spu#` on mount.
- [ ] **Pulse guide / autoguiding**: Test DirectGuideInterface + `:Mgdnnnn#` at night.
- [ ] **Daylight saving** (`:GH#`/`:SHn#`): SMTI handles timezone; DST handling may need verification.
- [ ] **WiFi connection**: Test over WiFi (192.168.4.1:4030).

### Not Needed
- EQ/AZ mode switch — always equatorial mode
- Bluetooth commands — not applicable to X2 serial/TCP
- Clear multi-star calibration (`:NSC#`) — no use case identified

## Known Limitations

### TSX "Set Park Position" / "Clear Park Position" Menu Items

**Standard OnStep:** menu items disabled (driver returns `DriverSlewsToParkPositionInterface`; TSX suppresses them). Behavior unchanged.

**ZWO:** menu items enabled. TSX converts its stored AltAz park position to RA/Dec via `HzToEq` and slews via `SlewToInterface`. When the slew completes, TSX calls `startPark` → `isCompletePark` → `endPark`. `gotoPark()` sends `:Sp01#` + `:hP#` to finalize park state in the firmware.

**Height limit constraint:** Park position altitude must be above the firmware's lower height limit (`:SLL#`). The firmware rejects any slew below this limit with error `e6` ("Target under height limitation"). Set park position within the reachable operating zone.

### `:Gps#` Unreliable on ZWO Firmware

`:Gps#` always returns an empty response (`#` with no data) regardless of park state, contrary to ZWO spec v2.1. `isParkingComplete()` returns true immediately rather than polling it. `getAtPark()` uses `m_bZWOParked` as a persistent fallback. `:GU#` also never sets the 'P' (parked) flag on ZWO.

### Diagonal Open Loop Move (Both Axes)

TSX calls `startOpenLoopMove` once per axis for diagonal moves. Fixed in base class: `m_nOpenLoopDirMask` (bitmask) replaces the previous single-direction scalar, so `stopOpenLoopMove` stops all active axes. Affects both OnStep and ZWO.
