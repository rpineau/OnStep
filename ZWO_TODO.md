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
| Set guide rate | `:Rg0.nn#` | ✅ | ✅ | ❌ | `setGuideRate()` sends `:Rg%.2f#`. Called on settings dialog save when connected. **Spec confirmed: mount honors this.** UI spinner clamped 0.10–0.90 matches spec range. |
| Get guide rate | `:Ggr#` | ✅ | ✅ | ❌ | `getGuideRate()` called at end of `Connect()`; result synced into `X2Mount::m_dZWOGuideRate` so DirectGuide math and settings dialog both reflect actual mount state. **Spec confirmed.** |
| **Sync** | | | | | |
| Sync position | `:CM#` | ✅ | ❌ (base) | ❌ | — |
| Clear multi-star cal | `:NSC#` | ❌ | — | ❌ | — |
| **Homing** | | | | | |
| Go to zero position | `:hC#` | ✅ | ✅ | ✅ | Pass |
| Check homing done | `:GU#` (H flag) | ✅ | ❌ (base) | ✅ | Pass |
| Query homing success | `:Gh#` | ✅ | ✅ | ✅ | Returns 0 even after homing (firmware issue?) |
| **Parking** | | | | | |
| Park (goto + finalize) | `:Sp01#` `:hP#` | ✅ | ✅ | 🔄 | TSX slews mount to park position (SlewToInterface), then calls startPark → gotoPark. gotoPark sends :Sp01# (register current position as park 1) + :hP# (execute park). isParkingComplete returns true immediately (mount already there). |
| Park status (isParkingComplete) | `:Gps#` | ⚠️ | ✅ | 🔄 | `:Gps#` always returns empty `#` on this firmware. isParkingComplete() returns true immediately instead of polling. |
| Park status (isParked/getAtPark) | `:Gps#` → `m_bZWOParked` | ✅ | ✅ | 🔄 | `:Gps#` unreliable (always empty). getAtPark() falls back to m_bZWOParked, set by finalizepark()/endPark and cleared by gotoPark()/unPark(). getStatus()/:GU# never sets 'P' on ZWO. |
| Set custom park position | `:Sp01#` | ✅ | ✅ | 🔄 | Auto-sent in gotoPark() before :hP#. Also callable via "Set Current Position as Park" button. |
| Unpark | `:Spu#` | ✅ | ✅ | ❌ | ZWO override sends `:Spu#` instead of `:hR#`. Clears m_bZWOParked. |
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
| Park (goto + finalize) | 🔄 Testing | gotoPark sends :Sp01# + :hP#; isParkingComplete returns true immediately; finalizepark sets m_bZWOParked |
| Start/stop tracking | ✅ Pass | :Te# / :Td# |
| Tracking status readback | ❌ Untested | Implemented via :GAT# override; needs on-mount confirmation of response format |
| isParked() after park | 🔄 Testing | getAtPark() falls back to m_bZWOParked (set by finalizepark); :Gps# always returns empty on this firmware |
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

## TSX "Set Park Position" / "Clear Park Position" — No Driver Interface

TSX has no `SetParkPositionInterface` — it stores park coordinates internally and passes them to `startPark(dAz, dAlt)`. Since ZWO has no AltAz GOTO (`:MA#` doesn't exist), our `gotoPark()` override sends `:hP#` instead, ignoring TSX's coordinates.

**ZWO workaround**: Use the "Set Current Position as Park" button in the settings dialog (ZWO Advanced section). This sends `:Sp01#` to store the current position in the mount. Subsequent `:hP#` park commands return there.

---

## Known Limitations

### TSX "Set Park Position" / "Clear Park Position" Menu Items

**Standard OnStep:** disabled (driver returns `DriverSlewsToParkPositionInterface`; intentional, behavior unchanged).

**ZWO:** enabled. TSX converts its stored AltAz park position to RA/Dec using its own `HzToEq` and calls `startSlewTo`. When the slew completes, TSX calls `startPark` → `isCompletePark` → `endPark` (ParkInterface). `gotoPark()` sends `:Sp01#` + `:hP#` to finalize the park state in the firmware. See TODO-4.

**Height limit constraint:** The park position altitude must be above the firmware's lower height limit (`:SLL#`). The firmware rejects any slew to a position below this limit with error `e6`. Set the park position somewhere in the reachable operating zone.

---

## Strategic TODOs

### TODO-2: Internal State Storage Audit (Homed/Parked/etc.)

**Status:** ✅ Resolved

Audit found 1 dead field (`m_bSyncDone`), 1 connect-path divergence risk (`ZWOMount::Connect()` doesn't call `OnStep::Connect()`), 1 init gap (`m_bIsParked` not explicitly set in ZWO connect path), and 1 low-priority design note (`m_bLinked` vs `m_bIsConnected` two-layer split). All other state fields are correct.

**Issue 1 — `m_bIsAtHome` vs `m_bHasBeenHomed` (N/A — plan was wrong).** Pre-analysis incorrectly concluded these are equivalent. In fact, `m_bIsAtHome` is reset to `false` at the start of every `getStatus()` call, then conditionally set from the `:GU#` 'H' flag — it reflects current position. `m_bHasBeenHomed` is sticky and never cleared. Both are needed. No change.

**Issue 2 — `m_bSyncDone` removed.** ✅ Field removed from `OnStep.h`; assignments removed from `OnStep::Connect()`, `OnStep::Disconnect()`, `syncTo()`, and `ZWOMount::Connect()`. The only consumer (a tracking-start gate in `syncTo()`) was already commented out.

**Issue 3 — `m_bIsParked` init gap fixed.** ✅ Added `getAtPark()` call at end of `ZWOMount::Connect()`. `getStatus()` via `isHomingDone()` sets `m_bIsParked` from `:GU#`, but ZWO firmware doesn't set the P flag there — `getAtPark()` re-queries via `:Gps#` and syncs `m_bIsParked` as a side-effect.

**Issue 4 — `ZWOMount::Connect()` divergence documented.** ✅ Added a comment to both `OnStep::Connect()` and `ZWOMount::Connect()` explaining why ZWO doesn't call the base (base would call `setSiteData()` with `:Sg`/`:St`/`:SG` which ZWO doesn't support; ZWO uses SMGE+SMTI instead) and flagging that future base init must be replicated manually.

**Issue 5 — Two-layer design documented.** ✅ Added comment block in `x2mount.h` near `m_bLinked` explaining the intentional separation from `m_bIsConnected` and the known divergence scenario on mid-session serial failure.

### TODO-4: Fix ZWO Park Completion (Option B — TSX-Driven Slew)

**Status:** ✅ Implemented (🔄 Testing in progress)

#### What was discovered during live testing

**TSX park flow (confirmed):** When `DriverSlewsToParkPositionInterface` is null, TSX uses *both* `SlewToInterface` and `ParkInterface`. It calls `startSlewTo(parkRa, parkDec)` to physically move the mount, polls `isSlewToComplete`, then calls `startPark` → `isCompletePark` → `endPark`. `startPark` is always called — `DriverSlewsToParkPositionInterface` only controls whether the driver or TSX performs the slew.

**`:Gps#` is broken on this firmware:** Always returns `#` with no data (empty response). Cannot be used to detect park completion or parked state. Originally assumed to work per ZWO spec v2.1.

**`:GU#` 'P' flag not set by ZWO:** The ZWO AMx firmware does not set the 'P' (parked) flag in `:GU#` responses. `getStatus()` always sees `m_bIsParked = false`. Since `getStatus()` is called on every poll cycle, any `m_bIsParked = true` set elsewhere gets immediately clobbered.

**`e6` root cause:** Park position altitude was below the firmware's lower height limit. Error 6 = "Target under height limitation" (spec §Error code statement). Not a coordinate reversal.

#### What was implemented

**`gotoPark()`:** Sends `:Sp01#` (register current position as custom park 1) then `:hP#` (execute park). At call time, TSX has already slewed the mount to the park position, so both commands complete immediately.

**`isParkingComplete()`:** Returns `bComplete=true` on first call. Polling `:Gps#` or `:GU#` both fail on ZWO firmware. Since the mount is already at the park position when `gotoPark()` is called, park is instantaneous.

**`finalizepark()`:** Called by `X2Mount::endPark()` when TSX acknowledges completion. Sets `m_bZWOParked = true`.

**`m_bZWOParked` (ZWOMount private):** Persistent parked flag unaffected by `getStatus()`. Set by `finalizepark()`, cleared by `gotoPark()` and `unPark()`. `getAtPark()` falls back to this when `:Gps#` returns empty.

#### Protocol notes (spec v2.1)

- `:Sp01#` response: `1`=success, or park error code. Error **5** = `PARK_NOT_GO_HOME` (homing not done). Error **9** = `PARK_MOVING` (mount moving). Requires equatorial mode. In practice, returns empty on this firmware — treated as "accepted."
- `:hP#` response: **None** (fire-and-forget). Only valid in equatorial mode.
- `:Gps#`: Spec says valid after a park command. In practice always returns empty `#` — completely unreliable on this firmware.
- `:Spu#` (unpark): `0`=failed, `1`=success. Already implemented in `ZWOMount::unPark`.

#### Known limitation — park position must respect height limit

The ZWO firmware enforces a lower altitude limit (`:SLL#`) on all slews, including TSX-driven park slews. If the stored park position is below this limit, the firmware rejects the `:MS#` with **e6** = "Target under height limitation." The user must set the park position at an altitude above the configured floor. There is no workaround in the driver — this is by design in the firmware.

---

### TODO-3: Re-enable TSX "Set/Clear Park Position" for ZWO via AltAz→RA/Dec Conversion

**Status:** ✅ Implemented (TSX-managed park enabled; see TODO-4 for completion fix)

**Standard OnStep behavior: unchanged.** `DriverSlewsToParkPositionInterface` stays registered for OnStep. TSX menu stays disabled for OnStep. Nothing in this TODO touches standard OnStep behavior.

**Goal:** Re-enable TSX's "Set Park Position" and "Clear Park Position" Shutdown menu items for ZWO, making TSX the owner of park position storage for ZWO the same as for any other mount.

**The cheat:** ZWO has no AltAz GOTO (`:MA#` doesn't exist), but it does have RA/Dec GOTO (`:MS#`). TSX facade provides `HzToEq(dAz, dAlt, &dRa, &dDec)` — an exact AltAz-to-equatorial conversion using the site location and current LST. We convert TSX's stored park position to RA/Dec at park time and issue a standard RA/Dec GOTO. The mount physically ends up at the right AltAz; TSX is managing what position that is.

#### Changes required (all in `x2mount.cpp`)

**1 — Don't return `DriverSlewsToParkPositionInterface` for ZWO.**
```cpp
// was: unconditional
else if (!strcmp(pszName, DriverSlewsToParkPositionInterface_Name))
    *ppVal = dynamic_cast<DriverSlewsToParkPositionInterface*>(this);

// becomes: OnStep only (same pattern as FindHomeInterface / MotorStatusInterface)
else if (!strcmp(pszName, DriverSlewsToParkPositionInterface_Name) && !m_bIsZWOMount)
    *ppVal = dynamic_cast<DriverSlewsToParkPositionInterface*>(this);
```

**2 — `startPark(dAz, dAlt)`: use `HzToEq` + RA/Dec GOTO for ZWO.**
TSX's `HzToEq` is available as `m_pTheSkyXForMounts->HzToEq(dAz, dAlt, dRa, dDec)`.
```cpp
int X2Mount::startPark(const double& dAz, const double& dAlt)
{
    if(!m_bLinked) return ERR_NOLINK;
    X2MutexLocker ml(GetMutex());

    if(m_bIsZWOMount) {
        double dRa, dDec;
        if(m_pTheSkyXForMounts->HzToEq(dAz, dAlt, dRa, dDec) == SB_OK) {
            m_bParked = false;
            return m_pMount->startSlewTo(dRa, dDec);
        }
        // HzToEq failed (e.g. below horizon) — fall through to :hP# as before
    }
    return m_pMount->gotoPark();
}
```
`m_bParked` is cleared here so `isCompleteUnpark` works correctly on a subsequent unpark.

**3 — `isCompletePark`: poll slew completion for ZWO RA/Dec park path.**
Currently polls `isParkingComplete()` which uses `:Gps#`. After a RA/Dec GOTO park, `:Gps#` will never return `2` (we never sent `:hP#`), so completion detection would be broken. Add slew-based polling for ZWO.

A clean way: add a flag `m_bZWOParkViaGoto` (set in step 2 when `HzToEq` succeeds), then in `isCompletePark`:
```cpp
if(m_bIsZWOMount && m_bZWOParkViaGoto) {
    nErr = m_pMount->isSlewToComplete(bComplete);
    if(bComplete) {
        m_bParked = true;
        m_bZWOParkViaGoto = false;
        m_pMount->setTrackingRates(false, true, 0.0, 0.0); // stop tracking
    }
    return nErr;
}
// else fall through to existing isParkingComplete() path (:Gps#)
```

**4 — Optional: teach the mount its park position as a side effect.**
When the RA/Dec GOTO completes and the mount is physically at the TSX-stored park position, optionally call `:Sp01#` to record this as the mount's stored park position. This means future `:hP#` calls (e.g., from a power cycle) will also return to the correct position.
This step is low-risk but not strictly required for the feature to work.

#### Effect on the ZWO settings dialog "Set Current Position as Park" button
That button (`:Sp01#`) still works and remains the way to set the mount-firmware park position for `:hP#`-based parking (e.g., if TSX coordinates are unavailable). The two mechanisms coexist independently.

#### Testing
1. Set park position in TSX Shutdown menu ("Set Park Position") while scope is at desired position.
2. Slew scope elsewhere. Click "Park". Scope should return to the set position.
3. "Clear Park Position" in TSX menu should clear it; subsequent Park should use the mount's own stored position (fallback `:hP#` path).
4. ZWO Advanced settings "Set Current Position as Park" button should continue to work independently.
