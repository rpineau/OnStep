# ZWO Protocol Differences & Fix Plan

## Problem

The OnStep X2 mount plugin crashes when connecting to a ZWO mount. The crash occurs
in `OnStep::getLimits()` (OnStep.cpp:880) when the OnStep-proprietary `:GXEe#` command
times out and `std::stod("")` is called on the empty response string, throwing an uncaught
`std::invalid_argument` exception.

The same crash pattern exists in `OnStep::getflipHourAngle()` (OnStep.cpp:919, 928).

## Sources Consulted

- **ZWO Mount Serial Communication Protocol v2.1** (`~/Downloads/ZWO_Mount_communication_protocol_v2.1.pdf`)
- **TheSkyX X2 SDK** (`~/appinstall/X2-Examples/licensedinterfaces/mount/asymmetricalequatorialinterface.h`)
- **X2 Example Mount Plugin** (`~/appinstall/X2-Examples/mountplugins/x2mount/`)
- **OnStep plugin source** (`OnStep.cpp`, `x2mount.cpp`, `x2mount.h`)
- **Debug log output** (`~/OnStepLog.txt`, captured 2026-03-17)

## Protocol Comparison: Limit Commands

| Feature | OnStep Command | ZWO Equivalent | Notes |
|---|---|---|---|
| East hour angle limit | `:GXEe#` -> degrees | **None** | OnStep-proprietary `:GXE` extended command |
| West hour angle limit | `:GXEw#` -> degrees | **None** | OnStep-proprietary `:GXE` extended command |
| Flip hour angle (east) | `:GXE9#` -> degrees | **None** | OnStep-proprietary |
| Flip hour angle (west) | `:GXEA#` -> degrees | **None** | OnStep-proprietary |
| Meridian crossing behavior | N/A | `:GTa#` -> `nnsnn#` | Angle 0-15 deg past meridian |
| Altitude upper limit | N/A | `:GLH#` -> `nn#` | 60-90 degrees |
| Altitude lower limit | N/A | `:GLL#` -> `nn#` | 0-30 degrees |
| Pier side | `:Gm#` -> E/W/N | `:Gm#` -> E/W/N | **Compatible** |

### ZWO `:GTa#` Response Format

Returns `nnsnn#` where:
- Digit 1: Meridian flip enabled (1=yes, 0=no) — temporarily not supported by ZWO
- Digit 2: Continue tracking after meridian (1=track, 0=stop)
- Digits 3-5: `snn` = limit angle past meridian (0-15 degrees, negative = stop before meridian)

This is the closest ZWO equivalent to OnStep's hour angle limits. The angle can be
converted to hours via `degrees / 15.0`.

## X2 SDK Interface Behavior

The `AsymmetricalEquatorialInterface` (which our plugin implements) provides these
methods to TheSkyX:

```
gemLimits(east, west)  — East/west hour angle limits for the GEM
flipHourAngle()        — Hour angle at which the mount auto-flips
beyondThePole(bYes)    — Is the OTA west of the pier?
knowsBeyondThePole()   — Can the mount report pier side?
```

**SDK defaults** (from asymmetricalequatorialinterface.h): `gemLimits` defaults to 0/0 (most
restrictive — cannot pass meridian at all). The X2 example plugin does not implement this
interface at all, avoiding the problem entirely.

Our plugin registers `AsymmetricalEquatorialInterface` in `queryAbstraction`, so TheSkyX
**will** call `gemLimits` and `flipHourAngle` after every connection.

## Bugs Found (3 total)

### Bug 1 — Crash in `getLimits()` (OnStep.cpp:872-890)

The error check logs but does not return. Execution falls through to `std::stod("")` on an
empty response string, throwing an uncaught exception.

```cpp
nErr = sendCommand(":GXEe#", sResp);  // fails with error 6, sResp = ""
if(nErr) {
    // logs error but DOES NOT RETURN
}
dHoursEast = std::stod(sResp)/15.0;   // std::stod("") throws std::invalid_argument
```

Same pattern for `:GXEw#` on line 890.

### Bug 2 — Crash in `getflipHourAngle()` (OnStep.cpp:912-928)

Identical fall-through pattern for `:GXE9#` and `:GXEA#`.

### Bug 3 — `gemLimits()` ignores error (x2mount.cpp:888)

Always returns `SB_OK` regardless of whether `getLimits()` failed:

```cpp
nErr = m_OnStep.getLimits(dHoursEast, dHoursWest);
return SB_OK;  // should be: return nErr;
```

## Proposed Fix

### Strategy: Try OnStep commands first, fall back to ZWO, then defaults

No explicit mount-type detection needed. The fallback chain handles both protocols
through the same code path.

### `getLimits()` changes

1. Try `:GXEe#` — if it succeeds, parse normally (OnStep path)
2. If `:GXEe#` fails, try ZWO's `:GTa#` to read meridian crossing angle
   - Parse digits 3-5 (`snn`) as the west limit in degrees
   - Convert to hours: `degrees / 15.0`
   - Mirror for east limit (ZWO has no separate east limit)
3. If `:GTa#` also fails, set reasonable defaults (e.g. 6.0 hours east/west)
4. Return `PLUGIN_OK` with the derived or default values
5. Wrap all `std::stod` calls in try/catch as a safety net

### `getflipHourAngle()` changes

1. Try `:GXE9#` / `:GXEA#` — if they succeed, parse normally (OnStep path)
2. If they fail, try ZWO's `:GTa#` to read meridian crossing angle
   - Use the `snn` angle (digits 3-5) converted to hours as the flip hour angle
3. If `:GTa#` also fails, return 0.0 (flip at meridian — the SDK default)
4. Wrap all `std::stod` calls in try/catch

### `gemLimits()` in x2mount.cpp

Change `return SB_OK;` to `return nErr;`

### What this achieves

- **OnStep mounts**: OnStep commands succeed on first try; no behavior change
- **ZWO mounts**: Get real limits derived from the meridian crossing configuration
- **Unknown mounts**: Get safe defaults; no crash under any circumstances
