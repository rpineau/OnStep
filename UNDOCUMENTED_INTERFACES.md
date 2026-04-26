# Undocumented X2 Interfaces in TheSkyX

This document provides a reference for undocumented X2 interfaces discovered by reverse-engineering
TheSkyX (TSX) binaries and plugins. It covers interface contracts, vtable layouts, TSX internal
function behaviour, known bugs and their fixes, and investigation methodology.

---

## Background

The official X2-Examples SDK doesn't document several advanced mount features. We wanted to add
support for DirectGuide and Find Home to our OnStep plugin. Since the SDK was silent on these, we
examined the TSX binary directly.

### Discovery approach

1. Search the X2-Examples directory: `grep -rnIE "DirectGuide|Startup|Home|FindHome"`.
   Found `dapiFindHome` for Domes and `ERR_MOUNTNOTHOMED` (231), but nothing for mount homing.
2. Examine the TheSkyX binary and bundled mount plugins. On Linux:
   `~/TheSkyX/TheSkyX` and `~/TheSkyX/Resources/Common/PlugIns64/MountPlugIns/*.so`.
3. `strings ~/TheSkyX/TheSkyX | c++filt` — reveals vtables and virtual method declarations for
   classes not in the SDK. Found clear signatures for `FindHomeInterface` and `DirectGuideInterface`.
4. Examine built-in plugins (e.g. `LX200FindHome`) to see what methods they implement alongside
   the documented ones. `LX200FindHome::motorStatus(unsigned short&, unsigned short&)` proved
   `motorStatus` belongs to the find home implementation.
5. Disassemble the binary for deeper analysis:
   `objdump -d ~/TheSkyX/TheSkyX > tsx_disasm.txt` (7.5M lines, ARM64 ELF).
   Use `awk` to extract regions; use `dump_vtable.py` to read vtable slots from the live binary.
6. Correlate runtime behaviour with disassembly: add `log()` calls to the plugin, identify TSX
   function addresses from call patterns (e.g. two `queryAbstraction` calls → `updateHomeStatus`),
   then trace ARM64 control flow to find discrepancies.

---

## Discovered Interfaces

### DirectGuideInterface

DirectGuide is Software Bisque's high-resolution guiding method. It sends offsets directly to the
mount and is separate from the standard `PulseGuideInterface2`.

```cpp
class DirectGuideInterface {
public:
    virtual ~DirectGuideInterface() {}
    virtual bool directGuideAsynchronous() = 0;
    virtual int setDirectGuideAsynchronous(bool bAsync) = 0;
    virtual int directGuideMoveTelescope(const double& dRA, const double& dDec) = 0;
    virtual int directGuideAbort() = 0;
};
```

vtable layout (ARM64, 8-byte slots, vptr base = first function pointer):

| Slot | Offset | Method |
|------|--------|--------|
| 0 | +0x00 | deleting destructor |
| 1 | +0x08 | complete object destructor |
| 2 | +0x10 | `directGuideAsynchronous()` |
| 3 | +0x18 | `setDirectGuideAsynchronous(bool)` |
| 4 | +0x20 | `directGuideMoveTelescope(const double&, const double&)` |
| 5 | +0x28 | `directGuideAbort()` |

Verified by tracing `DlgWorkbench::seriesJogMount` (0x48f7d4) and `DirectGuide()` (0x598848)
in the TSX ARM64 binary:
- vptr+0x18 called with `(this, bool)` → `setDirectGuideAsynchronous`
- vptr+0x20 called with `(this, &double, &double)` → `directGuideMoveTelescope`; return value checked as int error code
- vptr+0x28 called with `(this)`, return ignored → `directGuideAbort` (only when all guide params are zero)

**IMPORTANT:** Declaration order in the C++ class determines vtable slot order. The method order
above must be preserved exactly. Getting this wrong causes TSX to call the wrong methods —
e.g. slot 3 being `directGuideAbort` instead of `setDirectGuideAsynchronous` causes TSX to
abort any pending move instead of setting async mode, and then get error code 1 back from
`directGuideAsynchronous` when it tries to call `directGuideMoveTelescope`.

### FindHomeInterface

Enables the "Startup → Find Home" menu option in TSX.

```cpp
class FindHomeInterface {
public:
    virtual ~FindHomeInterface() {}
    virtual int startFindHome() = 0;
    virtual int isCompleteFindHome(bool& bComplete) const = 0;
    virtual int endFindHome() = 0;
    virtual int motorStatus(unsigned short& u1, unsigned short& u2) = 0;
};
```

**`motorStatus` is required.** Without it, TSX encounters a vtable mismatch and misroutes the call
intended for `startFindHome()`, resulting in `ERR_COMMANDNOTSUPPORTED` (Error 228).

vtable layout (ARM64, 8-byte slots, from TSX binary):

| Slot | Offset | Method |
|------|--------|--------|
| 0 | +0x00 | deleting destructor |
| 1 | +0x08 | complete object destructor |
| 2 | +0x10 | `startFindHome()` |
| 3 | +0x18 | `isCompleteFindHome(bool&) const` |
| 4 | +0x20 | `endFindHome()` |
| 5 | +0x28 | `motorStatus(unsigned short&, unsigned short&)` |

### MotorStatusInterface

A second independent interface queried by name `"MotorStatusInterface"`. Controls both the mount
state machine (via `motorStatus`) and the UI homed indicator (via `motorStatus2`).

```cpp
class MotorStatusInterface {
public:
    virtual ~MotorStatusInterface() {}
    virtual int motorStatus(unsigned short& u1, unsigned short& u2) = 0;
    virtual int motorStatus2(unsigned short& u1, unsigned short& u2) = 0;
};
```

vtable layout (ARM64, 8-byte slots, from TSX binary):

| Slot | Offset | Method |
|------|--------|--------|
| 0 | +0x00 | deleting destructor |
| 1 | +0x08 | complete object destructor |
| 2 | +0x10 | `motorStatus(unsigned short&, unsigned short&)` |
| 3 | +0x18 | `motorStatus2(unsigned short&, unsigned short&)` |

**`motorStatus` vs `motorStatus2` have completely different semantics** — see below.

---

## motorStatus / motorStatus2 Contracts

### motorStatus — drives the mount state machine

Called by `MountThread::poll1HomeAndJoysticking` (0x634c40) every poll cycle. Results are used by
`Mount::setMountState` (0x631450), which stores the state integer at `[Mount+88]`.

**Bit field reference (u1 and u2 are both `unsigned short`):**

| Value / Bit | Check | Mount state set | Meaning |
|-------------|-------|-----------------|---------|
| `== 0x0fa2` | u1 OR u2 | 5 | Motor position error (ERR_MKS_MOTOR_POSERRORLIM) |
| `bit 0x0100` | u1 AND u2 both set | 18 | Joystick/motor-active |
| `bit 0x1000` missing | u1 OR u2 clear | 7 | NOT_HOMED → `ERR_MOUNTNOTHOMED` (231) on slew |
| `bit 0x1000` set | u1 AND u2 both set | 0 | Ready (slews allowed) |
| `bit 0x2000` | u1 OR u2 | 14 | Unknown (secondary joystick?) |

Priority order (highest wins): `0xfa2` error → bit `0x100` → bit `0x1000` missing → bit `0x2000`.

`Mount::startSlewTo` (0x630cdc) returns `ERR_MOUNTNOTHOMED` (231) when state == 7.

**Implementation rule:** Return `u1 = 0x1000, u2 = 0x1000` when homed; `u1 = 0, u2 = 0` otherwise.
Must write u2 — both are checked. Do NOT return `0xfa2` unless there is an actual motor error.
Do NOT issue serial commands — called at poll rate.

**IMPORTANT — u2 write safety:** All known call sites for `motorStatus` pass a valid stack address
in x2. Writing u2 is safe. (The "do not write u2" restriction applies only to `motorStatus2` —
see Bug 5 below.)

### motorStatus2 — drives the UI homed indicator

Called by `DlgWorkbench::updateHomeStatus` (0x48bce0) on every periodic status poll and after
Find Home. TSX reads u1 with `ldrb` and checks the exact value:

```
if nErr==0 AND u1_byte==1: GREEN ("Mount is homed.", green stylesheet)
else:                       RED  ("Mount not homed!" or "Homing mount...")
```

**The comparison is `== 1`, not `!= 0`. Do NOT return `0x1000` here.**

Also called by `findHomeLoop` every ~100ms during homing: `u1 == 0xfa2` aborts homing.

**Implementation rule:** Return `u1 = 1` when homed; `u1 = 0` otherwise. Do NOT write u2 — at
the `updateHomeStatus` call site, x2 holds the vtable thunk address, not an output pointer.
Writing u2 corrupts the thunk's first instruction (Bug 5).

---

## TSX Internal Functions Reference

These addresses are from the ARM64 Linux build of TheSkyX.

### `MountThread::findHomeLoop()` at 0x63786c — async background thread

```
1. Mount state = 18
2. startFindHome()
3. Poll every ~100ms: isCompleteFindHome() + motorStatus() + motorStatus2()
   - If motorStatus or motorStatus2 u1 == 0xfa2: abort with ERR_MKS_MOTOR_POSERRORLIM
4. endFindHome()
5. emitAsyncOpComplete(opType=18)
```

### `MountThread::poll1HomeAndJoysticking()` at 0x634c40 — per-poll state machine

Stack layout (offsets within this function's frame):

| sp offset | Content | Width |
|-----------|---------|-------|
| sp+48 | motorStatus2 u2 | 16-bit |
| sp+50 | motorStatus2 u1 | 16-bit |
| sp+52 | motorStatus u2 | 16-bit |
| sp+54 | motorStatus u1 | 16-bit |
| sp+61 | flag: bit 0x1000 result | 8-bit |
| sp+62 | flag: bit 0x100 result | 8-bit |
| sp+63 | flag: 0xfa2 error | 8-bit |

### `DlgWorkbench::updateHomeStatus(bool bHoming)` at 0x48bce0

```
1. queryAbstraction("MotorStatusInterface") × 2 → DlgWorkbench+2384, +2376
2. Enable button if (DlgWorkbench+2384 != null) AND (DlgWorkbench+0x1720 != 0)
3. if DlgWorkbench+2384 != null:
       x2 = *(vptr + 0x18)          ← motorStatus2 function pointer
       x1 = sp + 0x3f               ← output byte for u1
       blr x2                        ← call motorStatus2(this, u1=*x1)
       if nErr==0 AND *(sp+63)==1: GREEN
       else:                          RED
```

### `DlgWorkbench::hardwareGetDynamicStatus()` at 0x48aea4 — periodic poll (~1 s)

```
1. DlgWorkbench+0x1720 = Mount::isLinked()
2. updateHomeStatus(DlgWorkbench+2392)    ← 2392=0 after homing → bHoming=false
3. ... further status ops including motorStatus call (not fully traced) ...
```

### `DlgWorkbench::hardwareFindHome()` at 0x48ba64 — "Find Home" button handler

```
1. updateHomeStatus(1)       ← sets "Homing mount..." (red)
2. Kick off async findHomeLoop
3. DlgWorkbench+2392 = 0
4. Call motorStatus2 (UI thread)
5. updateHomeStatus(0)       ← RED regardless of u1 value
```

### `Mount::setMountState()` at 0x631450

Stores state integer at `[Mount+88]`.

### `Mount::startSlewTo()` at 0x630cdc

Returns `ERR_MOUNTNOTHOMED` (231) when `[Mount+88] == 7`.

### `Mount::asyncOpComplete_slot` at 0x633368 — CONFIRMED NO-OP

Four instructions: save regs, nop, restore regs, ret.

---

## Implementation Guide

### queryAbstraction

```cpp
int X2Mount::queryAbstraction(const char* pszName, void** ppVal) {
    *ppVal = NULL;

    if (!strcmp(pszName, "DirectGuideInterface"))
        *ppVal = dynamic_cast<DirectGuideInterface*>(this);
    else if (!strcmp(pszName, "FindHomeInterface") && m_bIsZWOMount)
        *ppVal = static_cast<FindHomeInterface*>(this);
    else if (!strcmp(pszName, "MotorStatusInterface") && m_bIsZWOMount)
        *ppVal = static_cast<MotorStatusInterface*>(this);

    return SB_OK;
}
```

`FindHomeInterface` and `MotorStatusInterface` are gated behind `m_bIsZWOMount`. Standard OnStep
mounts do not expose these — without the gate, a non-homing mount would have `motorStatus` return
`0,0` permanently, locking mount state to 7 and blocking all slews.

Use `static_cast` for `FindHomeInterface` and `MotorStatusInterface` (required for correct thunk
generation in the vtable). `dynamic_cast` is fine for `DirectGuideInterface`.

---

## ZWO Protocol Implementation

### DirectGuideInterface

| Method | TSX contract | Our implementation |
|--------|-------------|-------------------|
| `directGuideMoveTelescope(dRA, dDec)` | Apply arcsecond offsets | Converts to pulse durations → `:Mgd{dir}{nnnn}#` |
| `directGuideAbort()` | Abort in-progress move | Calls `Abort()` → `:Q#` |
| `directGuideAsynchronous()` | True if async | Returns `true` |
| `setDirectGuideAsynchronous(bool)` | Toggle async mode | No-op; always async |

#### Arcsecond-to-millisecond conversion

```
sidereal_rate = 15.04106858 arcsec/sec
guide_rate    = m_dZWOGuideRate × sidereal_rate   (arcsec/sec)
duration_ms   = |offset_arcsec| / guide_rate × 1000
```

Sign → direction: RA+ = `e`, RA− = `w`, Dec+ = `n`, Dec− = `s`. Zero-duration pulses skipped.

#### Wire format: `:Mgd{dir}{nnnn}#`

`{nnnn}` is zero-padded 4-digit milliseconds (e.g. `:Mgde0250#` = 250 ms east). No response —
`sendCommand` called with `SHORT_RESPONSE, 0`.

#### Guide rate

`m_dZWOGuideRate` persisted in INI (default `0.5` = 50% sidereal ≈ 7.52 arcsec/sec), exposed as
spinbox in settings dialog. On connect, `ZWOMount::getGuideRate()` sends `:Ggr#` to read actual
hardware value and overwrites the cached rate. On settings save, `setGuideRate()` sends `:Rg{rate}#`.

### FindHomeInterface / MotorStatusInterface — Bug History

These bugs were found and fixed while getting Find Home working on the ZWO AM-series mount.

**Bug 1: `ZWOMount::homeMount()` was a no-op**
ZWO firmware always sets the `H` flag in `:GU#` at power-on even before a physical homing sweep.
The base `OnStep::homeMount()` had `if(m_bIsAtHome) return PLUGIN_OK;` which skipped `:hC#`.
**Fix:** `ZWOMount::homeMount()` unconditionally resets `m_bHasBeenHomed = false`, sends `:hC#`,
sleeps 500 ms.

**Bug 2: `isHomingDone()` returned true prematurely**
After sending `:hC#`, returned `bIsHomed=true` immediately because `m_bIsAtHome` was still set.
**Fix:** `ZWOMount::isHomingDone()` returns `bIsHomed=false` while `m_bIsHoming || m_bIsSlewing`.

**Bug 3: `motorStatus2` always returned u1=0**
**Fix:** `motorStatus2` returns `u1=1` when `m_bHasBeenHomed` is true (cached, no serial I/O).

**Bug 4: `isCompleteFindHome` returned false in background polls**
After `endFindHome()` sets `m_bFindHomeInitiated = false`, TSX background polls call
`isCompleteFindHome` without a prior `startFindHome`.
**Fix:** When `m_bFindHomeInitiated == false`, return `cachedHasBeenHomed()` immediately.

**Bug 5: `u2 = 0` in `motorStatus2` corrupted the vtable thunk** (2026-03-23)
TSX's `updateHomeStatus` call site passes only one output arg (x1 = sp+63 for u1). x2 holds the
`motorStatus2` **thunk address**, not a second output pointer. Our `u2 = 0` compiled to
`strh wzr, [x2]`, overwriting the first 2 bytes of the thunk. The thunk's first instruction
`sub x0, x0, #0xa0` (this-pointer adjustment of -160) became `sub x0, x0, #0x20` (-32). The
corruption is idempotent (stabilises after first call) and doesn't crash due to MAP_PRIVATE COW.
**How found:** Disassembled the vtable thunk at the vptr address after a live run; first instruction
differed from a cold binary.
**Fix:** Never write u2 in `motorStatus2`. The parameter is kept in the signature (required for
vtable ABI mangling) but is intentionally not written.

**Bug 6: `motorStatus` bit 0x1000 never set → ERR_MOUNTNOTHOMED (231) on every slew** (2026-03-24)
`poll1HomeAndJoysticking` checks bit 0x1000 in both u1 AND u2. Our `motorStatus` was returning
`u1=1` (bit 0) and never writing u2 — bit 0x1000 was never set, locking mount state to 7.
**How found:** Searched `tsx_disasm.txt` for `0xe7` (231), filtered Qt layout false positives,
traced to `Mount::startSlewTo` → `[Mount+88]==7` → `setMountState` callers →
`poll1HomeAndJoysticking` bit-test logic.
**Fix:** `motorStatus` returns `u1 = 0x1000, u2 = 0x1000` when homed; `0, 0` otherwise.

---

## Known Issue: UI Button Shows "Not Homed" After Homing

**Status:** Unresolved pending re-test after Bug 6 fix deployment.

`motorStatus2` correctly returns `u1=1` (→ green path per `updateHomeStatus` disassembly), yet the
button remains red. Hypothesis: mount state 7 (set by Bug 6) influences a separate display path not
traced in `hardwareGetDynamicStatus` step 3. Test after Bug 6 fix; if still red, trace the
`motorStatus` call in `hardwareGetDynamicStatus` beyond the `updateHomeStatus` call at 0x48aecc.

---

## Relevant Files

| File | Purpose |
|------|---------|
| `x2mount.h` | `FindHomeInterface`, `MotorStatusInterface` class definitions with full inline docs |
| `x2mount.cpp` | `startFindHome`, `isCompleteFindHome`, `endFindHome`, `motorStatus`, `motorStatus2` |
| `ZWOMount.cpp` | `homeMount()`, `isHomingDone()` ZWO overrides |
| `OnStep.cpp` | Base `homeMount()`, `isHomingDone()`, `getStatus()` |
| `~/TheSkyX/TheSkyX` | TSX binary (ARM64 ELF) |
| `tsx_disasm.txt` | Full TSX disassembly (7.5M lines, `objdump -d`) |
| `dump_vtable.py` | Reads vtable slot pointers from the live binary |
| `~/OnStepLog.txt` | Runtime plugin log |
