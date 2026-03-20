# Undocumented X2 Interfaces in TheSkyX

This document provides a reference for undocumented X2 interfaces discovered by reverse-engineering TheSkyX (TSX) binaries and plugins. These interfaces allow custom mount plugins to support advanced features like DirectGuide.

## Background

The official X2-Examples SDK doesn't document several advanced mount features. We wanted to add support for DirectGuide to our OnStep plugin. Since the SDK was silent on these, we looked into the TSX binary itself to find how it communicates with Software Bisque's own mounts.

## Investigation Chain

You can reproduce or expand this work by following these steps.

1. Search the X2-Examples directory for keywords. We used `grep -rnIE "DirectGuide|Startup|Home|FindHome"`. This found `dapiFindHome` for Domes and the error code `ERR_MOUNTNOTHOMED` (231), but nothing for mount homing.
2. Examine the TheSkyX binary and bundled mount plugins. On Linux, these are at `~/TheSkyX/TheSkyX` and `~/TheSkyX/Resources/Common/PlugIns64/MountPlugIns/*.so`.
3. Use `strings` and `c++filt` to identify demangled C++ symbols. Running `strings ~/TheSkyX/TheSkyX | c++filt` reveals vtables and virtual method declarations for classes that aren't in the SDK.
4. Identify specific interface structures. We found clear signatures for `FindHomeInterface` and `DirectGuideInterface` within the binary's symbol table.

## Discovered Interfaces

We reconstructed the following interface from the demangled symbols.

### DirectGuideInterface

DirectGuide is Software Bisque's high-resolution guiding method. It sends offsets directly to the mount and is separate from the standard `PulseGuideInterface2`.

```cpp
class DirectGuideInterface {
public:
    virtual ~DirectGuideInterface() {}
    virtual int directGuideMoveTelescope(const double& dRA, const double& dDec) = 0;
    virtual int directGuideAbort() = 0;
    virtual bool directGuideAsynchronous() = 0;
    virtual int setDirectGuideAsynchronous(bool bAsync) = 0;
};
```

## Implementation Guide

Follow these steps to implement these interfaces in your X2 plugin.

1. Declare the classes in your header file, such as `x2mount.h`. Use the signatures provided above.
2. Make your main plugin class inherit from these interfaces. For example, `class X2Mount : public SlewToInterface, public DirectGuideInterface`.
3. Update the `queryAbstraction` method to return the casted pointer when TSX asks for these interface names.

### Example queryAbstraction Implementation

```cpp
int X2Mount::queryAbstraction(const char* pszName, void** ppVal) {
    *ppVal = NULL;

    if (!strcmp(pszName, "DirectGuideInterface"))
        *ppVal = dynamic_cast<DirectGuideInterface*>(this);

    return (*ppVal != NULL) ? SB_OK : ERR_NOT_IMPL;
}
```

The string names used in `queryAbstraction` match the class names exactly.

## ZWO Protocol Implementation Notes

### DirectGuideInterface

`DirectGuideInterface` is fully implemented for ZWO. The interface delivers guide offsets as arcsecond deltas; the mount executes time-based pulses. The conversion happens in `X2Mount::directGuideMoveTelescope()`.

#### Interface contract

| Method | TSX contract | Our implementation |
|--------|-------------|-------------------|
| `directGuideMoveTelescope(dRA, dDec)` | Apply `dRA`/`dDec` arcsecond offsets to the mount | Converts to pulse durations and sends `:Mgd{dir}{nnnn}#` |
| `directGuideAbort()` | Abort any in-progress guide move | Calls `Abort()` → `:Q#` |
| `directGuideAsynchronous()` | Return true if moves complete asynchronously | Returns `true` — ZWO pulses are fire-and-forget |
| `setDirectGuideAsynchronous(bool)` | Allow TSX to toggle async mode | No-op; always async |

#### Arcsecond-to-millisecond conversion

TSX passes `dRA` and `dDec` as signed arcseconds. The mount expects a pulse duration in milliseconds via `:Mgd{dir}{nnnn}#`. Conversion uses the configured guide rate (`m_dZWOGuideRate`, a fraction of sidereal):

```
sidereal_rate  = 15.04106858 arcsec/sec
guide_rate     = m_dZWOGuideRate × sidereal_rate   (arcsec/sec)
duration_ms    = |offset_arcsec| / guide_rate × 1000
```

Sign → direction mapping: RA+ = `e`, RA− = `w`, Dec+ = `n`, Dec− = `s`. Zero-duration pulses are skipped. RA and Dec are sent as separate sequential commands.

#### Wire format

`OnStep::startPulseGuide(dir, ms)` formats the command:

```
:Mgd{dir}{nnnn}#
```

where `{dir}` is one of `n`/`s`/`e`/`w` and `{nnnn}` is a zero-padded 4-digit millisecond duration (e.g. `:Mgde0250#` for a 250 ms east pulse). The mount sends no response — `sendCommand` is called with `SHORT_RESPONSE, 0`.

#### Guide rate configuration

`m_dZWOGuideRate` is persisted in INI under `ZWOGuideRate` (default `0.5`, i.e. 50% sidereal ≈ 7.52 arcsec/sec) and exposed as a spinbox in the ZWO settings dialog. On connect, `ZWOMount::getGuideRate()` sends `:Ggr#` to read the mount's actual guide rate and overwrites both the OnStep member and `X2Mount::m_dZWOGuideRate`, so the math always reflects hardware reality. When the settings dialog is saved, `ZWOMount::setGuideRate()` sends `:Rg{rate}#` to keep the mount register in sync with any value the user changed in the spinbox.