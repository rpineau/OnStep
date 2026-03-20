# AGENTS.md — OnStep X2 Mount Plugin

## Project Overview

TheSkyX X2 mount driver plugin for OnStep telescope controllers. C++ shared library
communicating via LX200-compatible serial protocol. Targets Linux (.so), macOS (.dylib),
and Windows (.dll). Author: Rodolphe Pineau / RTI-Zone.

**IMPORTANT:** This is an **open source project!**.
All contributions are voluntary assistance from AI agents in conversation with a user.
**You MUST ask for explicit confirmation before pushing ANY changes to the remote repository,
including branches and tags.** Do not assume permission to push—always verify with the user first.

### Architecture (3 files)

- **`OnStep.h/cpp`** — Hardware driver. Serial command/response, coordinate conversion,
  mount state machine (tracking, slewing, parking, homing). All LX200 protocol here.
- **`x2mount.h/cpp`** — X2 SDK adapter. Implements TheSkyX interfaces (SlewTo, Sync,
  Park, Unpark, TrackingRates, OpenLoopMove, AsymmetricalEquatorial, etc.).
  Delegates to `OnStep` instance via `m_OnStep`. All methods acquire `X2MutexLocker`.
- **`main.h/cpp`** — Plugin factory. `sbPlugInFactory2` creates `X2Mount` instances.

### Key dependencies

- TheSkyX X2 SDK: `../../licensedinterfaces/` (relative path, not in this repo)
- `StopWatch.h` — Timer utility (third-party, BSD license, do not modify)

## Build Commands

The Makefile auto-detects the platform (`uname -s`) and sets the correct flags.

```bash
make clean    # Remove build artifacts
make          # Produces libOnStep.dylib (macOS) or libOnStep.so (Linux)
```

| Platform | Output | OS flag | Link flags |
|----------|--------|---------|------------|
| macOS    | `libOnStep.dylib` | `-DSB_MACOSX_BUILD` | `-dynamiclib -lstdc++` |
| Linux    | `libOnStep.so`    | `-DSB_LINUX_BUILD`  | `-shared -lstdc++`     |

The Makefile also runs `uic OnStep.ui` validation before compiling if `uic` is on `PATH`
or at the standard Homebrew path (`/usr/local/opt/qt@5/bin/uic` or `/opt/homebrew/opt/qt@5/bin/uic`).

### Windows build

Use Visual Studio solution in `libOnStep/` — produces `libOnStep.dll` (32 & 64-bit).

### Installation (Linux and macOS)

The installer script is self-contained — it builds the plugin and installs it in one step.
Run it from the project root (it is self-relocating, so the cwd does not strictly matter).

```bash
./installer/install.sh              # build + install
./installer/install.sh --uninstall  # remove installed files
```

**macOS:** installs to `/Applications/TheSkyX Professional Edition.app/Contents/PlugIns/MountPlugIns/`.

**Linux:** installs to `~/TheSkyX/Resources/Common/PlugIns64/MountPlugIns/` (auto-detects ARM variants).
Override the TheSkyX home directory by setting `TSX_HOME` before running the script.

## Testing

**No test framework.** No unit tests exist. Verification is manual against live hardware.


### UI File Validation

Before committing or building changes to `.ui` files, validate their XML structure. Merge conflicts often silently break `.ui` files. Use the Qt User Interface Compiler (`uic`) to catch syntax errors and ensure the UI definition is valid:

```bash
uic OnStep.ui > /dev/null
```
If the command outputs nothing and exits with a `0` status, the syntax is intact. If it fails, it will print a parse error indicating the line number and exact nature of the malformed XML tags, which you must manually fix in `OnStep.ui`.

**Dependencies:**
`uic` is provided by the Qt5 base development tools. To install it on Debian/Ubuntu systems, run:
```bash
sudo apt-get update && sudo apt-get install -y qtbase5-dev-tools qtchooser
```


To enable debug logging, uncomment in `OnStep.h`:
```cpp
#define PLUGIN_DEBUG 2   // 1 = errors only, 2+ = full trace
```
Log writes to `~/OnStepLog.txt` (Linux/macOS) or `%HOMEDRIVE%%HOMEPATH%\OnStepLog.txt` (Windows).

## Code Style

### Language & Standard

- **C++11** (`-std=gnu++11`). Do not use C++14/17/20 features.
- Compile with `-Wall -Wextra`. Fix all warnings.

### Naming Conventions

| Element          | Convention                | Example                          |
|------------------|---------------------------|----------------------------------|
| Class            | PascalCase                | `OnStep`, `X2Mount`, `CStopWatch` |
| Member variable  | `m_` + Hungarian prefix   | `m_bIsConnected`, `m_dRa`, `m_nPortSpeed`, `m_sPort` |
| Local variable   | Hungarian prefix          | `nErr`, `sResp`, `dRa`, `bComplete` |
| Method (public)  | camelCase                 | `getRaAndDec()`, `startSlewTo()` |
| Method (private) | camelCase                 | `sendCommand()`, `readResponse()` |
| Constants/macros | UPPER_SNAKE_CASE          | `MAX_TIMEOUT`, `SERIAL_BUFFER_SIZE` |
| Enum values      | UPPER_SNAKE_CASE          | `PLUGIN_OK`, `NOT_CONNECTED` |

**Hungarian prefixes:** `b` = bool, `n` = int, `d` = double, `s` = string, `ss` = stringstream,
`p` = pointer, `v`/`sv` = vector, `c` = char, `ul` = unsigned long.

### Formatting

- **Indentation:** Tabs (not spaces).
- **Braces:** Opening brace on same line as statement. Closing brace on own line.
- **Single-statement if:** No braces (codebase convention, maintain consistency).
- **Spacing:** Space after `if`/`for`/`while`, no space before `(` in function calls.
- **Line width:** No strict limit; long lines are acceptable (debug logging, stream chains).

```cpp
// Canonical pattern
if(nErr) {
    return nErr;
}

if(!m_bLinked)
    return ERR_NOLINK;
```

### File Organization

- Use `#pragma mark - Section Name` to separate logical sections.
- Header guards: `#ifndef __ClassName__` / `#define __ClassName__` (double underscore style).
- Also include `#pragma once` after the `#ifndef`.
- System includes before project includes. SDK includes use relative paths.

### Includes (order)

```cpp
// 1. Own header
#include "OnStep.h"
// 2. C standard headers
#include <stdio.h>
// 3. C++ standard headers
#include <string>
#include <vector>
// 4. SDK interfaces (relative path)
#include "../../licensedinterfaces/sberrorx.h"
// 5. Local project headers
#include "StopWatch.h"
```

### Error Handling

- **Return codes, not exceptions.** Every function returns `int` (0 = success).
- Use `OnStepErrors` enum: `PLUGIN_OK`, `NOT_CONNECTED`, `COMMAND_FAILED`, `COMMAND_TIMEOUT`, etc.
- SDK error codes: `SB_OK`, `ERR_NOLINK`, `ERR_CMDFAILED`, `ERR_COMMNOLINK`, etc.
- On failed serial comms, set `m_bIsConnected = false` and return error.
- The only exceptions allowed are in `std::stod()` conversions — always wrap in `try/catch`.

```cpp
int nErr = PLUGIN_OK;
nErr = sendCommand(":GVN#", sResp);
if(nErr) {
    return nErr;       // propagate, do not swallow
}
```

### Serial Communication Pattern

All mount communication goes through `sendCommand()` → `readResponse()`:

```cpp
// Standard command (response ends with '#')
nErr = sendCommand(":GRH#", sResp);

// Short response (single byte, no '#' terminator)
nErr = sendCommand(":MS#", sResp, MAX_TIMEOUT, SHORT_RESPONSE, 1);

// Fire-and-forget (no response expected)
nErr = sendCommand(":Q#", sResp, 0);
```

After commands that modify mount state, add a delay:
```cpp
std::this_thread::sleep_for(std::chrono::milliseconds(100));
```

### Debug Logging

Wrap all debug output in preprocessor guards:

```cpp
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [methodName] message" << std::endl;
    m_sLogFile.flush();
#endif
```

- Level 1 (`PLUGIN_DEBUG >= 1`): Errors only.
- Level 2 (`PLUGIN_DEBUG >= 2`): Full trace (function entry, values, flow).
- Level 3 (`PLUGIN_DEBUG >= 3`): Byte-level serial I/O.
- Always `flush()` after writing.

### X2 Interface Pattern

All X2Mount methods that access hardware must:
1. Check `m_bLinked` — return `ERR_NOLINK` if not connected.
2. Acquire mutex — `X2MutexLocker ml(GetMutex());`
3. Delegate to `m_OnStep` — never talk to serial directly.

```cpp
int X2Mount::someMethod() {
    int nErr = SB_OK;
    if(!m_bLinked)
        return ERR_NOLINK;
    X2MutexLocker ml(GetMutex());
    nErr = m_OnStep.doSomething();
    if(nErr)
        return ERR_CMDFAILED;
    return nErr;
}
```

For `const` methods that need non-const access:
```cpp
X2Mount* pMe = (X2Mount*)this;
X2MutexLocker ml(pMe->GetMutex());
```

### Configuration / Settings

INI keys defined as macros in `x2mount.h`:
```cpp
#define PARENT_KEY           "OnStepMount"
#define CHILD_KEY_PORT_NAME  "PortName"
// etc.
```

Read/write via `m_pIniUtil->readInt()` / `m_pIniUtil->writeInt()`.

## Common Pitfalls

- **WiFi latency:** OnStep WiFi can take up to 1600ms to respond. Default `MAX_TIMEOUT = 2000`.
- **Inter-command delay:** 150ms minimum between serial commands (`INTER_COMMAND_DELAY_SECONDS`).
- **Retry on failure:** For coordinate reads (`getRaAndDec`, `getAltAndAz`), retry once
  after 200ms sleep, then return cached values with `PLUGIN_OK` (graceful degradation).
- **Platform guards:** Use `#ifdef WIN32` for platform-specific code, `SB_LINUX_BUILD` for Linux.
- **Do not modify** `StopWatch.h` — third-party BSD-licensed utility.
