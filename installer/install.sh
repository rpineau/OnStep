#!/usr/bin/env bash
# install.sh — Build and install the OnStep X2 mount plugin for TheSkyX
#
# Usage (run from the project root, or from anywhere — script is self-relocating):
#   ./installer/install.sh              # build and install
#   ./installer/install.sh --uninstall  # remove installed files
#
# No sudo needed if you own the TheSkyX app bundle (typical single-user install).
# If you get "Permission denied": sudo ./installer/install.sh

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

# ---------------------------------------------------------------------------
# Platform detection
# ---------------------------------------------------------------------------
UNAME_S="$(uname -s)"
case "${UNAME_S}" in
    Darwin)
        TSX_APP="/Applications/TheSkyX Professional Edition.app"
        PLUGIN_DIR="${TSX_APP}/Contents/PlugIns/MountPlugIns"
        LIST_DIR="${TSX_APP}/Contents/Resources/Common/Miscellaneous Files"
        LIB_NAME="libOnStep.dylib"
        ;;
    Linux)
        TSX_HOME="${HOME}/TheSkyX"
        # Detect the installed architecture variant
        PLUGINS_ROOT=""
        for ARCH_DIR in PlugIns64 PlugInsARM64 PlugInsARM32 PlugIns; do
            if [ -d "${TSX_HOME}/Resources/Common/${ARCH_DIR}" ]; then
                PLUGINS_ROOT="${TSX_HOME}/Resources/Common/${ARCH_DIR}"
                break
            fi
        done
        if [ -z "${PLUGINS_ROOT}" ]; then
            echo "ERROR: Cannot find TheSkyX plugins directory under:" >&2
            echo "  ${TSX_HOME}/Resources/Common/" >&2
            echo "Is TheSkyX installed? Set TSX_HOME if it lives elsewhere." >&2
            exit 1
        fi
        PLUGIN_DIR="${PLUGINS_ROOT}/MountPlugIns"
        LIST_DIR="${TSX_HOME}/Resources/Common/Miscellaneous Files"
        LIB_NAME="libOnStep.so"
        ;;
    *)
        echo "ERROR: Unsupported platform: ${UNAME_S}" >&2
        exit 1
        ;;
esac

LIST_FILE="mountlist OnStep.txt"

# ---------------------------------------------------------------------------
# Uninstall
# ---------------------------------------------------------------------------
if [[ "${1:-}" == "--uninstall" ]]; then
    echo "Uninstalling OnStep plugin..."
    rm -f "${PLUGIN_DIR}/${LIB_NAME}"  && echo "  Removed ${PLUGIN_DIR}/${LIB_NAME}"
    rm -f "${PLUGIN_DIR}/OnStep.ui"    && echo "  Removed ${PLUGIN_DIR}/OnStep.ui"
    rm -f "${PLUGIN_DIR}/OnStep.png"   && echo "  Removed ${PLUGIN_DIR}/OnStep.png"
    rm -f "${PLUGIN_DIR}/ZWO.png"      && echo "  Removed ${PLUGIN_DIR}/ZWO.png"
    rm -f "${LIST_DIR}/${LIST_FILE}"   && echo "  Removed ${LIST_DIR}/${LIST_FILE}"
    echo "Done."
    exit 0
fi

# ---------------------------------------------------------------------------
# Pre-flight checks
# ---------------------------------------------------------------------------
if [[ ! -d "${PLUGIN_DIR}" ]]; then
    echo "ERROR: Plugin directory not found:" >&2
    echo "  ${PLUGIN_DIR}" >&2
    echo "Is TheSkyX installed?" >&2
    exit 1
fi

if [[ ! -d "${LIST_DIR}" ]]; then
    echo "ERROR: Miscellaneous Files directory not found:" >&2
    echo "  ${LIST_DIR}" >&2
    exit 1
fi

if [[ ! -f "${PROJECT_DIR}/${LIST_FILE}" ]]; then
    echo "ERROR: Mount list file not found:" >&2
    echo "  ${PROJECT_DIR}/${LIST_FILE}" >&2
    exit 1
fi

# ---------------------------------------------------------------------------
# Build
# ---------------------------------------------------------------------------
echo "Building ${LIB_NAME}..."
cd "${PROJECT_DIR}"
make clean
make

if [[ ! -f "${PROJECT_DIR}/${LIB_NAME}" ]]; then
    echo "ERROR: Build failed — ${LIB_NAME} not produced." >&2
    exit 1
fi

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------
echo "Installing plugin..."

cp "${PROJECT_DIR}/${LIB_NAME}"       "${PLUGIN_DIR}/${LIB_NAME}"
echo "  -> ${PLUGIN_DIR}/${LIB_NAME}"

cp "${PROJECT_DIR}/OnStep.ui"         "${PLUGIN_DIR}/OnStep.ui"
echo "  -> ${PLUGIN_DIR}/OnStep.ui"

cp "${PROJECT_DIR}/OnStep.png"        "${PLUGIN_DIR}/OnStep.png"
echo "  -> ${PLUGIN_DIR}/OnStep.png"

cp "${PROJECT_DIR}/ZWO.png"           "${PLUGIN_DIR}/ZWO.png"
echo "  -> ${PLUGIN_DIR}/ZWO.png"

cp "${PROJECT_DIR}/${LIST_FILE}"      "${LIST_DIR}/${LIST_FILE}"
echo "  -> ${LIST_DIR}/${LIST_FILE}"

chmod 755 "${PLUGIN_DIR}/${LIB_NAME}"

echo ""
echo "Installation complete."
echo "Restart TheSkyX and select your OnStep mount from the mount device list."
