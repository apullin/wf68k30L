#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/../.." && pwd)"
cd "${REPO_ROOT}"

TOP=${TOP:-WF68K30L_TOP}
BUILD_DIR=${BUILD_DIR:-build/rep_ecp5}

YOSYS_BIN=${YOSYS_BIN:-yosys}
NEXTPNR_BIN=${NEXTPNR_BIN:-nextpnr-ecp5}
ECPPACK_BIN=${ECPPACK_BIN:-ecppack}

DEVICE=${DEVICE:-45k}          # 12k, 25k, 45k, 85k, um-*, um5g-*
PACKAGE=${PACKAGE:-CABGA381}
SPEED=${SPEED:-8}
FREQ_MHZ=${FREQ_MHZ:-25}
LPF=${LPF:-synth/constraints/wf68k30l_representative_25mhz.lpf}
SDC=${SDC:-synth/constraints/wf68K30L.sdc}
# Default to ABC9; set USE_ABC9=0 as a temporary fallback until older toolchains are retired.
USE_ABC9=${USE_ABC9:-1}

mkdir -p "${BUILD_DIR}"

echo "[1/3] Synthesizing ${TOP} with Yosys..."
if [[ "${USE_ABC9}" == "1" ]]; then
  SYNTH_CMD="read_verilog -sv -I sv sv/wf68k30L_*.sv; synth_ecp5 -top ${TOP}; write_json ${BUILD_DIR}/${TOP}.json"
  echo "      using ABC9 flow (default)"
else
  # Fallback if a specific Yosys/nextpnr combination still reports ABC9 loop issues.
  SYNTH_CMD="read_verilog -sv -I sv sv/wf68k30L_*.sv; synth_ecp5 -noabc9 -top ${TOP}; write_json ${BUILD_DIR}/${TOP}.json"
  echo "      using non-ABC9 flow (-noabc9)"
fi
"${YOSYS_BIN}" -l "${BUILD_DIR}/synth.log" -p "${SYNTH_CMD}"

echo "[2/3] Running nextpnr-ecp5 (${DEVICE}, ${PACKAGE}, speed ${SPEED}, ${FREQ_MHZ} MHz)..."
"${NEXTPNR_BIN}" \
  "--${DEVICE}" \
  --package "${PACKAGE}" \
  --speed "${SPEED}" \
  --json "${BUILD_DIR}/${TOP}.json" \
  --textcfg "${BUILD_DIR}/${TOP}.config" \
  --lpf "${LPF}" \
  --sdc "${SDC}" \
  --lpf-allow-unconstrained \
  --freq "${FREQ_MHZ}" \
  --timing-allow-fail \
  --log "${BUILD_DIR}/nextpnr.log"

echo "[3/3] Packing bitstream..."
"${ECPPACK_BIN}" "${BUILD_DIR}/${TOP}.config" "${BUILD_DIR}/${TOP}.bit"

echo
echo "Done."
echo "  JSON:   ${BUILD_DIR}/${TOP}.json"
echo "  Config: ${BUILD_DIR}/${TOP}.config"
echo "  Bit:    ${BUILD_DIR}/${TOP}.bit"
echo "  Logs:   ${BUILD_DIR}/synth.log, ${BUILD_DIR}/nextpnr.log"
