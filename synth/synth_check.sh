#!/bin/bash
# synth_check.sh -- Run Yosys synthesis and compare LUT/FF counts against baseline
#
# Usage: ./synth_check.sh [--ecp5]
#
# Baseline values (from reference synthesis):
#   LUT4:       39317
#   TRELLIS_FF: 6515

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

# Baseline. These are a regression tripwire, not a target: they correspond to
# plain `synth_ecp5` defaults on the whole sv/ tree (no ABC9 flags, no
# -noflatten). They are much larger than the pre-2026 figures because the MMU
# and cache subsystems did not exist then. Re-measure and update deliberately.
BASELINE_LUT4=39317
BASELINE_FF=6515
TOLERANCE_PCT=10  # Allow +/- 10% deviation

# Source files
SV_DIR="$REPO_DIR/sv"
SV_PKG="$SV_DIR/wf68k30L_pkg.sv"
# Read every module in sv/ so the list cannot go stale as modules are added.
# (Matches the direct Yosys command documented in README.md.)
SV_FILES=()
while IFS= read -r f; do
    [ "$f" = "$SV_PKG" ] && continue
    SV_FILES+=("$f")
done < <(ls "$SV_DIR"/wf68k30L_*.sv | sort)

TOP=WF68K30L_TOP
TMPLOG=$(mktemp /tmp/synth_check_XXXXXX.log)
trap "rm -f $TMPLOG" EXIT

echo "=== WF68K30L Synthesis Check ==="
echo "Running Yosys ECP5 synthesis..."

# Build Yosys script -- use read_verilog for SystemVerilog
# (Falls back to GHDL+VHDL if SV read fails)
YOSYS_SCRIPT="
read_verilog -sv -I${REPO_DIR} -I${SV_DIR} ${SV_PKG} ${SV_FILES[*]};
synth_ecp5 -top ${TOP};
stat;
"

if yosys -p "$YOSYS_SCRIPT" > "$TMPLOG" 2>&1; then
    echo "Yosys completed successfully."
else
    echo "WARNING: Yosys synthesis had issues. Checking log..."
fi

# Extract counts from stat output
# Read the design-wide totals from the hierarchy summary, not a per-module line.
HIER=$(awk '/^=== design hierarchy ===/,0' "$TMPLOG")
LUT4=$(echo "$HIER" | grep -oE '[0-9]+[[:space:]]+LUT4$' | grep -oE '^[0-9]+' | tail -1 || echo "0")
FF=$(echo "$HIER" | grep -oE '[0-9]+[[:space:]]+TRELLIS_FF$' | grep -oE '^[0-9]+' | tail -1 || echo "0")

if [ -z "$LUT4" ] || [ "$LUT4" = "0" ]; then
    echo "FAIL: Could not extract LUT4 count from synthesis output"
    echo "Last 30 lines of log:"
    tail -30 "$TMPLOG"
    exit 1
fi

if [ -z "$FF" ] || [ "$FF" = "0" ]; then
    echo "FAIL: Could not extract TRELLIS_FF count from synthesis output"
    echo "Last 30 lines of log:"
    tail -30 "$TMPLOG"
    exit 1
fi

echo ""
echo "=== Synthesis Results ==="
echo "  LUT4:       $LUT4  (baseline: $BASELINE_LUT4)"
echo "  TRELLIS_FF: $FF    (baseline: $BASELINE_FF)"

# Calculate percentage difference
LUT4_DIFF=$(( (LUT4 - BASELINE_LUT4) * 100 / BASELINE_LUT4 ))
FF_DIFF=$(( (FF - BASELINE_FF) * 100 / BASELINE_FF ))

echo "  LUT4 delta: ${LUT4_DIFF}%"
echo "  FF delta:   ${FF_DIFF}%"
echo ""

# Check tolerance
PASS=true

if [ "${LUT4_DIFF#-}" -gt "$TOLERANCE_PCT" ]; then
    echo "FAIL: LUT4 count deviates by ${LUT4_DIFF}% (tolerance: +/-${TOLERANCE_PCT}%)"
    PASS=false
else
    echo "PASS: LUT4 within tolerance"
fi

if [ "${FF_DIFF#-}" -gt "$TOLERANCE_PCT" ]; then
    echo "FAIL: TRELLIS_FF count deviates by ${FF_DIFF}% (tolerance: +/-${TOLERANCE_PCT}%)"
    PASS=false
else
    echo "PASS: TRELLIS_FF within tolerance"
fi

echo ""
if [ "$PASS" = true ]; then
    echo "=== OVERALL: PASS ==="
    exit 0
else
    echo "=== OVERALL: FAIL ==="
    exit 1
fi
