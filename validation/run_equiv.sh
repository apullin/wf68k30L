#!/bin/bash
# VHDL-vs-SV equivalence co-simulation
#
# Synthesizes both designs to gate-level Verilog via Yosys, then
# runs 50K cycles of randomized bus stimulus comparing all outputs.
#
# Requirements: yosys (with ghdl plugin via -m ghdl), iverilog, vvp, python3
# GHDL_PREFIX must point to the GHDL library dir
#
# Results (2026-02-22): 50,000 cycles, 5 seeds, 0 mismatches.

set -e
REPO="$(cd "$(dirname "$0")/.." && pwd)"
cd "$REPO"

: "${GHDL_PREFIX:?Set GHDL_PREFIX to your GHDL library dir (e.g. .../oss-cad-suite/lib/ghdl)}"

WORKDIR=$(mktemp -d)
trap "rm -rf $WORKDIR" EXIT

echo "=== Exporting VHDL netlist (gold) ==="
GHDL_PREFIX="$GHDL_PREFIX" yosys -m ghdl -q \
    -p "script validation/export_gold.ys; write_verilog $WORKDIR/gold_top.v"

echo "=== Exporting SV netlist (gate) ==="
yosys -q \
    -p "script validation/export_gate.ys; write_verilog $WORKDIR/gate_top.v"

echo "=== Adding register initialization for simulation ==="
python3 -c "
import re, sys
for fname in sys.argv[1:]:
    with open(fname) as f:
        content = f.read()
    regs = re.findall(r'^\s+reg\s+(?:\[\d+:\d+\]\s+)?(\S+)\s*;', content, re.MULTILINE)
    init_lines = ['  initial begin'] + [f'    {r} = 0;' for r in regs] + ['  end']
    content = content.replace('endmodule', chr(10).join(init_lines) + chr(10) + 'endmodule')
    with open(fname, 'w') as f:
        f.write(content)
    print(f'  {len(regs)} registers in {fname.split(\"/\")[-1]}')
" "$WORKDIR/gold_top.v" "$WORKDIR/gate_top.v"

echo "=== Compiling co-simulation ==="
iverilog -o "$WORKDIR/equiv_sim" \
    "$WORKDIR/gold_top.v" "$WORKDIR/gate_top.v" validation/equiv_tb.v

echo "=== Running 50K-cycle equivalence test (5 seeds) ==="
vvp "$WORKDIR/equiv_sim"
