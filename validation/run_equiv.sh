#!/bin/bash
# Bounded sequential equivalence between two revisions of the SV core.
#
#   ./validation/run_equiv.sh [GOLD_REV] [CYCLES]
#
# GOLD_REV defaults to HEAD, so the common use is "does my working tree behave
# identically to the last commit?" -- a refactoring guard. Both revisions are
# elaborated, combined into a miter, and proved equivalent over CYCLES clock
# cycles with Yosys's SAT engine. Unlike a randomized testbench this is
# exhaustive over all inputs within the bound.
#
# Requirements: yosys, git.
#
# Scope, and it is narrow -- read this before trusting a clean run:
#
#   * It detects *change*, not correctness. A pass means the refactor preserved
#     observable behaviour; it says nothing about whether that behaviour matches
#     the MC68030. The cocotb suite remains the correctness gate.
#   * The bound is small. 12 cycles from a zeroed reset state is barely one bus
#     cycle, so this catches changes observable at the ports early -- a mistyped
#     output expression, a dropped enable, a swapped polarity -- and will NOT
#     catch anything that needs an instruction to execute. Verified both ways:
#     tying CIOUTn low is caught immediately, while forcing TRAP_DIVZERO low is
#     NOT caught at this depth, because reaching a divide takes far longer than
#     the bound. Depth costs roughly 60-75s at 8-12 cycles and climbs steeply.
#
# So: use it as a fast "did I change anything at all?" check around a refactor,
# in addition to the regression suite, never instead of it.
#
# The former VHDL-vs-SV comparison is retired and is not coming back:
#
#   * The SV top has MMU, cache and coprocessor subsystems the VHDL never had,
#     so the two port lists cannot correspond.
#   * The July 2026 audit fixed defects the SV had inherited faithfully from the
#     VHDL -- NBCD's borrow, CMPA.W's flag width, CHK2/CMP2 size gating, RTD's
#     stack adjustment, the T1 trace return address, the reset-and-HALT filter,
#     and others. The SV is now deliberately not equivalent to the VHDL there,
#     and should not be.

set -e
REPO="$(cd "$(dirname "$0")/.." && pwd)"
cd "$REPO"

GOLD_REV="${1:-HEAD}"
CYCLES="${2:-12}"

command -v yosys >/dev/null 2>&1 || { echo "missing required tool: yosys"; exit 1; }
command -v git   >/dev/null 2>&1 || { echo "missing required tool: git"; exit 1; }

git rev-parse --verify "$GOLD_REV" >/dev/null

WORKDIR=$(mktemp -d)
trap 'rm -rf "$WORKDIR"' EXIT

GOLDTREE="$WORKDIR/gold"
mkdir -p "$GOLDTREE"
git archive "$GOLD_REV" sv | tar -x -C "$GOLDTREE"

srcs() {  # $1 = tree root; package first, top last
    local root="$1"
    echo -n "$root/sv/wf68k30L_pkg.sv "
    ls "$root"/sv/wf68k30L_*.sv \
        | grep -v 'wf68k30L_pkg\.sv$' \
        | grep -v 'wf68k30L_top\.sv$' \
        | tr '\n' ' '
    echo -n "$root/sv/wf68k30L_top.sv"
}

echo "=== gold: $GOLD_REV ($(git log -1 --format='%h %s' "$GOLD_REV")) ==="
echo "=== gate: working tree ==="
echo "=== proving equivalence over $CYCLES cycles ==="

# memory_map bit-blasts the register files and caches; the SAT engine wants them
# as plain logic rather than $mem cells. keep_hierarchy (set for the FPGA timing
# flow) would otherwise survive prep -flatten and leave cells the SAT engine
# cannot import.
yosys -p "
    read_verilog -sv -I $GOLDTREE/sv $(srcs "$GOLDTREE");
    hierarchy -top WF68K30L_TOP;
    setattr -mod -unset keep_hierarchy;
    prep -top WF68K30L_TOP -flatten;
    memory_map; async2sync; opt -full;
    rename WF68K30L_TOP gold_top;
    design -stash gold;

    read_verilog -sv -I $REPO/sv $(srcs "$REPO");
    hierarchy -top WF68K30L_TOP;
    setattr -mod -unset keep_hierarchy;
    prep -top WF68K30L_TOP -flatten;
    memory_map; async2sync; opt -full;
    rename WF68K30L_TOP gate_top;
    design -stash gate;

    design -copy-from gold -as gold_top gold_top;
    design -copy-from gate -as gate_top gate_top;

    miter -equiv -flatten -make_assert gold_top gate_top miter;
    hierarchy -top miter;
    opt -full;
    sat -seq $CYCLES -verify -prove-asserts -set-init-zero miter
"
