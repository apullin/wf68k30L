# WF68K30L — SystemVerilog Port

This is a SystemVerilog port of the WF68K30L MC68030-compatible CPU core,
originally written in VHDL by Wolfgang Foerster.

## Branches

- **main** — Original VHDL with minor fixes for GHDL/Yosys synthesis compatibility
- **verilog** — Full SystemVerilog port, synthesizable with Yosys

## Synthesis Results

Current tree, plain `synth_ecp5` defaults over all of `sv/` (`bash
synth/synth_check.sh`, which also checks these as a +/-10% regression tripwire):

| Resource   | Count  |
|------------|--------|
| LUT4       | 40,544 |
| TRELLIS_FF |  6,924 |
| CCU2C      |  2,180 |
| DP16KD     |     10 |
| MULT18X18D |      5 |

At ~40.5k LUT4 this no longer fits an LFE5U-25F; it needs an LFE5U-45F (43,848
LUT4, so tight at ~92%) or an 85F. The older "19,205 LUT4 / fits a 25F" figure
that used to appear here predates the MMU, cache and coprocessor subsystems and
is not comparable -- those were added after it was measured, and they dominate
the growth.

Xilinx, for cross-checking portability (Vivado 2025.2.1, xc7a200tfbg484-2,
`synth_design -flatten_hierarchy none`): 18,899 LUTs (14.0%) and 6,170
registers (2.3%), with zero latches, zero combinational loops and zero
multiply-driven nets. LUT counts are not comparable across vendors -- ECP5 LUT4
versus Xilinx LUT6.

The original VHDL-vs-SV comparison (26,902 -> 19,205 LUT4) applied to the
pre-MMU core only. The reduction came from replacing VHDL for-loops with
equality comparisons and bitwise mask operations, plus dropping `async2sync`
overhead.

Build commands (repo flow):

    ./synth/fpga/run_ecp5_representative.sh
    USE_ABC9=0 ./synth/fpga/run_ecp5_representative.sh

Direct Yosys command (default ABC9 mapping):

    yosys -p 'read_verilog -sv -I sv sv/wf68k30L_*.sv; synth_ecp5 -top WF68K30L_TOP'

## Representative P&R Baseline (ECP5, 25 MHz)

Current repo defaults:

- `synth/fpga/run_ecp5_representative.sh` uses **ABC9 by default** (`USE_ABC9=1`).
- `synth/constraints/wf68K30L.sdc` intentionally keeps only the primary clock constraint; this
  avoids unsupported false-path directives in `nextpnr-ecp5`.

Baseline seed sweep command:

    ./synth/fpga/run_ecp5_seed_sweep.sh

### Xilinx KV260 (Kria K26, Zynq UltraScale+) — timing closes

Default target of `synth/vivado/run_vivado.tcl`. Full place-and-route,
`xck26-sfvc784-2LV-c`, 25 MHz constraint:

| Metric | Value |
|--------|-------|
| Fmax | **32.57 MHz (passes the 25 MHz target)** |
| Worst negative slack | +9.300 ns |
| Failing endpoints | 0 of 15,061 setup, 0 hold |
| CLB LUTs | 17,322 / 117,120 (14.8%) |
| CLB Registers | 5,924 / 234,240 (2.5%) |
| Block RAM | 3 / 144 tiles (2.1%) |
| Errors / inferred latches / DRC violations | 0 / 0 / 0 |

Caveat: this is an out-of-context build with no pinout, so clock skew is not
fully modelled (Vivado warns `HD.CLK_SRC` is unset). Treat the figure as
indicative rather than board-final — a real KV260 design clocks the PL from the
PS through the Zynq block rather than from a bare port.

### Lattice ECP5 — timing does NOT close

`./synth/fpga/run_ecp5_representative.sh`, LFE5U-85F CABGA381 speed 8, 25 MHz
target, ABC9:

| Metric | Value |
|--------|-------|
| TRELLIS_COMB | 46,225 / 83,640 (55%) |
| TRELLIS_FF | 6,922 / 83,640 (8%) |
| DP16KD | 10 / 208 (4%) |
| MULT18X18D | 5 / 156 (3%) |
| Fmax | **11.21 MHz — fails the 25 MHz target** |

The critical path runs from the decoded-opcode register (`I_OPCODE_DECODER.OP`)
through the control module's combinational next-state logic to the `CIOUT`
decision latch: **127 logic stages, about 89 ns**. The cache-inhibit decision
therefore sits at the end of a deep cone containing the transparent-translation
match, the ATC lookup and the function-code decode. That cone is what an ECP5
build would need to pipeline; it is deliberately left alone because the KV260
target clears its constraint by a third of a period, and pipelining it would be
invasive for no benefit there.

The figures that used to appear here (21,941 LUT4, 26.69 MHz passing) predate the
MMU, cache and coprocessor subsystems and are not comparable.

## Equivalence Checking (SV revision vs SV revision)

    ./validation/run_equiv.sh [GOLD_REV] [CYCLES]

Proves bounded sequential equivalence between a git revision (default `HEAD`)
and the working tree, by elaborating both, building a miter and discharging it
with Yosys's SAT engine. Exhaustive over all inputs within the bound, unlike a
randomized testbench. Intended as a refactoring guard.

Read the scope limits in the script before trusting a pass. In short: it detects
*change*, not correctness, and the bound is small — 12 cycles from reset is
barely one bus cycle, so it catches changes observable at the ports early and
will not catch anything requiring an instruction to execute. Both directions are
verified: tying `CIOUTn` low is caught, while forcing `TRAP_DIVZERO` low is not,
because reaching a divide takes far longer than the bound. Roughly 60-75s at
8-12 cycles, climbing steeply after.

**The former VHDL-vs-SV comparison is retired**, and the figure that used to
appear here (50,000 cycles, 5 seeds, 0 mismatches) is withdrawn. It cannot be
revived meaningfully: the SV top has MMU, cache and coprocessor subsystems the
VHDL never had, so the port lists cannot correspond; and the July 2026 audit
fixed a number of defects the SV had inherited faithfully from the VHDL, so the
SV is now deliberately *not* equivalent to it in those places. See
`notes/AUDIT_2026-07.md` and the BUG-R009/R014 amendment in `notes/BUGLIST.md`.

## Software Smoke Battery

In addition to instruction-by-instruction regressions, `tb/test_software_battery.py`
runs longer software-style kernels that mix control flow, memory traffic, and ALU/divider
operations in one program image.

These cases enable lightweight bus invariants in the harness:

- Per-cycle stability: `RWn`/`SIZE` remain stable while `ASn` is asserted
- Bounded handshake progress: no single bus cycle may remain active beyond a
  configurable cycle bound
- Control decode sanity: sampled `ASn`/`DSn`/`RWn`/`SIZE` values must remain in
  legal encoded ranges

Run directly with:

    make -C tb TEST_MODULE=test_software_battery TOPLEVEL=WF68K30L_TOP

## QEMU Differential Smoke

A lightweight differential check is available against `qemu-system-m68k`
(`-cpu m68030`). It compares the first instruction-start PC trace of a short
deterministic program between:

- WF68K30L in cocotb/Verilator
- QEMU m68030 running the same raw program image

Run with:

    make test-qemu-diff

Seeded randomized differential run (register-state check at epilogue):

    make test-qemu-diff-fuzz
    QEMU_DIFF_SEED=7 QEMU_DIFF_OPS=64 make test-qemu-diff-fuzz
    QEMU_DIFF_SEEDS=1-200 QEMU_DIFF_OPS=128 make test-qemu-diff-fuzz

## Csmith Smoke

A bare-metal csmith flow is available for fuzz-style software smoke tests.
Each seed builds a random C program with `csmith`, cross-compiles with
`m68k-elf-gcc`, runs it in the cocotb harness, and **compares the program's
checksum against `qemu-system-m68k -cpu m68030` running the identical binary**.

That checksum comparison is the strongest end-to-end gate in the repo: it checks
computed results on real compiler output, not just that the program terminated.
It is also what caught the base-register-suppression defect that made every
indexed array access read through the wrong address (see
`notes/AUDIT_2026-07.md`) -- the core computed `0xFFFFFFFF` where QEMU computed
`0x3FE75C61`, while every hand-written instruction test passed.

Note the cycle budget: verifying the checksum costs real work, so the default is
12M cycles per seed. A budget too low presents as "did not reach sentinel",
which looks like a hang rather than the wrong answer it may actually be.

Requirements:

- `csmith`
- `m68k-elf-gcc`
- `m68k-elf-objcopy`

Run the integrated cocotb smoke module:

    make test-csmith-smoke

Default run covers a curated 10-seed set:
`1,4,5,6,7,8,10,12,13,19`.
When `CSMITH_CC_EXTRA_FLAGS` is unset, the csmith compile step defaults to
`-fno-jump-tables`.
Override seed selection or cycle budget:

    CSMITH_SEEDS=1-25 make test-csmith-smoke
    CSMITH_SEEDS=3,7,19 CSMITH_MAX_CYCLES=12000000 make test-csmith-smoke
    CSMITH_CC_EXTRA_FLAGS='-fno-jump-tables' make test-csmith-smoke

Run with jump tables enabled:

    make test-csmith-smoke-jump-tables
    CSMITH_CC_EXTRA_FLAGS='' make test-csmith-smoke

Build a standalone seed image manually:

    ./tooling/csmith/build_case.sh --seed 13 --out-dir build/csmith/seed_13

## CoreMark Smoke

CoreMark is integrated as a local bare-metal smoke run in `tb/test_coremark_smoke.py`.
This path builds and runs four optimization variants (`-O0`, `-O1`, `-O2`, `-Os`)
on the WF68K30L cocotb harness and prints a summary table with image size,
cycles, and run status (`ok`/`timeout`/`trap:*`).

Run with:

    make test-coremark-smoke

Defaults:

- `COREMARK_MAX_CYCLES=100000000`
- `COREMARK_ITERATIONS=1`
- `COREMARK_TOTAL_DATA_SIZE=2000`
- `COREMARK_OPTS=O0,O1,O2,Os`
- `COREMARK_SEED3=0x66`
- `COREMARK_EXECS_MASK=ID_LIST|ID_MATRIX|ID_STATE`
- `COREMARK_EXTRA_CFLAGS` unset

Optional run knobs:

    COREMARK_ITERATIONS=2 COREMARK_MAX_CYCLES=12000000 COREMARK_OPTS=O2 make test-coremark-smoke
    COREMARK_TOTAL_DATA_SIZE=600 COREMARK_OPTS=O2 make test-coremark-smoke
    COREMARK_OPTS=O2,Os make test-coremark-smoke
    COREMARK_LIST_ITEMS=1 COREMARK_SEED3=1 COREMARK_EXECS_MASK=ID_LIST COREMARK_MAX_CYCLES=500000 make test-coremark-smoke
    COREMARK_EXTRA_CFLAGS='-fno-jump-tables' make test-coremark-smoke

## Long Shakeout Campaigns

For long unattended local shakeout runs, use:

    make test-qemu-diff-campaign
    make test-software-torture
    make test-shakeout

These commands write per-run logs and a `summary.json` to
`build/shakeout/<timestamp>/`.
`test-software-torture` now fails hard if any CoreMark optimization row reports
status other than `ok` (for example, `timeout` or `trap:*`) for required
optimization levels. By default, required levels are all values in
`SHAKEOUT_COREMARK_OPTS`. Override with `SHAKEOUT_COREMARK_REQUIRED_OPTS`.

Default campaign scope:

- QEMU differential campaign: `SHAKEOUT_QEMU_SEEDS=1-300`, `SHAKEOUT_QEMU_OPS=128`
- Software torture campaign:
  `SHAKEOUT_CSMITH_SEEDS=1,4-10,12-17,19-23,25-32,34-37,39-59`,
  `SHAKEOUT_CSMITH_JUMP_SEEDS=1,4,7,10,13`,
  `SHAKEOUT_CSMITH_MAX_CYCLES=12000000`,
  `SHAKEOUT_COREMARK_OPTS=O0,O1,O2,Os`,
  `SHAKEOUT_COREMARK_REQUIRED_OPTS=` (empty => all from `SHAKEOUT_COREMARK_OPTS`),
  `SHAKEOUT_COREMARK_MAX_CYCLES=5000000`,
  `SHAKEOUT_COREMARK_ITERATIONS=1`,
  `SHAKEOUT_COREMARK_TOTAL_DATA_SIZE=600`

Example override:

    SHAKEOUT_QEMU_SEEDS=1-1000 SHAKEOUT_QEMU_OPS=128 make test-qemu-diff-campaign
    SHAKEOUT_CSMITH_SEEDS=1-80 SHAKEOUT_COREMARK_ITERATIONS=2 make test-software-torture
    SHAKEOUT_COREMARK_REQUIRED_OPTS=O2,Os make test-software-torture

## Formal Smoke

    make formal-smoke

Checks:

- **data-register file and hazard tracking** — bound to the real
  `WF68K30L_DATA_REGISTERS`, writes enabled, proven by temporal induction.
- **MMU runtime request gating** and **MMU walk-delay state transitions** —
  these two harnesses re-implement the gating equations rather than
  instantiating the RTL, so they check the intended contract and would not
  catch an implementation divergence. Binding them to the real modules is
  pending; the files say so at the top.

`formal-smoke` depends on `formal-selftest`, which asserts false and requires
FAILED. That guard exists because the flow was previously producing vacuous
passes for everything: as of Yosys 0.62 an immediate `assert` becomes a
`$check` cell and `write_smt2` emits nothing for it, so the SMT2 files
contained zero assertions. `clk2fflogic; chformal -lower` fixes it.

    make formal-deep

Opt-in exhaustive register-file property (symbolic register index plus a 32-bit
shadow). Kept separate because the UNSAT direction did not complete at depth 10
under yices, boolector or bitwuzla, while counterexamples are still found in
seconds.

## MMU Random Campaign

In addition to directed MMU instruction tests, a randomized descriptor campaign
is available:

    make test-mmu-random

This campaign randomizes descriptor format (short/long), FCL on/off, bottom-level
indirection, and selected fault conditions, then checks that the RTL either:

- returns the expected translated data value, or
- vectors through the MMU fault path (vector 56 marker)

This suite is also included in `test-full`.

## Jump-Table Repro Tests

Focused cocotb reproductions for switch/jump-table control flow:

    make test-jump-tables

This suite uses compiler-style `MOVE.W table(PC,Dn*scale)` + `JMP table(PC,Dn)`
patterns, including a nested `JSR/RTS` variant.
It is also included in `test-full`.

---

## Original README

> Historical, and describing the **pre-fork** core: the MMU, caches and
> coprocessor surface it says are absent have since been implemented, and the
> RTE/SSW limitation it notes has been addressed. Kept verbatim for provenance.
> See the top of this file and `notes/AUDIT_2026-07.md` for current behaviour.


This is the top level structural design unit of the 68K30L complex instruction set (CISC) microcontroller. It's programming model is (hopefully) fully compatible with Motorola's MC68030. This core features a pipelined architecture. In comparision to the fully featured 68K30 the core has no MMU, no data and instruction cache and no coprocessor interface. This results in missing burstmodes which are not required due to lack of cache. Missing coprocessor operations are: cpBcc, cpDBcc, cpGEN, cpRESTORE, cpSAVE, cpScc, cpTRAPcc. Missing MMU operations are: PFLUSH, PLOAD, PMOVE and PTEST. The trap handler does not process the following exceptions which lack due to the missing MMU and coprocessor interface: PRE_EXC_CP, MID_EXC_CP, POST_EXC_CP, EXC_VECT_CP, MMU_CFG_ERR. The shifter in the 68K30 is a barrel shifter and in this core it is a conventional shift register controlled logic. This core features the loop operation mode of the 68010 to deal with DBcc loops. This feature is a predecessor to the MC68020/30/40 caches. The exception handler works for the RTE but without taking the SSW into account which is intended to restore from a defectice bus error stack frame.

Enjoy.

Author(s):
- Wolfgang Foerster, wf@experiment-s.de; wf@inventronik.de
