# WF68K30L — SystemVerilog Port

This is a SystemVerilog port of the WF68K30L MC68030-compatible CPU core,
originally written in VHDL by Wolfgang Foerster.

## Branches

- **main** — Original VHDL with minor fixes for GHDL/Yosys synthesis compatibility
- **verilog** — Full SystemVerilog port, synthesizable with Yosys

## Synthesis Results (ECP5 via `synth_ecp5`)

| Resource   | VHDL   | SystemVerilog | Change |
|------------|--------|---------------|--------|
| LUT4       | 26,902 | 19,205        | -29%   |
| TRELLIS_FF |  2,542 |  2,534        | -0.3%  |
| CCU2C      |  3,039 |  1,640        | -46%   |
| PFUMX      |  6,274 |  4,598        | -27%   |
| L6MUX21    |  2,127 |  1,699        | -20%   |
| MULT18X18D |      4 |      4        | same   |

The SV port fits in an LFE5U-25F (~77% utilization) or larger. LUT reduction
comes primarily from replacing VHDL for-loops with direct equality comparisons
and bitwise mask operations, plus eliminating `async2sync` overhead.

Build commands (repo flow):

    ./run_ecp5_representative.sh
    USE_ABC9=0 ./run_ecp5_representative.sh

Direct Yosys command (default ABC9 mapping):

    yosys -p 'read_verilog -sv wf68k30L_*.sv; synth_ecp5 -top WF68K30L_TOP'

## Representative P&R Baseline (ECP5, 25 MHz)

Current repo defaults:

- `run_ecp5_representative.sh` uses **ABC9 by default** (`USE_ABC9=1`).
- `wf68K30L.sdc` intentionally keeps only the primary clock constraint; this
  avoids unsupported false-path directives in `nextpnr-ecp5`.

Baseline seed sweep command:

    ./run_ecp5_seed_sweep.sh

Representative baseline from 10 seeds (`build/rep_ecp5_seed_sweep_task3/results.csv`):

| Metric | Value |
|--------|-------|
| Fmax min/med/max | 16.131 / 16.481 / 16.898 MHz |
| TRELLIS_COMB | 18,689 (all 10 seeds) |
| TRELLIS_FF | 2,522 (all 10 seeds) |

## Equivalence Validation

The `validation/` directory contains a co-simulation harness that compares
the VHDL and SystemVerilog designs cycle-by-cycle. Both are synthesized to
gate-level Verilog netlists via Yosys (VHDL through the GHDL plugin, SV
natively), then driven with identical randomized bus stimulus in iverilog.

Result: 50,000 cycles across 5 random seeds, 0 output mismatches.

**Important:** This validates equivalence to the original VHDL — it does not
guarantee correctness to the MC68030 specification. The original design has
not been verified against the Motorola programmer's manual or any known-good
68030 emulator.

Run with:

    GHDL_PREFIX=/path/to/oss-cad-suite/lib/ghdl ./validation/run_equiv.sh

See [NOTES.md](NOTES.md) for detailed build requirements, compatibility
changes, and technical notes.

---

## Original README

This is the top level structural design unit of the 68K30L complex instruction set (CISC) microcontroller. It's programming model is (hopefully) fully compatible with Motorola's MC68030. This core features a pipelined architecture. In comparision to the fully featured 68K30 the core has no MMU, no data and instruction cache and no coprocessor interface. This results in missing burstmodes which are not required due to lack of cache. Missing coprocessor operations are: cpBcc, cpDBcc, cpGEN, cpRESTORE, cpSAVE, cpScc, cpTRAPcc. Missing MMU operations are: PFLUSH, PLOAD, PMOVE and PTEST. The trap handler does not process the following exceptions which lack due to the missing MMU and coprocessor interface: PRE_EXC_CP, MID_EXC_CP, POST_EXC_CP, EXC_VECT_CP, MMU_CFG_ERR. The shifter in the 68K30 is a barrel shifter and in this core it is a conventional shift register controlled logic. This core features the loop operation mode of the 68010 to deal with DBcc loops. This feature is a predecessor to the MC68020/30/40 caches. The exception handler works for the RTE but without taking the SSW into account which is intended to restore from a defectice bus error stack frame.

Enjoy.

Author(s):
- Wolfgang Foerster, wf@experiment-s.de; wf@inventronik.de
