# NOTES — Yosys/GHDL Bringup

## Build Requirements

- **GHDL** and **Yosys** with the GHDL plugin (oss-cad-suite provides both)
- `GHDL_PREFIX` must point to the GHDL library directory (e.g. `<oss-cad-suite>/lib/ghdl`)
- GHDL flags: `--ieee=synopsys` (design uses `ieee.std_logic_unsigned`) and `-fexplicit` (resolves operator overload ambiguity between IEEE and Synopsys `=`/`/=` operators)

## Changes Made for GHDL/Yosys Compatibility

### 1. Port mode mismatches — `wf68k30L_pkg.vhd`

Six ports were declared as `buffer` in the entity definitions but `out` in the
component declarations in the package. GHDL (correctly per VHDL-93) rejects this.
Changed the component declarations to `buffer` to match the entities:

- `ALU_REQ` in `WF68K30L_ALU`
- `AERR` in `WF68K30L_BUS_INTERFACE`
- `EW_ACK`, `PC_EW_OFFSET`, `OP`, `BIW_0` in `WF68K30L_OPCODE_DECODER`

### 2. Vector bounds mismatch — `wf68k30L_bus_interface.vhd`

Line 271: `To_StdLogicVector()` on a concatenated `bit_vector` returns an
ascending-range result (`0 to 2`) which GHDL synthesis cannot concatenate with
descending-range `std_logic_vector` operands. Replaced the single-line
concatenation with individual bit/slice assignments using `To_StdULogic()`.

### 3. Mixed clocked/combinational processes — `wf68k30L_control.vhd`, `wf68k30L_opcode_decoder.vhd`

GHDL synthesis does not support processes that assign signals both inside and
outside an `if CLK'event` block. Two processes were split:

- **`MOVEM_CONTROL`** in `wf68k30L_control.vhd` — clocked register logic stays
  in the original process; combinational outputs (`MOVEM_PNTR`, `MOVEM_ADn`,
  `MOVEM_ADn_I`, `MOVEM_COND`) moved to a new `MOVEM_COMB` process, connected
  via a new `MOVEM_PVAR_S` signal.

- **`P_PC_OFFSET`** in `wf68k30L_opcode_decoder.vhd` — clocked logic stays;
  combinational outputs (`PC_OFFSET`, `PC_ADR_OFFSET`) moved to a new
  `P_PC_OFFSET_COMB` process, connected via `ADR_OFFSET_S`, `PC_VAR_S`, and
  `PC_VAR_MEM_S` signals.

## Current Results

### Generic synthesis (`synth`)

Passes cleanly through GHDL import and Yosys generic synthesis.

### ECP5 synthesis (`synth_ecp5`)

Requires `-asyncprld` and `-noabc9` flags, plus an `async2sync` pass to handle
FFs with both async set and reset (ECP5 does not support `$_DFFSR_PPP_` natively;
the original design targeted ASICs).

Resource utilization on ECP5:

| Resource     | Count  |
|------------- |--------|
| LUT4         | 26,902 |
| TRELLIS_FF   |  2,542 |
| CCU2C        |  3,039 |
| PFUMX        |  6,274 |
| L6MUX21      |  2,127 |
| MULT18X18D   |      4 |

Fits in an LFE5U-45F (~60% utilization) or larger.
