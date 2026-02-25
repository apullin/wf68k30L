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

## SystemVerilog Port — Yosys Compatibility Changes

### 1. Package include strategy

Yosys does not support SV `import` statements. Enum typedefs are placed in
`wf68k30L_pkg.svh` (no package wrapper, no include guards) and included inside
each module body after the port list. This avoids enum namespace collisions with
port names (`RESET`) and ECP5 library names (`LSR`).

### 2. Port declarations use raw logic types

Yosys does not support custom enum types in port declarations. All ports use
`logic [N:0]` instead of `OP_SIZETYPE`, `OP_68K`, `TRAPTYPE_OPC`.

### 3. Enum rename — `RESET` to `OP_RESET`

The `RESET` opcode enum value collides with the `RESET` input port name when
both are in the same namespace. Renamed to `OP_RESET` in the package and all
references (4 sites in control.sv and opcode_decoder.sv).

### 4. Integer narrowing in ALU

VHDL `integer range 0 to N` declarations became plain `integer` (32-bit) in
the initial SV port. This caused Yosys to generate 4-billion-entry mux trees
for variable bit selects like `RESULT[MSB]`. Fixed by using appropriately-sized
`logic` vectors: `MSB` (5-bit), `BITPOS` (5-bit), `BF_LOWER_BND`/`BF_UPPER_BND`
(6-bit), `BF_WIDTH` (6-bit), `BITCNT` (7-bit).

### 5. For-loop elimination in ALU

Yosys `proc_mux` pass hangs on chained read-modify-write for-loops in
`always_comb` blocks (e.g. `TMP = TMP | result[i]` over 32-40 iterations).
Replaced with:
- Z-flag: direct `== 0` equality comparisons with `OP_SIZE` case select
- Bitfield ops (BFCHG/BFCLR/BFSET/BFINS): computed mask + bitwise ops
- Bitfield extract (BFEXTS/BFEXTU): shift + mask

### 6. Multi-edge sensitivity list in opcode decoder

`always @(posedge CLK or posedge OPCODE_RDY or posedge BUSY_EXH or negedge
IPIPE_FILL)` — Yosys rejects multiple edge-sensitive events. Converted to
synchronous `always_ff @(posedge CLK)` with priority encoding.

### 7. Latch prevention

Added `default` case for `NEXT_EXEC_WB_STATE` in control.sv EXEC_WB_DEC, and
`i = 0` initialization for loop variables in ALU `always_comb` blocks.

### 8. BF_BYTES lookup table

Yosys does not support `'{...}` array initialization syntax. Replaced the
`BF_BYTES_I` 2D lookup table with the formula `(offset + width + 7) >> 3`.

## Current Results

### VHDL — ECP5 synthesis via GHDL + Yosys (`synth_ecp5`)

Requires `-asyncprld` and `-noabc9` flags, plus an `async2sync` pass to handle
FFs with both async set and reset (ECP5 does not support `$_DFFSR_PPP_` natively;
the original design targeted ASICs).

| Resource     | Count  |
|------------- |--------|
| LUT4         | 26,902 |
| TRELLIS_FF   |  2,542 |
| CCU2C        |  3,039 |
| PFUMX        |  6,274 |
| L6MUX21      |  2,127 |
| MULT18X18D   |      4 |

### SystemVerilog — ECP5 synthesis via Yosys (`synth_ecp5`)

Build commands (repo flow):

    ./run_ecp5_representative.sh
    USE_ABC9=0 ./run_ecp5_representative.sh

Direct Yosys command (default ABC9 mapping):

    yosys -p 'read_verilog -sv wf68k30L_*.sv; synth_ecp5 -top WF68K30L_TOP'

No `-asyncprld` or `async2sync` needed (multi-edge process converted to
synchronous logic in the SV port).

| Resource     | Count  | vs VHDL |
|------------- |--------|---------|
| LUT4         | 19,205 |   -29%  |
| TRELLIS_FF   |  2,534 |   -0.3% |
| CCU2C        |  1,640 |   -46%  |
| PFUMX        |  4,598 |   -27%  |
| L6MUX21      |  1,699 |   -20%  |
| MULT18X18D   |      4 |    same |

LUT reduction primarily from replacing VHDL for-loops (32-40 iteration
conditional OR chains for Z-flag computation, bitfield mask operations)
with direct equality comparisons and bitwise mask operations in SV. Also
from eliminating async2sync overhead.

Fits in an LFE5U-25F (~77% utilization) or larger.
