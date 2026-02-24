# WF68K30L SystemVerilog Port — Bug List

Tracking bugs discovered during MC68030 compliance testing of the SV port.
VHDL baseline results are in [VHDL_BASELINE.md](VHDL_BASELINE.md).

**Test suite:** 537 tests across 12 modules (cocotb 2.0.1 + Verilator 5.044).
All 537 pass, with 9 known bugs masked by `expect_error` markers.

## Open Bugs

### BUG-001: Prefetch pipeline hazard with multi-word instructions

**Severity:** High
**Status:** Open (workaround in tests)
**Found:** Phase 1 smoke testing

**Description:**
After executing 2+ single-word instructions (e.g., MOVEQ, SWAP, CLR), a multi-word
instruction's extension words are fetched from the wrong address. The first extension
word address is fetched twice instead of advancing to the second extension word.

**Root cause (suspected):**
The registered `ADR_OFFSET_S` in the opcode decoder lags by one cycle when the bus
controller latches the fetch address. A combinatorial bypass (`adr_offset_fwd`) was
added to fix part of the problem, but the prefetch pipeline still missequences when
the pipeline is fully populated with single-word instructions.

**Reproduction:**
```asm
    MOVEQ  #1, D0          ; single-word
    MOVEQ  #2, D1          ; single-word
    MOVE.L D0, ($020000).L ; multi-word (3 words) — extension words garbled
```

**Workaround:**
Use register-indirect addressing instead of absolute long for store operations:
```asm
    MOVEA.L #$020000, A0   ; load address into A register
    MOVE.L  D0, (A0)       ; register-indirect store (single extension word)
```

**Affected tests:** All tests that store results to memory use the workaround pattern.

**Files involved:**
- `wf68k30L_opcode_decoder.sv` — PC offset bypass logic
- `wf68k30L_control.sv` — fetch pipeline state machine
- `wf68k30L_bus_interface.sv` — address latch timing

---

### BUG-002: Yosys ABC9 combinational loop assertion

**Severity:** Low (tooling)
**Status:** Open (workaround)
**Found:** Synthesis validation

**Description:**
Running `yosys synth_ecp5` with ABC9 optimization fails with:
```
Assert `no_loops' failed in `abc9_ops.exe'
```

**Workaround:**
Use `-noabc9` flag: `synth_ecp5 -noabc9 -top WF68K30L_TOP`

Synthesis completes successfully without ABC9. This is likely a Yosys issue with
the combinational logic structure rather than an actual hardware loop.

**Impact:** None on functionality. May miss some optimization opportunities.

---

### BUG-003: Division iterative loop returns incorrect results

**Severity:** High
**Status:** Open (expected failures in tests)
**Found:** Phase 3 muldiv testing

**Description:**
The SV divider (`wf68k30L_divider.sv`) produces incorrect results when the iterative
division loop (DIV_CALC state) executes. Fast-path cases in DIV_INIT work correctly:
- Divisor > dividend → quotient 0 (correct)
- Divisor == dividend → quotient 1 (correct)

Non-trivial divisions (e.g., 100/10, 0x7FFF/3) return quotient=0 and remainder=0.

**Root cause (suspected):**
The QUOTIENT/REMAINDER latching from ALU results has a timing mismatch. The ALU
result mux or OP_SIZE encoding during DIV_CALC iterations may not be propagating
correctly through the registered pipeline.

**Reproduction:**
```asm
    MOVE.L #100, D0     ; dividend
    DIVU.W #10, D0      ; should give quotient=10, remainder=0
    ; Result: D0 = 0x00000000 (incorrect)
```

**Affected tests:** 12 DIVU/DIVS tests marked with `expect_error=AssertionError` in
`test_instr_muldiv.py`. Multiply (MULS/MULU) works correctly.

**Files involved:**
- `wf68k30L_divider.sv` — DIV_CALC state machine, QUOTIENT/REMAINDER latch
- `wf68k30L_alu.sv` — Result mux during division

---

### BUG-004: MOVEM word mode and pre-decrement unreliable

**Severity:** Medium
**Status:** Open (workaround in tests)
**Found:** Phase 3 memory instruction testing

**Description:**
MOVEM (Move Multiple Registers) has issues with:
1. **Word mode (MOVEM.W):** Produces incorrect values when saving/restoring word-sized
   register lists. Long mode (MOVEM.L) works correctly.
2. **Pre-decrement mode `-(An)`:** Register values are garbled when using pre-decrement
   addressing. Post-increment `(An)+` and indirect `(An)` work correctly.
3. **Small register masks:** Masks with fewer than 3 registers may produce unreliable
   results.

**Workaround:**
Use MOVEM.L with `(An)` or `(An)+` addressing and 3+ registers in the mask.

**Affected tests:** `test_instr_memory.py` — all MOVEM tests use the workaround pattern.

**Files involved:**
- `wf68k30L_control.sv` — MOVEM state machine sequencing
- `wf68k30L_bus_interface.sv` — Word-size bus cycle generation

---

### BUG-005: CHK exception not generated for out-of-range values

**Severity:** Medium
**Status:** Open (tests only cover in-range cases)
**Found:** Phase 3 control instruction testing

**Description:**
CHK.W instruction does not reliably generate a CHK exception (vector 6) when the
register value is negative or exceeds the upper bound. In-range cases (where no
exception should occur) work correctly.

**Affected tests:** `test_instr_control.py` — CHK tests only verify in-range behavior.

**Files involved:**
- `wf68k30L_control.sv` — CHK trap generation
- `wf68k30L_exception_handler.sv` — Exception vector dispatch

---

### BUG-006: RTE does not return from exception handlers

**Severity:** High
**Status:** Open (expected failures in tests)
**Found:** Phase 5 exception testing

**Description:**
RTE (Return from Exception) does not properly restore PC and SR from the exception
stack frame. The CPU hangs instead of resuming execution at the return address.
This means exception handlers cannot return to normal program flow.

**Affected tests:** `test_exceptions.py` — `test_trap_rte_returns`, `test_trap_rte_restores_sr`
marked with `expect_error`.

**Files involved:**
- `wf68k30L_control.sv` — RTE state machine
- `wf68k30L_exception_handler.sv` — Stack frame pop logic

---

### BUG-007: TRAPV does not detect V flag set by MOVE to CCR

**Severity:** Medium
**Status:** Open (expected failure in tests)
**Found:** Phase 5 exception testing
**Note:** Inherited from original VHDL — not a port regression.

**Description:**
TRAPV instruction only detects the V (overflow) flag when set by ALU operations
(e.g., ADDI causing overflow). When V is loaded directly via MOVE to CCR, TRAPV
does not trigger the exception. Likely a pipeline forwarding issue where TRAPV
reads the pre-MOVE CCR value.

**Affected tests:** `test_exceptions.py` — `test_trapv_v_set_via_ccr` marked with
`expect_error`.

**Files involved:**
- `wf68k30L_control.sv` — TRAPV condition evaluation timing
- `wf68k30L_alu.sv` — CCR forwarding path

---

### BUG-009: Divide-by-zero clobbers destination register

**Severity:** Medium
**Status:** Open (expected failure in tests)
**Found:** Phase 5 exception testing

**Description:**
When DIVU.W or DIVS.W divides by zero, the MC68030 specification says the
destination register should be preserved (unchanged). The divider wrote
0xFFFFFFFF to QUOTIENT/REMAINDER; that was removed but the register still reads
back as 0x00000000 instead of the original value. The writeback path in the ALU
or control unit may still be zeroing the destination before the exception fires.

**Affected tests:** `test_exceptions.py` — `test_divu_divide_by_zero_preserves_dividend`
marked with `expect_error`.

**Files involved:**
- `wf68k30L_divider.sv` — Register writeback during divide-by-zero path
- `wf68k30L_alu.sv` — Result mux / writeback gating

---

## Resolved Bugs

### BUG-R005: ADD.L Dn,Dn does not set V for negative overflow (was BUG-008)

**Severity:** Medium
**Status:** Closed — not a bug
**Found:** Phase 5 exception testing

**Description:**
The ALU V-flag computation is correct and matches the VHDL. The test was
triggering **BUG-001** (prefetch pipeline hazard): `MOVEQ #-1,D3` (single-word)
followed by `ADD.L D2,D3` (single-word) caused the ADD to be misdecoded entirely.
The ALU never received an ADD operation.

**Fix:** Rewrote the test to use multi-word `MOVE.L #imm` instead of `MOVEQ`,
avoiding the prefetch hazard. Direct ALU unit tests confirm V=1 for negative
overflow (0x80000001 + 0x80000001 = 0x00000002).

---

### BUG-R004: Bus interface fails with 2+ wait states (was BUG-010)

**Severity:** Medium
**Status:** Fixed
**Found:** Phase 6 bus protocol testing

**Description:**
The RTL bus interface is correct and matches the VHDL. The bug was in the
cocotb **bus model** (`bus_model.py`): after responding to a bus cycle, the model
checked `ASn` on the next rising edge and saw it still low (ASn stays asserted
through S4 per MC68030 protocol). This caused the model to respond to the same
cycle a second time, overwriting correct data with stale values.

With 0-1 wait states the second response was fast enough that data hadn't changed,
masking the bug. With 2+ wait states the delay caused data corruption.

**Fix:** Added bus cycle boundary tracking: after driving DSACKn, the model waits
for ASn to deassert before accepting a new cycle.

---

### BUG-R003: Indexed addressing mode ignores index register

**Severity:** Critical
**Status:** Fixed (commit f1c0044)
**Found:** Phase 4 addressing mode testing

**Description:**
All indexed addressing modes `(d8,An,Xn)` and `(d8,PC,Xn)` computed effective addresses
without the index register contribution, effectively treating the index as zero. This
affected all scale factors (x1, x2, x4, x8) and both data and address register indexes.

**Root cause:**
The VHDL used **process variables** (`INDEX` and `INDEX_SCALED`) with immediate `:=`
assignment, meaning writes were visible to later reads within the same clock edge.
The SV port incorrectly converted these to **registered signals** (`INDEX_REG` and
`INDEX_SCALED_REG`) with deferred `<=` assignment, introducing a one-cycle latency.

**Fix:**
Extracted index computation into combinational `always_comb` blocks:
- `index_next` feeds `INDEX_REG` combinationally
- `index_scaled_comb` replaces `INDEX_SCALED_REG` in all effective address calculations
This matches the VHDL's variable-based same-cycle semantics.

**Files involved:**
- `wf68k30L_address_registers.sv` — INDEX/INDEX_SCALED computation

---

### BUG-R002: BITPOS truncation limits bit operations to positions 0-15

**Severity:** High
**Status:** Fixed (commit a33ac4e)
**Found:** Phase 3 bit instruction testing

**Description:**
Bit operations (BTST, BSET, BCLR, BCHG) on data registers only worked for bit
positions 0-15. Positions 16-31 were silently mapped to 0-15.

**Root cause:**
In `wf68k30L_alu.sv` line 123, the SV port incorrectly truncated BITPOS to 4 bits:
```sv
BITPOS <= {1'b0, BITPOS_IN[3:0]};  // BUG: truncates to 4 bits
```

**Fix:**
```sv
BITPOS <= BITPOS_IN;  // Full 5-bit position (0-31)
```

The original VHDL used the full width of `BITPOS_IN`. The SV port added an
unnecessary truncation.

---

### BUG-R001: CPU write bus cycles never generated

**Severity:** Critical
**Status:** Fixed (commits bb9e70e)
**Found:** Phase 1 smoke testing

**Description:**
Instructions fetched and decoded correctly, but MOVE.L to memory never produced
write bus cycles. Two root causes:

1. **Stale PC offset:** Registered `ADR_OFFSET_S` was one cycle behind when bus
   controller latched fetch address, causing duplicate fetches and preventing
   multi-word instructions from completing decode.

2. **Byte lane misalignment:** Bus model placed data on D[31:24]/D[31:16] regardless
   of address. MC68030 expects address-aligned byte lane placement.

**Fix:**
- Added combinatorial bypass `adr_offset_fwd` in opcode_decoder.sv
- Fixed byte lane computation in bus_model.py: `byte_lane = addr & 3`
