# WF68K30L SystemVerilog Port — Bug List

Tracking bugs discovered during MC68030 compliance testing of the SV port.
VHDL baseline results are in [VHDL_BASELINE.md](VHDL_BASELINE.md).

**Test suite:** cocotb + Verilator regression (see `tb/` for current modules/tests).
Known bugs are tracked below; unresolved behavior is covered with `expect_error`
tests where appropriate.

## Open Bugs

None currently.

---

## Resolved Bugs

### BUG-R013: Prefetch pipeline hazard with multi-word instructions (was BUG-001)

**Severity:** High
**Status:** Fixed
**Found:** Phase 1 smoke testing

**Description:**
After chains of single-word instructions, a following multi-word instruction could
misfetch extension words (for example duplicate fetch of the first extension word).

**Fix:**
- In `wf68k30L_opcode_decoder.sv`, prefetch address advancement is now gated by
  accepted opcode responses (`OPCODE_ACCEPT`) and uses combinational forward
  compensation (`pc_offset_comb`) so bus fetch addresses track same-cycle PC updates.

**Validation:**
- `test_instr_basic.py::test_prefetch_after_two_single_word_ops` passes.
- Added `test_instr_basic.py::test_prefetch_single_word_prefix_sweep`, which
  sweeps longer single-word prefixes and verifies opcode/extension fetch sequencing
  and result correctness; this passes.
- Full basic regression passes: `test_instr_basic` 12/12.

---

### BUG-R012: Yosys ABC9 combinational loop assertion (was BUG-002)

**Severity:** Low (tooling)
**Status:** Resolved by Yosys PR #5704 (open)
**Found:** Synthesis validation

**Description:**
The prior ABC9 crash:
```
Assert `no_loops' failed in passes/techmap/abc9_ops.cc:795.
```
is no longer reproducible with the updated local Yosys build.

**Validation:**
- Minimized repro (`abc9_bug/minimized/files_min.txt`) completes with
  `repro=False`, `retcode=0`.
- Full in-situ design repro (`abc9_bug/files_full.txt`) completes with
  `repro=False`, `retcode=0`.
- Legacy failing logs are retained in `abc9_bug/runs/` for comparison.

**Impact:** None on functionality. ABC9 flow is now the default for this repo's
ECP5 script (`run_ecp5_representative.sh`), with `USE_ABC9=0` fallback retained
while Yosys PR #5704 remains open.

---

### BUG-R011: MOVEM word mode and pre-decrement reliability (was BUG-004)

**Severity:** Medium
**Status:** Closed — behavior now correct
**Found:** Phase 3 memory instruction testing

**Description:**
The earlier MOVEM issue report is no longer reproducible on current RTL.
Targeted regressions now verify:
- `MOVEM.W` register-to-memory low-word storage.
- `MOVEM.W` memory-to-register sign extension (for both Dn and An destinations).
- `MOVEM.L` pre-decrement `-(An)` ordering and final address update.
- Small masks (for example, two-register masks) in both normal and pre-decrement forms.

**Validation:**
- Added/updated regressions in `tb/test_instr_memory.py`:
  - `test_movem_to_mem_two_regs` (now direct 2-register mask)
  - `test_movem_word_to_mem_two_regs`
  - `test_movem_word_from_mem_sign_extend`
  - `test_movem_predec_two_regs_order_and_final_a1`
  - `test_movem_word_from_mem_to_aregs_sign_extend`
  - `test_movem_predec_mixed_da_regs_order`
- Full memory regression passes: `test_instr_memory` 47/47.

---

### BUG-R010: DIVS/DIVU iterative divide produced incorrect results (was BUG-003)

**Severity:** High
**Status:** Fixed
**Found:** Phase 3 muldiv testing

**Description:**
The original issue appeared as broad iterative divide failure. After prefetch sequencing
fixes, remaining defects were in signed divide semantics:
- `DIVS` remainder kept absolute magnitude instead of using the dividend sign.
- `DIVS.W` overflow used unsigned thresholds, missing `+32768` and `-32769` cases.
- Fast paths (`|divisor| > |dividend|` and `|divisor| == |dividend|`) did not apply
  signed result sign rules.

This now matches MC68030 behavior: quotient truncates toward zero and remainder has
the dividend's sign.

**Fix:**
- In `wf68k30L_divider.sv`:
  - Latched signed-mode metadata at `DIV_INIT` (signed/unsigned mode, quotient sign,
    remainder sign, word/long limits).
  - Restored remainder sign for `DIVS`.
  - Applied signed overflow limits (`DIVS.W`: `+32767/-32768`, `DIVS.L`:
    `+2147483647/-2147483648`) and preserved destination on overflow.
  - Made fast-path quotient/remainder sign handling correct for signed divide.
- In `tb/test_instr_muldiv.py`, converted remaining `DIVU/DIVS` expected-failure tests
  to normal regressions.
- Full mul/div regression passes: `test_instr_muldiv` 33/33.

---

### BUG-R009: RTE restored corrupted PC from misaligned long read (was BUG-006)

**Severity:** High
**Status:** Fixed
**Found:** Phase 5 exception testing

**Description:**
`RTE` was restoring `PC` as `0xXXXXXXXX` with duplicated halves (for example,
`0x010A010A` instead of `0x0000010A`) when popping the exception frame. The
exception FSM was correct; corruption happened in the bus-interface input
alignment for `LONG_32` misaligned long reads (`ADR[1:0] != 2'b00`), which fed
bad `DATA_TO_CORE` into `PC_RESTORE`.

**Fix:**
- In `wf68k30L_bus_interface.sv`, corrected `LONG_32` long-read lane mapping for
  `ADR_10 = 2'b01/2'b10/2'b11` to assemble the upper bytes from
  `DATA_PORT_IN[31:8]`, `DATA_PORT_IN[31:16]`, and `DATA_PORT_IN[31:24]`.
- Converted `test_exceptions.py::test_trap_rte_returns` and
  `test_exceptions.py::test_trap_rte_restores_sr` from `expect_error` to normal
  regressions; both now pass.
- Full exception regression passes: `test_exceptions` 39/39.

---

### BUG-R008: CHK trapped against An instead of Dn (was BUG-005)

**Severity:** Medium
**Status:** Fixed
**Found:** Phase 3 control instruction testing

**Description:**
`CHK.W <ea>,Dn` must test `Dn` against bounds. The SV operand mux was incorrectly
feeding CHK's tested operand from `AR_OUT_2` (address register path) instead of
`DR_OUT_2` (data register path). This suppressed CHK traps when An did not match Dn.

**Fix:**
- In `wf68k30L_operand_mux.sv`, route `OP == CHK` operand 2 to `DR_OUT_2`.
- Keep CHK2/CMP2 dual-path behavior (`DR_OUT_2` vs `AR_OUT_2`) gated by `USE_DREG`.
- Converted `test_exceptions.py::test_chk_w_negative_traps` and
  `test_exceptions.py::test_chk_w_above_upper_traps` from `expect_error` to
  normal regressions; both now pass.

---

### BUG-R007: Divide-by-zero clobbered destination register (was BUG-009)

**Severity:** Medium
**Status:** Fixed
**Found:** Phase 5 exception testing

**Description:**
On `DIVU.W #0,Dn` / `DIVS.W #0,Dn`, destination registers must be preserved per
MC68030 behavior. The RTL was returning zeroed divide outputs on divide-by-zero,
which then propagated through writeback and overwrote `Dn`.

**Fix:**
- In `wf68k30L_divider.sv`, drive divide outputs to the pre-operation restore
  values on divide-by-zero (`QUOTIENT_REST` / `REMAINDER_REST`) so writeback
  preserves the architectural destination value.
- Converted `test_exceptions.py::test_divu_divide_by_zero_preserves_dividend`
  from `expect_error` to a normal regression.
- Added `test_exceptions.py::test_divs_divide_by_zero_preserves_dividend`.

---

### BUG-R006: TRAPV did not honor V set via MOVE to CCR (was BUG-007)

**Severity:** Medium
**Status:** Closed — behavior now correct
**Found:** Phase 5 exception testing

**Description:**
The previous `expect_error` marker for `TRAPV` after `MOVE ... ,CCR` is no longer
valid. With current RTL, `TRAPV` correctly traps when `CCR.V=1`, including when V
is set via `MOVE to CCR`.

**Fix:**
- Converted `test_exceptions.py::test_trapv_v_set_via_ccr` from
  `expect_error=AssertionError` to a normal regression test.
- Verified the test now passes as a standard (non-expected-failure) test.

---

### BUG-R005: ADD.L Dn,Dn does not set V for negative overflow (was BUG-008)

**Severity:** Medium
**Status:** Closed — not a bug
**Found:** Phase 5 exception testing

**Description:**
The ALU V-flag computation is correct and matches the VHDL. The test was
triggering the then-open prefetch hazard (now **BUG-R013**):
`MOVEQ #-1,D3` (single-word) followed by `ADD.L D2,D3` (single-word)
caused the ADD to be misdecoded entirely.
The ALU never received an ADD operation.

**Fix:** Rewrote the test to use multi-word `MOVE.L #imm` instead of `MOVEQ`,
avoiding that hazard. Direct ALU unit tests confirm V=1 for negative
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
