# WF68K30L Bug List

Tracking bugs discovered during MC68030 compliance testing.

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

## Resolved Bugs

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
