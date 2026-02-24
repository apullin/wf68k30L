"""
Shift and rotate instruction compliance tests for WF68K30L.

Tests: ASL, ASR, LSL, LSR, ROL, ROR, ROXL, ROXR with:
  - Immediate counts (#1-#8) and register counts (count in Dn)
  - Count=0 edge cases
  - Maximum count per size
  - Overflow detection (V flag for ASL)
  - X/C flag behavior from last bit shifted out
  - Byte, word, and long operand sizes

A consistent result-store pattern is applied in every test:
  - MOVEA.L #RESULT_BASE, A0 early
  - MOVE.L Dn, (A0) for stores
  - ADDQ.L #4, A0 to advance

IMPORTANT: When capturing CCR after an operation, the CCR must be read
BEFORE any MOVE.L Dn,(A0) store, because MOVE sets flags based on the
moved value. Use _store_ccr_and_advance() BEFORE _store_and_advance().
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles

from cpu_harness import CPUTestHarness
from m68k_encode import (
    BYTE, WORD, LONG,
    DN, AN, AN_IND, SPECIAL, IMMEDIATE,
    moveq, move, movea, nop, addq,
    asl, asr, lsl, lsr, rol, ror, roxl, roxr,
    move_from_ccr, move_to_ccr,
    imm_long, imm_word, imm_byte,
)


# ---- Helpers ---------------------------------------------------------------

def _setup_a0(h):
    """MOVEA.L #RESULT_BASE, A0."""
    return [*movea(LONG, SPECIAL, IMMEDIATE, 0), *imm_long(h.RESULT_BASE)]


def _store_and_advance(dn=0):
    """MOVE.L Dn,(A0) ; ADDQ.L #4,A0."""
    return [*move(LONG, DN, dn, AN_IND, 0), *addq(LONG, 4, AN, 0)]


def _store_ccr_and_advance():
    """MOVE CCR,D6 ; MOVE.L D6,(A0) ; ADDQ.L #4,A0."""
    return [*move_from_ccr(DN, 6), *move(LONG, DN, 6, AN_IND, 0), *addq(LONG, 4, AN, 0)]


# CCR bit positions
CCR_C = 0x01
CCR_V = 0x02
CCR_Z = 0x04
CCR_N = 0x08
CCR_X = 0x10


# ===== ASL (Arithmetic Shift Left) ==========================================

@cocotb.test()
async def test_asl_count0_long(dut):
    """ASL.L #0 (encoded as 8),D0 with count 0 in register -- no shift, X unchanged, V=0, C=0."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        *moveq(0, 1),               # D1 = 0 (count)
        *moveq(42, 0),              # D0 = 42
        *asl(LONG, 1, 0, ir=1),     # ASL.L D1, D0 (shift count from D1=0)
        *_store_ccr_and_advance(),   # CCR first (before MOVE.L clobbers flags)
        *_store_and_advance(),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    if not found:
        dut._log.warning("Sentinel not reached"); return
    ccr = h.read_result_long(0) & 0x1F
    assert h.read_result_long(4) == 42
    assert not (ccr & CCR_V), f"V should be clear for shift 0, CCR=0x{ccr:02X}"
    assert not (ccr & CCR_C), f"C should be clear for shift 0, CCR=0x{ccr:02X}"
    h.cleanup()


@cocotb.test()
async def test_asl_count1_long(dut):
    """ASL.L #1,D0: 1 << 1 = 2; C=0 (bit shifted out was 0)."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        *moveq(1, 0),
        *asl(LONG, 1, 0, ir=0),     # ASL.L #1, D0
        *_store_ccr_and_advance(),
        *_store_and_advance(),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    if not found:
        dut._log.warning("Sentinel not reached"); return
    ccr = h.read_result_long(0) & 0x1F
    assert h.read_result_long(4) == 2
    assert not (ccr & CCR_C), f"C should be 0, CCR=0x{ccr:02X}"
    h.cleanup()


@cocotb.test()
async def test_asl_count1_byte(dut):
    """ASL.B #1,D0: 0x40 << 1 = 0x80; C=0, V=1 (sign changed)."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        *moveq(0x40, 0),            # D0 = 0x40
        *asl(BYTE, 1, 0, ir=0),     # ASL.B #1, D0
        *_store_ccr_and_advance(),
        *_store_and_advance(),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    if not found:
        dut._log.warning("Sentinel not reached"); return
    ccr = h.read_result_long(0) & 0x1F
    result = h.read_result_long(4) & 0xFF
    assert result == 0x80, f"Expected 0x80, got 0x{result:02X}"
    assert ccr & CCR_N, f"N should be set (0x80 byte), CCR=0x{ccr:02X}"
    assert ccr & CCR_V, f"V should be set (sign changed 0->1), CCR=0x{ccr:02X}"
    h.cleanup()


@cocotb.test()
async def test_asl_count7_byte(dut):
    """ASL.B #7,D0: 1 << 7 = 0x80."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        *moveq(1, 0),
        *asl(BYTE, 7, 0, ir=0),     # ASL.B #7, D0
        *_store_and_advance(),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    if not found:
        dut._log.warning("Sentinel not reached"); return
    result = h.read_result_long(0) & 0xFF
    assert result == 0x80, f"Expected 0x80, got 0x{result:02X}"
    h.cleanup()


@cocotb.test()
async def test_asl_count8_byte(dut):
    """ASL.B #8,D0: 0xFF << 8 = 0x00 (all bits shifted out); Z=1, C=X=1."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        *moveq(-1, 0),              # D0 = 0xFF in low byte
        *asl(BYTE, 0, 0, ir=0),     # #8 is encoded as 0 in count field
        *_store_ccr_and_advance(),
        *_store_and_advance(),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    if not found:
        dut._log.warning("Sentinel not reached"); return
    ccr = h.read_result_long(0) & 0x1F
    result = h.read_result_long(4) & 0xFF
    assert result == 0x00, f"Expected 0x00, got 0x{result:02X}"
    assert ccr & CCR_Z, f"Z should be set, CCR=0x{ccr:02X}"
    h.cleanup()


@cocotb.test()
async def test_asl_overflow(dut):
    """ASL.L #1, D0 with 0x40000000 -> 0x80000000: V=1 (sign changed)."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0), *imm_long(0x40000000),
        *nop(),
        *asl(LONG, 1, 0, ir=0),
        *_store_ccr_and_advance(),
        *_store_and_advance(),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    if not found:
        dut._log.warning("Sentinel not reached"); return
    ccr = h.read_result_long(0) & 0x1F
    result = h.read_result_long(4)
    assert result == 0x80000000, f"Expected 0x80000000, got 0x{result:08X}"
    assert ccr & CCR_V, f"V should be set (sign change), CCR=0x{ccr:02X}"
    h.cleanup()


@cocotb.test()
async def test_asl_no_overflow(dut):
    """ASL.L #1, D0 with 0x20000000 -> 0x40000000: V=0 (no sign change)."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0), *imm_long(0x20000000),
        *nop(),
        *asl(LONG, 1, 0, ir=0),
        *_store_ccr_and_advance(),
        *_store_and_advance(),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    if not found:
        dut._log.warning("Sentinel not reached"); return
    ccr = h.read_result_long(0) & 0x1F
    result = h.read_result_long(4)
    assert result == 0x40000000, f"Expected 0x40000000, got 0x{result:08X}"
    assert not (ccr & CCR_V), f"V should be clear, CCR=0x{ccr:02X}"
    h.cleanup()


@cocotb.test()
async def test_asl_xc_from_last_bit(dut):
    """ASL.L #1, D0 with 0x80000001: last bit out is 1, so X=C=1."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0), *imm_long(0x80000001),
        *nop(),
        *asl(LONG, 1, 0, ir=0),     # 0x80000001 << 1 = 0x00000002
        *_store_ccr_and_advance(),
        *_store_and_advance(),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    if not found:
        dut._log.warning("Sentinel not reached"); return
    ccr = h.read_result_long(0) & 0x1F
    result = h.read_result_long(4)
    assert result == 0x00000002, f"Expected 0x00000002, got 0x{result:08X}"
    assert ccr & CCR_C, f"C should be set (MSB was 1), CCR=0x{ccr:02X}"
    assert ccr & CCR_X, f"X should be set (MSB was 1), CCR=0x{ccr:02X}"
    h.cleanup()


@cocotb.test()
async def test_asl_reg_count(dut):
    """ASL.L with register count: D1=4, D0=1 -> 1<<4=16."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        *moveq(4, 1),               # D1 = 4 (count)
        *moveq(1, 0),               # D0 = 1
        *asl(LONG, 1, 0, ir=1),     # ASL.L D1, D0
        *_store_and_advance(),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    if not found:
        dut._log.warning("Sentinel not reached"); return
    assert h.read_result_long(0) == 16
    h.cleanup()


# ===== ASR (Arithmetic Shift Right) =========================================

@cocotb.test()
async def test_asr_count0_long(dut):
    """ASR.L with count=0 in register: no change, C=0."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        *moveq(0, 1),               # D1 = 0
        *moveq(42, 0),
        *asr(LONG, 1, 0, ir=1),
        *_store_ccr_and_advance(),
        *_store_and_advance(),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    if not found:
        dut._log.warning("Sentinel not reached"); return
    ccr = h.read_result_long(0) & 0x1F
    assert h.read_result_long(4) == 42
    assert not (ccr & CCR_C), f"C should be 0, CCR=0x{ccr:02X}"
    h.cleanup()


@cocotb.test()
async def test_asr_count1_long(dut):
    """ASR.L #1,D0: 4 >> 1 = 2."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        *moveq(4, 0),
        *asr(LONG, 1, 0, ir=0),
        *_store_and_advance(),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    if not found:
        dut._log.warning("Sentinel not reached"); return
    assert h.read_result_long(0) == 2
    h.cleanup()


@cocotb.test()
async def test_asr_preserves_sign(dut):
    """ASR.L #1,D0: 0x80000000 >> 1 = 0xC0000000 (sign bit preserved)."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0), *imm_long(0x80000000),
        *nop(),
        *asr(LONG, 1, 0, ir=0),
        *_store_ccr_and_advance(),
        *_store_and_advance(),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    if not found:
        dut._log.warning("Sentinel not reached"); return
    ccr = h.read_result_long(0) & 0x1F
    result = h.read_result_long(4)
    assert result == 0xC0000000, f"Expected 0xC0000000, got 0x{result:08X}"
    assert ccr & CCR_N, f"N should be set, CCR=0x{ccr:02X}"
    h.cleanup()


@cocotb.test()
async def test_asr_xc_last_bit(dut):
    """ASR.L #1,D0: 3 >> 1 = 1; last bit out = 1 -> C=X=1."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        *moveq(3, 0),
        *asr(LONG, 1, 0, ir=0),
        *_store_ccr_and_advance(),
        *_store_and_advance(),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    if not found:
        dut._log.warning("Sentinel not reached"); return
    ccr = h.read_result_long(0) & 0x1F
    assert h.read_result_long(4) == 1
    assert ccr & CCR_C, f"C should be set (bit 0 was 1), CCR=0x{ccr:02X}"
    assert ccr & CCR_X, f"X should be set, CCR=0x{ccr:02X}"
    h.cleanup()


@cocotb.test()
async def test_asr_negative_fills_ones(dut):
    """ASR.B #4, D0: 0x80 >> 4 = 0xF8 (sign-extends)."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0), *imm_long(0x00000080),
        *nop(),
        *asr(BYTE, 4, 0, ir=0),     # ASR.B #4, D0
        *_store_and_advance(),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    if not found:
        dut._log.warning("Sentinel not reached"); return
    result = h.read_result_long(0) & 0xFF
    assert result == 0xF8, f"Expected 0xF8, got 0x{result:02X}"
    h.cleanup()


@cocotb.test()
async def test_asr_reg_count(dut):
    """ASR.L with register count: D1=3, D0=64 -> 64>>3=8."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        *moveq(3, 1),
        *moveq(64, 0),
        *asr(LONG, 1, 0, ir=1),
        *_store_and_advance(),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    if not found:
        dut._log.warning("Sentinel not reached"); return
    assert h.read_result_long(0) == 8
    h.cleanup()


# ===== LSL (Logical Shift Left) =============================================

@cocotb.test()
async def test_lsl_count1_long(dut):
    """LSL.L #1,D0: 1 << 1 = 2; V always 0 for LSL."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        *moveq(1, 0),
        *lsl(LONG, 1, 0, ir=0),
        *_store_ccr_and_advance(),
        *_store_and_advance(),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    if not found:
        dut._log.warning("Sentinel not reached"); return
    ccr = h.read_result_long(0) & 0x1F
    assert h.read_result_long(4) == 2
    assert not (ccr & CCR_V), f"V should always be 0 for LSL, CCR=0x{ccr:02X}"
    h.cleanup()


@cocotb.test()
async def test_lsl_count0_long(dut):
    """LSL.L with count=0 (register): no shift, C=0."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        *moveq(0, 1),
        *moveq(99, 0),
        *lsl(LONG, 1, 0, ir=1),
        *_store_ccr_and_advance(),
        *_store_and_advance(),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    if not found:
        dut._log.warning("Sentinel not reached"); return
    ccr = h.read_result_long(0) & 0x1F
    assert h.read_result_long(4) == 99
    assert not (ccr & CCR_C), f"C should be 0 for shift 0, CCR=0x{ccr:02X}"
    h.cleanup()


@cocotb.test()
async def test_lsl_count8_byte(dut):
    """LSL.B #8,D0: 0xFF << 8 = 0x00; Z=1, C=X=1."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        *moveq(-1, 0),
        *lsl(BYTE, 0, 0, ir=0),     # #8 encoded as 0
        *_store_ccr_and_advance(),
        *_store_and_advance(),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    if not found:
        dut._log.warning("Sentinel not reached"); return
    ccr = h.read_result_long(0) & 0x1F
    result = h.read_result_long(4) & 0xFF
    assert result == 0, f"Expected 0x00, got 0x{result:02X}"
    assert ccr & CCR_Z, f"Z should be set, CCR=0x{ccr:02X}"
    h.cleanup()


@cocotb.test()
async def test_lsl_word_count3(dut):
    """LSL.W #3,D0: 0x0001 << 3 = 0x0008."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        *moveq(1, 0),
        *lsl(WORD, 3, 0, ir=0),
        *_store_and_advance(),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    if not found:
        dut._log.warning("Sentinel not reached"); return
    result = h.read_result_long(0) & 0xFFFF
    assert result == 8, f"Expected 8, got 0x{result:04X}"
    h.cleanup()


@cocotb.test()
async def test_lsl_carry_out(dut):
    """LSL.L #1, D0: 0x80000000 << 1 = 0; C=X=1."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0), *imm_long(0x80000000),
        *nop(),
        *lsl(LONG, 1, 0, ir=0),
        *_store_ccr_and_advance(),
        *_store_and_advance(),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    if not found:
        dut._log.warning("Sentinel not reached"); return
    ccr = h.read_result_long(0) & 0x1F
    assert h.read_result_long(4) == 0
    assert ccr & CCR_C, f"C should be set, CCR=0x{ccr:02X}"
    assert ccr & CCR_X, f"X should be set, CCR=0x{ccr:02X}"
    h.cleanup()


@cocotb.test()
async def test_lsl_reg_count(dut):
    """LSL.L D1,D0: D1=5, D0=1 -> 1<<5=32."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        *moveq(5, 1),
        *moveq(1, 0),
        *lsl(LONG, 1, 0, ir=1),
        *_store_and_advance(),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    if not found:
        dut._log.warning("Sentinel not reached"); return
    assert h.read_result_long(0) == 32
    h.cleanup()


# ===== LSR (Logical Shift Right) ============================================

@cocotb.test()
async def test_lsr_count1_long(dut):
    """LSR.L #1,D0: 4 >> 1 = 2; zero fills from left."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        *moveq(4, 0),
        *lsr(LONG, 1, 0, ir=0),
        *_store_and_advance(),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    if not found:
        dut._log.warning("Sentinel not reached"); return
    assert h.read_result_long(0) == 2
    h.cleanup()


@cocotb.test()
async def test_lsr_zero_fill(dut):
    """LSR.L #1,D0: 0x80000000 >> 1 = 0x40000000 (zero filled, not sign-extended)."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0), *imm_long(0x80000000),
        *nop(),
        *lsr(LONG, 1, 0, ir=0),
        *_store_and_advance(),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    if not found:
        dut._log.warning("Sentinel not reached"); return
    result = h.read_result_long(0)
    assert result == 0x40000000, f"Expected 0x40000000, got 0x{result:08X}"
    h.cleanup()


@cocotb.test()
async def test_lsr_count0(dut):
    """LSR.L with count=0 (register): no shift, V=0, C=0."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        *moveq(0, 1),
        *moveq(55, 0),
        *lsr(LONG, 1, 0, ir=1),
        *_store_ccr_and_advance(),
        *_store_and_advance(),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    if not found:
        dut._log.warning("Sentinel not reached"); return
    ccr = h.read_result_long(0) & 0x1F
    assert h.read_result_long(4) == 55
    assert not (ccr & CCR_V), f"V should be 0, CCR=0x{ccr:02X}"
    assert not (ccr & CCR_C), f"C should be 0, CCR=0x{ccr:02X}"
    h.cleanup()


@cocotb.test()
async def test_lsr_carry(dut):
    """LSR.L #1,D0: 3 >> 1 = 1; C=X=1 (bit 0 was 1)."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        *moveq(3, 0),
        *lsr(LONG, 1, 0, ir=0),
        *_store_ccr_and_advance(),
        *_store_and_advance(),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    if not found:
        dut._log.warning("Sentinel not reached"); return
    ccr = h.read_result_long(0) & 0x1F
    assert h.read_result_long(4) == 1
    assert ccr & CCR_C, f"C should be set, CCR=0x{ccr:02X}"
    assert ccr & CCR_X, f"X should be set, CCR=0x{ccr:02X}"
    h.cleanup()


@cocotb.test()
async def test_lsr_v_always_zero(dut):
    """LSR always clears V, even for large shifts."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0), *imm_long(0xFFFFFFFF),
        *nop(),
        *lsr(LONG, 1, 0, ir=0),
        *_store_ccr_and_advance(),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    if not found:
        dut._log.warning("Sentinel not reached"); return
    ccr = h.read_result_long(0) & 0x1F
    assert not (ccr & CCR_V), f"V should always be 0 for LSR, CCR=0x{ccr:02X}"
    h.cleanup()


@cocotb.test()
async def test_lsr_byte_count4(dut):
    """LSR.B #4,D0: 0xF0 >> 4 = 0x0F."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0), *imm_long(0x000000F0),
        *nop(),
        *lsr(BYTE, 4, 0, ir=0),
        *_store_and_advance(),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    if not found:
        dut._log.warning("Sentinel not reached"); return
    result = h.read_result_long(0) & 0xFF
    assert result == 0x0F, f"Expected 0x0F, got 0x{result:02X}"
    h.cleanup()


@cocotb.test()
async def test_lsr_reg_count(dut):
    """LSR.L D1,D0: D1=4, D0=256 -> 256>>4=16."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        *moveq(4, 1),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0), *imm_long(256),
        *nop(),
        *lsr(LONG, 1, 0, ir=1),
        *_store_and_advance(),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    if not found:
        dut._log.warning("Sentinel not reached"); return
    assert h.read_result_long(0) == 16
    h.cleanup()


# ===== ROL (Rotate Left) ====================================================

@cocotb.test()
async def test_rol_count1_long(dut):
    """ROL.L #1,D0: 0x80000001 -> 0x00000003; C=1 (MSB rotated out)."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0), *imm_long(0x80000001),
        *nop(),
        *rol(LONG, 1, 0, ir=0),
        *_store_ccr_and_advance(),
        *_store_and_advance(),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    if not found:
        dut._log.warning("Sentinel not reached"); return
    ccr = h.read_result_long(0) & 0x1F
    result = h.read_result_long(4)
    assert result == 0x00000003, f"Expected 0x00000003, got 0x{result:08X}"
    assert ccr & CCR_C, f"C should be set (MSB was 1), CCR=0x{ccr:02X}"
    h.cleanup()


@cocotb.test()
async def test_rol_count0_long(dut):
    """ROL.L count=0 (register): no change, C=0."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        *moveq(0, 1),
        *moveq(77, 0),
        *rol(LONG, 1, 0, ir=1),
        *_store_ccr_and_advance(),
        *_store_and_advance(),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    if not found:
        dut._log.warning("Sentinel not reached"); return
    ccr = h.read_result_long(0) & 0x1F
    assert h.read_result_long(4) == 77
    assert not (ccr & CCR_C), f"C should be 0 for count 0, CCR=0x{ccr:02X}"
    h.cleanup()


@cocotb.test()
async def test_rol_x_not_affected(dut):
    """ROL does not change X flag. Set X first via ASL, then ROL should keep it."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0), *imm_long(0x80000000),
        *nop(),
        *asl(LONG, 1, 0, ir=0),     # shifts out MSB=1, X=1
        # D0 = 0, now load a fresh value and ROL
        *moveq(1, 0),
        *rol(LONG, 1, 0, ir=0),     # ROL #1, D0: 1->2, C=0
        *_store_ccr_and_advance(),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    if not found:
        dut._log.warning("Sentinel not reached"); return
    ccr = h.read_result_long(0) & 0x1F
    # X should still be 1 from the ASL
    assert ccr & CCR_X, f"X should remain set (ROL does not affect X), CCR=0x{ccr:02X}"
    assert not (ccr & CCR_C), f"C should be 0 (bit 31 was 0), CCR=0x{ccr:02X}"
    h.cleanup()


@cocotb.test()
async def test_rol_byte(dut):
    """ROL.B #1,D0: 0x81 -> 0x03; C=1."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0), *imm_long(0x00000081),
        *nop(),
        *rol(BYTE, 1, 0, ir=0),
        *_store_ccr_and_advance(),
        *_store_and_advance(),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    if not found:
        dut._log.warning("Sentinel not reached"); return
    ccr = h.read_result_long(0) & 0x1F
    result = h.read_result_long(4) & 0xFF
    assert result == 0x03, f"Expected 0x03, got 0x{result:02X}"
    assert ccr & CCR_C, f"C should be set, CCR=0x{ccr:02X}"
    h.cleanup()


@cocotb.test()
async def test_rol_reg_count(dut):
    """ROL.L D1,D0: D1=4, D0=0x10000001 -> 0x00000011."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        *moveq(4, 1),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0), *imm_long(0x10000001),
        *nop(),
        *rol(LONG, 1, 0, ir=1),
        *_store_and_advance(),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    if not found:
        dut._log.warning("Sentinel not reached"); return
    result = h.read_result_long(0)
    assert result == 0x00000011, f"Expected 0x00000011, got 0x{result:08X}"
    h.cleanup()


# ===== ROR (Rotate Right) ===================================================

@cocotb.test()
async def test_ror_count1_long(dut):
    """ROR.L #1,D0: 0x00000001 -> 0x80000000; C=1 (bit 0 rotated)."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        *moveq(1, 0),
        *ror(LONG, 1, 0, ir=0),
        *_store_ccr_and_advance(),
        *_store_and_advance(),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    if not found:
        dut._log.warning("Sentinel not reached"); return
    ccr = h.read_result_long(0) & 0x1F
    result = h.read_result_long(4)
    assert result == 0x80000000, f"Expected 0x80000000, got 0x{result:08X}"
    assert ccr & CCR_C, f"C should be set, CCR=0x{ccr:02X}"
    h.cleanup()


@cocotb.test()
async def test_ror_count0_long(dut):
    """ROR.L count=0 (register): no change, C=0."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        *moveq(0, 1),
        *moveq(42, 0),
        *ror(LONG, 1, 0, ir=1),
        *_store_ccr_and_advance(),
        *_store_and_advance(),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    if not found:
        dut._log.warning("Sentinel not reached"); return
    ccr = h.read_result_long(0) & 0x1F
    assert h.read_result_long(4) == 42
    assert not (ccr & CCR_C), f"C should be 0, CCR=0x{ccr:02X}"
    h.cleanup()


@cocotb.test()
async def test_ror_byte(dut):
    """ROR.B #1,D0: 0x01 -> 0x80; C=1."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        *moveq(1, 0),
        *ror(BYTE, 1, 0, ir=0),
        *_store_ccr_and_advance(),
        *_store_and_advance(),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    if not found:
        dut._log.warning("Sentinel not reached"); return
    ccr = h.read_result_long(0) & 0x1F
    result = h.read_result_long(4) & 0xFF
    assert result == 0x80, f"Expected 0x80, got 0x{result:02X}"
    assert ccr & CCR_C, f"C should be set, CCR=0x{ccr:02X}"
    h.cleanup()


@cocotb.test()
async def test_ror_x_not_affected(dut):
    """ROR does not change X. Set X with a shift first."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        *moveq(1, 0),
        *asr(LONG, 1, 0, ir=0),     # 1>>1=0, bit 0 was 1 -> X=1
        *moveq(4, 0),
        *ror(LONG, 1, 0, ir=0),     # ROR #1: 4 -> 2, C=0
        *_store_ccr_and_advance(),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    if not found:
        dut._log.warning("Sentinel not reached"); return
    ccr = h.read_result_long(0) & 0x1F
    assert ccr & CCR_X, f"X should still be set, CCR=0x{ccr:02X}"
    assert not (ccr & CCR_C), f"C should be 0, CCR=0x{ccr:02X}"
    h.cleanup()


@cocotb.test()
async def test_ror_reg_count(dut):
    """ROR.L D1,D0: D1=8, D0=0xFF -> 0xFF000000."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        *moveq(8, 1),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0), *imm_long(0x000000FF),
        *nop(),
        *ror(LONG, 1, 0, ir=1),
        *_store_and_advance(),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    if not found:
        dut._log.warning("Sentinel not reached"); return
    result = h.read_result_long(0)
    assert result == 0xFF000000, f"Expected 0xFF000000, got 0x{result:08X}"
    h.cleanup()


# ===== ROXL (Rotate Left through eXtend) ====================================

@cocotb.test()
async def test_roxl_count1_long(dut):
    """ROXL.L #1,D0: with X=0, 0x80000000 -> 0x00000000, C=X=1."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        # Clear X: ASL.L #1 on 0 shifts out a 0 bit -> X=0
        *moveq(0, 0),
        *asl(LONG, 1, 0, ir=0),     # 0 << 1 = 0, last bit out = 0, X=0
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0), *imm_long(0x80000000),
        *nop(),
        *roxl(LONG, 1, 0, ir=0),    # ROXL.L #1, D0
        *_store_ccr_and_advance(),
        *_store_and_advance(),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    if not found:
        dut._log.warning("Sentinel not reached"); return
    ccr = h.read_result_long(0) & 0x1F
    result = h.read_result_long(4)
    assert result == 0x00000000, f"Expected 0x00000000, got 0x{result:08X}"
    assert ccr & CCR_C, f"C should be set, CCR=0x{ccr:02X}"
    assert ccr & CCR_X, f"X should be set, CCR=0x{ccr:02X}"
    h.cleanup()


@cocotb.test()
async def test_roxl_count0_c_equals_x(dut):
    """ROXL.L count=0 (register): C = X (per spec). Set X=1 first."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        # Set X=1 via ASL that shifts out a 1
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0), *imm_long(0x80000000),
        *nop(),
        *asl(LONG, 1, 0, ir=0),     # X=1
        *moveq(42, 0),              # D0 = 42
        *moveq(0, 1),               # count = 0
        *roxl(LONG, 1, 0, ir=1),    # ROXL.L D1(=0), D0
        *_store_ccr_and_advance(),
        *_store_and_advance(),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    if not found:
        dut._log.warning("Sentinel not reached"); return
    ccr = h.read_result_long(0) & 0x1F
    assert h.read_result_long(4) == 42  # unchanged
    # X was set to 1, and with count=0, C should equal X
    assert ccr & CCR_X, f"X should be set, CCR=0x{ccr:02X}"
    assert ccr & CCR_C, f"C should equal X=1 for count 0, CCR=0x{ccr:02X}"
    h.cleanup()


@cocotb.test()
async def test_roxl_x_participates(dut):
    """ROXL.L #1 with X=1: X bit rotates into bit 0."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        # Set X=1
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0), *imm_long(0x80000000),
        *nop(),
        *asl(LONG, 1, 0, ir=0),     # X=1, D0=0
        *moveq(0, 0),               # D0 = 0
        *roxl(LONG, 1, 0, ir=0),    # ROXL #1: 0 with X=1 -> bit 0 = 1 = 0x00000001
        *_store_and_advance(),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    if not found:
        dut._log.warning("Sentinel not reached"); return
    result = h.read_result_long(0)
    assert result == 0x00000001, f"Expected 0x00000001, got 0x{result:08X}"
    h.cleanup()


# ===== ROXR (Rotate Right through eXtend) ===================================

@cocotb.test()
async def test_roxr_count1_long(dut):
    """ROXR.L #1,D0: with X=0, 0x00000001 -> 0x00000000, C=X=1."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        # Clear X: ASL.L #1 on 0 shifts out a 0 bit -> X=0
        *moveq(0, 0),
        *asl(LONG, 1, 0, ir=0),     # 0 << 1 = 0, X=0
        *moveq(1, 0),
        *roxr(LONG, 1, 0, ir=0),
        *_store_ccr_and_advance(),
        *_store_and_advance(),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    if not found:
        dut._log.warning("Sentinel not reached"); return
    ccr = h.read_result_long(0) & 0x1F
    result = h.read_result_long(4)
    assert result == 0x00000000, f"Expected 0x00000000, got 0x{result:08X}"
    assert ccr & CCR_C, f"C should be set, CCR=0x{ccr:02X}"
    assert ccr & CCR_X, f"X should be set, CCR=0x{ccr:02X}"
    h.cleanup()


@cocotb.test()
async def test_roxr_count0_c_equals_x(dut):
    """ROXR.L count=0: C should equal X."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        # Set X=1
        *moveq(1, 0),
        *asr(LONG, 1, 0, ir=0),     # 1>>1=0, X=1
        *moveq(42, 0),
        *moveq(0, 1),
        *roxr(LONG, 1, 0, ir=1),    # count=0
        *_store_ccr_and_advance(),
        *_store_and_advance(),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    if not found:
        dut._log.warning("Sentinel not reached"); return
    ccr = h.read_result_long(0) & 0x1F
    assert h.read_result_long(4) == 42
    assert ccr & CCR_X, f"X should be set, CCR=0x{ccr:02X}"
    assert ccr & CCR_C, f"C should equal X for count 0, CCR=0x{ccr:02X}"
    h.cleanup()


@cocotb.test()
async def test_roxr_x_participates(dut):
    """ROXR.L #1 with X=1: X bit rotates into MSB."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        # Set X=1
        *moveq(1, 0),
        *asr(LONG, 1, 0, ir=0),     # X=1
        *moveq(0, 0),               # D0 = 0
        *roxr(LONG, 1, 0, ir=0),    # ROXR #1: 0 with X=1 -> MSB = 1 = 0x80000000
        *_store_and_advance(),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    if not found:
        dut._log.warning("Sentinel not reached"); return
    result = h.read_result_long(0)
    assert result == 0x80000000, f"Expected 0x80000000, got 0x{result:08X}"
    h.cleanup()


# ===== ASL/ASR Word sizes ===================================================

@cocotb.test()
async def test_asl_word_count1(dut):
    """ASL.W #1,D0: 0x4000 -> 0x8000; V=1 (sign change), N=1."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0), *imm_long(0x00004000),
        *nop(),
        *asl(WORD, 1, 0, ir=0),
        *_store_ccr_and_advance(),
        *_store_and_advance(),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    if not found:
        dut._log.warning("Sentinel not reached"); return
    ccr = h.read_result_long(0) & 0x1F
    result = h.read_result_long(4) & 0xFFFF
    assert result == 0x8000, f"Expected 0x8000, got 0x{result:04X}"
    assert ccr & CCR_V, f"V should be set, CCR=0x{ccr:02X}"
    assert ccr & CCR_N, f"N should be set, CCR=0x{ccr:02X}"
    h.cleanup()


@cocotb.test()
async def test_asr_word_count1(dut):
    """ASR.W #1,D0: 0x8000 -> 0xC000."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0), *imm_long(0x00008000),
        *nop(),
        *asr(WORD, 1, 0, ir=0),
        *_store_and_advance(),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    if not found:
        dut._log.warning("Sentinel not reached"); return
    result = h.read_result_long(0) & 0xFFFF
    assert result == 0xC000, f"Expected 0xC000, got 0x{result:04X}"
    h.cleanup()


# ===== Various immediate counts =============================================

@cocotb.test()
async def test_lsl_imm2_long(dut):
    """LSL.L #2,D0: 1 << 2 = 4."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        *moveq(1, 0),
        *lsl(LONG, 2, 0, ir=0),
        *_store_and_advance(),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    if not found:
        dut._log.warning("Sentinel not reached"); return
    assert h.read_result_long(0) == 4
    h.cleanup()


@cocotb.test()
async def test_lsl_imm3_long(dut):
    """LSL.L #3,D0: 1 << 3 = 8."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        *moveq(1, 0),
        *lsl(LONG, 3, 0, ir=0),
        *_store_and_advance(),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    if not found:
        dut._log.warning("Sentinel not reached"); return
    assert h.read_result_long(0) == 8
    h.cleanup()


@cocotb.test()
async def test_lsl_imm4_long(dut):
    """LSL.L #4,D0: 1 << 4 = 16."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        *moveq(1, 0),
        *lsl(LONG, 4, 0, ir=0),
        *_store_and_advance(),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    if not found:
        dut._log.warning("Sentinel not reached"); return
    assert h.read_result_long(0) == 16
    h.cleanup()


@cocotb.test()
async def test_lsl_imm5_long(dut):
    """LSL.L #5,D0: 1 << 5 = 32."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        *moveq(1, 0),
        *lsl(LONG, 5, 0, ir=0),
        *_store_and_advance(),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    if not found:
        dut._log.warning("Sentinel not reached"); return
    assert h.read_result_long(0) == 32
    h.cleanup()


@cocotb.test()
async def test_lsl_imm6_long(dut):
    """LSL.L #6,D0: 1 << 6 = 64."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        *moveq(1, 0),
        *lsl(LONG, 6, 0, ir=0),
        *_store_and_advance(),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    if not found:
        dut._log.warning("Sentinel not reached"); return
    assert h.read_result_long(0) == 64
    h.cleanup()


@cocotb.test()
async def test_lsl_imm7_long(dut):
    """LSL.L #7,D0: 1 << 7 = 128."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        *moveq(1, 0),
        *lsl(LONG, 7, 0, ir=0),
        *_store_and_advance(),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    if not found:
        dut._log.warning("Sentinel not reached"); return
    assert h.read_result_long(0) == 128
    h.cleanup()


@cocotb.test()
async def test_lsl_imm8_long(dut):
    """LSL.L #8,D0: 1 << 8 = 256."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        *moveq(1, 0),
        *lsl(LONG, 0, 0, ir=0),     # #8 encoded as 0
        *_store_and_advance(),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    if not found:
        dut._log.warning("Sentinel not reached"); return
    assert h.read_result_long(0) == 256
    h.cleanup()


# ===== Large register counts ================================================

@cocotb.test()
async def test_lsl_reg_count_16(dut):
    """LSL.L D1,D0: D1=16, D0=1 -> 1<<16 = 65536."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        *moveq(16, 1),
        *moveq(1, 0),
        *lsl(LONG, 1, 0, ir=1),
        *_store_and_advance(),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    if not found:
        dut._log.warning("Sentinel not reached"); return
    assert h.read_result_long(0) == 65536
    h.cleanup()


@cocotb.test()
async def test_lsr_reg_count_31(dut):
    """LSR.L D1,D0: D1=31, D0=0x80000000 -> 1."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        *moveq(31, 1),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0), *imm_long(0x80000000),
        *nop(),
        *lsr(LONG, 1, 0, ir=1),
        *_store_and_advance(),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    if not found:
        dut._log.warning("Sentinel not reached"); return
    assert h.read_result_long(0) == 1
    h.cleanup()


# ===== Shift/rotate of zero -> Z flag =======================================

@cocotb.test()
async def test_lsl_zero_result(dut):
    """LSL.L #1 of 0 -> 0; Z=1."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        *moveq(0, 0),
        *lsl(LONG, 1, 0, ir=0),
        *_store_ccr_and_advance(),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    if not found:
        dut._log.warning("Sentinel not reached"); return
    ccr = h.read_result_long(0) & 0x1F
    assert ccr & CCR_Z, f"Z should be set, CCR=0x{ccr:02X}"
    h.cleanup()


@cocotb.test()
async def test_ror_preserves_value(dut):
    """ROR.L #32 (via register) should return original value."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        *moveq(32, 1),              # D1 = 32
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0), *imm_long(0xDEADBEEF),
        *nop(),
        *ror(LONG, 1, 0, ir=1),     # ROR.L D1(32), D0 -> full rotation
        *_store_and_advance(),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    if not found:
        dut._log.warning("Sentinel not reached"); return
    result = h.read_result_long(0)
    assert result == 0xDEADBEEF, f"Expected 0xDEADBEEF, got 0x{result:08X}"
    h.cleanup()


@cocotb.test()
async def test_rol_preserves_value(dut):
    """ROL.L #32 (via register) should return original value."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        *moveq(32, 1),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0), *imm_long(0x12345678),
        *nop(),
        *rol(LONG, 1, 0, ir=1),
        *_store_and_advance(),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    if not found:
        dut._log.warning("Sentinel not reached"); return
    result = h.read_result_long(0)
    assert result == 0x12345678, f"Expected 0x12345678, got 0x{result:08X}"
    h.cleanup()


# ===== ASR produces all-ones for negative shifted fully right ================

@cocotb.test()
async def test_asr_full_negative(dut):
    """ASR.L D1(32), D0: -1 >> 32 = -1 (0xFFFFFFFF)."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        *moveq(32, 1),
        *moveq(-1, 0),
        *asr(LONG, 1, 0, ir=1),
        *_store_and_advance(),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    if not found:
        dut._log.warning("Sentinel not reached"); return
    result = h.read_result_long(0)
    assert result == 0xFFFFFFFF, f"Expected 0xFFFFFFFF, got 0x{result:08X}"
    h.cleanup()
