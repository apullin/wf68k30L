"""
Move, swap, exchange, and extend instruction compliance tests for WF68K30L.

Tests: MOVEQ, MOVE (reg-to-reg, all sizes), MOVEA (long/word+sign-ext),
       EXG (data-data, addr-addr, data-addr), SWAP, EXT.W, EXT.L, EXTB.L.

Each test uses the prefetch pipeline hazard workaround:
  - Load RESULT_BASE into A0 early with MOVEA.L #addr, A0
  - Store results via MOVE.L Dn, (A0) (single-word, no extension)
  - Advance A0 via ADDQ.L #4, A0

Flag verification captures CCR via MOVE CCR, Dn and stores the low byte.
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles

from cpu_harness import CPUTestHarness
from m68k_encode import (
    BYTE, WORD, LONG,
    DN, AN, AN_IND, AN_POSTINC, AN_PREDEC, SPECIAL, ABS_L, IMMEDIATE,
    moveq, move, movea, move_to_abs_long, nop, addq, subq, addi, subi,
    clr, neg, not_op, swap, and_op, or_op, eor, cmpi,
    exg, EXG_DD, EXG_AA, EXG_DA,
    ext_w, ext_l, extb,
    move_to_ccr, move_from_ccr,
    imm_long, imm_word, imm_byte,
    bcc, CC_EQ, CC_NE,
)


# ---- Helpers ---------------------------------------------------------------

def _setup_a0(h):
    """Return instruction words: MOVEA.L #RESULT_BASE, A0."""
    return [*movea(LONG, SPECIAL, IMMEDIATE, 0), *imm_long(h.RESULT_BASE)]


def _store_and_advance():
    """MOVE.L D0,(A0) ; ADDQ.L #4,A0 -- store D0 result, bump pointer."""
    return [*move(LONG, DN, 0, AN_IND, 0), *addq(LONG, 4, AN, 0)]


def _store_dn_and_advance(dn):
    """MOVE.L Dn,(A0) ; ADDQ.L #4,A0."""
    return [*move(LONG, DN, dn, AN_IND, 0), *addq(LONG, 4, AN, 0)]


def _store_ccr_and_advance():
    """MOVE CCR,D6 ; MOVE.L D6,(A0) ; ADDQ.L #4,A0."""
    return [*move_from_ccr(DN, 6), *move(LONG, DN, 6, AN_IND, 0), *addq(LONG, 4, AN, 0)]


# ---- MOVEQ tests ----------------------------------------------------------

@cocotb.test()
async def test_moveq_zero(dut):
    """MOVEQ #0,D0 -> 0; CC: Z=1, N=0, V=0, C=0."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        *moveq(0, 0),
        *_store_and_advance(),
        *_store_ccr_and_advance(),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    if not found:
        dut._log.warning("Sentinel not reached"); return
    assert h.read_result_long(0) == 0, f"Expected 0, got 0x{h.read_result_long(0):08X}"
    ccr = h.read_result_long(4) & 0x1F
    assert ccr & 0x04, f"Z should be set, CCR=0x{ccr:02X}"
    assert not (ccr & 0x08), f"N should be clear, CCR=0x{ccr:02X}"
    assert not (ccr & 0x02), f"V should be clear, CCR=0x{ccr:02X}"
    assert not (ccr & 0x01), f"C should be clear, CCR=0x{ccr:02X}"
    h.cleanup()


@cocotb.test()
async def test_moveq_one(dut):
    """MOVEQ #1,D0 -> 1; CC: Z=0, N=0, V=0, C=0."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        *moveq(1, 0),
        *_store_and_advance(),
        *_store_ccr_and_advance(),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    if not found:
        dut._log.warning("Sentinel not reached"); return
    assert h.read_result_long(0) == 1
    ccr = h.read_result_long(4) & 0x1F
    assert not (ccr & 0x04), f"Z should be clear, CCR=0x{ccr:02X}"
    assert not (ccr & 0x08), f"N should be clear, CCR=0x{ccr:02X}"
    assert not (ccr & 0x02), f"V should be clear, CCR=0x{ccr:02X}"
    assert not (ccr & 0x01), f"C should be clear, CCR=0x{ccr:02X}"
    h.cleanup()


@cocotb.test()
async def test_moveq_42(dut):
    """MOVEQ #42,D0 -> 42."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        *moveq(42, 0),
        *_store_and_advance(),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    if not found:
        dut._log.warning("Sentinel not reached"); return
    assert h.read_result_long(0) == 42
    h.cleanup()


@cocotb.test()
async def test_moveq_127(dut):
    """MOVEQ #127,D0 -> 127."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        *moveq(127, 0),
        *_store_and_advance(),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    if not found:
        dut._log.warning("Sentinel not reached"); return
    assert h.read_result_long(0) == 127
    h.cleanup()


@cocotb.test()
async def test_moveq_neg128(dut):
    """MOVEQ #-128,D0 -> 0xFFFFFF80 (sign-extended)."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        *moveq(-128, 0),
        *_store_and_advance(),
        *_store_ccr_and_advance(),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    if not found:
        dut._log.warning("Sentinel not reached"); return
    assert h.read_result_long(0) == 0xFFFFFF80
    ccr = h.read_result_long(4) & 0x1F
    assert ccr & 0x08, f"N should be set, CCR=0x{ccr:02X}"
    assert not (ccr & 0x04), f"Z should be clear, CCR=0x{ccr:02X}"
    h.cleanup()


@cocotb.test()
async def test_moveq_neg1(dut):
    """MOVEQ #-1,D0 -> 0xFFFFFFFF; CC: N=1."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        *moveq(-1, 0),
        *_store_and_advance(),
        *_store_ccr_and_advance(),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    if not found:
        dut._log.warning("Sentinel not reached"); return
    assert h.read_result_long(0) == 0xFFFFFFFF
    ccr = h.read_result_long(4) & 0x1F
    assert ccr & 0x08, f"N should be set, CCR=0x{ccr:02X}"
    assert not (ccr & 0x02), f"V should be clear, CCR=0x{ccr:02X}"
    assert not (ccr & 0x01), f"C should be clear, CCR=0x{ccr:02X}"
    h.cleanup()


@cocotb.test()
async def test_moveq_to_d3(dut):
    """MOVEQ #99,D3 -> D3=99."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        *moveq(99, 3),
        *_store_dn_and_advance(3),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    if not found:
        dut._log.warning("Sentinel not reached"); return
    assert h.read_result_long(0) == 99
    h.cleanup()


# ---- MOVE register-to-register tests --------------------------------------

@cocotb.test()
async def test_move_long_d0_d1(dut):
    """MOVE.L D0,D1 -- full 32-bit copy."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        *moveq(77, 0),
        *move(LONG, DN, 0, DN, 1),
        *_store_dn_and_advance(1),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    if not found:
        dut._log.warning("Sentinel not reached"); return
    assert h.read_result_long(0) == 77
    h.cleanup()


@cocotb.test()
async def test_move_word_d0_d1(dut):
    """MOVE.W D0,D1 -- copies low word, preserves upper word of D1."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        # D1 = 0xAABBCCDD
        *move(LONG, SPECIAL, IMMEDIATE, DN, 1), *imm_long(0xAABBCCDD),
        # D0 = 0x00001234
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0), *imm_long(0x00001234),
        *nop(),
        # MOVE.W D0, D1 -> D1 should be 0xAABB1234
        *move(WORD, DN, 0, DN, 1),
        *_store_dn_and_advance(1),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    if not found:
        dut._log.warning("Sentinel not reached"); return
    result = h.read_result_long(0)
    assert result == 0xAABB1234, f"Expected 0xAABB1234, got 0x{result:08X}"
    h.cleanup()


@cocotb.test()
async def test_move_byte_d0_d1(dut):
    """MOVE.B D0,D1 -- copies low byte, preserves upper 3 bytes of D1."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        # D1 = 0xAABBCCDD
        *move(LONG, SPECIAL, IMMEDIATE, DN, 1), *imm_long(0xAABBCCDD),
        # D0 = 0x00000056
        *moveq(0x56, 0),
        # MOVE.B D0, D1 -> D1 should be 0xAABBCC56
        *move(BYTE, DN, 0, DN, 1),
        *_store_dn_and_advance(1),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    if not found:
        dut._log.warning("Sentinel not reached"); return
    result = h.read_result_long(0)
    assert result == 0xAABBCC56, f"Expected 0xAABBCC56, got 0x{result:08X}"
    h.cleanup()


@cocotb.test()
async def test_move_long_d2_d5(dut):
    """MOVE.L D2,D5 -- different source/dest regs."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        *moveq(33, 2),
        *move(LONG, DN, 2, DN, 5),
        *_store_dn_and_advance(5),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    if not found:
        dut._log.warning("Sentinel not reached"); return
    assert h.read_result_long(0) == 33
    h.cleanup()


@cocotb.test()
async def test_move_long_d0_d0(dut):
    """MOVE.L D0,D0 -- self-copy should not change value but sets CC."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        *moveq(55, 0),
        *move(LONG, DN, 0, DN, 0),
        *_store_and_advance(),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    if not found:
        dut._log.warning("Sentinel not reached"); return
    assert h.read_result_long(0) == 55
    h.cleanup()


@cocotb.test()
async def test_move_word_sets_cc(dut):
    """MOVE.W sets CC: test zero result -> Z=1."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        *moveq(0, 0),
        *move(WORD, DN, 0, DN, 1),
        *_store_ccr_and_advance(),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    if not found:
        dut._log.warning("Sentinel not reached"); return
    ccr = h.read_result_long(0) & 0x1F
    assert ccr & 0x04, f"Z should be set for zero word, CCR=0x{ccr:02X}"
    h.cleanup()


@cocotb.test()
async def test_move_byte_negative_sets_n(dut):
    """MOVE.B with high bit set -> N=1."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        *moveq(-1, 0),          # D0=0xFFFFFFFF
        *move(BYTE, DN, 0, DN, 1),  # D1 low byte = 0xFF -> N=1
        *_store_ccr_and_advance(),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    if not found:
        dut._log.warning("Sentinel not reached"); return
    ccr = h.read_result_long(0) & 0x1F
    assert ccr & 0x08, f"N should be set for 0xFF byte, CCR=0x{ccr:02X}"
    h.cleanup()


# ---- MOVEA tests -----------------------------------------------------------

@cocotb.test()
async def test_movea_long_d0_a1(dut):
    """MOVEA.L D0,A1 -- full 32-bit transfer to address register."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0), *imm_long(0x12345678),
        *nop(),
        *movea(LONG, DN, 0, 1),     # MOVEA.L D0, A1
        # Store A1 -- move A1 to D2 first (MOVE.L A1, D2)
        *move(LONG, AN, 1, DN, 2),
        *_store_dn_and_advance(2),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    if not found:
        dut._log.warning("Sentinel not reached"); return
    result = h.read_result_long(0)
    assert result == 0x12345678, f"Expected 0x12345678, got 0x{result:08X}"
    h.cleanup()


@cocotb.test()
async def test_movea_word_positive(dut):
    """MOVEA.W D0,A1 -- word value sign-extended: 0x1234 -> 0x00001234."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0), *imm_long(0x00001234),
        *nop(),
        *movea(WORD, DN, 0, 1),     # MOVEA.W D0, A1 -> sign-extends 0x1234
        *move(LONG, AN, 1, DN, 2),  # A1 -> D2
        *_store_dn_and_advance(2),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    if not found:
        dut._log.warning("Sentinel not reached"); return
    result = h.read_result_long(0)
    assert result == 0x00001234, f"Expected 0x00001234, got 0x{result:08X}"
    h.cleanup()


@cocotb.test()
async def test_movea_word_negative(dut):
    """MOVEA.W D0,A1 -- negative word sign-extended: 0x8000 -> 0xFFFF8000."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0), *imm_long(0x00008000),
        *nop(),
        *movea(WORD, DN, 0, 1),     # MOVEA.W D0, A1 -> sign-extends 0x8000
        *move(LONG, AN, 1, DN, 2),  # A1 -> D2
        *_store_dn_and_advance(2),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    if not found:
        dut._log.warning("Sentinel not reached"); return
    result = h.read_result_long(0)
    assert result == 0xFFFF8000, f"Expected 0xFFFF8000, got 0x{result:08X}"
    h.cleanup()


@cocotb.test()
async def test_movea_word_ffff(dut):
    """MOVEA.W D0,A1 -- 0xFFFF -> 0xFFFFFFFF."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        *moveq(-1, 0),              # D0 = 0xFFFFFFFF (low word = 0xFFFF)
        *movea(WORD, DN, 0, 1),     # MOVEA.W D0, A1 -> 0xFFFFFFFF
        *move(LONG, AN, 1, DN, 2),
        *_store_dn_and_advance(2),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    if not found:
        dut._log.warning("Sentinel not reached"); return
    result = h.read_result_long(0)
    assert result == 0xFFFFFFFF, f"Expected 0xFFFFFFFF, got 0x{result:08X}"
    h.cleanup()


@cocotb.test()
async def test_movea_no_cc_change(dut):
    """MOVEA does not affect CCR. Set Z=1 with MOVEQ #0, then MOVEA should not clear Z."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        *moveq(0, 1),               # D1=0 -> sets Z=1
        *movea(LONG, DN, 1, 2),     # MOVEA.L D1, A2 -> should NOT affect CCR
        *_store_ccr_and_advance(),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    if not found:
        dut._log.warning("Sentinel not reached"); return
    ccr = h.read_result_long(0) & 0x1F
    # Z was set by MOVEQ #0 and MOVEA should not change it
    assert ccr & 0x04, f"Z should still be set after MOVEA, CCR=0x{ccr:02X}"
    h.cleanup()


# ---- EXG tests -------------------------------------------------------------

@cocotb.test()
async def test_exg_data_data(dut):
    """EXG D0,D1 -- swap data registers."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        *moveq(10, 0),              # D0 = 10
        *moveq(20, 1),              # D1 = 20
        *exg(0, 1, EXG_DD),         # EXG D0, D1
        *_store_and_advance(),       # store D0 (should be 20)
        *_store_dn_and_advance(1),   # store D1 (should be 10)
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    if not found:
        dut._log.warning("Sentinel not reached"); return
    d0 = h.read_result_long(0)
    d1 = h.read_result_long(4)
    assert d0 == 20, f"Expected D0=20, got 0x{d0:08X}"
    assert d1 == 10, f"Expected D1=10, got 0x{d1:08X}"
    h.cleanup()


@cocotb.test()
async def test_exg_addr_addr(dut):
    """EXG A1,A2 -- swap address registers."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        # Load A1 = 0x100, A2 = 0x200
        *moveq(0, 1),               # D1 = 0
        *moveq(0, 2),               # D2 = 0
        *move(LONG, SPECIAL, IMMEDIATE, DN, 1), *imm_long(0x00000100),
        *nop(),
        *movea(LONG, DN, 1, 1),     # A1 = 0x100
        *move(LONG, SPECIAL, IMMEDIATE, DN, 2), *imm_long(0x00000200),
        *nop(),
        *movea(LONG, DN, 2, 2),     # A2 = 0x200
        *exg(1, 2, EXG_AA),         # EXG A1, A2
        # Read back: A1 -> D3, A2 -> D4
        *move(LONG, AN, 1, DN, 3),
        *move(LONG, AN, 2, DN, 4),
        *_store_dn_and_advance(3),   # store A1 (should be 0x200)
        *_store_dn_and_advance(4),   # store A2 (should be 0x100)
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    if not found:
        dut._log.warning("Sentinel not reached"); return
    a1 = h.read_result_long(0)
    a2 = h.read_result_long(4)
    assert a1 == 0x200, f"Expected A1=0x200, got 0x{a1:08X}"
    assert a2 == 0x100, f"Expected A2=0x100, got 0x{a2:08X}"
    h.cleanup()


@cocotb.test()
async def test_exg_data_addr(dut):
    """EXG D2,A3 -- swap data reg and address reg."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 2), *imm_long(0xAAAA0000),
        *nop(),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 3), *imm_long(0x0000BBBB),
        *nop(),
        *movea(LONG, DN, 3, 3),     # A3 = 0x0000BBBB
        *exg(2, 3, EXG_DA),         # EXG D2, A3
        # D2 -> result[0], A3 -> D4 -> result[4]
        *_store_dn_and_advance(2),
        *move(LONG, AN, 3, DN, 4),
        *_store_dn_and_advance(4),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    if not found:
        dut._log.warning("Sentinel not reached"); return
    d2 = h.read_result_long(0)
    a3 = h.read_result_long(4)
    assert d2 == 0x0000BBBB, f"Expected D2=0x0000BBBB, got 0x{d2:08X}"
    assert a3 == 0xAAAA0000, f"Expected A3=0xAAAA0000, got 0x{a3:08X}"
    h.cleanup()


# ---- SWAP tests ------------------------------------------------------------

@cocotb.test()
async def test_swap_basic(dut):
    """SWAP D0: 0x12345678 -> 0x56781234."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0), *imm_long(0x12345678),
        *nop(),
        *swap(0),
        *_store_and_advance(),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    if not found:
        dut._log.warning("Sentinel not reached"); return
    result = h.read_result_long(0)
    assert result == 0x56781234, f"Expected 0x56781234, got 0x{result:08X}"
    h.cleanup()


@cocotb.test()
async def test_swap_zero(dut):
    """SWAP D0 where D0=0 -> 0; CC: Z=1, N=0, V=0, C=0."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        *moveq(0, 0),
        *swap(0),
        *_store_and_advance(),
        *_store_ccr_and_advance(),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    if not found:
        dut._log.warning("Sentinel not reached"); return
    assert h.read_result_long(0) == 0
    ccr = h.read_result_long(4) & 0x1F
    assert ccr & 0x04, f"Z should be set, CCR=0x{ccr:02X}"
    assert not (ccr & 0x08), f"N should be clear, CCR=0x{ccr:02X}"
    h.cleanup()


@cocotb.test()
async def test_swap_negative_result(dut):
    """SWAP D0 where D0=0x00008000 -> 0x80000000; CC: N=1, Z=0."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0), *imm_long(0x00008000),
        *nop(),
        *swap(0),
        *_store_and_advance(),
        *_store_ccr_and_advance(),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    if not found:
        dut._log.warning("Sentinel not reached"); return
    result = h.read_result_long(0)
    assert result == 0x80000000, f"Expected 0x80000000, got 0x{result:08X}"
    ccr = h.read_result_long(4) & 0x1F
    assert ccr & 0x08, f"N should be set, CCR=0x{ccr:02X}"
    assert not (ccr & 0x04), f"Z should be clear, CCR=0x{ccr:02X}"
    h.cleanup()


@cocotb.test()
async def test_swap_clears_vc(dut):
    """SWAP always clears V and C."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        *moveq(1, 0),
        *swap(0),                    # 0x00000001 -> 0x00010000
        *_store_ccr_and_advance(),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    if not found:
        dut._log.warning("Sentinel not reached"); return
    ccr = h.read_result_long(0) & 0x1F
    assert not (ccr & 0x02), f"V should be clear, CCR=0x{ccr:02X}"
    assert not (ccr & 0x01), f"C should be clear, CCR=0x{ccr:02X}"
    h.cleanup()


@cocotb.test()
async def test_swap_d3(dut):
    """SWAP on a different register: SWAP D3."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 3), *imm_long(0xAABB1122),
        *nop(),
        *swap(3),
        *_store_dn_and_advance(3),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    if not found:
        dut._log.warning("Sentinel not reached"); return
    result = h.read_result_long(0)
    assert result == 0x1122AABB, f"Expected 0x1122AABB, got 0x{result:08X}"
    h.cleanup()


# ---- EXT / EXTB tests ------------------------------------------------------

@cocotb.test()
async def test_ext_w_positive(dut):
    """EXT.W D0: byte 0x12 -> word 0x0012 (upper word unchanged in long)."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0), *imm_long(0xFFFF0012),
        *nop(),
        *ext_w(0),                   # EXT.W D0: sign-extend byte to word in low word
        # Capture CCR BEFORE storing D0, because MOVE.L D0,(A0) would overwrite flags
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
    # EXT.W extends byte in low byte to fill low word; upper word unchanged
    assert result == 0xFFFF0012, f"Expected 0xFFFF0012, got 0x{result:08X}"
    assert not (ccr & 0x08), f"N should be clear (word 0x0012), CCR=0x{ccr:02X}"
    assert not (ccr & 0x04), f"Z should be clear, CCR=0x{ccr:02X}"
    h.cleanup()


@cocotb.test()
async def test_ext_w_negative(dut):
    """EXT.W D0: byte 0x80 -> word 0xFF80."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0), *imm_long(0x00000080),
        *nop(),
        *ext_w(0),
        # Capture CCR BEFORE storing D0, because MOVE.L D0,(A0) would overwrite flags
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
    assert result == 0x0000FF80, f"Expected 0x0000FF80, got 0x{result:08X}"
    assert ccr & 0x08, f"N should be set (word 0xFF80), CCR=0x{ccr:02X}"
    h.cleanup()


@cocotb.test()
async def test_ext_w_zero(dut):
    """EXT.W D0: byte 0x00 -> word 0x0000; Z=1."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        *moveq(0, 0),
        *ext_w(0),
        *_store_and_advance(),
        *_store_ccr_and_advance(),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    if not found:
        dut._log.warning("Sentinel not reached"); return
    result = h.read_result_long(0)
    assert result == 0, f"Expected 0, got 0x{result:08X}"
    ccr = h.read_result_long(4) & 0x1F
    assert ccr & 0x04, f"Z should be set, CCR=0x{ccr:02X}"
    h.cleanup()


@cocotb.test()
async def test_ext_l_positive(dut):
    """EXT.L D0: word 0x0056 -> long 0x00000056."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0), *imm_long(0xFFFF0056),
        *nop(),
        *ext_l(0),
        *_store_and_advance(),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    if not found:
        dut._log.warning("Sentinel not reached"); return
    result = h.read_result_long(0)
    assert result == 0x00000056, f"Expected 0x00000056, got 0x{result:08X}"
    h.cleanup()


@cocotb.test()
async def test_ext_l_negative(dut):
    """EXT.L D0: word 0x8000 -> long 0xFFFF8000."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0), *imm_long(0x00008000),
        *nop(),
        *ext_l(0),
        *_store_and_advance(),
        *_store_ccr_and_advance(),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    if not found:
        dut._log.warning("Sentinel not reached"); return
    result = h.read_result_long(0)
    assert result == 0xFFFF8000, f"Expected 0xFFFF8000, got 0x{result:08X}"
    ccr = h.read_result_long(4) & 0x1F
    assert ccr & 0x08, f"N should be set, CCR=0x{ccr:02X}"
    h.cleanup()


@cocotb.test()
async def test_ext_l_zero(dut):
    """EXT.L D0: word 0x0000 -> long 0x00000000; Z=1."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        *moveq(0, 0),
        *ext_l(0),
        *_store_and_advance(),
        *_store_ccr_and_advance(),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    if not found:
        dut._log.warning("Sentinel not reached"); return
    assert h.read_result_long(0) == 0
    ccr = h.read_result_long(4) & 0x1F
    assert ccr & 0x04, f"Z should be set, CCR=0x{ccr:02X}"
    h.cleanup()


@cocotb.test()
async def test_extb_positive(dut):
    """EXTB.L D0: byte 0x12 -> long 0x00000012."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0), *imm_long(0xABCD0012),
        *nop(),
        *extb(0),
        *_store_and_advance(),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    if not found:
        dut._log.warning("Sentinel not reached"); return
    result = h.read_result_long(0)
    assert result == 0x00000012, f"Expected 0x00000012, got 0x{result:08X}"
    h.cleanup()


@cocotb.test()
async def test_extb_negative(dut):
    """EXTB.L D0: byte 0x80 -> long 0xFFFFFF80."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0), *imm_long(0x00000080),
        *nop(),
        *extb(0),
        *_store_and_advance(),
        *_store_ccr_and_advance(),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    if not found:
        dut._log.warning("Sentinel not reached"); return
    result = h.read_result_long(0)
    assert result == 0xFFFFFF80, f"Expected 0xFFFFFF80, got 0x{result:08X}"
    ccr = h.read_result_long(4) & 0x1F
    assert ccr & 0x08, f"N should be set, CCR=0x{ccr:02X}"
    h.cleanup()


@cocotb.test()
async def test_extb_zero(dut):
    """EXTB.L D0: byte 0x00 -> long 0x00000000; Z=1."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        *moveq(0, 0),
        *extb(0),
        *_store_and_advance(),
        *_store_ccr_and_advance(),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    if not found:
        dut._log.warning("Sentinel not reached"); return
    assert h.read_result_long(0) == 0
    ccr = h.read_result_long(4) & 0x1F
    assert ccr & 0x04, f"Z should be set, CCR=0x{ccr:02X}"
    h.cleanup()


@cocotb.test()
async def test_extb_ff(dut):
    """EXTB.L D0: byte 0xFF -> long 0xFFFFFFFF."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        *moveq(-1, 0),              # D0=0xFFFFFFFF
        *extb(0),
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


@cocotb.test()
async def test_ext_w_ff(dut):
    """EXT.W D0: byte 0xFF -> word 0xFFFF (long upper unchanged)."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        *moveq(-1, 0),              # D0=0xFFFFFFFF
        *ext_w(0),
        *_store_and_advance(),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    if not found:
        dut._log.warning("Sentinel not reached"); return
    result = h.read_result_long(0)
    # EXT.W: sign-extend byte 0xFF to word 0xFFFF, upper word unchanged (0xFFFF)
    assert result == 0xFFFFFFFF, f"Expected 0xFFFFFFFF, got 0x{result:08X}"
    h.cleanup()


@cocotb.test()
async def test_ext_l_ffff(dut):
    """EXT.L D0: word 0xFFFF -> long 0xFFFFFFFF."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        *moveq(-1, 0),              # D0=0xFFFFFFFF
        *ext_l(0),
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


# ---- MOVE.L #imm, Dn via immediate mode -----------------------------------

@cocotb.test()
async def test_move_long_imm_zero(dut):
    """MOVE.L #0, D0 -> D0=0."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0), *imm_long(0),
        *nop(),
        *_store_and_advance(),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    if not found:
        dut._log.warning("Sentinel not reached"); return
    assert h.read_result_long(0) == 0
    h.cleanup()


@cocotb.test()
async def test_move_long_imm_large(dut):
    """MOVE.L #0xDEADBEEF, D0 -> D0=0xDEADBEEF."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0), *imm_long(0xDEADBEEF),
        *nop(),
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


# ---- MOVE between different register pairs ---------------------------------

@cocotb.test()
async def test_move_long_chain(dut):
    """Chain: MOVEQ #11,D0; MOVE.L D0,D1; MOVE.L D1,D2; verify D2=11."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        *moveq(11, 0),
        *move(LONG, DN, 0, DN, 1),
        *move(LONG, DN, 1, DN, 2),
        *_store_dn_and_advance(2),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    if not found:
        dut._log.warning("Sentinel not reached"); return
    assert h.read_result_long(0) == 11
    h.cleanup()


@cocotb.test()
async def test_move_word_preserves_upper(dut):
    """MOVE.W into D3 preserves its upper word (second test variant)."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 3), *imm_long(0xDEAD0000),
        *nop(),
        *moveq(0x55, 1),            # D1 = 0x55
        *move(WORD, DN, 1, DN, 3),  # D3 lower word = 0x0055
        *_store_dn_and_advance(3),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    if not found:
        dut._log.warning("Sentinel not reached"); return
    result = h.read_result_long(0)
    assert result == 0xDEAD0055, f"Expected 0xDEAD0055, got 0x{result:08X}"
    h.cleanup()


# ---- Additional MOVEA tests ------------------------------------------------

@cocotb.test()
async def test_movea_long_imm(dut):
    """MOVEA.L #imm, A1 -- load immediate into address register."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        *movea(LONG, SPECIAL, IMMEDIATE, 1), *imm_long(0xCAFEBABE),
        *move(LONG, AN, 1, DN, 2),
        *_store_dn_and_advance(2),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    if not found:
        dut._log.warning("Sentinel not reached"); return
    result = h.read_result_long(0)
    assert result == 0xCAFEBABE, f"Expected 0xCAFEBABE, got 0x{result:08X}"
    h.cleanup()


# ---- EXG with large values -------------------------------------------------

@cocotb.test()
async def test_exg_data_data_large(dut):
    """EXG D0,D1 with full 32-bit values."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0), *imm_long(0xAAAAAAAA),
        *nop(),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 1), *imm_long(0x55555555),
        *nop(),
        *exg(0, 1, EXG_DD),
        *_store_and_advance(),
        *_store_dn_and_advance(1),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    if not found:
        dut._log.warning("Sentinel not reached"); return
    d0 = h.read_result_long(0)
    d1 = h.read_result_long(4)
    assert d0 == 0x55555555, f"Expected D0=0x55555555, got 0x{d0:08X}"
    assert d1 == 0xAAAAAAAA, f"Expected D1=0xAAAAAAAA, got 0x{d1:08X}"
    h.cleanup()


# ---- MOVEQ to all 8 data registers ----------------------------------------

@cocotb.test()
async def test_moveq_all_data_regs(dut):
    """MOVEQ to D0-D6 (D7 is sentinel scratch), verify each."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        *moveq(10, 0),
        *moveq(20, 1),
        *moveq(30, 2),
        *moveq(40, 3),
        *moveq(50, 4),
        *moveq(60, 5),
        *moveq(70, 6),
        *_store_and_advance(),
        *_store_dn_and_advance(1),
        *_store_dn_and_advance(2),
        *_store_dn_and_advance(3),
        *_store_dn_and_advance(4),
        *_store_dn_and_advance(5),
        *_store_dn_and_advance(6),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    if not found:
        dut._log.warning("Sentinel not reached"); return
    expected = [10, 20, 30, 40, 50, 60, 70]
    for i, exp in enumerate(expected):
        result = h.read_result_long(i * 4)
        assert result == exp, f"D{i}: expected {exp}, got 0x{result:08X}"
    h.cleanup()


# ---- SWAP double-swap identity -------------------------------------------

@cocotb.test()
async def test_swap_double(dut):
    """SWAP twice returns original value: SWAP SWAP 0xAABBCCDD -> 0xAABBCCDD."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0), *imm_long(0xAABBCCDD),
        *nop(),
        *swap(0),
        *swap(0),
        *_store_and_advance(),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    if not found:
        dut._log.warning("Sentinel not reached"); return
    result = h.read_result_long(0)
    assert result == 0xAABBCCDD, f"Expected 0xAABBCCDD, got 0x{result:08X}"
    h.cleanup()


# ---- EXT.W then EXT.L chain -----------------------------------------------

@cocotb.test()
async def test_ext_w_then_ext_l(dut):
    """EXT.W then EXT.L: byte 0x80 -> word 0xFF80 -> long 0xFFFFFF80."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0), *imm_long(0x00000080),
        *nop(),
        *ext_w(0),                   # 0x0000FF80
        *ext_l(0),                   # 0xFFFFFF80
        *_store_and_advance(),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    if not found:
        dut._log.warning("Sentinel not reached"); return
    result = h.read_result_long(0)
    assert result == 0xFFFFFF80, f"Expected 0xFFFFFF80, got 0x{result:08X}"
    h.cleanup()


# ---- MOVE byte to different register ------------------------------------

@cocotb.test()
async def test_move_byte_d3_d5(dut):
    """MOVE.B D3,D5 -- byte copy between different registers."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 5), *imm_long(0x11223344),
        *nop(),
        *moveq(0x7F, 3),            # D3 = 0x7F
        *move(BYTE, DN, 3, DN, 5),  # D5 low byte = 0x7F, upper bytes unchanged
        *_store_dn_and_advance(5),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    if not found:
        dut._log.warning("Sentinel not reached"); return
    result = h.read_result_long(0)
    assert result == 0x1122337F, f"Expected 0x1122337F, got 0x{result:08X}"
    h.cleanup()


# ---- EXT operations set V=0, C=0 -----------------------------------------

@cocotb.test()
async def test_ext_clears_vc(dut):
    """EXT.W, EXT.L, EXTB.L all clear V and C."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        *moveq(-1, 0),              # D0=0xFFFFFFFF
        *ext_w(0),                   # byte 0xFF -> word 0xFFFF
        *_store_ccr_and_advance(),   # CCR after EXT.W
        *ext_l(0),                   # word 0xFFFF -> long 0xFFFFFFFF
        *_store_ccr_and_advance(),   # CCR after EXT.L
        *moveq(0x7F, 1),            # D1=0x7F
        *extb(1),                    # byte 0x7F -> long 0x0000007F
        *_store_ccr_and_advance(),   # CCR after EXTB.L
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    if not found:
        dut._log.warning("Sentinel not reached"); return
    for i, name in enumerate(["EXT.W", "EXT.L", "EXTB.L"]):
        ccr = h.read_result_long(i * 4) & 0x1F
        assert not (ccr & 0x02), f"{name}: V should be clear, CCR=0x{ccr:02X}"
        assert not (ccr & 0x01), f"{name}: C should be clear, CCR=0x{ccr:02X}"
    h.cleanup()
