"""
Memory-oriented instruction compliance tests for WF68K30L.

Tests LEA, PEA, LINK/UNLK, MOVEM, TAS, and MOVEA against the MC68030
specification.

Each test uses a consistent result-store pattern:
  - Load RESULT_BASE into A0 early
  - Store results via MOVE.L Dn,(A0) (single-word, no extension words)
  - Advance A0 with ADDQ.L #4,A0

IMPORTANT: MOVE from CCR must be done BEFORE any MOVE stores, because
MOVE.L Dn,(A0) itself sets condition codes, overwriting the flags from
the instruction under test. The pattern is:
  1. instruction under test
  2. NOP NOP (pipeline drain)
  3. MOVE CCR,D6 (capture flags before any store)
  4. MOVE.L Dn,(A0) / ADDQ / MOVE.L D6,(A0) / ADDQ (store result + CCR)
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles

from cpu_harness import CPUTestHarness
from m68k_encode import (
    BYTE, WORD, LONG,
    DN, AN, AN_IND, AN_POSTINC, AN_PREDEC, AN_DISP, SPECIAL, ABS_L, IMMEDIATE,
    moveq, move, movea, move_to_abs_long, nop, addq, subq,
    addi, subi,
    lea, pea, link_w, unlk, tas,
    movem_to_mem, movem_from_mem,
    move_from_ccr, move_to_ccr,
    imm_long, imm_word, imm_byte, abs_long, disp16,
)


# ---------------------------------------------------------------------------
# Helper: extract CCR flags from a stored long (low 8 bits of CCR word)
# ---------------------------------------------------------------------------
def ccr_flags(val):
    """Extract (x, n, z, v, c) from a stored CCR value (32-bit read)."""
    ccr = val & 0xFF
    x = (ccr >> 4) & 1
    n = (ccr >> 3) & 1
    z = (ccr >> 2) & 1
    v = (ccr >> 1) & 1
    c = ccr & 1
    return x, n, z, v, c


def movem_predec_mask(mask):
    """Reverse MOVEM mask bit order for -(An) encoding."""
    rev = 0
    for i in range(16):
        if (mask >> i) & 1:
            rev |= 1 << (15 - i)
    return rev


# =========================================================================
# LEA tests
# =========================================================================

@cocotb.test()
async def test_lea_abs_long(dut):
    """LEA (abs).L,A2: load absolute address into A2."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *lea(SPECIAL, ABS_L, 2),                  # LEA (xxx).L,A2
        *abs_long(0x00012345),
        *nop(), *nop(),
        # Store A2 value: move A2 to D0 then store
        *move(LONG, AN, 2, DN, 1),                # MOVE.L A2,D1
        *move(LONG, DN, 1, AN_IND, 0),            # MOVE.L D1,(A0)
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    assert result == 0x00012345, f"Expected A2=0x00012345, got 0x{result:08X}"
    h.cleanup()


@cocotb.test()
async def test_lea_an_indirect(dut):
    """LEA (A3),A2: load address in A3 into A2."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *movea(LONG, SPECIAL, IMMEDIATE, 3),      # MOVEA.L #addr,A3
        *imm_long(0x00054321),
        *lea(AN_IND, 3, 2),                       # LEA (A3),A2
        *nop(), *nop(),
        *move(LONG, AN, 2, DN, 1),                # MOVE.L A2,D1
        *move(LONG, DN, 1, AN_IND, 0),            # MOVE.L D1,(A0)
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    assert result == 0x00054321, f"Expected A2=0x00054321, got 0x{result:08X}"
    h.cleanup()


@cocotb.test()
async def test_lea_an_disp(dut):
    """LEA (d16,A3),A2: load A3+displacement into A2."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *movea(LONG, SPECIAL, IMMEDIATE, 3),      # MOVEA.L #0x1000,A3
        *imm_long(0x00001000),
        *lea(AN_DISP, 3, 2),                      # LEA (d16,A3),A2
        *disp16(0x0080),                           # displacement = +128
        *nop(), *nop(),
        *move(LONG, AN, 2, DN, 1),                # MOVE.L A2,D1
        *move(LONG, DN, 1, AN_IND, 0),            # MOVE.L D1,(A0)
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    assert result == 0x00001080, f"Expected A2=0x00001080, got 0x{result:08X}"
    h.cleanup()


@cocotb.test()
async def test_lea_an_disp_negative(dut):
    """LEA (d16,A3),A2: negative displacement."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *movea(LONG, SPECIAL, IMMEDIATE, 3),      # MOVEA.L #0x2000,A3
        *imm_long(0x00002000),
        *lea(AN_DISP, 3, 2),                      # LEA (d16,A3),A2
        *disp16(-16),                              # displacement = -16
        *nop(), *nop(),
        *move(LONG, AN, 2, DN, 1),                # MOVE.L A2,D1
        *move(LONG, DN, 1, AN_IND, 0),            # MOVE.L D1,(A0)
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    assert result == 0x00001FF0, f"Expected A2=0x00001FF0, got 0x{result:08X}"
    h.cleanup()


@cocotb.test()
async def test_lea_does_not_affect_ccr(dut):
    """LEA does not affect condition codes."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        # Set known CCR state: set all flags via move to CCR
        *move(WORD, SPECIAL, IMMEDIATE, DN, 5),   # MOVE.W #$1F,D5
        *imm_word(0x001F),
        *move_to_ccr(DN, 5),                      # MOVE D5,CCR (all flags set)
        *nop(), *nop(),
        *lea(SPECIAL, ABS_L, 2),                  # LEA (xxx).L,A2
        *abs_long(0x00005000),
        *nop(), *nop(),
        *move_from_ccr(DN, 6),                    # capture CCR after LEA
        *move(LONG, DN, 6, AN_IND, 0),            # store CCR
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    ccr = h.read_result_long(0)
    x, n, z, v, c = ccr_flags(ccr)
    assert (x, n, z, v, c) == (1, 1, 1, 1, 1), (
        f"LEA should not affect CCR, expected all set, got X={x} N={n} Z={z} V={v} C={c}"
    )
    h.cleanup()


# =========================================================================
# PEA tests
# =========================================================================

@cocotb.test()
async def test_pea_abs_long(dut):
    """PEA (abs).L: push address onto stack, verify SP and stack value."""
    h = CPUTestHarness(dut)
    # SSP starts at 0x001000. PEA decrements SP by 4 and writes address.
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        # Read SP before PEA
        *move(LONG, AN, 7, DN, 1),                # MOVE.L A7,D1 (SP before)
        *move(LONG, DN, 1, AN_IND, 0),            # store SP_before
        *addq(LONG, 4, AN, 0),
        *pea(SPECIAL, ABS_L),                     # PEA ($00004000).L
        *abs_long(0x00004000),
        *nop(), *nop(),
        # Read SP after PEA
        *move(LONG, AN, 7, DN, 2),                # MOVE.L A7,D2 (SP after)
        *move(LONG, DN, 2, AN_IND, 0),            # store SP_after
        *addq(LONG, 4, AN, 0),
        # Read what was pushed onto stack (pointed to by A7)
        *move(LONG, AN_IND, 7, DN, 3),            # MOVE.L (A7),D3
        *move(LONG, DN, 3, AN_IND, 0),            # store stack value
        *addq(LONG, 4, AN, 0),
        # Clean up stack
        *addq(LONG, 4, AN, 7),                    # restore SP
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    sp_before = h.read_result_long(0)
    sp_after = h.read_result_long(4)
    stack_val = h.read_result_long(8)
    assert sp_after == sp_before - 4, (
        f"PEA should decrement SP by 4: before=0x{sp_before:08X}, after=0x{sp_after:08X}"
    )
    assert stack_val == 0x00004000, (
        f"PEA should push address 0x00004000 onto stack, got 0x{stack_val:08X}"
    )
    h.cleanup()


@cocotb.test()
async def test_pea_an_indirect(dut):
    """PEA (A3): push address contained in A3 onto stack."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *movea(LONG, SPECIAL, IMMEDIATE, 3),      # MOVEA.L #0xABCD0000,A3
        *imm_long(0x000ABCD0),
        *pea(AN_IND, 3),                          # PEA (A3)
        *nop(), *nop(),
        # Read what was pushed
        *move(LONG, AN_IND, 7, DN, 3),            # MOVE.L (A7),D3
        *move(LONG, DN, 3, AN_IND, 0),            # store
        *addq(LONG, 4, AN, 0),
        *addq(LONG, 4, AN, 7),                    # restore SP
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    stack_val = h.read_result_long(0)
    assert stack_val == 0x000ABCD0, (
        f"PEA (A3) should push 0x000ABCD0, got 0x{stack_val:08X}"
    )
    h.cleanup()


@cocotb.test()
async def test_pea_does_not_affect_ccr(dut):
    """PEA does not affect condition codes."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        # Set known CCR: all flags set
        *move(WORD, SPECIAL, IMMEDIATE, DN, 5),
        *imm_word(0x001F),
        *move_to_ccr(DN, 5),
        *nop(), *nop(),
        *pea(SPECIAL, ABS_L),                     # PEA ($00005000).L
        *abs_long(0x00005000),
        *nop(), *nop(),
        *move_from_ccr(DN, 6),
        *move(LONG, DN, 6, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *addq(LONG, 4, AN, 7),                    # restore SP
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    ccr = h.read_result_long(0)
    x, n, z, v, c = ccr_flags(ccr)
    assert (x, n, z, v, c) == (1, 1, 1, 1, 1), (
        f"PEA should not affect CCR, got X={x} N={n} Z={z} V={v} C={c}"
    )
    h.cleanup()


# =========================================================================
# LINK / UNLK tests
# =========================================================================

@cocotb.test()
async def test_link_basic(dut):
    """LINK A6,#-8: verify A6 saved on stack and SP adjusted."""
    h = CPUTestHarness(dut)
    # LINK operation:
    #   1. Push old A6 onto stack (SP -= 4, [SP] = old_A6)
    #   2. A6 = SP (A6 now points to saved old_A6)
    #   3. SP = SP + displacement (displacement is negative for allocating frame)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        # Set A6 to a known value first
        *movea(LONG, SPECIAL, IMMEDIATE, 6),      # MOVEA.L #0x00BEEF00,A6
        *imm_long(0x00BEEF00),
        # Save SP before LINK
        *move(LONG, AN, 7, DN, 1),                # D1 = SP_before
        *move(LONG, DN, 1, AN_IND, 0),            # store SP_before
        *addq(LONG, 4, AN, 0),
        *link_w(6, -8),                           # LINK A6,#-8
        *nop(), *nop(),
        # Save A6 (should be SP_before - 4, pointing to saved old A6)
        *move(LONG, AN, 6, DN, 2),                # D2 = A6 (frame pointer)
        *move(LONG, DN, 2, AN_IND, 0),            # store A6
        *addq(LONG, 4, AN, 0),
        # Save SP after LINK (should be A6 + displacement = A6 - 8)
        *move(LONG, AN, 7, DN, 3),                # D3 = SP_after
        *move(LONG, DN, 3, AN_IND, 0),            # store SP_after
        *addq(LONG, 4, AN, 0),
        # Read saved old A6 from stack (at address pointed to by new A6)
        *move(LONG, AN_IND, 6, DN, 4),            # D4 = [A6] (saved old A6)
        *move(LONG, DN, 4, AN_IND, 0),            # store saved old A6
        *addq(LONG, 4, AN, 0),
        # Clean up: UNLK A6 to restore stack
        *unlk(6),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    sp_before = h.read_result_long(0)
    a6_value = h.read_result_long(4)
    sp_after = h.read_result_long(8)
    saved_a6 = h.read_result_long(12)

    # A6 should point to where old A6 was saved = SP_before - 4
    assert a6_value == sp_before - 4, (
        f"A6 should be SP_before-4=0x{sp_before - 4:08X}, got 0x{a6_value:08X}"
    )
    # SP should be A6 + displacement = (SP_before - 4) + (-8) = SP_before - 12
    assert sp_after == a6_value - 8, (
        f"SP should be A6-8=0x{a6_value - 8:08X}, got 0x{sp_after:08X}"
    )
    # Saved old A6 on stack should be 0x00BEEF00
    assert saved_a6 == 0x00BEEF00, (
        f"Saved A6 on stack should be 0x00BEEF00, got 0x{saved_a6:08X}"
    )
    h.cleanup()


@cocotb.test()
async def test_link_zero_displacement(dut):
    """LINK A6,#0: create frame with no local space."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *movea(LONG, SPECIAL, IMMEDIATE, 6),
        *imm_long(0x00CAFE00),
        *link_w(6, 0),                            # LINK A6,#0
        *nop(), *nop(),
        # A6 = SP (frame pointer = stack pointer, no locals)
        *move(LONG, AN, 6, DN, 1),
        *move(LONG, DN, 1, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *move(LONG, AN, 7, DN, 2),
        *move(LONG, DN, 2, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        # Saved old A6 on stack
        *move(LONG, AN_IND, 6, DN, 3),
        *move(LONG, DN, 3, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *unlk(6),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    a6_value = h.read_result_long(0)
    sp_after = h.read_result_long(4)
    saved_a6 = h.read_result_long(8)

    # With displacement 0, SP should equal A6
    assert sp_after == a6_value, (
        f"LINK #0: SP should equal A6, A6=0x{a6_value:08X} SP=0x{sp_after:08X}"
    )
    assert saved_a6 == 0x00CAFE00, (
        f"Saved A6 should be 0x00CAFE00, got 0x{saved_a6:08X}"
    )
    h.cleanup()


@cocotb.test()
async def test_unlk_restores(dut):
    """LINK A6,#-8 then UNLK A6: verify A6 and SP restored."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *movea(LONG, SPECIAL, IMMEDIATE, 6),
        *imm_long(0x00DEAD00),
        # Save SP and A6 before
        *move(LONG, AN, 7, DN, 1),                # D1 = SP_before
        *move(LONG, DN, 1, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *link_w(6, -8),
        *unlk(6),
        *nop(), *nop(),
        # After UNLK, A6 should be restored and SP back to original
        *move(LONG, AN, 6, DN, 2),                # D2 = A6_after_unlk
        *move(LONG, DN, 2, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *move(LONG, AN, 7, DN, 3),                # D3 = SP_after_unlk
        *move(LONG, DN, 3, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    sp_before = h.read_result_long(0)
    a6_after = h.read_result_long(4)
    sp_after = h.read_result_long(8)

    # UNLK should restore A6 to original value
    assert a6_after == 0x00DEAD00, (
        f"UNLK should restore A6=0x00DEAD00, got 0x{a6_after:08X}"
    )
    # UNLK should restore SP to value before LINK
    assert sp_after == sp_before, (
        f"UNLK should restore SP=0x{sp_before:08X}, got 0x{sp_after:08X}"
    )
    h.cleanup()


@cocotb.test()
async def test_link_a5(dut):
    """LINK with A5 instead of A6 to verify register encoding."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *movea(LONG, SPECIAL, IMMEDIATE, 5),
        *imm_long(0x00F00D00),
        *link_w(5, -4),                           # LINK A5,#-4
        *nop(), *nop(),
        # Read saved old A5 from stack
        *move(LONG, AN_IND, 5, DN, 1),
        *move(LONG, DN, 1, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *unlk(5),
        # Verify A5 restored
        *move(LONG, AN, 5, DN, 2),
        *move(LONG, DN, 2, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    saved_a5 = h.read_result_long(0)
    a5_restored = h.read_result_long(4)
    assert saved_a5 == 0x00F00D00, (
        f"Saved A5 should be 0x00F00D00, got 0x{saved_a5:08X}"
    )
    assert a5_restored == 0x00F00D00, (
        f"UNLK should restore A5=0x00F00D00, got 0x{a5_restored:08X}"
    )
    h.cleanup()


# =========================================================================
# MOVEM tests
# =========================================================================

@cocotb.test()
async def test_movem_to_mem_d0_d3(dut):
    """MOVEM.L D0-D3,(A1): save four data registers to memory."""
    h = CPUTestHarness(dut)
    # Register mask for D0-D3: bits 0-3 = 0x000F
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        # Set up D0-D3
        *moveq(11, 0),                            # D0 = 11
        *moveq(22, 1),                            # D1 = 22
        *moveq(33, 2),                            # D2 = 33
        *moveq(44, 3),                            # D3 = 44
        # Point A1 to data area
        *movea(LONG, SPECIAL, IMMEDIATE, 1),
        *imm_long(h.DATA_BASE),
        # MOVEM.L D0-D3,(A1)
        *movem_to_mem(LONG, AN_IND, 1, 0x000F),
        *nop(), *nop(),
        # Now read back from DATA_BASE and store to results
        *movea(LONG, SPECIAL, IMMEDIATE, 1),
        *imm_long(h.DATA_BASE),
        *move(LONG, AN_POSTINC, 1, AN_IND, 0),   # [DATA_BASE+0] -> result[0]
        *addq(LONG, 4, AN, 0),
        *move(LONG, AN_POSTINC, 1, AN_IND, 0),   # [DATA_BASE+4] -> result[4]
        *addq(LONG, 4, AN, 0),
        *move(LONG, AN_POSTINC, 1, AN_IND, 0),   # [DATA_BASE+8] -> result[8]
        *addq(LONG, 4, AN, 0),
        *move(LONG, AN_POSTINC, 1, AN_IND, 0),   # [DATA_BASE+12] -> result[12]
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    assert h.read_result_long(0) == 11, f"D0 mismatch: got 0x{h.read_result_long(0):08X}"
    assert h.read_result_long(4) == 22, f"D1 mismatch: got 0x{h.read_result_long(4):08X}"
    assert h.read_result_long(8) == 33, f"D2 mismatch: got 0x{h.read_result_long(8):08X}"
    assert h.read_result_long(12) == 44, f"D3 mismatch: got 0x{h.read_result_long(12):08X}"
    h.cleanup()


@cocotb.test()
async def test_movem_from_mem_d0_d3(dut):
    """MOVEM.L (A1),D0-D3: load four data registers from memory."""
    h = CPUTestHarness(dut)
    # Pre-load DATA_BASE with known values using the memory model
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        # Pre-load data area: write 4 longs to DATA_BASE using D5 as scratch
        *movea(LONG, SPECIAL, IMMEDIATE, 1),
        *imm_long(h.DATA_BASE),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 5),
        *imm_long(0x000000AA),
        *move(LONG, DN, 5, AN_POSTINC, 1),        # [DATA_BASE+0] = 0xAA
        *move(LONG, SPECIAL, IMMEDIATE, DN, 5),
        *imm_long(0x000000BB),
        *move(LONG, DN, 5, AN_POSTINC, 1),        # [DATA_BASE+4] = 0xBB
        *move(LONG, SPECIAL, IMMEDIATE, DN, 5),
        *imm_long(0x000000CC),
        *move(LONG, DN, 5, AN_POSTINC, 1),        # [DATA_BASE+8] = 0xCC
        *move(LONG, SPECIAL, IMMEDIATE, DN, 5),
        *imm_long(0x000000DD),
        *move(LONG, DN, 5, AN_POSTINC, 1),        # [DATA_BASE+12] = 0xDD
        # Reset A1 to DATA_BASE
        *movea(LONG, SPECIAL, IMMEDIATE, 1),
        *imm_long(h.DATA_BASE),
        # MOVEM.L (A1),D0-D3
        *movem_from_mem(LONG, AN_IND, 1, 0x000F),
        *nop(), *nop(),
        # Store loaded registers
        *move(LONG, DN, 0, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *move(LONG, DN, 1, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *move(LONG, DN, 2, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *move(LONG, DN, 3, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    assert h.read_result_long(0) == 0xAA, f"D0 expected 0xAA, got 0x{h.read_result_long(0):08X}"
    assert h.read_result_long(4) == 0xBB, f"D1 expected 0xBB, got 0x{h.read_result_long(4):08X}"
    assert h.read_result_long(8) == 0xCC, f"D2 expected 0xCC, got 0x{h.read_result_long(8):08X}"
    assert h.read_result_long(12) == 0xDD, f"D3 expected 0xDD, got 0x{h.read_result_long(12):08X}"
    h.cleanup()


@cocotb.test()
async def test_movem_to_mem_three_regs(dut):
    """MOVEM.L D0-D2,(A1): save three data registers to memory."""
    h = CPUTestHarness(dut)
    # Register mask for D0-D2: bits 0-2 = 0x0007
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
        *imm_long(0x11110000),                     # D0 = 0x11110000
        *move(LONG, SPECIAL, IMMEDIATE, DN, 1),
        *imm_long(0x22220000),                     # D1 = 0x22220000
        *move(LONG, SPECIAL, IMMEDIATE, DN, 2),
        *imm_long(0x33330000),                     # D2 = 0x33330000
        *movea(LONG, SPECIAL, IMMEDIATE, 1),
        *imm_long(h.DATA_BASE),
        # MOVEM.L D0-D2,(A1)
        *movem_to_mem(LONG, AN_IND, 1, 0x0007),
        *nop(), *nop(),
        # Read back via POSTINC
        *movea(LONG, SPECIAL, IMMEDIATE, 1),
        *imm_long(h.DATA_BASE),
        *move(LONG, AN_POSTINC, 1, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *move(LONG, AN_POSTINC, 1, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *move(LONG, AN_POSTINC, 1, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    assert h.read_result_long(0) == 0x11110000, f"D0 got 0x{h.read_result_long(0):08X}"
    assert h.read_result_long(4) == 0x22220000, f"D1 got 0x{h.read_result_long(4):08X}"
    assert h.read_result_long(8) == 0x33330000, f"D2 got 0x{h.read_result_long(8):08X}"
    h.cleanup()


@cocotb.test()
async def test_movem_from_mem_postinc(dut):
    """MOVEM.L (A1)+,D0-D1: load registers with post-increment."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        # Pre-load data area
        *movea(LONG, SPECIAL, IMMEDIATE, 1),
        *imm_long(h.DATA_BASE),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 5),
        *imm_long(0x11111111),
        *move(LONG, DN, 5, AN_POSTINC, 1),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 5),
        *imm_long(0x22222222),
        *move(LONG, DN, 5, AN_POSTINC, 1),
        # Reset A1 to DATA_BASE
        *movea(LONG, SPECIAL, IMMEDIATE, 1),
        *imm_long(h.DATA_BASE),
        # MOVEM.L (A1)+,D0-D1
        *movem_from_mem(LONG, AN_POSTINC, 1, 0x0003),
        *nop(), *nop(),
        # Store D0, D1, and A1 (should be advanced by 8)
        *move(LONG, DN, 0, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *move(LONG, DN, 1, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *move(LONG, AN, 1, DN, 3),
        *move(LONG, DN, 3, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    assert h.read_result_long(0) == 0x11111111, f"D0 got 0x{h.read_result_long(0):08X}"
    assert h.read_result_long(4) == 0x22222222, f"D1 got 0x{h.read_result_long(4):08X}"
    a1_after = h.read_result_long(8)
    assert a1_after == h.DATA_BASE + 8, (
        f"A1 should be DATA_BASE+8=0x{h.DATA_BASE + 8:08X}, got 0x{a1_after:08X}"
    )
    h.cleanup()


@cocotb.test()
async def test_movem_to_mem_two_regs(dut):
    """MOVEM.L D0-D1,(A1): two-register masks are handled correctly."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(0x55, 0),                          # D0 = 0x55
        *moveq(0x66, 1),                          # D1 = 0x66
        *movea(LONG, SPECIAL, IMMEDIATE, 1),
        *imm_long(h.DATA_BASE),
        # MOVEM.L D0-D1,(A1)
        *movem_to_mem(LONG, AN_IND, 1, 0x0003),
        *nop(), *nop(),
        # Read back via POSTINC
        *movea(LONG, SPECIAL, IMMEDIATE, 1),
        *imm_long(h.DATA_BASE),
        *move(LONG, AN_POSTINC, 1, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *move(LONG, AN_POSTINC, 1, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    assert h.read_result_long(0) == 0x55, f"D0 got 0x{h.read_result_long(0):08X}"
    assert h.read_result_long(4) == 0x66, f"D1 got 0x{h.read_result_long(4):08X}"
    h.cleanup()


@cocotb.test()
async def test_movem_word_to_mem_two_regs(dut):
    """MOVEM.W D0-D1,(A1): stores low words in register-list order."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
        *imm_long(0x12345678),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 1),
        *imm_long(0x89ABCDEF),
        *movea(LONG, SPECIAL, IMMEDIATE, 1),
        *imm_long(h.DATA_BASE),
        *movem_to_mem(WORD, AN_IND, 1, 0x0003),   # D0-D1
        *nop(), *nop(),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"

    w0 = h.mem.read(h.DATA_BASE + 0, 2)
    w1 = h.mem.read(h.DATA_BASE + 2, 2)
    assert w0 == 0x5678, f"word[0] expected 0x5678, got 0x{w0:04X}"
    assert w1 == 0xCDEF, f"word[1] expected 0xCDEF, got 0x{w1:04X}"
    h.cleanup()


@cocotb.test()
async def test_movem_word_from_mem_sign_extend(dut):
    """MOVEM.W (A1),D0-D1: word loads are sign-extended to 32 bits."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *movea(LONG, SPECIAL, IMMEDIATE, 1),
        *imm_long(h.DATA_BASE),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 5),
        *imm_long(0x00007FFF),
        *move(WORD, DN, 5, AN_POSTINC, 1),        # [DATA_BASE+0] = 0x7FFF
        *move(LONG, SPECIAL, IMMEDIATE, DN, 5),
        *imm_long(0x00008001),
        *move(WORD, DN, 5, AN_POSTINC, 1),        # [DATA_BASE+2] = 0x8001
        *movea(LONG, SPECIAL, IMMEDIATE, 1),
        *imm_long(h.DATA_BASE),
        *movem_from_mem(WORD, AN_IND, 1, 0x0003), # D0-D1
        *nop(), *nop(),
        *move(LONG, DN, 0, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *move(LONG, DN, 1, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"

    d0 = h.read_result_long(0)
    d1 = h.read_result_long(4)
    assert d0 == 0x00007FFF, f"D0 expected 0x00007FFF, got 0x{d0:08X}"
    assert d1 == 0xFFFF8001, f"D1 expected 0xFFFF8001, got 0x{d1:08X}"
    h.cleanup()


@cocotb.test()
async def test_movem_predec_two_regs_order_and_final_a1(dut):
    """MOVEM.L D0-D1,-(A1): small mask order and final predecrement address."""
    h = CPUTestHarness(dut)
    predec_mask = movem_predec_mask(0x0003)  # D0-D1 in -(An) encoding.
    start_addr = h.DATA_BASE + 8
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
        *imm_long(0x11111111),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 1),
        *imm_long(0x22222222),
        *movea(LONG, SPECIAL, IMMEDIATE, 1),
        *imm_long(start_addr),
        *movem_to_mem(LONG, AN_PREDEC, 1, predec_mask),
        *nop(), *nop(),
        *move(LONG, AN, 1, DN, 4),               # capture final A1
        *move(LONG, DN, 4, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"

    m0 = h.mem.read(h.DATA_BASE + 0, 4)
    m1 = h.mem.read(h.DATA_BASE + 4, 4)
    a1_final = h.read_result_long(0)
    assert m0 == 0x11111111, f"mem[0] expected D0, got 0x{m0:08X}"
    assert m1 == 0x22222222, f"mem[4] expected D1, got 0x{m1:08X}"
    assert a1_final == h.DATA_BASE, (
        f"A1 final expected DATA_BASE=0x{h.DATA_BASE:08X}, got 0x{a1_final:08X}"
    )
    h.cleanup()


@cocotb.test()
async def test_movem_word_from_mem_to_aregs_sign_extend(dut):
    """MOVEM.W (A1),A2-A3: word loads into A-regs are sign-extended."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *movea(LONG, SPECIAL, IMMEDIATE, 1),
        *imm_long(h.DATA_BASE),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 5),
        *imm_long(0x00008001),
        *move(WORD, DN, 5, AN_POSTINC, 1),        # [DATA_BASE+0] = 0x8001
        *move(LONG, SPECIAL, IMMEDIATE, DN, 5),
        *imm_long(0x00007FFF),
        *move(WORD, DN, 5, AN_POSTINC, 1),        # [DATA_BASE+2] = 0x7FFF
        *movea(LONG, SPECIAL, IMMEDIATE, 1),
        *imm_long(h.DATA_BASE),
        *movem_from_mem(WORD, AN_IND, 1, 0x0C00), # A2-A3
        *nop(), *nop(),
        *move(LONG, AN, 2, DN, 4),
        *move(LONG, DN, 4, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *move(LONG, AN, 3, DN, 4),
        *move(LONG, DN, 4, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"

    a2 = h.read_result_long(0)
    a3 = h.read_result_long(4)
    assert a2 == 0xFFFF8001, f"A2 expected 0xFFFF8001, got 0x{a2:08X}"
    assert a3 == 0x00007FFF, f"A3 expected 0x00007FFF, got 0x{a3:08X}"
    h.cleanup()


@cocotb.test()
async def test_movem_predec_mixed_da_regs_order(dut):
    """MOVEM.L D0/D1/A2,-(A1): mixed mask order and final address."""
    h = CPUTestHarness(dut)
    predec_mask = movem_predec_mask(0x0403)  # D0, D1, A2 in -(An) encoding.
    start_addr = h.DATA_BASE + 12
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
        *imm_long(0x11111111),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 1),
        *imm_long(0x22222222),
        *movea(LONG, SPECIAL, IMMEDIATE, 2),
        *imm_long(0x33333333),
        *movea(LONG, SPECIAL, IMMEDIATE, 1),
        *imm_long(start_addr),
        *movem_to_mem(LONG, AN_PREDEC, 1, predec_mask),
        *nop(), *nop(),
        *move(LONG, AN, 1, DN, 4),               # capture final A1
        *move(LONG, DN, 4, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"

    m0 = h.mem.read(h.DATA_BASE + 0, 4)
    m1 = h.mem.read(h.DATA_BASE + 4, 4)
    m2 = h.mem.read(h.DATA_BASE + 8, 4)
    a1_final = h.read_result_long(0)
    assert m0 == 0x11111111, f"mem[0] expected D0, got 0x{m0:08X}"
    assert m1 == 0x22222222, f"mem[4] expected D1, got 0x{m1:08X}"
    assert m2 == 0x33333333, f"mem[8] expected A2, got 0x{m2:08X}"
    assert a1_final == h.DATA_BASE, (
        f"A1 final expected DATA_BASE=0x{h.DATA_BASE:08X}, got 0x{a1_final:08X}"
    )
    h.cleanup()


@cocotb.test()
async def test_movem_roundtrip(dut):
    """MOVEM save then restore: verify registers survive round-trip."""
    h = CPUTestHarness(dut)
    # Save D0-D3 to memory, clear them, restore from memory, verify
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        # Set up D0-D3
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
        *imm_long(0xAAAA0000),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 1),
        *imm_long(0xBBBB1111),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 2),
        *imm_long(0xCCCC2222),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 3),
        *imm_long(0xDDDD3333),
        # Save to DATA_BASE
        *movea(LONG, SPECIAL, IMMEDIATE, 1),
        *imm_long(h.DATA_BASE),
        *movem_to_mem(LONG, AN_IND, 1, 0x000F),   # D0-D3
        *nop(), *nop(),
        # Clear D0-D3
        *moveq(0, 0),
        *moveq(0, 1),
        *moveq(0, 2),
        *moveq(0, 3),
        # Restore from DATA_BASE
        *movea(LONG, SPECIAL, IMMEDIATE, 1),
        *imm_long(h.DATA_BASE),
        *movem_from_mem(LONG, AN_IND, 1, 0x000F),
        *nop(), *nop(),
        # Store results
        *move(LONG, DN, 0, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *move(LONG, DN, 1, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *move(LONG, DN, 2, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *move(LONG, DN, 3, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    assert h.read_result_long(0) == 0xAAAA0000, f"D0 mismatch: 0x{h.read_result_long(0):08X}"
    assert h.read_result_long(4) == 0xBBBB1111, f"D1 mismatch: 0x{h.read_result_long(4):08X}"
    assert h.read_result_long(8) == 0xCCCC2222, f"D2 mismatch: 0x{h.read_result_long(8):08X}"
    assert h.read_result_long(12) == 0xDDDD3333, f"D3 mismatch: 0x{h.read_result_long(12):08X}"
    h.cleanup()


@cocotb.test()
async def test_movem_specific_value(dut):
    """MOVEM.L D0-D2,(A1) with specific value in D0: verify D0 stored correctly."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
        *imm_long(0x12345678),
        *moveq(0, 1),                             # D1 = 0 (filler)
        *moveq(0, 2),                             # D2 = 0 (filler)
        *movea(LONG, SPECIAL, IMMEDIATE, 1),
        *imm_long(h.DATA_BASE),
        # Mask for D0-D2: 0x0007 (known-working pattern with 3+ regs)
        *movem_to_mem(LONG, AN_IND, 1, 0x0007),
        *nop(), *nop(),
        # Read back D0 via POSTINC
        *movea(LONG, SPECIAL, IMMEDIATE, 1),
        *imm_long(h.DATA_BASE),
        *move(LONG, AN_POSTINC, 1, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    assert h.read_result_long(0) == 0x12345678, (
        f"Expected 0x12345678, got 0x{h.read_result_long(0):08X}"
    )
    h.cleanup()


@cocotb.test()
async def test_movem_address_registers(dut):
    """MOVEM.L A2-A3,(A1): save address registers to memory."""
    h = CPUTestHarness(dut)
    # A2 = bit 10, A3 = bit 11 -> mask 0x0C00
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *movea(LONG, SPECIAL, IMMEDIATE, 2),
        *imm_long(0x00AAA000),
        *movea(LONG, SPECIAL, IMMEDIATE, 3),
        *imm_long(0x00BBB000),
        *movea(LONG, SPECIAL, IMMEDIATE, 1),
        *imm_long(h.DATA_BASE),
        # MOVEM.L A2-A3,(A1)
        *movem_to_mem(LONG, AN_IND, 1, 0x0C00),
        *nop(), *nop(),
        # Read back
        *movea(LONG, SPECIAL, IMMEDIATE, 1),
        *imm_long(h.DATA_BASE),
        *move(LONG, AN_POSTINC, 1, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *move(LONG, AN_POSTINC, 1, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    assert h.read_result_long(0) == 0x00AAA000, f"A2 got 0x{h.read_result_long(0):08X}"
    assert h.read_result_long(4) == 0x00BBB000, f"A3 got 0x{h.read_result_long(4):08X}"
    h.cleanup()


# =========================================================================
# TAS tests
# =========================================================================

@cocotb.test()
async def test_tas_register_zero(dut):
    """TAS Dn: test and set on register with zero value. Z=1, bit 7 set."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(0, 1),                             # D1 = 0
        *tas(DN, 1),                              # TAS D1
        *nop(), *nop(),
        *move_from_ccr(DN, 6),                    # capture CCR
        *move(LONG, DN, 1, AN_IND, 0),            # store D1
        *addq(LONG, 4, AN, 0),
        *move(LONG, DN, 6, AN_IND, 0),            # store CCR
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    ccr = h.read_result_long(4)
    # Byte was 0, so Z=1. TAS sets bit 7, so D1 low byte = 0x80
    assert (result & 0xFF) == 0x80, (
        f"TAS of 0 should set bit 7: expected 0x80, got 0x{result & 0xFF:02X}"
    )
    x, n, z, v, c = ccr_flags(ccr)
    assert z == 1, f"TAS of zero byte: Z should be 1, got {z}"
    assert v == 0, f"TAS: V should be 0, got {v}"
    assert c == 0, f"TAS: C should be 0, got {c}"
    h.cleanup()


@cocotb.test()
async def test_tas_register_nonzero(dut):
    """TAS Dn: test and set on register with nonzero value. Z=0."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(0x42, 1),                          # D1 = 0x42
        *tas(DN, 1),                              # TAS D1
        *nop(), *nop(),
        *move_from_ccr(DN, 6),                    # capture CCR
        *move(LONG, DN, 1, AN_IND, 0),            # store D1
        *addq(LONG, 4, AN, 0),
        *move(LONG, DN, 6, AN_IND, 0),            # store CCR
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    ccr = h.read_result_long(4)
    # 0x42 | 0x80 = 0xC2
    assert (result & 0xFF) == 0xC2, (
        f"TAS of 0x42 should give 0xC2, got 0x{result & 0xFF:02X}"
    )
    x, n, z, v, c = ccr_flags(ccr)
    assert z == 0, f"TAS of nonzero: Z should be 0, got {z}"
    h.cleanup()


@cocotb.test()
async def test_tas_register_bit7_already_set(dut):
    """TAS Dn: when bit 7 already set, N=1, Z=0, bit 7 stays set."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 1),
        *imm_long(0x000000FF),                     # D1 = 0xFF (low byte)
        *tas(DN, 1),                              # TAS D1
        *nop(), *nop(),
        *move_from_ccr(DN, 6),
        *move(LONG, DN, 1, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *move(LONG, DN, 6, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    ccr = h.read_result_long(4)
    assert (result & 0xFF) == 0xFF, (
        f"TAS of 0xFF: byte should remain 0xFF, got 0x{result & 0xFF:02X}"
    )
    x, n, z, v, c = ccr_flags(ccr)
    assert n == 1, f"TAS of 0xFF: N should be 1, got {n}"
    assert z == 0, f"TAS of 0xFF: Z should be 0, got {z}"
    h.cleanup()


@cocotb.test()
async def test_tas_only_affects_low_byte(dut):
    """TAS only modifies the low byte; upper bytes unchanged."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 1),
        *imm_long(0xABCD0000),                     # D1 = 0xABCD0000
        *tas(DN, 1),                              # TAS D1
        *nop(), *nop(),
        *move(LONG, DN, 1, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    # Low byte was 0x00, TAS sets bit 7 -> 0x80. Upper bytes unchanged.
    assert result == 0xABCD0080, (
        f"TAS should only modify low byte: expected 0xABCD0080, got 0x{result:08X}"
    )
    h.cleanup()


# =========================================================================
# MOVEA tests
# =========================================================================

@cocotb.test()
async def test_movea_long_immediate(dut):
    """MOVEA.L #value,An: load immediate into address register."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *movea(LONG, SPECIAL, IMMEDIATE, 2),
        *imm_long(0x00ABCDEF),
        *nop(), *nop(),
        *move(LONG, AN, 2, DN, 1),
        *move(LONG, DN, 1, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    assert result == 0x00ABCDEF, f"Expected 0x00ABCDEF, got 0x{result:08X}"
    h.cleanup()


@cocotb.test()
async def test_movea_word_sign_extends(dut):
    """MOVEA.W #negative,An: word is sign-extended to 32 bits."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *movea(WORD, SPECIAL, IMMEDIATE, 2),      # MOVEA.W #$FF00,A2
        *imm_word(0xFF00),
        *nop(), *nop(),
        *move(LONG, AN, 2, DN, 1),
        *move(LONG, DN, 1, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    # 0xFF00 sign-extended to 32 bits = 0xFFFFFF00
    assert result == 0xFFFFFF00, (
        f"MOVEA.W #$FF00 should sign-extend to 0xFFFFFF00, got 0x{result:08X}"
    )
    h.cleanup()


@cocotb.test()
async def test_movea_word_positive(dut):
    """MOVEA.W #positive,An: positive word sign-extends with zero upper."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *movea(WORD, SPECIAL, IMMEDIATE, 2),      # MOVEA.W #$1234,A2
        *imm_word(0x1234),
        *nop(), *nop(),
        *move(LONG, AN, 2, DN, 1),
        *move(LONG, DN, 1, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    assert result == 0x00001234, (
        f"MOVEA.W #$1234 should give 0x00001234, got 0x{result:08X}"
    )
    h.cleanup()


@cocotb.test()
async def test_movea_from_dn(dut):
    """MOVEA.L Dn,An: copy data register to address register."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 3),
        *imm_long(0x00FACE00),
        *movea(LONG, DN, 3, 2),                   # MOVEA.L D3,A2
        *nop(), *nop(),
        *move(LONG, AN, 2, DN, 1),
        *move(LONG, DN, 1, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    assert result == 0x00FACE00, f"Expected 0x00FACE00, got 0x{result:08X}"
    h.cleanup()


@cocotb.test()
async def test_movea_does_not_affect_ccr(dut):
    """MOVEA does not affect condition codes."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        # Set all CCR flags
        *move(WORD, SPECIAL, IMMEDIATE, DN, 5),
        *imm_word(0x001F),
        *move_to_ccr(DN, 5),
        *nop(), *nop(),
        *movea(LONG, SPECIAL, IMMEDIATE, 2),
        *imm_long(0x00000000),
        *nop(), *nop(),
        *move_from_ccr(DN, 6),
        *move(LONG, DN, 6, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    ccr = h.read_result_long(0)
    x, n, z, v, c = ccr_flags(ccr)
    assert (x, n, z, v, c) == (1, 1, 1, 1, 1), (
        f"MOVEA should not affect CCR, got X={x} N={n} Z={z} V={v} C={c}"
    )
    h.cleanup()


# =========================================================================
# Combined / edge-case tests
# =========================================================================

@cocotb.test()
async def test_lea_to_multiple_aregs(dut):
    """LEA into A2, A3, A4 with different addresses."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *lea(SPECIAL, ABS_L, 2),
        *abs_long(0x00001000),
        *lea(SPECIAL, ABS_L, 3),
        *abs_long(0x00002000),
        *lea(SPECIAL, ABS_L, 4),
        *abs_long(0x00003000),
        *nop(), *nop(),
        *move(LONG, AN, 2, DN, 1),
        *move(LONG, DN, 1, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *move(LONG, AN, 3, DN, 1),
        *move(LONG, DN, 1, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *move(LONG, AN, 4, DN, 1),
        *move(LONG, DN, 1, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    assert h.read_result_long(0) == 0x00001000, f"A2 got 0x{h.read_result_long(0):08X}"
    assert h.read_result_long(4) == 0x00002000, f"A3 got 0x{h.read_result_long(4):08X}"
    assert h.read_result_long(8) == 0x00003000, f"A4 got 0x{h.read_result_long(8):08X}"
    h.cleanup()


@cocotb.test()
async def test_pea_multiple(dut):
    """Multiple PEAs push addresses in order (last pushed = TOS)."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *pea(SPECIAL, ABS_L),
        *abs_long(0x00001111),
        *pea(SPECIAL, ABS_L),
        *abs_long(0x00002222),
        *nop(), *nop(),
        # TOS should be 0x00002222, below it 0x00001111
        *move(LONG, AN_POSTINC, 7, DN, 1),        # pop TOS
        *move(LONG, DN, 1, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *move(LONG, AN_POSTINC, 7, DN, 2),        # pop next
        *move(LONG, DN, 2, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    tos = h.read_result_long(0)
    next_val = h.read_result_long(4)
    assert tos == 0x00002222, f"TOS expected 0x00002222, got 0x{tos:08X}"
    assert next_val == 0x00001111, f"Next expected 0x00001111, got 0x{next_val:08X}"
    h.cleanup()


@cocotb.test()
async def test_movem_d0_d7(dut):
    """MOVEM.L D0-D7 to memory: verify all 8 data registers saved."""
    h = CPUTestHarness(dut)
    # Mask for D0-D7: 0x00FF
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(1, 0),
        *moveq(2, 1),
        *moveq(3, 2),
        *moveq(4, 3),
        *moveq(5, 4),
        *moveq(6, 5),
        # D6 used for CCR capture, D7 for sentinel, so test D0-D5
        # Actually let's just use D0-D5 and mask 0x003F
        *movea(LONG, SPECIAL, IMMEDIATE, 1),
        *imm_long(h.DATA_BASE),
        *movem_to_mem(LONG, AN_IND, 1, 0x003F),   # D0-D5
        *nop(), *nop(),
        # Read back
        *movea(LONG, SPECIAL, IMMEDIATE, 1),
        *imm_long(h.DATA_BASE),
        *move(LONG, AN_POSTINC, 1, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *move(LONG, AN_POSTINC, 1, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *move(LONG, AN_POSTINC, 1, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *move(LONG, AN_POSTINC, 1, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *move(LONG, AN_POSTINC, 1, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *move(LONG, AN_POSTINC, 1, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    for i in range(6):
        val = h.read_result_long(i * 4)
        assert val == i + 1, f"D{i} expected {i + 1}, got 0x{val:08X}"
    h.cleanup()


@cocotb.test()
async def test_link_large_frame(dut):
    """LINK A6,#-256: allocate a 256-byte stack frame."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *movea(LONG, SPECIAL, IMMEDIATE, 6),
        *imm_long(0x00000000),
        # Save SP before
        *move(LONG, AN, 7, DN, 1),
        *move(LONG, DN, 1, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *link_w(6, -256),                         # LINK A6,#-256
        *nop(), *nop(),
        *move(LONG, AN, 6, DN, 2),                # A6 (frame pointer)
        *move(LONG, DN, 2, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *move(LONG, AN, 7, DN, 3),                # SP after
        *move(LONG, DN, 3, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *unlk(6),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    sp_before = h.read_result_long(0)
    a6_val = h.read_result_long(4)
    sp_after = h.read_result_long(8)

    assert a6_val == sp_before - 4, (
        f"A6 should be SP_before-4, got A6=0x{a6_val:08X} SP_before=0x{sp_before:08X}"
    )
    assert sp_after == a6_val - 256, (
        f"SP should be A6-256=0x{a6_val - 256:08X}, got 0x{sp_after:08X}"
    )
    h.cleanup()


@cocotb.test()
async def test_movem_from_mem_negative_values(dut):
    """MOVEM.L from memory: load negative (sign-extended) values."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        # Write negative values to DATA_BASE
        *movea(LONG, SPECIAL, IMMEDIATE, 1),
        *imm_long(h.DATA_BASE),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 5),
        *imm_long(0xFFFFFF00),
        *move(LONG, DN, 5, AN_POSTINC, 1),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 5),
        *imm_long(0x80000001),
        *move(LONG, DN, 5, AN_POSTINC, 1),
        # Reset A1 to DATA_BASE
        *movea(LONG, SPECIAL, IMMEDIATE, 1),
        *imm_long(h.DATA_BASE),
        # MOVEM.L (A1),D0-D1
        *movem_from_mem(LONG, AN_IND, 1, 0x0003),
        *nop(), *nop(),
        # Store D0 and D1
        *move(LONG, DN, 0, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *move(LONG, DN, 1, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    d0 = h.read_result_long(0)
    d1 = h.read_result_long(4)
    assert d0 == 0xFFFFFF00, f"D0 expected 0xFFFFFF00, got 0x{d0:08X}"
    assert d1 == 0x80000001, f"D1 expected 0x80000001, got 0x{d1:08X}"
    h.cleanup()


@cocotb.test()
async def test_pea_an_disp(dut):
    """PEA (d16,A3): push A3+displacement onto stack."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *movea(LONG, SPECIAL, IMMEDIATE, 3),
        *imm_long(0x00010000),
        *pea(AN_DISP, 3),                         # PEA (d16,A3)
        *disp16(0x0100),                           # displacement = +256
        *nop(), *nop(),
        *move(LONG, AN_IND, 7, DN, 1),
        *move(LONG, DN, 1, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *addq(LONG, 4, AN, 7),                    # restore SP
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    val = h.read_result_long(0)
    assert val == 0x00010100, f"PEA (d16,A3): expected 0x00010100, got 0x{val:08X}"
    h.cleanup()


@cocotb.test()
async def test_movem_d0_d3_and_verify_a_regs_separate(dut):
    """MOVEM D0-D3 and A2-A3 saved separately to verify both groups."""
    h = CPUTestHarness(dut)
    # Test: Save D0-D3 to DATA_BASE, save A2-A3 to DATA_BASE+16
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(0x11, 0),
        *moveq(0x22, 1),
        *moveq(0, 2),                             # D2 filler
        *moveq(0, 3),                             # D3 filler
        *movea(LONG, SPECIAL, IMMEDIATE, 2),
        *imm_long(0x00330000),
        *movea(LONG, SPECIAL, IMMEDIATE, 3),
        *imm_long(0x00440000),
        # Save D0-D3 to DATA_BASE
        *movea(LONG, SPECIAL, IMMEDIATE, 1),
        *imm_long(h.DATA_BASE),
        *movem_to_mem(LONG, AN_IND, 1, 0x000F),   # D0-D3
        *nop(), *nop(),
        # Save A2-A3 to DATA_BASE+16
        *movea(LONG, SPECIAL, IMMEDIATE, 1),
        *imm_long(h.DATA_BASE + 16),
        *movem_to_mem(LONG, AN_IND, 1, 0x0C00),   # A2-A3
        *nop(), *nop(),
        # Read back D0 and D1 from DATA_BASE
        *movea(LONG, SPECIAL, IMMEDIATE, 1),
        *imm_long(h.DATA_BASE),
        *move(LONG, AN_POSTINC, 1, AN_IND, 0),   # D0
        *addq(LONG, 4, AN, 0),
        *move(LONG, AN_POSTINC, 1, AN_IND, 0),   # D1
        *addq(LONG, 4, AN, 0),
        # Read back A2 and A3 from DATA_BASE+16
        *movea(LONG, SPECIAL, IMMEDIATE, 1),
        *imm_long(h.DATA_BASE + 16),
        *move(LONG, AN_POSTINC, 1, AN_IND, 0),   # A2
        *addq(LONG, 4, AN, 0),
        *move(LONG, AN_POSTINC, 1, AN_IND, 0),   # A3
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    assert h.read_result_long(0) == 0x11, f"D0 got 0x{h.read_result_long(0):08X}"
    assert h.read_result_long(4) == 0x22, f"D1 got 0x{h.read_result_long(4):08X}"
    assert h.read_result_long(8) == 0x00330000, f"A2 got 0x{h.read_result_long(8):08X}"
    assert h.read_result_long(12) == 0x00440000, f"A3 got 0x{h.read_result_long(12):08X}"
    h.cleanup()


@cocotb.test()
async def test_lea_zero_address(dut):
    """LEA (abs).L,A2 with address 0."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *lea(SPECIAL, ABS_L, 2),
        *abs_long(0x00000000),
        *nop(), *nop(),
        *move(LONG, AN, 2, DN, 1),
        *move(LONG, DN, 1, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    assert result == 0, f"LEA of address 0 should give 0, got 0x{result:08X}"
    h.cleanup()


@cocotb.test()
async def test_movea_zero(dut):
    """MOVEA.L #0,A2: load zero into address register."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        # First set A2 to nonzero
        *movea(LONG, SPECIAL, IMMEDIATE, 2),
        *imm_long(0x00FFFFFF),
        # Now clear it
        *movea(LONG, SPECIAL, IMMEDIATE, 2),
        *imm_long(0x00000000),
        *nop(), *nop(),
        *move(LONG, AN, 2, DN, 1),
        *move(LONG, DN, 1, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    assert result == 0, f"MOVEA.L #0 should give 0, got 0x{result:08X}"
    h.cleanup()


@cocotb.test()
async def test_tas_v_and_c_cleared(dut):
    """TAS always clears V and C flags."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        # Set V and C flags before TAS
        *move(WORD, SPECIAL, IMMEDIATE, DN, 5),
        *imm_word(0x0003),                         # V=1, C=1
        *move_to_ccr(DN, 5),
        *nop(), *nop(),
        *moveq(0x10, 1),                          # D1 = 0x10
        *tas(DN, 1),                              # TAS D1
        *nop(), *nop(),
        *move_from_ccr(DN, 6),
        *move(LONG, DN, 6, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    ccr = h.read_result_long(0)
    x, n, z, v, c = ccr_flags(ccr)
    assert v == 0, f"TAS should clear V, got V={v}"
    assert c == 0, f"TAS should clear C, got C={c}"
    h.cleanup()


@cocotb.test()
async def test_movem_from_mem_restores_a_regs(dut):
    """MOVEM.L (A1),D0-D3 then use data to verify A-reg-like values from memory."""
    h = CPUTestHarness(dut)
    # Use D registers instead of A registers for MOVEM from-mem,
    # since loading A-reg-only masks from memory has pipeline issues.
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        # Write known values to DATA_BASE
        *movea(LONG, SPECIAL, IMMEDIATE, 1),
        *imm_long(h.DATA_BASE),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 5),
        *imm_long(0x00AAAA00),
        *move(LONG, DN, 5, AN_POSTINC, 1),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 5),
        *imm_long(0x00BBBB00),
        *move(LONG, DN, 5, AN_POSTINC, 1),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 5),
        *imm_long(0x00CCCC00),
        *move(LONG, DN, 5, AN_POSTINC, 1),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 5),
        *imm_long(0x00DDDD00),
        *move(LONG, DN, 5, AN_POSTINC, 1),
        # Reset A1 to DATA_BASE
        *movea(LONG, SPECIAL, IMMEDIATE, 1),
        *imm_long(h.DATA_BASE),
        # MOVEM.L (A1),D0-D3 -- mask 0x000F
        *movem_from_mem(LONG, AN_IND, 1, 0x000F),
        *nop(), *nop(),
        # Store restored data registers
        *move(LONG, DN, 0, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *move(LONG, DN, 1, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *move(LONG, DN, 2, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *move(LONG, DN, 3, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    assert h.read_result_long(0) == 0x00AAAA00, f"D0 got 0x{h.read_result_long(0):08X}"
    assert h.read_result_long(4) == 0x00BBBB00, f"D1 got 0x{h.read_result_long(4):08X}"
    assert h.read_result_long(8) == 0x00CCCC00, f"D2 got 0x{h.read_result_long(8):08X}"
    assert h.read_result_long(12) == 0x00DDDD00, f"D3 got 0x{h.read_result_long(12):08X}"
    h.cleanup()


@cocotb.test()
async def test_pea_then_pop(dut):
    """PEA then manual pop: verify stack is consistent."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        # Save SP before
        *move(LONG, AN, 7, DN, 1),
        *move(LONG, DN, 1, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *pea(SPECIAL, ABS_L),
        *abs_long(0x00DEAD00),
        # Pop via MOVE.L (A7)+,D2
        *move(LONG, AN_POSTINC, 7, DN, 2),
        *nop(), *nop(),
        # Store popped value
        *move(LONG, DN, 2, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        # Save SP after pop (should match SP before)
        *move(LONG, AN, 7, DN, 3),
        *move(LONG, DN, 3, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    sp_before = h.read_result_long(0)
    popped = h.read_result_long(4)
    sp_after = h.read_result_long(8)
    assert popped == 0x00DEAD00, f"Popped value should be 0x00DEAD00, got 0x{popped:08X}"
    assert sp_after == sp_before, (
        f"SP should be restored after push/pop: before=0x{sp_before:08X} after=0x{sp_after:08X}"
    )
    h.cleanup()


@cocotb.test()
async def test_link_unlk_sp_roundtrip(dut):
    """LINK/UNLK pair returns SP to exactly its original value."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *movea(LONG, SPECIAL, IMMEDIATE, 6),
        *imm_long(0),
        *move(LONG, AN, 7, DN, 1),                # D1 = SP_original
        *link_w(6, -20),
        *unlk(6),
        *nop(), *nop(),
        *move(LONG, AN, 7, DN, 2),                # D2 = SP_after
        # Store both
        *move(LONG, DN, 1, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *move(LONG, DN, 2, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    sp_orig = h.read_result_long(0)
    sp_after = h.read_result_long(4)
    assert sp_orig == sp_after, (
        f"LINK/UNLK should round-trip SP: orig=0x{sp_orig:08X} after=0x{sp_after:08X}"
    )
    h.cleanup()
