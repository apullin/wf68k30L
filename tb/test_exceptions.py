"""
Exception and trap compliance tests for WF68K30L (Phase 5).

Tests MC68030 exception handling including:
  - Illegal instruction (vector 4)
  - Line-A emulator (vector 10)
  - Line-F emulator (vector 11)
  - Divide by zero (vector 5)
  - TRAPV with and without overflow (vector 7)
  - TRAP #n stack frame verification (vectors 32-47)
  - CHK exception (vector 6) for out-of-range values
  - Privilege violation (vector 8)
  - RTE return from exception

Each exception test follows the same pattern:
  1. Set up the exception vector to point to handler code
  2. Place handler code at a known address (writes marker + sentinel)
  3. Execute the triggering instruction
  4. Verify the handler was reached via the sentinel

Handler routines write a marker and sentinel directly (no RTE return)
because MC68030 exception frame format makes RTE return unreliable
in this test harness.

IMPORTANT: The CPU starts in supervisor mode with SSP = 0x001000.
IMPORTANT: MOVE from CCR must be done BEFORE any MOVE stores.
IMPORTANT: MOVEQ sign-extends 8-bit values. Marker values > 0x7F will
           appear sign-extended when read as 32-bit longs.
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles

from cpu_harness import CPUTestHarness
from m68k_encode import (
    BYTE, WORD, LONG,
    DN, AN, AN_IND, AN_POSTINC, AN_PREDEC, AN_DISP, SPECIAL, ABS_L, IMMEDIATE,
    moveq, move, movea, move_to_abs_long, nop, addq, subq,
    addi, subi, add,
    jmp, jmp_abs, jsr, jsr_abs, rts, rte,
    trap, trapv, illegal,
    chk_w,
    move_from_ccr, move_to_ccr, move_from_sr, move_to_sr,
    divu_w, divs_w,
    imm_long, imm_word, abs_long, disp16,
    stop,
)


# ---------------------------------------------------------------------------
# Helper: generate exception handler code
# ---------------------------------------------------------------------------

HANDLER_BASE = 0x000400  # Default base address for handler code


def _handler_code(h, marker_reg, marker_val):
    """Generate handler code that writes a marker to RESULT_BASE and sentinel.

    NOTE: marker_val must be 0x00-0x7F to avoid MOVEQ sign extension issues.
    """
    return [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(marker_val, marker_reg),
        *move(LONG, DN, marker_reg, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]


def _handler_code_read_sp(h, marker_val):
    """Handler that reads SP (A7) and stores it along with a marker.

    Writes to RESULT_BASE:
      offset 0: marker_val (long)
      offset 4: SP value (long)
    Then sentinel.
    """
    return [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(marker_val, 1),
        *move(LONG, DN, 1, AN_IND, 0),       # [RESULT+0] = marker
        *addq(LONG, 4, AN, 0),
        *move(LONG, AN, 7, DN, 2),            # D2 = SP
        *move(LONG, DN, 2, AN_IND, 0),        # [RESULT+4] = SP
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]


def _handler_code_read_frame(h, marker_val):
    """Handler that reads the exception stack frame and stores it.

    MC68030 group 1/2 exception frame (format $0, 4-word frame):
      SP+0: SR (word)
      SP+2: PC high (word)
      SP+4: PC low (word)
      SP+6: format/vector offset (word)

    Reads individual words to avoid alignment issues with MOVE.L from
    non-long-aligned addresses.

    Writes to RESULT_BASE:
      offset 0:  marker_val (long)
      offset 4:  SP value (long)
      offset 8:  [SP+0] - saved SR (long, word zero-extended)
      offset 12: saved PC reconstructed from two words (long)
      offset 16: [SP+6] - format/vector (long, word zero-extended)
    Then sentinel.
    """
    return [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        # marker
        *moveq(marker_val, 1),
        *move(LONG, DN, 1, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        # SP
        *move(LONG, AN, 7, DN, 2),
        *move(LONG, DN, 2, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        # Read SR from stack: MOVE.W (SP),D3 then store as long
        *move(WORD, AN_IND, 7, DN, 3),       # D3 = [SP] (saved SR, word)
        *move(LONG, DN, 3, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        # Read PC from stack as two separate words to avoid alignment issues.
        # SP+2 = PC high word, SP+4 = PC low word
        # Use (d16,A7) addressing: MOVE.W (2,A7),D4
        *move(WORD, AN_DISP, 7, DN, 4),      # D4 = [SP+2] (PC high word)
        *disp16(2),
        # Shift D4 left by 16: SWAP D4 then clear low word
        # Actually, easier: store PC high word, read PC low word, reconstruct
        # Store PC high word shifted: use SWAP
        0x4844,                                # SWAP D4
        *move(WORD, AN_DISP, 7, DN, 5),      # D5 = [SP+4] (PC low word)
        *disp16(4),
        # Combine: D4 has high word in low 16 bits (after SWAP it's in high 16),
        # now OR in D5 for low 16 bits
        # D4 = (PC_high << 16), D5 = PC_low
        # Use: MOVE.W D5,D4 to put low word into D4's low half
        *move(WORD, DN, 5, DN, 4),            # D4[15:0] = D5[15:0]
        *move(LONG, DN, 4, AN_IND, 0),        # store reconstructed PC
        *addq(LONG, 4, AN, 0),
        # Read format/vector offset from stack: at SP+6
        *move(WORD, AN_DISP, 7, DN, 5),      # D5 = [SP+6] (format/vector, word)
        *disp16(6),
        *move(LONG, DN, 5, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]


# =========================================================================
# Illegal instruction tests (vector 4)
# =========================================================================

@cocotb.test()
async def test_illegal_instruction(dut):
    """ILLEGAL (0x4AFC): triggers vector 4 handler."""
    h = CPUTestHarness(dut)
    handler_addr = HANDLER_BASE
    vector_addr = 4 * 4  # 0x010

    program = [
        *moveq(0, 1),
        *illegal(),                            # ILLEGAL opcode
        *nop(), *nop(),
        *h.sentinel_program(),
    ]

    await h.setup(program)
    h.mem.load_long(vector_addr, handler_addr)
    h.mem.load_words(handler_addr, _handler_code(h, 1, 0x44))

    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    d1_val = h.read_result_long(0)
    assert d1_val == 0x44, (
        f"ILLEGAL should trigger vector 4 handler: D1 expected 0x44, got 0x{d1_val:08X}"
    )
    h.cleanup()


@cocotb.test()
async def test_illegal_instruction_preserves_regs(dut):
    """ILLEGAL: registers set before the exception are visible in handler."""
    h = CPUTestHarness(dut)
    handler_addr = HANDLER_BASE
    vector_addr = 4 * 4

    handler_code = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *move(LONG, DN, 2, AN_IND, 0),       # store D2 (set before exception)
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]

    program = [
        *moveq(0x37, 2),                      # D2 = 0x37
        *illegal(),
        *nop(), *nop(),
        *h.sentinel_program(),
    ]

    await h.setup(program)
    h.mem.load_long(vector_addr, handler_addr)
    h.mem.load_words(handler_addr, handler_code)

    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    d2_val = h.read_result_long(0)
    assert d2_val == 0x37, (
        f"ILLEGAL handler should see D2=0x37, got 0x{d2_val:08X}"
    )
    h.cleanup()


@cocotb.test()
async def test_illegal_instruction_stack_frame(dut):
    """ILLEGAL: verify exception stack frame contains correct PC and vector."""
    h = CPUTestHarness(dut)
    handler_addr = HANDLER_BASE + 0x200
    vector_addr = 4 * 4

    # Program layout:
    #   0x100: MOVEQ #0,D1            (2 bytes)
    #   0x102: ILLEGAL (0x4AFC)       (2 bytes)  <-- exception PC
    #   0x104: NOP ...
    program = [
        *moveq(0, 1),                         # 0x100
        *illegal(),                            # 0x102: ILLEGAL
        *nop(), *nop(),
        *h.sentinel_program(),
    ]

    await h.setup(program)
    h.mem.load_long(vector_addr, handler_addr)
    h.mem.load_words(handler_addr, _handler_code_read_frame(h, 0x44))

    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    marker = h.read_result_long(0)
    assert marker == 0x44, f"Marker expected 0x44, got 0x{marker:08X}"
    # SP should have been decremented from SSP_INIT
    sp = h.read_result_long(4)
    assert sp < h.SSP_INIT, f"SP should be below SSP_INIT, got 0x{sp:08X}"
    # Saved PC should point at the ILLEGAL instruction (0x102)
    saved_pc = h.read_result_long(12)
    assert saved_pc == 0x102, (
        f"Saved PC should be 0x102 (ILLEGAL addr), got 0x{saved_pc:08X}"
    )
    h.cleanup()


# =========================================================================
# Line-A emulator tests (vector 10, 0x0A)
# =========================================================================

@cocotb.test()
async def test_line_a_emulator_0xA000(dut):
    """Line-A (0xA000): triggers vector 10 handler."""
    h = CPUTestHarness(dut)
    handler_addr = HANDLER_BASE + 0x100
    vector_addr = 10 * 4  # 0x028

    program = [
        *moveq(0, 1),
        0xA000,                                # Line-A opcode
        *nop(), *nop(),
        *h.sentinel_program(),
    ]

    await h.setup(program)
    h.mem.load_long(vector_addr, handler_addr)
    h.mem.load_words(handler_addr, _handler_code(h, 1, 0x0A))

    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    d1_val = h.read_result_long(0)
    assert d1_val == 0x0A, (
        f"Line-A 0xA000 should trigger vector 10: D1 expected 0x0A, got 0x{d1_val:08X}"
    )
    h.cleanup()


@cocotb.test()
async def test_line_a_emulator_0xA123(dut):
    """Line-A (0xA123): different Line-A opcode triggers same vector 10."""
    h = CPUTestHarness(dut)
    handler_addr = HANDLER_BASE + 0x100
    vector_addr = 10 * 4

    program = [
        *moveq(0, 1),
        0xA123,                                # Line-A opcode variant
        *nop(), *nop(),
        *h.sentinel_program(),
    ]

    await h.setup(program)
    h.mem.load_long(vector_addr, handler_addr)
    h.mem.load_words(handler_addr, _handler_code(h, 1, 0x0A))

    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    d1_val = h.read_result_long(0)
    assert d1_val == 0x0A, (
        f"Line-A 0xA123 should trigger vector 10: D1 expected 0x0A, got 0x{d1_val:08X}"
    )
    h.cleanup()


@cocotb.test()
async def test_line_a_emulator_0xAFFF(dut):
    """Line-A (0xAFFF): max Line-A opcode triggers vector 10."""
    h = CPUTestHarness(dut)
    handler_addr = HANDLER_BASE + 0x100
    vector_addr = 10 * 4

    program = [
        *moveq(0, 1),
        0xAFFF,                                # Line-A opcode max
        *nop(), *nop(),
        *h.sentinel_program(),
    ]

    await h.setup(program)
    h.mem.load_long(vector_addr, handler_addr)
    h.mem.load_words(handler_addr, _handler_code(h, 1, 0x0A))

    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    d1_val = h.read_result_long(0)
    assert d1_val == 0x0A, (
        f"Line-A 0xAFFF should trigger vector 10: got 0x{d1_val:08X}"
    )
    h.cleanup()


# =========================================================================
# Line-F emulator tests (vector 11, 0x0B)
# =========================================================================

@cocotb.test()
async def test_line_f_emulator_0xF000(dut):
    """Line-F (0xF000): triggers vector 11 handler."""
    h = CPUTestHarness(dut)
    handler_addr = HANDLER_BASE + 0x180
    vector_addr = 11 * 4  # 0x02C

    program = [
        *moveq(0, 1),
        0xF000,                                # Line-F opcode
        *nop(), *nop(),
        *h.sentinel_program(),
    ]

    await h.setup(program)
    h.mem.load_long(vector_addr, handler_addr)
    h.mem.load_words(handler_addr, _handler_code(h, 1, 0x0F))

    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    d1_val = h.read_result_long(0)
    assert d1_val == 0x0F, (
        f"Line-F 0xF000 should trigger vector 11: D1 expected 0x0F, got 0x{d1_val:08X}"
    )
    h.cleanup()


@cocotb.test()
async def test_line_f_emulator_0xF123(dut):
    """Line-F (0xF123): different Line-F opcode triggers same vector 11."""
    h = CPUTestHarness(dut)
    handler_addr = HANDLER_BASE + 0x180
    vector_addr = 11 * 4

    program = [
        *moveq(0, 1),
        0xF123,                                # Line-F opcode variant
        *nop(), *nop(),
        *h.sentinel_program(),
    ]

    await h.setup(program)
    h.mem.load_long(vector_addr, handler_addr)
    h.mem.load_words(handler_addr, _handler_code(h, 1, 0x0F))

    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    d1_val = h.read_result_long(0)
    assert d1_val == 0x0F, (
        f"Line-F 0xF123 should trigger vector 11: got 0x{d1_val:08X}"
    )
    h.cleanup()


@cocotb.test()
async def test_line_f_emulator_0xFFFF(dut):
    """Line-F (0xFFFF): max Line-F opcode triggers vector 11."""
    h = CPUTestHarness(dut)
    handler_addr = HANDLER_BASE + 0x180
    vector_addr = 11 * 4

    program = [
        *moveq(0, 1),
        0xFFFF,                                # Line-F opcode max
        *nop(), *nop(),
        *h.sentinel_program(),
    ]

    await h.setup(program)
    h.mem.load_long(vector_addr, handler_addr)
    h.mem.load_words(handler_addr, _handler_code(h, 1, 0x0F))

    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    d1_val = h.read_result_long(0)
    assert d1_val == 0x0F, (
        f"Line-F 0xFFFF should trigger vector 11: got 0x{d1_val:08X}"
    )
    h.cleanup()


# =========================================================================
# Divide by zero tests (vector 5)
# =========================================================================

@cocotb.test()
async def test_divu_divide_by_zero(dut):
    """DIVU.W #0,Dn: triggers divide-by-zero exception (vector 5)."""
    h = CPUTestHarness(dut)
    handler_addr = HANDLER_BASE + 0x300
    vector_addr = 5 * 4  # 0x014

    program = [
        *moveq(0, 1),
        *moveq(42, 0),                         # D0 = 42 (dividend)
        *divu_w(SPECIAL, IMMEDIATE, 0),        # DIVU.W #0,D0
        *imm_word(0),                          # divisor = 0
        *nop(), *nop(),
        *h.sentinel_program(),
    ]

    await h.setup(program)
    h.mem.load_long(vector_addr, handler_addr)
    h.mem.load_words(handler_addr, _handler_code(h, 1, 0x55))

    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    d1_val = h.read_result_long(0)
    assert d1_val == 0x55, (
        f"DIVU #0 should trigger vector 5 handler: D1 expected 0x55, got 0x{d1_val:08X}"
    )
    h.cleanup()


@cocotb.test()
async def test_divs_divide_by_zero(dut):
    """DIVS.W #0,Dn: triggers divide-by-zero exception (vector 5)."""
    h = CPUTestHarness(dut)
    handler_addr = HANDLER_BASE + 0x300
    vector_addr = 5 * 4

    program = [
        *moveq(0, 1),
        *moveq(42, 0),                         # D0 = 42
        *divs_w(SPECIAL, IMMEDIATE, 0),        # DIVS.W #0,D0
        *imm_word(0),                          # divisor = 0
        *nop(), *nop(),
        *h.sentinel_program(),
    ]

    await h.setup(program)
    h.mem.load_long(vector_addr, handler_addr)
    h.mem.load_words(handler_addr, _handler_code(h, 1, 0x55))

    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    d1_val = h.read_result_long(0)
    assert d1_val == 0x55, (
        f"DIVS #0 should trigger vector 5 handler: D1 expected 0x55, got 0x{d1_val:08X}"
    )
    h.cleanup()


@cocotb.test(expect_error=AssertionError)
async def test_divu_divide_by_zero_preserves_dividend(dut):
    """DIVU.W #0: dividend register should be unchanged after exception.

    CORE BUG: The WF68K30L divider clobbers the destination register to
    0xFFFFFFFF on divide-by-zero instead of preserving it as per the
    MC68030 specification. Marked expect_error.
    """
    h = CPUTestHarness(dut)
    handler_addr = HANDLER_BASE + 0x300
    vector_addr = 5 * 4

    handler_code = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *move(LONG, DN, 2, AN_IND, 0),        # store D2 (the dividend register)
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]

    program = [
        *move(LONG, SPECIAL, IMMEDIATE, DN, 2),
        *imm_long(0x12345678),                 # D2 = 0x12345678
        *divu_w(SPECIAL, IMMEDIATE, 2),        # DIVU.W #0,D2
        *imm_word(0),
        *nop(), *nop(),
        *h.sentinel_program(),
    ]

    await h.setup(program)
    h.mem.load_long(vector_addr, handler_addr)
    h.mem.load_words(handler_addr, handler_code)

    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    d2_val = h.read_result_long(0)
    assert d2_val == 0x12345678, (
        f"DIVU #0 should preserve dividend: expected 0x12345678, got 0x{d2_val:08X}"
    )
    h.cleanup()


# =========================================================================
# TRAPV tests (vector 7)
# =========================================================================

@cocotb.test()
async def test_trapv_no_overflow(dut):
    """TRAPV with V=0: no trap, execution continues normally."""
    h = CPUTestHarness(dut)
    handler_addr = HANDLER_BASE + 0x400
    trapv_vector_addr = 7 * 4  # 0x01C

    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(0, 1),                          # clears V flag
        *trapv(),                               # V=0, should NOT trap
        *nop(), *nop(),
        *moveq(0x77, 1),                       # reached if no trap
        *move(LONG, DN, 1, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]

    await h.setup(program)
    h.mem.load_long(trapv_vector_addr, handler_addr)
    h.mem.load_words(handler_addr, _handler_code(h, 1, 0x33))

    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    d1_val = h.read_result_long(0)
    assert d1_val == 0x77, (
        f"TRAPV with V=0 should not trap: D1 expected 0x77, got 0x{d1_val:08X}"
    )
    h.cleanup()


@cocotb.test()
async def test_trapv_with_overflow(dut):
    """TRAPV with V=1: trap occurs, handler executes."""
    h = CPUTestHarness(dut)
    handler_addr = HANDLER_BASE + 0x400
    trapv_vector_addr = 7 * 4

    program = [
        *moveq(0, 1),
        # Create overflow: D2 = 0x7FFFFFFF, then ADD.L #1,D2 -> V=1
        *move(LONG, SPECIAL, IMMEDIATE, DN, 2),
        *imm_long(0x7FFFFFFF),
        *addi(LONG, DN, 2, 1),                 # D2 overflow -> V=1
        *trapv(),                               # V=1, should trap
        *nop(), *nop(),
        *h.sentinel_program(),
    ]

    await h.setup(program)
    h.mem.load_long(trapv_vector_addr, handler_addr)
    h.mem.load_words(handler_addr, _handler_code(h, 1, 0x33))

    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    d1_val = h.read_result_long(0)
    assert d1_val == 0x33, (
        f"TRAPV with V=1 should trap: D1 expected 0x33, got 0x{d1_val:08X}"
    )
    h.cleanup()


@cocotb.test()
async def test_trapv_overflow_via_add_negative(dut):
    """TRAPV with V=1 set by adding two negative numbers that overflow.

    -1 (0xFFFFFFFF) + 0x80000000 = 0x7FFFFFFF, which is positive from
    two negative operands -> V=1.

    Uses multi-word MOVE.L to load both operands, avoiding the prefetch
    pipeline hazard (BUG-001) that occurs when single-word instructions
    precede register-form ADD.
    """
    h = CPUTestHarness(dut)
    handler_addr = HANDLER_BASE + 0x400
    trapv_vector_addr = 7 * 4

    program = [
        *moveq(0, 1),
        # D2 = 0x80000000 (most negative), D3 = -1 (0xFFFFFFFF)
        # Use multi-word MOVE.L for both to avoid prefetch pipeline hazard
        *move(LONG, SPECIAL, IMMEDIATE, DN, 2),
        *imm_long(0x80000000),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 3),
        *imm_long(0xFFFFFFFF),                  # D3 = 0xFFFFFFFF
        # ADD.L D2,D3: D3 = D3 + D2 = 0xFFFFFFFF + 0x80000000 = 0x7FFFFFFF -> V=1
        *add(LONG, 3, 0, DN, 2),
        *trapv(),
        *nop(), *nop(),
        *h.sentinel_program(),
    ]

    await h.setup(program)
    h.mem.load_long(trapv_vector_addr, handler_addr)
    h.mem.load_words(handler_addr, _handler_code(h, 1, 0x33))

    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    d1_val = h.read_result_long(0)
    assert d1_val == 0x33, (
        f"TRAPV with V=1 (neg add) should trap: D1 expected 0x33, got 0x{d1_val:08X}"
    )
    h.cleanup()


@cocotb.test()
async def test_trapv_no_overflow_after_clear(dut):
    """TRAPV after overflow is cleared: no trap."""
    h = CPUTestHarness(dut)
    handler_addr = HANDLER_BASE + 0x400
    trapv_vector_addr = 7 * 4

    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        # Create overflow
        *move(LONG, SPECIAL, IMMEDIATE, DN, 2),
        *imm_long(0x7FFFFFFF),
        *addi(LONG, DN, 2, 1),                 # V=1
        # Now clear V by doing something that does not overflow
        *moveq(0, 3),                          # MOVEQ clears V
        *trapv(),                               # V=0, should NOT trap
        *nop(), *nop(),
        *moveq(0x77, 1),                       # reached if no trap
        *move(LONG, DN, 1, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]

    await h.setup(program)
    h.mem.load_long(trapv_vector_addr, handler_addr)
    h.mem.load_words(handler_addr, _handler_code(h, 1, 0x33))

    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    d1_val = h.read_result_long(0)
    assert d1_val == 0x77, (
        f"TRAPV with cleared V should not trap: D1 expected 0x77, got 0x{d1_val:08X}"
    )
    h.cleanup()


@cocotb.test(expect_error=AssertionError)
async def test_trapv_v_set_via_ccr(dut):
    """TRAPV with V=1 set explicitly via MOVE to CCR: trap occurs.

    CORE BUG: TRAPV does not see the V flag when set via MOVE to CCR.
    The V flag set by ADDI works (see test_trapv_with_overflow), but
    setting V via MOVE to CCR then executing TRAPV does not trigger the
    trap. This may be a pipeline forwarding issue where TRAPV reads V
    from the ALU result rather than the committed CCR. Marked expect_error.
    """
    h = CPUTestHarness(dut)
    handler_addr = HANDLER_BASE + 0x400
    trapv_vector_addr = 7 * 4

    program = [
        *moveq(0, 1),
        # Set V flag explicitly via CCR: V is bit 1 of CCR
        *move(WORD, SPECIAL, IMMEDIATE, DN, 5),
        *imm_word(0x0002),                      # V=1 only
        *move_to_ccr(DN, 5),
        *trapv(),                               # V=1, should trap
        *nop(), *nop(),
        *h.sentinel_program(),
    ]

    await h.setup(program)
    h.mem.load_long(trapv_vector_addr, handler_addr)
    h.mem.load_words(handler_addr, _handler_code(h, 1, 0x33))

    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    d1_val = h.read_result_long(0)
    assert d1_val == 0x33, (
        f"TRAPV with V=1 via CCR should trap: D1 expected 0x33, got 0x{d1_val:08X}"
    )
    h.cleanup()


# =========================================================================
# TRAP #n stack frame verification tests
# =========================================================================

@cocotb.test()
async def test_trap_0_stack_frame(dut):
    """TRAP #0: verify exception stack frame (SP, saved SR, saved PC)."""
    h = CPUTestHarness(dut)
    handler_addr = HANDLER_BASE + 0x500
    trap_vector_addr = 32 * 4  # 0x080

    # Program layout:
    #   0x100: MOVEQ #0,D1            (2 bytes)
    #   0x102: TRAP #0                 (2 bytes) <- trap instruction
    #   0x104: NOP ...
    program = [
        *moveq(0, 1),                          # 0x100
        *trap(0),                               # 0x102
        *nop(), *nop(),
        *h.sentinel_program(),
    ]

    await h.setup(program)
    h.mem.load_long(trap_vector_addr, handler_addr)
    h.mem.load_words(handler_addr, _handler_code_read_frame(h, 0x55))

    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"

    marker = h.read_result_long(0)
    assert marker == 0x55, f"Marker expected 0x55, got 0x{marker:08X}"

    sp = h.read_result_long(4)
    assert sp < h.SSP_INIT, f"SP should be below SSP_INIT, got 0x{sp:08X}"

    # For TRAP, saved PC should point to instruction after TRAP
    saved_pc = h.read_result_long(12)
    assert saved_pc == 0x104, (
        f"TRAP #0 saved PC should be 0x104, got 0x{saved_pc:08X}"
    )
    h.cleanup()


@cocotb.test()
async def test_trap_0_stack_sr_supervisor(dut):
    """TRAP #0: saved SR on stack should have S bit set (supervisor mode)."""
    h = CPUTestHarness(dut)
    handler_addr = HANDLER_BASE + 0x500
    trap_vector_addr = 32 * 4

    program = [
        *moveq(0, 1),
        *trap(0),
        *nop(), *nop(),
        *h.sentinel_program(),
    ]

    await h.setup(program)
    h.mem.load_long(trap_vector_addr, handler_addr)
    h.mem.load_words(handler_addr, _handler_code_read_frame(h, 0x55))

    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"

    saved_sr = h.read_result_long(8)
    # S bit is bit 13 of SR
    s_bit = (saved_sr >> 13) & 1
    assert s_bit == 1, (
        f"Saved SR should have S bit set: SR=0x{saved_sr:04X}"
    )
    h.cleanup()


@cocotb.test()
async def test_trap_0_vector_offset(dut):
    """TRAP #0: format/vector offset word on stack should indicate vector 32."""
    h = CPUTestHarness(dut)
    handler_addr = HANDLER_BASE + 0x500
    trap_vector_addr = 32 * 4

    program = [
        *moveq(0, 1),
        *trap(0),
        *nop(), *nop(),
        *h.sentinel_program(),
    ]

    await h.setup(program)
    h.mem.load_long(trap_vector_addr, handler_addr)
    h.mem.load_words(handler_addr, _handler_code_read_frame(h, 0x55))

    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"

    fmt_vec = h.read_result_long(16)
    # Format/vector offset word: bits 15-12 = format, bits 11-0 = vector offset
    # Vector offset = vector_number * 4 = 32 * 4 = 0x080
    vec_offset = fmt_vec & 0x0FFF
    assert vec_offset == 0x080, (
        f"Vector offset should be 0x080 (vector 32): got 0x{vec_offset:03X} (full=0x{fmt_vec:04X})"
    )
    h.cleanup()


@cocotb.test()
async def test_trap_15_vector_offset(dut):
    """TRAP #15: format/vector offset should indicate vector 47."""
    h = CPUTestHarness(dut)
    handler_addr = HANDLER_BASE + 0x580
    trap_vector_addr = (32 + 15) * 4  # 0xBC

    program = [
        *moveq(0, 1),
        *trap(15),
        *nop(), *nop(),
        *h.sentinel_program(),
    ]

    await h.setup(program)
    h.mem.load_long(trap_vector_addr, handler_addr)
    h.mem.load_words(handler_addr, _handler_code_read_frame(h, 0x55))

    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"

    fmt_vec = h.read_result_long(16)
    vec_offset = fmt_vec & 0x0FFF
    # Vector 47 * 4 = 0x0BC
    assert vec_offset == 0x0BC, (
        f"TRAP #15 vector offset should be 0x0BC (vector 47): got 0x{vec_offset:03X}"
    )
    h.cleanup()


@cocotb.test()
async def test_trap_stack_decrements_sp(dut):
    """TRAP decrements SP to push the exception frame."""
    h = CPUTestHarness(dut)
    handler_addr = HANDLER_BASE + 0x500
    trap_vector_addr = 32 * 4

    handler_code = _handler_code_read_sp(h, 0x55)

    program = [
        *moveq(0, 1),
        *trap(0),
        *nop(), *nop(),
        *h.sentinel_program(),
    ]

    await h.setup(program)
    h.mem.load_long(trap_vector_addr, handler_addr)
    h.mem.load_words(handler_addr, handler_code)

    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"

    marker = h.read_result_long(0)
    assert marker == 0x55
    sp_in_handler = h.read_result_long(4)
    assert sp_in_handler < h.SSP_INIT, (
        f"SP in handler should be below SSP_INIT(0x{h.SSP_INIT:08X}), "
        f"got 0x{sp_in_handler:08X}"
    )
    h.cleanup()


@cocotb.test()
async def test_trap_5_vector_offset(dut):
    """TRAP #5: format/vector offset should indicate vector 37."""
    h = CPUTestHarness(dut)
    handler_addr = HANDLER_BASE + 0x560
    trap_vector_addr = (32 + 5) * 4  # vector 37

    program = [
        *moveq(0, 1),
        *trap(5),
        *nop(), *nop(),
        *h.sentinel_program(),
    ]

    await h.setup(program)
    h.mem.load_long(trap_vector_addr, handler_addr)
    h.mem.load_words(handler_addr, _handler_code_read_frame(h, 0x55))

    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"

    fmt_vec = h.read_result_long(16)
    vec_offset = fmt_vec & 0x0FFF
    # Vector 37 * 4 = 0x094
    assert vec_offset == 0x094, (
        f"TRAP #5 vector offset should be 0x094 (vector 37): got 0x{vec_offset:03X}"
    )
    h.cleanup()


# =========================================================================
# CHK exception tests (vector 6) - out of range
# =========================================================================

@cocotb.test()
async def test_chk_w_negative_traps(dut):
    """CHK.W with negative value: triggers CHK exception (vector 6)."""
    h = CPUTestHarness(dut)
    handler_addr = HANDLER_BASE + 0x600
    vector_addr = 6 * 4  # 0x018

    program = [
        *moveq(0, 1),
        *moveq(-1, 0),                         # D0 = 0xFFFFFFFF (-1)
        *moveq(10, 3),                         # D3 = 10 (upper bound)
        *chk_w(DN, 3, 0),                      # CHK.W D3,D0: D0 < 0 -> trap
        *nop(), *nop(),
        *h.sentinel_program(),
    ]

    await h.setup(program)
    h.mem.load_long(vector_addr, handler_addr)
    h.mem.load_words(handler_addr, _handler_code(h, 1, 0x66))

    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    d1_val = h.read_result_long(0)
    assert d1_val == 0x66, (
        f"CHK with negative value should trap: D1 expected 0x66, got 0x{d1_val:08X}"
    )
    h.cleanup()


@cocotb.test()
async def test_chk_w_above_upper_traps(dut):
    """CHK.W with value > upper bound: triggers CHK exception (vector 6)."""
    h = CPUTestHarness(dut)
    handler_addr = HANDLER_BASE + 0x600
    vector_addr = 6 * 4

    program = [
        *moveq(0, 1),
        *moveq(20, 0),                         # D0 = 20 (value)
        *moveq(10, 3),                         # D3 = 10 (upper bound)
        *chk_w(DN, 3, 0),                      # CHK.W D3,D0: D0 > D3 -> trap
        *nop(), *nop(),
        *h.sentinel_program(),
    ]

    await h.setup(program)
    h.mem.load_long(vector_addr, handler_addr)
    h.mem.load_words(handler_addr, _handler_code(h, 1, 0x66))

    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    d1_val = h.read_result_long(0)
    assert d1_val == 0x66, (
        f"CHK with value > upper should trap: D1 expected 0x66, got 0x{d1_val:08X}"
    )
    h.cleanup()


# =========================================================================
# Privilege violation tests (vector 8)
# =========================================================================

@cocotb.test()
async def test_privilege_violation_move_to_sr(dut):
    """MOVE to SR in user mode: triggers privilege violation (vector 8).

    Steps:
      1. Set up privilege violation vector
      2. Switch to user mode via MOVE to SR (clear S bit)
      3. Execute MOVE to SR (privileged) in user mode
      4. Verify vector 8 handler reached
    """
    h = CPUTestHarness(dut)
    handler_addr = HANDLER_BASE + 0x700
    vector_addr = 8 * 4  # 0x020

    # Use marker value <= 0x7F to avoid MOVEQ sign extension
    handler_code = _handler_code(h, 1, 0x08)

    program = [
        *moveq(0, 1),                          # D1 = 0
        # Switch to user mode: SR = 0x0000 (S=0, T=0, IPL=0)
        *move(WORD, SPECIAL, IMMEDIATE, DN, 5),
        *imm_word(0x0000),
        *move_to_sr(DN, 5),
        # Now in user mode. Execute privileged instruction:
        *move(WORD, SPECIAL, IMMEDIATE, DN, 5),
        *imm_word(0x2700),
        *move_to_sr(DN, 5),                    # Privileged -> exception
        *nop(), *nop(),
        *h.sentinel_program(),
    ]

    await h.setup(program)
    h.mem.load_long(vector_addr, handler_addr)
    h.mem.load_words(handler_addr, handler_code)

    found = await h.run_until_sentinel(max_cycles=5000)
    assert found, "Sentinel not reached"
    d1_val = h.read_result_long(0)
    assert d1_val == 0x08, (
        f"Privilege violation should trigger vector 8: D1 expected 0x08, got 0x{d1_val:08X}"
    )
    h.cleanup()


@cocotb.test()
async def test_privilege_violation_stop(dut):
    """STOP in user mode: triggers privilege violation (vector 8).

    STOP is a privileged instruction. Executing it in user mode should
    cause a privilege violation exception.
    """
    h = CPUTestHarness(dut)
    handler_addr = HANDLER_BASE + 0x700
    vector_addr = 8 * 4

    handler_code = _handler_code(h, 1, 0x08)

    program = [
        *moveq(0, 1),
        # Switch to user mode
        *move(WORD, SPECIAL, IMMEDIATE, DN, 5),
        *imm_word(0x0000),
        *move_to_sr(DN, 5),
        # Now in user mode, execute STOP (privileged)
        *stop(0x2700),                          # Privileged -> exception
        *nop(), *nop(),
        *h.sentinel_program(),
    ]

    await h.setup(program)
    h.mem.load_long(vector_addr, handler_addr)
    h.mem.load_words(handler_addr, handler_code)

    found = await h.run_until_sentinel(max_cycles=5000)
    assert found, "Sentinel not reached"
    d1_val = h.read_result_long(0)
    assert d1_val == 0x08, (
        f"STOP in user mode should trigger vector 8: D1 expected 0x08, got 0x{d1_val:08X}"
    )
    h.cleanup()


@cocotb.test()
async def test_privilege_violation_move_from_sr_user(dut):
    """MOVE from SR in user mode: triggers privilege violation (vector 8).

    On MC68010+, MOVE from SR is privileged. On MC68000, it was not.
    The MC68030 should generate a privilege violation.
    """
    h = CPUTestHarness(dut)
    handler_addr = HANDLER_BASE + 0x700
    vector_addr = 8 * 4

    handler_code = _handler_code(h, 1, 0x08)

    program = [
        *moveq(0, 1),
        # Switch to user mode
        *move(WORD, SPECIAL, IMMEDIATE, DN, 5),
        *imm_word(0x0000),
        *move_to_sr(DN, 5),
        # Now in user mode, execute MOVE from SR (privileged on 68010+)
        *move_from_sr(DN, 3),                   # Privileged -> exception
        *nop(), *nop(),
        *h.sentinel_program(),
    ]

    await h.setup(program)
    h.mem.load_long(vector_addr, handler_addr)
    h.mem.load_words(handler_addr, handler_code)

    found = await h.run_until_sentinel(max_cycles=5000)
    assert found, "Sentinel not reached"
    d1_val = h.read_result_long(0)
    assert d1_val == 0x08, (
        f"MOVE from SR in user mode should trigger vector 8: D1 expected 0x08, got 0x{d1_val:08X}"
    )
    h.cleanup()


# =========================================================================
# Illegal instruction - undefined opcodes (not 0x4AFC, but truly undefined)
# =========================================================================

@cocotb.test()
async def test_undefined_opcode_triggers_illegal(dut):
    """An undefined opcode (not Line-A/F) triggers illegal instruction (vector 4).

    Opcode 0x4AFB is in the 0100 group but not a defined instruction,
    so it should trigger the illegal instruction exception.
    """
    h = CPUTestHarness(dut)
    handler_addr = HANDLER_BASE
    vector_addr = 4 * 4

    program = [
        *moveq(0, 1),
        0x4AFB,                                # Undefined opcode near ILLEGAL
        *nop(), *nop(),
        *h.sentinel_program(),
    ]

    await h.setup(program)
    h.mem.load_long(vector_addr, handler_addr)
    h.mem.load_words(handler_addr, _handler_code(h, 1, 0x44))

    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    d1_val = h.read_result_long(0)
    assert d1_val == 0x44, (
        f"Undefined opcode should trigger vector 4: D1 expected 0x44, got 0x{d1_val:08X}"
    )
    h.cleanup()


# =========================================================================
# Multiple exception types: verify different vectors dispatch independently
# =========================================================================

@cocotb.test()
async def test_illegal_uses_vector4_not_vector10(dut):
    """ILLEGAL dispatches to vector 4, not vector 10 (Line-A)."""
    h = CPUTestHarness(dut)
    handler4_addr = HANDLER_BASE
    handler10_addr = HANDLER_BASE + 0x100
    vector4_addr = 4 * 4
    vector10_addr = 10 * 4

    program = [
        *moveq(0, 1),
        *illegal(),
        *nop(), *nop(),
        *h.sentinel_program(),
    ]

    await h.setup(program)
    # Set up both vectors with different markers (both <= 0x7F)
    h.mem.load_long(vector4_addr, handler4_addr)
    h.mem.load_words(handler4_addr, _handler_code(h, 1, 0x44))
    h.mem.load_long(vector10_addr, handler10_addr)
    h.mem.load_words(handler10_addr, _handler_code(h, 1, 0x1A))

    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    d1_val = h.read_result_long(0)
    assert d1_val == 0x44, (
        f"ILLEGAL should use vector 4 (marker 0x44), not vector 10: got 0x{d1_val:08X}"
    )
    h.cleanup()


@cocotb.test()
async def test_line_a_uses_vector10_not_vector4(dut):
    """Line-A dispatches to vector 10, not vector 4 (illegal)."""
    h = CPUTestHarness(dut)
    handler4_addr = HANDLER_BASE
    handler10_addr = HANDLER_BASE + 0x100
    vector4_addr = 4 * 4
    vector10_addr = 10 * 4

    program = [
        *moveq(0, 1),
        0xA000,
        *nop(), *nop(),
        *h.sentinel_program(),
    ]

    await h.setup(program)
    h.mem.load_long(vector4_addr, handler4_addr)
    h.mem.load_words(handler4_addr, _handler_code(h, 1, 0x44))
    h.mem.load_long(vector10_addr, handler10_addr)
    h.mem.load_words(handler10_addr, _handler_code(h, 1, 0x1A))

    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    d1_val = h.read_result_long(0)
    assert d1_val == 0x1A, (
        f"Line-A should use vector 10 (marker 0x1A), not vector 4: got 0x{d1_val:08X}"
    )
    h.cleanup()


@cocotb.test()
async def test_line_f_uses_vector11_not_vector4(dut):
    """Line-F dispatches to vector 11, not vector 4 (illegal)."""
    h = CPUTestHarness(dut)
    handler4_addr = HANDLER_BASE
    handler11_addr = HANDLER_BASE + 0x180
    vector4_addr = 4 * 4
    vector11_addr = 11 * 4

    program = [
        *moveq(0, 1),
        0xF000,
        *nop(), *nop(),
        *h.sentinel_program(),
    ]

    await h.setup(program)
    h.mem.load_long(vector4_addr, handler4_addr)
    h.mem.load_words(handler4_addr, _handler_code(h, 1, 0x44))
    h.mem.load_long(vector11_addr, handler11_addr)
    h.mem.load_words(handler11_addr, _handler_code(h, 1, 0x1F))

    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    d1_val = h.read_result_long(0)
    assert d1_val == 0x1F, (
        f"Line-F should use vector 11 (marker 0x1F), not vector 4: got 0x{d1_val:08X}"
    )
    h.cleanup()


# =========================================================================
# ILLEGAL instruction stack frame: format word check
# =========================================================================

@cocotb.test()
async def test_illegal_vector_offset_in_frame(dut):
    """ILLEGAL: vector offset in stack frame should indicate vector 4."""
    h = CPUTestHarness(dut)
    handler_addr = HANDLER_BASE + 0x200
    vector_addr = 4 * 4

    program = [
        *moveq(0, 1),                          # 0x100
        *illegal(),                             # 0x102
        *nop(), *nop(),
        *h.sentinel_program(),
    ]

    await h.setup(program)
    h.mem.load_long(vector_addr, handler_addr)
    h.mem.load_words(handler_addr, _handler_code_read_frame(h, 0x44))

    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"

    fmt_vec = h.read_result_long(16)
    vec_offset = fmt_vec & 0x0FFF
    # Vector 4 * 4 = 0x010
    assert vec_offset == 0x010, (
        f"ILLEGAL vector offset should be 0x010 (vector 4): got 0x{vec_offset:03X}"
    )
    h.cleanup()


@cocotb.test()
async def test_line_a_vector_offset_in_frame(dut):
    """Line-A: vector offset in stack frame should indicate vector 10."""
    h = CPUTestHarness(dut)
    handler_addr = HANDLER_BASE + 0x200
    vector_addr = 10 * 4

    program = [
        *moveq(0, 1),                          # 0x100
        0xA000,                                 # 0x102: Line-A
        *nop(), *nop(),
        *h.sentinel_program(),
    ]

    await h.setup(program)
    h.mem.load_long(vector_addr, handler_addr)
    h.mem.load_words(handler_addr, _handler_code_read_frame(h, 0x44))

    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"

    fmt_vec = h.read_result_long(16)
    vec_offset = fmt_vec & 0x0FFF
    # Vector 10 * 4 = 0x028
    assert vec_offset == 0x028, (
        f"Line-A vector offset should be 0x028 (vector 10): got 0x{vec_offset:03X}"
    )
    h.cleanup()


@cocotb.test()
async def test_line_f_vector_offset_in_frame(dut):
    """Line-F: vector offset in stack frame should indicate vector 11."""
    h = CPUTestHarness(dut)
    handler_addr = HANDLER_BASE + 0x200
    vector_addr = 11 * 4

    program = [
        *moveq(0, 1),                          # 0x100
        0xF000,                                 # 0x102: Line-F
        *nop(), *nop(),
        *h.sentinel_program(),
    ]

    await h.setup(program)
    h.mem.load_long(vector_addr, handler_addr)
    h.mem.load_words(handler_addr, _handler_code_read_frame(h, 0x44))

    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"

    fmt_vec = h.read_result_long(16)
    vec_offset = fmt_vec & 0x0FFF
    # Vector 11 * 4 = 0x02C
    assert vec_offset == 0x02C, (
        f"Line-F vector offset should be 0x02C (vector 11): got 0x{vec_offset:03X}"
    )
    h.cleanup()


# =========================================================================
# TRAP with RTE: verify we can return from a trap handler
#
# CORE BUG: RTE does not properly return from exception handlers in the
# WF68K30L. The CPU appears to hang or not restore PC correctly.
# These tests are marked expect_error.
# =========================================================================

@cocotb.test(expect_error=AssertionError)
async def test_trap_rte_returns(dut):
    """TRAP #0 with RTE: handler returns, execution continues after TRAP.

    CORE BUG: RTE does not properly return from exception handlers.
    The CPU hangs instead of continuing execution after the TRAP.
    Marked expect_error.
    """
    h = CPUTestHarness(dut)
    handler_addr = HANDLER_BASE + 0x800
    trap_vector_addr = 32 * 4

    # Handler code: just RTE (return from exception)
    handler_code = [
        *rte(),
    ]

    # Main program
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),   # 0x100
        *imm_long(h.RESULT_BASE),
        *moveq(0, 1),                          # 0x106: D1 = 0
        *trap(0),                               # 0x108: TRAP #0
        # After RTE returns here: 0x10A
        *nop(), *nop(),
        *moveq(0x42, 1),                       # D1 = 0x42 (reached after RTE)
        *move(LONG, DN, 1, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]

    await h.setup(program)
    h.mem.load_long(trap_vector_addr, handler_addr)
    h.mem.load_words(handler_addr, handler_code)

    found = await h.run_until_sentinel(max_cycles=5000)
    assert found, "Sentinel not reached - RTE may not have returned"
    d1_val = h.read_result_long(0)
    assert d1_val == 0x42, (
        f"After TRAP/RTE, D1 should be 0x42: got 0x{d1_val:08X}"
    )
    h.cleanup()


@cocotb.test(expect_error=AssertionError)
async def test_trap_rte_restores_sr(dut):
    """TRAP with RTE: verify SR is restored from the stack frame.

    CORE BUG: RTE does not properly return from exception handlers.
    Marked expect_error.
    """
    h = CPUTestHarness(dut)
    handler_addr = HANDLER_BASE + 0x800
    trap_vector_addr = 32 * 4

    handler_code = [
        *move(WORD, SPECIAL, IMMEDIATE, DN, 5),
        *imm_word(0x001F),
        *move_to_ccr(DN, 5),
        *rte(),
    ]

    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *move(WORD, SPECIAL, IMMEDIATE, DN, 5),
        *imm_word(0x0000),
        *move_to_ccr(DN, 5),
        *trap(0),
        *move_from_ccr(DN, 6),                 # capture CCR BEFORE any store
        *move(LONG, DN, 6, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]

    await h.setup(program)
    h.mem.load_long(trap_vector_addr, handler_addr)
    h.mem.load_words(handler_addr, handler_code)

    found = await h.run_until_sentinel(max_cycles=5000)
    assert found, "Sentinel not reached"
    ccr_val = h.read_result_long(0) & 0xFF
    assert ccr_val == 0x00, (
        f"After TRAP/RTE, CCR should be restored to 0x00: got 0x{ccr_val:02X}"
    )
    h.cleanup()


# =========================================================================
# Divide by zero - stack frame verification
# =========================================================================

@cocotb.test()
async def test_divu_divide_by_zero_vector_offset(dut):
    """DIVU.W #0: vector offset in stack frame should indicate vector 5."""
    h = CPUTestHarness(dut)
    handler_addr = HANDLER_BASE + 0x300
    vector_addr = 5 * 4

    program = [
        *moveq(0, 1),                          # 0x100
        *moveq(42, 0),                         # 0x102
        *divu_w(SPECIAL, IMMEDIATE, 0),        # 0x104
        *imm_word(0),                          # 0x106 (extension word)
        *nop(), *nop(),
        *h.sentinel_program(),
    ]

    await h.setup(program)
    h.mem.load_long(vector_addr, handler_addr)
    h.mem.load_words(handler_addr, _handler_code_read_frame(h, 0x55))

    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"

    fmt_vec = h.read_result_long(16)
    vec_offset = fmt_vec & 0x0FFF
    # Vector 5 * 4 = 0x014
    assert vec_offset == 0x014, (
        f"Div-by-zero vector offset should be 0x014 (vector 5): got 0x{vec_offset:03X}"
    )
    h.cleanup()
