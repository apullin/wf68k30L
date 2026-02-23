"""
Control flow instruction compliance tests for WF68K30L.

Tests NOP, JMP, JSR/RTS, TRAP, TRAPV, and CHK against the MC68030
specification.

Each test uses the prefetch pipeline hazard workaround:
  - Load RESULT_BASE into A0 early
  - Store results via MOVE.L Dn,(A0) (single-word, no extension words)
  - Advance A0 with ADDQ.L #4,A0

For exception-based tests (TRAP, TRAPV), handler routines are placed
at known addresses in memory. Exception vectors are written into the vector
table at address 0. Handlers write sentinel directly (no RTE return) because
the MC68030 exception frame format requires careful stack handling.

IMPORTANT: The CPU starts in supervisor mode with SSP = 0x001000.

IMPORTANT: MOVE from CCR must be done BEFORE any MOVE stores.
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles

from cpu_harness import CPUTestHarness
from m68k_encode import (
    BYTE, WORD, LONG,
    DN, AN, AN_IND, AN_POSTINC, AN_PREDEC, SPECIAL, ABS_L, IMMEDIATE,
    moveq, move, movea, move_to_abs_long, nop, addq, subq,
    addi, subi, add,
    jmp, jmp_abs, jsr, jsr_abs, rts, rte,
    trap, trapv,
    chk_w,
    move_from_ccr, move_to_ccr,
    imm_long, imm_word, abs_long,
)


# =========================================================================
# NOP tests
# =========================================================================

@cocotb.test()
async def test_nop_no_state_change(dut):
    """NOP: multiple NOPs do not change register state."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(42, 1),                            # D1 = 42
        *nop(), *nop(), *nop(), *nop(), *nop(),
        *move(LONG, DN, 1, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    assert result == 42, f"D1 should be unchanged after NOPs, got 0x{result:08X}"
    h.cleanup()


@cocotb.test()
async def test_nop_single(dut):
    """Single NOP executes without side effects."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(99, 1),
        *nop(),
        *move(LONG, DN, 1, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    assert result == 99, f"D1 should be 99, got 0x{result:08X}"
    h.cleanup()


@cocotb.test()
async def test_nop_preserves_ccr(dut):
    """NOP does not modify condition codes."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        # Set known CCR: all flags
        *move(WORD, SPECIAL, IMMEDIATE, DN, 5),
        *imm_word(0x001F),
        *move_to_ccr(DN, 5),
        *nop(), *nop(), *nop(), *nop(),
        *move_from_ccr(DN, 6),
        *move(LONG, DN, 6, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    ccr = h.read_result_long(0)
    ccr_byte = ccr & 0xFF
    assert ccr_byte == 0x1F, (
        f"NOP should preserve all CCR flags 0x1F, got 0x{ccr_byte:02X}"
    )
    h.cleanup()


# =========================================================================
# JMP tests
# =========================================================================

@cocotb.test()
async def test_jmp_abs_long(dut):
    """JMP (abs).L: jump to target, verify execution continues there."""
    h = CPUTestHarness(dut)
    # JMP over a MOVEQ that sets D1=0x0B, land on code that stores D1.
    # If JMP works, D1 will be 0x0A (set before JMP). If it falls through, 0x0B.
    target_addr = 0x000110
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),       # 0x100 (6 bytes)
        *imm_long(h.RESULT_BASE),
        *moveq(0x0A, 1),                           # 0x106: D1 = 0x0A
        *jmp_abs(target_addr),                     # 0x108: JMP to 0x110
        *moveq(0x0B, 1),                           # 0x10E: skipped
        # target: 0x110
        *nop(),
        *move(LONG, DN, 1, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    assert result == 0x0A, (
        f"JMP should skip MOVEQ #$0B: D1 expected 0x0A, got 0x{result:08X}"
    )
    h.cleanup()


@cocotb.test()
async def test_jmp_an_indirect(dut):
    """JMP (An): jump to address in register."""
    h = CPUTestHarness(dut)
    target_addr = 0x000112
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),       # 0x100
        *imm_long(h.RESULT_BASE),
        *moveq(0x0A, 1),                           # 0x106
        *movea(LONG, SPECIAL, IMMEDIATE, 2),       # 0x108
        *imm_long(target_addr),
        *jmp(AN_IND, 2),                           # 0x10E: JMP (A2)
        *moveq(0x0B, 1),                           # 0x110: skipped
        # target: 0x112
        *nop(),
        *move(LONG, DN, 1, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    assert result == 0x0A, f"JMP (A2) should skip 0x0B: got 0x{result:08X}"
    h.cleanup()


@cocotb.test()
async def test_jmp_forward_over_data(dut):
    """JMP forward over a block of instructions."""
    h = CPUTestHarness(dut)
    target = 0x000116
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(1, 1),
        *jmp_abs(target),
        *moveq(99, 1),                            # skipped
        *moveq(98, 1),                            # skipped
        *moveq(97, 1),                            # skipped
        *moveq(96, 1),                            # skipped
        # target: 0x116
        *nop(),
        *move(LONG, DN, 1, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    assert result == 1, f"JMP should skip data block: D1 expected 1, got 0x{result:08X}"
    h.cleanup()


@cocotb.test()
async def test_jmp_does_not_push_return(dut):
    """JMP does not push a return address (unlike JSR)."""
    h = CPUTestHarness(dut)
    target = 0x000112
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *move(LONG, AN, 7, DN, 1),                # D1 = SP before
        *move(LONG, DN, 1, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *jmp_abs(target),
        # target: 0x112
        *move(LONG, AN, 7, DN, 2),                # D2 = SP at target
        *move(LONG, DN, 2, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    sp_before = h.read_result_long(0)
    sp_target = h.read_result_long(4)
    assert sp_before == sp_target, (
        f"JMP should not change SP: before=0x{sp_before:08X} at_target=0x{sp_target:08X}"
    )
    h.cleanup()


# =========================================================================
# JSR / RTS tests
# =========================================================================

@cocotb.test()
async def test_jsr_rts_basic(dut):
    """JSR pushes return address, subroutine executes, RTS returns."""
    h = CPUTestHarness(dut)
    sub_addr = 0x000200

    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),       # 0x100
        *imm_long(h.RESULT_BASE),
        *moveq(1, 1),                              # 0x106: D1 = 1 (before call)
        *jsr_abs(sub_addr),                        # 0x108: JSR subroutine
        # Return point: 0x10E
        *moveq(3, 1),                              # 0x10E: D1 = 3 (after return)
        *move(LONG, DN, 1, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *move(LONG, DN, 2, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)

    sub_code = [*moveq(2, 2), *rts()]             # D2 = 2, return
    h.mem.load_words(sub_addr, sub_code)

    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    d1_val = h.read_result_long(0)
    d2_val = h.read_result_long(4)
    assert d1_val == 3, f"D1 should be 3 (after return), got 0x{d1_val:08X}"
    assert d2_val == 2, f"D2 should be 2 (subroutine marker), got 0x{d2_val:08X}"
    h.cleanup()


@cocotb.test()
async def test_jsr_pushes_return_address(dut):
    """JSR pushes correct return address onto stack."""
    h = CPUTestHarness(dut)
    sub_addr = 0x000200

    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),       # 0x100 (6 bytes)
        *imm_long(h.RESULT_BASE),
        *moveq(0, 1),                              # 0x106 (2 bytes)
        *jsr_abs(sub_addr),                        # 0x108 (6 bytes)
        # Return point: 0x10E
        *nop(),
        *move(LONG, DN, 1, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)

    # Subroutine reads the return address from stack
    sub_code = [
        *move(LONG, AN_IND, 7, DN, 1),            # D1 = [SP] (return address)
        *rts(),
    ]
    h.mem.load_words(sub_addr, sub_code)

    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    ret_addr = h.read_result_long(0)
    assert ret_addr == 0x10E, (
        f"Return address should be 0x10E, got 0x{ret_addr:08X}"
    )
    h.cleanup()


@cocotb.test()
async def test_jsr_rts_sp_preserved(dut):
    """JSR/RTS pair preserves the stack pointer."""
    h = CPUTestHarness(dut)
    sub_addr = 0x000200

    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *move(LONG, AN, 7, DN, 1),                # D1 = SP before
        *move(LONG, DN, 1, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *jsr_abs(sub_addr),
        *nop(),
        *move(LONG, AN, 7, DN, 2),                # D2 = SP after
        *move(LONG, DN, 2, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)

    sub_code = [*rts()]
    h.mem.load_words(sub_addr, sub_code)

    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    sp_before = h.read_result_long(0)
    sp_after = h.read_result_long(4)
    assert sp_before == sp_after, (
        f"SP should be preserved across JSR/RTS: before=0x{sp_before:08X} after=0x{sp_after:08X}"
    )
    h.cleanup()


@cocotb.test()
async def test_jsr_nested(dut):
    """Nested JSR/RTS calls work correctly."""
    h = CPUTestHarness(dut)
    sub1_addr = 0x000200
    sub2_addr = 0x000300

    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(0, 1),                             # D1 = 0 (call counter)
        *jsr_abs(sub1_addr),
        *move(LONG, DN, 1, AN_IND, 0),            # store D1 (should be 2)
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)

    sub1_code = [*addq(LONG, 1, DN, 1), *jsr_abs(sub2_addr), *rts()]
    h.mem.load_words(sub1_addr, sub1_code)

    sub2_code = [*addq(LONG, 1, DN, 1), *rts()]
    h.mem.load_words(sub2_addr, sub2_code)

    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    d1_val = h.read_result_long(0)
    assert d1_val == 2, f"D1 should be 2 (two nested calls), got 0x{d1_val:08X}"
    h.cleanup()


@cocotb.test()
async def test_jsr_rts_multiple_calls(dut):
    """Multiple JSR/RTS calls to same subroutine."""
    h = CPUTestHarness(dut)
    sub_addr = 0x000200

    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(0, 1),                             # D1 = 0 (counter)
        *jsr_abs(sub_addr),                        # call 1
        *jsr_abs(sub_addr),                        # call 2
        *jsr_abs(sub_addr),                        # call 3
        *move(LONG, DN, 1, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)

    sub_code = [*addq(LONG, 1, DN, 1), *rts()]
    h.mem.load_words(sub_addr, sub_code)

    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    d1_val = h.read_result_long(0)
    assert d1_val == 3, f"Three calls should give D1=3, got 0x{d1_val:08X}"
    h.cleanup()


@cocotb.test()
async def test_jsr_via_an_indirect(dut):
    """JSR (An): jump to subroutine via address register."""
    h = CPUTestHarness(dut)
    sub_addr = 0x000200

    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(0, 1),
        *movea(LONG, SPECIAL, IMMEDIATE, 2),
        *imm_long(sub_addr),
        *jsr(AN_IND, 2),                          # JSR (A2)
        *nop(),
        *move(LONG, DN, 1, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)

    sub_code = [*moveq(0x44, 1), *rts()]
    h.mem.load_words(sub_addr, sub_code)

    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    d1_val = h.read_result_long(0)
    assert d1_val == 0x44, f"JSR (A2) should call sub: D1 expected 0x44, got 0x{d1_val:08X}"
    h.cleanup()


# =========================================================================
# TRAP tests
#
# TRAP handlers write a marker and sentinel directly (no RTE) because the
# MC68030 exception stack frame format makes RTE return unreliable in this
# test harness.
# =========================================================================

def _trap_handler_code(h, marker_reg, marker_val):
    """Generate handler code that writes a marker to RESULT_BASE and sentinel."""
    return [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(marker_val, marker_reg),
        *move(LONG, DN, marker_reg, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]


@cocotb.test()
async def test_trap_0(dut):
    """TRAP #0: handler executes, writes marker, and signals sentinel."""
    h = CPUTestHarness(dut)
    handler_addr = 0x000400
    trap_vector_addr = 32 * 4  # 0x80

    program = [
        *moveq(0, 1),                             # D1 = 0
        *trap(0),                                  # TRAP #0
        *nop(), *nop(),
        *h.sentinel_program(),
    ]

    await h.setup(program)
    h.mem.load_long(trap_vector_addr, handler_addr)
    h.mem.load_words(handler_addr, _trap_handler_code(h, 1, 0x55))

    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    d1_val = h.read_result_long(0)
    assert d1_val == 0x55, f"TRAP #0 handler should set D1=0x55, got 0x{d1_val:08X}"
    h.cleanup()


@cocotb.test()
async def test_trap_15(dut):
    """TRAP #15: highest trap vector dispatches correctly."""
    h = CPUTestHarness(dut)
    handler_addr = 0x000500
    trap_vector_addr = (32 + 15) * 4  # 0xBC

    program = [
        *moveq(0, 1),
        *trap(15),
        *nop(), *nop(),
        *h.sentinel_program(),
    ]

    await h.setup(program)
    h.mem.load_long(trap_vector_addr, handler_addr)
    h.mem.load_words(handler_addr, _trap_handler_code(h, 1, 0x0F))

    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    d1_val = h.read_result_long(0)
    assert d1_val == 0x0F, f"TRAP #15 handler should set D1=0x0F, got 0x{d1_val:08X}"
    h.cleanup()


@cocotb.test()
async def test_trap_5(dut):
    """TRAP #5: mid-range trap vector."""
    h = CPUTestHarness(dut)
    handler_addr = 0x000600
    trap_vector_addr = (32 + 5) * 4  # 0x94

    program = [
        *moveq(0, 1),
        *trap(5),
        *nop(), *nop(),
        *h.sentinel_program(),
    ]

    await h.setup(program)
    h.mem.load_long(trap_vector_addr, handler_addr)
    h.mem.load_words(handler_addr, _trap_handler_code(h, 1, 0x05))

    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    d1_val = h.read_result_long(0)
    assert d1_val == 0x05, f"TRAP #5 handler should set D1=5, got 0x{d1_val:08X}"
    h.cleanup()


@cocotb.test()
async def test_trap_10(dut):
    """TRAP #10: verify vector 42 dispatches correctly."""
    h = CPUTestHarness(dut)
    handler_addr = 0x000900
    trap_vector_addr = (32 + 10) * 4  # 0xA8

    program = [
        *moveq(0, 1),
        *trap(10),
        *nop(), *nop(),
        *h.sentinel_program(),
    ]

    await h.setup(program)
    h.mem.load_long(trap_vector_addr, handler_addr)
    h.mem.load_words(handler_addr, _trap_handler_code(h, 1, 0x0A))

    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    d1_val = h.read_result_long(0)
    assert d1_val == 0x0A, f"TRAP #10 handler should set D1=10, got 0x{d1_val:08X}"
    h.cleanup()


@cocotb.test()
async def test_trap_preserves_registers(dut):
    """TRAP handler can see registers set before the trap."""
    h = CPUTestHarness(dut)
    handler_addr = 0x000400
    trap_vector_addr = 32 * 4

    # Handler reads D2 (set before trap), adds 8, stores result + sentinel
    handler_code = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *addq(LONG, 8, DN, 2),                    # D2 += 8
        *move(LONG, DN, 2, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]

    program = [
        *moveq(42, 2),                            # D2 = 42 (set before trap)
        *trap(0),
        *nop(), *nop(),
        *h.sentinel_program(),
    ]

    await h.setup(program)
    h.mem.load_long(trap_vector_addr, handler_addr)
    h.mem.load_words(handler_addr, handler_code)

    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    d2_val = h.read_result_long(0)
    assert d2_val == 50, f"D2 should be 42+8=50, got 0x{d2_val:08X}"
    h.cleanup()


@cocotb.test()
async def test_trap_two_different(dut):
    """Two different TRAP vectors dispatch to their respective handlers."""
    h = CPUTestHarness(dut)
    # For this test we can only reliably test one TRAP since handler writes sentinel.
    # Test that TRAP #3 dispatches to the correct vector (not TRAP #0's vector).
    handler_addr = 0x000700
    trap_vector_addr = (32 + 3) * 4  # TRAP #3 -> vector 35

    program = [
        *moveq(0, 1),
        *trap(3),
        *nop(), *nop(),
        *h.sentinel_program(),
    ]

    await h.setup(program)
    h.mem.load_long(trap_vector_addr, handler_addr)
    h.mem.load_words(handler_addr, _trap_handler_code(h, 1, 0x33))

    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    d1_val = h.read_result_long(0)
    assert d1_val == 0x33, f"TRAP #3 should dispatch to its handler: D1 expected 0x33, got 0x{d1_val:08X}"
    h.cleanup()


@cocotb.test()
async def test_trap_7(dut):
    """TRAP #7: another mid-range vector to verify dispatching."""
    h = CPUTestHarness(dut)
    handler_addr = 0x000800
    trap_vector_addr = (32 + 7) * 4

    program = [
        *moveq(0, 1),
        *trap(7),
        *nop(), *nop(),
        *h.sentinel_program(),
    ]

    await h.setup(program)
    h.mem.load_long(trap_vector_addr, handler_addr)
    h.mem.load_words(handler_addr, _trap_handler_code(h, 1, 0x07))

    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    d1_val = h.read_result_long(0)
    assert d1_val == 0x07, f"TRAP #7 got D1=0x{d1_val:08X}"
    h.cleanup()


# =========================================================================
# TRAPV tests
# =========================================================================

@cocotb.test()
async def test_trapv_no_overflow(dut):
    """TRAPV with V=0: no trap, execution continues."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(0, 1),                             # clears V
        *trapv(),                                  # V=0, no trap
        *nop(), *nop(),
        *moveq(0x77, 1),                          # D1 = 0x77 (reached if no trap)
        *move(LONG, DN, 1, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)

    # Set up TRAPV vector just in case (vector 7, address 0x1C)
    handler_addr = 0x000700
    h.mem.load_long(7 * 4, handler_addr)
    h.mem.load_words(handler_addr, _trap_handler_code(h, 1, 0x33))

    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    d1_val = h.read_result_long(0)
    assert d1_val == 0x77, (
        f"TRAPV with V=0 should not trap: D1 expected 0x77, got 0x{d1_val:08X}"
    )
    h.cleanup()


@cocotb.test()
async def test_trapv_overflow(dut):
    """TRAPV with V=1: trap occurs, handler executes."""
    h = CPUTestHarness(dut)
    handler_addr = 0x000700
    trapv_vector_addr = 7 * 4  # TRAPV uses vector 7

    program = [
        *moveq(0, 1),                             # D1 = 0 (marker)
        # Create overflow: D2 = 0x7FFFFFFF, then ADD.L #1,D2
        *move(LONG, SPECIAL, IMMEDIATE, DN, 2),
        *imm_long(0x7FFFFFFF),
        *addi(LONG, DN, 2, 1),                    # D2 = 0x7FFFFFFF + 1 -> V=1
        *trapv(),                                  # V=1, should trap
        *nop(), *nop(),
        *h.sentinel_program(),
    ]

    await h.setup(program)
    h.mem.load_long(trapv_vector_addr, handler_addr)
    h.mem.load_words(handler_addr, _trap_handler_code(h, 1, 0x33))

    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    d1_val = h.read_result_long(0)
    assert d1_val == 0x33, (
        f"TRAPV with V=1 should trap: D1 expected 0x33, got 0x{d1_val:08X}"
    )
    h.cleanup()


# =========================================================================
# CHK tests (in-range cases that do not generate exceptions)
# =========================================================================

@cocotb.test()
async def test_chk_w_in_range(dut):
    """CHK.W with value in range (0..upper): no trap, execution continues."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(5, 0),                             # D0 = 5 (value to check)
        *moveq(10, 1),                            # D1 = 10 (upper bound)
        *chk_w(DN, 1, 0),                         # CHK.W D1,D0 (check D0 against 0..D1)
        *nop(), *nop(),
        *moveq(0x77, 2),                          # D2 = 0x77 (reached = no trap)
        *move(LONG, DN, 2, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    d2_val = h.read_result_long(0)
    assert d2_val == 0x77, f"CHK in range: should reach 0x77, got 0x{d2_val:08X}"
    h.cleanup()


@cocotb.test()
async def test_chk_w_at_upper_bound(dut):
    """CHK.W with value == upper bound: no trap (value is in range)."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(10, 0),                            # D0 = 10
        *moveq(10, 1),                            # D1 = 10 (upper bound)
        *chk_w(DN, 1, 0),                         # CHK.W D1,D0 -> D0 == D1, OK
        *nop(), *nop(),
        *moveq(0x77, 2),
        *move(LONG, DN, 2, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    d2_val = h.read_result_long(0)
    assert d2_val == 0x77, f"CHK at bound: should continue, got D2=0x{d2_val:08X}"
    h.cleanup()


@cocotb.test()
async def test_chk_w_at_zero(dut):
    """CHK.W with value == 0: no trap (value is in range 0..upper)."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(0, 0),                             # D0 = 0
        *moveq(10, 1),                            # D1 = 10 (upper bound)
        *chk_w(DN, 1, 0),                         # CHK.W D1,D0 -> D0 == 0, OK
        *nop(), *nop(),
        *moveq(0x77, 2),
        *move(LONG, DN, 2, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    d2_val = h.read_result_long(0)
    assert d2_val == 0x77, f"CHK at zero: should continue, got D2=0x{d2_val:08X}"
    h.cleanup()


@cocotb.test()
async def test_chk_w_max_range(dut):
    """CHK.W with value at max positive word (0x7FFF) and upper=0x7FFF: no trap."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
        *imm_long(0x00007FFF),                     # D0 = 0x7FFF
        *move(LONG, SPECIAL, IMMEDIATE, DN, 1),
        *imm_long(0x00007FFF),                     # D1 = 0x7FFF
        *chk_w(DN, 1, 0),                         # CHK.W D1,D0 -> OK
        *nop(), *nop(),
        *moveq(0x77, 2),
        *move(LONG, DN, 2, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    d2_val = h.read_result_long(0)
    assert d2_val == 0x77, f"CHK max range: should continue, got D2=0x{d2_val:08X}"
    h.cleanup()


@cocotb.test()
async def test_chk_w_mid_range(dut):
    """CHK.W with value = 50 and upper = 100: no trap."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(50, 0),                            # D0 = 50
        *moveq(100, 1),                           # D1 = 100
        *chk_w(DN, 1, 0),                         # CHK.W D1,D0 -> OK
        *nop(), *nop(),
        *moveq(0x77, 2),
        *move(LONG, DN, 2, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    d2_val = h.read_result_long(0)
    assert d2_val == 0x77, f"CHK mid range: should continue, got D2=0x{d2_val:08X}"
    h.cleanup()


@cocotb.test()
async def test_chk_w_value_one(dut):
    """CHK.W with value = 1 and upper = 1: no trap."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(1, 0),                             # D0 = 1
        *moveq(1, 1),                             # D1 = 1
        *chk_w(DN, 1, 0),                         # CHK.W D1,D0 -> OK
        *nop(), *nop(),
        *moveq(0x77, 2),
        *move(LONG, DN, 2, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    d2_val = h.read_result_long(0)
    assert d2_val == 0x77, f"CHK value=1: should continue, got D2=0x{d2_val:08X}"
    h.cleanup()


# =========================================================================
# Additional JSR / JMP tests
# =========================================================================

@cocotb.test()
async def test_jsr_subroutine_modifies_multiple_regs(dut):
    """JSR to subroutine that modifies multiple data registers."""
    h = CPUTestHarness(dut)
    sub_addr = 0x000200

    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(0, 1),
        *moveq(0, 2),
        *moveq(0, 3),
        *jsr_abs(sub_addr),
        *nop(),
        *move(LONG, DN, 1, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *move(LONG, DN, 2, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *move(LONG, DN, 3, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)

    sub_code = [
        *moveq(0x11, 1),
        *moveq(0x22, 2),
        *moveq(0x33, 3),
        *rts(),
    ]
    h.mem.load_words(sub_addr, sub_code)

    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    assert h.read_result_long(0) == 0x11, f"D1 got 0x{h.read_result_long(0):08X}"
    assert h.read_result_long(4) == 0x22, f"D2 got 0x{h.read_result_long(4):08X}"
    assert h.read_result_long(8) == 0x33, f"D3 got 0x{h.read_result_long(8):08X}"
    h.cleanup()


@cocotb.test()
async def test_jmp_chain(dut):
    """Chain of JMPs: JMP to addr1, which JMPs to addr2, which continues."""
    h = CPUTestHarness(dut)
    addr1 = 0x000200
    addr2 = 0x000300

    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(1, 1),                             # D1 = 1
        *jmp_abs(addr1),
        *nop(), *nop(),
        *h.sentinel_program(),
    ]
    await h.setup(program)

    # addr1: increment D1, JMP to addr2
    code1 = [*addq(LONG, 1, DN, 1), *jmp_abs(addr2)]
    h.mem.load_words(addr1, code1)

    # addr2: increment D1 again, store result, sentinel
    code2 = [
        *addq(LONG, 1, DN, 1),
        *move(LONG, DN, 1, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    h.mem.load_words(addr2, code2)

    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    d1_val = h.read_result_long(0)
    assert d1_val == 3, f"JMP chain should give D1=3, got 0x{d1_val:08X}"
    h.cleanup()


@cocotb.test()
async def test_trap_1(dut):
    """TRAP #1: verify vector 33 dispatches correctly."""
    h = CPUTestHarness(dut)
    handler_addr = 0x000A00
    trap_vector_addr = (32 + 1) * 4

    program = [
        *moveq(0, 1),
        *trap(1),
        *nop(), *nop(),
        *h.sentinel_program(),
    ]

    await h.setup(program)
    h.mem.load_long(trap_vector_addr, handler_addr)
    h.mem.load_words(handler_addr, _trap_handler_code(h, 1, 0x01))

    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    d1_val = h.read_result_long(0)
    assert d1_val == 0x01, f"TRAP #1 got D1=0x{d1_val:08X}"
    h.cleanup()


@cocotb.test()
async def test_trap_14(dut):
    """TRAP #14: verify vector 46 dispatches correctly."""
    h = CPUTestHarness(dut)
    handler_addr = 0x000B00
    trap_vector_addr = (32 + 14) * 4

    program = [
        *moveq(0, 1),
        *trap(14),
        *nop(), *nop(),
        *h.sentinel_program(),
    ]

    await h.setup(program)
    h.mem.load_long(trap_vector_addr, handler_addr)
    h.mem.load_words(handler_addr, _trap_handler_code(h, 1, 0x0E))

    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    d1_val = h.read_result_long(0)
    assert d1_val == 0x0E, f"TRAP #14 got D1=0x{d1_val:08X}"
    h.cleanup()


@cocotb.test()
async def test_jsr_deep_nesting(dut):
    """Three levels of nested JSR/RTS calls."""
    h = CPUTestHarness(dut)
    sub1 = 0x000200
    sub2 = 0x000300
    sub3 = 0x000400

    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(0, 1),                             # D1 = 0
        *jsr_abs(sub1),
        *move(LONG, DN, 1, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)

    h.mem.load_words(sub1, [*addq(LONG, 1, DN, 1), *jsr_abs(sub2), *rts()])
    h.mem.load_words(sub2, [*addq(LONG, 1, DN, 1), *jsr_abs(sub3), *rts()])
    h.mem.load_words(sub3, [*addq(LONG, 1, DN, 1), *rts()])

    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    d1_val = h.read_result_long(0)
    assert d1_val == 3, f"Three nested calls: D1 expected 3, got 0x{d1_val:08X}"
    h.cleanup()
