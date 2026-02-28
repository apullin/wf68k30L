"""
Software-style end-to-end smoke battery for WF68K30L.

These tests are intentionally longer than instruction-unit regressions and
exercise mixed control flow, data path, memory traffic, and bus handshakes.
Each case enables lightweight bus invariants in CPUTestHarness.
"""

import cocotb

from cpu_harness import CPUTestHarness
from m68k_encode import (
    LONG,
    DN, AN, AN_IND, SPECIAL, IMMEDIATE,
    moveq, move, movea, addq, add, clr,
    jsr_abs, rts, mulu_w, divu_w, divs_w, movem_to_mem, movem_from_mem,
    imm_long,
)


def _load_imm_long_to_dn(value, dn):
    """Emit MOVE.L #imm,Dn."""
    return [
        *move(LONG, SPECIAL, IMMEDIATE, DN, dn),
        *imm_long(value),
    ]


def _store_and_advance(dn, an):
    """Emit MOVE.L Dn,(An) ; ADDQ.L #4,An."""
    return [
        *move(LONG, DN, dn, AN_IND, an),
        *addq(LONG, 4, AN, an),
    ]


@cocotb.test()
async def test_sw_loop_sum_and_subroutine(dut):
    """Call-chain kernel using repeated JSR/RTS plus arithmetic/store."""
    h = CPUTestHarness(dut)

    # Subroutine at fixed absolute address:
    #   ADDQ.L #1,D1
    #   RTS
    sub_addr = 0x000300

    # Main program:
    #   A0 = RESULT_BASE
    #   D1 = 0
    #   JSR sub_addr three times (D1 increments in subroutine)
    #   ADDQ.L #5,D1
    #   store D1
    #   sentinel
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),   # A0 = RESULT_BASE
        *imm_long(h.RESULT_BASE),
        *moveq(0, 1),                           # D1 = 0
        *jsr_abs(sub_addr),                     # call sub #1
        *jsr_abs(sub_addr),                     # call sub #2
        *jsr_abs(sub_addr),                     # call sub #3
        *addq(LONG, 5, DN, 1),                  # D1 += 5
        *_store_and_advance(1, 0),              # RESULT[0] = D1
        *h.sentinel_program(),
    ]

    await h.setup(program)
    h.mem.load_words(sub_addr, [
        *addq(LONG, 1, DN, 1),
        *rts(),
    ])
    found = await h.run_until_sentinel(
        check_bus_invariants=True,
        max_bus_cycle_cycles=192,
    )
    assert found, "Sentinel not reached"

    result_d1 = h.read_result_long(0)
    assert result_d1 == 8, (
        f"Call-chain result mismatch: expected 8, got 0x{result_d1:08X}"
    )
    h.cleanup()


@cocotb.test()
async def test_sw_movem_roundtrip(dut):
    """Round-trip register set through MOVEM memory image and combine values."""
    h = CPUTestHarness(dut)

    # Memory image for MOVEM target/source.
    image_addr = h.DATA_BASE + 0x100

    # Program:
    #   A1 = image_addr, A0 = RESULT_BASE
    #   D0/D1 = constants
    #   MOVEM.L D0-D1,(A1)
    #   CLR.L D0/D1
    #   MOVEM.L (A1),D0-D1
    #   ADD.L D1,D0
    #   store D0, D1
    #   sentinel
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 1),   # A1 = image_addr
        *imm_long(image_addr),
        *movea(LONG, SPECIAL, IMMEDIATE, 0),   # A0 = RESULT_BASE
        *imm_long(h.RESULT_BASE),
        *_load_imm_long_to_dn(0x11112222, 0),  # D0
        *_load_imm_long_to_dn(0x33334444, 1),  # D1
        *movem_to_mem(LONG, AN_IND, 1, 0x0003), # MOVEM.L D0-D1,(A1)
        *clr(LONG, DN, 0),                      # D0 = 0
        *clr(LONG, DN, 1),                      # D1 = 0
        *movem_from_mem(LONG, AN_IND, 1, 0x0003), # MOVEM.L (A1),D0-D1
        *add(LONG, 0, 0, DN, 1),               # D0 += D1
        *_store_and_advance(0, 0),              # RESULT[0] = D0
        *_store_and_advance(1, 0),              # RESULT[1] = D1
        *h.sentinel_program(),
    ]

    await h.setup(program)
    found = await h.run_until_sentinel(
        check_bus_invariants=True,
        max_bus_cycle_cycles=192,
    )
    assert found, "Sentinel not reached"

    result_d0 = h.read_result_long(0)
    result_d1 = h.read_result_long(4)
    assert result_d0 == 0x44446666, (
        f"MOVEM+ADD result mismatch: expected 0x44446666, got 0x{result_d0:08X}"
    )
    assert result_d1 == 0x33334444, (
        f"MOVEM restore mismatch: expected 0x33334444, got 0x{result_d1:08X}"
    )
    h.cleanup()


@cocotb.test()
async def test_sw_mul_div_kernel(dut):
    """Small arithmetic kernel exercising MULU, DIVU, and DIVS data paths."""
    h = CPUTestHarness(dut)

    # Program:
    #   D0 = 300; D1 = 7
    #   MULU.W D1,D0      => 2100
    #   D2 = 10
    #   DIVU.W D2,D0      => remainder=0, quotient=210 (0x000000D2)
    #   D4 = -300; D5 = 7
    #   DIVS.W D5,D4      => remainder=-6, quotient=-42 (0xFFFAFFD6)
    #   store D0, D4
    #   sentinel
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),   # A0 = RESULT_BASE
        *imm_long(h.RESULT_BASE),
        *_load_imm_long_to_dn(300, 0),
        *moveq(7, 1),
        *mulu_w(DN, 1, 0),                      # D0 = 300 * 7
        *moveq(10, 2),
        *divu_w(DN, 2, 0),                      # D0 = (r:0, q:210)
        *_load_imm_long_to_dn(0xFFFFFED4, 4),   # D4 = -300
        *moveq(7, 5),
        *divs_w(DN, 5, 4),                      # D4 = (r:-6, q:-42)
        *_store_and_advance(0, 0),
        *_store_and_advance(4, 0),
        *h.sentinel_program(),
    ]

    await h.setup(program)
    found = await h.run_until_sentinel(
        check_bus_invariants=True,
        max_bus_cycle_cycles=192,
    )
    assert found, "Sentinel not reached"

    result_divu = h.read_result_long(0)
    result_divs = h.read_result_long(4)
    assert result_divu == 0x000000D2, (
        f"DIVU kernel mismatch: expected 0x000000D2, got 0x{result_divu:08X}"
    )
    assert result_divs == 0xFFFAFFD6, (
        f"DIVS kernel mismatch: expected 0xFFFAFFD6, got 0x{result_divs:08X}"
    )
    h.cleanup()
