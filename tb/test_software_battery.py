"""
Software-style end-to-end smoke battery for WF68K30L.

These tests are intentionally longer than instruction-unit regressions and
exercise mixed control flow, data path, memory traffic, and bus handshakes.
Each case enables lightweight bus invariants in CPUTestHarness.
"""

import cocotb

from cpu_harness import CPUTestHarness
from m68k_encode import (
    BYTE, WORD, LONG,
    DN, AN, AN_IND, SPECIAL, IMMEDIATE,
    moveq, move, movea, addq, add, clr,
    jsr_abs, rts, mulu_w, divu_w, divs_w, movem_to_mem, movem_from_mem,
    imm_long, imm_word,
    asl, asr, lsl, lsr,
    move_from_ccr, move_to_ccr, move_to_abs_long,
)
# m68k_reference uses the same BYTE/WORD/LONG = 0/1/2 convention as m68k_encode.
from m68k_reference import (
    cc_shift, mask_val, size_bits, size_mask,
    _self_test as m68k_reference_self_test,
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


# ---------------------------------------------------------------------------
# Reference-model coverage
#
# tb/m68k_reference.py computes the expected condition codes for a large part
# of the instruction suite, so a defect in it silently relaxes every
# expectation derived from it. cc_shift() in particular had no caller at all --
# the shift tests hardcode their flags -- so cc_shift('asl', count == size)
# returned C=0, where the last bit shifted out is source bit 0, for the life of
# the file. The two tests below close both halves: the model's own self-test now
# runs in the regression, and the count == size boundary is cross-checked
# against the RTL so model and hardware cannot drift apart unnoticed.
# ---------------------------------------------------------------------------

# (op, size, count, count_in_register, source)
_SHIFT_CARRY_CASES = (
    ('asl', BYTE, 8, False, 0x12345601),   # C = source bit 0 = 1
    ('asl', BYTE, 8, False, 0x123456FE),   # C = source bit 0 = 0
    ('lsl', BYTE, 8, False, 0x12345601),   # correct-by-construction twin
    ('asl', WORD, 16, True, 0x12340001),   # register count, count == size
    ('asl', LONG, 32, True, 0x00000001),   # register count, count == size
    ('asl', BYTE, 9, True, 0x12345601),    # count > size -> C = 0
    ('asr', BYTE, 8, False, 0x12345680),   # sign bit is the last one out
    ('lsr', BYTE, 8, False, 0x12345680),
)

_SHIFT_ENCODERS = {'asl': asl, 'asr': asr, 'lsl': lsl, 'lsr': lsr}


def _shift_result(op, size, count, src):
    """Model the shifted value, preserving the bits above the operand size."""
    bits = size_bits(size)
    mask = size_mask(size)
    val = src & mask
    if op in ('asl', 'lsl'):
        res = (val << count) & mask
    elif op == 'lsr':
        res = (val >> count) & mask if count < bits else 0
    else:  # asr
        sign = (val >> (bits - 1)) & 1
        if count >= bits:
            res = mask if sign else 0
        else:
            res = (val >> count) | (((mask << (bits - count)) & mask) if sign else 0)
    return (src & ~mask & 0xFFFFFFFF) | res


def _ccr_bits(ccr):
    return "".join(
        name if (ccr >> shift) & 1 else "-"
        for name, shift in zip("XNZVC", (4, 3, 2, 1, 0))
    )


@cocotb.test()
async def test_reference_model_self_test(dut):
    """The CCR reference model must pass its own self-test.

    Pure Python: the model gates a large share of the instruction suite's
    expectations, and until now no make target ran its self-test.
    """
    assert m68k_reference_self_test(), (
        "tb/m68k_reference.py self-test failed; see the printed errors. Every "
        "test that derives expected condition codes from this model is "
        "checking against a broken reference."
    )


@cocotb.test()
async def test_shift_carry_at_count_equals_size(dut):
    """Shift carry at count == operand size: RTL against the reference model."""
    h = CPUTestHarness(dut)

    program = []
    for index, (op, size, count, reg_count, src) in enumerate(_SHIFT_CARRY_CASES):
        off = index * 8
        program += [
            *_load_imm_long_to_dn(src, 0),
            *move_to_ccr(SPECIAL, IMMEDIATE), *imm_word(0x0000),  # X=N=Z=V=C=0
        ]
        if reg_count:
            program += [
                *moveq(count, 2),
                *_SHIFT_ENCODERS[op](size, 2, 0, ir=1),
            ]
        else:
            # An immediate count of 8 is encoded as 0 (counts run 1..8).
            program += [*_SHIFT_ENCODERS[op](size, count & 7, 0, ir=0)]
        program += [
            *move_from_ccr(DN, 6),                        # capture before stores
            *move_to_abs_long(LONG, DN, 6, h.RESULT_BASE + off),
            *move_to_abs_long(LONG, DN, 0, h.RESULT_BASE + off + 4),
        ]
    program += h.sentinel_program()

    await h.setup(program)
    found = await h.run_until_sentinel(max_cycles=20000)
    assert found, "Sentinel not reached"

    failures = []
    for index, (op, size, count, _reg_count, src) in enumerate(_SHIFT_CARRY_CASES):
        off = index * 8
        got_ccr = h.read_result_long(off) & 0x1F
        got_val = h.read_result_long(off + 4)
        expect_val = _shift_result(op, size, count, src)
        expect_cc = cc_shift(op, size, count, mask_val(src, size),
                             mask_val(expect_val, size), 0)
        expect_ccr = (
            ((expect_cc['x'] or 0) << 4)
            | (expect_cc['n'] << 3)
            | (expect_cc['z'] << 2)
            | (expect_cc['v'] << 1)
            | expect_cc['c']
        )
        name = f"{op.upper()}.{'BWL'[size]} #{count} of 0x{src:08X}"
        if got_val != expect_val:
            failures.append(
                f"{name}: result exp=0x{expect_val:08X} got=0x{got_val:08X}"
            )
        if got_ccr != expect_ccr:
            failures.append(
                f"{name}: CCR exp={_ccr_bits(expect_ccr)} got={_ccr_bits(got_ccr)}"
            )
    h.cleanup()
    assert not failures, "Shift carry mismatches:\n  " + "\n  ".join(failures)
