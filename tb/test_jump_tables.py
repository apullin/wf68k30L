"""
Jump-table control-flow regressions for WF68K30L.

These tests exercise compiler-style switch lowering that uses:
  MOVE.W table(PC,Dn.W/L*scale),Dn
  JMP    table(PC,Dn.W)

This pattern is common for optimized C switch statements.
"""

import cocotb

from cpu_harness import CPUTestHarness
from m68k_encode import (
    WORD,
    LONG,
    DN,
    SPECIAL,
    PC_IDX,
    IMMEDIATE,
    move,
    moveq,
    jmp,
    jsr_abs,
    rts,
    move_to_abs_long,
    imm_long,
    _w,
)


def _brief_extension_word(da, reg, wl, scale, disp8):
    """Build brief-format indexed extension word."""
    return _w(
        (da << 15)
        | ((reg & 7) << 12)
        | ((wl & 1) << 11)
        | ((scale & 0x3) << 9)
        | (disp8 & 0xFF)
    )


def _patch_abs_long(words, hi_word_idx, value):
    """Patch two words with a 32-bit absolute address."""
    words[hi_word_idx] = (value >> 16) & 0xFFFF
    words[hi_word_idx + 1] = value & 0xFFFF


@cocotb.test()
async def test_jump_table_pc_idx_switch_pattern(dut):
    """Switch-like jump table: MOVE.W table(PC,D0.L*2),D0; JMP table(PC,D0.W)."""
    h = CPUTestHarness(dut)
    selector = 2
    expected = 0x33
    case_values = [0x11, 0x22, 0x33, 0x44]

    prefix = [*moveq(selector, 0)]
    lookup_op = move(WORD, SPECIAL, PC_IDX, DN, 0)
    jmp_op = jmp(SPECIAL, PC_IDX)

    fail_path = [
        *moveq(0x7E, 2),
        *move_to_abs_long(LONG, DN, 2, h.RESULT_BASE),
        *h.sentinel_program(),
    ]

    case_blocks = []
    for value in case_values:
        case_blocks.append(
            [
                *moveq(value, 2),
                *move_to_abs_long(LONG, DN, 2, h.RESULT_BASE),
                *h.sentinel_program(),
            ]
        )

    lookup_ext_idx = len(prefix) + len(lookup_op)
    jmp_ext_idx = len(prefix) + len(lookup_op) + 1 + len(jmp_op)
    table_start_idx = len(prefix) + len(lookup_op) + 1 + len(jmp_op) + 1 + len(
        fail_path
    )

    table_words = [0] * len(case_values)
    case_start_idx = table_start_idx + len(table_words)
    case_addrs = []
    idx = case_start_idx
    for block in case_blocks:
        case_addrs.append(h.PROGRAM_BASE + idx * 2)
        idx += len(block)

    table_addr = h.PROGRAM_BASE + table_start_idx * 2
    lookup_ext_addr = h.PROGRAM_BASE + lookup_ext_idx * 2
    jmp_ext_addr = h.PROGRAM_BASE + jmp_ext_idx * 2

    disp_lookup = table_addr - lookup_ext_addr
    disp_jmp = table_addr - jmp_ext_addr
    assert -128 <= disp_lookup <= 127
    assert -128 <= disp_jmp <= 127

    lookup_ext = _brief_extension_word(
        da=0, reg=0, wl=1, scale=1, disp8=disp_lookup
    )  # D0.L * 2
    jmp_ext = _brief_extension_word(
        da=0, reg=0, wl=0, scale=0, disp8=disp_jmp
    )  # D0.W

    table_words = [_w(case_addr - table_addr) for case_addr in case_addrs]
    program = [
        *prefix,
        *lookup_op,
        lookup_ext,
        *jmp_op,
        jmp_ext,
        *fail_path,
        *table_words,
    ]
    for block in case_blocks:
        program.extend(block)

    await h.setup(program)
    found = await h.run_until_sentinel(
        max_cycles=30000, check_bus_invariants=True, max_bus_cycle_cycles=512
    )
    assert found, "Jump-table switch did not reach sentinel"
    result = h.read_result_long(0)
    assert result == expected, (
        f"Jump-table switch returned wrong case: expected 0x{expected:08X}, "
        f"got 0x{result:08X}"
    )
    h.cleanup()


def _build_jump_table_subroutine(program_base, sub_start_idx, selector, result_reg):
    """Return words for a switch-like subroutine ending in RTS."""
    case_values = [0x51, 0x52, 0x53, 0x54]
    fail_value = 0x6E

    prefix = [*moveq(selector, 0)]
    lookup_op = move(WORD, SPECIAL, PC_IDX, DN, 0)
    jmp_op = jmp(SPECIAL, PC_IDX)
    fail_path = [*moveq(fail_value, result_reg), *rts()]
    case_blocks = [[*moveq(v, result_reg), *rts()] for v in case_values]

    lookup_ext_idx = sub_start_idx + len(prefix) + len(lookup_op)
    jmp_ext_idx = sub_start_idx + len(prefix) + len(lookup_op) + 1 + len(jmp_op)
    table_start_idx = (
        sub_start_idx
        + len(prefix)
        + len(lookup_op)
        + 1
        + len(jmp_op)
        + 1
        + len(fail_path)
    )

    table_words = [0] * len(case_values)
    case_start_idx = table_start_idx + len(table_words)
    case_addrs = []
    idx = case_start_idx
    for block in case_blocks:
        case_addrs.append(program_base + idx * 2)
        idx += len(block)

    table_addr = program_base + table_start_idx * 2
    lookup_ext_addr = program_base + lookup_ext_idx * 2
    jmp_ext_addr = program_base + jmp_ext_idx * 2

    disp_lookup = table_addr - lookup_ext_addr
    disp_jmp = table_addr - jmp_ext_addr
    assert -128 <= disp_lookup <= 127
    assert -128 <= disp_jmp <= 127

    lookup_ext = _brief_extension_word(
        da=0, reg=0, wl=1, scale=1, disp8=disp_lookup
    )  # D0.L * 2
    jmp_ext = _brief_extension_word(
        da=0, reg=0, wl=0, scale=0, disp8=disp_jmp
    )  # D0.W

    table_words = [_w(case_addr - table_addr) for case_addr in case_addrs]

    words = [*prefix, *lookup_op, lookup_ext, *jmp_op, jmp_ext, *fail_path, *table_words]
    for block in case_blocks:
        words.extend(block)
    return words


@cocotb.test()
async def test_jump_table_pc_idx_nested_jsr_rts(dut):
    """Jump-table subroutine after nested JSR/RTS must return correct case."""
    h = CPUTestHarness(dut)
    selector = 1
    expected = 0x52

    main = [
        *jsr_abs(0),  # Patched to wrapper.
        *move_to_abs_long(LONG, DN, 4, h.RESULT_BASE),
        *h.sentinel_program(),
    ]
    main_jsr_hi_idx = 1

    wrapper_start_idx = len(main)
    wrapper = [
        *jsr_abs(0),  # Patched to switch subroutine.
        *rts(),
    ]
    wrapper_jsr_hi_idx = wrapper_start_idx + 1

    sub_start_idx = len(main) + len(wrapper)
    sub = _build_jump_table_subroutine(
        program_base=h.PROGRAM_BASE,
        sub_start_idx=sub_start_idx,
        selector=selector,
        result_reg=4,
    )

    program = [*main, *wrapper, *sub]

    wrapper_addr = h.PROGRAM_BASE + wrapper_start_idx * 2
    sub_addr = h.PROGRAM_BASE + sub_start_idx * 2
    _patch_abs_long(program, main_jsr_hi_idx, wrapper_addr)
    _patch_abs_long(program, wrapper_jsr_hi_idx, sub_addr)

    await h.setup(program)
    found = await h.run_until_sentinel(
        max_cycles=30000, check_bus_invariants=True, max_bus_cycle_cycles=512
    )
    assert found, "Nested JSR/RTS jump-table did not reach sentinel"
    result = h.read_result_long(0)
    assert result == expected, (
        f"Nested JSR/RTS jump-table returned wrong case: expected 0x{expected:08X}, "
        f"got 0x{result:08X}"
    )
    h.cleanup()
