"""
Randomized MMU runtime translation campaign against a local reference expectation.

This is a lightweight differential-style check:
  - Randomizes descriptor format mix (short/long), FCL on/off, and indirection.
  - Generates expected "success vs fault" outcomes from the generated descriptors.
  - Verifies the RTL returns translated data on success or vectors through 56 on fault.
"""

import random

import cocotb

from cpu_harness import CPUTestHarness
from m68k_encode import (
    LONG,
    DN,
    AN_IND,
    SPECIAL,
    IMMEDIATE,
    move,
    movea,
    moveq,
    move_to_abs_long,
    imm_long,
)


def _sentinel_program():
    words = []
    words.extend(move(LONG, SPECIAL, IMMEDIATE, DN, 7))
    words.append(0xDEAD)
    words.append(0xCAFE)
    words.extend(move_to_abs_long(LONG, DN, 7, 0x00030000))
    return words


def _mk_short_table_desc(table_addr):
    return (table_addr & 0xFFFFFFF0) | 0x00000002


def _mk_short_page_desc(page_base):
    return (page_base & 0xFFFFFF00) | 0x00000001


def _mk_long_table_desc(table_addr, limit=0x7FFF, lower=0):
    lo = ((lower & 1) << 31) | ((limit & 0x7FFF) << 16) | 0x00000003
    hi = table_addr & 0xFFFFFFF0
    return lo, hi


def _mk_long_page_desc(page_base):
    lo = 0x00000001
    hi = page_base & 0xFFFFFF00
    return lo, hi


def _build_case(rng, case_idx):
    use_long = bool(rng.getrandbits(1))
    use_fcl = bool(rng.getrandbits(1))
    use_indirect = bool(rng.getrandbits(1))

    # Keep the fault model focused and deterministic.
    fault_mode = "none"
    roll = rng.randrange(10)
    if roll == 0:
        fault_mode = "invalid"
    elif roll == 1 and use_long and not use_fcl:
        fault_mode = "limit"

    logical_addr = 0x12345008
    page_base = 0x00400000 + ((case_idx & 0x3F) << 12)
    expected_value = rng.getrandbits(32)
    logical_value = expected_value ^ 0xFFFFFFFF

    root_tbl = 0x00004000
    tbl_a = 0x00004400
    tbl_b = 0x00004800
    tbl_c = 0x00004C00
    ind_page_desc = 0x00005000

    desc_size = 8 if use_long else 4
    desc_shift = 3 if use_long else 2

    root_index = 5 if use_fcl else 0x12
    root_next = tbl_a if use_fcl else tbl_b
    root_desc_addr = root_tbl + (root_index << desc_shift)

    a_desc_addr = tbl_a + (0x12 << desc_shift)
    b_desc_addr = tbl_b + (0x34 << desc_shift)
    c_desc_addr = tbl_c + (0x5 << desc_shift)

    mem_init = {}
    prime_addrs = []

    def put_long(addr, value):
        mem_init[addr & 0xFFFFF] = value & 0xFFFFFFFF

    def prime_word(addr):
        prime_addrs.append(addr & 0xFFFFF)

    # Root pointer register value.
    if use_long:
        root_limit = 0x7FFF
        if fault_mode == "limit":
            # A-index is 0x12; force upper limit below that value.
            root_limit = 0x0008
        crp_hi = ((root_limit & 0x7FFF) << 16) | 0x00000003
    else:
        crp_hi = 0x7FFF0002
    crp_lo = root_tbl

    # Root descriptor -> next table.
    if use_long:
        root_lo, root_hi = _mk_long_table_desc(root_next)
        put_long(root_desc_addr + 0, root_lo)
        put_long(root_desc_addr + 4, root_hi)
        prime_word(root_desc_addr + 0)
        prime_word(root_desc_addr + 4)
    else:
        put_long(root_desc_addr, _mk_short_table_desc(root_next))
        prime_word(root_desc_addr)

    # Intermediate descriptor(s): always table descriptors.
    if use_fcl:
        if use_long:
            a_lo, a_hi = _mk_long_table_desc(tbl_b)
            put_long(a_desc_addr + 0, a_lo)
            put_long(a_desc_addr + 4, a_hi)
            prime_word(a_desc_addr + 0)
            prime_word(a_desc_addr + 4)
        else:
            put_long(a_desc_addr, _mk_short_table_desc(tbl_b))
            prime_word(a_desc_addr)

    if use_long:
        b_lo, b_hi = _mk_long_table_desc(tbl_c)
        put_long(b_desc_addr + 0, b_lo)
        put_long(b_desc_addr + 4, b_hi)
        prime_word(b_desc_addr + 0)
        prime_word(b_desc_addr + 4)
    else:
        put_long(b_desc_addr, _mk_short_table_desc(tbl_c))
        prime_word(b_desc_addr)

    expect_fault = fault_mode in ("invalid", "limit")

    # Leaf descriptor: page or indirect, with optional forced invalid descriptor.
    if use_long:
        if fault_mode == "invalid":
            c_lo, c_hi = (0x00000000, 0x00000000)
        elif use_indirect:
            c_lo, c_hi = (0x00000003, ind_page_desc)
            p_lo, p_hi = _mk_long_page_desc(page_base)
            put_long(ind_page_desc + 0, p_lo)
            put_long(ind_page_desc + 4, p_hi)
            prime_word(ind_page_desc + 0)
            prime_word(ind_page_desc + 4)
        else:
            c_lo, c_hi = _mk_long_page_desc(page_base)
        put_long(c_desc_addr + 0, c_lo)
        put_long(c_desc_addr + 4, c_hi)
        prime_word(c_desc_addr + 0)
        prime_word(c_desc_addr + 4)
    else:
        if fault_mode == "invalid":
            c = 0x00000000
        elif use_indirect:
            c = (ind_page_desc & 0xFFFFFFFC) | 0x00000002
            put_long(ind_page_desc, _mk_short_page_desc(page_base))
            prime_word(ind_page_desc)
        else:
            c = _mk_short_page_desc(page_base)
        put_long(c_desc_addr, c)
        prime_word(c_desc_addr)

    expected_phys = (page_base + (logical_addr & 0xFFF)) & 0xFFFFF
    put_long(expected_phys, expected_value)
    put_long(logical_addr & 0xFFFFF, logical_value)

    tt0_src = 0x00018000
    tt1_src = 0x00018020
    crp_src = 0x00018040
    tc_src = 0x00018060

    tt0_prog = 0x00008360
    tt1_prog = 0x00008350
    tc_val = 0x80C08840 | (0x01000000 if use_fcl else 0x00000000)

    put_long(tt0_src, tt0_prog)
    put_long(tt1_src, tt1_prog)
    put_long(crp_src + 0, crp_hi)
    put_long(crp_src + 4, crp_lo)
    put_long(tc_src, tc_val)

    program = []
    for addr in prime_addrs:
        program.extend(movea(LONG, SPECIAL, IMMEDIATE, 2))
        program.extend(imm_long(addr))
        program.extend(move(LONG, AN_IND, 2, DN, 2))

    program.extend(movea(LONG, SPECIAL, IMMEDIATE, 6))
    program.extend(imm_long(tt0_src))
    program.extend([0xF016, 0x0800])  # PMOVE (A6),TT0
    program.extend(movea(LONG, SPECIAL, IMMEDIATE, 7))
    program.extend(imm_long(tt1_src))
    program.extend([0xF017, 0x0C00])  # PMOVE (A7),TT1

    program.extend(movea(LONG, SPECIAL, IMMEDIATE, 1))
    program.extend(imm_long(crp_src))
    program.extend([0xF011, 0x4C00])  # PMOVE (A1),CRP
    program.extend(movea(LONG, SPECIAL, IMMEDIATE, 2))
    program.extend(imm_long(tc_src))
    program.extend([0xF012, 0x4000])  # PMOVE (A2),TC

    program.extend(movea(LONG, SPECIAL, IMMEDIATE, 0))
    program.extend(imm_long(logical_addr))
    program.extend(move(LONG, AN_IND, 0, DN, 1))
    program.extend(move_to_abs_long(LONG, DN, 1, 0x00020000))
    program.extend(_sentinel_program())

    return {
        "program": program,
        "mem_init": mem_init,
        "expect_fault": expect_fault,
        "expected_value": expected_value,
        "params": {
            "long": use_long,
            "fcl": use_fcl,
            "indirect": use_indirect,
            "fault_mode": fault_mode,
        },
    }


@cocotb.test()
async def test_mmu_runtime_random_descriptor_campaign(dut):
    """Randomized MMU campaign over descriptor formats/FCL/indirection/fault cases."""
    rng = random.Random(0x68030)

    handler_addr = 0x0009C0
    vector_addr = 56 * 4
    handler_code = [
        *moveq(0x38, 1),
        *move_to_abs_long(LONG, DN, 1, 0x00020000),
        *_sentinel_program(),
    ]

    num_cases = 24
    for case_idx in range(num_cases):
        case = _build_case(rng, case_idx)
        h = CPUTestHarness(dut)
        await h.setup(case["program"])

        h.mem.load_long(vector_addr, handler_addr)
        h.mem.load_words(handler_addr, handler_code)
        for addr, value in case["mem_init"].items():
            h.mem.load_long(addr, value)

        found = await h.run_until_sentinel(max_cycles=180000)
        assert found, f"Random MMU case {case_idx} did not complete: {case['params']}"

        got = h.read_result_long(0)
        if case["expect_fault"]:
            assert got == 0x38, (
                f"Random MMU case {case_idx} expected vector56 marker, got 0x{got:08X}; "
                f"params={case['params']}"
            )
        else:
            assert got == case["expected_value"], (
                f"Random MMU case {case_idx} expected 0x{case['expected_value']:08X}, got 0x{got:08X}; "
                f"params={case['params']}"
            )

        h.cleanup()
