"""
Cache control register surface tests for WF68K30L.

Coverage in this phase:
  - MOVEC decode for CACR/CAAR selector codes.
  - CACR write/read masking semantics for architecturally visible bits.
  - CAAR full-width round-trip via MOVEC.
  - Privilege violation on cache-control MOVEC in user mode.
  - Instruction/data cache behavior with data updates and WA allocation.
  - MMU TT cache-inhibit (`CI`) interaction with cache fills/allocations.
"""

import cocotb
from cocotb.triggers import RisingEdge

from cpu_harness import CPUTestHarness
from m68k_encode import (
    LONG,
    WORD,
    DN,
    AN,
    AN_IND,
    SPECIAL,
    IMMEDIATE,
    move,
    moveq,
    addq,
    subq,
    bcc,
    CC_NE,
    jsr_abs,
    rts,
    movea,
    move_to_abs_long,
    move_to_sr,
    imm_long,
    imm_word,
)

# MOVEC selector codes (Table 10-5 in MC68030 manual)
CR_CACR = 0x002
CR_CAAR = 0x802

# CACR persistent/readable bits in this surface model:
# WA(13), DBE(12), FD(9), ED(8), IBE(4), FI(1), EI(0)
CACR_RW_MASK = 0x0000_3313


def movec_dn_to_cr(dn: int, cr_sel: int):
    """Encode MOVEC Dn,Cr."""
    ext = ((dn & 0x7) << 12) | (cr_sel & 0x0FFF)
    return [0x4E7B, ext]


def movec_cr_to_dn(cr_sel: int, dn: int):
    """Encode MOVEC Cr,Dn."""
    ext = ((dn & 0x7) << 12) | (cr_sel & 0x0FFF)
    return [0x4E7A, ext]


def pmove_an_to_tt0(an: int):
    """Encode PMOVE (An),TT0."""
    return [0xF010 | (an & 0x7), 0x0800]


@cocotb.test()
async def test_movec_cacr_surface_masking(dut):
    """CACR writes keep only defined persistent bits; clear-action bits do not latch."""
    h = CPUTestHarness(dut)

    res0 = h.RESULT_BASE + 0x00
    res1 = h.RESULT_BASE + 0x10

    program = [
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
        *imm_long(0xFFFF_FFFF),
        *movec_dn_to_cr(0, CR_CACR),            # MOVEC D0,CACR
        *movec_cr_to_dn(CR_CACR, 1),            # MOVEC CACR,D1
        *move_to_abs_long(LONG, DN, 1, res0),

        *move(LONG, SPECIAL, IMMEDIATE, DN, 2),
        *imm_long(0x0000_0C0C),                 # CD/CED/CI/CEI only
        *movec_dn_to_cr(2, CR_CACR),            # MOVEC D2,CACR
        *movec_cr_to_dn(CR_CACR, 3),            # MOVEC CACR,D3
        *move_to_abs_long(LONG, DN, 3, res1),

        *h.sentinel_program(),
    ]

    await h.setup(program)
    found = await h.run_until_sentinel(max_cycles=12000)
    assert found, "CACR MOVEC surface test did not complete"

    got0 = h.mem.read(res0, 4)
    got1 = h.mem.read(res1, 4)

    assert got0 == CACR_RW_MASK, f"Expected CACR masked value 0x{CACR_RW_MASK:08X}, got 0x{got0:08X}"
    assert got1 == 0x0000_0000, f"Expected CACR clear-action bits not to latch, got 0x{got1:08X}"
    h.cleanup()


@cocotb.test()
async def test_movec_caar_roundtrip(dut):
    """CAAR should round-trip all 32 bits through MOVEC."""
    h = CPUTestHarness(dut)

    value = 0xA5F0_3C2D
    res = h.RESULT_BASE + 0x20

    program = [
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
        *imm_long(value),
        *movec_dn_to_cr(0, CR_CAAR),            # MOVEC D0,CAAR
        *movec_cr_to_dn(CR_CAAR, 1),            # MOVEC CAAR,D1
        *move_to_abs_long(LONG, DN, 1, res),
        *h.sentinel_program(),
    ]

    await h.setup(program)
    found = await h.run_until_sentinel(max_cycles=8000)
    assert found, "CAAR MOVEC round-trip test did not complete"

    got = h.mem.read(res, 4)
    assert got == value, f"CAAR round-trip mismatch: got 0x{got:08X}, expected 0x{value:08X}"
    h.cleanup()


@cocotb.test()
async def test_movec_cache_privilege_violation_user_mode(dut):
    """User mode execution of MOVEC to cache control register should trap to vector 8."""
    h = CPUTestHarness(dut)
    handler_addr = 0x000660
    vector_addr = 8 * 4

    handler_code = [
        *moveq(0x08, 1),
        *move_to_abs_long(LONG, DN, 1, h.RESULT_BASE),
        *h.sentinel_program(),
    ]

    program = [
        *moveq(0x22, 1),                      # Marker if no exception occurs.
        *move(WORD, SPECIAL, IMMEDIATE, DN, 5),
        *imm_word(0x0000),                    # Clear S-bit -> user mode.
        *move_to_sr(DN, 5),
        *moveq(0x01, 0),
        *movec_dn_to_cr(0, CR_CACR),          # Privileged
        *move_to_abs_long(LONG, DN, 1, h.RESULT_BASE),
        *h.sentinel_program(),
    ]

    await h.setup(program)
    h.mem.load_long(vector_addr, handler_addr)
    h.mem.load_words(handler_addr, handler_code)

    found = await h.run_until_sentinel(max_cycles=9000)
    assert found, "MOVEC cache privilege-violation test did not complete"

    marker = h.read_result_long(0)
    assert marker == 0x08, f"Expected privilege vector handler marker 0x08, got 0x{marker:08X}"
    h.cleanup()


@cocotb.test()
async def test_icache_hits_reduce_branch_loop_refetches(dut):
    """With EI enabled, repeated branch-loop fetches should hit and reduce bus reads."""
    h = CPUTestHarness(dut)

    # Program map (base 0x000100):
    # 0x0100 moveq #6,d2
    # 0x0102 subq.l #1,d2
    # 0x0104 bne.s 0x0102
    # 0x0106 move.l #1,d0
    # 0x010c movec d0,cacr   (enable EI)
    # 0x0110 moveq #6,d3
    # 0x0112 subq.l #1,d3
    # 0x0114 bne.s 0x0112
    loop1_addrs = {0x000102, 0x000104}
    loop2_addrs = {0x000112, 0x000114}
    res0 = h.RESULT_BASE + 0x40
    res1 = h.RESULT_BASE + 0x44

    program = [
        *moveq(6, 2),
        *subq(LONG, 1, DN, 2),
        *bcc(CC_NE, -4),

        *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
        *imm_long(0x0000_0001),                 # EI=1
        *movec_dn_to_cr(0, CR_CACR),

        *moveq(6, 3),
        *subq(LONG, 1, DN, 3),
        *bcc(CC_NE, -4),

        *move_to_abs_long(LONG, DN, 2, res0),
        *move_to_abs_long(LONG, DN, 3, res1),
        *h.sentinel_program(),
    ]

    await h.setup(program)

    found = False
    prev_as_n = 1
    loop1_fetches = 0
    loop2_fetches = 0
    for _ in range(30000):
        await RisingEdge(dut.CLK)
        try:
            as_n = int(dut.ASn.value)
            rw_n = int(dut.RWn.value)
        except ValueError:
            continue
        if prev_as_n == 1 and as_n == 0 and rw_n == 1:
            addr = int(dut.ADR_OUT.value)
            if addr in loop1_addrs:
                loop1_fetches += 1
            if addr in loop2_addrs:
                loop2_fetches += 1
        prev_as_n = as_n
        if h.mem.read(h.SENTINEL_ADDR, 4) == h.SENTINEL_VAL:
            found = True
            break

    assert found, "I-cache loop fetch test did not complete"
    assert h.mem.read(res0, 4) == 0, "Loop-1 counter did not terminate at zero"
    assert h.mem.read(res1, 4) == 0, "Loop-2 counter did not terminate at zero"
    assert loop2_fetches < loop1_fetches, (
        f"Expected fewer loop fetches with EI enabled: loop1={loop1_fetches}, loop2={loop2_fetches}"
    )
    h.cleanup()


@cocotb.test()
async def test_icache_cei_clears_target_entry(dut):
    """CAAR + CEI should invalidate the selected instruction longword and force refetch."""
    h = CPUTestHarness(dut)

    sub_addr = 0x000800
    res = h.RESULT_BASE + 0x48

    program = [
        *moveq(0, 6),                               # D6 = call counter

        *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
        *imm_long(0x0000_0001),                    # EI=1
        *movec_dn_to_cr(0, CR_CACR),

        *jsr_abs(sub_addr),                        # First call: miss/fill
        *jsr_abs(sub_addr),                        # Second call: hit (no refetch expected)

        *move(LONG, SPECIAL, IMMEDIATE, DN, 1),
        *imm_long(sub_addr),                       # CAAR index selects subroutine longword
        *movec_dn_to_cr(1, CR_CAAR),

        *move(LONG, SPECIAL, IMMEDIATE, DN, 2),
        *imm_long(0x0000_0005),                    # CEI action + EI remains set
        *movec_dn_to_cr(2, CR_CACR),

        *jsr_abs(sub_addr),                        # Third call: should refetch after CEI
        *move_to_abs_long(LONG, DN, 6, res),
        *h.sentinel_program(),
    ]
    subroutine = [
        *addq(LONG, 1, DN, 6),
        *rts(),
    ]

    await h.setup(program)
    h.mem.load_words(sub_addr, subroutine)

    found = False
    prev_as_n = 1
    sub_fetches = 0
    for _ in range(35000):
        await RisingEdge(dut.CLK)
        try:
            as_n = int(dut.ASn.value)
            rw_n = int(dut.RWn.value)
        except ValueError:
            continue
        if prev_as_n == 1 and as_n == 0 and rw_n == 1:
            addr = int(dut.ADR_OUT.value)
            if addr == sub_addr:
                sub_fetches += 1
        prev_as_n = as_n
        if h.mem.read(h.SENTINEL_ADDR, 4) == h.SENTINEL_VAL:
            found = True
            break

    assert found, "I-cache CEI invalidation test did not complete"
    assert h.mem.read(res, 4) == 3, "Subroutine should have executed exactly three times"
    assert sub_fetches == 2, (
        f"Expected two fetches of subroutine head (warm + post-CEI), got {sub_fetches}"
    )
    h.cleanup()


@cocotb.test()
async def test_dcache_read_hit_after_fill(dut):
    """ED=1: second longword read of same address should hit in data cache."""
    h = CPUTestHarness(dut)
    data_addr = h.DATA_BASE + 0x100
    res = h.RESULT_BASE + 0x60

    program = [
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
        *imm_long(0x0000_0100),                    # ED=1
        *movec_dn_to_cr(0, CR_CACR),
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(data_addr),
        *move(LONG, AN_IND, 0, DN, 1),             # miss/fill
        *move(LONG, AN_IND, 0, DN, 2),             # should hit
        *move_to_abs_long(LONG, DN, 2, res),
        *h.sentinel_program(),
    ]

    h.mem.load_long(data_addr, 0x89AB_CDEF)
    await h.setup(program)

    found = False
    prev_as_n = 1
    data_reads = 0
    for _ in range(25000):
        await RisingEdge(dut.CLK)
        try:
            as_n = int(dut.ASn.value)
            rw_n = int(dut.RWn.value)
        except ValueError:
            continue
        if prev_as_n == 1 and as_n == 0 and rw_n == 1 and int(dut.ADR_OUT.value) == data_addr:
            data_reads += 1
        prev_as_n = as_n
        if h.mem.read(h.SENTINEL_ADDR, 4) == h.SENTINEL_VAL:
            found = True
            break

    assert found, "D-cache read-hit test did not complete"
    assert h.mem.read(res, 4) == 0x89AB_CDEF, "Cached read returned wrong value"
    assert data_reads == 1, f"Expected one external read (miss then hit), saw {data_reads}"
    h.cleanup()


@cocotb.test()
async def test_dcache_write_hit_updates_entry(dut):
    """ED=1: write hit should update cached longword; following read should hit."""
    h = CPUTestHarness(dut)
    data_addr = h.DATA_BASE + 0x120
    res = h.RESULT_BASE + 0x64

    program = [
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
        *imm_long(0x0000_0100),                    # ED=1
        *movec_dn_to_cr(0, CR_CACR),
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(data_addr),
        *move(LONG, AN_IND, 0, DN, 1),             # miss/fill
        *move(LONG, SPECIAL, IMMEDIATE, DN, 2),
        *imm_long(0x1122_3344),
        *move(LONG, DN, 2, AN_IND, 0),             # write hit
        *move(LONG, AN_IND, 0, DN, 3),             # should hit updated entry
        *move_to_abs_long(LONG, DN, 3, res),
        *h.sentinel_program(),
    ]

    h.mem.load_long(data_addr, 0xDEAD_BEEF)
    await h.setup(program)

    found = False
    prev_as_n = 1
    data_reads = 0
    for _ in range(26000):
        await RisingEdge(dut.CLK)
        try:
            as_n = int(dut.ASn.value)
            rw_n = int(dut.RWn.value)
        except ValueError:
            continue
        if prev_as_n == 1 and as_n == 0 and rw_n == 1 and int(dut.ADR_OUT.value) == data_addr:
            data_reads += 1
        prev_as_n = as_n
        if h.mem.read(h.SENTINEL_ADDR, 4) == h.SENTINEL_VAL:
            found = True
            break

    assert found, "D-cache write-hit update test did not complete"
    assert h.mem.read(res, 4) == 0x1122_3344, "Post-write read did not return updated value"
    assert data_reads == 1, f"Expected one external read (warm fill only), saw {data_reads}"
    h.cleanup()


@cocotb.test()
async def test_dcache_wa_allocates_on_write_miss(dut):
    """ED=1,WA=1: aligned long write miss allocates; following read should hit."""
    h = CPUTestHarness(dut)
    data_addr = h.DATA_BASE + 0x140
    res = h.RESULT_BASE + 0x68

    program = [
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
        *imm_long(0x0000_2100),                    # WA=1, ED=1
        *movec_dn_to_cr(0, CR_CACR),
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(data_addr),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 1),
        *imm_long(0xCAFEBABE),
        *move(LONG, DN, 1, AN_IND, 0),             # write miss (allocate)
        *move(LONG, AN_IND, 0, DN, 2),             # should hit
        *move_to_abs_long(LONG, DN, 2, res),
        *h.sentinel_program(),
    ]

    h.mem.load_long(data_addr, 0x0000_0000)
    await h.setup(program)

    found = False
    prev_as_n = 1
    data_reads = 0
    for _ in range(26000):
        await RisingEdge(dut.CLK)
        try:
            as_n = int(dut.ASn.value)
            rw_n = int(dut.RWn.value)
        except ValueError:
            continue
        if prev_as_n == 1 and as_n == 0 and rw_n == 1 and int(dut.ADR_OUT.value) == data_addr:
            data_reads += 1
        prev_as_n = as_n
        if h.mem.read(h.SENTINEL_ADDR, 4) == h.SENTINEL_VAL:
            found = True
            break

    assert found, "D-cache WA allocation test did not complete"
    assert h.mem.read(res, 4) == 0xCAFEBABE, "Read after WA write-miss allocation mismatched"
    assert data_reads == 0, f"Expected zero external data reads, saw {data_reads}"
    h.cleanup()


@cocotb.test()
async def test_dcache_cd_forces_refill(dut):
    """CD action should invalidate data cache so next read misses/refills."""
    h = CPUTestHarness(dut)
    data_addr = h.DATA_BASE + 0x180
    res = h.RESULT_BASE + 0x6C

    program = [
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
        *imm_long(0x0000_0100),                    # ED=1
        *movec_dn_to_cr(0, CR_CACR),
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(data_addr),
        *move(LONG, AN_IND, 0, DN, 1),             # miss/fill
        *move(LONG, SPECIAL, IMMEDIATE, DN, 2),
        *imm_long(0x0000_0900),                    # CD action + keep ED=1
        *movec_dn_to_cr(2, CR_CACR),               # invalidate all D-cache entries
        *move(LONG, AN_IND, 0, DN, 3),             # should miss/refill again
        *move_to_abs_long(LONG, DN, 3, res),
        *h.sentinel_program(),
    ]

    h.mem.load_long(data_addr, 0x0BAD_F00D)
    await h.setup(program)

    found = False
    prev_as_n = 1
    data_reads = 0
    for _ in range(28000):
        await RisingEdge(dut.CLK)
        try:
            as_n = int(dut.ASn.value)
            rw_n = int(dut.RWn.value)
        except ValueError:
            continue
        if prev_as_n == 1 and as_n == 0 and rw_n == 1 and int(dut.ADR_OUT.value) == data_addr:
            data_reads += 1
        prev_as_n = as_n
        if h.mem.read(h.SENTINEL_ADDR, 4) == h.SENTINEL_VAL:
            found = True
            break

    assert found, "D-cache CD invalidation test did not complete"
    assert h.mem.read(res, 4) == 0x0BAD_F00D, "Read after CD returned wrong value"
    assert data_reads == 2, f"Expected two external reads (before/after CD), saw {data_reads}"
    h.cleanup()


@cocotb.test()
async def test_icache_tt_ci_blocks_refill(dut):
    """EI=1 with TT CI=1 for supervisor program space: instruction refills should be inhibited."""
    h = CPUTestHarness(dut)
    tt_addr = h.DATA_BASE + 0x1C0
    res = h.RESULT_BASE + 0x70

    # TT0: E=1, CI=1, read-only match, FC mask=111 (match any FC).
    tt0_ci_read_any_fc = 0x00FF_8607

    loop_addrs = {0x00011E, 0x000120}
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(tt_addr),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
        *imm_long(tt0_ci_read_any_fc),
        *move(LONG, DN, 0, AN_IND, 0),             # (A0) = TT0 value
        *pmove_an_to_tt0(0),                       # PMOVE (A0),TT0

        *move(LONG, SPECIAL, IMMEDIATE, DN, 1),
        *imm_long(0x0000_0001),                    # EI=1
        *movec_dn_to_cr(1, CR_CACR),

        *moveq(6, 2),
        *subq(LONG, 1, DN, 2),                     # loop body @ 0x118
        *bcc(CC_NE, -4),                           # branch target @ 0x11A
        *move_to_abs_long(LONG, DN, 2, res),
        *h.sentinel_program(),
    ]

    await h.setup(program)

    found = False
    prev_as_n = 1
    loop_fetches = 0
    for _ in range(28000):
        await RisingEdge(dut.CLK)
        try:
            as_n = int(dut.ASn.value)
            rw_n = int(dut.RWn.value)
        except ValueError:
            continue
        if prev_as_n == 1 and as_n == 0 and rw_n == 1 and int(dut.ADR_OUT.value) in loop_addrs:
            loop_fetches += 1
        prev_as_n = as_n
        if h.mem.read(h.SENTINEL_ADDR, 4) == h.SENTINEL_VAL:
            found = True
            break

    assert found, "I-cache TT CI inhibition test did not complete"
    assert h.mem.read(res, 4) == 0, "Loop counter did not terminate at zero"
    assert loop_fetches >= 8, f"Expected repeated external loop fetches with CI inhibit, saw {loop_fetches}"
    h.cleanup()


@cocotb.test()
async def test_dcache_tt_ci_blocks_wa_allocation(dut):
    """ED=1,WA=1 with TT CI=1 for supervisor data space: write-miss allocation should be inhibited."""
    h = CPUTestHarness(dut)
    tt_addr = h.DATA_BASE + 0x1E0
    data_addr = h.DATA_BASE + 0x1A0
    res = h.RESULT_BASE + 0x74

    # TT0: E=1, CI=1, RWM=1 (read+write), FC base=supervisor data (101), FC mask=000.
    tt0_ci_super_data = 0x00FF_8750

    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(tt_addr),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
        *imm_long(tt0_ci_super_data),
        *move(LONG, DN, 0, AN_IND, 0),             # (A0) = TT0 value
        *pmove_an_to_tt0(0),                       # PMOVE (A0),TT0

        *move(LONG, SPECIAL, IMMEDIATE, DN, 1),
        *imm_long(0x0000_2100),                    # WA=1, ED=1
        *movec_dn_to_cr(1, CR_CACR),

        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(data_addr),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 2),
        *imm_long(0xCAFEBABE),
        *move(LONG, DN, 2, AN_IND, 0),             # write miss, CI should block allocation
        *move(LONG, AN_IND, 0, DN, 3),             # should miss/read from bus (not cache hit)
        *move_to_abs_long(LONG, DN, 3, res),
        *h.sentinel_program(),
    ]

    h.mem.load_long(data_addr, 0x0000_0000)
    await h.setup(program)

    found = False
    prev_as_n = 1
    data_reads = 0
    for _ in range(30000):
        await RisingEdge(dut.CLK)
        try:
            as_n = int(dut.ASn.value)
            rw_n = int(dut.RWn.value)
        except ValueError:
            continue
        if prev_as_n == 1 and as_n == 0 and rw_n == 1 and int(dut.ADR_OUT.value) == data_addr:
            data_reads += 1
        prev_as_n = as_n
        if h.mem.read(h.SENTINEL_ADDR, 4) == h.SENTINEL_VAL:
            found = True
            break

    assert found, "D-cache TT CI inhibition test did not complete"
    assert h.mem.read(res, 4) == 0xCAFEBABE, "Read after CI-inhibited WA write returned wrong value"
    assert data_reads == 1, f"Expected one external read with CI blocking WA allocation, saw {data_reads}"
    h.cleanup()
