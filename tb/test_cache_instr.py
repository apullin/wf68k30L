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
    BYTE,
    LONG,
    WORD,
    DN,
    AN,
    AN_IND,
    SPECIAL,
    IMMEDIATE,
    move,
    moveq,
    nop,
    addq,
    subq,
    bra,
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


def pmove_an_to_tt1(an: int):
    """Encode PMOVE (An),TT1."""
    return [0xF010 | (an & 0x7), 0x0C00]


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
async def test_dcache_word_read_hits_after_long_fill(dut):
    """ED=1: a word read from the same longword should hit after an initial long miss/fill."""
    h = CPUTestHarness(dut)
    data_addr = h.DATA_BASE + 0x300
    res = h.RESULT_BASE + 0x90

    program = [
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
        *imm_long(0x0000_0100),                    # ED=1
        *movec_dn_to_cr(0, CR_CACR),
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(data_addr),
        *move(LONG, AN_IND, 0, DN, 1),             # miss/fill longword
        *addq(LONG, 2, AN, 0),                     # A0 = data_addr + 2
        *move(WORD, AN_IND, 0, DN, 2),             # should hit same cache entry
        *move_to_abs_long(WORD, DN, 2, res),
        *h.sentinel_program(),
    ]

    h.mem.load_long(data_addr, 0x1122_3344)
    await h.setup(program)

    found = False
    prev_as_n = 1
    data_reads = 0
    for _ in range(26000):
        await RisingEdge(dut.CLK)
        try:
            as_n = int(dut.ASn.value)
            rw_n = int(dut.RWn.value)
            addr = int(dut.ADR_OUT.value)
        except ValueError:
            continue
        if prev_as_n == 1 and as_n == 0 and rw_n == 1 and addr in (data_addr, data_addr + 2):
            data_reads += 1
        prev_as_n = as_n
        if h.mem.read(h.SENTINEL_ADDR, 4) == h.SENTINEL_VAL:
            found = True
            break

    assert found, "D-cache word-after-long hit test did not complete"
    assert h.mem.read(res, 2) == 0x3344, "Word read from cached longword returned wrong value"
    assert data_reads == 1, f"Expected one external read (warm fill only), saw {data_reads}"
    h.cleanup()


@cocotb.test()
async def test_dcache_byte_write_hit_merges_cached_longword(dut):
    """ED=1: byte write hit should merge into cached longword and subsequent long read should hit updated data."""
    h = CPUTestHarness(dut)
    data_addr = h.DATA_BASE + 0x320
    res = h.RESULT_BASE + 0x94

    program = [
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
        *imm_long(0x0000_0100),                    # ED=1
        *movec_dn_to_cr(0, CR_CACR),
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(data_addr),
        *move(LONG, AN_IND, 0, DN, 1),             # miss/fill
        *moveq(0x11, 2),
        *addq(LONG, 3, AN, 0),                     # A0 = data_addr + 3
        *move(BYTE, DN, 2, AN_IND, 0),             # write hit, update low byte
        *subq(LONG, 3, AN, 0),                     # A0 = data_addr
        *move(LONG, AN_IND, 0, DN, 3),             # should hit updated entry
        *move_to_abs_long(LONG, DN, 3, res),
        *h.sentinel_program(),
    ]

    h.mem.load_long(data_addr, 0xAABB_CCDD)
    await h.setup(program)

    found = False
    prev_as_n = 1
    data_reads = 0
    for _ in range(28000):
        await RisingEdge(dut.CLK)
        try:
            as_n = int(dut.ASn.value)
            rw_n = int(dut.RWn.value)
            addr = int(dut.ADR_OUT.value)
        except ValueError:
            continue
        if prev_as_n == 1 and as_n == 0 and rw_n == 1 and addr == data_addr:
            data_reads += 1
        prev_as_n = as_n
        if h.mem.read(h.SENTINEL_ADDR, 4) == h.SENTINEL_VAL:
            found = True
            break

    assert found, "D-cache byte write-hit merge test did not complete"
    assert h.mem.read(res, 4) == 0xAABB_CC11, "Byte write-hit did not merge into cached longword correctly"
    assert data_reads == 1, f"Expected one external read (warm fill only), saw {data_reads}"
    h.cleanup()


@cocotb.test()
async def test_dcache_fd_freezes_new_fills(dut):
    """ED=1,FD=1: existing hits remain usable, but new-line misses should not allocate/fill."""
    h = CPUTestHarness(dut)
    data_hit_addr = h.DATA_BASE + 0x340
    data_miss_addr = h.DATA_BASE + 0x380
    res = h.RESULT_BASE + 0x98

    program = [
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
        *imm_long(0x0000_0100),                    # ED=1
        *movec_dn_to_cr(0, CR_CACR),
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(data_hit_addr),
        *move(LONG, AN_IND, 0, DN, 1),             # warm fill
        *move(LONG, SPECIAL, IMMEDIATE, DN, 2),
        *imm_long(0x0000_0300),                    # FD=1, ED=1
        *movec_dn_to_cr(2, CR_CACR),
        *move(LONG, AN_IND, 0, DN, 3),             # hit should still work under FD
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(data_miss_addr),
        *move(LONG, AN_IND, 0, DN, 4),             # miss under FD (no fill)
        *move(LONG, AN_IND, 0, DN, 5),             # should miss again (still no fill)
        *move_to_abs_long(LONG, DN, 5, res),
        *h.sentinel_program(),
    ]

    h.mem.load_long(data_hit_addr, 0x5566_7788)
    h.mem.load_long(data_miss_addr, 0x99AA_BBCC)
    await h.setup(program)

    found = False
    prev_as_n = 1
    miss_reads = 0
    for _ in range(32000):
        await RisingEdge(dut.CLK)
        try:
            as_n = int(dut.ASn.value)
            rw_n = int(dut.RWn.value)
            addr = int(dut.ADR_OUT.value)
        except ValueError:
            continue
        if prev_as_n == 1 and as_n == 0 and rw_n == 1 and addr == data_miss_addr:
            miss_reads += 1
        prev_as_n = as_n
        if h.mem.read(h.SENTINEL_ADDR, 4) == h.SENTINEL_VAL:
            found = True
            break

    assert found, "D-cache FD freeze test did not complete"
    assert h.mem.read(res, 4) == 0x99AA_BBCC, "FD freeze test returned wrong data value"
    assert miss_reads == 2, f"Expected two external miss reads with FD freeze, saw {miss_reads}"
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


@cocotb.test()
async def test_ciout_reflects_tt_ci_data_cycles(dut):
    """CIOUTn should assert for TT CI-matched supervisor-data cycles and stay negated on program fetches."""
    h = CPUTestHarness(dut)
    tt_addr = h.DATA_BASE + 0x240
    data_addr = h.DATA_BASE + 0x200
    res = h.RESULT_BASE + 0x78

    # E=1, CI=1, RWM=1, FC base=supervisor data, FC mask=000.
    tt0_ci_super_data = 0x00FF_8750

    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(tt_addr),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
        *imm_long(tt0_ci_super_data),
        *move(LONG, DN, 0, AN_IND, 0),             # (A0) = TT0 value
        *pmove_an_to_tt0(0),                       # PMOVE (A0),TT0

        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(data_addr),
        *move(LONG, AN_IND, 0, DN, 1),             # supervisor data read
        *move_to_abs_long(LONG, DN, 1, res),       # supervisor data write
        *h.sentinel_program(),
    ]

    h.mem.load_long(data_addr, 0x1020_3040)
    await h.setup(program)

    found = False
    prev_as_n = 1
    saw_prog_fetch = False
    prog_fetch_ciout_high = True
    saw_data_read = False
    data_read_ciout_low = True
    saw_result_write = False
    result_write_ciout_low = True

    for _ in range(28000):
        await RisingEdge(dut.CLK)
        try:
            as_n = int(dut.ASn.value)
            rw_n = int(dut.RWn.value)
            addr = int(dut.ADR_OUT.value)
            ciout_n = int(dut.CIOUTn.value)
        except ValueError:
            continue

        if prev_as_n == 1 and as_n == 0:
            if rw_n == 1 and addr < h.DATA_BASE:
                saw_prog_fetch = True
                prog_fetch_ciout_high &= (ciout_n == 1)
            if rw_n == 1 and addr == data_addr:
                saw_data_read = True
                data_read_ciout_low &= (ciout_n == 0)
            if rw_n == 0 and addr == res:
                saw_result_write = True
                result_write_ciout_low &= (ciout_n == 0)

        prev_as_n = as_n
        if h.mem.read(h.SENTINEL_ADDR, 4) == h.SENTINEL_VAL:
            found = True
            break

    assert found, "CIOUT TT-CI data-cycle test did not complete"
    assert h.mem.read(res, 4) == 0x1020_3040, "Result writeback mismatch in CIOUT test"
    assert saw_prog_fetch, "Did not observe any program fetch cycles"
    assert prog_fetch_ciout_high, "CIOUTn asserted unexpectedly on program fetch cycle(s)"
    assert saw_data_read, "Did not observe TT-CI-matched data read cycle"
    assert data_read_ciout_low, "CIOUTn not asserted on TT-CI-matched data read cycle(s)"
    assert saw_result_write, "Did not observe TT-CI-matched data write cycle"
    assert result_write_ciout_low, "CIOUTn not asserted on TT-CI-matched data write cycle(s)"
    h.cleanup()


@cocotb.test()
async def test_ciout_ors_tt0_tt1_ci_bits(dut):
    """If TT0 and TT1 both match, CIOUTn should assert when either CI bit is set."""
    h = CPUTestHarness(dut)
    tt0_addr = h.DATA_BASE + 0x260
    tt1_addr = h.DATA_BASE + 0x264
    data_addr = h.DATA_BASE + 0x220
    res = h.RESULT_BASE + 0x7C

    # Both entries match supervisor-data accesses; only TT1 has CI=1.
    tt0_no_ci_super_data = 0x00FF_8350
    tt1_ci_super_data = 0x00FF_8750

    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(tt0_addr),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
        *imm_long(tt0_no_ci_super_data),
        *move(LONG, DN, 0, AN_IND, 0),
        *pmove_an_to_tt0(0),

        *movea(LONG, SPECIAL, IMMEDIATE, 1),
        *imm_long(tt1_addr),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 1),
        *imm_long(tt1_ci_super_data),
        *move(LONG, DN, 1, AN_IND, 1),
        *pmove_an_to_tt1(1),

        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(data_addr),
        *move(LONG, AN_IND, 0, DN, 2),
        *move_to_abs_long(LONG, DN, 2, res),
        *h.sentinel_program(),
    ]

    h.mem.load_long(data_addr, 0x5566_7788)
    await h.setup(program)

    found = False
    prev_as_n = 1
    saw_data_read = False
    data_read_ciout_low = True

    for _ in range(30000):
        await RisingEdge(dut.CLK)
        try:
            as_n = int(dut.ASn.value)
            rw_n = int(dut.RWn.value)
            addr = int(dut.ADR_OUT.value)
            ciout_n = int(dut.CIOUTn.value)
        except ValueError:
            continue

        if prev_as_n == 1 and as_n == 0 and rw_n == 1 and addr == data_addr:
            saw_data_read = True
            data_read_ciout_low &= (ciout_n == 0)

        prev_as_n = as_n
        if h.mem.read(h.SENTINEL_ADDR, 4) == h.SENTINEL_VAL:
            found = True
            break

    assert found, "CIOUT TT0/TT1 OR test did not complete"
    assert h.mem.read(res, 4) == 0x5566_7788, "TT0/TT1 OR test result mismatch"
    assert saw_data_read, "Did not observe TT0/TT1-matched data read cycle"
    assert data_read_ciout_low, "CIOUTn not asserted when TT1 CI bit should force ORed CI output"
    h.cleanup()


@cocotb.test()
async def test_cbreq_asserts_on_dcache_read_miss_with_dbe(dut):
    """ED=1+DBE=1: aligned long read miss should assert CBREQn on the external data-read cycle."""
    h = CPUTestHarness(dut)
    data_addr = h.DATA_BASE + 0x280
    res = h.RESULT_BASE + 0x80

    program = [
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
        *imm_long(0x0000_1100),                    # DBE=1, ED=1
        *movec_dn_to_cr(0, CR_CACR),
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(data_addr),
        *move(LONG, AN_IND, 0, DN, 1),             # aligned long read miss
        *move_to_abs_long(LONG, DN, 1, res),
        *h.sentinel_program(),
    ]

    h.mem.load_long(data_addr, 0xA0B1_C2D3)
    await h.setup(program)

    found = False
    prev_as_n = 1
    saw_data_read = False
    saw_cbreq_low = False

    for _ in range(28000):
        await RisingEdge(dut.CLK)
        try:
            as_n = int(dut.ASn.value)
            rw_n = int(dut.RWn.value)
            addr = int(dut.ADR_OUT.value)
            cbreq_n = int(dut.CBREQn.value)
        except ValueError:
            continue

        if prev_as_n == 1 and as_n == 0 and rw_n == 1 and addr == data_addr:
            saw_data_read = True
            if cbreq_n == 0:
                saw_cbreq_low = True

        prev_as_n = as_n
        if h.mem.read(h.SENTINEL_ADDR, 4) == h.SENTINEL_VAL:
            found = True
            break

    assert found, "CBREQ dcache read-miss test did not complete"
    assert h.mem.read(res, 4) == 0xA0B1_C2D3, "CBREQ dcache read-miss data mismatch"
    assert saw_data_read, "Did not observe target data-read cycle"
    assert saw_cbreq_low, "Expected CBREQn assertion on burst-eligible dcache read miss"
    h.cleanup()


@cocotb.test()
async def test_cbreq_suppressed_on_tt_ci_noncacheable_read(dut):
    """TT CI=1 should keep CBREQn negated on matching data reads (noncacheable access)."""
    h = CPUTestHarness(dut)
    tt_addr = h.DATA_BASE + 0x2A0
    data_addr = h.DATA_BASE + 0x2C0
    res = h.RESULT_BASE + 0x84

    # E=1, CI=1, RWM=1, FC base=supervisor data, FC mask=000.
    tt0_ci_super_data = 0x00FF_8750

    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(tt_addr),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
        *imm_long(tt0_ci_super_data),
        *move(LONG, DN, 0, AN_IND, 0),
        *pmove_an_to_tt0(0),

        *move(LONG, SPECIAL, IMMEDIATE, DN, 1),
        *imm_long(0x0000_1100),                    # DBE=1, ED=1
        *movec_dn_to_cr(1, CR_CACR),

        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(data_addr),
        *move(LONG, AN_IND, 0, DN, 2),
        *move_to_abs_long(LONG, DN, 2, res),
        *h.sentinel_program(),
    ]

    h.mem.load_long(data_addr, 0x1357_9BDF)
    await h.setup(program)

    found = False
    prev_as_n = 1
    saw_data_read = False
    cbreq_always_high = True
    saw_ciout_low = False

    for _ in range(32000):
        await RisingEdge(dut.CLK)
        try:
            as_n = int(dut.ASn.value)
            rw_n = int(dut.RWn.value)
            addr = int(dut.ADR_OUT.value)
            cbreq_n = int(dut.CBREQn.value)
            ciout_n = int(dut.CIOUTn.value)
        except ValueError:
            continue

        if prev_as_n == 1 and as_n == 0 and rw_n == 1 and addr == data_addr:
            saw_data_read = True
            cbreq_always_high &= (cbreq_n == 1)
            if ciout_n == 0:
                saw_ciout_low = True

        prev_as_n = as_n
        if h.mem.read(h.SENTINEL_ADDR, 4) == h.SENTINEL_VAL:
            found = True
            break

    assert found, "CBREQ TT-CI suppression test did not complete"
    assert h.mem.read(res, 4) == 0x1357_9BDF, "CBREQ TT-CI suppression data mismatch"
    assert saw_data_read, "Did not observe TT-CI-matched data read cycle"
    assert cbreq_always_high, "CBREQn asserted unexpectedly for TT-CI noncacheable read"
    assert saw_ciout_low, "Expected CIOUTn assertion on TT-CI noncacheable read"
    h.cleanup()


@cocotb.test()
async def test_icache_burst_tracking_suppresses_redundant_cbreq(dut):
    """With EI+IBE and burst ack, only the first miss in a line should request burst intent."""
    h = CPUTestHarness(dut)
    res = h.RESULT_BASE + 0x88
    line_base = 0x000120
    line_addrs = {line_base + 2 * i for i in range(8)}

    program = [
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
        *imm_long(0x0000_0011),                    # EI=1, IBE=1
        *movec_dn_to_cr(0, CR_CACR),
        # Pad from 0x010A to 0x0120.
        *nop(), *nop(), *nop(), *nop(), *nop(), *nop(),
        *nop(), *nop(), *nop(), *nop(), *nop(),
        # Exactly one I-cache line (16 bytes, 8 words) of NOPs.
        *nop(), *nop(), *nop(), *nop(),
        *nop(), *nop(), *nop(), *nop(),
        *moveq(0x5A, 1),
        *move_to_abs_long(LONG, DN, 1, res),
        *h.sentinel_program(),
    ]

    await h.setup(program)
    dut.CBACKn.value = 1  # Assert burst acknowledge only after first request intent.

    found = False
    prev_as_n = 1
    line_reads = 0
    cbreq_inst_req_on_line = 0
    burst_acked = False

    for _ in range(32000):
        await RisingEdge(dut.CLK)
        try:
            as_n = int(dut.ASn.value)
            rw_n = int(dut.RWn.value)
            addr = int(dut.ADR_OUT.value)
            adr_p_phys = int(dut.ADR_P_PHYS.value)
            cbreq_inst_req_now = int(dut.CBREQ_INST_REQ_NOW.value)
        except ValueError:
            continue

        if cbreq_inst_req_now and ((adr_p_phys >> 4) == (line_base >> 4)):
            cbreq_inst_req_on_line += 1
            if not burst_acked:
                # Keep CBACKn low once first request is observed so first fill records burst-track context.
                dut.CBACKn.value = 0
                burst_acked = True

        if prev_as_n == 1 and as_n == 0 and rw_n == 1 and addr in line_addrs:
            line_reads += 1

        prev_as_n = as_n
        if h.mem.read(h.SENTINEL_ADDR, 4) == h.SENTINEL_VAL:
            found = True
            break

    assert found, "I-cache burst tracking test did not complete"
    assert h.mem.read(res, 4) == 0x0000_005A, "I-cache burst tracking result mismatch"
    assert line_reads >= 4, f"Expected multiple line reads, saw {line_reads}"
    assert burst_acked, "Did not observe initial burst request intent for target I-cache line"
    assert cbreq_inst_req_on_line == 1, (
        f"Expected one burst request intent for burst-tracked I-cache line, saw {cbreq_inst_req_on_line}"
    )
    h.cleanup()


@cocotb.test()
async def test_icache_phase8_autonomous_line_completion(dut):
    """Phase-8: burst-acknowledged I-cache miss should complete untouched line words in background."""
    h = CPUTestHarness(dut)
    line_base = 0x000120
    line_idx = (line_base >> 4) & 0xF
    line_tag = line_base >> 8

    program = [
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
        *imm_long(0x0000_0011),                    # EI=1, IBE=1
        *movec_dn_to_cr(0, CR_CACR),
        *bra(0x14),                                # 0x010C + 0x14 = 0x0120
        # Pad from 0x010C to 0x0120.
        *nop(), *nop(), *nop(), *nop(), *nop(),
        *nop(), *nop(), *nop(), *nop(), *nop(),
        *bra(0x3E),                                # 0x0122 + 0x3E = 0x0160
        # Fill remainder from 0x0122 to 0x0160.
        *nop(), *nop(), *nop(), *nop(), *nop(), *nop(), *nop(), *nop(),
        *nop(), *nop(), *nop(), *nop(), *nop(), *nop(), *nop(), *nop(),
        *nop(), *nop(), *nop(), *nop(), *nop(), *nop(), *nop(), *nop(),
        *nop(), *nop(), *nop(), *nop(), *nop(), *nop(), *nop(),
        *bra(-2),                                  # Park core while background completion runs.
    ]

    await h.setup(program)
    dut.CBACKn.value = 1

    burst_acked = False
    cback_released = False
    saw_full_line = False
    for _ in range(120000):
        await RisingEdge(dut.CLK)
        try:
            icache_mask = int(dut.ICACHE_VALID[line_idx].value)
            icache_tag = int(dut.ICACHE_TAG[line_idx].value)
            adr_p_phys = int(dut.ADR_P_PHYS.value)
            cbreq_inst_req_now = int(dut.CBREQ_INST_REQ_NOW.value)
            icache_burst_fill_valid = int(dut.ICACHE_BURST_FILL_VALID.value)
        except (ValueError, TypeError, IndexError):
            continue
        if (not burst_acked and
            cbreq_inst_req_now and
            ((adr_p_phys >> 4) & 0xF) == line_idx and
            (adr_p_phys >> 8) == line_tag):
            # Acknowledge the first burst request for the target line only.
            dut.CBACKn.value = 0
            burst_acked = True
        elif burst_acked and not cback_released and icache_burst_fill_valid:
            # Keep acknowledgment scoped to the initial line-start event.
            dut.CBACKn.value = 1
            cback_released = True
        if icache_tag == line_tag and icache_mask == 0xFF:
            saw_full_line = True
            break

    assert burst_acked, "Did not observe target-line burst request intent for I-cache phase-8 test"
    assert saw_full_line, "Expected full I-cache line validity after burst-acknowledged miss"
    h.cleanup()


@cocotb.test()
async def test_dcache_phase8_autonomous_line_completion(dut):
    """Phase-8: burst-acknowledged D-cache miss should complete untouched entries in the same line."""
    h = CPUTestHarness(dut)
    data_base = h.DATA_BASE + 0x340
    line_idx = (data_base >> 4) & 0xF

    program = [
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
        *imm_long(0x0000_1100),                    # ED=1, DBE=1
        *movec_dn_to_cr(0, CR_CACR),
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(data_base),
        *move(LONG, AN_IND, 0, DN, 1),             # First miss starts burst line completion.
        *bra(-2),                                  # Park core; remaining line entries should fill autonomously.
    ]

    h.mem.load_long(data_base + 0x0, 0xABC0_0000)
    h.mem.load_long(data_base + 0x4, 0xABC0_0001)
    h.mem.load_long(data_base + 0x8, 0xABC0_0002)
    h.mem.load_long(data_base + 0xC, 0xABC0_0003)
    await h.setup(program)
    dut.CBACKn.value = 0

    saw_full_line = False
    for _ in range(90000):
        await RisingEdge(dut.CLK)
        try:
            dcache_mask = int(dut.DCACHE_VALID[line_idx].value)
        except (ValueError, TypeError, IndexError):
            dcache_mask = 0
        if dcache_mask == 0xF:
            saw_full_line = True
            break

    assert saw_full_line, "Expected full D-cache line validity after burst-acknowledged miss"
    h.cleanup()


@cocotb.test()
async def test_dcache_miss_bus_error_does_not_allocate_line(dut):
    """A data-read miss completed with bus error must not allocate/update D-cache state."""
    h = CPUTestHarness(dut)
    res = h.RESULT_BASE + 0xA0
    data_addr = h.DATA_BASE + 0x500
    line_idx = (data_addr >> 4) & 0xF
    entry_idx = (data_addr >> 2) & 0x3
    handler_addr = 0x000740

    program = [
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
        *imm_long(0x0000_0100),                    # ED=1
        *movec_dn_to_cr(0, CR_CACR),
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(data_addr),
        *move(LONG, AN_IND, 0, DN, 1),             # Miss read -> injected bus error.
        *moveq(0x66, 2),                           # Fallback marker (no bus error).
        *move_to_abs_long(LONG, DN, 2, res),
        *h.sentinel_program(),
    ]

    await h.setup(program)
    h.mem.load_long(2 * 4, handler_addr)  # Bus error vector.
    h.mem.load_words(
        handler_addr,
        [
            *moveq(0x02, 2),
            *move_to_abs_long(LONG, DN, 2, res),
            *h.sentinel_program(),
        ],
    )

    fault_injected = False
    prev_as_n = 1
    found = False
    for _ in range(30000):
        await RisingEdge(dut.CLK)
        try:
            as_n = int(dut.ASn.value)
            rw_n = int(dut.RWn.value)
            addr = int(dut.ADR_OUT.value)
        except ValueError:
            continue

        if (not fault_injected and
            prev_as_n == 1 and as_n == 0 and rw_n == 1 and addr == data_addr):
            dut.BERRn.value = 0
            fault_injected = True
            await RisingEdge(dut.CLK)
            await RisingEdge(dut.CLK)
            dut.BERRn.value = 1

        prev_as_n = as_n
        if h.mem.read(h.SENTINEL_ADDR, 4) == h.SENTINEL_VAL:
            found = True
            break

    assert fault_injected, "Did not inject the target data-read bus error"
    assert found, "D-cache bus-error miss test did not reach sentinel"
    assert h.mem.read(res, 4) == 0x0000_0002, "Expected bus-error handler marker"

    dcache_mask = int(dut.DCACHE_VALID[line_idx].value)
    assert ((dcache_mask >> entry_idx) & 0x1) == 0, (
        "Bus-error read miss must not allocate/update D-cache entry"
    )
    h.cleanup()


@cocotb.test()
async def test_icache_miss_bus_error_does_not_allocate_word(dut):
    """An opcode-miss bus error must not mark the fetched I-cache word valid."""
    h = CPUTestHarness(dut)
    res = h.RESULT_BASE + 0xA4
    fault_fetch_addr = 0x000180
    line_idx = (fault_fetch_addr >> 4) & 0xF
    word_idx = (fault_fetch_addr >> 1) & 0x7
    line_tag = fault_fetch_addr >> 8
    handler_addr = 0x0007A0

    program = [
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
        *imm_long(0x0000_0001),                    # EI=1
        *movec_dn_to_cr(0, CR_CACR),
        *bra(0x72),                                # 0x010C + 0x72 = 0x0180
    ]
    while (h.PROGRAM_BASE + len(program) * 2) < fault_fetch_addr:
        program.extend(nop())
    program.extend(
        [
            *moveq(0x66, 2),                       # Fallback marker (no bus error).
            *move_to_abs_long(LONG, DN, 2, res),
            *h.sentinel_program(),
        ]
    )

    await h.setup(program)
    h.mem.load_long(2 * 4, handler_addr)  # Bus error vector.
    h.mem.load_words(
        handler_addr,
        [
            *moveq(0x02, 2),
            *move_to_abs_long(LONG, DN, 2, res),
            *h.sentinel_program(),
        ],
    )

    fault_injected = False
    prev_as_n = 1
    found = False
    for _ in range(30000):
        await RisingEdge(dut.CLK)
        try:
            as_n = int(dut.ASn.value)
            rw_n = int(dut.RWn.value)
            addr = int(dut.ADR_OUT.value)
        except ValueError:
            continue

        if (not fault_injected and
            prev_as_n == 1 and as_n == 0 and rw_n == 1 and addr == fault_fetch_addr):
            dut.BERRn.value = 0
            fault_injected = True
            await RisingEdge(dut.CLK)
            await RisingEdge(dut.CLK)
            dut.BERRn.value = 1

        prev_as_n = as_n
        if h.mem.read(h.SENTINEL_ADDR, 4) == h.SENTINEL_VAL:
            found = True
            break

    assert fault_injected, "Did not inject the target opcode-fetch bus error"
    assert found, "I-cache bus-error miss test did not reach sentinel"
    assert h.mem.read(res, 4) == 0x0000_0002, "Expected bus-error handler marker"

    icache_mask = int(dut.ICACHE_VALID[line_idx].value)
    icache_tag = int(dut.ICACHE_TAG[line_idx].value)
    word_valid = (icache_mask >> word_idx) & 0x1
    assert not (icache_tag == line_tag and word_valid == 1), (
        "Opcode bus-error miss must not mark target I-cache word valid"
    )
    h.cleanup()


@cocotb.test()
async def test_icache_phase8_wrap_order_from_midline_miss(dut):
    """I-cache burst selector should choose the first pending word in wrap order."""
    h = CPUTestHarness(dut)
    line_base = 0x000120

    program = [
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
        *imm_long(0x0000_0011),                    # EI=1, IBE=1
        *movec_dn_to_cr(0, CR_CACR),
        *bra(0x1C),                                # 0x010C + 0x1C = 0x0128
        *nop(), *nop(), *nop(), *nop(), *nop(), *nop(),
        *nop(), *nop(), *nop(), *nop(), *nop(), *nop(), *nop(),
        *bra(0x34),                                # 0x0128 -> 0x015E (fetches at 0x012A)
        *nop(), *nop(), *nop(), *nop(), *nop(), *nop(), *nop(), *nop(),
        *nop(), *nop(), *nop(), *nop(), *nop(), *nop(), *nop(), *nop(),
        *nop(), *nop(), *nop(), *nop(), *nop(), *nop(), *nop(), *nop(),
        *nop(), *nop(),
        *bra(-2),                                  # park
    ]

    await h.setup(program)
    dut.CBACKn.value = 1

    burst_acked = False
    observed_words = []
    burst_fill_seen = False
    for _ in range(120000):
        await RisingEdge(dut.CLK)
        try:
            cbreq_inst_req_now = int(dut.CBREQ_INST_REQ_NOW.value)
            adr_p_phys = int(dut.ADR_P_PHYS.value)
            bus_bsy = int(dut.BUS_BSY.value)
            burst_fill_valid = int(dut.ICACHE_BURST_FILL_VALID.value)
            burst_fill_pending = int(dut.ICACHE_BURST_FILL_PENDING.value)
            burst_fill_next = int(dut.ICACHE_BURST_FILL_NEXT_WORD.value)
            burst_prefetch_op_req = int(dut.BURST_PREFETCH_OP_REQ.value)
            burst_prefetch_op_word = int(dut.BURST_PREFETCH_OP_WORD.value)
            burst_prefetch_addr = int(dut.BURST_PREFETCH_ADDR.value)
        except ValueError:
            continue

        if (not burst_acked and
            cbreq_inst_req_now and
            (adr_p_phys & 0xFFFFFFF0) == line_base):
            dut.CBACKn.value = 0
            burst_acked = True

        if burst_acked and burst_fill_valid:
            burst_fill_seen = True

        if (burst_fill_seen and burst_fill_valid and burst_fill_pending != 0 and
            not bus_bsy and burst_prefetch_op_req and
            (burst_prefetch_addr & 0xFFFFFFF0) == line_base):
            expected_word = None
            for offset in range(8):
                word = (burst_fill_next + offset) & 0x7
                if (burst_fill_pending >> word) & 0x1:
                    expected_word = word
                    break
            assert expected_word is not None, "Expected pending I-cache burst word not found"
            assert burst_prefetch_op_word == expected_word, (
                f"I-cache burst selector mismatch: expected word {expected_word}, "
                f"got {burst_prefetch_op_word}, pending=0x{burst_fill_pending:02x}, next={burst_fill_next}"
            )
            if burst_prefetch_op_word not in observed_words:
                observed_words.append(burst_prefetch_op_word)

        if burst_fill_seen and not burst_fill_valid:
            break

    assert burst_acked, "Did not observe burst-acknowledged mid-line miss"
    assert burst_fill_seen, "I-cache burst-fill context never became active"
    assert observed_words, "Did not observe any I-cache background burst selection"
    h.cleanup()


@cocotb.test()
async def test_dcache_phase8_wrap_order_from_midline_miss(dut):
    """D-cache burst selector should choose the first pending entry in wrap order."""
    h = CPUTestHarness(dut)
    line_base = h.DATA_BASE + 0x3C0
    miss_addr = line_base + 0x8   # entry index 2

    program = [
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
        *imm_long(0x0000_1100),                    # ED=1, DBE=1
        *movec_dn_to_cr(0, CR_CACR),
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(miss_addr),
        *move(LONG, AN_IND, 0, DN, 1),             # first miss entry=2
        *bra(-2),                                  # park
    ]

    h.mem.load_long(line_base + 0x0, 0xABC0_0100)
    h.mem.load_long(line_base + 0x4, 0xABC0_0101)
    h.mem.load_long(line_base + 0x8, 0xABC0_0102)
    h.mem.load_long(line_base + 0xC, 0xABC0_0103)
    await h.setup(program)
    dut.CBACKn.value = 0

    bg_order_entries = []
    burst_fill_seen = False
    for _ in range(90000):
        await RisingEdge(dut.CLK)
        try:
            bus_bsy = int(dut.BUS_BSY.value)
            burst_fill_valid = int(dut.DCACHE_BURST_FILL_VALID.value)
            burst_fill_pending = int(dut.DCACHE_BURST_FILL_PENDING.value)
            burst_fill_next = int(dut.DCACHE_BURST_FILL_NEXT_ENTRY.value)
            burst_prefetch_data_req = int(dut.BURST_PREFETCH_DATA_REQ.value)
            burst_prefetch_data_entry = int(dut.BURST_PREFETCH_DATA_ENTRY.value)
            burst_prefetch_addr = int(dut.BURST_PREFETCH_ADDR.value)
        except ValueError:
            continue

        if burst_fill_valid:
            burst_fill_seen = True

        if (burst_fill_seen and burst_fill_valid and burst_fill_pending != 0 and
            not bus_bsy and burst_prefetch_data_req and
            (burst_prefetch_addr & 0xFFFFFFF0) == line_base):
            expected_entry = None
            for offset in range(4):
                entry = (burst_fill_next + offset) & 0x3
                if (burst_fill_pending >> entry) & 0x1:
                    expected_entry = entry
                    break
            assert expected_entry is not None, "Expected pending D-cache burst entry not found"
            assert burst_prefetch_data_entry == expected_entry, (
                f"D-cache burst selector mismatch: expected entry {expected_entry}, "
                f"got {burst_prefetch_data_entry}, pending=0x{burst_fill_pending:x}, next={burst_fill_next}"
            )
            if burst_prefetch_data_entry != ((miss_addr >> 2) & 0x3) and burst_prefetch_data_entry not in bg_order_entries:
                bg_order_entries.append(burst_prefetch_data_entry)

        if burst_fill_seen and not burst_fill_valid:
            break

    assert burst_fill_seen, "D-cache burst-fill context never became active"
    assert bg_order_entries, "Did not observe any D-cache background burst selection"
    h.cleanup()
