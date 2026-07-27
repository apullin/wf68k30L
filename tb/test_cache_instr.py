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
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles

from bus_model import BusModel
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
    cas,
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


def cache_state(dut):
    """Tag/valid arrays live inside the extracted cache-state submodule."""
    return dut.I_TOP_CACHE_STATE


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
async def test_dcache_subword_reads_hit_at_every_offset(dut):
    """ED=1: every byte and word offset in a filled longword should hit with the right lane."""
    h = CPUTestHarness(dut)
    data_addr = h.DATA_BASE + 0x360
    res = h.RESULT_BASE + 0xB0

    program = [
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
        *imm_long(0x0000_0100),                    # ED=1
        *movec_dn_to_cr(0, CR_CACR),
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(data_addr),
        *move(LONG, AN_IND, 0, DN, 1),             # miss/fill the whole longword
        *move(BYTE, AN_IND, 0, DN, 2),             # byte at +0
        *addq(LONG, 1, AN, 0),
        *move(BYTE, AN_IND, 0, DN, 3),             # byte at +1
        *addq(LONG, 1, AN, 0),
        *move(BYTE, AN_IND, 0, DN, 4),             # byte at +2
        *addq(LONG, 1, AN, 0),
        *move(BYTE, AN_IND, 0, DN, 5),             # byte at +3
        *subq(LONG, 3, AN, 0),
        *move(WORD, AN_IND, 0, DN, 6),             # word at +0
        *addq(LONG, 2, AN, 0),
        *move(WORD, AN_IND, 0, DN, 0),             # word at +2 (D0 held 0x00000100)
        *move_to_abs_long(LONG, DN, 2, res + 0x00),
        *move_to_abs_long(LONG, DN, 3, res + 0x04),
        *move_to_abs_long(LONG, DN, 4, res + 0x08),
        *move_to_abs_long(LONG, DN, 5, res + 0x0C),
        *move_to_abs_long(LONG, DN, 6, res + 0x10),
        *move_to_abs_long(LONG, DN, 0, res + 0x14),
        *h.sentinel_program(),
    ]

    h.mem.load_long(data_addr, 0x12_34_56_78)
    await h.setup(program)

    found = False
    prev_as_n = 1
    line_addrs = {data_addr + i for i in range(4)}
    data_reads = 0
    for _ in range(30000):
        await RisingEdge(dut.CLK)
        try:
            as_n = int(dut.ASn.value)
            rw_n = int(dut.RWn.value)
            addr = int(dut.ADR_OUT.value)
        except ValueError:
            continue
        if prev_as_n == 1 and as_n == 0 and rw_n == 1 and addr in line_addrs:
            data_reads += 1
        prev_as_n = as_n
        if h.mem.read(h.SENTINEL_ADDR, 4) == h.SENTINEL_VAL:
            found = True
            break

    assert found, "D-cache subword offset test did not complete"
    # MOVE.B/MOVE.W only replace the low byte/word of the destination; D2-D6
    # start at zero and D0 held 0x00000100 from the CACR setup.
    expected = [
        (0x00, 0x0000_0012, "byte +0"),
        (0x04, 0x0000_0034, "byte +1"),
        (0x08, 0x0000_0056, "byte +2"),
        (0x0C, 0x0000_0078, "byte +3"),
        (0x10, 0x0000_1234, "word +0"),
        (0x14, 0x0000_5678, "word +2"),
    ]
    for offset, want, what in expected:
        got = h.mem.read(res + offset, 4)
        assert got == want, f"{what} read wrong lane: got 0x{got:08X}, expected 0x{want:08X}"
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
async def test_dcache_misaligned_write_invalidates_overlapping_entry(dut):
    """ED=1: a misaligned long write must not leave a stale hit on the entry it overlaps."""
    h = CPUTestHarness(dut)
    data_addr = h.DATA_BASE + 0x400        # Aligned entry that gets cached.
    write_addr = data_addr - 2             # Misaligned long spanning into data_addr.
    res = h.RESULT_BASE + 0x68

    program = [
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
        *imm_long(0x0000_0100),                    # ED=1
        *movec_dn_to_cr(0, CR_CACR),
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(data_addr),
        *move(LONG, AN_IND, 0, DN, 1),             # miss/fill entry for data_addr
        *move(LONG, SPECIAL, IMMEDIATE, DN, 2),
        *imm_long(0x1122_3344),
        *move_to_abs_long(LONG, DN, 2, write_addr),  # misaligned write over data_addr[0:1]
        *move(LONG, AN_IND, 0, DN, 3),             # must not be served from the stale entry
        *move_to_abs_long(LONG, DN, 3, res),
        *h.sentinel_program(),
    ]

    h.mem.load_long(write_addr, 0x0000_0000)
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
        except ValueError:
            continue
        if prev_as_n == 1 and as_n == 0 and rw_n == 1 and int(dut.ADR_OUT.value) == data_addr:
            data_reads += 1
        prev_as_n = as_n
        if h.mem.read(h.SENTINEL_ADDR, 4) == h.SENTINEL_VAL:
            found = True
            break

    assert found, "D-cache misaligned-write invalidation test did not complete"
    # The write put 0x33,0x44 over the first two bytes of the cached longword.
    assert h.mem.read(data_addr, 4) == 0x3344_CCDD, (
        f"Memory not updated as expected, got 0x{h.mem.read(data_addr, 4):08X}"
    )
    assert h.mem.read(res, 4) == 0x3344_CCDD, (
        f"Post-write read returned pre-write bytes, got 0x{h.mem.read(res, 4):08X}"
    )
    assert data_reads == 2, (
        f"Expected the overlapped entry to be invalidated and refetched, saw {data_reads} reads"
    )
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
    """Only the first miss in an I-cache line may request a burst.

    UM 6.1.3.2 lists the two conditions that raise a burst request: "A read
    cycle for either the instruction or data cache misses due to the indexed tag
    not matching" or "A read cycle tag matches, but all long words in the line
    are invalid". Once a line has been burst-acknowledged neither holds again, so
    a second request for the same line/tag must not appear -- and with the burst
    actually filling the line, there is no second miss in it to ask.
    """
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

    bus = BusModel(dut, h.mem, sync_term=True, cback=True)
    await _bringup(dut, h, program, bus)

    found = False
    cbreq_inst_req_on_line = 0
    for _ in range(32000):
        await RisingEdge(dut.CLK)
        try:
            adr_p_phys = int(dut.ADR_P_PHYS.value)
            cbreq_inst_req_now = int(dut.CBREQ_INST_REQ_NOW.value)
        except ValueError:
            continue

        if cbreq_inst_req_now and ((adr_p_phys >> 4) == (line_base >> 4)):
            cbreq_inst_req_on_line += 1

        if h.mem.read(h.SENTINEL_ADDR, 4) == h.SENTINEL_VAL:
            found = True
            break

    assert found, "I-cache burst tracking test did not complete"
    h.check_no_unexpected_exception()
    assert h.mem.read(res, 4) == 0x0000_005A, "I-cache burst tracking result mismatch"
    assert cbreq_inst_req_on_line == 1, (
        f"Expected one burst request for the burst-filled I-cache line, saw "
        f"{cbreq_inst_req_on_line}"
    )
    line_reads = [addr for addr, _c, is_write in bus.sub_cycles
                  if not is_write and addr in line_addrs]
    assert len(line_reads) == 1, (
        f"the burst-filled line was fetched from the bus more than once: "
        f"{[hex(a) for a in line_reads]}"
    )
    h.cleanup()


@cocotb.test()
async def test_icache_burst_fills_the_whole_line_under_one_as(dut):
    """One burst-acknowledged I-cache miss fills all eight words of the line.

    UM 6.1.3.2: "The bursting mechanism allows addresses to wrap around so that
    the entire four long words in the cache line can be filled in a single burst
    operation, regardless of the initial address and operand alignment", and "The
    MC68030 holds the entire address bus constant for the duration of the burst
    cycle". So the line goes fully valid off one AS cycle, and the two
    instruction words of each long word both become valid even though the fetch
    that asked for the line was word sized.
    """
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
        *bra(-2),                                  # Park the core once the line is behind us.
    ]

    bus = BusModel(dut, h.mem, sync_term=True, cback=True)
    await _bringup(dut, h, program, bus)

    saw_full_line = False
    for _ in range(20000):
        await RisingEdge(dut.CLK)
        try:
            icache_mask = int(cache_state(dut).ICACHE_VALID[line_idx].value)
            icache_tag = int(cache_state(dut).ICACHE_TAG[line_idx].value)
        except (ValueError, TypeError, IndexError):
            continue
        if icache_tag == line_tag and icache_mask == 0xFF:
            saw_full_line = True
            break

    assert saw_full_line, (
        "the burst did not fill all eight instruction words of the line"
    )
    assert _line_read_cycles(bus, line_base) == [line_base], (
        f"the line was filled with more than the one burst cycle: "
        f"{[hex(a) for a in _line_read_cycles(bus, line_base)]}"
    )
    h.cleanup()


@cocotb.test()
async def test_dcache_burst_fills_the_whole_line_under_one_as(dut):
    """One burst-acknowledged D-cache miss fills all four entries of the line.

    Same clause as the I-cache case, on the data side: the device supplies four
    long words with the address held, so the line is complete without any further
    bus cycle for it.
    """
    h = CPUTestHarness(dut)
    data_base = h.DATA_BASE + 0x340
    line_idx = (data_base >> 4) & 0xF
    res = h.RESULT_BASE + 0x8C

    program = [
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
        *imm_long(0x0000_1100),                    # ED=1, DBE=1
        *movec_dn_to_cr(0, CR_CACR),
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(data_base),
        *move(LONG, AN_IND, 0, DN, 1),             # Aligned long miss: bursts the line.
        *move_to_abs_long(LONG, DN, 1, res),
        *h.sentinel_program(),
    ]

    for i in range(4):
        h.mem.load_long(data_base + 4 * i, 0xABC0_0000 | i)

    bus = BusModel(dut, h.mem, sync_term=True, cback=True)
    await _bringup(dut, h, program, bus)

    found = False
    saw_full_line = False
    for _ in range(20000):
        await RisingEdge(dut.CLK)
        try:
            if int(cache_state(dut).DCACHE_VALID[line_idx].value) == 0xF:
                saw_full_line = True
        except (ValueError, TypeError, IndexError):
            pass
        if h.mem.read(h.SENTINEL_ADDR, 4) == h.SENTINEL_VAL:
            found = True
            break

    assert found, "D-cache burst line-fill test did not complete"
    h.check_no_unexpected_exception()
    assert h.mem.read(res, 4) == 0xABC0_0000, (
        "the burst corrupted the long word the program asked for"
    )
    assert saw_full_line, "the burst did not fill all four entries of the line"
    assert _line_read_cycles(bus, data_base) == [data_base], (
        f"the line was filled with more than the one burst cycle: "
        f"{[hex(a) for a in _line_read_cycles(bus, data_base)]}"
    )
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

    dcache_mask = int(cache_state(dut).DCACHE_VALID[line_idx].value)
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

    icache_mask = int(cache_state(dut).ICACHE_VALID[line_idx].value)
    icache_tag = int(cache_state(dut).ICACHE_TAG[line_idx].value)
    word_valid = (icache_mask >> word_idx) & 0x1
    assert not (icache_tag == line_tag and word_valid == 1), (
        "Opcode bus-error miss must not mark target I-cache word valid"
    )
    h.cleanup()


async def _collect_burst_beats(dut, h, line_base, limit, done=None):
    """Line entries the streaming engine accepted, in arrival order."""
    entries = []
    for _ in range(limit):
        await RisingEdge(dut.CLK)
        try:
            if int(dut.BURST_BEAT_RDY.value):
                addr = int(dut.BURST_BEAT_ADDR.value)
                if (addr & ~0xF) == line_base:
                    entries.append((addr >> 2) & 0x3)
        except ValueError:
            pass
        if done is not None and done():
            break
    return entries


@cocotb.test()
async def test_icache_burst_wrap_order_from_midline_miss(dut):
    """A mid-line I-cache miss bursts in wraparound order from the entry it needs.

    UM 6.1.3.2: "Since the initial address is $06 when CBREQ is asserted, the next
    entry to be burst filled into the cache should correspond to address $08, then
    $0C, and last, $00" (Figure 6-12). The processor holds the address and the
    device increments A3:A2, so the order the entries arrive in is the wraparound
    order from the requested one.
    """
    h = CPUTestHarness(dut)
    line_base = 0x000120
    line_idx = (line_base >> 4) & 0xF
    miss_addr = line_base + 0x8   # entry 2

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

    bus = BusModel(dut, h.mem, sync_term=True, cback=True)
    await _bringup(dut, h, program, bus)

    def line_full():
        try:
            return int(cache_state(dut).ICACHE_VALID[line_idx].value) == 0xFF
        except (ValueError, TypeError, IndexError):
            return False

    entries = await _collect_burst_beats(dut, h, line_base, 20000, done=line_full)

    assert line_full(), "the mid-line burst did not complete the line"
    assert entries == [2, 3, 0, 1], (
        f"burst entries arrived as {entries}; UM Figure 6-12 wraps from the "
        f"requested entry ({(miss_addr >> 2) & 0x3})"
    )
    assert _line_read_cycles(bus, line_base) == [miss_addr], (
        f"the line was filled with more than the one burst cycle: "
        f"{[hex(a) for a in _line_read_cycles(bus, line_base)]}"
    )
    h.cleanup()


@cocotb.test()
async def test_dcache_burst_wrap_order_from_midline_miss(dut):
    """A mid-line D-cache miss bursts in wraparound order from the entry it needs.

    Same clause and the same Figure 6-12 example on the data side, where the
    requested long word is the third entry of the line.
    """
    h = CPUTestHarness(dut)
    line_base = h.DATA_BASE + 0x3C0
    line_idx = (line_base >> 4) & 0xF
    miss_addr = line_base + 0x8   # entry 2
    res = h.RESULT_BASE + 0x90

    program = [
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
        *imm_long(0x0000_1100),                    # ED=1, DBE=1
        *movec_dn_to_cr(0, CR_CACR),
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(miss_addr),
        *move(LONG, AN_IND, 0, DN, 1),             # miss on entry 2
        *move_to_abs_long(LONG, DN, 1, res),
        *h.sentinel_program(),
    ]

    for i in range(4):
        h.mem.load_long(line_base + 4 * i, 0xABC0_0100 | i)

    bus = BusModel(dut, h.mem, sync_term=True, cback=True)
    await _bringup(dut, h, program, bus)

    def done():
        return h.mem.read(h.SENTINEL_ADDR, 4) == h.SENTINEL_VAL

    entries = await _collect_burst_beats(dut, h, line_base, 20000, done=done)

    assert done(), "D-cache mid-line burst test did not complete"
    h.check_no_unexpected_exception()
    assert h.mem.read(res, 4) == 0xABC0_0102, (
        "the burst corrupted the long word the program asked for"
    )
    assert entries == [2, 3, 0, 1], (
        f"burst entries arrived as {entries}; UM Figure 6-12 wraps from the "
        f"requested entry ({(miss_addr >> 2) & 0x3})"
    )
    assert int(cache_state(dut).DCACHE_VALID[line_idx].value) == 0xF, (
        "the mid-line burst did not complete the line"
    )
    assert _line_read_cycles(bus, line_base) == [miss_addr], (
        f"the line was filled with more than the one burst cycle: "
        f"{[hex(a) for a in _line_read_cycles(bus, line_base)]}"
    )
    h.cleanup()


@cocotb.test()
async def test_ciout_holds_for_the_whole_cycle(dut):
    """CIOUTn must stay asserted for every clock of the cycle it belongs to.

    UM 7.3.1 drives CIOUT with the address and keeps it valid until the cycle
    terminates. This TT entry has RWM clear, so the CI decision depends on the
    read/write request strobes, which the request latches drop after the first
    clock of the cycle.
    """
    h = CPUTestHarness(dut)
    tt_addr = h.DATA_BASE + 0x260
    data_addr = h.DATA_BASE + 0x220
    res = h.RESULT_BASE + 0x7C

    # E=1, CI=1, R/W=1 (read), RWM=0, FC base=supervisor data, FC mask=000.
    tt0_ci_super_data_read = 0x00FF_8650

    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(tt_addr),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
        *imm_long(tt0_ci_super_data_read),
        *move(LONG, DN, 0, AN_IND, 0),             # (A0) = TT0 value
        *pmove_an_to_tt0(0),                       # PMOVE (A0),TT0

        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(data_addr),
        *move(LONG, AN_IND, 0, DN, 1),             # supervisor data read
        *move_to_abs_long(LONG, DN, 1, res),
        *h.sentinel_program(),
    ]

    h.mem.load_long(data_addr, 0x5566_7788)
    await h.setup(program)

    found = False
    saw_data_read = False
    ciout_low_clocks = 0
    ciout_high_clocks = 0

    for _ in range(28000):
        await RisingEdge(dut.CLK)
        try:
            as_n = int(dut.ASn.value)
            rw_n = int(dut.RWn.value)
            addr = int(dut.ADR_OUT.value)
            ciout_n = int(dut.CIOUTn.value)
        except ValueError:
            continue

        # Every clock of the read cycle, not just its first.
        if as_n == 0 and rw_n == 1 and addr == data_addr:
            saw_data_read = True
            if ciout_n == 0:
                ciout_low_clocks += 1
            else:
                ciout_high_clocks += 1

        if h.mem.read(h.SENTINEL_ADDR, 4) == h.SENTINEL_VAL:
            found = True
            break

    assert found, "CIOUT hold test did not complete"
    assert h.mem.read(res, 4) == 0x5566_7788, "Result writeback mismatch in CIOUT hold test"
    assert saw_data_read, "Did not observe the TT-CI-matched data read cycle"
    assert ciout_low_clocks >= 2, (
        f"Expected the read cycle to span at least two clocks with CIOUTn asserted, saw {ciout_low_clocks}"
    )
    assert ciout_high_clocks == 0, (
        f"CIOUTn negated for {ciout_high_clocks} clock(s) inside a cache-inhibited read cycle"
    )
    h.cleanup()


@cocotb.test()
async def test_ciin_prevents_dcache_read_fill(dut):
    """CIIN on a read cycle must keep the data out of the D-cache (UM 6.1.3.1).

    The control for this case is test_dcache_read_hit_after_fill, which shows
    the second read of the same longword served from the cache.
    """
    h = CPUTestHarness(dut)
    data_addr = h.DATA_BASE + 0x3A0
    res = h.RESULT_BASE + 0xC0

    program = [
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
        *imm_long(0x0000_0100),                    # ED=1
        *movec_dn_to_cr(0, CR_CACR),
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(data_addr),
        *move(LONG, AN_IND, 0, DN, 1),             # read 1 with CIIN asserted
        *move(LONG, AN_IND, 0, DN, 2),             # read 2 must reach the bus again
        *move_to_abs_long(LONG, DN, 2, res),
        *h.sentinel_program(),
    ]

    h.mem.load_long(data_addr, 0x1234_ABCD)
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

        # The responding device marks this longword non-cachable.
        dut.CIINn.value = 0 if (as_n == 0 and addr == data_addr) else 1

        if prev_as_n == 1 and as_n == 0 and rw_n == 1 and addr == data_addr:
            data_reads += 1
        prev_as_n = as_n
        if h.mem.read(h.SENTINEL_ADDR, 4) == h.SENTINEL_VAL:
            found = True
            break

    dut.CIINn.value = 1
    assert found, "CIIN D-cache read-fill test did not complete"
    assert h.mem.read(res, 4) == 0x1234_ABCD, "CIIN read must still deliver the data to the core"
    assert data_reads == 2, (
        f"Expected both reads of a CIIN-marked longword to reach the bus, saw {data_reads}"
    )
    h.cleanup()


@cocotb.test()
async def test_ciin_ignored_on_write_cycles(dut):
    """CIIN is ignored during write cycles (UM 5.7.1), so a write hit still updates."""
    h = CPUTestHarness(dut)
    data_addr = h.DATA_BASE + 0x3C0
    res = h.RESULT_BASE + 0xC4

    program = [
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
        *imm_long(0x0000_0100),                    # ED=1
        *movec_dn_to_cr(0, CR_CACR),
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(data_addr),
        *move(LONG, AN_IND, 0, DN, 1),             # warm fill, CIIN negated
        *moveq(0x22, 2),
        *addq(LONG, 3, AN, 0),                     # A0 = data_addr + 3
        *move(BYTE, DN, 2, AN_IND, 0),             # write hit with CIIN asserted
        *subq(LONG, 3, AN, 0),                     # A0 = data_addr
        *move(LONG, AN_IND, 0, DN, 3),             # must hit the merged entry
        *move_to_abs_long(LONG, DN, 3, res),
        *h.sentinel_program(),
    ]

    h.mem.load_long(data_addr, 0x7788_99AA)
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

        # Assert CIIN on every write cycle; the core must ignore it there.
        dut.CIINn.value = 0 if (as_n == 0 and rw_n == 0) else 1

        if prev_as_n == 1 and as_n == 0 and rw_n == 1 and addr == data_addr:
            data_reads += 1
        prev_as_n = as_n
        if h.mem.read(h.SENTINEL_ADDR, 4) == h.SENTINEL_VAL:
            found = True
            break

    dut.CIINn.value = 1
    assert found, "CIIN write-cycle test did not complete"
    assert h.mem.read(res, 4) == 0x7788_9922, (
        "CIIN on a write cycle must not stop the D-cache entry being updated"
    )
    assert data_reads == 1, f"Expected one external read (warm fill only), saw {data_reads}"
    h.cleanup()


@cocotb.test()
async def test_ciin_prevents_icache_fill(dut):
    """CIIN on an instruction prefetch must keep the word out of the I-cache."""
    h = CPUTestHarness(dut)

    # Same two loops as test_icache_hits_reduce_branch_loop_refetches, but with
    # CIIN asserted for every prefetch so the second loop cannot be cached.
    loop1_addrs = {0x000102, 0x000104}
    loop2_addrs = {0x000112, 0x000114}
    res0 = h.RESULT_BASE + 0xC8
    res1 = h.RESULT_BASE + 0xCC

    program = [
        *moveq(6, 2),
        *subq(LONG, 1, DN, 2),
        *bcc(CC_NE, -4),

        *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
        *imm_long(0x0000_0001),                    # EI=1
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
            addr = int(dut.ADR_OUT.value)
        except ValueError:
            continue

        # Program space is non-cachable for this system.
        dut.CIINn.value = 0 if (as_n == 0 and rw_n == 1 and addr < h.DATA_BASE) else 1

        if prev_as_n == 1 and as_n == 0 and rw_n == 1:
            if addr in loop1_addrs:
                loop1_fetches += 1
            if addr in loop2_addrs:
                loop2_fetches += 1
        prev_as_n = as_n
        if h.mem.read(h.SENTINEL_ADDR, 4) == h.SENTINEL_VAL:
            found = True
            break

    dut.CIINn.value = 1
    assert found, "CIIN I-cache fill test did not complete"
    assert h.mem.read(res0, 4) == 0, "Loop-1 counter did not terminate at zero"
    assert h.mem.read(res1, 4) == 0, "Loop-2 counter did not terminate at zero"
    assert loop2_fetches >= loop1_fetches, (
        f"CIIN-marked prefetches were cached anyway: loop1={loop1_fetches}, loop2={loop2_fetches}"
    )
    h.cleanup()


# ===================================================================
# Burst handshake -- UM 6.1.3.2 / 7.3.7
# ===================================================================
#
# UM 7.3.7: "The MC68030 allows burst filling only from 32-bit ports that
# terminate bus cycles with STERM and respond to CBREQ by asserting CBACK. When
# the MC68030 recognizes STERM and CBACK and it has asserted CBREQ, it maintains
# AS, DS, R/W, A0-A31, FC0-FC2, SIZ0-SIZ1 in their current state throughout the
# burst operation. The processor continues to accept data on every clock during
# which STERM is asserted until the burst is complete or an abnormal termination
# occurs." UM 6.1.3.2 adds: "The MC68030 ignores the assertion of CBACK during
# cycles terminated with DSACKx."
#
# This section covers the handshake -- which acknowledges the processor may honour
# at all, and that no burst is requested inside an RMC sequence -- and then the
# address-held streaming itself: a full line under one AS, and the three abnormal
# terminations UM 6.1.3.2 lists.

BURST_DATA_CACR = 0x0000_1100  # DBE, ED
BURST_ALL_CACR = 0x0000_1111   # DBE, ED, IBE, EI


async def _bringup(dut, h, program, bus):
    """Bring the CPU up against a caller-supplied bus model.

    CPUTestHarness.setup() always installs a plain asynchronous BusModel, so a
    test needing a synchronous or burst-capable device does the sequence itself.
    """
    clock = Clock(dut.CLK, 10, unit="ns")
    cocotb.start_soon(clock.start())
    h._init_idle_inputs()
    h._install_trap_stubs()
    h._load_memory(program)
    await RisingEdge(dut.CLK)
    await RisingEdge(dut.CLK)
    dut.RESET_INn.value = 0
    dut.HALT_INn.value = 0
    await ClockCycles(dut.CLK, 20)
    h.bus = bus
    await bus.start()
    dut.RESET_INn.value = 1
    dut.HALT_INn.value = 1
    await ClockCycles(dut.CLK, 4)


def _burst_read_program(h, data_addr, res, cacr=BURST_DATA_CACR):
    """Enable the data cache with burst filling, then miss on an aligned long.

    Instruction bursting is left off by default so the only burst-eligible cycle
    in the run is the one under test.
    """
    return [
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
        *imm_long(cacr),
        *movec_dn_to_cr(0, CR_CACR),
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(data_addr),
        *move(LONG, AN_IND, 0, DN, 1),             # aligned long read miss
        *move_to_abs_long(LONG, DN, 1, res),
        *h.sentinel_program(),
    ]


@cocotb.test()
async def test_cback_on_dsack_cycle_is_not_honoured(dut):
    """CBACK asserted on a DSACKx-terminated cycle must be ignored entirely.

    UM 6.1.3.2: "The MC68030 ignores the assertion of CBACK during cycles
    terminated with DSACKx." Ignoring it means the burst request is not consumed
    either, so CBREQ stays asserted for the whole cycle instead of being
    released by an acknowledge that never counted.
    """
    h = CPUTestHarness(dut)
    data_addr = h.DATA_BASE + 0x600
    res = h.RESULT_BASE + 0xD0

    h.mem.load_long(data_addr, 0x5A6B_7C8D)
    await h.setup(_burst_read_program(h, data_addr, res))
    dut.CBACKn.value = 0  # Device claims burst capability on an async cycle.

    found = False
    cbreq_samples = []
    honoured = False
    for _ in range(30000):
        await RisingEdge(dut.CLK)
        try:
            as_n = int(dut.ASn.value)
            rw_n = int(dut.RWn.value)
            addr = int(dut.ADR_OUT.value)
            cbreq_n = int(dut.CBREQn.value)
            honoured |= int(dut.CBACK_HONOURED.value) == 1
        except ValueError:
            continue

        if as_n == 0 and rw_n == 1 and addr == data_addr:
            cbreq_samples.append(cbreq_n)
        if h.mem.read(h.SENTINEL_ADDR, 4) == h.SENTINEL_VAL:
            found = True
            break

    dut.CBACKn.value = 1
    assert found, "DSACK-cycle CBACK test did not complete"
    assert h.mem.read(res, 4) == 0x5A6B_7C8D, "burst-eligible read returned wrong data"
    assert cbreq_samples, "never observed the burst-eligible data-read cycle"
    assert all(s == 0 for s in cbreq_samples), (
        f"CBREQn was released mid-cycle ({cbreq_samples}) by a CBACK on a "
        "DSACKx-terminated cycle, which the UM says to ignore"
    )
    assert not honoured, (
        "CBACK was honoured on a DSACKx-terminated cycle"
    )
    h.cleanup()


@cocotb.test()
async def test_cback_on_sterm_cycle_is_honoured(dut):
    """CBACK with STERM on a burst-eligible read is the one legal acknowledge.

    UM 7.3.7: "burst mode is only initiated if both of these signals are
    asserted for a synchronous cycle." The handshake must be recognised, and the
    requested operand must still reach the core intact.
    """
    h = CPUTestHarness(dut)
    data_addr = h.DATA_BASE + 0x640
    res = h.RESULT_BASE + 0xD4

    for i in range(4):
        h.mem.load_long((data_addr & ~0xF) + 4 * i, 0xC0DE_0000 | i)
    h.mem.load_long(data_addr, 0x91A2_B3C4)

    bus = BusModel(dut, h.mem, sync_term=True, cback=True)
    await _bringup(dut, h, _burst_read_program(h, data_addr, res), bus)

    found = False
    honoured = False
    for _ in range(30000):
        await RisingEdge(dut.CLK)
        try:
            honoured |= int(dut.CBACK_HONOURED.value) == 1
        except ValueError:
            pass
        if h.mem.read(h.SENTINEL_ADDR, 4) == h.SENTINEL_VAL:
            found = True
            break

    assert found, "STERM-cycle CBACK test did not complete"
    h.check_no_unexpected_exception()
    assert h.mem.read(res, 4) == 0x91A2_B3C4, (
        "the burst handshake corrupted the requested long word"
    )
    assert bus.bursts, (
        "the device never saw CBREQ asserted on a synchronous read, so the "
        "burst handshake was not exercised"
    )
    assert honoured, "CBACK with STERM on a burst-eligible read was not honoured"
    h.cleanup()


@cocotb.test()
async def test_cbreq_not_asserted_inside_rmc_sequence(dut):
    """No burst is requested anywhere inside a read-modify-write sequence.

    UM 7.3.7: "CBREQ is not asserted during the read or write cycles of any
    read-modify-write operation", and UM 6.1.3.2 lists the read portion of an
    RMW among the conditions under which CBREQ is never asserted.
    """
    h = CPUTestHarness(dut)
    data_addr = h.DATA_BASE + 0x680
    res = h.RESULT_BASE + 0xD8

    program = [
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
        *imm_long(BURST_ALL_CACR),
        *movec_dn_to_cr(0, CR_CACR),
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(data_addr),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 1),
        *imm_long(0x1111_2222),                    # Dc: the compare value.
        *move(LONG, SPECIAL, IMMEDIATE, DN, 2),
        *imm_long(0x3333_4444),                    # Du: the update value.
        *cas(LONG, 1, 2, AN_IND, 0),               # CAS.L D1,D2,(A0)
        *move(LONG, AN_IND, 0, DN, 3),
        *move_to_abs_long(LONG, DN, 3, res),
        *h.sentinel_program(),
    ]

    h.mem.load_long(data_addr, 0x1111_2222)  # Compare succeeds, so CAS writes.
    await h.setup(program)

    found = False
    rmc_cycles = 0
    rmc_cbreq_low = 0
    for _ in range(30000):
        await RisingEdge(dut.CLK)
        try:
            as_n = int(dut.ASn.value)
            rmc_n = int(dut.RMCn.value)
            cbreq_n = int(dut.CBREQn.value)
        except ValueError:
            continue

        if as_n == 0 and rmc_n == 0:
            rmc_cycles += 1
            if cbreq_n == 0:
                rmc_cbreq_low += 1
        if h.mem.read(h.SENTINEL_ADDR, 4) == h.SENTINEL_VAL:
            found = True
            break

    assert found, "RMC burst-request test did not complete"
    assert h.mem.read(res, 4) == 0x3333_4444, "CAS did not write the update value"
    assert rmc_cycles, "never observed a bus cycle with RMC asserted"
    assert rmc_cbreq_low == 0, (
        f"CBREQn was asserted on {rmc_cbreq_low} of {rmc_cycles} RMC bus clocks"
    )
    h.cleanup()


# -------------------------------------------------------------------
# Address-held burst streaming (UM 7.3.7).
#
# UM 7.3.7 requires the processor to hold AS, DS, R/W, A0-A31, FC0-FC2 and
# SIZ0-SIZ1 for the whole burst and to take one long word per STERM. The device
# side is BusModel(sync_term=True, cback=True): it holds CBACK with STERM,
# advances A3:A2 itself with line wraparound (UM Figure 6-12), and supports
# cback_drop_after and ciin_on_beat for the abnormal terminations.
#
# Residual deviations, deliberate and not observable on the bus: the requested
# long word reaches the execution unit when the burst completes rather than at
# the end of the first access (UM 6.1.3.2 makes it available immediately), and
# SIZ0-SIZ1 during an instruction burst reads word rather than long because this
# core's instruction prefetch is word sized. UM Figure 6-13's deferred burst --
# the second portion of an operand that crosses a line boundary requesting a
# burst of its own -- is not implemented either: only an aligned long-word data
# read or an instruction prefetch requests one.
# -------------------------------------------------------------------


def _line_read_cycles(bus, line_base):
    """Read sub-cycle addresses the device saw inside one cache line."""
    return [
        addr for addr, _code, is_write in bus.sub_cycles
        if not is_write and line_base <= addr < line_base + 16
    ]


@cocotb.test()
async def test_burst_streams_a_full_line_under_one_as(dut):
    """A burst-eligible miss fills the line without re-driving the address.

    UM 6.1.3.2: "The MC68030 holds the entire address bus constant for the
    duration of the burst cycle." So one AS cycle covers the whole line and the
    device supplies four long words, wrapping A3:A2 from the requested one.
    """
    h = CPUTestHarness(dut)
    data_addr = h.DATA_BASE + 0x700
    line_base = data_addr & ~0xF
    res = h.RESULT_BASE + 0xDC

    for i in range(4):
        h.mem.load_long(line_base + 4 * i, 0xB105_0000 | i)

    bus = BusModel(dut, h.mem, sync_term=True, cback=True)
    await _bringup(dut, h, _burst_read_program(h, data_addr, res), bus)

    found = False
    for _ in range(30000):
        await RisingEdge(dut.CLK)
        if h.mem.read(h.SENTINEL_ADDR, 4) == h.SENTINEL_VAL:
            found = True
            break
    assert found, "burst streaming test did not complete"

    assert bus.bursts, "no burst was acknowledged"
    assert bus.bursts[0]["beats"] == 4, (
        f"device supplied {bus.bursts[0]['beats']} long words; the processor did "
        "not hold the bus for a four-beat burst"
    )
    assert _line_read_cycles(bus, line_base) == [data_addr], (
        f"line filled with separate AS cycles: "
        f"{[hex(a) for a in _line_read_cycles(bus, line_base)]}"
    )
    h.cleanup()


@cocotb.test()
async def test_cback_negated_mid_burst_ends_it_cleanly(dut):
    """CBACK negated after two long words ends the burst, keeping what arrived.

    UM 6.1.3.2: "The premature negation of the CBACK signal during the burst
    operation causes the current cycle to complete normally, loading the data
    successfully transferred into the appropriate cache. However, the burst
    operation aborts and CBREQ negates."
    """
    h = CPUTestHarness(dut)
    data_addr = h.DATA_BASE + 0x740
    line_base = data_addr & ~0xF
    line_idx = (line_base >> 4) & 0xF
    res = h.RESULT_BASE + 0xE0

    for i in range(4):
        h.mem.load_long(line_base + 4 * i, 0xB2A5_0000 | i)

    bus = BusModel(dut, h.mem, sync_term=True, cback=True, cback_drop_after=2)
    await _bringup(dut, h, _burst_read_program(h, data_addr, res), bus)

    found = False
    for _ in range(30000):
        await RisingEdge(dut.CLK)
        if h.mem.read(h.SENTINEL_ADDR, 4) == h.SENTINEL_VAL:
            found = True
            break
    assert found, "premature CBACK negation test did not complete"

    assert bus.bursts and bus.bursts[0]["cback_dropped"], "CBACK was never dropped"
    assert bus.bursts[0]["beats"] == 2, (
        f"device supplied {bus.bursts[0]['beats']} long words, expected 2"
    )
    # Two entries loaded, and no further external traffic for the other two.
    valid = int(cache_state(dut).DCACHE_VALID[line_idx].value)
    assert bin(valid).count("1") == 2, (
        f"D-cache line validity 0x{valid:X} after a two-beat burst"
    )
    assert _line_read_cycles(bus, line_base) == [data_addr], (
        "the aborted burst was followed by extra AS cycles for the same line"
    )
    h.cleanup()


@cocotb.test()
async def test_ciin_mid_burst_aborts_it(dut):
    """CIIN on a burst fill beat keeps that long word uncached and ends the burst.

    UM 6.1.3.2: "The assertion of CIIN during the second, third, or fourth cycle
    of a burst operation prevents the data during that cycle from being loaded
    into the appropriate cache and causes CBREQ to negate, aborting the burst
    operation."
    """
    h = CPUTestHarness(dut)
    data_addr = h.DATA_BASE + 0x780
    line_base = data_addr & ~0xF
    line_idx = (line_base >> 4) & 0xF
    res = h.RESULT_BASE + 0xE4

    for i in range(4):
        h.mem.load_long(line_base + 4 * i, 0xB3C1_0000 | i)

    bus = BusModel(dut, h.mem, sync_term=True, cback=True, ciin_on_beat=3)
    await _bringup(dut, h, _burst_read_program(h, data_addr, res), bus)

    found = False
    for _ in range(30000):
        await RisingEdge(dut.CLK)
        if h.mem.read(h.SENTINEL_ADDR, 4) == h.SENTINEL_VAL:
            found = True
            break
    assert found, "CIIN mid-burst test did not complete"

    assert bus.bursts and bus.bursts[0]["ciin"], "CIIN was never asserted mid-burst"
    valid = int(cache_state(dut).DCACHE_VALID[line_idx].value)
    assert bin(valid).count("1") == 2, (
        f"D-cache line validity 0x{valid:X}: the CIIN-marked beat and everything "
        "after it must stay out of the cache"
    )
    assert _line_read_cycles(bus, line_base) == [data_addr], (
        "the aborted burst was followed by extra AS cycles for the same line"
    )
    h.cleanup()
