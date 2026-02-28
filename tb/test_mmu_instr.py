"""
MMU instruction bring-up tests for WF68K30L.

Coverage in this phase:
  - F-line MMU decode reaches execution in supervisor mode (no Line-F trap).
  - PMOVE TC round-trip through memory using (An) addressing.
  - PMOVE/PMOVEFD SRP/CRP quadword transfers.
  - MMU configuration exception (vector 56) on invalid TC/CRP/SRP writes.
  - Minimal translation path: TC[E], SRE root select, DT=1 constant offset, TT matches.
  - PTEST/PLOAD/PFLUSH Phase-3 semantics (MMUSR + ATC side effects).
  - PMOVE FD/PFLUSH flush-side-effect strobe.
  - MMU instructions are privileged in user mode (vector 8).
"""

import cocotb

from cpu_harness import CPUTestHarness
from m68k_encode import (
    LONG,
    WORD,
    DN,
    AN_IND,
    SPECIAL,
    IMMEDIATE,
    move,
    movea,
    moveq,
    move_to_abs_long,
    move_to_sr,
    imm_long,
    imm_word,
)

MMUSR_B = 1 << 15
MMUSR_L = 1 << 14
MMUSR_S = 1 << 13
MMUSR_W = 1 << 11
MMUSR_I = 1 << 10
MMUSR_M = 1 << 9
MMUSR_T = 1 << 6
MMUSR_N = 0x0007


@cocotb.test()
async def test_mmu_decode_supervisor_surface(dut):
    """Supervisor mode: PFLUSHA/PLOADR/PTESTR should execute and reach sentinel."""
    h = CPUTestHarness(dut)

    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),  # A0 = test logical address
        *imm_long(h.DATA_BASE),
        0xF000, 0x2400,                       # PFLUSHA
        0xF010, 0x2211,                       # PLOADR #1,(A0)
        0xF010, 0x8611,                       # PTESTR #1,(A0),#1
        *h.sentinel_program(),
    ]

    await h.setup(program)
    found = await h.run_until_sentinel(max_cycles=6000)
    assert found, "MMU supervisor instruction sequence did not complete"
    h.cleanup()


@cocotb.test()
async def test_pmove_tc_roundtrip_memory(dut):
    """PMOVE (A0)->TC then TC->(A1) should round-trip the longword value."""
    h = CPUTestHarness(dut)
    src_addr = h.DATA_BASE + 0x40
    dst_addr = h.DATA_BASE + 0x80
    payload = 0x12345678

    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),  # A0 = src
        *imm_long(src_addr),
        *movea(LONG, SPECIAL, IMMEDIATE, 1),  # A1 = dst
        *imm_long(dst_addr),
        0xF010, 0x4000,                       # PMOVE (A0),TC
        0xF011, 0x4200,                       # PMOVE TC,(A1)
        *h.sentinel_program(),
    ]

    await h.setup(program)
    h.mem.load_long(src_addr, payload)
    found = await h.run_until_sentinel(max_cycles=8000)
    assert found, "PMOVE TC round-trip did not complete"

    got = h.mem.read(dst_addr, 4)
    assert got == payload, f"PMOVE TC round-trip mismatch: got 0x{got:08X}, expected 0x{payload:08X}"
    h.cleanup()


@cocotb.test()
async def test_pmove_root_pointer_quadword_roundtrip(dut):
    """PMOVE/PMOVEFD for SRP/CRP should transfer full 64-bit quadwords."""
    h = CPUTestHarness(dut)

    srp_src = h.DATA_BASE + 0x100
    srp_dst = h.DATA_BASE + 0x120
    crp_src = h.DATA_BASE + 0x140
    crp_dst = h.DATA_BASE + 0x160

    # Keep DT!=0 in SRP/CRP descriptors so MMU configuration exception is not raised.
    srp_hi = 0xA1B2C3D5
    srp_lo = 0x11223344
    crp_hi = 0x55667789
    crp_lo = 0x99AABBCC

    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),  # A0 = SRP src
        *imm_long(srp_src),
        *movea(LONG, SPECIAL, IMMEDIATE, 1),  # A1 = SRP dst
        *imm_long(srp_dst),
        0xF010, 0x4800,                       # PMOVE (A0),SRP
        0xF011, 0x4A00,                       # PMOVE SRP,(A1)

        *movea(LONG, SPECIAL, IMMEDIATE, 2),  # A2 = CRP src
        *imm_long(crp_src),
        *movea(LONG, SPECIAL, IMMEDIATE, 3),  # A3 = CRP dst
        *imm_long(crp_dst),
        0xF012, 0x4D00,                       # PMOVEFD (A2),CRP
        0xF013, 0x4E00,                       # PMOVE CRP,(A3)

        *h.sentinel_program(),
    ]

    await h.setup(program)
    h.mem.load_long(srp_src + 0, srp_hi)
    h.mem.load_long(srp_src + 4, srp_lo)
    h.mem.load_long(crp_src + 0, crp_hi)
    h.mem.load_long(crp_src + 4, crp_lo)

    found = await h.run_until_sentinel(max_cycles=14000)
    assert found, "PMOVE SRP/CRP quadword round-trip did not complete"

    got_srp_hi = h.mem.read(srp_dst + 0, 4)
    got_srp_lo = h.mem.read(srp_dst + 4, 4)
    got_crp_hi = h.mem.read(crp_dst + 0, 4)
    got_crp_lo = h.mem.read(crp_dst + 4, 4)

    assert (got_srp_hi, got_srp_lo) == (srp_hi, srp_lo), (
        f"SRP round-trip mismatch: got 0x{got_srp_hi:08X}_"
        f"{got_srp_lo:08X}, expected 0x{srp_hi:08X}_{srp_lo:08X}"
    )
    assert (got_crp_hi, got_crp_lo) == (crp_hi, crp_lo), (
        f"CRP round-trip mismatch: got 0x{got_crp_hi:08X}_"
        f"{got_crp_lo:08X}, expected 0x{crp_hi:08X}_{crp_lo:08X}"
    )
    h.cleanup()


@cocotb.test()
async def test_mmu_config_exception_invalid_tc(dut):
    """Invalid TC load (E=1 with reserved PS) should trap vector 56 and clear TC[E]."""
    h = CPUTestHarness(dut)
    handler_addr = 0x000680
    vector_addr = 56 * 4
    tc_src = h.DATA_BASE + 0x200
    tc_dst = h.DATA_BASE + 0x220
    invalid_tc = 0x80700000  # E=1, PS=0x7 (reserved) -> MMU configuration exception.

    handler_code = [
        *moveq(0x38, 1),
        *move_to_abs_long(LONG, DN, 1, h.RESULT_BASE),
        *movea(LONG, SPECIAL, IMMEDIATE, 1),  # A1 = TC dump destination
        *imm_long(tc_dst),
        0xF011, 0x4200,                       # PMOVE TC,(A1)
        *h.sentinel_program(),
    ]

    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),  # A0 = TC source
        *imm_long(tc_src),
        0xF010, 0x4000,                       # PMOVE (A0),TC
        *moveq(0x11, 1),                      # Should not execute (post-instruction exception).
        *move_to_abs_long(LONG, DN, 1, h.RESULT_BASE),
        *h.sentinel_program(),
    ]

    await h.setup(program)
    h.mem.load_long(vector_addr, handler_addr)
    h.mem.load_words(handler_addr, handler_code)
    h.mem.load_long(tc_src, invalid_tc)

    found = await h.run_until_sentinel(max_cycles=14000)
    assert found, "Invalid TC MMU configuration test did not complete"

    marker = h.read_result_long(0)
    assert marker == 0x38, f"Expected MMU configuration vector marker 0x38, got 0x{marker:08X}"
    tc_after = h.mem.read(tc_dst, 4)
    assert tc_after == (invalid_tc & 0x7FFFFFFF), (
        f"TC after invalid load should clear E bit: got 0x{tc_after:08X}, expected 0x{(invalid_tc & 0x7FFFFFFF):08X}"
    )
    h.cleanup()


@cocotb.test()
async def test_mmu_config_exception_invalid_srp(dut):
    """Invalid DT=0 load into SRP should trap vector 56 after register update."""
    h = CPUTestHarness(dut)
    handler_addr = 0x0006E0
    vector_addr = 56 * 4
    srp_src = h.DATA_BASE + 0x240
    srp_dst = h.DATA_BASE + 0x260

    srp_hi = 0xAA550000  # DT field (bits 33:32) = 0 -> invalid root descriptor.
    srp_lo = 0x00112233

    handler_code = [
        *moveq(0x38, 1),
        *move_to_abs_long(LONG, DN, 1, h.RESULT_BASE),
        *movea(LONG, SPECIAL, IMMEDIATE, 1),  # A1 = SRP dump destination
        *imm_long(srp_dst),
        0xF011, 0x4A00,                       # PMOVE SRP,(A1)
        *h.sentinel_program(),
    ]

    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),  # A0 = SRP source
        *imm_long(srp_src),
        0xF010, 0x4800,                       # PMOVE (A0),SRP -> MMU cfg exception
        *h.sentinel_program(),
    ]

    await h.setup(program)
    h.mem.load_long(vector_addr, handler_addr)
    h.mem.load_words(handler_addr, handler_code)
    h.mem.load_long(srp_src + 0, srp_hi)
    h.mem.load_long(srp_src + 4, srp_lo)

    found = await h.run_until_sentinel(max_cycles=14000)
    assert found, "Invalid SRP MMU configuration test did not complete"

    marker = h.read_result_long(0)
    assert marker == 0x38, f"Expected MMU configuration vector marker 0x38, got 0x{marker:08X}"
    got_srp = (h.mem.read(srp_dst + 0, 4), h.mem.read(srp_dst + 4, 4))
    assert got_srp == (srp_hi, srp_lo), (
        f"SRP should be updated before MMU cfg exception: got 0x{got_srp[0]:08X}_{got_srp[1]:08X}"
    )
    h.cleanup()


@cocotb.test()
async def test_mmu_config_exception_invalid_crp(dut):
    """Invalid DT=0 load into CRP should trap vector 56 after register update."""
    h = CPUTestHarness(dut)
    handler_addr = 0x000720
    vector_addr = 56 * 4
    crp_src = h.DATA_BASE + 0x280
    crp_dst = h.DATA_BASE + 0x2A0

    crp_hi = 0xCC770000  # DT field (bits 33:32) = 0 -> invalid root descriptor.
    crp_lo = 0x44556677

    handler_code = [
        *moveq(0x38, 1),
        *move_to_abs_long(LONG, DN, 1, h.RESULT_BASE),
        *movea(LONG, SPECIAL, IMMEDIATE, 2),  # A2 = CRP dump destination
        *imm_long(crp_dst),
        0xF012, 0x4E00,                       # PMOVE CRP,(A2)
        *h.sentinel_program(),
    ]

    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 3),  # A3 = CRP source
        *imm_long(crp_src),
        0xF013, 0x4C00,                       # PMOVE (A3),CRP -> MMU cfg exception
        *h.sentinel_program(),
    ]

    await h.setup(program)
    h.mem.load_long(vector_addr, handler_addr)
    h.mem.load_words(handler_addr, handler_code)
    h.mem.load_long(crp_src + 0, crp_hi)
    h.mem.load_long(crp_src + 4, crp_lo)

    found = await h.run_until_sentinel(max_cycles=14000)
    assert found, "Invalid CRP MMU configuration test did not complete"

    marker = h.read_result_long(0)
    assert marker == 0x38, f"Expected MMU configuration vector marker 0x38, got 0x{marker:08X}"
    got_crp = (h.mem.read(crp_dst + 0, 4), h.mem.read(crp_dst + 4, 4))
    assert got_crp == (crp_hi, crp_lo), (
        f"CRP should be updated before MMU cfg exception: got 0x{got_crp[0]:08X}_{got_crp[1]:08X}"
    )
    h.cleanup()


@cocotb.test()
async def test_mmu_translation_minimal_path_tc_sre_dt1(dut):
    """MMU translation uses SRP when SRE=1 and DT=1 root offset mode is enabled."""
    h = CPUTestHarness(dut)

    logical_addr = 0x12000400
    crp_offset = 0x00010000
    srp_offset = 0x00020000

    logical_pre = 0x11223344
    via_crp = 0x55667788
    via_srp = 0xAABBCCDD

    tt0_src = h.DATA_BASE + 0x300
    tt1_src = h.DATA_BASE + 0x320
    crp_src = h.DATA_BASE + 0x340
    srp_src = h.DATA_BASE + 0x360
    tc_src = h.DATA_BASE + 0x380

    # TT0: transparent supervisor program fetches (FC=6), top-byte base/mask = 0x00/0x00.
    tt0_prog = 0x00008360
    # TT1: transparent supervisor data accesses in top-byte 0x00 region (FC=5), RWM=1.
    tt1_data = 0x00008350
    crp_hi = 0x7FFF0001
    srp_hi = 0x7FFF0001
    tc_val = 0x82808880  # E=1, SRE=1, PS=8, IS=0, TIA=8, TIB=8, TIC=8, TID=0 -> sum 32.

    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),  # A0 = logical data address
        *imm_long(logical_addr),
        *move(LONG, AN_IND, 0, DN, 0),        # D0 = [logical] while TC.E=0 (pass-through)
        *move_to_abs_long(LONG, DN, 0, h.RESULT_BASE),

        *movea(LONG, SPECIAL, IMMEDIATE, 2),  # A2 = TT0 source
        *imm_long(tt0_src),
        0xF012, 0x0800,                       # PMOVE (A2),TT0
        *movea(LONG, SPECIAL, IMMEDIATE, 3),  # A3 = TT1 source
        *imm_long(tt1_src),
        0xF013, 0x0C00,                       # PMOVE (A3),TT1

        *movea(LONG, SPECIAL, IMMEDIATE, 4),  # A4 = CRP source
        *imm_long(crp_src),
        0xF014, 0x4C00,                       # PMOVE (A4),CRP
        *movea(LONG, SPECIAL, IMMEDIATE, 5),  # A5 = SRP source
        *imm_long(srp_src),
        0xF015, 0x4800,                       # PMOVE (A5),SRP
        *movea(LONG, SPECIAL, IMMEDIATE, 6),  # A6 = TC source
        *imm_long(tc_src),
        0xF016, 0x4000,                       # PMOVE (A6),TC

        *move(LONG, AN_IND, 0, DN, 1),        # D1 = [logical] with TC.E=1, SRE=1 -> SRP offset path.
        *move_to_abs_long(LONG, DN, 1, h.RESULT_BASE + 4),
        *h.sentinel_program(),
    ]

    await h.setup(program)
    h.mem.load_long(tt0_src, tt0_prog)
    h.mem.load_long(tt1_src, tt1_data)
    h.mem.load_long(crp_src + 0, crp_hi)
    h.mem.load_long(crp_src + 4, crp_offset)
    h.mem.load_long(srp_src + 0, srp_hi)
    h.mem.load_long(srp_src + 4, srp_offset)
    h.mem.load_long(tc_src, tc_val)

    # Logical and translated physical locations (memory model wraps at 1 MiB).
    h.mem.load_long(logical_addr & 0xFFFFF, logical_pre)
    h.mem.load_long((logical_addr + crp_offset) & 0xFFFFF, via_crp)
    h.mem.load_long((logical_addr + srp_offset) & 0xFFFFF, via_srp)

    found = await h.run_until_sentinel(max_cycles=26000)
    assert found, "Minimal MMU translation test did not complete"

    pre = h.read_result_long(0)
    post = h.read_result_long(4)
    assert pre == logical_pre, f"With TC disabled, expected logical read 0x{logical_pre:08X}, got 0x{pre:08X}"
    assert post == via_srp, f"With TC enabled+SRE, expected SRP translated read 0x{via_srp:08X}, got 0x{post:08X}"
    assert post != via_crp, "Post-translation read should use SRP, not CRP"
    h.cleanup()


@cocotb.test()
async def test_mmu_tt_rw_rwm_field_behavior(dut):
    """TT R/W and RWM fields follow MC68030 transparent translation register semantics."""
    h = CPUTestHarness(dut)

    logical_addr = 0x12000500
    crp_offset = 0x00018000
    logical_value = 0x11112222
    translated_value = 0x33334444

    tt_prog_src = h.DATA_BASE + 0x3A0
    tt_ro_src = h.DATA_BASE + 0x3C0
    tt_wo_src = h.DATA_BASE + 0x3E0
    tt_rwm_src = h.DATA_BASE + 0x400
    crp_src = h.DATA_BASE + 0x420
    tc_src = h.DATA_BASE + 0x440

    # TT1: transparent low-address accesses for supervisor spaces (FC2=1), R/W ignored.
    # This keeps instruction fetch and test data/result buffers in top-byte 0x00 pass-through.
    tt_prog = 0x00008143
    # TT0 variants for supervisor data (FC=5), top-byte base=0x12, mask=0x00:
    #   R/W=1,RWM=0 -> read-only transparent hit
    #   R/W=0,RWM=0 -> write-only transparent hit (read misses)
    #   R/W=0,RWM=1 -> R/W ignored (read hits)
    tt_read_only = 0x12008250
    tt_write_only = 0x12008050
    tt_rw_ignored = 0x12008150

    crp_hi = 0x7FFF0001
    tc_val = 0x80808880  # E=1, SRE=0, PS=8, IS=0, TIA=8, TIB=8, TIC=8, TID=0.

    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),  # A0 = logical test address
        *imm_long(logical_addr),

        *movea(LONG, SPECIAL, IMMEDIATE, 2),  # A2 = TT1 source (program pass-through)
        *imm_long(tt_prog_src),
        0xF012, 0x0C00,                       # PMOVE (A2),TT1

        *movea(LONG, SPECIAL, IMMEDIATE, 3),  # A3 = CRP source
        *imm_long(crp_src),
        0xF013, 0x4C00,                       # PMOVE (A3),CRP
        *movea(LONG, SPECIAL, IMMEDIATE, 4),  # A4 = TC source
        *imm_long(tc_src),
        0xF014, 0x4000,                       # PMOVE (A4),TC

        *movea(LONG, SPECIAL, IMMEDIATE, 1),  # A1 = TT0 read-only
        *imm_long(tt_ro_src),
        0xF011, 0x0800,                       # PMOVE (A1),TT0
        *move(LONG, AN_IND, 0, DN, 0),        # D0 = read should hit TT0 (logical)
        *move_to_abs_long(LONG, DN, 0, h.RESULT_BASE),

        *movea(LONG, SPECIAL, IMMEDIATE, 5),  # A5 = TT0 write-only
        *imm_long(tt_wo_src),
        0xF015, 0x0800,                       # PMOVE (A5),TT0
        *move(LONG, AN_IND, 0, DN, 1),        # D1 = read should miss TT0 (translated)
        *move_to_abs_long(LONG, DN, 1, h.RESULT_BASE + 4),

        *movea(LONG, SPECIAL, IMMEDIATE, 6),  # A6 = TT0 R/W ignored
        *imm_long(tt_rwm_src),
        0xF016, 0x0800,                       # PMOVE (A6),TT0
        *move(LONG, AN_IND, 0, DN, 2),        # D2 = read should hit TT0 (logical)
        *move_to_abs_long(LONG, DN, 2, h.RESULT_BASE + 8),

        *h.sentinel_program(),
    ]

    await h.setup(program)
    h.mem.load_long(tt_prog_src, tt_prog)
    h.mem.load_long(tt_ro_src, tt_read_only)
    h.mem.load_long(tt_wo_src, tt_write_only)
    h.mem.load_long(tt_rwm_src, tt_rw_ignored)
    h.mem.load_long(crp_src + 0, crp_hi)
    h.mem.load_long(crp_src + 4, crp_offset)
    h.mem.load_long(tc_src, tc_val)

    h.mem.load_long(logical_addr & 0xFFFFF, logical_value)
    h.mem.load_long((logical_addr + crp_offset) & 0xFFFFF, translated_value)

    found = await h.run_until_sentinel(max_cycles=32000)
    assert found, "MMU TT R/W/RWM behavior test did not complete"

    got_read_only = h.read_result_long(0)
    got_write_only = h.read_result_long(4)
    got_rw_ignored = h.read_result_long(8)

    assert got_read_only == logical_value, (
        f"TT read-only expected logical value 0x{logical_value:08X}, got 0x{got_read_only:08X}"
    )
    assert got_write_only == translated_value, (
        f"TT write-only read should translate to 0x{translated_value:08X}, got 0x{got_write_only:08X}"
    )
    assert got_rw_ignored == logical_value, (
        f"TT RWM=1 should ignore R/W and read logical 0x{logical_value:08X}, got 0x{got_rw_ignored:08X}"
    )
    h.cleanup()


@cocotb.test()
async def test_mmu_ptest_level0_vs_level7_t_bit(dut):
    """PTEST level 0 reports TT hit in MMUSR.T; level 7 clears T for the same address."""
    h = CPUTestHarness(dut)

    logical_addr = 0x12000600
    tt1_src = h.DATA_BASE + 0x460
    mmusr0_dst = h.DATA_BASE + 0x480
    mmusr7_dst = h.DATA_BASE + 0x4A0
    crp_src = h.DATA_BASE + 0x4C0
    tc_src = h.DATA_BASE + 0x4E0

    # TT1: supervisor program pass-through for low addresses (keeps test code/data stable).
    tt_prog = 0x00008143
    # TT0: supervisor data transparent mapping for top-byte 0x12, both R/W (RWM=1), FC=5.
    tt_data = 0x12008150
    crp_hi = 0x7FFF0001
    crp_offs = 0x00000000
    tc_val = 0x80808880

    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),  # A0 = logical address to test
        *imm_long(logical_addr),
        *movea(LONG, SPECIAL, IMMEDIATE, 1),  # A1 = TT1 source
        *imm_long(tt1_src),
        0xF011, 0x0C00,                       # PMOVE (A1),TT1
        *movea(LONG, SPECIAL, IMMEDIATE, 2),  # A2 = TT0 source
        *imm_long(tt1_src + 4),
        0xF012, 0x0800,                       # PMOVE (A2),TT0

        # Install a valid root/TC model so level-7 table search returns a valid status.
        *movea(LONG, SPECIAL, IMMEDIATE, 5),
        *imm_long(crp_src),
        0xF015, 0x4C00,                       # PMOVE (A5),CRP
        *movea(LONG, SPECIAL, IMMEDIATE, 6),
        *imm_long(tc_src),
        0xF016, 0x4000,                       # PMOVE (A6),TC

        0xF010, 0x8215,                       # PTESTR #5,(A0),#0
        *movea(LONG, SPECIAL, IMMEDIATE, 3),  # A3 = MMUSR L0 dump
        *imm_long(mmusr0_dst),
        0xF013, 0x6200,                       # PMOVE MMUSR,(A3)

        0xF010, 0x9E15,                       # PTESTR #5,(A0),#7
        *movea(LONG, SPECIAL, IMMEDIATE, 4),  # A4 = MMUSR L7 dump
        *imm_long(mmusr7_dst),
        0xF014, 0x6200,                       # PMOVE MMUSR,(A4)

        *h.sentinel_program(),
    ]

    await h.setup(program)
    h.mem.load_long(tt1_src, tt_prog)
    h.mem.load_long(tt1_src + 4, tt_data)
    h.mem.load_long(crp_src + 0, crp_hi)
    h.mem.load_long(crp_src + 4, crp_offs)
    h.mem.load_long(tc_src, tc_val)

    found = await h.run_until_sentinel(max_cycles=24000)
    assert found, "PTEST level0/level7 TT behavior test did not complete"

    mmusr_l0 = h.mem.read(mmusr0_dst, 2)
    mmusr_l7 = h.mem.read(mmusr7_dst, 2)
    assert (mmusr_l0 & MMUSR_T) != 0, f"Level-0 PTEST should set MMUSR.T on TT hit, got 0x{mmusr_l0:04X}"
    assert (mmusr_l7 & MMUSR_T) == 0, f"Level-7 PTEST should clear MMUSR.T, got 0x{mmusr_l7:04X}"
    assert (mmusr_l7 & MMUSR_I) == 0, f"Level-7 PTEST on mapped address should be valid, got 0x{mmusr_l7:04X}"
    h.cleanup()


@cocotb.test()
async def test_mmu_ptest_level7_short_walk_reports_n_w_m(dut):
    """PTEST level 7 reports levels/WP/modified from short-descriptor table search."""
    h = CPUTestHarness(dut)

    logical_addr = 0x12345008
    root_tbl = 0x00000400
    lvlb_tbl = 0x00000800
    page_tbl = 0x00000C00
    page_base = 0x00123000

    desc_a_addr = root_tbl + (0x12 << 2)
    desc_b_addr = lvlb_tbl + (0x34 << 2)
    desc_c_addr = page_tbl + (0x5 << 2)
    desc_a = (lvlb_tbl & 0xFFFFFFF0) | 0x00000002
    desc_b = (page_tbl & 0xFFFFFFF0) | 0x00000002
    desc_c = (page_base & 0xFFFFFF00) | 0x00000015  # DT=1, WP=1, M=1

    tt0_src = h.DATA_BASE + 0xB00
    tt1_src = h.DATA_BASE + 0xB20
    crp_src = h.DATA_BASE + 0xB40
    tc_src = h.DATA_BASE + 0xB60
    mmusr_dst = h.DATA_BASE + 0xB80

    tt0_prog = 0x00008360
    tt1_prog = 0x00008350
    crp_hi = 0x7FFF0002
    crp_lo = root_tbl
    tc_val = 0x80C08840

    program = [
        # Prime descriptor shadow while MMU is disabled.
        *movea(LONG, SPECIAL, IMMEDIATE, 2),
        *imm_long(desc_a_addr),
        *move(LONG, AN_IND, 2, DN, 2),
        *movea(LONG, SPECIAL, IMMEDIATE, 3),
        *imm_long(desc_b_addr),
        *move(LONG, AN_IND, 3, DN, 3),
        *movea(LONG, SPECIAL, IMMEDIATE, 4),
        *imm_long(desc_c_addr),
        *move(LONG, AN_IND, 4, DN, 4),

        *movea(LONG, SPECIAL, IMMEDIATE, 6),
        *imm_long(tt0_src),
        0xF016, 0x0800,                       # PMOVE (A6),TT0
        *movea(LONG, SPECIAL, IMMEDIATE, 7),
        *imm_long(tt1_src),
        0xF017, 0x0C00,                       # PMOVE (A7),TT1

        *movea(LONG, SPECIAL, IMMEDIATE, 1),
        *imm_long(crp_src),
        0xF011, 0x4C00,                       # PMOVE (A1),CRP
        *movea(LONG, SPECIAL, IMMEDIATE, 2),
        *imm_long(tc_src),
        0xF012, 0x4000,                       # PMOVE (A2),TC

        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(logical_addr),
        0xF010, 0x9E15,                       # PTESTR #5,(A0),#7
        *movea(LONG, SPECIAL, IMMEDIATE, 5),
        *imm_long(mmusr_dst),
        0xF015, 0x6200,                       # PMOVE MMUSR,(A5)

        *h.sentinel_program(),
    ]

    await h.setup(program)
    h.mem.load_long(desc_a_addr, desc_a)
    h.mem.load_long(desc_b_addr, desc_b)
    h.mem.load_long(desc_c_addr, desc_c)
    h.mem.load_long(tt0_src, tt0_prog)
    h.mem.load_long(tt1_src, tt1_prog)
    h.mem.load_long(crp_src + 0, crp_hi)
    h.mem.load_long(crp_src + 4, crp_lo)
    h.mem.load_long(tc_src, tc_val)

    found = await h.run_until_sentinel(max_cycles=100000)
    assert found, "PTEST level-7 short-walk MMUSR test did not complete"

    mmusr = h.mem.read(mmusr_dst, 2)
    assert (mmusr & MMUSR_T) == 0, f"Level-7 PTEST must clear T, got 0x{mmusr:04X}"
    assert (mmusr & MMUSR_I) == 0, f"Expected valid translation (I=0), got 0x{mmusr:04X}"
    assert (mmusr & MMUSR_W) != 0, f"Expected WP accumulation (W=1), got 0x{mmusr:04X}"
    assert (mmusr & MMUSR_M) != 0, f"Expected modified-page status (M=1), got 0x{mmusr:04X}"
    assert (mmusr & MMUSR_N) == 0x3, f"Expected N=3 levels, got N={mmusr & MMUSR_N} (MMUSR=0x{mmusr:04X})"
    assert (mmusr & (MMUSR_B | MMUSR_L | MMUSR_S)) == 0, f"Unexpected fault bits set: 0x{mmusr:04X}"
    h.cleanup()


@cocotb.test()
async def test_mmu_ptest_level7_limit_violation_sets_l_i(dut):
    """PTEST level 7 reports L/I for root-index limit violation."""
    h = CPUTestHarness(dut)

    logical_addr = 0x12345008
    root_tbl = 0x00000400
    tt0_src = h.DATA_BASE + 0xBA0
    tt1_src = h.DATA_BASE + 0xBC0
    crp_src = h.DATA_BASE + 0xBE0
    tc_src = h.DATA_BASE + 0xC00
    mmusr_dst = h.DATA_BASE + 0xC20

    tt0_prog = 0x00008360
    tt1_prog = 0x00008350
    crp_hi = 0x00010002  # DT=2, upper-limit mode, limit=1 (index 0x12 will violate).
    crp_lo = root_tbl
    tc_val = 0x80C08840

    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 6),
        *imm_long(tt0_src),
        0xF016, 0x0800,                       # PMOVE (A6),TT0
        *movea(LONG, SPECIAL, IMMEDIATE, 7),
        *imm_long(tt1_src),
        0xF017, 0x0C00,                       # PMOVE (A7),TT1

        *movea(LONG, SPECIAL, IMMEDIATE, 1),
        *imm_long(crp_src),
        0xF011, 0x4C00,                       # PMOVE (A1),CRP
        *movea(LONG, SPECIAL, IMMEDIATE, 2),
        *imm_long(tc_src),
        0xF012, 0x4000,                       # PMOVE (A2),TC

        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(logical_addr),
        0xF010, 0x9E15,                       # PTESTR #5,(A0),#7
        *movea(LONG, SPECIAL, IMMEDIATE, 5),
        *imm_long(mmusr_dst),
        0xF015, 0x6200,                       # PMOVE MMUSR,(A5)

        *h.sentinel_program(),
    ]

    await h.setup(program)
    h.mem.load_long(tt0_src, tt0_prog)
    h.mem.load_long(tt1_src, tt1_prog)
    h.mem.load_long(crp_src + 0, crp_hi)
    h.mem.load_long(crp_src + 4, crp_lo)
    h.mem.load_long(tc_src, tc_val)

    found = await h.run_until_sentinel(max_cycles=60000)
    assert found, "PTEST level-7 limit MMUSR test did not complete"

    mmusr = h.mem.read(mmusr_dst, 2)
    assert (mmusr & MMUSR_L) != 0, f"Expected limit violation (L=1), got 0x{mmusr:04X}"
    assert (mmusr & MMUSR_I) != 0, f"Expected invalid translation (I=1) with limit fault, got 0x{mmusr:04X}"
    assert (mmusr & MMUSR_N) == 0x0, f"Expected N=0 for pre-fetch root limit fault, got N={mmusr & MMUSR_N}"
    assert (mmusr & (MMUSR_B | MMUSR_S)) == 0, f"Unexpected B/S on limit-only case: 0x{mmusr:04X}"
    h.cleanup()


@cocotb.test()
async def test_mmu_ptest_level7_long_supervisor_violation_sets_s_i(dut):
    """PTEST level 7 reports S/I when user FC hits long-descriptor supervisor-only mapping."""
    h = CPUTestHarness(dut)

    logical_addr = 0x12345008
    root_tbl = 0x00002000
    lvla_tbl = 0x00002400
    desc_a_addr = root_tbl + (0x12 << 3)
    desc_a_lo = 0x7FFF0103  # DT=3 long table descriptor with S=1.
    desc_a_hi = lvla_tbl

    tt0_src = h.DATA_BASE + 0xC40
    tt1_src = h.DATA_BASE + 0xC60
    crp_src = h.DATA_BASE + 0xC80
    tc_src = h.DATA_BASE + 0xCA0
    mmusr_dst = h.DATA_BASE + 0xCC0

    tt0_prog = 0x00008360
    tt1_prog = 0x00008350
    crp_hi = 0x7FFF0003
    crp_lo = root_tbl
    tc_val = 0x80C08840

    program = [
        # Prime descriptor shadow while MMU is disabled.
        *movea(LONG, SPECIAL, IMMEDIATE, 2),
        *imm_long(desc_a_addr + 0),
        *move(LONG, AN_IND, 2, DN, 2),
        *movea(LONG, SPECIAL, IMMEDIATE, 2),
        *imm_long(desc_a_addr + 4),
        *move(LONG, AN_IND, 2, DN, 2),

        *movea(LONG, SPECIAL, IMMEDIATE, 6),
        *imm_long(tt0_src),
        0xF016, 0x0800,                       # PMOVE (A6),TT0
        *movea(LONG, SPECIAL, IMMEDIATE, 7),
        *imm_long(tt1_src),
        0xF017, 0x0C00,                       # PMOVE (A7),TT1

        *movea(LONG, SPECIAL, IMMEDIATE, 1),
        *imm_long(crp_src),
        0xF011, 0x4C00,                       # PMOVE (A1),CRP
        *movea(LONG, SPECIAL, IMMEDIATE, 2),
        *imm_long(tc_src),
        0xF012, 0x4000,                       # PMOVE (A2),TC

        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(logical_addr),
        0xF010, 0x9E11,                       # PTESTR #1,(A0),#7 (user data FC)
        *movea(LONG, SPECIAL, IMMEDIATE, 5),
        *imm_long(mmusr_dst),
        0xF015, 0x6200,                       # PMOVE MMUSR,(A5)

        *h.sentinel_program(),
    ]

    await h.setup(program)
    h.mem.load_long(desc_a_addr + 0, desc_a_lo)
    h.mem.load_long(desc_a_addr + 4, desc_a_hi)
    h.mem.load_long(tt0_src, tt0_prog)
    h.mem.load_long(tt1_src, tt1_prog)
    h.mem.load_long(crp_src + 0, crp_hi)
    h.mem.load_long(crp_src + 4, crp_lo)
    h.mem.load_long(tc_src, tc_val)

    found = await h.run_until_sentinel(max_cycles=90000)
    assert found, "PTEST level-7 long S-bit MMUSR test did not complete"

    mmusr = h.mem.read(mmusr_dst, 2)
    assert (mmusr & MMUSR_S) != 0, f"Expected supervisor violation (S=1), got 0x{mmusr:04X}"
    assert (mmusr & MMUSR_I) == 0, f"Supervisor violation should not force I in level-7 MMUSR, got 0x{mmusr:04X}"
    assert (mmusr & MMUSR_N) == 0x1, f"Expected N=1 (fault on first long descriptor), got N={mmusr & MMUSR_N}"
    assert (mmusr & MMUSR_L) == 0, f"Unexpected limit violation in supervisor-only case: 0x{mmusr:04X}"
    h.cleanup()


@cocotb.test()
async def test_mmu_ptest_level7_invalid_descriptor_sets_i(dut):
    """PTEST level 7 reports I on invalid descriptor DT=0."""
    h = CPUTestHarness(dut)

    logical_addr = 0x12345008
    root_tbl = 0x00003000
    desc_a_addr = root_tbl + (0x12 << 2)
    desc_a = 0x00000000  # DT=0 invalid descriptor.

    tt0_src = h.DATA_BASE + 0xCE0
    tt1_src = h.DATA_BASE + 0xD00
    crp_src = h.DATA_BASE + 0xD20
    tc_src = h.DATA_BASE + 0xD40
    mmusr_dst = h.DATA_BASE + 0xD60

    tt0_prog = 0x00008360
    tt1_prog = 0x00008350
    crp_hi = 0x7FFF0002
    crp_lo = root_tbl
    tc_val = 0x80C08840

    program = [
        # Prime descriptor shadow while MMU is disabled.
        *movea(LONG, SPECIAL, IMMEDIATE, 2),
        *imm_long(desc_a_addr),
        *move(LONG, AN_IND, 2, DN, 2),

        *movea(LONG, SPECIAL, IMMEDIATE, 6),
        *imm_long(tt0_src),
        0xF016, 0x0800,                       # PMOVE (A6),TT0
        *movea(LONG, SPECIAL, IMMEDIATE, 7),
        *imm_long(tt1_src),
        0xF017, 0x0C00,                       # PMOVE (A7),TT1

        *movea(LONG, SPECIAL, IMMEDIATE, 1),
        *imm_long(crp_src),
        0xF011, 0x4C00,                       # PMOVE (A1),CRP
        *movea(LONG, SPECIAL, IMMEDIATE, 2),
        *imm_long(tc_src),
        0xF012, 0x4000,                       # PMOVE (A2),TC

        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(logical_addr),
        0xF010, 0x9E15,                       # PTESTR #5,(A0),#7
        *movea(LONG, SPECIAL, IMMEDIATE, 5),
        *imm_long(mmusr_dst),
        0xF015, 0x6200,                       # PMOVE MMUSR,(A5)

        *h.sentinel_program(),
    ]

    await h.setup(program)
    h.mem.load_long(desc_a_addr, desc_a)
    h.mem.load_long(tt0_src, tt0_prog)
    h.mem.load_long(tt1_src, tt1_prog)
    h.mem.load_long(crp_src + 0, crp_hi)
    h.mem.load_long(crp_src + 4, crp_lo)
    h.mem.load_long(tc_src, tc_val)

    found = await h.run_until_sentinel(max_cycles=90000)
    assert found, "PTEST level-7 invalid-descriptor MMUSR test did not complete"

    mmusr = h.mem.read(mmusr_dst, 2)
    assert (mmusr & MMUSR_I) != 0, f"Expected invalid descriptor status (I=1), got 0x{mmusr:04X}"
    assert (mmusr & (MMUSR_B | MMUSR_L | MMUSR_S)) == 0, f"Unexpected B/L/S for DT=0 invalid descriptor: 0x{mmusr:04X}"
    assert (mmusr & MMUSR_N) == 0x1, f"Expected N=1 (first descriptor examined), got N={mmusr & MMUSR_N}"
    h.cleanup()


@cocotb.test()
async def test_mmu_pload_and_pflush_fc_mask_semantics(dut):
    """PLOADR/PLOADW update ATC entry state; PFLUSH FC/MASK invalidates matching entries."""
    h = CPUTestHarness(dut)

    logical_addr = 0x34000700
    crp_src = h.DATA_BASE + 0x4C0
    mmusr_before = h.DATA_BASE + 0x4E0
    mmusr_after_r = h.DATA_BASE + 0x500
    mmusr_after_w = h.DATA_BASE + 0x520
    mmusr_after_flush = h.DATA_BASE + 0x540
    tt_prog_src = h.DATA_BASE + 0x560

    crp_hi = 0x7FFF0001
    crp_offset = 0x00010000
    tt_prog = 0x00008143

    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 7),  # A7 = TT1 source
        *imm_long(tt_prog_src),
        0xF017, 0x0C00,                       # PMOVE (A7),TT1

        *movea(LONG, SPECIAL, IMMEDIATE, 1),  # A1 = CRP source
        *imm_long(crp_src),
        0xF011, 0x4C00,                       # PMOVE (A1),CRP

        *movea(LONG, SPECIAL, IMMEDIATE, 0),  # A0 = logical address
        *imm_long(logical_addr),

        0xF010, 0x8215,                       # PTESTR #5,(A0),#0 -> expect I=1 (ATC miss)
        *movea(LONG, SPECIAL, IMMEDIATE, 2),
        *imm_long(mmusr_before),
        0xF012, 0x6200,                       # PMOVE MMUSR,(A2)

        0xF010, 0x2215,                       # PLOADR #5,(A0)
        0xF010, 0x8215,                       # PTESTR #5,(A0),#0 -> expect I=0, M=0
        *movea(LONG, SPECIAL, IMMEDIATE, 3),
        *imm_long(mmusr_after_r),
        0xF013, 0x6200,                       # PMOVE MMUSR,(A3)

        0xF010, 0x2015,                       # PLOADW #5,(A0)
        0xF010, 0x8215,                       # PTESTR #5,(A0),#0 -> expect M=1
        *movea(LONG, SPECIAL, IMMEDIATE, 4),
        *imm_long(mmusr_after_w),
        0xF014, 0x6200,                       # PMOVE MMUSR,(A4)

        0xF000, 0x30F5,                       # PFLUSH #5,#111
        0xF010, 0x8215,                       # PTESTR #5,(A0),#0 -> expect I=1
        *movea(LONG, SPECIAL, IMMEDIATE, 5),
        *imm_long(mmusr_after_flush),
        0xF015, 0x6200,                       # PMOVE MMUSR,(A5)

        *h.sentinel_program(),
    ]

    await h.setup(program)
    h.mem.load_long(crp_src + 0, crp_hi)
    h.mem.load_long(crp_src + 4, crp_offset)
    h.mem.load_long(tt_prog_src, tt_prog)

    found = await h.run_until_sentinel(max_cycles=36000)
    assert found, "PLOAD/PFLUSH FC+MASK semantics test did not complete"

    mmusr0 = h.mem.read(mmusr_before, 2)
    mmusr_r = h.mem.read(mmusr_after_r, 2)
    mmusr_w = h.mem.read(mmusr_after_w, 2)
    mmusr_f = h.mem.read(mmusr_after_flush, 2)

    assert (mmusr0 & MMUSR_I) != 0, f"Expected ATC miss before PLOAD (I=1), got 0x{mmusr0:04X}"
    assert (mmusr_r & MMUSR_I) == 0, f"Expected ATC hit after PLOADR (I=0), got 0x{mmusr_r:04X}"
    assert (mmusr_r & MMUSR_M) == 0, f"Expected M=0 after PLOADR, got 0x{mmusr_r:04X}"
    assert (mmusr_w & MMUSR_I) == 0, f"Expected ATC hit after PLOADW (I=0), got 0x{mmusr_w:04X}"
    assert (mmusr_w & MMUSR_M) != 0, f"Expected M=1 after PLOADW, got 0x{mmusr_w:04X}"
    assert (mmusr_f & MMUSR_I) != 0, f"Expected entry invalidated by PFLUSH FC/MASK, got 0x{mmusr_f:04X}"
    h.cleanup()


@cocotb.test()
async def test_mmu_pflush_fc_mask_ea_selective(dut):
    """PFLUSH FC/MASK,<ea> invalidates only the selected logical page."""
    h = CPUTestHarness(dut)

    logical_a = 0x35000800
    logical_b = 0x36000900
    crp_src = h.DATA_BASE + 0x580
    mmusr_pre_a = h.DATA_BASE + 0x590
    mmusr_pre_b = h.DATA_BASE + 0x594
    mmusr_a = h.DATA_BASE + 0x5A0
    mmusr_b = h.DATA_BASE + 0x5C0
    tt_prog_src = h.DATA_BASE + 0x5E0

    crp_hi = 0x7FFF0001
    crp_offset = 0x00010000
    tt_prog = 0x00008143

    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 7),  # A7 = TT1 source
        *imm_long(tt_prog_src),
        0xF017, 0x0C00,                       # PMOVE (A7),TT1

        *movea(LONG, SPECIAL, IMMEDIATE, 6),  # A6 = CRP source
        *imm_long(crp_src),
        0xF016, 0x4C00,                       # PMOVE (A6),CRP

        *movea(LONG, SPECIAL, IMMEDIATE, 0),  # A0 = logical A
        *imm_long(logical_a),
        *movea(LONG, SPECIAL, IMMEDIATE, 1),  # A1 = logical B
        *imm_long(logical_b),

        0xF010, 0x2215,                       # PLOADR #5,(A0)
        0xF011, 0x2215,                       # PLOADR #5,(A1)

        0xF010, 0x8215,                       # PTESTR #5,(A0),#0 -> expect I=0
        *movea(LONG, SPECIAL, IMMEDIATE, 4),
        *imm_long(mmusr_pre_a),
        0xF014, 0x6200,                       # PMOVE MMUSR,(A4)

        0xF011, 0x8215,                       # PTESTR #5,(A1),#0 -> expect I=0
        *movea(LONG, SPECIAL, IMMEDIATE, 5),
        *imm_long(mmusr_pre_b),
        0xF015, 0x6200,                       # PMOVE MMUSR,(A5)

        0xF010, 0x38F5,                       # PFLUSH #5,#111,(A0)

        0xF010, 0x8215,                       # PTESTR #5,(A0),#0 -> expect I=1
        *movea(LONG, SPECIAL, IMMEDIATE, 2),
        *imm_long(mmusr_a),
        0xF012, 0x6200,                       # PMOVE MMUSR,(A2)

        0xF011, 0x8215,                       # PTESTR #5,(A1),#0 -> expect I=0
        *movea(LONG, SPECIAL, IMMEDIATE, 3),
        *imm_long(mmusr_b),
        0xF013, 0x6200,                       # PMOVE MMUSR,(A3)

        *h.sentinel_program(),
    ]

    await h.setup(program)
    h.mem.load_long(crp_src + 0, crp_hi)
    h.mem.load_long(crp_src + 4, crp_offset)
    h.mem.load_long(tt_prog_src, tt_prog)

    found = await h.run_until_sentinel(max_cycles=36000)
    assert found, "PFLUSH FC/MASK/EA selective invalidation test did not complete"

    mmusr_pre_a_val = h.mem.read(mmusr_pre_a, 2)
    mmusr_pre_b_val = h.mem.read(mmusr_pre_b, 2)
    mmusr_a_val = h.mem.read(mmusr_a, 2)
    mmusr_b_val = h.mem.read(mmusr_b, 2)
    assert (mmusr_pre_a_val & MMUSR_I) == 0, f"Expected A0 resident before selective flush, got 0x{mmusr_pre_a_val:04X}"
    assert (mmusr_pre_b_val & MMUSR_I) == 0, f"Expected A1 resident before selective flush, got 0x{mmusr_pre_b_val:04X}"
    assert (mmusr_a_val & MMUSR_I) != 0, f"Selected EA should be invalidated, got 0x{mmusr_a_val:04X}"
    assert (mmusr_b_val & MMUSR_I) == 0, f"Non-selected EA should remain resident, got 0x{mmusr_b_val:04X}"
    h.cleanup()


@cocotb.test()
async def test_mmu_runtime_atc_refill_hit_and_flush(dut):
    """Runtime accesses refill ATC, reuse cached mapping, and honor subsequent flush."""
    h = CPUTestHarness(dut)

    logical_addr = 0x37000A00
    crp1_src = h.DATA_BASE + 0x620
    crp2_src = h.DATA_BASE + 0x640
    tc_src = h.DATA_BASE + 0x660
    tt1_src = h.DATA_BASE + 0x680

    crp_hi = 0x7FFF0001
    crp_offs_1 = 0x00012000
    crp_offs_2 = 0x00024000
    tc_val = 0x80808880  # E=1, SRE=0, PS=8, IS=0, TIA=8, TIB=8, TIC=8, TID=0.
    tt_prog = 0x00008143  # Keep low-address supervisor accesses transparent.

    value_logical = 0xCAFEBABE
    value_phys_1 = 0x11223344
    value_phys_2 = 0x55667788

    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 7),  # A7 = TT1 source
        *imm_long(tt1_src),
        0xF017, 0x0C00,                       # PMOVE (A7),TT1

        *movea(LONG, SPECIAL, IMMEDIATE, 1),  # A1 = CRP #1 source
        *imm_long(crp1_src),
        0xF011, 0x4C00,                       # PMOVE (A1),CRP
        *movea(LONG, SPECIAL, IMMEDIATE, 2),  # A2 = TC source
        *imm_long(tc_src),
        0xF012, 0x4000,                       # PMOVE (A2),TC

        *movea(LONG, SPECIAL, IMMEDIATE, 0),  # A0 = logical address
        *imm_long(logical_addr),
        *move(LONG, AN_IND, 0, DN, 0),        # D0 = translated via CRP #1 (ATC refill)
        *move_to_abs_long(LONG, DN, 0, h.RESULT_BASE),

        *movea(LONG, SPECIAL, IMMEDIATE, 3),  # A3 = CRP #2 source
        *imm_long(crp2_src),
        0xF013, 0x4D00,                       # PMOVEFD (A3),CRP (no ATC flush)
        *move(LONG, AN_IND, 0, DN, 1),        # D1 = ATC hit, still old mapping
        *move_to_abs_long(LONG, DN, 1, h.RESULT_BASE + 4),

        0xF000, 0x2400,                       # PFLUSHA
        *move(LONG, AN_IND, 0, DN, 2),        # D2 = miss/refill with CRP #2 mapping
        *move_to_abs_long(LONG, DN, 2, h.RESULT_BASE + 8),

        *h.sentinel_program(),
    ]

    await h.setup(program)
    h.mem.load_long(tt1_src, tt_prog)
    h.mem.load_long(crp1_src + 0, crp_hi)
    h.mem.load_long(crp1_src + 4, crp_offs_1)
    h.mem.load_long(crp2_src + 0, crp_hi)
    h.mem.load_long(crp2_src + 4, crp_offs_2)
    h.mem.load_long(tc_src, tc_val)

    h.mem.load_long(logical_addr & 0xFFFFF, value_logical)
    h.mem.load_long((logical_addr + crp_offs_1) & 0xFFFFF, value_phys_1)
    h.mem.load_long((logical_addr + crp_offs_2) & 0xFFFFF, value_phys_2)

    found = await h.run_until_sentinel(max_cycles=42000)
    assert found, "Runtime ATC refill/hit/flush MMU test did not complete"

    got_first = h.read_result_long(0)
    got_cached = h.read_result_long(4)
    got_after_flush = h.read_result_long(8)
    assert got_first == value_phys_1, (
        f"First translated read expected 0x{value_phys_1:08X}, got 0x{got_first:08X}"
    )
    assert got_cached == value_phys_1, (
        f"ATC hit should preserve old mapping 0x{value_phys_1:08X}, got 0x{got_cached:08X}"
    )
    assert got_after_flush == value_phys_2, (
        f"After PFLUSHA expected CRP#2 mapping 0x{value_phys_2:08X}, got 0x{got_after_flush:08X}"
    )
    assert got_after_flush != got_cached, "Flush should force translation refresh to new CRP mapping"
    h.cleanup()


@cocotb.test()
async def test_mmu_runtime_invalid_root_traps_vector56(dut):
    """Runtime MMU fault on unsupported root DT should trap to vector 56."""
    h = CPUTestHarness(dut)

    handler_addr = 0x0007C0
    vector_addr = 56 * 4
    logical_addr = 0x38000B00
    tt1_src = h.DATA_BASE + 0x6A0
    crp_bad_src = h.DATA_BASE + 0x6C0
    tc_src = h.DATA_BASE + 0x6E0

    tt_prog = 0x00008143
    crp_bad_hi = 0x7FFF0002  # DT=2: unsupported in current runtime path, no PMOVE cfg exception.
    crp_bad_lo = 0x00010000
    tc_val = 0x80808880

    handler_code = [
        *moveq(0x38, 1),
        *move_to_abs_long(LONG, DN, 1, h.RESULT_BASE),
        *h.sentinel_program(),
    ]

    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 7),  # A7 = TT1 source
        *imm_long(tt1_src),
        0xF017, 0x0C00,                       # PMOVE (A7),TT1

        *movea(LONG, SPECIAL, IMMEDIATE, 1),  # A1 = CRP invalid-root source
        *imm_long(crp_bad_src),
        0xF011, 0x4D00,                       # PMOVEFD (A1),CRP
        *movea(LONG, SPECIAL, IMMEDIATE, 2),  # A2 = TC source
        *imm_long(tc_src),
        0xF012, 0x4000,                       # PMOVE (A2),TC

        *movea(LONG, SPECIAL, IMMEDIATE, 0),  # A0 = faulting logical address
        *imm_long(logical_addr),
        *move(LONG, AN_IND, 0, DN, 0),        # Runtime MMU fault -> vector 56
        *moveq(0x11, 1),                      # Should not execute
        *move_to_abs_long(LONG, DN, 1, h.RESULT_BASE),
        *h.sentinel_program(),
    ]

    await h.setup(program)
    h.mem.load_long(vector_addr, handler_addr)
    h.mem.load_words(handler_addr, handler_code)
    h.mem.load_long(tt1_src, tt_prog)
    h.mem.load_long(crp_bad_src + 0, crp_bad_hi)
    h.mem.load_long(crp_bad_src + 4, crp_bad_lo)
    h.mem.load_long(tc_src, tc_val)

    found = await h.run_until_sentinel(max_cycles=26000)
    if not found:
        debug = (
            f"TRAP_MMU_CFG={int(dut.TRAP_MMU_CFG.value)} "
            f"BUSY_EXH={int(dut.BUSY_EXH.value)} "
            f"BUS_BSY={int(dut.BUS_BSY.value)} "
            f"ASn={int(dut.ASn.value)} "
            f"DSn={int(dut.DSn.value)} "
            f"RWn={int(dut.RWn.value)} "
            f"OPCODE_REQ={int(dut.OPCODE_REQ.value)} "
            f"OPCODE_REQ_CORE_MISS={int(dut.OPCODE_REQ_CORE_MISS.value)} "
            f"DATA_RD_BUS={int(dut.DATA_RD_BUS.value)} "
            f"DATA_WR={int(dut.DATA_WR.value)} "
            f"RD_REQ={int(dut.RD_REQ.value)} "
            f"WR_REQ={int(dut.WR_REQ.value)}"
        )
        assert found, f"Runtime MMU invalid-root fault test did not complete ({debug})"

    marker = h.read_result_long(0)
    assert marker == 0x38, f"Expected MMU configuration vector marker 0x38, got 0x{marker:08X}"
    h.cleanup()


@cocotb.test()
async def test_mmu_runtime_short_descriptor_table_walk(dut):
    """Runtime MMU translates through short-format descriptor tables on ATC miss."""
    h = CPUTestHarness(dut)

    logical_addr = 0x12345008
    page_base = 0x00234000
    expected_phys_addr = page_base + (logical_addr & 0xFFF)
    expected_value = 0xA1B2C3D4
    logical_value = 0x0BADF00D

    root_tbl = 0x00000400
    lvlb_tbl = 0x00000800
    page_tbl = 0x00000C00
    desc_a_addr = root_tbl + (0x12 << 2)
    desc_b_addr = lvlb_tbl + (0x34 << 2)
    desc_c_addr = page_tbl + (0x5 << 2)
    desc_a = (lvlb_tbl & 0xFFFFFFF0) | 0x00000002  # DT=2 short table descriptor
    desc_b = (page_tbl & 0xFFFFFFF0) | 0x00000002  # DT=2 short table descriptor
    desc_c = (page_base & 0xFFFFFF00) | 0x00000001  # DT=1 short page descriptor

    tt0_src = h.DATA_BASE + 0x700
    tt1_src = h.DATA_BASE + 0x720
    crp_src = h.DATA_BASE + 0x740
    tc_src = h.DATA_BASE + 0x760
    tt0_prog = 0x00008360
    tt1_prog = 0x00008350
    crp_hi = 0x7FFF0002  # Root DT=2 (short descriptors), upper-limit mode.
    crp_lo = root_tbl
    tc_val = 0x80C08840  # E=1, PS=12, IS=0, TIA=8, TIB=8, TIC=4, TID=0.
    handler_addr = 0x000900
    vector_addr = 56 * 4
    handler_code = [
        *moveq(0x38, 1),
        *move_to_abs_long(LONG, DN, 1, h.RESULT_BASE),
        *h.sentinel_program(),
    ]

    program = [
        # Prime descriptor shadow with explicit long reads while MMU is disabled.
        *movea(LONG, SPECIAL, IMMEDIATE, 2),
        *imm_long(desc_a_addr),
        *move(LONG, AN_IND, 2, DN, 2),
        *movea(LONG, SPECIAL, IMMEDIATE, 3),
        *imm_long(desc_b_addr),
        *move(LONG, AN_IND, 3, DN, 3),
        *movea(LONG, SPECIAL, IMMEDIATE, 4),
        *imm_long(desc_c_addr),
        *move(LONG, AN_IND, 4, DN, 4),

        # Keep low-address supervisor program/data traffic transparent.
        *movea(LONG, SPECIAL, IMMEDIATE, 6),
        *imm_long(tt0_src),
        0xF016, 0x0800,                       # PMOVE (A6),TT0
        *movea(LONG, SPECIAL, IMMEDIATE, 7),
        *imm_long(tt1_src),
        0xF017, 0x0C00,                       # PMOVE (A7),TT1

        # Install CRP and TC (enables MMU table walk).
        *movea(LONG, SPECIAL, IMMEDIATE, 1),
        *imm_long(crp_src),
        0xF011, 0x4C00,                       # PMOVE (A1),CRP
        *movea(LONG, SPECIAL, IMMEDIATE, 2),
        *imm_long(tc_src),
        0xF012, 0x4000,                       # PMOVE (A2),TC

        *movea(LONG, SPECIAL, IMMEDIATE, 0),  # A0 = logical address
        *imm_long(logical_addr),
        *move(LONG, AN_IND, 0, DN, 1),        # D1 = translated load
        *move_to_abs_long(LONG, DN, 1, h.RESULT_BASE),

        *h.sentinel_program(),
    ]

    await h.setup(program)
    h.mem.load_long(vector_addr, handler_addr)
    h.mem.load_words(handler_addr, handler_code)
    h.mem.load_long(desc_a_addr, desc_a)
    h.mem.load_long(desc_b_addr, desc_b)
    h.mem.load_long(desc_c_addr, desc_c)
    h.mem.load_long(expected_phys_addr & 0xFFFFF, expected_value)
    h.mem.load_long(logical_addr & 0xFFFFF, logical_value)
    h.mem.load_long(tt0_src, tt0_prog)
    h.mem.load_long(tt1_src, tt1_prog)
    h.mem.load_long(crp_src + 0, crp_hi)
    h.mem.load_long(crp_src + 4, crp_lo)
    h.mem.load_long(tc_src, tc_val)

    found = await h.run_until_sentinel(max_cycles=70000)
    if not found:
        debug = (
            f"TRAP_MMU_CFG={int(dut.TRAP_MMU_CFG.value)} "
            f"BUSY_EXH={int(dut.BUSY_EXH.value)} "
            f"BUS_BSY={int(dut.BUS_BSY.value)} "
            f"ASn={int(dut.ASn.value)} "
            f"RWn={int(dut.RWn.value)} "
            f"OPCODE_REQ_CORE_MISS={int(dut.OPCODE_REQ_CORE_MISS.value)} "
            f"DATA_RD_BUS={int(dut.DATA_RD_BUS.value)} "
            f"RD_REQ={int(dut.RD_REQ.value)} "
            f"OP={int(dut.OP.value)} "
            f"FC={int(dut.FC_I.value)} "
            f"ADR_P=0x{int(dut.ADR_P.value):08X} "
            f"ADR_P_PHYS=0x{int(dut.ADR_P_PHYS.value):08X} "
            f"MMU_REQ={int(dut.MMU_RUNTIME_REQ.value)} "
            f"MMU_FAULT={int(dut.MMU_RUNTIME_FAULT.value)} "
            f"PC=0x{int(dut.PC.value):08X} "
            f"DESCV_POP={int(dut.MMU_DESC_SHADOW_V.value).bit_count()} "
            f"RESULT0=0x{h.read_result_long(0):08X} "
            f"SENT=0x{h.mem.read(h.SENTINEL_ADDR, 4):08X}"
        )
        assert found, f"Runtime short-table descriptor MMU translation test did not complete ({debug})"

    got = h.read_result_long(0)
    assert got == expected_value, (
        f"Expected translated value 0x{expected_value:08X}, got 0x{got:08X} "
        f"(FAULT_ADR=0x{int(dut.FAULT_ADR.value):08X} FC={int(dut.FC_I.value)} "
        f"SBIT={int(dut.SBIT.value)} PC=0x{int(dut.PC.value):08X})"
    )
    h.cleanup()


@cocotb.test()
async def test_mmu_runtime_short_descriptor_wp_fault_vector56(dut):
    """Short-format page descriptor WP bit should fault writes and vector to 56."""
    h = CPUTestHarness(dut)

    handler_addr = 0x000860
    vector_addr = 56 * 4
    logical_addr = 0x1234500C
    page_base = 0x00234000

    root_tbl = 0x00000400
    lvlb_tbl = 0x00000800
    page_tbl = 0x00000C00
    desc_a_addr = root_tbl + (0x12 << 2)
    desc_b_addr = lvlb_tbl + (0x34 << 2)
    desc_c_addr = page_tbl + (0x5 << 2)
    desc_a = (lvlb_tbl & 0xFFFFFFF0) | 0x00000002  # DT=2
    desc_b = (page_tbl & 0xFFFFFFF0) | 0x00000002  # DT=2
    desc_c_wp = (page_base & 0xFFFFFF00) | 0x00000005  # DT=1, WP=1

    tt0_src = h.DATA_BASE + 0x760
    tt1_src = h.DATA_BASE + 0x780
    crp_src = h.DATA_BASE + 0x7A0
    tc_src = h.DATA_BASE + 0x7C0
    tt0_prog = 0x00008260
    tt1_prog = 0x00008350
    crp_hi = 0x7FFF0002
    crp_lo = root_tbl
    tc_val = 0x80C08840

    handler_code = [
        *moveq(0x38, 1),
        *move_to_abs_long(LONG, DN, 1, h.RESULT_BASE),
        *h.sentinel_program(),
    ]

    program = [
        # Prime descriptor shadow with descriptor reads before enabling MMU.
        *movea(LONG, SPECIAL, IMMEDIATE, 2),
        *imm_long(desc_a_addr),
        *move(LONG, AN_IND, 2, DN, 2),
        *movea(LONG, SPECIAL, IMMEDIATE, 3),
        *imm_long(desc_b_addr),
        *move(LONG, AN_IND, 3, DN, 3),
        *movea(LONG, SPECIAL, IMMEDIATE, 4),
        *imm_long(desc_c_addr),
        *move(LONG, AN_IND, 4, DN, 4),

        *movea(LONG, SPECIAL, IMMEDIATE, 6),
        *imm_long(tt0_src),
        0xF016, 0x0800,                       # PMOVE (A6),TT0
        *movea(LONG, SPECIAL, IMMEDIATE, 7),
        *imm_long(tt1_src),
        0xF017, 0x0C00,                       # PMOVE (A7),TT1

        *movea(LONG, SPECIAL, IMMEDIATE, 1),
        *imm_long(crp_src),
        0xF011, 0x4C00,                       # PMOVE (A1),CRP
        *movea(LONG, SPECIAL, IMMEDIATE, 2),
        *imm_long(tc_src),
        0xF012, 0x4000,                       # PMOVE (A2),TC

        *movea(LONG, SPECIAL, IMMEDIATE, 0),  # A0 = logical WP page
        *imm_long(logical_addr),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 1),
        *imm_long(0x55AA7733),
        *move(LONG, DN, 1, AN_IND, 0),        # Write should fault via MMU WP.

        *moveq(0x11, 1),                      # Must not execute.
        *move_to_abs_long(LONG, DN, 1, h.RESULT_BASE),
        *h.sentinel_program(),
    ]

    await h.setup(program)
    h.mem.load_long(vector_addr, handler_addr)
    h.mem.load_words(handler_addr, handler_code)
    h.mem.load_long(desc_a_addr, desc_a)
    h.mem.load_long(desc_b_addr, desc_b)
    h.mem.load_long(desc_c_addr, desc_c_wp)
    h.mem.load_long(tt0_src, tt0_prog)
    h.mem.load_long(tt1_src, tt1_prog)
    h.mem.load_long(crp_src + 0, crp_hi)
    h.mem.load_long(crp_src + 4, crp_lo)
    h.mem.load_long(tc_src, tc_val)

    found = await h.run_until_sentinel(max_cycles=80000)
    assert found, "Runtime short-table WP MMU fault test did not complete"

    marker = h.read_result_long(0)
    assert marker == 0x38, f"Expected MMU configuration vector marker 0x38, got 0x{marker:08X}"
    h.cleanup()


@cocotb.test()
async def test_mmu_runtime_long_descriptor_table_walk(dut):
    """Runtime MMU translates through long-format descriptor tables on ATC miss."""
    h = CPUTestHarness(dut)

    logical_addr = 0x12345008
    page_base = 0x00456000
    expected_phys_addr = page_base + (logical_addr & 0xFFF)
    expected_value = 0xA55A96E7
    logical_value = 0x10203040

    root_tbl = 0x00000400
    lvla_tbl = 0x00001000
    lvlb_tbl = 0x00001400

    desc_a_addr = root_tbl + (0x12 << 3)
    desc_b_addr = lvla_tbl + (0x34 << 3)
    desc_c_addr = lvlb_tbl + (0x5 << 3)

    # Long table descriptors: limit disabled (upper = 0x7FFF), DT=3 (next table is long format).
    desc_tbl_lo = 0x7FFF0003
    desc_a_hi = lvla_tbl
    desc_b_hi = lvlb_tbl

    # Long page descriptor: DT=1.
    desc_c_lo = 0x00000001
    desc_c_hi = page_base

    tt0_src = h.DATA_BASE + 0x820
    tt1_src = h.DATA_BASE + 0x840
    crp_src = h.DATA_BASE + 0x860
    tc_src = h.DATA_BASE + 0x880

    tt0_prog = 0x00008360
    tt1_prog = 0x00008350
    crp_hi = 0x7FFF0003  # Root DT=3 (long descriptors), upper-limit mode.
    crp_lo = root_tbl
    tc_val = 0x80C08840  # E=1, PS=12, IS=0, TIA=8, TIB=8, TIC=4, TID=0.

    program = [
        # Prime descriptor shadow with explicit long reads while MMU is disabled.
        *movea(LONG, SPECIAL, IMMEDIATE, 2),
        *imm_long(desc_a_addr + 0),
        *move(LONG, AN_IND, 2, DN, 2),
        *movea(LONG, SPECIAL, IMMEDIATE, 2),
        *imm_long(desc_a_addr + 4),
        *move(LONG, AN_IND, 2, DN, 2),
        *movea(LONG, SPECIAL, IMMEDIATE, 2),
        *imm_long(desc_b_addr + 0),
        *move(LONG, AN_IND, 2, DN, 2),
        *movea(LONG, SPECIAL, IMMEDIATE, 2),
        *imm_long(desc_b_addr + 4),
        *move(LONG, AN_IND, 2, DN, 2),
        *movea(LONG, SPECIAL, IMMEDIATE, 2),
        *imm_long(desc_c_addr + 0),
        *move(LONG, AN_IND, 2, DN, 2),
        *movea(LONG, SPECIAL, IMMEDIATE, 2),
        *imm_long(desc_c_addr + 4),
        *move(LONG, AN_IND, 2, DN, 2),

        # Keep low-address supervisor traffic transparent.
        *movea(LONG, SPECIAL, IMMEDIATE, 6),
        *imm_long(tt0_src),
        0xF016, 0x0800,                       # PMOVE (A6),TT0
        *movea(LONG, SPECIAL, IMMEDIATE, 7),
        *imm_long(tt1_src),
        0xF017, 0x0C00,                       # PMOVE (A7),TT1

        # Install CRP and TC (enables MMU table walk).
        *movea(LONG, SPECIAL, IMMEDIATE, 1),
        *imm_long(crp_src),
        0xF011, 0x4C00,                       # PMOVE (A1),CRP
        *movea(LONG, SPECIAL, IMMEDIATE, 2),
        *imm_long(tc_src),
        0xF012, 0x4000,                       # PMOVE (A2),TC

        *movea(LONG, SPECIAL, IMMEDIATE, 0),  # A0 = logical address
        *imm_long(logical_addr),
        *move(LONG, AN_IND, 0, DN, 1),        # D1 = translated load
        *move_to_abs_long(LONG, DN, 1, h.RESULT_BASE),
        *h.sentinel_program(),
    ]

    await h.setup(program)
    h.mem.load_long(desc_a_addr + 0, desc_tbl_lo)
    h.mem.load_long(desc_a_addr + 4, desc_a_hi)
    h.mem.load_long(desc_b_addr + 0, desc_tbl_lo)
    h.mem.load_long(desc_b_addr + 4, desc_b_hi)
    h.mem.load_long(desc_c_addr + 0, desc_c_lo)
    h.mem.load_long(desc_c_addr + 4, desc_c_hi)

    h.mem.load_long(expected_phys_addr & 0xFFFFF, expected_value)
    h.mem.load_long(logical_addr & 0xFFFFF, logical_value)
    h.mem.load_long(tt0_src, tt0_prog)
    h.mem.load_long(tt1_src, tt1_prog)
    h.mem.load_long(crp_src + 0, crp_hi)
    h.mem.load_long(crp_src + 4, crp_lo)
    h.mem.load_long(tc_src, tc_val)

    found = await h.run_until_sentinel(max_cycles=120000)
    assert found, "Runtime long-table descriptor MMU translation test did not complete"

    got = h.read_result_long(0)
    assert got == expected_value, (
        f"Expected translated value 0x{expected_value:08X}, got 0x{got:08X}"
    )
    h.cleanup()


@cocotb.test()
async def test_mmu_runtime_fcl_first_level_lookup(dut):
    """FCL=1 uses function-code index at the first table level and ignores root LIMIT."""
    h = CPUTestHarness(dut)

    logical_addr = 0x12345008
    page_base = 0x00278000
    expected_phys_addr = page_base + (logical_addr & 0xFFF)
    expected_value = 0x5A1122CC
    logical_value = 0xDEADBEEF

    root_tbl = 0x00002000
    lvla_tbl = 0x00002400
    lvlb_tbl = 0x00002800
    lvlc_tbl = 0x00002C00

    # FC lookup uses supervisor-data function code (FC=5) for data accesses after reset.
    desc_fc_addr = root_tbl + (5 << 2)
    desc_a_addr = lvla_tbl + (0x12 << 2)
    desc_b_addr = lvlb_tbl + (0x34 << 2)
    desc_c_addr = lvlc_tbl + (0x5 << 2)

    desc_fc = (lvla_tbl & 0xFFFFFFF0) | 0x00000002  # DT=2 short table descriptor
    desc_a = (lvlb_tbl & 0xFFFFFFF0) | 0x00000002
    desc_b = (lvlc_tbl & 0xFFFFFFF0) | 0x00000002
    desc_c = (page_base & 0xFFFFFF00) | 0x00000001  # DT=1 short page descriptor

    tt0_src = h.DATA_BASE + 0x8A0
    tt1_src = h.DATA_BASE + 0x8C0
    crp_src = h.DATA_BASE + 0x8E0
    tc_src = h.DATA_BASE + 0x900

    tt0_prog = 0x00008360
    tt1_prog = 0x00008350
    crp_hi = 0x00000002  # DT=2, root limit intentionally restrictive if applied.
    crp_lo = root_tbl
    tc_val = 0x81C08840  # E=1, FCL=1, PS=12, IS=0, TIA=8, TIB=8, TIC=4, TID=0.

    program = [
        # Prime descriptor shadow while MMU is disabled.
        *movea(LONG, SPECIAL, IMMEDIATE, 2),
        *imm_long(desc_fc_addr),
        *move(LONG, AN_IND, 2, DN, 2),
        *movea(LONG, SPECIAL, IMMEDIATE, 2),
        *imm_long(desc_a_addr),
        *move(LONG, AN_IND, 2, DN, 2),
        *movea(LONG, SPECIAL, IMMEDIATE, 2),
        *imm_long(desc_b_addr),
        *move(LONG, AN_IND, 2, DN, 2),
        *movea(LONG, SPECIAL, IMMEDIATE, 2),
        *imm_long(desc_c_addr),
        *move(LONG, AN_IND, 2, DN, 2),

        # Keep low-address supervisor traffic transparent.
        *movea(LONG, SPECIAL, IMMEDIATE, 6),
        *imm_long(tt0_src),
        0xF016, 0x0800,                       # PMOVE (A6),TT0
        *movea(LONG, SPECIAL, IMMEDIATE, 7),
        *imm_long(tt1_src),
        0xF017, 0x0C00,                       # PMOVE (A7),TT1

        *movea(LONG, SPECIAL, IMMEDIATE, 1),
        *imm_long(crp_src),
        0xF011, 0x4C00,                       # PMOVE (A1),CRP
        *movea(LONG, SPECIAL, IMMEDIATE, 2),
        *imm_long(tc_src),
        0xF012, 0x4000,                       # PMOVE (A2),TC

        *movea(LONG, SPECIAL, IMMEDIATE, 0),  # A0 = logical address
        *imm_long(logical_addr),
        *move(LONG, AN_IND, 0, DN, 1),        # D1 = translated load
        *move_to_abs_long(LONG, DN, 1, h.RESULT_BASE),
        *h.sentinel_program(),
    ]

    await h.setup(program)
    h.mem.load_long(desc_fc_addr, desc_fc)
    h.mem.load_long(desc_a_addr, desc_a)
    h.mem.load_long(desc_b_addr, desc_b)
    h.mem.load_long(desc_c_addr, desc_c)

    h.mem.load_long(expected_phys_addr & 0xFFFFF, expected_value)
    h.mem.load_long(logical_addr & 0xFFFFF, logical_value)
    h.mem.load_long(tt0_src, tt0_prog)
    h.mem.load_long(tt1_src, tt1_prog)
    h.mem.load_long(crp_src + 0, crp_hi)
    h.mem.load_long(crp_src + 4, crp_lo)
    h.mem.load_long(tc_src, tc_val)

    found = await h.run_until_sentinel(max_cycles=130000)
    assert found, "Runtime FCL MMU translation test did not complete"

    got = h.read_result_long(0)
    assert got == expected_value, (
        f"Expected FCL-translated value 0x{expected_value:08X}, got 0x{got:08X}"
    )
    h.cleanup()


@cocotb.test()
async def test_mmu_runtime_long_indirect_descriptor(dut):
    """Bottom-level long-format indirect descriptor resolves to a long page descriptor."""
    h = CPUTestHarness(dut)

    logical_addr = 0x12345008
    page_base = 0x00567000
    expected_phys_addr = page_base + (logical_addr & 0xFFF)
    expected_value = 0xC001D00D
    logical_value = 0x0BADCAFE

    root_tbl = 0x00003000
    lvla_tbl = 0x00003400
    lvlb_tbl = 0x00003800
    ind_page_desc = 0x00003C00

    desc_a_addr = root_tbl + (0x12 << 3)
    desc_b_addr = lvla_tbl + (0x34 << 3)
    desc_c_addr = lvlb_tbl + (0x5 << 3)

    desc_tbl_lo = 0x7FFF0003  # Long table descriptor, DT=3
    desc_a_hi = lvla_tbl
    desc_b_hi = lvlb_tbl

    # Long indirect descriptor at bottom level.
    desc_c_lo = 0x00000003  # DT=3 => indirect to long descriptor
    desc_c_hi = ind_page_desc

    # Referenced long page descriptor.
    ind_page_lo = 0x00000001
    ind_page_hi = page_base

    tt0_src = h.DATA_BASE + 0x920
    tt1_src = h.DATA_BASE + 0x940
    crp_src = h.DATA_BASE + 0x960
    tc_src = h.DATA_BASE + 0x980

    tt0_prog = 0x00008360
    tt1_prog = 0x00008350
    crp_hi = 0x7FFF0003
    crp_lo = root_tbl
    tc_val = 0x80C08840

    program = [
        # Prime descriptor shadow while MMU is disabled.
        *movea(LONG, SPECIAL, IMMEDIATE, 2),
        *imm_long(desc_a_addr + 0),
        *move(LONG, AN_IND, 2, DN, 2),
        *movea(LONG, SPECIAL, IMMEDIATE, 2),
        *imm_long(desc_a_addr + 4),
        *move(LONG, AN_IND, 2, DN, 2),
        *movea(LONG, SPECIAL, IMMEDIATE, 2),
        *imm_long(desc_b_addr + 0),
        *move(LONG, AN_IND, 2, DN, 2),
        *movea(LONG, SPECIAL, IMMEDIATE, 2),
        *imm_long(desc_b_addr + 4),
        *move(LONG, AN_IND, 2, DN, 2),
        *movea(LONG, SPECIAL, IMMEDIATE, 2),
        *imm_long(desc_c_addr + 0),
        *move(LONG, AN_IND, 2, DN, 2),
        *movea(LONG, SPECIAL, IMMEDIATE, 2),
        *imm_long(desc_c_addr + 4),
        *move(LONG, AN_IND, 2, DN, 2),
        *movea(LONG, SPECIAL, IMMEDIATE, 2),
        *imm_long(ind_page_desc + 0),
        *move(LONG, AN_IND, 2, DN, 2),
        *movea(LONG, SPECIAL, IMMEDIATE, 2),
        *imm_long(ind_page_desc + 4),
        *move(LONG, AN_IND, 2, DN, 2),

        *movea(LONG, SPECIAL, IMMEDIATE, 6),
        *imm_long(tt0_src),
        0xF016, 0x0800,                       # PMOVE (A6),TT0
        *movea(LONG, SPECIAL, IMMEDIATE, 7),
        *imm_long(tt1_src),
        0xF017, 0x0C00,                       # PMOVE (A7),TT1

        *movea(LONG, SPECIAL, IMMEDIATE, 1),
        *imm_long(crp_src),
        0xF011, 0x4C00,                       # PMOVE (A1),CRP
        *movea(LONG, SPECIAL, IMMEDIATE, 2),
        *imm_long(tc_src),
        0xF012, 0x4000,                       # PMOVE (A2),TC

        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(logical_addr),
        *move(LONG, AN_IND, 0, DN, 1),
        *move_to_abs_long(LONG, DN, 1, h.RESULT_BASE),
        *h.sentinel_program(),
    ]

    await h.setup(program)
    h.mem.load_long(desc_a_addr + 0, desc_tbl_lo)
    h.mem.load_long(desc_a_addr + 4, desc_a_hi)
    h.mem.load_long(desc_b_addr + 0, desc_tbl_lo)
    h.mem.load_long(desc_b_addr + 4, desc_b_hi)
    h.mem.load_long(desc_c_addr + 0, desc_c_lo)
    h.mem.load_long(desc_c_addr + 4, desc_c_hi)
    h.mem.load_long(ind_page_desc + 0, ind_page_lo)
    h.mem.load_long(ind_page_desc + 4, ind_page_hi)

    h.mem.load_long(expected_phys_addr & 0xFFFFF, expected_value)
    h.mem.load_long(logical_addr & 0xFFFFF, logical_value)
    h.mem.load_long(tt0_src, tt0_prog)
    h.mem.load_long(tt1_src, tt1_prog)
    h.mem.load_long(crp_src + 0, crp_hi)
    h.mem.load_long(crp_src + 4, crp_lo)
    h.mem.load_long(tc_src, tc_val)

    found = await h.run_until_sentinel(max_cycles=150000)
    assert found, "Runtime long-indirect MMU translation test did not complete"

    got = h.read_result_long(0)
    assert got == expected_value, (
        f"Expected long-indirect translated value 0x{expected_value:08X}, got 0x{got:08X}"
    )
    h.cleanup()


@cocotb.test()
async def test_mmu_runtime_early_termination_contiguous_reads(dut):
    """Early-termination page descriptor maps a contiguous region correctly."""
    h = CPUTestHarness(dut)

    root_tbl = 0x00006000
    page_base = 0x00030000
    desc_addr = root_tbl + (0x12 << 2)
    desc = (page_base & 0xFFFFFF00) | 0x00000001  # Short early-termination page descriptor

    logical_addrs = [
        0x12001000,
        0x12002004,
        0x12003008,
        0x1200F00C,
        0x12010010,
    ]
    expected_vals = [
        0x01020304,
        0x11121314,
        0x21222324,
        0x31323334,
        0x41424344,
    ]

    tt0_src = h.DATA_BASE + 0x9A0
    tt1_src = h.DATA_BASE + 0x9C0
    crp_src = h.DATA_BASE + 0x9E0
    tc_src = h.DATA_BASE + 0xA00

    tt0_prog = 0x00008360
    tt1_prog = 0x00008350
    crp_hi = 0x7FFF0002
    crp_lo = root_tbl
    tc_val = 0x80C08840

    program = [
        # Prime descriptor shadow before MMU enable.
        *movea(LONG, SPECIAL, IMMEDIATE, 2),
        *imm_long(desc_addr),
        *move(LONG, AN_IND, 2, DN, 2),

        *movea(LONG, SPECIAL, IMMEDIATE, 6),
        *imm_long(tt0_src),
        0xF016, 0x0800,                       # PMOVE (A6),TT0
        *movea(LONG, SPECIAL, IMMEDIATE, 7),
        *imm_long(tt1_src),
        0xF017, 0x0C00,                       # PMOVE (A7),TT1

        *movea(LONG, SPECIAL, IMMEDIATE, 1),
        *imm_long(crp_src),
        0xF011, 0x4C00,                       # PMOVE (A1),CRP
        *movea(LONG, SPECIAL, IMMEDIATE, 2),
        *imm_long(tc_src),
        0xF012, 0x4000,                       # PMOVE (A2),TC

        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(logical_addrs[0]),
        *move(LONG, AN_IND, 0, DN, 1),
        *move_to_abs_long(LONG, DN, 1, h.RESULT_BASE + 0),

        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(logical_addrs[1]),
        *move(LONG, AN_IND, 0, DN, 2),
        *move_to_abs_long(LONG, DN, 2, h.RESULT_BASE + 4),

        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(logical_addrs[2]),
        *move(LONG, AN_IND, 0, DN, 3),
        *move_to_abs_long(LONG, DN, 3, h.RESULT_BASE + 8),

        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(logical_addrs[3]),
        *move(LONG, AN_IND, 0, DN, 4),
        *move_to_abs_long(LONG, DN, 4, h.RESULT_BASE + 12),

        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(logical_addrs[4]),
        *move(LONG, AN_IND, 0, DN, 5),
        *move_to_abs_long(LONG, DN, 5, h.RESULT_BASE + 16),

        *h.sentinel_program(),
    ]

    await h.setup(program)
    h.mem.load_long(desc_addr, desc)
    h.mem.load_long(tt0_src, tt0_prog)
    h.mem.load_long(tt1_src, tt1_prog)
    h.mem.load_long(crp_src + 0, crp_hi)
    h.mem.load_long(crp_src + 4, crp_lo)
    h.mem.load_long(tc_src, tc_val)

    for la, val in zip(logical_addrs, expected_vals):
        pa = (page_base + (la & 0x00FFFFFF)) & 0xFFFFF
        h.mem.load_long(pa, val)

    found = await h.run_until_sentinel(max_cycles=180000)
    assert found, "MMU early-termination contiguous-read stress test did not complete"

    for idx, expected in enumerate(expected_vals):
        got = h.read_result_long(idx * 4)
        assert got == expected, (
            f"Contiguous mapping mismatch at idx {idx}: expected 0x{expected:08X}, got 0x{got:08X}"
        )
    h.cleanup()


@cocotb.test()
async def test_mmu_flush_side_effect_fd_and_pflush(dut):
    """ATC flush strobe: PMOVE FD=0 and PFLUSH flush; PMOVEFD does not flush."""
    h = CPUTestHarness(dut)

    tt0_src = h.DATA_BASE + 0x3C0
    tt1_src = h.DATA_BASE + 0x3E0
    tt0_val = 0x00008260
    tt1_val = 0x00008350

    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),  # A0 = TT0 source
        *imm_long(tt0_src),
        0xF010, 0x0800,                       # PMOVE (A0),TT0      -> flush
        *movea(LONG, SPECIAL, IMMEDIATE, 1),  # A1 = TT1 source
        *imm_long(tt1_src),
        0xF011, 0x0D00,                       # PMOVEFD (A1),TT1    -> no flush
        0xF000, 0x2400,                       # PFLUSHA             -> flush
        *h.sentinel_program(),
    ]

    await h.setup(program)
    h.mem.load_long(tt0_src, tt0_val)
    h.mem.load_long(tt1_src, tt1_val)

    flush_before = int(dut.MMU_ATC_FLUSH_COUNT.value)
    found = await h.run_until_sentinel(max_cycles=14000)
    assert found, "MMU flush side-effect test did not complete"
    flush_after = int(dut.MMU_ATC_FLUSH_COUNT.value)

    assert flush_after - flush_before == 2, (
        f"Expected exactly 2 flush strobes (PMOVE FD=0 + PFLUSHA), got {flush_after - flush_before}"
    )
    h.cleanup()


@cocotb.test()
async def test_mmu_privilege_violation_user_mode(dut):
    """User mode execution of MMU instruction should trap to privilege vector 8."""
    h = CPUTestHarness(dut)
    handler_addr = 0x000640
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
        *movea(LONG, SPECIAL, IMMEDIATE, 0),  # A0 = test address.
        *imm_long(h.DATA_BASE),
        0xF010, 0x8211,                       # PTESTR #1,(A0),#0 (privileged)
        *move_to_abs_long(LONG, DN, 1, h.RESULT_BASE),
        *h.sentinel_program(),
    ]

    await h.setup(program)
    h.mem.load_long(vector_addr, handler_addr)
    h.mem.load_words(handler_addr, handler_code)

    found = await h.run_until_sentinel(max_cycles=8000)
    assert found, "Privilege-violation MMU test did not complete"

    marker = h.read_result_long(0)
    assert marker == 0x08, f"Expected privilege vector handler marker 0x08, got 0x{marker:08X}"
    h.cleanup()
