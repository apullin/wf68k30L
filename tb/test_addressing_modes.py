"""
Addressing mode compliance tests for WF68K30L.

Systematically tests all MC68030 addressing modes using MOVE.L as the vehicle
instruction.  Each test pre-loads memory or registers, executes one or more
MOVE operations through the addressing mode under test, then verifies the
result in the RESULT_BASE area.

Addressing modes tested:
  Mode 000 (Dn)            - data register direct  (sanity)
  Mode 001 (An)            - address register direct (sanity)
  Mode 010 (An)            - address register indirect
  Mode 011 (An)+           - post-increment
  Mode 100 -(An)           - pre-decrement
  Mode 101 (d16,An)        - displacement
  Mode 110 (d8,An,Xn)      - indexed with displacement
  Mode 111/000 (abs).W     - absolute short
  Mode 111/001 (abs).L     - absolute long
  Mode 111/010 (d16,PC)    - PC-relative displacement
  Mode 111/011 (d8,PC,Xn)  - PC-relative indexed
  Mode 111/100 #imm        - immediate

Each test uses a consistent result-store pattern:
  - Load RESULT_BASE into A0 early with MOVEA.L #addr, A0
  - Store results via MOVE.L Dn, (A0) (single-word, no extension)
  - Advance A0 via ADDQ.L #4, A0
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles

from cpu_harness import CPUTestHarness
from m68k_encode import (
    BYTE, WORD, LONG,
    DN, AN, AN_IND, AN_POSTINC, AN_PREDEC, AN_DISP, AN_IDX, SPECIAL,
    ABS_W, ABS_L, PC_DISP, PC_IDX, IMMEDIATE,
    moveq, move, movea, move_to_abs_long, nop, addq, subq,
    move_from_ccr, move_to_ccr,
    imm_long, imm_word, imm_byte, disp16, abs_word, abs_long,
    lea, _w, _signed_byte,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _setup_a0(h):
    """Return instruction words: MOVEA.L #RESULT_BASE, A0."""
    return [*movea(LONG, SPECIAL, IMMEDIATE, 0), *imm_long(h.RESULT_BASE)]


def _store_dn_and_advance(dn):
    """MOVE.L Dn,(A0) ; ADDQ.L #4,A0."""
    return [*move(LONG, DN, dn, AN_IND, 0), *addq(LONG, 4, AN, 0)]


def _store_an_via_dn(an, dn):
    """Move An to Dn then store Dn via A0-indirect and advance.

    Useful for capturing address register values without clobbering A0.
    """
    return [
        *move(LONG, AN, an, DN, dn),
        *move(LONG, DN, dn, AN_IND, 0),
        *addq(LONG, 4, AN, 0),
    ]


def _brief_extension_word(da, reg, wl, scale, disp8):
    """Build a brief format extension word for indexed addressing modes.

    Format: D/A | register | W/L | scale | 0 | displacement
      bit 15:    D/A (0=Dn, 1=An)
      bits 14-12: register number
      bit 11:    W/L (0=sign-extend word index, 1=long index)
      bits 10-9: scale (00=1, 01=2, 10=4, 11=8)
      bit 8:     0 (brief format)
      bits 7-0:  8-bit signed displacement
    """
    return _w(
        (da << 15) |
        ((reg & 7) << 12) |
        (wl << 11) |
        ((scale & 3) << 9) |
        (0 << 8) |
        (disp8 & 0xFF)
    )


# =========================================================================
# Mode 000: Dn (Data Register Direct) -- sanity
# =========================================================================

@cocotb.test()
async def test_dn_move_long(dut):
    """MOVE.L D1,D2 -- data register direct (sanity)."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 1), *imm_long(0xCAFEBABE),
        *nop(),
        *move(LONG, DN, 1, DN, 2),
        *_store_dn_and_advance(2),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    assert h.read_result_long(0) == 0xCAFEBABE
    h.cleanup()


@cocotb.test()
async def test_dn_move_byte(dut):
    """MOVE.B D3,D4 -- byte copy preserves upper bytes of D4."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 4), *imm_long(0x11223344),
        *nop(),
        *moveq(0x77, 3),
        *move(BYTE, DN, 3, DN, 4),
        *_store_dn_and_advance(4),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    assert h.read_result_long(0) == 0x11223377
    h.cleanup()


# =========================================================================
# Mode 001: An (Address Register Direct) -- sanity
# =========================================================================

@cocotb.test()
async def test_an_movea_long(dut):
    """MOVEA.L #imm,A2 then MOVE.L A2,D0 -- address register direct."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        *movea(LONG, SPECIAL, IMMEDIATE, 2), *imm_long(0x00ABCDEF),
        *move(LONG, AN, 2, DN, 0),
        *_store_dn_and_advance(0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    assert h.read_result_long(0) == 0x00ABCDEF
    h.cleanup()


@cocotb.test()
async def test_an_movea_word_sign_ext(dut):
    """MOVEA.W sign-extends: 0x8000 -> 0xFFFF8000."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 1), *imm_long(0x00008000),
        *nop(),
        *movea(WORD, DN, 1, 2),
        *move(LONG, AN, 2, DN, 0),
        *_store_dn_and_advance(0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    assert h.read_result_long(0) == 0xFFFF8000
    h.cleanup()


# =========================================================================
# Mode 010: (An) -- Address Register Indirect
# =========================================================================

@cocotb.test()
async def test_an_ind_read_long(dut):
    """MOVE.L (A2),D0 -- read long through address register indirect."""
    h = CPUTestHarness(dut)
    # Pre-load data at DATA_BASE
    h.mem.load_long(h.DATA_BASE, 0xDEADBEEF)
    program = [
        *_setup_a0(h),
        *movea(LONG, SPECIAL, IMMEDIATE, 2), *imm_long(h.DATA_BASE),
        *move(LONG, AN_IND, 2, DN, 0),
        *_store_dn_and_advance(0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    assert result == 0xDEADBEEF, f"Expected 0xDEADBEEF, got 0x{result:08X}"
    h.cleanup()


@cocotb.test()
async def test_an_ind_write_long(dut):
    """MOVE.L D0,(A2) -- write long through address register indirect."""
    h = CPUTestHarness(dut)
    target = h.DATA_BASE + 0x100
    program = [
        *_setup_a0(h),
        *movea(LONG, SPECIAL, IMMEDIATE, 2), *imm_long(target),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 1), *imm_long(0x12345678),
        *nop(),
        *move(LONG, DN, 1, AN_IND, 2),       # write D1 to (A2)
        # Now read it back: A3 points to target, read via (A3)
        *movea(LONG, SPECIAL, IMMEDIATE, 3), *imm_long(target),
        *move(LONG, AN_IND, 3, DN, 0),
        *_store_dn_and_advance(0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    assert result == 0x12345678, f"Expected 0x12345678, got 0x{result:08X}"
    h.cleanup()


@cocotb.test()
async def test_an_ind_read_word(dut):
    """MOVE.W (A2),D0 -- read word through address register indirect."""
    h = CPUTestHarness(dut)
    h.mem.load_long(h.DATA_BASE, 0xABCD1234)  # word at DATA_BASE = 0xABCD
    program = [
        *_setup_a0(h),
        *movea(LONG, SPECIAL, IMMEDIATE, 2), *imm_long(h.DATA_BASE),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0), *imm_long(0xFFFFFFFF),
        *nop(),
        *move(WORD, AN_IND, 2, DN, 0),
        *_store_dn_and_advance(0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    # MOVE.W only writes low word, upper word of D0 preserved (0xFFFF)
    assert result == 0xFFFFABCD, f"Expected 0xFFFFABCD, got 0x{result:08X}"
    h.cleanup()


@cocotb.test()
async def test_an_ind_read_byte(dut):
    """MOVE.B (A2),D0 -- read byte through address register indirect."""
    h = CPUTestHarness(dut)
    h.mem.load_long(h.DATA_BASE, 0x42000000)  # byte at DATA_BASE = 0x42
    program = [
        *_setup_a0(h),
        *movea(LONG, SPECIAL, IMMEDIATE, 2), *imm_long(h.DATA_BASE),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0), *imm_long(0xFFFFFFFF),
        *nop(),
        *move(BYTE, AN_IND, 2, DN, 0),
        *_store_dn_and_advance(0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    # MOVE.B only writes low byte, upper 3 bytes of D0 preserved
    assert result == 0xFFFFFF42, f"Expected 0xFFFFFF42, got 0x{result:08X}"
    h.cleanup()


# =========================================================================
# Mode 011: (An)+ -- Post-Increment
# =========================================================================

@cocotb.test()
async def test_postinc_long_read(dut):
    """MOVE.L (A2)+,D0 -- read long, A2 incremented by 4."""
    h = CPUTestHarness(dut)
    h.mem.load_long(h.DATA_BASE, 0x11223344)
    program = [
        *_setup_a0(h),
        *movea(LONG, SPECIAL, IMMEDIATE, 2), *imm_long(h.DATA_BASE),
        *move(LONG, AN_POSTINC, 2, DN, 0),     # D0 = (A2)+
        *_store_dn_and_advance(0),               # store D0
        *_store_an_via_dn(2, 1),                 # store A2 (should be DATA_BASE+4)
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    assert result == 0x11223344, f"Expected 0x11223344, got 0x{result:08X}"
    a2_val = h.read_result_long(4)
    expected_a2 = h.DATA_BASE + 4
    assert a2_val == expected_a2, f"A2: expected 0x{expected_a2:08X}, got 0x{a2_val:08X}"
    h.cleanup()


@cocotb.test()
async def test_postinc_word_read(dut):
    """MOVE.W (A2)+,D0 -- read word, A2 incremented by 2."""
    h = CPUTestHarness(dut)
    h.mem.load_long(h.DATA_BASE, 0xBEEF0000)  # word at DATA_BASE = 0xBEEF
    program = [
        *_setup_a0(h),
        *movea(LONG, SPECIAL, IMMEDIATE, 2), *imm_long(h.DATA_BASE),
        *moveq(0, 0),                            # clear D0
        *move(WORD, AN_POSTINC, 2, DN, 0),       # D0 low word = (A2)+
        *_store_dn_and_advance(0),
        *_store_an_via_dn(2, 1),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    assert (result & 0xFFFF) == 0xBEEF, f"Expected low word 0xBEEF, got 0x{result:08X}"
    a2_val = h.read_result_long(4)
    expected_a2 = h.DATA_BASE + 2
    assert a2_val == expected_a2, f"A2: expected 0x{expected_a2:08X}, got 0x{a2_val:08X}"
    h.cleanup()


@cocotb.test()
async def test_postinc_byte_read(dut):
    """MOVE.B (A2)+,D0 -- read byte, A2 incremented by 1."""
    h = CPUTestHarness(dut)
    h.mem.load_long(h.DATA_BASE, 0xAB000000)  # byte at DATA_BASE = 0xAB
    program = [
        *_setup_a0(h),
        *movea(LONG, SPECIAL, IMMEDIATE, 2), *imm_long(h.DATA_BASE),
        *moveq(0, 0),
        *move(BYTE, AN_POSTINC, 2, DN, 0),
        *_store_dn_and_advance(0),
        *_store_an_via_dn(2, 1),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    assert (result & 0xFF) == 0xAB, f"Expected low byte 0xAB, got 0x{result:08X}"
    a2_val = h.read_result_long(4)
    expected_a2 = h.DATA_BASE + 1
    assert a2_val == expected_a2, f"A2: expected 0x{expected_a2:08X}, got 0x{a2_val:08X}"
    h.cleanup()


@cocotb.test()
async def test_postinc_byte_a7_read(dut):
    """MOVE.B (A7)+,D0 -- stack pointer: byte post-increment is 2, not 1.

    On the MC68030, byte post-increment of A7 (SP) is always 2 to keep
    the stack pointer word-aligned.
    """
    h = CPUTestHarness(dut)
    # Place test byte near top of stack area
    sp_data = h.SSP_INIT - 0x10
    h.mem.load_long(sp_data, 0x99000000)  # byte = 0x99
    program = [
        *_setup_a0(h),
        # Set A7 to our test location
        *movea(LONG, SPECIAL, IMMEDIATE, 7), *imm_long(sp_data),
        *move(BYTE, AN_POSTINC, 7, DN, 0),   # D0 = (A7)+
        *_store_dn_and_advance(0),
        *_store_an_via_dn(7, 1),              # capture A7
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    assert (result & 0xFF) == 0x99, f"Expected low byte 0x99, got 0x{result:08X}"
    a7_val = h.read_result_long(4)
    expected_a7 = sp_data + 2  # byte increment on A7 = 2
    assert a7_val == expected_a7, (
        f"A7: expected 0x{expected_a7:08X} (sp+2), got 0x{a7_val:08X}"
    )
    h.cleanup()


@cocotb.test()
async def test_postinc_sequential_reads(dut):
    """Two consecutive MOVE.L (A2)+,Dn reads consecutive longs."""
    h = CPUTestHarness(dut)
    h.mem.load_long(h.DATA_BASE, 0xAAAAAAAA)
    h.mem.load_long(h.DATA_BASE + 4, 0xBBBBBBBB)
    program = [
        *_setup_a0(h),
        *movea(LONG, SPECIAL, IMMEDIATE, 2), *imm_long(h.DATA_BASE),
        *move(LONG, AN_POSTINC, 2, DN, 1),   # D1 = first long
        *move(LONG, AN_POSTINC, 2, DN, 2),   # D2 = second long
        *_store_dn_and_advance(1),
        *_store_dn_and_advance(2),
        *_store_an_via_dn(2, 3),              # A2 should be DATA_BASE+8
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    assert h.read_result_long(0) == 0xAAAAAAAA
    assert h.read_result_long(4) == 0xBBBBBBBB
    a2_val = h.read_result_long(8)
    assert a2_val == h.DATA_BASE + 8, f"A2: expected 0x{h.DATA_BASE+8:08X}, got 0x{a2_val:08X}"
    h.cleanup()


@cocotb.test()
async def test_postinc_write_long(dut):
    """MOVE.L D1,(A2)+ -- write long, A2 incremented by 4."""
    h = CPUTestHarness(dut)
    target = h.DATA_BASE + 0x200
    program = [
        *_setup_a0(h),
        *movea(LONG, SPECIAL, IMMEDIATE, 2), *imm_long(target),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 1), *imm_long(0xFACEFACE),
        *nop(),
        *move(LONG, DN, 1, AN_POSTINC, 2),      # (A2)+ = D1
        # Read back from target to verify write
        *movea(LONG, SPECIAL, IMMEDIATE, 3), *imm_long(target),
        *move(LONG, AN_IND, 3, DN, 0),
        *_store_dn_and_advance(0),
        *_store_an_via_dn(2, 1),                  # capture A2 after
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    assert result == 0xFACEFACE, f"Expected 0xFACEFACE, got 0x{result:08X}"
    a2_val = h.read_result_long(4)
    assert a2_val == target + 4, f"A2: expected 0x{target+4:08X}, got 0x{a2_val:08X}"
    h.cleanup()


# =========================================================================
# Mode 100: -(An) -- Pre-Decrement
# =========================================================================

@cocotb.test()
async def test_predec_read_long(dut):
    """MOVE.L -(A2),D0 -- A2 decremented by 4 before read."""
    h = CPUTestHarness(dut)
    # Store data 4 bytes below where A2 starts
    data_addr = h.DATA_BASE + 0x04
    h.mem.load_long(h.DATA_BASE, 0x55667788)
    program = [
        *_setup_a0(h),
        *movea(LONG, SPECIAL, IMMEDIATE, 2), *imm_long(data_addr),
        *move(LONG, AN_PREDEC, 2, DN, 0),    # D0 = -(A2)
        *_store_dn_and_advance(0),
        *_store_an_via_dn(2, 1),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    assert result == 0x55667788, f"Expected 0x55667788, got 0x{result:08X}"
    a2_val = h.read_result_long(4)
    assert a2_val == h.DATA_BASE, f"A2: expected 0x{h.DATA_BASE:08X}, got 0x{a2_val:08X}"
    h.cleanup()


@cocotb.test()
async def test_predec_write_long(dut):
    """MOVE.L D1,-(A2) -- push to pre-decremented address."""
    h = CPUTestHarness(dut)
    target = h.DATA_BASE + 0x300
    program = [
        *_setup_a0(h),
        *movea(LONG, SPECIAL, IMMEDIATE, 2), *imm_long(target + 4),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 1), *imm_long(0xDECAFBAD),
        *nop(),
        *move(LONG, DN, 1, AN_PREDEC, 2),    # -(A2) = D1
        # Read back from target
        *movea(LONG, SPECIAL, IMMEDIATE, 3), *imm_long(target),
        *move(LONG, AN_IND, 3, DN, 0),
        *_store_dn_and_advance(0),
        *_store_an_via_dn(2, 1),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    assert result == 0xDECAFBAD, f"Expected 0xDECAFBAD, got 0x{result:08X}"
    a2_val = h.read_result_long(4)
    assert a2_val == target, f"A2: expected 0x{target:08X}, got 0x{a2_val:08X}"
    h.cleanup()


@cocotb.test()
async def test_predec_word(dut):
    """MOVE.W -(A2),D0 -- A2 decremented by 2."""
    h = CPUTestHarness(dut)
    data_addr = h.DATA_BASE + 0x02
    h.mem.load_long(h.DATA_BASE, 0xBEEF0000)  # word at DATA_BASE = 0xBEEF
    program = [
        *_setup_a0(h),
        *movea(LONG, SPECIAL, IMMEDIATE, 2), *imm_long(data_addr),
        *moveq(0, 0),
        *move(WORD, AN_PREDEC, 2, DN, 0),    # D0 low word = -(A2)
        *_store_dn_and_advance(0),
        *_store_an_via_dn(2, 1),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    assert (result & 0xFFFF) == 0xBEEF, f"Expected low word 0xBEEF, got 0x{result:08X}"
    a2_val = h.read_result_long(4)
    assert a2_val == h.DATA_BASE, f"A2: expected 0x{h.DATA_BASE:08X}, got 0x{a2_val:08X}"
    h.cleanup()


@cocotb.test()
async def test_predec_byte_a7(dut):
    """MOVE.B D0,-(A7) -- stack pointer byte pre-decrement is 2."""
    h = CPUTestHarness(dut)
    sp_start = h.SSP_INIT
    program = [
        *_setup_a0(h),
        *movea(LONG, SPECIAL, IMMEDIATE, 7), *imm_long(sp_start),
        *moveq(0x42, 1),
        *move(BYTE, DN, 1, AN_PREDEC, 7),    # -(A7) = D1.B
        *_store_an_via_dn(7, 2),              # capture A7
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    a7_val = h.read_result_long(0)
    expected_a7 = sp_start - 2  # byte on A7: decrement by 2
    assert a7_val == expected_a7, (
        f"A7: expected 0x{expected_a7:08X} (sp-2), got 0x{a7_val:08X}"
    )
    h.cleanup()


@cocotb.test()
async def test_predec_sequential_reads(dut):
    """Two consecutive -(A2) reads: second reads from lower address."""
    h = CPUTestHarness(dut)
    base = h.DATA_BASE
    h.mem.load_long(base, 0x11111111)
    h.mem.load_long(base + 4, 0x22222222)
    program = [
        *_setup_a0(h),
        *movea(LONG, SPECIAL, IMMEDIATE, 2), *imm_long(base + 8),
        *move(LONG, AN_PREDEC, 2, DN, 1),    # D1 = -(A2) -> reads base+4
        *move(LONG, AN_PREDEC, 2, DN, 2),    # D2 = -(A2) -> reads base
        *_store_dn_and_advance(1),
        *_store_dn_and_advance(2),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    assert h.read_result_long(0) == 0x22222222
    assert h.read_result_long(4) == 0x11111111
    h.cleanup()


# =========================================================================
# Mode 101: (d16,An) -- Displacement
# =========================================================================

@cocotb.test()
async def test_disp_positive(dut):
    """MOVE.L (d16,A3),D0 -- positive displacement."""
    h = CPUTestHarness(dut)
    offset = 0x20
    h.mem.load_long(h.DATA_BASE + offset, 0xFEEDFACE)
    # Manually encode: MOVE.L (d16,A3),D0
    # MOVE.L src=(AN_DISP, A3), dst=(DN, D0)
    # opcode: 10 000 000 101 011 = 0x2028 + disp16 word
    opcode = move(LONG, AN_DISP, 3, DN, 0)
    program = [
        *_setup_a0(h),
        *movea(LONG, SPECIAL, IMMEDIATE, 3), *imm_long(h.DATA_BASE),
        *opcode, *disp16(offset),
        *_store_dn_and_advance(0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    assert result == 0xFEEDFACE, f"Expected 0xFEEDFACE, got 0x{result:08X}"
    h.cleanup()


@cocotb.test()
async def test_disp_zero(dut):
    """MOVE.L (0,A3),D0 -- zero displacement (equivalent to (A3))."""
    h = CPUTestHarness(dut)
    h.mem.load_long(h.DATA_BASE, 0xABCDABCD)
    opcode = move(LONG, AN_DISP, 3, DN, 0)
    program = [
        *_setup_a0(h),
        *movea(LONG, SPECIAL, IMMEDIATE, 3), *imm_long(h.DATA_BASE),
        *opcode, *disp16(0),
        *_store_dn_and_advance(0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    assert result == 0xABCDABCD, f"Expected 0xABCDABCD, got 0x{result:08X}"
    h.cleanup()


@cocotb.test()
async def test_disp_negative(dut):
    """MOVE.L (-4,A3),D0 -- negative displacement."""
    h = CPUTestHarness(dut)
    h.mem.load_long(h.DATA_BASE, 0x99887766)
    opcode = move(LONG, AN_DISP, 3, DN, 0)
    program = [
        *_setup_a0(h),
        # Set A3 to DATA_BASE + 4, then use displacement -4 to read DATA_BASE
        *movea(LONG, SPECIAL, IMMEDIATE, 3), *imm_long(h.DATA_BASE + 4),
        *opcode, *disp16(-4),
        *_store_dn_and_advance(0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    assert result == 0x99887766, f"Expected 0x99887766, got 0x{result:08X}"
    h.cleanup()


@cocotb.test()
async def test_disp_write(dut):
    """MOVE.L D1,(d16,A3) -- write through displacement mode."""
    h = CPUTestHarness(dut)
    offset = 0x10
    target = h.DATA_BASE + 0x400 + offset
    opcode = move(LONG, DN, 1, AN_DISP, 3)
    program = [
        *_setup_a0(h),
        *movea(LONG, SPECIAL, IMMEDIATE, 3), *imm_long(h.DATA_BASE + 0x400),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 1), *imm_long(0xCAFE1234),
        *nop(),
        *opcode, *disp16(offset),
        # Read back
        *movea(LONG, SPECIAL, IMMEDIATE, 4), *imm_long(target),
        *move(LONG, AN_IND, 4, DN, 0),
        *_store_dn_and_advance(0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    assert result == 0xCAFE1234, f"Expected 0xCAFE1234, got 0x{result:08X}"
    h.cleanup()


@cocotb.test()
async def test_disp_large_positive(dut):
    """MOVE.L (0x100,A3),D0 -- larger displacement."""
    h = CPUTestHarness(dut)
    offset = 0x100
    h.mem.load_long(h.DATA_BASE + offset, 0x12121212)
    opcode = move(LONG, AN_DISP, 3, DN, 0)
    program = [
        *_setup_a0(h),
        *movea(LONG, SPECIAL, IMMEDIATE, 3), *imm_long(h.DATA_BASE),
        *opcode, *disp16(offset),
        *_store_dn_and_advance(0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    assert result == 0x12121212, f"Expected 0x12121212, got 0x{result:08X}"
    h.cleanup()


# =========================================================================
# Mode 110: (d8,An,Xn) -- Indexed
# =========================================================================

@cocotb.test()
async def test_indexed_dn_long(dut):
    """MOVE.L (d8,A3,D1.L),D0 -- indexed with data register, long index."""
    h = CPUTestHarness(dut)
    disp8 = 8
    idx_val = 0x10
    h.mem.load_long(h.DATA_BASE + disp8 + idx_val, 0xBAADF00D)
    opcode = move(LONG, AN_IDX, 3, DN, 0)
    ext = _brief_extension_word(da=0, reg=1, wl=1, scale=0, disp8=disp8)
    program = [
        *_setup_a0(h),
        *movea(LONG, SPECIAL, IMMEDIATE, 3), *imm_long(h.DATA_BASE),
        *moveq(idx_val, 1),                   # D1 = 0x10
        *nop(), *nop(),
        *opcode, ext,
        *_store_dn_and_advance(0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    assert result == 0xBAADF00D, f"Expected 0xBAADF00D, got 0x{result:08X}"
    h.cleanup()


@cocotb.test()
async def test_indexed_dn_word(dut):
    """MOVE.L (d8,A3,D1.W),D0 -- indexed with word-size index register."""
    h = CPUTestHarness(dut)
    disp8 = 0
    idx_val = 0x10
    h.mem.load_long(h.DATA_BASE + idx_val, 0xC0FFEE00)
    opcode = move(LONG, AN_IDX, 3, DN, 0)
    ext = _brief_extension_word(da=0, reg=1, wl=0, scale=0, disp8=disp8)
    program = [
        *_setup_a0(h),
        *movea(LONG, SPECIAL, IMMEDIATE, 3), *imm_long(h.DATA_BASE),
        *moveq(idx_val, 1),
        *nop(), *nop(),                        # pipeline drain
        *opcode, ext,
        *_store_dn_and_advance(0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    assert result == 0xC0FFEE00, f"Expected 0xC0FFEE00, got 0x{result:08X}"
    h.cleanup()


@cocotb.test()
async def test_indexed_an_long(dut):
    """MOVE.L (d8,A3,A4.L),D0 -- indexed with address register as index."""
    h = CPUTestHarness(dut)
    disp8 = 8
    idx_val = 0x10
    h.mem.load_long(h.DATA_BASE + disp8 + idx_val, 0xFACEFACE)
    opcode = move(LONG, AN_IDX, 3, DN, 0)
    ext = _brief_extension_word(da=1, reg=4, wl=1, scale=0, disp8=disp8)
    program = [
        *_setup_a0(h),
        *movea(LONG, SPECIAL, IMMEDIATE, 3), *imm_long(h.DATA_BASE),
        *movea(LONG, SPECIAL, IMMEDIATE, 4), *imm_long(idx_val),
        *opcode, ext,
        *_store_dn_and_advance(0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    assert result == 0xFACEFACE, f"Expected 0xFACEFACE, got 0x{result:08X}"
    h.cleanup()


@cocotb.test()
async def test_indexed_scale2(dut):
    """MOVE.L (0,A3,D1.L*2),D0 -- index scaled by 2."""
    h = CPUTestHarness(dut)
    idx_val = 4
    # Effective address = A3 + D1*2 + 0 = DATA_BASE + 8
    h.mem.load_long(h.DATA_BASE + idx_val * 2, 0x22334455)
    opcode = move(LONG, AN_IDX, 3, DN, 0)
    ext = _brief_extension_word(da=0, reg=1, wl=1, scale=1, disp8=0)
    program = [
        *_setup_a0(h),
        *movea(LONG, SPECIAL, IMMEDIATE, 3), *imm_long(h.DATA_BASE),
        *moveq(idx_val, 1),
        *nop(), *nop(),                        # pipeline drain
        *opcode, ext,
        *_store_dn_and_advance(0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    assert result == 0x22334455, f"Expected 0x22334455, got 0x{result:08X}"
    h.cleanup()


@cocotb.test()
async def test_indexed_scale4(dut):
    """MOVE.L (0,A3,D1.L*4),D0 -- index scaled by 4 (long table lookup)."""
    h = CPUTestHarness(dut)
    idx_val = 3
    # Effective address = A3 + D1*4 + 0 = DATA_BASE + 12
    h.mem.load_long(h.DATA_BASE + idx_val * 4, 0xAA55AA55)
    opcode = move(LONG, AN_IDX, 3, DN, 0)
    ext = _brief_extension_word(da=0, reg=1, wl=1, scale=2, disp8=0)
    program = [
        *_setup_a0(h),
        *movea(LONG, SPECIAL, IMMEDIATE, 3), *imm_long(h.DATA_BASE),
        *moveq(idx_val, 1),
        *nop(), *nop(),                        # pipeline drain
        *opcode, ext,
        *_store_dn_and_advance(0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    assert result == 0xAA55AA55, f"Expected 0xAA55AA55, got 0x{result:08X}"
    h.cleanup()


@cocotb.test()
async def test_indexed_write(dut):
    """MOVE.L D1,(d8,A3,D2.L) -- write via indexed mode."""
    h = CPUTestHarness(dut)
    disp8 = 0x10
    idx_val = 0x04
    target = h.DATA_BASE + 0x500 + disp8 + idx_val
    opcode = move(LONG, DN, 1, AN_IDX, 3)
    ext = _brief_extension_word(da=0, reg=2, wl=1, scale=0, disp8=disp8)
    program = [
        *_setup_a0(h),
        *movea(LONG, SPECIAL, IMMEDIATE, 3), *imm_long(h.DATA_BASE + 0x500),
        *moveq(idx_val, 2),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 1), *imm_long(0xDEADC0DE),
        *nop(), *nop(),                        # pipeline drain
        *opcode, ext,
        # Read back
        *movea(LONG, SPECIAL, IMMEDIATE, 4), *imm_long(target),
        *move(LONG, AN_IND, 4, DN, 0),
        *_store_dn_and_advance(0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    assert result == 0xDEADC0DE, f"Expected 0xDEADC0DE, got 0x{result:08X}"
    h.cleanup()


@cocotb.test()
async def test_indexed_negative_disp(dut):
    """MOVE.L (-8,A3,D1.L),D0 -- negative 8-bit displacement."""
    h = CPUTestHarness(dut)
    # A3 = DATA_BASE + 0x20, D1 = 0, disp = -8 -> addr = DATA_BASE + 0x18
    target_addr = h.DATA_BASE + 0x18
    h.mem.load_long(target_addr, 0x77889900)
    opcode = move(LONG, AN_IDX, 3, DN, 0)
    ext = _brief_extension_word(da=0, reg=1, wl=1, scale=0, disp8=-8)
    program = [
        *_setup_a0(h),
        *movea(LONG, SPECIAL, IMMEDIATE, 3), *imm_long(h.DATA_BASE + 0x20),
        *moveq(0, 1),
        *nop(), *nop(),                        # pipeline drain
        *opcode, ext,
        *_store_dn_and_advance(0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    assert result == 0x77889900, f"Expected 0x77889900, got 0x{result:08X}"
    h.cleanup()


# =========================================================================
# Mode 111/000: (abs).W -- Absolute Short
# =========================================================================

@cocotb.test()
async def test_abs_w_read(dut):
    """MOVE.L (abs).W,D0 -- read from absolute short address.

    Absolute short is sign-extended, so address must be 0x0000-0x7FFF
    for positive addresses.  Use a low address within memory range.
    """
    h = CPUTestHarness(dut)
    addr = 0x0400  # Safe low address, above vector table and PROGRAM_BASE
    h.mem.load_long(addr, 0xBEEFCAFE)
    # MOVE.L (abs).W, D0 -> move(LONG, SPECIAL, ABS_W, DN, 0) + abs_word
    opcode = move(LONG, SPECIAL, ABS_W, DN, 0)
    program = [
        *_setup_a0(h),
        *opcode, *abs_word(addr),
        *_store_dn_and_advance(0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    assert result == 0xBEEFCAFE, f"Expected 0xBEEFCAFE, got 0x{result:08X}"
    h.cleanup()


@cocotb.test()
async def test_abs_w_write(dut):
    """MOVE.L D1,(abs).W -- write to absolute short address."""
    h = CPUTestHarness(dut)
    addr = 0x0200
    opcode = move(LONG, DN, 1, SPECIAL, ABS_W)
    program = [
        *_setup_a0(h),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 1), *imm_long(0x44556677),
        *nop(),
        *opcode, *abs_word(addr),
        # Read back via (An)
        *movea(LONG, SPECIAL, IMMEDIATE, 3), *imm_long(addr),
        *move(LONG, AN_IND, 3, DN, 0),
        *_store_dn_and_advance(0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    assert result == 0x44556677, f"Expected 0x44556677, got 0x{result:08X}"
    h.cleanup()


@cocotb.test()
async def test_abs_w_word_size(dut):
    """MOVE.W (abs).W,D0 -- word-size read from absolute short."""
    h = CPUTestHarness(dut)
    addr = 0x0300
    h.mem.load_long(addr, 0xFACE0000)  # word at addr = 0xFACE
    opcode = move(WORD, SPECIAL, ABS_W, DN, 0)
    program = [
        *_setup_a0(h),
        *moveq(0, 0),
        *opcode, *abs_word(addr),
        *_store_dn_and_advance(0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    assert (result & 0xFFFF) == 0xFACE, f"Expected low word 0xFACE, got 0x{result:08X}"
    h.cleanup()


# =========================================================================
# Mode 111/001: (abs).L -- Absolute Long
# =========================================================================

@cocotb.test()
async def test_abs_l_read(dut):
    """MOVE.L (abs).L,D0 -- read from absolute long address."""
    h = CPUTestHarness(dut)
    addr = h.DATA_BASE + 0x600
    h.mem.load_long(addr, 0x13579BDF)
    opcode = move(LONG, SPECIAL, ABS_L, DN, 0)
    program = [
        *_setup_a0(h),
        *opcode, *abs_long(addr),
        *_store_dn_and_advance(0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    assert result == 0x13579BDF, f"Expected 0x13579BDF, got 0x{result:08X}"
    h.cleanup()


@cocotb.test()
async def test_abs_l_write(dut):
    """MOVE.L D1,(abs).L -- write to absolute long address.

    NOTE: This uses MOVE.L Dn,(abs).L as destination, which the harness
    notes can have prefetch issues.  This test exists to verify the mode
    itself works; results are read back via register-indirect.
    """
    h = CPUTestHarness(dut)
    addr = h.DATA_BASE + 0x700
    program = [
        *_setup_a0(h),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 1), *imm_long(0x2468ACE0),
        *nop(),
        *move_to_abs_long(LONG, DN, 1, addr),
        # Read back
        *movea(LONG, SPECIAL, IMMEDIATE, 3), *imm_long(addr),
        *move(LONG, AN_IND, 3, DN, 0),
        *_store_dn_and_advance(0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    assert result == 0x2468ACE0, f"Expected 0x2468ACE0, got 0x{result:08X}"
    h.cleanup()


# =========================================================================
# Mode 111/010: (d16,PC) -- PC-Relative Displacement
# =========================================================================

@cocotb.test()
async def test_pc_disp_read(dut):
    """MOVE.L (d16,PC),D0 -- read data using negative PC-relative displacement.

    The displacement is relative to the address of the extension word
    (opcode address + 2).  We place test data at address 0x0080 (below
    PROGRAM_BASE but above vectors) and use a negative displacement to
    reach it.
    """
    h = CPUTestHarness(dut)
    data_val = 0xAAAABBBB
    data_addr = 0x0080  # Below PROGRAM_BASE, within signed 16-bit reach
    h.mem.load_long(data_addr, data_val)

    setup_words = _setup_a0(h)
    pc_at_ext = h.PROGRAM_BASE + len(setup_words) * 2 + 2  # +2 for opcode word
    disp = data_addr - pc_at_ext  # negative displacement

    opcode = move(LONG, SPECIAL, PC_DISP, DN, 0)
    program = [
        *setup_words,
        *opcode, *disp16(disp),
        *_store_dn_and_advance(0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    assert result == data_val, f"Expected 0x{data_val:08X}, got 0x{result:08X}"
    h.cleanup()


@cocotb.test()
async def test_pc_disp_forward(dut):
    """MOVE.L (d16,PC),D0 -- forward displacement to data after program.

    Embed the data value at an address forward from the instruction, still
    within signed 16-bit displacement range.
    """
    h = CPUTestHarness(dut)
    data_val = 0x55AA55AA
    # Place data forward from program at a known offset
    # Programs are typically ~100 bytes, so 0x400 forward is safe
    data_addr = h.PROGRAM_BASE + 0x400
    h.mem.load_long(data_addr, data_val)

    setup_words = _setup_a0(h)
    pc_at_ext = h.PROGRAM_BASE + len(setup_words) * 2 + 2
    disp = data_addr - pc_at_ext  # positive displacement

    opcode = move(LONG, SPECIAL, PC_DISP, DN, 0)
    program = [
        *setup_words,
        *opcode, *disp16(disp),
        *_store_dn_and_advance(0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    assert result == data_val, f"Expected 0x{data_val:08X}, got 0x{result:08X}"
    h.cleanup()


@cocotb.test()
async def test_pc_disp_word_read(dut):
    """MOVE.W (d16,PC),D0 -- word-size PC-relative read."""
    h = CPUTestHarness(dut)
    data_val = 0x1234
    data_addr = 0x00C0  # Below PROGRAM_BASE, within reach
    h.mem.load_long(data_addr, (data_val << 16))  # word in upper 16 bits of long

    setup_words = _setup_a0(h)
    # After setup, we do MOVEQ then the MOVE.W PC-relative
    moveq_words = moveq(0, 0)
    pc_at_ext = h.PROGRAM_BASE + (len(setup_words) + len(moveq_words)) * 2 + 2
    disp = data_addr - pc_at_ext

    opcode = move(WORD, SPECIAL, PC_DISP, DN, 0)
    program = [
        *setup_words,
        *moveq_words,
        *opcode, *disp16(disp),
        *_store_dn_and_advance(0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    assert (result & 0xFFFF) == data_val, f"Expected low word 0x{data_val:04X}, got 0x{result:08X}"
    h.cleanup()


# =========================================================================
# Mode 111/011: (d8,PC,Xn) -- PC-Relative Indexed
# =========================================================================

@cocotb.test()
async def test_pc_idx_read(dut):
    """MOVE.L (d8,PC,D1.L),D0 -- PC-relative with data register index.

    Place data at an address reachable by PC + d8 + D1.L.
    We use d8=0 and D1 holds the full offset from PC to data.
    Data is placed at PROGRAM_BASE + 0x400 (forward, within range).
    """
    h = CPUTestHarness(dut)
    data_val = 0xDECADE00
    data_addr = h.PROGRAM_BASE + 0x400
    h.mem.load_long(data_addr, data_val)

    setup_words = _setup_a0(h)
    # D1 will be loaded via MOVE.L #imm (3 words), then NOP NOP (2 words)
    load_idx_placeholder = [*move(LONG, SPECIAL, IMMEDIATE, DN, 1), *imm_long(0)]
    nop_pad = [*nop(), *nop()]
    words_before_opcode = len(setup_words) + len(load_idx_placeholder) + len(nop_pad)
    pc_at_ext = h.PROGRAM_BASE + words_before_opcode * 2 + 2
    disp8 = 0
    idx_val = data_addr - pc_at_ext - disp8

    load_idx = [*move(LONG, SPECIAL, IMMEDIATE, DN, 1), *imm_long(idx_val)]
    # Verify sizes are stable
    assert len(load_idx) == len(load_idx_placeholder), "Size mismatch"

    opcode = move(LONG, SPECIAL, PC_IDX, DN, 0)
    ext = _brief_extension_word(da=0, reg=1, wl=1, scale=0, disp8=disp8)
    program = [
        *setup_words,
        *load_idx,
        *nop_pad,                              # pipeline drain for D1
        *opcode, ext,
        *_store_dn_and_advance(0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    assert result == data_val, f"Expected 0x{data_val:08X}, got 0x{result:08X}"
    h.cleanup()


@cocotb.test()
async def test_pc_idx_with_disp(dut):
    """MOVE.L (d8,PC,D1.L),D0 -- PC-relative indexed with nonzero d8.

    Place data at PROGRAM_BASE + 0x500.  Split the offset into d8=0x10
    and the remainder in D1.
    """
    h = CPUTestHarness(dut)
    data_val = 0x0BADF00D
    data_addr = h.PROGRAM_BASE + 0x500
    h.mem.load_long(data_addr, data_val)

    setup_words = _setup_a0(h)
    load_idx_placeholder = [*move(LONG, SPECIAL, IMMEDIATE, DN, 1), *imm_long(0)]
    nop_pad = [*nop(), *nop()]
    words_before_opcode = len(setup_words) + len(load_idx_placeholder) + len(nop_pad)
    pc_at_ext = h.PROGRAM_BASE + words_before_opcode * 2 + 2

    disp8 = 0x10
    idx_val = data_addr - pc_at_ext - disp8
    load_idx = [*move(LONG, SPECIAL, IMMEDIATE, DN, 1), *imm_long(idx_val)]
    assert len(load_idx) == len(load_idx_placeholder)

    opcode = move(LONG, SPECIAL, PC_IDX, DN, 0)
    ext = _brief_extension_word(da=0, reg=1, wl=1, scale=0, disp8=disp8)
    program = [
        *setup_words,
        *load_idx,
        *nop_pad,
        *opcode, ext,
        *_store_dn_and_advance(0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    assert result == data_val, f"Expected 0x{data_val:08X}, got 0x{result:08X}"
    h.cleanup()


# =========================================================================
# Mode 111/100: #imm -- Immediate
# =========================================================================

@cocotb.test()
async def test_imm_long(dut):
    """MOVE.L #imm,D0 -- immediate long."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0), *imm_long(0x12345678),
        *nop(),
        *_store_dn_and_advance(0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    assert h.read_result_long(0) == 0x12345678
    h.cleanup()


@cocotb.test()
async def test_imm_word(dut):
    """MOVE.W #imm,D0 -- immediate word."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0), *imm_long(0xFFFFFFFF),
        *nop(),
        *move(WORD, SPECIAL, IMMEDIATE, DN, 0), *imm_word(0x5678),
        *_store_dn_and_advance(0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    # MOVE.W replaces low word, preserves upper word
    assert result == 0xFFFF5678, f"Expected 0xFFFF5678, got 0x{result:08X}"
    h.cleanup()


@cocotb.test()
async def test_imm_byte(dut):
    """MOVE.B #imm,D0 -- immediate byte."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0), *imm_long(0xFFFFFFFF),
        *nop(),
        *move(BYTE, SPECIAL, IMMEDIATE, DN, 0), *imm_byte(0x42),
        *_store_dn_and_advance(0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    assert result == 0xFFFFFF42, f"Expected 0xFFFFFF42, got 0x{result:08X}"
    h.cleanup()


@cocotb.test()
async def test_imm_zero(dut):
    """MOVE.L #0,D0 -- immediate zero."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0), *imm_long(0),
        *nop(),
        *_store_dn_and_advance(0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    assert h.read_result_long(0) == 0
    h.cleanup()


@cocotb.test()
async def test_imm_to_an_ind(dut):
    """MOVE.L #imm,(A2) -- immediate to address register indirect."""
    h = CPUTestHarness(dut)
    target = h.DATA_BASE + 0x800
    opcode = move(LONG, SPECIAL, IMMEDIATE, AN_IND, 2)
    program = [
        *_setup_a0(h),
        *movea(LONG, SPECIAL, IMMEDIATE, 2), *imm_long(target),
        *opcode, *imm_long(0xBEEF1234),
        # Read back
        *movea(LONG, SPECIAL, IMMEDIATE, 3), *imm_long(target),
        *move(LONG, AN_IND, 3, DN, 0),
        *_store_dn_and_advance(0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    assert result == 0xBEEF1234, f"Expected 0xBEEF1234, got 0x{result:08X}"
    h.cleanup()


# =========================================================================
# LEA tests (effective address calculation for various modes)
# =========================================================================

@cocotb.test()
async def test_lea_an_ind(dut):
    """LEA (A3),A4 -- loads address in A3 into A4."""
    h = CPUTestHarness(dut)
    program = [
        *_setup_a0(h),
        *movea(LONG, SPECIAL, IMMEDIATE, 3), *imm_long(0x00012345),
        *lea(AN_IND, 3, 4),
        *_store_an_via_dn(4, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    assert result == 0x00012345, f"Expected 0x00012345, got 0x{result:08X}"
    h.cleanup()


@cocotb.test()
async def test_lea_disp(dut):
    """LEA (d16,A3),A4 -- computes A3 + displacement."""
    h = CPUTestHarness(dut)
    base_addr = 0x00010000
    offset = 0x0040
    opcode = lea(AN_DISP, 3, 4)
    program = [
        *_setup_a0(h),
        *movea(LONG, SPECIAL, IMMEDIATE, 3), *imm_long(base_addr),
        *opcode, *disp16(offset),
        *_store_an_via_dn(4, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    expected = base_addr + offset
    assert result == expected, f"Expected 0x{expected:08X}, got 0x{result:08X}"
    h.cleanup()


@cocotb.test()
async def test_lea_indexed(dut):
    """LEA (d8,A3,D1.L),A4 -- computes A3 + D1 + d8."""
    h = CPUTestHarness(dut)
    base_addr = 0x00010000
    disp8 = 0x08
    idx_val = 0x20
    opcode = lea(AN_IDX, 3, 4)
    ext = _brief_extension_word(da=0, reg=1, wl=1, scale=0, disp8=disp8)
    program = [
        *_setup_a0(h),
        *movea(LONG, SPECIAL, IMMEDIATE, 3), *imm_long(base_addr),
        *moveq(idx_val, 1),
        *nop(), *nop(),                        # pipeline drain
        *opcode, ext,
        *_store_an_via_dn(4, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    expected = base_addr + idx_val + disp8
    assert result == expected, f"Expected 0x{expected:08X}, got 0x{result:08X}"
    h.cleanup()


@cocotb.test()
async def test_lea_abs_w(dut):
    """LEA (abs).W,A4 -- loads sign-extended absolute short address."""
    h = CPUTestHarness(dut)
    addr = 0x2000
    opcode = lea(SPECIAL, ABS_W, 4)
    program = [
        *_setup_a0(h),
        *opcode, *abs_word(addr),
        *_store_an_via_dn(4, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    assert result == addr, f"Expected 0x{addr:08X}, got 0x{result:08X}"
    h.cleanup()


@cocotb.test()
async def test_lea_abs_l(dut):
    """LEA (abs).L,A4 -- loads absolute long address."""
    h = CPUTestHarness(dut)
    addr = h.DATA_BASE + 0x900
    opcode = lea(SPECIAL, ABS_L, 4)
    program = [
        *_setup_a0(h),
        *opcode, *abs_long(addr),
        *_store_an_via_dn(4, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    assert result == addr, f"Expected 0x{addr:08X}, got 0x{result:08X}"
    h.cleanup()


# =========================================================================
# Cross-mode compound tests
# =========================================================================

@cocotb.test()
async def test_postinc_to_predec_copy(dut):
    """Copy block using (A2)+ source and -(A3) destination (reverse copy).

    Copy two longs from DATA_BASE to DATA_BASE+0x100 in reverse order.
    """
    h = CPUTestHarness(dut)
    src = h.DATA_BASE
    dst = h.DATA_BASE + 0x100
    h.mem.load_long(src, 0x11111111)
    h.mem.load_long(src + 4, 0x22222222)
    program = [
        *_setup_a0(h),
        *movea(LONG, SPECIAL, IMMEDIATE, 2), *imm_long(src),
        *movea(LONG, SPECIAL, IMMEDIATE, 3), *imm_long(dst + 8),  # end of dest
        *move(LONG, AN_POSTINC, 2, AN_PREDEC, 3),  # copy first long to -(A3)
        *move(LONG, AN_POSTINC, 2, AN_PREDEC, 3),  # copy second long to -(A3)
        # Verify: dst should contain 0x22222222, 0x11111111 (reversed)
        *movea(LONG, SPECIAL, IMMEDIATE, 4), *imm_long(dst),
        *move(LONG, AN_POSTINC, 4, DN, 0),
        *_store_dn_and_advance(0),
        *move(LONG, AN_IND, 4, DN, 1),
        *_store_dn_and_advance(1),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    first = h.read_result_long(0)
    second = h.read_result_long(4)
    assert first == 0x22222222, f"First long: expected 0x22222222, got 0x{first:08X}"
    assert second == 0x11111111, f"Second long: expected 0x11111111, got 0x{second:08X}"
    h.cleanup()


@cocotb.test()
async def test_disp_struct_field_access(dut):
    """Simulate struct field access: base pointer in A3, fields at offsets."""
    h = CPUTestHarness(dut)
    struct_base = h.DATA_BASE + 0xA00
    # Field layout: offset 0 = ID, offset 4 = value, offset 8 = status
    h.mem.load_long(struct_base, 0x00000001)       # ID
    h.mem.load_long(struct_base + 4, 0xDEADBEEF)   # value
    h.mem.load_long(struct_base + 8, 0x000000FF)   # status

    op_f0 = move(LONG, AN_DISP, 3, DN, 1)
    op_f1 = move(LONG, AN_DISP, 3, DN, 2)
    op_f2 = move(LONG, AN_DISP, 3, DN, 3)
    program = [
        *_setup_a0(h),
        *movea(LONG, SPECIAL, IMMEDIATE, 3), *imm_long(struct_base),
        *op_f0, *disp16(0),                  # D1 = struct.ID
        *op_f1, *disp16(4),                  # D2 = struct.value
        *op_f2, *disp16(8),                  # D3 = struct.status
        *_store_dn_and_advance(1),
        *_store_dn_and_advance(2),
        *_store_dn_and_advance(3),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    assert h.read_result_long(0) == 0x00000001
    assert h.read_result_long(4) == 0xDEADBEEF
    assert h.read_result_long(8) == 0x000000FF
    h.cleanup()


@cocotb.test()
async def test_indexed_table_lookup(dut):
    """Array index lookup: base + index*4 to access a table of longs."""
    h = CPUTestHarness(dut)
    table_base = h.DATA_BASE + 0xB00
    # Table: [0x10, 0x20, 0x30, 0x40]
    for i, val in enumerate([0x10, 0x20, 0x30, 0x40]):
        h.mem.load_long(table_base + i * 4, val)

    # Read entry at index 2 (value 0x30) using scale*4
    opcode = move(LONG, AN_IDX, 3, DN, 0)
    ext = _brief_extension_word(da=0, reg=1, wl=1, scale=2, disp8=0)  # scale=2 means *4
    program = [
        *_setup_a0(h),
        *movea(LONG, SPECIAL, IMMEDIATE, 3), *imm_long(table_base),
        *moveq(2, 1),                         # index = 2
        *nop(), *nop(),                        # pipeline drain
        *opcode, ext,
        *_store_dn_and_advance(0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    assert result == 0x30, f"Expected 0x30 (table[2]), got 0x{result:08X}"
    h.cleanup()


@cocotb.test()
async def test_an_ind_all_address_regs(dut):
    """Test (An) indirect with A1-A5 (A0 is result ptr, A6/A7 are special)."""
    h = CPUTestHarness(dut)
    for i in range(1, 6):
        h.mem.load_long(h.DATA_BASE + i * 0x10, 0x10000000 + i)

    program = [*_setup_a0(h)]
    # Load each An with its data address, then read through it
    for i in range(1, 6):
        addr = h.DATA_BASE + i * 0x10
        program.extend(movea(LONG, SPECIAL, IMMEDIATE, i))
        program.extend(imm_long(addr))

    for i in range(1, 6):
        # Read (Ai) into D0, store
        program.extend(move(LONG, AN_IND, i, DN, 0))
        program.extend(_store_dn_and_advance(0))

    program.extend(h.sentinel_program())
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    for i in range(5):
        result = h.read_result_long(i * 4)
        expected = 0x10000000 + (i + 1)
        assert result == expected, (
            f"A{i+1} indirect: expected 0x{expected:08X}, got 0x{result:08X}"
        )
    h.cleanup()


@cocotb.test()
async def test_scale8_indexed(dut):
    """MOVE.L (0,A3,D1.L*8),D0 -- index scaled by 8."""
    h = CPUTestHarness(dut)
    idx_val = 2
    # Effective address = A3 + D1*8 + 0 = DATA_BASE + 16
    h.mem.load_long(h.DATA_BASE + idx_val * 8, 0xDEF01234)
    opcode = move(LONG, AN_IDX, 3, DN, 0)
    ext = _brief_extension_word(da=0, reg=1, wl=1, scale=3, disp8=0)  # scale=3 means *8
    program = [
        *_setup_a0(h),
        *movea(LONG, SPECIAL, IMMEDIATE, 3), *imm_long(h.DATA_BASE),
        *moveq(idx_val, 1),
        *nop(), *nop(),                        # pipeline drain
        *opcode, ext,
        *_store_dn_and_advance(0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"
    result = h.read_result_long(0)
    assert result == 0xDEF01234, f"Expected 0xDEF01234, got 0x{result:08X}"
    h.cleanup()
