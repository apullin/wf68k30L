"""
Audit probes: decode/control findings DC-1 (RTD), DC-2 (CHK over-accept),
DC-4 (TRAPcc reserved opmode).
"""

import cocotb
from cocotb.triggers import RisingEdge, ClockCycles

import m68k_encode as enc
from cpu_harness import CPUTestHarness
from bus_model import BusModel

RES = 0x020000
SENTINEL = [0x2E3C, 0xDEAD, 0xCAFE, 0x23C7, 0x0003, 0x0000]
# vec 4 (illegal) marker handler
ILLEGAL_HANDLER = [
    0x203C, 0x111E, 0x6A11,          # MOVE.L #$111E6A11,D0
    0x23C0, 0x0002, 0x0010,          # MOVE.L D0,($20010).L
    *SENTINEL,
    0x4E72, 0x2700,
]


async def _run(h, program, handlers=(), cycles=6000):
    clock = cocotb.clock.Clock(h.dut.CLK, 10, unit="ns")
    cocotb.start_soon(clock.start())
    h._init_idle_inputs()
    h._load_memory(program)
    for vec_addr, base, words in handlers:
        h.mem.load_long(vec_addr, base)
        h.mem.load_words(base, words)
    await RisingEdge(h.dut.CLK)
    await RisingEdge(h.dut.CLK)
    h.dut.RESET_INn.value = 0
    h.dut.HALT_INn.value = 0
    await ClockCycles(h.dut.CLK, 20)
    h.bus = BusModel(h.dut, h.mem, 0)
    await h.bus.start()
    h.dut.RESET_INn.value = 1
    h.dut.HALT_INn.value = 1
    await ClockCycles(h.dut.CLK, 4)
    for _ in range(cycles):
        await RisingEdge(h.dut.CLK)
        if h.mem.read(h.SENTINEL_ADDR, 4) == h.SENTINEL_VAL:
            return True
    return False


async def _rtd_case(dut, disp):
    """JSR to a stub that does RTD #disp; return the resulting A7."""
    h = CPUTestHarness(dut)
    h.mem.load_words(0x800, enc.rtd(disp))
    program = [
        0x2E7C, 0x0000, 0x1000,      # MOVEA.L #$1000,A7
        0x4EB9, 0x0000, 0x0800,      # JSR ($800).L     -> pushes 4, SP=0xFFC
        0x2E8F,                       # MOVE.L A7,(A7)   (dummy, keeps A7 live)
        0x23CF, 0x0002, 0x0000,      # MOVE.L A7,($20000).L
        *SENTINEL, 0x60FE,
    ]
    assert await _run(h, program), "program did not complete"
    return h.mem.read(RES, 4)


@cocotb.test()
async def test_rtd_stack_adjust(dut):
    """RTD #4: SP must end at entry_SP + 4 (PC pop) + 4 (displacement)."""
    sp = await _rtd_case(dut, 4)
    assert sp == 0x1004, (
        f"After JSR then RTD #4: A7=0x{sp:08X}, expected 0x00001004 "
        f"(0x1000 - 4 pushed, then +4 popped +4 displacement)"
    )


@cocotb.test()
async def test_rtd_zero_displacement(dut):
    """RTD #0 must behave exactly like RTS: SP + 4 + 0."""
    sp = await _rtd_case(dut, 0)
    assert sp == 0x1000, (
        f"After JSR then RTD #0: A7=0x{sp:08X}, expected 0x00001000 "
        f"(pure PC pop, no deallocation)"
    )


@cocotb.test()
async def test_rtd_negative_displacement(dut):
    """RTD #-8: the displacement is sign-extended, so SP + 4 - 8."""
    sp = await _rtd_case(dut, -8)
    assert sp == 0x0FF8, (
        f"After JSR then RTD #-8: A7=0x{sp:08X}, expected 0x00000FF8 "
        f"(0xFFC + 4 popped - 8; 16-bit displacement is sign-extended)"
    )


@cocotb.test()
async def test_rtd_large_displacement(dut):
    """RTD #$1000: full 16-bit positive displacement range."""
    sp = await _rtd_case(dut, 0x1000)
    assert sp == 0x2000, (
        f"After JSR then RTD #$1000: A7=0x{sp:08X}, expected 0x00002000 "
        f"(0xFFC + 4 popped + 0x1000)"
    )


@cocotb.test()
async def test_unallocated_4b40_is_illegal(dut):
    """Opcode 0x4B40 is unallocated -> illegal instruction (vector 4)."""
    h = CPUTestHarness(dut)
    program = [
        0x4B40,                       # unallocated
        *SENTINEL, 0x60FE,
    ]
    assert await _run(h, program, [(4 * 4, 0x900, ILLEGAL_HANDLER)]), "no completion"
    marker = h.mem.read(RES + 0x10, 4)
    assert marker == 0x111E6A11, (
        f"0x4B40 did not take the illegal-instruction vector "
        f"(marker=0x{marker:08X}); CHK decode accepts bit6=1 encodings"
    )


async def _illegal_program(dut, words):
    """Run a suspect instruction sequence; return True if vector 4 was taken."""
    h = CPUTestHarness(dut)
    program = [*words, *SENTINEL, 0x60FE]
    assert await _run(h, program, [(4 * 4, 0x900, ILLEGAL_HANDLER)]), "no completion"
    return h.mem.read(RES + 0x10, 4) == 0x111E6A11


async def _illegal_case(dut, opcode):
    """Run a single suspect opcode; return True if vector 4 was taken."""
    return await _illegal_program(dut, [opcode])


@cocotb.test()
async def test_invalid_lea_an_direct_is_illegal(dut):
    """0x41C8 is LEA A0,A0 - An direct is not a control mode, so illegal."""
    assert await _illegal_case(dut, 0x41C8), (
        "0x41C8 (LEA with An-direct EA) did not take the illegal-instruction "
        "vector; it falls through into the CHK arm"
    )


@cocotb.test()
async def test_invalid_lea_immediate_is_illegal(dut):
    """0x41FC is LEA #<data>,A0 - immediate is not a control mode."""
    assert await _illegal_case(dut, 0x41FC), (
        "0x41FC (LEA with immediate EA) did not take the illegal-instruction "
        "vector; it falls through into the CHK arm"
    )


@cocotb.test()
async def test_chk_bit6_set_dn_is_illegal(dut):
    """0x4148: CHK.L pattern, mode 001 (An direct) with bit 6 set."""
    assert await _illegal_case(dut, 0x4148), (
        "0x4148 (CHK pattern with bit 6 = 1) did not take the "
        "illegal-instruction vector; bit 6 is fixed 0 in CHK"
    )


@cocotb.test()
async def test_chk_bit6_set_an_is_illegal(dut):
    """0x4B48: CHK.L pattern, mode 001 with bit 6 set, register field 101."""
    assert await _illegal_case(dut, 0x4B48), (
        "0x4B48 (CHK pattern with bit 6 = 1) did not take the "
        "illegal-instruction vector; bit 6 is fixed 0 in CHK"
    )


# vec 7 (TRAPcc) marker handler, so vector 4 and vector 7 can be told apart
TRAPCC_HANDLER = [
    0x203C, 0x7777, 0x0007,          # MOVE.L #$77770007,D0
    0x23C0, 0x0002, 0x0014,          # MOVE.L D0,($20014).L
    *SENTINEL,
    0x4E72, 0x2700,
]


async def _trapcc_case(dut, words):
    """Run a TRAPcc encoding; return (took_vec4, took_vec7)."""
    h = CPUTestHarness(dut)
    program = [*words, *SENTINEL, 0x60FE]
    assert await _run(h, program, [(4 * 4, 0x900, ILLEGAL_HANDLER),
                                   (7 * 4, 0xA00, TRAPCC_HANDLER)]), "no completion"
    return (h.mem.read(RES + 0x10, 4) == 0x111E6A11,
            h.mem.read(RES + 0x14, 4) == 0x77770007)


@cocotb.test()
async def test_trapcc_reserved_opmode_is_illegal(dut):
    """0x50FD (TRAPT with reserved opmode 101) -> illegal instruction."""
    illegal, trapcc = await _trapcc_case(dut, [0x50FD])
    assert illegal, (
        f"0x50FD took vector {'7 (TRAPcc)' if trapcc else 'none'} "
        f"instead of vector 4; TRAPcc opmode 101 is reserved and must be illegal"
    )


@cocotb.test()
async def test_trapcc_reserved_opmode_110_is_illegal(dut):
    """0x50FE (TRAPT with reserved opmode 110) -> illegal instruction."""
    illegal, trapcc = await _trapcc_case(dut, [0x50FE])
    assert illegal, (
        f"0x50FE took vector {'7 (TRAPcc)' if trapcc else 'none'} "
        f"instead of vector 4; TRAPcc opmode 110 is reserved"
    )


@cocotb.test()
async def test_trapcc_reserved_opmode_111_is_illegal(dut):
    """0x50FF (TRAPT with reserved opmode 111) -> illegal instruction."""
    illegal, trapcc = await _trapcc_case(dut, [0x50FF])
    assert illegal, (
        f"0x50FF took vector {'7 (TRAPcc)' if trapcc else 'none'} "
        f"instead of vector 4; TRAPcc opmode 111 is reserved"
    )


@cocotb.test()
async def test_trapt_no_operand_traps(dut):
    """TRAPT (opmode 100) is legal and must take vector 7."""
    illegal, trapcc = await _trapcc_case(dut, enc.trapcc(enc.CC_T))
    assert trapcc and not illegal, (
        f"TRAPT (0x50FC, opmode 100) took vector "
        f"{'4 (illegal)' if illegal else 'none'} instead of vector 7"
    )


@cocotb.test()
async def test_trapt_word_operand_traps(dut):
    """TRAPT.W #$1234 (opmode 010) is legal and must take vector 7."""
    illegal, trapcc = await _trapcc_case(dut, enc.trapcc(enc.CC_T, 0x1234, "W"))
    assert trapcc and not illegal, (
        f"TRAPT.W (0x50FA, opmode 010) took vector "
        f"{'4 (illegal)' if illegal else 'none'} instead of vector 7"
    )


@cocotb.test()
async def test_trapt_long_operand_traps(dut):
    """TRAPT.L #$12345678 (opmode 011) is legal and must take vector 7."""
    illegal, trapcc = await _trapcc_case(dut, enc.trapcc(enc.CC_T, 0x12345678, "L"))
    assert trapcc and not illegal, (
        f"TRAPT.L (0x50FB, opmode 011) took vector "
        f"{'4 (illegal)' if illegal else 'none'} instead of vector 7"
    )


@cocotb.test()
async def test_trapf_word_operand_skips_operand(dut):
    """TRAPF.W must not trap and must skip its operand word, not execute it."""
    h = CPUTestHarness(dut)
    program = [
        0x7000,                                  # MOVEQ #0,D0
        *enc.trapcc(enc.CC_F, 0x7001, "W"),      # TRAPF.W #$7001 (= MOVEQ #1,D0)
        0x23C0, 0x0002, 0x0000,                  # MOVE.L D0,($20000).L
        *SENTINEL, 0x60FE,
    ]
    assert await _run(h, program, [(4 * 4, 0x900, ILLEGAL_HANDLER),
                                   (7 * 4, 0xA00, TRAPCC_HANDLER)]), "no completion"
    assert h.mem.read(RES + 0x10, 4) == 0, "TRAPF.W wrongly took vector 4"
    assert h.mem.read(RES + 0x14, 4) == 0, "TRAPF.W wrongly took vector 7"
    assert h.mem.read(RES, 4) == 0, (
        f"D0=0x{h.mem.read(RES, 4):08X}: the TRAPF.W operand word was executed "
        f"as an opcode instead of being skipped"
    )


@cocotb.test()
async def test_trapf_long_operand_skips_operand(dut):
    """TRAPF.L must not trap and must skip both operand words."""
    h = CPUTestHarness(dut)
    program = [
        0x7000,                                        # MOVEQ #0,D0
        *enc.trapcc(enc.CC_F, 0x70017002, "L"),        # TRAPF.L (two MOVEQ words)
        0x23C0, 0x0002, 0x0000,                        # MOVE.L D0,($20000).L
        *SENTINEL, 0x60FE,
    ]
    assert await _run(h, program, [(4 * 4, 0x900, ILLEGAL_HANDLER),
                                   (7 * 4, 0xA00, TRAPCC_HANDLER)]), "no completion"
    assert h.mem.read(RES + 0x10, 4) == 0, "TRAPF.L wrongly took vector 4"
    assert h.mem.read(RES + 0x14, 4) == 0, "TRAPF.L wrongly took vector 7"
    assert h.mem.read(RES, 4) == 0, (
        f"D0=0x{h.mem.read(RES, 4):08X}: a TRAPF.L operand word was executed "
        f"as an opcode instead of being skipped"
    )


MMUSR_T = 1 << 6


@cocotb.test()
async def test_ptest_fc_from_nonzero_dn(dut):
    """PTESTR D3,(A0),#0 must take the function code from D3, not from D0.

    TT0 is programmed to match only FC=5. D3 holds 5, D0 holds 1, so a TT hit
    (MMUSR.T) proves the encoded register was used. This mirrors the immediate
    form in test_mmu_instr.test_mmu_ptest_level0_vs_level7_t_bit.
    """
    h = CPUTestHarness(dut)

    logical_addr = 0x12000600
    tt1_src = h.DATA_BASE + 0x460
    mmusr_dst = h.DATA_BASE + 0x480

    tt_prog = 0x00008143   # TT1: supervisor program pass-through for low addresses
    tt_data = 0x12008150   # TT0: transparent for top-byte 0x12, FC base 5, mask 000

    program = [
        *enc.movea(enc.LONG, enc.SPECIAL, enc.IMMEDIATE, 0),  # A0 = address to test
        *enc.imm_long(logical_addr),
        *enc.movea(enc.LONG, enc.SPECIAL, enc.IMMEDIATE, 1),  # A1 = TT1 source
        *enc.imm_long(tt1_src),
        0xF011, 0x0C00,                       # PMOVE (A1),TT1
        *enc.movea(enc.LONG, enc.SPECIAL, enc.IMMEDIATE, 2),  # A2 = TT0 source
        *enc.imm_long(tt1_src + 4),
        0xF012, 0x0800,                       # PMOVE (A2),TT0

        *enc.moveq(1, 0),                     # D0 = 1 (the wrong function code)
        *enc.moveq(5, 3),                     # D3 = 5 (the encoded function code)

        0xF010, 0x820B,                       # PTESTR D3,(A0),#0   (FC field 01011)
        *enc.movea(enc.LONG, enc.SPECIAL, enc.IMMEDIATE, 3),  # A3 = MMUSR dump
        *enc.imm_long(mmusr_dst),
        0xF013, 0x6200,                       # PMOVE MMUSR,(A3)

        *h.sentinel_program(),
    ]

    await h.setup(program)
    h.mem.load_long(tt1_src, tt_prog)
    h.mem.load_long(tt1_src + 4, tt_data)

    found = await h.run_until_sentinel(max_cycles=24000)
    assert found, "PTEST FC-from-Dn test did not complete"

    mmusr = h.mem.read(mmusr_dst, 2)
    assert (mmusr & MMUSR_T) != 0, (
        f"MMUSR=0x{mmusr:04X}: PTESTR D3,(A0),#0 did not hit the FC=5 transparent "
        f"translation, so the function code was not read from D3 (D0 held 1)"
    )
    h.cleanup()


# MOVE.L <ea>,D1 with ea = mode 110 / reg 000 (A0 with an extension word).
MOVE_L_EXT_D1 = 0x2230
# MOVEA.L #$00020100,A0 - a base pointing into writable test memory.
SETUP_A0 = [0x207C, 0x0002, 0x0100]


async def _full_format_illegal(dut, ext_word, trailing=()):
    """MOVE.L (ext_word...),D1 - true if the encoding took vector 4."""
    return await _illegal_program(
        dut, [*SETUP_A0, MOVE_L_EXT_D1, ext_word, *trailing])


@cocotb.test()
async def test_full_format_reserved_bd_size_is_illegal(dut):
    """BD SIZE = 00 is reserved (PRM Table 2-1) and must not be abandoned silently."""
    # bit8=1 full format, BS=0, IS=1, BD SIZE=00, I/IS=000.
    assert await _full_format_illegal(dut, 0x0140), (
        "a full-format extension word with the reserved BD SIZE = 00 did not "
        "take the illegal-instruction vector; the instruction was abandoned "
        "silently and following operand words decode as opcodes"
    )


@cocotb.test()
async def test_full_format_reserved_iis_index_suppressed_is_illegal(dut):
    """IS = 1 with I/IS = 100 is reserved (PRM Table 2-2)."""
    # bit8=1, BS=0, IS=1, BD SIZE=10 (word), I/IS=100.
    assert await _full_format_illegal(dut, 0x0164, [0x0010]), (
        "a full-format extension word with IS = 1 and the reserved I/IS = 100 "
        "did not take the illegal-instruction vector"
    )


@cocotb.test()
async def test_full_format_reserved_iis_indexed_is_illegal(dut):
    """IS = 0 with I/IS = 100 is reserved (PRM Table 2-2)."""
    # bit8=1, BS=0, IS=0, BD SIZE=10 (word), I/IS=100.
    assert await _full_format_illegal(dut, 0x0124, [0x0010]), (
        "a full-format extension word with IS = 0 and the reserved I/IS = 100 "
        "did not take the illegal-instruction vector"
    )


@cocotb.test()
async def test_full_format_reserved_iis_111_index_suppressed_is_illegal(dut):
    """IS = 1 with I/IS = 111 is reserved (PRM Table 2-2)."""
    # bit8=1, BS=0, IS=1, BD SIZE=10 (word), I/IS=111.
    assert await _full_format_illegal(dut, 0x0167, [0x0010]), (
        "a full-format extension word with IS = 1 and the reserved I/IS = 111 "
        "did not take the illegal-instruction vector"
    )


@cocotb.test()
async def test_full_format_word_bd_still_works(dut):
    """Guard against over-tightening: (bd16,A0) with IS=1, I/IS=000 must execute."""
    h = CPUTestHarness(dut)
    payload = 0x5A5AA5A5
    program = [
        *SETUP_A0,                   # A0 = $00020100
        # bit8=1, BS=0, IS=1, BD SIZE=10 (word), I/IS=000 -> EA = A0 + $20
        MOVE_L_EXT_D1, 0x0160, 0x0020,
        0x23C1, 0x0002, 0x0000,      # MOVE.L D1,($20000).L
        *SENTINEL, 0x60FE,
    ]
    h.mem.load_long(0x020120, payload)
    assert await _run(h, program, [(4 * 4, 0x900, ILLEGAL_HANDLER)]), "no completion"
    assert h.mem.read(RES + 0x10, 4) == 0, (
        "a valid (bd16,A0) full-format EA wrongly took the illegal vector"
    )
    got = h.mem.read(RES, 4)
    assert got == payload, (
        f"D1=0x{got:08X}, expected 0x{payload:08X} from ($20100 + $20)"
    )


@cocotb.test()
async def test_full_format_memory_indirect_still_works(dut):
    """Guard against over-tightening: IS=1, I/IS=001 memory indirect must execute."""
    h = CPUTestHarness(dut)
    payload = 0x0BADF00D
    program = [
        *SETUP_A0,                   # A0 = $00020100
        # bit8=1, BS=0, IS=1, BD SIZE=10 (word), I/IS=001 -> EA = [(A0 + $20)]
        MOVE_L_EXT_D1, 0x0161, 0x0020,
        0x23C1, 0x0002, 0x0000,      # MOVE.L D1,($20000).L
        *SENTINEL, 0x60FE,
    ]
    h.mem.load_long(0x020120, 0x00020200)   # intermediate pointer
    h.mem.load_long(0x020200, payload)
    assert await _run(h, program, [(4 * 4, 0x900, ILLEGAL_HANDLER)]), "no completion"
    assert h.mem.read(RES + 0x10, 4) == 0, (
        "a valid memory-indirect full-format EA wrongly took the illegal vector"
    )
    got = h.mem.read(RES, 4)
    assert got == payload, (
        f"D1=0x{got:08X}, expected 0x{payload:08X} from [($20100 + $20)]"
    )
