"""
Audit probes: datapath findings DP-1/2/6/7 (MULS.L, BFFFO, DIVU.W #0-word, CMPA.W).

Each asserts MC68030-correct behavior; failures confirm the audit findings.
"""

import cocotb
from cocotb.triggers import RisingEdge, ClockCycles

from cpu_harness import CPUTestHarness
from bus_model import BusModel

RES = 0x020000


async def _run(h, program, handlers=(), cycles=4000):
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


SENTINEL = [0x2E3C, 0xDEAD, 0xCAFE, 0x23C7, 0x0003, 0x0000]


@cocotb.test()
async def test_muls_l_64bit_negative(dut):
    """MULS.L D1,D3:D2 with -2 * 3 must give D3:D2 = FFFFFFFF:FFFFFFFA."""
    h = CPUTestHarness(dut)
    program = [
        0x223C, 0xFFFF, 0xFFFE,      # MOVE.L #-2,D1
        0x243C, 0x0000, 0x0003,      # MOVE.L #3,D2
        0x4C01, 0x2C03,              # MULS.L D1,D3:D2 (64-bit)
        0x23C2, 0x0002, 0x0000,      # MOVE.L D2,($20000).L
        0x23C3, 0x0002, 0x0004,      # MOVE.L D3,($20004).L
        *SENTINEL, 0x60FE,
    ]
    assert await _run(h, program), "program did not complete"
    lo = h.mem.read(RES, 4)
    hi = h.mem.read(RES + 4, 4)
    assert (hi, lo) == (0xFFFFFFFF, 0xFFFFFFFA), (
        f"MULS.L -2*3: D3:D2 = {hi:08X}:{lo:08X}, expected FFFFFFFF:FFFFFFFA"
    )


@cocotb.test()
async def test_bfffo_leading_zero_count(dut):
    """BFFFO D0{0:32},D1 with D0=0x00008000 must return offset 16."""
    h = CPUTestHarness(dut)
    program = [
        0x203C, 0x0000, 0x8000,      # MOVE.L #$00008000,D0
        0xEDC0, 0x1000,              # BFFFO D0{0:32},D1
        0x23C1, 0x0002, 0x0000,      # MOVE.L D1,($20000).L
        *SENTINEL, 0x60FE,
    ]
    assert await _run(h, program), "program did not complete"
    d1 = h.mem.read(RES, 4)
    assert d1 == 16, f"BFFFO: D1={d1}, expected 16 (bit offset from field MSB)"


@cocotb.test()
async def test_divu_w_zero_word_divisor_traps(dut):
    """DIVU.W D2,D0 with D2=0x00010000 (low word 0) must take vector 5."""
    h = CPUTestHarness(dut)
    handler = [                       # vec 5: marker + sentinel + stop
        0x203C, 0xD1B2, 0x0005,
        0x23C0, 0x0002, 0x0010,
        *SENTINEL,
        0x4E72, 0x2700,
    ]
    program = [
        0x243C, 0x0001, 0x0000,      # MOVE.L #$00010000,D2
        0x203C, 0x0000, 0x1234,      # MOVE.L #$1234,D0
        0x80C2,                       # DIVU.W D2,D0
        *SENTINEL, 0x60FE,            # (reached only if no trap)
    ]
    assert await _run(h, program, [(5 * 4, 0x800, handler)]), "no completion"
    marker = h.mem.read(RES + 0x10, 4)
    assert marker == 0xD1B20005, (
        f"DIVU.W by word-0 divisor did not trap (marker=0x{marker:08X}) — "
        f"divide-by-zero must consider only the low 16 bits"
    )


@cocotb.test()
async def test_cmpa_w_full_width_flags(dut):
    """CMPA.W #$8000,A0 with A0=0x00018000: 32-bit compare -> Z=0, C=1."""
    h = CPUTestHarness(dut)
    program = [
        0x207C, 0x0001, 0x8000,      # MOVEA.L #$00018000,A0
        0xB0FC, 0x8000,              # CMPA.W #$8000,A0
        0x42C2,                       # MOVE CCR,D2
        0x23C2, 0x0002, 0x0000,      # MOVE.L D2,($20000).L
        *SENTINEL, 0x60FE,
    ]
    assert await _run(h, program), "program did not complete"
    ccr = h.mem.read(RES, 4) & 0x1F
    z = (ccr >> 2) & 1
    c = ccr & 1
    n = (ccr >> 3) & 1
    assert (n, z, c) == (0, 0, 1), (
        f"CMPA.W flags N={n} Z={z} C={c}, expected N=0 Z=0 C=1 "
        f"(0x00018000 - sext(0x8000) = 0x00020000 with borrow)"
    )


@cocotb.test()
async def test_divs_l_64bit_overflow(dut):
    """DIVS.L D2,D1:D0 with 2^32/1: quotient overflows 32 bits -> V=1, regs unchanged."""
    h = CPUTestHarness(dut)
    program = [
        0x203C, 0x0000, 0x0000,      # MOVE.L #0,D0        (dividend low)
        0x223C, 0x0000, 0x0001,      # MOVE.L #1,D1        (dividend high)
        0x243C, 0x0000, 0x0001,      # MOVE.L #1,D2        (divisor)
        0x4C42, 0x0C01,              # DIVS.L D2,D1:D0  (64/32 signed)
        0x42C3,                      # MOVE CCR,D3
        0x23C0, 0x0002, 0x0000,      # MOVE.L D0,($20000).L
        0x23C1, 0x0002, 0x0004,      # MOVE.L D1,($20004).L
        0x23C3, 0x0002, 0x0008,      # MOVE.L D3,($20008).L
        *SENTINEL, 0x60FE,
    ]
    assert await _run(h, program), "program did not complete"
    d0 = h.mem.read(RES, 4)
    d1 = h.mem.read(RES + 4, 4)
    v = (h.mem.read(RES + 8, 4) >> 1) & 1
    assert v == 1 and d0 == 0 and d1 == 1, (
        f"DIVS.L 2^32/1: V={v} D0=0x{d0:08X} D1=0x{d1:08X}; "
        f"expected V=1 with D0/D1 unchanged (overflow)"
    )


@cocotb.test()
async def test_nbcd_x0_low_digit(dut):
    """NBCD D0 with X=0, D0=0x05 must give 0x95 with C=X=1 (tens complement)."""
    h = CPUTestHarness(dut)
    program = [
        0x44FC, 0x0000,              # MOVE #0,CCR (X=0)
        0x203C, 0x0000, 0x0005,      # MOVE.L #5,D0
        0x4800,                      # NBCD D0
        0x42C3,                      # MOVE CCR,D3
        0x23C0, 0x0002, 0x0000,      # MOVE.L D0,($20000).L
        0x23C3, 0x0002, 0x0004,      # MOVE.L D3,($20004).L
        *SENTINEL, 0x60FE,
    ]
    assert await _run(h, program), "program did not complete"
    d0 = h.mem.read(RES, 4) & 0xFF
    ccr = h.mem.read(RES + 4, 4) & 0x1F
    c = ccr & 1
    x = (ccr >> 4) & 1
    assert (d0, c, x) == (0x95, 1, 1), (
        f"NBCD 0x05 with X=0: got 0x{d0:02X} C={c} X={x}; expected 0x95 C=1 X=1"
    )


@cocotb.test()
async def test_chk2_l_in_bounds_no_trap(dut):
    """CHK2.L (A0),D3 with bounds 0x100..0x200 and D3=0x180 must not trap."""
    h = CPUTestHarness(dut)
    handler = [                       # vec 6 handler: marker + sentinel + stop
        0x203C, 0xBAD0, 0x0006,
        0x23C0, 0x0002, 0x0010,
        *SENTINEL,
        0x4E72, 0x2700,
    ]
    program = [
        0x207C, 0x0001, 0x0000,      # MOVEA.L #$10000,A0
        0x263C, 0x0000, 0x0180,      # MOVE.L #$180,D3
        0x04D0, 0x3800,              # CHK2.L (A0),D3
        *SENTINEL, 0x60FE,
    ]
    h.mem.load_long(0x10000, 0x00000100)   # lower bound
    h.mem.load_long(0x10004, 0x00000200)   # upper bound
    assert await _run(h, program, [(6 * 4, 0x800, handler)]), "no completion"
    marker = h.mem.read(RES + 0x10, 4)
    assert marker != 0xBAD00006, (
        "CHK2.L trapped although D3=0x180 is inside [0x100,0x200] "
        "(byte-compare terms not size-gated)"
    )
