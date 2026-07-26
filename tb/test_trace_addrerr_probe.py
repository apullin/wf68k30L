"""
Audit probes: trace exceptions (T1/T0) and address error on odd fetch.

Neither path is exercised by the existing suite.
"""

import cocotb
from cocotb.triggers import RisingEdge, ClockCycles

from cpu_harness import CPUTestHarness
from bus_model import BusModel

COUNTER_ADDR = 0x020000
FMT_ADDR = 0x020008
MARKER_ADDR = 0x020010
STACKED_PC_ADDR = 0x020018
D3_ADDR = 0x02001C

# trace handler (vec 9 @ 0x24): counter++, save frame format word, RTE
TRACE_HANDLER = [
    0x2039, 0x0002, 0x0000,          # MOVE.L ($20000).L,D0
    0x5280,                          # ADDQ.L #1,D0
    0x23C0, 0x0002, 0x0000,          # MOVE.L D0,($20000).L
    0x322F, 0x0006,                  # MOVE.W 6(A7),D1
    0x33C1, 0x0002, 0x0008,          # MOVE.W D1,($20008).L
    0x4E73,                          # RTE
]

# address-error handler (vec 3 @ 0x0C): marker, format word, sentinel, stop
ADDRERR_HANDLER = [
    0x203C, 0xBAD0, 0x0003,          # MOVE.L #$BAD00003,D0
    0x23C0, 0x0002, 0x0010,          # MOVE.L D0,($20010).L
    0x322F, 0x0006,                  # MOVE.W 6(A7),D1
    0x33C1, 0x0002, 0x0008,          # MOVE.W D1,($20008).L
    0x2E3C, 0xDEAD, 0xCAFE,          # MOVE.L #$DEADCAFE,D7
    0x23C7, 0x0003, 0x0000,          # MOVE.L D7,($30000).L
    0x4E72, 0x2700,                  # STOP #$2700
]

HANDLER_BASE = 0x000800
HANDLER2_BASE = 0x000A00

SENTINEL_WRITES = [
    0x2E3C, 0xDEAD, 0xCAFE,          # MOVE.L #$DEADCAFE,D7
    0x23C7, 0x0003, 0x0000,          # MOVE.L D7,($30000).L
]


async def _setup(h, program, handlers):
    """handlers: list of (vector_addr, base, words)."""
    clock = cocotb.clock.Clock(h.dut.CLK, 10, unit="ns")
    cocotb.start_soon(clock.start())
    h._init_idle_inputs()
    h._load_memory(program)
    for vec_addr, base, words in handlers:
        h.mem.load_long(vec_addr, base)
        h.mem.load_words(base, words)
    h.mem.load_long(COUNTER_ADDR, 0)
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


@cocotb.test()
async def test_trace_t1_traces_each_instruction(dut):
    """T1=1: every instruction traces; frame is format 2, vector offset 0x24."""
    h = CPUTestHarness(dut)
    program = [
        0x46FC, 0xA000,              # MOVE #$A000,SR  (S=1, T1=1)
        0x7005,                      # MOVEQ #5,D0     -> trace after this
        0x7206,                      # MOVEQ #6,D1     -> trace
        0x7407,                      # MOVEQ #7,D2     -> trace
        *SENTINEL_WRITES,            # each also traces; then spin
        0x60FE,                      # BRA.S self
    ]
    await _setup(h, program, [(9 * 4, HANDLER_BASE, TRACE_HANDLER)])

    found = False
    for _ in range(6000):
        await RisingEdge(dut.CLK)
        if h.mem.read(h.SENTINEL_ADDR, 4) == h.SENTINEL_VAL:
            found = True
            break
    count_at_sentinel = h.mem.read(COUNTER_ADDR, 4)
    fmt = h.mem.read(FMT_ADDR, 2)
    assert found, (
        f"Program never completed under T1 trace (counter={count_at_sentinel})"
    )
    # By sentinel time: MOVEQ x3 + MOVE.L #,D7 + MOVE.L D7,(abs).L = >=4 traces
    # (the final store's own trace may not have executed yet).
    assert count_at_sentinel >= 4, (
        f"T1 trace fired {count_at_sentinel} times; expected >=4 "
        f"(one per instruction after SR write)"
    )
    assert (fmt >> 12) == 0x2 and (fmt & 0xFFF) == 0x24, (
        f"Trace frame format/vector word = 0x{fmt:04X}; expected 0x2024 "
        f"(format 2, vector offset 0x24)"
    )


@cocotb.test()
async def test_trace_t1_stacks_next_instruction_pc(dut):
    """UM 8.1.7: the format-2 trace frame carries the NEXT instruction's PC.

    The traced instruction is a 6-byte MOVE.L #imm,D3 at 0x104, so the stacked
    PC must be 0x10A - not 0x104 (own address) and not 0x108 (short by one word).
    The handler clears T1 in the stacked SR so exactly one trace is taken.
    """
    h = CPUTestHarness(dut)
    handler = [
        0x202F, 0x0002,              # MOVE.L 2(A7),D0    (stacked PC)
        0x23C0, 0x0002, 0x0018,      # MOVE.L D0,($20018).L
        0x0257, 0x7FFF,              # ANDI.W #$7FFF,(A7) clear T1 in stacked SR
        0x4E73,                      # RTE
    ]
    program = [
        0x46FC, 0xA000,              # 0x100: MOVE #$A000,SR (S=1, T1=1)
        0x263C, 0x1122, 0x3344,      # 0x104: MOVE.L #$11223344,D3 (6 bytes)
        0x23C3, 0x0002, 0x001C,      # 0x10A: MOVE.L D3,($2001C).L
        *SENTINEL_WRITES,
        0x60FE,                      # BRA.S self
    ]
    await _setup(h, program, [(9 * 4, HANDLER_BASE, handler)])

    found = False
    for _ in range(6000):
        await RisingEdge(dut.CLK)
        if h.mem.read(h.SENTINEL_ADDR, 4) == h.SENTINEL_VAL:
            found = True
            break
    stacked_pc = h.mem.read(STACKED_PC_ADDR, 4)
    d3 = h.mem.read(D3_ADDR, 4)
    assert found, (
        f"Program never completed (stacked PC seen = 0x{stacked_pc:08X})"
    )
    assert d3 == 0x11223344, (
        f"Traced instruction did not execute: D3=0x{d3:08X}"
    )
    assert stacked_pc == 0x0000010A, (
        f"Trace frame stacked PC = 0x{stacked_pc:08X}; expected 0x0000010A "
        f"(address of the instruction following the 6-byte MOVE.L at 0x104)"
    )


@cocotb.test()
async def test_trace_t0_straight_line_no_trace(dut):
    """T0=1 (trace on flow change): straight-line code must NOT trace."""
    h = CPUTestHarness(dut)
    program = [
        0x46FC, 0x6000,              # MOVE #$6000,SR  (S=1, T0=1)
        0x7005, 0x7206, 0x7407,      # MOVEQs (no flow change)
        *SENTINEL_WRITES,
        0x60FE,                      # BRA.S self (flow change - traces after)
    ]
    await _setup(h, program, [(9 * 4, HANDLER_BASE, TRACE_HANDLER)])

    found = False
    for _ in range(6000):
        await RisingEdge(dut.CLK)
        if h.mem.read(h.SENTINEL_ADDR, 4) == h.SENTINEL_VAL:
            found = True
            break
    count_at_sentinel = h.mem.read(COUNTER_ADDR, 4)
    assert found, "Program never completed under T0"
    assert count_at_sentinel == 0, (
        f"T0 traced {count_at_sentinel} straight-line instructions; "
        f"must trace only flow changes"
    )


@cocotb.test()
async def test_trace_t0_taken_branch_traces(dut):
    """T0=1: a taken BRA must generate exactly one trace at that point."""
    h = CPUTestHarness(dut)
    program = [
        0x46FC, 0x6000,              # MOVE #$6000,SR  (S=1, T0=1)
        0x7005,                      # MOVEQ #5,D0 (no trace)
        0x6002,                      # BRA.S +2 (taken -> one trace)
        0x4E71,                      # NOP (skipped)
        *SENTINEL_WRITES,            # straight-line (no trace)
        0x60FE,                      # BRA.S self
    ]
    await _setup(h, program, [(9 * 4, HANDLER_BASE, TRACE_HANDLER)])

    found = False
    for _ in range(6000):
        await RisingEdge(dut.CLK)
        if h.mem.read(h.SENTINEL_ADDR, 4) == h.SENTINEL_VAL:
            found = True
            break
    count_at_sentinel = h.mem.read(COUNTER_ADDR, 4)
    assert found, (
        f"Program never completed under T0 (counter={count_at_sentinel})"
    )
    assert count_at_sentinel == 1, (
        f"T0 trace count at sentinel = {count_at_sentinel}; expected exactly 1 "
        f"(the taken BRA)"
    )


@cocotb.test()
async def test_address_error_on_odd_jmp(dut):
    """JMP to an odd address must take vector 3 (address error)."""
    h = CPUTestHarness(dut)
    program = [
        0x4EF9, 0x0000, 0x0101,      # JMP ($101).L  (odd)
        0x4E71,                      # NOP (must not be reached)
    ]
    await _setup(h, program, [(3 * 4, HANDLER2_BASE, ADDRERR_HANDLER)])

    found = False
    for _ in range(6000):
        await RisingEdge(dut.CLK)
        if h.mem.read(h.SENTINEL_ADDR, 4) == h.SENTINEL_VAL:
            found = True
            break
    marker = h.mem.read(MARKER_ADDR, 4)
    fmt = h.mem.read(FMT_ADDR, 2)
    assert found, "Address-error handler never ran for odd JMP target"
    assert marker == 0xBAD00003, f"Wrong handler ran: marker=0x{marker:08X}"
    assert (fmt & 0xFFF) == 0x00C, (
        f"Vector offset in frame = 0x{fmt & 0xFFF:03X}; expected 0x00C (vec 3); "
        f"full format word 0x{fmt:04X}"
    )
