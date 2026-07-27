"""
Audit probe: exception stacking with SR.M=1 (master stack).

UM 8.1.9/Table 8-6: with M=1, a (non-interrupt) exception pushes its frame
on the master stack, decrementing MSP. Probe sets MSP, sets M, executes
TRAP #0, and checks the handler's A7 and frame.
"""

import cocotb
from cocotb.triggers import RisingEdge, ClockCycles

from cpu_harness import CPUTestHarness
from bus_model import BusModel
from test_interrupt_probe import IackBusModel

RES = 0x020000
MSP_INIT = 0x003000
ISP_INIT = CPUTestHarness.SSP_INIT


@cocotb.test()
async def test_trap_with_m_bit_uses_master_stack(dut):
    h = CPUTestHarness(dut)
    handler = [
        0x23CF, 0x0002, 0x0000,      # MOVE.L A7,($20000).L
        0x3217,                      # MOVE.W (A7),D1     (stacked SR)
        0x33C1, 0x0002, 0x0004,      # MOVE.W D1,($20004).L
        0x322F, 0x0006,              # MOVE.W 6(A7),D1    (format/vector)
        0x33C1, 0x0002, 0x0006,      # MOVE.W D1,($20006).L
        0x2E3C, 0xDEAD, 0xCAFE,      # sentinel
        0x23C7, 0x0003, 0x0000,
        0x4E72, 0x2700,              # STOP
    ]
    program = [
        0x203C, 0x0000, MSP_INIT,    # MOVE.L #MSP_INIT,D0
        0x4E7B, 0x0803,              # MOVEC D0,MSP
        0x007C, 0x1000,              # ORI #$1000,SR   (set M)
        0x4E40,                      # TRAP #0
        0x60FE,                      # (not reached)
    ]

    clock = cocotb.clock.Clock(dut.CLK, 10, unit="ns")
    cocotb.start_soon(clock.start())
    h._init_idle_inputs()
    h._load_memory(program)
    h.mem.load_long(32 * 4, 0x800)   # TRAP #0 vector
    h.mem.load_words(0x800, handler)
    await RisingEdge(dut.CLK)
    await RisingEdge(dut.CLK)
    dut.RESET_INn.value = 0
    dut.HALT_INn.value = 0
    await ClockCycles(dut.CLK, 20)
    h.bus = BusModel(dut, h.mem, 0)
    await h.bus.start()
    dut.RESET_INn.value = 1
    dut.HALT_INn.value = 1
    await ClockCycles(dut.CLK, 4)

    found = False
    for _ in range(6000):
        await RisingEdge(dut.CLK)
        if h.mem.read(h.SENTINEL_ADDR, 4) == h.SENTINEL_VAL:
            found = True
            break
    assert found, "handler never completed"
    a7 = h.mem.read(RES, 4)
    sr = h.mem.read(RES + 4, 2)
    fmt = h.mem.read(RES + 6, 2)
    assert a7 == MSP_INIT - 8, (
        f"A7 in TRAP handler = 0x{a7:08X}; expected MSP-8 = 0x{MSP_INIT - 8:08X} "
        f"(M=1 exception must push the frame on the master stack). "
        f"stacked SR=0x{sr:04X} fmt=0x{fmt:04X}"
    )
    assert (sr >> 12) & 1 == 1 and (fmt & 0xFFF) == 0x080, (
        f"frame contents wrong: SR=0x{sr:04X} fmt=0x{fmt:04X}"
    )


@cocotb.test()
async def test_interrupt_with_m_bit_throwaway_frame(dut):
    """UM 8.1.9: with M=1 an interrupt puts its normal frame on the master
    stack, then clears M and puts a format-$1 throwaway frame on the
    interrupt stack."""
    h = CPUTestHarness(dut)
    handler = [
        0x23CF, 0x0002, 0x0000,      # MOVE.L A7,($20000).L    (active SP = ISP)
        0x40C1,                      # MOVE SR,D1
        0x33C1, 0x0002, 0x0004,      # MOVE.W D1,($20004).L    (live SR: M must be 0)
        0x322F, 0x0006,              # MOVE.W 6(A7),D1         (throwaway format word)
        0x33C1, 0x0002, 0x0006,      # MOVE.W D1,($20006).L
        0x4E7A, 0x2803,              # MOVEC MSP,D2
        0x23C2, 0x0002, 0x000C,      # MOVE.L D2,($2000C).L
        0x3639, 0x0000, 0x2FFE,      # MOVE.W ($2FFE).L,D3     (master frame +6)
        0x33C3, 0x0002, 0x0010,      # MOVE.W D3,($20010).L
        0x2E3C, 0xDEAD, 0xCAFE,      # sentinel
        0x23C7, 0x0003, 0x0000,
        0x4E72, 0x2700,              # STOP
    ]
    program = [
        0x203C, 0x0000, MSP_INIT,    # MOVE.L #MSP_INIT,D0
        0x4E7B, 0x0803,              # MOVEC D0,MSP
        0x46FC, 0x3000,              # MOVE #$3000,SR  (S=1, M=1, mask 0)
        0x60FE,                      # BRA.S self
    ]

    bus = IackBusModel(dut, h.mem, use_avec=True)
    clock = cocotb.clock.Clock(dut.CLK, 10, unit="ns")
    cocotb.start_soon(clock.start())
    h._init_idle_inputs()
    h._load_memory(program)
    h.mem.load_long(26 * 4, 0x800)   # level-2 autovector
    h.mem.load_words(0x800, handler)
    await RisingEdge(dut.CLK)
    await RisingEdge(dut.CLK)
    dut.RESET_INn.value = 0
    dut.HALT_INn.value = 0
    await ClockCycles(dut.CLK, 20)
    h.bus = bus
    await bus.start()
    dut.RESET_INn.value = 1
    dut.HALT_INn.value = 1
    await ClockCycles(dut.CLK, 300)

    dut.IPLn.value = 0b101           # level 2 (active low)

    found = False
    for _ in range(6000):
        await RisingEdge(dut.CLK)
        if h.mem.read(h.SENTINEL_ADDR, 4) == h.SENTINEL_VAL:
            found = True
            break
    dut.IPLn.value = 0b111

    a7 = h.mem.read(RES, 4)
    sr = h.mem.read(RES + 4, 2)
    tfmt = h.mem.read(RES + 6, 2)
    msp = h.mem.read(RES + 0x0C, 4)
    mfmt = h.mem.read(RES + 0x10, 2)
    assert found, (
        f"interrupt handler never completed (A7=0x{a7:08X} MSP=0x{msp:08X})"
    )
    assert msp == MSP_INIT - 8, (
        f"MSP = 0x{msp:08X}; expected 0x{MSP_INIT - 8:08X} "
        f"(normal frame belongs on the master stack)"
    )
    assert (mfmt >> 12) == 0x0 and (mfmt & 0xFFF) == 0x068, (
        f"master-stack frame format/vector word = 0x{mfmt:04X}; expected 0x0068"
    )
    assert a7 == ISP_INIT - 8, (
        f"handler A7 = 0x{a7:08X}; expected ISP-8 = 0x{ISP_INIT - 8:08X} "
        f"(throwaway frame belongs on the interrupt stack)"
    )
    assert (tfmt >> 12) == 0x1 and (tfmt & 0xFFF) == 0x068, (
        f"throwaway frame format/vector word = 0x{tfmt:04X}; expected 0x1068"
    )
    assert (sr >> 12) & 1 == 0, (
        f"live SR in the handler = 0x{sr:04X}; M must be cleared by interrupt "
        f"exception processing"
    )
