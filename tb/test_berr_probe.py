"""
Audit probe: live bus-error exception + RTE rerun of the faulted access.

Existing RTE format-A/B tests build frames by hand; this drives BERRn on a
real bus cycle and expects the full fault -> handler -> RTE -> rerun loop.
"""

import cocotb
from cocotb.triggers import RisingEdge, ClockCycles

from cpu_harness import CPUTestHarness
from bus_model import BusModel

FAULT_ADDR = 0x0E0000
COUNTER_ADDR = 0x020000
FMT_ADDR = 0x020008
D0_ADDR = 0x020014


class BerrOnceBusModel(BusModel):
    """Asserts BERRn (no DSACK) for the first N accesses to FAULT_ADDR."""

    def __init__(self, dut, memory, wait_states=0, fault_addr=FAULT_ADDR, times=1):
        super().__init__(dut, memory, wait_states)
        self.fault_addr = fault_addr
        self.faults_left = times
        self.fault_count = 0
        self.log = []

    async def _responder(self):
        while self._running:
            await RisingEdge(self.dut.CLK)
            try:
                as_n = int(self.dut.ASn.value)
            except ValueError:
                self.dut.DSACKn.value = 0b11
                continue

            if as_n == 0:
                try:
                    addr = int(self.dut.ADR_OUT.value)
                except ValueError:
                    addr = 0

                try:
                    fc = int(self.dut.FC_OUT.value)
                except ValueError:
                    fc = -1
                if self.fault_count and len(self.log) < 60:
                    try:
                        rw = int(self.dut.RWn.value)
                    except ValueError:
                        rw = -1
                    if not self.log or self.log[-1] != (addr, rw, fc):
                        self.log.append((addr, rw, fc))
                if (addr & ~3) == (self.fault_addr & ~3) and self.faults_left > 0:
                    self.faults_left -= 1
                    self.fault_count += 1
                    self.dut.BERRn.value = 0
                    while self._running:
                        await RisingEdge(self.dut.CLK)
                        try:
                            if int(self.dut.ASn.value) == 1:
                                break
                        except ValueError:
                            break
                    self.dut.BERRn.value = 1
                    continue

                try:
                    rw_n = int(self.dut.RWn.value)
                except ValueError:
                    self.dut.DSACKn.value = 0b11
                    continue
                size_code = self._get_size_code()
                start_lane, byte_count = self._cycle_layout(
                    addr, size_code, is_write=(rw_n == 0)
                )
                for _ in range(self.wait_states):
                    await RisingEdge(self.dut.CLK)
                if rw_n == 1:
                    data = 0
                    for i in range(byte_count):
                        b = self.memory.read(addr + i, 1) & 0xFF
                        data |= b << ((3 - (start_lane + i)) * 8)
                    self.dut.DATA_IN.value = data
                    self.dut.DSACKn.value = 0b00
                else:
                    try:
                        data = int(self.dut.DATA_OUT.value)
                    except ValueError:
                        data = 0
                    for i in range(byte_count):
                        self.memory.write(
                            addr + i, 1, (data >> ((3 - (start_lane + i)) * 8)) & 0xFF
                        )
                    self.dut.DSACKn.value = 0b00
                while self._running:
                    await RisingEdge(self.dut.CLK)
                    try:
                        if int(self.dut.ASn.value) == 1:
                            break
                    except ValueError:
                        break
                self.dut.DSACKn.value = 0b11
            else:
                self.dut.DSACKn.value = 0b11


# vec 2 handler: counter++, capture format word, RTE (rerun faulted access)
BERR_HANDLER = [
    0x2039, 0x0002, 0x0000,          # MOVE.L ($20000).L,D0
    0x5280,                          # ADDQ.L #1,D0
    0x23C0, 0x0002, 0x0000,          # MOVE.L D0,($20000).L
    0x322F, 0x0006,                  # MOVE.W 6(A7),D1
    0x33C1, 0x0002, 0x0008,          # MOVE.W D1,($20008).L
    0x4E73,                          # RTE
]

PROGRAM = [
    0x2039, 0x000E, 0x0000,          # MOVE.L ($E0000).L,D0   <- BERR first time
    0x23C0, 0x0002, 0x0014,          # MOVE.L D0,($20014).L
    0x2E3C, 0xDEAD, 0xCAFE,          # MOVE.L #$DEADCAFE,D7
    0x23C7, 0x0003, 0x0000,          # MOVE.L D7,($30000).L
    0x60FE,                          # BRA.S self
]

HANDLER_BASE = 0x000800


@cocotb.test()
async def test_berr_exception_and_rte_rerun(dut):
    """BERR on a data read takes vector 2; RTE reruns and program completes."""
    h = CPUTestHarness(dut)
    bus = BerrOnceBusModel(dut, h.mem, times=1)

    clock = cocotb.clock.Clock(dut.CLK, 10, unit="ns")
    cocotb.start_soon(clock.start())
    h._init_idle_inputs()
    h._load_memory(PROGRAM)
    h.mem.load_long(2 * 4, HANDLER_BASE)      # vector 2
    h.mem.load_words(HANDLER_BASE, BERR_HANDLER)
    h.mem.load_long(COUNTER_ADDR, 0)
    h.mem.load_long(FAULT_ADDR, 0x5A5AA5A5)   # data at faulting address
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

    found = False
    for _ in range(8000):
        await RisingEdge(dut.CLK)
        if h.mem.read(h.SENTINEL_ADDR, 4) == h.SENTINEL_VAL:
            found = True
            break

    counter = h.mem.read(COUNTER_ADDR, 4)
    fmt = h.mem.read(FMT_ADDR, 2)
    d0 = h.mem.read(D0_ADDR, 4)
    dut._log.warning("bus log after fault: %s",
                     [(hex(a), 'R' if r == 1 else 'W', f) for a, r, f in bus.log])
    frame = [h.mem.read(0xFA4 + i * 2, 2) for i in range(46)]
    dut._log.warning("stacked frame @0xFA4 (46 words): %s",
                     " ".join(f"{w:04X}" for w in frame))
    assert bus.fault_count == 1, f"BERR injected {bus.fault_count} times"
    assert counter == 1, (
        f"Bus-error handler ran {counter} times (expected 1); fmt=0x{fmt:04X}"
    )
    assert found, (
        f"Program did not complete after RTE rerun "
        f"(handler runs={counter}, fmt=0x{fmt:04X})"
    )
    assert (fmt & 0xFFF) == 0x008, (
        f"Vector offset = 0x{fmt & 0xFFF:03X}, expected 0x008 (vector 2); "
        f"format=0x{fmt >> 12:X}"
    )
    assert d0 == 0x5A5AA5A5, (
        f"Rerun read returned 0x{d0:08X}, expected 0x5A5AA5A5"
    )


@cocotb.test()
async def test_berr_probe_control_no_fault(dut):
    """Control: same program with no fault injected must run clean."""
    h = CPUTestHarness(dut)
    bus = BerrOnceBusModel(dut, h.mem, times=0)

    clock = cocotb.clock.Clock(dut.CLK, 10, unit="ns")
    cocotb.start_soon(clock.start())
    h._init_idle_inputs()
    h._load_memory(PROGRAM)
    h.mem.load_long(2 * 4, HANDLER_BASE)
    h.mem.load_words(HANDLER_BASE, BERR_HANDLER)
    h.mem.load_long(COUNTER_ADDR, 0)
    h.mem.load_long(FAULT_ADDR, 0x5A5AA5A5)
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

    found = False
    for _ in range(8000):
        await RisingEdge(dut.CLK)
        if h.mem.read(h.SENTINEL_ADDR, 4) == h.SENTINEL_VAL:
            found = True
            break
    assert found, "control run did not complete"
    assert h.mem.read(COUNTER_ADDR, 4) == 0, "handler ran without fault"
    d0 = h.mem.read(D0_ADDR, 4)
    assert d0 == 0x5A5AA5A5, f"control run stored 0x{d0:08X}"
