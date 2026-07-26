"""
Audit probe: hardware interrupt paths (autovectored + vectored + RTE resume).

These paths have zero coverage in the existing suite (IPLn is tied to
'no interrupt' everywhere). This module drives IPLn directly and services
the resulting interrupt-acknowledge (FC=7) cycles, which the stock
BusModel does not understand.
"""

import cocotb
from cocotb.triggers import RisingEdge, ClockCycles

from cpu_harness import CPUTestHarness
from bus_model import BusModel


class IackBusModel(BusModel):
    """BusModel that recognizes CPU-space (FC=7) interrupt-acknowledge cycles.

    use_avec=True  -> assert AVECn (autovector) for IACK cycles.
    use_avec=False -> supply `vector` on all byte lanes with DSACK (vectored).
    """

    def __init__(self, dut, memory, wait_states=0, use_avec=True, vector=None):
        super().__init__(dut, memory, wait_states)
        self.use_avec = use_avec
        self.vector = vector
        self.iack_count = 0
        self.iack_levels = []

    async def _responder(self):
        while self._running:
            await RisingEdge(self.dut.CLK)

            try:
                as_n = int(self.dut.ASn.value)
                fc = int(self.dut.FC_OUT.value)
            except ValueError:
                self.dut.DSACKn.value = 0b11
                continue

            if as_n == 0 and fc == 7:
                # Interrupt acknowledge (CPU space) cycle.
                try:
                    addr = int(self.dut.ADR_OUT.value)
                except ValueError:
                    addr = 0
                self.iack_count += 1
                self.iack_levels.append((addr >> 1) & 0x7)

                for _ in range(self.wait_states):
                    await RisingEdge(self.dut.CLK)

                if self.use_avec:
                    self.dut.AVECn.value = 0
                else:
                    v = self.vector & 0xFF
                    self.dut.DATA_IN.value = (v << 24) | (v << 16) | (v << 8) | v
                    self.dut.DSACKn.value = 0b00

                while self._running:
                    await RisingEdge(self.dut.CLK)
                    try:
                        if int(self.dut.ASn.value) == 1:
                            break
                    except ValueError:
                        break
                self.dut.AVECn.value = 1
                self.dut.DSACKn.value = 0b11
                continue

            if as_n == 0:
                # Normal memory cycle: reuse the parent implementation inline.
                try:
                    rw_n = int(self.dut.RWn.value)
                except ValueError:
                    self.dut.DSACKn.value = 0b11
                    continue
                try:
                    addr = int(self.dut.ADR_OUT.value)
                except ValueError:
                    addr = 0

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
                        lane = start_lane + i
                        data |= b << ((3 - lane) * 8)
                    self.dut.DATA_IN.value = data
                    self.dut.DSACKn.value = 0b00
                else:
                    try:
                        data = int(self.dut.DATA_OUT.value)
                    except ValueError:
                        data = 0
                    for i in range(byte_count):
                        lane = start_lane + i
                        self.memory.write(addr + i, 1, (data >> ((3 - lane) * 8)) & 0xFF)
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


HANDLER_BASE = 0x000800
COUNTER_ADDR = 0x020000
MARKER_ADDR = 0x020004

# main: MOVE #$2000,SR ; spin: BRA.S spin
MAIN_PROGRAM = [0x46FC, 0x2000, 0x60FE]

# autovector handler: write marker, write sentinel, STOP #$2700
AVEC_HANDLER = [
    0x203C, 0xCAFE, 0xD00D,          # MOVE.L #$CAFED00D,D0
    0x23C0, 0x0002, 0x0004,          # MOVE.L D0,($20004).L
    0x2E3C, 0xDEAD, 0xCAFE,          # MOVE.L #$DEADCAFE,D7
    0x23C7, 0x0003, 0x0000,          # MOVE.L D7,($30000).L
    0x4E72, 0x2700,                  # STOP #$2700
]

# vectored handler: increment counter, RTE
VEC_HANDLER = [
    0x2039, 0x0002, 0x0000,          # MOVE.L ($20000).L,D0
    0x5280,                          # ADDQ.L #1,D0
    0x23C0, 0x0002, 0x0000,          # MOVE.L D0,($20000).L
    0x4E73,                          # RTE
]

USER_VECTOR = 0xC0  # vector table slot 0x300, clear of program/sled


async def _setup(h, bus, handler_words, vector_addr):
    clock = cocotb.clock.Clock(h.dut.CLK, 10, unit="ns")
    cocotb.start_soon(clock.start())
    h._init_idle_inputs()
    h._load_memory(MAIN_PROGRAM)
    h.mem.load_long(vector_addr, HANDLER_BASE)
    h.mem.load_words(HANDLER_BASE, handler_words)
    h.mem.load_long(COUNTER_ADDR, 0)
    await RisingEdge(h.dut.CLK)
    await RisingEdge(h.dut.CLK)
    h.dut.RESET_INn.value = 0
    h.dut.HALT_INn.value = 0
    await ClockCycles(h.dut.CLK, 20)
    h.bus = bus
    await bus.start()
    h.dut.RESET_INn.value = 1
    h.dut.HALT_INn.value = 1
    await ClockCycles(h.dut.CLK, 4)


@cocotb.test()
async def test_autovector_level2_interrupt(dut):
    """Assert IPL level 2, service IACK with AVECn, expect handler to run."""
    h = CPUTestHarness(dut)
    bus = IackBusModel(dut, h.mem, use_avec=True)
    await _setup(h, bus, AVEC_HANDLER, 26 * 4)  # level-2 autovector = 26

    # Let the main program lower the interrupt mask and start spinning.
    await ClockCycles(dut.CLK, 300)

    dut.IPLn.value = 0b101  # level 2 (active low)

    ipend_seen = False
    found = False
    for _ in range(4000):
        await RisingEdge(dut.CLK)
        try:
            if int(dut.IPENDn.value) == 0:
                ipend_seen = True
        except ValueError:
            pass
        if h.mem.read(h.SENTINEL_ADDR, 4) == h.SENTINEL_VAL:
            found = True
            break

    assert found, (
        f"Interrupt handler never completed (IACK cycles seen: {bus.iack_count}, "
        f"IPENDn observed low: {ipend_seen})"
    )
    assert bus.iack_count == 1, f"Expected 1 IACK cycle, saw {bus.iack_count}"
    assert bus.iack_levels == [2], f"IACK level encoding wrong: {bus.iack_levels}"
    assert ipend_seen, "IPENDn never asserted"
    marker = h.mem.read(MARKER_ADDR, 4)
    assert marker == 0xCAFED00D, f"Handler marker wrong: 0x{marker:08X}"


@cocotb.test()
async def test_vectored_interrupt_rte_resume_twice(dut):
    """Vectored interrupt (user vector 0xC0), RTE resume, then re-interrupt."""
    h = CPUTestHarness(dut)
    bus = IackBusModel(dut, h.mem, use_avec=False, vector=USER_VECTOR)
    await _setup(h, bus, VEC_HANDLER, USER_VECTOR * 4)

    await ClockCycles(dut.CLK, 300)

    dut.IPLn.value = 0b100  # level 3

    count = 0
    for _ in range(4000):
        await RisingEdge(dut.CLK)
        if h.mem.read(COUNTER_ADDR, 4) == 1:
            count = 1
            break
    assert count == 1, (
        f"First vectored interrupt not serviced (IACK cycles: {bus.iack_count})"
    )

    dut.IPLn.value = 0b111  # deassert; let RTE resume the spin loop
    await ClockCycles(dut.CLK, 200)

    dut.IPLn.value = 0b100  # level 3 again
    ok = False
    for _ in range(4000):
        await RisingEdge(dut.CLK)
        if h.mem.read(COUNTER_ADDR, 4) == 2:
            ok = True
            break
    dut.IPLn.value = 0b111

    assert ok, (
        f"Second interrupt after RTE resume not serviced "
        f"(IACK cycles total: {bus.iack_count}, counter="
        f"{h.mem.read(COUNTER_ADDR, 4)})"
    )
    assert bus.iack_count == 2, f"Expected 2 IACK cycles, saw {bus.iack_count}"
    assert bus.iack_levels == [3, 3], f"IACK level encoding: {bus.iack_levels}"


@cocotb.test()
async def test_level7_edge_semantics(dut):
    """Level 7 is transition-sensitive when the mask stays at 7.

    UM 8.1.9: level 7 is nonmaskable — a transition TO 7 interrupts even
    with the mask at 7. But while IPL is HELD at 7 and the mask stays 7
    (handler RTEs back to a mask-7 SR), no second interrupt may occur.
    (The mask-lowering retrigger case — RTE to a mask<7 SR with IPL held —
    is legitimately retriggering and verified implicitly by the first two
    tests' setup.) A fresh transition (drop below 7, back to 7) must retake.
    """
    h = CPUTestHarness(dut)
    bus = IackBusModel(dut, h.mem, use_avec=True)
    # Main program variant that NEVER lowers the mask: reset SR has mask 7,
    # so the spin loop runs at mask 7 and RTE restores mask 7.
    global MAIN_PROGRAM
    saved_main = MAIN_PROGRAM
    MAIN_PROGRAM = [0x60FE]  # spin: BRA.S spin (mask stays 7)
    try:
        await _setup(h, bus, VEC_HANDLER, 31 * 4)  # level-7 autovector = 31
    finally:
        MAIN_PROGRAM = saved_main

    await ClockCycles(dut.CLK, 300)

    dut.IPLn.value = 0b000  # level 7 (active low), held
    serviced = False
    for _ in range(4000):
        await RisingEdge(dut.CLK)
        if h.mem.read(COUNTER_ADDR, 4) == 1:
            serviced = True
            break
    assert serviced, f"Level-7 interrupt not serviced (IACK: {bus.iack_count})"

    # Hold IPL at 7 through and after RTE: must NOT be serviced again.
    await ClockCycles(dut.CLK, 1500)
    count_held = h.mem.read(COUNTER_ADDR, 4)

    # Now create a new transition: drop below 7, then back to 7.
    dut.IPLn.value = 0b111
    await ClockCycles(dut.CLK, 100)
    dut.IPLn.value = 0b000
    retriggered = False
    for _ in range(4000):
        await RisingEdge(dut.CLK)
        if h.mem.read(COUNTER_ADDR, 4) == count_held + 1:
            retriggered = True
            break
    dut.IPLn.value = 0b111

    assert count_held == 1, (
        f"Level-7 retriggered while held (counter={count_held}, "
        f"IACK cycles={bus.iack_count}) — must be transition-sensitive"
    )
    assert retriggered, "Level-7 not retaken after a fresh transition"


@cocotb.test()
async def test_stop_wakes_on_interrupt(dut):
    """STOP #$2000 must wake on an interrupt above the mask and continue."""
    h = CPUTestHarness(dut)
    bus = IackBusModel(dut, h.mem, use_avec=True)
    global MAIN_PROGRAM
    saved_main = MAIN_PROGRAM
    # STOP with mask 0, then (after wake + handler RTE) write sentinel, spin.
    MAIN_PROGRAM = [
        0x4E72, 0x2000,              # STOP #$2000
        0x2E3C, 0xDEAD, 0xCAFE,      # MOVE.L #$DEADCAFE,D7
        0x23C7, 0x0003, 0x0000,      # MOVE.L D7,($30000).L
        0x60FE,                      # BRA.S self
    ]
    try:
        await _setup(h, bus, VEC_HANDLER, 26 * 4)  # level-2 autovector
    finally:
        MAIN_PROGRAM = saved_main

    # Let it reach STOP and idle there a while.
    await ClockCycles(dut.CLK, 500)
    assert h.mem.read(h.SENTINEL_ADDR, 4) != h.SENTINEL_VAL, \
        "Sentinel written before interrupt: STOP fell through"

    dut.IPLn.value = 0b101  # level 2
    found = False
    for _ in range(4000):
        await RisingEdge(dut.CLK)
        if h.mem.read(COUNTER_ADDR, 4) == 1 and int(dut.IPENDn.value) == 1:
            dut.IPLn.value = 0b111  # deassert once serviced
        if h.mem.read(h.SENTINEL_ADDR, 4) == h.SENTINEL_VAL:
            found = True
            break
    dut.IPLn.value = 0b111
    assert h.mem.read(COUNTER_ADDR, 4) >= 1, \
        f"STOP never woke on interrupt (IACK: {bus.iack_count})"
    assert found, "Execution did not continue past STOP after handler RTE"
