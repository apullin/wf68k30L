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
        self.faulted = []
        self.log = []

    def _should_fault(self, addr, fc):
        return (addr & ~3) == (self.fault_addr & ~3)

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
                if self._should_fault(addr, fc) and self.faults_left > 0:
                    self.faults_left -= 1
                    self.fault_count += 1
                    self.faulted.append((addr, fc))
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
    # The SSW at frame +0x0A must describe the faulted cycle, not whatever bus
    # cycle happened to run next: DF (bit 8) set, RW (bit 6) set for a read,
    # FC (bits 2:0) = 5 (supervisor data).
    ssw = frame[0x0A // 2]
    assert ssw & 0x0100, (
        f"SSW = 0x{ssw:04X}: DF clear, so RTE will not rerun the faulted access"
    )
    assert (ssw & 0x0007) == 5 and (ssw & 0x0040), (
        f"SSW = 0x{ssw:04X}: expected a supervisor-data read (RW=1, FC=5); "
        f"FC=6 means the SSW was overwritten by a later opcode prefetch"
    )
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
    # UM 8.1.2: the stacked PC is the logical address of the instruction that
    # was executing. RTE reruns the faulted access by re-executing from that
    # address, so any other value silently drops the access.
    stacked_pc = (frame[1] << 16) | frame[2]
    assert stacked_pc == h.PROGRAM_BASE, (
        f"Stacked PC = 0x{stacked_pc:08X}, expected 0x{h.PROGRAM_BASE:08X} "
        f"(the faulting instruction)"
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


class BerrOnIackBusModel(BerrOnceBusModel):
    """Terminates every interrupt-acknowledge (FC=7) cycle with BERRn."""

    def _should_fault(self, addr, fc):
        return fc == 7


MARK_ADDR = 0x020020


def _marker_handler(marker):
    return [
        0x203C, 0x0000, marker,      # MOVE.L #marker,D0
        0x23C0, 0x0002, 0x0020,      # MOVE.L D0,($20020).L
        0x2E3C, 0xDEAD, 0xCAFE,      # MOVE.L #$DEADCAFE,D7
        0x23C7, 0x0003, 0x0000,      # MOVE.L D7,($30000).L
        0x4E72, 0x2700,              # STOP #$2700
    ]


@cocotb.test()
async def test_berr_on_iack_is_spurious_only(dut):
    """UM 8.1.9: a bus error during the interrupt acknowledge cycle makes the
    interrupt spurious (vector 24). It must not also raise a bus error."""
    h = CPUTestHarness(dut)
    bus = BerrOnIackBusModel(dut, h.mem, times=1)

    spurious_handler = _marker_handler(0x0018)
    berr_handler = _marker_handler(0x0002)

    clock = cocotb.clock.Clock(dut.CLK, 10, unit="ns")
    cocotb.start_soon(clock.start())
    h._init_idle_inputs()
    h._load_memory([0x46FC, 0x2000, 0x60FE])   # MOVE #$2000,SR ; BRA.S self
    h.mem.load_long(2 * 4, 0x000900)           # vector 2  (bus error)
    h.mem.load_words(0x000900, berr_handler)
    h.mem.load_long(24 * 4, 0x000800)          # vector 24 (spurious interrupt)
    h.mem.load_words(0x000800, spurious_handler)
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

    dut.IPLn.value = 0b101                     # level 2 (active low)

    found = False
    for _ in range(6000):
        await RisingEdge(dut.CLK)
        if h.mem.read(h.SENTINEL_ADDR, 4) == h.SENTINEL_VAL:
            found = True
            break
    dut.IPLn.value = 0b111

    marker = h.mem.read(MARK_ADDR, 4)
    assert bus.fault_count == 1, (
        f"BERRn asserted on {bus.fault_count} IACK cycles (expected 1)"
    )
    assert found, f"neither handler completed (marker=0x{marker:08X})"
    assert marker == 0x0018, (
        f"handler marker = 0x{marker:08X}; expected 0x00000018 (spurious "
        f"interrupt, vector 24). 0x00000002 means a phantom bus-error "
        f"exception ran as well."
    )


CODE_FAULT_ADDR = 0x0E0000
PC_ADDR = 0x02000C
SSW_ADDR = 0x020010
STORE_WITNESS = 0x020018

# vec 2 handler for fetch faults: record format word, stacked PC and SSW, then
# finish. The faulted instruction word is gone, so this one does not RTE.
FETCH_FAULT_HANDLER = [
    0x322F, 0x0006,                  # MOVE.W 6(A7),D1
    0x33C1, 0x0002, 0x0008,          # MOVE.W D1,($20008).L
    0x222F, 0x0002,                  # MOVE.L 2(A7),D1
    0x23C1, 0x0002, 0x000C,          # MOVE.L D1,($2000C).L
    0x342F, 0x000A,                  # MOVE.W 10(A7),D2
    0x33C2, 0x0002, 0x0010,          # MOVE.W D2,($20010).L
    0x2E3C, 0xDEAD, 0xCAFE,          # MOVE.L #$DEADCAFE,D7
    0x23C7, 0x0003, 0x0000,          # MOVE.L D7,($30000).L
    0x4E72, 0x2700,                  # STOP #$2700
]

# vec 2 handler that reruns: counter++, record stacked PC, RTE. Touches only
# D0/D1 so the faulted store's source register survives the rerun.
STORE_FAULT_HANDLER = [
    0x2039, 0x0002, 0x0000,          # MOVE.L ($20000).L,D0
    0x5280,                          # ADDQ.L #1,D0
    0x23C0, 0x0002, 0x0000,          # MOVE.L D0,($20000).L
    0x222F, 0x0002,                  # MOVE.L 2(A7),D1
    0x23C1, 0x0002, 0x000C,          # MOVE.L D1,($2000C).L
    0x4E73,                          # RTE
]


async def _boot(dut, h, bus, program, handler):
    clock = cocotb.clock.Clock(dut.CLK, 10, unit="ns")
    cocotb.start_soon(clock.start())
    h._init_idle_inputs()
    h._load_memory(program)
    h.mem.load_long(2 * 4, HANDLER_BASE)          # vector 2
    h.mem.load_words(HANDLER_BASE, handler)
    h.mem.load_long(COUNTER_ADDR, 0)
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
    for _ in range(12000):
        await RisingEdge(dut.CLK)
        if h.mem.read(h.SENTINEL_ADDR, 4) == h.SENTINEL_VAL:
            return True
    return False


@cocotb.test()
async def test_berr_on_opcode_fetch_takes_vector_2(dut):
    """UM 8.1.2: a bus error on an instruction prefetch raises vector 2 once the
    processor attempts to use the prefetched word."""
    h = CPUTestHarness(dut)
    bus = BerrOnceBusModel(dut, h.mem, fault_addr=CODE_FAULT_ADDR, times=1)

    # The jump target's first opcode fetch is faulted; nothing there executes.
    program = [0x4EF9, 0x000E, 0x0000]            # JMP ($E0000).L
    h.mem.load_words(CODE_FAULT_ADDR, [0x4E71] * 8)

    found = await _boot(dut, h, bus, program, FETCH_FAULT_HANDLER)

    fmt = h.mem.read(FMT_ADDR, 2)
    pc = h.mem.read(PC_ADDR, 4)
    ssw = h.mem.read(SSW_ADDR, 2)
    assert bus.fault_count == 1, f"BERRn asserted {bus.fault_count} times"
    assert bus.faulted == [(CODE_FAULT_ADDR, 6)], (
        f"BERRn must land on a supervisor-program (FC=6) fetch of "
        f"0x{CODE_FAULT_ADDR:06X}, got {bus.faulted}"
    )
    assert found, (
        f"Bus-error handler never completed: the faulted prefetch was never "
        f"delivered as an exception (fmt=0x{fmt:04X})"
    )
    assert (fmt & 0xFFF) == 0x008, (
        f"Vector offset = 0x{fmt & 0xFFF:03X}, expected 0x008 (vector 2); "
        f"format=0x{fmt >> 12:X}"
    )
    # UM Table 8-7: a fault taken with the execution unit at an instruction
    # boundary uses the short bus fault frame, format $A.
    assert (fmt >> 12) == 0xA, (
        f"Frame format = 0x{fmt >> 12:X}, expected 0xA (short bus fault frame)"
    )
    assert pc == CODE_FAULT_ADDR, (
        f"Stacked PC = 0x{pc:08X}, expected 0x{CODE_FAULT_ADDR:08X} (the "
        f"instruction whose fetch faulted)"
    )
    # SSW: FC=6 (supervisor program), RW=1 (read), DF clear -- an instruction
    # fetch is not a data cycle, so there is nothing for RTE to rerun.
    assert (ssw & 0x0007) == 6, (
        f"SSW = 0x{ssw:04X}: expected FC=6 (supervisor program space)"
    )
    assert ssw & 0x0040, f"SSW = 0x{ssw:04X}: RW clear, but a fetch is a read"
    assert not (ssw & 0x0100), (
        f"SSW = 0x{ssw:04X}: DF set on an instruction fetch fault"
    )


@cocotb.test()
async def test_berr_on_speculative_prefetch_does_not_trap(dut):
    """UM 8.1.2: the exception may be delayed until the prefetched information
    is used, so a prefetch beyond a taken branch must never trap."""
    h = CPUTestHarness(dut)
    # The granule holding 0x108/0x10A is the branch shadow of the BRA at 0x106.
    bus = BerrOnceBusModel(dut, h.mem, fault_addr=0x000108, times=2)

    program = [
        0x203C, 0x1122, 0x3344,                   # 0x100 MOVE.L #$11223344,D0
        0x6018,                                   # 0x106 BRA.S ->0x120
    ] + [0x4E71] * 12 + [                         # 0x108..0x11E never executed
        0x23C0, 0x0002, 0x0020,                   # 0x120 MOVE.L D0,($20020).L
        0x2E3C, 0xDEAD, 0xCAFE,                   # 0x126 MOVE.L #$DEADCAFE,D7
        0x23C7, 0x0003, 0x0000,                   # 0x12C MOVE.L D7,($30000).L
        0x4E72, 0x2700,                           # 0x132 STOP #$2700
    ]

    found = await _boot(dut, h, bus, program, FETCH_FAULT_HANDLER)

    fmt = h.mem.read(FMT_ADDR, 2)
    assert bus.fault_count > 0, (
        "No prefetch was faulted, so this test proves nothing; the branch "
        "shadow was never prefetched"
    )
    assert all(a in (0x108, 0x10A) and fc == 6 for a, fc in bus.faulted), (
        f"BERRn was expected only on branch-shadow fetches, got {bus.faulted}"
    )
    assert fmt == 0, (
        f"A speculative prefetch fault raised an exception (frame word "
        f"0x{fmt:04X}); it must be discarded when the branch flushes the pipe"
    )
    assert found, "Program did not complete past the faulted branch shadow"
    assert h.mem.read(0x020020, 4) == 0x11223344, (
        "The instruction before the branch did not execute correctly"
    )


@cocotb.test()
async def test_berr_on_store_stacks_the_storing_instruction(dut):
    """UM 8.1.2: a faulted store stacks its own address, and RTE reruns the write.

    A short store (MOVE.L D3,(A0), CLR.L (A0), MOVE.L D0,(A0)+, any size) has the
    following instruction dispatched before its write is acknowledged -- PC_INC
    fires one cycle before ASn falls -- so the live PC already names the next
    instruction when the fault arrives. Stacking that PC is not merely cosmetic:
    RTE suppresses the pipe reload whenever the SSW asks for a rerun, so the core
    reruns by re-executing from the restored PC. A frame carrying 0x10E for a
    store at 0x10C therefore resumes *past* the store and discards the write
    outright.

    Fixed by carrying a writeback-stage PC (PC_WB in the top-level datapath
    helper) captured on ADR_MARK_USED, the same event that loads the
    writeback-stage effective address ADR_EFF_WB, and selecting it for the
    format $A/$B stacked PC when the faulted access was a core data write. PC
    itself still advances at dispatch, because PC_EW_BASE needs it there.

    This test therefore asserts the whole loop, not just the frame: the handler
    runs exactly once, the stacked PC is the storing instruction, and the target
    memory holds the stored value after the rerun.
    """
    h = CPUTestHarness(dut)
    bus = BerrOnceBusModel(dut, h.mem, times=1)

    store_addr = 0x00010C
    program = [
        0x207C, 0x000E, 0x0000,                   # 0x100 MOVEA.L #$E0000,A0
        0x263C, 0x5A5A, 0xA5A5,                   # 0x106 MOVE.L #$5A5AA5A5,D3
        0x2083,                                   # 0x10C MOVE.L D3,(A0)  <- BERR
        0x7421,                                   # 0x10E MOVEQ #$21,D2
        0x23C2, 0x0002, 0x0018,                   # 0x110 MOVE.L D2,($20018).L
        0x2E3C, 0xDEAD, 0xCAFE,                   # 0x116 MOVE.L #$DEADCAFE,D7
        0x23C7, 0x0003, 0x0000,                   # 0x11C MOVE.L D7,($30000).L
        0x4E72, 0x2700,                           # 0x122 STOP #$2700
    ]

    found = await _boot(dut, h, bus, program, STORE_FAULT_HANDLER)

    counter = h.mem.read(COUNTER_ADDR, 4)
    pc = h.mem.read(PC_ADDR, 4)
    assert bus.fault_count == 1, f"BERRn asserted {bus.fault_count} times"
    assert counter == 1, f"Bus-error handler ran {counter} times (expected 1)"
    assert pc == store_addr, (
        f"Stacked PC = 0x{pc:08X}, expected 0x{store_addr:08X}. A later value "
        f"means the following instruction was dispatched before the store "
        f"faulted, so RTE resumes past the store and discards it"
    )
    assert found, "Program did not complete after the RTE rerun"
    assert h.mem.read(FAULT_ADDR, 4) == 0x5A5AA5A5, (
        f"Faulted store was not rerun: 0x{FAULT_ADDR:06X} holds "
        f"0x{h.mem.read(FAULT_ADDR, 4):08X}"
    )
    assert h.mem.read(STORE_WITNESS, 4) == 0x21, (
        "Execution did not continue correctly after the RTE rerun"
    )


@cocotb.test()
async def test_berr_on_clr_stacks_the_clearing_instruction(dut):
    """The same rule holds for the other short-store forms, not just MOVE.

    CLR reaches its write through a different arm of the mark-used logic than
    MOVE does, and a word store is a different bus size, so both are checked
    here: whichever instruction owns the faulted write is the one the format
    $A/$B frame must name, and the write must land on the rerun.
    """
    h = CPUTestHarness(dut)
    bus = BerrOnceBusModel(dut, h.mem, times=1)

    store_addr = 0x000106
    program = [
        0x207C, 0x000E, 0x0000,                   # 0x100 MOVEA.L #$E0000,A0
        0x4290,                                   # 0x106 CLR.L (A0)  <- BERR
        0x7421,                                   # 0x108 MOVEQ #$21,D2
        0x23C2, 0x0002, 0x0018,                   # 0x10A MOVE.L D2,($20018).L
        0x2E3C, 0xDEAD, 0xCAFE,                   # 0x110 MOVE.L #$DEADCAFE,D7
        0x23C7, 0x0003, 0x0000,                   # 0x116 MOVE.L D7,($30000).L
        0x4E72, 0x2700,                           # 0x11C STOP #$2700
    ]
    # Preloaded so a completed CLR is distinguishable from a discarded one.
    h.mem.load_long(FAULT_ADDR, 0x5A5AA5A5)

    found = await _boot(dut, h, bus, program, STORE_FAULT_HANDLER)

    counter = h.mem.read(COUNTER_ADDR, 4)
    pc = h.mem.read(PC_ADDR, 4)
    assert bus.fault_count == 1, f"BERRn asserted {bus.fault_count} times"
    assert counter == 1, f"Bus-error handler ran {counter} times (expected 1)"
    assert pc == store_addr, (
        f"Stacked PC = 0x{pc:08X}, expected 0x{store_addr:08X} (the CLR)"
    )
    assert found, "Program did not complete after the RTE rerun"
    assert h.mem.read(FAULT_ADDR, 4) == 0, (
        f"Faulted CLR was not rerun: 0x{FAULT_ADDR:06X} still holds "
        f"0x{h.mem.read(FAULT_ADDR, 4):08X}"
    )
    assert h.mem.read(STORE_WITNESS, 4) == 0x21, (
        "Execution did not continue correctly after the RTE rerun"
    )


@cocotb.test()
async def test_berr_on_word_store_stacks_the_storing_instruction(dut):
    """A faulted word store must name itself too -- the defect was size-blind."""
    h = CPUTestHarness(dut)
    bus = BerrOnceBusModel(dut, h.mem, times=1)

    store_addr = 0x00010A
    program = [
        0x207C, 0x000E, 0x0000,                   # 0x100 MOVEA.L #$E0000,A0
        0x363C, 0x5A5A,                           # 0x106 MOVE.W #$5A5A,D3
        0x3083,                                   # 0x10A MOVE.W D3,(A0)  <- BERR
        0x7421,                                   # 0x10C MOVEQ #$21,D2
        0x23C2, 0x0002, 0x0018,                   # 0x10E MOVE.L D2,($20018).L
        0x2E3C, 0xDEAD, 0xCAFE,                   # 0x114 MOVE.L #$DEADCAFE,D7
        0x23C7, 0x0003, 0x0000,                   # 0x11A MOVE.L D7,($30000).L
        0x4E72, 0x2700,                           # 0x120 STOP #$2700
    ]

    found = await _boot(dut, h, bus, program, STORE_FAULT_HANDLER)

    counter = h.mem.read(COUNTER_ADDR, 4)
    pc = h.mem.read(PC_ADDR, 4)
    assert bus.fault_count == 1, f"BERRn asserted {bus.fault_count} times"
    assert counter == 1, f"Bus-error handler ran {counter} times (expected 1)"
    assert pc == store_addr, (
        f"Stacked PC = 0x{pc:08X}, expected 0x{store_addr:08X} (the MOVE.W)"
    )
    assert found, "Program did not complete after the RTE rerun"
    assert h.mem.read(FAULT_ADDR, 2) == 0x5A5A, (
        f"Faulted word store was not rerun: 0x{FAULT_ADDR:06X} holds "
        f"0x{h.mem.read(FAULT_ADDR, 2):04X}"
    )
    assert h.mem.read(STORE_WITNESS, 4) == 0x21, (
        "Execution did not continue correctly after the RTE rerun"
    )
