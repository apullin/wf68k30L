"""
Audit probe: misaligned long reads against a UM-compliant 32-bit port.

Models the most common real-world 32-bit memory: on every read it drives
the full aligned longword on D31:0 (each byte on its address-matched lane,
as MC68030UM Table 7-4 requires). Writes sample address-matched lanes like
the stock model. If the RTL's read-lane muxing follows the UM, all reads
work; if it expects 'first requested byte on the top lane' (the convention
the stock bus model codifies), misaligned long reads return wrong data.
"""

import cocotb
from cocotb.triggers import RisingEdge, ClockCycles

from cpu_harness import CPUTestHarness
from bus_model import BusModel

RES = 0x020000


class Sram32BusModel(BusModel):
    """UM Table 7-4 compliant 32-bit port: reads drive the full aligned long."""

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
                    rw_n = int(self.dut.RWn.value)
                    addr = int(self.dut.ADR_OUT.value)
                except ValueError:
                    self.dut.DSACKn.value = 0b11
                    continue
                if rw_n == 1:
                    base = addr & ~3
                    data = 0
                    for i in range(4):
                        data |= (self.memory.read(base + i, 1) & 0xFF) << ((3 - i) * 8)
                    self.dut.DATA_IN.value = data
                    self.dut.DSACKn.value = 0b00
                else:
                    size_code = self._get_size_code()
                    start_lane, byte_count = self._cycle_layout(
                        addr, size_code, is_write=True
                    )
                    try:
                        data = int(self.dut.DATA_OUT.value)
                    except ValueError:
                        data = 0
                    for i in range(byte_count):
                        self.memory.write(
                            addr + i, 1,
                            (data >> ((3 - (start_lane + i)) * 8)) & 0xFF,
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


PROGRAM = [
    0x2039, 0x0001, 0x0000,   # MOVE.L ($10000).L,D0   (aligned)
    0x23C0, 0x0002, 0x0000,   # MOVE.L D0,($20000).L
    0x2039, 0x0001, 0x0011,   # MOVE.L ($10011).L,D0   (a=1)
    0x23C0, 0x0002, 0x0004,   # MOVE.L D0,($20004).L
    0x2039, 0x0001, 0x0022,   # MOVE.L ($10022).L,D0   (a=2)
    0x23C0, 0x0002, 0x0008,   # MOVE.L D0,($20008).L
    0x2039, 0x0001, 0x0033,   # MOVE.L ($10033).L,D0   (a=3)
    0x23C0, 0x0002, 0x000C,   # MOVE.L D0,($2000C).L
    0x3039, 0x0001, 0x0042,   # MOVE.W ($10042).L,D0   (word a=2, control)
    0x33C0, 0x0002, 0x0010,   # MOVE.W D0,($20010).L
    0x2E3C, 0xDEAD, 0xCAFE,   # sentinel
    0x23C7, 0x0003, 0x0000,
    0x60FE,
]


@cocotb.test()
async def test_misaligned_long_reads_um_port(dut):
    """All four alignments of MOVE.L (abs).L,Dn against a UM-true 32-bit port."""
    h = CPUTestHarness(dut)
    bus = Sram32BusModel(dut, h.mem)

    clock = cocotb.clock.Clock(dut.CLK, 10, unit="ns")
    cocotb.start_soon(clock.start())
    h._init_idle_inputs()
    h._load_memory(PROGRAM)
    # known pattern: byte value = low 8 address bits
    for a in range(0x10000, 0x10050):
        h.mem.write(a, 1, a & 0xFF)
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
    assert found, "program did not complete"

    exp = {
        0x0: 0x00010203,   # from 0x10000: bytes 00 01 02 03
        0x4: 0x11121314,   # from 0x10011: bytes 11 12 13 14
        0x8: 0x22232425,   # from 0x10022
        0xC: 0x33343536,   # from 0x10033
    }
    errs = []
    for off, want in exp.items():
        got = h.mem.read(RES + off, 4)
        if got != want:
            errs.append(f"offset a={off >> 2 if off else 0} read: got 0x{got:08X} want 0x{want:08X}")
    w = h.mem.read(RES + 0x10, 2)
    if w != 0x4243:
        errs.append(f"word a=2 control: got 0x{w:04X} want 0x4243")
    assert not errs, "UM-compliant port misreads: " + "; ".join(errs)
