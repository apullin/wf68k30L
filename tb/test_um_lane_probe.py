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
from m68k_encode import (
    moveq, move, move_from_abs_long, move_to_abs_long, imm_long,
    BYTE, WORD, LONG, DN, SPECIAL, IMMEDIATE,
)

RES = 0x020000

# Data window served by a narrow port; instruction space stays 32 bits wide.
NARROW_BASE = 0x018000
NARROW_SIZE = 0x001000


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


# ===================================================================
# Dynamic bus sizing: 16-bit and 8-bit ports (UM 7.2.1, Tables 7-4/7-5)
# ===================================================================

class NarrowPortBusModel(BusModel):
    """Narrow data window inside an otherwise 32-bit system.

    Keeping instruction space 32 bits wide isolates the dynamic-bus-sizing
    paths under test from opcode-fetch timing.
    """

    def _port_width_for(self, addr):
        if NARROW_BASE <= addr < NARROW_BASE + NARROW_SIZE:
            return self.port_width
        return 4


# Every byte in the window is distinct within its long word, and distinct
# from BusModel.POISON_BYTE, so a wrong lane is always visible.
def _fill_window(mem):
    for a in range(NARROW_BASE, NARROW_BASE + 0x600):
        mem.write(a, 1, a & 0xFF)


SIZES = ((LONG, 4), (WORD, 2), (BYTE, 1))

RD_WINDOW = NARROW_BASE + 0x100
WR_WINDOW = NARROW_BASE + 0x400

# Write payloads, one per (size, alignment) case; low bytes carry the operand.
WR_PATTERNS = [
    0x8191A1B1, 0x8292A2B2, 0x8393A3B3, 0x8494A4B4,
    0x8595A5B5, 0x8696A6B6, 0x8797A7B7, 0x8898A8B8,
    0x8999A9B9, 0x8A9AAABA, 0x8B9BABBB, 0x8C9CACBC,
]


def _case_addr(window, idx, align):
    """Distinct, non-overlapping address whose A1:A0 equals align."""
    return window + 0x20 * idx + align


async def _run_narrow(dut, port_width, program_fn, max_cycles=40000):
    """Bring the CPU up against a narrow-window bus and run to the sentinel."""
    h = CPUTestHarness(dut)
    bus = NarrowPortBusModel(dut, h.mem, port_width=port_width)

    clock = cocotb.clock.Clock(dut.CLK, 10, unit="ns")
    cocotb.start_soon(clock.start())
    h._init_idle_inputs()
    h._load_memory(program_fn(h))
    _fill_window(h.mem)
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
    for _ in range(max_cycles):
        await RisingEdge(dut.CLK)
        if h.mem.read(h.SENTINEL_ADDR, 4) == h.SENTINEL_VAL:
            found = True
            break
    return h, found


def _read_program(h):
    """MOVE.<size> (narrow).L,D0 -> MOVE.L D0,(RESULT).L for every case."""
    program = []
    for idx in range(len(SIZES) * 4):
        size = SIZES[idx // 4][0]
        align = idx % 4
        src = _case_addr(RD_WINDOW, idx, align)
        program += moveq(0, 0)  # zero-extend sub-long results
        program += move_from_abs_long(size, src, DN, 0)
        program += move_to_abs_long(LONG, DN, 0, h.RESULT_BASE + 4 * idx)
    program += h.sentinel_program()
    return program


def _write_program(h):
    """MOVE.L #pat,D0 -> MOVE.<size> D0,(narrow).L for every case."""
    program = []
    for idx in range(len(SIZES) * 4):
        size = SIZES[idx // 4][0]
        align = idx % 4
        dst = _case_addr(WR_WINDOW, idx, align)
        program += move(LONG, SPECIAL, IMMEDIATE, DN, 0)
        program += imm_long(WR_PATTERNS[idx])
        program += move_to_abs_long(size, DN, 0, dst)
    program += h.sentinel_program()
    return program


def _check_reads(h, port_width):
    errs = []
    for idx in range(len(SIZES) * 4):
        size, nbytes = SIZES[idx // 4]
        align = idx % 4
        src = _case_addr(RD_WINDOW, idx, align)
        want = 0
        for i in range(nbytes):
            want = (want << 8) | (h.mem.read(src + i, 1) & 0xFF)
        got = h.read_result_long(4 * idx)
        if got != want:
            errs.append(
                f"{8 * port_width}-bit port read size={nbytes} a={align} "
                f"addr=0x{src:08X}: got 0x{got:08X} want 0x{want:08X}"
            )
    return errs


def _check_writes(h, port_width):
    errs = []
    for idx in range(len(SIZES) * 4):
        size, nbytes = SIZES[idx // 4]
        align = idx % 4
        dst = _case_addr(WR_WINDOW, idx, align)
        want = WR_PATTERNS[idx] & ((1 << (8 * nbytes)) - 1)
        got = h.mem.read(dst, nbytes)
        if got != want:
            errs.append(
                f"{8 * port_width}-bit port write size={nbytes} a={align} "
                f"addr=0x{dst:08X}: got 0x{got:0{2 * nbytes}X} "
                f"want 0x{want:0{2 * nbytes}X}"
            )
    return errs


class SyncTermBusModel(BusModel):
    """32-bit port that terminates every cycle with STERMn (UM 7.3.2).

    STERMn is held asserted the way a zero-wait-state SRAM region does: it
    has setup and hold requirements against every rising edge while ASn is
    asserted, so it must already be valid at the edge that ends S1.
    """

    def __init__(self, dut, memory, **kwargs):
        super().__init__(dut, memory, sync_term=True, **kwargs)

    async def start(self):
        self.dut.STERMn.value = 0
        await super().start()

    def _negate_term(self):
        self.dut.DSACKn.value = 0b11


async def _run_sync(dut, program_fn, watch=None, max_cycles=40000):
    """Run against a STERM port, collecting sub-cycle addresses from `watch` on."""
    h = CPUTestHarness(dut)
    bus = SyncTermBusModel(dut, h.mem)

    clock = cocotb.clock.Clock(dut.CLK, 10, unit="ns")
    cocotb.start_soon(clock.start())
    h._init_idle_inputs()
    h._load_memory(program_fn(h))
    _fill_window(h.mem)
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
    addrs = []
    prev_as = 1
    for _ in range(max_cycles):
        await RisingEdge(dut.CLK)
        if watch is not None:
            try:
                as_n = int(dut.ASn.value)
                addr = int(dut.ADR_OUT.value)
            except ValueError:
                as_n, addr = 1, 0
            if prev_as == 1 and as_n == 0 and (addrs or addr == watch):
                addrs.append(addr)
            prev_as = as_n
        if h.mem.read(h.SENTINEL_ADDR, 4) == h.SENTINEL_VAL:
            found = True
            break
    return h, found, addrs


@cocotb.test()
async def test_sync_term_aligned_transfers(dut):
    """Aligned long/word/byte reads and writes on a STERM-terminated port."""
    src = RD_WINDOW + 0x40
    dst = WR_WINDOW + 0x40
    pattern = 0x8797A7B7

    def program(h):
        words = []
        for i, (size, _n) in enumerate(SIZES):
            words += moveq(0, 0)
            words += move_from_abs_long(size, src, DN, 0)
            words += move_to_abs_long(LONG, DN, 0, h.RESULT_BASE + 4 * i)
        for i, (size, _n) in enumerate(SIZES):
            words += move(LONG, SPECIAL, IMMEDIATE, DN, 0)
            words += imm_long(pattern)
            words += move_to_abs_long(size, DN, 0, dst + 8 * i)
        return words + h.sentinel_program()

    h, found, _ = await _run_sync(dut, program)
    assert found, "program did not complete"

    errs = []
    for i, (_size, nbytes) in enumerate(SIZES):
        want = 0
        for k in range(nbytes):
            want = (want << 8) | (h.mem.read(src + k, 1) & 0xFF)
        got = h.read_result_long(4 * i)
        if got != want:
            errs.append(f"read size={nbytes}: got 0x{got:08X} want 0x{want:08X}")
    for i, (_size, nbytes) in enumerate(SIZES):
        want = pattern & ((1 << (8 * nbytes)) - 1)
        got = h.mem.read(dst + 8 * i, nbytes)
        if got != want:
            errs.append(f"write size={nbytes}: got 0x{got:X} want 0x{want:X}")
    assert not errs, "STERM aligned transfer errors: " + "; ".join(errs)
    h.cleanup()


@cocotb.test()
async def test_sync_term_misaligned_address_step(dut):
    """A synchronous cycle terminates on the same edge that computes its step.

    The step therefore has to be applied within that cycle: the second
    sub-cycle of a long read at A1:A0=01 must address base + 3. Only the
    address sequence is checked here -- assembling the operand additionally
    depends on the SIZE_N countdown, which does not reach zero correctly on
    a synchronous split transfer.
    """
    target = RD_WINDOW + 0x21  # A1:A0 = 01

    def program(h):
        return [
            *move_from_abs_long(LONG, target, DN, 0),
            *move_to_abs_long(LONG, DN, 0, h.RESULT_BASE),
            *h.sentinel_program(),
        ]

    h, found, addrs = await _run_sync(dut, program, watch=target)
    assert found, "program did not complete"
    assert len(addrs) >= 2, (
        f"expected a split transfer, saw {[hex(a) for a in addrs]}"
    )
    assert addrs[1] == target + 3, (
        f"second sub-cycle addressed 0x{addrs[1]:08X}, expected "
        f"0x{target + 3:08X}: the address step lagged by one cycle"
    )
    h.cleanup()


@cocotb.test()
async def test_word_port_reads_all_alignments(dut):
    """Long/word/byte reads at every alignment from a 16-bit port."""
    h, found = await _run_narrow(dut, 2, _read_program)
    assert found, "program did not complete"
    errs = _check_reads(h, 2)
    assert not errs, "16-bit port read errors: " + "; ".join(errs)
    h.cleanup()


@cocotb.test()
async def test_word_port_writes_all_alignments(dut):
    """Long/word/byte writes at every alignment to a 16-bit port."""
    h, found = await _run_narrow(dut, 2, _write_program)
    assert found, "program did not complete"
    errs = _check_writes(h, 2)
    assert not errs, "16-bit port write errors: " + "; ".join(errs)
    h.cleanup()


@cocotb.test()
async def test_byte_port_reads_all_alignments(dut):
    """Long/word/byte reads at every alignment from an 8-bit port."""
    h, found = await _run_narrow(dut, 1, _read_program)
    assert found, "program did not complete"
    errs = _check_reads(h, 1)
    assert not errs, "8-bit port read errors: " + "; ".join(errs)
    h.cleanup()


@cocotb.test()
async def test_byte_port_writes_all_alignments(dut):
    """Long/word/byte writes at every alignment to an 8-bit port."""
    h, found = await _run_narrow(dut, 1, _write_program)
    assert found, "program did not complete"
    errs = _check_writes(h, 1)
    assert not errs, "8-bit port write errors: " + "; ".join(errs)
    h.cleanup()
