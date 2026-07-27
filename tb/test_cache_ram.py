"""
Unit test for WF68K30L_SYNC_RAM_1R1W, the cache data store.

The contract the cache state machines rely on:
  - The read port is registered: data addressed while RD_EN is high is readable
    in the following cycle, and it holds while RD_EN is low.
  - A read and a write in the same cycle at the same address returns the value
    being written, not the one being replaced.

The second clause is load bearing. Both caches set an entry's valid bit on the
same edge that registers its RAM write strobe, so the cycle in which the write is
presented to this RAM is already a cycle in which a lookup can see the entry as
valid and address it. A read-before-write RAM hands that lookup the entry's
pre-fill content -- for an instruction fetch, a word that was never in memory.

Set TOPLEVEL=WF68K30L_SYNC_RAM_1R1W when running this test.
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge


def _idle(dut):
    dut.RD_EN.value = 0
    dut.RD_ADDR.value = 0
    dut.WR_EN.value = 0
    dut.WR_ADDR.value = 0
    dut.WR_DATA.value = 0


async def _start(dut):
    cocotb.start_soon(Clock(dut.CLK, 10, unit="ns").start())
    _idle(dut)
    await RisingEdge(dut.CLK)


async def _write(dut, addr, data):
    dut.WR_EN.value = 1
    dut.WR_ADDR.value = addr
    dut.WR_DATA.value = data
    await RisingEdge(dut.CLK)
    dut.WR_EN.value = 0


async def _read(dut, addr):
    """Present a read address for one cycle and return the registered data."""
    dut.RD_EN.value = 1
    dut.RD_ADDR.value = addr
    await RisingEdge(dut.CLK)
    dut.RD_EN.value = 0
    await RisingEdge(dut.CLK)
    return int(dut.RD_DATA.value)


@cocotb.test()
async def test_write_then_read_returns_written_value(dut):
    """A word written in one cycle reads back in a later cycle."""
    await _start(dut)
    await _write(dut, 3, 0x5A)
    await _write(dut, 4, 0xC3)
    assert await _read(dut, 3) == 0x5A, "RAM did not return the word written to 3"
    assert await _read(dut, 4) == 0xC3, "RAM did not return the word written to 4"


@cocotb.test()
async def test_same_cycle_read_write_returns_new_value(dut):
    """A read concurrent with a write to the same address returns the new value."""
    await _start(dut)
    await _write(dut, 7, 0x11)

    dut.WR_EN.value = 1
    dut.WR_ADDR.value = 7
    dut.WR_DATA.value = 0x22
    dut.RD_EN.value = 1
    dut.RD_ADDR.value = 7
    await RisingEdge(dut.CLK)
    _idle(dut)
    await RisingEdge(dut.CLK)

    assert int(dut.RD_DATA.value) == 0x22, (
        f"same-cycle read returned 0x{int(dut.RD_DATA.value):02X}: the RAM handed "
        "back the word being replaced instead of the word being written"
    )
    assert await _read(dut, 7) == 0x22, "the write itself did not land"


@cocotb.test()
async def test_same_cycle_write_elsewhere_does_not_disturb_the_read(dut):
    """The bypass is address matched: a write elsewhere leaves the read alone."""
    await _start(dut)
    await _write(dut, 9, 0x3C)

    dut.WR_EN.value = 1
    dut.WR_ADDR.value = 10
    dut.WR_DATA.value = 0xFF
    dut.RD_EN.value = 1
    dut.RD_ADDR.value = 9
    await RisingEdge(dut.CLK)
    _idle(dut)
    await RisingEdge(dut.CLK)

    assert int(dut.RD_DATA.value) == 0x3C, (
        f"read of 9 returned 0x{int(dut.RD_DATA.value):02X} while 10 was written"
    )


@cocotb.test()
async def test_read_data_holds_while_rd_en_low(dut):
    """The registered read port holds its value until the next enabled read."""
    await _start(dut)
    await _write(dut, 12, 0x77)
    await _write(dut, 13, 0x88)

    assert await _read(dut, 12) == 0x77
    for _ in range(4):
        await RisingEdge(dut.CLK)
        assert int(dut.RD_DATA.value) == 0x77, (
            "RD_DATA changed with RD_EN low"
        )
    assert await _read(dut, 13) == 0x88
