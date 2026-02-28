"""
MC68030 bus responder model for cocotb testbench.

Monitors the WF68K30L_TOP bus interface signals and responds to
read and write cycles using the memory model.

The MC68030 bus protocol:
  - Read cycle:  ASn goes low, RWn stays high.  Slave drives DATA_IN and
                 asserts DSACKn (00 = 32-bit port acknowledge).
  - Write cycle: ASn goes low, RWn goes low.  Slave captures DATA_OUT when
                 DSn goes low, then asserts DSACKn.

SIZE[1:0] encoding (active-low in real 68030, but this core uses active-high encoding):
  00 = long (4 bytes)
  01 = byte
  10 = word (2 bytes)
  11 = 3-byte (line -- not used here)
"""

import os

import cocotb
from cocotb.triggers import RisingEdge, FallingEdge, Timer


class BusModel:
    """Async bus responder that connects to WF68K30L_TOP signals."""

    def __init__(self, dut, memory, wait_states=0):
        """
        Args:
            dut: cocotb handle to the DUT (WF68K30L_TOP).
            memory: Memory instance for read/write data.
            wait_states: Number of extra clock cycles before asserting DSACKn.
        """
        self.dut = dut
        self.memory = memory
        self.wait_states = wait_states
        self._running = False
        self._trace = os.environ.get("BUS_TRACE", "0") not in ("", "0", "false", "False")
        self._trace_min = int(os.environ.get("BUS_TRACE_MIN", "0"), 0)
        self._trace_max = int(os.environ.get("BUS_TRACE_MAX", "0xFFFFFFFF"), 0)

    def _trace_enabled(self, addr):
        return self._trace and (self._trace_min <= addr <= self._trace_max)

    async def start(self):
        """Start the bus responder coroutine."""
        self._running = True
        cocotb.start_soon(self._responder())

    def stop(self):
        """Stop the bus responder."""
        self._running = False

    def _get_size_code(self):
        """Decode SIZE[1:0] from the DUT.

        Encoding:
          00 = long
          01 = byte
          10 = word
          11 = three-byte transfer
        """
        try:
            return int(self.dut.SIZE.value)
        except ValueError:
            return 0

    def _cycle_layout(self, addr, size_code, *, is_write=False):
        """Return (start_lane, byte_count) for this bus cycle.

        Lane 0 is DATA[31:24], lane 1 is DATA[23:16], lane 2 is DATA[15:8],
        lane 3 is DATA[7:0].

        This matches the WF68K30L bus-interface alignment behavior for
        split transfers (e.g., unaligned long reads/writes).
        """
        a = addr & 0x3
        if size_code == 0:  # long
            # WF68K30L read cycles consume long data from the top lanes and
            # shift by cycle boundaries, while write cycles source valid bytes
            # starting at A1:A0 for misaligned stores.
            if is_write:
                return a, 4 - a
            return 0, 4 - a
        if size_code == 1:  # byte
            return a, 1
        if size_code == 2:  # word
            return a, 1 if a == 3 else 2
        # three-byte transfer
        count = 3 - a
        if count <= 0:
            count = 1
        return a, count

    async def _responder(self):
        """Main bus responder loop -- runs as a cocotb coroutine.

        Implements proper bus cycle tracking: once a bus cycle is detected
        (ASn goes low), the responder handles it completely (including wait
        states and DSACKn assertion), then waits for ASn to go high before
        accepting a new bus cycle. This prevents double-responding to the
        same bus cycle when wait states cause the response to extend beyond
        the S4/S5 bus phases where ASn is still asserted.
        """
        while self._running:
            # Wait for bus cycle start: ASn goes low
            await RisingEdge(self.dut.CLK)

            try:
                as_n = int(self.dut.ASn.value)
            except ValueError:
                # ASn is X or Z during reset
                self.dut.DSACKn.value = 0b11  # Deassert
                continue

            if as_n == 0:
                # Bus cycle active -- handle it completely
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

                # Insert wait states
                for _ in range(self.wait_states):
                    await RisingEdge(self.dut.CLK)

                if rw_n == 1:
                    # READ cycle: drive DATA_IN with data from memory
                    # Pack bytes into the exact bus lanes expected by the
                    # core's SIZE/ADR transfer semantics.
                    data = 0
                    for i in range(byte_count):
                        b = self.memory.read(addr + i, 1) & 0xFF
                        lane = start_lane + i
                        shift = (3 - lane) * 8  # MSB-first lane mapping
                        data |= b << shift

                    self.dut.DATA_IN.value = data
                    if self._trace_enabled(addr):
                        self.dut._log.warning(
                            "bus rd addr=0x%08X size=%d a=%d lanes=%d+%d data=0x%08X",
                            addr,
                            size_code,
                            addr & 0x3,
                            start_lane,
                            byte_count,
                            data,
                        )
                    self.dut.DSACKn.value = 0b00  # 32-bit port ack

                else:
                    # WRITE cycle: capture DATA_OUT
                    try:
                        data = int(self.dut.DATA_OUT.value)
                    except ValueError:
                        data = 0

                    # Unpack only the lanes that are valid for this cycle.
                    for i in range(byte_count):
                        lane = start_lane + i
                        shift = (3 - lane) * 8
                        byte_val = (data >> shift) & 0xFF
                        self.memory.write(addr + i, 1, byte_val)
                    if self._trace_enabled(addr):
                        self.dut._log.warning(
                            "bus wr addr=0x%08X size=%d a=%d lanes=%d+%d data=0x%08X",
                            addr,
                            size_code,
                            addr & 0x3,
                            start_lane,
                            byte_count,
                            data,
                        )

                    self.dut.DSACKn.value = 0b00  # 32-bit port ack

                # Wait for bus cycle to complete: ASn goes high
                # This prevents responding to the same cycle twice when
                # the bus master keeps ASn asserted through S4/S5.
                while self._running:
                    await RisingEdge(self.dut.CLK)
                    try:
                        as_n = int(self.dut.ASn.value)
                    except ValueError:
                        break
                    if as_n == 1:
                        break

                # Deassert DSACKn after the bus cycle completes
                self.dut.DSACKn.value = 0b11

            else:
                # No bus cycle active -- deassert DSACKn
                self.dut.DSACKn.value = 0b11
