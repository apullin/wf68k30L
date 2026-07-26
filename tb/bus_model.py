"""
MC68030 bus responder model for cocotb testbench.

Monitors the WF68K30L_TOP bus interface signals and responds to
read and write cycles using the memory model.

The MC68030 bus protocol:
  - Read cycle:  ASn goes low, RWn stays high.  Slave drives DATA_IN and
                 asserts DSACKn (00 = 32-bit port, 01 = 16-bit, 10 = 8-bit).
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

    # DSACKn code acknowledging a port of the given width in bytes.
    DSACK_FOR_WIDTH = {4: 0b00, 2: 0b01, 1: 0b10}

    # Driven on data bus lanes that a narrow port does not physically
    # connect, so that latching them shows up as corruption.
    POISON_BYTE = 0xA5

    def __init__(self, dut, memory, wait_states=0, port_width=4, sync_term=False):
        """
        Args:
            dut: cocotb handle to the DUT (WF68K30L_TOP).
            memory: Memory instance for read/write data.
            wait_states: Number of extra clock cycles before asserting DSACKn.
            port_width: Responding port width in bytes (4, 2 or 1).
            sync_term: Terminate with STERMn instead of DSACKn. STERM implies a
                32-bit port, and the two must never be asserted together.
        """
        assert port_width in self.DSACK_FOR_WIDTH, f"bad port_width {port_width}"
        assert not (sync_term and port_width != 4), "STERM requires a 32-bit port"
        self.dut = dut
        self.memory = memory
        self.wait_states = wait_states
        self.port_width = port_width
        self.sync_term = sync_term
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

    def _port_width_for(self, addr):
        """Width in bytes of the port responding at addr.

        Override to model a system that mixes port widths.
        """
        return self.port_width

    def _assert_term(self, dsack):
        """Terminate the current cycle, synchronously or asynchronously."""
        if self.sync_term:
            self.dut.STERMn.value = 0
        else:
            self.dut.DSACKn.value = dsack

    def _negate_term(self):
        if self.sync_term:
            self.dut.STERMn.value = 1
        self.dut.DSACKn.value = 0b11

    def _cycle_layout(self, addr, size_code, *, is_write=False):
        """Return (start_lane, byte_count) for this bus cycle.

        Lane 0 is DATA[31:24], lane 1 is DATA[23:16], lane 2 is DATA[15:8],
        lane 3 is DATA[7:0].

        Lanes are address-matched in both directions, as MC68030UM
        Tables 7-4 and 7-5 require: the first byte of the transfer appears
        on the lane selected by A1:A0, and a port only transfers as many
        bytes as fit between that lane and the end of the port.
        """
        want = {0: 4, 1: 1, 2: 2, 3: 3}[size_code]
        width = self._port_width_for(addr)
        # A narrow port is aliased across the long word, so only the low
        # log2(width) address bits select a lane within it.
        lane = addr & (width - 1)
        return lane, min(want, width - lane)

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
                self._negate_term()  # Deassert
                continue

            if as_n == 0:
                # Bus cycle active -- handle it completely
                try:
                    rw_n = int(self.dut.RWn.value)
                except ValueError:
                    self._negate_term()
                    continue

                try:
                    addr = int(self.dut.ADR_OUT.value)
                except ValueError:
                    addr = 0

                size_code = self._get_size_code()
                start_lane, byte_count = self._cycle_layout(
                    addr, size_code, is_write=(rw_n == 0)
                )
                width = self._port_width_for(addr)
                dsack = self.DSACK_FOR_WIDTH[width]

                # Insert wait states
                for _ in range(self.wait_states):
                    await RisingEdge(self.dut.CLK)

                if rw_n == 1:
                    # READ cycle: drive DATA_IN with data from memory
                    # Pack bytes into the exact bus lanes expected by the
                    # core's SIZE/ADR transfer semantics.
                    data = 0
                    for lane in range(width, 4):
                        data |= self.POISON_BYTE << ((3 - lane) * 8)
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
                    self._assert_term(dsack)

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

                    self._assert_term(dsack)

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

                # Deassert the termination signal after the cycle completes
                self._negate_term()

            else:
                # No bus cycle active -- deassert
                self._negate_term()
