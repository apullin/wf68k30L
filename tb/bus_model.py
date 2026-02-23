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

    async def start(self):
        """Start the bus responder coroutine."""
        self._running = True
        cocotb.start_soon(self._responder())

    def stop(self):
        """Stop the bus responder."""
        self._running = False

    def _get_bus_size(self):
        """Decode the SIZE output to number of bytes.

        The bus interface SIZE output follows the MC68030 encoding:
          SIZE[1:0] = 00 -> 4 bytes (long word)
          SIZE[1:0] = 01 -> 1 byte
          SIZE[1:0] = 10 -> 2 bytes (word)
          SIZE[1:0] = 11 -> 3 bytes (not typical)
        """
        try:
            size_val = int(self.dut.SIZE.value)
        except ValueError:
            return 4  # Default to long if X/Z
        if size_val == 0:
            return 4
        elif size_val == 1:
            return 1
        elif size_val == 2:
            return 2
        else:
            return 4  # Fallback

    async def _responder(self):
        """Main bus responder loop -- runs as a cocotb coroutine."""
        while self._running:
            await RisingEdge(self.dut.CLK)

            try:
                as_n = int(self.dut.ASn.value)
            except ValueError:
                # ASn is X or Z during reset
                self.dut.DSACKn.value = 0b11  # Deassert
                continue

            if as_n == 0:
                # Bus cycle active
                try:
                    rw_n = int(self.dut.RWn.value)
                except ValueError:
                    self.dut.DSACKn.value = 0b11
                    continue

                try:
                    addr = int(self.dut.ADR_OUT.value)
                except ValueError:
                    addr = 0

                num_bytes = self._get_bus_size()

                # Insert wait states
                for _ in range(self.wait_states):
                    await RisingEdge(self.dut.CLK)

                if rw_n == 1:
                    # READ cycle: drive DATA_IN with data from memory
                    data = self.memory.read(addr, num_bytes)
                    # For the 68030 bus, data is always aligned to the MSB
                    # of the 32-bit data bus. The bus interface handles
                    # alignment internally, so we provide the full 32-bit
                    # value.
                    if num_bytes == 1:
                        # Byte on bits 31:24
                        data = (data & 0xFF) << 24
                    elif num_bytes == 2:
                        # Word on bits 31:16
                        data = (data & 0xFFFF) << 16
                    # Long: already 32-bit aligned

                    self.dut.DATA_IN.value = data
                    self.dut.DSACKn.value = 0b00  # 32-bit port ack

                else:
                    # WRITE cycle: capture DATA_OUT
                    try:
                        data = int(self.dut.DATA_OUT.value)
                    except ValueError:
                        data = 0

                    if num_bytes == 1:
                        # Byte from bits 31:24
                        byte_val = (data >> 24) & 0xFF
                        self.memory.write(addr, 1, byte_val)
                    elif num_bytes == 2:
                        # Word from bits 31:16
                        word_val = (data >> 16) & 0xFFFF
                        self.memory.write(addr, 2, word_val)
                    else:
                        self.memory.write(addr, 4, data)

                    self.dut.DSACKn.value = 0b00  # 32-bit port ack

            else:
                # No bus cycle active -- deassert DSACKn
                self.dut.DSACKn.value = 0b11
