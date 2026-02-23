"""
MC68030 CPU test execution framework for cocotb testbench.

Provides CPUTestHarness, a reusable test fixture that handles:
  - Clock generation and CPU reset
  - Memory initialization with reset vectors and program code
  - Bus model setup and management
  - Sentinel-based program termination detection
  - Result verification from memory

Usage in a cocotb test:

    @cocotb.test()
    async def test_example(dut):
        h = CPUTestHarness(dut)

        program = [
            *moveq(42, 0),                                    # MOVEQ #42,D0
            *move_to_abs_long(LONG, DN, 0, h.RESULT_BASE),    # MOVE.L D0,(RESULT_BASE)
            *h.sentinel_program(),                             # Write sentinel
        ]
        await h.setup(program)
        found = await h.run_until_sentinel()
        assert found
        assert h.read_result_long(0) == 42

Signal names match WF68K30L_TOP as defined in wf68k30L_top.sv:
  - CLK, RESET_INn, HALT_INn
  - DATA_IN, DATA_OUT, ADR_OUT
  - DSACKn, BERRn, AVECn, IPLn
  - STERMn, BRn, BGACKn
  - ASn, DSn, RWn, SIZE
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles

from memory import Memory
from bus_model import BusModel
from m68k_encode import (
    moveq, move, move_to_abs_long, nop, jmp_abs,
    LONG, DN, SPECIAL, ABS_L, IMMEDIATE,
)


class CPUTestHarness:
    """Reusable test harness for WF68K30L CPU integration tests."""

    # Memory layout constants
    PROGRAM_BASE  = 0x000100   # Where program code is loaded
    DATA_BASE     = 0x010000   # Available for test data
    RESULT_BASE   = 0x020000   # Where tests write results for verification
    SENTINEL_ADDR = 0x030000   # Address checked for sentinel value
    SENTINEL_VAL  = 0xDEADCAFE # Value written to signal program completion
    SSP_INIT      = 0x001000   # Initial Supervisor Stack Pointer
    CYCLE_BUDGET  = 3000       # Default maximum clock cycles to wait

    # NOP sled size after program (prevents runaway execution)
    NOP_SLED_SIZE = 64

    def __init__(self, dut, wait_states=0):
        """Initialize the test harness.

        Args:
            dut: cocotb DUT handle (WF68K30L_TOP).
            wait_states: Number of bus wait states (0 for fastest).
        """
        self.dut = dut
        self.mem = Memory(size=1 << 20)  # 1 MB address space
        self.bus = None
        self.wait_states = wait_states

    async def setup(self, program_words=None):
        """Initialize clock, load memory, reset CPU, and start bus model.

        The sequence is critical for correct operation:
          1. Start clock and set inputs to idle
          2. Load memory with reset vectors and program (BEFORE reset release)
          3. Assert reset for 20 cycles
          4. Start bus model (so it can respond to vector fetches)
          5. Release reset (CPU fetches reset vectors from pre-loaded memory)

        Args:
            program_words: Optional list of 16-bit instruction words to load.
                          If provided, load_program() is called automatically.
        """
        # Start 100 MHz clock (10 ns period)
        clock = Clock(self.dut.CLK, 10, unit="ns")
        cocotb.start_soon(clock.start())

        # Set all inputs to idle/deasserted state
        self._init_idle_inputs()

        # Load memory BEFORE reset (so vectors are ready when CPU fetches them)
        if program_words is not None:
            self._load_memory(program_words)

        # Let signals settle
        await RisingEdge(self.dut.CLK)
        await RisingEdge(self.dut.CLK)

        # Assert reset (RESET_INn and HALT_INn low)
        self.dut.RESET_INn.value = 0
        self.dut.HALT_INn.value = 0
        await ClockCycles(self.dut.CLK, 20)

        # Start the bus responder BEFORE releasing reset,
        # so it is ready to respond to the first vector fetch.
        self.bus = BusModel(self.dut, self.mem, self.wait_states)
        await self.bus.start()

        # Release reset -- CPU will immediately begin fetching reset vectors
        self.dut.RESET_INn.value = 1
        self.dut.HALT_INn.value = 1

        # Let reset propagate
        await ClockCycles(self.dut.CLK, 4)

    def _init_idle_inputs(self):
        """Set all active-low DUT inputs to their idle (deasserted) state."""
        self.dut.RESET_INn.value = 1
        self.dut.HALT_INn.value = 1
        self.dut.BERRn.value = 1
        self.dut.DSACKn.value = 0b11   # No acknowledge
        self.dut.IPLn.value = 0b111    # No interrupt (all high = no IRQ)
        self.dut.AVECn.value = 1       # No auto-vector
        self.dut.DATA_IN.value = 0
        self.dut.STERMn.value = 1      # No synchronous termination
        self.dut.BRn.value = 1         # No bus request
        self.dut.BGACKn.value = 1      # No bus grant acknowledge

    def _load_memory(self, words):
        """Load reset vectors and program words into the memory model.

        This populates the memory before the CPU comes out of reset, ensuring
        the reset vector fetch returns correct SSP and PC values.

        Args:
            words: List of 16-bit instruction words to load at PROGRAM_BASE.
        """
        # MC68030 reset vectors (big-endian longwords)
        self.mem.load_long(0x000000, self.SSP_INIT)     # Initial SSP
        self.mem.load_long(0x000004, self.PROGRAM_BASE)  # Initial PC

        # Load program instruction words
        self.mem.load_words(self.PROGRAM_BASE, words)

        # Append NOP sled after program as safety net
        nop_addr = self.PROGRAM_BASE + len(words) * 2
        for i in range(self.NOP_SLED_SIZE):
            self.mem.load_words(nop_addr + i * 2, [0x4E71])

    async def load_program(self, words):
        """Load program into memory (legacy API, for use after setup(None)).

        IMPORTANT: If the CPU has already come out of reset before this is
        called, the reset vectors will have been fetched as zeros. For
        correct operation, pass program_words to setup() instead.
        """
        self._load_memory(words)

    async def run_until_sentinel(self, max_cycles=None):
        """Run the CPU until the sentinel value appears in memory.

        The test program should write SENTINEL_VAL to SENTINEL_ADDR
        when it completes. This method polls memory each clock cycle.

        Args:
            max_cycles: Maximum clock cycles to wait. Defaults to CYCLE_BUDGET.

        Returns:
            True if the sentinel was detected, False if the budget expired.
        """
        budget = max_cycles or self.CYCLE_BUDGET
        for _ in range(budget):
            await RisingEdge(self.dut.CLK)
            # Check if sentinel has been written to memory
            val = self.mem.read(self.SENTINEL_ADDR, 4)
            if val == self.SENTINEL_VAL:
                return True
        return False

    def read_result_long(self, offset=0):
        """Read a 32-bit value from RESULT_BASE + offset.

        Args:
            offset: Byte offset from RESULT_BASE.

        Returns:
            32-bit unsigned integer.
        """
        return self.mem.read(self.RESULT_BASE + offset, 4)

    def read_result_word(self, offset=0):
        """Read a 16-bit value from RESULT_BASE + offset.

        Args:
            offset: Byte offset from RESULT_BASE.

        Returns:
            16-bit unsigned integer.
        """
        return self.mem.read(self.RESULT_BASE + offset, 2)

    def read_result_byte(self, offset=0):
        """Read an 8-bit value from RESULT_BASE + offset.

        Args:
            offset: Byte offset from RESULT_BASE.

        Returns:
            8-bit unsigned integer.
        """
        return self.mem.read(self.RESULT_BASE + offset, 1)

    def sentinel_program(self):
        """Return instruction words that write the sentinel to memory.

        Generates the following instruction sequence:
            MOVE.L #SENTINEL_VAL, D7     ; Load 0xDEADCAFE into D7
            MOVE.L D7, (SENTINEL_ADDR).L ; Store to sentinel address

        Uses D7 as a scratch register. Callers should avoid using D7 for
        test data if they use sentinel_program().
        """
        words = []

        # MOVE.L #SENTINEL_VAL, D7
        # Encoding: size=10(long), dst_reg=111(D7), dst_mode=000(Dn),
        #           src_mode=111(special), src_reg=100(immediate)
        words.extend(move(LONG, SPECIAL, IMMEDIATE, DN, 7))
        # Append 32-bit immediate value
        words.append((self.SENTINEL_VAL >> 16) & 0xFFFF)
        words.append(self.SENTINEL_VAL & 0xFFFF)

        # MOVE.L D7, (SENTINEL_ADDR).L
        words.extend(move_to_abs_long(LONG, DN, 7, self.SENTINEL_ADDR))

        return words

    def stop_program(self):
        """Return a STOP instruction (alternative to sentinel for supervisor tests).

        STOP #$2700 halts the CPU with supervisor mode, IPL mask = 7.
        """
        from m68k_encode import stop
        return stop(0x2700)

    async def execute_and_check(self, program_words, expected_results):
        """Convenience: setup, load program, run, and verify results.

        This is a complete one-call test flow. The program_words should
        include sentinel_program() at the end.

        Args:
            program_words: List of 16-bit instruction words (should include
                          sentinel_program() at the end).
            expected_results: Dict mapping byte offsets (from RESULT_BASE)
                            to expected 32-bit values.

        Raises:
            AssertionError if sentinel not reached or any result mismatches.
        """
        await self.setup(program_words)
        found = await self.run_until_sentinel()
        assert found, "Sentinel not reached within cycle budget"
        for offset, expected in expected_results.items():
            actual = self.read_result_long(offset)
            assert actual == expected, (
                f"Result at offset 0x{offset:04X}: "
                f"expected 0x{expected:08X}, got 0x{actual:08X}"
            )

    def cleanup(self):
        """Stop the bus model. Call at test end if needed."""
        if self.bus is not None:
            self.bus.stop()
