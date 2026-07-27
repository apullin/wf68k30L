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

Unexpected exceptions fail the run. Every exception vector the test does not
populate itself is pointed at a stub that records the vector and halts, and
run_until_sentinel turns that into an assertion naming the vector. Tests that
install their own handler (before or after setup()) keep it. A test that
deliberately leaves an exception unhandled declares it with
h.expect_exception(vector); CPUTestHarness(dut, trap_detect=False) turns the
whole mechanism off.

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
    moveq, move, move_to_abs_long, nop, jmp_abs, stop,
    abs_long, disp16, imm_long,
    WORD, LONG, DN, AN_IND, AN_DISP, SPECIAL, ABS_L, IMMEDIATE,
)


# Names for the exception vectors the harness guards, used only to make the
# failure message readable (MC68030 UM Table 8-7).
VECTOR_NAMES = {
    2: "bus error",
    3: "address error",
    4: "illegal instruction",
    5: "zero divide",
    6: "CHK/CHK2",
    7: "TRAPcc/TRAPV",
    8: "privilege violation",
    9: "trace",
    10: "line 1010 emulator",
    11: "line 1111 emulator",
    13: "coprocessor protocol violation",
    14: "format error",
    15: "uninitialized interrupt",
    24: "spurious interrupt",
    25: "level 1 autovector",
    26: "level 2 autovector",
    27: "level 3 autovector",
    28: "level 4 autovector",
    29: "level 5 autovector",
    30: "level 6 autovector",
    31: "level 7 autovector",
    48: "FPCP branch/set on unordered",
    49: "FPCP inexact result",
    50: "FPCP divide by zero",
    51: "FPCP underflow",
    52: "FPCP operand error",
    53: "FPCP overflow",
    54: "FPCP signalling NAN",
    56: "MMU configuration error",
    57: "MMU illegal operation",
    58: "MMU access level violation",
}


def _vector_name(vec):
    if vec in VECTOR_NAMES:
        return VECTOR_NAMES[vec]
    if 32 <= vec <= 47:
        return f"TRAP #{vec - 32}"
    if 16 <= vec <= 23 or 59 <= vec <= 63 or vec == 12 or vec == 1:
        return "reserved"
    return "user interrupt"


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

    # --- Unexpected-exception detection ------------------------------------
    #
    # An exception with a zero vector entry sends the CPU to PC=0, where
    # zero-filled RAM decodes as a stream of ORI.B #0,D0 that slides straight
    # back into the program at PROGRAM_BASE. A deterministic test then simply
    # re-runs, recomputes the same results and writes the sentinel, so the
    # spurious exception is invisible. To make that loud, every vector the
    # test did not populate itself gets a stub that records the vector and the
    # exception frame, then halts.
    #
    # Only vectors 2..63 are covered: 0x008..0x0FF is exactly the part of the
    # vector table that PROGRAM_BASE (0x000100) does not overlap, and it holds
    # every exception the CPU can raise on its own. Vectors 64..255 share
    # memory with program code (they always have) and are only reachable via a
    # device-supplied interrupt vector, so they are left alone.
    TRAP_STUB_VEC_LO  = 2
    TRAP_STUB_VEC_HI  = 63
    TRAP_STUB_BASE    = 0x0D0000    # 62 stubs x 0x40 -> 0x0D0000..0x0D0F7F
    TRAP_STUB_STRIDE  = 0x40
    TRAP_REPORT_ADDR  = 0x0D8000    # +0 PC, +4 fmt/vec, +8 SR, +12 marker
    TRAP_MARKER_MAGIC = 0x7BAD0000  # marker = MAGIC | vector number

    def __init__(self, dut, wait_states=0, trap_detect=True):
        """Initialize the test harness.

        Args:
            dut: cocotb DUT handle (WF68K30L_TOP).
            wait_states: Number of bus wait states (0 for fastest).
            trap_detect: Install fail stubs on the unused exception vectors and
                fail the run if one of them is taken. Set False only for tests
                that deliberately let an unhandled exception run (a per-vector
                opt-out via expect_exception() is usually the better choice).
        """
        self.dut = dut
        self.mem = Memory(size=1 << 20)  # 1 MB address space
        self.bus = None
        self.wait_states = wait_states
        self.trap_detect = trap_detect
        self._expected_vectors = set()
        self._trap_stubs_installed = False

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

        # Guard the vectors this test has not populated. Done before the
        # program is loaded so program code always wins on any overlap, and
        # skipped for vectors that already hold a handler address, so tests
        # that install their own handlers before setup() keep them.
        if self.trap_detect:
            self._install_trap_stubs()

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
        self.dut.CBACKn.value = 1      # No burst acknowledge
        self.dut.CIINn.value = 1       # No external cache inhibit
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

    # ------------------------------------------------------------------
    # Unexpected-exception detection
    # ------------------------------------------------------------------

    def _trap_stub_words(self, vec):
        """Return the fail-stub body for one exception vector.

        Every MC68030 exception frame starts with SR (word), PC (long) and the
        format/vector word (UM 8.2), so the same three loads describe any frame
        format. The marker longword is written last and doubles as the report's
        valid flag. STOP makes the failure deterministic: the stub never
        returns, so the CPU cannot wander back into the program and recompute a
        passing result.
        """
        report = self.TRAP_REPORT_ADDR
        return [
            # MOVE.L (2,A7),(report+0).L    -- stacked PC
            *move(LONG, AN_DISP, 7, SPECIAL, ABS_L),
            *disp16(2), *abs_long(report + 0),
            # MOVE.W (6,A7),(report+6).L    -- format/vector word
            *move(WORD, AN_DISP, 7, SPECIAL, ABS_L),
            *disp16(6), *abs_long(report + 6),
            # MOVE.W (A7),(report+10).L     -- stacked SR
            *move(WORD, AN_IND, 7, SPECIAL, ABS_L),
            *abs_long(report + 10),
            # MOVE.L #marker,(report+12).L  -- valid flag, written last
            *move(LONG, SPECIAL, IMMEDIATE, SPECIAL, ABS_L),
            *imm_long(self.TRAP_MARKER_MAGIC | vec), *abs_long(report + 12),
            # Halt: no RTE, no fall-through.
            *stop(0x2700),
        ]

    def _install_trap_stubs(self):
        """Point every unused exception vector at a recording fail stub."""
        for vec in range(self.TRAP_STUB_VEC_LO, self.TRAP_STUB_VEC_HI + 1):
            if vec in self._expected_vectors:
                continue
            if self.mem.read(vec * 4, 4) != 0:
                continue  # test already installed a handler for this vector
            stub = (
                self.TRAP_STUB_BASE
                + (vec - self.TRAP_STUB_VEC_LO) * self.TRAP_STUB_STRIDE
            )
            self.mem.load_words(stub, self._trap_stub_words(vec))
            self.mem.load_long(vec * 4, stub)
        self._trap_stubs_installed = True

    def expect_exception(self, *vectors):
        """Opt out of unexpected-exception detection for these vectors.

        For tests that legitimately take an exception without installing a
        handler. The vector is left exactly as the test found it, so the
        pre-detection behavior (vector 0 -> PC 0) is preserved, and a stub
        report for that vector is not treated as a failure.

        Safe to call either side of setup().
        """
        for vec in vectors:
            self._expected_vectors.add(vec)
            if self._trap_stubs_installed:
                stub = (
                    self.TRAP_STUB_BASE
                    + (vec - self.TRAP_STUB_VEC_LO) * self.TRAP_STUB_STRIDE
                )
                if self.mem.read(vec * 4, 4) == stub:
                    self.mem.load_long(vec * 4, 0)

    def trap_report(self):
        """Return the recorded unexpected-exception frame, or None.

        Keys: vector, pc (stacked PC), format_vector, sr.
        """
        marker = self.mem.read(self.TRAP_REPORT_ADDR + 12, 4)
        if (marker & 0xFFFF0000) != self.TRAP_MARKER_MAGIC:
            return None
        return {
            "vector": marker & 0xFF,
            "pc": self.mem.read(self.TRAP_REPORT_ADDR + 0, 4),
            "format_vector": self.mem.read(self.TRAP_REPORT_ADDR + 4, 4) & 0xFFFF,
            "sr": self.mem.read(self.TRAP_REPORT_ADDR + 8, 4) & 0xFFFF,
        }

    def check_no_unexpected_exception(self):
        """Raise AssertionError if a guarded exception vector was taken."""
        if not self.trap_detect:
            return
        report = self.trap_report()
        if report is None or report["vector"] in self._expected_vectors:
            return
        vec = report["vector"]
        raise AssertionError(
            f"Unexpected exception: vector {vec} ({_vector_name(vec)}) was "
            f"taken and no handler was installed for it. "
            f"Stacked PC=0x{report['pc']:08X}, "
            f"frame format/vector word=0x{report['format_vector']:04X} "
            f"(format ${report['format_vector'] >> 12:X}, vector offset "
            f"0x{report['format_vector'] & 0xFFF:03X}), "
            f"SR=0x{report['sr']:04X}. "
            f"If this exception is intended, install a handler for vector "
            f"{vec} or call h.expect_exception({vec}) before setup()."
        )

    async def load_program(self, words):
        """Load program into memory (legacy API, for use after setup(None)).

        IMPORTANT: If the CPU has already come out of reset before this is
        called, the reset vectors will have been fetched as zeros. For
        correct operation, pass program_words to setup() instead.
        """
        self._load_memory(words)

    async def run_until_sentinel(
        self,
        max_cycles=None,
        check_bus_invariants=False,
        max_bus_cycle_cycles=128,
    ):
        """Run the CPU until the sentinel value appears in memory.

        The test program should write SENTINEL_VAL to SENTINEL_ADDR
        when it completes. This method polls memory each clock cycle.

        Args:
            max_cycles: Maximum clock cycles to wait. Defaults to CYCLE_BUDGET.
            check_bus_invariants: If True, check lightweight bus-protocol
                safety invariants each cycle.
            max_bus_cycle_cycles: Maximum allowed cycles with ASn asserted
                for a single bus cycle before flagging a stalled handshake.

        Returns:
            True if the sentinel was detected, False if the budget expired.

        Raises:
            AssertionError if an exception vector the test did not populate was
            taken (see check_no_unexpected_exception).
        """
        budget = max_cycles or self.CYCLE_BUDGET
        monitor_state = {
            "in_cycle": False,
            "cycle_len": 0,
            "rw": 1,
            "size": 0,
        }
        report_addr = self.TRAP_REPORT_ADDR + 12

        for _ in range(budget):
            await RisingEdge(self.dut.CLK)
            if check_bus_invariants:
                self._check_bus_invariants(
                    monitor_state,
                    max_bus_cycle_cycles=max_bus_cycle_cycles,
                )
            # A fail stub reached its marker write: report the vector instead of
            # letting the run burn the whole budget or slide into a pass.
            if self.trap_detect and self.mem.read(report_addr, 4) != 0:
                self.check_no_unexpected_exception()
            # Check if sentinel has been written to memory
            val = self.mem.read(self.SENTINEL_ADDR, 4)
            if val == self.SENTINEL_VAL:
                return True
        self.check_no_unexpected_exception()
        return False

    def _check_bus_invariants(self, state, max_bus_cycle_cycles=128):
        """Check lightweight safety invariants over ASn/DSn/RWn/SIZE.

        These checks are intentionally local and bounded:
          - RWn and SIZE remain stable for a single ASn-asserted cycle.
          - A single ASn-asserted cycle must complete within a bounded window.
          - Control outputs decode to legal binary values (no X/Z once sampled).
        """
        try:
            as_n = int(self.dut.ASn.value)
            ds_n = int(self.dut.DSn.value)
            rw_n = int(self.dut.RWn.value)
            size = int(self.dut.SIZE.value)
        except ValueError:
            # During reset and initialization, control lines may be X/Z.
            return

        if as_n not in (0, 1) or ds_n not in (0, 1) or rw_n not in (0, 1):
            raise AssertionError(
                f"Invalid bus control decode: ASn={as_n}, DSn={ds_n}, RWn={rw_n}"
            )
        if size not in (0, 1, 2, 3):
            raise AssertionError(f"Invalid SIZE decode: SIZE={size}")

        if as_n == 1:
            state["in_cycle"] = False
            state["cycle_len"] = 0
            return

        if not state["in_cycle"]:
            state["in_cycle"] = True
            state["cycle_len"] = 1
            state["rw"] = rw_n
            state["size"] = size
            return

        state["cycle_len"] += 1
        if rw_n != state["rw"]:
            raise AssertionError(
                f"RWn changed mid-cycle: start={state['rw']} now={rw_n}"
            )
        if size != state["size"]:
            raise AssertionError(
                f"SIZE changed mid-cycle: start={state['size']} now={size}"
            )
        if state["cycle_len"] > max_bus_cycle_cycles:
            raise AssertionError(
                f"Bus cycle exceeded {max_bus_cycle_cycles} clocks"
            )

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

    async def execute_and_check(
        self,
        program_words,
        expected_results,
        check_bus_invariants=False,
        max_bus_cycle_cycles=128,
    ):
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
        found = await self.run_until_sentinel(
            check_bus_invariants=check_bus_invariants,
            max_bus_cycle_cycles=max_bus_cycle_cycles,
        )
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
