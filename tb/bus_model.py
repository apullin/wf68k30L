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
from cocotb.triggers import RisingEdge, FallingEdge, Timer, ClockCycles


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
        # Arbitration hook -- see "Bus arbitration support" below.
        self.bus_owner = None
        self.foreign_cycle_starts = []
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

    # -- Bus arbitration support (MC68030UM 7.7) -------------------------
    # The responder body needs exactly one line of arbitration awareness,
    # and it calls this predicate.  Everything else about arbitration lives
    # in AlternateBusMaster at the bottom of this module.
    def _bus_yielded(self):
        """True while an alternate bus master owns the bus.

        UM 7.7.3: bus mastership belongs to the device holding BGACK, so the
        slave model must be off the bus entirely -- it neither samples nor
        terminates anything the core may still be driving.
        """
        return self.bus_owner is not None and self.bus_owner.owns_bus

    def _note_foreign_cycle(self, addr, rw_n):
        """Record a core-driven cycle seen while the bus was handed away."""
        if len(self.foreign_cycle_starts) < 32:
            self.foreign_cycle_starts.append((addr, rw_n))
    # -- end bus arbitration support -------------------------------------

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

            if self._bus_yielded():
                # An alternate bus master holds BGACK: stay off the bus.
                self._negate_term()
                if as_n == 0:
                    try:
                        self._note_foreign_cycle(int(self.dut.ADR_OUT.value),
                                                 int(self.dut.RWn.value))
                    except ValueError:
                        self._note_foreign_cycle(None, None)
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


# =========================================================================
# Bus arbitration: alternate bus master model (MC68030UM 7.7)
#
# Self-contained addition -- nothing above this line depends on it except
# BusModel._bus_yielded(), which is the single hook that keeps the slave
# responder off the bus while an alternate master owns it.
# =========================================================================


class AlternateBusMaster:
    """External device that arbitrates the bus away from the processor.

    Implements both arbitration protocols the MC68030 supports:

      Three-wire (UM 7.7, steps 1-3): assert BR, wait for BG, wait for the
      bus to go free (UM 7.7.3: AS and DSACKx/STERM negated, BGACK inactive),
      assert BGACK, run for a while, negate BGACK.

      Single-wire (UM 7.7.4): assert BGACK alone with no BR and no BG.  "An
      alternate master forces the MC68030 to release the bus by asserting
      BGACK and waits for AS to negate before taking the bus... Note that for
      the method to operate properly, AS must be observed to be negated
      (high) on two consecutive clock edges before the alternate bus master
      takes the bus."

    While this model owns the bus it also acts as the contention checker.
    UM 7.7.4 makes the processor's three-state control (T) part of the
    arbitration state machine, so once BGACK is acknowledged the core must
    have released the bus: BUS_EN low (which is what the CPU wrapper uses to
    three-state A/FC/SIZE/AS/DS/RW/RMC/DBEN/CIOUT/CBREQ per UM Table 5-2) and
    no new AS assertion.  Both are recorded rather than raised so a test can
    report a precise diagnosis.
    """

    def __init__(self, dut, responder=None, release_grace=4):
        """
        Args:
            dut: cocotb handle to WF68K30L_TOP.
            responder: BusModel to hand the bus back and forth with.  Wiring
                it up makes the slave model go quiet while this master owns
                the bus, and makes it record any cycle the core still starts.
            release_grace: Clocks allowed between BGACK assertion and BUS_EN
                going low.  UM 7.7.4 synchronizes BGACK in at most two clocks
                and changes state on the next rising edge.
        """
        self.dut = dut
        self.responder = responder
        self.release_grace = release_grace

        self.owns_bus = False

        # Grant observations.
        self.grant_seen = False
        self.grant_latency = None       # clocks from BR assertion to BG low
        self.as_low_at_request = None   # was a cycle in progress at BR time?
        self.as_low_at_grant = None     # UM 7.7.2: BG may overlap that cycle
        self.cycle_starts_before_grant = 0

        # Ownership observations.
        self.bus_en_release_clocks = None  # clocks until BUS_EN went low
        self.bus_en_high_clocks = 0        # clocks BUS_EN stayed high past grace
        self.as_low_clocks = 0             # clocks the core drove AS at us
        self.owned_clocks = 0
        self.violations = []

        self._monitor_task = None
        self._monitoring = False

    # ---- signal helpers ------------------------------------------------

    def _rd(self, name, default=None):
        """Read an integer DUT signal, tolerating X/Z during reset."""
        try:
            return int(getattr(self.dut, name).value)
        except (ValueError, AttributeError):
            return default

    def _note(self, text):
        if len(self.violations) < 32:
            self.violations.append(text)

    # ---- three-wire arbitration ---------------------------------------

    async def assert_br(self):
        """UM 7.7.1: request the bus.  BR may be issued at any time."""
        self.as_low_at_request = self._rd("ASn")
        self.dut.BRn.value = 0

    def negate_br(self):
        self.dut.BRn.value = 1

    async def wait_for_grant(self, timeout=256):
        """Wait for BG, recording how long it took and what the bus was doing.

        UM 7.7.2: "The processor asserts BG as soon as possible after receipt
        of BR.  This is immediately following internal synchronization except
        during a read-modify-write cycle or following an internal decision to
        execute a bus cycle."  A grant is therefore allowed to overlap a cycle
        that is already running, so as_low_at_grant records whether it did.
        """
        prev_as = self._rd("ASn", 1)
        for n in range(1, timeout + 1):
            await RisingEdge(self.dut.CLK)
            as_n = self._rd("ASn", 1)
            if prev_as == 1 and as_n == 0:
                self.cycle_starts_before_grant += 1
            prev_as = as_n
            if self._rd("BGn", 1) == 0:
                self.grant_seen = True
                self.grant_latency = n
                self.as_low_at_grant = as_n
                return True
        return False

    async def wait_bus_free(self, timeout=256, settled=2):
        """UM 7.7.3 / 7.7.4: wait until the bus is safe to take.

        Requires AS negated on `settled` consecutive rising edges (the UM asks
        for two), the termination signals negated, and BGACK inactive.
        """
        high = 0
        for _ in range(timeout):
            await RisingEdge(self.dut.CLK)
            free = (
                self._rd("ASn", 0) == 1
                and self._rd("DSACKn", 0) == 0b11
                and self._rd("STERMn", 0) == 1
                and self._rd("BGACKn", 0) == 1
            )
            high = high + 1 if free else 0
            if high >= settled:
                return True
        return False

    async def assert_bgack(self, negate_br=True):
        """Assume bus mastership (UM 7.7.3) and start the contention check.

        UM 7.7.3: "The BR from the granted device should be negated after
        BGACK is asserted."
        """
        self.dut.BGACKn.value = 0
        if negate_br:
            self.negate_br()
        self.owns_bus = True
        if self.responder is not None:
            self.responder.bus_owner = self
        self._monitoring = True
        self._monitor_task = cocotb.start_soon(self._ownership_monitor())

    async def hold(self, clocks):
        """Stay bus master for a while, standing in for the master's cycles."""
        await ClockCycles(self.dut.CLK, clocks)

    async def release(self, settle=2):
        """UM 7.7.3: bus mastership terminates at the negation of BGACK."""
        self._monitoring = False
        self.owns_bus = False
        self.dut.BGACKn.value = 1
        if self.responder is not None:
            self.responder.bus_owner = None
        await ClockCycles(self.dut.CLK, settle)

    async def acquire(self, hold_clocks=16, timeout=256):
        """Full three-wire sequence.  Returns False if BG never arrived."""
        await self.assert_br()
        if not await self.wait_for_grant(timeout=timeout):
            self.negate_br()
            return False
        if not await self.wait_bus_free(timeout=timeout):
            self.negate_br()
            return False
        await self.assert_bgack()
        await self.hold(hold_clocks)
        return True

    # ---- single-wire arbitration (UM 7.7.4) ---------------------------

    async def acquire_single_wire(self, hold_clocks=16, timeout=256,
                                  settled=2):
        """Take the bus with BGACK alone: no BR, no BG, state 0 -> state 4.

        UM 7.7.4: "As shown by the path from state 0 to state 4, BGACK alone
        can be used to place the processor's external bus buffers in the
        high-impedance state, providing single-wire arbitration capability."
        """
        if not await self.wait_bus_free(timeout=timeout, settled=settled):
            return False
        await self.assert_bgack(negate_br=False)
        await self.hold(hold_clocks)
        return True

    async def force_release_now(self, hold_clocks=16):
        """Assert BGACK immediately, without waiting for the bus to go free.

        For probing the forced-release path in situations where the core never
        lets the bus go idle on its own (a locked read-modify-write sequence,
        for instance).  The AS-negated precondition of UM 7.7.4 is deliberately
        not applied, so the contention checks are advisory here.
        """
        await self.assert_bgack(negate_br=False)
        await self.hold(hold_clocks)

    # ---- ownership monitor --------------------------------------------

    async def _ownership_monitor(self):
        """Check the core really is off the bus for as long as we own it."""
        clk = 0
        while self._monitoring:
            await RisingEdge(self.dut.CLK)
            clk += 1
            self.owned_clocks = clk

            bus_en = self._rd("BUS_EN")
            if bus_en == 0 and self.bus_en_release_clocks is None:
                self.bus_en_release_clocks = clk
            if clk > self.release_grace and bus_en == 1:
                self.bus_en_high_clocks += 1
                if self.bus_en_high_clocks == 1:
                    self._note(
                        f"BUS_EN still asserted {clk} clocks after BGACK: the "
                        f"core has not three-stated the bus (UM 7.7.4 signal T)"
                    )

            if self._rd("ASn", 1) == 0:
                self.as_low_clocks += 1
                if self.as_low_clocks == 1:
                    addr = self._rd("ADR_OUT")
                    rw_n = self._rd("RWn")
                    self._note(
                        f"core asserted AS at clock {clk} of alternate-master "
                        f"ownership (addr="
                        f"{'?' if addr is None else format(addr, '#010x')}, "
                        f"RWn={rw_n}): bus contention"
                    )

    # ---- reporting -----------------------------------------------------

    def report(self):
        """One-line summary, for assertion messages."""
        return (
            f"grant_seen={self.grant_seen} grant_latency={self.grant_latency} "
            f"as_low_at_request={self.as_low_at_request} "
            f"as_low_at_grant={self.as_low_at_grant} "
            f"cycle_starts_before_grant={self.cycle_starts_before_grant} "
            f"owned_clocks={self.owned_clocks} "
            f"bus_en_release_clocks={self.bus_en_release_clocks} "
            f"bus_en_high_clocks={self.bus_en_high_clocks} "
            f"as_low_clocks={self.as_low_clocks} "
            f"violations={self.violations}"
        )
