"""Arbitration timing contract of MC68030UM 7.7, in the four places it broke.

Each test here started life as a reproducer for a diagnosed deviation from the
bus arbitration contract in MC68030UM Section 7.7; all four are now closed and
the module is an ordinary regression. Everything an external bus master can
observe about *when* the bus becomes available lives here, which is why they are
filed apart from test_bus_arbitration.py's protocol-shape tests.

All four had one root cause in sv/wf68k30L_bus_interface.sv: arb_dec gated both
of its ARB_IDLE transitions on `BUS_CTRL_STATE == BUS_IDLE`, which defers the
arbitration state change -- and with it BG -- to the *end* of the current bus
cycle. UM 7.7.2 defers it only until the cycle has *begun*. The fix drops that
gate and re-derives the three-state from UM 7.7.4's signal T, so the state change
is immediate and only the release waits for AS; that separation is also what
makes arbitration safe against a burst, which holds AS asserted across four long
words and is "a single cycle" for arbitration purposes (UM 7.3.7).

What each test pins down, and what it measured before the fix:

  1. BG must overlap a cycle already in progress. Measured: BG arrived 10
     clocks after BR with AS already negated.
     -> test_bus_grant_overlaps_an_in_progress_bus_cycle

  2. At most the one cycle already begun may delay BG -- not a whole split
     operand's worth of chained sub-cycles, which never return the bus
     controller to BUS_IDLE. Measured: 19 clocks, with a further sub-cycle
     starting first.
     -> test_bus_grant_not_deferred_across_a_split_operand_transfer

  3. Single-wire BGACK must pre-empt a pending cycle, not queue behind it.
     UM 7.7.4: observing AS negated on two consecutive clock edges "ensures
     that any current or pending bus activity has completed or has been
     pre-empted". Measured: 3 of 6 takeovers at the point UM 7.7.4 declares
     safe left the core still driving.
     -> test_single_wire_bgack_alone_preempts_a_pending_bus_cycle

  4. Relinquish and retry must break into the first read of a
     read-modify-write sequence, and only that one. arb_dec did try, with
     `if (RMC && !RETRY)`, but the exemption was unreachable: RETRY also holds
     BUS_CYC_RDY low, so bus_ctrl_dec cannot leave DATA_C1C4, and both
     remaining transitions needed BUS_IDLE. Measured: BG never asserted at
     all, so neither of UM 7.7.4's two escape hatches worked.
     -> test_relinquish_and_retry_breaks_into_first_rmc_read
"""

import cocotb
from cocotb.triggers import RisingEdge, ClockCycles

from cpu_harness import CPUTestHarness
from bus_model import AlternateBusMaster
from m68k_encode import (
    move, movea, moveq, imm_long, abs_long, tas, nop,
    LONG, DN, AN_IND, SPECIAL, IMMEDIATE, ABS_L,
)


def _rd(dut, name, default=None):
    try:
        return int(getattr(dut, name).value)
    except (ValueError, AttributeError):
        return default


def _busy_program(h):
    return [
        *movea(LONG, SPECIAL, IMMEDIATE, 0), *imm_long(h.RESULT_BASE),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 1), *imm_long(0xA5A55A5A),
        *move(LONG, DN, 1, AN_IND, 0),
        *moveq(0x33, 2),
        *move(LONG, DN, 2, AN_IND, 0),
        *h.sentinel_program(),
    ]


async def _wait_cycle_start(dut, limit=6000, want_addr=None, want_rw=None):
    """Advance to the rising edge on which AS falls."""
    prev = _rd(dut, "ASn", 1)
    for _ in range(limit):
        await RisingEdge(dut.CLK)
        as_n = _rd(dut, "ASn", 1)
        hit = prev == 1 and as_n == 0
        if hit and want_addr is not None and _rd(dut, "ADR_OUT") != want_addr:
            hit = False
        if hit and want_rw is not None and _rd(dut, "RWn") != want_rw:
            hit = False
        prev = as_n
        if hit:
            return True
    return False


@cocotb.test()
async def test_bus_grant_overlaps_an_in_progress_bus_cycle(dut):
    """UM 7.7.2: BG is asserted while the current cycle runs.

    "The processor asserts BG as soon as possible after receipt of BR. This is
    immediately following internal synchronization except during a
    read-modify-write cycle or following an internal decision to execute a bus
    cycle... In the case an internal decision to execute another bus cycle, BG
    is deferred until the bus cycle has begun."

    UM 7.7 adds: "This technique allows processing of bus requests during data
    transfer cycles", and UM 7.7.3 puts the burden of waiting for AS, DSACKx
    and BGACK on the *requester*, not on the grant.

    BR is asserted on the edge a long wait-stated cycle begins, so the cycle
    has already begun and no deferral is permitted: BG must appear while AS is
    still asserted, within the two clocks UM 7.7.4 allows for synchronizing an
    asynchronous input.

    WAS: sv/wf68k30L_bus_interface.sv arb_dec, ARB_IDLE state --
    `else if (!BR_In && BUS_CTRL_STATE == BUS_IDLE) NEXT_ARB_STATE = GRANT;`
    The BUS_CTRL_STATE == BUS_IDLE term deferred the grant until the cycle had
    ended. Dropping it also means BUS_EN (and DATA_PORT_EN) can no longer be
    `ARB_STATE == ARB_IDLE` alone: UM 7.7.4 says signal T places the buses in
    high impedance "after the next rising edge following the negation of AS
    and RMC", so the three-state is qualified on AS negation instead of on the
    arbitration state alone.
    """
    h = CPUTestHarness(dut, wait_states=6)
    await h.setup(_busy_program(h))
    alt = AlternateBusMaster(dut, responder=h.bus)

    await ClockCycles(dut.CLK, 40)
    assert await _wait_cycle_start(dut), "no bus cycle to request against"
    await alt.assert_br()
    granted = await alt.wait_for_grant(timeout=256)
    alt.negate_br()

    assert granted, f"BG never arrived: {alt.report()}"
    assert alt.as_low_at_request == 0, (
        f"test setup error: BR was not asserted during a cycle: {alt.report()}"
    )
    assert alt.as_low_at_grant == 0, (
        f"BG was deferred until the in-progress bus cycle had finished: AS was "
        f"already negated when BG appeared, {alt.grant_latency} clocks after "
        f"BR. UM 7.7.2 defers BG only until the cycle has begun, so BG must "
        f"overlap it. {alt.report()}"
    )
    h.cleanup()


@cocotb.test()
async def test_bus_grant_not_deferred_across_a_split_operand_transfer(dut):
    """UM 7.7.2: at most the cycle already begun may delay BG.

    A misaligned long word is transferred as several chained sub-cycles. Since
    UM 7.7.2 defers BG only "until the bus cycle has begun", no *further* bus
    cycle may start before BG appears -- otherwise arbitration latency scales
    with operand size, which UM 11.9 lists only for the address translation
    search, never for ordinary misaligned operands.

    WAS: sv/wf68k30L_bus_interface.sv bus_ctrl_dec keeps every sub-cycle of a
    split transfer inside DATA_C1C4 (it leaves only on
    `BUS_CYC_RDY && LAST_SUB_CYCLE`), so the `BUS_CTRL_STATE == BUS_IDLE` gate
    on arb_dec's grant transition could not be satisfied until the entire
    operand had been transferred. Same root cause as
    test_bus_grant_overlaps_an_in_progress_bus_cycle, but the deferral was
    unbounded in operand size rather than in one cycle.
    """
    h = CPUTestHarness(dut, wait_states=6)
    addr = h.DATA_BASE + 0x181           # misaligned long: split transfer
    h.mem.load_long(addr, 0x89ABCDEF)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0), *imm_long(h.RESULT_BASE),
        *move(LONG, SPECIAL, ABS_L, DN, 0), *abs_long(addr),
        *move(LONG, DN, 0, AN_IND, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    alt = AlternateBusMaster(dut, responder=h.bus)

    assert await _wait_cycle_start(dut, want_addr=addr, want_rw=1), (
        "never saw the first sub-cycle of the misaligned long read"
    )
    await alt.assert_br()
    granted = await alt.wait_for_grant(timeout=512)
    alt.negate_br()

    assert granted, f"BG never arrived: {alt.report()}"
    assert alt.cycle_starts_before_grant == 0, (
        f"{alt.cycle_starts_before_grant} further bus cycle(s) began between BR "
        f"and BG, so the grant was deferred across the whole split operand "
        f"transfer rather than across the one cycle already in progress "
        f"(grant took {alt.grant_latency} clocks). {alt.report()}"
    )
    h.cleanup()


@cocotb.test()
async def test_single_wire_bgack_alone_preempts_a_pending_bus_cycle(dut):
    """UM 7.7.4: the two-edge AS check is enough to take the bus safely.

    "Note that for the method to operate properly, AS must be observed to be
    negated (high) on two consecutive clock edges before the alternate bus
    master takes the bus. Waiting for this condition ensures that any current
    or pending bus activity has completed or has been pre-empted."

    The model asserts BGACK and then does exactly that (Figure 7-62 NOTE: the
    two edges are counted "after BGACK is recognized low"). The core must not
    drive the bus afterwards. Several attempts are made because the hazard needs
    the core to have a cycle queued at that moment; a conforming core passes
    every attempt.

    WAS: sv/wf68k30L_bus_interface.sv. arb_dec's forced-release transition
    carried the same `BUS_CTRL_STATE == BUS_IDLE` gate as the grant, so a
    BGACK that arrived while a cycle was committed could not change the
    arbitration state until that cycle ended -- and because the slave model is
    correctly off the bus by then, the cycle stalled with AS asserted until
    BGACK was released. Neither completed nor pre-empted. The fix asserts signal
    T immediately (Figure 7-61 state 0 -> state 4) and pre-empts a cycle that
    has been committed in START_CYCLE but has not driven AS yet; the release
    itself then waits for AS.
    """
    h = CPUTestHarness(dut, wait_states=1)
    await h.setup(_busy_program(h))

    attempts = 0
    offenders = []
    for _ in range(6):
        alt = AlternateBusMaster(dut, responder=h.bus)
        took = await alt.acquire_single_wire(hold_clocks=12, settled=2,
                                             timeout=512)
        if not took:
            break
        attempts += 1
        if alt.as_low_clocks or alt.bus_en_high_clocks:
            offenders.append(alt.report())
        await alt.release()
        await ClockCycles(dut.CLK, 6)

    assert attempts, "never managed a single-wire takeover attempt"
    assert not offenders, (
        f"{len(offenders)} of {attempts} single-wire takeovers made at the "
        f"point UM 7.7.4 declares safe left the core driving the bus:\n  "
        + "\n  ".join(offenders)
    )
    h.cleanup()


@cocotb.test()
async def test_relinquish_and_retry_breaks_into_first_rmc_read(dut):
    """UM 7.7.4: BERR+HALT+BR must work on the first RMC read.

    "One way for an alternate bus master to force the MC68030 to release the
    bus applies only to the first read cycle of an read-modify-write sequence.
    The MC68030 allows normal bus arbitration during this read cycle; a normal
    relinquish and retry operation (asserting BERR, HALT, and BR at the same
    time) is used."

    UM 7.5.2 says the same from the other side: "The MC68030 does not
    relinquish the bus during a read-modify-write operation, except during the
    first read cycle."

    BERR, HALT and BR are asserted together on the first (and only) read of a
    TAS, so BG must be asserted.

    WAS: sv/wf68k30L_bus_interface.sv. arb_dec did try to allow this -- its RMC
    lock read `if (RMC && !RETRY)` -- but the exemption was unreachable.
    BUS_CYC_RDY is forced to 0 while RETRY is asserted, so bus_ctrl_dec cannot
    leave DATA_C1C4, and arb_dec's grant transition additionally required
    `BUS_CTRL_STATE == BUS_IDLE`. RETRY is cleared on the same negedge that
    samples HALT negated, so there was never a clock in which
    RMC && RETRY && BUS_CTRL_STATE == BUS_IDLE all held. Combined with
    single-wire arbitration during RMC (fixed separately), this was the second
    of the two escape hatches UM 7.7.4 provides, and neither worked. The lock is
    now `RMC && !(RETRY && RMC_FIRST_READ)`, which is UM 7.5.2's limit exactly:
    a relinquish and retry lands the release, and only on the first read.
    """
    h = CPUTestHarness(dut)
    tas_addr = h.DATA_BASE + 0x40
    h.mem.write(tas_addr, 1, 0x00)
    berr_handler = 0x000800
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 1), *imm_long(tas_addr),
        *nop(), *nop(),
        *tas(AN_IND, 1),
        *nop(), *nop(),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    h.mem.load_long(2 * 4, berr_handler)
    h.mem.load_words(berr_handler, [*h.sentinel_program()])

    # Catch the first read of the locked sequence and relinquish-and-retry it.
    assert await _wait_cycle_start(dut, want_addr=tas_addr, want_rw=1), (
        "never saw the first read of the read-modify-write sequence"
    )
    assert _rd(dut, "RMCn", 1) == 0, "the TAS read was not a locked cycle"
    dut.BERRn.value = 0
    dut.HALT_INn.value = 0
    dut.BRn.value = 0

    berr_released = False
    bg_clocks = 0
    watched = 0
    for _ in range(160):
        await RisingEdge(dut.CLK)
        watched += 1
        if not berr_released and _rd(dut, "ASn", 1) == 1:
            dut.BERRn.value = 1     # UM 7.5.2: negate BERR, keep holding HALT
            berr_released = True
        if _rd(dut, "BGn", 1) == 0:
            bg_clocks += 1
    dut.BERRn.value = 1
    dut.HALT_INn.value = 1
    dut.BRn.value = 1

    assert bg_clocks > 0, (
        f"BG was never asserted in {watched} clocks of a relinquish-and-retry "
        f"on the first read of a read-modify-write sequence, so an alternate "
        f"bus master has no way to break into a locked sequence by the means "
        f"UM 7.7.4 documents"
    )
    h.cleanup()
