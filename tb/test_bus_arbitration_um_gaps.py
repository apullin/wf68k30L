"""Arbitration deviations from MC68030UM 7.7 that are confirmed by test.

EVERY TEST IN THIS MODULE IS EXPECTED TO FAIL against the current RTL. Each
one reproduces a specific, diagnosed deviation from the bus arbitration
contract in MC68030UM Section 7.7. They are deliberately not weakened and not
skipped: an external bus master can observe every one of them, so they belong
in the tree until the RTL matches the manual.

All four have the same neighbourhood: WF68K30L_BUS_INTERFACE's arb_dec state
machine and the BUS_EN three-state qualifier next to it. They are filed as a
separate module from test_bus_arbitration.py (which passes) so the green
contract tests can be wired into the regular regression immediately while
these stay visible as known gaps.

Summary of the diagnoses, all in sv/wf68k30L_bus_interface.sv:

  1. arb_dec's ARB_IDLE -> GRANT transition is gated on
     `BUS_CTRL_STATE == BUS_IDLE`, so BG is deferred until the current bus
     cycle has *finished*. UM 7.7.2 defers BG only until the cycle "has
     begun". The requester is the party that qualifies on AS/DSACK/BGACK
     (UM 7.7.3), so the grant itself is supposed to overlap the cycle.
     -> test_bus_grant_overlaps_an_in_progress_bus_cycle

  2. The same gate is never satisfied part-way through a split operand
     transfer, because bus_ctrl_dec keeps chained sub-cycles inside
     DATA_C1C4 and never returns to BUS_IDLE between them. A grant can
     therefore be deferred by a whole misaligned or narrow-port operand.
     -> test_bus_grant_not_deferred_across_a_split_operand_transfer

  3. arb_dec's ARB_IDLE -> WAIT_RELEASE_3WIRE (single-wire BGACK) transition
     carries the same BUS_CTRL_STATE == BUS_IDLE gate, and bus_ctrl_dec can
     move BUS_IDLE -> START_CYCLE on the very edge the alternate master
     legitimately takes the bus. UM 7.7.4 states that observing AS negated
     on two consecutive clock edges "ensures that any current or pending bus
     activity has completed or has been pre-empted"; here the pending
     activity is neither.
     -> test_single_wire_bgack_alone_preempts_a_pending_bus_cycle

  4. There is no way at all to force the core off the bus during the first
     read of a read-modify-write sequence by relinquish and retry. arb_dec
     bypasses the RMC lock when RETRY is asserted, but RETRY also pins
     bus_ctrl_dec in DATA_C1C4 (BUS_CYC_RDY is forced low while RETRY), and
     both remaining arb_dec transitions require BUS_CTRL_STATE == BUS_IDLE.
     So the `!RETRY` term is dead and UM 7.7.4's first escape hatch is
     missing. Note this refutes, rather than confirms, the static reading
     that relinquish and retry is permitted on *any* RMC cycle: it is
     permitted on none.
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
    """EXPECTED FAIL. UM 7.7.2: BG is asserted while the current cycle runs.

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

    DIAGNOSIS: sv/wf68k30L_bus_interface.sv arb_dec, ARB_IDLE state --
    `else if (!BR_In && BUS_CTRL_STATE == BUS_IDLE) NEXT_ARB_STATE = GRANT;`
    The BUS_CTRL_STATE == BUS_IDLE term defers the grant until the cycle has
    ended. Removing it also means BUS_EN (and DATA_PORT_EN) can no longer be
    `ARB_STATE == ARB_IDLE` alone: UM 7.7.4 says signal T places the buses in
    high impedance "after the next rising edge following the negation of AS
    and RMC", so the three-state has to be qualified on AS/RMC negation
    instead of on the arbitration state alone.
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
    """EXPECTED FAIL. UM 7.7.2: at most the cycle already begun may delay BG.

    A misaligned long word is transferred as several chained sub-cycles. Since
    UM 7.7.2 defers BG only "until the bus cycle has begun", no *further* bus
    cycle may start before BG appears -- otherwise arbitration latency scales
    with operand size, which UM 11.9 lists only for the address translation
    search, never for ordinary misaligned operands.

    DIAGNOSIS: sv/wf68k30L_bus_interface.sv bus_ctrl_dec keeps every sub-cycle
    of a split transfer inside DATA_C1C4 (it leaves only on
    `BUS_CYC_RDY && SIZE_N == 3'b000`), so the `BUS_CTRL_STATE == BUS_IDLE`
    gate on arb_dec's grant transition cannot be satisfied until the entire
    operand has been transferred. Same root cause as
    test_bus_grant_overlaps_an_in_progress_bus_cycle, but the deferral is
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
    """EXPECTED FAIL. UM 7.7.4: the two-edge AS check is meant to be enough.

    "Note that for the method to operate properly, AS must be observed to be
    negated (high) on two consecutive clock edges before the alternate bus
    master takes the bus. Waiting for this condition ensures that any current
    or pending bus activity has completed or has been pre-empted."

    The model does exactly that and then asserts BGACK. The core must not
    drive the bus afterwards. Several attempts are made because the hazard
    needs the core to have a cycle queued at that moment; a conforming core
    passes every attempt.

    DIAGNOSIS: sv/wf68k30L_bus_interface.sv. In back-to-back traffic AS is
    high for exactly two rising edges: one in BUS_CTRL_STATE == BUS_IDLE and
    one in START_CYCLE. Asserting BGACK on the second of those is legal per
    UM 7.7.4, but BGACK_In is only sampled on the following negedge, by which
    time bus_ctrl_dec has already committed BUS_IDLE -> START_CYCLE ->
    DATA_C1C4; arb_dec's `!BGACK_In && BUS_CTRL_STATE == BUS_IDLE` guard then
    cannot fire until that cycle ends. The pending activity is neither
    completed nor pre-empted, and because the slave model is correctly off the
    bus the cycle also stalls with AS asserted until BGACK is released. The UM
    behaviour is to assert signal T immediately (state 0 -> state 4) and let
    the three-state wait for AS to negate, rather than to gate the state
    change on the bus controller being idle.
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
    """EXPECTED FAIL. UM 7.7.4: BERR+HALT+BR must work on the first RMC read.

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

    DIAGNOSIS: sv/wf68k30L_bus_interface.sv. arb_dec does try to allow this --
    its RMC lock reads `if (RMC && !RETRY)` -- but the exemption is
    unreachable. BUS_CYC_RDY is forced to 0 while RETRY is asserted, so
    bus_ctrl_dec cannot leave DATA_C1C4, and arb_dec's grant transition
    additionally requires `BUS_CTRL_STATE == BUS_IDLE`. RETRY is cleared on
    the same negedge that samples HALT negated, so there is never a clock in
    which RMC && RETRY && BUS_CTRL_STATE == BUS_IDLE all hold. Combined with
    single-wire arbitration during RMC (fixed separately), this was the second
    of the two escape hatches UM 7.7.4 provides, and neither worked.
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
