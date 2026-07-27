"""Unit checks for PTEST_ABORT on WF68K30L_TOP_MMU_PTEST.

A bus error on a core access resets EXEC_WB_STATE, which abandons the PTEST
instruction that requested a table search. MMU_PTEST_CONSUME is qualified on
OP_WB == PTEST, so it can never fire for an abandoned instruction, and
MMU_PTEST_START is gated on !MMU_PTEST_READY. So without an abort path a search
left in flight does not merely leak: it blocks every later PTEST, and the one
that waits collects the abandoned search's stale MMUSR instead of its own result.

What the abort deliberately does NOT do is tear the search down on the spot.
MMU_DESC_BUS_STATE is a registered FSM that may already have armed a descriptor
cycle for this search; dropping PTEST_BUSY underneath it would leave that bus
access with no owner, which the formal property "every descriptor access belongs
to a search in flight" rejects. So the abort records itself and retires the
published result, the search runs to its natural end, and completion returns to
idle instead of publishing. These tests encode that contract.
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, RisingEdge

# Short-format page descriptor: DT = 2'b01 terminates the walk at this level.
PAGE_DESC = 0x0010_0001
# DESC_LOOKUP is {no_bus_error, data[31:0]}.
LOOKUP_OK = (1 << 32) | PAGE_DESC


def _drive_idle(dut):
    dut.RESET_CPU.value = 1
    dut.PTEST_START.value = 0
    dut.PTEST_CONSUME.value = 0
    dut.PTEST_ABORT.value = 0
    dut.MMU_TC.value = 0x8000_8155
    # L/U = 1 (lower limit) with limit 0 so no index can violate it, DT = 2'b10
    # (valid 4-byte table descriptor) so a search walks rather than terminating at
    # the root, and a table address well clear of zero.
    crp = (1 << 63) | (0b10 << 32) | 0x0001_0000
    dut.MMU_CRP.value = crp
    dut.MMU_SRP.value = crp
    dut.PTEST_FC.value = 5
    dut.PTEST_LOGICAL.value = 0x1234_5008
    dut.PTEST_LEVEL.value = 7
    dut.DESC_DONE.value = 0
    dut.DESC_LOOKUP.value = 0


async def _boot(dut):
    cocotb.start_soon(Clock(dut.CLK, 10, unit="ns").start())
    _drive_idle(dut)
    await ClockCycles(dut.CLK, 3)
    dut.RESET_CPU.value = 0
    await ClockCycles(dut.CLK, 2)


async def _descriptor_responder(dut, stop):
    """Answer every descriptor request with a page descriptor.

    Without this the search waits forever and PTEST_BUSY never falls, so the
    abort's effect on the completion could not be observed at all.
    """
    while not stop["done"]:
        await RisingEdge(dut.CLK)
        if int(dut.DESC_REQ.value):
            await RisingEdge(dut.CLK)
            dut.DESC_LOOKUP.value = LOOKUP_OK
            dut.DESC_DONE.value = 1
            await RisingEdge(dut.CLK)
            dut.DESC_DONE.value = 0


async def _run_search(dut, abort_after=None, timeout=400):
    """Start a search, optionally abort it, and run it to completion.

    Returns (busy, ready) once the module is no longer busy, or raises on timeout.
    """
    dut.PTEST_START.value = 1
    await RisingEdge(dut.CLK)
    dut.PTEST_START.value = 0

    aborted = False
    for i in range(timeout):
        await RisingEdge(dut.CLK)
        if abort_after is not None and not aborted and i >= abort_after:
            dut.PTEST_ABORT.value = 1
            await RisingEdge(dut.CLK)
            dut.PTEST_ABORT.value = 0
            aborted = True
        if not int(dut.PTEST_BUSY.value) and i > 2:
            # Settled: busy has fallen, so the search has retired one way or the
            # other. One more edge so READY is stable.
            await RisingEdge(dut.CLK)
            return int(dut.PTEST_BUSY.value), int(dut.PTEST_READY.value)
    raise AssertionError(
        f"search never retired within {timeout} cycles "
        f"(BUSY={int(dut.PTEST_BUSY.value)}, READY={int(dut.PTEST_READY.value)})"
    )


@cocotb.test()
async def test_ptest_search_publishes_its_result(dut):
    """Baseline: an un-aborted search must finish with READY set.

    Without this the abort tests could pass simply because nothing ever publishes.
    """
    await _boot(dut)
    stop = {"done": False}
    cocotb.start_soon(_descriptor_responder(dut, stop))

    busy, ready = await _run_search(dut)
    stop["done"] = True

    assert busy == 0, "search still busy after retiring"
    assert ready == 1, (
        "An un-aborted search must publish its result in PTEST_READY; got 0, so "
        "this suite would not be able to tell a suppressed result from a normal one"
    )


@cocotb.test()
async def test_aborted_search_publishes_nothing(dut):
    """An abandoned search must retire without setting READY.

    A stale READY is what blocks the next PTEST (MMU_PTEST_START is gated on
    !MMU_PTEST_READY) and makes it report this search's MMUSR.
    """
    await _boot(dut)
    stop = {"done": False}
    cocotb.start_soon(_descriptor_responder(dut, stop))

    busy, ready = await _run_search(dut, abort_after=1)
    stop["done"] = True

    assert busy == 0, "aborted search never retired"
    assert ready == 0, (
        "PTEST_READY set for an abandoned search: the next PTEST would be blocked "
        "by it and would consume this stale result"
    )


@cocotb.test()
async def test_search_after_an_abort_publishes_its_own_result(dut):
    """A search following an aborted one is unaffected through the normal path.

    Scope, stated precisely because it is narrower than it looks: this covers the
    case where the aborted search reaches a completion arm, which clears the abort
    flag itself. It does NOT cover the flag leaking across searches -- that needs
    an abandoned search returning to idle WITHOUT reaching a completion arm, which
    this responder never produces. Mutation-checked: deleting the clear-at-start in
    wf68k30L_top_mmu_ptest.sv leaves all three tests here passing.

    That leak path is covered by the formal PTEST non-starvation property in
    formal/mmu_walk_delay_state_formal.sv, which is what found the missing clear in
    the first place. Do not treat this test as the guard for it.
    """
    await _boot(dut)
    stop = {"done": False}
    cocotb.start_soon(_descriptor_responder(dut, stop))

    busy, ready = await _run_search(dut, abort_after=1)
    assert busy == 0 and ready == 0, "first (aborted) search did not retire cleanly"

    busy, ready = await _run_search(dut)
    stop["done"] = True

    assert busy == 0, "second search never retired"
    assert ready == 1, (
        "The search after an aborted one published nothing, so the abort flag "
        "leaked across searches and PTEST is now permanently wedged"
    )
