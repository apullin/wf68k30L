"""Bus arbitration and read-modify-write indivisibility (MC68030UM 7.7, 7.3.3).

These are the contract properties an external bus master or a driver can
legitimately depend on:

  - The three-wire handshake: BR -> BG -> BGACK -> release (UM 7.7).
  - The single-wire handshake: BGACK alone forces the processor off the bus
    (UM 7.7.4, the state 0 -> state 4 path of Figure 7-61).
  - While an alternate master owns the bus the processor must be off it: no
    AS assertion and BUS_EN negated, which is what WF68K30L_CPU_WRAPPER uses
    to three-state exactly the outputs UM Table 5-2 marks three-stateable.
  - RMC is asserted continuously over a whole read-modify-write sequence, BG
    is withheld for its duration, and no foreign bus cycle -- including a
    background cache fill -- appears inside it (UM 5.6.4, 7.3.3).

Cycle accuracy is not the goal; contract accuracy is.

Note on high impedance: Verilator flattens `1'bz` to 0, so the actual
floating of A/FC/SIZE/AS/... in the CPU wrapper cannot be observed in this
simulator. What is checkable, and is checked here, is the core-side qualifier
BUS_EN plus a structural check that the wrapper gates exactly the UM Table
5-2 three-state outputs with it and leaves ECS/OCS driven.

Deviations from the UM that are confirmed by test but not fixed live in
test_bus_arbitration_um_gaps.py.
"""

import os
import re

import cocotb
from cocotb.triggers import RisingEdge, ClockCycles

from cpu_harness import CPUTestHarness
from bus_model import AlternateBusMaster
from m68k_encode import (
    move, movea, moveq, imm_long, abs_long, tas, nop, divu_l, movec_to_cr,
    LONG, DN, AN_IND, SPECIAL, IMMEDIATE,
)

CR_CACR = 0x002
# CACR: EI(0) + IBE(4) + ED(8) + DBE(12) -- both caches with burst filling.
CACR_CACHES_AND_BURST = 0x0000_1111

FC_SUPER_DATA = 5
FC_SUPER_PROG = 6


def _busy_program(h, extra=()):
    """A program with continuous bus traffic and a checkable result."""
    return [
        *movea(LONG, SPECIAL, IMMEDIATE, 0), *imm_long(h.RESULT_BASE),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 1), *imm_long(0xA5A55A5A),
        *move(LONG, DN, 1, AN_IND, 0),
        *moveq(0x33, 2),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 3), *imm_long(0x0F0F0F0F),
        *extra,
        *h.sentinel_program(),
    ]


def _tas_program(h, tas_addr, lead_in=(), trailer=()):
    return [
        *movea(LONG, SPECIAL, IMMEDIATE, 1), *imm_long(tas_addr),
        *lead_in,
        *nop(), *nop(),
        *tas(AN_IND, 1),
        *nop(), *nop(),
        *trailer,
        *h.sentinel_program(),
    ]


async def _wait_for(dut, predicate, limit=6000):
    """Advance to the first rising edge where predicate(dut) is true."""
    for _ in range(limit):
        await RisingEdge(dut.CLK)
        try:
            if predicate(dut):
                return True
        except ValueError:
            continue
    return False


def _rd(dut, name, default=None):
    try:
        return int(getattr(dut, name).value)
    except (ValueError, AttributeError):
        return default


# ===================================================================
# Three-wire arbitration (UM 7.7.1 - 7.7.3)
# ===================================================================

@cocotb.test()
async def test_three_wire_arbitration_grant_take_release(dut):
    """UM 7.7 protocol: BR, then BG, then BGACK, then the bus comes back.

    Checks the whole handshake plus the ownership contract: while BGACK is
    held the core must not assert AS and must negate BUS_EN, and once BGACK is
    released the core must drive the bus again and finish its program.
    """
    h = CPUTestHarness(dut, wait_states=1)
    await h.setup(_busy_program(h))
    alt = AlternateBusMaster(dut, responder=h.bus)

    await ClockCycles(dut.CLK, 60)
    took = await alt.acquire(hold_clocks=24)
    assert took, f"never became bus master: {alt.report()}"
    assert alt.grant_seen, f"BG was never asserted: {alt.report()}"

    assert alt.bus_en_release_clocks is not None, (
        f"BUS_EN never negated while the alternate master owned the bus, so "
        f"the CPU wrapper would keep driving A/FC/SIZE/AS/DS/RW/RMC against "
        f"it (UM 7.7.4 signal T): {alt.report()}"
    )
    assert not alt.violations, f"bus contention during ownership: {alt.report()}"
    assert not h.bus.foreign_cycle_starts, (
        f"core ran bus cycles while the alternate master held BGACK: "
        f"{h.bus.foreign_cycle_starts}"
    )

    await alt.release()
    # UM 7.7.3: mastership terminates at the negation of BGACK, so the core
    # owns the bus again immediately afterwards.
    back = await _wait_for(dut, lambda d: _rd(d, "BUS_EN") == 1, limit=32)
    assert back, "BUS_EN was not restored after BGACK was negated"

    found = await h.run_until_sentinel(max_cycles=8000)
    assert found, "program did not complete after the bus was handed back"
    assert h.read_result_long(0) == 0xA5A55A5A, (
        f"result corrupted by arbitration: 0x{h.read_result_long(0):08X}"
    )
    h.cleanup()


@cocotb.test()
async def test_bus_request_withdrawn_without_acknowledge(dut):
    """UM 7.7.1: with no BGACK, the processor stays bus master once BR negates.

    "This prevents unnecessary interference with ordinary processing if the
    arbitration circuitry inadvertently responds to noise or if an external
    device determines that it no longer requires use of the bus before it has
    been granted mastership."
    """
    h = CPUTestHarness(dut, wait_states=1)
    await h.setup(_busy_program(h))
    alt = AlternateBusMaster(dut, responder=h.bus)

    await ClockCycles(dut.CLK, 60)
    await alt.assert_br()
    assert await alt.wait_for_grant(timeout=256), (
        f"BG was never asserted for a bus request: {alt.report()}"
    )
    alt.negate_br()

    # BG must drop again, and the core must never have three-stated: it was
    # never acknowledged, so it is still bus master.
    dropped = await _wait_for(dut, lambda d: _rd(d, "BGn") == 1, limit=64)
    assert dropped, "BG stayed asserted after BR was withdrawn"
    for _ in range(32):
        await RisingEdge(dut.CLK)
        assert _rd(dut, "BUS_EN") == 1, (
            "core gave up the bus without ever seeing BGACK (UM 7.7.1)"
        )

    found = await h.run_until_sentinel(max_cycles=8000)
    assert found, "program did not complete after a withdrawn bus request"
    assert h.read_result_long(0) == 0xA5A55A5A
    h.cleanup()


@cocotb.test()
async def test_bus_grant_regranted_for_a_still_pending_request(dut):
    """UM 7.7.3: a request still pending at BGACK time gets another BG.

    "If a BR is still pending after the assertion of BGACK, another BG is
    asserted within a few clocks of the negation of BG... Note that the
    processor does not perform any external bus cycles before it reasserts BG
    in this case."
    """
    h = CPUTestHarness(dut, wait_states=1)
    await h.setup(_busy_program(h))
    alt = AlternateBusMaster(dut, responder=h.bus)

    await ClockCycles(dut.CLK, 60)
    await alt.assert_br()
    assert await alt.wait_for_grant(timeout=256), alt.report()
    assert await alt.wait_bus_free(timeout=256), alt.report()
    # Keep BR asserted: a second master is still waiting.
    await alt.assert_bgack(negate_br=False)
    await alt.hold(8)
    await alt.release()

    # BG has to come back, and no processor bus cycle may run first.
    regranted = 0
    as_low_before_regrant = 0
    for _ in range(64):
        await RisingEdge(dut.CLK)
        if _rd(dut, "BGn") == 0:
            regranted += 1
            break
        if _rd(dut, "ASn", 1) == 0:
            as_low_before_regrant += 1
    alt.negate_br()
    assert regranted, (
        "BG was not reasserted for the request still pending after BGACK "
        "(UM 7.7.3)"
    )
    assert as_low_before_regrant == 0, (
        f"processor ran {as_low_before_regrant} clocks of bus cycle before "
        f"reasserting BG, which UM 7.7.3 forbids"
    )

    found = await h.run_until_sentinel(max_cycles=8000)
    assert found, "program did not complete after the re-grant"
    h.cleanup()


@cocotb.test()
async def test_arbitration_recognized_while_processor_halted(dut):
    """UM 7.7: "Bus arbitration requests are recognized during normal
    processing, RESET assertion, HALT assertion, and even when the processor
    has halted due to a double bus fault."
    """
    h = CPUTestHarness(dut, wait_states=1)
    await h.setup(_busy_program(h))
    alt = AlternateBusMaster(dut, responder=h.bus)

    await ClockCycles(dut.CLK, 60)
    # Halt between bus cycles (UM 7.5.3), then arbitrate.
    assert await _wait_for(dut, lambda d: _rd(d, "ASn", 0) == 1, limit=256)
    dut.HALT_INn.value = 0
    await ClockCycles(dut.CLK, 8)

    await alt.assert_br()
    granted = await alt.wait_for_grant(timeout=256)
    assert granted, f"no BG while the processor was halted: {alt.report()}"
    assert await alt.wait_bus_free(timeout=256), alt.report()
    await alt.assert_bgack()
    await alt.hold(16)
    assert not alt.violations, f"contention while halted: {alt.report()}"
    await alt.release()

    dut.HALT_INn.value = 1
    found = await h.run_until_sentinel(max_cycles=12000)
    assert found, "program did not resume after halt plus arbitration"
    assert h.read_result_long(0) == 0xA5A55A5A
    h.cleanup()


# ===================================================================
# Single-wire arbitration (UM 7.7.4)
# ===================================================================

@cocotb.test()
async def test_single_wire_bgack_alone_releases_idle_bus(dut):
    """UM 7.7.4: BGACK alone takes the bus, with no BR and no BG.

    "As shown by the path from state 0 to state 4, BGACK alone can be used to
    place the processor's external bus buffers in the high-impedance state,
    providing single-wire arbitration capability."

    The takeover is placed in the long idle window of a DIVU.L so that no
    processor cycle is already committed; the narrower question of whether the
    UM's own two-consecutive-edges precondition is sufficient is a separate
    test in test_bus_arbitration_um_gaps.py.
    """
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0), *imm_long(h.RESULT_BASE),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 1), *imm_long(0x7FFFFFFF),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 2), *imm_long(0x00000003),
        *divu_l(DN, 2, 1, 3),
        *move(LONG, DN, 1, AN_IND, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    alt = AlternateBusMaster(dut, responder=h.bus)

    took = await alt.acquire_single_wire(hold_clocks=20, settled=8,
                                         timeout=2000)
    assert took, "never found an idle window to take the bus in"
    assert alt.bus_en_release_clocks is not None, (
        f"BUS_EN never negated for a single-wire BGACK: {alt.report()}"
    )
    assert not alt.violations, f"contention after single-wire takeover: {alt.report()}"
    assert not h.bus.foreign_cycle_starts, (
        f"core ran cycles while BGACK was held: {h.bus.foreign_cycle_starts}"
    )
    await alt.release()

    found = await h.run_until_sentinel(max_cycles=20000)
    assert found, "program did not complete after a single-wire takeover"
    assert h.read_result_long(0) == 0x2AAAAAAA, (
        f"DIVU.L result corrupted: 0x{h.read_result_long(0):08X}"
    )
    h.cleanup()

# ===================================================================
# Read-modify-write indivisibility (UM 5.6.4, 7.3.3)
# ===================================================================

@cocotb.test()
async def test_bus_grant_withheld_during_read_modify_write(dut):
    """UM 7.3.3: "The MC68030 does not issue a bus grant (BG) signal in
    response to a bus request (BR) signal during this operation."

    Figure 7-61 NOTE: "The BG output will not be asserted while RMC is
    asserted."
    """
    h = CPUTestHarness(dut)
    tas_addr = h.DATA_BASE + 0x40
    h.mem.write(tas_addr, 1, 0x00)
    await h.setup(_tas_program(h, tas_addr))
    alt = AlternateBusMaster(dut, responder=h.bus)

    assert await _wait_for(dut, lambda d: _rd(d, "RMCn", 1) == 0), (
        "RMC was never asserted for TAS"
    )
    await alt.assert_br()

    rmc_clocks = 0
    bg_while_rmc = 0
    for _ in range(600):
        await RisingEdge(dut.CLK)
        rmcn = _rd(dut, "RMCn", 1)
        if rmcn == 0:
            rmc_clocks += 1
            if _rd(dut, "BGn", 1) == 0:
                bg_while_rmc += 1
        elif rmc_clocks:
            break
    assert rmc_clocks > 4, f"RMC window too short to be meaningful: {rmc_clocks}"
    assert bg_while_rmc == 0, (
        f"BG was asserted for {bg_while_rmc} of {rmc_clocks} clocks while RMC "
        f"was asserted, breaking the indivisibility of the sequence"
    )

    # Once the lock is released the request has to be honoured.
    granted = await alt.wait_for_grant(timeout=512)
    alt.negate_br()
    assert granted, (
        f"BG never arrived after RMC negated, so the request was lost rather "
        f"than deferred: {alt.report()}"
    )

    found = await h.run_until_sentinel(max_cycles=12000)
    assert found, "TAS program did not complete"
    assert h.mem.read(tas_addr, 1) == 0x80
    h.cleanup()


@cocotb.test()
async def test_rmc_asserted_continuously_across_read_modify_write(dut):
    """UM 5.6.4: RMC "remains asserted during all bus cycles of the
    read-modify-write operation".

    The interesting part is the gap between the read and the write: an
    external lock built on RMC has to stay closed there too, so RMC must not
    blink between the two halves of the sequence.
    """
    h = CPUTestHarness(dut)
    tas_addr = h.DATA_BASE + 0x40
    h.mem.write(tas_addr, 1, 0x00)
    await h.setup(_tas_program(h, tas_addr))

    windows = []          # contiguous runs of RMCn low
    run = 0
    read_window = None
    write_window = None
    prev_as = 1
    for _ in range(6000):
        await RisingEdge(dut.CLK)
        rmcn = _rd(dut, "RMCn")
        as_n = _rd(dut, "ASn", 1)
        if rmcn is None:
            continue
        if rmcn == 0:
            run += 1
            if prev_as == 1 and as_n == 0 and _rd(dut, "ADR_OUT") == tas_addr:
                if _rd(dut, "RWn", 1) == 1:
                    read_window = len(windows)
                else:
                    write_window = len(windows)
        elif run:
            windows.append(run)
            run = 0
        prev_as = as_n
        if h.mem.read(h.SENTINEL_ADDR, 4) == h.SENTINEL_VAL:
            break
    if run:
        windows.append(run)

    assert h.mem.read(tas_addr, 1) == 0x80, "TAS did not run"
    assert windows, "RMC was never asserted"
    assert read_window is not None, "no locked read to the TAS operand"
    assert write_window is not None, "no locked write to the TAS operand"
    assert read_window == write_window, (
        f"RMC was negated between the read and the write of the sequence: the "
        f"read fell in RMC window {read_window} and the write in window "
        f"{write_window}; window lengths (clocks) = {windows}"
    )
    h.cleanup()


@cocotb.test()
async def test_no_background_cache_fill_inside_read_modify_write(dut):
    """UM 7.3.3: "No burst filling of the data cache occurs during a
    read-modify-write operation."

    Both caches and burst filling are enabled and the device acknowledges
    burst mode, so the core's background line-fill cycles really do run -- the
    test asserts that at least one of them happened, which is what keeps it
    from passing vacuously -- and none of them lands inside the RMC window.
    """
    h = CPUTestHarness(dut)
    tas_addr = h.DATA_BASE + 0x2040
    line_addr = h.DATA_BASE + 0x1000     # cache line armed for a burst fill
    h.mem.write(tas_addr, 1, 0x00)
    for i in range(16):
        h.mem.load_long(line_addr + 4 * i, 0x1000_0000 + i)

    program = _tas_program(
        h, tas_addr,
        lead_in=[
            *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
            *imm_long(CACR_CACHES_AND_BURST),
            *movec_to_cr(0, CR_CACR),
            *movea(LONG, SPECIAL, IMMEDIATE, 2), *imm_long(line_addr),
            # Arms a data-cache burst fill for the whole line.
            *move(LONG, AN_IND, 2, DN, 3),
        ],
    )
    await h.setup(program)
    dut.CBACKn.value = 0                 # slave supports burst mode

    inside_rmc = []
    line_cycles = []
    prev_as = 1
    for _ in range(10000):
        await RisingEdge(dut.CLK)
        as_n = _rd(dut, "ASn", 1)
        rmcn = _rd(dut, "RMCn", 1)
        adr = _rd(dut, "ADR_OUT")
        if adr is None:
            continue
        if prev_as == 1 and as_n == 0:
            rec = (adr, _rd(dut, "RWn"), _rd(dut, "FC_OUT"))
            if line_addr <= adr < line_addr + 0x40:
                line_cycles.append(rec)
            if rmcn == 0:
                inside_rmc.append(rec)
        prev_as = as_n
        if h.mem.read(h.SENTINEL_ADDR, 4) == h.SENTINEL_VAL:
            break

    assert h.mem.read(tas_addr, 1) == 0x80, "TAS did not run"
    background = [c for c in line_cycles if c[0] != line_addr]
    assert background, (
        f"no background cache-fill cycle ran at all, so this test would not "
        f"have detected one inside RMC; line cycles seen = "
        f"{[(hex(a), rw, fc) for a, rw, fc in line_cycles]}"
    )
    strays = [c for c in inside_rmc if not (c[0] == tas_addr
                                            or c[2] == FC_SUPER_PROG)]
    assert not strays, (
        f"foreign bus cycle inside the read-modify-write sequence: "
        f"{[(hex(a), rw, fc) for a, rw, fc in strays]}"
    )
    assert not [c for c in inside_rmc if line_addr <= c[0] < line_addr + 0x40], (
        f"a cache-line fill cycle ran inside the RMC window: "
        f"{[(hex(a), rw, fc) for a, rw, fc in inside_rmc]}"
    )
    h.cleanup()


# ===================================================================
# Three-state control (UM Table 5-2)
# ===================================================================

@cocotb.test()
async def test_wrapper_three_states_exactly_the_table_5_2_outputs(dut):
    """UM Table 5-2: only the outputs marked three-stateable may be released.

    ECS, OCS, BG, IPEND, STATUS and REFILL are marked "Three-State: No" and
    must keep driving while the processor is not the bus master; A, FC, SIZ,
    R/W, RMC, AS, DS, DBEN, CIOUT and CBREQ are marked "Yes".

    This is a structural check on WF68K30L_CPU_WRAPPER rather than a
    simulation one: Verilator flattens `1'bz` to 0, so the floating state
    cannot be observed at all in this testbench. The core-side qualifier
    BUS_EN is checked by simulation in the arbitration tests above.
    """
    wrapper = os.path.join(os.path.dirname(__file__), "..", "sv",
                           "wf68k30L_cpu_wrapper.sv")
    with open(wrapper) as f:
        src = f.read()

    gated = set(re.findall(
        r"assign\s+(\w+)\s*=\s*BUS_EN\s*\?[^;]*'[hbz]*z", src, re.IGNORECASE))
    must_release = {"A", "FC", "SIZE", "ASn", "RWn", "RMCn", "DSn",
                    "CIOUTn", "CBREQn", "DBENn"}
    must_keep_driving = {"ECSn", "OCSn", "BGn", "IPENDn", "STATUSn", "REFILLn"}

    missing = must_release - gated
    assert not missing, (
        f"UM Table 5-2 marks these three-stateable but the wrapper drives "
        f"them unconditionally: {sorted(missing)}"
    )
    wrong = must_keep_driving & gated
    assert not wrong, (
        f"UM Table 5-2 marks these as not three-stateable, so they must keep "
        f"driving while the bus is granted away: {sorted(wrong)}"
    )
    # The data bus has its own qualifier, which already folds in BUS_EN.
    assert re.search(r"assign\s+D\s*=\s*DATA_EN\s*\?", src), (
        "the wrapper no longer three-states the data bus on DATA_EN"
    )
    dut._log.info("wrapper three-states %s and keeps %s driving",
                  sorted(gated), sorted(must_keep_driving))
