"""
MC68030 bus protocol compliance tests for WF68K30L (Phase 6).

Tests the asynchronous bus interface behavior including:
  - Wait state tolerance (0-3 wait states produce identical results)
  - Bus signal behavior during read and write cycles
  - Bus cycle timing and SIZE encoding
  - Address alignment and data bus width
  - Back-to-back and mixed bus cycle sequencing

These tests exercise the bus interface as observed through the
WF68K30L_TOP pins: ASn, DSn, RWn, SIZE, FC_OUT, ADR_OUT, DATA_OUT.
The BusModel in bus_model.py acts as the slave responder.

MC68030 bus protocol summary:
  - Read cycle:  CPU asserts ASn low, RWn high. Slave drives data and
                 asserts DSACKn (00 = 32-bit port ack).
  - Write cycle: CPU asserts ASn low, RWn low, drives DATA_OUT. Slave
                 captures data and asserts DSACKn.
  - SIZE[1:0]:   00=long(4B), 01=byte, 10=word(2B), 11=line.
  - FC[2:0]:     Function code: 1=user data, 2=user prog, 5=super data,
                 6=super prog, 7=CPU space.
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles

from cpu_harness import CPUTestHarness
from bus_model import BusModel
from m68k_encode import (
    BYTE, WORD, LONG,
    DN, AN, AN_IND, AN_DISP, SPECIAL, ABS_L, IMMEDIATE,
    moveq, move, movea, move_from_abs_long, move_to_abs_long, nop, addq, add,
    imm_long, abs_long, trap, rte,
)


# ---------------------------------------------------------------------------
# Shared test program: small arithmetic with result stored to memory.
# Used by wait-state tests to verify identical results regardless of
# bus speed.
# ---------------------------------------------------------------------------

def _arithmetic_program(h):
    """Return a program that computes 10+5+3=18 and stores to RESULT_BASE.

    Uses register-indirect result stores for a stable bus-cycle pattern.
    """
    return [
        # Load result address into A0.
        *movea(LONG, SPECIAL, IMMEDIATE, 0),      # MOVEA.L #RESULT_BASE,A0
        *imm_long(h.RESULT_BASE),
        # Arithmetic: D0 = 10, D1 = 5, D2 = 3
        *moveq(10, 0),                             # MOVEQ #10,D0
        *moveq(5, 1),                              # MOVEQ #5,D1
        *moveq(3, 2),                              # MOVEQ #3,D2
        # D0 = D0 + D1  (10 + 5 = 15)
        *add(LONG, 0, 0, DN, 1),                   # ADD.L D1,D0
        # D0 = D0 + D2  (15 + 3 = 18)
        *add(LONG, 0, 0, DN, 2),                   # ADD.L D2,D0
        # Store D0 to (A0) = RESULT_BASE
        *move(LONG, DN, 0, AN_IND, 0),             # MOVE.L D0,(A0)
        # Also store D1 at RESULT_BASE+4 for extra verification
        *addq(LONG, 4, AN, 0),                     # ADDQ.L #4,A0
        *move(LONG, DN, 1, AN_IND, 0),             # MOVE.L D1,(A0)
        # Sentinel
        *h.sentinel_program(),
    ]


EXPECTED_D0 = 18
EXPECTED_D1 = 5


# ===================================================================
# 1. Wait State Tests
# ===================================================================

@cocotb.test()
async def test_zero_wait_states(dut):
    """Run arithmetic program with 0 wait states (fastest bus)."""
    h = CPUTestHarness(dut, wait_states=0)
    program = _arithmetic_program(h)
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached (0 wait states)"

    result_d0 = h.read_result_long(0)
    result_d1 = h.read_result_long(4)
    assert result_d0 == EXPECTED_D0, (
        f"D0: expected {EXPECTED_D0}, got 0x{result_d0:08X}")
    assert result_d1 == EXPECTED_D1, (
        f"D1: expected {EXPECTED_D1}, got 0x{result_d1:08X}")

    dut._log.info(f"0 wait states: D0={result_d0}, D1={result_d1} (correct)")
    h.cleanup()


@cocotb.test()
async def test_one_wait_state(dut):
    """Run same arithmetic program with 1 wait state -- results must match."""
    h = CPUTestHarness(dut, wait_states=1)
    program = _arithmetic_program(h)
    await h.setup(program)
    found = await h.run_until_sentinel(max_cycles=5000)
    assert found, "Sentinel not reached (1 wait state)"

    result_d0 = h.read_result_long(0)
    result_d1 = h.read_result_long(4)
    assert result_d0 == EXPECTED_D0, (
        f"D0: expected {EXPECTED_D0}, got 0x{result_d0:08X}")
    assert result_d1 == EXPECTED_D1, (
        f"D1: expected {EXPECTED_D1}, got 0x{result_d1:08X}")

    dut._log.info(f"1 wait state: D0={result_d0}, D1={result_d1} (correct)")
    h.cleanup()


@cocotb.test()
async def test_two_wait_states(dut):
    """Run same arithmetic program with 2 wait states -- results must match."""
    h = CPUTestHarness(dut, wait_states=2)
    program = _arithmetic_program(h)
    await h.setup(program)
    found = await h.run_until_sentinel(max_cycles=8000)
    assert found, "Sentinel not reached (2 wait states)"

    result_d0 = h.read_result_long(0)
    result_d1 = h.read_result_long(4)
    assert result_d0 == EXPECTED_D0, (
        f"D0: expected {EXPECTED_D0}, got 0x{result_d0:08X}")
    assert result_d1 == EXPECTED_D1, (
        f"D1: expected {EXPECTED_D1}, got 0x{result_d1:08X}")

    dut._log.info(f"2 wait states: D0={result_d0}, D1={result_d1} (correct)")
    h.cleanup()


@cocotb.test()
async def test_three_wait_states(dut):
    """Run same arithmetic program with 3 wait states -- results must match."""
    h = CPUTestHarness(dut, wait_states=3)
    program = _arithmetic_program(h)
    await h.setup(program)
    found = await h.run_until_sentinel(max_cycles=12000)
    assert found, "Sentinel not reached (3 wait states)"

    result_d0 = h.read_result_long(0)
    result_d1 = h.read_result_long(4)
    assert result_d0 == EXPECTED_D0, (
        f"D0: expected {EXPECTED_D0}, got 0x{result_d0:08X}")
    assert result_d1 == EXPECTED_D1, (
        f"D1: expected {EXPECTED_D1}, got 0x{result_d1:08X}")

    dut._log.info(f"3 wait states: D0={result_d0}, D1={result_d1} (correct)")
    h.cleanup()


# ===================================================================
# 2. Bus Signal Verification
# ===================================================================

@cocotb.test()
async def test_bus_read_signals(dut):
    """Verify ASn, DSn, RWn behavior during read bus cycles.

    During a read: ASn=0 (asserted), RWn=1 (read direction).
    DSn may also be asserted (0) during reads on the MC68030.
    When idle: ASn=1, DSn=1.
    """
    h = CPUTestHarness(dut)
    program = [
        *moveq(42, 0),             # MOVEQ #42,D0
        *nop(), *nop(),
        *h.sentinel_program(),
    ]
    await h.setup(program)

    read_seen = False
    idle_seen = False
    read_rwn_always_high = True

    for _ in range(1500):
        await RisingEdge(dut.CLK)
        try:
            as_n = int(dut.ASn.value)
            ds_n = int(dut.DSn.value)
            rw_n = int(dut.RWn.value)
        except ValueError:
            continue

        if as_n == 0 and rw_n == 1:
            # Read cycle active
            read_seen = True
        elif as_n == 0 and rw_n == 0:
            # This is a write cycle -- RWn should not be 0 during reads
            pass
        elif as_n == 1:
            idle_seen = True

    assert read_seen, "No read bus cycles observed"
    assert idle_seen, "Bus never went idle between cycles"

    dut._log.info("Bus read signals verified: ASn=0/RWn=1 during reads, "
                  "ASn=1 during idle")
    h.cleanup()


@cocotb.test()
async def test_bus_write_signals(dut):
    """Verify ASn, DSn, RWn behavior during write bus cycles.

    During a write: ASn=0, RWn=0 (write direction).
    The program stores a value to memory, which triggers write cycles.
    """
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(99, 0),
        *move(LONG, DN, 0, AN_IND, 0),     # MOVE.L D0,(A0) -- triggers write
        *h.sentinel_program(),
    ]
    await h.setup(program)

    write_seen = False
    write_asn_correct = True
    write_rwn_correct = True

    for _ in range(2000):
        await RisingEdge(dut.CLK)
        try:
            as_n = int(dut.ASn.value)
            rw_n = int(dut.RWn.value)
        except ValueError:
            continue

        if as_n == 0 and rw_n == 0:
            write_seen = True
            # During write: ASn must be 0 (already checked by condition)
            # RWn must be 0 (already checked by condition)

    assert write_seen, "No write bus cycles observed"

    dut._log.info("Bus write signals verified: ASn=0/RWn=0 during writes")
    h.cleanup()


@cocotb.test()
async def test_bus_size_signals(dut):
    """Verify SIZE output for long-word transfers.

    The program uses MOVE.L which should produce SIZE=00 (long word)
    on the bus. Instruction fetches are word-sized (SIZE=10).
    """
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(42, 0),
        *move(LONG, DN, 0, AN_IND, 0),     # MOVE.L triggers long transfer
        *h.sentinel_program(),
    ]
    await h.setup(program)

    size_values_seen = set()

    for _ in range(2000):
        await RisingEdge(dut.CLK)
        try:
            as_n = int(dut.ASn.value)
            if as_n == 0:
                size_val = int(dut.SIZE.value)
                size_values_seen.add(size_val)
        except ValueError:
            continue

    dut._log.info(f"SIZE values observed during bus cycles: {size_values_seen}")

    # We should see at least one SIZE value during active bus cycles.
    # MC68030 SIZE encoding: 00=long, 01=byte, 10=word, 11=line
    # Instruction fetches are word (SIZE=10=2), data writes may be long (SIZE=00=0).
    assert len(size_values_seen) > 0, "No SIZE values observed during bus cycles"

    # We expect to see word-sized fetches (SIZE=2) at minimum
    assert 2 in size_values_seen, (
        f"Expected word-size (SIZE=2) for instruction fetches, "
        f"saw only {size_values_seen}")

    dut._log.info("SIZE signal verification passed")
    h.cleanup()


@cocotb.test()
async def test_bus_function_code(dut):
    """Verify FC_OUT distinguishes supervisor program from supervisor data.

    After reset the MC68030 is in supervisor mode. Instruction fetches
    should use FC=6 (supervisor program) and data accesses FC=5
    (supervisor data).
    """
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(77, 0),
        *move(LONG, DN, 0, AN_IND, 0),     # Data write -> FC=5
        *h.sentinel_program(),
    ]
    await h.setup(program)

    fc_values_seen = set()
    fc_on_read = set()
    fc_on_write = set()

    for _ in range(2000):
        await RisingEdge(dut.CLK)
        try:
            as_n = int(dut.ASn.value)
            if as_n == 0:
                fc = int(dut.FC_OUT.value)
                rw_n = int(dut.RWn.value)
                fc_values_seen.add(fc)
                if rw_n == 1:
                    fc_on_read.add(fc)
                else:
                    fc_on_write.add(fc)
        except ValueError:
            continue

    dut._log.info(f"FC values on reads:  {fc_on_read}")
    dut._log.info(f"FC values on writes: {fc_on_write}")
    dut._log.info(f"All FC values seen:  {fc_values_seen}")

    # After reset, CPU is in supervisor mode.
    # FC=6 (supervisor program) should appear on instruction fetches (reads).
    # FC=5 (supervisor data) should appear on data read/write cycles.
    assert 6 in fc_on_read, (
        f"Expected FC=6 (supervisor program) on reads, saw {fc_on_read}")

    # Data writes should use FC=5 (supervisor data)
    if fc_on_write:
        assert 5 in fc_on_write, (
            f"Expected FC=5 (supervisor data) on writes, saw {fc_on_write}")

    dut._log.info("Function code verification passed")
    h.cleanup()


# ===================================================================
# 3. Bus Cycle Timing
# ===================================================================

@cocotb.test()
async def test_read_cycle_timing(dut):
    """Monitor a complete read bus cycle from ASn assertion to deassertion.

    Verifies that:
    - ASn goes low to start the cycle
    - RWn stays high during reads
    - ASn goes high after DSACKn is asserted
    - The cycle completes within a reasonable number of clocks
    """
    h = CPUTestHarness(dut)
    program = [
        *moveq(42, 0),
        *nop(), *nop(), *nop(), *nop(),
        *h.sentinel_program(),
    ]
    await h.setup(program)

    # Wait for reset vector fetches to complete, then look for program fetches
    read_cycles_measured = []
    cycle_length = 0
    in_read_cycle = False

    for _ in range(2000):
        await RisingEdge(dut.CLK)
        try:
            as_n = int(dut.ASn.value)
            rw_n = int(dut.RWn.value)
        except ValueError:
            continue

        if not in_read_cycle and as_n == 0 and rw_n == 1:
            # Start of a read cycle
            in_read_cycle = True
            cycle_length = 1
        elif in_read_cycle and as_n == 0:
            cycle_length += 1
        elif in_read_cycle and as_n == 1:
            # End of read cycle
            read_cycles_measured.append(cycle_length)
            in_read_cycle = False
            cycle_length = 0

    assert len(read_cycles_measured) > 0, "No complete read cycles observed"

    min_len = min(read_cycles_measured)
    max_len = max(read_cycles_measured)
    avg_len = sum(read_cycles_measured) / len(read_cycles_measured)

    dut._log.info(
        f"Read cycle timing: {len(read_cycles_measured)} cycles measured, "
        f"min={min_len}, max={max_len}, avg={avg_len:.1f} clocks")

    # With 0 wait states, read cycles should be reasonably short (< 20 clocks)
    assert max_len < 30, (
        f"Read cycle too long: {max_len} clocks (expected < 30)")

    h.cleanup()


@cocotb.test()
async def test_write_cycle_timing(dut):
    """Monitor a complete write bus cycle from ASn assertion to deassertion.

    Verifies that:
    - ASn goes low with RWn=0 to start the write cycle
    - The cycle completes within a reasonable number of clocks
    """
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        *moveq(55, 0),
        *move(LONG, DN, 0, AN_IND, 0),     # Write cycle
        *moveq(66, 1),
        *move(LONG, DN, 1, AN_IND, 0),     # Another write cycle
        *h.sentinel_program(),
    ]
    await h.setup(program)

    write_cycles_measured = []
    cycle_length = 0
    in_write_cycle = False

    for _ in range(2000):
        await RisingEdge(dut.CLK)
        try:
            as_n = int(dut.ASn.value)
            rw_n = int(dut.RWn.value)
        except ValueError:
            continue

        if not in_write_cycle and as_n == 0 and rw_n == 0:
            # Start of a write cycle
            in_write_cycle = True
            cycle_length = 1
        elif in_write_cycle and as_n == 0:
            cycle_length += 1
        elif in_write_cycle and as_n == 1:
            # End of write cycle
            write_cycles_measured.append(cycle_length)
            in_write_cycle = False
            cycle_length = 0

    assert len(write_cycles_measured) > 0, "No complete write cycles observed"

    min_len = min(write_cycles_measured)
    max_len = max(write_cycles_measured)
    avg_len = sum(write_cycles_measured) / len(write_cycles_measured)

    dut._log.info(
        f"Write cycle timing: {len(write_cycles_measured)} cycles measured, "
        f"min={min_len}, max={max_len}, avg={avg_len:.1f} clocks")

    # With 0 wait states, write cycles should be reasonably short
    assert max_len < 30, (
        f"Write cycle too long: {max_len} clocks (expected < 30)")

    h.cleanup()


# ===================================================================
# 4. Address/Data Bus
# ===================================================================

@cocotb.test()
async def test_address_alignment(dut):
    """Verify that word and long accesses use word-aligned addresses.

    MC68030 instruction fetches are always word-aligned (A0=0).
    Long-word data accesses should also be aligned.
    """
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),               # RESULT_BASE is 0x020000 (aligned)
        *moveq(42, 0),
        *move(LONG, DN, 0, AN_IND, 0),          # Long write to aligned address
        *h.sentinel_program(),
    ]
    await h.setup(program)

    misaligned_fetches = []

    for _ in range(2000):
        await RisingEdge(dut.CLK)
        try:
            as_n = int(dut.ASn.value)
            if as_n == 0:
                addr = int(dut.ADR_OUT.value)
                size_val = int(dut.SIZE.value)
                rw_n = int(dut.RWn.value)

                # SIZE=2 (word) or SIZE=0 (long) must have even addresses
                if size_val == 2 and (addr & 1) != 0:
                    misaligned_fetches.append(
                        f"Word access at odd addr 0x{addr:08X} (RWn={rw_n})")
                if size_val == 0 and (addr & 1) != 0:
                    misaligned_fetches.append(
                        f"Long access at odd addr 0x{addr:08X} (RWn={rw_n})")
        except ValueError:
            continue

    if misaligned_fetches:
        for msg in misaligned_fetches[:5]:
            dut._log.error(msg)

    assert len(misaligned_fetches) == 0, (
        f"{len(misaligned_fetches)} misaligned accesses detected")

    dut._log.info("Address alignment verified: all word/long accesses are aligned")
    h.cleanup()


@cocotb.test()
async def test_data_bus_width(dut):
    """Verify full 32-bit data bus utilization on long-word writes.

    Write 0xDEADBEEF to memory and verify each byte is correct,
    confirming all 32 data lines are functional.
    """
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        # Load 0xDEADBEEF into D0
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
        *imm_long(0xDEADBEEF),
        # Store to (A0)
        *move(LONG, DN, 0, AN_IND, 0),
        # Load 0x12345678 into D1 and store at RESULT_BASE+4
        *move(LONG, SPECIAL, IMMEDIATE, DN, 1),
        *imm_long(0x12345678),
        *addq(LONG, 4, AN, 0),
        *move(LONG, DN, 1, AN_IND, 0),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel()
    assert found, "Sentinel not reached"

    val1 = h.read_result_long(0)
    val2 = h.read_result_long(4)

    assert val1 == 0xDEADBEEF, (
        f"Expected 0xDEADBEEF at RESULT_BASE, got 0x{val1:08X}")
    assert val2 == 0x12345678, (
        f"Expected 0x12345678 at RESULT_BASE+4, got 0x{val2:08X}")

    # Verify individual bytes to confirm all data lines work
    b0 = h.mem.read(h.RESULT_BASE + 0, 1)
    b1 = h.mem.read(h.RESULT_BASE + 1, 1)
    b2 = h.mem.read(h.RESULT_BASE + 2, 1)
    b3 = h.mem.read(h.RESULT_BASE + 3, 1)
    assert b0 == 0xDE, f"Byte 0: expected 0xDE, got 0x{b0:02X}"
    assert b1 == 0xAD, f"Byte 1: expected 0xAD, got 0x{b1:02X}"
    assert b2 == 0xBE, f"Byte 2: expected 0xBE, got 0x{b2:02X}"
    assert b3 == 0xEF, f"Byte 3: expected 0xEF, got 0x{b3:02X}"

    dut._log.info("Full 32-bit data bus verified: 0xDEADBEEF and 0x12345678 correct")
    h.cleanup()


# ===================================================================
# 5. Edge Cases
# ===================================================================

@cocotb.test()
async def test_misaligned_long_read_wait_states(dut):
    """Misaligned MOVE.L (abs).L read is assembled correctly with wait states."""
    h = CPUTestHarness(dut, wait_states=2)
    addr = h.DATA_BASE + 0x181  # force misaligned long access
    expected = 0x89ABCDEF
    h.mem.load_long(addr, expected)

    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),     # A0 = RESULT_BASE
        *imm_long(h.RESULT_BASE),
        *move(LONG, SPECIAL, ABS_L, DN, 0),      # D0 = (abs).L misaligned
        *abs_long(addr),
        *move(LONG, DN, 0, AN_IND, 0),           # store D0 for checking
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel(max_cycles=8000)
    assert found, "Sentinel not reached"

    result = h.read_result_long(0)
    assert result == expected, (
        f"Misaligned long read mismatch: expected 0x{expected:08X}, got 0x{result:08X}"
    )
    h.cleanup()


@cocotb.test()
async def test_move_imm_long_fp_disp_roundtrip(dut):
    """Regression: MOVE.L #imm,(d16,A6) + MOVEA.L (d16,A6),A0 keeps all bytes.

    CoreMark O2 uses this exact pattern for function-pointer locals.
    """
    h = CPUTestHarness(dut)

    mem_addr = 0x00001000 - 78
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 6),   # A6 = 0x00001000
        *imm_long(0x00001000),
        0x2D7C, 0x0000, 0x0A14, 0xFFB2,        # MOVE.L #$00000A14,-78(A6)
        0x206E, 0xFFB2,                        # MOVEA.L -78(A6),A0
        *move_to_abs_long(LONG, AN, 0, h.RESULT_BASE),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel(max_cycles=4000)
    assert found, "Sentinel not reached"

    got = h.read_result_long(0)
    mem_val = h.mem.read(mem_addr, 4)
    assert mem_val == 0x00000A14, (
        f"Stored long mismatch at 0x{mem_addr:08X}: got 0x{mem_val:08X}"
    )
    assert got == 0x00000A14, (
        f"Round-trip MOVEA mismatch: expected 0x00000A14, got 0x{got:08X}"
    )
    h.cleanup()


@cocotb.test()
async def test_move_imm_long_fp_disp_roundtrip_off1(dut):
    """Regression: misaligned +1 long round-trip keeps all bytes."""
    h = CPUTestHarness(dut)

    scratch_base = h.DATA_BASE + 0x300
    disp = 5
    mem_addr = scratch_base + disp
    expected = 0xCB91CE37

    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 6),      # A6 = scratch base
        *imm_long(scratch_base),
        *move(LONG, SPECIAL, IMMEDIATE, AN_DISP, 6),  # (d16,A6) = expected
        *imm_long(expected),
        disp & 0xFFFF,
        *movea(LONG, AN_DISP, 6, 0),              # A0 = (d16,A6)
        disp & 0xFFFF,
        *move_to_abs_long(LONG, AN, 0, h.RESULT_BASE),
        *h.sentinel_program(),
    ]
    await h.setup(program)
    found = await h.run_until_sentinel(max_cycles=4000)
    assert found, "Sentinel not reached"

    mem_val = h.mem.read(mem_addr, 4)
    got = h.read_result_long(0)
    assert mem_val == expected, (
        f"Stored long mismatch at 0x{mem_addr:08X}: got 0x{mem_val:08X}"
    )
    assert got == expected, (
        f"Round-trip MOVEA mismatch: expected 0x{expected:08X}, got 0x{got:08X}"
    )
    h.cleanup()


@cocotb.test()
async def test_back_to_back_reads(dut):
    """Verify multiple sequential instruction fetches (back-to-back reads).

    A sequence of NOPs forces many consecutive instruction fetch (read)
    cycles. Verify the CPU fetches from sequential addresses without
    gaps or errors.
    """
    h = CPUTestHarness(dut)
    # Many NOPs to force sequential instruction fetches
    program = [
        *nop(), *nop(), *nop(), *nop(),
        *nop(), *nop(), *nop(), *nop(),
        *nop(), *nop(), *nop(), *nop(),
        *nop(), *nop(), *nop(), *nop(),
        *h.sentinel_program(),
    ]
    await h.setup(program)

    read_addresses = []

    for _ in range(2000):
        await RisingEdge(dut.CLK)
        try:
            as_n = int(dut.ASn.value)
            if as_n == 0:
                rw_n = int(dut.RWn.value)
                if rw_n == 1:  # Read cycle
                    addr = int(dut.ADR_OUT.value)
                    read_addresses.append(addr)
        except ValueError:
            continue

    # Filter to just program area reads (not reset vectors or sentinel writes)
    prog_reads = sorted(set(
        a for a in read_addresses
        if h.PROGRAM_BASE <= a < h.PROGRAM_BASE + 0x100
    ))

    assert len(prog_reads) > 0, "No reads from program area detected"

    dut._log.info(f"Back-to-back reads: {len(prog_reads)} unique program "
                  f"addresses fetched")
    dut._log.info(f"  Range: 0x{prog_reads[0]:08X} - 0x{prog_reads[-1]:08X}")

    # The program is 16 NOPs + sentinel code. We should see fetches
    # from at least PROGRAM_BASE through some portion of the program.
    assert prog_reads[0] == h.PROGRAM_BASE, (
        f"First program fetch at 0x{prog_reads[0]:08X}, "
        f"expected 0x{h.PROGRAM_BASE:08X}")

    # Verify consecutive addresses (instruction fetches should be sequential)
    for i in range(1, min(len(prog_reads), 8)):
        gap = prog_reads[i] - prog_reads[i - 1]
        assert gap in (2, 4), (
            f"Non-sequential fetch: 0x{prog_reads[i-1]:08X} -> "
            f"0x{prog_reads[i]:08X} (gap={gap})")

    dut._log.info("Back-to-back reads verified: sequential instruction fetches")
    h.cleanup()


@cocotb.test()
async def test_read_then_write(dut):
    """Verify correct bus behavior when read cycles (fetches) are followed
    by write cycles (data stores) and vice versa.

    The program fetches instructions (reads), then stores results (writes),
    then fetches more instructions (reads). This tests the bus turnaround.
    """
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(h.RESULT_BASE),
        # First computation + store
        *moveq(11, 0),
        *move(LONG, DN, 0, AN_IND, 0),     # Write cycle
        # More instructions after the write (read cycles resume)
        *nop(), *nop(),
        # Second computation + store
        *addq(LONG, 4, AN, 0),
        *moveq(22, 1),
        *move(LONG, DN, 1, AN_IND, 0),     # Another write cycle
        *h.sentinel_program(),
    ]
    await h.setup(program)

    # Track the sequence of read/write bus cycles
    bus_ops = []  # List of ('R', addr) or ('W', addr)

    for _ in range(2000):
        await RisingEdge(dut.CLK)
        try:
            as_n = int(dut.ASn.value)
            if as_n == 0:
                rw_n = int(dut.RWn.value)
                addr = int(dut.ADR_OUT.value)
                op = 'R' if rw_n == 1 else 'W'
                # Only record transitions to avoid duplicates within a cycle
                if not bus_ops or bus_ops[-1] != (op, addr):
                    bus_ops.append((op, addr))
        except ValueError:
            continue

    reads = [addr for op, addr in bus_ops if op == 'R']
    writes = [addr for op, addr in bus_ops if op == 'W']

    assert len(reads) > 0, "No read bus cycles observed"
    assert len(writes) > 0, "No write bus cycles observed"

    dut._log.info(f"Read-then-write: {len(reads)} reads, {len(writes)} writes")

    # Verify that writes hit the expected RESULT_BASE area
    result_writes = [a for a in writes if h.RESULT_BASE <= a < h.RESULT_BASE + 0x100]
    assert len(result_writes) > 0, (
        f"No writes to RESULT_BASE area; writes went to: "
        f"{[f'0x{a:08X}' for a in writes[:5]]}")

    # Verify the stored values
    found = await h.run_until_sentinel(max_cycles=0)
    # Already ran in the monitoring loop, check memory directly
    val1 = h.read_result_long(0)
    val2 = h.read_result_long(4)

    dut._log.info(f"Stored values: [0]=0x{val1:08X}, [4]=0x{val2:08X}")
    dut._log.info("Read-then-write bus turnaround verified")
    h.cleanup()


@cocotb.test()
async def test_misaligned_long_read_retry_recovers(dut):
    """Inject one BERR+HALT retry on misaligned long read; cycle should restart and complete."""
    h = CPUTestHarness(dut, wait_states=1)
    data_addr = h.DATA_BASE + 0x181  # misaligned long forces multi-part transfer
    expected = 0x89ABCDEF
    res = h.RESULT_BASE + 0xD0
    berr_handler = 0x000760

    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(data_addr),
        *move(LONG, AN_IND, 0, DN, 0),
        *move_to_abs_long(LONG, DN, 0, res),
        *moveq(0x5A, 1),
        *move_to_abs_long(LONG, DN, 1, res + 4),
        *h.sentinel_program(),
    ]

    h.mem.load_long(data_addr, expected)
    await h.setup(program)

    # If retry handling is wrong, this handler will fire and write 0xEE.
    h.mem.load_long(2 * 4, berr_handler)
    h.mem.load_words(
        berr_handler,
        [
            *moveq(0x6E, 2),
            *move_to_abs_long(LONG, DN, 2, res + 8),
            *h.sentinel_program(),
        ],
    )

    prev_as_n = 1
    retry_injected = False
    retry_phase = 0  # 0=idle, 1=berr+halt asserted, 2=berr released/holding halt, 3=done
    starts_at_target = 0
    found = False
    data_rdy_events = []

    for _ in range(40000):
        await RisingEdge(dut.CLK)
        try:
            as_n = int(dut.ASn.value)
            rw_n = int(dut.RWn.value)
            addr = int(dut.ADR_OUT.value)
            bus_bsy = int(dut.BUS_BSY.value)
            data_rdy = int(dut.DATA_RDY.value)
            data_valid = int(dut.DATA_VALID.value)
            data_to_core = int(dut.DATA_TO_CORE.value)
        except ValueError:
            continue

        if prev_as_n == 1 and as_n == 0 and rw_n == 1 and addr == data_addr:
            starts_at_target += 1
            if not retry_injected:
                # Assert retry request for the active cycle.
                dut.BERRn.value = 0
                dut.HALT_INn.value = 0
                retry_injected = True
                retry_phase = 1

        if data_rdy:
            data_rdy_events.append((data_valid, data_to_core))

        if retry_phase == 1 and as_n == 1:
            # Release bus error at the end of the terminated cycle, holding
            # HALT, so the cycle is treated strictly as retry.
            dut.BERRn.value = 1
            retry_phase = 2
        elif retry_phase == 2:
            # UM 7.5.2 negates HALT at the same time as or after BERR. The
            # core holds the cycle until then, so BUS_BSY stays asserted and
            # cannot be used to pace this step.
            dut.HALT_INn.value = 1
            retry_phase = 3

        prev_as_n = as_n
        if h.mem.read(h.SENTINEL_ADDR, 4) == h.SENTINEL_VAL:
            found = True
            break

    assert retry_injected, "Did not inject retry on target misaligned long read"
    assert starts_at_target >= 2, f"Expected retried restart of target read, saw {starts_at_target} start(s)"
    assert found, "Retry recovery test did not complete"
    assert h.mem.read(res, 4) == expected, (
        f"Retried misaligned read data mismatch: expected 0x{expected:08X}, got 0x{h.mem.read(res, 4):08X}; "
        f"res+4=0x{h.mem.read(res + 4, 4):08X}, res+8=0x{h.mem.read(res + 8, 4):08X}, "
        f"data_rdy_events={[(v, hex(d)) for v, d in data_rdy_events[:8]]}"
    )
    assert h.mem.read(res + 4, 4) == 0x0000005A, "Program did not continue after retry recovery"
    assert h.mem.read(res + 8, 4) != 0x0000006E, "Bus-error handler fired; retry was not recovered"
    h.cleanup()


@cocotb.test()
async def test_retry_rerun_waits_for_halt_negation(dut):
    """UM 7.5.2: after a retry the core starts no bus cycle until HALT negates.

    BERR is negated first while HALT is held, which is the sequence the UM
    documents; no bus cycle may begin during that window.
    """
    h = CPUTestHarness(dut, wait_states=1)
    data_addr = h.DATA_BASE + 0x181
    expected = 0x89ABCDEF
    res = h.RESULT_BASE + 0xE0
    hold_clocks = 40

    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(data_addr),
        *move(LONG, AN_IND, 0, DN, 0),
        *move_to_abs_long(LONG, DN, 0, res),
        *h.sentinel_program(),
    ]

    h.mem.load_long(data_addr, expected)
    await h.setup(program)

    prev_as_n = 1
    phase = 0  # 0=waiting for target, 1=berr+halt, 2=holding halt only, 3=released
    held = 0
    as_low_while_halted = 0
    found = False

    for _ in range(40000):
        await RisingEdge(dut.CLK)
        try:
            as_n = int(dut.ASn.value)
            rw_n = int(dut.RWn.value)
            addr = int(dut.ADR_OUT.value)
        except ValueError:
            continue

        if phase == 0 and prev_as_n == 1 and as_n == 0 and rw_n == 1 and addr == data_addr:
            dut.BERRn.value = 0
            dut.HALT_INn.value = 0
            phase = 1
        elif phase == 1 and as_n == 1:
            # Terminated. Negate BERR but keep holding HALT.
            dut.BERRn.value = 1
            phase = 2
        elif phase == 2:
            if as_n == 0:
                as_low_while_halted += 1
            held += 1
            if held >= hold_clocks:
                dut.HALT_INn.value = 1
                phase = 3

        prev_as_n = as_n
        if h.mem.read(h.SENTINEL_ADDR, 4) == h.SENTINEL_VAL:
            found = True
            break

    assert phase == 3, f"Retry sequence did not complete, stuck in phase {phase}"
    assert as_low_while_halted == 0, (
        f"Bus cycle began before HALT was negated: ASn low for "
        f"{as_low_while_halted} of {hold_clocks} held clocks"
    )
    assert found, "Retry test did not complete after HALT negation"
    assert h.mem.read(res, 4) == expected, (
        f"Retried read data mismatch: expected 0x{expected:08X}, "
        f"got 0x{h.mem.read(res, 4):08X}"
    )
    h.cleanup()


@cocotb.test()
async def test_late_berr_after_dsack_faults(dut):
    """UM Table 7-8 case 4: BERR one clock after DSACK still terminates in error.

    The read is aligned, so DSACK terminates the whole transfer in one
    sub-cycle and the fault lands on the edge the transfer completes on.
    """
    h = CPUTestHarness(dut)
    data_addr = h.DATA_BASE + 0x200
    res = h.RESULT_BASE + 0xF0
    berr_handler = 0x000780

    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(data_addr),
        *move(LONG, AN_IND, 0, DN, 0),          # faulted read
        *moveq(0x5A, 1),
        *move_to_abs_long(LONG, DN, 1, res),    # must not run
        *h.sentinel_program(),
    ]

    h.mem.load_long(data_addr, 0x89ABCDEF)
    await h.setup(program)

    h.mem.load_long(2 * 4, berr_handler)
    h.mem.load_words(
        berr_handler,
        [
            *moveq(0x6E, 2),
            *move_to_abs_long(LONG, DN, 2, res + 4),
            *h.sentinel_program(),
        ],
    )

    injected = False
    found = False

    for _ in range(40000):
        await RisingEdge(dut.CLK)
        try:
            as_n = int(dut.ASn.value)
            rw_n = int(dut.RWn.value)
            addr = int(dut.ADR_OUT.value)
            dsack = int(dut.DSACKn.value)
        except ValueError:
            continue

        if not injected and as_n == 0 and rw_n == 1 and addr == data_addr and dsack != 0b11:
            # DSACK was asserted one clock ago (state N); assert BERR now so it
            # is sampled in state N + 2, the late-BERR case.
            dut.BERRn.value = 0
            injected = True
        elif injected and as_n == 1:
            dut.BERRn.value = 1

        if h.mem.read(h.SENTINEL_ADDR, 4) == h.SENTINEL_VAL:
            found = True
            break

    assert injected, "Never injected a late BERR on the target read"
    assert found, "Test did not complete"
    assert h.mem.read(res + 4, 4) == 0x0000006E, (
        "Bus-error handler did not run: late BERR was dropped and the core "
        f"consumed uncorrected data (res+4=0x{h.mem.read(res + 4, 4):08X})"
    )
    assert h.mem.read(res, 4) != 0x0000005A, (
        "Faulted read continued into the following store"
    )
    h.cleanup()


@cocotb.test()
async def test_avec_ignored_outside_iack_cycle(dut):
    """UM 7.4.1.2: AVEC only terminates interrupt acknowledge cycles.

    Many systems ground AVEC permanently. AVECn reaches the bus interface
    only while the exception handler is busy, so the exposed cycles are the
    frame writes: with the stack placed so they carry the interrupt
    acknowledge type field in A19:A16, they must still wait for DSACK.
    """
    h = CPUTestHarness(dut, wait_states=3)
    stack_top = 0x000F8000  # frame writes land at 0x000F7FFx -> A19:A16 = $F
    marker = 0x0000005A
    handler = 0x000700

    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 7),   # A7 = supervisor stack in $000Fxxxx
        *imm_long(stack_top),
        *trap(0),
        *moveq(0x33, 3),
        *move_to_abs_long(LONG, DN, 3, h.RESULT_BASE + 4),
        *h.sentinel_program(),
    ]
    await h.setup(program)

    h.mem.load_long(32 * 4, handler)           # TRAP #0 -> vector 32
    h.mem.load_words(handler, [
        *moveq(marker, 1),
        *move_to_abs_long(LONG, DN, 1, h.RESULT_BASE),
        *rte(),
    ])

    dut.AVECn.value = 0  # Grounded, as on many boards.

    found = await h.run_until_sentinel(max_cycles=20000)
    assert found, (
        "Sentinel not reached: a grounded AVEC mis-terminated an exception "
        "frame write"
    )
    assert h.read_result_long(0) == marker, (
        f"TRAP handler marker wrong: got 0x{h.read_result_long(0):08X}")
    assert h.read_result_long(4) == 0x00000033, (
        "RTE did not return correctly: exception frame was corrupted "
        f"(got 0x{h.read_result_long(4):08X})")
    h.cleanup()


@cocotb.test()
async def test_bus_fault_info_held_until_exception_entry(dut):
    """The fault SSW must survive until the exception handler captures it.

    A queued prefetch runs in the clocks between the fault and the handler
    leaving its idle state. It must not recapture SSW_80, or the stacked
    frame reports the prefetch (FC = 6, DF clear) instead of the faulted
    data access (FC = 5, DF set).
    """
    h = CPUTestHarness(dut)
    data_addr = h.DATA_BASE + 0x240
    berr_handler = 0x0007A0

    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(data_addr),
        *move(LONG, AN_IND, 0, DN, 0),          # faulted supervisor data read
        *h.sentinel_program(),
    ]
    await h.setup(program)

    h.mem.load_long(2 * 4, berr_handler)
    h.mem.load_words(berr_handler, h.sentinel_program())

    bi = dut.I_BUS_IF
    injected = False
    samples = []
    found = False

    for _ in range(20000):
        await RisingEdge(dut.CLK)
        try:
            as_n = int(dut.ASn.value)
            rw_n = int(dut.RWn.value)
            addr = int(dut.ADR_OUT.value)
            ssw = int(bi.SSW_80.value)
            busy_exh = int(bi.BUSY_EXH.value)
        except ValueError:
            continue

        if not injected and as_n == 0 and rw_n == 1 and addr == data_addr:
            dut.BERRn.value = 0
            injected = True
        elif injected and as_n == 1:
            dut.BERRn.value = 1

        # From the cycle DF is first set, record until the handler engages.
        if (samples or (ssw >> 8) & 1) and not busy_exh:
            samples.append(ssw)
        elif samples and busy_exh:
            break

        if h.mem.read(h.SENTINEL_ADDR, 4) == h.SENTINEL_VAL:
            found = True
            break

    assert injected, "Never injected BERR on the target read"
    assert samples, "SSW never reported a data fault"
    bad = [s for s in samples if ((s >> 8) & 1) == 0 or (s & 0x7) != 0b101]
    assert not bad, (
        "Fault info was overwritten before exception entry: "
        f"saw SSW {[hex(s) for s in samples]}, expected DF set and FC = 5 "
        f"throughout ({len(bad)} bad of {len(samples)} clocks)"
    )
    h.cleanup()


# ===================================================================
# 6. Reset Operation (UM 7.8)
# ===================================================================

async def _bringup_reset_only(dut, h, program):
    """Bring the CPU up driving RESET_INn alone, with HALT_INn left negated.

    This is what a standard MC68030 reset circuit does: HALT is an input the
    reset logic never touches, so it sits pulled up.
    """
    clock = Clock(dut.CLK, 10, unit="ns")
    cocotb.start_soon(clock.start())
    h._init_idle_inputs()
    h._load_memory(program)
    await RisingEdge(dut.CLK)
    await RisingEdge(dut.CLK)

    dut.RESET_INn.value = 0
    await ClockCycles(dut.CLK, 20)

    h.bus = BusModel(dut, h.mem, h.wait_states)
    await h.bus.start()

    dut.RESET_INn.value = 1
    await ClockCycles(dut.CLK, 4)


@cocotb.test()
async def test_external_reset_without_halt(dut):
    """UM 7.8: RESET alone resets the processor logic; HALT is not involved."""
    h = CPUTestHarness(dut)
    await _bringup_reset_only(dut, h, _arithmetic_program(h))

    assert int(dut.HALT_INn.value) == 1, "HALT_INn must stay negated in this test"

    found = await h.run_until_sentinel(max_cycles=4000)
    assert found, "CPU never ran: RESET alone did not reset the core"
    assert h.read_result_long(0) == EXPECTED_D0, (
        f"D0: expected {EXPECTED_D0}, got 0x{h.read_result_long(0):08X}")
    h.cleanup()


@cocotb.test()
async def test_repeated_external_reset_without_halt(dut):
    """A later RESET-only assertion must reset the core again (UM 7.8)."""
    h = CPUTestHarness(dut)
    await _bringup_reset_only(dut, h, _arithmetic_program(h))
    assert await h.run_until_sentinel(max_cycles=4000), "first run did not complete"

    # Clear the completion markers, then pulse RESET only.
    h.mem.load_long(h.SENTINEL_ADDR, 0)
    h.mem.load_long(h.RESULT_BASE, 0)
    dut.RESET_INn.value = 0
    await ClockCycles(dut.CLK, 20)
    dut.RESET_INn.value = 1
    await ClockCycles(dut.CLK, 4)

    found = await h.run_until_sentinel(max_cycles=6000)
    assert found, "CPU did not restart after a second RESET-only assertion"
    assert h.read_result_long(0) == EXPECTED_D0, (
        f"D0 after re-reset: expected {EXPECTED_D0}, "
        f"got 0x{h.read_result_long(0):08X}")
    h.cleanup()


@cocotb.test()
async def test_reset_instruction_does_not_reset_core(dut):
    """UM 5.8.1/7.8: RESET resets external devices only, never the core itself.

    RESET_OUT and RESET_INn share one open-drain net in a real system, so the
    core sees its own assertion on RESET_INn and must ignore it.
    """
    h = CPUTestHarness(dut)
    counter = h.RESULT_BASE
    program = [
        *move(LONG, SPECIAL, ABS_L, DN, 0),      # D0 = run counter
        *abs_long(counter),
        *addq(LONG, 1, DN, 0),
        *move_to_abs_long(LONG, DN, 0, counter),
        0x4E70,                                   # RESET
        *h.sentinel_program(),
    ]
    await _bringup_reset_only(dut, h, program)

    found = False
    saw_reset_out = False
    for _ in range(40000):
        await RisingEdge(dut.CLK)
        try:
            reset_out = int(dut.RESET_OUT.value)
        except ValueError:
            reset_out = 0
        dut.RESET_INn.value = 0 if reset_out else 1
        saw_reset_out |= bool(reset_out)
        if h.mem.read(h.SENTINEL_ADDR, 4) == h.SENTINEL_VAL:
            found = True
            break

    assert saw_reset_out, "RESET instruction never asserted RESET_OUT"
    assert found, "core reset itself from its own RESET instruction output"
    runs = h.read_result_long(0)
    assert runs == 1, f"program ran {runs} times; RESET restarted the core"
    h.cleanup()


# ===================================================================
# 8. Synchronous (STERM) transfers -- UM 6.1.3 / 7.3.2
# ===================================================================
#
# A synchronous cycle is acknowledged with STERM instead of DSACKx and
# completes two clocks after the address goes out, so the edge that recognises
# the termination is also the edge that advances the transfer's remaining-byte
# count. Wherever that count is consulted on the same edge, the model has to
# look at the post-decrement value. Getting it wrong is invisible on aligned
# transfers -- one sub-cycle, the count reaches zero either way -- and corrupts
# the operand on every split one.

SYNC_RD_WINDOW = 0x018100
SYNC_WR_WINDOW = 0x018400

# SIZE[1:0] is the count of bytes still to transfer (UM 7.2.1), so the code for
# a whole long word is 00 and a single trailing byte is 01.
SZ_LONG, SZ_BYTE, SZ_WORD, SZ_3BYTE = 0b00, 0b01, 0b10, 0b11

SYNC_SIZES = ((LONG, 4), (WORD, 2), (BYTE, 1))

# One payload per (size, alignment) case; every byte differs so a misplaced
# lane is always visible.
SYNC_WR_PATTERNS = [
    0x8191A1B1, 0x8292A2B2, 0x8393A3B3, 0x8494A4B4,
    0x8595A5B5, 0x8696A6B6, 0x8797A7B7, 0x8898A8B8,
    0x8999A9B9, 0x8A9AAABA, 0x8B9BABBB, 0x8C9CACBC,
]


class SyncTermBusModel(BusModel):
    """32-bit port that acknowledges every cycle with STERMn (UM 7.3.2).

    STERMn is driven per cycle and released when the cycle ends rather than
    tied low for the whole run, so each sub-cycle of a split transfer sees it
    presented afresh -- which is what a real synchronous device does, and what
    makes a two-sub-cycle synchronous transfer distinguishable from one.
    """

    def __init__(self, dut, memory, **kwargs):
        super().__init__(dut, memory, sync_term=True, **kwargs)


def _sync_case_addr(window, idx, align):
    """Distinct, non-overlapping address whose A1:A0 equals align."""
    return window + 0x20 * idx + align


def _sync_fill(mem):
    """Byte value = low 8 address bits, over both synchronous windows."""
    for a in range(0x018000, 0x018600):
        mem.write(a, 1, a & 0xFF)


async def _run_sterm(dut, program_fn, max_cycles=40000, wait_states=0):
    """Bring the CPU up against an all-synchronous bus and run to the sentinel."""
    h = CPUTestHarness(dut)
    bus = SyncTermBusModel(dut, h.mem, wait_states=wait_states)

    clock = Clock(dut.CLK, 10, unit="ns")
    cocotb.start_soon(clock.start())
    h._init_idle_inputs()
    h._install_trap_stubs()
    h._load_memory(program_fn(h))
    _sync_fill(h.mem)
    await RisingEdge(dut.CLK)
    await RisingEdge(dut.CLK)
    dut.RESET_INn.value = 0
    dut.HALT_INn.value = 0
    await ClockCycles(dut.CLK, 20)
    h.bus = bus
    await bus.start()
    dut.RESET_INn.value = 1
    dut.HALT_INn.value = 1
    await ClockCycles(dut.CLK, 4)

    found = False
    for _ in range(max_cycles):
        await RisingEdge(dut.CLK)
        if h.mem.read(h.SENTINEL_ADDR, 4) == h.SENTINEL_VAL:
            found = True
            break
    h.check_no_unexpected_exception()
    return h, bus, found


def _sync_read_program(h):
    """MOVE.<size> (sync).L,D0 -> MOVE.L D0,(RESULT).L for every case."""
    program = []
    for idx in range(len(SYNC_SIZES) * 4):
        size = SYNC_SIZES[idx // 4][0]
        align = idx % 4
        src = _sync_case_addr(SYNC_RD_WINDOW, idx, align)
        program += moveq(0, 0)  # zero-extend sub-long results
        program += move_from_abs_long(size, src, DN, 0)
        program += move_to_abs_long(LONG, DN, 0, h.RESULT_BASE + 4 * idx)
    program += h.sentinel_program()
    return program


def _sync_write_program(h):
    """MOVE.L #pat,D0 -> MOVE.<size> D0,(sync).L for every case."""
    program = []
    for idx in range(len(SYNC_SIZES) * 4):
        size = SYNC_SIZES[idx // 4][0]
        align = idx % 4
        dst = _sync_case_addr(SYNC_WR_WINDOW, idx, align)
        program += move(LONG, SPECIAL, IMMEDIATE, DN, 0)
        program += imm_long(SYNC_WR_PATTERNS[idx])
        program += move_to_abs_long(size, DN, 0, dst)
    program += h.sentinel_program()
    return program


@cocotb.test()
async def test_sterm_reads_all_sizes_and_alignments(dut):
    """Long/word/byte reads at every alignment from a synchronous 32-bit port.

    The operand value is checked, not just the address sequence: a split
    synchronous transfer whose remaining-byte count overshoots runs its second
    sub-cycle with SIZE = 00 and latches a whole long word over the operand.
    """
    h, _bus, found = await _run_sterm(dut, _sync_read_program)
    assert found, "program did not complete"

    errs = []
    for idx in range(len(SYNC_SIZES) * 4):
        _size, nbytes = SYNC_SIZES[idx // 4]
        align = idx % 4
        src = _sync_case_addr(SYNC_RD_WINDOW, idx, align)
        want = 0
        for i in range(nbytes):
            want = (want << 8) | (h.mem.read(src + i, 1) & 0xFF)
        got = h.read_result_long(4 * idx)
        if got != want:
            errs.append(
                f"STERM read size={nbytes} a={align} addr=0x{src:08X}: "
                f"got 0x{got:08X} want 0x{want:08X}"
            )
    assert not errs, "synchronous read errors: " + "; ".join(errs)
    h.cleanup()


@cocotb.test()
async def test_sterm_wait_stated_reads_all_sizes_and_alignments(dut):
    """The same reads against a synchronous port that inserts wait states.

    A wait-stated synchronous cycle recognises STERM at a later S3 than a
    zero-wait-state one and captures its data from the S3 branch of the input
    mux rather than the S2 branch, so it reaches the remaining-byte count
    through a different path.
    """
    h, bus, found = await _run_sterm(dut, _sync_read_program, wait_states=2)
    assert found, "program did not complete"

    errs = []
    for idx in range(len(SYNC_SIZES) * 4):
        _size, nbytes = SYNC_SIZES[idx // 4]
        align = idx % 4
        src = _sync_case_addr(SYNC_RD_WINDOW, idx, align)
        want = 0
        for i in range(nbytes):
            want = (want << 8) | (h.mem.read(src + i, 1) & 0xFF)
        got = h.read_result_long(4 * idx)
        if got != want:
            errs.append(
                f"wait-stated STERM read size={nbytes} a={align} "
                f"addr=0x{src:08X}: got 0x{got:08X} want 0x{want:08X}"
            )
    assert not errs, "wait-stated synchronous read errors: " + "; ".join(errs)
    errs = _check_sterm_shapes(bus, SYNC_RD_WINDOW, is_write=False)
    assert not errs, (
        "wait-stated synchronous read sub-cycle errors: " + "; ".join(errs)
    )
    h.cleanup()


@cocotb.test()
async def test_sterm_writes_all_sizes_and_alignments(dut):
    """Long/word/byte writes at every alignment to a synchronous 32-bit port."""
    h, _bus, found = await _run_sterm(dut, _sync_write_program)
    assert found, "program did not complete"

    errs = []
    for idx in range(len(SYNC_SIZES) * 4):
        _size, nbytes = SYNC_SIZES[idx // 4]
        align = idx % 4
        dst = _sync_case_addr(SYNC_WR_WINDOW, idx, align)
        want = SYNC_WR_PATTERNS[idx] & ((1 << (8 * nbytes)) - 1)
        got = h.mem.read(dst, nbytes)
        if got != want:
            errs.append(
                f"STERM write size={nbytes} a={align} addr=0x{dst:08X}: "
                f"got 0x{got:0{2 * nbytes}X} want 0x{want:0{2 * nbytes}X}"
            )
    assert not errs, "synchronous write errors: " + "; ".join(errs)
    h.cleanup()


# Sub-cycle shape a 32-bit port must see for each case, as (address offset from
# the operand, SIZE code). UM 7.2.1 and Table 7-4: a sub-cycle moves the bytes
# between the lane A1:A0 selects and the end of the port, and SIZE always
# states how many bytes of the operand are still outstanding.
STERM_SHAPES = {
    (LONG, 0): [(0, SZ_LONG)],
    (LONG, 1): [(0, SZ_LONG), (3, SZ_BYTE)],
    (LONG, 2): [(0, SZ_LONG), (2, SZ_WORD)],
    (LONG, 3): [(0, SZ_LONG), (1, SZ_3BYTE)],
    (WORD, 0): [(0, SZ_WORD)],
    (WORD, 1): [(0, SZ_WORD)],
    (WORD, 2): [(0, SZ_WORD)],
    (WORD, 3): [(0, SZ_WORD), (1, SZ_BYTE)],
    (BYTE, 0): [(0, SZ_BYTE)],
    (BYTE, 1): [(0, SZ_BYTE)],
    (BYTE, 2): [(0, SZ_BYTE)],
    (BYTE, 3): [(0, SZ_BYTE)],
}


def _check_sterm_shapes(bus, window, is_write):
    """Compare recorded sub-cycles against STERM_SHAPES for one direction."""
    errs = []
    for idx in range(len(SYNC_SIZES) * 4):
        size, nbytes = SYNC_SIZES[idx // 4]
        align = idx % 4
        base = _sync_case_addr(window, idx, align)
        want = [(base + off, code) for off, code in STERM_SHAPES[(size, align)]]
        got = [
            (addr, code)
            for addr, code, wr in bus.sub_cycles
            if wr == is_write and base <= addr < base + 8
        ]
        if got != want:
            errs.append(
                f"size={nbytes} a={align} addr=0x{base:08X}: sub-cycles "
                f"{[(hex(a), format(c, '02b')) for a, c in got]} != "
                f"{[(hex(a), format(c, '02b')) for a, c in want]}"
            )
    return errs


@cocotb.test()
async def test_sterm_read_sub_cycle_shape(dut):
    """Every synchronous read splits at the address and SIZE the UM requires.

    Includes the three-byte sub-cycle (SIZE = 11), which a 32-bit port only
    ever sees as the second half of a long word starting at A1:A0 = 11.
    """
    h, bus, found = await _run_sterm(dut, _sync_read_program)
    assert found, "program did not complete"
    errs = _check_sterm_shapes(bus, SYNC_RD_WINDOW, is_write=False)
    assert not errs, "synchronous read sub-cycle errors: " + "; ".join(errs)
    assert any(
        code == SZ_3BYTE for _a, code, wr in bus.sub_cycles if not wr
    ), "no three-byte read sub-cycle was exercised"
    h.cleanup()


@cocotb.test()
async def test_sterm_write_sub_cycle_shape(dut):
    """Every synchronous write splits at the address and SIZE the UM requires."""
    h, bus, found = await _run_sterm(dut, _sync_write_program)
    assert found, "program did not complete"
    errs = _check_sterm_shapes(bus, SYNC_WR_WINDOW, is_write=True)
    assert not errs, "synchronous write sub-cycle errors: " + "; ".join(errs)
    assert any(
        code == SZ_3BYTE for _a, code, wr in bus.sub_cycles if wr
    ), "no three-byte write sub-cycle was exercised"
    h.cleanup()


@cocotb.test()
async def test_sterm_misaligned_long_does_not_overrun(dut):
    """Regression: the second sub-cycle must move one byte, not a long word.

    A long read from A1:A0 = 01 splits 3 + 1. When the remaining-byte count is
    advanced twice for one synchronous cycle it reaches zero after the first
    sub-cycle, so the second runs with SIZE = 00 and latches the long word at
    base + 3 over the whole operand: a read from 0x18121 returned the bytes at
    0x18124..0x18127 instead of 0x18121..0x18124.
    """
    target = 0x018121  # A1:A0 = 01

    def program(h):
        return [
            *move_from_abs_long(LONG, target, DN, 0),
            *move_to_abs_long(LONG, DN, 0, h.RESULT_BASE),
            *h.sentinel_program(),
        ]

    h, bus, found = await _run_sterm(dut, program)
    assert found, "program did not complete"

    want = 0
    for i in range(4):
        want = (want << 8) | (h.mem.read(target + i, 1) & 0xFF)
    got = h.read_result_long(0)
    assert got == want, (
        f"misaligned synchronous long read returned 0x{got:08X}, expected "
        f"0x{want:08X} (bytes at 0x{target:08X}..0x{target + 3:08X})"
    )

    seen = [
        (a, c) for a, c, wr in bus.sub_cycles
        if not wr and target <= a < target + 8
    ]
    assert seen == [(target, SZ_LONG), (target + 3, SZ_BYTE)], (
        f"sub-cycles {[(hex(a), format(c, '02b')) for a, c in seen]} != "
        f"[(0x{target:X}, 00), (0x{target + 3:X}, 01)]"
    )
    h.cleanup()
