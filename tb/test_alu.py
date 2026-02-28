"""
Unit test for WF68K30L_ALU module.

Tests arithmetic and logic operations by driving the ALU directly:
  - Load operands via LOAD_OP1/OP2/OP3
  - Assert ALU_INIT with the operation code
  - Wait for ALU_REQ
  - Assert ALU_ACK to read the result
  - Check RESULT and condition codes (STATUS_REG_OUT)

OP codes from wf68k30L_pkg.svh (OP_68K enum):
  ADD  = 7'd1      SUB  = 7'd92     AND_B = 7'd6
  OR_B = 7'd74     MOVEQ = 7'd65    CLR   = 7'd32
  CMP  = 7'd33     NEG  = 7'd70

OP_SIZE encoding:
  LONG = 2'b00 = 0    WORD = 2'b01 = 1    BYTE = 2'b10 = 2

Status register bits [4:0] = X N Z V C

Set TOPLEVEL=WF68K30L_ALU when running this test.
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles

# OP_SIZE
LONG = 0
WORD = 1
BYTE = 2

# OP codes (from OP_68K enum)
ADD   = 1
ADDI  = 3
SUB   = 92
SUBI  = 94
AND_B = 6
ANDI  = 7
OR_B  = 74
ORI   = 75
MOVEQ = 65
CLR   = 32
CMP   = 33
NEG   = 70
MOVE  = 55
EOR   = 41
NOT_B = 73


def init_signals(dut):
    """Initialize all ALU inputs to safe defaults."""
    dut.RESET.value = 0
    dut.LOAD_OP1.value = 0
    dut.LOAD_OP2.value = 0
    dut.LOAD_OP3.value = 0
    dut.OP1_IN.value = 0
    dut.OP2_IN.value = 0
    dut.OP3_IN.value = 0
    dut.BF_OFFSET_IN.value = 0
    dut.BF_WIDTH_IN.value = 0
    dut.BITPOS_IN.value = 0
    dut.ADR_MODE_IN.value = 0
    dut.OP_SIZE_IN.value = LONG
    dut.OP_IN.value = 0
    dut.OP_WB.value = 0
    dut.BIW_0_IN.value = 0
    dut.BIW_1_IN.value = 0
    dut.SR_WR.value = 0
    dut.SR_INIT.value = 0
    dut.SR_CLR_MBIT.value = 0
    dut.CC_UPDT.value = 0
    dut.ALU_INIT.value = 0
    dut.ALU_ACK.value = 0
    dut.USE_DREG.value = 0
    dut.HILOn.value = 0
    dut.IRQ_PEND.value = 0


async def do_reset(dut, cycles=5):
    """Assert RESET for the given number of cycles."""
    dut.RESET.value = 1
    await ClockCycles(dut.CLK, cycles)
    dut.RESET.value = 0
    await RisingEdge(dut.CLK)


async def alu_op(dut, op, op_size, op1, op2, op3=0, biw_0=0, biw_1=0, adr_mode=0):
    """Execute an ALU operation and return (result_lo32, status_reg).

    Protocol:
      1. Load operands (LOAD_OP1, LOAD_OP2, LOAD_OP3 concurrently)
      2. On the next cycle, assert ALU_INIT with OP_IN set
      3. Wait for ALU_REQ to go high
      4. Assert CC_UPDT to capture condition codes
      5. Assert ALU_ACK to complete the operation
      6. Read RESULT and STATUS_REG_OUT
    """
    # Load operands
    dut.OP1_IN.value = op1 & 0xFFFFFFFF
    dut.OP2_IN.value = op2 & 0xFFFFFFFF
    dut.OP3_IN.value = op3 & 0xFFFFFFFF
    dut.LOAD_OP1.value = 1
    dut.LOAD_OP2.value = 1
    dut.LOAD_OP3.value = 1
    dut.OP_IN.value = op
    dut.OP_SIZE_IN.value = op_size
    dut.BIW_0_IN.value = biw_0
    dut.BIW_1_IN.value = biw_1
    dut.ADR_MODE_IN.value = adr_mode
    await RisingEdge(dut.CLK)

    # Assert ALU_INIT
    dut.ALU_INIT.value = 1
    dut.LOAD_OP1.value = 0
    dut.LOAD_OP2.value = 0
    dut.LOAD_OP3.value = 0
    await RisingEdge(dut.CLK)

    dut.ALU_INIT.value = 0

    # Wait for ALU_REQ (up to 100 cycles for division etc)
    for _ in range(100):
        await RisingEdge(dut.CLK)
        try:
            if int(dut.ALU_REQ.value) == 1:
                break
        except ValueError:
            pass
    else:
        raise TimeoutError("ALU_REQ never asserted")

    # Update condition codes
    dut.CC_UPDT.value = 1
    await RisingEdge(dut.CLK)
    dut.CC_UPDT.value = 0

    # Read result before ACK (result is latched when ALU_REQ is high)
    result_full = int(dut.RESULT.value)
    result_lo = result_full & 0xFFFFFFFF

    # ACK to complete
    dut.ALU_ACK.value = 1
    await RisingEdge(dut.CLK)
    dut.ALU_ACK.value = 0
    await RisingEdge(dut.CLK)

    # Read status register
    status = int(dut.STATUS_REG_OUT.value)
    return result_lo, status


def get_flags(status):
    """Extract XNZVC flags from status register.

    Returns dict with keys: X, N, Z, V, C
    """
    return {
        "X": (status >> 4) & 1,
        "N": (status >> 3) & 1,
        "Z": (status >> 2) & 1,
        "V": (status >> 1) & 1,
        "C": (status >> 0) & 1,
    }


# --- Test cases ---

@cocotb.test()
async def test_add_simple(dut):
    """ADD: 5 + 3 = 8, check Z=0, N=0, V=0, C=0."""
    clock = Clock(dut.CLK, 10, unit="ns")
    cocotb.start_soon(clock.start())
    init_signals(dut)
    await RisingEdge(dut.CLK)
    await do_reset(dut)

    # ADD: OP1=source, OP2=destination, result = OP2 + OP1
    result, status = await alu_op(dut, ADD, LONG, 3, 5)
    flags = get_flags(status)

    dut._log.info(f"ADD 5+3: result=0x{result:08X}, flags={flags}")
    assert result == 8, f"Expected 8, got {result}"
    assert flags["Z"] == 0, "Z should be 0"
    assert flags["N"] == 0, "N should be 0"
    assert flags["V"] == 0, "V should be 0"
    assert flags["C"] == 0, "C should be 0"
    dut._log.info("PASS: ADD 5+3=8")


@cocotb.test()
async def test_add_overflow(dut):
    """ADD: 0x7FFFFFFF + 1, check V=1 (signed overflow), N=1."""
    clock = Clock(dut.CLK, 10, unit="ns")
    cocotb.start_soon(clock.start())
    init_signals(dut)
    await RisingEdge(dut.CLK)
    await do_reset(dut)

    result, status = await alu_op(dut, ADD, LONG, 1, 0x7FFFFFFF)
    flags = get_flags(status)

    dut._log.info(f"ADD 0x7FFFFFFF+1: result=0x{result:08X}, flags={flags}")
    assert result == 0x80000000, f"Expected 0x80000000, got 0x{result:08X}"
    assert flags["V"] == 1, "V should be 1 (signed overflow)"
    assert flags["N"] == 1, "N should be 1 (MSB set)"
    dut._log.info("PASS: ADD overflow detected")


@cocotb.test()
async def test_sub_simple(dut):
    """SUB: 8 - 3 = 5."""
    clock = Clock(dut.CLK, 10, unit="ns")
    cocotb.start_soon(clock.start())
    init_signals(dut)
    await RisingEdge(dut.CLK)
    await do_reset(dut)

    # SUB: result = OP2 - OP1  (destination - source)
    result, status = await alu_op(dut, SUB, LONG, 3, 8)
    flags = get_flags(status)

    dut._log.info(f"SUB 8-3: result=0x{result:08X}, flags={flags}")
    assert result == 5, f"Expected 5, got {result}"
    assert flags["Z"] == 0, "Z should be 0"
    assert flags["N"] == 0, "N should be 0"
    dut._log.info("PASS: SUB 8-3=5")


@cocotb.test()
async def test_sub_underflow(dut):
    """SUB: 0 - 1 = 0xFFFFFFFF, check C=1, N=1."""
    clock = Clock(dut.CLK, 10, unit="ns")
    cocotb.start_soon(clock.start())
    init_signals(dut)
    await RisingEdge(dut.CLK)
    await do_reset(dut)

    result, status = await alu_op(dut, SUB, LONG, 1, 0)
    flags = get_flags(status)

    dut._log.info(f"SUB 0-1: result=0x{result:08X}, flags={flags}")
    assert result == 0xFFFFFFFF, f"Expected 0xFFFFFFFF, got 0x{result:08X}"
    assert flags["C"] == 1, "C should be 1 (borrow)"
    assert flags["N"] == 1, "N should be 1 (negative)"
    dut._log.info("PASS: SUB underflow")


@cocotb.test()
async def test_and(dut):
    """AND: 0xFF00 & 0x0FF0 = 0x0F00."""
    clock = Clock(dut.CLK, 10, unit="ns")
    cocotb.start_soon(clock.start())
    init_signals(dut)
    await RisingEdge(dut.CLK)
    await do_reset(dut)

    # AND_B: result = OP1 & OP2
    result, status = await alu_op(dut, AND_B, WORD, 0xFF00, 0x0FF0)
    flags = get_flags(status)

    # Mask to word size for comparison
    result_word = result & 0xFFFF

    dut._log.info(f"AND 0xFF00 & 0x0FF0: result=0x{result_word:04X}, flags={flags}")
    assert result_word == 0x0F00, f"Expected 0x0F00, got 0x{result_word:04X}"
    assert flags["V"] == 0, "V should be 0 for AND"
    assert flags["C"] == 0, "C should be 0 for AND"
    dut._log.info("PASS: AND")


@cocotb.test()
async def test_or(dut):
    """OR: 0xFF00 | 0x0FF0 = 0xFFF0."""
    clock = Clock(dut.CLK, 10, unit="ns")
    cocotb.start_soon(clock.start())
    init_signals(dut)
    await RisingEdge(dut.CLK)
    await do_reset(dut)

    # OR_B: result = OP1 | OP2
    result, status = await alu_op(dut, OR_B, WORD, 0xFF00, 0x0FF0)
    flags = get_flags(status)

    result_word = result & 0xFFFF

    dut._log.info(f"OR 0xFF00 | 0x0FF0: result=0x{result_word:04X}, flags={flags}")
    assert result_word == 0xFFF0, f"Expected 0xFFF0, got 0x{result_word:04X}"
    dut._log.info("PASS: OR")


@cocotb.test()
async def test_zero_result(dut):
    """SUB equal values -> zero result, check Z=1."""
    clock = Clock(dut.CLK, 10, unit="ns")
    cocotb.start_soon(clock.start())
    init_signals(dut)
    await RisingEdge(dut.CLK)
    await do_reset(dut)

    result, status = await alu_op(dut, SUB, LONG, 42, 42)
    flags = get_flags(status)

    dut._log.info(f"SUB 42-42: result=0x{result:08X}, flags={flags}")
    assert result == 0, f"Expected 0, got {result}"
    assert flags["Z"] == 1, "Z should be 1 (zero result)"
    dut._log.info("PASS: Zero result Z=1")


@cocotb.test()
async def test_word_size_flags(dut):
    """ADD with WORD size: only lower 16 bits affect flags."""
    clock = Clock(dut.CLK, 10, unit="ns")
    cocotb.start_soon(clock.start())
    init_signals(dut)
    await RisingEdge(dut.CLK)
    await do_reset(dut)

    # WORD add: 0x7FFF + 1 = 0x8000 -- overflow in word, not in long
    result, status = await alu_op(dut, ADD, WORD, 1, 0x7FFF)
    flags = get_flags(status)

    result_word = result & 0xFFFF
    dut._log.info(f"ADD.W 0x7FFF+1: result=0x{result_word:04X}, flags={flags}")
    assert result_word == 0x8000, f"Expected 0x8000, got 0x{result_word:04X}"
    assert flags["V"] == 1, "V should be 1 (word-size overflow)"
    assert flags["N"] == 1, "N should be 1 (word MSB set)"
    dut._log.info("PASS: WORD size flags correct")


@cocotb.test()
async def test_byte_size_flags(dut):
    """ADD with BYTE size: only lower 8 bits affect flags."""
    clock = Clock(dut.CLK, 10, unit="ns")
    cocotb.start_soon(clock.start())
    init_signals(dut)
    await RisingEdge(dut.CLK)
    await do_reset(dut)

    # BYTE add: 0x7F + 1 = 0x80 -- overflow in byte
    result, status = await alu_op(dut, ADD, BYTE, 1, 0x7F)
    flags = get_flags(status)

    result_byte = result & 0xFF
    dut._log.info(f"ADD.B 0x7F+1: result=0x{result_byte:02X}, flags={flags}")
    assert result_byte == 0x80, f"Expected 0x80, got 0x{result_byte:02X}"
    assert flags["V"] == 1, "V should be 1 (byte-size overflow)"
    assert flags["N"] == 1, "N should be 1 (byte MSB set)"
    dut._log.info("PASS: BYTE size flags correct")


@cocotb.test()
async def test_clr(dut):
    """CLR: result should be 0, Z=1, N=0, V=0, C=0."""
    clock = Clock(dut.CLK, 10, unit="ns")
    cocotb.start_soon(clock.start())
    init_signals(dut)
    await RisingEdge(dut.CLK)
    await do_reset(dut)

    result, status = await alu_op(dut, CLR, LONG, 0, 0x12345678)
    flags = get_flags(status)

    dut._log.info(f"CLR: result=0x{result:08X}, flags={flags}")
    assert result == 0, f"Expected 0, got 0x{result:08X}"
    assert flags["Z"] == 1, "Z should be 1"
    assert flags["N"] == 0, "N should be 0"
    assert flags["V"] == 0, "V should be 0"
    assert flags["C"] == 0, "C should be 0"
    dut._log.info("PASS: CLR")


@cocotb.test()
async def test_add_negative_overflow(dut):
    """ADD: 0x80000001 + 0x80000001 = 0x00000002 -- negative overflow, V=1."""
    clock = Clock(dut.CLK, 10, unit="ns")
    cocotb.start_soon(clock.start())
    init_signals(dut)
    await RisingEdge(dut.CLK)
    await do_reset(dut)

    # Both operands are negative (MSB=1), result wraps to positive (MSB=0)
    # SM=1, DM=1, RM=0 -> V should be 1
    result, status = await alu_op(dut, ADD, LONG, 0x80000001, 0x80000001)
    flags = get_flags(status)

    dut._log.info(f"ADD 0x80000001+0x80000001: result=0x{result:08X}, flags={flags}")
    assert result == 0x00000002, f"Expected 0x00000002, got 0x{result:08X}"
    assert flags["V"] == 1, f"V should be 1 (negative overflow), got {flags['V']}"
    assert flags["N"] == 0, f"N should be 0 (result positive), got {flags['N']}"
    assert flags["C"] == 1, f"C should be 1 (unsigned carry), got {flags['C']}"
    assert flags["X"] == 1, f"X should be 1 (same as C), got {flags['X']}"
    dut._log.info("PASS: ADD negative overflow V=1")


@cocotb.test()
async def test_add_negative_overflow_exact(dut):
    """ADD: 0x80000000 + 0xFFFFFFFF = 0x7FFFFFFF -- negative overflow, V=1.
    
    Reproduces the exact values from the TRAPV test.
    """
    clock = Clock(dut.CLK, 10, unit="ns")
    cocotb.start_soon(clock.start())
    init_signals(dut)
    await RisingEdge(dut.CLK)
    await do_reset(dut)

    # OP1=source=0x80000000, OP2=dest=0xFFFFFFFF
    result, status = await alu_op(dut, ADD, LONG, 0x80000000, 0xFFFFFFFF)
    flags = get_flags(status)

    dut._log.info(f"ADD 0xFFFFFFFF+0x80000000: result=0x{result:08X}, flags={flags}")
    assert result == 0x7FFFFFFF, f"Expected 0x7FFFFFFF, got 0x{result:08X}"
    assert flags["V"] == 1, f"V should be 1 (negative overflow), got {flags['V']}"
    assert flags["N"] == 0, f"N should be 0 (result positive), got {flags['N']}"
    assert flags["C"] == 1, f"C should be 1 (unsigned carry), got {flags['C']}"
    dut._log.info("PASS: ADD negative overflow (exact test values) V=1")


@cocotb.test()
async def test_load_and_init_same_cycle_uses_new_operands(dut):
    """LOAD_OP* and ALU_INIT in the same cycle must use current input operands."""
    clock = Clock(dut.CLK, 10, unit="ns")
    cocotb.start_soon(clock.start())
    init_signals(dut)
    await RisingEdge(dut.CLK)
    await do_reset(dut)

    # Preload stale buffered values to make sure same-cycle bypass is exercised.
    dut.OP1_IN.value = 10
    dut.OP2_IN.value = 20
    dut.LOAD_OP1.value = 1
    dut.LOAD_OP2.value = 1
    await RisingEdge(dut.CLK)
    dut.LOAD_OP1.value = 0
    dut.LOAD_OP2.value = 0

    # Issue ADD with LOAD_OP* and ALU_INIT asserted in the same cycle.
    dut.OP_IN.value = ADD
    dut.OP_SIZE_IN.value = LONG
    dut.OP1_IN.value = 1
    dut.OP2_IN.value = 2
    dut.LOAD_OP1.value = 1
    dut.LOAD_OP2.value = 1
    dut.ALU_INIT.value = 1
    await RisingEdge(dut.CLK)
    dut.LOAD_OP1.value = 0
    dut.LOAD_OP2.value = 0
    dut.ALU_INIT.value = 0

    for _ in range(20):
        await RisingEdge(dut.CLK)
        if int(dut.ALU_REQ.value) == 1:
            break
    else:
        raise TimeoutError("ALU_REQ never asserted for same-cycle load/init test")

    dut.CC_UPDT.value = 1
    await RisingEdge(dut.CLK)
    dut.CC_UPDT.value = 0

    result = int(dut.RESULT.value) & 0xFFFFFFFF
    status = int(dut.STATUS_REG_OUT.value)
    flags = get_flags(status)

    dut.ALU_ACK.value = 1
    await RisingEdge(dut.CLK)
    dut.ALU_ACK.value = 0
    await RisingEdge(dut.CLK)

    dut._log.info(f"same-cycle load/init ADD: result=0x{result:08X}, flags={flags}")
    assert result == 3, f"Expected 3 from 2+1, got {result}"
    assert flags["Z"] == 0, "Z should be 0"
