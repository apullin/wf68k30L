"""
QEMU differential smoke test.

Compares the first instruction-start PC trace from WF68K30L against
qemu-system-m68k (CPU m68030) for a short deterministic program.
"""

import cocotb
from cocotb.triggers import RisingEdge

from cpu_harness import CPUTestHarness
from m68k_encode import (
    LONG,
    DN,
    SPECIAL,
    IMMEDIATE,
    add,
    addq,
    imm_long,
    move_to_abs_long,
    movea,
    moveq,
)
from qemu_m68k_ref import qemu_m68030_pc_trace


def _program(h):
    return [
        *movea(LONG, SPECIAL, IMMEDIATE, 7),      # A7 = 0x00001000
        *imm_long(0x00001000),
        *moveq(1, 0),                             # D0 = 1
        *addq(LONG, 2, DN, 0),                    # D0 += 2
        *moveq(3, 1),                             # D1 = 3
        *add(LONG, 0, 0, DN, 1),                  # D0 += D1
        *move_to_abs_long(LONG, DN, 0, h.RESULT_BASE),
        *h.sentinel_program(),
    ]


async def _dut_pc_trace(dut, h, instruction_limit, max_cycles=20000):
    pcs = []
    prev_ack = 0
    for _ in range(max_cycles):
        await RisingEdge(dut.CLK)
        try:
            ack = int(dut.OPD_ACK_MAIN.value)
            pc = int(dut.PC.value) & 0xFFFFFFFF
        except ValueError:
            continue

        if ack == 1 and prev_ack == 0:
            pcs.append(pc)
            if len(pcs) >= instruction_limit:
                return pcs
        prev_ack = ack
    return pcs


@cocotb.test()
async def test_qemu_diff_pc_trace_smoke(dut):
    """Instruction-start PC sequence should match QEMU m68030."""
    h = CPUTestHarness(dut)
    program = _program(h)
    trace_len = 8

    expected = qemu_m68030_pc_trace(
        program,
        trace_len,
        program_base=h.PROGRAM_BASE,
        timeout_s=3.0,
    )

    await h.setup(program)
    got = await _dut_pc_trace(dut, h, trace_len)

    assert len(got) >= trace_len, (
        f"DUT trace too short: got {len(got)}, need {trace_len}"
    )
    assert got[:trace_len] == expected, (
        "PC trace mismatch vs QEMU m68030\n"
        f"expected={['0x%08X' % x for x in expected]}\n"
        f"got=     {['0x%08X' % x for x in got[:trace_len]]}"
    )
