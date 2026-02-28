"""
QEMU differential random-program checks.

Generates deterministic random 68k programs (seeded), executes each on:
  - WF68K30L (cocotb/Verilator)
  - qemu-system-m68k (CPU m68030)

Compares full integer register state (D0-D7/A0-A7) at epilogue entry.

Environment knobs:
  QEMU_DIFF_SEED   Single seed (default: 1)
  QEMU_DIFF_SEEDS  Seed expression (e.g. "1-100" or "1,4,7,20"); overrides
                   QEMU_DIFF_SEED when set.
  QEMU_DIFF_OPS    Random operations per seed (default: 48)
"""

from __future__ import annotations

import os
import random

import cocotb

from cpu_harness import CPUTestHarness
from m68k_encode import (
    LONG,
    DN,
    AN,
    AN_DISP,
    SPECIAL,
    IMMEDIATE,
    add,
    addi,
    addq,
    and_op,
    eor,
    imm_long,
    move,
    move_to_abs_long,
    movea,
    moveq,
    or_op,
    sub,
    subi,
    subq,
)
from qemu_m68k_ref import qemu_m68030_state_trace


def _w16(v):
    return v & 0xFFFF


def _build_random_program(h, seed, op_count):
    rnd = random.Random(seed)
    words = []
    insn_count = 0

    # Stable base pointers for deterministic scratch-memory accesses.
    scratch_base = h.DATA_BASE + 0x300
    words += [*movea(LONG, SPECIAL, IMMEDIATE, 6), *imm_long(scratch_base)]
    insn_count += 1
    words += [*movea(LONG, SPECIAL, IMMEDIATE, 7), *imm_long(0x00001800)]
    insn_count += 1

    disps = [-79, -78, -77, -3, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11]
    used_disps = [0, 1, 2, 3]

    for _ in range(op_count):
        op = rnd.randrange(16)

        if op == 0:
            dn = rnd.randrange(8)
            imm8 = rnd.randrange(-128, 128)
            words += moveq(imm8, dn)
            insn_count += 1
        elif op == 1:
            dn = rnd.randrange(8)
            qv = rnd.randrange(1, 9)
            words += addq(LONG, qv, DN, dn)
            insn_count += 1
        elif op == 2:
            dn = rnd.randrange(8)
            qv = rnd.randrange(1, 9)
            words += subq(LONG, qv, DN, dn)
            insn_count += 1
        elif op == 3:
            dst = rnd.randrange(8)
            src = rnd.randrange(8)
            words += add(LONG, dst, 0, DN, src)  # ADD.L Dsrc,Ddst
            insn_count += 1
        elif op == 4:
            dst = rnd.randrange(8)
            src = rnd.randrange(8)
            words += sub(LONG, dst, 0, DN, src)  # SUB.L Dsrc,Ddst
            insn_count += 1
        elif op == 5:
            dst = rnd.randrange(8)
            src = rnd.randrange(8)
            words += and_op(LONG, dst, 0, DN, src)  # AND.L Dsrc,Ddst
            insn_count += 1
        elif op == 6:
            dst = rnd.randrange(8)
            src = rnd.randrange(8)
            words += or_op(LONG, dst, 0, DN, src)  # OR.L Dsrc,Ddst
            insn_count += 1
        elif op == 7:
            dst = rnd.randrange(8)
            src = rnd.randrange(8)
            words += eor(LONG, src, DN, dst)  # EOR.L Dsrc,Ddst
            insn_count += 1
        elif op == 8:
            dn = rnd.randrange(8)
            imm32 = rnd.getrandbits(32)
            words += addi(LONG, DN, dn, imm32)
            insn_count += 1
        elif op == 9:
            dn = rnd.randrange(8)
            imm32 = rnd.getrandbits(32)
            words += subi(LONG, DN, dn, imm32)
            insn_count += 1
        elif op == 10:
            disp = rnd.choice(disps)
            imm32 = rnd.getrandbits(32)
            words += move(LONG, SPECIAL, IMMEDIATE, AN_DISP, 6)
            words += imm_long(imm32)
            words += [_w16(disp)]
            insn_count += 1
            used_disps.append(disp)
        elif op == 11:
            disp = rnd.choice(used_disps)
            dn = rnd.randrange(8)
            words += move(LONG, AN_DISP, 6, DN, dn)
            words += [_w16(disp)]
            insn_count += 1
        elif op == 12:
            disp = rnd.choice(used_disps)
            an = rnd.randrange(6)  # keep A6 as scratch base; avoid A7 stack ptr
            words += movea(LONG, AN_DISP, 6, an)
            words += [_w16(disp)]
            insn_count += 1
        elif op == 13:
            disp = rnd.choice(disps)
            dn = rnd.randrange(8)
            words += move(LONG, DN, dn, AN_DISP, 6)
            words += [_w16(disp)]
            insn_count += 1
            used_disps.append(disp)
        elif op == 14:
            src = rnd.randrange(8)
            dst = rnd.randrange(8)
            words += move(LONG, DN, src, DN, dst)
            insn_count += 1
        else:
            src = rnd.randrange(6)
            dst = rnd.randrange(6)
            words += movea(LONG, AN, src, dst)
            insn_count += 1

    # Mark epilogue start for state comparison.
    epilogue_pc = h.PROGRAM_BASE + len(words) * 2

    # Persist full integer register state for DUT-side checking.
    for reg in range(8):
        words += move_to_abs_long(LONG, DN, reg, h.RESULT_BASE + (reg * 4))
    for reg in range(8):
        words += move_to_abs_long(LONG, AN, reg, h.RESULT_BASE + 32 + (reg * 4))

    words += h.sentinel_program()

    return words, epilogue_pc, insn_count


def _parse_seed_expr(expr):
    seeds = set()
    for token in expr.split(","):
        part = token.strip()
        if not part:
            continue
        if "-" in part:
            start_s, end_s = part.split("-", 1)
            start = int(start_s.strip(), 0)
            end = int(end_s.strip(), 0)
            if end < start:
                raise ValueError(f"Invalid seed range '{part}'")
            seeds.update(range(start, end + 1))
        else:
            seeds.add(int(part, 0))
    parsed = sorted(seeds)
    if not parsed:
        raise ValueError("No seeds parsed from QEMU_DIFF_SEEDS")
    return parsed


async def _run_qemu_diff_seed(dut, seed, op_count):
    """Randomized differential check for one seed vs QEMU m68030."""
    if op_count <= 0:
        raise AssertionError(f"QEMU_DIFF_OPS must be > 0 (got {op_count})")

    h = CPUTestHarness(dut)
    program, epilogue_pc, insn_count = _build_random_program(h, seed, op_count)

    # Need enough snapshots to include epilogue entry plus a little slack.
    qemu_snaps = qemu_m68030_state_trace(
        program,
        insn_count + 16,
        program_base=h.PROGRAM_BASE,
        timeout_s=4.0,
    )

    qemu_ep = None
    for snap in qemu_snaps:
        if snap["pc"] == epilogue_pc:
            qemu_ep = snap
            break
    assert qemu_ep is not None, (
        f"QEMU did not reach epilogue PC 0x{epilogue_pc:08X} (seed={seed})"
    )

    await h.setup(program)
    found = await h.run_until_sentinel(max_cycles=250000)
    assert found, f"Sentinel not reached (seed={seed})"

    got_d = [h.read_result_long(reg * 4) & 0xFFFFFFFF for reg in range(8)]
    got_a = [h.read_result_long(32 + reg * 4) & 0xFFFFFFFF for reg in range(8)]
    exp_d = [value & 0xFFFFFFFF for value in qemu_ep["d"]]
    exp_a = [value & 0xFFFFFFFF for value in qemu_ep["a"]]

    mismatches = []
    for reg, (got, exp) in enumerate(zip(got_d, exp_d)):
        if got != exp:
            mismatches.append(f"D{reg}: exp=0x{exp:08X} got=0x{got:08X}")
    for reg, (got, exp) in enumerate(zip(got_a, exp_a)):
        if got != exp:
            mismatches.append(f"A{reg}: exp=0x{exp:08X} got=0x{got:08X}")

    assert not mismatches, (
        f"QEMU diff mismatch (seed={seed}, ops={op_count})\n"
        f"epilogue_pc=0x{epilogue_pc:08X}\n"
        f"{'\n'.join(mismatches)}"
    )


def _register_seed_tests():
    op_count = int(os.environ.get("QEMU_DIFF_OPS", "48"), 0)
    seed_expr = os.environ.get("QEMU_DIFF_SEEDS", "").strip()

    if seed_expr:
        seeds = _parse_seed_expr(seed_expr)
        for seed in seeds:
            async def _seed_test(dut, _seed=seed):
                await _run_qemu_diff_seed(dut, seed=_seed, op_count=op_count)

            test_name = f"test_qemu_diff_seed_{seed:05d}"
            _seed_test.__name__ = test_name
            _seed_test.__qualname__ = test_name
            _seed_test.__doc__ = f"QEMU randomized differential check for seed {seed}."
            globals()[test_name] = cocotb.test()(_seed_test)
        return

    seed = int(os.environ.get("QEMU_DIFF_SEED", "1"), 0)

    async def _single_seed_test(dut):
        await _run_qemu_diff_seed(dut, seed=seed, op_count=op_count)

    _single_seed_test.__name__ = "test_qemu_diff_random_seed"
    _single_seed_test.__qualname__ = "test_qemu_diff_random_seed"
    _single_seed_test.__doc__ = (
        "Randomized differential check (single seed) vs QEMU m68030."
    )
    globals()["test_qemu_diff_random_seed"] = cocotb.test()(_single_seed_test)


_register_seed_tests()
