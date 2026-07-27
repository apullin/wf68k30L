"""
QEMU differential random-program checks.

Generates deterministic random 68k programs (seeded), executes each on:
  - WF68K30L (cocotb/Verilator)
  - qemu-system-m68k (CPU m68030)

Compares, at epilogue entry:
  - the full integer register state (D0-D7 / A0-A7)
  - the defined CCR bits of SR (X/N/Z/V/C)
  - a longword sum over the scratch memory window the program writes to,
    computed by the program itself so that a memory divergence shows up as a
    register divergence (QEMU's RAM is not otherwise observable here)

The operation mix deliberately covers the classes a register-only, long-only
comparison cannot see: byte and word sizes, shifts and rotates with both
immediate and register counts, CMP/CMPI feeding Bcc and Scc so flags reach
control flow, MULS.W/MULU.W, and (An)+ / -(An) memory traffic.

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
    BYTE,
    WORD,
    LONG,
    DN,
    AN,
    AN_IND,
    AN_POSTINC,
    AN_PREDEC,
    AN_DISP,
    ABS_L,
    SPECIAL,
    IMMEDIATE,
    abs_long,
    add,
    addi,
    addq,
    and_op,
    asl,
    asr,
    bcc,
    cmp_reg,
    cmpi,
    eor,
    ext_l,
    ext_w,
    imm_long,
    imm_word,
    lsl,
    lsr,
    move,
    move_from_sr,
    move_to_abs_long,
    move_to_ccr,
    movea,
    moveq,
    muls_w,
    mulu_w,
    neg,
    not_op,
    or_op,
    rol,
    ror,
    roxl,
    roxr,
    scc,
    sub,
    subi,
    subq,
    swap,
    tst,
)
from qemu_m68k_ref import qemu_m68030_state_trace


# Scratch layout, all relative to CPUTestHarness.DATA_BASE. The fold window is
# the only memory the program is allowed to write, so the checksum the program
# computes over it covers every store the generator can emit.
SCRATCH_OFF = 0x300   # A6 base for (d16,A6) accesses; disps span -79..+11
CURSOR_OFF = 0x340    # A5 cursor start for (A5)+ / -(A5)
CURSOR_LO_OFF = 0x320
CURSOR_HI_OFF = 0x370
FOLD_LO_OFF = 0x280   # fold covers [DATA_BASE+0x280, DATA_BASE+0x380)
FOLD_LONGS = 64

RESULT_SR_OFF = 64        # word: SR captured at epilogue entry
RESULT_MEMSUM_OFF = 68    # long: sum of the fold window

# Bcc/Scc conditions. Condition 0 is T and 1 is F; in the Bcc encoding those
# two slots are BRA and BSR, and BSR would change control flow, so only real
# conditions 2..15 are used.
CONDITIONS = tuple(range(2, 16))
SIZES = (BYTE, WORD, LONG)


def _w16(v):
    return v & 0xFFFF


class _Emitter:
    """Accumulates instruction words and counts emitted instructions."""

    def __init__(self, base_pc):
        self.words = []
        self.count = 0
        self.base_pc = base_pc

    def emit(self, *chunks):
        for chunk in chunks:
            self.words.extend(chunk)
        self.count += 1

    @property
    def pc(self):
        return self.base_pc + len(self.words) * 2


def _imm_for(rnd, size):
    if size == BYTE:
        return rnd.randrange(0, 256)
    if size == WORD:
        return rnd.getrandbits(16)
    return rnd.getrandbits(32)


def _size_bytes(size):
    return {BYTE: 1, WORD: 2, LONG: 4}[size]


# ---------------------------------------------------------------------------
# Operation emitters
#
# Each takes (em, rnd, st) where st carries the generator's model of the A5
# cursor offset and the (d16,A6) displacements written so far.
# ---------------------------------------------------------------------------

def _op_moveq(em, rnd, st):
    em.emit(moveq(rnd.randrange(-128, 128), rnd.randrange(8)))


def _op_addq(em, rnd, st):
    em.emit(addq(rnd.choice(SIZES), rnd.randrange(1, 9), DN, rnd.randrange(8)))


def _op_subq(em, rnd, st):
    em.emit(subq(rnd.choice(SIZES), rnd.randrange(1, 9), DN, rnd.randrange(8)))


def _op_add(em, rnd, st):
    em.emit(add(rnd.choice(SIZES), rnd.randrange(8), 0, DN, rnd.randrange(8)))


def _op_sub(em, rnd, st):
    em.emit(sub(rnd.choice(SIZES), rnd.randrange(8), 0, DN, rnd.randrange(8)))


def _op_and(em, rnd, st):
    em.emit(and_op(rnd.choice(SIZES), rnd.randrange(8), 0, DN, rnd.randrange(8)))


def _op_or(em, rnd, st):
    em.emit(or_op(rnd.choice(SIZES), rnd.randrange(8), 0, DN, rnd.randrange(8)))


def _op_eor(em, rnd, st):
    # EOR only has the Dn -> <ea> direction.
    em.emit(eor(rnd.choice(SIZES), rnd.randrange(8), DN, rnd.randrange(8)))


def _op_addi(em, rnd, st):
    size = rnd.choice(SIZES)
    em.emit(addi(size, DN, rnd.randrange(8), _imm_for(rnd, size)))


def _op_subi(em, rnd, st):
    size = rnd.choice(SIZES)
    em.emit(subi(size, DN, rnd.randrange(8), _imm_for(rnd, size)))


def _op_unary(em, rnd, st):
    """NEG / NOT / TST / SWAP / EXT: cheap size-sensitive flag setters."""
    which = rnd.randrange(6)
    dn = rnd.randrange(8)
    if which == 0:
        em.emit(neg(rnd.choice(SIZES), DN, dn))
    elif which == 1:
        em.emit(not_op(rnd.choice(SIZES), DN, dn))
    elif which == 2:
        em.emit(tst(rnd.choice(SIZES), DN, dn))
    elif which == 3:
        em.emit(swap(dn))
    elif which == 4:
        em.emit(ext_w(dn))
    else:
        em.emit(ext_l(dn))


def _op_shift_imm(em, rnd, st):
    """Shift/rotate by an immediate count of 1..8."""
    fn = rnd.choice((asl, asr, lsl, lsr, rol, ror, roxl, roxr))
    count = rnd.randrange(1, 9)
    em.emit(fn(rnd.choice(SIZES), count & 7, rnd.randrange(8), ir=0))


def _op_shift_reg(em, rnd, st):
    """Shift/rotate by a register count (taken mod 64 by the hardware)."""
    fn = rnd.choice((asl, asr, lsl, lsr, rol, ror, roxl, roxr))
    em.emit(fn(rnd.choice(SIZES), rnd.randrange(8), rnd.randrange(8), ir=1))


def _op_cmp_branch(em, rnd, st):
    """CMP.<sz> Ds,Dd ; Bcc.S over one word ; <skipped 1-word insn>.

    The only construct here where a condition code decides control flow, so a
    CCR bug can change which instructions execute rather than only a stored
    value.
    """
    size = rnd.choice(SIZES)
    dst = rnd.randrange(8)
    src = rnd.randrange(8)
    em.emit(cmp_reg(size, dst, DN, src))
    # Displacement is relative to PC+2, so 2 skips exactly one one-word
    # instruction.
    em.emit(bcc(rnd.choice(CONDITIONS), 2))
    em.emit(addq(LONG, rnd.randrange(1, 9), DN, rnd.randrange(8)))


def _op_cmpi_scc(em, rnd, st):
    """CMPI.<sz> #imm,Dn ; Scc Dn -- a flag consumer without control flow."""
    size = rnd.choice(SIZES)
    em.emit(cmpi(size, DN, rnd.randrange(8), _imm_for(rnd, size)))
    em.emit(scc(rnd.choice(CONDITIONS), DN, rnd.randrange(8)))


def _op_mul(em, rnd, st):
    """MULS.W / MULU.W Ds,Dd (16x16 -> 32)."""
    fn = rnd.choice((muls_w, mulu_w))
    em.emit(fn(DN, rnd.randrange(8), rnd.randrange(8)))


def _op_mem_disp_store_imm(em, rnd, st):
    """MOVE.<sz> #imm,(d16,A6)."""
    size = rnd.choice(SIZES)
    disp = rnd.choice(st["disps"])
    em.emit(
        move(size, SPECIAL, IMMEDIATE, AN_DISP, 6),
        imm_long(_imm_for(rnd, LONG)) if size == LONG else imm_word(
            _imm_for(rnd, size)
        ),
        [_w16(disp)],
    )
    st["used_disps"].append(disp)


def _op_mem_disp_load(em, rnd, st):
    """MOVE.<sz> (d16,A6),Dn."""
    size = rnd.choice(SIZES)
    disp = rnd.choice(st["used_disps"])
    em.emit(move(size, AN_DISP, 6, DN, rnd.randrange(8)), [_w16(disp)])


def _op_mem_disp_store_reg(em, rnd, st):
    """MOVE.<sz> Dn,(d16,A6)."""
    size = rnd.choice(SIZES)
    disp = rnd.choice(st["disps"])
    em.emit(move(size, DN, rnd.randrange(8), AN_DISP, 6), [_w16(disp)])
    st["used_disps"].append(disp)


def _op_movea_disp(em, rnd, st):
    """MOVEA.L (d16,A6),An -- A5/A6/A7 are reserved, so An is A0..A4."""
    disp = rnd.choice(st["used_disps"])
    em.emit(movea(LONG, AN_DISP, 6, rnd.randrange(5)), [_w16(disp)])


def _op_movea_reg(em, rnd, st):
    em.emit(movea(LONG, AN, rnd.randrange(5), rnd.randrange(5)))


def _op_move_reg(em, rnd, st):
    em.emit(move(rnd.choice(SIZES), DN, rnd.randrange(8), DN, rnd.randrange(8)))


def _recenter_cursor(em, st):
    """MOVEA.L #cursor_start,A5 -- keeps the cursor inside the fold window."""
    em.emit(movea(LONG, SPECIAL, IMMEDIATE, 5), imm_long(st["cursor_base"]))
    st["cursor"] = st["cursor_base"]


def _op_cursor(em, rnd, st):
    """(A5)+ / -(A5) traffic, in whichever direction and size fits the window.

    The generator tracks the cursor exactly, so it knows when an access would
    leave the window and re-centers first. Byte accesses are allowed to leave
    the cursor odd: misaligned word/long accesses are legal on the 68020+ and
    are exactly the case a long-only, aligned-only generator never reaches.
    """
    size = rnd.choice(SIZES)
    nbytes = _size_bytes(size)
    postinc = rnd.randrange(2) == 0
    if postinc:
        if st["cursor"] + nbytes > st["cursor_hi"]:
            _recenter_cursor(em, st)
        mode = AN_POSTINC
    else:
        if st["cursor"] - nbytes < st["cursor_lo"]:
            _recenter_cursor(em, st)
        mode = AN_PREDEC

    dn = rnd.randrange(8)
    if rnd.randrange(2) == 0:
        em.emit(move(size, mode, 5, DN, dn))       # load
    else:
        em.emit(move(size, DN, dn, mode, 5))       # store
    st["cursor"] += nbytes if postinc else -nbytes


_OPS = (
    _op_moveq,
    _op_addq,
    _op_subq,
    _op_add,
    _op_sub,
    _op_and,
    _op_or,
    _op_eor,
    _op_addi,
    _op_subi,
    _op_unary,
    _op_shift_imm,
    _op_shift_imm,
    _op_shift_reg,
    _op_shift_reg,
    _op_cmp_branch,
    _op_cmp_branch,
    _op_cmpi_scc,
    _op_mul,
    _op_mem_disp_store_imm,
    _op_mem_disp_load,
    _op_mem_disp_store_reg,
    _op_movea_disp,
    _op_movea_reg,
    _op_move_reg,
    _op_cursor,
    _op_cursor,
    _op_cursor,
)


def _build_random_program(h, seed, op_count):
    """Build one seeded random program.

    Returns (words, epilogue_pc, post_fold_pc, executed_estimate).
    """
    rnd = random.Random(seed)
    em = _Emitter(h.PROGRAM_BASE)

    scratch_base = h.DATA_BASE + SCRATCH_OFF
    st = {
        "disps": [-79, -78, -77, -3, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11],
        "used_disps": [0, 1, 2, 3],
        "cursor_base": h.DATA_BASE + CURSOR_OFF,
        "cursor_lo": h.DATA_BASE + CURSOR_LO_OFF,
        "cursor_hi": h.DATA_BASE + CURSOR_HI_OFF,
        "cursor": h.DATA_BASE + CURSOR_OFF,
    }

    # Prologue. MOVE #0,CCR pins all five CCR bits (X included) to a defined
    # value on both sides, so the SR comparison never depends on reset state.
    em.emit(move_to_ccr(SPECIAL, IMMEDIATE), imm_word(0x0000))
    em.emit(movea(LONG, SPECIAL, IMMEDIATE, 6), imm_long(scratch_base))
    em.emit(movea(LONG, SPECIAL, IMMEDIATE, 5), imm_long(st["cursor_base"]))
    em.emit(movea(LONG, SPECIAL, IMMEDIATE, 7), imm_long(0x00001800))
    for an in range(5):
        em.emit(
            movea(LONG, SPECIAL, IMMEDIATE, an),
            imm_long(st["cursor_base"] + an * 4),
        )
    for dn in range(8):
        em.emit(
            move(LONG, SPECIAL, IMMEDIATE, DN, dn),
            imm_long(rnd.getrandbits(32)),
        )

    for _ in range(op_count):
        rnd.choice(_OPS)(em, rnd, st)

    # Epilogue. The SR store must come first: every store below it writes the
    # condition codes.
    epilogue_pc = em.pc
    em.emit(
        move_from_sr(SPECIAL, ABS_L),
        abs_long(h.RESULT_BASE + RESULT_SR_OFF),
    )
    for reg in range(8):
        em.emit(move_to_abs_long(LONG, DN, reg, h.RESULT_BASE + (reg * 4)))
    for reg in range(8):
        em.emit(move_to_abs_long(LONG, AN, reg, h.RESULT_BASE + 32 + (reg * 4)))

    # Fold the scratch window into D0 so memory divergence is observable in a
    # register. Registers are already stored, so D0/D1/A0 are free here.
    fold_lo = h.DATA_BASE + FOLD_LO_OFF
    em.emit(movea(LONG, SPECIAL, IMMEDIATE, 0), imm_long(fold_lo))
    em.emit(moveq(0, 0))
    em.emit(moveq(FOLD_LONGS, 1))
    loop_pc = em.pc
    em.emit(add(LONG, 0, 0, AN_POSTINC, 0))    # ADD.L (A0)+,D0
    em.emit(subq(LONG, 1, DN, 1))              # SUBQ.L #1,D1
    em.emit(bcc(0x6, -6))                      # BNE.S loop  (back 3 words)
    assert em.pc == loop_pc + 6, "fold loop displacement assumes a 3-word body"
    post_fold_pc = em.pc
    em.emit(move_to_abs_long(LONG, DN, 0, h.RESULT_BASE + RESULT_MEMSUM_OFF))

    em.emit(h.sentinel_program())              # counts as one; two in practice

    # QEMU must trace at least as far as post_fold_pc. The fold loop is emitted
    # once but executed FOLD_LONGS times, so the executed count is well above
    # the emitted count; overshooting is harmless (QEMU keeps running).
    executed_estimate = em.count + (FOLD_LONGS - 1) * 3 + 32

    return em.words, epilogue_pc, post_fold_pc, executed_estimate


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


# Only the five CCR bits are compared.
#
#   bit 4 X, 3 N, 2 Z, 1 V, 0 C  -- architecturally defined results for every
#       instruction this generator emits (X is pinned by MOVE #0,CCR in the
#       prologue rather than inherited from reset).
#
# Masked out, deliberately:
#   bits 15-13 T1/T0/S, bit 12 M, bits 10-8 I2-I0 -- execution context, not
#       instruction results. They match by construction (both sides start in
#       supervisor with IPL=7 and the program never writes SR), but comparing
#       them would fail on a harness difference rather than a core defect.
#   bits 11, 7-5 -- unused in the MC68030 SR (UM Figure 1-4 / PRM 1.3). The
#       architecture does not define what a read returns, so a mismatch there
#       would only record QEMU's choice to report them as zero.
CCR_MASK = 0x001F


async def _run_qemu_diff_seed(dut, seed, op_count):
    """Randomized differential check for one seed vs QEMU m68030."""
    if op_count <= 0:
        raise AssertionError(f"QEMU_DIFF_OPS must be > 0 (got {op_count})")

    h = CPUTestHarness(dut)
    program, epilogue_pc, post_fold_pc, insn_budget = _build_random_program(
        h, seed, op_count
    )

    qemu_snaps = qemu_m68030_state_trace(
        program,
        insn_budget,
        program_base=h.PROGRAM_BASE,
        timeout_s=10.0,
    )

    qemu_ep = None
    qemu_fold = None
    for snap in qemu_snaps:
        if qemu_ep is None and snap["pc"] == epilogue_pc:
            qemu_ep = snap
        elif qemu_ep is not None and snap["pc"] == post_fold_pc:
            qemu_fold = snap
            break
    assert qemu_ep is not None, (
        f"QEMU did not reach epilogue PC 0x{epilogue_pc:08X} (seed={seed})"
    )
    assert qemu_fold is not None, (
        f"QEMU did not reach post-fold PC 0x{post_fold_pc:08X} (seed={seed})"
    )

    await h.setup(program)
    found = await h.run_until_sentinel(max_cycles=250000)
    assert found, f"Sentinel not reached (seed={seed})"

    got_d = [h.read_result_long(reg * 4) & 0xFFFFFFFF for reg in range(8)]
    got_a = [h.read_result_long(32 + reg * 4) & 0xFFFFFFFF for reg in range(8)]
    got_sr = h.read_result_word(RESULT_SR_OFF)
    got_memsum = h.read_result_long(RESULT_MEMSUM_OFF)
    exp_d = [value & 0xFFFFFFFF for value in qemu_ep["d"]]
    exp_a = [value & 0xFFFFFFFF for value in qemu_ep["a"]]
    exp_sr = qemu_ep["sr"]
    exp_memsum = qemu_fold["d"][0] & 0xFFFFFFFF

    mismatches = []
    for reg, (got, exp) in enumerate(zip(got_d, exp_d)):
        if got != exp:
            mismatches.append(f"D{reg}: exp=0x{exp:08X} got=0x{got:08X}")
    for reg, (got, exp) in enumerate(zip(got_a, exp_a)):
        if got != exp:
            mismatches.append(f"A{reg}: exp=0x{exp:08X} got=0x{got:08X}")
    if (got_sr ^ exp_sr) & CCR_MASK:
        mismatches.append(
            f"CCR: exp={_ccr_str(exp_sr)} got={_ccr_str(got_sr)} "
            f"(SR exp=0x{exp_sr:04X} got=0x{got_sr:04X}, compared bits "
            f"0x{CCR_MASK:04X})"
        )
    if got_memsum != exp_memsum:
        mismatches.append(
            f"scratch memory sum: exp=0x{exp_memsum:08X} got=0x{got_memsum:08X}"
        )

    assert not mismatches, (
        f"QEMU diff mismatch (seed={seed}, ops={op_count})\n"
        f"epilogue_pc=0x{epilogue_pc:08X}\n"
        f"{'\n'.join(mismatches)}"
    )


def _ccr_str(sr):
    bits = "XNZVC"
    return "".join(
        bit if (sr >> shift) & 1 else "-"
        for bit, shift in zip(bits, (4, 3, 2, 1, 0))
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
