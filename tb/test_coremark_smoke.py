"""
CoreMark smoke run on WF68K30L for optimization levels O0/O1/O2/Os.

This is a functional completion smoke, not a standards-compliant CoreMark score
submission flow. The benchmark is built bare-metal and reports completion via
the existing sentinel address used by the cocotb harness.

Environment knobs:
  COREMARK_MAX_CYCLES      Max cycles per optimization run (default: 100000000)
  COREMARK_ITERATIONS      Iterations passed to build script (default: 1)
  COREMARK_TOTAL_DATA_SIZE Data size passed to build script (default: 2000)
  COREMARK_OPTS            Comma list of opts to run (default: O0,O1,O2,Os)
  COREMARK_SEED1           Seed1 override passed to build script
  COREMARK_SEED2           Seed2 override passed to build script
  COREMARK_SEED3           Seed3 override passed to build script
  COREMARK_EXECS_MASK      Algorithm mask passed to build script
  COREMARK_LIST_ITEMS      Linked-list item cap passed to build script
  COREMARK_EXTRA_CFLAGS    Extra compiler flags passed to build script
"""

import os
import shutil
import subprocess
from pathlib import Path

import cocotb
from cocotb.triggers import RisingEdge

from cpu_harness import CPUTestHarness
from m68k_encode import (
    LONG,
    AN,
    DN,
    SPECIAL,
    IMMEDIATE,
    imm_long,
    move,
    move_to_abs_long,
)


REPO_ROOT = Path(__file__).resolve().parents[1]
BUILD_SCRIPT = REPO_ROOT / "tooling" / "coremark" / "build_case.sh"
OPTS = ("O0", "O1", "O2", "Os")
DEFAULT_TOTAL_DATA_SIZE = "2000"
DEFAULT_ITERATIONS = "1"

# CoreMark's own CRC verdict, published at RESULT_BASE+8 by
# tooling/coremark/port/core_portme.c. Keep in sync with that file.
VALIDATION_UNKNOWN = 0
VALIDATION_OK = 1
VALIDATION_ERRORS = 2
VALIDATION_UNCHECKABLE = 3
VALIDATION_NAMES = {
    VALIDATION_UNKNOWN: "none",
    VALIDATION_OK: "crc-ok",
    VALIDATION_ERRORS: "crc-ERRORS",
    VALIDATION_UNCHECKABLE: "no-known-crc",
}

# The seedcrc values core_main.c has known-good algorithm CRCs for (the switch
# at core_main.c:292). If the configuration produces one of these, CoreMark
# validates its own results and this test can require it to succeed.
KNOWN_SEEDCRCS = (0x8A02, 0x7B05, 0x4EAF, 0xE9F5, 0x18F2)
NUM_ALGORITHMS = 3  # ID_LIST | ID_MATRIX | ID_STATE, the port's default mask


def _crcu8(data, crc):
    """CoreMark's crcu8 (tooling/coremark/src/core_util.c:165)."""
    for _ in range(8):
        x16 = (data & 1) ^ (crc & 1)
        data >>= 1
        if x16 == 1:
            crc ^= 0x4002
            carry = 1
        else:
            carry = 0
        crc >>= 1
        crc = (crc | 0x8000) if carry else (crc & 0x7FFF)
    return crc & 0xFFFF


def _crc16(value, crc):
    value &= 0xFFFF
    return _crcu8((value >> 8) & 0xFF, _crcu8(value & 0xFF, crc))


def _expected_seedcrc(seed1, seed2, seed3, size_per_algorithm):
    """Reproduce core_main.c's seedcrc chain in Python.

    This is an independent computation of a value the benchmark computes on the
    core, so comparing them checks arithmetic rather than termination.
    """
    crc = 0
    for value in (seed1, seed2, seed3, size_per_algorithm):
        crc = _crc16(value, crc)
    return crc


def _configured_seedcrc():
    """Expected seedcrc for the seeds and data size this run was built with."""
    seed1 = int(os.environ.get("COREMARK_SEED1", "0"), 0)
    seed2 = int(os.environ.get("COREMARK_SEED2", "0"), 0)
    seed3 = int(os.environ.get("COREMARK_SEED3", "0x66"), 0)
    total = int(
        os.environ.get("COREMARK_TOTAL_DATA_SIZE", DEFAULT_TOTAL_DATA_SIZE), 0
    )
    return _expected_seedcrc(seed1, seed2, seed3, total // NUM_ALGORITHMS)


def _missing_tools():
    missing = []
    for tool in ("m68k-elf-gcc", "m68k-elf-objcopy"):
        if shutil.which(tool) is None:
            missing.append(tool)
    if not BUILD_SCRIPT.exists():
        missing.append(str(BUILD_SCRIPT))
    return missing


def _build_coremark(opt):
    out_dir = REPO_ROOT / "build" / "coremark" / opt
    env = os.environ.copy()
    env["ITERATIONS"] = env.get("COREMARK_ITERATIONS", DEFAULT_ITERATIONS)
    env["TOTAL_DATA_SIZE"] = env.get(
        "COREMARK_TOTAL_DATA_SIZE", DEFAULT_TOTAL_DATA_SIZE
    )

    cmd = [str(BUILD_SCRIPT), "--opt", opt, "--out-dir", str(out_dir)]
    result = subprocess.run(
        cmd,
        cwd=REPO_ROOT,
        env=env,
        capture_output=True,
        text=True,
        check=False,
    )
    if result.returncode != 0:
        raise AssertionError(
            f"CoreMark build failed for {opt}\n"
            f"stdout:\n{result.stdout}\n"
            f"stderr:\n{result.stderr}"
        )
    bin_path = out_dir / f"coremark_{opt}.bin"
    if not bin_path.exists():
        raise AssertionError(f"CoreMark build did not produce {bin_path}")
    return bin_path


def _exception_handler_words(marker):
    return [
        *move(LONG, AN, 7, DN, 6),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 0),
        *imm_long(marker),
        *move_to_abs_long(LONG, DN, 0, CPUTestHarness.RESULT_BASE),
        *move_to_abs_long(LONG, DN, 6, CPUTestHarness.RESULT_BASE + 4),
        *move(LONG, SPECIAL, IMMEDIATE, DN, 7),
        *imm_long(CPUTestHarness.SENTINEL_VAL),
        *move_to_abs_long(LONG, DN, 7, CPUTestHarness.SENTINEL_ADDR),
    ]


def _install_exception_handlers(mem):
    # Keep handlers far away from the loaded image at PROGRAM_BASE.
    handler_base = 0x00060000
    for vec in range(2, 256):
        handler_addr = handler_base + (vec - 2) * 0x20
        marker = 0xBAD00000 | vec
        mem.load_long(vec * 4, handler_addr)
        mem.load_words(handler_addr, _exception_handler_words(marker))


async def _run_coremark_bin(dut, bin_path, max_cycles):
    image = bin_path.read_bytes()
    if not image:
        raise AssertionError(f"CoreMark image is empty: {bin_path}")
    image_lo = CPUTestHarness.PROGRAM_BASE
    image_hi = image_lo + len(image)

    h = CPUTestHarness(dut)
    h.mem.load_long(0x000000, 0x0001FF00)
    h.mem.load_long(0x000004, h.PROGRAM_BASE)
    _install_exception_handlers(h.mem)
    h.mem.load_binary(h.PROGRAM_BASE, image)

    await h.setup(program_words=None)

    monitor_state = {
        "in_cycle": False,
        "cycle_len": 0,
        "rw": 1,
        "size": 0,
    }

    recent_reads = []
    recent_reads_max = 32
    read_hist = {}
    prog_fetch_hist = {}
    max_hist = 256

    for cycle in range(1, max_cycles + 1):
        await RisingEdge(dut.CLK)
        h._check_bus_invariants(monitor_state, max_bus_cycle_cycles=1024)

        try:
            as_n = int(dut.ASn.value)
            rw_n = int(dut.RWn.value)
            addr = int(dut.ADR_OUT.value)
            fc = int(dut.FC_OUT.value)
        except ValueError:
            as_n = 1
            rw_n = 1
            addr = 0
            fc = 7

        if as_n == 0 and rw_n == 1:
            recent_reads.append(addr)
            if len(recent_reads) > recent_reads_max:
                recent_reads.pop(0)
            if addr in read_hist:
                read_hist[addr] += 1
            elif len(read_hist) < max_hist:
                read_hist[addr] = 1

            # Program-space reads (instruction stream) on m68k:
            # FC=6 supervisor program, FC=2 user program.
            if fc in (2, 6):
                if addr in prog_fetch_hist:
                    prog_fetch_hist[addr] += 1
                elif len(prog_fetch_hist) < max_hist:
                    prog_fetch_hist[addr] = 1

        if h.mem.read(h.SENTINEL_ADDR, 4) == h.SENTINEL_VAL:
            fake_ticks = h.mem.read(h.RESULT_BASE, 4)
            validation = h.mem.read(h.RESULT_BASE + 8, 4)
            seedcrc = h.mem.read(h.RESULT_BASE + 12, 4)
            data_size = h.mem.read(h.RESULT_BASE + 16, 4)
            h.cleanup()
            if (fake_ticks & 0xFFFF0000) == 0xBAD00000:
                vec = fake_ticks & 0xFF
                trap_sp = h.mem.read(h.RESULT_BASE + 4, 4)
                stack_words = [
                    h.mem.read((trap_sp + (i * 2)) & 0xFFFFF, 2) for i in range(8)
                ]
                recent = ", ".join(f"0x{a:08X}" for a in recent_reads[-12:])
                stack_txt = " ".join(f"{w:04X}" for w in stack_words)
                return {
                    "status": f"trap:{vec}",
                    "cycles": cycle,
                    "ticks": fake_ticks,
                    "validation": VALIDATION_UNKNOWN,
                    "seedcrc": 0,
                    "diag": (
                        f"exception vector {vec} sp=0x{trap_sp:08X} "
                        f"stack16={stack_txt} recent_reads=[{recent}]"
                    ),
                }
            crcs = {
                "crclist": h.mem.read(h.RESULT_BASE + 20, 4),
                "crcmatrix": h.mem.read(h.RESULT_BASE + 24, 4),
                "crcstate": h.mem.read(h.RESULT_BASE + 28, 4),
                "crcfinal": h.mem.read(h.RESULT_BASE + 32, 4),
            }
            diag = ""
            if validation != VALIDATION_OK:
                diag = (
                    f"seedcrc=0x{seedcrc:04X} size={data_size}/algorithm "
                    + " ".join(f"{k}=0x{v:04X}" for k, v in crcs.items())
                )
            return {
                "status": "ok",
                "cycles": cycle,
                "ticks": fake_ticks,
                "validation": validation,
                "seedcrc": seedcrc,
                "diag": diag,
            }

    h.cleanup()
    top_reads = sorted(read_hist.items(), key=lambda kv: kv[1], reverse=True)[:8]
    read_txt = ", ".join(f"0x{a:08X}:{c}" for a, c in top_reads) or "<none>"
    top_prog = sorted(prog_fetch_hist.items(), key=lambda kv: kv[1], reverse=True)[:8]
    prog_txt = ", ".join(f"0x{a:08X}:{c}" for a, c in top_prog) or "<none>"
    outside_prog = [addr for addr, _ in top_prog if not (image_lo <= addr < image_hi)]
    outside_reads = [addr for addr, _ in top_reads if not (image_lo <= addr < image_hi)]
    outside_txt = ""
    if outside_prog or outside_reads:
        outside_txt = (
            f"; outside image prog_fetches: "
            + (", ".join(f"0x{addr:08X}" for addr in outside_prog) or "<none>")
            + f"; outside image reads: "
            + (", ".join(f"0x{addr:08X}" for addr in outside_reads) or "<none>")
        )
    return {
        "status": "timeout",
        "cycles": max_cycles,
        "ticks": 0,
        "validation": VALIDATION_UNKNOWN,
        "seedcrc": 0,
        "diag": f"top prog_fetch {prog_txt}; top reads {read_txt}{outside_txt}",
    }


def _format_table(rows):
    header = (
        f"{'Opt':<4} {'Size (bytes)':>12} {'Cycles':>12} {'Fake ticks':>12} "
        f"{'Status':>10} {'Validation':>12}"
    )
    sep = f"{'-'*4} {'-'*12} {'-'*12} {'-'*12} {'-'*10} {'-'*12}"
    lines = [header, sep]
    for row in rows:
        lines.append(
            f"{row['opt']:<4} {row['size']:>12} {row['cycles']:>12} "
            f"{row['ticks']:>12} {row['status']:>10} "
            f"{VALIDATION_NAMES.get(row['validation'], row['validation']):>12}"
        )
    for row in rows:
        if row["diag"]:
            lines.append(f"{row['opt']:<4} note: {row['diag']}")
    return "\n".join(lines)


def _required_opts(opts):
    """Which optimization levels must reach the sentinel for this run.

    COREMARK_REQUIRED_OPTS unset or empty -> every level that ran (the strict
    default). "none" -> gate nothing, for deliberately partial runs such as a
    bring-up sweep where a level is known not to complete yet; the table is
    still logged. Otherwise a comma list of levels.
    """
    expr = os.environ.get("COREMARK_REQUIRED_OPTS", "").strip()
    if not expr:
        return list(opts)
    if expr.lower() == "none":
        return []
    required = [opt.strip() for opt in expr.split(",") if opt.strip()]
    unknown = [opt for opt in required if opt not in OPTS]
    if unknown:
        raise AssertionError(
            "COREMARK_REQUIRED_OPTS names unsupported levels: "
            + ", ".join(unknown)
        )
    return required


@cocotb.test()
async def test_coremark_optimizations(dut):
    missing = _missing_tools()
    if missing:
        raise AssertionError("Missing required CoreMark tools: " + ", ".join(missing))

    max_cycles = int(os.environ.get("COREMARK_MAX_CYCLES", "100000000"))
    opt_expr = os.environ.get("COREMARK_OPTS", ",".join(OPTS))
    opts = [opt.strip() for opt in opt_expr.split(",") if opt.strip()]
    required = _required_opts(opts)
    rows = []
    for opt in opts:
        if opt not in OPTS:
            raise AssertionError(f"Unsupported optimization level '{opt}'")
        bin_path = _build_coremark(opt)
        run = await _run_coremark_bin(dut, bin_path, max_cycles=max_cycles)
        rows.append(
            {
                "opt": opt,
                "size": bin_path.stat().st_size,
                "cycles": run["cycles"],
                "ticks": run["ticks"],
                "status": run["status"],
                "validation": run["validation"],
                "seedcrc": run["seedcrc"],
                "diag": run["diag"],
            }
        )

    table = _format_table(rows)
    dut._log.info("CoreMark optimization summary:\n%s", table)

    # The table used to be logged and nothing else, so this target passed with
    # every row 'timeout' -- only tooling/shakeout/run_shakeout.py gated on it.
    expect_seedcrc = _configured_seedcrc()
    checkable = expect_seedcrc in KNOWN_SEEDCRCS
    dut._log.info(
        "CoreMark configuration: expected seedcrc=0x%04X (%s)",
        expect_seedcrc,
        "has known-good CRCs, results are validated"
        if checkable
        else "no known-good CRCs, results cannot be validated",
    )
    failures = []
    for row in rows:
        gated = row["opt"] in required
        if row["status"] != "ok":
            if gated:
                failures.append(
                    f"{row['opt']}: status={row['status']} "
                    f"after {row['cycles']} cycles ({row['diag'] or 'no diagnostics'})"
                )
            else:
                dut._log.warning(
                    "CoreMark %s: status=%s (not in COREMARK_REQUIRED_OPTS, "
                    "not gated)", row["opt"], row["status"]
                )
            continue
        # Reaching the sentinel only proves termination. CoreMark checks its own
        # CRCs against known-good values and reports the verdict through
        # ee_printf, which is a no-op here; core_portme.c now publishes it so a
        # run that computes wrong values cannot pass as a completion.
        if row["validation"] == VALIDATION_ERRORS:
            failures.append(
                f"{row['opt']}: CoreMark reported 'Errors detected' -- the CRC "
                f"of at least one algorithm does not match the known-good "
                f"value, i.e. the core computed wrong results ({row['diag']})"
            )
        elif row["validation"] == VALIDATION_UNKNOWN and gated:
            failures.append(
                f"{row['opt']}: reached the sentinel but published no "
                f"validation verdict. core_main.c always prints one of the "
                f"three end-of-run messages, so either the image predates the "
                f"core_portme.c verdict hook or the run did not complete "
                f"main()"
            )
        elif not checkable:
            # Nothing to gate on: CoreMark has no known-good CRC for this
            # configuration, so it cannot validate itself.
            dut._log.warning(
                "CoreMark %s: seed/data-size combination has no known-good CRC "
                "(expected seedcrc 0x%04X is not one of %s), so results were "
                "not validated. Use COREMARK_TOTAL_DATA_SIZE=2000 (666 per "
                "algorithm) with the default seeds for a checkable run.",
                row["opt"],
                expect_seedcrc,
                ", ".join(f"0x{c:04X}" for c in KNOWN_SEEDCRCS),
            )
        elif gated:
            # The configuration IS checkable, so both of these must hold. The
            # seedcrc comparison is the sharper of the two: it is a value the
            # benchmark computed on the core against the same chain computed in
            # Python, independent of CoreMark's own verdict logic.
            if row["seedcrc"] != expect_seedcrc:
                failures.append(
                    f"{row['opt']}: seedcrc computed on the core is "
                    f"0x{row['seedcrc']:04X}, expected 0x{expect_seedcrc:04X} "
                    f"(crc16 chain over seed1,seed2,seed3,size). The benchmark "
                    f"ran to completion but computed a wrong value: "
                    f"{row['diag']}"
                )
            elif row["validation"] != VALIDATION_OK:
                failures.append(
                    f"{row['opt']}: seedcrc matches but CoreMark did not report "
                    f"'Correct operation validated' "
                    f"(verdict={VALIDATION_NAMES.get(row['validation'])}): "
                    f"{row['diag']}"
                )

    assert not failures, (
        "CoreMark smoke failures (required levels: "
        + (",".join(required) or "<none>")
        + "):\n  "
        + "\n  ".join(failures)
        + "\n"
        + table
    )
