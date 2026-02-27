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
                    "diag": (
                        f"exception vector {vec} sp=0x{trap_sp:08X} "
                        f"stack16={stack_txt} recent_reads=[{recent}]"
                    ),
                }
            return {"status": "ok", "cycles": cycle, "ticks": fake_ticks, "diag": ""}

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
        "diag": f"top prog_fetch {prog_txt}; top reads {read_txt}{outside_txt}",
    }


def _format_table(rows):
    header = (
        f"{'Opt':<4} {'Size (bytes)':>12} {'Cycles':>12} {'Fake ticks':>12} "
        f"{'Status':>10}"
    )
    sep = f"{'-'*4} {'-'*12} {'-'*12} {'-'*12} {'-'*10}"
    lines = [header, sep]
    for row in rows:
        lines.append(
            f"{row['opt']:<4} {row['size']:>12} {row['cycles']:>12} "
            f"{row['ticks']:>12} {row['status']:>10}"
        )
    for row in rows:
        if row["diag"]:
            lines.append(f"{row['opt']:<4} note: {row['diag']}")
    return "\n".join(lines)


@cocotb.test()
async def test_coremark_optimizations(dut):
    missing = _missing_tools()
    if missing:
        raise AssertionError("Missing required CoreMark tools: " + ", ".join(missing))

    max_cycles = int(os.environ.get("COREMARK_MAX_CYCLES", "100000000"))
    opt_expr = os.environ.get("COREMARK_OPTS", ",".join(OPTS))
    opts = [opt.strip() for opt in opt_expr.split(",") if opt.strip()]
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
                "diag": run["diag"],
            }
        )

    dut._log.info("CoreMark optimization summary:\n%s", _format_table(rows))
