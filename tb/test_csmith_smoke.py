"""
Bare-metal Csmith smoke tests for WF68K30L.

Flow per seed:
  1. Generate random C via csmith.
  2. Cross-compile to m68k bare-metal binary.
  3. Load binary at PROGRAM_BASE and run until sentinel.

Environment knobs:
  CSMITH_SEEDS       Seed expression, e.g. "1-10" or "1,4,13,20"
                     (default: 1,4,5,6,7,8,10,12,13,19)
  CSMITH_MAX_CYCLES  Cycle budget per seed (default: 400000)
  CSMITH_CC_EXTRA_FLAGS Extra compiler flags for csmith build script
"""

import os
import shutil
import subprocess
from pathlib import Path

import cocotb

from cpu_harness import CPUTestHarness


REPO_ROOT = Path(__file__).resolve().parents[1]
BUILD_SCRIPT = REPO_ROOT / "tooling" / "csmith" / "build_case.sh"
DEFAULT_SEED_EXPR = "1,4,5,6,7,8,10,12,13,19"
DEFAULT_MAX_CYCLES = 400000


def _missing_tools():
    missing = []
    for tool in ("csmith", "m68k-elf-gcc", "m68k-elf-objcopy"):
        if shutil.which(tool) is None:
            missing.append(tool)
    if not BUILD_SCRIPT.exists():
        missing.append(str(BUILD_SCRIPT))
    return missing


def _require_tools():
    missing = _missing_tools()
    if missing:
        raise AssertionError("Missing required Csmith tools: " + ", ".join(missing))


def _parse_seed_expr(expr):
    seeds = set()
    for token in expr.split(","):
        part = token.strip()
        if not part:
            continue
        if "-" in part:
            start_s, end_s = part.split("-", 1)
            start = int(start_s.strip())
            end = int(end_s.strip())
            if end < start:
                raise ValueError(f"Invalid seed range '{part}'")
            seeds.update(range(start, end + 1))
        else:
            seeds.add(int(part))
    parsed = sorted(seeds)
    if not parsed:
        raise ValueError("No seeds parsed from expression")
    return parsed


def _build_csmith_bin(seed):
    out_dir = REPO_ROOT / "build" / "csmith" / f"seed_{seed}"
    cmd = [str(BUILD_SCRIPT), "--seed", str(seed), "--out-dir", str(out_dir)]
    result = subprocess.run(
        cmd,
        cwd=REPO_ROOT,
        capture_output=True,
        text=True,
        check=False,
    )
    if result.returncode != 0:
        raise AssertionError(
            f"Csmith build failed for seed {seed}\n"
            f"stdout:\n{result.stdout}\n"
            f"stderr:\n{result.stderr}"
        )
    bin_path = out_dir / "program.bin"
    if not bin_path.exists():
        raise AssertionError(f"Csmith build did not produce {bin_path}")
    return bin_path


async def _run_csmith_seed(dut, seed, max_cycles):
    bin_path = _build_csmith_bin(seed)
    image = bin_path.read_bytes()

    h = CPUTestHarness(dut)

    # Use a high stack pointer below RESULT/SENTINEL regions.
    h.mem.load_long(0x000000, 0x0001FF00)
    h.mem.load_long(0x000004, h.PROGRAM_BASE)
    h.mem.load_binary(h.PROGRAM_BASE, image)

    await h.setup(program_words=None)
    found = await h.run_until_sentinel(
        max_cycles=max_cycles,
        check_bus_invariants=True,
        max_bus_cycle_cycles=512,
    )
    h.cleanup()

    assert found, (
        f"Csmith seed {seed} did not reach sentinel within {max_cycles} cycles"
    )


def _register_seed_tests():
    _require_tools()
    seed_expr = os.environ.get("CSMITH_SEEDS", DEFAULT_SEED_EXPR)
    max_cycles = int(os.environ.get("CSMITH_MAX_CYCLES", str(DEFAULT_MAX_CYCLES)))
    seeds = _parse_seed_expr(seed_expr)

    for seed in seeds:
        async def _seed_test(dut, _seed=seed):
            await _run_csmith_seed(dut, seed=_seed, max_cycles=max_cycles)

        test_name = f"test_csmith_seed_{seed:03d}"
        _seed_test.__name__ = test_name
        _seed_test.__qualname__ = test_name
        _seed_test.__doc__ = f"Csmith bare-metal smoke seed {seed}."
        globals()[test_name] = cocotb.test()(_seed_test)


_register_seed_tests()
