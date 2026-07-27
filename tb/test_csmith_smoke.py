"""
Bare-metal Csmith smoke tests for WF68K30L.

Flow per seed:
  1. Generate random C via csmith, with its checksum enabled.
  2. Cross-compile to m68k bare-metal binary.
  3. Run the same binary on qemu-system-m68k (CPU m68030) and read the
     checksum it computed out of QEMU's RAM.
  4. Load the binary at PROGRAM_BASE, run until sentinel, and require the
     checksum the core computed to match QEMU's.

Step 3/4 are what make this a correctness check. Previously csmith ran with
--no-checksum and the runtime threw the CRC away, so a program that computed
entirely wrong values passed as long as it reached the sentinel.

Environment knobs:
  CSMITH_SEEDS       Seed expression, e.g. "1-10" or "1,4,13,20"
                     (default: 1,4,5,6,7,8,10,12,13,19)
  CSMITH_MAX_CYCLES  Cycle budget per seed (default: 12000000)
  CSMITH_CC_EXTRA_FLAGS Extra compiler flags for csmith build script
  CSMITH_QEMU_SETTLE_S  Seconds to let the QEMU reference run (default: 2.0)
"""

import os
import shutil
import subprocess
from pathlib import Path

import cocotb

from cpu_harness import CPUTestHarness
from qemu_m68k_ref import qemu_m68030_image_memory


REPO_ROOT = Path(__file__).resolve().parents[1]
BUILD_SCRIPT = REPO_ROOT / "tooling" / "csmith" / "build_case.sh"
DEFAULT_SEED_EXPR = "1,4,5,6,7,8,10,12,13,19"
# With the checksum enabled the generated program can no longer be dead-
# stripped: every global is hashed after func_1(), so the programs execute
# orders of magnitude more work than the ~2200 cycles they used to. Measured
# worst case across the default seed set is ~7.4M cycles, so this leaves
# headroom without letting a genuine hang run forever.
DEFAULT_MAX_CYCLES = 12_000_000

# Memory contract with tooling/csmith/runtime/csmith.h.
CRC_ADDR = CPUTestHarness.RESULT_BASE          # computed checksum
CRC_FLAG_ADDR = CPUTestHarness.RESULT_BASE + 4  # magic once published
CRC_MAGIC = 0xC50FC50F


def _missing_tools():
    missing = []
    for tool in ("csmith", "m68k-elf-gcc", "m68k-elf-objcopy", "qemu-system-m68k"):
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


def _qemu_reference_crc(seed, image):
    """Checksum the same image computes under qemu-system-m68k."""
    settle = float(os.environ.get("CSMITH_QEMU_SETTLE_S", "2.0"))
    dump = qemu_m68030_image_memory(
        image,
        [(CRC_ADDR, 2)],
        load_addr=CPUTestHarness.PROGRAM_BASE,
        stack_pointer=0x0001FF00,
        expect=(CPUTestHarness.SENTINEL_ADDR, CPUTestHarness.SENTINEL_VAL),
        settle_s=settle,
    )
    crc, flag = dump[CRC_ADDR]
    assert flag == CRC_MAGIC, (
        f"Csmith seed {seed}: the QEMU reference run reached the sentinel but "
        f"did not publish a checksum (flag=0x{flag:08X}). The image was built "
        f"without csmith's checksum, or the runtime contract in "
        f"tooling/csmith/runtime/csmith.h changed."
    )
    return crc


async def _run_csmith_seed(dut, seed, max_cycles):
    bin_path = _build_csmith_bin(seed)
    image = bin_path.read_bytes()
    expected_crc = _qemu_reference_crc(seed, image)

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
    got_crc = h.mem.read(CRC_ADDR, 4)
    got_flag = h.mem.read(CRC_FLAG_ADDR, 4)
    h.cleanup()

    assert found, (
        f"Csmith seed {seed} did not reach sentinel within {max_cycles} cycles"
    )
    assert got_flag == CRC_MAGIC, (
        f"Csmith seed {seed}: sentinel reached but no checksum was published "
        f"(flag=0x{got_flag:08X}); the run did not go through "
        f"platform_main_end()"
    )
    assert got_crc == expected_crc, (
        f"Csmith seed {seed}: checksum mismatch on the identical binary -- "
        f"core computed 0x{got_crc:08X}, qemu-system-m68k (m68030) computed "
        f"0x{expected_crc:08X}. The program terminated, so this is a wrong "
        f"computed result rather than a hang. Image: {bin_path}"
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
