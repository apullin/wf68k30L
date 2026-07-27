"""
QEMU-based m68k reference helpers.

Runs qemu-system-m68k (CPU=m68030) on a flat RAM image and either extracts
instruction/state trace entries from QEMU's disassembly log, or -- for programs
far too long to trace instruction by instruction -- runs the image to
completion and reads physical memory back through the QEMU monitor.
"""

from __future__ import annotations

import os
import re
import shutil
import struct
import subprocess
import tempfile
import time
from pathlib import Path


_ANSI_RE = re.compile(r"\x1b\[[0-9;]*[A-Za-z]|[\x00-\x08\x0b\x0c\x0e-\x1f]")
_XP_RE = re.compile(r"^([0-9a-fA-F]{8,16}):\s+((?:0x[0-9a-fA-F]+\s*)+)$")
_PC_RE = re.compile(r"^0x([0-9a-fA-F]+):")
_DA_RE = re.compile(r"^D([0-7])\s*=\s*([0-9A-Fa-f]{8}).*A\1\s*=\s*([0-9A-Fa-f]{8})")
_PC_SR_RE = re.compile(r"^PC\s*=\s*([0-9A-Fa-f]{8})\s+SR\s*=\s*([0-9A-Fa-f]{4})")


def _encode_words(words):
    data = bytearray()
    for w in words:
        data.extend(struct.pack(">H", w & 0xFFFF))
    return data


def _parse_pcs(log_text):
    pcs = []
    for line in log_text.splitlines():
        m = _PC_RE.match(line.strip())
        if m:
            pcs.append(int(m.group(1), 16))
    return pcs


def _parse_state_trace(log_text):
    lines = log_text.splitlines()
    snaps = []
    i = 0
    while i < len(lines):
        line = lines[i].strip()
        if not line.startswith("D0 ="):
            i += 1
            continue

        d_regs = [0] * 8
        a_regs = [0] * 8
        ok = True
        for reg in range(8):
            if i + reg >= len(lines):
                ok = False
                break
            m = _DA_RE.match(lines[i + reg].strip())
            if m is None or int(m.group(1)) != reg:
                ok = False
                break
            d_regs[reg] = int(m.group(2), 16)
            a_regs[reg] = int(m.group(3), 16)
        if not ok:
            i += 1
            continue

        j = i + 8
        while j < len(lines) and not lines[j].strip().startswith("PC ="):
            j += 1
        if j >= len(lines):
            break

        mpc = _PC_SR_RE.match(lines[j].strip())
        if mpc is None:
            i = j + 1
            continue

        snaps.append(
            {
                "pc": int(mpc.group(1), 16),
                "sr": int(mpc.group(2), 16),
                "d": d_regs,
                "a": a_regs,
            }
        )
        i = j + 1
    return snaps


def _parse_xp_dump(text):
    """Parse `xp` monitor output into {address: [word, ...]}."""
    out = {}
    for raw in text.splitlines():
        line = _ANSI_RE.sub("", raw).strip()
        match = _XP_RE.match(line)
        if match is None:
            continue
        addr = int(match.group(1), 16)
        out[addr] = [int(tok, 16) for tok in match.group(2).split()]
    return out


def qemu_m68030_image_memory(
    image_bytes,
    dumps,
    *,
    load_addr=0x00000100,
    stack_pointer=0x0001FF00,
    ram_size_bytes=16 * 1024 * 1024,
    expect=None,
    settle_s=2.0,
    attempts=3,
):
    """Run a flat bare-metal image on QEMU m68030 and read physical memory.

    Used as the reference side for compiled programs (csmith, CoreMark) that are
    far too long to compare instruction by instruction: both implementations run
    the *same* binary, so any difference in the values it computes is a
    difference between the two ISA implementations.

    QEMU's `-M none` does not fetch the reset vectors the cocotb harness
    supplies, so a four-instruction preamble (MOVEA.L #sp,A7 ; JMP load_addr) is
    placed in unused vector space and the loader's PC points at it.

    Args:
        image_bytes: Flat image to load at load_addr.
        dumps: Iterable of (address, word_count) to read after the run.
        load_addr: Load/entry address of the image (must leave room for the
            preamble at 0x40).
        stack_pointer: Value loaded into A7 before entry.
        expect: Optional (address, value) that must be present before the dump
            is trusted, e.g. the sentinel. Retried with a longer settle.
        settle_s: Seconds to let the program run before reading memory.
        attempts: Number of settle attempts (each doubles the wait).

    Returns:
        {address: [word, ...]} for each requested dump.
    """
    qemu = shutil.which("qemu-system-m68k")
    if qemu is None:
        raise RuntimeError("Missing qemu-system-m68k in PATH")
    preamble_end = 0x40 + 12
    if load_addr < preamble_end:
        raise ValueError(
            f"load_addr 0x{load_addr:X} overlaps the entry preamble at 0x40"
        )

    image = bytearray(load_addr + len(image_bytes))
    image[load_addr:] = image_bytes
    image[0x40:0x40 + 12] = (
        struct.pack(">H", 0x2E7C)                    # MOVEA.L #imm32,A7
        + struct.pack(">I", stack_pointer & 0xFFFFFFFF)
        + struct.pack(">H", 0x4EF9)                  # JMP (xxx).L
        + struct.pack(">I", load_addr & 0xFFFFFFFF)
    )

    requests = list(dumps)
    if expect is not None:
        requests = requests + [(expect[0], 1)]
    commands = "".join(
        f"xp /{count}xw 0x{addr:x}\n" for addr, count in requests
    ) + "quit\n"

    with tempfile.TemporaryDirectory(prefix="wf68k_qemu_mem_") as td:
        image_path = Path(td) / "image.bin"
        image_path.write_bytes(bytes(image))
        cmd = [
            qemu,
            "-M",
            "none,memory-backend=ram0",
            "-object",
            f"memory-backend-ram,id=ram0,size={ram_size_bytes}",
            "-accel",
            "tcg",
            "-cpu",
            "m68030",
            "-nodefaults",
            "-nographic",
            "-serial",
            "none",
            "-display",
            "none",
            "-device",
            f"loader,file={image_path},addr=0x0",
            "-device",
            "loader,addr=0x40,cpu-num=0",
            "-monitor",
            "stdio",
        ]

        wait = settle_s
        last = {}
        for _ in range(max(1, attempts)):
            proc = subprocess.Popen(
                cmd,
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
            )
            try:
                time.sleep(wait)
                out, _err = proc.communicate(input=commands, timeout=30)
            finally:
                if proc.poll() is None:
                    proc.kill()
                    proc.wait(timeout=5)
            last = _parse_xp_dump(out)
            if expect is None:
                break
            values = last.get(expect[0] & ~0x3, [])
            if values and values[0] == expect[1]:
                break
            wait *= 2
        else:
            raise RuntimeError(
                f"QEMU run did not reach the expected marker "
                f"0x{expect[1]:08X} at 0x{expect[0]:08X} within "
                f"{wait:.1f}s; got {last}"
            )

        result = {}
        for addr, count in dumps:
            words = last.get(addr)
            if words is None or len(words) < count:
                raise RuntimeError(
                    f"QEMU monitor did not return {count} word(s) at "
                    f"0x{addr:08X}; parsed {last}"
                )
            result[addr] = words[:count]
        return result


def qemu_m68030_pc_trace(
    program_words,
    instruction_limit,
    *,
    program_base=0x00000100,
    ram_size_bytes=16 * 1024 * 1024,
    timeout_s=3.0,
):
    """Return first instruction_limit PCs from QEMU m68030 execution."""
    qemu = shutil.which("qemu-system-m68k")
    if qemu is None:
        raise RuntimeError("Missing qemu-system-m68k in PATH")
    if instruction_limit <= 0:
        raise ValueError("instruction_limit must be > 0")
    if program_base < 0:
        raise ValueError("program_base must be >= 0")

    program_bytes = _encode_words(program_words)
    image_size = max(program_base + len(program_bytes), 0x1000)
    image = bytearray(image_size)
    image[program_base : program_base + len(program_bytes)] = program_bytes

    with tempfile.TemporaryDirectory(prefix="wf68k_qemu_ref_") as td:
        td_path = Path(td)
        image_path = td_path / "image.bin"
        log_path = td_path / "qemu.log"
        image_path.write_bytes(image)

        cmd = [
            qemu,
            "-M",
            "none,memory-backend=ram0",
            "-object",
            f"memory-backend-ram,id=ram0,size={ram_size_bytes}",
            "-accel",
            "tcg,thread=single,one-insn-per-tb=on",
            "-cpu",
            "m68030",
            "-nodefaults",
            "-nographic",
            "-serial",
            "none",
            "-monitor",
            "none",
            "-display",
            "none",
            "-device",
            f"loader,file={image_path},addr=0x0",
            "-device",
            f"loader,addr=0x{program_base:X},cpu-num=0",
            "-d",
            "in_asm,nochain",
            "-D",
            str(log_path),
        ]

        proc = subprocess.Popen(
            cmd,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )

        deadline = time.time() + timeout_s
        pcs = []
        try:
            while time.time() < deadline:
                if log_path.exists():
                    # Instruction-limit is small for differential smoke tests.
                    text = log_path.read_text(errors="ignore")
                    pcs = _parse_pcs(text)
                    if len(pcs) >= instruction_limit:
                        break
                if proc.poll() is not None:
                    break
                time.sleep(0.01)
        finally:
            if proc.poll() is None:
                proc.terminate()
                try:
                    proc.wait(timeout=1.0)
                except subprocess.TimeoutExpired:
                    proc.kill()
                    proc.wait(timeout=1.0)

        if len(pcs) < instruction_limit:
            raise RuntimeError(
                f"QEMU trace too short: got {len(pcs)} instruction PCs, "
                f"need {instruction_limit}"
            )

        return pcs[:instruction_limit]


def qemu_m68030_state_trace(
    program_words,
    instruction_limit,
    *,
    program_base=0x00000100,
    ram_size_bytes=16 * 1024 * 1024,
    timeout_s=3.0,
):
    """Return first instruction_limit CPU state snapshots from QEMU m68030."""
    qemu = shutil.which("qemu-system-m68k")
    if qemu is None:
        raise RuntimeError("Missing qemu-system-m68k in PATH")
    if instruction_limit <= 0:
        raise ValueError("instruction_limit must be > 0")
    if program_base < 0:
        raise ValueError("program_base must be >= 0")

    program_bytes = _encode_words(program_words)
    image_size = max(program_base + len(program_bytes), 0x1000)
    image = bytearray(image_size)
    image[program_base : program_base + len(program_bytes)] = program_bytes

    with tempfile.TemporaryDirectory(prefix="wf68k_qemu_ref_") as td:
        td_path = Path(td)
        image_path = td_path / "image.bin"
        log_path = td_path / "qemu.log"
        image_path.write_bytes(image)

        cmd = [
            qemu,
            "-M",
            "none,memory-backend=ram0",
            "-object",
            f"memory-backend-ram,id=ram0,size={ram_size_bytes}",
            "-accel",
            "tcg,thread=single,one-insn-per-tb=on",
            "-cpu",
            "m68030",
            "-nodefaults",
            "-nographic",
            "-serial",
            "none",
            "-monitor",
            "none",
            "-display",
            "none",
            "-device",
            f"loader,file={image_path},addr=0x0",
            "-device",
            f"loader,addr=0x{program_base:X},cpu-num=0",
            "-d",
            "cpu,in_asm,nochain",
            "-D",
            str(log_path),
        ]

        proc = subprocess.Popen(
            cmd,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )

        deadline = time.time() + timeout_s
        snaps = []
        try:
            while time.time() < deadline:
                if log_path.exists():
                    text = log_path.read_text(errors="ignore")
                    snaps = _parse_state_trace(text)
                    if len(snaps) >= instruction_limit:
                        break
                if proc.poll() is not None:
                    break
                time.sleep(0.01)
        finally:
            if proc.poll() is None:
                proc.terminate()
                try:
                    proc.wait(timeout=1.0)
                except subprocess.TimeoutExpired:
                    proc.kill()
                    proc.wait(timeout=1.0)

        if len(snaps) < instruction_limit:
            raise RuntimeError(
                f"QEMU state trace too short: got {len(snaps)} snapshots, "
                f"need {instruction_limit}"
            )

        return snaps[:instruction_limit]
