"""
QEMU-based m68k reference trace helper.

Runs qemu-system-m68k (CPU=m68030) on a flat RAM image and extracts
instruction PC trace entries from QEMU's disassembly log.
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
