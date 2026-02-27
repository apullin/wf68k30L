#!/usr/bin/env python3
"""
WF68K30L local shakeout campaign runner.

Runs long-form local validation campaigns and writes reproducible logs and a
machine-readable summary under build/shakeout/.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import shlex
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_OUT_ROOT = REPO_ROOT / "build" / "shakeout"
_COCOTB_COUNTS_RE = re.compile(r"TESTS=(\d+)\s+PASS=(\d+)\s+FAIL=(\d+)\s+SKIP=(\d+)")
_COREMARK_ROW_RE = re.compile(r"^(O0|O1|O2|Os)\s+(\d+)\s+(\d+)\s+(\d+)\s+(\S+)\s*$")


def _timestamp():
    return datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")


def _merge_env(extra):
    env = os.environ.copy()
    env.update(extra)
    return env


def _quote_cmd(cmd):
    return " ".join(shlex.quote(part) for part in cmd)


def _parse_cocotb_counts(text):
    matches = _COCOTB_COUNTS_RE.findall(text)
    if not matches:
        return None
    tests, passed, failed, skipped = matches[-1]
    return {
        "tests": int(tests),
        "pass": int(passed),
        "fail": int(failed),
        "skip": int(skipped),
    }


def _parse_coremark_rows(text):
    rows = []
    for line in text.splitlines():
        match = _COREMARK_ROW_RE.match(line.strip())
        if match:
            rows.append(
                {
                    "opt": match.group(1),
                    "size_bytes": int(match.group(2)),
                    "cycles": int(match.group(3)),
                    "ticks": int(match.group(4)),
                    "status": match.group(5),
                }
            )
    return rows


def _extract_qemu_mismatch_excerpt(text, max_lines=12):
    idx = text.find("QEMU diff mismatch")
    if idx < 0:
        return ""
    return "\n".join(text[idx:].splitlines()[:max_lines])


def _parse_opt_list(expr):
    return [part.strip() for part in expr.split(",") if part.strip()]


def _run_command(label, cmd, log_path, env):
    log_path.parent.mkdir(parents=True, exist_ok=True)
    print(f"[{label}] $ {_quote_cmd(cmd)}")
    start = time.time()
    proc = subprocess.run(
        cmd,
        cwd=REPO_ROOT,
        env=env,
        text=True,
        capture_output=True,
        check=False,
    )
    elapsed_s = time.time() - start

    out_text = proc.stdout or ""
    err_text = proc.stderr or ""
    with log_path.open("w", encoding="utf-8", errors="replace") as f:
        f.write(out_text)
        if err_text:
            f.write("\n--- STDERR ---\n")
            f.write(err_text)

    print(
        f"[{label}] rc={proc.returncode} elapsed={elapsed_s:.1f}s "
        f"log={log_path.relative_to(REPO_ROOT)}"
    )
    return {
        "label": label,
        "cmd": cmd,
        "returncode": proc.returncode,
        "elapsed_s": elapsed_s,
        "output": out_text + ("\n" + err_text if err_text else ""),
        "log": str(log_path),
    }


def _run_qemu_campaign(args, run_dir):
    env = _merge_env(
        {
            "QEMU_DIFF_SEEDS": args.qemu_seeds,
            "QEMU_DIFF_OPS": str(args.qemu_ops),
        }
    )
    result = _run_command(
        "qemu-diff",
        ["make", "-s", "test-qemu-diff-fuzz"],
        run_dir / "qemu_diff.log",
        env,
    )
    counts = _parse_cocotb_counts(result["output"])
    mismatch = _extract_qemu_mismatch_excerpt(result["output"])
    return {
        "ok": result["returncode"] == 0,
        "seed_expr": args.qemu_seeds,
        "ops": args.qemu_ops,
        "elapsed_s": result["elapsed_s"],
        "counts": counts,
        "log": result["log"],
        "mismatch_excerpt": mismatch,
        "returncode": result["returncode"],
    }


def _run_software_campaign(args, run_dir):
    out = {}

    csmith_env = _merge_env(
        {
            "CSMITH_SEEDS": args.csmith_seeds,
            "CSMITH_MAX_CYCLES": str(args.csmith_max_cycles),
        }
    )
    csmith_result = _run_command(
        "csmith",
        ["make", "-s", "test-csmith-smoke"],
        run_dir / "csmith.log",
        csmith_env,
    )
    out["csmith"] = {
        "ok": csmith_result["returncode"] == 0,
        "seed_expr": args.csmith_seeds,
        "max_cycles": args.csmith_max_cycles,
        "elapsed_s": csmith_result["elapsed_s"],
        "counts": _parse_cocotb_counts(csmith_result["output"]),
        "log": csmith_result["log"],
        "returncode": csmith_result["returncode"],
    }

    if not out["csmith"]["ok"] and not args.continue_on_fail:
        out["coremark"] = {
            "ok": False,
            "skipped": True,
            "reason": "csmith failed and continue-on-fail is disabled",
        }
        out["ok"] = False
        return out

    coremark_env = _merge_env(
        {
            "COREMARK_OPTS": args.coremark_opts,
            "COREMARK_MAX_CYCLES": str(args.coremark_max_cycles),
            "COREMARK_ITERATIONS": str(args.coremark_iterations),
            "COREMARK_TOTAL_DATA_SIZE": str(args.coremark_total_data_size),
        }
    )
    coremark_result = _run_command(
        "coremark",
        ["make", "-s", "test-coremark-smoke"],
        run_dir / "coremark.log",
        coremark_env,
    )
    rows = _parse_coremark_rows(coremark_result["output"])
    required_opts = _parse_opt_list(args.coremark_required_opts)
    if not required_opts:
        required_opts = _parse_opt_list(args.coremark_opts)
    non_ok_rows = [row for row in rows if row["status"] != "ok"]
    non_ok_required = [row for row in rows if row["opt"] in required_opts and row["status"] != "ok"]
    coremark_ok = (
        coremark_result["returncode"] == 0 and bool(rows) and not non_ok_required
    )
    out["coremark"] = {
        "ok": coremark_ok,
        "opts": args.coremark_opts,
        "max_cycles": args.coremark_max_cycles,
        "iterations": args.coremark_iterations,
        "total_data_size": args.coremark_total_data_size,
        "elapsed_s": coremark_result["elapsed_s"],
        "rows": rows,
        "non_ok_rows": non_ok_rows,
        "required_opts": required_opts,
        "non_ok_required": non_ok_required,
        "log": coremark_result["log"],
        "returncode": coremark_result["returncode"],
    }
    out["ok"] = out["csmith"]["ok"] and out["coremark"]["ok"]
    return out


def _print_summary(summary):
    print()
    print("Shakeout summary")
    print("================")
    print(f"Run dir: {summary['run_dir']}")
    print(f"Mode:    {summary['mode']}")

    qemu = summary.get("qemu_diff")
    if qemu is not None:
        counts = qemu.get("counts") or {}
        print(
            f"QEMU diff: {'PASS' if qemu['ok'] else 'FAIL'}  "
            f"seeds={qemu['seed_expr']} ops={qemu['ops']} "
            f"time={qemu['elapsed_s']:.1f}s "
            f"tests={counts.get('tests', 'n/a')} "
            f"pass={counts.get('pass', 'n/a')} "
            f"fail={counts.get('fail', 'n/a')}"
        )
        if qemu.get("mismatch_excerpt"):
            print("QEMU mismatch excerpt:")
            print(qemu["mismatch_excerpt"])

    software = summary.get("software")
    if software is not None:
        csmith = software.get("csmith", {})
        c_counts = csmith.get("counts") or {}
        print(
            f"CSmith:   {'PASS' if csmith.get('ok') else 'FAIL'}  "
            f"seeds={csmith.get('seed_expr')} "
            f"time={csmith.get('elapsed_s', 0.0):.1f}s "
            f"tests={c_counts.get('tests', 'n/a')} "
            f"pass={c_counts.get('pass', 'n/a')} "
            f"fail={c_counts.get('fail', 'n/a')}"
        )

        coremark = software.get("coremark", {})
        if coremark.get("skipped"):
            print(f"CoreMark: skipped ({coremark.get('reason', 'unknown')})")
        else:
            rows = coremark.get("rows", [])
            statuses = ",".join(f"{r['opt']}={r['status']}" for r in rows) or "n/a"
            print(
                f"CoreMark: {'PASS' if coremark.get('ok') else 'FAIL'}  "
                f"opts={coremark.get('opts')} "
                f"time={coremark.get('elapsed_s', 0.0):.1f}s "
                f"status={statuses}"
            )
            req = coremark.get("required_opts", [])
            if req:
                print(f"  required opts: {','.join(req)}")
            for row in coremark.get("non_ok_required", []):
                print(
                    f"  required non-ok: opt={row['opt']} status={row['status']} "
                    f"cycles={row['cycles']}"
                )
            for row in coremark.get("non_ok_rows", []):
                print(
                    f"  non-ok: opt={row['opt']} status={row['status']} "
                    f"cycles={row['cycles']}"
                )

    print(f"Overall: {'PASS' if summary['ok'] else 'FAIL'}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "mode",
        nargs="?",
        choices=("qemu", "software", "all"),
        default="all",
        help="Campaign mode to run",
    )
    parser.add_argument(
        "--out-dir",
        default="",
        help="Output directory (default: build/shakeout/<timestamp>)",
    )
    parser.add_argument(
        "--continue-on-fail",
        action="store_true",
        help="Continue to later stages after an earlier failure",
    )

    parser.add_argument("--qemu-seeds", default="1-300")
    parser.add_argument("--qemu-ops", type=int, default=128)

    parser.add_argument(
        "--csmith-seeds",
        default="1,4-10,12-17,19-23,25-32,34-37,39-59",
    )
    parser.add_argument("--csmith-max-cycles", type=int, default=800000)

    parser.add_argument("--coremark-opts", default="O0,O1,O2,Os")
    parser.add_argument(
        "--coremark-required-opts",
        default="",
        help=(
            "Comma list of CoreMark opts that must be status=ok for pass. "
            "Default: all opts listed in --coremark-opts."
        ),
    )
    parser.add_argument("--coremark-max-cycles", type=int, default=5000000)
    parser.add_argument("--coremark-iterations", type=int, default=1)
    parser.add_argument("--coremark-total-data-size", type=int, default=600)

    args = parser.parse_args()

    run_dir = Path(args.out_dir) if args.out_dir else (DEFAULT_OUT_ROOT / _timestamp())
    run_dir.mkdir(parents=True, exist_ok=True)

    summary = {
        "mode": args.mode,
        "started_utc": datetime.now(timezone.utc).isoformat(),
        "repo_root": str(REPO_ROOT),
        "run_dir": str(run_dir),
        "ok": True,
    }

    if args.mode in ("qemu", "all"):
        summary["qemu_diff"] = _run_qemu_campaign(args, run_dir)
        summary["ok"] = summary["ok"] and summary["qemu_diff"]["ok"]
        if not summary["ok"] and not args.continue_on_fail and args.mode == "all":
            summary["software"] = {
                "ok": False,
                "skipped": True,
                "reason": "qemu_diff failed and continue-on-fail is disabled",
            }
    if args.mode in ("software", "all"):
        should_run_software = (
            args.mode == "software"
            or args.continue_on_fail
            or summary.get("qemu_diff", {}).get("ok", True)
        )
        if should_run_software:
            summary["software"] = _run_software_campaign(args, run_dir)
            summary["ok"] = summary["ok"] and summary["software"]["ok"]

    summary["finished_utc"] = datetime.now(timezone.utc).isoformat()
    summary_path = run_dir / "summary.json"
    with summary_path.open("w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2, sort_keys=True)
        f.write("\n")

    _print_summary(summary)
    print(f"Summary JSON: {summary_path.relative_to(REPO_ROOT)}")
    return 0 if summary["ok"] else 1


if __name__ == "__main__":
    sys.exit(main())
