#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/../.." && pwd)"
cd "${REPO_ROOT}"

TOP=${TOP:-WF68K30L_TOP}
BUILD_ROOT=${BUILD_ROOT:-build/rep_ecp5_seed_sweep}

YOSYS_BIN=${YOSYS_BIN:-yosys}
NEXTPNR_BIN=${NEXTPNR_BIN:-nextpnr-ecp5}

DEVICE=${DEVICE:-45k}
PACKAGE=${PACKAGE:-CABGA381}
SPEED=${SPEED:-8}
FREQ_MHZ=${FREQ_MHZ:-25}
LPF=${LPF:-synth/constraints/wf68k30l_representative_25mhz.lpf}
SDC=${SDC:-synth/constraints/wf68K30L.sdc}
USE_ABC9=${USE_ABC9:-1}
SEEDS=${SEEDS:-"1 2 3 4 5 6 7 8 9 10"}

mkdir -p "${BUILD_ROOT}"

read -r -a SEED_ARR <<< "${SEEDS}"
if [[ ${#SEED_ARR[@]} -eq 0 ]]; then
  echo "No seeds provided. Set SEEDS to a whitespace-separated list." >&2
  exit 1
fi

JSON_PATH="${BUILD_ROOT}/${TOP}.json"
CSV_PATH="${BUILD_ROOT}/results.csv"

echo "[1/3] Synthesizing ${TOP} once (USE_ABC9=${USE_ABC9})..."
if [[ "${USE_ABC9}" == "1" ]]; then
  SYNTH_CMD="read_verilog -sv -I sv sv/wf68k30L_*.sv; synth_ecp5 -top ${TOP}; write_json ${JSON_PATH}"
else
  SYNTH_CMD="read_verilog -sv -I sv sv/wf68k30L_*.sv; synth_ecp5 -noabc9 -top ${TOP}; write_json ${JSON_PATH}"
fi
"${YOSYS_BIN}" -l "${BUILD_ROOT}/synth.log" -p "${SYNTH_CMD}"

echo "[2/3] Running nextpnr seed sweep (${#SEED_ARR[@]} seeds)..."
echo "seed,fmax_mhz,trellis_comb,trellis_ff" > "${CSV_PATH}"

for seed in "${SEED_ARR[@]}"; do
  run_dir="${BUILD_ROOT}/seed_${seed}"
  mkdir -p "${run_dir}"

  "${NEXTPNR_BIN}" \
    "--${DEVICE}" \
    --package "${PACKAGE}" \
    --speed "${SPEED}" \
    --seed "${seed}" \
    --json "${JSON_PATH}" \
    --textcfg "${run_dir}/${TOP}.config" \
    --lpf "${LPF}" \
    --sdc "${SDC}" \
    --lpf-allow-unconstrained \
    --freq "${FREQ_MHZ}" \
    --timing-allow-fail \
    --report "${run_dir}/report.json" \
    --log "${run_dir}/nextpnr.log" >/dev/null 2>&1

  python3 - "${seed}" "${run_dir}/report.json" "${CSV_PATH}" <<'PY'
import csv
import json
import sys

seed = sys.argv[1]
report_path = sys.argv[2]
csv_path = sys.argv[3]

with open(report_path) as f:
    rpt = json.load(f)

fmax_dict = rpt.get("fmax", {})
if fmax_dict:
    first_key = next(iter(fmax_dict))
    fmax = float(fmax_dict[first_key].get("achieved", 0.0))
else:
    fmax = 0.0

util = rpt.get("utilization", {})
comb = int(util.get("TRELLIS_COMB", {}).get("used", 0))
ff = int(util.get("TRELLIS_FF", {}).get("used", 0))

with open(csv_path, "a", newline="") as f:
    w = csv.writer(f)
    w.writerow([seed, f"{fmax:.3f}", comb, ff])
PY

done

echo "[3/3] Summary"
python3 - "${CSV_PATH}" <<'PY'
import csv
import statistics
import sys

csv_path = sys.argv[1]
rows = []
with open(csv_path) as f:
    reader = csv.DictReader(f)
    for r in reader:
        rows.append({
            "seed": int(r["seed"]),
            "fmax": float(r["fmax_mhz"]),
            "comb": int(r["trellis_comb"]),
            "ff": int(r["trellis_ff"]),
        })

if not rows:
    print("No seed results found.")
    sys.exit(1)

rows.sort(key=lambda r: r["seed"])

print()
print(f"{'Seed':>6}  {'Fmax (MHz)':>11}  {'TRELLIS_COMB':>12}  {'TRELLIS_FF':>10}")
print(f"{'-'*6}  {'-'*11}  {'-'*12}  {'-'*10}")
for r in rows:
    print(f"{r['seed']:>6}  {r['fmax']:>11.3f}  {r['comb']:>12}  {r['ff']:>10}")

fmax_vals = [r["fmax"] for r in rows]
comb_vals = [r["comb"] for r in rows]
ff_vals = [r["ff"] for r in rows]

print()
print(f"Fmax min/med/max: {min(fmax_vals):.3f} / {statistics.median(fmax_vals):.3f} / {max(fmax_vals):.3f} MHz")
print(f"TRELLIS_COMB min/med/max: {min(comb_vals)} / {int(statistics.median(comb_vals))} / {max(comb_vals)}")
print(f"TRELLIS_FF   min/med/max: {min(ff_vals)} / {int(statistics.median(ff_vals))} / {max(ff_vals)}")
print()
print(f"Raw CSV: {csv_path}")
PY
