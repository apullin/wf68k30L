#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/../.." && pwd)"

SEED=1
OUT_DIR=""
CPU="${CPU:-68030}"

usage() {
  cat <<'EOF'
Usage:
  tooling/csmith/build_case.sh [--seed N] [--out-dir DIR]

Environment overrides:
  CSMITH_BIN      Path to csmith (default: csmith)
  CC_BIN          Path to m68k compiler (default: m68k-elf-gcc)
  OBJCOPY_BIN     Path to objcopy (default: m68k-elf-objcopy)
  CPU             m68k CPU variant (default: 68030)
  CSMITH_EXTRA_FLAGS  Extra flags appended to csmith invocation
  CSMITH_CC_EXTRA_FLAGS Extra C compiler flags (default: -fno-jump-tables)
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --seed)
      SEED="$2"
      shift 2
      ;;
    --out-dir)
      OUT_DIR="$2"
      shift 2
      ;;
    --help|-h)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage >&2
      exit 1
      ;;
  esac
done

if [[ -z "${OUT_DIR}" ]]; then
  OUT_DIR="${REPO_ROOT}/build/csmith/seed_${SEED}"
fi

CSMITH_BIN="${CSMITH_BIN:-csmith}"
CC_BIN="${CC_BIN:-m68k-elf-gcc}"
OBJCOPY_BIN="${OBJCOPY_BIN:-m68k-elf-objcopy}"

if ! command -v "${CSMITH_BIN}" >/dev/null 2>&1; then
  echo "Missing csmith tool: ${CSMITH_BIN}" >&2
  exit 1
fi
if ! command -v "${CC_BIN}" >/dev/null 2>&1; then
  echo "Missing cross compiler: ${CC_BIN}" >&2
  exit 1
fi
if ! command -v "${OBJCOPY_BIN}" >/dev/null 2>&1; then
  echo "Missing objcopy: ${OBJCOPY_BIN}" >&2
  exit 1
fi

RUNTIME_DIR="${SCRIPT_DIR}/runtime"
LINKER_SCRIPT="${RUNTIME_DIR}/link.ld"

mkdir -p "${OUT_DIR}"

GEN_C="${OUT_DIR}/program.c"
GEN_O="${OUT_DIR}/program.o"
CRT0_O="${OUT_DIR}/crt0.o"
RUNTIME_O="${OUT_DIR}/runtime.o"
ELF_PATH="${OUT_DIR}/program.elf"
BIN_PATH="${OUT_DIR}/program.bin"

read -r -a CSMITH_EXTRA_ARR <<< "${CSMITH_EXTRA_FLAGS:-}"
read -r -a CSMITH_CC_EXTRA_ARR <<< "${CSMITH_CC_EXTRA_FLAGS:--fno-jump-tables}"

CSMITH_FLAGS=(
  --seed "${SEED}"
  --nomain
  --no-checksum
  --no-safe-math
  --no-packed-struct
  --no-float
  --no-longlong
  --no-int8
  --no-uint8
)

"${CSMITH_BIN}" "${CSMITH_FLAGS[@]}" "${CSMITH_EXTRA_ARR[@]}" > "${GEN_C}"

cat >> "${GEN_C}" <<'EOF'

/* Harness wrapper injected by WF68K30L Csmith tooling. */
#define WF_SENTINEL_ADDR ((volatile uint32_t*)0x00030000u)
#define WF_SENTINEL_VAL  0xDEADCAFEu

int main(void)
{
    (void)func_1();
    *WF_SENTINEL_ADDR = WF_SENTINEL_VAL;
    for (;;) {
    }
}
EOF

"${CC_BIN}" -mcpu="${CPU}" -Os -ffreestanding -fno-builtin -w \
  "${CSMITH_CC_EXTRA_ARR[@]}" \
  -I"${RUNTIME_DIR}" -c "${GEN_C}" -o "${GEN_O}"
"${CC_BIN}" -mcpu="${CPU}" -Os -ffreestanding -fno-builtin -w \
  "${CSMITH_CC_EXTRA_ARR[@]}" \
  -c "${RUNTIME_DIR}/crt0.S" -o "${CRT0_O}"
"${CC_BIN}" -mcpu="${CPU}" -Os -ffreestanding -fno-builtin -w \
  "${CSMITH_CC_EXTRA_ARR[@]}" \
  -c "${RUNTIME_DIR}/runtime.c" -o "${RUNTIME_O}"

"${CC_BIN}" -mcpu="${CPU}" -nostdlib \
  -Wl,-T,"${LINKER_SCRIPT}" \
  "${CRT0_O}" "${RUNTIME_O}" "${GEN_O}" -lgcc \
  -o "${ELF_PATH}"

"${OBJCOPY_BIN}" -O binary "${ELF_PATH}" "${BIN_PATH}"

echo "Generated ${BIN_PATH}"
