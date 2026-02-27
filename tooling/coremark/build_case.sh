#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/../.." && pwd)"

OPT="O2"
OUT_DIR=""
CPU="${CPU:-68030}"
ITERATIONS="${ITERATIONS:-1}"
TOTAL_DATA_SIZE="${TOTAL_DATA_SIZE:-2000}"
COREMARK_SEED1="${COREMARK_SEED1:-0}"
COREMARK_SEED2="${COREMARK_SEED2:-0}"
COREMARK_SEED3="${COREMARK_SEED3:-0x66}"
COREMARK_EXECS_MASK="${COREMARK_EXECS_MASK:-ID_LIST|ID_MATRIX|ID_STATE}"
COREMARK_LIST_ITEMS="${COREMARK_LIST_ITEMS:-}"
COREMARK_EXTRA_CFLAGS="${COREMARK_EXTRA_CFLAGS:-}"

usage() {
  cat <<'EOF'
Usage:
  tooling/coremark/build_case.sh --opt O0|O1|O2|Os [--out-dir DIR]

Environment overrides:
  CC_BIN           Path to m68k compiler (default: m68k-elf-gcc)
  OBJCOPY_BIN      Path to objcopy (default: m68k-elf-objcopy)
  CPU              m68k CPU variant (default: 68030)
  ITERATIONS       CoreMark iterations (default: 1)
  TOTAL_DATA_SIZE  CoreMark total data size (default: 2000)
  COREMARK_SEED1   Seed1 override (default: 0)
  COREMARK_SEED2   Seed2 override (default: 0)
  COREMARK_SEED3   Seed3/list search count override (default: 0x66)
  COREMARK_EXECS_MASK Algorithm bitmask: ID_LIST/ID_MATRIX/ID_STATE combos (default: ID_LIST|ID_MATRIX|ID_STATE)
  COREMARK_LIST_ITEMS Cap linked-list items for debug (optional)
  COREMARK_EXTRA_CFLAGS Additional compiler flags (default: none)
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --opt)
      OPT="$2"
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

case "${OPT}" in
  O0|O1|O2|Os) ;;
  *)
    echo "Invalid --opt '${OPT}'. Expected one of: O0 O1 O2 Os" >&2
    exit 1
    ;;
esac

if [[ -z "${OUT_DIR}" ]]; then
  OUT_DIR="${REPO_ROOT}/build/coremark/${OPT}"
fi

CC_BIN="${CC_BIN:-m68k-elf-gcc}"
OBJCOPY_BIN="${OBJCOPY_BIN:-m68k-elf-objcopy}"

if ! command -v "${CC_BIN}" >/dev/null 2>&1; then
  echo "Missing cross compiler: ${CC_BIN}" >&2
  exit 1
fi
if ! command -v "${OBJCOPY_BIN}" >/dev/null 2>&1; then
  echo "Missing objcopy: ${OBJCOPY_BIN}" >&2
  exit 1
fi

SRC_DIR="${SCRIPT_DIR}/src"
PORT_DIR="${SCRIPT_DIR}/port"
RUNTIME_DIR="${SCRIPT_DIR}/runtime"

mkdir -p "${OUT_DIR}"

COMMON_CFLAGS=(
  "-mcpu=${CPU}"
  "-${OPT}"
  -ffreestanding
  -fno-builtin
  -fomit-frame-pointer
  -fno-unwind-tables
  -fno-asynchronous-unwind-tables
  -w
  -I"${SRC_DIR}"
  -I"${PORT_DIR}"
  -DITERATIONS="${ITERATIONS}"
  -DTOTAL_DATA_SIZE="${TOTAL_DATA_SIZE}"
  -DCOREMARK_SEED1="${COREMARK_SEED1}"
  -DCOREMARK_SEED2="${COREMARK_SEED2}"
  -DCOREMARK_SEED3="${COREMARK_SEED3}"
  -DCOREMARK_EXECS_MASK="${COREMARK_EXECS_MASK}"
  -DPERFORMANCE_RUN=1
)

if [[ -n "${COREMARK_LIST_ITEMS}" ]]; then
  COMMON_CFLAGS+=("-DCOREMARK_LIST_ITEMS=${COREMARK_LIST_ITEMS}")
fi

if [[ -n "${COREMARK_EXTRA_CFLAGS}" ]]; then
  read -r -a EXTRA_FLAGS_ARR <<< "${COREMARK_EXTRA_CFLAGS}"
  COMMON_CFLAGS+=("${EXTRA_FLAGS_ARR[@]}")
fi

CORE_SRCS=(
  core_list_join.c
  core_main.c
  core_matrix.c
  core_state.c
  core_util.c
)

OBJS=()
for src in "${CORE_SRCS[@]}"; do
  in_path="${SRC_DIR}/${src}"
  out_path="${OUT_DIR}/${src%.c}.o"
  "${CC_BIN}" "${COMMON_CFLAGS[@]}" -c "${in_path}" -o "${out_path}"
  OBJS+=("${out_path}")
done

PORT_O="${OUT_DIR}/core_portme.o"
CRT0_O="${OUT_DIR}/crt0.o"
RUNTIME_O="${OUT_DIR}/runtime.o"
ELF_PATH="${OUT_DIR}/coremark_${OPT}.elf"
BIN_PATH="${OUT_DIR}/coremark_${OPT}.bin"

"${CC_BIN}" "${COMMON_CFLAGS[@]}" -c "${PORT_DIR}/core_portme.c" -o "${PORT_O}"
"${CC_BIN}" "${COMMON_CFLAGS[@]}" -c "${RUNTIME_DIR}/crt0.S" -o "${CRT0_O}"
"${CC_BIN}" "${COMMON_CFLAGS[@]}" -c "${RUNTIME_DIR}/runtime.c" -o "${RUNTIME_O}"

"${CC_BIN}" "-mcpu=${CPU}" -nostdlib \
  -Wl,-T,"${RUNTIME_DIR}/link.ld" \
  "${CRT0_O}" "${RUNTIME_O}" "${PORT_O}" "${OBJS[@]}" -lgcc \
  -o "${ELF_PATH}"

"${OBJCOPY_BIN}" -O binary "${ELF_PATH}" "${BIN_PATH}"

echo "Generated ${BIN_PATH}"
