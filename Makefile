# Makefile for WF68K30L MC68030 core synthesis with GHDL + Yosys
#
# Requirements: ghdl, yosys with ghdl plugin (oss-cad-suite provides both)

# Set GHDL_PREFIX if your GHDL libraries aren't found automatically.
# For oss-cad-suite, this is typically <oss-cad-suite>/lib/ghdl
GHDL_PREFIX ?= $(shell dirname $$(dirname $$(which ghdl)))/lib/ghdl
export GHDL_PREFIX

TOP        := WF68K30L_TOP
GHDL_FLAGS := --ieee=synopsys -fexplicit

SV_DIR   := sv
VHDL_DIR := vhdl
SYNTH_OUT_DIR := synth/out
JSON_OUT := $(SYNTH_OUT_DIR)/$(TOP).json

VHDL_PKG := $(VHDL_DIR)/wf68k30L_pkg.vhd
VHDL_SRC := $(VHDL_DIR)/wf68k30L_address_registers.vhd \
            $(VHDL_DIR)/wf68k30L_data_registers.vhd \
            $(VHDL_DIR)/wf68k30L_alu.vhd \
            $(VHDL_DIR)/wf68k30L_bus_interface.vhd \
            $(VHDL_DIR)/wf68k30L_opcode_decoder.vhd \
            $(VHDL_DIR)/wf68k30L_exception_handler.vhd \
            $(VHDL_DIR)/wf68k30L_control.vhd \
            $(VHDL_DIR)/wf68k30L_top.vhd

ALL_SRC := $(VHDL_PKG) $(VHDL_SRC)

.PHONY: all synth json clean test-fast test-full test-mmu-random test-csmith-smoke test-csmith-smoke-jump-tables test-coremark-smoke test-jump-tables test-qemu-diff test-qemu-diff-fuzz test-qemu-diff-campaign test-software-torture test-shakeout formal-smoke

all: json

# Import VHDL into Yosys via GHDL and run generic synthesis
synth: $(ALL_SRC)
	yosys -m ghdl -p \
	  "ghdl $(GHDL_FLAGS) $(ALL_SRC) -e $(TOP); \
	   synth -top $(TOP); \
	   stat"

# Produce a JSON netlist (useful for nextpnr or inspection)
json: $(JSON_OUT)

$(JSON_OUT): $(ALL_SRC)
	mkdir -p $(SYNTH_OUT_DIR)
	yosys -m ghdl -p \
	  "ghdl $(GHDL_FLAGS) $(ALL_SRC) -e $(TOP); \
	   synth -top $(TOP); \
	   write_json $(JSON_OUT); \
	   stat"

clean:
	rm -f $(JSON_OUT) $(TOP).json *.cf work-obj93.cf

# Quick sanity regression used during RTL iteration.
test-fast:
	$(MAKE) -C tb TEST_MODULE=test_alu TOPLEVEL=WF68K30L_ALU
	$(MAKE) -C tb TEST_MODULE=test_instr_basic TOPLEVEL=WF68K30L_TOP
	$(MAKE) -C tb TEST_MODULE=test_instr_arithmetic TOPLEVEL=WF68K30L_TOP

# Broader stable regression set used before larger refactors and releases.
test-full: test-fast
	$(MAKE) -C tb TEST_MODULE=test_smoke COCOTB_TEST_FILTER=test_smoke_run$$ TOPLEVEL=WF68K30L_TOP
	$(MAKE) -C tb TEST_MODULE=test_jump_tables TOPLEVEL=WF68K30L_TOP
	$(MAKE) -C tb TEST_MODULE=test_software_battery TOPLEVEL=WF68K30L_TOP
	$(MAKE) -C tb TEST_MODULE=test_data_regs TOPLEVEL=WF68K30L_DATA_REGISTERS
	$(MAKE) -C tb TEST_MODULE=test_addr_regs TOPLEVEL=WF68K30L_ADDRESS_REGISTERS
	$(MAKE) -C tb TEST_MODULE=test_divider TOPLEVEL=WF68K30L_ALU
	$(MAKE) -C tb TEST_MODULE=test_addressing_modes TOPLEVEL=WF68K30L_TOP
	$(MAKE) -C tb TEST_MODULE=test_bus_protocol TOPLEVEL=WF68K30L_TOP
	$(MAKE) -C tb TEST_MODULE=test_exceptions TOPLEVEL=WF68K30L_TOP
	$(MAKE) -C tb TEST_MODULE=test_mmu_instr TOPLEVEL=WF68K30L_TOP
	$(MAKE) -C tb TEST_MODULE=test_mmu_random TOPLEVEL=WF68K30L_TOP
	$(MAKE) -C tb TEST_MODULE=test_instr_bit TOPLEVEL=WF68K30L_TOP
	$(MAKE) -C tb TEST_MODULE=test_instr_branch TOPLEVEL=WF68K30L_TOP
	$(MAKE) -C tb TEST_MODULE=test_instr_control TOPLEVEL=WF68K30L_TOP
	$(MAKE) -C tb TEST_MODULE=test_instr_logical TOPLEVEL=WF68K30L_TOP
	$(MAKE) -C tb TEST_MODULE=test_instr_memory TOPLEVEL=WF68K30L_TOP
	$(MAKE) -C tb TEST_MODULE=test_instr_move TOPLEVEL=WF68K30L_TOP
	$(MAKE) -C tb TEST_MODULE=test_instr_muldiv TOPLEVEL=WF68K30L_TOP
	$(MAKE) -C tb TEST_MODULE=test_instr_shift TOPLEVEL=WF68K30L_TOP

# Focused repro suite for switch/jump-table control-flow issues.
test-jump-tables:
	$(MAKE) -C tb TEST_MODULE=test_jump_tables TOPLEVEL=WF68K30L_TOP

# Randomized MMU descriptor-format/FCL campaign.
test-mmu-random:
	$(MAKE) -C tb TEST_MODULE=test_mmu_random TOPLEVEL=WF68K30L_TOP

# Optional software fuzz-style smoke using csmith + m68k-elf cross tools.
test-csmith-smoke:
	$(MAKE) -C tb TEST_MODULE=test_csmith_smoke TOPLEVEL=WF68K30L_TOP

# Optional software fuzz-style smoke with jump tables enabled in compiler output.
test-csmith-smoke-jump-tables:
	CSMITH_CC_EXTRA_FLAGS='' $(MAKE) -C tb TEST_MODULE=test_csmith_smoke TOPLEVEL=WF68K30L_TOP

# CoreMark smoke across -O0/-O1/-O2/-Os using m68k-elf bare-metal flow.
test-coremark-smoke:
	$(MAKE) -C tb TEST_MODULE=test_coremark_smoke TOPLEVEL=WF68K30L_TOP

# Differential smoke against qemu-system-m68k (CPU m68030).
test-qemu-diff:
	$(MAKE) -C tb TEST_MODULE=test_qemu_diff_smoke TOPLEVEL=WF68K30L_TOP

# Seeded randomized differential check vs qemu-system-m68k (CPU m68030).
test-qemu-diff-fuzz:
	$(MAKE) -C tb TEST_MODULE=test_qemu_diff_fuzz TOPLEVEL=WF68K30L_TOP

# Long-form local shakeout defaults (overridable at invocation).
SHAKEOUT_QEMU_SEEDS ?= 1-300
SHAKEOUT_QEMU_OPS ?= 128
SHAKEOUT_CSMITH_SEEDS ?= 1,4-10,12-17,19-23,25-32,34-37,39-59
SHAKEOUT_CSMITH_JUMP_SEEDS ?= 1,4,7,10,13
SHAKEOUT_CSMITH_MAX_CYCLES ?= 800000
SHAKEOUT_COREMARK_OPTS ?= O0,O1,O2,Os
SHAKEOUT_COREMARK_REQUIRED_OPTS ?=
SHAKEOUT_COREMARK_MAX_CYCLES ?= 5000000
SHAKEOUT_COREMARK_ITERATIONS ?= 1
SHAKEOUT_COREMARK_TOTAL_DATA_SIZE ?= 600

# QEMU differential campaign with per-run logs and JSON summary under build/shakeout/.
test-qemu-diff-campaign:
	python3 tooling/shakeout/run_shakeout.py qemu \
	  --qemu-seeds "$(SHAKEOUT_QEMU_SEEDS)" \
	  --qemu-ops "$(SHAKEOUT_QEMU_OPS)"

# Expanded software torture campaign: csmith battery + CoreMark opt matrix.
test-software-torture:
	python3 tooling/shakeout/run_shakeout.py software \
	  --csmith-seeds "$(SHAKEOUT_CSMITH_SEEDS)" \
	  --csmith-jump-seeds "$(SHAKEOUT_CSMITH_JUMP_SEEDS)" \
	  --csmith-max-cycles "$(SHAKEOUT_CSMITH_MAX_CYCLES)" \
	  --coremark-opts "$(SHAKEOUT_COREMARK_OPTS)" \
	  --coremark-required-opts "$(SHAKEOUT_COREMARK_REQUIRED_OPTS)" \
	  --coremark-max-cycles "$(SHAKEOUT_COREMARK_MAX_CYCLES)" \
	  --coremark-iterations "$(SHAKEOUT_COREMARK_ITERATIONS)" \
	  --coremark-total-data-size "$(SHAKEOUT_COREMARK_TOTAL_DATA_SIZE)"

# Full shakeout campaign: QEMU differential + software torture.
test-shakeout:
	python3 tooling/shakeout/run_shakeout.py all \
	  --qemu-seeds "$(SHAKEOUT_QEMU_SEEDS)" \
	  --qemu-ops "$(SHAKEOUT_QEMU_OPS)" \
	  --csmith-seeds "$(SHAKEOUT_CSMITH_SEEDS)" \
	  --csmith-jump-seeds "$(SHAKEOUT_CSMITH_JUMP_SEEDS)" \
	  --csmith-max-cycles "$(SHAKEOUT_CSMITH_MAX_CYCLES)" \
	  --coremark-opts "$(SHAKEOUT_COREMARK_OPTS)" \
	  --coremark-required-opts "$(SHAKEOUT_COREMARK_REQUIRED_OPTS)" \
	  --coremark-max-cycles "$(SHAKEOUT_COREMARK_MAX_CYCLES)" \
	  --coremark-iterations "$(SHAKEOUT_COREMARK_ITERATIONS)" \
	  --coremark-total-data-size "$(SHAKEOUT_COREMARK_TOTAL_DATA_SIZE)"

# Lightweight formal smoke on the data-register hazard tracker.
formal-smoke:
	mkdir -p build/formal
	yosys -q -p \
	  "read_verilog -formal -sv -I $(SV_DIR) $(SV_DIR)/wf68k30L_data_registers.sv formal/data_regs_hazard_formal.sv; \
	   prep -top data_regs_hazard_formal -flatten; \
	   write_smt2 -wires build/formal/data_regs_hazard.smt2"
	yosys-smtbmc -s yices -t 24 build/formal/data_regs_hazard.smt2
	yosys -q -p \
	  "read_verilog -formal -sv formal/mmu_runtime_gate_formal.sv; \
	   prep -top mmu_runtime_gate_formal -flatten; \
	   write_smt2 -wires build/formal/mmu_runtime_gate.smt2"
	yosys-smtbmc -s yices -t 24 build/formal/mmu_runtime_gate.smt2
	yosys -q -p \
	  "read_verilog -formal -sv formal/mmu_walk_delay_state_formal.sv; \
	   prep -top mmu_walk_delay_state_formal -flatten; \
	   write_smt2 -wires build/formal/mmu_walk_delay_state.smt2"
	yosys-smtbmc -s yices -t 24 build/formal/mmu_walk_delay_state.smt2
