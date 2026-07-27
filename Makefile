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

# The SystemVerilog design -- this is the core under development. Globbed so the
# list cannot go stale as modules are added; the package must be read first.
SV_PKG := $(SV_DIR)/wf68k30L_pkg.sv
SV_SRC := $(filter-out $(SV_PKG),$(sort $(wildcard $(SV_DIR)/wf68k30L_*.sv)))
SV_INC := $(wildcard $(SV_DIR)/*.svh) \
          $(wildcard $(SV_DIR)/wf68k30L_top_sections/*.svh) \
          $(wildcard $(SV_DIR)/wf68k30L_top_sections/*/*.svh)

YOSYS_SV_READ := read_verilog -sv -I. -I$(SV_DIR) $(SV_PKG) $(SV_SRC)

.PHONY: all synth json synth-vhdl-ref clean test-fast test-full test-mmu-random test-csmith-smoke test-csmith-smoke-jump-tables test-coremark-smoke test-jump-tables test-qemu-diff test-qemu-diff-fuzz test-qemu-diff-campaign test-software-torture test-shakeout formal-smoke

all: json

# Generic Yosys synthesis of the SystemVerilog design. Target-neutral: for a real
# part use synth/fpga/run_ecp5_representative.sh, synth/vivado/run_vivado.tcl, or
# synth/synth_check.sh for the LUT/FF regression tripwire.
synth: $(SV_PKG) $(SV_SRC) $(SV_INC)
	yosys -p "$(YOSYS_SV_READ); synth -top $(TOP); stat"

# Produce a JSON netlist (useful for nextpnr or inspection)
json: $(JSON_OUT)

$(JSON_OUT): $(SV_PKG) $(SV_SRC) $(SV_INC)
	mkdir -p $(SYNTH_OUT_DIR)
	yosys -p "$(YOSYS_SV_READ); synth -top $(TOP); write_json $(JSON_OUT); stat"

# The upstream VHDL in vhdl/ is kept as a historical reference only. It is NOT
# the design: it predates the MMU, caches and coprocessor interface, and its own
# README records that it was never verified against the manual, which is why
# VHDL-vs-SV equivalence was retired (see notes on VER-2). This target exists so
# the reference can still be elaborated; it does not build the core.
synth-vhdl-ref: $(ALL_SRC)
	yosys -m ghdl -p \
	  "ghdl $(GHDL_FLAGS) $(ALL_SRC) -e $(TOP); \
	   synth -top $(TOP); \
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
	$(MAKE) -C tb TEST_MODULE=test_cache_instr TOPLEVEL=WF68K30L_TOP
	$(MAKE) -C tb TEST_MODULE=test_cache_ram TOPLEVEL=WF68K30L_SYNC_RAM_1R1W
	$(MAKE) -C tb TEST_MODULE=test_coprocessor_instr TOPLEVEL=WF68K30L_TOP
	$(MAKE) -C tb TEST_MODULE=test_interrupt_probe TOPLEVEL=WF68K30L_TOP
	$(MAKE) -C tb TEST_MODULE=test_trace_addrerr_probe TOPLEVEL=WF68K30L_TOP
	$(MAKE) -C tb TEST_MODULE=test_berr_probe TOPLEVEL=WF68K30L_TOP
	$(MAKE) -C tb TEST_MODULE=test_um_lane_probe TOPLEVEL=WF68K30L_TOP
	$(MAKE) -C tb TEST_MODULE=test_mbit_probe TOPLEVEL=WF68K30L_TOP
	$(MAKE) -C tb TEST_MODULE=test_datapath_probe TOPLEVEL=WF68K30L_TOP
	$(MAKE) -C tb TEST_MODULE=test_decode_probe TOPLEVEL=WF68K30L_TOP
	$(MAKE) -C tb TEST_MODULE=test_instr_bcd TOPLEVEL=WF68K30L_TOP
	$(MAKE) -C tb TEST_MODULE=test_instr_bitfield TOPLEVEL=WF68K30L_TOP
	$(MAKE) -C tb TEST_MODULE=test_instr_atomic TOPLEVEL=WF68K30L_TOP
	$(MAKE) -C tb TEST_MODULE=test_instr_misc TOPLEVEL=WF68K30L_TOP
	$(MAKE) -C tb TEST_MODULE=test_bus_arbitration TOPLEVEL=WF68K30L_TOP
	$(MAKE) -C tb TEST_MODULE=test_bus_arbitration_um_gaps TOPLEVEL=WF68K30L_TOP
	$(MAKE) -C tb TEST_MODULE=test_fullformat_ea_probe TOPLEVEL=WF68K30L_TOP

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
SHAKEOUT_CSMITH_MAX_CYCLES ?= 12000000
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

# Bounded formal checks against the real RTL.
#
# clk2fflogic + `chformal -lower` are REQUIRED: as of Yosys 0.62 an immediate
# assert becomes a $check cell, and write_smt2 emits nothing for $check. Without
# these two passes the SMT2 contains zero assertions and every run reports
# PASSED vacuously. `make formal-selftest` guards against that regressing.
#
# The harnesses derive CLK by toggling it from $global_clock, so one clock cycle
# costs two solver steps: measured, -t 24 reaches 11 posedges and -t 26 reaches
# 12. FORMAL_CYCLES is therefore expressed in clock cycles and converted below,
# so the knob means what it says.
#
# `memory_map` matters as much as the solver here: without it write_smt2 emits
# the register file as an SMT-LIB array (store/select over
# (Array (_ BitVec 3) (_ BitVec 32))), and every solver tried -- yices,
# boolector, bitwuzla -- times out on the exhaustive register-file property.
# Bit-blasting it first makes that property tractable.
#
# Solver choice is a large effect, not a detail. On the register-file property
# at depth 10, measured on this design: bitwuzla 2s, yices 68s; at depth 24
# bitwuzla 26s, yices >180s. bitwuzla (or boolector, its predecessor) is the
# default for that reason; override with SMT_SOLVER=yices if it is unavailable,
# in which case use `make formal-smoke FORMAL_REGFILE=0` to skip the
# register-file property.
# The MMU harnesses instantiate the request-gating and translate modules, so the
# whole cone they depend on has to be read in.
FORMAL_MMU_RTL := $(SV_DIR)/wf68k30L_top_routing_bus_cache.sv \
                  $(SV_DIR)/wf68k30L_top_routing_mmu_translate.sv \
                  $(SV_DIR)/wf68k30L_top_mmu_ptest.sv
FORMAL_LOWER := memory_map; clk2fflogic; chformal -lower
SMT_SOLVER ?= bitwuzla
FORMAL_CYCLES ?= 12
FORMAL_DEPTH := $(shell expr 2 \* $(FORMAL_CYCLES) + 2)
FORMAL_REGFILE ?= 1

.PHONY: formal-selftest formal-solver-check

# The solver is load-bearing, not a preference: yices cannot finish the
# register-file property at this depth. Fail loudly rather than silently
# running something that will time out.
formal-solver-check:
	@command -v $(SMT_SOLVER) >/dev/null 2>&1 || { \
	   echo "formal: SMT_SOLVER='$(SMT_SOLVER)' not found on PATH."; \
	   echo "        bitwuzla or boolector is required for the register-file"; \
	   echo "        property. With yices only, run:"; \
	   echo "            make formal-smoke SMT_SOLVER=yices FORMAL_REGFILE=0"; \
	   exit 1; }
	@case '$(SMT_SOLVER)' in \
	   bitwuzla|boolector) ;; \
	   *) if [ '$(FORMAL_REGFILE)' = '1' ]; then \
	        echo "formal: SMT_SOLVER='$(SMT_SOLVER)' is not bitwuzla/boolector;"; \
	        echo "        the register-file property is expected to time out."; \
	        echo "        Add FORMAL_REGFILE=0 to skip it."; \
	      fi ;; \
	 esac
	@echo "formal: solver=$(SMT_SOLVER) cycles=$(FORMAL_CYCLES) (depth=$(FORMAL_DEPTH) steps) regfile=$(FORMAL_REGFILE)"

formal-smoke: formal-selftest formal-solver-check
	mkdir -p build/formal
	# Hazard tracker: bounded, then proven unbounded by temporal induction.
	yosys -q -p \
	  "read_verilog -formal -sv -I $(SV_DIR) $(SV_DIR)/wf68k30L_data_registers.sv formal/data_regs_hazard_formal.sv; \
	   prep -top data_regs_hazard_formal -flatten; $(FORMAL_LOWER); \
	   write_smt2 -wires build/formal/data_regs_hazard.smt2"
	yosys-smtbmc -s $(SMT_SOLVER) -t $(FORMAL_DEPTH) build/formal/data_regs_hazard.smt2
	yosys-smtbmc -i -s $(SMT_SOLVER) -t $(FORMAL_DEPTH) build/formal/data_regs_hazard.smt2
	# Register-file data integrity: BMC only. The shadow is not inductive on its
	# own (from an arbitrary state it may already disagree with the RTL), so
	# induction fails here without extra invariants; that is a limitation of the
	# property, not a counterexample.
ifeq ($(FORMAL_REGFILE),1)
	yosys -q -p \
	  "read_verilog -formal -sv -DREGFILE_PROPERTY -I $(SV_DIR) $(SV_DIR)/wf68k30L_data_registers.sv formal/data_regs_hazard_formal.sv; \
	   prep -top data_regs_hazard_formal -flatten; $(FORMAL_LOWER); \
	   write_smt2 -wires build/formal/data_regs_regfile.smt2"
	yosys-smtbmc -s $(SMT_SOLVER) -t $(FORMAL_DEPTH) build/formal/data_regs_regfile.smt2
endif
	yosys -q -p \
	  "read_verilog -formal -sv -I $(SV_DIR) $(FORMAL_MMU_RTL) formal/mmu_runtime_gate_formal.sv; \
	   prep -top mmu_runtime_gate_formal -flatten; $(FORMAL_LOWER); \
	   write_smt2 -wires build/formal/mmu_runtime_gate.smt2"
	yosys-smtbmc -s $(SMT_SOLVER) -t $(FORMAL_DEPTH) build/formal/mmu_runtime_gate.smt2
	yosys -q -p \
	  "read_verilog -formal -sv -I $(SV_DIR) $(FORMAL_MMU_RTL) formal/mmu_walk_delay_state_formal.sv; \
	   prep -top mmu_walk_delay_state_formal -flatten; $(FORMAL_LOWER); \
	   write_smt2 -wires build/formal/mmu_walk_delay_state.smt2"
	yosys-smtbmc -s $(SMT_SOLVER) -t $(FORMAL_DEPTH) build/formal/mmu_walk_delay_state.smt2

# Proves the formal flow can actually fail. A trivially false assertion must be
# reported FAILED; if it passes, assertions are being dropped and every other
# formal result in this Makefile is meaningless.
formal-selftest:
	mkdir -p build/formal
	printf 'module formal_selftest;\n logic CLK = 1'"'"'b0;\n always_ff @($$global_clock) CLK <= !CLK;\n always_ff @(posedge CLK) assert(1'"'"'b0);\nendmodule\n' \
	  > build/formal/formal_selftest.sv
	yosys -q -p \
	  "read_verilog -formal -sv build/formal/formal_selftest.sv; \
	   prep -top formal_selftest -flatten; $(FORMAL_LOWER); \
	   write_smt2 -wires build/formal/formal_selftest.smt2"
	@if yosys-smtbmc -s $(SMT_SOLVER) -t 8 build/formal/formal_selftest.smt2 2>&1 | grep -q 'Status: FAILED'; then \
	   echo "formal-selftest: OK (assertions are live)"; \
	 else \
	   echo "formal-selftest: FAILED -- assertions are not reaching the solver;"; \
	   echo "                 every formal result would be a vacuous PASS."; \
	   exit 1; \
	 fi
