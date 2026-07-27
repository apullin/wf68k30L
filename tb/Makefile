# Cocotb + Verilator testbench Makefile for WF68K30L
#
# Usage:
#   make                          # Run smoke test (default)
#   make TEST_MODULE=test_smoke   # Run smoke test
#   make TEST_MODULE=test_data_regs TOPLEVEL=WF68K30L_DATA_REGISTERS
#   make TEST_MODULE=test_alu     TOPLEVEL=WF68K30L_ALU

SIM ?= verilator
TOPLEVEL_LANG ?= verilog

# Root of the HDL source tree (one level up from tb/)
SRC_DIR = $(shell cd .. && pwd)
SV_DIR = $(SRC_DIR)/sv

# Verilog sources
# NOTE: wf68k30L_pkg.sv is NOT included because the .sv modules use
# `include "wf68k30L_pkg.svh" (raw typedefs) rather than importing the
# SystemVerilog package. The .svh file is found via -I$(SRC_DIR) and -I$(SV_DIR).
# Every module in sv/ except the package wrapper. Globbed so the list cannot
# go stale as modules are added or removed. wf68k30L_top.sv is placed last so
# the top level is elaborated after its submodules.
VERILOG_SOURCES = \
    $(filter-out $(SV_DIR)/wf68k30L_pkg.sv $(SV_DIR)/wf68k30L_top.sv, \
                 $(wildcard $(SV_DIR)/wf68k30L_*.sv)) \
    $(SV_DIR)/wf68k30L_top.sv

# Default: full top-level smoke test
TOPLEVEL ?= WF68K30L_TOP
TEST_MODULE ?= test_smoke
COCOTB_TEST_MODULES ?= $(TEST_MODULE)

# Keep simulator build products per top-level and isolate the cocotb
# verilator wrapper object (`../verilator.o`) per top-level too.
# Using an extra subdir level prevents ABI mismatches when switching
# TOPLEVEL values without a full clean.
SIM_BUILD ?= sim_build/$(TOPLEVEL)/sim

# Verilator flags
EXTRA_ARGS += -I$(SRC_DIR)
EXTRA_ARGS += -I$(SV_DIR)
EXTRA_ARGS += --timing
EXTRA_ARGS += --public-flat-rw
EXTRA_ARGS += -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC -Wno-CASEINCOMPLETE
EXTRA_ARGS += -Wno-UNSIGNED -Wno-ENUMVALUE -Wno-CMPCONST
EXTRA_ARGS += -Wno-UNOPTFLAT -Wno-BLKANDNBLK -Wno-LATCH
EXTRA_ARGS += -Wno-MULTIDRIVEN -Wno-SYNCASYNCNET -Wno-INITIALDLY

# cocotb Makefile.sim provides the sim targets
include $(shell cocotb-config --makefiles)/Makefile.sim

# Included .svh bodies are passed via -I, so cocotb's dependency tracking (which
# only follows VERILOG_SOURCES) cannot see them: editing one would silently reuse
# the previously compiled model and report a stale PASS. Stamp their contents and
# drop the build directory whenever it changes.
SVH_SOURCES := $(sort $(wildcard $(SV_DIR)/*.svh) \
                      $(wildcard $(SV_DIR)/wf68k30L_top_sections/*.svh) \
                      $(wildcard $(SV_DIR)/wf68k30L_top_sections/*/*.svh))
SVH_STAMP := $(SIM_BUILD)/.svh_stamp

.PHONY: svh-check
svh-check:
	@mkdir -p $(SIM_BUILD)
	@cat $(SVH_SOURCES) 2>/dev/null | shasum > $(SVH_STAMP).new
	@if [ -f $(SVH_STAMP) ] && ! cmp -s $(SVH_STAMP) $(SVH_STAMP).new; then \
	   echo "include (.svh) change detected -- rebuilding $(SIM_BUILD)"; \
	   find $(SIM_BUILD) -mindepth 1 -not -name '.svh_stamp*' -delete; \
	 fi
	@mv -f $(SVH_STAMP).new $(SVH_STAMP)

results.xml: svh-check
