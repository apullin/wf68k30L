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

# Verilog sources
# NOTE: wf68k30L_pkg.sv is NOT included because the .sv modules use
# `include "wf68k30L_pkg.svh" (raw typedefs) rather than importing the
# SystemVerilog package. The .svh file is found via -I$(SRC_DIR).
VERILOG_SOURCES = \
    $(SRC_DIR)/wf68k30L_address_registers.sv \
    $(SRC_DIR)/wf68k30L_data_registers.sv \
    $(SRC_DIR)/wf68k30L_alu.sv \
    $(SRC_DIR)/wf68k30L_bus_interface.sv \
    $(SRC_DIR)/wf68k30L_opcode_decoder.sv \
    $(SRC_DIR)/wf68k30L_exception_handler.sv \
    $(SRC_DIR)/wf68k30L_operand_mux.sv \
    $(SRC_DIR)/wf68k30L_shifter.sv \
    $(SRC_DIR)/wf68k30L_divider.sv \
    $(SRC_DIR)/wf68k30L_control.sv \
    $(SRC_DIR)/wf68k30L_ctrl_comb.sv \
    $(SRC_DIR)/wf68k30L_ctrl_exec_dec.sv \
    $(SRC_DIR)/wf68k30L_ctrl_fetch_dec.sv \
    $(SRC_DIR)/wf68k30L_ctrl_fetch_other.sv \
    $(SRC_DIR)/wf68k30L_ctrl_fetch_start.sv \
    $(SRC_DIR)/wf68k30L_ctrl_movem.sv \
    $(SRC_DIR)/wf68k30L_ctrl_regsel.sv \
    $(SRC_DIR)/wf68k30L_top.sv

# Default: full top-level smoke test
TOPLEVEL ?= WF68K30L_TOP
MODULE ?= $(TEST_MODULE)
TEST_MODULE ?= test_smoke

# Verilator flags
EXTRA_ARGS += -I$(SRC_DIR)
EXTRA_ARGS += --timing
EXTRA_ARGS += --public-flat-rw
EXTRA_ARGS += -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC -Wno-CASEINCOMPLETE
EXTRA_ARGS += -Wno-UNSIGNED -Wno-ENUMVALUE -Wno-CMPCONST
EXTRA_ARGS += -Wno-UNOPTFLAT -Wno-BLKANDNBLK -Wno-LATCH
EXTRA_ARGS += -Wno-MULTIDRIVEN -Wno-SYNCASYNCNET -Wno-INITIALDLY

# cocotb Makefile.sim provides the sim targets
include $(shell cocotb-config --makefiles)/Makefile.sim
