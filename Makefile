# Makefile for WF68K30L MC68030 core synthesis with GHDL + Yosys
#
# Requirements: ghdl, yosys with ghdl plugin (oss-cad-suite provides both)

# Set GHDL_PREFIX if your GHDL libraries aren't found automatically.
# For oss-cad-suite, this is typically <oss-cad-suite>/lib/ghdl
GHDL_PREFIX ?= $(shell dirname $$(dirname $$(which ghdl)))/lib/ghdl
export GHDL_PREFIX

TOP        := WF68K30L_TOP
GHDL_FLAGS := --ieee=synopsys -fexplicit

VHDL_PKG := wf68k30L_pkg.vhd
VHDL_SRC := wf68k30L_address_registers.vhd \
            wf68k30L_data_registers.vhd \
            wf68k30L_alu.vhd \
            wf68k30L_bus_interface.vhd \
            wf68k30L_opcode_decoder.vhd \
            wf68k30L_exception_handler.vhd \
            wf68k30L_control.vhd \
            wf68k30L_top.vhd

ALL_SRC := $(VHDL_PKG) $(VHDL_SRC)

.PHONY: all synth json clean

all: json

# Import VHDL into Yosys via GHDL and run generic synthesis
synth: $(ALL_SRC)
	yosys -m ghdl -p \
	  "ghdl $(GHDL_FLAGS) $(ALL_SRC) -e $(TOP); \
	   synth -top $(TOP); \
	   stat"

# Produce a JSON netlist (useful for nextpnr or inspection)
json: $(ALL_SRC)
	yosys -m ghdl -p \
	  "ghdl $(GHDL_FLAGS) $(ALL_SRC) -e $(TOP); \
	   synth -top $(TOP); \
	   write_json $(TOP).json; \
	   stat"

clean:
	rm -f $(TOP).json *.cf work-obj93.cf
