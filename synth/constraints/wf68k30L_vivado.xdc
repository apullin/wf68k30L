# WF68K30L timing constraints for Vivado.
#
# 25 MHz core clock (40.000 ns period), matching the nextpnr/ECP5 representative
# flow in synth/constraints/wf68K30L.sdc so the two backends are compared
# against the same target rather than against whatever each defaults to.
create_clock -name CLK -period 40.000 [get_ports CLK]

# The core samples these asynchronously and synchronises them internally (the
# reset filter and the two-stage IPL synchroniser), so they carry no timing
# relationship to CLK. Without this Vivado reports meaningless input-path
# failures on them.
set_false_path -from [get_ports {RESET_INn HALT_INn BERRn AVECn IPLn[*] BRn BGACKn CIINn CBACKn STERMn}]

# Report-only design: no board pinout is asserted here. Add an -object_list
# constraint file for a real target.
set_property IOSTANDARD LVCMOS33 [get_ports -filter {DIRECTION != INTERNAL}]
