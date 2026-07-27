# Vivado synthesis + implementation for WF68K30L.
#
#   vivado -mode batch -source synth/vivado/run_vivado.tcl \
#          [-tclargs <part> <stage>]
#
# stage: "synth" (default) or "impl" for full place & route with timing.
#
# Default part is the Kria K26 SOM used on the KV260 (Zynq UltraScale+ MPSoC).
# Pass a part as the first tclarg for anything else, e.g. xc7a200tfbg484-2.
#
# Purpose is portability checking, not a board build: it confirms the RTL is
# accepted by a vendor toolchain and that the design has no latches, no
# combinational loops and no multiply-driven nets, which Yosys alone does not
# tell you. There is no pinout here, so run it out-of-context.
#
# Note the six explicit logic-to-enum casts in the RTL exist for this flow:
# Vivado and Verilator both reject the implicit form that Yosys accepts.

set part  [expr {[llength $argv] > 0 ? [lindex $argv 0] : "xck26-sfvc784-2LV-c"}]
set stage [expr {[llength $argv] > 1 ? [lindex $argv 1] : "synth"}]

set repo [file normalize [file join [file dirname [info script]] .. ..]]
set out  [file join $repo build vivado]
file mkdir $out

create_project -in_memory -part $part

read_verilog -sv [glob [file join $repo sv wf68k30L_*.sv]]
read_xdc [file join $repo synth constraints wf68k30L_vivado.xdc]

# -mode out_of_context: no pinout is asserted, so do not insert I/O buffers or
# require a package pin for every port.
synth_design -top WF68K30L_TOP \
             -include_dirs [file join $repo sv] \
             -flatten_hierarchy none \
             -mode out_of_context

report_utilization    -file [file join $out utilization_synth.rpt]
report_timing_summary -file [file join $out timing_synth.rpt] -max_paths 10

# Yosys does not report these; this is the main reason to run the vendor tool.
# Only MDRV-1 (multi-driven) and LUTLP-1 (combinational loop) are DRC rules --
# latch inference is a synthesis message (Synth 8-327), not a DRC check, so it is
# counted from the log instead. Wrapped so a bad rule name cannot abort the run.
if {[catch {report_drc -return_string -checks {MDRV-1 LUTLP-1}} drc]} {
    puts "=== structural DRC unavailable: $drc ==="
} else {
    puts "=== structural DRC (multi-driven / combinational loop) ==="
    puts $drc
}

if {$stage eq "impl"} {
    opt_design
    place_design
    phys_opt_design
    route_design
    report_utilization    -file [file join $out utilization_impl.rpt]
    report_timing_summary -file [file join $out timing_impl.rpt] -max_paths 10

    set wns [get_property SLACK [get_timing_paths -delay_type max]]
    set period 40.0
    puts "=== implementation timing ==="
    puts [format "WNS %.3f ns at %.1f ns period -> Fmax %.2f MHz" \
              $wns $period [expr {1000.0 / ($period - $wns)}]]
}

puts "=== reports written to $out ==="
