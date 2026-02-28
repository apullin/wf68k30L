# WF68K30L representative timing constraints for nextpnr.
#
# 25 MHz core clock (40.000 ns period).
create_clock -name CLK -period 40.000 [get_ports {CLK}]

# NOTE:
# nextpnr currently parses `set_false_path` but reports it as non-functional
# ("does not do anything(yet)"). To keep timing logs clean and deterministic,
# this default SDC keeps only the real, active clock constraint.
#
# Keep asynchronous-interface path exceptions in board/tool-specific timing
# flows once the backend supports active false-path handling.
