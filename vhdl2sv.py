#!/usr/bin/env python3
"""Mechanical VHDL-to-SystemVerilog converter for the WF68K30L project.
Handles the bulk of syntax transformation; manual fixup required afterward."""

import re
import sys

def convert_vhdl_to_sv(vhdl_text, module_name):
    lines = vhdl_text.split('\n')
    sv_lines = []

    # State tracking
    in_entity = False
    in_architecture = False
    in_port = False
    in_signal_decl = False
    in_process = False
    process_is_clocked = False
    in_case = False
    paren_depth = 0

    # Collect ports and signals separately
    ports = []
    signals = []
    body_lines = []
    header_comments = []

    # Phase 1: Parse structure
    i = 0
    phase = 'header'  # header, entity, arch_decl, arch_body

    while i < len(lines):
        line = lines[i]
        stripped = line.strip()

        # Convert comments
        line = re.sub(r'----(.*)----', r'//\1//', line)
        line = re.sub(r'--(.*)$', r'//\1', line)

        if phase == 'header':
            if re.match(r'^\s*entity\s+', stripped, re.I):
                phase = 'entity'
                header_comments.append(line)
                i += 1
                continue
            elif re.match(r'^\s*(library|use)\s+', stripped, re.I):
                # Skip library/use - replaced by import
                i += 1
                continue
            header_comments.append(line)
            i += 1
            continue

        elif phase == 'entity':
            if re.match(r'^\s*generic\s*\(', stripped, re.I):
                # Handle generic
                phase = 'generic'
                i += 1
                continue
            elif re.match(r'^\s*port\s*\(', stripped, re.I):
                phase = 'port'
                i += 1
                continue
            elif re.match(r'^\s*end\s+(entity|' + module_name + r')', stripped, re.I):
                phase = 'post_entity'
                i += 1
                continue
            i += 1
            continue

        elif phase == 'generic':
            if re.match(r'^\s*port\s*\(', stripped, re.I):
                phase = 'port'
                i += 1
                continue
            # Parse generic parameters
            m = re.match(r'\s*(\w+)\s*:\s*(\w+)\s*:=\s*(.*?)[\);]', stripped, re.I)
            if m:
                name, typ, default = m.groups()
                default = default.strip().rstrip(';').rstrip(')')
                if typ.lower() == 'boolean':
                    default = '1' if default.lower() == 'true' else '0'
                    typ = 'logic'
                    # Actually for generic boolean, use int
                    ports.append(f'    parameter {name} = {default}')
                else:
                    ports.append(f'    parameter {name} = {default}')
            i += 1
            continue

        elif phase == 'port':
            if re.match(r'^\s*\)\s*;', stripped):
                phase = 'post_entity'
                i += 1
                continue
            # Parse port declarations
            port_line = convert_port(stripped, line)
            if port_line:
                ports.append(port_line)
            i += 1
            continue

        elif phase == 'post_entity':
            if re.match(r'^\s*architecture\s+', stripped, re.I):
                phase = 'arch_decl'
            i += 1
            continue

        elif phase == 'arch_decl':
            if re.match(r'^\s*begin\s*$', stripped, re.I):
                phase = 'arch_body'
                i += 1
                continue
            # Parse signal/type/constant declarations
            sig_line = convert_signal_or_type(stripped, line)
            if sig_line:
                signals.append(sig_line)
            i += 1
            continue

        elif phase == 'arch_body':
            if re.match(r'^\s*end\s+(architecture|behaviour|behavior)', stripped, re.I):
                break
            body_lines.append(convert_body_line(stripped, line))
            i += 1
            continue

        i += 1

    # Phase 2: Assemble output
    result = []
    result.extend(header_comments)
    result.append('')
    result.append(f'module {module_name} (')

    # Remove trailing comma from last port
    if ports:
        for j, p in enumerate(ports):
            if j < len(ports) - 1:
                result.append(p + ',')
            else:
                result.append(p)
    result.append(');')
    result.append('')
    result.append('import wf68k30l_pkg::*;')
    result.append('')

    result.extend(signals)
    result.append('')
    result.extend(body_lines)
    result.append('')
    result.append('endmodule')

    return '\n'.join(result)


def convert_port(stripped, original):
    """Convert a VHDL port declaration to SV."""
    # Remove comments for parsing, preserve for output
    comment = ''
    cm = re.search(r'//(.*)$', original)
    if cm:
        comment = ' //' + cm.group(1)

    # Strip comment and trailing punctuation
    s = re.sub(r'//.*$', '', stripped).strip().rstrip(';').rstrip(',').rstrip(')')
    if not s or s.startswith('//'):
        return None

    # Match: NAME : direction type
    m = re.match(r'(\w+)\s*:\s*(in|out|buffer|inout)\s+(.+)', s, re.I)
    if not m:
        return None

    name, direction, typ = m.groups()
    direction = direction.lower()
    sv_dir = {'in': 'input', 'out': 'output', 'buffer': 'output', 'inout': 'inout'}[direction]
    sv_type = convert_type(typ.strip())

    return f'    {sv_dir} {sv_type} {name}{comment}'


def convert_type(typ):
    """Convert VHDL type to SV type."""
    typ = typ.strip().rstrip(';')

    if typ.lower() == 'std_logic' or typ.lower() == 'bit':
        return 'logic'
    if typ.lower() == 'boolean':
        return 'logic'

    # std_logic_vector(N downto 0)
    m = re.match(r'[Ss]td_[Ll]ogic_[Vv]ector\s*\(\s*(\d+)\s+downto\s+(\d+)\s*\)', typ)
    if m:
        hi, lo = m.groups()
        return f'logic [{hi}:{lo}]'

    # Std_Logic_Vector with different casing
    m = re.match(r'\w+_vector\s*\(\s*(\d+)\s+downto\s+(\d+)\s*\)', typ, re.I)
    if m:
        hi, lo = m.groups()
        return f'logic [{hi}:{lo}]'

    # integer range X to Y
    m = re.match(r'integer\s+range\s+(\d+)\s+to\s+(\d+)', typ, re.I)
    if m:
        lo, hi = int(m.group(1)), int(m.group(2))
        # Use int for small ranges
        return 'int'

    # natural range
    m = re.match(r'natural\s+range\s+(\d+)\s+to\s+(\d+)', typ, re.I)
    if m:
        return 'int'

    # OP_68K, OP_SIZETYPE, etc (custom types from package)
    if typ in ('OP_68K', 'OP_SIZETYPE', 'TRAPTYPE_OPC'):
        return typ

    return typ  # Return as-is for manual fixup


def convert_signal_or_type(stripped, original):
    """Convert signal/type/constant declarations."""
    s = stripped

    # Convert comments
    s = re.sub(r'----(.*)----', r'//\1//', s)
    s = re.sub(r'--(.*)$', r'//\1', s)

    if not s or s.startswith('//'):
        return s

    # Signal declaration
    m = re.match(r'signal\s+(\w+)\s*:\s*(.+?)(?:\s*:=\s*(.+?))?\s*;', s, re.I)
    if m:
        name, typ, default = m.groups()
        sv_type = convert_type(typ.strip())
        if default:
            default = convert_literal(default.strip())
            return f'{sv_type} {name} = {default};'
        return f'{sv_type} {name};'

    # Type declaration
    if re.match(r'type\s+', s, re.I):
        return '// ' + s  # Comment out for manual conversion

    # Constant
    if re.match(r'constant\s+', s, re.I):
        return '// ' + s  # Comment out for manual conversion

    return '// ' + s


def convert_literal(val):
    """Convert VHDL literals to SV."""
    val = val.strip()

    # x"FFFF" -> 16'hFFFF
    m = re.match(r'x"([0-9A-Fa-f]+)"', val)
    if m:
        hex_val = m.group(1)
        bits = len(hex_val) * 4
        return f"{bits}'h{hex_val}"

    # "0101" -> 4'b0101
    m = re.match(r'"([01]+)"', val)
    if m:
        bin_val = m.group(1)
        bits = len(bin_val)
        return f"{bits}'b{bin_val}"

    # '0' or '1'
    if val == "'0'":
        return "1'b0"
    if val == "'1'":
        return "1'b1"

    # Boolean
    if val.lower() == 'true':
        return "1'b1"
    if val.lower() == 'false':
        return "1'b0"

    # UNIMPLEMENTED or other enum
    return val


def convert_body_line(stripped, original):
    """Convert a line from the architecture body."""
    line = original

    # Convert comments
    line = re.sub(r'----(.*)----', r'//\1//', line)
    line = re.sub(r'--(.*)$', r'//\1', line)

    s = stripped

    # Skip pure comment lines
    if s.startswith('//'):
        return line

    # Process headers
    if re.match(r'\w+\s*:\s*process', s, re.I):
        return '// PROCESS: ' + line.strip()
    if s.lower().startswith('process'):
        return '// PROCESS: ' + line.strip()

    # begin/end process
    if re.match(r'^\s*begin\s*$', s, re.I):
        return '    begin'
    if re.match(r'^\s*end\s+process\s+\w+\s*;', s, re.I):
        return '    end // ' + s

    # Replace VHDL operators and syntax
    line = convert_expressions(line)

    return line


def convert_expressions(line):
    """Convert VHDL expressions to SV equivalents."""
    # Don't touch comment-only lines
    if line.strip().startswith('//'):
        return line

    # Split line into code and comment parts
    comment = ''
    cm = re.search(r'(//.*$)', line)
    if cm:
        comment = cm.group(1)
        line = line[:cm.start()]

    # wait until CLK = '1' and CLK' event;
    line = re.sub(r"wait\s+until\s+CLK\s*=\s*'1'\s+and\s+CLK'\s*event\s*;",
                  '// always_ff @(posedge CLK) begin', line, flags=re.I)

    # if CLK = '1' and CLK' event then
    line = re.sub(r"if\s+CLK\s*=\s*'1'\s+and\s+CLK'\s*event\s+then",
                  '// always_ff @(posedge CLK) begin', line, flags=re.I)

    # Bit literals
    line = re.sub(r"'1'", "1'b1", line)
    line = re.sub(r"'0'", "1'b0", line)

    # Hex literals x"FFFF" -> 16'hFFFF (calculate width from hex digits)
    def hex_replace(m):
        hex_val = m.group(1)
        bits = len(hex_val) * 4
        return f"{bits}'h{hex_val}"
    line = re.sub(r'x"([0-9A-Fa-f]+)"', hex_replace, line)

    # Binary literals "0101" -> 4'b0101
    def bin_replace(m):
        bin_val = m.group(1)
        bits = len(bin_val)
        return f"{bits}'b{bin_val}"
    line = re.sub(r'"([01]+)"', bin_replace, line)

    # Boolean
    line = re.sub(r'\btrue\b', "1'b1", line, flags=re.I)
    line = re.sub(r'\bfalse\b', "1'b0", line, flags=re.I)

    # VHDL concatenation & -> {,} — this is tricky, skip for manual fixup
    # line = re.sub(r'\s+&\s+', ', ', line)  # Too aggressive

    # not X -> ~X (but not "not" in comments)
    line = re.sub(r'\bnot\s+', '~', line)

    # Reassemble
    if comment:
        line = line + comment

    return line


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: vhdl2sv.py <file.vhd> [module_name]")
        sys.exit(1)

    filename = sys.argv[1]
    module_name = sys.argv[2] if len(sys.argv) > 2 else None

    with open(filename) as f:
        vhdl = f.read()

    if not module_name:
        m = re.search(r'entity\s+(\w+)\s+is', vhdl, re.I)
        module_name = m.group(1) if m else 'unknown'

    sv = convert_vhdl_to_sv(vhdl, module_name)

    outfile = filename.replace('.vhd', '.sv')
    with open(outfile, 'w') as f:
        f.write(sv)

    print(f"Wrote {outfile}")
