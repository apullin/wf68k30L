// ============================================================================
// WF68K30L IP Core: Opcode decode functions
//
// This file is `included inside the WF68K30L_OPCODE_DECODER module.
// It contains all EA-validity helper functions and per-group decode
// functions for the 68030 instruction set.
// ============================================================================

// ============================================================================
// Addressing mode validation functions
//
// These encapsulate the repeated EA-validity patterns used throughout the
// 68030 opcode map. Each takes the 3-bit mode and register fields from
// the instruction word and returns whether the combination is legal for
// the given addressing category.
// ============================================================================

// Data EA: all modes except An direct (001). Mode 111 limited by reg_fld.
function automatic logic is_data_ea(input logic [2:0] mode, input logic [2:0] reg_fld, input logic [2:0] max_reg);
    if (mode == 3'b111)
        is_data_ea = reg_fld < max_reg;
    else
        is_data_ea = mode != 3'b001;
endfunction

// All EA including An direct. Mode 111 limited by reg_fld.
function automatic logic is_all_ea(input logic [2:0] mode, input logic [2:0] reg_fld, input logic [2:0] max_reg);
    if (mode == 3'b111)
        is_all_ea = reg_fld < max_reg;
    else
        is_all_ea = 1'b1;
endfunction

// Data alterable: no An (001), no PC-relative, no immediate.
// Modes 000, 010-110 valid. Mode 111 with reg < 010.
function automatic logic is_data_alterable(input logic [2:0] mode, input logic [2:0] reg_fld);
    if (mode == 3'b111)
        is_data_alterable = reg_fld < 3'b010;
    else
        is_data_alterable = mode != 3'b001;
endfunction

// Memory alterable: modes 010-110 valid. Mode 111 with reg < 010.
function automatic logic is_mem_alterable(input logic [2:0] mode, input logic [2:0] reg_fld);
    if (mode == 3'b111)
        is_mem_alterable = reg_fld < 3'b010;
    else
        is_mem_alterable = mode > 3'b001;
endfunction

// Control EA: modes 010, 101, 110 valid. Mode 111 with reg < max_reg.
function automatic logic is_control_ea(input logic [2:0] mode, input logic [2:0] reg_fld, input logic [2:0] max_reg);
    if (mode == 3'b111)
        is_control_ea = reg_fld < max_reg;
    else
        is_control_ea = mode == 3'b010 || mode == 3'b101 || mode == 3'b110;
endfunction

// Bitfield EA: Dn (000), (An) (010), (d16,An) (101), (d8,An,Xn) (110).
// Mode 111 with reg < max_reg.
function automatic logic is_bf_ea(input logic [2:0] mode, input logic [2:0] reg_fld, input logic [2:0] max_reg);
    if (mode == 3'b111)
        is_bf_ea = reg_fld < max_reg;
    else
        is_bf_ea = mode == 3'b000 || mode == 3'b010 || mode >= 3'b101;
endfunction

// ============================================================================
// Per-group decode functions
//
// Each function handles one (or a pair of) top-level opcode groups,
// corresponding to bits [15:12] of the instruction word. They accept
// the instruction word (and the second pipe word where needed) and
// produce the decoded OP_68K enumeration value.
//
// Yosys does not support the SystemVerilog 'return' statement, so all
// functions use assignment to the function name with nested if/else.
// ============================================================================

// decode_0xxx: Bit manipulation / MOVEP / Immediate ops
function automatic OP_68K decode_0xxx(input logic [15:0] iw, input logic [15:0] iw_c);
    logic [2:0] mode, reg_fld;
    logic matched;
    mode    = iw[5:3];
    reg_fld = iw[2:0];
    decode_0xxx = ILLEGAL;
    matched = 1'b0;

    // --- Exact-match special opcodes ---
    if (!matched) begin
        case (iw[11:0])
            12'h03C: begin decode_0xxx = ORI_TO_CCR;  matched = 1'b1; end
            12'h07C: begin decode_0xxx = ORI_TO_SR;   matched = 1'b1; end
            12'h23C: begin decode_0xxx = ANDI_TO_CCR;  matched = 1'b1; end
            12'h27C: begin decode_0xxx = ANDI_TO_SR;   matched = 1'b1; end
            12'hA3C: begin decode_0xxx = EORI_TO_CCR;  matched = 1'b1; end
            12'hA7C: begin decode_0xxx = EORI_TO_SR;   matched = 1'b1; end
            12'hCFC: begin decode_0xxx = CAS2;         matched = 1'b1; end
            12'hEFC: begin decode_0xxx = CAS2;         matched = 1'b1; end
            default: ;
        endcase
    end

    // --- MOVES ---
    if (!matched && iw[11:8] == 4'hE && iw[7:6] < 2'b11 && is_mem_alterable(mode, reg_fld)) begin
        decode_0xxx = MOVES;  matched = 1'b1;
    end

    // --- MOVEP ---
    if (!matched && iw[8:6] > 3'b011 && mode == 3'b001) begin
        decode_0xxx = MOVEP;  matched = 1'b1;
    end

    // --- CHK2 / CMP2 (require second word) ---
    if (!matched && !iw[11] && iw[10:9] != 2'b11 && iw[8:6] == 3'b011
            && is_control_ea(mode, reg_fld, 3'b100)) begin
        if (iw_c[11])
            decode_0xxx = CHK2;
        else
            decode_0xxx = CMP2;
        matched = 1'b1;
    end

    // --- CAS ---
    if (!matched && iw[11] && iw[10:9] != 2'b00 && iw[8:6] == 3'b011
            && is_mem_alterable(mode, reg_fld)) begin
        decode_0xxx = CAS;  matched = 1'b1;
    end

    // --- Static bit operations (immediate bit number in second word) ---
    if (!matched) begin
        case (iw[11:6])
            6'b100000: if (is_data_ea(mode, reg_fld, 3'b100))   begin decode_0xxx = BTST; matched = 1'b1; end
            6'b100001: if (is_data_alterable(mode, reg_fld))     begin decode_0xxx = BCHG; matched = 1'b1; end
            6'b100010: if (is_data_alterable(mode, reg_fld))     begin decode_0xxx = BCLR; matched = 1'b1; end
            6'b100011: if (is_data_alterable(mode, reg_fld))     begin decode_0xxx = BSET; matched = 1'b1; end
            default: ;
        endcase
    end

    // --- Logic immediate operations (size in [7:6]) ---
    if (!matched && iw[7:6] < 2'b11 && is_data_alterable(mode, reg_fld)) begin
        case (iw[11:8])
            4'h0: begin decode_0xxx = ORI;  matched = 1'b1; end
            4'h2: begin decode_0xxx = ANDI; matched = 1'b1; end
            4'h4: begin decode_0xxx = SUBI; matched = 1'b1; end
            4'h6: begin decode_0xxx = ADDI; matched = 1'b1; end
            4'hA: begin decode_0xxx = EORI; matched = 1'b1; end
            default: ;
        endcase
    end

    // --- CMPI: data EA includes PC-relative (reg < 100) ---
    if (!matched && iw[11:8] == 4'hC && iw[7:6] < 2'b11 && is_data_ea(mode, reg_fld, 3'b100)) begin
        decode_0xxx = CMPI;  matched = 1'b1;
    end

    // --- Dynamic bit operations (register-specified bit number) ---
    if (!matched && mode != 3'b001) begin
        case (iw[8:6])
            3'b100: if (is_data_ea(mode, reg_fld, 3'b101))    begin decode_0xxx = BTST; matched = 1'b1; end
            3'b101: if (is_data_alterable(mode, reg_fld))      begin decode_0xxx = BCHG; matched = 1'b1; end
            3'b110: if (is_data_alterable(mode, reg_fld))      begin decode_0xxx = BCLR; matched = 1'b1; end
            3'b111: if (is_data_alterable(mode, reg_fld))      begin decode_0xxx = BSET; matched = 1'b1; end
            default: ;
        endcase
    end
endfunction

// decode_1xxx: MOVE.B
function automatic OP_68K decode_1xxx(input logic [15:0] iw);
    logic [2:0] src_mode, src_reg, dst_mode, dst_reg;
    logic src_ok, dst_ok;
    src_mode = iw[5:3];
    src_reg  = iw[2:0];
    dst_mode = iw[8:6];
    dst_reg  = iw[11:9];

    // Source: all modes except An direct (byte). Mode 111: reg < 101.
    src_ok = is_data_ea(src_mode, src_reg, 3'b101);

    // Destination: data alterable (no An for byte).
    if (dst_mode == 3'b111)
        dst_ok = dst_reg < 3'b010;
    else
        dst_ok = dst_mode != 3'b001;

    if (src_ok && dst_ok)
        decode_1xxx = MOVE;
    else
        decode_1xxx = ILLEGAL;
endfunction

// decode_23xxx: MOVE.W / MOVE.L / MOVEA
function automatic OP_68K decode_23xxx(input logic [15:0] iw);
    logic [2:0] src_mode, src_reg, dst_mode, dst_reg;
    logic src_ok;
    src_mode = iw[5:3];
    src_reg  = iw[2:0];
    dst_mode = iw[8:6];
    dst_reg  = iw[11:9];

    // Source: all modes. Mode 111: reg < 101.
    src_ok = is_all_ea(src_mode, src_reg, 3'b101);

    if (!src_ok)
        decode_23xxx = ILLEGAL;
    else if (dst_mode == 3'b001)
        decode_23xxx = MOVEA;
    else if (dst_mode == 3'b111 && dst_reg >= 3'b010)
        decode_23xxx = ILLEGAL;
    else
        decode_23xxx = MOVE;
endfunction

// decode_4xxx: Miscellaneous (LEA, CHK, MOVEM, CLR, NEG, etc.)
function automatic OP_68K decode_4xxx(input logic [15:0] iw, input logic [15:0] iw_c);
    logic [2:0] mode, reg_fld;
    logic matched;
    mode    = iw[5:3];
    reg_fld = iw[2:0];
    decode_4xxx = ILLEGAL;
    matched = 1'b0;

    // --- Exact-match system control opcodes ---
    if (!matched) begin
        case (iw[11:0])
            12'hE70: begin decode_4xxx = OP_RESET; matched = 1'b1; end
            12'hE71: begin decode_4xxx = NOP;      matched = 1'b1; end
            12'hE72: begin decode_4xxx = STOP;     matched = 1'b1; end
            12'hE73: begin decode_4xxx = RTE;      matched = 1'b1; end
            12'hE74: begin decode_4xxx = RTD;      matched = 1'b1; end
            12'hE75: begin decode_4xxx = RTS;      matched = 1'b1; end
            12'hE76: begin decode_4xxx = TRAPV;    matched = 1'b1; end
            12'hE77: begin decode_4xxx = RTR;      matched = 1'b1; end
            12'hAFC: begin decode_4xxx = ILLEGAL;  matched = 1'b1; end
            default: ;
        endcase
    end

    // --- MOVEC ---
    if (!matched && iw[11:1] == 11'b11100111101) begin
        case (iw_c[11:0])
            12'h000, 12'h001, 12'h800, 12'h801: decode_4xxx = MOVEC;
            default: decode_4xxx = ILLEGAL;
        endcase
        matched = 1'b1;
    end

    // --- Fixed register-group opcodes ---
    if (!matched) begin
        case (iw[11:3])
            9'b100001001: begin decode_4xxx = BKPT; matched = 1'b1; end
            9'b100000001: begin decode_4xxx = LINK; matched = 1'b1; end  // LONG
            9'b111001010: begin decode_4xxx = LINK; matched = 1'b1; end  // WORD
            9'b111001011: begin decode_4xxx = UNLK; matched = 1'b1; end
            9'b100001000: begin decode_4xxx = SWAP; matched = 1'b1; end
            default: ;
        endcase
    end

    if (!matched && iw[11:4] == 8'hE4) begin decode_4xxx = TRAP;     matched = 1'b1; end
    if (!matched && iw[11:4] == 8'hE6) begin decode_4xxx = MOVE_USP; matched = 1'b1; end

    // --- DIV long / MUL long / MOVE SR,CCR / NBCD / TAS ---
    if (!matched) begin
        case (iw[11:6])
            6'b110001: if (is_data_ea(mode, reg_fld, 3'b101)) begin
                decode_4xxx = iw_c[11] ? DIVS : DIVU;  matched = 1'b1;
            end
            6'b110000: if (is_data_ea(mode, reg_fld, 3'b101)) begin
                decode_4xxx = iw_c[11] ? MULS : MULU;  matched = 1'b1;
            end
            6'b001011: if (is_data_alterable(mode, reg_fld)) begin
                decode_4xxx = MOVE_FROM_CCR;  matched = 1'b1;
            end
            6'b000011: if (is_data_alterable(mode, reg_fld)) begin
                decode_4xxx = MOVE_FROM_SR;   matched = 1'b1;
            end
            6'b010011: if (is_data_ea(mode, reg_fld, 3'b101)) begin
                decode_4xxx = MOVE_TO_CCR;    matched = 1'b1;
            end
            6'b011011: if (is_data_ea(mode, reg_fld, 3'b101)) begin
                decode_4xxx = MOVE_TO_SR;     matched = 1'b1;
            end
            6'b100000: if (is_data_alterable(mode, reg_fld)) begin
                decode_4xxx = NBCD;           matched = 1'b1;
            end
            6'b101011: if (is_data_alterable(mode, reg_fld)) begin
                decode_4xxx = TAS;            matched = 1'b1;
            end
            default: ;
        endcase
    end

    // --- PEA / JSR / JMP (control addressing) ---
    if (!matched) begin
        case (iw[11:6])
            6'b100001: if (is_control_ea(mode, reg_fld, 3'b100)) begin
                decode_4xxx = PEA;  matched = 1'b1;
            end
            6'b111010: if (is_control_ea(mode, reg_fld, 3'b100)) begin
                decode_4xxx = JSR;  matched = 1'b1;
            end
            6'b111011: if (is_control_ea(mode, reg_fld, 3'b100)) begin
                decode_4xxx = JMP;  matched = 1'b1;
            end
            default: ;
        endcase
    end

    // --- NEGX / CLR / NEG / NOT (data alterable, size valid) ---
    if (!matched && iw[7:6] < 2'b11 && is_data_alterable(mode, reg_fld)) begin
        case (iw[11:8])
            4'h0: begin decode_4xxx = NEGX;  matched = 1'b1; end
            4'h2: begin decode_4xxx = CLR;   matched = 1'b1; end
            4'h4: begin decode_4xxx = NEG;   matched = 1'b1; end
            4'h6: begin decode_4xxx = NOT_B; matched = 1'b1; end
            default: ;
        endcase
    end

    // --- TST (word/long: all EAs; byte: data EAs) ---
    if (!matched && iw[11:8] == 4'hA && iw[7:6] < 2'b11) begin
        if (iw[7:6] == 2'b01 || iw[7:6] == 2'b10) begin
            if (is_all_ea(mode, reg_fld, 3'b101)) begin
                decode_4xxx = TST;  matched = 1'b1;
            end
        end else begin
            if (is_data_ea(mode, reg_fld, 3'b101)) begin
                decode_4xxx = TST;  matched = 1'b1;
            end
        end
    end

    // --- EXT / EXTB (data register only) ---
    if (!matched && iw[11:9] == 3'b100 && mode == 3'b000) begin
        case (iw[8:6])
            3'b010, 3'b011: begin decode_4xxx = EXT;  matched = 1'b1; end
            3'b111:         begin decode_4xxx = EXTB; matched = 1'b1; end
            default: ;
        endcase
    end

    // --- LEA (control addressing) ---
    if (!matched && iw[8:6] == 3'b111 && is_control_ea(mode, reg_fld, 3'b100)) begin
        decode_4xxx = LEA;  matched = 1'b1;
    end

    // --- MOVEM ---
    if (!matched && iw[11] && iw[9:7] == 3'b001) begin
        if (!iw[10]) begin
            // Register to memory: no postincrement (010, 100, 101, 110; 111 with reg < 010)
            case (mode)
                3'b010, 3'b100, 3'b101, 3'b110: begin decode_4xxx = MOVEM; matched = 1'b1; end
                3'b111: if (reg_fld < 3'b010) begin decode_4xxx = MOVEM; matched = 1'b1; end
                default: ;
            endcase
        end else begin
            // Memory to register: no predecrement (010, 011, 101, 110; 111 with reg < 100)
            case (mode)
                3'b010, 3'b011, 3'b101, 3'b110: begin decode_4xxx = MOVEM; matched = 1'b1; end
                3'b111: if (reg_fld < 3'b100) begin decode_4xxx = MOVEM; matched = 1'b1; end
                default: ;
            endcase
        end
    end

    // --- CHK (size must be 10 or 11, opmode not 001) ---
    if (!matched && iw[8:7] >= 2'b10) begin
        if (mode == 3'b111) begin
            if (reg_fld < 3'b101) begin
                decode_4xxx = CHK;  matched = 1'b1;
            end
        end else if (iw[6:3] != 4'h1) begin
            decode_4xxx = CHK;  matched = 1'b1;
        end
    end
endfunction

// decode_5xxx: ADDQ / SUBQ / Scc / DBcc / TRAPcc
function automatic OP_68K decode_5xxx(input logic [15:0] iw);
    logic [2:0] mode, reg_fld;
    logic ea_ok;
    mode    = iw[5:3];
    reg_fld = iw[2:0];
    decode_5xxx = ILLEGAL;

    if (iw[7:3] == 5'b11001) begin
        // DBcc: size=11, mode=001
        decode_5xxx = DBcc;
    end else if (iw[7:6] == 2'b11 && is_data_alterable(mode, reg_fld)) begin
        // Scc: size=11, data alterable
        decode_5xxx = Scc;
    end else if (iw[7:6] < 2'b11) begin
        // ADDQ / SUBQ
        if (iw[7:6] == 2'b00)
            ea_ok = is_data_alterable(mode, reg_fld); // Byte: no An direct
        else if (mode == 3'b111)
            ea_ok = reg_fld < 3'b010; // Word/Long: alterable including An
        else
            ea_ok = 1'b1;

        if (ea_ok)
            decode_5xxx = iw[8] ? SUBQ : ADDQ;
    end else if (iw[7:3] == 5'b11111) begin
        // TRAPcc
        decode_5xxx = TRAPcc;
    end
endfunction

// decode_6xxx: Bcc / BSR / BRA
function automatic OP_68K decode_6xxx(input logic [15:0] iw);
    if (iw[11:8] == 4'h0)
        decode_6xxx = BRA;
    else if (iw[11:8] == 4'h1)
        decode_6xxx = BSR;
    else
        decode_6xxx = Bcc;
endfunction

// decode_7xxx: MOVEQ
function automatic OP_68K decode_7xxx(input logic [15:0] iw);
    if (!iw[8])
        decode_7xxx = MOVEQ;
    else
        decode_7xxx = ILLEGAL;
endfunction

// decode_8xxx: OR / DIV / SBCD / PACK / UNPK
function automatic OP_68K decode_8xxx(input logic [15:0] iw);
    logic [2:0] mode, reg_fld;
    mode    = iw[5:3];
    reg_fld = iw[2:0];
    decode_8xxx = ILLEGAL;

    if (iw[8:4] == 5'b10100)
        decode_8xxx = PACK;
    else if (iw[8:4] == 5'b11000)
        decode_8xxx = UNPK;
    else if (iw[8:6] == 3'b011 && is_data_ea(mode, reg_fld, 3'b101))
        decode_8xxx = DIVU;
    else if (iw[8:6] == 3'b111 && is_data_ea(mode, reg_fld, 3'b101))
        decode_8xxx = DIVS;
    else if (iw[8:4] == 5'b10000)
        decode_8xxx = SBCD;
    else begin
        // OR: source EA to register (000-010) or register to memory EA (100-110)
        case (iw[8:6])
            3'b000, 3'b001, 3'b010:
                if (is_data_ea(mode, reg_fld, 3'b101)) decode_8xxx = OR_B;
            3'b100, 3'b101, 3'b110:
                if (is_mem_alterable(mode, reg_fld)) decode_8xxx = OR_B;
            default: ;
        endcase
    end
endfunction

// Shared helper for ADD/SUB groups (identical structure)
function automatic OP_68K decode_addsub_group(
    input logic [15:0] iw,
    input OP_68K       op_main,
    input OP_68K       op_extended,
    input OP_68K       op_address
);
    logic [2:0] mode, reg_fld;
    mode    = iw[5:3];
    reg_fld = iw[2:0];
    decode_addsub_group = ILLEGAL;

    case (iw[8:6])
        3'b000: begin // Byte: source EA to register, no An direct
            if (is_data_ea(mode, reg_fld, 3'b101))
                decode_addsub_group = op_main;
        end
        3'b001, 3'b010: begin // Word/Long: source EA to register, all modes
            if (is_all_ea(mode, reg_fld, 3'b101))
                decode_addsub_group = op_main;
        end
        3'b100: begin // Byte: register to EA or extended
            if (mode == 3'b000 || mode == 3'b001)
                decode_addsub_group = op_extended;
            else if (is_data_alterable(mode, reg_fld))
                decode_addsub_group = op_main;
        end
        3'b101, 3'b110: begin // Word/Long: register to EA or extended
            if (mode == 3'b000 || mode == 3'b001)
                decode_addsub_group = op_extended;
            else if (is_mem_alterable(mode, reg_fld))
                decode_addsub_group = op_main;
        end
        3'b011, 3'b111: begin // ADDA/SUBA: all source EAs
            if (is_all_ea(mode, reg_fld, 3'b101))
                decode_addsub_group = op_address;
        end
        default: ;
    endcase
endfunction

// decode_9xxx: SUB / SUBA / SUBX
function automatic OP_68K decode_9xxx(input logic [15:0] iw);
    decode_9xxx = decode_addsub_group(iw, SUB, SUBX, SUBA);
endfunction

// decode_Bxxx: CMP / CMPA / CMPM / EOR
function automatic OP_68K decode_Bxxx(input logic [15:0] iw);
    logic [2:0] mode, reg_fld;
    mode    = iw[5:3];
    reg_fld = iw[2:0];
    decode_Bxxx = ILLEGAL;

    if (iw[8] && iw[7:6] < 2'b11 && mode == 3'b001) begin
        // CMPM: postincrement mode (001) with size != 11
        decode_Bxxx = CMPM;
    end else begin
        case (iw[8:6])
            3'b000: // CMP byte: data EA (no An direct)
                if (is_data_ea(mode, reg_fld, 3'b101)) decode_Bxxx = CMP;
            3'b001, 3'b010: // CMP word/long: all EA
                if (is_all_ea(mode, reg_fld, 3'b101)) decode_Bxxx = CMP;
            3'b011, 3'b111: // CMPA: all EA
                if (is_all_ea(mode, reg_fld, 3'b101)) decode_Bxxx = CMPA;
            3'b100, 3'b101, 3'b110: // EOR: data alterable
                if (is_data_alterable(mode, reg_fld)) decode_Bxxx = EOR;
            default: ;
        endcase
    end
endfunction

// decode_Cxxx: AND / MUL / ABCD / EXG
function automatic OP_68K decode_Cxxx(input logic [15:0] iw);
    logic [2:0] mode, reg_fld;
    mode    = iw[5:3];
    reg_fld = iw[2:0];
    decode_Cxxx = ILLEGAL;

    if (iw[8:4] == 5'b10000)
        decode_Cxxx = ABCD;
    else if (iw[8:6] == 3'b011 && is_data_ea(mode, reg_fld, 3'b101))
        decode_Cxxx = MULU;
    else if (iw[8:6] == 3'b111 && is_data_ea(mode, reg_fld, 3'b101))
        decode_Cxxx = MULS;
    else if (iw[8:3] == 6'b101000 || iw[8:3] == 6'b101001 || iw[8:3] == 6'b110001)
        decode_Cxxx = EXG;
    else begin
        case (iw[8:6])
            3'b000, 3'b001, 3'b010:
                if (is_data_ea(mode, reg_fld, 3'b101)) decode_Cxxx = AND_B;
            3'b100, 3'b101, 3'b110:
                if (is_mem_alterable(mode, reg_fld)) decode_Cxxx = AND_B;
            default: ;
        endcase
    end
endfunction

// decode_Dxxx: ADD / ADDA / ADDX
function automatic OP_68K decode_Dxxx(input logic [15:0] iw);
    decode_Dxxx = decode_addsub_group(iw, ADD, ADDX, ADDA);
endfunction

// decode_Exxx: Shift / Rotate / Bit Field
function automatic OP_68K decode_Exxx(input logic [15:0] iw);
    logic [2:0] mode, reg_fld;
    logic matched;
    mode    = iw[5:3];
    reg_fld = iw[2:0];
    decode_Exxx = ILLEGAL;
    matched = 1'b0;

    // --- Bitfield operations (size field = 11, bit 11 set) ---
    if (!matched && iw[7:6] == 2'b11 && iw[11]) begin
        case (iw[11:6])
            6'b101011: if (is_bf_ea(mode, reg_fld, 3'b010)) begin decode_Exxx = BFCHG;  matched = 1'b1; end
            6'b110011: if (is_bf_ea(mode, reg_fld, 3'b010)) begin decode_Exxx = BFCLR;  matched = 1'b1; end
            6'b111011: if (is_bf_ea(mode, reg_fld, 3'b010)) begin decode_Exxx = BFSET;  matched = 1'b1; end
            6'b111111: if (is_bf_ea(mode, reg_fld, 3'b010)) begin decode_Exxx = BFINS;  matched = 1'b1; end
            6'b101111: if (is_bf_ea(mode, reg_fld, 3'b100)) begin decode_Exxx = BFEXTS; matched = 1'b1; end
            6'b100111: if (is_bf_ea(mode, reg_fld, 3'b100)) begin decode_Exxx = BFEXTU; matched = 1'b1; end
            6'b110111: if (is_bf_ea(mode, reg_fld, 3'b100)) begin decode_Exxx = BFFFO;  matched = 1'b1; end
            6'b100011: if (is_bf_ea(mode, reg_fld, 3'b100)) begin decode_Exxx = BFTST;  matched = 1'b1; end
            default: ;
        endcase
    end

    // --- Memory shifts/rotates (size field = 11, bit 11 clear) ---
    if (!matched && iw[7:6] == 2'b11 && !iw[11] && is_mem_alterable(mode, reg_fld)) begin
        case (iw[11:6])
            6'b000011: begin decode_Exxx = ASR;  matched = 1'b1; end
            6'b000111: begin decode_Exxx = ASL;  matched = 1'b1; end
            6'b001011: begin decode_Exxx = LSR;  matched = 1'b1; end
            6'b001111: begin decode_Exxx = LSL;  matched = 1'b1; end
            6'b010011: begin decode_Exxx = ROXR; matched = 1'b1; end
            6'b010111: begin decode_Exxx = ROXL; matched = 1'b1; end
            6'b011011: begin decode_Exxx = ROTR; matched = 1'b1; end
            6'b011111: begin decode_Exxx = ROTL; matched = 1'b1; end
            default: ;
        endcase
    end

    // --- Register shifts/rotates (size != 11) ---
    if (!matched && iw[7:6] < 2'b11) begin
        case ({iw[8], iw[4:3]})
            3'b000: decode_Exxx = ASR;
            3'b100: decode_Exxx = ASL;
            3'b001: decode_Exxx = LSR;
            3'b101: decode_Exxx = LSL;
            3'b010: decode_Exxx = ROXR;
            3'b110: decode_Exxx = ROXL;
            3'b011: decode_Exxx = ROTR;
            3'b111: decode_Exxx = ROTL;
        endcase
    end
endfunction
