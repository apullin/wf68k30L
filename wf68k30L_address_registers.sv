// ------------------------------------------------------------------------
// --                                                                    --
// -- WF68K30L IP Core: Address register logic.                          --
// --                                                                    --
// -- Description:                                                       --
// -- This module provides the address registers, stack pointers,        --
// -- the address arithmetics, the program counter logic and the SFC     --
// -- and DFC registers. The address registers are accessible by two     --
// -- read and two write ports simultaneously. For more information      --
// -- refer to the MC68030 User Manual.                                  --
// --                                                                    --
// -- Author(s):                                                         --
// -- - Wolfgang Foerster, wf@experiment-s.de; wf@inventronik.de         --
// --                                                                    --
// ------------------------------------------------------------------------
// --                                                                    --
// -- Copyright (c) 2014-2019 Wolfgang Foerster Inventronik GmbH.        --
// --                                                                    --
// -- This documentation describes Open Hardware and is licensed          --
// -- under the CERN OHL v. 1.2. You may redistribute and modify         --
// -- this documentation under the terms of the CERN OHL v.1.2.          --
// -- (http://ohwr.org/cernohl). This documentation is distributed       --
// -- WITHOUT ANY EXPRESS OR IMPLIED WARRANTY, INCLUDING OF               --
// -- MERCHANTABILITY, SATISFACTORY QUALITY AND FITNESS FOR A             --
// -- PARTICULAR PURPOSE. Please see the CERN OHL v.1.2 for              --
// -- applicable conditions                                              --
// --                                                                    --
// ------------------------------------------------------------------------
//
// Revision History
//
// Revision 2K14B 20141201 WF
//   Initial Release.
// Revision 2K16A 20160620 WF
//   Minor optimizations.
// Revision 2K18A 20180620 WF
//   Changed ADR_ATN logic to be valid one clock cycle earlier.
//   Fixed PC restoring during exception processing.
//   Fixed the writing ISP_REG during EXG instruction with two address registers.
//   Fixed writing the stack pointer registers (MSBIT is used now).
//   The address registers are always written long.
//   Bugfix: exception handler do not increment and decrement the USP any more.
//   MOVEM-Fix: the effective address in memory to register is stored (STORE_AEFF) not to be overwritten in case the addressing register is also loaded.
// Revision 2K19A 2019## WF
//   Removed ADR_ATN. We do not need this any more.
//

module WF68K30L_ADDRESS_REGISTERS (
    input  logic        CLK,
    input  logic        RESET,

    // Address and data:
    input  logic [31:0] AR_IN_1,
    input  logic [31:0] AR_IN_2,
    output logic [31:0] AR_OUT_1,
    output logic [31:0] AR_OUT_2,
    input  logic [31:0] INDEX_IN,
    output logic [31:0] PC,
    input  logic [3:0]  PC_EW_OFFSET,
    input  logic        FETCH_MEM_ADR,
    input  logic        STORE_ADR_FORMAT,
    input  logic        STORE_ABS_HI,
    input  logic        STORE_ABS_LO,
    input  logic        STORE_D16,
    input  logic        STORE_D32_LO,
    input  logic        STORE_D32_HI,
    input  logic        STORE_DISPL,
    input  logic        STORE_MEM_ADR,
    input  logic        STORE_OD_HI,
    input  logic        STORE_OD_LO,
    input  logic        STORE_AEFF,
    input  logic [1:0]  OP_SIZE,

    input  logic [31:0] ADR_OFFSET,
    input  logic        ADR_MARK_USED,
    input  logic        USE_APAIR,
    output logic        ADR_IN_USE,

    input  logic [2:0]  ADR_MODE,
    input  logic [2:0]  AMODE_SEL,
    input  logic        USE_DREG,
    output logic [31:0] ADR_EFF,
    output logic [31:0] ADR_EFF_WB,

    output logic [2:0]  DFC,
    input  logic        DFC_WR,
    output logic [2:0]  SFC,
    input  logic        SFC_WR,

    input  logic        ISP_DEC,
    input  logic        ISP_RD,
    input  logic        ISP_WR,
    input  logic        MSP_RD,
    input  logic        MSP_WR,
    input  logic        USP_RD,
    input  logic        USP_WR,

    // Registers controls:
    input  logic        AR_MARK_USED,
    output logic        AR_IN_USE,
    input  logic [2:0]  AR_SEL_RD_1,
    input  logic [2:0]  AR_SEL_RD_2,
    input  logic [2:0]  AR_SEL_WR_1,
    input  logic [2:0]  AR_SEL_WR_2,
    input  logic        AR_DEC,
    input  logic        AR_INC,
    input  logic        AR_WR_1,
    input  logic        AR_WR_2,
    input  logic        UNMARK,

    input  logic [15:0] EXT_WORD,

    input  logic        MBIT,
    input  logic        SBIT,

    input  logic        SP_ADD_DISPL,
    input  logic        RESTORE_ISP_PC,

    // Other controls:
    input  logic [31:0] DISPLACEMENT,
    input  logic        PC_ADD_DISPL,
    input  logic        PC_INC,
    input  logic        PC_LOAD,
    input  logic        PC_RESTORE,
    input  logic [7:0]  PC_OFFSET
);

`include "wf68k30L_pkg.svh"

logic [31:0] ADR_EFF_I;
logic [31:0] AR [0:6]; // Address registers A0 to A6.
logic [31:0] AR_OUT_1_I;
logic [31:0] AR_OUT_2_I;
logic [32:0] ADR_WB;
integer AR_PNTR_1;
integer AR_PNTR_2;
integer AR_PNTR_WB_1;
integer AR_PNTR_WB_2;
logic [3:0] AR_USED_1;
logic [3:0] AR_USED_2;
logic B_S; // Base register suppress.
logic [1:0] BD_SIZE; // Indexed / Indirect.
logic [2:0] DFC_REG_sig; // Special function code registers.
logic F_E; // Full extension word.
logic [2:0] I_IS; // Indexed / Indirect.
logic I_S; // Index suppress.
logic [31:0] ISP_REG; // Interrupt stack pointer (refers to A7'' in the supervisor mode).
logic [1:0] MSBIT;
logic [31:0] MSP_REG; // Master stack pointer (refers to A7' in the supervisor mode).
logic [31:0] PC_I; // Active program counter.
logic [1:0] SCALE; // Scale information for the index.
logic [2:0] SFC_REG_sig; // Special function code registers.
logic [31:0] USP_REG; // User stack pointer (refers to A7 in the user mode.).
logic [31:0] ADR_EFF_TMP_REG;

always_ff @(posedge CLK) begin : latch_write_sel
    if (AR_MARK_USED) begin
        AR_PNTR_WB_1 <= AR_SEL_WR_1;
        AR_PNTR_WB_2 <= AR_SEL_WR_2;
    end
end

assign AR_PNTR_1 = AR_SEL_RD_1;
assign AR_PNTR_2 = AR_SEL_RD_2;

always_ff @(posedge CLK) begin : track_in_use
    logic DELAY;
    if (RESET || UNMARK) begin
        AR_USED_1[3] <= 1'b0;
        AR_USED_2[3] <= 1'b0;
    end else if (AR_MARK_USED) begin
        AR_USED_1 <= {1'b1, AR_SEL_WR_1};
        if (USE_APAIR)
            AR_USED_2 <= {1'b1, AR_SEL_WR_2};
        MSBIT <= {MBIT, SBIT};
    end
    //
    if (RESET || UNMARK) begin
        ADR_WB[32] <= 1'b0;
        DELAY = 1'b0;
    end else if (ADR_MARK_USED) begin
        DELAY = 1'b1; // One clock cycle address calculation delay.
    end else if (DELAY) begin
        ADR_WB <= {1'b1, ADR_EFF_I};
        DELAY = 1'b0;
    end
end

assign AR_IN_USE = (AR_USED_1[2:0] == 3'b111 && SBIT && MSBIT[1] != MBIT) ? 1'b0 : // Wrong stack pointer.
                   (AR_USED_2[2:0] == 3'b111 && SBIT && MSBIT[1] != MBIT) ? 1'b0 : // Wrong stack pointer.
                   (AR_USED_1[3] && AR_USED_1[2:0] == AR_SEL_RD_1) ||
                   (AR_USED_1[3] && AR_USED_1[2:0] == AR_SEL_RD_2) ||
                   (AR_USED_2[3] && AR_USED_2[2:0] == AR_SEL_RD_1) ||
                   (AR_USED_2[3] && AR_USED_2[2:0] == AR_SEL_RD_2);

assign AR_OUT_1 = AR_OUT_1_I;
assign AR_OUT_2 = AR_OUT_2_I;

assign ADR_IN_USE = ADR_WB[32] && (
                        ADR_WB[31:2] == ADR_EFF_I[31:2]          ||  // Exact long-word match
                        ADR_WB[31:2] - 1'b1 == ADR_EFF_I[31:2]   ||  // Lock misaligned access (below)
                        ADR_WB[31:2] + 1'b1 == ADR_EFF_I[31:2]       // Lock misaligned access (above)
                    );

always_ff @(posedge CLK) begin : adr_format_latch
    if (STORE_ADR_FORMAT) begin
        SCALE <= EXT_WORD[10:9];
        F_E <= EXT_WORD[8];
        B_S <= EXT_WORD[7];
        I_S <= EXT_WORD[6];
        BD_SIZE <= EXT_WORD[5:4];
        I_IS <= EXT_WORD[2:0];
    end
end

// ADDRESS_MODES process - mixed clocked/combinational
// In VHDL this was a single process with both clocked and combinational parts.
// We split the registered part into always_ff and the combinational part into always_comb.

logic [31:0] INDEX_REG;
logic [31:0] MEM_ADR_REG;
logic [31:0] BASE_DISPL_REG;
logic [31:0] OUTER_DISPL_REG;
logic [31:0] ABS_ADDRESS_REG;

// Combinational next-value for INDEX.
// In the original VHDL, INDEX was a process variable with immediate (:=)
// assignment semantics: writes were visible to later reads in the same
// clock edge.  The SV port must reproduce this by computing the next
// INDEX value combinationally and feeding it into the INDEX_REG register.
logic [31:0] index_next;
always_comb begin : index_next_calc
    if (STORE_ADR_FORMAT && !EXT_WORD[15] && EXT_WORD[11])
        index_next = INDEX_IN; // Long data register.
    else if (STORE_ADR_FORMAT && !EXT_WORD[15])
        index_next = {{16{INDEX_IN[15]}}, INDEX_IN[15:0]}; // Sign extended data register.
    else if (STORE_ADR_FORMAT && EXT_WORD[11]) begin // Long address register.
        if (EXT_WORD[14:12] == 3'b111 && SBIT && MBIT)
            index_next = MSP_REG;
        else if (EXT_WORD[14:12] == 3'b111 && SBIT && !MBIT)
            index_next = ISP_REG;
        else if (EXT_WORD[14:12] == 3'b111 && !SBIT)
            index_next = USP_REG;
        else
            index_next = AR[EXT_WORD[14:12]];
    end else if (STORE_ADR_FORMAT) begin // Sign extended address register.
        if (EXT_WORD[14:12] == 3'b111 && SBIT && MBIT)
            index_next = {{16{MSP_REG[15]}}, MSP_REG[15:0]};
        else if (EXT_WORD[14:12] == 3'b111 && SBIT && !MBIT)
            index_next = {{16{ISP_REG[15]}}, ISP_REG[15:0]};
        else if (EXT_WORD[14:12] == 3'b111 && !SBIT)
            index_next = {{16{USP_REG[15]}}, USP_REG[15:0]};
        else
            index_next = {{16{AR[EXT_WORD[14:12]][15]}}, AR[EXT_WORD[14:12]][15:0]};
    end else begin
        index_next = INDEX_REG; // No update: retain current register value.
    end
end

// Combinational scaled index (mirrors VHDL variable INDEX_SCALED).
// In the VHDL, INDEX_SCALED was re-computed every clock cycle from the
// INDEX variable and the SCALE signal.  SCALE is registered and updates
// one cycle after STORE_ADR_FORMAT.  At the cycle when the effective
// address is actually used (CALC_AEFF), both INDEX_REG and SCALE hold
// their correct values, so the combinational product is correct.
logic [31:0] index_scaled_comb;
always_comb begin : index_scale_calc
    case (SCALE)
        2'b00:   index_scaled_comb = INDEX_REG;
        2'b01:   index_scaled_comb = {INDEX_REG[30:0], 1'b0};
        2'b10:   index_scaled_comb = {INDEX_REG[29:0], 2'b00};
        default: index_scaled_comb = {INDEX_REG[28:0], 3'b000};
    endcase
end

always_ff @(posedge CLK) begin : address_modes_reg
    // Capture the combinationally-computed index into the register.
    INDEX_REG <= index_next;
    //
    // Register for memory indirect addressing modes.
    if (STORE_MEM_ADR)
        MEM_ADR_REG <= AR_IN_1;

    // Base displacement
    if (RESET) begin
        BASE_DISPL_REG <= 32'h0;
    end else if (STORE_ADR_FORMAT && EXT_WORD[8] && EXT_WORD[5:4] == 2'b01) begin
        BASE_DISPL_REG <= 32'h0; // Null base displacement.
    end else if (STORE_ADR_FORMAT && !EXT_WORD[8]) begin
        BASE_DISPL_REG <= {{24{EXT_WORD[7]}}, EXT_WORD[7:0]};
    end else if (STORE_D16) begin
        BASE_DISPL_REG <= {{16{EXT_WORD[15]}}, EXT_WORD};
    end else if (STORE_D32_LO) begin
        if (BD_SIZE == 2'b10) // Word displacement.
            BASE_DISPL_REG[31:16] <= {16{EXT_WORD[15]}};
        BASE_DISPL_REG[15:0] <= EXT_WORD;
    end else if (STORE_D32_HI) begin
        BASE_DISPL_REG[31:16] <= EXT_WORD;
    end else if (STORE_DISPL) begin
        BASE_DISPL_REG <= DISPLACEMENT;
    end

    // Outer displacement
    if (STORE_ADR_FORMAT && EXT_WORD[8] && EXT_WORD[1:0] == 2'b01) begin
        OUTER_DISPL_REG <= 32'h0; // Null outer displacement.
    end else if (STORE_OD_LO) begin
        if (I_IS[1:0] == 2'b10)
            OUTER_DISPL_REG[31:16] <= {16{EXT_WORD[15]}};
        OUTER_DISPL_REG[15:0] <= EXT_WORD;
    end else if (STORE_OD_HI) begin
        OUTER_DISPL_REG[31:16] <= EXT_WORD;
    end

    // Absolute address
    if (STORE_ABS_LO) begin
        if (AMODE_SEL == 3'b000)
            ABS_ADDRESS_REG[31:16] <= {16{EXT_WORD[15]}};
        ABS_ADDRESS_REG[15:0] <= EXT_WORD;
    end else if (STORE_ABS_HI) begin
        ABS_ADDRESS_REG[31:16] <= EXT_WORD;
    end

    // Effective address register
    if (RESTORE_ISP_PC) begin
        ADR_EFF_I <= ADR_OFFSET; // During system initialization.
        ADR_EFF_TMP_REG <= ADR_EFF_VAR_comb;
    end else if (STORE_AEFF) begin // Used for MOVEM.
        ADR_EFF_I <= ADR_EFF_TMP_REG + ADR_OFFSET;
    end else begin // Normal operation:
        ADR_EFF_I <= ADR_EFF_VAR_comb + ADR_OFFSET;
        ADR_EFF_TMP_REG <= ADR_EFF_VAR_comb;
    end
end

// Combinational address calculation
logic [31:0] ADR_MUX_comb;
logic [31:0] ADR_EFF_VAR_comb;
logic [31:0] PCVAR_comb;
logic [3:0] I_S_IS_comb;

always_comb begin : adr_eff_calc
    I_S_IS_comb = {I_S, I_IS};
    PCVAR_comb = PC_I + {28'd0, PC_EW_OFFSET}; // Address of the extension word.

    // Address mux: select base register
    if (ADR_MODE == ADR_AN_IDX && FETCH_MEM_ADR && F_E && B_S)
        ADR_MUX_comb = 32'h0; // Base register suppress.
    else if (ADR_MODE == ADR_SPECIAL && FETCH_MEM_ADR && AMODE_SEL == 3'b011 && F_E && B_S)
        ADR_MUX_comb = 32'h0; // Base register suppress.
    else if (USE_DREG)
        ADR_MUX_comb = AR_IN_1;
    else begin
        case (AR_PNTR_1)
            7: begin
                if (SBIT && !MBIT)       ADR_MUX_comb = ISP_REG;
                else if (SBIT)            ADR_MUX_comb = MSP_REG;
                else                      ADR_MUX_comb = USP_REG;
            end
            default: ADR_MUX_comb = AR[AR_PNTR_1];
        endcase
    end

    // Effective address computation
    case (ADR_MODE)
        ADR_AN_IND, ADR_AN_POST, ADR_AN_PRE:  // (An), (An)+, -(An)
            ADR_EFF_VAR_comb = ADR_MUX_comb;

        ADR_AN_DISP:  // (d16,An)
            ADR_EFF_VAR_comb = ADR_MUX_comb + BASE_DISPL_REG;

        ADR_AN_IDX: begin  // (d8,An,Xn) or full extension word
            if (!F_E) begin // Brief extension word
                ADR_EFF_VAR_comb = ADR_MUX_comb + BASE_DISPL_REG + index_scaled_comb;
            end else begin // Full extension word
                case (I_S_IS_comb)
                    4'b0000, 4'b1000: // No memory indirect
                        ADR_EFF_VAR_comb = ADR_MUX_comb + BASE_DISPL_REG + index_scaled_comb;
                    4'b0001, 4'b0010, 4'b0011: // Preindexed
                        ADR_EFF_VAR_comb = FETCH_MEM_ADR ?
                            (ADR_MUX_comb + BASE_DISPL_REG + index_scaled_comb) :
                            (MEM_ADR_REG + OUTER_DISPL_REG);
                    4'b0101, 4'b0110, 4'b0111: // Postindexed
                        ADR_EFF_VAR_comb = FETCH_MEM_ADR ?
                            (ADR_MUX_comb + BASE_DISPL_REG) :
                            (MEM_ADR_REG + index_scaled_comb + OUTER_DISPL_REG);
                    4'b1001, 4'b1010, 4'b1011: // Memory indirect
                        ADR_EFF_VAR_comb = FETCH_MEM_ADR ?
                            (ADR_MUX_comb + BASE_DISPL_REG) :
                            (MEM_ADR_REG + OUTER_DISPL_REG);
                    default:
                        ADR_EFF_VAR_comb = 32'hxxxxxxxx; // Reserved
                endcase
            end
        end

        ADR_SPECIAL: begin  // Abs/PC/Imm
            case (AMODE_SEL)
                3'b000, 3'b001:
                    ADR_EFF_VAR_comb = ABS_ADDRESS_REG;
                3'b010: // (d16, PC)
                    ADR_EFF_VAR_comb = PCVAR_comb + BASE_DISPL_REG;
                3'b011: begin // (d8, PC, Xn) or full extension
                    if (!F_E) begin
                        ADR_EFF_VAR_comb = PCVAR_comb + BASE_DISPL_REG + index_scaled_comb;
                    end else begin
                        case (I_S_IS_comb)
                            4'b0000, 4'b1000:
                                ADR_EFF_VAR_comb = PCVAR_comb + BASE_DISPL_REG + index_scaled_comb;
                            4'b0001, 4'b0010, 4'b0011:
                                ADR_EFF_VAR_comb = FETCH_MEM_ADR ?
                                    (PCVAR_comb + BASE_DISPL_REG + index_scaled_comb) :
                                    (MEM_ADR_REG + OUTER_DISPL_REG);
                            4'b0101, 4'b0110, 4'b0111:
                                ADR_EFF_VAR_comb = FETCH_MEM_ADR ?
                                    (PCVAR_comb + BASE_DISPL_REG) :
                                    (MEM_ADR_REG + index_scaled_comb + OUTER_DISPL_REG);
                            4'b1001, 4'b1010, 4'b1011:
                                ADR_EFF_VAR_comb = FETCH_MEM_ADR ?
                                    (PCVAR_comb + BASE_DISPL_REG) :
                                    (MEM_ADR_REG + OUTER_DISPL_REG);
                            default:
                                ADR_EFF_VAR_comb = 32'hxxxxxxxx; // Reserved
                        endcase
                    end
                end
                default:
                    ADR_EFF_VAR_comb = 32'hxxxxxxxx;
            endcase
        end

        default:
            ADR_EFF_VAR_comb = 32'hxxxxxxxx; // Direct modes: no effective address
    endcase
end

assign ADR_EFF = ADR_EFF_I;
assign ADR_EFF_WB = ADR_WB[31:0];

// Data outputs:
assign AR_OUT_1_I = ISP_RD           ? ISP_REG :
                    MSP_RD           ? MSP_REG :
                    USP_RD           ? USP_REG :
                    (AR_PNTR_1 < 7)  ? AR[AR_PNTR_1] :
                    (SBIT &&  MBIT)  ? MSP_REG :
                    (SBIT && !MBIT)  ? ISP_REG : USP_REG;

assign AR_OUT_2_I = (AR_PNTR_2 < 7)  ? AR[AR_PNTR_2] :
                    (SBIT &&  MBIT)  ? MSP_REG :
                    (SBIT && !MBIT)  ? ISP_REG : USP_REG;

assign PC = PC_I;

always_ff @(posedge CLK) begin : program_counter
    // Note: PC_LOAD and PC_ADD_DISPL must be highest
    // prioritized. The reason is that in case of jumps
    // or branches the Ipipe is flushed in connection
    // with PC_INC. In such cases PC_LOAD or PC_ADD_DISPL
    // are asserted simultaneously with PC_INC.
    if (RESET)
        PC_I <= 32'h0;
    else if (PC_LOAD)
        PC_I <= AR_IN_1;
    else if (PC_ADD_DISPL)
        PC_I <= PC_I + DISPLACEMENT;
    else if (PC_RESTORE)
        PC_I <= AR_IN_1; // Keep prioritization!
    else if (PC_INC)
        PC_I <= PC_I + {24'd0, PC_OFFSET};
end

always_ff @(posedge CLK) begin : stack_pointers
    // The registers are modeled in a way that write and simultaneously
    // increment, decrement and others are possible for different registers.

    // ---------------------------------------- MSP section ----------------------------------------
    if (RESET)
        MSP_REG <= 32'h0;
    else if (AR_WR_1 && AR_PNTR_WB_1 == 7 && MSBIT == 2'b11)
        MSP_REG <= AR_IN_1; // Always written long.

    if (AR_INC && AR_PNTR_1 == 7 && SBIT && MBIT) begin
        case (OP_SIZE)
            BYTE:    MSP_REG <= MSP_REG + 32'd2; // Stack: byte increments by two.
            WORD:    MSP_REG <= MSP_REG + 32'd2;
            default: MSP_REG <= MSP_REG + 32'd4; // LONG
        endcase
    end

    if (AR_DEC && AR_PNTR_1 == 7 && SBIT && MBIT) begin
        case (OP_SIZE)
            BYTE:    MSP_REG <= MSP_REG - 32'd2; // Stack: byte decrements by two.
            WORD:    MSP_REG <= MSP_REG - 32'd2;
            default: MSP_REG <= MSP_REG - 32'd4; // LONG
        endcase
    end

    if (MSP_WR)
        MSP_REG <= AR_IN_1;
    else if (SP_ADD_DISPL && AR_INC && SBIT && MBIT)
        MSP_REG <= MSP_REG + DISPLACEMENT + 32'd4; // Used for RTD.
    else if (SP_ADD_DISPL && SBIT && MBIT)
        MSP_REG <= MSP_REG + DISPLACEMENT;

    // ---------------------------------------- ISP section ----------------------------------------
    if (RESET)
        ISP_REG <= 32'h0;
    else if (AR_WR_1 && AR_PNTR_WB_1 == 7 && MSBIT == 2'b01)
        ISP_REG <= AR_IN_1; // Always written long.

    if (AR_INC && AR_PNTR_1 == 7 && SBIT && !MBIT) begin
        case (OP_SIZE)
            BYTE:    ISP_REG <= ISP_REG + 32'd2; // Stack: byte increments by two.
            WORD:    ISP_REG <= ISP_REG + 32'd2;
            default: ISP_REG <= ISP_REG + 32'd4; // LONG
        endcase
    end

    if (ISP_DEC || (AR_DEC && AR_PNTR_1 == 7 && SBIT && !MBIT)) begin
        case (OP_SIZE)
            BYTE:    ISP_REG <= ISP_REG - 32'd2; // Stack: byte decrements by two.
            WORD:    ISP_REG <= ISP_REG - 32'd2;
            default: ISP_REG <= ISP_REG - 32'd4; // LONG
        endcase
    end

    if (ISP_WR)
        ISP_REG <= AR_IN_1;
    else if (SP_ADD_DISPL && AR_INC && SBIT && !MBIT)
        ISP_REG <= ISP_REG + DISPLACEMENT + 32'd4; // Used for RTD.
    else if (SP_ADD_DISPL && SBIT && !MBIT)
        ISP_REG <= ISP_REG + DISPLACEMENT;

    // ---------------------------------------- USP section ----------------------------------------
    if (RESET)
        USP_REG <= 32'h0;
    else if (AR_WR_1 && AR_PNTR_WB_1 == 7 && !MSBIT[0])
        USP_REG <= AR_IN_1; // Always written long.

    if (AR_INC && AR_PNTR_1 == 7 && !SBIT) begin
        case (OP_SIZE)
            BYTE:    USP_REG <= USP_REG + 32'd2; // Stack: byte increments by two.
            WORD:    USP_REG <= USP_REG + 32'd2;
            default: USP_REG <= USP_REG + 32'd4; // LONG
        endcase
    end

    if (AR_DEC && AR_PNTR_1 == 7 && !SBIT) begin
        case (OP_SIZE)
            BYTE:    USP_REG <= USP_REG - 32'd2; // Stack: byte decrements by two.
            WORD:    USP_REG <= USP_REG - 32'd2;
            default: USP_REG <= USP_REG - 32'd4; // LONG
        endcase
    end

    if (USP_WR)
        USP_REG <= AR_IN_1;
    else if (SP_ADD_DISPL && AR_INC && !SBIT)
        USP_REG <= USP_REG + DISPLACEMENT + 32'd4; // Used for RTD.
    else if (SP_ADD_DISPL && !SBIT)
        USP_REG <= USP_REG + DISPLACEMENT;

    // Write port 2: used for EXG and UNLK.
    if (AR_WR_2 && AR_PNTR_WB_2 == 7 && MSBIT == 2'b11)
        MSP_REG <= AR_IN_2;
    else if (AR_WR_2 && AR_PNTR_WB_2 == 7 && MSBIT == 2'b01)
        ISP_REG <= AR_IN_2;
    else if (AR_WR_2 && AR_PNTR_WB_2 == 7)
        USP_REG <= AR_IN_2;
end

always_ff @(posedge CLK) begin : address_registers_proc
    // Write and simultaneously increment/decrement are possible
    // for different registers.

    if (RESET) begin
        for (int i = 0; i < 7; i++)
            AR[i] <= 32'h0;
    end

    if (AR_WR_1 && AR_PNTR_WB_1 < 7)
        AR[AR_PNTR_WB_1] <= AR_IN_1; // Always written long.

    if (AR_INC && AR_PNTR_1 < 7) begin
        case (OP_SIZE)
            BYTE:    AR[AR_PNTR_1] <= AR[AR_PNTR_1] + 32'd1;
            WORD:    AR[AR_PNTR_1] <= AR[AR_PNTR_1] + 32'd2;
            default: AR[AR_PNTR_1] <= AR[AR_PNTR_1] + 32'd4;
        endcase
    end

    if (AR_DEC && AR_PNTR_1 < 7) begin
        case (OP_SIZE)
            BYTE:    AR[AR_PNTR_1] <= AR[AR_PNTR_1] - 32'd1;
            WORD:    AR[AR_PNTR_1] <= AR[AR_PNTR_1] - 32'd2;
            default: AR[AR_PNTR_1] <= AR[AR_PNTR_1] - 32'd4;
        endcase
    end

    if (AR_WR_2 && AR_PNTR_WB_2 < 7)
        AR[AR_PNTR_WB_2] <= AR_IN_2; // Used for EXG and UNLK.
end

always_ff @(posedge CLK) begin : fcodes
    // Alternate function code registers.
    if (DFC_WR)
        DFC_REG_sig <= AR_IN_1[2:0];

    if (SFC_WR)
        SFC_REG_sig <= AR_IN_1[2:0];

    DFC <= DFC_REG_sig;
    SFC <= SFC_REG_sig;
end

endmodule
