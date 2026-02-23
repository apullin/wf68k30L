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

always_ff @(posedge CLK) begin : INBUFFER
    if (AR_MARK_USED == 1'b1) begin
        AR_PNTR_WB_1 <= AR_SEL_WR_1;
        AR_PNTR_WB_2 <= AR_SEL_WR_2;
    end
end

assign AR_PNTR_1 = AR_SEL_RD_1;
assign AR_PNTR_2 = AR_SEL_RD_2;

always_ff @(posedge CLK) begin : P_IN_USE
    logic DELAY;
    if (RESET == 1'b1 || UNMARK == 1'b1) begin
        AR_USED_1[3] <= 1'b0;
        AR_USED_2[3] <= 1'b0;
    end else if (AR_MARK_USED == 1'b1) begin
        AR_USED_1 <= {1'b1, AR_SEL_WR_1};
        if (USE_APAIR == 1'b1) begin
            AR_USED_2 <= {1'b1, AR_SEL_WR_2};
        end
        MSBIT <= {MBIT, SBIT};
    end
    //
    if (RESET == 1'b1 || UNMARK == 1'b1) begin
        ADR_WB[32] <= 1'b0;
        DELAY = 1'b0;
    end else if (ADR_MARK_USED == 1'b1) begin
        DELAY = 1'b1; // One clock cycle address calculation delay.
    end else if (DELAY == 1'b1) begin
        ADR_WB <= {1'b1, ADR_EFF_I};
        DELAY = 1'b0;
    end
end

assign AR_IN_USE = (AR_USED_1[2:0] == 3'b111 && SBIT == 1'b1 && MSBIT[1] != MBIT) ? 1'b0 : // Wrong stack pointer.
                   (AR_USED_2[2:0] == 3'b111 && SBIT == 1'b1 && MSBIT[1] != MBIT) ? 1'b0 : // Wrong stack pointer.
                   (AR_USED_1[3] == 1'b1 && AR_USED_1[2:0] == AR_SEL_RD_1) ? 1'b1 :
                   (AR_USED_1[3] == 1'b1 && AR_USED_1[2:0] == AR_SEL_RD_2) ? 1'b1 :
                   (AR_USED_2[3] == 1'b1 && AR_USED_2[2:0] == AR_SEL_RD_1) ? 1'b1 :
                   (AR_USED_2[3] == 1'b1 && AR_USED_2[2:0] == AR_SEL_RD_2) ? 1'b1 : 1'b0;

assign AR_OUT_1 = AR_OUT_1_I;
assign AR_OUT_2 = AR_OUT_2_I;

assign ADR_IN_USE = (ADR_WB[32] == 1'b1 && ADR_WB[31:2] == ADR_EFF_I[31:2]) ? 1'b1 :  // Actual long word address.
                    (ADR_WB[32] == 1'b1 && ADR_WB[31:2] - 1'b1 == ADR_EFF_I[31:2]) ? 1'b1 :  // Lock a misaligned access.
                    (ADR_WB[32] == 1'b1 && ADR_WB[31:2] + 1'b1 == ADR_EFF_I[31:2]) ? 1'b1 : 1'b0; // Lock a misaligned access.

always_ff @(posedge CLK) begin : ADR_FORMAT_PROC
    if (STORE_ADR_FORMAT == 1'b1) begin
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
logic [31:0] INDEX_SCALED_REG;
logic [31:0] MEM_ADR_REG;
logic [31:0] BASE_DISPL_REG;
logic [31:0] OUTER_DISPL_REG;
logic [31:0] ABS_ADDRESS_REG;

always_ff @(posedge CLK) begin : ADDRESS_MODES_REG
    // INDEX selection
    if (STORE_ADR_FORMAT == 1'b1 && EXT_WORD[15] == 1'b0 && EXT_WORD[11] == 1'b1) begin
        INDEX_REG <= INDEX_IN; // Long data register.
    end else if (STORE_ADR_FORMAT == 1'b1 && EXT_WORD[15] == 1'b0) begin
        INDEX_REG <= {{16{INDEX_IN[15]}}, INDEX_IN[15:0]}; // Sign extended data register.
    end else if (STORE_ADR_FORMAT == 1'b1 && EXT_WORD[11] == 1'b1) begin // Long address register.
        if (EXT_WORD[14:12] == 3'b111 && SBIT == 1'b1 && MBIT == 1'b1)
            INDEX_REG <= MSP_REG;
        else if (EXT_WORD[14:12] == 3'b111 && SBIT == 1'b1 && MBIT == 1'b0)
            INDEX_REG <= ISP_REG;
        else if (EXT_WORD[14:12] == 3'b111 && SBIT == 1'b0)
            INDEX_REG <= USP_REG;
        else
            INDEX_REG <= AR[EXT_WORD[14:12]];
    end else if (STORE_ADR_FORMAT == 1'b1) begin // Sign extended address register.
        if (EXT_WORD[14:12] == 3'b111 && SBIT == 1'b1 && MBIT == 1'b1)
            INDEX_REG <= {{16{MSP_REG[15]}}, MSP_REG[15:0]};
        else if (EXT_WORD[14:12] == 3'b111 && SBIT == 1'b1 && MBIT == 1'b0)
            INDEX_REG <= {{16{ISP_REG[15]}}, ISP_REG[15:0]};
        else if (EXT_WORD[14:12] == 3'b111 && SBIT == 1'b0)
            INDEX_REG <= {{16{USP_REG[15]}}, USP_REG[15:0]};
        else
            INDEX_REG <= {{16{AR[EXT_WORD[14:12]][15]}}, AR[EXT_WORD[14:12]][15:0]};
    end
    //
    case (SCALE)
        2'b00: INDEX_SCALED_REG <= INDEX_REG; // Multiple by 1.
        2'b01: INDEX_SCALED_REG <= {INDEX_REG[30:0], 1'b0}; // Multiple by 2.
        2'b10: INDEX_SCALED_REG <= {INDEX_REG[29:0], 2'b00}; // Multiple by 4.
        default: INDEX_SCALED_REG <= {INDEX_REG[28:0], 3'b000}; // Multiple by 8.
    endcase
    //
    // Register for memory indirect addressing modes.
    if (STORE_MEM_ADR == 1'b1) begin
        MEM_ADR_REG <= AR_IN_1;
    end
    //
    // Base displacement
    if (RESET == 1'b1) begin
        BASE_DISPL_REG <= 32'h00000000; // Null base displacement.
    end else if (STORE_ADR_FORMAT == 1'b1 && EXT_WORD[8] == 1'b1 && EXT_WORD[5:4] == 2'b01) begin
        BASE_DISPL_REG <= 32'h00000000; // Null base displacement.
    end else if (STORE_ADR_FORMAT == 1'b1 && EXT_WORD[8] == 1'b0) begin
        BASE_DISPL_REG <= {{24{EXT_WORD[7]}}, EXT_WORD[7:0]};
    end else if (STORE_D16 == 1'b1) begin
        BASE_DISPL_REG <= {{16{EXT_WORD[15]}}, EXT_WORD};
    end else if (STORE_D32_LO == 1'b1) begin
        if (BD_SIZE == 2'b10) begin // Word displacement.
            BASE_DISPL_REG[31:16] <= {16{EXT_WORD[15]}};
        end
        BASE_DISPL_REG[15:0] <= EXT_WORD;
    end else if (STORE_D32_HI == 1'b1) begin
        BASE_DISPL_REG[31:16] <= EXT_WORD;
    end else if (STORE_DISPL == 1'b1) begin
        BASE_DISPL_REG <= DISPLACEMENT;
    end
    //
    // Outer displacement
    if (STORE_ADR_FORMAT == 1'b1 && EXT_WORD[8] == 1'b1 && EXT_WORD[1:0] == 2'b01) begin
        OUTER_DISPL_REG <= 32'h00000000; // Null outer displacement.
    end else if (STORE_OD_LO == 1'b1) begin
        if (I_IS[1:0] == 2'b10) begin
            OUTER_DISPL_REG[31:16] <= {16{EXT_WORD[15]}};
        end
        OUTER_DISPL_REG[15:0] <= EXT_WORD;
    end else if (STORE_OD_HI == 1'b1) begin
        OUTER_DISPL_REG[31:16] <= EXT_WORD;
    end
    //
    // Absolute address
    if (STORE_ABS_LO == 1'b1) begin
        if (AMODE_SEL == 3'b000) begin
            ABS_ADDRESS_REG[31:16] <= {16{EXT_WORD[15]}};
        end
        ABS_ADDRESS_REG[15:0] <= EXT_WORD;
    end else if (STORE_ABS_HI == 1'b1) begin
        ABS_ADDRESS_REG[31:16] <= EXT_WORD;
    end
    //
    // Effective address register
    if (RESTORE_ISP_PC == 1'b1) begin
        ADR_EFF_I <= ADR_OFFSET; // During system initialization.
        ADR_EFF_TMP_REG <= ADR_EFF_VAR_comb;
    end else if (STORE_AEFF == 1'b1) begin // Used for MOVEM.
        ADR_EFF_I <= ADR_EFF_TMP_REG + ADR_OFFSET; // Keep the effective address. See also CONTROL section.
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

always_comb begin
    I_S_IS_comb = {I_S, I_IS};
    PCVAR_comb = PC_I + {28'd0, PC_EW_OFFSET}; // This is the address of the extension word.

    // Address mux
    if (ADR_MODE == 3'b110 && FETCH_MEM_ADR == 1'b1 && F_E == 1'b1 && B_S == 1'b1)
        ADR_MUX_comb = 32'h00000000; // Base register suppress.
    else if (ADR_MODE == 3'b111 && FETCH_MEM_ADR == 1'b1 && AMODE_SEL == 3'b011 && F_E == 1'b1 && B_S == 1'b1)
        ADR_MUX_comb = 32'h00000000; // Base register suppress.
    else if (USE_DREG == 1'b1)
        ADR_MUX_comb = AR_IN_1;
    else begin
        case (AR_PNTR_1)
            7: begin
                if (SBIT == 1'b1 && MBIT == 1'b0)
                    ADR_MUX_comb = ISP_REG;
                else if (SBIT == 1'b1)
                    ADR_MUX_comb = MSP_REG;
                else
                    ADR_MUX_comb = USP_REG;
            end
            default: ADR_MUX_comb = AR[AR_PNTR_1];
        endcase
    end

    // Effective address computation
    case (ADR_MODE)
        // "000" | "001" => Direct address modes: no effective address required.
        3'b010, 3'b011, 3'b100:
            ADR_EFF_VAR_comb = ADR_MUX_comb; // (An), (An)+, -(An).
        3'b101: // Address register indirect with offset. Assembler syntax: (d16,An).
            ADR_EFF_VAR_comb = ADR_MUX_comb + BASE_DISPL_REG; // (d16,An).
        3'b110: begin
            if (F_E == 1'b0) begin // Brief extension word.
                ADR_EFF_VAR_comb = ADR_MUX_comb + BASE_DISPL_REG + INDEX_SCALED_REG; // (d8, An, Xn, SIZE*SCALE).
            end else begin // Full extension word.
                case (I_S_IS_comb)
                    4'b0000, 4'b1000: // No memory indirect action.
                        ADR_EFF_VAR_comb = ADR_MUX_comb + BASE_DISPL_REG + INDEX_SCALED_REG; // (bd, An, Xn, SIZE*SCALE).
                    4'b0001, 4'b0010, 4'b0011: begin // Memory indirect preindexed.
                        if (FETCH_MEM_ADR == 1'b1)
                            ADR_EFF_VAR_comb = ADR_MUX_comb + BASE_DISPL_REG + INDEX_SCALED_REG;
                        else
                            ADR_EFF_VAR_comb = MEM_ADR_REG + OUTER_DISPL_REG;
                    end
                    4'b0101, 4'b0110, 4'b0111: begin // Memory indirect postindexed.
                        if (FETCH_MEM_ADR == 1'b1)
                            ADR_EFF_VAR_comb = ADR_MUX_comb + BASE_DISPL_REG;
                        else
                            ADR_EFF_VAR_comb = MEM_ADR_REG + INDEX_SCALED_REG + OUTER_DISPL_REG;
                    end
                    4'b1001, 4'b1010, 4'b1011: begin // Memory indirect.
                        if (FETCH_MEM_ADR == 1'b1)
                            ADR_EFF_VAR_comb = ADR_MUX_comb + BASE_DISPL_REG;
                        else
                            ADR_EFF_VAR_comb = MEM_ADR_REG + OUTER_DISPL_REG;
                    end
                    default:
                        ADR_EFF_VAR_comb = 32'hxxxxxxxx; // Reserved, don't care.
                endcase
            end
        end
        3'b111: begin
            case (AMODE_SEL)
                3'b000, 3'b001:
                    ADR_EFF_VAR_comb = ABS_ADDRESS_REG;
                3'b010: // (d16, PC).
                    ADR_EFF_VAR_comb = PCVAR_comb + BASE_DISPL_REG;
                3'b011: begin
                    if (F_E == 1'b0) begin // Brief extension word.
                        ADR_EFF_VAR_comb = PCVAR_comb + BASE_DISPL_REG + INDEX_SCALED_REG; // (d8, PC, Xn, SIZE*SCALE).
                    end else begin // Full extension word.
                        case (I_S_IS_comb)
                            4'b0000, 4'b1000: // No memory indirect action.
                                ADR_EFF_VAR_comb = PCVAR_comb + BASE_DISPL_REG + INDEX_SCALED_REG; // (bd, PC, Xn, SIZE*SCALE).
                            4'b0001, 4'b0010, 4'b0011: begin // Memory indirect preindexed.
                                if (FETCH_MEM_ADR == 1'b1)
                                    ADR_EFF_VAR_comb = PCVAR_comb + BASE_DISPL_REG + INDEX_SCALED_REG;
                                else
                                    ADR_EFF_VAR_comb = MEM_ADR_REG + OUTER_DISPL_REG;
                            end
                            4'b0101, 4'b0110, 4'b0111: begin // Memory indirect postindexed.
                                if (FETCH_MEM_ADR == 1'b1)
                                    ADR_EFF_VAR_comb = PCVAR_comb + BASE_DISPL_REG;
                                else
                                    ADR_EFF_VAR_comb = MEM_ADR_REG + INDEX_SCALED_REG + OUTER_DISPL_REG;
                            end
                            4'b1001, 4'b1010, 4'b1011: begin // Memory indirect.
                                if (FETCH_MEM_ADR == 1'b1)
                                    ADR_EFF_VAR_comb = PCVAR_comb + BASE_DISPL_REG;
                                else
                                    ADR_EFF_VAR_comb = MEM_ADR_REG + OUTER_DISPL_REG;
                            end
                            default:
                                ADR_EFF_VAR_comb = 32'hxxxxxxxx; // Reserved, don't care.
                        endcase
                    end
                end
                default:
                    ADR_EFF_VAR_comb = 32'hxxxxxxxx; // Don't care, while not used.
            endcase
        end
        default:
            ADR_EFF_VAR_comb = 32'hxxxxxxxx; // Result not required.
    endcase
end

assign ADR_EFF = ADR_EFF_I;
assign ADR_EFF_WB = ADR_WB[31:0];

// Data outputs:
assign AR_OUT_1_I = (ISP_RD == 1'b1) ? ISP_REG :
                    (MSP_RD == 1'b1) ? MSP_REG :
                    (USP_RD == 1'b1) ? USP_REG :
                    (AR_PNTR_1 < 7)  ? AR[AR_PNTR_1] :
                    (SBIT == 1'b1 && MBIT == 1'b1) ? MSP_REG :
                    (SBIT == 1'b1 && MBIT == 1'b0) ? ISP_REG : USP_REG;

assign AR_OUT_2_I = (AR_PNTR_2 < 7)  ? AR[AR_PNTR_2] :
                    (SBIT == 1'b1 && MBIT == 1'b1) ? MSP_REG :
                    (SBIT == 1'b1 && MBIT == 1'b0) ? ISP_REG : USP_REG;

assign PC = PC_I;

always_ff @(posedge CLK) begin : PROGRAM_COUNTER
    // Note: PC_LOAD and PC_ADD_DISPL must be highest
    // prioritized. The reason is that in case of jumps
    // or branches the Ipipe is flushed in connection
    // with PC_INC. In such cases PC_LOAD or PC_ADD_DISPL
    // are asserted simultaneously with PC_INC.
    if (RESET == 1'b1)
        PC_I <= 32'h00000000;
    else if (PC_LOAD == 1'b1)
        PC_I <= AR_IN_1;
    else if (PC_ADD_DISPL == 1'b1)
        PC_I <= PC_I + DISPLACEMENT;
    else if (PC_RESTORE == 1'b1)
        PC_I <= AR_IN_1; // Keep prioritization!
    else if (PC_INC == 1'b1)
        PC_I <= PC_I + {24'd0, PC_OFFSET};
end

always_ff @(posedge CLK) begin : STACK_POINTERS
    // The registers are modeled in a way
    // that write and simultaneously increment
    // decrement and others are possible for
    // different registers.

    // ---------------------------------------- MSP section ----------------------------------------
    if (RESET == 1'b1)
        MSP_REG <= 32'h00000000;
    else if (AR_WR_1 == 1'b1 && AR_PNTR_WB_1 == 7 && MSBIT == 2'b11)
        MSP_REG <= AR_IN_1; // Always written long.

    if (AR_INC == 1'b1 && AR_PNTR_1 == 7 && SBIT == 1'b1 && MBIT == 1'b1) begin
        case (OP_SIZE)
            BYTE:    MSP_REG <= MSP_REG + 32'd2; // Increment by two!
            WORD:    MSP_REG <= MSP_REG + 32'd2; // Increment by two.
            default: MSP_REG <= MSP_REG + 32'd4; // Increment by four, (LONG).
        endcase
    end

    if (AR_DEC == 1'b1 && AR_PNTR_1 == 7 && SBIT == 1'b1 && MBIT == 1'b1) begin
        case (OP_SIZE)
            BYTE:    MSP_REG <= MSP_REG - 32'd2; // Decrement by two!
            WORD:    MSP_REG <= MSP_REG - 32'd2; // Decrement by two.
            default: MSP_REG <= MSP_REG - 32'd4; // Decrement by four, (LONG).
        endcase
    end

    if (MSP_WR == 1'b1)
        MSP_REG <= AR_IN_1;
    else if (SP_ADD_DISPL == 1'b1 && AR_INC == 1'b1 && SBIT == 1'b1 && MBIT == 1'b1)
        MSP_REG <= MSP_REG + DISPLACEMENT + 32'd4; // Used for RTD.
    else if (SP_ADD_DISPL == 1'b1 && SBIT == 1'b1 && MBIT == 1'b1)
        MSP_REG <= MSP_REG + DISPLACEMENT;

    // ---------------------------------------- ISP section ----------------------------------------
    if (RESET == 1'b1)
        ISP_REG <= 32'h00000000;
    else if (AR_WR_1 == 1'b1 && AR_PNTR_WB_1 == 7 && MSBIT == 2'b01)
        ISP_REG <= AR_IN_1; // Always written long.

    if (AR_INC == 1'b1 && AR_PNTR_1 == 7 && SBIT == 1'b1 && MBIT == 1'b0) begin
        case (OP_SIZE)
            BYTE:    ISP_REG <= ISP_REG + 32'd2; // Increment by two!
            WORD:    ISP_REG <= ISP_REG + 32'd2; // Increment by two.
            default: ISP_REG <= ISP_REG + 32'd4; // Increment by four, (LONG).
        endcase
    end

    if (ISP_DEC == 1'b1 || (AR_DEC == 1'b1 && AR_PNTR_1 == 7 && SBIT == 1'b1 && MBIT == 1'b0)) begin
        case (OP_SIZE)
            BYTE:    ISP_REG <= ISP_REG - 32'd2; // Decrement by two!
            WORD:    ISP_REG <= ISP_REG - 32'd2; // Decrement by two.
            default: ISP_REG <= ISP_REG - 32'd4; // Decrement by four, (LONG).
        endcase
    end

    if (ISP_WR == 1'b1)
        ISP_REG <= AR_IN_1;
    else if (SP_ADD_DISPL == 1'b1 && AR_INC == 1'b1 && SBIT == 1'b1 && MBIT == 1'b0)
        ISP_REG <= ISP_REG + DISPLACEMENT + 32'd4; // Used for RTD.
    else if (SP_ADD_DISPL == 1'b1 && SBIT == 1'b1 && MBIT == 1'b0)
        ISP_REG <= ISP_REG + DISPLACEMENT;

    // ---------------------------------------- USP section ----------------------------------------
    if (RESET == 1'b1)
        USP_REG <= 32'h00000000;
    else if (AR_WR_1 == 1'b1 && AR_PNTR_WB_1 == 7 && MSBIT[0] == 1'b0)
        USP_REG <= AR_IN_1; // Always written long.

    if (AR_INC == 1'b1 && AR_PNTR_1 == 7 && SBIT == 1'b0) begin
        case (OP_SIZE)
            BYTE:    USP_REG <= USP_REG + 32'd2; // Increment by two!
            WORD:    USP_REG <= USP_REG + 32'd2; // Increment by two.
            default: USP_REG <= USP_REG + 32'd4; // Increment by four, (LONG).
        endcase
    end

    if (AR_DEC == 1'b1 && AR_PNTR_1 == 7 && SBIT == 1'b0) begin
        case (OP_SIZE)
            BYTE:    USP_REG <= USP_REG - 32'd2; // Decrement by two!
            WORD:    USP_REG <= USP_REG - 32'd2; // Decrement by two.
            default: USP_REG <= USP_REG - 32'd4; // Decrement by four, (LONG).
        endcase
    end

    if (USP_WR == 1'b1)
        USP_REG <= AR_IN_1;
    else if (SP_ADD_DISPL == 1'b1 && AR_INC == 1'b1 && SBIT == 1'b0)
        USP_REG <= USP_REG + DISPLACEMENT + 32'd4; // Used for RTD.
    else if (SP_ADD_DISPL == 1'b1 && SBIT == 1'b0)
        USP_REG <= USP_REG + DISPLACEMENT;

    if (AR_WR_2 == 1'b1 && AR_PNTR_WB_2 == 7 && MSBIT == 2'b11)
        MSP_REG <= AR_IN_2; // Used for EXG and UNLK.
    else if (AR_WR_2 == 1'b1 && AR_PNTR_WB_2 == 7 && MSBIT == 2'b01)
        ISP_REG <= AR_IN_2; // Used for EXG and UNLK.
    else if (AR_WR_2 == 1'b1 && AR_PNTR_WB_2 == 7)
        USP_REG <= AR_IN_2; // Used for EXG and UNLK.
end

always_ff @(posedge CLK) begin : ADDRESS_REGISTERS_PROC
    // The registers are modeled in a way
    // that write and simultaneously increment
    // decrement and others are possible for
    // different registers.

    if (RESET == 1'b1) begin
        AR[0] <= 32'h00000000;
        AR[1] <= 32'h00000000;
        AR[2] <= 32'h00000000;
        AR[3] <= 32'h00000000;
        AR[4] <= 32'h00000000;
        AR[5] <= 32'h00000000;
        AR[6] <= 32'h00000000;
    end

    if (AR_WR_1 == 1'b1 && AR_PNTR_WB_1 < 7)
        AR[AR_PNTR_WB_1] <= AR_IN_1; // Always written long.

    if (AR_INC == 1'b1 && AR_PNTR_1 < 7) begin
        case (OP_SIZE)
            BYTE:    AR[AR_PNTR_1] <= AR[AR_PNTR_1] + 32'd1;
            WORD:    AR[AR_PNTR_1] <= AR[AR_PNTR_1] + 32'd2;
            default: AR[AR_PNTR_1] <= AR[AR_PNTR_1] + 32'd4;
        endcase
    end

    if (AR_DEC == 1'b1 && AR_PNTR_1 < 7) begin
        case (OP_SIZE)
            BYTE:    AR[AR_PNTR_1] <= AR[AR_PNTR_1] - 32'd1;
            WORD:    AR[AR_PNTR_1] <= AR[AR_PNTR_1] - 32'd2;
            default: AR[AR_PNTR_1] <= AR[AR_PNTR_1] - 32'd4;
        endcase
    end

    if (AR_WR_2 == 1'b1 && AR_PNTR_WB_2 < 7)
        AR[AR_PNTR_WB_2] <= AR_IN_2; // Used for EXG and UNLK.
end

always_ff @(posedge CLK) begin : FCODES
    // These flip flops provide the alternate function
    // code registers.
    if (DFC_WR == 1'b1) begin
        DFC_REG_sig <= AR_IN_1[2:0];
    end
    //
    if (SFC_WR == 1'b1) begin
        SFC_REG_sig <= AR_IN_1[2:0];
    end
    //
    DFC <= DFC_REG_sig;
    SFC <= SFC_REG_sig;
end

endmodule
