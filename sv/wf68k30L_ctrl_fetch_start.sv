//--------------------------------------------------------------------//
//                                                                    //
// WF68K30L IP Core: Fetch state START_OP next-state decoder.         //
//                                                                    //
// This module contains the combinational next-state logic for the    //
// START_OP state of the fetch pipeline state machine.                //
// Extracted from wf68k30L_ctrl_fetch_dec.sv for maintainability.     //
//                                                                    //
//--------------------------------------------------------------------//

module WF68K30L_CTRL_FETCH_START (
    // Operation
    input  logic [6:0]  OP,
    input  logic [13:0] BIW_0,
    input  logic [15:0] BIW_1,

    // Data availability
    input  logic        OPD_ACK,
    input  logic        OW_RDY,

    // Hazard signals
    input  logic        DR_IN_USE,
    input  logic        AR_IN_USE,

    // ALU
    input  logic        ALU_BSY,

    // Internal state
    input  logic [1:0]  OP_SIZE_I,

    // Output
    output logic [4:0]  NEXT_FETCH_STATE
);

`include "wf68k30L_pkg.svh"

localparam logic [4:0]
    START_OP       = 5'd0,
    CALC_AEFF      = 5'd1,
    FETCH_DISPL    = 5'd2,
    FETCH_EXWORD_1 = 5'd3,
    FETCH_ABS_HI   = 5'd8,
    FETCH_ABS_LO   = 5'd9,
    FETCH_IDATA_B2 = 5'd10,
    FETCH_IDATA_B1 = 5'd11,
    FETCH_OPERAND  = 5'd13,
    INIT_EXEC_WB   = 5'd14,
    SWITCH_STATE   = 5'd16;

function automatic logic imm_is_long_startgrp2(
    input logic [6:0]  op_i,
    input logic [13:0] biw0_i
);
begin
    // Decode long-immediate width directly from opcode bits for START_OP
    // immediate-source cases to avoid a combinational dependency on OP_SIZE_I.
    case (op_i)
        ADDA, CMPA, SUBA: imm_is_long_startgrp2 = (biw0_i[8:7] == 2'b11);
        MOVEA:            imm_is_long_startgrp2 = (biw0_i[13:12] == 2'b10);
        CHK:              imm_is_long_startgrp2 = (biw0_i[8:7] == 2'b10);
        CHK2, CMP2:       imm_is_long_startgrp2 = (biw0_i[10:9] == 2'b10);
        DIVS, DIVU,
        MULS, MULU:       imm_is_long_startgrp2 = !biw0_i[7];
        default:          imm_is_long_startgrp2 = 1'b0;
    endcase
end
endfunction

always_comb begin : start_op_dec
    logic imm_is_long_startgrp1;
    logic imm_is_long_startgrp2_v;
    imm_is_long_startgrp1 = (BIW_0[7:6] == 2'b10);
    imm_is_long_startgrp2_v = imm_is_long_startgrp2(OP, BIW_0);

    if (!OPD_ACK && !OW_RDY) begin
        NEXT_FETCH_STATE = START_OP;
    end else begin
        case (OP)
            ILLEGAL, RTE, TRAP, UNIMPLEMENTED: begin
                NEXT_FETCH_STATE = START_OP;
            end
            DBcc, EXT, EXTB, MOVEQ, SWAP: begin
                if (DR_IN_USE) begin
                    NEXT_FETCH_STATE = START_OP; // Wait, ADH.
                end else begin
                    NEXT_FETCH_STATE = INIT_EXEC_WB; // Proceed.
                end
            end
            ABCD, SBCD, ADDX, SUBX, PACK, UNPK: begin
                if (!BIW_0[3] && !DR_IN_USE) begin // Check for destination addressing register. ADH.
                    NEXT_FETCH_STATE = INIT_EXEC_WB; // Register to register.
                end else if (BIW_0[3] && !AR_IN_USE) begin // Check for destination addressing register. ADH.
                    NEXT_FETCH_STATE = CALC_AEFF; // Memory to memory.
                end else begin
                    NEXT_FETCH_STATE = START_OP; // Wait, ADH.
                end
            end
            ADD, ADDI, ADDQ, AND_B, ANDI, CAS, CMP, CMPI, EOR, EORI,
            NBCD, NEG, NEGX, NOT_B, OR_B, ORI, SUB, SUBI, SUBQ, TST, TAS: begin
                // These instructions have to take the destination into aspect
                // because the destination is an ALU operand and may cause data hazards.
                case (BIW_0[5:3])
                    3'b000: begin // Dn.
                        if (!DR_IN_USE) begin
                            NEXT_FETCH_STATE = INIT_EXEC_WB;
                        end else begin
                            NEXT_FETCH_STATE = START_OP;
                        end
                    end
                    3'b001: begin // An.
                        if ((OP == ADD || OP == SUB || OP == AND_B || OP == EOR || OP == OR_B || OP == CMP) && (AR_IN_USE || DR_IN_USE)) begin // ADH.
                            NEXT_FETCH_STATE = START_OP;
                        end else if (!AR_IN_USE) begin
                            NEXT_FETCH_STATE = INIT_EXEC_WB;
                        end else begin
                            NEXT_FETCH_STATE = START_OP;
                        end
                    end
                    3'b010, 3'b011: begin // (An), (An)+.
                        if (AR_IN_USE) begin // ADH.
                            NEXT_FETCH_STATE = START_OP; // Wait, ADH.
                        end else begin
                            NEXT_FETCH_STATE = FETCH_OPERAND;
                        end
                    end
                    3'b100: begin // -(An).
                        if (AR_IN_USE) begin // ADH.
                            NEXT_FETCH_STATE = START_OP; // Wait, ADH.
                        end else begin
                            NEXT_FETCH_STATE = CALC_AEFF;
                        end
                    end
                    3'b101: begin
                        NEXT_FETCH_STATE = FETCH_DISPL;
                    end
                    3'b110: begin
                        NEXT_FETCH_STATE = FETCH_EXWORD_1;
                    end
                    default: begin // 3'b111
                        if (BIW_0[2:0] == 3'b000) begin
                            NEXT_FETCH_STATE = FETCH_ABS_LO;
                        end else if (BIW_0[2:0] == 3'b001) begin
                            NEXT_FETCH_STATE = FETCH_ABS_HI;
                        end else if (BIW_0[2:0] == 3'b100 && imm_is_long_startgrp1) begin
                            NEXT_FETCH_STATE = FETCH_IDATA_B2;
                        end else if (BIW_0[2:0] == 3'b100) begin // Word or byte.
                            NEXT_FETCH_STATE = FETCH_IDATA_B1;
                        end else if (BIW_0[2:0] == 3'b010) begin
                            NEXT_FETCH_STATE = FETCH_DISPL;
                        end else begin
                            NEXT_FETCH_STATE = FETCH_EXWORD_1;
                        end
                    end
                endcase
            end
            ADDA, BCHG, BCLR, BFCHG, BFCLR, BFEXTS, BFEXTU, BFFFO, BFINS, BFSET, BFTST, BSET, BTST, CHK, CHK2, CMP2, CMPA,
            DIVS, DIVU, MULS, MULU, MOVEA, MOVE_TO_CCR, MOVE_TO_SR, SUBA: begin
                case (BIW_0[5:3])
                    3'b000: begin // Source is Dn.
                        if ((OP == ADDA || OP == SUBA || OP == CMPA || OP == MOVEA) && (AR_IN_USE || DR_IN_USE)) begin // ADH.
                            NEXT_FETCH_STATE = START_OP;
                        end else if (!DR_IN_USE) begin
                            NEXT_FETCH_STATE = INIT_EXEC_WB;
                        end else begin
                            NEXT_FETCH_STATE = START_OP;
                        end
                    end
                    3'b001: begin // Valid for ADDA, CMPA, MOVEA, SUBA; source is An.
                        if (!AR_IN_USE) begin
                            NEXT_FETCH_STATE = INIT_EXEC_WB; // ADH.
                        end else begin
                            NEXT_FETCH_STATE = START_OP;
                        end
                    end
                    3'b010, 3'b011: begin // (An), (An)+.
                        if (AR_IN_USE) begin // ADH.
                            NEXT_FETCH_STATE = START_OP; // Wait, ADH!
                        end else begin
                            NEXT_FETCH_STATE = FETCH_OPERAND;
                        end
                    end
                    3'b100: begin // -(An).
                        if (AR_IN_USE) begin // ADH.
                            NEXT_FETCH_STATE = START_OP; // Wait, ADH!
                        end else begin
                            NEXT_FETCH_STATE = CALC_AEFF;
                        end
                    end
                    3'b101: begin
                        NEXT_FETCH_STATE = FETCH_DISPL;
                    end
                    3'b110: begin
                        NEXT_FETCH_STATE = FETCH_EXWORD_1;
                    end
                    default: begin // 3'b111
                        if (BIW_0[2:0] == 3'b000) begin
                            NEXT_FETCH_STATE = FETCH_ABS_LO;
                        end else if (BIW_0[2:0] == 3'b001) begin
                            NEXT_FETCH_STATE = FETCH_ABS_HI;
                        end else if (BIW_0[2:0] == 3'b100 && imm_is_long_startgrp2_v) begin
                            NEXT_FETCH_STATE = FETCH_IDATA_B2;
                        end else if (BIW_0[2:0] == 3'b100) begin // Word or Byte.
                            NEXT_FETCH_STATE = FETCH_IDATA_B1;
                        end else if (BIW_0[2:0] == 3'b010) begin
                            NEXT_FETCH_STATE = FETCH_DISPL;
                        end else begin
                            NEXT_FETCH_STATE = FETCH_EXWORD_1;
                        end
                    end
                endcase
            end
            EXG: begin
                if (BIW_0[7:3] == 5'b10001 && (DR_IN_USE || AR_IN_USE)) begin
                    NEXT_FETCH_STATE = START_OP; // Wait, ADH.
                end else if (BIW_0[7:3] == 5'b01000 && DR_IN_USE) begin
                    NEXT_FETCH_STATE = START_OP; // Wait, ADH.
                end else if (BIW_0[7:3] == 5'b01001 && AR_IN_USE) begin
                    NEXT_FETCH_STATE = START_OP; // Wait, ADH.
                end else begin
                    NEXT_FETCH_STATE = INIT_EXEC_WB;
                end
            end
            MOVE_FROM_CCR, MOVE_FROM_SR: begin
                if (!ALU_BSY) begin
                    case (BIW_0[5:3])
                        3'b000: begin // Destination is Dn.
                            if (!DR_IN_USE) begin
                                NEXT_FETCH_STATE = INIT_EXEC_WB;
                            end else begin
                                NEXT_FETCH_STATE = START_OP;
                            end
                        end
                        3'b010, 3'b011, 3'b100: begin // (An), (An)+, -(An).
                            if (!AR_IN_USE) begin
                                NEXT_FETCH_STATE = INIT_EXEC_WB;
                            end else begin
                                NEXT_FETCH_STATE = START_OP; // Wait, ADH.
                            end
                        end
                        3'b101: begin
                            NEXT_FETCH_STATE = FETCH_DISPL;
                        end
                        3'b110: begin
                            NEXT_FETCH_STATE = FETCH_EXWORD_1;
                        end
                        default: begin // 3'b111
                            if (BIW_0[2:0] == 3'b000) begin
                                NEXT_FETCH_STATE = FETCH_ABS_LO;
                            end else begin
                                NEXT_FETCH_STATE = FETCH_ABS_HI;
                            end
                        end
                    endcase
                end else begin
                    NEXT_FETCH_STATE = START_OP;
                end
            end
            ASL, ASR, LSL, LSR, ROTL, ROTR, ROXL, ROXR: begin
                if (BIW_0[7:6] != 2'b11) begin // Register shifts.
                    if (!DR_IN_USE) begin
                        NEXT_FETCH_STATE = INIT_EXEC_WB; // ADH.
                    end else begin
                        NEXT_FETCH_STATE = START_OP;
                    end
                end else begin // Memory shifts.
                    case (BIW_0[5:3])
                        3'b010, 3'b011: begin // (An), (An)+.
                            if (!AR_IN_USE) begin // ADH.
                                NEXT_FETCH_STATE = FETCH_OPERAND;
                            end else begin
                                NEXT_FETCH_STATE = START_OP;
                            end
                        end
                        3'b100: begin // -(An).
                            if (!AR_IN_USE) begin // ADH.
                                NEXT_FETCH_STATE = CALC_AEFF;
                            end else begin
                                NEXT_FETCH_STATE = START_OP;
                            end
                        end
                        3'b101: begin
                            NEXT_FETCH_STATE = FETCH_DISPL;
                        end
                        3'b110: begin
                            NEXT_FETCH_STATE = FETCH_EXWORD_1;
                        end
                        default: begin // 3'b111.
                            if (BIW_0[2:0] == 3'b000) begin
                                NEXT_FETCH_STATE = FETCH_ABS_LO;
                            end else begin
                                NEXT_FETCH_STATE = FETCH_ABS_HI;
                            end
                        end
                    endcase
                end
            end
            BKPT: begin
                // Wait until the bus controller is free to avoid a structural
                // hazard due to the top level function code multiplexer which
                // switches on the CPU_SPACE selector.
                if (!ALU_BSY) begin
                    NEXT_FETCH_STATE = FETCH_OPERAND;
                end else begin
                    NEXT_FETCH_STATE = START_OP;
                end
            end
            CAS2, CMPM, RTD, RTR, RTS: begin
                if (!AR_IN_USE) begin // ADH.
                    NEXT_FETCH_STATE = FETCH_OPERAND;
                end else begin
                    NEXT_FETCH_STATE = START_OP;
                end
            end
            CLR, JMP, JSR, LEA, PEA, PFLUSH, PLOAD, PTEST, Scc: begin // No read access required.
                case (BIW_0[5:3])
                    3'b000: begin // CLR, Scc.
                        if (!DR_IN_USE) begin
                            NEXT_FETCH_STATE = INIT_EXEC_WB;
                        end else begin
                            NEXT_FETCH_STATE = START_OP;
                        end
                    end
                    3'b001, 3'b010, 3'b011, 3'b100: begin
                        if (AR_IN_USE) begin
                            NEXT_FETCH_STATE = START_OP; // Wait, ADH.
                        end else if (OP == LEA || OP == PEA) begin
                            NEXT_FETCH_STATE = SWITCH_STATE;
                        end else begin
                            NEXT_FETCH_STATE = INIT_EXEC_WB;
                        end
                    end
                    3'b101: begin
                        NEXT_FETCH_STATE = FETCH_DISPL;
                    end
                    3'b110: begin
                        NEXT_FETCH_STATE = FETCH_EXWORD_1;
                    end
                    default: begin // 3'b111
                        if (BIW_0[2:0] == 3'b000) begin
                            NEXT_FETCH_STATE = FETCH_ABS_LO;
                        end else if (BIW_0[2:0] == 3'b001) begin
                            NEXT_FETCH_STATE = FETCH_ABS_HI;
                        end else if (BIW_0[2:0] == 3'b010) begin
                            NEXT_FETCH_STATE = FETCH_DISPL;
                        end else begin
                            NEXT_FETCH_STATE = FETCH_EXWORD_1;
                        end
                    end
                endcase
            end
            PMOVE: begin
                if (BIW_1[9]) begin
                    // PMOVE MR,<ea>: no operand read, destination is memory.
                    case (BIW_0[5:3])
                        3'b010, 3'b011, 3'b100: begin
                            if (AR_IN_USE) begin
                                NEXT_FETCH_STATE = START_OP; // Wait, ADH.
                            end else begin
                                NEXT_FETCH_STATE = INIT_EXEC_WB;
                            end
                        end
                        3'b101: begin
                            NEXT_FETCH_STATE = FETCH_DISPL;
                        end
                        3'b110: begin
                            NEXT_FETCH_STATE = FETCH_EXWORD_1;
                        end
                        default: begin // 3'b111
                            if (BIW_0[2:0] == 3'b000) begin
                                NEXT_FETCH_STATE = FETCH_ABS_LO;
                            end else begin
                                NEXT_FETCH_STATE = FETCH_ABS_HI;
                            end
                        end
                    endcase
                end else begin
                    // PMOVE <ea>,MR: read source from memory.
                    case (BIW_0[5:3])
                        3'b010, 3'b011: begin
                            if (AR_IN_USE) begin
                                NEXT_FETCH_STATE = START_OP; // Wait, ADH.
                            end else begin
                                NEXT_FETCH_STATE = FETCH_OPERAND;
                            end
                        end
                        3'b100: begin
                            if (AR_IN_USE) begin
                                NEXT_FETCH_STATE = START_OP; // Wait, ADH.
                            end else begin
                                NEXT_FETCH_STATE = CALC_AEFF;
                            end
                        end
                        3'b101: begin
                            NEXT_FETCH_STATE = FETCH_DISPL;
                        end
                        3'b110: begin
                            NEXT_FETCH_STATE = FETCH_EXWORD_1;
                        end
                        default: begin // 3'b111
                            if (BIW_0[2:0] == 3'b000) begin
                                NEXT_FETCH_STATE = FETCH_ABS_LO;
                            end else begin
                                NEXT_FETCH_STATE = FETCH_ABS_HI;
                            end
                        end
                    endcase
                end
            end
            LINK, UNLK: begin
                // We have to wait for the ALU because the registers are written without pipelining
                // through the ALU and the stack is decremented early.
                if (ALU_BSY) begin
                    NEXT_FETCH_STATE = START_OP; // Wait, ADH, ASH (two address registers are affected).
                end else begin
                    NEXT_FETCH_STATE = SWITCH_STATE; // Stack pointer is decremented in this state.
                end
            end
            MOVE: begin
                case (BIW_0[5:3]) // Source operand.
                    3'b000: begin // Dn.
                        // Destination is -(An) and will be decremented here, wait.
                        if (BIW_0[8:6] == 3'b100 && (AR_IN_USE || DR_IN_USE)) begin
                            NEXT_FETCH_STATE = START_OP;
                        end else if (DR_IN_USE) begin
                            NEXT_FETCH_STATE = START_OP;
                        end else begin
                            NEXT_FETCH_STATE = INIT_EXEC_WB;
                        end
                    end
                    3'b001: begin // An.
                        if (AR_IN_USE) begin
                            NEXT_FETCH_STATE = START_OP;
                        end else begin
                            NEXT_FETCH_STATE = INIT_EXEC_WB;
                        end
                    end
                    3'b010, 3'b011: begin // (An), (An)+.
                        if (!AR_IN_USE) begin
                            NEXT_FETCH_STATE = FETCH_OPERAND;
                        end else begin
                            NEXT_FETCH_STATE = START_OP;
                        end
                    end
                    3'b100: begin // -(An).
                        if (!AR_IN_USE) begin
                            NEXT_FETCH_STATE = CALC_AEFF;
                        end else begin
                            NEXT_FETCH_STATE = START_OP;
                        end
                    end
                    3'b101: begin
                        NEXT_FETCH_STATE = FETCH_DISPL;
                    end
                    3'b110: begin
                        NEXT_FETCH_STATE = FETCH_EXWORD_1;
                    end
                    default: begin
                        if (BIW_0[2:0] == 3'b000) begin
                            NEXT_FETCH_STATE = FETCH_ABS_LO;
                        end else if (BIW_0[2:0] == 3'b001) begin
                            NEXT_FETCH_STATE = FETCH_ABS_HI;
                        end else if (BIW_0[2:0] == 3'b100 && BIW_0[13:12] == 2'b10) begin // Long.
                            NEXT_FETCH_STATE = FETCH_IDATA_B2;
                        end else if (BIW_0[2:0] == 3'b100) begin // Word or Byte.
                            NEXT_FETCH_STATE = FETCH_IDATA_B1;
                        end else if (BIW_0[2:0] == 3'b010) begin
                            NEXT_FETCH_STATE = FETCH_DISPL;
                        end else begin
                            NEXT_FETCH_STATE = FETCH_EXWORD_1;
                        end
                    end
                endcase
            end
            MOVEM: begin
                case (BIW_0[5:3])
                    3'b010, 3'b011, 3'b100: begin // (An), (An)+, -(An).
                        if (AR_IN_USE) begin // ADH.
                            NEXT_FETCH_STATE = START_OP;
                        end else begin
                            NEXT_FETCH_STATE = INIT_EXEC_WB;
                        end
                    end
                    3'b101: begin
                        NEXT_FETCH_STATE = FETCH_DISPL;
                    end
                    3'b110: begin
                        NEXT_FETCH_STATE = FETCH_EXWORD_1;
                    end
                    default: begin
                        if (BIW_0[2:0] == 3'b000) begin
                            NEXT_FETCH_STATE = FETCH_ABS_LO;
                        end else if (BIW_0[2:0] == 3'b001) begin
                            NEXT_FETCH_STATE = FETCH_ABS_HI;
                        end else if (BIW_0[2:0] == 3'b010) begin
                            NEXT_FETCH_STATE = FETCH_DISPL;
                        end else begin
                            NEXT_FETCH_STATE = FETCH_EXWORD_1;
                        end
                    end
                endcase
            end
            MOVEP: begin
                if (!AR_IN_USE && BIW_0[7:6] < 2'b10) begin
                    NEXT_FETCH_STATE = SWITCH_STATE; // Memory to register.
                end else if (!AR_IN_USE && !DR_IN_USE) begin // ADH.
                    NEXT_FETCH_STATE = SWITCH_STATE; // Register to memory.
                end else begin
                    NEXT_FETCH_STATE = START_OP;
                end
            end
            BSR, MOVE_USP: begin
                // MOVE_USP: wait until A7 has been updated to load the correct data to the ALU.
                // BSR: wait until A7 has been updated before decrementing.
                if (!AR_IN_USE) begin // ADH.
                    NEXT_FETCH_STATE = INIT_EXEC_WB;
                end else begin
                    NEXT_FETCH_STATE = START_OP;
                end
            end
            MOVEC: begin
                if (BIW_0[0] && BIW_1[15] && AR_IN_USE) begin // Address register is source.
                    NEXT_FETCH_STATE = START_OP; // Wait, ADH.
                end else if (BIW_0[0] && !BIW_1[15] && DR_IN_USE) begin // Data register is source.
                    NEXT_FETCH_STATE = START_OP; // Wait, ADH.
                end else begin
                    NEXT_FETCH_STATE = INIT_EXEC_WB;
                end
            end
            MOVES: begin
                case (BIW_0[5:3])
                    3'b010, 3'b011: begin // (An), (An)+.
                        if (!BIW_1[11] && !AR_IN_USE) begin // ADH.
                            NEXT_FETCH_STATE = FETCH_OPERAND; // Memory to register.
                        end else if (BIW_1[11] && !AR_IN_USE) begin // ADH.
                            NEXT_FETCH_STATE = INIT_EXEC_WB; // Register to memory.
                        end else begin
                            NEXT_FETCH_STATE = START_OP; // Wait, ADH.
                        end
                    end
                    3'b100: begin // -(An).
                        if (!BIW_1[11] && !AR_IN_USE) begin // ADH.
                            NEXT_FETCH_STATE = CALC_AEFF; // Memory to register.
                        end else if (BIW_1[11] && !AR_IN_USE) begin // ADH.
                            NEXT_FETCH_STATE = INIT_EXEC_WB; // Register to memory.
                        end else begin
                            NEXT_FETCH_STATE = START_OP; // Wait, ADH.
                        end
                    end
                    3'b101: begin
                        NEXT_FETCH_STATE = FETCH_DISPL;
                    end
                    3'b110: begin
                        NEXT_FETCH_STATE = FETCH_EXWORD_1;
                    end
                    default: begin // 3'b111
                        if (BIW_0[2:0] == 3'b000) begin
                            NEXT_FETCH_STATE = FETCH_ABS_LO;
                        end else begin
                            NEXT_FETCH_STATE = FETCH_ABS_HI;
                        end
                    end
                endcase
                // Register to memory:
                if (BIW_1[11] && BIW_1[15] && AR_IN_USE) begin // ADH.
                    NEXT_FETCH_STATE = START_OP;
                end else if (BIW_1[11] && !BIW_1[15] && DR_IN_USE) begin // ADH.
                    NEXT_FETCH_STATE = START_OP;
                end
            end
            ANDI_TO_CCR, ANDI_TO_SR, EORI_TO_CCR, EORI_TO_SR, ORI_TO_CCR, ORI_TO_SR, OP_RESET: begin
                // Wait until the status register / condition codes have been updated. Otherwise we
                // possibly have a data hazard using the wrong condition codes for the operation.
                if (ALU_BSY) begin
                    NEXT_FETCH_STATE = START_OP;
                end else begin
                    NEXT_FETCH_STATE = INIT_EXEC_WB;
                end
            end
            default: begin // Bcc, BRA, NOP, STOP, TRAPV.
                NEXT_FETCH_STATE = INIT_EXEC_WB;
            end
        endcase
    end
end

endmodule
