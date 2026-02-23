//--------------------------------------------------------------------//
//                                                                    //
// WF68K30L IP Core: Execute/writeback state machine next-state       //
// decoder.                                                           //
//                                                                    //
// This module contains the combinational next-state logic for the    //
// execute and writeback pipeline state machine. It was extracted      //
// from wf68k30L_control.sv to improve maintainability.               //
//                                                                    //
//--------------------------------------------------------------------//

module WF68K30L_CTRL_EXEC_DEC (
    // Current states
    input  logic [2:0]  EXEC_WB_STATE,
    input  logic [4:0]  FETCH_STATE,

    // Operation (writeback)
    input  logic [6:0]  OP_WB_I,
    input  logic [11:0] BIW_0_WB,
    input  logic [15:0] BIW_1_WB,

    // ALU
    input  logic        ALU_INIT_I,
    input  logic        ALU_REQ,
    input  logic        ALU_COND,

    // Data
    input  logic        WR_RDY,

    // MOVEM
    input  logic        MOVEM_INH_WR,

    // Bitfield
    input  int          BF_BYTES,

    // Phase
    input  logic        PHASE2,

    // Output
    output logic [2:0]  NEXT_EXEC_WB_STATE
);

`include "wf68k30L_pkg.svh"

// Local state type parameters matching the parent module
localparam logic [4:0]
    FETCH_OPERAND  = 5'd13;

localparam logic [2:0]
    IDLE           = 3'd0,
    EXECUTE        = 3'd1,
    ADR_PIPELINE   = 3'd2,
    WRITEBACK      = 3'd3,
    WRITE_DEST     = 3'd4;

always_comb begin : exec_wb_dec
    case (EXEC_WB_STATE)
        IDLE: begin
            if (ALU_INIT_I) begin
                NEXT_EXEC_WB_STATE = EXECUTE;
            end else begin
                NEXT_EXEC_WB_STATE = IDLE;
            end
        end
        EXECUTE: begin
            if (ALU_REQ) begin
                case (OP_WB_I)
                    ABCD, SBCD, ADDX, SUBX, PACK, UNPK: begin
                        if (!BIW_0_WB[3]) begin // Register to register.
                            NEXT_EXEC_WB_STATE = WRITEBACK;
                        end else begin // Memory to memory.
                            NEXT_EXEC_WB_STATE = WRITE_DEST;
                        end
                    end
                    ADD, SUB, AND_B, OR_B: begin
                        if (!BIW_0_WB[8]) begin
                            NEXT_EXEC_WB_STATE = WRITEBACK; // Destination is register.
                        end else begin
                            NEXT_EXEC_WB_STATE = WRITE_DEST; // Destination is in memory.
                        end
                    end
                    ADDA, SUBA, ANDI_TO_SR, BFEXTS, BFEXTU, BFFFO, DIVS, DIVU, EORI_TO_SR, EXG, EXT, EXTB,
                    LEA, MOVE_TO_CCR, MOVE_TO_SR, MOVE_USP, MOVEA, MOVEC, MOVEQ, MULS, MULU, ORI_TO_SR, STOP, SWAP, UNLK: begin
                        NEXT_EXEC_WB_STATE = WRITEBACK;
                    end
                    ADDI, ADDQ, ANDI, BCHG, BCLR, BFCHG, BFCLR, BFINS, BFSET, BSET, CLR, EOR, EORI,
                    MOVE_FROM_CCR, MOVE_FROM_SR, NBCD, NEG, NEGX, NOT_B, ORI, Scc, SUBI, SUBQ, TAS: begin
                        if (BIW_0_WB[5:3] == 3'b000) begin
                            NEXT_EXEC_WB_STATE = WRITEBACK; // Destination is a data register.
                        end else if (BIW_0_WB[5:3] == 3'b001) begin // Valid for ADDQ and SUBQ.
                            NEXT_EXEC_WB_STATE = WRITEBACK; // Destination is an address register.
                        end else begin
                            NEXT_EXEC_WB_STATE = WRITE_DEST; // Destination is in memory.
                        end
                    end
                    ASL, ASR, LSL, LSR, ROTL, ROTR, ROXL, ROXR: begin
                        if (BIW_0_WB[7:6] != 2'b11) begin
                            NEXT_EXEC_WB_STATE = WRITEBACK; // Register shifts.
                        end else begin
                            NEXT_EXEC_WB_STATE = WRITE_DEST; // Memory shifts.
                        end
                    end
                    CAS: begin
                        if (!ALU_COND) begin
                            NEXT_EXEC_WB_STATE = WRITEBACK; // Update Dc.
                        end else begin
                            NEXT_EXEC_WB_STATE = WRITE_DEST; // Update destination.
                        end
                    end
                    CAS2: begin
                        if (FETCH_STATE == FETCH_OPERAND) begin
                            NEXT_EXEC_WB_STATE = IDLE; // Second read access.
                        end else if (!ALU_COND) begin
                            NEXT_EXEC_WB_STATE = WRITEBACK; // Update Dc.
                        end else begin
                            NEXT_EXEC_WB_STATE = ADR_PIPELINE; // Update destination.
                        end
                    end
                    DBcc: begin
                        if (ALU_COND) begin
                            NEXT_EXEC_WB_STATE = IDLE;
                        end else begin
                            NEXT_EXEC_WB_STATE = WRITEBACK;
                        end
                    end
                    MOVE: begin
                        if (BIW_0_WB[8:6] == 3'b000) begin
                            NEXT_EXEC_WB_STATE = WRITEBACK; // Destination is register.
                        end else if (!PHASE2) begin
                            NEXT_EXEC_WB_STATE = WRITE_DEST; // Destination is in memory.
                        end else begin
                            NEXT_EXEC_WB_STATE = EXECUTE; // Wait for PHASE2 address calculation.
                        end
                    end
                    BSR, JSR, LINK, PEA: begin
                        NEXT_EXEC_WB_STATE = WRITE_DEST;
                    end
                    MOVEM: begin
                        if (OP_WB_I == MOVEM && MOVEM_INH_WR) begin
                            NEXT_EXEC_WB_STATE = IDLE; // Discard the write cycle.
                        end else if (BIW_0_WB[10]) begin
                            NEXT_EXEC_WB_STATE = WRITEBACK; // Destination is register.
                        end else begin
                            NEXT_EXEC_WB_STATE = WRITE_DEST; // Destination is in memory.
                        end
                    end
                    MOVEP: begin
                        if (BIW_0_WB[7:6] < 2'b10) begin
                            NEXT_EXEC_WB_STATE = WRITEBACK; // Memory to register.
                        end else begin
                            NEXT_EXEC_WB_STATE = WRITE_DEST; // Register to memory.
                        end
                    end
                    MOVES: begin
                        if (!BIW_1_WB[11]) begin
                            NEXT_EXEC_WB_STATE = WRITEBACK; // Destination is register.
                        end else begin
                            NEXT_EXEC_WB_STATE = WRITE_DEST; // Destination is in memory.
                        end
                    end
                    // Default is for:
                    // ANDI_TO_CCR, Bcc, BFTST, BTST, CHK, CHK2,
                    // CMP, CMP2, CMPA, CMPI, CMPM, EORI_TO_CCR,
                    // ORI_TO_CCR, RTR, TRAPV, TST.
                    default: begin
                        NEXT_EXEC_WB_STATE = IDLE;
                    end
                endcase
            end else begin
                NEXT_EXEC_WB_STATE = EXECUTE;
            end
        end
        ADR_PIPELINE: begin // Effective address calculation takes one clock cycle.
            NEXT_EXEC_WB_STATE = WRITE_DEST;
        end
        WRITEBACK: begin
            case (OP_WB_I)
                CAS2: begin
                    if (!PHASE2) begin
                        NEXT_EXEC_WB_STATE = WRITEBACK; // Update Dc2
                    end else begin
                        NEXT_EXEC_WB_STATE = IDLE;
                    end
                end
                default: begin
                    NEXT_EXEC_WB_STATE = IDLE;
                end
            endcase
        end
        WRITE_DEST: begin
            if (WR_RDY) begin
                case (OP_WB_I)
                    BFCHG, BFCLR, BFINS, BFSET: begin
                        if (BF_BYTES <= 4) begin
                            NEXT_EXEC_WB_STATE = IDLE;
                        end else begin
                            NEXT_EXEC_WB_STATE = ADR_PIPELINE;
                        end
                    end
                    CAS2: begin
                        if (!PHASE2) begin
                            NEXT_EXEC_WB_STATE = WRITE_DEST; // Update destination 2.
                        end else begin
                            NEXT_EXEC_WB_STATE = IDLE;
                        end
                    end
                    default: begin
                        NEXT_EXEC_WB_STATE = IDLE;
                    end
                endcase
            end else begin
                NEXT_EXEC_WB_STATE = WRITE_DEST;
            end
        end
        default: begin
            NEXT_EXEC_WB_STATE = IDLE;
        end
    endcase
end

endmodule
