//--------------------------------------------------------------------//
//                                                                    //
// WF68K30L IP Core: Fetch state machine decoder for non-START_OP     //
// states.                                                            //
//                                                                    //
// This module contains the combinational next-state logic for all    //
// fetch states except START_OP. Extracted from                       //
// wf68k30L_ctrl_fetch_dec.sv for maintainability.                    //
//                                                                    //
//--------------------------------------------------------------------//

module WF68K30L_CTRL_FETCH_OTHER (
    // Current states
    input  logic [4:0]  FETCH_STATE,
    input  logic [2:0]  EXEC_WB_STATE,
    input  logic [2:0]  NEXT_EXEC_WB_STATE,

    // Operation
    input  logic [6:0]  OP,
    input  logic [13:0] BIW_0,
    input  logic [15:0] BIW_1,
    input  logic [15:0] EXT_WORD,

    // Data availability
    input  logic        EW_ACK,
    input  logic        EW_RDY,
    input  logic        RD_RDY,
    input  logic        MEMADR_RDY,
    input  logic        WR_RDY,

    // Hazard signals
    input  logic        DR_IN_USE,
    input  logic        AR_IN_USE,

    // ALU
    input  logic        ALU_BSY,
    input  logic        ALU_COND,

    // Internal state
    input  logic [2:0]  ADR_MODE_I,
    input  logic        PHASE2,

    // MOVEM/MOVEP
    input  logic        MOVEM_COND,
    input  logic [3:0]  MOVEM_PNTR,
    input  logic        MOVEM_FIRST_RD,
    input  int          MOVEP_PNTR_I,

    // Bitfield
    input  int          BF_BYTES,

    // Control
    input  logic        BRANCH_ATN,
    input  logic        DBcc_COND,
    input  logic [1:0]  TRACE_MODE,
    input  logic        EXH_REQ,
    input  logic        LOOP_BSY,

    // Address format
    input  logic        OD_REQ_32,
    input  logic        OD_REQ_16,
    input  logic        MEM_INDIRECT,

    // Output
    output logic [4:0]  NEXT_FETCH_STATE
);

`include "wf68k30L_pkg.svh"

localparam logic [4:0]
    START_OP       = 5'd0,
    CALC_AEFF      = 5'd1,
    FETCH_DISPL    = 5'd2,
    FETCH_EXWORD_1 = 5'd3,
    FETCH_D_LO     = 5'd4,
    FETCH_D_HI     = 5'd5,
    FETCH_OD_HI    = 5'd6,
    FETCH_OD_LO    = 5'd7,
    FETCH_ABS_HI   = 5'd8,
    FETCH_ABS_LO   = 5'd9,
    FETCH_IDATA_B2 = 5'd10,
    FETCH_IDATA_B1 = 5'd11,
    FETCH_MEMADR   = 5'd12,
    FETCH_OPERAND  = 5'd13,
    INIT_EXEC_WB   = 5'd14,
    SLEEP          = 5'd15,
    SWITCH_STATE   = 5'd16;

localparam logic [2:0]
    IDLE           = 3'd0,
    EXECUTE        = 3'd1,
    WRITEBACK      = 3'd3,
    WRITE_DEST     = 3'd4;

always_comb begin : other_states_dec
    case (FETCH_STATE)
        FETCH_DISPL: begin
            case (OP)
                ADD, CMP, SUB, AND_B, EOR, OR_B: begin
                    if ((EW_ACK || EW_RDY) && !AR_IN_USE) begin // ADH.
                        NEXT_FETCH_STATE = CALC_AEFF;
                    end else begin
                        NEXT_FETCH_STATE = FETCH_DISPL;
                    end
                end
                ADDA, BCHG, BCLR, BFCHG, BFCLR, BFEXTS, BFEXTU, BFFFO, BFINS, BFSET, BFTST, BSET, BTST, CHK, CHK2, CMP2, CMPA, DIVS, DIVU, MULS, MULU, MOVE, MOVEA, MOVE_TO_CCR, MOVE_TO_SR, SUBA: begin
                    if ((EW_ACK || EW_RDY) && OP == MOVE && PHASE2 && !AR_IN_USE) begin // ADH.
                        NEXT_FETCH_STATE = INIT_EXEC_WB;
                    end else if ((EW_ACK || EW_RDY) && !AR_IN_USE) begin // ADH.
                        NEXT_FETCH_STATE = CALC_AEFF;
                    end else begin
                        NEXT_FETCH_STATE = FETCH_DISPL;
                    end
                end
                ADDI, ADDQ, ANDI, CAS, CMPI, EORI, NBCD, NEG, NEGX, NOT_B, ORI, SUBI, SUBQ, TST, TAS, ASL, ASR, LSL, LSR, ROTL, ROTR, ROXL, ROXR: begin
                    if ((EW_ACK || EW_RDY) && !AR_IN_USE) begin // ADH.
                        NEXT_FETCH_STATE = CALC_AEFF;
                    end else begin
                        NEXT_FETCH_STATE = FETCH_DISPL;
                    end
                end
                MOVES: begin
                    if ((EW_ACK || EW_RDY) && !AR_IN_USE && !BIW_1[11]) begin
                        NEXT_FETCH_STATE = CALC_AEFF;
                    end else if ((EW_ACK || EW_RDY) && !AR_IN_USE) begin
                        NEXT_FETCH_STATE = INIT_EXEC_WB;
                    end else begin
                        NEXT_FETCH_STATE = FETCH_DISPL;
                    end
                end
                LEA, PEA: begin
                    if ((EW_ACK || EW_RDY) && !AR_IN_USE) begin // ADH.
                        NEXT_FETCH_STATE = SWITCH_STATE;
                    end else begin
                        NEXT_FETCH_STATE = FETCH_DISPL;
                    end
                end
                default: begin // CLR, JMP, JSR, MOVE_FROM_CCR, MOVE_FROM_SR, MOVEM, Scc.
                    if ((EW_ACK || EW_RDY) && !AR_IN_USE) begin // ADH.
                        NEXT_FETCH_STATE = INIT_EXEC_WB;
                    end else begin
                        NEXT_FETCH_STATE = FETCH_DISPL;
                    end
                end
            endcase
        end
        FETCH_EXWORD_1: begin
            // Be aware that the An registers which will be addressed by EXWORD_1 and are used for several addressing modes
            // are valid right after this state (because every address register manipulation requires no more than two clock cycles).
            if (EW_ACK && EXT_WORD[8] && EXT_WORD[5:4] == 2'b11) begin // 32 bit displacement.
                NEXT_FETCH_STATE = FETCH_D_HI;
            end else if (EW_ACK && EXT_WORD[8] && EXT_WORD[5:4] == 2'b10) begin // 16 bit displacement.
                NEXT_FETCH_STATE = FETCH_D_LO;
            end else if (EW_ACK && EXT_WORD[8] && EXT_WORD[5:4] == 2'b00) begin // Reserved.
                NEXT_FETCH_STATE = START_OP;
            end else if (EW_ACK && EXT_WORD[8] && EXT_WORD[1:0] == 2'b11) begin
                NEXT_FETCH_STATE = FETCH_OD_HI; // Long outer displacement.
            end else if (EW_ACK && EXT_WORD[8] && EXT_WORD[1:0] == 2'b10) begin
                NEXT_FETCH_STATE = FETCH_OD_LO; // Word outer displacement.
            end else if (EW_ACK && EXT_WORD[8] && EXT_WORD[1:0] == 2'b01) begin
                NEXT_FETCH_STATE = FETCH_MEMADR; // Null outer displacement, go to intermediate address.
            end else if (EW_ACK || EW_RDY) begin // Null displacement, no outer displacement.
                case (OP)
                    ADD, CMP, SUB, AND_B, EOR, OR_B: begin
                        if ((!BIW_1[15] && !AR_IN_USE && !DR_IN_USE) || (BIW_1[15] && !AR_IN_USE)) begin // ADH.
                            NEXT_FETCH_STATE = CALC_AEFF;
                        end else begin
                            NEXT_FETCH_STATE = FETCH_EXWORD_1;
                        end
                    end
                    ADDA, BCHG, BCLR, BFCHG, BFCLR, BFEXTS, BFEXTU, BFFFO, BFINS, BFSET, BFTST, BSET, BTST, CHK, CHK2, CMP2, CMPA, DIVS, DIVU, MULS, MULU, MOVE, MOVEA, MOVE_TO_CCR, MOVE_TO_SR, SUBA: begin
                        if (OP == MOVE && PHASE2 && !BIW_1[15] && !AR_IN_USE && !DR_IN_USE) begin // ADH.
                            NEXT_FETCH_STATE = INIT_EXEC_WB;
                        end else if (OP == MOVE && PHASE2 && BIW_1[15] && !AR_IN_USE) begin // ADH.
                            NEXT_FETCH_STATE = INIT_EXEC_WB;
                        end else if ((!BIW_1[15] && !AR_IN_USE && !DR_IN_USE) || (BIW_1[15] && !AR_IN_USE)) begin // ADH.
                            NEXT_FETCH_STATE = CALC_AEFF;
                        end else begin
                            NEXT_FETCH_STATE = FETCH_EXWORD_1;
                        end
                    end
                    MOVES: begin
                        if ((!BIW_1[15] && !AR_IN_USE && !DR_IN_USE) || (BIW_1[15] && !AR_IN_USE)) begin // ADH.
                            if (!BIW_1[11]) begin
                                NEXT_FETCH_STATE = CALC_AEFF;
                            end else begin
                                NEXT_FETCH_STATE = INIT_EXEC_WB;
                            end
                        end else begin
                            NEXT_FETCH_STATE = FETCH_EXWORD_1;
                        end
                    end
                    ADDI, ADDQ, ANDI, CAS, CMPI, EORI, NBCD, NEG, NEGX, NOT_B, ORI, SUBI, SUBQ, TST, TAS, ASL, ASR, LSL, LSR, ROTL, ROTR, ROXL, ROXR: begin
                        if ((!BIW_1[15] && !AR_IN_USE && !DR_IN_USE) || (BIW_1[15] && !AR_IN_USE)) begin // ADH.
                            NEXT_FETCH_STATE = CALC_AEFF;
                        end else begin
                            NEXT_FETCH_STATE = FETCH_EXWORD_1;
                        end
                    end
                    LEA, PEA: begin
                        if ((!BIW_1[15] && !AR_IN_USE && !DR_IN_USE) || (BIW_1[15] && !AR_IN_USE)) begin // ADH.
                            NEXT_FETCH_STATE = SWITCH_STATE;
                        end else begin
                            NEXT_FETCH_STATE = FETCH_EXWORD_1;
                        end
                    end
                    default: begin // CLR, JMP, JSR, MOVE_FROM_CCR, MOVE_FROM_SR, MOVEM, Scc.
                        if ((!BIW_1[15] && !AR_IN_USE && !DR_IN_USE) || (BIW_1[15] && !AR_IN_USE)) begin // ADH.
                            NEXT_FETCH_STATE = INIT_EXEC_WB;
                        end else begin
                            NEXT_FETCH_STATE = FETCH_EXWORD_1;
                        end
                    end
                endcase
            end else begin
                NEXT_FETCH_STATE = FETCH_EXWORD_1;
            end
        end
        FETCH_D_HI: begin
            if (EW_ACK) begin
                NEXT_FETCH_STATE = FETCH_D_LO;
            end else begin
                NEXT_FETCH_STATE = FETCH_D_HI;
            end
        end
        FETCH_D_LO: begin
            if (EW_ACK && OD_REQ_32) begin
                NEXT_FETCH_STATE = FETCH_OD_HI;
            end else if (EW_ACK && OD_REQ_16) begin
                NEXT_FETCH_STATE = FETCH_OD_LO;
            end else if (EW_ACK && MEM_INDIRECT) begin // Null displacement.
                NEXT_FETCH_STATE = FETCH_MEMADR;
            end else if (EW_ACK || EW_RDY) begin
                case (OP)
                    ADD, CMP, SUB, AND_B, EOR, OR_B: begin
                        if (!AR_IN_USE) begin // ADH.
                            NEXT_FETCH_STATE = CALC_AEFF;
                        end else begin
                            NEXT_FETCH_STATE = FETCH_D_LO;
                        end
                    end
                    ADDA, BCHG, BCLR, BFCHG, BFCLR, BFEXTS, BFEXTU, BFFFO, BFINS, BFSET, BFTST, BSET, BTST, CHK, CHK2, CMP2, CMPA, DIVS, DIVU, MULS, MULU, MOVE, MOVEA, MOVE_TO_CCR, MOVE_TO_SR, SUBA: begin
                        if (OP == MOVE && PHASE2 && !AR_IN_USE) begin // ADH.
                            NEXT_FETCH_STATE = INIT_EXEC_WB;
                        end else if (!AR_IN_USE) begin // ADH.
                            NEXT_FETCH_STATE = CALC_AEFF;
                        end else begin
                            NEXT_FETCH_STATE = FETCH_D_LO;
                        end
                    end
                    MOVES: begin
                        if (AR_IN_USE) begin // ADH.
                            NEXT_FETCH_STATE = FETCH_D_LO; // Wait, ADH.
                        end else if (!BIW_1[11]) begin
                            NEXT_FETCH_STATE = CALC_AEFF;
                        end else begin
                            NEXT_FETCH_STATE = INIT_EXEC_WB;
                        end
                    end
                    ADDI, ADDQ, ANDI, CAS, CMPI, EORI, NBCD, NEG, NEGX, NOT_B, ORI, SUBI, SUBQ, TST, TAS, ASL, ASR, LSL, LSR, ROTL, ROTR, ROXL, ROXR: begin
                        if (!AR_IN_USE) begin // ADH.
                            NEXT_FETCH_STATE = CALC_AEFF;
                        end else begin
                            NEXT_FETCH_STATE = FETCH_D_LO;
                        end
                    end
                    LEA, PEA: begin
                        if (!AR_IN_USE) begin // ADH.
                            NEXT_FETCH_STATE = SWITCH_STATE;
                        end else begin
                            NEXT_FETCH_STATE = FETCH_D_LO;
                        end
                    end
                    default: begin // CLR, JMP, JSR, MOVE_FROM_CCR, MOVE_FROM_SR, MOVEM, Scc.
                        if (!AR_IN_USE) begin // ADH.
                            NEXT_FETCH_STATE = INIT_EXEC_WB;
                        end else begin
                            NEXT_FETCH_STATE = FETCH_D_LO;
                        end
                    end
                endcase
            end else begin
                NEXT_FETCH_STATE = FETCH_D_LO;
            end
        end
        FETCH_OD_HI: begin
            if (EW_ACK) begin
                NEXT_FETCH_STATE = FETCH_OD_LO;
            end else begin
                NEXT_FETCH_STATE = FETCH_OD_HI;
            end
        end
        FETCH_OD_LO: begin
            if (EW_ACK) begin
                NEXT_FETCH_STATE = FETCH_MEMADR;
            end else begin
                NEXT_FETCH_STATE = FETCH_OD_LO;
            end
        end
        FETCH_MEMADR: begin
            if (RD_RDY || MEMADR_RDY) begin
                case (OP)
                    ADD, CMP, SUB, AND_B, EOR, OR_B: begin
                        if (!AR_IN_USE) begin // ADH.
                            NEXT_FETCH_STATE = CALC_AEFF;
                        end else begin
                            NEXT_FETCH_STATE = FETCH_MEMADR;
                        end
                    end
                    ADDA, BCHG, BCLR, BFCHG, BFCLR, BFEXTS, BFEXTU, BFFFO, BFINS, BFSET, BFTST, BSET, BTST, CHK, CHK2, CMP2, CMPA, DIVS, DIVU, MULS, MULU, MOVE, MOVEA, MOVE_TO_CCR, MOVE_TO_SR, SUBA: begin
                        if (OP == MOVE && PHASE2 && !AR_IN_USE) begin // ADH.
                            NEXT_FETCH_STATE = INIT_EXEC_WB;
                        end else if (!AR_IN_USE) begin // ADH.
                            NEXT_FETCH_STATE = CALC_AEFF;
                        end else begin
                            NEXT_FETCH_STATE = FETCH_MEMADR;
                        end
                    end
                    MOVES: begin
                        if (AR_IN_USE) begin // ADH.
                            NEXT_FETCH_STATE = FETCH_MEMADR; // Wait, ADH.
                        end else if (!BIW_1[11]) begin
                            NEXT_FETCH_STATE = CALC_AEFF;
                        end else begin
                            NEXT_FETCH_STATE = INIT_EXEC_WB;
                        end
                    end
                    ADDI, ADDQ, ANDI, CAS, CMPI, EORI, NBCD, NEG, NEGX, NOT_B, ORI, SUBI, SUBQ, TST, TAS, ASL, ASR, LSL, LSR, ROTL, ROTR, ROXL, ROXR: begin
                        if (!AR_IN_USE) begin // ADH.
                            NEXT_FETCH_STATE = CALC_AEFF;
                        end else begin
                            NEXT_FETCH_STATE = FETCH_MEMADR;
                        end
                    end
                    LEA, PEA: begin
                        if (!AR_IN_USE) begin // ADH.
                            NEXT_FETCH_STATE = SWITCH_STATE;
                        end else begin
                            NEXT_FETCH_STATE = FETCH_MEMADR;
                        end
                    end
                    default: begin // CLR, JMP, JSR, MOVE_FROM_CCR, MOVE_FROM_SR, MOVEM, Scc.
                        if (!AR_IN_USE) begin // ADH.
                            NEXT_FETCH_STATE = INIT_EXEC_WB;
                        end else begin
                            NEXT_FETCH_STATE = FETCH_MEMADR;
                        end
                    end
                endcase
            end else begin
                NEXT_FETCH_STATE = FETCH_MEMADR;
            end
        end
        FETCH_ABS_HI: begin
            if (EW_ACK) begin
                NEXT_FETCH_STATE = FETCH_ABS_LO;
            end else begin
                NEXT_FETCH_STATE = FETCH_ABS_HI;
            end
        end
        FETCH_ABS_LO: begin
            if (EW_ACK) begin
                case (OP)
                    CLR, JMP, JSR, MOVE_FROM_CCR, MOVE_FROM_SR, MOVEM, Scc: begin
                        NEXT_FETCH_STATE = INIT_EXEC_WB;
                    end
                    LEA, PEA: begin
                        NEXT_FETCH_STATE = SWITCH_STATE;
                    end
                    MOVE: begin
                        if (!PHASE2) begin
                            NEXT_FETCH_STATE = CALC_AEFF;
                        end else begin
                            NEXT_FETCH_STATE = INIT_EXEC_WB;
                        end
                    end
                    MOVES: begin
                        if (!BIW_1[11]) begin
                            NEXT_FETCH_STATE = CALC_AEFF;
                        end else begin
                            NEXT_FETCH_STATE = INIT_EXEC_WB;
                        end
                    end
                    default: begin
                        NEXT_FETCH_STATE = CALC_AEFF;
                    end
                endcase
            end else begin
                NEXT_FETCH_STATE = FETCH_ABS_LO;
            end
        end
        FETCH_IDATA_B2: begin
            if (EW_ACK) begin
                NEXT_FETCH_STATE = FETCH_IDATA_B1;
            end else begin
                NEXT_FETCH_STATE = FETCH_IDATA_B2;
            end
        end
        FETCH_IDATA_B1: begin
            if (EW_ACK || EW_RDY) begin
                case (OP) // ADH.
                    ADD, SUB, AND_B, OR_B, BTST, DIVS, DIVU, MULS, MULU, CHK, MOVE: begin
                        NEXT_FETCH_STATE = SWITCH_STATE;
                    end
                    ADDA, CMPA, SUBA, MOVEA: begin
                        NEXT_FETCH_STATE = SWITCH_STATE;
                    end
                    default: begin
                        if (DR_IN_USE) begin // ADH.
                            NEXT_FETCH_STATE = FETCH_IDATA_B1;
                        end else begin
                            NEXT_FETCH_STATE = INIT_EXEC_WB;
                        end
                    end
                endcase
            end else begin
                NEXT_FETCH_STATE = FETCH_IDATA_B1;
            end
        end
        CALC_AEFF: begin
            NEXT_FETCH_STATE = FETCH_OPERAND; // One CLK calculation delay.
        end
        FETCH_OPERAND: begin
            if (RD_RDY) begin
                case (OP)
                    ABCD, ADDX, SBCD, SUBX: begin
                        if (!PHASE2) begin
                            NEXT_FETCH_STATE = CALC_AEFF;
                        end else begin
                            NEXT_FETCH_STATE = INIT_EXEC_WB;
                        end
                    end
                    ADD, CMP, CHK, SUB, AND_B, EOR, OR_B, BCHG, BCLR, BSET, BTST, DIVS, DIVU, MULS, MULU: begin
                        if (DR_IN_USE) begin
                            NEXT_FETCH_STATE = SWITCH_STATE;
                        end else begin
                            NEXT_FETCH_STATE = INIT_EXEC_WB;
                        end
                    end
                    ADDA, CMPA, SUBA: begin
                        if (BIW_0[11:9] == BIW_0[2:0] && ADR_MODE_I == 3'b011) begin
                            NEXT_FETCH_STATE = SWITCH_STATE; // Postincrement (Ax)+, AX; wait before loading the ALU.
                        end else begin
                            NEXT_FETCH_STATE = INIT_EXEC_WB;
                        end
                    end
                    BFCHG, BFCLR, BFEXTS, BFEXTU, BFFFO, BFINS, BFSET, BFTST: begin
                        if (BF_BYTES == 5) begin // Another Byte required.
                            NEXT_FETCH_STATE = CALC_AEFF;
                        end else begin
                            NEXT_FETCH_STATE = INIT_EXEC_WB;
                        end
                    end
                    MOVE: begin
                        if (BIW_0[8:6] == 3'b100 && ADR_MODE_I == 3'b011) begin // (An)+,-(An).
                            NEXT_FETCH_STATE = SWITCH_STATE;
                        end else begin
                            NEXT_FETCH_STATE = INIT_EXEC_WB;
                        end
                    end
                    default: begin
                        NEXT_FETCH_STATE = INIT_EXEC_WB;
                    end
                endcase
            end else begin
                NEXT_FETCH_STATE = FETCH_OPERAND;
            end
        end
        SWITCH_STATE: begin // This state is used individually by several operations.
            case (OP)
                ADDA, CMPA, SUBA, MOVEA: begin // Address register operations.
                    if (!AR_IN_USE) begin
                        NEXT_FETCH_STATE = INIT_EXEC_WB;
                    end else begin
                        NEXT_FETCH_STATE = SWITCH_STATE;
                    end
                end
                LEA, LINK, MOVE: begin
                    // LEA: calculate effective address (1 clock cycle) load it in INIT_EXEC_WB.
                    // LINK: used to load the decremented stack pointer.
                    // MOVE: Used for (An)+,-(An). address mode.
                    NEXT_FETCH_STATE = INIT_EXEC_WB;
                end
                UNLK: begin // SP is updated here.
                    NEXT_FETCH_STATE = CALC_AEFF;
                end
                MOVEM: begin // MOVEM requires 1 CLK cycle for address calculation.
                    if (!MOVEM_COND) begin
                        NEXT_FETCH_STATE = INIT_EXEC_WB; // Cancel bus access.
                    end else begin
                        NEXT_FETCH_STATE = CALC_AEFF;
                    end
                end
                MOVEP: begin // Register select and displacement update.
                    if (!DR_IN_USE && BIW_0[7:6] < 2'b10) begin
                        NEXT_FETCH_STATE = CALC_AEFF;
                    end else if (!DR_IN_USE && !ALU_BSY) begin // ASH.
                        NEXT_FETCH_STATE = INIT_EXEC_WB; // Register to memory.
                    end else begin
                        NEXT_FETCH_STATE = SWITCH_STATE;
                    end
                end
                PEA: begin
                    // PEA requires two clock cycles here for effective adress calculation because it
                    // is loaded early. The first clock cycle the address becomes valid and after the
                    // second the address is loaded to the ALU for writing on the stack.
                    if (!PHASE2) begin
                        NEXT_FETCH_STATE = SWITCH_STATE;
                    end else begin
                        NEXT_FETCH_STATE = INIT_EXEC_WB;
                    end
                end
                default: begin // Data register operations.
                    if (!DR_IN_USE) begin
                        NEXT_FETCH_STATE = INIT_EXEC_WB;
                    end else begin
                        NEXT_FETCH_STATE = SWITCH_STATE;
                    end
                end
            endcase
        end
        INIT_EXEC_WB: begin
            case (OP)
                ANDI_TO_SR, EORI_TO_SR, MOVE_TO_SR, ORI_TO_SR: begin
                    if (!ALU_BSY && BRANCH_ATN) begin
                        NEXT_FETCH_STATE = SLEEP; // Wait for new processor context.
                    end else if (!ALU_BSY) begin
                        NEXT_FETCH_STATE = START_OP; // Proceed normally.
                    end else begin
                        NEXT_FETCH_STATE = INIT_EXEC_WB;
                    end
                end
                Bcc, CHK, DBcc, JMP, TRAPcc, TRAPV: begin
                    if (!ALU_BSY) begin
                        NEXT_FETCH_STATE = SLEEP; // Check conditions.
                    end else begin
                        NEXT_FETCH_STATE = INIT_EXEC_WB;
                    end
                end
                CAS, JSR, MOVEC, TAS: begin
                    // CAS, TAS provide a RMC operation so have to sleep a little bit ;-)
                    if (!ALU_BSY) begin
                        NEXT_FETCH_STATE = SLEEP;
                    end else begin
                        NEXT_FETCH_STATE = INIT_EXEC_WB;
                    end
                end
                BRA, BSR, RTD, RTS: begin
                    if (!ALU_BSY) begin
                        NEXT_FETCH_STATE = START_OP;
                    end else begin
                        NEXT_FETCH_STATE = INIT_EXEC_WB;
                    end
                end
                BFCHG, BFCLR, BFINS, BFSET: begin
                    if (!ALU_BSY && BF_BYTES < 5) begin
                        NEXT_FETCH_STATE = START_OP;
                    end else if (!ALU_BSY) begin
                        NEXT_FETCH_STATE = SLEEP;
                    end else begin
                        NEXT_FETCH_STATE = INIT_EXEC_WB; // Wait, ASH.
                    end
                end
                CAS2: begin // RMC operation.
                    if (!ALU_BSY && !PHASE2) begin
                        NEXT_FETCH_STATE = CALC_AEFF; // Second compare.
                    end else if (!ALU_BSY) begin
                        NEXT_FETCH_STATE = SLEEP;
                    end else begin
                        NEXT_FETCH_STATE = INIT_EXEC_WB;
                    end
                end
                CHK2: begin
                    if (!ALU_BSY && !PHASE2) begin
                        NEXT_FETCH_STATE = FETCH_OPERAND; // Second compare required?
                    end else if (!ALU_BSY) begin // ASH.
                        NEXT_FETCH_STATE = SLEEP;
                    end else begin
                        NEXT_FETCH_STATE = INIT_EXEC_WB;
                    end
                end
                CMP2, CMPM: begin
                    if (!ALU_BSY && !PHASE2) begin
                        NEXT_FETCH_STATE = FETCH_OPERAND; // Second compare required?
                    end else if (!ALU_BSY) begin // ASH.
                        NEXT_FETCH_STATE = START_OP;
                    end else begin
                        NEXT_FETCH_STATE = INIT_EXEC_WB;
                    end
                end
                MOVE_USP: begin
                    if (!ALU_BSY && !BIW_0[3]) begin // An to USP.
                        NEXT_FETCH_STATE = SLEEP;
                    end else if (!ALU_BSY) begin
                        NEXT_FETCH_STATE = START_OP; // USP to An.
                    end else begin
                        NEXT_FETCH_STATE = INIT_EXEC_WB;
                    end
                end
                MOVE: begin
                    if (!ALU_BSY && !PHASE2) begin // Load the Operand into the ALU here.
                        case (BIW_0[8:6]) // Destination operand.
                            3'b101: begin
                                NEXT_FETCH_STATE = FETCH_DISPL;
                            end
                            3'b110: begin
                                NEXT_FETCH_STATE = FETCH_EXWORD_1;
                            end
                            3'b111: begin
                                if (BIW_0[11:9] == 3'b000) begin
                                    NEXT_FETCH_STATE = FETCH_ABS_LO;
                                end else begin
                                    NEXT_FETCH_STATE = FETCH_ABS_HI;
                                end
                            end
                            default: begin // No destination address calculation required.
                                NEXT_FETCH_STATE = START_OP;
                            end
                        endcase
                    end else if (PHASE2) begin // ALU is not required at this point.
                        NEXT_FETCH_STATE = START_OP;
                    end else begin
                        NEXT_FETCH_STATE = INIT_EXEC_WB;
                    end
                end
                MOVEM: begin
                    if (!ALU_BSY && !BIW_0[10] && ADR_MODE_I == 3'b100 && MOVEM_PNTR == 4'h0) begin // -(An), register to memory.
                        NEXT_FETCH_STATE = SLEEP; // Data completely transfered to the ALU.
                    end else if (!ALU_BSY && !BIW_0[10] && ADR_MODE_I != 3'b100 && MOVEM_PNTR == 4'hF) begin // Register to memory
                        NEXT_FETCH_STATE = SLEEP; // Data completely transfered to the ALU.
                    end else if (!ALU_BSY && BIW_0[10] && MOVEM_PNTR == 4'hF) begin // Memory to register.
                        NEXT_FETCH_STATE = SLEEP; // Data completely transfered to the ALU.
                    end else if (!ALU_BSY && BIW_0[10]) begin // Memory to register.
                        NEXT_FETCH_STATE = SWITCH_STATE;
                    end else begin
                        NEXT_FETCH_STATE = INIT_EXEC_WB; // Register to memory.
                    end
                end
                MOVEP: begin
                    if (!ALU_BSY && MOVEP_PNTR_I == 0) begin
                        NEXT_FETCH_STATE = START_OP; // Ready.
                    end else if (!ALU_BSY && BIW_0[7:6] < 2'b10) begin
                        NEXT_FETCH_STATE = CALC_AEFF; // Memory to register.
                    end else begin
                        NEXT_FETCH_STATE = INIT_EXEC_WB; // Register to memory.
                    end
                end
                NOP: begin
                    if (ALU_BSY) begin // ASH.
                        NEXT_FETCH_STATE = INIT_EXEC_WB; // Wait for all pending bus cycles to be completed.
                    end else begin
                        NEXT_FETCH_STATE = START_OP;
                    end
                end
                RTR: begin
                    if (!ALU_BSY && !PHASE2) begin
                        NEXT_FETCH_STATE = CALC_AEFF;
                    end else if (!ALU_BSY && PHASE2) begin
                        NEXT_FETCH_STATE = START_OP;
                    end else begin
                        NEXT_FETCH_STATE = INIT_EXEC_WB;
                    end
                end
                STOP: begin
                    if (!ALU_BSY) begin // ASH.
                        NEXT_FETCH_STATE = SLEEP;
                    end else begin
                        NEXT_FETCH_STATE = INIT_EXEC_WB;
                    end
                end
                UNLK: begin
                    if (!ALU_BSY && !AR_IN_USE) begin // ADH, ASH.
                        NEXT_FETCH_STATE = START_OP;
                    end else begin
                        NEXT_FETCH_STATE = INIT_EXEC_WB;
                    end
                end
                default: begin
                    if (!ALU_BSY) begin // ASH.
                        NEXT_FETCH_STATE = START_OP;
                    end else begin
                        NEXT_FETCH_STATE = INIT_EXEC_WB; // ASH.
                    end
                end
            endcase
        end
        SLEEP: begin
            case (OP)
                ANDI_TO_SR, CAS, CAS2, EORI_TO_SR, MOVE_TO_SR, MOVEM, ORI_TO_SR, TAS: begin
                    if (NEXT_EXEC_WB_STATE == IDLE) begin
                        NEXT_FETCH_STATE = START_OP;
                    end else begin
                        NEXT_FETCH_STATE = SLEEP;
                    end
                end
                BFCHG, BFCLR, BFINS, BFSET: begin
                    if (EXEC_WB_STATE == WRITE_DEST && WR_RDY && BF_BYTES == 1) begin
                        // Wait until second writeback access has been processed.
                        NEXT_FETCH_STATE = START_OP;
                    end else begin
                        NEXT_FETCH_STATE = SLEEP;
                    end
                end
                JSR: begin
                    if (!PHASE2) begin
                        NEXT_FETCH_STATE = SLEEP; // Wait for address calculation.
                    end else begin
                        NEXT_FETCH_STATE = START_OP;
                    end
                end
                MOVE_USP, MOVEC: begin
                    // MOVE_USP: wait for writeback not to conflict with AR_DEC in START_OP.
                    if (NEXT_EXEC_WB_STATE == IDLE) begin
                        NEXT_FETCH_STATE = START_OP;
                    end else begin
                        NEXT_FETCH_STATE = SLEEP; // Wait for new processor context.
                    end
                end
                DBcc: begin
                    // DBcc: evaluate conditions.
                    if (NEXT_EXEC_WB_STATE == IDLE) begin
                        NEXT_FETCH_STATE = START_OP;
                    end else begin
                        NEXT_FETCH_STATE = SLEEP;
                    end
                end
                STOP: begin
                    if (TRACE_MODE != 2'b00) begin
                        NEXT_FETCH_STATE = START_OP; // Do not perform a stop while tracing.
                    end else if (EXH_REQ) begin
                        NEXT_FETCH_STATE = START_OP; // Wait on interrupt.
                    end else begin
                        NEXT_FETCH_STATE = SLEEP;
                    end
                end
                default: begin // Bcc, CHK, JMP, TRAPV.
                    NEXT_FETCH_STATE = START_OP;
                end
            endcase
        end
        default: begin
            NEXT_FETCH_STATE = START_OP;
        end
    endcase
end

endmodule
