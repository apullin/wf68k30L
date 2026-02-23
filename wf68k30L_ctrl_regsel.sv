//--------------------------------------------------------------------//
//                                                                    //
// WF68K30L IP Core: Register select combinational logic.             //
//                                                                    //
// This module contains the combinational logic that drives the       //
// address and data register select/write/increment/decrement         //
// signals. Extracted from wf68k30L_control.sv for maintainability.   //
//                                                                    //
//--------------------------------------------------------------------//

module WF68K30L_CTRL_REGSEL (
    // Operation
    input  logic [6:0]  OP,
    input  logic [6:0]  OP_WB_I,
    input  logic [13:0] BIW_0,
    input  logic [15:0] BIW_1,
    input  logic [15:0] BIW_2,
    input  logic [15:0] EXT_WORD,
    input  logic [11:0] BIW_0_WB,
    input  logic [15:0] BIW_1_WB,

    // State
    input  logic [4:0]  FETCH_STATE,
    input  logic [4:0]  NEXT_FETCH_STATE,
    input  logic [2:0]  EXEC_WB_STATE,
    input  logic        INIT_ENTRY,
    input  logic        PHASE2,

    // Internal signals
    input  logic [2:0]  ADR_MODE_I,
    input  logic [1:0]  OP_SIZE_I,
    input  logic        ALU_INIT_I,
    input  logic        ALU_BSY,
    input  logic        RD_RDY,
    input  logic        WR_RDY,
    input  logic        DATA_RDY,
    input  logic        DATA_VALID,

    // MOVEM
    input  logic [3:0]  MOVEM_PNTR,
    input  logic        MOVEM_ADn_I,
    input  logic        MOVEM_ADn_WB,
    input  logic        MOVEM_COND,
    input  logic        MOVEM_LAST_WR,

    // In-use flags
    input  logic        AR_IN_USE,
    input  logic        DR_IN_USE,

    // Address register outputs
    output logic [2:0]  AR_SEL_RD_1,
    output logic [2:0]  AR_SEL_RD_2,
    output logic [2:0]  AR_SEL_WR_1,
    output logic [2:0]  AR_SEL_WR_2,
    output logic        AR_INC,
    output logic        AR_DEC,
    output logic        AR_WR_1,
    output logic        AR_WR_2,

    // Data register outputs
    output logic [2:0]  DR_SEL_RD_1,
    output logic [2:0]  DR_SEL_RD_2,
    output logic [2:0]  DR_SEL_WR_1,
    output logic [2:0]  DR_SEL_WR_2,
    output logic        DR_WR_1,
    output logic        DR_WR_2,

    // Other
    output logic        USE_DREG
);

`include "wf68k30L_pkg.svh"

// Local state type parameters matching the parent module
localparam logic [4:0]
    START_OP       = 5'd0,
    CALC_AEFF      = 5'd1,
    FETCH_DISPL    = 5'd2,
    FETCH_EXWORD_1 = 5'd3,
    FETCH_ABS_LO   = 5'd9,
    FETCH_OPERAND  = 5'd13,
    INIT_EXEC_WB   = 5'd14,
    SWITCH_STATE   = 5'd16;

localparam logic [2:0]
    IDLE           = 3'd0,
    EXECUTE        = 3'd1,
    ADR_PIPELINE   = 3'd2,
    WRITEBACK      = 3'd3,
    WRITE_DEST     = 3'd4;

// Internal signals
logic AR_DEC_I;
logic AR_WR_I;
logic AR_WR_II;

// Used for the addressing modes and as source selector.
// In case of the addressing modes, the selector must be valid one clock cycle before the bus cycle
// starts due to the pipeline stage for ADR_REG in the address register section.
assign AR_SEL_RD_1 = ((OP == ABCD || OP == SBCD) && FETCH_STATE == START_OP) ? BIW_0[11:9] :
                     ((OP == ABCD || OP == SBCD) && FETCH_STATE == CALC_AEFF && !PHASE2) ? BIW_0[11:9] :
                     ((OP == ABCD || OP == SBCD) && FETCH_STATE == FETCH_OPERAND && !PHASE2 && !RD_RDY) ? BIW_0[11:9] : // Destination first.
                     ((OP == ABCD || OP == SBCD) && FETCH_STATE == INIT_EXEC_WB) ? BIW_0[11:9] :
                     ((OP == ADDX || OP == SUBX) && FETCH_STATE == START_OP) ? BIW_0[11:9] :
                     ((OP == ADDX || OP == SUBX) && FETCH_STATE == CALC_AEFF && !PHASE2) ? BIW_0[11:9] :
                     ((OP == ADDX || OP == SUBX) && FETCH_STATE == FETCH_OPERAND && !PHASE2 && !RD_RDY) ? BIW_0[11:9] : // Destination first.
                     ((OP == ADDX || OP == SUBX) && FETCH_STATE == INIT_EXEC_WB) ? BIW_0[11:9] :
                     (OP == CMPM && FETCH_STATE == FETCH_OPERAND && !PHASE2) ? BIW_0[11:9] : // Fetch destination.
                     (OP == MOVE && FETCH_STATE == 5'd12 && PHASE2) ? BIW_0[11:9] : // FETCH_MEMADR
                     (OP == MOVE && FETCH_STATE == START_OP && BIW_0[5:3] < 3'b010 && BIW_0[8:6] != 3'b000) ? BIW_0[11:9] : // Dn, An.
                     (OP == MOVE && FETCH_STATE == CALC_AEFF && PHASE2) ? BIW_0[11:9] :
                     (OP == MOVE && FETCH_STATE == FETCH_OPERAND && RD_RDY && BIW_0[8:6] == 3'b100 && BIW_0[5:3] != 3'b011) ? BIW_0[11:9] : // All except (An)+,-(An).
                     (OP == MOVE && FETCH_STATE == 5'd11 && BIW_0[8:6] != 3'b000) ? BIW_0[11:9] : // FETCH_IDATA_B1
                     (OP == MOVE && FETCH_STATE == FETCH_ABS_LO && BIW_0[8:6] != 3'b000 && !PHASE2) ? BIW_0[11:9] :
                     (OP == MOVE && FETCH_STATE == SWITCH_STATE && BIW_0[8:6] != 3'b000) ? BIW_0[11:9] :
                     (OP == MOVE && FETCH_STATE == INIT_EXEC_WB && BIW_0[8:6] != 3'b000) ? BIW_0[11:9] :
                     (OP == MOVEC && BIW_0[0]) ? BIW_1[14:12] : // MOVEC: general register to control register.
                     (OP == BSR || OP == MOVEC) ? 3'b111 : // Stack pointers.
                     ((OP == PACK || OP == UNPK) && FETCH_STATE == START_OP && !BIW_0[3] && !DR_IN_USE) ? BIW_0[11:9] : // Destination address.
                     ((OP == PACK || OP == UNPK) && FETCH_STATE == FETCH_OPERAND && RD_RDY) ? BIW_0[11:9] : // Destination address.
                     ((OP == PACK || OP == UNPK) && FETCH_STATE == INIT_EXEC_WB) ? BIW_0[11:9] : // Destination address.
                     (OP == CAS2 && FETCH_STATE == START_OP) ? BIW_1[14:12] : // Address operand.
                     (OP == CAS2 && FETCH_STATE == FETCH_OPERAND && !PHASE2) ? BIW_1[14:12] : // Address operand.
                     (OP == CAS2 && FETCH_STATE == CALC_AEFF) ? BIW_2[14:12] : // Address operand.
                     (OP == CAS2 && FETCH_STATE == FETCH_OPERAND) ? BIW_2[14:12] : // Address operand.
                     (OP_WB_I == CAS2 && (EXEC_WB_STATE == EXECUTE || EXEC_WB_STATE == ADR_PIPELINE)) ? BIW_1[14:12] : // Address operand.
                     (OP_WB_I == CAS2 && EXEC_WB_STATE == WRITE_DEST) ? BIW_2[14:12] : // Address operand.
                     ((OP == JSR || OP == LINK) && FETCH_STATE == START_OP) ? 3'b111 : // Select the SP to decrement.
                     (OP == PEA && FETCH_STATE == SWITCH_STATE && PHASE2) ? 3'b111 : // Select the SP to decrement.
                     ((OP == JSR || OP == LINK || OP == PEA) && FETCH_STATE == INIT_EXEC_WB) ? 3'b111 : // Writeback address is the SP.
                     (OP == RTD || OP == RTR || OP == RTS) ? 3'b111 : // Stack pointer.
                     (OP == UNLK && (FETCH_STATE == START_OP || FETCH_STATE == CALC_AEFF || FETCH_STATE == FETCH_OPERAND)) ? 3'b111 : // Check in START_OP to avoid data hazards!
                     (OP == ABCD || OP == ADD || OP == ADDA || OP == ADDI || OP == ADDQ || OP == ADDX || OP == AND_B || OP == ANDI) ? BIW_0[2:0] :
                     (OP == ASL || OP == ASR || OP == BCHG || OP == BCLR || OP == BSET || OP == BTST || OP == BFCHG || OP == BFCLR) ? BIW_0[2:0] :
                     (OP == BFEXTS || OP == BFEXTU || OP == BFFFO || OP == BFINS || OP == BFSET || OP == BFTST) ? BIW_0[2:0] :
                     (OP == CAS || OP == CHK || OP == CHK2 || OP == CLR || OP == CMP || OP == CMPA || OP == CMPI || OP == CMPM || OP == CMP2) ? BIW_0[2:0] :
                     (OP == DIVS || OP == DIVU || OP == EOR || OP == EORI || OP == EXG || OP == JMP || OP == JSR || OP == LEA || OP == LINK || OP == LSL || OP == LSR) ? BIW_0[2:0] :
                     (OP == MOVE || OP == MOVEA || OP == MOVE_FROM_CCR || OP == MOVE_FROM_SR || OP == MOVE_TO_CCR || OP == MOVE_TO_SR) ? BIW_0[2:0] :
                     (OP == MOVE_USP || OP == MOVEM || OP == MOVEP || OP == MOVES || OP == MULS || OP == MULU) ? BIW_0[2:0] :
                     (OP == NBCD || OP == NEG || OP == NEGX || OP == NOT_B || OP == OR_B || OP == ORI || OP == PACK || OP == PEA) ? BIW_0[2:0] :
                     (OP == ROTL || OP == ROTR || OP == ROXL || OP == ROXR || OP == SBCD || OP == Scc || OP == SUB || OP == SUBA) ? BIW_0[2:0] :
                     (OP == SUBI || OP == SUBQ || OP == SUBX || OP == TAS || OP == TST || OP == UNLK || OP == UNPK) ? BIW_0[2:0] : 3'b000;

// Always the destination.
assign AR_SEL_WR_1 = (OP == ADDQ || OP == SUBQ) ? BIW_0[2:0] :
                     (OP == EXG && BIW_0[7:3] == 5'b10001) ? BIW_0[2:0] : // Data and Address register.
                     (OP == MOVEC || OP == MOVES) ? BIW_1[14:12] :
                     (OP == UNLK && FETCH_STATE == START_OP) ? 3'b111 :
                     (OP == LINK) ? BIW_0[2:0] :
                     (OP == MOVEM) ? MOVEM_PNTR[2:0] :
                     (OP == MOVE_USP) ? BIW_0[2:0] :
                     BIW_0[11:9]; // ADDA, EXG, LEA, MOVE, MOVEA, SUBA.

assign AR_WR_1 = AR_WR_I;
assign AR_WR_I = (OP == LINK && FETCH_STATE == INIT_EXEC_WB && !ALU_BSY) ? 1'b1 :
                 (OP == UNLK && FETCH_STATE == SWITCH_STATE && NEXT_FETCH_STATE != SWITCH_STATE) ? 1'b1 : // Write An to SP.
                 (EXEC_WB_STATE != WRITEBACK) ? 1'b0 :
                 (OP_WB_I == ADDA || OP_WB_I == SUBA) ? 1'b1 :
                 ((OP_WB_I == ADDQ || OP_WB_I == SUBQ) && BIW_0_WB[5:3] == 3'b001) ? 1'b1 :
                 (OP_WB_I == EXG && BIW_0_WB[7:3] == 5'b01001) ? 1'b1 : // Two address registers.
                 (OP_WB_I == EXG && BIW_0_WB[7:3] == 5'b10001) ? 1'b1 : // Data and Address register.
                 (OP_WB_I == LEA) ? 1'b1 :
                 (OP_WB_I == MOVE_USP && BIW_0_WB[3]) ? 1'b1 :
                 (OP_WB_I == MOVEA) ? 1'b1 :
                 (OP_WB_I == MOVEC && BIW_1_WB[15] && !BIW_0_WB[0]) ? 1'b1 : // To general register.
                 (OP_WB_I == MOVES && BIW_1_WB[15]) ? 1'b1 :
                 (OP_WB_I == MOVEM && MOVEM_ADn_WB) ? 1'b1 : 1'b0;

assign AR_SEL_RD_2 = (OP == CHK2 || OP == CMP2) ? BIW_1[14:12] :
                     (OP == MOVEM) ? MOVEM_PNTR[2:0] : // This is the non addressing output.
                     (OP == MOVES) ? BIW_1[14:12] :
                     (OP == ADDQ || OP == MOVE || OP == SUBQ || OP == TST) ? BIW_0[2:0] :
                     (OP == EXG && BIW_0[7:3] == 5'b10001) ? BIW_0[2:0] : // Data and address register.
                     (OP == ADDA || OP == CMPA || OP == EXG || OP == SUBA) ? BIW_0[11:9] : 3'b000;

assign AR_SEL_WR_2 = BIW_0[2:0]; // Used for EXG, UNLK.

assign AR_WR_2 = AR_WR_II;
assign AR_WR_II = (OP_WB_I == UNLK && EXEC_WB_STATE == WRITEBACK) ? 1'b1 : // Write (SP) to An.
                  (OP_WB_I == EXG && EXEC_WB_STATE == WRITEBACK && BIW_0_WB[7:3] == 5'b01001) ? 1'b1 : 1'b0; // Two address registers.

assign AR_INC = ((OP == ADD || OP == CMP || OP == SUB) && ADR_MODE_I == 3'b011 && ALU_INIT_I) ? 1'b1 :
                ((OP == ADDA || OP == CMPA || OP == SUBA) && ADR_MODE_I == 3'b011 && FETCH_STATE == FETCH_OPERAND && DATA_RDY) ? 1'b1 :
                ((OP == ADDI || OP == CMPI || OP == SUBI) && ADR_MODE_I == 3'b011 && ALU_INIT_I) ? 1'b1 :
                ((OP == ADDQ || OP == SUBQ) && ADR_MODE_I == 3'b011 && ALU_INIT_I) ? 1'b1 :
                ((OP == AND_B || OP == EOR || OP == OR_B) && ADR_MODE_I == 3'b011 && ALU_INIT_I) ? 1'b1 :
                ((OP == ANDI || OP == EORI || OP == ORI) && ADR_MODE_I == 3'b011 && ALU_INIT_I) ? 1'b1 :
                ((OP == ASL || OP == ASR || OP == LSL || OP == LSR) && BIW_0[7:3] == 5'b11011 && ALU_INIT_I) ? 1'b1 :
                ((OP == ROTL || OP == ROTR || OP == ROXL || OP == ROXR) && BIW_0[7:3] == 5'b11011 && ALU_INIT_I) ? 1'b1 :
                ((OP == BCHG || OP == BCLR || OP == BSET || OP == BTST) && ADR_MODE_I == 3'b011 && ALU_INIT_I) ? 1'b1 :
                ((OP == CAS || OP == TAS) && ADR_MODE_I == 3'b011 && FETCH_STATE == INIT_EXEC_WB && NEXT_FETCH_STATE != INIT_EXEC_WB) ? 1'b1 :
                ((OP == CHK || OP == CLR || OP == TST || OP == Scc) && ADR_MODE_I == 3'b011 && ALU_INIT_I) ? 1'b1 :
                ((OP == NBCD || OP == NEG || OP == NEGX || OP == NOT_B) && ADR_MODE_I == 3'b011 && ALU_INIT_I) ? 1'b1 :
                (OP == CMPM && FETCH_STATE == FETCH_OPERAND && RD_RDY && DATA_VALID) ? 1'b1 :
                ((OP == MULS || OP == MULU || OP == DIVS || OP == DIVU) && ADR_MODE_I == 3'b011 && ALU_INIT_I) ? 1'b1 :
                ((OP == MOVE_FROM_CCR || OP == MOVE_FROM_SR) && ADR_MODE_I == 3'b011 && FETCH_STATE == INIT_EXEC_WB && NEXT_FETCH_STATE != INIT_EXEC_WB) ? 1'b1 :
                ((OP == MOVE_TO_CCR || OP == MOVE_TO_SR) && ADR_MODE_I == 3'b011 && FETCH_STATE == INIT_EXEC_WB && NEXT_FETCH_STATE != INIT_EXEC_WB) ? 1'b1 :
                (OP == MOVE && ADR_MODE_I == 3'b011 && FETCH_STATE == FETCH_OPERAND && NEXT_FETCH_STATE != FETCH_OPERAND) ? 1'b1 :
                (OP == MOVE && BIW_0[8:6] == 3'b011 && FETCH_STATE == INIT_EXEC_WB && NEXT_FETCH_STATE != INIT_EXEC_WB) ? 1'b1 :
                (OP == MOVEA && ADR_MODE_I == 3'b011 && FETCH_STATE == INIT_EXEC_WB && NEXT_FETCH_STATE != INIT_EXEC_WB) ? 1'b1 :
                (OP == MOVEM && ADR_MODE_I == 3'b011 && ALU_INIT_I) ? 1'b1 :
                (OP == MOVES && ADR_MODE_I == 3'b011 && ALU_INIT_I) ? 1'b1 :
                ((OP == UNLK || OP == RTD || OP == RTR || OP == RTS) && FETCH_STATE == FETCH_OPERAND && RD_RDY && DATA_VALID) ? 1'b1 : 1'b0;

always_comb begin
    case (OP)
        ABCD, ADD, ADDA, ADDI, ADDQ, ADDX, AND_B, ANDI, ASL, ASR, BCHG, BCLR, BSET, BTST, CHK, CMP, CMPA, CMPI,
        DIVS, DIVU, EOR, EORI, LSL, LSR, MOVE, MOVEA, MOVE_TO_CCR, MOVE_TO_SR, MOVES, MULS, MULU, NBCD, NEG, NEGX,
        NOT_B, OR_B, ORI, ROTL, ROTR, ROXL, ROXR, SBCD, SUB, SUBA, SUBI, SUBQ, SUBX, TAS, TST:
            AR_DEC_I = 1'b1;
        default:
            AR_DEC_I = 1'b0;
    endcase
end

assign AR_DEC = (ADR_MODE_I == 3'b100 && FETCH_STATE != CALC_AEFF && NEXT_FETCH_STATE == CALC_AEFF) ? AR_DEC_I :
                ((OP == BSR || OP == JSR || OP == LINK) && FETCH_STATE == START_OP && NEXT_FETCH_STATE != START_OP) ? 1'b1 :
                ((OP == CLR || OP == Scc) && ADR_MODE_I == 3'b100 && INIT_ENTRY) ? 1'b1 :
                (OP == MOVE && BIW_0[8:6] == 3'b100 && BIW_0[5:3] != 3'b011 && INIT_ENTRY) ? 1'b1 : // (An)+, -(An)
                (OP == MOVE && BIW_0[8:6] == 3'b100 && FETCH_STATE == SWITCH_STATE) ? 1'b1 : // Needed for source (An)+ mode.
                (OP == MOVEM && ADR_MODE_I == 3'b100 && INIT_ENTRY) ? 1'b1 : // Decrement before the first bus access.
                (OP == MOVEM && ADR_MODE_I == 3'b100 && ALU_INIT_I && !MOVEM_LAST_WR) ? 1'b1 : // After the last bus access the address register is not decremented.
                ((OP == MOVE_FROM_CCR || OP == MOVE_FROM_SR) && ADR_MODE_I == 3'b100 && INIT_ENTRY) ? 1'b1 :
                (OP == MOVES && ADR_MODE_I == 3'b100 && BIW_1[11] && INIT_ENTRY) ? 1'b1 :
                ((OP == PACK || OP == UNPK) && BIW_0[3] && INIT_ENTRY) ? 1'b1 :
                // PEA: decrement late in SWITCH_STATE not to decremented address register for PEA (xx,A7,yy) address modi.
                (OP == PEA && FETCH_STATE == SWITCH_STATE && PHASE2) ? 1'b1 : 1'b0;

assign DR_SEL_RD_1 = (FETCH_STATE == FETCH_EXWORD_1) ? EXT_WORD[14:12] : // Index register
                     ((OP == ADD || OP == SUB) && BIW_0[8]) ? BIW_0[11:9] :
                     ((OP == AND_B || OP == EOR || OP == OR_B) && BIW_0[8]) ? BIW_0[11:9] :
                     (OP == BCHG || OP == BCLR || OP == BSET || OP == BTST) ? BIW_0[11:9] :
                     (OP == ASL || OP == ASR || OP == LSL || OP == LSR) ? BIW_0[11:9] :
                     (OP == ROTL || OP == ROTR || OP == ROXL || OP == ROXR) ? BIW_0[11:9] :
                     ((OP == BFCHG || OP == BFCLR || OP == BFEXTS || OP == BFEXTU) && FETCH_STATE == START_OP && BIW_0[5:3] == 3'b000) ? BIW_0[2:0] :
                     ((OP == BFCHG || OP == BFCLR || OP == BFEXTS || OP == BFEXTU) && FETCH_STATE == START_OP && BIW_0[5:3] == 3'b001) ? BIW_0[2:0] :
                     ((OP == BFCHG || OP == BFCLR || OP == BFEXTS || OP == BFEXTU) && FETCH_STATE == FETCH_ABS_LO) ? BIW_0[2:0] :
                     ((OP == BFFFO || OP == BFINS || OP == BFSET || OP == BFTST) && FETCH_STATE == START_OP && BIW_0[5:3] == 3'b000) ? BIW_0[2:0] :
                     ((OP == BFFFO || OP == BFINS || OP == BFSET || OP == BFTST) && FETCH_STATE == START_OP && BIW_0[5:3] == 3'b001) ? BIW_0[2:0] :
                     ((OP == BFFFO || OP == BFINS || OP == BFSET || OP == BFTST) && FETCH_STATE == FETCH_ABS_LO) ? BIW_0[2:0] :
                     (OP == BFCHG || OP == BFCLR || OP == BFEXTS || OP == BFEXTU) ? BIW_1[8:6] : // Width value.
                     (OP == BFFFO || OP == BFINS || OP == BFSET || OP == BFTST) ? BIW_1[8:6] : // Width value.
                     (OP == CAS) ? BIW_1[2:0] : // Compare operand.
                     (OP == CAS2 && FETCH_STATE == START_OP) ? BIW_1[14:12] : // Address operand.
                     (OP == CAS2 && FETCH_STATE == FETCH_OPERAND && !PHASE2) ? BIW_1[14:12] : // Address operand.
                     (OP == CAS2 && FETCH_STATE == CALC_AEFF) ? BIW_2[14:12] : // Address operand.
                     (OP == CAS2 && FETCH_STATE == FETCH_OPERAND) ? BIW_2[14:12] : // Address operand.
                     (OP_WB_I == CAS2 && !BIW_1[15] && (EXEC_WB_STATE == EXECUTE || EXEC_WB_STATE == ADR_PIPELINE)) ? BIW_1[14:12] : // Address operand.
                     (OP_WB_I == CAS2 && !BIW_2[15] && EXEC_WB_STATE == WRITE_DEST) ? BIW_2[14:12] : // Address operand.
                     (OP == CAS2 && !PHASE2) ? BIW_1[2:0] : // Compare operand.
                     (OP == CAS2) ? BIW_2[2:0] : // Compare operand.
                     ((OP == DIVS || OP == DIVU) && FETCH_STATE != INIT_EXEC_WB && BIW_0[8:6] == 3'b001) ? BIW_1[2:0] : // LONG 64.
                     (OP == MOVEM) ? MOVEM_PNTR[2:0] :
                     (OP == MOVEP) ? BIW_0[11:9] :
                     (OP == MOVEC || OP == MOVES) ? BIW_1[14:12] :
                     (OP == ADD || OP == AND_B || OP == OR_B || OP == SUB) ? BIW_0[2:0] :
                     (OP == ABCD || OP == ADDA || OP == ADDX || OP == CHK || OP == CMP || OP == CMPA) ? BIW_0[2:0] :
                     (OP == EXG && BIW_0[7:3] == 5'b10001) ? BIW_0[11:9] : // Data and address register.
                     (OP == DIVS || OP == DIVU || OP == EXG) ? BIW_0[2:0] :
                     (OP == MOVE || OP == MOVEA || OP == MOVE_TO_CCR || OP == MOVE_TO_SR || OP == MULS || OP == MULU || OP == PACK) ? BIW_0[2:0] :
                     (OP == SBCD || OP == SUBA || OP == SUBX || OP == UNPK) ? BIW_0[2:0] : 3'b000;

assign DR_SEL_WR_1 = (OP == BFEXTS || OP == BFEXTU || OP == BFFFO) ? BIW_1[14:12] :
                     (OP == MOVEC || OP == MOVES) ? BIW_1[14:12] :
                     (OP == ABCD || OP == SBCD) ? BIW_0[11:9] :
                     (OP == ADDX || OP == SUBX) ? BIW_0[11:9] :
                     (OP == ADD || OP == SUB) ? BIW_0[11:9] :
                     (OP == AND_B || OP == OR_B) ? BIW_0[11:9] :
                     ((OP == DIVS || OP == DIVU) && OP_SIZE_I == WORD) ? BIW_0[11:9] :
                     (OP == DIVS || OP == DIVU) ? BIW_1[14:12] :
                     ((OP == MULS || OP == MULU) && OP_SIZE_I == WORD) ? BIW_0[11:9] :
                     (OP == MULS || OP == MULU) ? BIW_1[14:12] : // Low order result and operand.
                     (OP == EXG) ? BIW_0[11:9] :
                     (OP == MOVE || OP == MOVEP || OP == MOVEQ) ? BIW_0[11:9] :
                     (OP == PACK || OP == UNPK) ? BIW_0[11:9] :
                     (OP == CAS) ? BIW_1[2:0] : // Compare operand.
                     (OP_WB_I == CAS2 && EXEC_WB_STATE == EXECUTE) ? BIW_1[2:0] : // Compare operand 1.
                     (OP_WB_I == CAS2 && EXEC_WB_STATE == WRITEBACK) ? BIW_2[2:0] : // Compare operand 2.
                     (OP == MOVEM) ? MOVEM_PNTR[2:0] :
                     BIW_0[2:0];

assign DR_WR_1 = (OP_WB_I == EXG && EXEC_WB_STATE == WRITEBACK && BIW_0_WB[7:3] == 5'b10001) ? 1'b1 : // Address- and data register.
                 (AR_WR_I) ? 1'b0 : // This is the locking AR against DR.
                 (AR_WR_II) ? 1'b0 : // This is the locking AR against DR.
                 (OP_WB_I == ANDI_TO_SR || OP_WB_I == EORI_TO_SR || OP_WB_I == ORI_TO_SR) ? 1'b0 :
                 (OP_WB_I == MOVE_TO_CCR || OP_WB_I == MOVE_TO_SR) ? 1'b0 :
                 (OP_WB_I == MOVE_USP) ? 1'b0 : // USP is written.
                 (OP_WB_I == MOVEC && BIW_0_WB[0]) ? 1'b0 : // To control register.
                 (OP_WB_I == STOP) ? 1'b0 : // SR is written but not DR.
                 (EXEC_WB_STATE == WRITEBACK) ? 1'b1 : 1'b0;

assign DR_SEL_RD_2 = (OP == ABCD || OP == SBCD || OP == ADDX || OP == SUBX) ? BIW_0[11:9] :
                     ((OP == ADD || OP == AND_B || OP == OR_B || OP == SUB) && BIW_0[8]) ? BIW_0[2:0] :
                     (OP == ADD || OP == CMP || OP == SUB || OP == AND_B || OP == OR_B) ? BIW_0[11:9] :
                     (OP == CHK || OP == EXG) ? BIW_0[11:9] :
                     (OP == BFINS && FETCH_STATE == START_OP && BIW_0[5:3] == 3'b000) ? BIW_1[14:12] :
                     (OP == BFINS && FETCH_STATE == START_OP && BIW_0[5:3] == 3'b001) ? BIW_1[14:12] :
                     (OP == BFINS && FETCH_STATE == FETCH_ABS_LO) ? BIW_1[14:12] :
                     (OP == CAS) ? BIW_1[8:6] : // Update operand.
                     (OP == CAS2 && !PHASE2) ? BIW_1[8:6] : // Update operand.
                     (OP == CAS2) ? BIW_2[8:6] : // Update operand.
                     (OP == CHK2 || OP == CMP2) ? BIW_1[14:12] :
                     ((OP == DIVS || OP == DIVU) && BIW_0[7]) ? BIW_0[11:9] : // WORD size.
                     (OP == DIVS || OP == DIVU) ? BIW_1[14:12] : // Quotient low portion.
                     ((OP == MULS || OP == MULU) && BIW_0[7]) ? BIW_0[11:9] : // WORD size.
                     (OP == MULS || OP == MULU) ? BIW_1[14:12] :
                     (OP == BCHG || OP == BCLR || OP == BSET || OP == BTST) ? BIW_0[2:0] :
                     (OP == ADDI || OP == ADDQ || OP == ANDI || OP == BCHG || OP == BCLR || OP == BSET || OP == BTST || OP == CMPI) ? BIW_0[2:0] :
                     (OP == DBcc || OP == EOR || OP == EORI || OP == EXT || OP == EXTB || OP == NBCD || OP == NEG || OP == NEGX) ? BIW_0[2:0] :
                     (OP == NOT_B || OP == ORI || OP == SUBI || OP == SUBQ || OP == SWAP || OP == TAS || OP == TST) ? BIW_0[2:0] :
                     (OP == ASL || OP == ASR || OP == LSL || OP == LSR) ? BIW_0[2:0] :
                     (OP == ROTL || OP == ROTR || OP == ROXL || OP == ROXR) ? BIW_0[2:0] : 3'b000;

assign DR_SEL_WR_2 = (OP == EXG) ? BIW_0[2:0] : BIW_1[2:0]; // Default is for DIVS and DIVU, MULS, MULU.

// Normally source register. Written in a few exceptions.
assign DR_WR_2 = (OP_WB_I == EXG && EXEC_WB_STATE == WRITEBACK && BIW_0_WB[7:3] == 5'b01000) ? 1'b1 : // Two data registers.
                 (OP_WB_I == DIVS && EXEC_WB_STATE == WRITEBACK && BIW_0_WB[8:6] == 3'b001 && BIW_1_WB[14:12] != BIW_1_WB[2:0]) ? 1'b1 :
                 (OP_WB_I == DIVU && EXEC_WB_STATE == WRITEBACK && BIW_0_WB[8:6] == 3'b001 && BIW_1_WB[14:12] != BIW_1_WB[2:0]) ? 1'b1 :
                 (OP_WB_I == MULS && EXEC_WB_STATE == WRITEBACK && BIW_0_WB[8:6] == 3'b000 && BIW_1_WB[10] && BIW_1_WB[14:12] != BIW_1_WB[2:0]) ? 1'b1 :
                 (OP_WB_I == MULU && EXEC_WB_STATE == WRITEBACK && BIW_0_WB[8:6] == 3'b000 && BIW_1_WB[10] && BIW_1_WB[14:12] != BIW_1_WB[2:0]) ? 1'b1 : 1'b0;

assign USE_DREG = (OP == CAS2 && !BIW_1[15] && !PHASE2) ? 1'b1 :
                  (OP == CAS2 && !BIW_2[15]) ? 1'b1 :
                  ((OP == CHK2 || OP == CMP2) && !BIW_1[15] && FETCH_STATE == FETCH_OPERAND && RD_RDY && PHASE2) ? 1'b1 : // Select destination register.
                  ((OP == CHK2 || OP == CMP2) && !BIW_1[15] && ALU_INIT_I) ? 1'b1 : 1'b0; // Store compare information

endmodule
