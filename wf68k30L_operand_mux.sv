//----------------------------------------------------------------------//
//                                                                      //
// WF68K30L IP Core: Operand routing and multiplexing.                  //
//                                                                      //
// This module contains all the combinational operand routing logic      //
// extracted from the top-level module:                                  //
//   - DATA_IMMEDIATE multiplexer                                       //
//   - ALU_OP1_IN multiplexer                                           //
//   - ALU_OP2_IN multiplexer                                           //
//   - ALU_OP3_IN multiplexer                                           //
//   - Register file input multiplexers (AR_IN_1, AR_IN_2, DR_IN_1/2)  //
//   - Bit field offset/width and BITPOS logic                          //
//   - Branch prediction logic (BRANCH_ATN)                             //
//   - DBcc condition                                                   //
//                                                                      //
//----------------------------------------------------------------------//

module WF68K30L_OPERAND_MUX (
    input  logic        CLK,

    // --- Instruction / operation context ---
    input  logic [6:0]  OP,
    input  logic [6:0]  OP_WB,
    input  logic [1:0]  OP_SIZE,
    input  logic [15:0] BIW_0,
    input  logic [7:3]  BIW_0_WB_73,
    input  logic [15:0] BIW_1,
    input  logic [15:0] BIW_2,
    input  logic [2:0]  ADR_MODE,

    // --- Data sources ---
    input  logic [31:0] DATA_TO_CORE,
    input  logic [31:0] DR_OUT_1,
    input  logic [31:0] DR_OUT_2,
    input  logic [31:0] AR_OUT_1,
    input  logic [31:0] AR_OUT_2,
    input  logic [31:0] ADR_EFF,
    input  logic [31:0] PC,
    input  logic [3:0]  PC_EW_OFFSET,
    input  logic [15:0] STATUS_REG,
    input  logic [31:0] VBR,
    input  logic [2:0]  SFC,
    input  logic [2:0]  DFC,
    input  logic [63:0] ALU_RESULT,

    // --- Immediate data buffer inputs ---
    input  logic        STORE_IDATA_B1,
    input  logic        STORE_IDATA_B2,
    input  logic [15:0] EXT_WORD,

    // --- Address offset inputs ---
    input  logic        BUSY_MAIN,
    input  logic [31:0] ADR_OFFSET_EXH,
    input  logic [5:0]  ADR_OFFSET_MAIN,

    // --- Control flags ---
    input  logic        BUSY_EXH,
    input  logic        ALU_BSY,
    input  logic        AR_WR_1,
    input  logic        DR_WR_1,
    input  logic        DFC_WR,
    input  logic        SFC_WR,
    input  logic        ISP_WR,
    input  logic        MSP_WR,
    input  logic        USP_WR,
    input  logic        FETCH_MEM_ADR,
    input  logic        USE_DREG,
    input  logic        VBR_RD,
    input  logic        SFC_RD,
    input  logic        DFC_RD,
    input  logic        ISP_RD,
    input  logic        MSP_RD,
    input  logic        USP_RD,
    input  logic        SR_WR_EXH,
    input  logic        ADn,
    input  int          MOVEP_PNTR,

    // --- Outputs ---
    output logic [31:0] ALU_OP1_IN,
    output logic [31:0] ALU_OP2_IN,
    output logic [31:0] ALU_OP3_IN,
    output logic [31:0] AR_IN_1,
    output logic [31:0] AR_IN_2,
    output logic [31:0] DR_IN_1,
    output logic [31:0] DR_IN_2,
    output logic [31:0] BF_OFFSET,
    output logic [5:0]  BF_WIDTH,
    output logic [4:0]  BITPOS,
    output logic        BRANCH_ATN,
    output logic        DBcc_COND,
    output logic [31:0] DATA_IMMEDIATE,
    output logic [31:0] ADR_OFFSET
);

`include "wf68k30L_pkg.svh"

logic [31:0] IBUFFER;

// ========================================================================
// Immediate data buffer
// ========================================================================

always_ff @(posedge CLK) begin : idata_buffer
    if (STORE_IDATA_B2)
        IBUFFER[31:16] <= EXT_WORD;
    else if (STORE_IDATA_B1)
        IBUFFER[15:0] <= EXT_WORD;
end

// ========================================================================
// Immediate data multiplexer
// ========================================================================
// Selects the correct immediate value based on the current operation and
// operand size. LONG immediates come from concatenated BIW words; shorter
// sizes are zero-extended from BIW_1 or the instruction word itself.

always_comb begin : data_immediate_mux
    // Check for operations with dedicated immediate encodings first.
    if ((OP == ADDQ || OP == SUBQ) && BIW_0[11:9] == 3'b000)
        DATA_IMMEDIATE = 32'h8; // Quick #8 encoded as 000 in the instruction.
    else if (OP == ADDQ || OP == SUBQ)
        DATA_IMMEDIATE = {24'h0, 5'b00000, BIW_0[11:9]};
    else if (OP == MOVEQ)
        DATA_IMMEDIATE = {24'h0, BIW_0[7:0]};
    else if (OP == DBcc)
        DATA_IMMEDIATE = 32'h1;
    else if (OP == PACK || OP == UNPK)
        DATA_IMMEDIATE = {16'h0, BIW_1[15:0]};
    // Status register immediate operations (always word or byte).
    else if (OP == ANDI_TO_SR || OP == EORI_TO_SR || OP == ORI_TO_SR || OP == STOP)
        DATA_IMMEDIATE = {16'h0, BIW_1};
    else if (OP == ANDI_TO_CCR || OP == EORI_TO_CCR || OP == ORI_TO_CCR)
        DATA_IMMEDIATE = {24'h0, BIW_1[7:0]};
    // Sized ALU immediates (ADDI, ANDI, CMPI, EORI, SUBI, ORI).
    else if ((OP == ADDI || OP == ANDI || OP == CMPI || OP == EORI || OP == SUBI || OP == ORI) && OP_SIZE == LONG)
        DATA_IMMEDIATE = {BIW_1, BIW_2};
    else if ((OP == ADDI || OP == ANDI || OP == CMPI || OP == EORI || OP == SUBI || OP == ORI) && OP_SIZE == WORD)
        DATA_IMMEDIATE = {16'h0, BIW_1};
    else if (OP == ADDI || OP == ANDI || OP == CMPI || OP == EORI || OP == SUBI || OP == ORI)
        DATA_IMMEDIATE = {24'h0, BIW_1[7:0]}; // BYTE
    // Default: from the immediate data buffer.
    else if (OP_SIZE == LONG)
        DATA_IMMEDIATE = IBUFFER;
    else
        DATA_IMMEDIATE = {16'h0, IBUFFER[15:0]};
end

// ========================================================================
// Register file input multiplexers
// ========================================================================

// Address register port 1 input: exception handler, ALU writeback, or instruction-specific routing.
always_comb begin : ar_in_1_mux
    if (BUSY_EXH)
        AR_IN_1 = DATA_TO_CORE;
    else if (ALU_BSY && AR_WR_1)
        AR_IN_1 = ALU_RESULT[31:0];
    else if (ALU_BSY && (DFC_WR || SFC_WR || ISP_WR || MSP_WR || USP_WR))
        AR_IN_1 = ALU_RESULT[31:0];
    else if (OP == JMP || OP == JSR)
        AR_IN_1 = ADR_EFF;
    else if (FETCH_MEM_ADR)
        AR_IN_1 = DATA_TO_CORE;
    else if (USE_DREG)
        AR_IN_1 = DR_OUT_1; // CAS2: Address register from data register.
    else if (OP == LINK || OP == UNLK)
        AR_IN_1 = AR_OUT_1;
    else
        AR_IN_1 = DATA_TO_CORE; // Default used for RTD, RTR, RTS.
end

assign AR_IN_2 = (OP_WB == EXG) ? ALU_RESULT[63:32] : ALU_RESULT[31:0];

// Data register port 1 input: EXG swaps upper/lower result halves.
assign DR_IN_1 = (OP_WB == EXG && ALU_BSY && DR_WR_1 && BIW_0_WB_73 == 5'b10001) ? ALU_RESULT[63:32] :
                  ALU_RESULT[31:0];

assign DR_IN_2 = ALU_RESULT[63:32];

// ========================================================================
// ALU operand 1 multiplexer
// ========================================================================
// Routes the first ALU operand from the appropriate source based on the
// current instruction. Priority ordering prevents data hazards.

always_comb begin : alu_op1_mux
    if (SR_WR_EXH)
        ALU_OP1_IN = DATA_TO_CORE;
    else if (OP == DBcc || OP == PACK || OP == UNPK)
        ALU_OP1_IN = DATA_IMMEDIATE;
    // BCD operations: register direct vs memory.
    else if ((OP == ABCD || OP == SBCD) && !BIW_0[3])
        ALU_OP1_IN = DR_OUT_1;
    else if (OP == ABCD || OP == SBCD)
        ALU_OP1_IN = DATA_TO_CORE;
    // ADD/SUB direction bit [8]: 1=Dn op EA->EA, 0=EA op Dn->Dn.
    else if ((OP == ADD || OP == SUB) && BIW_0[8])
        ALU_OP1_IN = DR_OUT_1;
    else if ((OP == AND_B || OP == EOR || OP == OR_B) && BIW_0[8])
        ALU_OP1_IN = DR_OUT_1;
    // Extended arithmetic: register direct vs memory.
    else if ((OP == ADDX || OP == SUBX) && !BIW_0[3])
        ALU_OP1_IN = DR_OUT_1;
    else if (OP == ADDX || OP == SUBX)
        ALU_OP1_IN = DATA_TO_CORE;
    // Shift/rotate register operand.
    else if (OP == ASL || OP == ASR || OP == LSL || OP == LSR)
        ALU_OP1_IN = DR_OUT_1;
    else if (OP == ROTL || OP == ROTR || OP == ROXL || OP == ROXR)
        ALU_OP1_IN = DR_OUT_1;
    // Bit field insert pattern.
    else if (OP == BFINS)
        ALU_OP1_IN = DR_OUT_2;
    // CAS/CAS2 compare operand.
    else if (OP == CAS || OP == CAS2)
        ALU_OP1_IN = DR_OUT_1;
    else if (OP == CMPM)
        ALU_OP1_IN = DATA_TO_CORE;
    // Subroutine calls push return address.
    else if (OP == BSR || OP == JSR)
        ALU_OP1_IN = PC + PC_EW_OFFSET;
    // Load effective address.
    else if (OP == LEA || OP == PEA)
        ALU_OP1_IN = ADR_EFF;
    // MOVE from address register.
    else if (OP == MOVE && BIW_0[5:3] == ADR_AN)
        ALU_OP1_IN = AR_OUT_2;
    else if (OP == MOVE_USP)
        ALU_OP1_IN = AR_OUT_1;
    // EXG: two address registers vs two data registers.
    else if (OP == EXG && BIW_0[7:3] == 5'b01001)
        ALU_OP1_IN = AR_OUT_1;
    else if (OP == EXG)
        ALU_OP1_IN = DR_OUT_1;
    // MOVEC: source depends on control register being accessed.
    else if (OP == MOVEC && VBR_RD)
        ALU_OP1_IN = VBR;
    else if (OP == MOVEC && SFC_RD)
        ALU_OP1_IN = {28'h0, 1'b0, SFC};
    else if (OP == MOVEC && DFC_RD)
        ALU_OP1_IN = {28'h0, 1'b0, DFC};
    else if (OP == MOVEC && (ISP_RD || MSP_RD || USP_RD))
        ALU_OP1_IN = AR_OUT_1;
    else if (OP == MOVEC && BIW_1[15])
        ALU_OP1_IN = AR_OUT_1;
    else if (OP == MOVEC)
        ALU_OP1_IN = DR_OUT_1;
    // MOVEM: register to memory.
    else if (OP == MOVEM && !BIW_0[10] && !ADn)
        ALU_OP1_IN = DR_OUT_1;
    else if (OP == MOVEM && !BIW_0[10])
        ALU_OP1_IN = AR_OUT_2;
    // MOVES: register to memory.
    else if (OP == MOVES && BIW_1[11] && !BIW_1[15])
        ALU_OP1_IN = DR_OUT_1;
    else if (OP == MOVES && BIW_1[11])
        ALU_OP1_IN = AR_OUT_2;
    // MOVEP: byte extraction/insertion by pointer position.
    else if (OP == MOVEP && MOVEP_PNTR == 3 && BIW_0[7:6] > 2'b01)
        ALU_OP1_IN = {24'h0, DR_OUT_1[31:24]};
    else if (OP == MOVEP && MOVEP_PNTR == 2 && BIW_0[7:6] > 2'b01)
        ALU_OP1_IN = {24'h0, DR_OUT_1[23:16]};
    else if (OP == MOVEP && MOVEP_PNTR == 1 && BIW_0[7:6] > 2'b01)
        ALU_OP1_IN = {24'h0, DR_OUT_1[15:8]};
    else if (OP == MOVEP && BIW_0[7:6] > 2'b01)
        ALU_OP1_IN = {24'h0, DR_OUT_1[7:0]};
    else if (OP == MOVEP && MOVEP_PNTR == 3)
        ALU_OP1_IN = {DATA_TO_CORE[7:0], DR_OUT_1[23:0]};
    else if (OP == MOVEP && MOVEP_PNTR == 2)
        ALU_OP1_IN = {DR_OUT_1[31:24], DATA_TO_CORE[7:0], DR_OUT_1[15:0]};
    else if (OP == MOVEP && MOVEP_PNTR == 1)
        ALU_OP1_IN = {DR_OUT_1[31:16], DATA_TO_CORE[7:0], DR_OUT_1[7:0]};
    else if (OP == MOVEP)
        ALU_OP1_IN = {DR_OUT_1[31:8], DATA_TO_CORE[7:0]};
    // MOVE to CCR/SR: source depends on EA mode.
    else if (OP == MOVE_TO_CCR && BIW_0[5:3] == ADR_DN)
        ALU_OP1_IN = {16'h0, STATUS_REG[15:8], DR_OUT_1[7:0]};
    else if (OP == MOVE_TO_CCR && BIW_0[5:0] == 6'b111100)
        ALU_OP1_IN = {16'h0, STATUS_REG[15:8], DATA_IMMEDIATE[7:0]};
    else if (OP == MOVE_TO_CCR)
        ALU_OP1_IN = {16'h0, STATUS_REG[15:8], DATA_TO_CORE[7:0]};
    else if (OP == MOVE_TO_SR && BIW_0[5:3] == ADR_DN)
        ALU_OP1_IN = {16'h0, DR_OUT_1[15:0]};
    else if (OP == MOVE_TO_SR && BIW_0[5:0] == 6'b111100)
        ALU_OP1_IN = {16'h0, DATA_IMMEDIATE[15:0]};
    else if (OP == MOVE_TO_SR)
        ALU_OP1_IN = {16'h0, DATA_TO_CORE[15:0]};
    // MOVE from CCR/SR.
    else if (OP == MOVE_FROM_CCR)
        ALU_OP1_IN = {24'h0, 3'b000, STATUS_REG[4:0]};
    else if (OP == MOVE_FROM_SR)
        ALU_OP1_IN = {16'h0, STATUS_REG};
    // STOP loads immediate into SR.
    else if (OP == STOP)
        ALU_OP1_IN = DATA_IMMEDIATE;
    else if (OP == MOVEQ)
        ALU_OP1_IN = DATA_IMMEDIATE;
    // Negate/BCD negate: second operand is zero.
    else if (OP == NEG || OP == NEGX || OP == NBCD)
        ALU_OP1_IN = 32'h0;
    // Immediate ALU operations.
    else if (OP == ADDI || OP == CMPI || OP == SUBI || OP == ANDI || OP == EORI || OP == ORI)
        ALU_OP1_IN = DATA_IMMEDIATE;
    else if (OP == ADDQ || OP == SUBQ)
        ALU_OP1_IN = DATA_IMMEDIATE;
    else if (OP == ANDI_TO_CCR || OP == ANDI_TO_SR)
        ALU_OP1_IN = DATA_IMMEDIATE;
    else if (OP == EORI_TO_CCR || OP == EORI_TO_SR)
        ALU_OP1_IN = DATA_IMMEDIATE;
    else if (OP == ORI_TO_CCR || OP == ORI_TO_SR)
        ALU_OP1_IN = DATA_IMMEDIATE;
    // Default EA-mode routing.
    else if (BIW_0[5:3] == ADR_DN)
        ALU_OP1_IN = DR_OUT_1;
    else if (BIW_0[5:3] == ADR_AN)
        ALU_OP1_IN = AR_OUT_1;
    else if (BIW_0[5:0] == 6'b111100)
        ALU_OP1_IN = DATA_IMMEDIATE;
    else
        ALU_OP1_IN = DATA_TO_CORE;
end

// ========================================================================
// ALU operand 2 multiplexer
// ========================================================================

always_comb begin : alu_op2_mux
    // BCD: register direct vs memory.
    if ((OP == ABCD || OP == SBCD) && !BIW_0[3])
        ALU_OP2_IN = DR_OUT_2;
    else if (OP == ABCD || OP == SBCD)
        ALU_OP2_IN = DATA_TO_CORE;
    // Extended arithmetic.
    else if ((OP == ADDX || OP == SUBX) && !BIW_0[3])
        ALU_OP2_IN = DR_OUT_2;
    else if (OP == ADDX || OP == SUBX)
        ALU_OP2_IN = DATA_TO_CORE;
    // ADD/CMP/SUB: direction bit selects register vs memory.
    else if ((OP == ADD || OP == CMP || OP == SUB) && !BIW_0[8])
        ALU_OP2_IN = DR_OUT_2;
    else if ((OP == AND_B || OP == OR_B) && !BIW_0[8])
        ALU_OP2_IN = DR_OUT_2;
    // Address register arithmetic.
    else if (OP == ADDA || OP == CMPA || OP == SUBA)
        ALU_OP2_IN = AR_OUT_2;
    // EXG: two data registers vs data+address or two address registers.
    else if (OP == EXG && BIW_0[7:3] == 5'b01000)
        ALU_OP2_IN = DR_OUT_2;
    else if (OP == EXG)
        ALU_OP2_IN = AR_OUT_2;
    // Register shifts/rotates (not memory shifts, which use BIW_0[7:6]==11).
    else if ((OP == ASL || OP == ASR) && BIW_0[7:6] != 2'b11)
        ALU_OP2_IN = DR_OUT_2;
    else if ((OP == LSL || OP == LSR) && BIW_0[7:6] != 2'b11)
        ALU_OP2_IN = DR_OUT_2;
    else if ((OP == ROTL || OP == ROTR) && BIW_0[7:6] != 2'b11)
        ALU_OP2_IN = DR_OUT_2;
    else if ((OP == ROXL || OP == ROXR) && BIW_0[7:6] != 2'b11)
        ALU_OP2_IN = DR_OUT_2;
    // Immediate-to-SR operations combine immediate with current SR.
    else if (OP == ANDI_TO_CCR || OP == ANDI_TO_SR)
        ALU_OP2_IN = {16'h0, STATUS_REG};
    else if (OP == EORI_TO_CCR || OP == EORI_TO_SR)
        ALU_OP2_IN = {16'h0, STATUS_REG};
    else if (OP == ORI_TO_CCR || OP == ORI_TO_SR)
        ALU_OP2_IN = {16'h0, STATUS_REG};
    // CAS/CAS2 destination operand.
    else if (OP == CAS || OP == CAS2)
        ALU_OP2_IN = DATA_TO_CORE;
    // CHK uses Dn as the tested operand.
    else if (OP == CHK)
        ALU_OP2_IN = DR_OUT_2;
    // CHK2/CMP2 can test either Dn or An.
    else if ((OP == CHK2 || OP == CMP2) && USE_DREG)
        ALU_OP2_IN = DR_OUT_2;
    else if (OP == CHK2 || OP == CMP2)
        ALU_OP2_IN = AR_OUT_2;
    else if (OP == CMPM)
        ALU_OP2_IN = DATA_TO_CORE;
    else if (OP == DBcc || OP == SWAP)
        ALU_OP2_IN = DR_OUT_2;
    else if (OP == DIVS || OP == DIVU)
        ALU_OP2_IN = DR_OUT_2;
    else if (OP == MULS || OP == MULU)
        ALU_OP2_IN = DR_OUT_2;
    // PACK/UNPK: register direct vs memory.
    else if ((OP == PACK || OP == UNPK) && !BIW_0[3])
        ALU_OP2_IN = DR_OUT_1;
    else if (OP == PACK || OP == UNPK)
        ALU_OP2_IN = DATA_TO_CORE;
    else if (OP == LINK)
        ALU_OP2_IN = AR_OUT_1;
    // Default EA-mode routing.
    else if (BIW_0[5:3] == ADR_DN)
        ALU_OP2_IN = DR_OUT_2;
    else if (BIW_0[5:3] == ADR_AN)
        ALU_OP2_IN = AR_OUT_2;
    else
        ALU_OP2_IN = DATA_TO_CORE;
end

// ========================================================================
// ALU operand 3 multiplexer (bit field / CAS2 / CHK2)
// ========================================================================

assign ALU_OP3_IN = ((OP == BFCHG || OP == BFCLR || OP == BFEXTS || OP == BFEXTU) && BIW_0[5:3] != ADR_DN) ? DATA_TO_CORE :
                     ((OP == BFFFO || OP == BFINS || OP == BFSET || OP == BFTST) && BIW_0[5:3] != ADR_DN) ? DATA_TO_CORE :
                     (OP == CAS2 || OP == CHK2 || OP == CMP2) ? DATA_TO_CORE : DR_OUT_1;

// ========================================================================
// Bit field offset/width and BITPOS
// ========================================================================

// The bit field offset is bit-wise.
assign BF_OFFSET = !BIW_1[11] ? {24'h0, 3'b000, BIW_1[10:6]} : DR_OUT_1;
assign BF_WIDTH = (BIW_1[4:0] != 5'b00000 && !BIW_1[5]) ? {1'b0, BIW_1[4:0]} :
                   (DR_OUT_1[4:0] != 5'b00000 && BIW_1[5]) ? {1'b0, DR_OUT_1[4:0]} : 6'b100000;

// BITPOS spans 0-31 for register direct mode, modulo-8 for memory mode.
// For bit field operations in memory, values are 0-7 (byte-aligned loads).
always_comb begin : bitpos_mux
    if ((OP == BCHG || OP == BCLR || OP == BSET || OP == BTST) && !BIW_0[8] && ADR_MODE == ADR_DN)
        BITPOS = BIW_1[4:0];
    else if ((OP == BCHG || OP == BCLR || OP == BSET || OP == BTST) && !BIW_0[8])
        BITPOS = {2'b00, BIW_1[2:0]};
    else if ((OP == BCHG || OP == BCLR || OP == BSET || OP == BTST) && ADR_MODE == ADR_DN)
        BITPOS = DR_OUT_1[4:0];
    else if (OP == BCHG || OP == BCLR || OP == BSET || OP == BTST)
        BITPOS = {2'b00, DR_OUT_1[2:0]};
    else if (!BIW_1[11] && ADR_MODE == ADR_DN)
        BITPOS = BIW_1[10:6];
    else if (!BIW_1[11])
        BITPOS = {2'b00, BIW_1[8:6]};
    else if (ADR_MODE == ADR_DN)
        BITPOS = DR_OUT_1[4:0];
    else
        BITPOS = {2'b00, DR_OUT_1[2:0]};
end

// ========================================================================
// Condition and branch prediction
// ========================================================================

assign DBcc_COND = (OP_WB == DBcc && ALU_RESULT[15:0] == 16'hFFFF);

// Predict a branch if an SR-modifying instruction will change the CPU space.
always_comb begin : branch_prediction
    BRANCH_ATN = 1'b0;
    // Use raw immediate extension word bits for SR writes to avoid
    // introducing a control-loop dependency through DATA_IMMEDIATE/OP_SIZE.
    if (OP == ANDI_TO_SR && !BIW_1[13] && STATUS_REG[13])
        BRANCH_ATN = 1'b1;
    else if (OP == ANDI_TO_SR && !BIW_1[12] && STATUS_REG[12])
        BRANCH_ATN = 1'b1;
    else if (OP == EORI_TO_SR && BIW_1[13])
        BRANCH_ATN = 1'b1;
    else if (OP == EORI_TO_SR && BIW_1[12])
        BRANCH_ATN = 1'b1;
    else if (OP == ORI_TO_SR && BIW_1[13] && !STATUS_REG[13])
        BRANCH_ATN = 1'b1;
    else if (OP == ORI_TO_SR && BIW_1[12] && !STATUS_REG[12])
        BRANCH_ATN = 1'b1;
    else if (OP == MOVE_TO_SR && BIW_0[5:3] == ADR_DN && DR_OUT_1[13:12] != STATUS_REG[13:12])
        BRANCH_ATN = 1'b1;
    else if (OP == MOVE_TO_SR && BIW_0[5:0] == 6'b111100 && BIW_1[13:12] != STATUS_REG[13:12])
        BRANCH_ATN = 1'b1;
    else if (OP == MOVE_TO_SR && DATA_TO_CORE[13:12] != STATUS_REG[13:12])
        BRANCH_ATN = 1'b1;
end

// ========================================================================
// Address offset multiplexer
// ========================================================================
// Byte-aligned bit field offset for address calculation.

always_comb begin : adr_offset_mux
    if (FETCH_MEM_ADR)
        ADR_OFFSET = 32'h0;
    else if (BUSY_EXH)
        ADR_OFFSET = ADR_OFFSET_EXH;
    else if ((OP == BFCHG || OP == BFCLR || OP == BFEXTS || OP == BFEXTU) && !BF_OFFSET[31])
        ADR_OFFSET = {3'b000, BF_OFFSET[31:3]} + ADR_OFFSET_MAIN;
    else if (OP == BFCHG || OP == BFCLR || OP == BFEXTS || OP == BFEXTU)
        ADR_OFFSET = {3'b111, BF_OFFSET[31:3]} + ADR_OFFSET_MAIN;
    else if ((OP == BFFFO || OP == BFINS || OP == BFSET || OP == BFTST) && !BF_OFFSET[31])
        ADR_OFFSET = {3'b000, BF_OFFSET[31:3]} + ADR_OFFSET_MAIN;
    else if (OP == BFFFO || OP == BFINS || OP == BFSET || OP == BFTST)
        ADR_OFFSET = {3'b111, BF_OFFSET[31:3]} + ADR_OFFSET_MAIN;
    else
        ADR_OFFSET = {24'h0, 2'b00, ADR_OFFSET_MAIN};
end

endmodule
