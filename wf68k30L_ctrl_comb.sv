//--------------------------------------------------------------------//
//                                                                    //
// WF68K30L IP Core: Combinational output-assign decoder.             //
//                                                                    //
// This module contains the bulk of the combinational assign logic    //
// that was previously inline in wf68k30L_control.sv. It computes    //
// bus control, store strobes, LOAD_OP, OP_SIZE, ALU control,        //
// mark-used signals, PC/pipe control, system register access,       //
// traps, and miscellaneous control outputs.                          //
//                                                                    //
//--------------------------------------------------------------------//

module WF68K30L_CTRL_COMB #(
    parameter NO_PIPELINE = 0
) (
    // Current states
    input  logic [4:0]  FETCH_STATE,
    input  logic [4:0]  NEXT_FETCH_STATE,
    input  logic [2:0]  EXEC_WB_STATE,
    input  logic [2:0]  NEXT_EXEC_WB_STATE,

    // Operation
    input  logic [6:0]  OP,
    input  logic [6:0]  OP_WB_I,
    input  logic [13:0] BIW_0,
    input  logic [15:0] BIW_1,
    input  logic [15:0] BIW_2,
    input  logic [11:0] BIW_0_WB,
    input  logic [15:0] BIW_1_WB,

    // Data availability
    input  logic        OPD_ACK,
    input  logic        OW_VALID,
    input  logic        OW_RDY,
    input  logic        EW_ACK,
    input  logic        EW_RDY,
    input  logic        DATA_RDY,
    input  logic        DATA_VALID,
    input  logic        MEMADR_RDY,
    input  logic        READ_CYCLE,
    input  logic        WRITE_CYCLE,

    // Hazard signals
    input  logic        ADR_IN_USE,
    input  logic        DR_IN_USE,
    input  logic        AR_IN_USE,

    // ALU
    input  logic        ALU_BSY,
    input  logic        ALU_REQ,
    input  logic        ALU_COND,

    // Internal state
    input  logic        PHASE2,
    input  logic        LOOP_BSY,

    // MOVEM
    input  logic        MOVEM_ADn_I,
    input  logic        MOVEM_COND,
    input  logic        MOVEM_FIRST_RD,

    // Bitfield
    input  int          BF_BYTES,
    input  logic        BF_HILOn,

    // Control
    input  logic        BRANCH_ATN,
    input  logic        DBcc_COND,
    input  logic [1:0]  TRACE_MODE,
    input  logic        VBIT,
    input  logic        EXH_REQ,
    input  logic        BUSY_EXH,

    // --- Outputs ---

    // Top-level control
    output logic        BUSY,
    output logic        INT_TRIG,
    output logic        OW_REQ,

    // Bus control
    output logic        EW_REQ,
    output logic        DATA_RD,
    output logic        DATA_WR,
    output logic        RMC,
    output logic        ALU_ACK,

    // Derived ready/entry signals (used by clocked processes in control.sv)
    output logic        RD_RDY,
    output logic        WR_RDY,
    output logic        INIT_ENTRY,

    // Store signals
    output logic        FETCH_MEM_ADR,
    output logic        STORE_MEM_ADR,
    output logic        STORE_ADR_FORMAT,
    output logic        STORE_D16,
    output logic        STORE_D32_LO,
    output logic        STORE_D32_HI,
    output logic        STORE_DISPL,
    output logic        STORE_OD_HI,
    output logic        STORE_OD_LO,
    output logic        STORE_ABS_HI,
    output logic        STORE_ABS_LO,
    output logic        STORE_IDATA_B2,
    output logic        STORE_IDATA_B1,

    // Load operand
    output logic        LOAD_OP1,
    output logic        LOAD_OP2,
    output logic        LOAD_OP3,

    // SR / status
    output logic        SR_WR,
    output logic        HILOn,

    // Addressing mode
    output logic [2:0]  ADR_MODE,
    output logic [2:0]  AMODE_SEL,

    // Operation size
    output logic [1:0]  OP_SIZE,

    // BKPT / Traps / BERR
    output logic        BKPT_CYCLE,
    output logic        BKPT_INSERT,
    output logic        TRAP_ILLEGAL,
    output logic        TRAP_cc,
    output logic        TRAP_V,
    output logic        BERR,

    // System register access
    output logic        SFC_RD,
    output logic        SFC_WR,
    output logic        DFC_RD,
    output logic        DFC_WR,
    output logic        VBR_RD,
    output logic        VBR_WR,
    output logic        ISP_RD,
    output logic        ISP_WR,
    output logic        MSP_RD,
    output logic        MSP_WR,
    output logic        USP_RD,
    output logic        USP_WR,

    // PC / pipe control
    output logic        PC_ADD_DISPL,
    output logic        PC_LOAD,
    output logic        IPIPE_FLUSH,
    output logic        SP_ADD_DISPL,

    // ALU control
    output logic        ALU_INIT,

    // Condition code update
    output logic        CC_UPDT,

    // Mark-used signals
    output logic        ADR_MARK_USED,
    output logic        AR_MARK_USED,
    output logic        DR_MARK_USED,
    output logic        UNMARK,
    output logic        USE_APAIR,
    output logic        USE_DPAIR,

    // Loop control
    output logic        LOOP_EXIT,

    // Misc
    output logic        RESET_STRB,
    output logic        EX_TRACE
);

`include "wf68k30L_pkg.svh"

// Fetch state constants
localparam logic [4:0] START_OP       = 5'd0;
localparam logic [4:0] CALC_AEFF      = 5'd1;
localparam logic [4:0] FETCH_DISPL    = 5'd2;
localparam logic [4:0] FETCH_EXWORD_1 = 5'd3;
localparam logic [4:0] FETCH_D_LO     = 5'd4;
localparam logic [4:0] FETCH_D_HI     = 5'd5;
localparam logic [4:0] FETCH_OD_HI    = 5'd6;
localparam logic [4:0] FETCH_OD_LO    = 5'd7;
localparam logic [4:0] FETCH_ABS_HI   = 5'd8;
localparam logic [4:0] FETCH_ABS_LO   = 5'd9;
localparam logic [4:0] FETCH_IDATA_B2 = 5'd10;
localparam logic [4:0] FETCH_IDATA_B1 = 5'd11;
localparam logic [4:0] FETCH_MEMADR   = 5'd12;
localparam logic [4:0] FETCH_OPERAND  = 5'd13;
localparam logic [4:0] INIT_EXEC_WB   = 5'd14;
localparam logic [4:0] SLEEP          = 5'd15;
localparam logic [4:0] SWITCH_STATE   = 5'd16;

// Exec/WB state constants
localparam logic [2:0] IDLE           = 3'd0;
localparam logic [2:0] EXECUTE        = 3'd1;
localparam logic [2:0] ADR_PIPELINE   = 3'd2;
localparam logic [2:0] WRITEBACK      = 3'd3;
localparam logic [2:0] WRITE_DEST     = 3'd4;

// Internal signals
logic        DATA_RD_I;
logic        DATA_WR_I;
logic        SR_WR_I;
logic        ALU_TRIG;
logic        ALU_INIT_I;
logic        UPDT_CC;
logic        ADR_MARK_USED_I;
logic        PC_ADD_DISPL_I;
logic        PC_LOAD_I;
logic        IPIPE_FLUSH_I;
logic        LOOP_EXIT_I;
logic [2:0]  ADR_MODE_I;
logic [1:0]  OP_SIZE_I;

// ====================================================================
// Top-level control
// ====================================================================

assign BUSY = OPD_ACK ||              // Early indication.
              LOOP_BSY ||              // Finish the DBcc loop.
              ALU_BSY ||               // Busy, wait.
              (FETCH_STATE != START_OP); // Main controller is busy.

assign INT_TRIG = (FETCH_STATE == INIT_EXEC_WB || FETCH_STATE == SLEEP);

assign OW_REQ = (BUSY_EXH) ? 1'b0 :
                (EXH_REQ && !LOOP_BSY) ? 1'b0 :
                (OPD_ACK || OW_RDY) ? 1'b0 :
                (NO_PIPELINE == 1 && FETCH_STATE == START_OP && !ALU_BSY) ? 1'b1 :
                (NO_PIPELINE == 0 && FETCH_STATE == START_OP) ? 1'b1 : 1'b0;

// ====================================================================
// Derived ready / entry signals
// ====================================================================

assign RD_RDY = (READ_CYCLE) ? DATA_RDY : 1'b0;
assign WR_RDY = (WRITE_CYCLE) ? DATA_RDY : 1'b0;
assign INIT_ENTRY = (FETCH_STATE != INIT_EXEC_WB && NEXT_FETCH_STATE == INIT_EXEC_WB) ? 1'b1 : 1'b0;

// ====================================================================
// Bus control
// ====================================================================

assign EW_REQ = (EW_ACK || EW_RDY) ? 1'b0 :
                (FETCH_STATE == FETCH_DISPL || FETCH_STATE == FETCH_EXWORD_1) ? 1'b1 :
                (FETCH_STATE == FETCH_D_HI || FETCH_STATE == FETCH_D_LO) ? 1'b1 :
                (FETCH_STATE == FETCH_OD_HI || FETCH_STATE == FETCH_OD_LO) ? 1'b1 :
                (FETCH_STATE == FETCH_ABS_HI || FETCH_STATE == FETCH_ABS_LO) ? 1'b1 :
                (FETCH_STATE == FETCH_IDATA_B2 || FETCH_STATE == FETCH_IDATA_B1) ? 1'b1 :
                (FETCH_STATE != FETCH_DISPL && NEXT_FETCH_STATE == FETCH_DISPL) ? 1'b1 :
                (FETCH_STATE != FETCH_EXWORD_1 && NEXT_FETCH_STATE == FETCH_EXWORD_1) ? 1'b1 :
                (FETCH_STATE != FETCH_D_HI && NEXT_FETCH_STATE == FETCH_D_HI) ? 1'b1 :
                (FETCH_STATE != FETCH_D_LO && NEXT_FETCH_STATE == FETCH_D_LO) ? 1'b1 :
                (FETCH_STATE != FETCH_OD_HI && NEXT_FETCH_STATE == FETCH_OD_HI) ? 1'b1 :
                (FETCH_STATE != FETCH_OD_LO && NEXT_FETCH_STATE == FETCH_OD_LO) ? 1'b1 :
                (FETCH_STATE != FETCH_ABS_HI && NEXT_FETCH_STATE == FETCH_ABS_HI) ? 1'b1 :
                (FETCH_STATE != FETCH_ABS_LO && NEXT_FETCH_STATE == FETCH_ABS_LO) ? 1'b1 :
                (FETCH_STATE != FETCH_IDATA_B2 && NEXT_FETCH_STATE == FETCH_IDATA_B2) ? 1'b1 :
                (FETCH_STATE != FETCH_IDATA_B1 && NEXT_FETCH_STATE == FETCH_IDATA_B1) ? 1'b1 : 1'b0;

assign DATA_RD = DATA_RD_I;
assign DATA_RD_I = (DATA_WR_I && !READ_CYCLE && !WRITE_CYCLE) ? 1'b0 : // Write is prioritized.
                   (WRITE_CYCLE) ? 1'b0 : // Do not read during a write cycle.
                   (ADR_IN_USE) ? 1'b0 : // Avoid data hazards.
                   (DATA_RDY || MEMADR_RDY) ? 1'b0 :
                   (FETCH_STATE == FETCH_MEMADR) ? 1'b1 :
                   (FETCH_STATE == FETCH_OPERAND) ? 1'b1 : 1'b0;

assign DATA_WR = DATA_WR_I;
assign DATA_WR_I = (READ_CYCLE) ? 1'b0 : // Do not write during a read cycle.
                   (DATA_RDY) ? 1'b0 :
                   (EXEC_WB_STATE == WRITE_DEST) ? 1'b1 : 1'b0;

assign RMC = ((OP == CAS || OP == CAS2 || OP == TAS) && FETCH_STATE != START_OP) ? 1'b1 : 1'b0;

assign ALU_ACK = (EXEC_WB_STATE == EXECUTE && NEXT_EXEC_WB_STATE == IDLE) ? 1'b1 :
                 (EXEC_WB_STATE == WRITEBACK) ? 1'b1 :
                 ((OP_WB_I == BFCHG || OP_WB_I == BFCLR) && EXEC_WB_STATE == WRITE_DEST && BF_BYTES == 5) ? 1'b0 :
                 ((OP_WB_I == BFINS || OP_WB_I == BFSET) && EXEC_WB_STATE == WRITE_DEST && BF_BYTES == 5) ? 1'b0 :
                 (EXEC_WB_STATE == WRITE_DEST && WR_RDY) ? 1'b1 : 1'b0;

// ====================================================================
// Store / fetch signals
// ====================================================================

assign FETCH_MEM_ADR = (FETCH_STATE == FETCH_MEMADR) ? 1'b1 : 1'b0;
assign STORE_MEM_ADR = (FETCH_STATE == FETCH_MEMADR && RD_RDY && DATA_VALID) ? 1'b1 : 1'b0;

// Store the extension word right in the end due to used data and/or address registers.
assign STORE_ADR_FORMAT = (FETCH_STATE == FETCH_EXWORD_1 && NEXT_FETCH_STATE != FETCH_EXWORD_1) ? 1'b1 : 1'b0;

assign STORE_D16 = (FETCH_STATE == FETCH_DISPL && EW_ACK) ? 1'b1 : 1'b0;
assign STORE_D32_LO = (FETCH_STATE == FETCH_D_LO && EW_ACK) ? 1'b1 : 1'b0;
assign STORE_D32_HI = (FETCH_STATE == FETCH_D_HI && EW_ACK) ? 1'b1 : 1'b0;

assign STORE_DISPL = (OP == MOVEP && FETCH_STATE == START_OP && NEXT_FETCH_STATE != START_OP && BIW_0[7:6] < 2'b10) ? 1'b1 : // Memory to register.
                     (OP == MOVEP && FETCH_STATE == SWITCH_STATE) ? 1'b1 : 1'b0; // Register to memory.

assign STORE_OD_HI = (FETCH_STATE == FETCH_OD_HI && EW_ACK) ? 1'b1 : 1'b0;
assign STORE_OD_LO = (FETCH_STATE == FETCH_OD_LO && EW_ACK) ? 1'b1 : 1'b0;

assign STORE_ABS_HI = (FETCH_STATE == FETCH_ABS_HI && EW_ACK) ? 1'b1 : 1'b0;
assign STORE_ABS_LO = (FETCH_STATE == FETCH_ABS_LO && EW_ACK) ? 1'b1 : 1'b0;

assign STORE_IDATA_B2 = (FETCH_STATE == FETCH_IDATA_B2 && EW_ACK) ? 1'b1 : 1'b0;
assign STORE_IDATA_B1 = (FETCH_STATE == FETCH_IDATA_B1 && EW_ACK) ? 1'b1 : 1'b0;

// ====================================================================
// LOAD_OP1 / LOAD_OP2 / LOAD_OP3
// ====================================================================

assign LOAD_OP1 = (OP == BFINS && INIT_ENTRY) ? 1'b1 : // Load insertion pattern.
                  ((OP == CHK2 || OP == CMP2) && FETCH_STATE == FETCH_OPERAND && RD_RDY && DATA_VALID && !PHASE2) ? 1'b1 :
                  (OP == CMPM && FETCH_STATE == FETCH_OPERAND && RD_RDY && DATA_VALID && PHASE2) ? 1'b1 :
                  (OP == MOVE && BIW_0[8:6] == 3'b100 && BIW_0[5:3] == 3'b001 && BIW_0[11:9] == BIW_0[2:0] && INIT_ENTRY) ? 1'b1 : // Load early to write the undecremented Register for Ax, -(Ax).
                  (OP == MOVES && FETCH_STATE == START_OP && NEXT_FETCH_STATE != START_OP && ADR_MODE_I == 3'b100 && BIW_1[15] && BIW_1[11] && BIW_1[14:12] == BIW_0[2:0]) ? 1'b1 : // Load the addressing register before decrementing.
                  (OP == PEA && FETCH_STATE == SWITCH_STATE && PHASE2) ? 1'b1 : // Load early not to stack the decremented value.
                  (OP == BFINS || OP == CHK2 || OP == CMP2 || OP == CMPM || OP == PEA) ? 1'b0 :
                  (OP == MOVE && BIW_0[8:6] == 3'b100 && BIW_0[5:3] == 3'b001 && BIW_0[11:9] == BIW_0[2:0]) ? 1'b0 :
                  (OP == MOVES && ADR_MODE_I == 3'b100 && BIW_1[15] && BIW_1[11] && BIW_1[14:12] == BIW_0[2:0]) ? 1'b0 : // Do not load the decremented addressing register.
                  (OP == PEA && ADR_MODE_I == 3'b001 && BIW_0[2:0] == 3'b111) ? 1'b0 :
                  (ALU_INIT_I) ? 1'b1 : 1'b0;

assign LOAD_OP2 = ((OP == ABCD || OP == SBCD) && FETCH_STATE == FETCH_OPERAND && RD_RDY && DATA_VALID && !PHASE2) ? 1'b1 :
                  ((OP == ADDX || OP == SUBX) && FETCH_STATE == FETCH_OPERAND && RD_RDY && DATA_VALID && !PHASE2) ? 1'b1 :
                  ((OP == ABCD || OP == SBCD) && FETCH_STATE == INIT_EXEC_WB && !BIW_0[3]) ? 1'b1 : // Register direct.
                  ((OP == ADDX || OP == SUBX) && FETCH_STATE == INIT_EXEC_WB && !BIW_0[3]) ? 1'b1 : // Register direct.
                  ((OP == BFCHG || OP == BFCLR) && FETCH_STATE == FETCH_OPERAND && RD_RDY && DATA_VALID && !BF_HILOn) ? 1'b1 :
                  ((OP == BFINS || OP == BFSET) && FETCH_STATE == FETCH_OPERAND && RD_RDY && DATA_VALID && !BF_HILOn) ? 1'b1 :
                  ((OP == BFEXTS || OP == BFEXTU) && FETCH_STATE == FETCH_OPERAND && RD_RDY && DATA_VALID && !BF_HILOn) ? 1'b1 :
                  ((OP == BFFFO || OP == BFTST) && FETCH_STATE == FETCH_OPERAND && RD_RDY && DATA_VALID && !BF_HILOn) ? 1'b1 :
                  (OP == CMPM && FETCH_STATE == FETCH_OPERAND && RD_RDY && DATA_VALID && !PHASE2) ? 1'b1 :
                  (OP == LINK && BIW_0[2:0] == 3'b111 && FETCH_STATE == START_OP && NEXT_FETCH_STATE != START_OP) ? 1'b1 : // Load early not to stack the decremented address register.
                  (OP == ABCD || OP == SBCD || OP == ADDX || OP == SUBX || OP == CMPM) ? 1'b0 :
                  (OP == BFCHG || OP == BFCLR || OP == BFEXTS || OP == BFEXTU || OP == BFFFO || OP == BFINS || OP == BFSET || OP == BFTST) ? 1'b0 :
                  (OP == LINK && BIW_0[2:0] == 3'b111) ? 1'b0 :
                  (INIT_ENTRY) ? 1'b1 : 1'b0;

assign LOAD_OP3 = ((OP == BFCHG || OP == BFCLR || OP == BFINS || OP == BFSET) && BIW_0[5:3] == 3'b000 && INIT_ENTRY) ? 1'b1 :
                  ((OP == BFEXTS || OP == BFEXTU || OP == BFFFO || OP == BFTST) && BIW_0[5:3] == 3'b000 && INIT_ENTRY) ? 1'b1 :
                  ((OP == BFCHG || OP == BFCLR) && FETCH_STATE == FETCH_OPERAND && RD_RDY && DATA_VALID && BF_HILOn) ? 1'b1 :
                  ((OP == BFINS || OP == BSET) && FETCH_STATE == FETCH_OPERAND && RD_RDY && DATA_VALID && BF_HILOn) ? 1'b1 :
                  ((OP == BFEXTS || OP == BFEXTU) && FETCH_STATE == FETCH_OPERAND && RD_RDY && DATA_VALID && BF_HILOn) ? 1'b1 :
                  ((OP == BFFFO || OP == BFTST) && FETCH_STATE == FETCH_OPERAND && RD_RDY && DATA_VALID && BF_HILOn) ? 1'b1 :
                  (OP == CAS2 && INIT_ENTRY && !PHASE2) ? 1'b1 : // Memory operand 2.
                  ((OP == CHK2 || OP == CMP2) && FETCH_STATE == FETCH_OPERAND && RD_RDY && DATA_VALID && PHASE2) ? 1'b1 :
                  ((OP == DIVS || OP == DIVU) && OP_SIZE_I == LONG && BIW_1[10] && INIT_ENTRY) ? 1'b1 : 1'b0; // 64 bit operand.

// ====================================================================
// SR write / HILOn
// ====================================================================

assign SR_WR = SR_WR_I;
assign SR_WR_I = ((OP_WB_I == ANDI_TO_SR || OP_WB_I == EORI_TO_SR || OP_WB_I == ORI_TO_SR) && EXEC_WB_STATE == WRITEBACK) ? 1'b1 :
                 ((OP_WB_I == MOVE_TO_CCR || OP_WB_I == MOVE_TO_SR) && EXEC_WB_STATE == WRITEBACK) ? 1'b1 :
                 (OP_WB_I == STOP && EXEC_WB_STATE == WRITEBACK) ? 1'b1 : 1'b0;

assign HILOn = (OP_WB_I == CAS2 && FETCH_STATE == FETCH_OPERAND && !PHASE2) ? 1'b1 :
               (OP_WB_I == CAS2 && EXEC_WB_STATE == WRITEBACK && !PHASE2) ? 1'b1 :
               (OP == CAS2) ? 1'b0 : BF_HILOn; // Select destinations.

// ====================================================================
// Addressing mode
// ====================================================================

assign ADR_MODE = ADR_MODE_I;
assign ADR_MODE_I = (OP == BSR || OP == CAS2 || OP == LINK || OP == UNLK) ? 3'b010 : // (An), (Dn).
                    (OP == RTD || OP == RTR || OP == RTS) ? 3'b010 : // (An).
                    (OP == CMPM) ? 3'b011 : // (An)+
                    (OP == ABCD || OP == SBCD) ? 3'b100 : // -(An).
                    (OP == ADDX || OP == SUBX) ? 3'b100 : // -(An).
                    (OP == PACK || OP == UNPK) ? 3'b100 : // -(An).
                    (OP == MOVEP) ? 3'b101 : // (d16, An).
                    // The following two conditions change the address mode right in the end of the fetch phase.
                    ((OP == JSR || OP == PEA) && FETCH_STATE == INIT_EXEC_WB && !ALU_BSY) ? 3'b010 : // (A7).
                    (OP == MOVE && FETCH_STATE == INIT_EXEC_WB && !ALU_BSY) ? BIW_0[8:6] :
                    (OP == MOVE && PHASE2) ? BIW_0[8:6] :
                    BIW_0[5:3];

// This is the selector for the address mode "111".
assign AMODE_SEL = (OP == MOVE && (NEXT_FETCH_STATE == INIT_EXEC_WB || FETCH_STATE == INIT_EXEC_WB)) ? BIW_0[11:9] :
                   (OP == MOVE && PHASE2) ? BIW_0[11:9] : BIW_0[2:0];

// ====================================================================
// OP_SIZE
// ====================================================================

assign OP_SIZE = OP_SIZE_I;
assign OP_SIZE_I = (FETCH_STATE == FETCH_MEMADR && !RD_RDY) ? LONG : // (RD_RDY: release early to provide correct OP_SIZE.
                   ((OP == ADDA || OP == CMPA || OP == SUBA) && BIW_0[8:7] == 2'b11) ? LONG :
                   ((OP == BCHG || OP == BCLR || OP == BTST || OP == BSET) && BIW_0[5:3] == 3'b000) ? LONG :
                   ((OP == BFCHG || OP == BFCLR || OP == BFINS || OP == BFSET) && BIW_0[5:3] == 3'b000) ? LONG :
                   ((OP == BFEXTS || OP == BFEXTU || OP == BFFFO || OP == BFTST) && BIW_0[5:3] == 3'b000) ? LONG :
                   ((OP == BFCHG || OP == BFCLR || OP == BFINS || OP == BFSET) && BF_BYTES > 2) ? LONG :
                   ((OP == BFEXTS || OP == BFEXTU || OP == BFFFO || OP == BFTST) && BF_BYTES > 2) ? LONG :
                   (OP == EXT && BIW_0[8:6] == 3'b011) ? LONG :
                   (OP == BSR || OP == EXG || OP == EXTB || OP == JSR || OP == LEA || OP == LINK || OP == PEA || OP == SWAP || OP == UNLK) ? LONG :
                   ((OP == CAS || OP == CAS2) && BIW_0[10:9] == 2'b11) ? LONG :
                   (OP == CHK && BIW_0[8:7] == 2'b10) ? LONG :
                   ((OP == CHK2 || OP == CMP2) && BIW_0[10:9] == 2'b10) ? LONG :
                   (OP == CMPM && BIW_0[7:6] == 2'b10) ? LONG :
                   ((OP == MOVE || OP == MOVEA) && BIW_0[13:12] == 2'b10) ? LONG :
                   (OP == MOVEC || OP == MOVEQ || OP == MOVE_USP || OP == RTD || OP == RTS) ? LONG :
                   (OP == MOVEM && BIW_0[6]) ? LONG :
                   (OP == MOVEP && FETCH_STATE == INIT_EXEC_WB && BIW_0[7:6] < 2'b10) ? LONG : // Writeback to registers is long (see top level multiplexer).
                   ((OP == DIVS || OP == DIVU || OP == MULS || OP == MULU) && !BIW_0[7]) ? LONG :
                   (OP == RTR && PHASE2) ? LONG : // Read PC.
                   ((OP == ADDA || OP == CMPA || OP == SUBA) && BIW_0[8:7] == 2'b01) ? WORD :
                   ((OP == ASL || OP == ASR) && BIW_0[7:6] == 2'b11) ? WORD : // Memory shifts.
                   (OP == ANDI_TO_SR || OP == EORI_TO_SR || OP == ORI_TO_SR) ? WORD :
                   ((OP == BFCHG || OP == BFCLR || OP == BFINS || OP == BFSET) && BF_BYTES == 2) ? WORD :
                   ((OP == BFEXTS || OP == BFEXTU || OP == BFFFO || OP == BFTST) && BF_BYTES == 2) ? WORD :
                   (OP == BKPT && FETCH_STATE == FETCH_OPERAND && DATA_RD_I) ? WORD :
                   ((OP == CAS || OP == CAS2) && BIW_0[10:9] == 2'b10) ? WORD :
                   (OP == CHK && BIW_0[8:7] == 2'b11) ? WORD :
                   ((OP == CHK2 || OP == CMP2) && BIW_0[10:9] == 2'b01) ? WORD :
                   (OP == CMPM && BIW_0[7:6] == 2'b01) ? WORD :
                   (OP == DBcc || OP == EXT) ? WORD :
                   ((OP == LSL || OP == LSR) && BIW_0[7:6] == 2'b11) ? WORD : // Memory shifts.
                   ((OP == MOVE || OP == MOVEA) && BIW_0[13:12] == 2'b11) ? WORD :
                   (OP == MOVE_FROM_CCR || OP == MOVE_TO_CCR) ? WORD :
                   (OP == MOVE_FROM_SR || OP == MOVE_TO_SR) ? WORD :
                   (OP == MOVEM || OP == RTR) ? WORD :
                   (OP == DIVS || OP == DIVU || OP == MULS || OP == MULU) ? WORD :
                   (OP == PACK && (NEXT_FETCH_STATE == FETCH_OPERAND || FETCH_STATE == FETCH_OPERAND) && !INIT_ENTRY) ? WORD : // Read data is word wide.
                   ((OP == ROTL || OP == ROTR) && BIW_0[7:6] == 2'b11) ? WORD : // Memory shifts.
                   ((OP == ROXL || OP == ROXR) && BIW_0[7:6] == 2'b11) ? WORD : // Memory shifts.
                   (OP == UNPK && (INIT_ENTRY || FETCH_STATE == INIT_EXEC_WB)) ? WORD : // Writeback data is a word.
                   (OP == ABCD || OP == NBCD || OP == SBCD) ? BYTE :
                   (OP == ANDI_TO_CCR || OP == EORI_TO_CCR || OP == ORI_TO_CCR) ? BYTE :
                   (OP == BCHG || OP == BCLR || OP == BTST || OP == BSET) ? BYTE :
                   (OP == BFCHG || OP == BFCLR || OP == BFINS || OP == BFSET) ? BYTE :
                   (OP == BFEXTS || OP == BFEXTU || OP == BFFFO || OP == BFTST) ? BYTE :
                   (OP == CAS && BIW_0[10:9] == 2'b01) ? BYTE :
                   ((OP == CHK2 || OP == CMP2) && BIW_0[10:9] == 2'b00) ? BYTE :
                   (OP == CMPM && BIW_0[7:6] == 2'b00) ? BYTE :
                   (OP == MOVE || OP == MOVEP) ? BYTE :
                   (OP == PACK) ? BYTE : // Writeback data is a byte.
                   (OP == Scc || OP == TAS) ? BYTE :
                   (OP == UNPK) ? BYTE : // Read data is byte wide.
                   (BIW_0[7:6] == 2'b00) ? BYTE :
                   (BIW_0[7:6] == 2'b01) ? WORD : LONG;

// ====================================================================
// BKPT / Traps / BERR
// ====================================================================

assign BKPT_CYCLE = (OP == BKPT && FETCH_STATE == FETCH_OPERAND && DATA_RD_I) ? 1'b1 : 1'b0;
assign BKPT_INSERT = (OP == BKPT && FETCH_STATE == FETCH_OPERAND && RD_RDY && DATA_VALID) ? 1'b1 : 1'b0;

// All traps must be modeled as strobes.
assign TRAP_ILLEGAL = (OP == BKPT && FETCH_STATE == FETCH_OPERAND && RD_RDY && !DATA_VALID) ? 1'b1 : 1'b0;

assign TRAP_cc = (OP == TRAPcc && ALU_COND && FETCH_STATE == SLEEP && NEXT_FETCH_STATE == START_OP) ? 1'b1 : 1'b0;
assign TRAP_V = (OP == TRAPV && ALU_COND && FETCH_STATE == SLEEP && NEXT_FETCH_STATE == START_OP) ? 1'b1 : 1'b0;

assign BERR = (FETCH_STATE == START_OP && EXEC_WB_STATE == IDLE) ? 1'b0 : // Disable when controller is not active.
              (OP == BKPT) ? 1'b0 : // No bus error during breakpoint cycle.
              (DATA_RDY && !DATA_VALID) ? 1'b1 :
              (OPD_ACK && !OW_VALID) ? 1'b1 :
              (EW_ACK && !OW_VALID) ? 1'b1 : 1'b0;

// ====================================================================
// System register access (SFC, DFC, VBR, ISP, MSP, USP)
// ====================================================================

assign SFC_RD = (OP == MOVEC && !BIW_0[0] && BIW_1[11:0] == 12'h000) ? 1'b1 : 1'b0;
assign SFC_WR = (OP_WB_I == MOVEC && BIW_0_WB[0] && BIW_1[11:0] == 12'h000 && EXEC_WB_STATE == WRITEBACK) ? 1'b1 : 1'b0;

assign DFC_RD = (OP == MOVEC && !BIW_0[0] && BIW_1[11:0] == 12'h001) ? 1'b1 : 1'b0;
assign DFC_WR = (OP_WB_I == MOVEC && BIW_0_WB[0] && BIW_1_WB[11:0] == 12'h001 && EXEC_WB_STATE == WRITEBACK) ? 1'b1 : 1'b0;

assign VBR_RD = (OP == MOVEC && !BIW_0[0] && BIW_1[11:0] == 12'h801) ? 1'b1 : 1'b0;
assign VBR_WR = (OP_WB_I == MOVEC && BIW_0_WB[0] && BIW_1_WB[11:0] == 12'h801 && EXEC_WB_STATE == WRITEBACK) ? 1'b1 : 1'b0;

assign ISP_RD = (OP == MOVEC && !BIW_0[0] && BIW_1[11:0] == 12'h804) ? 1'b1 : 1'b0;
assign ISP_WR = (OP_WB_I == MOVEC && BIW_0_WB[0] && BIW_1_WB[11:0] == 12'h804 && EXEC_WB_STATE == WRITEBACK) ? 1'b1 : 1'b0;

assign MSP_RD = (OP == MOVEC && !BIW_0[0] && BIW_1[11:0] == 12'h803) ? 1'b1 : 1'b0;
assign MSP_WR = (OP_WB_I == MOVEC && BIW_0_WB[0] && BIW_1_WB[11:0] == 12'h803 && EXEC_WB_STATE == WRITEBACK) ? 1'b1 : 1'b0;

assign USP_RD = (OP == MOVE_USP && BIW_0[3]) ? 1'b1 :
                (OP == MOVEC && !BIW_0[0] && BIW_1[11:0] == 12'h800) ? 1'b1 : 1'b0;
assign USP_WR = (OP_WB_I == MOVE_USP && EXEC_WB_STATE == WRITEBACK && !BIW_0_WB[3]) ? 1'b1 :
                (OP_WB_I == MOVEC && BIW_0_WB[0] && BIW_1_WB[11:0] == 12'h800 && EXEC_WB_STATE == WRITEBACK) ? 1'b1 : 1'b0;

// ====================================================================
// PC / pipe control
// ====================================================================

assign PC_ADD_DISPL = PC_ADD_DISPL_I;
assign PC_ADD_DISPL_I = (OP == Bcc && FETCH_STATE != START_OP && NEXT_FETCH_STATE == START_OP && ALU_COND) ? 1'b1 :
                        ((OP == BRA || OP == BSR) && FETCH_STATE != START_OP && NEXT_FETCH_STATE == START_OP) ? 1'b1 :
                        (OP == DBcc && FETCH_STATE != START_OP && NEXT_FETCH_STATE == START_OP && !ALU_COND && !DBcc_COND) ? 1'b1 : 1'b0;

assign PC_LOAD = PC_LOAD_I;
assign PC_LOAD_I = ((OP == JMP || OP == JSR) && FETCH_STATE != START_OP && NEXT_FETCH_STATE == START_OP) ? 1'b1 :
                   ((OP == RTD || OP == RTR || OP == RTS) && FETCH_STATE != START_OP && NEXT_FETCH_STATE == START_OP) ? 1'b1 : 1'b0;

// The pipe is flushed for the system control instructions.
assign IPIPE_FLUSH = IPIPE_FLUSH_I;
assign IPIPE_FLUSH_I = ((OP == BRA || OP == BSR) && FETCH_STATE != START_OP && NEXT_FETCH_STATE == START_OP) ? 1'b1 :
                       (OP == Bcc && FETCH_STATE != START_OP && NEXT_FETCH_STATE == START_OP && ALU_COND) ? 1'b1 :
                       (OP == DBcc && !LOOP_BSY && FETCH_STATE != START_OP && NEXT_FETCH_STATE == START_OP && !ALU_COND && !DBcc_COND) ? 1'b1 :
                       (OP == DBcc && LOOP_EXIT_I && (ALU_COND || DBcc_COND)) ? 1'b1 : // Flush the pipe after a finished loop.
                       ((OP == ANDI_TO_SR || OP == EORI_TO_SR || OP == MOVE_TO_SR || OP == ORI_TO_SR) && FETCH_STATE == SLEEP && NEXT_FETCH_STATE == START_OP) ? 1'b1 :
                       ((OP == JMP || OP == JSR) && FETCH_STATE != START_OP && NEXT_FETCH_STATE == START_OP) ? 1'b1 :
                       (OP == MOVEC && BIW_0[0] && FETCH_STATE != START_OP && NEXT_FETCH_STATE == START_OP) ? 1'b1 : // Writing control registers.
                       ((OP == RTD || OP == RTR || OP == RTS) && FETCH_STATE != START_OP && NEXT_FETCH_STATE == START_OP) ? 1'b1 : 1'b0;

assign SP_ADD_DISPL = (OP == LINK && FETCH_STATE == INIT_EXEC_WB && !ALU_BSY) ? 1'b1 :
                      (OP == RTD && FETCH_STATE == FETCH_OPERAND && RD_RDY && DATA_VALID) ? 1'b1 : 1'b0;

// ====================================================================
// ALU control
// ====================================================================

assign ALU_TRIG = (ALU_BSY || FETCH_STATE != INIT_EXEC_WB) ? 1'b0 :
                  ((OP == CHK2 || OP == CMP2 || OP == CMPM) && PHASE2) ? 1'b0 :
                  (OP == MOVE && PHASE2) ? 1'b0 : // no ALU required after second portion of address calculation.
                  (OP == MOVEM && !MOVEM_COND) ? 1'b0 :
                  (OP == MOVEM && BIW_0[10] && !MOVEM_FIRST_RD) ? 1'b0 : // Do not load before the first read access.
                  (OP == RTR && PHASE2) ? 1'b0 : 1'b1; // RTR: not when PC is loaded.

// This is the signal loading the operands into the ALU registers:
assign ALU_INIT = ALU_INIT_I;
always_comb begin
    case (OP)
        ABCD, ADD, ADDA, ADDI, ADDQ, ADDX, AND_B, ANDI, ANDI_TO_CCR, ANDI_TO_SR, ASL, ASR, Bcc, BCHG,
        BCLR, BFCHG, BFCLR, BFEXTS, BFEXTU, BFFFO, BFINS, BFSET, BFTST, BSET, BSR, BTST, CAS, CAS2,
        CHK, CHK2, CLR, CMP, CMPA, CMPI, CMPM, CMP2, DBcc, DIVS, DIVU, EOR, EORI, EORI_TO_CCR, EORI_TO_SR,
        EXG, EXT, EXTB, JSR, LEA, LINK, LSL, LSR, MOVE, MOVEA, MOVE_FROM_CCR, MOVE_TO_CCR, MOVE_FROM_SR,
        MOVE_TO_SR, MOVE_USP, MOVEC, MOVEM, MOVEQ, MOVEP, MOVES, MULS, MULU, NBCD, NEG, NEGX, NOT_B, OR_B,
        ORI, ORI_TO_CCR, ORI_TO_SR, PACK, PEA, ROTL, ROTR, ROXL, ROXR, RTR, SBCD, Scc, SUB, SUBA, SUBI,
        SUBQ, SUBX, SWAP, STOP, TAS, TRAPV, TRAPcc, TST, UNLK, UNPK: ALU_INIT_I = ALU_TRIG;
        default: ALU_INIT_I = 1'b0;
    endcase
end

// ====================================================================
// Condition code update
// ====================================================================

assign UPDT_CC = ((OP_WB_I == ADDQ || OP_WB_I == SUBQ) && BIW_0_WB[5:3] == 3'b001) ? 1'b0 : // No update for ADDQ and SUBQ when destination is an address register.
                 (OP == CAS2 && FETCH_STATE == INIT_EXEC_WB && EXEC_WB_STATE == WRITEBACK) ? 1'b0 : // First 'Z' flag was zero, do not update the second access.
                 (OP == CAS2 && FETCH_STATE == INIT_EXEC_WB && EXEC_WB_STATE == WRITE_DEST && PHASE2) ? 1'b0 : ALU_REQ; // Suppress third update.

always_comb begin
    case (OP_WB_I)
        ABCD, ADD, ADDI, ADDQ, ADDX, AND_B, ANDI, ANDI_TO_CCR, ASL, ASR, BCHG, BCLR,
        BFCHG, BFCLR, BFEXTS, BFEXTU, BFFFO, BFINS, BFSET, BFTST, BSET, BTST,
        CAS, CAS2, CHK, CHK2, CLR, CMP, CMPA, CMPI, CMPM, CMP2, DIVS, DIVU,
        EOR, EORI, EORI_TO_CCR, EXT, EXTB, LSL, LSR, MOVE, MOVEQ, MULS, MULU, NBCD,
        NEG, NEGX, NOT_B, OR_B, ORI, ORI_TO_CCR, ROTL, ROTR, ROXL, ROXR, RTR, SBCD,
        SUB, SUBI, SUBQ, SUBX, SWAP, TAS, TST: CC_UPDT = UPDT_CC;
        default: CC_UPDT = 1'b0;
    endcase
end

// ====================================================================
// Mark-used signals
// ====================================================================

assign ADR_MARK_USED_I = (OP == MOVE && FETCH_STATE == INIT_EXEC_WB && PHASE2) ? 1'b1 : // Destination address calculation done.
                         (FETCH_STATE != INIT_EXEC_WB || ALU_BSY) ? 1'b0 : // Deactivate except in the end of INIT_EXEC_WB.
                         (OP == BFCHG || OP == BFCLR || OP == BFINS || OP == BFSET || OP == BSR || OP == JSR || OP == LINK || OP == PEA) ? 1'b1 :
                         ((OP == ADDI || OP == ANDI || OP == EOR || OP == EORI || OP == ORI || OP == SUBI) && BIW_0[5:3] != 3'b000) ? 1'b1 :
                         ((OP == ABCD || OP == SBCD || OP == ADDX || OP == SUBX) && BIW_0[3]) ? 1'b1 :
                         ((OP == ADD || OP == AND_B || OP == OR_B || OP == SUB) && BIW_0[8]) ? 1'b1 :
                         ((OP == ADDQ || OP == BCHG || OP == BCLR || OP == BSET || OP == CLR || OP == MOVE_FROM_CCR || OP == MOVE_FROM_SR) && BIW_0[5:3] > 3'b001) ? 1'b1 :
                         ((OP == NBCD || OP == NEG || OP == NEGX || OP == NOT_B || OP == Scc || OP == SUBQ || OP == TAS) && BIW_0[5:3] > 3'b001) ? 1'b1 :
                         ((OP == ASL || OP == ASR || OP == LSL || OP == LSR) && BIW_0[7:6] == 2'b11) ? 1'b1 : // Memory shifts.
                         ((OP == ROTL || OP == ROTR || OP == ROXL || OP == ROXR) && BIW_0[7:6] == 2'b11) ? 1'b1 : // Memory shifts.
                         (OP == MOVE && !PHASE2 && BIW_0[8:6] != 3'b000 && BIW_0[8:6] < 3'b101) ? 1'b1 : // We do not need destination address calculation access.
                         (OP == MOVEM && !BIW_0[10] && MOVEM_COND) ? 1'b1 : // Register to memory.
                         (OP == MOVEP && BIW_0[7:6] > 2'b01) ? 1'b1 : // Register to Memory.
                         (OP == MOVES && BIW_1[11]) ? 1'b1 : // Register to memory.
                         ((OP == PACK || OP == UNPK) && BIW_0[3]) ? 1'b1 : 1'b0;

assign ADR_MARK_USED = ((OP_WB_I == BFCHG || OP_WB_I == BFCLR) && EXEC_WB_STATE == WRITE_DEST && WR_RDY && BF_BYTES == 5) ? 1'b1 :
                       ((OP_WB_I == BFINS || OP_WB_I == BFSET) && EXEC_WB_STATE == WRITE_DEST && WR_RDY && BF_BYTES == 5) ? 1'b1 :
                       (OP_WB_I == CAS && EXEC_WB_STATE == EXECUTE && ALU_COND) ? 1'b1 :
                       (OP_WB_I == CAS2 && EXEC_WB_STATE == ADR_PIPELINE && ALU_COND) ? 1'b1 :
                       (OP_WB_I == CAS2 && EXEC_WB_STATE == WRITE_DEST && WR_RDY && !PHASE2) ? 1'b1 : ADR_MARK_USED_I;

assign AR_MARK_USED = (OP == UNLK && FETCH_STATE != SWITCH_STATE && NEXT_FETCH_STATE == SWITCH_STATE) ? 1'b1 : // This is for An to SP.
                      (OP == LINK && FETCH_STATE == SWITCH_STATE && NEXT_FETCH_STATE != SWITCH_STATE) ? 1'b1 : // This is for SP to An.
                      (FETCH_STATE != INIT_EXEC_WB || NEXT_FETCH_STATE == INIT_EXEC_WB) ? 1'b0 : // Deactivate except in the end of INIT_EXEC_WB.
                      (OP == ADDA || OP == SUBA) ? 1'b1 :
                      ((OP == ADDQ || OP == SUBQ) && BIW_0[5:3] == 3'b001) ? 1'b1 :
                      (OP == EXG && BIW_0[7:3] != 5'b01000) ? 1'b1 :
                      (OP == LEA) ? 1'b1 :
                      (OP == MOVE_USP) ? 1'b1 :
                      (OP == MOVEM && BIW_0[10] && MOVEM_ADn_I && MOVEM_COND) ? 1'b1 : // Memory to register.
                      (OP == MOVEA) ? 1'b1 :
                      (OP == MOVEC && !BIW_0[0] && BIW_1[15]) ? 1'b1 : // Destination is Ax.
                      (OP == MOVES && BIW_1[15] && !BIW_1[11]) ? 1'b1 :
                      (OP == UNLK) ? 1'b1 : 1'b0;

assign DR_MARK_USED = (OP_WB_I == CAS && EXEC_WB_STATE == EXECUTE && !ALU_COND) ? 1'b1 :
                      (OP_WB_I == CAS2 && EXEC_WB_STATE == EXECUTE && !ALU_COND) ? 1'b1 :
                      (OP_WB_I == CAS2 && EXEC_WB_STATE == WRITEBACK && !PHASE2) ? 1'b1 :
                      (FETCH_STATE != INIT_EXEC_WB || NEXT_FETCH_STATE == INIT_EXEC_WB) ? 1'b0 : // Deactivate except in the end of INIT_EXEC_WB.
                      ((OP == ABCD || OP == SBCD) && !BIW_0[3]) ? 1'b1 :
                      ((OP == ADDX || OP == SUBX) && !BIW_0[3]) ? 1'b1 :
                      ((OP == ADDQ || OP == SUBQ) && BIW_0[5:3] == 3'b000) ? 1'b1 :
                      ((OP == ADD || OP == SUB) && !BIW_0[8]) ? 1'b1 :
                      ((OP == ADDI || OP == ANDI || OP == EOR || OP == EORI || OP == ORI || OP == SUBI) && BIW_0[5:3] == 3'b000) ? 1'b1 :
                      ((OP == AND_B || OP == OR_B) && !BIW_0[8]) ? 1'b1 :
                      ((OP == ASL || OP == ASR) && BIW_0[7:6] != 2'b11) ? 1'b1 :
                      ((OP == LSL || OP == LSR) && BIW_0[7:6] != 2'b11) ? 1'b1 :
                      ((OP == ROTL || OP == ROTR) && BIW_0[7:6] != 2'b11) ? 1'b1 :
                      ((OP == ROXL || OP == ROXR) && BIW_0[7:6] != 2'b11) ? 1'b1 :
                      ((OP == BCHG || OP == BCLR || OP == BSET) && BIW_0[5:3] == 3'b000) ? 1'b1 :
                      ((OP == BFCHG || OP == BFCLR || OP == BFINS || OP == BFSET) && BIW_0[5:3] == 3'b000) ? 1'b1 :
                      (OP == BFEXTS || OP == BFEXTU || OP == BFFFO) ? 1'b1 :
                      ((OP == CLR || OP == TAS || OP == Scc) && BIW_0[5:3] == 3'b000) ? 1'b1 :
                      (OP == DBcc || OP == DIVS || OP == DIVU || OP == MULS || OP == MULU) ? 1'b1 :
                      (OP == EXG && BIW_0[7:3] != 5'b01001) ? 1'b1 :
                      (OP == EXT || OP == EXTB || OP == SWAP) ? 1'b1 :
                      (OP == MOVE && BIW_0[8:6] == 3'b000) ? 1'b1 :
                      ((OP == MOVE_FROM_CCR || OP == MOVE_FROM_SR) && BIW_0[5:3] == 3'b000) ? 1'b1 :
                      (OP == MOVEM && BIW_0[10] && !MOVEM_ADn_I && MOVEM_COND) ? 1'b1 : // Memory to register.
                      (OP == MOVEP && BIW_0[7:6] < 2'b10) ? 1'b1 : // Memory to register.
                      (OP == MOVEC && !BIW_0[0] && !BIW_1[15]) ? 1'b1 : // Destination is Dx.
                      (OP == MOVEQ) ? 1'b1 :
                      (OP == MOVES && !BIW_1[15] && !BIW_1[11]) ? 1'b1 :
                      ((OP == NBCD || OP == NEG || OP == NEGX || OP == NOT_B) && BIW_0[5:3] == 3'b000) ? 1'b1 :
                      ((OP == PACK || OP == UNPK) && !BIW_0[3]) ? 1'b1 : 1'b0;

assign UNMARK = (EXEC_WB_STATE != IDLE && NEXT_EXEC_WB_STATE == IDLE) ? 1'b1 : 1'b0;

// These signals indicates, that two registers are prepared to be written.
assign USE_APAIR = (OP == EXG && BIW_0[7:3] == 5'b01001) ? 1'b1 : 1'b0;
assign USE_DPAIR = (OP == EXG && BIW_0[7:3] == 5'b01000) ? 1'b1 :
                   ((OP == DIVS || OP == DIVU) && OP_SIZE_I == LONG && BIW_1[14:12] != BIW_1[2:0]) ? 1'b1 :
                   ((OP == MULS || OP == MULU) && OP_SIZE_I == LONG && BIW_1[10] && BIW_1[14:12] != BIW_1[2:0]) ? 1'b1 : 1'b0;

// ====================================================================
// Loop control
// ====================================================================

assign LOOP_EXIT = LOOP_EXIT_I;
assign LOOP_EXIT_I = (OP != DBcc && LOOP_BSY && FETCH_STATE != START_OP && NEXT_FETCH_STATE == START_OP && EXH_REQ) ? 1'b1 : // Exception! break the loop.
                     (OP == DBcc && LOOP_BSY && FETCH_STATE == SLEEP && NEXT_FETCH_STATE == START_OP && (ALU_COND || DBcc_COND)) ? 1'b1 : 1'b0; // 68010 loop mechanism.

// ====================================================================
// Misc
// ====================================================================

assign RESET_STRB = (OP == OP_RESET && INIT_ENTRY) ? 1'b1 : 1'b0;

assign EX_TRACE = (OP == ILLEGAL || OP == UNIMPLEMENTED) ? 1'b0 :
                  (TRACE_MODE == 2'b10 && OPD_ACK && FETCH_STATE == START_OP && OP == TRAP) ? 1'b1 :
                  (TRACE_MODE == 2'b10 && FETCH_STATE != START_OP && NEXT_FETCH_STATE == START_OP) ? 1'b1 :
                  (TRACE_MODE == 2'b01 && FETCH_STATE == INIT_EXEC_WB && !ALU_BSY && OP == TRAP) ? 1'b1 :
                  (TRACE_MODE == 2'b01 && FETCH_STATE == INIT_EXEC_WB && !ALU_BSY && OP == TRAPcc && ALU_COND) ? 1'b1 :
                  (TRACE_MODE == 2'b01 && FETCH_STATE == INIT_EXEC_WB && !ALU_BSY && OP == TRAPV && VBIT) ? 1'b1 :
                  (TRACE_MODE == 2'b01 && SR_WR_I) ? 1'b1 : // Status register manipulations.
                  (TRACE_MODE == 2'b01 && (PC_ADD_DISPL_I | PC_LOAD_I)); // All branches and jumps.

endmodule
