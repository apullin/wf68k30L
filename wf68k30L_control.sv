//--------------------------------------------------------------------//
//                                                                    //
// WF68K30L IP Core: this is the main controller to handle all        //
// integer instructions.                                              //
//                                                                    //
// Description:                                                       //
// This controller handles all integer instructions and provides      //
// all required system control signals. The instructions are          //
// requested from the instruction prefetch unit in the opcode         //
// decoder unit. The data is mostly written to the ALU and after-     //
// wards from the ALU to the writeback logic. This pipelined          //
// structure requires a correct management of data in use. Any        //
// address or data registers or memory addresses which are in use     //
// are marked by a flag not to be used befor written back. Any        //
// time data or address reegisters or the effective address are       //
// in the writeback pipe, the respective use flag is evaluated.       //
// Instructions like MOVE read unused source and the destination      //
// is written, when the ALU and writeback controller 'free'.          //
// Instructions like ADD, SUB etc. read unused source and desti-      //
// nations and write back the destination, when the operands are      //
// not used any more and the ALU and writeback controller are not     //
// in use by another operation.                                       //
// The main controller is the second pipeline stage of the CPU.       //
// The pipelining structure also requires some special treatment      //
// for the system control instructions as described as follows:       //
//--------------------------------------------------------------------//
// System Control Instructions:                                       //
// There are several instructions which require the instruction       //
// pipe to be flushed as described in the following. For further      //
// information refer to the 68Kxx data sheets.                        //
//                                                                    //
// 1. TRAP generating:                                                //
//     The following instructions result in a pipe flush when the     //
//     exception handler takes control over the system:               //
//     BKPT, CHK, ILLEGAL, TRAP, TRAPV                                //
//     There are some other indirect conditions flushing the pipe     //
//     such as the STOP which is invoked by an external interrupt.    //
//                                                                    //
// 2. Priviledge violations:                                          //
//     Some instructions may result in a priviledge violation         //
//     when executed in user space. The result will be a              //
//     priviledge violation trap and the ipipe is flushed when        //
//     the exception handler takes over. The instructions are:        //
//     ANDI_TO_SR, EORI_TO_SR, ORI_TO_SR, MOVEC, MOVES, MOVE_USP,   //
//     MOVE_FROM_SR, MOVE_TO_SR, RESET, RTE and STOP                  //
//                                                                    //
// 3. Branches and Jumps:                                             //
//    In case of branches and jumps and the respective return         //
//    operations it is required to flush the instruction pipe.        //
//    If PC value changes due to any branch or jump, it is neces-     //
//    sary to flush the instruction pipe to invalidate already        //
//    loaded 'old' instructions. This affects:                        //
//    BRA, BSR, Bcc, DBcc, JMP, JSR and the returns:                  //
//    RTD, RTR, RTS, RTE and also STOP.                               //
//                                                                    //
//--------------------------------------------------------------------//
// Data hazards:                                                      //
//    To avoid malfunction by using old data several things have      //
//    to be taken into account:                                       //
//    1. Operations manipulating the system registers must wait       //
//      in the end of the operation until the ALU has updated         //
//      the condition codes. These operations are ANDI_TO_SR,         //
//      EORI_TO_SR, ORI_TO_SR, MOVE_TO_SR and MOVEC.                  //
//    2. Operations using the staus register must not start until     //
//      the ALU has updated the condition codes. These operations     //
//      are MOVE_FROM_CCR, MOVE_FROM_SR and MOVEC.                    //
//    3. Operations using the stack pointer must not start until      //
//      the stack pointer is updated by the previous operation.       //
//      Operations using the stack pointer are RTD, RTR, RTS,         //
//      MOVEC, MOVE_USP and UNLK.                                     //
//    4. Operations manipulating the stack pointer without using      //
//      the ALU must not start until the stack is written by the      //
//      previous operation. Stack pointer manipulating operations     //
//      are BSR, JSR, LINK and PEA.                                   //
//--------------------------------------------------------------------//
//                                                                    //
// Remarks:                                                           //
//                                                                    //
// Author(s):                                                         //
// - Wolfgang Foerster, wf@experiment-s.de; wf@inventronik.de         //
//                                                                    //
//--------------------------------------------------------------------//
//                                                                    //
// Copyright (c) 2014-2019 Wolfgang Foerster Inventronik GmbH.        //
//                                                                    //
// This documentation describes Open Hardware and is licensed         //
// under the CERN OHL v. 1.2. You may redistribute and modify         //
// this documentation under the terms of the CERN OHL v.1.2.          //
// (http://ohwr.org/cernohl). This documentation is distributed       //
// WITHOUT ANY EXPRESS OR IMPLIED WARRANTY, INCLUDING OF              //
// MERCHANTABILITY, SATISFACTORY QUALITY AND FITNESS FOR A            //
// PARTICULAR PURPOSE. Please see the CERN OHL v.1.2 for              //
// applicable conditions                                              //
//                                                                    //
//--------------------------------------------------------------------//
//
// Revision History
//
// Revision 2K14B 20141201 WF
//   Initial Release.
// Revision 2K16A 20160620 WF
//   Fixed a bug in the MOVEM operation.
// Revision 2K18A (unreleased) WF
//   Fixed a bug in MOVE An,-(Ay). Thanks to Gary Bingham for the support.
//   Fixed wrong PEA behaviour.
//   Fixed the displacement for LINK.
//   Fixed AR_MARK_USED in LINK.
//   Fixed the operation size for MOVEQ.
//   ADDQ, SUBQ Fix: address registers are always written long.
//   ADDI, ANDI, EORI, ORI, SUBI: address is not marked used if destination is Dn.
//   ADDI, ANDI, EORI, ORI, SUBI: data register is marked used if destination is Dn.
//   EXG: rearranged logic to meet the new top level multiplexers.
//   LINK, UNLK: wait in START_OP until the ALU is ready (avoids possible data hazards).
//   LINK, UNLK: fixed the write back operation size.
//   MOVEM: Fixed predecrement mode for consecutive MOVEM -(An).
//   MOVEP: MOVEP_PNTR is now correct for consecutive MOVEP.
//   MOVEP: avoid structural hazard in SWITCH_STATE by waiting for ALU.
//   EOR: fixed a bug in the writeback mechanism.
//   BSR, JSR: EXEC_WB state machine waits now for ALU_INIT. Avoid structural / data hazard.
//   The instruction pipe is not flushed for ANDI_TO_CCR, EORI_TO_CCR, ORI_TO_CCR.
//   The instruction pipe is not flushed for MOVE_FROM_CCR, MOVE_TO_CCR, MOVE_FROM_SR, MOVE_USP, MOVEC.
//   Modifications in the FETCH state machine to avoid several data hazards for MOVEM, MOVE_FROM_CCR, MOVE_FROM_SR.
//   Modifications in the FETCH state machine to avoid several data hazards for ANDI_TO_CCR, ANDI_TO_SR, EORI_TO_CCR, EORI_TO_SR, ORI_TO_CCR, ORI_TO_SR.
//   We have to stop a pending operation in case of a pending interrupt. This is done by rejecting OW_RDY.
//   TOP, CONTROL, Exception Handler Opcode Decoder: Rearranged PC_INC and ipipe flush logic.
//   Write the undecremented Register for MOVE Ax, -(Ax).
//   LINK A7 and PEA(A7) stacks the undecremented A7.
//   Control: LINK A7 and PEA(A7) stacks the undecremented A7.
//   Bus interface: Rearranged the DATA_RDY vs. BUS_FLT logic.
//   UNMARK is now asserted in the end of the write cycle. This avoids data hazards.
//   Fixed a MOVEC writeback issue (use BIW_WB... instead of BIW_...).
//   Fixed a USP writeback issue (use BIW_WB... instead of BIW_...).
//   Introduced a switch NO_PIPELINE.
//   MOVEM-Fix: the effective address in memory to register is stored (STORE_AEFF) not to be overwritten in case the addressing register is also loaded.
//   DBcc: fixed a data hazard for DBcc_COND evaluation by waiting on the ALU result.
//   Fixed DR_WR_1 locking against AR_WR_2.
//   IPIPE is flushed, when there is a memory space change in the end of xx_TO_SR operations.
//   To handle correct addressing, ADR_OFFSET is now cleared right in the end of the respective operation.
//   Fixed a bug in EOR writing back in register direct mode.
//   ADDQ and SUBQ: fixed (An)+ mode. An increments now.
//   Fixed DIVS, DIVU in memory address modes (wrong control flow).
//   ADDQ and SUBQ: fixed condition code control UPDT_CC.
//   Fixed a data hazard bug using addressing modes with index register.
//   Implemented the 68K10 loop mechanism.
//   Fixed several pipelinig issues (hazards).
// Revision 2K19A 20190419 WF
//   Introdeced a new state CALC_AEFF which results in no need of ADR_ATN and a twice highre fmax.
//

module WF68K30L_CONTROL #(
    parameter NO_PIPELINE = 0  // If true the controller work in scalar mode.
) (
    input  logic        CLK,                // System clock.
    input  logic        RESET_CPU,          // CPU reset.

    output logic        BUSY,               // Main controller finished an execution.
    input  logic        BUSY_EXH,
    input  logic        EXH_REQ,
    output logic        INT_TRIG,

    output logic        OW_REQ,             // Operation words request.
    input  logic        OW_VALID,           // Operation words is valid.
    output logic        EW_REQ,             // Extension word request.
    input  logic        EW_ACK,             // Extension word available.
    input  logic        OPD_ACK,            // Opcode has new data.

    output logic        ADR_MARK_USED,
    input  logic        ADR_IN_USE,
    output logic [5:0]  ADR_OFFSET,

    output logic        DATA_RD,
    output logic        DATA_WR,
    input  logic        DATA_RDY,
    input  logic        DATA_VALID,
    output logic        RMC,

    output logic        FETCH_MEM_ADR,
    output logic        LOAD_OP1,
    output logic        LOAD_OP2,
    output logic        LOAD_OP3,
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
    output logic        STORE_MEM_ADR,
    output logic        STORE_AEFF,

    // System control signals:
    input  logic [6:0]  OP,
    output logic [1:0]  OP_SIZE,
    input  logic [13:0] BIW_0,
    input  logic [15:0] BIW_1,
    input  logic [15:0] BIW_2,
    input  logic [15:0] EXT_WORD,

    output logic [2:0]  ADR_MODE,
    output logic [2:0]  AMODE_SEL,
    output logic        USE_DREG,
    output logic        HILOn,

    output logic [6:0]  OP_WB,
    output logic [1:0]  OP_SIZE_WB,
    output logic [7:3]  BIW_0_WB_73,       // Used for EXG.

    output logic        AR_MARK_USED,
    output logic        USE_APAIR,
    input  logic        AR_IN_USE,
    output logic [2:0]  AR_SEL_RD_1,
    output logic [2:0]  AR_SEL_RD_2,
    output logic [2:0]  AR_SEL_WR_1,
    output logic [2:0]  AR_SEL_WR_2,
    output logic        AR_INC,
    output logic        AR_DEC,
    output logic        AR_WR_1,
    output logic        AR_WR_2,

    output logic        DR_MARK_USED,
    output logic        USE_DPAIR,
    input  logic        DR_IN_USE,
    output logic [2:0]  DR_SEL_WR_1,
    output logic [2:0]  DR_SEL_WR_2,
    output logic [2:0]  DR_SEL_RD_1,
    output logic [2:0]  DR_SEL_RD_2,
    output logic        DR_WR_1,
    output logic        DR_WR_2,

    output logic        UNMARK,

    output logic [31:0] DISPLACEMENT,
    output logic        PC_ADD_DISPL,
    output logic        PC_LOAD,
    input  logic        PC_INC_EXH,

    output logic        SP_ADD_DISPL,

    output logic        DFC_WR,
    output logic        DFC_RD,
    output logic        SFC_WR,
    output logic        SFC_RD,

    output logic        VBR_WR,
    output logic        VBR_RD,

    output logic        ISP_RD,
    output logic        ISP_WR,
    output logic        MSP_RD,
    output logic        MSP_WR,
    output logic        USP_RD,
    output logic        USP_WR,

    output logic        IPIPE_FLUSH,        // Abandon the instruction pipeline.

    output logic        ALU_INIT,
    input  logic        ALU_BSY,
    input  logic        ALU_REQ,
    output logic        ALU_ACK,

    output logic        BKPT_CYCLE,
    output logic        BKPT_INSERT,

    input  logic        LOOP_BSY,
    output logic        LOOP_SPLIT,
    output logic        LOOP_EXIT,

    input  logic [2:0]  BF_OFFSET,
    input  logic [5:0]  BF_WIDTH,

    output logic        SR_WR,
    output logic        MOVEM_ADn,
    output int          MOVEP_PNTR,
    output logic        CC_UPDT,
    input  logic [1:0]  TRACE_MODE,
    input  logic        VBIT,
    input  logic        ALU_COND,
    input  logic        DBcc_COND,
    input  logic        BRANCH_ATN,
    output logic        RESET_STRB,
    output logic        BERR,
    output logic        STATUSn,
    output logic        EX_TRACE,
    output logic        TRAP_cc,
    output logic        TRAP_ILLEGAL,       // Used for BKPT.
    output logic        TRAP_V
);

`include "wf68k30L_pkg.svh"

//
typedef enum logic [4:0] {
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
    SWITCH_STATE   = 5'd16
} FETCH_STATES;

typedef enum logic [2:0] {
    IDLE           = 3'd0,
    EXECUTE        = 3'd1,
    ADR_PIPELINE   = 3'd2,
    WRITEBACK      = 3'd3,
    WRITE_DEST     = 3'd4
} EXEC_WB_STATES;

FETCH_STATES    FETCH_STATE;
FETCH_STATES    NEXT_FETCH_STATE;
EXEC_WB_STATES  EXEC_WB_STATE;
EXEC_WB_STATES  NEXT_EXEC_WB_STATE;
int             BF_BYTES;
logic           BF_HILOn;
logic [11:0]    BIW_0_WB;
logic [15:0]    BIW_1_WB;
logic           EW_RDY;
logic           INIT_ENTRY;
logic           MEM_INDIRECT;
logic           MEMADR_RDY;
logic           MOVEM_ADn_I;
logic           MOVEM_ADn_WB;
logic           MOVEM_COND;
logic           MOVEM_FIRST_RD;
logic           MOVEM_INH_WR;
logic           MOVEM_LAST_WR;
logic [3:0]     MOVEM_PNTR;
int             MOVEP_PNTR_I;
logic           OD_REQ_16;
logic           OD_REQ_32;
OP_68K          OP_WB_I = UNIMPLEMENTED;
logic           OW_RDY;
logic           PHASE2;
logic           RD_RDY;
logic           READ_CYCLE;
logic           SBIT_I;
logic           WR_RDY;
logic           WRITE_CYCLE;

// ====================================================================
// Combinational output-assign decoder (submodule)
// ====================================================================
WF68K30L_CTRL_COMB #(
    .NO_PIPELINE     (NO_PIPELINE)
) I_COMB (
    .FETCH_STATE        (FETCH_STATE),
    .NEXT_FETCH_STATE   (NEXT_FETCH_STATE),
    .EXEC_WB_STATE      (EXEC_WB_STATE),
    .NEXT_EXEC_WB_STATE (NEXT_EXEC_WB_STATE),
    .OP                 (OP),
    .OP_WB_I            (OP_WB_I),
    .BIW_0              (BIW_0),
    .BIW_1              (BIW_1),
    .BIW_2              (BIW_2),
    .BIW_0_WB           (BIW_0_WB),
    .BIW_1_WB           (BIW_1_WB),
    .OPD_ACK            (OPD_ACK),
    .OW_VALID           (OW_VALID),
    .OW_RDY             (OW_RDY),
    .EW_ACK             (EW_ACK),
    .EW_RDY             (EW_RDY),
    .DATA_RDY            (DATA_RDY),
    .DATA_VALID          (DATA_VALID),
    .MEMADR_RDY          (MEMADR_RDY),
    .READ_CYCLE          (READ_CYCLE),
    .WRITE_CYCLE         (WRITE_CYCLE),
    .ADR_IN_USE          (ADR_IN_USE),
    .DR_IN_USE           (DR_IN_USE),
    .AR_IN_USE           (AR_IN_USE),
    .ALU_BSY             (ALU_BSY),
    .ALU_REQ             (ALU_REQ),
    .ALU_COND            (ALU_COND),
    .PHASE2              (PHASE2),
    .LOOP_BSY            (LOOP_BSY),
    .MOVEM_ADn_I         (MOVEM_ADn_I),
    .MOVEM_COND          (MOVEM_COND),
    .MOVEM_FIRST_RD      (MOVEM_FIRST_RD),
    .BF_BYTES            (BF_BYTES),
    .BF_HILOn            (BF_HILOn),
    .BRANCH_ATN          (BRANCH_ATN),
    .DBcc_COND           (DBcc_COND),
    .TRACE_MODE          (TRACE_MODE),
    .VBIT                (VBIT),
    .EXH_REQ             (EXH_REQ),
    .BUSY_EXH            (BUSY_EXH),
    // Outputs
    .BUSY                (BUSY),
    .INT_TRIG            (INT_TRIG),
    .OW_REQ              (OW_REQ),
    .EW_REQ              (EW_REQ),
    .DATA_RD             (DATA_RD),
    .DATA_WR             (DATA_WR),
    .RMC                 (RMC),
    .ALU_ACK             (ALU_ACK),
    .RD_RDY              (RD_RDY),
    .WR_RDY              (WR_RDY),
    .INIT_ENTRY          (INIT_ENTRY),
    .FETCH_MEM_ADR       (FETCH_MEM_ADR),
    .STORE_MEM_ADR       (STORE_MEM_ADR),
    .STORE_ADR_FORMAT    (STORE_ADR_FORMAT),
    .STORE_D16           (STORE_D16),
    .STORE_D32_LO        (STORE_D32_LO),
    .STORE_D32_HI        (STORE_D32_HI),
    .STORE_DISPL         (STORE_DISPL),
    .STORE_OD_HI         (STORE_OD_HI),
    .STORE_OD_LO         (STORE_OD_LO),
    .STORE_ABS_HI        (STORE_ABS_HI),
    .STORE_ABS_LO        (STORE_ABS_LO),
    .STORE_IDATA_B2      (STORE_IDATA_B2),
    .STORE_IDATA_B1      (STORE_IDATA_B1),
    .LOAD_OP1            (LOAD_OP1),
    .LOAD_OP2            (LOAD_OP2),
    .LOAD_OP3            (LOAD_OP3),
    .SR_WR               (SR_WR),
    .HILOn               (HILOn),
    .ADR_MODE            (ADR_MODE),
    .AMODE_SEL           (AMODE_SEL),
    .OP_SIZE             (OP_SIZE),
    .BKPT_CYCLE          (BKPT_CYCLE),
    .BKPT_INSERT         (BKPT_INSERT),
    .TRAP_ILLEGAL        (TRAP_ILLEGAL),
    .TRAP_cc             (TRAP_cc),
    .TRAP_V              (TRAP_V),
    .BERR                (BERR),
    .SFC_RD              (SFC_RD),
    .SFC_WR              (SFC_WR),
    .DFC_RD              (DFC_RD),
    .DFC_WR              (DFC_WR),
    .VBR_RD              (VBR_RD),
    .VBR_WR              (VBR_WR),
    .ISP_RD              (ISP_RD),
    .ISP_WR              (ISP_WR),
    .MSP_RD              (MSP_RD),
    .MSP_WR              (MSP_WR),
    .USP_RD              (USP_RD),
    .USP_WR              (USP_WR),
    .PC_ADD_DISPL        (PC_ADD_DISPL),
    .PC_LOAD             (PC_LOAD),
    .IPIPE_FLUSH         (IPIPE_FLUSH),
    .SP_ADD_DISPL        (SP_ADD_DISPL),
    .ALU_INIT            (ALU_INIT),
    .CC_UPDT             (CC_UPDT),
    .ADR_MARK_USED       (ADR_MARK_USED),
    .AR_MARK_USED        (AR_MARK_USED),
    .DR_MARK_USED        (DR_MARK_USED),
    .UNMARK              (UNMARK),
    .USE_APAIR           (USE_APAIR),
    .USE_DPAIR           (USE_DPAIR),
    .LOOP_EXIT           (LOOP_EXIT),
    .RESET_STRB          (RESET_STRB),
    .EX_TRACE            (EX_TRACE)
);

// ====================================================================
// Clocked processes
// ====================================================================

always_ff @(posedge CLK) begin : data_available
    // These flip flops store the information whether the data required in the different
    // states is available or not. This is necessary in case of delayed cycles for
    // example if the required address register is not ready to be read.
    if (RESET_CPU) begin
        OW_RDY <= 1'b0;
    end else if (FETCH_STATE == START_OP && NEXT_FETCH_STATE != START_OP) begin
        OW_RDY <= 1'b0; // Reset.
    end else if (FETCH_STATE == START_OP && (OP == ILLEGAL || OP == RTE || OP == TRAP || OP == UNIMPLEMENTED) && BUSY_EXH) begin
        OW_RDY <= 1'b0; // Done.
    end else if (OPD_ACK) begin
        OW_RDY <= 1'b1; // Set.
    end

    if (FETCH_STATE == START_OP) begin
        EW_RDY <= 1'b0;
    end else if (FETCH_STATE == FETCH_DISPL && NEXT_FETCH_STATE != FETCH_DISPL) begin
        EW_RDY <= 1'b0;
    end else if (FETCH_STATE == FETCH_EXWORD_1 && NEXT_FETCH_STATE != FETCH_EXWORD_1) begin
        EW_RDY <= 1'b0;
    end else if (FETCH_STATE == FETCH_D_LO && NEXT_FETCH_STATE != FETCH_D_LO) begin
        EW_RDY <= 1'b0;
    end else if (FETCH_STATE == FETCH_IDATA_B1 && NEXT_FETCH_STATE != FETCH_IDATA_B1) begin
        EW_RDY <= 1'b0;
    end else if ((FETCH_STATE == FETCH_DISPL || FETCH_STATE == FETCH_EXWORD_1 || FETCH_STATE == FETCH_IDATA_B1 || FETCH_STATE == FETCH_D_LO) && EW_ACK) begin
        EW_RDY <= 1'b1;
    end

    if (FETCH_STATE == START_OP) begin
        MEMADR_RDY <= 1'b0;
    end else if (FETCH_STATE == FETCH_MEMADR && NEXT_FETCH_STATE != FETCH_MEMADR) begin
        MEMADR_RDY <= 1'b0;
    end else if (FETCH_STATE == FETCH_MEMADR && DATA_RDY) begin
        MEMADR_RDY <= 1'b1;
    end
end

always_ff @(posedge CLK) begin : cycle_control
    // This process controls the read and write signals, if
    // asserted simultaneously. In this way, a read cycle is
    // not interrupted by a write cycle and vice versa.
    if (DATA_RDY) begin
        WRITE_CYCLE <= 1'b0;
        READ_CYCLE <= 1'b0;
    end else if (DATA_WR) begin
        WRITE_CYCLE <= 1'b1;
        READ_CYCLE <= 1'b0;
    end else if (DATA_RD) begin
        READ_CYCLE <= 1'b1;
        WRITE_CYCLE <= 1'b0;
    end
end

    // Register select logic (submodule)
    WF68K30L_CTRL_REGSEL I_REGSEL (
        .OP              (OP),
        .OP_WB_I         (OP_WB_I),
        .BIW_0           (BIW_0),
        .BIW_1           (BIW_1),
        .BIW_2           (BIW_2),
        .EXT_WORD        (EXT_WORD),
        .BIW_0_WB        (BIW_0_WB),
        .BIW_1_WB        (BIW_1_WB),
        .FETCH_STATE     (FETCH_STATE),
        .NEXT_FETCH_STATE(NEXT_FETCH_STATE),
        .EXEC_WB_STATE   (EXEC_WB_STATE),
        .INIT_ENTRY      (INIT_ENTRY),
        .PHASE2          (PHASE2),
        .ADR_MODE_I      (ADR_MODE),
        .OP_SIZE_I       (OP_SIZE),
        .ALU_INIT_I      (ALU_INIT),
        .ALU_BSY         (ALU_BSY),
        .RD_RDY          (RD_RDY),
        .WR_RDY          (WR_RDY),
        .DATA_RDY        (DATA_RDY),
        .DATA_VALID      (DATA_VALID),
        .MOVEM_PNTR      (MOVEM_PNTR),
        .MOVEM_ADn_I     (MOVEM_ADn_I),
        .MOVEM_ADn_WB    (MOVEM_ADn_WB),
        .MOVEM_COND      (MOVEM_COND),
        .MOVEM_LAST_WR   (MOVEM_LAST_WR),
        .AR_IN_USE       (AR_IN_USE),
        .DR_IN_USE       (DR_IN_USE),
        .AR_SEL_RD_1     (AR_SEL_RD_1),
        .AR_SEL_RD_2     (AR_SEL_RD_2),
        .AR_SEL_WR_1     (AR_SEL_WR_1),
        .AR_SEL_WR_2     (AR_SEL_WR_2),
        .AR_INC          (AR_INC),
        .AR_DEC          (AR_DEC),
        .AR_WR_1         (AR_WR_1),
        .AR_WR_2         (AR_WR_2),
        .DR_SEL_RD_1     (DR_SEL_RD_1),
        .DR_SEL_RD_2     (DR_SEL_RD_2),
        .DR_SEL_WR_1     (DR_SEL_WR_1),
        .DR_SEL_WR_2     (DR_SEL_WR_2),
        .DR_WR_1         (DR_WR_1),
        .DR_WR_2         (DR_WR_2),
        .USE_DREG        (USE_DREG)
    );

    // This process stores the data for the
    // WRITEBACK or the WRITE_DEST procedure.
    // The MOVEM condition is foreseen to bring
    // the ADn_WB and the PNTR_WB right in time
    // before the address or data registers are
    // marked used.
    always_ff @(posedge CLK) begin : wb_buffer
        if ((OP_WB_I == BFCHG || OP_WB_I == BFCLR || OP_WB_I == BFINS || OP_WB_I == BFSET) && EXEC_WB_STATE == WRITE_DEST && WR_RDY && BF_BYTES == 5) begin
            // This condition may not overwhelm the ALU_INIT so we have to wait in INIT_EXEC_WB for the
            // bit field operations until the last bus cycle finishes.
            OP_SIZE_WB <= BYTE; // Remaining Byte.
        end else if (OP == LINK && FETCH_STATE == START_OP && NEXT_FETCH_STATE == SWITCH_STATE) begin
            OP_SIZE_WB <= OP_SIZE; // Bring this information early because the registers are written early.
        end else if (ALU_INIT) begin
            if (OP == DIVS || OP == DIVU || OP == MULS || OP == MULU) begin
                OP_SIZE_WB <= LONG;
            end else if (OP == MOVEM && BIW_0[10]) begin // Memory to register.
                OP_SIZE_WB <= LONG; // Registers are always written long.
            end else begin
                OP_SIZE_WB <= OP_SIZE; // Store right in the end before data processing starts.
            end

            MOVEM_ADn_WB <= MOVEM_ADn_I;
            OP_WB_I <= OP;
            BIW_0_WB <= BIW_0[11:0];
            BIW_1_WB <= BIW_1;
        end
    end

    assign OP_WB = OP_WB_I;
    assign BIW_0_WB_73 = BIW_0_WB[7:3];

    // P_DISPLACEMENT process (clocked)
    always_ff @(posedge CLK) begin
        logic [31:0] DISPL_VAR;
        case (OP)
            Bcc, BRA, BSR: begin
                case (BIW_0[7:0])
                    8'hFF: begin
                        DISPL_VAR = {BIW_1, BIW_2};
                    end
                    8'h00: begin
                        for (int i = 16; i <= 31; i++) begin
                            DISPL_VAR[i] = BIW_1[15];
                        end
                        DISPL_VAR[15:0] = BIW_1;
                    end
                    default: begin
                        for (int i = 8; i <= 31; i++) begin
                            DISPL_VAR[i] = BIW_0[7];
                        end
                        DISPL_VAR[7:0] = BIW_0[7:0];
                    end
                endcase
            end
            DBcc, MOVEP, RTD: begin
                for (int i = 16; i <= 31; i++) begin
                    DISPL_VAR[i] = BIW_1[15];
                end
                DISPL_VAR[15:0] = BIW_1;
            end
            default: begin // Used for LINK.
                case (BIW_0[11:3])
                    9'b100000001: begin // Long.
                        DISPL_VAR = {BIW_1, BIW_2};
                    end
                    default: begin // Word.
                        for (int i = 16; i <= 31; i++) begin
                            DISPL_VAR[i] = BIW_1[15];
                        end
                        DISPL_VAR[15:0] = BIW_1;
                    end
                endcase
            end
        endcase
        //
        case (OP)
            LINK, MOVEP: DISPLACEMENT <= DISPL_VAR;
            default: DISPLACEMENT <= DISPL_VAR + 32'd2;
        endcase
    end

    // ADR_FORMAT process (clocked)
    always_ff @(posedge CLK) begin
        if (FETCH_STATE == FETCH_EXWORD_1 && EW_ACK && EXT_WORD[8]) begin
            case (EXT_WORD[1:0])
                2'b11: begin
                    MEM_INDIRECT <= 1'b1;
                    OD_REQ_32 <= 1'b1;
                    OD_REQ_16 <= 1'b0;
                end
                2'b10: begin
                    MEM_INDIRECT <= 1'b1;
                    OD_REQ_32 <= 1'b0;
                    OD_REQ_16 <= 1'b1;
                end
                2'b01: begin
                    MEM_INDIRECT <= 1'b1;
                    OD_REQ_32 <= 1'b0;
                    OD_REQ_16 <= 1'b0;
                end
                default: begin
                    MEM_INDIRECT <= 1'b0;
                    OD_REQ_32 <= 1'b0;
                    OD_REQ_16 <= 1'b0;
                end
            endcase
        end
    end

    // MOVEM, MOVEP, bitfield, and address offset control (submodule)
    WF68K30L_CTRL_MOVEM I_MOVEM (
        .CLK             (CLK),
        .RESET_CPU       (RESET_CPU),
        .OP              (OP),
        .BIW_0           (BIW_0),
        .BIW_1           (BIW_1),
        .FETCH_STATE     (FETCH_STATE),
        .NEXT_FETCH_STATE(NEXT_FETCH_STATE),
        .EXEC_WB_STATE   (EXEC_WB_STATE),
        .ADR_MODE_I      (ADR_MODE),
        .OP_SIZE_I       (OP_SIZE),
        .ALU_BSY         (ALU_BSY),
        .ALU_INIT_I      (ALU_INIT),
        .RD_RDY          (RD_RDY),
        .WR_RDY          (WR_RDY),
        .INIT_ENTRY      (INIT_ENTRY),
        .BF_OFFSET       (BF_OFFSET),
        .BF_WIDTH        (BF_WIDTH),
        .MOVEM_ADn       (MOVEM_ADn),
        .MOVEM_ADn_I     (MOVEM_ADn_I),
        .MOVEM_COND      (MOVEM_COND),
        .MOVEM_FIRST_RD  (MOVEM_FIRST_RD),
        .MOVEM_INH_WR    (MOVEM_INH_WR),
        .MOVEM_LAST_WR   (MOVEM_LAST_WR),
        .MOVEM_PNTR      (MOVEM_PNTR),
        .STORE_AEFF      (STORE_AEFF),
        .MOVEP_PNTR      (MOVEP_PNTR),
        .MOVEP_PNTR_I    (MOVEP_PNTR_I),
        .BF_BYTES        (BF_BYTES),
        .BF_HILOn        (BF_HILOn),
        .ADR_OFFSET      (ADR_OFFSET)
    );

    // PHASE2_CONTROL process (clocked)
    // This is used for some operations which require
    // two control sequences.
    always_ff @(posedge CLK) begin
        if (NEXT_FETCH_STATE == START_OP) begin
            PHASE2 <= 1'b0;
        end else if ((OP == ABCD || OP == SBCD) && FETCH_STATE == FETCH_OPERAND && RD_RDY) begin
            PHASE2 <= 1'b1; // One clock cycle delay for destination address calculation.
        end else if ((OP == ADDX || OP == SUBX) && FETCH_STATE == FETCH_OPERAND && RD_RDY) begin
            PHASE2 <= 1'b1; // One clock cycle delay for destination address calculation.
        end else if (OP == CAS2 && FETCH_STATE == INIT_EXEC_WB && !ALU_BSY && !PHASE2) begin
            PHASE2 <= 1'b1; // Used as a control flow switch.
        end else if (OP == CAS2 && FETCH_STATE == INIT_EXEC_WB && !ALU_BSY) begin
            PHASE2 <= 1'b0; // Prepare for writeback / write destination.
        end else if (OP_WB_I == CAS2 && EXEC_WB_STATE == WRITEBACK) begin
            PHASE2 <= 1'b1; // Used as a control flow switch.
        end else if (OP_WB_I == CAS2 && EXEC_WB_STATE == WRITE_DEST && WR_RDY) begin
            PHASE2 <= 1'b1; // Used as a control flow switch.
        end else if ((OP == CHK2 || OP == CMP2 || OP == CMPM) && FETCH_STATE == INIT_EXEC_WB && !ALU_BSY) begin
            PHASE2 <= 1'b1; // Used as a control flow switch.
        end else if (OP == JSR && FETCH_STATE == SLEEP) begin
            PHASE2 <= 1'b1; // One clock cycle delay for address calculation.
        end else if (OP == PEA && FETCH_STATE == SWITCH_STATE) begin
            PHASE2 <= 1'b1; // One clock cycle delay for address calculation.
        end else if (OP == RTR && FETCH_STATE == INIT_EXEC_WB && NEXT_FETCH_STATE == CALC_AEFF) begin
            PHASE2 <= 1'b1; // Used as a control flow switch.
        end else if (OP == MOVE && FETCH_STATE == INIT_EXEC_WB && !ALU_BSY && BIW_0[8:6] > 3'b100) begin
            PHASE2 <= 1'b1; // Indicate destination address calculation is in progress.
        end
    end

    // LOOP_CTRL process (clocked)
    // This flip flop indicates, if a DBcc loop operation has finished if the exception handler indicates an interrupt.
    // If so, no action is required. If the loop is split (when not finished) the exception handler may not increment
    // the PC to hold the value of the loop operation.
    always_ff @(posedge CLK) begin
        if (FETCH_STATE == START_OP && NEXT_FETCH_STATE != START_OP) begin
            LOOP_SPLIT <= 1'b0;
        end else if (PC_INC_EXH) begin
            LOOP_SPLIT <= 1'b0;
        end else if (OP != DBcc && EXH_REQ && LOOP_BSY && FETCH_STATE == SLEEP && NEXT_FETCH_STATE == START_OP) begin
            LOOP_SPLIT <= 1'b1;
        end
    end

    // P_STATUSn process (clocked)
    // This logic is registered to enhance the system performance concerning fmax.
    always_ff @(posedge CLK) begin
        if (FETCH_STATE == START_OP && NEXT_FETCH_STATE != START_OP) begin
            STATUSn <= 1'b0;
        end else begin
            STATUSn <= 1'b1;
        end
    end

    // STATE_REGs process (clocked)
    always_ff @(posedge CLK) begin
        if (RESET_CPU) begin
            FETCH_STATE <= START_OP;
            EXEC_WB_STATE <= IDLE;
        end else if (EW_ACK && !OW_VALID) begin
            FETCH_STATE <= START_OP; // Bus error.
            EXEC_WB_STATE <= IDLE;
        end else if (OPD_ACK && !OW_VALID) begin
            FETCH_STATE <= START_OP; // Bus error.
            EXEC_WB_STATE <= IDLE;
        end else if (DATA_RD && RD_RDY && !DATA_VALID) begin
            FETCH_STATE <= START_OP; // Bus error.
            EXEC_WB_STATE <= IDLE;
        end else if (DATA_WR && RD_RDY && !DATA_VALID) begin
            FETCH_STATE <= START_OP; // Bus error.
            EXEC_WB_STATE <= IDLE;
        end else begin
            FETCH_STATE <= NEXT_FETCH_STATE;
            EXEC_WB_STATE <= NEXT_EXEC_WB_STATE;
        end
    end

    // Debugging:
    // Use this signal to detect instructions in use in the writeback path (OP_WB_I) or in the fetch path (OP).
    // for these instructions you can halt the pipeline in the START_OP state to detect any problems.

    // Fetch state machine next-state decoder (submodule)
    WF68K30L_CTRL_FETCH_DEC I_FETCH_DEC (
        .FETCH_STATE        (FETCH_STATE),
        .EXEC_WB_STATE      (EXEC_WB_STATE),
        .NEXT_EXEC_WB_STATE (NEXT_EXEC_WB_STATE),
        .OP                 (OP),
        .BIW_0              (BIW_0),
        .BIW_1              (BIW_1),
        .BIW_2              (BIW_2),
        .EXT_WORD           (EXT_WORD),
        .OPD_ACK            (OPD_ACK),
        .OW_RDY             (OW_RDY),
        .EW_ACK             (EW_ACK),
        .EW_RDY             (EW_RDY),
        .RD_RDY             (RD_RDY),
        .MEMADR_RDY         (MEMADR_RDY),
        .WR_RDY             (WR_RDY),
        .DR_IN_USE          (DR_IN_USE),
        .AR_IN_USE          (AR_IN_USE),
        .ALU_BSY            (ALU_BSY),
        .ALU_COND           (ALU_COND),
        .ADR_MODE_I         (ADR_MODE),
        .OP_SIZE_I          (OP_SIZE),
        .PHASE2             (PHASE2),
        .MOVEM_COND         (MOVEM_COND),
        .MOVEM_PNTR         (MOVEM_PNTR),
        .MOVEM_FIRST_RD     (MOVEM_FIRST_RD),
        .MOVEP_PNTR_I       (MOVEP_PNTR_I),
        .BF_BYTES           (BF_BYTES),
        .BRANCH_ATN         (BRANCH_ATN),
        .DBcc_COND          (DBcc_COND),
        .TRACE_MODE         (TRACE_MODE),
        .EXH_REQ            (EXH_REQ),
        .BUSY_EXH           (BUSY_EXH),
        .LOOP_BSY           (LOOP_BSY),
        .OD_REQ_32          (OD_REQ_32),
        .OD_REQ_16          (OD_REQ_16),
        .MEM_INDIRECT       (MEM_INDIRECT),
        .NEXT_FETCH_STATE   (NEXT_FETCH_STATE)
    );

    // Execute/writeback state machine next-state decoder (submodule)
    WF68K30L_CTRL_EXEC_DEC I_EXEC_DEC (
        .EXEC_WB_STATE      (EXEC_WB_STATE),
        .FETCH_STATE         (FETCH_STATE),
        .OP_WB_I             (OP_WB_I),
        .BIW_0_WB            (BIW_0_WB),
        .BIW_1_WB            (BIW_1_WB),
        .ALU_INIT_I          (ALU_INIT),
        .ALU_REQ             (ALU_REQ),
        .ALU_COND            (ALU_COND),
        .WR_RDY              (WR_RDY),
        .MOVEM_INH_WR        (MOVEM_INH_WR),
        .BF_BYTES            (BF_BYTES),
        .PHASE2              (PHASE2),
        .NEXT_EXEC_WB_STATE  (NEXT_EXEC_WB_STATE)
    );

endmodule
