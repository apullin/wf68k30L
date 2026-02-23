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

// BF_BYTES: number of bytes for bit field operations = ceil((offset + width) / 8).
// Replaced original lookup table with formula.

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
logic           ADR_MARK_USED_I;
logic [2:0]     ADR_MODE_I;
logic           ALU_INIT_I;
logic           ALU_TRIG;
logic           AR_DEC_I;
logic           AR_WR_I;
logic           AR_WR_II;
int             BF_BYTES;
logic           BF_HILOn;
int             BF_OFFSET_I;
int             BF_WIDTH_I;
logic [11:0]    BIW_0_WB;
logic [15:0]    BIW_1_WB;
logic           DATA_RD_I;
logic           DATA_WR_I;
logic           EW_RDY;
logic           INIT_ENTRY;
logic           IPIPE_FLUSH_I;
logic           LOOP_EXIT_I;
logic           MEM_INDIRECT;
logic           MEMADR_RDY;
logic           MOVEM_ADn_I;
logic           MOVEM_ADn_WB;
logic           MOVEM_COND;
logic           MOVEM_FIRST_RD;
logic           MOVEM_INH_WR;
logic           MOVEM_LAST_WR;
logic [3:0]     MOVEM_PNTR;
logic [3:0]     MOVEM_PVAR_S;
int             MOVEP_PNTR_I;
logic           OD_REQ_16;
logic           OD_REQ_32;
OP_SIZETYPE     OP_SIZE_I;
OP_68K          OP_WB_I = UNIMPLEMENTED;
logic           OW_RDY;
logic           PC_ADD_DISPL_I;
logic           PC_LOAD_I;
logic           PHASE2;
logic           RD_RDY;
logic           READ_CYCLE;
logic           SBIT_I;
logic           SR_WR_I;
logic           UPDT_CC;
logic           WR_RDY;
logic           WRITE_CYCLE;
// Debugging:
logic           OP_TEST;

assign BUSY = OPD_ACK ||              // Early indication.
              LOOP_BSY ||              // Finish the DBcc loop.
              ALU_BSY ||               // Busy, wait.
              (FETCH_STATE != START_OP); // Main controller is busy.

// The interrupt must not be activated when the controller is in its START_OP
// state and fetches new OPWORDS. so we define the trigger for the interrupt
// in the end of the FETCH phase. The SLEEP state is important for STOP.
assign INT_TRIG = (FETCH_STATE == INIT_EXEC_WB || FETCH_STATE == SLEEP);

assign OW_REQ = (BUSY_EXH) ? 1'b0 :
                (EXH_REQ && !LOOP_BSY) ? 1'b0 :  // Non interrupt exception requests, loop has priority.
                (OPD_ACK || OW_RDY) ? 1'b0 :
                (NO_PIPELINE == 1 && FETCH_STATE == START_OP && !ALU_BSY) ? 1'b1 :
                (NO_PIPELINE == 0 && FETCH_STATE == START_OP) ? 1'b1 : 1'b0;

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

always_ff @(posedge CLK) begin : cycle_control
    // This process contros the read and write signals, if
    // asserted simultaneously. In this way, a read cycle is
    // not interrupted by a write cycle and vice versa.
    if (DATA_RDY) begin
        WRITE_CYCLE <= 1'b0;
        READ_CYCLE <= 1'b0;
    end else if (DATA_WR_I) begin
        WRITE_CYCLE <= 1'b1;
        READ_CYCLE <= 1'b0;
    end else if (DATA_RD_I) begin
        READ_CYCLE <= 1'b1;
        WRITE_CYCLE <= 1'b0;
    end
end

    assign RD_RDY = (READ_CYCLE) ? DATA_RDY : 1'b0;
    assign WR_RDY = (WRITE_CYCLE) ? DATA_RDY : 1'b0;

    assign INIT_ENTRY = (FETCH_STATE != INIT_EXEC_WB && NEXT_FETCH_STATE == INIT_EXEC_WB) ? 1'b1 : 1'b0;

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

    assign SR_WR = SR_WR_I;
    assign SR_WR_I = ((OP_WB_I == ANDI_TO_SR || OP_WB_I == EORI_TO_SR || OP_WB_I == ORI_TO_SR) && EXEC_WB_STATE == WRITEBACK) ? 1'b1 :
                     ((OP_WB_I == MOVE_TO_CCR || OP_WB_I == MOVE_TO_SR) && EXEC_WB_STATE == WRITEBACK) ? 1'b1 :
                     (OP_WB_I == STOP && EXEC_WB_STATE == WRITEBACK) ? 1'b1 : 1'b0;

    assign HILOn = (OP_WB_I == CAS2 && FETCH_STATE == FETCH_OPERAND && !PHASE2) ? 1'b1 :
                   (OP_WB_I == CAS2 && EXEC_WB_STATE == WRITEBACK && !PHASE2) ? 1'b1 :
                   (OP == CAS2) ? 1'b0 : BF_HILOn; // Select destinations.

    // Addressing mode:
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
                         (OP == MOVE && FETCH_STATE == FETCH_MEMADR && PHASE2) ? BIW_0[11:9] :
                         (OP == MOVE && FETCH_STATE == START_OP && BIW_0[5:3] < 3'b010 && BIW_0[8:6] != 3'b000) ? BIW_0[11:9] : // Dn, An.
                         (OP == MOVE && FETCH_STATE == CALC_AEFF && PHASE2) ? BIW_0[11:9] :
                         (OP == MOVE && FETCH_STATE == FETCH_OPERAND && RD_RDY && BIW_0[8:6] == 3'b100 && BIW_0[5:3] != 3'b011) ? BIW_0[11:9] : // All except (An)+,-(An).
                         (OP == MOVE && FETCH_STATE == FETCH_IDATA_B1 && BIW_0[8:6] != 3'b000) ? BIW_0[11:9] :
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

    // This process stores the data for the
    // WRITEBACK or the WRITE_DEST procedure.
    // The MOVEM condition is foreseen to bring
    // the ADn_WB and the PNTR_WB right in time
    // before the address or data registers are
    // marked used.
    always_ff @(posedge CLK) begin : wb_buffer
        if ((OP_WB_I == BFCHG || OP_WB_I == BFCLR || OP_WB_I == BFINS || OP_WB_I == BFSET) && EXEC_WB_STATE == WRITE_DEST && WR_RDY && BF_BYTES == 5) begin
            // This condition may not overwhelm the ALU_INIT_I so we have to wait in INIT_EXEC_WB for the
            // bit field operations until the last bus cycle finishes.
            OP_SIZE_WB <= BYTE; // Remaining Byte.
        end else if (OP == LINK && FETCH_STATE == START_OP && NEXT_FETCH_STATE == SWITCH_STATE) begin
            OP_SIZE_WB <= OP_SIZE_I; // Bring this information early because the registers are written early.
        end else if (ALU_INIT_I) begin
            if (OP == DIVS || OP == DIVU || OP == MULS || OP == MULU) begin
                OP_SIZE_WB <= LONG;
            end else if (OP == MOVEM && BIW_0[10]) begin // Memory to register.
                OP_SIZE_WB <= LONG; // Registers are always written long.
            end else begin
                OP_SIZE_WB <= OP_SIZE_I; // Store right in the end before data processing starts.
            end

            MOVEM_ADn_WB <= MOVEM_ADn_I;
            OP_WB_I <= OP;
            BIW_0_WB <= BIW_0[11:0];
            BIW_1_WB <= BIW_1;
        end
    end

    assign OP_WB = OP_WB_I;
    assign BIW_0_WB_73 = BIW_0_WB[7:3];

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

    assign BKPT_CYCLE = (OP == BKPT && FETCH_STATE == FETCH_OPERAND && DATA_RD_I) ? 1'b1 : 1'b0;
    assign BKPT_INSERT = (OP == BKPT && FETCH_STATE == FETCH_OPERAND && RD_RDY && DATA_VALID) ? 1'b1 : 1'b0;

    // All traps must be modeled as strobes. Be aware that the TRAP_cc is released right in the end of the TRAPcc operation.
    // This is necessary to meet the timing requirements (BUSY_EXH, IPIPE_FLUSH, PC_INC) to provide the next PC address. See
    // the exception handler unit for more details.
    assign TRAP_ILLEGAL = (OP == BKPT && FETCH_STATE == FETCH_OPERAND && RD_RDY && !DATA_VALID) ? 1'b1 : 1'b0;

    assign TRAP_cc = (OP == TRAPcc && ALU_COND && FETCH_STATE == SLEEP && NEXT_FETCH_STATE == START_OP) ? 1'b1 : 1'b0;
    assign TRAP_V = (OP == TRAPV && ALU_COND && FETCH_STATE == SLEEP && NEXT_FETCH_STATE == START_OP) ? 1'b1 : 1'b0;

    assign BERR = (FETCH_STATE == START_OP && EXEC_WB_STATE == IDLE) ? 1'b0 : // Disable when controller is not active.
                  (OP == BKPT) ? 1'b0 : // No bus error during breakpoint cycle.
                  (DATA_RDY && !DATA_VALID) ? 1'b1 :
                  (OPD_ACK && !OW_VALID) ? 1'b1 :
                  (EW_ACK && !OW_VALID) ? 1'b1 : 1'b0;

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

    // Concurrent assigns
    assign PC_ADD_DISPL = PC_ADD_DISPL_I;
    assign PC_ADD_DISPL_I = (OP == Bcc && FETCH_STATE != START_OP && NEXT_FETCH_STATE == START_OP && ALU_COND) ? 1'b1 :
                            ((OP == BRA || OP == BSR) && FETCH_STATE != START_OP && NEXT_FETCH_STATE == START_OP) ? 1'b1 :
                            (OP == DBcc && FETCH_STATE != START_OP && NEXT_FETCH_STATE == START_OP && !ALU_COND && !DBcc_COND) ? 1'b1 : 1'b0;

    assign PC_LOAD = PC_LOAD_I;
    assign PC_LOAD_I = ((OP == JMP || OP == JSR) && FETCH_STATE != START_OP && NEXT_FETCH_STATE == START_OP) ? 1'b1 :
                       ((OP == RTD || OP == RTR || OP == RTS) && FETCH_STATE != START_OP && NEXT_FETCH_STATE == START_OP) ? 1'b1 : 1'b0;

    // The pipe is flushed for the system control instructions. Be aware, that the operations resulting in an exception
    // like the CHK or TRAP operations flush the pipe via the exception handler.
    // Context switch may occur from:
    //   changing the PC value (branches etc.)
    //   changing the RAM space (status register MSBs).
    //   changing Function codes or the active stack pointer.
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

    // UPDT_CC concurrent assign
    assign UPDT_CC = ((OP_WB_I == ADDQ || OP_WB_I == SUBQ) && BIW_0_WB[5:3] == 3'b001) ? 1'b0 : // No update for ADDQ and SUBQ when destination is an address register.
                     (OP == CAS2 && FETCH_STATE == INIT_EXEC_WB && EXEC_WB_STATE == WRITEBACK) ? 1'b0 : // First 'Z' flag was zero, do not update the second access.
                     (OP == CAS2 && FETCH_STATE == INIT_EXEC_WB && EXEC_WB_STATE == WRITE_DEST && PHASE2) ? 1'b0 : ALU_REQ; // Suppress third update.

    // CC_UPDT with OP_WB_I select
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

    // ADR_MARK_USED_I concurrent assign
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

    // ADR_MARK_USED concurrent assign
    assign ADR_MARK_USED = ((OP_WB_I == BFCHG || OP_WB_I == BFCLR) && EXEC_WB_STATE == WRITE_DEST && WR_RDY && BF_BYTES == 5) ? 1'b1 :
                           ((OP_WB_I == BFINS || OP_WB_I == BFSET) && EXEC_WB_STATE == WRITE_DEST && WR_RDY && BF_BYTES == 5) ? 1'b1 :
                           (OP_WB_I == CAS && EXEC_WB_STATE == EXECUTE && ALU_COND) ? 1'b1 :
                           (OP_WB_I == CAS2 && EXEC_WB_STATE == ADR_PIPELINE && ALU_COND) ? 1'b1 :
                           (OP_WB_I == CAS2 && EXEC_WB_STATE == WRITE_DEST && WR_RDY && !PHASE2) ? 1'b1 : ADR_MARK_USED_I;

    // AR_MARK_USED concurrent assign
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

    // DR_MARK_USED concurrent assign
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

    assign UNMARK = (EXEC_WB_STATE != IDLE && NEXT_EXEC_WB_STATE == IDLE) ? 1'b1 : 1'b0; // Release a pending write cycle when done.

    // These signals indicates, that two registers are prepared to be written. In this case, the values
    // in both of these registers are invalidated before the writeback.
    assign USE_APAIR = (OP == EXG && BIW_0[7:3] == 5'b01001) ? 1'b1 : 1'b0;
    assign USE_DPAIR = (OP == EXG && BIW_0[7:3] == 5'b01000) ? 1'b1 :
                       ((OP == DIVS || OP == DIVU) && OP_SIZE_I == LONG && BIW_1[14:12] != BIW_1[2:0]) ? 1'b1 :
                       ((OP == MULS || OP == MULU) && OP_SIZE_I == LONG && BIW_1[10] && BIW_1[14:12] != BIW_1[2:0]) ? 1'b1 : 1'b0;

    assign LOOP_EXIT = LOOP_EXIT_I;
    assign LOOP_EXIT_I = (OP != DBcc && LOOP_BSY && FETCH_STATE != START_OP && NEXT_FETCH_STATE == START_OP && EXH_REQ) ? 1'b1 : // Exception! break the loop.
                         (OP == DBcc && LOOP_BSY && FETCH_STATE == SLEEP && NEXT_FETCH_STATE == START_OP && (ALU_COND || DBcc_COND)) ? 1'b1 : 1'b0; // 68010 loop mechanism.

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

    // BF_OFFSET_I, BF_WIDTH_I concurrent assigns
    assign BF_OFFSET_I = BF_OFFSET;
    assign BF_WIDTH_I = BF_WIDTH;

    // RESET_STRB, EX_TRACE concurrent assigns
    assign RESET_STRB = (OP == OP_RESET && INIT_ENTRY) ? 1'b1 : 1'b0;

    assign EX_TRACE = (OP == ILLEGAL || OP == UNIMPLEMENTED) ? 1'b0 :
                      (TRACE_MODE == 2'b10 && OPD_ACK && FETCH_STATE == START_OP && OP == TRAP) ? 1'b1 :
                      (TRACE_MODE == 2'b10 && FETCH_STATE != START_OP && NEXT_FETCH_STATE == START_OP) ? 1'b1 :
                      (TRACE_MODE == 2'b01 && FETCH_STATE == INIT_EXEC_WB && !ALU_BSY && OP == TRAP) ? 1'b1 :
                      (TRACE_MODE == 2'b01 && FETCH_STATE == INIT_EXEC_WB && !ALU_BSY && OP == TRAPcc && ALU_COND) ? 1'b1 :
                      (TRACE_MODE == 2'b01 && FETCH_STATE == INIT_EXEC_WB && !ALU_BSY && OP == TRAPV && VBIT) ? 1'b1 :
                      (TRACE_MODE == 2'b01 && SR_WR_I) ? 1'b1 : // Status register manipulations.
                      (TRACE_MODE == 2'b01 && (PC_ADD_DISPL_I | PC_LOAD_I)); // All branches and jumps.

    // P_STATUSn process (clocked)
    // This logic is registered to enhance the system performance concerning fmax.
    always_ff @(posedge CLK) begin
        if (FETCH_STATE == START_OP && NEXT_FETCH_STATE != START_OP) begin
            STATUSn <= 1'b0;
        end else begin
            STATUSn <= 1'b1;
        end
    end

    // ADDRESS_OFFSET process (clocked)
    always_ff @(posedge CLK) begin
        logic [5:0] ADR_OFFS_VAR;
        if (FETCH_STATE == START_OP) begin
            ADR_OFFS_VAR = 6'b000000;
        end else begin
            case (OP)
                BFCHG, BFCLR, BFINS, BFSET: begin
                    if (FETCH_STATE == FETCH_OPERAND && RD_RDY && BF_BYTES == 5) begin
                        ADR_OFFS_VAR = ADR_OFFS_VAR + 6'b000100; // Another Byte required.
                    end else if (INIT_ENTRY) begin
                        ADR_OFFS_VAR = 6'b000000; // Restore.
                    end else if (FETCH_STATE == INIT_EXEC_WB && !ALU_BSY && BF_BYTES == 5) begin
                        ADR_OFFS_VAR = ADR_OFFS_VAR + 6'b000100; // Another Byte required.
                    end
                end
                BFEXTS, BFEXTU, BFFFO, BFTST: begin
                    if (FETCH_STATE == FETCH_OPERAND && RD_RDY && BF_BYTES == 5) begin
                        ADR_OFFS_VAR = ADR_OFFS_VAR + 6'b000100; // Another Byte required.
                    end
                end
                CHK2, CMP2: begin
                    if (FETCH_STATE == FETCH_OPERAND && RD_RDY && OP_SIZE_I == LONG) begin
                        ADR_OFFS_VAR = ADR_OFFS_VAR + 6'b000100;
                    end else if (FETCH_STATE == FETCH_OPERAND && RD_RDY && OP_SIZE_I == WORD) begin
                        ADR_OFFS_VAR = ADR_OFFS_VAR + 6'b000010;
                    end else if (FETCH_STATE == FETCH_OPERAND && RD_RDY) begin
                        ADR_OFFS_VAR = ADR_OFFS_VAR + 6'b000001;
                    end
                end
                MOVEM: begin
                    if (ADR_MODE_I == 3'b011 || ADR_MODE_I == 3'b100) begin // (An)+, -(An).
                        // null -- Offset comes from addressing register.
                    end else if (BIW_0[10] && !MOVEM_FIRST_RD) begin
                        // null -- Do not increment before the first bus access.
                    end else if (MOVEM_COND && FETCH_STATE == INIT_EXEC_WB && !ALU_BSY && OP_SIZE_I == LONG) begin
                        ADR_OFFS_VAR = ADR_OFFS_VAR + 6'b000100; // Register to memory.
                    end else if (MOVEM_COND && FETCH_STATE == INIT_EXEC_WB && !ALU_BSY) begin
                        ADR_OFFS_VAR = ADR_OFFS_VAR + 6'b000010; // Register to memory.
                    end
                end
                MOVEP: begin
                    if (FETCH_STATE == INIT_EXEC_WB && !ALU_BSY) begin
                        ADR_OFFS_VAR = ADR_OFFS_VAR + 6'b000010;
                    end
                end
                default: begin
                    // null
                end
            endcase
        end
        ADR_OFFSET <= ADR_OFFS_VAR;
    end

    // BITFIELD_CONTROL process (clocked)
    always_ff @(posedge CLK) begin
        if (FETCH_STATE == START_OP) begin
            case (OP)
                BFCHG, BFCLR, BFEXTS, BFEXTU, BFFFO, BFINS, BFSET, BFTST: begin
                    if (NEXT_FETCH_STATE == INIT_EXEC_WB) begin
                        BF_BYTES <= 4; // Register access.
                    end else begin
                        BF_BYTES <= (BF_OFFSET_I + BF_WIDTH_I + 7) >> 3;
                    end
                    BF_HILOn <= 1'b1;
                end
                default: begin
                    // null
                end
            endcase
        end else if (FETCH_STATE == FETCH_OPERAND) begin
            if (RD_RDY && BF_BYTES == 5) begin
                BF_BYTES <= 1;
                BF_HILOn <= 1'b0;
            end else if (RD_RDY) begin
                BF_BYTES <= (BF_OFFSET_I + BF_WIDTH_I + 7) >> 3; // Restore.
                BF_HILOn <= 1'b1; // Restore.
            end
        end else if (EXEC_WB_STATE == WRITE_DEST) begin
            if (WR_RDY && BF_BYTES == 5) begin
                BF_BYTES <= 1;
                BF_HILOn <= 1'b0;
            end else if (WR_RDY) begin
                BF_HILOn <= 1'b1; // Restore.
            end
        end
    end

    // MOVEM_CONTROL process (clocked)
    always_ff @(posedge CLK) begin
        logic [3:0] MOVEM_PVAR;
        logic [4:0] BITS;
        if (FETCH_STATE == START_OP) begin
            MOVEM_PVAR = 4'h0;
        end else if (FETCH_STATE == INIT_EXEC_WB && !MOVEM_COND && MOVEM_PVAR < 4'hF && !ALU_BSY) begin
            MOVEM_PVAR = MOVEM_PVAR + 4'd1; // No data to write.
        end else if (BIW_0[10] && MOVEM_FIRST_RD && FETCH_STATE == INIT_EXEC_WB && MOVEM_PVAR < 4'hF && !ALU_BSY) begin
            MOVEM_PVAR = MOVEM_PVAR + 4'd1; // Data has not been read.
        end else if (!BIW_0[10] && FETCH_STATE == INIT_EXEC_WB && MOVEM_PVAR < 4'hF && !ALU_BSY) begin
            MOVEM_PVAR = MOVEM_PVAR + 4'd1; // Data has been written.
        end

        if (OP == MOVEM && ALU_INIT_I && ADR_MODE_I == 3'b011 && MOVEM_ADn_I && MOVEM_PNTR[2:0] == BIW_0[2:0]) begin
            MOVEM_INH_WR <= 1'b1; // Do not write the addressing register.
        end else if (ALU_INIT_I) begin
            MOVEM_INH_WR <= 1'b0;
        end

        if (FETCH_STATE == START_OP) begin
            MOVEM_FIRST_RD <= 1'b0;
        end else if (OP == MOVEM && FETCH_STATE == FETCH_OPERAND && RD_RDY) begin
            MOVEM_FIRST_RD <= 1'b1;
        end

        if (RESET_CPU || (FETCH_STATE != START_OP && NEXT_FETCH_STATE == START_OP)) begin
            BITS = 5'b00000;
            MOVEM_LAST_WR <= 1'b0;
        end else if (OP == MOVEM && FETCH_STATE == START_OP && NEXT_FETCH_STATE != START_OP && ADR_MODE_I == 3'b100) begin // -(An).
            for (int i = 0; i <= 15; i++) begin
                BITS = BITS + {4'b0, BIW_1[i]}; // Count number of '1's.
            end
            MOVEM_LAST_WR <= 1'b0;
        end else if (OP == MOVEM && ALU_INIT_I && BITS > 5'b00001) begin
            BITS = BITS - 5'd1;
        end else if (OP == MOVEM && BITS == 5'b00001) begin
            MOVEM_LAST_WR <= 1'b1;
        end

        // During the MOVEM instruction in memory to register operation and addressing modes "010", "101","110" the effective address might be
        // affected, if the addressing register is active in the register list mask. To deal with it, the effective address is stored until the
        // MOVEM has read all registers from memory addressed by the initial addressing register (old value).
        // This logic is modeled synchronously (one clock latency) due to the one clock delay of the address calculation.
        if (OP != MOVEM || BIW_0[10] != 1'b1 || (ADR_MODE_I != 3'b010 && ADR_MODE_I != 3'b101 && ADR_MODE_I != 3'b110)) begin
            STORE_AEFF <= 1'b0;
        end else if (FETCH_STATE != START_OP && NEXT_FETCH_STATE == START_OP) begin
            STORE_AEFF <= 1'b0; // Operation completed.
        end else if (FETCH_STATE == SWITCH_STATE || FETCH_STATE == CALC_AEFF || FETCH_STATE == FETCH_OPERAND || FETCH_STATE == INIT_EXEC_WB) begin
            STORE_AEFF <= 1'b1;
        end

        MOVEM_PVAR_S <= MOVEM_PVAR;
    end

    // MOVEM_COMB process (combinational)
    always_comb begin
        logic [3:0] INDEX;
        // This signal determines whether to handle address or data registers.
        if (ADR_MODE_I == 3'b100) begin // -(An).
            MOVEM_ADn_I = ~MOVEM_PVAR_S[3];
            MOVEM_ADn = ~MOVEM_PVAR_S[3];
        end else begin
            MOVEM_ADn_I = MOVEM_PVAR_S[3];
            MOVEM_ADn = MOVEM_PVAR_S[3];
        end

        INDEX = MOVEM_PVAR_S;

        // The following signal determines if a register is affected or not, depending
        // on the status of the register list bit.
        if (OP == MOVEM && BIW_1[INDEX]) begin
            MOVEM_COND = 1'b1;
        end else begin
            MOVEM_COND = 1'b0;
        end

        // This signal determines whether to handle address or data registers.
        if (ADR_MODE_I == 3'b100) begin // -(An).
            MOVEM_PNTR = ~MOVEM_PVAR_S; // Count down.
        end else begin
            MOVEM_PNTR = MOVEM_PVAR_S;
        end
    end

    // MOVEP_CONTROL process (clocked part)
    // This logic handles the bytes to be written or read during the MOVEP
    // operation. In LONG mode 4 bytes are affected and in WORD mode two bytes.
    always_ff @(posedge CLK) begin
        if (RESET_CPU || (FETCH_STATE != START_OP && NEXT_FETCH_STATE == START_OP)) begin
            MOVEP_PNTR_I <= 0;
        end else if (FETCH_STATE == START_OP && (BIW_0[8:6] == 3'b101 || BIW_0[8:6] == 3'b111)) begin
            MOVEP_PNTR_I <= 3; // LONG.
        end else if (FETCH_STATE == START_OP) begin
            MOVEP_PNTR_I <= 1; // WORD.
        end else if (FETCH_STATE == INIT_EXEC_WB && !ALU_BSY && MOVEP_PNTR_I != 0) begin
            MOVEP_PNTR_I <= MOVEP_PNTR_I - 1; // Register to memory
        end
    end
    // Concurrent assign from MOVEP_CONTROL process
    assign MOVEP_PNTR = MOVEP_PNTR_I;

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
        end else if (DATA_RD_I && RD_RDY && !DATA_VALID) begin
            FETCH_STATE <= START_OP; // Bus error.
            EXEC_WB_STATE <= IDLE;
        end else if (DATA_WR_I && RD_RDY && !DATA_VALID) begin
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
    // case (OP)
    // case (OP_WB_I)
    //     OP_TEST <= '1' when ADDA | ADDQ | EXG | LEA | LINK | MOVEA | MOVE_USP | MOVEC | MOVEM | MOVES | SUBA | SUBQ | UNLK | // Address register manipulations.
    //                         ASL | ASR | LSL | LSR | ROTL | ROTR | ROXL | ROXR | DIVS | DIVU | // Long ALU operations. (68K10, 68K30L have no barrel shifter).
    //                         ADD | AND_B | CLR | EOR | SUB | OR_B | CMP | CMPA | CMPM | NOT_B | NBCD | NEG | NEGX | SWAP | TAS | TST
    //                         ANDI_TO_CCR | ANDI_TO_SR | EORI_TO_CCR | EORI_TO_SR | ORI_TO_CCR | ORI_TO_SR | MOVE_FROM_CCR |
    //                         MOVE_TO_CCR | MOVE_FROM_SR | MOVE_TO_SR | MOVE | MOVEQ | MOVEP | PEA |
    //                         ABCD | ADDX | SBCD | SUBX | BCHG | BCLR | BSET | BTST | EXT |
    //                         Bcc | BSR | CHK | DBcc | JSR | TRAPV | RTR | Scc | STOP |
    //                         ADDI | ANDI | SUBI | CMPI | EORI | ORI | MULS | MULU: OP_TEST = 1'b1;
    //     default: OP_TEST = 1'b0;
    // endcase
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
        .ADR_MODE_I         (ADR_MODE_I),
        .OP_SIZE_I          (OP_SIZE_I),
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
        .ALU_INIT_I          (ALU_INIT_I),
        .ALU_REQ             (ALU_REQ),
        .ALU_COND            (ALU_COND),
        .WR_RDY              (WR_RDY),
        .MOVEM_INH_WR        (MOVEM_INH_WR),
        .BF_BYTES            (BF_BYTES),
        .PHASE2              (PHASE2),
        .NEXT_EXEC_WB_STATE  (NEXT_EXEC_WB_STATE)
    );

endmodule
