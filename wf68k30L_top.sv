//----------------------------------------------------------------------//
//                                                                      //
// WF68K30L IP Core.                                                    //
//                                                                      //
// This is the top level structural design unit of the 68K30L           //
// complex instruction set (CISC) microcontroller. It's program-        //
// ming model is (hopefully) fully compatible with Motorola's           //
// MC68030. This core features a pipelined architecture. In com-        //
// parision to the fully featured 68K30 the core has no MMU, no         //
// data and instruction cache and no coprocessor interface. This        //
// results in missing burstmodes which are not required due to          //
// lack of cache. Missing coprocessor operations are:                   //
// cpBcc, cpDBcc, cpGEN, cpRESTORE, cpSAVE, cpScc, cpTRAPcc.           //
// Missing MMU operations are: PFLUSH, PLOAD, PMOVE and PTEST.          //
// The trap handler does not process the following exceptions           //
// which lack due to the missing MMU and coprocessor interface:         //
// PRE_EXC_CP, MID_EXC_CP , POST_EXC_CP, EXC_VECT_CP, MMU_CFG_ERR     //
// The shifter in the 68K30 is a barrel shifter and in this core        //
// it is a conventional shift register controlled logic.                //
// This core features the loop operation mode of the 68010 to           //
// deal with DBcc loops. This feature is a predecessor to the           //
// MC68020/30/40 caches.                                                //
// The exception handler works for the RTE but without taking the       //
// SSW into account which is intended to restore from a defectice       //
// bus error stack frame.                                               //
//                                                                      //
// Enjoy.                                                               //
//                                                                      //
// Author(s):                                                           //
// - Wolfgang Foerster, wf@experiment-s.de; wf@inventronik.de           //
//                                                                      //
//----------------------------------------------------------------------//
//                                                                      //
// Copyright (c) 2014-2019 Wolfgang Foerster Inventronik GmbH.          //
//                                                                      //
// This documentation describes Open Hardware and is licensed           //
// under the CERN OHL v. 1.2. You may redistribute and modify           //
// this documentation under the terms of the CERN OHL v.1.2.            //
// (http://ohwr.org/cernohl). This documentation is distributed         //
// WITHOUT ANY EXPRESS OR IMPLIED WARRANTY, INCLUDING OF                //
// MERCHANTABILITY, SATISFACTORY QUALITY AND FITNESS FOR A              //
// PARTICULAR PURPOSE. Please see the CERN OHL v.1.2 for                //
// applicable conditions                                                //
//                                                                      //
//----------------------------------------------------------------------//
//
// Revision History
//
// Revision 2K14B 20141201 WF
//   Initial Release.
// Revision 2K16A 20160620 WF
//   Control section: fixed a bug in the MOVEM operation. Thanks to Raymond Mounissamy.
//   Address section: minor optimizations.
// Revision 2K18A 20180620 WF
//   Control: Fixed a bug in MOVE An,-(Ay). Thanks to Gary Bingham for the support.
//   Top: changed ALU_OP1_IN multiplexer due to MOVE An,-(Ay) bug.
//   Address registers: Changed ADR_ATN logic to be valid one clock cycle earlier.
// Revision 2K18A (unreleased) WF
//   Top / Opdecoder / Exhandler: Removed REST_BIW_0.
//   Exhandler: Removed PC_OFFSET.
//   ALU: Bug fix: MOVEM sign extension.
//   Address registers: Removed PC_REG.
//   Control: Fixed wrong PEA behaviour.
//   Control: Fixed the displacement for LINK.
//   ALU: Fix for restoring correct values during the DIVS and DIVU in word format.
//   Control: Fixed the operation size for MOVEQ.
//   Control: ADDQ, SUBQ Fix: address registers are always written long.
//   Top, exception handler, address registers: Fixed PC restoring during exception processing.
//   Control: ADDI, ANDI, EORI, ORI, SUBI: address is not marked used if destination is Dn.
//   Control: ADDI, ANDI, EORI, ORI, SUBI: data register is marked used if destination is Dn.
//   Bus interface: Suppress bus faults during RESET instruction.
//   Bus interface: Optimized ASn and DSn timing for synchronous RAM.
//   ALU: Fixed the SUBQ calculation.
//   Opcode decoder: Fixed the PW_EW_OFFSET calculation for JSR.
//   Top: fixed the value of DATA_IMMEDIATE for ADDQ and SUBQ in case of #8.
//   ALU: Rearanged the Offset for the JSR instruction.
//   Control: Fixed AR_MARK_USED in LINK.
//   ALU: EXT instruction uses now RESULT(63 downto 0).
//   Top: Introduced OP_WB form the control unit.
//   Top: Rearanged the AR_IN_1 multiplexer to avoid data hazards.
//   Top: Rearanged the AR_IN_2 multiplexer to avoid data hazards.
//   Top: Rearanged the DR_IN_1 multiplexer to avoid data hazards.
//   Top: Rearanged the DR_IN_2 multiplexer to avoid data hazards.
//   EXG: rearanged logic to meet the new top level multiplexers.
//   Control: LINK, UNLK: wait in START_OP until the ALU is ready (avoids possible data hazards).
//   Top: ALU_OP1_IN to meet the new EXG multiplexers.
//   Top: ALU_OP2_IN to meet the new EXG multiplexers.
//   Address registers: Fixed the writing of ISP_REG during EXG instruction with two address registers.
//   Control: MOVEM: Fixed predecrement mode for consecutive MOVEM -(An).
//   Control: MOVEP: MOVEP_PNTR is now correct for consecutive MOVEP.
//   Control: MOVEP: avoid structural hazard in SWITCH_STATE by waiting for ALU.
//   Control: LINK, UNLK: fixed the write back operation size.
//   Exception handler: Fixed the vector calculation of INT vectors.
//   Exception handler: Fixed faulty modelling in IRQ_FILTER.
//   Exception handler: Implemented the AVEC_FILTER to better meet bus timings.
//   Control: EOR: fixed a bug in the writeback mechanism.
//   Control: BSR, JSR: EXEC_WB state machine waits now for ALU_INIT. Avoid structural / data hazard.
//   Control: the instruction pipe is not flushed for MOVE_FROM_CCR, MOVE_FROM_SR, MOVE_USP, MOVEC.
//   Control: Modifications in the FETCH state machine to avoid several data hazards for MOVEM, MOVE_FROM_CCR, MOVE_FROM_SR.
//   Control: Modifications in the FETCH state machine to avoid several data hazards for ANDI_TO_CCR, ANDI_TO_SR, EORI_TO_CCR, EORI_TO_SR, ORI_TO_CCR, ORI_TO_SR.
//   Exception handler: The RTE exception has now highest priority (avoids mismatch).
//   Control: Bugfix: the condition codes were not updated if there was a pending interrupt.
//   Control: We have to stop a pending operation in case of a pending interrupt. This is done by rejecting OW_RDY.
//   TOP, Control, Exception handler Opcode Decoder: Rearanged PC_INC and ipipe flush logic.
//   Bus interface: BUS_EN is now active except during arbitration.
//   Control: Write the undecremented Register for MOVE Ax, -(Ax).
//   ALU: Fixed wrong condition codes for AND_B, ANDI, EOR, EORI, OR_B, ORI and NOT_B.
//   Exception handler: Update the IRQ mask only for RESET and interrupts.
//   Bus interface: Opted out START_READ and CHK_RD.
//   Control: UNMARK is now asserted in the end of the write cycle. This avoids data hazards.
//   Exception handler: external interrupts are postponed if any system controllers are in initialize operation status.
//   ALU: Fixed writeback issues in the status register logic.
//   ALU: Fixed writing the stack pointer registers (MSBIT is used now).
//   Control: Fixed a MOVEC writeback issue (use BIW_WB... instead of BIW_...).
//   Control: Fixed a USP writeback issue (use BIW_WB... instead of BIW_...).
//   ALU: the address registers are always written long.
//   Top: opted out SBIT_AREG and MBIT_AREG.
//   Address register bugfix: exception handler do not increment and decrement the USP any more.
//   Control: Introduced a switch NO_PIPELINE.
//   Address registers, Control: MOVEM-Fix see STORE_AEFF.
//   Control: DBcc: fixed a data hazard for DBcc_COND evaluation by waiting on the ALU result.
//   Control: Fixed DR_WR_1 locking against AR_WR_2.
//   Control: IPIPE is flushed, when there is a memory space change in the end of xx_TO_SR operations.
//   Control: To handle correct addressing, ADR_OFFSET is now cleared right in the end of the respective operation.
//   Control: Fixed a bug in EOR writing back in register direct mode.
//   ALU: Fixed a bug in MULU.W (input operands are now 16 bit wide).
//   Control: ADDQ and SUBQ: fixed (An)+ mode. An increments now.
//   Control: Fixed DIVS, DIVU in memory address modes (wrong control flow).
//   Control: ADDQ and SUBQ: fixed condition code control UPDT_CC.
//   Control: fixed a data hazard bug using addressing modes with index register.
//   Opcode decoder: Fix for unimplemented or illegal operations: PC is increased before stacked.
//   Exception handler: RTE now loads the address offset correctly when entering the handler.
//   Control: Implemented the 68K10 loop mechanism.
//   Opcode decoder Implemented the 68K10 loop mechanism.
//   Bus interface: Fixed the faulty bus arbitration logic.
//   Opcode decoder: Removed CAHR, we have no cache.
//   Top: Removed a data hazard in the DR_IN_1 multiplexer (EXG operation).
// Revision 2K19A 20190419 WF
//   Control: Fixed several pipelinig issues (hazards).
//   New feature: Branch prediction for the status register manipulation operations (BRANCH_ATN).
//   Exception handler, Opcode decoder, Top: Rearranged address error handling.
//   Bus interface, Exception handler, Opcode decoder, Top: Rearranged address error handling.
//   Top, Address registers: Removed ADR_ATN. Not required any more.
//   Control: Introdeced a new state CALC_AEFF which results in no need of ADR_ATN and a twice highre fmax.
//

module WF68K30L_TOP #(
    parameter logic [15:0] VERSION = 16'h1904, // CPU version number.
    // The following two switches are for debugging purposes. Default for both is false.
    parameter NO_PIPELINE = 0,  // If true the main controller work in scalar mode.
    parameter NO_LOOP     = 0   // If true the DBcc loop mechanism is disabled.
) (
    input  logic        CLK,

    // Address and data:
    output logic [31:0] ADR_OUT,
    input  logic [31:0] DATA_IN,
    output logic [31:0] DATA_OUT,
    output logic        DATA_EN,         // Enables the data port.

    // System control:
    input  logic        BERRn,
    input  logic        RESET_INn,
    output logic        RESET_OUT,       // Open drain.
    input  logic        HALT_INn,
    output logic        HALT_OUTn,       // Open drain.

    // Processor status:
    output logic [2:0]  FC_OUT,

    // Interrupt control:
    input  logic        AVECn,
    input  logic [2:0]  IPLn,
    output logic        IPENDn,

    // Aynchronous bus control:
    input  logic [1:0]  DSACKn,
    output logic [1:0]  SIZE,
    output logic        ASn,
    output logic        RWn,
    output logic        RMCn,
    output logic        DSn,
    output logic        ECSn,
    output logic        OCSn,
    output logic        DBENn,           // Data buffer enable.
    output logic        BUS_EN,          // Enables ADR, ASn, DSn, RWn, RMCn, FC and SIZE.

    // Synchronous bus control:
    input  logic        STERMn,

    // Status controls:
    output logic        STATUSn,
    output logic        REFILLn,

    // Bus arbitration control:
    input  logic        BRn,
    output logic        BGn,
    input  logic        BGACKn
);

`include "wf68k30L_pkg.svh"

// ---- Internal signal declarations ----

logic        ADn;
logic [31:0] ADR_CPY_EXH;
logic [31:0] ADR_EFF;
logic [31:0] ADR_EFF_WB;
logic [31:0] ADR_L;
logic [31:0] ADR_LATCH;
logic [2:0]  ADR_MODE;
logic [2:0]  ADR_MODE_MAIN;
logic        ADR_IN_USE;
logic [31:0] ADR_OFFSET;
logic [31:0] ADR_OFFSET_EXH;
logic [5:0]  ADR_OFFSET_MAIN;
logic [31:0] ADR_P;
logic        ADR_MARK_UNUSED_MAIN;
logic        ADR_MARK_USED;
logic        AERR;
logic        ALU_ACK;
logic        ALU_BSY;
logic        ALU_COND;
logic        ALU_INIT;
logic        ALU_LOAD_OP1;
logic        ALU_LOAD_OP2;
logic        ALU_LOAD_OP3;
logic [31:0] ALU_OP1_IN;
logic [31:0] ALU_OP2_IN;
logic [31:0] ALU_OP3_IN;
logic        ALU_REQ;
logic [63:0] ALU_RESULT;
logic [2:0]  AMODE_SEL;
logic        AR_DEC;
logic [31:0] AR_IN_1;
logic [31:0] AR_IN_2;
logic        AR_IN_USE;
logic        AR_INC;
logic        AR_MARK_USED;
logic [31:0] AR_OUT_1;
logic [31:0] AR_OUT_2;
logic [2:0]  AR_SEL_RD_1;
logic [2:0]  AR_SEL_RD_1_MAIN;
logic [2:0]  AR_SEL_RD_2;
logic [2:0]  AR_SEL_WR_1;
logic [2:0]  AR_SEL_WR_2;
logic        AR_WR_1;
logic        AR_WR_2;
logic        AVECn_BUSIF;
logic        BERR_MAIN;
logic [4:0]  BITPOS;
logic [15:0] BIW_0;
logic [7:3]  BIW_0_WB_73;
logic [15:0] BIW_1;
logic [15:0] BIW_2;
logic [31:0] BF_OFFSET;
logic [5:0]  BF_WIDTH = 6'b100000; // Default needed for simulation!
logic        BKPT_CYCLE;
logic        BKPT_INSERT;
logic        BRANCH_ATN;
logic        BUS_BSY;
logic        BUSY_EXH;
logic        BUSY_MAIN;
logic        BUSY_OPD;
logic        CC_UPDT;
logic        CPU_SPACE;
logic        CPU_SPACE_EXH;
logic [2:0]  DFC;
logic        DFC_RD;
logic        DFC_WR;
logic        DR_WR_1;
logic        DR_WR_2;
logic        DR_MARK_USED;
logic [31:0] DATA_FROM_CORE;
logic [31:0] DATA;
logic [31:0] DATA_IN_EXH;
logic [31:0] DATA_IMMEDIATE;
logic [31:0] DATA_EXH;
logic        DATA_RD;
logic        DATA_WR;
logic        DATA_RD_EXH;
logic        DATA_WR_EXH;
logic        DATA_RD_MAIN;
logic        DATA_WR_MAIN;
logic        DATA_RDY;
logic [31:0] DATA_TO_CORE;
logic        DATA_VALID;
logic [31:0] DISPLACEMENT;
logic [31:0] DISPLACEMENT_MAIN;
logic [7:0]  DISPLACEMENT_EXH;
logic [31:0] DATA_BUFFER;
logic        DBcc_COND;
logic [31:0] DR_IN_1;
logic [31:0] DR_IN_2;
logic [31:0] DR_OUT_2;
logic [31:0] DR_OUT_1;
logic [2:0]  DR_SEL_WR_1;
logic [2:0]  DR_SEL_WR_2;
logic [2:0]  DR_SEL_RD_1;
logic [2:0]  DR_SEL_RD_2;
logic        DR_IN_USE;
logic        EW_ACK;
logic        EW_REQ_MAIN;
logic        EX_TRACE;
logic        EXEC_RDY;
logic        EXH_REQ;
logic [15:0] EXT_WORD;
logic [31:0] FAULT_ADR;
logic        FB;
logic        FC;
logic        FETCH_MEM_ADR;
logic [2:0]  FC_I;
logic [2:0]  FC_LATCH;
logic        HILOn;
logic [31:0] IBUFFER;
logic [31:0] INBUFFER;
logic        INT_TRIG;
logic [2:0]  IPL;
logic        ISP_DEC;
logic        ISP_RD;
logic        ISP_LOAD_EXH;
logic        ISP_WR_MAIN;
logic        ISP_WR;
logic [9:0]  IVECT_OFFS;
logic        IPEND_In;
logic [2:0]  IRQ_PEND;
logic        IPIPE_FILL;
logic        IPIPE_FLUSH;
logic        IPIPE_FLUSH_EXH;
logic        IPIPE_FLUSH_MAIN;
logic [2:0]  IPIPE_OFFESET;
logic        LOOP_BSY;
logic        LOOP_SPLIT;
logic        LOOP_EXIT;
int          MOVEP_PNTR;
logic        MSP_RD;
logic        MSP_WR;
logic        OPCODE_RD;
logic        OPCODE_RDY;
logic        OPCODE_VALID;
logic [15:0] OPCODE_TO_CORE;
OP_SIZETYPE  OP_SIZE;
OP_SIZETYPE  OP_SIZE_BUS;
OP_SIZETYPE  OP_SIZE_EXH;
OP_SIZETYPE  OP_SIZE_MAIN;
OP_SIZETYPE  OP_SIZE_WB; // Writeback.
logic        OPCODE_REQ;
logic        OPCODE_REQ_I;
logic        OW_VALID;
logic        OPD_ACK_MAIN;
OP_68K       OP;
OP_68K       OP_WB;
logic        OW_REQ_MAIN;
logic [31:0] OUTBUFFER;
logic [31:0] PC;
logic        PC_ADD_DISPL;
logic [7:0]  PC_ADR_OFFSET;
logic [3:0]  PC_EW_OFFSET;
logic        PC_INC;
logic        PC_INC_EXH;
logic        PC_INC_EXH_I;
logic [31:0] PC_L;
logic        PC_LOAD;
logic        PC_LOAD_EXH;
logic        PC_LOAD_MAIN;
logic [7:0]  PC_OFFSET;
logic [7:0]  PC_OFFSET_OPD;
logic        PC_RESTORE_EXH;
logic        RB;
logic        RC;
logic        RD_REQ;
logic        RD_REQ_I;
logic        RMC;
logic        REFILLn_EXH;
logic        RESTORE_ISP_PC;
logic        RESET_CPU;
logic        RESET_IN;
logic        RESET_STRB;
logic        SP_ADD_DISPL;
logic        SP_ADD_DISPL_EXH;
logic        SP_ADD_DISPL_MAIN;
logic        SBIT;
logic [8:0]  SSW_80;
logic [2:0]  SFC;
logic        SFC_RD;
logic        SFC_WR;
logic [15:0] SR_CPY;
logic        SR_RD;
logic        SR_INIT;
logic        SR_CLR_MBIT;
logic        SR_WR;
logic        SR_WR_EXH;
logic        SR_WR_MAIN;
logic [3:0]  STACK_FORMAT;
int          STACK_POS;
logic [15:0] STATUS_REG;
logic        STATUSn_MAIN;
logic        STATUSn_EXH;
logic        STORE_ADR_FORMAT;
logic        STORE_ABS_HI;
logic        STORE_ABS_LO;
logic        STORE_AEFF;
logic        STORE_D16;
logic        STORE_D32_LO;
logic        STORE_D32_HI;
logic        STORE_DISPL;
logic        STORE_MEM_ADR;
logic        STORE_OD_HI;
logic        STORE_OD_LO;
logic        STORE_IDATA_B1;
logic        STORE_IDATA_B2;
logic        TRAP_AERR;
logic        TRAP_ILLEGAL;
TRAPTYPE_OPC TRAP_CODE_OPC;
logic        TRAP_cc;
logic        TRAP_CHK;
logic        TRAP_DIVZERO;
logic        TRAP_V;
logic        UNMARK;
logic        USE_APAIR;
logic        USE_DFC;
logic        USE_SFC;
logic        USE_DPAIR;
logic        USE_DREG;
logic        USP_RD;
logic        USP_WR;
logic [31:0] VBR;
logic        VBR_WR;
logic        VBR_RD;
logic        WR_REQ;
logic        WR_REQ_I;

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
// Exception handler data multiplexer
// ========================================================================

assign DATA_EXH = (STACK_POS == 2) ? {SR_CPY, PC[31:16]} :
                   (STACK_POS == 4) ? {PC[15:0], STACK_FORMAT, 2'b00, IVECT_OFFS} :
                   (STACK_FORMAT == 4'h2 && STACK_POS == 6) ? PC :
                   (STACK_FORMAT == 4'h9 && STACK_POS == 6) ? PC :
                   (STACK_POS == 6) ? {BIW_0, FC, FB, RC, RB, 3'b000, SSW_80} : // Format A and B.
                   (STACK_POS == 8) ? {BIW_1, BIW_2} : // Format A and B.
                   (STACK_FORMAT == 4'h9 && STACK_POS == 10) ? FAULT_ADR :
                   (STACK_POS == 10) ? ADR_CPY_EXH :
                   (STACK_POS == 14) ? OUTBUFFER :
                   (STACK_POS == 20) ? PC + 32'd4 : // Stage B address.
                   (STACK_POS == 24) ? INBUFFER :
                   (STACK_POS == 28) ? {16'h0, VERSION} : 32'h0;

// ========================================================================
// Core data path routing
// ========================================================================

assign DATA_IN_EXH = BUSY_MAIN ? ALU_RESULT[31:0] : DATA_TO_CORE; // MOVEC handles the VBR.

assign DATA_FROM_CORE = BUSY_EXH ? DATA_EXH :
                         (OP_WB == CAS || OP_WB == CAS2) ? DR_OUT_2 :
                         ALU_RESULT[31:0];

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
    // CHK bounds check.
    else if ((OP == CHK2 || OP == CMP2) && USE_DREG)
        ALU_OP2_IN = DR_OUT_2;
    else if (OP == CHK || OP == CHK2 || OP == CMP2)
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
// Operand size routing
// ========================================================================

assign OP_SIZE = BUSY_EXH ? OP_SIZE_EXH : OP_SIZE_MAIN;
assign OP_SIZE_BUS = DATA_WR_MAIN ? OP_SIZE_WB : OP_SIZE;

// ========================================================================
// PC-related signals
// ========================================================================

assign PC_OFFSET = PC_OFFSET_OPD;
assign PC_L = PC + PC_ADR_OFFSET;
assign PC_INC_EXH_I = !LOOP_SPLIT ? PC_INC_EXH : 1'b0; // Suppress for a split loop.
assign PC_LOAD = PC_LOAD_EXH || PC_LOAD_MAIN;

// ========================================================================
// Address path control
// ========================================================================

assign ADR_MODE = BUSY_EXH ? 3'b010 : ADR_MODE_MAIN; // (ISP) during exception.
assign SP_ADD_DISPL = SP_ADD_DISPL_MAIN || SP_ADD_DISPL_EXH;
assign AR_SEL_RD_1 = BUSY_EXH ? 3'b111 : AR_SEL_RD_1_MAIN; // ISP during exception.

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

// ========================================================================
// Condition and branch prediction
// ========================================================================

assign DBcc_COND = (OP_WB == DBcc && ALU_RESULT[15:0] == 16'hFFFF);

// Predict a branch if an SR-modifying instruction will change the CPU space.
always_comb begin : branch_prediction
    BRANCH_ATN = 1'b0;
    if (OP == ANDI_TO_SR && !DATA_IMMEDIATE[13] && STATUS_REG[13])
        BRANCH_ATN = 1'b1;
    else if (OP == ANDI_TO_SR && !DATA_IMMEDIATE[12] && STATUS_REG[12])
        BRANCH_ATN = 1'b1;
    else if (OP == EORI_TO_SR && DATA_IMMEDIATE[13])
        BRANCH_ATN = 1'b1;
    else if (OP == EORI_TO_SR && DATA_IMMEDIATE[12])
        BRANCH_ATN = 1'b1;
    else if (OP == ORI_TO_SR && DATA_IMMEDIATE[13] && !STATUS_REG[13])
        BRANCH_ATN = 1'b1;
    else if (OP == ORI_TO_SR && DATA_IMMEDIATE[12] && !STATUS_REG[12])
        BRANCH_ATN = 1'b1;
    else if (OP == MOVE_TO_SR && BIW_0[5:3] == ADR_DN && DR_OUT_1[13:12] != STATUS_REG[13:12])
        BRANCH_ATN = 1'b1;
    else if (OP == MOVE_TO_SR && BIW_0[5:0] == 6'b111100 && DATA_IMMEDIATE[13:12] != STATUS_REG[13:12])
        BRANCH_ATN = 1'b1;
    else if (OP == MOVE_TO_SR && DATA_TO_CORE[13:12] != STATUS_REG[13:12])
        BRANCH_ATN = 1'b1;
end

// ========================================================================
// Bus request arbitration
// ========================================================================

assign DATA_RD = DATA_RD_EXH || DATA_RD_MAIN;
assign DATA_WR = DATA_WR_EXH || DATA_WR_MAIN;

always_ff @(posedge CLK) begin : bus_req_latch
    // Flip-flops break combinatorial loops between core requests and bus controller.
    // Requests are valid until the bus controller enters START_CYCLE and asserts BUS_BSY.
    if (!BUS_BSY) begin
        RD_REQ_I <= DATA_RD;
        WR_REQ_I <= DATA_WR;
        OPCODE_REQ_I <= OPCODE_RD;
    end else if (BUS_BSY) begin
        RD_REQ_I <= 1'b0;
        WR_REQ_I <= 1'b0;
        OPCODE_REQ_I <= 1'b0;
    end
end

assign RD_REQ = !BUS_BSY ? DATA_RD : RD_REQ_I;
assign WR_REQ = !BUS_BSY ? DATA_WR : WR_REQ_I;
assign OPCODE_REQ = !BUS_BSY ? OPCODE_RD : OPCODE_REQ_I;

// ========================================================================
// Miscellaneous signal multiplexing
// ========================================================================

assign DISPLACEMENT = BUSY_MAIN ? DISPLACEMENT_MAIN : {24'h0, DISPLACEMENT_EXH};
assign SR_WR = SR_WR_EXH || SR_WR_MAIN;
assign IPIPE_FLUSH = IPIPE_FLUSH_EXH || IPIPE_FLUSH_MAIN;
assign ISP_WR = ISP_WR_MAIN || ISP_LOAD_EXH;
assign AVECn_BUSIF = BUSY_EXH ? AVECn : 1'b1;

assign CPU_SPACE = (OP == BKPT && DATA_RD_MAIN) ? 1'b1 :
                    BUSY_EXH ? CPU_SPACE_EXH : 1'b0;

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
// Trap and function code signals
// ========================================================================

assign TRAP_AERR = !BUSY_EXH ? AERR : 1'b0; // No address error from the system during exception processing.
assign USE_DFC = (OP_WB == MOVES && DATA_WR_MAIN);
assign USE_SFC = (OP_WB == MOVES && DATA_RD_MAIN);

// ========================================================================
// Input synchronization
// ========================================================================

assign RESET_IN = ~RESET_INn;
assign IPL = ~IPLn;

// ========================================================================
// Status output (active on negedge CLK)
// ========================================================================

always_ff @(negedge CLK) begin : refill_status
    STATUSn <= !(STATUSn_EXH && STATUSn_MAIN) ? 1'b0 : 1'b1;
    REFILLn <= REFILLn_EXH;
end

// ========================================================================
// Status register and address bus
// ========================================================================

assign SBIT = STATUS_REG[13];

assign ADR_L = BKPT_CYCLE ? {24'h0, 3'b000, BIW_0[2:0], 2'b00} :
                CPU_SPACE_EXH ? {28'hFFFFFFF, IRQ_PEND, 1'b1} :
                DATA_WR_MAIN ? ADR_EFF_WB : ADR_EFF;

assign ADR_P = BUS_BSY ? ADR_LATCH :
                (DATA_RD || DATA_WR) ? ADR_L : PC_L;

// Address and fault address latches.
always_ff @(posedge CLK) begin : adr_latches
    if (!BUS_BSY) begin
        ADR_LATCH <= ADR_P;
        FC_LATCH <= FC_I;
    end else if (!BERRn) begin
        FAULT_ADR <= ADR_LATCH;
    end
end

// Function code generation.
always_comb begin : fc_generation
    if (BUS_BSY)
        FC_I = FC_LATCH;
    else if (USE_SFC)
        FC_I = SFC;
    else if (USE_DFC)
        FC_I = DFC;
    else if ((DATA_RD || DATA_WR) && CPU_SPACE)
        FC_I = FC_CPU_SPACE;
    else if ((DATA_RD || DATA_WR) && SBIT)
        FC_I = FC_SUPER_DATA;
    else if (DATA_RD || DATA_WR)
        FC_I = FC_USER_DATA;
    else if (OPCODE_RD && SBIT)
        FC_I = FC_SUPER_PROG;
    else
        FC_I = FC_USER_PROG;
end

// ========================================================================
// Submodule instantiations
// ========================================================================

    WF68K30L_ADDRESS_REGISTERS I_ADDRESSREGISTERS (
        .CLK                    (CLK),
        .RESET                  (RESET_CPU),
        .AR_IN_1                (AR_IN_1),
        .AR_IN_2                (AR_IN_2),
        .AR_OUT_1               (AR_OUT_1),
        .AR_OUT_2               (AR_OUT_2),
        .INDEX_IN               (DR_OUT_1), // From data register section.
        .PC                     (PC),
        .FETCH_MEM_ADR          (FETCH_MEM_ADR),
        .STORE_ADR_FORMAT       (STORE_ADR_FORMAT),
        .STORE_ABS_HI           (STORE_ABS_HI),
        .STORE_ABS_LO           (STORE_ABS_LO),
        .STORE_D16              (STORE_D16),
        .STORE_D32_LO           (STORE_D32_LO),
        .STORE_D32_HI           (STORE_D32_HI),
        .STORE_DISPL            (STORE_DISPL),
        .STORE_MEM_ADR          (STORE_MEM_ADR),
        .STORE_OD_HI            (STORE_OD_HI),
        .STORE_OD_LO            (STORE_OD_LO),
        .STORE_AEFF             (STORE_AEFF),
        .OP_SIZE                (OP_SIZE),
        .AR_MARK_USED           (AR_MARK_USED),
        .USE_APAIR              (USE_APAIR),
        .AR_IN_USE              (AR_IN_USE),
        .AR_SEL_RD_1            (AR_SEL_RD_1),
        .AR_SEL_RD_2            (AR_SEL_RD_2),
        .AR_SEL_WR_1            (AR_SEL_WR_1),
        .AR_SEL_WR_2            (AR_SEL_WR_2),
        .ADR_OFFSET             (ADR_OFFSET), // Byte aligned.
        .ADR_MARK_USED          (ADR_MARK_USED),
        .ADR_IN_USE             (ADR_IN_USE),
        .ADR_MODE               (ADR_MODE),
        .AMODE_SEL              (AMODE_SEL),
        .USE_DREG               (USE_DREG),
        .ADR_EFF                (ADR_EFF),
        .ADR_EFF_WB             (ADR_EFF_WB),
        .DFC                    (DFC),
        .DFC_WR                 (DFC_WR),
        .SFC                    (SFC),
        .SFC_WR                 (SFC_WR),
        .ISP_DEC                (ISP_DEC),
        .ISP_RD                 (ISP_RD),
        .ISP_WR                 (ISP_WR),
        .MSP_RD                 (MSP_RD),
        .MSP_WR                 (MSP_WR),
        .USP_RD                 (USP_RD),
        .USP_WR                 (USP_WR),
        .AR_DEC                 (AR_DEC),
        .AR_INC                 (AR_INC),
        .AR_WR_1                (AR_WR_1),
        .AR_WR_2                (AR_WR_2),
        .UNMARK                 (UNMARK),
        .EXT_WORD               (EXT_WORD),
        .MBIT                   (STATUS_REG[12]),
        .SBIT                   (SBIT),
        .SP_ADD_DISPL           (SP_ADD_DISPL),
        .RESTORE_ISP_PC         (RESTORE_ISP_PC),
        .DISPLACEMENT           (DISPLACEMENT),
        .PC_ADD_DISPL           (PC_ADD_DISPL),
        .PC_EW_OFFSET           (PC_EW_OFFSET),
        .PC_INC                 (PC_INC),
        .PC_LOAD                (PC_LOAD),
        .PC_RESTORE             (PC_RESTORE_EXH),
        .PC_OFFSET              (PC_OFFSET)
    );

    WF68K30L_ALU I_ALU (
        .CLK                    (CLK),
        .RESET                  (RESET_CPU),
        .LOAD_OP2               (ALU_LOAD_OP2),
        .LOAD_OP3               (ALU_LOAD_OP3),
        .LOAD_OP1               (ALU_LOAD_OP1),
        .OP1_IN                 (ALU_OP1_IN),
        .OP2_IN                 (ALU_OP2_IN),
        .OP3_IN                 (ALU_OP3_IN),
        .BF_OFFSET_IN           (BF_OFFSET),
        .BF_WIDTH_IN            (BF_WIDTH),
        .BITPOS_IN              (BITPOS),
        .RESULT                 (ALU_RESULT),
        .ADR_MODE_IN            (ADR_MODE),
        .USE_DREG               (USE_DREG),
        .HILOn                  (HILOn),
        .OP_SIZE_IN             (OP_SIZE),
        .OP_IN                  (OP),
        .OP_WB                  (OP_WB),
        .BIW_0_IN               (BIW_0[11:0]),
        .BIW_1_IN               (BIW_1),
        .SR_WR                  (SR_WR),
        .SR_INIT                (SR_INIT),
        .SR_CLR_MBIT            (SR_CLR_MBIT),
        .CC_UPDT                (CC_UPDT),
        .STATUS_REG_OUT         (STATUS_REG),
        .ALU_COND               (ALU_COND),
        .ALU_INIT               (ALU_INIT),
        .ALU_BSY                (ALU_BSY),
        .ALU_REQ                (ALU_REQ),
        .ALU_ACK                (ALU_ACK),
        .IRQ_PEND               (IRQ_PEND),
        .TRAP_CHK               (TRAP_CHK),
        .TRAP_DIVZERO           (TRAP_DIVZERO)
    );

    WF68K30L_BUS_INTERFACE I_BUS_IF (
        .CLK                (CLK),

        .ADR_IN_P           (ADR_P),
        .ADR_OUT_P          (ADR_OUT),

        .FC_IN              (FC_I),
        .FC_OUT             (FC_OUT),

        .DATA_PORT_IN       (DATA_IN),
        .DATA_PORT_OUT      (DATA_OUT),
        .DATA_FROM_CORE     (DATA_FROM_CORE),
        .DATA_TO_CORE       (DATA_TO_CORE),
        .OPCODE_TO_CORE     (OPCODE_TO_CORE),

        .DATA_PORT_EN       (DATA_EN),
        .BUS_EN             (BUS_EN),

        .SIZE               (SIZE),
        .OP_SIZE            (OP_SIZE_BUS),

        .RD_REQ             (RD_REQ),
        .WR_REQ             (WR_REQ),
        .DATA_RDY           (DATA_RDY),
        .DATA_VALID         (DATA_VALID),
        .OPCODE_REQ         (OPCODE_REQ),
        .OPCODE_RDY         (OPCODE_RDY),
        .OPCODE_VALID       (OPCODE_VALID),
        .RMC                (RMC),
        .BUSY_EXH           (BUSY_EXH),
        .SSW_80             (SSW_80),
        .INBUFFER           (INBUFFER),
        .OUTBUFFER          (OUTBUFFER),

        .DSACKn             (DSACKn),
        .ASn                (ASn),
        .DSn                (DSn),
        .RWn                (RWn),
        .RMCn               (RMCn),
        .ECSn               (ECSn),
        .OCSn               (OCSn),
        .DBENn              (DBENn),

        .STERMn             (STERMn),

        .BRn                (BRn),
        .BGACKn             (BGACKn),
        .BGn                (BGn),

        .RESET_STRB         (RESET_STRB),
        .RESET_IN           (RESET_IN),
        .RESET_OUT          (RESET_OUT),
        .RESET_CPU          (RESET_CPU),

        .AVECn              (AVECn_BUSIF),
        .HALTn              (HALT_INn),
        .BERRn              (BERRn),
        .AERR               (AERR),

        .BUS_BSY            (BUS_BSY)
    );

    WF68K30L_CONTROL #(
        .NO_PIPELINE(NO_PIPELINE)
    ) I_CONTROL (
        .CLK                    (CLK),
        .RESET_CPU              (RESET_CPU),
        .BUSY                   (BUSY_MAIN),
        .BUSY_EXH               (BUSY_EXH),
        .EXH_REQ                (EXH_REQ),
        .INT_TRIG               (INT_TRIG),
        .OW_REQ                 (OW_REQ_MAIN),
        .OW_VALID               (OW_VALID),
        .EW_REQ                 (EW_REQ_MAIN),
        .EW_ACK                 (EW_ACK),
        .OPD_ACK                (OPD_ACK_MAIN),
        .ADR_MARK_USED          (ADR_MARK_USED),
        .ADR_IN_USE             (ADR_IN_USE),
        .ADR_OFFSET             (ADR_OFFSET_MAIN),
        .DATA_RD                (DATA_RD_MAIN),
        .DATA_WR                (DATA_WR_MAIN),
        .DATA_RDY               (DATA_RDY),
        .DATA_VALID             (DATA_VALID),
        .RMC                    (RMC),
        .FETCH_MEM_ADR          (FETCH_MEM_ADR),
        .LOAD_OP1               (ALU_LOAD_OP1),
        .LOAD_OP2               (ALU_LOAD_OP2),
        .LOAD_OP3               (ALU_LOAD_OP3),
        .STORE_ADR_FORMAT       (STORE_ADR_FORMAT),
        .STORE_ABS_HI           (STORE_ABS_HI),
        .STORE_ABS_LO           (STORE_ABS_LO),
        .STORE_D16              (STORE_D16),
        .STORE_D32_LO           (STORE_D32_LO),
        .STORE_D32_HI           (STORE_D32_HI),
        .STORE_DISPL            (STORE_DISPL),
        .STORE_MEM_ADR          (STORE_MEM_ADR),
        .STORE_OD_HI            (STORE_OD_HI),
        .STORE_OD_LO            (STORE_OD_LO),
        .STORE_AEFF             (STORE_AEFF),
        .STORE_IDATA_B1         (STORE_IDATA_B1),
        .STORE_IDATA_B2         (STORE_IDATA_B2),
        .OP                     (OP),
        .OP_SIZE                (OP_SIZE_MAIN),
        .BIW_0                  (BIW_0[13:0]),
        .BIW_1                  (BIW_1),
        .BIW_2                  (BIW_2),
        .EXT_WORD               (EXT_WORD),
        .ADR_MODE               (ADR_MODE_MAIN),
        .AMODE_SEL              (AMODE_SEL),
        .USE_DREG               (USE_DREG),
        .HILOn                  (HILOn),
        .OP_WB                  (OP_WB),
        .OP_SIZE_WB             (OP_SIZE_WB),
        .BIW_0_WB_73            (BIW_0_WB_73),
        .AR_MARK_USED           (AR_MARK_USED),
        .AR_IN_USE              (AR_IN_USE),
        .AR_SEL_RD_1            (AR_SEL_RD_1_MAIN),
        .AR_SEL_RD_2            (AR_SEL_RD_2),
        .AR_SEL_WR_1            (AR_SEL_WR_1),
        .AR_SEL_WR_2            (AR_SEL_WR_2),
        .AR_INC                 (AR_INC),
        .AR_DEC                 (AR_DEC),
        .AR_WR_1                (AR_WR_1),
        .AR_WR_2                (AR_WR_2),
        .DR_MARK_USED           (DR_MARK_USED),
        .USE_APAIR              (USE_APAIR),
        .USE_DPAIR              (USE_DPAIR),
        .DR_IN_USE              (DR_IN_USE),
        .DR_SEL_WR_1            (DR_SEL_WR_1),
        .DR_SEL_WR_2            (DR_SEL_WR_2),
        .DR_SEL_RD_1            (DR_SEL_RD_1),
        .DR_SEL_RD_2            (DR_SEL_RD_2),
        .DR_WR_1                (DR_WR_1),
        .DR_WR_2                (DR_WR_2),
        .UNMARK                 (UNMARK),
        .DISPLACEMENT           (DISPLACEMENT_MAIN),
        .PC_ADD_DISPL           (PC_ADD_DISPL),
        .PC_LOAD                (PC_LOAD_MAIN),
        .PC_INC_EXH             (PC_INC_EXH),
        .SP_ADD_DISPL           (SP_ADD_DISPL_MAIN),
        .DFC_RD                 (DFC_RD),
        .DFC_WR                 (DFC_WR),
        .SFC_RD                 (SFC_RD),
        .SFC_WR                 (SFC_WR),
        .VBR_RD                 (VBR_RD),
        .VBR_WR                 (VBR_WR),
        .ISP_RD                 (ISP_RD),
        .ISP_WR                 (ISP_WR_MAIN),
        .MSP_RD                 (MSP_RD),
        .MSP_WR                 (MSP_WR),
        .USP_RD                 (USP_RD),
        .USP_WR                 (USP_WR),
        .IPIPE_FLUSH            (IPIPE_FLUSH_MAIN),
        .ALU_INIT               (ALU_INIT),
        .ALU_BSY                (ALU_BSY),
        .ALU_REQ                (ALU_REQ),
        .ALU_ACK                (ALU_ACK),
        .BKPT_CYCLE             (BKPT_CYCLE),
        .BKPT_INSERT            (BKPT_INSERT),
        .LOOP_BSY               (LOOP_BSY),
        .LOOP_SPLIT             (LOOP_SPLIT),
        .LOOP_EXIT              (LOOP_EXIT),
        .BF_OFFSET              (BF_OFFSET[2:0]),
        .BF_WIDTH               (BF_WIDTH),
        .SR_WR                  (SR_WR_MAIN),
        .MOVEM_ADn              (ADn),
        .MOVEP_PNTR             (MOVEP_PNTR),
        .CC_UPDT                (CC_UPDT),
        .TRACE_MODE             (STATUS_REG[15:14]),
        .VBIT                   (STATUS_REG[1]),
        .ALU_COND               (ALU_COND),
        .DBcc_COND              (DBcc_COND),
        .BRANCH_ATN             (BRANCH_ATN),
        .RESET_STRB             (RESET_STRB),
        .BERR                   (BERR_MAIN),
        .STATUSn                (STATUSn_MAIN),
        .EX_TRACE               (EX_TRACE),
        .TRAP_cc                (TRAP_cc),
        .TRAP_V                 (TRAP_V),
        .TRAP_ILLEGAL           (TRAP_ILLEGAL)
    );

    WF68K30L_DATA_REGISTERS I_DATA_REGISTERS (
        .CLK                    (CLK),
        .RESET                  (RESET_CPU),
        .DR_IN_1                (DR_IN_1),
        .DR_IN_2                (DR_IN_2),
        .DR_OUT_2               (DR_OUT_2),
        .DR_OUT_1               (DR_OUT_1),
        .DR_SEL_WR_1            (DR_SEL_WR_1),
        .DR_SEL_WR_2            (DR_SEL_WR_2),
        .DR_SEL_RD_1            (DR_SEL_RD_1),
        .DR_SEL_RD_2            (DR_SEL_RD_2),
        .DR_WR_1                (DR_WR_1),
        .DR_WR_2                (DR_WR_2),
        .DR_MARK_USED           (DR_MARK_USED),
        .USE_DPAIR              (USE_DPAIR),
        .DR_IN_USE              (DR_IN_USE),
        .UNMARK                 (UNMARK),
        .OP_SIZE                (OP_SIZE_WB)
    );

    WF68K30L_EXCEPTION_HANDLER #(
        .VERSION(VERSION)
    ) I_EXC_HANDLER (
        .CLK                    (CLK),

        .RESET                  (RESET_CPU),
        .BUSY_MAIN              (BUSY_MAIN),
        .BUSY_OPD               (BUSY_OPD),

        .EXH_REQ                (EXH_REQ),
        .BUSY_EXH               (BUSY_EXH),

        .ADR_IN                 (ADR_EFF),
        .ADR_CPY                (ADR_CPY_EXH),
        .ADR_OFFSET             (ADR_OFFSET_EXH),
        .CPU_SPACE              (CPU_SPACE_EXH),

        .DATA_0                 (DATA_TO_CORE[0]),
        .DATA_RD                (DATA_RD_EXH),
        .DATA_WR                (DATA_WR_EXH),
        .DATA_IN                (DATA_IN_EXH),

        .OP_SIZE                (OP_SIZE_EXH),
        .DATA_RDY               (DATA_RDY),
        .DATA_VALID             (DATA_VALID),

        .OPCODE_RDY             (OPCODE_RDY),
        .OPD_ACK                (OPD_ACK_MAIN),
        .OW_VALID               (OW_VALID),

        .STATUS_REG_IN          (STATUS_REG),
        .SR_CPY                 (SR_CPY),
        .SR_INIT                (SR_INIT),
        .SR_CLR_MBIT            (SR_CLR_MBIT),

        .SR_WR                  (SR_WR_EXH),
        .ISP_DEC                (ISP_DEC),
        .ISP_LOAD               (ISP_LOAD_EXH),
        .PC_LOAD                (PC_LOAD_EXH),
        .PC_INC                 (PC_INC_EXH),
        .PC_RESTORE             (PC_RESTORE_EXH),

        .STACK_FORMAT           (STACK_FORMAT),
        .STACK_POS              (STACK_POS),

        .SP_ADD_DISPL           (SP_ADD_DISPL_EXH),
        .DISPLACEMENT           (DISPLACEMENT_EXH),
        .IPIPE_FILL             (IPIPE_FILL),
        .IPIPE_FLUSH            (IPIPE_FLUSH_EXH),
        .REFILLn                (REFILLn_EXH),
        .RESTORE_ISP_PC         (RESTORE_ISP_PC),

        .HALT_OUTn              (HALT_OUTn),
        .STATUSn                (STATUSn_EXH),

        .INT_TRIG               (INT_TRIG),
        .IRQ_IN                 (IPL),
        .IRQ_PEND               (IRQ_PEND),
        .AVECn                  (AVECn),
        .IPENDn                 (IPENDn),
        .IVECT_OFFS             (IVECT_OFFS),

        .TRAP_AERR              (TRAP_AERR),
        .TRAP_BERR              (BERR_MAIN),
        .TRAP_CHK               (TRAP_CHK),
        .TRAP_DIVZERO           (TRAP_DIVZERO),
        .TRAP_ILLEGAL           (TRAP_ILLEGAL),
        .TRAP_CODE_OPC          (TRAP_CODE_OPC),
        .TRAP_VECTOR            (BIW_0[3:0]),
        .TRAP_cc                (TRAP_cc),
        .TRAP_V                 (TRAP_V),
        .EX_TRACE_IN            (EX_TRACE),
        .VBR_WR                 (VBR_WR),
        .VBR                    (VBR)
    );

    WF68K30L_OPCODE_DECODER #(
        .NO_LOOP(NO_LOOP)
    ) I_OPCODE_DECODER (
        .CLK                    (CLK),

        .OW_REQ_MAIN            (OW_REQ_MAIN),
        .EW_REQ_MAIN            (EW_REQ_MAIN),

        .EXH_REQ                (EXH_REQ),
        .BUSY_EXH               (BUSY_EXH),
        .BUSY_MAIN              (BUSY_MAIN),
        .BUSY_OPD               (BUSY_OPD),

        .BKPT_INSERT            (BKPT_INSERT),
        .BKPT_DATA              (DATA_TO_CORE[15:0]),

        .LOOP_EXIT              (LOOP_EXIT),
        .LOOP_BSY               (LOOP_BSY),

        .OPD_ACK_MAIN           (OPD_ACK_MAIN),
        .EW_ACK                 (EW_ACK),

        .PC_INC                 (PC_INC),
        .PC_INC_EXH             (PC_INC_EXH_I),
        .PC_ADR_OFFSET          (PC_ADR_OFFSET),
        .PC_EW_OFFSET           (PC_EW_OFFSET),
        .PC_OFFSET              (PC_OFFSET_OPD),

        .OPCODE_RD              (OPCODE_RD),
        .OPCODE_RDY             (OPCODE_RDY),
        .OPCODE_VALID           (OPCODE_VALID),
        .OPCODE_DATA            (OPCODE_TO_CORE),

        .IPIPE_FILL             (IPIPE_FILL),
        .IPIPE_FLUSH            (IPIPE_FLUSH),

        // Fault logic:
        .OW_VALID               (OW_VALID),
        .RC                     (RC),
        .RB                     (RB),
        .FC                     (FC),
        .FB                     (FB),

        // Trap logic:
        .SBIT                   (SBIT),
        .TRAP_CODE              (TRAP_CODE_OPC),

        // System control:
        .OP                     (OP),
        .BIW_0                  (BIW_0),
        .BIW_1                  (BIW_1),
        .BIW_2                  (BIW_2),
        .EXT_WORD               (EXT_WORD)
    );
endmodule
