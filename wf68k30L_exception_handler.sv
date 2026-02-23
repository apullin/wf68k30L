// ------------------------------------------------------------------------
// --                                                                    --
// -- WF68K30L IP Core: this is the exception handler module.            --
// --                                                                    --
// -- Description:                                                       --
// -- This is the exception handler which is responsible for the         --
// -- interrupt management of the external interrupt and internal        --
// -- exception processing. It manages auto-vectored interrupt           --
// -- cycles, priority resolving and correct vector numbers.             --
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
// Revision 2K16A 20141201 WF
//   Fixed a bug in PC_LOAD.
// Revision 2K18A 20180620 WF
//   Various fixes.
//

module WF68K30L_EXCEPTION_HANDLER #(
    parameter [15:0] VERSION = 16'h1409
)(
    input  logic        CLK,
    input  logic        RESET,

    input  logic        BUSY_MAIN,
    input  logic        BUSY_OPD,

    output logic        EXH_REQ,
    output logic        BUSY_EXH,

    input  logic [31:0] ADR_IN,
    output logic [31:0] ADR_CPY,
    output logic [31:0] ADR_OFFSET,
    output logic        CPU_SPACE,

    input  logic        DATA_0,
    output logic        DATA_RD,
    output logic        DATA_WR,
    input  logic [31:0] DATA_IN,

    output logic [1:0]  OP_SIZE,        // Operand size.
    input  logic        DATA_RDY,
    input  logic        DATA_VALID,

    input  logic        OPCODE_RDY,     // OPCODE is available.
    input  logic        OPD_ACK,        // Opword is available.
    input  logic        OW_VALID,       // Status from the opcode decoder.

    input  logic [15:0] STATUS_REG_IN,
    output logic [15:0] SR_CPY,
    output logic        SR_INIT,
    output logic        SR_CLR_MBIT,
    output logic        SR_WR,

    output logic        ISP_DEC,
    output logic        ISP_LOAD,
    output logic        PC_INC,
    output logic        PC_LOAD,
    output logic        PC_RESTORE,

    output logic [3:0]  STACK_FORMAT,
    output integer      STACK_POS,

    output logic        SP_ADD_DISPL,
    output logic [7:0]  DISPLACEMENT,

    output logic        IPIPE_FILL,
    output logic        IPIPE_FLUSH,
    output logic        REFILLn,
    output logic        RESTORE_ISP_PC,

    output logic        HALT_OUTn,
    output logic        STATUSn,

    // Interrupt controls:
    input  logic        INT_TRIG,
    input  logic [2:0]  IRQ_IN,
    output logic [2:0]  IRQ_PEND,
    input  logic        AVECn,
    output logic        IPENDn,
    output logic [9:0]  IVECT_OFFS,     // Interrupt vector offset.

    // Trap signals:
    input  logic        TRAP_AERR,
    input  logic        TRAP_BERR,
    input  logic        TRAP_CHK,
    input  logic        TRAP_DIVZERO,
    input  logic        TRAP_ILLEGAL,
    input  logic [2:0]  TRAP_CODE_OPC,  // T_1010, T_1111, T_ILLEGAL, T_TRAP, T_PRIV.
    input  logic [3:0]  TRAP_VECTOR,
    input  logic        TRAP_cc,
    input  logic        TRAP_V,
    input  logic        EX_TRACE_IN,
    input  logic        VBR_WR,
    output logic [31:0] VBR
);

`include "wf68k30L_pkg.svh"

typedef enum logic [4:0] {
    EXS_IDLE            = 5'd0,
    EXS_BUILD_STACK     = 5'd1,
    EXS_BUILD_TSTACK    = 5'd2,
    EXS_CALC_VECT_NO    = 5'd3,
    EXS_EXAMINE_VERSION = 5'd4,
    EXS_GET_VECTOR      = 5'd5,
    EXS_HALTED          = 5'd6,
    EXS_INIT            = 5'd7,
    EXS_READ_BOTTOM     = 5'd8,
    EXS_READ_TOP        = 5'd9,
    EXS_REFILL_PIPE     = 5'd10,
    EXS_RESTORE_ISP     = 5'd11,
    EXS_RESTORE_PC      = 5'd12,
    EXS_RESTORE_STATUS  = 5'd13,
    EXS_UPDATE_PC       = 5'd14,
    EXS_SWITCH_STATE    = 5'd15,
    EXS_VALIDATE_FRAME  = 5'd16
} EX_STATES;

typedef enum logic [4:0] {
    EX_NONE     = 5'd0,
    EX_1010     = 5'd1,
    EX_1111     = 5'd2,
    EX_AERR     = 5'd3,
    EX_BERR     = 5'd4,
    EX_CHK      = 5'd5,
    EX_DIVZERO  = 5'd6,
    EX_FORMAT   = 5'd7,
    EX_ILLEGAL  = 5'd8,
    EX_INT      = 5'd9,
    EX_PRIV     = 5'd10,
    EX_RESET_EX = 5'd11,
    EX_RTE      = 5'd12,
    EX_TRACE    = 5'd13,
    EX_TRAP     = 5'd14,
    EX_TRAPcc   = 5'd15,
    EX_TRAPV    = 5'd16
} EXCEPTIONS;

logic        ACCESS_ERR;
logic        AVEC;
logic        DATA_RD_I;
logic        DATA_WR_I;
logic        DOUBLE_BUSFLT;
EXCEPTIONS   EXCEPTION;     // Currently executed exception.
EX_STATES    EX_STATE;
EX_STATES    NEXT_EX_STATE;
logic        EX_P_1010;
logic        EX_P_1111;
logic        EX_P_AERR;
logic        EX_P_BERR;
logic        EX_P_CHK;
logic        EX_P_DIVZERO;
logic        EX_P_FORMAT;
logic        EX_P_ILLEGAL;
logic        EX_P_INT;
logic        EX_P_RESET;
logic        EX_P_RTE;
logic        EX_P_PRIV;
logic        EX_P_TRACE;
logic        EX_P_TRAP;
logic        EX_P_TRAPcc;
logic        EX_P_TRAPV;
logic [31:0] INT_VECT;      // Interrupt vector.
logic        IBOUND;
logic [2:0]  IRQ;
logic [2:0]  IRQ_PEND_I;
logic        MBIT;
logic [1:0]  PIPE_CNT;
logic        PIPE_FULL;
integer      STACK_CNT;
logic [3:0]  STACK_FORMAT_I;
logic        SYS_INIT;

assign BUSY_EXH = (EX_STATE != EXS_IDLE) ? 1'b1 : 1'b0;

// IRQ_FILTER - negedge CLK
always_ff @(negedge CLK) begin : IRQ_FILTER
    reg [2:0] IRQ_TMP_1;
    reg [2:0] IRQ_TMP_2;
    if (IRQ_TMP_1 == IRQ_TMP_2) begin
        IRQ <= IRQ_TMP_2;
    end
    IRQ_TMP_2 = IRQ_TMP_1;
    IRQ_TMP_1 = IRQ_IN;
end

// AVEC_FILTER
always_ff @(posedge CLK) begin : AVEC_FILTER
    if (AVECn == 1'b0) begin
        AVEC <= 1'b1;
    end else if (DATA_RDY == 1'b1 || RESET == 1'b1) begin
        AVEC <= 1'b0;
    end
end

// INSTRUCTION_BOUNDARY
always_ff @(posedge CLK) begin : INSTRUCTION_BOUNDARY
    if (RESET == 1'b1) begin
        IBOUND <= 1'b0;
    end else if (OPD_ACK == 1'b1 && OW_VALID == 1'b0) begin
        IBOUND <= 1'b1;
    end else if (EX_STATE == EXS_BUILD_STACK) begin
        IBOUND <= 1'b0;
    end
end

// PENDING
always_ff @(posedge CLK) begin : PENDING
    reg INT7_TRIG;
    reg [2:0] INT_VAR;
    reg [2:0] SR_VAR;

    if (RESET == 1'b1) begin
        EX_P_RESET <= 1'b1;
    end else if (EX_STATE == EXS_RESTORE_PC && DATA_RDY == 1'b1 && EXCEPTION == EX_RESET_EX) begin
        EX_P_RESET <= 1'b0;
    end
    //
    if (TRAP_BERR == 1'b1) begin
        EX_P_BERR <= 1'b1;
    end else if (EX_STATE != EXS_IDLE && DATA_RDY == 1'b1 && DATA_VALID == 1'b0) begin
        EX_P_BERR <= 1'b1;
    end else if (EX_STATE == EXS_INIT && EXCEPTION == EX_BERR) begin
        EX_P_BERR <= 1'b0; // Reset in the beginning to enable retriggering.
    end else if (SYS_INIT == 1'b1) begin
        EX_P_BERR <= 1'b0;
    end
    //
    if (TRAP_AERR == 1'b1) begin
        EX_P_AERR <= 1'b1;
    end else if (EX_STATE == EXS_BUILD_STACK && EXCEPTION == EX_AERR) begin
        EX_P_AERR <= 1'b0;
    end else if (SYS_INIT == 1'b1) begin
        EX_P_AERR <= 1'b0;
    end
    //
    if (EX_TRACE_IN == 1'b1) begin
        EX_P_TRACE <= 1'b1;
    end else if (EX_STATE == EXS_BUILD_STACK && EXCEPTION == EX_TRACE) begin
        EX_P_TRACE <= 1'b0;
    end else if (SYS_INIT == 1'b1) begin
        EX_P_TRACE <= 1'b0;
    end
    //
    if (IRQ == 3'b111 && SR_VAR == 3'b111 && STATUS_REG_IN[10:8] != 3'b111) begin
        INT7_TRIG = 1'b1; // Trigger by lowering the mask from 7 to any value.
    end else if (IRQ == 3'b111 && INT_VAR < 3'b111) begin
        INT7_TRIG = 1'b1; // Trigger when level 7 is entered.
    end else begin
        INT7_TRIG = 1'b0;
    end
    //
    SR_VAR = STATUS_REG_IN[10:8]; // Update after use!
    INT_VAR = IRQ; // Update after use!
    //
    if (SYS_INIT == 1'b1) begin // Reset when disabling the interrupts.
        EX_P_INT <= 1'b0;
        IRQ_PEND_I <= 3'b111; // This is required for system startup.
    end else if (EX_STATE == EXS_GET_VECTOR && DATA_RDY == 1'b1) begin
        EX_P_INT <= 1'b0;
    end else if (INT7_TRIG == 1'b1) begin // Level 7 is nonmaskable ...
        EX_P_INT <= 1'b1;
        IRQ_PEND_I <= IRQ;
    end else if (INT_TRIG == 1'b1 && STATUS_REG_IN[10:8] < IRQ) begin
        EX_P_INT <= 1'b1;
        IRQ_PEND_I <= IRQ;
    end
    //
    // The following nine traps never appear at the same time:
    if (TRAP_CHK == 1'b1) begin
        EX_P_CHK <= 1'b1;
    end else if (TRAP_DIVZERO == 1'b1) begin
        EX_P_DIVZERO <= 1'b1;
    end else if (TRAP_CODE_OPC == T_TRAP) begin
        EX_P_TRAP <= 1'b1;
    end else if (TRAP_cc == 1'b1) begin
        EX_P_TRAPcc <= 1'b1;
    end else if (TRAP_V == 1'b1) begin
        EX_P_TRAPV <= 1'b1;
    end else if (TRAP_CODE_OPC == T_PRIV) begin
        EX_P_PRIV <= 1'b1;
    end else if (TRAP_CODE_OPC == T_1010) begin
        EX_P_1010 <= 1'b1;
    end else if (TRAP_CODE_OPC == T_1111) begin
        EX_P_1111 <= 1'b1;
    end else if (TRAP_CODE_OPC == T_ILLEGAL) begin
        EX_P_ILLEGAL <= 1'b1;
    end else if (TRAP_ILLEGAL == 1'b1) begin // Used for BKPT.
        EX_P_ILLEGAL <= 1'b1;
    end else if (EX_STATE == EXS_VALIDATE_FRAME && DATA_RDY == 1'b1 && DATA_VALID == 1'b1 && NEXT_EX_STATE == EXS_IDLE) begin
        EX_P_FORMAT <= 1'b1;
    end else if (EX_STATE == EXS_EXAMINE_VERSION && DATA_RDY == 1'b1 && DATA_VALID == 1'b1 && NEXT_EX_STATE == EXS_IDLE) begin
        EX_P_FORMAT <= 1'b1;
    end else if (TRAP_CODE_OPC == T_RTE) begin
        EX_P_RTE <= 1'b1;
    end else if (EX_STATE == EXS_REFILL_PIPE && NEXT_EX_STATE != EXS_REFILL_PIPE) begin // Clear after IPIPE_FLUSH.
        case (EXCEPTION)
            EX_1010, EX_1111, EX_CHK, EX_DIVZERO, EX_ILLEGAL, EX_TRAP, EX_TRAPcc, EX_TRAPV, EX_FORMAT, EX_PRIV, EX_RTE: begin
                EX_P_CHK <= 1'b0;
                EX_P_DIVZERO <= 1'b0;
                EX_P_PRIV <= 1'b0;
                EX_P_1010 <= 1'b0;
                EX_P_1111 <= 1'b0;
                EX_P_ILLEGAL <= 1'b0;
                EX_P_RTE <= 1'b0;
                EX_P_TRAP <= 1'b0;
                EX_P_TRAPcc <= 1'b0;
                EX_P_TRAPV <= 1'b0;
                EX_P_FORMAT <= 1'b0;
            end
            default: ;
        endcase
    // Clear all possible traps during reset exception:
    end else if (SYS_INIT == 1'b1) begin
        EX_P_CHK <= 1'b0;
        EX_P_DIVZERO <= 1'b0;
        EX_P_PRIV <= 1'b0;
        EX_P_1010 <= 1'b0;
        EX_P_1111 <= 1'b0;
        EX_P_ILLEGAL <= 1'b0;
        EX_P_RTE <= 1'b0;
        EX_P_TRAP <= 1'b0;
        EX_P_TRAPcc <= 1'b0;
        EX_P_TRAPV <= 1'b0;
        EX_P_FORMAT <= 1'b0;
    end
end

assign ACCESS_ERR = (EX_STATE == EXS_RESTORE_PC && DATA_RDY == 1'b1 && DATA_0 == 1'b1) ? 1'b1 : // Odd PC value.
                    (DATA_RDY == 1'b1 && DATA_VALID == 1'b0) ? 1'b1 : 1'b0; // Bus error.

assign IRQ_PEND = (EXCEPTION == EX_RESET_EX || EXCEPTION == EX_INT) ? IRQ_PEND_I : STATUS_REG_IN[10:8];
assign IPENDn = (EX_P_INT == 1'b1 || EX_P_RESET == 1'b1 || EX_P_TRACE == 1'b1) ? 1'b0 : 1'b1;

assign EXH_REQ = (EX_STATE != EXS_IDLE) ? 1'b0 :
                 (TRAP_CODE_OPC != NONE) ? 1'b1 :
                 ((EX_P_RESET | EX_P_BERR | EX_P_AERR | EX_P_DIVZERO | EX_P_CHK) == 1'b1) ? 1'b1 :
                 ((EX_P_TRAPcc | EX_P_TRAPV | EX_P_TRACE | EX_P_FORMAT | EX_P_INT) == 1'b1) ? 1'b1 : 1'b0;

// INT_VECTOR
always_ff @(posedge CLK) begin : INT_VECTOR
    reg [7:0] VECT_No;
    reg [31:0] VB_REG;

    if (VBR_WR == 1'b1) begin
        VB_REG = DATA_IN;
    end else if (SYS_INIT == 1'b1) begin
        VB_REG = 32'h00000000;
    end
    //
    if (EX_STATE == EXS_CALC_VECT_NO || EX_STATE == EXS_GET_VECTOR) begin
        case (EXCEPTION)
            EX_RESET_EX: VECT_No = 8'h00;
            EX_BERR:     VECT_No = 8'h02;
            EX_AERR:     VECT_No = 8'h03;
            EX_ILLEGAL:  VECT_No = 8'h04;
            EX_DIVZERO:  VECT_No = 8'h05;
            EX_CHK:      VECT_No = 8'h06;
            EX_TRAPcc:   VECT_No = 8'h07;
            EX_TRAPV:    VECT_No = 8'h07;
            EX_PRIV:     VECT_No = 8'h08;
            EX_TRACE:    VECT_No = 8'h09;
            EX_1010:     VECT_No = 8'h0A;
            EX_1111:     VECT_No = 8'h0B;
            EX_FORMAT:   VECT_No = 8'h0E;
            EX_INT: begin
                if (DATA_RDY == 1'b1 && AVEC == 1'b1) begin
                    VECT_No = 8'h18 + {5'b0, IRQ_PEND_I}; // Autovector.
                end else if (DATA_RDY == 1'b1 && DATA_VALID == 1'b0) begin
                    VECT_No = 8'h18; // Spurious interrupt.
                end else if (DATA_RDY == 1'b1) begin
                    VECT_No = DATA_IN[7:0]; // Non autovector.
                end
            end
            EX_TRAP: VECT_No = {4'h2, TRAP_VECTOR};
            default: VECT_No = 8'hxx; // Don't care.
        endcase
    end
    //
    INT_VECT <= VB_REG + {22'b0, VECT_No, 2'b00};
    VBR <= VB_REG;
    IVECT_OFFS <= {VECT_No, 2'b00};
end

// STORE_CURRENT_EXCEPTION
always_ff @(posedge CLK) begin : STORE_CURRENT_EXCEPTION
    // Priority level 0:
    if (EX_STATE == EXS_IDLE && EX_P_RESET == 1'b1) begin
        EXCEPTION <= EX_RESET_EX;
    // Priority level 1:
    end else if (EX_STATE == EXS_IDLE && EX_P_AERR == 1'b1) begin
        EXCEPTION <= EX_AERR;
    end else if (EX_STATE == EXS_IDLE && EX_P_BERR == 1'b1) begin
        EXCEPTION <= EX_BERR;
    // Priority level 2:
    end else if (EX_STATE == EXS_IDLE && EX_P_CHK == 1'b1) begin
        EXCEPTION <= EX_CHK;
    end else if (EX_STATE == EXS_IDLE && EX_P_TRAPcc == 1'b1) begin
        EXCEPTION <= EX_TRAPcc;
    end else if (EX_STATE == EXS_IDLE && EX_P_DIVZERO == 1'b1) begin
        EXCEPTION <= EX_DIVZERO;
    end else if (EX_STATE == EXS_IDLE && EX_P_TRAP == 1'b1) begin
        EXCEPTION <= EX_TRAP;
    end else if (EX_STATE == EXS_IDLE && EX_P_TRAPV == 1'b1) begin
        EXCEPTION <= EX_TRAPV;
    end else if (EX_STATE == EXS_IDLE && EX_P_FORMAT == 1'b1) begin
        EXCEPTION <= EX_FORMAT;
    // Priority level 3:
    end else if (EX_STATE == EXS_IDLE && EX_P_ILLEGAL == 1'b1) begin
        EXCEPTION <= EX_ILLEGAL;
    end else if (EX_STATE == EXS_IDLE && EX_P_RTE == 1'b1) begin
        EXCEPTION <= EX_RTE;
    end else if (EX_STATE == EXS_IDLE && EX_P_1010 == 1'b1) begin
        EXCEPTION <= EX_1010;
    end else if (EX_STATE == EXS_IDLE && EX_P_1111 == 1'b1) begin
        EXCEPTION <= EX_1111;
    end else if (EX_STATE == EXS_IDLE && EX_P_PRIV == 1'b1) begin
        EXCEPTION <= EX_PRIV;
    end else if (EX_STATE == EXS_IDLE && EX_P_TRACE == 1'b1) begin
        EXCEPTION <= EX_TRACE;
    end else if (EX_STATE == EXS_IDLE && EX_P_INT == 1'b1) begin
        EXCEPTION <= EX_INT;
    end else if (NEXT_EX_STATE == EXS_IDLE) begin
        EXCEPTION <= EX_NONE;
    end
end

assign CPU_SPACE = (NEXT_EX_STATE == EXS_GET_VECTOR) ? 1'b1 : 1'b0;

assign ADR_OFFSET = (EX_STATE == EXS_REFILL_PIPE) ? {24'h000000, 5'b00000, PIPE_CNT, 1'b0} :
                    (NEXT_EX_STATE == EXS_RESTORE_PC && EXCEPTION == EX_RESET_EX) ? 32'h00000004 :
                    (NEXT_EX_STATE == EXS_RESTORE_PC) ? 32'h00000002 :
                    (NEXT_EX_STATE == EXS_VALIDATE_FRAME) ? 32'h00000006 :
                    (NEXT_EX_STATE == EXS_EXAMINE_VERSION) ? 32'h00000036 :
                    (NEXT_EX_STATE == EXS_READ_BOTTOM) ? 32'h0000005C :
                    (NEXT_EX_STATE == EXS_UPDATE_PC) ? INT_VECT : 32'h00000000; // Default is top of the stack.

assign OP_SIZE = (EX_STATE == EXS_INIT) ? LONG : // Decrement the stack by four (ISP_DEC).
                 (NEXT_EX_STATE == EXS_RESTORE_ISP || NEXT_EX_STATE == EXS_RESTORE_PC) ? LONG :
                 (NEXT_EX_STATE == EXS_BUILD_STACK || NEXT_EX_STATE == EXS_BUILD_TSTACK) ? LONG : // Always long access.
                 (NEXT_EX_STATE == EXS_UPDATE_PC || EX_STATE == EXS_UPDATE_PC) ? LONG :
                 (EX_STATE == EXS_SWITCH_STATE) ? LONG :
                 (NEXT_EX_STATE == EXS_GET_VECTOR) ? BYTE : WORD;

// DISPLACEMENT
always_comb begin
    case (STACK_FORMAT_I)
        4'h0, 4'h1: DISPLACEMENT = 8'h08;
        4'h2:       DISPLACEMENT = 8'h0C;
        4'h9:       DISPLACEMENT = 8'h12;
        4'hA:       DISPLACEMENT = 8'h20;
        default:    DISPLACEMENT = 8'h5C; // x"B".
    endcase
end

assign SP_ADD_DISPL = (EX_STATE == EXS_RESTORE_STATUS && DATA_RDY == 1'b1 && DATA_VALID == 1'b1) ? 1'b1 : 1'b0;

// P_D: asynchronous reset with DATA_RDY
always @(posedge CLK or posedge DATA_RDY) begin : P_D
    if (DATA_RDY == 1'b1) begin
        DATA_RD <= 1'b0;
        DATA_WR <= 1'b0;
    end else begin
        DATA_RD <= DATA_RD_I;
        DATA_WR <= DATA_WR_I;
    end
end

assign DATA_RD_I = (DATA_RDY == 1'b1) ? 1'b0 :
                   (NEXT_EX_STATE == EXS_GET_VECTOR) ? 1'b1 :
                   (NEXT_EX_STATE == EXS_VALIDATE_FRAME) ? 1'b1 :
                   (NEXT_EX_STATE == EXS_EXAMINE_VERSION) ? 1'b1 :
                   (NEXT_EX_STATE == EXS_READ_TOP) ? 1'b1 :
                   (NEXT_EX_STATE == EXS_READ_BOTTOM) ? 1'b1 :
                   (NEXT_EX_STATE == EXS_RESTORE_ISP) ? 1'b1 :
                   (NEXT_EX_STATE == EXS_RESTORE_STATUS) ? 1'b1 :
                   (NEXT_EX_STATE == EXS_UPDATE_PC) ? 1'b1 :
                   (NEXT_EX_STATE == EXS_RESTORE_PC) ? 1'b1 : 1'b0;

assign DATA_WR_I = (DATA_RDY == 1'b1) ? 1'b0 :
                   (EX_STATE == EXS_BUILD_STACK) ? 1'b1 :
                   (EX_STATE == EXS_BUILD_TSTACK) ? 1'b1 : 1'b0;

assign ISP_LOAD = (EX_STATE == EXS_RESTORE_ISP && DATA_RDY == 1'b1 && DATA_VALID == 1'b1) ? 1'b1 : 1'b0;
assign PC_RESTORE = (EX_STATE == EXS_RESTORE_PC && DATA_RDY == 1'b1 && DATA_VALID == 1'b1) ? 1'b1 : 1'b0;
assign PC_LOAD = (EXCEPTION != EX_RESET_EX && EXCEPTION != EX_RTE && EX_STATE != EXS_REFILL_PIPE && NEXT_EX_STATE == EXS_REFILL_PIPE) ? 1'b1 : 1'b0;

assign IPIPE_FILL = (EX_STATE == EXS_REFILL_PIPE) ? 1'b1 : 1'b0;

assign PC_INC = (EXCEPTION == EX_CHK && EX_STATE != EXS_BUILD_STACK && NEXT_EX_STATE == EXS_BUILD_STACK) ? 1'b1 :
                (EXCEPTION == EX_DIVZERO && EX_STATE != EXS_BUILD_STACK && NEXT_EX_STATE == EXS_BUILD_STACK) ? 1'b1 :
                (EXCEPTION == EX_INT && EX_STATE != EXS_BUILD_STACK && NEXT_EX_STATE == EXS_BUILD_STACK) ? 1'b1 :
                (EXCEPTION == EX_TRAP && EX_STATE != EXS_BUILD_STACK && NEXT_EX_STATE == EXS_BUILD_STACK) ? 1'b1 :
                (EXCEPTION == EX_TRAPcc && EX_STATE != EXS_BUILD_STACK && NEXT_EX_STATE == EXS_BUILD_STACK) ? 1'b1 :
                (EXCEPTION == EX_TRAPV && EX_STATE != EXS_BUILD_STACK && NEXT_EX_STATE == EXS_BUILD_STACK) ? 1'b1 : 1'b0;

assign ISP_DEC = (EX_STATE == EXS_INIT && EXCEPTION != EX_RESET_EX && EXCEPTION != EX_RTE) ? 1'b1 : // Early due to one clock cycle address calculation.
                 (EX_STATE == EXS_BUILD_STACK && DATA_RDY == 1'b1 && NEXT_EX_STATE == EXS_BUILD_STACK) ? 1'b1 :
                 (EX_STATE == EXS_SWITCH_STATE) ? 1'b1 :
                 (EX_STATE == EXS_BUILD_TSTACK && DATA_RDY == 1'b1 && NEXT_EX_STATE == EXS_BUILD_TSTACK) ? 1'b1 : 1'b0;

assign SR_INIT = (EX_STATE == EXS_INIT) ? 1'b1 : 1'b0;
assign SR_CLR_MBIT = (EX_STATE == EXS_BUILD_STACK && DATA_RDY == 1'b1 && STACK_CNT == 2 && EXCEPTION == EX_INT && MBIT == 1'b1) ? 1'b1 : 1'b0;
assign SR_WR = (EX_STATE == EXS_RESTORE_STATUS && DATA_RDY == 1'b1 && DATA_VALID == 1'b1) ? 1'b1 : 1'b0;

assign SYS_INIT = (EX_STATE == EXS_IDLE && EX_P_RESET == 1'b1) ? 1'b1 : 1'b0;

assign HALT_OUTn = (EX_STATE == EXS_HALTED) ? 1'b0 : 1'b1;

assign RESTORE_ISP_PC = (EXCEPTION == EX_RESET_EX && (NEXT_EX_STATE == EXS_RESTORE_ISP || EX_STATE == EXS_RESTORE_ISP)) ? 1'b1 :
                        (EXCEPTION == EX_RESET_EX && (NEXT_EX_STATE == EXS_RESTORE_PC || EX_STATE == EXS_RESTORE_PC)) ? 1'b1 :
                        (NEXT_EX_STATE == EXS_UPDATE_PC) ? 1'b1 : 1'b0;

assign REFILLn = (EX_STATE == EXS_REFILL_PIPE) ? 1'b0 : 1'b1;

assign STACK_FORMAT = STACK_FORMAT_I;

assign IPIPE_FLUSH = (EXCEPTION == EX_RESET_EX && EX_STATE != EXS_REFILL_PIPE) ? 1'b1 :
                     (EXCEPTION != EX_NONE && EX_STATE != EXS_REFILL_PIPE && NEXT_EX_STATE == EXS_REFILL_PIPE) ? 1'b1 : 1'b0;

assign DOUBLE_BUSFLT = ((EXCEPTION == EX_AERR || EXCEPTION == EX_RESET_EX) && EX_STATE == EXS_RESTORE_PC && DATA_RDY == 1'b1 && DATA_0 == 1'b1) ? 1'b1 : // Odd PC value.
                       (EX_STATE != EXS_IDLE && EXCEPTION == EX_AERR && DATA_RDY == 1'b1 && DATA_VALID == 1'b0) ? 1'b1 :
                       (EX_STATE != EXS_IDLE && EXCEPTION == EX_BERR && DATA_RDY == 1'b1 && DATA_VALID == 1'b0) ? 1'b1 :
                       (EX_STATE != EXS_IDLE && EXCEPTION == EX_RESET_EX && DATA_RDY == 1'b1 && DATA_VALID == 1'b0) ? 1'b1 : 1'b0;

// P_TMP_CPY
always_ff @(posedge CLK) begin : P_TMP_CPY
    if (EX_STATE == EXS_IDLE && NEXT_EX_STATE != EXS_IDLE) begin
        SR_CPY <= STATUS_REG_IN;
        ADR_CPY <= ADR_IN;
        MBIT <= STATUS_REG_IN[12];
    end else if (EX_STATE == EXS_BUILD_STACK && NEXT_EX_STATE == EXS_SWITCH_STATE) begin
        SR_CPY[13] <= 1'b1; // Set S bit.
    end
end

// STACK_CTRL
always_ff @(posedge CLK) begin : STACK_CTRL
    reg [5:0] STACK_POS_VAR;
    if (EX_STATE != EXS_BUILD_TSTACK && NEXT_EX_STATE == EXS_BUILD_TSTACK) begin
        STACK_POS_VAR = 6'd4;
        STACK_FORMAT_I <= 4'h1;
    end else if (EX_STATE != EXS_BUILD_STACK && NEXT_EX_STATE == EXS_BUILD_STACK) begin
        case (EXCEPTION)
            EX_INT, EX_ILLEGAL, EX_1010, EX_1111, EX_FORMAT, EX_PRIV, EX_TRAP: begin
                STACK_POS_VAR = 6'd4; // Format 0.
                STACK_FORMAT_I <= 4'h0;
            end
            EX_CHK, EX_TRAPcc, EX_TRAPV, EX_TRACE, EX_DIVZERO: begin
                STACK_POS_VAR = 6'd6; // Format 2.
                STACK_FORMAT_I <= 4'h2;
            end
            EX_AERR, EX_BERR: begin
                if (IBOUND == 1'b1) begin
                    STACK_POS_VAR = 6'd16; // Format A.
                    STACK_FORMAT_I <= 4'hA;
                end else begin
                    STACK_POS_VAR = 6'd46; // Format B.
                    STACK_FORMAT_I <= 4'hB;
                end
            end
            default: ;
        endcase
    end else if (EX_STATE == EXS_VALIDATE_FRAME && DATA_RDY == 1'b1 && DATA_VALID == 1'b1) begin
        STACK_FORMAT_I <= DATA_IN[15:12];
    end else if ((EX_STATE == EXS_BUILD_STACK || EX_STATE == EXS_BUILD_TSTACK) && DATA_RDY == 1'b1) begin
        STACK_POS_VAR = STACK_POS_VAR - 6'd2; // Always long words are written.
    end
    //
    STACK_CNT <= STACK_POS_VAR;
    STACK_POS <= STACK_POS_VAR;
end

// P_STATUSn
always_ff @(posedge CLK) begin : P_STATUSn
    reg [1:0] CNT;
    if (EX_STATE == EXS_CALC_VECT_NO) begin
        case (EXCEPTION)
            EX_RESET_EX, EX_AERR, EX_BERR, EX_1111: begin
                CNT = 2'b11;
                STATUSn <= 1'b0;
            end
            EX_INT, EX_TRACE: begin
                CNT = 2'b10;
            end
            default: begin
                CNT = 2'b00;
            end
        endcase
    end

    if (EX_STATE == EXS_HALTED) begin
        STATUSn <= 1'b0;
    end else if (CNT > 2'b00) begin
        CNT = CNT - 2'd1;
        STATUSn <= 1'b0;
    end else begin
        STATUSn <= 1'b1;
    end
end

// PIPE_STATUS
always_ff @(posedge CLK) begin : PIPE_STATUS
    reg [1:0] CNT;
    if (EX_STATE != EXS_REFILL_PIPE) begin
        PIPE_FULL <= 1'b0;
        CNT = 2'b00;
    end else if (EX_STATE == EXS_REFILL_PIPE && OPCODE_RDY == 1'b1 && CNT < 2'b10) begin
        CNT = CNT + 2'd1;
    end else if (EX_STATE == EXS_REFILL_PIPE && OPCODE_RDY == 1'b1) begin
        PIPE_FULL <= 1'b1;
    end
    PIPE_CNT <= CNT;
end

// EXCEPTION_HANDLER_REG
always_ff @(posedge CLK) begin : EXCEPTION_HANDLER_REG
    if (RESET == 1'b1) begin
        EX_STATE <= EXS_IDLE;
    end else begin
        EX_STATE <= NEXT_EX_STATE;
    end
end

// EXCEPTION_HANDLER_DEC
always_comb begin : EXCEPTION_HANDLER_DEC
    case (EX_STATE)
        EXS_IDLE: begin
            if ((BUSY_MAIN == 1'b1 || BUSY_OPD == 1'b1) && EX_P_RESET == 1'b0) begin
                NEXT_EX_STATE = EXS_IDLE; // Wait until the pipelined architecture is ready.
            end else if (EX_P_RESET == 1'b1 || EX_P_AERR == 1'b1 || EX_P_BERR == 1'b1) begin
                NEXT_EX_STATE = EXS_INIT;
            end else if (EX_P_TRAP == 1'b1 || EX_P_TRAPcc == 1'b1 || EX_P_TRAPV == 1'b1 || EX_P_CHK == 1'b1 || EX_P_DIVZERO == 1'b1) begin
                NEXT_EX_STATE = EXS_INIT;
            end else if (EX_P_FORMAT == 1'b1) begin
                NEXT_EX_STATE = EXS_INIT;
            end else if (EX_P_TRACE == 1'b1 || EX_P_ILLEGAL == 1'b1 || EX_P_1010 == 1'b1 || EX_P_1111 == 1'b1 || EX_P_PRIV == 1'b1) begin
                NEXT_EX_STATE = EXS_INIT;
            end else if (EX_P_RTE == 1'b1) begin
                NEXT_EX_STATE = EXS_INIT;
            end else if (EX_P_INT == 1'b1) begin
                NEXT_EX_STATE = EXS_INIT;
            end else begin // No exception.
                NEXT_EX_STATE = EXS_IDLE;
            end
        end
        EXS_INIT: begin
            case (EXCEPTION)
                EX_RTE:
                    NEXT_EX_STATE = EXS_VALIDATE_FRAME; // 68K10.
                EX_INT:
                    NEXT_EX_STATE = EXS_GET_VECTOR;
                default:
                    NEXT_EX_STATE = EXS_CALC_VECT_NO;
            endcase
        end
        EXS_GET_VECTOR: begin
            if (DATA_RDY == 1'b1) begin
                NEXT_EX_STATE = EXS_BUILD_STACK;
            end else begin
                NEXT_EX_STATE = EXS_GET_VECTOR;
            end
        end
        EXS_CALC_VECT_NO: begin
            case (EXCEPTION)
                EX_RESET_EX:
                    NEXT_EX_STATE = EXS_RESTORE_ISP; // Do not stack anything but update the SSP and PC.
                default:
                    NEXT_EX_STATE = EXS_BUILD_STACK;
            endcase
        end
        EXS_BUILD_STACK: begin
            if (DOUBLE_BUSFLT == 1'b1) begin
                NEXT_EX_STATE = EXS_HALTED;
            end else if (ACCESS_ERR == 1'b1) begin
                NEXT_EX_STATE = EXS_IDLE;
            end else if (DATA_RDY == 1'b1 && STACK_CNT == 2 && EXCEPTION == EX_INT && MBIT == 1'b1) begin
                NEXT_EX_STATE = EXS_SWITCH_STATE; // Build throwaway stack frame.
            end else if (DATA_RDY == 1'b1 && STACK_CNT == 2) begin
                NEXT_EX_STATE = EXS_UPDATE_PC;
            end else begin
                NEXT_EX_STATE = EXS_BUILD_STACK;
            end
        end
        EXS_SWITCH_STATE: begin // Required to decrement the correct stack pointer.
            NEXT_EX_STATE = EXS_BUILD_TSTACK;
        end
        EXS_BUILD_TSTACK: begin // Build throwaway stack frame.
            if (ACCESS_ERR == 1'b1) begin
                NEXT_EX_STATE = EXS_IDLE;
            end else if (DATA_RDY == 1'b1 && STACK_CNT == 2) begin
                NEXT_EX_STATE = EXS_UPDATE_PC;
            end else begin
                NEXT_EX_STATE = EXS_BUILD_TSTACK;
            end
        end
        EXS_UPDATE_PC: begin
            if (DOUBLE_BUSFLT == 1'b1) begin
                NEXT_EX_STATE = EXS_HALTED;
            end else if (ACCESS_ERR == 1'b1) begin
                NEXT_EX_STATE = EXS_IDLE;
            end else if (DATA_RDY == 1'b1) begin
                NEXT_EX_STATE = EXS_REFILL_PIPE;
            end else begin
                NEXT_EX_STATE = EXS_UPDATE_PC;
            end
        end
        EXS_VALIDATE_FRAME: begin
            if (ACCESS_ERR == 1'b1) begin
                NEXT_EX_STATE = EXS_IDLE;
            end else if (DATA_RDY == 1'b1) begin
                case (DATA_IN[15:12])
                    4'h0, 4'h1, 4'h2, 4'h9:
                        NEXT_EX_STATE = EXS_RESTORE_PC;
                    4'hA, 4'hB:
                        NEXT_EX_STATE = EXS_EXAMINE_VERSION;
                    default:
                        NEXT_EX_STATE = EXS_IDLE; // Format error.
                endcase
            end else begin
                NEXT_EX_STATE = EXS_VALIDATE_FRAME;
            end
        end
        EXS_EXAMINE_VERSION: begin
            if (ACCESS_ERR == 1'b1) begin
                NEXT_EX_STATE = EXS_IDLE;
            end else if (DATA_RDY == 1'b1) begin
                if (DATA_IN != {16'b0, VERSION}) begin
                    NEXT_EX_STATE = EXS_IDLE; // Format error.
                end else begin
                    NEXT_EX_STATE = EXS_READ_TOP;
                end
            end else begin
                NEXT_EX_STATE = EXS_EXAMINE_VERSION;
            end
        end
        EXS_READ_TOP: begin
            if (ACCESS_ERR == 1'b1) begin
                NEXT_EX_STATE = EXS_IDLE;
            end else if (DATA_RDY == 1'b1) begin
                NEXT_EX_STATE = EXS_READ_BOTTOM;
            end else begin
                NEXT_EX_STATE = EXS_READ_TOP;
            end
        end
        EXS_READ_BOTTOM: begin
            if (ACCESS_ERR == 1'b1) begin
                NEXT_EX_STATE = EXS_IDLE;
            end else if (DATA_RDY == 1'b1) begin
                NEXT_EX_STATE = EXS_RESTORE_PC;
            end else begin
                NEXT_EX_STATE = EXS_READ_BOTTOM;
            end
        end
        EXS_RESTORE_STATUS: begin
            if (DOUBLE_BUSFLT == 1'b1) begin
                NEXT_EX_STATE = EXS_HALTED;
            end else if (ACCESS_ERR == 1'b1) begin
                NEXT_EX_STATE = EXS_IDLE;
            end else if (DATA_RDY == 1'b1 && STACK_FORMAT_I == 4'h1) begin
                NEXT_EX_STATE = EXS_VALIDATE_FRAME; // Throwaway stack frame.
            end else if (DATA_RDY == 1'b1) begin
                NEXT_EX_STATE = EXS_REFILL_PIPE;
            end else begin
                NEXT_EX_STATE = EXS_RESTORE_STATUS;
            end
        end
        EXS_RESTORE_ISP: begin
            if (DOUBLE_BUSFLT == 1'b1) begin
                NEXT_EX_STATE = EXS_HALTED;
            end else if (ACCESS_ERR == 1'b1) begin
                NEXT_EX_STATE = EXS_IDLE;
            end else if (DATA_RDY == 1'b1) begin
                NEXT_EX_STATE = EXS_RESTORE_PC;
            end else begin
                NEXT_EX_STATE = EXS_RESTORE_ISP;
            end
        end
        EXS_RESTORE_PC: begin
            if (DOUBLE_BUSFLT == 1'b1) begin
                NEXT_EX_STATE = EXS_HALTED; // Double bus fault.
            end else if (ACCESS_ERR == 1'b1) begin
                NEXT_EX_STATE = EXS_IDLE;
            end else if (EXCEPTION == EX_RESET_EX && DATA_RDY == 1'b1) begin
                NEXT_EX_STATE = EXS_REFILL_PIPE;
            end else if (DATA_RDY == 1'b1) begin
                NEXT_EX_STATE = EXS_RESTORE_STATUS;
            end else begin
                NEXT_EX_STATE = EXS_RESTORE_PC;
            end
        end
        EXS_REFILL_PIPE: begin
            if (DOUBLE_BUSFLT == 1'b1) begin
                NEXT_EX_STATE = EXS_HALTED;
            end else if (ACCESS_ERR == 1'b1) begin
                NEXT_EX_STATE = EXS_IDLE;
            end else if (PIPE_FULL == 1'b1) begin
                NEXT_EX_STATE = EXS_IDLE;
            end else begin
                NEXT_EX_STATE = EXS_REFILL_PIPE;
            end
        end
        EXS_HALTED: begin
            // Processor halted, Double bus error!
            NEXT_EX_STATE = EXS_HALTED;
        end
        default: begin
            NEXT_EX_STATE = EXS_IDLE;
        end
    endcase
end

endmodule
