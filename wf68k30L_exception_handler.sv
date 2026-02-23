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

assign BUSY_EXH = (EX_STATE != EXS_IDLE);

// IRQ_FILTER - negedge CLK
always_ff @(negedge CLK) begin : irq_filter
    reg [2:0] IRQ_TMP_1;
    reg [2:0] IRQ_TMP_2;
    if (IRQ_TMP_1 == IRQ_TMP_2) begin
        IRQ <= IRQ_TMP_2;
    end
    IRQ_TMP_2 = IRQ_TMP_1;
    IRQ_TMP_1 = IRQ_IN;
end

// AVEC_FILTER
always_ff @(posedge CLK) begin : avec_filter
    if (!AVECn)
        AVEC <= 1'b1;
    else if (DATA_RDY || RESET)
        AVEC <= 1'b0;
end

// INSTRUCTION_BOUNDARY
always_ff @(posedge CLK) begin : instruction_boundary
    if (RESET)
        IBOUND <= 1'b0;
    else if (OPD_ACK && !OW_VALID)
        IBOUND <= 1'b1;
    else if (EX_STATE == EXS_BUILD_STACK)
        IBOUND <= 1'b0;
end

// PENDING
always_ff @(posedge CLK) begin : pending
    reg INT7_TRIG;
    reg [2:0] INT_VAR;
    reg [2:0] SR_VAR;

    if (RESET)
        EX_P_RESET <= 1'b1;
    else if (EX_STATE == EXS_RESTORE_PC && DATA_RDY && EXCEPTION == EX_RESET_EX)
        EX_P_RESET <= 1'b0;

    if (TRAP_BERR)
        EX_P_BERR <= 1'b1;
    else if (EX_STATE != EXS_IDLE && DATA_RDY && !DATA_VALID)
        EX_P_BERR <= 1'b1;
    else if (EX_STATE == EXS_INIT && EXCEPTION == EX_BERR)
        EX_P_BERR <= 1'b0; // Reset in the beginning to enable retriggering.
    else if (SYS_INIT)
        EX_P_BERR <= 1'b0;

    if (TRAP_AERR)
        EX_P_AERR <= 1'b1;
    else if (EX_STATE == EXS_BUILD_STACK && EXCEPTION == EX_AERR)
        EX_P_AERR <= 1'b0;
    else if (SYS_INIT)
        EX_P_AERR <= 1'b0;

    if (EX_TRACE_IN)
        EX_P_TRACE <= 1'b1;
    else if (EX_STATE == EXS_BUILD_STACK && EXCEPTION == EX_TRACE)
        EX_P_TRACE <= 1'b0;
    else if (SYS_INIT)
        EX_P_TRACE <= 1'b0;

    if (IRQ == 3'b111 && SR_VAR == 3'b111 && STATUS_REG_IN[10:8] != 3'b111)
        INT7_TRIG = 1'b1; // Trigger by lowering the mask from 7 to any value.
    else if (IRQ == 3'b111 && INT_VAR < 3'b111)
        INT7_TRIG = 1'b1; // Trigger when level 7 is entered.
    else
        INT7_TRIG = 1'b0;

    SR_VAR = STATUS_REG_IN[10:8]; // Update after use!
    INT_VAR = IRQ; // Update after use!

    if (SYS_INIT) begin // Reset when disabling the interrupts.
        EX_P_INT <= 1'b0;
        IRQ_PEND_I <= 3'b111; // This is required for system startup.
    end else if (EX_STATE == EXS_GET_VECTOR && DATA_RDY) begin
        EX_P_INT <= 1'b0;
    end else if (INT7_TRIG) begin // Level 7 is nonmaskable ...
        EX_P_INT <= 1'b1;
        IRQ_PEND_I <= IRQ;
    end else if (INT_TRIG && STATUS_REG_IN[10:8] < IRQ) begin
        EX_P_INT <= 1'b1;
        IRQ_PEND_I <= IRQ;
    end

    // The following nine traps never appear at the same time:
    if (TRAP_CHK)
        EX_P_CHK <= 1'b1;
    else if (TRAP_DIVZERO)
        EX_P_DIVZERO <= 1'b1;
    else if (TRAP_CODE_OPC == T_TRAP)
        EX_P_TRAP <= 1'b1;
    else if (TRAP_cc)
        EX_P_TRAPcc <= 1'b1;
    else if (TRAP_V)
        EX_P_TRAPV <= 1'b1;
    else if (TRAP_CODE_OPC == T_PRIV)
        EX_P_PRIV <= 1'b1;
    else if (TRAP_CODE_OPC == T_1010)
        EX_P_1010 <= 1'b1;
    else if (TRAP_CODE_OPC == T_1111)
        EX_P_1111 <= 1'b1;
    else if (TRAP_CODE_OPC == T_ILLEGAL)
        EX_P_ILLEGAL <= 1'b1;
    else if (TRAP_ILLEGAL) // Used for BKPT.
        EX_P_ILLEGAL <= 1'b1;
    else if (EX_STATE == EXS_VALIDATE_FRAME && DATA_RDY && DATA_VALID && NEXT_EX_STATE == EXS_IDLE)
        EX_P_FORMAT <= 1'b1;
    else if (EX_STATE == EXS_EXAMINE_VERSION && DATA_RDY && DATA_VALID && NEXT_EX_STATE == EXS_IDLE)
        EX_P_FORMAT <= 1'b1;
    else if (TRAP_CODE_OPC == T_RTE)
        EX_P_RTE <= 1'b1;
    else if (EX_STATE == EXS_REFILL_PIPE && NEXT_EX_STATE != EXS_REFILL_PIPE) begin // Clear after IPIPE_FLUSH.
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
    end else if (SYS_INIT) begin
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

assign ACCESS_ERR = (EX_STATE == EXS_RESTORE_PC && DATA_RDY && DATA_0) || // Odd PC value.
                    (DATA_RDY && !DATA_VALID); // Bus error.

assign IRQ_PEND = (EXCEPTION == EX_RESET_EX || EXCEPTION == EX_INT) ? IRQ_PEND_I : STATUS_REG_IN[10:8];
assign IPENDn = !(EX_P_INT || EX_P_RESET || EX_P_TRACE);

assign EXH_REQ = (EX_STATE != EXS_IDLE) ? 1'b0 :
                 (TRAP_CODE_OPC != NONE) ? 1'b1 :
                 (EX_P_RESET | EX_P_BERR | EX_P_AERR | EX_P_DIVZERO | EX_P_CHK) ||
                 (EX_P_TRAPcc | EX_P_TRAPV | EX_P_TRACE | EX_P_FORMAT | EX_P_INT);

// INT_VECTOR
always_ff @(posedge CLK) begin : int_vector
    reg [7:0] VECT_No;
    reg [31:0] VB_REG;

    if (VBR_WR)
        VB_REG = DATA_IN;
    else if (SYS_INIT)
        VB_REG = 32'h0;
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
                if (DATA_RDY && AVEC)
                    VECT_No = 8'h18 + {5'b0, IRQ_PEND_I}; // Autovector.
                else if (DATA_RDY && !DATA_VALID)
                    VECT_No = 8'h18; // Spurious interrupt.
                else if (DATA_RDY)
                    VECT_No = DATA_IN[7:0]; // Non autovector.
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
always_ff @(posedge CLK) begin : store_current_exception
    // Priority level 0:
    if (EX_STATE == EXS_IDLE && EX_P_RESET)
        EXCEPTION <= EX_RESET_EX;
    // Priority level 1:
    else if (EX_STATE == EXS_IDLE && EX_P_AERR)
        EXCEPTION <= EX_AERR;
    else if (EX_STATE == EXS_IDLE && EX_P_BERR)
        EXCEPTION <= EX_BERR;
    // Priority level 2:
    else if (EX_STATE == EXS_IDLE && EX_P_CHK)
        EXCEPTION <= EX_CHK;
    else if (EX_STATE == EXS_IDLE && EX_P_TRAPcc)
        EXCEPTION <= EX_TRAPcc;
    else if (EX_STATE == EXS_IDLE && EX_P_DIVZERO)
        EXCEPTION <= EX_DIVZERO;
    else if (EX_STATE == EXS_IDLE && EX_P_TRAP)
        EXCEPTION <= EX_TRAP;
    else if (EX_STATE == EXS_IDLE && EX_P_TRAPV)
        EXCEPTION <= EX_TRAPV;
    else if (EX_STATE == EXS_IDLE && EX_P_FORMAT)
        EXCEPTION <= EX_FORMAT;
    // Priority level 3:
    else if (EX_STATE == EXS_IDLE && EX_P_ILLEGAL)
        EXCEPTION <= EX_ILLEGAL;
    else if (EX_STATE == EXS_IDLE && EX_P_RTE)
        EXCEPTION <= EX_RTE;
    else if (EX_STATE == EXS_IDLE && EX_P_1010)
        EXCEPTION <= EX_1010;
    else if (EX_STATE == EXS_IDLE && EX_P_1111)
        EXCEPTION <= EX_1111;
    else if (EX_STATE == EXS_IDLE && EX_P_PRIV)
        EXCEPTION <= EX_PRIV;
    else if (EX_STATE == EXS_IDLE && EX_P_TRACE)
        EXCEPTION <= EX_TRACE;
    else if (EX_STATE == EXS_IDLE && EX_P_INT)
        EXCEPTION <= EX_INT;
    else if (NEXT_EX_STATE == EXS_IDLE)
        EXCEPTION <= EX_NONE;
end

assign CPU_SPACE = (NEXT_EX_STATE == EXS_GET_VECTOR);

assign ADR_OFFSET = (EX_STATE == EXS_REFILL_PIPE) ? {24'h0, 5'b0, PIPE_CNT, 1'b0} :
                    (NEXT_EX_STATE == EXS_RESTORE_PC && EXCEPTION == EX_RESET_EX) ? 32'h4 :
                    (NEXT_EX_STATE == EXS_RESTORE_PC) ? 32'h2 :
                    (NEXT_EX_STATE == EXS_VALIDATE_FRAME) ? 32'h6 :
                    (NEXT_EX_STATE == EXS_EXAMINE_VERSION) ? 32'h36 :
                    (NEXT_EX_STATE == EXS_READ_BOTTOM) ? 32'h5C :
                    (NEXT_EX_STATE == EXS_UPDATE_PC) ? INT_VECT : 32'h0; // Default is top of the stack.

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

assign SP_ADD_DISPL = EX_STATE == EXS_RESTORE_STATUS && DATA_RDY && DATA_VALID;

// P_D: asynchronous reset with DATA_RDY
always @(posedge CLK or posedge DATA_RDY) begin : data_rw_async
    if (DATA_RDY) begin
        DATA_RD <= 1'b0;
        DATA_WR <= 1'b0;
    end else begin
        DATA_RD <= DATA_RD_I;
        DATA_WR <= DATA_WR_I;
    end
end

assign DATA_RD_I = DATA_RDY ? 1'b0 :
                   (NEXT_EX_STATE == EXS_GET_VECTOR) ? 1'b1 :
                   (NEXT_EX_STATE == EXS_VALIDATE_FRAME) ? 1'b1 :
                   (NEXT_EX_STATE == EXS_EXAMINE_VERSION) ? 1'b1 :
                   (NEXT_EX_STATE == EXS_READ_TOP) ? 1'b1 :
                   (NEXT_EX_STATE == EXS_READ_BOTTOM) ? 1'b1 :
                   (NEXT_EX_STATE == EXS_RESTORE_ISP) ? 1'b1 :
                   (NEXT_EX_STATE == EXS_RESTORE_STATUS) ? 1'b1 :
                   (NEXT_EX_STATE == EXS_UPDATE_PC) ? 1'b1 :
                   (NEXT_EX_STATE == EXS_RESTORE_PC) ? 1'b1 : 1'b0;

assign DATA_WR_I = DATA_RDY ? 1'b0 :
                   (EX_STATE == EXS_BUILD_STACK) ? 1'b1 :
                   (EX_STATE == EXS_BUILD_TSTACK) ? 1'b1 : 1'b0;

assign ISP_LOAD = EX_STATE == EXS_RESTORE_ISP && DATA_RDY && DATA_VALID;
assign PC_RESTORE = EX_STATE == EXS_RESTORE_PC && DATA_RDY && DATA_VALID;
assign PC_LOAD = EXCEPTION != EX_RESET_EX && EXCEPTION != EX_RTE && EX_STATE != EXS_REFILL_PIPE && NEXT_EX_STATE == EXS_REFILL_PIPE;

assign IPIPE_FILL = (EX_STATE == EXS_REFILL_PIPE);

assign PC_INC = (EX_STATE != EXS_BUILD_STACK && NEXT_EX_STATE == EXS_BUILD_STACK) &&
                (EXCEPTION == EX_CHK || EXCEPTION == EX_DIVZERO || EXCEPTION == EX_INT ||
                 EXCEPTION == EX_TRAP || EXCEPTION == EX_TRAPcc || EXCEPTION == EX_TRAPV);

assign ISP_DEC = (EX_STATE == EXS_INIT && EXCEPTION != EX_RESET_EX && EXCEPTION != EX_RTE) || // Early due to one clock cycle address calculation.
                 (EX_STATE == EXS_BUILD_STACK && DATA_RDY && NEXT_EX_STATE == EXS_BUILD_STACK) ||
                 (EX_STATE == EXS_SWITCH_STATE) ||
                 (EX_STATE == EXS_BUILD_TSTACK && DATA_RDY && NEXT_EX_STATE == EXS_BUILD_TSTACK);

assign SR_INIT = (EX_STATE == EXS_INIT);
assign SR_CLR_MBIT = EX_STATE == EXS_BUILD_STACK && DATA_RDY && STACK_CNT == 2 && EXCEPTION == EX_INT && MBIT;
assign SR_WR = EX_STATE == EXS_RESTORE_STATUS && DATA_RDY && DATA_VALID;

assign SYS_INIT = EX_STATE == EXS_IDLE && EX_P_RESET;

assign HALT_OUTn = !(EX_STATE == EXS_HALTED);

assign RESTORE_ISP_PC = (EXCEPTION == EX_RESET_EX && (NEXT_EX_STATE == EXS_RESTORE_ISP || EX_STATE == EXS_RESTORE_ISP)) ||
                        (EXCEPTION == EX_RESET_EX && (NEXT_EX_STATE == EXS_RESTORE_PC || EX_STATE == EXS_RESTORE_PC)) ||
                        (NEXT_EX_STATE == EXS_UPDATE_PC);

assign REFILLn = !(EX_STATE == EXS_REFILL_PIPE);

assign STACK_FORMAT = STACK_FORMAT_I;

assign IPIPE_FLUSH = (EXCEPTION == EX_RESET_EX && EX_STATE != EXS_REFILL_PIPE) ||
                     (EXCEPTION != EX_NONE && EX_STATE != EXS_REFILL_PIPE && NEXT_EX_STATE == EXS_REFILL_PIPE);

assign DOUBLE_BUSFLT = ((EXCEPTION == EX_AERR || EXCEPTION == EX_RESET_EX) && EX_STATE == EXS_RESTORE_PC && DATA_RDY && DATA_0) || // Odd PC value.
                       (EX_STATE != EXS_IDLE && EXCEPTION == EX_AERR && DATA_RDY && !DATA_VALID) ||
                       (EX_STATE != EXS_IDLE && EXCEPTION == EX_BERR && DATA_RDY && !DATA_VALID) ||
                       (EX_STATE != EXS_IDLE && EXCEPTION == EX_RESET_EX && DATA_RDY && !DATA_VALID);

// P_TMP_CPY
always_ff @(posedge CLK) begin : tmp_cpy
    if (EX_STATE == EXS_IDLE && NEXT_EX_STATE != EXS_IDLE) begin
        SR_CPY <= STATUS_REG_IN;
        ADR_CPY <= ADR_IN;
        MBIT <= STATUS_REG_IN[12];
    end else if (EX_STATE == EXS_BUILD_STACK && NEXT_EX_STATE == EXS_SWITCH_STATE) begin
        SR_CPY[13] <= 1'b1; // Set S bit.
    end
end

// STACK_CTRL
always_ff @(posedge CLK) begin : stack_ctrl
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
                if (IBOUND) begin
                    STACK_POS_VAR = 6'd16; // Format A.
                    STACK_FORMAT_I <= 4'hA;
                end else begin
                    STACK_POS_VAR = 6'd46; // Format B.
                    STACK_FORMAT_I <= 4'hB;
                end
            end
            default: ;
        endcase
    end else if (EX_STATE == EXS_VALIDATE_FRAME && DATA_RDY && DATA_VALID) begin
        STACK_FORMAT_I <= DATA_IN[15:12];
    end else if ((EX_STATE == EXS_BUILD_STACK || EX_STATE == EXS_BUILD_TSTACK) && DATA_RDY) begin
        STACK_POS_VAR = STACK_POS_VAR - 6'd2; // Always long words are written.
    end
    //
    STACK_CNT <= STACK_POS_VAR;
    STACK_POS <= STACK_POS_VAR;
end

// P_STATUSn
always_ff @(posedge CLK) begin : status_out
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
always_ff @(posedge CLK) begin : pipe_status
    reg [1:0] CNT;
    if (EX_STATE != EXS_REFILL_PIPE) begin
        PIPE_FULL <= 1'b0;
        CNT = 2'b00;
    end else if (EX_STATE == EXS_REFILL_PIPE && OPCODE_RDY && CNT < 2'b10) begin
        CNT = CNT + 2'd1;
    end else if (EX_STATE == EXS_REFILL_PIPE && OPCODE_RDY) begin
        PIPE_FULL <= 1'b1;
    end
    PIPE_CNT <= CNT;
end

// EXCEPTION_HANDLER_REG
always_ff @(posedge CLK) begin : exception_handler_reg
    if (RESET)
        EX_STATE <= EXS_IDLE;
    else
        EX_STATE <= NEXT_EX_STATE;
end

// EXCEPTION_HANDLER_DEC
always_comb begin : exception_handler_dec
    case (EX_STATE)
        EXS_IDLE: begin
            if ((BUSY_MAIN || BUSY_OPD) && !EX_P_RESET)
                NEXT_EX_STATE = EXS_IDLE; // Wait until the pipelined architecture is ready.
            else if (EX_P_RESET || EX_P_AERR || EX_P_BERR)
                NEXT_EX_STATE = EXS_INIT;
            else if (EX_P_TRAP || EX_P_TRAPcc || EX_P_TRAPV || EX_P_CHK || EX_P_DIVZERO)
                NEXT_EX_STATE = EXS_INIT;
            else if (EX_P_FORMAT)
                NEXT_EX_STATE = EXS_INIT;
            else if (EX_P_TRACE || EX_P_ILLEGAL || EX_P_1010 || EX_P_1111 || EX_P_PRIV)
                NEXT_EX_STATE = EXS_INIT;
            else if (EX_P_RTE)
                NEXT_EX_STATE = EXS_INIT;
            else if (EX_P_INT)
                NEXT_EX_STATE = EXS_INIT;
            else // No exception.
                NEXT_EX_STATE = EXS_IDLE;
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
            if (DATA_RDY)
                NEXT_EX_STATE = EXS_BUILD_STACK;
            else
                NEXT_EX_STATE = EXS_GET_VECTOR;
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
            if (DOUBLE_BUSFLT)
                NEXT_EX_STATE = EXS_HALTED;
            else if (ACCESS_ERR)
                NEXT_EX_STATE = EXS_IDLE;
            else if (DATA_RDY && STACK_CNT == 2 && EXCEPTION == EX_INT && MBIT)
                NEXT_EX_STATE = EXS_SWITCH_STATE; // Build throwaway stack frame.
            else if (DATA_RDY && STACK_CNT == 2)
                NEXT_EX_STATE = EXS_UPDATE_PC;
            else
                NEXT_EX_STATE = EXS_BUILD_STACK;
        end
        EXS_SWITCH_STATE: begin // Required to decrement the correct stack pointer.
            NEXT_EX_STATE = EXS_BUILD_TSTACK;
        end
        EXS_BUILD_TSTACK: begin // Build throwaway stack frame.
            if (ACCESS_ERR)
                NEXT_EX_STATE = EXS_IDLE;
            else if (DATA_RDY && STACK_CNT == 2)
                NEXT_EX_STATE = EXS_UPDATE_PC;
            else
                NEXT_EX_STATE = EXS_BUILD_TSTACK;
        end
        EXS_UPDATE_PC: begin
            if (DOUBLE_BUSFLT)
                NEXT_EX_STATE = EXS_HALTED;
            else if (ACCESS_ERR)
                NEXT_EX_STATE = EXS_IDLE;
            else if (DATA_RDY)
                NEXT_EX_STATE = EXS_REFILL_PIPE;
            else
                NEXT_EX_STATE = EXS_UPDATE_PC;
        end
        EXS_VALIDATE_FRAME: begin
            if (ACCESS_ERR)
                NEXT_EX_STATE = EXS_IDLE;
            else if (DATA_RDY) begin
                case (DATA_IN[15:12])
                    4'h0, 4'h1, 4'h2, 4'h9:
                        NEXT_EX_STATE = EXS_RESTORE_PC;
                    4'hA, 4'hB:
                        NEXT_EX_STATE = EXS_EXAMINE_VERSION;
                    default:
                        NEXT_EX_STATE = EXS_IDLE; // Format error.
                endcase
            end else
                NEXT_EX_STATE = EXS_VALIDATE_FRAME;
        end
        EXS_EXAMINE_VERSION: begin
            if (ACCESS_ERR)
                NEXT_EX_STATE = EXS_IDLE;
            else if (DATA_RDY) begin
                if (DATA_IN != {16'b0, VERSION})
                    NEXT_EX_STATE = EXS_IDLE; // Format error.
                else
                    NEXT_EX_STATE = EXS_READ_TOP;
            end else
                NEXT_EX_STATE = EXS_EXAMINE_VERSION;
        end
        EXS_READ_TOP: begin
            if (ACCESS_ERR)
                NEXT_EX_STATE = EXS_IDLE;
            else if (DATA_RDY)
                NEXT_EX_STATE = EXS_READ_BOTTOM;
            else
                NEXT_EX_STATE = EXS_READ_TOP;
        end
        EXS_READ_BOTTOM: begin
            if (ACCESS_ERR)
                NEXT_EX_STATE = EXS_IDLE;
            else if (DATA_RDY)
                NEXT_EX_STATE = EXS_RESTORE_PC;
            else
                NEXT_EX_STATE = EXS_READ_BOTTOM;
        end
        EXS_RESTORE_STATUS: begin
            if (DOUBLE_BUSFLT)
                NEXT_EX_STATE = EXS_HALTED;
            else if (ACCESS_ERR)
                NEXT_EX_STATE = EXS_IDLE;
            else if (DATA_RDY && STACK_FORMAT_I == 4'h1)
                NEXT_EX_STATE = EXS_VALIDATE_FRAME; // Throwaway stack frame.
            else if (DATA_RDY)
                NEXT_EX_STATE = EXS_REFILL_PIPE;
            else
                NEXT_EX_STATE = EXS_RESTORE_STATUS;
        end
        EXS_RESTORE_ISP: begin
            if (DOUBLE_BUSFLT)
                NEXT_EX_STATE = EXS_HALTED;
            else if (ACCESS_ERR)
                NEXT_EX_STATE = EXS_IDLE;
            else if (DATA_RDY)
                NEXT_EX_STATE = EXS_RESTORE_PC;
            else
                NEXT_EX_STATE = EXS_RESTORE_ISP;
        end
        EXS_RESTORE_PC: begin
            if (DOUBLE_BUSFLT)
                NEXT_EX_STATE = EXS_HALTED; // Double bus fault.
            else if (ACCESS_ERR)
                NEXT_EX_STATE = EXS_IDLE;
            else if (EXCEPTION == EX_RESET_EX && DATA_RDY)
                NEXT_EX_STATE = EXS_REFILL_PIPE;
            else if (DATA_RDY)
                NEXT_EX_STATE = EXS_RESTORE_STATUS;
            else
                NEXT_EX_STATE = EXS_RESTORE_PC;
        end
        EXS_REFILL_PIPE: begin
            if (DOUBLE_BUSFLT)
                NEXT_EX_STATE = EXS_HALTED;
            else if (ACCESS_ERR)
                NEXT_EX_STATE = EXS_IDLE;
            else if (PIPE_FULL)
                NEXT_EX_STATE = EXS_IDLE;
            else
                NEXT_EX_STATE = EXS_REFILL_PIPE;
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
