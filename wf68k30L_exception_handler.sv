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

// ---- Exception handler state machine ----
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

// ---- Exception type identifiers ----
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

typedef enum logic [3:0] {
    TRAP_SRC_NONE    = 4'd0,
    TRAP_SRC_CHK     = 4'd1,
    TRAP_SRC_DIVZERO = 4'd2,
    TRAP_SRC_TRAP    = 4'd3,
    TRAP_SRC_TRAPCC  = 4'd4,
    TRAP_SRC_TRAPV   = 4'd5,
    TRAP_SRC_PRIV    = 4'd6,
    TRAP_SRC_1010    = 4'd7,
    TRAP_SRC_1111    = 4'd8,
    TRAP_SRC_ILLEGAL = 4'd9,
    TRAP_SRC_FORMAT  = 4'd10,
    TRAP_SRC_RTE     = 4'd11
} TRAP_SOURCES;

// ---- Internal signals ----
logic        ACCESS_ERR;
logic        AVEC;
logic        DATA_RD_I;
logic        DATA_WR_I;
logic        DOUBLE_BUSFLT;
EXCEPTIONS   EXCEPTION;     // Currently executed exception.
EX_STATES    EX_STATE;
EX_STATES    NEXT_EX_STATE;

// Pending exception flags (one per exception source).
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

logic [31:0] INT_VECT;      // Interrupt vector address (VBR + vector# * 4).
logic        IBOUND;
logic [2:0]  IRQ;
logic [2:0]  IRQ_PEND_I;
logic        MBIT;
logic [1:0]  PIPE_CNT;
logic        PIPE_FULL;
integer      STACK_CNT;
logic [3:0]  STACK_FORMAT_I;
logic        SYS_INIT;
logic        clear_instruction_traps;
TRAP_SOURCES trap_source;
logic        trap_set_chk;
logic        trap_set_divzero;
logic        trap_set_trap;
logic        trap_set_trapcc;
logic        trap_set_trapv;
logic        trap_set_priv;
logic        trap_set_1010;
logic        trap_set_1111;
logic        trap_set_illegal;
logic        trap_set_format;
logic        trap_set_rte;

// True when any exception pending flag is asserted.
logic        any_exception_pending;

assign any_exception_pending = EX_P_RESET | EX_P_BERR | EX_P_AERR |
                               EX_P_TRAP | EX_P_TRAPcc | EX_P_TRAPV | EX_P_CHK | EX_P_DIVZERO |
                               EX_P_FORMAT |
                               EX_P_TRACE | EX_P_ILLEGAL | EX_P_1010 | EX_P_1111 | EX_P_PRIV |
                               EX_P_RTE | EX_P_INT;

// ---- Exception priority encoder (combinational) ----
// MC68030 exception priority: reset > address/bus error > instruction traps >
// illegal/privileged > trace > external interrupt.
// Returns the highest-priority pending exception type.
function automatic EXCEPTIONS resolve_exception_priority(
    input logic p_reset, p_aerr, p_berr,
    input logic p_chk, p_trapcc, p_divzero, p_trap, p_trapv, p_format,
    input logic p_illegal, p_rte, p_1010, p_1111, p_priv, p_trace,
    input logic p_int
);
    // Priority level 0 (highest): system reset
         if (p_reset)   resolve_exception_priority = EX_RESET_EX;
    // Priority level 1: bus/address faults
    else if (p_aerr)    resolve_exception_priority = EX_AERR;
    else if (p_berr)    resolve_exception_priority = EX_BERR;
    // Priority level 2: instruction-generated traps
    else if (p_chk)     resolve_exception_priority = EX_CHK;
    else if (p_trapcc)  resolve_exception_priority = EX_TRAPcc;
    else if (p_divzero) resolve_exception_priority = EX_DIVZERO;
    else if (p_trap)    resolve_exception_priority = EX_TRAP;
    else if (p_trapv)   resolve_exception_priority = EX_TRAPV;
    else if (p_format)  resolve_exception_priority = EX_FORMAT;
    // Priority level 3: decode-time exceptions and trace
    else if (p_illegal) resolve_exception_priority = EX_ILLEGAL;
    else if (p_rte)     resolve_exception_priority = EX_RTE;
    else if (p_1010)    resolve_exception_priority = EX_1010;
    else if (p_1111)    resolve_exception_priority = EX_1111;
    else if (p_priv)    resolve_exception_priority = EX_PRIV;
    else if (p_trace)   resolve_exception_priority = EX_TRACE;
    // Priority level 4: external interrupt (lowest)
    else if (p_int)     resolve_exception_priority = EX_INT;
    else                resolve_exception_priority = EX_NONE;
endfunction

assign BUSY_EXH = (EX_STATE != EXS_IDLE);

// ---- IRQ input synchronizer (double-sample on negedge for metastability) ----
always_ff @(negedge CLK) begin : irq_filter
    reg [2:0] IRQ_TMP_1;
    reg [2:0] IRQ_TMP_2;
    if (IRQ_TMP_1 == IRQ_TMP_2)
        IRQ <= IRQ_TMP_2;
    IRQ_TMP_2 = IRQ_TMP_1;
    IRQ_TMP_1 = IRQ_IN;
end

// ---- Autovector latch ----
always_ff @(posedge CLK) begin : avec_filter
    if (!AVECn)
        AVEC <= 1'b1;
    else if (DATA_RDY || RESET)
        AVEC <= 1'b0;
end

// ---- Instruction boundary tracker ----
// IBOUND is set at opword acknowledge to distinguish mid-instruction
// faults (format B stack frame) from instruction-boundary faults (format A).
always_ff @(posedge CLK) begin : instruction_boundary
    if (RESET)
        IBOUND <= 1'b0;
    else if (OPD_ACK && !OW_VALID)
        IBOUND <= 1'b1;
    else if (EX_STATE == EXS_BUILD_STACK)
        IBOUND <= 1'b0;
end

// ---- Exception pending flag management ----
// Each exception source sets its flag independently; flags are cleared
// when the exception handler begins processing or at system init.
always_ff @(posedge CLK) begin : pending_system_faults
    // Reset exception
    if (RESET)
        EX_P_RESET <= 1'b1;
    else if (EX_STATE == EXS_RESTORE_PC && DATA_RDY && EXCEPTION == EX_RESET_EX)
        EX_P_RESET <= 1'b0;

    // Bus error
    if (TRAP_BERR)
        EX_P_BERR <= 1'b1;
    else if (EX_STATE != EXS_IDLE && DATA_RDY && !DATA_VALID)
        EX_P_BERR <= 1'b1;
    else if (EX_STATE == EXS_INIT && EXCEPTION == EX_BERR)
        EX_P_BERR <= 1'b0; // Reset in the beginning to enable retriggering.
    else if (SYS_INIT)
        EX_P_BERR <= 1'b0;

    // Address error
    if (TRAP_AERR)
        EX_P_AERR <= 1'b1;
    else if (EX_STATE == EXS_BUILD_STACK && EXCEPTION == EX_AERR)
        EX_P_AERR <= 1'b0;
    else if (SYS_INIT)
        EX_P_AERR <= 1'b0;

    // Trace
    if (EX_TRACE_IN)
        EX_P_TRACE <= 1'b1;
    else if (EX_STATE == EXS_BUILD_STACK && EXCEPTION == EX_TRACE)
        EX_P_TRACE <= 1'b0;
    else if (SYS_INIT)
        EX_P_TRACE <= 1'b0;
end

always_ff @(posedge CLK) begin : pending_interrupts
    reg INT7_TRIG;
    reg [2:0] INT_VAR;
    reg [2:0] SR_VAR;

    // Detect level-7 NMI edge: either mask lowered from 7, or level-7 newly asserted.
    if (IRQ == 3'b111 && SR_VAR == 3'b111 && STATUS_REG_IN[10:8] != 3'b111)
        INT7_TRIG = 1'b1;
    else if (IRQ == 3'b111 && INT_VAR < 3'b111)
        INT7_TRIG = 1'b1;
    else
        INT7_TRIG = 1'b0;

    SR_VAR = STATUS_REG_IN[10:8]; // Update after use!
    INT_VAR = IRQ; // Update after use!

    if (SYS_INIT) begin
        EX_P_INT <= 1'b0;
        IRQ_PEND_I <= 3'b111; // Required for system startup.
    end else if (EX_STATE == EXS_GET_VECTOR && DATA_RDY) begin
        EX_P_INT <= 1'b0;
    end else if (INT7_TRIG) begin // Level 7 is nonmaskable.
        EX_P_INT <= 1'b1;
        IRQ_PEND_I <= IRQ;
    end else if (INT_TRIG && STATUS_REG_IN[10:8] < IRQ) begin
        EX_P_INT <= 1'b1;
        IRQ_PEND_I <= IRQ;
    end
end

always_comb begin : decode_instruction_trap_source
    // Preserve original priority between trap sources.
    trap_source = TRAP_SRC_NONE;
    if (TRAP_CHK)
        trap_source = TRAP_SRC_CHK;
    else if (TRAP_DIVZERO)
        trap_source = TRAP_SRC_DIVZERO;
    else if (TRAP_CODE_OPC == T_TRAP)
        trap_source = TRAP_SRC_TRAP;
    else if (TRAP_cc)
        trap_source = TRAP_SRC_TRAPCC;
    else if (TRAP_V)
        trap_source = TRAP_SRC_TRAPV;
    else if (TRAP_CODE_OPC == T_PRIV)
        trap_source = TRAP_SRC_PRIV;
    else if (TRAP_CODE_OPC == T_1010)
        trap_source = TRAP_SRC_1010;
    else if (TRAP_CODE_OPC == T_1111)
        trap_source = TRAP_SRC_1111;
    else if (TRAP_CODE_OPC == T_ILLEGAL || TRAP_ILLEGAL)
        trap_source = TRAP_SRC_ILLEGAL;
    else if ((EX_STATE == EXS_VALIDATE_FRAME && DATA_RDY && DATA_VALID && NEXT_EX_STATE == EXS_IDLE) ||
             (EX_STATE == EXS_EXAMINE_VERSION && DATA_RDY && DATA_VALID && NEXT_EX_STATE == EXS_IDLE))
        trap_source = TRAP_SRC_FORMAT;
    else if (TRAP_CODE_OPC == T_RTE)
        trap_source = TRAP_SRC_RTE;
end

always_comb begin : decode_instruction_trap_clear
    clear_instruction_traps = SYS_INIT;
    if (EX_STATE == EXS_REFILL_PIPE && NEXT_EX_STATE != EXS_REFILL_PIPE &&
        (EXCEPTION == EX_1010 || EXCEPTION == EX_1111 || EXCEPTION == EX_CHK ||
         EXCEPTION == EX_DIVZERO || EXCEPTION == EX_ILLEGAL || EXCEPTION == EX_TRAP ||
         EXCEPTION == EX_TRAPcc || EXCEPTION == EX_TRAPV || EXCEPTION == EX_FORMAT ||
         EXCEPTION == EX_PRIV || EXCEPTION == EX_RTE))
        clear_instruction_traps = 1'b1;
end

always_comb begin : decode_instruction_trap_set
    trap_set_chk     = 1'b0;
    trap_set_divzero = 1'b0;
    trap_set_trap    = 1'b0;
    trap_set_trapcc  = 1'b0;
    trap_set_trapv   = 1'b0;
    trap_set_priv    = 1'b0;
    trap_set_1010    = 1'b0;
    trap_set_1111    = 1'b0;
    trap_set_illegal = 1'b0;
    trap_set_format  = 1'b0;
    trap_set_rte     = 1'b0;

    case (trap_source)
        TRAP_SRC_CHK:     trap_set_chk = 1'b1;
        TRAP_SRC_DIVZERO: trap_set_divzero = 1'b1;
        TRAP_SRC_TRAP:    trap_set_trap = 1'b1;
        TRAP_SRC_TRAPCC:  trap_set_trapcc = 1'b1;
        TRAP_SRC_TRAPV:   trap_set_trapv = 1'b1;
        TRAP_SRC_PRIV:    trap_set_priv = 1'b1;
        TRAP_SRC_1010:    trap_set_1010 = 1'b1;
        TRAP_SRC_1111:    trap_set_1111 = 1'b1;
        TRAP_SRC_ILLEGAL: trap_set_illegal = 1'b1;
        TRAP_SRC_FORMAT:  trap_set_format = 1'b1;
        TRAP_SRC_RTE:     trap_set_rte = 1'b1;
        default: ;
    endcase
end

always_ff @(posedge CLK) begin : pending_instruction_traps
    EX_P_CHK     <= trap_set_chk     ? 1'b1 : (clear_instruction_traps ? 1'b0 : EX_P_CHK);
    EX_P_DIVZERO <= trap_set_divzero ? 1'b1 : (clear_instruction_traps ? 1'b0 : EX_P_DIVZERO);
    EX_P_TRAP    <= trap_set_trap    ? 1'b1 : (clear_instruction_traps ? 1'b0 : EX_P_TRAP);
    EX_P_TRAPcc  <= trap_set_trapcc  ? 1'b1 : (clear_instruction_traps ? 1'b0 : EX_P_TRAPcc);
    EX_P_TRAPV   <= trap_set_trapv   ? 1'b1 : (clear_instruction_traps ? 1'b0 : EX_P_TRAPV);
    EX_P_PRIV    <= trap_set_priv    ? 1'b1 : (clear_instruction_traps ? 1'b0 : EX_P_PRIV);
    EX_P_1010    <= trap_set_1010    ? 1'b1 : (clear_instruction_traps ? 1'b0 : EX_P_1010);
    EX_P_1111    <= trap_set_1111    ? 1'b1 : (clear_instruction_traps ? 1'b0 : EX_P_1111);
    EX_P_ILLEGAL <= trap_set_illegal ? 1'b1 : (clear_instruction_traps ? 1'b0 : EX_P_ILLEGAL);
    EX_P_FORMAT  <= trap_set_format  ? 1'b1 : (clear_instruction_traps ? 1'b0 : EX_P_FORMAT);
    EX_P_RTE     <= trap_set_rte     ? 1'b1 : (clear_instruction_traps ? 1'b0 : EX_P_RTE);
end

// ---- Derived status signals ----

assign ACCESS_ERR = (EX_STATE == EXS_RESTORE_PC && DATA_RDY && DATA_0) || // Odd PC value.
                    (DATA_RDY && !DATA_VALID); // Bus error.

assign IRQ_PEND = (EXCEPTION == EX_RESET_EX || EXCEPTION == EX_INT) ? IRQ_PEND_I : STATUS_REG_IN[10:8];
assign IPENDn = !(EX_P_INT || EX_P_RESET || EX_P_TRACE);

assign EXH_REQ = (EX_STATE != EXS_IDLE) ? 1'b0 :
                 (TRAP_CODE_OPC != NONE) ? 1'b1 :
                 any_exception_pending;

// ---- Vector number calculation ----
// Maps exception type to MC68030 vector number, then computes the
// vector table address as VBR + (vector# * 4).
always_ff @(posedge CLK) begin : int_vector
    reg [7:0] VECT_No;
    reg [31:0] VB_REG;

    if (VBR_WR)
        VB_REG = DATA_IN;
    else if (SYS_INIT)
        VB_REG = 32'h0;

    if (EX_STATE == EXS_CALC_VECT_NO || EX_STATE == EXS_GET_VECTOR) begin
        case (EXCEPTION)
            EX_RESET_EX: VECT_No = VEC_RESET;
            EX_BERR:     VECT_No = VEC_BUS_ERROR;
            EX_AERR:     VECT_No = VEC_ADDR_ERROR;
            EX_ILLEGAL:  VECT_No = VEC_ILLEGAL;
            EX_DIVZERO:  VECT_No = VEC_DIVZERO;
            EX_CHK:      VECT_No = VEC_CHK;
            EX_TRAPcc:   VECT_No = VEC_TRAPCC;
            EX_TRAPV:    VECT_No = VEC_TRAPCC;
            EX_PRIV:     VECT_No = VEC_PRIV;
            EX_TRACE:    VECT_No = VEC_TRACE;
            EX_1010:     VECT_No = VEC_LINE_A;
            EX_1111:     VECT_No = VEC_LINE_F;
            EX_FORMAT:   VECT_No = VEC_FORMAT;
            EX_INT: begin
                if (DATA_RDY && AVEC)
                    VECT_No = VEC_SPURIOUS + {5'b0, IRQ_PEND_I}; // Autovector.
                else if (DATA_RDY && !DATA_VALID)
                    VECT_No = VEC_SPURIOUS; // Spurious interrupt.
                else if (DATA_RDY)
                    VECT_No = DATA_IN[7:0]; // Non-autovector (device supplies vector).
            end
            EX_TRAP:     VECT_No = VEC_TRAP_BASE + {4'h0, TRAP_VECTOR};
            default:     VECT_No = 8'hxx; // Don't care.
        endcase
    end

    INT_VECT <= VB_REG + {22'b0, VECT_No, 2'b00};
    VBR <= VB_REG;
    IVECT_OFFS <= {VECT_No, 2'b00};
end

// ---- Exception priority encoder (registered) ----
// Latches the highest-priority pending exception when entering from idle.
always_ff @(posedge CLK) begin : store_current_exception
    if (EX_STATE == EXS_IDLE)
        EXCEPTION <= resolve_exception_priority(
            EX_P_RESET, EX_P_AERR, EX_P_BERR,
            EX_P_CHK, EX_P_TRAPcc, EX_P_DIVZERO, EX_P_TRAP, EX_P_TRAPV, EX_P_FORMAT,
            EX_P_ILLEGAL, EX_P_RTE, EX_P_1010, EX_P_1111, EX_P_PRIV, EX_P_TRACE,
            EX_P_INT
        );
    else if (NEXT_EX_STATE == EXS_IDLE)
        EXCEPTION <= EX_NONE;
end

// ---- CPU space and address offset routing ----

assign CPU_SPACE = (NEXT_EX_STATE == EXS_GET_VECTOR);

assign ADR_OFFSET = (EX_STATE == EXS_REFILL_PIPE) ? {24'h0, 5'b0, PIPE_CNT, 1'b0} :
                    (NEXT_EX_STATE == EXS_RESTORE_PC && EXCEPTION == EX_RESET_EX) ? 32'h4 :
                    (NEXT_EX_STATE == EXS_RESTORE_PC) ? 32'h2 :
                    (NEXT_EX_STATE == EXS_VALIDATE_FRAME) ? 32'h6 :
                    (NEXT_EX_STATE == EXS_EXAMINE_VERSION) ? 32'h36 :
                    (NEXT_EX_STATE == EXS_READ_BOTTOM) ? 32'h5C :
                    (NEXT_EX_STATE == EXS_UPDATE_PC) ? INT_VECT : 32'h0;

// ---- Operand size for bus transactions ----

assign OP_SIZE = (EX_STATE == EXS_INIT) ? LONG : // Decrement the stack by four (ISP_DEC).
                 (NEXT_EX_STATE == EXS_RESTORE_ISP || NEXT_EX_STATE == EXS_RESTORE_PC) ? LONG :
                 (NEXT_EX_STATE == EXS_BUILD_STACK || NEXT_EX_STATE == EXS_BUILD_TSTACK) ? LONG :
                 (NEXT_EX_STATE == EXS_UPDATE_PC || EX_STATE == EXS_UPDATE_PC) ? LONG :
                 (EX_STATE == EXS_SWITCH_STATE) ? LONG :
                 (NEXT_EX_STATE == EXS_GET_VECTOR) ? BYTE : WORD;

// ---- Stack frame displacement lookup ----
always_comb begin : stack_frame_displacement
    case (STACK_FORMAT_I)
        4'h0, 4'h1: DISPLACEMENT = 8'h08; // Format 0/1: 4-word frame.
        4'h2:       DISPLACEMENT = 8'h0C; // Format 2: 6-word frame.
        4'h9:       DISPLACEMENT = 8'h12; // Format 9: coprocessor mid-instruction.
        4'hA:       DISPLACEMENT = 8'h20; // Format A: short bus/address error.
        default:    DISPLACEMENT = 8'h5C; // Format B: long bus/address error.
    endcase
end

assign SP_ADD_DISPL = EX_STATE == EXS_RESTORE_STATUS && DATA_RDY && DATA_VALID;

// ---- Data bus read/write control (async clear on DATA_RDY) ----
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

// ---- Stack pointer, PC, and status register control ----

assign ISP_LOAD = EX_STATE == EXS_RESTORE_ISP && DATA_RDY && DATA_VALID;
assign PC_RESTORE = EX_STATE == EXS_RESTORE_PC && DATA_RDY && DATA_VALID;
assign PC_LOAD = EXCEPTION != EX_RESET_EX && EXCEPTION != EX_RTE && EX_STATE != EXS_REFILL_PIPE && NEXT_EX_STATE == EXS_REFILL_PIPE;

assign IPIPE_FILL = (EX_STATE == EXS_REFILL_PIPE);

// Increment PC before stacking for exceptions that report the next instruction address.
assign PC_INC = (EX_STATE != EXS_BUILD_STACK && NEXT_EX_STATE == EXS_BUILD_STACK) &&
                (EXCEPTION == EX_CHK || EXCEPTION == EX_DIVZERO || EXCEPTION == EX_INT ||
                 EXCEPTION == EX_TRAP || EXCEPTION == EX_TRAPcc || EXCEPTION == EX_TRAPV);

assign ISP_DEC = (EX_STATE == EXS_INIT && EXCEPTION != EX_RESET_EX && EXCEPTION != EX_RTE) ||
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

// ---- Double bus fault detection ----
// A double fault halts the processor: bus error during bus/address error or reset processing.
assign DOUBLE_BUSFLT = ((EXCEPTION == EX_AERR || EXCEPTION == EX_RESET_EX) && EX_STATE == EXS_RESTORE_PC && DATA_RDY && DATA_0) ||
                       (EX_STATE != EXS_IDLE && EXCEPTION == EX_AERR && DATA_RDY && !DATA_VALID) ||
                       (EX_STATE != EXS_IDLE && EXCEPTION == EX_BERR && DATA_RDY && !DATA_VALID) ||
                       (EX_STATE != EXS_IDLE && EXCEPTION == EX_RESET_EX && DATA_RDY && !DATA_VALID);

// ---- Snapshot status register and address on exception entry ----
always_ff @(posedge CLK) begin : tmp_cpy
    if (EX_STATE == EXS_IDLE && NEXT_EX_STATE != EXS_IDLE) begin
        SR_CPY <= STATUS_REG_IN;
        ADR_CPY <= ADR_IN;
        MBIT <= STATUS_REG_IN[12];
    end else if (EX_STATE == EXS_BUILD_STACK && NEXT_EX_STATE == EXS_SWITCH_STATE) begin
        SR_CPY[13] <= 1'b1; // Set S bit for master->interrupt stack switch.
    end
end

// ---- Stack frame builder ----
// Selects the stack frame format based on exception type and manages
// the stack position counter that drives the stacking FSM.
always_ff @(posedge CLK) begin : stack_ctrl
    reg [5:0] STACK_POS_VAR;
    if (EX_STATE != EXS_BUILD_TSTACK && NEXT_EX_STATE == EXS_BUILD_TSTACK) begin
        STACK_POS_VAR = 6'd4;
        STACK_FORMAT_I <= 4'h1; // Throwaway frame (format 1).
    end else if (EX_STATE != EXS_BUILD_STACK && NEXT_EX_STATE == EXS_BUILD_STACK) begin
        case (EXCEPTION)
            EX_INT, EX_ILLEGAL, EX_1010, EX_1111, EX_FORMAT, EX_PRIV, EX_TRAP: begin
                STACK_POS_VAR = 6'd4;   // Format 0: 4-word stack frame.
                STACK_FORMAT_I <= 4'h0;
            end
            EX_CHK, EX_TRAPcc, EX_TRAPV, EX_TRACE, EX_DIVZERO: begin
                STACK_POS_VAR = 6'd6;   // Format 2: 6-word stack frame.
                STACK_FORMAT_I <= 4'h2;
            end
            EX_AERR, EX_BERR: begin
                if (IBOUND) begin
                    STACK_POS_VAR = 6'd16; // Format A: short bus fault frame.
                    STACK_FORMAT_I <= 4'hA;
                end else begin
                    STACK_POS_VAR = 6'd46; // Format B: long bus fault frame.
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

    STACK_CNT <= STACK_POS_VAR;
    STACK_POS <= STACK_POS_VAR;
end

// ---- STATUSn output timing ----
always_ff @(posedge CLK) begin : status_out
    reg [1:0] CNT;
    if (EX_STATE == EXS_CALC_VECT_NO) begin
        case (EXCEPTION)
            EX_RESET_EX, EX_AERR, EX_BERR, EX_1111: begin
                CNT = 2'b11;
                STATUSn <= 1'b0;
            end
            EX_INT, EX_TRACE:
                CNT = 2'b10;
            default:
                CNT = 2'b00;
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

// ---- Instruction pipe refill tracker ----
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

// ---- Exception handler state register ----
always_ff @(posedge CLK) begin : exception_handler_reg
    if (RESET)
        EX_STATE <= EXS_IDLE;
    else
        EX_STATE <= NEXT_EX_STATE;
end

// ---- Exception handler next-state logic ----
always_comb begin : exception_handler_dec
    case (EX_STATE)
        EXS_IDLE: begin
            if ((BUSY_MAIN || BUSY_OPD) && !EX_P_RESET)
                NEXT_EX_STATE = EXS_IDLE; // Wait until the pipeline is idle.
            else if (any_exception_pending)
                NEXT_EX_STATE = EXS_INIT;
            else
                NEXT_EX_STATE = EXS_IDLE;
        end

        EXS_INIT: begin
            case (EXCEPTION)
                EX_RTE:  NEXT_EX_STATE = EXS_VALIDATE_FRAME;
                EX_INT:  NEXT_EX_STATE = EXS_GET_VECTOR;
                default: NEXT_EX_STATE = EXS_CALC_VECT_NO;
            endcase
        end

        EXS_GET_VECTOR:
            NEXT_EX_STATE = DATA_RDY ? EXS_BUILD_STACK : EXS_GET_VECTOR;

        EXS_CALC_VECT_NO:
            // Reset: skip stacking, go directly to restore ISP and PC.
            NEXT_EX_STATE = (EXCEPTION == EX_RESET_EX) ? EXS_RESTORE_ISP : EXS_BUILD_STACK;

        EXS_BUILD_STACK: begin
            if (DOUBLE_BUSFLT)
                NEXT_EX_STATE = EXS_HALTED;
            else if (ACCESS_ERR)
                NEXT_EX_STATE = EXS_IDLE;
            else if (DATA_RDY && STACK_CNT == 2 && EXCEPTION == EX_INT && MBIT)
                NEXT_EX_STATE = EXS_SWITCH_STATE; // Master->interrupt stack switch.
            else if (DATA_RDY && STACK_CNT == 2)
                NEXT_EX_STATE = EXS_UPDATE_PC;
            else
                NEXT_EX_STATE = EXS_BUILD_STACK;
        end

        EXS_SWITCH_STATE: // Decrement the correct stack pointer before throwaway frame.
            NEXT_EX_STATE = EXS_BUILD_TSTACK;

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

        EXS_READ_TOP:
            NEXT_EX_STATE = ACCESS_ERR ? EXS_IDLE :
                            DATA_RDY   ? EXS_READ_BOTTOM : EXS_READ_TOP;

        EXS_READ_BOTTOM:
            NEXT_EX_STATE = ACCESS_ERR ? EXS_IDLE :
                            DATA_RDY   ? EXS_RESTORE_PC : EXS_READ_BOTTOM;

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
                NEXT_EX_STATE = EXS_HALTED;
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

        EXS_HALTED: // Processor halted on double bus fault.
            NEXT_EX_STATE = EXS_HALTED;

        default:
            NEXT_EX_STATE = EXS_IDLE;
    endcase
end

endmodule
