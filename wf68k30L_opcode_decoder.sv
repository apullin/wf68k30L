// ------------------------------------------------------------------------
// --                                                                    --
// -- WF68K30L IP Core: this is the bus controller module.               --
// --                                                                    --
// -- Description:                                                       --
// -- This module is a 68030 compatible instruction word decoder.        --
// --                                                                    --
// -- It is primarily controlled by the following signals:               --
// -- OW_REQ, OPD_ACK, EW_REQ and EW_ACK. The handshaking is as         --
// -- follows: if a new instruction is required, assert the signal       --
// -- OW_REQ and wait until ACK is asserted by the decoder logic.        --
// -- Deassert OW_REQ right after ACK (in the same clock cycle).         --
// -- At this point, the required instruction has already been copied    --
// -- from the pipe to the register BIW_0. The respective additional    --
// -- instruction words are located in BIW_1, BIW_2. For more infor-    --
// -- mation see the 68010 "Programmers Reference Manual" and the       --
// -- signal INSTR_LVL in this module.                                   --
// -- The extension request works in the same manner by asserting        --
// -- EW_REQ. At the time of EXT_ACK one extension word has been         --
// -- copied to EXT_WORD.                                                --
// -- Be aware that it is in the scope of the logic driving              --
// -- OW_REQ and EW_REQ to hold the instruction pipe aligned.            --
// -- This means in detail, that the correct number or instruction       --
// -- and extension words must be requested. Otherwise unpredictable    --
// -- processor behaviour will occur. Furthermore OW_REQ and EW_REQ    --
// -- must not be asserted the same time.                                --
// -- This operation code decoder with the handshake logic as des-       --
// -- cribed above is the first pipeline stage of the CPU architec-      --
// -- ture.                                                              --
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
// Revision 2K18A 20180620 WF
//   Removed REST_BIW_0.
//   Fixed the PW_EW_OFFSET calculation for JSR.
//   TOP, CONTROL, Exception Handler Opcode Decoder: Rearranged PC_INC and ipipe flush logic.
//   Fix for unimplemented or illegal operations: PC is increased before stacked.
//   Implemented the 68K10 loop mechanism.
//   Removed CAHR, we have no cache.
//   Rearranged address error handling.
//

module WF68K30L_OPCODE_DECODER #(
    parameter NO_LOOP = 0  // If 1 the DBcc loop mechanism is disabled.
)(
    input  logic        CLK,

    input  logic        OW_REQ_MAIN,    // Request from the execution unit.
    input  logic        EW_REQ_MAIN,    // Extension words request.

    input  logic        EXH_REQ,        // Exception request.
    input  logic        BUSY_EXH,       // Exception handler is busy.
    input  logic        BUSY_MAIN,      // Main controller busy.
    output logic        BUSY_OPD,       // This unit is busy.

    input  logic        BKPT_INSERT,
    input  logic [15:0] BKPT_DATA,

    input  logic        LOOP_EXIT,
    output logic        LOOP_BSY,

    output logic        OPD_ACK_MAIN,   // Operation controller acknowledge.
    output logic        EW_ACK,         // Extension word available.

    output logic        PC_INC,
    input  logic        PC_INC_EXH,
    output logic [7:0]  PC_ADR_OFFSET,
    output logic [3:0]  PC_EW_OFFSET,
    output logic [7:0]  PC_OFFSET,

    output logic        OPCODE_RD,
    input  logic        OPCODE_RDY,
    input  logic        OPCODE_VALID,
    input  logic [15:0] OPCODE_DATA,

    input  logic        IPIPE_FILL,
    input  logic        IPIPE_FLUSH,    // Abandon the instruction pipe.

    // Fault logic:
    output logic        OW_VALID,       // Operation words valid.
    output logic        RC,             // Rerun flag on instruction pipe stage C.
    output logic        RB,             // Rerun flag on instruction pipe stage B.
    output logic        FC,             // Fault on use of instruction pipe stage C.
    output logic        FB,             // Fault on use of instruction pipe stage B.

    // Trap logic:
    input  logic        SBIT,
    output logic [2:0]  TRAP_CODE,

    // System control signals:
    output logic [6:0]  OP,
    output logic [15:0] BIW_0,
    output logic [15:0] BIW_1,
    output logic [15:0] BIW_2,
    output logic [15:0] EXT_WORD
);

`include "wf68k30L_pkg.svh"

typedef enum logic [1:0] {
    LVL_D = 2'd0,
    LVL_C = 2'd1,
    LVL_B = 2'd2
} INSTR_LVL_TYPE;

logic        REQ;
logic        EW_REQ;

// Instruction pipe - record fields become separate signals
logic [15:0] IPIPE_D;
logic [15:0] IPIPE_C;
logic [15:0] IPIPE_B;

logic        FIFO_RD;
logic        IPIPE_B_FAULT;
logic        IPIPE_C_FAULT;
logic        IPIPE_D_FAULT;
logic [1:0]  IPIPE_PNTR;     // 0 to 3

INSTR_LVL_TYPE INSTR_LVL;
logic        LOOP_ATN;
logic        LOOP_BSY_I;
logic        LOOP_OP;

logic        BKPT_REQ;

OP_68K       OP_I;

logic        OPCODE_FLUSH;
logic        OPCODE_RD_I;
logic        OPCODE_RDY_I;
logic        OW_REQ;

TRAPTYPE_OPC TRAP_CODE_I;
logic        FLUSHED;
logic        PC_INC_I;
logic        PIPE_RDY;
logic [6:0]  ADR_OFFSET_S;
logic [6:0]  PC_VAR_S;
logic [6:0]  PC_VAR_MEM_S;

// P_BSY: This logic requires asynchronous reset. This flip flop is intended
// to break combinatorial loops. If an opcode cycle in the bus controller
// unit is currently running, the actual PC address is stored during this
// cycle. Therefore it is not possible to flush the pipe and manipulate
// the PC during a running cycle. For the exception handler reading the
// opcode is inhibited during a pipe flush. For the main controller unit
// and the coprocessor interface the pipe is flushed after a running
// opcode cycle.
// Restructured for Yosys: use synchronous logic with priority encoding
// to emulate the multi-edge VHDL process.
always_ff @(posedge CLK) begin : opcode_rd_ctrl
    if (OPCODE_RDY)
        OPCODE_RD_I <= 1'b0;
    else if (BUSY_EXH && !IPIPE_FILL)
        OPCODE_RD_I <= 1'b0;
    else if (IPIPE_FLUSH)
        OPCODE_RD_I <= 1'b1;
    else if ((LOOP_ATN && !OPCODE_RD_I) || LOOP_BSY_I)
        OPCODE_RD_I <= 1'b0;
    else if (IPIPE_PNTR < 2'd3)
        OPCODE_RD_I <= 1'b1;
end

// P_OPCODE_FLUSH
always_ff @(posedge CLK) begin : opcode_flush
    if (IPIPE_FLUSH && OPCODE_RD_I && !OPCODE_RDY)
        OPCODE_FLUSH <= 1'b1;
    else if (OPCODE_RDY || BUSY_EXH)
        OPCODE_FLUSH <= 1'b0;
end

assign OPCODE_RD = OPCODE_RD_I;
assign OPCODE_RDY_I = OPCODE_FLUSH ? 1'b0 : OPCODE_RDY; // Dismiss the current read cycle.
assign BUSY_OPD = (EXH_REQ && !BUSY_MAIN && IPIPE_PNTR > 2'd0 && !OPCODE_RD_I) ? 1'b0 : // Fill one opcode is sufficient here.
                  (IPIPE_PNTR < 2'd3 || OPCODE_RD_I) ? 1'b1 : 1'b0;

// INSTRUCTION_PIPE
always_ff @(posedge CLK) begin : instruction_pipe
    reg [15:0] IPIPE_D_VAR;
    if (IPIPE_FLUSH) begin
        IPIPE_D <= 16'h0;
        IPIPE_C <= 16'h0;
        IPIPE_B <= 16'h0;
        IPIPE_PNTR <= 2'd0;
    end else if (BKPT_INSERT) begin
        IPIPE_D_VAR = IPIPE_D;
        IPIPE_D <= BKPT_DATA; // Insert the breakpoint data.
        BKPT_REQ <= 1'b1;
    end else if (OW_REQ && BKPT_REQ) begin
        IPIPE_D <= IPIPE_D_VAR; // Restore from breakpoint.
        BKPT_REQ <= 1'b0;
    end else if (LOOP_ATN && OPCODE_RD_I) begin
        ; // Wait for pending opcodes.
    end else if (OW_REQ && PIPE_RDY && OP_I == DBcc && LOOP_OP && IPIPE_C == 16'hFFFC) begin // Initialize the loop.
        IPIPE_D <= BIW_0; // This is the LEVEL D operation for the loop.
    end else if (OW_REQ && LOOP_BSY_I) begin
        IPIPE_D <= BIW_0; // Recycle the loop operations.
    end else if (LOOP_BSY_I) begin
        ; // Do not change the pipe during the loop.
    end else if (OW_REQ && INSTR_LVL == LVL_D && PIPE_RDY && IPIPE_PNTR == 2'd2) begin
        if (OPCODE_RDY_I) begin
            IPIPE_D <= IPIPE_C;
            IPIPE_C <= OPCODE_DATA;
            IPIPE_D_FAULT <= IPIPE_C_FAULT;
            IPIPE_C_FAULT <= ~OPCODE_VALID;
        end else begin
            IPIPE_D <= IPIPE_C;
            IPIPE_D_FAULT <= IPIPE_C_FAULT;
            IPIPE_PNTR <= IPIPE_PNTR - 2'd1;
        end
    end else if (OW_REQ && INSTR_LVL == LVL_D && PIPE_RDY && IPIPE_PNTR == 2'd3) begin
        if (OPCODE_RDY_I) begin
            IPIPE_D <= IPIPE_C;
            IPIPE_C <= IPIPE_B;
            IPIPE_B <= OPCODE_DATA;
            IPIPE_D_FAULT <= IPIPE_C_FAULT;
            IPIPE_C_FAULT <= IPIPE_B_FAULT;
            IPIPE_B_FAULT <= ~OPCODE_VALID;
        end else begin
            IPIPE_D <= IPIPE_C;
            IPIPE_C <= IPIPE_B;
            IPIPE_D_FAULT <= IPIPE_C_FAULT;
            IPIPE_C_FAULT <= IPIPE_B_FAULT;
            IPIPE_PNTR <= IPIPE_PNTR - 2'd1;
        end
    end else if (OW_REQ && INSTR_LVL == LVL_C && PIPE_RDY && IPIPE_PNTR == 2'd2) begin
        if (OPCODE_RDY_I) begin
            IPIPE_D <= OPCODE_DATA;
            IPIPE_D_FAULT <= ~OPCODE_VALID;
            IPIPE_PNTR <= IPIPE_PNTR - 2'd1;
        end else begin
            IPIPE_PNTR <= 2'd0;
        end
    end else if (OW_REQ && INSTR_LVL == LVL_C && PIPE_RDY && IPIPE_PNTR == 2'd3) begin
        if (OPCODE_RDY_I) begin
            IPIPE_D <= IPIPE_B;
            IPIPE_C <= OPCODE_DATA;
            IPIPE_D_FAULT <= IPIPE_B_FAULT;
            IPIPE_C_FAULT <= ~OPCODE_VALID;
            IPIPE_PNTR <= IPIPE_PNTR - 2'd1;
        end else begin
            IPIPE_D <= IPIPE_B;
            IPIPE_D_FAULT <= IPIPE_B_FAULT;
            IPIPE_PNTR <= IPIPE_PNTR - 2'd2;
        end
    end else if (OW_REQ && INSTR_LVL == LVL_B && PIPE_RDY) begin // IPIPE_PNTR = 3.
        if (OPCODE_RDY_I) begin
            IPIPE_D <= OPCODE_DATA;
            IPIPE_D_FAULT <= ~OPCODE_VALID;
            IPIPE_PNTR <= IPIPE_PNTR - 2'd2;
        end else begin
            IPIPE_PNTR <= 2'd0;
        end
    end else if (EW_REQ && IPIPE_PNTR >= 2'd1) begin
        case (IPIPE_PNTR)
            2'd3: begin
                if (OPCODE_RDY_I) begin
                    IPIPE_D <= IPIPE_C;
                    IPIPE_C <= IPIPE_B;
                    IPIPE_B <= OPCODE_DATA;
                    IPIPE_D_FAULT <= IPIPE_C_FAULT;
                    IPIPE_C_FAULT <= IPIPE_B_FAULT;
                    IPIPE_B_FAULT <= ~OPCODE_VALID;
                end else begin
                    IPIPE_D <= IPIPE_C;
                    IPIPE_C <= IPIPE_B;
                    IPIPE_D_FAULT <= IPIPE_C_FAULT;
                    IPIPE_C_FAULT <= IPIPE_B_FAULT;
                    IPIPE_PNTR <= IPIPE_PNTR - 2'd1;
                end
            end
            2'd2: begin
                if (OPCODE_RDY_I) begin
                    IPIPE_D <= IPIPE_C;
                    IPIPE_C <= OPCODE_DATA;
                    IPIPE_D_FAULT <= IPIPE_C_FAULT;
                    IPIPE_C_FAULT <= ~OPCODE_VALID;
                end else begin
                    IPIPE_D <= IPIPE_C;
                    IPIPE_D_FAULT <= IPIPE_C_FAULT;
                    IPIPE_PNTR <= IPIPE_PNTR - 2'd1;
                end
            end
            2'd1: begin
                if (OPCODE_RDY_I) begin
                    IPIPE_D <= OPCODE_DATA;
                    IPIPE_D_FAULT <= ~OPCODE_VALID;
                end else begin
                    IPIPE_PNTR <= 2'd0;
                end
            end
            default: ;
        endcase
    end else if (OPCODE_RDY_I) begin
        case (IPIPE_PNTR)
            2'd2: begin
                IPIPE_B <= OPCODE_DATA;
                IPIPE_B_FAULT <= ~OPCODE_VALID;
                IPIPE_PNTR <= 2'd3;
            end
            2'd1: begin
                IPIPE_C <= OPCODE_DATA;
                IPIPE_C_FAULT <= ~OPCODE_VALID;
                IPIPE_PNTR <= 2'd2;
            end
            2'd0: begin
                IPIPE_D <= OPCODE_DATA;
                IPIPE_D_FAULT <= ~OPCODE_VALID;
                IPIPE_PNTR <= 2'd1;
            end
            default: ;
        endcase
    end
end

// P_FAULT
always_ff @(posedge CLK) begin : fault_tracking
    if (IPIPE_FLUSH) begin
        OW_VALID <= 1'b0;
        FC <= 1'b0;
        FB <= 1'b0;
    end else if (OW_REQ && LOOP_BSY_I) begin
        OW_VALID <= 1'b1;
    end else if (OW_REQ && PIPE_RDY && INSTR_LVL == LVL_D) begin
        OW_VALID <= ~IPIPE_D_FAULT;
        FC <= 1'b0;
        FB <= 1'b0;
    end else if (OW_REQ && PIPE_RDY && INSTR_LVL == LVL_C) begin
        OW_VALID <= ~(IPIPE_D_FAULT | IPIPE_C_FAULT);
        FC <= IPIPE_C_FAULT;
        FB <= 1'b0;
    end else if (OW_REQ && PIPE_RDY && INSTR_LVL == LVL_B) begin
        OW_VALID <= ~(IPIPE_D_FAULT | IPIPE_C_FAULT | IPIPE_B_FAULT);
        FC <= IPIPE_C_FAULT;
        FB <= IPIPE_B_FAULT;
    end else if (EW_REQ && PIPE_RDY) begin
        OW_VALID <= ~IPIPE_D_FAULT;
        FC <= 1'b0;
        FB <= 1'b0;
    end
    // The Rerun Flags:
    if (IPIPE_FLUSH) begin
        RC <= 1'b0;
        RB <= 1'b0;
    end else if ((EW_REQ | OW_REQ) && PIPE_RDY) begin
        RC <= IPIPE_C_FAULT;
        RB <= IPIPE_B_FAULT;
    end
end

// OUTBUFFERS
always_ff @(posedge CLK) begin : outbuffers
    reg OP_STOP;
    if (OP_STOP && IPIPE_FLUSH) begin
        TRAP_CODE <= NONE;
        OP_STOP = 1'b0;
    end else if (IPIPE_FLUSH) begin
        TRAP_CODE <= NONE;
    end else if (OP_STOP) begin
        ; // Do not update after PC is incremented.
    end else if (LOOP_ATN && OPCODE_RD_I) begin
        ; // Wait for pending opcodes.
    end else if (OW_REQ && LOOP_BSY_I) begin
        OP <= OP_I;
        BIW_0 <= IPIPE_D;
        TRAP_CODE <= TRAP_CODE_I;
    end else if (OW_REQ && (PIPE_RDY || BKPT_REQ)) begin
        // Be aware: all BIW are written unaffected
        // if they are all used.
        OP <= OP_I;
        BIW_0 <= IPIPE_D;
        BIW_1 <= IPIPE_C;
        BIW_2 <= IPIPE_B;
        TRAP_CODE <= TRAP_CODE_I;
        //
        if (OP_I == STOP) begin
            OP_STOP = 1'b1;
        end
    end else if (EW_REQ && IPIPE_PNTR != 2'd0) begin
        EXT_WORD <= IPIPE_D;
    end
end

// LOOP_OP
generate
    if (NO_LOOP) begin : GEN_NO_LOOP
        assign LOOP_OP = 1'b0;
    end else begin : GEN_LOOP
        assign LOOP_OP = (OP == MOVE && BIW_0[8:3] == 6'b010010) || // (Ay) to (Ax).
                         (OP == MOVE && BIW_0[8:3] == 6'b011010) || // (Ay) to (Ax)+.
                         (OP == MOVE && BIW_0[8:3] == 6'b100010) || // (Ay) to -(Ax).
                         (OP == MOVE && BIW_0[8:3] == 6'b010011) || // (Ay)+ to (Ax).
                         (OP == MOVE && BIW_0[8:3] == 6'b011011) || // (Ay)+ to (Ax)+.
                         (OP == MOVE && BIW_0[8:3] == 6'b100011) || // (Ay)+ to -(Ax).
                         (OP == MOVE && BIW_0[8:3] == 6'b010100) || // -(Ay) to (Ax).
                         (OP == MOVE && BIW_0[8:3] == 6'b011100) || // -(Ay) to (Ax)+.
                         (OP == MOVE && BIW_0[8:3] == 6'b100100) || // -(Ay) to -(Ax).
                         (OP == MOVE && BIW_0[8:3] == 6'b010000) || // Dy to (Ax).
                         (OP == MOVE && BIW_0[8:3] == 6'b011000) || // Dy to (Ax)+.
                         (OP == MOVE && BIW_0[8:3] == 6'b010001) || // Ay to (Ax).
                         (OP == MOVE && BIW_0[8:3] == 6'b011001) || // Ay to (Ax)+.
                         ((OP == ADD || OP == AND_B || OP == CMP || OP == EOR || OP == OR_B || OP == SUB) && BIW_0[5:3] == 3'b010) || // (Ay) to Dx, Dx to (Ay).
                         ((OP == ADD || OP == AND_B || OP == CMP || OP == EOR || OP == OR_B || OP == SUB) && BIW_0[5:3] == 3'b011) || // (Ay)+ to Dx, Dx to (Ay)+.
                         ((OP == ADD || OP == AND_B || OP == CMP || OP == EOR || OP == OR_B || OP == SUB) && BIW_0[5:3] == 3'b100) || // -(Ay) to Dx, Dx to -(Ay).
                         ((OP == ADDA || OP == CMPA || OP == SUBA) && BIW_0[5:3] == 3'b010) || // (Ay) to Ax.
                         ((OP == ADDA || OP == CMPA || OP == SUBA) && BIW_0[5:3] == 3'b011) || // (Ay)+ to Ax.
                         ((OP == ADDA || OP == CMPA || OP == SUBA) && BIW_0[5:3] == 3'b100) || // -(Ay) to Ax.
                         (OP == ABCD || OP == SBCD || OP == ADDX || OP == SUBX || OP == CMPM) || // -(Ay) to -(Ay), (Ay)+ to (Ay)+ for CMPM.
                         ((OP == CLR || OP == NEG || OP == NEGX || OP == NOT_B || OP == TST || OP == NBCD) && BIW_0[5:3] == 3'b010) || // (Ay).
                         ((OP == CLR || OP == NEG || OP == NEGX || OP == NOT_B || OP == TST || OP == NBCD) && BIW_0[5:3] == 3'b011) || // (Ay)+.
                         ((OP == CLR || OP == NEG || OP == NEGX || OP == NOT_B || OP == TST || OP == NBCD) && BIW_0[5:3] == 3'b100) || // -(Ay).
                         ((OP == ASL || OP == ASR || OP == LSL || OP == LSR) && BIW_0[7:3] == 5'b11010) || // (Ay) by #1.
                         ((OP == ASL || OP == ASR || OP == LSL || OP == LSR) && BIW_0[7:3] == 5'b11011) || // (Ay)+ by #1.
                         ((OP == ASL || OP == ASR || OP == LSL || OP == LSR) && BIW_0[7:3] == 5'b11100) || // -(Ay) by #1.
                         ((OP == ROTL || OP == ROTR || OP == ROXL || OP == ROXR) && BIW_0[7:3] == 5'b11010) || // (Ay) by #1.
                         ((OP == ROTL || OP == ROTR || OP == ROXL || OP == ROXR) && BIW_0[7:3] == 5'b11011) || // (Ay)+ by #1.
                         ((OP == ROTL || OP == ROTR || OP == ROXL || OP == ROXR) && BIW_0[7:3] == 5'b11100); // -(Ay) by #1.
    end
endgenerate

// LOOP_ATN
assign LOOP_ATN = EXH_REQ ? 1'b0 :
                  (!LOOP_BSY_I && LOOP_OP && OP_I == DBcc && IPIPE_C == 16'hFFFC) ? 1'b1 : 1'b0; // IPIPE_C value must be minus four.

// P_LOOP
always_ff @(posedge CLK) begin : loop_ctrl
    if (LOOP_ATN && OW_REQ && !OPCODE_RD_I) begin
        LOOP_BSY <= 1'b1;
        LOOP_BSY_I <= 1'b1;
    end else if (LOOP_EXIT || BUSY_EXH) begin
        LOOP_BSY <= 1'b0;
        LOOP_BSY_I <= 1'b0;
    end
end

assign OW_REQ = BUSY_EXH ? 1'b0 : OW_REQ_MAIN;
assign EW_REQ = EW_REQ_MAIN;

assign PIPE_RDY = (OW_REQ && IPIPE_PNTR == 2'd3 && INSTR_LVL == LVL_B) ||
                  (OW_REQ && IPIPE_PNTR > 2'd1 && INSTR_LVL == LVL_C) ||
                  (OW_REQ && IPIPE_PNTR > 2'd1 && INSTR_LVL == LVL_D) || // We need always pipe C and D to determine the INSTR_LVL.
                  (EW_REQ && IPIPE_PNTR > 2'd0);

// HANDSHAKING
always_ff @(posedge CLK) begin : handshaking
    if (EW_REQ && IPIPE_PNTR != 2'd0)
        EW_ACK <= 1'b1;
    else
        EW_ACK <= 1'b0;

    if (IPIPE_FLUSH) begin
        OPD_ACK_MAIN <= 1'b0;
    end else if (TRAP_CODE_I == T_PRIV) begin // No action when privileged.
        OPD_ACK_MAIN <= 1'b0;
    end else if (OW_REQ && LOOP_BSY_I) begin
        OPD_ACK_MAIN <= 1'b1;
    end else if (OW_REQ && (PIPE_RDY || BKPT_REQ)) begin
        OPD_ACK_MAIN <= 1'b1;
    end else begin
        OPD_ACK_MAIN <= 1'b0;
    end
end

// P_PC_OFFSET
// Be Aware: the ADR_OFFSET requires the 'old' PC_VAR.
// To arrange this, the ADR_OFFSET logic is located
// above the PC_VAR logic. Do not change this!
// The PC_VAR is modeled in a way, that the PC points
// always to the BIW_0.
// The PC_EW_OFFSET is also used for the calculation
// of the correct PC value written to the stack pointer
// during BSR, JSR and exceptions.
always_ff @(posedge CLK) begin : pc_offset
    reg [6:0] ADR_OFFSET;
    reg [6:0] PC_VAR;
    reg [6:0] PC_VAR_MEM;

    if (IPIPE_FLUSH) begin
        ADR_OFFSET = 7'd0;
    end else if (PC_INC_I && OPCODE_RDY_I) begin
        ADR_OFFSET = ADR_OFFSET + 7'd1 - PC_VAR;
    end else if (OPCODE_RDY_I) begin
        ADR_OFFSET = ADR_OFFSET + 7'd1;
    end else if (PC_INC_I) begin
        ADR_OFFSET = ADR_OFFSET - PC_VAR;
    end
    //
    if (!BUSY_EXH) begin
        PC_VAR_MEM = PC_VAR; // Store the old offset to write back on the stack.
    end

    if (BUSY_EXH) begin
        // New PC is loaded by the exception handler.
        // So PC_VAR must be initialized.
        PC_VAR = 7'd0;
    end else if (PC_INC_I || FLUSHED) begin
        case (INSTR_LVL)
            LVL_D: PC_VAR = 7'd1;
            LVL_C: PC_VAR = 7'd2;
            LVL_B: PC_VAR = 7'd3;
        endcase
    end else if (EW_REQ && IPIPE_PNTR != 2'd0) begin
        PC_VAR = PC_VAR + 7'd1;
    end
    //
    if (OW_REQ && BKPT_REQ) begin
        PC_EW_OFFSET <= 4'b0010; // Always level D operations.
    end else if (OW_REQ && PIPE_RDY && OP_I == JSR) begin // Initialize.
        PC_EW_OFFSET <= 4'h0;
    end else if (OW_REQ && PIPE_RDY) begin // BSR.
        case (INSTR_LVL)
            LVL_D: PC_EW_OFFSET <= 4'b0010;
            LVL_C: PC_EW_OFFSET <= 4'b0100;
            default: PC_EW_OFFSET <= 4'b0110; // LONG displacement.
        endcase
    end else if (EW_ACK && OP == JSR) begin // Calculate the required extension words.
        PC_EW_OFFSET <= PC_EW_OFFSET + 4'b0010;
    end

    ADR_OFFSET_S <= ADR_OFFSET;
    PC_VAR_S <= PC_VAR;
    PC_VAR_MEM_S <= PC_VAR_MEM;
end

// P_PC_OFFSET_COMB
always_comb begin : pc_offset_comb
    if (BUSY_EXH && PC_INC_I) begin
        PC_OFFSET = {PC_VAR_MEM_S, 1'b0};
    end else if (OP == DBcc && LOOP_BSY_I && !LOOP_EXIT) begin
        // Suppress to increment after DBcc operation during the loop to
        // handle a correct PC with displacement when looping around.
        PC_OFFSET = 8'h00;
    end else begin
        PC_OFFSET = {PC_VAR_S, 1'b0};
    end
    PC_ADR_OFFSET = {ADR_OFFSET_S, 1'b0};
end

// P_FLUSH
always_ff @(posedge CLK) begin : flush_tracker
    if (IPIPE_FLUSH)
        FLUSHED <= 1'b1;
    else if (OW_REQ && PIPE_RDY)
        FLUSHED <= 1'b0;
end

assign PC_INC = PC_INC_I;
assign PC_INC_I = FLUSHED ? 1'b0 : // Avoid double increment after a flushed pipe.
                  (IPIPE_FLUSH && BUSY_MAIN) ? 1'b1 : // If the pipe is flushed, we need the new PC value for refilling.
                  BKPT_REQ ? 1'b0 : // Do not update!
                  (OW_REQ && PIPE_RDY) ? 1'b1 : PC_INC_EXH;

// INSTR_LVL
assign INSTR_LVL = (TRAP_CODE_I == T_PRIV) ? LVL_D : // Points to the first word. Required for stacking.
                   (OP_I == ADDI && IPIPE_D[7:6] == 2'b10) ? LVL_B :
                   (OP_I == ANDI && IPIPE_D[7:6] == 2'b10) ? LVL_B :
                   ((OP_I == Bcc || OP_I == BRA || OP_I == BSR) && IPIPE_D[7:0] == 8'hFF) ? LVL_B :
                   (OP_I == CAS2 || (OP_I == CMPI && IPIPE_D[7:6] == 2'b10)) ? LVL_B :
                   (OP_I == EORI && IPIPE_D[7:6] == 2'b10) ? LVL_B :
                   (OP_I == LINK && IPIPE_D[11:3] == 9'b100000001) ? LVL_B : // LONG.
                   (OP_I == ORI && IPIPE_D[7:6] == 2'b10) ? LVL_B :
                   (OP_I == SUBI && IPIPE_D[7:6] == 2'b10) ? LVL_B :
                   (OP_I == TRAPcc && IPIPE_D[2:0] == 3'b011) ? LVL_B :
                   (OP_I == ADDI || OP_I == ANDI || OP_I == ANDI_TO_SR || OP_I == ANDI_TO_CCR) ? LVL_C :
                   ((OP_I == BCHG || OP_I == BCLR || OP_I == BSET || OP_I == BTST) && !IPIPE_D[8]) ? LVL_C :
                   ((OP_I == Bcc || OP_I == BRA || OP_I == BSR) && IPIPE_D[7:0] == 8'h00) ? LVL_C :
                   (OP_I == BFCHG || OP_I == BFCLR || OP_I == BFEXTS || OP_I == BFEXTU) ? LVL_C :
                   (OP_I == BFFFO || OP_I == BFINS || OP_I == BFSET || OP_I == BFTST) ? LVL_C :
                   (OP_I == CAS || OP_I == CHK2 || OP_I == CMP2 || OP_I == CMPI || OP_I == DBcc) ? LVL_C :
                   ((OP_I == DIVS || OP_I == DIVU) && IPIPE_D[8:6] == 3'b001) ? LVL_C :
                   (OP_I == EORI || OP_I == EORI_TO_CCR || OP_I == EORI_TO_SR) ? LVL_C :
                   (OP_I == LINK || OP_I == MOVEC) ? LVL_C :
                   (OP_I == MOVEM || OP_I == MOVEP || OP_I == MOVES) ? LVL_C :
                   ((OP_I == MULS || OP_I == MULU) && IPIPE_D[8:6] == 3'b000) ? LVL_C :
                   (OP_I == ORI_TO_CCR || OP_I == ORI_TO_SR || OP_I == ORI) ? LVL_C :
                   (OP_I == PACK || OP_I == RTD || OP_I == SUBI || OP_I == STOP) ? LVL_C :
                   (OP_I == TRAPcc && IPIPE_D[2:0] == 3'b010) ? LVL_C :
                   (OP_I == UNPK) ? LVL_C : LVL_D;

// TRAP_CODE_I
assign TRAP_CODE_I = (OP_I == UNIMPLEMENTED && IPIPE_D[15:12] == 4'hA) ? T_1010 :
                     (OP_I == UNIMPLEMENTED && IPIPE_D[15:12] == 4'hF) ? T_1111 :
                     (OP_I == ILLEGAL) ? T_ILLEGAL :
                     (OP_I == RTE && SBIT) ? T_RTE : // Handled like a trap simplifies the code.
                     (OP_I == TRAP) ? T_TRAP :
                     (OP_I == ANDI_TO_SR && !SBIT) ? T_PRIV :
                     (OP_I == EORI_TO_SR && !SBIT) ? T_PRIV :
                     ((OP_I == MOVE_FROM_SR || OP_I == MOVE_TO_SR) && !SBIT) ? T_PRIV :
                     ((OP_I == MOVE_USP || OP_I == MOVEC || OP_I == MOVES) && !SBIT) ? T_PRIV :
                     (OP_I == ORI_TO_SR && !SBIT) ? T_PRIV :
                     ((OP_I == OP_RESET || OP_I == RTE) && !SBIT) ? T_PRIV :
                     (OP_I == STOP && !SBIT) ? T_PRIV : NONE;


// ============================================================================
// Addressing mode validation functions
//
// These encapsulate the repeated EA-validity patterns used throughout the
// 68030 opcode map. Each takes the 3-bit mode and register fields from
// the instruction word and returns whether the combination is legal for
// the given addressing category.
// ============================================================================

// Data EA: all modes except An direct (001). Mode 111 limited by reg_fld.
function automatic logic is_data_ea(input logic [2:0] mode, input logic [2:0] reg_fld, input logic [2:0] max_reg);
    if (mode == 3'b111)
        is_data_ea = reg_fld < max_reg;
    else
        is_data_ea = mode != 3'b001;
endfunction

// All EA including An direct. Mode 111 limited by reg_fld.
function automatic logic is_all_ea(input logic [2:0] mode, input logic [2:0] reg_fld, input logic [2:0] max_reg);
    if (mode == 3'b111)
        is_all_ea = reg_fld < max_reg;
    else
        is_all_ea = 1'b1;
endfunction

// Data alterable: no An (001), no PC-relative, no immediate.
// Modes 000, 010-110 valid. Mode 111 with reg < 010.
function automatic logic is_data_alterable(input logic [2:0] mode, input logic [2:0] reg_fld);
    if (mode == 3'b111)
        is_data_alterable = reg_fld < 3'b010;
    else
        is_data_alterable = mode != 3'b001;
endfunction

// Memory alterable: modes 010-110 valid. Mode 111 with reg < 010.
function automatic logic is_mem_alterable(input logic [2:0] mode, input logic [2:0] reg_fld);
    if (mode == 3'b111)
        is_mem_alterable = reg_fld < 3'b010;
    else
        is_mem_alterable = mode > 3'b001;
endfunction

// Control EA: modes 010, 101, 110 valid. Mode 111 with reg < max_reg.
function automatic logic is_control_ea(input logic [2:0] mode, input logic [2:0] reg_fld, input logic [2:0] max_reg);
    if (mode == 3'b111)
        is_control_ea = reg_fld < max_reg;
    else
        is_control_ea = mode == 3'b010 || mode == 3'b101 || mode == 3'b110;
endfunction

// Bitfield EA: Dn (000), (An) (010), (d16,An) (101), (d8,An,Xn) (110).
// Mode 111 with reg < max_reg.
function automatic logic is_bf_ea(input logic [2:0] mode, input logic [2:0] reg_fld, input logic [2:0] max_reg);
    if (mode == 3'b111)
        is_bf_ea = reg_fld < max_reg;
    else
        is_bf_ea = mode == 3'b000 || mode == 3'b010 || mode >= 3'b101;
endfunction

// ============================================================================
// Per-group decode functions
//
// Each function handles one (or a pair of) top-level opcode groups,
// corresponding to bits [15:12] of the instruction word. They accept
// the instruction word (and the second pipe word where needed) and
// produce the decoded OP_68K enumeration value.
//
// Yosys does not support the SystemVerilog 'return' statement, so all
// functions use assignment to the function name with nested if/else.
// ============================================================================

// decode_0xxx: Bit manipulation / MOVEP / Immediate ops
function automatic OP_68K decode_0xxx(input logic [15:0] iw, input logic [15:0] iw_c);
    logic [2:0] mode, reg_fld;
    logic matched;
    mode    = iw[5:3];
    reg_fld = iw[2:0];
    decode_0xxx = ILLEGAL;
    matched = 1'b0;

    // --- Exact-match special opcodes ---
    if (!matched) begin
        case (iw[11:0])
            12'h03C: begin decode_0xxx = ORI_TO_CCR;  matched = 1'b1; end
            12'h07C: begin decode_0xxx = ORI_TO_SR;   matched = 1'b1; end
            12'h23C: begin decode_0xxx = ANDI_TO_CCR;  matched = 1'b1; end
            12'h27C: begin decode_0xxx = ANDI_TO_SR;   matched = 1'b1; end
            12'hA3C: begin decode_0xxx = EORI_TO_CCR;  matched = 1'b1; end
            12'hA7C: begin decode_0xxx = EORI_TO_SR;   matched = 1'b1; end
            12'hCFC: begin decode_0xxx = CAS2;         matched = 1'b1; end
            12'hEFC: begin decode_0xxx = CAS2;         matched = 1'b1; end
            default: ;
        endcase
    end

    // --- MOVES ---
    if (!matched && iw[11:8] == 4'hE && iw[7:6] < 2'b11 && is_mem_alterable(mode, reg_fld)) begin
        decode_0xxx = MOVES;  matched = 1'b1;
    end

    // --- MOVEP ---
    if (!matched && iw[8:6] > 3'b011 && mode == 3'b001) begin
        decode_0xxx = MOVEP;  matched = 1'b1;
    end

    // --- CHK2 / CMP2 (require second word) ---
    if (!matched && !iw[11] && iw[10:9] != 2'b11 && iw[8:6] == 3'b011
            && is_control_ea(mode, reg_fld, 3'b100)) begin
        if (iw_c[11])
            decode_0xxx = CHK2;
        else
            decode_0xxx = CMP2;
        matched = 1'b1;
    end

    // --- CAS ---
    if (!matched && iw[11] && iw[10:9] != 2'b00 && iw[8:6] == 3'b011
            && is_mem_alterable(mode, reg_fld)) begin
        decode_0xxx = CAS;  matched = 1'b1;
    end

    // --- Static bit operations (immediate bit number in second word) ---
    if (!matched) begin
        case (iw[11:6])
            6'b100000: if (is_data_ea(mode, reg_fld, 3'b100))   begin decode_0xxx = BTST; matched = 1'b1; end
            6'b100001: if (is_data_alterable(mode, reg_fld))     begin decode_0xxx = BCHG; matched = 1'b1; end
            6'b100010: if (is_data_alterable(mode, reg_fld))     begin decode_0xxx = BCLR; matched = 1'b1; end
            6'b100011: if (is_data_alterable(mode, reg_fld))     begin decode_0xxx = BSET; matched = 1'b1; end
            default: ;
        endcase
    end

    // --- Logic immediate operations (size in [7:6]) ---
    if (!matched && iw[7:6] < 2'b11 && is_data_alterable(mode, reg_fld)) begin
        case (iw[11:8])
            4'h0: begin decode_0xxx = ORI;  matched = 1'b1; end
            4'h2: begin decode_0xxx = ANDI; matched = 1'b1; end
            4'h4: begin decode_0xxx = SUBI; matched = 1'b1; end
            4'h6: begin decode_0xxx = ADDI; matched = 1'b1; end
            4'hA: begin decode_0xxx = EORI; matched = 1'b1; end
            default: ;
        endcase
    end

    // --- CMPI: data EA includes PC-relative (reg < 100) ---
    if (!matched && iw[11:8] == 4'hC && iw[7:6] < 2'b11 && is_data_ea(mode, reg_fld, 3'b100)) begin
        decode_0xxx = CMPI;  matched = 1'b1;
    end

    // --- Dynamic bit operations (register-specified bit number) ---
    if (!matched && mode != 3'b001) begin
        case (iw[8:6])
            3'b100: if (is_data_ea(mode, reg_fld, 3'b101))    begin decode_0xxx = BTST; matched = 1'b1; end
            3'b101: if (is_data_alterable(mode, reg_fld))      begin decode_0xxx = BCHG; matched = 1'b1; end
            3'b110: if (is_data_alterable(mode, reg_fld))      begin decode_0xxx = BCLR; matched = 1'b1; end
            3'b111: if (is_data_alterable(mode, reg_fld))      begin decode_0xxx = BSET; matched = 1'b1; end
            default: ;
        endcase
    end
endfunction

// decode_1xxx: MOVE.B
function automatic OP_68K decode_1xxx(input logic [15:0] iw);
    logic [2:0] src_mode, src_reg, dst_mode, dst_reg;
    logic src_ok, dst_ok;
    src_mode = iw[5:3];
    src_reg  = iw[2:0];
    dst_mode = iw[8:6];
    dst_reg  = iw[11:9];

    // Source: all modes except An direct (byte). Mode 111: reg < 101.
    src_ok = is_data_ea(src_mode, src_reg, 3'b101);

    // Destination: data alterable (no An for byte).
    if (dst_mode == 3'b111)
        dst_ok = dst_reg < 3'b010;
    else
        dst_ok = dst_mode != 3'b001;

    if (src_ok && dst_ok)
        decode_1xxx = MOVE;
    else
        decode_1xxx = ILLEGAL;
endfunction

// decode_23xxx: MOVE.W / MOVE.L / MOVEA
function automatic OP_68K decode_23xxx(input logic [15:0] iw);
    logic [2:0] src_mode, src_reg, dst_mode, dst_reg;
    logic src_ok;
    src_mode = iw[5:3];
    src_reg  = iw[2:0];
    dst_mode = iw[8:6];
    dst_reg  = iw[11:9];

    // Source: all modes. Mode 111: reg < 101.
    src_ok = is_all_ea(src_mode, src_reg, 3'b101);

    if (!src_ok)
        decode_23xxx = ILLEGAL;
    else if (dst_mode == 3'b001)
        decode_23xxx = MOVEA;
    else if (dst_mode == 3'b111 && dst_reg >= 3'b010)
        decode_23xxx = ILLEGAL;
    else
        decode_23xxx = MOVE;
endfunction

// decode_4xxx: Miscellaneous (LEA, CHK, MOVEM, CLR, NEG, etc.)
function automatic OP_68K decode_4xxx(input logic [15:0] iw, input logic [15:0] iw_c);
    logic [2:0] mode, reg_fld;
    logic matched;
    mode    = iw[5:3];
    reg_fld = iw[2:0];
    decode_4xxx = ILLEGAL;
    matched = 1'b0;

    // --- Exact-match system control opcodes ---
    if (!matched) begin
        case (iw[11:0])
            12'hE70: begin decode_4xxx = OP_RESET; matched = 1'b1; end
            12'hE71: begin decode_4xxx = NOP;      matched = 1'b1; end
            12'hE72: begin decode_4xxx = STOP;     matched = 1'b1; end
            12'hE73: begin decode_4xxx = RTE;      matched = 1'b1; end
            12'hE74: begin decode_4xxx = RTD;      matched = 1'b1; end
            12'hE75: begin decode_4xxx = RTS;      matched = 1'b1; end
            12'hE76: begin decode_4xxx = TRAPV;    matched = 1'b1; end
            12'hE77: begin decode_4xxx = RTR;      matched = 1'b1; end
            12'hAFC: begin decode_4xxx = ILLEGAL;  matched = 1'b1; end
            default: ;
        endcase
    end

    // --- MOVEC ---
    if (!matched && iw[11:1] == 11'b11100111101) begin
        case (iw_c[11:0])
            12'h000, 12'h001, 12'h800, 12'h801: decode_4xxx = MOVEC;
            default: decode_4xxx = ILLEGAL;
        endcase
        matched = 1'b1;
    end

    // --- Fixed register-group opcodes ---
    if (!matched) begin
        case (iw[11:3])
            9'b100001001: begin decode_4xxx = BKPT; matched = 1'b1; end
            9'b100000001: begin decode_4xxx = LINK; matched = 1'b1; end  // LONG
            9'b111001010: begin decode_4xxx = LINK; matched = 1'b1; end  // WORD
            9'b111001011: begin decode_4xxx = UNLK; matched = 1'b1; end
            9'b100001000: begin decode_4xxx = SWAP; matched = 1'b1; end
            default: ;
        endcase
    end

    if (!matched && iw[11:4] == 8'hE4) begin decode_4xxx = TRAP;     matched = 1'b1; end
    if (!matched && iw[11:4] == 8'hE6) begin decode_4xxx = MOVE_USP; matched = 1'b1; end

    // --- DIV long / MUL long / MOVE SR,CCR / NBCD / TAS ---
    if (!matched) begin
        case (iw[11:6])
            6'b110001: if (is_data_ea(mode, reg_fld, 3'b101)) begin
                decode_4xxx = iw_c[11] ? DIVS : DIVU;  matched = 1'b1;
            end
            6'b110000: if (is_data_ea(mode, reg_fld, 3'b101)) begin
                decode_4xxx = iw_c[11] ? MULS : MULU;  matched = 1'b1;
            end
            6'b001011: if (is_data_alterable(mode, reg_fld)) begin
                decode_4xxx = MOVE_FROM_CCR;  matched = 1'b1;
            end
            6'b000011: if (is_data_alterable(mode, reg_fld)) begin
                decode_4xxx = MOVE_FROM_SR;   matched = 1'b1;
            end
            6'b010011: if (is_data_ea(mode, reg_fld, 3'b101)) begin
                decode_4xxx = MOVE_TO_CCR;    matched = 1'b1;
            end
            6'b011011: if (is_data_ea(mode, reg_fld, 3'b101)) begin
                decode_4xxx = MOVE_TO_SR;     matched = 1'b1;
            end
            6'b100000: if (is_data_alterable(mode, reg_fld)) begin
                decode_4xxx = NBCD;           matched = 1'b1;
            end
            6'b101011: if (is_data_alterable(mode, reg_fld)) begin
                decode_4xxx = TAS;            matched = 1'b1;
            end
            default: ;
        endcase
    end

    // --- PEA / JSR / JMP (control addressing) ---
    if (!matched) begin
        case (iw[11:6])
            6'b100001: if (is_control_ea(mode, reg_fld, 3'b100)) begin
                decode_4xxx = PEA;  matched = 1'b1;
            end
            6'b111010: if (is_control_ea(mode, reg_fld, 3'b100)) begin
                decode_4xxx = JSR;  matched = 1'b1;
            end
            6'b111011: if (is_control_ea(mode, reg_fld, 3'b100)) begin
                decode_4xxx = JMP;  matched = 1'b1;
            end
            default: ;
        endcase
    end

    // --- NEGX / CLR / NEG / NOT (data alterable, size valid) ---
    if (!matched && iw[7:6] < 2'b11 && is_data_alterable(mode, reg_fld)) begin
        case (iw[11:8])
            4'h0: begin decode_4xxx = NEGX;  matched = 1'b1; end
            4'h2: begin decode_4xxx = CLR;   matched = 1'b1; end
            4'h4: begin decode_4xxx = NEG;   matched = 1'b1; end
            4'h6: begin decode_4xxx = NOT_B; matched = 1'b1; end
            default: ;
        endcase
    end

    // --- TST (word/long: all EAs; byte: data EAs) ---
    if (!matched && iw[11:8] == 4'hA && iw[7:6] < 2'b11) begin
        if (iw[7:6] == 2'b01 || iw[7:6] == 2'b10) begin
            if (is_all_ea(mode, reg_fld, 3'b101)) begin
                decode_4xxx = TST;  matched = 1'b1;
            end
        end else begin
            if (is_data_ea(mode, reg_fld, 3'b101)) begin
                decode_4xxx = TST;  matched = 1'b1;
            end
        end
    end

    // --- EXT / EXTB (data register only) ---
    if (!matched && iw[11:9] == 3'b100 && mode == 3'b000) begin
        case (iw[8:6])
            3'b010, 3'b011: begin decode_4xxx = EXT;  matched = 1'b1; end
            3'b111:         begin decode_4xxx = EXTB; matched = 1'b1; end
            default: ;
        endcase
    end

    // --- LEA (control addressing) ---
    if (!matched && iw[8:6] == 3'b111 && is_control_ea(mode, reg_fld, 3'b100)) begin
        decode_4xxx = LEA;  matched = 1'b1;
    end

    // --- MOVEM ---
    if (!matched && iw[11] && iw[9:7] == 3'b001) begin
        if (!iw[10]) begin
            // Register to memory: no postincrement (010, 100, 101, 110; 111 with reg < 010)
            case (mode)
                3'b010, 3'b100, 3'b101, 3'b110: begin decode_4xxx = MOVEM; matched = 1'b1; end
                3'b111: if (reg_fld < 3'b010) begin decode_4xxx = MOVEM; matched = 1'b1; end
                default: ;
            endcase
        end else begin
            // Memory to register: no predecrement (010, 011, 101, 110; 111 with reg < 100)
            case (mode)
                3'b010, 3'b011, 3'b101, 3'b110: begin decode_4xxx = MOVEM; matched = 1'b1; end
                3'b111: if (reg_fld < 3'b100) begin decode_4xxx = MOVEM; matched = 1'b1; end
                default: ;
            endcase
        end
    end

    // --- CHK (size must be 10 or 11, opmode not 001) ---
    if (!matched && iw[8:7] >= 2'b10) begin
        if (mode == 3'b111) begin
            if (reg_fld < 3'b101) begin
                decode_4xxx = CHK;  matched = 1'b1;
            end
        end else if (iw[6:3] != 4'h1) begin
            decode_4xxx = CHK;  matched = 1'b1;
        end
    end
endfunction

// decode_5xxx: ADDQ / SUBQ / Scc / DBcc / TRAPcc
function automatic OP_68K decode_5xxx(input logic [15:0] iw);
    logic [2:0] mode, reg_fld;
    logic ea_ok;
    mode    = iw[5:3];
    reg_fld = iw[2:0];
    decode_5xxx = ILLEGAL;

    if (iw[7:3] == 5'b11001) begin
        // DBcc: size=11, mode=001
        decode_5xxx = DBcc;
    end else if (iw[7:6] == 2'b11 && is_data_alterable(mode, reg_fld)) begin
        // Scc: size=11, data alterable
        decode_5xxx = Scc;
    end else if (iw[7:6] < 2'b11) begin
        // ADDQ / SUBQ
        if (iw[7:6] == 2'b00)
            ea_ok = is_data_alterable(mode, reg_fld); // Byte: no An direct
        else if (mode == 3'b111)
            ea_ok = reg_fld < 3'b010; // Word/Long: alterable including An
        else
            ea_ok = 1'b1;

        if (ea_ok)
            decode_5xxx = iw[8] ? SUBQ : ADDQ;
    end else if (iw[7:3] == 5'b11111) begin
        // TRAPcc
        decode_5xxx = TRAPcc;
    end
endfunction

// decode_6xxx: Bcc / BSR / BRA
function automatic OP_68K decode_6xxx(input logic [15:0] iw);
    if (iw[11:8] == 4'h0)
        decode_6xxx = BRA;
    else if (iw[11:8] == 4'h1)
        decode_6xxx = BSR;
    else
        decode_6xxx = Bcc;
endfunction

// decode_7xxx: MOVEQ
function automatic OP_68K decode_7xxx(input logic [15:0] iw);
    if (!iw[8])
        decode_7xxx = MOVEQ;
    else
        decode_7xxx = ILLEGAL;
endfunction

// decode_8xxx: OR / DIV / SBCD / PACK / UNPK
function automatic OP_68K decode_8xxx(input logic [15:0] iw);
    logic [2:0] mode, reg_fld;
    mode    = iw[5:3];
    reg_fld = iw[2:0];
    decode_8xxx = ILLEGAL;

    if (iw[8:4] == 5'b10100)
        decode_8xxx = PACK;
    else if (iw[8:4] == 5'b11000)
        decode_8xxx = UNPK;
    else if (iw[8:6] == 3'b011 && is_data_ea(mode, reg_fld, 3'b101))
        decode_8xxx = DIVU;
    else if (iw[8:6] == 3'b111 && is_data_ea(mode, reg_fld, 3'b101))
        decode_8xxx = DIVS;
    else if (iw[8:4] == 5'b10000)
        decode_8xxx = SBCD;
    else begin
        // OR: source EA to register (000-010) or register to memory EA (100-110)
        case (iw[8:6])
            3'b000, 3'b001, 3'b010:
                if (is_data_ea(mode, reg_fld, 3'b101)) decode_8xxx = OR_B;
            3'b100, 3'b101, 3'b110:
                if (is_mem_alterable(mode, reg_fld)) decode_8xxx = OR_B;
            default: ;
        endcase
    end
endfunction

// Shared helper for ADD/SUB groups (identical structure)
function automatic OP_68K decode_addsub_group(
    input logic [15:0] iw,
    input OP_68K       op_main,
    input OP_68K       op_extended,
    input OP_68K       op_address
);
    logic [2:0] mode, reg_fld;
    mode    = iw[5:3];
    reg_fld = iw[2:0];
    decode_addsub_group = ILLEGAL;

    case (iw[8:6])
        3'b000: begin // Byte: source EA to register, no An direct
            if (is_data_ea(mode, reg_fld, 3'b101))
                decode_addsub_group = op_main;
        end
        3'b001, 3'b010: begin // Word/Long: source EA to register, all modes
            if (is_all_ea(mode, reg_fld, 3'b101))
                decode_addsub_group = op_main;
        end
        3'b100: begin // Byte: register to EA or extended
            if (mode == 3'b000 || mode == 3'b001)
                decode_addsub_group = op_extended;
            else if (is_data_alterable(mode, reg_fld))
                decode_addsub_group = op_main;
        end
        3'b101, 3'b110: begin // Word/Long: register to EA or extended
            if (mode == 3'b000 || mode == 3'b001)
                decode_addsub_group = op_extended;
            else if (is_mem_alterable(mode, reg_fld))
                decode_addsub_group = op_main;
        end
        3'b011, 3'b111: begin // ADDA/SUBA: all source EAs
            if (is_all_ea(mode, reg_fld, 3'b101))
                decode_addsub_group = op_address;
        end
        default: ;
    endcase
endfunction

// decode_9xxx: SUB / SUBA / SUBX
function automatic OP_68K decode_9xxx(input logic [15:0] iw);
    decode_9xxx = decode_addsub_group(iw, SUB, SUBX, SUBA);
endfunction

// decode_Bxxx: CMP / CMPA / CMPM / EOR
function automatic OP_68K decode_Bxxx(input logic [15:0] iw);
    logic [2:0] mode, reg_fld;
    mode    = iw[5:3];
    reg_fld = iw[2:0];
    decode_Bxxx = ILLEGAL;

    if (iw[8] && iw[7:6] < 2'b11 && mode == 3'b001) begin
        // CMPM: postincrement mode (001) with size != 11
        decode_Bxxx = CMPM;
    end else begin
        case (iw[8:6])
            3'b000: // CMP byte: data EA (no An direct)
                if (is_data_ea(mode, reg_fld, 3'b101)) decode_Bxxx = CMP;
            3'b001, 3'b010: // CMP word/long: all EA
                if (is_all_ea(mode, reg_fld, 3'b101)) decode_Bxxx = CMP;
            3'b011, 3'b111: // CMPA: all EA
                if (is_all_ea(mode, reg_fld, 3'b101)) decode_Bxxx = CMPA;
            3'b100, 3'b101, 3'b110: // EOR: data alterable
                if (is_data_alterable(mode, reg_fld)) decode_Bxxx = EOR;
            default: ;
        endcase
    end
endfunction

// decode_Cxxx: AND / MUL / ABCD / EXG
function automatic OP_68K decode_Cxxx(input logic [15:0] iw);
    logic [2:0] mode, reg_fld;
    mode    = iw[5:3];
    reg_fld = iw[2:0];
    decode_Cxxx = ILLEGAL;

    if (iw[8:4] == 5'b10000)
        decode_Cxxx = ABCD;
    else if (iw[8:6] == 3'b011 && is_data_ea(mode, reg_fld, 3'b101))
        decode_Cxxx = MULU;
    else if (iw[8:6] == 3'b111 && is_data_ea(mode, reg_fld, 3'b101))
        decode_Cxxx = MULS;
    else if (iw[8:3] == 6'b101000 || iw[8:3] == 6'b101001 || iw[8:3] == 6'b110001)
        decode_Cxxx = EXG;
    else begin
        case (iw[8:6])
            3'b000, 3'b001, 3'b010:
                if (is_data_ea(mode, reg_fld, 3'b101)) decode_Cxxx = AND_B;
            3'b100, 3'b101, 3'b110:
                if (is_mem_alterable(mode, reg_fld)) decode_Cxxx = AND_B;
            default: ;
        endcase
    end
endfunction

// decode_Dxxx: ADD / ADDA / ADDX
function automatic OP_68K decode_Dxxx(input logic [15:0] iw);
    decode_Dxxx = decode_addsub_group(iw, ADD, ADDX, ADDA);
endfunction

// decode_Exxx: Shift / Rotate / Bit Field
function automatic OP_68K decode_Exxx(input logic [15:0] iw);
    logic [2:0] mode, reg_fld;
    logic matched;
    mode    = iw[5:3];
    reg_fld = iw[2:0];
    decode_Exxx = ILLEGAL;
    matched = 1'b0;

    // --- Bitfield operations (size field = 11, bit 11 set) ---
    if (!matched && iw[7:6] == 2'b11 && iw[11]) begin
        case (iw[11:6])
            6'b101011: if (is_bf_ea(mode, reg_fld, 3'b010)) begin decode_Exxx = BFCHG;  matched = 1'b1; end
            6'b110011: if (is_bf_ea(mode, reg_fld, 3'b010)) begin decode_Exxx = BFCLR;  matched = 1'b1; end
            6'b111011: if (is_bf_ea(mode, reg_fld, 3'b010)) begin decode_Exxx = BFSET;  matched = 1'b1; end
            6'b111111: if (is_bf_ea(mode, reg_fld, 3'b010)) begin decode_Exxx = BFINS;  matched = 1'b1; end
            6'b101111: if (is_bf_ea(mode, reg_fld, 3'b100)) begin decode_Exxx = BFEXTS; matched = 1'b1; end
            6'b100111: if (is_bf_ea(mode, reg_fld, 3'b100)) begin decode_Exxx = BFEXTU; matched = 1'b1; end
            6'b110111: if (is_bf_ea(mode, reg_fld, 3'b100)) begin decode_Exxx = BFFFO;  matched = 1'b1; end
            6'b100011: if (is_bf_ea(mode, reg_fld, 3'b100)) begin decode_Exxx = BFTST;  matched = 1'b1; end
            default: ;
        endcase
    end

    // --- Memory shifts/rotates (size field = 11, bit 11 clear) ---
    if (!matched && iw[7:6] == 2'b11 && !iw[11] && is_mem_alterable(mode, reg_fld)) begin
        case (iw[11:6])
            6'b000011: begin decode_Exxx = ASR;  matched = 1'b1; end
            6'b000111: begin decode_Exxx = ASL;  matched = 1'b1; end
            6'b001011: begin decode_Exxx = LSR;  matched = 1'b1; end
            6'b001111: begin decode_Exxx = LSL;  matched = 1'b1; end
            6'b010011: begin decode_Exxx = ROXR; matched = 1'b1; end
            6'b010111: begin decode_Exxx = ROXL; matched = 1'b1; end
            6'b011011: begin decode_Exxx = ROTR; matched = 1'b1; end
            6'b011111: begin decode_Exxx = ROTL; matched = 1'b1; end
            default: ;
        endcase
    end

    // --- Register shifts/rotates (size != 11) ---
    if (!matched && iw[7:6] < 2'b11) begin
        case ({iw[8], iw[4:3]})
            3'b000: decode_Exxx = ASR;
            3'b100: decode_Exxx = ASL;
            3'b001: decode_Exxx = LSR;
            3'b101: decode_Exxx = LSL;
            3'b010: decode_Exxx = ROXR;
            3'b110: decode_Exxx = ROXL;
            3'b011: decode_Exxx = ROTR;
            3'b111: decode_Exxx = ROTL;
        endcase
    end
endfunction

// ============================================================================
// Top-level decode dispatch
// ============================================================================

always_comb begin : op_decode
    case (IPIPE_D[15:12])
        4'h0:    OP_I = decode_0xxx(IPIPE_D, IPIPE_C);
        4'h1:    OP_I = decode_1xxx(IPIPE_D);
        4'h2:    OP_I = decode_23xxx(IPIPE_D);
        4'h3:    OP_I = decode_23xxx(IPIPE_D);
        4'h4:    OP_I = decode_4xxx(IPIPE_D, IPIPE_C);
        4'h5:    OP_I = decode_5xxx(IPIPE_D);
        4'h6:    OP_I = decode_6xxx(IPIPE_D);
        4'h7:    OP_I = decode_7xxx(IPIPE_D);
        4'h8:    OP_I = decode_8xxx(IPIPE_D);
        4'h9:    OP_I = decode_9xxx(IPIPE_D);
        4'hA:    OP_I = UNIMPLEMENTED;
        4'hB:    OP_I = decode_Bxxx(IPIPE_D);
        4'hC:    OP_I = decode_Cxxx(IPIPE_D);
        4'hD:    OP_I = decode_Dxxx(IPIPE_D);
        4'hE:    OP_I = decode_Exxx(IPIPE_D);
        4'hF:    OP_I = UNIMPLEMENTED;
        default: OP_I = ILLEGAL;
    endcase
end

endmodule
