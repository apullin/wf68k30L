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
always @(posedge CLK or posedge OPCODE_RDY or posedge BUSY_EXH or negedge IPIPE_FILL) begin
    if (OPCODE_RDY == 1'b1) begin
        OPCODE_RD_I <= 1'b0;
    end else if (BUSY_EXH == 1'b1 && IPIPE_FILL == 1'b0) begin
        OPCODE_RD_I <= 1'b0;
    end else begin
        // posedge CLK path
        if (IPIPE_FLUSH == 1'b1) begin
            OPCODE_RD_I <= 1'b1;
        end else if ((LOOP_ATN == 1'b1 && OPCODE_RD_I == 1'b0) || LOOP_BSY_I == 1'b1) begin
            OPCODE_RD_I <= 1'b0;
        end else if (IPIPE_PNTR < 2'd3) begin
            OPCODE_RD_I <= 1'b1;
        end
    end
end

// P_OPCODE_FLUSH
always_ff @(posedge CLK) begin : P_OPCODE_FLUSH
    if (IPIPE_FLUSH == 1'b1 && OPCODE_RD_I == 1'b1 && OPCODE_RDY == 1'b0) begin
        OPCODE_FLUSH <= 1'b1;
    end else if (OPCODE_RDY == 1'b1 || BUSY_EXH == 1'b1) begin
        OPCODE_FLUSH <= 1'b0;
    end
end

assign OPCODE_RD = OPCODE_RD_I;
assign OPCODE_RDY_I = (OPCODE_FLUSH == 1'b1) ? 1'b0 : OPCODE_RDY; // Dismiss the current read cycle.
assign BUSY_OPD = (EXH_REQ == 1'b1 && BUSY_MAIN == 1'b0 && IPIPE_PNTR > 2'd0 && OPCODE_RD_I == 1'b0) ? 1'b0 : // Fill one opcode is sufficient here.
                  (IPIPE_PNTR < 2'd3 || OPCODE_RD_I == 1'b1) ? 1'b1 : 1'b0;

// INSTRUCTION_PIPE
always_ff @(posedge CLK) begin : INSTRUCTION_PIPE
    reg [15:0] IPIPE_D_VAR;
    if (IPIPE_FLUSH == 1'b1) begin
        IPIPE_D <= 16'h0000;
        IPIPE_C <= 16'h0000;
        IPIPE_B <= 16'h0000;
        IPIPE_PNTR <= 2'd0;
    end else if (BKPT_INSERT == 1'b1) begin
        IPIPE_D_VAR = IPIPE_D;
        IPIPE_D <= BKPT_DATA; // Insert the breakpoint data.
        BKPT_REQ <= 1'b1;
    end else if (OW_REQ == 1'b1 && BKPT_REQ == 1'b1) begin
        IPIPE_D <= IPIPE_D_VAR; // Restore from breakpoint.
        BKPT_REQ <= 1'b0;
    end else if (LOOP_ATN == 1'b1 && OPCODE_RD_I == 1'b1) begin
        ; // Wait for pending opcodes.
    end else if (OW_REQ == 1'b1 && PIPE_RDY == 1'b1 && OP_I == DBcc && LOOP_OP == 1'b1 && IPIPE_C == 16'hFFFC) begin // Initialize the loop.
        IPIPE_D <= BIW_0; // This is the LEVEL D operation for the loop.
    end else if (OW_REQ == 1'b1 && LOOP_BSY_I == 1'b1) begin
        IPIPE_D <= BIW_0; // Recycle the loop operations.
    end else if (LOOP_BSY_I == 1'b1) begin
        ; // Do not change the pipe during the loop.
    end else if (OW_REQ == 1'b1 && INSTR_LVL == LVL_D && PIPE_RDY == 1'b1 && IPIPE_PNTR == 2'd2) begin
        if (OPCODE_RDY_I == 1'b1) begin
            IPIPE_D <= IPIPE_C;
            IPIPE_C <= OPCODE_DATA;
            IPIPE_D_FAULT <= IPIPE_C_FAULT;
            IPIPE_C_FAULT <= ~OPCODE_VALID;
        end else begin
            IPIPE_D <= IPIPE_C;
            IPIPE_D_FAULT <= IPIPE_C_FAULT;
            IPIPE_PNTR <= IPIPE_PNTR - 2'd1;
        end
    end else if (OW_REQ == 1'b1 && INSTR_LVL == LVL_D && PIPE_RDY == 1'b1 && IPIPE_PNTR == 2'd3) begin
        if (OPCODE_RDY_I == 1'b1) begin
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
    end else if (OW_REQ == 1'b1 && INSTR_LVL == LVL_C && PIPE_RDY == 1'b1 && IPIPE_PNTR == 2'd2) begin
        if (OPCODE_RDY_I == 1'b1) begin
            IPIPE_D <= OPCODE_DATA;
            IPIPE_D_FAULT <= ~OPCODE_VALID;
            IPIPE_PNTR <= IPIPE_PNTR - 2'd1;
        end else begin
            IPIPE_PNTR <= 2'd0;
        end
    end else if (OW_REQ == 1'b1 && INSTR_LVL == LVL_C && PIPE_RDY == 1'b1 && IPIPE_PNTR == 2'd3) begin
        if (OPCODE_RDY_I == 1'b1) begin
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
    end else if (OW_REQ == 1'b1 && INSTR_LVL == LVL_B && PIPE_RDY == 1'b1) begin // IPIPE_PNTR = 3.
        if (OPCODE_RDY_I == 1'b1) begin
            IPIPE_D <= OPCODE_DATA;
            IPIPE_D_FAULT <= ~OPCODE_VALID;
            IPIPE_PNTR <= IPIPE_PNTR - 2'd2;
        end else begin
            IPIPE_PNTR <= 2'd0;
        end
    end else if (EW_REQ == 1'b1 && IPIPE_PNTR >= 2'd1) begin
        case (IPIPE_PNTR)
            2'd3: begin
                if (OPCODE_RDY_I == 1'b1) begin
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
                if (OPCODE_RDY_I == 1'b1) begin
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
                if (OPCODE_RDY_I == 1'b1) begin
                    IPIPE_D <= OPCODE_DATA;
                    IPIPE_D_FAULT <= ~OPCODE_VALID;
                end else begin
                    IPIPE_PNTR <= 2'd0;
                end
            end
            default: ;
        endcase
    end else if (OPCODE_RDY_I == 1'b1) begin
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
always_ff @(posedge CLK) begin : P_FAULT
    if (IPIPE_FLUSH == 1'b1) begin
        OW_VALID <= 1'b0;
        FC <= 1'b0;
        FB <= 1'b0;
    end else if (OW_REQ == 1'b1 && LOOP_BSY_I == 1'b1) begin
        OW_VALID <= 1'b1;
    end else if (OW_REQ == 1'b1 && PIPE_RDY == 1'b1 && INSTR_LVL == LVL_D) begin
        OW_VALID <= ~IPIPE_D_FAULT;
        FC <= 1'b0;
        FB <= 1'b0;
    end else if (OW_REQ == 1'b1 && PIPE_RDY == 1'b1 && INSTR_LVL == LVL_C) begin
        OW_VALID <= ~(IPIPE_D_FAULT | IPIPE_C_FAULT);
        FC <= IPIPE_C_FAULT;
        FB <= 1'b0;
    end else if (OW_REQ == 1'b1 && PIPE_RDY == 1'b1 && INSTR_LVL == LVL_B) begin
        OW_VALID <= ~(IPIPE_D_FAULT | IPIPE_C_FAULT | IPIPE_B_FAULT);
        FC <= IPIPE_C_FAULT;
        FB <= IPIPE_B_FAULT;
    end else if (EW_REQ == 1'b1 && PIPE_RDY == 1'b1) begin
        OW_VALID <= ~IPIPE_D_FAULT;
        FC <= 1'b0;
        FB <= 1'b0;
    end
    // The Rerun Flags:
    if (IPIPE_FLUSH == 1'b1) begin
        RC <= 1'b0;
        RB <= 1'b0;
    end else if ((EW_REQ | OW_REQ) == 1'b1 && PIPE_RDY == 1'b1) begin
        RC <= IPIPE_C_FAULT;
        RB <= IPIPE_B_FAULT;
    end
end

// OUTBUFFERS
always_ff @(posedge CLK) begin : OUTBUFFERS
    reg OP_STOP;
    if (OP_STOP == 1'b1 && IPIPE_FLUSH == 1'b1) begin
        TRAP_CODE <= NONE;
        OP_STOP = 1'b0;
    end else if (IPIPE_FLUSH == 1'b1) begin
        TRAP_CODE <= NONE;
    end else if (OP_STOP == 1'b1) begin
        ; // Do not update after PC is incremented.
    end else if (LOOP_ATN == 1'b1 && OPCODE_RD_I == 1'b1) begin
        ; // Wait for pending opcodes.
    end else if (OW_REQ == 1'b1 && LOOP_BSY_I == 1'b1) begin
        OP <= OP_I;
        BIW_0 <= IPIPE_D;
        TRAP_CODE <= TRAP_CODE_I;
    end else if (OW_REQ == 1'b1 && (PIPE_RDY == 1'b1 || BKPT_REQ == 1'b1)) begin
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
    end else if (EW_REQ == 1'b1 && IPIPE_PNTR != 2'd0) begin
        EXT_WORD <= IPIPE_D;
    end
end

// LOOP_OP
generate
    if (NO_LOOP) begin : GEN_NO_LOOP
        assign LOOP_OP = 1'b0;
    end else begin : GEN_LOOP
        assign LOOP_OP = (OP == MOVE && BIW_0[8:3] == 6'b010010) ? 1'b1 : // (Ay) to (Ax).
                         (OP == MOVE && BIW_0[8:3] == 6'b011010) ? 1'b1 : // (Ay) to (Ax)+.
                         (OP == MOVE && BIW_0[8:3] == 6'b100010) ? 1'b1 : // (Ay) to -(Ax).
                         (OP == MOVE && BIW_0[8:3] == 6'b010011) ? 1'b1 : // (Ay)+ to (Ax).
                         (OP == MOVE && BIW_0[8:3] == 6'b011011) ? 1'b1 : // (Ay)+ to (Ax)+.
                         (OP == MOVE && BIW_0[8:3] == 6'b100011) ? 1'b1 : // (Ay)+ to -(Ax).
                         (OP == MOVE && BIW_0[8:3] == 6'b010100) ? 1'b1 : // -(Ay) to (Ax).
                         (OP == MOVE && BIW_0[8:3] == 6'b011100) ? 1'b1 : // -(Ay) to (Ax)+.
                         (OP == MOVE && BIW_0[8:3] == 6'b100100) ? 1'b1 : // -(Ay) to -(Ax).
                         (OP == MOVE && BIW_0[8:3] == 6'b010000) ? 1'b1 : // Dy to (Ax).
                         (OP == MOVE && BIW_0[8:3] == 6'b011000) ? 1'b1 : // Dy to (Ax)+.
                         (OP == MOVE && BIW_0[8:3] == 6'b010001) ? 1'b1 : // Ay to (Ax).
                         (OP == MOVE && BIW_0[8:3] == 6'b011001) ? 1'b1 : // Ay to (Ax)+.
                         ((OP == ADD || OP == AND_B || OP == CMP || OP == EOR || OP == OR_B || OP == SUB) && BIW_0[5:3] == 3'b010) ? 1'b1 : // (Ay) to Dx, Dx to (Ay).
                         ((OP == ADD || OP == AND_B || OP == CMP || OP == EOR || OP == OR_B || OP == SUB) && BIW_0[5:3] == 3'b011) ? 1'b1 : // (Ay)+ to Dx, Dx to (Ay)+.
                         ((OP == ADD || OP == AND_B || OP == CMP || OP == EOR || OP == OR_B || OP == SUB) && BIW_0[5:3] == 3'b100) ? 1'b1 : // -(Ay) to Dx, Dx to -(Ay).
                         ((OP == ADDA || OP == CMPA || OP == SUBA) && BIW_0[5:3] == 3'b010) ? 1'b1 : // (Ay) to Ax.
                         ((OP == ADDA || OP == CMPA || OP == SUBA) && BIW_0[5:3] == 3'b011) ? 1'b1 : // (Ay)+ to Ax.
                         ((OP == ADDA || OP == CMPA || OP == SUBA) && BIW_0[5:3] == 3'b100) ? 1'b1 : // -(Ay) to Ax.
                         (OP == ABCD || OP == SBCD || OP == ADDX || OP == SUBX || OP == CMPM) ? 1'b1 : // -(Ay) to -(Ay), (Ay)+ to (Ay)+ for CMPM.
                         ((OP == CLR || OP == NEG || OP == NEGX || OP == NOT_B || OP == TST || OP == NBCD) && BIW_0[5:3] == 3'b010) ? 1'b1 : // (Ay).
                         ((OP == CLR || OP == NEG || OP == NEGX || OP == NOT_B || OP == TST || OP == NBCD) && BIW_0[5:3] == 3'b011) ? 1'b1 : // (Ay)+.
                         ((OP == CLR || OP == NEG || OP == NEGX || OP == NOT_B || OP == TST || OP == NBCD) && BIW_0[5:3] == 3'b100) ? 1'b1 : // -(Ay).
                         ((OP == ASL || OP == ASR || OP == LSL || OP == LSR) && BIW_0[7:3] == 5'b11010) ? 1'b1 : // (Ay) by #1.
                         ((OP == ASL || OP == ASR || OP == LSL || OP == LSR) && BIW_0[7:3] == 5'b11011) ? 1'b1 : // (Ay)+ by #1.
                         ((OP == ASL || OP == ASR || OP == LSL || OP == LSR) && BIW_0[7:3] == 5'b11100) ? 1'b1 : // -(Ay) by #1.
                         ((OP == ROTL || OP == ROTR || OP == ROXL || OP == ROXR) && BIW_0[7:3] == 5'b11010) ? 1'b1 : // (Ay) by #1.
                         ((OP == ROTL || OP == ROTR || OP == ROXL || OP == ROXR) && BIW_0[7:3] == 5'b11011) ? 1'b1 : // (Ay)+ by #1.
                         ((OP == ROTL || OP == ROTR || OP == ROXL || OP == ROXR) && BIW_0[7:3] == 5'b11100) ? 1'b1 : 1'b0; // -(Ay) by #1.
    end
endgenerate

// LOOP_ATN
assign LOOP_ATN = (EXH_REQ == 1'b1) ? 1'b0 :
                  (LOOP_BSY_I == 1'b0 && LOOP_OP == 1'b1 && OP_I == DBcc && IPIPE_C == 16'hFFFC) ? 1'b1 : 1'b0; // IPIPE_C value must be minus four.

// P_LOOP
always_ff @(posedge CLK) begin : P_LOOP
    if (LOOP_ATN == 1'b1 && OW_REQ == 1'b1 && OPCODE_RD_I == 1'b0) begin
        LOOP_BSY <= 1'b1;
        LOOP_BSY_I <= 1'b1;
    end else if (LOOP_EXIT == 1'b1 || BUSY_EXH == 1'b1) begin
        LOOP_BSY <= 1'b0;
        LOOP_BSY_I <= 1'b0;
    end
end

assign OW_REQ = (BUSY_EXH == 1'b1) ? 1'b0 : OW_REQ_MAIN;
assign EW_REQ = EW_REQ_MAIN;

assign PIPE_RDY = (OW_REQ == 1'b1 && IPIPE_PNTR == 2'd3 && INSTR_LVL == LVL_B) ? 1'b1 :
                  (OW_REQ == 1'b1 && IPIPE_PNTR > 2'd1 && INSTR_LVL == LVL_C) ? 1'b1 :
                  (OW_REQ == 1'b1 && IPIPE_PNTR > 2'd1 && INSTR_LVL == LVL_D) ? 1'b1 : // We need always pipe C and D to determine the INSTR_LVL.
                  (EW_REQ == 1'b1 && IPIPE_PNTR > 2'd0) ? 1'b1 : 1'b0;

// HANDSHAKING
always_ff @(posedge CLK) begin : HANDSHAKING
    if (EW_REQ == 1'b1 && IPIPE_PNTR != 2'd0) begin
        EW_ACK <= 1'b1;
    end else begin
        EW_ACK <= 1'b0;
    end

    if (IPIPE_FLUSH == 1'b1) begin
        OPD_ACK_MAIN <= 1'b0;
    end else if (TRAP_CODE_I == T_PRIV) begin // No action when privileged.
        OPD_ACK_MAIN <= 1'b0;
    end else if (OW_REQ == 1'b1 && LOOP_BSY_I == 1'b1) begin
        OPD_ACK_MAIN <= 1'b1;
    end else if (OW_REQ == 1'b1 && (PIPE_RDY == 1'b1 || BKPT_REQ == 1'b1)) begin
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
always_ff @(posedge CLK) begin : P_PC_OFFSET
    reg [6:0] ADR_OFFSET;
    reg [6:0] PC_VAR;
    reg [6:0] PC_VAR_MEM;

    if (IPIPE_FLUSH == 1'b1) begin
        ADR_OFFSET = 7'b0000000;
    end else if (PC_INC_I == 1'b1 && OPCODE_RDY_I == 1'b1) begin
        ADR_OFFSET = ADR_OFFSET + 7'd1 - PC_VAR;
    end else if (OPCODE_RDY_I == 1'b1) begin
        ADR_OFFSET = ADR_OFFSET + 7'd1;
    end else if (PC_INC_I == 1'b1) begin
        ADR_OFFSET = ADR_OFFSET - PC_VAR;
    end
    //
    if (BUSY_EXH == 1'b0) begin
        PC_VAR_MEM = PC_VAR; // Store the old offset to write back on the stack.
    end

    if (BUSY_EXH == 1'b1) begin
        // New PC is loaded by the exception handler.
        // So PC_VAR must be initialized.
        PC_VAR = 7'b0000000;
    end else if (PC_INC_I == 1'b1 || FLUSHED == 1'b1) begin
        case (INSTR_LVL)
            LVL_D: PC_VAR = 7'b0000001;
            LVL_C: PC_VAR = 7'b0000010;
            LVL_B: PC_VAR = 7'b0000011;
        endcase
    end else if (EW_REQ == 1'b1 && IPIPE_PNTR != 2'd0) begin
        PC_VAR = PC_VAR + 7'd1;
    end
    //
    if (OW_REQ == 1'b1 && BKPT_REQ == 1'b1) begin
        PC_EW_OFFSET <= 4'b0010; // Always level D operations.
    end else if (OW_REQ == 1'b1 && PIPE_RDY == 1'b1 && OP_I == JSR) begin // Initialize.
        PC_EW_OFFSET <= 4'h0;
    end else if (OW_REQ == 1'b1 && PIPE_RDY == 1'b1) begin // BSR.
        case (INSTR_LVL)
            LVL_D: PC_EW_OFFSET <= 4'b0010;
            LVL_C: PC_EW_OFFSET <= 4'b0100;
            default: PC_EW_OFFSET <= 4'b0110; // LONG displacement.
        endcase
    end else if (EW_ACK == 1'b1 && OP == JSR) begin // Calculate the required extension words.
        PC_EW_OFFSET <= PC_EW_OFFSET + 4'b0010;
    end

    ADR_OFFSET_S <= ADR_OFFSET;
    PC_VAR_S <= PC_VAR;
    PC_VAR_MEM_S <= PC_VAR_MEM;
end

// P_PC_OFFSET_COMB
always_comb begin : P_PC_OFFSET_COMB
    if (BUSY_EXH == 1'b1 && PC_INC_I == 1'b1) begin
        PC_OFFSET = {PC_VAR_MEM_S, 1'b0};
    end else if (OP == DBcc && LOOP_BSY_I == 1'b1 && LOOP_EXIT == 1'b0) begin
        // Suppress to increment after DBcc operation during the loop to
        // handle a correct PC with displacement when looping around.
        PC_OFFSET = 8'h00;
    end else begin
        PC_OFFSET = {PC_VAR_S, 1'b0};
    end
    PC_ADR_OFFSET = {ADR_OFFSET_S, 1'b0};
end

// P_FLUSH
always_ff @(posedge CLK) begin : P_FLUSH
    if (IPIPE_FLUSH == 1'b1) begin
        FLUSHED <= 1'b1;
    end else if (OW_REQ == 1'b1 && PIPE_RDY == 1'b1) begin
        FLUSHED <= 1'b0;
    end
end

assign PC_INC = PC_INC_I;
assign PC_INC_I = (FLUSHED == 1'b1) ? 1'b0 : // Avoid double increment after a flushed pipe.
                  (IPIPE_FLUSH == 1'b1 && BUSY_MAIN == 1'b1) ? 1'b1 : // If the pipe is flushed, we need the new PC value for refilling.
                  (BKPT_REQ == 1'b1) ? 1'b0 : // Do not update!
                  (OW_REQ == 1'b1 && PIPE_RDY == 1'b1) ? 1'b1 : PC_INC_EXH;

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
                   ((OP_I == BCHG || OP_I == BCLR || OP_I == BSET || OP_I == BTST) && IPIPE_D[8] == 1'b0) ? LVL_C :
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
                     (OP_I == RTE && SBIT == 1'b1) ? T_RTE : // Handled like a trap simplifies the code.
                     (OP_I == TRAP) ? T_TRAP :
                     (OP_I == ANDI_TO_SR && SBIT == 1'b0) ? T_PRIV :
                     (OP_I == EORI_TO_SR && SBIT == 1'b0) ? T_PRIV :
                     ((OP_I == MOVE_FROM_SR || OP_I == MOVE_TO_SR) && SBIT == 1'b0) ? T_PRIV :
                     ((OP_I == MOVE_USP || OP_I == MOVEC || OP_I == MOVES) && SBIT == 1'b0) ? T_PRIV :
                     (OP_I == ORI_TO_SR && SBIT == 1'b0) ? T_PRIV :
                     ((OP_I == OP_RESET || OP_I == RTE) && SBIT == 1'b0) ? T_PRIV :
                     (OP_I == STOP && SBIT == 1'b0) ? T_PRIV : NONE;

// OP_DECODE
always_comb begin : OP_DECODE
    // The default OPCODE is the ILLEGAL operation, if none of the following conditions are met.
    OP_I = ILLEGAL;
    case (IPIPE_D[15:12]) // Operation code map.
        4'h0: begin // Bit manipulation / MOVEP / Immediate.
            if (IPIPE_D[11:0] == 12'h03C) begin
                OP_I = ORI_TO_CCR;
            end else if (IPIPE_D[11:0] == 12'h07C) begin
                OP_I = ORI_TO_SR;
            end else if (IPIPE_D[11:0] == 12'h23C) begin
                OP_I = ANDI_TO_CCR;
            end else if (IPIPE_D[11:0] == 12'h27C) begin
                OP_I = ANDI_TO_SR;
            end else if (IPIPE_D[11:0] == 12'hA3C) begin
                OP_I = EORI_TO_CCR;
            end else if (IPIPE_D[11:0] == 12'hA7C) begin
                OP_I = EORI_TO_SR;
            end else if (IPIPE_D[11:0] == 12'b110011111100) begin
                OP_I = CAS2;
            end else if (IPIPE_D[11:0] == 12'b111011111100) begin
                OP_I = CAS2;
            end else if (IPIPE_D[11:8] == 4'b1110 && IPIPE_D[7:6] < 2'b11 && IPIPE_D[5:3] >= 3'b010 && IPIPE_D[5:3] < 3'b111) begin
                OP_I = MOVES;
            end else if (IPIPE_D[11:8] == 4'b1110 && IPIPE_D[7:6] < 2'b11 && IPIPE_D[5:3] == 3'b111 && IPIPE_D[2:0] < 3'b010) begin
                OP_I = MOVES;
            end else if (IPIPE_D[8:6] > 3'b011 && IPIPE_D[5:3] == 3'b001) begin
                OP_I = MOVEP;
            end else if (IPIPE_D[11] == 1'b0 && IPIPE_D[10:9] != 2'b11 && IPIPE_D[8:6] == 3'b011 && IPIPE_D[5:3] == 3'b010 && IPIPE_C[11] == 1'b1) begin
                OP_I = CHK2;
            end else if (IPIPE_D[11] == 1'b0 && IPIPE_D[10:9] != 2'b11 && IPIPE_D[8:6] == 3'b011 && IPIPE_D[5:3] > 3'b100 && IPIPE_D[5:3] < 3'b111 && IPIPE_C[11] == 1'b1) begin
                OP_I = CHK2;
            end else if (IPIPE_D[11] == 1'b0 && IPIPE_D[10:9] != 2'b11 && IPIPE_D[8:6] == 3'b011 && IPIPE_D[5:3] == 3'b111 && IPIPE_D[2:0] < 3'b100 && IPIPE_C[11] == 1'b1) begin
                OP_I = CHK2;
            end else if (IPIPE_D[11] == 1'b0 && IPIPE_D[10:9] != 2'b11 && IPIPE_D[8:6] == 3'b011 && IPIPE_D[5:3] == 3'b010 && IPIPE_C[11] == 1'b0) begin
                OP_I = CMP2;
            end else if (IPIPE_D[11] == 1'b0 && IPIPE_D[10:9] != 2'b11 && IPIPE_D[8:6] == 3'b011 && IPIPE_D[5:3] > 3'b100 && IPIPE_D[5:3] < 3'b111 && IPIPE_C[11] == 1'b0) begin
                OP_I = CMP2;
            end else if (IPIPE_D[11] == 1'b0 && IPIPE_D[10:9] != 2'b11 && IPIPE_D[8:6] == 3'b011 && IPIPE_D[5:3] == 3'b111 && IPIPE_D[2:0] < 3'b100 && IPIPE_C[11] == 1'b0) begin
                OP_I = CMP2;
            end else if (IPIPE_D[11] == 1'b1 && IPIPE_D[10:9] != 2'b00 && IPIPE_D[8:6] == 3'b011 && IPIPE_D[5:3] > 3'b001 && IPIPE_D[5:3] < 3'b111) begin
                OP_I = CAS;
            end else if (IPIPE_D[11] == 1'b1 && IPIPE_D[10:9] != 2'b00 && IPIPE_D[8:6] == 3'b011 && IPIPE_D[5:3] == 3'b111 && IPIPE_D[2:0] < 3'b010) begin
                OP_I = CAS;
            end else begin
                case (IPIPE_D[5:3]) // Addressing mode.
                    3'b000, 3'b010, 3'b011, 3'b100, 3'b101, 3'b110: begin
                        // Bit operations with static bit number:
                        if (IPIPE_D[11:6] == 6'b100000) begin
                            OP_I = BTST;
                        end else if (IPIPE_D[11:6] == 6'b100001) begin
                            OP_I = BCHG;
                        end else if (IPIPE_D[11:6] == 6'b100010) begin
                            OP_I = BCLR;
                        end else if (IPIPE_D[11:6] == 6'b100011) begin
                            OP_I = BSET;
                        // Logic operations:
                        end else if (IPIPE_D[11:8] == 4'h0 && IPIPE_D[7:6] < 2'b11) begin
                            OP_I = ORI;
                        end else if (IPIPE_D[11:8] == 4'h2 && IPIPE_D[7:6] < 2'b11) begin
                            OP_I = ANDI;
                        end else if (IPIPE_D[11:8] == 4'h4 && IPIPE_D[7:6] < 2'b11) begin
                            OP_I = SUBI;
                        end else if (IPIPE_D[11:8] == 4'h6 && IPIPE_D[7:6] < 2'b11) begin
                            OP_I = ADDI;
                        end else if (IPIPE_D[11:8] == 4'hA && IPIPE_D[7:6] < 2'b11) begin
                            OP_I = EORI;
                        end else if (IPIPE_D[11:8] == 4'hC && IPIPE_D[7:6] < 2'b11) begin
                            OP_I = CMPI;
                        // Bit operations with dynamic bit number:
                        end else if (IPIPE_D[8:6] == 3'b100) begin
                            OP_I = BTST;
                        end else if (IPIPE_D[8:6] == 3'b101) begin
                            OP_I = BCHG;
                        end else if (IPIPE_D[8:6] == 3'b110) begin
                            OP_I = BCLR;
                        end else if (IPIPE_D[8:6] == 3'b111) begin
                            OP_I = BSET;
                        end
                    end
                    3'b111: begin
                        // In the addressing mode "111" not all register selections are valid.
                        // Bit operations with static bit number:
                        if (IPIPE_D[11:6] == 6'b100000 && IPIPE_D[2:0] < 3'b100) begin
                            OP_I = BTST;
                        end else if (IPIPE_D[11:6] == 6'b100001 && IPIPE_D[2:0] < 3'b010) begin
                            OP_I = BCHG;
                        end else if (IPIPE_D[11:6] == 6'b100010 && IPIPE_D[2:0] < 3'b010) begin
                            OP_I = BCLR;
                        end else if (IPIPE_D[11:6] == 6'b100011 && IPIPE_D[2:0] < 3'b010) begin
                            OP_I = BSET;
                        // Logic operations:
                        end else if (IPIPE_D[11:8] == 4'h0 && IPIPE_D[7:6] < 2'b11 && IPIPE_D[2:0] < 3'b010) begin
                            OP_I = ORI;
                        end else if (IPIPE_D[11:8] == 4'h2 && IPIPE_D[7:6] < 2'b11 && IPIPE_D[2:0] < 3'b010) begin
                            OP_I = ANDI;
                        end else if (IPIPE_D[11:8] == 4'h4 && IPIPE_D[7:6] < 2'b11 && IPIPE_D[2:0] < 3'b010) begin
                            OP_I = SUBI;
                        end else if (IPIPE_D[11:8] == 4'h6 && IPIPE_D[7:6] < 2'b11 && IPIPE_D[2:0] < 3'b010) begin
                            OP_I = ADDI;
                        end else if (IPIPE_D[11:8] == 4'hA && IPIPE_D[7:6] < 2'b11 && IPIPE_D[2:0] < 3'b010) begin
                            OP_I = EORI;
                        end else if (IPIPE_D[11:8] == 4'hC && IPIPE_D[7:6] < 2'b11 && IPIPE_D[2:0] < 3'b100) begin
                            OP_I = CMPI;
                        // Bit operations with dynamic bit number:
                        end else if (IPIPE_D[8:6] == 3'b100 && IPIPE_D[2:0] < 3'b101) begin
                            OP_I = BTST;
                        end else if (IPIPE_D[8:6] == 3'b101 && IPIPE_D[2:0] < 3'b010) begin
                            OP_I = BCHG;
                        end else if (IPIPE_D[8:6] == 3'b110 && IPIPE_D[2:0] < 3'b010) begin
                            OP_I = BCLR;
                        end else if (IPIPE_D[8:6] == 3'b111 && IPIPE_D[2:0] < 3'b010) begin
                            OP_I = BSET;
                        end
                    end
                    default: ;
                endcase
            end
        end
        4'h1: begin // Move BYTE.
            if (IPIPE_D[8:6] == 3'b111 && IPIPE_D[11:9] < 3'b010
                    && IPIPE_D[5:3] == 3'b111 && IPIPE_D[2:0] < 3'b101) begin
                OP_I = MOVE;
            end else if (IPIPE_D[8:6] == 3'b111 && IPIPE_D[11:9] < 3'b010 && IPIPE_D[5:3] != 3'b001 && IPIPE_D[5:3] != 3'b111) begin
                OP_I = MOVE;
            end else if (IPIPE_D[8:6] != 3'b001 && IPIPE_D[8:6] != 3'b111
                    && IPIPE_D[5:3] == 3'b111 && IPIPE_D[2:0] < 3'b101) begin
                OP_I = MOVE;
            end else if (IPIPE_D[8:6] != 3'b001 && IPIPE_D[8:6] != 3'b111 && IPIPE_D[5:3] != 3'b001 && IPIPE_D[5:3] != 3'b111) begin
                OP_I = MOVE;
            end
        end
        4'h2, 4'h3: begin // Move WORD or LONG.
            if (IPIPE_D[8:6] == 3'b111 && IPIPE_D[11:9] < 3'b010
                    && IPIPE_D[5:3] == 3'b111 && IPIPE_D[2:0] < 3'b101) begin
                OP_I = MOVE;
            end else if (IPIPE_D[8:6] == 3'b111 && IPIPE_D[11:9] < 3'b010 && IPIPE_D[5:3] != 3'b111) begin
                OP_I = MOVE;
            end else if (IPIPE_D[8:6] == 3'b001 && IPIPE_D[5:3] == 3'b111 && IPIPE_D[2:0] < 3'b101) begin
                OP_I = MOVEA;
            end else if (IPIPE_D[8:6] == 3'b001 && IPIPE_D[5:3] != 3'b111) begin
                OP_I = MOVEA;
            end else if (IPIPE_D[8:6] != 3'b001 && IPIPE_D[8:6] != 3'b111 && IPIPE_D[5:3] == 3'b111 && IPIPE_D[2:0] < 3'b101) begin
                OP_I = MOVE;
            end else if (IPIPE_D[8:6] != 3'b001 && IPIPE_D[8:6] != 3'b111 && IPIPE_D[5:3] != 3'b111) begin
                OP_I = MOVE;
            end
        end
        4'h4: begin // Miscellaneous.
            if (IPIPE_D[11:0] == 12'hE70) begin
                OP_I = OP_RESET;
            end else if (IPIPE_D[11:0] == 12'hE71) begin
                OP_I = NOP;
            end else if (IPIPE_D[11:0] == 12'hE72) begin
                OP_I = STOP;
            end else if (IPIPE_D[11:0] == 12'hE73) begin
                OP_I = RTE;
            end else if (IPIPE_D[11:0] == 12'hE74) begin
                OP_I = RTD;
            end else if (IPIPE_D[11:0] == 12'hE75) begin
                OP_I = RTS;
            end else if (IPIPE_D[11:0] == 12'hE76) begin
                OP_I = TRAPV;
            end else if (IPIPE_D[11:0] == 12'hE77) begin
                OP_I = RTR;
            end else if (IPIPE_D[11:0] == 12'hAFC) begin
                OP_I = ILLEGAL;
            end else if (IPIPE_D[11:1] == 11'b11100111101 && IPIPE_C[11:0] == 12'h000) begin
                OP_I = MOVEC;
            end else if (IPIPE_D[11:1] == 11'b11100111101 && IPIPE_C[11:0] == 12'h001) begin
                OP_I = MOVEC;
            end else if (IPIPE_D[11:1] == 11'b11100111101 && IPIPE_C[11:0] == 12'h800) begin
                OP_I = MOVEC;
            end else if (IPIPE_D[11:1] == 11'b11100111101 && IPIPE_C[11:0] == 12'h801) begin
                OP_I = MOVEC;
            end else if (IPIPE_D[11:1] == 11'b11100111101) begin
                OP_I = ILLEGAL; // Not valid MOVEC patterns.
            end else if (IPIPE_D[11:3] == 9'b100001001) begin // 68K20, 68K30, 68K40
                OP_I = BKPT;
            end else if (IPIPE_D[11:3] == 9'b100000001) begin // 68K20, 68K30, 68K40
                OP_I = LINK; // LONG.
            end else if (IPIPE_D[11:3] == 9'b111001010) begin
                OP_I = LINK; // WORD.
            end else if (IPIPE_D[11:3] == 9'b111001011) begin
                OP_I = UNLK;
            end else if (IPIPE_D[11:3] == 9'b100001000) begin
                OP_I = SWAP;
            end else if (IPIPE_D[11:4] == 8'hE4) begin
                OP_I = TRAP;
            end else if (IPIPE_D[11:4] == 8'hE6) begin
                OP_I = MOVE_USP;
            end else begin
                case (IPIPE_D[5:3]) // Addressing mode.
                    3'b000, 3'b010, 3'b011, 3'b100, 3'b101, 3'b110: begin
                        if (IPIPE_D[11:6] == 6'b110001) begin
                            if (IPIPE_C[11] == 1'b1) begin
                                OP_I = DIVS; // Long.
                            end else begin
                                OP_I = DIVU; // Long.
                            end
                        end else if (IPIPE_D[11:6] == 6'b001011) begin
                            OP_I = MOVE_FROM_CCR;
                        end else if (IPIPE_D[11:6] == 6'b000011) begin
                            OP_I = MOVE_FROM_SR;
                        end else if (IPIPE_D[11:6] == 6'b010011) begin
                            OP_I = MOVE_TO_CCR;
                        end else if (IPIPE_D[11:6] == 6'b011011) begin
                            OP_I = MOVE_TO_SR;
                        end else if (IPIPE_D[11:6] == 6'b110000) begin
                            if (IPIPE_C[11] == 1'b1) begin
                                OP_I = MULS; // Long.
                            end else begin
                                OP_I = MULU; // Long.
                            end
                        end else if (IPIPE_D[11:6] == 6'b100000) begin
                            OP_I = NBCD;
                        end else if (IPIPE_D[11:6] == 6'b101011) begin
                            OP_I = TAS;
                        end
                    end
                    3'b111: begin // Not all registers are valid for this mode.
                        if (IPIPE_D[11:6] == 6'b110001 && IPIPE_D[2:0] < 3'b101) begin
                            if (IPIPE_C[11] == 1'b1) begin
                                OP_I = DIVS; // Long.
                            end else begin
                                OP_I = DIVU; // Long.
                            end
                        end else if (IPIPE_D[11:6] == 6'b001011 && IPIPE_D[2:0] < 3'b010) begin
                            OP_I = MOVE_FROM_CCR;
                        end else if (IPIPE_D[11:6] == 6'b000011 && IPIPE_D[2:0] < 3'b010) begin
                            OP_I = MOVE_FROM_SR;
                        end else if (IPIPE_D[11:6] == 6'b010011 && IPIPE_D[2:0] < 3'b101) begin
                            OP_I = MOVE_TO_CCR;
                        end else if (IPIPE_D[11:6] == 6'b011011 && IPIPE_D[2:0] < 3'b101) begin
                            OP_I = MOVE_TO_SR;
                        end else if (IPIPE_D[11:6] == 6'b110000 && IPIPE_D[2:0] < 3'b101) begin
                            if (IPIPE_C[11] == 1'b1) begin
                                OP_I = MULS; // Long.
                            end else begin
                                OP_I = MULU; // Long.
                            end
                        end else if (IPIPE_D[11:6] == 6'b100000 && IPIPE_D[2:0] < 3'b010) begin
                            OP_I = NBCD;
                        end else if (IPIPE_D[11:6] == 6'b101011 && IPIPE_D[2:0] < 3'b010) begin
                            OP_I = TAS;
                        end
                    end
                    default: ;
                endcase

                case (IPIPE_D[5:3]) // Addressing mode.
                    3'b010, 3'b101, 3'b110: begin
                        if (IPIPE_D[11:6] == 6'b100001) begin
                            OP_I = PEA;
                        end else if (IPIPE_D[11:6] == 6'b111010) begin
                            OP_I = JSR;
                        end else if (IPIPE_D[11:6] == 6'b111011) begin
                            OP_I = JMP;
                        end
                    end
                    3'b111: begin // Not all registers are valid for this mode.
                        if (IPIPE_D[11:6] == 6'b100001 && IPIPE_D[2:0] < 3'b100) begin
                            OP_I = PEA;
                        end else if (IPIPE_D[11:6] == 6'b111010 && IPIPE_D[2:0] < 3'b100) begin
                            OP_I = JSR;
                        end else if (IPIPE_D[11:6] == 6'b111011 && IPIPE_D[2:0] < 3'b100) begin
                            OP_I = JMP;
                        end
                    end
                    default: ;
                endcase

                // For the following operation codes a SIZE (IPIPE_D[7:6]) is not valid.
                // For the following operation codes an addressing mode x"001" is not valid.
                if (IPIPE_D[7:6] < 2'b11 && IPIPE_D[5:3] != 3'b001 && IPIPE_D[5:3] != 3'b111) begin
                    case (IPIPE_D[11:8])
                        4'h0: OP_I = NEGX;
                        4'h2: OP_I = CLR;
                        4'h4: OP_I = NEG;
                        4'h6: OP_I = NOT_B;
                        default: ;
                    endcase
                // Not all registers are valid for the addressing mode "111":
                end else if (IPIPE_D[7:6] < 2'b11 && IPIPE_D[5:3] == 3'b111 && IPIPE_D[2:0] < 3'b010) begin
                    case (IPIPE_D[11:8])
                        4'h0: OP_I = NEGX;
                        4'h2: OP_I = CLR;
                        4'h4: OP_I = NEG;
                        4'h6: OP_I = NOT_B;
                        default: ;
                    endcase
                end

                if (IPIPE_D[11:8] == 4'hA && IPIPE_D[7:6] < 2'b11 && IPIPE_D[5:3] == 3'b111 && IPIPE_D[2:0] < 3'b101) begin
                    case (IPIPE_D[7:6])
                        2'b01, 2'b10: OP_I = TST; // Long or word, all addressing modes.
                        default: begin // Byte: Address register direct not allowed.
                            if (IPIPE_D[2:0] != 3'b100) begin
                                OP_I = TST;
                            end
                        end
                    endcase
                end else if (IPIPE_D[11:8] == 4'hA && IPIPE_D[7:6] < 2'b11 && IPIPE_D[5:3] != 3'b111) begin
                    case (IPIPE_D[7:6])
                        2'b01, 2'b10: OP_I = TST; // Long or word, all addressing modes.
                        default: begin // Byte: Address register direct not allowed.
                            if (IPIPE_D[5:3] != 3'b001) begin
                                OP_I = TST;
                            end
                        end
                    endcase
                end

                if (IPIPE_D[11:9] == 3'b100 && IPIPE_D[5:3] == 3'b000) begin
                    case (IPIPE_D[8:6]) // Valid OPMODES for this operation code.
                        3'b010, 3'b011: OP_I = EXT;
                        3'b111: OP_I = EXTB;
                        default: ;
                    endcase
                end

                if (IPIPE_D[8:6] == 3'b111) begin
                    case (IPIPE_D[5:3]) // OPMODES.
                        3'b010, 3'b101, 3'b110:
                            OP_I = LEA;
                        3'b111: begin
                            if (IPIPE_D[2:0] < 3'b100) begin // Not all registers are valid for this OPMODE.
                                OP_I = LEA;
                            end
                        end
                        default: ;
                    endcase
                end

                if (IPIPE_D[11] == 1'b1 && IPIPE_D[9:7] == 3'b001) begin
                    if (IPIPE_D[10] == 1'b0) begin // Register to memory transfer.
                        case (IPIPE_D[5:3]) // OPMODES, no postincrement addressing.
                            3'b010, 3'b100, 3'b101, 3'b110:
                                OP_I = MOVEM;
                            3'b111: begin
                                if (IPIPE_D[2:0] == 3'b000 || IPIPE_D[2:0] == 3'b001) begin
                                    OP_I = MOVEM;
                                end
                            end
                            default: ;
                        endcase
                    end else begin // Memory to register transfer, no predecrement addressing.
                        case (IPIPE_D[5:3]) // OPMODES.
                            3'b010, 3'b011, 3'b101, 3'b110:
                                OP_I = MOVEM;
                            3'b111: begin
                                if (IPIPE_D[2:0] < 3'b100) begin
                                    OP_I = MOVEM;
                                end
                            end
                            default: ;
                        endcase
                    end
                end

                // The size must be "10" or "11" and the OPMODE may not be "001".
                if (IPIPE_D[8:7] >= 2'b10 && IPIPE_D[6:3] == 4'h7 && IPIPE_D[2:0] < 3'b101) begin
                    OP_I = CHK;
                end else if (IPIPE_D[8:7] >= 2'b10 && IPIPE_D[6:3] != 4'h1 && IPIPE_D[6:3] < 4'h7) begin
                    OP_I = CHK;
                end
            end
        end
        4'h5: begin // ADDQ / SUBQ / Scc / DBcc / TRAPcc.
            if (IPIPE_D[7:3] == 5'b11001) begin
                OP_I = DBcc;
            end else if (IPIPE_D[7:6] == 2'b11 && IPIPE_D[5:3] == 3'b111 && IPIPE_D[2:0] < 3'b010) begin
                OP_I = Scc;
            end else if (IPIPE_D[7:6] == 2'b11 && IPIPE_D[5:3] != 3'b001 && IPIPE_D[5:3] != 3'b111) begin
                OP_I = Scc;
            //
            end else if (IPIPE_D[8] == 1'b0 && IPIPE_D[7:6] < 2'b11 && IPIPE_D[5:3] == 3'b111 && IPIPE_D[2:0] < 3'b010) begin
                OP_I = ADDQ;
            end else if (IPIPE_D[8] == 1'b0 && (IPIPE_D[7:6] == 2'b01 || IPIPE_D[7:6] == 2'b10) && IPIPE_D[5:3] != 3'b111) begin
                OP_I = ADDQ;
            end else if (IPIPE_D[8] == 1'b0 && IPIPE_D[7:6] == 2'b00 && IPIPE_D[5:3] != 3'b001 && IPIPE_D[5:3] != 3'b111) begin
                OP_I = ADDQ;
            //
            end else if (IPIPE_D[8] == 1'b1 && IPIPE_D[7:6] < 2'b11 && IPIPE_D[5:3] == 3'b111 && IPIPE_D[2:0] < 3'b010) begin
                OP_I = SUBQ;
            end else if (IPIPE_D[8] == 1'b1 && (IPIPE_D[7:6] == 2'b01 || IPIPE_D[7:6] == 2'b10) && IPIPE_D[5:3] != 3'b111) begin
                OP_I = SUBQ;
            end else if (IPIPE_D[8] == 1'b1 && IPIPE_D[7:6] == 2'b00 && IPIPE_D[5:3] != 3'b001 && IPIPE_D[5:3] != 3'b111) begin
                OP_I = SUBQ;
            //
            end else if (IPIPE_D[7:3] == 5'b11111) begin
                OP_I = TRAPcc;
            end
        end
        4'h6: begin // Bcc / BSR / BRA.
            if (IPIPE_D[11:8] == 4'h0) begin
                OP_I = BRA;
            end else if (IPIPE_D[11:8] == 4'h1) begin
                OP_I = BSR;
            end else begin
                OP_I = Bcc;
            end
        end
        4'h7: begin // MOVEQ.
            if (IPIPE_D[8] == 1'b0) begin
                OP_I = MOVEQ;
            end
        end
        4'h8: begin // OR / DIV / SBCD / PACK / UNPK.
            if (IPIPE_D[8:4] == 5'b10100) begin
                OP_I = PACK;
            end else if (IPIPE_D[8:4] == 5'b11000) begin
                OP_I = UNPK;
            end else if (IPIPE_D[8:6] == 3'b011 && IPIPE_D[5:3] == 3'b111 && IPIPE_D[2:0] < 3'b101) begin
                OP_I = DIVU; // WORD.
            end else if (IPIPE_D[8:6] == 3'b011 && IPIPE_D[5:3] != 3'b001 && IPIPE_D[5:3] != 3'b111) begin
                OP_I = DIVU; // WORD.
            end else if (IPIPE_D[8:6] == 3'b111 && IPIPE_D[5:3] == 3'b111 && IPIPE_D[2:0] < 3'b101) begin
                OP_I = DIVS; // WORD.
            end else if (IPIPE_D[8:6] == 3'b111 && IPIPE_D[5:3] != 3'b001 && IPIPE_D[5:3] != 3'b111) begin
                OP_I = DIVS; // WORD.
            end else if (IPIPE_D[8:4] == 5'b10000) begin
                OP_I = SBCD;
            end
            //
            case (IPIPE_D[8:6])
                3'b000, 3'b001, 3'b010: begin
                    if (IPIPE_D[5:3] == 3'b111 && IPIPE_D[2:0] < 3'b101) begin
                        OP_I = OR_B;
                    end else if (IPIPE_D[5:3] != 3'b001 && IPIPE_D[5:3] != 3'b111) begin
                        OP_I = OR_B;
                    end
                end
                3'b100, 3'b101, 3'b110: begin
                    if (IPIPE_D[5:3] == 3'b111 && IPIPE_D[2:0] < 3'b010) begin
                        OP_I = OR_B;
                    end else if (IPIPE_D[5:3] > 3'b001 && IPIPE_D[5:3] != 3'b111) begin
                        OP_I = OR_B;
                    end
                end
                default: ;
            endcase
        end
        4'h9: begin // SUB / SUBX.
            case (IPIPE_D[8:6])
                3'b000: begin // Byte size.
                    if (IPIPE_D[5:3] == 3'b111 && IPIPE_D[2:0] < 3'b101) begin
                        OP_I = SUB;
                    end else if (IPIPE_D[5:3] != 3'b111 && IPIPE_D[5:3] != 3'b001) begin
                        OP_I = SUB;
                    end
                end
                3'b001, 3'b010: begin // Word and long.
                    if (IPIPE_D[5:3] == 3'b111 && IPIPE_D[2:0] < 3'b101) begin
                        OP_I = SUB;
                    end else if (IPIPE_D[5:3] != 3'b111) begin
                        OP_I = SUB;
                    end
                end
                3'b100: begin
                    if (IPIPE_D[5:3] == 3'b000 || IPIPE_D[5:3] == 3'b001) begin
                        OP_I = SUBX;
                    end else if (IPIPE_D[5:3] == 3'b111 && IPIPE_D[2:0] < 3'b010) begin
                        OP_I = SUB;
                    end else if (IPIPE_D[5:3] != 3'b111 && IPIPE_D[5:3] != 3'b001) begin // Byte size.
                        OP_I = SUB;
                    end
                end
                3'b101, 3'b110: begin
                    if (IPIPE_D[5:3] == 3'b000 || IPIPE_D[5:3] == 3'b001) begin
                        OP_I = SUBX;
                    end else if (IPIPE_D[5:3] == 3'b111 && IPIPE_D[2:0] < 3'b010) begin
                        OP_I = SUB;
                    end else if (IPIPE_D[5:3] != 3'b111) begin // Word and long.
                        OP_I = SUB;
                    end
                end
                3'b011, 3'b111: begin
                    if (IPIPE_D[5:3] == 3'b111 && IPIPE_D[2:0] < 3'b101) begin
                        OP_I = SUBA;
                    end else if (IPIPE_D[5:3] != 3'b111) begin
                        OP_I = SUBA;
                    end
                end
                default: ;
            endcase
        end
        4'hA: begin // (1010, Unassigned, Reserved).
            OP_I = UNIMPLEMENTED;
        end
        4'hB: begin // CMP / EOR.
            if (IPIPE_D[8] == 1'b1 && IPIPE_D[7:6] < 2'b11 && IPIPE_D[5:3] == 3'b001) begin
                OP_I = CMPM;
            end else begin
                case (IPIPE_D[8:6]) // OPMODE field.
                    3'b000: begin
                        if (IPIPE_D[5:3] == 3'b111 && IPIPE_D[2:0] < 3'b101) begin
                            OP_I = CMP;
                        end else if (IPIPE_D[5:3] != 3'b001 && IPIPE_D[5:3] != 3'b111) begin
                            OP_I = CMP;
                        end
                    end
                    3'b001, 3'b010: begin
                        if (IPIPE_D[5:3] == 3'b111 && IPIPE_D[2:0] < 3'b101) begin
                            OP_I = CMP;
                        end else if (IPIPE_D[5:3] != 3'b111) begin
                            OP_I = CMP;
                        end
                    end
                    3'b011, 3'b111: begin
                        if (IPIPE_D[5:3] == 3'b111 && IPIPE_D[2:0] < 3'b101) begin
                            OP_I = CMPA;
                        end else if (IPIPE_D[5:3] != 3'b111) begin
                            OP_I = CMPA;
                        end
                    end
                    3'b100, 3'b101, 3'b110: begin
                        if (IPIPE_D[5:3] == 3'b111 && IPIPE_D[2:0] < 3'b101) begin
                            OP_I = EOR;
                        end else if (IPIPE_D[5:3] != 3'b001 && IPIPE_D[5:3] != 3'b111) begin
                            OP_I = EOR;
                        end
                    end
                    default: ;
                endcase
            end
        end
        4'hC: begin // AND / MUL / ABCD / EXG.
            if (IPIPE_D[8:4] == 5'b10000) begin
                OP_I = ABCD;
            end else if (IPIPE_D[8:6] == 3'b011 && IPIPE_D[5:3] == 3'b111 && IPIPE_D[2:0] < 3'b101) begin
                OP_I = MULU; // WORD.
            end else if (IPIPE_D[8:6] == 3'b011 && IPIPE_D[5:3] != 3'b001 && IPIPE_D[5:3] != 3'b111) begin
                OP_I = MULU; // WORD.
            end else if (IPIPE_D[8:6] == 3'b111 && IPIPE_D[5:3] == 3'b111 && IPIPE_D[2:0] < 3'b101) begin
                OP_I = MULS; // WORD.
            end else if (IPIPE_D[8:6] == 3'b111 && IPIPE_D[5:3] != 3'b001 && IPIPE_D[5:3] != 3'b111) begin
                OP_I = MULS; // WORD.
            end else if (IPIPE_D[8:3] == 6'b101000 || IPIPE_D[8:3] == 6'b101001 || IPIPE_D[8:3] == 6'b110001) begin
                OP_I = EXG;
            end else begin
                case (IPIPE_D[8:6]) // OPMODE
                    3'b000, 3'b001, 3'b010: begin
                        if (IPIPE_D[5:3] == 3'b111 && IPIPE_D[2:0] < 3'b101) begin
                            OP_I = AND_B;
                        end else if (IPIPE_D[5:3] != 3'b001 && IPIPE_D[5:3] != 3'b111) begin
                            OP_I = AND_B;
                        end
                    end
                    3'b100, 3'b101, 3'b110: begin
                        if (IPIPE_D[5:3] == 3'b111 && IPIPE_D[2:0] < 3'b010) begin
                            OP_I = AND_B;
                        end else if (IPIPE_D[5:3] > 3'b001 && IPIPE_D[5:3] != 3'b111) begin
                            OP_I = AND_B;
                        end
                    end
                    default: ;
                endcase
            end
        end
        4'hD: begin // ADD / ADDX.
            case (IPIPE_D[8:6])
                3'b000: begin
                    if (IPIPE_D[5:3] == 3'b111 && IPIPE_D[2:0] < 3'b101) begin
                        OP_I = ADD;
                    end else if (IPIPE_D[5:3] != 3'b111 && IPIPE_D[5:3] != 3'b001) begin
                        OP_I = ADD;
                    end
                end
                3'b001, 3'b010: begin
                    if (IPIPE_D[5:3] == 3'b111 && IPIPE_D[2:0] < 3'b101) begin
                        OP_I = ADD;
                    end else if (IPIPE_D[5:3] != 3'b111) begin
                        OP_I = ADD;
                    end
                end
                3'b100: begin
                    if (IPIPE_D[5:3] == 3'b000 || IPIPE_D[5:3] == 3'b001) begin
                        OP_I = ADDX;
                    end else if (IPIPE_D[5:3] == 3'b111 && IPIPE_D[2:0] < 3'b010) begin
                        OP_I = ADD;
                    end else if (IPIPE_D[5:3] != 3'b111 && IPIPE_D[5:3] != 3'b001) begin
                        OP_I = ADD;
                    end
                end
                3'b101, 3'b110: begin
                    if (IPIPE_D[5:3] == 3'b000 || IPIPE_D[5:3] == 3'b001) begin
                        OP_I = ADDX;
                    end else if (IPIPE_D[5:3] == 3'b111 && IPIPE_D[2:0] < 3'b010) begin
                        OP_I = ADD;
                    end else if (IPIPE_D[5:3] != 3'b111) begin
                        OP_I = ADD;
                    end
                end
                3'b011, 3'b111: begin
                    if (IPIPE_D[5:3] == 3'b111 && IPIPE_D[2:0] < 3'b101) begin
                        OP_I = ADDA;
                    end else if (IPIPE_D[5:3] != 3'b111) begin
                        OP_I = ADDA;
                    end
                end
                default: ;
            endcase
        end
        4'hE: begin // Shift / Rotate / Bit Field.
            if (IPIPE_D[11:6] == 6'b101011 && (IPIPE_D[5:3] == 3'b000 || IPIPE_D[5:3] == 3'b010)) begin
                OP_I = BFCHG;
            end else if (IPIPE_D[11:6] == 6'b101011 && (IPIPE_D[5:3] >= 3'b101 || IPIPE_D[5:3] <= 3'b110)) begin
                OP_I = BFCHG;
            end else if (IPIPE_D[11:6] == 6'b101011 && IPIPE_D[5:3] == 3'b111 && IPIPE_D[2:0] < 3'b010) begin
                OP_I = BFCHG;
            end else if (IPIPE_D[11:6] == 6'b110011 && (IPIPE_D[5:3] == 3'b000 || IPIPE_D[5:3] == 3'b010)) begin
                OP_I = BFCLR;
            end else if (IPIPE_D[11:6] == 6'b110011 && (IPIPE_D[5:3] >= 3'b101 || IPIPE_D[5:3] <= 3'b110)) begin
                OP_I = BFCLR;
            end else if (IPIPE_D[11:6] == 6'b110011 && IPIPE_D[5:3] == 3'b111 && IPIPE_D[2:0] < 3'b010) begin
                OP_I = BFCLR;
            end else if (IPIPE_D[11:6] == 6'b101111 && (IPIPE_D[5:3] == 3'b000 || IPIPE_D[5:3] == 3'b010)) begin
                OP_I = BFEXTS;
            end else if (IPIPE_D[11:6] == 6'b101111 && (IPIPE_D[5:3] >= 3'b101 || IPIPE_D[5:3] <= 3'b110)) begin
                OP_I = BFEXTS;
            end else if (IPIPE_D[11:6] == 6'b101111 && IPIPE_D[5:3] == 3'b111 && IPIPE_D[2:0] < 3'b100) begin
                OP_I = BFEXTS;
            end else if (IPIPE_D[11:6] == 6'b100111 && (IPIPE_D[5:3] == 3'b000 || IPIPE_D[5:3] == 3'b010)) begin
                OP_I = BFEXTU;
            end else if (IPIPE_D[11:6] == 6'b100111 && (IPIPE_D[5:3] >= 3'b101 || IPIPE_D[5:3] <= 3'b110)) begin
                OP_I = BFEXTU;
            end else if (IPIPE_D[11:6] == 6'b100111 && IPIPE_D[5:3] == 3'b111 && IPIPE_D[2:0] < 3'b100) begin
                OP_I = BFEXTU;
            end else if (IPIPE_D[11:6] == 6'b110111 && (IPIPE_D[5:3] == 3'b000 || IPIPE_D[5:3] == 3'b010)) begin
                OP_I = BFFFO;
            end else if (IPIPE_D[11:6] == 6'b110111 && (IPIPE_D[5:3] >= 3'b101 || IPIPE_D[5:3] <= 3'b110)) begin
                OP_I = BFFFO;
            end else if (IPIPE_D[11:6] == 6'b110111 && IPIPE_D[5:3] == 3'b111 && IPIPE_D[2:0] < 3'b100) begin
                OP_I = BFFFO;
            end else if (IPIPE_D[11:6] == 6'b111111 && (IPIPE_D[5:3] == 3'b000 || IPIPE_D[5:3] == 3'b010)) begin
                OP_I = BFINS;
            end else if (IPIPE_D[11:6] == 6'b111111 && (IPIPE_D[5:3] >= 3'b101 || IPIPE_D[5:3] <= 3'b110)) begin
                OP_I = BFINS;
            end else if (IPIPE_D[11:6] == 6'b111111 && IPIPE_D[5:3] == 3'b111 && IPIPE_D[2:0] < 3'b010) begin
                OP_I = BFINS;
            end else if (IPIPE_D[11:6] == 6'b111011 && (IPIPE_D[5:3] == 3'b000 || IPIPE_D[5:3] == 3'b010)) begin
                OP_I = BFSET;
            end else if (IPIPE_D[11:6] == 6'b111011 && (IPIPE_D[5:3] >= 3'b101 || IPIPE_D[5:3] <= 3'b110)) begin
                OP_I = BFSET;
            end else if (IPIPE_D[11:6] == 6'b111011 && IPIPE_D[5:3] == 3'b111 && IPIPE_D[2:0] < 3'b010) begin
                OP_I = BFSET;
            end else if (IPIPE_D[11:6] == 6'b100011 && (IPIPE_D[5:3] == 3'b000 || IPIPE_D[5:3] == 3'b010)) begin
                OP_I = BFTST;
            end else if (IPIPE_D[11:6] == 6'b100011 && (IPIPE_D[5:3] >= 3'b101 || IPIPE_D[5:3] <= 3'b110)) begin
                OP_I = BFTST;
            end else if (IPIPE_D[11:6] == 6'b100011 && IPIPE_D[5:3] == 3'b111 && IPIPE_D[2:0] < 3'b100) begin
                OP_I = BFTST;
            end else if (IPIPE_D[11:6] == 6'b000011 && IPIPE_D[5:3] == 3'b111 && IPIPE_D[2:0] < 3'b010) begin
                OP_I = ASR; // Memory shifts.
            end else if (IPIPE_D[11:6] == 6'b000011 && IPIPE_D[5:3] > 3'b001 && IPIPE_D[5:3] != 3'b111) begin
                OP_I = ASR; // Memory shifts.
            end else if (IPIPE_D[11:6] == 6'b000111 && IPIPE_D[5:3] == 3'b111 && IPIPE_D[2:0] < 3'b010) begin
                OP_I = ASL; // Memory shifts.
            end else if (IPIPE_D[11:6] == 6'b000111 && IPIPE_D[5:3] > 3'b001 && IPIPE_D[5:3] != 3'b111) begin
                OP_I = ASL; // Memory shifts.
            end else if (IPIPE_D[11:6] == 6'b001011 && IPIPE_D[5:3] == 3'b111 && IPIPE_D[2:0] < 3'b010) begin
                OP_I = LSR; // Memory shifts.
            end else if (IPIPE_D[11:6] == 6'b001011 && IPIPE_D[5:3] > 3'b001 && IPIPE_D[5:3] != 3'b111) begin
                OP_I = LSR; // Memory shifts.
            end else if (IPIPE_D[11:6] == 6'b001111 && IPIPE_D[5:3] == 3'b111 && IPIPE_D[2:0] < 3'b010) begin
                OP_I = LSL; // Memory shifts.
            end else if (IPIPE_D[11:6] == 6'b001111 && IPIPE_D[5:3] > 3'b001 && IPIPE_D[5:3] != 3'b111) begin
                OP_I = LSL; // Memory shifts.
            end else if (IPIPE_D[11:6] == 6'b010011 && IPIPE_D[5:3] == 3'b111 && IPIPE_D[2:0] < 3'b010) begin
                OP_I = ROXR; // Memory shifts.
            end else if (IPIPE_D[11:6] == 6'b010011 && IPIPE_D[5:3] > 3'b001 && IPIPE_D[5:3] != 3'b111) begin
                OP_I = ROXR; // Memory shifts.
            end else if (IPIPE_D[11:6] == 6'b010111 && IPIPE_D[5:3] == 3'b111 && IPIPE_D[2:0] < 3'b010) begin
                OP_I = ROXL; // Memory shifts.
            end else if (IPIPE_D[11:6] == 6'b010111 && IPIPE_D[5:3] > 3'b001 && IPIPE_D[5:3] != 3'b111) begin
                OP_I = ROXL; // Memory shifts.
            end else if (IPIPE_D[11:6] == 6'b011011 && IPIPE_D[5:3] == 3'b111 && IPIPE_D[2:0] < 3'b010) begin
                OP_I = ROTR; // Memory shifts.
            end else if (IPIPE_D[11:6] == 6'b011011 && IPIPE_D[5:3] > 3'b001 && IPIPE_D[5:3] != 3'b111) begin
                OP_I = ROTR; // Memory shifts.
            end else if (IPIPE_D[11:6] == 6'b011111 && IPIPE_D[5:3] == 3'b111 && IPIPE_D[2:0] < 3'b010) begin
                OP_I = ROTL; // Memory shifts.
            end else if (IPIPE_D[11:6] == 6'b011111 && IPIPE_D[5:3] > 3'b001 && IPIPE_D[5:3] != 3'b111) begin
                OP_I = ROTL; // Memory shifts.
            end else if (IPIPE_D[8] == 1'b0 && IPIPE_D[7:6] < 2'b11 && IPIPE_D[4:3] == 2'b00) begin
                OP_I = ASR; // Register shifts.
            end else if (IPIPE_D[8] == 1'b1 && IPIPE_D[7:6] < 2'b11 && IPIPE_D[4:3] == 2'b00) begin
                OP_I = ASL; // Register shifts.
            end else if (IPIPE_D[8] == 1'b0 && IPIPE_D[7:6] < 2'b11 && IPIPE_D[4:3] == 2'b01) begin
                OP_I = LSR; // Register shifts.
            end else if (IPIPE_D[8] == 1'b1 && IPIPE_D[7:6] < 2'b11 && IPIPE_D[4:3] == 2'b01) begin
                OP_I = LSL; // Register shifts.
            end else if (IPIPE_D[8] == 1'b0 && IPIPE_D[7:6] < 2'b11 && IPIPE_D[4:3] == 2'b10) begin
                OP_I = ROXR; // Register shifts.
            end else if (IPIPE_D[8] == 1'b1 && IPIPE_D[7:6] < 2'b11 && IPIPE_D[4:3] == 2'b10) begin
                OP_I = ROXL; // Register shifts.
            end else if (IPIPE_D[8] == 1'b0 && IPIPE_D[7:6] < 2'b11 && IPIPE_D[4:3] == 2'b11) begin
                OP_I = ROTR; // Register shifts.
            end else if (IPIPE_D[8] == 1'b1 && IPIPE_D[7:6] < 2'b11 && IPIPE_D[4:3] == 2'b11) begin
                OP_I = ROTL; // Register shifts.
            end
        end
        4'hF: begin // 1111, Coprocessor Interface / 68K40 Extensions.
            OP_I = UNIMPLEMENTED;
        end
        default: ;
    endcase
end

endmodule
