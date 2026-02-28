// ------------------------------------------------------------------------
// -- WF68K30L IP Core: Division State Machine                           --
// -- Extracted from wf68k30L_alu.sv                                     --
// -- Author(s): Wolfgang Foerster, wf@experiment-s.de                   --
// -- Copyright (c) 2014-2019 Wolfgang Foerster Inventronik GmbH.        --
// -- CERN OHL v. 1.2                                                    --
// --                                                                    --
// -- BUG-009 fix: On divide-by-zero, drive restored destination values   --
// -- on QUOTIENT/REMAINDER so architectural Dn is preserved.             --
// ------------------------------------------------------------------------

module WF68K30L_DIVIDER (
    input  logic        CLK,

    // Operation control
    input  logic [6:0]  OP,
    input  logic [6:0]  OP_IN,
    input  logic [1:0]  OP_SIZE,
    input  logic        ALU_INIT,

    // BIW_1 for 64-bit dividend control
    input  logic [15:0] BIW_1,

    // Operands
    input  logic [31:0] OP1,
    input  logic [31:0] OP2,
    input  logic [31:0] OP3,

    // Outputs
    output logic [31:0] QUOTIENT,
    output logic [31:0] REMAINDER,
    output logic        VFLAG_DIV,
    output logic        DIV_RDY
);

`include "wf68k30L_pkg.svh"

typedef enum logic [1:0] {DIV_IDLE, DIV_INIT, DIV_CALC} DIV_STATES;

DIV_STATES DIV_STATE;

always_ff @(posedge CLK) begin : division
    logic [6:0]  BITCNT;
    logic [63:0] DIVIDEND;
    logic [31:0] DIVISOR;
    logic [31:0] QUOTIENT_REST;
    logic [31:0] QUOTIENT_VAR;
    logic [31:0] REMAINDER_REST;
    logic [31:0] REMAINDER_VAR;
    logic        DIV_IS_SIGNED;
    logic        DIV_QUOT_NEG;
    logic        DIV_REM_NEG;
    logic        DIV_WORD_MODE;
    logic        DIV_OVERFLOW;
    logic [31:0] QUOTIENT_FINAL;
    logic [31:0] REMAINDER_FINAL;

    DIV_RDY <= 1'b0;
    case (DIV_STATE)
        DIV_IDLE: begin
            if (ALU_INIT && (OP_IN == DIVS || OP_IN == DIVU))
                DIV_STATE <= DIV_INIT;
        end
        DIV_INIT: begin
            // Compute absolute value of dividend
            if (OP == DIVS && OP_SIZE == LONG && BIW_1[10] && OP3[31])
                DIVIDEND = ~{OP3, OP2} + 1'b1; // 64-bit signed negative dividend
            else if ((OP == DIVS || OP == DIVU) && OP_SIZE == LONG && BIW_1[10])
                DIVIDEND = {OP3, OP2}; // 64-bit positive or unsigned dividend
            else if (OP == DIVS && OP2[31])
                DIVIDEND = {32'h0, ~OP2 + 1'b1}; // 32-bit signed negative dividend
            else
                DIVIDEND = {32'h0, OP2}; // 32-bit positive or unsigned dividend

            // Compute absolute value of divisor
            if (OP == DIVS && OP_SIZE == LONG && OP1[31])
                DIVISOR = ~OP1 + 1'b1; // 32-bit signed negative divisor
            else if (OP_SIZE == LONG)
                DIVISOR = OP1; // 32-bit positive or unsigned divisor
            else if (OP == DIVS && OP_SIZE == WORD && OP1[15])
                DIVISOR = {16'h0, ~OP1[15:0] + 1'b1}; // 16-bit signed negative divisor
            else
                DIVISOR = {16'h0, OP1[15:0]}; // 16-bit positive or unsigned divisor

            VFLAG_DIV <= 1'b0;
            QUOTIENT_VAR = 32'h0;
            QUOTIENT_REST = OP2;

            REMAINDER_VAR = 32'h0;

            case (OP_SIZE)
                LONG:    REMAINDER_REST = OP3;
                default: REMAINDER_REST = {16'h0, OP2[31:16]};
            endcase

            DIV_IS_SIGNED = (OP == DIVS);
            DIV_WORD_MODE = (OP_SIZE == WORD);
            if (OP_SIZE == WORD) begin
                DIV_QUOT_NEG = (OP == DIVS) && (OP2[31] ^ OP1[15]);
                DIV_REM_NEG = (OP == DIVS) && OP2[31];
            end else begin
                DIV_QUOT_NEG = (OP == DIVS) && ((BIW_1[10] ? OP3[31] : OP2[31]) ^ OP1[31]);
                DIV_REM_NEG = (OP == DIVS) && (BIW_1[10] ? OP3[31] : OP2[31]);
            end

            if (OP_SIZE == LONG && BIW_1[10])
                BITCNT = 7'd64;
            else
                BITCNT = 7'd32;

            if (DIVISOR == 32'h0) begin
                // Division by zero: present the unmodified destination value on the
                // divider outputs so downstream writeback paths preserve Dn.
                // The exception itself is still signaled by TRAP_DIVZERO in the ALU.
                QUOTIENT <= QUOTIENT_REST;
                REMAINDER <= REMAINDER_REST;
                DIV_STATE <= DIV_IDLE;
                DIV_RDY <= 1'b1;
            end else if ({32'h0, DIVISOR} > DIVIDEND) begin
                // Divisor > dividend: quotient=0, remainder=dividend
                QUOTIENT <= 32'h0;
                if (DIV_REM_NEG && DIVIDEND[31:0] != 32'h0)
                    REMAINDER <= ~DIVIDEND[31:0] + 1'b1;
                else
                    REMAINDER <= DIVIDEND[31:0];
                DIV_STATE <= DIV_IDLE;
                DIV_RDY <= 1'b1;
            end else if ({32'h0, DIVISOR} == DIVIDEND) begin
                // Result is 1
                if (DIV_IS_SIGNED && DIV_QUOT_NEG)
                    QUOTIENT <= 32'hFFFF_FFFF;
                else
                    QUOTIENT <= 32'h1;
                REMAINDER <= 32'h0;
                DIV_STATE <= DIV_IDLE;
                DIV_RDY <= 1'b1;
            end else begin
                QUOTIENT <= 32'h0;
                REMAINDER <= 32'h0;
                DIV_STATE <= DIV_CALC;
            end
        end
        DIV_CALC: begin
            BITCNT = BITCNT - 1;

            if ({REMAINDER_VAR, DIVIDEND[BITCNT]} < {1'b0, DIVISOR}) begin
                REMAINDER_VAR = {REMAINDER_VAR[30:0], DIVIDEND[BITCNT]};
            end else if (!DIV_IS_SIGNED && OP_SIZE == LONG && BITCNT > 31) begin
                // Division overflow in 64-bit mode
                VFLAG_DIV <= 1'b1;
                DIV_STATE <= DIV_IDLE;
                DIV_RDY <= 1'b1;
                QUOTIENT <= QUOTIENT_REST;
                REMAINDER <= REMAINDER_REST;
            end else if (!DIV_IS_SIGNED && OP_SIZE == WORD && BITCNT > 15) begin
                // Division overflow in word mode
                VFLAG_DIV <= 1'b1;
                DIV_STATE <= DIV_IDLE;
                DIV_RDY <= 1'b1;
                QUOTIENT <= QUOTIENT_REST;
                REMAINDER <= REMAINDER_REST;
            end else begin
                REMAINDER_VAR = {REMAINDER_VAR[30:0], DIVIDEND[BITCNT]} - DIVISOR;
                QUOTIENT_VAR[BITCNT] = 1'b1;
            end

            if (BITCNT == 7'd0) begin
                if (DIV_IS_SIGNED) begin
                    if (DIV_WORD_MODE) begin
                        if (DIV_QUOT_NEG)
                            DIV_OVERFLOW = (QUOTIENT_VAR > 32'h0000_8000);
                        else
                            DIV_OVERFLOW = (QUOTIENT_VAR > 32'h0000_7FFF);
                    end else begin
                        if (DIV_QUOT_NEG)
                            DIV_OVERFLOW = (QUOTIENT_VAR > 32'h8000_0000);
                        else
                            DIV_OVERFLOW = (QUOTIENT_VAR > 32'h7FFF_FFFF);
                    end

                    if (DIV_OVERFLOW) begin
                        VFLAG_DIV <= 1'b1;
                        DIV_STATE <= DIV_IDLE;
                        DIV_RDY <= 1'b1;
                        QUOTIENT <= QUOTIENT_REST;
                        REMAINDER <= REMAINDER_REST;
                    end else begin
                        if (DIV_QUOT_NEG)
                            QUOTIENT_FINAL = ~QUOTIENT_VAR + 1'b1;
                        else
                            QUOTIENT_FINAL = QUOTIENT_VAR;

                        if (DIV_REM_NEG && REMAINDER_VAR != 32'h0)
                            REMAINDER_FINAL = ~REMAINDER_VAR + 1'b1;
                        else
                            REMAINDER_FINAL = REMAINDER_VAR;

                        QUOTIENT <= QUOTIENT_FINAL;
                        REMAINDER <= REMAINDER_FINAL;
                        DIV_RDY <= 1'b1;
                        DIV_STATE <= DIV_IDLE;
                    end
                end else begin
                    QUOTIENT <= QUOTIENT_VAR;
                    REMAINDER <= REMAINDER_VAR;
                    DIV_RDY <= 1'b1;
                    DIV_STATE <= DIV_IDLE;
                end
            end
        end
    endcase
end

endmodule
