// ------------------------------------------------------------------------
// -- WF68K30L IP Core: Shifter/Rotator Unit                             --
// -- Extracted from wf68k30L_alu.sv                                     --
// -- Author(s): Wolfgang Foerster, wf@experiment-s.de                   --
// -- Copyright (c) 2014-2019 Wolfgang Foerster Inventronik GmbH.        --
// -- CERN OHL v. 1.2                                                    --
// ------------------------------------------------------------------------

module WF68K30L_SHIFTER (
    input  logic        CLK,

    // Operation control (latched in ALU parameter_buffer)
    input  logic [6:0]  OP,
    input  logic [1:0]  OP_SIZE,

    // Shift width: pre-latch value for FSM init, latched value for flags
    input  logic [5:0]  SHIFT_WIDTH_IN,   // un-latched, used by FSM on SHFT_LOAD cycle
    input  logic [5:0]  SHIFT_WIDTH,      // latched, used by flag logic

    // Load strobe and data input
    input  logic        SHFT_LOAD,
    input  logic [31:0] DATA_IN,

    // Status register X flag for ROXL/ROXR and initialization
    input  logic        SR_X_FLAG,

    // Outputs
    output logic [31:0] RESULT_SHIFTOP,
    output logic        SHFT_RDY,

    // Flag outputs
    output logic        XFLAG_SHFT,
    output logic        CFLAG_SHFT,
    output logic        VFLAG_SHFT
);

`include "wf68k30L_pkg.svh"

typedef enum logic {SHIFT_IDLE, SHIFT_RUN} SHIFT_STATES;

SHIFT_STATES SHIFT_STATE;
logic        SHFT_EN;

// -- Shift control FSM --
always_ff @(posedge CLK) begin : shift_ctrl
    logic [5:0] BIT_CNT;

    SHFT_RDY <= 1'b0;

    if (SHIFT_STATE == SHIFT_IDLE) begin
        if (SHFT_LOAD && SHIFT_WIDTH_IN == 6'd0) begin
            SHFT_RDY <= 1'b1;
        end else if (SHFT_LOAD) begin
            SHIFT_STATE <= SHIFT_RUN;
            BIT_CNT = SHIFT_WIDTH_IN;
            SHFT_EN <= 1'b1;
        end else begin
            SHIFT_STATE <= SHIFT_IDLE;
            BIT_CNT = 6'd0;
            SHFT_EN <= 1'b0;
        end
    end else begin // SHIFT_RUN
        if (BIT_CNT == 6'd1) begin
            SHIFT_STATE <= SHIFT_IDLE;
            SHFT_EN <= 1'b0;
            SHFT_RDY <= 1'b1;
        end else begin
            SHIFT_STATE <= SHIFT_RUN;
            BIT_CNT = BIT_CNT - 1'b1;
            SHFT_EN <= 1'b1;
        end
    end
end

// -- Shifter data path --
always_ff @(posedge CLK) begin : shifter_datapath
    if (SHFT_LOAD) begin
        RESULT_SHIFTOP <= DATA_IN;
    end else if (SHFT_EN) begin
        case (OP)
            ASL: begin
                if (OP_SIZE == LONG)
                    RESULT_SHIFTOP <= {RESULT_SHIFTOP[30:0], 1'b0};
                else if (OP_SIZE == WORD)
                    RESULT_SHIFTOP <= {16'h0, RESULT_SHIFTOP[14:0], 1'b0};
                else
                    RESULT_SHIFTOP <= {24'h0, RESULT_SHIFTOP[6:0], 1'b0};
            end
            ASR: begin
                if (OP_SIZE == LONG)
                    RESULT_SHIFTOP <= {RESULT_SHIFTOP[31], RESULT_SHIFTOP[31:1]};
                else if (OP_SIZE == WORD)
                    RESULT_SHIFTOP <= {16'h0, RESULT_SHIFTOP[15], RESULT_SHIFTOP[15:1]};
                else
                    RESULT_SHIFTOP <= {24'h0, RESULT_SHIFTOP[7], RESULT_SHIFTOP[7:1]};
            end
            LSL: begin
                if (OP_SIZE == LONG)
                    RESULT_SHIFTOP <= {RESULT_SHIFTOP[30:0], 1'b0};
                else if (OP_SIZE == WORD)
                    RESULT_SHIFTOP <= {16'h0, RESULT_SHIFTOP[14:0], 1'b0};
                else
                    RESULT_SHIFTOP <= {24'h0, RESULT_SHIFTOP[6:0], 1'b0};
            end
            LSR: begin
                if (OP_SIZE == LONG)
                    RESULT_SHIFTOP <= {1'b0, RESULT_SHIFTOP[31:1]};
                else if (OP_SIZE == WORD)
                    RESULT_SHIFTOP <= {16'h0, 1'b0, RESULT_SHIFTOP[15:1]};
                else
                    RESULT_SHIFTOP <= {24'h0, 1'b0, RESULT_SHIFTOP[7:1]};
            end
            ROTL: begin
                if (OP_SIZE == LONG)
                    RESULT_SHIFTOP <= {RESULT_SHIFTOP[30:0], RESULT_SHIFTOP[31]};
                else if (OP_SIZE == WORD)
                    RESULT_SHIFTOP <= {16'h0, RESULT_SHIFTOP[14:0], RESULT_SHIFTOP[15]};
                else
                    RESULT_SHIFTOP <= {24'h0, RESULT_SHIFTOP[6:0], RESULT_SHIFTOP[7]};
            end
            ROTR: begin
                if (OP_SIZE == LONG)
                    RESULT_SHIFTOP <= {RESULT_SHIFTOP[0], RESULT_SHIFTOP[31:1]};
                else if (OP_SIZE == WORD)
                    RESULT_SHIFTOP <= {16'h0, RESULT_SHIFTOP[0], RESULT_SHIFTOP[15:1]};
                else
                    RESULT_SHIFTOP <= {24'h0, RESULT_SHIFTOP[0], RESULT_SHIFTOP[7:1]};
            end
            ROXL: begin
                if (OP_SIZE == LONG)
                    RESULT_SHIFTOP <= {RESULT_SHIFTOP[30:0], XFLAG_SHFT};
                else if (OP_SIZE == WORD)
                    RESULT_SHIFTOP <= {16'h0, RESULT_SHIFTOP[14:0], XFLAG_SHFT};
                else
                    RESULT_SHIFTOP <= {24'h0, RESULT_SHIFTOP[6:0], XFLAG_SHFT};
            end
            ROXR: begin
                if (OP_SIZE == LONG)
                    RESULT_SHIFTOP <= {XFLAG_SHFT, RESULT_SHIFTOP[31:1]};
                else if (OP_SIZE == WORD)
                    RESULT_SHIFTOP <= {16'h0, XFLAG_SHFT, RESULT_SHIFTOP[15:1]};
                else
                    RESULT_SHIFTOP <= {24'h0, XFLAG_SHFT, RESULT_SHIFTOP[7:1]};
            end
            default: ;
        endcase
    end
end

// -- Shifter flags (X, C, V) --
always_ff @(posedge CLK) begin : shifter_flags
    // X flag
    if (SHFT_LOAD || SHIFT_WIDTH == 6'd0) begin
        XFLAG_SHFT <= SR_X_FLAG;
    end else if (SHFT_EN) begin
        case (OP)
            ROTL, ROTR:
                XFLAG_SHFT <= SR_X_FLAG; // Unaffected.
            ASL, LSL, ROXL:
                case (OP_SIZE)
                    LONG:    XFLAG_SHFT <= RESULT_SHIFTOP[31];
                    WORD:    XFLAG_SHFT <= RESULT_SHIFTOP[15];
                    BYTE:    XFLAG_SHFT <= RESULT_SHIFTOP[7];
                    default: ;
                endcase
            default: // ASR, LSR, ROXR.
                XFLAG_SHFT <= RESULT_SHIFTOP[0];
        endcase
    end

    // C flag
    if ((OP == ROXL || OP == ROXR) && SHIFT_WIDTH == 6'd0)
        CFLAG_SHFT <= SR_X_FLAG;
    else if (SHIFT_WIDTH == 6'd0)
        CFLAG_SHFT <= 1'b0;
    else if (SHFT_EN) begin
        case (OP)
            ASL, LSL, ROTL, ROXL:
                case (OP_SIZE)
                    LONG:    CFLAG_SHFT <= RESULT_SHIFTOP[31];
                    WORD:    CFLAG_SHFT <= RESULT_SHIFTOP[15];
                    BYTE:    CFLAG_SHFT <= RESULT_SHIFTOP[7];
                    default: ;
                endcase
            default: // ASR, LSR, ROTR, ROXR
                CFLAG_SHFT <= RESULT_SHIFTOP[0];
        endcase
    end

    // V flag (only for ASL)
    if (SHFT_LOAD || SHIFT_WIDTH == 6'd0)
        VFLAG_SHFT <= 1'b0;
    else if (SHFT_EN) begin
        case (OP)
            ASL: begin
                if (OP_SIZE == LONG)
                    VFLAG_SHFT <= (RESULT_SHIFTOP[31] ^ RESULT_SHIFTOP[30]) | VFLAG_SHFT;
                else if (OP_SIZE == WORD)
                    VFLAG_SHFT <= (RESULT_SHIFTOP[15] ^ RESULT_SHIFTOP[14]) | VFLAG_SHFT;
                else
                    VFLAG_SHFT <= (RESULT_SHIFTOP[7] ^ RESULT_SHIFTOP[6]) | VFLAG_SHFT;
            end
            default:
                VFLAG_SHFT <= 1'b0;
        endcase
    end
end

endmodule
