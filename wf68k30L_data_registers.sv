// ------------------------------------------------------------------------
// WF68K30L IP Core: Data register logic.
//
// Description:
// These are the eight data registers. The logic provides two
// read and two write ports providing simultaneous access. For
// more information refer to the MC68030 User Manual.
//
// Author(s):
// - Wolfgang Foerster, wf@experiment-s.de; wf@inventronik.de
//
// Copyright (c) 2014-2019 Wolfgang Foerster Inventronik GmbH.
// CERN OHL v. 1.2
// ------------------------------------------------------------------------

module WF68K30L_DATA_REGISTERS (
    input  logic        CLK,
    input  logic        RESET,

    // Data lines:
    input  logic [31:0] DR_IN_1,
    input  logic [31:0] DR_IN_2,
    output logic [31:0] DR_OUT_1,
    output logic [31:0] DR_OUT_2,

    // Register controls:
    input  logic [2:0]  DR_SEL_WR_1,
    input  logic [2:0]  DR_SEL_WR_2,
    input  logic [2:0]  DR_SEL_RD_1,
    input  logic [2:0]  DR_SEL_RD_2,
    input  logic        DR_WR_1,
    input  logic        DR_WR_2,
    input  logic        DR_MARK_USED,
    input  logic        USE_DPAIR,
    output logic        DR_IN_USE,
    input  logic        UNMARK,

    input  logic [1:0]  OP_SIZE
);

`include "wf68k30L_pkg.svh"

// ---- Internal signals ----

logic [31:0] DR [0:7];          // Data registers D0-D7
logic [2:0]  dr_sel_wr_lat1;    // Latched write select port 1
logic [2:0]  dr_sel_wr_lat2;    // Latched write select port 2
logic [3:0]  dr_used_1;         // {valid, reg_num} for hazard tracking port 1
logic [3:0]  dr_used_2;         // {valid, reg_num} for hazard tracking port 2

// ---- Write select latch ----
// Captures write register selection when DR_MARK_USED is asserted.

always_ff @(posedge CLK) begin : latch_write_sel
    if (DR_MARK_USED) begin
        dr_sel_wr_lat1 <= DR_SEL_WR_1;
        dr_sel_wr_lat2 <= DR_SEL_WR_2;
    end
end

// ---- Hazard detection ----
// Tracks which registers are "in use" (pending writeback) and flags
// a hazard if a read port selects a register that is still in flight.

always_ff @(posedge CLK) begin : track_in_use
    if (RESET || UNMARK) begin
        dr_used_1[3] <= 1'b0;
        dr_used_2[3] <= 1'b0;
    end else if (DR_MARK_USED) begin
        dr_used_1 <= {1'b1, DR_SEL_WR_1};
        if (USE_DPAIR)
            dr_used_2 <= {1'b1, DR_SEL_WR_2};
    end
end

assign DR_IN_USE = (dr_used_1[3] && dr_used_1[2:0] == DR_SEL_RD_1) ||
                   (dr_used_1[3] && dr_used_1[2:0] == DR_SEL_RD_2) ||
                   (dr_used_2[3] && dr_used_2[2:0] == DR_SEL_RD_1) ||
                   (dr_used_2[3] && dr_used_2[2:0] == DR_SEL_RD_2);

// ---- Read ports (combinational) ----

assign DR_OUT_1 = DR[DR_SEL_RD_1];
assign DR_OUT_2 = DR[DR_SEL_RD_2];

// ---- Register file ----
// Supports LONG (32-bit), WORD (lower 16-bit), and BYTE (lower 8-bit) writes.
// Partial writes preserve the upper bits.

always_ff @(posedge CLK) begin : reg_file
    if (RESET) begin
        for (int i = 0; i < 8; i++)
            DR[i] <= 32'h0;
    end

    if (DR_WR_1) begin
        case (OP_SIZE)
            LONG:    DR[dr_sel_wr_lat1]       <= DR_IN_1;
            WORD:    DR[dr_sel_wr_lat1][15:0]  <= DR_IN_1[15:0];
            BYTE:    DR[dr_sel_wr_lat1][7:0]   <= DR_IN_1[7:0];
            default: ;
        endcase
    end

    if (DR_WR_2) begin
        case (OP_SIZE)
            LONG:    DR[dr_sel_wr_lat2]       <= DR_IN_2;
            WORD:    DR[dr_sel_wr_lat2][15:0]  <= DR_IN_2[15:0];
            BYTE:    DR[dr_sel_wr_lat2][7:0]   <= DR_IN_2[7:0];
            default: ;
        endcase
    end
end

endmodule
