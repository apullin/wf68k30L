// ------------------------------------------------------------------------
// --                                                                    --
// -- WF68K30L IP Core: Data register logic.                             --
// --                                                                    --
// -- Description:                                                       --
// -- These are the eight data registers. The logic provides two         --
// -- read and two write ports providing simultaneous access. For        --
// -- more information refer to the MC68030 User Manual.                 --
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
//

module WF68K30L_DATA_REGISTERS (
    input  logic        CLK,
    input  logic        RESET,

    // Data lines:
    input  logic [31:0] DR_IN_1,
    input  logic [31:0] DR_IN_2,
    output logic [31:0] DR_OUT_1,
    output logic [31:0] DR_OUT_2,

    // Registers controls:
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

logic [31:0] DR [0:7]; // Data registers D0 to D7.
integer DR_PNTR_WR_1;
integer DR_PNTR_WR_2;
integer DR_PNTR_RD_1;
integer DR_PNTR_RD_2;
logic [2:0] DR_SEL_WR_I1;
logic [2:0] DR_SEL_WR_I2;
logic [3:0] DR_USED_1;
logic [3:0] DR_USED_2;

always_ff @(posedge CLK) begin : INBUFFER
    if (DR_MARK_USED == 1'b1) begin
        DR_SEL_WR_I1 <= DR_SEL_WR_1;
        DR_SEL_WR_I2 <= DR_SEL_WR_2;
    end
end

assign DR_PNTR_WR_1 = DR_SEL_WR_I1;
assign DR_PNTR_WR_2 = DR_SEL_WR_I2;
assign DR_PNTR_RD_1 = DR_SEL_RD_1;
assign DR_PNTR_RD_2 = DR_SEL_RD_2;

always_ff @(posedge CLK) begin : P_IN_USE
    if (RESET == 1'b1 || UNMARK == 1'b1) begin
        DR_USED_1[3] <= 1'b0;
        DR_USED_2[3] <= 1'b0;
    end else if (DR_MARK_USED == 1'b1) begin
        DR_USED_1 <= {1'b1, DR_SEL_WR_1};
        if (USE_DPAIR == 1'b1) begin
            DR_USED_2 <= {1'b1, DR_SEL_WR_2};
        end
    end
end

assign DR_IN_USE = (DR_USED_1[3] == 1'b1 && DR_USED_1[2:0] == DR_SEL_RD_1) ? 1'b1 :
                   (DR_USED_1[3] == 1'b1 && DR_USED_1[2:0] == DR_SEL_RD_2) ? 1'b1 :
                   (DR_USED_2[3] == 1'b1 && DR_USED_2[2:0] == DR_SEL_RD_1) ? 1'b1 :
                   (DR_USED_2[3] == 1'b1 && DR_USED_2[2:0] == DR_SEL_RD_2) ? 1'b1 : 1'b0;

assign DR_OUT_1 = DR[DR_PNTR_RD_1];
assign DR_OUT_2 = DR[DR_PNTR_RD_2];

always_ff @(posedge CLK) begin : REGISTERS
    if (RESET == 1'b1) begin
        DR[0] <= 32'h00000000;
        DR[1] <= 32'h00000000;
        DR[2] <= 32'h00000000;
        DR[3] <= 32'h00000000;
        DR[4] <= 32'h00000000;
        DR[5] <= 32'h00000000;
        DR[6] <= 32'h00000000;
        DR[7] <= 32'h00000000;
    end
    if (DR_WR_1 == 1'b1) begin
        case (OP_SIZE)
            LONG: DR[DR_PNTR_WR_1] <= DR_IN_1;
            WORD: DR[DR_PNTR_WR_1][15:0] <= DR_IN_1[15:0];
            BYTE: DR[DR_PNTR_WR_1][7:0] <= DR_IN_1[7:0];
            default: ;
        endcase
    end
    if (DR_WR_2 == 1'b1) begin
        case (OP_SIZE)
            LONG: DR[DR_PNTR_WR_2] <= DR_IN_2;
            WORD: DR[DR_PNTR_WR_2][15:0] <= DR_IN_2[15:0];
            BYTE: DR[DR_PNTR_WR_2][7:0] <= DR_IN_2[7:0];
            default: ;
        endcase
    end
end

endmodule
