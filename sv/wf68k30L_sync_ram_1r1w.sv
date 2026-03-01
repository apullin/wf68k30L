// ====================================================================
// WF68K30L_SYNC_RAM_1R1W
// --------------------------------------------------------------------
// Portable inference-first synchronous single-read/single-write RAM.
// - One registered read port.
// - One write port.
// - Read and write may occur in the same cycle.
// ====================================================================

module WF68K30L_SYNC_RAM_1R1W #(
    parameter int WIDTH = 8,
    parameter int DEPTH = 32,
    parameter int ADDR_BITS = $clog2(DEPTH)
) (
    input  logic                 CLK,
    input  logic                 RD_EN,
    input  logic [ADDR_BITS-1:0] RD_ADDR,
    output logic [WIDTH-1:0]     RD_DATA,
    input  logic                 WR_EN,
    input  logic [ADDR_BITS-1:0] WR_ADDR,
    input  logic [WIDTH-1:0]     WR_DATA
);

    (* ram_style = "block" *) logic [WIDTH-1:0] mem [0:DEPTH-1];

    always_ff @(posedge CLK) begin
        if (WR_EN)
            mem[WR_ADDR] <= WR_DATA;
        if (RD_EN)
            RD_DATA <= mem[RD_ADDR];
    end

endmodule

