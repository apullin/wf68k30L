(* keep_hierarchy = "yes" *)
module WF68K30L_TOP_DESC_SHADOW_PORT #(
    parameter int MMU_DESC_SHADOW_LINES = 64,
    parameter int MMU_DESC_SHADOW_WAYS = 1,
    parameter int MMU_DESC_SHADOW_SETS = MMU_DESC_SHADOW_LINES / MMU_DESC_SHADOW_WAYS,
    parameter int MMU_DESC_SHADOW_SET_BITS = $clog2(MMU_DESC_SHADOW_SETS),
    parameter int MMU_DESC_SHADOW_WAY_BITS = (MMU_DESC_SHADOW_WAYS > 1) ? $clog2(MMU_DESC_SHADOW_WAYS) : 1
) (
    input  logic        CLK,
    input  logic        RESET_CPU,
    input  logic        RD_EN,
    input  logic [31:0] RD_ADDR,
    output logic [32:0] RD_LOOKUP,
    input  logic        WR_EN,
    input  logic [31:0] WR_ADDR,
    input  logic [31:0] WR_DATA
);

localparam int SHADOW_WORD_WIDTH = 64;

logic [MMU_DESC_SHADOW_WAYS-1:0] MMU_DESC_SHADOW_V [0:MMU_DESC_SHADOW_SETS-1];
logic [MMU_DESC_SHADOW_WAY_BITS-1:0] MMU_DESC_SHADOW_REPL_PTR [0:MMU_DESC_SHADOW_SETS-1];
logic [SHADOW_WORD_WIDTH-1:0] SHADOW_RD_DATA [0:MMU_DESC_SHADOW_WAYS-1];
logic [MMU_DESC_SHADOW_WAYS-1:0] SHADOW_WR_EN;
logic [MMU_DESC_SHADOW_SET_BITS-1:0] rd_set_idx;
logic [MMU_DESC_SHADOW_SET_BITS-1:0] wr_set_idx;
logic [31:0] rd_addr_aligned;
logic [31:0] wr_addr_aligned;
logic [31:0] rd_addr_r;
logic [MMU_DESC_SHADOW_SET_BITS-1:0] rd_set_idx_r;
logic        rd_pending_r;
logic [MMU_DESC_SHADOW_WAY_BITS-1:0] wr_way_sel;
logic [32:0] rd_lookup_next;

function automatic logic [MMU_DESC_SHADOW_SET_BITS-1:0] mmu_desc_shadow_set_idx(input logic [31:0] addr);
    logic [31:0] addr_w;
begin
    addr_w = {addr[31:2], 2'b00};
    mmu_desc_shadow_set_idx = addr_w[MMU_DESC_SHADOW_SET_BITS+1:2];
end
endfunction

assign rd_addr_aligned = {RD_ADDR[31:2], 2'b00};
assign wr_addr_aligned = {WR_ADDR[31:2], 2'b00};
assign rd_set_idx = mmu_desc_shadow_set_idx(rd_addr_aligned);
assign wr_set_idx = mmu_desc_shadow_set_idx(wr_addr_aligned);
assign wr_way_sel = (MMU_DESC_SHADOW_WAYS == 1) ? '0 : MMU_DESC_SHADOW_REPL_PTR[wr_set_idx];
assign RD_LOOKUP = rd_lookup_next;

for (genvar way_i = 0; way_i < MMU_DESC_SHADOW_WAYS; way_i = way_i + 1) begin : gen_shadow_way_ram
    assign SHADOW_WR_EN[way_i] = WR_EN && (wr_way_sel == way_i[MMU_DESC_SHADOW_WAY_BITS-1:0]);

    WF68K30L_SYNC_RAM_1R1W #(
        .WIDTH(SHADOW_WORD_WIDTH),
        .DEPTH(MMU_DESC_SHADOW_SETS),
        .ADDR_BITS(MMU_DESC_SHADOW_SET_BITS)
    ) I_SHADOW_RAM (
        .CLK(CLK),
        .RD_EN(RD_EN),
        .RD_ADDR(rd_set_idx),
        .RD_DATA(SHADOW_RD_DATA[way_i]),
        .WR_EN(SHADOW_WR_EN[way_i]),
        .WR_ADDR(wr_set_idx),
        .WR_DATA({wr_addr_aligned, WR_DATA})
    );
end

always_comb begin : shadow_lookup_comb
    rd_lookup_next = 33'h0;
    if (rd_pending_r) begin
        for (int way_i = 0; way_i < MMU_DESC_SHADOW_WAYS; way_i = way_i + 1) begin
            if (!rd_lookup_next[32] &&
                MMU_DESC_SHADOW_V[rd_set_idx_r][way_i] &&
                (SHADOW_RD_DATA[way_i][63:32] == rd_addr_r)) begin
                rd_lookup_next = {1'b1, SHADOW_RD_DATA[way_i][31:0]};
            end
        end
    end
end

always_ff @(posedge CLK) begin : shadow_state
    integer set_i;
    if (RESET_CPU) begin
        rd_pending_r <= 1'b0;
        rd_set_idx_r <= '0;
        rd_addr_r <= 32'h0000_0000;
        for (set_i = 0; set_i < MMU_DESC_SHADOW_SETS; set_i = set_i + 1) begin
            MMU_DESC_SHADOW_V[set_i] <= '0;
            MMU_DESC_SHADOW_REPL_PTR[set_i] <= '0;
        end
    end else begin
        rd_pending_r <= RD_EN;
        if (RD_EN) begin
            rd_set_idx_r <= rd_set_idx;
            rd_addr_r <= rd_addr_aligned;
        end

        if (WR_EN) begin
            MMU_DESC_SHADOW_V[wr_set_idx][wr_way_sel] <= 1'b1;
            if (MMU_DESC_SHADOW_WAYS > 1)
                MMU_DESC_SHADOW_REPL_PTR[wr_set_idx] <= MMU_DESC_SHADOW_REPL_PTR[wr_set_idx] + 1'b1;
        end
    end
end

endmodule
