(* keep_hierarchy = "yes" *)
module WF68K30L_TOP_DESC_SHADOW_LOOKUP #(
    parameter int MMU_DESC_SHADOW_LINES = 64,
    parameter int MMU_DESC_SHADOW_WAYS = 1,
    parameter int MMU_DESC_SHADOW_SETS = MMU_DESC_SHADOW_LINES / MMU_DESC_SHADOW_WAYS,
    parameter int MMU_DESC_SHADOW_SET_BITS = $clog2(MMU_DESC_SHADOW_SETS)
) (
    input  logic [31:0] ADDR,
    input  logic [MMU_DESC_SHADOW_SETS*MMU_DESC_SHADOW_WAYS-1:0] MMU_DESC_SHADOW_V_FLAT,
    input  logic [MMU_DESC_SHADOW_SETS*MMU_DESC_SHADOW_WAYS*32-1:0] MMU_DESC_SHADOW_ADDR_FLAT,
    input  logic [MMU_DESC_SHADOW_SETS*MMU_DESC_SHADOW_WAYS*32-1:0] MMU_DESC_SHADOW_DATA_FLAT,
    output logic [32:0] LOOKUP
);

function automatic logic [MMU_DESC_SHADOW_SET_BITS-1:0] mmu_desc_shadow_set_idx(input logic [31:0] addr);
    logic [31:0] addr_w;
begin
    addr_w = {addr[31:2], 2'b00};
    mmu_desc_shadow_set_idx = addr_w[MMU_DESC_SHADOW_SET_BITS+1:2];
end
endfunction

always_comb begin : shadow_lookup
    logic [31:0] addr_w;
    logic [MMU_DESC_SHADOW_SET_BITS-1:0] set_idx;
    logic hit;
    logic [31:0] data;
    integer flat_idx;
    integer way;

    addr_w = {ADDR[31:2], 2'b00};
    set_idx = mmu_desc_shadow_set_idx(addr_w);
    hit = 1'b0;
    data = 32'h0;

    if (MMU_DESC_SHADOW_WAYS == 1) begin
        if (MMU_DESC_SHADOW_V_FLAT[set_idx] &&
            MMU_DESC_SHADOW_ADDR_FLAT[(set_idx*32) +: 32] == addr_w) begin
            hit = 1'b1;
            data = MMU_DESC_SHADOW_DATA_FLAT[(set_idx*32) +: 32];
        end
    end else begin
        for (way = 0; way < MMU_DESC_SHADOW_WAYS; way = way + 1) begin
            flat_idx = (set_idx * MMU_DESC_SHADOW_WAYS) + way;
            if (!hit &&
                MMU_DESC_SHADOW_V_FLAT[flat_idx] &&
                MMU_DESC_SHADOW_ADDR_FLAT[(flat_idx*32) +: 32] == addr_w) begin
                hit = 1'b1;
                data = MMU_DESC_SHADOW_DATA_FLAT[(flat_idx*32) +: 32];
            end
        end
    end

    LOOKUP = {hit, data};
end

endmodule
