(* keep_hierarchy = "yes" *)
module WF68K30L_TOP_MMU_PTEST #(
    parameter int MMU_DESC_SHADOW_LINES = 64,
    parameter int MMU_DESC_SHADOW_WAYS = 1,
    parameter int MMU_DESC_SHADOW_SETS = MMU_DESC_SHADOW_LINES / MMU_DESC_SHADOW_WAYS,
    parameter int MMU_DESC_SHADOW_SET_BITS = $clog2(MMU_DESC_SHADOW_SETS),
    parameter int MMU_DESC_SHADOW_WAY_BITS = (MMU_DESC_SHADOW_WAYS > 1) ? $clog2(MMU_DESC_SHADOW_WAYS) : 1
) (
    input  logic        CLK,
    input  logic        RESET_CPU,
    input  logic        PTEST_START,
    input  logic        PTEST_CONSUME,
    input  logic [63:0] MMU_SRP,
    input  logic [63:0] MMU_CRP,
    input  logic [31:0] MMU_TC,
    input  logic [2:0]  PTEST_FC,
    input  logic [31:0] PTEST_LOGICAL,
    input  logic [2:0]  PTEST_LEVEL,
    input  logic [MMU_DESC_SHADOW_SETS*MMU_DESC_SHADOW_WAYS-1:0] MMU_DESC_SHADOW_V_FLAT,
    input  logic [MMU_DESC_SHADOW_SETS*MMU_DESC_SHADOW_WAYS*32-1:0] MMU_DESC_SHADOW_ADDR_FLAT,
    input  logic [MMU_DESC_SHADOW_SETS*MMU_DESC_SHADOW_WAYS*32-1:0] MMU_DESC_SHADOW_DATA_FLAT,
    output logic        PTEST_BUSY,
    output logic        PTEST_READY,
    output logic [15:0] PTEST_WALK_MMUSR
);

`include "wf68k30L_pkg.svh"
`include "wf68k30L_top_sections/helpers/wf68k30L_top_helpers_mmu_pure.svh"

localparam logic [15:0] MMUSR_B = 16'h8000;
localparam logic [15:0] MMUSR_L = 16'h4000;
localparam logic [15:0] MMUSR_S = 16'h2000;
localparam logic [15:0] MMUSR_W = 16'h0800;
localparam logic [15:0] MMUSR_I = 16'h0400;
localparam logic [15:0] MMUSR_M = 16'h0200;
localparam logic [15:0] MMUSR_N = 16'h0007;

logic [31:0] ptest_tc_r;
logic [2:0]  ptest_fc_r;
logic [31:0] ptest_logical_r;
logic [2:0]  search_limit_r;
logic [14:0] root_limit_r;
logic        root_limit_lower_r;

logic [31:0] walk_table_base_r;
logic [5:0]  walk_consumed_r;
logic [3:0]  walk_desc_size_r;
logic [2:0]  walk_tlx_level_r;
logic        walk_fc_level_r;
logic [2:0]  walk_levels_r;
logic        walk_done_r;
logic        walk_fault_r;
logic        walk_limit_fault_r;
logic        walk_super_fault_r;
logic        walk_bus_fault_r;
logic        walk_invalid_fault_r;
logic        walk_wp_accum_r;
logic        walk_page_m_r;

logic [31:0] walk_table_base_next;
logic [5:0]  walk_consumed_next;
logic [3:0]  walk_desc_size_next;
logic [2:0]  walk_tlx_level_next;
logic        walk_fc_level_next;
logic [2:0]  walk_levels_next;
logic        walk_done_next;
logic        walk_fault_next;
logic        walk_limit_fault_next;
logic        walk_super_fault_next;
logic        walk_bus_fault_next;
logic        walk_invalid_fault_next;
logic        walk_wp_accum_next;
logic        walk_page_m_next;

logic        walk_complete_now;
logic        walk_force_invalid_now;
logic [15:0] walk_mmusr_now;

function automatic logic [15:0] ptest_root_mmusr(
    input logic [14:0] limit,
    input logic        limit_lower,
    input logic [31:0] first_index
);
    logic [15:0] mmusr;
begin
    mmusr = 16'h0000;
    if (mmu_limit_violation(limit_lower, limit, first_index))
        mmusr = mmusr | MMUSR_L | MMUSR_I;
    ptest_root_mmusr = mmusr & ~MMUSR_N;
end
endfunction

function automatic logic [15:0] ptest_walk_mmusr(
    input logic        limit_fault,
    input logic        super_fault,
    input logic        bus_fault,
    input logic        wp_accum,
    input logic        done,
    input logic        page_m,
    input logic        invalid_fault,
    input logic        force_invalid,
    input logic [2:0]  levels
);
    logic [15:0] mmusr;
begin
    mmusr = 16'h0000;
    if (limit_fault)
        mmusr = mmusr | MMUSR_L;
    if (super_fault)
        mmusr = mmusr | MMUSR_S;
    if (bus_fault)
        mmusr = mmusr | MMUSR_B;
    if (wp_accum)
        mmusr = mmusr | MMUSR_W;
    if (done && page_m)
        mmusr = mmusr | MMUSR_M;
    if (invalid_fault || force_invalid || bus_fault || limit_fault)
        mmusr = mmusr | MMUSR_I;
    if ((mmusr & MMUSR_L) != 16'h0000)
        mmusr = mmusr & ~(MMUSR_W | MMUSR_M);
    ptest_walk_mmusr = (mmusr & ~MMUSR_N) | ({13'h0, levels} & MMUSR_N);
end
endfunction

WF68K30L_TOP_MMU_PTEST_STAGE #(
    .MMU_DESC_SHADOW_LINES(MMU_DESC_SHADOW_LINES),
    .MMU_DESC_SHADOW_WAYS(MMU_DESC_SHADOW_WAYS),
    .MMU_DESC_SHADOW_SETS(MMU_DESC_SHADOW_SETS),
    .MMU_DESC_SHADOW_SET_BITS(MMU_DESC_SHADOW_SET_BITS),
    .MMU_DESC_SHADOW_WAY_BITS(MMU_DESC_SHADOW_WAY_BITS)
) I_STAGE (
    .MMU_TC(ptest_tc_r),
    .FC_IN(ptest_fc_r),
    .LOGICAL_ADDR(ptest_logical_r),
    .SEARCH_LIMIT(search_limit_r),
    .ROOT_LIMIT(root_limit_r),
    .ROOT_LIMIT_LOWER(root_limit_lower_r),
    .MMU_DESC_SHADOW_V_FLAT(MMU_DESC_SHADOW_V_FLAT),
    .MMU_DESC_SHADOW_ADDR_FLAT(MMU_DESC_SHADOW_ADDR_FLAT),
    .MMU_DESC_SHADOW_DATA_FLAT(MMU_DESC_SHADOW_DATA_FLAT),
    .IN_TABLE_BASE(walk_table_base_r),
    .IN_CONSUMED(walk_consumed_r),
    .IN_DESC_SIZE(walk_desc_size_r),
    .IN_TLX_LEVEL(walk_tlx_level_r),
    .IN_FC_LEVEL(walk_fc_level_r),
    .IN_LEVELS(walk_levels_r),
    .IN_DONE(walk_done_r),
    .IN_FAULT(walk_fault_r),
    .IN_LIMIT_FAULT(walk_limit_fault_r),
    .IN_SUPER_FAULT(walk_super_fault_r),
    .IN_BUS_FAULT(walk_bus_fault_r),
    .IN_INVALID_FAULT(walk_invalid_fault_r),
    .IN_WP_ACCUM(walk_wp_accum_r),
    .IN_PAGE_M(walk_page_m_r),
    .OUT_TABLE_BASE(walk_table_base_next),
    .OUT_CONSUMED(walk_consumed_next),
    .OUT_DESC_SIZE(walk_desc_size_next),
    .OUT_TLX_LEVEL(walk_tlx_level_next),
    .OUT_FC_LEVEL(walk_fc_level_next),
    .OUT_LEVELS(walk_levels_next),
    .OUT_DONE(walk_done_next),
    .OUT_FAULT(walk_fault_next),
    .OUT_LIMIT_FAULT(walk_limit_fault_next),
    .OUT_SUPER_FAULT(walk_super_fault_next),
    .OUT_BUS_FAULT(walk_bus_fault_next),
    .OUT_INVALID_FAULT(walk_invalid_fault_next),
    .OUT_WP_ACCUM(walk_wp_accum_next),
    .OUT_PAGE_M(walk_page_m_next)
);

assign walk_complete_now = walk_done_next || walk_fault_next || (walk_levels_next >= search_limit_r);
assign walk_force_invalid_now = !walk_fault_next && !walk_done_next && (search_limit_r == 3'd7);
assign walk_mmusr_now = ptest_walk_mmusr(
    walk_limit_fault_next,
    walk_super_fault_next,
    walk_bus_fault_next,
    walk_wp_accum_next,
    walk_done_next,
    walk_page_m_next,
    walk_invalid_fault_next,
    walk_force_invalid_now,
    walk_levels_next
);

always_ff @(posedge CLK) begin : ptest_fsm
    logic [63:0] root_ptr;
    logic [1:0]  root_dt;
    logic [2:0]  search_limit;
    logic [31:0] first_index;
    if (RESET_CPU) begin
        PTEST_BUSY <= 1'b0;
        PTEST_READY <= 1'b0;
        PTEST_WALK_MMUSR <= 16'h0000;
        ptest_tc_r <= 32'h0000_0000;
        ptest_fc_r <= 3'b000;
        ptest_logical_r <= 32'h0000_0000;
        search_limit_r <= 3'b000;
        root_limit_r <= 15'h0000;
        root_limit_lower_r <= 1'b0;
        walk_table_base_r <= 32'h0000_0000;
        walk_consumed_r <= 6'd0;
        walk_desc_size_r <= 4'd0;
        walk_tlx_level_r <= 3'd0;
        walk_fc_level_r <= 1'b0;
        walk_levels_r <= 3'd0;
        walk_done_r <= 1'b0;
        walk_fault_r <= 1'b0;
        walk_limit_fault_r <= 1'b0;
        walk_super_fault_r <= 1'b0;
        walk_bus_fault_r <= 1'b0;
        walk_invalid_fault_r <= 1'b0;
        walk_wp_accum_r <= 1'b0;
        walk_page_m_r <= 1'b0;
    end else begin
        if (PTEST_CONSUME)
            PTEST_READY <= 1'b0;

        if (PTEST_START) begin
            root_ptr = (MMU_TC[25] && PTEST_FC[2]) ? MMU_SRP : MMU_CRP;
            root_dt = root_ptr[33:32];
            search_limit = (PTEST_LEVEL == 3'b000) ? 3'd7 : PTEST_LEVEL;
            first_index = MMU_TC[24] ?
                          {29'h0, PTEST_FC} :
                          mmu_index_extract(PTEST_LOGICAL, MMU_TC[19:16], 6'd0, MMU_TC[15:12]);

            ptest_tc_r <= MMU_TC;
            ptest_fc_r <= PTEST_FC;
            ptest_logical_r <= PTEST_LOGICAL;
            search_limit_r <= search_limit;
            root_limit_r <= root_ptr[62:48];
            root_limit_lower_r <= root_ptr[63];

            walk_table_base_r <= {root_ptr[31:4], 4'b0000};
            walk_consumed_r <= 6'd0;
            walk_desc_size_r <= (root_dt == 2'b11) ? 4'd8 : 4'd4;
            walk_tlx_level_r <= 3'd0;
            walk_fc_level_r <= MMU_TC[24];
            walk_levels_r <= 3'd0;
            walk_done_r <= 1'b0;
            walk_fault_r <= 1'b0;
            walk_limit_fault_r <= 1'b0;
            walk_super_fault_r <= 1'b0;
            walk_bus_fault_r <= 1'b0;
            walk_invalid_fault_r <= 1'b0;
            walk_wp_accum_r <= 1'b0;
            walk_page_m_r <= 1'b0;

            if (root_dt == 2'b01) begin
                PTEST_WALK_MMUSR <= ptest_root_mmusr(root_ptr[62:48], root_ptr[63], first_index);
                PTEST_BUSY <= 1'b0;
                PTEST_READY <= 1'b1;
            end else if (root_dt == 2'b10 || root_dt == 2'b11) begin
                PTEST_WALK_MMUSR <= 16'h0000;
                PTEST_BUSY <= 1'b1;
                PTEST_READY <= 1'b0;
            end else begin
                PTEST_WALK_MMUSR <= MMUSR_I;
                PTEST_BUSY <= 1'b0;
                PTEST_READY <= 1'b1;
            end
        end else if (PTEST_BUSY) begin
            walk_table_base_r <= walk_table_base_next;
            walk_consumed_r <= walk_consumed_next;
            walk_desc_size_r <= walk_desc_size_next;
            walk_tlx_level_r <= walk_tlx_level_next;
            walk_fc_level_r <= walk_fc_level_next;
            walk_levels_r <= walk_levels_next;
            walk_done_r <= walk_done_next;
            walk_fault_r <= walk_fault_next;
            walk_limit_fault_r <= walk_limit_fault_next;
            walk_super_fault_r <= walk_super_fault_next;
            walk_bus_fault_r <= walk_bus_fault_next;
            walk_invalid_fault_r <= walk_invalid_fault_next;
            walk_wp_accum_r <= walk_wp_accum_next;
            walk_page_m_r <= walk_page_m_next;

            if (walk_complete_now) begin
                PTEST_WALK_MMUSR <= walk_mmusr_now;
                PTEST_BUSY <= 1'b0;
                PTEST_READY <= 1'b1;
            end
        end
    end
end

endmodule
