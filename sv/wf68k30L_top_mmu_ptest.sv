(* keep_hierarchy = "yes" *)
module WF68K30L_TOP_MMU_PTEST (
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
    output logic        SHADOW_RD_EN,
    output logic [31:0] SHADOW_RD_ADDR,
    input  logic [32:0] SHADOW_LOOKUP,
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

localparam logic [3:0] PTEST_ST_IDLE             = 4'd0;
localparam logic [3:0] PTEST_ST_STEP             = 4'd1;
localparam logic [3:0] PTEST_ST_FETCH_LO_REQ     = 4'd2;
localparam logic [3:0] PTEST_ST_FETCH_LO_RESP    = 4'd3;
localparam logic [3:0] PTEST_ST_FETCH_HI_REQ     = 4'd4;
localparam logic [3:0] PTEST_ST_FETCH_HI_RESP    = 4'd5;
localparam logic [3:0] PTEST_ST_EVAL             = 4'd6;
localparam logic [3:0] PTEST_ST_INDIRECT_LO_REQ  = 4'd7;
localparam logic [3:0] PTEST_ST_INDIRECT_LO_RESP = 4'd8;
localparam logic [3:0] PTEST_ST_INDIRECT_HI_REQ  = 4'd9;
localparam logic [3:0] PTEST_ST_INDIRECT_HI_RESP = 4'd10;

logic [3:0]  ptest_state_r;
logic [31:0] ptest_tc_r;
logic [2:0]  ptest_fc_r;
logic [31:0] ptest_logical_r;
logic [2:0]  search_limit_r;
logic [14:0] root_limit_r;
logic        root_limit_lower_r;

logic [31:0] walk_table_base_r;
logic [31:0] walk_desc_addr_r;
logic [31:0] walk_desc_ptr_r;
logic [5:0]  walk_consumed_r;
logic [3:0]  walk_desc_size_r;
logic [3:0]  walk_next_width_r;
logic [2:0]  walk_tlx_level_r;
logic        walk_fc_level_r;
logic [2:0]  walk_levels_r;
logic [1:0]  walk_desc_dt_r;
logic        walk_has_next_r;
logic        walk_done_r;
logic        walk_fault_r;
logic        walk_limit_fault_r;
logic        walk_super_fault_r;
logic        walk_bus_fault_r;
logic        walk_invalid_fault_r;
logic        walk_wp_accum_r;
logic        walk_page_m_r;
logic [31:0] walk_fetch_lo_word_r;
logic [31:0] walk_fetch_hi_word_r;
logic        walk_fetch_lo_valid_r;
logic        walk_fetch_hi_valid_r;

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

always_comb begin : ptest_shadow_req
    SHADOW_RD_EN = 1'b0;
    SHADOW_RD_ADDR = 32'h0000_0000;
    case (ptest_state_r)
        PTEST_ST_FETCH_LO_REQ: begin
            SHADOW_RD_EN = 1'b1;
            SHADOW_RD_ADDR = walk_desc_addr_r;
        end
        PTEST_ST_FETCH_HI_REQ: begin
            SHADOW_RD_EN = 1'b1;
            SHADOW_RD_ADDR = walk_desc_addr_r + 32'd4;
        end
        PTEST_ST_INDIRECT_LO_REQ: begin
            SHADOW_RD_EN = 1'b1;
            SHADOW_RD_ADDR = (walk_desc_dt_r == 2'b10) ?
                             {walk_desc_ptr_r[31:2], 2'b00} :
                             {walk_desc_ptr_r[31:3], 3'b000};
        end
        PTEST_ST_INDIRECT_HI_REQ: begin
            SHADOW_RD_EN = 1'b1;
            SHADOW_RD_ADDR = {walk_desc_ptr_r[31:3], 3'b000} + 32'd4;
        end
        default: begin end
    endcase
end

always_ff @(posedge CLK) begin : ptest_fsm
    logic [63:0] root_ptr;
    logic [1:0]  root_dt;
    logic [2:0]  search_limit;
    logic [31:0] first_index;
    logic [3:0]  walk_width;
    logic [3:0]  walk_next_width;
    logic [31:0] walk_index;
    logic [31:0] walk_limit_index;
    logic [2:0]  next_tlx_level;
    logic [2:0]  level_after;
    logic [1:0]  desc_dt;
    logic        walk_has_next;
    logic        walk_fault;
    logic        walk_limit_fault;
    logic        walk_super_fault;
    logic        walk_bus_fault;
    logic        walk_invalid_fault;
    logic        walk_wp_accum;
    logic        walk_page_m;
    logic        walk_done;
    logic        force_invalid;
    if (RESET_CPU) begin
        PTEST_BUSY <= 1'b0;
        PTEST_READY <= 1'b0;
        PTEST_WALK_MMUSR <= 16'h0000;
        ptest_state_r <= PTEST_ST_IDLE;
        ptest_tc_r <= 32'h0000_0000;
        ptest_fc_r <= 3'b000;
        ptest_logical_r <= 32'h0000_0000;
        search_limit_r <= 3'b000;
        root_limit_r <= 15'h0000;
        root_limit_lower_r <= 1'b0;
        walk_table_base_r <= 32'h0000_0000;
        walk_desc_addr_r <= 32'h0000_0000;
        walk_desc_ptr_r <= 32'h0000_0000;
        walk_consumed_r <= 6'd0;
        walk_desc_size_r <= 4'd0;
        walk_next_width_r <= 4'd0;
        walk_tlx_level_r <= 3'd0;
        walk_fc_level_r <= 1'b0;
        walk_levels_r <= 3'd0;
        walk_desc_dt_r <= 2'b00;
        walk_has_next_r <= 1'b0;
        walk_done_r <= 1'b0;
        walk_fault_r <= 1'b0;
        walk_limit_fault_r <= 1'b0;
        walk_super_fault_r <= 1'b0;
        walk_bus_fault_r <= 1'b0;
        walk_invalid_fault_r <= 1'b0;
        walk_wp_accum_r <= 1'b0;
        walk_page_m_r <= 1'b0;
        walk_fetch_lo_word_r <= 32'h0000_0000;
        walk_fetch_hi_word_r <= 32'h0000_0000;
        walk_fetch_lo_valid_r <= 1'b0;
        walk_fetch_hi_valid_r <= 1'b0;
    end else begin
        if (PTEST_CONSUME)
            PTEST_READY <= 1'b0;

        case (ptest_state_r)
            PTEST_ST_IDLE: begin
                PTEST_BUSY <= 1'b0;
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
                    walk_desc_addr_r <= 32'h0000_0000;
                    walk_desc_ptr_r <= 32'h0000_0000;
                    walk_consumed_r <= 6'd0;
                    walk_desc_size_r <= (root_dt == 2'b11) ? 4'd8 : 4'd4;
                    walk_next_width_r <= 4'd0;
                    walk_tlx_level_r <= 3'd0;
                    walk_fc_level_r <= MMU_TC[24];
                    walk_levels_r <= 3'd0;
                    walk_desc_dt_r <= 2'b00;
                    walk_has_next_r <= 1'b0;
                    walk_done_r <= 1'b0;
                    walk_fault_r <= 1'b0;
                    walk_limit_fault_r <= 1'b0;
                    walk_super_fault_r <= 1'b0;
                    walk_bus_fault_r <= 1'b0;
                    walk_invalid_fault_r <= 1'b0;
                    walk_wp_accum_r <= 1'b0;
                    walk_page_m_r <= 1'b0;
                    walk_fetch_lo_word_r <= 32'h0000_0000;
                    walk_fetch_hi_word_r <= 32'h0000_0000;
                    walk_fetch_lo_valid_r <= 1'b0;
                    walk_fetch_hi_valid_r <= 1'b0;

                    if (root_dt == 2'b01) begin
                        PTEST_WALK_MMUSR <= ptest_root_mmusr(root_ptr[62:48], root_ptr[63], first_index);
                        PTEST_READY <= 1'b1;
                    end else if (root_dt == 2'b10 || root_dt == 2'b11) begin
                        PTEST_BUSY <= 1'b1;
                        PTEST_READY <= 1'b0;
                        ptest_state_r <= PTEST_ST_STEP;
                    end else begin
                        PTEST_WALK_MMUSR <= MMUSR_I;
                        PTEST_READY <= 1'b1;
                    end
                end
            end

            PTEST_ST_STEP: begin
                walk_fault = 1'b0;
                walk_limit_fault = walk_limit_fault_r;
                walk_invalid_fault = walk_invalid_fault_r;
                walk_index = 32'h0000_0000;
                walk_width = 4'h0;
                walk_next_width = 4'h0;
                walk_has_next = 1'b0;

                if (walk_levels_r >= 3'd5)
                    walk_fault = 1'b1;

                if (!walk_fault) begin
                    if (walk_fc_level_r) begin
                        walk_index = {29'h0, ptest_fc_r};
                        walk_next_width = mmu_tlx_width(ptest_tc_r, walk_tlx_level_r[1:0]);
                        walk_has_next = (walk_next_width != 4'h0);
                        walk_fc_level_r <= 1'b0;
                    end else begin
                        walk_width = mmu_tlx_width(ptest_tc_r, walk_tlx_level_r[1:0]);
                        if (walk_width == 4'h0) begin
                            walk_fault = 1'b1;
                            walk_invalid_fault = 1'b1;
                        end else begin
                            walk_index = mmu_index_extract(
                                ptest_logical_r,
                                ptest_tc_r[19:16],
                                walk_consumed_r,
                                walk_width
                            );
                            walk_consumed_r <= walk_consumed_r + {2'b00, walk_width};
                            next_tlx_level = walk_tlx_level_r + 3'd1;
                            walk_tlx_level_r <= next_tlx_level;
                            if (next_tlx_level < 3'd4) begin
                                walk_next_width = mmu_tlx_width(ptest_tc_r, next_tlx_level[1:0]);
                                walk_has_next = (walk_next_width != 4'h0);
                            end
                            if ((walk_levels_r == 3'd0) && !ptest_tc_r[24] &&
                                mmu_limit_violation(root_limit_lower_r, root_limit_r, walk_index)) begin
                                walk_fault = 1'b1;
                                walk_limit_fault = 1'b1;
                            end
                        end
                    end
                end

                if (walk_fault) begin
                    PTEST_WALK_MMUSR <= ptest_walk_mmusr(
                        walk_limit_fault,
                        walk_super_fault_r,
                        walk_bus_fault_r,
                        walk_wp_accum_r,
                        1'b0,
                        walk_page_m_r,
                        walk_invalid_fault,
                        1'b0,
                        walk_levels_r
                    );
                    PTEST_BUSY <= 1'b0;
                    PTEST_READY <= 1'b1;
                    ptest_state_r <= PTEST_ST_IDLE;
                    walk_fault_r <= walk_fault;
                    walk_limit_fault_r <= walk_limit_fault;
                    walk_invalid_fault_r <= walk_invalid_fault;
                end else begin
                    walk_next_width_r <= walk_next_width;
                    walk_has_next_r <= walk_has_next;
                    walk_desc_addr_r <= mmu_desc_addr(walk_table_base_r, walk_index, walk_desc_size_r);
                    ptest_state_r <= PTEST_ST_FETCH_LO_REQ;
                end
            end

            PTEST_ST_FETCH_LO_REQ: begin
                ptest_state_r <= PTEST_ST_FETCH_LO_RESP;
            end

            PTEST_ST_FETCH_LO_RESP: begin
                walk_fetch_lo_word_r <= SHADOW_LOOKUP[31:0];
                walk_fetch_lo_valid_r <= SHADOW_LOOKUP[32];
                if (walk_desc_size_r == 4'd8) begin
                    ptest_state_r <= PTEST_ST_FETCH_HI_REQ;
                end else begin
                    walk_fetch_hi_word_r <= 32'h0000_0000;
                    walk_fetch_hi_valid_r <= 1'b1;
                    ptest_state_r <= PTEST_ST_EVAL;
                end
            end

            PTEST_ST_FETCH_HI_REQ: begin
                ptest_state_r <= PTEST_ST_FETCH_HI_RESP;
            end

            PTEST_ST_FETCH_HI_RESP: begin
                walk_fetch_hi_word_r <= SHADOW_LOOKUP[31:0];
                walk_fetch_hi_valid_r <= SHADOW_LOOKUP[32];
                ptest_state_r <= PTEST_ST_EVAL;
            end

            PTEST_ST_EVAL: begin
                walk_fault = walk_fault_r;
                walk_limit_fault = walk_limit_fault_r;
                walk_super_fault = walk_super_fault_r;
                walk_bus_fault = walk_bus_fault_r;
                walk_invalid_fault = walk_invalid_fault_r;
                walk_wp_accum = walk_wp_accum_r;
                walk_page_m = walk_page_m_r;
                walk_done = walk_done_r;
                desc_dt = walk_fetch_lo_word_r[1:0];
                level_after = walk_levels_r + 3'd1;
                force_invalid = 1'b0;

                walk_wp_accum = walk_wp_accum || walk_fetch_lo_word_r[2];

                if (!walk_fetch_lo_valid_r || !walk_fetch_hi_valid_r) begin
                    walk_fault = 1'b1;
                    walk_bus_fault = 1'b1;
                end
                if (!walk_fault && (walk_desc_size_r == 4'd8) && !ptest_fc_r[2] && walk_fetch_lo_word_r[8]) begin
                    walk_fault = 1'b1;
                    walk_super_fault = 1'b1;
                end

                if (!walk_fault) begin
                    case (desc_dt)
                        2'b00: begin
                            walk_fault = 1'b1;
                            walk_invalid_fault = 1'b1;
                        end

                        2'b01: begin
                            if ((walk_desc_size_r == 4'd8) && walk_has_next_r && (walk_next_width_r != 4'h0)) begin
                                walk_limit_index = mmu_index_extract(
                                    ptest_logical_r,
                                    ptest_tc_r[19:16],
                                    walk_consumed_r,
                                    walk_next_width_r
                                );
                                if (mmu_limit_violation(walk_fetch_lo_word_r[31], walk_fetch_lo_word_r[30:16], walk_limit_index)) begin
                                    walk_fault = 1'b1;
                                    walk_limit_fault = 1'b1;
                                end
                            end

                            if (!walk_fault) begin
                                walk_done = 1'b1;
                                walk_page_m = walk_fetch_lo_word_r[4];
                            end
                        end

                        default: begin
                            if (walk_has_next_r && (level_after < search_limit_r)) begin
                                if ((walk_desc_size_r == 4'd8) && (walk_next_width_r != 4'h0)) begin
                                    walk_limit_index = mmu_index_extract(
                                        ptest_logical_r,
                                        ptest_tc_r[19:16],
                                        walk_consumed_r,
                                        walk_next_width_r
                                    );
                                    if (mmu_limit_violation(walk_fetch_lo_word_r[31], walk_fetch_lo_word_r[30:16], walk_limit_index)) begin
                                        walk_fault = 1'b1;
                                        walk_limit_fault = 1'b1;
                                    end
                                end

                                if (!walk_fault) begin
                                    walk_table_base_r <= (walk_desc_size_r == 4'd8) ?
                                                         {walk_fetch_hi_word_r[31:4], 4'b0000} :
                                                         {walk_fetch_lo_word_r[31:4], 4'b0000};
                                    walk_desc_size_r <= (desc_dt == 2'b10) ? 4'd4 : 4'd8;
                                    walk_wp_accum_r <= walk_wp_accum;
                                    walk_levels_r <= level_after;
                                    ptest_state_r <= PTEST_ST_STEP;
                                end
                            end else if (!walk_has_next_r) begin
                                walk_desc_ptr_r <= (walk_desc_size_r == 4'd8) ? walk_fetch_hi_word_r : walk_fetch_lo_word_r;
                                walk_desc_dt_r <= desc_dt;
                                walk_wp_accum_r <= walk_wp_accum;
                                walk_levels_r <= level_after;
                                ptest_state_r <= PTEST_ST_INDIRECT_LO_REQ;
                            end else begin
                                force_invalid = (search_limit_r == 3'd7);
                            end
                        end
                    endcase
                end

                if ((ptest_state_r == PTEST_ST_EVAL) && (walk_done || walk_fault || force_invalid || (level_after >= search_limit_r && walk_has_next_r))) begin
                    PTEST_WALK_MMUSR <= ptest_walk_mmusr(
                        walk_limit_fault,
                        walk_super_fault,
                        walk_bus_fault,
                        walk_wp_accum,
                        walk_done,
                        walk_page_m,
                        walk_invalid_fault,
                        force_invalid,
                        level_after
                    );
                    PTEST_BUSY <= 1'b0;
                    PTEST_READY <= 1'b1;
                    ptest_state_r <= PTEST_ST_IDLE;
                    walk_done_r <= walk_done;
                    walk_fault_r <= walk_fault;
                    walk_limit_fault_r <= walk_limit_fault;
                    walk_super_fault_r <= walk_super_fault;
                    walk_bus_fault_r <= walk_bus_fault;
                    walk_invalid_fault_r <= walk_invalid_fault;
                    walk_wp_accum_r <= walk_wp_accum;
                    walk_page_m_r <= walk_page_m;
                end
            end

            PTEST_ST_INDIRECT_LO_REQ: begin
                ptest_state_r <= PTEST_ST_INDIRECT_LO_RESP;
            end

            PTEST_ST_INDIRECT_LO_RESP: begin
                walk_fetch_lo_word_r <= SHADOW_LOOKUP[31:0];
                walk_fetch_lo_valid_r <= SHADOW_LOOKUP[32];
                if (walk_desc_dt_r == 2'b10)
                    ptest_state_r <= PTEST_ST_INDIRECT_HI_RESP;
                else
                    ptest_state_r <= PTEST_ST_INDIRECT_HI_REQ;
            end

            PTEST_ST_INDIRECT_HI_REQ: begin
                ptest_state_r <= PTEST_ST_INDIRECT_HI_RESP;
            end

            PTEST_ST_INDIRECT_HI_RESP: begin
                walk_fault = 1'b0;
                walk_limit_fault = walk_limit_fault_r;
                walk_super_fault = 1'b0;
                walk_bus_fault = 1'b0;
                walk_invalid_fault = 1'b0;
                walk_wp_accum = walk_wp_accum_r;
                walk_page_m = walk_page_m_r;
                walk_done = 1'b0;

                if (walk_desc_dt_r == 2'b10) begin
                    if (!walk_fetch_lo_valid_r) begin
                        walk_fault = 1'b1;
                        walk_bus_fault = 1'b1;
                    end else if (walk_fetch_lo_word_r[1:0] != 2'b01) begin
                        walk_fault = 1'b1;
                        walk_invalid_fault = 1'b1;
                    end else begin
                        walk_wp_accum = walk_wp_accum || walk_fetch_lo_word_r[2];
                        walk_page_m = walk_fetch_lo_word_r[4];
                        walk_done = 1'b1;
                    end
                end else begin
                    if (!walk_fetch_lo_valid_r || !SHADOW_LOOKUP[32]) begin
                        walk_fault = 1'b1;
                        walk_bus_fault = 1'b1;
                    end else if (walk_fetch_lo_word_r[1:0] != 2'b01) begin
                        walk_fault = 1'b1;
                        walk_invalid_fault = 1'b1;
                    end else begin
                        walk_wp_accum = walk_wp_accum || walk_fetch_lo_word_r[2];
                        if (!ptest_fc_r[2] && walk_fetch_lo_word_r[8]) begin
                            walk_fault = 1'b1;
                            walk_super_fault = 1'b1;
                        end else begin
                            walk_page_m = walk_fetch_lo_word_r[4];
                            walk_done = 1'b1;
                        end
                    end
                end

                PTEST_WALK_MMUSR <= ptest_walk_mmusr(
                    walk_limit_fault,
                    walk_super_fault,
                    walk_bus_fault,
                    walk_wp_accum,
                    walk_done,
                    walk_page_m,
                    walk_invalid_fault,
                    1'b0,
                    walk_levels_r
                );
                PTEST_BUSY <= 1'b0;
                PTEST_READY <= 1'b1;
                ptest_state_r <= PTEST_ST_IDLE;
                walk_done_r <= walk_done;
                walk_fault_r <= walk_fault;
                walk_limit_fault_r <= walk_limit_fault;
                walk_super_fault_r <= walk_super_fault;
                walk_bus_fault_r <= walk_bus_fault;
                walk_invalid_fault_r <= walk_invalid_fault;
                walk_wp_accum_r <= walk_wp_accum;
                walk_page_m_r <= walk_page_m;
            end

            default: begin
                PTEST_BUSY <= 1'b0;
                ptest_state_r <= PTEST_ST_IDLE;
            end
        endcase
    end
end
endmodule
