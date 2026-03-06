(* keep_hierarchy = "yes" *)
module WF68K30L_TOP_MMU_PTEST_STAGE #(
    parameter int MMU_DESC_SHADOW_LINES = 64,
    parameter int MMU_DESC_SHADOW_WAYS = 1,
    parameter int MMU_DESC_SHADOW_SETS = MMU_DESC_SHADOW_LINES / MMU_DESC_SHADOW_WAYS,
    parameter int MMU_DESC_SHADOW_SET_BITS = $clog2(MMU_DESC_SHADOW_SETS),
    parameter int MMU_DESC_SHADOW_WAY_BITS = (MMU_DESC_SHADOW_WAYS > 1) ? $clog2(MMU_DESC_SHADOW_WAYS) : 1
) (
    input  logic [31:0] MMU_TC,
    input  logic [2:0]  FC_IN,
    input  logic [31:0] LOGICAL_ADDR,
    input  logic [2:0]  SEARCH_LIMIT,
    input  logic [14:0] ROOT_LIMIT,
    input  logic        ROOT_LIMIT_LOWER,
    input  logic [MMU_DESC_SHADOW_SETS*MMU_DESC_SHADOW_WAYS-1:0] MMU_DESC_SHADOW_V_FLAT,
    input  logic [MMU_DESC_SHADOW_SETS*MMU_DESC_SHADOW_WAYS*32-1:0] MMU_DESC_SHADOW_ADDR_FLAT,
    input  logic [MMU_DESC_SHADOW_SETS*MMU_DESC_SHADOW_WAYS*32-1:0] MMU_DESC_SHADOW_DATA_FLAT,
    input  logic [31:0] IN_TABLE_BASE,
    input  logic [5:0]  IN_CONSUMED,
    input  logic [3:0]  IN_DESC_SIZE,
    input  logic [2:0]  IN_TLX_LEVEL,
    input  logic        IN_FC_LEVEL,
    input  logic [2:0]  IN_LEVELS,
    input  logic        IN_DONE,
    input  logic        IN_FAULT,
    input  logic        IN_LIMIT_FAULT,
    input  logic        IN_SUPER_FAULT,
    input  logic        IN_BUS_FAULT,
    input  logic        IN_INVALID_FAULT,
    input  logic        IN_WP_ACCUM,
    input  logic        IN_PAGE_M,
    output logic [31:0] OUT_TABLE_BASE,
    output logic [5:0]  OUT_CONSUMED,
    output logic [3:0]  OUT_DESC_SIZE,
    output logic [2:0]  OUT_TLX_LEVEL,
    output logic        OUT_FC_LEVEL,
    output logic [2:0]  OUT_LEVELS,
    output logic        OUT_DONE,
    output logic        OUT_FAULT,
    output logic        OUT_LIMIT_FAULT,
    output logic        OUT_SUPER_FAULT,
    output logic        OUT_BUS_FAULT,
    output logic        OUT_INVALID_FAULT,
    output logic        OUT_WP_ACCUM,
    output logic        OUT_PAGE_M
);

`include "wf68k30L_pkg.svh"
`include "wf68k30L_top_sections/helpers/wf68k30L_top_helpers_mmu_pure.svh"

logic        STAGE_ACTIVE;
logic [31:0] STAGE_DESC_ADDR;
logic [31:0] STAGE_INDEX;
logic [5:0]  STAGE_CONSUMED;
logic [2:0]  STAGE_TLX_LEVEL;
logic        STAGE_FC_LEVEL;
logic [3:0]  STAGE_NEXT_WIDTH;
logic        STAGE_HAS_NEXT;
logic        STAGE_PRE_FAULT;
logic        STAGE_PRE_LIMIT_FAULT;
logic        STAGE_PRE_INVALID_FAULT;

logic [32:0] STAGE_DESC_LOOKUP_LO;
logic [32:0] STAGE_DESC_LOOKUP_HI;
logic [32:0] STAGE_INDIRECT_SHORT_LOOKUP;
logic [32:0] STAGE_INDIRECT_LONG_LOOKUP_LO;
logic [32:0] STAGE_INDIRECT_LONG_LOOKUP_HI;

logic [31:0] STAGE_DESC_WORD;
logic [31:0] STAGE_DESC_HI_WORD;
logic [1:0]  STAGE_DESC_DT;
logic [31:0] STAGE_DESC_PTR;
logic [31:0] STAGE_INDIRECT_SHORT_ADDR;
logic [31:0] STAGE_INDIRECT_LONG_BASE;

assign STAGE_DESC_WORD = STAGE_DESC_LOOKUP_LO[31:0];
assign STAGE_DESC_HI_WORD = STAGE_DESC_LOOKUP_HI[31:0];
assign STAGE_DESC_DT = STAGE_DESC_WORD[1:0];
assign STAGE_DESC_PTR = (IN_DESC_SIZE == 4'd8) ? STAGE_DESC_HI_WORD : STAGE_DESC_WORD;
assign STAGE_INDIRECT_SHORT_ADDR = {STAGE_DESC_PTR[31:2], 2'b00};
assign STAGE_INDIRECT_LONG_BASE = {STAGE_DESC_PTR[31:3], 3'b000};

WF68K30L_TOP_DESC_SHADOW_LOOKUP #(
    .MMU_DESC_SHADOW_LINES(MMU_DESC_SHADOW_LINES),
    .MMU_DESC_SHADOW_WAYS(MMU_DESC_SHADOW_WAYS),
    .MMU_DESC_SHADOW_SETS(MMU_DESC_SHADOW_SETS),
    .MMU_DESC_SHADOW_SET_BITS(MMU_DESC_SHADOW_SET_BITS)
) I_DESC_LOOKUP_LO (
    .ADDR(STAGE_DESC_ADDR),
    .MMU_DESC_SHADOW_V_FLAT(MMU_DESC_SHADOW_V_FLAT),
    .MMU_DESC_SHADOW_ADDR_FLAT(MMU_DESC_SHADOW_ADDR_FLAT),
    .MMU_DESC_SHADOW_DATA_FLAT(MMU_DESC_SHADOW_DATA_FLAT),
    .LOOKUP(STAGE_DESC_LOOKUP_LO)
);

WF68K30L_TOP_DESC_SHADOW_LOOKUP #(
    .MMU_DESC_SHADOW_LINES(MMU_DESC_SHADOW_LINES),
    .MMU_DESC_SHADOW_WAYS(MMU_DESC_SHADOW_WAYS),
    .MMU_DESC_SHADOW_SETS(MMU_DESC_SHADOW_SETS),
    .MMU_DESC_SHADOW_SET_BITS(MMU_DESC_SHADOW_SET_BITS)
) I_DESC_LOOKUP_HI (
    .ADDR(STAGE_DESC_ADDR + 32'd4),
    .MMU_DESC_SHADOW_V_FLAT(MMU_DESC_SHADOW_V_FLAT),
    .MMU_DESC_SHADOW_ADDR_FLAT(MMU_DESC_SHADOW_ADDR_FLAT),
    .MMU_DESC_SHADOW_DATA_FLAT(MMU_DESC_SHADOW_DATA_FLAT),
    .LOOKUP(STAGE_DESC_LOOKUP_HI)
);

WF68K30L_TOP_DESC_SHADOW_LOOKUP #(
    .MMU_DESC_SHADOW_LINES(MMU_DESC_SHADOW_LINES),
    .MMU_DESC_SHADOW_WAYS(MMU_DESC_SHADOW_WAYS),
    .MMU_DESC_SHADOW_SETS(MMU_DESC_SHADOW_SETS),
    .MMU_DESC_SHADOW_SET_BITS(MMU_DESC_SHADOW_SET_BITS)
) I_INDIRECT_SHORT_LOOKUP (
    .ADDR(STAGE_INDIRECT_SHORT_ADDR),
    .MMU_DESC_SHADOW_V_FLAT(MMU_DESC_SHADOW_V_FLAT),
    .MMU_DESC_SHADOW_ADDR_FLAT(MMU_DESC_SHADOW_ADDR_FLAT),
    .MMU_DESC_SHADOW_DATA_FLAT(MMU_DESC_SHADOW_DATA_FLAT),
    .LOOKUP(STAGE_INDIRECT_SHORT_LOOKUP)
);

WF68K30L_TOP_DESC_SHADOW_LOOKUP #(
    .MMU_DESC_SHADOW_LINES(MMU_DESC_SHADOW_LINES),
    .MMU_DESC_SHADOW_WAYS(MMU_DESC_SHADOW_WAYS),
    .MMU_DESC_SHADOW_SETS(MMU_DESC_SHADOW_SETS),
    .MMU_DESC_SHADOW_SET_BITS(MMU_DESC_SHADOW_SET_BITS)
) I_INDIRECT_LONG_LOOKUP_LO (
    .ADDR(STAGE_INDIRECT_LONG_BASE),
    .MMU_DESC_SHADOW_V_FLAT(MMU_DESC_SHADOW_V_FLAT),
    .MMU_DESC_SHADOW_ADDR_FLAT(MMU_DESC_SHADOW_ADDR_FLAT),
    .MMU_DESC_SHADOW_DATA_FLAT(MMU_DESC_SHADOW_DATA_FLAT),
    .LOOKUP(STAGE_INDIRECT_LONG_LOOKUP_LO)
);

WF68K30L_TOP_DESC_SHADOW_LOOKUP #(
    .MMU_DESC_SHADOW_LINES(MMU_DESC_SHADOW_LINES),
    .MMU_DESC_SHADOW_WAYS(MMU_DESC_SHADOW_WAYS),
    .MMU_DESC_SHADOW_SETS(MMU_DESC_SHADOW_SETS),
    .MMU_DESC_SHADOW_SET_BITS(MMU_DESC_SHADOW_SET_BITS)
) I_INDIRECT_LONG_LOOKUP_HI (
    .ADDR(STAGE_INDIRECT_LONG_BASE + 32'd4),
    .MMU_DESC_SHADOW_V_FLAT(MMU_DESC_SHADOW_V_FLAT),
    .MMU_DESC_SHADOW_ADDR_FLAT(MMU_DESC_SHADOW_ADDR_FLAT),
    .MMU_DESC_SHADOW_DATA_FLAT(MMU_DESC_SHADOW_DATA_FLAT),
    .LOOKUP(STAGE_INDIRECT_LONG_LOOKUP_HI)
);

always_comb begin : stage_prepare
    logic [3:0] walk_width;

    STAGE_ACTIVE = !IN_DONE && !IN_FAULT && (IN_LEVELS < SEARCH_LIMIT);
    STAGE_DESC_ADDR = 32'h0;
    STAGE_INDEX = 32'h0;
    STAGE_CONSUMED = IN_CONSUMED;
    STAGE_TLX_LEVEL = IN_TLX_LEVEL;
    STAGE_FC_LEVEL = IN_FC_LEVEL;
    STAGE_NEXT_WIDTH = 4'h0;
    STAGE_HAS_NEXT = 1'b0;
    STAGE_PRE_FAULT = 1'b0;
    STAGE_PRE_LIMIT_FAULT = 1'b0;
    STAGE_PRE_INVALID_FAULT = 1'b0;
    walk_width = 4'h0;

    if (STAGE_ACTIVE) begin
        if (IN_FC_LEVEL) begin
            STAGE_INDEX = {29'h0, FC_IN};
            STAGE_FC_LEVEL = 1'b0;
            STAGE_NEXT_WIDTH = mmu_tlx_width(MMU_TC, IN_TLX_LEVEL[1:0]);
            STAGE_HAS_NEXT = (STAGE_NEXT_WIDTH != 4'h0);
        end else begin
            walk_width = mmu_tlx_width(MMU_TC, IN_TLX_LEVEL[1:0]);
            if (walk_width == 4'h0) begin
                STAGE_PRE_FAULT = 1'b1;
                STAGE_PRE_INVALID_FAULT = 1'b1;
            end else begin
                STAGE_INDEX = mmu_index_extract(LOGICAL_ADDR, MMU_TC[19:16], IN_CONSUMED, walk_width);
                STAGE_CONSUMED = IN_CONSUMED + {2'b00, walk_width};
                STAGE_TLX_LEVEL = IN_TLX_LEVEL + 3'd1;
                if (STAGE_TLX_LEVEL < 3'd4) begin
                    STAGE_NEXT_WIDTH = mmu_tlx_width(MMU_TC, STAGE_TLX_LEVEL[1:0]);
                    STAGE_HAS_NEXT = (STAGE_NEXT_WIDTH != 4'h0);
                end
                if ((IN_LEVELS == 3'd0) &&
                    !MMU_TC[24] &&
                    mmu_limit_violation(ROOT_LIMIT_LOWER, ROOT_LIMIT, STAGE_INDEX)) begin
                    STAGE_PRE_FAULT = 1'b1;
                    STAGE_PRE_LIMIT_FAULT = 1'b1;
                end
            end
        end

        if (!STAGE_PRE_FAULT)
            STAGE_DESC_ADDR = mmu_desc_addr(IN_TABLE_BASE, STAGE_INDEX, IN_DESC_SIZE);
    end
end

always_comb begin : stage_eval
    logic [2:0] level_after;
    logic [31:0] walk_limit_index;

    OUT_TABLE_BASE = IN_TABLE_BASE;
    OUT_CONSUMED = IN_CONSUMED;
    OUT_DESC_SIZE = IN_DESC_SIZE;
    OUT_TLX_LEVEL = IN_TLX_LEVEL;
    OUT_FC_LEVEL = IN_FC_LEVEL;
    OUT_LEVELS = IN_LEVELS;
    OUT_DONE = IN_DONE;
    OUT_FAULT = IN_FAULT;
    OUT_LIMIT_FAULT = IN_LIMIT_FAULT;
    OUT_SUPER_FAULT = IN_SUPER_FAULT;
    OUT_BUS_FAULT = IN_BUS_FAULT;
    OUT_INVALID_FAULT = IN_INVALID_FAULT;
    OUT_WP_ACCUM = IN_WP_ACCUM;
    OUT_PAGE_M = IN_PAGE_M;
    level_after = IN_LEVELS;
    walk_limit_index = 32'h0;

    if (STAGE_ACTIVE) begin
        OUT_CONSUMED = STAGE_CONSUMED;
        OUT_TLX_LEVEL = STAGE_TLX_LEVEL;
        OUT_FC_LEVEL = STAGE_FC_LEVEL;

        if (STAGE_PRE_FAULT) begin
            OUT_FAULT = 1'b1;
            OUT_LIMIT_FAULT = IN_LIMIT_FAULT || STAGE_PRE_LIMIT_FAULT;
            OUT_INVALID_FAULT = IN_INVALID_FAULT || STAGE_PRE_INVALID_FAULT;
        end else if (!STAGE_DESC_LOOKUP_LO[32] ||
                     ((IN_DESC_SIZE == 4'd8) && !STAGE_DESC_LOOKUP_HI[32])) begin
            OUT_FAULT = 1'b1;
            OUT_BUS_FAULT = 1'b1;
        end else begin
            level_after = IN_LEVELS + 3'd1;
            OUT_LEVELS = level_after;
            OUT_WP_ACCUM = IN_WP_ACCUM || STAGE_DESC_WORD[2];

            if ((IN_DESC_SIZE == 4'd8) && !FC_IN[2] && STAGE_DESC_WORD[8]) begin
                OUT_SUPER_FAULT = 1'b1;
                OUT_FAULT = 1'b1;
            end

            if (!OUT_FAULT) begin
                case (STAGE_DESC_DT)
                    2'b00: begin
                        OUT_FAULT = 1'b1;
                        OUT_INVALID_FAULT = 1'b1;
                    end

                    2'b01: begin
                        if ((IN_DESC_SIZE == 4'd8) && STAGE_HAS_NEXT && (STAGE_NEXT_WIDTH != 4'h0)) begin
                            walk_limit_index = mmu_index_extract(
                                LOGICAL_ADDR,
                                MMU_TC[19:16],
                                STAGE_CONSUMED,
                                STAGE_NEXT_WIDTH
                            );
                            if (mmu_limit_violation(STAGE_DESC_WORD[31], STAGE_DESC_WORD[30:16], walk_limit_index)) begin
                                OUT_FAULT = 1'b1;
                                OUT_LIMIT_FAULT = 1'b1;
                            end
                        end

                        if (!OUT_FAULT) begin
                            OUT_PAGE_M = STAGE_DESC_WORD[4];
                            OUT_DONE = 1'b1;
                        end
                    end

                    default: begin
                        if (STAGE_HAS_NEXT && (level_after < SEARCH_LIMIT)) begin
                            if ((IN_DESC_SIZE == 4'd8) && (STAGE_NEXT_WIDTH != 4'h0)) begin
                                walk_limit_index = mmu_index_extract(
                                    LOGICAL_ADDR,
                                    MMU_TC[19:16],
                                    STAGE_CONSUMED,
                                    STAGE_NEXT_WIDTH
                                );
                                if (mmu_limit_violation(STAGE_DESC_WORD[31], STAGE_DESC_WORD[30:16], walk_limit_index)) begin
                                    OUT_FAULT = 1'b1;
                                    OUT_LIMIT_FAULT = 1'b1;
                                end
                            end

                            if (!OUT_FAULT) begin
                                OUT_TABLE_BASE = (IN_DESC_SIZE == 4'd8) ?
                                                 {STAGE_DESC_HI_WORD[31:4], 4'b0000} :
                                                 {STAGE_DESC_WORD[31:4], 4'b0000};
                                OUT_DESC_SIZE = (STAGE_DESC_DT == 2'b10) ? 4'd4 : 4'd8;
                            end
                        end else if (!STAGE_HAS_NEXT) begin
                            if (STAGE_DESC_DT == 2'b10) begin
                                if (!STAGE_INDIRECT_SHORT_LOOKUP[32]) begin
                                    OUT_FAULT = 1'b1;
                                    OUT_BUS_FAULT = 1'b1;
                                end else if (STAGE_INDIRECT_SHORT_LOOKUP[1:0] != 2'b01) begin
                                    OUT_FAULT = 1'b1;
                                    OUT_INVALID_FAULT = 1'b1;
                                end else begin
                                    OUT_WP_ACCUM = OUT_WP_ACCUM || STAGE_INDIRECT_SHORT_LOOKUP[2];
                                    OUT_PAGE_M = STAGE_INDIRECT_SHORT_LOOKUP[4];
                                    OUT_DONE = 1'b1;
                                end
                            end else begin
                                if (!STAGE_INDIRECT_LONG_LOOKUP_LO[32] || !STAGE_INDIRECT_LONG_LOOKUP_HI[32]) begin
                                    OUT_FAULT = 1'b1;
                                    OUT_BUS_FAULT = 1'b1;
                                end else if (STAGE_INDIRECT_LONG_LOOKUP_LO[1:0] != 2'b01) begin
                                    OUT_FAULT = 1'b1;
                                    OUT_INVALID_FAULT = 1'b1;
                                end else begin
                                    OUT_WP_ACCUM = OUT_WP_ACCUM || STAGE_INDIRECT_LONG_LOOKUP_LO[2];
                                    if (!FC_IN[2] && STAGE_INDIRECT_LONG_LOOKUP_LO[8]) begin
                                        OUT_FAULT = 1'b1;
                                        OUT_SUPER_FAULT = 1'b1;
                                    end
                                    if (!OUT_FAULT) begin
                                        OUT_PAGE_M = STAGE_INDIRECT_LONG_LOOKUP_LO[4];
                                        OUT_DONE = 1'b1;
                                    end
                                end
                            end
                        end
                    end
                endcase
            end
        end
    end
end

endmodule
