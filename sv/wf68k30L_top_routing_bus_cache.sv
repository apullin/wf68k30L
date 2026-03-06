(* keep_hierarchy = "yes" *)
module WF68K30L_TOP_ROUTING_BUS_CACHE #(
    parameter int MMU_DESC_SHADOW_LINES = 64,
    parameter int MMU_DESC_SHADOW_WAYS = 1,
    parameter int MMU_DESC_SHADOW_SETS = MMU_DESC_SHADOW_LINES / MMU_DESC_SHADOW_WAYS,
    parameter int MMU_DESC_SHADOW_SET_BITS = $clog2(MMU_DESC_SHADOW_SETS),
    parameter int MMU_DESC_SHADOW_WAY_BITS = (MMU_DESC_SHADOW_WAYS > 1) ? $clog2(MMU_DESC_SHADOW_WAYS) : 1
) (
    input  logic        CLK,
    input  logic        RESET_CPU,
    input  logic        BUS_BSY,
    input  logic        BUSY_EXH,

    input  logic        DATA_RD_EXH,
    input  logic        DATA_RD_MAIN,
    input  logic        DATA_WR_EXH,
    input  logic        DATA_WR_MAIN,
    input  logic        OPCODE_RD,

    input  logic        MMU_RUNTIME_REQ,
    input  logic        MMU_RUNTIME_FAULT,
    input  logic        MMU_RUNTIME_STALL,
    input  logic [2:0]  MMU_RUNTIME_ATC_FC,

    input  logic [31:0] MMU_TC,
    input  logic [63:0] MMU_SRP,
    input  logic [63:0] MMU_CRP,
    input  logic [31:0] ADR_P,
    input  logic [31:0] ADR_P_PHYS,
    input  logic [31:0] DATA_OUT,
    input  logic [31:0] DATA_TO_CORE_BUSIF,
    input  logic        DATA_RDY_BUSIF_CORE,
    input  logic        ICACHE_HIT_NOW,
    input  logic        DCACHE_HIT_NOW,
    input  logic [2:0]  MMU_PTEST_FC,
    input  logic [31:0] MMU_PTEST_LOGICAL,
    input  logic [2:0]  MMU_PTEST_LEVEL,
    input  logic        MMU_PTEST_START,
    input  logic        MMU_PTEST_CONSUME,

    input  logic        ICACHE_BURST_FILL_VALID,
    input  logic [3:0]  ICACHE_BURST_FILL_LINE,
    input  logic [23:0] ICACHE_BURST_FILL_TAG,
    input  logic [7:0]  ICACHE_BURST_FILL_PENDING,
    input  logic [2:0]  ICACHE_BURST_FILL_FC,
    input  logic [2:0]  ICACHE_BURST_FILL_NEXT_WORD,

    input  logic        DCACHE_BURST_FILL_VALID,
    input  logic [3:0]  DCACHE_BURST_FILL_LINE,
    input  logic [23:0] DCACHE_BURST_FILL_TAG,
    input  logic [3:0]  DCACHE_BURST_FILL_PENDING,
    input  logic [2:0]  DCACHE_BURST_FILL_FC,
    input  logic [1:0]  DCACHE_BURST_FILL_NEXT_ENTRY,

    input  logic        MMU_TWALK_START,

    output logic        DATA_RD,
    output logic        DATA_WR,
    output logic        DATA_RD_BUS,

    output logic        BURST_PREFETCH_OP_REQ,
    output logic        BURST_PREFETCH_DATA_REQ,
    output logic [2:0]  BURST_PREFETCH_OP_WORD,
    output logic [1:0]  BURST_PREFETCH_DATA_ENTRY,
    output logic [31:0] BURST_PREFETCH_ADDR,
    output logic [2:0]  BURST_PREFETCH_FC,

    output logic        RD_REQ,
    output logic        WR_REQ,
    output logic        OPCODE_REQ,
    output logic        OPCODE_REQ_CORE,
    output logic        OPCODE_REQ_CORE_MISS,

    output logic        RD_REQ_I,
    output logic        WR_REQ_I,
    output logic        OPCODE_REQ_I,
    output logic        MMU_FAULT_DATA_ACK,
    output logic        MMU_FAULT_OPCODE_ACK,
    output logic        BUS_CYCLE_BURST,
    output logic        BUS_CYCLE_BURST_IS_OP,

    output logic        MMU_TWALK_BUSY,
    output logic        MMU_TWALK_VALID,
    output logic [2:0]  MMU_TWALK_FC,
    output logic [31:0] MMU_TWALK_LOGICAL,
    output logic        MMU_TWALK_WRITE,
    output logic [35:0] MMU_TWALK_RESULT,
    output logic [2:0]  MMU_TWALK_STATE,
    output logic [31:0] MMU_TWALK_TC,
    output logic [31:0] MMU_TWALK_TABLE_BASE,
    output logic [31:0] MMU_TWALK_DESC_ADDR,
    output logic [31:0] MMU_TWALK_PAGE_BASE,
    output logic [31:0] MMU_TWALK_DESC_PTR,
    output logic [31:0] MMU_TWALK_INDEX,
    output logic [5:0]  MMU_TWALK_CONSUMED,
    output logic [3:0]  MMU_TWALK_DESC_SIZE,
    output logic [3:0]  MMU_TWALK_NEXT_WIDTH,
    output logic [2:0]  MMU_TWALK_LEVELS,
    output logic [2:0]  MMU_TWALK_TLX_LEVEL,
    output logic [1:0]  MMU_TWALK_DESC_DT,
    output logic [14:0] MMU_TWALK_ROOT_LIMIT,
    output logic        MMU_TWALK_ROOT_LIMIT_LOWER,
    output logic        MMU_TWALK_FC_LEVEL,
    output logic        MMU_TWALK_HAS_NEXT,
    output logic        MMU_TWALK_WP_ACCUM,
    output logic        MMU_PTEST_BUSY,
    output logic        MMU_PTEST_READY,
    output logic [15:0] MMU_PTEST_WALK_MMUSR
);

`include "wf68k30L_pkg.svh"
`include "wf68k30L_top_sections/helpers/wf68k30L_top_helpers_mmu_pure.svh"

logic        MMU_DESC_SHADOW_PENDING;
logic [31:0] MMU_DESC_SHADOW_PENDING_ADDR;
logic        MMU_DESC_SHADOW_PENDING_WR;
logic        MMU_DESC_SHADOW_WR_EN;
logic [31:0] MMU_DESC_SHADOW_WR_ADDR;
logic [31:0] MMU_DESC_SHADOW_WR_DATA;
logic [31:0] MMU_TWALK_FETCH_LO_WORD;
logic [31:0] MMU_TWALK_FETCH_HI_WORD;
logic        MMU_TWALK_FETCH_LO_VALID;
logic        MMU_TWALK_FETCH_HI_VALID;
logic [31:0] MMU_TWALK_INDIRECT_SHORT_ADDR;
logic [31:0] MMU_TWALK_INDIRECT_LONG_ADDR;
logic        MMU_TWALK_SHADOW_RD_EN;
logic [31:0] MMU_TWALK_SHADOW_RD_ADDR;
logic [32:0] MMU_TWALK_SHADOW_LOOKUP;
logic        MMU_PTEST_SHADOW_RD_EN;
logic [31:0] MMU_PTEST_SHADOW_RD_ADDR;
logic [32:0] MMU_PTEST_SHADOW_LOOKUP;
logic [1:0]  MMU_TWALK_DESC_DT_CUR;
logic [31:0] MMU_TWALK_DESC_PAGE_BASE;
logic [31:0] MMU_TWALK_DESC_TABLE_BASE_NEXT;
logic [31:0] MMU_TWALK_NEXT_LIMIT_INDEX;
logic [31:0] MMU_TWALK_OFFSET_MASK_CUR;

assign MMU_TWALK_INDIRECT_SHORT_ADDR = {MMU_TWALK_DESC_PTR[31:2], 2'b00};
assign MMU_TWALK_INDIRECT_LONG_ADDR = {MMU_TWALK_DESC_PTR[31:3], 3'b000};

function automatic logic [35:0] mmu_twalk_fault_result(
    input logic [31:0] logical_addr,
    input logic        wp_accum,
    input logic        write_access
);
begin
    mmu_twalk_fault_result = 36'h0;
    mmu_twalk_fault_result[31:0] = logical_addr;
    mmu_twalk_fault_result[32] = 1'b1;
    mmu_twalk_fault_result[34] = wp_accum;
    mmu_twalk_fault_result[35] = write_access;
end
endfunction

function automatic logic [35:0] mmu_twalk_page_result(
    input logic [31:0] phys_addr,
    input logic        fault,
    input logic        wp_accum,
    input logic        page_m
);
begin
    mmu_twalk_page_result = 36'h0;
    mmu_twalk_page_result[31:0] = phys_addr;
    mmu_twalk_page_result[32] = fault;
    mmu_twalk_page_result[34] = wp_accum;
    mmu_twalk_page_result[35] = page_m;
end
endfunction

function automatic logic [31:0] mmu_twalk_offset_mask(
    input logic [31:0] tc_in,
    input logic [5:0]  consumed_bits
);
    logic [5:0] used_bits;
begin
    used_bits = {2'b00, tc_in[19:16]} + consumed_bits;
    if (used_bits >= 6'd32)
        mmu_twalk_offset_mask = 32'h0000_0000;
    else
        mmu_twalk_offset_mask = 32'hFFFF_FFFF >> used_bits;
end
endfunction

assign MMU_TWALK_DESC_DT_CUR = MMU_TWALK_FETCH_LO_WORD[1:0];
assign MMU_TWALK_DESC_PAGE_BASE = (MMU_TWALK_DESC_SIZE == 4'd8) ?
                                  {MMU_TWALK_FETCH_HI_WORD[31:8], 8'h00} :
                                  {MMU_TWALK_FETCH_LO_WORD[31:8], 8'h00};
assign MMU_TWALK_DESC_TABLE_BASE_NEXT = (MMU_TWALK_DESC_SIZE == 4'd8) ?
                                        {MMU_TWALK_FETCH_HI_WORD[31:4], 4'b0000} :
                                        {MMU_TWALK_FETCH_LO_WORD[31:4], 4'b0000};
assign MMU_TWALK_NEXT_LIMIT_INDEX = mmu_index_extract(
    MMU_TWALK_LOGICAL,
    MMU_TWALK_TC[19:16],
    MMU_TWALK_CONSUMED,
    MMU_TWALK_NEXT_WIDTH
);
assign MMU_TWALK_OFFSET_MASK_CUR = mmu_twalk_offset_mask(MMU_TWALK_TC, MMU_TWALK_CONSUMED);

WF68K30L_TOP_DESC_SHADOW_PORT #(
    .MMU_DESC_SHADOW_LINES(MMU_DESC_SHADOW_LINES),
    .MMU_DESC_SHADOW_WAYS(MMU_DESC_SHADOW_WAYS),
    .MMU_DESC_SHADOW_SETS(MMU_DESC_SHADOW_SETS),
    .MMU_DESC_SHADOW_SET_BITS(MMU_DESC_SHADOW_SET_BITS),
    .MMU_DESC_SHADOW_WAY_BITS(MMU_DESC_SHADOW_WAY_BITS)
) I_MMU_TWALK_DESC_SHADOW (
    .CLK(CLK),
    .RESET_CPU(RESET_CPU),
    .RD_EN(MMU_TWALK_SHADOW_RD_EN),
    .RD_ADDR(MMU_TWALK_SHADOW_RD_ADDR),
    .RD_LOOKUP(MMU_TWALK_SHADOW_LOOKUP),
    .WR_EN(MMU_DESC_SHADOW_WR_EN),
    .WR_ADDR(MMU_DESC_SHADOW_WR_ADDR),
    .WR_DATA(MMU_DESC_SHADOW_WR_DATA)
);

WF68K30L_TOP_DESC_SHADOW_PORT #(
    .MMU_DESC_SHADOW_LINES(MMU_DESC_SHADOW_LINES),
    .MMU_DESC_SHADOW_WAYS(MMU_DESC_SHADOW_WAYS),
    .MMU_DESC_SHADOW_SETS(MMU_DESC_SHADOW_SETS),
    .MMU_DESC_SHADOW_SET_BITS(MMU_DESC_SHADOW_SET_BITS),
    .MMU_DESC_SHADOW_WAY_BITS(MMU_DESC_SHADOW_WAY_BITS)
) I_MMU_PTEST_DESC_SHADOW (
    .CLK(CLK),
    .RESET_CPU(RESET_CPU),
    .RD_EN(MMU_PTEST_SHADOW_RD_EN),
    .RD_ADDR(MMU_PTEST_SHADOW_RD_ADDR),
    .RD_LOOKUP(MMU_PTEST_SHADOW_LOOKUP),
    .WR_EN(MMU_DESC_SHADOW_WR_EN),
    .WR_ADDR(MMU_DESC_SHADOW_WR_ADDR),
    .WR_DATA(MMU_DESC_SHADOW_WR_DATA)
);

WF68K30L_TOP_MMU_PTEST I_TOP_MMU_PTEST (
    .CLK(CLK),
    .RESET_CPU(RESET_CPU),
    .PTEST_START(MMU_PTEST_START),
    .PTEST_CONSUME(MMU_PTEST_CONSUME),
    .MMU_SRP(MMU_SRP),
    .MMU_CRP(MMU_CRP),
    .MMU_TC(MMU_TC),
    .PTEST_FC(MMU_PTEST_FC),
    .PTEST_LOGICAL(MMU_PTEST_LOGICAL),
    .PTEST_LEVEL(MMU_PTEST_LEVEL),
    .SHADOW_RD_EN(MMU_PTEST_SHADOW_RD_EN),
    .SHADOW_RD_ADDR(MMU_PTEST_SHADOW_RD_ADDR),
    .SHADOW_LOOKUP(MMU_PTEST_SHADOW_LOOKUP),
    .PTEST_BUSY(MMU_PTEST_BUSY),
    .PTEST_READY(MMU_PTEST_READY),
    .PTEST_WALK_MMUSR(MMU_PTEST_WALK_MMUSR)
);

// ========================================================================
// Bus request arbitration
// ========================================================================

assign DATA_RD = DATA_RD_EXH || DATA_RD_MAIN;
assign DATA_WR = DATA_WR_EXH || DATA_WR_MAIN;

always_comb begin : burst_prefetch_select
    integer      scan_idx;
    logic        icache_found;
    logic [2:0]  icache_scan_word;
    logic        dcache_found;
    logic [1:0]  dcache_scan_entry;
    BURST_PREFETCH_OP_REQ = 1'b0;
    BURST_PREFETCH_DATA_REQ = 1'b0;
    BURST_PREFETCH_OP_WORD = 3'b000;
    BURST_PREFETCH_DATA_ENTRY = 2'b00;
    BURST_PREFETCH_ADDR = 32'h0000_0000;
    BURST_PREFETCH_FC = FC_USER_PROG;
    scan_idx = 0;
    icache_found = 1'b0;
    icache_scan_word = ICACHE_BURST_FILL_NEXT_WORD;
    dcache_found = 1'b0;
    dcache_scan_entry = DCACHE_BURST_FILL_NEXT_ENTRY;

    if (!BUS_BSY && !DATA_WR && !DATA_RD && !OPCODE_RD && !BUSY_EXH) begin
        if (ICACHE_BURST_FILL_VALID && ICACHE_BURST_FILL_PENDING != 8'h00) begin
            for (scan_idx = 0; scan_idx < 8; scan_idx = scan_idx + 1) begin
                if (!icache_found) begin
                    icache_scan_word = ICACHE_BURST_FILL_NEXT_WORD + scan_idx[2:0];
                    if (ICACHE_BURST_FILL_PENDING[icache_scan_word]) begin
                        BURST_PREFETCH_OP_WORD = icache_scan_word;
                        icache_found = 1'b1;
                    end
                end
            end
            BURST_PREFETCH_OP_REQ = 1'b1;
            BURST_PREFETCH_ADDR = {
                ICACHE_BURST_FILL_TAG,
                ICACHE_BURST_FILL_LINE,
                BURST_PREFETCH_OP_WORD,
                1'b0
            };
            BURST_PREFETCH_FC = ICACHE_BURST_FILL_FC;
        end else if (DCACHE_BURST_FILL_VALID && DCACHE_BURST_FILL_PENDING != 4'h0) begin
            for (scan_idx = 0; scan_idx < 4; scan_idx = scan_idx + 1) begin
                if (!dcache_found) begin
                    dcache_scan_entry = DCACHE_BURST_FILL_NEXT_ENTRY + scan_idx[1:0];
                    if (DCACHE_BURST_FILL_PENDING[dcache_scan_entry]) begin
                        BURST_PREFETCH_DATA_ENTRY = dcache_scan_entry;
                        dcache_found = 1'b1;
                    end
                end
            end
            BURST_PREFETCH_DATA_REQ = 1'b1;
            BURST_PREFETCH_ADDR = {
                DCACHE_BURST_FILL_TAG,
                DCACHE_BURST_FILL_LINE,
                BURST_PREFETCH_DATA_ENTRY,
                2'b00
            };
            BURST_PREFETCH_FC = DCACHE_BURST_FILL_FC;
        end
    end
end

// Request/fault latches that decouple core-side combinational logic from bus FSM timing.
always_ff @(posedge CLK) begin : bus_req_latch
    // Flip-flops break combinatorial loops between core requests and bus controller.
    // Requests are valid until the bus controller enters START_CYCLE and asserts BUS_BSY.
    if (RESET_CPU) begin
        RD_REQ_I <= 1'b0;
        WR_REQ_I <= 1'b0;
        OPCODE_REQ_I <= 1'b0;
        MMU_FAULT_DATA_ACK <= 1'b0;
        MMU_FAULT_OPCODE_ACK <= 1'b0;
        BUS_CYCLE_BURST <= 1'b0;
        BUS_CYCLE_BURST_IS_OP <= 1'b0;
    end else if (!BUS_BSY) begin
        MMU_FAULT_DATA_ACK <= MMU_RUNTIME_FAULT && (DATA_RD_BUS || DATA_WR);
        MMU_FAULT_OPCODE_ACK <= MMU_RUNTIME_FAULT && OPCODE_REQ_CORE_MISS && !DATA_RD_BUS && !DATA_WR;
        RD_REQ_I <= (DATA_RD_BUS && !MMU_RUNTIME_FAULT && !MMU_RUNTIME_STALL) || BURST_PREFETCH_DATA_REQ;
        WR_REQ_I <= DATA_WR && !MMU_RUNTIME_FAULT && !MMU_RUNTIME_STALL;
        OPCODE_REQ_I <= (OPCODE_REQ_CORE_MISS && !MMU_RUNTIME_FAULT && !MMU_RUNTIME_STALL) || BURST_PREFETCH_OP_REQ;
        if (BURST_PREFETCH_OP_REQ) begin
            BUS_CYCLE_BURST <= 1'b1;
            BUS_CYCLE_BURST_IS_OP <= 1'b1;
        end else if (BURST_PREFETCH_DATA_REQ) begin
            BUS_CYCLE_BURST <= 1'b1;
            BUS_CYCLE_BURST_IS_OP <= 1'b0;
        end else begin
            BUS_CYCLE_BURST <= 1'b0;
            BUS_CYCLE_BURST_IS_OP <= 1'b0;
        end
    end else if (BUS_BSY) begin
        RD_REQ_I <= 1'b0;
        WR_REQ_I <= 1'b0;
        OPCODE_REQ_I <= 1'b0;
        MMU_FAULT_DATA_ACK <= 1'b0;
        MMU_FAULT_OPCODE_ACK <= 1'b0;
    end
end

localparam logic [3:0] MMU_TWALK_ST_IDLE             = 4'd0;
localparam logic [3:0] MMU_TWALK_ST_STEP             = 4'd1;
localparam logic [3:0] MMU_TWALK_ST_FETCH_LO_REQ     = 4'd2;
localparam logic [3:0] MMU_TWALK_ST_FETCH_LO_RESP    = 4'd3;
localparam logic [3:0] MMU_TWALK_ST_FETCH_HI_REQ     = 4'd4;
localparam logic [3:0] MMU_TWALK_ST_FETCH_HI_RESP    = 4'd5;
localparam logic [3:0] MMU_TWALK_ST_EVAL             = 4'd6;
localparam logic [3:0] MMU_TWALK_ST_INDIRECT_LO_REQ  = 4'd7;
localparam logic [3:0] MMU_TWALK_ST_INDIRECT_LO_RESP = 4'd8;
localparam logic [3:0] MMU_TWALK_ST_INDIRECT_HI_REQ  = 4'd9;
localparam logic [3:0] MMU_TWALK_ST_INDIRECT_HI_RESP = 4'd10;

logic [3:0] MMU_TWALK_STATE_INT;

always_comb begin : mmu_twalk_shadow_read
    MMU_TWALK_SHADOW_RD_EN = 1'b0;
    MMU_TWALK_SHADOW_RD_ADDR = 32'h0000_0000;
    case (MMU_TWALK_STATE_INT)
        MMU_TWALK_ST_FETCH_LO_REQ: begin
            MMU_TWALK_SHADOW_RD_EN = 1'b1;
            MMU_TWALK_SHADOW_RD_ADDR = MMU_TWALK_DESC_ADDR;
        end
        MMU_TWALK_ST_FETCH_HI_REQ: begin
            MMU_TWALK_SHADOW_RD_EN = 1'b1;
            MMU_TWALK_SHADOW_RD_ADDR = MMU_TWALK_DESC_ADDR + 32'd4;
        end
        MMU_TWALK_ST_INDIRECT_LO_REQ: begin
            MMU_TWALK_SHADOW_RD_EN = 1'b1;
            MMU_TWALK_SHADOW_RD_ADDR = (MMU_TWALK_DESC_DT == 2'b10) ?
                                       MMU_TWALK_INDIRECT_SHORT_ADDR :
                                       MMU_TWALK_INDIRECT_LONG_ADDR;
        end
        MMU_TWALK_ST_INDIRECT_HI_REQ: begin
            MMU_TWALK_SHADOW_RD_EN = 1'b1;
            MMU_TWALK_SHADOW_RD_ADDR = MMU_TWALK_INDIRECT_LONG_ADDR + 32'd4;
        end
        default: begin end
    endcase
end

always_comb begin : mmu_twalk_state_status
    unique case (MMU_TWALK_STATE_INT)
        MMU_TWALK_ST_IDLE: MMU_TWALK_STATE = 3'd0;
        MMU_TWALK_ST_STEP: MMU_TWALK_STATE = 3'd1;
        MMU_TWALK_ST_FETCH_LO_REQ,
        MMU_TWALK_ST_FETCH_LO_RESP: MMU_TWALK_STATE = 3'd2;
        MMU_TWALK_ST_FETCH_HI_REQ,
        MMU_TWALK_ST_FETCH_HI_RESP: MMU_TWALK_STATE = 3'd3;
        MMU_TWALK_ST_EVAL: MMU_TWALK_STATE = 3'd4;
        MMU_TWALK_ST_INDIRECT_LO_REQ,
        MMU_TWALK_ST_INDIRECT_LO_RESP: MMU_TWALK_STATE = 3'd5;
        default: MMU_TWALK_STATE = 3'd6;
    endcase
end

// Runtime MMU table-walk sequencer.
always_ff @(posedge CLK) begin : mmu_runtime_walk_eval
    logic [63:0] root_ptr;
    logic [32:0] walk_lookup;
    logic [35:0] walk_result;
    logic [31:0] walk_desc;
    logic [31:0] walk_index;
    logic [5:0]  next_consumed;
    logic [3:0]  walk_width;
    logic [3:0]  walk_next_width;
    logic [2:0]  next_tlx_level;
    logic [2:0]  next_levels;
    logic [1:0]  walk_tlx_sel;
    logic [1:0]  walk_tlx_next_sel;
    logic [1:0]  walk_desc_dt;
    logic [14:0] walk_limit;
    logic        walk_limit_lower;
    logic        walk_has_next;
    logic        walk_fault;
    logic        walk_wp_accum;
    logic        walk_page_m;
    if (RESET_CPU) begin
        MMU_TWALK_BUSY <= 1'b0;
        MMU_TWALK_VALID <= 1'b0;
        MMU_TWALK_FC <= 3'b000;
        MMU_TWALK_LOGICAL <= 32'h0000_0000;
        MMU_TWALK_WRITE <= 1'b0;
        MMU_TWALK_RESULT <= 36'h0;
        MMU_TWALK_TC <= 32'h0;
        MMU_TWALK_TABLE_BASE <= 32'h0;
        MMU_TWALK_DESC_ADDR <= 32'h0;
        MMU_TWALK_PAGE_BASE <= 32'h0;
        MMU_TWALK_DESC_PTR <= 32'h0;
        MMU_TWALK_INDEX <= 32'h0;
        MMU_TWALK_CONSUMED <= 6'd0;
        MMU_TWALK_DESC_SIZE <= 4'd0;
        MMU_TWALK_NEXT_WIDTH <= 4'd0;
        MMU_TWALK_LEVELS <= 3'd0;
        MMU_TWALK_TLX_LEVEL <= 3'd0;
        MMU_TWALK_DESC_DT <= 2'b00;
        MMU_TWALK_ROOT_LIMIT <= 15'h0;
        MMU_TWALK_ROOT_LIMIT_LOWER <= 1'b0;
        MMU_TWALK_FC_LEVEL <= 1'b0;
        MMU_TWALK_HAS_NEXT <= 1'b0;
        MMU_TWALK_WP_ACCUM <= 1'b0;
        MMU_TWALK_FETCH_LO_WORD <= 32'h0;
        MMU_TWALK_FETCH_HI_WORD <= 32'h0;
        MMU_TWALK_FETCH_LO_VALID <= 1'b0;
        MMU_TWALK_FETCH_HI_VALID <= 1'b0;
        MMU_TWALK_STATE_INT <= MMU_TWALK_ST_IDLE;
    end else if (BUS_BSY) begin
        MMU_TWALK_BUSY <= 1'b0;
        MMU_TWALK_VALID <= 1'b0;
        MMU_TWALK_STATE_INT <= MMU_TWALK_ST_IDLE;
        MMU_TWALK_FETCH_LO_VALID <= 1'b0;
        MMU_TWALK_FETCH_HI_VALID <= 1'b0;
    end else begin
        case (MMU_TWALK_STATE_INT)
            MMU_TWALK_ST_IDLE: begin
                MMU_TWALK_BUSY <= 1'b0;
                if (!MMU_RUNTIME_REQ)
                    MMU_TWALK_VALID <= 1'b0;

                if (MMU_TWALK_START && !MMU_TWALK_BUSY) begin
                    root_ptr = (MMU_TC[25] && MMU_RUNTIME_ATC_FC[2]) ? MMU_SRP : MMU_CRP;

                    MMU_TWALK_FC <= MMU_RUNTIME_ATC_FC;
                    MMU_TWALK_LOGICAL <= ADR_P;
                    MMU_TWALK_WRITE <= DATA_WR;
                    MMU_TWALK_TC <= MMU_TC;
                    MMU_TWALK_ROOT_LIMIT <= root_ptr[62:48];
                    MMU_TWALK_ROOT_LIMIT_LOWER <= root_ptr[63];
                    MMU_TWALK_TABLE_BASE <= {root_ptr[31:4], 4'b0000};
                    MMU_TWALK_DESC_SIZE <= (root_ptr[33:32] == 2'b11) ? 4'd8 : 4'd4;
                    MMU_TWALK_DESC_ADDR <= 32'h0;
                    MMU_TWALK_PAGE_BASE <= ADR_P;
                    MMU_TWALK_DESC_PTR <= 32'h0;
                    MMU_TWALK_INDEX <= 32'h0;
                    MMU_TWALK_CONSUMED <= 6'd0;
                    MMU_TWALK_NEXT_WIDTH <= 4'h0;
                    MMU_TWALK_LEVELS <= 3'd0;
                    MMU_TWALK_TLX_LEVEL <= 3'd0;
                    MMU_TWALK_DESC_DT <= 2'b00;
                    MMU_TWALK_FC_LEVEL <= MMU_TC[24];
                    MMU_TWALK_HAS_NEXT <= 1'b0;
                    MMU_TWALK_WP_ACCUM <= 1'b0;
                    MMU_TWALK_FETCH_LO_WORD <= 32'h0;
                    MMU_TWALK_FETCH_HI_WORD <= 32'h0;
                    MMU_TWALK_FETCH_LO_VALID <= 1'b0;
                    MMU_TWALK_FETCH_HI_VALID <= 1'b0;
                    MMU_TWALK_VALID <= 1'b0;

                    if (root_ptr[33:32] != 2'b10 && root_ptr[33:32] != 2'b11) begin
                        walk_result = mmu_twalk_fault_result(ADR_P, 1'b0, DATA_WR);
                        MMU_TWALK_RESULT <= walk_result;
                        MMU_TWALK_VALID <= 1'b1;
                        MMU_TWALK_BUSY <= 1'b0;
                        MMU_TWALK_STATE_INT <= MMU_TWALK_ST_IDLE;
                    end else begin
                        MMU_TWALK_BUSY <= 1'b1;
                        MMU_TWALK_STATE_INT <= MMU_TWALK_ST_STEP;
                    end
                end
            end

            MMU_TWALK_ST_STEP: begin
                walk_fault = 1'b0;
                walk_index = 32'h0;
                walk_width = 4'h0;
                walk_next_width = 4'h0;
                walk_has_next = 1'b0;
                walk_tlx_sel = MMU_TWALK_TLX_LEVEL[1:0];
                walk_tlx_next_sel = (MMU_TWALK_TLX_LEVEL == 3'd3) ? 2'd3 : (MMU_TWALK_TLX_LEVEL[1:0] + 2'd1);

                if (MMU_TWALK_LEVELS >= 3'd5)
                    walk_fault = 1'b1;

                if (!walk_fault) begin
                    if (MMU_TWALK_FC_LEVEL) begin
                        walk_index = {29'h0, MMU_TWALK_FC};
                        walk_next_width = mmu_tlx_width(MMU_TWALK_TC, walk_tlx_sel);
                        walk_has_next = (walk_next_width != 4'h0);
                        MMU_TWALK_FC_LEVEL <= 1'b0;
                    end else begin
                        walk_width = mmu_tlx_width(MMU_TWALK_TC, walk_tlx_sel);
                        if (walk_width == 4'h0) begin
                            walk_fault = 1'b1;
                        end else begin
                            walk_index = mmu_index_extract(
                                MMU_TWALK_LOGICAL,
                                MMU_TWALK_TC[19:16],
                                MMU_TWALK_CONSUMED,
                                walk_width
                            );
                            next_consumed = MMU_TWALK_CONSUMED + {2'b00, walk_width};
                            MMU_TWALK_CONSUMED <= next_consumed;

                            next_tlx_level = MMU_TWALK_TLX_LEVEL + 3'd1;
                            MMU_TWALK_TLX_LEVEL <= next_tlx_level;
                            if (next_tlx_level < 3'd4) begin
                                walk_tlx_next_sel = next_tlx_level[1:0];
                                walk_next_width = mmu_tlx_width(MMU_TWALK_TC, walk_tlx_next_sel);
                                walk_has_next = (walk_next_width != 4'h0);
                            end

                            if (MMU_TWALK_LEVELS == 3'd0 && !MMU_TWALK_TC[24] &&
                                mmu_limit_violation(MMU_TWALK_ROOT_LIMIT_LOWER, MMU_TWALK_ROOT_LIMIT, walk_index))
                                walk_fault = 1'b1;
                        end
                    end
                end

                if (walk_fault) begin
                    walk_result = mmu_twalk_fault_result(MMU_TWALK_LOGICAL, MMU_TWALK_WP_ACCUM, MMU_TWALK_WRITE);
                    MMU_TWALK_RESULT <= walk_result;
                    MMU_TWALK_VALID <= 1'b1;
                    MMU_TWALK_BUSY <= 1'b0;
                    MMU_TWALK_STATE_INT <= MMU_TWALK_ST_IDLE;
                end else begin
                    MMU_TWALK_INDEX <= walk_index;
                    MMU_TWALK_NEXT_WIDTH <= walk_next_width;
                    MMU_TWALK_HAS_NEXT <= walk_has_next;
                    MMU_TWALK_DESC_ADDR <= mmu_desc_addr(MMU_TWALK_TABLE_BASE, walk_index, MMU_TWALK_DESC_SIZE);
                    MMU_TWALK_STATE_INT <= MMU_TWALK_ST_FETCH_LO_REQ;
                end
            end

            MMU_TWALK_ST_FETCH_LO_REQ: begin
                MMU_TWALK_STATE_INT <= MMU_TWALK_ST_FETCH_LO_RESP;
            end

            MMU_TWALK_ST_FETCH_LO_RESP: begin
                walk_lookup = MMU_TWALK_SHADOW_LOOKUP;
                MMU_TWALK_FETCH_LO_WORD <= walk_lookup[31:0];
                MMU_TWALK_FETCH_LO_VALID <= walk_lookup[32];
                if (MMU_TWALK_DESC_SIZE == 4'd8) begin
                    MMU_TWALK_STATE_INT <= MMU_TWALK_ST_FETCH_HI_REQ;
                end else begin
                    MMU_TWALK_FETCH_HI_WORD <= 32'h0;
                    MMU_TWALK_FETCH_HI_VALID <= 1'b1;
                    MMU_TWALK_STATE_INT <= MMU_TWALK_ST_EVAL;
                end
            end

            MMU_TWALK_ST_FETCH_HI_REQ: begin
                MMU_TWALK_STATE_INT <= MMU_TWALK_ST_FETCH_HI_RESP;
            end

            MMU_TWALK_ST_FETCH_HI_RESP: begin
                walk_lookup = MMU_TWALK_SHADOW_LOOKUP;
                MMU_TWALK_FETCH_HI_WORD <= walk_lookup[31:0];
                MMU_TWALK_FETCH_HI_VALID <= walk_lookup[32];
                MMU_TWALK_STATE_INT <= MMU_TWALK_ST_EVAL;
            end

            MMU_TWALK_ST_EVAL: begin
                walk_fault = 1'b0;
                walk_wp_accum = MMU_TWALK_WP_ACCUM;
                walk_page_m = MMU_TWALK_WRITE;

                walk_desc = MMU_TWALK_FETCH_LO_WORD;
                walk_desc_dt = MMU_TWALK_DESC_DT_CUR;
                next_levels = MMU_TWALK_LEVELS + 3'd1;
                walk_wp_accum = walk_wp_accum || walk_desc[2];

                if (!MMU_TWALK_FETCH_LO_VALID || !MMU_TWALK_FETCH_HI_VALID)
                    walk_fault = 1'b1;
                if (!walk_fault && MMU_TWALK_DESC_SIZE == 4'd8 && !MMU_TWALK_FC[2] && walk_desc[8])
                    walk_fault = 1'b1;

                if (!walk_fault) begin
                    case (walk_desc_dt)
                        2'b00: begin
                            walk_result = mmu_twalk_fault_result(MMU_TWALK_LOGICAL, walk_wp_accum, MMU_TWALK_WRITE);
                            MMU_TWALK_RESULT <= walk_result;
                            MMU_TWALK_VALID <= 1'b1;
                            MMU_TWALK_BUSY <= 1'b0;
                            MMU_TWALK_STATE_INT <= MMU_TWALK_ST_IDLE;
                        end
                        2'b01: begin
                            if (MMU_TWALK_DESC_SIZE == 4'd8 && MMU_TWALK_HAS_NEXT && MMU_TWALK_NEXT_WIDTH != 4'h0) begin
                                walk_limit_lower = walk_desc[31];
                                walk_limit = walk_desc[30:16];
                                if (mmu_limit_violation(walk_limit_lower, walk_limit, MMU_TWALK_NEXT_LIMIT_INDEX))
                                    walk_fault = 1'b1;
                            end

                            walk_page_m = walk_desc[4] || MMU_TWALK_WRITE;
                            walk_result = mmu_twalk_page_result(
                                MMU_TWALK_DESC_PAGE_BASE + (MMU_TWALK_LOGICAL & MMU_TWALK_OFFSET_MASK_CUR),
                                walk_fault,
                                walk_wp_accum,
                                walk_page_m
                            );
                            if (MMU_TWALK_WRITE && walk_wp_accum)
                                walk_result[32] = 1'b1;
                            MMU_TWALK_RESULT <= walk_result;
                            MMU_TWALK_VALID <= 1'b1;
                            MMU_TWALK_BUSY <= 1'b0;
                            MMU_TWALK_STATE_INT <= MMU_TWALK_ST_IDLE;
                        end
                        default: begin
                            if (MMU_TWALK_HAS_NEXT) begin
                                if (MMU_TWALK_DESC_SIZE == 4'd8 && MMU_TWALK_NEXT_WIDTH != 4'h0) begin
                                    walk_limit_lower = walk_desc[31];
                                    walk_limit = walk_desc[30:16];
                                    if (mmu_limit_violation(walk_limit_lower, walk_limit, MMU_TWALK_NEXT_LIMIT_INDEX))
                                        walk_fault = 1'b1;
                                end

                                if (walk_fault) begin
                                    walk_result = mmu_twalk_fault_result(
                                        MMU_TWALK_LOGICAL,
                                        walk_wp_accum,
                                        MMU_TWALK_WRITE
                                    );
                                    MMU_TWALK_RESULT <= walk_result;
                                    MMU_TWALK_VALID <= 1'b1;
                                    MMU_TWALK_BUSY <= 1'b0;
                                    MMU_TWALK_STATE_INT <= MMU_TWALK_ST_IDLE;
                                end else begin
                                    MMU_TWALK_TABLE_BASE <= MMU_TWALK_DESC_TABLE_BASE_NEXT;
                                    MMU_TWALK_DESC_SIZE <= (walk_desc_dt == 2'b10) ? 4'd4 : 4'd8;
                                    MMU_TWALK_WP_ACCUM <= walk_wp_accum;
                                    MMU_TWALK_LEVELS <= next_levels;
                                    MMU_TWALK_STATE_INT <= MMU_TWALK_ST_STEP;
                                end
                            end else begin
                                MMU_TWALK_DESC_PTR <= (MMU_TWALK_DESC_SIZE == 4'd8) ? MMU_TWALK_FETCH_HI_WORD : walk_desc;
                                MMU_TWALK_DESC_DT <= walk_desc_dt;
                                MMU_TWALK_WP_ACCUM <= walk_wp_accum;
                                MMU_TWALK_LEVELS <= next_levels;
                                MMU_TWALK_STATE_INT <= MMU_TWALK_ST_INDIRECT_LO_REQ;
                            end
                        end
                    endcase
                end else begin
                    walk_result = mmu_twalk_fault_result(MMU_TWALK_LOGICAL, walk_wp_accum, MMU_TWALK_WRITE);
                    MMU_TWALK_RESULT <= walk_result;
                    MMU_TWALK_VALID <= 1'b1;
                    MMU_TWALK_BUSY <= 1'b0;
                    MMU_TWALK_STATE_INT <= MMU_TWALK_ST_IDLE;
                end
            end

            MMU_TWALK_ST_INDIRECT_LO_REQ: begin
                MMU_TWALK_STATE_INT <= MMU_TWALK_ST_INDIRECT_LO_RESP;
            end

            MMU_TWALK_ST_INDIRECT_LO_RESP: begin
                walk_lookup = MMU_TWALK_SHADOW_LOOKUP;
                MMU_TWALK_FETCH_LO_WORD <= walk_lookup[31:0];
                MMU_TWALK_FETCH_LO_VALID <= walk_lookup[32];
                if (MMU_TWALK_DESC_DT == 2'b10)
                    MMU_TWALK_STATE_INT <= MMU_TWALK_ST_INDIRECT_HI_RESP;
                else
                    MMU_TWALK_STATE_INT <= MMU_TWALK_ST_INDIRECT_HI_REQ;
            end

            MMU_TWALK_ST_INDIRECT_HI_REQ: begin
                MMU_TWALK_STATE_INT <= MMU_TWALK_ST_INDIRECT_HI_RESP;
            end

            MMU_TWALK_ST_INDIRECT_HI_RESP: begin
                walk_fault = 1'b0;
                walk_wp_accum = MMU_TWALK_WP_ACCUM;
                walk_page_m = MMU_TWALK_WRITE;
                walk_lookup = {MMU_TWALK_FETCH_LO_VALID, MMU_TWALK_FETCH_LO_WORD};

                if (MMU_TWALK_DESC_DT == 2'b10) begin
                    if (!walk_lookup[32] || walk_lookup[1:0] != 2'b01) begin
                        walk_fault = 1'b1;
                        walk_result = mmu_twalk_fault_result(MMU_TWALK_LOGICAL, walk_wp_accum, MMU_TWALK_WRITE);
                    end else begin
                        walk_wp_accum = walk_wp_accum || walk_lookup[2];
                        walk_page_m = walk_lookup[4] || MMU_TWALK_WRITE;
                        walk_result = mmu_twalk_page_result(
                            {walk_lookup[31:8], 8'h00} + (MMU_TWALK_LOGICAL & MMU_TWALK_OFFSET_MASK_CUR),
                            1'b0,
                            walk_wp_accum,
                            walk_page_m
                        );
                    end
                end else begin
                    if (!walk_lookup[32] || !MMU_TWALK_SHADOW_LOOKUP[32] || walk_lookup[1:0] != 2'b01) begin
                        walk_fault = 1'b1;
                        walk_result = mmu_twalk_fault_result(MMU_TWALK_LOGICAL, walk_wp_accum, MMU_TWALK_WRITE);
                    end else begin
                        walk_wp_accum = walk_wp_accum || walk_lookup[2];
                        if (!MMU_TWALK_FC[2] && walk_lookup[8]) begin
                            walk_fault = 1'b1;
                            walk_result = mmu_twalk_fault_result(MMU_TWALK_LOGICAL, walk_wp_accum, MMU_TWALK_WRITE);
                        end else begin
                            walk_page_m = walk_lookup[4] || MMU_TWALK_WRITE;
                            walk_result = mmu_twalk_page_result(
                                {MMU_TWALK_SHADOW_LOOKUP[31:8], 8'h00} + (MMU_TWALK_LOGICAL & MMU_TWALK_OFFSET_MASK_CUR),
                                1'b0,
                                walk_wp_accum,
                                walk_page_m
                            );
                        end
                    end
                end

                if (MMU_TWALK_WRITE && walk_wp_accum)
                    walk_result[32] = 1'b1;
                MMU_TWALK_RESULT <= walk_result;
                MMU_TWALK_VALID <= 1'b1;
                MMU_TWALK_BUSY <= 1'b0;
                MMU_TWALK_STATE_INT <= MMU_TWALK_ST_IDLE;
            end

            default: begin
                MMU_TWALK_BUSY <= 1'b0;
                MMU_TWALK_VALID <= 1'b0;
                MMU_TWALK_STATE_INT <= MMU_TWALK_ST_IDLE;
            end
        endcase
    end
end

// Descriptor-shadow update model used by MMU table-walk lookups.
always_ff @(posedge CLK) begin : mmu_desc_shadow_update
    if (RESET_CPU) begin
        MMU_DESC_SHADOW_PENDING <= 1'b0;
        MMU_DESC_SHADOW_PENDING_ADDR <= 32'h0;
        MMU_DESC_SHADOW_PENDING_WR <= 1'b0;
    end else begin
        if (!BUS_BSY && (DATA_RD_BUS || DATA_WR) && !MMU_RUNTIME_FAULT && !MMU_RUNTIME_STALL) begin
            MMU_DESC_SHADOW_PENDING <= 1'b1;
            MMU_DESC_SHADOW_PENDING_ADDR <= ADR_P_PHYS;
            MMU_DESC_SHADOW_PENDING_WR <= DATA_WR;
        end

        if (DATA_RDY_BUSIF_CORE && MMU_DESC_SHADOW_PENDING) begin
            MMU_DESC_SHADOW_PENDING <= 1'b0;
        end else if (!BUS_BSY && MMU_RUNTIME_FAULT) begin
            MMU_DESC_SHADOW_PENDING <= 1'b0;
        end
    end
end

assign MMU_DESC_SHADOW_WR_EN = DATA_RDY_BUSIF_CORE && MMU_DESC_SHADOW_PENDING;
assign MMU_DESC_SHADOW_WR_ADDR = {MMU_DESC_SHADOW_PENDING_ADDR[31:2], 2'b00};
assign MMU_DESC_SHADOW_WR_DATA = MMU_DESC_SHADOW_PENDING_WR ? DATA_OUT : DATA_TO_CORE_BUSIF;

// Core-visible bus requests: direct when idle, held via latches while BUS_BSY.
assign RD_REQ = !BUS_BSY ? ((DATA_RD_BUS && !MMU_RUNTIME_FAULT && !MMU_RUNTIME_STALL) || BURST_PREFETCH_DATA_REQ) : RD_REQ_I;
assign WR_REQ = !BUS_BSY ? (DATA_WR && !MMU_RUNTIME_FAULT && !MMU_RUNTIME_STALL) : WR_REQ_I;
assign OPCODE_REQ_CORE = !BUS_BSY ? OPCODE_RD : OPCODE_REQ_I;

assign OPCODE_REQ_CORE_MISS = OPCODE_REQ_CORE && !ICACHE_HIT_NOW;

// On an instruction-cache hit, satisfy the opcode fetch internally.
assign OPCODE_REQ = (OPCODE_REQ_CORE_MISS && !MMU_RUNTIME_FAULT && !MMU_RUNTIME_STALL) || BURST_PREFETCH_OP_REQ;

assign DATA_RD_BUS = DATA_RD && !DCACHE_HIT_NOW;

endmodule
