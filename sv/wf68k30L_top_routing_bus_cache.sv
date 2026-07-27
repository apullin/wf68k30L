(* keep_hierarchy = "yes" *)
module WF68K30L_TOP_ROUTING_BUS_CACHE (
    input  logic        CLK,
    input  logic        RESET_CPU,
    input  logic        BUS_BSY,
    input  logic        BUSY_EXH,

    input  logic        DATA_RD_EXH,
    input  logic        DATA_RD_MAIN,
    input  logic        DATA_WR_EXH,
    input  logic        DATA_WR_MAIN,
    input  logic        OPCODE_RD,
    input  logic        RMC,

    input  logic        MMU_RUNTIME_REQ,
    input  logic        MMU_RUNTIME_FAULT,
    input  logic        MMU_RUNTIME_STALL,
    input  logic [2:0]  MMU_RUNTIME_ATC_FC,

    input  logic [31:0] MMU_TC,
    input  logic [63:0] MMU_SRP,
    input  logic [63:0] MMU_CRP,
    input  logic [31:0] ADR_P,
    input  logic [1:0]  OP_SIZE_BUS,
    input  logic [31:0] DATA_TO_CORE_BUSIF,
    input  logic        DATA_RDY_BUSIF,
    input  logic        DATA_VALID_BUSIF,
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

    input  logic        MMU_PLOAD_START,
    input  logic [2:0]  MMU_PLOAD_FC,
    input  logic [31:0] MMU_PLOAD_LOGICAL,
    input  logic        MMU_PLOAD_WRITE,
    output logic        MMU_PLOAD_DONE,
    output logic [35:0] MMU_PLOAD_RESULT,

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
    output logic        MMU_FAULT_SSW_VALID,
    output logic [8:0]  MMU_FAULT_SSW,
    output logic        BUS_CYCLE_BURST,
    output logic        BUS_CYCLE_BURST_IS_OP,

    // MMU table-search bus master (see the banner block below).
    output logic        MMU_TWALK_BUS_RD,
    output logic        MMU_TWALK_BUS_WR,
    output logic        MMU_TWALK_BUS_ACTIVE,
    output logic [31:0] MMU_TWALK_BUS_ADDR,
    output logic [31:0] MMU_TWALK_BUS_DATA,
    output logic        MMU_TWALK_RMC,
    output logic        BUS_CYCLE_MMU_WALK,

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
    output logic [15:0] MMU_PTEST_WALK_MMUSR,
    output logic [31:0] MMU_PTEST_DESC_ADDR
);

`include "wf68k30L_pkg.svh"
`include "wf68k30L_top_sections/helpers/wf68k30L_top_helpers_mmu_pure.svh"

logic [31:0] MMU_DESC_HIST_ADDR;
logic [31:0] MMU_DESC_HIST_DATA;
logic [31:0] MMU_TWALK_FETCH_LO_WORD;
logic [31:0] MMU_TWALK_FETCH_HI_WORD;
logic        MMU_TWALK_FETCH_LO_VALID;
logic        MMU_TWALK_FETCH_HI_VALID;
logic [31:0] MMU_TWALK_INDIRECT_SHORT_ADDR;
logic [31:0] MMU_TWALK_INDIRECT_LONG_ADDR;
logic        MMU_PTEST_DESC_REQ;
logic [31:0] MMU_PTEST_DESC_FETCH_ADDR;
logic        MMU_DESC_SEARCH_BUSY;
// Declared here rather than in the descriptor-bus block below because the PTEST
// sequencer is instantiated above it and reads both.
logic        MMU_DESC_BUS_DONE;
logic [32:0] MMU_DESC_BUS_LOOKUP;
logic [1:0]  MMU_TWALK_DESC_DT_CUR;
logic [31:0] MMU_TWALK_DESC_PAGE_BASE;
logic [31:0] MMU_TWALK_DESC_TABLE_BASE_NEXT;
logic [31:0] MMU_TWALK_NEXT_LIMIT_INDEX;
logic [31:0] MMU_TWALK_OFFSET_MASK_CUR;
logic        MMU_TWALK_IS_PLOAD;
logic        MMU_PLOAD_PENDING;
logic        OPCODE_REQ_BUS_OK;
logic        BURST_PREFETCH_OP_REQ_NOW;
logic        BURST_PREFETCH_DATA_REQ_NOW;
logic [2:0]  BURST_PREFETCH_OP_WORD_NOW;
logic [1:0]  BURST_PREFETCH_DATA_ENTRY_NOW;
logic [31:0] BURST_PREFETCH_ADDR_NOW;
logic [2:0]  BURST_PREFETCH_FC_NOW;
logic        BURST_PREFETCH_OP_REQ_HELD;
logic        BURST_PREFETCH_DATA_REQ_HELD;
logic [2:0]  BURST_PREFETCH_OP_WORD_HELD;
logic [1:0]  BURST_PREFETCH_DATA_ENTRY_HELD;
logic [31:0] BURST_PREFETCH_ADDR_HELD;
logic [2:0]  BURST_PREFETCH_FC_HELD;

assign MMU_TWALK_INDIRECT_SHORT_ADDR = {MMU_TWALK_DESC_PTR[31:2], 2'b00};
assign MMU_TWALK_INDIRECT_LONG_ADDR = {MMU_TWALK_DESC_PTR[31:3], 3'b000};

// UM 9.5.2: during a table search the U bit of every descriptor encountered is
// set, and for a write access the M bit of the page descriptor is set unless a
// WP bit or a supervisor violation was encountered. U is bit 3, M is bit 4 of
// the descriptor status (UM Figures 9-10..9-14).
function automatic logic [31:0] mmu_desc_history_bits(
    input logic [31:0] desc,
    input logic        set_modified
);
begin
    mmu_desc_history_bits = desc | 32'h0000_0008;
    if (set_modified)
        mmu_desc_history_bits = mmu_desc_history_bits | 32'h0000_0010;
end
endfunction

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
    input logic        page_m,
    input logic        page_ci
);
begin
    mmu_twalk_page_result = 36'h0;
    mmu_twalk_page_result[31:0] = phys_addr;
    mmu_twalk_page_result[32] = fault;
    mmu_twalk_page_result[33] = page_ci;
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

// Every table search -- runtime, PLOAD and PTEST alike -- now reads its
// descriptors from memory over the shared descriptor-bus port below, so the
// snooped descriptor shadow that used to stand in for memory is gone.
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
    .DESC_REQ(MMU_PTEST_DESC_REQ),
    .DESC_ADDR(MMU_PTEST_DESC_FETCH_ADDR),
    .DESC_DONE(MMU_DESC_BUS_DONE),
    .DESC_LOOKUP(MMU_DESC_BUS_LOOKUP),
    .PTEST_BUSY(MMU_PTEST_BUSY),
    .PTEST_READY(MMU_PTEST_READY),
    .PTEST_WALK_MMUSR(MMU_PTEST_WALK_MMUSR),
    .PTEST_DESC_ADDR(MMU_PTEST_DESC_ADDR)
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
    BURST_PREFETCH_OP_REQ_NOW = 1'b0;
    BURST_PREFETCH_DATA_REQ_NOW = 1'b0;
    BURST_PREFETCH_OP_WORD_NOW = 3'b000;
    BURST_PREFETCH_DATA_ENTRY_NOW = 2'b00;
    BURST_PREFETCH_ADDR_NOW = 32'h0000_0000;
    BURST_PREFETCH_FC_NOW = FC_USER_PROG;
    scan_idx = 0;
    icache_found = 1'b0;
    icache_scan_word = ICACHE_BURST_FILL_NEXT_WORD;
    dcache_found = 1'b0;
    dcache_scan_entry = DCACHE_BURST_FILL_NEXT_ENTRY;

    // UM 7.3.7: RMC is asserted for the whole indivisible read-modify-write
    // sequence and no burst filling occurs during it, so no background fill
    // cycle may be injected between its halves either. UM 9.5.2 makes the same
    // demand of a table search, which owns the bus from its first descriptor
    // cycle to its last -- the PTEST search included (UM 9.8).
    if (!BUS_BSY && !DATA_WR && !DATA_RD && !OPCODE_RD && !BUSY_EXH && !RMC &&
        !MMU_DESC_SEARCH_BUSY) begin
        if (ICACHE_BURST_FILL_VALID && ICACHE_BURST_FILL_PENDING != 8'h00) begin
            for (scan_idx = 0; scan_idx < 8; scan_idx = scan_idx + 1) begin
                if (!icache_found) begin
                    icache_scan_word = ICACHE_BURST_FILL_NEXT_WORD + scan_idx[2:0];
                    if (ICACHE_BURST_FILL_PENDING[icache_scan_word]) begin
                        BURST_PREFETCH_OP_WORD_NOW = icache_scan_word;
                        icache_found = 1'b1;
                    end
                end
            end
            BURST_PREFETCH_OP_REQ_NOW = 1'b1;
            BURST_PREFETCH_ADDR_NOW = {
                ICACHE_BURST_FILL_TAG,
                ICACHE_BURST_FILL_LINE,
                BURST_PREFETCH_OP_WORD_NOW,
                1'b0
            };
            BURST_PREFETCH_FC_NOW = ICACHE_BURST_FILL_FC;
        end else if (DCACHE_BURST_FILL_VALID && DCACHE_BURST_FILL_PENDING != 4'h0) begin
            for (scan_idx = 0; scan_idx < 4; scan_idx = scan_idx + 1) begin
                if (!dcache_found) begin
                    dcache_scan_entry = DCACHE_BURST_FILL_NEXT_ENTRY + scan_idx[1:0];
                    if (DCACHE_BURST_FILL_PENDING[dcache_scan_entry]) begin
                        BURST_PREFETCH_DATA_ENTRY_NOW = dcache_scan_entry;
                        dcache_found = 1'b1;
                    end
                end
            end
            BURST_PREFETCH_DATA_REQ_NOW = 1'b1;
            BURST_PREFETCH_ADDR_NOW = {
                DCACHE_BURST_FILL_TAG,
                DCACHE_BURST_FILL_LINE,
                BURST_PREFETCH_DATA_ENTRY_NOW,
                2'b00
            };
            BURST_PREFETCH_FC_NOW = DCACHE_BURST_FILL_FC;
        end
    end
end

// The bus controller samples address, size and function code live for the whole
// cycle, so the selected burst context must be held until the cycle completes
// instead of collapsing when BUS_BSY rises.
always_ff @(posedge CLK) begin : burst_prefetch_hold
    if (RESET_CPU) begin
        BURST_PREFETCH_OP_REQ_HELD <= 1'b0;
        BURST_PREFETCH_DATA_REQ_HELD <= 1'b0;
        BURST_PREFETCH_OP_WORD_HELD <= 3'b000;
        BURST_PREFETCH_DATA_ENTRY_HELD <= 2'b00;
        BURST_PREFETCH_ADDR_HELD <= 32'h0000_0000;
        BURST_PREFETCH_FC_HELD <= FC_USER_PROG;
    end else if (!BUS_BSY) begin
        BURST_PREFETCH_OP_REQ_HELD <= BURST_PREFETCH_OP_REQ_NOW;
        BURST_PREFETCH_DATA_REQ_HELD <= BURST_PREFETCH_DATA_REQ_NOW;
        BURST_PREFETCH_OP_WORD_HELD <= BURST_PREFETCH_OP_WORD_NOW;
        BURST_PREFETCH_DATA_ENTRY_HELD <= BURST_PREFETCH_DATA_ENTRY_NOW;
        BURST_PREFETCH_ADDR_HELD <= BURST_PREFETCH_ADDR_NOW;
        BURST_PREFETCH_FC_HELD <= BURST_PREFETCH_FC_NOW;
    end
end

assign BURST_PREFETCH_OP_REQ = BUS_BSY ? BURST_PREFETCH_OP_REQ_HELD : BURST_PREFETCH_OP_REQ_NOW;
assign BURST_PREFETCH_DATA_REQ = BUS_BSY ? BURST_PREFETCH_DATA_REQ_HELD : BURST_PREFETCH_DATA_REQ_NOW;
assign BURST_PREFETCH_OP_WORD = BUS_BSY ? BURST_PREFETCH_OP_WORD_HELD : BURST_PREFETCH_OP_WORD_NOW;
assign BURST_PREFETCH_DATA_ENTRY = BUS_BSY ? BURST_PREFETCH_DATA_ENTRY_HELD : BURST_PREFETCH_DATA_ENTRY_NOW;
assign BURST_PREFETCH_ADDR = BUS_BSY ? BURST_PREFETCH_ADDR_HELD : BURST_PREFETCH_ADDR_NOW;
assign BURST_PREFETCH_FC = BUS_BSY ? BURST_PREFETCH_FC_HELD : BURST_PREFETCH_FC_NOW;

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
        BUS_CYCLE_MMU_WALK <= 1'b0;
    end else if (!BUS_BSY) begin
        MMU_FAULT_DATA_ACK <= MMU_RUNTIME_FAULT && (DATA_RD_BUS || DATA_WR);
        MMU_FAULT_OPCODE_ACK <= MMU_RUNTIME_FAULT && OPCODE_REQ_CORE_MISS && !DATA_RD_BUS && !DATA_WR;
        RD_REQ_I <= (DATA_RD_BUS && !MMU_RUNTIME_FAULT && !MMU_RUNTIME_STALL && !MMU_TWALK_BUS_LOCK) ||
                    BURST_PREFETCH_DATA_REQ || MMU_TWALK_BUS_RD;
        WR_REQ_I <= (DATA_WR && !MMU_RUNTIME_FAULT && !MMU_RUNTIME_STALL && !MMU_TWALK_BUS_LOCK) ||
                    MMU_TWALK_BUS_WR;
        OPCODE_REQ_I <= (OPCODE_REQ_CORE_MISS && !DATA_RD && !DATA_WR &&
                         !MMU_RUNTIME_FAULT && !MMU_RUNTIME_STALL && !MMU_TWALK_BUS_LOCK) ||
                        BURST_PREFETCH_OP_REQ;
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
        // Descriptor traffic is invisible to the core's ready strobes.
        BUS_CYCLE_MMU_WALK <= MMU_TWALK_BUS_RD || MMU_TWALK_BUS_WR;
    end else if (BUS_BSY) begin
        RD_REQ_I <= 1'b0;
        WR_REQ_I <= 1'b0;
        OPCODE_REQ_I <= 1'b0;
        MMU_FAULT_DATA_ACK <= 1'b0;
        MMU_FAULT_OPCODE_ACK <= 1'b0;
    end
end

// ========================================================================
// Special status word for an MMU translation fault
//
// A translation fault runs no bus cycle, so the bus controller never captures
// fault information for it and the stacked SSW described whichever cycle ran
// last -- with DF clear. UM 8.1.6 makes DF the bit that tells RTE to rerun the
// faulted data cycle, so with it clear RTE restored the pipeline image instead
// and execution resumed past the access that never happened. On the silicon the
// fault is reported on the retried access (UM 8.1.2: "a bus error exception
// occurs when the aborted bus cycle is retried"), which is a real data cycle,
// so DF is set. This builds the equivalent word from the access the MMU
// refused: DF set, RM from RMC, RW from the direction, SIZE as UM Table 8-9
// encodes it, and the access's own function code.
//
// Instruction-fetch faults are left alone: those are reported through the
// stage B/C fault bits, not DF.
// ========================================================================

logic MMU_FAULT_BUSY_EXH_D;

always_ff @(posedge CLK) begin : mmu_fault_ssw_capture
    logic [1:0] ssw_size;
    if (RESET_CPU) begin
        MMU_FAULT_SSW_VALID <= 1'b0;
        MMU_FAULT_SSW <= 9'h000;
        MMU_FAULT_BUSY_EXH_D <= 1'b0;
    end else begin
        MMU_FAULT_BUSY_EXH_D <= BUSY_EXH;
        // A real bus fault carries its own captured status; and once the
        // handler that consumed this word has finished, retire it.
        if ((DATA_RDY_BUSIF && !DATA_VALID_BUSIF) ||
            (MMU_FAULT_BUSY_EXH_D && !BUSY_EXH))
            MMU_FAULT_SSW_VALID <= 1'b0;

        if (!BUS_BSY && MMU_RUNTIME_FAULT && (DATA_RD_BUS || DATA_WR)) begin
            // UM Table 8-9 SIZE encoding; matches the bus controller's own
            // capture for a real fault.
            case (OP_SIZE_BUS)
                LONG:    ssw_size = 2'b10;
                WORD:    ssw_size = 2'b01;
                default: ssw_size = 2'b00; // BYTE
            endcase
            MMU_FAULT_SSW_VALID <= 1'b1;
            MMU_FAULT_SSW <= {1'b1,                 // DF: rerun the data cycle.
                              RMC,                  // RM
                              !DATA_WR,             // RW: 1 = read
                              ssw_size,
                              1'b0,
                              MMU_RUNTIME_ATC_FC};
        end
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
localparam logic [3:0] MMU_TWALK_ST_HIST_WR          = 4'd11;

logic [3:0] MMU_TWALK_STATE_INT;
logic [3:0] MMU_TWALK_HIST_NEXT;

// ========================================================================
// MMU table-search bus master  (BEGIN -- self-contained block)
//
// UM 9.5.2: "The table search procedure uses physical addresses to access the
// translation tables ... The read-modify-write (RMC) signal is asserted on the
// first bus cycle of the search and remains asserted throughout, ensuring that
// the entire table search completes without interruption."
// UM 13 (7.3.7 list): "Table search accesses required for the MMU are always
// read-modify-write cycles to the supervisor data space. During these cycles, a
// write does not occur unless a descriptor is updated. No data is internally
// cached for table search accesses since the MMU uses physical addresses to
// access the tables."
//
// This port injects a sequencer's descriptor reads and descriptor updates into
// the same request path BURST_PREFETCH_DATA_REQ uses: the request is raised
// while the bus is idle, the address/FC/size/write-data overrides are held for
// the whole cycle, and the response is hidden from the core's ready strobes via
// BUS_CYCLE_MMU_WALK. Descriptor addresses are already physical, so they bypass
// translation entirely.
//
// Exclusivity against the core: while MMU_DESC_SEARCH_BUSY every other
// requester (core data read and write, core opcode fetch, background burst
// fill) is held off, so the only cycle that can start once a search is armed is
// that search's own. A cycle already in flight when the search starts is not
// disturbed: the port only drives after the bus has been idle for two
// consecutive cycles, which also skips the single-cycle BUS_IDLE window a
// BERR+HALT retry passes through.
//
// ---- Two clients, one port ----
//
// UM 9.8 makes PTEST perform a table search too, so the port has two clients:
// the runtime/PLOAD walk sequencer and the PTEST sequencer. A search is a
// multi-cycle sequence of bus cycles that UM 9.5.2 requires to run
// uninterrupted, so the two must never interleave. They are made mutually
// exclusive at the level of whole searches, not individual cycles:
//
//   * PTEST may not begin while a walk is in flight. That gate is on
//     MMU_PTEST_START (wf68k30L_top_cache_mmu_state.svh).
//   * A walk may not begin while a PTEST search is in flight, and loses a
//     same-cycle tie to a starting PTEST. That gate is on the walk
//     sequencer's idle state below.
//
// Those two rules make !(MMU_PTEST_BUSY && MMU_TWALK_BUSY) inductive: from a
// state where at most one is busy, the only transition that could set both is a
// simultaneous start, and the tie-break resolves it. So MMU_PTEST_DESC_REQ and
// the walk's request are never both asserted and the mux below needs no
// round-robin.
//
// Neither client can starve the other. Both starts are levels, not pulses --
// PTEST's is (ALU_BSY && ALU_REQ) while the instruction is held in EXECUTE, the
// walk's is a pending access re-evaluated every cycle and, for PLOAD, a latched
// request -- so a deferred start is retried, never lost. And each waits only on
// the other's *completion*, which depends on the bus alone: every descriptor
// cycle terminates (DSACK/STERM/BERR) and a search runs a bounded number of
// them (at most five levels plus the indirect leg). While a PTEST search is in
// flight the lock also stops any core bus cycle from completing, so the pending
// access that could ask for a walk cannot change and can trigger at most one
// walk -- which is why a PTEST search waits for at most one walk, not a stream
// of them.
// ========================================================================

localparam logic [1:0] MMU_DESC_BUS_ST_IDLE = 2'd0;
localparam logic [1:0] MMU_DESC_BUS_ST_ARM  = 2'd1;
localparam logic [1:0] MMU_DESC_BUS_ST_RUN  = 2'd2;
localparam logic [1:0] MMU_DESC_BUS_ST_DONE = 2'd3;

logic [1:0]  MMU_DESC_BUS_STATE;
logic        MMU_DESC_BUS_IDLE_D;
logic        MMU_DESC_BUS_REQ;
logic        MMU_DESC_BUS_REQ_WR;
logic [31:0] MMU_DESC_BUS_REQ_ADDR;
logic [31:0] MMU_DESC_BUS_REQ_DATA;
logic        MMU_DESC_BUS_HOLD_WR;
logic        MMU_DESC_BUS_DRIVE;
logic        MMU_DESC_BUS_ERR;
logic [31:0] MMU_DESC_BUS_RD_DATA;
logic        MMU_TWALK_BUS_LOCK;
logic        MMU_TWALK_BUS_LOCK_HELD;
logic        MMU_TWALK_RMC_R;

// A search is in flight, whichever sequencer owns it.
assign MMU_DESC_SEARCH_BUSY = MMU_TWALK_BUSY || MMU_PTEST_BUSY;

// The requesting state of the owning sequencer names the descriptor access.
// PTEST only ever reads, so it has no write leg here.
always_comb begin : mmu_desc_bus_request
    MMU_DESC_BUS_REQ = 1'b0;
    MMU_DESC_BUS_REQ_WR = 1'b0;
    MMU_DESC_BUS_REQ_ADDR = 32'h0000_0000;
    MMU_DESC_BUS_REQ_DATA = 32'h0000_0000;
    if (MMU_PTEST_DESC_REQ) begin
        MMU_DESC_BUS_REQ = 1'b1;
        MMU_DESC_BUS_REQ_ADDR = MMU_PTEST_DESC_FETCH_ADDR;
    end else begin
        case (MMU_TWALK_STATE_INT)
            MMU_TWALK_ST_FETCH_LO_REQ: begin
                MMU_DESC_BUS_REQ = 1'b1;
                MMU_DESC_BUS_REQ_ADDR = MMU_TWALK_DESC_ADDR;
            end
            MMU_TWALK_ST_FETCH_HI_REQ: begin
                MMU_DESC_BUS_REQ = 1'b1;
                MMU_DESC_BUS_REQ_ADDR = MMU_TWALK_DESC_ADDR + 32'd4;
            end
            MMU_TWALK_ST_INDIRECT_LO_REQ: begin
                MMU_DESC_BUS_REQ = 1'b1;
                MMU_DESC_BUS_REQ_ADDR = (MMU_TWALK_DESC_DT == 2'b10) ?
                                        MMU_TWALK_INDIRECT_SHORT_ADDR :
                                        MMU_TWALK_INDIRECT_LONG_ADDR;
            end
            MMU_TWALK_ST_INDIRECT_HI_REQ: begin
                MMU_DESC_BUS_REQ = 1'b1;
                MMU_DESC_BUS_REQ_ADDR = MMU_TWALK_INDIRECT_LONG_ADDR + 32'd4;
            end
            MMU_TWALK_ST_HIST_WR: begin
                MMU_DESC_BUS_REQ = 1'b1;
                MMU_DESC_BUS_REQ_WR = 1'b1;
                MMU_DESC_BUS_REQ_ADDR = MMU_DESC_HIST_ADDR;
                MMU_DESC_BUS_REQ_DATA = MMU_DESC_HIST_DATA;
            end
            default: begin end
        endcase
    end
end

assign MMU_DESC_BUS_DRIVE = (MMU_DESC_BUS_STATE == MMU_DESC_BUS_ST_ARM) &&
                            !BUS_BSY && MMU_DESC_BUS_IDLE_D;
assign MMU_TWALK_BUS_RD = MMU_DESC_BUS_DRIVE && !MMU_DESC_BUS_HOLD_WR;
assign MMU_TWALK_BUS_WR = MMU_DESC_BUS_DRIVE && MMU_DESC_BUS_HOLD_WR;
// The bus controller samples address, function code, size and write data live,
// so they stay overridden for the whole cycle, including a BERR+HALT rerun.
assign MMU_TWALK_BUS_ACTIVE = MMU_DESC_BUS_DRIVE ||
                              (MMU_DESC_BUS_STATE == MMU_DESC_BUS_ST_RUN);
assign MMU_DESC_BUS_DONE = (MMU_DESC_BUS_STATE == MMU_DESC_BUS_ST_DONE);
assign MMU_DESC_BUS_LOOKUP = {!MMU_DESC_BUS_ERR, MMU_DESC_BUS_RD_DATA};
// Held so a cycle that started before the search armed keeps its own
// qualification for its whole duration (the bus controller re-reads OPCODE_REQ
// in START_CYCLE).
assign MMU_TWALK_BUS_LOCK = BUS_BSY ? MMU_TWALK_BUS_LOCK_HELD : MMU_DESC_SEARCH_BUSY;
// UM 9.5.2: "The read-modify-write (RMC) signal is asserted on the first bus
// cycle of the search and remains asserted throughout, ensuring that the entire
// table search completes without interruption." That is a property of the table
// search procedure, which UM 9.8 has PTEST perform, so it is held across a
// PTEST search too: without it another master could rewrite the tables between
// two of PTEST's descriptor fetches and PTEST would report a mixture of the
// before and after trees. PTEST never writes a descriptor, so the "modify" half
// stays unused -- RMC here is about indivisibility, not about updating.
assign MMU_TWALK_RMC = MMU_TWALK_RMC_R || MMU_DESC_BUS_DRIVE;

always_ff @(posedge CLK) begin : mmu_desc_bus_port
    if (RESET_CPU) begin
        MMU_DESC_BUS_STATE <= MMU_DESC_BUS_ST_IDLE;
        MMU_DESC_BUS_IDLE_D <= 1'b0;
        MMU_DESC_BUS_HOLD_WR <= 1'b0;
        MMU_TWALK_BUS_ADDR <= 32'h0000_0000;
        MMU_TWALK_BUS_DATA <= 32'h0000_0000;
        MMU_DESC_BUS_RD_DATA <= 32'h0000_0000;
        MMU_DESC_BUS_ERR <= 1'b0;
        MMU_TWALK_BUS_LOCK_HELD <= 1'b0;
        MMU_TWALK_RMC_R <= 1'b0;
    end else begin
        MMU_DESC_BUS_IDLE_D <= !BUS_BSY;
        if (!BUS_BSY)
            MMU_TWALK_BUS_LOCK_HELD <= MMU_DESC_SEARCH_BUSY;

        if (MMU_DESC_BUS_DRIVE)
            MMU_TWALK_RMC_R <= 1'b1;
        else if (!MMU_DESC_SEARCH_BUSY)
            MMU_TWALK_RMC_R <= 1'b0;

        case (MMU_DESC_BUS_STATE)
            MMU_DESC_BUS_ST_IDLE: begin
                if (MMU_DESC_BUS_REQ) begin
                    MMU_DESC_BUS_HOLD_WR <= MMU_DESC_BUS_REQ_WR;
                    MMU_TWALK_BUS_ADDR <= {MMU_DESC_BUS_REQ_ADDR[31:2], 2'b00};
                    MMU_TWALK_BUS_DATA <= MMU_DESC_BUS_REQ_DATA;
                    MMU_DESC_BUS_STATE <= MMU_DESC_BUS_ST_ARM;
                end
            end
            MMU_DESC_BUS_ST_ARM: begin
                if (!MMU_DESC_BUS_REQ)
                    MMU_DESC_BUS_STATE <= MMU_DESC_BUS_ST_IDLE; // Requester withdrew.
                else if (MMU_DESC_BUS_DRIVE)
                    MMU_DESC_BUS_STATE <= MMU_DESC_BUS_ST_RUN;
            end
            MMU_DESC_BUS_ST_RUN: begin
                if (DATA_RDY_BUSIF) begin
                    MMU_DESC_BUS_RD_DATA <= DATA_TO_CORE_BUSIF;
                    // UM 8.1.2: BERR during a table search bus cycle faults the
                    // original access; the walk turns it into a fault result.
                    MMU_DESC_BUS_ERR <= !DATA_VALID_BUSIF;
                    MMU_DESC_BUS_STATE <= MMU_DESC_BUS_ST_DONE;
                end else if (!BUS_BSY && MMU_DESC_BUS_IDLE_D) begin
                    // The cycle never reached the bus; ask again.
                    MMU_DESC_BUS_STATE <= MMU_DESC_BUS_ST_ARM;
                end
            end
            default: begin
                MMU_DESC_BUS_STATE <= MMU_DESC_BUS_ST_IDLE;
            end
        endcase
    end
end

// ========================================================================
// MMU table-search bus master  (END)
// ========================================================================

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
        MMU_TWALK_ST_HIST_WR: MMU_TWALK_STATE = 3'd7;
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
    logic        walk_is_pload;
    logic [2:0]  walk_start_fc;
    logic [31:0] walk_start_logical;
    logic        walk_start_write;
    logic [31:0] walk_hist_data;
    logic        walk_hist_req;
    logic        walk_complete;
    logic [3:0]  walk_next_state;
    if (RESET_CPU) begin
        MMU_TWALK_HIST_NEXT <= MMU_TWALK_ST_IDLE;
        MMU_TWALK_IS_PLOAD <= 1'b0;
        MMU_PLOAD_PENDING <= 1'b0;
        MMU_DESC_HIST_ADDR <= 32'h0;
        MMU_DESC_HIST_DATA <= 32'h0;
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
    end else if (BUS_BSY && !MMU_TWALK_IS_PLOAD && !MMU_TWALK_BUSY) begin
        // A runtime walk belongs to the pending access, so it is abandoned once
        // a bus cycle it does not own starts. A search in flight owns every
        // cycle that can start (see the bus-master banner above), and a PLOAD
        // walk carries its own latched operands, so both survive BUS_BSY.
        MMU_TWALK_BUSY <= 1'b0;
        MMU_TWALK_VALID <= 1'b0;
        MMU_TWALK_STATE_INT <= MMU_TWALK_ST_IDLE;
        MMU_TWALK_FETCH_LO_VALID <= 1'b0;
        MMU_TWALK_FETCH_HI_VALID <= 1'b0;
        if (MMU_PLOAD_START)
            MMU_PLOAD_PENDING <= 1'b1;
    end else begin
        if (MMU_PLOAD_START)
            MMU_PLOAD_PENDING <= 1'b1;
        case (MMU_TWALK_STATE_INT)
            MMU_TWALK_ST_IDLE: begin
                MMU_TWALK_BUSY <= 1'b0;
                if (!MMU_RUNTIME_REQ)
                    MMU_TWALK_VALID <= 1'b0;
                if (MMU_TWALK_IS_PLOAD && !MMU_TWALK_BUSY)
                    MMU_TWALK_IS_PLOAD <= 1'b0;

                walk_is_pload = !MMU_TWALK_START && (MMU_PLOAD_START || MMU_PLOAD_PENDING);
                // A PTEST search owns the shared descriptor port for its whole
                // duration, and wins a same-cycle tie because MMU_PTEST_BUSY
                // does not rise until the edge after MMU_PTEST_START. Both
                // requests here are levels (a pending access is re-evaluated
                // every cycle, a PLOAD request is latched), so deferring one is
                // not dropping it. See the descriptor-bus banner above.
                if ((MMU_TWALK_START || walk_is_pload) && !MMU_TWALK_BUSY && !BUS_BSY &&
                    !MMU_PTEST_BUSY && !MMU_PTEST_START) begin
                    walk_start_fc = walk_is_pload ? MMU_PLOAD_FC : MMU_RUNTIME_ATC_FC;
                    walk_start_logical = walk_is_pload ? MMU_PLOAD_LOGICAL : ADR_P;
                    walk_start_write = walk_is_pload ? MMU_PLOAD_WRITE : DATA_WR;
                    root_ptr = (MMU_TC[25] && walk_start_fc[2]) ? MMU_SRP : MMU_CRP;

                    MMU_TWALK_IS_PLOAD <= walk_is_pload;
                    if (walk_is_pload)
                        MMU_PLOAD_PENDING <= 1'b0;
                    MMU_TWALK_FC <= walk_start_fc;
                    MMU_TWALK_LOGICAL <= walk_start_logical;
                    MMU_TWALK_WRITE <= walk_start_write;
                    MMU_TWALK_TC <= MMU_TC;
                    MMU_TWALK_ROOT_LIMIT <= root_ptr[62:48];
                    MMU_TWALK_ROOT_LIMIT_LOWER <= root_ptr[63];
                    MMU_TWALK_TABLE_BASE <= {root_ptr[31:4], 4'b0000};
                    MMU_TWALK_DESC_SIZE <= (root_ptr[33:32] == 2'b11) ? 4'd8 : 4'd4;
                    MMU_TWALK_DESC_ADDR <= 32'h0;
                    MMU_TWALK_PAGE_BASE <= walk_start_logical;
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

                    if (root_ptr[33:32] == 2'b01) begin
                        // DT=1 root: the table address field is a page offset.
                        // Its limit applies only when FCL is clear (UM 9.5.1.2).
                        walk_index = MMU_TC[24] ? {29'h0, walk_start_fc} :
                                     mmu_index_extract(walk_start_logical, MMU_TC[19:16], 6'd0, MMU_TC[15:12]);
                        walk_fault = !MMU_TC[24] &&
                                     mmu_limit_violation(root_ptr[63], root_ptr[62:48], walk_index);
                        if (walk_fault)
                            walk_result = mmu_twalk_fault_result(walk_start_logical, 1'b0, walk_start_write);
                        else
                            walk_result = mmu_twalk_page_result(
                                walk_start_logical + {root_ptr[31:4], 4'b0000},
                                1'b0,
                                1'b0,
                                walk_start_write,
                                1'b0
                            );
                        MMU_TWALK_RESULT <= walk_result;
                        MMU_TWALK_VALID <= 1'b1;
                        MMU_TWALK_BUSY <= 1'b0;
                        MMU_TWALK_STATE_INT <= MMU_TWALK_ST_IDLE;
                    end else if (root_ptr[33:32] != 2'b10 && root_ptr[33:32] != 2'b11) begin
                        walk_result = mmu_twalk_fault_result(walk_start_logical, 1'b0, walk_start_write);
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
                if (MMU_DESC_BUS_DONE)
                    MMU_TWALK_STATE_INT <= MMU_TWALK_ST_FETCH_LO_RESP;
            end

            MMU_TWALK_ST_FETCH_LO_RESP: begin
                walk_lookup = MMU_DESC_BUS_LOOKUP;
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
                if (MMU_DESC_BUS_DONE)
                    MMU_TWALK_STATE_INT <= MMU_TWALK_ST_FETCH_HI_RESP;
            end

            MMU_TWALK_ST_FETCH_HI_RESP: begin
                walk_lookup = MMU_DESC_BUS_LOOKUP;
                MMU_TWALK_FETCH_HI_WORD <= walk_lookup[31:0];
                MMU_TWALK_FETCH_HI_VALID <= walk_lookup[32];
                MMU_TWALK_STATE_INT <= MMU_TWALK_ST_EVAL;
            end

            MMU_TWALK_ST_EVAL: begin
                walk_fault = 1'b0;
                walk_wp_accum = MMU_TWALK_WP_ACCUM;
                walk_page_m = MMU_TWALK_WRITE;
                walk_hist_req = 1'b0;
                walk_complete = 1'b0;
                walk_next_state = MMU_TWALK_ST_IDLE;

                walk_desc = MMU_TWALK_FETCH_LO_WORD;
                walk_desc_dt = MMU_TWALK_DESC_DT_CUR;
                next_levels = MMU_TWALK_LEVELS + 3'd1;
                walk_wp_accum = walk_wp_accum || walk_desc[2];

                if (!MMU_TWALK_FETCH_LO_VALID || !MMU_TWALK_FETCH_HI_VALID)
                    walk_fault = 1'b1;
                if (!walk_fault && MMU_TWALK_DESC_SIZE == 4'd8 && !MMU_TWALK_FC[2] && walk_desc[8])
                    walk_fault = 1'b1;

                // UM 9.5.2/9.6.1: U is set in every descriptor encountered, and
                // M in the page descriptor of a write, except after a supervisor
                // violation. The update is a real memory write (UM 13), so it is
                // sequenced before the search advances or reports.
                if (!walk_fault) begin
                    walk_hist_data = mmu_desc_history_bits(
                        walk_desc,
                        (walk_desc_dt == 2'b01) && MMU_TWALK_WRITE && !walk_wp_accum
                    );
                    if (walk_hist_data != walk_desc) begin
                        walk_hist_req = 1'b1;
                        MMU_DESC_HIST_ADDR <= MMU_TWALK_DESC_ADDR;
                        MMU_DESC_HIST_DATA <= walk_hist_data;
                    end
                end

                if (!walk_fault) begin
                    case (walk_desc_dt)
                        2'b00: begin
                            walk_result = mmu_twalk_fault_result(MMU_TWALK_LOGICAL, walk_wp_accum, MMU_TWALK_WRITE);
                            MMU_TWALK_RESULT <= walk_result;
                            walk_complete = 1'b1;
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
                                walk_page_m,
                                walk_desc[6]
                            );
                            if (MMU_TWALK_WRITE && walk_wp_accum)
                                walk_result[32] = 1'b1;
                            MMU_TWALK_RESULT <= walk_result;
                            walk_complete = 1'b1;
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
                                    walk_complete = 1'b1;
                                end else begin
                                    MMU_TWALK_TABLE_BASE <= MMU_TWALK_DESC_TABLE_BASE_NEXT;
                                    MMU_TWALK_DESC_SIZE <= (walk_desc_dt == 2'b10) ? 4'd4 : 4'd8;
                                    MMU_TWALK_WP_ACCUM <= walk_wp_accum;
                                    MMU_TWALK_LEVELS <= next_levels;
                                    walk_next_state = MMU_TWALK_ST_STEP;
                                end
                            end else begin
                                MMU_TWALK_DESC_PTR <= (MMU_TWALK_DESC_SIZE == 4'd8) ? MMU_TWALK_FETCH_HI_WORD : walk_desc;
                                MMU_TWALK_DESC_DT <= walk_desc_dt;
                                MMU_TWALK_WP_ACCUM <= walk_wp_accum;
                                MMU_TWALK_LEVELS <= next_levels;
                                walk_next_state = MMU_TWALK_ST_INDIRECT_LO_REQ;
                            end
                        end
                    endcase
                end else begin
                    walk_result = mmu_twalk_fault_result(MMU_TWALK_LOGICAL, walk_wp_accum, MMU_TWALK_WRITE);
                    MMU_TWALK_RESULT <= walk_result;
                    walk_complete = 1'b1;
                end

                if (walk_complete)
                    walk_next_state = MMU_TWALK_ST_IDLE;

                if (walk_hist_req) begin
                    MMU_TWALK_HIST_NEXT <= walk_next_state;
                    MMU_TWALK_STATE_INT <= MMU_TWALK_ST_HIST_WR;
                end else begin
                    MMU_TWALK_STATE_INT <= walk_next_state;
                    if (walk_complete) begin
                        MMU_TWALK_VALID <= 1'b1;
                        MMU_TWALK_BUSY <= 1'b0;
                    end
                end
            end

            // A descriptor update is an extended read-modify-write leg of the
            // search: the write must land in memory before the page it describes
            // may be accessed (UM 9.6.1).
            MMU_TWALK_ST_HIST_WR: begin
                if (MMU_DESC_BUS_DONE) begin
                    if (MMU_DESC_BUS_ERR) begin
                        walk_result = mmu_twalk_fault_result(
                            MMU_TWALK_LOGICAL,
                            MMU_TWALK_WP_ACCUM,
                            MMU_TWALK_WRITE
                        );
                        MMU_TWALK_RESULT <= walk_result;
                        MMU_TWALK_VALID <= 1'b1;
                        MMU_TWALK_BUSY <= 1'b0;
                        MMU_TWALK_STATE_INT <= MMU_TWALK_ST_IDLE;
                    end else begin
                        // The update landed in memory, which is the only copy
                        // there is: every search reads descriptors from the bus,
                        // so a PTEST run afterwards sees the new value by
                        // reading it, not by being told about it.
                        MMU_TWALK_STATE_INT <= MMU_TWALK_HIST_NEXT;
                        if (MMU_TWALK_HIST_NEXT == MMU_TWALK_ST_IDLE) begin
                            MMU_TWALK_VALID <= 1'b1;
                            MMU_TWALK_BUSY <= 1'b0;
                        end
                    end
                end
            end

            MMU_TWALK_ST_INDIRECT_LO_REQ: begin
                if (MMU_DESC_BUS_DONE) begin
                    walk_lookup = MMU_DESC_BUS_LOOKUP;
                    MMU_TWALK_FETCH_LO_WORD <= walk_lookup[31:0];
                    MMU_TWALK_FETCH_LO_VALID <= walk_lookup[32];
                    MMU_TWALK_STATE_INT <= MMU_TWALK_ST_INDIRECT_LO_RESP;
                end
            end

            MMU_TWALK_ST_INDIRECT_LO_RESP: begin
                if (MMU_TWALK_DESC_DT == 2'b10)
                    MMU_TWALK_STATE_INT <= MMU_TWALK_ST_INDIRECT_HI_RESP;
                else
                    MMU_TWALK_STATE_INT <= MMU_TWALK_ST_INDIRECT_HI_REQ;
            end

            MMU_TWALK_ST_INDIRECT_HI_REQ: begin
                if (MMU_DESC_BUS_DONE)
                    MMU_TWALK_STATE_INT <= MMU_TWALK_ST_INDIRECT_HI_RESP;
            end

            MMU_TWALK_ST_INDIRECT_HI_RESP: begin
                walk_fault = 1'b0;
                walk_wp_accum = MMU_TWALK_WP_ACCUM;
                walk_page_m = MMU_TWALK_WRITE;
                walk_hist_req = 1'b0;
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
                            walk_page_m,
                            walk_lookup[6]
                        );
                    end
                end else begin
                    if (!walk_lookup[32] || !MMU_DESC_BUS_LOOKUP[32] || walk_lookup[1:0] != 2'b01) begin
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
                                {MMU_DESC_BUS_LOOKUP[31:8], 8'h00} + (MMU_TWALK_LOGICAL & MMU_TWALK_OFFSET_MASK_CUR),
                                1'b0,
                                walk_wp_accum,
                                walk_page_m,
                                walk_lookup[6]
                            );
                        end
                    end
                end

                if (MMU_TWALK_WRITE && walk_wp_accum)
                    walk_result[32] = 1'b1;
                if (!walk_fault) begin
                    walk_hist_data = mmu_desc_history_bits(
                        walk_lookup[31:0],
                        MMU_TWALK_WRITE && !walk_wp_accum
                    );
                    if (walk_hist_data != walk_lookup[31:0]) begin
                        walk_hist_req = 1'b1;
                        MMU_DESC_HIST_ADDR <= (MMU_TWALK_DESC_DT == 2'b10) ?
                                              MMU_TWALK_INDIRECT_SHORT_ADDR :
                                              MMU_TWALK_INDIRECT_LONG_ADDR;
                        MMU_DESC_HIST_DATA <= walk_hist_data;
                    end
                end
                MMU_TWALK_RESULT <= walk_result;
                if (walk_hist_req) begin
                    MMU_TWALK_HIST_NEXT <= MMU_TWALK_ST_IDLE;
                    MMU_TWALK_STATE_INT <= MMU_TWALK_ST_HIST_WR;
                end else begin
                    MMU_TWALK_VALID <= 1'b1;
                    MMU_TWALK_BUSY <= 1'b0;
                    MMU_TWALK_STATE_INT <= MMU_TWALK_ST_IDLE;
                end
            end

            default: begin
                MMU_TWALK_BUSY <= 1'b0;
                MMU_TWALK_VALID <= 1'b0;
                MMU_TWALK_STATE_INT <= MMU_TWALK_ST_IDLE;
            end
        endcase
    end
end

// PLOAD collects the result of the walk it started; the sequencer writes
// MMU_TWALK_RESULT in the same cycle it drops MMU_TWALK_BUSY.
always_ff @(posedge CLK) begin : mmu_pload_result_capture
    if (RESET_CPU) begin
        MMU_PLOAD_DONE <= 1'b0;
        MMU_PLOAD_RESULT <= 36'h0;
    end else begin
        MMU_PLOAD_DONE <= MMU_TWALK_IS_PLOAD && !MMU_TWALK_BUSY;
        if (MMU_TWALK_IS_PLOAD && !MMU_TWALK_BUSY)
            MMU_PLOAD_RESULT <= MMU_TWALK_RESULT;
    end
end

// Core-visible bus requests: direct when idle, held via latches while BUS_BSY.
// A table search in flight owns the bus (UM 9.5.2), so core traffic is held off
// and the search's own descriptor cycles are injected instead.
assign RD_REQ = !BUS_BSY ? ((DATA_RD_BUS && !MMU_RUNTIME_FAULT && !MMU_RUNTIME_STALL && !MMU_TWALK_BUS_LOCK) ||
                            BURST_PREFETCH_DATA_REQ || MMU_TWALK_BUS_RD) : RD_REQ_I;
assign WR_REQ = !BUS_BSY ? ((DATA_WR && !MMU_RUNTIME_FAULT && !MMU_RUNTIME_STALL && !MMU_TWALK_BUS_LOCK) ||
                            MMU_TWALK_BUS_WR) : WR_REQ_I;
assign OPCODE_REQ_CORE = !BUS_BSY ? OPCODE_RD : OPCODE_REQ_I;

assign OPCODE_REQ_CORE_MISS = OPCODE_REQ_CORE && !ICACHE_HIT_NOW;

// ADR_P presents the data address for as long as the core asserts DATA_RD or
// DATA_WR, even when no data bus cycle results (cache hit), so an opcode fetch
// started in that window would run at the data address. While BUS_BSY the
// request comes from the held latch, which was already qualified.
assign OPCODE_REQ_BUS_OK = BUS_BSY || (!DATA_RD && !DATA_WR);

// On an instruction-cache hit, satisfy the opcode fetch internally.
assign OPCODE_REQ = (OPCODE_REQ_CORE_MISS && OPCODE_REQ_BUS_OK &&
                     !MMU_RUNTIME_FAULT && !MMU_RUNTIME_STALL && !MMU_TWALK_BUS_LOCK) ||
                    BURST_PREFETCH_OP_REQ;

assign DATA_RD_BUS = DATA_RD && !DCACHE_HIT_NOW;

endmodule
