// Bounded check on the state that holds a runtime access back while its table
// search runs. The one-shot MMU_WALK_DELAY_ARMED latch this harness used to
// model was replaced by the walk sequencer inside
// WF68K30L_TOP_ROUTING_BUS_CACHE, so the arm state is now MMU_TWALK_BUSY /
// MMU_TWALK_VALID / MMU_TWALK_STATE. Both that sequencer and the producer of
// MMU_TWALK_START (WF68K30L_TOP_ROUTING_MMU_TRANSLATE) are real RTL, wired to
// each other exactly as sv/wf68k30L_top_sections/routing/
// wf68k30L_top_routing_cache_mmu_modules.svh wires them; everything else is
// free apart from MMU_PTEST_START, noted where it is driven.
//
// It also checks the arbitration of the descriptor-bus port the walk shares
// with the PTEST search (UM 9.8), whose sequencer is instantiated inside
// WF68K30L_TOP_ROUTING_BUS_CACHE and so is real RTL here too: single ownership,
// PTEST never writing a descriptor, RMC held across a search, and neither
// client being dropped by the arbitration.
module mmu_walk_delay_state_formal;
    logic CLK = 1'b0;
    always_ff @($global_clock) begin
        CLK <= !CLK;
    end

    logic f_past_valid = 1'b0;
    always_ff @(posedge CLK) begin
        f_past_valid <= 1'b1;
    end

    // Reset is forced for the first cycle so the walk sequencer, the PTEST
    // sequencer and the request latches start from their power-on state
    // instead of a symbolic one, and is free afterwards.
    logic RESET_FREE;
    logic RESET_CPU;
    always_comb RESET_FREE = $anyseq;
    assign RESET_CPU = !f_past_valid || RESET_FREE;

    // ---- free environment ----
    logic        BUS_BSY;
    logic        BUSY_EXH;
    logic        DATA_RD_EXH;
    logic        DATA_RD_MAIN;
    logic        DATA_WR_EXH;
    logic        DATA_WR_MAIN;
    logic        OPCODE_RD;
    logic        RMC;
    logic [31:0] MMU_TC;
    logic [63:0] MMU_SRP;
    logic [63:0] MMU_CRP;
    logic [31:0] MMU_TT0;
    logic [31:0] MMU_TT1;
    logic [2:0]  FC_I;
    logic [31:0] ADR_P;
    logic [31:0] ADR_P_PHYS_LATCH;
    logic [1:0]  OP_SIZE_BUS;
    logic [31:0] DATA_TO_CORE_BUSIF;
    logic        DATA_RDY_BUSIF;
    logic        DATA_VALID_BUSIF;
    logic        ICACHE_HIT_NOW;
    logic        DCACHE_HIT_NOW;
    logic [2:0]  MMU_PTEST_FC;
    logic [31:0] MMU_PTEST_LOGICAL;
    logic [2:0]  MMU_PTEST_LEVEL;
    logic        MMU_PTEST_CONSUME;
    logic        ICACHE_BURST_FILL_VALID;
    logic [3:0]  ICACHE_BURST_FILL_LINE;
    logic [23:0] ICACHE_BURST_FILL_TAG;
    logic [7:0]  ICACHE_BURST_FILL_PENDING;
    logic [2:0]  ICACHE_BURST_FILL_FC;
    logic [2:0]  ICACHE_BURST_FILL_NEXT_WORD;
    logic        DCACHE_BURST_FILL_VALID;
    logic [3:0]  DCACHE_BURST_FILL_LINE;
    logic [23:0] DCACHE_BURST_FILL_TAG;
    logic [3:0]  DCACHE_BURST_FILL_PENDING;
    logic [2:0]  DCACHE_BURST_FILL_FC;
    logic [1:0]  DCACHE_BURST_FILL_NEXT_ENTRY;
    logic [7:0]  MMU_ATC_V_FLAT;
    logic [7:0]  MMU_ATC_B_FLAT;
    logic [7:0]  MMU_ATC_W_FLAT;
    logic [7:0]  MMU_ATC_M_FLAT;
    logic [7:0]  MMU_ATC_CI_FLAT;
    logic [23:0] MMU_ATC_FC_FLAT;
    logic [255:0] MMU_ATC_TAG_FLAT;
    logic [255:0] MMU_ATC_PTAG_FLAT;

    always_comb begin
        BUS_BSY = $anyseq;
        BUSY_EXH = $anyseq;
        DATA_RD_EXH = $anyseq;
        DATA_RD_MAIN = $anyseq;
        DATA_WR_EXH = $anyseq;
        DATA_WR_MAIN = $anyseq;
        OPCODE_RD = $anyseq;
        RMC = $anyseq;
        MMU_TC = $anyseq;
        MMU_SRP = $anyseq;
        MMU_CRP = $anyseq;
        MMU_TT0 = $anyseq;
        MMU_TT1 = $anyseq;
        FC_I = $anyseq;
        ADR_P = $anyseq;
        ADR_P_PHYS_LATCH = $anyseq;
        OP_SIZE_BUS = $anyseq;
        DATA_TO_CORE_BUSIF = $anyseq;
        DATA_RDY_BUSIF = $anyseq;
        DATA_VALID_BUSIF = $anyseq;
        ICACHE_HIT_NOW = $anyseq;
        DCACHE_HIT_NOW = $anyseq;
        MMU_PTEST_FC = $anyseq;
        MMU_PTEST_LOGICAL = $anyseq;
        MMU_PTEST_LEVEL = $anyseq;
        MMU_PTEST_CONSUME = $anyseq;
        ICACHE_BURST_FILL_VALID = $anyseq;
        ICACHE_BURST_FILL_LINE = $anyseq;
        ICACHE_BURST_FILL_TAG = $anyseq;
        ICACHE_BURST_FILL_PENDING = $anyseq;
        ICACHE_BURST_FILL_FC = $anyseq;
        ICACHE_BURST_FILL_NEXT_WORD = $anyseq;
        DCACHE_BURST_FILL_VALID = $anyseq;
        DCACHE_BURST_FILL_LINE = $anyseq;
        DCACHE_BURST_FILL_TAG = $anyseq;
        DCACHE_BURST_FILL_PENDING = $anyseq;
        DCACHE_BURST_FILL_FC = $anyseq;
        DCACHE_BURST_FILL_NEXT_ENTRY = $anyseq;
        MMU_ATC_V_FLAT = $anyseq;
        MMU_ATC_B_FLAT = $anyseq;
        MMU_ATC_W_FLAT = $anyseq;
        MMU_ATC_M_FLAT = $anyseq;
        MMU_ATC_CI_FLAT = $anyseq;
        MMU_ATC_FC_FLAT = $anyseq;
        MMU_ATC_TAG_FLAT = $anyseq;
        MMU_ATC_PTAG_FLAT = $anyseq;
    end

    // MMU_PTEST_START is not free. sv/wf68k30L_top_sections/
    // wf68k30L_top_cache_mmu_state.svh gates it on !MMU_TWALK_BUSY, and that
    // term is one half of the descriptor-port arbitration these properties
    // are about, so it is reproduced here as wiring -- the same way the two
    // modules below are wired to each other -- rather than as an assume. The
    // remaining terms of that expression are core-pipeline conditions
    // (OP_WB, ALU_BSY, ALU_REQ, level != 0) and stay free.
    logic        MMU_PTEST_START;
    logic        MMU_PTEST_START_FREE;
    always_comb MMU_PTEST_START_FREE = $anyseq;
    assign MMU_PTEST_START = MMU_PTEST_START_FREE && !MMU_TWALK_BUSY;

    logic        MMU_PLOAD_START;
    logic [2:0]  MMU_PLOAD_FC;
    logic [31:0] MMU_PLOAD_LOGICAL;
    logic        MMU_PLOAD_WRITE;
    always_comb begin
        MMU_PLOAD_START = $anyseq;
        MMU_PLOAD_FC = $anyseq;
        MMU_PLOAD_LOGICAL = $anyseq;
        MMU_PLOAD_WRITE = $anyseq;
    end

    // ---- nets between the two modules and out of them ----
    logic        MMU_RUNTIME_REQ;
    logic        MMU_RUNTIME_FAULT;
    logic        MMU_RUNTIME_STALL;
    logic        MMU_RUNTIME_ATC_REFILL;
    logic [2:0]  MMU_RUNTIME_ATC_FC;
    logic [31:0] MMU_RUNTIME_ATC_TAG;
    logic [31:0] MMU_RUNTIME_ATC_PTAG;
    logic        MMU_RUNTIME_ATC_B;
    logic        MMU_RUNTIME_ATC_W;
    logic        MMU_RUNTIME_ATC_M;
    logic        MMU_RUNTIME_ATC_CI;
    logic [31:0] ADR_P_PHYS;
    logic [31:0] ADR_P_PHYS_CALC;
    logic [31:0] ADR_BUS_REQ_PHYS;
    logic        MMU_TWALK_START;

    logic        DATA_RD;
    logic        DATA_WR;
    logic        DATA_RD_BUS;
    logic        BURST_PREFETCH_OP_REQ;
    logic        BURST_PREFETCH_DATA_REQ;
    logic [2:0]  BURST_PREFETCH_OP_WORD;
    logic [1:0]  BURST_PREFETCH_DATA_ENTRY;
    logic [31:0] BURST_PREFETCH_ADDR;
    logic [2:0]  BURST_PREFETCH_FC;
    logic        RD_REQ;
    logic        WR_REQ;
    logic        OPCODE_REQ;
    logic        OPCODE_REQ_CORE;
    logic        OPCODE_REQ_CORE_MISS;
    logic        RD_REQ_I;
    logic        WR_REQ_I;
    logic        OPCODE_REQ_I;
    logic        MMU_FAULT_DATA_ACK;
    logic        MMU_FAULT_OPCODE_ACK;
    logic        BUS_CYCLE_BURST;
    logic        BUS_CYCLE_BURST_IS_OP;
    // MMU table-search bus master (UM 9.5.2).
    logic        MMU_TWALK_BUS_RD;
    logic        MMU_TWALK_BUS_WR;
    logic        MMU_TWALK_BUS_ACTIVE;
    logic [31:0] MMU_TWALK_BUS_ADDR;
    logic [31:0] MMU_TWALK_BUS_DATA;
    logic        MMU_TWALK_RMC;
    logic        BUS_CYCLE_MMU_WALK;
    logic        MMU_PLOAD_DONE;
    logic [35:0] MMU_PLOAD_RESULT;
    logic        MMU_TWALK_BUSY;
    logic        MMU_TWALK_VALID;
    logic [2:0]  MMU_TWALK_FC;
    logic [31:0] MMU_TWALK_LOGICAL;
    logic        MMU_TWALK_WRITE;
    logic [35:0] MMU_TWALK_RESULT;
    logic [2:0]  MMU_TWALK_STATE;
    logic [31:0] MMU_TWALK_TC;
    logic [31:0] MMU_TWALK_TABLE_BASE;
    logic [31:0] MMU_TWALK_DESC_ADDR;
    logic [31:0] MMU_TWALK_PAGE_BASE;
    logic [31:0] MMU_TWALK_DESC_PTR;
    logic [31:0] MMU_TWALK_INDEX;
    logic [5:0]  MMU_TWALK_CONSUMED;
    logic [3:0]  MMU_TWALK_DESC_SIZE;
    logic [3:0]  MMU_TWALK_NEXT_WIDTH;
    logic [2:0]  MMU_TWALK_LEVELS;
    logic [2:0]  MMU_TWALK_TLX_LEVEL;
    logic [1:0]  MMU_TWALK_DESC_DT;
    logic [14:0] MMU_TWALK_ROOT_LIMIT;
    logic        MMU_TWALK_ROOT_LIMIT_LOWER;
    logic        MMU_TWALK_FC_LEVEL;
    logic        MMU_TWALK_HAS_NEXT;
    logic        MMU_TWALK_WP_ACCUM;
    logic        MMU_PTEST_BUSY;
    logic        MMU_PTEST_READY;
    logic [15:0] MMU_PTEST_WALK_MMUSR;
    logic [31:0] MMU_PTEST_DESC_ADDR;

    WF68K30L_TOP_ROUTING_BUS_CACHE dut_bus_cache (
        .CLK(CLK),
        .RESET_CPU(RESET_CPU),
        .BUS_BSY(BUS_BSY),
        .BUSY_EXH(BUSY_EXH),
        .DATA_RD_EXH(DATA_RD_EXH),
        .DATA_RD_MAIN(DATA_RD_MAIN),
        .DATA_WR_EXH(DATA_WR_EXH),
        .DATA_WR_MAIN(DATA_WR_MAIN),
        .OPCODE_RD(OPCODE_RD),
        .RMC(RMC),
        .MMU_RUNTIME_REQ(MMU_RUNTIME_REQ),
        .MMU_RUNTIME_FAULT(MMU_RUNTIME_FAULT),
        .MMU_RUNTIME_STALL(MMU_RUNTIME_STALL),
        .MMU_RUNTIME_ATC_FC(MMU_RUNTIME_ATC_FC),
        .MMU_TC(MMU_TC),
        .MMU_SRP(MMU_SRP),
        .MMU_CRP(MMU_CRP),
        .ADR_P(ADR_P),
        .OP_SIZE_BUS(OP_SIZE_BUS),
        .DATA_TO_CORE_BUSIF(DATA_TO_CORE_BUSIF),
        .DATA_RDY_BUSIF(DATA_RDY_BUSIF),
        .DATA_VALID_BUSIF(DATA_VALID_BUSIF),
        .ICACHE_HIT_NOW(ICACHE_HIT_NOW),
        .DCACHE_HIT_NOW(DCACHE_HIT_NOW),
        .MMU_PTEST_FC(MMU_PTEST_FC),
        .MMU_PTEST_LOGICAL(MMU_PTEST_LOGICAL),
        .MMU_PTEST_LEVEL(MMU_PTEST_LEVEL),
        .MMU_PTEST_START(MMU_PTEST_START),
        .MMU_PTEST_CONSUME(MMU_PTEST_CONSUME),
        .ICACHE_BURST_FILL_VALID(ICACHE_BURST_FILL_VALID),
        .ICACHE_BURST_FILL_LINE(ICACHE_BURST_FILL_LINE),
        .ICACHE_BURST_FILL_TAG(ICACHE_BURST_FILL_TAG),
        .ICACHE_BURST_FILL_PENDING(ICACHE_BURST_FILL_PENDING),
        .ICACHE_BURST_FILL_FC(ICACHE_BURST_FILL_FC),
        .ICACHE_BURST_FILL_NEXT_WORD(ICACHE_BURST_FILL_NEXT_WORD),
        .DCACHE_BURST_FILL_VALID(DCACHE_BURST_FILL_VALID),
        .DCACHE_BURST_FILL_LINE(DCACHE_BURST_FILL_LINE),
        .DCACHE_BURST_FILL_TAG(DCACHE_BURST_FILL_TAG),
        .DCACHE_BURST_FILL_PENDING(DCACHE_BURST_FILL_PENDING),
        .DCACHE_BURST_FILL_FC(DCACHE_BURST_FILL_FC),
        .DCACHE_BURST_FILL_NEXT_ENTRY(DCACHE_BURST_FILL_NEXT_ENTRY),
        .MMU_TWALK_START(MMU_TWALK_START),
        .MMU_PLOAD_START(MMU_PLOAD_START),
        .MMU_PLOAD_FC(MMU_PLOAD_FC),
        .MMU_PLOAD_LOGICAL(MMU_PLOAD_LOGICAL),
        .MMU_PLOAD_WRITE(MMU_PLOAD_WRITE),
        .MMU_PLOAD_DONE(MMU_PLOAD_DONE),
        .MMU_PLOAD_RESULT(MMU_PLOAD_RESULT),
        .DATA_RD(DATA_RD),
        .DATA_WR(DATA_WR),
        .DATA_RD_BUS(DATA_RD_BUS),
        .BURST_PREFETCH_OP_REQ(BURST_PREFETCH_OP_REQ),
        .BURST_PREFETCH_DATA_REQ(BURST_PREFETCH_DATA_REQ),
        .BURST_PREFETCH_OP_WORD(BURST_PREFETCH_OP_WORD),
        .BURST_PREFETCH_DATA_ENTRY(BURST_PREFETCH_DATA_ENTRY),
        .BURST_PREFETCH_ADDR(BURST_PREFETCH_ADDR),
        .BURST_PREFETCH_FC(BURST_PREFETCH_FC),
        .RD_REQ(RD_REQ),
        .WR_REQ(WR_REQ),
        .OPCODE_REQ(OPCODE_REQ),
        .OPCODE_REQ_CORE(OPCODE_REQ_CORE),
        .OPCODE_REQ_CORE_MISS(OPCODE_REQ_CORE_MISS),
        .RD_REQ_I(RD_REQ_I),
        .WR_REQ_I(WR_REQ_I),
        .OPCODE_REQ_I(OPCODE_REQ_I),
        .MMU_FAULT_DATA_ACK(MMU_FAULT_DATA_ACK),
        .MMU_FAULT_OPCODE_ACK(MMU_FAULT_OPCODE_ACK),
        .BUS_CYCLE_BURST(BUS_CYCLE_BURST),
        .BUS_CYCLE_BURST_IS_OP(BUS_CYCLE_BURST_IS_OP),
        .MMU_TWALK_BUS_RD(MMU_TWALK_BUS_RD),
        .MMU_TWALK_BUS_WR(MMU_TWALK_BUS_WR),
        .MMU_TWALK_BUS_ACTIVE(MMU_TWALK_BUS_ACTIVE),
        .MMU_TWALK_BUS_ADDR(MMU_TWALK_BUS_ADDR),
        .MMU_TWALK_BUS_DATA(MMU_TWALK_BUS_DATA),
        .MMU_TWALK_RMC(MMU_TWALK_RMC),
        .BUS_CYCLE_MMU_WALK(BUS_CYCLE_MMU_WALK),
        .MMU_TWALK_BUSY(MMU_TWALK_BUSY),
        .MMU_TWALK_VALID(MMU_TWALK_VALID),
        .MMU_TWALK_FC(MMU_TWALK_FC),
        .MMU_TWALK_LOGICAL(MMU_TWALK_LOGICAL),
        .MMU_TWALK_WRITE(MMU_TWALK_WRITE),
        .MMU_TWALK_RESULT(MMU_TWALK_RESULT),
        .MMU_TWALK_STATE(MMU_TWALK_STATE),
        .MMU_TWALK_TC(MMU_TWALK_TC),
        .MMU_TWALK_TABLE_BASE(MMU_TWALK_TABLE_BASE),
        .MMU_TWALK_DESC_ADDR(MMU_TWALK_DESC_ADDR),
        .MMU_TWALK_PAGE_BASE(MMU_TWALK_PAGE_BASE),
        .MMU_TWALK_DESC_PTR(MMU_TWALK_DESC_PTR),
        .MMU_TWALK_INDEX(MMU_TWALK_INDEX),
        .MMU_TWALK_CONSUMED(MMU_TWALK_CONSUMED),
        .MMU_TWALK_DESC_SIZE(MMU_TWALK_DESC_SIZE),
        .MMU_TWALK_NEXT_WIDTH(MMU_TWALK_NEXT_WIDTH),
        .MMU_TWALK_LEVELS(MMU_TWALK_LEVELS),
        .MMU_TWALK_TLX_LEVEL(MMU_TWALK_TLX_LEVEL),
        .MMU_TWALK_DESC_DT(MMU_TWALK_DESC_DT),
        .MMU_TWALK_ROOT_LIMIT(MMU_TWALK_ROOT_LIMIT),
        .MMU_TWALK_ROOT_LIMIT_LOWER(MMU_TWALK_ROOT_LIMIT_LOWER),
        .MMU_TWALK_FC_LEVEL(MMU_TWALK_FC_LEVEL),
        .MMU_TWALK_HAS_NEXT(MMU_TWALK_HAS_NEXT),
        .MMU_TWALK_WP_ACCUM(MMU_TWALK_WP_ACCUM),
        .MMU_PTEST_BUSY(MMU_PTEST_BUSY),
        .MMU_PTEST_READY(MMU_PTEST_READY),
        .MMU_PTEST_WALK_MMUSR(MMU_PTEST_WALK_MMUSR),
        .MMU_PTEST_DESC_ADDR(MMU_PTEST_DESC_ADDR)
    );

    WF68K30L_TOP_ROUTING_MMU_TRANSLATE dut_mmu_translate (
        .BUS_BSY(BUS_BSY),
        .DATA_WR(DATA_WR),
        .DATA_RD(DATA_RD),
        .OPCODE_RD(OPCODE_RD),
        .MMU_TC(MMU_TC),
        .FC_I(FC_I),
        .MMU_SRP(MMU_SRP),
        .MMU_CRP(MMU_CRP),
        .ADR_P(ADR_P),
        .MMU_TT0(MMU_TT0),
        .MMU_TT1(MMU_TT1),
        .RMC(RMC),
        .MMU_TWALK_VALID(MMU_TWALK_VALID),
        .MMU_TWALK_FC(MMU_TWALK_FC),
        .MMU_TWALK_LOGICAL(MMU_TWALK_LOGICAL),
        .MMU_TWALK_WRITE(MMU_TWALK_WRITE),
        .MMU_TWALK_RESULT(MMU_TWALK_RESULT),
        .MMU_TWALK_BUSY(MMU_TWALK_BUSY),
        .ADR_P_PHYS_LATCH(ADR_P_PHYS_LATCH),
        .BURST_PREFETCH_OP_REQ(BURST_PREFETCH_OP_REQ),
        .BURST_PREFETCH_DATA_REQ(BURST_PREFETCH_DATA_REQ),
        .BURST_PREFETCH_ADDR(BURST_PREFETCH_ADDR),
        .MMU_ATC_V_FLAT(MMU_ATC_V_FLAT),
        .MMU_ATC_B_FLAT(MMU_ATC_B_FLAT),
        .MMU_ATC_W_FLAT(MMU_ATC_W_FLAT),
        .MMU_ATC_M_FLAT(MMU_ATC_M_FLAT),
        .MMU_ATC_CI_FLAT(MMU_ATC_CI_FLAT),
        .MMU_ATC_FC_FLAT(MMU_ATC_FC_FLAT),
        .MMU_ATC_TAG_FLAT(MMU_ATC_TAG_FLAT),
        .MMU_ATC_PTAG_FLAT(MMU_ATC_PTAG_FLAT),
        .ADR_P_PHYS_CALC(ADR_P_PHYS_CALC),
        .ADR_P_PHYS(ADR_P_PHYS),
        .ADR_BUS_REQ_PHYS(ADR_BUS_REQ_PHYS),
        .MMU_RUNTIME_REQ(MMU_RUNTIME_REQ),
        .MMU_RUNTIME_FAULT(MMU_RUNTIME_FAULT),
        .MMU_RUNTIME_ATC_REFILL(MMU_RUNTIME_ATC_REFILL),
        .MMU_RUNTIME_ATC_FC(MMU_RUNTIME_ATC_FC),
        .MMU_RUNTIME_ATC_TAG(MMU_RUNTIME_ATC_TAG),
        .MMU_RUNTIME_ATC_PTAG(MMU_RUNTIME_ATC_PTAG),
        .MMU_RUNTIME_ATC_B(MMU_RUNTIME_ATC_B),
        .MMU_RUNTIME_ATC_W(MMU_RUNTIME_ATC_W),
        .MMU_RUNTIME_ATC_M(MMU_RUNTIME_ATC_M),
        .MMU_RUNTIME_ATC_CI(MMU_RUNTIME_ATC_CI),
        .MMU_RUNTIME_STALL(MMU_RUNTIME_STALL),
        .MMU_TWALK_START(MMU_TWALK_START)
    );

    // A PLOAD walk carries its own latched operands and is deliberately exempt
    // from the abandon-on-bus-cycle rule below, so the runtime-walk properties
    // are guarded by "no PLOAD has been requested since reset" rather than by
    // an assume on MMU_PLOAD_START. The guard is sound because the sequencer's
    // internal is-PLOAD flag can only be set from MMU_PLOAD_START, and it
    // constrains no input, so nothing else here goes vacuous.
    logic f_pload_req = 1'b0;
    always_ff @(posedge CLK) begin
        if (RESET_CPU)
            f_pload_req <= 1'b0;
        else if (MMU_PLOAD_START)
            f_pload_req <= 1'b1;
    end

    // "The search in flight has already driven a descriptor cycle." Needed to
    // say what UM 9.5.2 says about RMC without conflating one search's tail
    // with the next search's start: RMC is raised by the first descriptor cycle,
    // so before that cycle a search legitimately runs with it negated. The
    // arbitration guarantees at least one cycle with neither sequencer busy
    // between two searches, so this cannot carry across from one to the next.
    logic f_search_drove = 1'b0;
    always_ff @(posedge CLK) begin
        if (RESET_CPU || !(MMU_TWALK_BUSY || MMU_PTEST_BUSY))
            f_search_drove <= 1'b0;
        else if (MMU_TWALK_BUS_RD || MMU_TWALK_BUS_WR)
            f_search_drove <= 1'b1;
    end

    always_ff @(posedge CLK) begin
        if (f_past_valid) begin
            // The arm state is one thing published on two ports: a search is in
            // flight exactly while the sequencer is out of its idle state.
            assert(MMU_TWALK_BUSY == (MMU_TWALK_STATE != 3'd0));

            // Reset clears the arm state.
            if ($past(RESET_CPU)) begin
                assert(!MMU_TWALK_BUSY);
                assert(!MMU_TWALK_VALID);
                assert(MMU_TWALK_STATE == 3'd0);
            end

            // A bus cycle the search does not own abandons the runtime walk:
            // that cycle belongs to the access which displaced it. The search
            // itself masters the bus for its descriptor accesses (UM 9.5.2), so
            // a cycle started while a search is in flight is the search's own
            // and must not abandon it. The property used to be unconditional in
            // $past(BUS_BSY) because no walk could ever own a cycle.
            if (!f_pload_req && $past(BUS_BSY) && !$past(MMU_TWALK_BUSY)) begin
                assert(!MMU_TWALK_BUSY);
                assert(!MMU_TWALK_VALID);
                assert(MMU_TWALK_STATE == 3'd0);
            end

            // The descriptor-bus port has exactly one owner at a time. UM 9.8
            // makes PTEST perform a table search too, so the port has two
            // clients; a search is a sequence of bus cycles UM 9.5.2 requires
            // to run uninterrupted, so the two must never be in flight
            // together. This is the invariant the whole arbitration rests on,
            // and it is what makes the request mux in the RTL safe without a
            // round-robin.
            assert(!(MMU_TWALK_BUSY && MMU_PTEST_BUSY));

            // What makes "the search's own cycle" well defined: a descriptor
            // access is only ever requested by a search in flight, only while
            // the bus is free, and while a search is in flight the core's own
            // read, write and opcode requests are all held off. So no other
            // request can be taken up in the window the search drives, and the
            // search cannot be starved by one either -- which is the exclusivity
            // the abandon rule above now depends on. Both owners are admitted
            // here; the exclusivity assert above is what keeps that from being
            // a weaker statement than the single-owner version it replaced.
            assert(!(MMU_TWALK_BUS_RD || MMU_TWALK_BUS_WR) ||
                   MMU_TWALK_BUSY || MMU_PTEST_BUSY);
            assert(!(MMU_TWALK_BUS_RD || MMU_TWALK_BUS_WR) || !BUS_BSY);
            assert(!(MMU_TWALK_BUS_RD && MMU_TWALK_BUS_WR));
            // Only the runtime/PLOAD search ever writes a descriptor. PTEST
            // reports status and must not disturb the ATC or the tables
            // (UM 9.8); the U/M history update belongs to the search that is
            // about to allow an access to the page (UM 9.5.2/9.6.1).
            assert(!MMU_TWALK_BUS_WR || MMU_TWALK_BUSY);
            if ((MMU_TWALK_BUSY || MMU_PTEST_BUSY) && !BUS_BSY) begin
                assert(RD_REQ == MMU_TWALK_BUS_RD);
                assert(WR_REQ == MMU_TWALK_BUS_WR);
                assert(!OPCODE_REQ);
            end

            // UM 9.5.2: RMC "is asserted on the first bus cycle of the search
            // and remains asserted throughout". Both halves are checked: it is
            // asserted on every descriptor cycle the port drives, and it does
            // not drop while a search is in flight. MMU_TWALK_BUS_RD or
            // MMU_TWALK_BUS_WR is exactly "the port is driving a cycle now".
            assert(!(MMU_TWALK_BUS_RD || MMU_TWALK_BUS_WR) || MMU_TWALK_RMC);
            if (f_search_drove && (MMU_TWALK_BUSY || MMU_PTEST_BUSY))
                assert(MMU_TWALK_RMC);

            // The negation is one cycle late: MMU_TWALK_RMC_R is cleared by a
            // registered "no search is in flight", so RMC stays asserted for
            // the cycle after the search retires. That is a real deviation --
            // the core's first post-search bus cycle can be presented with
            // RMCn asserted -- but it is not this harness's subject and it
            // predates the PTEST search joining the port: the clear was
            // "else if (!MMU_TWALK_BUSY)" before and is
            // "else if (!MMU_DESC_SEARCH_BUSY)" now, the same shape with a
            // wider hold. Tightening it means negating RMC while the last
            // descriptor cycle may still have AS asserted, which is a change to
            // the bus contract rather than to the search. Stated here as a
            // bound so the tail cannot silently grow past one cycle.
            assert(!MMU_TWALK_RMC || MMU_TWALK_BUSY || MMU_PTEST_BUSY ||
                   $past(MMU_TWALK_BUSY) || $past(MMU_PTEST_BUSY));

            if (!f_pload_req && !$past(RESET_CPU) && !$past(BUS_BSY)) begin
                // Nothing arms a walk except a request for one.
                if (!$past(MMU_TWALK_START) && !$past(MMU_TWALK_BUSY))
                    assert(!MMU_TWALK_BUSY);

                // A requested walk with the sequencer free and the descriptor
                // port not claimed by PTEST is always taken up, either as a
                // search in flight or as a result available at once. The PTEST
                // terms are the arbitration, not an escape hatch: a walk that
                // loses to PTEST is deferred, and MMU_TWALK_START is a level, so
                // it is taken up as soon as the PTEST search retires. The
                // symmetric non-starvation property for PTEST is below.
                if ($past(MMU_TWALK_START) && !$past(MMU_TWALK_BUSY) &&
                    !$past(MMU_PTEST_BUSY) && !$past(MMU_PTEST_START))
                    assert(MMU_TWALK_BUSY || MMU_TWALK_VALID);
            end

            // A requested PTEST search is always taken up: either the sequencer
            // goes busy on it or it answers from the root pointer alone. So
            // neither client can be dropped by the arbitration -- the walk side
            // above waits for PTEST, and PTEST never waits for itself.
            if (!$past(RESET_CPU) && $past(MMU_PTEST_START))
                assert(MMU_PTEST_BUSY || MMU_PTEST_READY);

            // An idle cycle retires the published result, so the next access
            // cannot be answered from the previous one's search.
            if (!f_pload_req && !$past(RESET_CPU) &&
                !$past(MMU_RUNTIME_REQ) && !$past(MMU_TWALK_BUSY))
                assert(!MMU_TWALK_VALID);
        end
    end
endmodule
