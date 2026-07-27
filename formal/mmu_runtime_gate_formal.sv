// Bounded check that no core-originated bus request survives an MMU runtime
// fault or a table-search stall. Both halves of the contract are real RTL:
// WF68K30L_TOP_ROUTING_BUS_CACHE owns RD_REQ/WR_REQ/OPCODE_REQ and the gating,
// WF68K30L_TOP_ROUTING_MMU_TRANSLATE produces MMU_RUNTIME_FAULT/STALL and
// MMU_TWALK_START. They are wired to each other exactly as
// sv/wf68k30L_top_sections/routing/wf68k30L_top_routing_cache_mmu_modules.svh
// wires them; everything else is free.
module mmu_runtime_gate_formal;
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
    logic        MMU_PTEST_START;
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
        MMU_PTEST_START = $anyseq;
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

    always_ff @(posedge CLK) begin
        // Core-originated requests must be suppressed whenever the MMU reports
        // a fault or a table-search stall window; only traffic that is not the
        // core's own access may still ask for a bus cycle. Since the walk
        // started mastering the bus (UM 9.5.2) there are two such sources: a
        // background burst fill and the search's own descriptor cycle. RD_REQ,
        // WR_REQ and OPCODE_REQ each carry the core term plus those, so
        // equality with the non-core terms is the port-visible statement that
        // the core term contributes nothing.
        if (MMU_RUNTIME_FAULT || MMU_RUNTIME_STALL) begin
            assert(RD_REQ == (BURST_PREFETCH_DATA_REQ || MMU_TWALK_BUS_RD));
            assert(WR_REQ == MMU_TWALK_BUS_WR);
            assert(OPCODE_REQ == BURST_PREFETCH_OP_REQ);
        end

        // While BUS_BSY the requests are replayed from latched copies of the
        // same equations, and MMU_RUNTIME_FAULT/STALL are already low by then,
        // so the check above cannot reach that copy. Pin it to the cycle the
        // latch sampled instead.
        if (f_past_valid && !$past(RESET_CPU) &&
            $past(MMU_RUNTIME_FAULT || MMU_RUNTIME_STALL)) begin
            assert(RD_REQ_I == $past(BURST_PREFETCH_DATA_REQ || MMU_TWALK_BUS_RD));
            assert(WR_REQ_I == $past(MMU_TWALK_BUS_WR));
            assert(OPCODE_REQ_I == $past(BURST_PREFETCH_OP_REQ));
        end
    end
endmodule
