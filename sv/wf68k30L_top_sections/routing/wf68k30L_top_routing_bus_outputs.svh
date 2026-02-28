// Address and fault address latches.
always_ff @(posedge CLK) begin : adr_latches
    if (!BUS_BSY) begin
        ADR_LATCH <= ADR_P;
        ADR_P_PHYS_LATCH <= ADR_P_PHYS_CALC;
        FC_LATCH <= FC_I;
    end else if (!BERRn) begin
        FAULT_ADR <= ADR_LATCH;
    end
end

// Function code generation.
always_comb begin : fc_generation
    if (BUS_BSY)
        FC_I = FC_LATCH;
    else if (USE_SFC)
        FC_I = SFC;
    else if (USE_DFC)
        FC_I = DFC;
    else if ((DATA_RD || DATA_WR) && CPU_SPACE)
        FC_I = FC_CPU_SPACE;
    else if ((DATA_RD || DATA_WR) && SBIT)
        FC_I = FC_SUPER_DATA;
    else if (DATA_RD || DATA_WR)
        FC_I = FC_USER_DATA;
    else if (OPCODE_RD && SBIT)
        FC_I = FC_SUPER_PROG;
    else
        FC_I = FC_USER_PROG;
end

assign FC_BUS_REQ = (BURST_PREFETCH_OP_REQ || BURST_PREFETCH_DATA_REQ) ?
                    BURST_PREFETCH_FC : FC_I;

// External CIOUT reflects MMU CI status for bus cycles that reach the external bus.
always_comb begin : ciout_generation
    logic read_access;
    logic write_access;
    logic rmw_access;

    read_access = RD_REQ || OPCODE_REQ;
    write_access = WR_REQ;
    rmw_access = RMC;

    CIOUT_ASSERT = mmu_ci_out(FC_I, ADR_P, read_access, write_access, rmw_access, MMU_TT0, MMU_TT1);
end

assign CIOUTn = (ASn == 1'b0) ? !CIOUT_ASSERT : 1'b1;

// External burst request semantics (phase 5/7):
// Request burst fills for cacheable miss cycles when burst-enable bits are set.
// Once a line has been burst-acknowledged (CBACKn low), suppress redundant
// requests for follow-on misses in that same line/tag context.
always_comb begin : cbreq_generation
    logic icache_same_burst_line;
    logic dcache_same_burst_line;
    CBREQ_INST_REQ_NOW = 1'b0;
    CBREQ_DATA_REQ_NOW = 1'b0;
    icache_same_burst_line = ICACHE_BURST_TRACK_VALID &&
                             ICACHE_BURST_TRACK_LINE == ADR_P_PHYS[7:4] &&
                             ICACHE_BURST_TRACK_TAG == ADR_P_PHYS[31:8];
    dcache_same_burst_line = DCACHE_BURST_TRACK_VALID &&
                             DCACHE_BURST_TRACK_LINE == ADR_P_PHYS[7:4] &&
                             DCACHE_BURST_TRACK_TAG == ADR_P_PHYS[31:8];

    // Instruction burst-request candidate: I-cache enabled + burst enabled + miss fillable.
    if (!BUS_BSY && !MMU_RUNTIME_FAULT && !MMU_RUNTIME_STALL && OPCODE_REQ_CORE_MISS && CACR[0] && CACR[4] && !CACR[1]) begin
        CBREQ_INST_REQ_NOW = !mmu_cache_inhibit(FC_I, ADR_P_PHYS, 1'b1, 1'b0, 1'b0, MMU_TT0, MMU_TT1) &&
                             !icache_same_burst_line;
    end

    // Data burst-request candidate: D-cache read miss fill on aligned longword access.
    if (!BUS_BSY && !MMU_RUNTIME_FAULT && !MMU_RUNTIME_STALL && DATA_RD_BUS && CACR[8] && CACR[12] && !CACR[9] && !RMC &&
        OP_SIZE_BUS == LONG && ADR_P_PHYS[1:0] == 2'b00) begin
        CBREQ_DATA_REQ_NOW = !mmu_cache_inhibit(FC_I, ADR_P_PHYS, 1'b1, 1'b0, 1'b0, MMU_TT0, MMU_TT1) &&
                             !dcache_same_burst_line;
    end

    CBREQ_REQ_NOW = CBREQ_INST_REQ_NOW || CBREQ_DATA_REQ_NOW;
end

always_ff @(posedge CLK) begin : cbreq_latch
    if (RESET_CPU) begin
        CBREQ_REQ_LATCH <= 1'b0;
    end else if (!BUS_BSY) begin
        CBREQ_REQ_LATCH <= CBREQ_REQ_NOW;
    end else if (!CBACKn) begin
        // In this surface model, a burst acknowledge consumes the request.
        CBREQ_REQ_LATCH <= 1'b0;
    end
end

assign CBREQ_ASSERT = BUS_BSY ? CBREQ_REQ_LATCH : CBREQ_REQ_NOW;
assign CBREQn = (ASn == 1'b0 && CBREQ_ASSERT) ? 1'b0 : 1'b1;
