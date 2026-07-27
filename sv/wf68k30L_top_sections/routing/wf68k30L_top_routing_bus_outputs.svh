// Address and fault address latches.
always_ff @(posedge CLK) begin : adr_latches
    if (!BUS_BSY) begin
        ADR_LATCH <= ADR_P;
        ADR_P_PHYS_LATCH <= ADR_P_PHYS_CALC;
        FC_LATCH <= FC_I;
        CIOUT_LATCH <= CIOUT_ASSERT_NOW;
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

    // A background burst-completion cycle runs at the burst address, not at the
    // core's, and is only ever started for a line that already passed the
    // cache-inhibit check, so its CI status is negated by construction.
    if (BURST_PREFETCH_OP_REQ || BURST_PREFETCH_DATA_REQ)
        CIOUT_ASSERT_NOW = 1'b0;
    else
        CIOUT_ASSERT_NOW = mmu_ci_out(FC_I, ADR_P, read_access, write_access, rmw_access, MMU_TT0, MMU_TT1, MMU_RUNTIME_ATC_CI);
end

// UM 7.3.1: CIOUT is driven with the address and stays valid for the whole
// cycle, so it must reflect the access that requested it. ADR_P and the request
// strobes both move on while BUS_BSY.
assign CIOUT_ASSERT = BUS_BSY ? CIOUT_LATCH : CIOUT_ASSERT_NOW;

assign CIOUTn = (ASn == 1'b0) ? !CIOUT_ASSERT : 1'b1;

// ---- Burst acknowledge qualification ----
// UM 7.3.7: "burst mode is only initiated if both of these signals are
// asserted for a synchronous cycle", and UM 6.1.3.2: "The MC68030 ignores the
// assertion of CBACK during cycles terminated with DSACKx." STERM and DSACKx
// are never both asserted for the same cycle (UM 7.3.2), so STERM seen while
// AS is asserted with DSACKx negated is exactly the cycle shape whose CBACK
// may be honoured -- STERM is the 32-bit synchronous acknowledge.
//
// Tracked across the whole AS window rather than at one nominated slice,
// because UM 7.4.1 requires STERM to meet setup and hold against every rising
// edge while AS is asserted. The register is re-armed while the bus is idle,
// which is the clock on which the completing cycle raises its ready strobe, so
// a consumer sampling on that strobe still reads this cycle's value.

assign STERM_NOW = !ASn && !STERMn && DSACKn == 2'b11;

always_ff @(posedge CLK) begin : cycle_sterm_track
    if (RESET_CPU || !BUS_BSY)
        CYCLE_STERM_32 <= 1'b0;
    else if (!ASn && DSACKn != 2'b11)
        CYCLE_STERM_32 <= 1'b0; // Asynchronously terminated: CBACK is ignored.
    else if (STERM_NOW)
        CYCLE_STERM_32 <= 1'b1;
end

// CBACK gated by the termination the cycle in flight actually saw. The
// registered term carries the qualification one clock past the cycle, so a
// consumer sampling on the cycle's ready strobe still sees it; STERM_NOW covers
// the terminating edge itself, which a two-clock synchronous cycle reaches
// before the register can update.
//
// The cache state machine still consumes the raw CBACKn pin and so still opens
// a burst-fill context off an acknowledge the UM says to ignore. Wiring its
// CBACKn port to !CBACK_HONOURED closes that, which needs the net declared in
// wf68k30L_top_sections/wf68k30L_top_decls.svh (that file is included ahead of
// the instantiation) and the port re-pointed in
// wf68k30L_top_sections/wf68k30L_top_cache_mmu_state.svh.
assign CBACK_HONOURED = !CBACKn && (CYCLE_STERM_32 || STERM_NOW);

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
    // UM 7.3.7: "CBREQ is not asserted during the read or write cycles of any
    // read-modify-write operation", and no burst filling occurs while RMC is
    // asserted, so nothing inside that indivisible sequence may request one.
    if (!BUS_BSY && !MMU_RUNTIME_FAULT && !MMU_RUNTIME_STALL && OPCODE_REQ_CORE_MISS && !RMC && CACR[0] && CACR[4] && !CACR[1]) begin
        CBREQ_INST_REQ_NOW = !mmu_cache_inhibit(FC_I, ADR_P_PHYS, 1'b1, 1'b0, 1'b0, MMU_TT0, MMU_TT1, MMU_RUNTIME_ATC_CI) &&
                             !icache_same_burst_line;
    end

    // Data burst-request candidate: D-cache read miss fill on aligned longword access.
    if (!BUS_BSY && !MMU_RUNTIME_FAULT && !MMU_RUNTIME_STALL && DATA_RD_BUS && CACR[8] && CACR[12] && !CACR[9] && !RMC &&
        OP_SIZE == LONG && ADR_P_PHYS[1:0] == 2'b00) begin
        CBREQ_DATA_REQ_NOW = !mmu_cache_inhibit(FC_I, ADR_P_PHYS, 1'b1, 1'b0, 1'b0, MMU_TT0, MMU_TT1, MMU_RUNTIME_ATC_CI) &&
                             !dcache_same_burst_line;
    end

    CBREQ_REQ_NOW = CBREQ_INST_REQ_NOW || CBREQ_DATA_REQ_NOW;
end

always_ff @(posedge CLK) begin : cbreq_latch
    if (RESET_CPU) begin
        CBREQ_REQ_LATCH <= 1'b0;
    end else if (!BUS_BSY) begin
        CBREQ_REQ_LATCH <= CBREQ_REQ_NOW;
    end else if (CBACK_HONOURED || CACHE_INHIBIT_IN) begin
        // A burst acknowledge consumes the request. On a real burst UM 7.3.7
        // holds CBREQ until STERM for the third of four long words; with no
        // streaming engine there is only ever one cycle to hold it for, so it
        // is released at the end of that cycle instead. An acknowledge on a
        // DSACKx-terminated cycle is not one, and must leave CBREQ alone.
        // UM 6.1.3.1: CIIN also negates CBREQ, aborting the burst.
        CBREQ_REQ_LATCH <= 1'b0;
    end
end

assign CBREQ_ASSERT = BUS_BSY ? CBREQ_REQ_LATCH : CBREQ_REQ_NOW;

// UM 7.3.7: "CBREQ is not asserted during the read or write cycles of any
// read-modify-write operation." That is a statement about the pin, so it is
// enforced at the pin: a request latched before RMC rose must not survive into
// the sequence either. RMC is the same signal the bus interface drives RMCn
// from, so CBREQ and RMC can never be asserted together on the bus.
assign CBREQn = (ASn == 1'b0 && CBREQ_ASSERT && !RMC) ? 1'b0 : 1'b1;
