// MMU address translation flow:
// - TT hit bypass.
// - ATC hit/fault check.
// - Root-pointer direct map (DT=01) or table walk (DT=10/11).
// - Runtime sideband outputs (stall/fault/ATC refill).
always_comb begin : mmu_address_translate
    logic        read_access;
    logic        write_access;
    logic        rmw_access;
    logic        mmu_req_now;
    logic        tt_hit;
    logic        atc_hit;
    logic        atc_fault;
    logic [2:0]  atc_fc;
    logic [31:0] atc_logical;
    logic [31:0] atc_tag;
    logic [31:0] atc_phys;
    logic [31:0] atc_ptag;
    logic        atc_b;
    logic        atc_w;
    logic        atc_m;
    logic [4:0]  page_shift;
    logic [31:0] page_mask;
    logic        root_valid;
    logic        root_short_table;
    logic        root_long_table;
    logic [1:0]  root_dt;
    logic [63:0] root_ptr;
    logic [31:0] root_offs;
    logic [14:0] root_limit;
    logic        root_limit_lower;
    logic [31:0] first_index;
    logic        root_limit_fault;
    logic [35:0] walk_eval;
    integer      i;

    read_access = OPCODE_RD || DATA_RD;
    write_access = DATA_WR;
    rmw_access = RMC;
    mmu_req_now = !BUS_BSY && (DATA_WR || DATA_RD_BUS || OPCODE_REQ_CORE_MISS);

    root_ptr = (MMU_TC[25] && FC_I[2]) ? MMU_SRP : MMU_CRP; // SRE + supervisor access.
    root_offs = {root_ptr[31:4], 4'b0000}; // DT=1 constant offset.
    root_dt = root_ptr[33:32];
    root_valid = (root_dt == 2'b01);
    root_short_table = (root_dt == 2'b10);
    root_long_table = (root_dt == 2'b11);
    root_limit = root_ptr[62:48];
    root_limit_lower = root_ptr[63];
    if (MMU_TC[24])
        first_index = {29'h0, FC_I};
    else
        first_index = mmu_index_extract(ADR_P, MMU_TC[19:16], 6'd0, MMU_TC[15:12]);
    root_limit_fault = (root_limit_lower && first_index[14:0] < root_limit) ||
                       (!root_limit_lower && first_index[14:0] > root_limit);
    page_shift = mmu_page_shift(MMU_TC);
    if (page_shift == 5'd0)
        page_mask = 32'h0000_0000;
    else
        page_mask = 32'hFFFF_FFFF >> (6'd32 - {1'b0, page_shift});

    atc_fc = FC_I;
    atc_logical = ADR_P;
    atc_tag = mmu_page_tag(MMU_TC, atc_logical);
    atc_hit = 1'b0;
    atc_fault = 1'b0;
    atc_phys = ADR_P;
    atc_ptag = 32'h0;
    atc_b = 1'b0;
    atc_w = 1'b0;
    atc_m = write_access;
    for (i = 0; i < MMU_ATC_LINES; i = i + 1) begin
        if (!atc_hit &&
            MMU_ATC_V[i] &&
            MMU_ATC_FC[i] == atc_fc &&
            MMU_ATC_TAG[i] == atc_tag) begin
            atc_hit = 1'b1;
            atc_ptag = MMU_ATC_PTAG[i];
            atc_b = MMU_ATC_B[i];
            atc_w = MMU_ATC_W[i];
            atc_m = MMU_ATC_M[i] || write_access;
        end
    end
    if (atc_hit) begin
        atc_fault = atc_b || (write_access && atc_w);
        atc_phys = mmu_page_compose_addr(MMU_TC, atc_ptag, atc_logical);
    end

    tt_hit = mmu_tt_match(MMU_TT0, FC_I, ADR_P, read_access, write_access, rmw_access) ||
             mmu_tt_match(MMU_TT1, FC_I, ADR_P, read_access, write_access, rmw_access);

    MMU_RUNTIME_REQ = mmu_req_now;
    MMU_RUNTIME_FAULT = 1'b0;
    MMU_RUNTIME_ATC_REFILL = 1'b0;
    MMU_RUNTIME_ATC_FC = atc_fc;
    MMU_RUNTIME_ATC_TAG = atc_tag;
    MMU_RUNTIME_ATC_PTAG = 32'h0;
    MMU_RUNTIME_ATC_B = 1'b0;
    MMU_RUNTIME_ATC_W = 1'b0;
    MMU_RUNTIME_ATC_M = write_access;
    MMU_RUNTIME_STALL = 1'b0;

    if (tt_hit) begin
        ADR_P_PHYS_CALC = ADR_P;
    end else if (MMU_TC[31] && FC_I != FC_CPU_SPACE) begin
        if (atc_hit && !atc_fault) begin
            ADR_P_PHYS_CALC = atc_phys;
            MMU_RUNTIME_ATC_M = atc_m;
        end else if (atc_hit && atc_fault) begin
            ADR_P_PHYS_CALC = ADR_P;
            MMU_RUNTIME_FAULT = mmu_req_now;
        end else if (root_valid) begin
            // Root DT=1 direct mapping still applies root limit checks (FCL-independent).
            if (root_limit_fault) begin
                ADR_P_PHYS_CALC = ADR_P;
                MMU_RUNTIME_FAULT = mmu_req_now;
            end else begin
                ADR_P_PHYS_CALC = ADR_P + root_offs;
                MMU_RUNTIME_ATC_REFILL = mmu_req_now;
                MMU_RUNTIME_ATC_PTAG = mmu_page_tag(MMU_TC, ADR_P + root_offs);
                MMU_RUNTIME_ATC_M = write_access;
            end
        end else if (root_short_table || root_long_table) begin
            walk_eval = mmu_runtime_table_walk(
                ADR_P,
                FC_I,
                MMU_TC,
                root_ptr,
                write_access,
                mmu_req_now,
                MMU_WALK_DELAY_ARMED
            );
            if (walk_eval[33]) begin
                MMU_RUNTIME_STALL = 1'b1;
                ADR_P_PHYS_CALC = ADR_P;
            end else if (walk_eval[32]) begin
                ADR_P_PHYS_CALC = ADR_P;
                MMU_RUNTIME_FAULT = mmu_req_now;
            end else begin
                ADR_P_PHYS_CALC = walk_eval[31:0];
                MMU_RUNTIME_ATC_REFILL = mmu_req_now;
                MMU_RUNTIME_ATC_PTAG = mmu_page_tag(MMU_TC, walk_eval[31:0]);
                MMU_RUNTIME_ATC_W = walk_eval[34];
                MMU_RUNTIME_ATC_M = walk_eval[35];
            end
        end else begin
            ADR_P_PHYS_CALC = ADR_P;
            MMU_RUNTIME_FAULT = mmu_req_now;
        end
    end else begin
        ADR_P_PHYS_CALC = ADR_P;
    end
end

assign ADR_P_PHYS = BUS_BSY ? ADR_P_PHYS_LATCH : ADR_P_PHYS_CALC;
assign ADR_BUS_REQ_PHYS = (BURST_PREFETCH_OP_REQ || BURST_PREFETCH_DATA_REQ) ?
                          BURST_PREFETCH_ADDR : ADR_P_PHYS;
