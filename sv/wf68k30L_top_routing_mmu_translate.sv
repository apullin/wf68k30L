(* keep_hierarchy = "yes" *)
module WF68K30L_TOP_ROUTING_MMU_TRANSLATE #(
    parameter int MMU_ATC_LINES = 8,
    parameter int MMU_ATC_WAYS = 2,
    parameter int MMU_ATC_SETS = MMU_ATC_LINES / MMU_ATC_WAYS,
    parameter int MMU_ATC_SET_BITS = $clog2(MMU_ATC_SETS)
) (
    input  logic        BUS_BSY,
    input  logic        DATA_WR,
    input  logic        DATA_RD,
    input  logic        OPCODE_RD,
    input  logic [31:0] MMU_TC,
    input  logic [2:0]  FC_I,
    input  logic [63:0] MMU_SRP,
    input  logic [63:0] MMU_CRP,
    input  logic [31:0] ADR_P,
    input  logic [31:0] MMU_TT0,
    input  logic [31:0] MMU_TT1,
    input  logic        RMC,
    input  logic        MMU_TWALK_VALID,
    input  logic [2:0]  MMU_TWALK_FC,
    input  logic [31:0] MMU_TWALK_LOGICAL,
    input  logic        MMU_TWALK_WRITE,
    input  logic [35:0] MMU_TWALK_RESULT,
    input  logic        MMU_TWALK_BUSY,
    input  logic [31:0] ADR_P_PHYS_LATCH,
    input  logic        BURST_PREFETCH_OP_REQ,
    input  logic        BURST_PREFETCH_DATA_REQ,
    input  logic [31:0] BURST_PREFETCH_ADDR,

    input  logic [MMU_ATC_SETS*MMU_ATC_WAYS-1:0] MMU_ATC_V_FLAT,
    input  logic [MMU_ATC_SETS*MMU_ATC_WAYS-1:0] MMU_ATC_B_FLAT,
    input  logic [MMU_ATC_SETS*MMU_ATC_WAYS-1:0] MMU_ATC_W_FLAT,
    input  logic [MMU_ATC_SETS*MMU_ATC_WAYS-1:0] MMU_ATC_M_FLAT,
    input  logic [MMU_ATC_SETS*MMU_ATC_WAYS-1:0] MMU_ATC_CI_FLAT,
    input  logic [MMU_ATC_SETS*MMU_ATC_WAYS*3-1:0] MMU_ATC_FC_FLAT,
    input  logic [MMU_ATC_SETS*MMU_ATC_WAYS*32-1:0] MMU_ATC_TAG_FLAT,
    input  logic [MMU_ATC_SETS*MMU_ATC_WAYS*32-1:0] MMU_ATC_PTAG_FLAT,

    output logic [31:0] ADR_P_PHYS_CALC,
    output logic [31:0] ADR_P_PHYS,
    output logic [31:0] ADR_BUS_REQ_PHYS,

    output logic        MMU_RUNTIME_REQ,
    output logic        MMU_RUNTIME_FAULT,
    output logic        MMU_RUNTIME_ATC_REFILL,
    output logic [2:0]  MMU_RUNTIME_ATC_FC,
    output logic [31:0] MMU_RUNTIME_ATC_TAG,
    output logic [31:0] MMU_RUNTIME_ATC_PTAG,
    output logic        MMU_RUNTIME_ATC_B,
    output logic        MMU_RUNTIME_ATC_W,
    output logic        MMU_RUNTIME_ATC_M,
    output logic        MMU_RUNTIME_ATC_CI,
    output logic        MMU_RUNTIME_STALL,
    output logic        MMU_TWALK_START
);

`include "wf68k30L_pkg.svh"
`include "wf68k30L_top_sections/helpers/wf68k30L_top_helpers_mmu_pure.svh"

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
    logic        atc_m_stored;
    logic        atc_m_research;
    logic        atc_ci;
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
    logic        walk_match;
    logic [MMU_ATC_SET_BITS-1:0] atc_set_idx;
    logic [2:0]  atc_way_fc;
    logic [31:0] atc_way_tag;
    logic [31:0] atc_way_ptag;
    integer      atc_way_idx;
    integer way;

    read_access = OPCODE_RD || DATA_RD;
    write_access = DATA_WR;
    rmw_access = RMC;
    // Qualify MMU runtime activity from core-side access intent to avoid
    // combinational feedback through cache-hit/miss qualification nets.
    mmu_req_now = !BUS_BSY && (DATA_WR || DATA_RD || OPCODE_RD);

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
    // UM 9.5.1.2: with TC.FCL set, the root pointer L/U and LIMIT fields are unused.
    // The comparison itself goes through the shared helper -- the other eight
    // limit checks in the design already do, and this was the one hand-written
    // copy that could drift away from them.
    root_limit_fault = !MMU_TC[24] &&
                       mmu_limit_violation(root_limit_lower, root_limit, first_index);

    atc_fc = FC_I;
    atc_logical = ADR_P;
    atc_tag = mmu_page_tag(MMU_TC, atc_logical);
    atc_hit = 1'b0;
    atc_fault = 1'b0;
    atc_phys = ADR_P;
    walk_match = 1'b0;
    atc_ptag = 32'h0;
    atc_b = 1'b0;
    atc_w = 1'b0;
    atc_m = write_access;
    atc_m_stored = 1'b0;
    atc_m_research = 1'b0;
    atc_ci = 1'b0;

    atc_set_idx = atc_tag[MMU_ATC_SET_BITS-1:0] ^ atc_fc[MMU_ATC_SET_BITS-1:0];
    for (way = 0; way < MMU_ATC_WAYS; way = way + 1) begin
        atc_way_idx = (atc_set_idx * MMU_ATC_WAYS) + way;
        atc_way_fc = MMU_ATC_FC_FLAT[(atc_way_idx*3) +: 3];
        atc_way_tag = MMU_ATC_TAG_FLAT[(atc_way_idx*32) +: 32];
        atc_way_ptag = MMU_ATC_PTAG_FLAT[(atc_way_idx*32) +: 32];
        if (!atc_hit &&
            MMU_ATC_V_FLAT[atc_way_idx] &&
            atc_way_fc == atc_fc &&
            atc_way_tag == atc_tag) begin
            atc_hit = 1'b1;
            atc_ptag = atc_way_ptag;
            atc_b = MMU_ATC_B_FLAT[atc_way_idx];
            atc_w = MMU_ATC_W_FLAT[atc_way_idx];
            atc_m_stored = MMU_ATC_M_FLAT[atc_way_idx];
            atc_m = atc_m_stored || write_access;
            atc_ci = MMU_ATC_CI_FLAT[atc_way_idx];
        end
    end

    if (atc_hit) begin
        atc_fault = atc_b || (write_access && atc_w);
        atc_phys = mmu_page_compose_addr(MMU_TC, atc_ptag, atc_logical);
        // UM 9.4: "If the M bit is clear and a write access to this logical
        // address is attempted, the MC68030 aborts the access and initiates a
        // table search, setting the M bit in the page descriptor, invalidating
        // the old ATC entry, and creating a new entry with the M bit set. The
        // MMU then allows the original write access to be performed." So a
        // write to an M-clear entry is routed through the search rather than
        // setting M in the entry alone -- the descriptor in memory has to move
        // too, and only the search can write it.
        atc_m_research = !atc_fault && write_access && !atc_m_stored;
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
    // Only an access the ATC (or a completed walk) resolved may report CI; a
    // TT hit bypasses translation and must not see a stale entry's CI bit.
    MMU_RUNTIME_ATC_CI = 1'b0;
    MMU_RUNTIME_STALL = 1'b0;
    MMU_TWALK_START = 1'b0;

    if (tt_hit) begin
        ADR_P_PHYS_CALC = ADR_P;
    end else if (MMU_TC[31] && FC_I != FC_CPU_SPACE) begin
        // UM 9.4: a bus error, invalid descriptor, supervisor violation or limit
        // violation during a search loads an entry with the B bit set, which
        // stays until a PFLUSH or PLOAD replaces the entry.
        if (atc_hit && !atc_fault && !atc_m_research) begin
            ADR_P_PHYS_CALC = atc_phys;
            MMU_RUNTIME_ATC_M = atc_m;
            MMU_RUNTIME_ATC_CI = atc_ci;
        end else if (atc_hit && atc_fault) begin
            ADR_P_PHYS_CALC = ADR_P;
            MMU_RUNTIME_FAULT = mmu_req_now;
        end else if (root_valid) begin
            if (root_limit_fault) begin
                ADR_P_PHYS_CALC = ADR_P;
                MMU_RUNTIME_FAULT = mmu_req_now;
                MMU_RUNTIME_ATC_REFILL = mmu_req_now;
                MMU_RUNTIME_ATC_B = 1'b1;
            end else begin
                ADR_P_PHYS_CALC = ADR_P + root_offs;
                MMU_RUNTIME_ATC_REFILL = mmu_req_now;
                MMU_RUNTIME_ATC_PTAG = mmu_page_tag(MMU_TC, ADR_P + root_offs);
                MMU_RUNTIME_ATC_M = write_access;
            end
        end else if (root_short_table || root_long_table) begin
            walk_match = MMU_TWALK_VALID &&
                         (MMU_TWALK_FC == atc_fc) &&
                         (MMU_TWALK_LOGICAL == atc_logical) &&
                         (MMU_TWALK_WRITE == write_access);
            if (!walk_match) begin
                MMU_RUNTIME_STALL = mmu_req_now;
                MMU_TWALK_START = mmu_req_now && !MMU_TWALK_BUSY;
                ADR_P_PHYS_CALC = ADR_P;
            end else if (MMU_TWALK_RESULT[32]) begin
                ADR_P_PHYS_CALC = ADR_P;
                MMU_RUNTIME_FAULT = mmu_req_now;
                MMU_RUNTIME_ATC_REFILL = mmu_req_now;
                MMU_RUNTIME_ATC_B = 1'b1;
                MMU_RUNTIME_ATC_W = MMU_TWALK_RESULT[34];
            end else begin
                ADR_P_PHYS_CALC = MMU_TWALK_RESULT[31:0];
                MMU_RUNTIME_ATC_REFILL = mmu_req_now;
                MMU_RUNTIME_ATC_PTAG = mmu_page_tag(MMU_TC, MMU_TWALK_RESULT[31:0]);
                MMU_RUNTIME_ATC_W = MMU_TWALK_RESULT[34];
                MMU_RUNTIME_ATC_M = MMU_TWALK_RESULT[35];
                MMU_RUNTIME_ATC_CI = MMU_TWALK_RESULT[33];
            end
        end else begin
            ADR_P_PHYS_CALC = ADR_P;
            MMU_RUNTIME_FAULT = mmu_req_now;
            MMU_RUNTIME_ATC_REFILL = mmu_req_now;
            MMU_RUNTIME_ATC_B = 1'b1;
        end
    end else begin
        ADR_P_PHYS_CALC = ADR_P;
    end
end

assign ADR_P_PHYS = BUS_BSY ? ADR_P_PHYS_LATCH : ADR_P_PHYS_CALC;
assign ADR_BUS_REQ_PHYS = (BURST_PREFETCH_OP_REQ || BURST_PREFETCH_DATA_REQ) ?
                          BURST_PREFETCH_ADDR : ADR_P_PHYS;

endmodule
