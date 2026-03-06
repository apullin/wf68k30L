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
    root_limit_fault = (root_limit_lower && first_index[14:0] < root_limit) ||
                       (!root_limit_lower && first_index[14:0] > root_limit);

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
            atc_m = MMU_ATC_M_FLAT[atc_way_idx] || write_access;
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
    MMU_TWALK_START = 1'b0;

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
            end else begin
                ADR_P_PHYS_CALC = MMU_TWALK_RESULT[31:0];
                MMU_RUNTIME_ATC_REFILL = mmu_req_now;
                MMU_RUNTIME_ATC_PTAG = mmu_page_tag(MMU_TC, MMU_TWALK_RESULT[31:0]);
                MMU_RUNTIME_ATC_W = MMU_TWALK_RESULT[34];
                MMU_RUNTIME_ATC_M = MMU_TWALK_RESULT[35];
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

endmodule
