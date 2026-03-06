(* keep_hierarchy = "yes" *)
module WF68K30L_TOP_MMU_STATE #(
    parameter int MMU_ATC_LINES = 8,
    parameter int MMU_ATC_WAYS = 2,
    parameter int MMU_ATC_SETS = MMU_ATC_LINES / MMU_ATC_WAYS,
    parameter int MMU_ATC_SET_BITS = $clog2(MMU_ATC_SETS),
    parameter int MMU_ATC_WAY_BITS = $clog2(MMU_ATC_WAYS)
) (
    input  logic        CLK,
    input  logic        RESET_CPU,
    input  logic [63:0] ALU_RESULT,
    input  logic        ALU_ACK,
    input  logic [6:0]  OP_WB,
    input  logic        MMU_ATC_FLUSH,
    input  logic        MMU_TC_WR,
    input  logic        MMU_SRP_WR,
    input  logic        MMU_CRP_WR,
    input  logic        MMU_TT0_WR,
    input  logic        MMU_TT1_WR,
    input  logic        MMU_MMUSR_WR,
    input  logic        MMU_RUNTIME_REQ,
    input  logic        MMU_RUNTIME_FAULT,
    input  logic        MMU_RUNTIME_ATC_REFILL,
    input  logic [2:0]  MMU_RUNTIME_ATC_FC,
    input  logic [31:0] MMU_RUNTIME_ATC_TAG,
    input  logic [31:0] MMU_RUNTIME_ATC_PTAG,
    input  logic        MMU_RUNTIME_ATC_B,
    input  logic        MMU_RUNTIME_ATC_W,
    input  logic        MMU_RUNTIME_ATC_M,
    input  logic [15:0] BIW_1,
    input  logic [31:0] DR_OUT_1,
    input  logic [2:0]  SFC,
    input  logic [2:0]  DFC,
    input  logic [31:0] ADR_EFF,
    input  logic [15:0] PTEST_WALK_MMUSR,

    output logic [63:0] MMU_SRP,
    output logic [63:0] MMU_CRP,
    output logic [31:0] MMU_TC,
    output logic [31:0] MMU_TT0,
    output logic [31:0] MMU_TT1,
    output logic [31:0] MMU_MMUSR,
    output logic        TRAP_MMU_CFG,
    output logic [31:0] MMU_ATC_FLUSH_COUNT,
    output logic [MMU_ATC_SETS*MMU_ATC_WAYS-1:0] MMU_ATC_V_FLAT,
    output logic [MMU_ATC_SETS*MMU_ATC_WAYS-1:0] MMU_ATC_B_FLAT,
    output logic [MMU_ATC_SETS*MMU_ATC_WAYS-1:0] MMU_ATC_W_FLAT,
    output logic [MMU_ATC_SETS*MMU_ATC_WAYS-1:0] MMU_ATC_M_FLAT,
    output logic [MMU_ATC_SETS*MMU_ATC_WAYS*3-1:0] MMU_ATC_FC_FLAT,
    output logic [MMU_ATC_SETS*MMU_ATC_WAYS*32-1:0] MMU_ATC_TAG_FLAT,
    output logic [MMU_ATC_SETS*MMU_ATC_WAYS*32-1:0] MMU_ATC_PTAG_FLAT
);

`include "wf68k30L_pkg.svh"
`include "wf68k30L_top_sections/helpers/wf68k30L_top_helpers_mmu_pure.svh"

localparam logic [15:0] MMUSR_B = 16'h8000;
localparam logic [15:0] MMUSR_W = 16'h0800;
localparam logic [15:0] MMUSR_I = 16'h0400;
localparam logic [15:0] MMUSR_M = 16'h0200;
localparam logic [15:0] MMUSR_T = 16'h0040;

logic [MMU_ATC_WAYS-1:0] MMU_ATC_V [0:MMU_ATC_SETS-1];
logic [MMU_ATC_WAYS-1:0] MMU_ATC_B [0:MMU_ATC_SETS-1];
logic [MMU_ATC_WAYS-1:0] MMU_ATC_W [0:MMU_ATC_SETS-1];
logic [MMU_ATC_WAYS-1:0] MMU_ATC_M [0:MMU_ATC_SETS-1];
logic [2:0]  MMU_ATC_FC [0:MMU_ATC_SETS-1][0:MMU_ATC_WAYS-1];
logic [31:0] MMU_ATC_TAG[0:MMU_ATC_SETS-1][0:MMU_ATC_WAYS-1];
logic [31:0] MMU_ATC_PTAG[0:MMU_ATC_SETS-1][0:MMU_ATC_WAYS-1];
logic [MMU_ATC_WAY_BITS-1:0] MMU_ATC_REPL_PTR [0:MMU_ATC_SETS-1];

for (genvar set_i = 0; set_i < MMU_ATC_SETS; set_i = set_i + 1) begin : gen_pack_mmu_atc_set
    for (genvar way_i = 0; way_i < MMU_ATC_WAYS; way_i = way_i + 1) begin : gen_pack_mmu_atc_way
        localparam int IDX = (set_i * MMU_ATC_WAYS) + way_i;
        assign MMU_ATC_V_FLAT[IDX] = MMU_ATC_V[set_i][way_i];
        assign MMU_ATC_B_FLAT[IDX] = MMU_ATC_B[set_i][way_i];
        assign MMU_ATC_W_FLAT[IDX] = MMU_ATC_W[set_i][way_i];
        assign MMU_ATC_M_FLAT[IDX] = MMU_ATC_M[set_i][way_i];
        assign MMU_ATC_FC_FLAT[(IDX*3) +: 3] = MMU_ATC_FC[set_i][way_i];
        assign MMU_ATC_TAG_FLAT[(IDX*32) +: 32] = MMU_ATC_TAG[set_i][way_i];
        assign MMU_ATC_PTAG_FLAT[(IDX*32) +: 32] = MMU_ATC_PTAG[set_i][way_i];
    end
end

function automatic logic mmu_tc_cfg_error(input logic [31:0] tc_in);
    logic [3:0] ps;
    logic [5:0] tl_sum;
    logic [5:0] total;
begin
    mmu_tc_cfg_error = 1'b0;
    if (tc_in[31]) begin
        ps = tc_in[23:20];
        if (ps < 4'h8)
            mmu_tc_cfg_error = 1'b1;

        tl_sum = {2'b00, tc_in[15:12]};
        if (tc_in[15:12] != 4'h0) begin
            tl_sum = tl_sum + {2'b00, tc_in[11:8]};
            if (tc_in[11:8] != 4'h0) begin
                tl_sum = tl_sum + {2'b00, tc_in[7:4]};
                if (tc_in[7:4] != 4'h0)
                    tl_sum = tl_sum + {2'b00, tc_in[3:0]};
            end
        end
        total = tl_sum + {2'b00, tc_in[19:16]} + {2'b00, ps};
        if (total != 6'd32)
            mmu_tc_cfg_error = 1'b1;
    end
end
endfunction

function automatic logic [2:0] mmu_fc_decode(
    input logic [4:0]  fc_sel,
    input logic [31:0] dreg_value,
    input logic [2:0]  sfc_value,
    input logic [2:0]  dfc_value
);
begin
    if (fc_sel[4:3] == 2'b10)
        mmu_fc_decode = fc_sel[2:0];
    else if (fc_sel[4:3] == 2'b01)
        mmu_fc_decode = dreg_value[2:0];
    else if (fc_sel == 5'b00000)
        mmu_fc_decode = sfc_value;
    else
        mmu_fc_decode = dfc_value;
end
endfunction

function automatic logic [MMU_ATC_SET_BITS-1:0] mmu_atc_set_idx(
    input logic [2:0]  fc,
    input logic [31:0] tag
);
    logic [MMU_ATC_SET_BITS-1:0] tag_bits;
    logic [MMU_ATC_SET_BITS-1:0] fc_bits;
begin
    tag_bits = tag[MMU_ATC_SET_BITS-1:0];
    fc_bits = fc[MMU_ATC_SET_BITS-1:0];
    mmu_atc_set_idx = tag_bits ^ fc_bits;
end
endfunction

function automatic logic [35:0] mmu_atc_lookup(
    input logic [2:0]  fc,
    input logic [31:0] tag,
    input logic        write_access
);
    logic [MMU_ATC_SET_BITS-1:0] set_idx;
    logic        hit;
    logic        b;
    logic        w;
    logic        m;
    logic [31:0] ptag;
    integer way;
begin
    set_idx = mmu_atc_set_idx(fc, tag);
    hit = 1'b0;
    b = 1'b0;
    w = 1'b0;
    m = write_access;
    ptag = 32'h0;

    for (way = 0; way < MMU_ATC_WAYS; way = way + 1) begin
        if (!hit &&
            MMU_ATC_V[set_idx][way] &&
            MMU_ATC_FC[set_idx][way] == fc &&
            MMU_ATC_TAG[set_idx][way] == tag) begin
            hit = 1'b1;
            ptag = MMU_ATC_PTAG[set_idx][way];
            b = MMU_ATC_B[set_idx][way];
            w = MMU_ATC_W[set_idx][way];
            m = MMU_ATC_M[set_idx][way] || write_access;
        end
    end

    mmu_atc_lookup = {hit, b, w, m, ptag};
end
endfunction

function automatic logic [2*MMU_ATC_WAY_BITS+1:0] mmu_atc_probe_set(
    input logic [MMU_ATC_SET_BITS-1:0] set_idx,
    input logic [2:0]                  fc,
    input logic [31:0]                 tag
);
    logic hit;
    logic free;
    logic [MMU_ATC_WAY_BITS-1:0] hit_way;
    logic [MMU_ATC_WAY_BITS-1:0] free_way;
    integer way;
begin
    hit = 1'b0;
    free = 1'b0;
    hit_way = '0;
    free_way = '0;
    for (way = 0; way < MMU_ATC_WAYS; way = way + 1) begin
        if (!hit &&
            MMU_ATC_V[set_idx][way] &&
            MMU_ATC_FC[set_idx][way] == fc &&
            MMU_ATC_TAG[set_idx][way] == tag) begin
            hit = 1'b1;
            hit_way = way[MMU_ATC_WAY_BITS-1:0];
        end
        if (!free && !MMU_ATC_V[set_idx][way]) begin
            free = 1'b1;
            free_way = way[MMU_ATC_WAY_BITS-1:0];
        end
    end
    mmu_atc_probe_set = {hit, free, hit_way, free_way};
end
endfunction

always_ff @(posedge CLK) begin : mmu_registers
    logic        tc_cfg_err;
    logic        ptest_exec;
    logic        pload_exec;
    logic        pflush_exec;
    logic        pmove_flush_exec;
    logic [2:0]  fc_sel;
    logic [2:0]  ptest_level;
    logic [2:0]  atc_fc;
    logic [2:0]  atc_mask;
    logic [31:0] atc_logical;
    logic [31:0] atc_tag;
    logic [31:0] atc_ptag;
    logic [63:0] root_ptr;
    logic [31:0] root_offs;
    logic [15:0] mmusr_value;
    logic        tt_hit;
    logic        atc_hit;
    logic        atc_free;
    logic        atc_valid_result;
    logic        atc_b_result;
    logic        atc_w_result;
    logic        atc_m_result;
    logic        atc_rmw;
    logic [31:0] atc_phys;
    logic [35:0] atc_lookup;
    logic [MMU_ATC_SET_BITS-1:0] atc_set_idx;
    logic [MMU_ATC_WAY_BITS-1:0] atc_hit_way;
    logic [MMU_ATC_WAY_BITS-1:0] atc_free_way;
    logic [MMU_ATC_WAY_BITS-1:0] atc_ins_way;
    logic [2*MMU_ATC_WAY_BITS+1:0] atc_probe;
    integer set_i;
    integer way_i;
    if (RESET_CPU) begin
        MMU_SRP <= 64'h0;
        MMU_CRP <= 64'h0;
        MMU_TC <= 32'h0;
        MMU_TT0 <= 32'h0;
        MMU_TT1 <= 32'h0;
        MMU_MMUSR <= 32'h0;
        TRAP_MMU_CFG <= 1'b0;
        MMU_ATC_FLUSH_COUNT <= 32'h0;
        for (set_i = 0; set_i < MMU_ATC_SETS; set_i = set_i + 1) begin
            MMU_ATC_V[set_i] <= '0;
            MMU_ATC_B[set_i] <= '0;
            MMU_ATC_W[set_i] <= '0;
            MMU_ATC_M[set_i] <= '0;
            MMU_ATC_REPL_PTR[set_i] <= '0;
            for (way_i = 0; way_i < MMU_ATC_WAYS; way_i = way_i + 1) begin
                MMU_ATC_FC[set_i][way_i] <= 3'b000;
                MMU_ATC_TAG[set_i][way_i] <= 32'h0;
                MMU_ATC_PTAG[set_i][way_i] <= 32'h0;
            end
        end
    end else begin
        tc_cfg_err = mmu_tc_cfg_error(ALU_RESULT[31:0]);
        ptest_exec = ALU_ACK && (OP_WB == PTEST);
        pload_exec = ALU_ACK && (OP_WB == PLOAD);
        pmove_flush_exec = MMU_ATC_FLUSH &&
                           (MMU_TC_WR || MMU_SRP_WR || MMU_CRP_WR || MMU_TT0_WR || MMU_TT1_WR);
        pflush_exec = MMU_ATC_FLUSH && !pmove_flush_exec;
        TRAP_MMU_CFG <= 1'b0;
        if (MMU_RUNTIME_REQ && MMU_RUNTIME_FAULT)
            TRAP_MMU_CFG <= 1'b1;
        if (MMU_ATC_FLUSH)
            MMU_ATC_FLUSH_COUNT <= MMU_ATC_FLUSH_COUNT + 32'd1;
        if (MMU_SRP_WR) begin
            MMU_SRP <= ALU_RESULT[63:0];
            if (ALU_RESULT[33:32] == 2'b00)
                TRAP_MMU_CFG <= 1'b1;
        end
        if (MMU_CRP_WR) begin
            MMU_CRP <= ALU_RESULT[63:0];
            if (ALU_RESULT[33:32] == 2'b00)
                TRAP_MMU_CFG <= 1'b1;
        end
        if (MMU_TC_WR) begin
            MMU_TC <= tc_cfg_err ? {1'b0, ALU_RESULT[30:0]} : ALU_RESULT[31:0];
            if (tc_cfg_err)
                TRAP_MMU_CFG <= 1'b1;
        end
        if (MMU_TT0_WR)
            MMU_TT0 <= ALU_RESULT[31:0];
        if (MMU_TT1_WR)
            MMU_TT1 <= ALU_RESULT[31:0];
        if (MMU_MMUSR_WR)
            MMU_MMUSR <= {16'h0, ALU_RESULT[15:0]};

        if (pmove_flush_exec) begin
            for (set_i = 0; set_i < MMU_ATC_SETS; set_i = set_i + 1)
                MMU_ATC_V[set_i] <= '0;
        end

        if (pflush_exec) begin
            atc_fc = mmu_fc_decode(BIW_1[4:0], DR_OUT_1, SFC, DFC);
            atc_mask = BIW_1[7:5];
            atc_logical = ADR_EFF;
            atc_tag = mmu_page_tag(MMU_TC, atc_logical);
            case (BIW_1[12:10])
                3'b001: begin
                    for (set_i = 0; set_i < MMU_ATC_SETS; set_i = set_i + 1)
                        MMU_ATC_V[set_i] <= '0;
                end
                3'b100: begin
                    for (set_i = 0; set_i < MMU_ATC_SETS; set_i = set_i + 1) begin
                        for (way_i = 0; way_i < MMU_ATC_WAYS; way_i = way_i + 1) begin
                            if (MMU_ATC_V[set_i][way_i] &&
                                (((MMU_ATC_FC[set_i][way_i] ^ atc_fc) & atc_mask) == 3'b000))
                                MMU_ATC_V[set_i][way_i] <= 1'b0;
                        end
                    end
                end
                3'b110: begin
                    for (set_i = 0; set_i < MMU_ATC_SETS; set_i = set_i + 1) begin
                        for (way_i = 0; way_i < MMU_ATC_WAYS; way_i = way_i + 1) begin
                            if (MMU_ATC_V[set_i][way_i] &&
                                (((MMU_ATC_FC[set_i][way_i] ^ atc_fc) & atc_mask) == 3'b000) &&
                                (MMU_ATC_TAG[set_i][way_i] == atc_tag))
                                MMU_ATC_V[set_i][way_i] <= 1'b0;
                        end
                    end
                end
                default: begin
                end
            endcase
        end

        if (pload_exec) begin
            atc_fc = mmu_fc_decode(BIW_1[4:0], DR_OUT_1, SFC, DFC);
            atc_logical = ADR_EFF;
            atc_tag = mmu_page_tag(MMU_TC, atc_logical);
            atc_rmw = 1'b0;

            tt_hit = mmu_tt_match(MMU_TT0, atc_fc, atc_logical, BIW_1[9], !BIW_1[9], atc_rmw) ||
                     mmu_tt_match(MMU_TT1, atc_fc, atc_logical, BIW_1[9], !BIW_1[9], atc_rmw);
            root_ptr = (MMU_TC[25] && atc_fc[2]) ? MMU_SRP : MMU_CRP;
            root_offs = {root_ptr[31:4], 4'b0000};
            atc_valid_result = (!tt_hit) && (atc_fc != FC_CPU_SPACE) && (root_ptr[33:32] == 2'b01);
            atc_phys = atc_valid_result ? (atc_logical + root_offs) : 32'h0;
            atc_ptag = mmu_page_tag(MMU_TC, atc_phys);
            atc_b_result = !atc_valid_result;
            atc_w_result = 1'b0;
            atc_m_result = !BIW_1[9];

            atc_set_idx = mmu_atc_set_idx(atc_fc, atc_tag);
            atc_probe = mmu_atc_probe_set(atc_set_idx, atc_fc, atc_tag);
            atc_hit = atc_probe[2*MMU_ATC_WAY_BITS+1];
            atc_free = atc_probe[2*MMU_ATC_WAY_BITS];
            atc_hit_way = atc_probe[2*MMU_ATC_WAY_BITS-1:MMU_ATC_WAY_BITS];
            atc_free_way = atc_probe[MMU_ATC_WAY_BITS-1:0];

            atc_ins_way = atc_hit ? atc_hit_way : (atc_free ? atc_free_way : MMU_ATC_REPL_PTR[atc_set_idx]);

            if (!tt_hit) begin
                for (way_i = 0; way_i < MMU_ATC_WAYS; way_i = way_i + 1) begin
                    if (MMU_ATC_V[atc_set_idx][way_i] &&
                        MMU_ATC_FC[atc_set_idx][way_i] == atc_fc &&
                        MMU_ATC_TAG[atc_set_idx][way_i] == atc_tag)
                        MMU_ATC_V[atc_set_idx][way_i] <= 1'b0;
                end
                MMU_ATC_V[atc_set_idx][atc_ins_way] <= 1'b1;
                MMU_ATC_B[atc_set_idx][atc_ins_way] <= atc_b_result;
                MMU_ATC_W[atc_set_idx][atc_ins_way] <= atc_w_result;
                MMU_ATC_M[atc_set_idx][atc_ins_way] <= atc_m_result;
                MMU_ATC_FC[atc_set_idx][atc_ins_way] <= atc_fc;
                MMU_ATC_TAG[atc_set_idx][atc_ins_way] <= atc_tag;
                MMU_ATC_PTAG[atc_set_idx][atc_ins_way] <= atc_ptag;
                if (!atc_hit && !atc_free)
                    MMU_ATC_REPL_PTR[atc_set_idx] <= MMU_ATC_REPL_PTR[atc_set_idx] + 1'b1;
            end
        end

        if (MMU_RUNTIME_ATC_REFILL) begin
            atc_fc = MMU_RUNTIME_ATC_FC;
            atc_tag = MMU_RUNTIME_ATC_TAG;
            atc_ptag = MMU_RUNTIME_ATC_PTAG;
            atc_b_result = MMU_RUNTIME_ATC_B;
            atc_w_result = MMU_RUNTIME_ATC_W;
            atc_m_result = MMU_RUNTIME_ATC_M;

            atc_set_idx = mmu_atc_set_idx(atc_fc, atc_tag);
            atc_probe = mmu_atc_probe_set(atc_set_idx, atc_fc, atc_tag);
            atc_hit = atc_probe[2*MMU_ATC_WAY_BITS+1];
            atc_free = atc_probe[2*MMU_ATC_WAY_BITS];
            atc_hit_way = atc_probe[2*MMU_ATC_WAY_BITS-1:MMU_ATC_WAY_BITS];
            atc_free_way = atc_probe[MMU_ATC_WAY_BITS-1:0];

            atc_ins_way = atc_hit ? atc_hit_way : (atc_free ? atc_free_way : MMU_ATC_REPL_PTR[atc_set_idx]);

            for (way_i = 0; way_i < MMU_ATC_WAYS; way_i = way_i + 1) begin
                if (MMU_ATC_V[atc_set_idx][way_i] &&
                    MMU_ATC_FC[atc_set_idx][way_i] == atc_fc &&
                    MMU_ATC_TAG[atc_set_idx][way_i] == atc_tag)
                    MMU_ATC_V[atc_set_idx][way_i] <= 1'b0;
            end
            MMU_ATC_V[atc_set_idx][atc_ins_way] <= 1'b1;
            MMU_ATC_B[atc_set_idx][atc_ins_way] <= atc_b_result;
            MMU_ATC_W[atc_set_idx][atc_ins_way] <= atc_w_result;
            MMU_ATC_M[atc_set_idx][atc_ins_way] <= atc_m_result;
            MMU_ATC_FC[atc_set_idx][atc_ins_way] <= atc_fc;
            MMU_ATC_TAG[atc_set_idx][atc_ins_way] <= atc_tag;
            MMU_ATC_PTAG[atc_set_idx][atc_ins_way] <= atc_ptag;
            if (!atc_hit && !atc_free)
                MMU_ATC_REPL_PTR[atc_set_idx] <= MMU_ATC_REPL_PTR[atc_set_idx] + 1'b1;
        end

        if (ptest_exec) begin
            atc_fc = mmu_fc_decode(BIW_1[4:0], DR_OUT_1, SFC, DFC);
            ptest_level = BIW_1[12:10];
            atc_logical = ADR_EFF;
            atc_tag = mmu_page_tag(MMU_TC, atc_logical);
            atc_rmw = 1'b0;

            tt_hit = mmu_tt_match(MMU_TT0, atc_fc, atc_logical, BIW_1[9], !BIW_1[9], atc_rmw) ||
                     mmu_tt_match(MMU_TT1, atc_fc, atc_logical, BIW_1[9], !BIW_1[9], atc_rmw);

            atc_lookup = mmu_atc_lookup(atc_fc, atc_tag, 1'b0);
            atc_hit = atc_lookup[35];
            atc_b_result = atc_lookup[34];
            atc_w_result = atc_lookup[33];
            atc_m_result = atc_lookup[32];

            mmusr_value = 16'h0000;
            if (ptest_level == 3'b000) begin
                if (tt_hit)
                    mmusr_value = MMUSR_T;
                else if (!atc_hit)
                    mmusr_value = MMUSR_I;
                else begin
                    if (atc_b_result)
                        mmusr_value = mmusr_value | MMUSR_B | MMUSR_I;
                    if (atc_w_result)
                        mmusr_value = mmusr_value | MMUSR_W;
                    if (atc_m_result)
                        mmusr_value = mmusr_value | MMUSR_M;
                end
            end else begin
                mmusr_value = PTEST_WALK_MMUSR;
            end
            MMU_MMUSR <= {16'h0, mmusr_value};
        end
    end
end

endmodule
