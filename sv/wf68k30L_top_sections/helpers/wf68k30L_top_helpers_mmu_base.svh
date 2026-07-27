// ========================================================================
// MMU/cache helper functions
// ========================================================================

function automatic logic mmu_tc_cfg_error(input logic [31:0] tc_in);
    logic [3:0] ps;
    logic [5:0] tl_sum;
    logic [5:0] total;
begin
    mmu_tc_cfg_error = 1'b0;
    if (tc_in[31]) begin
        ps = tc_in[23:20];
        // PS values 0..7 are reserved.
        if (ps < 4'h8)
            mmu_tc_cfg_error = 1'b1;

        // Sum TIA..TID until the first zero field, then add IS and PS.
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

// The eight pure page/table-walk helpers below used to be copied out here in
// full -- 147 duplicated lines, differing from the original only by a redundant
// pair of parentheses and one reworded comment. They are included instead, so
// the top level and the five extracted MMU modules cannot drift apart. Only the
// functions kept in this file need module scope: mmu_atc_lookup and
// mmu_atc_probe_set read MMU_ATC_*_FLAT, and mmu_atc_set_idx returns a width
// derived from MMU_ATC_SET_BITS, none of which exist in the pure header.
`include "wf68k30L_top_sections/helpers/wf68k30L_top_helpers_mmu_pure.svh"

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

// Packed ATC lookup return:
// [35]    hit
// [34]    B (bus/invalid fault marker)
// [33]    W (write-protect marker)
// [32]    M (modified marker after access)
// [31:0]  physical page tag
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
    logic [2:0]  atc_way_fc;
    logic [31:0] atc_way_tag;
    logic [31:0] atc_way_ptag;
    integer atc_way_idx;
    integer way;
begin
    set_idx = mmu_atc_set_idx(fc, tag);
    hit = 1'b0;
    b = 1'b0;
    w = 1'b0;
    m = write_access;
    ptag = 32'h0;

    for (way = 0; way < MMU_ATC_WAYS; way = way + 1) begin
        atc_way_idx = (set_idx * MMU_ATC_WAYS) + way;
        atc_way_fc = MMU_ATC_FC_FLAT[(atc_way_idx*3) +: 3];
        atc_way_tag = MMU_ATC_TAG_FLAT[(atc_way_idx*32) +: 32];
        atc_way_ptag = MMU_ATC_PTAG_FLAT[(atc_way_idx*32) +: 32];
        if (!hit &&
            MMU_ATC_V_FLAT[atc_way_idx] &&
            atc_way_fc == fc &&
            atc_way_tag == tag) begin
            hit = 1'b1;
            ptag = atc_way_ptag;
            b = MMU_ATC_B_FLAT[atc_way_idx];
            w = MMU_ATC_W_FLAT[atc_way_idx];
            m = MMU_ATC_M_FLAT[atc_way_idx] || write_access;
        end
    end

    mmu_atc_lookup = {hit, b, w, m, ptag};
end
endfunction

// Packed ATC probe return:
// [2*MMU_ATC_WAY_BITS+1]     hit
// [2*MMU_ATC_WAY_BITS]       free-way-available
// [2*MMU_ATC_WAY_BITS-1:MMU_ATC_WAY_BITS] hit way index
// [MMU_ATC_WAY_BITS-1:0]     free way index
function automatic logic [2*MMU_ATC_WAY_BITS+1:0] mmu_atc_probe_set(
    input logic [MMU_ATC_SET_BITS-1:0] set_idx,
    input logic [2:0]                  fc,
    input logic [31:0]                 tag
);
    logic hit;
    logic free;
    logic [MMU_ATC_WAY_BITS-1:0] hit_way;
    logic [MMU_ATC_WAY_BITS-1:0] free_way;
    logic [2:0]  atc_way_fc;
    logic [31:0] atc_way_tag;
    integer atc_way_idx;
    integer way;
begin
    hit = 1'b0;
    free = 1'b0;
    hit_way = '0;
    free_way = '0;
    for (way = 0; way < MMU_ATC_WAYS; way = way + 1) begin
        atc_way_idx = (set_idx * MMU_ATC_WAYS) + way;
        atc_way_fc = MMU_ATC_FC_FLAT[(atc_way_idx*3) +: 3];
        atc_way_tag = MMU_ATC_TAG_FLAT[(atc_way_idx*32) +: 32];
        if (!hit &&
            MMU_ATC_V_FLAT[atc_way_idx] &&
            atc_way_fc == fc &&
            atc_way_tag == tag) begin
            hit = 1'b1;
            hit_way = way[MMU_ATC_WAY_BITS-1:0];
        end
        if (!free && !MMU_ATC_V_FLAT[atc_way_idx]) begin
            free = 1'b1;
            free_way = way[MMU_ATC_WAY_BITS-1:0];
        end
    end
    mmu_atc_probe_set = {hit, free, hit_way, free_way};
end
endfunction
