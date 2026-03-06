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

function automatic logic mmu_tt_match(
    input logic [31:0] tt,
    input logic [2:0]  fc,
    input logic [31:0] addr,
    input logic        read_access,
    input logic        write_access,
    input logic        rmw_access
);
    logic fc_ok;
    logic la_ok;
    logic rw_ok;
begin
    // TT format (MC68030): [31:24] logical base, [23:16] logical mask,
    // [15] E, [10] CI, [9] R/W, [8] RWM, [7] 0, [6:4] FC base, [3] 0, [2:0] FC mask.
    if (!tt[15] || fc == FC_CPU_SPACE) begin
        mmu_tt_match = 1'b0;
    end else begin
        fc_ok = (((fc ^ tt[6:4]) & ~tt[2:0]) == 3'b000);
        la_ok = ((((addr[31:24] ^ tt[31:24]) & ~tt[23:16]) == 8'h00));
        if (rmw_access)
            rw_ok = tt[8];
        else if (tt[8])
            rw_ok = 1'b1;
        else
            rw_ok = (read_access && tt[9]) || (write_access && !tt[9]);
        mmu_tt_match = fc_ok && la_ok && rw_ok;
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

function automatic logic [4:0] mmu_page_shift(input logic [31:0] tc_in);
begin
    if (tc_in[23:20] >= 4'h8)
        mmu_page_shift = {1'b0, tc_in[23:20]};
    else
        mmu_page_shift = 5'd12; // Reserved PS values are treated as 4 Kbytes in this model.
end
endfunction

function automatic logic [31:0] mmu_page_tag(
    input logic [31:0] tc_in,
    input logic [31:0] addr
);
    logic [4:0] shift;
begin
    shift = mmu_page_shift(tc_in);
    mmu_page_tag = addr >> shift;
end
endfunction

function automatic logic [31:0] mmu_page_compose_addr(
    input logic [31:0] tc_in,
    input logic [31:0] ptag,
    input logic [31:0] logical_addr
);
    logic [4:0] shift;
    logic [31:0] page_mask;
begin
    shift = mmu_page_shift(tc_in);
    if (shift == 5'd0)
        page_mask = 32'h0000_0000;
    else
        page_mask = 32'hFFFF_FFFF >> (6'd32 - {1'b0, shift});
    mmu_page_compose_addr = (ptag << shift) | (logical_addr & page_mask);
end
endfunction

function automatic logic [31:0] mmu_index_extract(
    input logic [31:0] logical_addr,
    input logic [3:0]  initial_shift,
    input logic [5:0]  consumed_bits,
    input logic [3:0]  width
);
    logic [5:0]  start_bit;
    logic [31:0] bit_stream;
    logic [31:0] width_mask;
begin
    start_bit = {2'b00, initial_shift} + consumed_bits;
    if (start_bit >= 6'd32)
        bit_stream = 32'h0000_0000;
    else
        bit_stream = logical_addr << start_bit;

    if (width == 4'h0) begin
        mmu_index_extract = 32'h0000_0000;
    end else begin
        width_mask = (32'h1 << width) - 32'h1;
        mmu_index_extract = (bit_stream >> (6'd32 - {2'b00, width})) & width_mask;
    end
end
endfunction

function automatic logic [3:0] mmu_tlx_width(
    input logic [31:0] tc_in,
    input logic [1:0]  tlx_sel
);
begin
    case (tlx_sel)
        2'd0: mmu_tlx_width = tc_in[15:12];
        2'd1: mmu_tlx_width = tc_in[11:8];
        2'd2: mmu_tlx_width = tc_in[7:4];
        default: mmu_tlx_width = tc_in[3:0];
    endcase
end
endfunction

function automatic logic mmu_limit_violation(
    input logic        limit_lower,
    input logic [14:0] limit,
    input logic [31:0] index
);
begin
    mmu_limit_violation = (limit_lower && index[14:0] < limit) ||
                          (!limit_lower && index[14:0] > limit);
end
endfunction

function automatic logic [31:0] mmu_desc_addr(
    input logic [31:0] table_base,
    input logic [31:0] index,
    input logic [3:0]  desc_size
);
begin
    if (desc_size == 4'd8)
        mmu_desc_addr = table_base + (index << 3);
    else
        mmu_desc_addr = table_base + (index << 2);
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
