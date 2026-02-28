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
    logic [31:0] idx;
    logic [5:0] bitpos;
    integer j;
begin
    idx = 32'h0;
    for (j = 0; j < 16; j = j + 1) begin
        if (j < width) begin
            bitpos = {2'b00, initial_shift} + consumed_bits + j[5:0];
            if (bitpos < 6'd32)
                idx = {idx[30:0], logical_addr[31 - bitpos]};
            else
                idx = {idx[30:0], 1'b0};
        end
    end
    mmu_index_extract = idx;
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

function automatic logic [32:0] mmu_desc_shadow_read(input logic [31:0] addr);
    logic hit;
    logic [31:0] data;
    logic [31:0] addr_w;
    integer i;
begin
    hit = 1'b0;
    data = 32'h0;
    addr_w = {addr[31:2], 2'b00};
    for (i = 0; i < MMU_DESC_SHADOW_LINES; i = i + 1) begin
        if (!hit && MMU_DESC_SHADOW_V[i] && MMU_DESC_SHADOW_ADDR[i] == addr_w) begin
            hit = 1'b1;
            data = MMU_DESC_SHADOW_DATA[i];
        end
    end
    mmu_desc_shadow_read = {hit, data};
end
endfunction

