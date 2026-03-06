// MMU helper functions that only depend on scalar inputs.
// These are shared by extracted routing modules.

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
        la_ok = (((addr[31:24] ^ tt[31:24]) & ~tt[23:16]) == 8'h00);
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

function automatic logic [4:0] mmu_page_shift(input logic [31:0] tc_in);
begin
    if (tc_in[23:20] >= 4'h8)
        mmu_page_shift = {1'b0, tc_in[23:20]};
    else
        mmu_page_shift = 5'd12; // Reserved PS values map as 4K in this model.
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
