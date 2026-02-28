// MMU cache policy helpers (TT CI matching + CPU-space override).
function automatic logic mmu_cache_inhibit(
    input logic [2:0]  fc,
    input logic [31:0] addr,
    input logic        read_access,
    input logic        write_access,
    input logic        rmw_access,
    input logic [31:0] tt0,
    input logic [31:0] tt1
);
    logic ci_out;
begin
    ci_out = mmu_ci_out(fc, addr, read_access, write_access, rmw_access, tt0, tt1);
    if (fc == FC_CPU_SPACE)
        mmu_cache_inhibit = 1'b1;
    else
        mmu_cache_inhibit = ci_out;
end
endfunction

function automatic logic mmu_ci_out(
    input logic [2:0]  fc,
    input logic [31:0] addr,
    input logic        read_access,
    input logic        write_access,
    input logic        rmw_access,
    input logic [31:0] tt0,
    input logic [31:0] tt1
);
    logic tt0_hit;
    logic tt1_hit;
begin
    tt0_hit = mmu_tt_match(tt0, fc, addr, read_access, write_access, rmw_access);
    tt1_hit = mmu_tt_match(tt1, fc, addr, read_access, write_access, rmw_access);
    mmu_ci_out = (tt0_hit && tt0[10]) || (tt1_hit && tt1[10]); // CI bit.
end
endfunction

// D-cache line access helpers (alignment, lane select, write merge).
function automatic logic dcache_access_supported(
    input OP_SIZETYPE  size_in,
    input logic [1:0]  a10
);
begin
    case (size_in)
        BYTE: dcache_access_supported = 1'b1;
        WORD: dcache_access_supported = !a10[0];
        LONG: dcache_access_supported = (a10 == 2'b00);
        default: dcache_access_supported = 1'b0;
    endcase
end
endfunction

function automatic logic [31:0] dcache_read_extract(
    input logic [31:0] line_word,
    input OP_SIZETYPE  size_in,
    input logic [1:0]  a10
);
begin
    case (size_in)
        LONG: dcache_read_extract = line_word;
        WORD: dcache_read_extract = a10[1] ? {16'h0000, line_word[15:0]} : {16'h0000, line_word[31:16]};
        BYTE: begin
            case (a10)
                2'b00: dcache_read_extract = {24'h000000, line_word[31:24]};
                2'b01: dcache_read_extract = {24'h000000, line_word[23:16]};
                2'b10: dcache_read_extract = {24'h000000, line_word[15:8]};
                default: dcache_read_extract = {24'h000000, line_word[7:0]};
            endcase
        end
        default: dcache_read_extract = 32'h0000_0000;
    endcase
end
endfunction

function automatic logic [31:0] dcache_write_merge(
    input logic [31:0] line_word,
    input logic [31:0] wr_data,
    input OP_SIZETYPE  size_in,
    input logic [1:0]  a10
);
begin
    dcache_write_merge = line_word;
    case (size_in)
        LONG: begin
            if (a10 == 2'b00)
                dcache_write_merge = wr_data;
        end
        WORD: begin
            if (!a10[0]) begin
                if (a10[1])
                    dcache_write_merge = {line_word[31:16], wr_data[15:0]};
                else
                    dcache_write_merge = {wr_data[15:0], line_word[15:0]};
            end
        end
        BYTE: begin
            case (a10)
                2'b00: dcache_write_merge = {wr_data[7:0], line_word[23:0]};
                2'b01: dcache_write_merge = {line_word[31:24], wr_data[7:0], line_word[15:0]};
                2'b10: dcache_write_merge = {line_word[31:16], wr_data[7:0], line_word[7:0]};
                default: dcache_write_merge = {line_word[31:8], wr_data[7:0]};
            endcase
        end
        default: ;
    endcase
end
endfunction
