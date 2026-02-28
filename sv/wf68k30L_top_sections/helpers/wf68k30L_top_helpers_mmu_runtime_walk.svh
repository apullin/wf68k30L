// Runtime MMU table walk helper.
// Returns packed fields:
// [31:0] translated physical address
// [32]   walk fault
// [33]   one-cycle deterministic stall request
// [34]   write-protect accumulated state (ATC W)
// [35]   modified state (ATC M)
function automatic logic [35:0] mmu_runtime_table_walk(
    input logic [31:0] logical_addr,
    input logic [2:0]  fc_in,
    input logic [31:0] tc_in,
    input logic [63:0] root_ptr,
    input logic        write_access,
    input logic        req_now,
    input logic        walk_delay_armed
);
    logic        root_short_table;
    logic        root_long_table;
    logic [14:0] root_limit;
    logic        root_limit_lower;
    logic [31:0] walk_table_base;
    logic [31:0] walk_desc_addr;
    logic [31:0] walk_desc;
    logic [31:0] walk_desc_hi;
    logic [31:0] walk_desc_ptr;
    logic [31:0] walk_page_desc;
    logic [31:0] walk_page_base;
    logic [31:0] walk_index;
    logic [31:0] walk_limit_index;
    logic [31:0] walk_phys;
    logic [5:0]  walk_consumed;
    logic [5:0]  walk_used_bits;
    logic [31:0] walk_offset_mask;
    logic [3:0]  walk_width;
    logic [3:0]  walk_next_width;
    logic        walk_has_next;
    logic        walk_done;
    logic        walk_fault;
    logic        walk_limit_fault;
    logic        walk_super_fault;
    logic        walk_wp_accum;
    logic        walk_page_m;
    logic [32:0] walk_lookup;
    logic [32:0] walk_lookup_hi;
    logic [2:0]  walk_desc_size;
    logic [1:0]  walk_desc_dt;
    logic [2:0]  walk_tlx_level;
    logic        walk_fc_level;
    logic [1:0]  walk_tlx_sel;
    logic [1:0]  walk_tlx_next_sel;
    logic        walk_limit_lower;
    logic [14:0] walk_limit;
    logic [2:0]  walk_levels;
    logic        walk_delay_block;
    integer      level;
begin
    mmu_runtime_table_walk = '0;
    mmu_runtime_table_walk[31:0] = logical_addr;

    root_short_table = (root_ptr[33:32] == 2'b10);
    root_long_table = (root_ptr[33:32] == 2'b11);
    root_limit = root_ptr[62:48];
    root_limit_lower = root_ptr[63];

    walk_delay_block = req_now && !walk_delay_armed;
    mmu_runtime_table_walk[33] = walk_delay_block;

    if (!root_short_table && !root_long_table) begin
        mmu_runtime_table_walk[32] = 1'b1;
    end else begin
        walk_table_base = {root_ptr[31:4], 4'b0000};
        walk_page_desc = 32'h0;
        walk_page_base = logical_addr;
        walk_phys = logical_addr;
        walk_consumed = 6'd0;
        walk_used_bits = 6'd0;
        walk_offset_mask = 32'h0;
        walk_done = 1'b0;
        walk_fault = 1'b0;
        walk_limit_fault = 1'b0;
        walk_super_fault = 1'b0;
        walk_wp_accum = 1'b0;
        walk_page_m = write_access;
        walk_desc = 32'h0;
        walk_desc_hi = 32'h0;
        walk_desc_ptr = 32'h0;
        walk_lookup_hi = 33'h0;
        walk_desc_size = root_long_table ? 3'd8 : 3'd4;
        walk_desc_dt = 2'b00;
        walk_tlx_level = 3'd0;
        walk_fc_level = tc_in[24];
        walk_levels = 3'd0;
        walk_limit_lower = 1'b0;
        walk_limit = 15'h0;
        walk_limit_index = 32'h0;

        for (level = 0; level < 5; level = level + 1) begin
            if (!walk_done && !walk_fault) begin
                walk_index = 32'h0;
                walk_width = 4'h0;
                walk_next_width = 4'h0;
                walk_has_next = 1'b0;
                walk_tlx_sel = walk_tlx_level[1:0];
                walk_tlx_next_sel = (walk_tlx_level == 3'd3) ? 2'd3 : (walk_tlx_level[1:0] + 2'd1);

                if (walk_fc_level) begin
                    walk_index = {29'h0, fc_in};
                    walk_fc_level = 1'b0;
                    walk_next_width = mmu_tlx_width(tc_in, walk_tlx_sel);
                    walk_has_next = (walk_next_width != 4'h0);
                end else begin
                    walk_width = mmu_tlx_width(tc_in, walk_tlx_sel);
                    if (walk_width == 4'h0) begin
                        walk_fault = 1'b1;
                        walk_limit_fault = 1'b1;
                    end else begin
                        walk_index = mmu_index_extract(logical_addr, tc_in[19:16], walk_consumed, walk_width);
                        walk_consumed = walk_consumed + {2'b00, walk_width};
                        walk_tlx_level = walk_tlx_level + 3'd1;

                        if (walk_tlx_level < 3'd4) begin
                            walk_tlx_next_sel = walk_tlx_level[1:0];
                            walk_next_width = mmu_tlx_width(tc_in, walk_tlx_next_sel);
                            walk_has_next = (walk_next_width != 4'h0);
                        end

                        if (walk_levels == 3'd0 && !tc_in[24] &&
                            ((root_limit_lower && walk_index[14:0] < root_limit) ||
                             (!root_limit_lower && walk_index[14:0] > root_limit))) begin
                            walk_fault = 1'b1;
                            walk_limit_fault = 1'b1;
                        end
                    end
                end

                if (!walk_fault) begin
                    if (walk_desc_size == 3'd8)
                        walk_desc_addr = walk_table_base + (walk_index << 3);
                    else
                        walk_desc_addr = walk_table_base + (walk_index << 2);

                    walk_lookup = mmu_desc_shadow_read(walk_desc_addr);
                    walk_desc = 32'h0;
                    walk_desc_hi = 32'h0;
                    if (!walk_lookup[32]) begin
                        walk_fault = 1'b1;
                    end else begin
                        walk_desc = walk_lookup[31:0];
                        if (walk_desc_size == 3'd8) begin
                            walk_lookup_hi = mmu_desc_shadow_read(walk_desc_addr + 32'd4);
                            if (!walk_lookup_hi[32]) begin
                                walk_fault = 1'b1;
                            end else begin
                                walk_desc_hi = walk_lookup_hi[31:0];
                            end
                        end
                    end
                end

                if (!walk_fault) begin
                    walk_levels = walk_levels + 3'd1;
                    walk_desc_dt = walk_desc[1:0];
                    walk_wp_accum = walk_wp_accum || walk_desc[2];

                    if (walk_desc_size == 3'd8 && !fc_in[2] && walk_desc[8]) begin
                        walk_super_fault = 1'b1;
                        walk_fault = 1'b1;
                    end

                    case (walk_desc_dt)
                        2'b00: begin
                            walk_fault = 1'b1;
                        end
                        2'b01: begin
                            if (walk_desc_size == 3'd8 && walk_has_next && walk_next_width != 4'h0) begin
                                walk_limit_lower = walk_desc[31];
                                walk_limit = walk_desc[30:16];
                                walk_limit_index = mmu_index_extract(
                                    logical_addr,
                                    tc_in[19:16],
                                    walk_consumed,
                                    walk_next_width
                                );
                                if ((walk_limit_lower && walk_limit_index[14:0] < walk_limit) ||
                                    (!walk_limit_lower && walk_limit_index[14:0] > walk_limit)) begin
                                    walk_fault = 1'b1;
                                    walk_limit_fault = 1'b1;
                                end
                            end

                            if (!walk_fault) begin
                                walk_page_desc = walk_desc;
                                walk_page_base = (walk_desc_size == 3'd8) ?
                                                 {walk_desc_hi[31:8], 8'h00} :
                                                 {walk_desc[31:8], 8'h00};
                                walk_page_m = walk_desc[4] || write_access;
                                walk_done = 1'b1;
                            end
                        end
                        default: begin
                            if (walk_has_next) begin
                                if (walk_desc_size == 3'd8 && walk_next_width != 4'h0) begin
                                    walk_limit_lower = walk_desc[31];
                                    walk_limit = walk_desc[30:16];
                                    walk_limit_index = mmu_index_extract(
                                        logical_addr,
                                        tc_in[19:16],
                                        walk_consumed,
                                        walk_next_width
                                    );
                                    if ((walk_limit_lower && walk_limit_index[14:0] < walk_limit) ||
                                        (!walk_limit_lower && walk_limit_index[14:0] > walk_limit)) begin
                                        walk_fault = 1'b1;
                                        walk_limit_fault = 1'b1;
                                    end
                                end

                                if (!walk_fault) begin
                                    walk_table_base = (walk_desc_size == 3'd8) ?
                                                      {walk_desc_hi[31:4], 4'b0000} :
                                                      {walk_desc[31:4], 4'b0000};
                                    walk_desc_size = (walk_desc_dt == 2'b10) ? 3'd4 : 3'd8;
                                end
                            end else begin
                                // Bottom-level table descriptor encodes indirection.
                                walk_desc_ptr = (walk_desc_size == 3'd8) ? walk_desc_hi : walk_desc;
                                if (walk_desc_dt == 2'b10) begin
                                    walk_lookup = mmu_desc_shadow_read({walk_desc_ptr[31:2], 2'b00});
                                    if (!walk_lookup[32] || walk_lookup[1:0] != 2'b01) begin
                                        walk_fault = 1'b1;
                                    end else begin
                                        walk_page_desc = walk_lookup[31:0];
                                        walk_wp_accum = walk_wp_accum || walk_lookup[2];
                                        walk_page_base = {walk_lookup[31:8], 8'h00};
                                        walk_page_m = walk_lookup[4] || write_access;
                                        walk_done = 1'b1;
                                    end
                                end else begin
                                    walk_lookup = mmu_desc_shadow_read({walk_desc_ptr[31:3], 3'b000});
                                    walk_lookup_hi = mmu_desc_shadow_read({walk_desc_ptr[31:3], 3'b000} + 32'd4);
                                    if (!walk_lookup[32] || !walk_lookup_hi[32] || walk_lookup[1:0] != 2'b01) begin
                                        walk_fault = 1'b1;
                                    end else begin
                                        walk_page_desc = walk_lookup[31:0];
                                        walk_wp_accum = walk_wp_accum || walk_lookup[2];
                                        if (!fc_in[2] && walk_lookup[8]) begin
                                            walk_fault = 1'b1;
                                            walk_super_fault = 1'b1;
                                        end
                                        if (!walk_fault) begin
                                            walk_page_base = {walk_lookup_hi[31:8], 8'h00};
                                            walk_page_m = walk_lookup[4] || write_access;
                                            walk_done = 1'b1;
                                        end
                                    end
                                end
                            end
                        end
                    endcase
                end
            end
        end

        if (!walk_fault && !walk_done)
            walk_fault = 1'b1;

        if (!walk_fault) begin
            walk_used_bits = {2'b00, tc_in[19:16]} + walk_consumed;
            if (walk_used_bits >= 6'd32)
                walk_offset_mask = 32'h0000_0000;
            else
                walk_offset_mask = 32'hFFFF_FFFF >> walk_used_bits;
            walk_phys = walk_page_base + (logical_addr & walk_offset_mask);
            if (write_access && walk_wp_accum)
                walk_fault = 1'b1;
        end

        mmu_runtime_table_walk[31:0] = walk_phys;
        mmu_runtime_table_walk[32] = walk_fault;
        mmu_runtime_table_walk[34] = walk_wp_accum;
        mmu_runtime_table_walk[35] = walk_page_m;
    end
end
endfunction
