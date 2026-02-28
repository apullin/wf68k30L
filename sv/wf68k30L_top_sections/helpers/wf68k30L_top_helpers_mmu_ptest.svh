function automatic logic [15:0] mmu_ptest_walk_mmusr(
    input logic [2:0]  fc_in,
    input logic [31:0] logical_addr,
    input logic [2:0]  ptest_level
);
    logic [15:0] mmusr;
    logic [63:0] root_ptr;
    logic [1:0]  root_dt;
    logic        root_valid;
    logic        root_short_table;
    logic        root_long_table;
    logic [14:0] root_limit;
    logic        root_limit_lower;
    logic [31:0] first_index;
    logic [31:0] walk_table_base;
    logic [31:0] walk_desc_addr;
    logic [31:0] walk_desc;
    logic [31:0] walk_desc_hi;
    logic [31:0] walk_desc_ptr;
    logic [31:0] walk_page_desc;
    logic [31:0] walk_index;
    logic [31:0] walk_limit_index;
    logic [5:0]  walk_consumed;
    logic [3:0]  walk_width;
    logic [3:0]  walk_next_width;
    logic        walk_has_next;
    logic [2:0]  walk_tlx_level;
    logic        walk_fc_level;
    logic [1:0]  walk_tlx_sel;
    logic [1:0]  walk_tlx_next_sel;
    logic [2:0]  walk_levels;
    logic [2:0]  walk_search_limit;
    logic [2:0]  walk_desc_size;
    logic [1:0]  walk_desc_dt;
    logic [14:0] walk_limit;
    logic        walk_limit_lower;
    logic        walk_fault;
    logic        walk_done;
    logic        walk_limit_fault;
    logic        walk_super_fault;
    logic        walk_bus_fault;
    logic        walk_invalid_fault;
    logic        walk_wp_accum;
    logic        walk_page_m;
    logic [32:0] walk_lookup;
    logic [32:0] walk_lookup_hi;
    integer      level;
begin
    mmusr = 16'h0000;
    root_ptr = (MMU_TC[25] && fc_in[2]) ? MMU_SRP : MMU_CRP;
    root_dt = root_ptr[33:32];
    root_valid = (root_dt == 2'b01);
    root_short_table = (root_dt == 2'b10);
    root_long_table = (root_dt == 2'b11);
    root_limit = root_ptr[62:48];
    root_limit_lower = root_ptr[63];
    walk_search_limit = (ptest_level == 3'b000) ? 3'd7 : ptest_level;

    if (MMU_TC[24])
        first_index = {29'h0, fc_in};
    else
        first_index = mmu_index_extract(logical_addr, MMU_TC[19:16], 6'd0, MMU_TC[15:12]);

    if (root_valid) begin
        if ((root_limit_lower && first_index[14:0] < root_limit) ||
            (!root_limit_lower && first_index[14:0] > root_limit)) begin
            mmusr = mmusr | MMUSR_L | MMUSR_I;
        end
        mmusr = (mmusr & ~MMUSR_N) | ({13'h0, 3'd0} & MMUSR_N);
    end else if (root_short_table || root_long_table) begin
        walk_table_base = {root_ptr[31:4], 4'b0000};
        walk_page_desc = 32'h0;
        walk_consumed = 6'd0;
        walk_done = 1'b0;
        walk_fault = 1'b0;
        walk_limit_fault = 1'b0;
        walk_super_fault = 1'b0;
        walk_bus_fault = 1'b0;
        walk_invalid_fault = 1'b0;
        walk_wp_accum = 1'b0;
        walk_page_m = 1'b0;
        walk_desc = 32'h0;
        walk_desc_hi = 32'h0;
        walk_desc_ptr = 32'h0;
        walk_lookup_hi = 33'h0;
        walk_desc_size = root_long_table ? 3'd8 : 3'd4;
        walk_desc_dt = 2'b00;
        walk_tlx_level = 3'd0;
        walk_fc_level = MMU_TC[24];
        walk_levels = 3'd0;
        walk_limit_lower = 1'b0;
        walk_limit = 15'h0;
        walk_limit_index = 32'h0;

        // Search translation tables up to requested PTEST level and derive MMUSR bits.
        for (level = 0; level < 6; level = level + 1) begin
            if (!walk_done && !walk_fault && (walk_levels < walk_search_limit)) begin
                walk_index = 32'h0;
                walk_width = 4'h0;
                walk_next_width = 4'h0;
                walk_has_next = 1'b0;
                walk_tlx_sel = walk_tlx_level[1:0];
                walk_tlx_next_sel = (walk_tlx_level == 3'd3) ? 2'd3 : (walk_tlx_level[1:0] + 2'd1);

                if (walk_fc_level) begin
                    walk_index = {29'h0, fc_in};
                    walk_fc_level = 1'b0;
                    walk_next_width = mmu_tlx_width(MMU_TC, walk_tlx_sel);
                    walk_has_next = (walk_next_width != 4'h0);
                end else begin
                    walk_width = mmu_tlx_width(MMU_TC, walk_tlx_sel);
                    if (walk_width == 4'h0) begin
                        walk_fault = 1'b1;
                        walk_invalid_fault = 1'b1;
                    end else begin
                        walk_index = mmu_index_extract(logical_addr, MMU_TC[19:16], walk_consumed, walk_width);
                        walk_consumed = walk_consumed + {2'b00, walk_width};
                        walk_tlx_level = walk_tlx_level + 3'd1;

                        if (walk_tlx_level < 3'd4) begin
                            walk_tlx_next_sel = walk_tlx_level[1:0];
                            walk_next_width = mmu_tlx_width(MMU_TC, walk_tlx_next_sel);
                            walk_has_next = (walk_next_width != 4'h0);
                        end

                        if (walk_levels == 3'd0 && !MMU_TC[24] &&
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
                        walk_bus_fault = 1'b1;
                    end else begin
                        walk_desc = walk_lookup[31:0];
                        if (walk_desc_size == 3'd8) begin
                            walk_lookup_hi = mmu_desc_shadow_read(walk_desc_addr + 32'd4);
                            if (!walk_lookup_hi[32]) begin
                                walk_fault = 1'b1;
                                walk_bus_fault = 1'b1;
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
                            walk_invalid_fault = 1'b1;
                        end
                        2'b01: begin
                            if (walk_desc_size == 3'd8 && walk_has_next && walk_next_width != 4'h0) begin
                                walk_limit_lower = walk_desc[31];
                                walk_limit = walk_desc[30:16];
                                walk_limit_index = mmu_index_extract(
                                    logical_addr,
                                    MMU_TC[19:16],
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
                                walk_page_m = walk_desc[4];
                                walk_done = 1'b1;
                            end
                        end
                        default: begin
                            if (walk_has_next && (walk_levels < walk_search_limit)) begin
                                if (walk_desc_size == 3'd8 && walk_next_width != 4'h0) begin
                                    walk_limit_lower = walk_desc[31];
                                    walk_limit = walk_desc[30:16];
                                    walk_limit_index = mmu_index_extract(
                                        logical_addr,
                                        MMU_TC[19:16],
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
                            end else if (!walk_has_next) begin
                                // Bottom-level table descriptor encodes indirection.
                                walk_desc_ptr = (walk_desc_size == 3'd8) ? walk_desc_hi : walk_desc;
                                if (walk_desc_dt == 2'b10) begin
                                    walk_lookup = mmu_desc_shadow_read({walk_desc_ptr[31:2], 2'b00});
                                    if (!walk_lookup[32]) begin
                                        walk_fault = 1'b1;
                                        walk_bus_fault = 1'b1;
                                    end else if (walk_lookup[1:0] != 2'b01) begin
                                        walk_fault = 1'b1;
                                        walk_invalid_fault = 1'b1;
                                    end else begin
                                        walk_page_desc = walk_lookup[31:0];
                                        walk_wp_accum = walk_wp_accum || walk_lookup[2];
                                        walk_page_m = walk_lookup[4];
                                        walk_done = 1'b1;
                                    end
                                end else begin
                                    walk_lookup = mmu_desc_shadow_read({walk_desc_ptr[31:3], 3'b000});
                                    walk_lookup_hi = mmu_desc_shadow_read({walk_desc_ptr[31:3], 3'b000} + 32'd4);
                                    if (!walk_lookup[32] || !walk_lookup_hi[32]) begin
                                        walk_fault = 1'b1;
                                        walk_bus_fault = 1'b1;
                                    end else if (walk_lookup[1:0] != 2'b01) begin
                                        walk_fault = 1'b1;
                                        walk_invalid_fault = 1'b1;
                                    end else begin
                                        walk_page_desc = walk_lookup[31:0];
                                        walk_wp_accum = walk_wp_accum || walk_lookup[2];
                                        if (!fc_in[2] && walk_lookup[8]) begin
                                            walk_fault = 1'b1;
                                            walk_super_fault = 1'b1;
                                        end
                                        if (!walk_fault) begin
                                            walk_page_m = walk_lookup[4];
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

        if (!walk_fault && !walk_done && (walk_search_limit == 3'd7)) begin
            walk_fault = 1'b1;
            walk_invalid_fault = 1'b1;
        end

        if (walk_limit_fault)
            mmusr = mmusr | MMUSR_L;
        if (walk_super_fault)
            mmusr = mmusr | MMUSR_S;
        if (walk_bus_fault)
            mmusr = mmusr | MMUSR_B;
        if (walk_wp_accum)
            mmusr = mmusr | MMUSR_W;
        if (walk_done && walk_page_m)
            mmusr = mmusr | MMUSR_M;
        if (walk_invalid_fault || walk_bus_fault || walk_limit_fault)
            mmusr = mmusr | MMUSR_I;
        if (mmusr & MMUSR_L)
            mmusr = mmusr & ~(MMUSR_W | MMUSR_M);
        mmusr = (mmusr & ~MMUSR_N) | ({13'h0, walk_levels} & MMUSR_N);
    end else begin
        mmusr = MMUSR_I;
        mmusr = (mmusr & ~MMUSR_N) | ({13'h0, 3'd0} & MMUSR_N);
    end

    mmu_ptest_walk_mmusr = mmusr;
end
endfunction
