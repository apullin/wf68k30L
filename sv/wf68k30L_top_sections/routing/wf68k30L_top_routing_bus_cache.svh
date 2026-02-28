// ========================================================================
// Bus request arbitration
// ========================================================================

assign DATA_RD = DATA_RD_EXH || DATA_RD_MAIN;
assign DATA_WR = DATA_WR_EXH || DATA_WR_MAIN;

always_comb begin : burst_prefetch_select
    integer      scan_idx;
    logic        icache_found;
    logic [2:0]  icache_scan_word;
    logic        dcache_found;
    logic [1:0]  dcache_scan_entry;
    BURST_PREFETCH_OP_REQ = 1'b0;
    BURST_PREFETCH_DATA_REQ = 1'b0;
    BURST_PREFETCH_OP_WORD = 3'b000;
    BURST_PREFETCH_DATA_ENTRY = 2'b00;
    BURST_PREFETCH_ADDR = 32'h0000_0000;
    BURST_PREFETCH_FC = FC_USER_PROG;
    icache_found = 1'b0;
    icache_scan_word = ICACHE_BURST_FILL_NEXT_WORD;
    dcache_found = 1'b0;
    dcache_scan_entry = DCACHE_BURST_FILL_NEXT_ENTRY;

    if (!BUS_BSY && !DATA_WR && !DATA_RD_BUS && !OPCODE_REQ_CORE_MISS && !BUSY_EXH) begin
        if (ICACHE_BURST_FILL_VALID && ICACHE_BURST_FILL_PENDING != 8'h00) begin
            for (scan_idx = 0; scan_idx < 8; scan_idx = scan_idx + 1) begin
                if (!icache_found) begin
                    icache_scan_word = ICACHE_BURST_FILL_NEXT_WORD + scan_idx[2:0];
                    if (ICACHE_BURST_FILL_PENDING[icache_scan_word]) begin
                        BURST_PREFETCH_OP_WORD = icache_scan_word;
                        icache_found = 1'b1;
                    end
                end
            end
            BURST_PREFETCH_OP_REQ = 1'b1;
            BURST_PREFETCH_ADDR = {
                ICACHE_BURST_FILL_TAG,
                ICACHE_BURST_FILL_LINE,
                BURST_PREFETCH_OP_WORD,
                1'b0
            };
            BURST_PREFETCH_FC = ICACHE_BURST_FILL_FC;
        end else if (DCACHE_BURST_FILL_VALID && DCACHE_BURST_FILL_PENDING != 4'h0) begin
            for (scan_idx = 0; scan_idx < 4; scan_idx = scan_idx + 1) begin
                if (!dcache_found) begin
                    dcache_scan_entry = DCACHE_BURST_FILL_NEXT_ENTRY + scan_idx[1:0];
                    if (DCACHE_BURST_FILL_PENDING[dcache_scan_entry]) begin
                        BURST_PREFETCH_DATA_ENTRY = dcache_scan_entry;
                        dcache_found = 1'b1;
                    end
                end
            end
            BURST_PREFETCH_DATA_REQ = 1'b1;
            BURST_PREFETCH_ADDR = {
                DCACHE_BURST_FILL_TAG,
                DCACHE_BURST_FILL_LINE,
                BURST_PREFETCH_DATA_ENTRY,
                2'b00
            };
            BURST_PREFETCH_FC = DCACHE_BURST_FILL_FC;
        end
    end
end

// Request/fault latches that decouple core-side combinational logic from bus FSM timing.
always_ff @(posedge CLK) begin : bus_req_latch
    // Flip-flops break combinatorial loops between core requests and bus controller.
    // Requests are valid until the bus controller enters START_CYCLE and asserts BUS_BSY.
    if (RESET_CPU) begin
        RD_REQ_I <= 1'b0;
        WR_REQ_I <= 1'b0;
        OPCODE_REQ_I <= 1'b0;
        MMU_FAULT_DATA_ACK <= 1'b0;
        MMU_FAULT_OPCODE_ACK <= 1'b0;
        BUS_CYCLE_BURST <= 1'b0;
        BUS_CYCLE_BURST_IS_OP <= 1'b0;
    end else if (!BUS_BSY) begin
        MMU_FAULT_DATA_ACK <= MMU_RUNTIME_FAULT && (DATA_RD_BUS || DATA_WR);
        MMU_FAULT_OPCODE_ACK <= MMU_RUNTIME_FAULT && OPCODE_REQ_CORE_MISS && !DATA_RD_BUS && !DATA_WR;
        RD_REQ_I <= (DATA_RD_BUS && !MMU_RUNTIME_FAULT && !MMU_RUNTIME_STALL) || BURST_PREFETCH_DATA_REQ;
        WR_REQ_I <= DATA_WR && !MMU_RUNTIME_FAULT && !MMU_RUNTIME_STALL;
        OPCODE_REQ_I <= (OPCODE_REQ_CORE_MISS && !MMU_RUNTIME_FAULT && !MMU_RUNTIME_STALL) || BURST_PREFETCH_OP_REQ;
        if (BURST_PREFETCH_OP_REQ) begin
            BUS_CYCLE_BURST <= 1'b1;
            BUS_CYCLE_BURST_IS_OP <= 1'b1;
        end else if (BURST_PREFETCH_DATA_REQ) begin
            BUS_CYCLE_BURST <= 1'b1;
            BUS_CYCLE_BURST_IS_OP <= 1'b0;
        end else begin
            BUS_CYCLE_BURST <= 1'b0;
            BUS_CYCLE_BURST_IS_OP <= 1'b0;
        end
    end else if (BUS_BSY) begin
        RD_REQ_I <= 1'b0;
        WR_REQ_I <= 1'b0;
        OPCODE_REQ_I <= 1'b0;
        MMU_FAULT_DATA_ACK <= 1'b0;
        MMU_FAULT_OPCODE_ACK <= 1'b0;
    end
end

// Deterministic one-cycle staging for MMU table-search results.
always_ff @(posedge CLK) begin : mmu_walk_delay_state
    if (RESET_CPU) begin
        MMU_WALK_DELAY_ARMED <= 1'b0;
    end else if (BUS_BSY) begin
        MMU_WALK_DELAY_ARMED <= 1'b0;
    end else if (MMU_RUNTIME_REQ) begin
        MMU_WALK_DELAY_ARMED <= MMU_RUNTIME_STALL;
    end else begin
        MMU_WALK_DELAY_ARMED <= 1'b0;
    end
end

// Descriptor-shadow update model used by MMU table-walk lookups.
always_ff @(posedge CLK) begin : mmu_desc_shadow_update
    logic [31:0] shadow_addr;
    logic [31:0] shadow_data;
    logic        hit;
    logic        free;
    logic [$clog2(MMU_DESC_SHADOW_LINES)-1:0] hit_idx;
    logic [$clog2(MMU_DESC_SHADOW_LINES)-1:0] free_idx;
    logic [$clog2(MMU_DESC_SHADOW_LINES)-1:0] ins_idx;
    integer i;
    if (RESET_CPU) begin
        MMU_DESC_SHADOW_V <= '0;
        MMU_DESC_SHADOW_REPL_PTR <= '0;
        MMU_DESC_SHADOW_PENDING <= 1'b0;
        MMU_DESC_SHADOW_PENDING_ADDR <= 32'h0;
        MMU_DESC_SHADOW_PENDING_WR <= 1'b0;
        for (i = 0; i < MMU_DESC_SHADOW_LINES; i = i + 1) begin
            MMU_DESC_SHADOW_ADDR[i] <= 32'h0;
            MMU_DESC_SHADOW_DATA[i] <= 32'h0;
        end
    end else begin
        if (!BUS_BSY && (DATA_RD_BUS || DATA_WR) && !MMU_RUNTIME_FAULT && !MMU_RUNTIME_STALL) begin
            MMU_DESC_SHADOW_PENDING <= 1'b1;
            MMU_DESC_SHADOW_PENDING_ADDR <= ADR_P_PHYS;
            MMU_DESC_SHADOW_PENDING_WR <= DATA_WR;
        end

        if (DATA_RDY_BUSIF_CORE && MMU_DESC_SHADOW_PENDING) begin
            // Capture data accesses at completion using the request-latched address.
            shadow_addr = {MMU_DESC_SHADOW_PENDING_ADDR[31:2], 2'b00};
            shadow_data = MMU_DESC_SHADOW_PENDING_WR ? DATA_OUT : DATA_TO_CORE_BUSIF;

            hit = 1'b0;
            free = 1'b0;
            hit_idx = '0;
            free_idx = '0;
            for (i = 0; i < MMU_DESC_SHADOW_LINES; i = i + 1) begin
                if (!hit && MMU_DESC_SHADOW_V[i] && MMU_DESC_SHADOW_ADDR[i] == shadow_addr) begin
                    hit = 1'b1;
                    hit_idx = i[$clog2(MMU_DESC_SHADOW_LINES)-1:0];
                end
                if (!free && !MMU_DESC_SHADOW_V[i]) begin
                    free = 1'b1;
                    free_idx = i[$clog2(MMU_DESC_SHADOW_LINES)-1:0];
                end
            end

            ins_idx = hit ? hit_idx : (free ? free_idx : MMU_DESC_SHADOW_REPL_PTR);
            MMU_DESC_SHADOW_V[ins_idx] <= 1'b1;
            MMU_DESC_SHADOW_ADDR[ins_idx] <= shadow_addr;
            MMU_DESC_SHADOW_DATA[ins_idx] <= shadow_data;
            if (!hit && !free)
                MMU_DESC_SHADOW_REPL_PTR <= MMU_DESC_SHADOW_REPL_PTR + 1'b1;
            MMU_DESC_SHADOW_PENDING <= 1'b0;
        end else if (!BUS_BSY && MMU_RUNTIME_FAULT) begin
            MMU_DESC_SHADOW_PENDING <= 1'b0;
        end
    end
end

// Core-visible bus requests: direct when idle, held via latches while BUS_BSY.
assign RD_REQ = !BUS_BSY ? ((DATA_RD_BUS && !MMU_RUNTIME_FAULT && !MMU_RUNTIME_STALL) || BURST_PREFETCH_DATA_REQ) : RD_REQ_I;
assign WR_REQ = !BUS_BSY ? (DATA_WR && !MMU_RUNTIME_FAULT && !MMU_RUNTIME_STALL) : WR_REQ_I;
assign OPCODE_REQ_CORE = !BUS_BSY ? OPCODE_RD : OPCODE_REQ_I;

// Minimal instruction-cache lookup on opcode requests.
always_comb begin : icache_lookup
    logic [3:0]  req_line;
    logic [2:0]  req_word;
    logic [23:0] req_tag;
    logic        req_cacheable;
    ICACHE_HIT_NOW = 1'b0;
    if (!BUS_BSY && OPCODE_REQ_CORE && CACR[0]) begin // EI=1
        req_cacheable = !mmu_cache_inhibit(FC_I, ADR_P_PHYS, 1'b1, 1'b0, 1'b0, MMU_TT0, MMU_TT1);
        req_line = ADR_P_PHYS[7:4];
        req_word = ADR_P_PHYS[3:1];
        req_tag = ADR_P_PHYS[31:8];
        if (req_cacheable && ICACHE_TAG[req_line] == req_tag && ICACHE_VALID[req_line][req_word])
            ICACHE_HIT_NOW = 1'b1;
    end
end

assign OPCODE_REQ_CORE_MISS = OPCODE_REQ_CORE && !ICACHE_HIT_NOW;

// On an instruction-cache hit, satisfy the opcode fetch internally.
assign OPCODE_REQ = (OPCODE_REQ_CORE_MISS && !MMU_RUNTIME_FAULT && !MMU_RUNTIME_STALL) || BURST_PREFETCH_OP_REQ;

// Minimal data-cache lookup on data read requests.
always_comb begin : dcache_lookup
    logic [3:0]  req_line;
    logic [1:0]  req_entry;
    logic [23:0] req_tag;
    logic        req_cacheable;
    DCACHE_HIT_NOW = 1'b0;
    DCACHE_HIT_DATA_NOW = 32'h0000_0000;
    if (!BUS_BSY && DATA_RD && CACR[8] && !RMC) begin // ED=1 and not RMW read portion.
        req_cacheable = !mmu_cache_inhibit(FC_I, ADR_P_PHYS, 1'b1, 1'b0, 1'b0, MMU_TT0, MMU_TT1);
        req_line = ADR_P_PHYS[7:4];
        req_entry = ADR_P_PHYS[3:2];
        req_tag = ADR_P_PHYS[31:8];
        if (req_cacheable &&
            dcache_access_supported(OP_SIZE_BUS, ADR_P_PHYS[1:0]) &&
            DCACHE_TAG[req_line] == req_tag &&
            DCACHE_VALID[req_line][req_entry]) begin
            DCACHE_HIT_NOW = 1'b1;
            DCACHE_HIT_DATA_NOW = dcache_read_extract(
                DCACHE_DATA[req_line][req_entry],
                OP_SIZE_BUS,
                ADR_P_PHYS[1:0]
            );
        end
    end
end

assign DATA_RD_BUS = DATA_RD && !DCACHE_HIT_NOW;
