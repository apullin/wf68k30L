// ========================================================================
// MMU/cache constants and sequential state
// ========================================================================

localparam logic [15:0] MMUSR_B = 16'h8000;
localparam logic [15:0] MMUSR_L = 16'h4000;
localparam logic [15:0] MMUSR_S = 16'h2000;
localparam logic [15:0] MMUSR_W = 16'h0800;
localparam logic [15:0] MMUSR_I = 16'h0400;
localparam logic [15:0] MMUSR_M = 16'h0200;
localparam logic [15:0] MMUSR_T = 16'h0040;
localparam logic [15:0] MMUSR_N = 16'h0007;
localparam logic [31:0] CACR_RW_MASK = 32'h0000_3313; // WA,DBE,FD,ED,IBE,FI,EI
localparam int ICACHE_LINES = 16;
localparam int DCACHE_LINES = 16;

// Cache arrays, fill tracking, and MOVEC(CACR/CAAR) semantics.
always_ff @(posedge CLK) begin : cache_registers
    integer i;
    integer j;
    logic [31:0] cacr_write_value;
    logic [3:0]  caar_line;
    logic [1:0]  caar_entry;
    logic [3:0]  fill_line;
    logic [2:0]  fill_word;
    logic [23:0] fill_tag;
    logic [3:0]  dcache_line;
    logic [1:0]  dcache_entry;
    logic [23:0] dcache_tag;
    logic [7:0]  icache_valid_after;
    logic [7:0]  icache_burst_pending_after;
    logic [3:0]  dcache_valid_after;
    logic [3:0]  dcache_burst_pending_after;
    logic [31:0] dcache_merged_word;
    logic        dcache_hit;
    if (RESET_CPU) begin
        CACR <= 32'h0;
        CAAR <= 32'h0;
        ICACHE_RDY <= 1'b0;
        ICACHE_OPCODE_WORD <= 16'h0000;
        ICACHE_FILL_PENDING <= 1'b0;
        ICACHE_FILL_ADDR <= 32'h0;
        ICACHE_FILL_CACHEABLE <= 1'b0;
        ICACHE_FILL_FC <= 3'b000;
        ICACHE_BURST_TRACK_VALID <= 1'b0;
        ICACHE_BURST_TRACK_LINE <= 4'h0;
        ICACHE_BURST_TRACK_TAG <= 24'h0;
        ICACHE_BURST_FILL_VALID <= 1'b0;
        ICACHE_BURST_FILL_LINE <= 4'h0;
        ICACHE_BURST_FILL_TAG <= 24'h0;
        ICACHE_BURST_FILL_PENDING <= 8'h00;
        ICACHE_BURST_FILL_FC <= 3'b000;
        ICACHE_BURST_FILL_NEXT_WORD <= 3'd0;
        DATA_RDY_CACHE <= 1'b0;
        DATA_VALID_CACHE <= 1'b0;
        DATA_TO_CORE_CACHE <= 32'h0;
        DATA_LAST_FROM_CACHE <= 1'b0;
        DCACHE_HIT_PENDING <= 1'b0;
        DCACHE_HIT_DATA_PENDING <= 32'h0;
        DCACHE_READ_FILL_PENDING <= 1'b0;
        DCACHE_READ_FILL_ADDR <= 32'h0;
        DCACHE_READ_FILL_SIZE <= LONG;
        DCACHE_READ_FILL_CACHEABLE <= 1'b0;
        DCACHE_READ_FILL_FC <= 3'b000;
        DCACHE_BURST_TRACK_VALID <= 1'b0;
        DCACHE_BURST_TRACK_LINE <= 4'h0;
        DCACHE_BURST_TRACK_TAG <= 24'h0;
        DCACHE_BURST_FILL_VALID <= 1'b0;
        DCACHE_BURST_FILL_LINE <= 4'h0;
        DCACHE_BURST_FILL_TAG <= 24'h0;
        DCACHE_BURST_FILL_PENDING <= 4'h0;
        DCACHE_BURST_FILL_FC <= 3'b000;
        DCACHE_BURST_FILL_NEXT_ENTRY <= 2'd0;
        DCACHE_WRITE_PENDING <= 1'b0;
        DCACHE_WRITE_ADDR <= 32'h0;
        DCACHE_WRITE_SIZE <= LONG;
        DCACHE_WRITE_DATA <= 32'h0;
        DCACHE_WRITE_CACHEABLE <= 1'b0;
        for (i = 0; i < ICACHE_LINES; i = i + 1) begin
            ICACHE_TAG[i] <= 24'h0;
            ICACHE_VALID[i] <= 8'h00;
            for (j = 0; j < 8; j = j + 1)
                ICACHE_DATA[i][j] <= 16'h0000;
        end
        for (i = 0; i < DCACHE_LINES; i = i + 1) begin
            DCACHE_TAG[i] <= 24'h0;
            DCACHE_VALID[i] <= 4'h0;
            for (j = 0; j < 4; j = j + 1)
                DCACHE_DATA[i][j] <= 32'h0000_0000;
        end
    end else begin
        ICACHE_RDY <= 1'b0;
        DATA_RDY_CACHE <= 1'b0;
        DATA_VALID_CACHE <= 1'b0;

        // Hold the last data-return source between ready strobes.
        if (DATA_RDY_CACHE)
            DATA_LAST_FROM_CACHE <= 1'b1;
        else if (DATA_RDY_BUSIF_CORE)
            DATA_LAST_FROM_CACHE <= 1'b0;

        // Serve opcode requests directly from the instruction-cache model.
        if (!BUS_BSY && OPCODE_REQ_CORE && ICACHE_HIT_NOW) begin
            ICACHE_OPCODE_WORD <= ICACHE_DATA[ADR_P_PHYS[7:4]][ADR_P_PHYS[3:1]];
            ICACHE_RDY <= 1'b1;
        end

        // Serve data-cache hits with one-cycle latency to match core handshake timing.
        if (DCACHE_HIT_PENDING) begin
            DATA_TO_CORE_CACHE <= DCACHE_HIT_DATA_PENDING;
            DATA_RDY_CACHE <= 1'b1;
            DATA_VALID_CACHE <= 1'b1;
            DCACHE_HIT_PENDING <= 1'b0;
        end else if (!BUS_BSY && DATA_RD && DCACHE_HIT_NOW && !DATA_RDY_BUSIF_CORE) begin
            DCACHE_HIT_DATA_PENDING <= DCACHE_HIT_DATA_NOW;
            DCACHE_HIT_PENDING <= 1'b1;
        end

        // Track in-flight opcode misses to fill cache on bus completion.
        if (!BUS_BSY && OPCODE_REQ) begin
            ICACHE_FILL_PENDING <= 1'b1;
            ICACHE_FILL_ADDR <= ADR_BUS_REQ_PHYS;
            ICACHE_FILL_CACHEABLE <= BURST_PREFETCH_OP_REQ ? 1'b1 :
                                     !mmu_cache_inhibit(FC_I, ADR_P_PHYS, 1'b1, 1'b0, 1'b0, MMU_TT0, MMU_TT1);
            ICACHE_FILL_FC <= FC_BUS_REQ;
        end else if (OPCODE_RDY_BUSIF) begin
            ICACHE_FILL_PENDING <= 1'b0;
        end

        // Track in-flight data reads for cache fill on completion.
        if (!BUS_BSY && ((DATA_RD_BUS && !MMU_RUNTIME_FAULT && !MMU_RUNTIME_STALL) || BURST_PREFETCH_DATA_REQ)) begin
            DCACHE_READ_FILL_PENDING <= 1'b1;
            DCACHE_READ_FILL_ADDR <= ADR_BUS_REQ_PHYS;
            DCACHE_READ_FILL_SIZE <= BURST_PREFETCH_DATA_REQ ? LONG : OP_SIZE_BUS;
            DCACHE_READ_FILL_CACHEABLE <= BURST_PREFETCH_DATA_REQ ? 1'b1 :
                                          !mmu_cache_inhibit(FC_I, ADR_P_PHYS, 1'b1, 1'b0, RMC, MMU_TT0, MMU_TT1);
            DCACHE_READ_FILL_FC <= FC_BUS_REQ;
        end else if (DATA_RDY_BUSIF) begin
            DCACHE_READ_FILL_PENDING <= 1'b0;
        end

        // Track in-flight data writes for write-through cache maintenance.
        if (!BUS_BSY && DATA_WR && !MMU_RUNTIME_FAULT && !MMU_RUNTIME_STALL) begin
            DCACHE_WRITE_PENDING <= 1'b1;
            DCACHE_WRITE_ADDR <= ADR_P_PHYS;
            DCACHE_WRITE_SIZE <= OP_SIZE_BUS;
            DCACHE_WRITE_DATA <= DATA_FROM_CORE;
            DCACHE_WRITE_CACHEABLE <= !mmu_cache_inhibit(FC_I, ADR_P_PHYS, 1'b0, 1'b1, RMC, MMU_TT0, MMU_TT1);
        end else if (DATA_RDY_BUSIF_CORE && DCACHE_WRITE_PENDING) begin
            DCACHE_WRITE_PENDING <= 1'b0;
        end

        // Fill one word on each opcode bus response when enabled and unfrozen.
        if (OPCODE_RDY_BUSIF && OPCODE_VALID_BUSIF && BERRn &&
            ICACHE_FILL_PENDING && ICACHE_FILL_CACHEABLE && CACR[0] && !CACR[1]) begin
            fill_line = ICACHE_FILL_ADDR[7:4];
            fill_word = ICACHE_FILL_ADDR[3:1];
            fill_tag = ICACHE_FILL_ADDR[31:8];
            if (ICACHE_TAG[fill_line] != fill_tag)
                ICACHE_VALID[fill_line] <= 8'h00;
            ICACHE_TAG[fill_line] <= fill_tag;
            ICACHE_DATA[fill_line][fill_word] <= OPCODE_TO_CORE_BUSIF;
            ICACHE_VALID[fill_line][fill_word] <= 1'b1;

            icache_valid_after = (ICACHE_TAG[fill_line] == fill_tag) ?
                                 (ICACHE_VALID[fill_line] | (8'h01 << fill_word)) :
                                 (8'h01 << fill_word);
            icache_burst_pending_after = ICACHE_BURST_FILL_PENDING & ~(8'h01 << fill_word);

            // Phase-5 line-aware burst tracking:
            // A burst-acknowledged fill marks this line as the active burst context.
            // Subsequent same-line misses suppress redundant CBREQ assertions.
            if (CACR[4] && !CBACKn) begin
                ICACHE_BURST_TRACK_VALID <= 1'b1;
                ICACHE_BURST_TRACK_LINE <= fill_line;
                ICACHE_BURST_TRACK_TAG <= fill_tag;
            end else if (ICACHE_BURST_TRACK_VALID &&
                         ICACHE_BURST_TRACK_LINE == fill_line &&
                         ICACHE_BURST_TRACK_TAG == fill_tag &&
                         icache_valid_after == 8'hFF) begin
                ICACHE_BURST_TRACK_VALID <= 1'b0; // Entire line now resident.
            end else if (ICACHE_BURST_TRACK_VALID &&
                         (ICACHE_BURST_TRACK_LINE != fill_line || ICACHE_BURST_TRACK_TAG != fill_tag)) begin
                ICACHE_BURST_TRACK_VALID <= 1'b0;
            end

            // Phase-8 autonomous burst completion:
            // once the first burst-acknowledged miss returns, background reads
            // complete the rest of the line without exposing extra ready strobes
            // to the core.
            if (ICACHE_BURST_FILL_VALID &&
                ICACHE_BURST_FILL_LINE == fill_line &&
                ICACHE_BURST_FILL_TAG == fill_tag) begin
                ICACHE_BURST_FILL_PENDING <= icache_burst_pending_after;
                ICACHE_BURST_FILL_NEXT_WORD <= fill_word + 3'd1;
                if (icache_burst_pending_after == 8'h00)
                    ICACHE_BURST_FILL_VALID <= 1'b0;
            end

            if (!BUS_CYCLE_BURST && CACR[4] && !CBACKn) begin
                ICACHE_BURST_FILL_PENDING <= ~icache_valid_after;
                ICACHE_BURST_FILL_LINE <= fill_line;
                ICACHE_BURST_FILL_TAG <= fill_tag;
                ICACHE_BURST_FILL_FC <= ICACHE_FILL_FC;
                ICACHE_BURST_FILL_NEXT_WORD <= fill_word + 3'd1;
                ICACHE_BURST_FILL_VALID <= (~icache_valid_after != 8'h00);
            end
        end

        // Fill data cache on eligible read misses (long-word aligned only).
        if (DATA_RDY_BUSIF && DATA_VALID_BUSIF && BERRn &&
            DCACHE_READ_FILL_PENDING && DCACHE_READ_FILL_CACHEABLE &&
            CACR[8] && !CACR[9] && DCACHE_READ_FILL_SIZE == LONG && DCACHE_READ_FILL_ADDR[1:0] == 2'b00) begin
            dcache_line = DCACHE_READ_FILL_ADDR[7:4];
            dcache_entry = DCACHE_READ_FILL_ADDR[3:2];
            dcache_tag = DCACHE_READ_FILL_ADDR[31:8];
            if (DCACHE_TAG[dcache_line] != dcache_tag)
                DCACHE_VALID[dcache_line] <= 4'h0;
            DCACHE_TAG[dcache_line] <= dcache_tag;
            DCACHE_DATA[dcache_line][dcache_entry] <= DATA_TO_CORE_BUSIF;
            DCACHE_VALID[dcache_line][dcache_entry] <= 1'b1;

            dcache_valid_after = (DCACHE_TAG[dcache_line] == dcache_tag) ?
                                 (DCACHE_VALID[dcache_line] | (4'h1 << dcache_entry)) :
                                 (4'h1 << dcache_entry);
            dcache_burst_pending_after = DCACHE_BURST_FILL_PENDING & ~(4'h1 << dcache_entry);

            if (CACR[12] && !CBACKn) begin
                DCACHE_BURST_TRACK_VALID <= 1'b1;
                DCACHE_BURST_TRACK_LINE <= dcache_line;
                DCACHE_BURST_TRACK_TAG <= dcache_tag;
            end else if (DCACHE_BURST_TRACK_VALID &&
                         DCACHE_BURST_TRACK_LINE == dcache_line &&
                         DCACHE_BURST_TRACK_TAG == dcache_tag &&
                         dcache_valid_after == 4'hF) begin
                DCACHE_BURST_TRACK_VALID <= 1'b0;
            end else if (DCACHE_BURST_TRACK_VALID &&
                         (DCACHE_BURST_TRACK_LINE != dcache_line || DCACHE_BURST_TRACK_TAG != dcache_tag)) begin
                DCACHE_BURST_TRACK_VALID <= 1'b0;
            end

            if (DCACHE_BURST_FILL_VALID &&
                DCACHE_BURST_FILL_LINE == dcache_line &&
                DCACHE_BURST_FILL_TAG == dcache_tag) begin
                DCACHE_BURST_FILL_PENDING <= dcache_burst_pending_after;
                DCACHE_BURST_FILL_NEXT_ENTRY <= dcache_entry + 2'd1;
                if (dcache_burst_pending_after == 4'h0)
                    DCACHE_BURST_FILL_VALID <= 1'b0;
            end

            if (!BUS_CYCLE_BURST && CACR[12] && !CBACKn) begin
                DCACHE_BURST_FILL_PENDING <= ~dcache_valid_after;
                DCACHE_BURST_FILL_LINE <= dcache_line;
                DCACHE_BURST_FILL_TAG <= dcache_tag;
                DCACHE_BURST_FILL_FC <= DCACHE_READ_FILL_FC;
                DCACHE_BURST_FILL_NEXT_ENTRY <= dcache_entry + 2'd1;
                DCACHE_BURST_FILL_VALID <= (~dcache_valid_after != 4'h0);
            end
        end

        // Write-through updates: hit always updates; WA controls miss allocation.
        if (DATA_RDY_BUSIF && DATA_VALID_BUSIF && BERRn &&
            DCACHE_WRITE_PENDING && DCACHE_WRITE_CACHEABLE &&
            CACR[8] && dcache_access_supported(DCACHE_WRITE_SIZE, DCACHE_WRITE_ADDR[1:0])) begin
            dcache_line = DCACHE_WRITE_ADDR[7:4];
            dcache_entry = DCACHE_WRITE_ADDR[3:2];
            dcache_tag = DCACHE_WRITE_ADDR[31:8];
            dcache_hit = (DCACHE_TAG[dcache_line] == dcache_tag) && DCACHE_VALID[dcache_line][dcache_entry];
            dcache_merged_word = dcache_write_merge(
                DCACHE_DATA[dcache_line][dcache_entry],
                DCACHE_WRITE_DATA,
                DCACHE_WRITE_SIZE,
                DCACHE_WRITE_ADDR[1:0]
            );
            if (dcache_hit) begin
                DCACHE_DATA[dcache_line][dcache_entry] <= dcache_merged_word;
                DCACHE_VALID[dcache_line][dcache_entry] <= 1'b1;
            end else if (CACR[13] && !CACR[9] && DCACHE_WRITE_SIZE == LONG && DCACHE_WRITE_ADDR[1:0] == 2'b00) begin
                if (DCACHE_TAG[dcache_line] != dcache_tag)
                    DCACHE_VALID[dcache_line] <= 4'h0;
                DCACHE_TAG[dcache_line] <= dcache_tag;
                DCACHE_DATA[dcache_line][dcache_entry] <= DCACHE_WRITE_DATA;
                DCACHE_VALID[dcache_line][dcache_entry] <= 1'b1;
            end
        end

        if (CACR_WR) begin
            cacr_write_value = ALU_RESULT[31:0];
            caar_line = CAAR[7:4];
            caar_entry = CAAR[3:2];
            CACR <= cacr_write_value & CACR_RW_MASK;

            // Clear-data operations update only the internal validity model.
            if (cacr_write_value[11]) begin // CD
                for (i = 0; i < DCACHE_LINES; i = i + 1)
                    DCACHE_VALID[i] <= 4'h0;
                DCACHE_BURST_TRACK_VALID <= 1'b0;
                DCACHE_BURST_FILL_VALID <= 1'b0;
                DCACHE_BURST_FILL_PENDING <= 4'h0;
                DCACHE_BURST_FILL_NEXT_ENTRY <= 2'd0;
            end
            if (cacr_write_value[10]) begin // CED
                DCACHE_VALID[caar_line][caar_entry] <= 1'b0;
                DCACHE_BURST_TRACK_VALID <= 1'b0;
                DCACHE_BURST_FILL_VALID <= 1'b0;
                DCACHE_BURST_FILL_PENDING <= 4'h0;
                DCACHE_BURST_FILL_NEXT_ENTRY <= 2'd0;
            end

            // Clear-instruction operations invalidate cache entries immediately.
            if (cacr_write_value[3]) begin // CI
                for (i = 0; i < ICACHE_LINES; i = i + 1)
                    ICACHE_VALID[i] <= 8'h00;
                ICACHE_BURST_TRACK_VALID <= 1'b0;
                ICACHE_BURST_FILL_VALID <= 1'b0;
                ICACHE_BURST_FILL_PENDING <= 8'h00;
                ICACHE_BURST_FILL_NEXT_WORD <= 3'd0;
            end
            if (cacr_write_value[2]) begin // CEI
                ICACHE_VALID[caar_line][{caar_entry, 1'b0}] <= 1'b0;
                ICACHE_VALID[caar_line][{caar_entry, 1'b1}] <= 1'b0;
                ICACHE_BURST_TRACK_VALID <= 1'b0;
                ICACHE_BURST_FILL_VALID <= 1'b0;
                ICACHE_BURST_FILL_PENDING <= 8'h00;
                ICACHE_BURST_FILL_NEXT_WORD <= 3'd0;
            end

            // Disabling burst capability clears line-tracking context.
            if (!cacr_write_value[0] || !cacr_write_value[4]) begin
                ICACHE_BURST_TRACK_VALID <= 1'b0;
                ICACHE_BURST_FILL_VALID <= 1'b0;
                ICACHE_BURST_FILL_PENDING <= 8'h00;
                ICACHE_BURST_FILL_NEXT_WORD <= 3'd0;
            end
            if (!cacr_write_value[8] || !cacr_write_value[12]) begin
                DCACHE_BURST_TRACK_VALID <= 1'b0;
                DCACHE_BURST_FILL_VALID <= 1'b0;
                DCACHE_BURST_FILL_PENDING <= 4'h0;
                DCACHE_BURST_FILL_NEXT_ENTRY <= 2'd0;
            end
        end

        if (CAAR_WR)
            CAAR <= ALU_RESULT[31:0];
end
end

// Coprocessor-interface model (HW-003 phase 4/5):
// tracks initiation/no-response and provides model-scope pre/mid/post exception
// injection controls for stack-frame fidelity testing.
localparam logic [2:0] CPIF_ST_IDLE    = 3'd0;
localparam logic [2:0] CPIF_ST_INIT    = 3'd1;
localparam logic [2:0] CPIF_ST_STEP    = 3'd2;
localparam logic [2:0] CPIF_ST_RESP    = 3'd3;
localparam logic [2:0] CPIF_ST_TIMEOUT = 3'd4;
localparam logic [2:0] CPIF_ST_DONE    = 3'd5;

always_ff @(posedge CLK) begin : coprocessor_interface_model
    logic        cpif_launch;
    logic        cpif_priv_bypass;
    logic        cpif_linef_noproto;
    logic        cpif_emit_model_exc;
    logic [2:0]  cpif_cat;
    logic [4:0]  cpif_cir;
    logic        cpif_read;
    logic [3:0]  cpif_steps;
    logic        cpif_resp_ready;
    logic        cpif_last_step;
    logic [3:0]  cpif_step_idx;
    logic [4:0]  cpif_step_cir_val;
    logic        cpif_step_read_val;
    logic        cpif_badresp;
    if (RESET_CPU) begin
        CPIF_STATE <= CPIF_ST_IDLE;
        CPIF_ACTIVE <= 1'b0;
        CPIF_LAST_CAT <= 3'd0;
        CPIF_LAST_CPID <= 3'b000;
        CPIF_LAST_CIR <= 5'h00;
        CPIF_LAST_OPWORD <= 16'h0000;
        CPIF_LAST_READ <= 1'b0;
        CPIF_LAST_WRITE <= 1'b0;
        CPIF_LAST_RESPONDED <= 1'b0;
        CPIF_LAST_RESP_CIR <= 5'h00;
        CPIF_LAST_PHASE_LEN <= 4'd0;
        CPIF_PHASE_STEPS_LEFT <= 4'd0;
        CPIF_RESP_DELAY_LEFT <= 4'd0;
        CPIF_STEP_INDEX <= 4'd0;
        CPIF_STEP_CIR <= 5'h00;
        CPIF_STEP_READ <= 1'b0;
        CPIF_STEP_WRITE <= 1'b0;
        CPIF_RESP_CIR_EXPECT <= 5'h00;
        CPIF_REQ_PULSE <= 1'b0;
        CPIF_STEP_PULSE <= 1'b0;
        CPIF_RESP_PULSE <= 1'b0;
        CPIF_NORESP_PULSE <= 1'b0;
        CPIF_TIMEOUT_PULSE <= 1'b0;
        CPIF_BADRESP_PULSE <= 1'b0;
        CPIF_REQ_COUNT <= 32'h0000_0000;
        CPIF_STEP_COUNT <= 32'h0000_0000;
        CPIF_RESP_COUNT <= 32'h0000_0000;
        CPIF_NORESP_COUNT <= 32'h0000_0000;
        CPIF_TIMEOUT_COUNT <= 32'h0000_0000;
        CPIF_BADRESP_COUNT <= 32'h0000_0000;
        CPIF_PRIV_BYPASS_COUNT <= 32'h0000_0000;
        CPIF_LINEF_NOPROTO_COUNT <= 32'h0000_0000;
        CPIF_MODEL_EXC_ENABLE <= 1'b1;
        CPIF_MODEL_EXC_ON_NORESP <= 1'b0;
        CPIF_MODEL_EXC_KIND <= 2'd0; // pre-instruction
        CPIF_MODEL_EXC_VECTOR <= VEC_LINE_F;
        CPIF_MODEL_RESP_ENABLE <= 1'b0;
        CPIF_MODEL_RESP_DELAY <= 4'd1;
        CPIF_MODEL_RESP_CIR <= 5'h00;
        CPIF_TRAP_PRE <= 1'b0;
        CPIF_TRAP_MID <= 1'b0;
        CPIF_TRAP_POST <= 1'b0;
        CPIF_TRAP_VECTOR <= VEC_LINE_F;
        CPIF_PRE_EXC_COUNT <= 32'h0000_0000;
        CPIF_MID_EXC_COUNT <= 32'h0000_0000;
        CPIF_POST_EXC_COUNT <= 32'h0000_0000;
        CPIF_PROTO_COUNT <= 32'h0000_0000;
    end else begin
        cpif_launch = OPD_ACK_MAIN &&
                      OP == UNIMPLEMENTED &&
                      TRAP_CODE_OPC == T_1111 &&
                      cpif_is_protocol_candidate(BIW_0);

        cpif_priv_bypass = OPD_ACK_MAIN &&
                           OP == UNIMPLEMENTED &&
                           TRAP_CODE_OPC == T_PRIV &&
                           BIW_0[15:12] == 4'hF &&
                           BIW_0[11:9] != 3'b000 &&
                           (BIW_0[8:6] == 3'b100 || BIW_0[8:6] == 3'b101);

        cpif_linef_noproto = OPD_ACK_MAIN &&
                             OP == UNIMPLEMENTED &&
                             TRAP_CODE_OPC == T_1111 &&
                             BIW_0[15:12] == 4'hF &&
                             BIW_0[11:9] != 3'b000 &&
                             !cpif_is_protocol_candidate(BIW_0);
        cpif_emit_model_exc = 1'b0;
        cpif_resp_ready = 1'b0;
        cpif_last_step = 1'b0;
        cpif_badresp = 1'b0;

        CPIF_REQ_PULSE <= 1'b0;
        CPIF_STEP_PULSE <= 1'b0;
        CPIF_RESP_PULSE <= 1'b0;
        CPIF_NORESP_PULSE <= 1'b0;
        CPIF_TIMEOUT_PULSE <= 1'b0;
        CPIF_BADRESP_PULSE <= 1'b0;
        CPIF_TRAP_PRE <= 1'b0;
        CPIF_TRAP_MID <= 1'b0;
        CPIF_TRAP_POST <= 1'b0;

        if (cpif_priv_bypass)
            CPIF_PRIV_BYPASS_COUNT <= CPIF_PRIV_BYPASS_COUNT + 32'd1;

        if (cpif_linef_noproto)
            CPIF_LINEF_NOPROTO_COUNT <= CPIF_LINEF_NOPROTO_COUNT + 32'd1;

        if (cpif_launch) begin
            cpif_cat = cpif_category(BIW_0);
            cpif_cir = cpif_initiator_cir(cpif_cat);
            cpif_read = cpif_initiator_is_read(cpif_cat);
            cpif_steps = cpif_protocol_steps(cpif_cat);
            if (cpif_steps == 4'd0)
                cpif_steps = 4'd1;

            CPIF_STATE <= CPIF_ST_INIT;
            CPIF_LAST_CAT <= cpif_cat;
            CPIF_LAST_CPID <= BIW_0[11:9];
            CPIF_LAST_CIR <= cpif_cir;
            CPIF_LAST_OPWORD <= BIW_0;
            CPIF_LAST_READ <= cpif_read;
            CPIF_LAST_WRITE <= !cpif_read;
            CPIF_LAST_RESPONDED <= 1'b0;
            CPIF_LAST_RESP_CIR <= 5'h00;
            CPIF_LAST_PHASE_LEN <= cpif_steps;
            CPIF_PHASE_STEPS_LEFT <= cpif_steps;
            CPIF_RESP_DELAY_LEFT <= CPIF_MODEL_RESP_DELAY;
            CPIF_STEP_INDEX <= 4'd0;
            CPIF_STEP_CIR <= cpif_cir;
            CPIF_STEP_READ <= cpif_read;
            CPIF_STEP_WRITE <= !cpif_read;
            CPIF_RESP_CIR_EXPECT <= cpif_response_cir(cpif_cat);
            CPIF_REQ_COUNT <= CPIF_REQ_COUNT + 32'd1;
            CPIF_REQ_PULSE <= 1'b1;
            cpif_emit_model_exc = CPIF_MODEL_EXC_ENABLE && !CPIF_MODEL_EXC_ON_NORESP;
        end else begin
            case (CPIF_STATE)
                CPIF_ST_INIT: begin
                    CPIF_STATE <= CPIF_ST_STEP;
                end
                CPIF_ST_STEP: begin
                    cpif_resp_ready = CPIF_MODEL_RESP_ENABLE && CPIF_RESP_DELAY_LEFT == 4'd0;
                    cpif_last_step = (CPIF_PHASE_STEPS_LEFT <= 4'd1);

                    if (CPIF_PHASE_STEPS_LEFT != 4'd0) begin
                        cpif_step_idx = CPIF_LAST_PHASE_LEN - CPIF_PHASE_STEPS_LEFT;
                        cpif_step_cir_val = cpif_step_cir(CPIF_LAST_CAT, cpif_step_idx);
                        cpif_step_read_val = cpif_step_is_read(CPIF_LAST_CAT, cpif_step_idx);
                        CPIF_PHASE_STEPS_LEFT <= CPIF_PHASE_STEPS_LEFT - 4'd1;
                        CPIF_STEP_INDEX <= cpif_step_idx;
                        CPIF_STEP_CIR <= cpif_step_cir_val;
                        CPIF_STEP_READ <= cpif_step_read_val;
                        CPIF_STEP_WRITE <= !cpif_step_read_val;
                        CPIF_STEP_COUNT <= CPIF_STEP_COUNT + 32'd1;
                        CPIF_STEP_PULSE <= 1'b1;
                    end

                    if (CPIF_MODEL_RESP_ENABLE && CPIF_RESP_DELAY_LEFT != 4'd0)
                        CPIF_RESP_DELAY_LEFT <= CPIF_RESP_DELAY_LEFT - 4'd1;

                    if (cpif_resp_ready)
                        CPIF_STATE <= CPIF_ST_RESP;
                    else if (cpif_last_step)
                        CPIF_STATE <= CPIF_ST_TIMEOUT;
                end
                CPIF_ST_RESP: begin
                    CPIF_STATE <= CPIF_ST_DONE;
                    CPIF_RESP_COUNT <= CPIF_RESP_COUNT + 32'd1;
                    CPIF_RESP_PULSE <= 1'b1;
                    CPIF_LAST_RESPONDED <= 1'b1;
                    CPIF_LAST_RESP_CIR <= CPIF_MODEL_RESP_CIR;
                    cpif_badresp = (CPIF_MODEL_RESP_CIR != CPIF_RESP_CIR_EXPECT);
                    if (cpif_badresp) begin
                        CPIF_BADRESP_COUNT <= CPIF_BADRESP_COUNT + 32'd1;
                        CPIF_BADRESP_PULSE <= 1'b1;
                        CPIF_TRAP_MID <= 1'b1;
                        CPIF_TRAP_VECTOR <= VEC_CP_PROTO;
                        CPIF_MID_EXC_COUNT <= CPIF_MID_EXC_COUNT + 32'd1;
                        CPIF_PROTO_COUNT <= CPIF_PROTO_COUNT + 32'd1;
                    end
                end
                CPIF_ST_TIMEOUT: begin
                    CPIF_STATE <= CPIF_ST_DONE;
                    CPIF_TIMEOUT_COUNT <= CPIF_TIMEOUT_COUNT + 32'd1;
                    CPIF_TIMEOUT_PULSE <= 1'b1;
                    CPIF_NORESP_COUNT <= CPIF_NORESP_COUNT + 32'd1;
                    CPIF_NORESP_PULSE <= 1'b1;
                    cpif_emit_model_exc = CPIF_MODEL_EXC_ENABLE && CPIF_MODEL_EXC_ON_NORESP;
                end
                CPIF_ST_DONE: begin
                    CPIF_STATE <= CPIF_ST_IDLE;
                end
                default: begin
                    CPIF_STATE <= CPIF_ST_IDLE;
                end
            endcase
        end

        if (cpif_emit_model_exc) begin
            CPIF_TRAP_VECTOR <= CPIF_MODEL_EXC_VECTOR;
            case (CPIF_MODEL_EXC_KIND)
                2'd1: begin
                    CPIF_TRAP_MID <= 1'b1;
                    CPIF_MID_EXC_COUNT <= CPIF_MID_EXC_COUNT + 32'd1;
                    if (CPIF_MODEL_EXC_VECTOR == VEC_CP_PROTO)
                        CPIF_PROTO_COUNT <= CPIF_PROTO_COUNT + 32'd1;
                end
                2'd2: begin
                    CPIF_TRAP_POST <= 1'b1;
                    CPIF_POST_EXC_COUNT <= CPIF_POST_EXC_COUNT + 32'd1;
                end
                default: begin
                    CPIF_TRAP_PRE <= 1'b1;
                    CPIF_PRE_EXC_COUNT <= CPIF_PRE_EXC_COUNT + 32'd1;
                end
            endcase
        end

        CPIF_ACTIVE <= (cpif_launch ||
                        CPIF_STATE == CPIF_ST_INIT ||
                        CPIF_STATE == CPIF_ST_STEP ||
                        CPIF_STATE == CPIF_ST_RESP ||
                        CPIF_STATE == CPIF_ST_TIMEOUT ||
                        CPIF_STATE == CPIF_ST_DONE);
    end
end

// MMU register surface, PMOVE/PFLUSH/PLOAD/PTEST handling, and ATC maintenance.
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
    logic [$clog2(MMU_ATC_LINES)-1:0] atc_hit_idx;
    logic [$clog2(MMU_ATC_LINES)-1:0] atc_free_idx;
    logic [$clog2(MMU_ATC_LINES)-1:0] atc_ins_idx;
    integer i;
    if (RESET_CPU) begin
        MMU_SRP <= 64'h0;
        MMU_CRP <= 64'h0;
        MMU_TC <= 32'h0;
        MMU_TT0 <= 32'h0;
        MMU_TT1 <= 32'h0;
        MMU_MMUSR <= 32'h0;
        TRAP_MMU_CFG <= 1'b0;
        MMU_ATC_FLUSH_COUNT <= 32'h0;
        MMU_ATC_V <= '0;
        MMU_ATC_B <= '0;
        MMU_ATC_W <= '0;
        MMU_ATC_M <= '0;
        MMU_ATC_REPL_PTR <= '0;
        for (i = 0; i < MMU_ATC_LINES; i = i + 1) begin
            MMU_ATC_FC[i] <= 3'b000;
            MMU_ATC_TAG[i] <= 32'h0;
            MMU_ATC_PTAG[i] <= 32'h0;
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

        // PMOVE FD=0 register writes flush all ATC entries.
        if (pmove_flush_exec)
            MMU_ATC_V <= '0;

        // PFLUSH supports all-entries, FC/mask, and FC/mask/EA selection.
        if (pflush_exec) begin
            atc_fc = mmu_fc_decode(BIW_1[4:0], DR_OUT_1, SFC, DFC);
            atc_mask = BIW_1[7:5];
            atc_logical = ADR_EFF;
            atc_tag = mmu_page_tag(MMU_TC, atc_logical);
            case (BIW_1[12:10])
                3'b001: begin
                    MMU_ATC_V <= '0; // PFLUSHA
                end
                3'b100: begin // PFLUSH FC,MASK
                    for (i = 0; i < MMU_ATC_LINES; i = i + 1) begin
                        if (MMU_ATC_V[i] && (((MMU_ATC_FC[i] ^ atc_fc) & atc_mask) == 3'b000))
                            MMU_ATC_V[i] <= 1'b0;
                    end
                end
                3'b110: begin // PFLUSH FC,MASK,<ea>
                    for (i = 0; i < MMU_ATC_LINES; i = i + 1) begin
                        if (MMU_ATC_V[i] &&
                            (((MMU_ATC_FC[i] ^ atc_fc) & atc_mask) == 3'b000) &&
                            (MMU_ATC_TAG[i] == atc_tag))
                            MMU_ATC_V[i] <= 1'b0;
                    end
                end
                default: begin
                end
            endcase
        end

        // PLOAD updates the ATC entry for <FC, logical page>; MMUSR is unchanged.
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
            atc_m_result = !BIW_1[9]; // PLOADW marks modified.

            atc_hit = 1'b0;
            atc_free = 1'b0;
            atc_hit_idx = '0;
            atc_free_idx = '0;
            for (i = 0; i < MMU_ATC_LINES; i = i + 1) begin
                if (!atc_hit && MMU_ATC_V[i] && MMU_ATC_FC[i] == atc_fc && MMU_ATC_TAG[i] == atc_tag) begin
                    atc_hit = 1'b1;
                    atc_hit_idx = i[$clog2(MMU_ATC_LINES)-1:0];
                end
                if (!atc_free && !MMU_ATC_V[i]) begin
                    atc_free = 1'b1;
                    atc_free_idx = i[$clog2(MMU_ATC_LINES)-1:0];
                end
            end

            atc_ins_idx = atc_hit ? atc_hit_idx : (atc_free ? atc_free_idx : MMU_ATC_REPL_PTR);

            // Keep TT-only mappings out of the ATC model.
            if (!tt_hit) begin
                for (i = 0; i < MMU_ATC_LINES; i = i + 1) begin
                    if (MMU_ATC_V[i] && MMU_ATC_FC[i] == atc_fc && MMU_ATC_TAG[i] == atc_tag)
                        MMU_ATC_V[i] <= 1'b0;
                end
                MMU_ATC_V[atc_ins_idx] <= 1'b1;
                MMU_ATC_B[atc_ins_idx] <= atc_b_result;
                MMU_ATC_W[atc_ins_idx] <= atc_w_result;
                MMU_ATC_M[atc_ins_idx] <= atc_m_result;
                MMU_ATC_FC[atc_ins_idx] <= atc_fc;
                MMU_ATC_TAG[atc_ins_idx] <= atc_tag;
                MMU_ATC_PTAG[atc_ins_idx] <= atc_ptag;
                if (!atc_hit && !atc_free)
                    MMU_ATC_REPL_PTR <= MMU_ATC_REPL_PTR + 1'b1;
            end
        end

        // Runtime translation miss fill: insert resolved mapping into ATC.
        if (MMU_RUNTIME_ATC_REFILL) begin
            atc_fc = MMU_RUNTIME_ATC_FC;
            atc_tag = MMU_RUNTIME_ATC_TAG;
            atc_ptag = MMU_RUNTIME_ATC_PTAG;
            atc_b_result = MMU_RUNTIME_ATC_B;
            atc_w_result = MMU_RUNTIME_ATC_W;
            atc_m_result = MMU_RUNTIME_ATC_M;

            atc_hit = 1'b0;
            atc_free = 1'b0;
            atc_hit_idx = '0;
            atc_free_idx = '0;
            for (i = 0; i < MMU_ATC_LINES; i = i + 1) begin
                if (!atc_hit && MMU_ATC_V[i] && MMU_ATC_FC[i] == atc_fc && MMU_ATC_TAG[i] == atc_tag) begin
                    atc_hit = 1'b1;
                    atc_hit_idx = i[$clog2(MMU_ATC_LINES)-1:0];
                end
                if (!atc_free && !MMU_ATC_V[i]) begin
                    atc_free = 1'b1;
                    atc_free_idx = i[$clog2(MMU_ATC_LINES)-1:0];
                end
            end

            atc_ins_idx = atc_hit ? atc_hit_idx : (atc_free ? atc_free_idx : MMU_ATC_REPL_PTR);

            for (i = 0; i < MMU_ATC_LINES; i = i + 1) begin
                if (MMU_ATC_V[i] && MMU_ATC_FC[i] == atc_fc && MMU_ATC_TAG[i] == atc_tag)
                    MMU_ATC_V[i] <= 1'b0;
            end
            MMU_ATC_V[atc_ins_idx] <= 1'b1;
            MMU_ATC_B[atc_ins_idx] <= atc_b_result;
            MMU_ATC_W[atc_ins_idx] <= atc_w_result;
            MMU_ATC_M[atc_ins_idx] <= atc_m_result;
            MMU_ATC_FC[atc_ins_idx] <= atc_fc;
            MMU_ATC_TAG[atc_ins_idx] <= atc_tag;
            MMU_ATC_PTAG[atc_ins_idx] <= atc_ptag;
            if (!atc_hit && !atc_free)
                MMU_ATC_REPL_PTR <= MMU_ATC_REPL_PTR + 1'b1;
        end

        // PTEST writes MMUSR with level-specific status.
        if (ptest_exec) begin
            atc_fc = mmu_fc_decode(BIW_1[4:0], DR_OUT_1, SFC, DFC);
            ptest_level = BIW_1[12:10];
            atc_logical = ADR_EFF;
            atc_tag = mmu_page_tag(MMU_TC, atc_logical);
            atc_rmw = 1'b0;

            tt_hit = mmu_tt_match(MMU_TT0, atc_fc, atc_logical, BIW_1[9], !BIW_1[9], atc_rmw) ||
                     mmu_tt_match(MMU_TT1, atc_fc, atc_logical, BIW_1[9], !BIW_1[9], atc_rmw);

            atc_hit = 1'b0;
            atc_hit_idx = '0;
            atc_b_result = 1'b0;
            atc_w_result = 1'b0;
            atc_m_result = 1'b0;
            for (i = 0; i < MMU_ATC_LINES; i = i + 1) begin
                if (!atc_hit && MMU_ATC_V[i] && MMU_ATC_FC[i] == atc_fc && MMU_ATC_TAG[i] == atc_tag) begin
                    atc_hit = 1'b1;
                    atc_hit_idx = i[$clog2(MMU_ATC_LINES)-1:0];
                    atc_b_result = MMU_ATC_B[i];
                    atc_w_result = MMU_ATC_W[i];
                    atc_m_result = MMU_ATC_M[i];
                end
            end

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
                // Level 1-7 search derives B/L/S/W/I/M/N from the descriptor-walk model.
                mmusr_value = mmu_ptest_walk_mmusr(atc_fc, atc_logical, ptest_level);
            end
            MMU_MMUSR <= {16'h0, mmusr_value};
        end
    end
end
