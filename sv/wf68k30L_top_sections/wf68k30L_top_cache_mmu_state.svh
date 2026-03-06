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

WF68K30L_TOP_CACHE_STATE I_TOP_CACHE_STATE (
    .CLK(CLK),
    .RESET_CPU(RESET_CPU),
    .BUS_BSY(BUS_BSY),
    .DATA_RDY_BUSIF_CORE(DATA_RDY_BUSIF_CORE),
    .OPCODE_REQ_CORE(OPCODE_REQ_CORE),
    .ADR_P_PHYS(ADR_P_PHYS),
    .ICACHE_RAM_RD_DATA(ICACHE_RAM_RD_DATA),
    .DATA_RD(DATA_RD),
    .OP_SIZE_BUS(OP_SIZE_BUS),
    .OPCODE_REQ(OPCODE_REQ),
    .ADR_BUS_REQ_PHYS(ADR_BUS_REQ_PHYS),
    .BURST_PREFETCH_OP_REQ(BURST_PREFETCH_OP_REQ),
    .FC_I(FC_I),
    .MMU_TT0(MMU_TT0),
    .MMU_TT1(MMU_TT1),
    .FC_BUS_REQ(FC_BUS_REQ),
    .OPCODE_RDY_BUSIF(OPCODE_RDY_BUSIF),
    .DATA_RD_BUS(DATA_RD_BUS),
    .MMU_RUNTIME_FAULT(MMU_RUNTIME_FAULT),
    .MMU_RUNTIME_STALL(MMU_RUNTIME_STALL),
    .BURST_PREFETCH_DATA_REQ(BURST_PREFETCH_DATA_REQ),
    .RMC(RMC),
    .DATA_RDY_BUSIF(DATA_RDY_BUSIF),
    .DATA_VALID_BUSIF(DATA_VALID_BUSIF),
    .OPCODE_VALID_BUSIF(OPCODE_VALID_BUSIF),
    .BERRn(BERRn),
    .CBACKn(CBACKn),
    .BUS_CYCLE_BURST(BUS_CYCLE_BURST),
    .OPCODE_TO_CORE_BUSIF(OPCODE_TO_CORE_BUSIF),
    .DATA_TO_CORE_BUSIF(DATA_TO_CORE_BUSIF),
    .DATA_WR(DATA_WR),
    .DATA_FROM_CORE(DATA_FROM_CORE),
    .CACR_WR(CACR_WR),
    .CAAR_WR(CAAR_WR),
    .ALU_RESULT(ALU_RESULT),
    .CACR(CACR),
    .CAAR(CAAR),
    .ICACHE_HIT_NOW(ICACHE_HIT_NOW),
    .ICACHE_RDY(ICACHE_RDY),
    .ICACHE_OPCODE_WORD(ICACHE_OPCODE_WORD),
    .ICACHE_FILL_PENDING(ICACHE_FILL_PENDING),
    .ICACHE_FILL_ADDR(ICACHE_FILL_ADDR),
    .ICACHE_FILL_CACHEABLE(ICACHE_FILL_CACHEABLE),
    .ICACHE_FILL_FC(ICACHE_FILL_FC),
    .ICACHE_BURST_TRACK_VALID(ICACHE_BURST_TRACK_VALID),
    .ICACHE_BURST_TRACK_LINE(ICACHE_BURST_TRACK_LINE),
    .ICACHE_BURST_TRACK_TAG(ICACHE_BURST_TRACK_TAG),
    .ICACHE_BURST_FILL_VALID(ICACHE_BURST_FILL_VALID),
    .ICACHE_BURST_FILL_LINE(ICACHE_BURST_FILL_LINE),
    .ICACHE_BURST_FILL_TAG(ICACHE_BURST_FILL_TAG),
    .ICACHE_BURST_FILL_PENDING(ICACHE_BURST_FILL_PENDING),
    .ICACHE_BURST_FILL_FC(ICACHE_BURST_FILL_FC),
    .ICACHE_BURST_FILL_NEXT_WORD(ICACHE_BURST_FILL_NEXT_WORD),
    .DATA_RDY_CACHE(DATA_RDY_CACHE),
    .DATA_VALID_CACHE(DATA_VALID_CACHE),
    .DATA_TO_CORE_CACHE(DATA_TO_CORE_CACHE),
    .DATA_LAST_FROM_CACHE(DATA_LAST_FROM_CACHE),
    .DCACHE_HIT_PENDING(DCACHE_HIT_PENDING),
    .DCACHE_HIT_SIZE_PENDING(DCACHE_HIT_SIZE_PENDING),
    .DCACHE_HIT_ADDR10_PENDING(DCACHE_HIT_ADDR10_PENDING),
    .ICACHE_RAM_RD_EN(ICACHE_RAM_RD_EN),
    .ICACHE_RAM_RD_ADDR(ICACHE_RAM_RD_ADDR),
    .ICACHE_RAM_WR_EN(ICACHE_RAM_WR_EN),
    .ICACHE_RAM_WR_ADDR(ICACHE_RAM_WR_ADDR),
    .ICACHE_RAM_WR_DATA(ICACHE_RAM_WR_DATA),
    .DCACHE_RAM_RD_EN(DCACHE_RAM_RD_EN),
    .DCACHE_RAM_RD_ADDR(DCACHE_RAM_RD_ADDR),
    .DCACHE_RAM_RD_DATA(DCACHE_RAM_RD_DATA),
    .DCACHE_RAM_WR_EN(DCACHE_RAM_WR_EN),
    .DCACHE_RAM_WR_ADDR(DCACHE_RAM_WR_ADDR),
    .DCACHE_RAM_WR_DATA(DCACHE_RAM_WR_DATA),
    .DCACHE_HIT_NOW(DCACHE_HIT_NOW),
    .DCACHE_READ_FILL_PENDING(DCACHE_READ_FILL_PENDING),
    .DCACHE_READ_FILL_ADDR(DCACHE_READ_FILL_ADDR),
    .DCACHE_READ_FILL_SIZE(DCACHE_READ_FILL_SIZE),
    .DCACHE_READ_FILL_CACHEABLE(DCACHE_READ_FILL_CACHEABLE),
    .DCACHE_READ_FILL_FC(DCACHE_READ_FILL_FC),
    .DCACHE_BURST_TRACK_VALID(DCACHE_BURST_TRACK_VALID),
    .DCACHE_BURST_TRACK_LINE(DCACHE_BURST_TRACK_LINE),
    .DCACHE_BURST_TRACK_TAG(DCACHE_BURST_TRACK_TAG),
    .DCACHE_BURST_FILL_VALID(DCACHE_BURST_FILL_VALID),
    .DCACHE_BURST_FILL_LINE(DCACHE_BURST_FILL_LINE),
    .DCACHE_BURST_FILL_TAG(DCACHE_BURST_FILL_TAG),
    .DCACHE_BURST_FILL_PENDING(DCACHE_BURST_FILL_PENDING),
    .DCACHE_BURST_FILL_FC(DCACHE_BURST_FILL_FC),
    .DCACHE_BURST_FILL_NEXT_ENTRY(DCACHE_BURST_FILL_NEXT_ENTRY),
    .DCACHE_WRITE_PENDING(DCACHE_WRITE_PENDING),
    .DCACHE_WRITE_ADDR(DCACHE_WRITE_ADDR),
    .DCACHE_WRITE_SIZE(DCACHE_WRITE_SIZE),
    .DCACHE_WRITE_DATA(DCACHE_WRITE_DATA),
    .DCACHE_WRITE_CACHEABLE(DCACHE_WRITE_CACHEABLE)
);

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

assign MMU_PTEST_FC = mmu_fc_decode(BIW_1[4:0], DR_OUT_1, SFC, DFC);
assign MMU_PTEST_LEVEL = BIW_1[12:10];
assign MMU_PTEST_LOGICAL = ADR_EFF;
assign MMU_PTEST_START = (OP_WB == PTEST) &&
                         (MMU_PTEST_LEVEL != 3'b000) &&
                         ALU_BSY &&
                         ALU_REQ &&
                         !MMU_PTEST_BUSY &&
                         !MMU_PTEST_READY;
assign MMU_PTEST_CONSUME = (OP_WB == PTEST) &&
                           (MMU_PTEST_LEVEL != 3'b000) &&
                           ALU_ACK;

WF68K30L_TOP_MMU_STATE #(
    .MMU_ATC_LINES(MMU_ATC_LINES),
    .MMU_ATC_WAYS(MMU_ATC_WAYS),
    .MMU_ATC_SETS(MMU_ATC_SETS),
    .MMU_ATC_SET_BITS(MMU_ATC_SET_BITS),
    .MMU_ATC_WAY_BITS(MMU_ATC_WAY_BITS)
) I_TOP_MMU_STATE (
    .CLK(CLK),
    .RESET_CPU(RESET_CPU),
    .ALU_RESULT(ALU_RESULT),
    .ALU_ACK(ALU_ACK),
    .OP_WB(OP_WB),
    .MMU_ATC_FLUSH(MMU_ATC_FLUSH),
    .MMU_TC_WR(MMU_TC_WR),
    .MMU_SRP_WR(MMU_SRP_WR),
    .MMU_CRP_WR(MMU_CRP_WR),
    .MMU_TT0_WR(MMU_TT0_WR),
    .MMU_TT1_WR(MMU_TT1_WR),
    .MMU_MMUSR_WR(MMU_MMUSR_WR),
    .MMU_RUNTIME_REQ(MMU_RUNTIME_REQ),
    .MMU_RUNTIME_FAULT(MMU_RUNTIME_FAULT),
    .MMU_RUNTIME_ATC_REFILL(MMU_RUNTIME_ATC_REFILL),
    .MMU_RUNTIME_ATC_FC(MMU_RUNTIME_ATC_FC),
    .MMU_RUNTIME_ATC_TAG(MMU_RUNTIME_ATC_TAG),
    .MMU_RUNTIME_ATC_PTAG(MMU_RUNTIME_ATC_PTAG),
    .MMU_RUNTIME_ATC_B(MMU_RUNTIME_ATC_B),
    .MMU_RUNTIME_ATC_W(MMU_RUNTIME_ATC_W),
    .MMU_RUNTIME_ATC_M(MMU_RUNTIME_ATC_M),
    .BIW_1(BIW_1),
    .DR_OUT_1(DR_OUT_1),
    .SFC(SFC),
    .DFC(DFC),
    .ADR_EFF(ADR_EFF),
    .PTEST_WALK_START(MMU_PTEST_START),
    .PTEST_WALK_MMUSR(MMU_PTEST_WALK_MMUSR),
    .MMU_SRP(MMU_SRP),
    .MMU_CRP(MMU_CRP),
    .MMU_TC(MMU_TC),
    .MMU_TT0(MMU_TT0),
    .MMU_TT1(MMU_TT1),
    .MMU_MMUSR(MMU_MMUSR),
    .TRAP_MMU_CFG(TRAP_MMU_CFG),
    .MMU_ATC_FLUSH_COUNT(MMU_ATC_FLUSH_COUNT),
    .MMU_ATC_V_FLAT(MMU_ATC_V_FLAT),
    .MMU_ATC_B_FLAT(MMU_ATC_B_FLAT),
    .MMU_ATC_W_FLAT(MMU_ATC_W_FLAT),
    .MMU_ATC_M_FLAT(MMU_ATC_M_FLAT),
    .MMU_ATC_FC_FLAT(MMU_ATC_FC_FLAT),
    .MMU_ATC_TAG_FLAT(MMU_ATC_TAG_FLAT),
    .MMU_ATC_PTAG_FLAT(MMU_ATC_PTAG_FLAT)
);
