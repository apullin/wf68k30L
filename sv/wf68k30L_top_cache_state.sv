(* keep_hierarchy = "yes" *)
module WF68K30L_TOP_CACHE_STATE (
    input  logic        CLK,
    input  logic        RESET_CPU,
    input  logic        BUS_BSY,
    input  logic        DATA_RDY_BUSIF_CORE,
    input  logic        OPCODE_REQ_CORE,
    output logic        ICACHE_HIT_NOW,
    input  logic [31:0] ADR_P_PHYS,
    input  logic [15:0] ICACHE_RAM_RD_DATA,
    input  logic        DATA_RD,
    output logic        DCACHE_HIT_NOW,
    input  logic [1:0]  OP_SIZE_BUS,
    input  logic        OPCODE_REQ,
    input  logic [31:0] ADR_BUS_REQ_PHYS,
    input  logic        BURST_PREFETCH_OP_REQ,
    input  logic [2:0]  FC_I,
    input  logic [31:0] MMU_TT0,
    input  logic [31:0] MMU_TT1,
    input  logic [2:0]  FC_BUS_REQ,
    input  logic        OPCODE_RDY_BUSIF,
    input  logic        DATA_RD_BUS,
    input  logic        MMU_RUNTIME_FAULT,
    input  logic        MMU_RUNTIME_STALL,
    input  logic        BURST_PREFETCH_DATA_REQ,
    input  logic        RMC,
    input  logic        DATA_RDY_BUSIF,
    input  logic        DATA_VALID_BUSIF,
    input  logic        OPCODE_VALID_BUSIF,
    input  logic        BERRn,
    input  logic        CBACKn,
    input  logic        BUS_CYCLE_BURST,
    input  logic [15:0] OPCODE_TO_CORE_BUSIF,
    input  logic [31:0] DATA_TO_CORE_BUSIF,
    input  logic        DATA_WR,
    input  logic [31:0] DATA_FROM_CORE,
    input  logic        CACR_WR,
    input  logic        CAAR_WR,
    input  logic [63:0] ALU_RESULT,

    output logic [31:0] CACR,
    output logic [31:0] CAAR,
    output logic        ICACHE_RDY,
    output logic [15:0] ICACHE_OPCODE_WORD,
    output logic        ICACHE_FILL_PENDING,
    output logic [31:0] ICACHE_FILL_ADDR,
    output logic        ICACHE_FILL_CACHEABLE,
    output logic [2:0]  ICACHE_FILL_FC,
    output logic        ICACHE_BURST_TRACK_VALID,
    output logic [3:0]  ICACHE_BURST_TRACK_LINE,
    output logic [23:0] ICACHE_BURST_TRACK_TAG,
    output logic        ICACHE_BURST_FILL_VALID,
    output logic [3:0]  ICACHE_BURST_FILL_LINE,
    output logic [23:0] ICACHE_BURST_FILL_TAG,
    output logic [7:0]  ICACHE_BURST_FILL_PENDING,
    output logic [2:0]  ICACHE_BURST_FILL_FC,
    output logic [2:0]  ICACHE_BURST_FILL_NEXT_WORD,
    output logic        DATA_RDY_CACHE,
    output logic        DATA_VALID_CACHE,
    output logic [31:0] DATA_TO_CORE_CACHE,
    output logic        DATA_LAST_FROM_CACHE,
    output logic        DCACHE_HIT_PENDING,
    output logic [1:0]  DCACHE_HIT_SIZE_PENDING,
    output logic [1:0]  DCACHE_HIT_ADDR10_PENDING,
    output logic        ICACHE_RAM_RD_EN,
    output logic [6:0]  ICACHE_RAM_RD_ADDR,
    output logic        ICACHE_RAM_WR_EN,
    output logic [6:0]  ICACHE_RAM_WR_ADDR,
    output logic [15:0] ICACHE_RAM_WR_DATA,
    output logic        DCACHE_RAM_RD_EN,
    output logic [5:0]  DCACHE_RAM_RD_ADDR,
    input  logic [31:0] DCACHE_RAM_RD_DATA,
    output logic        DCACHE_RAM_WR_EN,
    output logic [5:0]  DCACHE_RAM_WR_ADDR,
    output logic [31:0] DCACHE_RAM_WR_DATA,
    output logic        DCACHE_READ_FILL_PENDING,
    output logic [31:0] DCACHE_READ_FILL_ADDR,
    output logic [1:0]  DCACHE_READ_FILL_SIZE,
    output logic        DCACHE_READ_FILL_CACHEABLE,
    output logic [2:0]  DCACHE_READ_FILL_FC,
    output logic        DCACHE_BURST_TRACK_VALID,
    output logic [3:0]  DCACHE_BURST_TRACK_LINE,
    output logic [23:0] DCACHE_BURST_TRACK_TAG,
    output logic        DCACHE_BURST_FILL_VALID,
    output logic [3:0]  DCACHE_BURST_FILL_LINE,
    output logic [23:0] DCACHE_BURST_FILL_TAG,
    output logic [3:0]  DCACHE_BURST_FILL_PENDING,
    output logic [2:0]  DCACHE_BURST_FILL_FC,
    output logic [1:0]  DCACHE_BURST_FILL_NEXT_ENTRY,
    output logic        DCACHE_WRITE_PENDING,
    output logic [31:0] DCACHE_WRITE_ADDR,
    output logic [1:0]  DCACHE_WRITE_SIZE,
    output logic [31:0] DCACHE_WRITE_DATA,
    output logic        DCACHE_WRITE_CACHEABLE
);

`include "wf68k30L_pkg.svh"
`include "wf68k30L_top_sections/helpers/wf68k30L_top_helpers_mmu_pure.svh"
`include "wf68k30L_top_sections/helpers/wf68k30L_top_helpers_cache.svh"

localparam logic [31:0] CACR_RW_MASK = 32'h0000_3313;
localparam int ICACHE_LINES = 16;
localparam int DCACHE_LINES = 16;

logic [23:0] ICACHE_TAG [0:15];
logic [7:0]  ICACHE_VALID [0:15];
logic [23:0] DCACHE_TAG [0:15];
logic [3:0]  DCACHE_VALID [0:15];
logic        ICACHE_HIT_PENDING;

always_comb begin : icache_lookup
    logic [3:0]  req_line;
    logic [2:0]  req_word;
    logic [23:0] req_tag;
    logic        req_cacheable;
    ICACHE_HIT_NOW = 1'b0;
    req_line = 4'h0;
    req_word = 3'h0;
    req_tag = 24'h0;
    req_cacheable = 1'b0;

    if (!BUS_BSY && OPCODE_REQ_CORE && CACR[0]) begin
        req_cacheable = !mmu_cache_inhibit(FC_I, ADR_P_PHYS, 1'b1, 1'b0, 1'b0, MMU_TT0, MMU_TT1);
        req_line = ADR_P_PHYS[7:4];
        req_word = ADR_P_PHYS[3:1];
        req_tag = ADR_P_PHYS[31:8];
        if (req_cacheable &&
            ICACHE_TAG[req_line] == req_tag &&
            ICACHE_VALID[req_line][req_word]) begin
            ICACHE_HIT_NOW = 1'b1;
        end
    end
end

always_comb begin : dcache_lookup
    logic [3:0]  req_line;
    logic [1:0]  req_entry;
    logic [23:0] req_tag;
    logic        req_cacheable;
    DCACHE_HIT_NOW = 1'b0;
    req_line = 4'h0;
    req_entry = 2'h0;
    req_tag = 24'h0;
    req_cacheable = 1'b0;

    if (!BUS_BSY && DATA_RD && CACR[8] && !RMC) begin
        req_cacheable = !mmu_cache_inhibit(FC_I, ADR_P_PHYS, 1'b1, 1'b0, 1'b0, MMU_TT0, MMU_TT1);
        req_line = ADR_P_PHYS[7:4];
        req_entry = ADR_P_PHYS[3:2];
        req_tag = ADR_P_PHYS[31:8];
        if (req_cacheable &&
            dcache_access_supported(OP_SIZE_BUS, ADR_P_PHYS[1:0]) &&
            DCACHE_TAG[req_line] == req_tag &&
            DCACHE_VALID[req_line][req_entry]) begin
            DCACHE_HIT_NOW = 1'b1;
        end
    end
end

always_ff @(posedge CLK) begin : cache_registers
    integer i;
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
    logic        dcache_hit;
    if (RESET_CPU) begin
        CACR <= 32'h0;
        CAAR <= 32'h0;
        ICACHE_RDY <= 1'b0;
        ICACHE_OPCODE_WORD <= 16'h0000;
        ICACHE_HIT_PENDING <= 1'b0;
        ICACHE_RAM_RD_EN <= 1'b0;
        ICACHE_RAM_RD_ADDR <= 7'h00;
        ICACHE_RAM_WR_EN <= 1'b0;
        ICACHE_RAM_WR_ADDR <= 7'h00;
        ICACHE_RAM_WR_DATA <= 16'h0000;
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
        DCACHE_HIT_SIZE_PENDING <= LONG;
        DCACHE_HIT_ADDR10_PENDING <= 2'b00;
        DCACHE_RAM_RD_EN <= 1'b0;
        DCACHE_RAM_RD_ADDR <= 6'h00;
        DCACHE_RAM_WR_EN <= 1'b0;
        DCACHE_RAM_WR_ADDR <= 6'h00;
        DCACHE_RAM_WR_DATA <= 32'h0000_0000;
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
        end
        for (i = 0; i < DCACHE_LINES; i = i + 1) begin
            DCACHE_TAG[i] <= 24'h0;
            DCACHE_VALID[i] <= 4'h0;
        end
    end else begin
        ICACHE_RDY <= 1'b0;
        ICACHE_RAM_RD_EN <= 1'b0;
        ICACHE_RAM_WR_EN <= 1'b0;
        DCACHE_RAM_RD_EN <= 1'b0;
        DCACHE_RAM_WR_EN <= 1'b0;
        DATA_RDY_CACHE <= 1'b0;
        DATA_VALID_CACHE <= 1'b0;

        if (DATA_RDY_CACHE)
            DATA_LAST_FROM_CACHE <= 1'b1;
        else if (DATA_RDY_BUSIF_CORE)
            DATA_LAST_FROM_CACHE <= 1'b0;

        if (ICACHE_HIT_PENDING) begin
            ICACHE_OPCODE_WORD <= ICACHE_RAM_RD_DATA;
            ICACHE_RDY <= 1'b1;
            ICACHE_HIT_PENDING <= 1'b0;
        end else if (!BUS_BSY && OPCODE_REQ_CORE && ICACHE_HIT_NOW) begin
            ICACHE_RAM_RD_EN <= 1'b1;
            ICACHE_RAM_RD_ADDR <= {ADR_P_PHYS[7:4], ADR_P_PHYS[3:1]};
            ICACHE_HIT_PENDING <= 1'b1;
        end

        if (DCACHE_HIT_PENDING) begin
            DATA_TO_CORE_CACHE <= dcache_read_extract(
                DCACHE_RAM_RD_DATA,
                DCACHE_HIT_SIZE_PENDING,
                DCACHE_HIT_ADDR10_PENDING
            );
            DATA_RDY_CACHE <= 1'b1;
            DATA_VALID_CACHE <= 1'b1;
            DCACHE_HIT_PENDING <= 1'b0;
        end else if (!BUS_BSY && DATA_RD && DCACHE_HIT_NOW && !DATA_RDY_BUSIF_CORE) begin
            dcache_line = ADR_P_PHYS[7:4];
            dcache_entry = ADR_P_PHYS[3:2];
            DCACHE_RAM_RD_EN <= 1'b1;
            DCACHE_RAM_RD_ADDR <= {dcache_line, dcache_entry};
            DCACHE_HIT_SIZE_PENDING <= OP_SIZE_BUS;
            DCACHE_HIT_ADDR10_PENDING <= ADR_P_PHYS[1:0];
            DCACHE_HIT_PENDING <= 1'b1;
        end

        if (!BUS_BSY && OPCODE_REQ) begin
            ICACHE_FILL_PENDING <= 1'b1;
            ICACHE_FILL_ADDR <= ADR_BUS_REQ_PHYS;
            ICACHE_FILL_CACHEABLE <= BURST_PREFETCH_OP_REQ ? 1'b1 :
                                     !mmu_cache_inhibit(FC_I, ADR_P_PHYS, 1'b1, 1'b0, 1'b0, MMU_TT0, MMU_TT1);
            ICACHE_FILL_FC <= FC_BUS_REQ;
        end else if (OPCODE_RDY_BUSIF) begin
            ICACHE_FILL_PENDING <= 1'b0;
        end

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

        if (!BUS_BSY && DATA_WR && !MMU_RUNTIME_FAULT && !MMU_RUNTIME_STALL) begin
            DCACHE_WRITE_PENDING <= 1'b1;
            DCACHE_WRITE_ADDR <= ADR_P_PHYS;
            DCACHE_WRITE_SIZE <= OP_SIZE_BUS;
            DCACHE_WRITE_DATA <= DATA_FROM_CORE;
            DCACHE_WRITE_CACHEABLE <= !mmu_cache_inhibit(FC_I, ADR_P_PHYS, 1'b0, 1'b1, RMC, MMU_TT0, MMU_TT1);
        end else if (DATA_RDY_BUSIF_CORE && DCACHE_WRITE_PENDING) begin
            DCACHE_WRITE_PENDING <= 1'b0;
        end

        if (OPCODE_RDY_BUSIF && OPCODE_VALID_BUSIF && BERRn &&
            ICACHE_FILL_PENDING && ICACHE_FILL_CACHEABLE && CACR[0] && !CACR[1]) begin
            fill_line = ICACHE_FILL_ADDR[7:4];
            fill_word = ICACHE_FILL_ADDR[3:1];
            fill_tag = ICACHE_FILL_ADDR[31:8];
            if (ICACHE_TAG[fill_line] != fill_tag)
                ICACHE_VALID[fill_line] <= 8'h00;
            ICACHE_TAG[fill_line] <= fill_tag;
            ICACHE_RAM_WR_EN <= 1'b1;
            ICACHE_RAM_WR_ADDR <= {fill_line, fill_word};
            ICACHE_RAM_WR_DATA <= OPCODE_TO_CORE_BUSIF;
            ICACHE_VALID[fill_line][fill_word] <= 1'b1;

            icache_valid_after = (ICACHE_TAG[fill_line] == fill_tag) ?
                                 (ICACHE_VALID[fill_line] | (8'h01 << fill_word)) :
                                 (8'h01 << fill_word);
            icache_burst_pending_after = ICACHE_BURST_FILL_PENDING & ~(8'h01 << fill_word);

            if (CACR[4] && !CBACKn) begin
                ICACHE_BURST_TRACK_VALID <= 1'b1;
                ICACHE_BURST_TRACK_LINE <= fill_line;
                ICACHE_BURST_TRACK_TAG <= fill_tag;
            end else if (ICACHE_BURST_TRACK_VALID &&
                         ICACHE_BURST_TRACK_LINE == fill_line &&
                         ICACHE_BURST_TRACK_TAG == fill_tag &&
                         icache_valid_after == 8'hFF) begin
                ICACHE_BURST_TRACK_VALID <= 1'b0;
            end else if (ICACHE_BURST_TRACK_VALID &&
                         (ICACHE_BURST_TRACK_LINE != fill_line || ICACHE_BURST_TRACK_TAG != fill_tag)) begin
                ICACHE_BURST_TRACK_VALID <= 1'b0;
            end

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

        if (DATA_RDY_BUSIF && DATA_VALID_BUSIF && BERRn &&
            DCACHE_READ_FILL_PENDING && DCACHE_READ_FILL_CACHEABLE &&
            CACR[8] && !CACR[9] && DCACHE_READ_FILL_SIZE == LONG && DCACHE_READ_FILL_ADDR[1:0] == 2'b00) begin
            dcache_line = DCACHE_READ_FILL_ADDR[7:4];
            dcache_entry = DCACHE_READ_FILL_ADDR[3:2];
            dcache_tag = DCACHE_READ_FILL_ADDR[31:8];
            if (DCACHE_TAG[dcache_line] != dcache_tag)
                DCACHE_VALID[dcache_line] <= 4'h0;
            DCACHE_TAG[dcache_line] <= dcache_tag;
            DCACHE_RAM_WR_EN <= 1'b1;
            DCACHE_RAM_WR_ADDR <= {dcache_line, dcache_entry};
            DCACHE_RAM_WR_DATA <= DATA_TO_CORE_BUSIF;
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

        if (DATA_RDY_BUSIF && DATA_VALID_BUSIF && BERRn &&
            DCACHE_WRITE_PENDING && DCACHE_WRITE_CACHEABLE &&
            CACR[8] && dcache_access_supported(DCACHE_WRITE_SIZE, DCACHE_WRITE_ADDR[1:0])) begin
            dcache_line = DCACHE_WRITE_ADDR[7:4];
            dcache_entry = DCACHE_WRITE_ADDR[3:2];
            dcache_tag = DCACHE_WRITE_ADDR[31:8];
            dcache_hit = (DCACHE_TAG[dcache_line] == dcache_tag) && DCACHE_VALID[dcache_line][dcache_entry];
            if (dcache_hit) begin
                if (DCACHE_WRITE_SIZE == LONG && DCACHE_WRITE_ADDR[1:0] == 2'b00) begin
                    DCACHE_RAM_WR_EN <= 1'b1;
                    DCACHE_RAM_WR_ADDR <= {dcache_line, dcache_entry};
                    DCACHE_RAM_WR_DATA <= DCACHE_WRITE_DATA;
                    DCACHE_VALID[dcache_line][dcache_entry] <= 1'b1;
                end else begin
                    DCACHE_VALID[dcache_line][dcache_entry] <= 1'b0;
                end
            end else if (CACR[13] && !CACR[9] && DCACHE_WRITE_SIZE == LONG && DCACHE_WRITE_ADDR[1:0] == 2'b00) begin
                if (DCACHE_TAG[dcache_line] != dcache_tag)
                    DCACHE_VALID[dcache_line] <= 4'h0;
                DCACHE_TAG[dcache_line] <= dcache_tag;
                DCACHE_RAM_WR_EN <= 1'b1;
                DCACHE_RAM_WR_ADDR <= {dcache_line, dcache_entry};
                DCACHE_RAM_WR_DATA <= DCACHE_WRITE_DATA;
                DCACHE_VALID[dcache_line][dcache_entry] <= 1'b1;
            end
        end

        if (CACR_WR) begin
            cacr_write_value = ALU_RESULT[31:0];
            caar_line = CAAR[7:4];
            caar_entry = CAAR[3:2];
            CACR <= cacr_write_value & CACR_RW_MASK;

            if (cacr_write_value[11]) begin
                for (i = 0; i < DCACHE_LINES; i = i + 1)
                    DCACHE_VALID[i] <= 4'h0;
                DCACHE_BURST_TRACK_VALID <= 1'b0;
                DCACHE_BURST_FILL_VALID <= 1'b0;
                DCACHE_BURST_FILL_PENDING <= 4'h0;
                DCACHE_BURST_FILL_NEXT_ENTRY <= 2'd0;
            end
            if (cacr_write_value[10]) begin
                DCACHE_VALID[caar_line][caar_entry] <= 1'b0;
                DCACHE_BURST_TRACK_VALID <= 1'b0;
                DCACHE_BURST_FILL_VALID <= 1'b0;
                DCACHE_BURST_FILL_PENDING <= 4'h0;
                DCACHE_BURST_FILL_NEXT_ENTRY <= 2'd0;
            end

            if (cacr_write_value[3]) begin
                for (i = 0; i < ICACHE_LINES; i = i + 1)
                    ICACHE_VALID[i] <= 8'h00;
                ICACHE_HIT_PENDING <= 1'b0;
                ICACHE_BURST_TRACK_VALID <= 1'b0;
                ICACHE_BURST_FILL_VALID <= 1'b0;
                ICACHE_BURST_FILL_PENDING <= 8'h00;
                ICACHE_BURST_FILL_NEXT_WORD <= 3'd0;
            end
            if (cacr_write_value[2]) begin
                ICACHE_VALID[caar_line][{caar_entry, 1'b0}] <= 1'b0;
                ICACHE_VALID[caar_line][{caar_entry, 1'b1}] <= 1'b0;
                ICACHE_HIT_PENDING <= 1'b0;
                ICACHE_BURST_TRACK_VALID <= 1'b0;
                ICACHE_BURST_FILL_VALID <= 1'b0;
                ICACHE_BURST_FILL_PENDING <= 8'h00;
                ICACHE_BURST_FILL_NEXT_WORD <= 3'd0;
            end

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

endmodule
