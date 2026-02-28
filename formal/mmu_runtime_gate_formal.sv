module mmu_runtime_gate_formal;
    logic data_rd_bus, data_wr, opcode_req_core_miss;
    logic mmu_runtime_fault, mmu_runtime_stall;
    logic burst_prefetch_data_req, burst_prefetch_op_req;

    logic rd_req_core;
    logic wr_req_core;
    logic opcode_req_core;
    logic rd_req;
    logic wr_req;
    logic opcode_req;

    always_comb begin
        data_rd_bus = $anyseq;
        data_wr = $anyseq;
        opcode_req_core_miss = $anyseq;
        mmu_runtime_fault = $anyseq;
        mmu_runtime_stall = $anyseq;
        burst_prefetch_data_req = $anyseq;
        burst_prefetch_op_req = $anyseq;
    end

    always_comb begin
        rd_req_core = data_rd_bus && !mmu_runtime_fault && !mmu_runtime_stall;
        wr_req_core = data_wr && !mmu_runtime_fault && !mmu_runtime_stall;
        opcode_req_core = opcode_req_core_miss && !mmu_runtime_fault && !mmu_runtime_stall;

        rd_req = rd_req_core || burst_prefetch_data_req;
        wr_req = wr_req_core;
        opcode_req = opcode_req_core || burst_prefetch_op_req;
    end

    always_ff @($global_clock) begin
        // Core-originated requests must be suppressed whenever MMU indicates
        // a fault or a table-search stall window.
        assert(!(mmu_runtime_fault && (rd_req_core || wr_req_core || opcode_req_core)));
        assert(!(mmu_runtime_stall && (rd_req_core || wr_req_core || opcode_req_core)));

        // During stall, only background burst-prefetch traffic may request bus cycles.
        if (mmu_runtime_stall) begin
            assert(rd_req == burst_prefetch_data_req);
            assert(wr_req == 1'b0);
            assert(opcode_req == burst_prefetch_op_req);
        end
    end
endmodule
