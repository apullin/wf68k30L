// ========================================================================
// Exception handler data multiplexer
// ========================================================================

// Table 8-6 distinguishes the stacked PC (next instruction) from the format $2
// instruction address (the faulting instruction). PC advances only via the
// handler's PC_INC, which fires after BUSY_EXH is already asserted, so the
// pre-entry value is the faulting instruction's address.
always_ff @(posedge CLK) begin
    if (!BUSY_EXH)
        PC_INSTR_EXH <= PC;
end

// ========================================================================
// Writeback-stage program counter (format $A/$B bus-fault frames)
// ========================================================================

// UM 8.1.2 requires a bus-fault frame to stack "the logical address of the
// instruction that was executing" when the fault occurred. PC alone cannot
// supply it for a store: PC advances at dispatch (PC_EW_BASE needs it there),
// and a short store issues its write one cycle *after* the following
// instruction has been dispatched, so by the time the fault arrives PC already
// names that following instruction. Since RTE suppresses the pipeline reload
// whenever the SSW asks for a rerun, a frame built from the live PC resumes
// past the store and the faulting write is discarded outright.
//
// PC_WB carries the owning instruction's PC alongside the writeback stage, in
// step with the writeback-stage effective address ADR_EFF_WB: it is captured on
// ADR_MARK_USED, the same event that loads ADR_WB in address_registers.
// ADR_MARK_USED is asserted only for memory-destination operations, and always
// while PC still names the operation itself, so any core data write that
// reaches the bus has already deposited its own instruction address here.
always_ff @(posedge CLK) begin : writeback_stage_pc
    if (ADR_MARK_USED && !BUSY_EXH)
        PC_WB <= PC;
end

// The faulted access is a core data write when a write request was outstanding
// in the cycle before the fault: DATA_WR_MAIN drops on the same edge the failed
// acknowledge arrives, so it has to be sampled one cycle late. This also covers
// MMU-refused writes, which never reach the bus and so leave the SSW untouched.
always_ff @(posedge CLK) begin : data_write_pending
    if (RESET_CPU)
        DATA_WR_PENDING <= 1'b0;
    else
        DATA_WR_PENDING <= DATA_WR_MAIN && !BUSY_EXH;
end

// Freeze the bus-fault instruction PC the way the bus interface freezes the SSW
// (SSW_FROZEN): the handler can take several clocks to leave EXS_IDLE, and PC
// must not drift in the meantime. A fresh fault always re-arms the value so a
// stale freeze cannot pin an old PC. Only a faulted write is redirected to
// PC_WB; reads, opcode fetches and address errors keep the live PC they stack
// today.
always_ff @(posedge CLK) begin : bus_fault_pc
    if (RESET_CPU) begin
        PC_BF        <= 32'h0;
        PC_BF_FROZEN <= 1'b0;
    end else if (!BUSY_EXH && (BERR_MAIN || !PC_BF_FROZEN)) begin
        PC_BF        <= (BERR_MAIN && DATA_WR_PENDING) ? PC_WB : PC;
        PC_BF_FROZEN <= BERR_MAIN;
    end else if (BUSY_EXH) begin
        PC_BF_FROZEN <= 1'b0;
    end
end

// Format $A/$B name the instruction that owned the faulted access. Every other
// format keeps the live PC, so the format $0/$2 stacked PC is unchanged.
assign PC_STACKED = (STACK_FORMAT == 4'hA || STACK_FORMAT == 4'hB) ? PC_BF : PC;

assign DATA_EXH = (STACK_POS == 2) ? {SR_CPY, PC_STACKED[31:16]} :
                   (STACK_POS == 4) ? {PC_STACKED[15:0], STACK_FORMAT, 2'b00, IVECT_OFFS} :
                   (STACK_FORMAT == 4'h2 && STACK_POS == 6) ? PC_INSTR_EXH :
                   (STACK_FORMAT == 4'h9 && STACK_POS == 6) ? PC_INSTR_EXH :
                   (STACK_POS == 6) ? {BIW_0, FC, FB, RC, RB, 3'b000, SSW_80} : // Format A and B.
                   (STACK_POS == 8) ? {BIW_1, BIW_2} : // Format A and B.
                   (STACK_FORMAT == 4'h9 && STACK_POS == 10) ? FAULT_ADR :
                   (STACK_POS == 10) ? ADR_CPY_EXH :
                   (STACK_POS == 14) ? OUTBUFFER :
                   (STACK_POS == 20) ? PC + 32'd4 : // Stage B address.
                   (STACK_POS == 24) ? INBUFFER :
                   (STACK_POS == 28) ? {16'h0, VERSION} : 32'h0;

// ========================================================================
// Core data path routing
// ========================================================================

assign DATA_IN_EXH = BUSY_MAIN ? ALU_RESULT[31:0] : DATA_TO_CORE; // MOVEC handles the VBR.

assign DATA_FROM_CORE = BUSY_EXH ? DATA_EXH :
                         (OP_WB == CAS || OP_WB == CAS2) ? DR_OUT_2 :
                         ALU_RESULT[31:0];

