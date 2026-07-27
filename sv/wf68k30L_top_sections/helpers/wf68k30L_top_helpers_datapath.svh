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
// PC_NEXT_WB is the same capture one instruction on: PC_OFFSET is the byte
// length PC_INC will add for the instruction currently in the pipe, and by the
// time its effective address is marked used every extension word has been
// consumed, so PC + PC_OFFSET already names the following instruction. That is
// the continuation point after a faulted store has been completed by replaying
// its bus cycle - the store itself has nothing left to do. The live PC cannot
// serve: it names the following instruction only when the fault happened to
// arrive after that instruction was dispatched, which depends on how many
// extension words the store had (measured: 0x10E for MOVE.L D3,(A0)+ at 0x10C,
// but 0x106 for MOVE.L D3,($E0000).L at 0x106).
always_ff @(posedge CLK) begin : writeback_stage_pc
    if (RESET_CPU) begin
        PC_WB      <= 32'h0;
        PC_NEXT_WB <= 32'h0;
    end else if (ADR_MARK_USED && !BUSY_EXH) begin
        PC_WB      <= PC;
        PC_NEXT_WB <= PC + {24'h0, PC_OFFSET};
    end
end

// The data output buffer and replay eligibility of the writeback stage.
//
// WP_BUFFER in the bus interface - and therefore OUTBUFFER, the frame's $18
// field - is only loaded when a bus cycle actually starts, so an MMU-refused
// write leaves it describing whichever cycle ran last. Capturing the core's
// write data here covers that case; see DOB_STACKED below.
//
// WB_REPLAYABLE says the write is the last memory transfer of its instruction
// and that the instruction has nothing left to do afterwards, which is what
// makes "replay the cycle, then continue at PC_NEXT_WB" equivalent to UM
// 8.2.3's "rerun the faulted data cycle and continue the suspended
// instruction". Excluded are the multi-transfer operations (MOVEM, MOVEP and
// the bitfield writes, which can straddle two long words), the read-modify-write
// operations (UM 8.2.3: with RM set the rerun "reruns the entire instruction"),
// and the operations that act on the PC or an address register after their push
// (JSR, BSR, LINK).
always_ff @(posedge CLK) begin : writeback_stage_data
    if (RESET_CPU) begin
        DOB_WB        <= 32'h0;
        WB_REPLAYABLE <= 1'b0;
    end else if (DATA_WR_MAIN && !BUSY_EXH) begin
        DOB_WB        <= DATA_FROM_CORE;
        WB_REPLAYABLE <= !(OP_WB == MOVEM  || OP_WB == MOVEP  ||
                           OP_WB == CAS    || OP_WB == CAS2   || OP_WB == TAS ||
                           OP_WB == JSR    || OP_WB == BSR    || OP_WB == LINK ||
                           OP_WB == BFINS  || OP_WB == BFSET  || OP_WB == BFCLR ||
                           OP_WB == BFCHG);
    end
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
//
// The fault address field has the same entry-timing dependence and is frozen
// alongside: ADR_CPY snapshots the live ADR_EFF at entry, which for a
// postincrement store has already advanced past the address the write used --
// MOVE.L D3,(A0)+ faulting at $E0000 stacked $E0004. ADR_EFF_WB is the address
// the cycle actually drove, since ADR_L selects it for every core write.
always_ff @(posedge CLK) begin : bus_fault_pc
    if (RESET_CPU) begin
        PC_BF         <= 32'h0;
        ADR_BF        <= 32'h0;
        BF_IS_WRITE   <= 1'b0;
        PC_BF_FROZEN  <= 1'b0;
        PC_CONT_BF    <= 32'h0;
        DOB_BF        <= 32'h0;
        BF_REPLAY_WR  <= 1'b0;
    end else if (!BUSY_EXH && (BERR_MAIN || !PC_BF_FROZEN)) begin
        PC_BF         <= (BERR_MAIN && DATA_WR_PENDING) ? PC_WB : PC;
        ADR_BF        <= ADR_EFF_WB;
        BF_IS_WRITE   <= BERR_MAIN && DATA_WR_PENDING;
        PC_BF_FROZEN  <= BERR_MAIN;
        PC_CONT_BF    <= PC_NEXT_WB;
        DOB_BF        <= DOB_WB;
        BF_REPLAY_WR  <= BERR_MAIN && DATA_WR_PENDING && WB_REPLAYABLE;
    end else if (BUSY_EXH) begin
        PC_BF_FROZEN <= 1'b0;
    end
end

// Format $A/$B name the instruction that owned the faulted access. Every other
// format keeps the live PC, so the format $0/$2 stacked PC is unchanged.
assign PC_STACKED = (STACK_FORMAT == 4'hA || STACK_FORMAT == 4'hB) ? PC_BF : PC;

// Only formats $A and $B reach the STACK_POS 10 arm below (format $0/$2 frames
// are too short and format $9 has its own arm), so this needs no format guard.
assign ADR_STACKED = BF_IS_WRITE ? ADR_BF : ADR_CPY_EXH;

// UM Figure 8-9 marks SSW bits 11-9 and 3 "X = for internal use only", and UM
// 8.2.2 forbids a handler from altering them: "The only bits in the SSW that may
// be modified are DF, RB, and RC; all other bits, including those defined for
// internal use, must remain unchanged." Bit 3 therefore carries the one piece of
// internal state RTE needs to complete a faulted write by rerunning just the bus
// cycle - whether that is equivalent to continuing the suspended instruction.
// Frames a handler builds by hand leave it clear, so they keep the whole-
// instruction rerun they get today.
assign SSW_LOW_STACKED = (MMU_FAULT_SSW_VALID ? MMU_FAULT_SSW : SSW_80) |
                         {5'b0, BF_REPLAY_WR, 3'b000};

// The $18 field must be the data the faulted cycle was carrying. OUTBUFFER is
// that for a cycle that reached the bus; for an MMU-refused write no cycle ever
// started, so the core-side capture is used instead - the same split the SSW
// already makes just above.
assign DOB_STACKED = MMU_FAULT_SSW_VALID ? DOB_BF : OUTBUFFER;

assign DATA_EXH = (RTE_RERUN_WR) ? RTE_RERUN_DATA : // Replayed data write.
                   (STACK_POS == 2) ? {SR_CPY, PC_STACKED[31:16]} :
                   (STACK_POS == 4) ? {PC_STACKED[15:0], STACK_FORMAT, 2'b00, IVECT_OFFS} :
                   (STACK_FORMAT == 4'h2 && STACK_POS == 6) ? PC_INSTR_EXH :
                   (STACK_FORMAT == 4'h9 && STACK_POS == 6) ? PC_INSTR_EXH :
                   // An MMU translation fault runs no bus cycle, so the bus
                   // controller has no fault information for it and SSW_80 still
                   // describes whichever cycle ran last. The MMU supplies the
                   // word for its own faults instead (see mmu_fault_ssw_capture).
                   (STACK_POS == 6) ? {BIW_0, FC, FB, RC, RB, 3'b000,
                                       SSW_LOW_STACKED} : // Format A and B.
                   (STACK_POS == 8) ? {BIW_1, BIW_2} : // Format A and B.
                   (STACK_FORMAT == 4'h9 && STACK_POS == 10) ? FAULT_ADR :
                   (STACK_POS == 10) ? ADR_STACKED :
                   // Internal register, present in both format $A and format $B.
                   (STACK_POS == 12) ? PC_CONT_BF :
                   (STACK_POS == 14) ? DOB_STACKED :
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

