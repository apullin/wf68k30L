// ========================================================================
// Exception handler data multiplexer
// ========================================================================

assign DATA_EXH = (STACK_POS == 2) ? {SR_CPY, PC[31:16]} :
                   (STACK_POS == 4) ? {PC[15:0], STACK_FORMAT, 2'b00, IVECT_OFFS} :
                   (STACK_FORMAT == 4'h2 && STACK_POS == 6) ? PC :
                   (STACK_FORMAT == 4'h9 && STACK_POS == 6) ? PC :
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

