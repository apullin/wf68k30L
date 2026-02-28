// ========================================================================
// Operand size routing
// ========================================================================

assign OP_SIZE = BUSY_EXH ? OP_SIZE_EXH : OP_SIZE_MAIN;
assign OP_SIZE_BUS = BURST_PREFETCH_DATA_REQ ? LONG :
                     DATA_WR_MAIN ? OP_SIZE_WB : OP_SIZE;

// ========================================================================
// PC-related signals
// ========================================================================

assign PC_OFFSET = PC_OFFSET_OPD;
assign PC_L = PC + PC_ADR_OFFSET;
assign PC_INC_EXH_I = !LOOP_SPLIT ? PC_INC_EXH : 1'b0; // Suppress for a split loop.
assign PC_LOAD = PC_LOAD_EXH || PC_LOAD_MAIN;

// ========================================================================
// Address path control
// ========================================================================

assign ADR_MODE = BUSY_EXH ? 3'b010 : ADR_MODE_MAIN; // (ISP) during exception.
assign SP_ADD_DISPL = SP_ADD_DISPL_MAIN || SP_ADD_DISPL_EXH;
assign AR_SEL_RD_1 = BUSY_EXH ? 3'b111 : AR_SEL_RD_1_MAIN; // ISP during exception.
