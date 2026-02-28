// ========================================================================
// Miscellaneous signal multiplexing
// ========================================================================

assign DISPLACEMENT = BUSY_MAIN ? DISPLACEMENT_MAIN : {24'h0, DISPLACEMENT_EXH};
assign SR_WR = SR_WR_EXH || SR_WR_MAIN;
assign IPIPE_FLUSH = IPIPE_FLUSH_EXH || IPIPE_FLUSH_MAIN;
assign ISP_WR = ISP_WR_MAIN || ISP_LOAD_EXH;
assign AVECn_BUSIF = BUSY_EXH ? AVECn : 1'b1;

assign CPU_SPACE = (OP == BKPT && DATA_RD_MAIN) ? 1'b1 :
                    BUSY_EXH ? CPU_SPACE_EXH : 1'b0;


// ========================================================================
// Trap and function code signals
// ========================================================================

assign TRAP_AERR = !BUSY_EXH ? AERR : 1'b0; // No address error from the system during exception processing.
assign USE_DFC = (OP_WB == MOVES && DATA_WR_MAIN);
assign USE_SFC = (OP_WB == MOVES && DATA_RD_MAIN);

// ========================================================================
// Input synchronization
// ========================================================================

assign RESET_IN = ~RESET_INn;
assign IPL = ~IPLn;

// ========================================================================
// Status output (active on negedge CLK)
// ========================================================================

always_ff @(negedge CLK) begin : refill_status
    STATUSn <= !(STATUSn_EXH && STATUSn_MAIN) ? 1'b0 : 1'b1;
    REFILLn <= REFILLn_EXH;
end

// ========================================================================
// Status register and address bus
// ========================================================================

assign SBIT = STATUS_REG[13];

assign ADR_L = BKPT_CYCLE ? {24'h0, 3'b000, BIW_0[2:0], 2'b00} :
                CPU_SPACE_EXH ? {28'hFFFFFFF, IRQ_PEND, 1'b1} :
                DATA_WR_MAIN ? ADR_EFF_WB : ADR_EFF;

assign ADR_P = BUS_BSY ? ADR_LATCH :
                (DATA_RD || DATA_WR) ? ADR_L : PC_L;
