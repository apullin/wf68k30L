//--------------------------------------------------------------------//
//                                                                    //
// WF68K30L IP Core: MOVEM, MOVEP, and bitfield/address offset        //
// control logic.                                                     //
//                                                                    //
// This module contains the clocked and combinational logic for       //
// MOVEM register scanning, MOVEP byte pointer, bitfield control,    //
// and address offset computation. Extracted from                     //
// wf68k30L_control.sv for maintainability.                           //
//                                                                    //
//--------------------------------------------------------------------//

module WF68K30L_CTRL_MOVEM (
    input  logic        CLK,
    input  logic        RESET_CPU,

    // Operation
    input  logic [6:0]  OP,
    input  logic [13:0] BIW_0,
    input  logic [15:0] BIW_1,

    // State
    input  logic [4:0]  FETCH_STATE,
    input  logic [4:0]  NEXT_FETCH_STATE,
    input  logic [2:0]  EXEC_WB_STATE,

    // Internal signals
    input  logic [2:0]  ADR_MODE_I,
    input  logic [1:0]  OP_SIZE_I,
    input  logic        PHASE2,
    input  logic        ALU_BSY,
    input  logic        ALU_INIT_I,
    input  logic        RD_RDY,
    input  logic        WR_RDY,
    input  logic        INIT_ENTRY,

    // Bitfield inputs
    input  logic [2:0]  BF_OFFSET,
    input  logic [5:0]  BF_WIDTH,

    // MOVEM outputs
    output logic        MOVEM_ADn,
    output logic        MOVEM_ADn_I,
    output logic        MOVEM_COND,
    output logic        MOVEM_FIRST_RD,
    output logic        MOVEM_INH_WR,
    output logic        MOVEM_LAST_WR,
    output logic [3:0]  MOVEM_PNTR,

    // MOVEM: effective address storage
    output logic        STORE_AEFF,

    // MOVEP outputs
    output int          MOVEP_PNTR,
    output int          MOVEP_PNTR_I,

    // Bitfield outputs
    output int          BF_BYTES,
    output logic        BF_HILOn,

    // Address offset output
    output logic [5:0]  ADR_OFFSET
);

`include "wf68k30L_pkg.svh"

// Local state type parameters matching the parent module
localparam logic [4:0]
    START_OP       = 5'd0,
    CALC_AEFF      = 5'd1,
    FETCH_EXWORD_1 = 5'd3,
    FETCH_OPERAND  = 5'd13,
    INIT_EXEC_WB   = 5'd14,
    SWITCH_STATE   = 5'd16;

localparam logic [2:0]
    WRITE_DEST     = 3'd4;

// Internal signals
logic [3:0] MOVEM_PVAR_S;
int         BF_OFFSET_I;
int         BF_WIDTH_I;

// BF_OFFSET_I, BF_WIDTH_I concurrent assigns
assign BF_OFFSET_I = BF_OFFSET;
assign BF_WIDTH_I = BF_WIDTH;

// ADDRESS_OFFSET process (clocked)
always_ff @(posedge CLK) begin
    logic [5:0] ADR_OFFS_VAR;
    if (FETCH_STATE == START_OP) begin
        ADR_OFFS_VAR = 6'b000000;
    end else begin
        case (OP)
            BFCHG, BFCLR, BFINS, BFSET: begin
                if (FETCH_STATE == FETCH_OPERAND && RD_RDY && BF_BYTES == 5) begin
                    ADR_OFFS_VAR = ADR_OFFS_VAR + 6'b000100; // Another Byte required.
                end else if (INIT_ENTRY) begin
                    ADR_OFFS_VAR = 6'b000000; // Restore.
                end else if (FETCH_STATE == INIT_EXEC_WB && !ALU_BSY && BF_BYTES == 5) begin
                    ADR_OFFS_VAR = ADR_OFFS_VAR + 6'b000100; // Another Byte required.
                end
            end
            BFEXTS, BFEXTU, BFFFO, BFTST: begin
                if (FETCH_STATE == FETCH_OPERAND && RD_RDY && BF_BYTES == 5) begin
                    ADR_OFFS_VAR = ADR_OFFS_VAR + 6'b000100; // Another Byte required.
                end
            end
            CHK2, CMP2: begin
                if (FETCH_STATE == FETCH_OPERAND && RD_RDY && OP_SIZE_I == LONG) begin
                    ADR_OFFS_VAR = ADR_OFFS_VAR + 6'b000100;
                end else if (FETCH_STATE == FETCH_OPERAND && RD_RDY && OP_SIZE_I == WORD) begin
                    ADR_OFFS_VAR = ADR_OFFS_VAR + 6'b000010;
                end else if (FETCH_STATE == FETCH_OPERAND && RD_RDY) begin
                    ADR_OFFS_VAR = ADR_OFFS_VAR + 6'b000001;
                end
            end
            MOVEM: begin
                if (ADR_MODE_I == 3'b011 || ADR_MODE_I == 3'b100) begin // (An)+, -(An).
                    // null -- Offset comes from addressing register.
                end else if (BIW_0[10] && !MOVEM_FIRST_RD) begin
                    // null -- Do not increment before the first bus access.
                end else if (MOVEM_COND && FETCH_STATE == INIT_EXEC_WB && !ALU_BSY && OP_SIZE_I == LONG) begin
                    ADR_OFFS_VAR = ADR_OFFS_VAR + 6'b000100; // Register to memory.
                end else if (MOVEM_COND && FETCH_STATE == INIT_EXEC_WB && !ALU_BSY) begin
                    ADR_OFFS_VAR = ADR_OFFS_VAR + 6'b000010; // Register to memory.
                end
            end
            MOVEP: begin
                if (FETCH_STATE == INIT_EXEC_WB && !ALU_BSY) begin
                    ADR_OFFS_VAR = ADR_OFFS_VAR + 6'b000010;
                end
            end
            PMOVE: begin
                if (BIW_1[15:13] == 3'b010 && BIW_1[12:10] >= 3'b010 && BIW_1[12:10] <= 3'b011) begin
                    // PMOVE SRP/CRP transfers are quadword and use two longword accesses.
                    if (!BIW_1[9] && FETCH_STATE == FETCH_OPERAND && RD_RDY && !PHASE2) begin
                        ADR_OFFS_VAR = ADR_OFFS_VAR + 6'b000100; // Second source longword.
                    end else if (BIW_1[9] && FETCH_STATE == INIT_EXEC_WB && !ALU_BSY && !PHASE2) begin
                        ADR_OFFS_VAR = ADR_OFFS_VAR + 6'b000100; // Second destination longword.
                    end
                end
            end
            default: begin
                // null
            end
        endcase
    end
    ADR_OFFSET <= ADR_OFFS_VAR;
end

// BITFIELD_CONTROL process (clocked)
always_ff @(posedge CLK) begin
    if (FETCH_STATE == START_OP) begin
        case (OP)
            BFCHG, BFCLR, BFEXTS, BFEXTU, BFFFO, BFINS, BFSET, BFTST: begin
                if (NEXT_FETCH_STATE == INIT_EXEC_WB) begin
                    BF_BYTES <= 4; // Register access.
                end else begin
                    BF_BYTES <= (BF_OFFSET_I + BF_WIDTH_I + 7) >> 3;
                end
                BF_HILOn <= 1'b1;
            end
            default: begin
                // null
            end
        endcase
    end else if (FETCH_STATE == FETCH_OPERAND) begin
        if (RD_RDY && BF_BYTES == 5) begin
            BF_BYTES <= 1;
            BF_HILOn <= 1'b0;
        end else if (RD_RDY) begin
            BF_BYTES <= (BF_OFFSET_I + BF_WIDTH_I + 7) >> 3; // Restore.
            BF_HILOn <= 1'b1; // Restore.
        end
    end else if (EXEC_WB_STATE == WRITE_DEST) begin
        if (WR_RDY && BF_BYTES == 5) begin
            BF_BYTES <= 1;
            BF_HILOn <= 1'b0;
        end else if (WR_RDY) begin
            BF_HILOn <= 1'b1; // Restore.
        end
    end
end

// MOVEM_CONTROL process (clocked)
always_ff @(posedge CLK) begin
    logic [3:0] MOVEM_PVAR;
    logic [4:0] BITS;
    if (FETCH_STATE == START_OP) begin
        MOVEM_PVAR = 4'h0;
    end else if (FETCH_STATE == INIT_EXEC_WB && !MOVEM_COND && MOVEM_PVAR < 4'hF && !ALU_BSY) begin
        MOVEM_PVAR = MOVEM_PVAR + 4'd1; // No data to write.
    end else if (BIW_0[10] && MOVEM_FIRST_RD && FETCH_STATE == INIT_EXEC_WB && MOVEM_PVAR < 4'hF && !ALU_BSY) begin
        MOVEM_PVAR = MOVEM_PVAR + 4'd1; // Data has not been read.
    end else if (!BIW_0[10] && FETCH_STATE == INIT_EXEC_WB && MOVEM_PVAR < 4'hF && !ALU_BSY) begin
        MOVEM_PVAR = MOVEM_PVAR + 4'd1; // Data has been written.
    end

    if (OP == MOVEM && ALU_INIT_I && ADR_MODE_I == 3'b011 && MOVEM_ADn_I && MOVEM_PNTR[2:0] == BIW_0[2:0]) begin
        MOVEM_INH_WR <= 1'b1; // Do not write the addressing register.
    end else if (ALU_INIT_I) begin
        MOVEM_INH_WR <= 1'b0;
    end

    if (FETCH_STATE == START_OP) begin
        MOVEM_FIRST_RD <= 1'b0;
    end else if (OP == MOVEM && FETCH_STATE == FETCH_OPERAND && RD_RDY) begin
        MOVEM_FIRST_RD <= 1'b1;
    end

    if (RESET_CPU || (FETCH_STATE != START_OP && NEXT_FETCH_STATE == START_OP)) begin
        BITS = 5'b00000;
        MOVEM_LAST_WR <= 1'b0;
    end else if (OP == MOVEM && FETCH_STATE == START_OP && NEXT_FETCH_STATE != START_OP && ADR_MODE_I == 3'b100) begin // -(An).
        for (int i = 0; i <= 15; i++) begin
            BITS = BITS + {4'b0, BIW_1[i]}; // Count number of '1's.
        end
        MOVEM_LAST_WR <= 1'b0;
    end else if (OP == MOVEM && ALU_INIT_I && BITS > 5'b00001) begin
        BITS = BITS - 5'd1;
    end else if (OP == MOVEM && BITS == 5'b00001) begin
        MOVEM_LAST_WR <= 1'b1;
    end

    // During the MOVEM instruction in memory to register operation and addressing modes "010", "101","110" the effective address might be
    // affected, if the addressing register is active in the register list mask. To deal with it, the effective address is stored until the
    // MOVEM has read all registers from memory addressed by the initial addressing register (old value).
    // This logic is modeled synchronously (one clock latency) due to the one clock delay of the address calculation.
    if (OP != MOVEM || BIW_0[10] != 1'b1 || (ADR_MODE_I != 3'b010 && ADR_MODE_I != 3'b101 && ADR_MODE_I != 3'b110)) begin
        STORE_AEFF <= 1'b0;
    end else if (FETCH_STATE != START_OP && NEXT_FETCH_STATE == START_OP) begin
        STORE_AEFF <= 1'b0; // Operation completed.
    end else if (FETCH_STATE == SWITCH_STATE || FETCH_STATE == CALC_AEFF || FETCH_STATE == FETCH_OPERAND || FETCH_STATE == INIT_EXEC_WB) begin
        STORE_AEFF <= 1'b1;
    end

    MOVEM_PVAR_S <= MOVEM_PVAR;
end

// MOVEM_COMB process (combinational)
always_comb begin
    logic [3:0] INDEX;
    // This signal determines whether to handle address or data registers.
    if (ADR_MODE_I == 3'b100) begin // -(An).
        MOVEM_ADn_I = ~MOVEM_PVAR_S[3];
        MOVEM_ADn = ~MOVEM_PVAR_S[3];
    end else begin
        MOVEM_ADn_I = MOVEM_PVAR_S[3];
        MOVEM_ADn = MOVEM_PVAR_S[3];
    end

    INDEX = MOVEM_PVAR_S;

    // The following signal determines if a register is affected or not, depending
    // on the status of the register list bit.
    if (OP == MOVEM && BIW_1[INDEX]) begin
        MOVEM_COND = 1'b1;
    end else begin
        MOVEM_COND = 1'b0;
    end

    // This signal determines whether to handle address or data registers.
    if (ADR_MODE_I == 3'b100) begin // -(An).
        MOVEM_PNTR = ~MOVEM_PVAR_S; // Count down.
    end else begin
        MOVEM_PNTR = MOVEM_PVAR_S;
    end
end

// MOVEP_CONTROL process (clocked part)
// This logic handles the bytes to be written or read during the MOVEP
// operation. In LONG mode 4 bytes are affected and in WORD mode two bytes.
always_ff @(posedge CLK) begin
    if (RESET_CPU || (FETCH_STATE != START_OP && NEXT_FETCH_STATE == START_OP)) begin
        MOVEP_PNTR_I <= 0;
    end else if (FETCH_STATE == START_OP && (BIW_0[8:6] == 3'b101 || BIW_0[8:6] == 3'b111)) begin
        MOVEP_PNTR_I <= 3; // LONG.
    end else if (FETCH_STATE == START_OP) begin
        MOVEP_PNTR_I <= 1; // WORD.
    end else if (FETCH_STATE == INIT_EXEC_WB && !ALU_BSY && MOVEP_PNTR_I != 0) begin
        MOVEP_PNTR_I <= MOVEP_PNTR_I - 1; // Register to memory
    end
end
// Concurrent assign from MOVEP_CONTROL process
assign MOVEP_PNTR = MOVEP_PNTR_I;

endmodule
