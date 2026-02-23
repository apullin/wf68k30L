// ------------------------------------------------------------------------
// -- WF68K30L IP Core: ALU (Arithmetic Logic Unit)                      --
// -- Author(s): Wolfgang Foerster, wf@experiment-s.de                   --
// -- Copyright (c) 2014-2019 Wolfgang Foerster Inventronik GmbH.        --
// -- CERN OHL v. 1.2                                                    --
// ------------------------------------------------------------------------

module WF68K30L_ALU (
    input  logic        CLK,
    input  logic        RESET,

    input  logic        LOAD_OP1,
    input  logic        LOAD_OP2,
    input  logic        LOAD_OP3,

    input  logic [31:0] OP1_IN,
    input  logic [31:0] OP2_IN,
    input  logic [31:0] OP3_IN,

    input  logic [31:0] BF_OFFSET_IN,
    input  logic [5:0]  BF_WIDTH_IN,
    input  logic [4:0]  BITPOS_IN,

    output logic [63:0] RESULT,

    input  logic [2:0]  ADR_MODE_IN,
    input  logic [1:0]  OP_SIZE_IN,
    input  logic [6:0]  OP_IN,
    input  logic [6:0]  OP_WB,
    input  logic [11:0] BIW_0_IN,
    input  logic [15:0] BIW_1_IN,

    // The Flags:
    input  logic        SR_WR,
    input  logic        SR_INIT,
    input  logic        SR_CLR_MBIT,
    input  logic        CC_UPDT,

    output logic [15:0] STATUS_REG_OUT,
    output logic        ALU_COND,

    // Status and Control:
    input  logic        ALU_INIT,
    output logic        ALU_BSY,
    output logic        ALU_REQ,
    input  logic        ALU_ACK,
    input  logic        USE_DREG,
    input  logic        HILOn,
    input  logic [2:0]  IRQ_PEND,
    output logic        TRAP_CHK,
    output logic        TRAP_DIVZERO
);

`include "wf68k30L_pkg.svh"

typedef enum logic [1:0] {DIV_IDLE, DIV_INIT, DIV_CALC} DIV_STATES;
typedef enum logic {SHIFT_IDLE, SHIFT_RUN} SHIFT_STATES;

logic        ALU_COND_I;
logic [2:0]  ADR_MODE;
logic [4:0]  BITPOS;
logic [39:0] BF_DATA_IN;
logic [5:0]  BF_LOWER_BND;
logic [31:0] BF_OFFSET;
logic [5:0]  BF_UPPER_BND;
logic [5:0]  BF_WIDTH;
logic [11:0] BIW_0;
logic [15:0] BIW_1;
logic        CAS2_COND;
logic        CB_BCD;
logic        CHK_CMP_COND;
logic        CHK2CMP2_DR;
logic        DIV_RDY;
DIV_STATES   DIV_STATE;
logic [4:0]  MSB;
OP_68K       OP;
logic [31:0] OP1;
logic [31:0] OP2;
logic [31:0] OP3;
logic [31:0] OP1_SIGNEXT;
logic [31:0] OP2_SIGNEXT;
logic [31:0] OP3_SIGNEXT;
OP_SIZETYPE  OP_SIZE;
logic [31:0] QUOTIENT;
logic [31:0] REMAINDER;
logic [7:0]  RESULT_BCDOP;
logic [39:0] RESULT_BITFIELD;
logic [31:0] RESULT_BITOP;
logic [31:0] RESULT_INTOP;
logic [31:0] RESULT_LOGOP;
logic [63:0] RESULT_MUL;
logic [31:0] RESULT_SHIFTOP;
logic [31:0] RESULT_OTHERS;
SHIFT_STATES SHIFT_STATE;
logic [5:0]  SHIFT_WIDTH;
logic [5:0]  SHIFT_WIDTH_IN_sig;
logic        SHFT_LOAD;
logic        SHFT_RDY;
logic        SHFT_EN;
logic [15:0] STATUS_REG;
logic        VFLAG_DIV;
logic        XFLAG_SHFT;
logic [4:0]  XNZVC;

always_ff @(posedge CLK) begin : parameter_buffer
    if (ALU_INIT) begin
        ADR_MODE <= ADR_MODE_IN;
        CHK2CMP2_DR <= USE_DREG;
        OP_SIZE <= OP_SIZE_IN;
        OP <= OP_IN;
        BIW_0 <= BIW_0_IN;
        BIW_1 <= BIW_1_IN;
        BF_OFFSET <= BF_OFFSET_IN;
        BITPOS <= {1'b0, BITPOS_IN[3:0]};
        BF_WIDTH <= BF_WIDTH_IN;
        BF_UPPER_BND <= 6'd39 - {1'b0, BITPOS_IN};
        SHIFT_WIDTH <= SHIFT_WIDTH_IN_sig;

        if ((BITPOS_IN + BF_WIDTH_IN) > 40)
            BF_LOWER_BND <= 6'd0;
        else
            BF_LOWER_BND <= 6'd40 - ({1'b0, BITPOS_IN} + BF_WIDTH_IN);
    end
end

always_ff @(posedge CLK) begin : operands
    // During instruction execution, the buffers are written
    // before or during ALU_INIT and copied to the operands
    // during ALU_INIT.
    logic [31:0] OP1_BUFFER;
    logic [31:0] OP2_BUFFER;
    logic [31:0] OP3_BUFFER;

    if (LOAD_OP1)
        OP1_BUFFER = OP1_IN;

    if (LOAD_OP2)
        OP2_BUFFER = OP2_IN;

    if (LOAD_OP3)
        OP3_BUFFER = OP3_IN;

    if (ALU_INIT) begin
        OP1 <= OP1_BUFFER;
        OP2 <= OP2_BUFFER;
        OP3 <= OP3_BUFFER;
    end
end

always_ff @(posedge CLK) begin : alu_busy
    if (ALU_INIT)
        ALU_BSY <= 1'b1;
    else if (ALU_ACK || RESET)
        ALU_BSY <= 1'b0;

    // This signal requests the control state machine to proceed when the ALU is ready.
    if (ALU_ACK)
        ALU_REQ <= 1'b0;
    else if ((OP == ASL || OP == ASR || OP == LSL || OP == LSR || OP == ROTL || OP == ROTR || OP == ROXL || OP == ROXR) && SHFT_RDY)
        ALU_REQ <= 1'b1;
    else if ((OP == DIVS || OP == DIVU) && DIV_RDY)
        ALU_REQ <= 1'b1;
    else if (OP_IN == DIVS || OP_IN == DIVU)
        ; // null
    else if (OP_IN == ASL || OP_IN == ASR || OP_IN == LSL || OP_IN == LSR)
        ; // null
    else if (OP_IN == ROTL || OP_IN == ROTR || OP_IN == ROXL || OP_IN == ROXR)
        ; // null
    else if (ALU_INIT)
        ALU_REQ <= 1'b1;
end

always_comb begin
    case (OP_SIZE)
        LONG:    MSB = 5'd31;
        WORD:    MSB = 5'd15;
        BYTE:    MSB = 5'd7;
        default: MSB = 5'd31;
    endcase
end

always_comb begin : sign_extend
    // This module provides the required sign extensions.
    case (OP_SIZE)
        LONG: begin
            OP1_SIGNEXT = OP1;
            OP2_SIGNEXT = OP2;
            OP3_SIGNEXT = OP3;
        end
        WORD: begin
            OP1_SIGNEXT = {{16{OP1[15]}}, OP1[15:0]};
            OP2_SIGNEXT = {{16{OP2[15]}}, OP2[15:0]};
            OP3_SIGNEXT = {{16{OP3[15]}}, OP3[15:0]};
        end
        BYTE: begin
            OP1_SIGNEXT = {{24{OP1[7]}}, OP1[7:0]};
            OP2_SIGNEXT = {{24{OP2[7]}}, OP2[7:0]};
            OP3_SIGNEXT = {{24{OP3[7]}}, OP3[7:0]};
        end
        default: begin
            OP1_SIGNEXT = OP1;
            OP2_SIGNEXT = OP2;
            OP3_SIGNEXT = OP3;
        end
    endcase
end

always_comb begin : bcd_op
    // The BCD operations are all byte wide and unsigned.
    logic [4:0] TEMP0;
    logic [4:0] TEMP1;
    logic [3:0] Z_0;
    logic       C_0;
    logic [3:0] Z_1;
    logic       C_1;
    logic [3:0] S_0;
    logic [3:0] S_1;
    logic       X_IN_I;

    X_IN_I = STATUS_REG[4]; // Extended Flag.

    case (OP)
        ABCD:
            TEMP0 = {1'b0, OP2[3:0]} + {1'b0, OP1[3:0]} + {4'b0000, X_IN_I};
        NBCD:
            TEMP0 = OP1[4:0] - {1'b0, OP2[3:0]} - {4'b0000, X_IN_I};
        default: // Valid for SBCD.
            TEMP0 = {1'b0, OP2[3:0]} - {1'b0, OP1[3:0]} - {4'b0000, X_IN_I};
    endcase

    if (TEMP0 > 5'b01001) begin
        Z_0 = 4'b0110;
        C_0 = 1'b1;
    end else begin
        Z_0 = 4'b0000;
        C_0 = 1'b0;
    end

    case (OP)
        ABCD:
            TEMP1 = {1'b0, OP2[7:4]} + {1'b0, OP1[7:4]} + {4'b0000, C_0};
        NBCD:
            TEMP1 = OP1[4:0] - {1'b0, OP2[7:4]} - {4'b0000, X_IN_I};
        default: // Valid for SBCD.
            TEMP1 = {1'b0, OP2[7:4]} - {1'b0, OP1[7:4]} - {4'b0000, C_0};
    endcase

    if (TEMP1 > 5'b01001) begin
        Z_1 = 4'b0110;
        C_1 = 1'b1;
    end else begin
        Z_1 = 4'b0000;
        C_1 = 1'b0;
    end

    case (OP)
        ABCD: begin
            S_1 = TEMP1[3:0] + Z_1;
            S_0 = TEMP0[3:0] + Z_0;
        end
        default: begin // Valid for SBCD, NBCD.
            S_1 = TEMP1[3:0] - Z_1;
            S_0 = TEMP0[3:0] - Z_0;
        end
    endcase
    //
    CB_BCD = C_1;
    RESULT_BCDOP[7:4] = S_1;
    RESULT_BCDOP[3:0] = S_0;
end

assign BF_DATA_IN = {OP3, OP2[7:0]};

always_comb begin : bitfield_op
    logic BF_NZ;
    logic [5:0] BFFFO_CNT;
    logic [39:0] bf_mask;
    logic [39:0] shifted_data;
    logic [31:0] width_mask;
    integer i;

    i = 0;
    bf_mask = ((40'd1 << (BF_UPPER_BND - BF_LOWER_BND + 6'd1)) - 40'd1) << BF_LOWER_BND;
    width_mask = (BF_WIDTH == 6'd32) ? 32'hFFFFFFFF : ((32'd1 << BF_WIDTH) - 32'd1);
    shifted_data = BF_DATA_IN >> BF_LOWER_BND;

    RESULT_BITFIELD = BF_DATA_IN; // Default.
    BF_NZ = 1'b0;
    BFFFO_CNT = 6'b000000;
    case (OP)
        BFCHG: begin
            RESULT_BITFIELD = BF_DATA_IN ^ bf_mask;
        end
        BFCLR: begin
            RESULT_BITFIELD = BF_DATA_IN & ~bf_mask;
        end
        BFEXTS: begin // Result is in (39 downto 8).
            RESULT_BITFIELD[7:0] = BF_DATA_IN[7:0];
            RESULT_BITFIELD[39:8] = (shifted_data[31:0] & width_mask) |
                                    ({32{BF_DATA_IN[BF_UPPER_BND]}} & ~width_mask);
        end
        BFEXTU: begin // Result is in (39 downto 8).
            RESULT_BITFIELD[7:0] = BF_DATA_IN[7:0];
            RESULT_BITFIELD[39:8] = shifted_data[31:0] & width_mask;
        end
        BFFFO: begin // Result is in (39 downto 8).
            // Count consecutive zeros from LSB of field upward
            for (i = 0; i < 40; i = i + 1) begin
                if (i <= BF_UPPER_BND && i >= BF_LOWER_BND) begin
                    if (!BF_DATA_IN[i] && !BF_NZ)
                        BFFFO_CNT = BFFFO_CNT + 1'b1;
                    else
                        BF_NZ = 1'b1;
                end
            end
            RESULT_BITFIELD = {(BF_OFFSET[31:0] + {26'd0, BFFFO_CNT}), 8'h00};
        end
        BFINS: begin
            RESULT_BITFIELD = (BF_DATA_IN & ~bf_mask) | (({8'h00, OP1} << BF_LOWER_BND) & bf_mask);
        end
        BFSET: begin
            RESULT_BITFIELD = BF_DATA_IN | bf_mask;
        end
        default: ; // BFTST. no calculation required for BFTST.
    endcase
end

always_comb begin : bit_op
    // Bit manipulation operations.
    RESULT_BITOP = OP2; // The default is the unmanipulated data.
    //
    case (OP)
        BCHG:    RESULT_BITOP[BITPOS] = ~OP2[BITPOS];
        BCLR:    RESULT_BITOP[BITPOS] = 1'b0;
        BSET:    RESULT_BITOP[BITPOS] = 1'b1;
        default: RESULT_BITOP = OP2; // Dummy, no result required for BTST.
    endcase
end

always_ff @(posedge CLK) begin : division
    logic [6:0] BITCNT;
    logic [63:0] DIVIDEND;
    logic [31:0] DIVISOR;
    logic [31:0] QUOTIENT_REST;
    logic [31:0] QUOTIENT_VAR;
    logic [31:0] REMAINDER_REST;
    logic [31:0] REMAINDER_VAR;

    DIV_RDY <= 1'b0;
    case (DIV_STATE)
        DIV_IDLE: begin
            if (ALU_INIT && (OP_IN == DIVS || OP_IN == DIVU))
                DIV_STATE <= DIV_INIT;
        end
        DIV_INIT: begin
            if (OP == DIVS && OP_SIZE == LONG && BIW_1[10] && OP3[31]) // 64 bit signed negative dividend.
                DIVIDEND = ~{OP3, OP2} + 1'b1;
            else if ((OP == DIVS || OP == DIVU) && OP_SIZE == LONG && BIW_1[10]) // 64 bit positive or unsigned dividend.
                DIVIDEND = {OP3, OP2};
            else if (OP == DIVS && OP2[31]) // 32 bit signed negative dividend.
                DIVIDEND = {32'h0, ~OP2 + 1'b1};
            else // 32 bit positive or unsigned dividend.
                DIVIDEND = {32'h0, OP2};

            if (OP == DIVS && OP_SIZE == LONG && OP1[31]) // 32 bit signed negative divisor.
                DIVISOR = ~OP1 + 1'b1;
            else if (OP_SIZE == LONG) // 32 bit positive or unsigned divisor.
                DIVISOR = OP1;
            else if (OP == DIVS && OP_SIZE == WORD && OP1[15]) // 16 bit signed negative divisor.
                DIVISOR = {16'h0, ~OP1[15:0] + 1'b1};
            else // 16 bit positive or unsigned divisor.
                DIVISOR = {16'h0, OP1[15:0]};

            VFLAG_DIV <= 1'b0;
            QUOTIENT <= 32'h0;
            QUOTIENT_VAR = 32'h0;
            QUOTIENT_REST = OP2;

            REMAINDER <= 32'h0;
            REMAINDER_VAR = 32'h0;

            case (OP_SIZE)
                LONG:    REMAINDER_REST = OP3;
                default: REMAINDER_REST = {16'h0, OP2[31:16]};
            endcase

            if (OP_SIZE == LONG && BIW_1[10])
                BITCNT = 7'd64;
            else
                BITCNT = 7'd32;

            if (DIVISOR == 32'h0) begin // Division by zero.
                QUOTIENT <= 32'hFFFFFFFF;
                REMAINDER <= 32'hFFFFFFFF;
                DIV_STATE <= DIV_IDLE;
                DIV_RDY <= 1'b1;
            end else if ({32'h0, DIVISOR} > DIVIDEND) begin // Divisor > dividend.
                REMAINDER <= DIVIDEND[31:0];
                DIV_STATE <= DIV_IDLE;
                DIV_RDY <= 1'b1;
            end else if ({32'h0, DIVISOR} == DIVIDEND) begin // Result is 1.
                QUOTIENT <= 32'h1;
                DIV_STATE <= DIV_IDLE;
                DIV_RDY <= 1'b1;
            end else begin
                DIV_STATE <= DIV_CALC;
            end
        end
        DIV_CALC: begin
            BITCNT = BITCNT - 1;

            if ({REMAINDER_VAR, DIVIDEND[BITCNT]} < {1'b0, DIVISOR}) begin
                REMAINDER_VAR = {REMAINDER_VAR[30:0], DIVIDEND[BITCNT]};
            end else if (OP_SIZE == LONG && BITCNT > 31) begin // Division overflow in 64 bit mode.
                VFLAG_DIV <= 1'b1;
                DIV_STATE <= DIV_IDLE;
                DIV_RDY <= 1'b1;
                QUOTIENT <= QUOTIENT_REST;
                REMAINDER <= REMAINDER_REST;
            end else if (OP_SIZE == WORD && BITCNT > 15) begin // Division overflow in word mode.
                VFLAG_DIV <= 1'b1;
                DIV_STATE <= DIV_IDLE;
                DIV_RDY <= 1'b1;
                QUOTIENT <= QUOTIENT_REST;
                REMAINDER <= REMAINDER_REST;
            end else begin
                REMAINDER_VAR = {REMAINDER_VAR[30:0], DIVIDEND[BITCNT]} - DIVISOR;
                QUOTIENT_VAR[BITCNT] = 1'b1;
            end

            if (BITCNT == 7'd0) begin
                // Adjust signs:
                if (OP == DIVS && OP_SIZE == LONG && BIW_1[10] && (OP3[31] ^ OP1[31]))
                    QUOTIENT <= ~QUOTIENT_VAR + 1'b1; // Negative, change sign.
                else if (OP == DIVS && OP_SIZE == LONG && !BIW_1[10] && (OP2[31] ^ OP1[31]))
                    QUOTIENT <= ~QUOTIENT_VAR + 1'b1; // Negative, change sign.
                else if (OP == DIVS && OP_SIZE == WORD && (OP2[31] ^ OP1[15]))
                    QUOTIENT <= ~QUOTIENT_VAR + 1'b1; // Negative, change sign.
                else
                    QUOTIENT <= QUOTIENT_VAR;

                REMAINDER <= REMAINDER_VAR;
                DIV_RDY <= 1'b1;
                DIV_STATE <= DIV_IDLE;
            end
        end
    endcase
end

always_comb begin : integer_op
    // The integer arithmetics ADD, SUB, NEG and CMP in their different variations are modelled here.
    logic [0:0] X_IN_I;
    logic [31:0] RESULT_tmp;

    X_IN_I[0] = STATUS_REG[4]; // Extended Flag.
    case (OP)
        ADDA: // No sign extension for the destination.
            RESULT_tmp = OP2 + OP1_SIGNEXT;
        ADDQ:
            case (ADR_MODE)
                3'b001:  RESULT_tmp = OP2 + OP1; // No sign extension for address destination.
                default: RESULT_tmp = OP2_SIGNEXT + OP1;
            endcase
        SUBQ:
            case (ADR_MODE)
                3'b001:  RESULT_tmp = OP2 - OP1; // No sign extension for address destination.
                default: RESULT_tmp = OP2_SIGNEXT - OP1;
            endcase
        ADD, ADDI:
            RESULT_tmp = OP2_SIGNEXT + OP1_SIGNEXT;
        ADDX:
            RESULT_tmp = OP2_SIGNEXT + OP1_SIGNEXT + {31'd0, X_IN_I};
        CMPA, DBcc, SUBA: // No sign extension for the destination.
            RESULT_tmp = OP2 - OP1_SIGNEXT;
        CAS, CAS2, CMP, CMPI, CMPM, SUB, SUBI:
            RESULT_tmp = OP2_SIGNEXT - OP1_SIGNEXT;
        SUBX:
            RESULT_tmp = OP2_SIGNEXT - OP1_SIGNEXT - {31'd0, X_IN_I};
        NEG:
            RESULT_tmp = OP1_SIGNEXT - OP2_SIGNEXT;
        NEGX:
            RESULT_tmp = OP1_SIGNEXT - OP2_SIGNEXT - {31'd0, X_IN_I};
        CLR:
            RESULT_tmp = 32'h0;
        default:
            RESULT_tmp = 32'h0; // Don't care.
    endcase
    RESULT_INTOP = RESULT_tmp;
end

always_comb begin : logic_op
    // This process provides the logic operations:
    // AND, OR, XOR and NOT.
    case (OP)
        AND_B, ANDI, ANDI_TO_CCR, ANDI_TO_SR:
            RESULT_LOGOP = OP1 & OP2;
        OR_B, ORI, ORI_TO_CCR, ORI_TO_SR:
            RESULT_LOGOP = OP1 | OP2;
        EOR, EORI, EORI_TO_CCR, EORI_TO_SR:
            RESULT_LOGOP = OP1 ^ OP2;
        default: // NOT_B.
            RESULT_LOGOP = ~OP2;
    endcase
end

assign RESULT_MUL = (OP == MULS) ? ($signed(OP1_SIGNEXT) * $signed(OP2_SIGNEXT)) :
                    (OP_SIZE == LONG) ? (OP1 * OP2) :
                    ({16'h0, OP1[15:0]} * {16'h0, OP2[15:0]});

always_comb begin : other_ops
    // This process provides the calculation for special operations.
    logic [31:0] RESULT_tmp;
    RESULT_tmp = 32'h0;
    case (OP)
        CAS:
            RESULT_tmp = OP2; // Destination operand.
        CAS2: // Destination operands.
            RESULT_tmp = HILOn ? OP3 : OP2;
        EXT:
            case (BIW_0[8:6])
                3'b011:  RESULT_tmp = {{16{OP2[15]}}, OP2[15:0]};
                default: RESULT_tmp = {OP2[31:16], {8{OP2[7]}}, OP2[7:0]}; // Word.
            endcase
        EXTB:
            RESULT_tmp = {{24{OP2[7]}}, OP2[7:0]};
        JSR:
            RESULT_tmp = OP1 + 32'd2; // Add offset of two to the Pointer of the last extension word.
        MOVEQ:
            RESULT_tmp = {{24{OP1[7]}}, OP1[7:0]};
        Scc:
            RESULT_tmp = ALU_COND_I ? 32'hFFFFFFFF : 32'h0;
        SWAP:
            RESULT_tmp = {OP2[15:0], OP2[31:16]};
        TAS:
            RESULT_tmp = {24'h0, 1'b1, OP2[6:0]}; // Set the MSB.
        PACK:
            RESULT_tmp = {16'h0, OP1[15:0] + OP2[15:0]};
        UNPK:
            RESULT_tmp = {16'h0, OP1[15:0] + {4'h0, OP2[7:4], 4'h0, OP2[3:0]}};
        LINK, TST:
            RESULT_tmp = OP2;
        MOVEA, MOVEM, MOVES:
            RESULT_tmp = OP1_SIGNEXT;
        default: // MOVE_FROM_CCR, MOVE_TO_CCR, MOVE_FROM_SR, MOVE_TO_SR, MOVE, MOVEC, MOVEP, STOP.
            RESULT_tmp = OP1;
    endcase
    RESULT_OTHERS = RESULT_tmp;
end

assign SHFT_LOAD = ALU_INIT && (OP_IN == ASL  || OP_IN == ASR  ||
                                OP_IN == LSL  || OP_IN == LSR  ||
                                OP_IN == ROTL || OP_IN == ROTR ||
                                OP_IN == ROXL || OP_IN == ROXR);

assign SHIFT_WIDTH_IN_sig = (BIW_0_IN[7:6] == 2'b11) ? 6'd1 : // Memory shifts.
                            (!BIW_0_IN[5] && BIW_0_IN[11:9] == 3'b000) ? 6'd8 : // Direct, count=0 means 8.
                            (!BIW_0_IN[5]) ? {3'b000, BIW_0_IN[11:9]} : // Direct.
                            OP1_IN[5:0];

always_ff @(posedge CLK) begin : shift_ctrl
    logic [5:0] BIT_CNT;

    SHFT_RDY <= 1'b0;

    if (SHIFT_STATE == SHIFT_IDLE) begin
        if (SHFT_LOAD && SHIFT_WIDTH_IN_sig == 6'd0) begin
            SHFT_RDY <= 1'b1;
        end else if (SHFT_LOAD) begin
            SHIFT_STATE <= SHIFT_RUN;
            BIT_CNT = SHIFT_WIDTH_IN_sig;
            SHFT_EN <= 1'b1;
        end else begin
            SHIFT_STATE <= SHIFT_IDLE;
            BIT_CNT = 6'd0;
            SHFT_EN <= 1'b0;
        end
    end else begin // SHIFT_RUN
        if (BIT_CNT == 6'd1) begin
            SHIFT_STATE <= SHIFT_IDLE;
            SHFT_EN <= 1'b0;
            SHFT_RDY <= 1'b1;
        end else begin
            SHIFT_STATE <= SHIFT_RUN;
            BIT_CNT = BIT_CNT - 1'b1;
            SHFT_EN <= 1'b1;
        end
    end
end

always_ff @(posedge CLK) begin : shifter
    if (SHFT_LOAD) begin // Load data in the shifter unit.
        RESULT_SHIFTOP <= OP2_IN; // Load data for the shift or rotate operations.
    end else if (SHFT_EN) begin // Shift and rotate operations:
        case (OP)
            ASL: begin
                if (OP_SIZE == LONG)
                    RESULT_SHIFTOP <= {RESULT_SHIFTOP[30:0], 1'b0};
                else if (OP_SIZE == WORD)
                    RESULT_SHIFTOP <= {16'h0, RESULT_SHIFTOP[14:0], 1'b0};
                else // OP_SIZE == BYTE.
                    RESULT_SHIFTOP <= {24'h0, RESULT_SHIFTOP[6:0], 1'b0};
            end
            ASR: begin
                if (OP_SIZE == LONG)
                    RESULT_SHIFTOP <= {RESULT_SHIFTOP[31], RESULT_SHIFTOP[31:1]};
                else if (OP_SIZE == WORD)
                    RESULT_SHIFTOP <= {16'h0, RESULT_SHIFTOP[15], RESULT_SHIFTOP[15:1]};
                else // OP_SIZE == BYTE.
                    RESULT_SHIFTOP <= {24'h0, RESULT_SHIFTOP[7], RESULT_SHIFTOP[7:1]};
            end
            LSL: begin
                if (OP_SIZE == LONG)
                    RESULT_SHIFTOP <= {RESULT_SHIFTOP[30:0], 1'b0};
                else if (OP_SIZE == WORD)
                    RESULT_SHIFTOP <= {16'h0, RESULT_SHIFTOP[14:0], 1'b0};
                else // OP_SIZE == BYTE.
                    RESULT_SHIFTOP <= {24'h0, RESULT_SHIFTOP[6:0], 1'b0};
            end
            LSR: begin
                if (OP_SIZE == LONG)
                    RESULT_SHIFTOP <= {1'b0, RESULT_SHIFTOP[31:1]};
                else if (OP_SIZE == WORD)
                    RESULT_SHIFTOP <= {16'h0, 1'b0, RESULT_SHIFTOP[15:1]};
                else // OP_SIZE == BYTE.
                    RESULT_SHIFTOP <= {24'h0, 1'b0, RESULT_SHIFTOP[7:1]};
            end
            ROTL: begin
                if (OP_SIZE == LONG)
                    RESULT_SHIFTOP <= {RESULT_SHIFTOP[30:0], RESULT_SHIFTOP[31]};
                else if (OP_SIZE == WORD)
                    RESULT_SHIFTOP <= {16'h0, RESULT_SHIFTOP[14:0], RESULT_SHIFTOP[15]};
                else // OP_SIZE == BYTE.
                    RESULT_SHIFTOP <= {24'h0, RESULT_SHIFTOP[6:0], RESULT_SHIFTOP[7]};
            end
            ROTR: begin
                if (OP_SIZE == LONG)
                    RESULT_SHIFTOP <= {RESULT_SHIFTOP[0], RESULT_SHIFTOP[31:1]};
                else if (OP_SIZE == WORD)
                    RESULT_SHIFTOP <= {16'h0, RESULT_SHIFTOP[0], RESULT_SHIFTOP[15:1]};
                else // OP_SIZE == BYTE.
                    RESULT_SHIFTOP <= {24'h0, RESULT_SHIFTOP[0], RESULT_SHIFTOP[7:1]};
            end
            ROXL: begin
                if (OP_SIZE == LONG)
                    RESULT_SHIFTOP <= {RESULT_SHIFTOP[30:0], XFLAG_SHFT};
                else if (OP_SIZE == WORD)
                    RESULT_SHIFTOP <= {16'h0, RESULT_SHIFTOP[14:0], XFLAG_SHFT};
                else // OP_SIZE == BYTE.
                    RESULT_SHIFTOP <= {24'h0, RESULT_SHIFTOP[6:0], XFLAG_SHFT};
            end
            ROXR: begin
                if (OP_SIZE == LONG)
                    RESULT_SHIFTOP <= {XFLAG_SHFT, RESULT_SHIFTOP[31:1]};
                else if (OP_SIZE == WORD)
                    RESULT_SHIFTOP <= {16'h0, XFLAG_SHFT, RESULT_SHIFTOP[15:1]};
                else // OP_SIZE == BYTE.
                    RESULT_SHIFTOP <= {24'h0, XFLAG_SHFT, RESULT_SHIFTOP[7:1]};
            end
            default: ; // Unaffected, forbidden.
        endcase
    end
end

always_ff @(posedge CLK) begin : result_out
    if (ALU_REQ) begin
        case (OP)
            ABCD, NBCD, SBCD:
                RESULT <= {56'h0, RESULT_BCDOP}; // Byte only.
            BFCHG, BFCLR, BFEXTS, BFEXTU, BFFFO, BFINS, BFSET, BFTST:
                RESULT <= HILOn ? {32'h0, RESULT_BITFIELD[39:8]} :
                                  {56'h0, RESULT_BITFIELD[7:0]};
            BCHG, BCLR, BSET, BTST:
                RESULT <= {32'h0, RESULT_BITOP};
            ADD, ADDA, ADDI, ADDQ, ADDX, CLR, CMP, CMPA, CMPI:
                RESULT <= {32'h0, RESULT_INTOP};
            CMPM, DBcc, NEG, NEGX, SUB, SUBA, SUBI, SUBQ, SUBX:
                RESULT <= {32'h0, RESULT_INTOP};
            AND_B, ANDI, EOR, EORI, NOT_B, OR_B, ORI:
                RESULT <= {32'h0, RESULT_LOGOP};
            ANDI_TO_SR, EORI_TO_SR, ORI_TO_SR: // Used for branch prediction.
                RESULT <= {32'h0, RESULT_LOGOP};
            ASL, ASR, LSL, LSR, ROTL, ROTR, ROXL, ROXR:
                RESULT <= {32'h0, RESULT_SHIFTOP};
            DIVS, DIVU:
                case (OP_SIZE)
                    LONG:    RESULT <= {REMAINDER, QUOTIENT};
                    default: RESULT <= {32'h0, REMAINDER[15:0], QUOTIENT[15:0]};
                endcase
            MULS, MULU:
                RESULT <= RESULT_MUL;
            PACK:
                RESULT <= {56'h0, RESULT_OTHERS[11:8], RESULT_OTHERS[3:0]};
            default:
                RESULT <= {OP2, RESULT_OTHERS}; // OP2 is used for EXG.
        endcase
    end
end

// Out of bounds condition:
assign CHK_CMP_COND = (OP == CHK && OP2_SIGNEXT[MSB]) || // Negative destination.
                      (OP == CHK && $signed(OP2_SIGNEXT) > $signed(OP1_SIGNEXT)) ||
                      ((OP == CHK2 || OP == CMP2) &&  CHK2CMP2_DR && OP_SIZE == LONG && OP2 < OP1) ||
                      ((OP == CHK2 || OP == CMP2) &&  CHK2CMP2_DR && OP_SIZE == LONG && OP2 > OP3) ||
                      ((OP == CHK2 || OP == CMP2) &&  CHK2CMP2_DR && OP_SIZE == WORD && OP2[15:0] < OP1[15:0]) ||
                      ((OP == CHK2 || OP == CMP2) &&  CHK2CMP2_DR && OP_SIZE == WORD && OP2[15:0] > OP3[15:0]) ||
                      ((OP == CHK2 || OP == CMP2) &&  CHK2CMP2_DR && OP2[7:0] < OP1[7:0]) ||
                      ((OP == CHK2 || OP == CMP2) &&  CHK2CMP2_DR && OP2[7:0] > OP3[7:0]) ||
                      ((OP == CHK2 || OP == CMP2) && !CHK2CMP2_DR && $signed(OP2_SIGNEXT) < $signed(OP1_SIGNEXT)) ||
                      ((OP == CHK2 || OP == CMP2) && !CHK2CMP2_DR && $signed(OP2_SIGNEXT) > $signed(OP3_SIGNEXT));

// All traps must be modeled as strobes.
assign TRAP_CHK = ALU_ACK && (OP == CHK || OP == CHK2) && CHK_CMP_COND;
assign TRAP_DIVZERO = ALU_INIT && (OP_IN == DIVS || OP_IN == DIVU) && OP1_IN == 32'h0;

// COND_CODES - mixed clocked/combinational process
// Registered part for XFLAG_SHFT, CFLAG_SHFT, VFLAG_SHFT
logic CFLAG_SHFT;
logic VFLAG_SHFT;

always_ff @(posedge CLK) begin : shifter_flags
    // Shifter X flag
    if (SHFT_LOAD || SHIFT_WIDTH == 6'd0) begin
        XFLAG_SHFT <= STATUS_REG[4];
    end else if (SHFT_EN) begin
        case (OP)
            ROTL, ROTR:
                XFLAG_SHFT <= STATUS_REG[4]; // Unaffected.
            ASL, LSL, ROXL:
                case (OP_SIZE)
                    LONG:    XFLAG_SHFT <= RESULT_SHIFTOP[31];
                    WORD:    XFLAG_SHFT <= RESULT_SHIFTOP[15];
                    BYTE:    XFLAG_SHFT <= RESULT_SHIFTOP[7];
                    default: ;
                endcase
            default: // ASR, LSR, ROXR.
                XFLAG_SHFT <= RESULT_SHIFTOP[0];
        endcase
    end
    //
    if ((OP == ROXL || OP == ROXR) && SHIFT_WIDTH == 6'd0)
        CFLAG_SHFT <= STATUS_REG[4];
    else if (SHIFT_WIDTH == 6'd0)
        CFLAG_SHFT <= 1'b0;
    else if (SHFT_EN) begin
        case (OP)
            ASL, LSL, ROTL, ROXL:
                case (OP_SIZE)
                    LONG:    CFLAG_SHFT <= RESULT_SHIFTOP[31];
                    WORD:    CFLAG_SHFT <= RESULT_SHIFTOP[15];
                    BYTE:    CFLAG_SHFT <= RESULT_SHIFTOP[7];
                    default: ;
                endcase
            default: // ASR, LSR, ROTR, ROXR
                CFLAG_SHFT <= RESULT_SHIFTOP[0];
        endcase
    end
    //
    // V flag for ASL
    if (SHFT_LOAD || SHIFT_WIDTH == 6'd0)
        VFLAG_SHFT <= 1'b0;
    else if (SHFT_EN) begin
        case (OP)
            ASL: begin // ASR MSB is always unchanged.
                if (OP_SIZE == LONG)
                    VFLAG_SHFT <= (RESULT_SHIFTOP[31] ^ RESULT_SHIFTOP[30]) | VFLAG_SHFT;
                else if (OP_SIZE == WORD)
                    VFLAG_SHFT <= (RESULT_SHIFTOP[15] ^ RESULT_SHIFTOP[14]) | VFLAG_SHFT;
                else // OP_SIZE == BYTE.
                    VFLAG_SHFT <= (RESULT_SHIFTOP[7] ^ RESULT_SHIFTOP[6]) | VFLAG_SHFT;
            end
            default:
                VFLAG_SHFT <= 1'b0;
        endcase
    end
end

// Combinational XNZVC calculation
always_comb begin : cond_codes_comb
    logic Z, RM_sig, SM_sig, DM_sig;
    logic NFLAG_DIV_sig;
    logic NFLAG_MUL_sig;
    logic VFLAG_MUL_sig;
    logic [2:0] RM_SM_DM;
    logic [39:0] bf_mask;

    bf_mask = 40'd0;
    if (OP_SIZE == LONG && QUOTIENT[31])
        NFLAG_DIV_sig = 1'b1;
    else if (OP_SIZE == WORD && QUOTIENT[15])
        NFLAG_DIV_sig = 1'b1;
    else
        NFLAG_DIV_sig = 1'b0;

    // Integer operations:
    case (OP)
        ADD, ADDI, ADDQ, ADDX, CMP, CMPA, CMPI, CMPM, NEG, NEGX, SUB, SUBI, SUBQ, SUBX: begin
            case (OP_SIZE)
                BYTE: begin
                    RM_sig = RESULT_INTOP[7];
                    SM_sig = OP1_SIGNEXT[7];
                    DM_sig = OP2_SIGNEXT[7];
                end
                WORD: begin
                    RM_sig = RESULT_INTOP[15];
                    SM_sig = OP1_SIGNEXT[15];
                    DM_sig = OP2_SIGNEXT[15];
                end
                default: begin
                    RM_sig = RESULT_INTOP[31];
                    SM_sig = OP1_SIGNEXT[31];
                    DM_sig = OP2_SIGNEXT[31];
                end
            endcase
        end
        default: begin
            RM_sig = 1'b0; SM_sig = 1'b0; DM_sig = 1'b0;
        end
    endcase

    RM_SM_DM = {RM_sig, SM_sig, DM_sig};

    // Multiplication:
    if (OP_SIZE == LONG && BIW_1[10] && RESULT_MUL[63]) // 64 bit result.
        NFLAG_MUL_sig = 1'b1;
    else if (RESULT_MUL[31]) // 32 bit result.
        NFLAG_MUL_sig = 1'b1;
    else
        NFLAG_MUL_sig = 1'b0;

    if (OP_SIZE == LONG && !BIW_1[10] && OP == MULS && !RESULT_MUL[31] && RESULT_MUL[63:32] != 32'h0)
        VFLAG_MUL_sig = 1'b1;
    else if (OP_SIZE == LONG && !BIW_1[10] && OP == MULS && RESULT_MUL[31] && RESULT_MUL[63:32] != 32'hFFFFFFFF)
        VFLAG_MUL_sig = 1'b1;
    else if (OP_SIZE == LONG && !BIW_1[10] && OP == MULU && RESULT_MUL[63:32] != 32'h0)
        VFLAG_MUL_sig = 1'b1;
    else
        VFLAG_MUL_sig = 1'b0;

    // The Z Flag:
    Z = 1'b0;
    case (OP)
        ADD, ADDI, ADDQ, ADDX, CAS, CAS2, CMP, CMPA, CMPI, CMPM, NEG, NEGX, SUB, SUBI, SUBQ, SUBX: begin
            case (OP_SIZE)
                BYTE:    Z = (RESULT_INTOP[7:0]  == 8'd0);
                WORD:    Z = (RESULT_INTOP[15:0] == 16'd0);
                default: Z = (RESULT_INTOP[31:0] == 32'd0);
            endcase
        end
        AND_B, ANDI, EOR, EORI, OR_B, ORI, NOT_B: begin
            case (OP_SIZE)
                BYTE:    Z = (RESULT_LOGOP[7:0]  == 8'd0);
                WORD:    Z = (RESULT_LOGOP[15:0] == 16'd0);
                default: Z = (RESULT_LOGOP[31:0] == 32'd0);
            endcase
        end
        ASL, ASR, LSL, LSR, ROTL, ROTR, ROXL, ROXR: begin
            case (OP_SIZE)
                BYTE:    Z = (RESULT_SHIFTOP[7:0]  == 8'd0);
                WORD:    Z = (RESULT_SHIFTOP[15:0] == 16'd0);
                default: Z = (RESULT_SHIFTOP[31:0] == 32'd0);
            endcase
        end
        BFCHG, BFCLR, BFEXTS, BFEXTU, BFFFO, BFINS, BFSET, BFTST: begin
            // Test if any bit in [BF_LOWER_BND:BF_UPPER_BND] is set
            bf_mask = ((40'd1 << (BF_UPPER_BND - BF_LOWER_BND + 6'd1)) - 40'd1) << BF_LOWER_BND;
            Z = ~|(BF_DATA_IN & bf_mask);
        end
        CHK2, CMP2: begin
            if (USE_DREG && OP_SIZE == LONG && OP2 == OP1)
                Z = 1'b1;
            else if (USE_DREG && OP_SIZE == LONG && OP2 == OP3)
                Z = 1'b1;
            else if (USE_DREG && OP_SIZE == WORD && OP2[15:0] == OP1[15:0])
                Z = 1'b1;
            else if (USE_DREG && OP_SIZE == WORD && OP2[15:0] == OP3[15:0])
                Z = 1'b1;
            else if (USE_DREG && OP2[7:0] == OP1[7:0])
                Z = 1'b1;
            else if (USE_DREG && OP2[7:0] == OP3[7:0])
                Z = 1'b1;
            else if (!USE_DREG && OP2_SIGNEXT == OP1_SIGNEXT)
                Z = 1'b1;
            else if (!USE_DREG && OP2_SIGNEXT == OP3_SIGNEXT)
                Z = 1'b1;
            else
                Z = 1'b0;
        end
        BCHG, BCLR, BSET, BTST:
            Z = ~OP2[BITPOS];
        DIVS, DIVU:
            Z = (QUOTIENT == 32'h0);
        EXT, EXTB, MOVE, SWAP, TST: begin
            case (OP_SIZE)
                BYTE:    Z = (RESULT_OTHERS[7:0]  == 8'd0);
                WORD:    Z = (RESULT_OTHERS[15:0] == 16'd0);
                default: Z = (RESULT_OTHERS[31:0] == 32'd0);
            endcase
        end
        MULS, MULU: begin
            if (OP_SIZE == LONG && BIW_1[10] && RESULT_MUL == 64'h0) // 64 bit result.
                Z = 1'b1;
            else if (RESULT_MUL[31:0] == 32'h0) // 32 bit result.
                Z = 1'b1;
            else
                Z = 1'b0;
        end
        TAS: begin
            case (OP_SIZE)
                BYTE:    Z = (OP2_SIGNEXT[7:0]  == 8'd0);
                WORD:    Z = (OP2_SIGNEXT[15:0] == 16'd0);
                default: Z = (OP2_SIGNEXT[31:0] == 32'd0);
            endcase
        end
        default:
            Z = 1'b0;
    endcase

    // XNZVC:
    XNZVC = 5'b00000;
    case (OP)
        ABCD, NBCD, SBCD: begin
            if (RESULT_BCDOP == 8'h00) // N and V are undefined, don't care.
                XNZVC = {CB_BCD, 1'bx, STATUS_REG[2], 1'bx, CB_BCD};
            else
                XNZVC = {CB_BCD, 1'bx, 1'b0, 1'bx, CB_BCD};
        end
        ADD, ADDI, ADDQ, ADDX: begin
            if ((SM_sig && DM_sig) || (!RM_sig && SM_sig) || (!RM_sig && DM_sig)) begin
                XNZVC[4] = 1'b1;
                XNZVC[0] = 1'b1;
            end else begin
                XNZVC[4] = 1'b0;
                XNZVC[0] = 1'b0;
            end
            //
            if (Z) begin
                if (OP == ADDX)
                    XNZVC[3:2] = {1'b0, STATUS_REG[2]};
                else
                    XNZVC[3:2] = 2'b01;
            end else begin
                XNZVC[3:2] = {RM_sig, 1'b0};
            end

            case (RM_SM_DM)
                3'b011, 3'b100: XNZVC[1] = 1'b1;
                default:        XNZVC[1] = 1'b0;
            endcase
        end
        AND_B, ANDI, EOR, EORI, OR_B, ORI, NOT_B:
            XNZVC = {STATUS_REG[4], RESULT_LOGOP[MSB], Z, 2'b00};
        ANDI_TO_CCR, EORI_TO_CCR, ORI_TO_CCR:
            XNZVC = RESULT_LOGOP[4:0];
        ASL, ASR, LSL, LSR, ROTL, ROTR, ROXL, ROXR:
            XNZVC = {XFLAG_SHFT, RESULT_SHIFTOP[MSB], Z, VFLAG_SHFT, CFLAG_SHFT};
        BCHG, BCLR, BSET, BTST:
            XNZVC = {STATUS_REG[4:3], Z, STATUS_REG[1:0]};
        BFCHG, BFCLR, BFEXTS, BFEXTU, BFFFO, BFINS, BFSET, BFTST:
            XNZVC = {STATUS_REG[4], BF_DATA_IN[BF_UPPER_BND], Z, 2'b00};
        CLR:
            XNZVC = {STATUS_REG[4], 4'b0100};
        SUB, SUBI, SUBQ, SUBX: begin
            if ((SM_sig && !DM_sig) || (RM_sig && SM_sig) || (RM_sig && !DM_sig)) begin
                XNZVC[4] = 1'b1;
                XNZVC[0] = 1'b1;
            end else begin
                XNZVC[4] = 1'b0;
                XNZVC[0] = 1'b0;
            end
            //
            if (Z) begin
                if (OP == SUBX)
                    XNZVC[3:2] = {1'b0, STATUS_REG[2]};
                else
                    XNZVC[3:2] = 2'b01;
            end else begin
                XNZVC[3:2] = {RM_sig, 1'b0};
            end

            case (RM_SM_DM)
                3'b001, 3'b110: XNZVC[1] = 1'b1;
                default:        XNZVC[1] = 1'b0;
            endcase
        end
        CAS, CAS2, CMP, CMPA, CMPI, CMPM: begin
            XNZVC[4] = STATUS_REG[4];

            if (Z)
                XNZVC[3:2] = 2'b01;
            else
                XNZVC[3:2] = {RM_sig, 1'b0};

            case (RM_SM_DM)
                3'b001, 3'b110: XNZVC[1] = 1'b1;
                default:        XNZVC[1] = 1'b0;
            endcase

            if ((SM_sig && !DM_sig) || (RM_sig && SM_sig) || (RM_sig && !DM_sig))
                XNZVC[0] = 1'b1;
            else
                XNZVC[0] = 1'b0;
        end
        CHK: begin
            if (OP2_SIGNEXT[MSB])
                XNZVC = {STATUS_REG[4], 1'b1, 3'b000};
            else if (CHK_CMP_COND)
                XNZVC = {STATUS_REG[4], 1'b0, 3'b000};
            else
                XNZVC = {STATUS_REG[4:3], 3'b000};
        end
        CHK2, CMP2: begin
            if (CHK_CMP_COND)
                XNZVC = {STATUS_REG[4], 3'b000, 1'b1};
            else
                XNZVC = {STATUS_REG[4], 1'b0, Z, 2'b00};
        end
        DIVS, DIVU:
            XNZVC = {STATUS_REG[4], NFLAG_DIV_sig, Z, VFLAG_DIV, 1'b0};
        EXT, EXTB, MOVE, TST:
            XNZVC = {STATUS_REG[4], RESULT_OTHERS[MSB], Z, 2'b00};
        MOVEQ: begin
            if (OP1_SIGNEXT[7:0] == 8'h00)
                XNZVC = {STATUS_REG[4], 4'b0100};
            else
                XNZVC = {STATUS_REG[4], OP1_SIGNEXT[7], 3'b000};
        end
        MULS, MULU:
            XNZVC = {STATUS_REG[4], NFLAG_MUL_sig, Z, VFLAG_MUL_sig, 1'b0};
        NEG, NEGX: begin
            XNZVC[4] = DM_sig | RM_sig;

            if (Z) begin
                if (OP == NEGX)
                    XNZVC[3:2] = {1'b0, STATUS_REG[2]};
                else
                    XNZVC[3:2] = 2'b01;
            end else begin
                XNZVC[3:2] = {RM_sig, 1'b0};
            end

            XNZVC[1] = DM_sig & RM_sig;
            XNZVC[0] = DM_sig | RM_sig;
        end
        RTR:
            XNZVC = OP2[4:0];
        SWAP:
            XNZVC = {STATUS_REG[4], RESULT_OTHERS[MSB], Z, 2'b00};
        default: // TAS, Byte only.
            XNZVC = {STATUS_REG[4], OP2_SIGNEXT[MSB], Z, 2'b00};
    endcase
end

always_ff @(posedge CLK) begin : cas_conditions
    if (LOAD_OP2 && XNZVC[2])
        CAS2_COND <= 1'b1;
    else if (LOAD_OP2)
        CAS2_COND <= 1'b0;
end

assign ALU_COND = ALU_COND_I; // This signal may not be registered to meet a correct timing.
// Status register conditions:
assign ALU_COND_I = (OP == CAS2 && !CAS2_COND) ? 1'b0 :
                    ((OP == CAS || OP == CAS2) && OP_SIZE == LONG && RESULT_INTOP == 32'h0) ? 1'b1 :
                    ((OP == CAS || OP == CAS2) && OP_SIZE == WORD && RESULT_INTOP[15:0] == 16'h0) ? 1'b1 :
                    (OP == CAS && RESULT_INTOP[7:0] == 8'h0) ? 1'b1 :
                    (OP == CAS || OP == CAS2) ? 1'b0 :
                    (OP == TRAPV &&  STATUS_REG[1]) ? 1'b1 :
                    (OP == TRAPV) ? 1'b0 :
                    (BIW_0[11:8] == 4'h0) ? 1'b1 : // True.
                    (BIW_0[11:8] == 4'h2 && !(STATUS_REG[2] | STATUS_REG[0])) ? 1'b1 : // High.
                    (BIW_0[11:8] == 4'h3 &&  (STATUS_REG[2] | STATUS_REG[0])) ? 1'b1 : // Low or same.
                    (BIW_0[11:8] == 4'h4 && !STATUS_REG[0]) ? 1'b1 : // Carry clear.
                    (BIW_0[11:8] == 4'h5 &&  STATUS_REG[0]) ? 1'b1 : // Carry set.
                    (BIW_0[11:8] == 4'h6 && !STATUS_REG[2]) ? 1'b1 : // Not Equal.
                    (BIW_0[11:8] == 4'h7 &&  STATUS_REG[2]) ? 1'b1 : // Equal.
                    (BIW_0[11:8] == 4'h8 && !STATUS_REG[1]) ? 1'b1 : // Overflow clear.
                    (BIW_0[11:8] == 4'h9 &&  STATUS_REG[1]) ? 1'b1 : // Overflow set.
                    (BIW_0[11:8] == 4'hA && !STATUS_REG[3]) ? 1'b1 : // Plus.
                    (BIW_0[11:8] == 4'hB &&  STATUS_REG[3]) ? 1'b1 : // Minus.
                    (BIW_0[11:8] == 4'hC && !(STATUS_REG[3] ^ STATUS_REG[1])) ? 1'b1 : // Greater or Equal.
                    (BIW_0[11:8] == 4'hD &&  (STATUS_REG[3] ^ STATUS_REG[1])) ? 1'b1 : // Less than.
                    (BIW_0[11:8] == 4'hE && STATUS_REG[3:1] == 3'b101) ? 1'b1 : // Greater than.
                    (BIW_0[11:8] == 4'hE && STATUS_REG[3:1] == 3'b000) ? 1'b1 : // Greater than.
                    (BIW_0[11:8] == 4'hF &&  STATUS_REG[2]) ? 1'b1 : // Less or equal.
                    (BIW_0[11:8] == 4'hF &&  (STATUS_REG[3] ^ STATUS_REG[1])) ? 1'b1 : 1'b0; // Less or equal.

always_ff @(posedge CLK) begin : status_reg_proc
    // This process is the status register with its related logic.
    logic [15:0] SREG_MEM;

    if (CC_UPDT)
        SREG_MEM[4:0] = XNZVC;

    if (SR_INIT) begin
        SREG_MEM[15:13] = 3'b001; // Trace cleared, S = '1'.
        SREG_MEM[10:8] = IRQ_PEND; // Update IRQ level.
    end

    if (SR_CLR_MBIT)
        SREG_MEM[12] = 1'b0;
    else if (SR_WR && OP_IN == RTE) // Written by the exception handler, no ALU required.
        SREG_MEM = OP1_IN[15:0];
    else if (SR_WR && (OP_WB == MOVE_TO_CCR || OP_WB == MOVE_TO_SR || OP_WB == STOP))
        SREG_MEM = RESULT_OTHERS[15:0];
    else if (SR_WR && (OP_WB == ANDI_TO_CCR || OP_WB == EORI_TO_CCR || OP_WB == ORI_TO_CCR))
        SREG_MEM[7:5] = RESULT_LOGOP[7:5]; // Bits 4 downto 0 are written via CC_UPDT.
    else if (SR_WR) // ANDI_TO_SR, EORI_TO_SR, ORI_TO_SR.
        SREG_MEM = RESULT_LOGOP[15:0];

    STATUS_REG <= SREG_MEM; // Fully populated status register.
end

//
assign STATUS_REG_OUT = STATUS_REG;

endmodule
