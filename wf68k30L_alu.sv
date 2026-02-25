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

// ---- BCD constants ----
localparam logic [4:0] BCD_DIGIT_MAX  = 5'b01001; // 9: threshold for BCD correction
localparam logic [3:0] BCD_CORRECTION = 4'b0110;  // 6: BCD digit correction value

// ---- Internal signals ----
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
logic [4:0]  MSB;
OP_68K       OP;
logic [31:0] OP1;
logic [31:0] OP1_BUFFER;
logic [31:0] OP2;
logic [31:0] OP2_BUFFER;
logic [31:0] OP3;
logic [31:0] OP3_BUFFER;
logic [31:0] OP1_SIGNEXT;
logic [31:0] OP2_SIGNEXT;
logic [31:0] OP3_SIGNEXT;
OP_SIZETYPE  OP_SIZE;
logic [7:0]  RESULT_BCDOP;
logic [39:0] RESULT_BITFIELD;
logic [31:0] RESULT_BITOP;
logic [31:0] RESULT_INTOP;
logic [31:0] RESULT_LOGOP;
logic [63:0] RESULT_MUL;
logic [31:0] RESULT_OTHERS;
logic [5:0]  SHIFT_WIDTH;
logic [5:0]  SHIFT_WIDTH_IN_sig;
logic        SHFT_LOAD;
logic [15:0] STATUS_REG;
logic [4:0]  XNZVC;

// ---- Submodule interface signals ----
// Shifter
logic [31:0] RESULT_SHIFTOP;
logic        SHFT_RDY;
logic        XFLAG_SHFT;
logic        CFLAG_SHFT;
logic        VFLAG_SHFT;

// Divider
logic [31:0] QUOTIENT;
logic [31:0] REMAINDER;
logic        VFLAG_DIV;
logic        DIV_RDY;

// ========================================================================
// Parameter buffer: latch inputs on ALU_INIT
// ========================================================================
always_ff @(posedge CLK) begin : parameter_buffer
    if (ALU_INIT) begin
        ADR_MODE <= ADR_MODE_IN;
        CHK2CMP2_DR <= USE_DREG;
        OP_SIZE <= OP_SIZE_IN;
        OP <= OP_IN;
        BIW_0 <= BIW_0_IN;
        BIW_1 <= BIW_1_IN;
        BF_OFFSET <= BF_OFFSET_IN;
        BITPOS <= BITPOS_IN;
        BF_WIDTH <= BF_WIDTH_IN;
        BF_UPPER_BND <= 6'd39 - {1'b0, BITPOS_IN};
        SHIFT_WIDTH <= SHIFT_WIDTH_IN_sig;

        if ((BITPOS_IN + BF_WIDTH_IN) > 40)
            BF_LOWER_BND <= 6'd0;
        else
            BF_LOWER_BND <= 6'd40 - ({1'b0, BITPOS_IN} + BF_WIDTH_IN);
    end
end

// ========================================================================
// Operand buffers
// ========================================================================
always_ff @(posedge CLK) begin : operands
    // During instruction execution, operand buffers can be loaded one or
    // more cycles ahead of ALU_INIT. Keep explicit buffer registers and
    // preserve the same-cycle load+init bypass behavior.
    if (LOAD_OP1)
        OP1_BUFFER <= OP1_IN;
    if (LOAD_OP2)
        OP2_BUFFER <= OP2_IN;
    if (LOAD_OP3)
        OP3_BUFFER <= OP3_IN;

    if (ALU_INIT) begin
        OP1 <= LOAD_OP1 ? OP1_IN : OP1_BUFFER;
        OP2 <= LOAD_OP2 ? OP2_IN : OP2_BUFFER;
        OP3 <= LOAD_OP3 ? OP3_IN : OP3_BUFFER;
    end
end

// ========================================================================
// ALU busy/request handshake
// ========================================================================
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

// ========================================================================
// MSB position based on operand size
// ========================================================================
always_comb begin
    case (OP_SIZE)
        LONG:    MSB = 5'd31;
        WORD:    MSB = 5'd15;
        BYTE:    MSB = 5'd7;
        default: MSB = 5'd31;
    endcase
end

// ========================================================================
// Sign extension
// ========================================================================
always_comb begin : sign_extend
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

// ========================================================================
// BCD operations (ABCD, SBCD, NBCD)
// ========================================================================
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

    X_IN_I = STATUS_REG[SR_X]; // Extended Flag.

    // Low nibble computation
    case (OP)
        ABCD:
            TEMP0 = {1'b0, OP2[3:0]} + {1'b0, OP1[3:0]} + {4'b0000, X_IN_I};
        NBCD:
            TEMP0 = OP1[4:0] - {1'b0, OP2[3:0]} - {4'b0000, X_IN_I};
        default: // Valid for SBCD.
            TEMP0 = {1'b0, OP2[3:0]} - {1'b0, OP1[3:0]} - {4'b0000, X_IN_I};
    endcase

    if (TEMP0 > BCD_DIGIT_MAX) begin
        Z_0 = BCD_CORRECTION;
        C_0 = 1'b1;
    end else begin
        Z_0 = 4'b0000;
        C_0 = 1'b0;
    end

    // High nibble computation
    case (OP)
        ABCD:
            TEMP1 = {1'b0, OP2[7:4]} + {1'b0, OP1[7:4]} + {4'b0000, C_0};
        NBCD:
            TEMP1 = OP1[4:0] - {1'b0, OP2[7:4]} - {4'b0000, X_IN_I};
        default: // Valid for SBCD.
            TEMP1 = {1'b0, OP2[7:4]} - {1'b0, OP1[7:4]} - {4'b0000, C_0};
    endcase

    if (TEMP1 > BCD_DIGIT_MAX) begin
        Z_1 = BCD_CORRECTION;
        C_1 = 1'b1;
    end else begin
        Z_1 = 4'b0000;
        C_1 = 1'b0;
    end

    // Apply correction
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

    CB_BCD = C_1;
    RESULT_BCDOP[7:4] = S_1;
    RESULT_BCDOP[3:0] = S_0;
end

// ========================================================================
// Bit field operations
// ========================================================================
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

// ========================================================================
// Bit manipulation operations
// ========================================================================
always_comb begin : bit_op
    RESULT_BITOP = OP2; // The default is the unmanipulated data.
    case (OP)
        BCHG:    RESULT_BITOP[BITPOS] = ~OP2[BITPOS];
        BCLR:    RESULT_BITOP[BITPOS] = 1'b0;
        BSET:    RESULT_BITOP[BITPOS] = 1'b1;
        default: RESULT_BITOP = OP2; // Dummy, no result required for BTST.
    endcase
end

// ========================================================================
// Divider submodule
// ========================================================================
WF68K30L_DIVIDER I_DIVIDER (
    .CLK        (CLK),
    .OP         (OP),
    .OP_IN      (OP_IN),
    .OP_SIZE    (OP_SIZE),
    .ALU_INIT   (ALU_INIT),
    .BIW_1      (BIW_1),
    .OP1        (OP1),
    .OP2        (OP2),
    .OP3        (OP3),
    .QUOTIENT   (QUOTIENT),
    .REMAINDER  (REMAINDER),
    .VFLAG_DIV  (VFLAG_DIV),
    .DIV_RDY    (DIV_RDY)
);

// ========================================================================
// Integer arithmetic operations
// ========================================================================
always_comb begin : integer_op
    logic [0:0] X_IN_I;
    logic [31:0] RESULT_tmp;

    X_IN_I[0] = STATUS_REG[SR_X]; // Extended Flag.
    case (OP)
        ADDA: // No sign extension for the destination.
            RESULT_tmp = OP2 + OP1_SIGNEXT;
        ADDQ:
            case (ADR_MODE)
                ADR_AN:  RESULT_tmp = OP2 + OP1; // No sign extension for address destination.
                default: RESULT_tmp = OP2_SIGNEXT + OP1;
            endcase
        SUBQ:
            case (ADR_MODE)
                ADR_AN:  RESULT_tmp = OP2 - OP1; // No sign extension for address destination.
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

// ========================================================================
// Logic operations
// ========================================================================
always_comb begin : logic_op
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

// ========================================================================
// Multiplication
// ========================================================================
assign RESULT_MUL = (OP == MULS) ? ($signed(OP1_SIGNEXT) * $signed(OP2_SIGNEXT)) :
                    (OP_SIZE == LONG) ? (OP1 * OP2) :
                    ({16'h0, OP1[15:0]} * {16'h0, OP2[15:0]});

// ========================================================================
// Other/special operations
// ========================================================================
always_comb begin : other_ops
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

// ========================================================================
// Shifter submodule
// ========================================================================
assign SHFT_LOAD = ALU_INIT && (OP_IN == ASL  || OP_IN == ASR  ||
                                OP_IN == LSL  || OP_IN == LSR  ||
                                OP_IN == ROTL || OP_IN == ROTR ||
                                OP_IN == ROXL || OP_IN == ROXR);

assign SHIFT_WIDTH_IN_sig = (BIW_0_IN[7:6] == 2'b11) ? 6'd1 : // Memory shifts.
                            (!BIW_0_IN[5] && BIW_0_IN[11:9] == 3'b000) ? 6'd8 : // Direct, count=0 means 8.
                            (!BIW_0_IN[5]) ? {3'b000, BIW_0_IN[11:9]} : // Direct.
                            OP1_IN[5:0];

WF68K30L_SHIFTER I_SHIFTER (
    .CLK            (CLK),
    .OP             (OP),
    .OP_SIZE        (OP_SIZE),
    .SHIFT_WIDTH_IN (SHIFT_WIDTH_IN_sig),
    .SHIFT_WIDTH    (SHIFT_WIDTH),
    .SHFT_LOAD      (SHFT_LOAD),
    .DATA_IN        (OP2_IN),
    .SR_X_FLAG      (STATUS_REG[SR_X]),
    .RESULT_SHIFTOP (RESULT_SHIFTOP),
    .SHFT_RDY       (SHFT_RDY),
    .XFLAG_SHFT     (XFLAG_SHFT),
    .CFLAG_SHFT     (CFLAG_SHFT),
    .VFLAG_SHFT     (VFLAG_SHFT)
);

// ========================================================================
// Result output mux
// ========================================================================
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

// ========================================================================
// Out of bounds condition (CHK, CHK2, CMP2)
// ========================================================================
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

// ========================================================================
// Condition code computation
// ========================================================================

// -- Helper: compute Z flag for a sized result --
function automatic logic compute_z_sized(input logic [31:0] result, input logic [1:0] size);
    if (size == BYTE)
        compute_z_sized = (result[7:0]  == 8'd0);
    else if (size == WORD)
        compute_z_sized = (result[15:0] == 16'd0);
    else
        compute_z_sized = (result[31:0] == 32'd0);
endfunction

// -- Helper: compute addition flags (ADD/ADDX) --
// Returns {X, N, Z, V, C}
function automatic logic [4:0] compute_add_flags(
    input logic RM, input logic SM, input logic DM, input logic Z,
    input logic is_addx, input logic prev_z
);
    logic XC, N, ZF, V;
    XC = (SM && DM) || (!RM && SM) || (!RM && DM);
    if (Z) begin
        N  = 1'b0;
        ZF = is_addx ? prev_z : 1'b1;
    end else begin
        N  = RM;
        ZF = 1'b0;
    end
    if ((!RM && SM && DM) || (RM && !SM && !DM))
        V = 1'b1;
    else
        V = 1'b0;
    compute_add_flags = {XC, N, ZF, V, XC};
endfunction

// -- Helper: compute subtraction flags (SUB/SUBX) --
function automatic logic [4:0] compute_sub_flags(
    input logic RM, input logic SM, input logic DM, input logic Z,
    input logic is_subx, input logic prev_z
);
    logic XC, N, ZF, V;
    XC = (SM && !DM) || (RM && SM) || (RM && !DM);
    if (Z) begin
        N  = 1'b0;
        ZF = is_subx ? prev_z : 1'b1;
    end else begin
        N  = RM;
        ZF = 1'b0;
    end
    // V: overflow -- case {RM,SM,DM} 3'b001 or 3'b110
    if ((!RM && !SM && DM) || (RM && SM && !DM))
        V = 1'b1;
    else
        V = 1'b0;
    compute_sub_flags = {XC, N, ZF, V, XC};
endfunction

// -- Helper: compute compare flags (CAS/CMP -- subtraction without X update) --
function automatic logic [4:0] compute_cmp_flags(
    input logic RM, input logic SM, input logic DM, input logic Z, input logic prev_x
);
    logic C, N, ZF, V;
    C = (SM && !DM) || (RM && SM) || (RM && !DM);
    if (Z) begin
        N  = 1'b0;
        ZF = 1'b1;
    end else begin
        N  = RM;
        ZF = 1'b0;
    end
    // V: overflow -- case {RM,SM,DM} 3'b001 or 3'b110
    if ((!RM && !SM && DM) || (RM && SM && !DM))
        V = 1'b1;
    else
        V = 1'b0;
    compute_cmp_flags = {prev_x, N, ZF, V, C};
endfunction

// -- Helper: compute logical flags (AND/OR/EOR/NOT) --
function automatic logic [4:0] compute_logical_flags(
    input logic [31:0] result, input logic [4:0] msb_pos, input logic Z, input logic prev_x
);
    compute_logical_flags = {prev_x, result[msb_pos], Z, 2'b00};
endfunction

// -- Helper: compute shift/rotate flags --
function automatic logic [4:0] compute_shift_flags(
    input logic [31:0] result, input logic [4:0] msb_pos, input logic Z,
    input logic xflag, input logic vflag, input logic cflag
);
    compute_shift_flags = {xflag, result[msb_pos], Z, vflag, cflag};
endfunction

// -- Helper: compute BCD flags --
function automatic logic [4:0] compute_bcd_flags(
    input logic [7:0] result_bcd, input logic cb, input logic prev_z
);
    if (result_bcd == 8'h00)
        compute_bcd_flags = {cb, 1'bx, prev_z, 1'bx, cb};
    else
        compute_bcd_flags = {cb, 1'bx, 1'b0, 1'bx, cb};
endfunction

// Combinational XNZVC calculation
always_comb begin : cond_codes_comb
    logic Z, RM_sig, SM_sig, DM_sig;
    logic NFLAG_DIV_sig;
    logic NFLAG_MUL_sig;
    logic VFLAG_MUL_sig;
    logic [39:0] bf_mask;

    bf_mask = 40'd0;

    // Division N flag
    if (OP_SIZE == LONG && QUOTIENT[31])
        NFLAG_DIV_sig = 1'b1;
    else if (OP_SIZE == WORD && QUOTIENT[15])
        NFLAG_DIV_sig = 1'b1;
    else
        NFLAG_DIV_sig = 1'b0;

    // Integer result/source/destination MSBs for arithmetic flag computation
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

    // Multiplication flags
    if (OP_SIZE == LONG && BIW_1[10] && RESULT_MUL[63])
        NFLAG_MUL_sig = 1'b1;
    else if (RESULT_MUL[31])
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

    // Z flag computation
    Z = 1'b0;
    case (OP)
        ADD, ADDI, ADDQ, ADDX, CAS, CAS2, CMP, CMPA, CMPI, CMPM, NEG, NEGX, SUB, SUBI, SUBQ, SUBX:
            Z = compute_z_sized(RESULT_INTOP, OP_SIZE);
        AND_B, ANDI, EOR, EORI, OR_B, ORI, NOT_B:
            Z = compute_z_sized(RESULT_LOGOP, OP_SIZE);
        ASL, ASR, LSL, LSR, ROTL, ROTR, ROXL, ROXR:
            Z = compute_z_sized(RESULT_SHIFTOP, OP_SIZE);
        BFCHG, BFCLR, BFEXTS, BFEXTU, BFFFO, BFINS, BFSET, BFTST: begin
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
        EXT, EXTB, MOVE, SWAP, TST:
            Z = compute_z_sized(RESULT_OTHERS, OP_SIZE);
        MULS, MULU: begin
            if (OP_SIZE == LONG && BIW_1[10] && RESULT_MUL == 64'h0)
                Z = 1'b1;
            else if (RESULT_MUL[31:0] == 32'h0)
                Z = 1'b1;
            else
                Z = 1'b0;
        end
        TAS:
            Z = compute_z_sized(OP2_SIGNEXT, OP_SIZE);
        default:
            Z = 1'b0;
    endcase

    // XNZVC flag assignment using helper functions
    XNZVC = 5'b00000;
    case (OP)
        ABCD, NBCD, SBCD:
            XNZVC = compute_bcd_flags(RESULT_BCDOP, CB_BCD, STATUS_REG[SR_Z]);
        ADD, ADDI, ADDQ, ADDX:
            XNZVC = compute_add_flags(RM_sig, SM_sig, DM_sig, Z,
                                      (OP == ADDX), STATUS_REG[SR_Z]);
        AND_B, ANDI, EOR, EORI, OR_B, ORI, NOT_B:
            XNZVC = compute_logical_flags(RESULT_LOGOP, MSB, Z, STATUS_REG[SR_X]);
        ANDI_TO_CCR, EORI_TO_CCR, ORI_TO_CCR:
            XNZVC = RESULT_LOGOP[4:0];
        ASL, ASR, LSL, LSR, ROTL, ROTR, ROXL, ROXR:
            XNZVC = compute_shift_flags(RESULT_SHIFTOP, MSB, Z, XFLAG_SHFT, VFLAG_SHFT, CFLAG_SHFT);
        BCHG, BCLR, BSET, BTST:
            XNZVC = {STATUS_REG[SR_X:SR_N], Z, STATUS_REG[SR_V:SR_C]};
        BFCHG, BFCLR, BFEXTS, BFEXTU, BFFFO, BFINS, BFSET, BFTST:
            XNZVC = {STATUS_REG[SR_X], BF_DATA_IN[BF_UPPER_BND], Z, 2'b00};
        CLR:
            XNZVC = {STATUS_REG[SR_X], 4'b0100};
        SUB, SUBI, SUBQ, SUBX:
            XNZVC = compute_sub_flags(RM_sig, SM_sig, DM_sig, Z,
                                      (OP == SUBX), STATUS_REG[SR_Z]);
        CAS, CAS2, CMP, CMPA, CMPI, CMPM:
            XNZVC = compute_cmp_flags(RM_sig, SM_sig, DM_sig, Z, STATUS_REG[SR_X]);
        CHK: begin
            if (OP2_SIGNEXT[MSB])
                XNZVC = {STATUS_REG[SR_X], 1'b1, 3'b000};
            else if (CHK_CMP_COND)
                XNZVC = {STATUS_REG[SR_X], 1'b0, 3'b000};
            else
                XNZVC = {STATUS_REG[SR_X:SR_N], 3'b000};
        end
        CHK2, CMP2: begin
            if (CHK_CMP_COND)
                XNZVC = {STATUS_REG[SR_X], 3'b000, 1'b1};
            else
                XNZVC = {STATUS_REG[SR_X], 1'b0, Z, 2'b00};
        end
        DIVS, DIVU:
            XNZVC = {STATUS_REG[SR_X], NFLAG_DIV_sig, Z, VFLAG_DIV, 1'b0};
        EXT, EXTB, MOVE, TST:
            XNZVC = {STATUS_REG[SR_X], RESULT_OTHERS[MSB], Z, 2'b00};
        MOVEQ: begin
            if (OP1_SIGNEXT[7:0] == 8'h00)
                XNZVC = {STATUS_REG[SR_X], 4'b0100};
            else
                XNZVC = {STATUS_REG[SR_X], OP1_SIGNEXT[7], 3'b000};
        end
        MULS, MULU:
            XNZVC = {STATUS_REG[SR_X], NFLAG_MUL_sig, Z, VFLAG_MUL_sig, 1'b0};
        NEG, NEGX: begin
            XNZVC[4] = DM_sig | RM_sig;

            if (Z) begin
                if (OP == NEGX)
                    XNZVC[3:2] = {1'b0, STATUS_REG[SR_Z]};
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
            XNZVC = {STATUS_REG[SR_X], RESULT_OTHERS[MSB], Z, 2'b00};
        default: // TAS, Byte only.
            XNZVC = {STATUS_REG[SR_X], OP2_SIGNEXT[MSB], Z, 2'b00};
    endcase
end

// ========================================================================
// CAS2 condition latch
// ========================================================================
always_ff @(posedge CLK) begin : cas_conditions
    if (LOAD_OP2 && XNZVC[SR_Z])
        CAS2_COND <= 1'b1;
    else if (LOAD_OP2)
        CAS2_COND <= 1'b0;
end

// ========================================================================
// ALU condition evaluation
// ========================================================================
assign ALU_COND = ALU_COND_I; // This signal may not be registered to meet a correct timing.
// Status register conditions:
always_comb begin : alu_condition_eval
    logic n_flag;
    logic z_flag;
    logic v_flag;
    logic c_flag;
    logic n_xor_v;
    logic cas_compare_match;

    n_flag = STATUS_REG[SR_N];
    z_flag = STATUS_REG[SR_Z];
    v_flag = STATUS_REG[SR_V];
    c_flag = STATUS_REG[SR_C];
    n_xor_v = n_flag ^ v_flag;

    cas_compare_match = 1'b0;
    if (OP == CAS || OP == CAS2) begin
        case (OP_SIZE)
            LONG:    cas_compare_match = (RESULT_INTOP == 32'h00000000);
            WORD:    cas_compare_match = (RESULT_INTOP[15:0] == 16'h0000);
            default: cas_compare_match = (OP == CAS) && (RESULT_INTOP[7:0] == 8'h00);
        endcase
    end

    if (OP == CAS2 && !CAS2_COND) begin
        ALU_COND_I = 1'b0;
    end else if (OP == CAS || OP == CAS2) begin
        ALU_COND_I = cas_compare_match;
    end else if (OP == TRAPV) begin
        ALU_COND_I = v_flag;
    end else begin
        case (BIW_0[11:8])
            4'h0: ALU_COND_I = 1'b1;                   // True.
            4'h1: ALU_COND_I = 1'b0;                   // False.
            4'h2: ALU_COND_I = !(z_flag | c_flag);     // High.
            4'h3: ALU_COND_I = (z_flag | c_flag);      // Low or same.
            4'h4: ALU_COND_I = !c_flag;                // Carry clear.
            4'h5: ALU_COND_I = c_flag;                 // Carry set.
            4'h6: ALU_COND_I = !z_flag;                // Not Equal.
            4'h7: ALU_COND_I = z_flag;                 // Equal.
            4'h8: ALU_COND_I = !v_flag;                // Overflow clear.
            4'h9: ALU_COND_I = v_flag;                 // Overflow set.
            4'hA: ALU_COND_I = !n_flag;                // Plus.
            4'hB: ALU_COND_I = n_flag;                 // Minus.
            4'hC: ALU_COND_I = !n_xor_v;               // Greater or Equal.
            4'hD: ALU_COND_I = n_xor_v;                // Less than.
            4'hE: ALU_COND_I = !z_flag && !n_xor_v;    // Greater than.
            4'hF: ALU_COND_I = z_flag || n_xor_v;      // Less or equal.
            default: ALU_COND_I = 1'b0;
        endcase
    end
end

// ========================================================================
// Status register
// ========================================================================
always_ff @(posedge CLK) begin : status_reg_proc
    logic [15:0] SREG_MEM;
    SREG_MEM = STATUS_REG;

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

assign STATUS_REG_OUT = STATUS_REG;

endmodule
