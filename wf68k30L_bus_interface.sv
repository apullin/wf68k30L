// ------------------------------------------------------------------------
// -- WF68K30L IP Core: Bus Interface                                    --
// -- Author(s): Wolfgang Foerster, wf@experiment-s.de                   --
// -- Copyright (c) 2014-2019 Wolfgang Foerster Inventronik GmbH.        --
// -- CERN OHL v. 1.2                                                    --
// ------------------------------------------------------------------------

module WF68K30L_BUS_INTERFACE (
    input  logic        CLK,

    input  logic [31:0] ADR_IN_P,
    output logic [31:0] ADR_OUT_P,

    input  logic [2:0]  FC_IN,
    output logic [2:0]  FC_OUT,

    input  logic [31:0] DATA_PORT_IN,
    output logic [31:0] DATA_PORT_OUT,
    input  logic [31:0] DATA_FROM_CORE,
    output logic [31:0] DATA_TO_CORE,
    output logic [15:0] OPCODE_TO_CORE,

    output logic        DATA_PORT_EN,
    output logic        BUS_EN,

    output logic [1:0]  SIZE,
    input  logic [1:0]  OP_SIZE,

    input  logic        RD_REQ,
    input  logic        WR_REQ,
    output logic        DATA_RDY,
    output logic        DATA_VALID,
    input  logic        OPCODE_REQ,
    output logic        OPCODE_RDY,
    output logic        OPCODE_VALID,
    input  logic        RMC,
    input  logic        BUSY_EXH,
    output logic [31:0] INBUFFER,
    output logic [31:0] OUTBUFFER,
    output logic [8:0]  SSW_80,

    input  logic [1:0]  DSACKn,
    output logic        ASn,
    output logic        DSn,
    output logic        RWn,
    output logic        RMCn,
    output logic        ECSn,
    output logic        OCSn,
    output logic        DBENn,

    input  logic        STERMn,

    input  logic        BRn,
    input  logic        BGACKn,
    output logic        BGn,

    input  logic        RESET_STRB,
    input  logic        RESET_IN,
    output logic        RESET_OUT,
    output logic        RESET_CPU,
    input  logic        AVECn,
    input  logic        HALTn,
    input  logic        BERRn,
    output logic        AERR,

    output logic        BUS_BSY
);

`include "wf68k30L_pkg.svh"

typedef enum logic [1:0] {BUS_IDLE, START_CYCLE, DATA_C1C4} BUS_CTRL_STATES;
typedef enum logic [1:0] {ARB_IDLE, GRANT, WAIT_RELEASE_3WIRE} ARB_STATES;
typedef enum logic [1:0] {LONG_32, BW_WORD, BW_BYTE} BUS_WIDTH_TYPE;
typedef enum logic [2:0] {SLICE_IDLE, S0, S1, S2, S3, S4, S5} TIME_SLICES;

logic [1:0]         ADR_10;
logic [5:0]         ADR_OFFSET;
logic [31:0]        ADR_OUT_I;
logic               AERR_I;
ARB_STATES          ARB_STATE;
logic               AVEC_In;
logic               BGACK_In;
logic               BR_In;
BUS_CTRL_STATES     BUS_CTRL_STATE;
logic               BUS_CYC_RDY;
logic               BUS_FLT;
BUS_WIDTH_TYPE      BUS_WIDTH;
logic [31:0]        DATA_INMUX;
logic               DATA_RDY_I;
logic [31:0]        DBUFFER;
logic [1:0]         DSACK_In;
logic [1:0]         DSACK_MEM;
logic               OCS_INH;
logic               HALT_In;
logic               HALTED;
ARB_STATES          NEXT_ARB_STATE;
BUS_CTRL_STATES     NEXT_BUS_CTRL_STATE;
logic [15:0]        OBUFFER;
logic               OPCODE_ACCESS;
logic               OPCODE_RDY_I;
logic               READ_ACCESS;
logic               RESET_CPU_I;
logic               RESET_OUT_I;
logic               RETRY;
logic [1:0]         SIZE_D;
logic [1:0]         SIZE_I;
logic [2:0]         SIZE_N;
logic [2:0]         SLICE_CNT_N;
logic [2:0]         SLICE_CNT_P;
logic               STERM_In;
TIME_SLICES         T_SLICE;
logic               WAITSTATES;
logic [31:0]        WP_BUFFER;
logic               WRITE_ACCESS;

// P_SYNC: synchronize bus termination on negative clock edge
logic BUS_FLT_VAR;

always_ff @(negedge CLK) begin
    DSACK_In <= DSACKn;
    HALT_In <= HALTn;
    BUS_FLT_VAR <= ~BERRn;
    STERM_In <= STERMn;
    BR_In <= BRn;
    BGACK_In <= BGACKn;
    AVEC_In <= AVECn;
    //
    if (BERRn == 1'b0 && HALTn == 1'b0 && BUS_CTRL_STATE == DATA_C1C4 && SIZE_N != 3'b000)
        RETRY <= 1'b1;
    else if (T_SLICE == SLICE_IDLE && (BERRn == 1'b1 || HALTn == 1'b1))
        RETRY <= 1'b0;
end

always_ff @(posedge CLK) begin
    if (BUS_CTRL_STATE == START_CYCLE)
        AERR <= AERR_I;
    else if (BUS_CTRL_STATE == DATA_C1C4)
        BUS_FLT <= BUS_FLT_VAR;
    else begin
        BUS_FLT <= 1'b0;
        AERR <= 1'b0;
    end
end

always_ff @(posedge CLK) begin : ACCESSTYPE
    if (BUS_CTRL_STATE == START_CYCLE) begin
        if (READ_ACCESS == 1'b1 || WRITE_ACCESS == 1'b1 || OPCODE_ACCESS == 1'b1)
            ; // Do not start either new cycle.
        else if (RD_REQ == 1'b1)
            READ_ACCESS <= 1'b1;
        else if (WR_REQ == 1'b1)
            WRITE_ACCESS <= 1'b1;
        else if (OPCODE_REQ == 1'b1)
            OPCODE_ACCESS <= 1'b1;
    end else if (AERR == 1'b1) begin // Reject due to address error.
        READ_ACCESS <= 1'b0;
        WRITE_ACCESS <= 1'b0;
        OPCODE_ACCESS <= 1'b0;
    end else if (BUS_CTRL_STATE == DATA_C1C4 && NEXT_BUS_CTRL_STATE == BUS_IDLE && SIZE_N == 3'b000) begin
        READ_ACCESS <= 1'b0;
        WRITE_ACCESS <= 1'b0;
        OPCODE_ACCESS <= 1'b0;
    end
end

always_ff @(posedge CLK) begin : P_DF
    logic [1:0] SIZEVAR;
    if (BUSY_EXH == 1'b0) begin // Do not alter during exception processing.
        case (OP_SIZE)
            LONG: SIZEVAR = 2'b10;
            WORD: SIZEVAR = 2'b01;
            BYTE: SIZEVAR = 2'b00;
            default: SIZEVAR = 2'b00;
        endcase
        //
        if (BUS_CTRL_STATE == START_CYCLE && NEXT_BUS_CTRL_STATE == DATA_C1C4) begin
            SSW_80[8] <= 1'b0;
            SSW_80[7] <= RMC;
            SSW_80[6] <= ~WR_REQ;
            SSW_80[5:4] <= SIZEVAR;
            SSW_80[3] <= 1'b0;
            SSW_80[2:0] <= FC_IN;
        end else if (BUS_CTRL_STATE == DATA_C1C4 && (READ_ACCESS == 1'b1 || WRITE_ACCESS == 1'b1) && BUS_FLT == 1'b1) begin
            SSW_80[8] <= 1'b1;
        end

        OUTBUFFER <= WP_BUFFER; // Used for exception stack frame type A and B.
        INBUFFER <= DATA_INMUX; // Used for exception stack frame type B.
    end
end

always_ff @(posedge CLK) begin : WRITEBACK_INFO
    if (BUS_CTRL_STATE == BUS_IDLE && NEXT_BUS_CTRL_STATE == START_CYCLE) // Freeze during a bus cycle.
        WP_BUFFER <= DATA_FROM_CORE;
end

always_ff @(posedge CLK) begin : P_BUSWIDTH
    if (DSACK_In == 2'b01)
        DSACK_MEM <= 2'b01;
    else if (DSACK_In == 2'b10)
        DSACK_MEM <= 2'b10;
    else if (BUS_CTRL_STATE == BUS_IDLE)
        DSACK_MEM <= 2'b11;
end

assign BUS_WIDTH = (DSACKn == 2'b01 || DSACK_MEM == 2'b01) ? BW_WORD :
                   (DSACKn == 2'b10 || DSACK_MEM == 2'b10) ? BW_BYTE :
                   LONG_32; // Also used during synchronous cycles.

assign BUS_BSY = (BUS_CTRL_STATE != BUS_IDLE) ? 1'b1 : 1'b0;

always_ff @(posedge CLK) begin : PARTITIONING
    logic [2:0] RESTORE_VAR;

    if (BUS_CTRL_STATE == DATA_C1C4 && T_SLICE == S1) begin // On positive clock edge.
        RESTORE_VAR = SIZE_N; // We need this initial value for early RETRY.
    end else if (BUS_CTRL_STATE == DATA_C1C4 && ((T_SLICE == S1 && STERMn == 1'b0) || (T_SLICE == S3 && WAITSTATES == 1'b0))) begin
        RESTORE_VAR = SIZE_N; // This is for late RETRY.
    end
    //
    if (RESET_CPU_I == 1'b1) begin
        SIZE_N <= 3'b000;
    end else if (BUS_CTRL_STATE != DATA_C1C4 && NEXT_BUS_CTRL_STATE == DATA_C1C4) begin
        if (RD_REQ == 1'b1 || WR_REQ == 1'b1) begin
            case (OP_SIZE)
                LONG:    SIZE_N <= 3'b100;
                WORD:    SIZE_N <= 3'b010;
                BYTE:    SIZE_N <= 3'b001;
                default: SIZE_N <= 3'b001;
            endcase
        end else begin // OPCODE_ACCESS.
            SIZE_N <= 3'b010; // WORD.
        end
    end

    // Decrementing the size information:
    if (RETRY == 1'b1) begin
        SIZE_N <= RESTORE_VAR;
    end else if (BUS_CTRL_STATE == DATA_C1C4 && ((T_SLICE == S1 && STERMn == 1'b0) || (T_SLICE == S3 && WAITSTATES == 1'b0))) begin
        if (BUS_WIDTH == LONG_32 && SIZE_N > 3'd3 && ADR_OUT_I[1:0] == 2'b01)
            SIZE_N <= SIZE_N - 3'b011;
        else if (BUS_WIDTH == LONG_32 && SIZE_N > 3'd2 && ADR_OUT_I[1:0] == 2'b10)
            SIZE_N <= SIZE_N - 3'b010;
        else if (BUS_WIDTH == LONG_32 && SIZE_N > 3'd1 && ADR_OUT_I[1:0] == 2'b11)
            SIZE_N <= SIZE_N - 3'b001;
        else if (BUS_WIDTH == LONG_32)
            SIZE_N <= 3'b000;
        //
        else if (BUS_WIDTH == BW_WORD && ADR_OUT_I[1:0] == 2'b11)
            SIZE_N <= SIZE_N - 3'b001;
        else if (BUS_WIDTH == BW_WORD && ADR_OUT_I[1:0] == 2'b01)
            SIZE_N <= SIZE_N - 3'b001;
        else if (BUS_WIDTH == BW_WORD && SIZE_N == 3'b001)
            SIZE_N <= SIZE_N - 3'b001;
        else if (BUS_WIDTH == BW_WORD)
            SIZE_N <= SIZE_N - 3'b010;
        //
        else if (BUS_WIDTH == BW_BYTE)
            SIZE_N <= SIZE_N - 3'b001;
    end
    //
    if (BUS_FLT == 1'b1 && HALT_In == 1'b1) begin // Abort bus cycle.
        SIZE_N <= 3'b000;
        RESTORE_VAR = 3'b000;
    end
end

assign SIZE_I = (T_SLICE == S0 || T_SLICE == S1) ? SIZE_N[1:0] : SIZE_D;
assign SIZE = SIZE_I;

always_ff @(posedge CLK) begin : P_DELAY
    SIZE_D <= SIZE_N[1:0];
end

always_ff @(posedge CLK) begin : BUS_STATE_REG
    BUS_CTRL_STATE <= NEXT_BUS_CTRL_STATE;
end

always_comb begin : BUS_CTRL_DEC
    case (BUS_CTRL_STATE)
        BUS_IDLE: begin
            if (RESET_CPU_I == 1'b1)
                NEXT_BUS_CTRL_STATE = BUS_IDLE;
            else if (HALT_In == 1'b0)
                NEXT_BUS_CTRL_STATE = BUS_IDLE;
            else if ((BR_In == 1'b0 && RMC == 1'b0) || ARB_STATE != ARB_IDLE || BGACK_In == 1'b0)
                NEXT_BUS_CTRL_STATE = BUS_IDLE;
            else if (RD_REQ == 1'b1 && SIZE_N == 3'b000)
                NEXT_BUS_CTRL_STATE = START_CYCLE;
            else if (WR_REQ == 1'b1 && SIZE_N == 3'b000)
                NEXT_BUS_CTRL_STATE = START_CYCLE;
            else if (OPCODE_REQ == 1'b1 && SIZE_N == 3'b000)
                NEXT_BUS_CTRL_STATE = START_CYCLE;
            else if (READ_ACCESS == 1'b1 || WRITE_ACCESS == 1'b1 || OPCODE_ACCESS == 1'b1)
                NEXT_BUS_CTRL_STATE = START_CYCLE;
            else
                NEXT_BUS_CTRL_STATE = BUS_IDLE;
        end
        START_CYCLE: begin
            if (RD_REQ == 1'b1)
                NEXT_BUS_CTRL_STATE = DATA_C1C4;
            else if (WR_REQ == 1'b1)
                NEXT_BUS_CTRL_STATE = DATA_C1C4;
            else if (OPCODE_REQ == 1'b1 && ADR_IN_P[0] == 1'b1)
                NEXT_BUS_CTRL_STATE = BUS_IDLE; // Abort due to address error.
            else if (OPCODE_REQ == 1'b1 && ADR_IN_P[0] == 1'b1)
                NEXT_BUS_CTRL_STATE = BUS_IDLE; // Abort due to address error.
            else if (OPCODE_REQ == 1'b1)
                NEXT_BUS_CTRL_STATE = DATA_C1C4;
            else
                NEXT_BUS_CTRL_STATE = BUS_IDLE;
        end
        DATA_C1C4: begin
            if (BUS_CYC_RDY == 1'b1 && SIZE_N == 3'b000)
                NEXT_BUS_CTRL_STATE = BUS_IDLE;
            else
                NEXT_BUS_CTRL_STATE = DATA_C1C4;
        end
        default:
            NEXT_BUS_CTRL_STATE = BUS_IDLE;
    endcase
end

always_ff @(posedge CLK) begin : P_ADR_OFFS
    logic [2:0] OFFSET_VAR;

    if (RESET_CPU_I == 1'b1) begin
        OFFSET_VAR = 3'b000;
    end else if ((T_SLICE == S2 && STERMn == 1'b0) || T_SLICE == S3) begin
        case (BUS_WIDTH)
            LONG_32: begin
                case (ADR_OUT_I[1:0])
                    2'b11:   OFFSET_VAR = 3'b001;
                    2'b10:   OFFSET_VAR = 3'b010;
                    2'b01:   OFFSET_VAR = 3'b011;
                    default: OFFSET_VAR = 3'b100;
                endcase
            end
            BW_WORD: begin
                case (ADR_OUT_I[1:0])
                    2'b01, 2'b11: OFFSET_VAR = 3'b001;
                    default:      OFFSET_VAR = 3'b010;
                endcase
            end
            BW_BYTE:
                OFFSET_VAR = 3'b001;
            default:
                OFFSET_VAR = 3'b001;
        endcase
    end
    //
    if (RESET_CPU_I == 1'b1)
        ADR_OFFSET <= 6'b000000;
    else if (RETRY == 1'b1)
        ; // Do not update if there is a retry cycle.
    else if (BUS_CTRL_STATE != BUS_IDLE && NEXT_BUS_CTRL_STATE == BUS_IDLE)
        ADR_OFFSET <= 6'b000000;
    else if (BUS_CYC_RDY == 1'b1)
        ADR_OFFSET <= ADR_OFFSET + {3'b000, OFFSET_VAR};
end

assign ADR_OUT_I = ADR_IN_P + {26'd0, ADR_OFFSET};
assign ADR_OUT_P = ADR_OUT_I;

always_ff @(posedge CLK) begin : P_ADR_10
    ADR_10 <= ADR_OUT_I[1:0];
end

// Address and bus errors:
assign AERR_I = (BUS_CTRL_STATE == START_CYCLE && OPCODE_REQ == 1'b1 && RD_REQ == 1'b0 && WR_REQ == 1'b0 && ADR_IN_P[0] == 1'b1) ? 1'b1 : 1'b0;

assign FC_OUT = FC_IN;

// Data output multiplexer.
always_comb begin
    // LONG:
    if (SIZE_I == 2'b00 && ADR_OUT_I[1:0] == 2'b00)
        DATA_PORT_OUT = WP_BUFFER[31:0];
    else if (SIZE_I == 2'b00 && ADR_OUT_I[1:0] == 2'b01)
        DATA_PORT_OUT = {WP_BUFFER[31:24], WP_BUFFER[31:8]};
    else if (SIZE_I == 2'b00 && ADR_OUT_I[1:0] == 2'b10)
        DATA_PORT_OUT = {WP_BUFFER[31:16], WP_BUFFER[31:16]};
    else if (SIZE_I == 2'b00 && ADR_OUT_I[1:0] == 2'b11)
        DATA_PORT_OUT = {WP_BUFFER[31:24], WP_BUFFER[31:16], WP_BUFFER[31:24]};
    // 3 bytes:
    else if (SIZE_I == 2'b11 && ADR_OUT_I[1:0] == 2'b00)
        DATA_PORT_OUT = {WP_BUFFER[23:0], WP_BUFFER[23:16]};
    else if (SIZE_I == 2'b11 && ADR_OUT_I[1:0] == 2'b01)
        DATA_PORT_OUT = {WP_BUFFER[23:16], WP_BUFFER[23:0]};
    else if (SIZE_I == 2'b11 && ADR_OUT_I[1:0] == 2'b10)
        DATA_PORT_OUT = {WP_BUFFER[23:8], WP_BUFFER[23:8]};
    else if (SIZE_I == 2'b11 && ADR_OUT_I[1:0] == 2'b11)
        DATA_PORT_OUT = {WP_BUFFER[23:16], WP_BUFFER[23:8], WP_BUFFER[23:16]};
    // Word:
    else if (SIZE_I == 2'b10 && ADR_OUT_I[1:0] == 2'b00)
        DATA_PORT_OUT = {WP_BUFFER[15:0], WP_BUFFER[15:0]};
    else if (SIZE_I == 2'b10 && ADR_OUT_I[1:0] == 2'b01)
        DATA_PORT_OUT = {WP_BUFFER[15:8], WP_BUFFER[15:0], WP_BUFFER[15:8]};
    else if (SIZE_I == 2'b10 && ADR_OUT_I[1:0] == 2'b10)
        DATA_PORT_OUT = {WP_BUFFER[15:0], WP_BUFFER[15:0]};
    else if (SIZE_I == 2'b10 && ADR_OUT_I[1:0] == 2'b11)
        DATA_PORT_OUT = {WP_BUFFER[15:8], WP_BUFFER[15:0], WP_BUFFER[15:8]};
    // Byte:
    else // SIZE = "01"
        DATA_PORT_OUT = {WP_BUFFER[7:0], WP_BUFFER[7:0], WP_BUFFER[7:0], WP_BUFFER[7:0]};
end

always_ff @(negedge CLK) begin : IN_MUX
    if (((T_SLICE == S2 || T_SLICE == S3) && STERMn == 1'b0) || T_SLICE == S4) begin
        case (BUS_WIDTH)
            BW_BYTE: begin
                case (SIZE_I)
                    2'b00: DATA_INMUX[31:24] <= DATA_PORT_IN[31:24]; // LONG.
                    2'b11: DATA_INMUX[23:16] <= DATA_PORT_IN[31:24]; // Three bytes.
                    2'b10: DATA_INMUX[15:8]  <= DATA_PORT_IN[31:24]; // Word.
                    default: DATA_INMUX[7:0] <= DATA_PORT_IN[31:24]; // Byte.
                endcase
            end
            BW_WORD: begin
                case (SIZE_I)
                    2'b01: begin // Byte.
                        case (ADR_10)
                            2'b00, 2'b10: DATA_INMUX[7:0] <= DATA_PORT_IN[31:24];
                            default:      DATA_INMUX[7:0] <= DATA_PORT_IN[23:16]; // "01", "11".
                        endcase
                    end
                    2'b10: begin // Word.
                        case (ADR_10)
                            2'b00: DATA_INMUX[15:0]  <= DATA_PORT_IN[31:16];
                            2'b01: DATA_INMUX[15:8]  <= DATA_PORT_IN[23:16];
                            2'b10: DATA_INMUX[15:0]  <= DATA_PORT_IN[31:16];
                            default: DATA_INMUX[15:8] <= DATA_PORT_IN[23:16]; // "11".
                        endcase
                    end
                    2'b11: begin // Three bytes.
                        case (ADR_10)
                            2'b00: DATA_INMUX[23:8]  <= DATA_PORT_IN[31:16];
                            2'b01: DATA_INMUX[23:16] <= DATA_PORT_IN[23:16];
                            2'b10: DATA_INMUX[23:8]  <= DATA_PORT_IN[31:16];
                            default: DATA_INMUX[23:16] <= DATA_PORT_IN[23:16]; // "11".
                        endcase
                    end
                    default: begin // "00" = LONG.
                        case (ADR_10)
                            2'b00: DATA_INMUX[31:16] <= DATA_PORT_IN[31:16];
                            2'b01: DATA_INMUX[31:24] <= DATA_PORT_IN[23:16];
                            2'b10: DATA_INMUX[31:16] <= DATA_PORT_IN[31:16];
                            default: DATA_INMUX[31:24] <= DATA_PORT_IN[23:16]; // "11".
                        endcase
                    end
                endcase
            end
            LONG_32: begin
                case (SIZE_I)
                    2'b01: begin // Byte.
                        case (ADR_10)
                            2'b00:   DATA_INMUX[7:0] <= DATA_PORT_IN[31:24];
                            2'b01:   DATA_INMUX[7:0] <= DATA_PORT_IN[23:16];
                            2'b10:   DATA_INMUX[7:0] <= DATA_PORT_IN[15:8];
                            default: DATA_INMUX[7:0] <= DATA_PORT_IN[7:0]; // "11".
                        endcase
                    end
                    2'b10: begin // Word.
                        case (ADR_10)
                            2'b00:   DATA_INMUX[15:0] <= DATA_PORT_IN[31:16];
                            2'b01:   DATA_INMUX[15:0] <= DATA_PORT_IN[23:8];
                            2'b10:   DATA_INMUX[15:0] <= DATA_PORT_IN[15:0];
                            default: DATA_INMUX[15:8] <= DATA_PORT_IN[7:0]; // "11".
                        endcase
                    end
                    2'b11: begin // Three bytes.
                        case (ADR_10)
                            2'b00:   DATA_INMUX[23:0]  <= DATA_PORT_IN[31:8];
                            2'b01:   DATA_INMUX[23:0]  <= DATA_PORT_IN[23:0];
                            2'b10:   DATA_INMUX[23:8]  <= DATA_PORT_IN[15:0];
                            default: DATA_INMUX[23:16] <= DATA_PORT_IN[7:0]; // "11".
                        endcase
                    end
                    default: begin // "00" = LONG.
                        case (ADR_10)
                            2'b00:   DATA_INMUX[31:0]  <= DATA_PORT_IN[31:0];
                            2'b01:   DATA_INMUX[31:8]  <= DATA_PORT_IN[23:0];
                            2'b10:   DATA_INMUX[31:16] <= DATA_PORT_IN[15:0];
                            default: DATA_INMUX[31:24] <= DATA_PORT_IN[7:0]; // "11".
                        endcase
                    end
                endcase
            end
            default: ;
        endcase
    end
end

always_ff @(posedge CLK) begin : VALIDATION
    if (RESET_CPU_I == 1'b1)
        OPCODE_VALID <= 1'b1;
    else if (OPCODE_ACCESS == 1'b1 && BUS_CTRL_STATE == DATA_C1C4 && BUS_FLT == 1'b1)
        OPCODE_VALID <= 1'b0;
    else if (OPCODE_RDY_I == 1'b1)
        OPCODE_VALID <= 1'b1;
    //
    if (RESET_CPU_I == 1'b1)
        DATA_VALID <= 1'b1;
    else if (READ_ACCESS == 1'b1 && BUS_CTRL_STATE == DATA_C1C4 && BUS_FLT == 1'b1 && HALT_In == 1'b0)
        ; // This is the RETRY condition, no bus error.
    else if (BUS_CTRL_STATE == DATA_C1C4 && BUS_FLT == 1'b1)
        DATA_VALID <= 1'b0;
    else if (DATA_RDY_I == 1'b1)
        DATA_VALID <= 1'b1;
end

always_ff @(posedge CLK) begin : PREFETCH_BUFFERS
    logic RDY_VAR;
    //
    OPCODE_RDY_I <= 1'b0; // This is a strobe.
    DATA_RDY_I <= 1'b0; // This is a strobe.
    //
    if (DATA_RDY_I == 1'b1 || OPCODE_RDY_I == 1'b1)
        RDY_VAR = 1'b0;
    else if (BUS_CTRL_STATE == START_CYCLE)
        RDY_VAR = 1'b1;
    // Opcode cycle:
    if (AERR_I == 1'b1)
        OPCODE_RDY_I <= 1'b1;
    else if (OPCODE_ACCESS == 1'b1 && BUS_CTRL_STATE == DATA_C1C4 && BUS_CYC_RDY == 1'b1 && SIZE_N == 3'b000) begin
        OBUFFER <= DATA_INMUX[15:0];
        OPCODE_RDY_I <= RDY_VAR;
    end
    // Data cycle:
    if (WRITE_ACCESS == 1'b1 && BUS_CTRL_STATE == DATA_C1C4 && BUS_CYC_RDY == 1'b1 && SIZE_N == 3'b000) begin
        DATA_RDY_I <= RDY_VAR;
    end else if (READ_ACCESS == 1'b1 && BUS_CTRL_STATE == DATA_C1C4 && BUS_CYC_RDY == 1'b1) begin
        case (OP_SIZE)
            LONG: begin
                if (SIZE_N == 3'b000) begin
                    DBUFFER <= DATA_INMUX;
                    DATA_RDY_I <= RDY_VAR;
                end
            end
            WORD: begin
                if (SIZE_N == 3'b000) begin
                    DBUFFER <= {16'h0000, DATA_INMUX[15:0]};
                    DATA_RDY_I <= RDY_VAR;
                end
            end
            BYTE: begin // Byte always aligned.
                DATA_RDY_I <= RDY_VAR;
                DBUFFER <= {24'h000000, DATA_INMUX[7:0]};
            end
            default: ;
        endcase
    end
end

assign DATA_RDY = DATA_RDY_I;
assign OPCODE_RDY = OPCODE_RDY_I;

assign DATA_TO_CORE = DBUFFER;
assign OPCODE_TO_CORE = OBUFFER;

assign WAITSTATES = (T_SLICE != S3) ? 1'b0 :
                    (RESET_OUT_I == 1'b1) ? 1'b1 : // No bus fault during RESET instruction.
                    (DSACK_In != 2'b11) ? 1'b0 : // For asynchronous bus cycles.
                    (STERM_In == 1'b0) ? 1'b0 : // For synchronous bus cycles.
                    (ADR_IN_P[19:16] == 4'hF && AVEC_In == 1'b0) ? 1'b0 : // Interrupt acknowledge space cycle.
                    (BUS_FLT == 1'b1) ? 1'b0 : // In case of a bus error.
                    (RESET_CPU_I == 1'b1) ? 1'b0 : 1'b1; // A CPU reset terminates the current bus cycle.

always_ff @(posedge CLK) begin
    if (BUS_CTRL_STATE == BUS_IDLE)
        SLICE_CNT_P <= 3'b111; // Init.
    else if (RETRY == 1'b1)
        SLICE_CNT_P <= 3'b111;
    else if (BUS_CTRL_STATE != BUS_IDLE && NEXT_BUS_CTRL_STATE == BUS_IDLE)
        SLICE_CNT_P <= 3'b111; // Init.
    else if (SLICE_CNT_P == 3'b001 && STERM_In == 1'b0) // Synchronous cycle.
        SLICE_CNT_P <= 3'b111; // Ready.
    else if (SLICE_CNT_P == 3'b010) begin
        if (RETRY == 1'b1)
            SLICE_CNT_P <= 3'b111; // Go IDLE.
        else if (BUS_CTRL_STATE == DATA_C1C4 && NEXT_BUS_CTRL_STATE == BUS_IDLE)
            SLICE_CNT_P <= 3'b111; // Ready.
        else
            SLICE_CNT_P <= 3'b000; // Go on.
    end else if (WAITSTATES == 1'b0)
        SLICE_CNT_P <= SLICE_CNT_P + 1'b1; // Cycle active.
end

always_ff @(negedge CLK) begin
    SLICE_CNT_N <= SLICE_CNT_P; // Follow the P counter.
end

assign T_SLICE = (SLICE_CNT_P == 3'b000 && SLICE_CNT_N == 3'b111) ? S0 :
                 (SLICE_CNT_P == 3'b000 && SLICE_CNT_N == 3'b000) ? S1 :
                 (SLICE_CNT_P == 3'b001 && SLICE_CNT_N == 3'b000) ? S2 :
                 (SLICE_CNT_P == 3'b001 && SLICE_CNT_N == 3'b001) ? S3 :
                 (SLICE_CNT_P == 3'b010 && SLICE_CNT_N == 3'b001) ? S4 :
                 (SLICE_CNT_P == 3'b010 && SLICE_CNT_N == 3'b010) ? S5 :
                 (SLICE_CNT_P == 3'b000 && SLICE_CNT_N == 3'b010) ? S0 : SLICE_IDLE; // Rollover from state S5 to S0.

always_ff @(posedge CLK) begin : P_OCS
    if (BUS_CTRL_STATE == START_CYCLE && NEXT_BUS_CTRL_STATE != BUS_IDLE)
        OCS_INH <= 1'b0;
    else if (BUS_CYC_RDY == 1'b1 && RETRY == 1'b0) // No inhibit if first portion results in a retry cycle.
        OCS_INH <= 1'b1;
end

// Bus control signals:
assign RWn  = (WRITE_ACCESS == 1'b1 && BUS_CTRL_STATE == DATA_C1C4) ? 1'b0 : 1'b1;
assign RMCn = (RMC == 1'b1) ? 1'b0 : 1'b1;
assign ECSn = (T_SLICE == S0) ? 1'b0 : 1'b1;
assign OCSn = (T_SLICE == S0 && OCS_INH == 1'b0) ? 1'b0 : 1'b1;
assign ASn  = (T_SLICE == S0 || T_SLICE == S1 || T_SLICE == S2 || T_SLICE == S3 || T_SLICE == S4) ? 1'b0 : 1'b1;
assign DSn  = ((T_SLICE == S3 || T_SLICE == S4 || T_SLICE == S5) && WRITE_ACCESS == 1'b1) ? 1'b0 : // Write.
              (T_SLICE == S0 || T_SLICE == S1 || T_SLICE == S2 || T_SLICE == S3 || T_SLICE == S4) ? 1'b0 : 1'b1; // Read.

assign DBENn = ((T_SLICE == S1 || T_SLICE == S2 || T_SLICE == S3 || T_SLICE == S4 || T_SLICE == S5) && WRITE_ACCESS == 1'b1) ? 1'b0 : // Write.
               (T_SLICE == S2 || T_SLICE == S3 || T_SLICE == S4) ? 1'b0 : 1'b1; // Read.

// Bus tri state controls:
assign BUS_EN       = (ARB_STATE == ARB_IDLE && RESET_CPU_I == 1'b0) ? 1'b1 : 1'b0;
assign DATA_PORT_EN = (WRITE_ACCESS == 1'b1 && ARB_STATE == ARB_IDLE && RESET_CPU_I == 1'b0) ? 1'b1 : 1'b0;

// Progress controls:
assign BUS_CYC_RDY = (RETRY == 1'b1) ? 1'b0 :
                     (T_SLICE == S3 && STERM_In == 1'b0) ? 1'b1 : // Synchronous cycles.
                     (T_SLICE == S5) ? 1'b1 : 1'b0; // Asynchronous cycles.

// Bus arbitration:
always_ff @(posedge CLK) begin : ARB_REG
    if (RESET_CPU_I == 1'b1)
        ARB_STATE <= ARB_IDLE;
    else
        ARB_STATE <= NEXT_ARB_STATE;
end

always_comb begin : ARB_DEC
    case (ARB_STATE)
        ARB_IDLE: begin
            if (RMC == 1'b1 && RETRY == 1'b0)
                NEXT_ARB_STATE = ARB_IDLE;
            else if (BGACK_In == 1'b0 && BUS_CTRL_STATE == BUS_IDLE)
                NEXT_ARB_STATE = WAIT_RELEASE_3WIRE;
            else if (BR_In == 1'b0 && BUS_CTRL_STATE == BUS_IDLE)
                NEXT_ARB_STATE = GRANT;
            else
                NEXT_ARB_STATE = ARB_IDLE;
        end
        GRANT: begin
            if (BGACK_In == 1'b0)
                NEXT_ARB_STATE = WAIT_RELEASE_3WIRE;
            else if (BR_In == 1'b1)
                NEXT_ARB_STATE = ARB_IDLE;
            else
                NEXT_ARB_STATE = GRANT;
        end
        WAIT_RELEASE_3WIRE: begin
            if (BGACK_In == 1'b1 && BR_In == 1'b0)
                NEXT_ARB_STATE = GRANT;
            else if (BGACK_In == 1'b1)
                NEXT_ARB_STATE = ARB_IDLE;
            else
                NEXT_ARB_STATE = WAIT_RELEASE_3WIRE;
        end
        default:
            NEXT_ARB_STATE = ARB_IDLE;
    endcase
end

assign BGn = (ARB_STATE == GRANT) ? 1'b0 : 1'b1;

// RESET logic:
always_ff @(posedge CLK) begin : RESET_FILTER
    logic STARTUP;
    logic [3:0] TMP;

    if (RESET_IN == 1'b1 && HALT_In == 1'b0 && RESET_OUT_I == 1'b0 && TMP < 4'hF)
        TMP = TMP + 1'b1;
    else if (RESET_IN == 1'b0 || HALT_In == 1'b1 || RESET_OUT_I == 1'b1)
        TMP = 4'h0;

    if (TMP > 4'hA) begin
        RESET_CPU_I <= 1'b1;
        STARTUP = 1'b1;
    end else if (STARTUP == 1'b0) begin
        RESET_CPU_I <= 1'b1;
    end else begin
        RESET_CPU_I <= 1'b0;
    end
end

always_ff @(posedge CLK) begin : RESET_TIMER
    logic [8:0] TMP;

    if (RESET_STRB == 1'b1 || TMP > 9'b000000000)
        RESET_OUT_I <= 1'b1;
    else
        RESET_OUT_I <= 1'b0;

    if (RESET_STRB == 1'b1)
        TMP = 9'b111111111; // 512 initial value.
    else if (TMP > 9'b000000000)
        TMP = TMP - 1'b1;
end

assign RESET_CPU = RESET_CPU_I;
assign RESET_OUT = RESET_OUT_I;

endmodule
