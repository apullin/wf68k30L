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

// ---- Bus controller state machine ----
typedef enum logic [1:0] {BUS_IDLE, START_CYCLE, DATA_C1C4} BUS_CTRL_STATES;

// ---- Bus arbitration state machine ----
typedef enum logic [1:0] {ARB_IDLE, GRANT, WAIT_RELEASE_3WIRE} ARB_STATES;

// ---- Port width classification ----
typedef enum logic [1:0] {LONG_32, BW_WORD, BW_BYTE} BUS_WIDTH_TYPE;

// ---- Bus timing slices ----
// The MC68030 bus cycle is divided into six half-clock slices (S0-S5):
//   S0: External cycle start (ECS asserted), address driven
//   S1: Address strobe (AS) asserted, data strobe (DS) for reads
//   S2: Synchronous termination sample point
//   S3: Wait-state insertion point; DSACKn/STERMn sampled here
//   S4: Data latched (read) or held (write); AS/DS still active
//   S5: Bus cycle ends; AS/DS deasserted; next S0 may begin immediately
typedef enum logic [2:0] {SLICE_IDLE, S0, S1, S2, S3, S4, S5} TIME_SLICES;

logic [1:0]         ADR_10;
logic [5:0]         ADR_OFFSET;
logic [2:0]         ADR_STEP;
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
logic [2:0]         SIZE_RESTORE;
logic               RDY_ARMED;
logic               SLICE_S0_TO_S4;
logic               SLICE_S1_TO_S5;
logic               SLICE_S2_TO_S4;
logic               SLICE_S3_TO_S5;

// ---- Synchronize bus termination signals on negative clock edge ----
logic BUS_FLT_VAR;

always_ff @(negedge CLK) begin : sync_bus_termination
    DSACK_In <= DSACKn;
    HALT_In <= HALTn;
    BUS_FLT_VAR <= ~BERRn;
    STERM_In <= STERMn;
    BR_In <= BRn;
    BGACK_In <= BGACKn;
    AVEC_In <= AVECn;

    // Retry detection: BERR + HALT during active bus cycle with remaining transfers.
    if (!BERRn && !HALTn && BUS_CTRL_STATE == DATA_C1C4 && SIZE_N != 3'b000)
        RETRY <= 1'b1;
    else if (T_SLICE == SLICE_IDLE && (BERRn || HALTn))
        RETRY <= 1'b0;
end

// ---- Latch bus fault and address error on posedge ----
always_ff @(posedge CLK) begin : fault_latch
    if (BUS_CTRL_STATE == START_CYCLE)
        AERR <= AERR_I;
    else if (BUS_CTRL_STATE == DATA_C1C4)
        BUS_FLT <= BUS_FLT_VAR;
    else begin
        BUS_FLT <= 1'b0;
        AERR <= 1'b0;
    end
end

// ---- Access type tracking ----
// Latches which type of access (read/write/opcode) is active during a bus cycle.
always_ff @(posedge CLK) begin : access_type
    if (BUS_CTRL_STATE == START_CYCLE) begin
        if (READ_ACCESS || WRITE_ACCESS || OPCODE_ACCESS)
            ; // Do not start either new cycle.
        else if (RD_REQ)
            READ_ACCESS <= 1'b1;
        else if (WR_REQ)
            WRITE_ACCESS <= 1'b1;
        else if (OPCODE_REQ)
            OPCODE_ACCESS <= 1'b1;
    end else if (AERR) begin // Reject due to address error.
        READ_ACCESS <= 1'b0;
        WRITE_ACCESS <= 1'b0;
        OPCODE_ACCESS <= 1'b0;
    end else if (BUS_CTRL_STATE == DATA_C1C4 && NEXT_BUS_CTRL_STATE == BUS_IDLE && SIZE_N == 3'b000) begin
        READ_ACCESS <= 1'b0;
        WRITE_ACCESS <= 1'b0;
        OPCODE_ACCESS <= 1'b0;
    end
end

// ---- Special Status Word (SSW) and exception buffers ----
// Captures bus cycle attributes for format A/B stack frames on bus faults.
always_ff @(posedge CLK) begin : data_fault_info
    logic [1:0] SIZEVAR;
    if (!BUSY_EXH) begin // Do not alter during exception processing.
        case (OP_SIZE)
            LONG:    SIZEVAR = 2'b10;
            WORD:    SIZEVAR = 2'b01;
            BYTE:    SIZEVAR = 2'b00;
            default: SIZEVAR = 2'b00;
        endcase

        if (BUS_CTRL_STATE == START_CYCLE && NEXT_BUS_CTRL_STATE == DATA_C1C4) begin
            SSW_80[8] <= 1'b0;
            SSW_80[7] <= RMC;
            SSW_80[6] <= ~WR_REQ;
            SSW_80[5:4] <= SIZEVAR;
            SSW_80[3] <= 1'b0;
            SSW_80[2:0] <= FC_IN;
        end else if (BUS_CTRL_STATE == DATA_C1C4 && (READ_ACCESS || WRITE_ACCESS) && BUS_FLT) begin
            SSW_80[8] <= 1'b1;
        end

        OUTBUFFER <= WP_BUFFER; // Used for exception stack frame type A and B.
        INBUFFER <= DATA_INMUX; // Used for exception stack frame type B.
    end
end

// ---- Write buffer latch ----
// Captures data from core at bus cycle start so it remains stable throughout.
always_ff @(posedge CLK) begin : writeback_info
    if (BUS_CTRL_STATE == BUS_IDLE && NEXT_BUS_CTRL_STATE == START_CYCLE)
        WP_BUFFER <= DATA_FROM_CORE;
end

// ---- Bus width detection ----
// Remembers the responding port width (32-bit, 16-bit, or 8-bit) from DSACKn.
always_ff @(posedge CLK) begin : bus_width_latch
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

assign BUS_BSY = (BUS_CTRL_STATE != BUS_IDLE);

// ---- Transfer size partitioning ----
// Tracks remaining bytes to transfer across multiple bus cycles when the
// port width is narrower than the operand size. SIZE_N counts down to zero.
always_ff @(posedge CLK) begin : partitioning
    if (RESET_CPU_I || (BUS_FLT && HALT_In))
        SIZE_RESTORE <= 3'b000;
    else if (BUS_CTRL_STATE == DATA_C1C4 && (T_SLICE == S1 || (T_SLICE == S3 && !WAITSTATES)))
        SIZE_RESTORE <= SIZE_N;

    if (RESET_CPU_I) begin
        SIZE_N <= 3'b000;
    end else if (BUS_CTRL_STATE != DATA_C1C4 && NEXT_BUS_CTRL_STATE == DATA_C1C4) begin
        // Initialize transfer size from operand size.
        if (RD_REQ || WR_REQ) begin
            case (OP_SIZE)
                LONG:    SIZE_N <= 3'b100;
                WORD:    SIZE_N <= 3'b010;
                BYTE:    SIZE_N <= 3'b001;
                default: SIZE_N <= 3'b001;
            endcase
        end else begin // OPCODE_ACCESS.
            SIZE_N <= 3'b010; // Opcodes are always word-sized.
        end
    end

    // Decrement remaining size based on how many bytes this cycle transferred.
    if (RETRY) begin
        SIZE_N <= SIZE_RESTORE;
    end else if (BUS_CTRL_STATE == DATA_C1C4 && ((T_SLICE == S1 && !STERMn) || (T_SLICE == S3 && !WAITSTATES))) begin
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

    if (BUS_FLT && HALT_In) begin // Abort bus cycle on unrecoverable fault.
        SIZE_N <= 3'b000;
    end
end

assign SIZE_I = (T_SLICE == S0 || T_SLICE == S1) ? SIZE_N[1:0] : SIZE_D;
assign SIZE = SIZE_I;

always_ff @(posedge CLK) begin : size_delay
    SIZE_D <= SIZE_N[1:0];
end

// ---- Bus controller state register ----
always_ff @(posedge CLK) begin : bus_state_reg
    BUS_CTRL_STATE <= NEXT_BUS_CTRL_STATE;
end

// ---- Bus controller next-state logic ----
always_comb begin : bus_ctrl_dec
    case (BUS_CTRL_STATE)
        BUS_IDLE: begin
            if (RESET_CPU_I)
                NEXT_BUS_CTRL_STATE = BUS_IDLE;
            else if (!HALT_In)
                NEXT_BUS_CTRL_STATE = BUS_IDLE;
            else if ((!BR_In && !RMC) || ARB_STATE != ARB_IDLE || !BGACK_In)
                NEXT_BUS_CTRL_STATE = BUS_IDLE;
            else if (RD_REQ && SIZE_N == 3'b000)
                NEXT_BUS_CTRL_STATE = START_CYCLE;
            else if (WR_REQ && SIZE_N == 3'b000)
                NEXT_BUS_CTRL_STATE = START_CYCLE;
            else if (OPCODE_REQ && SIZE_N == 3'b000)
                NEXT_BUS_CTRL_STATE = START_CYCLE;
            else if (READ_ACCESS || WRITE_ACCESS || OPCODE_ACCESS)
                NEXT_BUS_CTRL_STATE = START_CYCLE;
            else
                NEXT_BUS_CTRL_STATE = BUS_IDLE;
        end
        START_CYCLE: begin
            if (RD_REQ)
                NEXT_BUS_CTRL_STATE = DATA_C1C4;
            else if (WR_REQ)
                NEXT_BUS_CTRL_STATE = DATA_C1C4;
            else if (OPCODE_REQ && ADR_IN_P[0])
                NEXT_BUS_CTRL_STATE = BUS_IDLE; // Abort due to address error.
            else if (OPCODE_REQ)
                NEXT_BUS_CTRL_STATE = DATA_C1C4;
            else
                NEXT_BUS_CTRL_STATE = BUS_IDLE;
        end
        DATA_C1C4: begin
            if (BUS_CYC_RDY && SIZE_N == 3'b000)
                NEXT_BUS_CTRL_STATE = BUS_IDLE;
            else
                NEXT_BUS_CTRL_STATE = DATA_C1C4;
        end
        default:
            NEXT_BUS_CTRL_STATE = BUS_IDLE;
    endcase
end

// ---- Address offset accumulator ----
// Tracks the byte offset within a multi-cycle transfer for dynamic bus sizing.
always_ff @(posedge CLK) begin : adr_offset_calc
    if (RESET_CPU_I) begin
        ADR_STEP <= 3'b000;
    end else if ((T_SLICE == S2 && !STERMn) || T_SLICE == S3) begin
        case (BUS_WIDTH)
            LONG_32: begin
                case (ADR_OUT_I[1:0])
                    2'b11:   ADR_STEP <= 3'b001;
                    2'b10:   ADR_STEP <= 3'b010;
                    2'b01:   ADR_STEP <= 3'b011;
                    default: ADR_STEP <= 3'b100;
                endcase
            end
            BW_WORD: begin
                case (ADR_OUT_I[1:0])
                    2'b01, 2'b11: ADR_STEP <= 3'b001;
                    default:      ADR_STEP <= 3'b010;
                endcase
            end
            BW_BYTE:
                ADR_STEP <= 3'b001;
            default:
                ADR_STEP <= 3'b001;
        endcase
    end

    if (RESET_CPU_I)
        ADR_OFFSET <= 6'd0;
    else if (RETRY)
        ; // Do not update if there is a retry cycle.
    else if (BUS_CTRL_STATE != BUS_IDLE && NEXT_BUS_CTRL_STATE == BUS_IDLE)
        ADR_OFFSET <= 6'd0;
    else if (BUS_CYC_RDY)
        ADR_OFFSET <= ADR_OFFSET + {3'b000, ADR_STEP};
end

assign ADR_OUT_I = ADR_IN_P + {26'd0, ADR_OFFSET};
assign ADR_OUT_P = ADR_OUT_I;

always_ff @(posedge CLK) begin : adr_10_latch
    ADR_10 <= ADR_OUT_I[1:0];
end

// ---- Address error detection ----
// Odd address on opcode fetch (word-aligned access required).
assign AERR_I = BUS_CTRL_STATE == START_CYCLE && OPCODE_REQ && !RD_REQ && !WR_REQ && ADR_IN_P[0];

assign FC_OUT = FC_IN;

// ---- Data output alignment mux ----
// Routes core write data onto the external data bus according to the
// current SIZE and address alignment. The 68030 replicates data bytes
// across the bus so that the target device sees valid data on its port
// regardless of address alignment.
always_comb begin : data_out_alignment
    case ({SIZE_I, ADR_OUT_I[1:0]})
        // LONG (SIZE=00): full 32-bit transfer
        {2'b00, 2'b00}: DATA_PORT_OUT = WP_BUFFER[31:0];
        {2'b00, 2'b01}: DATA_PORT_OUT = {WP_BUFFER[31:24], WP_BUFFER[31:8]};
        {2'b00, 2'b10}: DATA_PORT_OUT = {WP_BUFFER[31:16], WP_BUFFER[31:16]};
        {2'b00, 2'b11}: DATA_PORT_OUT = {WP_BUFFER[31:24], WP_BUFFER[31:16], WP_BUFFER[31:24]};
        // 3 bytes (SIZE=11)
        {2'b11, 2'b00}: DATA_PORT_OUT = {WP_BUFFER[23:0], WP_BUFFER[23:16]};
        {2'b11, 2'b01}: DATA_PORT_OUT = {WP_BUFFER[23:16], WP_BUFFER[23:0]};
        {2'b11, 2'b10}: DATA_PORT_OUT = {WP_BUFFER[23:8], WP_BUFFER[23:8]};
        {2'b11, 2'b11}: DATA_PORT_OUT = {WP_BUFFER[23:16], WP_BUFFER[23:8], WP_BUFFER[23:16]};
        // Word (SIZE=10)
        {2'b10, 2'b00}: DATA_PORT_OUT = {WP_BUFFER[15:0], WP_BUFFER[15:0]};
        {2'b10, 2'b01}: DATA_PORT_OUT = {WP_BUFFER[15:8], WP_BUFFER[15:0], WP_BUFFER[15:8]};
        {2'b10, 2'b10}: DATA_PORT_OUT = {WP_BUFFER[15:0], WP_BUFFER[15:0]};
        {2'b10, 2'b11}: DATA_PORT_OUT = {WP_BUFFER[15:8], WP_BUFFER[15:0], WP_BUFFER[15:8]};
        // Byte (SIZE=01): replicate across all lanes
        default:         DATA_PORT_OUT = {WP_BUFFER[7:0], WP_BUFFER[7:0], WP_BUFFER[7:0], WP_BUFFER[7:0]};
    endcase
end

// ---- Data input alignment mux ----
// Routes external data bus bytes into the correct position within the
// internal data register, based on bus width, transfer size, and address
// alignment. Registered on negedge to capture data at the end of S4.
always_ff @(negedge CLK) begin : data_in_alignment
    if (((T_SLICE == S2 || T_SLICE == S3) && !STERMn) || T_SLICE == S4) begin
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
                            default:      DATA_INMUX[7:0] <= DATA_PORT_IN[23:16];
                        endcase
                    end
                    2'b10: begin // Word.
                        case (ADR_10)
                            2'b00: DATA_INMUX[15:0]  <= DATA_PORT_IN[31:16];
                            2'b01: DATA_INMUX[15:8]  <= DATA_PORT_IN[23:16];
                            2'b10: DATA_INMUX[15:0]  <= DATA_PORT_IN[31:16];
                            default: DATA_INMUX[15:8] <= DATA_PORT_IN[23:16];
                        endcase
                    end
                    2'b11: begin // Three bytes.
                        case (ADR_10)
                            2'b00: DATA_INMUX[23:8]  <= DATA_PORT_IN[31:16];
                            2'b01: DATA_INMUX[23:16] <= DATA_PORT_IN[23:16];
                            2'b10: DATA_INMUX[23:8]  <= DATA_PORT_IN[31:16];
                            default: DATA_INMUX[23:16] <= DATA_PORT_IN[23:16];
                        endcase
                    end
                    default: begin // LONG.
                        case (ADR_10)
                            2'b00: DATA_INMUX[31:16] <= DATA_PORT_IN[31:16];
                            2'b01: DATA_INMUX[31:24] <= DATA_PORT_IN[23:16];
                            2'b10: DATA_INMUX[31:16] <= DATA_PORT_IN[15:0];
                            default: DATA_INMUX[31:24] <= DATA_PORT_IN[23:16];
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
                            default: DATA_INMUX[7:0] <= DATA_PORT_IN[7:0];
                        endcase
                    end
                    2'b10: begin // Word.
                        case (ADR_10)
                            2'b00:   DATA_INMUX[15:0] <= DATA_PORT_IN[31:16];
                            2'b01:   DATA_INMUX[15:0] <= DATA_PORT_IN[23:8];
                            2'b10:   DATA_INMUX[15:0] <= DATA_PORT_IN[15:0];
                            default: DATA_INMUX[15:8] <= DATA_PORT_IN[7:0];
                        endcase
                    end
                    2'b11: begin // Three bytes.
                        case (ADR_10)
                            2'b00:   DATA_INMUX[23:0]  <= DATA_PORT_IN[31:8];
                            2'b01:   DATA_INMUX[23:0]  <= DATA_PORT_IN[23:0];
                            2'b10:   DATA_INMUX[23:8]  <= DATA_PORT_IN[15:0];
                            default: DATA_INMUX[23:16] <= DATA_PORT_IN[7:0];
                        endcase
                    end
                    default: begin // LONG.
                        case (ADR_10)
                            2'b00:   DATA_INMUX[31:0]  <= DATA_PORT_IN[31:0];
                            2'b01:   DATA_INMUX[31:8]  <= DATA_PORT_IN[31:8];
                            2'b10:   DATA_INMUX[31:16] <= DATA_PORT_IN[31:16];
                            default: DATA_INMUX[31:24] <= DATA_PORT_IN[31:24];
                        endcase
                    end
                endcase
            end
            default: ;
        endcase
    end
end

// ---- Data/opcode validity tracking ----
always_ff @(posedge CLK) begin : validation
    if (RESET_CPU_I)
        OPCODE_VALID <= 1'b1;
    else if (OPCODE_ACCESS && BUS_CTRL_STATE == DATA_C1C4 && BUS_FLT && !HALT_In)
        ; // RETRY condition: no opcode bus error.
    else if (OPCODE_ACCESS && BUS_CTRL_STATE == DATA_C1C4 && BUS_FLT)
        OPCODE_VALID <= 1'b0;
    else if (OPCODE_RDY_I)
        OPCODE_VALID <= 1'b1;

    if (RESET_CPU_I)
        DATA_VALID <= 1'b1;
    else if (BUS_CTRL_STATE == DATA_C1C4 && BUS_FLT && !HALT_In)
        ; // RETRY condition: no bus error.
    else if (BUS_CTRL_STATE == DATA_C1C4 && BUS_FLT)
        DATA_VALID <= 1'b0;
    else if (DATA_RDY_I)
        DATA_VALID <= 1'b1;
end

// ---- Prefetch / data buffers and ready strobes ----
always_ff @(posedge CLK) begin : prefetch_buffers
    OPCODE_RDY_I <= 1'b0; // Strobe.
    DATA_RDY_I <= 1'b0;   // Strobe.

    if (RESET_CPU_I || DATA_RDY_I || OPCODE_RDY_I)
        RDY_ARMED <= 1'b0;
    else if (BUS_CTRL_STATE == START_CYCLE)
        RDY_ARMED <= 1'b1;

    // Opcode cycle complete:
    if (AERR_I)
        OPCODE_RDY_I <= 1'b1;
    else if (OPCODE_ACCESS && BUS_CTRL_STATE == DATA_C1C4 && BUS_CYC_RDY && SIZE_N == 3'b000) begin
        OBUFFER <= DATA_INMUX[15:0];
        OPCODE_RDY_I <= RDY_ARMED;
    end

    // Data cycle complete:
    if (WRITE_ACCESS && BUS_CTRL_STATE == DATA_C1C4 && BUS_CYC_RDY && SIZE_N == 3'b000) begin
        DATA_RDY_I <= RDY_ARMED;
    end else if (READ_ACCESS && BUS_CTRL_STATE == DATA_C1C4 && BUS_CYC_RDY) begin
        case (OP_SIZE)
            LONG: begin
                if (SIZE_N == 3'b000) begin
                    DBUFFER <= DATA_INMUX;
                    DATA_RDY_I <= RDY_ARMED;
                end
            end
            WORD: begin
                if (SIZE_N == 3'b000) begin
                    DBUFFER <= {16'h0, DATA_INMUX[15:0]};
                    DATA_RDY_I <= RDY_ARMED;
                end
            end
            BYTE: begin // Byte always aligned.
                DATA_RDY_I <= RDY_ARMED;
                DBUFFER <= {24'h0, DATA_INMUX[7:0]};
            end
            default: ;
        endcase
    end
end

assign DATA_RDY = DATA_RDY_I;
assign OPCODE_RDY = OPCODE_RDY_I;

assign DATA_TO_CORE = DBUFFER;
assign OPCODE_TO_CORE = OBUFFER;

// ---- Wait-state / termination detection ----
// WAITSTATES is active when S3 has not yet seen a valid termination signal.
assign WAITSTATES = (T_SLICE != S3) ? 1'b0 :
                    RESET_OUT_I ? 1'b1 : // No bus fault during RESET instruction.
                    (DSACK_In != 2'b11) ? 1'b0 : // Asynchronous termination.
                    !STERM_In ? 1'b0 :            // Synchronous termination.
                    (ADR_IN_P[19:16] == 4'hF && !AVEC_In) ? 1'b0 : // Autovector acknowledge.
                    BUS_FLT ? 1'b0 :              // Bus error terminates cycle.
                    RESET_CPU_I ? 1'b0 : 1'b1;   // CPU reset terminates cycle.

// ---- Time-slice counter ----
// Generates the S0-S5 bus timing from a 3-bit counter pair (posedge/negedge).
always_ff @(posedge CLK) begin : slice_counter_pos
    if (BUS_CTRL_STATE == BUS_IDLE)
        SLICE_CNT_P <= 3'b111; // Init.
    else if (RETRY)
        SLICE_CNT_P <= 3'b111;
    else if (BUS_CTRL_STATE != BUS_IDLE && NEXT_BUS_CTRL_STATE == BUS_IDLE)
        SLICE_CNT_P <= 3'b111; // Init.
    else if (SLICE_CNT_P == 3'b001 && !STERM_In) // Synchronous cycle.
        SLICE_CNT_P <= 3'b111; // Ready.
    else if (SLICE_CNT_P == 3'b010) begin
        if (RETRY)
            SLICE_CNT_P <= 3'b111;
        else if (BUS_CTRL_STATE == DATA_C1C4 && NEXT_BUS_CTRL_STATE == BUS_IDLE)
            SLICE_CNT_P <= 3'b111; // Ready.
        else
            SLICE_CNT_P <= 3'b000; // Continue to next sub-cycle.
    end else if (!WAITSTATES)
        SLICE_CNT_P <= SLICE_CNT_P + 1'b1;
end

always_ff @(negedge CLK) begin : slice_counter_neg
    SLICE_CNT_N <= SLICE_CNT_P; // Follow the positive-edge counter.
end

// Decode the time slice from the counter pair.
// Each slice spans one half-clock period between posedge and negedge transitions.
assign T_SLICE = (SLICE_CNT_P == 3'b000 && SLICE_CNT_N == 3'b111) ? S0 :
                 (SLICE_CNT_P == 3'b000 && SLICE_CNT_N == 3'b000) ? S1 :
                 (SLICE_CNT_P == 3'b001 && SLICE_CNT_N == 3'b000) ? S2 :
                 (SLICE_CNT_P == 3'b001 && SLICE_CNT_N == 3'b001) ? S3 :
                 (SLICE_CNT_P == 3'b010 && SLICE_CNT_N == 3'b001) ? S4 :
                 (SLICE_CNT_P == 3'b010 && SLICE_CNT_N == 3'b010) ? S5 :
                 (SLICE_CNT_P == 3'b000 && SLICE_CNT_N == 3'b010) ? S0 : SLICE_IDLE; // Rollover S5->S0.

// ---- OCS inhibit after first portion of multi-cycle transfer ----
always_ff @(posedge CLK) begin : ocs_inhibit
    if (BUS_CTRL_STATE == START_CYCLE && NEXT_BUS_CTRL_STATE != BUS_IDLE)
        OCS_INH <= 1'b0;
    else if (BUS_CYC_RDY && !RETRY)
        OCS_INH <= 1'b1;
end

// ---- Bus control signal generation ----
// Active-low signals follow the MC68030 bus protocol timing.
assign SLICE_S0_TO_S4 = (T_SLICE == S0 || T_SLICE == S1 || T_SLICE == S2 || T_SLICE == S3 || T_SLICE == S4);
assign SLICE_S1_TO_S5 = (T_SLICE == S1 || T_SLICE == S2 || T_SLICE == S3 || T_SLICE == S4 || T_SLICE == S5);
assign SLICE_S2_TO_S4 = (T_SLICE == S2 || T_SLICE == S3 || T_SLICE == S4);
assign SLICE_S3_TO_S5 = (T_SLICE == S3 || T_SLICE == S4 || T_SLICE == S5);

assign RWn  = !(WRITE_ACCESS && BUS_CTRL_STATE == DATA_C1C4);
assign RMCn = !RMC;
assign ECSn = !(T_SLICE == S0);
assign OCSn = !(T_SLICE == S0 && !OCS_INH);
assign ASn  = !SLICE_S0_TO_S4;
assign DSn  = !(WRITE_ACCESS ? SLICE_S3_TO_S5 : SLICE_S0_TO_S4); // Write: DS late, read: DS early.
assign DBENn = !(WRITE_ACCESS ? SLICE_S1_TO_S5 : SLICE_S2_TO_S4); // Write: S1-S5, read: S2-S4.

// ---- Bus tri-state controls ----
assign BUS_EN       = ARB_STATE == ARB_IDLE && !RESET_CPU_I;
assign DATA_PORT_EN = WRITE_ACCESS && ARB_STATE == ARB_IDLE && !RESET_CPU_I;

// ---- Bus cycle completion detection ----
assign BUS_CYC_RDY = RETRY ? 1'b0 :
                     (T_SLICE == S3 && !STERM_In) ? 1'b1 : // Synchronous termination.
                     (T_SLICE == S5);                       // Asynchronous termination.

// ---- Bus arbitration state machine ----
always_ff @(posedge CLK) begin : arb_reg
    if (RESET_CPU_I)
        ARB_STATE <= ARB_IDLE;
    else
        ARB_STATE <= NEXT_ARB_STATE;
end

always_comb begin : arb_dec
    case (ARB_STATE)
        ARB_IDLE: begin
            if (RMC && !RETRY)
                NEXT_ARB_STATE = ARB_IDLE;
            else if (!BGACK_In && BUS_CTRL_STATE == BUS_IDLE)
                NEXT_ARB_STATE = WAIT_RELEASE_3WIRE;
            else if (!BR_In && BUS_CTRL_STATE == BUS_IDLE)
                NEXT_ARB_STATE = GRANT;
            else
                NEXT_ARB_STATE = ARB_IDLE;
        end
        GRANT: begin
            if (!BGACK_In)
                NEXT_ARB_STATE = WAIT_RELEASE_3WIRE;
            else if (BR_In)
                NEXT_ARB_STATE = ARB_IDLE;
            else
                NEXT_ARB_STATE = GRANT;
        end
        WAIT_RELEASE_3WIRE: begin
            if (BGACK_In && !BR_In)
                NEXT_ARB_STATE = GRANT;
            else if (BGACK_In)
                NEXT_ARB_STATE = ARB_IDLE;
            else
                NEXT_ARB_STATE = WAIT_RELEASE_3WIRE;
        end
        default:
            NEXT_ARB_STATE = ARB_IDLE;
    endcase
end

assign BGn = (ARB_STATE == GRANT) ? 1'b0 : 1'b1;

// ---- Reset input filter ----
// Requires RESET_IN held low with HALTn low for ~10 clock cycles to trigger.
always_ff @(posedge CLK) begin : reset_filter
    logic STARTUP;
    logic [3:0] TMP;

    if (RESET_IN && !HALT_In && !RESET_OUT_I && TMP < 4'hF)
        TMP = TMP + 1'b1;
    else if (!RESET_IN || HALT_In || RESET_OUT_I)
        TMP = 4'h0;

    if (TMP > 4'hA) begin
        RESET_CPU_I <= 1'b1;
        STARTUP = 1'b1;
    end else if (!STARTUP) begin
        RESET_CPU_I <= 1'b1;
    end else begin
        RESET_CPU_I <= 1'b0;
    end
end

// ---- Reset output timer ----
// Drives RESET_OUT for 512 clock cycles when RESET_STRB is asserted.
always_ff @(posedge CLK) begin : reset_timer
    logic [8:0] TMP;

    if (RESET_STRB || TMP > 9'd0)
        RESET_OUT_I <= 1'b1;
    else
        RESET_OUT_I <= 1'b0;

    if (RESET_STRB)
        TMP = 9'd511;
    else if (TMP > 9'd0)
        TMP = TMP - 1'b1;
end

assign RESET_CPU = RESET_CPU_I;
assign RESET_OUT = RESET_OUT_I;

endmodule
