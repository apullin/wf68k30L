//--------------------------------------------------------------------//
//                                                                    //
// WF68K30L IP Core: Fetch state machine next-state decoder.          //
//                                                                    //
// This module contains the combinational next-state logic for the    //
// fetch pipeline state machine. It was extracted from                //
// wf68k30L_control.sv to improve maintainability.                    //
//                                                                    //
// The logic is split into two sub-decoders:                          //
//   - WF68K30L_CTRL_FETCH_START: handles the START_OP state          //
//   - WF68K30L_CTRL_FETCH_OTHER: handles all other states            //
// This top-level module selects between their outputs based on       //
// the current FETCH_STATE.                                           //
//                                                                    //
//--------------------------------------------------------------------//

module WF68K30L_CTRL_FETCH_DEC (
    // Current states
    input  logic [4:0]  FETCH_STATE,
    input  logic [2:0]  EXEC_WB_STATE,
    input  logic [2:0]  NEXT_EXEC_WB_STATE,

    // Operation
    input  logic [6:0]  OP,
    input  logic [13:0] BIW_0,
    input  logic [15:0] BIW_1,
    input  logic [15:0] BIW_2,
    input  logic [15:0] EXT_WORD,

    // Data availability
    input  logic        OPD_ACK,
    input  logic        OW_RDY,
    input  logic        EW_ACK,
    input  logic        EW_RDY,
    input  logic        RD_RDY,
    input  logic        MEMADR_RDY,
    input  logic        WR_RDY,

    // Hazard signals
    input  logic        DR_IN_USE,
    input  logic        AR_IN_USE,

    // ALU
    input  logic        ALU_BSY,
    input  logic        ALU_COND,

    // Internal state
    input  logic [2:0]  ADR_MODE_I,
    input  logic [1:0]  OP_SIZE_I,
    input  logic        PHASE2,

    // MOVEM/MOVEP
    input  logic        MOVEM_COND,
    input  logic [3:0]  MOVEM_PNTR,
    input  logic        MOVEM_FIRST_RD,
    input  int          MOVEP_PNTR_I,

    // Bitfield
    input  int          BF_BYTES,

    // Control
    input  logic        BRANCH_ATN,
    input  logic        DBcc_COND,
    input  logic [1:0]  TRACE_MODE,
    input  logic        EXH_REQ,
    input  logic        BUSY_EXH,
    input  logic        LOOP_BSY,

    // Address format
    input  logic        OD_REQ_32,
    input  logic        OD_REQ_16,
    input  logic        MEM_INDIRECT,

    // Output
    output logic [4:0]  NEXT_FETCH_STATE
);

`include "wf68k30L_pkg.svh"

// Local state constant
localparam logic [4:0] START_OP = 5'd0;

// Sub-decoder outputs
logic [4:0] NEXT_STATE_START;
logic [4:0] NEXT_STATE_OTHER;

// START_OP state decoder
WF68K30L_CTRL_FETCH_START I_FETCH_START (
    .OP              (OP),
    .BIW_0           (BIW_0),
    .BIW_1           (BIW_1),
    .OPD_ACK         (OPD_ACK),
    .OW_RDY          (OW_RDY),
    .DR_IN_USE       (DR_IN_USE),
    .AR_IN_USE       (AR_IN_USE),
    .ALU_BSY         (ALU_BSY),
    .OP_SIZE_I       (OP_SIZE_I),
    .NEXT_FETCH_STATE(NEXT_STATE_START)
);

// All other states decoder
WF68K30L_CTRL_FETCH_OTHER I_FETCH_OTHER (
    .FETCH_STATE        (FETCH_STATE),
    .EXEC_WB_STATE      (EXEC_WB_STATE),
    .NEXT_EXEC_WB_STATE (NEXT_EXEC_WB_STATE),
    .OP                 (OP),
    .BIW_0              (BIW_0),
    .BIW_1              (BIW_1),
    .EXT_WORD           (EXT_WORD),
    .EW_ACK             (EW_ACK),
    .EW_RDY             (EW_RDY),
    .RD_RDY             (RD_RDY),
    .MEMADR_RDY         (MEMADR_RDY),
    .WR_RDY             (WR_RDY),
    .DR_IN_USE          (DR_IN_USE),
    .AR_IN_USE          (AR_IN_USE),
    .ALU_BSY            (ALU_BSY),
    .ALU_COND           (ALU_COND),
    .ADR_MODE_I         (ADR_MODE_I),
    .PHASE2             (PHASE2),
    .MOVEM_COND         (MOVEM_COND),
    .MOVEM_PNTR         (MOVEM_PNTR),
    .MOVEM_FIRST_RD     (MOVEM_FIRST_RD),
    .MOVEP_PNTR_I       (MOVEP_PNTR_I),
    .BF_BYTES           (BF_BYTES),
    .BRANCH_ATN         (BRANCH_ATN),
    .DBcc_COND          (DBcc_COND),
    .TRACE_MODE         (TRACE_MODE),
    .EXH_REQ            (EXH_REQ),
    .LOOP_BSY           (LOOP_BSY),
    .OD_REQ_32          (OD_REQ_32),
    .OD_REQ_16          (OD_REQ_16),
    .MEM_INDIRECT       (MEM_INDIRECT),
    .NEXT_FETCH_STATE   (NEXT_STATE_OTHER)
);

// Mux: select START_OP decoder output when in START_OP, otherwise other decoder
always_comb begin
    if (FETCH_STATE == START_OP) begin
        NEXT_FETCH_STATE = NEXT_STATE_START;
    end else begin
        NEXT_FETCH_STATE = NEXT_STATE_OTHER;
    end
end

endmodule
