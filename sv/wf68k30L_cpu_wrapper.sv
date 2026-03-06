// ------------------------------------------------------------------------
// -- WF68K30L CPU Wrapper                                               --
// -- Presents a CPU-like external bus around WF68K30L_TOP without       --
// -- baking FPGA pin choices or board-specific I/O buffers into the     --
// -- core itself.                                                        --
// ------------------------------------------------------------------------

module WF68K30L_CPU_WRAPPER #(
    parameter logic [15:0] VERSION = 16'h1904,
    parameter NO_PIPELINE = 0,
    parameter NO_LOOP     = 0
) (
    input  logic        CLK,

    // CPU-style address/data bus.
    output logic [31:0] A,
    inout  wire  [31:0] D,

    // System control. RESETn is combined into a bidirectional open-drain pin.
    input  logic        BERRn,
    inout  wire         RESETn,
    input  logic        HALTn,
    output logic        HALT_OUTn_DBG,

    // Processor status / interrupt control.
    output logic [2:0]  FC,
    input  logic        AVECn,
    input  logic [2:0]  IPLn,
    output logic        IPENDn,

    // Asynchronous bus control.
    input  logic [1:0]  DSACKn,
    output logic [1:0]  SIZE,
    output logic        ASn,
    output logic        RWn,
    output logic        RMCn,
    output logic        DSn,
    output logic        ECSn,
    output logic        OCSn,
    output logic        CIOUTn,
    output logic        CBREQn,
    output logic        DBENn,
    output logic        BUS_EN,

    // Synchronous bus control.
    input  logic        CBACKn,
    input  logic        STERMn,

    // Cache/status sideband.
    output logic        STATUSn,
    output logic        REFILLn,

    // Bus arbitration.
    input  logic        BRn,
    output logic        BGn,
    input  logic        BGACKn
);

logic [31:0] DATA_IN;
logic [31:0] DATA_OUT;
logic        DATA_EN;
logic        RESET_OUT;
logic        HALT_OUTn;

// Drive the bidirectional package-style data bus only on core writes.
assign D = DATA_EN ? DATA_OUT : 32'hzzzzzzzz;
assign DATA_IN = D;

// The core exposes RESET request as an active-high open-drain intent.
assign RESETn = RESET_OUT ? 1'b0 : 1'bz;

// The real MC68030 HALT pin is input-only. Keep the core's halt-output
// indication as a separate sideband so board wrappers can decide whether
// to ignore it, log it, or route it to debug hardware.
assign HALT_OUTn_DBG = HALT_OUTn;

WF68K30L_TOP #(
    .VERSION(VERSION),
    .NO_PIPELINE(NO_PIPELINE),
    .NO_LOOP(NO_LOOP)
) u_core (
    .CLK(CLK),

    .ADR_OUT(A),
    .DATA_IN(DATA_IN),
    .DATA_OUT(DATA_OUT),
    .DATA_EN(DATA_EN),

    .BERRn(BERRn),
    .RESET_INn(RESETn),
    .RESET_OUT(RESET_OUT),
    .HALT_INn(HALTn),
    .HALT_OUTn(HALT_OUTn),

    .FC_OUT(FC),

    .AVECn(AVECn),
    .IPLn(IPLn),
    .IPENDn(IPENDn),

    .DSACKn(DSACKn),
    .SIZE(SIZE),
    .ASn(ASn),
    .RWn(RWn),
    .RMCn(RMCn),
    .DSn(DSn),
    .ECSn(ECSn),
    .OCSn(OCSn),
    .CIOUTn(CIOUTn),
    .CBREQn(CBREQn),
    .DBENn(DBENn),
    .BUS_EN(BUS_EN),

    .CBACKn(CBACKn),
    .STERMn(STERMn),

    .STATUSn(STATUSn),
    .REFILLn(REFILLn),

    .BRn(BRn),
    .BGn(BGn),
    .BGACKn(BGACKn)
);

endmodule
