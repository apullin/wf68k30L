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
    output wire  [31:0] A,
    inout  wire  [31:0] D,

    // System control. RESETn is combined into a bidirectional open-drain pin,
    // so the net needs the board pull-up to resolve when nobody drives it.
    // Yosys parses neither tri1 nor the pullup primitive; on real hardware
    // this is a pad attribute rather than RTL.
    input  logic        BERRn,
`ifdef YOSYS
    inout  wire         RESETn,
`else
    inout  tri1         RESETn,
`endif
    input  logic        HALTn,
    output logic        HALT_OUTn_DBG,

    // Processor status / interrupt control.
    output wire  [2:0]  FC,
    input  logic        AVECn,
    input  logic [2:0]  IPLn,
    output logic        IPENDn,

    // Asynchronous bus control.
    input  logic [1:0]  DSACKn,
    output wire  [1:0]  SIZE,
    output wire         ASn,
    output wire         RWn,
    output wire         RMCn,
    output wire         DSn,
    output logic        ECSn,
    output logic        OCSn,
    output wire         CIOUTn,
    output wire         CBREQn,
    output wire         DBENn,
    output logic        BUS_EN,

    // Synchronous bus control.
    input  logic        CBACKn,
    input  logic        CIINn,
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

// Core-side bus outputs, ahead of the arbitration three-state.
logic [31:0] A_CORE;
logic [2:0]  FC_CORE;
logic [1:0]  SIZE_CORE;
logic        ASn_CORE;
logic        RWn_CORE;
logic        RMCn_CORE;
logic        DSn_CORE;
logic        CIOUTn_CORE;
logic        CBREQn_CORE;
logic        DBENn_CORE;

// Drive the bidirectional package-style data bus only on core writes.
// DATA_EN already accounts for arbitration and reset.
assign D = DATA_EN ? DATA_OUT : 32'hzzzzzzzz;
assign DATA_IN = D;

// UM Table 5-2 lists exactly these outputs as three-state, and UM 7.7.4 and
// 7.8 require them released while the processor is not the bus master and
// while reset is asserted. BUS_EN is the core's own qualifier for both.
// ECS, OCS, IPEND and BG are driven at all times per the same table.
assign A      = BUS_EN ? A_CORE      : 32'hzzzzzzzz;
assign FC     = BUS_EN ? FC_CORE     : 3'bzzz;
assign SIZE   = BUS_EN ? SIZE_CORE   : 2'bzz;
assign ASn    = BUS_EN ? ASn_CORE    : 1'bz;
assign RWn    = BUS_EN ? RWn_CORE    : 1'bz;
assign RMCn   = BUS_EN ? RMCn_CORE   : 1'bz;
assign DSn    = BUS_EN ? DSn_CORE    : 1'bz;
assign CIOUTn = BUS_EN ? CIOUTn_CORE : 1'bz;
assign CBREQn = BUS_EN ? CBREQn_CORE : 1'bz;
assign DBENn  = BUS_EN ? DBENn_CORE  : 1'bz;

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

    .ADR_OUT(A_CORE),
    .DATA_IN(DATA_IN),
    .DATA_OUT(DATA_OUT),
    .DATA_EN(DATA_EN),

    .BERRn(BERRn),
    .RESET_INn(RESETn),
    .RESET_OUT(RESET_OUT),
    .HALT_INn(HALTn),
    .HALT_OUTn(HALT_OUTn),

    .FC_OUT(FC_CORE),

    .AVECn(AVECn),
    .IPLn(IPLn),
    .IPENDn(IPENDn),

    .DSACKn(DSACKn),
    .SIZE(SIZE_CORE),
    .ASn(ASn_CORE),
    .RWn(RWn_CORE),
    .RMCn(RMCn_CORE),
    .DSn(DSn_CORE),
    .ECSn(ECSn),
    .OCSn(OCSn),
    .CIOUTn(CIOUTn_CORE),
    .CBREQn(CBREQn_CORE),
    .DBENn(DBENn_CORE),
    .BUS_EN(BUS_EN),

    .CBACKn(CBACKn),
    .CIINn(CIINn),
    .STERMn(STERMn),

    .STATUSn(STATUSn),
    .REFILLn(REFILLn),

    .BRn(BRn),
    .BGn(BGn),
    .BGACKn(BGACKn)
);

endmodule
