`timescale 1ns/1ps
module equiv_tb;
reg         CLK;
reg  [31:0] DATA_IN;
reg         BERRn, RESET_INn, HALT_INn, AVECn;
reg  [2:0]  IPLn;
reg  [1:0]  DSACKn;
reg         STERMn, BRn, BGACKn;

wire [31:0] g_ADR_OUT, g_DATA_OUT;
wire        g_DATA_EN, g_RESET_OUT, g_HALT_OUTn;
wire [2:0]  g_FC_OUT;
wire        g_IPENDn;
wire [1:0]  g_SIZE;
wire        g_ASn, g_RWn, g_RMCn, g_DSn, g_ECSn, g_OCSn;
wire        g_DBENn, g_BUS_EN, g_STATUSn, g_REFILLn, g_BGn;

wire [31:0] s_ADR_OUT, s_DATA_OUT;
wire        s_DATA_EN, s_RESET_OUT, s_HALT_OUTn;
wire [2:0]  s_FC_OUT;
wire        s_IPENDn;
wire [1:0]  s_SIZE;
wire        s_ASn, s_RWn, s_RMCn, s_DSn, s_ECSn, s_OCSn;
wire        s_DBENn, s_BUS_EN, s_STATUSn, s_REFILLn, s_BGn;

GOLD_TOP gold (
    .CLK(CLK), .DATA_IN(DATA_IN), .BERRn(BERRn),
    .RESET_INn(RESET_INn), .HALT_INn(HALT_INn), .AVECn(AVECn),
    .IPLn(IPLn), .DSACKn(DSACKn), .STERMn(STERMn),
    .BRn(BRn), .BGACKn(BGACKn),
    .ADR_OUT(g_ADR_OUT), .DATA_OUT(g_DATA_OUT), .DATA_EN(g_DATA_EN),
    .RESET_OUT(g_RESET_OUT), .HALT_OUTn(g_HALT_OUTn),
    .FC_OUT(g_FC_OUT), .IPENDn(g_IPENDn), .SIZE(g_SIZE),
    .ASn(g_ASn), .RWn(g_RWn), .RMCn(g_RMCn), .DSn(g_DSn),
    .ECSn(g_ECSn), .OCSn(g_OCSn), .DBENn(g_DBENn),
    .BUS_EN(g_BUS_EN), .STATUSn(g_STATUSn), .REFILLn(g_REFILLn),
    .BGn(g_BGn)
);

GATE_TOP gate (
    .CLK(CLK), .DATA_IN(DATA_IN), .BERRn(BERRn),
    .RESET_INn(RESET_INn), .HALT_INn(HALT_INn), .AVECn(AVECn),
    .IPLn(IPLn), .DSACKn(DSACKn), .STERMn(STERMn),
    .BRn(BRn), .BGACKn(BGACKn),
    .ADR_OUT(s_ADR_OUT), .DATA_OUT(s_DATA_OUT), .DATA_EN(s_DATA_EN),
    .RESET_OUT(s_RESET_OUT), .HALT_OUTn(s_HALT_OUTn),
    .FC_OUT(s_FC_OUT), .IPENDn(s_IPENDn), .SIZE(s_SIZE),
    .ASn(s_ASn), .RWn(s_RWn), .RMCn(s_RMCn), .DSn(s_DSn),
    .ECSn(s_ECSn), .OCSn(s_OCSn), .DBENn(s_DBENn),
    .BUS_EN(s_BUS_EN), .STATUSn(s_STATUSn), .REFILLn(s_REFILLn),
    .BGn(s_BGn)
);

initial CLK = 0;
always #10 CLK = ~CLK;

integer cycle_count, mismatch_count, total_mismatches, total_cycles;
integer seed, run;

function check_vec;
    input [31:0] gold, gate;
    input integer width;
    integer i;
    begin
        check_vec = 0;
        for (i = 0; i < width; i = i + 1)
            if (gold[i] !== 1'bx && gold[i] !== 1'bz &&
                gate[i] !== 1'bx && gate[i] !== 1'bz &&
                gold[i] !== gate[i])
                check_vec = 1;
    end
endfunction

task do_reset;
    begin
        DATA_IN = 0; BERRn = 1; RESET_INn = 1; HALT_INn = 1;
        AVECn = 1; IPLn = 3'b111; DSACKn = 2'b11;
        STERMn = 1; BRn = 1; BGACKn = 1;
        #5;
        RESET_INn = 0; HALT_INn = 0;
        repeat (20) @(posedge CLK);
        RESET_INn = 1; HALT_INn = 1;
        repeat (10) @(posedge CLK);
    end
endtask

task check_all;
    reg mm;
    begin
        mm = 0;
        if (check_vec(g_ADR_OUT, s_ADR_OUT, 32)) begin
            if (mismatch_count < 5) $display("  MISMATCH cycle %0d: ADR_OUT gold=%h gate=%h", cycle_count, g_ADR_OUT, s_ADR_OUT);
            mm = 1;
        end
        if (check_vec(g_DATA_OUT, s_DATA_OUT, 32)) begin
            if (mismatch_count < 5) $display("  MISMATCH cycle %0d: DATA_OUT gold=%h gate=%h", cycle_count, g_DATA_OUT, s_DATA_OUT);
            mm = 1;
        end
        if (check_vec({31'b0, g_DATA_EN}, {31'b0, s_DATA_EN}, 1)) mm = 1;
        if (check_vec({31'b0, g_RESET_OUT}, {31'b0, s_RESET_OUT}, 1)) mm = 1;
        if (check_vec({31'b0, g_HALT_OUTn}, {31'b0, s_HALT_OUTn}, 1)) mm = 1;
        if (check_vec({29'b0, g_FC_OUT}, {29'b0, s_FC_OUT}, 3)) mm = 1;
        if (check_vec({31'b0, g_IPENDn}, {31'b0, s_IPENDn}, 1)) mm = 1;
        if (check_vec({30'b0, g_SIZE}, {30'b0, s_SIZE}, 2)) mm = 1;
        if (check_vec({31'b0, g_ASn}, {31'b0, s_ASn}, 1)) mm = 1;
        if (check_vec({31'b0, g_RWn}, {31'b0, s_RWn}, 1)) mm = 1;
        if (check_vec({31'b0, g_RMCn}, {31'b0, s_RMCn}, 1)) mm = 1;
        if (check_vec({31'b0, g_DSn}, {31'b0, s_DSn}, 1)) mm = 1;
        if (check_vec({31'b0, g_ECSn}, {31'b0, s_ECSn}, 1)) mm = 1;
        if (check_vec({31'b0, g_OCSn}, {31'b0, s_OCSn}, 1)) mm = 1;
        if (check_vec({31'b0, g_DBENn}, {31'b0, s_DBENn}, 1)) mm = 1;
        if (check_vec({31'b0, g_BUS_EN}, {31'b0, s_BUS_EN}, 1)) mm = 1;
        if (check_vec({31'b0, g_STATUSn}, {31'b0, s_STATUSn}, 1)) mm = 1;
        if (check_vec({31'b0, g_REFILLn}, {31'b0, s_REFILLn}, 1)) mm = 1;
        if (check_vec({31'b0, g_BGn}, {31'b0, s_BGn}, 1)) mm = 1;
        if (mm) mismatch_count = mismatch_count + 1;
    end
endtask

initial begin
    total_mismatches = 0;
    total_cycles = 0;

    for (run = 0; run < 5; run = run + 1) begin
        seed = run * 1000 + 42;
        mismatch_count = 0;
        cycle_count = 0;
        do_reset();

        repeat (10000) begin
            @(posedge CLK);
            cycle_count = cycle_count + 1;
            if ($random(seed) % 8 == 0) DATA_IN = $random(seed);
            if ($random(seed) % 16 == 0) BERRn = ~BERRn;
            if ($random(seed) % 32 == 0) DSACKn = $random(seed);
            if ($random(seed) % 64 == 0) STERMn = ~STERMn;
            if ($random(seed) % 32 == 0) IPLn = $random(seed);
            if ($random(seed) % 128 == 0) AVECn = ~AVECn;
            if ($random(seed) % 256 == 0) BRn = ~BRn;
            if ($random(seed) % 256 == 0) BGACKn = ~BGACKn;
            @(negedge CLK);
            check_all();
        end

        $display("Run %0d (seed=%0d): %0d cycles, %0d mismatches", run, seed, cycle_count, mismatch_count);
        total_mismatches = total_mismatches + mismatch_count;
        total_cycles = total_cycles + cycle_count;
    end

    $display("");
    $display("=== MULTI-SEED EQUIVALENCE TEST ===");
    $display("Total cycles: %0d", total_cycles);
    $display("Total mismatches: %0d", total_mismatches);
    if (total_mismatches == 0)
        $display("RESULT: PASS");
    else
        $display("RESULT: FAIL");
    $finish;
end
endmodule
