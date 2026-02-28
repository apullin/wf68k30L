module data_regs_hazard_formal;
    logic CLK = 1'b0;
    always_ff @($global_clock) begin
        CLK <= !CLK;
    end

    logic RESET;
    logic [31:0] DR_IN_1, DR_IN_2;
    logic [31:0] DR_OUT_1, DR_OUT_2;
    logic [2:0] DR_SEL_WR_1, DR_SEL_WR_2, DR_SEL_RD_1, DR_SEL_RD_2;
    logic DR_WR_1, DR_WR_2;
    logic DR_MARK_USED;
    logic USE_DPAIR;
    logic DR_IN_USE;
    logic UNMARK;
    logic [1:0] OP_SIZE;

    WF68K30L_DATA_REGISTERS dut (
        .CLK(CLK),
        .RESET(RESET),
        .DR_IN_1(DR_IN_1),
        .DR_IN_2(DR_IN_2),
        .DR_OUT_1(DR_OUT_1),
        .DR_OUT_2(DR_OUT_2),
        .DR_SEL_WR_1(DR_SEL_WR_1),
        .DR_SEL_WR_2(DR_SEL_WR_2),
        .DR_SEL_RD_1(DR_SEL_RD_1),
        .DR_SEL_RD_2(DR_SEL_RD_2),
        .DR_WR_1(DR_WR_1),
        .DR_WR_2(DR_WR_2),
        .DR_MARK_USED(DR_MARK_USED),
        .USE_DPAIR(USE_DPAIR),
        .DR_IN_USE(DR_IN_USE),
        .UNMARK(UNMARK),
        .OP_SIZE(OP_SIZE)
    );

    // Inputs used by the hazard path are unconstrained.
    always_comb begin
        DR_IN_1 = $anyseq;
        DR_IN_2 = $anyseq;
        DR_SEL_WR_1 = $anyseq;
        DR_SEL_WR_2 = $anyseq;
        DR_SEL_RD_1 = $anyseq;
        DR_SEL_RD_2 = $anyseq;
        DR_MARK_USED = $anyseq;
        USE_DPAIR = $anyseq;
        UNMARK = $anyseq;
        OP_SIZE = $anyseq;
    end

    // Keep register-file writes disabled in this smoke check so assertions
    // focus on hazard-tracker behavior only.
    assign DR_WR_1 = 1'b0;
    assign DR_WR_2 = 1'b0;

    logic f_past_valid = 1'b0;

    always_ff @(posedge CLK) begin
        f_past_valid <= 1'b1;

        // Start from reset then run with reset deasserted.
        if (!f_past_valid) begin
            RESET <= 1'b1;
        end else begin
            RESET <= 1'b0;
        end

        if (f_past_valid) begin
            // If the previous cycle cleared hazard tracking and the current
            // cycle does not mark a new in-flight destination, no hazard can
            // be reported.
            if ($past(RESET || UNMARK) && !(RESET || UNMARK) && !DR_MARK_USED) begin
                assert(!DR_IN_USE);
            end

            // A previous mark-used on WR1 must trigger hazard visibility for
            // either read port when that register is selected, unless the
            // tracker is overwritten/cleared in the current cycle.
            if (
                $past(DR_MARK_USED && !(RESET || UNMARK)) &&
                !(RESET || UNMARK) &&
                !DR_MARK_USED
            ) begin
                if (
                    DR_SEL_RD_1 == $past(DR_SEL_WR_1) ||
                    DR_SEL_RD_2 == $past(DR_SEL_WR_1)
                ) begin
                    assert(DR_IN_USE);
                end
            end

            // Same check for WR2 when pair mode was used.
            if (
                $past(DR_MARK_USED && USE_DPAIR && !(RESET || UNMARK)) &&
                !(RESET || UNMARK) &&
                !DR_MARK_USED
            ) begin
                if (
                    DR_SEL_RD_1 == $past(DR_SEL_WR_2) ||
                    DR_SEL_RD_2 == $past(DR_SEL_WR_2)
                ) begin
                    assert(DR_IN_USE);
                end
            end
        end
    end
endmodule
