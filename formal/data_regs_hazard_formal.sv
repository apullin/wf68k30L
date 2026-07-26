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

    always_comb begin
        DR_IN_1 = $anyseq;
        DR_IN_2 = $anyseq;
        DR_SEL_WR_1 = $anyseq;
        DR_SEL_WR_2 = $anyseq;
        DR_SEL_RD_1 = $anyseq;
        DR_SEL_RD_2 = $anyseq;
        DR_WR_1 = $anyseq;
        DR_WR_2 = $anyseq;
        DR_MARK_USED = $anyseq;
        USE_DPAIR = $anyseq;
        UNMARK = $anyseq;
        OP_SIZE = $anyseq;
    end

    // OP_SIZE 2'd3 is not an OP_SIZETYPE member; the register file ignores it.
    always_comb assume(OP_SIZE != 2'd3);

`ifdef REGFILE_PROPERTY
    // Writes land on the select captured by the last DR_MARK_USED, so a write
    // before any mark would target an undefined register. The control logic
    // never does that; without the assumption the solver picks the RTL's
    // uninitialized latch and the shadow's independently.
    logic f_marked = 1'b0;
    always_ff @(posedge CLK) begin
        if (DR_MARK_USED)
            f_marked <= 1'b1;
    end
    always_comb begin
        if (!f_marked)
            assume(!DR_WR_1 && !DR_WR_2);
    end

`endif

    logic f_past_valid = 1'b0;

    always_ff @(posedge CLK) begin
        f_past_valid <= 1'b1;

        // Start from reset then run with reset deasserted.
        if (!f_past_valid) begin
            RESET <= 1'b1;
        end else begin
            RESET <= 1'b0;
        end
    end

`ifdef REGFILE_PROPERTY
// The register-file property below is exhaustive but expensive: the $anyconst
// index turns both read ports into symbolic 8-way muxes over 32-bit words, and
// after clk2fflogic unrolling the UNSAT direction did not complete at depth 10
// with yices, boolector or bitwuzla (counterexamples are still found in
// seconds). Kept out of formal-smoke and exercised by `make formal-deep`.
    // ---- Register-file data integrity ----
    // Shadow one symbolically-chosen register and require the read ports to
    // agree with it. This covers size-partial writes preserving upper bits and
    // the write-port priority when both ports target the same register.

    logic [2:0]  f_idx = $anyconst;
    logic [31:0] f_shadow;
    logic        f_shadow_valid = 1'b0;

    // Writes land on the select latched when DR_MARK_USED was asserted.
    logic [2:0] f_lat_1, f_lat_2;
    always_ff @(posedge CLK) begin
        if (DR_MARK_USED) begin
            f_lat_1 <= DR_SEL_WR_1;
            f_lat_2 <= DR_SEL_WR_2;
        end
    end

    // Mirrors the RTL block structure exactly: the reset clear is not an else
    // arm, so a write in the same cycle as RESET wins, and port 2 wins a tie
    // with port 1 because it is assigned last.
    always_ff @(posedge CLK) begin
        if (RESET) begin
            f_shadow <= 32'h0;
            f_shadow_valid <= 1'b1;
        end

        if (DR_WR_1 && f_lat_1 == f_idx) begin
            case (OP_SIZE)
                2'd0: f_shadow        <= DR_IN_1;
                2'd1: f_shadow[15:0]  <= DR_IN_1[15:0];
                2'd2: f_shadow[7:0]   <= DR_IN_1[7:0];
                default: ;
            endcase
        end

        if (DR_WR_2 && f_lat_2 == f_idx) begin
            case (OP_SIZE)
                2'd0: f_shadow        <= DR_IN_2;
                2'd1: f_shadow[15:0]  <= DR_IN_2[15:0];
                2'd2: f_shadow[7:0]   <= DR_IN_2[7:0];
                default: ;
            endcase
        end
    end

    always_ff @(posedge CLK) begin
        if (f_shadow_valid) begin
            if (DR_SEL_RD_1 == f_idx)
                assert(DR_OUT_1 == f_shadow);
            if (DR_SEL_RD_2 == f_idx)
                assert(DR_OUT_2 == f_shadow);
        end
    end

`endif // REGFILE_PROPERTY

    // ---- Hazard tracker ----

    always_ff @(posedge CLK) begin
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
