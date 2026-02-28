module mmu_walk_delay_state_formal;
    logic CLK = 1'b0;
    always_ff @($global_clock) begin
        CLK <= !CLK;
    end

    logic RESET_CPU;
    logic BUS_BSY;
    logic MMU_RUNTIME_REQ;
    logic MMU_RUNTIME_STALL;
    logic MMU_WALK_DELAY_ARMED;
    logic f_past_valid = 1'b0;

    always_comb begin
        BUS_BSY = $anyseq;
        MMU_RUNTIME_REQ = $anyseq;
        MMU_RUNTIME_STALL = $anyseq;
    end

    always_ff @(posedge CLK) begin
        if (RESET_CPU) begin
            MMU_WALK_DELAY_ARMED <= 1'b0;
        end else if (BUS_BSY) begin
            MMU_WALK_DELAY_ARMED <= 1'b0;
        end else if (MMU_RUNTIME_REQ) begin
            MMU_WALK_DELAY_ARMED <= MMU_RUNTIME_STALL;
        end else begin
            MMU_WALK_DELAY_ARMED <= 1'b0;
        end
    end

    always_ff @(posedge CLK) begin
        f_past_valid <= 1'b1;
        if (!f_past_valid)
            RESET_CPU <= 1'b1;
        else
            RESET_CPU <= 1'b0;

        if (f_past_valid) begin
            // No armed state can survive reset or active bus cycles.
            if (RESET_CPU || BUS_BSY)
                assert(!MMU_WALK_DELAY_ARMED);

            // Armed state must directly mirror prior-cycle stall when request is present.
            if (!$past(RESET_CPU) && !$past(BUS_BSY) && $past(MMU_RUNTIME_REQ))
                assert(MMU_WALK_DELAY_ARMED == $past(MMU_RUNTIME_STALL));

            // Idle cycles clear the arm latch.
            if (!$past(RESET_CPU) && !$past(BUS_BSY) && !$past(MMU_RUNTIME_REQ))
                assert(!MMU_WALK_DELAY_ARMED);
        end
    end
endmodule
