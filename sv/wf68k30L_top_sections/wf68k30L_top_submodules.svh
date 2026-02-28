// ========================================================================
// Submodule instantiations
// ========================================================================

    // Architectural state: address registers and effective-address helper path.
    WF68K30L_ADDRESS_REGISTERS I_ADDRESSREGISTERS (
        .CLK                    (CLK),
        .RESET                  (RESET_CPU),
        .AR_IN_1                (AR_IN_1),
        .AR_IN_2                (AR_IN_2),
        .AR_OUT_1               (AR_OUT_1),
        .AR_OUT_2               (AR_OUT_2),
        .INDEX_IN               (DR_OUT_1), // From data register section.
        .PC                     (PC),
        .FETCH_MEM_ADR          (FETCH_MEM_ADR),
        .STORE_ADR_FORMAT       (STORE_ADR_FORMAT),
        .STORE_ABS_HI           (STORE_ABS_HI),
        .STORE_ABS_LO           (STORE_ABS_LO),
        .STORE_D16              (STORE_D16),
        .STORE_D32_LO           (STORE_D32_LO),
        .STORE_D32_HI           (STORE_D32_HI),
        .STORE_DISPL            (STORE_DISPL),
        .STORE_MEM_ADR          (STORE_MEM_ADR),
        .STORE_OD_HI            (STORE_OD_HI),
        .STORE_OD_LO            (STORE_OD_LO),
        .STORE_AEFF             (STORE_AEFF),
        .OP_SIZE                (OP_SIZE),
        .AR_MARK_USED           (AR_MARK_USED),
        .USE_APAIR              (USE_APAIR),
        .AR_IN_USE              (AR_IN_USE),
        .AR_SEL_RD_1            (AR_SEL_RD_1),
        .AR_SEL_RD_2            (AR_SEL_RD_2),
        .AR_SEL_WR_1            (AR_SEL_WR_1),
        .AR_SEL_WR_2            (AR_SEL_WR_2),
        .ADR_OFFSET             (ADR_OFFSET), // Byte aligned.
        .ADR_MARK_USED          (ADR_MARK_USED),
        .ADR_IN_USE             (ADR_IN_USE),
        .ADR_MODE               (ADR_MODE),
        .AMODE_SEL              (AMODE_SEL),
        .USE_DREG               (USE_DREG),
        .ADR_EFF                (ADR_EFF),
        .ADR_EFF_WB             (ADR_EFF_WB),
        .DFC                    (DFC),
        .DFC_WR                 (DFC_WR),
        .SFC                    (SFC),
        .SFC_WR                 (SFC_WR),
        .ISP_DEC                (ISP_DEC),
        .ISP_RD                 (ISP_RD),
        .ISP_WR                 (ISP_WR),
        .MSP_RD                 (MSP_RD),
        .MSP_WR                 (MSP_WR),
        .USP_RD                 (USP_RD),
        .USP_WR                 (USP_WR),
        .AR_DEC                 (AR_DEC),
        .AR_INC                 (AR_INC),
        .AR_WR_1                (AR_WR_1),
        .AR_WR_2                (AR_WR_2),
        .UNMARK                 (UNMARK),
        .EXT_WORD               (EXT_WORD),
        .MBIT                   (STATUS_REG[12]),
        .SBIT                   (SBIT),
        .SP_ADD_DISPL           (SP_ADD_DISPL),
        .RESTORE_ISP_PC         (RESTORE_ISP_PC),
        .DISPLACEMENT           (DISPLACEMENT),
        .PC_ADD_DISPL           (PC_ADD_DISPL),
        .PC_EW_OFFSET           (PC_EW_OFFSET),
        .PC_INC                 (PC_INC),
        .PC_LOAD                (PC_LOAD),
        .PC_RESTORE             (PC_RESTORE_EXH),
        .PC_OFFSET              (PC_OFFSET)
    );

    // Arithmetic/logic execution engine and condition-code updates.
    WF68K30L_ALU I_ALU (
        .CLK                    (CLK),
        .RESET                  (RESET_CPU),
        .LOAD_OP2               (ALU_LOAD_OP2),
        .LOAD_OP3               (ALU_LOAD_OP3),
        .LOAD_OP1               (ALU_LOAD_OP1),
        .OP1_IN                 (ALU_OP1_IN),
        .OP2_IN                 (ALU_OP2_IN),
        .OP3_IN                 (ALU_OP3_IN),
        .BF_OFFSET_IN           (BF_OFFSET),
        .BF_WIDTH_IN            (BF_WIDTH),
        .BITPOS_IN              (BITPOS),
        .RESULT                 (ALU_RESULT),
        .ADR_MODE_IN            (ADR_MODE),
        .USE_DREG               (USE_DREG),
        .HILOn                  (HILOn),
        .OP_SIZE_IN             (OP_SIZE),
        .OP_IN                  (OP),
        .OP_WB                  (OP_WB),
        .BIW_0_IN               (BIW_0[11:0]),
        .BIW_1_IN               (BIW_1),
        .SR_WR                  (SR_WR),
        .SR_INIT                (SR_INIT),
        .SR_CLR_MBIT            (SR_CLR_MBIT),
        .CC_UPDT                (CC_UPDT),
        .STATUS_REG_OUT         (STATUS_REG),
        .ALU_COND               (ALU_COND),
        .ALU_INIT               (ALU_INIT),
        .ALU_BSY                (ALU_BSY),
        .ALU_REQ                (ALU_REQ),
        .ALU_ACK                (ALU_ACK),
        .IRQ_PEND               (IRQ_PEND),
        .TRAP_CHK               (TRAP_CHK),
        .TRAP_DIVZERO           (TRAP_DIVZERO)
    );

    // External bus protocol controller and physical bus drive/read path.
    WF68K30L_BUS_INTERFACE I_BUS_IF (
        .CLK                (CLK),

        .ADR_IN_P           (ADR_BUS_REQ_PHYS),
        .ADR_OUT_P          (ADR_OUT),

        .FC_IN              (FC_BUS_REQ),
        .FC_OUT             (FC_OUT),

        .DATA_PORT_IN       (DATA_IN),
        .DATA_PORT_OUT      (DATA_OUT),
        .DATA_FROM_CORE     (DATA_FROM_CORE),
        .DATA_TO_CORE       (DATA_TO_CORE_BUSIF),
        .OPCODE_TO_CORE     (OPCODE_TO_CORE_BUSIF),

        .DATA_PORT_EN       (DATA_EN),
        .BUS_EN             (BUS_EN),

        .SIZE               (SIZE),
        .OP_SIZE            (OP_SIZE_BUS),

        .RD_REQ             (RD_REQ),
        .WR_REQ             (WR_REQ),
        .DATA_RDY           (DATA_RDY_BUSIF),
        .DATA_VALID         (DATA_VALID_BUSIF),
        .OPCODE_REQ         (OPCODE_REQ),
        .OPCODE_RDY         (OPCODE_RDY_BUSIF),
        .OPCODE_VALID       (OPCODE_VALID_BUSIF),
        .RMC                (RMC),
        .BUSY_EXH           (BUSY_EXH),
        .SSW_80             (SSW_80),
        .INBUFFER           (INBUFFER),
        .OUTBUFFER          (OUTBUFFER),

        .DSACKn             (DSACKn),
        .ASn                (ASn),
        .DSn                (DSn),
        .RWn                (RWn),
        .RMCn               (RMCn),
        .ECSn               (ECSn),
        .OCSn               (OCSn),
        .DBENn              (DBENn),

        .STERMn             (STERMn),

        .BRn                (BRn),
        .BGACKn             (BGACKn),
        .BGn                (BGn),

        .RESET_STRB         (RESET_STRB),
        .RESET_IN           (RESET_IN),
        .RESET_OUT          (RESET_OUT),
        .RESET_CPU          (RESET_CPU),

        .AVECn              (AVECn_BUSIF),
        .HALTn              (HALT_INn),
        .BERRn              (BERRn),
        .AERR               (AERR),

        .BUS_BSY            (BUS_BSY)
    );

    // Core-facing return muxes: bus data/opcodes vs cache/fault synthesized responses.
    assign DATA_RDY_BUSIF_CORE = DATA_RDY_BUSIF &&
                                 !(BUS_CYCLE_BURST && !BUS_CYCLE_BURST_IS_OP);
    assign OPCODE_RDY_BUSIF_CORE = OPCODE_RDY_BUSIF &&
                                   !(BUS_CYCLE_BURST && BUS_CYCLE_BURST_IS_OP);

    // Data source mux: bus interface or data-cache hit path.
    assign DATA_RDY = DATA_RDY_BUSIF_CORE || DATA_RDY_CACHE || MMU_FAULT_DATA_ACK;
    assign DATA_VALID = DATA_RDY_CACHE ? DATA_VALID_CACHE :
                        MMU_FAULT_DATA_ACK ? 1'b1 : DATA_VALID_BUSIF;
    assign DATA_TO_CORE = DATA_RDY_CACHE ? DATA_TO_CORE_CACHE :
                          MMU_FAULT_DATA_ACK ? 32'h00000000 :
                          DATA_RDY_BUSIF_CORE ? DATA_TO_CORE_BUSIF :
                          DATA_LAST_FROM_CACHE ? DATA_TO_CORE_CACHE : DATA_TO_CORE_BUSIF;

    // Opcode source mux: bus interface or instruction-cache hit path.
    assign OPCODE_RDY = OPCODE_RDY_BUSIF_CORE || ICACHE_RDY || MMU_FAULT_OPCODE_ACK;
    assign OPCODE_VALID = ICACHE_RDY ? 1'b1 :
                          MMU_FAULT_OPCODE_ACK ? 1'b1 : OPCODE_VALID_BUSIF;
    assign OPCODE_TO_CORE = ICACHE_RDY ? ICACHE_OPCODE_WORD :
                            MMU_FAULT_OPCODE_ACK ? 16'h4E71 : OPCODE_TO_CORE_BUSIF;

    // Main pipeline controller (issue/writeback sequencing and sideband controls).
    WF68K30L_CONTROL #(
        .NO_PIPELINE(NO_PIPELINE)
    ) I_CONTROL (
        .CLK                    (CLK),
        .RESET_CPU              (RESET_CPU),
        .BUSY                   (BUSY_MAIN),
        .BUSY_EXH               (BUSY_EXH),
        .EXH_REQ                (EXH_REQ),
        .INT_TRIG               (INT_TRIG),
        .OW_REQ                 (OW_REQ_MAIN),
        .OW_VALID               (OW_VALID),
        .EW_REQ                 (EW_REQ_MAIN),
        .EW_ACK                 (EW_ACK),
        .OPD_ACK                (OPD_ACK_MAIN),
        .ADR_MARK_USED          (ADR_MARK_USED),
        .ADR_IN_USE             (ADR_IN_USE),
        .ADR_OFFSET             (ADR_OFFSET_MAIN),
        .DATA_RD                (DATA_RD_MAIN),
        .DATA_WR                (DATA_WR_MAIN),
        .DATA_RDY               (DATA_RDY),
        .DATA_VALID             (DATA_VALID),
        .RMC                    (RMC),
        .FETCH_MEM_ADR          (FETCH_MEM_ADR),
        .LOAD_OP1               (ALU_LOAD_OP1),
        .LOAD_OP2               (ALU_LOAD_OP2),
        .LOAD_OP3               (ALU_LOAD_OP3),
        .STORE_ADR_FORMAT       (STORE_ADR_FORMAT),
        .STORE_ABS_HI           (STORE_ABS_HI),
        .STORE_ABS_LO           (STORE_ABS_LO),
        .STORE_D16              (STORE_D16),
        .STORE_D32_LO           (STORE_D32_LO),
        .STORE_D32_HI           (STORE_D32_HI),
        .STORE_DISPL            (STORE_DISPL),
        .STORE_MEM_ADR          (STORE_MEM_ADR),
        .STORE_OD_HI            (STORE_OD_HI),
        .STORE_OD_LO            (STORE_OD_LO),
        .STORE_AEFF             (STORE_AEFF),
        .STORE_IDATA_B1         (STORE_IDATA_B1),
        .STORE_IDATA_B2         (STORE_IDATA_B2),
        .OP                     (OP),
        .OP_SIZE                (OP_SIZE_MAIN),
        .BIW_0                  (BIW_0[13:0]),
        .BIW_1                  (BIW_1),
        .BIW_2                  (BIW_2),
        .EXT_WORD               (EXT_WORD),
        .ADR_MODE               (ADR_MODE_MAIN),
        .AMODE_SEL              (AMODE_SEL),
        .USE_DREG               (USE_DREG),
        .HILOn                  (HILOn),
        .OP_WB                  (OP_WB),
        .OP_SIZE_WB             (OP_SIZE_WB),
        .BIW_0_WB_73            (BIW_0_WB_73),
        .AR_MARK_USED           (AR_MARK_USED),
        .AR_IN_USE              (AR_IN_USE),
        .AR_SEL_RD_1            (AR_SEL_RD_1_MAIN),
        .AR_SEL_RD_2            (AR_SEL_RD_2),
        .AR_SEL_WR_1            (AR_SEL_WR_1),
        .AR_SEL_WR_2            (AR_SEL_WR_2),
        .AR_INC                 (AR_INC),
        .AR_DEC                 (AR_DEC),
        .AR_WR_1                (AR_WR_1),
        .AR_WR_2                (AR_WR_2),
        .DR_MARK_USED           (DR_MARK_USED),
        .USE_APAIR              (USE_APAIR),
        .USE_DPAIR              (USE_DPAIR),
        .DR_IN_USE              (DR_IN_USE),
        .DR_SEL_WR_1            (DR_SEL_WR_1),
        .DR_SEL_WR_2            (DR_SEL_WR_2),
        .DR_SEL_RD_1            (DR_SEL_RD_1),
        .DR_SEL_RD_2            (DR_SEL_RD_2),
        .DR_WR_1                (DR_WR_1),
        .DR_WR_2                (DR_WR_2),
        .UNMARK                 (UNMARK),
        .DISPLACEMENT           (DISPLACEMENT_MAIN),
        .PC_ADD_DISPL           (PC_ADD_DISPL),
        .PC_LOAD                (PC_LOAD_MAIN),
        .PC_INC_EXH             (PC_INC_EXH),
        .SP_ADD_DISPL           (SP_ADD_DISPL_MAIN),
        .DFC_RD                 (DFC_RD),
        .DFC_WR                 (DFC_WR),
        .SFC_RD                 (SFC_RD),
        .SFC_WR                 (SFC_WR),
        .VBR_RD                 (VBR_RD),
        .VBR_WR                 (VBR_WR),
        .CACR_RD                (CACR_RD),
        .CACR_WR                (CACR_WR),
        .CAAR_RD                (CAAR_RD),
        .CAAR_WR                (CAAR_WR),
        .ISP_RD                 (ISP_RD),
        .ISP_WR                 (ISP_WR_MAIN),
        .MSP_RD                 (MSP_RD),
        .MSP_WR                 (MSP_WR),
        .USP_RD                 (USP_RD),
        .USP_WR                 (USP_WR),
        .MMU_TC_RD              (MMU_TC_RD),
        .MMU_TC_WR              (MMU_TC_WR),
        .MMU_SRP_RD             (MMU_SRP_RD),
        .MMU_SRP_WR             (MMU_SRP_WR),
        .MMU_CRP_RD             (MMU_CRP_RD),
        .MMU_CRP_WR             (MMU_CRP_WR),
        .MMU_TT0_RD             (MMU_TT0_RD),
        .MMU_TT0_WR             (MMU_TT0_WR),
        .MMU_TT1_RD             (MMU_TT1_RD),
        .MMU_TT1_WR             (MMU_TT1_WR),
        .MMU_MMUSR_RD           (MMU_MMUSR_RD),
        .MMU_MMUSR_WR           (MMU_MMUSR_WR),
        .MMU_ATC_FLUSH          (MMU_ATC_FLUSH),
        .IPIPE_FLUSH            (IPIPE_FLUSH_MAIN),
        .ALU_INIT               (ALU_INIT),
        .ALU_BSY                (ALU_BSY),
        .ALU_REQ                (ALU_REQ),
        .ALU_ACK                (ALU_ACK),
        .BKPT_CYCLE             (BKPT_CYCLE),
        .BKPT_INSERT            (BKPT_INSERT),
        .LOOP_BSY               (LOOP_BSY),
        .LOOP_SPLIT             (LOOP_SPLIT),
        .LOOP_EXIT              (LOOP_EXIT),
        .BF_OFFSET              (BF_OFFSET[2:0]),
        .BF_WIDTH               (BF_WIDTH),
        .SR_WR                  (SR_WR_MAIN),
        .MOVEM_ADn              (ADn),
        .MOVEP_PNTR             (MOVEP_PNTR),
        .CC_UPDT                (CC_UPDT),
        .TRACE_MODE             (STATUS_REG[15:14]),
        .VBIT                   (STATUS_REG[1]),
        .ALU_COND               (ALU_COND),
        .DBcc_COND              (DBcc_COND),
        .BRANCH_ATN             (BRANCH_ATN),
        .RESET_STRB             (RESET_STRB),
        .BERR                   (BERR_MAIN),
        .STATUSn                (STATUSn_MAIN),
        .EX_TRACE               (EX_TRACE),
        .TRAP_cc                (TRAP_cc),
        .TRAP_V                 (TRAP_V),
        .TRAP_ILLEGAL           (TRAP_ILLEGAL),
        .PHASE2_O               (PHASE2_MAIN)
    );

    // Data-register file.
    WF68K30L_DATA_REGISTERS I_DATA_REGISTERS (
        .CLK                    (CLK),
        .RESET                  (RESET_CPU),
        .DR_IN_1                (DR_IN_1),
        .DR_IN_2                (DR_IN_2),
        .DR_OUT_2               (DR_OUT_2),
        .DR_OUT_1               (DR_OUT_1),
        .DR_SEL_WR_1            (DR_SEL_WR_1),
        .DR_SEL_WR_2            (DR_SEL_WR_2),
        .DR_SEL_RD_1            (DR_SEL_RD_1),
        .DR_SEL_RD_2            (DR_SEL_RD_2),
        .DR_WR_1                (DR_WR_1),
        .DR_WR_2                (DR_WR_2),
        .DR_MARK_USED           (DR_MARK_USED),
        .USE_DPAIR              (USE_DPAIR),
        .DR_IN_USE              (DR_IN_USE),
        .UNMARK                 (UNMARK),
        .OP_SIZE                (OP_SIZE_WB)
    );

    // Exception/interrupt/trap stack sequencing.
    WF68K30L_EXCEPTION_HANDLER #(
        .VERSION(VERSION)
    ) I_EXC_HANDLER (
        .CLK                    (CLK),

        .RESET                  (RESET_CPU),
        .BUSY_MAIN              (BUSY_MAIN),
        .BUSY_OPD               (BUSY_OPD),

        .EXH_REQ                (EXH_REQ),
        .BUSY_EXH               (BUSY_EXH),

        .ADR_IN                 (ADR_EFF),
        .ADR_CPY                (ADR_CPY_EXH),
        .ADR_OFFSET             (ADR_OFFSET_EXH),
        .CPU_SPACE              (CPU_SPACE_EXH),

        .DATA_0                 (DATA_TO_CORE[0]),
        .DATA_RD                (DATA_RD_EXH),
        .DATA_WR                (DATA_WR_EXH),
        .DATA_IN                (DATA_IN_EXH),

        .OP_SIZE                (OP_SIZE_EXH),
        .DATA_RDY               (DATA_RDY),
        .DATA_VALID             (DATA_VALID),

        .OPCODE_RDY             (OPCODE_RDY),
        .OPD_ACK                (OPD_ACK_MAIN),
        .OW_VALID               (OW_VALID),

        .STATUS_REG_IN          (STATUS_REG),
        .SR_CPY                 (SR_CPY),
        .SR_INIT                (SR_INIT),
        .SR_CLR_MBIT            (SR_CLR_MBIT),

        .SR_WR                  (SR_WR_EXH),
        .ISP_DEC                (ISP_DEC),
        .ISP_LOAD               (ISP_LOAD_EXH),
        .PC_LOAD                (PC_LOAD_EXH),
        .PC_INC                 (PC_INC_EXH),
        .PC_RESTORE             (PC_RESTORE_EXH),

        .STACK_FORMAT           (STACK_FORMAT),
        .STACK_POS              (STACK_POS),

        .SP_ADD_DISPL           (SP_ADD_DISPL_EXH),
        .DISPLACEMENT           (DISPLACEMENT_EXH),
        .IPIPE_FILL             (IPIPE_FILL),
        .IPIPE_FLUSH            (IPIPE_FLUSH_EXH),
        .RTE_PIPE_LOAD          (RTE_PIPE_LOAD),
        .RTE_PIPE_BIW_0         (RTE_PIPE_BIW_0),
        .RTE_PIPE_BIW_1         (RTE_PIPE_BIW_1),
        .RTE_PIPE_BIW_2         (RTE_PIPE_BIW_2),
        .RTE_PIPE_C_FAULT       (RTE_PIPE_C_FAULT),
        .RTE_PIPE_B_FAULT       (RTE_PIPE_B_FAULT),
        .REFILLn                (REFILLn_EXH),
        .RESTORE_ISP_PC         (RESTORE_ISP_PC),

        .HALT_OUTn              (HALT_OUTn),
        .STATUSn                (STATUSn_EXH),

        .INT_TRIG               (INT_TRIG),
        .IRQ_IN                 (IPL),
        .IRQ_PEND               (IRQ_PEND),
        .AVECn                  (AVECn),
        .IPENDn                 (IPENDn),
        .IVECT_OFFS             (IVECT_OFFS),

        .TRAP_AERR              (TRAP_AERR),
        .TRAP_BERR              (BERR_MAIN),
        .TRAP_CHK               (TRAP_CHK),
        .TRAP_DIVZERO           (TRAP_DIVZERO),
        .TRAP_ILLEGAL           (TRAP_ILLEGAL),
        .TRAP_CODE_OPC          (TRAP_CODE_OPC),
        .TRAP_VECTOR            (BIW_0[3:0]),
        .TRAP_cc                (TRAP_cc),
        .TRAP_V                 (TRAP_V),
        .TRAP_MMU_CFG           (TRAP_MMU_CFG),
        .TRAP_CP_PRE            (CPIF_TRAP_PRE),
        .TRAP_CP_MID            (CPIF_TRAP_MID),
        .TRAP_CP_POST           (CPIF_TRAP_POST),
        .TRAP_CP_VECTOR         (CPIF_TRAP_VECTOR),
        .EX_TRACE_IN            (EX_TRACE),
        .VBR_WR                 (VBR_WR),
        .VBR                    (VBR)
    );

    // Opcode fetch/decode and loop-mode frontend.
    WF68K30L_OPCODE_DECODER #(
        .NO_LOOP(NO_LOOP)
    ) I_OPCODE_DECODER (
        .CLK                    (CLK),

        .OW_REQ_MAIN            (OW_REQ_MAIN),
        .EW_REQ_MAIN            (EW_REQ_MAIN),

        .EXH_REQ                (EXH_REQ),
        .BUSY_EXH               (BUSY_EXH),
        .BUSY_MAIN              (BUSY_MAIN),
        .BUSY_OPD               (BUSY_OPD),

        .BKPT_INSERT            (BKPT_INSERT),
        .BKPT_DATA              (DATA_TO_CORE[15:0]),

        .LOOP_EXIT              (LOOP_EXIT),
        .LOOP_BSY               (LOOP_BSY),

        .OPD_ACK_MAIN           (OPD_ACK_MAIN),
        .EW_ACK                 (EW_ACK),

        .PC_INC                 (PC_INC),
        .PC_INC_EXH             (PC_INC_EXH_I),
        .PC_ADR_OFFSET          (PC_ADR_OFFSET),
        .PC_EW_OFFSET           (PC_EW_OFFSET),
        .PC_OFFSET              (PC_OFFSET_OPD),

        .OPCODE_RD              (OPCODE_RD),
        .OPCODE_RDY             (OPCODE_RDY),
        .OPCODE_VALID           (OPCODE_VALID),
        .OPCODE_DATA            (OPCODE_TO_CORE),

        .IPIPE_FILL             (IPIPE_FILL),
        .IPIPE_FLUSH            (IPIPE_FLUSH),
        .RTE_PIPE_LOAD          (RTE_PIPE_LOAD),
        .RTE_PIPE_BIW_0         (RTE_PIPE_BIW_0),
        .RTE_PIPE_BIW_1         (RTE_PIPE_BIW_1),
        .RTE_PIPE_BIW_2         (RTE_PIPE_BIW_2),
        .RTE_PIPE_C_FAULT       (RTE_PIPE_C_FAULT),
        .RTE_PIPE_B_FAULT       (RTE_PIPE_B_FAULT),

        // Fault logic:
        .OW_VALID               (OW_VALID),
        .RC                     (RC),
        .RB                     (RB),
        .FC                     (FC),
        .FB                     (FB),

        // Trap logic:
        .SBIT                   (SBIT),
        .TRAP_CODE              (TRAP_CODE_OPC),

        // System control:
        .OP                     (OP),
        .BIW_0                  (BIW_0),
        .BIW_1                  (BIW_1),
        .BIW_2                  (BIW_2),
        .EXT_WORD               (EXT_WORD)
    );
