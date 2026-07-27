// ---- Internal signal declarations ----
// File roadmap:
// 1) Flat signal declarations (grouped by role).
// 2) Helper functions (MMU/cache decode, match, and table-walk helpers).
// 3) Sequential state blocks (cache/MMU/register tracking).
// 4) Combinational routing (bus requests, translation, FC/CI/CBREQ generation).
// 5) Submodule instantiations and top-level glue muxes.
//
// Core pipeline, register-file, ALU, and bus control signals.

logic        ADn;
logic [31:0] ADR_CPY_EXH;
logic [31:0] ADR_EFF;
logic [31:0] ADR_EFF_WB;
logic [31:0] ADR_L;
logic [31:0] ADR_LATCH;
logic [2:0]  ADR_MODE;
logic [2:0]  ADR_MODE_MAIN;
logic        ADR_IN_USE;
logic [31:0] ADR_OFFSET;
logic [31:0] ADR_OFFSET_EXH;
logic [5:0]  ADR_OFFSET_MAIN;
logic [31:0] ADR_P;
logic [31:0] ADR_BUS_REQ_PHYS;
logic [31:0] ADR_P_PHYS;
logic [31:0] ADR_P_PHYS_CALC;
logic [31:0] ADR_P_PHYS_LATCH;
logic        ADR_MARK_USED;
logic        AERR;
logic        ALU_ACK;
logic        ALU_BSY;
logic        ALU_COND;
logic        ALU_INIT;
logic        ALU_LOAD_OP1;
logic        ALU_LOAD_OP2;
logic        ALU_LOAD_OP3;
logic [31:0] ALU_OP1_IN;
logic [31:0] ALU_OP2_IN;
logic [31:0] ALU_OP3_IN;
logic        ALU_REQ;
logic [63:0] ALU_RESULT;
logic [2:0]  AMODE_SEL;
logic        AR_DEC;
logic [31:0] AR_IN_1;
logic [31:0] AR_IN_2;
logic        AR_IN_USE;
logic        AR_INC;
logic        AR_MARK_USED;
logic [31:0] AR_OUT_1;
logic [31:0] AR_OUT_2;
logic [2:0]  AR_SEL_RD_1;
logic [2:0]  AR_SEL_RD_1_MAIN;
logic [2:0]  AR_SEL_RD_2;
logic [2:0]  AR_SEL_WR_1;
logic [2:0]  AR_SEL_WR_2;
logic        AR_WR_1;
logic        AR_WR_2;
logic        AVECn_BUSIF;
logic        BERR_MAIN;
logic [4:0]  BITPOS;
logic [15:0] BIW_0;
logic [7:3]  BIW_0_WB_73;
logic [15:0] BIW_1;
logic [15:0] BIW_2;
logic [31:0] BF_OFFSET;
logic [5:0]  BF_WIDTH;
logic        BKPT_CYCLE;
logic        BKPT_INSERT;
logic        BRANCH_ATN;
logic        BUS_BSY;
logic        BUSY_EXH;
logic        BUSY_MAIN;
logic        BUSY_OPD;
logic        CC_UPDT;
logic [31:0] CAAR;
logic        CAAR_RD;
logic        CAAR_WR;
logic [31:0] CACR;
logic        CACR_RD;
logic        CACR_WR;
logic        CACHE_INHIBIT_IN;
logic        CPU_SPACE;
logic        CPU_SPACE_EXH;
logic [2:0]  DFC;
logic        DFC_RD;
logic        DFC_WR;

// MMU programmer-visible registers and ATC/shadow structures.
logic [63:0] MMU_SRP;
logic [63:0] MMU_CRP;
logic [31:0] MMU_TC;
logic [31:0] MMU_TT0;
logic [31:0] MMU_TT1;
logic [31:0] MMU_MMUSR;
logic        MMU_TC_RD;
logic        MMU_TC_WR;
logic        MMU_SRP_RD;
logic        MMU_SRP_WR;
logic        MMU_CRP_RD;
logic        MMU_CRP_WR;
logic        MMU_TT0_RD;
logic        MMU_TT0_WR;
logic        MMU_TT1_RD;
logic        MMU_TT1_WR;
logic        MMU_MMUSR_RD;
logic        MMU_MMUSR_WR;
logic        MMU_ATC_FLUSH;
localparam int MMU_ATC_LINES = 8;
localparam int MMU_ATC_WAYS = 2;
localparam int MMU_ATC_SETS = MMU_ATC_LINES / MMU_ATC_WAYS;
localparam int MMU_ATC_SET_BITS = $clog2(MMU_ATC_SETS);
localparam int MMU_ATC_WAY_BITS = $clog2(MMU_ATC_WAYS);
logic [MMU_ATC_SETS*MMU_ATC_WAYS-1:0] MMU_ATC_V_FLAT;
logic [MMU_ATC_SETS*MMU_ATC_WAYS-1:0] MMU_ATC_B_FLAT;
logic [MMU_ATC_SETS*MMU_ATC_WAYS-1:0] MMU_ATC_W_FLAT;
logic [MMU_ATC_SETS*MMU_ATC_WAYS-1:0] MMU_ATC_M_FLAT;
logic [MMU_ATC_SETS*MMU_ATC_WAYS-1:0] MMU_ATC_CI_FLAT;
logic [MMU_ATC_SETS*MMU_ATC_WAYS*3-1:0] MMU_ATC_FC_FLAT;
logic [MMU_ATC_SETS*MMU_ATC_WAYS*32-1:0] MMU_ATC_TAG_FLAT;
logic [MMU_ATC_SETS*MMU_ATC_WAYS*32-1:0] MMU_ATC_PTAG_FLAT;
// The descriptor-shadow store that used to back the MMU walk helpers is gone:
// every table search -- runtime, PLOAD and PTEST -- reads its descriptors from
// memory over the bus (UM 9.5.2), so there is no internal copy to size.

// Remaining datapath, handshake, trap, and runtime-translation signals.
logic        DR_WR_1;
logic        DR_WR_2;
logic        DR_MARK_USED;
logic [31:0] DATA_FROM_CORE;
logic [31:0] DATA;
logic [31:0] DATA_IN_EXH;
logic [31:0] DATA_IMMEDIATE;
logic [31:0] DATA_EXH;
logic        DATA_RD;
logic        DATA_WR;
logic        DATA_RD_EXH;
logic        DATA_WR_EXH;
logic        DATA_RD_MAIN;
logic        DATA_WR_MAIN;
logic        DATA_RDY;
logic        DATA_RDY_BUSIF;
logic        DATA_RDY_CACHE;
logic        MMU_FAULT_DATA_ACK;
logic        MMU_FAULT_OPCODE_ACK;
logic        MMU_FAULT_SSW_VALID;
logic [8:0]  MMU_FAULT_SSW;
logic [31:0] DATA_TO_CORE;
logic [31:0] DATA_TO_CORE_BUSIF;
logic [31:0] DATA_TO_CORE_CACHE;
logic        DATA_LAST_FROM_CACHE;
logic        DATA_VALID;
logic        DATA_VALID_BUSIF;
logic        DATA_VALID_CACHE;
logic [31:0] DISPLACEMENT;
logic [31:0] DISPLACEMENT_MAIN;
logic [7:0]  DISPLACEMENT_EXH;
logic        DBcc_COND;
logic [31:0] DR_IN_1;
logic [31:0] DR_IN_2;
logic [31:0] DR_OUT_2;
logic [31:0] DR_OUT_1;
logic [2:0]  DR_SEL_WR_1;
logic [2:0]  DR_SEL_WR_2;
logic [2:0]  DR_SEL_RD_1;
logic [2:0]  DR_SEL_RD_2;
logic        DR_IN_USE;
logic        EW_ACK;
logic        EW_REQ_MAIN;
logic        EX_TRACE;
logic        EXH_REQ;
logic [15:0] EXT_WORD;
logic [31:0] FAULT_ADR;
logic        FB;
logic        FC;
logic        FETCH_MEM_ADR;
logic [2:0]  FC_I;
logic [2:0]  FC_BUS_REQ;
logic [2:0]  FC_LATCH;
logic        HILOn;
logic [31:0] INBUFFER;
logic        INT_TRIG;
logic [2:0]  IPL;
logic        ISP_DEC;
logic        ISP_RD;
logic        ISP_LOAD_EXH;
logic        ISP_WR_MAIN;
logic        ISP_WR;
logic [9:0]  IVECT_OFFS;
logic [2:0]  IRQ_PEND;
logic        IPIPE_FILL;
logic        IPIPE_FLUSH;
logic        IPIPE_FLUSH_EXH;
logic        IPIPE_FLUSH_MAIN;
logic        RTE_PIPE_LOAD;
logic [15:0] RTE_PIPE_BIW_0;
logic [15:0] RTE_PIPE_BIW_1;
logic [15:0] RTE_PIPE_BIW_2;
logic        RTE_PIPE_C_FAULT;
logic        RTE_PIPE_B_FAULT;
logic [2:0]  IPIPE_OFFESET;
logic        LOOP_BSY;
logic        LOOP_SPLIT;
logic        LOOP_EXIT;
int          MOVEP_PNTR;
logic        MSP_RD;
logic        MSP_WR;
logic        OPCODE_RD;
logic        OPCODE_RDY;
logic        OPCODE_RDY_BUSIF;
logic        OPCODE_RDY_BUSIF_CORE;
logic        OPCODE_VALID;
logic        OPCODE_VALID_BUSIF;
logic [15:0] OPCODE_TO_CORE;
logic [15:0] OPCODE_TO_CORE_BUSIF;
OP_SIZETYPE  OP_SIZE;
OP_SIZETYPE  OP_SIZE_BUS;
OP_SIZETYPE  OP_SIZE_EXH;
OP_SIZETYPE  OP_SIZE_MAIN;
OP_SIZETYPE  OP_SIZE_WB; // Writeback.
logic        OPCODE_REQ;
logic        OPCODE_REQ_CORE;
logic        OPCODE_REQ_I;
logic        OW_VALID;
logic        OPD_ACK_MAIN;
OP_68K       OP;
OP_68K       OP_WB;
logic        OW_REQ_MAIN;
logic [31:0] OUTBUFFER;
logic [31:0] PC;
logic        PHASE2_MAIN;
logic        PC_ADD_DISPL;
logic [7:0]  PC_ADR_OFFSET;
logic [3:0]  PC_EW_OFFSET;
logic [3:0]  PC_EW_BASE;
logic [31:0] PC_INSTR_EXH;   // PC frozen at exception entry (format $2 IA field).
logic [31:0] PC_WB;          // PC of the instruction owning the writeback stage.
logic        DATA_WR_PENDING; // Core data write outstanding in the previous cycle.
logic [31:0] PC_BF;          // Instruction PC for a format $A/$B bus-fault frame.
logic [31:0] ADR_BF;         // Address the faulted write cycle drove.
logic        BF_IS_WRITE;    // The faulted access was a core data write.
logic        PC_BF_FROZEN;   // PC_BF/ADR_BF hold the faulted access's owner.
logic [31:0] PC_STACKED;     // PC written to the frame at offset $2.
logic [31:0] ADR_STACKED;    // Fault address written to the frame at offset $10.
// Data-write replay support for RTE (UM 8.2.3). The frame carries the
// continuation PC in the format $A/$B internal-register long word at $14 and
// flags replay eligibility in SSW bit 3, one of the SSW's internal-use bits.
logic [31:0] PC_NEXT_WB;     // PC after the instruction owning the writeback stage.
logic [31:0] DOB_WB;         // Write data of the instruction owning the writeback stage.
logic        WB_REPLAYABLE;  // That write is its instruction's last memory transfer.
logic [31:0] PC_CONT_BF;     // Continuation PC for a format $A/$B bus-fault frame.
logic [31:0] DOB_BF;         // Core write data behind the faulted access.
logic        BF_REPLAY_WR;   // Faulted write completable by replaying the cycle.
logic [8:0]  SSW_LOW_STACKED; // SSW[8:0] written to the frame at offset $A.
logic [31:0] DOB_STACKED;    // Output buffer written to the frame at offset $18.
logic        RTE_RERUN_WR;   // The handler is replaying the faulted data write.
logic [31:0] RTE_RERUN_DATA; // Output buffer popped from the frame for the replay.
logic [2:0]  RTE_RERUN_FC;   // Address space of the replay, from the frame's SSW.
// Data-read replay support for RTE (UM 8.2.2). The frame carries the repaired
// operand in the data input buffer image at $2C, flags replay eligibility in SSW
// bit 3 and "the faulted read already updated its address register" in bit 9;
// both bits are UM internal-use bits (Figure 8-9).
logic        DATA_RD_PENDING; // Core data read outstanding in the previous cycle.
logic        RD_REPLAYABLE;   // That read is its instruction's only memory transfer.
logic        RD_AN_UPDATED;   // ... and it has already applied its (An)+/-(An).
logic        BF_REPLAY_RD;    // Faulted read completable by handing over the DIB.
logic        BF_AN_UPDATED;   // The faulted access already updated its An.
logic        RTE_RERUN_RD;      // The handler armed a faulted-read replay.
logic [31:0] RTE_RERUN_RD_DATA; // Operand the replayed read must return.
logic        RTE_RERUN_RD_AN;   // Frame's SSW bit 9, forwarded to the replay.
logic        RTE_DIB_PEND;    // Replay armed, waiting for the operand read.
logic        RTE_DIB_AN_PEND; // One address-register update still to be skipped.
logic        RTE_DIB_ACK;     // Synthesized acknowledge for the replayed read.
logic        RTE_DIB_LAST;    // The last data acknowledge was that synthesized one.
logic [31:0] RTE_DIB_VALUE;   // Operand the synthesized acknowledge returns.
logic        rte_dib_take;    // The resumed instruction's operand read is now.
logic [2:0]  SSW_INTERNAL_HI; // SSW[11:9] written to the frame at offset $A.
logic        RTE_DIB_HOLD;    // Keep the replayed read off the bus.
logic        RTE_DIB_AN_HOLD; // Skip this address-register update.
logic        AR_INC_EFF;      // AR_INC after the replay's one-shot suppression.
logic        AR_DEC_EFF;      // AR_DEC after the replay's one-shot suppression.
logic        CYCLE_STERM_32; // Burst eligibility: 32-bit synchronous cycle.
logic        STERM_NOW;
logic        CBACK_HONOURED; // UM 6.1.3.2: CBACK only counts on a STERM cycle.
logic        PC_INC;
logic        PC_INC_EXH;
logic        PC_INC_EXH_I;
logic [31:0] PC_L;
logic        PC_LOAD;
logic        PC_LOAD_EXH;
logic        PC_LOAD_MAIN;
logic [7:0]  PC_OFFSET;
logic [7:0]  PC_OFFSET_OPD;
logic        PC_RESTORE_EXH;
logic        RB;
logic        RC;
logic        RD_REQ;
logic        RD_REQ_I;
logic        DATA_RD_BUS;
logic        DATA_RDY_BUSIF_CORE;
logic        RMC;
logic        REFILLn_EXH;
logic        RESTORE_ISP_PC;
logic        RESET_CPU;
logic        RESET_IN;
logic        RESET_STRB;
logic        CIOUT_ASSERT;
logic        CIOUT_ASSERT_NOW;
logic        CIOUT_LATCH;
logic        CBREQ_ASSERT;
logic        CBREQ_REQ_NOW;
logic        CBREQ_REQ_LATCH;
logic        CBREQ_INST_REQ_NOW;
logic        CBREQ_DATA_REQ_NOW;
logic        OPCODE_REQ_CORE_MISS;
logic        SP_ADD_DISPL;
logic        SP_ADD_DISPL_EXH;
logic        SP_ADD_DISPL_MAIN;
logic        SBIT;
logic [8:0]  SSW_80;
logic [2:0]  SFC;
logic        SFC_RD;
logic        SFC_WR;
logic [15:0] SR_CPY;
logic        SR_INIT;
logic        SR_CLR_MBIT;
logic        SR_WR;
logic        SR_WR_EXH;
logic        SR_WR_MAIN;
logic [3:0]  STACK_FORMAT;
int          STACK_POS;
logic [15:0] STATUS_REG;
logic        STATUSn_MAIN;
logic        STATUSn_EXH;
logic        STORE_ADR_FORMAT;
logic        STORE_ABS_HI;
logic        STORE_ABS_LO;
logic        STORE_AEFF;
logic        STORE_D16;
logic        STORE_D32_LO;
logic        STORE_D32_HI;
logic        STORE_DISPL;
logic        STORE_MEM_ADR;
logic        STORE_OD_HI;
logic        STORE_OD_LO;
logic        STORE_IDATA_B1;
logic        STORE_IDATA_B2;
logic        TRAP_AERR;
logic        TRAP_ILLEGAL;
TRAPTYPE_OPC TRAP_CODE_OPC;
logic        TRAP_cc;
logic        TRAP_CHK;
logic        TRAP_DIVZERO;
logic        TRAP_V;
logic        TRAP_MMU_CFG;
// Coprocessor interface model (HW-003 phase 4/5, no external coprocessor present).
logic [2:0]  CPIF_STATE;
logic        CPIF_ACTIVE;
logic [2:0]  CPIF_LAST_CAT;
logic [2:0]  CPIF_LAST_CPID;
logic [4:0]  CPIF_LAST_CIR;
logic [15:0] CPIF_LAST_OPWORD;
logic        CPIF_LAST_READ;
logic        CPIF_LAST_WRITE;
logic        CPIF_LAST_RESPONDED;
logic [4:0]  CPIF_LAST_RESP_CIR;
logic [3:0]  CPIF_LAST_PHASE_LEN;
logic [3:0]  CPIF_PHASE_STEPS_LEFT;
logic [3:0]  CPIF_RESP_DELAY_LEFT;
logic [3:0]  CPIF_STEP_INDEX;
logic [4:0]  CPIF_STEP_CIR;
logic        CPIF_STEP_READ;
logic        CPIF_STEP_WRITE;
logic [4:0]  CPIF_RESP_CIR_EXPECT;
logic        CPIF_REQ_PULSE;
logic        CPIF_STEP_PULSE;
logic        CPIF_RESP_PULSE;
logic        CPIF_NORESP_PULSE;
logic        CPIF_TIMEOUT_PULSE;
logic        CPIF_BADRESP_PULSE;
logic [31:0] CPIF_REQ_COUNT;
logic [31:0] CPIF_STEP_COUNT;
logic [31:0] CPIF_RESP_COUNT;
logic [31:0] CPIF_NORESP_COUNT;
logic [31:0] CPIF_TIMEOUT_COUNT;
logic [31:0] CPIF_BADRESP_COUNT;
logic [31:0] CPIF_PRIV_BYPASS_COUNT;
logic [31:0] CPIF_LINEF_NOPROTO_COUNT;
logic        CPIF_MODEL_EXC_ENABLE;
logic        CPIF_MODEL_EXC_ON_NORESP;
logic [1:0]  CPIF_MODEL_EXC_KIND;   // 0=pre, 1=mid, 2=post
logic [7:0]  CPIF_MODEL_EXC_VECTOR;
logic        CPIF_MODEL_RESP_ENABLE;
logic [3:0]  CPIF_MODEL_RESP_DELAY;
logic [4:0]  CPIF_MODEL_RESP_CIR;
logic        CPIF_TRAP_PRE;
logic        CPIF_TRAP_MID;
logic        CPIF_TRAP_POST;
logic [7:0]  CPIF_TRAP_VECTOR;
logic [31:0] CPIF_PRE_EXC_COUNT;
logic [31:0] CPIF_MID_EXC_COUNT;
logic [31:0] CPIF_POST_EXC_COUNT;
logic [31:0] CPIF_PROTO_COUNT;
logic        MMU_RUNTIME_REQ;
logic        MMU_RUNTIME_FAULT;
logic        MMU_RUNTIME_STALL;
logic        MMU_RUNTIME_ATC_REFILL;
logic [2:0]  MMU_RUNTIME_ATC_FC;
logic [31:0] MMU_RUNTIME_ATC_TAG;
logic [31:0] MMU_RUNTIME_ATC_PTAG;
logic        MMU_RUNTIME_ATC_B;
logic        MMU_RUNTIME_ATC_W;
logic        MMU_RUNTIME_ATC_M;
logic        MMU_RUNTIME_ATC_CI;
logic [31:0] MMU_ATC_FLUSH_COUNT;
logic [2:0]  MMU_PTEST_FC;
logic [2:0]  MMU_PTEST_LEVEL;
logic [31:0] MMU_PTEST_LOGICAL;
logic        MMU_PTEST_START;
logic        MMU_PTEST_CONSUME;
logic        MMU_PTEST_ABORT;
logic        MMU_PLOAD_ABORT;
logic        MMU_PTEST_BUSY;
logic        MMU_PTEST_READY;
logic [15:0] MMU_PTEST_WALK_MMUSR;
logic [31:0] MMU_PTEST_DESC_ADDR;
logic        MMU_TWALK_START;
logic        MMU_TWALK_BUSY;
logic        MMU_TWALK_VALID;
logic [2:0]  MMU_TWALK_FC;
logic [31:0] MMU_TWALK_LOGICAL;
logic        MMU_TWALK_WRITE;
logic [35:0] MMU_TWALK_RESULT;
logic [2:0]  MMU_TWALK_STATE;
logic [31:0] MMU_TWALK_TC;
logic [31:0] MMU_TWALK_TABLE_BASE;
logic [31:0] MMU_TWALK_DESC_ADDR;
logic [31:0] MMU_TWALK_PAGE_BASE;
logic [31:0] MMU_TWALK_DESC_PTR;
logic [31:0] MMU_TWALK_INDEX;
logic [5:0]  MMU_TWALK_CONSUMED;
logic [3:0]  MMU_TWALK_DESC_SIZE;
logic [3:0]  MMU_TWALK_NEXT_WIDTH;
logic [2:0]  MMU_TWALK_LEVELS;
logic [2:0]  MMU_TWALK_TLX_LEVEL;
logic [1:0]  MMU_TWALK_DESC_DT;
logic [14:0] MMU_TWALK_ROOT_LIMIT;
logic        MMU_TWALK_ROOT_LIMIT_LOWER;
logic        MMU_TWALK_FC_LEVEL;
logic        MMU_TWALK_HAS_NEXT;
logic        MMU_TWALK_WP_ACCUM;
logic [31:0] MMU_TWALK_FETCH_LO_WORD;
logic [31:0] MMU_TWALK_FETCH_HI_WORD;
logic        MMU_TWALK_FETCH_LO_VALID;
logic        MMU_TWALK_FETCH_HI_VALID;
// MMU table-search bus master (UM 9.5.2).
logic        MMU_TWALK_BUS_RD;
logic        MMU_TWALK_BUS_WR;
logic        MMU_TWALK_BUS_ACTIVE;
logic [31:0] MMU_TWALK_BUS_ADDR;
logic [31:0] MMU_TWALK_BUS_DATA;
logic        MMU_TWALK_RMC;
logic [2:0]  MMU_TWALK_BUS_FC;
logic        BUS_CYCLE_MMU_WALK;
logic        UNMARK;
logic        USE_APAIR;
logic        USE_DFC;
logic        USE_SFC;
logic        USE_DPAIR;
logic        USE_DREG;
logic        USP_RD;
logic        USP_WR;
logic [31:0] VBR;
logic        VBR_WR;
logic        VBR_RD;
logic        WR_REQ;
logic        WR_REQ_I;

// Cache model state: instruction/data lookup, fill, and burst tracking.
logic        ICACHE_HIT_NOW;
logic        ICACHE_RDY;
logic [15:0] ICACHE_OPCODE_WORD;
logic        ICACHE_FILL_PENDING;
logic [31:0] ICACHE_FILL_ADDR;
logic        ICACHE_FILL_CACHEABLE;
logic [2:0]  ICACHE_FILL_FC;
logic        ICACHE_BURST_TRACK_VALID;
logic [3:0]  ICACHE_BURST_TRACK_LINE;
logic [23:0] ICACHE_BURST_TRACK_TAG;
logic        ICACHE_BURST_FILL_VALID;
logic [3:0]  ICACHE_BURST_FILL_LINE;
logic [23:0] ICACHE_BURST_FILL_TAG;
logic [7:0]  ICACHE_BURST_FILL_PENDING;
logic [2:0]  ICACHE_BURST_FILL_FC;
logic [2:0]  ICACHE_BURST_FILL_NEXT_WORD;
logic        ICACHE_HIT_PENDING;
logic        ICACHE_RAM_RD_EN;
logic [6:0]  ICACHE_RAM_RD_ADDR;
logic [15:0] ICACHE_RAM_RD_DATA;
logic        ICACHE_RAM_WR_EN;
logic [6:0]  ICACHE_RAM_WR_ADDR;
logic [15:0] ICACHE_RAM_WR_DATA;
logic        DCACHE_HIT_NOW;
logic        DCACHE_HIT_PENDING;
OP_SIZETYPE  DCACHE_HIT_SIZE_PENDING;
logic [1:0]  DCACHE_HIT_ADDR10_PENDING;
logic        DCACHE_RAM_RD_EN;
logic [5:0]  DCACHE_RAM_RD_ADDR;
logic [31:0] DCACHE_RAM_RD_DATA;
logic        DCACHE_RAM_WR_EN;
logic [5:0]  DCACHE_RAM_WR_ADDR;
logic [31:0] DCACHE_RAM_WR_DATA;
logic        DCACHE_READ_FILL_PENDING;
logic [31:0] DCACHE_READ_FILL_ADDR;
OP_SIZETYPE  DCACHE_READ_FILL_SIZE;
logic        DCACHE_READ_FILL_CACHEABLE;
logic [2:0]  DCACHE_READ_FILL_FC;
logic        DCACHE_BURST_TRACK_VALID;
logic [3:0]  DCACHE_BURST_TRACK_LINE;
logic [23:0] DCACHE_BURST_TRACK_TAG;
logic        DCACHE_BURST_FILL_VALID;
logic [3:0]  DCACHE_BURST_FILL_LINE;
logic [23:0] DCACHE_BURST_FILL_TAG;
logic [3:0]  DCACHE_BURST_FILL_PENDING;
logic [2:0]  DCACHE_BURST_FILL_FC;
logic [1:0]  DCACHE_BURST_FILL_NEXT_ENTRY;
logic        DCACHE_WRITE_PENDING;
logic [31:0] DCACHE_WRITE_ADDR;
OP_SIZETYPE  DCACHE_WRITE_SIZE;
logic [31:0] DCACHE_WRITE_DATA;
logic        DCACHE_WRITE_CACHEABLE;
// Retired background line-completion request path (UM 7.3.7 replaced it with
// address-held burst streaming). Tied off in WF68K30L_TOP_ROUTING_BUS_CACHE; the
// nets remain because WF68K30L_TOP_ROUTING_MMU_TRANSLATE still takes them.
logic        BURST_PREFETCH_OP_REQ;
logic        BURST_PREFETCH_DATA_REQ;
logic [2:0]  BURST_PREFETCH_OP_WORD;
logic [1:0]  BURST_PREFETCH_DATA_ENTRY;
logic [31:0] BURST_PREFETCH_ADDR;
logic [2:0]  BURST_PREFETCH_FC;
logic        BUS_CYCLE_BURST;
logic        BUS_CYCLE_BURST_IS_OP;

// Address-held burst streaming (UM 7.3.7). BURST_REQ_BUSIF is the burst request
// the bus interface arms the engine from -- the same term CBREQn is driven from --
// and the beat port carries one streamed long word per clock to the cache.
logic        BURST_REQ_BUSIF;
logic        BURST_ACTIVE;
logic        BURST_CBREQ_HOLD;
logic        BURST_BEAT_RDY;
logic        BURST_BEAT_FIRST;
logic        BURST_BEAT_IS_OP;
logic [31:0] BURST_BEAT_ADDR;
logic [31:0] BURST_BEAT_DATA;
