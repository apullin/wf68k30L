//----------------------------------------------------------------------//
//                                                                      //
// WF68K30L IP Core.                                                    //
//                                                                      //
// This is the top level structural design unit of the 68K30L           //
// complex instruction set (CISC) microcontroller. It's program-        //
// ming model is (hopefully) fully compatible with Motorola's           //
// MC68030. This core features a pipelined architecture. In com-        //
// parision to the fully featured 68K30 the core has no full MMU, no    //
// instruction/data cache arrays, and no coprocessor interface. This    //
// results in missing burstmodes which are not required due to          //
// lack of cache. Missing coprocessor operations are:                   //
// cpBcc, cpDBcc, cpGEN, cpRESTORE, cpSAVE, cpScc, cpTRAPcc.           //
// MMU support is partial: decode/privilege handling, PMOVE register     //
// transfers, minimal translation support (TT matching and root DT=1),   //
// plus a lightweight ATC/MMUSR model for PTEST/PLOAD/PFLUSH semantics.  //
// Cache support is currently limited to CACR/CAAR MOVEC surface         //
// semantics (no internal cache fill/replacement model yet).             //
// The trap handler does not process coprocessor exceptions due to the //
// missing coprocessor interface: PRE_EXC_CP, MID_EXC_CP, POST_EXC_CP, //
// EXC_VECT_CP.                                                         //
// The shifter in the 68K30 is a barrel shifter and in this core        //
// it is a conventional shift register controlled logic.                //
// This core features the loop operation mode of the 68010 to           //
// deal with DBcc loops. This feature is a predecessor to the           //
// MC68020/30/40 caches.                                                //
// The exception handler works for the RTE but without taking the       //
// SSW into account which is intended to restore from a defectice       //
// bus error stack frame.                                               //
//                                                                      //
// Enjoy.                                                               //
//                                                                      //
// Author(s):                                                           //
// - Wolfgang Foerster, wf@experiment-s.de; wf@inventronik.de           //
//                                                                      //
//----------------------------------------------------------------------//
//                                                                      //
// Copyright (c) 2014-2019 Wolfgang Foerster Inventronik GmbH.          //
//                                                                      //
// This documentation describes Open Hardware and is licensed           //
// under the CERN OHL v. 1.2. You may redistribute and modify           //
// this documentation under the terms of the CERN OHL v.1.2.            //
// (http://ohwr.org/cernohl). This documentation is distributed         //
// WITHOUT ANY EXPRESS OR IMPLIED WARRANTY, INCLUDING OF                //
// MERCHANTABILITY, SATISFACTORY QUALITY AND FITNESS FOR A              //
// PARTICULAR PURPOSE. Please see the CERN OHL v.1.2 for                //
// applicable conditions                                                //
//                                                                      //
//----------------------------------------------------------------------//
// Revision History: see original VHDL source for full changelog.
// Revisions 2K14B through 2K19A by Wolfgang Foerster.
//

module WF68K30L_TOP #(
    parameter logic [15:0] VERSION = 16'h1904, // CPU version number.
    // The following two switches are for debugging purposes. Default for both is false.
    parameter NO_PIPELINE = 0,  // If true the main controller work in scalar mode.
    parameter NO_LOOP     = 0   // If true the DBcc loop mechanism is disabled.
) (
    input  logic        CLK,

    // Address and data:
    output logic [31:0] ADR_OUT,
    input  logic [31:0] DATA_IN,
    output logic [31:0] DATA_OUT,
    output logic        DATA_EN,         // Enables the data port.

    // System control:
    input  logic        BERRn,
    input  logic        RESET_INn,
    output logic        RESET_OUT,       // Open drain.
    input  logic        HALT_INn,
    output logic        HALT_OUTn,       // Open drain.

    // Processor status:
    output logic [2:0]  FC_OUT,

    // Interrupt control:
    input  logic        AVECn,
    input  logic [2:0]  IPLn,
    output logic        IPENDn,

    // Aynchronous bus control:
    input  logic [1:0]  DSACKn,
    output logic [1:0]  SIZE,
    output logic        ASn,
    output logic        RWn,
    output logic        RMCn,
    output logic        DSn,
    output logic        ECSn,
    output logic        OCSn,
    output logic        DBENn,           // Data buffer enable.
    output logic        BUS_EN,          // Enables ADR, ASn, DSn, RWn, RMCn, FC and SIZE.

    // Synchronous bus control:
    input  logic        STERMn,

    // Status controls:
    output logic        STATUSn,
    output logic        REFILLn,

    // Bus arbitration control:
    input  logic        BRn,
    output logic        BGn,
    input  logic        BGACKn
);

`include "wf68k30L_pkg.svh"

// ---- Internal signal declarations ----

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
logic [31:0] ADR_P_PHYS;
logic [31:0] ADR_P_PHYS_CALC;
logic [31:0] ADR_P_PHYS_LATCH;
logic        ADR_MARK_UNUSED_MAIN;
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
logic        CPU_SPACE;
logic        CPU_SPACE_EXH;
logic [2:0]  DFC;
logic        DFC_RD;
logic        DFC_WR;
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
logic [31:0] MMU_ATC_FLUSH_COUNT;
localparam int MMU_ATC_LINES = 8;
logic [MMU_ATC_LINES-1:0] MMU_ATC_V;
logic [MMU_ATC_LINES-1:0] MMU_ATC_B;
logic [MMU_ATC_LINES-1:0] MMU_ATC_W;
logic [MMU_ATC_LINES-1:0] MMU_ATC_M;
logic [2:0]  MMU_ATC_FC [0:MMU_ATC_LINES-1];
logic [31:0] MMU_ATC_TAG[0:MMU_ATC_LINES-1];
logic [$clog2(MMU_ATC_LINES)-1:0] MMU_ATC_REPL_PTR;
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
logic [31:0] DATA_BUFFER;
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
logic        EXEC_RDY;
logic        EXH_REQ;
logic [15:0] EXT_WORD;
logic [31:0] FAULT_ADR;
logic        FB;
logic        FC;
logic        FETCH_MEM_ADR;
logic [2:0]  FC_I;
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
logic        IPEND_In;
logic [2:0]  IRQ_PEND;
logic        IPIPE_FILL;
logic        IPIPE_FLUSH;
logic        IPIPE_FLUSH_EXH;
logic        IPIPE_FLUSH_MAIN;
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
logic        RMC;
logic        REFILLn_EXH;
logic        RESTORE_ISP_PC;
logic        RESET_CPU;
logic        RESET_IN;
logic        RESET_STRB;
logic        SP_ADD_DISPL;
logic        SP_ADD_DISPL_EXH;
logic        SP_ADD_DISPL_MAIN;
logic        SBIT;
logic [8:0]  SSW_80;
logic [2:0]  SFC;
logic        SFC_RD;
logic        SFC_WR;
logic [15:0] SR_CPY;
logic        SR_RD;
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

// Minimal cache model state (phase-2): instruction fetch lookup/fill path
// plus cache-control clear-entry bookkeeping.
logic        ICACHE_HIT_NOW;
logic        ICACHE_RDY;
logic [15:0] ICACHE_OPCODE_WORD;
logic        ICACHE_FILL_PENDING;
logic [31:0] ICACHE_FILL_ADDR;
logic        ICACHE_FILL_CACHEABLE;
logic [23:0] ICACHE_TAG [0:15];
logic [7:0]  ICACHE_VALID [0:15];
logic [15:0] ICACHE_DATA [0:15][0:7];
logic        DCACHE_HIT_NOW;
logic [31:0] DCACHE_HIT_DATA_NOW;
logic        DCACHE_HIT_PENDING;
logic [31:0] DCACHE_HIT_DATA_PENDING;
logic [23:0] DCACHE_TAG [0:15];
logic [31:0] DCACHE_DATA [0:15][0:3];
logic [3:0]  DCACHE_VALID [0:15];
logic        DCACHE_READ_FILL_PENDING;
logic [31:0] DCACHE_READ_FILL_ADDR;
OP_SIZETYPE  DCACHE_READ_FILL_SIZE;
logic        DCACHE_READ_FILL_CACHEABLE;
logic        DCACHE_WRITE_PENDING;
logic [31:0] DCACHE_WRITE_ADDR;
OP_SIZETYPE  DCACHE_WRITE_SIZE;
logic [31:0] DCACHE_WRITE_DATA;
logic        DCACHE_WRITE_CACHEABLE;

// ========================================================================
// Exception handler data multiplexer
// ========================================================================

assign DATA_EXH = (STACK_POS == 2) ? {SR_CPY, PC[31:16]} :
                   (STACK_POS == 4) ? {PC[15:0], STACK_FORMAT, 2'b00, IVECT_OFFS} :
                   (STACK_FORMAT == 4'h2 && STACK_POS == 6) ? PC :
                   (STACK_FORMAT == 4'h9 && STACK_POS == 6) ? PC :
                   (STACK_POS == 6) ? {BIW_0, FC, FB, RC, RB, 3'b000, SSW_80} : // Format A and B.
                   (STACK_POS == 8) ? {BIW_1, BIW_2} : // Format A and B.
                   (STACK_FORMAT == 4'h9 && STACK_POS == 10) ? FAULT_ADR :
                   (STACK_POS == 10) ? ADR_CPY_EXH :
                   (STACK_POS == 14) ? OUTBUFFER :
                   (STACK_POS == 20) ? PC + 32'd4 : // Stage B address.
                   (STACK_POS == 24) ? INBUFFER :
                   (STACK_POS == 28) ? {16'h0, VERSION} : 32'h0;

// ========================================================================
// Core data path routing
// ========================================================================

assign DATA_IN_EXH = BUSY_MAIN ? ALU_RESULT[31:0] : DATA_TO_CORE; // MOVEC handles the VBR.

assign DATA_FROM_CORE = BUSY_EXH ? DATA_EXH :
                         (OP_WB == CAS || OP_WB == CAS2) ? DR_OUT_2 :
                         ALU_RESULT[31:0];

function automatic logic mmu_tc_cfg_error(input logic [31:0] tc_in);
    logic [3:0] ps;
    logic [5:0] tl_sum;
    logic [5:0] total;
begin
    mmu_tc_cfg_error = 1'b0;
    if (tc_in[31]) begin
        ps = tc_in[23:20];
        // PS values 0..7 are reserved.
        if (ps < 4'h8)
            mmu_tc_cfg_error = 1'b1;

        // Sum TIA..TID until the first zero field, then add IS and PS.
        tl_sum = {2'b00, tc_in[15:12]};
        if (tc_in[15:12] != 4'h0) begin
            tl_sum = tl_sum + {2'b00, tc_in[11:8]};
            if (tc_in[11:8] != 4'h0) begin
                tl_sum = tl_sum + {2'b00, tc_in[7:4]};
                if (tc_in[7:4] != 4'h0)
                    tl_sum = tl_sum + {2'b00, tc_in[3:0]};
            end
        end
        total = tl_sum + {2'b00, tc_in[19:16]} + {2'b00, ps};
        if (total != 6'd32)
            mmu_tc_cfg_error = 1'b1;
    end
end
endfunction

function automatic logic mmu_tt_match(
    input logic [31:0] tt,
    input logic [2:0]  fc,
    input logic [31:0] addr,
    input logic        read_access,
    input logic        write_access,
    input logic        rmw_access
);
    logic fc_ok;
    logic la_ok;
    logic rw_ok;
begin
    // TT format (MC68030): [31:24] logical base, [23:16] logical mask,
    // [15] E, [10] CI, [9] R/W, [8] RWM, [7] 0, [6:4] FC base, [3] 0, [2:0] FC mask.
    if (!tt[15] || fc == FC_CPU_SPACE) begin
        mmu_tt_match = 1'b0;
    end else begin
        fc_ok = (((fc ^ tt[6:4]) & ~tt[2:0]) == 3'b000);
        la_ok = ((((addr[31:24] ^ tt[31:24]) & ~tt[23:16]) == 8'h00));
        if (rmw_access)
            rw_ok = tt[8];
        else if (tt[8])
            rw_ok = 1'b1;
        else
            rw_ok = (read_access && tt[9]) || (write_access && !tt[9]);
        mmu_tt_match = fc_ok && la_ok && rw_ok;
    end
end
endfunction

function automatic logic [2:0] mmu_fc_decode(
    input logic [4:0]  fc_sel,
    input logic [31:0] dreg_value,
    input logic [2:0]  sfc_value,
    input logic [2:0]  dfc_value
);
begin
    if (fc_sel[4:3] == 2'b10)
        mmu_fc_decode = fc_sel[2:0];
    else if (fc_sel[4:3] == 2'b01)
        mmu_fc_decode = dreg_value[2:0];
    else if (fc_sel == 5'b00000)
        mmu_fc_decode = sfc_value;
    else
        mmu_fc_decode = dfc_value;
end
endfunction

function automatic logic [4:0] mmu_page_shift(input logic [31:0] tc_in);
begin
    if (tc_in[23:20] >= 4'h8)
        mmu_page_shift = {1'b0, tc_in[23:20]};
    else
        mmu_page_shift = 5'd12; // Reserved PS values are treated as 4 Kbytes in this model.
end
endfunction

function automatic logic [31:0] mmu_page_tag(
    input logic [31:0] tc_in,
    input logic [31:0] addr
);
    logic [4:0] shift;
begin
    shift = mmu_page_shift(tc_in);
    mmu_page_tag = addr >> shift;
end
endfunction

function automatic logic mmu_cache_inhibit(
    input logic [2:0]  fc,
    input logic [31:0] addr,
    input logic        read_access,
    input logic        write_access,
    input logic        rmw_access,
    input logic [31:0] tt0,
    input logic [31:0] tt1
);
    logic tt0_hit;
    logic tt1_hit;
begin
    if (fc == FC_CPU_SPACE) begin
        mmu_cache_inhibit = 1'b1;
    end else begin
        tt0_hit = mmu_tt_match(tt0, fc, addr, read_access, write_access, rmw_access);
        tt1_hit = mmu_tt_match(tt1, fc, addr, read_access, write_access, rmw_access);
        mmu_cache_inhibit = (tt0_hit && tt0[10]) || (tt1_hit && tt1[10]); // CI bit.
    end
end
endfunction

function automatic logic dcache_access_supported(
    input OP_SIZETYPE  size_in,
    input logic [1:0]  a10
);
begin
    case (size_in)
        BYTE: dcache_access_supported = 1'b1;
        WORD: dcache_access_supported = !a10[0];
        LONG: dcache_access_supported = (a10 == 2'b00);
        default: dcache_access_supported = 1'b0;
    endcase
end
endfunction

function automatic logic [31:0] dcache_read_extract(
    input logic [31:0] line_word,
    input OP_SIZETYPE  size_in,
    input logic [1:0]  a10
);
begin
    case (size_in)
        LONG: dcache_read_extract = line_word;
        WORD: dcache_read_extract = a10[1] ? {16'h0000, line_word[15:0]} : {16'h0000, line_word[31:16]};
        BYTE: begin
            case (a10)
                2'b00: dcache_read_extract = {24'h000000, line_word[31:24]};
                2'b01: dcache_read_extract = {24'h000000, line_word[23:16]};
                2'b10: dcache_read_extract = {24'h000000, line_word[15:8]};
                default: dcache_read_extract = {24'h000000, line_word[7:0]};
            endcase
        end
        default: dcache_read_extract = 32'h0000_0000;
    endcase
end
endfunction

function automatic logic [31:0] dcache_write_merge(
    input logic [31:0] line_word,
    input logic [31:0] wr_data,
    input OP_SIZETYPE  size_in,
    input logic [1:0]  a10
);
begin
    dcache_write_merge = line_word;
    case (size_in)
        LONG: begin
            if (a10 == 2'b00)
                dcache_write_merge = wr_data;
        end
        WORD: begin
            if (!a10[0]) begin
                if (a10[1])
                    dcache_write_merge = {line_word[31:16], wr_data[15:0]};
                else
                    dcache_write_merge = {wr_data[15:0], line_word[15:0]};
            end
        end
        BYTE: begin
            case (a10)
                2'b00: dcache_write_merge = {wr_data[7:0], line_word[23:0]};
                2'b01: dcache_write_merge = {line_word[31:24], wr_data[7:0], line_word[15:0]};
                2'b10: dcache_write_merge = {line_word[31:16], wr_data[7:0], line_word[7:0]};
                default: dcache_write_merge = {line_word[31:8], wr_data[7:0]};
            endcase
        end
        default: ;
    endcase
end
endfunction

localparam logic [15:0] MMUSR_B = 16'h8000;
localparam logic [15:0] MMUSR_L = 16'h4000;
localparam logic [15:0] MMUSR_S = 16'h2000;
localparam logic [15:0] MMUSR_W = 16'h0800;
localparam logic [15:0] MMUSR_I = 16'h0400;
localparam logic [15:0] MMUSR_M = 16'h0200;
localparam logic [15:0] MMUSR_T = 16'h0040;
localparam logic [15:0] MMUSR_N = 16'h0007;
localparam logic [31:0] CACR_RW_MASK = 32'h0000_3313; // WA,DBE,FD,ED,IBE,FI,EI
localparam int ICACHE_LINES = 16;
localparam int DCACHE_LINES = 16;

always_ff @(posedge CLK) begin : cache_registers
    integer i;
    integer j;
    logic [31:0] cacr_write_value;
    logic [3:0]  caar_line;
    logic [1:0]  caar_entry;
    logic [3:0]  fill_line;
    logic [2:0]  fill_word;
    logic [23:0] fill_tag;
    logic [3:0]  dcache_line;
    logic [1:0]  dcache_entry;
    logic [23:0] dcache_tag;
    logic [31:0] dcache_merged_word;
    logic        dcache_hit;
    if (RESET_CPU) begin
        CACR <= 32'h0;
        CAAR <= 32'h0;
        ICACHE_RDY <= 1'b0;
        ICACHE_OPCODE_WORD <= 16'h0000;
        ICACHE_FILL_PENDING <= 1'b0;
        ICACHE_FILL_ADDR <= 32'h0;
        ICACHE_FILL_CACHEABLE <= 1'b0;
        DATA_RDY_CACHE <= 1'b0;
        DATA_VALID_CACHE <= 1'b0;
        DATA_TO_CORE_CACHE <= 32'h0;
        DATA_LAST_FROM_CACHE <= 1'b0;
        DCACHE_HIT_PENDING <= 1'b0;
        DCACHE_HIT_DATA_PENDING <= 32'h0;
        DCACHE_READ_FILL_PENDING <= 1'b0;
        DCACHE_READ_FILL_ADDR <= 32'h0;
        DCACHE_READ_FILL_SIZE <= LONG;
        DCACHE_READ_FILL_CACHEABLE <= 1'b0;
        DCACHE_WRITE_PENDING <= 1'b0;
        DCACHE_WRITE_ADDR <= 32'h0;
        DCACHE_WRITE_SIZE <= LONG;
        DCACHE_WRITE_DATA <= 32'h0;
        DCACHE_WRITE_CACHEABLE <= 1'b0;
        for (i = 0; i < ICACHE_LINES; i = i + 1) begin
            ICACHE_TAG[i] <= 24'h0;
            ICACHE_VALID[i] <= 8'h00;
            for (j = 0; j < 8; j = j + 1)
                ICACHE_DATA[i][j] <= 16'h0000;
        end
        for (i = 0; i < DCACHE_LINES; i = i + 1) begin
            DCACHE_TAG[i] <= 24'h0;
            DCACHE_VALID[i] <= 4'h0;
            for (j = 0; j < 4; j = j + 1)
                DCACHE_DATA[i][j] <= 32'h0000_0000;
        end
    end else begin
        ICACHE_RDY <= 1'b0;
        DATA_RDY_CACHE <= 1'b0;
        DATA_VALID_CACHE <= 1'b0;

        // Hold the last data-return source between ready strobes.
        if (DATA_RDY_CACHE)
            DATA_LAST_FROM_CACHE <= 1'b1;
        else if (DATA_RDY_BUSIF)
            DATA_LAST_FROM_CACHE <= 1'b0;

        // Serve opcode requests directly from the instruction-cache model.
        if (!BUS_BSY && OPCODE_REQ_CORE && ICACHE_HIT_NOW) begin
            ICACHE_OPCODE_WORD <= ICACHE_DATA[ADR_P_PHYS[7:4]][ADR_P_PHYS[3:1]];
            ICACHE_RDY <= 1'b1;
        end

        // Serve data-cache hits with one-cycle latency to match core handshake timing.
        if (DCACHE_HIT_PENDING) begin
            DATA_TO_CORE_CACHE <= DCACHE_HIT_DATA_PENDING;
            DATA_RDY_CACHE <= 1'b1;
            DATA_VALID_CACHE <= 1'b1;
            DCACHE_HIT_PENDING <= 1'b0;
        end else if (!BUS_BSY && DATA_RD && DCACHE_HIT_NOW && !DATA_RDY_BUSIF) begin
            DCACHE_HIT_DATA_PENDING <= DCACHE_HIT_DATA_NOW;
            DCACHE_HIT_PENDING <= 1'b1;
        end

        // Track in-flight opcode misses to fill cache on bus completion.
        if (!BUS_BSY && OPCODE_REQ) begin
            ICACHE_FILL_PENDING <= 1'b1;
            ICACHE_FILL_ADDR <= ADR_P_PHYS;
            ICACHE_FILL_CACHEABLE <= !mmu_cache_inhibit(FC_I, ADR_P_PHYS, 1'b1, 1'b0, 1'b0, MMU_TT0, MMU_TT1);
        end else if (OPCODE_RDY_BUSIF) begin
            ICACHE_FILL_PENDING <= 1'b0;
        end

        // Track in-flight data reads for cache fill on completion.
        if (!BUS_BSY && DATA_RD_BUS) begin
            DCACHE_READ_FILL_PENDING <= 1'b1;
            DCACHE_READ_FILL_ADDR <= ADR_P_PHYS;
            DCACHE_READ_FILL_SIZE <= OP_SIZE_BUS;
            DCACHE_READ_FILL_CACHEABLE <= !mmu_cache_inhibit(FC_I, ADR_P_PHYS, 1'b1, 1'b0, RMC, MMU_TT0, MMU_TT1);
        end else if (DATA_RDY_BUSIF) begin
            DCACHE_READ_FILL_PENDING <= 1'b0;
        end

        // Track in-flight data writes for write-through cache maintenance.
        if (!BUS_BSY && DATA_WR) begin
            DCACHE_WRITE_PENDING <= 1'b1;
            DCACHE_WRITE_ADDR <= ADR_P_PHYS;
            DCACHE_WRITE_SIZE <= OP_SIZE_BUS;
            DCACHE_WRITE_DATA <= DATA_FROM_CORE;
            DCACHE_WRITE_CACHEABLE <= !mmu_cache_inhibit(FC_I, ADR_P_PHYS, 1'b0, 1'b1, RMC, MMU_TT0, MMU_TT1);
        end else if (DATA_RDY_BUSIF && DCACHE_WRITE_PENDING) begin
            DCACHE_WRITE_PENDING <= 1'b0;
        end

        // Fill one word on each opcode bus response when enabled and unfrozen.
        if (OPCODE_RDY_BUSIF && ICACHE_FILL_PENDING && ICACHE_FILL_CACHEABLE && CACR[0] && !CACR[1]) begin
            fill_line = ICACHE_FILL_ADDR[7:4];
            fill_word = ICACHE_FILL_ADDR[3:1];
            fill_tag = ICACHE_FILL_ADDR[31:8];
            if (ICACHE_TAG[fill_line] != fill_tag)
                ICACHE_VALID[fill_line] <= 8'h00;
            ICACHE_TAG[fill_line] <= fill_tag;
            ICACHE_DATA[fill_line][fill_word] <= OPCODE_TO_CORE_BUSIF;
            ICACHE_VALID[fill_line][fill_word] <= 1'b1;
        end

        // Fill data cache on eligible read misses (long-word aligned only).
        if (DATA_RDY_BUSIF && DCACHE_READ_FILL_PENDING && DCACHE_READ_FILL_CACHEABLE &&
            CACR[8] && !CACR[9] && DCACHE_READ_FILL_SIZE == LONG && DCACHE_READ_FILL_ADDR[1:0] == 2'b00) begin
            dcache_line = DCACHE_READ_FILL_ADDR[7:4];
            dcache_entry = DCACHE_READ_FILL_ADDR[3:2];
            dcache_tag = DCACHE_READ_FILL_ADDR[31:8];
            if (DCACHE_TAG[dcache_line] != dcache_tag)
                DCACHE_VALID[dcache_line] <= 4'h0;
            DCACHE_TAG[dcache_line] <= dcache_tag;
            DCACHE_DATA[dcache_line][dcache_entry] <= DATA_TO_CORE_BUSIF;
            DCACHE_VALID[dcache_line][dcache_entry] <= 1'b1;
        end

        // Write-through updates: hit always updates; WA controls miss allocation.
        if (DATA_RDY_BUSIF && DCACHE_WRITE_PENDING && DCACHE_WRITE_CACHEABLE &&
            CACR[8] && dcache_access_supported(DCACHE_WRITE_SIZE, DCACHE_WRITE_ADDR[1:0])) begin
            dcache_line = DCACHE_WRITE_ADDR[7:4];
            dcache_entry = DCACHE_WRITE_ADDR[3:2];
            dcache_tag = DCACHE_WRITE_ADDR[31:8];
            dcache_hit = (DCACHE_TAG[dcache_line] == dcache_tag) && DCACHE_VALID[dcache_line][dcache_entry];
            dcache_merged_word = dcache_write_merge(
                DCACHE_DATA[dcache_line][dcache_entry],
                DCACHE_WRITE_DATA,
                DCACHE_WRITE_SIZE,
                DCACHE_WRITE_ADDR[1:0]
            );
            if (dcache_hit) begin
                DCACHE_DATA[dcache_line][dcache_entry] <= dcache_merged_word;
                DCACHE_VALID[dcache_line][dcache_entry] <= 1'b1;
            end else if (CACR[13] && !CACR[9] && DCACHE_WRITE_SIZE == LONG && DCACHE_WRITE_ADDR[1:0] == 2'b00) begin
                if (DCACHE_TAG[dcache_line] != dcache_tag)
                    DCACHE_VALID[dcache_line] <= 4'h0;
                DCACHE_TAG[dcache_line] <= dcache_tag;
                DCACHE_DATA[dcache_line][dcache_entry] <= DCACHE_WRITE_DATA;
                DCACHE_VALID[dcache_line][dcache_entry] <= 1'b1;
            end
        end

        if (CACR_WR) begin
            cacr_write_value = ALU_RESULT[31:0];
            caar_line = CAAR[7:4];
            caar_entry = CAAR[3:2];
            CACR <= cacr_write_value & CACR_RW_MASK;

            // Clear-data operations update only the internal validity model.
            if (cacr_write_value[11]) begin // CD
                for (i = 0; i < DCACHE_LINES; i = i + 1)
                    DCACHE_VALID[i] <= 4'h0;
            end
            if (cacr_write_value[10]) // CED
                DCACHE_VALID[caar_line][caar_entry] <= 1'b0;

            // Clear-instruction operations invalidate cache entries immediately.
            if (cacr_write_value[3]) begin // CI
                for (i = 0; i < ICACHE_LINES; i = i + 1)
                    ICACHE_VALID[i] <= 8'h00;
            end
            if (cacr_write_value[2]) begin // CEI
                ICACHE_VALID[caar_line][{caar_entry, 1'b0}] <= 1'b0;
                ICACHE_VALID[caar_line][{caar_entry, 1'b1}] <= 1'b0;
            end
        end

        if (CAAR_WR)
            CAAR <= ALU_RESULT[31:0];
    end
end

always_ff @(posedge CLK) begin : mmu_registers
    logic        tc_cfg_err;
    logic        ptest_exec;
    logic        pload_exec;
    logic        pflush_exec;
    logic        pmove_flush_exec;
    logic [2:0]  fc_sel;
    logic [2:0]  ptest_level;
    logic [2:0]  atc_fc;
    logic [2:0]  atc_mask;
    logic [31:0] atc_logical;
    logic [31:0] atc_tag;
    logic [63:0] root_ptr;
    logic [31:0] root_offs;
    logic [15:0] mmusr_value;
    logic        tt_hit;
    logic        atc_hit;
    logic        atc_free;
    logic        atc_valid_result;
    logic        atc_b_result;
    logic        atc_w_result;
    logic        atc_m_result;
    logic        atc_rmw;
    logic [31:0] atc_phys;
    logic [$clog2(MMU_ATC_LINES)-1:0] atc_hit_idx;
    logic [$clog2(MMU_ATC_LINES)-1:0] atc_free_idx;
    logic [$clog2(MMU_ATC_LINES)-1:0] atc_ins_idx;
    integer i;
    if (RESET_CPU) begin
        MMU_SRP <= 64'h0;
        MMU_CRP <= 64'h0;
        MMU_TC <= 32'h0;
        MMU_TT0 <= 32'h0;
        MMU_TT1 <= 32'h0;
        MMU_MMUSR <= 32'h0;
        TRAP_MMU_CFG <= 1'b0;
        MMU_ATC_FLUSH_COUNT <= 32'h0;
        MMU_ATC_V <= '0;
        MMU_ATC_B <= '0;
        MMU_ATC_W <= '0;
        MMU_ATC_M <= '0;
        MMU_ATC_REPL_PTR <= '0;
        for (i = 0; i < MMU_ATC_LINES; i = i + 1) begin
            MMU_ATC_FC[i] <= 3'b000;
            MMU_ATC_TAG[i] <= 32'h0;
        end
    end else begin
        tc_cfg_err = mmu_tc_cfg_error(ALU_RESULT[31:0]);
        ptest_exec = ALU_ACK && (OP_WB == PTEST);
        pload_exec = ALU_ACK && (OP_WB == PLOAD);
        pmove_flush_exec = MMU_ATC_FLUSH &&
                           (MMU_TC_WR || MMU_SRP_WR || MMU_CRP_WR || MMU_TT0_WR || MMU_TT1_WR);
        pflush_exec = MMU_ATC_FLUSH && !pmove_flush_exec;
        TRAP_MMU_CFG <= 1'b0;
        if (MMU_ATC_FLUSH)
            MMU_ATC_FLUSH_COUNT <= MMU_ATC_FLUSH_COUNT + 32'd1;
        if (MMU_SRP_WR) begin
            MMU_SRP <= ALU_RESULT[63:0];
            if (ALU_RESULT[33:32] == 2'b00)
                TRAP_MMU_CFG <= 1'b1;
        end
        if (MMU_CRP_WR) begin
            MMU_CRP <= ALU_RESULT[63:0];
            if (ALU_RESULT[33:32] == 2'b00)
                TRAP_MMU_CFG <= 1'b1;
        end
        if (MMU_TC_WR) begin
            MMU_TC <= tc_cfg_err ? {1'b0, ALU_RESULT[30:0]} : ALU_RESULT[31:0];
            if (tc_cfg_err)
                TRAP_MMU_CFG <= 1'b1;
        end
        if (MMU_TT0_WR)
            MMU_TT0 <= ALU_RESULT[31:0];
        if (MMU_TT1_WR)
            MMU_TT1 <= ALU_RESULT[31:0];
        if (MMU_MMUSR_WR)
            MMU_MMUSR <= {16'h0, ALU_RESULT[15:0]};

        // PMOVE FD=0 register writes flush all ATC entries.
        if (pmove_flush_exec)
            MMU_ATC_V <= '0;

        // PFLUSH supports all-entries, FC/mask, and FC/mask/EA selection.
        if (pflush_exec) begin
            atc_fc = mmu_fc_decode(BIW_1[4:0], DR_OUT_1, SFC, DFC);
            atc_mask = BIW_1[7:5];
            atc_logical = ADR_EFF;
            atc_tag = mmu_page_tag(MMU_TC, atc_logical);
            case (BIW_1[12:10])
                3'b001: begin
                    MMU_ATC_V <= '0; // PFLUSHA
                end
                3'b100: begin // PFLUSH FC,MASK
                    for (i = 0; i < MMU_ATC_LINES; i = i + 1) begin
                        if (MMU_ATC_V[i] && (((MMU_ATC_FC[i] ^ atc_fc) & atc_mask) == 3'b000))
                            MMU_ATC_V[i] <= 1'b0;
                    end
                end
                3'b110: begin // PFLUSH FC,MASK,<ea>
                    for (i = 0; i < MMU_ATC_LINES; i = i + 1) begin
                        if (MMU_ATC_V[i] &&
                            (((MMU_ATC_FC[i] ^ atc_fc) & atc_mask) == 3'b000) &&
                            (MMU_ATC_TAG[i] == atc_tag))
                            MMU_ATC_V[i] <= 1'b0;
                    end
                end
                default: begin
                end
            endcase
        end

        // PLOAD updates the ATC entry for <FC, logical page>; MMUSR is unchanged.
        if (pload_exec) begin
            atc_fc = mmu_fc_decode(BIW_1[4:0], DR_OUT_1, SFC, DFC);
            atc_logical = ADR_EFF;
            atc_tag = mmu_page_tag(MMU_TC, atc_logical);
            atc_rmw = 1'b0;

            tt_hit = mmu_tt_match(MMU_TT0, atc_fc, atc_logical, BIW_1[9], !BIW_1[9], atc_rmw) ||
                     mmu_tt_match(MMU_TT1, atc_fc, atc_logical, BIW_1[9], !BIW_1[9], atc_rmw);
            root_ptr = (MMU_TC[25] && atc_fc[2]) ? MMU_SRP : MMU_CRP;
            root_offs = {root_ptr[31:4], 4'b0000};
            atc_valid_result = (!tt_hit) && (atc_fc != FC_CPU_SPACE) && (root_ptr[33:32] == 2'b01);
            atc_phys = atc_valid_result ? (atc_logical + root_offs) : 32'h0;
            atc_b_result = !atc_valid_result;
            atc_w_result = 1'b0;
            atc_m_result = !BIW_1[9]; // PLOADW marks modified.

            atc_hit = 1'b0;
            atc_free = 1'b0;
            atc_hit_idx = '0;
            atc_free_idx = '0;
            for (i = 0; i < MMU_ATC_LINES; i = i + 1) begin
                if (!atc_hit && MMU_ATC_V[i] && MMU_ATC_FC[i] == atc_fc && MMU_ATC_TAG[i] == atc_tag) begin
                    atc_hit = 1'b1;
                    atc_hit_idx = i[$clog2(MMU_ATC_LINES)-1:0];
                end
                if (!atc_free && !MMU_ATC_V[i]) begin
                    atc_free = 1'b1;
                    atc_free_idx = i[$clog2(MMU_ATC_LINES)-1:0];
                end
            end

            atc_ins_idx = atc_hit ? atc_hit_idx : (atc_free ? atc_free_idx : MMU_ATC_REPL_PTR);

            // Keep TT-only mappings out of the ATC model.
            if (!tt_hit) begin
                for (i = 0; i < MMU_ATC_LINES; i = i + 1) begin
                    if (MMU_ATC_V[i] && MMU_ATC_FC[i] == atc_fc && MMU_ATC_TAG[i] == atc_tag)
                        MMU_ATC_V[i] <= 1'b0;
                end
                MMU_ATC_V[atc_ins_idx] <= 1'b1;
                MMU_ATC_B[atc_ins_idx] <= atc_b_result;
                MMU_ATC_W[atc_ins_idx] <= atc_w_result;
                MMU_ATC_M[atc_ins_idx] <= atc_m_result;
                MMU_ATC_FC[atc_ins_idx] <= atc_fc;
                MMU_ATC_TAG[atc_ins_idx] <= atc_tag;
                if (!atc_hit && !atc_free)
                    MMU_ATC_REPL_PTR <= MMU_ATC_REPL_PTR + 1'b1;
            end
        end

        // PTEST writes MMUSR with level-specific status.
        if (ptest_exec) begin
            atc_fc = mmu_fc_decode(BIW_1[4:0], DR_OUT_1, SFC, DFC);
            ptest_level = BIW_1[12:10];
            atc_logical = ADR_EFF;
            atc_tag = mmu_page_tag(MMU_TC, atc_logical);
            atc_rmw = 1'b0;

            tt_hit = mmu_tt_match(MMU_TT0, atc_fc, atc_logical, BIW_1[9], !BIW_1[9], atc_rmw) ||
                     mmu_tt_match(MMU_TT1, atc_fc, atc_logical, BIW_1[9], !BIW_1[9], atc_rmw);

            atc_hit = 1'b0;
            atc_hit_idx = '0;
            atc_b_result = 1'b0;
            atc_w_result = 1'b0;
            atc_m_result = 1'b0;
            for (i = 0; i < MMU_ATC_LINES; i = i + 1) begin
                if (!atc_hit && MMU_ATC_V[i] && MMU_ATC_FC[i] == atc_fc && MMU_ATC_TAG[i] == atc_tag) begin
                    atc_hit = 1'b1;
                    atc_hit_idx = i[$clog2(MMU_ATC_LINES)-1:0];
                    atc_b_result = MMU_ATC_B[i];
                    atc_w_result = MMU_ATC_W[i];
                    atc_m_result = MMU_ATC_M[i];
                end
            end

            mmusr_value = 16'h0000;
            if (ptest_level == 3'b000) begin
                if (tt_hit)
                    mmusr_value = MMUSR_T;
                else if (!atc_hit)
                    mmusr_value = MMUSR_I;
                else begin
                    if (atc_b_result)
                        mmusr_value = mmusr_value | MMUSR_B | MMUSR_I;
                    if (atc_w_result)
                        mmusr_value = mmusr_value | MMUSR_W;
                    if (atc_m_result)
                        mmusr_value = mmusr_value | MMUSR_M;
                end
            end else begin
                root_ptr = (MMU_TC[25] && atc_fc[2]) ? MMU_SRP : MMU_CRP;
                if (tt_hit) begin
                    // Level 1-7 PTEST clears T and reports no table-walk condition in this model.
                    mmusr_value = 16'h0000;
                end else if (atc_fc != FC_CPU_SPACE && root_ptr[33:32] == 2'b01) begin
                    // DT=1 root-only translation path: one level observed.
                    mmusr_value = MMUSR_N & 16'h0001;
                end else begin
                    mmusr_value = MMUSR_I;
                end
            end
            MMU_MMUSR <= {16'h0, mmusr_value};
        end
    end
end

// ========================================================================
// Operand routing submodule
// ========================================================================
// ALU operand multiplexers, register file input muxes, bit field
// offset/width/BITPOS, branch prediction, and DBcc condition are all
// handled by the operand mux submodule.

    WF68K30L_OPERAND_MUX I_OPERAND_MUX (
        .CLK                    (CLK),
        .OP                     (OP),
        .OP_WB                  (OP_WB),
        .OP_SIZE                (OP_SIZE),
        .BIW_0                  (BIW_0),
        .BIW_0_WB_73            (BIW_0_WB_73),
        .BIW_1                  (BIW_1),
        .BIW_2                  (BIW_2),
        .ADR_MODE               (ADR_MODE),
        .PHASE2                 (PHASE2_MAIN),
        .DATA_TO_CORE           (DATA_TO_CORE),
        .DR_OUT_1               (DR_OUT_1),
        .DR_OUT_2               (DR_OUT_2),
        .AR_OUT_1               (AR_OUT_1),
        .AR_OUT_2               (AR_OUT_2),
        .ADR_EFF                (ADR_EFF),
        .PC                     (PC),
        .PC_EW_OFFSET           (PC_EW_OFFSET),
        .STATUS_REG             (STATUS_REG),
        .VBR                    (VBR),
        .CACR                   (CACR),
        .CAAR                   (CAAR),
        .SFC                    (SFC),
        .DFC                    (DFC),
        .MMU_SRP                (MMU_SRP),
        .MMU_CRP                (MMU_CRP),
        .MMU_TC                 (MMU_TC),
        .MMU_TT0                (MMU_TT0),
        .MMU_TT1                (MMU_TT1),
        .MMU_MMUSR              (MMU_MMUSR),
        .ALU_RESULT             (ALU_RESULT),
        .STORE_IDATA_B1         (STORE_IDATA_B1),
        .STORE_IDATA_B2         (STORE_IDATA_B2),
        .EXT_WORD               (EXT_WORD),
        .BUSY_MAIN              (BUSY_MAIN),
        .ADR_OFFSET_EXH         (ADR_OFFSET_EXH),
        .ADR_OFFSET_MAIN        (ADR_OFFSET_MAIN),
        .BUSY_EXH               (BUSY_EXH),
        .ALU_BSY                (ALU_BSY),
        .AR_WR_1                (AR_WR_1),
        .DR_WR_1                (DR_WR_1),
        .DFC_WR                 (DFC_WR),
        .SFC_WR                 (SFC_WR),
        .ISP_WR                 (ISP_WR),
        .MSP_WR                 (MSP_WR),
        .USP_WR                 (USP_WR),
        .FETCH_MEM_ADR          (FETCH_MEM_ADR),
        .USE_DREG               (USE_DREG),
        .VBR_RD                 (VBR_RD),
        .CACR_RD                (CACR_RD),
        .CAAR_RD                (CAAR_RD),
        .SFC_RD                 (SFC_RD),
        .DFC_RD                 (DFC_RD),
        .ISP_RD                 (ISP_RD),
        .MSP_RD                 (MSP_RD),
        .USP_RD                 (USP_RD),
        .MMU_SRP_RD             (MMU_SRP_RD),
        .MMU_CRP_RD             (MMU_CRP_RD),
        .MMU_TC_RD              (MMU_TC_RD),
        .MMU_TT0_RD             (MMU_TT0_RD),
        .MMU_TT1_RD             (MMU_TT1_RD),
        .MMU_MMUSR_RD           (MMU_MMUSR_RD),
        .SR_WR_EXH              (SR_WR_EXH),
        .ADn                    (ADn),
        .MOVEP_PNTR             (MOVEP_PNTR),
        .ALU_OP1_IN             (ALU_OP1_IN),
        .ALU_OP2_IN             (ALU_OP2_IN),
        .ALU_OP3_IN             (ALU_OP3_IN),
        .AR_IN_1                (AR_IN_1),
        .AR_IN_2                (AR_IN_2),
        .DR_IN_1                (DR_IN_1),
        .DR_IN_2                (DR_IN_2),
        .BF_OFFSET              (BF_OFFSET),
        .BF_WIDTH               (BF_WIDTH),
        .BITPOS                 (BITPOS),
        .BRANCH_ATN             (BRANCH_ATN),
        .DBcc_COND              (DBcc_COND),
        .DATA_IMMEDIATE         (DATA_IMMEDIATE),
        .ADR_OFFSET             (ADR_OFFSET)
    );

// ========================================================================
// Operand size routing
// ========================================================================

assign OP_SIZE = BUSY_EXH ? OP_SIZE_EXH : OP_SIZE_MAIN;
assign OP_SIZE_BUS = DATA_WR_MAIN ? OP_SIZE_WB : OP_SIZE;

// ========================================================================
// PC-related signals
// ========================================================================

assign PC_OFFSET = PC_OFFSET_OPD;
assign PC_L = PC + PC_ADR_OFFSET;
assign PC_INC_EXH_I = !LOOP_SPLIT ? PC_INC_EXH : 1'b0; // Suppress for a split loop.
assign PC_LOAD = PC_LOAD_EXH || PC_LOAD_MAIN;

// ========================================================================
// Address path control
// ========================================================================

assign ADR_MODE = BUSY_EXH ? 3'b010 : ADR_MODE_MAIN; // (ISP) during exception.
assign SP_ADD_DISPL = SP_ADD_DISPL_MAIN || SP_ADD_DISPL_EXH;
assign AR_SEL_RD_1 = BUSY_EXH ? 3'b111 : AR_SEL_RD_1_MAIN; // ISP during exception.



// ========================================================================
// Bus request arbitration
// ========================================================================

assign DATA_RD = DATA_RD_EXH || DATA_RD_MAIN;
assign DATA_WR = DATA_WR_EXH || DATA_WR_MAIN;

always_ff @(posedge CLK) begin : bus_req_latch
    // Flip-flops break combinatorial loops between core requests and bus controller.
    // Requests are valid until the bus controller enters START_CYCLE and asserts BUS_BSY.
    if (!BUS_BSY) begin
        RD_REQ_I <= DATA_RD_BUS;
        WR_REQ_I <= DATA_WR;
        OPCODE_REQ_I <= OPCODE_RD;
    end else if (BUS_BSY) begin
        RD_REQ_I <= 1'b0;
        WR_REQ_I <= 1'b0;
        OPCODE_REQ_I <= 1'b0;
    end
end

assign RD_REQ = !BUS_BSY ? DATA_RD_BUS : RD_REQ_I;
assign WR_REQ = !BUS_BSY ? DATA_WR : WR_REQ_I;
assign OPCODE_REQ_CORE = !BUS_BSY ? OPCODE_RD : OPCODE_REQ_I;

// Minimal instruction-cache lookup on opcode requests.
always_comb begin : icache_lookup
    logic [3:0]  req_line;
    logic [2:0]  req_word;
    logic [23:0] req_tag;
    logic        req_cacheable;
    ICACHE_HIT_NOW = 1'b0;
    if (!BUS_BSY && OPCODE_REQ_CORE && CACR[0]) begin // EI=1
        req_cacheable = !mmu_cache_inhibit(FC_I, ADR_P_PHYS, 1'b1, 1'b0, 1'b0, MMU_TT0, MMU_TT1);
        req_line = ADR_P_PHYS[7:4];
        req_word = ADR_P_PHYS[3:1];
        req_tag = ADR_P_PHYS[31:8];
        if (req_cacheable && ICACHE_TAG[req_line] == req_tag && ICACHE_VALID[req_line][req_word])
            ICACHE_HIT_NOW = 1'b1;
    end
end

// On an instruction-cache hit, satisfy the opcode fetch internally.
assign OPCODE_REQ = OPCODE_REQ_CORE && !ICACHE_HIT_NOW;

// Minimal data-cache lookup on data read requests.
always_comb begin : dcache_lookup
    logic [3:0]  req_line;
    logic [1:0]  req_entry;
    logic [23:0] req_tag;
    logic        req_cacheable;
    DCACHE_HIT_NOW = 1'b0;
    DCACHE_HIT_DATA_NOW = 32'h0000_0000;
    if (!BUS_BSY && DATA_RD && CACR[8] && !RMC) begin // ED=1 and not RMW read portion.
        req_cacheable = !mmu_cache_inhibit(FC_I, ADR_P_PHYS, 1'b1, 1'b0, 1'b0, MMU_TT0, MMU_TT1);
        req_line = ADR_P_PHYS[7:4];
        req_entry = ADR_P_PHYS[3:2];
        req_tag = ADR_P_PHYS[31:8];
        if (req_cacheable &&
            dcache_access_supported(OP_SIZE_BUS, ADR_P_PHYS[1:0]) &&
            DCACHE_TAG[req_line] == req_tag &&
            DCACHE_VALID[req_line][req_entry]) begin
            DCACHE_HIT_NOW = 1'b1;
            DCACHE_HIT_DATA_NOW = dcache_read_extract(
                DCACHE_DATA[req_line][req_entry],
                OP_SIZE_BUS,
                ADR_P_PHYS[1:0]
            );
        end
    end
end

assign DATA_RD_BUS = DATA_RD && !DCACHE_HIT_NOW;

// ========================================================================
// Miscellaneous signal multiplexing
// ========================================================================

assign DISPLACEMENT = BUSY_MAIN ? DISPLACEMENT_MAIN : {24'h0, DISPLACEMENT_EXH};
assign SR_WR = SR_WR_EXH || SR_WR_MAIN;
assign IPIPE_FLUSH = IPIPE_FLUSH_EXH || IPIPE_FLUSH_MAIN;
assign ISP_WR = ISP_WR_MAIN || ISP_LOAD_EXH;
assign AVECn_BUSIF = BUSY_EXH ? AVECn : 1'b1;

assign CPU_SPACE = (OP == BKPT && DATA_RD_MAIN) ? 1'b1 :
                    BUSY_EXH ? CPU_SPACE_EXH : 1'b0;


// ========================================================================
// Trap and function code signals
// ========================================================================

assign TRAP_AERR = !BUSY_EXH ? AERR : 1'b0; // No address error from the system during exception processing.
assign USE_DFC = (OP_WB == MOVES && DATA_WR_MAIN);
assign USE_SFC = (OP_WB == MOVES && DATA_RD_MAIN);

// ========================================================================
// Input synchronization
// ========================================================================

assign RESET_IN = ~RESET_INn;
assign IPL = ~IPLn;

// ========================================================================
// Status output (active on negedge CLK)
// ========================================================================

always_ff @(negedge CLK) begin : refill_status
    STATUSn <= !(STATUSn_EXH && STATUSn_MAIN) ? 1'b0 : 1'b1;
    REFILLn <= REFILLn_EXH;
end

// ========================================================================
// Status register and address bus
// ========================================================================

assign SBIT = STATUS_REG[13];

assign ADR_L = BKPT_CYCLE ? {24'h0, 3'b000, BIW_0[2:0], 2'b00} :
                CPU_SPACE_EXH ? {28'hFFFFFFF, IRQ_PEND, 1'b1} :
                DATA_WR_MAIN ? ADR_EFF_WB : ADR_EFF;

assign ADR_P = BUS_BSY ? ADR_LATCH :
                (DATA_RD || DATA_WR) ? ADR_L : PC_L;

always_comb begin : mmu_address_translate
    logic        read_access;
    logic        write_access;
    logic        rmw_access;
    logic        tt_hit;
    logic [63:0] root_ptr;
    logic [31:0] root_offs;

    read_access = OPCODE_RD || DATA_RD;
    write_access = DATA_WR;
    rmw_access = RMC;

    root_ptr = (MMU_TC[25] && FC_I[2]) ? MMU_SRP : MMU_CRP; // SRE + supervisor access.
    root_offs = {root_ptr[31:4], 4'b0000}; // DT=1 constant offset.

    tt_hit = mmu_tt_match(MMU_TT0, FC_I, ADR_P, read_access, write_access, rmw_access) ||
             mmu_tt_match(MMU_TT1, FC_I, ADR_P, read_access, write_access, rmw_access);

    if (tt_hit)
        ADR_P_PHYS_CALC = ADR_P;
    else if (MMU_TC[31] && FC_I != FC_CPU_SPACE && root_ptr[33:32] == 2'b01)
        ADR_P_PHYS_CALC = ADR_P + root_offs;
    else
        ADR_P_PHYS_CALC = ADR_P;
end

assign ADR_P_PHYS = BUS_BSY ? ADR_P_PHYS_LATCH : ADR_P_PHYS_CALC;

// Address and fault address latches.
always_ff @(posedge CLK) begin : adr_latches
    if (!BUS_BSY) begin
        ADR_LATCH <= ADR_P;
        ADR_P_PHYS_LATCH <= ADR_P_PHYS_CALC;
        FC_LATCH <= FC_I;
    end else if (!BERRn) begin
        FAULT_ADR <= ADR_LATCH;
    end
end

// Function code generation.
always_comb begin : fc_generation
    if (BUS_BSY)
        FC_I = FC_LATCH;
    else if (USE_SFC)
        FC_I = SFC;
    else if (USE_DFC)
        FC_I = DFC;
    else if ((DATA_RD || DATA_WR) && CPU_SPACE)
        FC_I = FC_CPU_SPACE;
    else if ((DATA_RD || DATA_WR) && SBIT)
        FC_I = FC_SUPER_DATA;
    else if (DATA_RD || DATA_WR)
        FC_I = FC_USER_DATA;
    else if (OPCODE_RD && SBIT)
        FC_I = FC_SUPER_PROG;
    else
        FC_I = FC_USER_PROG;
end

// ========================================================================
// Submodule instantiations
// ========================================================================

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

    WF68K30L_BUS_INTERFACE I_BUS_IF (
        .CLK                (CLK),

        .ADR_IN_P           (ADR_P_PHYS),
        .ADR_OUT_P          (ADR_OUT),

        .FC_IN              (FC_I),
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

    // Data source mux: bus interface or data-cache hit path.
    assign DATA_RDY = DATA_RDY_BUSIF || DATA_RDY_CACHE;
    assign DATA_VALID = DATA_RDY_CACHE ? DATA_VALID_CACHE : DATA_VALID_BUSIF;
    assign DATA_TO_CORE = DATA_RDY_CACHE ? DATA_TO_CORE_CACHE :
                          DATA_RDY_BUSIF ? DATA_TO_CORE_BUSIF :
                          DATA_LAST_FROM_CACHE ? DATA_TO_CORE_CACHE : DATA_TO_CORE_BUSIF;

    // Opcode source mux: bus interface or instruction-cache hit path.
    assign OPCODE_RDY = OPCODE_RDY_BUSIF || ICACHE_RDY;
    assign OPCODE_VALID = ICACHE_RDY ? 1'b1 : OPCODE_VALID_BUSIF;
    assign OPCODE_TO_CORE = ICACHE_RDY ? ICACHE_OPCODE_WORD : OPCODE_TO_CORE_BUSIF;

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
        .EX_TRACE_IN            (EX_TRACE),
        .VBR_WR                 (VBR_WR),
        .VBR                    (VBR)
    );

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
endmodule
