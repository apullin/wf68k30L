//----------------------------------------------------------------------//
//                                                                      //
// WF68K30L IP Core.                                                    //
//                                                                      //
// This is the top level structural design unit of the 68K30L           //
// complex instruction set (CISC) microcontroller. It's program-        //
// ming model is (hopefully) fully compatible with Motorola's           //
// MC68030. This core features a pipelined architecture. In com-        //
// parision to the fully featured 68K30 the core has no full MMU, no    //
// instruction/data cache arrays, and no external coprocessor           //
// interface execution engine. This                                      //
// results in missing burstmodes which are not required due to          //
// lack of cache. Missing coprocessor operations are:                   //
// cpBcc, cpDBcc, cpGEN, cpRESTORE, cpSAVE, cpScc, cpTRAPcc.           //
// MMU support (model scope): decode/privilege handling, PMOVE register   //
// transfers, ATC/MMUSR model for PTEST/PLOAD/PFLUSH semantics, and       //
// runtime translation using short/long descriptor walks.                 //
// Cache support (model scope): direct-mapped I/D cache lookup/fill,      //
// CACR/CAAR invalidation controls, TT CI integration, burst intent/line  //
// tracking, and autonomous background line completion.                   //
// HW-003 phase-4/5 adds an internal coprocessor-CIR model with         //
// no-response tracking and model-scope pre/mid/post exception-frame    //
// plumbing (including protocol-violation vector modeling). Full        //
// external coprocessor execution and CIR bus-cycle sequencing remain    //
// incomplete.                                                           //
// The shifter in the 68K30 is a barrel shifter and in this core        //
// it is a conventional shift register controlled logic.                //
// This core features the loop operation mode of the 68010 to           //
// deal with DBcc loops. This feature is a predecessor to the           //
// MC68020/30/40 caches.                                                //
// RTE bus-fault return support includes format-A/B frame validation,   //
// version checks for long format, and model-scope SSW/pipeline restore //
// behavior for restart-vs-resume handling.                             //
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
    output logic        CIOUTn,
    output logic        CBREQn,
    output logic        DBENn,           // Data buffer enable.
    output logic        BUS_EN,          // Enables ADR, ASn, DSn, RWn, RMCn, FC and SIZE.

    // Synchronous bus control:
    input  logic        CBACKn,
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


// Structural split: keep this file as a readable top-level skeleton.
// Implementation details live in wf68k30L_top_sections/*.svh.
`include "wf68k30L_top_sections/wf68k30L_top_decls.svh"
`include "wf68k30L_top_sections/wf68k30L_top_helpers.svh"
`include "wf68k30L_top_sections/wf68k30L_top_cache_mmu_state.svh"
`include "wf68k30L_top_sections/wf68k30L_top_routing.svh"
`include "wf68k30L_top_sections/wf68k30L_top_submodules.svh"
endmodule
