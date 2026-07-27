//----------------------------------------------------------------------//
//                                                                      //
// WF68K30L IP Core.                                                    //
//                                                                      //
// This is the top level structural design unit of the 68K30L           //
// complex instruction set (CISC) microcontroller. Its programming      //
// model is (hopefully) fully compatible with Motorola's MC68030.       //
// This core features a pipelined architecture.                         //
//                                                                      //
// What is implemented, against the MC68030 user's manual:              //
// - MMU: short and long descriptor table walks over the bus, ATC,      //
//   PMOVE/PTEST/PLOAD/PFLUSH, U and M history bits written back to      //
//   memory, and demand paging on both reads and writes.                 //
// - Caches: direct-mapped instruction and data cache arrays with        //
//   CACR/CAAR controls and TT cache-inhibit integration.                //
// - Bursts: the real CBREQ/CBACK protocol per UM 7.3.7 and 6.1.3.2 --   //
//   address, FC, SIZ and R/W held for the whole burst, one long word    //
//   per STERM, up to four.                                              //
// - RTE bus-fault return: format A/B frame validation, long-format      //
//   version checks, replay of a faulted data write cycle, and replay    //
//   of a faulted data read -- the frame's data input buffer image at    //
//   $2C becomes the suspended instruction's operand (UM 8.2.2), or the  //
//   read is rerun when the handler left DF set (UM 8.2.1/8.2.3).        //
//                                                                      //
// What is NOT implemented:                                             //
// - External coprocessor execution and CIR bus-cycle sequencing. An     //
//   internal CIR model provides pre/mid/post exception-frame plumbing   //
//   and protocol-violation vectors, but these operations are absent:    //
//   cpBcc, cpDBcc, cpGEN, cpRESTORE, cpSAVE, cpScc, cpTRAPcc.          //
// - Cycle accuracy. This is deliberate, not an omission: the pipeline   //
//   differs from the silicon and the shifter is iterative. The shifter  //
//   is a shift register where the 68030's is a barrel shifter.          //
//                                                                      //
// This core also features the 68010 loop operation mode for DBcc loops, //
// a predecessor to the MC68020/30/40 caches.                           //
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
    input  logic        CIINn,
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
