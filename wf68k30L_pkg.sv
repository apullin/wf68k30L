// ------------------------------------------------------------------------
// --                                                                    --
// -- WF68K30L IP Core: this is the package file containing the data     --
// -- types and the component declarations.                              --
// --                                                                    --
// -- Author(s):                                                         --
// -- - Wolfgang Foerster, wf@experiment-s.de; wf@inventronik.de         --
// --                                                                    --
// ------------------------------------------------------------------------
// --                                                                    --
// -- Copyright (c) 2014-2019 Wolfgang Foerster Inventronik GmbH.        --
// --                                                                    --
// -- This documentation describes Open Hardware and is licensed          --
// -- under the CERN OHL v. 1.2. You may redistribute and modify         --
// -- this documentation under the terms of the CERN OHL v.1.2.          --
// -- (http://ohwr.org/cernohl). This documentation is distributed       --
// -- WITHOUT ANY EXPRESS OR IMPLIED WARRANTY, INCLUDING OF               --
// -- MERCHANTABILITY, SATISFACTORY QUALITY AND FITNESS FOR A             --
// -- PARTICULAR PURPOSE. Please see the CERN OHL v.1.2 for              --
// -- applicable conditions                                              --
// --                                                                    --
// ------------------------------------------------------------------------
//
// Revision History
//
// Revision 2K14B 20141201 WF
//   Initial Release.
// Later revisions
//   Modifications according to changes of the entity in other modules.
//

package wf68k30l_pkg;

typedef enum logic [1:0] {
    LONG = 2'd0,
    WORD = 2'd1,
    BYTE = 2'd2
} OP_SIZETYPE;

// The OPCODES AND, NOT, OR, ROR and ROL are defined keywords in VHDL. Therefore the assignment is
// AND_B, NOT_B, OR_B, ROTR and ROTL.
typedef enum logic [6:0] {
    ABCD         = 7'd0,
    ADD          = 7'd1,
    ADDA         = 7'd2,
    ADDI         = 7'd3,
    ADDQ         = 7'd4,
    ADDX         = 7'd5,
    AND_B        = 7'd6,
    ANDI         = 7'd7,
    ANDI_TO_CCR  = 7'd8,
    ANDI_TO_SR   = 7'd9,
    ASL          = 7'd10,
    ASR          = 7'd11,
    Bcc          = 7'd12,
    BCHG         = 7'd13,
    BCLR         = 7'd14,
    BFCHG        = 7'd15,
    BFCLR        = 7'd16,
    BFEXTS       = 7'd17,
    BFEXTU       = 7'd18,
    BFFFO        = 7'd19,
    BFINS        = 7'd20,
    BFSET        = 7'd21,
    BFTST        = 7'd22,
    BKPT         = 7'd23,
    BRA          = 7'd24,
    BSET         = 7'd25,
    BSR          = 7'd26,
    BTST         = 7'd27,
    CAS          = 7'd28,
    CAS2         = 7'd29,
    CHK          = 7'd30,
    CHK2         = 7'd31,
    CLR          = 7'd32,
    CMP          = 7'd33,
    CMP2         = 7'd34,
    CMPA         = 7'd35,
    CMPI         = 7'd36,
    CMPM         = 7'd37,
    DBcc         = 7'd38,
    DIVS         = 7'd39,
    DIVU         = 7'd40,
    EOR          = 7'd41,
    EORI         = 7'd42,
    EORI_TO_CCR  = 7'd43,
    EORI_TO_SR   = 7'd44,
    EXG          = 7'd45,
    EXT          = 7'd46,
    EXTB         = 7'd47,
    ILLEGAL      = 7'd48,
    JMP          = 7'd49,
    JSR          = 7'd50,
    LEA          = 7'd51,
    LINK         = 7'd52,
    LSL          = 7'd53,
    LSR          = 7'd54,
    MOVE         = 7'd55,
    MOVE_FROM_CCR = 7'd56,
    MOVE_TO_CCR  = 7'd57,
    MOVE_FROM_SR = 7'd58,
    MOVE_TO_SR   = 7'd59,
    MOVE_USP     = 7'd60,
    MOVEA        = 7'd61,
    MOVEC        = 7'd62,
    MOVEM        = 7'd63,
    MOVEP        = 7'd64,
    MOVEQ        = 7'd65,
    MOVES        = 7'd66,
    MULS         = 7'd67,
    MULU         = 7'd68,
    NBCD         = 7'd69,
    NEG          = 7'd70,
    NEGX         = 7'd71,
    NOP          = 7'd72,
    NOT_B        = 7'd73,
    OR_B         = 7'd74,
    ORI          = 7'd75,
    ORI_TO_CCR   = 7'd76,
    ORI_TO_SR    = 7'd77,
    PACK         = 7'd78,
    PEA          = 7'd79,
    OP_RESET     = 7'd80,
    ROTL         = 7'd81,
    ROTR         = 7'd82,
    ROXL         = 7'd83,
    ROXR         = 7'd84,
    RTD          = 7'd85,
    RTE          = 7'd86,
    RTR          = 7'd87,
    RTS          = 7'd88,
    SBCD         = 7'd89,
    Scc          = 7'd90,
    STOP         = 7'd91,
    SUB          = 7'd92,
    SUBA         = 7'd93,
    SUBI         = 7'd94,
    SUBQ         = 7'd95,
    SUBX         = 7'd96,
    SWAP         = 7'd97,
    TAS          = 7'd98,
    TRAP         = 7'd99,
    TRAPcc       = 7'd100,
    TRAPV        = 7'd101,
    TST          = 7'd102,
    UNLK         = 7'd103,
    UNPK         = 7'd104,
    UNIMPLEMENTED = 7'd105
} OP_68K;

typedef enum logic [2:0] {
    NONE      = 3'd0,
    T_1010    = 3'd1,
    T_1111    = 3'd2,
    T_ILLEGAL = 3'd3,
    T_TRAP    = 3'd4,
    T_PRIV    = 3'd5,
    T_RTE     = 3'd6
} TRAPTYPE_OPC;

// ---- Status Register bit positions ----

// Condition Code Register bit positions
parameter int SR_C = 0;       // Carry
parameter int SR_V = 1;       // Overflow
parameter int SR_Z = 2;       // Zero
parameter int SR_N = 3;       // Negative
parameter int SR_X = 4;       // Extend

// System byte bit positions
parameter int SR_IPL_LO = 8;  // Interrupt priority mask low bit
parameter int SR_IPL_HI = 10; // Interrupt priority mask high bit
parameter int SR_S = 13;      // Supervisor mode
parameter int SR_T = 15;      // Trace mode

// ---- Addressing mode constants ----
parameter logic [2:0] ADR_DN      = 3'b000; // Data register direct
parameter logic [2:0] ADR_AN      = 3'b001; // Address register direct
parameter logic [2:0] ADR_AN_IND  = 3'b010; // Address register indirect
parameter logic [2:0] ADR_AN_POST = 3'b011; // (An)+ post-increment
parameter logic [2:0] ADR_AN_PRE  = 3'b100; // -(An) pre-decrement
parameter logic [2:0] ADR_AN_DISP = 3'b101; // (d16,An) displacement
parameter logic [2:0] ADR_AN_IDX  = 3'b110; // (d8,An,Xn) indexed
parameter logic [2:0] ADR_SPECIAL = 3'b111; // Abs/PC/Imm (sub-selected by reg field)

// ---- Function code constants ----
parameter logic [2:0] FC_USER_DATA  = 3'b001;
parameter logic [2:0] FC_USER_PROG  = 3'b010;
parameter logic [2:0] FC_SUPER_DATA = 3'b101;
parameter logic [2:0] FC_SUPER_PROG = 3'b110;
parameter logic [2:0] FC_CPU_SPACE  = 3'b111;

// ---- Condition code flags struct ----
typedef struct packed {
    logic x; // Extend
    logic n; // Negative
    logic z; // Zero
    logic v; // Overflow
    logic c; // Carry
} ccr_t;

endpackage
