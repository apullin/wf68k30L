"""
MC68030 instruction encoder for cocotb testbench.

Encodes MC68030 instructions into lists of 16-bit words matching the
binary format expected by the WF68K30L opcode decoder.

Each encoding function returns list[int] where each element is a 16-bit word.
The first element is always the opcode word; subsequent elements are extension
words, immediate data, or displacement values.

Reference:
  - MC68030 User Manual (MC68030UM.md), instruction encoding tables
  - wf68k30L_decode_functions.svh for decoder expectations
  - wf68k30L_pkg.svh for OP_68K enum values

MC68030 instruction word layout (varies per instruction group):
  Bits 15-12: Opcode group
  Bits 11-6:  Opcode subfields (varies)
  Bits 5-3:   EA mode
  Bits 2-0:   EA register

Size field encoding (for most instructions):
  00 = byte, 01 = word, 10 = long

EA mode encoding:
  000 = Dn           001 = An           010 = (An)         011 = (An)+
  100 = -(An)        101 = (d16,An)     110 = (d8,An,Xn)   111 = special

EA special register field:
  000 = abs.W        001 = abs.L        010 = (d16,PC)     011 = (d8,PC,Xn)
  100 = #imm

IMPORTANT: MC68030 MOVE instruction uses a *swapped* destination encoding:
  Bits 11-9 = destination register
  Bits 8-6  = destination mode
  (This is reversed compared to the source EA field encoding.)
"""

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

# Operand sizes (as used in most 68K instruction size fields)
BYTE = 0  # 00
WORD = 1  # 01
LONG = 2  # 10

# EA mode constants
DN = 0         # 000 - Data register direct
AN = 1         # 001 - Address register direct
AN_IND = 2     # 010 - Address register indirect
AN_POSTINC = 3 # 011 - (An)+ post-increment
AN_PREDEC = 4  # 100 - -(An) pre-decrement
AN_DISP = 5    # 101 - (d16,An) displacement
AN_IDX = 6     # 110 - (d8,An,Xn) indexed
SPECIAL = 7    # 111 - Special (abs.W, abs.L, PC-relative, immediate)

# EA special register field values (when mode == SPECIAL)
ABS_W = 0      # Absolute word
ABS_L = 1      # Absolute long
PC_DISP = 2    # (d16,PC)
PC_IDX = 3     # (d8,PC,Xn)
IMMEDIATE = 4  # #imm

# Condition codes for Bcc/DBcc/Scc
CC_T = 0    # True (always)
CC_F = 1    # False (never)
CC_HI = 2   # High
CC_LS = 3   # Low or Same
CC_CC = 4   # Carry Clear (HS)
CC_CS = 5   # Carry Set (LO)
CC_NE = 6   # Not Equal
CC_EQ = 7   # Equal
CC_VC = 8   # Overflow Clear
CC_VS = 9   # Overflow Set
CC_PL = 10  # Plus
CC_MI = 11  # Minus
CC_GE = 12  # Greater or Equal
CC_LT = 13  # Less Than
CC_GT = 14  # Greater Than
CC_LE = 15  # Less or Equal

# EXG mode field values
EXG_DD = 0b01000  # Exchange Dx, Dy (data-data)
EXG_AA = 0b01001  # Exchange Ax, Ay (address-address)
EXG_DA = 0b10001  # Exchange Dx, Ay (data-address)

# ---------------------------------------------------------------------------
# Internal helpers
# ---------------------------------------------------------------------------

def _mask(val, bits):
    """Mask value to given number of bits."""
    return val & ((1 << bits) - 1)


def _w(val):
    """Ensure value fits in 16 bits."""
    return val & 0xFFFF


def _signed_byte(val):
    """Convert a Python int to 8-bit signed representation."""
    return val & 0xFF


def _signed_word(val):
    """Convert a Python int to 16-bit signed representation."""
    return val & 0xFFFF


def _signed_long(val):
    """Convert a Python int to 32-bit unsigned representation."""
    return val & 0xFFFFFFFF


# ---------------------------------------------------------------------------
# Immediate data helpers
# ---------------------------------------------------------------------------

def imm_byte(val):
    """Encode a byte immediate as one extension word (value in low byte)."""
    return [_w(val & 0xFF)]


def imm_word(val):
    """Encode a word immediate as one extension word."""
    return [_w(val)]


def imm_long(val):
    """Encode a long immediate as two extension words (high, low)."""
    v = _signed_long(val)
    return [_w(v >> 16), _w(v)]


def abs_word(addr):
    """Encode an absolute word address as one extension word."""
    return [_w(addr)]


def abs_long(addr):
    """Encode an absolute long address as two extension words (high, low)."""
    a = _signed_long(addr)
    return [_w(a >> 16), _w(a)]


def disp16(val):
    """Encode a 16-bit displacement as one extension word."""
    return [_w(val)]


def _size_field(size):
    """Map our BYTE/WORD/LONG constants to the 2-bit size field.

    Most 68K instructions use: 00=byte, 01=word, 10=long.
    """
    if size == BYTE:
        return 0b00
    elif size == WORD:
        return 0b01
    elif size == LONG:
        return 0b10
    else:
        raise ValueError(f"Invalid size: {size}")


def _imm_for_size(size, imm):
    """Return immediate extension words for the given operand size."""
    if size == BYTE:
        return imm_byte(imm)
    elif size == WORD:
        return imm_word(imm)
    elif size == LONG:
        return imm_long(imm)
    else:
        raise ValueError(f"Invalid size: {size}")


def _ea_extension_words(mode, reg, size=None, imm=None, disp=None, addr=None):
    """Return extension words needed for the given EA mode.

    This is a convenience function for when you want to include EA extension
    words automatically. For most test usage, callers will assemble words
    manually.

    Args:
        mode: EA mode (0-7)
        reg: EA register (0-7)
        size: Operand size (BYTE/WORD/LONG) for immediate mode
        imm: Immediate value (for mode=SPECIAL, reg=IMMEDIATE)
        disp: Displacement value (for AN_DISP, PC_DISP)
        addr: Address value (for ABS_W, ABS_L)
    """
    words = []
    if mode == AN_DISP or (mode == SPECIAL and reg == PC_DISP):
        if disp is not None:
            words.extend(disp16(disp))
    elif mode == SPECIAL and reg == ABS_W:
        if addr is not None:
            words.extend(abs_word(addr))
    elif mode == SPECIAL and reg == ABS_L:
        if addr is not None:
            words.extend(abs_long(addr))
    elif mode == SPECIAL and reg == IMMEDIATE:
        if imm is not None and size is not None:
            words.extend(_imm_for_size(size, imm))
    return words


# ---------------------------------------------------------------------------
# Data movement instructions
# ---------------------------------------------------------------------------

def nop():
    """NOP - No Operation.

    Encoding: 0100 1110 0111 0001 = 0x4E71
    """
    return [0x4E71]


def moveq(data8, dn):
    """MOVEQ #<data8>,Dn - Move Quick.

    Encoding: 0111 Dn 0 data8
      Bits 15-12: 0111
      Bits 11-9:  Dn (destination data register)
      Bit 8:      0
      Bits 7-0:   data8 (8-bit signed immediate)
    """
    return [_w(0x7000 | (_mask(dn, 3) << 9) | _signed_byte(data8))]


def move(size, src_mode, src_reg, dst_mode, dst_reg):
    """MOVE <ea>,<ea> - Move Data.

    The MOVE instruction has a unique encoding where the destination
    register and mode fields are SWAPPED compared to the source:
      Bits 15-14: size (01=byte, 11=word, 10=long) -- NOTE: different from usual!
      Bits 13-12: 00
      Bits 11-9:  destination register
      Bits 8-6:   destination mode
      Bits 5-3:   source mode
      Bits 2-0:   source register

    MOVE size field encoding:
      BYTE=01, WORD=11, LONG=10
    """
    if size == BYTE:
        sz = 0b01
    elif size == WORD:
        sz = 0b11
    elif size == LONG:
        sz = 0b10
    else:
        raise ValueError(f"Invalid size: {size}")

    opcode = ((sz << 12) |
              (_mask(dst_reg, 3) << 9) |
              (_mask(dst_mode, 3) << 6) |
              (_mask(src_mode, 3) << 3) |
              _mask(src_reg, 3))
    return [_w(opcode)]


def movea(size, ea_mode, ea_reg, an):
    """MOVEA <ea>,An - Move Address.

    Encoding is the same as MOVE but destination mode = 001 (An).
    Only WORD and LONG sizes are valid.
      WORD: size field = 11
      LONG: size field = 10
    """
    return move(size, ea_mode, ea_reg, AN, an)


def move_to_ccr(ea_mode, ea_reg):
    """MOVE <ea>,CCR - Move to Condition Code Register.

    Encoding: 0100 0100 11 ea_mode ea_reg
    """
    opcode = 0x44C0 | (_mask(ea_mode, 3) << 3) | _mask(ea_reg, 3)
    return [_w(opcode)]


def move_from_ccr(ea_mode, ea_reg):
    """MOVE CCR,<ea> - Move from Condition Code Register.

    Encoding: 0100 0010 11 ea_mode ea_reg
    """
    opcode = 0x42C0 | (_mask(ea_mode, 3) << 3) | _mask(ea_reg, 3)
    return [_w(opcode)]


def move_from_sr(ea_mode, ea_reg):
    """MOVE SR,<ea> - Move from Status Register.

    Encoding: 0100 0000 11 ea_mode ea_reg
    """
    opcode = 0x40C0 | (_mask(ea_mode, 3) << 3) | _mask(ea_reg, 3)
    return [_w(opcode)]


def move_to_sr(ea_mode, ea_reg):
    """MOVE <ea>,SR - Move to Status Register.

    Encoding: 0100 0110 11 ea_mode ea_reg
    """
    opcode = 0x46C0 | (_mask(ea_mode, 3) << 3) | _mask(ea_reg, 3)
    return [_w(opcode)]


def swap(dn):
    """SWAP Dn - Swap Register Halves.

    Encoding: 0100 1000 0100 0 Dn
    """
    return [_w(0x4840 | _mask(dn, 3))]


def exg(rx, ry, mode):
    """EXG Rx,Ry - Exchange Registers.

    Encoding: 1100 Rx 1 opmode Ry
      opmode:
        01000 = Dx,Dy  (data-data)
        01001 = Ax,Ay  (address-address)
        10001 = Dx,Ay  (data-address)
    """
    opcode = (0xC100 |
              (_mask(rx, 3) << 9) |
              (_mask(mode, 5) << 3) |
              _mask(ry, 3))
    return [_w(opcode)]


def ext_w(dn):
    """EXT.W Dn - Sign Extend Byte to Word.

    Encoding: 0100 100 010 000 Dn
    """
    return [_w(0x4880 | _mask(dn, 3))]


def ext_l(dn):
    """EXT.L Dn - Sign Extend Word to Long.

    Encoding: 0100 100 011 000 Dn
    """
    return [_w(0x48C0 | _mask(dn, 3))]


def extb(dn):
    """EXTB.L Dn - Sign Extend Byte to Long (MC68020+).

    Encoding: 0100 100 111 000 Dn
    """
    return [_w(0x49C0 | _mask(dn, 3))]


def lea(ea_mode, ea_reg, an):
    """LEA <ea>,An - Load Effective Address.

    Encoding: 0100 An 111 ea_mode ea_reg
    """
    opcode = (0x41C0 |
              (_mask(an, 3) << 9) |
              (_mask(ea_mode, 3) << 3) |
              _mask(ea_reg, 3))
    return [_w(opcode)]


def pea(ea_mode, ea_reg):
    """PEA <ea> - Push Effective Address.

    Encoding: 0100 1000 01 ea_mode ea_reg
    """
    opcode = 0x4840 | (_mask(ea_mode, 3) << 3) | _mask(ea_reg, 3)
    return [_w(opcode)]


# ---------------------------------------------------------------------------
# Arithmetic instructions
# ---------------------------------------------------------------------------

def add(size, dn, direction, ea_mode, ea_reg):
    """ADD - Add Binary.

    Encoding: 1101 Dn opmode ea_mode ea_reg
      opmode:
        direction=0 (EA to Dn): 000=byte, 001=word, 010=long
        direction=1 (Dn to EA): 100=byte, 101=word, 110=long
    """
    opmode = _size_field(size) | (direction << 2)
    opcode = (0xD000 |
              (_mask(dn, 3) << 9) |
              (_mask(opmode, 3) << 6) |
              (_mask(ea_mode, 3) << 3) |
              _mask(ea_reg, 3))
    return [_w(opcode)]


def adda(size, ea_mode, ea_reg, an):
    """ADDA <ea>,An - Add Address.

    Encoding: 1101 An opmode ea_mode ea_reg
      opmode: 011=word, 111=long
    """
    opmode = 0b011 if size == WORD else 0b111
    opcode = (0xD000 |
              (_mask(an, 3) << 9) |
              (_mask(opmode, 3) << 6) |
              (_mask(ea_mode, 3) << 3) |
              _mask(ea_reg, 3))
    return [_w(opcode)]


def addi(size, ea_mode, ea_reg, imm):
    """ADDI #<data>,<ea> - Add Immediate.

    Encoding: 0000 0110 size ea_mode ea_reg
    Followed by immediate data word(s).
    """
    sz = _size_field(size)
    opcode = (0x0600 |
              (sz << 6) |
              (_mask(ea_mode, 3) << 3) |
              _mask(ea_reg, 3))
    return [_w(opcode)] + _imm_for_size(size, imm)


def addq(size, data3, ea_mode, ea_reg):
    """ADDQ #<data>,<ea> - Add Quick.

    Encoding: 0101 data 0 size ea_mode ea_reg
    data3: 1-8, where 8 is encoded as 0.
    """
    d = _mask(data3, 3) if data3 != 8 else 0
    sz = _size_field(size)
    opcode = (0x5000 |
              (d << 9) |
              (sz << 6) |
              (_mask(ea_mode, 3) << 3) |
              _mask(ea_reg, 3))
    return [_w(opcode)]


def sub(size, dn, direction, ea_mode, ea_reg):
    """SUB - Subtract Binary.

    Encoding: 1001 Dn opmode ea_mode ea_reg
    Same opmode structure as ADD.
    """
    opmode = _size_field(size) | (direction << 2)
    opcode = (0x9000 |
              (_mask(dn, 3) << 9) |
              (_mask(opmode, 3) << 6) |
              (_mask(ea_mode, 3) << 3) |
              _mask(ea_reg, 3))
    return [_w(opcode)]


def suba(size, ea_mode, ea_reg, an):
    """SUBA <ea>,An - Subtract Address.

    Encoding: 1001 An opmode ea_mode ea_reg
      opmode: 011=word, 111=long
    """
    opmode = 0b011 if size == WORD else 0b111
    opcode = (0x9000 |
              (_mask(an, 3) << 9) |
              (_mask(opmode, 3) << 6) |
              (_mask(ea_mode, 3) << 3) |
              _mask(ea_reg, 3))
    return [_w(opcode)]


def subi(size, ea_mode, ea_reg, imm):
    """SUBI #<data>,<ea> - Subtract Immediate.

    Encoding: 0000 0100 size ea_mode ea_reg
    """
    sz = _size_field(size)
    opcode = (0x0400 |
              (sz << 6) |
              (_mask(ea_mode, 3) << 3) |
              _mask(ea_reg, 3))
    return [_w(opcode)] + _imm_for_size(size, imm)


def subq(size, data3, ea_mode, ea_reg):
    """SUBQ #<data>,<ea> - Subtract Quick.

    Encoding: 0101 data 1 size ea_mode ea_reg
    """
    d = _mask(data3, 3) if data3 != 8 else 0
    sz = _size_field(size)
    opcode = (0x5100 |
              (d << 9) |
              (sz << 6) |
              (_mask(ea_mode, 3) << 3) |
              _mask(ea_reg, 3))
    return [_w(opcode)]


def neg(size, ea_mode, ea_reg):
    """NEG <ea> - Negate.

    Encoding: 0100 0100 size ea_mode ea_reg
    """
    sz = _size_field(size)
    opcode = (0x4400 |
              (sz << 6) |
              (_mask(ea_mode, 3) << 3) |
              _mask(ea_reg, 3))
    return [_w(opcode)]


def negx(size, ea_mode, ea_reg):
    """NEGX <ea> - Negate with Extend.

    Encoding: 0100 0000 size ea_mode ea_reg
    """
    sz = _size_field(size)
    opcode = (0x4000 |
              (sz << 6) |
              (_mask(ea_mode, 3) << 3) |
              _mask(ea_reg, 3))
    return [_w(opcode)]


def clr(size, ea_mode, ea_reg):
    """CLR <ea> - Clear an Operand.

    Encoding: 0100 0010 size ea_mode ea_reg
    """
    sz = _size_field(size)
    opcode = (0x4200 |
              (sz << 6) |
              (_mask(ea_mode, 3) << 3) |
              _mask(ea_reg, 3))
    return [_w(opcode)]


def cmp_reg(size, dn, ea_mode, ea_reg):
    """CMP <ea>,Dn - Compare.

    Encoding: 1011 Dn opmode ea_mode ea_reg
      opmode: 000=byte, 001=word, 010=long
    """
    opmode = _size_field(size)
    opcode = (0xB000 |
              (_mask(dn, 3) << 9) |
              (_mask(opmode, 3) << 6) |
              (_mask(ea_mode, 3) << 3) |
              _mask(ea_reg, 3))
    return [_w(opcode)]


def cmpa(size, ea_mode, ea_reg, an):
    """CMPA <ea>,An - Compare Address.

    Encoding: 1011 An opmode ea_mode ea_reg
      opmode: 011=word, 111=long
    """
    opmode = 0b011 if size == WORD else 0b111
    opcode = (0xB000 |
              (_mask(an, 3) << 9) |
              (_mask(opmode, 3) << 6) |
              (_mask(ea_mode, 3) << 3) |
              _mask(ea_reg, 3))
    return [_w(opcode)]


def cmpi(size, ea_mode, ea_reg, imm):
    """CMPI #<data>,<ea> - Compare Immediate.

    Encoding: 0000 1100 size ea_mode ea_reg
    """
    sz = _size_field(size)
    opcode = (0x0C00 |
              (sz << 6) |
              (_mask(ea_mode, 3) << 3) |
              _mask(ea_reg, 3))
    return [_w(opcode)] + _imm_for_size(size, imm)


# ---------------------------------------------------------------------------
# Logical instructions
# ---------------------------------------------------------------------------

def and_op(size, dn, direction, ea_mode, ea_reg):
    """AND - AND Logical.

    Encoding: 1100 Dn opmode ea_mode ea_reg
      direction=0 (EA to Dn): 000=byte, 001=word, 010=long
      direction=1 (Dn to EA): 100=byte, 101=word, 110=long
    """
    opmode = _size_field(size) | (direction << 2)
    opcode = (0xC000 |
              (_mask(dn, 3) << 9) |
              (_mask(opmode, 3) << 6) |
              (_mask(ea_mode, 3) << 3) |
              _mask(ea_reg, 3))
    return [_w(opcode)]


def andi(size, ea_mode, ea_reg, imm):
    """ANDI #<data>,<ea> - AND Immediate.

    Encoding: 0000 0010 size ea_mode ea_reg
    """
    sz = _size_field(size)
    opcode = (0x0200 |
              (sz << 6) |
              (_mask(ea_mode, 3) << 3) |
              _mask(ea_reg, 3))
    return [_w(opcode)] + _imm_for_size(size, imm)


def or_op(size, dn, direction, ea_mode, ea_reg):
    """OR - OR Logical.

    Encoding: 1000 Dn opmode ea_mode ea_reg
    """
    opmode = _size_field(size) | (direction << 2)
    opcode = (0x8000 |
              (_mask(dn, 3) << 9) |
              (_mask(opmode, 3) << 6) |
              (_mask(ea_mode, 3) << 3) |
              _mask(ea_reg, 3))
    return [_w(opcode)]


def ori(size, ea_mode, ea_reg, imm):
    """ORI #<data>,<ea> - OR Immediate.

    Encoding: 0000 0000 size ea_mode ea_reg
    """
    sz = _size_field(size)
    opcode = (0x0000 |
              (sz << 6) |
              (_mask(ea_mode, 3) << 3) |
              _mask(ea_reg, 3))
    return [_w(opcode)] + _imm_for_size(size, imm)


def eor(size, dn, ea_mode, ea_reg):
    """EOR Dn,<ea> - Exclusive OR.

    Encoding: 1011 Dn opmode ea_mode ea_reg
      opmode: 100=byte, 101=word, 110=long (always register to EA)
    """
    opmode = _size_field(size) | 0b100
    opcode = (0xB000 |
              (_mask(dn, 3) << 9) |
              (_mask(opmode, 3) << 6) |
              (_mask(ea_mode, 3) << 3) |
              _mask(ea_reg, 3))
    return [_w(opcode)]


def eori(size, ea_mode, ea_reg, imm):
    """EORI #<data>,<ea> - Exclusive OR Immediate.

    Encoding: 0000 1010 size ea_mode ea_reg
    """
    sz = _size_field(size)
    opcode = (0x0A00 |
              (sz << 6) |
              (_mask(ea_mode, 3) << 3) |
              _mask(ea_reg, 3))
    return [_w(opcode)] + _imm_for_size(size, imm)


def not_op(size, ea_mode, ea_reg):
    """NOT <ea> - Logical Complement.

    Encoding: 0100 0110 size ea_mode ea_reg
    """
    sz = _size_field(size)
    opcode = (0x4600 |
              (sz << 6) |
              (_mask(ea_mode, 3) << 3) |
              _mask(ea_reg, 3))
    return [_w(opcode)]


def tst(size, ea_mode, ea_reg):
    """TST <ea> - Test an Operand.

    Encoding: 0100 1010 size ea_mode ea_reg
    """
    sz = _size_field(size)
    opcode = (0x4A00 |
              (sz << 6) |
              (_mask(ea_mode, 3) << 3) |
              _mask(ea_reg, 3))
    return [_w(opcode)]


# ---------------------------------------------------------------------------
# Shift/Rotate instructions (register form)
# ---------------------------------------------------------------------------

def _shift_reg(size, count_or_reg, dn, direction, shift_type, ir=0):
    """Common encoder for register-form shift/rotate instructions.

    Encoding: 1110 count/reg direction size ir shift_type Dn
      Bits 15-12: 1110
      Bits 11-9:  count/register
      Bit 8:      direction (0=right, 1=left)
      Bits 7-6:   size (00=byte, 01=word, 10=long)
      Bit 5:      i/r (0=count is immediate, 1=count is in register)
      Bits 4-3:   shift type (00=ASx, 01=LSx, 10=ROXx, 11=ROx)
      Bits 2-0:   Dn (register to shift)
    """
    sz = _size_field(size)
    opcode = (0xE000 |
              (_mask(count_or_reg, 3) << 9) |
              (direction << 8) |
              (sz << 6) |
              ((_mask(ir, 1)) << 5) |
              (_mask(shift_type, 2) << 3) |
              _mask(dn, 3))
    return [_w(opcode)]


def asl(size, count_or_reg, dn, ir=0):
    """ASL (Arithmetic Shift Left) - register form."""
    return _shift_reg(size, count_or_reg, dn, 1, 0b00, ir)


def asr(size, count_or_reg, dn, ir=0):
    """ASR (Arithmetic Shift Right) - register form."""
    return _shift_reg(size, count_or_reg, dn, 0, 0b00, ir)


def lsl(size, count_or_reg, dn, ir=0):
    """LSL (Logical Shift Left) - register form."""
    return _shift_reg(size, count_or_reg, dn, 1, 0b01, ir)


def lsr(size, count_or_reg, dn, ir=0):
    """LSR (Logical Shift Right) - register form."""
    return _shift_reg(size, count_or_reg, dn, 0, 0b01, ir)


def rol(size, count_or_reg, dn, ir=0):
    """ROL (Rotate Left) - register form."""
    return _shift_reg(size, count_or_reg, dn, 1, 0b11, ir)


def ror(size, count_or_reg, dn, ir=0):
    """ROR (Rotate Right) - register form."""
    return _shift_reg(size, count_or_reg, dn, 0, 0b11, ir)


def roxl(size, count_or_reg, dn, ir=0):
    """ROXL (Rotate Left with Extend) - register form."""
    return _shift_reg(size, count_or_reg, dn, 1, 0b10, ir)


def roxr(size, count_or_reg, dn, ir=0):
    """ROXR (Rotate Right with Extend) - register form."""
    return _shift_reg(size, count_or_reg, dn, 0, 0b10, ir)


# ---------------------------------------------------------------------------
# Shift/Rotate instructions (memory form)
# ---------------------------------------------------------------------------

def _shift_mem(ea_mode, ea_reg, direction, shift_type):
    """Common encoder for memory-form shift/rotate (word-size only, shift by 1).

    Encoding: 1110 0 shift_type direction 11 ea_mode ea_reg
    """
    opcode = (0xE0C0 |
              (_mask(shift_type, 2) << 9) |
              (direction << 8) |
              (_mask(ea_mode, 3) << 3) |
              _mask(ea_reg, 3))
    return [_w(opcode)]


def asl_mem(ea_mode, ea_reg):
    """ASL <ea> - Arithmetic Shift Left (memory, 1 bit)."""
    return _shift_mem(ea_mode, ea_reg, 1, 0b00)


def asr_mem(ea_mode, ea_reg):
    """ASR <ea> - Arithmetic Shift Right (memory, 1 bit)."""
    return _shift_mem(ea_mode, ea_reg, 0, 0b00)


def lsl_mem(ea_mode, ea_reg):
    """LSL <ea> - Logical Shift Left (memory, 1 bit)."""
    return _shift_mem(ea_mode, ea_reg, 1, 0b01)


def lsr_mem(ea_mode, ea_reg):
    """LSR <ea> - Logical Shift Right (memory, 1 bit)."""
    return _shift_mem(ea_mode, ea_reg, 0, 0b01)


def rol_mem(ea_mode, ea_reg):
    """ROL <ea> - Rotate Left (memory, 1 bit)."""
    return _shift_mem(ea_mode, ea_reg, 1, 0b11)


def ror_mem(ea_mode, ea_reg):
    """ROR <ea> - Rotate Right (memory, 1 bit)."""
    return _shift_mem(ea_mode, ea_reg, 0, 0b11)


def roxl_mem(ea_mode, ea_reg):
    """ROXL <ea> - Rotate Left with Extend (memory, 1 bit)."""
    return _shift_mem(ea_mode, ea_reg, 1, 0b10)


def roxr_mem(ea_mode, ea_reg):
    """ROXR <ea> - Rotate Right with Extend (memory, 1 bit)."""
    return _shift_mem(ea_mode, ea_reg, 0, 0b10)


# ---------------------------------------------------------------------------
# Branch / Jump instructions
# ---------------------------------------------------------------------------

def bra(disp):
    """BRA <disp> - Branch Always.

    Encoding: 0110 0000 displacement
    If disp fits in 8 bits (and != 0 and != -1/0xFF):
      opcode word contains 8-bit displacement
    If disp == 0x00 (or does not fit):
      opcode word has 0x00 in low byte, followed by 16-bit displacement word
    If disp == 0xFF:
      opcode word has 0xFF in low byte, followed by 32-bit displacement
    """
    return _branch_encode(0x6000, disp)


def bsr(disp):
    """BSR <disp> - Branch to Subroutine.

    Encoding: 0110 0001 displacement
    Same displacement encoding as BRA.
    """
    return _branch_encode(0x6100, disp)


def bcc(condition, disp):
    """Bcc <disp> - Branch Conditionally.

    Encoding: 0110 condition displacement
    """
    return _branch_encode(0x6000 | (condition << 8), disp)


def _branch_encode(base, disp):
    """Common branch displacement encoder.

    Args:
        base: Base opcode word (upper byte) with condition already set.
        disp: Signed displacement value.

    The displacement is relative to PC+2 (the address of the next instruction
    word after the opcode word).
    """
    d = disp & 0xFFFFFFFF  # unsigned representation
    # Check for 8-bit fit: -128..+127, excluding 0x00 and 0xFF
    if disp != 0 and -128 <= disp <= 127 and (disp & 0xFF) != 0xFF:
        return [_w(base | (disp & 0xFF))]
    # Check for 16-bit fit: -32768..+32767
    elif -32768 <= disp <= 32767:
        return [_w(base | 0x00), _w(disp)]
    else:
        # 32-bit displacement (MC68020+)
        return [_w(base | 0xFF)] + imm_long(disp)


def dbcc(condition, dn, disp):
    """DBcc Dn,<label> - Test Condition, Decrement, and Branch.

    Encoding: 0101 condition 11001 Dn
    Followed by 16-bit signed displacement word.
    """
    opcode = (0x50C8 |
              (_mask(condition, 4) << 8) |
              _mask(dn, 3))
    return [_w(opcode), _w(disp)]


def scc(condition, ea_mode, ea_reg):
    """Scc <ea> - Set According to Condition.

    Encoding: 0101 condition 11 ea_mode ea_reg
    """
    opcode = (0x50C0 |
              (_mask(condition, 4) << 8) |
              (_mask(ea_mode, 3) << 3) |
              _mask(ea_reg, 3))
    return [_w(opcode)]


def jmp(ea_mode, ea_reg):
    """JMP <ea> - Jump.

    Encoding: 0100 1110 11 ea_mode ea_reg
    """
    opcode = 0x4EC0 | (_mask(ea_mode, 3) << 3) | _mask(ea_reg, 3)
    return [_w(opcode)]


def jmp_abs(addr):
    """JMP (abs.L) - Jump to absolute long address.

    Convenience function: JMP with absolute long addressing mode.
    """
    return jmp(SPECIAL, ABS_L) + abs_long(addr)


def jsr(ea_mode, ea_reg):
    """JSR <ea> - Jump to Subroutine.

    Encoding: 0100 1110 10 ea_mode ea_reg
    """
    opcode = 0x4E80 | (_mask(ea_mode, 3) << 3) | _mask(ea_reg, 3)
    return [_w(opcode)]


def jsr_abs(addr):
    """JSR (abs.L) - Jump to subroutine at absolute long address."""
    return jsr(SPECIAL, ABS_L) + abs_long(addr)


def rts():
    """RTS - Return from Subroutine.

    Encoding: 0100 1110 0111 0101 = 0x4E75
    """
    return [0x4E75]


def rte():
    """RTE - Return from Exception.

    Encoding: 0100 1110 0111 0011 = 0x4E73
    """
    return [0x4E73]


def rtd(disp):
    """RTD #<displacement> - Return and Deallocate.

    Encoding: 0100 1110 0111 0100 = 0x4E74, then a 16-bit displacement.
    Operation: (SP) -> PC; SP + 4 + dn -> SP.
    """
    return [0x4E74, _w(disp)]


def rtr():
    """RTR - Return and Restore Condition Codes.

    Encoding: 0100 1110 0111 0111 = 0x4E77
    """
    return [0x4E77]


# ---------------------------------------------------------------------------
# Bit operation instructions
# ---------------------------------------------------------------------------

def btst_reg(dn, ea_mode, ea_reg):
    """BTST Dn,<ea> - Test a Bit (dynamic, register-specified bit number).

    Encoding: 0000 Dn 100 ea_mode ea_reg
    """
    opcode = (0x0100 |
              (_mask(dn, 3) << 9) |
              (_mask(ea_mode, 3) << 3) |
              _mask(ea_reg, 3))
    return [_w(opcode)]


def btst_imm(ea_mode, ea_reg, bitnum):
    """BTST #<bitnum>,<ea> - Test a Bit (static, immediate bit number).

    Encoding: 0000 1000 00 ea_mode ea_reg
    Followed by extension word with bit number in bits 7:0.
    """
    opcode = 0x0800 | (_mask(ea_mode, 3) << 3) | _mask(ea_reg, 3)
    return [_w(opcode), _w(bitnum & 0xFF)]


def bset_reg(dn, ea_mode, ea_reg):
    """BSET Dn,<ea> - Test a Bit and Set.

    Encoding: 0000 Dn 111 ea_mode ea_reg
    """
    opcode = (0x01C0 |
              (_mask(dn, 3) << 9) |
              (_mask(ea_mode, 3) << 3) |
              _mask(ea_reg, 3))
    return [_w(opcode)]


def bset_imm(ea_mode, ea_reg, bitnum):
    """BSET #<bitnum>,<ea> - Test a Bit and Set.

    Encoding: 0000 1000 11 ea_mode ea_reg
    """
    opcode = 0x08C0 | (_mask(ea_mode, 3) << 3) | _mask(ea_reg, 3)
    return [_w(opcode), _w(bitnum & 0xFF)]


def bchg_reg(dn, ea_mode, ea_reg):
    """BCHG Dn,<ea> - Test a Bit and Change.

    Encoding: 0000 Dn 101 ea_mode ea_reg
    """
    opcode = (0x0140 |
              (_mask(dn, 3) << 9) |
              (_mask(ea_mode, 3) << 3) |
              _mask(ea_reg, 3))
    return [_w(opcode)]


def bchg_imm(ea_mode, ea_reg, bitnum):
    """BCHG #<bitnum>,<ea> - Test a Bit and Change.

    Encoding: 0000 1000 01 ea_mode ea_reg
    """
    opcode = 0x0840 | (_mask(ea_mode, 3) << 3) | _mask(ea_reg, 3)
    return [_w(opcode), _w(bitnum & 0xFF)]


def bclr_reg(dn, ea_mode, ea_reg):
    """BCLR Dn,<ea> - Test a Bit and Clear.

    Encoding: 0000 Dn 110 ea_mode ea_reg
    """
    opcode = (0x0180 |
              (_mask(dn, 3) << 9) |
              (_mask(ea_mode, 3) << 3) |
              _mask(ea_reg, 3))
    return [_w(opcode)]


def bclr_imm(ea_mode, ea_reg, bitnum):
    """BCLR #<bitnum>,<ea> - Test a Bit and Clear.

    Encoding: 0000 1000 10 ea_mode ea_reg
    """
    opcode = 0x0880 | (_mask(ea_mode, 3) << 3) | _mask(ea_reg, 3)
    return [_w(opcode), _w(bitnum & 0xFF)]


# ---------------------------------------------------------------------------
# Bit field instructions (MC68020+)
#
# PRM Section 8 instruction format summary:
#   opcode word: 1110 1ooo 11 <ea>    (ooo selects the operation)
#   extension:   bit 15    = 0
#                bits 14-12 = Dn register (BFEXTU/BFEXTS/BFFFO/BFINS only,
#                             zero for the read-modify-write forms)
#                bit 11    = Do  (0 = offset immediate, 1 = offset in Dn)
#                bits 10-6 = OFFSET (immediate value, or Dn number in 8-6)
#                bit 5     = Dw  (0 = width immediate, 1 = width in Dn)
#                bits 4-0  = WIDTH (immediate value, or Dn number in 2-0)
#
# PRM (BFCHG Dw field): "If Dw = 0 ... an operand value in the range 1-31
# specifies a field width of 1-31, and a value of zero specifies a width of 32."
# ---------------------------------------------------------------------------

BFTST_OP  = 0b000
BFEXTU_OP = 0b001
BFCHG_OP  = 0b010
BFEXTS_OP = 0b011
BFCLR_OP  = 0b100
BFFFO_OP  = 0b101
BFSET_OP  = 0b110
BFINS_OP  = 0b111


def _bf_ext(dn, offset, width, do, dw):
    """Build the bit-field extension word.

    Args:
        dn: Dn register field (bits 14-12); 0 for BFTST/BFCHG/BFCLR/BFSET.
        offset: Immediate offset 0-31 (do=0) or Dn number 0-7 (do=1).
        width: Immediate width 0-31 where 0 means 32 (dw=0), or Dn number (dw=1).
        do: 0 = offset is immediate, 1 = offset comes from a data register.
        dw: 0 = width is immediate, 1 = width comes from a data register.
    """
    return _w((_mask(dn, 3) << 12) |
              (_mask(do, 1) << 11) |
              (_mask(offset, 5) << 6) |
              (_mask(dw, 1) << 5) |
              _mask(width, 5))


def _bf_op(op3, ea_mode, ea_reg, dn, offset, width, do, dw):
    opcode = (0xE8C0 |
              (_mask(op3, 3) << 8) |
              (_mask(ea_mode, 3) << 3) |
              _mask(ea_reg, 3))
    return [_w(opcode), _bf_ext(dn, offset, width, do, dw)]


def bftst(ea_mode, ea_reg, offset, width, do=0, dw=0):
    """BFTST <ea>{offset:width} - Test Bit Field. Opcode base 0xE8C0."""
    return _bf_op(BFTST_OP, ea_mode, ea_reg, 0, offset, width, do, dw)


def bfchg(ea_mode, ea_reg, offset, width, do=0, dw=0):
    """BFCHG <ea>{offset:width} - Test Bit Field and Change. Base 0xEAC0."""
    return _bf_op(BFCHG_OP, ea_mode, ea_reg, 0, offset, width, do, dw)


def bfclr(ea_mode, ea_reg, offset, width, do=0, dw=0):
    """BFCLR <ea>{offset:width} - Test Bit Field and Clear. Base 0xECC0."""
    return _bf_op(BFCLR_OP, ea_mode, ea_reg, 0, offset, width, do, dw)


def bfset(ea_mode, ea_reg, offset, width, do=0, dw=0):
    """BFSET <ea>{offset:width} - Test Bit Field and Set. Base 0xEEC0."""
    return _bf_op(BFSET_OP, ea_mode, ea_reg, 0, offset, width, do, dw)


def bfextu(ea_mode, ea_reg, offset, width, dn, do=0, dw=0):
    """BFEXTU <ea>{offset:width},Dn - Extract Bit Field Unsigned. Base 0xE9C0."""
    return _bf_op(BFEXTU_OP, ea_mode, ea_reg, dn, offset, width, do, dw)


def bfexts(ea_mode, ea_reg, offset, width, dn, do=0, dw=0):
    """BFEXTS <ea>{offset:width},Dn - Extract Bit Field Signed. Base 0xEBC0."""
    return _bf_op(BFEXTS_OP, ea_mode, ea_reg, dn, offset, width, do, dw)


def bfffo(ea_mode, ea_reg, offset, width, dn, do=0, dw=0):
    """BFFFO <ea>{offset:width},Dn - Find First One in Bit Field. Base 0xEDC0."""
    return _bf_op(BFFFO_OP, ea_mode, ea_reg, dn, offset, width, do, dw)


def bfins(dn, ea_mode, ea_reg, offset, width, do=0, dw=0):
    """BFINS Dn,<ea>{offset:width} - Insert Bit Field. Base 0xEFC0."""
    return _bf_op(BFINS_OP, ea_mode, ea_reg, dn, offset, width, do, dw)


# ---------------------------------------------------------------------------
# BCD instructions
# ---------------------------------------------------------------------------

def abcd(rx, ry, rm=0):
    """ABCD - Add Decimal with Extend.

    Encoding: 1100 Rx 10000 R/M Ry  (PRM: base 0xC100)
      rm=0: ABCD Dy,Dx   (Rx = destination Dx, Ry = source Dy)
      rm=1: ABCD -(Ay),-(Ax)
    """
    return [_w(0xC100 | (_mask(rx, 3) << 9) | (_mask(rm, 1) << 3) | _mask(ry, 3))]


def sbcd(ry, rx, rm=0):
    """SBCD - Subtract Decimal with Extend.

    Encoding: 1000 Dy/Ay 10000 R/M Dx/Ax  (PRM: base 0x8100)
    PRM operation: Destination10 - Source10 - X -> Destination, where the
    bits 11-9 register (ry here) is the DESTINATION and bits 2-0 (rx) the source.
      rm=0: SBCD Dx,Dy   (Dy = destination)
      rm=1: SBCD -(Ax),-(Ay)
    """
    return [_w(0x8100 | (_mask(ry, 3) << 9) | (_mask(rm, 1) << 3) | _mask(rx, 3))]


def nbcd(ea_mode, ea_reg):
    """NBCD <ea> - Negate Decimal with Extend.

    Encoding: 0100 1000 00 ea_mode ea_reg  (PRM: base 0x4800)
    """
    return [_w(0x4800 | (_mask(ea_mode, 3) << 3) | _mask(ea_reg, 3))]


def pack(ry, rx, adjustment, rm=0):
    """PACK - Pack BCD.

    Encoding: 1000 Dy/Ay 10100 R/M Dx/Ax + 16-bit adjustment (base 0x8140)
      rm=0: PACK Dx,Dy,#adj
      rm=1: PACK -(Ax),-(Ay),#adj
    """
    return [_w(0x8140 | (_mask(ry, 3) << 9) | (_mask(rm, 1) << 3) | _mask(rx, 3)),
            _w(adjustment)]


def unpk(ry, rx, adjustment, rm=0):
    """UNPK - Unpack BCD.

    Encoding: 1000 Dy/Ay 11000 R/M Dx/Ax + 16-bit adjustment (base 0x8180)
      rm=0: UNPK Dx,Dy,#adj
      rm=1: UNPK -(Ax),-(Ay),#adj
    """
    return [_w(0x8180 | (_mask(ry, 3) << 9) | (_mask(rm, 1) << 3) | _mask(rx, 3)),
            _w(adjustment)]


# ---------------------------------------------------------------------------
# Extended-precision add/subtract and memory compare
# ---------------------------------------------------------------------------

def addx(size, rx, ry, rm=0):
    """ADDX - Add Extended.

    Encoding: 1101 Rx 1 SIZE 00 R/M Ry  (PRM: base 0xD100)
      rm=0: ADDX Dy,Dx  (Rx = destination)
      rm=1: ADDX -(Ay),-(Ax)
    """
    return [_w(0xD100 |
               (_mask(rx, 3) << 9) |
               (_size_field(size) << 6) |
               (_mask(rm, 1) << 3) |
               _mask(ry, 3))]


def subx(size, ry, rx, rm=0):
    """SUBX - Subtract with Extend.

    Encoding: 1001 Dy/Ay 1 SIZE 00 R/M Dx/Ax  (PRM: base 0x9100)
    PRM operation: Destination - Source - X -> Destination, where bits 11-9
    (ry) is the destination and bits 2-0 (rx) the source.
      rm=0: SUBX Dx,Dy   (Dy = destination)
      rm=1: SUBX -(Ax),-(Ay)
    """
    return [_w(0x9100 |
               (_mask(ry, 3) << 9) |
               (_size_field(size) << 6) |
               (_mask(rm, 1) << 3) |
               _mask(rx, 3))]


def cmpm(size, ax, ay):
    """CMPM (Ay)+,(Ax)+ - Compare Memory.

    Encoding: 1011 Ax 1 SIZE 001 Ay  (PRM: base 0xB108)
    Ax is always the destination, Ay always the source.
    """
    return [_w(0xB108 |
               (_mask(ax, 3) << 9) |
               (_size_field(size) << 6) |
               _mask(ay, 3))]


# ---------------------------------------------------------------------------
# Compare and swap (MC68020+)
# ---------------------------------------------------------------------------

def _cas_size(size):
    """CAS/CAS2 use a distinct size field: 01=byte, 10=word, 11=long."""
    if size == BYTE:
        return 0b01
    if size == WORD:
        return 0b10
    if size == LONG:
        return 0b11
    raise ValueError(f"Invalid CAS size: {size}")


def cas(size, dc, du, ea_mode, ea_reg):
    """CAS Dc,Du,<ea> - Compare and Swap with Operand.

    Encoding: 0000 1 SIZE 011 ea_mode ea_reg  (base 0x08C0, SIZE in bits 10-9)
    Extension: 0000000 Du 000 Dc
    """
    opcode = (0x08C0 |
              (_cas_size(size) << 9) |
              (_mask(ea_mode, 3) << 3) |
              _mask(ea_reg, 3))
    return [_w(opcode), _w((_mask(du, 3) << 6) | _mask(dc, 3))]


def cas2(size, dc1, du1, rn1, da1, dc2, du2, rn2, da2):
    """CAS2 Dc1:Dc2,Du1:Du2,(Rn1):(Rn2) - Compare and Swap with Two Operands.

    Encoding: 0000 1 SIZE 0 1111 1100  (base 0x08FC, SIZE in bits 10-9)
    Two extension words, each: D/A Rn 000 Du 000 Dc
    da1/da2: 0 = Rn is a data register, 1 = Rn is an address register.
    CAS2 cannot use byte operands (PRM footnote).
    """
    if size == BYTE:
        raise ValueError("CAS2 cannot use byte operands")
    opcode = 0x08FC | (_cas_size(size) << 9)
    ext1 = ((_mask(da1, 1) << 15) | (_mask(rn1, 3) << 12) |
            (_mask(du1, 3) << 6) | _mask(dc1, 3))
    ext2 = ((_mask(da2, 1) << 15) | (_mask(rn2, 3) << 12) |
            (_mask(du2, 3) << 6) | _mask(dc2, 3))
    return [_w(opcode), _w(ext1), _w(ext2)]


# ---------------------------------------------------------------------------
# MOVEP / MOVES / MOVEC
# ---------------------------------------------------------------------------

MOVEP_W_TO_REG = 0b100
MOVEP_L_TO_REG = 0b101
MOVEP_W_TO_MEM = 0b110
MOVEP_L_TO_MEM = 0b111


def movep(opmode, dx, ay, disp):
    """MOVEP - Move Peripheral Data.

    Encoding: 0000 Dx OPMODE 001 Ay + 16-bit displacement  (base 0x0008)
    opmode: 100 = word mem->reg, 101 = long mem->reg,
            110 = word reg->mem, 111 = long reg->mem
    """
    return [_w(0x0008 |
               (_mask(dx, 3) << 9) |
               (_mask(opmode, 3) << 6) |
               _mask(ay, 3)),
            _w(disp)]


def movep_to_mem(size, dx, ay, disp):
    """MOVEP Dx,(d16,Ay) - register to alternate memory bytes."""
    opmode = MOVEP_L_TO_MEM if size == LONG else MOVEP_W_TO_MEM
    return movep(opmode, dx, ay, disp)


def movep_to_reg(size, dx, ay, disp):
    """MOVEP (d16,Ay),Dx - alternate memory bytes to register."""
    opmode = MOVEP_L_TO_REG if size == LONG else MOVEP_W_TO_REG
    return movep(opmode, dx, ay, disp)


def moves(size, rn, ad, dr, ea_mode, ea_reg):
    """MOVES - Move Address Space (privileged, MC68010+).

    Encoding: 0000 1110 SIZE ea_mode ea_reg  (base 0x0E00)
    Extension: A/D REGISTER dr 00000000000
      ad: 0 = data register, 1 = address register
      dr: 0 = <ea> to general register, 1 = general register to <ea>
    """
    opcode = (0x0E00 |
              (_size_field(size) << 6) |
              (_mask(ea_mode, 3) << 3) |
              _mask(ea_reg, 3))
    ext = (_mask(ad, 1) << 15) | (_mask(rn, 3) << 12) | (_mask(dr, 1) << 11)
    return [_w(opcode), _w(ext)]


# MOVEC control-register selector codes (MC68030 UM Table 10-5)
CR_SFC  = 0x000
CR_DFC  = 0x001
CR_CACR = 0x002
CR_USP  = 0x800
CR_VBR  = 0x801
CR_CAAR = 0x802
CR_MSP  = 0x803
CR_ISP  = 0x804


def movec_to_cr(rn, cr_sel, ad=0):
    """MOVEC Rn,Cr - move general register to control register (privileged).

    Encoding: 0x4E7B + extension (A/D | register | 0 | control code).
    """
    ext = (_mask(ad, 1) << 15) | (_mask(rn, 3) << 12) | (cr_sel & 0x0FFF)
    return [0x4E7B, _w(ext)]


def movec_from_cr(cr_sel, rn, ad=0):
    """MOVEC Cr,Rn - move control register to general register (privileged)."""
    ext = (_mask(ad, 1) << 15) | (_mask(rn, 3) << 12) | (cr_sel & 0x0FFF)
    return [0x4E7A, _w(ext)]


# ---------------------------------------------------------------------------
# Multiply / Divide (word forms)
# ---------------------------------------------------------------------------

def muls_w(ea_mode, ea_reg, dn):
    """MULS.W <ea>,Dn - Signed Multiply (16x16->32).

    Encoding: 1100 Dn 111 ea_mode ea_reg
    """
    opcode = (0xC1C0 |
              (_mask(dn, 3) << 9) |
              (_mask(ea_mode, 3) << 3) |
              _mask(ea_reg, 3))
    return [_w(opcode)]


def mulu_w(ea_mode, ea_reg, dn):
    """MULU.W <ea>,Dn - Unsigned Multiply (16x16->32).

    Encoding: 1100 Dn 011 ea_mode ea_reg
    """
    opcode = (0xC0C0 |
              (_mask(dn, 3) << 9) |
              (_mask(ea_mode, 3) << 3) |
              _mask(ea_reg, 3))
    return [_w(opcode)]


def divs_w(ea_mode, ea_reg, dn):
    """DIVS.W <ea>,Dn - Signed Divide (32/16->16r:16q).

    Encoding: 1000 Dn 111 ea_mode ea_reg
    """
    opcode = (0x81C0 |
              (_mask(dn, 3) << 9) |
              (_mask(ea_mode, 3) << 3) |
              _mask(ea_reg, 3))
    return [_w(opcode)]


def divu_w(ea_mode, ea_reg, dn):
    """DIVU.W <ea>,Dn - Unsigned Divide (32/16->16r:16q).

    Encoding: 1000 Dn 011 ea_mode ea_reg
    """
    opcode = (0x80C0 |
              (_mask(dn, 3) << 9) |
              (_mask(ea_mode, 3) << 3) |
              _mask(ea_reg, 3))
    return [_w(opcode)]


# ---------------------------------------------------------------------------
# Multiply / Divide (long forms, MC68020+)
#
# PRM: MUL opcode word 0100 1100 00 <ea> (0x4C00), DIV 0100 1100 01 <ea>
# (0x4C40). Both take one extension word:
#     bit 15    = 0
#     bits 14-12 = Dl (MUL) / Dq (DIV) -- low product / quotient register
#     bit 11    = 1 for the signed form (MULS/DIVS), 0 for unsigned
#     bit 10    = SIZE: 0 = 32-bit result, 1 = 64-bit product / 64-bit dividend
#     bits 9-3  = 0
#     bits 2-0  = Dh (MUL) / Dr (DIV) -- high product / remainder register
# ---------------------------------------------------------------------------

def _muldiv_l(base, ea_mode, ea_reg, dl, dh, signed, size64):
    opcode = base | (_mask(ea_mode, 3) << 3) | _mask(ea_reg, 3)
    ext = ((_mask(dl, 3) << 12) |
           (_mask(signed, 1) << 11) |
           (_mask(size64, 1) << 10) |
           _mask(dh, 3))
    return [_w(opcode), _w(ext)]


def mulu_l(ea_mode, ea_reg, dl, dh=None):
    """MULU.L <ea>,Dl  (32x32->32) or MULU.L <ea>,Dh:Dl  (32x32->64).

    Pass dh to select the 64-bit form; the high 32 bits land in Dh.
    """
    if dh is None:
        return _muldiv_l(0x4C00, ea_mode, ea_reg, dl, 0, 0, 0)
    return _muldiv_l(0x4C00, ea_mode, ea_reg, dl, dh, 0, 1)


def muls_l(ea_mode, ea_reg, dl, dh=None):
    """MULS.L <ea>,Dl  (32x32->32) or MULS.L <ea>,Dh:Dl  (32x32->64)."""
    if dh is None:
        return _muldiv_l(0x4C00, ea_mode, ea_reg, dl, 0, 1, 0)
    return _muldiv_l(0x4C00, ea_mode, ea_reg, dl, dh, 1, 1)


def divu_l(ea_mode, ea_reg, dq, dr=None, size64=False):
    """DIVU.L / DIVUL.L <ea>,... - unsigned long divide.

    dr is None                -> DIVU.L  <ea>,Dq       32/32 -> 32q
    dr given, size64 is False  -> DIVUL.L <ea>,Dr:Dq    32/32 -> 32r:32q
    dr given, size64 is True   -> DIVU.L  <ea>,Dr:Dq    64/32 -> 32r:32q
    """
    if dr is None:
        # PRM: "If Dr and Dq are the same register, only the quotient is returned."
        return _muldiv_l(0x4C40, ea_mode, ea_reg, dq, dq, 0, 0)
    return _muldiv_l(0x4C40, ea_mode, ea_reg, dq, dr, 0, 1 if size64 else 0)


def divs_l(ea_mode, ea_reg, dq, dr=None, size64=False):
    """DIVS.L / DIVSL.L <ea>,... - signed long divide (see divu_l)."""
    if dr is None:
        return _muldiv_l(0x4C40, ea_mode, ea_reg, dq, dq, 1, 0)
    return _muldiv_l(0x4C40, ea_mode, ea_reg, dq, dr, 1, 1 if size64 else 0)


# ---------------------------------------------------------------------------
# Miscellaneous instructions
# ---------------------------------------------------------------------------

def trap(vector):
    """TRAP #<vector> - Trap.

    Encoding: 0100 1110 0100 vector
    vector: 0-15
    """
    return [_w(0x4E40 | _mask(vector, 4))]


def trapcc(condition, operand=None, size=None):
    """TRAPcc [#<operand>] - Trap on Condition.

    Encoding: 0101 condition 11111 opmode
    opmode 100 = no operand, 010 = word operand, 011 = long operand.
    Pass size="W" or "L" with an operand; omit both for the no-operand form.
    """
    if size is None:
        assert operand is None, "an operand requires size='W' or 'L'"
        return [_w(0x50FC | (_mask(condition, 4) << 8))]
    if size == "W":
        return [_w(0x50FA | (_mask(condition, 4) << 8)), _w(operand)]
    if size == "L":
        return ([_w(0x50FB | (_mask(condition, 4) << 8))] +
                [(operand >> 16) & 0xFFFF, operand & 0xFFFF])
    raise ValueError(f"bad TRAPcc size {size!r}")


def trapv():
    """TRAPV - Trap on Overflow.

    Encoding: 0100 1110 0111 0110 = 0x4E76
    """
    return [0x4E76]


def illegal():
    """ILLEGAL - Illegal Instruction.

    Encoding: 0100 1010 1111 1100 = 0x4AFC
    """
    return [0x4AFC]


def link_w(an, disp):
    """LINK An,#<disp> - Link and Allocate (word displacement).

    Encoding: 0100 1110 0101 0 An
    Followed by 16-bit signed displacement word.
    """
    opcode = 0x4E50 | _mask(an, 3)
    return [_w(opcode), _w(disp)]


def link_l(an, disp):
    """LINK An,#<disp> - Link and Allocate (long displacement, MC68020+).

    Encoding: 0100 1000 0000 1 An
    Followed by 32-bit signed displacement.
    """
    opcode = 0x4808 | _mask(an, 3)
    return [_w(opcode)] + imm_long(disp)


def unlk(an):
    """UNLK An - Unlink.

    Encoding: 0100 1110 0101 1 An
    """
    return [_w(0x4E58 | _mask(an, 3))]


def tas(ea_mode, ea_reg):
    """TAS <ea> - Test and Set an Operand.

    Encoding: 0100 1010 11 ea_mode ea_reg
    """
    opcode = 0x4AC0 | (_mask(ea_mode, 3) << 3) | _mask(ea_reg, 3)
    return [_w(opcode)]


def chk_w(ea_mode, ea_reg, dn):
    """CHK.W <ea>,Dn - Check Register Against Bounds (word).

    Encoding: 0100 Dn 110 ea_mode ea_reg
    """
    opcode = (0x4180 |
              (_mask(dn, 3) << 9) |
              (_mask(ea_mode, 3) << 3) |
              _mask(ea_reg, 3))
    return [_w(opcode)]


def chk_l(ea_mode, ea_reg, dn):
    """CHK.L <ea>,Dn - Check Register Against Bounds (long, MC68020+).

    Encoding: 0100 Dn 100 ea_mode ea_reg
    """
    opcode = (0x4100 |
              (_mask(dn, 3) << 9) |
              (_mask(ea_mode, 3) << 3) |
              _mask(ea_reg, 3))
    return [_w(opcode)]


def stop(imm16):
    """STOP #<data> - Load Status Register and Stop.

    Encoding: 0100 1110 0111 0010 = 0x4E72
    Followed by 16-bit immediate word.
    """
    return [0x4E72, _w(imm16)]


def movem_to_mem(size, ea_mode, ea_reg, regmask):
    """MOVEM Register to Memory.

    Encoding: 0100 1000 1 size ea_mode ea_reg
    Followed by 16-bit register mask word.
      size: 0=word (0100 1000 10), 1=long (0100 1000 11)
      Note: MOVEM uses size bit 6 only: 0=word, 1=long

    For predecrement mode -(An), the register mask is reversed
    (A7 is bit 0, D0 is bit 15). This reversal must be done by the caller.
    """
    sz = 1 if size == LONG else 0
    opcode = (0x4880 |
              (sz << 6) |
              (_mask(ea_mode, 3) << 3) |
              _mask(ea_reg, 3))
    return [_w(opcode), _w(regmask)]


def movem_from_mem(size, ea_mode, ea_reg, regmask):
    """MOVEM Memory to Register.

    Encoding: 0100 1100 1 size ea_mode ea_reg
    Followed by 16-bit register mask word.
    """
    sz = 1 if size == LONG else 0
    opcode = (0x4C80 |
              (sz << 6) |
              (_mask(ea_mode, 3) << 3) |
              _mask(ea_reg, 3))
    return [_w(opcode), _w(regmask)]


# ---------------------------------------------------------------------------
# MOVE.L <ea>,<ea> convenience for storing to absolute long
# ---------------------------------------------------------------------------

def move_to_abs_long(size, src_mode, src_reg, addr):
    """MOVE <ea>,(xxx).L - Move to absolute long address.

    Convenience: encodes MOVE with destination mode=SPECIAL, reg=ABS_L.
    """
    words = move(size, src_mode, src_reg, SPECIAL, ABS_L)
    words.extend(abs_long(addr))
    return words


def move_from_abs_long(size, addr, dst_mode, dst_reg):
    """MOVE (xxx).L,<ea> - Move from absolute long address.

    Convenience: encodes MOVE with source mode=SPECIAL, reg=ABS_L.
    """
    words = move(size, SPECIAL, ABS_L, dst_mode, dst_reg)
    words.extend(abs_long(addr))
    return words


# ---------------------------------------------------------------------------
# Self-test
# ---------------------------------------------------------------------------

def _self_test():
    """Verify a few known encodings for correctness."""
    errors = []

    def check(name, got, expected):
        if got != expected:
            errors.append(
                f"  {name}: expected {[f'0x{w:04X}' for w in expected]}, "
                f"got {[f'0x{w:04X}' for w in got]}"
            )

    # NOP = 0x4E71
    check("NOP", nop(), [0x4E71])

    # MOVEQ #42, D0 = 0x702A
    check("MOVEQ #42,D0", moveq(42, 0), [0x702A])

    # MOVEQ #-1, D3 = 0x76FF
    check("MOVEQ #-1,D3", moveq(-1, 3), [0x76FF])

    # RTS = 0x4E75
    check("RTS", rts(), [0x4E75])

    # RTE = 0x4E73
    check("RTE", rte(), [0x4E73])

    # ILLEGAL = 0x4AFC
    check("ILLEGAL", illegal(), [0x4AFC])

    # TRAPV = 0x4E76
    check("TRAPV", trapv(), [0x4E76])

    # SWAP D3 = 0x4843
    check("SWAP D3", swap(3), [0x4843])

    # CLR.L D0 = 0100 0010 10 000 000 = 0x4280
    check("CLR.L D0", clr(LONG, DN, 0), [0x4280])

    # MOVE.L D0,D1: size=10 (long), dst_reg=001, dst_mode=000, src_mode=000, src_reg=000
    #   = 10 001 000 000 000 = 0x2200
    check("MOVE.L D0,D1", move(LONG, DN, 0, DN, 1), [0x2200])

    # MOVE.W D0,D1: size=11 (word), dst_reg=001, dst_mode=000, src_mode=000, src_reg=000
    #   = 11 001 000 000 000 = 0x3200
    check("MOVE.W D0,D1", move(WORD, DN, 0, DN, 1), [0x3200])

    # MOVE.B D0,D1: size=01 (byte), dst_reg=001, dst_mode=000, src_mode=000, src_reg=000
    #   = 01 001 000 000 000 = 0x1200
    check("MOVE.B D0,D1", move(BYTE, DN, 0, DN, 1), [0x1200])

    # ADDI.L #$12345678,D0 = 0x0680, 0x1234, 0x5678
    check("ADDI.L #$12345678,D0",
          addi(LONG, DN, 0, 0x12345678),
          [0x0680, 0x1234, 0x5678])

    # ADDQ.L #1,D0 = 0101 001 0 10 000 000 = 0x5280
    check("ADDQ.L #1,D0", addq(LONG, 1, DN, 0), [0x5280])

    # SUBQ.L #1,D0 = 0101 001 1 10 000 000 = 0x5380
    check("SUBQ.L #1,D0", subq(LONG, 1, DN, 0), [0x5380])

    # CMPI.L #0,D0 = 0x0C80, 0x0000, 0x0000
    check("CMPI.L #0,D0",
          cmpi(LONG, DN, 0, 0),
          [0x0C80, 0x0000, 0x0000])

    # BRA with 8-bit displacement: BRA .+4 -> 0x6002 (disp = +2 relative to PC+2)
    check("BRA .+4", bra(2), [0x6002])

    # BEQ with 16-bit displacement: BEQ .+258 -> 0x6700, 0x0100
    check("BEQ .+258", bcc(CC_EQ, 256), [0x6700, 0x0100])

    # DBcc: DBRA D0,<disp> = 0x51C8, disp_word
    check("DBRA D0,-4", dbcc(CC_F, 0, 0xFFFC), [0x51C8, 0xFFFC])

    # EXT.W D0 = 0x4880
    check("EXT.W D0", ext_w(0), [0x4880])

    # EXT.L D0 = 0x48C0
    check("EXT.L D0", ext_l(0), [0x48C0])

    # EXTB.L D0 = 0x49C0
    check("EXTB.L D0", extb(0), [0x49C0])

    # TRAP #0 = 0x4E40
    check("TRAP #0", trap(0), [0x4E40])

    # TRAP #15 = 0x4E4F
    check("TRAP #15", trap(15), [0x4E4F])

    # STOP #$2700 = 0x4E72, 0x2700
    check("STOP #$2700", stop(0x2700), [0x4E72, 0x2700])

    # LEA (A0),A1 = 0100 001 111 010 000 = 0x43D0
    check("LEA (A0),A1", lea(AN_IND, 0, 1), [0x43D0])

    # TST.L D0 = 0100 1010 10 000 000 = 0x4A80
    check("TST.L D0", tst(LONG, DN, 0), [0x4A80])

    # LSL.W #3,D2 = 1110 011 1 01 0 01 010 = 0xE74A
    check("LSL.W #3,D2", lsl(WORD, 3, 2, 0), [0xE74A])

    # ASR.L #1,D0 = 1110 001 0 10 0 00 000 = 0xE280
    check("ASR.L #1,D0", asr(LONG, 1, 0, 0), [0xE280])

    # LINK A6,#-4 = 0x4E56, 0xFFFC
    check("LINK A6,#-4", link_w(6, -4), [0x4E56, 0xFFFC])

    # UNLK A6 = 0x4E5E
    check("UNLK A6", unlk(6), [0x4E5E])

    # BTST #7,D0 = 0x0800, 0x0007
    check("BTST #7,D0", btst_imm(DN, 0, 7), [0x0800, 0x0007])

    # MULU.W D1,D0 = 1100 000 011 000 001 = 0xC0C1
    check("MULU.W D1,D0", mulu_w(DN, 1, 0), [0xC0C1])

    # MULS.W D1,D0 = 1100 000 111 000 001 = 0xC1C1
    check("MULS.W D1,D0", muls_w(DN, 1, 0), [0xC1C1])

    # MOVEM.L D0-D7/A0-A6,-(A7) = 0x48E7, mask
    check("MOVEM.L to -(A7)",
          movem_to_mem(LONG, AN_PREDEC, 7, 0xFFFE),
          [0x48E7, 0xFFFE])

    # Scc: ST D0 = 0101 0000 11 000 000 = 0x50C0
    check("ST D0", scc(CC_T, DN, 0), [0x50C0])

    # JMP (abs.L) = 0x4EF9 + address
    check("JMP $00001000", jmp_abs(0x1000), [0x4EF9, 0x0000, 0x1000])

    # RTR = 0x4E77
    check("RTR", rtr(), [0x4E77])

    # NOT.L D0 = 0100 0110 10 000 000 = 0x4680
    check("NOT.L D0", not_op(LONG, DN, 0), [0x4680])

    # NEG.L D0 = 0100 0100 10 000 000 = 0x4480
    check("NEG.L D0", neg(LONG, DN, 0), [0x4480])

    # ADD.L D0,(A0) = 1101 000 110 010 000 = 0xD190
    check("ADD.L D0,(A0)", add(LONG, 0, 1, AN_IND, 0), [0xD190])

    # AND.L D3,D0 = 1100 000 010 000 011 = 0xC083
    check("AND.L D3,D0", and_op(LONG, 0, 0, DN, 3), [0xC083])

    # OR.L #$FF,D0 = 0x0080, 0x0000, 0x00FF
    check("ORI.L #$FF,D0", ori(LONG, DN, 0, 0xFF), [0x0080, 0x0000, 0x00FF])

    # EXG D0,D1 = 1100 000 1 01000 001 = 0xC141
    check("EXG D0,D1", exg(0, 1, EXG_DD), [0xC141])

    # -----------------------------------------------------------------------
    # Encodings below were cross-checked against `m68k-elf-as -m68030`
    # (GNU as 2.x) as well as the PRM opcode maps.
    # -----------------------------------------------------------------------

    # BCD: ABCD D3,D5 = 0xCB03; memory form sets R/M -> 0xCB0B
    check("ABCD D3,D5", abcd(5, 3), [0xCB03])
    check("ABCD -(A3),-(A5)", abcd(5, 3, rm=1), [0xCB0B])
    check("SBCD D3,D5", sbcd(5, 3), [0x8B03])
    check("SBCD -(A3),-(A5)", sbcd(5, 3, rm=1), [0x8B0B])
    check("NBCD D4", nbcd(DN, 4), [0x4804])
    check("NBCD -(A2)", nbcd(AN_PREDEC, 2), [0x4822])

    # PACK / UNPK
    check("PACK D1,D2,#$1234", pack(2, 1, 0x1234), [0x8541, 0x1234])
    check("PACK -(A1),-(A2),#0", pack(2, 1, 0, rm=1), [0x8549, 0x0000])
    check("UNPK D1,D2,#$3030", unpk(2, 1, 0x3030), [0x8581, 0x3030])
    check("UNPK -(A1),-(A2),#$3030", unpk(2, 1, 0x3030, rm=1), [0x8589, 0x3030])

    # ADDX / SUBX / CMPM
    check("ADDX.B D3,D5", addx(BYTE, 5, 3), [0xDB03])
    check("ADDX.L D3,D5", addx(LONG, 5, 3), [0xDB83])
    check("ADDX.L -(A3),-(A5)", addx(LONG, 5, 3, rm=1), [0xDB8B])
    check("SUBX.W D3,D5", subx(WORD, 5, 3), [0x9B43])
    check("SUBX.L -(A3),-(A5)", subx(LONG, 5, 3, rm=1), [0x9B8B])
    check("CMPM.B (A1)+,(A2)+", cmpm(BYTE, 2, 1), [0xB509])
    check("CMPM.W (A1)+,(A2)+", cmpm(WORD, 2, 1), [0xB549])
    check("CMPM.L (A1)+,(A2)+", cmpm(LONG, 2, 1), [0xB589])

    # Bit fields (all eight, plus register-sourced offset/width)
    check("BFTST D0{2:5}", bftst(DN, 0, 2, 5), [0xE8C0, 0x0085])
    check("BFTST (A1){0:32}", bftst(AN_IND, 1, 0, 0), [0xE8D1, 0x0000])
    check("BFTST (A1){D2:D3}", bftst(AN_IND, 1, 2, 3, do=1, dw=1),
          [0xE8D1, 0x08A3])
    check("BFEXTU D0{2:5},D7", bfextu(DN, 0, 2, 5, 7), [0xE9C0, 0x7085])
    check("BFCHG D0{4:8}", bfchg(DN, 0, 4, 8), [0xEAC0, 0x0108])
    check("BFEXTS (A1){7:13},D4", bfexts(AN_IND, 1, 7, 13, 4), [0xEBD1, 0x41CD])
    check("BFCLR (A1){3:9}", bfclr(AN_IND, 1, 3, 9), [0xECD1, 0x00C9])
    check("BFFFO D0{0:32},D1", bfffo(DN, 0, 0, 0, 1), [0xEDC0, 0x1000])
    check("BFSET (A1){31:1}", bfset(AN_IND, 1, 31, 1), [0xEED1, 0x07C1])
    check("BFINS D5,D0{2:5}", bfins(5, DN, 0, 2, 5), [0xEFC0, 0x5085])

    # CAS / CAS2
    check("CAS.B D1,D2,(A3)", cas(BYTE, 1, 2, AN_IND, 3), [0x0AD3, 0x0081])
    check("CAS.W D1,D2,(A3)", cas(WORD, 1, 2, AN_IND, 3), [0x0CD3, 0x0081])
    check("CAS.L D1,D2,(A3)", cas(LONG, 1, 2, AN_IND, 3), [0x0ED3, 0x0081])
    check("CAS2.L D1:D2,D3:D4,(A5):(A6)",
          cas2(LONG, 1, 3, 5, 1, 2, 4, 6, 1), [0x0EFC, 0xD0C1, 0xE102])
    check("CAS2.W D1:D2,D3:D4,(D5):(D6)",
          cas2(WORD, 1, 3, 5, 0, 2, 4, 6, 0), [0x0CFC, 0x50C1, 0x6102])

    # MOVEP / MOVES / MOVEC
    check("MOVEP.W D3,$10(A2)", movep_to_mem(WORD, 3, 2, 0x10), [0x078A, 0x0010])
    check("MOVEP.L D3,$10(A2)", movep_to_mem(LONG, 3, 2, 0x10), [0x07CA, 0x0010])
    check("MOVEP.W $10(A2),D3", movep_to_reg(WORD, 3, 2, 0x10), [0x070A, 0x0010])
    check("MOVEP.L $10(A2),D3", movep_to_reg(LONG, 3, 2, 0x10), [0x074A, 0x0010])
    check("MOVES.L D3,(A2)", moves(LONG, 3, 0, 1, AN_IND, 2), [0x0E92, 0x3800])
    check("MOVES.L (A2),D3", moves(LONG, 3, 0, 0, AN_IND, 2), [0x0E92, 0x3000])
    check("MOVES.B D3,(A2)", moves(BYTE, 3, 0, 1, AN_IND, 2), [0x0E12, 0x3800])
    check("MOVEC D0,SFC", movec_to_cr(0, CR_SFC), [0x4E7B, 0x0000])
    check("MOVEC D1,DFC", movec_to_cr(1, CR_DFC), [0x4E7B, 0x1001])
    check("MOVEC SFC,D2", movec_from_cr(CR_SFC, 2), [0x4E7A, 0x2000])

    # Long multiply / divide (MC68020+)
    check("MULU.L D2,D0", mulu_l(DN, 2, 0), [0x4C02, 0x0000])
    check("MULS.L D2,D0", muls_l(DN, 2, 0), [0x4C02, 0x0800])
    check("MULU.L D2,D3:D0", mulu_l(DN, 2, 0, 3), [0x4C02, 0x0403])
    check("MULS.L D2,D3:D0", muls_l(DN, 2, 0, 3), [0x4C02, 0x0C03])
    check("DIVU.L D2,D0", divu_l(DN, 2, 0), [0x4C42, 0x0000])
    check("DIVS.L D2,D0", divs_l(DN, 2, 0), [0x4C42, 0x0800])
    check("DIVU.L D2,D1:D0", divu_l(DN, 2, 0, 1, size64=True), [0x4C42, 0x0401])
    check("DIVS.L D2,D1:D0", divs_l(DN, 2, 0, 1, size64=True), [0x4C42, 0x0C01])
    check("DIVUL.L D2,D1:D0", divu_l(DN, 2, 0, 1), [0x4C42, 0x0001])
    check("DIVSL.L D2,D1:D0", divs_l(DN, 2, 0, 1), [0x4C42, 0x0801])

    if errors:
        print(f"FAIL: {len(errors)} encoding error(s):")
        for e in errors:
            print(e)
        return False
    else:
        print(f"PASS: All encoding self-tests passed.")
        return True


if __name__ == "__main__":
    import sys
    ok = _self_test()
    sys.exit(0 if ok else 1)
