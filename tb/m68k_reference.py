"""
MC68030 reference condition code model for cocotb testbench.

Pure Python implementation of the MC68030 condition code (CCR) rules
as defined in the MC68030 User Manual, Table 3-12 (Condition Code
Computations) and Table 3-13 (Conditional Tests).

All functions operate on plain Python integers. Size is encoded as
BYTE=0, WORD=1, LONG=2 matching m68k_encode conventions.

The CC result dictionaries use keys: 'x', 'n', 'z', 'v', 'c'
corresponding to the five CCR bits. Each value is 0 or 1.
"""

# ---------------------------------------------------------------------------
# Size constants and masks
# ---------------------------------------------------------------------------

BYTE = 0
WORD = 1
LONG = 2

BYTE_MASK = 0xFF
WORD_MASK = 0xFFFF
LONG_MASK = 0xFFFFFFFF

_SIZE_MASKS = {BYTE: BYTE_MASK, WORD: WORD_MASK, LONG: LONG_MASK}
_SIZE_BITS = {BYTE: 8, WORD: 16, LONG: 32}


def size_mask(size):
    """Return the bit mask for the given operand size."""
    return _SIZE_MASKS[size]


def size_bits(size):
    """Return the number of bits for the given operand size."""
    return _SIZE_BITS[size]


def msb(val, size):
    """Return the most significant bit (0 or 1) of val for the given size."""
    bits = _SIZE_BITS[size]
    return (val >> (bits - 1)) & 1


def mask_val(val, size):
    """Mask a value to the given operand size."""
    return val & _SIZE_MASKS[size]


# ---------------------------------------------------------------------------
# Condition code computations (Table 3-12)
# ---------------------------------------------------------------------------

def cc_add(size, src, dst, result):
    """Compute condition codes for ADD, ADDI, ADDQ.

    General rules:
      X = C
      N = Rm (MSB of result)
      Z = ~Rm & ~Rm-1 & ... & ~R0 (result == 0)
      V = Sm & Dm & ~Rm | ~Sm & ~Dm & Rm
      C = Sm & Dm | ~Rm & Dm | Sm & ~Rm

    Where S = source, D = destination (before operation), R = result.
    """
    sm = msb(src, size)
    dm = msb(dst, size)
    rm = msb(result, size)
    r = mask_val(result, size)

    v = (sm & dm & (rm ^ 1)) | ((sm ^ 1) & (dm ^ 1) & rm)
    c = (sm & dm) | ((rm ^ 1) & dm) | (sm & (rm ^ 1))
    n = rm
    z = 1 if r == 0 else 0

    return {'x': c, 'n': n, 'z': z, 'v': v, 'c': c}


def cc_addx(size, src, dst, result, old_z):
    """Compute condition codes for ADDX.

    Same as ADD but Z is only cleared, never set:
      Z = old_Z AND (result == 0)
    """
    cc = cc_add(size, src, dst, result)
    cc['z'] = old_z & cc['z']
    return cc


def cc_sub(size, src, dst, result):
    """Compute condition codes for SUB, SUBI, SUBQ.

    Operation: result = dst - src

    General rules:
      X = C
      N = Rm
      Z = (result == 0)
      V = ~Sm & Dm & ~Rm | Sm & ~Dm & Rm
      C = Sm & ~Dm | Rm & ~Dm | Sm & Rm

    Note: The "overline" notation in the manual is context-dependent.
    For subtraction, the V and C formulas use the convention that
    src is the "source" (subtrahend) and dst is the "destination" (minuend).
    """
    sm = msb(src, size)
    dm = msb(dst, size)
    rm = msb(result, size)
    r = mask_val(result, size)

    v = ((sm ^ 1) & dm & (rm ^ 1)) | (sm & (dm ^ 1) & rm)
    c = (sm & (dm ^ 1)) | (rm & (dm ^ 1)) | (sm & rm)
    n = rm
    z = 1 if r == 0 else 0

    return {'x': c, 'n': n, 'z': z, 'v': v, 'c': c}


def cc_subx(size, src, dst, result, old_z):
    """Compute condition codes for SUBX.

    Same as SUB but Z is only cleared, never set.
    """
    cc = cc_sub(size, src, dst, result)
    cc['z'] = old_z & cc['z']
    return cc


def cc_cmp(size, src, dst, result):
    """Compute condition codes for CMP, CMPI, CMPM.

    Same as SUB but X is not affected. We return X=None to indicate
    it should not be updated.
    """
    cc = cc_sub(size, src, dst, result)
    cc['x'] = None  # X not affected
    return cc


def cc_logic(size, result):
    """Compute condition codes for logic/move operations.

    Used by: AND, ANDI, OR, ORI, EOR, EORI, MOVEQ, MOVE, CLR, EXT, NOT, TAS, TST

    Rules:
      X = not affected
      N = Rm (MSB of result)
      Z = (result == 0)
      V = 0
      C = 0
    """
    r = mask_val(result, size)
    return {
        'x': None,  # X not affected
        'n': msb(result, size),
        'z': 1 if r == 0 else 0,
        'v': 0,
        'c': 0,
    }


def cc_neg(size, dst, result):
    """Compute condition codes for NEG.

    Operation: result = 0 - dst

    Rules:
      X = C
      N = Rm
      Z = (result == 0)
      V = Dm & Rm    (both destination MSB and result MSB are set)
      C = Dm | Rm    (either destination MSB or result MSB is set)
    """
    dm = msb(dst, size)
    rm = msb(result, size)
    r = mask_val(result, size)

    v = dm & rm
    c = dm | rm
    n = rm
    z = 1 if r == 0 else 0

    return {'x': c, 'n': n, 'z': z, 'v': v, 'c': c}


def cc_negx(size, dst, result, old_z):
    """Compute condition codes for NEGX.

    Same as NEG but Z is only cleared, never set.
    """
    cc = cc_neg(size, dst, result)
    cc['z'] = old_z & cc['z']
    return cc


# ---------------------------------------------------------------------------
# Shift/Rotate condition codes (Table 3-12, Sheet 2)
# ---------------------------------------------------------------------------

def cc_shift(op, size, count, src, result, x_in):
    """Compute condition codes for shift and rotate operations.

    Args:
        op: One of 'asl', 'asr', 'lsl', 'lsr', 'rol', 'ror', 'roxl', 'roxr'
        size: BYTE, WORD, or LONG
        count: Shift count (may be 0)
        src: Original value before shift
        result: Value after shift
        x_in: Current X flag value (needed for ROXx with count=0)

    Returns:
        Dict with 'x', 'n', 'z', 'v', 'c' condition codes.
        'x' is None if X is not affected.
    """
    r = mask_val(result, size)
    bits = _SIZE_BITS[size]
    rm = msb(result, size)
    n = rm
    z = 1 if r == 0 else 0

    if op == 'asl':
        if count == 0:
            return {'x': None, 'n': n, 'z': z, 'v': 0, 'c': 0}
        # C = last bit shifted out = bit (m - count + 1) of source
        # But for count >= bits, C = 0 (all bits shifted out)
        if count >= bits:
            c = 0
        else:
            c = (src >> (bits - count)) & 1
        # V = 1 if any bit changed sign during the shift
        # V = Dm & (~D(m-1) | ... | ~D(m-r)) | ~Dm & (D(m-1) | ... | D(m-r))
        # Simpler: check if the MSB changed at any point during the shift
        dm = msb(src, size)
        v = 0
        for i in range(1, min(count, bits) + 1):
            shifted = mask_val(src << i, size)
            if msb(shifted, size) != dm:
                v = 1
                break
        return {'x': c, 'n': n, 'z': z, 'v': v, 'c': c}

    elif op == 'asr':
        if count == 0:
            return {'x': None, 'n': n, 'z': z, 'v': 0, 'c': 0}
        # C = last bit shifted out = bit (count - 1) of source
        if count >= bits:
            c = msb(src, size)  # sign bit replicates
        else:
            c = (src >> (count - 1)) & 1
        return {'x': c, 'n': n, 'z': z, 'v': 0, 'c': c}

    elif op == 'lsl':
        if count == 0:
            return {'x': None, 'n': n, 'z': z, 'v': 0, 'c': 0}
        if count >= bits:
            c = 0 if count > bits else (src & 1)
        else:
            c = (src >> (bits - count)) & 1
        return {'x': c, 'n': n, 'z': z, 'v': 0, 'c': c}

    elif op == 'lsr':
        if count == 0:
            return {'x': None, 'n': n, 'z': z, 'v': 0, 'c': 0}
        if count >= bits:
            c = 0 if count > bits else msb(src, size)
        else:
            c = (src >> (count - 1)) & 1
        return {'x': c, 'n': n, 'z': z, 'v': 0, 'c': c}

    elif op == 'rol':
        if count == 0:
            return {'x': None, 'n': n, 'z': z, 'v': 0, 'c': 0}
        # C = last bit rotated (into bit 0) = LSB of result
        c = r & 1
        return {'x': None, 'n': n, 'z': z, 'v': 0, 'c': c}

    elif op == 'ror':
        if count == 0:
            return {'x': None, 'n': n, 'z': z, 'v': 0, 'c': 0}
        # C = last bit rotated (into MSB) = MSB of result
        c = rm
        return {'x': None, 'n': n, 'z': z, 'v': 0, 'c': c}

    elif op == 'roxl':
        if count == 0:
            # C = X (existing X flag)
            return {'x': None, 'n': n, 'z': z, 'v': 0, 'c': x_in}
        # C = last bit rotated out (through X)
        # For ROXL the carry chain includes X as an extra bit
        c = (src >> (bits - (count % (bits + 1)))) & 1 if count % (bits + 1) != 0 else x_in
        return {'x': c, 'n': n, 'z': z, 'v': 0, 'c': c}

    elif op == 'roxr':
        if count == 0:
            # C = X (existing X flag)
            return {'x': None, 'n': n, 'z': z, 'v': 0, 'c': x_in}
        c = (src >> ((count - 1) % (bits + 1))) & 1 if count % (bits + 1) != 0 else x_in
        return {'x': c, 'n': n, 'z': z, 'v': 0, 'c': c}

    else:
        raise ValueError(f"Unknown shift/rotate op: {op}")


# ---------------------------------------------------------------------------
# Conditional tests (Table 3-13)
# ---------------------------------------------------------------------------

# Condition code constants
CC_T = 0    # True
CC_F = 1    # False
CC_HI = 2   # High
CC_LS = 3   # Low or Same
CC_CC = 4   # Carry Clear
CC_CS = 5   # Carry Set
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


def eval_condition(cc_code, n, z, v, c):
    """Evaluate one of 16 conditional tests per Table 3-13.

    Args:
        cc_code: Condition code (0-15), matching CC_T through CC_LE.
        n: N flag (0 or 1)
        z: Z flag (0 or 1)
        v: V flag (0 or 1)
        c: C flag (0 or 1)

    Returns:
        True if condition is met, False otherwise.
    """
    if cc_code == CC_T:
        return True
    elif cc_code == CC_F:
        return False
    elif cc_code == CC_HI:
        # ~C & ~Z
        return (not c) and (not z)
    elif cc_code == CC_LS:
        # C | Z
        return bool(c or z)
    elif cc_code == CC_CC:
        # ~C
        return not c
    elif cc_code == CC_CS:
        # C
        return bool(c)
    elif cc_code == CC_NE:
        # ~Z
        return not z
    elif cc_code == CC_EQ:
        # Z
        return bool(z)
    elif cc_code == CC_VC:
        # ~V
        return not v
    elif cc_code == CC_VS:
        # V
        return bool(v)
    elif cc_code == CC_PL:
        # ~N
        return not n
    elif cc_code == CC_MI:
        # N
        return bool(n)
    elif cc_code == CC_GE:
        # N & V | ~N & ~V  (same sign)
        return bool((n and v) or (not n and not v))
    elif cc_code == CC_LT:
        # N & ~V | ~N & V  (different signs)
        return bool((n and not v) or (not n and v))
    elif cc_code == CC_GT:
        # N & V & ~Z | ~N & ~V & ~Z
        return bool((n and v and not z) or (not n and not v and not z))
    elif cc_code == CC_LE:
        # Z | N & ~V | ~N & V
        return bool(z or (n and not v) or (not n and v))
    else:
        raise ValueError(f"Invalid condition code: {cc_code}")


# ---------------------------------------------------------------------------
# Helper: apply CC result to a status register value
# ---------------------------------------------------------------------------

def apply_cc(sr, cc):
    """Apply a condition code result dict to a 16-bit status register value.

    Args:
        sr: Current 16-bit status register value.
        cc: Dict with keys 'x', 'n', 'z', 'v', 'c'. Values are 0, 1, or None.
             None means the flag is not affected.

    Returns:
        Updated 16-bit status register value.
    """
    result = sr
    if cc.get('c') is not None:
        result = (result & ~(1 << 0)) | (cc['c'] << 0)
    if cc.get('v') is not None:
        result = (result & ~(1 << 1)) | (cc['v'] << 1)
    if cc.get('z') is not None:
        result = (result & ~(1 << 2)) | (cc['z'] << 2)
    if cc.get('n') is not None:
        result = (result & ~(1 << 3)) | (cc['n'] << 3)
    if cc.get('x') is not None:
        result = (result & ~(1 << 4)) | (cc['x'] << 4)
    return result


def extract_cc(sr):
    """Extract condition code flags from a 16-bit status register.

    Returns:
        Tuple (x, n, z, v, c) as integers (0 or 1).
    """
    c = (sr >> 0) & 1
    v = (sr >> 1) & 1
    z = (sr >> 2) & 1
    n = (sr >> 3) & 1
    x = (sr >> 4) & 1
    return x, n, z, v, c


# ---------------------------------------------------------------------------
# Self-test
# ---------------------------------------------------------------------------

def _self_test():
    """Verify condition code computations against known results."""
    errors = []

    def check(name, got, expected):
        if got != expected:
            errors.append(f"  {name}: expected {expected}, got {got}")

    # ADD.B: 0x7F + 0x01 = 0x80 -> N=1, Z=0, V=1 (pos+pos=neg), C=0
    cc = cc_add(BYTE, 0x01, 0x7F, 0x80)
    check("ADD.B 0x7F+0x01 N", cc['n'], 1)
    check("ADD.B 0x7F+0x01 Z", cc['z'], 0)
    check("ADD.B 0x7F+0x01 V", cc['v'], 1)
    check("ADD.B 0x7F+0x01 C", cc['c'], 0)

    # ADD.B: 0xFF + 0x01 = 0x100 (0x00 masked) -> N=0, Z=1, V=0, C=1
    cc = cc_add(BYTE, 0x01, 0xFF, 0x100)
    check("ADD.B 0xFF+0x01 N", cc['n'], 0)
    check("ADD.B 0xFF+0x01 Z", cc['z'], 1)
    check("ADD.B 0xFF+0x01 V", cc['v'], 0)
    check("ADD.B 0xFF+0x01 C", cc['c'], 1)

    # SUB.L: 5 - 3 = 2 -> N=0, Z=0, V=0, C=0
    cc = cc_sub(LONG, 3, 5, 2)
    check("SUB.L 5-3 N", cc['n'], 0)
    check("SUB.L 5-3 Z", cc['z'], 0)
    check("SUB.L 5-3 V", cc['v'], 0)
    check("SUB.L 5-3 C", cc['c'], 0)

    # SUB.L: 3 - 5 = -2 (0xFFFFFFFE) -> N=1, Z=0, V=0, C=1
    cc = cc_sub(LONG, 5, 3, 0xFFFFFFFE)
    check("SUB.L 3-5 N", cc['n'], 1)
    check("SUB.L 3-5 Z", cc['z'], 0)
    check("SUB.L 3-5 V", cc['v'], 0)
    check("SUB.L 3-5 C", cc['c'], 1)

    # SUB.B: 0x80 - 0x01 = 0x7F -> N=0, Z=0, V=1 (neg-pos=pos overflow), C=0
    cc = cc_sub(BYTE, 0x01, 0x80, 0x7F)
    check("SUB.B 0x80-0x01 V", cc['v'], 1)
    check("SUB.B 0x80-0x01 C", cc['c'], 0)

    # CMP: X should be None
    cc = cc_cmp(LONG, 5, 5, 0)
    check("CMP 5-5 Z", cc['z'], 1)
    check("CMP 5-5 X", cc['x'], None)

    # Logic: MOVE #0 -> Z=1, N=0
    cc = cc_logic(LONG, 0)
    check("LOGIC 0 Z", cc['z'], 1)
    check("LOGIC 0 N", cc['n'], 0)
    check("LOGIC 0 V", cc['v'], 0)
    check("LOGIC 0 C", cc['c'], 0)

    # Logic: MOVE #0x80000000 -> Z=0, N=1
    cc = cc_logic(LONG, 0x80000000)
    check("LOGIC 0x80000000 N", cc['n'], 1)
    check("LOGIC 0x80000000 Z", cc['z'], 0)

    # NEG.B: 0 - 0x01 = 0xFF -> N=1, Z=0, V=0, C=1
    cc = cc_neg(BYTE, 0x01, 0xFF)
    check("NEG.B 0x01 N", cc['n'], 1)
    check("NEG.B 0x01 Z", cc['z'], 0)
    check("NEG.B 0x01 V", cc['v'], 0)
    check("NEG.B 0x01 C", cc['c'], 1)

    # NEG.B: 0 - 0x80 = 0x80 -> N=1, Z=0, V=1, C=1
    cc = cc_neg(BYTE, 0x80, 0x80)
    check("NEG.B 0x80 N", cc['n'], 1)
    check("NEG.B 0x80 V", cc['v'], 1)
    check("NEG.B 0x80 C", cc['c'], 1)

    # NEG.B: 0 - 0 = 0 -> N=0, Z=1, V=0, C=0
    cc = cc_neg(BYTE, 0x00, 0x00)
    check("NEG.B 0x00 Z", cc['z'], 1)
    check("NEG.B 0x00 C", cc['c'], 0)

    # Conditional tests
    check("CC_T", eval_condition(CC_T, 0, 0, 0, 0), True)
    check("CC_F", eval_condition(CC_F, 0, 0, 0, 0), False)
    check("CC_EQ z=1", eval_condition(CC_EQ, 0, 1, 0, 0), True)
    check("CC_EQ z=0", eval_condition(CC_EQ, 0, 0, 0, 0), False)
    check("CC_NE z=0", eval_condition(CC_NE, 0, 0, 0, 0), True)
    check("CC_NE z=1", eval_condition(CC_NE, 0, 1, 0, 0), False)
    check("CC_MI n=1", eval_condition(CC_MI, 1, 0, 0, 0), True)
    check("CC_PL n=0", eval_condition(CC_PL, 0, 0, 0, 0), True)
    check("CC_HI c=0,z=0", eval_condition(CC_HI, 0, 0, 0, 0), True)
    check("CC_HI c=1,z=0", eval_condition(CC_HI, 0, 0, 0, 1), False)
    check("CC_HI c=0,z=1", eval_condition(CC_HI, 0, 1, 0, 0), False)
    check("CC_LS c=1,z=0", eval_condition(CC_LS, 0, 0, 0, 1), True)
    check("CC_LS c=0,z=1", eval_condition(CC_LS, 0, 1, 0, 0), True)
    check("CC_LS c=0,z=0", eval_condition(CC_LS, 0, 0, 0, 0), False)
    check("CC_GE n=1,v=1", eval_condition(CC_GE, 1, 0, 1, 0), True)
    check("CC_GE n=0,v=0", eval_condition(CC_GE, 0, 0, 0, 0), True)
    check("CC_GE n=1,v=0", eval_condition(CC_GE, 1, 0, 0, 0), False)
    check("CC_LT n=1,v=0", eval_condition(CC_LT, 1, 0, 0, 0), True)
    check("CC_LT n=0,v=1", eval_condition(CC_LT, 0, 0, 1, 0), True)
    check("CC_GT n=0,v=0,z=0", eval_condition(CC_GT, 0, 0, 0, 0), True)
    check("CC_GT n=0,v=0,z=1", eval_condition(CC_GT, 0, 1, 0, 0), False)
    check("CC_LE z=1", eval_condition(CC_LE, 0, 1, 0, 0), True)
    check("CC_LE n=1,v=0,z=0", eval_condition(CC_LE, 1, 0, 0, 0), True)

    # apply_cc / extract_cc round-trip
    sr = 0x2700  # supervisor mode, IPL=7, all CC clear
    cc_result = {'x': 1, 'n': 1, 'z': 0, 'v': 0, 'c': 1}
    sr2 = apply_cc(sr, cc_result)
    x, n, z, v, c = extract_cc(sr2)
    check("apply/extract X", x, 1)
    check("apply/extract N", n, 1)
    check("apply/extract Z", z, 0)
    check("apply/extract V", v, 0)
    check("apply/extract C", c, 1)

    # X not affected
    sr3 = apply_cc(sr2, {'x': None, 'n': 0, 'z': 1, 'v': 0, 'c': 0})
    x3, n3, z3, v3, c3 = extract_cc(sr3)
    check("X preserved", x3, 1)  # Should still be 1
    check("N cleared", n3, 0)
    check("Z set", z3, 1)

    if errors:
        print(f"FAIL: {len(errors)} reference model error(s):")
        for e in errors:
            print(e)
        return False
    else:
        print("PASS: All reference model self-tests passed.")
        return True


if __name__ == "__main__":
    import sys
    ok = _self_test()
    sys.exit(0 if ok else 1)
