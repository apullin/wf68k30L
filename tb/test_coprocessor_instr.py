"""
MC68030 coprocessor-instruction exception surface tests.

Current HW-003 model scope in this phase:
  - Correct privilege-vs-F-line trap classification for coprocessor opwords
    when no coprocessor interface is implemented.
"""

import cocotb
from cocotb.triggers import RisingEdge
from typing import Optional

from cpu_harness import CPUTestHarness
from m68k_encode import (
    LONG,
    WORD,
    DN,
    AN_DISP,
    SPECIAL,
    IMMEDIATE,
    move,
    movea,
    moveq,
    move_to_abs_long,
    move_to_sr,
    disp16,
    imm_word,
    imm_long,
)


# Raw F-line coprocessor opwords used in this HW-003 phase.
CP_GEN_A0_CPID1 = 0xF210               # 1111 cpid=001 000 (A0)
CP_SCC_D0_CPID1 = 0xF240               # 1111 cpid=001 001 ... D0
CP_DBCC_D0_CPID1 = 0xF248              # 1111 cpid=001 001001 D0
CP_TRAPCC_NOP_CPID1 = 0xF27C           # 1111 cpid=001 001111100
CP_BCCW_CC0_CPID1 = 0xF280             # 1111 cpid=001 010 size=0 cc=0
CP_SAVE_A0_CPID1 = 0xF310              # 1111 cpid=001 100 (A0)
CP_RESTORE_A0_CPID1 = 0xF350           # 1111 cpid=001 101 (A0)
CP_SAVE_A0_CPID0 = 0xF110              # 1111 cpid=000 100 (A0)
CP_RESTORE_A0_CPID0 = 0xF150           # 1111 cpid=000 101 (A0)
CP_SAVE_IMM_CPID1 = 0xF33C             # cpSAVE with immediate EA (invalid)
CP_RESTORE_PREDEC_A0_CPID1 = 0xF360    # cpRESTORE with predecrement EA (invalid)
F_RESERVED_CPID0 = 0xF180              # 1111 cpid=000 110 xxx

# CPIF category model codes (sv/wf68k30L_top_sections/helpers/...coprocessor.svh).
CPIF_CAT_GEN = 1
CPIF_CAT_COND = 2
CPIF_CAT_BCC = 3
CPIF_CAT_SAVE = 4
CPIF_CAT_RESTORE = 5
CPIF_EXC_PRE = 0
CPIF_EXC_MID = 1
CPIF_EXC_POST = 2

VEC_TRAPCC = 7
VEC_LINEF = 11
VEC_CP_PROTO = 13


def _trap_handler_words(h: CPUTestHarness, marker: int):
    return [
        *moveq(marker & 0x7F, 1),
        *move_to_abs_long(LONG, DN, 1, h.RESULT_BASE),
        *h.sentinel_program(),
    ]


def _trap_handler_fmtvec_words(h: CPUTestHarness, marker: int):
    # Writes marker to RESULT_BASE and stacked format/vector word to RESULT_BASE+4.
    return [
        *moveq(marker & 0x7F, 1),
        *move_to_abs_long(LONG, DN, 1, h.RESULT_BASE),
        *move(WORD, AN_DISP, 7, DN, 2),   # D2 = [SP+6] format/vector
        *disp16(6),
        *move_to_abs_long(LONG, DN, 2, h.RESULT_BASE + 4),
        *h.sentinel_program(),
    ]


def _configure_cpif_model(
    dut,
    *,
    kind: int,
    vector: int,
    enable: bool = True,
    on_noresp: bool = False,
    resp_enable: Optional[bool] = None,
    resp_delay: Optional[int] = None,
    resp_cir: Optional[int] = None,
):
    dut.CPIF_MODEL_EXC_ENABLE.value = 1 if enable else 0
    dut.CPIF_MODEL_EXC_ON_NORESP.value = 1 if on_noresp else 0
    dut.CPIF_MODEL_EXC_KIND.value = kind
    dut.CPIF_MODEL_EXC_VECTOR.value = vector
    if resp_enable is not None:
        dut.CPIF_MODEL_RESP_ENABLE.value = 1 if resp_enable else 0
    if resp_delay is not None:
        dut.CPIF_MODEL_RESP_DELAY.value = resp_delay & 0xF
    if resp_cir is not None:
        dut.CPIF_MODEL_RESP_CIR.value = resp_cir & 0x1F


def _install_trap_handlers(h: CPUTestHarness):
    # Vector 8 (privilege), 11 (F-line emulator), and 13 (coprocessor protocol).
    vec_priv = 8 * 4
    vec_linef = 11 * 4
    vec_cp_proto = 13 * 4
    priv_handler = 0x000660
    linef_handler = 0x000680
    proto_handler = 0x0006A0

    h.mem.load_long(vec_priv, priv_handler)
    h.mem.load_long(vec_linef, linef_handler)
    h.mem.load_long(vec_cp_proto, proto_handler)
    h.mem.load_words(priv_handler, _trap_handler_words(h, 0x08))
    h.mem.load_words(linef_handler, _trap_handler_words(h, 0x0B))
    h.mem.load_words(proto_handler, _trap_handler_words(h, 0x0D))


def _cpif_snapshot(dut):
    return {
        "state": int(dut.CPIF_STATE.value),
        "active": int(dut.CPIF_ACTIVE.value),
        "last_cat": int(dut.CPIF_LAST_CAT.value),
        "last_cpid": int(dut.CPIF_LAST_CPID.value),
        "last_cir": int(dut.CPIF_LAST_CIR.value),
        "last_opword": int(dut.CPIF_LAST_OPWORD.value),
        "last_read": int(dut.CPIF_LAST_READ.value),
        "last_write": int(dut.CPIF_LAST_WRITE.value),
        "last_responded": int(dut.CPIF_LAST_RESPONDED.value),
        "last_resp_cir": int(dut.CPIF_LAST_RESP_CIR.value),
        "last_phase_len": int(dut.CPIF_LAST_PHASE_LEN.value),
        "phase_steps_left": int(dut.CPIF_PHASE_STEPS_LEFT.value),
        "resp_delay_left": int(dut.CPIF_RESP_DELAY_LEFT.value),
        "step_index": int(dut.CPIF_STEP_INDEX.value),
        "step_cir": int(dut.CPIF_STEP_CIR.value),
        "step_read": int(dut.CPIF_STEP_READ.value),
        "step_write": int(dut.CPIF_STEP_WRITE.value),
        "resp_cir_expect": int(dut.CPIF_RESP_CIR_EXPECT.value),
        "req_count": int(dut.CPIF_REQ_COUNT.value),
        "step_count": int(dut.CPIF_STEP_COUNT.value),
        "resp_count": int(dut.CPIF_RESP_COUNT.value),
        "noresp_count": int(dut.CPIF_NORESP_COUNT.value),
        "timeout_count": int(dut.CPIF_TIMEOUT_COUNT.value),
        "badresp_count": int(dut.CPIF_BADRESP_COUNT.value),
        "priv_bypass_count": int(dut.CPIF_PRIV_BYPASS_COUNT.value),
        "linef_noproto_count": int(dut.CPIF_LINEF_NOPROTO_COUNT.value),
        "model_exc_enable": int(dut.CPIF_MODEL_EXC_ENABLE.value),
        "model_exc_on_noresp": int(dut.CPIF_MODEL_EXC_ON_NORESP.value),
        "model_exc_kind": int(dut.CPIF_MODEL_EXC_KIND.value),
        "model_exc_vector": int(dut.CPIF_MODEL_EXC_VECTOR.value),
        "model_resp_enable": int(dut.CPIF_MODEL_RESP_ENABLE.value),
        "model_resp_delay": int(dut.CPIF_MODEL_RESP_DELAY.value),
        "model_resp_cir": int(dut.CPIF_MODEL_RESP_CIR.value),
        "trap_pre": int(dut.CPIF_TRAP_PRE.value),
        "trap_mid": int(dut.CPIF_TRAP_MID.value),
        "trap_post": int(dut.CPIF_TRAP_POST.value),
        "trap_vector": int(dut.CPIF_TRAP_VECTOR.value),
        "pre_exc_count": int(dut.CPIF_PRE_EXC_COUNT.value),
        "mid_exc_count": int(dut.CPIF_MID_EXC_COUNT.value),
        "post_exc_count": int(dut.CPIF_POST_EXC_COUNT.value),
        "proto_count": int(dut.CPIF_PROTO_COUNT.value),
    }


async def _run_single_trap_case(
    dut,
    *,
    opword: int,
    expected_marker: int,
    case_name: str,
    user_mode: bool = False,
    a0_addr: Optional[int] = None,
    capture_cpif: bool = False,
    cpif_model_kind: Optional[int] = None,
    cpif_model_vector: Optional[int] = None,
    cpif_model_on_noresp: bool = False,
    cpif_model_enable: bool = True,
    cpif_model_resp_enable: Optional[bool] = None,
    cpif_model_resp_delay: Optional[int] = None,
    cpif_model_resp_cir: Optional[int] = None,
):
    h = CPUTestHarness(dut)
    program = []

    if a0_addr is not None:
        program.extend(movea(LONG, SPECIAL, IMMEDIATE, 0))
        program.extend(imm_long(a0_addr))

    if user_mode:
        program.extend(move(WORD, SPECIAL, IMMEDIATE, DN, 5))
        program.extend(imm_word(0x0000))  # S=0 user mode
        program.extend(move_to_sr(DN, 5))

    program.extend(
        [
            opword,
            *moveq(0x66, 1),  # fallback marker if no trap
            *move_to_abs_long(LONG, DN, 1, h.RESULT_BASE),
            *h.sentinel_program(),
        ]
    )

    await h.setup(program)
    _install_trap_handlers(h)
    if cpif_model_kind is not None:
        _configure_cpif_model(
            dut,
            kind=cpif_model_kind,
            vector=cpif_model_vector if cpif_model_vector is not None else VEC_LINEF,
            enable=cpif_model_enable,
            on_noresp=cpif_model_on_noresp,
            resp_enable=cpif_model_resp_enable,
            resp_delay=cpif_model_resp_delay,
            resp_cir=cpif_model_resp_cir,
        )
    elif cpif_model_resp_enable is not None or cpif_model_resp_delay is not None or cpif_model_resp_cir is not None:
        if cpif_model_resp_enable is not None:
            dut.CPIF_MODEL_RESP_ENABLE.value = 1 if cpif_model_resp_enable else 0
        if cpif_model_resp_delay is not None:
            dut.CPIF_MODEL_RESP_DELAY.value = cpif_model_resp_delay & 0xF
        if cpif_model_resp_cir is not None:
            dut.CPIF_MODEL_RESP_CIR.value = cpif_model_resp_cir & 0x1F

    found = await h.run_until_sentinel(max_cycles=10000)
    assert found, f"{case_name} did not complete"
    got = h.read_result_long(0)
    assert got == expected_marker, (
        f"{case_name}: expected marker 0x{expected_marker:08X}, got 0x{got:08X}"
    )
    cpif = _cpif_snapshot(dut) if capture_cpif else None
    h.cleanup()
    return cpif


async def _run_cpif_frame_case(
    dut,
    *,
    opword: int,
    vector: int,
    expected_marker: int,
    expected_format: int,
    expected_vec: int,
    cpif_model_kind: int,
    cpif_model_vector: int,
    a0_addr: Optional[int] = None,
):
    h = CPUTestHarness(dut)
    program = []

    if a0_addr is not None:
        program.extend(movea(LONG, SPECIAL, IMMEDIATE, 0))
        program.extend(imm_long(a0_addr))

    program.extend(
        [
            opword,
            *moveq(0x66, 1),  # fallback marker if no trap
            *move_to_abs_long(LONG, DN, 1, h.RESULT_BASE),
            *h.sentinel_program(),
        ]
    )

    await h.setup(program)
    _install_trap_handlers(h)
    handler_addr = 0x0006C0 + (vector << 5)
    h.mem.load_long(vector * 4, handler_addr)
    h.mem.load_words(handler_addr, _trap_handler_fmtvec_words(h, expected_marker))
    _configure_cpif_model(
        dut,
        kind=cpif_model_kind,
        vector=cpif_model_vector,
        enable=True,
        on_noresp=False,
    )

    found = await h.run_until_sentinel(max_cycles=12000)
    assert found, "cpif frame case did not complete"
    marker = h.read_result_long(0)
    assert marker == expected_marker, (
        f"Expected marker 0x{expected_marker:08X}, got 0x{marker:08X}"
    )
    fmtvec = h.read_result_long(4) & 0xFFFF
    fmt = (fmtvec >> 12) & 0xF
    vec = fmtvec & 0x0FFF
    assert fmt == expected_format, (
        f"Expected stack format {expected_format:X}, got {fmt:X} (fmtvec=0x{fmtvec:04X})"
    )
    assert vec == (expected_vec << 2), (
        f"Expected vector offset 0x{expected_vec << 2:03X}, got 0x{vec:03X}"
    )
    cpif = _cpif_snapshot(dut)
    h.cleanup()
    return cpif


@cocotb.test()
async def test_cpsave_user_mode_privilege_violation(dut):
    """cpSAVE with nonzero CpID is privilege-checked in user mode."""
    await _run_single_trap_case(
        dut,
        opword=CP_SAVE_A0_CPID1,
        expected_marker=0x00000008,
        case_name="cpSAVE CpID=1 user mode privilege",
        user_mode=True,
        a0_addr=0x00005040,
    )


@cocotb.test()
async def test_cprestore_user_mode_privilege_violation(dut):
    """cpRESTORE with nonzero CpID is privilege-checked in user mode."""
    await _run_single_trap_case(
        dut,
        opword=CP_RESTORE_A0_CPID1,
        expected_marker=0x00000008,
        case_name="cpRESTORE CpID=1 user mode privilege",
        user_mode=True,
        a0_addr=0x00005080,
    )


@cocotb.test()
async def test_fline_reserved_cpid0_user_mode_is_linef_not_priv(dut):
    """Reserved F-line with CpID=000 should trap vector 11, not vector 8."""
    await _run_single_trap_case(
        dut,
        opword=F_RESERVED_CPID0,
        expected_marker=0x0000000B,
        case_name="reserved F-line CpID=0 user mode",
        user_mode=True,
    )


@cocotb.test()
async def test_cpsave_cprestore_cpid0_user_mode_are_linef(dut):
    """cpSAVE/cpRESTORE with CpID=000 should be Line-F, not privilege."""
    await _run_single_trap_case(
        dut,
        opword=CP_SAVE_A0_CPID0,
        expected_marker=0x0000000B,
        case_name="cpSAVE CpID=0 user mode line-F",
        user_mode=True,
        a0_addr=0x00005100,
    )
    await _run_single_trap_case(
        dut,
        opword=CP_RESTORE_A0_CPID0,
        expected_marker=0x0000000B,
        case_name="cpRESTORE CpID=0 user mode line-F",
        user_mode=True,
        a0_addr=0x00005140,
    )


@cocotb.test()
async def test_cpsave_cprestore_invalid_ea_user_mode_are_linef(dut):
    """Invalid cpSAVE/cpRESTORE EA forms should remain Line-F, not privilege."""
    await _run_single_trap_case(
        dut,
        opword=CP_SAVE_IMM_CPID1,
        expected_marker=0x0000000B,
        case_name="cpSAVE invalid EA user mode line-F",
        user_mode=True,
    )
    await _run_single_trap_case(
        dut,
        opword=CP_RESTORE_PREDEC_A0_CPID1,
        expected_marker=0x0000000B,
        case_name="cpRESTORE invalid EA user mode line-F",
        user_mode=True,
    )


@cocotb.test()
async def test_cpsave_cprestore_cpid1_supervisor_without_cp_interface_are_linef(dut):
    """In supervisor mode, cpSAVE/cpRESTORE reach F-line path without a coprocessor interface."""
    await _run_single_trap_case(
        dut,
        opword=CP_SAVE_A0_CPID1,
        expected_marker=0x0000000B,
        case_name="cpSAVE CpID=1 supervisor line-F (no cp interface)",
        user_mode=False,
        a0_addr=0x00005180,
    )
    await _run_single_trap_case(
        dut,
        opword=CP_RESTORE_A0_CPID1,
        expected_marker=0x0000000B,
        case_name="cpRESTORE CpID=1 supervisor line-F (no cp interface)",
        user_mode=False,
        a0_addr=0x000051C0,
    )


@cocotb.test()
async def test_other_coprocessor_opwords_surface_as_linef(dut):
    """cpBcc/cpDBcc/cpGEN/cpScc/cpTRAPcc opwords currently surface as Line-F."""
    opwords = (
        ("cpGEN", CP_GEN_A0_CPID1),
        ("cpScc", CP_SCC_D0_CPID1),
        ("cpDBcc", CP_DBCC_D0_CPID1),
        ("cpTRAPcc", CP_TRAPCC_NOP_CPID1),
        ("cpBcc", CP_BCCW_CC0_CPID1),
    )
    for name, opword in opwords:
        await _run_single_trap_case(
            dut,
            opword=opword,
            expected_marker=0x0000000B,
            case_name=f"{name} line-F surface",
            user_mode=False,
            a0_addr=0x00005200,
        )


@cocotb.test()
async def test_cpif_protocol_launch_for_cpgen(dut):
    """Phase-4: cpGEN should launch one modeled CIR protocol attempt."""
    cpif = await _run_single_trap_case(
        dut,
        opword=CP_GEN_A0_CPID1,
        expected_marker=0x0000000B,
        case_name="cpGEN supervisor line-F with CPIF model",
        user_mode=False,
        a0_addr=0x00005240,
        capture_cpif=True,
    )
    assert cpif["req_count"] == 1
    assert cpif["noresp_count"] == 1
    assert cpif["priv_bypass_count"] == 0
    assert cpif["linef_noproto_count"] == 0
    assert cpif["last_cat"] == CPIF_CAT_GEN
    assert cpif["last_cpid"] == 1
    assert cpif["last_cir"] == 0x0A
    assert cpif["last_write"] == 1
    assert cpif["last_read"] == 0
    assert cpif["state"] == 0 and cpif["active"] == 0


@cocotb.test()
async def test_cpif_protocol_launch_for_cpsave_and_cprestore(dut):
    """Phase-4: cpSAVE/cpRESTORE should record their modeled initiator CIR access."""
    cpif = await _run_single_trap_case(
        dut,
        opword=CP_SAVE_A0_CPID1,
        expected_marker=0x0000000B,
        case_name="cpSAVE supervisor line-F with CPIF model",
        user_mode=False,
        a0_addr=0x00005280,
        capture_cpif=True,
    )
    assert cpif["req_count"] == 1
    assert cpif["noresp_count"] == 1
    assert cpif["last_cat"] == CPIF_CAT_SAVE
    assert cpif["last_cir"] == 0x04
    assert cpif["last_read"] == 1
    assert cpif["last_write"] == 0

    cpif = await _run_single_trap_case(
        dut,
        opword=CP_RESTORE_A0_CPID1,
        expected_marker=0x0000000B,
        case_name="cpRESTORE supervisor line-F with CPIF model",
        user_mode=False,
        a0_addr=0x000052C0,
        capture_cpif=True,
    )
    assert cpif["req_count"] == 1
    assert cpif["noresp_count"] == 1
    assert cpif["last_cat"] == CPIF_CAT_RESTORE
    assert cpif["last_cir"] == 0x06
    assert cpif["last_read"] == 0
    assert cpif["last_write"] == 1


@cocotb.test()
async def test_cpif_privilege_bypass_for_user_cpsave(dut):
    """Phase-4: user-mode cpSAVE privilege trap should bypass modeled protocol launch."""
    cpif = await _run_single_trap_case(
        dut,
        opword=CP_SAVE_A0_CPID1,
        expected_marker=0x00000008,
        case_name="cpSAVE user privilege with CPIF bypass count",
        user_mode=True,
        a0_addr=0x00005300,
        capture_cpif=True,
    )
    assert cpif["req_count"] == 0
    assert cpif["noresp_count"] == 0
    assert cpif["linef_noproto_count"] == 0


@cocotb.test()
async def test_cpif_invalid_ea_linef_is_non_protocol(dut):
    """Phase-4: invalid cpSAVE/cpRESTORE line-F forms are tracked as non-protocol."""
    cpif = await _run_single_trap_case(
        dut,
        opword=CP_SAVE_IMM_CPID1,
        expected_marker=0x0000000B,
        case_name="cpSAVE invalid EA line-F non-protocol",
        user_mode=True,
        capture_cpif=True,
    )
    assert cpif["req_count"] == 0
    assert cpif["noresp_count"] == 0
    assert cpif["priv_bypass_count"] == 0
    assert cpif["linef_noproto_count"] == 1


@cocotb.test()
async def test_cpif_phase5_pre_exception_frame_vector11_format0(dut):
    """Phase-5: modeled pre-instruction CP exception uses vector 11 + format 0."""
    cpif = await _run_cpif_frame_case(
        dut,
        opword=CP_GEN_A0_CPID1,
        vector=VEC_LINEF,
        expected_marker=0x31,
        expected_format=0x0,
        expected_vec=VEC_LINEF,
        cpif_model_kind=CPIF_EXC_PRE,
        cpif_model_vector=VEC_LINEF,
        a0_addr=0x00005340,
    )
    assert cpif["pre_exc_count"] == 1
    assert cpif["mid_exc_count"] == 0
    assert cpif["post_exc_count"] == 0


@cocotb.test()
async def test_cpif_phase5_mid_exception_frame_protocol_vector13_format9(dut):
    """Phase-5: modeled protocol-violation path uses vector 13 + format 9."""
    cpif = await _run_cpif_frame_case(
        dut,
        opword=CP_GEN_A0_CPID1,
        vector=VEC_CP_PROTO,
        expected_marker=0x32,
        expected_format=0x9,
        expected_vec=VEC_CP_PROTO,
        cpif_model_kind=CPIF_EXC_MID,
        cpif_model_vector=VEC_CP_PROTO,
        a0_addr=0x00005380,
    )
    assert cpif["pre_exc_count"] == 0
    assert cpif["mid_exc_count"] == 1
    assert cpif["post_exc_count"] == 0
    assert cpif["proto_count"] == 1


@cocotb.test()
async def test_cpif_phase5_post_exception_frame_vector7_format2(dut):
    """Phase-5: modeled post-instruction CP exception uses vector 7 + format 2."""
    cpif = await _run_cpif_frame_case(
        dut,
        opword=CP_GEN_A0_CPID1,
        vector=VEC_TRAPCC,
        expected_marker=0x33,
        expected_format=0x2,
        expected_vec=VEC_TRAPCC,
        cpif_model_kind=CPIF_EXC_POST,
        cpif_model_vector=VEC_TRAPCC,
        a0_addr=0x000053C0,
    )
    assert cpif["pre_exc_count"] == 0
    assert cpif["mid_exc_count"] == 0
    assert cpif["post_exc_count"] == 1


@cocotb.test()
async def test_cpif_protocol_response_model_short_circuits_timeout(dut):
    """Response-enabled model should complete protocol without no-response timeout."""
    cpif = await _run_single_trap_case(
        dut,
        opword=CP_GEN_A0_CPID1,
        expected_marker=0x0000000B,
        case_name="cpGEN protocol response model",
        user_mode=False,
        a0_addr=0x00005420,
        capture_cpif=True,
        cpif_model_enable=False,
        cpif_model_resp_enable=True,
        cpif_model_resp_delay=1,
    )
    assert cpif["req_count"] == 1
    assert cpif["resp_count"] == 1
    assert cpif["timeout_count"] == 0
    assert cpif["noresp_count"] == 0
    assert cpif["last_responded"] == 1
    assert cpif["last_resp_cir"] == cpif["resp_cir_expect"]
    assert cpif["last_phase_len"] == 4
    assert 0 < cpif["phase_steps_left"] < cpif["last_phase_len"]
    assert cpif["step_count"] < cpif["last_phase_len"]
    assert cpif["state"] == 0 and cpif["active"] == 0


@cocotb.test()
async def test_cpif_protocol_timeout_is_bounded_by_category_steps(dut):
    """Timeout path should consume the modeled step budget for the launched category."""
    cpif = await _run_single_trap_case(
        dut,
        opword=CP_SAVE_A0_CPID1,
        expected_marker=0x0000000B,
        case_name="cpSAVE protocol timeout bounded steps",
        user_mode=False,
        a0_addr=0x00005440,
        capture_cpif=True,
        cpif_model_enable=False,
        cpif_model_resp_enable=False,
    )
    assert cpif["req_count"] == 1
    assert cpif["resp_count"] == 0
    assert cpif["timeout_count"] == 1
    assert cpif["noresp_count"] == 1
    assert cpif["last_phase_len"] == 2
    assert cpif["step_count"] == 2
    assert cpif["phase_steps_left"] == 0


@cocotb.test()
async def test_cpif_cpgen_step_sequence_is_deterministic(dut):
    """cpGEN protocol-attempt steps should follow the modeled CIR/read-write sequence."""
    h = CPUTestHarness(dut)
    program = [
        *movea(LONG, SPECIAL, IMMEDIATE, 0),
        *imm_long(0x00005480),
        CP_GEN_A0_CPID1,
        *moveq(0x66, 1),
        *move_to_abs_long(LONG, DN, 1, h.RESULT_BASE),
        *h.sentinel_program(),
    ]

    await h.setup(program)
    _install_trap_handlers(h)
    dut.CPIF_MODEL_EXC_ENABLE.value = 0
    dut.CPIF_MODEL_RESP_ENABLE.value = 0

    trace = []
    found = False
    for _ in range(20000):
        await RisingEdge(dut.CLK)
        try:
            if int(dut.CPIF_STEP_PULSE.value):
                trace.append(
                    (
                        int(dut.CPIF_STEP_INDEX.value),
                        int(dut.CPIF_STEP_CIR.value),
                        int(dut.CPIF_STEP_READ.value),
                        int(dut.CPIF_STEP_WRITE.value),
                    )
                )
        except ValueError:
            pass

        if h.mem.read(h.SENTINEL_ADDR, 4) == h.SENTINEL_VAL:
            found = True
            break

    assert found, "cpGEN step-sequence test did not complete"
    assert h.read_result_long(0) == 0x0000000B, "cpGEN should still surface through line-F path"
    assert trace == [
        (0, 0x0A, 0, 1),
        (1, 0x00, 1, 0),
        (2, 0x0B, 0, 1),
        (3, 0x00, 1, 0),
    ], f"Unexpected cpGEN step trace: {trace}"
    h.cleanup()


@cocotb.test()
async def test_cpif_bad_response_tracks_protocol_violation(dut):
    """Mismatched modeled response CIR should be counted as protocol violation."""
    cpif = await _run_single_trap_case(
        dut,
        opword=CP_GEN_A0_CPID1,
        expected_marker=0x0000000D,
        case_name="cpGEN bad-response protocol violation tracking",
        user_mode=False,
        a0_addr=0x000054A0,
        capture_cpif=True,
        cpif_model_enable=False,
        cpif_model_resp_enable=True,
        cpif_model_resp_delay=1,
        cpif_model_resp_cir=0x1F,
    )
    assert cpif["resp_count"] == 1
    assert cpif["badresp_count"] == 1
    assert cpif["proto_count"] == 1
    assert cpif["mid_exc_count"] == 1
    assert cpif["last_responded"] == 1
    assert cpif["last_resp_cir"] == 0x1F
    assert cpif["resp_cir_expect"] == 0x00
