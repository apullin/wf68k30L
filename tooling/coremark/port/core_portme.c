/*
CoreMark port layer for WF68K30L bare-metal simulation.
*/
#include <stdarg.h>

#include "coremark.h"
#include "core_portme.h"

#define WF_RESULT_ADDR     ((volatile ee_u32 *)0x00020000u)
#define WF_VALIDATION_ADDR ((volatile ee_u32 *)0x00020008u)
#define WF_SEEDCRC_ADDR    ((volatile ee_u32 *)0x0002000Cu)
#define WF_DATASIZE_ADDR   ((volatile ee_u32 *)0x00020010u)
#define WF_CRCLIST_ADDR    ((volatile ee_u32 *)0x00020014u)
#define WF_CRCMATRIX_ADDR  ((volatile ee_u32 *)0x00020018u)
#define WF_CRCSTATE_ADDR   ((volatile ee_u32 *)0x0002001Cu)
#define WF_CRCFINAL_ADDR   ((volatile ee_u32 *)0x00020020u)
#define WF_SENTINEL_ADDR   ((volatile ee_u32 *)0x00030000u)
#define WF_SENTINEL_VAL    ((ee_u32)0xDEADCAFEu)

/* Published at WF_VALIDATION_ADDR; kept in sync with tb/test_coremark_smoke.py. */
#define WF_VALIDATION_UNKNOWN     ((ee_u32)0u)
#define WF_VALIDATION_OK          ((ee_u32)1u)
#define WF_VALIDATION_ERRORS      ((ee_u32)2u)
#define WF_VALIDATION_UNCHECKABLE ((ee_u32)3u)

static ee_u32 wf_validation_state = WF_VALIDATION_UNKNOWN;
static ee_u32 wf_seedcrc = 0;
static ee_u32 wf_data_size = 0;
static ee_u32 wf_crclist = 0;
static ee_u32 wf_crcmatrix = 0;
static ee_u32 wf_crcstate = 0;
static ee_u32 wf_crcfinal = 0;

#ifndef COREMARK_FAKE_TICKS
#define COREMARK_FAKE_TICKS ((CORE_TICKS)11000000u)
#endif

#ifndef COREMARK_TICKS_PER_SEC
#define COREMARK_TICKS_PER_SEC ((CORE_TICKS)1000000u)
#endif

#ifndef COREMARK_EXECS_MASK
#define COREMARK_EXECS_MASK (ID_LIST | ID_MATRIX | ID_STATE)
#endif

#ifndef COREMARK_SEED3
#define COREMARK_SEED3 0x66
#endif

#ifndef COREMARK_SEED1
#define COREMARK_SEED1 0x0
#endif

#ifndef COREMARK_SEED2
#define COREMARK_SEED2 0x0
#endif

#if VALIDATION_RUN
volatile ee_s32 seed1_volatile = COREMARK_SEED1;
volatile ee_s32 seed2_volatile = COREMARK_SEED2;
volatile ee_s32 seed3_volatile = COREMARK_SEED3;
#endif
#if PERFORMANCE_RUN
volatile ee_s32 seed1_volatile = COREMARK_SEED1;
volatile ee_s32 seed2_volatile = COREMARK_SEED2;
volatile ee_s32 seed3_volatile = COREMARK_SEED3;
#endif
#if PROFILE_RUN
volatile ee_s32 seed1_volatile = COREMARK_SEED1;
volatile ee_s32 seed2_volatile = COREMARK_SEED2;
volatile ee_s32 seed3_volatile = COREMARK_SEED3;
#endif
volatile ee_s32 seed4_volatile = ITERATIONS;
volatile ee_s32 seed5_volatile = COREMARK_EXECS_MASK;

static CORE_TICKS start_time_val, stop_time_val;

void
start_time(void)
{
    start_time_val = 0;
}

void
stop_time(void)
{
    stop_time_val = COREMARK_FAKE_TICKS;
}

CORE_TICKS
get_time(void)
{
    return (CORE_TICKS)(stop_time_val - start_time_val);
}

secs_ret
time_in_secs(CORE_TICKS ticks)
{
#if HAS_FLOAT
    return ((secs_ret)ticks) / ((secs_ret)COREMARK_TICKS_PER_SEC);
#else
    return (secs_ret)(ticks / COREMARK_TICKS_PER_SEC);
#endif
}

ee_u32 default_num_contexts = 1;

void
portable_init(core_portable *p, int *argc, char *argv[])
{
    (void)argc;
    (void)argv;

    if (sizeof(ee_ptr_int) != sizeof(ee_u8 *))
    {
        ee_printf(
            "ERROR! Please define ee_ptr_int to a type that holds a pointer!\n");
    }
    if (sizeof(ee_u32) != 4)
    {
        ee_printf("ERROR! Please define ee_u32 to a 32b unsigned type!\n");
    }
    p->portable_id = 1;
}

void
portable_fini(core_portable *p)
{
    /* Publish completion for cocotb harness consumers. Validation first: the
       harness polls the sentinel, so everything it reads must already be in
       memory by the time the sentinel appears. */
    *WF_VALIDATION_ADDR = wf_validation_state;
    *WF_SEEDCRC_ADDR = wf_seedcrc;
    *WF_DATASIZE_ADDR = wf_data_size;
    *WF_CRCLIST_ADDR = wf_crclist;
    *WF_CRCMATRIX_ADDR = wf_crcmatrix;
    *WF_CRCSTATE_ADDR = wf_crcstate;
    *WF_CRCFINAL_ADDR = wf_crcfinal;
    *WF_RESULT_ADDR = (ee_u32)get_time();
    *WF_SENTINEL_ADDR = WF_SENTINEL_VAL;
    p->portable_id = 0;
}

/* CoreMark reports the outcome of its own CRC check through ee_printf, which
   has to be a no-op here (no console). Discarding it means a run that computes
   entirely wrong results still "passes" as long as it reaches the sentinel, so
   the verdict is recovered from the format string and published to memory
   instead. core_main.c prints exactly one of these three at the end:
   total_errors == 0, > 0, or < 0. */
static int
wf_str_starts_with(const char *s, const char *prefix)
{
    while (*prefix)
    {
        if (*s != *prefix)
        {
            return 0;
        }
        s++;
        prefix++;
    }
    return 1;
}

static int
wf_str_contains(const char *s, const char *needle)
{
    for (; *s != 0; s++)
    {
        if (wf_str_starts_with(s, needle))
        {
            return 1;
        }
    }
    return 0;
}

int
ee_printf(const char *fmt, ...)
{
    va_list ap;

    if (fmt != 0)
    {
        if (wf_str_starts_with(fmt, "Correct operation validated"))
        {
            wf_validation_state = WF_VALIDATION_OK;
        }
        else if (wf_str_starts_with(fmt, "Errors detected"))
        {
            wf_validation_state = WF_VALIDATION_ERRORS;
        }
        else if (wf_str_starts_with(fmt, "Cannot validate operation"))
        {
            wf_validation_state = WF_VALIDATION_UNCHECKABLE;
        }
        else if (wf_str_starts_with(fmt, "seedcrc"))
        {
            /* "seedcrc          : 0x%04x" -- ee_u16 promoted to int. Published
               so a 'no known CRC' verdict can be diagnosed: the seedcrc says
               which (seed1,seed2,seed3,size) tuple was actually built. */
            va_start(ap, fmt);
            wf_seedcrc = (ee_u32)va_arg(ap, unsigned int);
            va_end(ap);
        }
        else if (wf_str_starts_with(fmt, "CoreMark Size"))
        {
            /* "CoreMark Size    : %lu" -- per-algorithm data size. */
            va_start(ap, fmt);
            wf_data_size = (ee_u32)va_arg(ap, unsigned long);
            va_end(ap);
        }
        /* "[%d]crc<name>      : 0x%04x" -- context index first, CRC second.
           Published so a failing run can be compared value by value against a
           reference execution of the same image instead of only reporting a
           verdict. */
        else if (wf_str_contains(fmt, "crclist"))
        {
            va_start(ap, fmt);
            (void)va_arg(ap, int);
            wf_crclist = (ee_u32)va_arg(ap, unsigned int);
            va_end(ap);
        }
        else if (wf_str_contains(fmt, "crcmatrix"))
        {
            va_start(ap, fmt);
            (void)va_arg(ap, int);
            wf_crcmatrix = (ee_u32)va_arg(ap, unsigned int);
            va_end(ap);
        }
        else if (wf_str_contains(fmt, "crcstate"))
        {
            va_start(ap, fmt);
            (void)va_arg(ap, int);
            wf_crcstate = (ee_u32)va_arg(ap, unsigned int);
            va_end(ap);
        }
        else if (wf_str_contains(fmt, "crcfinal"))
        {
            va_start(ap, fmt);
            (void)va_arg(ap, int);
            wf_crcfinal = (ee_u32)va_arg(ap, unsigned int);
            va_end(ap);
        }
    }
    return 0;
}
