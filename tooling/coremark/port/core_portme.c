/*
CoreMark port layer for WF68K30L bare-metal simulation.
*/
#include "coremark.h"
#include "core_portme.h"

#define WF_RESULT_ADDR   ((volatile ee_u32 *)0x00020000u)
#define WF_SENTINEL_ADDR ((volatile ee_u32 *)0x00030000u)
#define WF_SENTINEL_VAL  ((ee_u32)0xDEADCAFEu)

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
    /* Publish completion for cocotb harness consumers. */
    *WF_RESULT_ADDR = (ee_u32)get_time();
    *WF_SENTINEL_ADDR = WF_SENTINEL_VAL;
    p->portable_id = 0;
}

int
ee_printf(const char *fmt, ...)
{
    (void)fmt;
    return 0;
}
