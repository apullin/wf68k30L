#ifndef WF68K30L_CSMITH_SHIM_H
#define WF68K30L_CSMITH_SHIM_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*
 * Bare-metal csmith runtime for the WF68K30L smoke flow.
 *
 * The checksum is the whole point of a csmith run: without it a program that
 * computes entirely wrong values still "passes" as long as it terminates. So
 * these are the real CRC32 routines from csmith's own runtime header
 * (csmith-2.3.0/csmith.h), byte for byte in behaviour, rather than the empty
 * stubs this file used to hold. What is different is the reporting: there is no
 * console, so platform_main_end publishes the checksum to memory where the
 * cocotb harness and a QEMU reference run can both read it.
 *
 * Memory contract (shared with tb/test_csmith_smoke.py):
 *   0x00020000  computed checksum (32-bit)
 *   0x00020004  WF_CSMITH_CRC_MAGIC once the checksum has been published
 *   0x00030000  sentinel, written last
 */

#define WF_CRC_ADDR      ((volatile uint32_t *)0x00020000u)
#define WF_CRC_FLAG_ADDR ((volatile uint32_t *)0x00020004u)
#define WF_CRC_MAGIC     ((uint32_t)0xC50FC50Fu)
#define WF_SENTINEL_ADDR ((volatile uint32_t *)0x00030000u)
#define WF_SENTINEL_VAL  ((uint32_t)0xDEADCAFEu)

/* Declared, never reached: csmith emits printf calls under
   `if (print_hash_value)`, which is a compile-time zero here, so the calls are
   removed before link. The declaration keeps the generated file compiling
   without pulling in a hosted <stdio.h>. */
int printf(const char *fmt, ...);

static uint32_t crc32_tab[256];
static uint32_t crc32_context = 0xFFFFFFFFUL;

static void
crc32_gentab(void)
{
    uint32_t crc;
    const uint32_t poly = 0xEDB88320UL;
    int i, j;

    for (i = 0; i < 256; i++)
    {
        crc = i;
        for (j = 8; j > 0; j--)
        {
            if (crc & 1)
            {
                crc = (crc >> 1) ^ poly;
            }
            else
            {
                crc >>= 1;
            }
        }
        crc32_tab[i] = crc;
    }
}

static void
crc32_byte(uint8_t b)
{
    crc32_context = ((crc32_context >> 8) & 0x00FFFFFF)
                    ^ crc32_tab[(crc32_context ^ b) & 0xFF];
}

static void
crc32_8bytes(uint64_t val)
{
    crc32_byte((val >> 0) & 0xff);
    crc32_byte((val >> 8) & 0xff);
    crc32_byte((val >> 16) & 0xff);
    crc32_byte((val >> 24) & 0xff);
    crc32_byte((val >> 32) & 0xff);
    crc32_byte((val >> 40) & 0xff);
    crc32_byte((val >> 48) & 0xff);
    crc32_byte((val >> 56) & 0xff);
}

static void
transparent_crc(uint64_t val, char *vname, int flag)
{
    crc32_8bytes(val);
    (void)vname;
    (void)flag;
}

static void
transparent_crc_bytes(char *ptr, int nbytes, char *vname, int flag)
{
    int i;
    for (i = 0; i < nbytes; i++)
    {
        crc32_byte(ptr[i]);
    }
    (void)vname;
    (void)flag;
}

static void
platform_main_begin(void)
{
}

static void
platform_main_end(uint64_t crc, int flag)
{
    (void)flag;
    *WF_CRC_ADDR = (uint32_t)crc;
    *WF_CRC_FLAG_ADDR = WF_CRC_MAGIC;
    *WF_SENTINEL_ADDR = WF_SENTINEL_VAL;
}

#endif
