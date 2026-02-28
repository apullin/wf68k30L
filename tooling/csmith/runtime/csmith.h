#ifndef WF68K30L_CSMITH_SHIM_H
#define WF68K30L_CSMITH_SHIM_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Minimal csmith runtime shim for bare-metal smoke generation. */
static inline void platform_main_begin(void)
{
}

static inline void crc32_gentab(void)
{
}

static inline void transparent_crc(uint64_t value, char *name, int flag)
{
    (void)value;
    (void)name;
    (void)flag;
}

static inline void transparent_crc_bytes(char *ptr, int nbytes, char *name, int flag)
{
    (void)ptr;
    (void)nbytes;
    (void)name;
    (void)flag;
}

static inline void platform_main_end(uint64_t crc, int flag)
{
    (void)crc;
    (void)flag;
}

#endif
