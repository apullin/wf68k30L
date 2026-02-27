ENTRY(_start)

MEMORY {
  PROG_RAM (rwx) : ORIGIN = 0x00000100, LENGTH = 0x0001FF00
}

SECTIONS {
  .text : {
    *(.text*)
    *(.rodata*)
  } > PROG_RAM

  .data : {
    *(.data*)
  } > PROG_RAM

  .bss (NOLOAD) : {
    *(.bss*)
    *(COMMON)
  } > PROG_RAM
}
