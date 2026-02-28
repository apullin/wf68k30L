.section .text
.global _start

_start:
    jsr main
1:
    bra 1b
