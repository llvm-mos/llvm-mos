; RUN: llvm-mc -triple mos -mcpu=mosw65816 -motorola-integers --filetype=obj -o=%t.obj %s
; RUN: llvm-objdump --all-headers --print-imm-hex -D %t.obj | FileCheck %s

; Test all the 65816-specific (24-bit) modifiers for the MOS assembler.

val8 = 0x01
val16 = 0x0202
val24 = 0x030303

. = 0x01
addr8:
.ds.b 0

. = 0x0202
addr16:
.ds.b 0
.ds.b 0

. = 0x0303
addr24:
.ds.b 0
.ds.b 0
.ds.b 0

_start:
    lda mos24(addr8)            ; CHECK: af 00 00 00
                                ; CHECK: R_MOS_ADDR24		.text+0x1
    lda mos24(addr16)           ; CHECK: af 00 00 00
                                ; CHECK: R_MOS_ADDR24		.text+0x202
    lda mos24(addr24)           ; CHECK: af 00 00 00
                                ; CHECK: R_MOS_ADDR24		.text+0x303
