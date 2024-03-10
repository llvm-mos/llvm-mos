; RUN: llvm-mc -triple mos -mcpu=mosspc700 -motorola-integers --filetype=obj -o=%t.obj %s
; RUN: llvm-objdump --all-headers --print-imm-hex -D %t.obj | FileCheck %s

. = 0x01
.zeropage addr8
addr8:
.ds.b 0

. = 0x0202
addr16:
.ds.b 0

_start:
    mov a, !addr8               ; CHECK: e5 00 00
                                ; CHECK: R_MOS_ADDR16		.text+0x1
    mov a, !addr8+x             ; CHECK: f5 00 00
                                ; CHECK: R_MOS_ADDR16		.text+0x1
    mov a, !addr8+y             ; CHECK: f6 00 00
                                ; CHECK: R_MOS_ADDR16		.text+0x1
    mov !addr8, a               ; CHECK: c5 00 00
                                ; CHECK: R_MOS_ADDR16		.text+0x1
    mov !addr8+x, a             ; CHECK: d5 00 00
                                ; CHECK: R_MOS_ADDR16		.text+0x1
    mov !addr8+y, a             ; CHECK: d6 00 00
                                ; CHECK: R_MOS_ADDR16		.text+0x1
    mov a, !addr16              ; CHECK: e5 00 00
                                ; CHECK: R_MOS_ADDR16		.text+0x202
    mov a, !addr16+x            ; CHECK: f5 00 00
                                ; CHECK: R_MOS_ADDR16		.text+0x202
    mov a, !addr16+y            ; CHECK: f6 00 00
                                ; CHECK: R_MOS_ADDR16		.text+0x202
    mov !addr16, a              ; CHECK: c5 00 00
                                ; CHECK: R_MOS_ADDR16		.text+0x202
    mov !addr16+x, a            ; CHECK: d5 00 00
                                ; CHECK: R_MOS_ADDR16		.text+0x202
    mov !addr16+y, a            ; CHECK: d6 00 00
                                ; CHECK: R_MOS_ADDR16		.text+0x202
