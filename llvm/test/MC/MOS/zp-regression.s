; RUN: llvm-mc -triple mos -motorola-integers --filetype=obj -o=%t %s
; RUN: llvm-objdump --all-headers --print-imm-hex -D %t | FileCheck %s


.section .zp, "zax", @nobits
temp: .byte 0
.text
; CHECK: R_MOS_ADDR8
sta temp
; CHECK: R_MOS_ADDR8
sta temp+0
