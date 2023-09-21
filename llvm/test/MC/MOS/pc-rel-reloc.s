; RUN: llvm-mc -triple mos -motorola-integers --filetype=obj -o=%t.o %s
; RUN: llvm-objdump -r %t.o | FileCheck %s

; CHECK: R_MOS_PCREL_8 sym
bpl sym
