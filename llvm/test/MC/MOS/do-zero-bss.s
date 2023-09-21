; RUN: llvm-mc -triple mos -motorola-integers < %s | FileCheck %s
.section .bss.foo
  .fill 1
; CHECK: Declaring this symbol
; CHECK: __do_zero_bss

.section .zp.bss.foo
  .fill 1
; CHECK: Declaring this symbol
; CHECK: __do_zero_zp_bss
