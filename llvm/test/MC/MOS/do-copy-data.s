; RUN: llvm-mc -triple mos -motorola-integers < %s | FileCheck %s
.data
.fill 1

.section .zp.data,"a"
.fill 1

; CHECK: Declaring this symbol
; CHECK: __do_copy_data

; CHECK: Declaring this symbol
; CHECK: __do_copy_zp_data
