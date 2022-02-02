; RUN: llvm-mc -triple mos < %s | FileCheck %s
.section .fini_array.10
; CHECK: Declaring this symbol
; CHECK: __do_fini_array
