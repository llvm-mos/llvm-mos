; RUN: llvm-mc -triple mos < %s | FileCheck %s
.section .init_array.10
; CHECK: Declaring this symbol
; CHECK: __do_init_array
