; RUN: llvm-mc -triple mos < %s | FileCheck %s
lda mos8(__rc0)
; CHECK: Declaring this symbol
; CHECK: __do_init_stack
