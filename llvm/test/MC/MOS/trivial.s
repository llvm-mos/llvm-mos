; RUN: llvm-mc -triple mos < %s | FileCheck %s
; CHECK-NOT: __do_init_stack
; CHECK-NOT: __do_zero_bss
; CHECK-NOT: __do_init_array
; CHECK-NOT: __do_fini_array
