; RUN: llvm-mc -triple mos -motorola-integers < %s | FileCheck %s
; CHECK-NOT: __do_init_stack
; CHECK-NOT: __do_zero_bss
; CHECK-NOT: __do_zero_zp_bss
; CHECK-NOT: __do_init_array
; CHECK-NOT: __do_fini_array
; CHECK-NOT: __do_copy_data
; CHECK-NOT: __do_copy_zp_data
