; RUN: llc -verify-machineinstrs -filetype=obj < %s -o %t.o
; RUN: llvm-dwarfdump --eh-frame %t.o | FileCheck %s \
; RUN:   --implicit-check-not=DW_CFA_def_cfa: \
; RUN:   --implicit-check-not=DW_CFA_def_cfa_register \
; RUN:   --implicit-check-not=DW_CFA_def_cfa_offset

; With a frame pointer, RS0 moves in the body (here, to allocate the VLA). The
; CFA stays hardware-stack based; instead, once the frame pointer (RS15) is set,
; the caller's RS0 is described relative to it. The epilogue copies the frame
; pointer back into RS0 before restoring the callee-saved registers, which may
; include the frame pointer itself, so the rule moves back to RS0 there. The
; frame has no fixed size, so the caller's RS0 is the base itself.

target datalayout = "e-m:e-p:16:8-p1:8:8-i16:8-i32:8-i64:8-f32:8-f64:8-a:8-Fi8-n8"
target triple = "mos"

declare void @use(ptr)

define void @test_vla(i16 %n) {
; CHECK:      FDE
; CHECK:      DW_CFA_val_expression: RS0 DW_OP_bregx RS15+0
; CHECK:      DW_CFA_val_expression: RS0 DW_OP_bregx RS0+0
; CHECK:      DW_CFA_restore_extended: RS0
  %vla = alloca i16, i16 %n
  call void @use(ptr %vla)
  ret void
}
