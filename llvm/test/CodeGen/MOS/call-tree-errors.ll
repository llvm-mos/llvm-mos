; RUN: not opt -passes=mos-call-tree-clone -S %s 2>&1 | FileCheck %s

target datalayout = "e-p:16:8:8-p1:8:8-i16:8:8-i32:8:8-i64:8:8-f32:8:8-f64:8:8-a:8:8-Fi8-n8"
target triple = "mos"

; Indirect call (function pointer) in a suffixed ISR tree. The callee is
; unknown, so it can't be redirected to its clone; this is a hard error in v1.
define void @isr_indirect() "interrupt-norecurse" "interrupt-rc-suffix"="__nmi" {
entry:
  %f = load ptr, ptr @fp
  call void %f()
  ret void
}

@fp = global ptr null

; CHECK: [MOS interrupt] indirect call
; CHECK: function pointer
