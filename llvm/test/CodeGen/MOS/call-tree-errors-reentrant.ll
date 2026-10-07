; RUN: not opt -passes=mos-call-tree-clone -S %s 2>&1 | FileCheck %s
; RUN: not llc -O2 < %s 2>&1 | FileCheck %s

target datalayout = "e-m:e-p:16:8-p1:8:8-i16:8-i32:8-i64:8-f32:8-f64:8-a:8-Fi8-n8"
target triple = "mos"

; A private register set requires a norecurse handler. The front-end rejects
; interrupt("suffix"); hand-written IR is rejected by the backend.
define void @irq_handler() "interrupt" "interrupt-rc-suffix"="__irq" {
entry:
  ret void
}

define void @main() {
entry:
  ret void
}

; CHECK: error:{{.*}}[MOS interrupt] a private register set requires "interrupt-norecurse"
; CHECK-SAME: (function "irq_handler")
; CHECK-NOT: PLEASE submit
