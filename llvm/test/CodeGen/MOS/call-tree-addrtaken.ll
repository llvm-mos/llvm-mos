; RUN: opt -passes=mos-call-tree-clone -S %s | FileCheck %s --check-prefix=IR
; RUN: llc -verify-machineinstrs -O2 < %s | FileCheck %s

target datalayout = "e-m:e-p:16:8-p1:8:8-i16:8-i32:8-i64:8-f32:8-f64:8-a:8-Fi8-n8"
target triple = "mos"

; A function reachable from a suffixed ISR has its address taken in mainline
; code. That's fine: the ISR calls the clone directly, and the pointer keeps
; referring to the unsuffixed original. (Indirect calls within the ISR tree are
; rejected separately.)
define void @isr_addrtaken() "interrupt-norecurse" "interrupt-rc-suffix"="__nmi" {
entry:
  call void @taken_helper()
  ret void
}

@g = global i8 0

define void @taken_helper() noinline {
entry:
  store volatile i8 1, ptr @g
  ret void
}

@taken_fp = global ptr @taken_helper

define void @main() {
entry:
  %f = load volatile ptr, ptr @taken_fp
  call void %f()
  ret void
}

; IR: @taken_fp = global ptr @taken_helper
; IR-LABEL: define void @isr_addrtaken()
; IR: call void @taken_helper__nmi()

; CHECK-LABEL: isr_addrtaken:
; CHECK: jsr taken_helper__nmi
; CHECK-LABEL: taken_helper__nmi:
; CHECK: taken_fp:
; CHECK-NEXT: .short taken_helper
