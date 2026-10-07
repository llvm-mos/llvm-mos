; RUN: opt -passes=mos-call-tree-clone -S %s | FileCheck %s --check-prefix=IR
; RUN: llc -verify-machineinstrs -O2 < %s | FileCheck %s

target datalayout = "e-m:e-p:16:8-p1:8:8-i16:8-i32:8-i64:8-f32:8-f64:8-a:8-Fi8-n8"
target triple = "mos"

; A call through an alias still has a known target: the aliasee is cloned into
; the call tree rather than the call being rejected as indirect.
@g = global i8 0

define void @f() noinline {
  store volatile i8 1, ptr @g
  ret void
}

@a = alias void (), ptr @f

define void @nmi() "interrupt-norecurse" "interrupt-rc-suffix"="__nmi" {
  call void @a()
  ret void
}

define void @main() {
  call void @a()
  ret void
}

; IR-LABEL: define void @nmi()
; IR-NEXT: call void @f__nmi()
; IR-LABEL: define void @main()
; IR-NEXT: call void @a()
; IR: define internal void @f__nmi()

; CHECK-LABEL: nmi:
; CHECK: jsr f__nmi
; CHECK-LABEL: f__nmi:
