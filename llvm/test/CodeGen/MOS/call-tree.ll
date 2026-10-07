; RUN: opt -passes=mos-call-tree-clone -S %s | FileCheck %s

target datalayout = "e-p:16:8:8-p1:8:8-i16:8:8-i32:8:8-i64:8:8-f32:8:8-f64:8:8-a:8:8-Fi8-n8"
target triple = "mos"

; Root ISR with suffix "__nmi".
define void @nmi_handler() "interrupt-norecurse" "interrupt-rc-suffix"="__nmi" {
entry:
  call void @shared_helper()
  ret void
}

; Helper shared between ISR tree and mainline.
define void @shared_helper() {
entry:
  ret void
}

define void @main() {
entry:
  call void @shared_helper()
  ret void
}

; The root should call the clone, not the original.
; CHECK: call void @shared_helper__nmi()

; Original is unchanged and still called from mainline.
; CHECK: define void @shared_helper() {
; CHECK: call void @shared_helper()

; Clone is created as internal.
; CHECK: define internal void @shared_helper__nmi()

; Root gets "rc-suffix" stamped; clone has "rc-suffix".
; CHECK: attributes {{.*}} = { "interrupt-norecurse" "interrupt-rc-suffix"="__nmi" "rc-suffix"="__nmi" }
; CHECK: attributes {{.*}} = { "rc-suffix"="__nmi" }
