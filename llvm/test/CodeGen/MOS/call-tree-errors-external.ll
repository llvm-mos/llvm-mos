; RUN: not opt -passes=mos-call-tree-clone -disable-output %s 2>&1 | FileCheck %s
; RUN: not llc -O2 -o /dev/null < %s 2>&1 | FileCheck %s

target datalayout = "e-p:16:8:8-p1:8:8-i16:8:8-i32:8:8-i64:8:8-f32:8:8-f64:8:8-a:8:8-Fi8-n8"
target triple = "mos"

; Call to an external declaration that is not a known runtime libcall or marked
; interrupt-safe. The clone pass cannot clone a declaration, so this is a hard
; error, reported once.
declare void @user_external()

define void @isr_external() "interrupt-norecurse" "interrupt-rc-suffix"="__nmi" {
entry:
  call void @user_external()
  ret void
}

define void @main() {
entry:
  ret void
}

; CHECK: [MOS interrupt] call to external function 'user_external'
; CHECK-SAME: not available as bitcode (defined in assembly or a non-LTO object); mark it interrupt_safe if it only uses A/X/Y
; CHECK-NOT: user_external
