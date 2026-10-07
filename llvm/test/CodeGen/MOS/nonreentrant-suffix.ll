; RUN: opt -passes=mos-call-tree-clone,mos-nonreentrant -S %s | FileCheck %s

target datalayout = "e-p:16:8:8-p1:8:8-i16:8:8-i32:8:8-i64:8:8-f32:8:8-f64:8:8-a:8:8-Fi8-n8"
target triple = "mos"

; Only suffixed interrupts exist, so the unsuffixed libcalls can't be called by
; an interrupt and stay nonreentrant. The per-suffix libcall clones have no IR
; callers, but are still analyzed: each suffix has a single norecurse root, so
; they are nonreentrant too, as is the helper clone they call.

define void @suffixed_norecurse() "interrupt-norecurse" "interrupt-rc-suffix"="__nmi" {
entry:
  ret void
}

define internal i8 @helper(i8 %a) noinline {
  ret i8 %a
}

define i8 @__udivqi3(i8 %a, i8 %b) {
  %r = call i8 @helper(i8 %a)
  ret i8 %r
}

define void @main() {
entry:
  ret void
}

; CHECK: define i8 @__udivqi3(i8 %a, i8 %b) #[[MAIN:[0-9]+]]
; CHECK: define internal i8 @__udivqi3__nmi(i8 %a, i8 %b) #[[CLONE:[0-9]+]]
; CHECK-NEXT: call i8 @helper__nmi(
; CHECK: define internal i8 @helper__nmi(i8 %a) #[[HELPER:[0-9]+]]

; CHECK-DAG: attributes #[[MAIN]] = { norecurse "nonreentrant" }
; CHECK-DAG: attributes #[[CLONE]] = { norecurse "mos-call-tree-libcall-clone" "nonreentrant" "rc-suffix"="__nmi" }
; CHECK-DAG: attributes #[[HELPER]] = { noinline norecurse "nonreentrant" "rc-suffix"="__nmi" }
