; RUN: not opt -passes=mos-call-tree-clone -S %s 2>&1 | FileCheck %s
; RUN: not llc -O2 < %s 2>&1 | FileCheck %s

target datalayout = "e-m:e-p:16:8-p1:8:8-i16:8-i32:8-i64:8-f32:8-f64:8-a:8-Fi8-n8"
target triple = "mos"

; Two handlers sharing a register suffix could preempt one another (e.g. NMI during IRQ)
; and clobber each other's registers and static frames.
@g = global i8 0

define internal void @work() noinline {
  %v = load volatile i8, ptr @g
  %w = add i8 %v, 1
  store volatile i8 %w, ptr @g
  ret void
}

define void @irq() "interrupt-norecurse" "interrupt-rc-suffix"="__nmi" {
  call void @work()
  ret void
}

define void @nmi() "interrupt-norecurse" "interrupt-rc-suffix"="__nmi" {
  call void @work()
  ret void
}

define void @main() {
  ret void
}

; CHECK: error:{{.*}}[MOS interrupt] register set suffix '__nmi' is used by both "irq" and "nmi"; give each handler a distinct suffix
; CHECK-NOT: PLEASE submit
