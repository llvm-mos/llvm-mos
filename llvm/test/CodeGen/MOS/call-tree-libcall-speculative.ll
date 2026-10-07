; RUN: llc -verify-machineinstrs -O2 < %s | FileCheck %s
; RUN: llc -verify-machineinstrs -O0 < %s | FileCheck %s

target datalayout = "e-m:e-p:16:8-p1:8:8-i16:8-i32:8-i64:8-f32:8-f64:8-a:8-Fi8-n8"
target triple = "mos"

; fmin is a runtime libcall with a definition, so it is speculatively cloned
; into the __nmi call tree. Its body needs __ltdf2, which has no definition. The NMI
; never uses fmin, so the clone is discarded and no error is reported.
define double @fmin(double %a, double %b) noinline {
  %c = fcmp olt double %a, %b
  %r = select i1 %c, double %a, double %b
  ret double %r
}

@g = global i8 0
@d = global double 0.0

define void @nmi() "interrupt-norecurse" "interrupt-rc-suffix"="__nmi" {
  store volatile i8 1, ptr @g
  ret void
}

define void @main() {
  %a = load volatile double, ptr @d
  %r = call double @fmin(double %a, double %a)
  store volatile double %r, ptr @d
  ret void
}

; CHECK-LABEL: nmi:
; CHECK-NOT: fmin__nmi
; CHECK-NOT: __ltdf2__nmi
