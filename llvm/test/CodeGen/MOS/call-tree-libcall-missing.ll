; RUN: not llc -O2 < %s 2>&1 | FileCheck %s
; RUN: not llc -O0 < %s 2>&1 | FileCheck %s
; RUN: not llc -O2 -mtriple=mos --filetype=obj -o /dev/null < %s 2>&1 | FileCheck %s

target datalayout = "e-m:e-p:16:8-p1:8:8-i16:8-i32:8-i64:8-f32:8-f64:8-a:8-Fi8-n8"
target triple = "mos"

; The NMI compares doubles, both directly and through the speculatively cloned
; fmin. __ltdf2 has no bitcode definition to clone, so it can't run in the
; __nmi call tree. This is a clean error, not a crash.
define double @fmin(double %a, double %b) noinline {
  %c = fcmp olt double %a, %b
  %r = select i1 %c, double %a, double %b
  ret double %r
}

@d = global double 0.0
@g = global i8 0

define void @nmi() "interrupt-norecurse" "interrupt-rc-suffix"="__nmi" {
  %a = load volatile double, ptr @d
  %b = load volatile double, ptr @d
  %c = fcmp olt double %a, %b
  %z = zext i1 %c to i8
  store volatile i8 %z, ptr @g
  %m = call double @fmin(double %a, double %b)
  store volatile double %m, ptr @d
  ret void
}

define void @main() {
  ret void
}

; CHECK: error:{{.*}}[MOS interrupt] libcall '__ltdf2' required by '{{nmi|fmin__nmi}}' has no bitcode definition
; CHECK-NOT: PLEASE submit
