; RUN: llc -verify-machineinstrs < %s | FileCheck %s
;
; The C three-way-compare ("spaceship") idiom `(a > b) - (a < b)` is
; canonicalized by clang/instcombine to the generic llvm.scmp/llvm.ucmp
; intrinsics (G_SCMP/G_UCMP in GlobalISel). The MOS legalizer had no rule for
; them, so any program qsort-ing with such a comparator aborted with
; "unable to legalize instruction: ... = G_SCMP ...". These now lower via
; LegalizerHelper::lowerThreewayCompare (icmp + select expansion the backend
; already legalizes). The checks pin each function's successful lowering to a
; return and that the expansion stays inline (CHECK-NOT: jsr) — a regression
; to either an abort or a libcall fails; the exact expansion is icmp/select
; codegen owned elsewhere.

target datalayout = "e-m:e-p:16:8-p1:8:8-i16:8-i32:8-i64:8-f32:8-f64:8-a:8-Fi8-n8"
target triple = "mos"

; CHECK-LABEL: scmp_i8:
; CHECK-NOT: jsr
; CHECK: rts
define i8 @scmp_i8(i8 %a, i8 %b) {
  %r = call i8 @llvm.scmp.i8.i8(i8 %a, i8 %b)
  ret i8 %r
}

; CHECK-LABEL: ucmp_i8:
; CHECK-NOT: jsr
; CHECK: rts
define i8 @ucmp_i8(i8 %a, i8 %b) {
  %r = call i8 @llvm.ucmp.i8.i8(i8 %a, i8 %b)
  ret i8 %r
}

; CHECK-LABEL: scmp_i16:
; CHECK-NOT: jsr
; CHECK: rts
define i16 @scmp_i16(i16 %a, i16 %b) {
  %r = call i16 @llvm.scmp.i16.i16(i16 %a, i16 %b)
  ret i16 %r
}

; CHECK-LABEL: ucmp_i16:
; CHECK-NOT: jsr
; CHECK: rts
define i16 @ucmp_i16(i16 %a, i16 %b) {
  %r = call i16 @llvm.ucmp.i16.i16(i16 %a, i16 %b)
  ret i16 %r
}

; CHECK-LABEL: scmp_i16_i32:
; CHECK-NOT: jsr
; CHECK: rts
define i16 @scmp_i16_i32(i32 %a, i32 %b) {
  %r = call i16 @llvm.scmp.i16.i32(i32 %a, i32 %b)
  ret i16 %r
}

; CHECK-LABEL: ucmp_i16_i64:
; CHECK-NOT: jsr
; CHECK: rts
define i16 @ucmp_i16_i64(i64 %a, i64 %b) {
  %r = call i16 @llvm.ucmp.i16.i64(i64 %a, i64 %b)
  ret i16 %r
}

; CHECK-LABEL: scmp_i32_i16:
; CHECK-NOT: jsr
; CHECK: rts
define i32 @scmp_i32_i16(i16 %a, i16 %b) {
  %r = call i32 @llvm.scmp.i32.i16(i16 %a, i16 %b)
  ret i32 %r
}
