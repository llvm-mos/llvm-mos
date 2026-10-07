; RUN: llc -verify-machineinstrs -O2 < %s | FileCheck %s

target datalayout = "e-m:e-p:16:8-p1:8:8-i16:8-i32:8-i64:8-f32:8-f64:8-a:8-Fi8-n8"
target triple = "mos"

; The legalizer turns the NMI's multiply into a call to __mulhi3, redirected to
; the __nmi clone. That clone calls an internal helper, which must be cloned
; into the call tree too, rather than left calling the unsuffixed helper.
@g = global i16 0
@h = global i16 0

define void @nmi() "interrupt-norecurse" "interrupt-rc-suffix"="__nmi" {
  %a = load volatile i16, ptr @g
  %b = load volatile i16, ptr @h
  %m = mul i16 %a, %b
  store volatile i16 %m, ptr @g
  ret void
}

define internal i16 @helper(i16 %a, i16 %b) noinline {
  %x = add i16 %a, %b
  %y = xor i16 %x, 85
  ret i16 %y
}

define i16 @__mulhi3(i16 %a, i16 %b) noinline {
  %r = call i16 @helper(i16 %a, i16 %b)
  ret i16 %r
}

define void @main() {
  %a = load volatile i16, ptr @g
  %r = call i16 @__mulhi3(i16 %a, i16 %a)
  store volatile i16 %r, ptr @h
  ret void
}

; CHECK-LABEL: nmi:
; CHECK: jsr __mulhi3__nmi
; CHECK-LABEL: __mulhi3__nmi:
; CHECK-NEXT: ; %bb.0:
; CHECK-NEXT: jmp helper__nmi
; CHECK-LABEL: helper__nmi:
; CHECK: __rc2__nmi
