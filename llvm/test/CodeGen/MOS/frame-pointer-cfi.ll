; RUN: llc -verify-machineinstrs < %s | FileCheck %s

; Test that frame pointer functions emit correct CFI with offset.
; When hasFP() is true (e.g., VLA usage), the CFI should use .cfi_def_cfa
; with an explicit offset, not .cfi_def_cfa_register which loses the offset.

target datalayout = "e-m:e-p:16:8-p1:8:8-i16:8-i32:8-i64:8-f32:8-f64:8-a:8-Fi8-n8"
target triple = "mos"

declare void @use(ptr)

; VLA triggers frame pointer usage (hasFP() returns true)
define void @test_vla(i16 %n) {
; CHECK-LABEL: test_vla:
; CHECK: .cfi_def_cfa
; After setting FP = SP, we should see .cfi_def_cfa with register AND offset
; CHECK-NOT: .cfi_def_cfa_register
  %vla = alloca i16, i16 %n
  call void @use(ptr %vla)
  ret void
}
