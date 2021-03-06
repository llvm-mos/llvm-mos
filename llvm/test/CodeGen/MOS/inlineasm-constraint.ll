; RUN: llc -mtriple=mos -verify-machineinstrs < %s | FileCheck %s

; CHECK-LABEL: inline__constraint__a:
define void @inline_constraint_a() {
  call void asm sideeffect "JSR 1234", "a"(i8 56)
  ; CHECK: LDA #56
  ; CHECK: JSR 1234
  ret void
}

; CHECK-LABEL: inline__constraint__x:
define void @inline_constraint_x() {
  call void asm sideeffect "JSR 1234", "x"(i8 56)
  ; CHECK: LDX #56
  ; CHECK: JSR 1234
  ret void
}

; CHECK-LABEL: inline__constraint__y:
define void @inline_constraint_y() {
  call void asm sideeffect "JSR 1234", "y"(i8 56)
  ; CHECK: LDY #56
  ; CHECK: JSR 1234
  ret void
}