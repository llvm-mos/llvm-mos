; RUN: llc -mtriple=mos -verify-machineinstrs < %s | FileCheck %s

; CHECK-LABEL: inline_constraint_a:
define void @inline_constraint_a() {
  call void asm sideeffect "JSR 1234", "a"(i8 56)
  ; CHECK: lda #56
  ; CHECK: jsr 1234
  ret void
}

; CHECK-LABEL: inline_constraint_x:
define void @inline_constraint_x() {
  call void asm sideeffect "JSR 1234", "x"(i8 56)
  ; CHECK: ldx #56
  ; CHECK: jsr 1234
  ret void
}

; CHECK-LABEL: inline_constraint_y:
define void @inline_constraint_y() {
  call void asm sideeffect "JSR 1234", "y"(i8 56)
  ; CHECK: ldy #56
  ; CHECK: jsr 1234
  ret void
}
