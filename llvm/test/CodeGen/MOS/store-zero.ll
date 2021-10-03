; RUN: llc -verify-machineinstrs -o - %s | FileCheck %s -check-prefixes=CHECK,6502
; RUN: llc -mcpu=mos65c02 -verify-machineinstrs -o - %s | FileCheck %s -check-prefixes=CHECK,65C02
target datalayout = "e-p:16:8:8-i16:8:8-i32:8:8-i64:8:8-f32:8:8-f64:8:8-a:8:8-Fi8-n8"
target triple = "mos"

@a.b_c = private unnamed_addr constant [1 x i8] c"H"

define void @main() {
; CHECK-LABEL: main:
; CHECK: %bb.0: ; %entry
; 6502-NEXT: lda #0
; 6502-NEXT: sta a.b_c
; 6502-NEXT: rts
; 65C02-NEXT: stz a.b_c
; 65C02-NEXT: rts
entry:
  %0 = getelementptr [1 x i8], [1 x i8]* @a.b_c, i16 0, i16 0
  store i8 0, i8* %0, align 1
  ret void
}

; CHECK-LABEL: a.b_c:
; CHECK-NEXT: .byte 72
