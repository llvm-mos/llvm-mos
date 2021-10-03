; RUN: llc -verify-machineinstrs -o - %s | FileCheck %s -check-prefixes=CHECK,6502
; RUN: llc -mcpu=mos65c02 -verify-machineinstrs -o - %s | FileCheck %s -check-prefixes=CHECK,65C02
target datalayout = "e-p:16:8:8-i16:8:8-i32:8:8-i64:8:8-f32:8:8-f64:8:8-a:8:8-Fi8-n8"
target triple = "mos"

@chars = dso_local local_unnamed_addr global [2 x i8] c"**", align 1

define void @main() {
; CHECK-LABEL: main:
; 6502: lda #0
; 6502-NEXT: sta chars
; 6502-NEXT: rts
; 65C02: stz chars
; 65C02-NEXT: rts
entry:
  %arrayidx = getelementptr inbounds [2 x i8], [2 x i8]* @chars, i16 0, i16 0
  store i8 0, i8* %arrayidx, align 1
  ret void
}

define void @main_idx(i8 %idx) {
; CHECK-LABEL: main_idx:
; 6502: tax
; 6502-NEXT: lda #0
; 6502-NEXT: sta chars,x
; 6502-NEXT: rts
; 65C02: tax
; 65C02-NEXT: stz chars,x
; 65C02-NEXT: rts
entry:
  %idxprom = zext i8 %idx to i16
  %arrayidx = getelementptr inbounds [2 x i8], [2 x i8]* @chars, i16 0, i16 %idxprom
  store i8 0, i8* %arrayidx, align 1
  ret void
}

; CHECK-LABEL: chars:
; CHECK-NEXT: .zero 2,42
