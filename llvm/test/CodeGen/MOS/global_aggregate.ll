; RUN: llc -verify-machineinstrs -o - %s | FileCheck %s
target datalayout = "e-p:16:8:8-i16:8:8-i32:8:8-i64:8:8-f32:8:8-f64:8:8-a:8:8-Fi8-n8"
target triple = "mos"

@agg = private unnamed_addr global [2 x i8] c"\00\00", align 1

define i8 @main() {
; CHECK-LABEL: main:
entry:
  %0 = getelementptr [2 x i8], [2 x i8]* @agg, i16 0, i16 0
  %1 = getelementptr [2 x i8], [2 x i8]* @agg, i16 0, i16 1
; CHECK:      lda #mos16lo(agg)
; CHECK-NEXT: ldx #mos16hi(agg)
  store i8 10, i8* %0, align 1
; CHECK:      ldy	#0
; CHECK:      sta	(mos8(__rc2)),y
  store i8 20, i8* %1, align 1
; CHECK:      ldy	#1
; CHECK:      sta	(mos8(__rc2)),y
  %2 = load i8, i8* %0, align 1
; CHECK:      ldy	#0
; CHECK:      lda	(mos8(__rc2)),y
  ret i8 %2
; CHECK: rts
}
