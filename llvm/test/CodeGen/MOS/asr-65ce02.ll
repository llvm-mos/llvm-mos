; RUN: llc -mcpu=mos65ce02 -verify-machineinstrs < %s | FileCheck %s
; RUN: llc -mcpu=mos45gs02 -verify-machineinstrs < %s | FileCheck %s
; RUN: llc -mcpu=mos6502 -verify-machineinstrs < %s | FileCheck %s --check-prefix=NOASR

target datalayout = "e-m:e-p:16:8-p1:8:8-i16:8-i32:8-i64:8-f32:8-f64:8-a:8-Fi8-n8"
target triple = "mos"

define i8 @ashr_i8_1(i8 %a) {
; CHECK-LABEL: ashr_i8_1:
; CHECK:       ; %bb.0: ; %entry
; CHECK-NEXT:    asr
; CHECK-NEXT:    rts
;
; NOASR-LABEL: ashr_i8_1:
; NOASR:       ; %bb.0: ; %entry
; NOASR-NEXT:    cmp #128
; NOASR-NEXT:    ror
; NOASR-NEXT:    rts
entry:
  %0 = ashr i8 %a, 1
  ret i8 %0
}

define i8 @ashr_i8_2(i8 %a) {
; CHECK-LABEL: ashr_i8_2:
; CHECK:       ; %bb.0: ; %entry
; CHECK-NEXT:    asr
; CHECK-NEXT:    asr
; CHECK-NEXT:    rts
;
; NOASR-LABEL: ashr_i8_2:
; NOASR:       ; %bb.0: ; %entry
; NOASR-NEXT:    cmp #128
; NOASR-NEXT:    ror
; NOASR-NEXT:    cmp #128
; NOASR-NEXT:    ror
; NOASR-NEXT:    rts
entry:
  %0 = ashr i8 %a, 2
  ret i8 %0
}

; Only the highest byte needs sign preservation; lower bytes propagate carry.
define i16 @ashr_i16_1(i16 %a) {
; CHECK-LABEL: ashr_i16_1:
; CHECK:       ; %bb.0: ; %entry
; CHECK-NEXT:    stx __rc2
; CHECK-NEXT:    asr __rc2
; CHECK-NEXT:    ror
; CHECK-NEXT:    ldx __rc2
; CHECK-NEXT:    rts
;
; NOASR-LABEL: ashr_i16_1:
; NOASR:       ; %bb.0: ; %entry
; NOASR-NEXT:    sta __rc2
; NOASR-NEXT:    txa
; NOASR-NEXT:    cpx #128
; NOASR-NEXT:    ror
; NOASR-NEXT:    ror __rc2
; NOASR-NEXT:    tax
; NOASR-NEXT:    lda __rc2
; NOASR-NEXT:    rts
entry:
  %0 = ashr i16 %a, 1
  ret i16 %0
}

define i16 @ashr_i16_2(i16 %a) {
; CHECK-LABEL: ashr_i16_2:
; CHECK:       ; %bb.0: ; %entry
; CHECK-NEXT:    stx __rc2
; CHECK-NEXT:    asr __rc2
; CHECK-NEXT:    ror
; CHECK-NEXT:    asr __rc2
; CHECK-NEXT:    ror
; CHECK-NEXT:    ldx __rc2
; CHECK-NEXT:    rts
;
; NOASR-LABEL: ashr_i16_2:
; NOASR:       ; %bb.0: ; %entry
; NOASR-NEXT:    sta __rc2
; NOASR-NEXT:    txa
; NOASR-NEXT:    cpx #128
; NOASR-NEXT:    ror
; NOASR-NEXT:    ror __rc2
; NOASR-NEXT:    cmp #128
; NOASR-NEXT:    ror
; NOASR-NEXT:    ror __rc2
; NOASR-NEXT:    tax
; NOASR-NEXT:    lda __rc2
; NOASR-NEXT:    rts
entry:
  %0 = ashr i16 %a, 2
  ret i16 %0
}

define i32 @ashr_i32_1(i32 %a) {
; CHECK-LABEL: ashr_i32_1:
; CHECK:       ; %bb.0: ; %entry
; CHECK-NEXT:    stx __rc4
; CHECK-NEXT:    asr __rc3
; CHECK-NEXT:    ror __rc2
; CHECK-NEXT:    ror __rc4
; CHECK-NEXT:    ror
; CHECK-NEXT:    ldx __rc4
; CHECK-NEXT:    rts
;
; NOASR-LABEL: ashr_i32_1:
; NOASR:       ; %bb.0: ; %entry
; NOASR-NEXT:    sta __rc4
; NOASR-NEXT:    stx __rc5
; NOASR-NEXT:    lda __rc3
; NOASR-NEXT:    cmp #128
; NOASR-NEXT:    ror
; NOASR-NEXT:    ror __rc2
; NOASR-NEXT:    ror __rc5
; NOASR-NEXT:    ror __rc4
; NOASR-NEXT:    sta __rc3
; NOASR-NEXT:    ldx __rc5
; NOASR-NEXT:    lda __rc4
; NOASR-NEXT:    rts
entry:
  %0 = ashr i32 %a, 1
  ret i32 %0
}

; Each shift-by-1 iteration must independently preserve the sign bit.
define i8 @ashr_i8_3(i8 %a) {
; CHECK-LABEL: ashr_i8_3:
; CHECK:       ; %bb.0: ; %entry
; CHECK-NEXT:    asr
; CHECK-NEXT:    asr
; CHECK-NEXT:    asr
; CHECK-NEXT:    rts
;
; NOASR-LABEL: ashr_i8_3:
; NOASR:       ; %bb.0: ; %entry
; NOASR-NEXT:    cmp #128
; NOASR-NEXT:    ror
; NOASR-NEXT:    cmp #128
; NOASR-NEXT:    ror
; NOASR-NEXT:    cmp #128
; NOASR-NEXT:    ror
; NOASR-NEXT:    rts
entry:
  %0 = ashr i8 %a, 3
  ret i8 %0
}

; Unsigned shifts must use LSR, not ASR (ASR would incorrectly preserve sign).
define i8 @lshr_i8_1(i8 %a) {
; CHECK-LABEL: lshr_i8_1:
; CHECK:       ; %bb.0: ; %entry
; CHECK-NEXT:    lsr
; CHECK-NEXT:    rts
entry:
  %0 = lshr i8 %a, 1
  ret i8 %0
}

; Multi-byte unsigned: high byte must use LSR to zero-extend, not ASR.
define i16 @lshr_i16_1(i16 %a) {
; CHECK-LABEL: lshr_i16_1:
; CHECK:       ; %bb.0: ; %entry
; CHECK-NEXT:    stx __rc2
; CHECK-NEXT:    lsr __rc2
; CHECK-NEXT:    ror
; CHECK-NEXT:    ldx __rc2
; CHECK-NEXT:    rts
entry:
  %0 = lshr i16 %a, 1
  ret i16 %0
}

; Left shifts must not be affected by the ASR pattern match.
define i16 @shl_i16_1(i16 %a) {
; CHECK-LABEL: shl_i16_1:
; CHECK:       ; %bb.0: ; %entry
; CHECK-NEXT:    stx __rc2
; CHECK-NEXT:    asl
; CHECK-NEXT:    rol __rc2
; CHECK-NEXT:    ldx __rc2
; CHECK-NEXT:    rts
entry:
  %0 = shl i16 %a, 1
  ret i16 %0
}

; Store-folded RMW: the shift result is consumed by RORAbs, leaving ASR's data
; output dead. Selecting ASR here would only add register pressure.
@g16 = global i16 0
define void @ashr_global_i16() {
; CHECK-LABEL: ashr_global_i16:
; CHECK:       ; %bb.0: ; %entry
; CHECK-NOT:     asr
; CHECK:         ror{{$}}
; CHECK-NOT:     asr
; CHECK:         ror g16
; CHECK-NOT:     asr
; CHECK:         ror g16+1
; CHECK-NOT:     asr
; CHECK:         rts
;
; NOASR-LABEL: ashr_global_i16:
; NOASR:       ; %bb.0: ; %entry
; NOASR-NEXT:    lda g16+1
; NOASR-NEXT:    cmp #128
; NOASR-NEXT:    ldx #1
; NOASR-NEXT:    bcs .LBB[[#]]_2
; NOASR:       ; %bb.1: ; %entry
; NOASR-NEXT:    ldx #0
; NOASR:       .LBB[[#]]_2: ; %entry
; NOASR-NEXT:    ror{{$}}
; NOASR-NEXT:    ror g16
; NOASR-NEXT:    cpx #1
; NOASR-NEXT:    ror g16+1
; NOASR-NEXT:    rts
entry:
  %v = load i16, ptr @g16
  %s = ashr i16 %v, 1
  store i16 %s, ptr @g16
  ret void
}
