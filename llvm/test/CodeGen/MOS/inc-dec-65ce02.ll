; RUN: llc -mcpu=mos65ce02 -verify-machineinstrs < %s | FileCheck %s
; RUN: llc -mcpu=mos45gs02 -verify-machineinstrs < %s | FileCheck %s

target datalayout = "e-m:e-p:16:8-p1:8:8-i16:8-i32:8-i64:8-f32:8-f64:8-a:8-Fi8-n8"
target triple = "mos"

define i8 @inc_i8(i8 %a) {
; CHECK-LABEL: inc_i8:
; CHECK:       ; %bb.0: ; %entry
; CHECK-NEXT:    inc
; CHECK-NEXT:    rts
entry:
  %0 = add i8 %a, 1
  ret i8 %0
}

define i16 @inc_i16(i16 %a) {
; CHECK-LABEL: inc_i16:
; CHECK:       ; %bb.0: ; %entry
; CHECK-NEXT:    inc
; CHECK-NEXT:    bne .LBB1_2
; CHECK-NEXT:  ; %bb.1: ; %entry
; CHECK-NEXT:    inx
; CHECK-NEXT:  .LBB1_2: ; %entry
; CHECK-NEXT:    rts
entry:
  %0 = add i16 %a, 1
  ret i16 %0
}

define i32 @inc_i32(i32 %a) {
; CHECK-LABEL: inc_i32:
; CHECK:       ; %bb.0: ; %entry
; CHECK-NEXT:    inc
; CHECK-NEXT:    bne .LBB2_4
; CHECK-NEXT:  ; %bb.1: ; %entry
; CHECK-NEXT:    inx
; CHECK-NEXT:    bne .LBB2_3
; CHECK-NEXT:  ; %bb.2: ; %entry
; CHECK-NEXT:    inw __rc2
; CHECK-NEXT:  .LBB2_3: ; %entry
; CHECK-NEXT:  .LBB2_4: ; %entry
; CHECK-NEXT:    rts
entry:
  %0 = add i32 %a, 1
  ret i32 %0
}

define i64 @inc_i64(i64 %a) {
; CHECK-LABEL: inc_i64:
; CHECK:       ; %bb.0: ; %entry
; CHECK-NEXT:    inc
; CHECK-NEXT:    bne .LBB3_8
; CHECK-NEXT:  ; %bb.1: ; %entry
; CHECK-NEXT:    inx
; CHECK-NEXT:    bne .LBB3_7
; CHECK-NEXT:  ; %bb.2: ; %entry
; CHECK-NEXT:    inw __rc2
; CHECK-NEXT:    bne .LBB3_6
; CHECK-NEXT:  ; %bb.3: ; %entry
; CHECK-NEXT:    inw __rc4
; CHECK-NEXT:    bne .LBB3_5
; CHECK-NEXT:  ; %bb.4: ; %entry
; CHECK-NEXT:    inw __rc6
; CHECK-NEXT:  .LBB3_5: ; %entry
; CHECK-NEXT:  .LBB3_6: ; %entry
; CHECK-NEXT:  .LBB3_7: ; %entry
; CHECK-NEXT:  .LBB3_8: ; %entry
; CHECK-NEXT:    rts
entry:
  %0 = add i64 %a, 1
  ret i64 %0
}

define i8 @dec_i8(i8 %a) {
; CHECK-LABEL: dec_i8:
; CHECK:       ; %bb.0: ; %entry
; CHECK-NEXT:    dec
; CHECK-NEXT:    rts
entry:
  %0 = add i8 %a, -1
  ret i8 %0
}

define i16 @dec_i16(i16 %a) {
; CHECK-LABEL: dec_i16:
; CHECK:       ; %bb.0: ; %entry
; CHECK-NEXT:    dec
; CHECK-NEXT:    cmp #255
; CHECK-NEXT:    bne .LBB5_2
; CHECK-NEXT:  ; %bb.1: ; %entry
; CHECK-NEXT:    dex
; CHECK-NEXT:  .LBB5_2: ; %entry
; CHECK-NEXT:    rts
entry:
  %0 = add i16 %a, -1
  ret i16 %0
}

define i32 @dec_i32(i32 %a) {
; CHECK-LABEL: dec_i32:
; CHECK:       ; %bb.0: ; %entry
; CHECK-NEXT:    dec
; CHECK-NEXT:    cmp #255
; CHECK-NEXT:    bne .LBB6_4
; CHECK-NEXT:  ; %bb.1: ; %entry
; CHECK-NEXT:    dex
; CHECK-NEXT:    cpx #255
; CHECK-NEXT:    bne .LBB6_3
; CHECK-NEXT:  ; %bb.2: ; %entry
; CHECK-NEXT:    dew __rc2
; CHECK-NEXT:  .LBB6_3: ; %entry
; CHECK-NEXT:  .LBB6_4: ; %entry
; CHECK-NEXT:    rts
entry:
  %0 = add i32 %a, -1
  ret i32 %0
}

define i64 @dec_i64(i64 %a) {
; CHECK-LABEL: dec_i64:
; CHECK:       ; %bb.0: ; %entry
; CHECK-NEXT:    dec
; CHECK-NEXT:    cmp #255
; CHECK-NEXT:    bne .LBB7_12
; CHECK-NEXT:  ; %bb.1: ; %entry
; CHECK-NEXT:    dex
; CHECK-NEXT:    cpx #255
; CHECK-NEXT:    bne .LBB7_11
; CHECK-NEXT:  ; %bb.2: ; %entry
; CHECK-NEXT:    ldy #255
; CHECK-NEXT:    dec __rc2
; CHECK-NEXT:    cpy __rc2
; CHECK-NEXT:    bne .LBB7_10
; CHECK-NEXT:  ; %bb.3: ; %entry
; CHECK-NEXT:    ldy #255
; CHECK-NEXT:    dec __rc3
; CHECK-NEXT:    cpy __rc3
; CHECK-NEXT:    bne .LBB7_9
; CHECK-NEXT:  ; %bb.4: ; %entry
; CHECK-NEXT:    ldy #255
; CHECK-NEXT:    dec __rc4
; CHECK-NEXT:    cpy __rc4
; CHECK-NEXT:    bne .LBB7_8
; CHECK-NEXT:  ; %bb.5: ; %entry
; CHECK-NEXT:    ldy #255
; CHECK-NEXT:    dec __rc5
; CHECK-NEXT:    cpy __rc5
; CHECK-NEXT:    bne .LBB7_7
; CHECK-NEXT:  ; %bb.6: ; %entry
; CHECK-NEXT:    dew __rc6
; CHECK-NEXT:  .LBB7_7: ; %entry
; CHECK-NEXT:  .LBB7_8: ; %entry
; CHECK-NEXT:  .LBB7_9: ; %entry
; CHECK-NEXT:  .LBB7_10: ; %entry
; CHECK-NEXT:  .LBB7_11: ; %entry
; CHECK-NEXT:  .LBB7_12: ; %entry
; CHECK-NEXT:    rts
entry:
  %0 = add i64 %a, -1
  ret i64 %0
}

define ptr @inc_ptr(ptr %p) {
; CHECK-LABEL: inc_ptr:
; CHECK:       ; %bb.0: ; %entry
; CHECK-NEXT:    inw __rc2
; CHECK-NEXT:    rts
entry:
  %0 = getelementptr inbounds i8, ptr %p, i16 1
  ret ptr %0
}

define ptr @dec_ptr(ptr %p) {
; CHECK-LABEL: dec_ptr:
; CHECK:       ; %bb.0: ; %entry
; CHECK-NEXT:    dew __rc2
; CHECK-NEXT:    rts
entry:
  %0 = getelementptr inbounds i8, ptr %p, i16 -1
  ret ptr %0
}

; --- Chained INW/DEW for small constant offsets (65CE02+) ---
; Integer increment: threshold +2 (saves 1 byte + 4 cycles in A:X vs ADC).
; Integer decrement: threshold -1 (chained DEC adds cmp #255 borrow overhead).
; Pointer add: threshold ±4 (pointers always land in ZP where INW/DEW shines).

define i16 @inc_i16_by2(i16 %a) {
; CHECK-LABEL: inc_i16_by2:
; CHECK:       ; %bb.0: ; %entry
; CHECK-NEXT:    inc
; CHECK-NEXT:    bne .LBB10_2
; CHECK-NEXT:  ; %bb.1: ; %entry
; CHECK-NEXT:    inx
; CHECK-NEXT:  .LBB10_2: ; %entry
; CHECK-NEXT:    inc
; CHECK-NEXT:    bne .LBB10_4
; CHECK-NEXT:  ; %bb.3: ; %entry
; CHECK-NEXT:    inx
; CHECK-NEXT:  .LBB10_4: ; %entry
; CHECK-NEXT:    rts
entry:
  %0 = add i16 %a, 2
  ret i16 %0
}

; Verify i16 -=2 falls back to generic ADC (decrement threshold is 1 due to
; cmp #255 borrow overhead: +3 bytes vs generic SBC for each chained DEC).
define i16 @dec_i16_by2(i16 %a) {
; CHECK-LABEL: dec_i16_by2:
; CHECK:       ; %bb.0: ; %entry
; CHECK-NEXT:    clc
; CHECK-NEXT:    adc #254
; CHECK-NEXT:    tay
; CHECK-NEXT:    txa
; CHECK-NEXT:    adc #255
; CHECK-NEXT:    tax
; CHECK-NEXT:    tya
; CHECK-NEXT:    rts
entry:
  %0 = add i16 %a, -2
  ret i16 %0
}

define ptr @inc_ptr_by2(ptr %p) {
; CHECK-LABEL: inc_ptr_by2:
; CHECK:       ; %bb.0: ; %entry
; CHECK-NEXT:    inw __rc2
; CHECK-NEXT:    inw __rc2
; CHECK-NEXT:    rts
entry:
  %0 = getelementptr inbounds i8, ptr %p, i16 2
  ret ptr %0
}

define ptr @inc_ptr_by4(ptr %p) {
; CHECK-LABEL: inc_ptr_by4:
; CHECK:       ; %bb.0: ; %entry
; CHECK-NEXT:    inw __rc2
; CHECK-NEXT:    inw __rc2
; CHECK-NEXT:    inw __rc2
; CHECK-NEXT:    inw __rc2
; CHECK-NEXT:    rts
entry:
  %0 = getelementptr inbounds i8, ptr %p, i16 4
  ret ptr %0
}

define ptr @dec_ptr_by2(ptr %p) {
; CHECK-LABEL: dec_ptr_by2:
; CHECK:       ; %bb.0: ; %entry
; CHECK-NEXT:    dew __rc2
; CHECK-NEXT:    dew __rc2
; CHECK-NEXT:    rts
entry:
  %0 = getelementptr inbounds i8, ptr %p, i16 -2
  ret ptr %0
}

define ptr @dec_ptr_by4(ptr %p) {
; CHECK-LABEL: dec_ptr_by4:
; CHECK:       ; %bb.0: ; %entry
; CHECK-NEXT:    dew __rc2
; CHECK-NEXT:    dew __rc2
; CHECK-NEXT:    dew __rc2
; CHECK-NEXT:    dew __rc2
; CHECK-NEXT:    rts
entry:
  %0 = getelementptr inbounds i8, ptr %p, i16 -4
  ret ptr %0
}

define i32 @inc_i32_by2(i32 %a) {
; CHECK-LABEL: inc_i32_by2:
; CHECK:       ; %bb.0: ; %entry
; CHECK-NEXT:    inc
; CHECK-NEXT:    bne .LBB16_4
; CHECK-NEXT:  ; %bb.1: ; %entry
; CHECK-NEXT:    inx
; CHECK-NEXT:    bne .LBB16_3
; CHECK-NEXT:  ; %bb.2: ; %entry
; CHECK-NEXT:    inw __rc2
; CHECK-NEXT:  .LBB16_3: ; %entry
; CHECK-NEXT:  .LBB16_4: ; %entry
; CHECK-NEXT:    inc
; CHECK-NEXT:    bne .LBB16_8
; CHECK-NEXT:  ; %bb.5: ; %entry
; CHECK-NEXT:    inx
; CHECK-NEXT:    bne .LBB16_7
; CHECK-NEXT:  ; %bb.6: ; %entry
; CHECK-NEXT:    inw __rc2
; CHECK-NEXT:  .LBB16_7: ; %entry
; CHECK-NEXT:  .LBB16_8: ; %entry
; CHECK-NEXT:    rts
entry:
  %0 = add i32 %a, 2
  ret i32 %0
}

; Verify i16 +=3 falls back to generic ADC (above integer threshold of 2).
define i16 @inc_i16_by3_fallback(i16 %a) {
; CHECK-LABEL: inc_i16_by3_fallback:
; CHECK:       ; %bb.0: ; %entry
; CHECK-NEXT:    clc
; CHECK-NEXT:    adc #3
; CHECK-NEXT:    tay
; CHECK-NEXT:    txa
; CHECK-NEXT:    adc #0
; CHECK-NEXT:    tax
; CHECK-NEXT:    tya
; CHECK-NEXT:    rts
entry:
  %0 = add i16 %a, 3
  ret i16 %0
}
