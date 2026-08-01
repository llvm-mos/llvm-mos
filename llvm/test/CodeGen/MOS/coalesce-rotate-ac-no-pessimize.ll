; RUN: llc -mtriple=mos -mcpu=mosw65816 -verify-machineinstrs < %s | FileCheck %s
;
; Cost side of the shouldCoalesce Ac guard (see coalesce-rotate-ac.mir): the
; guard refuses to coalesce two shift/rotate-referenced values into the A-only
; Ac class even in straight-line code, so pin that the common multi-shift
; shapes keep their tight form — the COPY the guard preserves is allocated to
; A and folds away, with no transfer or extra zero-page traffic between the
; shift ops.

target datalayout = "e-m:e-p:16:8-p1:8:8-i16:8-i32:8-i64:8-f32:8-f64:8-a:8-Fi8-n8"
target triple = "mos"

define i8 @shl2_u8(i8 %x) {
; CHECK-LABEL: shl2_u8:
; CHECK:      asl
; CHECK-NEXT: asl
; CHECK-NEXT: rts
  %r = shl i8 %x, 2
  ret i8 %r
}

define i16 @shl2_u16(i16 %x) {
; CHECK-LABEL: shl2_u16:
; CHECK:      stx [[HI:.*]]
; CHECK-NEXT: asl
; CHECK-NEXT: rol [[HI]]
; CHECK-NEXT: asl
; CHECK-NEXT: rol [[HI]]
; CHECK-NEXT: ldx [[HI]]
; CHECK-NEXT: rts
  %r = shl i16 %x, 2
  ret i16 %r
}
