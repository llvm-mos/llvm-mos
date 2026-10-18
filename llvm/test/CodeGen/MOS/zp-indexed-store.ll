; RUN: llc -O2 -zp-avail=224 -verify-machineinstrs -asm-show-inst < %s | FileCheck %s

; An X-indexed store to a zero page global should select STA zp,X, as the load
; selects LDA zp,X. The operand prints as mos8(t) either way, so check the
; selected instruction.

target datalayout = "e-m:e-p:16:8-p1:8:8-i16:8-i32:8-i64:8-f32:8-f64:8-a:8-Fi8-n8"
target triple = "mos-sim"

; Placed in the zero page by the zero page allocator.
@t = internal global [16 x i8] undef

define void @store_indexed_by_x(i8 %v, i8 %i) {
; CHECK-LABEL: store_indexed_by_x:
; CHECK: sta mos8(t),x {{.*}}STA_ZeroPageX
  %o = zext i8 %i to i16
  %p = getelementptr [16 x i8], ptr @t, i16 0, i16 %o
  store i8 %v, ptr %p
  ret void
}

define i8 @load_indexed_by_x(i8 %i, i8 %j) {
; CHECK-LABEL: load_indexed_by_x:
; CHECK: lda mos8(t),x {{.*}}LDA_ZeroPageX
  %o = zext i8 %j to i16
  %p = getelementptr [16 x i8], ptr @t, i16 0, i16 %o
  %v = load i8, ptr %p
  ret i8 %v
}

; STA has no zero page,Y form, so a store indexed by Y stays absolute.
define void @store_indexed_by_y(i8 %v, i8 %i, i8 %j) {
; CHECK-LABEL: store_indexed_by_y:
; CHECK: sta mos8(t),x {{.*}}STA_ZeroPageX
; CHECK: sta mos8(t),y {{.*}}STA_AbsoluteY
  %oi = zext i8 %i to i16
  %pi = getelementptr [16 x i8], ptr @t, i16 0, i16 %oi
  store volatile i8 %v, ptr %pi
  %oj = zext i8 %j to i16
  %pj = getelementptr [16 x i8], ptr @t, i16 0, i16 %oj
  store volatile i8 %v, ptr %pj
  ret void
}

; A constant base in page zero keeps the absolute form, wrapped in mos16 as
; for loads.
define void @store_const_page_zero_x(i8 %v, i8 %i) {
; CHECK-LABEL: store_const_page_zero_x:
; CHECK: sta mos16(16),x {{.*}}STA_AbsoluteX
  %o = zext i8 %i to i16
  %p = getelementptr i8, ptr inttoptr (i16 16 to ptr), i16 %o
  store i8 %v, ptr %p
  ret void
}

define i8 @load_const_page_zero_x(i8 %i, i8 %j) {
; CHECK-LABEL: load_const_page_zero_x:
; CHECK: lda mos16(16),x {{.*}}LDA_AbsoluteX
  %o = zext i8 %j to i16
  %p = getelementptr i8, ptr inttoptr (i16 16 to ptr), i16 %o
  %v = load i8, ptr %p
  ret i8 %v
}
