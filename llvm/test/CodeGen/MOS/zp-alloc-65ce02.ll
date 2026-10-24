; RUN: llc -mcpu=mos65ce02 -verify-machineinstrs -O2 --filetype=asm -zp-avail=224 < %s | FileCheck %s
; RUN: llc -mcpu=mos45gs02 -verify-machineinstrs -O2 --filetype=asm -zp-avail=224 < %s | FileCheck %s

target datalayout = "e-m:e-p:16:8-p1:8:8-i16:8-i32:8-i64:8-f32:8-f64:8-a:8-Fi8-n8"
target triple = "mos-sim"

declare void @ext() nocallback

; Pointers reference Imag16 before ZP allocation; this covers #551's path.
define ptr @ptr_add2(ptr %p) norecurse {
; CHECK-LABEL: ptr_add2:
; CHECK:       inw mos8(.Lptr_add2_zp_stk)
; CHECK-NEXT:  inw mos8(.Lptr_add2_zp_stk)
entry:
  tail call void @ext()
  %inc = getelementptr i8, ptr %p, i16 2
  ret ptr %inc
}

define void @csr() norecurse {
; CHECK-LABEL: csr:
; CHECK-NOT:   dew __rc20
; CHECK:       dew mos8(.Lcsr_zp_stk)
; CHECK-NEXT:  lda mos8(.Lcsr_zp_stk+1)
entry:
  br label %for.body

for.cond.cleanup:
  ret void

for.body:
  %i = phi i16 [ 0, %entry ], [ %inc1, %for.body ]
  tail call void @ext()
  %inc1 = add i16 %i, 1
  %exitcond.not = icmp eq i16 %inc1, 12345
  br i1 %exitcond.not, label %for.cond.cleanup, label %for.body
}
