; RUN: opt -passes=mos-indexiv -verify-each -S %s | FileCheck %s
target datalayout = "e-p:16:8:8-i16:8:8-i32:8:8-i64:8:8-f32:8:8-f64:8:8-a:8:8-Fi8-n8"
target triple = "mos"

@.str = constant [4 x i8] c"STR\00"

define i16 @main() {
entry:
  br label %while.body

; CHECK-LABEL: while.body:
while.body:
  %cur = phi i8* [ getelementptr inbounds ([4 x i8], [4 x i8]* @.str, i16 0, i16 0), %entry ], [ %incdec.ptr, %while.body ]
  %0 = load i8, i8* %cur
  %incdec.ptr = getelementptr inbounds i8, i8* %cur, i16 1
  %tobool.not = icmp eq i8 %0, 0
  br i1 %tobool.not, label %while.end, label %while.body

; CHECK: %mos-indexiv.iv = phi i8 [ %mos-indexiv.iv.next, %while.body ], [ 0, %entry ]
; CHECK: %cur = phi i8* [ getelementptr inbounds ([4 x i8], [4 x i8]* @.str, i16 0, i16 0), %entry ], [ %uglygep, %while.body ]
; CHECK: %1 = zext i8 %mos-indexiv.iv to i16
; CHECK: %uglygep = getelementptr i8, i8* getelementptr inbounds ([4 x i8], [4 x i8]* @.str, i16 0, i16 1), i16 %1
; CHECK: %mos-indexiv.iv.next = add nuw nsw i8 %mos-indexiv.iv, 1

; CHECK-LABEL: while.end:
while.end:
  ret i16 0
}
