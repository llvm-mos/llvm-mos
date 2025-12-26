; RUN: llc -mtriple=mos < %s | FileCheck %s

; Test that MOS can handle large stack allocations up to the 64KB limit.

define void @large_stack() {
; CHECK-LABEL: large_stack:
; CHECK: clc
; CHECK: lda __rc0
; CHECK: adc
; CHECK: sta __rc0
; CHECK: lda __rc1
; CHECK: adc
; CHECK: sta __rc1
  %arr = alloca [60000 x i8]
  call void @use(ptr %arr)
  ret void
}

declare void @use(ptr)
