; RUN: llc -verify-machineinstrs < %s | FileCheck %s

target datalayout = "e-m:e-p:16:8-p1:8:8-i16:8-i32:8-i64:8-f32:8-f64:8-a:8-Fi8-n8"
target triple = "mos"

; The stack protector libcalls must be available on MOS. Lowering `sspreq`
; needs both `__stack_chk_guard`, to load the canary, and `__stack_chk_fail`,
; for the failure path. When either is missing from MOSSystemLibrary,
; lowering reports "unable to lower stackguard".

define void @func() sspreq nounwind {
; CHECK-LABEL: func:
; CHECK: __stack_chk_guard
; CHECK: __stack_chk_fail
  %alloca = alloca i32, align 4
  call void @capture(ptr %alloca)
  ret void
}

declare void @capture(ptr)

; A nonreentrant function whose only frame object is the protector slot must
; not be laid out on the static stack. The first static object lands at offset
; 0, which prologue/epilogue insertion reads as an unset protector offset.
define void @protectoronly() sspreq nounwind {
; CHECK-LABEL: protectoronly:
; CHECK: __stack_chk_guard
; CHECK: __stack_chk_fail
  ret void
}
