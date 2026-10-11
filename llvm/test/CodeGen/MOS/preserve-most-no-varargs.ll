; RUN: not llc -verify-machineinstrs -O2 < %s 2>&1 | FileCheck %s

; PreserveMost does not support variadic functions. This is reported as an
; error, not a crash, for both definitions and calls.

target datalayout = "e-m:e-p:16:8-p1:8:8-i16:8-i32:8-i64:8-f32:8-f64:8-a:8-Fi8-n8"
target triple = "mos"

; CHECK: error: {{.*}}in function varargs_fn void (i8, ...): preserve_most calling convention does not support variadic functions
define preserve_mostcc void @varargs_fn(i8 %a, ...) {
entry:
  ret void
}

declare preserve_mostcc void @varargs_decl(i8, ...)

; CHECK: error: {{.*}}in function call_varargs void (): preserve_most calling convention does not support variadic functions
define void @call_varargs() {
entry:
  call preserve_mostcc void (i8, ...) @varargs_decl(i8 1, i8 2)
  ret void
}
