; RUN: llvm-mc -triple mos -motorola-integers --filetype=obj -I %S/Inputs -o=%t.obj %s
; RUN: llvm-objdump -d %t.obj | FileCheck %s

.text
.p2align 2
aligned:
; CHECK: <aligned>
  rts ; CHECK: 60 rts
.p2align 2
; CHECK: ea nop
; CHECK: ea nop
; CHECK: ea nop
realigned:
; CHECK: <realigned>
  rts ; CHECK: 60 rts
