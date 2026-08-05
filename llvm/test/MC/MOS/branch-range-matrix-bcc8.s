; RUN: llvm-mc -triple mos -mcpu=mos6502 --filetype=obj -I %S/Inputs -o=%t6502.obj %s
; RUN: llvm-objdump -d %t6502.obj | FileCheck %s
; RUN: llvm-mc -triple mos -mcpu=mos65ce02 --filetype=obj -I %S/Inputs -o=%t65ce02.obj %s
; RUN: llvm-objdump -d %t65ce02.obj | FileCheck %s

; PCRel8 +10
  bcc fwd                         ; CHECK: 90 0a bcc
  .fill 10, 1, 0xea
fwd:
  rts

; PCRel8 -12
back:
  .fill 10, 1, 0xea
  bcc back                        ; CHECK: 90 f4 bcc
