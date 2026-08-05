; RUN: llvm-mc -triple mos -mcpu=mos65ce02 --filetype=obj -I %S/Inputs -o=%t.obj %s
; RUN: llvm-objdump -d %t.obj | FileCheck %s

; 65CE02 PCRel16 fixups are relative to opcode + 2.

; PCRel16 +301
  bcc fwd16                       ; CHECK: 93 2d 01 bcc
  .fill 300, 1, 0xea
fwd16:
  rts

; PCRel16 -302
back16:
  .fill 300, 1, 0xea
  bcc back16                      ; CHECK: 93 d2 fe bcc
