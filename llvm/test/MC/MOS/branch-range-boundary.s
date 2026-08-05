; RUN: llvm-mc -triple mos -mcpu=mosw65816 --filetype=obj -I %S/Inputs -o=%t.obj %s
; RUN: llvm-objdump -d %t.obj | FileCheck %s

; PCRel8 +127
  beq fwd8                        ; CHECK: f0 7f beq
  .fill 127, 1, 0xea
fwd8:
  rts

; PCRel8 -128
back8:
  .fill 126, 1, 0xea
  beq back8                       ; CHECK: f0 80 beq

; PCRel16 +32767
  brl fwd16                       ; CHECK: 82 ff 7f brl
  .fill 32767, 1, 0xea
fwd16:
  rts

; PCRel16 -32768
back16:
  .fill 32765, 1, 0xea
  brl back16                      ; CHECK: 82 00 80 brl
