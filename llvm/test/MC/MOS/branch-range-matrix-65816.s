; RUN: llvm-mc -triple mos -mcpu=mosw65816 --filetype=obj -I %S/Inputs -o=%t.obj %s
; RUN: llvm-objdump -d %t.obj | FileCheck %s

; PCRel8 +20
  bra fwd8                        ; CHECK: 80 14 bra
  .fill 20, 1, 0xea
fwd8:
  rts

; PCRel8 -22
back8:
  .fill 20, 1, 0xea
  bra back8                       ; CHECK: 80 ea bra

; PCRel16 +300
  brl fwd16                       ; CHECK: 82 2c 01 brl
  .fill 300, 1, 0xea
fwd16:
  rts

; PCRel16 -303
back16:
  .fill 300, 1, 0xea
  brl back16                      ; CHECK: 82 d1 fe brl
