; RUN: llvm-mc -triple mos -mcpu=mosw65816 --filetype=obj -I %S/Inputs -o=%t.obj %s
; RUN: llvm-objdump -d %t.obj | FileCheck %s

  bra	short_branch ; CHECK: 80 08 bra
  .fill 8, 1, 0xEA
short_branch:
  bra	long_branch ; CHECK: 82 40 01 brl
  .fill 320, 1, 0xEA
long_branch:
  rts
