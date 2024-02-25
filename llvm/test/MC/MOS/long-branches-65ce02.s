; RUN: llvm-mc -triple mos -mcpu=mos65ce02 --filetype=obj -I %S/Inputs -o=%t.obj %s
; RUN: llvm-objdump -d %t.obj | FileCheck %s

  bcc	short_branch ; CHECK: 90 08 bcc
  .fill 8, 1, 0xEA
short_branch:
  bcc	long_branch ; CHECK: 93 40 01 bcc
  .fill 320, 1, 0xEA
long_branch:
  rts
