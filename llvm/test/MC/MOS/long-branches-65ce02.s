; RUN: llvm-mc -triple mos -mcpu=mos65ce02 --filetype=obj -I %S/Inputs -o=%t.obj %s
; RUN: llvm-objdump -d %t.obj | FileCheck %s
; RUN: llvm-mc -triple mos -mcpu=mos45gs02 --filetype=obj -I %S/Inputs -o=%t45.obj %s
; RUN: llvm-objdump -d %t45.obj | FileCheck %s

; --- Forward 8-bit branch (within range, stays 8-bit) ---
  bcc	short_branch ; CHECK: 90 08 bcc
  .fill 8, 1, 0xEA
short_branch:

; --- Forward 16-bit branch (exceeds 8-bit range, relaxed to 16-bit) ---
  bcc	long_branch ; CHECK: 93 41 01 bcc
  .fill 320, 1, 0xEA
long_branch:

; --- Backward 16-bit branch (negative offset encoding) ---
backward_target:
  nop
  .fill 200, 1, 0xEA
  bcc	backward_target ; CHECK: 93 35 ff bcc

; --- Forward boundary: offset 127 stays 8-bit ---
  bcc	boundary_8bit ; CHECK: 90 7f bcc
  .fill 127, 1, 0xEA
boundary_8bit:

; --- Forward boundary+1: offset 128 relaxes to 16-bit ---
  bcc	boundary_16bit ; CHECK: 93 81 00 bcc
  .fill 128, 1, 0xEA
boundary_16bit:
  rts
