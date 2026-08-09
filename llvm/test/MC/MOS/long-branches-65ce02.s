; RUN: llvm-mc -triple mos -mcpu=mos65ce02 --filetype=obj -I %S/Inputs -o=%t.obj %s
; RUN: llvm-objdump -d %t.obj | FileCheck %s
; RUN: llvm-mc -triple mos -mcpu=mos45gs02 --filetype=obj -I %S/Inputs -o=%t45.obj %s
; RUN: llvm-objdump -d %t45.obj | FileCheck %s

; Each branch is checked for its encoding and for the target the disassembler
; derives from it. A 16-bit branch is relative to opcode + 2, so the printed
; address must equal the label's, and the symbolic form must be a bare symbol
; rather than symbol+0x1.

; --- Forward 8-bit branch (within range, stays 8-bit) ---
  bcc	short_branch ; CHECK: 90 08 bcc $a <short_branch>
  .fill 8, 1, 0xEA
short_branch:

; --- Forward 16-bit branch (exceeds 8-bit range, relaxed to 16-bit) ---
  bcc	long_branch ; CHECK: 93 41 01 bcc $14d <{{[a-z_0-9]+}}>
  .fill 320, 1, 0xEA
long_branch:

; --- Backward 16-bit branch (negative offset encoding) ---
backward_target:
  nop
  .fill 200, 1, 0xEA
  bcc	backward_target ; CHECK: 93 35 ff bcc $14d <{{[a-z_0-9]+}}>

; --- Forward boundary: offset 127 stays 8-bit ---
  bcc	boundary_8bit ; CHECK: 90 7f bcc $29a <boundary_8bit>
  .fill 127, 1, 0xEA
boundary_8bit:

; --- Forward boundary+1: offset 128 relaxes to 16-bit ---
  bcc	boundary_16bit ; CHECK: 93 81 00 bcc $31d <{{[a-z_0-9]+}}>
  .fill 128, 1, 0xEA
boundary_16bit:

; --- Backward boundary: offset -128 stays 8-bit ---
neg_boundary_8bit:
  .fill 126, 1, 0xEA
  bcc	neg_boundary_8bit ; CHECK: 90 80 bcc $31d <{{[a-z_0-9]+}}>

; --- Backward boundary-1: offset -129 relaxes to 16-bit ---
neg_boundary_16bit:
  .fill 127, 1, 0xEA
  bcc	neg_boundary_16bit ; CHECK: 93 7f ff bcc $39d <neg_boundary_16bit>
  rts
