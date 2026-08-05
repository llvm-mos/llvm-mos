; RUN: llvm-mc -triple mos -mcpu=mos6502 --filetype=obj -I %S/Inputs -o=%t6502.obj %s
; RUN: llvm-objdump -d %t6502.obj | FileCheck %s
; RUN: llvm-mc -triple mos -mcpu=mos65ce02 --filetype=obj -I %S/Inputs -o=%t65ce02.obj %s
; RUN: llvm-objdump -d %t65ce02.obj | FileCheck %s

; The 8-bit bcc (opcode 0x90) encoding and its PC correction (-1) are CPU-
; independent, and unaffected by the #549 PCRel16 correction bug (which is
; PCRel16-only) -- so this file is run unchanged against both mos6502 and
; mos65ce02, and both should assemble byte-identically.
;
; Hand-derived (not captured from the assembler under test):
; displacement = target - (opcode_addr + 2), since bcc is 2 bytes.
; N filler bytes placed right after a forward bcc puts its target exactly N
; bytes past the next instruction, so displacement = N; placed right before
; a backward bcc, displacement = -(N + 2).

; --- Forward, modest distance ---
; N = 10 -> displacement = 10 = 0x0a
  bcc fwd                         ; CHECK: 90 0a bcc
  .fill 10, 1, 0xea
fwd:
  rts

; --- Backward, modest distance ---
; N = 10 -> displacement = -(10 + 2) = -12 = 0xf4
back:
  .fill 10, 1, 0xea
  bcc back                        ; CHECK: 90 f4 bcc
