; RUN: llvm-mc -triple mos -motorola-integers --mcpu=mosr65c02 --filetype=obj -o=%t.obj %s
; RUN: llvm-objdump -d %t.obj | FileCheck %s

; BBR/BBS are three bytes and their displacement is measured from the end of
; the instruction, not from opcode + 2 like the two-byte branches. The
; disassembled target must therefore be the label itself.

  bbr0	$10, fwd ; CHECK: 0f 10 01 bbr $0,$10,$4 <fwd>
  nop
fwd:
  nop
back:
  nop
  bbs7	$20, back ; CHECK: ff 20 fc bbs $7,$20,$5 <back>
  rts
