; RUN: not llvm-mc -triple mos -mcpu=mosw65816 --filetype=obj -o=/dev/null %s 2>&1 | FileCheck %s

; PCRel8 +128
; CHECK: [[#@LINE+1]]:7: error: 8-bit branch target out of range
  beq far_target
  .fill 128, 1, 0xea
far_target:
  rts

; PCRel8 -129
neg_far_target:
  .fill 127, 1, 0xea
; CHECK: [[#@LINE+1]]:7: error: 8-bit branch target out of range
  beq neg_far_target

; PCRel16 +32768
; CHECK: [[#@LINE+1]]:7: error: 16-bit branch target out of range
  brl far_target16
  .fill 32768, 1, 0xea
far_target16:
  rts

; PCRel16 -32769
neg_far_target16:
  .fill 32766, 1, 0xea
; CHECK: [[#@LINE+1]]:7: error: 16-bit branch target out of range
  brl neg_far_target16
