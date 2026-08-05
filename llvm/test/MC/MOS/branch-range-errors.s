; RUN: not llvm-mc -triple mos -mcpu=mosw65816 --filetype=obj -o=/dev/null %s 2>&1 | FileCheck %s

; A conditional branch has no automatic long form on the 65816. It must fail
; rather than silently truncating its signed 8-bit displacement.
; CHECK: [[#@LINE+1]]:7: error: 8-bit branch target out of range
  beq far_target
  .fill 128, 1, 0xea
far_target:
  rts
