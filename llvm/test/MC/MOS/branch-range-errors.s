; RUN: not llvm-mc -triple mos -mcpu=mosw65816 --filetype=obj -o=/dev/null %s 2>&1 | FileCheck %s

; A conditional branch has no automatic long form on the 65816. It must fail
; rather than silently truncating its signed 8-bit displacement.
; CHECK: [[#@LINE+1]]:7: error: 8-bit branch target out of range
  beq far_target
  .fill 128, 1, 0xea
far_target:
  rts

; Backward mirror of the above: for a 2-byte beq, displacement = target -
; (opcode_addr + 2). The legal edge is -128 at 126 filler bytes (see
; branch-range-boundary.s for the legal pairs); one byte further back
; (127 filler bytes, displacement -129) is the first illegal negative value.
neg_far_target:
  .fill 127, 1, 0xea
; CHECK: [[#@LINE+1]]:7: error: 8-bit branch target out of range
  beq neg_far_target

; PCRel16 (brl, 65816-only unconditional long branch, 3 bytes) forward: the
; legal edge is +32767 at 32767 filler bytes (see branch-range-boundary.s);
; one byte further errors.
; CHECK: [[#@LINE+1]]:7: error: 16-bit branch target out of range
  brl far_target16
  .fill 32768, 1, 0xea
far_target16:
  rts

; PCRel16 backward mirror: the legal edge is -32768 at 32765 filler bytes;
; one byte further back (32766) is the first illegal negative value.
neg_far_target16:
  .fill 32766, 1, 0xea
; CHECK: [[#@LINE+1]]:7: error: 16-bit branch target out of range
  brl neg_far_target16
