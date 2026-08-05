; RUN: llvm-mc -triple mos -mcpu=mosw65816 --filetype=obj -I %S/Inputs -o=%t.obj %s
; RUN: llvm-objdump -d %t.obj | FileCheck %s

; 65816-only branch kinds, resolved byte-exact at a modest distance (the
; boundary edges live in branch-range-boundary.s). Hand-derived (not
; captured from the assembler under test): displacement = target -
; address_of_next_instruction. bra (opcode 0x80, unconditional relative) is
; 2 bytes, so address_of_next_instruction = opcode_addr + 2; brl (opcode
; 0x82, unconditional long) is 3 bytes, so address_of_next_instruction =
; opcode_addr + 3.

; --- bra (PCRel8), forward ---
; N = 20 -> displacement = 20 = 0x14
  bra fwd8                        ; CHECK: 80 14 bra
  .fill 20, 1, 0xea
fwd8:
  rts

; --- bra (PCRel8), backward ---
; N = 20 -> displacement = -(20 + 2) = -22 = 0xea
back8:
  .fill 20, 1, 0xea
  bra back8                       ; CHECK: 80 ea bra

; --- brl (PCRel16), forward ---
; N = 300 -> displacement = 300 = 0x012c -> bytes 2c 01 (little-endian)
  brl fwd16                       ; CHECK: 82 2c 01 brl
  .fill 300, 1, 0xea
fwd16:
  rts

; --- brl (PCRel16), backward ---
; N = 300 -> displacement = -(300 + 3) = -303 = 0xfed1 -> bytes d1 fe
back16:
  .fill 300, 1, 0xea
  brl back16                      ; CHECK: 82 d1 fe brl
