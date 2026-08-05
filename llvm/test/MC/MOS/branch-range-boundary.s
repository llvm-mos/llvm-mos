; RUN: llvm-mc -triple mos -mcpu=mosw65816 --filetype=obj -I %S/Inputs -o=%t.obj %s
; RUN: llvm-objdump -d %t.obj | FileCheck %s

; Boundary-exact PCRel8/PCRel16 pairs, at the legal edge (the illegal edge,
; one byte further, is pinned in branch-range-errors.s). Expected bytes are
; hand-derived from 6502-family relative addressing, not captured from the
; assembler under test: displacement = target - address_of_next_instruction.
; beq (opcode 0xf0) is 2 bytes (1 opcode + 1-byte signed operand), so
; address_of_next_instruction = opcode_addr + 2; brl (opcode 0x82) is
; 65816-only and 3 bytes (1 opcode + 2-byte signed operand), so
; address_of_next_instruction = opcode_addr + 3 (see getRelativeMOSPCCorrection
; in MOSAsmBackend.cpp). With N filler bytes placed immediately after a
; forward branch, its target sits exactly N bytes past the next instruction,
; so the displacement equals N directly; placed immediately before a
; backward branch, the displacement is -(N + instruction length).

; --- PCRel8 (beq), forward, exactly +127 (INT8_MAX): legal ---
; N = 127 -> displacement = 127 = 0x7f
  beq fwd8                        ; CHECK: f0 7f beq
  .fill 127, 1, 0xea
fwd8:
  rts

; --- PCRel8 (beq), backward, exactly -128 (INT8_MIN): legal ---
; N = 126 -> displacement = -(126 + 2) = -128 = 0x80
back8:
  .fill 126, 1, 0xea
  beq back8                       ; CHECK: f0 80 beq

; --- PCRel16 (brl), forward, exactly +32767 (INT16_MAX): legal ---
; N = 32767 -> displacement = 32767 = 0x7fff -> bytes ff 7f (little-endian)
  brl fwd16                       ; CHECK: 82 ff 7f brl
  .fill 32767, 1, 0xea
fwd16:
  rts

; --- PCRel16 (brl), backward, exactly -32768 (INT16_MIN): legal ---
; N = 32765 -> displacement = -(32765 + 3) = -32768 = 0x8000 -> bytes 00 80
back16:
  .fill 32765, 1, 0xea
  brl back16                      ; CHECK: 82 00 80 brl
