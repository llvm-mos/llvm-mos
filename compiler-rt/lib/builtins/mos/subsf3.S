//===-- lib/mos/subsf3.S - Single-precision subtraction (MOS 6502) -*- Asm -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// __subsf3(a, b) = a - b, via a + (-b): flip b's sign bit and tail-call
// __addsf3.  All arithmetic, all edge-case handling, all rounding is
// inherited from __addsf3 (see addsf3.S).
//
// Why this shape:
//   - Duplicating __addsf3's ~900-byte body just to swap a subtract
//     sign somewhere in the middle would be pure maintenance risk for
//     zero behavioral gain -- especially given how sensitive the
//     rounding logic is (see the IEEE 754 sticky-bit sign-flip fix in
//     __addsf3 for what a subtract-path shortcut can go wrong).
//   - The upstream generic subsf3.c takes exactly this "a + (-b)"
//     approach.  Ours is the same idea, but 11 bytes instead of a
//     compiler-generated wrapper around C bit-fiddling.
//
// Registers on entry (llvm-mos ABI):
//     A  = a byte 0     __rc2 = a byte 2
//     X  = a byte 1     __rc3 = a byte 3
//                       __rc4 = b byte 0    __rc6 = b byte 2
//                       __rc5 = b byte 1    __rc7 = b byte 3  (sign bit = 7)
//
// TAY/TYA around the EOR is because 6502's EOR only operates on A, but
// __addsf3 expects a's byte 0 in A on entry.  Y is otherwise unused by
// the numeric-argument ABI (numeric args ride A/X + __rc2..__rc15).
// The JMP tail-call means __addsf3's RTS returns directly to
// __subsf3's caller; no stack frame here.
//
//===----------------------------------------------------------------------===//

#include "imag-regs-zp.inc"

.text
.globl __subsf3
.type __subsf3, @function
__subsf3:
        tay                                     // stash a byte 0
        lda __rc7                               // b byte 3 (sign in bit 7)
        eor #$80                                // flip sign
        sta __rc7
        tya                                     // restore a byte 0 to A
        jmp __addsf3
.size __subsf3, .-__subsf3
