//===-- lib/mos/subsf3.c - Single-precision subtraction (MOS) --*- C -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// __subsf3(a, b) -- IEEE 754 binary32 subtraction for MOS (6502).
//
// Implemented entirely in terms of __addsf3: since
//
//     a - b  ==  a + (-b)
//
// we negate b's sign bit and tail-call __addsf3.  This reuses the
// full, already-correct-and-tested add machinery (unpack, magnitude
// compare/swap, inf/nan/zero dispatch, align, add/subtract dispatch,
// normalize, subnormal fix, overflow, round, pack) unchanged, at the
// cost of six instructions (11 bytes / ~15 cycles) of wrapper overhead:
// TAY to stash a's byte 0 (which arrives in A), LDA/EOR/STA to flip b's
// sign bit in __rc7, TYA to restore A, then a JMP tail-call into
// __addsf3.  The A-save is required because the 6502's EOR only
// operates on the accumulator, and __addsf3 expects a's byte 0 in A
// on entry.  Y is used for the stash because Y is not part of the
// input ABI (numeric args ride only A/X/__rc2..__rc15).
//
// Why this and not a stand-alone hand-asm like __addsf3/__mulsf3?
// Subtraction introduces no new arithmetic cases -- its only semantic
// difference from addition is the sign of the second operand.  A
// duplicated copy of addsf3.c would be ~800 lines of dead-maintenance
// risk for zero behavioral gain.  The trampoline below is the minimal
// correct expression of that fact.
//
// ============================================================================
// ABI (identical to __addsf3)
// ============================================================================
//
// float __subsf3(float a, float b) with the standard llvm-mos ABI:
//
//   INPUT:  a's bytes in A, X, __rc2, __rc3   (byte 0 in A ... byte 3 in rc3)
//           b's bytes in __rc4, __rc5, __rc6, __rc7
//   OUTPUT: result bytes in A, X, __rc2, __rc3
//
// __rc7 holds b's byte 3, whose bit 7 is b's sign (per the IEEE 754
// binary32 little-endian layout documented in addsf3.c).  Flipping that
// bit negates b, after which __addsf3 performs the subtraction.
//
// This function is naked: there is no prologue or epilogue, and the
// final JMP transfers control permanently to __addsf3, whose own RTS
// becomes our return.  (The __addsf3 symbol is a non-static global, so
// the assembler's external reference resolves at link time.)
//
//===----------------------------------------------------------------------===//

#include "../int_lib.h"

__attribute__((naked, noinline))
COMPILER_RT_ABI float __subsf3(float af, float bf) {
    asm volatile(
        // A holds a's byte 0 on entry -- stash it in Y before we
        // clobber A doing the sign flip.  Y is otherwise unused by
        // the input ABI.
        "tay\n"
        // Negate b by flipping its sign bit (bit 7 of byte 3, in __rc7).
        "lda __rc7\n"
        "eor #$80\n"
        "sta __rc7\n"
        // Restore a's byte 0 into A for __addsf3's entry ABI.
        "tya\n"
        // Tail-call __addsf3: computes a + (-b) == a - b.
        "jmp __addsf3\n"
    );
}