//===-- lib/mos/addsf3.S - Single-precision addition (MOS 6502) --*- Asm -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// __addsf3(a, b) -- IEEE 754 binary32 addition for MOS (6502).
//
// Rounding: round-to-nearest-ties-to-even, hardcoded.  There is no
// support for fenv (no dynamic rounding mode, no inexact flag).  This
// is a deliberate choice: MOS programs almost never manipulate the
// floating-point environment, and the generic compiler-rt fenv
// machinery (__fe_getround, __fe_raise_inexact) is ~30x more expensive
// per call than the arithmetic it wraps.
//
// This file also provides the __subsf3 code path indirectly: the
// separate subsf3.S is a 6-instruction trampoline that flips b's sign
// bit and JMPs here.  Anything that fixes __addsf3 fixes __subsf3.
//
// ============================================================================
// REFERENCE: IEEE 754 binary32 memory layout (little-endian)
// ============================================================================
//
//   byte 3  : [S | EEEEEEE ]     S     = sign bit
//   byte 2  : [E | MMMMMMM ]     EEEEE = 8-bit biased exponent, split
//   byte 1  : [MMMMMMMM    ]     M     = 23-bit stored mantissa
//   byte 0  : [MMMMMMMM    ]     (implicit leading 1 for normals)
//
// The 8-bit biased exponent spans byte3's low 7 bits AND byte2's high
// bit.  To extract, shift bit 7 of byte2 into carry with ASL, then
// load byte3 and ROL through carry:
//
//     LDA byte2 ; ASL A    -- C = byte2[7] = E0 (low bit of exp)
//     LDA byte3 ; ROL A    -- A = (byte3 << 1) | C = biased exp
//
// Number semantics by exponent field:
//   exp == 0xFF, mantissa != 0 -- NaN (quiet if bit 22 (m3 bit 6) set)
//   exp == 0xFF, mantissa == 0 -- +/- infinity
//   exp == 0x00, mantissa != 0 -- subnormal (value = M * 2^-149)
//   exp == 0x00, mantissa == 0 -- +/- zero (sign preserved)
//   exp in [1,254]             -- normal (value = 1.M * 2^(exp-127))
//
// For arithmetic we restore the implicit leading 1 for normal operands
// so the mantissa is 24 bits [1.MMMMMMMMMMMMMMMMMMMMMMM].  We store it
// in the top three bytes of a 4-byte tuple [m3:m2:m1:m0] where m0 is a
// sub-LSB byte holding round + guard bits shifted down during
// alignment.  A separate `stk` byte accumulates a running sticky OR of
// bits that fall off the bottom of m0.
//
// ============================================================================
// REFERENCE: MOS calling convention for this function
// ============================================================================
//
// float __addsf3(float a, float b) with the standard llvm-mos ABI:
//
//   INPUT:  a's bytes in A, X, __rc2, __rc3  (byte 0 in A ... byte 3 in rc3)
//           b's bytes in __rc4, __rc5, __rc6, __rc7
//   OUTPUT: result bytes in A, X, __rc2, __rc3
//
//   Caller-saved:  A, X, Y, flags, __rc2..__rc19 (RS1..RS9)
//   Callee-saved:  __rc0/__rc1 (RS0 = soft SP), __rc20..__rc31 (RS10..RS15)
//
// We confine all state to caller-saved slots, so there is no prologue
// or epilogue and no zero-page save/restore cost.
//
// ============================================================================
// REFERENCE: zero-page layout during arithmetic
// ============================================================================
//
//   __rc2    B_M0    b's sub-LSB byte (0 on entry, holds R/G bits)
//   __rc3    B_M1    b's low mantissa byte  (was b_byte0 on entry)
//   __rc4    B_M2    b's mid mantissa byte  (was b_byte1 on entry)
//   __rc5    B_M3    b's high mantissa byte (implicit 1 in bit 7 for normal)
//   __rc6    B_EXP   b's biased exponent
//   __rc7    B_SIGN  b's sign in bit 7 (0x00 or 0x80)
//   __rc8    B_STK   b's sticky bit (0 or nonzero)
//   __rc9    TMP     scratch used in align/pack/round
//   __rc10   A_M0    a's sub-LSB byte
//   __rc11   A_M1    a's low mantissa byte
//   __rc12   A_M2    a's mid mantissa byte
//   __rc13   A_M3    a's high mantissa byte
//   __rc14   A_EXP   a's biased exponent
//   __rc15   A_SIGN  a's sign in bit 7
//   __rc16   A_STK   a's sticky bit
//
// After the magnitude-compare swap, `a` always holds the operand with
// the larger absolute value.
//
// ============================================================================
// REFERENCE: MOS assembler gotchas
// ============================================================================
//
// 1. Local labels use the ".L" prefix.  Since this is a stand-alone .S
//    file (one assembler invocation), ".L" labels are truly file-local
//    and cannot collide with sibling .S files.  Numeric labels
//    (1:, 2:, ..., 32:) also work; each `Nf` refers to the next `N:`
//    forward.  Do not reuse the same numeric label multiple times in
//    the same function.
//
// 2. Out-of-range branch offsets are silently TRUNCATED to 8 bits.
//    A BEQ/BNE/BCC/BCS whose target is more than +127/-128 bytes away
//    will wrap to a totally different address and execute garbage.
//    For any conditional branch that reaches a far label we use the
//    invert-and-jump pattern:
//        bne .Lskip ; jmp .Lfar_target ; .Lskip:
//    OR, when several far branches share a common target, we place a
//    small trampoline (jmp .Lfar_target) within byte range of all of
//    them and branch to the trampoline.  See .L__addsf3_tramp_pack_a.
//
// 3. Even-numbered __rcN slots are declared with PROVIDE() in the
//    linker script, so a platform can reorder them.  The only
//    guaranteed-contiguous groups are pointer pairs: (rc0,rc1),
//    (rc2,rc3), (rc4,rc5), ....  We therefore access every named slot
//    by name and never use zp,x indexed addressing over the __rcN range.
//
// ============================================================================
// REFERENCE: algorithm phases
// ============================================================================
//
//   1. UNPACK A: split A/X/rc2/rc3 into sign/exp/mantissa/sticky in
//      __rc10..__rc16.  Restore implicit-1 bit in m3 for normals.
//   2. UNPACK B: same for b's bytes in rc4..rc7, target __rc2..__rc8.
//   3. COMPARE + SWAP: compare |a| vs |b|; if a < b, swap the five
//      unpacked fields so a becomes the larger.
//   4. INF/NAN DISPATCH: classify special values.
//   5. ZERO DISPATCH: signed-zero rule for a == 0 (implies b == 0).
//   6. ALIGN: shift b's mantissa right by (a.exp - b.exp).  Whole-byte
//      fast path for shifts >= 8, bit loop for residual.  Accumulate
//      sticky in B_STK.
//   7. ADD or SUBTRACT: same-sign -> ADC chain; opposite-sign -> SBC
//      chain (borrow cannot propagate out of m3 because |a| >= |b|).
//      On the subtract path, when B_STK is set we ripple-decrement the
//      tuple by 1 and force A_STK=1 to get the correct sticky sign.
//   8. NORMALIZE (subtract only): shift left until bit 7 of m3 is set;
//      stop AT exp==1 (crossing into subnormal without shifting)
//      because subnormals share the same reference scale as the
//      smallest normal.  Not stopping there silently doubles subnormal
//      results.
//   9. SUBNORMAL FIX (add path only): if arithmetic produced m3 bit 7
//      set with exp==0 (two subnormals added to a normal), bump exp
//      to 1.
//  10. OVERFLOW: exp >= 0xFF -> signed infinity.
//  11. ROUND: nearest-ties-to-even using round/guard/sticky in m0/stk;
//      ripple-increment m1..m3; bump exp on all-1s wrap; overflow to
//      infinity if that pushes exp past 0xFE.
//  12. PACK: reassemble result bytes back into A, X, __rc2, __rc3.
//
//===----------------------------------------------------------------------===//

#include "imag-regs-zp.inc"

#define B_M0  __rc2
#define B_M1  __rc3
#define B_M2  __rc4
#define B_M3  __rc5
#define B_EXP  __rc6
#define B_SIGN  __rc7
#define B_STK  __rc8
#define A_M0  __rc10
#define A_M1  __rc11
#define A_M2  __rc12
#define A_M3  __rc13
#define A_EXP  __rc14
#define A_SIGN  __rc15
#define A_STK  __rc16
; __rc17..__rc18 are free; align scratch reuses __rc9 (TMP).
#define TMP  __rc9


.text
.globl __addsf3
.type __addsf3, @function
__addsf3:
        ; ================================================================
        ; Phase 1 -- UNPACK A: A/X/__rc2/__rc3 (a's incoming bytes) into
        ; our working slots __rc10..__rc16.  See the "IEEE 754 binary32
        ; memory layout" reference at top of file.

        ; On entry: A = a_byte0 (low mantissa), X = a_byte1,
        ; __rc2 = a_byte2 (E0|MMMMMMM), __rc3 = a_byte3 (S|EEEEEEE)

        ; A_M1 and A_M2 come straight from A and X.
        ; A_EXP uses the ASL/ROL trick: ASL __rc2 puts byte2's bit 7 (E0)
        ; in C, then ROL A on byte3 gives (byte3<<1)|C = 8-bit biased exp.
        ; A_SIGN is byte3's bit 7 preserved as 0x00 or 0x80.
        ; A_M3 is byte2's low 7 mantissa bits, with the implicit leading 1
        ; restored (bit 7 set) iff we're normal (exp != 0).
        ; A_M0 and A_STK start at 0 (no sub-LSB bits, no sticky yet).
        ; ================================================================
        sta A_M1   ; A_M1 = byte0
        stx A_M2   ; A_M2 = byte1

        ; Extract biased exponent from (byte3<<1)|(byte2>>7)
        lda __rc2   ; A = byte2
        asl a   ; C = byte2[7] = E0
        lda __rc3   ; A = byte3, C preserved
        rol a   ; A = (byte3<<1)|C = biased exp
        sta A_EXP

        ; Extract sign bit (byte3 & 0x80)
        lda __rc3
        and #$80
        sta A_SIGN

        ; Restore mantissa MSB with implicit leading 1 for normals:
        ; A_M3 = (byte2 & 0x7f) | (exp!=0 ? 0x80 : 0)
        lda __rc2
        and #$7f   ; strip E0
        ldy A_EXP   ; check exp
        beq 1f   ; exp==0 -> subnormal, no bit 7
        ora #$80   ; exp!=0 -> set implicit 1
        1: sta A_M3

        ; Initialise sub-LSB byte and sticky.
        lda #0
        sta A_M0
        sta A_STK

        ; ================================================================
        ; Phase 2 -- UNPACK B: __rc4..__rc7 (b's incoming bytes) into
        ; __rc2..__rc8.  Same shape as unpack A.  We can safely overwrite
        ; __rc2/__rc3 now because a's data has been fully extracted.

        ; Order matters: we read __rc4/__rc5 (b_byte0/1) before writing
        ; to B_M1 (=__rc3), which happens to be safe because we're
        ; writing to __rc3 and reading from __rc4/__rc5 which are
        ; preserved.
        ; ================================================================
        lda __rc4   ; b_byte0
        sta B_M1
        lda __rc5   ; b_byte1
        sta B_M2

        ; Biased exp via the ASL/ROL trick, stashed in Y temporarily
        ; so we can consult it while computing B_M3.
        lda __rc6
        asl a
        lda __rc7
        rol a
        tay   ; Y = b_exp

        lda __rc6
        and #$7f
        cpy #0
        beq 2f
        ora #$80
        2: sta B_M3
        sty B_EXP

        lda __rc7
        and #$80
        sta B_SIGN

        lda #0
        sta B_M0
        sta B_STK

        ; ================================================================
        ; COMPARE |a| vs |b|; swap if |a| < |b|
        ; ================================================================
        ; ================================================================
        ; Phase 3 -- MAGNITUDE COMPARE and SWAP.

        ; We want |a| >= |b| after this phase, so the arithmetic can
        ; freely assume a is the "larger" operand (align always shifts
        ; b, subtract never underflows a, special-case dispatch only
        ; checks a's classification).

        ; Since sign is stored separately, |a| = a's exp:m3:m2:m1 as an
        ; unsigned tuple.  IEEE 754's clever bit layout makes this a
        ; straight byte-wise lexicographic compare from high (exp) to
        ; low (m1) -- m0 and stk are both 0 for both operands at this
        ; point so we can ignore them.

        ; BCC/BNE cascade: at each level, BCC on "a<b at this byte"
        ; jumps to swap; BNE on "a>b at this byte" jumps past swap.
        ; Only equality falls through to the next-lower byte.
        ; ================================================================
        lda A_EXP
        cmp B_EXP
        bcc .L__addsf3_do_swap
        bne .L__addsf3_no_swap
        lda A_M3
        cmp B_M3
        bcc .L__addsf3_do_swap
        bne .L__addsf3_no_swap
        lda A_M2
        cmp B_M2
        bcc .L__addsf3_do_swap
        bne .L__addsf3_no_swap
        lda A_M1
        cmp B_M1
        bcs .L__addsf3_no_swap

        .L__addsf3_do_swap:
        ; Swap 5 fields (m1, m2, m3, exp, sign).  MUST be unrolled --
        ; we cannot use zp,x indexed addressing over the __rcN range
        ; because only pair members rc(2N)/rc(2N+1) are guaranteed
        ; contiguous; the platform's imag-regs.ld may reorder even-rc
        ; slots between pairs.  See llvm-mos-sdk .../lib/imag-regs.ld.
        ; m0 and stk are known 0 for both operands here, so no need to
        ; swap them.
        ldx A_M1
        lda B_M1
        sta A_M1
        stx B_M1
        ldx A_M2
        lda B_M2
        sta A_M2
        stx B_M2
        ldx A_M3
        lda B_M3
        sta A_M3
        stx B_M3
        ldx A_EXP
        lda B_EXP
        sta A_EXP
        stx B_EXP
        ldx A_SIGN
        lda B_SIGN
        sta A_SIGN
        stx B_SIGN

        .L__addsf3_no_swap:

        ; ================================================================
        ; Phase 4 -- INF/NAN dispatch.

        ; Exp == 0xFF marks a special value.  Since |a| >= |b|, we only
        ; need to check a first; b's classification only matters in
        ; sub-cases (b is also NaN, or inf+inf with opposite signs).

        ; Distinguishing NaN vs inf: mantissa != 0 -> NaN, mantissa == 0
        ; -> inf.  We inline the (m3&0x7f)|m2|m1 OR check at each site
        ; rather than precomputing it globally, because common-path
        ; inputs (exp in [1,254]) never need it.

        ; IEEE 754 propagates NaN: NaN + anything = NaN.  We "quiet" the
        ; NaN by setting bit 22 (which is bit 6 of m3, or 0x40 in
        ; "high byte + implicit 1 restored" form).
        ; ================================================================
        lda A_EXP
        cmp #$ff
        bne .L__addsf3_not_infnan

        ; a_exp == 0xff.  Is a a NaN or an inf?
        lda A_M3
        and #$7f
        ora A_M2
        ora A_M1
        beq .L__addsf3_a_is_inf
        ; a is NaN -> quiet and return a
        lda A_M3
        ora #$40
        sta A_M3
        jmp .L__addsf3_pack_a

        .L__addsf3_a_is_inf:
        ; a is +/- inf.  If b is finite, result is a.
        lda B_EXP
        cmp #$ff
        ; .L__addsf3_pack_a is > 127 bytes away and the assembler silently
        ; truncates out-of-range branch offsets, so instead of a bare
        ; "bne .L__addsf3_pack_a" we branch to the .L__addsf3_tramp_pack_a trampoline
        ; placed just a few instructions below (in byte range).
        bne .L__addsf3_tramp_pack_a
        ; b is also inf or NaN -- classify with an inline OR check.
        lda B_M3
        and #$7f
        ora B_M2
        ora B_M1
        beq .L__addsf3_both_inf
        ; b is NaN -> quiet b and return b.  The result flows through
        ; the pack_b_to_a prologue which copies b's fields into a's
        ; slots then falls into the shared pack_a exit.
        lda B_M3
        ora #$40
        sta B_M3
        jmp .L__addsf3_pack_b_to_a

        ; Shared trampoline: both .L__addsf3_a_is_inf and .L__addsf3_both_inf branch here
        ; to reach .L__addsf3_pack_a (which is >127 bytes below, past all the
        ; arithmetic phases).  Position matters -- it must be within
        ; byte range of every branch that targets it.  Both callers
        ; above are within ~20 bytes, so this location is fine.
        .L__addsf3_tramp_pack_a:
        jmp .L__addsf3_pack_a

        .L__addsf3_both_inf:
        ; Both operands are +/- inf.  Same sign -> return that inf.
        ; Different signs -> +inf + -inf is IEEE-undefined and returns
        ; a qNaN (canonical 0x7fc00000).
        lda A_SIGN
        cmp B_SIGN
        beq .L__addsf3_tramp_pack_a   ; same sign -> return a
        ; Different signs: build 0x7fc00000 directly into the ABI return
        ; slots and RTS.  byte0=0 (in A), byte1=0 (in X), byte2=0xc0
        ; (in __rc2), byte3=0x7f (in __rc3).
        lda #$7f
        sta __rc3
        lda #$c0
        sta __rc2
        ldx #0
        lda #0
        rts

        .L__addsf3_not_infnan:
        ; ================================================================
        ; Phase 5 -- ZERO dispatch.

        ; Since |a| >= |b|, if a is zero then b must also be zero.
        ; IEEE 754 signed-zero rule for addition:
        ; +0 + +0 = +0        -0 + -0 = -0
        ; +0 + -0 = +0        -0 + +0 = +0
        ; Equivalently: result sign = a_sign AND b_sign (bit 7 semantic).

        ; If a has exp==0 with a nonzero mantissa, it's a subnormal
        ; (not zero), and we fall through to the normal arithmetic path.
        ; ================================================================
        lda A_EXP
        bne .L__addsf3_not_zero
        ; a_exp == 0 -- is it a zero (all mantissa bits 0) or subnormal?
        lda A_M3
        and #$7f
        ora A_M2
        ora A_M1
        bne .L__addsf3_not_zero   ; subnormal -> normal path
        ; a is +/- 0.  b is also +/- 0.  Apply signed-zero rule to b's
        ; sign in place, then dispatch through pack_b_to_a.
        lda B_SIGN
        and A_SIGN
        sta B_SIGN
        jmp .L__addsf3_pack_b_to_a

        .L__addsf3_not_zero:
        ; ================================================================
        ; Phase 6 -- ALIGN b's mantissa to a's exponent.

        ; Shift count = a_effective_exp - b_effective_exp, where the
        ; "effective exp" of a subnormal (biased exp = 0) is 1 (since
        ; the smallest normal has biased exp 1 and represents 2^-126,
        ; and a subnormal represents the same power range with the
        ; implicit-1 removed).

        ; Cases:
        ; both normal:                shift = a_exp - b_exp
        ; a normal, b subnormal:      shift = a_exp - 1
        ; both subnormal (a_exp==0):  shift = 0  (early exit)
        ; a subnormal, b normal:      impossible after swap (|a|>=|b|)

        ; Do the a_exp-first check to skip everything for the "both
        ; subnormal" case (no shift needed).  Otherwise SBC on B_EXP
        ; gives a_exp - b_exp; the correction "-1 more if b was 0" is a
        ; single SBC #1 whose C-input is still set from the previous
        ; SBC (which cannot have borrowed because a >= b).
        ; ================================================================
        lda A_EXP
        beq .L__addsf3_align_done   ; both subnormal, no shift
        sec
        sbc B_EXP   ; A = a_exp - b_exp
        ; If b_exp was 0, we need one more decrement (want a_exp - 1).
        ; C is still set from the SBC above (a>=b -> no borrow).
        ldy B_EXP
        bne 7f
        sbc #1
        7: beq .L__addsf3_align_done   ; shift == 0, no work

        ; If shift count is >= 32, b's entire 4-byte mantissa is discarded
        ; in one shot -- much cheaper than iterating the bit loop 32+ times.
        ; Sticky bit reflects whether any bit was truncated: 1 iff b's
        ; pre-shift mantissa was nonzero, 0 if b was exactly 0.  (The
        ; zero-dispatch above only checks a; it is legal for b to be 0
        ; here.  See the subtract-path sticky ripple below -- it MUST
        ; NOT fire when no bits were actually truncated, because
        ; ripple-decrementing a's tuple in that case corrupts an
        ; otherwise-exact result.)
        cmp #32
        bcc .L__addsf3_align_byte_check
        lda B_M3
        ora B_M2
        ora B_M1
        ora B_M0
        sta TMP                              ; TMP = "b was nonzero" flag
        lda #0
        sta B_M0
        sta B_M1
        sta B_M2
        sta B_M3
        sta B_STK                            ; default: no bits truncated
        lda TMP
        beq .L__addsf3_align_done            ; b was 0, STK stays 0
        lda #1
        sta B_STK
        jmp .L__addsf3_align_done

        ; Whole-byte fast path: while count >= 8, shift the tuple down by
        ; 8 bits with a byte-shuffle (m0 <- m1, m1 <- m2, m2 <- m3, m3 <- 0)
        ; and decrement count by 8.  Anything that would have shifted out
        ; of m0 becomes sticky (we OR in the old m0's whole byte because
        ; sticky is "any bit was 1 anywhere below the LSB" -- exact value
        ; doesn't matter, only nonzero-ness).

        ; Count is parked in Y across the shuffle (which needs A for the
        ; byte moves).  Y is dead here -- the align entry test finished
        ; with it, and the bit loop below reloads it via TAY.
        .L__addsf3_align_byte_check:
        cmp #8
        bcc .L__addsf3_align_bit_setup
        sec
        sbc #8
        tay   ; park residual count
        lda B_M0
        beq 11f
        sta B_STK   ; any nonzero suffices for sticky
        11: lda B_M1
        sta B_M0
        lda B_M2
        sta B_M1
        lda B_M3
        sta B_M2
        lda #0
        sta B_M3
        tya
        jmp .L__addsf3_align_byte_check

        ; Bit loop for the residual 1..7 shifts.  After the LSR/ROR chain,
        ; C holds the bit that just fell off m0's LSB -- if set, mark
        ; sticky.  Y is the loop counter (DEY sets Z, BNE loops).
        .L__addsf3_align_bit_setup:
        tay
        beq .L__addsf3_align_done   ; residual == 0, done
        .L__addsf3_align_bit_loop:
        lsr B_M3
        ror B_M2
        ror B_M1
        ror B_M0
        bcc 12f   ; C = bit that fell off m0[0]
        lda #1
        sta B_STK
        12: dey
        bne .L__addsf3_align_bit_loop

        .L__addsf3_align_done:

        ; ================================================================
        ; Phase 7 -- ADD or SUBTRACT the aligned mantissas.

        ; Same-sign path is a straight 4-byte ADC chain over the
        ; [m3:m2:m1:m0] tuple.  A carry out of m3 means the sum grew
        ; past 2.0; we shift the whole tuple right by 1 and increment
        ; the exponent to keep the mantissa in [1.0, 2.0) form.

        ; Opposite-sign path is a 4-byte SBC chain; since a is the
        ; larger operand (post-swap), no borrow can propagate out of
        ; m3.  A result of exactly zero means exact cancellation -> +0.
        ; Otherwise we normalize (shift left until m3 bit 7 is set) to
        ; restore the leading-1 form.

        ; In both paths we OR b's sticky bit into a's sticky.
        ; ================================================================
        lda A_SIGN
        cmp B_SIGN
        bne .L__addsf3_do_subtract

        ; ---- SAME-SIGN ADD ----
        ; Straight ADC chain, byte-by-byte, low to high.
        clc
        lda A_M0
        adc B_M0
        sta A_M0
        lda A_M1
        adc B_M1
        sta A_M1
        lda A_M2
        adc B_M2
        sta A_M2
        lda A_M3
        adc B_M3
        sta A_M3
        bcc .L__addsf3_add_no_carry
        ; Carry out of the top: the sum has exceeded 24-bit mantissa
        ; space (there's now a virtual bit 24 = the carry).  Shift the
        ; whole tuple right by 1 to bring bit 24 back to bit 23, then
        ; OR in the implicit-1 marker (bit 7 of m3), and bump exp.
        ; C after the LSR/ROR chain is the bit that fell off m0 -- OR
        ; into sticky.
        lsr A_M3
        ror A_M2
        ror A_M1
        ror A_M0
        bcc 13f
        lda #1
        sta A_STK
        13: lda A_M3
        ora #$80
        sta A_M3
        inc A_EXP
        .L__addsf3_add_no_carry:
        ; Merge b's sticky into a's sticky (as a plain OR: any nonzero
        ; b-stk makes a-stk nonzero).
        lda B_STK
        beq .L__addsf3_skip_stk_or
        lda #1
        sta A_STK
        .L__addsf3_skip_stk_or:
        jmp .L__addsf3_after_arith

        .L__addsf3_do_subtract:
        ; ---- OPPOSITE-SIGN SUBTRACT ----
        ; Straight SBC chain low-to-high.  Since |a|>=|b| post-swap,
        ; borrow cannot propagate out of m3.  Result sign is a's sign.
        sec
        lda A_M0
        sbc B_M0
        sta A_M0
        lda A_M1
        sbc B_M1
        sta A_M1
        lda A_M2
        sbc B_M2
        sta A_M2
        lda A_M3
        sbc B_M3
        sta A_M3
        ; Bug fix: on the SUBTRACT path, a nonzero B_STK means b's true
        ; value is slightly larger than the truncated bytes we subtracted,
        ; so the computed difference is slightly TOO LARGE by an amount
        ; strictly less than one ULP.  Correct rounding requires biasing
        ; DOWN -- the mirror image of the add-path convention where a
        ; nonzero sticky biases up.  We encode that here by ripple-
        ; decrementing the [A_M3:A_M2:A_M1:A_M0] tuple by 1 and setting
        ; A_STK unconditionally, so the true value is representable as
        ; "(computed - 1 ULP) plus a strictly-positive inexact remainder"
        ; -- which is exactly what the standard add-path rounding logic
        ; below wants to see.
        ;
        ; The ripple decrement cannot underflow an all-zero tuple:
        ; whenever B_STK is set, alignment shifted b (logical right
        ; shift zero-fills), so B_M3 bit 7 is provably 0 post-shift;
        ; and A had exp != 0 (else the zero-dispatch above fired), so
        ; A_M3 bit 7 is provably 1 pre-sub.  Therefore the raw SBC
        ; result has bit 7 set (or the sub required a borrow-in that
        ; already reduced the higher byte), so the tuple is >= 1 before
        ; decrement.
        lda B_STK
        beq .L__addsf3_sub_stk_done
        dec A_M0
        lda A_M0
        cmp #$ff
        bne .L__addsf3_sub_stk_set
        dec A_M1
        lda A_M1
        cmp #$ff
        bne .L__addsf3_sub_stk_set
        dec A_M2
        lda A_M2
        cmp #$ff
        bne .L__addsf3_sub_stk_set
        dec A_M3
        .L__addsf3_sub_stk_set:
        lda #1
        sta A_STK
        .L__addsf3_sub_stk_done:

        ; Exact cancellation (whole tuple + sticky all zero) -> +0.
        ; Per IEEE 754, "correctly-rounded x - x" returns +0 for any
        ; finite x under round-to-nearest.
        lda A_M3
        ora A_M2
        ora A_M1
        ora A_M0
        ora A_STK
        bne .L__addsf3_do_normalize
        lda #0
        sta __rc2
        sta __rc3
        ldx #0
        lda #0
        rts

        .L__addsf3_do_normalize:
        ; ================================================================
        ; Phase 8 -- NORMALIZE (subtract path only).

        ; After cancellation, m3's bit 7 may no longer be set.  Shift
        ; the tuple left until it is, decrementing exp accordingly.  If
        ; exp reaches 0 first, the result is subnormal (or exact
        ; cancellation, already handled).

        ; Whole-byte fast path is taken when m3 == 0 (all bits) AND exp
        ; is high enough to afford an 8-bit shift.  Each iteration
        ; shifts everything up by a byte and decrements exp by 8.  When
        ; m3 has any bit set we bail to the bit loop (which will take
        ; at most 7 iterations to place bit 7).
        ; ================================================================
        .L__addsf3_norm_byte_check:
        bit A_M3   ; N flag <- bit 7 of A_M3
        bmi .L__addsf3_norm_done   ; bit 7 set -> normalized
        lda A_EXP
        cmp #9
        bcc .L__addsf3_norm_bit_loop   ; exp < 9: byte-shift would
                                             ; drop exp below 1; use bits
        lda A_M3
        bne .L__addsf3_norm_bit_loop   ; m3 has bits (below bit 7):
                                             ; bit shift will finish faster
        ; Whole-byte shift left:
        ; m3 <- m2, m2 <- m1, m1 <- m0, m0 <- (stk ? 0x80 : 0)
        ; The sticky-bit-becomes-new-m0-bit-7 rule is because sticky
        ; represents "some non-zero bit was below m0"; when we shift
        ; that region up into m0, it collapses to a single bit at the
        ; topmost position (bit 7).
        lda A_M2
        sta A_M3
        lda A_M1
        sta A_M2
        lda A_M0
        sta A_M1
        lda A_STK
        beq 14f
        lda #$80
        14: sta A_M0
        lda #0
        sta A_STK
        ; exp -= 8
        lda A_EXP
        sec
        sbc #8
        sta A_EXP
        jmp .L__addsf3_norm_byte_check

        ; Bit loop: shift left 1 per iteration, folding sticky bit into
        ; the new m0 LSB via LSR of stk into C, then ROL through the
        ; tuple.  After the ROL chain, stk is fully consumed (set to 0).
        .L__addsf3_norm_bit_loop:
        bit A_M3
        bmi .L__addsf3_norm_done
        lda A_EXP
        beq .L__addsf3_norm_done   ; already subnormal -- stop
        ; Bug fix: exp==1 is the transition into subnormal representation,
        ; not one more normal-to-normal renormalization step.  Normal
        ; exponents E and E-1 (both >=1) share a "shift mantissa left 1,
        ; decrement exp" invariant that preserves value; that invariant
        ; does NOT extend to E=1 -> E=0, because IEEE 754 subnormals
        ; (biased exp field 0) reuse the SAME reference scale as the
        ; smallest normal (biased exp field 1) -- they just drop the
        ; implicit leading 1.  So at exp==1 with bit 7 still clear, the
        ; CURRENT (unshifted) mantissa is already the correct subnormal
        ; encoding: relabel exp to 0 and stop, without shifting.
        ; Shifting here as if crossing a normal exponent boundary
        ; silently doubles the result.
        cmp #1
        beq .L__addsf3_norm_to_subnormal
        lda A_STK
        lsr a   ; C <- stk bit 0
        rol A_M0   ; shift with sticky into new m0[0]
        rol A_M1
        rol A_M2
        rol A_M3
        lda #0
        sta A_STK
        dec A_EXP
        jmp .L__addsf3_norm_bit_loop

        .L__addsf3_norm_to_subnormal:
        lda #0
        sta A_EXP
        ; fall through

        .L__addsf3_norm_done:
        ; (The old "NORMALIZE OVERSHOOT CORRECTION" block that sat here
        ; is unreachable now that the bit-loop exits cleanly at exp==1
        ; via .L__addsf3_norm_to_subnormal without a final shift, so
        ; "exp==0 AND m3 bit 7 set" cannot arise on the subtract path.
        ; Fall straight into the shared after-arith path.)

        .L__addsf3_after_arith:
        ; ================================================================
        ; Phase 9 -- SUBNORMAL-GREW-INTO-NORMAL FIX (add path only).

        ; Two subnormal operands can add to a value >= 2^-126 (smallest
        ; normal); the mantissa's implicit-1 bit (m3 bit 7) is set but
        ; exp is still 0.  IEEE 754 requires that be encoded with exp=1.
        ; Pack expects exp>=1 when m3 bit 7 is set, so bump exp here.

        ; The subtract path can't reach this state: its overshoot
        ; correction above shifts the tuple such that m3 bit 7 is
        ; guaranteed clear when exp is 0.
        ; ================================================================
        lda A_EXP
        bne .L__addsf3_skip_subnorm_fix   ; exp != 0, nothing to do
        bit A_M3
        bpl .L__addsf3_skip_subnorm_fix   ; m3 bit 7 clear -> true subnormal
        lda #1
        sta A_EXP
        .L__addsf3_skip_subnorm_fix:

        ; ================================================================
        ; Phase 10 -- OVERFLOW -> signed infinity.

        ; exp >= 0xFF means we've saturated.  Return +/- inf with a's
        ; sign.  Encoding: byte3 = sign|0x7F, byte2 = 0x80, low bytes 0.
        ; ================================================================
        lda A_EXP
        cmp #$ff
        bcc .L__addsf3_do_round

        .L__addsf3_return_inf:
        lda #$80
        sta __rc2
        lda A_SIGN
        ora #$7f
        sta __rc3
        ldx #0
        lda #0
        rts

        .L__addsf3_do_round:
        ; ================================================================
        ; Phase 11 -- ROUND to nearest, ties to even.

        ; Our round + guard + sticky bits are packed in m0 and stk:
        ; R = m0 bit 7      (round bit; nearest tie boundary)
        ; G = m0 bit 6      (guard; disambiguates > half from == half)
        ; S = (m0 & 0x3f) != 0  ||  stk != 0
        ; LSB = m1 bit 0    (destination LSB; for ties-to-even)

        ; Round-up condition: R && (G || S || LSB).
        ; - R=0: below half, round down (no increment).
        ; - R=1, G|S=1: strictly above half, round up.
        ; - R=1, G=0, S=0: exactly half, tie to even -> round up iff
        ; LSB is odd (would make it even after +1).
        ; ================================================================
        bit A_M0
        bpl .L__addsf3_pack_a   ; R bit clear -> no round

        ; Compute (G | S | LSB) via ORs; branch to pack_a if all zero
        ; (exact tie with even LSB -> no round).
        lda A_M0
        and #$7f   ; G bit + partial sticky bits
        ora A_STK   ; fold in accumulated sticky
        sta TMP
        lda A_M1
        and #1   ; LSB for tie-to-even
        ora TMP
        beq .L__addsf3_pack_a   ; exact tie, LSB even -> no round

        ; Round up: ripple-increment the 24-bit mantissa.  If it wraps
        ; all the way to zero, that means m1..m3 were 0xFFFFFF and the
        ; rounded value is exactly 2^(exp+1) -- restore m3 bit 7 and
        ; bump exp (which may overflow to infinity).
        inc A_M1
        bne .L__addsf3_pack_a
        inc A_M2
        bne .L__addsf3_pack_a
        inc A_M3
        bne .L__addsf3_pack_a
        lda #$80
        sta A_M3
        inc A_EXP
        lda A_EXP
        cmp #$ff
        bcs .L__addsf3_return_inf
        jmp .L__addsf3_pack_a                     ; skip pack_b_to_a fall-through

        ; ================================================================
        ; Phase 12a -- PACK B_TO_A prologue.

        ; The "b is NaN quiet" and "both zero" paths want to return b's
        ; value.  Rather than maintain a duplicate packer for b's slots,
        ; copy b's five fields into a's slots and fall through into the
        ; single shared .L__addsf3_pack_a exit.  Costs 5 load/store pairs (~30
        ; cycles on rare paths) in exchange for ~35 bytes of code we'd
        ; otherwise emit twice.
        ; ================================================================
        .L__addsf3_pack_b_to_a:
        lda B_M1
        sta A_M1
        lda B_M2
        sta A_M2
        lda B_M3
        sta A_M3
        lda B_EXP
        sta A_EXP
        lda B_SIGN
        sta A_SIGN
        ; fall through

        ; ================================================================
        ; Phase 12b -- PACK a into the ABI return position.

        ; Return convention: byte0 in A, byte1 in X, byte2 in __rc2,
        ; byte3 in __rc3.  We reconstruct the IEEE 754 layout by
        ; splitting the biased exponent between byte3's low 7 bits and
        ; byte2's high bit, and dropping the implicit-1 marker from m3.

        ; byte3 = sign | (exp >> 1)          (S | E7..E1)
        ; byte2 = (m3 & 0x7f) | ((exp & 1) << 7)  (E0 | M22..M16)
        ; byte1 = m2                          (M15..M8)
        ; byte0 = m1                          (M7..M0)

        ; Trick: LSR of exp puts the low bit in C.  That C is preserved
        ; through LDA/AND (which don't touch C), so we can BCC/BCS on
        ; it after loading m3 and ANDing to strip its bit 7.
        ; ================================================================
        .L__addsf3_pack_a:
        lda A_EXP
        lsr a   ; A = exp>>1, C = exp bit 0 (E0)
        sta TMP   ; stash exp>>1 for byte3
        lda A_M3
        and #$7f   ; strip implicit-1 marker
        bcc .L__addsf3_pack_no_bit7   ; if E0 was 0, no bit 7
        ora #$80   ; if E0 was 1, set bit 7
        .L__addsf3_pack_no_bit7:
        sta __rc2   ; byte2 = E0|(m3&0x7f)
        lda TMP
        ora A_SIGN
        sta __rc3   ; byte3 = sign|(exp>>1)
        lda A_M1   ; byte0 -> A
        ldx A_M2   ; byte1 -> X
        rts
