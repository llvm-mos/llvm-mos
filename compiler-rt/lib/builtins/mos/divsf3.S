//===-- lib/mos/divsf3.S - Single-precision division (MOS 6502) -*- Asm -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
// ===----------------------------------------------------------------------===//
//
// __divsf3(a, b) -- IEEE 754 binary32 division for MOS (6502).
//
// Rounding: round-to-nearest-ties-to-even, hardcoded.  No fenv support.
//
// Same overall shape as addsf3.c and mulsf3.c: one naked top-level
// function with all state in caller-saved zero-page (__rc2..__rc19),
// .L-prefixed local labels, invert-and-jmp for far branches.  See
// addsf3.c for the reference material on IEEE 754 layout and the
// assembler gotchas that applies unchanged here.
//
// ============================================================================
// REFERENCE: division algorithm outline
// ============================================================================
//
// value(a/b) = (-1)^(sa^sb) * (Ma/Mb) * 2^((ea-eb))     for normal a,b
//
// Sign of result is the XOR of input signs.  Magnitude is a 24 x 24 ->
// 25-bit mantissa division plus an exponent subtract.  Since both input
// mantissas are in [2^23, 2^24) (implicit-1 restored), the mantissa
// ratio Ma/Mb is in (0.5, 2.0).  Two cases fall out:
//
// Case A (Ma >= Mb, ratio in [1, 2)):  the true (Ma * 2^24) / Mb
// quotient is a 25-bit value with bit 24 = 1.  The result mantissa
// (with implicit-1) is the top 24 bits of that quotient.
// Case B (Ma < Mb, ratio in [0.5, 1)):  quotient is a 24-bit value
// in [2^23, 2^24).  The result mantissa is those 24 bits directly,
// and the result exponent is decremented by 1 (to compensate for
// the [0.5, 1) → [1, 2) normalization).
//
// Which case applies is determined by a pre-loop 24-bit compare of
// Ma and Mb; the flag is stored in Q_HI (1 for case A, 0 for case B).
//
// The divide loop is a standard 6502 shift-subtract long division.
// Init: R = Ma; Q = 0.  For case A, we also pre-subtract D once from
// R (matching the "would-be first quotient bit = 1" of the 25-bit
// quotient) to establish the loop invariant "R < D".  Case B needs no
// pre-subtract because Ma < Mb already satisfies it.
//
// Then 24 iterations of:
//
// 1. Shift [R:Q] left by 1.  The bit that falls off R2's MSB
// (captured in the C flag as C_out) reflects whether pre-shift R
// had bit 23 set -- i.e. whether the "true" 25-bit post-shift R
// was >= 2^24 > D.
// 2. If C_out = 1:  force subtract D from the 24-bit R.  The 24-bit
// SBC's borrow "consumes" the 2^24 bit that shifted out, so the
// stored 24-bit R correctly reflects the 25-bit R minus D.
// 3. Else:  compare R with D (high byte first, early-out) and
// subtract in place only if R >= D.
// 4. If we subtracted (in either branch), set Q bit 0 = 1.
//
// A post-loop cleanup handles the exact-division boundary: if R >= D
// after 24 iterations, subtract D once more and increment Q.  (Without
// this, exact-divisible cases like Ma == Mb would end with R = D and
// Q one short.)
//
// After the loop, Q holds the 24-bit low bits of the true quotient
// (which is 24-bit in case B or the low 24 bits of a 25-bit value in
// case A):
//
// Case A: mantissa = 0x800000 | (Q >> 1); round bit = Q bit 0.
// Case B: mantissa = Q; round bit computed from a "virtual 25th
// iteration" (shift R left, compare/sub with D, round bit = 1 iff
// the virtual subtract commits).
//
// Sticky = (R != 0) after all shifts and the virtual iteration.
//
// Underflow (R_EXP <= 0) triggers a shift-right of the mantissa with
// careful round/sticky tracking:  the old round bit becomes part of
// the new sticky before each shift, and the bit that falls off each
// shift becomes the new round bit.  After (1 - R_EXP) shifts, the
// mantissa is in subnormal-encoding form and R_EXP is set to 0.
//
// ============================================================================
// REFERENCE: result exponent
// ============================================================================
//
// R_EXP = a_exp + (127 - b_exp)
// = (a_exp - b_exp) + 127
//
// For normal inputs (a_exp, b_exp in [1, 254]) this is in
// [1-254+127, 254-1+127] = [-126, 380].  Fits in signed 16-bit.
// Subnormals renormalize like in mulsf3 -- shift the significand left
// until bit 23 is set, adjust the effective exponent one per shift.
//
// After the divide loop's mantissa case-B branch (ratio < 1), R_EXP
// gets one more decrement.
//
// ============================================================================
// REFERENCE: special cases
// ============================================================================
//
// NaN / anything = quiet(a)
// anything / NaN = quiet(b)
// Inf / Inf      = qNaN
// Inf / finite   = signed Inf
// finite / Inf   = signed 0
// x / 0 (x != 0) = signed Inf     (IEEE 754 "divide by zero" -- we
// don't raise a flag, we just return)
// 0 / x (x != 0) = signed 0
// 0 / 0          = qNaN
//
// Sign of Inf and 0 in x/0 and 0/x follows the XOR of the operand signs.
//
// ============================================================================
// REFERENCE: zero-page layout
// ============================================================================
//
// Same input constraints as addsf3/mulsf3 (a in A/X/__rc2/__rc3;
// b in __rc4..__rc7).  Unpack transforms in place where possible.
//
// __rc2   A_M2      a mantissa top (implicit 1 in bit 7 for normals)
// __rc3   A_EXP     a's biased exponent
// __rc4   D0        b mantissa byte 0 (unchanged from input)
// __rc5   D1        b mantissa byte 1 (unchanged from input)
// __rc6   D2        b mantissa top (implicit 1 restored)
// __rc7   B_EXP     b's biased exponent
// __rc8   A_M0      a mantissa byte 0 (from register A)
// __rc9   A_M1      a mantissa byte 1 (from register X)
// __rc10  A_SIGN -> Q_HI     a's sign in bit 7; after dispatch reused
// as bit 24 of the quotient
// __rc11  B_SIGN -> TMP0     b's sign; after the divide loop reused as
// the sticky flag
// __rc12  R_SIGN            result sign in bit 7
// __rc13  R_EXP_LO          signed 16-bit result exponent, low byte
// __rc14  R_EXP_HI          ... high byte
// __rc15  R0                remainder byte 0
// __rc16  R1
// __rc17  R2
// __rc18  CNT               shift counter for the subnormal-result
//                           shift (the divide loop counts down in Y)
// __rc19  TMP1              round bit / pack scratch
//
// A_M0/A_M1/A_M2 double as Q0/Q1/Q2 during the divide loop (numerator
// is consumed as quotient bits shift up from below).
//
// ===----------------------------------------------------------------------===//


#include "imag-regs-zp.inc"

#define A_M2  __rc2
#define A_EXP  __rc3
#define D0  __rc4
#define D1  __rc5
#define D2  __rc6
#define B_EXP  __rc7
#define A_M0  __rc8
#define A_M1  __rc9
#define A_SIGN  __rc10
#define Q_HI  __rc10
#define B_SIGN  __rc11
#define TMP0  __rc11
#define R_SIGN  __rc12
#define R_EXP_LO  __rc13
#define R_EXP_HI  __rc14
#define R0  __rc15
#define R1  __rc16
#define R2  __rc17
#define CNT  __rc18
#define TMP1  __rc19


.text
.globl __divsf3
.type __divsf3, @function
__divsf3:
        // ================================================================
        // Phase 1 -- UNPACK A (identical to mulsf3's unpack A).
        // ================================================================
        sta A_M0
        stx A_M1

        lda __rc3
        and #$80
        sta A_SIGN

        lda __rc2
        asl a
        lda __rc3
        rol a
        sta A_EXP

        lda __rc2
        and #$7f
        ldx A_EXP
        beq 1f
        ora #$80
        1: sta A_M2

        // ================================================================
        // Phase 2 -- UNPACK B.
        // ================================================================
        lda __rc7
        and #$80
        sta B_SIGN

        lda __rc6
        asl a
        lda __rc7
        rol a
        sta B_EXP

        lda __rc6
        and #$7f
        ldx B_EXP
        beq 2f
        ora #$80
        2: sta D2

        // ================================================================
        // Phase 3 -- RESULT SIGN (XOR of input signs, in bit 7).
        // ================================================================
        lda A_SIGN
        eor B_SIGN
        sta R_SIGN

        // ================================================================
        // Phase 4 -- INF/NAN dispatch.

        // NaN / anything -> quiet(a) with a's sign
        // anything / NaN -> quiet(b) with b's sign
        // Inf / Inf      -> qNaN
        // Inf / finite   -> signed Inf
        // finite / Inf   -> signed 0
        // ================================================================
        lda A_EXP
        cmp #$ff
        bne .L__divsf3_chk_b_special

        // a's exp is 0xFF.  NaN or Inf?
        lda A_M2
        and #$7f
        ora A_M1
        ora A_M0
        beq .L__divsf3_a_is_inf

        // a is NaN -> quiet a, use a's sign.
        lda A_M2
        ora #$40
        sta A_M2
        lda A_SIGN
        sta R_SIGN
        jmp .L__divsf3_pack_a

        .L__divsf3_a_is_inf:
        // a is Inf.  Check b.
        lda B_EXP
        cmp #$ff
        beq .L__divsf3_a_inf_b_infnan
        // b is finite -> return signed Inf.
        jmp .L__divsf3_return_signed_inf

        .L__divsf3_a_inf_b_infnan:
        // b's exp is 0xFF.  NaN or Inf?
        lda D2
        and #$7f
        ora D1
        ora D0
        beq .L__divsf3_return_qnan   // Inf / Inf = qNaN
        // b is NaN -> return quiet(b).
        jmp .L__divsf3_return_quiet_b

        .L__divsf3_chk_b_special:
        // a is not special.  Is b special?
        lda B_EXP
        cmp #$ff
        bne .L__divsf3_chk_zero

        // b is Inf or NaN.
        lda D2
        and #$7f
        ora D1
        ora D0
        beq .L__divsf3_b_is_inf
        // b is NaN -> return quiet(b).
        .L__divsf3_return_quiet_b:
        lda D2
        ora #$40
        sta A_M2
        lda D1
        sta A_M1
        lda D0
        sta A_M0
        lda B_EXP
        sta A_EXP
        lda B_SIGN
        sta R_SIGN
        jmp .L__divsf3_pack_a

        .L__divsf3_b_is_inf:
        // b is Inf, a is finite.  Return signed 0.
        jmp .L__divsf3_return_zero

        .L__divsf3_return_signed_inf:
        lda #0
        sta A_M0
        sta A_M1
        lda #$80
        sta A_M2
        lda #$ff
        sta A_EXP
        jmp .L__divsf3_pack_a

        .L__divsf3_return_qnan:
        // Canonical qNaN 0x7fc00000.
        lda #$c0
        sta __rc2
        lda #$7f
        sta __rc3
        ldx #0
        lda #0
        rts

        // ================================================================
        // Phase 5 -- ZERO dispatch.

        // 0 / 0            -> qNaN
        // 0 / nonzero      -> signed 0
        // nonzero / 0      -> signed Inf
        // nonzero / nonzero -> normal path
        // ================================================================
        .L__divsf3_chk_zero:
        // Test a for zero: exp | m0 | m1 | m2 == 0.
        lda A_EXP
        ora A_M0
        ora A_M1
        ora A_M2
        sta TMP0   // TMP0 = 0 iff a is zero
        lda B_EXP
        ora D0
        ora D1
        ora D2
        sta TMP1   // TMP1 = 0 iff b is zero
        // Dispatch:
        lda TMP0
        bne .L__divsf3_a_nonzero
        // a is zero.
        lda TMP1
        beq .L__divsf3_return_qnan   // 0 / 0
        jmp .L__divsf3_return_zero   // 0 / nonzero

        .L__divsf3_a_nonzero:
        lda TMP1
        beq .L__divsf3_return_signed_inf   // nonzero / 0
        // Both nonzero, proceed to renormalize.

        // ================================================================
        // Phase 6 -- COMPUTE R_EXP AND RENORMALIZE SUBNORMALS.

        // R_EXP = a_exp - b_exp + 127  (16-bit signed).  For subnormals,
        // fold the "effective exp is 1-k where k is shift count" via
        // pre-decrement + one dec per shift, just like mulsf3.
        // ================================================================

        // R_EXP = a_exp + 127 (as unsigned 8-bit + high 0).
        clc
        lda A_EXP
        adc #127
        sta R_EXP_LO
        lda #0
        adc #0
        sta R_EXP_HI

        // R_EXP -= b_exp.
        sec
        lda R_EXP_LO
        sbc B_EXP
        sta R_EXP_LO
        lda R_EXP_HI
        sbc #0
        sta R_EXP_HI

        // If a subnormal (a_exp == 0): R_EXP += 1, then shift A_M left
        // until A_M2 bit 7 set, decrementing R_EXP per shift.
        lda A_EXP
        bne .L__divsf3_renorm_b
        inc R_EXP_LO
        bne 3f
        inc R_EXP_HI
        3:
        .L__divsf3_renorm_a_loop:
        bit A_M2
        bmi .L__divsf3_renorm_b
        asl A_M0
        rol A_M1
        rol A_M2
        lda R_EXP_LO
        bne 4f
        dec R_EXP_HI
        4: dec R_EXP_LO
        jmp .L__divsf3_renorm_a_loop

        .L__divsf3_renorm_b:
        // If b subnormal: R_EXP -= 1 (subtract 1 from effective b_exp),
        // then shift D left, INCREMENTING R_EXP per shift (because
        // R_EXP has b_exp SUBTRACTED, and shifting b's mantissa left
        // means b's effective exponent decreases, so R_EXP increases).
        lda B_EXP
        bne .L__divsf3_div_init
        // Subnormal b's effective exp is (1 - k), so R_EXP was computed
        // with too much subtracted (b_exp = 0 was used, should be
        // 1 - k).  Adjust: R_EXP -= 1 initially (so we're at
        // R_EXP - (-1 + k) = R_EXP + 1 - k conceptually), then INC per
        // shift.  Actually: subtracting b_exp=0 vs b_eff=1-k means
        // R_EXP is too HIGH by (1-k) - 0 = 1 - k.  Correct by decrementing
        // by (1-k) = 1 - k, i.e. R_EXP -= 1 then INC per shift.
        lda R_EXP_LO
        bne 5f
        dec R_EXP_HI
        5: dec R_EXP_LO
        .L__divsf3_renorm_b_loop:
        bit D2
        bmi .L__divsf3_div_init
        asl D0
        rol D1
        rol D2
        inc R_EXP_LO
        bne 6f
        inc R_EXP_HI
        6:
        jmp .L__divsf3_renorm_b_loop

        // ================================================================
        // Phase 7a -- DIVIDE LOOP INIT.

        // Init R = M_A (numerator in the "high half" of a 48-bit
        // dividend [R:Q] = M_A * 2^24), Q = 0.

        // 24 iterations of shift-subtract-with-force will produce the
        // 24-bit quotient in Q (low 24 bits of the true 24- or 25-bit
        // quotient).  The "would-be bit 24" (implicit-1 for the case
        // when M_A/M_B >= 1) is not captured in Q; it's detected via a
        // pre-loop compare stored in Q_HI.
        // ================================================================
        .L__divsf3_div_init:
        // Pre-loop compare M_A vs D (24-bit unsigned).  Store 1 in Q_HI
        // if M_A >= M_B (case A), 0 otherwise (case B).
        lda A_M2
        cmp D2
        bcc .L__divsf3_case_b_init
        bne .L__divsf3_case_a_init
        lda A_M1
        cmp D1
        bcc .L__divsf3_case_b_init
        bne .L__divsf3_case_a_init
        lda A_M0
        cmp D0
        bcc .L__divsf3_case_b_init

        .L__divsf3_case_a_init:
        lda #1
        sta Q_HI
        jmp .L__divsf3_div_setup

        .L__divsf3_case_b_init:
        lda #0
        sta Q_HI

        .L__divsf3_div_setup:
        // Move M_A into R (as the "high half" of the 48-bit dividend).
        // Zero out Q slots.
        lda A_M0
        sta R0
        lda A_M1
        sta R1
        lda A_M2
        sta R2
        lda #0
        sta A_M0   // Q0 = 0
        sta A_M1   // Q1 = 0
        sta A_M2   // Q2 = 0

        // For case A (M_A >= M_B), pre-subtract D from R to establish
        // the loop invariant "R < D".  The invariant is required for
        // the shift-subtract algorithm to produce binary quotient bits.
        // (The pre-subtract accounts for the "would-be bit 24 = 1" of
        // the true 25-bit quotient; we already recorded that in Q_HI.)
        lda Q_HI
        beq .L__divsf3_no_presub
        sec
        lda R0
        sbc D0
        sta R0
        lda R1
        sbc D1
        sta R1
        lda R2
        sbc D2
        sta R2
        .L__divsf3_no_presub:

        ldy #24

        // ================================================================
        // Phase 7b -- DIVIDE LOOP: 24 iterations of shift-subtract.

        // Layout: [R2:R1:R0:Q2:Q1:Q0] as a 48-bit shift register.
        // Each iteration:
        // 1. Shift left [R:Q] by 1.  Bit 23 of Q shifts into R bit 0.
        // The bit shifted out of R2's MSB (call it R_hi) reflects
        // whether the pre-shift R had bit 23 set -- i.e. whether
        // "true" post-shift R was >= 2^24 > D.
        // 2. If R_hi = 1: force subtract D from R (the borrow in the
        // 24-bit SBC exactly cancels the 2^24 bit we shifted out).
        // 3. Else: compare R with D and subtract in place only if R >= D.
        // 4. If we subtracted, set Q bit 0 = 1.
        // ================================================================
        .L__divsf3_div_loop:
        asl A_M0   // ASL Q0
        rol A_M1   // ROL Q1
        rol A_M2   // ROL Q2
        rol R0
        rol R1
        rol R2   // C_out = pre-shift R2 bit 7
        bcs .L__divsf3_div_do_sub

        // C_out = 0: decide with a 24-bit compare (high byte first, with
        // early-out), then subtract in place only if it commits.  This is
        // cheaper than subtracting into scratch and copying back: the
        // no-subtract half of the iterations costs three instructions
        // instead of a full 24-bit SBC chain plus two copies.
        lda R2
        cmp D2
        bcc .L__divsf3_div_no_sub
        bne .L__divsf3_div_do_sub
        lda R1
        cmp D1
        bcc .L__divsf3_div_no_sub
        bne .L__divsf3_div_do_sub
        lda R0
        cmp D0
        bcc .L__divsf3_div_no_sub

        .L__divsf3_div_do_sub:
        // Either C_out = 1 (true post-shift R was in [2^24, 2^25) > D, so
        // the subtract is forced -- the borrow out of the 24-bit SBC chain
        // exactly consumes the 2^24 bit that shifted out), or the compare
        // above proved R >= D.  Both cases subtract D from R in place and
        // set the quotient bit.
        sec
        lda R0
        sbc D0
        sta R0
        lda R1
        sbc D1
        sta R1
        lda R2
        sbc D2
        sta R2
        inc A_M0   // Q bit 0 = 1

        .L__divsf3_div_no_sub:
        dey
        bne .L__divsf3_div_loop

        // ================================================================
        // Phase 7c -- POST-LOOP CLEANUP.

        // The algorithm maintains the invariant Q * D + R = dividend,
        // but at the boundary case where the dividend is an exact
        // multiple of D, R can end at R = D (rather than R = 0 with
        // Q += 1).  Detect and correct: if R >= D, subtract D from R
        // and increment Q.
        // ================================================================
        // Compare R with D (24-bit high byte first).
        lda R2
        cmp D2
        bcc .L__divsf3_div_end
        bne .L__divsf3_div_cleanup
        lda R1
        cmp D1
        bcc .L__divsf3_div_end
        bne .L__divsf3_div_cleanup
        lda R0
        cmp D0
        bcc .L__divsf3_div_end

        .L__divsf3_div_cleanup:
        sec
        lda R0
        sbc D0
        sta R0
        lda R1
        sbc D1
        sta R1
        lda R2
        sbc D2
        sta R2
        // Q += 1 (ripple through Q0/Q1/Q2).
        inc A_M0
        bne .L__divsf3_div_end
        inc A_M1
        bne .L__divsf3_div_end
        inc A_M2

        .L__divsf3_div_end:

        // ================================================================
        // Phase 8 -- POST-LOOP MANTISSA EXTRACTION.

        // Q_HI bit 0 = 1 (Ma >= Mb, ratio in [1, 2)):
        // mantissa = 0x800000 | (Q >> 1)
        // round bit = Q bit 0
        // R_EXP unchanged

        // Q_HI bit 0 = 0 (Ma < Mb, ratio in [0.5, 1)):
        // mantissa = Q (bit 23 should already be set)
        // round bit = 0 (approximated -- see algorithm outline)
        // R_EXP -= 1

        // Sticky = (R != 0).

        // Store round bit in TMP1, sticky in TMP0 for the round phase.
        // ================================================================
        lda Q_HI
        and #1
        beq .L__divsf3_case_b

        // Case A: Q_HI bit 0 = 1.  Shift Q right by 1, capture bit 0 as
        // round bit, OR bit 23 into position (from Q_HI).
        // Round bit = Q0 bit 0 pre-shift.
        lda A_M0
        and #1
        sta TMP1   // TMP1 = round bit
        lsr A_M2
        ror A_M1
        ror A_M0
        lda A_M2
        ora #$80   // set implicit-1 at bit 23
        sta A_M2
        jmp .L__divsf3_compute_sticky

        .L__divsf3_case_b:
        // Case B: Q_HI bit 0 = 0.  Q is mantissa as-is.  Round bit
        // requires a "virtual 25th iteration": shift R left, then
        // compare with D.  If (2*R) >= D (or shift produced C_out=1),
        // round bit = 1 and new R = 2*R - D; else round = 0 and new
        // R = 2*R.  The updated R provides sticky info.  R_EXP -= 1.
        asl R0
        rol R1
        rol R2
        bcs .L__divsf3_round_b_do_sub   // C_out=1: true R > D, always sub
        // Compare 24-bit R with D.
        lda R2
        cmp D2
        bcc .L__divsf3_round_b_zero
        bne .L__divsf3_round_b_do_sub
        lda R1
        cmp D1
        bcc .L__divsf3_round_b_zero
        bne .L__divsf3_round_b_do_sub
        lda R0
        cmp D0
        bcc .L__divsf3_round_b_zero

        .L__divsf3_round_b_do_sub:
        sec
        lda R0
        sbc D0
        sta R0
        lda R1
        sbc D1
        sta R1
        lda R2
        sbc D2
        sta R2
        lda #1
        sta TMP1   // round bit = 1
        jmp .L__divsf3_case_b_exp_dec

        .L__divsf3_round_b_zero:
        lda #0
        sta TMP1   // round bit = 0

        .L__divsf3_case_b_exp_dec:
        // R_EXP -= 1.
        lda R_EXP_LO
        bne 7f
        dec R_EXP_HI
        7: dec R_EXP_LO

        .L__divsf3_compute_sticky:
        // Sticky = R != 0.
        lda R0
        ora R1
        ora R2
        beq 8f
        lda #1
        8: sta TMP0   // TMP0 = sticky (0 or 1)

        // ================================================================
        // Phase 9 -- OVERFLOW / UNDERFLOW dispatch (same as mulsf3).

        // R_EXP < 0            -> underflow (subnormal or zero)
        // R_EXP == 0           -> subnormal
        // R_EXP in [1, 0xFE]   -> normal, round
        // R_EXP >= 0xFF        -> overflow (signed Inf)
        // ================================================================
        lda R_EXP_HI
        bmi .L__divsf3_und
        bne .L__divsf3_ovf
        lda R_EXP_LO
        beq .L__divsf3_und
        cmp #$ff
        bcc .L__divsf3_round
        // fall through: overflow

        .L__divsf3_ovf:
        lda #$ff
        sta A_EXP
        lda #0
        sta A_M0
        sta A_M1
        lda #$80
        sta A_M2
        jmp .L__divsf3_pack_a

        .L__divsf3_und:
        // Subnormal result: shift mantissa right by (1 - R_EXP).

        // Rounding through the shift is tricky.  Correct semantics:
        // per iteration, the OLD round bit becomes part of new sticky
        // (moves down one position), the shifted-out mantissa bit
        // becomes the new round bit.  After K shifts, round = pre-shift
        // mantissa bit (K-1); sticky = OR of pre-shift mantissa bits
        // (K-2..0) | pre-shift round | pre-shift sticky.
        sec
        lda #1
        sbc R_EXP_LO
        cmp #32
        bcc 9f
        jmp .L__divsf3_return_zero
        9: sta CNT

        .L__divsf3_und_loop:
        lda CNT
        beq .L__divsf3_und_done
        // OLD round bit contributes to sticky.
        lda TMP1
        beq 10f
        lda #1
        sta TMP0
        10:
        // Shift mantissa right by 1.
        lsr A_M2
        ror A_M1
        ror A_M0
        // C from ROR A_M0 = bit that just fell off = new round bit.
        lda #0
        rol a   // A = 1 if C=1 else 0
        sta TMP1
        dec CNT
        jmp .L__divsf3_und_loop
        .L__divsf3_und_done:
        lda #0
        sta R_EXP_LO
        sta R_EXP_HI

        .L__divsf3_round:
        // ================================================================
        // Phase 10 -- ROUND: nearest, ties to even.

        // round bit  = TMP1
        // sticky     = TMP0
        // LSB        = A_M0 bit 0

        // Round up iff round && (sticky || LSB).
        // ================================================================
        lda TMP1
        beq .L__divsf3_no_round   // round bit clear -> no round
        // round bit set: round up iff (sticky || LSB).
        lda TMP0
        bne .L__divsf3_do_round   // sticky set -> round up
        lda A_M0
        and #1
        beq .L__divsf3_no_round   // LSB even, sticky 0 -> tie to even
        .L__divsf3_do_round:
        inc A_M0
        bne 11f
        inc A_M1
        bne 11f
        inc A_M2
        bne 11f
        // Full carry-out: mantissa wrapped 0xFFFFFF -> 0.  Set bit 7
        // (implicit) and bump exp.
        lda #$80
        sta A_M2
        inc R_EXP_LO
        bne 11f
        inc R_EXP_HI
        11:
        // Re-check overflow.
        lda R_EXP_HI
        bne .L__divsf3_ovf
        lda R_EXP_LO
        cmp #$ff
        bcc .L__divsf3_no_round
        jmp .L__divsf3_ovf

        .L__divsf3_no_round:
        // Subnormal-grew-into-normal fix (only relevant if we came
        // via the underflow path): if R_EXP == 0 but mantissa bit 23
        // set, bump R_EXP to 1.
        lda R_EXP_LO
        ora R_EXP_HI
        bne .L__divsf3_setup_pack
        bit A_M2
        bpl .L__divsf3_setup_pack
        lda #1
        sta R_EXP_LO

        .L__divsf3_setup_pack:
        lda R_EXP_LO
        sta A_EXP
        // fall through to .L__divsf3_pack_a

        // ================================================================
        // Phase 11 -- PACK a into ABI return position.

        // Same layout as addsf3/mulsf3 pack.
        // ================================================================
        .L__divsf3_pack_a:
        lda A_EXP
        lsr a
        sta TMP1
        lda A_M2
        and #$7f
        bcc 12f
        ora #$80
        12: sta __rc2
        lda TMP1
        ora R_SIGN
        sta __rc3
        lda A_M0
        ldx A_M1
        rts

        // ================================================================
        // Return signed zero.  R_SIGN was computed in Phase 3; the low
        // three bytes are 0 and byte 3 is R_SIGN.
        // ================================================================
        .L__divsf3_return_zero:
        lda #0
        sta __rc2
        lda R_SIGN
        sta __rc3
        ldx #0
        lda #0
        rts
