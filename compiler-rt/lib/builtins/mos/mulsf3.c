//===-- lib/mos/mulsf3.c - Single-precision multiplication (MOS) -*- C -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// __mulsf3(a, b) -- IEEE 754 binary32 multiplication for MOS (6502).
//
// Rounding: round-to-nearest-ties-to-even, hardcoded.  No fenv support
// (see addsf3.c for the rationale).
//
// This file follows the same structural pattern as addsf3.c: one naked
// top-level function containing one big asm volatile block, all state
// in caller-saved zero-page slots, .L-prefixed local labels, invert-and-
// jump for out-of-range branches.  See addsf3.c for the reference
// material on IEEE 754 layout, the MOS ABI, and assembler gotchas that
// applies verbatim here.
//
// ============================================================================
// REFERENCE: multiplication algorithm outline
// ============================================================================
//
// value(x)  = (-1)^s * 1.m * 2^(e-127)     for normal x
//           = (-1)^s * 0.m * 2^-126        for subnormal x
//
// The product's sign is the XOR of input signs.  For the magnitude, we
// need a 24-bit x 24-bit unsigned integer multiply of the "significand
// with implicit 1 restored" -- a value in [2^23, 2^24) for normals.  The
// full 48-bit product lands in [2^46, 2^48); its top bit distinguishes
// the two cases:
//
//   product bit 47 set:  product in [2^47, 2^48)  ->  quotient in [2,4)
//                        (exp adjustment +1, no bit shift)
//   product bit 47 clear (bit 46 set): product in [2^46, 2^47) -> [1,2)
//                        (no exp change, shift product left by 1)
//
// After that shift, bit 47 of the 48-bit product always holds the new
// implicit-1 bit, so we can pull mantissa/round/guard/sticky out of
// fixed positions regardless of which case applied.
//
// The result exponent is computed in signed 16-bit as
//
//   result_exp = eff_a + eff_b - 127
//
// where eff_a is a's biased exponent (or, for subnormals renormalized
// by shifting their significand left by k bits, 1 - k -- which can go
// as low as -22).  Range is roughly [-171, 381].  The +1 from the bit-47
// case is applied after the multiply.
//
// Subnormal renormalization: while significand bit 23 (M2 bit 7) is 0,
// shift the 24-bit significand left by 1 and decrement the effective
// exponent (which starts at 0 for a raw subnormal).  A raw subnormal has
// at least one bit set (the exp==0 && mantissa==0 case is a true zero
// handled by the zero-dispatch above), so the loop terminates.
//
// ============================================================================
// REFERENCE: the shift-and-add multiply core
// ============================================================================
//
// Classic 6502 right-shift multiply, extended to 24 x 24 -> 48.  Layout
// during the loop:
//
//   [PH2 : PH1 : PH0 : B_M2 : B_M1 : B_M0]        (48-bit shift register)
//     ^^^^^^^^^^^^^^^^^                              product high half
//                       ^^^^^^^^^^^^^^^^^^^         multiplier B; becomes
//                                                   product low half as
//                                                   B's bits are consumed
//
//   A_M2 : A_M1 : A_M0                             multiplicand A (fixed)
//
// Setup: PH2:PH1:PH0 = 0; multiplier already in B_M2:B_M1:B_M0.
//
// Pre-shift: LSR B_M2 / ROR B_M1 / ROR B_M0.  This puts B's LSB into C
// for the first iteration's decide, and shifts the multiplier once so
// that after 24 iterations the accumulator has traveled the full width.
//
// Loop body (repeated 24 times):
//
//   BCC skip           ; if the bit we just extracted was 0, no add
//   CLC
//   LDA PH0; ADC A_M0; STA PH0
//   LDA PH1; ADC A_M1; STA PH1
//   LDA PH2; ADC A_M2; STA PH2
//   ; C now = carry-out of the 24-bit add (i.e., accumulator bit 24)
// skip:
//   ROR PH2            ; incorporate C into bit 7 of PH2, shift right
//   ROR PH1            ; ripple; each ROR moves its LSB into C for
//   ROR PH0            ; the next ROR up the chain
//   ROR B_M2
//   ROR B_M1
//   ROR B_M0           ; final C = B_M0's old bit 0 = next multiplier bit
//   DEY
//   BNE loop
//
// When BCC skips the add, C is still 0 (that's what we branched on), so
// the ROR chain shifts a 0 into PH2's MSB -- which is correct: no add
// means no carry into bit 24 of the accumulator.
//
// ============================================================================
// REFERENCE: rounding, packing, and edge cases
// ============================================================================
//
// After the multiply and the bit-47 normalize step, the layout is:
//
//   PH2 bit 7 = implicit 1 (mantissa MSB)
//   PH2..PH0  = 24-bit mantissa (including implicit)
//   B_M2 bit 7 = round bit (R)
//   B_M2 bit 6 = guard bit (G)
//   OR(B_M2 & 0x3F, B_M1, B_M0) = sticky (S)
//
// Round up iff R && (G || S || mantissa_LSB).  A rounding overflow
// (mantissa was 0xFFFFFF and got incremented) is detected by
// mantissa == 0 after the increment; we restore bit 7 and bump exp.
//
// If result_exp <= 0, the result is subnormal (or zero): shift the
// mantissa right by (1 - result_exp) with sticky, then round, then set
// exp = 0.  If the shift count would leave nothing (>= 25), return
// signed zero.
//
// If result_exp >= 0xFF (after any exp++ from bit-47 normalize or from
// round-up wrap), return signed infinity.
//
// ============================================================================
// REFERENCE: zero-page layout
// ============================================================================
//
// Slot assignment is constrained by the ABI: on entry a's bytes 2/3 sit
// in __rc2/__rc3 and b's four bytes sit in __rc4..__rc7.  Byte 0/1 of a
// arrive in A/X (registers, not zero page).  We picked the layout so
// unpack can reuse the input slots in-place without needing extra
// scratch: A's byte2/byte3 sit at __rc2/__rc3 so A_M2 and A_EXP land
// there naturally after transformation; B's byte 0/1 need no move at
// all (they're already at B_M0/B_M1 = __rc4/__rc5), and B_M2/B_EXP
// overwrite b's byte2/byte3 in place at __rc6/__rc7.  A_M0/A_M1 come
// from registers A/X so they get fresh slots (__rc8/__rc9).
//
//     __rc2  A_M2    a mantissa top byte (implicit 1 in bit 7 for normals)
//     __rc3  A_EXP   a's biased exponent
//     __rc4  B_M0    b mantissa byte 0 (untouched from input)
//     __rc5  B_M1    b mantissa byte 1 (untouched from input)
//     __rc6  B_M2    b mantissa top byte (implicit 1 in bit 7 for normals)
//     __rc7  B_EXP   b's biased exponent
//     __rc8  A_M0    a mantissa byte 0 (from register A)
//     __rc9  A_M1    a mantissa byte 1 (from register X)
//     __rc10 A_SIGN  a's sign in bit 7 (0x00 or 0x80)
//     __rc11 B_SIGN  b's sign in bit 7
//     __rc12 R_SIGN  result sign in bit 7
//     __rc13 R_EXP_LO signed 16-bit result exponent, low byte
//     __rc14 R_EXP_HI  ... high byte
//     __rc15 PH0     product high accumulator, byte 0
//     __rc16 PH1
//     __rc17 PH2     product high, MSB (bit 47 lands here after multiply)
//     __rc18 CNT     multiply loop counter (24 iterations)
//     __rc19 TMP     scratch (used by subnormal-result shift and pack)
//
// After unpack, __rc2 (A_M2) and __rc3 (A_EXP) hold a's transformed
// bytes.  The pack phase at the end overwrites those two slots directly
// with the result byte2/byte3, which is fine because A_M2 and A_EXP
// aren't needed after we've read them into the packer.
//
//===----------------------------------------------------------------------===//

#include "../int_lib.h"
#include <stdint.h>

#define A_M2     "__rc2"
#define A_EXP    "__rc3"
#define B_M0     "__rc4"
#define B_M1     "__rc5"
#define B_M2     "__rc6"
#define B_EXP    "__rc7"
#define A_M0     "__rc8"
#define A_M1     "__rc9"
#define A_SIGN   "__rc10"
#define B_SIGN   "__rc11"
#define R_SIGN   "__rc12"
#define R_EXP_LO "__rc13"
#define R_EXP_HI "__rc14"
#define PH0      "__rc15"
#define PH1      "__rc16"
#define PH2      "__rc17"
#define CNT      "__rc18"
#define TMP      "__rc19"

__attribute__((naked, noinline))
COMPILER_RT_ABI float __mulsf3(float af, float bf) {
    asm volatile (
        // ================================================================
        // Phase 1 -- UNPACK A.
        //
        // Incoming: A = a_byte0, X = a_byte1, __rc2 = a_byte2, __rc3 =
        // a_byte3.  We move byte0/byte1 out of the registers into
        // A_M0/A_M1 first (freeing A and X), then transform __rc2 and
        // __rc3 in place into A_M2 and A_EXP; the sign gets its own
        // slot (A_SIGN) since it doesn't share a destination.
        //
        // Restore the implicit leading 1 in A_M2 bit 7 for normals;
        // leave it clear for subnormals (renormalize later) and zeros
        // (handled by dispatch).
        // ================================================================
        "sta " A_M0 "\n"                     // A_M0 = byte0 (from A)
        "stx " A_M1 "\n"                     // A_M1 = byte1 (from X)

        // Sign from byte3
        "lda __rc3\n"
        "and #$80\n"
        "sta " A_SIGN "\n"

        // Exp = (byte3 << 1) | (byte2 >> 7).  Order: LDA __rc2 / ASL A
        // sets C from byte2 bit 7 without touching byte2 in memory;
        // then LDA __rc3 preserves C; ROL A produces the biased exp.
        "lda __rc2\n"
        "asl a\n"                            // C = byte2 bit 7 = E0
        "lda __rc3\n"                        // A = byte3, C preserved
        "rol a\n"                            // A = biased exp
        "sta " A_EXP "\n"                    // overwrites __rc3 (was byte3)

        // A_M2 = (byte2 & 0x7f) | (exp != 0 ? 0x80 : 0).  __rc2 still
        // holds byte2 (we only did LDA / ASL A, no store).
        "lda __rc2\n"
        "and #$7f\n"
        "ldx " A_EXP "\n"
        "beq 1f\n"
        "ora #$80\n"
        "1: sta " A_M2 "\n"                  // overwrites __rc2 (was byte2)

        // ================================================================
        // Phase 2 -- UNPACK B.
        //
        // Same shape as A, but no register moves needed: b_byte0 arrives
        // already at __rc4 = B_M0, and b_byte1 at __rc5 = B_M1.  We only
        // need to transform __rc6 (byte2 -> B_M2) and __rc7 (byte3 ->
        // B_EXP) in place, and extract the sign into B_SIGN.
        // ================================================================
        "lda __rc7\n"
        "and #$80\n"
        "sta " B_SIGN "\n"

        "lda __rc6\n"
        "asl a\n"
        "lda __rc7\n"
        "rol a\n"
        "sta " B_EXP "\n"                    // overwrites __rc7

        "lda __rc6\n"
        "and #$7f\n"
        "ldx " B_EXP "\n"
        "beq 2f\n"
        "ora #$80\n"
        "2: sta " B_M2 "\n"                  // overwrites __rc6

        // ================================================================
        // Phase 3 -- RESULT SIGN.
        //
        // The product sign is the XOR of the input signs.  Since both
        // A_SIGN and B_SIGN have bit 7 = sign and bits 6..0 = 0, an EOR
        // gives the right result directly in bit 7.
        // ================================================================
        "lda " A_SIGN "\n"
        "eor " B_SIGN "\n"
        "sta " R_SIGN "\n"

        // ================================================================
        // Phase 4 -- INF/NAN dispatch.
        //
        // If a's exp is 0xFF, a is either NaN (mantissa != 0) or Inf.
        // If b's exp is 0xFF, similar.  Special-case combinations:
        //
        //   NaN * anything  -> quiet NaN (return a with bit 22 set)
        //   anything * NaN  -> quiet NaN (return b with bit 22 set)
        //   Inf * 0         -> qNaN (0x7fc00000)
        //   0 * Inf         -> qNaN (0x7fc00000)
        //   Inf * finite    -> signed Inf (result_sign)
        //   finite * Inf    -> signed Inf (result_sign)
        //
        // Note that A_M2's implicit-1 bit was set during unpack because
        // exp is 0xFF (nonzero), so the "is mantissa zero" check has to
        // mask off bit 7.
        // ================================================================
        "lda " A_EXP "\n"
        "cmp #$ff\n"
        "bne .L__mulsf3_chk_b_special\n"

        // a is Inf or NaN.  Distinguish.
        "lda " A_M2 "\n"
        "and #$7f\n"
        "ora " A_M1 "\n"
        "ora " A_M0 "\n"
        "beq .L__mulsf3_a_is_inf\n"

        // a is NaN: quiet it and return a.  Setting bit 22 (M2 bit 6)
        // makes any signalling NaN quiet.  A_M2 already has the implicit
        // 1 in bit 7; setting bit 6 gives the "quiet" marker.  Also copy
        // a's original sign back into R_SIGN so pack uses it.
        "lda " A_M2 "\n"
        "ora #$40\n"
        "sta " A_M2 "\n"
        "lda " A_SIGN "\n"
        "sta " R_SIGN "\n"
        "jmp .L__mulsf3_pack_a\n"

        ".L__mulsf3_a_is_inf:\n"
        // a is Inf.  Check b for NaN or 0.  If b is NaN return b.  If b
        // is 0 return qNaN.  Otherwise return signed Inf.
        "lda " B_EXP "\n"
        "cmp #$ff\n"
        "beq .L__mulsf3_a_inf_b_infnan\n"

        // b is finite.  If b is 0, this is Inf * 0 -> qNaN.
        "lda " B_EXP "\n"
        "ora " B_M0 "\n"
        "ora " B_M1 "\n"
        "ora " B_M2 "\n"
        "beq .L__mulsf3_return_qnan\n"
        // b is finite and nonzero -> return signed Inf.
        "jmp .L__mulsf3_return_signed_inf\n"

        ".L__mulsf3_a_inf_b_infnan:\n"
        // b's exp is 0xFF.  NaN vs Inf?
        "lda " B_M2 "\n"
        "and #$7f\n"
        "ora " B_M1 "\n"
        "ora " B_M0 "\n"
        "beq .L__mulsf3_return_signed_inf\n"          // b is Inf -> Inf*Inf = signed Inf
        // b is NaN -> return quiet(b).
        "lda " B_M2 "\n"
        "ora #$40\n"
        "sta " A_M2 "\n"
        "lda " B_M1 "\n"
        "sta " A_M1 "\n"
        "lda " B_M0 "\n"
        "sta " A_M0 "\n"
        "lda " B_EXP "\n"
        "sta " A_EXP "\n"
        "lda " B_SIGN "\n"
        "sta " R_SIGN "\n"
        "jmp .L__mulsf3_pack_a\n"

        ".L__mulsf3_chk_b_special:\n"
        // a is not special.  Is b special?
        "lda " B_EXP "\n"
        "cmp #$ff\n"
        "bne .L__mulsf3_chk_zero\n"

        // b is Inf or NaN.
        "lda " B_M2 "\n"
        "and #$7f\n"
        "ora " B_M1 "\n"
        "ora " B_M0 "\n"
        "beq .L__mulsf3_b_is_inf\n"
        // b is NaN -> return quiet(b).
        "lda " B_M2 "\n"
        "ora #$40\n"
        "sta " A_M2 "\n"
        "lda " B_M1 "\n"
        "sta " A_M1 "\n"
        "lda " B_M0 "\n"
        "sta " A_M0 "\n"
        "lda " B_EXP "\n"
        "sta " A_EXP "\n"
        "lda " B_SIGN "\n"
        "sta " R_SIGN "\n"
        "jmp .L__mulsf3_pack_a\n"

        ".L__mulsf3_b_is_inf:\n"
        // b is Inf, a is finite.  If a is 0, Inf*0 -> qNaN.
        "lda " A_EXP "\n"
        "ora " A_M0 "\n"
        "ora " A_M1 "\n"
        "ora " A_M2 "\n"                     // a_M2 has bit 7 clear iff
                                             // subnormal or zero; a is not
                                             // NaN/Inf here so this OR is
                                             // 0 only for true zero
        "beq .L__mulsf3_return_qnan\n"
        // fall through to return signed Inf.

        ".L__mulsf3_return_signed_inf:\n"
        "lda #0\n"
        "sta " A_M0 "\n"
        "sta " A_M1 "\n"
        "lda #$80\n"
        "sta " A_M2 "\n"                     // mantissa 0, exp 0xff
        "lda #$ff\n"
        "sta " A_EXP "\n"
        "jmp .L__mulsf3_pack_a\n"

        ".L__mulsf3_return_qnan:\n"
        // Canonical qNaN 0x7fc00000: byte3 = 0x7f, byte2 = 0xc0, low = 0.
        "lda #$c0\n"
        "sta __rc2\n"
        "lda #$7f\n"
        "sta __rc3\n"
        "ldx #0\n"
        "lda #0\n"
        "rts\n"

        // ================================================================
        // Phase 5 -- ZERO dispatch.
        //
        // If either operand is zero, the product is signed zero
        // (R_SIGN was already computed as a XOR b).
        //
        // Test: (exp | m0 | m1 | m2) == 0.  For zero, all four are 0.
        // For subnormal, at least one mantissa byte is nonzero (implicit
        // wasn't set for subnormals during unpack).  For normal, exp
        // is nonzero.
        // ================================================================
        ".L__mulsf3_chk_zero:\n"
        "lda " A_EXP "\n"
        "ora " A_M0 "\n"
        "ora " A_M1 "\n"
        "ora " A_M2 "\n"
        "beq .L__mulsf3_return_zero\n"
        "lda " B_EXP "\n"
        "ora " B_M0 "\n"
        "ora " B_M1 "\n"
        "ora " B_M2 "\n"
        "bne .L__mulsf3_not_zero\n"

        ".L__mulsf3_return_zero:\n"
        // Return signed zero: byte3 = R_SIGN, byte2 = 0, low = 0.
        "lda #0\n"
        "sta __rc2\n"
        "lda " R_SIGN "\n"
        "sta __rc3\n"
        "ldx #0\n"
        "lda #0\n"
        "rts\n"

        ".L__mulsf3_not_zero:\n"

        // ================================================================
        // Phase 6 -- RESULT EXPONENT + SUBNORMAL RENORMALIZATION.
        //
        // R_EXP is a signed 16-bit accumulator that ends up as the biased
        // result exponent (with pre-round adjustments).  We build it in
        // three steps so we never have to sign-extend an ambiguous byte:
        //
        //   (a) R_EXP = A_EXP + B_EXP - 127.  Since A_EXP, B_EXP are
        //       unsigned 0..254 at this point, the sum is 0..508 (fits
        //       in 9 bits) and the subtraction can borrow into the high
        //       byte; result range [-127, 381], all representable.
        //
        //   (b) If A was subnormal (raw A_EXP == 0), the sum used 0 for
        //       A but a's "effective biased exp" is (1 - k) where k is
        //       the left-shift count needed to normalize a's significand.
        //       So we add 1 to R_EXP up front, then decrement it once
        //       for each left shift in the renormalize loop.
        //
        //   (c) Same for B.
        //
        // A raw subnormal has M2 bit 7 = 0 (unpack skipped the implicit-1
        // set for exp==0) and at least one mantissa byte nonzero (true
        // zero was dispatched in phase 5), so the shift loop terminates.
        // ================================================================

        // R_EXP = A_EXP + B_EXP (unsigned, 9-bit sum captured with carry)
        "clc\n"
        "lda " A_EXP "\n"
        "adc " B_EXP "\n"
        "sta " R_EXP_LO "\n"
        "lda #0\n"
        "adc #0\n"                           // A = carry (0 or 1)
        "sta " R_EXP_HI "\n"

        // R_EXP -= 127
        "sec\n"
        "lda " R_EXP_LO "\n"
        "sbc #127\n"
        "sta " R_EXP_LO "\n"
        "lda " R_EXP_HI "\n"
        "sbc #0\n"
        "sta " R_EXP_HI "\n"

        // If a subnormal (A_EXP == 0): R_EXP += 1, then loop left-shift
        // A's significand until M2 bit 7 is set, decrementing R_EXP per
        // shift.
        "lda " A_EXP "\n"
        "bne .L__mulsf3_renorm_b\n"
        "inc " R_EXP_LO "\n"
        "bne 3f\n"
        "inc " R_EXP_HI "\n"
        "3:\n"
        ".L__mulsf3_renorm_a_loop:\n"
        "bit " A_M2 "\n"
        "bmi .L__mulsf3_renorm_b\n"
        "asl " A_M0 "\n"
        "rol " A_M1 "\n"
        "rol " A_M2 "\n"
        // R_EXP -= 1  (16-bit dec)
        "lda " R_EXP_LO "\n"
        "bne 4f\n"
        "dec " R_EXP_HI "\n"
        "4: dec " R_EXP_LO "\n"
        "jmp .L__mulsf3_renorm_a_loop\n"

        ".L__mulsf3_renorm_b:\n"
        // Same for b.
        "lda " B_EXP "\n"
        "bne .L__mulsf3_mul_init\n"
        "inc " R_EXP_LO "\n"
        "bne 5f\n"
        "inc " R_EXP_HI "\n"
        "5:\n"
        ".L__mulsf3_renorm_b_loop:\n"
        "bit " B_M2 "\n"
        "bmi .L__mulsf3_mul_init\n"
        "asl " B_M0 "\n"
        "rol " B_M1 "\n"
        "rol " B_M2 "\n"
        "lda " R_EXP_LO "\n"
        "bne 6f\n"
        "dec " R_EXP_HI "\n"
        "6: dec " R_EXP_LO "\n"
        "jmp .L__mulsf3_renorm_b_loop\n"

        ".L__mulsf3_mul_init:\n"

        // ================================================================
        // Phase 8 -- MULTIPLY: 24 x 24 -> 48-bit unsigned.
        //
        // See the "shift-and-add multiply core" reference at the top.
        // Multiplicand: A_M0..A_M2.  Multiplier and low half of product:
        // B_M0..B_M2.  High half of product accumulator: PH0..PH2, init 0.
        // ================================================================
        "lda #0\n"
        "sta " PH0 "\n"
        "sta " PH1 "\n"
        "sta " PH2 "\n"
        "lda #24\n"
        "sta " CNT "\n"

        // Pre-shift: shift multiplier right by 1 to get bit 0 into C.
        "lsr " B_M2 "\n"
        "ror " B_M1 "\n"
        "ror " B_M0 "\n"

        ".L__mulsf3_mul_loop:\n"
        "bcc .L__mulsf3_mul_noadd\n"
        "clc\n"
        "lda " PH0 "\n" "adc " A_M0 "\n" "sta " PH0 "\n"
        "lda " PH1 "\n" "adc " A_M1 "\n" "sta " PH1 "\n"
        "lda " PH2 "\n" "adc " A_M2 "\n" "sta " PH2 "\n"
        ".L__mulsf3_mul_noadd:\n"
        "ror " PH2 "\n"
        "ror " PH1 "\n"
        "ror " PH0 "\n"
        "ror " B_M2 "\n"
        "ror " B_M1 "\n"
        "ror " B_M0 "\n"
        "dec " CNT "\n"
        "bne .L__mulsf3_mul_loop\n"

        // ================================================================
        // Phase 9 -- BIT-47 NORMALIZE.
        //
        // Product now in [PH2:PH1:PH0:B_M2:B_M1:B_M0].  Bit 47 = PH2 bit 7.
        //   set  -> product in [2^47, 2^48); exp += 1, no shift
        //   clear -> product in [2^46, 2^47) (bit 46 set); shift left 1
        //
        // For inputs with both implicit bits set (normals or renormalized
        // subnormals with M2 bit 7 = 1), the product is guaranteed >=
        // 2^46, so bit 46 or bit 47 is set.  (We would never reach here
        // with a zero mantissa product because zero was dispatched.)
        // ================================================================
        "bit " PH2 "\n"
        "bmi .L__mulsf3_norm_47_set\n"
        // Shift 48-bit product left by 1
        "asl " B_M0 "\n"
        "rol " B_M1 "\n"
        "rol " B_M2 "\n"
        "rol " PH0 "\n"
        "rol " PH1 "\n"
        "rol " PH2 "\n"
        "jmp .L__mulsf3_chk_ovf\n"

        ".L__mulsf3_norm_47_set:\n"
        // Bit 47 was already set; exp += 1 (may overflow into hi byte).
        "inc " R_EXP_LO "\n"
        "bne .L__mulsf3_chk_ovf\n"
        "inc " R_EXP_HI "\n"

        ".L__mulsf3_chk_ovf:\n"
        // ================================================================
        // Phase 10 -- OVERFLOW / UNDERFLOW dispatch.
        //
        //   R_EXP < 0            -> underflow: subnormal or zero
        //   R_EXP == 0           -> subnormal (biased-0 with mantissa
        //                           bit 47 set means smaller than the
        //                           smallest normal; shift right by 1)
        //   R_EXP in [1, 0xFE]   -> normal, proceed to round
        //   R_EXP >= 0xFF        -> overflow: signed infinity
        // ================================================================
        "lda " R_EXP_HI "\n"
        "bmi .L__mulsf3_chk_und\n"                    // R_EXP < 0
        "bne .L__mulsf3_overflow\n"                   // hi > 0 -> definitely overflow
        "lda " R_EXP_LO "\n"
        "beq .L__mulsf3_chk_und\n"                    // R_EXP == 0 -> subnormal
        "cmp #$ff\n"
        "bcc .L__mulsf3_round\n"                      // R_EXP in [1, 0xFE] -> normal
        // fall through: R_EXP >= 0xFF -> overflow

        ".L__mulsf3_overflow:\n"
        "lda #$ff\n"
        "sta " A_EXP "\n"
        "lda #0\n"
        "sta " PH0 "\n"
        "sta " PH1 "\n"
        "lda #$80\n"
        "sta " PH2 "\n"
        "jmp .L__mulsf3_pack_ph\n"

        ".L__mulsf3_chk_und:\n"
        // ================================================================
        // Phase 11 -- UNDERFLOW / SUBNORMAL RESULT.
        //
        // R_EXP <= 0 -> result is subnormal (or zero).  We need to shift
        // the [PH2:PH1:PH0:B_M2:B_M1:B_M0] product right by (1 - R_EXP)
        // bits, keeping sticky, then round, then set exp = 0.
        //
        // If the shift count >= 25, everything except sticky is gone; the
        // rounded result is either 0 or the smallest subnormal.  For
        // simplicity we return signed 0 for shift >= 32 and clamp shift
        // to 25 otherwise.
        // ================================================================
        // shift_count = 1 - R_EXP (positive; larger for more-negative
        // R_EXP).  We compute this as (1 - R_EXP_LO) with borrow, but
        // since R_EXP is negative (bit 7 of R_EXP_HI set), we know
        // shift_count = 1 + |R_EXP|.
        //
        // Compute: shift = 1 - R_EXP_LO - 256*R_EXP_HI.  Since R_EXP_HI
        // is 0xFF for small negatives (|R_EXP| < 128), shift =
        // 1 - R_EXP_LO + 256 which is > 256 iff R_EXP_LO < -255... which
        // can't happen (R_EXP >= -171).  Just use 1 - R_EXP_LO as byte
        // arithmetic; if R_EXP_LO is e.g. 0xEA (-22), 1 - 0xEA = 0x17
        // (as unsigned 8-bit, with borrow).  We want +23; 1 - (-22) = 23.
        // sec / lda #1 / sbc R_EXP_LO gives 1 - R_EXP_LO in A, which for
        // R_EXP_LO = 0xEA gives 0x17 = 23.  Perfect.

        "sec\n"
        "lda #1\n"
        "sbc " R_EXP_LO "\n"                 // A = 1 - R_EXP_LO (byte)

        // If R_EXP_HI is nonzero-and-not-0xFF (impossible), or if the
        // shift count is huge (>= 32), just return signed 0.  We treat
        // shift >= 32 as underflow-to-zero.  .L__mulsf3_return_zero is >128 bytes
        // back from here, out of short-branch range, so use invert-and-jmp
        // to avoid silent branch-offset truncation.
        "cmp #32\n"
        "bcc 15f\n"                          // shift < 32, continue
        "jmp .L__mulsf3_return_zero\n"
        "15:\n"

        // Shift the 48-bit product right by A bits.  Use whole-byte
        // shifts for count >= 8, then bit loop for residual.  Sticky
        // accumulates in TMP.
        "sta " CNT "\n"
        "lda #0\n"
        "sta " TMP "\n"                      // sticky accumulator = 0

        ".L__mulsf3_sub_byte:\n"
        "lda " CNT "\n"
        "cmp #8\n"
        "bcc .L__mulsf3_sub_bits\n"
        // Shift right by 8 = shuffle bytes down.  OR B_M0 into sticky first.
        "lda " B_M0 "\n"
        "beq 7f\n"
        "lda #1\n"
        "sta " TMP "\n"
        "7: lda " B_M1 "\n"
        "sta " B_M0 "\n"
        "lda " B_M2 "\n"
        "sta " B_M1 "\n"
        "lda " PH0 "\n"
        "sta " B_M2 "\n"
        "lda " PH1 "\n"
        "sta " PH0 "\n"
        "lda " PH2 "\n"
        "sta " PH1 "\n"
        "lda #0\n"
        "sta " PH2 "\n"
        "lda " CNT "\n"
        "sec\n"
        "sbc #8\n"
        "sta " CNT "\n"
        "jmp .L__mulsf3_sub_byte\n"

        ".L__mulsf3_sub_bits:\n"
        "ldy " CNT "\n"
        "beq .L__mulsf3_sub_done\n"
        ".L__mulsf3_sub_bit_loop:\n"
        "lsr " PH2 "\n"
        "ror " PH1 "\n"
        "ror " PH0 "\n"
        "ror " B_M2 "\n"
        "ror " B_M1 "\n"
        "ror " B_M0 "\n"
        "bcc 8f\n"
        "lda #1\n"
        "sta " TMP "\n"
        "8: dey\n"
        "bne .L__mulsf3_sub_bit_loop\n"

        ".L__mulsf3_sub_done:\n"
        // OR the accumulated sticky into B_M0's LSB region.  We use B_M0
        // for sticky by writing 1 to it if TMP is nonzero.  Actually we
        // want the sticky to be represented in the low bits below the
        // guard for the round logic below.  Simplest: OR TMP into B_M0.
        "lda " TMP "\n"
        "beq 10f\n"
        "lda " B_M0 "\n"
        "ora #1\n"
        "sta " B_M0 "\n"
        "10:\n"
        // Set R_EXP = 0 (subnormal marker).  R_EXP_HI already whatever.
        "lda #0\n"
        "sta " R_EXP_LO "\n"
        "sta " R_EXP_HI "\n"

        ".L__mulsf3_round:\n"
        // ================================================================
        // Phase 12 -- ROUND: nearest, ties to even.
        //
        // R = B_M2 bit 7, G = B_M2 bit 6, S = OR(B_M2 & 0x3F, B_M1, B_M0),
        // LSB = PH0 bit 0.  Round up iff R && (G || S || LSB).
        // ================================================================
        "bit " B_M2 "\n"                     // N = bit 7 (R), V = bit 6 (G)
        "bpl .L__mulsf3_no_round\n"                   // R clear -> no round

        // R set.  Compute (G || S || LSB).  Start with (B_M2 & 0x7F)
        // OR B_M1 OR B_M0 OR (PH0 & 1).
        "lda " B_M2 "\n"
        "and #$7f\n"                         // strip R, keep G+partial S
        "ora " B_M1 "\n"
        "ora " B_M0 "\n"
        "sta " TMP "\n"
        "lda " PH0 "\n"
        "and #1\n"                           // LSB
        "ora " TMP "\n"
        "beq .L__mulsf3_no_round\n"                   // exact tie AND LSB even -> no round

        // Round up: ripple increment PH0..PH2.
        "inc " PH0 "\n"
        "bne .L__mulsf3_no_round\n"
        "inc " PH1 "\n"
        "bne .L__mulsf3_no_round\n"
        "inc " PH2 "\n"
        "bne .L__mulsf3_no_round\n"
        // Full carry-out: mantissa wrapped 0xFFFFFF -> 0x000000.
        // Restore bit 7 (implicit 1) and bump exp; check overflow.
        "lda #$80\n"
        "sta " PH2 "\n"
        "inc " R_EXP_LO "\n"
        "bne 11f\n"
        "inc " R_EXP_HI "\n"
        "11:\n"
        // Recheck overflow after round bump.  .L__mulsf3_overflow is > 127 bytes
        // back from here, out of short-branch range -- the assembler
        // silently truncates out-of-range branch offsets, so we use
        // invert-and-jmp for both checks.
        "lda " R_EXP_HI "\n"
        "beq 13f\n"                          // hi==0, skip forward
        "jmp .L__mulsf3_overflow\n"
        "13: lda " R_EXP_LO "\n"
        "cmp #$ff\n"
        "bcc .L__mulsf3_no_round\n"                   // R_EXP <= 0xFE, no overflow
        "jmp .L__mulsf3_overflow\n"

        ".L__mulsf3_no_round:\n"
        // Post-round check: if we were subnormal (R_EXP == 0) and the
        // round caused mantissa to become normal (PH2 bit 7 set), we
        // need to bump R_EXP to 1 for correct encoding.  If R_EXP was
        // already nonzero we skip.  If PH2 bit 7 is clear the result
        // stays subnormal.
        "lda " R_EXP_LO "\n"
        "ora " R_EXP_HI "\n"
        "bne .L__mulsf3_setup_pack\n"                 // not exp 0, skip
        "bit " PH2 "\n"
        "bpl .L__mulsf3_setup_pack\n"                 // still subnormal
        "lda #1\n"
        "sta " R_EXP_LO "\n"

        ".L__mulsf3_setup_pack:\n"
        // Move R_EXP into A_EXP for the shared packer below (packer
        // expects A_EXP as the biased result exponent).
        "lda " R_EXP_LO "\n"
        "sta " A_EXP "\n"

        ".L__mulsf3_pack_ph:\n"
        // ================================================================
        // Phase 13 -- PACK: reassemble bytes into ABI return slots.
        //
        // Same layout as addsf3 pack, but our mantissa lives in
        // PH0..PH2 instead of A_M1..A_M3.  Copy PH -> A_M then reuse
        // the pack sequence.
        //
        // Layout on entry:
        //   A_EXP  = biased result exponent (0..0xFF)
        //   R_SIGN = result sign in bit 7
        //   PH2    = mantissa MSB (bit 7 = implicit 1 for normals, 0 for
        //            subnormals or zero)
        //   PH1    = mantissa mid
        //   PH0    = mantissa low
        // ================================================================
        "lda " PH0 "\n"
        "sta " A_M0 "\n"
        "lda " PH1 "\n"
        "sta " A_M1 "\n"
        "lda " PH2 "\n"
        "sta " A_M2 "\n"
        // fall through to .L__mulsf3_pack_a

        ".L__mulsf3_pack_a:\n"
        // Byte3 = R_SIGN | (A_EXP >> 1); byte2 = ((A_EXP & 1) << 7) |
        // (A_M2 & 0x7F); byte1 = A_M1; byte0 = A_M0.
        "lda " A_EXP "\n"
        "lsr a\n"                            // A = exp>>1, C = exp bit 0
        "sta " TMP "\n"
        "lda " A_M2 "\n"
        "and #$7f\n"
        "bcc 12f\n"
        "ora #$80\n"
        "12: sta __rc2\n"                    // byte2
        "lda " TMP "\n"
        "ora " R_SIGN "\n"
        "sta __rc3\n"                        // byte3
        "lda " A_M0 "\n"                     // byte0 -> A
        "ldx " A_M1 "\n"                     // byte1 -> X
        "rts\n"
    );
}
