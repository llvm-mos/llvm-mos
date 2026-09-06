// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception

// RUN: %clang_builtins %s %librt -o %t && %run %t
// REQUIRES: librt_has_addsf3

#include "int_lib.h"
#include <inttypes.h>
#include <stdio.h>

#include "fp_test.h"

// Returns: a + b
COMPILER_RT_ABI float __addsf3(float a, float b);

int test__addsf3(uint32_t a_rep, uint32_t b_rep, uint32_t expected_rep) {
  float a = fromRep32(a_rep), b = fromRep32(b_rep);
  float x = __addsf3(a, b);
  int ret = compareResultF(x, expected_rep);

  if (ret) {
    printf("error in test__addsf3(%08" PRIx32 ", %08" PRIx32 ") = %08" PRIx32
           ", expected %08" PRIx32 "\n",
           a_rep, b_rep, toRep32(x), expected_rep);
  }

  return ret;
}

int main(void) {
  int status = 0;

  // zero + zero = zero
  status |= test__addsf3(0x00000000, 0x00000000, 0x00000000);
  // -zero + -zero = -zero
  status |= test__addsf3(0x80000000, 0x80000000, 0x80000000);
  // zero + -zero = zero
  status |= test__addsf3(0x00000000, 0x80000000, 0x00000000);

  // zero + normal = normal
  status |= test__addsf3(0x00000000, 0x3f800000, 0x3f800000); // 0 + 1.0 = 1.0
  // normal + zero = normal
  status |= test__addsf3(0x3f800000, 0x00000000, 0x3f800000); // 1.0 + 0 = 1.0

  // normal + normal = normal
  status |= test__addsf3(0x3f800000, 0x3f800000, 0x40000000); // 1.0 + 1.0 = 2.0
  status |= test__addsf3(0x40000000, 0x3f800000, 0x40400000); // 2.0 + 1.0 = 3.0
  status |=
      test__addsf3(0x3f800000, 0xbf800000, 0x00000000); // 1.0 + -1.0 = 0.0
  status |=
      test__addsf3(0xbf800000, 0x3f800000, 0x00000000); // -1.0 + 1.0 = 0.0

  // inf + inf = inf
  status |= test__addsf3(0x7f800000, 0x7f800000, 0x7f800000);
  // -inf + -inf = -inf
  status |= test__addsf3(0xff800000, 0xff800000, 0xff800000);
  // inf + -inf = NaN
  status |= test__addsf3(0x7f800000, 0xff800000, 0x7fc00000);
  // inf + normal = inf
  status |= test__addsf3(0x7f800000, 0x3f800000, 0x7f800000);

  // NaN + anything = NaN
  status |= test__addsf3(0x7fc00000, 0x3f800000, 0x7fc00000);
  // anything + NaN = NaN
  status |= test__addsf3(0x3f800000, 0x7fc00000, 0x7fc00000);

  // smallest subnormal + smallest subnormal = 2 * smallest subnormal
  status |= test__addsf3(0x00000001, 0x00000001, 0x00000002);

  // subnormal + subnormal, result stays subnormal
  status |= test__addsf3(0x00000003, 0x00000005, 0x00000008);

  // Denormal addition overflow: two subnormals whose sum is normal.
  // This is the bug case from llvm/llvm-project#185245.
  //
  // 0x004d8ad0 = 0x1.362b4p-127 (subnormal)
  // 0x009b15a0 = 0x1.362b4p-126 (normal, smallest biased exponent)
  status |= test__addsf3(0x004d8ad0, 0x004d8ad0, 0x009b15a0);

  // Half of smallest normal doubled = smallest normal
  // 0x00400000 = 0x1.0p-127 (subnormal), doubled = 0x00800000 = 0x1.0p-126
  status |= test__addsf3(0x00400000, 0x00400000, 0x00800000);

  // Largest subnormal doubled overflows to normal
  // 0x007fffff = largest subnormal, doubled = 0x00fffffe (normal)
  status |= test__addsf3(0x007fffff, 0x007fffff, 0x00fffffe);

  // Negative subnormal overflow to normal
  status |= test__addsf3(0x804d8ad0, 0x804d8ad0, 0x809b15a0);

  // Large normal values near overflow
  status |= test__addsf3(0x7f7fffff, 0x7f7fffff, 0x7f800000); // max + max = inf

  // Regression: opposite-sign near-cancellation whose true result is a nonzero
  // subnormal.  The normalize bit-loop must stop *at* the exp==1 -> exp==0
  // transition without an additional shift-and-decrement, because IEEE 754
  // subnormals reuse the same reference scale as the smallest normal (they
  // just drop the implicit leading 1).  An implementation that shifts one
  // more time across that boundary silently returns exactly 2x the correct
  // magnitude for any such subnormal result.
  status |= test__addsf3(0x00a00000u, 0x80800000u, 0x00200000u);
  status |= test__addsf3(0x00810000u, 0x80800000u, 0x00010000u);
  status |= test__addsf3(0x00800001u, 0x80800000u, 0x00000001u);
  status |= test__addsf3(0x03000000u, 0x82fffffeu, 0x00000020u);
  // Additional cases discovered by fuzzing a Python model vs numpy float32.
  status |= test__addsf3(0x00513f55u, 0x808ac0efu, 0x8039819au);
  status |= test__addsf3(0x007247f1u, 0x808ac31eu, 0x80187b2du);
  status |= test__addsf3(0x007775f9u, 0x8093b73fu, 0x801c4146u);
  status |= test__addsf3(0x00740e32u, 0x809786e6u, 0x802378b4u);
  status |= test__addsf3(0x00ef502eu, 0x80c3a704u, 0x002ba92au);
  status |= test__addsf3(0x0250d539u, 0x825322c0u, 0x80126c38u);
  status |= test__addsf3(0x00d60017u, 0x80b739b2u, 0x001ec665u);
  status |= test__addsf3(0x02fcce07u, 0x82fd68ffu, 0x8009af80u);

  // Regression: opposite-sign add where b needs an alignment right-shift and
  // some of b's low bits are truncated (sticky bit set).  For the SUBTRACT
  // path this means the computed difference is slightly too large by a
  // strictly-positive amount less than one ULP, so a nonzero sticky must
  // bias the round DOWN -- the mirror image of the add-path convention where
  // a nonzero sticky biases up.  Implementations that reuse the add-path
  // rounding logic on the subtract path produce results that are exactly
  // +1 ULP high.
  status |= test__addsf3(0x216ee743u, 0x9a7a602fu, 0x216ee359u);
  status |= test__addsf3(0x291f4cbeu, 0xa2b4701fu, 0x291f471au);
  status |= test__addsf3(0x006ae03eu, 0x87a2a103u, 0x87a29f57u);
  status |= test__addsf3(0x237f6e73u, 0x9b70803cu, 0x237f6d82u);
  status |= test__addsf3(0x1843199eu, 0x8e380fb8u, 0x18431992u);
  status |= test__addsf3(0x222fb009u, 0x97d01c62u, 0x222fb002u);

  // Regression: subtract of (2^N) - epsilon where epsilon is far below the
  // ULP of the result.  The mantissa rounds up from 0xFFFFFF to 0x1000000,
  // which is renormalized to 0x800000 with exp incremented cleanly (no
  // overflow to infinity).  An implementation whose round-up-with-mantissa-
  // wrap path falls through into the "return b" packer (used by the
  // NaN-quiet and both-zeros paths) will corrupt the result by loading b's
  // fields into a's slots -- typically returning -b instead of the correct
  // ~2^N.  The path is only reachable when subtract's ripple-decrement +
  // normalize combines with a round-up that wraps the mantissa.
  status |= test__addsf3(0x40000000u, 0x80000001u, 0x40000000u);
  status |= test__addsf3(0x50000000u, 0xb0000000u, 0x50000000u);
  status |= test__addsf3(0x60000000u, 0x80000001u, 0x60000000u);
  status |= test__addsf3(0x7e000000u, 0x80000001u, 0x7e000000u);

  return status;
}
