//===--- MOS.cpp - Implement MOS target feature support -----------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file implements MOS TargetInfo objects.
//
//===----------------------------------------------------------------------===//

#include "MOS.h"

using namespace clang::targets;

MOSTargetInfo::MOSTargetInfo(const llvm::Triple &Triple,
                                     const TargetOptions &)
    : TargetInfo(Triple) {
  static const char Layout[] =
      "e-p:16:8-i16:8-i32:8-i64:8-f32:8-f64:8-a:8-Fi8-n8";
  resetDataLayout(Layout);

  PointerWidth = 16;
  PointerAlign = 8;
  IntWidth = 16;
  IntAlign = 8;
  LongAlign = 8;
  LongLongAlign = 8;
  ShortAccumAlign = 8;
  AccumWidth = 16;
  AccumAlign = 8;
  LongAccumAlign = 8;
  FractWidth = FractAlign = 8;
  LongFractAlign = 8;
  AccumScale = 7;
  SuitableAlign = 8;
  DefaultAlignForAttributeAligned = 8;
  SizeType = UnsignedShort;
  PtrDiffType = SignedShort;
  IntPtrType = SignedShort;
  WCharType = UnsignedLong;
  WIntType = UnsignedLong;
  Char32Type = UnsignedLong;
  SigAtomicType = UnsignedChar;
}

bool MOSTargetInfo::validateAsmConstraint(
    const char *&Name, TargetInfo::ConstraintInfo &Info) const {
  switch (*Name) {
  default:
    return false;
  // The A, X, or Y registers.
  case 'a':
  case 'x':
  case 'y':
    Info.setAllowsRegister();
    return true;
  }
  return false;
}
