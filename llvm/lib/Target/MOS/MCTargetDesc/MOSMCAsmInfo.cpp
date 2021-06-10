//===-- MOSMCAsmInfo.cpp - MOS asm properties -----------------------------===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains the declarations of the MOSMCAsmInfo properties.
//
//===----------------------------------------------------------------------===//

#include "MOSMCAsmInfo.h"

#include "llvm/ADT/Triple.h"

namespace llvm {

MOSMCAsmInfo::MOSMCAsmInfo(const Triple &TT, const MCTargetOptions &Options) {
  CodePointerSize = 2;
  CalleeSaveStackSlotSize = 0;
  SeparatorString = "\n";
  CommentString = ";";
  DollarIsHexPrefix = true;
  MaxInstLength = 3;
}

} //  namespace llvm
