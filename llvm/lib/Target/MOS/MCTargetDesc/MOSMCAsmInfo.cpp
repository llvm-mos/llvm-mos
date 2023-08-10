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
#include "MOSMCTargetDesc.h"

#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/TargetParser/Triple.h"

namespace llvm {

MOSMCAsmInfo::MOSMCAsmInfo(const Triple &TT, const MCTargetOptions &Options) {
  CodePointerSize = 2;
  CalleeSaveStackSlotSize = 0;
  SeparatorString = "\n";
  CommentString = ";";
  DollarIsHexPrefix = true;
  // Maximum instruction length across all supported subtargets.
  MaxInstLength = 7;
  SupportsDebugInformation = true;
}

unsigned MOSMCAsmInfo::getMaxInstLength(const MCSubtargetInfo *STI) const {
  if (!STI)
    return MaxInstLength;

  if (STI->hasFeature(MOS::FeatureHUC6280))
    return 7;
  if (STI->hasFeature(MOS::Feature45GS02))
    return 5;
  if (STI->hasFeature(MOS::FeatureW65816))
    return 4;
  return 3;
}

} //  namespace llvm
