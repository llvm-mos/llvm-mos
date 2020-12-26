//===-- MOSMCAsmInfo.cpp - MOS asm properties -----------------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
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
  CalleeSaveStackSlotSize = 2;
  CommentString = ";";
  PrivateGlobalPrefix = ".L";
  UsesELFSectionDirectiveForBSS = true;
  DollarIsHexPrefix = true;
  UseIntegratedAssembler = true;
}

} //  namespace llvm
