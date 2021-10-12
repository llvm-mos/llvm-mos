//===-- MOSTargetInfo.cpp - MOS Target Implementation ---------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "TargetInfo/MOSTargetInfo.h"
#include "llvm/MC/TargetRegistry.h"

namespace llvm {
Target &getTheMOSTarget() {
  static Target TheMOSTarget;
  return TheMOSTarget;
}
} // namespace llvm

extern "C" LLVM_EXTERNAL_VISIBILITY void LLVMInitializeMOSTargetInfo() { // NOLINT
  llvm::RegisterTarget<llvm::Triple::mos> X(llvm::getTheMOSTarget(), "mos",
                                            "MOS Technologies 65xx and variants", "MOS");
}

