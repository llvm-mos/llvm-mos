//===-- MOSTargetInfo.cpp - MOS Target Implementation ---------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "llvm/IR/Module.h"
#include "llvm/Support/TargetRegistry.h"

namespace llvm {
Target &getTheMOSTarget() {
  static Target TheMOSTarget;
  return TheMOSTarget;
}
} // namespace llvm

extern "C" void LLVM_EXTERNAL_VISIBILITY LLVMInitializeMOSTargetInfo() { // NOLINT
  llvm::RegisterTarget<llvm::Triple::mos> X(llvm::getTheMOSTarget(), "mos",
                                            "MOS Technologies 65xx and variants", "MOS");
}

