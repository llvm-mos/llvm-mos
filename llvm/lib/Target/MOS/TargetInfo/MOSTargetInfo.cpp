//===-- MOSTargetInfo.cpp - MOS Target Implementation ---------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "llvm/IR/Module.h"
#include "llvm/Support/TargetRegistry.h"
namespace llvm {
Target &getTheMOSTarget() {
  static Target TheMOSTarget;
  return TheMOSTarget;
}
}

extern "C" void LLVMInitializeMOSTargetInfo() {
  llvm::RegisterTarget<llvm::Triple::mos> X(llvm::getTheMOSTarget(), "mos",
                                            "MOS 65xx", "MOS");
}

