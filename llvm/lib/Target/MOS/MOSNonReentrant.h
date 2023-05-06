//===-- MOSNonReentrant.h - MOS NonReentrant Pass ---------------*- C++ -*-===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file declares the MOS NonReentrant pass.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MOS_MOSNORECURSE_H
#define LLVM_LIB_TARGET_MOS_MOSNORECURSE_H

#include "llvm/IR/Module.h"
#include "llvm/IR/PassManager.h"
#include "llvm/Pass.h"

namespace llvm {

ModulePass *createMOSNonReentrantPass();

struct MOSNonReentrantPass : PassInfoMixin<MOSNonReentrantPass> {
  PreservedAnalyses run(Module &M, ModuleAnalysisManager &AM);
};

} // end namespace llvm

#endif // not LLVM_LIB_TARGET_MOS_MOSNORECURSE_H
