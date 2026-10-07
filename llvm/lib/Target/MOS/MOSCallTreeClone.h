//===-- MOSCallTreeClone.h - MOS Call Tree Cloning -------------*- C++ -*-===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file declares the MOS call tree cloning pass. For each function with the
// "interrupt-rc-suffix" attribute, the pass clones the reachable call tree and
// the defined runtime libcalls into private per-suffix copies so they can use a
// private imaginary-register set.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MOS_MOSCALLTREECLONE_H
#define LLVM_LIB_TARGET_MOS_MOSCALLTREECLONE_H

#include "llvm/IR/PassManager.h"
#include "llvm/Pass.h"

namespace llvm {

ModulePass *createMOSCallTreeClonePass();

struct MOSCallTreeClonePass : RequiredPassInfoMixin<MOSCallTreeClonePass> {
  PreservedAnalyses run(Module &M, ModuleAnalysisManager &AM);
};

} // namespace llvm

#endif // not LLVM_LIB_TARGET_MOS_MOSCALLTREECLONE_H
