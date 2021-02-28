//===-- MOSIndexIV.h - MOS Index IV Pass ------------------------*- C++ -*-===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file declares the MOS Index IV pass.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MOS_MOSINDEXIV_H
#define LLVM_LIB_TARGET_MOS_MOSINDEXIV_H

#include "llvm/Analysis/LoopAnalysisManager.h"
#include "llvm/Transforms/Scalar/LoopPassManager.h"

namespace llvm {

struct MOSIndexIV : public PassInfoMixin<MOSIndexIV> {
  PreservedAnalyses run(Loop &L, LoopAnalysisManager &AM,
                        LoopStandardAnalysisResults &AR,
                        LPMUpdater &U);
};

} // end namespace llvm

#endif // not LLVM_LIB_TARGET_MOS_MOSINDEXIV_H
