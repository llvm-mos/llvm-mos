//===-- MOSIndexIV.cpp - MOS Index IV Pass --------------------------------===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file defines the MOS Index IV pass.
//
// This pass locates GEP instructions in a loop that have SCEV's of the form
// Base + Index, where Index fits within an unsigned 8-bit integer. It creates
// dedicated IVs for such indices, then rewrites the GEPs to use their zero
// extension. This allows the backend to recognize that the high byte of the
// index is zero and to use the 8-bit indexed addressing modes if appropriate.
//===----------------------------------------------------------------------===//

#include "MOSIndexIV.h"
#include "MOSInstrInfo.h"

#include "llvm/Analysis/ScalarEvolution.h"
#include "llvm/Transforms/Scalar/LoopPassManager.h"
#include "llvm/Transforms/Utils/BasicBlockUtils.h"
#include "llvm/Transforms/Utils/LoopUtils.h"
#include "llvm/Transforms/Utils/ScalarEvolutionExpander.h"

#define DEBUG_TYPE "mos-indexiv"

using namespace llvm;

PreservedAnalyses MOSIndexIV::run(Loop &L, LoopAnalysisManager &AM,
                                  LoopStandardAnalysisResults &AR,
                                  LPMUpdater &) {
  LLVM_DEBUG(dbgs() << "***************************** MOS INDEX IV PASS "
                       "*****************************\n");

  auto &SE = AR.SE;
  const DataLayout &DL = L.getHeader()->getModule()->getDataLayout();

  // InRange returns whether the given range can be contained within an
  // unsigned 8-bit index.
  const auto InRange = [](const ConstantRange &Range) {
    return Range.isAllNonNegative() &&
           Range.getUpper().ule(
               APInt::getMaxValue(8).zext(Range.getBitWidth()));
  };

  Type *I8 = Type::getInt8Ty(SE.getContext());
  Type *Ptr = PointerType::get(SE.getContext(), 0);
  bool Changed = false;

  for (BasicBlock *B : L.blocks()) {
    for (auto I = B->begin(), E = B->end(); I != E; ++I) {
      // For now, only direct GEP instructions are handled, but in principle,
      // any other means of forming pointers should work as well.
      auto *GEP = dyn_cast<GetElementPtrInst>(I);
      if (!GEP)
        continue;
      LLVM_DEBUG(dbgs() << "Considering: " << *GEP << "\n");

      // Only pointer values with an additive recurrence can be made into
      // Base+Index.
      const auto *R = dyn_cast<SCEVAddRecExpr>(SE.getSCEV(GEP));
      if (!R || R->getLoop() != &L)
        continue;
      // Only 16-bit pointer values are currently supported by this pass.
      if (R->getType()->getPointerAddressSpace() != MOS::AS_Memory)
        continue;
      LLVM_DEBUG(dbgs() << "SCEV: " << *R << "\n");

      // If the step doesn't fit in 8 bits, incrementing the index requires a
      // 16-bit add, so there's no point to the optimization.
      const auto *Step = R->getStepRecurrence(SE);
      const auto StepRange = SE.getSignedRange(Step);
      if (!InRange(StepRange)) {
        LLVM_DEBUG(dbgs() << "Step range does not fit in 8 bits\n");
        LLVM_DEBUG(dbgs() << "Step: " << *Step << "\n");
        LLVM_DEBUG(dbgs() << "Range: " << StepRange << "\n");
        continue;
      }

      // The index must itself fit into 8 bits.
      const auto *Index =
          SE.getAddRecExpr(/*Start=*/SE.getConstant(R->getType(), 0), Step, &L,
                           R->getNoWrapFlags());
      const auto IndexRange = SE.getSignedRange(Index);
      if (!InRange(IndexRange)) {
        LLVM_DEBUG(dbgs() << "Index range does not fit in 8 bits\n");
        LLVM_DEBUG(dbgs() << "Index: " << *Index << "\n");
        LLVM_DEBUG(dbgs() << "Range: " << IndexRange << "\n");
        continue;
      }

      // Once the step and index are both known to fit in 8 bits, we can
      // always rewrite to a 16-bit base + 8-bit index.
      LLVM_DEBUG(dbgs() << "Rewriting to 8-bit index.\n");
      Changed = true;

      SCEVExpander Rewriter(SE, DL, "mos-indexiv");
      // The IVs should be computed from already available subexpressions
      // wherever possible. Canonical mode instead expands them fully to make
      // them easier to analyze.
      Rewriter.disableCanonicalMode();

      Rewriter.setInsertPoint(&*I);

      // Get a value for the 16-bit base.
      Value *BaseVal = Rewriter.expandCodeFor(R->getStart());
      // Get a value for the 8-bit index.
      Value *IndexVal = Rewriter.expandCodeFor(SE.getTruncateExpr(Index, I8));

      // Emit an "uglygep" to avoid having to find a real GEP calculation that
      // leads to the SCEV. This always works, and still preserves at least
      // some aliasing information.
      IRBuilder<> Builder(B, I);
      Value *V = Builder.CreateBitCast(BaseVal, Ptr);
      V = Builder.CreateGEP(
          I8, V, Builder.CreateZExt(IndexVal, DL.getIndexType(Ptr)), "uglygep");
      V = Builder.CreateBitCast(V, GEP->getType());

      auto Inst = I;
      --I;
      ReplaceInstWithValue(Inst, V);
    }
  }

  LLVM_DEBUG(dbgs() << "*****************************************************"
                       "***************************\n");
  if (!Changed)
    return PreservedAnalyses::all();
  auto PA = getLoopPassPreservedAnalyses();
  PA.preserveSet<CFGAnalyses>();
  return PA;
}
