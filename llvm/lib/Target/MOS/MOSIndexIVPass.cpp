#include "MOSIndexIVPass.h"

#include "MOS.h"

#include "llvm/Analysis/ScalarEvolution.h"
#include "llvm/InitializePasses.h"
#include "llvm/Transforms/Scalar/LoopPassManager.h"
#include "llvm/Transforms/Utils/BasicBlockUtils.h"
#include "llvm/Transforms/Utils/LoopUtils.h"
#include "llvm/Transforms/Utils/ScalarEvolutionExpander.h"

#define DEBUG_TYPE "mos-indexiv"

using namespace llvm;

namespace {

// Locate GEP instructions in a loop that have SCEV's of the form Base + Index,
// where Base can be anything, but Index fits within an unsigned 8-bit integer.
// Creates IVs for such indices, then rewrites the GEPs to use their zero
// extension. This will allow the 6502's 8-bit indexed addressing modes to be
// used for operations involving these pointers.
//
// This can't easily be handled later in the pipeline, since loops may be
// unrolled very soon after this pass. Once unrolling happens, the knowledge
// that the index fits in 8 bits is lost.
struct MOSIndexIV : public LoopPass {
  static char ID; // Pass identification, replacement for typeid

  MOSIndexIV() : LoopPass(ID) {
    initializeMOSIndexIVPass(*PassRegistry::getPassRegistry());
  }

  bool runOnLoop(Loop *L, LPPassManager &LPM) override {
    // This is technically an optimization pass, and it depends on indvars and
    // LSR to work well, so skip if not optimizing.
    if (skipLoop(L))
      return false;

    LLVM_DEBUG(dbgs() << "***************************** MOS INDEX IV PASS "
                         "*****************************\n");

    auto *SE = &getAnalysis<ScalarEvolutionWrapperPass>().getSE();
    const DataLayout &DL = L->getHeader()->getModule()->getDataLayout();

    // InRange returns whether the given range can be contained within an
    // unsigned 8-bit index.
    const auto InRange = [](const ConstantRange &Range) {
      return Range.isAllNonNegative() &&
             Range.getUpper().ule(
                 APInt::getMaxValue(8).zext(Range.getBitWidth()));
    };

    Type *i8 = Type::getInt8Ty(SE->getContext());
    Type *i8Ptr = Type::getInt8PtrTy(SE->getContext());
    bool Changed = false;

    for (BasicBlock *B : L->blocks()) {
      for (BasicBlock::iterator I = B->begin(); I != B->end(); ++I) {
        // For now, only direct GEP instructions are handled, but in principle,
        // any other means of forming pointers should work as well.
        auto *GEP = dyn_cast<GetElementPtrInst>(I);
        if (!GEP)
          continue;
        LLVM_DEBUG(dbgs() << "Considering: " << *GEP << "\n");

        // Only pointer values with an additive recurrence can be made into
        // Base+Index.
        const auto *R = dyn_cast<SCEVAddRecExpr>(SE->getSCEV(GEP));
        if (!R || R->getLoop() != L)
          continue;
        LLVM_DEBUG(dbgs() << "SCEV: " << *R << "\n");

        // If the step doesn't fit in 8 bits, incrementing the index requires a
        // 16-bit add, so there's no point to the optimization.
        const auto *Step = R->getStepRecurrence(*SE);
        const auto StepRange = SE->getSignedRange(Step);
        if (!InRange(StepRange)) {
          LLVM_DEBUG(dbgs() << "Step range does not fit in 8 bits\n");
          LLVM_DEBUG(dbgs() << "Step: " << *Step << "\n");
          LLVM_DEBUG(dbgs() << "Range: " << StepRange << "\n");
          continue;
        }

        // The index must itself fit into 8 bits.
        const auto *Index =
            SE->getAddRecExpr(/*Start=*/SE->getConstant(R->getType(), 0), Step,
                              L, R->getNoWrapFlags());
        const auto IndexRange = SE->getSignedRange(Index);
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

        SCEVExpander Rewriter(*SE, DL, "mos-indexiv");
        // Don't overly expand the IVs, since we're late in the loop pipeline.
        Rewriter.disableCanonicalMode();
        Rewriter.setInsertPoint(&*I);

        // Get a value for the 16-bit base.
        Value *BaseVal = Rewriter.expandCodeFor(R->getStart());
        // Get a value for the 8-bit index.
        Value *IndexVal =
            Rewriter.expandCodeFor(SE->getTruncateExpr(Index, i8));

        // Emit an "uglygep" to avoid having to find a real GEP calculation that
        // leads to the SCEV. This always works, and preserves some aliasing
        // information.
        IRBuilder<> Builder(B, I);
        Value *V = Builder.CreateBitCast(BaseVal, i8Ptr);
        V = Builder.CreateGEP(
            i8, V, Builder.CreateZExt(IndexVal, DL.getIndexType(i8Ptr)),
            "uglygep");
        V = Builder.CreateBitCast(V, GEP->getType());

        auto Inst = I;
        --I;
        ReplaceInstWithValue(I->getParent()->getInstList(), Inst, V);
      }
    }

    LLVM_DEBUG(dbgs() << "*****************************************************"
                         "***************************\n");
    return Changed;
  }

  void getAnalysisUsage(AnalysisUsage &AU) const override {
    AU.setPreservesCFG();
    getLoopAnalysisUsage(AU);
  }
};

} // namespace

char MOSIndexIV::ID = 0;

INITIALIZE_PASS_BEGIN(MOSIndexIV, "mos-indexiv",
                      "Introduce 8-bit IVs for GEP indices", false, false)
INITIALIZE_PASS_DEPENDENCY(LoopPass)
INITIALIZE_PASS_END(MOSIndexIV, "mos-indexiv",
                    "Introduce 8-bit IVs for GEP indices", false, false)

LoopPass *llvm::createMOSIndexIVPass() { return new MOSIndexIV(); }
