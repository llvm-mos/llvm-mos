//===-- MOSZeroPageAlloc.cpp - MOS Zero Page Allocation ------------------===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
///
/// \file
/// This file defines the MOS zero page allocation pass.
///
//===----------------------------------------------------------------------===//

#include "MOSZeroPageAlloc.h"

#include "MCTargetDesc/MOSMCTargetDesc.h"
#include "MOS.h"
#include "MOSFrameLowering.h"
#include "MOSRegisterInfo.h"
#include "MOSSubtarget.h"
#include "llvm/ADT/SCCIterator.h"
#include "llvm/Analysis/BasicAliasAnalysis.h"
#include "llvm/Analysis/BlockFrequencyInfo.h"
#include "llvm/Analysis/CallGraph.h"
#include "llvm/Analysis/DominanceFrontier.h"
#include "llvm/Analysis/GlobalsModRef.h"
#include "llvm/Analysis/IVUsers.h"
#include "llvm/Analysis/LoopInfo.h"
#include "llvm/Analysis/MemoryDependenceAnalysis.h"
#include "llvm/Analysis/ScalarEvolution.h"
#include "llvm/Analysis/ScalarEvolutionAliasAnalysis.h"
#include "llvm/CodeGen/LazyMachineBlockFrequencyInfo.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineBlockFrequencyInfo.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineModuleInfo.h"
#include "llvm/CodeGen/TargetFrameLowering.h"
#include "llvm/CodeGen/TargetSubtargetInfo.h"
#include "llvm/IR/Dominators.h"
#include "llvm/Support/BlockFrequency.h"

#define DEBUG_TYPE "mos-zero-page-alloc"

using namespace llvm;

namespace {

class MOSZeroPageAlloc : public ModulePass {
public:
  static char ID;

  MOSZeroPageAlloc() : ModulePass(ID) {
    llvm::initializeMOSZeroPageAllocPass(*PassRegistry::getPassRegistry());
  }

  bool runOnModule(Module &M) override;
  void getAnalysisUsage(AnalysisUsage &AU) const override;
};

void MOSZeroPageAlloc::getAnalysisUsage(AnalysisUsage &AU) const {
  AU.setPreservesCFG();
  AU.addRequired<MachineModuleInfoWrapperPass>();
  AU.addPreserved<MachineModuleInfoWrapperPass>();
  AU.addRequired<BlockFrequencyInfoWrapperPass>();
  AU.addRequired<CallGraphWrapperPass>();
  AU.addPreserved<CallGraphWrapperPass>();

  AU.addPreserved<BasicAAWrapperPass>();
  AU.addPreserved<DominanceFrontierWrapperPass>();
  AU.addPreserved<DominatorTreeWrapperPass>();
  AU.addPreserved<AAResultsWrapperPass>();
  AU.addPreserved<GlobalsAAWrapperPass>();
  AU.addPreserved<IVUsersWrapperPass>();
  AU.addPreserved<LoopInfoWrapperPass>();
  AU.addPreserved<MemoryDependenceWrapperPass>();
  AU.addPreserved<ScalarEvolutionWrapperPass>();
  AU.addPreserved<SCEVAAWrapperPass>();
}

bool MOSZeroPageAlloc::runOnModule(Module &M) {
  auto &MMI = getAnalysis<MachineModuleInfoWrapperPass>().getMMI();
  auto &CG = getAnalysis<CallGraphWrapperPass>().getCallGraph();

  LLVM_DEBUG(dbgs() << "*******************************************************"
                       "*************************\n");
  LLVM_DEBUG(dbgs() << "** MOS Zero Page Allocation\n");
  LLVM_DEBUG(dbgs() << "*******************************************************"
                       "*************************\n");

  // Collect any external libcalls and add them to the call graph, which was
  // computed before code generation.
  for (auto &KV : CG) {
    CallGraphNode &CGN = *KV.second;
    if (!CGN.getFunction())
      continue;
    MachineFunction *MF = MMI.getMachineFunction(*CGN.getFunction());
    if (!MF)
      continue;
    for (const MachineBasicBlock &MBB : *MF) {
      for (const MachineInstr &MI : MBB) {
        if (!MI.isCall())
          continue;
        for (const MachineOperand &MO : MI.operands()) {
          if (!MO.isSymbol())
            continue;
          Function *Callee = M.getFunction(MO.getSymbolName());
          if (Callee && MMI.getMachineFunction(*Callee))
            CGN.addCalledFunction(nullptr, CG[Callee]);
        }
      }
    }
  }

  // Extract the list of strongly-connected components from the call graph.
  struct SCC {
    CallGraphNode *Node; // Nullptr if recursive SCC.
  };
  std::vector<SCC> SCCs;
  for (auto I = scc_begin(&CG), E = scc_end(&CG); I != E; ++I) {
    SCCs.emplace_back();
    if (!I.hasCycle()) {
      assert(I->size() == 1);
      SCCs.push_back(SCC{I->front()});
    }
  }

  for (const SCC &Component : SCCs) {
    if (!Component.Node || !Component.Node->getFunction())
      continue;
    MachineFunction *MF =
        MMI.getMachineFunction(*Component.Node->getFunction());
    if (!MF)
      continue;

    const MOSFrameLowering &TFL =
        *MF->getSubtarget<MOSSubtarget>().getFrameLowering();
    const TargetRegisterInfo &TRI = *MF->getSubtarget().getRegisterInfo();
    const MachineFrameInfo &MFI = MF->getFrameInfo();

    if (!TFL.usesStaticStack(*MF))
      continue;

    LLVM_DEBUG(dbgs() << MF->getName() << " candidates:\n");

    // We can't use machine block frequency due to a pass scheduling SNAFU, so
    // approximate with the IR block frequencies.
    auto &BFI =
        getAnalysis<BlockFrequencyInfoWrapperPass>(MF->getFunction()).getBFI();
    const auto GetFreq = [&](const MachineBasicBlock &MBB) -> BlockFrequency {
      if (!MBB.getBasicBlock())
        return 0;
      return BFI.getBlockFreq(MBB.getBasicBlock());
    };

    BlockFrequency SaveFreq;
    BlockFrequency RestoreFreq;
    if (MFI.getSavePoint()) {
      SaveFreq = BFI.getBlockFreq(MFI.getSavePoint()->getBasicBlock());
      MachineBasicBlock *RestoreBlock = MFI.getRestorePoint();
      // If RestoreBlock does not have any successor and is not a return block
      // then the end point is unreachable and we do not need to insert any
      // epilogue.
      if (!RestoreBlock->succ_empty() || RestoreBlock->isReturnBlock())
        RestoreFreq += GetFreq(*RestoreBlock);
    } else {
      SaveFreq = GetFreq(*MF->begin());
      for (const MachineBasicBlock &MBB : *MF) {
        if (MBB.isReturnBlock())
          RestoreFreq += GetFreq(MBB);
      }
    }

    // Compute the benefit for moving each CSR to a static ZP location.
    BitVector SavedRegs;
    TFL.determineCalleeSaves(*MF, SavedRegs, /*RS=*/nullptr);
    unsigned Idx = 0;
    uint64_t EntryFreq = BFI.getEntryFreq();
    for (Register Reg : SavedRegs.set_bits()) {
      if (!MOS::Imag8RegClass.contains(Reg))
        continue;

      LLVM_DEBUG(dbgs() << "  CSR " << printReg(Reg, &TRI) << ":\n");
      LLVM_DEBUG(dbgs() << "    Size: 1\n");

      BlockFrequency Benefit;
      if (Idx++ < 4) {
        Benefit = SaveFreq.getFrequency() * 9;      // LDA ZP,PHA
        Benefit += RestoreFreq.getFrequency() * 10; // PLA,STA ZP
      } else {
        Benefit = SaveFreq.getFrequency() * 12;     // LDA ZP,STA ABS
        Benefit += RestoreFreq.getFrequency() * 12; // LDA ABS,STA ZP
      }
      LLVM_DEBUG(dbgs() << "    BenefitNum: " << Benefit.getFrequency()
                        << '\n');
      LLVM_DEBUG(dbgs() << "    BenefitDenom: " << EntryFreq << '\n');
    }

    std::vector<SmallVector<const MachineInstr *>> FIMIs(
        MFI.getObjectIndexEnd());
    for (const MachineBasicBlock &MBB : *MF)
      for (const MachineInstr &MI : MBB)
        for (const MachineOperand &MO : MI.operands())
          if (MO.isFI() && MO.getIndex() >= 0)
            FIMIs[MO.getIndex()].push_back(&MI);

    // Compute the benefit for moving each stack object to the ZP.
    for (int I = 0, E = MFI.getObjectIndexEnd(); I != E; ++I) {
      if (MFI.isDeadObjectIndex(I) || MFI.isVariableSizedObjectIndex(I))
        continue;
      LLVM_DEBUG(dbgs() << "  FI #" << I << ":\n");
      LLVM_DEBUG(dbgs() << "    Size: " << MFI.getObjectSize(I) << '\n');

      BlockFrequency Benefit;
      for (const MachineInstr *MI : FIMIs[I]) {
        if (!MI->getParent()->getBasicBlock())
          continue;
        // Generally moving an absolute reference to the zero page saves one
        // cycle and one byte.
        Benefit +=
            2 *
            BFI.getBlockFreq(MI->getParent()->getBasicBlock()).getFrequency();
      }
      LLVM_DEBUG(dbgs() << "    BenefitNum: " << Benefit.getFrequency()
                        << '\n');
      LLVM_DEBUG(dbgs() << "    BenefitDenom: " << EntryFreq << '\n');
    }

    LLVM_DEBUG(dbgs() << '\n');
  }

  return false;
}

} // namespace

char MOSZeroPageAlloc::ID = 0;

INITIALIZE_PASS(MOSZeroPageAlloc, DEBUG_TYPE, "Allocate zero page", false,
                false)

ModulePass *llvm::createMOSZeroPageAllocPass() {
  return new MOSZeroPageAlloc();
}
