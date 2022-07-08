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
#include "llvm/ADT/PostOrderIterator.h"
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
#include "llvm/Support/Casting.h"
#include <memory>
#include <utility>

#define DEBUG_TYPE "mos-zero-page-alloc"

using namespace llvm;

namespace {

// A rational number expressed as a numerator and denominator pair. Binary
// arithmetic operations between entries generally uses the denominator of the
// larger operand and rounds, avoiding the explosion typical of exact rational
// libraries in exchange. The tradeoff is that the results are only approximate.
struct Freq {
  uint64_t Num;
  uint64_t Denom;

  Freq(uint64_t Num, uint64_t Denom) : Num(Num), Denom(Denom) {}
  Freq(uint64_t Scalar) : Num(Scalar), Denom(1) {}
  Freq() : Num(0), Denom(1) {}

  Freq operator*(const Freq &Other) const {
    if (Denom > Other.Denom)
      return Other * *this;
    // WLOG the other denominator is larger, so divide out the smaller
    // denominator after the multiplication.
    return {Num * Other.Num / Denom, Other.Denom};
  }

  Freq &operator+=(const Freq &Other) {
    if (Denom > Other.Denom) {
      Num += Other.Num * Denom / Other.Denom;
    } else {
      Num *= Other.Denom;
      Num /= Denom;
      Denom = Other.Denom;
      Num += Other.Num;
    }
    return *this;
  }
};

Freq operator*(uint64_t Scalar, const Freq &F) { return Freq(Scalar) * F; }
} // namespace

static raw_ostream &operator<<(raw_ostream &OS, const Freq &Freq) {
  OS << Freq.Num << '/' << Freq.Denom;
  return OS;
}

namespace {

struct Candidate {
  size_t Size;
  // Benefit relative to local function entry.
  Freq Benefit;

  // One of
  Register CSR = 0;
  GlobalValue *GV = nullptr;
  int FI = -1;

  void dump(raw_ostream &OS, const TargetRegisterInfo &TRI) const {
    if (CSR)
      OS << "CSR " << printReg(CSR, &TRI);
    else if (GV)
      OS << "Global " << *GV;
    else
      OS << "Frame Index " << FI;
    OS << ", Size " << Size << ", Benefit " << Benefit;
  }
};

// A strongly connected component in the call graph. These SCCs themselves form
// a DAG used throughout this algorithm.
struct SCC {
  SmallVector<Function *> Funcs;
  SmallVector<SCC *> Callees;
  std::vector<Candidate> Candidates;
};

// A view of the SCC graph rooted at an externally callable node. Since we can't
// generally reason about the relative frequencies of calls from outside the TU,
// instead ZP's are assigned equally to EntryGraphs round-robin.
struct EntryGraph {
  SCC *Entry;
};

} // namespace

template <> struct llvm::GraphTraits<EntryGraph> {
  using NodeRef = SCC *;
  using ChildIteratorType = SmallVector<SCC *>::iterator;

  static NodeRef getEntryNode(const EntryGraph &EG) { return EG.Entry; }
  static ChildIteratorType child_begin(NodeRef N) { return N->Callees.begin(); }
  static ChildIteratorType child_end(NodeRef N) { return N->Callees.end(); }
};

namespace {

class MOSZeroPageAlloc : public ModulePass {
public:
  static char ID;

  MOSZeroPageAlloc() : ModulePass(ID) {
    llvm::initializeMOSZeroPageAllocPass(*PassRegistry::getPassRegistry());
  }

  bool runOnModule(Module &M) override;
  void getAnalysisUsage(AnalysisUsage &AU) const override;

private:
  struct SCCGraph {
    std::vector<SCC> SCCs;
    DenseMap<const Function *, SCC *> FunctionSCCs;
    // Corresponds to the external calling sentinel call graph node.
    SCC *ExternalCallingSCC;
  };

  MachineModuleInfo *MMI;
  SCCGraph buildSCCGraph(Module &M);

  std::vector<Candidate> collectCandidates(MachineFunction &MF);
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

static Freq getFreq(const BlockFrequencyInfo &BFI, MachineBasicBlock &MBB);

bool MOSZeroPageAlloc::runOnModule(Module &M) {
  MMI = &getAnalysis<MachineModuleInfoWrapperPass>().getMMI();

  LLVM_DEBUG(dbgs() << "*******************************************************"
                       "*************************\n");
  LLVM_DEBUG(dbgs() << "** MOS Zero Page Allocation\n");
  LLVM_DEBUG(dbgs() << "*******************************************************"
                       "*************************\n");

  SCCGraph SCCGraph = buildSCCGraph(M);

  LLVM_DEBUG(
      dbgs()
      << "Computing relative entry frequences for each global entry point.\n"
      << "*****************************************"
         "***************************************\n");
  std::vector<EntryGraph> EntryGraphs;
  for (SCC *Entry : SCCGraph.ExternalCallingSCC->Callees)
    EntryGraphs.push_back(EntryGraph{Entry});
  for (const EntryGraph &EG : EntryGraphs) {
    LLVM_DEBUG({
      dbgs() << "Entry SCC\n";
      for (const Function *F : EG.Entry->Funcs)
        dbgs() << "  " << F->getName() << "\n";
      dbgs() << '\n';
    });

    DenseMap<const SCC *, Freq> EntryFreqs;
    EntryFreqs[EG.Entry] = Freq{1, 1};
    // Callers are traversed before callees.
    for (SCC *Component : ReversePostOrderTraversal<EntryGraph>(EG)) {
      // Keep track of the original entry frequency of the SCC. Recursive calls
      // within the SCC should increase the entry frequency, but this shouldn't
      // compound, so they're scaled to the original entry frequency, not the
      // increased one.
      Freq OldEntryFreq = EntryFreqs[Component];

      // This is the current entry frequency of the SCC, based on SCC callers
      // and recursive calls seen so far.
      Freq EntryFreq = OldEntryFreq;
      LLVM_DEBUG(dbgs() << "  SCC " << EntryFreq << "\n");

      // Find all calls within the SCC and propagate entry frequencies across
      // the edges.
      DenseMap<const Function *, Freq> CalleeFreqs;
      for (Function *F : Component->Funcs) {
        LLVM_DEBUG(dbgs() << "    " << F->getName() << "\n");
        MachineFunction *MF = MMI->getMachineFunction(*F);
        if (!MF)
          continue;

        auto &BFI = getAnalysis<BlockFrequencyInfoWrapperPass>(*F).getBFI();
        for (MachineBasicBlock &MBB : *MF) {
          for (const MachineInstr &MI : MBB) {
            if (!MI.isCall())
              continue;
            for (const MachineOperand &MO : MI.operands()) {
              const Function *Callee = nullptr;
              if (MO.isGlobal())
                Callee = dyn_cast<Function>(MO.getGlobal());
              else if (MO.isSymbol())
                Callee = M.getFunction(MO.getSymbolName());
              if (!Callee)
                continue;
              Freq Freq = getFreq(BFI, MBB);
              if (is_contained(Component->Funcs, Callee)) {
                LLVM_DEBUG(dbgs() << "      Recursively calls "
                                  << Callee->getName() << " " << Freq << '\n');
                // Recursive calls are another way to enter the given SCC, so
                // increase the Entry frequency. Don't compound the increases
                // though; it's not worth risking overflowing the entry counts,
                // especially since possible recursion paths may be accidental.
                EntryFreq += OldEntryFreq * Freq;
                LLVM_DEBUG(dbgs() << "    SCC freq += " << OldEntryFreq * Freq
                                  << '\n');
              }
              // Defer handling normal calls until after the loop; the final
              // entry frequency won't be known until afterwards.
              LLVM_DEBUG(dbgs() << "      Calls " << Callee->getName() << ' '
                                << Freq << '\n');
              CalleeFreqs[Callee] += Freq;
            }
          }
        }
      }
      // Now that recursion has been handled, the final entry frequency is known
      // for the component. Apply it to each outgoing call from the SCC and
      // propagate the resulting entry frequencies to callee SCCs.
      for (const auto &KV : CalleeFreqs) {
        Freq Freq = EntryFreq * KV.second;
        LLVM_DEBUG(dbgs() << "    " << KV.first->getName() << " += " << Freq
                          << '\n');
        EntryFreqs[SCCGraph.FunctionSCCs[KV.first]] += Freq;
      }
    }
    LLVM_DEBUG(dbgs() << '\n');
  }

  return false;
}

// Contract the call graph into its strongly-connect components, then build a
// SCC DAG out of the results.
MOSZeroPageAlloc::SCCGraph MOSZeroPageAlloc::buildSCCGraph(Module &M) {
  auto &CG = getAnalysis<CallGraphWrapperPass>().getCallGraph();

  // Collect any external libcalls and add them to the call graph, which was
  // computed before code generation.
  for (auto &KV : CG) {
    CallGraphNode &CGN = *KV.second;
    if (!CGN.getFunction())
      continue;
    MachineFunction *MF = MMI->getMachineFunction(*CGN.getFunction());
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
          if (Callee && MMI->getMachineFunction(*Callee))
            CGN.addCalledFunction(nullptr, CG[Callee]);
        }
      }
    }
  }

  std::vector<SCC> SCCs;
  std::vector<SmallSet<const CallGraphNode *, 4>> SCCCallees;
  DenseMap<const CallGraphNode *, size_t> SCCIdx;
  for (auto I = scc_begin(&CG), E = scc_end(&CG); I != E; ++I) {
    SCCs.emplace_back();
    SCC &SCC = SCCs.back();
    SCCCallees.emplace_back();
    for (const CallGraphNode *N : *I) {
      SCCIdx[N] = SCCs.size() - 1;
      auto &Callees = SCCCallees.back();
      for (const auto &KV : *N)
        Callees.insert(KV.second);

      Function *F = N->getFunction();
      if (F)
        SCC.Funcs.push_back(F);
    }
    if (SCC.Funcs.size() != 1)
      continue;
    MachineFunction *MF = MMI->getMachineFunction(*SCC.Funcs.front());
    if (!MF)
      continue;
    const MOSFrameLowering &TFL =
        *MF->getSubtarget<MOSSubtarget>().getFrameLowering();
    if (TFL.usesStaticStack(*MF))
      SCC.Candidates = collectCandidates(*MF);
  }
  for (const auto &KV : enumerate(SCCCallees)) {
    SmallVector<SCC *> &Callees = SCCs[KV.index()].Callees;
    for (const CallGraphNode *Callee : KV.value())
      Callees.push_back(&SCCs[SCCIdx[Callee]]);
  }
  DenseMap<const Function *, SCC *> FunctionSCCs;
  for (SCC &Component : SCCs)
    for (const Function *F : Component.Funcs)
      FunctionSCCs[F] = &Component;
  SCC *ExternalCallingSCC = &SCCs[SCCIdx[CG.getExternalCallingNode()]];
  return {std::move(SCCs), std::move(FunctionSCCs), ExternalCallingSCC};
}

// For each SCC, find all of the local options for ZP allocation and score
// them relative to the function's entry frequency.
std::vector<Candidate>
MOSZeroPageAlloc::collectCandidates(MachineFunction &MF) {
  const MOSFrameLowering &TFL =
      *MF.getSubtarget<MOSSubtarget>().getFrameLowering();
  const MachineFrameInfo &MFI = MF.getFrameInfo();
  auto &BFI =
      getAnalysis<BlockFrequencyInfoWrapperPass>(MF.getFunction()).getBFI();

  LLVM_DEBUG(dbgs() << MF.getName() << ":\n");
  std::vector<Candidate> Candidates;

  Freq SaveFreq;
  Freq RestoreFreq;
  if (MFI.getSavePoint()) {
    SaveFreq = getFreq(BFI, *MFI.getSavePoint());
    MachineBasicBlock *RestoreBlock = MFI.getRestorePoint();
    // If RestoreBlock does not have any successor and is not a return block
    // then the end point is unreachable and we do not need to insert any
    // epilogue.
    if (!RestoreBlock->succ_empty() || RestoreBlock->isReturnBlock())
      RestoreFreq += getFreq(BFI, *RestoreBlock);
  } else {
    SaveFreq = getFreq(BFI, *MF.begin());
    for (MachineBasicBlock &MBB : MF) {
      if (MBB.isReturnBlock())
        RestoreFreq += getFreq(BFI, MBB);
    }
  }

  // Compute the benefit for moving each CSR to a static ZP location.
  BitVector SavedRegs;
  TFL.determineCalleeSaves(MF, SavedRegs, /*RS=*/nullptr);
  unsigned Idx = 0;
  for (Register Reg : SavedRegs.set_bits()) {
    if (!MOS::Imag8RegClass.contains(Reg))
      continue;

    size_t Size = 1;
    Freq Benefit;
    if (Idx++ < 4) {
      Benefit = 9 * SaveFreq;      // LDA ZP,PHA
      Benefit += 10 * RestoreFreq; // PLA,STA ZP
    } else {
      Benefit = 12 * SaveFreq;     // LDA ZP,STA ABS
      Benefit += 12 * RestoreFreq; // LDA ABS,STA ZP
    }
    Candidates.push_back(Candidate{Size, Benefit, Reg});
  }

  std::vector<SmallVector<MachineInstr *>> FIMIs(MFI.getObjectIndexEnd());
  DenseMap<GlobalValue *, Freq> GlobalBenefit;
  for (MachineBasicBlock &MBB : MF) {
    for (MachineInstr &MI : MBB) {
      for (const MachineOperand &MO : MI.operands()) {
        if (MO.isFI() && MO.getIndex() >= 0)
          FIMIs[MO.getIndex()].push_back(&MI);
        if (MI.mayLoadOrStore() && MO.isGlobal()) {
          // Generally moving an absolute reference to the zero page saves one
          // cycle and one byte.
          GlobalBenefit[const_cast<GlobalValue *>(MO.getGlobal())] +=
              2 * getFreq(BFI, MBB);
        }
      }
    }
  }
  for (const auto &KV : GlobalBenefit) {
    GlobalValue *GV = KV.first;
    const Freq &Benefit = KV.second;
    size_t Size =
        GV->getParent()->getDataLayout().getTypeSizeInBits(GV->getValueType()) *
        7 / 8;
    Candidates.push_back(Candidate{Size, Benefit, /*CSR=*/0, GV});
  }

  // Compute the benefit for moving each stack object to the ZP.
  for (int I = 0, E = MFI.getObjectIndexEnd(); I != E; ++I) {
    if (MFI.isDeadObjectIndex(I) || MFI.isVariableSizedObjectIndex(I))
      continue;

    Freq Benefit;
    for (MachineInstr *MI : FIMIs[I]) {
      // Generally moving an absolute reference to the zero page saves one
      // cycle and one byte.
      Benefit += 2 * getFreq(BFI, *MI->getParent());
    }
    Candidates.push_back(Candidate{static_cast<size_t>(MFI.getObjectSize(I)),
                                   Benefit, /*CSR=*/0, /*GV=*/nullptr, I});
  }

  LLVM_DEBUG({
    const TargetRegisterInfo &TRI = *MF.getSubtarget().getRegisterInfo();
    for (const Candidate &C : Candidates) {
      dbgs() << "  ";
      C.dump(dbgs(), TRI);
      dbgs() << '\n';
    }
    dbgs() << '\n';
  });
  return Candidates;
}

// We can't use machine block frequency due to a pass scheduling SNAFU, so
// approximate with the IR block frequencies.
Freq getFreq(const BlockFrequencyInfo &BFI, MachineBasicBlock &MBB) {
  if (!MBB.getBasicBlock())
    return Freq{BFI.getEntryFreq(), BFI.getEntryFreq()};
  return Freq{BFI.getBlockFreq(MBB.getBasicBlock()).getFrequency(),
              BFI.getEntryFreq()};
}

} // namespace

char MOSZeroPageAlloc::ID = 0;

INITIALIZE_PASS(MOSZeroPageAlloc, DEBUG_TYPE, "Allocate zero page", false,
                false)

ModulePass *llvm::createMOSZeroPageAllocPass() {
  return new MOSZeroPageAlloc();
}
