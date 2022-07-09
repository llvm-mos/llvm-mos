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
#include "llvm/CodeGen/TargetRegisterInfo.h"
#include "llvm/CodeGen/TargetSubtargetInfo.h"
#include "llvm/IR/Dominators.h"
#include "llvm/Support/BlockFrequency.h"
#include "llvm/Support/Casting.h"
#include <algorithm>
#include <memory>
#include <utility>

#define DEBUG_TYPE "mos-zero-page-alloc"

using namespace llvm;

namespace {

cl::opt<uint64_t> ZPAvail("zp-avail",
                          cl::desc("Number of bytes of zero page available for "
                                   "the compiler in the current TU"),
                          cl::value_desc("bytes"));

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

  Freq operator+(const Freq &Other) const {
    if (Denom < Other.Denom)
      return Other + *this;
    if (Denom == Other.Denom)
      return {Num + Other.Num, Denom};
    return *this + scaleToDenom(Other);
  }

  Freq &operator+=(const Freq &Other) {
    *this = *this + Other;
    return *this;
  }

  Freq operator*(const Freq &Other) const {
    if (Denom < Other.Denom)
      return Other * *this;
    if (Denom == Other.Denom)
      return {Num * Other.Num / Denom, Denom};
    return *this * scaleToDenom(Other);
  }
  Freq &operator*=(const Freq &Other) {
    *this = *this * Other;
    return *this;
  }

  Freq operator-(const Freq &Other) const { return *this + Freq(-1) * Other; }
  Freq &operator-=(const Freq &Other) {
    *this = *this - Other;
    return *this;
  }

  Freq operator/(const Freq &Other) const {
    return *this * Freq(Other.Denom, Other.Num);
  }
  Freq &operator/=(const Freq &Other) {
    *this = *this / Other;
    return *this;
  }

  bool operator==(const Freq &Other) const {
    return Num == Other.Num && Denom == Other.Denom;
  }
  bool operator!=(const Freq &Other) const { return !(*this == Other); }
  bool operator<(const Freq &Other) const {
    return Num * Other.Denom < Other.Num * Denom;
  }
  bool operator<=(const Freq &Other) const {
    return *this == Other || *this < Other;
  }
  bool operator>(const Freq &Other) const { return Other < *this; }
  bool operator>=(const Freq &Other) const { return Other <= *this; }

private:
  Freq scaleToDenom(const Freq &Other) const {
    return Freq{Other.Num * Denom / Other.Denom, Denom};
  }
};

Freq operator*(uint64_t Scalar, const Freq &F) { return Freq(Scalar) * F; }
} // namespace

static raw_ostream &operator<<(raw_ostream &OS, const Freq &Freq) {
  OS << Freq.Num << '/' << Freq.Denom;
  return OS;
}

namespace {

struct SCC;

struct Candidate {
  MachineFunction *MF;
  size_t Size;
  // Benefit relative to local function entry.
  Freq Benefit;

  // One of
  Register CSR = 0;
  GlobalValue *GV = nullptr;
  int FI = -1;

  // The number of ZP locations tentatively assigned to this candidate. When
  // this reaches Size, the assignment is final.
  unsigned Assigned = 0;

  SCC *SCC = nullptr;
};

struct EntryCandidate {
  Candidate *Cand;
  Freq Benefit;
};

} // namespace

raw_ostream &operator<<(raw_ostream &OS, const Candidate &C) {
  OS << C.MF->getName() << ", ";
  if (C.CSR)
    OS << "CSR " << printReg(C.CSR, C.MF->getSubtarget().getRegisterInfo());
  else if (C.GV)
    OS << "Global " << *C.GV;
  else
    OS << "Frame Index " << C.FI;
  OS << ", Size " << C.Size << ", Benefit " << C.Benefit;
  if (C.Assigned)
    OS << ", Assigned " << C.Assigned;
  return OS;
}

raw_ostream &operator<<(raw_ostream &OS, const EntryCandidate &EC) {
  OS << *EC.Cand << ", Global benefit " << EC.Benefit;
  return OS;
}

namespace {

// A strongly connected component in the call graph. These SCCs themselves form
// a DAG used throughout this algorithm.
struct SCC {
  SmallVector<Function *> Funcs;
  SmallVector<SCC *> Callees;
  SmallVector<SCC *> Callers;
  std::vector<Candidate> Candidates;

  // Offset of the zero page area for this SCC from the start of the zero page
  // section for this TU.
  size_t ZPOffset = 0;

  // Current size of the zero page area of this SCC, in bytes.
  size_t ZPSize = 0;

  // The maximum amount of ZP used by any path that includes this SCC.
  size_t MaxZPSize = 0;
};

// A view of the SCC graph rooted at an externally callable node. Since we can't
// generally reason about the relative frequencies of calls from outside the TU,
// instead ZP's are assigned equally to EntryGraphs round-robin.
struct EntryGraph {
  SCC *Entry;

  // Candidates scored for this entry point, ordered best first.
  std::vector<EntryCandidate> Candidates = {};

  size_t NextCand = 0;
};

struct SCCGraph {
  std::vector<SCC> SCCs;
  DenseMap<const Function *, SCC *> FunctionSCCs;
  // Corresponds to the external calling sentinel call graph node.
  SCC *ExternalCallingSCC;
};

} // namespace

template <> struct llvm::GraphTraits<EntryGraph> {
  using NodeRef = SCC *;
  using ChildIteratorType = SmallVector<SCC *>::iterator;

  static NodeRef getEntryNode(const EntryGraph &EG) { return EG.Entry; }
  static ChildIteratorType child_begin(NodeRef N) { return N->Callees.begin(); }
  static ChildIteratorType child_end(NodeRef N) { return N->Callees.end(); }
};

template <> struct llvm::GraphTraits<SCC> {
  using NodeRef = SCC *;
  using ChildIteratorType = SmallVector<SCC *>::iterator;

  static NodeRef getEntryNode(const SCC &SCC) {
    return const_cast<struct SCC *>(&SCC);
  }
  static ChildIteratorType child_begin(NodeRef N) { return N->Callees.begin(); }
  static ChildIteratorType child_end(NodeRef N) { return N->Callees.end(); }
};

template <> struct llvm::GraphTraits<SCCGraph> {
  using NodeRef = SCC *;
  using ChildIteratorType = SmallVector<SCC *>::iterator;

  static NodeRef getEntryNode(const SCCGraph &G) {
    return G.ExternalCallingSCC;
  }
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
  MachineModuleInfo *MMI;
  SCCGraph buildSCCGraph(Module &M);

  std::vector<Candidate> collectCandidates(MachineFunction &MF);

  std::vector<EntryGraph> buildEntryGraphs(Module &M, SCCGraph &SCCGraph);
  bool assignZPs(SCCGraph &SCCGraph, std::vector<EntryGraph>::iterator Begin,
                 std::vector<EntryGraph>::iterator End);
  bool assignZP(SCCGraph &SCCGraph, EntryGraph &EG);
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
  if (!ZPAvail)
    return false;

  MMI = &getAnalysis<MachineModuleInfoWrapperPass>().getMMI();

  LLVM_DEBUG(dbgs() << "*******************************************************"
                       "*************************\n");
  LLVM_DEBUG(dbgs() << "** MOS Zero Page Allocation\n");
  LLVM_DEBUG(dbgs() << "*******************************************************"
                       "*************************\n");

  SCCGraph SCCGraph = buildSCCGraph(M);

  std::vector<EntryGraph> EntryGraphs = buildEntryGraphs(M, SCCGraph);

  // Interrupt-norecurse functions get absolute priority, since they're almost
  // always time-sensitive, and they have an abnormally high number of CSRs.
  const auto RegularEGBegin = partition(EntryGraphs, [](EntryGraph &EG) {
    return !EG.Candidates.empty() &&
           EG.Entry->Funcs.front()->hasFnAttribute("interrupt_norecurse");
  });

  // Assign ZP locations to entry graphs round-robin until no candidates remain.
  LLVM_DEBUG(dbgs() << "Assigning ZP to candidates:\n");
  while (assignZPs(SCCGraph, EntryGraphs.begin(), RegularEGBegin))
    ;
  while (assignZPs(SCCGraph, RegularEGBegin, EntryGraphs.end()))
    ;

  return false;
}

// Contract the call graph into its strongly-connect components, then build a
// SCC DAG out of the results.
SCCGraph MOSZeroPageAlloc::buildSCCGraph(Module &M) {
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

  for (SCC &Component : SCCs) {
    for (Candidate &C : Component.Candidates)
      C.SCC = &Component;
    for (const Function *F : Component.Funcs)
      FunctionSCCs[F] = &Component;
    for (SCC *Callee : Component.Callees)
      Callee->Callers.push_back(&Component);
  }
  SCC *ExternalCallingSCC = &SCCs[SCCIdx[CG.getExternalCallingNode()]];

  LLVM_DEBUG({
    dbgs() << "Candidates:\n";
    for (SCC &Component : SCCs)
      for (const Candidate &C : Component.Candidates)
        dbgs() << "  " << C << '\n';
    dbgs() << '\n';
  });

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
    Benefit /= Size;
    Candidates.push_back(Candidate{&MF, Size, Benefit, Reg});
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
    Freq Benefit = KV.second;
    size_t Size = (GV->getParent()->getDataLayout().getTypeSizeInBits(
                       GV->getValueType()) +
                   7) /
                  8;
    Benefit /= Size;
    Candidates.push_back(Candidate{&MF, Size, Benefit, /*CSR=*/0, GV});
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
    auto Size = static_cast<size_t>(MFI.getObjectSize(I));
    Benefit /= Size;
    Candidates.push_back(
        Candidate{&MF, Size, Benefit, /*CSR=*/0, /*GV=*/nullptr, I});
  }

  return Candidates;
}

// For each globally-callable entry point, trace the SCCs transitively callable
// from that entry, and assign each their relative frequency to the entry point.
// From these frequences, an ordered set of candidates is derived for each entry
// point.
std::vector<EntryGraph> MOSZeroPageAlloc::buildEntryGraphs(Module &M,
                                                           SCCGraph &SCCGraph) {
  std::vector<EntryGraph> EntryGraphs;
  for (SCC *Entry : SCCGraph.ExternalCallingSCC->Callees)
    EntryGraphs.push_back(EntryGraph{Entry});
  for (EntryGraph &EG : EntryGraphs) {
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
      // for the component.

      // Apply the final entry frequency to the candidates.
      for (Candidate &Cand : Component->Candidates)
        EG.Candidates.push_back(
            EntryCandidate{&Cand, EntryFreq * Cand.Benefit});

      // Apply the final entry frequency to each outgoing call from the SCC and
      // propagate the resulting entry frequencies to callee SCCs.
      for (const auto &KV : CalleeFreqs) {
        Freq Freq = EntryFreq * KV.second;
        LLVM_DEBUG(dbgs() << "    " << KV.first->getName() << " += " << Freq
                          << '\n');
        EntryFreqs[SCCGraph.FunctionSCCs[KV.first]] += Freq;
      }
    }

    stable_sort(EG.Candidates,
                [](const EntryCandidate &A, const EntryCandidate &B) {
                  return A.Benefit > B.Benefit;
                });
    LLVM_DEBUG({
      dbgs() << "\n  Candidates:\n";
      for (EntryCandidate &Cand : EG.Candidates)
        dbgs() << "    " << Cand << '\n';
      dbgs() << '\n';
    });
  }
  return EntryGraphs;
}

bool MOSZeroPageAlloc::assignZPs(SCCGraph &SCCGraph,
                                 std::vector<EntryGraph>::iterator Begin,
                                 std::vector<EntryGraph>::iterator End) {
  bool AssignedAny = false;
  for (auto I = Begin; I != End; ++I)
    AssignedAny |= assignZP(SCCGraph, *I);
  return AssignedAny;
}

bool MOSZeroPageAlloc::assignZP(SCCGraph &SCCGraph, EntryGraph &EG) {
  // Advance to first unassigned candidate.
  for (;; ++EG.NextCand) {
    if (EG.NextCand == EG.Candidates.size())
      return false;
    EntryCandidate &Cand = EG.Candidates[EG.NextCand];
    // Another entry path may have already assigned the candidate.
    if (Cand.Cand->Assigned == Cand.Cand->Size)
      continue;
    // If the candidate is too big to fit, no reason to start allocating bytes
    // to it.
    if (!Cand.Cand->Assigned &&
        Cand.Cand->Size + Cand.Cand->SCC->MaxZPSize > ZPAvail)
      continue;

    break;
  }
  EntryCandidate &Cand = EG.Candidates[EG.NextCand];

  ++Cand.Cand->Assigned;

  LLVM_DEBUG(dbgs() << "Entry " << EG.Entry->Funcs.front()->getName()
                    << ", Func " << Cand << '\n';);

  if (Cand.Cand->Assigned + Cand.Cand->SCC->MaxZPSize > ZPAvail) {
    LLVM_DEBUG(dbgs() << "No longer fits; unassigning.\n");
    size_t NumToReassign = Cand.Cand->Assigned;
    Cand.Cand->Assigned = 0;
    bool AssignedAny = false;
    for (size_t I = 0; I < NumToReassign; ++I)
      AssignedAny |= assignZP(SCCGraph, EG);
    return AssignedAny;
  }

  if (Cand.Cand->Assigned < Cand.Cand->Size)
    return true;

  Cand.Cand->SCC->ZPSize += Cand.Cand->Assigned;

  // Allocating a ZP can change the offsets of transitive callees, so propagate
  // those downward.
  for (SCC *Comp : ReversePostOrderTraversal<SCC>(*Cand.Cand->SCC)) {
    size_t EndOffset = Comp->ZPOffset + Comp->ZPSize;
    for (struct SCC *Callee : Comp->Callees)
      Callee->ZPOffset = std::max(Callee->ZPOffset, EndOffset);
  }

  // From the new offsets, the new max ZP sizes for paths through each node can
  // be computed. They're trivial at the leaves and computable upward.
  for (SCC *Comp : post_order(SCCGraph)) {
    if (Comp->Callees.empty()) {
      Comp->MaxZPSize = Comp->ZPOffset + Comp->ZPSize;
      continue;
    }

    Comp->MaxZPSize = 0;
    for (struct SCC *Callee : Comp->Callees)
      Comp->MaxZPSize = std::max(Comp->MaxZPSize, Callee->MaxZPSize);
  }

  return true;
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
