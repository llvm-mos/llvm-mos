//===-- MOSStaticStackAlloc.cpp - MOS Static Stack Allocation -------------===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file defines the MOS static stack allocation pass.
//
// The code generator lowers accesses to regions of the stack frame that can be
// allocated statically as a target-specific index operand. This pass allocates
// a global variable for the static stack, then the target-specific indices in a
// function with references to the corresponding offset within that global.
//
//===----------------------------------------------------------------------===//

#include "MOSStaticStackAlloc.h"

#include "MCTargetDesc/MOSMCTargetDesc.h"
#include "MOS.h"
#include "MOSCallGraphUtils.h"
#include "MOSFrameLowering.h"
#include "MOSMachineFunctionInfo.h"
#include "MOSSubtarget.h"

#include "llvm/ADT/SCCIterator.h"
#include "llvm/Analysis/CallGraph.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineModuleInfo.h"
#include "llvm/CodeGen/TargetFrameLowering.h"
#include "llvm/CodeGen/TargetSubtargetInfo.h"
#include "llvm/IR/GlobalVariable.h"
#include "llvm/IR/Module.h"
#include "llvm/PassRegistry.h"
#include "llvm/Support/ErrorHandling.h"

#define DEBUG_TYPE "mos-static-stack-alloc"

using namespace llvm;

namespace {

class MOSStaticStackAlloc : public ModulePass {
public:
  static char ID;

  MOSStaticStackAlloc() : ModulePass(ID) {
    llvm::initializeMOSStaticStackAllocPass(*PassRegistry::getPassRegistry());
  }

  bool runOnModule(Module &M) override;
  void getAnalysisUsage(AnalysisUsage &AU) const override;
};

void MOSStaticStackAlloc::getAnalysisUsage(AnalysisUsage &AU) const {
  AU.addRequired<MachineModuleInfoWrapperPass>();
  AU.addPreserved<MachineModuleInfoWrapperPass>();
  AU.addRequired<CallGraphWrapperPass>();
}

bool MOSStaticStackAlloc::runOnModule(Module &M) {
  auto &MMI = getAnalysis<MachineModuleInfoWrapperPass>().getMMI();
  auto &CG = getAnalysis<CallGraphWrapperPass>().getCallGraph();

  mos::addLibcallEdges(CG, MMI);
  mos::addExternalEdges(CG);

  // Extract the list of strongly-connected components from the call graph, and
  // make a note of which SCC contains each node.
  DenseMap<CallGraphNode *, uint64_t> SCCID;
  struct SCC {
    SmallVector<CallGraphNode *, 1> Nodes;
    uint64_t Offset = 0;
  };
  std::vector<SCC> SCCs;
  std::vector<uint64_t> SCCOffsets;
  for (auto I = scc_begin(&CG), E = scc_end(&CG); I != E; ++I) {
    SCCs.emplace_back();
    for (CallGraphNode *CGN : *I) {
      SCCID[CGN] = SCCs.size() - 1;
      SCCs.back().Nodes.push_back(CGN);
    }
  }

  // For each SCC, determine the set of calling SCCs, that is, those containing
  // at least one node that calls at least one node in the SCC. The calling SCCs
  // must be placed higher on the static stack than the called SCC; otherwise,
  // their stack would conflict.
  std::map<SCC *, SmallPtrSet<SCC *, 4>> CallerSCCs;
  for (const auto &KV : enumerate(SCCs)) {
    auto &CallerSCC = KV.value();
    for (CallGraphNode *CGN : CallerSCC.Nodes) {
      for (const auto &KV : *CGN) {
        SCC &CalleeSCC = SCCs[SCCID[KV.second]];
        if (&CalleeSCC != &CallerSCC)
          CallerSCCs[&CalleeSCC].insert(&CallerSCC);
      }
    }
  }

  // Collect the set of SCCs that have no callers. There must always be at least
  // one, since the SCC graph is acyclic.
  std::vector<SCC *> RootSCCs;
  for (SCC &SCC : SCCs)
    if (CallerSCCs.find(&SCC) == CallerSCCs.end())
      RootSCCs.push_back(&SCC);

  // Handle each root discovered in turn.
  uint64_t StackSize = 0;
  std::vector<SCC *> InterruptNorecurseSCCs;
  bool ToInterruptNorecurse = false;
  while (!RootSCCs.empty() || !InterruptNorecurseSCCs.empty()) {
    // If only INR's are left, then handle one. This will cause all its
    // descendants to be scheduled. Start its offset at the end of the current
    // stack, since it and its descendants might interrupt anything seen so far.
    if (RootSCCs.empty()) {
      ToInterruptNorecurse = true;
      RootSCCs.push_back(InterruptNorecurseSCCs.back());
      RootSCCs.back()->Offset = StackSize;
      InterruptNorecurseSCCs.pop_back();
      continue;
    }

    // For each reached root, it's certain that all calling SCCs have already
    // been assigned offsets. Accordingly, the current offset for this SCC can
    // be taken as its final position in the static stack.
    SCC &RootSCC = *RootSCCs.back();
    RootSCCs.pop_back();

    // Defer interrupt-norecurse SCCs and their descendants until later, since
    // they conflict with all other nodes.
    if (!ToInterruptNorecurse && RootSCC.Nodes.size() == 1 &&
        RootSCC.Nodes.front()->getFunction() &&
        RootSCC.Nodes.front()->getFunction()->hasFnAttribute(
            "interrupt-norecurse")) {
      InterruptNorecurseSCCs.push_back(&RootSCC);
      continue;
    }

    LLVM_DEBUG({
      dbgs() << "\nSCC:\n";
      for (CallGraphNode *CGN : RootSCC.Nodes)
        CGN->dump();
      dbgs() << "Offset: " << RootSCC.Offset << "\n";
    });

    // Any callee SCCs need to be placed below the end of this SCC's static
    // stack region. Note that this is true even if the SCC is internally
    // recursive; the SCCs above and below may not be.
    const auto &GetStaticStackSize = [&]() -> uint64_t {
      size_t Size = 0;
      for (CallGraphNode *Node : RootSCC.Nodes) {
        Function *F = Node->getFunction();
        if (!F)
          continue;
        MachineFunction *MF = MMI.getMachineFunction(*F);
        if (!MF)
          continue;
        const MOSFrameLowering &TFL =
            *MF->getSubtarget<MOSSubtarget>().getFrameLowering();
        Size += TFL.staticSize(MF->getFrameInfo());
      }
      return Size;
    };
    uint64_t Size = GetStaticStackSize();
    LLVM_DEBUG(dbgs() << "Size: " << Size << "\n");

    // Determine the new offset to propagate to callee SCCs, and note if this
    // increased the overall stack size.
    uint64_t Offset = RootSCC.Offset + Size;
    StackSize = std::max(StackSize, Offset);

    // For each callee SCC, propagate the offset and ensure that each occupies a
    // position in the static stack below the end of the current SCC.
    for (CallGraphNode *CGN : RootSCC.Nodes) {
      for (const auto &KV : *CGN) {
        SCC &CalleeSCC = SCCs[SCCID[KV.second]];
        if (&CalleeSCC != &RootSCC &&
            CallerSCCs[&CalleeSCC].contains(&RootSCC)) {
          CalleeSCC.Offset = std::max(CalleeSCC.Offset, Offset);
          CallerSCCs[&CalleeSCC].erase(&RootSCC);
          // If there are no longer any caller SCCs for a SCC, then that SCC is
          // a newly-discovered root, so schedule it for placement. Since the
          // SCC graph is acyclic, every SCC must eventually become a root via
          // this process.
          if (CallerSCCs[&CalleeSCC].empty())
            RootSCCs.push_back(&CalleeSCC);
        }
      }
    }
  }

  if (!StackSize)
    return false;

  // Create a global variable for the static stack as a whole.
  Type *Typ = ArrayType::get(Type::getInt8Ty(M.getContext()), StackSize);
  GlobalVariable *Stack =
      new GlobalVariable(M, Typ, false, GlobalValue::PrivateLinkage,
                         UndefValue::get(Typ), "static_stack");
  LLVM_DEBUG(dbgs() << *Stack << "\n");

  // Create an alias for each SCC's static stack region and rewrite instructions
  // to reference it.
  for (const SCC &SCC : SCCs) {
    size_t Offset = SCC.Offset;
    for (CallGraphNode *Node : SCC.Nodes) {
      Function *F = Node->getFunction();
      if (!F)
        continue;
      MachineFunction *MF = MMI.getMachineFunction(*F);
      if (!MF)
        continue;
      const MOSFrameLowering &TFL =
          *MF->getSubtarget<MOSSubtarget>().getFrameLowering();
      uint64_t Size = TFL.staticSize(MF->getFrameInfo());
      if (!Size)
        continue;

      Type *Typ = ArrayType::get(Type::getInt8Ty(M.getContext()), Size);
      Constant *Aliasee = Stack;
      if (Offset) {
        Type *I16 = Type::getInt16Ty(Stack->getContext());
        Aliasee = ConstantExpr::getGetElementPtr(
            Stack->getValueType(), Stack,
            SmallVector<Constant *>{ConstantInt::get(I16, 0),
                                    ConstantInt::get(I16, Offset)},
            /*InBounds=*/true);
      }
      Offset += Size;
      auto *Alias = GlobalAlias::create(
          Typ, Stack->getAddressSpace(), Stack->getLinkage(),
          Twine(F->getName()) + "_sstk", Aliasee, Stack->getParent());
      LLVM_DEBUG(dbgs() << *Alias << "\n");

      MOSFunctionInfo &MFI = *MF->getInfo<MOSFunctionInfo>();
      MFI.StaticStackValue = Alias;

      for (MachineBasicBlock &MBB : *MF) {
        for (MachineInstr &MI : MBB) {
          for (MachineOperand &MO : MI.operands()) {
            if (!MO.isTargetIndex())
              continue;
            MO.ChangeToGA(Alias, MO.getOffset(), MO.getTargetFlags());
          }
        }
      }
    }
  }
  return true;
}

} // namespace

char MOSStaticStackAlloc::ID = 0;

INITIALIZE_PASS(MOSStaticStackAlloc, DEBUG_TYPE,
                "Allocate non-recursive stack to static memory", false, false)

ModulePass *llvm::createMOSStaticStackAllocPass() {
  return new MOSStaticStackAlloc();
}
