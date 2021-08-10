//===-- MOSNoRecurse.cpp - MOS NoRecurse Pass -----------------------------===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file defines the MOS NoRecurse pass.
//
// This pass examines the full inter-procedural Module function call graph to
// identify functions that cannot possibly be recursive. Those functiosn are
// marked with the norecurse annotation, which allows the code generator to lay
// their local stack frames out in globally static memory. This is possible
// because such functions can have at most one invocation active at any given
// time.
//
// This pass is considerably more aggressive than LLVM's built-in NoRecurse
// passes, as it examines the call graph SCCs themselves, not individual
// functions in SCC order.
//===----------------------------------------------------------------------===//

#include "MOSNoRecurse.h"

#include "MOS.h"
#include "llvm/ADT/SCCIterator.h"
#include "llvm/Analysis/CallGraph.h"
#include "llvm/Analysis/CallGraphSCCPass.h"
#include "llvm/IR/Module.h"
#include "llvm/LTO/LTO.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/ErrorHandling.h"

#define DEBUG_TYPE "mos-norecurse"

using namespace llvm;

namespace {

struct MOSNoRecurse : public ModulePass {
  static char ID; // Pass identification, replacement for typeid
  SmallPtrSet<const CallGraphNode *, 8> ReachableFromMultipleInterrupts;
  SmallPtrSet<const CallGraphNode *, 8> ReachableFromCurrentNorecurseInterrupt;
  SmallPtrSet<const CallGraphNode *, 8> ReachableFromOtherNorecurseInterrupt;
  bool HasInterrupts = false;

  MOSNoRecurse() : ModulePass(ID) {
    initializeMOSNoRecursePass(*PassRegistry::getPassRegistry());
  }

  bool runOnModule(Module &M) override;
  void getAnalysisUsage(AnalysisUsage &Info) const override;

  bool runOnSCC(CallGraphSCC &SCC);
  void markReachableFromMultipleInterrupts(const CallGraphNode &CGN);
  void visitNorecurseInterrupt(const CallGraphNode &CGN);
};

static bool callsSelf(const CallGraphNode &N) {
  for (const CallGraphNode::CallRecord &CR : N)
    if (CR.second == &N)
      return true;
  return false;
}

bool MOSNoRecurse::runOnModule(Module &M) {
  LLVM_DEBUG(dbgs() << "**** MOS NoRecurse Pass ****\n");

  CallGraph &CG = getAnalysis<CallGraphWrapperPass>().getCallGraph();

  // For the conservative recursion analysis, any external call may call any
  // externally-callable function so add an edge from the calls-external node
  // to the called-by-external node.
  assert(CG.getCallsExternalNode()->empty());
  CG.getCallsExternalNode()->addCalledFunction(nullptr,
                                               CG.getExternalCallingNode());

  // Walk the callgraph in bottom-up SCC order.
  scc_iterator<CallGraph *> CGI = scc_begin(&CG);
  CallGraphSCC CurSCC(CG, &CGI);
  bool Changed = false;
  for (; !CGI.isAtEnd(); ++CGI) {
    CurSCC.initialize(*CGI);
    Changed |= runOnSCC(CurSCC);
  }

  // Mark all functions reachable from an interrupt function as possibly
  // recursive.
  for (Function &F : M.functions()) {
    if (F.hasFnAttribute("interrupt")) {
      HasInterrupts = true;
      markReachableFromMultipleInterrupts(*CG[&F]);
    }
  }

  // Mark all functions reachable from multiple interrupt-norecurse functions as
  // possibly recursive.
  for (Function &F : M.functions()) {
    if (F.hasFnAttribute("interrupt-norecurse") || F.getName() == "main") {
      if (F.hasFnAttribute("interrupt-norecurse"))
        HasInterrupts = true;
      visitNorecurseInterrupt(*CG[&F]);
      for (const auto *CGN : ReachableFromCurrentNorecurseInterrupt)
        ReachableFromOtherNorecurseInterrupt.insert(CGN);
      ReachableFromCurrentNorecurseInterrupt.clear();
    }
  }

  if (HasInterrupts) {
    Changed = true;

    // Mark all libcalls as possibly recursive if we have interrupts, since
    // there's no way to tell which will actually be called by an interrupt
    // before the interrupt is compiled. But the compilation of the interrupt
    // depends on whether or not it's norecurse, so we don't have much choice
    // other than making the conservative assumption here.
    for (const char *LibcallName : lto::LTO::getRuntimeLibcallSymbols()) {
      Function *Libcall = M.getFunction(LibcallName);
      if (Libcall && !Libcall->isDeclaration() && Libcall->doesNotRecurse()) {
        LLVM_DEBUG(dbgs() << "Marking libcall as possibly recursive: "
                          << Libcall->getName() << "\n");
        Libcall->removeFnAttr(Attribute::NoRecurse);
      }
    }
  }

  // Remove the artificial edge.
  CG.getCallsExternalNode()->removeAllCalledFunctions();
  return Changed;
}

void MOSNoRecurse::getAnalysisUsage(AnalysisUsage &AU) const {
  AU.addRequired<CallGraphWrapperPass>();
  AU.addPreserved<CallGraphWrapperPass>();
}

bool MOSNoRecurse::runOnSCC(CallGraphSCC &SCC) {
  // All nodes in SCCs with more than one node may be recursive. It's not
  // certain since CFG analysis is conservative, but there's no more
  // information to be gleaned from looking at the call graph, and other
  // sources of information are better used making the CFG analysis less
  // conservative.
  if (!SCC.isSingular())
    return false;

  const CallGraphNode &N = **SCC.begin();

  if (!N.getFunction() || N.getFunction()->isDeclaration() ||
      N.getFunction()->doesNotRecurse())
    return false;

  // Since the CFG analysis is conservative, any possible indirect recursion
  // involving N would have placed in an SCC with more than one node. Thus, N
  // is recursive iff it directly calls itself.
  if (callsSelf(N))
    return false;

  LLVM_DEBUG(dbgs() << "Found new non-recursive function.\n");
  LLVM_DEBUG(N.print(dbgs()));

  // At this point, the function in N is known non-recursive.
  N.getFunction()->setDoesNotRecurse();
  return true;
}

void MOSNoRecurse::markReachableFromMultipleInterrupts(
    const CallGraphNode &CGN) {
  if (ReachableFromMultipleInterrupts.contains(&CGN))
    return;
  ReachableFromMultipleInterrupts.insert(&CGN);

  Function *F = CGN.getFunction();
  if (F && !F->isDeclaration()) {
    LLVM_DEBUG(dbgs() << "Marking reachable from interrupt: " << F->getName()
                      << "\n");
    if (F->doesNotRecurse())
      F->removeFnAttr(Attribute::NoRecurse);
  }

  for (const auto &CallRecord : CGN)
    markReachableFromMultipleInterrupts(*CallRecord.second);
}

void MOSNoRecurse::visitNorecurseInterrupt(const CallGraphNode &CGN) {
  if (ReachableFromMultipleInterrupts.contains(&CGN))
    return;
  if (ReachableFromCurrentNorecurseInterrupt.contains(&CGN))
    return;
  ReachableFromCurrentNorecurseInterrupt.insert(&CGN);

  Function *F = CGN.getFunction();
  if (F && !F->isDeclaration() &&
      ReachableFromOtherNorecurseInterrupt.contains(&CGN)) {
    LLVM_DEBUG(
        dbgs() << "Marking reachable from multiple norecurse interrupts: "
               << F->getName() << "\n");
    ReachableFromMultipleInterrupts.insert(&CGN);
    if (F->doesNotRecurse())
      F->removeFnAttr(Attribute::NoRecurse);
  }
  for (const auto &CallRecord : CGN)
    visitNorecurseInterrupt(*CallRecord.second);
}
} // namespace

char MOSNoRecurse::ID = 0;

INITIALIZE_PASS(
    MOSNoRecurse, DEBUG_TYPE,
    "Detect non-recursive functions via detailed call graph analysis", false,
    false)

ModulePass *llvm::createMOSNoRecursePass() { return new MOSNoRecurse(); }
