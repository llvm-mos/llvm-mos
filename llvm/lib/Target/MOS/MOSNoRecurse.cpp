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
#include "llvm/Analysis/CallGraph.h"
#include "llvm/IR/Module.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/ErrorHandling.h"

#define DEBUG_TYPE "mos-norecurse"

using namespace llvm;

namespace {

struct MOSNoRecurse : public CallGraphSCCPass {
  static char ID; // Pass identification, replacement for typeid

  MOSNoRecurse() : CallGraphSCCPass(ID) {
    initializeMOSNoRecursePass(*PassRegistry::getPassRegistry());
  }

  bool runOnSCC(CallGraphSCC &SCC) override;

  bool doInitialization(CallGraph &CG) override {
    LLVM_DEBUG(dbgs() << "**** MOS NoRecurse Pass ****\n");
    // For the conservative recursion analysis, any external call may call any
    // externally-callable function so add an edge from the calls-external node
    // to the called-by-external node.
    assert(CG.getCallsExternalNode()->empty());
    CG.getCallsExternalNode()->addCalledFunction(nullptr,
                                                 CG.getExternalCallingNode());
    // Report unchanged, since the call graph will be returned to its original
    // condition on finalization.
    return false;
  }

  bool doFinalization(CallGraph &CG) override {
    // Remove the artificial edge added in initialization.
    CG.getCallsExternalNode()->removeAllCalledFunctions();
    return false;
  }
};

static bool callsSelf(const CallGraphNode& N) {
  for (const CallGraphNode::CallRecord &CR : N)
    if (CR.second == &N)
      return true;
  return false;
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

} // namespace

char MOSNoRecurse::ID = 0;

INITIALIZE_PASS(
    MOSNoRecurse, DEBUG_TYPE,
    "Detect non-recursive functions via detailed call graph analysis", false,
    false)

CallGraphSCCPass *llvm::createMOSNoRecursePass() {
  return new MOSNoRecurse();
}
