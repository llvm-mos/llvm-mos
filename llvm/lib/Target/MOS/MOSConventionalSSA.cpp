//===----------------------------------------------------------------------===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
///
/// \file
/// Normalize the SSA into "conventional" form before allocation.
///
/// To be executed, the PHI instructions in SSA form need to be eliminated by
/// converting them to real parallel copies which are then sequentialized.
/// Logically parallel copies would be inserted along the edges between basic
/// blocks, but it isn't performantly possible to do this in the presence of
/// indirect branches (e.g., C's computed goto).
///
/// The parallel copies need to go somewhere, so a natural choice is the
/// predecessor block. However, since this is only an approximation to edge
/// placement, this can create unsatisfiable conflicts between the variables
/// used and defined by the copies.
///
/// To avoid this, this pass inserts further parallel copies to ensure that
/// every value used or defined by the PHI is only used by the new copy and the
/// PHI. This is called "conventional SSA form" in the literature, and it
/// handily resolves the above issue by shifting its complexity onto coalescing
/// away as many of the new copies as possible. (This is done implicitly during
/// assignment.)
///
/// This pass further normalizes tied uses and defs to allow us to treat a tied
/// def as a mere continuation of its use's live range. To do so, this pass
/// inserts copies and rewrites to ensure that each tied use is a kill and that
/// it satisfies the constraints of its tied def.
///
/// The above has only been prototyped, but not yet upstreamed. This pass is
/// currently just a stub.
///
//===----------------------------------------------------------------------===//

#include "MOSConventionalSSA.h"
#include "MOS.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/InitializePasses.h"
#include "llvm/Support/ErrorHandling.h"

#define DEBUG_TYPE "mos-conventional-ssa"

using namespace llvm;

namespace {

class MOSConventionalSSA : public MachineFunctionPass {
public:
  static char ID;

  MOSConventionalSSA() : MachineFunctionPass(ID) {
    initializeMOSConventionalSSAPass(*PassRegistry::getPassRegistry());
  }

  bool runOnMachineFunction(MachineFunction &) override {
    report_fatal_error("MOSConventionalSSA is not implemented", false);
  }

  MachineFunctionProperties getRequiredProperties() const override {
    return MachineFunctionProperties().setIsSSA();
  }
};

} // namespace

char MOSConventionalSSA::ID = 0;
INITIALIZE_PASS(MOSConventionalSSA, DEBUG_TYPE, "MOS Conventional SSA", false,
                false)

MachineFunctionPass *llvm::createMOSConventionalSSAPass() {
  return new MOSConventionalSSA;
}
