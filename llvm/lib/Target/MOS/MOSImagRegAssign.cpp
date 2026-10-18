//===----------------------------------------------------------------------===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
///
/// \file
/// Assign imaginary registers after local and PHI spilling.
///
/// MOSImagRegSpill leaves an SSA program whose imaginary register demands can
/// be satisfied. This pass chooses assignments respecting
/// interference, ties, PHI congruence, and physical constraints, and records
/// them in VirtRegMap. It does not insert further spills. PHIs and parallel
/// copies remain for MOSRegAlloc to lower along with hardware allocation.
///
/// This pass is not implemented yet.
///
//===----------------------------------------------------------------------===//

#include "MOSImagRegAssign.h"
#include "MOS.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/InitializePasses.h"
#include "llvm/Support/ErrorHandling.h"

#define DEBUG_TYPE "mos-imag-reg-assign"

using namespace llvm;

namespace {

class MOSImagRegAssign : public MachineFunctionPass {
public:
  static char ID;

  MOSImagRegAssign() : MachineFunctionPass(ID) {
    initializeMOSImagRegAssignPass(*PassRegistry::getPassRegistry());
  }

  bool runOnMachineFunction(MachineFunction &) override {
    report_fatal_error("MOSImagRegAssign is not implemented", false);
  }

  MachineFunctionProperties getRequiredProperties() const override {
    return MachineFunctionProperties().setIsSSA();
  }
};

} // namespace

char MOSImagRegAssign::ID = 0;
INITIALIZE_PASS(MOSImagRegAssign, DEBUG_TYPE,
                "MOS Imaginary Register Assignment", false, false)

MachineFunctionPass *llvm::createMOSImagRegAssignPass() {
  return new MOSImagRegAssign;
}
