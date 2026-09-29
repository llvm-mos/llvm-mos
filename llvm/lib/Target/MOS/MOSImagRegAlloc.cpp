//===----------------------------------------------------------------------===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
///
/// \file
/// Allocate imaginary registers and spill values that cannot remain in them.
///
/// This pass colors a function-wide interference graph over the conventional
/// SSA produced by MOSConventionalSSA. Assignments respect imaginary register
/// classes, aliases, and physical imaginary register live ranges. Tied operands
/// and the isolated inputs and result of each PHI receive the same storage.
/// When registers are insufficient, the pass inserts spills and reloads and
/// updates the graph before completing assignment.
///
/// The output remains in SSA form, with imaginary register assignments recorded
/// in VirtRegMap and spills represented in the MIR. PHIs and parallel copies
/// remain for MOSRegAlloc to lower. MOSRegAlloc may keep values in hardware
/// registers and eliminate unnecessary imaginary register transfers, but must
/// satisfy explicit imaginary register constraints.
///
/// This pass is currently a stub; allocation and spilling are not implemented.
///
//===----------------------------------------------------------------------===//

#include "MOSImagRegAlloc.h"
#include "MOS.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/InitializePasses.h"
#include "llvm/Support/ErrorHandling.h"

#define DEBUG_TYPE "mos-imag-regalloc"

using namespace llvm;

namespace {

class MOSImagRegAlloc : public MachineFunctionPass {
public:
  static char ID;

  MOSImagRegAlloc() : MachineFunctionPass(ID) {
    initializeMOSImagRegAllocPass(*PassRegistry::getPassRegistry());
  }

  bool runOnMachineFunction(MachineFunction &) override {
    report_fatal_error("MOSImagRegAlloc is not implemented", false);
  }

  MachineFunctionProperties getRequiredProperties() const override {
    return MachineFunctionProperties().setIsSSA();
  }
};

} // namespace

char MOSImagRegAlloc::ID = 0;
INITIALIZE_PASS(MOSImagRegAlloc, DEBUG_TYPE,
                "MOS Imaginary Register Allocation", false, false)

MachineFunctionPass *llvm::createMOSImagRegAllocPass() {
  return new MOSImagRegAlloc;
}
