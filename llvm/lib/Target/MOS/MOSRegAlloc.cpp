//===----------------------------------------------------------------------===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
///
/// \file
/// Allocate and assign final physical registers and exit SSA form.
///
/// MOSImagRegAlloc supplies an SSA program with imaginary register assignments
/// in VirtRegMap and explicit spills. This pass chooses hardware placements and
/// emits the transfers needed to satisfy uses, while respecting those imaginary
/// assignments wherever imaginary storage is required. The resulting MIR has
/// physical operands and no PHIs or parallel-copy pseudos.
///
/// This pass performs global optimization of the use of hardware registers and
/// assigned imaginary registers using the treewidth dynamic programming
/// approach used in SDCC. Search is performed for a high-quality assignment,
/// accounting for the costs of various instructions against different
/// placements of their operands and the transfer costs between possible
/// assignments between neighboring instructions.
///
/// PHIs are trivially eliminated, while parallel copies are lowered to a
/// sequence of real instructions. (Such sequences are a regular part of the
/// search.)
///
/// The cost of search is bounded, and a sub-optimal allocation may be selected
/// in pathological cases.
///
/// The above has only been prototyped, but not yet upstreamed. This pass is
/// currently just a stub.
///
//===----------------------------------------------------------------------===//

#include "MOSRegAlloc.h"
#include "MOS.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/InitializePasses.h"
#include "llvm/Support/ErrorHandling.h"

#define DEBUG_TYPE "mos-regalloc"

using namespace llvm;

namespace {

class MOSRegAlloc : public MachineFunctionPass {
public:
  static char ID;

  MOSRegAlloc() : MachineFunctionPass(ID) {
    initializeMOSRegAllocPass(*PassRegistry::getPassRegistry());
  }

  bool runOnMachineFunction(MachineFunction &) override {
    report_fatal_error("MOSRegAlloc is not implemented", false);
  }

  MachineFunctionProperties getRequiredProperties() const override {
    return MachineFunctionProperties().setIsSSA();
  }
};

} // namespace

char MOSRegAlloc::ID = 0;
INITIALIZE_PASS(MOSRegAlloc, DEBUG_TYPE, "MOS Register Allocation", false,
                false)

MachineFunctionPass *llvm::createMOSRegAllocPass() { return new MOSRegAlloc; }
