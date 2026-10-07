//===-- MOSCallTreeLibcalls.cpp - MOS Call Tree Libcalls ------------------===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file defines the MOS call tree libcall redirect pass.
//
// The GlobalISel legalizer can emit calls to runtime libcalls (e.g. __mulhi3,
// __divqi3, memcpy) as MachineInstr operands of kind MO_ExternalSymbol. These
// calls do not appear in the IR, so MOSCallTreeClone cannot rewrite them at IR
// time. For each function carrying the "rc-suffix" attribute, this pass
// rewrites every external-symbol call operand that names a function for which
// a per-suffix clone exists in the module so it refers to that clone instead.
//
// The pass runs between the Legalizer and MOSInternalize. MOSInternalize then
// finds the suffixed clones by name (via M.getFunction(symbol-name)) and
// keeps/deletes them based on the references this pass leaves behind. Calls
// with no clone are left alone and diagnosed by MOSCallTreeVerify if reachable.
//
//===----------------------------------------------------------------------===//

#include "MOSCallTreeLibcalls.h"

#include "MOS.h"

#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/MachineOperand.h"
#include "llvm/IR/Function.h"
#include "llvm/IR/Module.h"

#define DEBUG_TYPE "mos-call-tree-libcalls"

using namespace llvm;

namespace {

class MOSCallTreeLibcalls : public MachineFunctionPass {
public:
  static char ID;

  MOSCallTreeLibcalls() : MachineFunctionPass(ID) {
    initializeMOSCallTreeLibcallsPass(*PassRegistry::getPassRegistry());
  }

  bool runOnMachineFunction(MachineFunction &MF) override;
};

} // namespace

char MOSCallTreeLibcalls::ID = 0;

INITIALIZE_PASS(MOSCallTreeLibcalls, DEBUG_TYPE,
                "Redirect libcalls in suffixed MOS ISR functions to clones",
                /* CFGOnly = */ false, /* Analysis = */ false)

MachineFunctionPass *llvm::createMOSCallTreeLibcallsPass() {
  return new MOSCallTreeLibcalls();
}

bool MOSCallTreeLibcalls::runOnMachineFunction(MachineFunction &MF) {
  const Function &F = MF.getFunction();
  Attribute SfxAttr = F.getFnAttribute("rc-suffix");
  if (!SfxAttr.isValid())
    return false;
  StringRef Sfx = SfxAttr.getValueAsString();

  const Module &M = *F.getParent();
  bool Changed = false;

  for (MachineBasicBlock &MBB : MF) {
    for (MachineInstr &MI : MBB) {
      if (!MI.isCall())
        continue;
      for (MachineOperand &MO : MI.operands()) {
        if (!MO.isSymbol())
          continue;
        // If a suffixed clone of this libcall is present in the module,
        // redirect the call to it. Otherwise leave it alone: this function may
        // be a speculative clone that never survives MOSInternalize, and
        // MOSCallTreeVerify reports the call if it is actually reachable from
        // an interrupt root.
        std::string SuffixedName = (MO.getSymbolName() + Sfx).str();
        if (M.getFunction(SuffixedName)) {
          MO.ChangeToES(MF.createExternalSymbolName(SuffixedName),
                        MO.getTargetFlags());
          Changed = true;
        }
      }
    }
  }
  return Changed;
}
