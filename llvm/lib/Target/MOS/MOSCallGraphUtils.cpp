//===-- MOSCalGraphUtils.cpp - MOS Call Graph Utilities -------------------===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
///
/// \file
/// This file defines utilities for temporarily modifying the semantics of the
/// CallGraph to enforce invariants needed for whole-program static stack and
/// zero page analyses.
///
//===----------------------------------------------------------------------===//

#include "MOSCallGraphUtils.h"

#include "llvm/Analysis/CallGraph.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineModuleInfo.h"
#include "llvm/IR/Module.h"

using namespace llvm;

Function *mos::getSymbolFunction(Module &M, StringRef Name) {
  Value *V = M.getNamedValue(Name);
  return V ? dyn_cast<Function>(V->stripPointerCastsAndAliases()) : nullptr;
}

void mos::addLibcallEdges(CallGraph &CG, const MachineModuleInfo &MMI) {
  for (auto &KV : CG) {
    CallGraphNode &CGN = *KV.second;
    if (!CGN.getFunction())
      continue;
    MachineFunction *MF = MMI.getMachineFunction(*CGN.getFunction());
    if (!MF)
      continue;
    for (const MachineBasicBlock &MBB : *MF) {
      for (const MachineInstr &MI : MBB) {
        if (!MI.isCall())
          continue;
        for (const MachineOperand &MO : MI.operands()) {
          if (!MO.isSymbol())
            continue;
          CG.getModule().getNamedGlobal(MO.getSymbolName());
          Function *Callee =
              getSymbolFunction(CG.getModule(), MO.getSymbolName());
          if (Callee && MMI.getMachineFunction(*Callee))
            CGN.addCalledFunction(nullptr, CG[Callee]);
        }
      }
    }
  }
}

void mos::addExternalEdges(CallGraph &CG) {
  assert(CG.getCallsExternalNode()->empty());
  for (auto &KV : *CG.getExternalCallingNode()) {
    Function *F = KV.second->getFunction();
    if (F && !F->hasFnAttribute("interrupt") &&
        !F->hasFnAttribute("interrupt-norecurse"))
      CG.getCallsExternalNode()->addCalledFunction(nullptr, KV.second);
  }
}
