//===-- MOSMachineFuctionInfo.h - MOS machine function info -----*- C++ -*-===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file declares MOS-specific per-machine-function information.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MOS_MOSMACHINEFUNCTIONINFO_H
#define LLVM_LIB_TARGET_MOS_MOSMACHINEFUNCTIONINFO_H

#include "llvm/CodeGen/MachineFunction.h"

namespace llvm {

class MOSFunctionInfo : public MachineFunctionInfo {
  int VarArgsStackIndex = -1;
  const GlobalVariable *StaticStackVariable = nullptr;

public:
  MOSFunctionInfo(MachineFunction& MF) {}

  /// Returns the fake frame index indicating the start of the varargs region of
  /// the incoming call stack.
  int getVarArgsStackIndex() const { return VarArgsStackIndex; }

  /// Sets the fake frame index indicating the start of the varargs region of the
  /// incoming call stack.
  void setVarArgsStackIndex(int Index) { VarArgsStackIndex = Index; }

  /// Returns the static stack variable allocated for this function.
  const GlobalVariable *getStaticStackVariable() const {
    return StaticStackVariable;
  }

  /// Sets the static stack variable allocated for this function.
  void setStaticStackVariable(const GlobalVariable *Var) {
    StaticStackVariable = Var;
  }
};

} // namespace llvm

#endif // not LLVM_LIB_TARGET_MOS_MOSMACHINEFUNCTIONINFO_H