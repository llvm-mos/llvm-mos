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

class MOSSubtarget;

struct MOSFunctionInfo : public MachineFunctionInfo {
  MOSFunctionInfo(const Function &F, const MOSSubtarget *STI) {}

  int VarArgsStackIndex = -1;
  const GlobalValue *StaticStackValue = nullptr;
  const GlobalValue *ZeroPageStackValue = nullptr;
  DenseMap<Register, size_t> CSRZPOffsets;
};

} // namespace llvm

#endif // not LLVM_LIB_TARGET_MOS_MOSMACHINEFUNCTIONINFO_H
