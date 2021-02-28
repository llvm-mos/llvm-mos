//===-- MOSCallLowering.h - Call lowering -----------------------*- C++ -*-===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file describes how to lower LLVM calls to machine code calls.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MOS_MOSCALLLOWERING_H
#define LLVM_LIB_TARGET_MOS_MOSCALLLOWERING_H

#include "llvm/CodeGen/FunctionLoweringInfo.h"
#include "llvm/CodeGen/GlobalISel/CallLowering.h"

namespace llvm {

class MOSCallLowering : public CallLowering {
public:
  MOSCallLowering(const llvm::TargetLowering *TL) : CallLowering(TL) {}

  bool lowerReturn(MachineIRBuilder &MIRBuiler, const Value *Val,
                   ArrayRef<Register> VRegs,
                   FunctionLoweringInfo &FLI) const override;

  bool lowerFormalArguments(MachineIRBuilder &MIRBuilder, const Function &F,
                            ArrayRef<ArrayRef<Register>> VRegs,
                            FunctionLoweringInfo &FLI) const override;

  bool lowerCall(MachineIRBuilder &MIRBuilder,
                 CallLoweringInfo &Info) const override;

private:
  void splitToValueTypes(const ArgInfo &OrigArg,
                         SmallVectorImpl<ArgInfo> &SplitArgs,
                         const DataLayout &DL) const;
};

} // namespace llvm

#endif // not LLVM_LIB_TARGET_MOS_MOSCALLLOWERING_H
