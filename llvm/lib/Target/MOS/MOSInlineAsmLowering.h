//===- MOSInlineAsmLowering.h -----------------------------------*- C++ -*-===//
//
// Part of the LLVM-MOS Project, under the Apache License v2.0 with LLVM
// Exceptions. See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
///
/// \file
/// This file describes how to lower LLVM inline asm to machine code INLINEASM.
///
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MOS_MOSINLINEASMLOWERING_H
#define LLVM_LIB_TARGET_MOS_MOSINLINEASMLOWERING_H

#include "MOSISelLowering.h"
#include "llvm/CodeGen/GlobalISel/InlineAsmLowering.h"

namespace llvm {

class MOSTargetLowering;

class MOSInlineAsmLowering : public InlineAsmLowering {
public:
  MOSInlineAsmLowering(MOSTargetLowering *TLI);

  bool
  lowerAsmOperandForConstraint(Value *Val, StringRef Constraint,
                               std::vector<MachineOperand> &Ops,
                               MachineIRBuilder &MIRBuilder) const override;
};

} // namespace llvm

#endif // LLVM_LIB_TARGET_MOS_MOSINLINEASMLOWERING_H
