//===-- MOSInlineAsmLowering.cpp ------------------------------------------===//
//
// Part of the LLVM-MOS Project, under the Apache License v2.0 with LLVM
// Exceptions. See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
///
/// \file
/// This file implements the lowering from LLVM IR inline asm to MIR INLINEASM
///
//===----------------------------------------------------------------------===//

#include "MOSInlineAsmLowering.h"
#include "llvm/CodeGen/GlobalISel/InlineAsmLowering.h"

using namespace llvm;

MOSInlineAsmLowering::MOSInlineAsmLowering(MOSTargetLowering *TLI)
    : InlineAsmLowering(TLI) {}

// Integer immediates may be used as addresses, but negative numbers are not
// legal addresses. Since positive numbers are always legal, treat all integer
// immediates as unsigned.
static bool handleIntImmediate(Value *Val, StringRef Constraint,
                               std::vector<MachineOperand> &Ops) {
  if (Constraint.size() > 1)
    return false;

  char ConstraintLetter = Constraint[0];
  switch (ConstraintLetter) {
  default:
    return false;
  // Simple Integer or Relocatable Constant
  case 'i':
  // immediate integer with a known value.
  case 'n': {
    ConstantInt *CI = dyn_cast<ConstantInt>(Val);
    if (!CI)
      return false;
    assert(CI->getBitWidth() <= 64 && "expected immediate to fit into 64-bits");
    Ops.push_back(MachineOperand::CreateImm(CI->getZExtValue()));
    return true;
  }
  }
}

bool MOSInlineAsmLowering::lowerAsmOperandForConstraint(
    Value *Val, StringRef Constraint, std::vector<MachineOperand> &Ops,
    MachineIRBuilder &MIRBuilder) const {
  if (handleIntImmediate(Val, Constraint, Ops))
    return true;
  return InlineAsmLowering::lowerAsmOperandForConstraint(Val, Constraint, Ops,
                                                         MIRBuilder);
}
