//===-- MOSMCInstrAnalysis.cpp - MOS instruction analysis -----------------===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains the declarations of the MOSMCAsmInfo properties.
//
//===----------------------------------------------------------------------===//

#include "MOSMCInstrAnalysis.h"
#include "MOSMCTargetDesc.h"

namespace llvm {

bool MOSMCInstrAnalysis::evaluateBranch(const MCInst &Inst,
                                        uint64_t Addr,
                                        uint64_t Size,
                                        uint64_t &Target) const {
  unsigned NumOps = Inst.getNumOperands();
  if (NumOps == 0 ||
    Info->get(Inst.getOpcode()).operands()[NumOps - 1].OperandType !=
      MCOI::OPERAND_PCREL)
    return false;
  Target = Addr + Size + Inst.getOperand(NumOps - 1).getImm();
  return true;
}

} //  namespace llvm
