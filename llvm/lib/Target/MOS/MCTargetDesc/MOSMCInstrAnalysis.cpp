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
#include "MOSSubtarget.h"
#include "llvm/MC/MCInstrDesc.h"
#include "llvm/MC/MCSubtargetInfo.h"

namespace llvm {

bool MOSMCInstrAnalysis::evaluateBranch(const MCInst &Inst,
                                        uint64_t Addr,
                                        uint64_t Size,
                                        uint64_t &Target) const {
  if ((!isBranch(Inst) && !isCall(Inst)) || isIndirectBranch(Inst))
    return false;
  unsigned NumOps = Inst.getNumOperands();
  if (NumOps == 0)
    return false;
  const auto &Op = Info->get(Inst.getOpcode()).operands()[NumOps - 1];
  switch (Op.OperandType) {
    case MOSOp::OPERAND_ADDR16: {
      Target = (Addr & 0xFFFF0000)
               | (Inst.getOperand(NumOps - 1).getImm() & 0xFFFF);
      return true;
    }
    case MOSOp::OPERAND_ADDR24: {
      Target = (Addr & 0xFF000000)
               | (Inst.getOperand(NumOps - 1).getImm() & 0xFFFFFF);
      return true;
    }
    case MCOI::OPERAND_PCREL: {
      Target = Addr + Size + Inst.getOperand(NumOps - 1).getImm();
      return true;
    }
  }
  return false;
}

std::optional<uint64_t>
MOSMCInstrAnalysis::evaluateMemoryOperandAddress(const MCInst &Inst,
                                                 const MCSubtargetInfo *STI,
                                                 uint64_t Addr,
                                                 uint64_t Size) const {
  uint64_t ZpAddrOffset = static_cast<const MOSSubtarget *>(STI)
                              ->getZeroPageOffset();
  uint64_t AbsAddrMask = STI->hasFeature(MOS::FeatureW65816)
                             ? 0xFFFFFF : 0xFFFF;

  unsigned NumOps = Inst.getNumOperands();
  // Assumption: Every opcode has only one memory operand.
  for (unsigned OpIdx = 0; OpIdx < NumOps; OpIdx++) {
    const auto &Op = Info->get(Inst.getOpcode()).operands()[OpIdx];
    switch (Op.OperandType) {
      case MOSOp::OPERAND_ADDR8: {
        return (Addr & ~AbsAddrMask) | ZpAddrOffset
               | (Inst.getOperand(OpIdx).getImm() & 0xFF);
      }
      case MOSOp::OPERAND_ADDR13: {
        return (Addr & ~AbsAddrMask)
               | (Inst.getOperand(OpIdx).getImm() & 0x1FFF);
      }
      case MOSOp::OPERAND_ADDR16: {
        return (Addr & ~AbsAddrMask)
               | (Inst.getOperand(OpIdx).getImm() & 0xFFFF);
      }
      case MOSOp::OPERAND_ADDR24: {
        return (Addr & ~AbsAddrMask)
               | (Inst.getOperand(OpIdx).getImm() & 0xFFFFFF);
      }
    }
  }
  return std::nullopt;
}

} //  namespace llvm
