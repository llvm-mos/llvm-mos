//===-- MOSInstrBuilder.h - Functions to aid building insts -----*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file exposes functions that may be used to handle MOS instruction
// building quirks, including subtarget-dependent selection, in a cleaner way.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MOS_MOSINSTRBUILDER_H
#define LLVM_LIB_TARGET_MOS_MOSINSTRBUILDER_H

#include "MOSSubtarget.h"
#include "MCTargetDesc/MOSMCTargetDesc.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineMemOperand.h"
#include "llvm/CodeGen/MachineOperand.h"

namespace llvm {

static inline unsigned getIncPseudoOpcode(const MachineIRBuilder &Builder) {
  const MOSSubtarget &STI = Builder.getMF().getSubtarget<MOSSubtarget>();
  return STI.hasGPRIncDec() ? MOS::INC : MOS::IncNMOS;
}

static inline unsigned getDecPseudoOpcode(const MachineIRBuilder &Builder) {
  const MOSSubtarget &STI = Builder.getMF().getSubtarget<MOSSubtarget>();
  return STI.hasGPRIncDec() ? MOS::DEC : MOS::DecNMOS;
}

static inline MachineInstrBuilder
buildLdImm(MachineIRBuilder &Builder, DstOp Dest) {
  const MOSSubtarget &STI = Builder.getMF().getSubtarget<MOSSubtarget>();
  LLT DestType = Dest.getLLTTy(*Builder.getMRI());
  assert(DestType.isByteSized() && DestType.getScalarSizeInBits() <= 16);

  if (DestType.getScalarSizeInBits() == 16) {
    if (STI.hasSPC700()) {
      return Builder
              .buildInstr(MOS::LDImm16SPC700, {Dest}, {});
    }

    return Builder
            .buildInstr(MOS::LDImm16, {Dest, &MOS::GPRRegClass}, {});
  }

  return Builder.buildInstr(MOS::LDImm, {Dest}, {});
}

} // end namespace llvm

#endif
