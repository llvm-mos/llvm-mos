//===-- MOSISelLowering.cpp - MOS DAG Lowering Implementation -------------===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file defines the interfaces that MOS uses to lower LLVM code into a
// selection DAG.
//
//===----------------------------------------------------------------------===//

#include "MOSISelLowering.h"

#include "llvm/ADT/StringSwitch.h"
#include "llvm/CodeGen/CallingConvLower.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/SelectionDAG.h"
#include "llvm/CodeGen/TargetLoweringObjectFileImpl.h"
#include "llvm/IR/Function.h"
#include "llvm/Support/ErrorHandling.h"

#include "MCTargetDesc/MOSMCTargetDesc.h"
#include "MOS.h"
#include "MOSRegisterInfo.h"
#include "MOSSubtarget.h"
#include "MOSTargetMachine.h"

using namespace llvm;

MOSTargetLowering::MOSTargetLowering(const MOSTargetMachine &TM,
                                     const MOSSubtarget &STI)
    : TargetLowering(TM) {
  // This is only used for CallLowering to determine how to split large
  // primitive types for the calling convention. All need to be split to 8 bits,
  // so that's all that we report here. The register class is irrelevant.
  addRegisterClass(MVT::i8, &MOS::Anyi8RegClass);
  computeRegisterProperties(STI.getRegisterInfo());
}

TargetLowering::ConstraintType
MOSTargetLowering::getConstraintType(StringRef Constraint) const {
  if (Constraint.size() == 1) {
    switch (Constraint[0]) {
    default:
      break;
    case 'a':
    case 'x':
    case 'y':
      return C_Register;
    }
  }
  return TargetLowering::getConstraintType(Constraint);
}

std::pair<unsigned, const TargetRegisterClass *>
MOSTargetLowering::getRegForInlineAsmConstraint(const TargetRegisterInfo *TRI,
                                                StringRef Constraint,
                                                MVT VT) const {
  if (Constraint.size() == 1) {
    switch (Constraint[0]) {
    default:
      break;
    case 'r':
      return std::make_pair(0U, &MOS::GPRRegClass);
    case 'a':
      return std::make_pair(MOS::A, &MOS::GPRRegClass);
    case 'x':
      return std::make_pair(MOS::X, &MOS::GPRRegClass);
    case 'y':
      return std::make_pair(MOS::Y, &MOS::GPRRegClass);
    }
  }

  return TargetLowering::getRegForInlineAsmConstraint(TRI, Constraint, VT);
}

bool MOSTargetLowering::isLegalAddressingMode(const DataLayout &DL,
                                              const AddrMode &AM, Type *Ty,
                                              unsigned AddrSpace,
                                              Instruction *I) const {
  // In general, the basereg and scalereg are the 16-bit GEP index type, which
  // cannot be natively supported.

  if (AM.Scale)
    return false;

  if (AM.HasBaseReg) {
    // A 16-bit base reg can be placed into a Imag16 register, then the base
    // offset added using the Y indexed addressing mode. This requires the Y
    // index reg as well as the base reg, but that's what it's there for.
    return !AM.BaseGV && 0 <= AM.BaseOffs && AM.BaseOffs < 256;
  }

  // Any other combination of GV and BaseOffset are just global offsets.
  return true;
}
