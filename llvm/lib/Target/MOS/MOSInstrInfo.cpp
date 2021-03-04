//===-- MOSInstrInfo.cpp - MOS Instruction Information --------------------===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains the MOS implementation of the TargetInstrInfo class.
//
//===----------------------------------------------------------------------===//

#include "MOSInstrInfo.h"

#include "llvm/ADT/STLExtras.h"
#include "llvm/CodeGen/MachineConstantPool.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineMemOperand.h"
#include "llvm/IR/Constants.h"
#include "llvm/IR/Function.h"
#include "llvm/MC/MCContext.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/TargetRegistry.h"

#include "MCTargetDesc/MOSMCTargetDesc.h"
#include "MOS.h"
#include "MOSRegisterInfo.h"
#include "MOSTargetMachine.h"

#define GET_INSTRINFO_CTOR_DTOR
#include "MOSGenInstrInfo.inc"

using namespace llvm;

MOSInstrInfo::MOSInstrInfo() : MOSGenInstrInfo(0, 0) {}

std::pair<unsigned, unsigned>
MOSInstrInfo::decomposeMachineOperandsTargetFlags(unsigned TF) const {
  return std::make_pair(TF, 0u);
}

ArrayRef<std::pair<unsigned, const char *>>
MOSInstrInfo::getSerializableDirectMachineOperandTargetFlags() const {
  static const std::pair<unsigned, const char *> Flags[] = {{MOS::MO_LO, "lo"},
                                                            {MOS::MO_HI, "hi"}};
  return Flags;
}
