//===-- MOSRegisterInfo.cpp - MOS Register Information --------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains the MOS implementation of the TargetRegisterInfo class.
//
//===----------------------------------------------------------------------===//

#include "MOSRegisterInfo.h"

#include "llvm/ADT/BitVector.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/TargetFrameLowering.h"
#include "llvm/IR/Function.h"
#include "llvm/MC/MCRegisterInfo.h"
#include "llvm/Support/ErrorHandling.h"

#include "MCTargetDesc/MOSMCTargetDesc.h"
#include "MOS.h"
#include "MOSInstrInfo.h"
#include "MOSTargetMachine.h"

#define GET_REGINFO_TARGET_DESC
#include "MOSGenRegisterInfo.inc"

using namespace llvm;

cl::opt<int> NumImagPtrs(
    "num-imag-ptrs", cl::init(128),
    cl::desc(
        "Number of imaginary pointer registers available for compiler use."),
    cl::value_desc("imaginary pointers"));

MOSRegisterInfo::MOSRegisterInfo()
    : MOSGenRegisterInfo(/*RA=*/0), Reserved(getNumRegs()) {
  // One stack pointer, one frame pointer, and at least one pointer for indirect
  // pointer access are necessary in the worst case.
  if (NumImagPtrs < 3)
    report_fatal_error("At least three zero-page pointers must be available.");

  // There are only 128 addressable zero-page register pairs.
  if (NumImagPtrs > 128)
    report_fatal_error("More than 128 zero-page pointers cannot be available.");

  // Any unavailable imaginary registers are reserved so the compiler cannot
  // make use of them.
  for (int Idx = 0; Idx < 128 - NumImagPtrs; Idx++)
    markSubRegs(Reserved, MOS::RS127 - Idx);

  // RS0 is reserved as the soft stack pointer.
  markSubRegs(Reserved, MOS::RS0);

  // S is reserved as the hard stack pointer.
  Reserved.set(MOS::S);

  assert(checkAllSuperRegsMarked(Reserved));
}

void MOSRegisterInfo::eliminateFrameIndex(MachineBasicBlock::iterator MI,
                                          int SPAdj, unsigned FIOperandNum,
                                          RegScavenger *RS /*= NULL*/) const {
  report_fatal_error("Not yet implemented.");
}

const MCPhysReg *
MOSRegisterInfo::getCalleeSavedRegs(const MachineFunction *MF /*= 0*/) const {
  return MOS_CSR_SaveList;
}

const uint32_t *MOSRegisterInfo::getCallPreservedMask(const MachineFunction &MF,
                                                      CallingConv::ID) const {
  return MOS_CSR_RegMask;
}

Register MOSRegisterInfo::getFrameRegister(const MachineFunction &MF) const {
  report_fatal_error("Not yet implemented.");
}

llvm::BitVector
MOSRegisterInfo::getReservedRegs(const MachineFunction &MF) const {
  return Reserved;
}

/// Adds \p Reg and all its subregisters to the \p RegisterSet
void MOSRegisterInfo::markSubRegs(BitVector &RegisterSet, MCRegister Reg) {
  for (MCSubRegIterator AI(Reg, this, /*IncludeSelf=*/true); AI.isValid(); ++AI)
    RegisterSet.set(*AI);
}