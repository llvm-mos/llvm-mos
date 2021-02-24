//===-- MOSRegisterInfo.h - MOS Register Information Impl -------*- C++ -*-===//
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

#ifndef LLVM_MOS_REGISTER_INFO_H
#define LLVM_MOS_REGISTER_INFO_H

#include "llvm/CodeGen/TargetRegisterInfo.h"

#define GET_REGINFO_HEADER
#include "MOSGenRegisterInfo.inc"

namespace llvm {

/// Utilities relating to MOS registers.
class MOSRegisterInfo : public MOSGenRegisterInfo {
  BitVector Reserved;

public:
  MOSRegisterInfo();

  void eliminateFrameIndex(MachineBasicBlock::iterator MI, int SPAdj,
                           unsigned FIOperandNum,
                           RegScavenger *RS = NULL) const override;

  const MCPhysReg *
  getCalleeSavedRegs(const MachineFunction *MF = 0) const override;

  const uint32_t *getCallPreservedMask(const MachineFunction &MF,
                                       CallingConv::ID) const override;

  virtual Register getFrameRegister(const MachineFunction &MF) const override;

  BitVector getReservedRegs(const MachineFunction &MF) const override;

private:
  void markSubRegs(BitVector &RegisterSet, MCRegister Reg);
};

} // end namespace llvm

#endif // LLVM_MOS_REGISTER_INFO_H
