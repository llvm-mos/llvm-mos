//===-- MOSRegisterInfo.h - MOS Register Information Impl -------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
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
public:
  MOSRegisterInfo();

public:
  void eliminateFrameIndex(MachineBasicBlock::iterator MI, int SPAdj,
                           unsigned FIOperandNum,
                           RegScavenger *RS = NULL) const override;

  const uint16_t *
  getCalleeSavedRegs(const MachineFunction *MF = 0) const override;
  virtual Register getFrameRegister(const MachineFunction &MF) const override;

  BitVector getReservedRegs(const MachineFunction &MF) const override;
};

} // end namespace llvm

#endif // LLVM_MOS_REGISTER_INFO_H
