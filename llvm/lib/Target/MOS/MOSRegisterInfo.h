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
  /// Stack Frame Processing Methods
  void eliminateFrameIndex(MachineBasicBlock::iterator MI, int SPAdj,
                           unsigned FIOperandNum,
                           RegScavenger *RS = NULL) const override;

  const uint16_t *
  getCalleeSavedRegs(const MachineFunction *MF = 0) const override;
  const uint32_t *getCallPreservedMask(const MachineFunction &MF,
                                       CallingConv::ID CC) const override;
  virtual Register getFrameRegister(const MachineFunction &MF) const override;

  const TargetRegisterClass *
  getLargestLegalSuperClass(const TargetRegisterClass *RC,
                            const MachineFunction &MF) const override;

  const TargetRegisterClass *
  getPointerRegClass(const MachineFunction &MF,
                     unsigned Kind = 0) const override;

  BitVector getReservedRegs(const MachineFunction &MF) const override;

  /// Splits a 16-bit `DREGS` register into the lo/hi register pair.
  /// \param Reg A 16-bit register to split.
  // void splitReg(unsigned Reg, unsigned &LoReg, unsigned &HiReg) const;

  bool trackLivenessAfterRegAlloc(const MachineFunction &) const override;

protected:
  
};

} // end namespace llvm

#endif // LLVM_MOS_REGISTER_INFO_H
