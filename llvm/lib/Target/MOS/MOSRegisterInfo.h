//===-- MOSRegisterInfo.h - MOS Register Information Impl -------*- C++ -*-===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains the MOS implementation of the TargetRegisterInfo class.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MOS_MOSREGISTERINFO_H
#define LLVM_LIB_TARGET_MOS_MOSREGISTERINFO_H

#include "llvm/CodeGen/TargetRegisterInfo.h"

#define GET_REGINFO_HEADER
#include "MOSGenRegisterInfo.inc"

namespace llvm {

class MOSRegisterInfo : public MOSGenRegisterInfo {
  std::unique_ptr<std::string[]> Imag8SymbolNames;
  BitVector Reserved;

public:
  MOSRegisterInfo();

  const MCPhysReg *getCalleeSavedRegs(const MachineFunction *MF) const override;

  const uint32_t *getCallPreservedMask(const MachineFunction &MF,
                                       CallingConv::ID) const override;

  BitVector getReservedRegs(const MachineFunction &MF) const override;

  const TargetRegisterClass *
  getLargestLegalSuperClass(const TargetRegisterClass *RC,
                            const MachineFunction &) const override;

  bool requiresRegisterScavenging(const MachineFunction &MF) const override {
    // Saving/restoring to stack may require temporary registers.
    return true;
  }

  bool requiresFrameIndexScavenging(const MachineFunction &MF) const override {
    // Saving/restoring to stack may require temporary registers.
    return true;
  }

  bool saveScavengerRegister(MachineBasicBlock &MBB,
                             MachineBasicBlock::iterator I,
                             MachineBasicBlock::iterator &UseMI,
                             const TargetRegisterClass *RC,
                             Register Reg) const override;

  void eliminateFrameIndex(MachineBasicBlock::iterator MI, int SPAdj,
                           unsigned FIOperandNum,
                           RegScavenger *RS = nullptr) const override;

  void expandLDSTstk(MachineBasicBlock::iterator MI) const;

  Register getFrameRegister(const MachineFunction &MF) const override;

  bool shouldCoalesce(MachineInstr *MI, const TargetRegisterClass *SrcRC,
                      unsigned SubReg, const TargetRegisterClass *DstRC,
                      unsigned DstSubReg, const TargetRegisterClass *NewRC,
                      LiveIntervals &LIS) const override;

  const char *getImag8SymbolName(Register Reg) const {
    return Imag8SymbolNames[Reg].c_str();
  }
};

} // namespace llvm

#endif // not LLVM_LIB_TARGET_MOS_MOSREGISTERINFO_H
