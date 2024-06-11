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

#include "MOSInstrCost.h"

#include "llvm/ADT/BitVector.h"
#include "llvm/CodeGen/MachineOperand.h"
#include "llvm/CodeGen/TargetRegisterInfo.h"

#define GET_REGINFO_HEADER
#include "MOSGenRegisterInfo.inc"

namespace llvm {

class MOSSubtarget;

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

  const TargetRegisterClass *
  getCrossCopyRegClass(const TargetRegisterClass *RC) const override;

  unsigned getCSRFirstUseCost(const MachineFunction &MF) const override;

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

  bool
  canSaveScavengerRegister(Register Reg, MachineBasicBlock::iterator I,
                           MachineBasicBlock::iterator UseMI) const override;

  bool eliminateFrameIndex(MachineBasicBlock::iterator MI, int SPAdj,
                           unsigned FIOperandNum,
                           RegScavenger *RS = nullptr) const override;

  void expandAddrLostk(MachineBasicBlock::iterator MI) const;
  void expandAddrHistk(MachineBasicBlock::iterator MI) const;
  void expandLDSTStk(MachineBasicBlock::iterator MI) const;

  Register getFrameRegister(const MachineFunction &MF) const override;

  bool shouldCoalesce(MachineInstr *MI, const TargetRegisterClass *SrcRC,
                      unsigned SubReg, const TargetRegisterClass *DstRC,
                      unsigned DstSubReg, const TargetRegisterClass *NewRC,
                      LiveIntervals &LIS) const override;

  bool
  getRegAllocationHints(Register VirtReg, ArrayRef<MCPhysReg> Order,
                        SmallVectorImpl<MCPhysReg> &Hints,
                        const MachineFunction &MF,
                        const VirtRegMap *VRM = nullptr,
                        const LiveRegMatrix *Matrix = nullptr) const override;

  const char *getImag8SymbolName(Register Reg) const {
    return Imag8SymbolNames[Reg].c_str();
  }

  MOSInstrCost copyCost(Register DestReg, Register SrcReg,
                        const MOSSubtarget &STI) const;

private:
  void reserveAllSubregs(BitVector *Reserved, Register Reg) const;

  std::optional<Register> getStrongCopyHint(Register VirtReg,
                                            const MachineFunction &MF,
                                            const VirtRegMap *VRM) const;
};

} // namespace llvm

#endif // not LLVM_LIB_TARGET_MOS_MOSREGISTERINFO_H
