//===-- MOSFrameLowering.h - Define frame lowering for MOS ------*- C++ -*-===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains the MOS declaration of TargetFrameLowering class.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MOS_MOSFRAMELOWERING_H
#define LLVM_LIB_TARGET_MOS_MOSFRAMELOWERING_H

#include "llvm/CodeGen/GlobalISel/MachineIRBuilder.h"
#include "llvm/CodeGen/TargetFrameLowering.h"

namespace llvm {

class MOSFrameLowering : public TargetFrameLowering {
public:
  MOSFrameLowering();

  bool usesStaticStack(const MachineFunction &MF) const;

  bool
  assignCalleeSavedSpillSlots(MachineFunction &MF,
                              const TargetRegisterInfo *TRI,
                              std::vector<CalleeSavedInfo> &CSI) const override;

  bool enableShrinkWrapping(const MachineFunction &MF) const override;

  bool spillCalleeSavedRegisters(MachineBasicBlock &MBB,
                                 MachineBasicBlock::iterator MI,
                                 ArrayRef<CalleeSavedInfo> CSI,
                                 const TargetRegisterInfo *TRI) const override;

  bool
  restoreCalleeSavedRegisters(MachineBasicBlock &MBB,
                              MachineBasicBlock::iterator MI,
                              MutableArrayRef<CalleeSavedInfo> CSI,
                              const TargetRegisterInfo *TRI) const override;

  bool enableCalleeSaveSkip(const MachineFunction &) const override;

  void determineCalleeSaves(MachineFunction &MF, BitVector &SavedRegs,
                            RegScavenger *RS) const override;

  void processFunctionBeforeFrameFinalized(
      MachineFunction &MF, RegScavenger *RS = nullptr) const override;

  MachineBasicBlock::iterator
  eliminateCallFramePseudoInstr(MachineFunction &MF, MachineBasicBlock &MBB,
                                MachineBasicBlock::iterator MI) const override;

  void emitPrologue(MachineFunction &MF, MachineBasicBlock &MBB) const override;
  void emitEpilogue(MachineFunction &MF, MachineBasicBlock &MBB) const override;
  bool hasFP(const MachineFunction &MF) const override;

  // Computes the size of the static stack.
  uint64_t staticSize(const MachineFrameInfo &MFI) const;

  // Return whether or not the function is a direct ISR.
  bool isISR(const MachineFunction& MF) const;

private:
  void offsetSP(MachineIRBuilder &Builder, int64_t Offset) const;
};

} // namespace llvm

#endif // not LLVM_LIB_TARGET_MOS_MOSFRAMELOWERING_H
