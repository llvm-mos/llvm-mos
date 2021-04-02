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

  bool
  assignCalleeSavedSpillSlots(MachineFunction &MF,
                              const TargetRegisterInfo *TRI,
                              std::vector<CalleeSavedInfo> &CSI) const override;

  // Prologues and epilogues are pretty expensive on the 6502; in the worst case
  // they involve a 16-bit addition. This ensures that they are sunk to as small
  // a control flow region around the use of stack as possible. For example,
  // shrink wrapping may move the prologue and epilogue blocks inside of a
  // conditionally-executed block.
  bool enableShrinkWrapping(const MachineFunction &MF) const override {
    return true;
  }

  bool spillCalleeSavedRegisters(MachineBasicBlock &MBB,
                                 MachineBasicBlock::iterator MI,
                                 ArrayRef<CalleeSavedInfo> CSI,
                                 const TargetRegisterInfo *TRI) const override;

  bool
  restoreCalleeSavedRegisters(MachineBasicBlock &MBB,
                              MachineBasicBlock::iterator MI,
                              MutableArrayRef<CalleeSavedInfo> CSI,
                              const TargetRegisterInfo *TRI) const override;

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

private:
  void emitIncSP(MachineIRBuilder &Builder, int64_t Offset) const;
};

} // namespace llvm

#endif // not LLVM_LIB_TARGET_MOS_MOSFRAMELOWERING_H
