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

  StackOffset getFrameIndexReference(const MachineFunction &MF, int FI,
                                     Register &FrameReg) const override;

  // Rewrite frame-index debug operands for zero-page and static-stack locals
  // into GlobalAddress / TargetIndex operands. These locals have absolute
  // addresses, which the register + offset form PEI would otherwise
  // substitute cannot express.
  bool replaceFrameIndexDebugOperand(MachineFunction &MF, MachineInstr &MI,
                                     unsigned OpIdx) const override;

  void resetCFIToInitialState(MachineBasicBlock &MBB) const override;

  // Computes the size of the static stack.
  uint64_t staticSize(const MachineFrameInfo &MFI) const;

  // Return whether or not the function is a direct ISR.
  bool isISR(const MachineFunction &MF) const;

private:
  bool hasFPImpl(const MachineFunction &MF) const override;

  void offsetSP(MachineIRBuilder &Builder, int64_t Offset) const;

  // The amount the prologue subtracts from the soft stack pointer.
  int64_t softStackAdjustment(const MachineFunction &MF) const;

  void buildCFI(MachineBasicBlock &MBB, MachineBasicBlock::iterator MBBI,
                const DebugLoc &DL, const MCCFIInstruction &CFIInst,
                MachineInstr::MIFlag Flag = MachineInstr::NoFlags) const;

  // Emit the hardware-stack CFI directives (CFA, PC, S expressions, plus P
  // for ISRs) that describe the state after `HardStackCSRCount` CSR pushes
  // on top of a JSR-entry (`IsISR=false`) or interrupt-entry (`IsISR=true`)
  // stack layout. Called at ISR entry and after each push/pull. The CFA is
  // always hardware-stack based.
  void emitHardwareStackCFI(MachineBasicBlock &MBB,
                            MachineBasicBlock::iterator MBBI,
                            const DebugLoc &DL, unsigned HardStackCSRCount,
                            bool IsISR, MachineInstr::MIFlag Flag) const;

  // Emit CFI describing the soft-stack frame relative to `Base`, which holds
  // the soft stack pointer as adjusted by the prologue (RS0, or the frame
  // pointer once it is set): if `DescribeRS0`, the caller's RS0, and if
  // `DescribeCSRs`, the save slot of each soft-stack callee-saved register.
  void emitSoftStackCFI(MachineBasicBlock &MBB,
                        MachineBasicBlock::iterator MBBI, const DebugLoc &DL,
                        Register Base, bool DescribeRS0, bool DescribeCSRs,
                        MachineInstr::MIFlag Flag) const;

  // Emit DW_CFA_restore for each soft-stack callee-saved register.
  void emitSoftStackCSRRestoreCFI(MachineBasicBlock &MBB,
                                  MachineBasicBlock::iterator MBBI,
                                  const DebugLoc &DL,
                                  MachineInstr::MIFlag Flag) const;
};

} // namespace llvm

#endif // not LLVM_LIB_TARGET_MOS_MOSFRAMELOWERING_H
