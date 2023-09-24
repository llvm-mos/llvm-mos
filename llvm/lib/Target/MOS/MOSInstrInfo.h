//===-- MOSInstrInfo.h - MOS Instruction Information ------------*- C++ -*-===//
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

#ifndef LLVM_LIB_TARGET_MOS_MOSINSTRINFO_H
#define LLVM_LIB_TARGET_MOS_MOSINSTRINFO_H

#include "llvm/CodeGen/GlobalISel/MachineIRBuilder.h"
#include "llvm/CodeGen/TargetInstrInfo.h"

#define GET_INSTRINFO_HEADER
#include "MOSGenInstrInfo.inc"

namespace llvm {

class MOSInstrInfo : public MOSGenInstrInfo {
public:
  MOSInstrInfo();

  bool isReallyTriviallyReMaterializable(const MachineInstr &MI) const override;

  unsigned isLoadFromStackSlot(const MachineInstr &MI,
                               int &FrameIndex) const override;

  unsigned isStoreToStackSlot(const MachineInstr &MI,
                              int &FrameIndex) const override;

  void reMaterialize(MachineBasicBlock &MBB, MachineBasicBlock::iterator MI,
                     Register DestReg, unsigned SubIdx,
                     const MachineInstr &Orig,
                     const TargetRegisterInfo &TRI) const override;

  MachineInstr *commuteInstructionImpl(MachineInstr &MI, bool NewMI,
                                       unsigned OpIdx1,
                                       unsigned OpIdx2) const override;

  unsigned getInstSizeInBytes(const MachineInstr &MI) const override;

  bool findCommutedOpIndices(const MachineInstr &MI, unsigned &SrcOpIdx1,
                             unsigned &SrcOpIdx2) const override;

  bool isBranchOffsetInRange(unsigned BranchOpc,
                             int64_t BrOffset) const override;

  MachineBasicBlock *getBranchDestBlock(const MachineInstr &MI) const override;

  bool analyzeBranch(MachineBasicBlock &MBB, MachineBasicBlock *&TBB,
                     MachineBasicBlock *&FBB,
                     SmallVectorImpl<MachineOperand> &Cond,
                     bool AllowModify = false) const override;

  unsigned removeBranch(MachineBasicBlock &MBB,
                        int *BytesRemoved = nullptr) const override;

  unsigned insertBranch(MachineBasicBlock &MBB, MachineBasicBlock *TBB,
                        MachineBasicBlock *FBB, ArrayRef<MachineOperand> Cond,
                        const DebugLoc &DL,
                        int *BytesAdded = nullptr) const override;

  void insertIndirectBranch(MachineBasicBlock &MBB,
                            MachineBasicBlock &NewDestBB,
                            MachineBasicBlock &RestoreBB, const DebugLoc &DL,
                            int64_t BrOffset = 0,
                            RegScavenger *RS = nullptr) const override;

  void copyPhysReg(MachineBasicBlock &MBB, MachineBasicBlock::iterator MI,
                   const DebugLoc &DL, MCRegister DestReg, MCRegister SrcReg,
                   bool KillSrc) const override;

  void storeRegToStackSlot(MachineBasicBlock &MBB,
                           MachineBasicBlock::iterator MI, Register SrcReg,
                           bool isKill, int FrameIndex,
                           const TargetRegisterClass *RC,
                           const TargetRegisterInfo *TRI,
                           Register VReg) const override;

  const TargetRegisterClass *canFoldCopy(const MachineInstr &MI,
                                         const TargetInstrInfo &TII,
                                         unsigned FoldIdx) const override;

  void loadRegFromStackSlot(MachineBasicBlock &MBB,
                            MachineBasicBlock::iterator MI, Register DestReg,
                            int FrameIndex, const TargetRegisterClass *RC,
                            const TargetRegisterInfo *TRI,
                            Register VReg) const override;

  void loadStoreRegStackSlot(MachineBasicBlock &MBB,
                             MachineBasicBlock::iterator MI, Register Reg,
                             bool IsKill, int FrameIndex,
                             const TargetRegisterClass *RC,
                             const TargetRegisterInfo *TRI, bool IsLoad) const;

  bool expandPostRAPseudo(MachineInstr &MI) const override;

  bool
  reverseBranchCondition(SmallVectorImpl<MachineOperand> &Cond) const override;

  std::pair<unsigned, unsigned>
  decomposeMachineOperandsTargetFlags(unsigned TF) const override;

  ArrayRef<std::pair<int, const char *>>
  getSerializableTargetIndices() const override;

  ArrayRef<std::pair<unsigned, const char *>>
  getSerializableDirectMachineOperandTargetFlags() const override;

  bool shouldOverlapInterval(const MachineInstr &MI) const override;

  bool hasCustomTiedOperands(unsigned Opcode) const override;

  unsigned findCustomTiedOperandIdx(const MachineInstr &MI,
                                    unsigned OpIdx) const override;

private:
  void copyPhysRegImpl(MachineIRBuilder &Builder, Register DestReg,
                       Register SrcReg, bool Force = false,
                       bool KillSrc = false) const;

  Register getRegWithVal(MachineIRBuilder &Builder, Register Val,
                         const TargetRegisterClass &RC) const;

  // Post RA pseudos
  void expandLDIdx(MachineIRBuilder &Builder, bool ZP) const;
  void expandLDImm1(MachineIRBuilder &Builder) const;
  void expandLDImm16(MachineIRBuilder &Builder) const;
  void expandLDImm16Remat(MachineIRBuilder &Builder) const;
  void expandLDZ(MachineIRBuilder &Builder) const;
  void expandIncDec(MachineIRBuilder &Builder) const;
  void expandIncDecPtr(MachineIRBuilder &Builder) const;

  // CMP pseudos
  void expandCMPTerm(MachineIRBuilder &Builder) const;

  // Control flow pseudos
  void expandGBR(MachineIRBuilder &Builder) const;

  bool shouldRematPhysRegCopy() const override { return false; }
};

namespace MOS {

enum AddressSpace {
  AS_Memory,
  AS_ZeroPage,
  NumAddrSpaces
};

enum TargetIndex {
  TI_STATIC_STACK,
};

enum TOF {
  MO_NO_FLAGS = 0,
  MO_LO,
  MO_HI,
  MO_HI_JT,
  MO_ZEROPAGE,
};

} // namespace MOS

} // namespace llvm

#endif // not LLVM_LIB_TARGET_MOS_MOSINSTRINFO_H
