//===-- MOSInstrInfo.h - MOS Instruction Information ------------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the MOS implementation of the TargetInstrInfo class.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_MOS_INSTR_INFO_H
#define LLVM_MOS_INSTR_INFO_H

#include "llvm/CodeGen/TargetInstrInfo.h"

#include "MOSRegisterInfo.h"

#define GET_INSTRINFO_HEADER
#include "MOSGenInstrInfo.inc"
#undef GET_INSTRINFO_HEADER

namespace llvm {

namespace MOSCC {

/// MOS specific condition codes.
/// These correspond to `MOS_*_COND` in `MOSInstrInfo.td`.
/// They must be kept in synch.
enum CondCodes {
  COND_EQ, //!< Equal
  COND_NE, //!< Not equal
  COND_GE, //!< Greater than or equal
  COND_LT, //!< Less than
  COND_SH, //!< Unsigned same or higher
  COND_LO, //!< Unsigned lower
  COND_MI, //!< Minus
  COND_PL, //!< Plus
  COND_INVALID
};

} // end of namespace MOSCC

namespace MOS {

/// Specifies a target operand flag.
enum TOF {
  MO_NO_FLAG,
};

} // end of namespace MOSII

/// Utilities related to the MOS instruction set.
class MOSInstrInfo : public MOSGenInstrInfo {
public:
  explicit MOSInstrInfo();

  // Branch analysis.
  bool analyzeBranch(MachineBasicBlock &MBB, MachineBasicBlock *&TBB,
                     MachineBasicBlock *&FBB,
                     SmallVectorImpl<MachineOperand> &Cond,
                     bool AllowModify = false) const override {return false;}
  void copyPhysReg(MachineBasicBlock &MBB, MachineBasicBlock::iterator MI,
                   const DebugLoc &DL, unsigned DestReg, unsigned SrcReg,
                   bool KillSrc) const override {}
  MachineBasicBlock *getBranchDestBlock(const MachineInstr &MI) const override {return nullptr;}

  const MCInstrDesc &getBrCond(MOSCC::CondCodes CC) const { return ID; }
  const MOSRegisterInfo &getRegisterInfo() const { return RI; }
  MOSCC::CondCodes getCondFromBranchOpc(unsigned Opc) const {return MOSCC::CondCodes::COND_INVALID;};
  MOSCC::CondCodes getOppositeCondition(MOSCC::CondCodes CC) const {return MOSCC::CondCodes::COND_INVALID;}
  unsigned getInstSizeInBytes(const MachineInstr &MI) const override {return 0;}

  unsigned insertBranch(MachineBasicBlock &MBB, MachineBasicBlock *TBB,
                        MachineBasicBlock *FBB, ArrayRef<MachineOperand> Cond,
                        const DebugLoc &DL,
                        int *BytesAdded = nullptr) const override {return 0;}
  unsigned insertIndirectBranch(MachineBasicBlock &MBB,
                                MachineBasicBlock &NewDestBB,
                                const DebugLoc &DL,
                                int64_t BrOffset,
                                RegScavenger *RS) const override {return 0;}
  bool isBranchOffsetInRange(unsigned BranchOpc,
                             int64_t BrOffset) const override {return false;}

  unsigned isLoadFromStackSlot(const MachineInstr &MI,
                               int &FrameIndex) const override {return 0;}
  unsigned isStoreToStackSlot(const MachineInstr &MI,
                              int &FrameIndex) const override {return 0;}

  void loadRegFromStackSlot(MachineBasicBlock &MBB,
                            MachineBasicBlock::iterator MI, unsigned DestReg,
                            int FrameIndex, const TargetRegisterClass *RC,
                            const TargetRegisterInfo *TRI) const override {}
  unsigned removeBranch(MachineBasicBlock &MBB,
                        int *BytesRemoved = nullptr) const override {return 0;}
  bool
  reverseBranchCondition(SmallVectorImpl<MachineOperand> &Cond) const override {return false;}

  void storeRegToStackSlot(MachineBasicBlock &MBB,
                           MachineBasicBlock::iterator MI, unsigned SrcReg,
                           bool isKill, int FrameIndex,
                           const TargetRegisterClass *RC,
                           const TargetRegisterInfo *TRI) const override {}
private:
  const MOSRegisterInfo RI;
  const MCInstrDesc ID;
};

} // end namespace llvm

#endif // LLVM_MOS_INSTR_INFO_H
