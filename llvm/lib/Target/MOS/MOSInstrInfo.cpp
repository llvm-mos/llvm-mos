//===-- MOSInstrInfo.cpp - MOS Instruction Information --------------------===//
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

#include "MOSInstrInfo.h"

#include "llvm/ADT/STLExtras.h"
#include "llvm/CodeGen/MachineConstantPool.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineMemOperand.h"
#include "llvm/IR/Constants.h"
#include "llvm/IR/Function.h"
#include "llvm/MC/MCContext.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/TargetRegistry.h"

#include "MOS.h"
#include "MOSMachineFunctionInfo.h"
#include "MOSRegisterInfo.h"
#include "MOSTargetMachine.h"
#include "MCTargetDesc/MOSMCTargetDesc.h"

#define GET_INSTRINFO_CTOR_DTOR
#include "MOSGenInstrInfo.inc"

namespace llvm {

MOSInstrInfo::MOSInstrInfo()
    : MOSGenInstrInfo(MOS::ADJCALLSTACKDOWN, MOS::ADJCALLSTACKUP), RI() {}

void MOSInstrInfo::copyPhysReg(MachineBasicBlock &MBB,
                               MachineBasicBlock::iterator MI,
                               const DebugLoc &DL, unsigned DestReg,
                               unsigned SrcReg, bool KillSrc) const {
  const MOSSubtarget &STI = MBB.getParent()->getSubtarget<MOSSubtarget>();
  const MOSRegisterInfo &TRI = *STI.getRegisterInfo();
  unsigned Opc;

  // Not all MOS devices support the 16-bit `MOVW` instruction.
  if (MOS::DREGSRegClass.contains(DestReg, SrcReg)) {
    if (STI.hasMOVW()) {
      BuildMI(MBB, MI, DL, get(MOS::MOVWRdRr), DestReg)
          .addReg(SrcReg, getKillRegState(KillSrc));
    } else {
      unsigned DestLo, DestHi, SrcLo, SrcHi;

      TRI.splitReg(DestReg, DestLo, DestHi);
      TRI.splitReg(SrcReg,  SrcLo,  SrcHi);

      // Copy each individual register with the `MOV` instruction.
      BuildMI(MBB, MI, DL, get(MOS::MOVRdRr), DestLo)
        .addReg(SrcLo, getKillRegState(KillSrc));
      BuildMI(MBB, MI, DL, get(MOS::MOVRdRr), DestHi)
        .addReg(SrcHi, getKillRegState(KillSrc));
    }
  } else {
    if (MOS::GPR8RegClass.contains(DestReg, SrcReg)) {
      Opc = MOS::MOVRdRr;
    } else if (SrcReg == MOS::SP && MOS::DREGSRegClass.contains(DestReg)) {
      Opc = MOS::SPREAD;
    } else if (DestReg == MOS::SP && MOS::DREGSRegClass.contains(SrcReg)) {
      Opc = MOS::SPWRITE;
    } else {
      llvm_unreachable("Impossible reg-to-reg copy");
    }

    BuildMI(MBB, MI, DL, get(Opc), DestReg)
        .addReg(SrcReg, getKillRegState(KillSrc));
  }
}

unsigned MOSInstrInfo::isLoadFromStackSlot(const MachineInstr &MI,
                                           int &FrameIndex) const {
  switch (MI.getOpcode()) {
  case MOS::LDDRdPtrQ:
  case MOS::LDDWRdYQ: { //:FIXME: remove this once PR13375 gets fixed
    if (MI.getOperand(1).isFI() && MI.getOperand(2).isImm() &&
        MI.getOperand(2).getImm() == 0) {
      FrameIndex = MI.getOperand(1).getIndex();
      return MI.getOperand(0).getReg();
    }
    break;
  }
  default:
    break;
  }

  return 0;
}

unsigned MOSInstrInfo::isStoreToStackSlot(const MachineInstr &MI,
                                          int &FrameIndex) const {
  switch (MI.getOpcode()) {
  case MOS::STDPtrQRr:
  case MOS::STDWPtrQRr: {
    if (MI.getOperand(0).isFI() && MI.getOperand(1).isImm() &&
        MI.getOperand(1).getImm() == 0) {
      FrameIndex = MI.getOperand(0).getIndex();
      return MI.getOperand(2).getReg();
    }
    break;
  }
  default:
    break;
  }

  return 0;
}

void MOSInstrInfo::storeRegToStackSlot(MachineBasicBlock &MBB,
                                       MachineBasicBlock::iterator MI,
                                       unsigned SrcReg, bool isKill,
                                       int FrameIndex,
                                       const TargetRegisterClass *RC,
                                       const TargetRegisterInfo *TRI) const {
  MachineFunction &MF = *MBB.getParent();
  MOSMachineFunctionInfo *AFI = MF.getInfo<MOSMachineFunctionInfo>();

  AFI->setHasSpills(true);

  DebugLoc DL;
  if (MI != MBB.end()) {
    DL = MI->getDebugLoc();
  }

  const MachineFrameInfo &MFI = MF.getFrameInfo();

  MachineMemOperand *MMO = MF.getMachineMemOperand(
      MachinePointerInfo::getFixedStack(MF, FrameIndex),
      MachineMemOperand::MOStore, MFI.getObjectSize(FrameIndex),
      MFI.getObjectAlignment(FrameIndex));

  unsigned Opcode = 0;
  if (TRI->isTypeLegalForClass(*RC, MVT::i8)) {
    Opcode = MOS::STDPtrQRr;
  } else if (TRI->isTypeLegalForClass(*RC, MVT::i16)) {
    Opcode = MOS::STDWPtrQRr;
  } else {
    llvm_unreachable("Cannot store this register into a stack slot!");
  }

  BuildMI(MBB, MI, DL, get(Opcode))
      .addFrameIndex(FrameIndex)
      .addImm(0)
      .addReg(SrcReg, getKillRegState(isKill))
      .addMemOperand(MMO);
}

void MOSInstrInfo::loadRegFromStackSlot(MachineBasicBlock &MBB,
                                        MachineBasicBlock::iterator MI,
                                        unsigned DestReg, int FrameIndex,
                                        const TargetRegisterClass *RC,
                                        const TargetRegisterInfo *TRI) const {
  DebugLoc DL;
  if (MI != MBB.end()) {
    DL = MI->getDebugLoc();
  }

  MachineFunction &MF = *MBB.getParent();
  const MachineFrameInfo &MFI = MF.getFrameInfo();

  MachineMemOperand *MMO = MF.getMachineMemOperand(
      MachinePointerInfo::getFixedStack(MF, FrameIndex),
      MachineMemOperand::MOLoad, MFI.getObjectSize(FrameIndex),
      MFI.getObjectAlignment(FrameIndex));

  unsigned Opcode = 0;
  if (TRI->isTypeLegalForClass(*RC, MVT::i8)) {
    Opcode = MOS::LDDRdPtrQ;
  } else if (TRI->isTypeLegalForClass(*RC, MVT::i16)) {
    // Opcode = MOS::LDDWRdPtrQ;
    //:FIXME: remove this once PR13375 gets fixed
    Opcode = MOS::LDDWRdYQ;
  } else {
    llvm_unreachable("Cannot load this register from a stack slot!");
  }

  BuildMI(MBB, MI, DL, get(Opcode), DestReg)
      .addFrameIndex(FrameIndex)
      .addImm(0)
      .addMemOperand(MMO);
}

const MCInstrDesc &MOSInstrInfo::getBrCond(MOSCC::CondCodes CC) const {
  switch (CC) {
  default:
    llvm_unreachable("Unknown condition code!");
  case MOSCC::COND_EQ:
    return get(MOS::BREQk);
  case MOSCC::COND_NE:
    return get(MOS::BRNEk);
  case MOSCC::COND_GE:
    return get(MOS::BRGEk);
  case MOSCC::COND_LT:
    return get(MOS::BRLTk);
  case MOSCC::COND_SH:
    return get(MOS::BRSHk);
  case MOSCC::COND_LO:
    return get(MOS::BRLOk);
  case MOSCC::COND_MI:
    return get(MOS::BRMIk);
  case MOSCC::COND_PL:
    return get(MOS::BRPLk);
  }
}

MOSCC::CondCodes MOSInstrInfo::getCondFromBranchOpc(unsigned Opc) const {
  switch (Opc) {
  default:
    return MOSCC::COND_INVALID;
  case MOS::BREQk:
    return MOSCC::COND_EQ;
  case MOS::BRNEk:
    return MOSCC::COND_NE;
  case MOS::BRSHk:
    return MOSCC::COND_SH;
  case MOS::BRLOk:
    return MOSCC::COND_LO;
  case MOS::BRMIk:
    return MOSCC::COND_MI;
  case MOS::BRPLk:
    return MOSCC::COND_PL;
  case MOS::BRGEk:
    return MOSCC::COND_GE;
  case MOS::BRLTk:
    return MOSCC::COND_LT;
  }
}

MOSCC::CondCodes MOSInstrInfo::getOppositeCondition(MOSCC::CondCodes CC) const {
  switch (CC) {
  default:
    llvm_unreachable("Invalid condition!");
  case MOSCC::COND_EQ:
    return MOSCC::COND_NE;
  case MOSCC::COND_NE:
    return MOSCC::COND_EQ;
  case MOSCC::COND_SH:
    return MOSCC::COND_LO;
  case MOSCC::COND_LO:
    return MOSCC::COND_SH;
  case MOSCC::COND_GE:
    return MOSCC::COND_LT;
  case MOSCC::COND_LT:
    return MOSCC::COND_GE;
  case MOSCC::COND_MI:
    return MOSCC::COND_PL;
  case MOSCC::COND_PL:
    return MOSCC::COND_MI;
  }
}

bool MOSInstrInfo::analyzeBranch(MachineBasicBlock &MBB,
                                 MachineBasicBlock *&TBB,
                                 MachineBasicBlock *&FBB,
                                 SmallVectorImpl<MachineOperand> &Cond,
                                 bool AllowModify) const {
  // Start from the bottom of the block and work up, examining the
  // terminator instructions.
  MachineBasicBlock::iterator I = MBB.end();
  MachineBasicBlock::iterator UnCondBrIter = MBB.end();

  while (I != MBB.begin()) {
    --I;
    if (I->isDebugInstr()) {
      continue;
    }

    // Working from the bottom, when we see a non-terminator
    // instruction, we're done.
    if (!isUnpredicatedTerminator(*I)) {
      break;
    }

    // A terminator that isn't a branch can't easily be handled
    // by this analysis.
    if (!I->getDesc().isBranch()) {
      return true;
    }

    // Handle unconditional branches.
    //:TODO: add here jmp
    if (I->getOpcode() == MOS::RJMPk) {
      UnCondBrIter = I;

      if (!AllowModify) {
        TBB = I->getOperand(0).getMBB();
        continue;
      }

      // If the block has any instructions after a JMP, delete them.
      while (std::next(I) != MBB.end()) {
        std::next(I)->eraseFromParent();
      }

      Cond.clear();
      FBB = 0;

      // Delete the JMP if it's equivalent to a fall-through.
      if (MBB.isLayoutSuccessor(I->getOperand(0).getMBB())) {
        TBB = 0;
        I->eraseFromParent();
        I = MBB.end();
        UnCondBrIter = MBB.end();
        continue;
      }

      // TBB is used to indicate the unconditinal destination.
      TBB = I->getOperand(0).getMBB();
      continue;
    }

    // Handle conditional branches.
    MOSCC::CondCodes BranchCode = getCondFromBranchOpc(I->getOpcode());
    if (BranchCode == MOSCC::COND_INVALID) {
      return true; // Can't handle indirect branch.
    }

    // Working from the bottom, handle the first conditional branch.
    if (Cond.empty()) {
      MachineBasicBlock *TargetBB = I->getOperand(0).getMBB();
      if (AllowModify && UnCondBrIter != MBB.end() &&
          MBB.isLayoutSuccessor(TargetBB)) {
        // If we can modify the code and it ends in something like:
        //
        //     jCC L1
        //     jmp L2
        //   L1:
        //     ...
        //   L2:
        //
        // Then we can change this to:
        //
        //     jnCC L2
        //   L1:
        //     ...
        //   L2:
        //
        // Which is a bit more efficient.
        // We conditionally jump to the fall-through block.
        BranchCode = getOppositeCondition(BranchCode);
        unsigned JNCC = getBrCond(BranchCode).getOpcode();
        MachineBasicBlock::iterator OldInst = I;

        BuildMI(MBB, UnCondBrIter, MBB.findDebugLoc(I), get(JNCC))
            .addMBB(UnCondBrIter->getOperand(0).getMBB());
        BuildMI(MBB, UnCondBrIter, MBB.findDebugLoc(I), get(MOS::RJMPk))
            .addMBB(TargetBB);

        OldInst->eraseFromParent();
        UnCondBrIter->eraseFromParent();

        // Restart the analysis.
        UnCondBrIter = MBB.end();
        I = MBB.end();
        continue;
      }

      FBB = TBB;
      TBB = I->getOperand(0).getMBB();
      Cond.push_back(MachineOperand::CreateImm(BranchCode));
      continue;
    }

    // Handle subsequent conditional branches. Only handle the case where all
    // conditional branches branch to the same destination.
    assert(Cond.size() == 1);
    assert(TBB);

    // Only handle the case where all conditional branches branch to
    // the same destination.
    if (TBB != I->getOperand(0).getMBB()) {
      return true;
    }

    MOSCC::CondCodes OldBranchCode = (MOSCC::CondCodes)Cond[0].getImm();
    // If the conditions are the same, we can leave them alone.
    if (OldBranchCode == BranchCode) {
      continue;
    }

    return true;
  }

  return false;
}

unsigned MOSInstrInfo::insertBranch(MachineBasicBlock &MBB,
                                    MachineBasicBlock *TBB,
                                    MachineBasicBlock *FBB,
                                    ArrayRef<MachineOperand> Cond,
                                    const DebugLoc &DL,
                                    int *BytesAdded) const {
  if (BytesAdded) *BytesAdded = 0;

  // Shouldn't be a fall through.
  assert(TBB && "insertBranch must not be told to insert a fallthrough");
  assert((Cond.size() == 1 || Cond.size() == 0) &&
         "MOS branch conditions have one component!");

  if (Cond.empty()) {
    assert(!FBB && "Unconditional branch with multiple successors!");
    auto &MI = *BuildMI(&MBB, DL, get(MOS::RJMPk)).addMBB(TBB);
    if (BytesAdded)
      *BytesAdded += getInstSizeInBytes(MI);
    return 1;
  }

  // Conditional branch.
  unsigned Count = 0;
  MOSCC::CondCodes CC = (MOSCC::CondCodes)Cond[0].getImm();
  auto &CondMI = *BuildMI(&MBB, DL, getBrCond(CC)).addMBB(TBB);

  if (BytesAdded) *BytesAdded += getInstSizeInBytes(CondMI);
  ++Count;

  if (FBB) {
    // Two-way Conditional branch. Insert the second branch.
    auto &MI = *BuildMI(&MBB, DL, get(MOS::RJMPk)).addMBB(FBB);
    if (BytesAdded) *BytesAdded += getInstSizeInBytes(MI);
    ++Count;
  }

  return Count;
}

unsigned MOSInstrInfo::removeBranch(MachineBasicBlock &MBB,
                                    int *BytesRemoved) const {
  if (BytesRemoved) *BytesRemoved = 0;

  MachineBasicBlock::iterator I = MBB.end();
  unsigned Count = 0;

  while (I != MBB.begin()) {
    --I;
    if (I->isDebugInstr()) {
      continue;
    }
    //:TODO: add here the missing jmp instructions once they are implemented
    // like jmp, {e}ijmp, and other cond branches, ...
    if (I->getOpcode() != MOS::RJMPk &&
        getCondFromBranchOpc(I->getOpcode()) == MOSCC::COND_INVALID) {
      break;
    }

    // Remove the branch.
    if (BytesRemoved) *BytesRemoved += getInstSizeInBytes(*I);
    I->eraseFromParent();
    I = MBB.end();
    ++Count;
  }

  return Count;
}

bool MOSInstrInfo::reverseBranchCondition(
    SmallVectorImpl<MachineOperand> &Cond) const {
  assert(Cond.size() == 1 && "Invalid MOS branch condition!");

  MOSCC::CondCodes CC = static_cast<MOSCC::CondCodes>(Cond[0].getImm());
  Cond[0].setImm(getOppositeCondition(CC));

  return false;
}

unsigned MOSInstrInfo::getInstSizeInBytes(const MachineInstr &MI) const {
  unsigned Opcode = MI.getOpcode();

  switch (Opcode) {
  // A regular instruction
  default: {
    const MCInstrDesc &Desc = get(Opcode);
    return Desc.getSize();
  }
  case TargetOpcode::EH_LABEL:
  case TargetOpcode::IMPLICIT_DEF:
  case TargetOpcode::KILL:
  case TargetOpcode::DBG_VALUE:
    return 0;
  case TargetOpcode::INLINEASM: {
    const MachineFunction &MF = *MI.getParent()->getParent();
    const MOSTargetMachine &TM = static_cast<const MOSTargetMachine&>(MF.getTarget());
    const MOSSubtarget &STI = MF.getSubtarget<MOSSubtarget>();
    const TargetInstrInfo &TII = *STI.getInstrInfo();

    return TII.getInlineAsmLength(MI.getOperand(0).getSymbolName(),
                                  *TM.getMCAsmInfo());
  }
  }
}

MachineBasicBlock *
MOSInstrInfo::getBranchDestBlock(const MachineInstr &MI) const {
  switch (MI.getOpcode()) {
  default:
    llvm_unreachable("unexpected opcode!");
  case MOS::JMPk:
  case MOS::CALLk:
  case MOS::RCALLk:
  case MOS::RJMPk:
  case MOS::BREQk:
  case MOS::BRNEk:
  case MOS::BRSHk:
  case MOS::BRLOk:
  case MOS::BRMIk:
  case MOS::BRPLk:
  case MOS::BRGEk:
  case MOS::BRLTk:
    return MI.getOperand(0).getMBB();
  case MOS::BRBSsk:
  case MOS::BRBCsk:
    return MI.getOperand(1).getMBB();
  case MOS::SBRCRrB:
  case MOS::SBRSRrB:
  case MOS::SBICAb:
  case MOS::SBISAb:
    llvm_unreachable("unimplemented branch instructions");
  }
}

bool MOSInstrInfo::isBranchOffsetInRange(unsigned BranchOp,
                                         int64_t BrOffset) const {

  switch (BranchOp) {
  default:
    llvm_unreachable("unexpected opcode!");
  case MOS::JMPk:
  case MOS::CALLk:
    return true;
  case MOS::RCALLk:
  case MOS::RJMPk:
    return isIntN(13, BrOffset);
  case MOS::BRBSsk:
  case MOS::BRBCsk:
  case MOS::BREQk:
  case MOS::BRNEk:
  case MOS::BRSHk:
  case MOS::BRLOk:
  case MOS::BRMIk:
  case MOS::BRPLk:
  case MOS::BRGEk:
  case MOS::BRLTk:
    return isIntN(7, BrOffset);
  }
}

unsigned MOSInstrInfo::insertIndirectBranch(MachineBasicBlock &MBB,
                                            MachineBasicBlock &NewDestBB,
                                            const DebugLoc &DL,
                                            int64_t BrOffset,
                                            RegScavenger *RS) const {
    // This method inserts a *direct* branch (JMP), despite its name.
    // LLVM calls this method to fixup unconditional branches; it never calls
    // insertBranch or some hypothetical "insertDirectBranch".
    // See lib/CodeGen/RegisterRelaxation.cpp for details.
    // We end up here when a jump is too long for a RJMP instruction.
    auto &MI = *BuildMI(&MBB, DL, get(MOS::JMPk)).addMBB(&NewDestBB);

    return getInstSizeInBytes(MI);
}

} // end of namespace llvm

