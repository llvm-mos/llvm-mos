//===-- MOSInstrInfo.cpp - MOS Instruction Information --------------------===//
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

#include "MOSInstrInfo.h"

#include "MCTargetDesc/MOSMCTargetDesc.h"
#include "MOSFrameLowering.h"
#include "MOSInstrBuilder.h"
#include "MOSRegisterInfo.h"

#include "MOSSubtarget.h"
#include "llvm/ADT/BitVector.h"
#include "llvm/ADT/SparseBitVector.h"
#include "llvm/CodeGen/GlobalISel/MachineIRBuilder.h"
#include "llvm/CodeGen/GlobalISel/Utils.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineMemOperand.h"
#include "llvm/CodeGen/MachineOperand.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/PseudoSourceValue.h"
#include "llvm/CodeGen/Register.h"
#include "llvm/CodeGen/TargetInstrInfo.h"
#include "llvm/CodeGen/TargetRegisterInfo.h"
#include "llvm/CodeGen/TargetSubtargetInfo.h"
#include "llvm/MC/MCAsmInfo.h"
#include "llvm/Support/Compiler.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Target/TargetMachine.h"

using namespace llvm;

#define DEBUG_TYPE "mos-instrinfo"

#define GET_INSTRINFO_CTOR_DTOR
#include "MOSGenInstrInfo.inc"

MOSInstrInfo::MOSInstrInfo()
    : MOSGenInstrInfo(/*CFSetupOpcode=*/MOS::ADJCALLSTACKDOWN,
                      /*CFDestroyOpcode=*/MOS::ADJCALLSTACKUP) {}

bool MOSInstrInfo::isReallyTriviallyReMaterializable(
    const MachineInstr &MI) const {
  switch (MI.getOpcode()) {
  default:
    return TargetInstrInfo::isReallyTriviallyReMaterializable(MI);
  case MOS::LDImm16:
    return true;
  }
}

Register MOSInstrInfo::isLoadFromStackSlot(const MachineInstr &MI,
                                           int &FrameIndex) const {
  switch (MI.getOpcode()) {
  default:
    return 0;
  case MOS::LDAbs:
    if (!MI.getOperand(0).isFI())
      return 0;
    FrameIndex = MI.getOperand(1).getIndex();
    return MI.getOperand(0).getReg();
  case MOS::LDStk:
    FrameIndex = MI.getOperand(2).getIndex();
    return MI.getOperand(0).getReg();
  }
}

Register MOSInstrInfo::isStoreToStackSlot(const MachineInstr &MI,
                                          int &FrameIndex) const {
  switch (MI.getOpcode()) {
  default:
    return 0;
  case MOS::STAbs:
    if (!MI.getOperand(0).isFI())
      return 0;
    FrameIndex = MI.getOperand(1).getIndex();
    return MI.getOperand(0).getReg();
  case MOS::STStk:
    FrameIndex = MI.getOperand(2).getIndex();
    return MI.getOperand(1).getReg();
  }
}

void MOSInstrInfo::reMaterialize(MachineBasicBlock &MBB,
                                 MachineBasicBlock::iterator I,
                                 Register DestReg, unsigned SubIdx,
                                 const MachineInstr &Orig,
                                 const TargetRegisterInfo &TRI) const {
  if (Orig.getOpcode() == MOS::LDImm16) {
    MachineInstr *MI = MBB.getParent()->CloneMachineInstr(&Orig);
    MI->removeOperand(1);
    MI->substituteRegister(MI->getOperand(0).getReg(), DestReg, SubIdx, TRI);
    MI->setDesc(get(MOS::LDImm16Remat));
    MBB.insert(I, MI);
  } else {
    TargetInstrInfo::reMaterialize(MBB, I, DestReg, SubIdx, Orig, TRI);
  }
}

// The main difficulty in commuting 6502 instructions is that their register
// classes aren't symmetric. This routine determines whether or not the operands
// of an instruction can be commuted anyway, potentially rewriting the register
// classes of virtual registers to do so.
MachineInstr *MOSInstrInfo::commuteInstructionImpl(MachineInstr &MI, bool NewMI,
                                                   unsigned Idx1,
                                                   unsigned Idx2) const {
  // NOTE: This doesn't seem to actually be used anywhere.
  if (NewMI)
    report_fatal_error("NewMI is not supported");

  MachineFunction &MF = *MI.getMF();
  const TargetRegisterInfo &TRI = *MF.getSubtarget().getRegisterInfo();
  MachineRegisterInfo &MRI = MF.getRegInfo();

  LLVM_DEBUG(dbgs() << "Commute: " << MI);

  // Determines the register class for a given virtual register constrained by a
  // target register class and all uses outside this instruction. This
  // effectively removes the constraints due to just this instruction, then
  // tries to apply the constraint for the other operand.
  const auto NewRegClass =
      [&](Register Reg,
          const TargetRegisterClass *RC) -> const TargetRegisterClass * {
    for (MachineOperand &MO : MRI.reg_nodbg_operands(Reg)) {
      MachineInstr *UseMI = MO.getParent();
      if (UseMI == &MI)
        continue;
      unsigned OpNo = &MO - &UseMI->getOperand(0);
      RC = UseMI->getRegClassConstraintEffect(OpNo, RC, this, &TRI);
      if (!RC)
        return nullptr;
    }
    return RC;
  };

  const TargetRegisterClass *RegClass1 =
      getRegClass(MI.getDesc(), Idx1, &TRI, MF);
  const TargetRegisterClass *RegClass2 =
      getRegClass(MI.getDesc(), Idx2, &TRI, MF);
  Register Reg1 = MI.getOperand(Idx1).getReg();
  Register Reg2 = MI.getOperand(Idx2).getReg();

  // See if swapping the two operands are possible given their register classes.
  const TargetRegisterClass *Reg1Class = nullptr;
  const TargetRegisterClass *Reg2Class = nullptr;
  if (Reg1.isVirtual()) {
    Reg1Class = NewRegClass(Reg1, RegClass2);
    if (!Reg1Class)
      return nullptr;
  }
  if (Reg1.isPhysical() && !RegClass2->contains(Reg1))
    return nullptr;
  if (Reg2.isVirtual()) {
    Reg2Class = NewRegClass(Reg2, RegClass1);
    if (!Reg2Class)
      return nullptr;
  }
  if (Reg2.isPhysical() && !RegClass1->contains(Reg2))
    return nullptr;

  // If this fails, make sure to get it out of the way before rewriting reg
  // classes.
  MachineInstr *CommutedMI =
      TargetInstrInfo::commuteInstructionImpl(MI, NewMI, Idx1, Idx2);
  if (!CommutedMI)
    return nullptr;

  // PHI nodes keep the register classes of all their arguments. By the time the
  // two address instruction pass occurs, these phis have already been lowered
  // to copies. Changing register classes here can make those register classes
  // mismatch the new ones; to avoid this, we recompute the register classes for
  // any vregs copied into or out of a commuted vreg.
  const auto RecomputeCopyRC = [&](Register Reg) {
    for (MachineInstr &MI : MRI.reg_nodbg_instructions(Reg)) {
      if (!MI.isCopy())
        continue;
      Register Other = MI.getOperand(0).getReg() == Reg
                           ? MI.getOperand(1).getReg()
                           : MI.getOperand(0).getReg();
      if (!Other.isVirtual())
        continue;
      MRI.recomputeRegClass(Other);
    }
  };

  // Use the new register classes computed above, if any.
  if (Reg1Class) {
    MRI.setRegClass(Reg1, Reg1Class);
    RecomputeCopyRC(Reg1);
  }
  if (Reg2Class) {
    MRI.setRegClass(Reg2, Reg2Class);
    RecomputeCopyRC(Reg2);
  }
  return CommutedMI;
}

unsigned MOSInstrInfo::getInstSizeInBytes(const MachineInstr &MI) const {
  if (MI.isDebugInstr())
    return 0;

  const MachineFunction &MF = *MI.getParent()->getParent();
  const MCAsmInfo &MCAI = *MF.getTarget().getMCAsmInfo();
  const TargetSubtargetInfo &STI = MF.getSubtarget();

  switch (MI.getOpcode()) {
  default: {
    unsigned Size = get(MI.getOpcode()).getSize();
    if (!Size)
      Size = MCAI.getMaxInstLength(&STI);
    return Size;
  }
  case MOS::INLINEASM:
  case MOS::INLINEASM_BR:
    return getInlineAsmLength(MI.getOperand(0).getSymbolName(), MCAI, &STI);
  }
}

// 6502 instructions aren't as regular as most commutable instructions, so this
// routine determines the commutable operands manually.
bool MOSInstrInfo::findCommutedOpIndices(const MachineInstr &MI,
                                         unsigned &SrcOpIdx1,
                                         unsigned &SrcOpIdx2) const {
  assert(!MI.isBundle() &&
         "MOSInstrInfo::findCommutedOpIndices() can't handle bundles");

  const MCInstrDesc &MCID = MI.getDesc();
  if (!MCID.isCommutable())
    return false;

  unsigned CommutableOpIdx1, CommutableOpIdx2;
  switch (MI.getOpcode()) {
  default:
    llvm_unreachable("Unexpected opcode; don't know how to commute.");
  case MOS::ADCImag8:
    CommutableOpIdx1 = 3;
    CommutableOpIdx2 = 4;
    break;
  case MOS::ANDImag8:
  case MOS::EORImag8:
  case MOS::ORAImag8:
    CommutableOpIdx1 = 1;
    CommutableOpIdx2 = 2;
    break;
  }

  if (!fixCommutedOpIndices(SrcOpIdx1, SrcOpIdx2, CommutableOpIdx1,
                            CommutableOpIdx2))
    return false;

  if (!MI.getOperand(SrcOpIdx1).isReg() || !MI.getOperand(SrcOpIdx2).isReg()) {
    // No idea.
    return false;
  }
  return true;
}

bool MOSInstrInfo::hasCommutePreference(MachineInstr &MI, bool &Commute) const {
  unsigned CommutableOpIdx1 = CommuteAnyOperandIndex;
  unsigned CommutableOpIdx2 = CommuteAnyOperandIndex;
  if (!findCommutedOpIndices(MI, CommutableOpIdx1, CommutableOpIdx2)) {
    return false;
  }

  MachineFunction &MF = *MI.getMF();
  MachineRegisterInfo &MRI = MF.getRegInfo();

  // Detect trivial copies of the form:
  //
  // %0:ac = COPY $a
  // %1:imag8 = COPY $x
  // ...
  // %3:ac, %4:cc, %7:vc = ADCImag8 %0:ac(tied-def 0), %1:imag8, %6:cc
  //
  // Avoid or prefer commuting based on target register classes.
  // This tries to ensure that relevant copies remain trivial.

  auto Reg1 = MI.getOperand(CommutableOpIdx1).getReg();
  auto Reg2 = MI.getOperand(CommutableOpIdx2).getReg();
  if (!Reg1.isVirtual() || !Reg2.isVirtual())
    return false;
  if (!MRI.hasOneDef(Reg1) || !MRI.hasOneDef(Reg2))
    return false;

  MachineInstr *Instr1 = MRI.getOneDef(Reg1)->getParent();
  MachineInstr *Instr2 = MRI.getOneDef(Reg2)->getParent();
  if (!Instr1 || Instr1->getOpcode() != MOS::COPY ||
      !Instr1->getOperand(1).isReg() ||
      !Instr1->getOperand(1).getReg().isPhysical())
    return false;
  if (!Instr2 || Instr2->getOpcode() != MOS::COPY ||
      !Instr2->getOperand(1).isReg() ||
      !Instr2->getOperand(1).getReg().isPhysical())
    return false;

  auto SrcReg1 = Instr1->getOperand(1).getReg();
  auto SrcReg2 = Instr2->getOperand(1).getReg();
  auto *DstRC1 = MRI.getRegClassOrNull(Reg1);
  auto *DstRC2 = MRI.getRegClassOrNull(Reg2);
  if (!DstRC1 || !DstRC2)
    return false;

  if (DstRC1->contains(SrcReg1) && DstRC2->contains(SrcReg2))
    Commute = false;
  else if (DstRC2->contains(SrcReg1) && DstRC1->contains(SrcReg2))
    Commute = true;
  else if (DstRC1->contains(SrcReg1) || DstRC2->contains(SrcReg2))
    Commute = false;
  else if (DstRC2->contains(SrcReg1) || DstRC1->contains(SrcReg2))
    Commute = true;
  else
    return false;

  return true;
}

bool MOSInstrInfo::isBranchOffsetInRange(unsigned BranchOpc,
                                         int64_t BrOffset) const {
  switch (BranchOpc) {
  default:
    llvm_unreachable("Bad branch opcode");
  case MOS::GBR:
  case MOS::BR:
  case MOS::BRA:
    // BR range is [-128,127] starting from the PC location after the
    // instruction, which is two bytes after the start of the instruction.
    return -126 <= BrOffset && BrOffset <= 129;
  case MOS::JMP:
    return true;
  }
}

MachineBasicBlock *
MOSInstrInfo::getBranchDestBlock(const MachineInstr &MI) const {
  switch (MI.getOpcode()) {
  default:
    llvm_unreachable("Bad branch opcode");
  case MOS::GBR:
  case MOS::BR:
  case MOS::BRA:
  case MOS::JMP:
  case MOS::CmpBrImm:
  case MOS::CmpBrImag8:
  case MOS::CmpBrZero:
  case MOS::CmpBrZeroMultiByte:
  case MOS::CmpBrZpIdx:
  case MOS::CmpBrAbs:
  case MOS::CmpBrAbsIdx:
  case MOS::CmpBrIndir:
  case MOS::CmpBrIndirIdx:
    return MI.getOperand(0).getMBB();
  case MOS::JMPIndir:
  case MOS::JMPIdxIndir:
    return nullptr;
  }
}

bool MOSInstrInfo::analyzeBranch(MachineBasicBlock &MBB,
                                 MachineBasicBlock *&TBB,
                                 MachineBasicBlock *&FBB,
                                 SmallVectorImpl<MachineOperand> &Cond,
                                 bool AllowModify) const {
  auto I = MBB.getFirstTerminator();

  // If no terminators, falls through.
  if (I == MBB.end()) {
    TBB = nullptr;
    FBB = nullptr;
    Cond.clear();
    return false;
  }

  // Non-branch terminators cannot be analyzed.
  if (!I->isBranch())
    return true;

  // Analyze first branch.
  auto FirstBR = I++;
  if (FirstBR->isPreISelOpcode())
    return true;
  // First branch always forms true edge, whether conditional or unconditional.
  TBB = getBranchDestBlock(*FirstBR);
  if (!TBB)
    return true;
  if (FirstBR->isConditionalBranch()) {
    if (!FirstBR->memoperands_empty()) {
      assert(FirstBR->hasOneMemOperand() &&
             "CmpBr should have at most one mem operand");
      // Don't futz with volatile compare and branches; the compare part has to
      // happen, and we can't lose the MMO that says the compare is volatile.
      if ((*FirstBR->memoperands_begin())->isVolatile())
        return true;
    }

    Cond.clear();
    Cond.push_back(MachineOperand::CreateImm(FirstBR->getOpcode()));
    // Push all arguments except the branch destination; that's not part of the
    // condition.
    for (unsigned I = 1, E = FirstBR->getNumExplicitOperands(); I != E; ++I)
      Cond.push_back(FirstBR->getOperand(I));
  }

  // If there's no second branch, done.
  if (I == MBB.end()) {
    FBB = nullptr;
    return false;
  }

  // Cannot analyze branch followed by non-branch.
  if (!I->isBranch())
    return true;

  auto SecondBR = I++;

  // If any instructions follow the second branch, cannot analyze.
  if (I != MBB.end())
    return true;

  // Exactly two branches present.

  // Can only analyze conditional branch followed by unconditional branch.
  if (!SecondBR->isUnconditionalBranch() || SecondBR->isPreISelOpcode())
    return true;

  // Second unconditional branch forms false edge.
  FBB = getBranchDestBlock(*SecondBR);
  if (!FBB)
    return true;
  return false;
}

unsigned MOSInstrInfo::removeBranch(MachineBasicBlock &MBB,
                                    int *BytesRemoved) const {
  auto Begin = MBB.getFirstTerminator();
  auto End = MBB.end();

  unsigned NumRemoved = std::distance(Begin, End);
  if (BytesRemoved) {
    *BytesRemoved = 0;
    for (const auto &I : make_range(Begin, End))
      *BytesRemoved += getInstSizeInBytes(I);
  }
  MBB.erase(Begin, End);
  return NumRemoved;
}

unsigned MOSInstrInfo::insertBranch(MachineBasicBlock &MBB,
                                    MachineBasicBlock *TBB,
                                    MachineBasicBlock *FBB,
                                    ArrayRef<MachineOperand> Cond,
                                    const DebugLoc &DL, int *BytesAdded) const {
  MachineFunction &MF = *MBB.getParent();
  const MOSSubtarget &STI = MF.getSubtarget<MOSSubtarget>();

  MachineIRBuilder Builder(MBB, MBB.end());
  unsigned NumAdded = 0;
  if (BytesAdded)
    *BytesAdded = 0;

  // Unconditional branch target.
  auto *UBB = TBB;

  // Conditional branch.
  if (!Cond.empty()) {
    assert(TBB);

    // The unconditional branch will be to the false branch (if any).
    UBB = FBB;

    // Add conditional branch.
    auto BR = Builder.buildInstr(Cond.front().getImm()).addMBB(TBB);
    for (const MachineOperand &Op : Cond.drop_front())
      BR.add(Op);

    // Add a fictitious MMO if necessary.
    if (BR->mayLoad())
      BR->addMemOperand(MF, MF.getMachineMemOperand(MachinePointerInfo{},
                                                    MachineMemOperand::MOLoad,
                                                    LLT::scalar(8), Align{}));

    ++NumAdded;
    if (BytesAdded)
      *BytesAdded += getInstSizeInBytes(*BR);
  }

  // Add unconditional branch if necessary.
  if (UBB) {
    // For 65C02/65DTV02, assume BRA and relax into JMP in
    // insertIndirectBranch if necessary.
    auto JMP =
        Builder.buildInstr(STI.hasBRA() ? MOS::BRA : MOS::JMP).addMBB(UBB);
    ++NumAdded;
    if (BytesAdded)
      *BytesAdded += getInstSizeInBytes(*JMP);
  }

  return NumAdded;
}

void MOSInstrInfo::insertIndirectBranch(MachineBasicBlock &MBB,
                                        MachineBasicBlock &NewDestBB,
                                        MachineBasicBlock &RestoreBB,
                                        const DebugLoc &DL, int64_t BrOffset,
                                        RegScavenger *RS) const {
  // This method inserts a *direct* branch (JMP), despite its name.
  // LLVM calls this method to fixup unconditional branches; it never calls
  // insertBranch or some hypothetical "insertDirectBranch".
  // See lib/CodeGen/BranchRelaxation.cpp for details.
  // We end up here when a jump is too long for a BRA instruction.

  MachineIRBuilder Builder(MBB, MBB.end());
  Builder.setDebugLoc(DL);

  Builder.buildInstr(MOS::JMP).addMBB(&NewDestBB);
}

void MOSInstrInfo::copyPhysReg(MachineBasicBlock &MBB,
                               MachineBasicBlock::iterator MI,
                               const DebugLoc &DL, MCRegister DestReg,
                               MCRegister SrcReg, bool KillSrc,
                               bool RenamableDest, bool RenamableSrc) const {
  MachineIRBuilder Builder(MBB, MI);
  Builder.setDebugLoc(DL);
  copyPhysRegImpl(Builder, DestReg, SrcReg, false, KillSrc);
}

static Register createVReg(MachineIRBuilder &Builder,
                           const TargetRegisterClass &RC) {
  Builder.getMF().getProperties().reset(
      MachineFunctionProperties::Property::NoVRegs);
  return Builder.getMRI()->createVirtualRegister(&RC);
}

bool MOSInstrInfo::shouldOverlapInterval(const MachineInstr &MI) const {
  return MI.getOpcode() != MOS::CmpBrZero;
}

static bool isTargetCopy(MachineInstr &MI) {
  switch (MI.getOpcode()) {
  case MOS::LDImag8:
  case MOS::MOVImag8:
  case MOS::STImag8:
  case MOS::TA:
  case MOS::T_A:
  case MOS::TX:
    return true;
  default:
    return false;
  }
}

static bool isCopyRedundant(MachineIRBuilder &Builder, Register Dst,
                            Register Src) {
  if (Dst == Src)
    return true;
  const TargetRegisterInfo *TRI = Builder.getMRI()->getTargetRegisterInfo();
  MachineInstr *DstKillMI = nullptr;
  for (MachineInstr &MI :
       make_range(MachineBasicBlock::reverse_iterator(Builder.getInsertPt()),
                  Builder.getMBB().rend())) {
    if (MI.killsRegister(Dst, TRI))
      DstKillMI = &MI;
    if (isTargetCopy(MI)) {
      Register CopyDst = MI.getOperand(0).getReg();
      Register CopySrc = MI.getOperand(1).getReg();
      if ((CopyDst == Dst && CopySrc == Src) ||
          (CopyDst == Src && CopySrc == Dst)) {
        MI.clearRegisterDeads(Dst);
        if (DstKillMI)
          DstKillMI->clearRegisterKills(Dst, TRI);
        return true;
      }
    }
    if (MI.modifiesRegister(Dst, TRI))
      return false;
    if (MI.modifiesRegister(Src, TRI))
      return false;
  }
  return false;
}

bool MOSInstrInfo::hasCustomTiedOperands(unsigned Opcode) const {
  return Opcode == MOS::IncMB || Opcode == MOS::DecMB ||
         Opcode == MOS::DecDcpMB;
}

unsigned MOSInstrInfo::findCustomTiedOperandIdx(const MachineInstr &MI,
                                                unsigned OpIdx) const {
  assert(hasCustomTiedOperands(MI.getOpcode()));
  if (OpIdx < MI.getNumExplicitDefs())
    return MI.getOpcode() == MOS::IncMB ? OpIdx + MI.getNumExplicitDefs()
                                        : OpIdx + MI.getNumExplicitDefs() - 1;
  return MI.getOpcode() == MOS::IncMB ? OpIdx - MI.getNumExplicitDefs()
                                      : OpIdx - MI.getNumExplicitDefs() + 1;
}

void MOSInstrInfo::copyPhysRegImpl(MachineIRBuilder &Builder, Register DestReg,
                                   Register SrcReg, bool Force,
                                   bool KillSrc) const {
  if (!Force && isCopyRedundant(Builder, DestReg, SrcReg))
    return;

  const MOSSubtarget &STI = Builder.getMF().getSubtarget<MOSSubtarget>();
  const TargetRegisterInfo &TRI = *STI.getRegisterInfo();

  const auto &IsClass = [&](Register Reg, const TargetRegisterClass &RC) {
    if (Reg.isPhysical() && !RC.contains(Reg))
      return false;
    if (Reg.isVirtual() &&
        !Builder.getMRI()->getRegClass(Reg)->hasSuperClassEq(&RC))
      return false;
    return true;
  };

  const auto &AreClasses = [&](const TargetRegisterClass &Dest,
                               const TargetRegisterClass &Src) {
    return IsClass(DestReg, Dest) && IsClass(SrcReg, Src);
  };

  if (AreClasses(MOS::GPRRegClass, MOS::GPRRegClass)) {
    if (IsClass(SrcReg, MOS::AcRegClass)) {
      assert(MOS::XYRegClass.contains(DestReg));
      Builder.buildInstr(MOS::TA).addDef(DestReg).addUse(SrcReg);
    } else if (IsClass(DestReg, MOS::AcRegClass)) {
      assert(MOS::XYRegClass.contains(SrcReg));
      Builder.buildInstr(MOS::T_A).addDef(DestReg).addUse(SrcReg);
    } else if (STI.hasW65816Or65EL02()) {
      assert(MOS::XYRegClass.contains(SrcReg));
      assert(MOS::XYRegClass.contains(DestReg));
      Builder.buildInstr(MOS::TX).addDef(DestReg).addUse(SrcReg);
    } else if (STI.hasHUC6280() && KillSrc) {
      // The HuC6280 does not have an X->Y or Y->X transfer function, but if
      // the source register is being killed, it can be modeled using a swap.
      assert(MOS::XYRegClass.contains(SrcReg));
      assert(MOS::XYRegClass.contains(DestReg));
      // The Dst (=> Src) value is not relevant to modeling a copy with a swap.
      Builder.buildInstr(MOS::SWAP)
          .addDef(DestReg)
          .addDef(SrcReg)
          .addUse(SrcReg, RegState::Kill)
          .addUse(DestReg, RegState::Kill | RegState::Undef);
    } else if (STI.hasGPRStackRegs()) {
      // The 65C02 can emit a PHX/PLY or PHY/PLX pair.
      assert(MOS::XYRegClass.contains(SrcReg));
      assert(MOS::XYRegClass.contains(DestReg));
      Builder.buildInstr(MOS::PH, {}, {SrcReg});
      auto I = Builder.buildInstr(MOS::PL, {DestReg}, {});
      if (!STI.hasSPC700())
        I.addDef(MOS::NZ, RegState::Implicit);
    } else {
      copyPhysRegImpl(Builder, DestReg,
                      getRegWithVal(Builder, SrcReg, MOS::AcRegClass));
    }
  } else if (AreClasses(MOS::Imag8RegClass, MOS::GPRRegClass)) {
    Builder.buildInstr(MOS::STImag8).addDef(DestReg).addUse(SrcReg);
  } else if (AreClasses(MOS::GPRRegClass, MOS::Imag8RegClass)) {
    Builder.buildInstr(MOS::LDImag8).addDef(DestReg).addUse(SrcReg);
  } else if (AreClasses(MOS::Imag8RegClass, MOS::Imag8RegClass)) {
    if (STI.hasSPC700()) {
      Builder.buildInstr(MOS::MOVImag8).addDef(DestReg).addUse(SrcReg);
    } else {
      copyPhysRegImpl(Builder, DestReg,
                      getRegWithVal(Builder, SrcReg, MOS::GPRRegClass));
    }
  } else if (AreClasses(MOS::Imag16RegClass, MOS::Imag16RegClass)) {
    assert(SrcReg.isPhysical() && DestReg.isPhysical());
    copyPhysRegImpl(Builder, TRI.getSubReg(DestReg, MOS::sublo),
                    TRI.getSubReg(SrcReg, MOS::sublo));
    copyPhysRegImpl(Builder, TRI.getSubReg(DestReg, MOS::subhi),
                    TRI.getSubReg(SrcReg, MOS::subhi));
  } else if (AreClasses(MOS::Anyi1RegClass, MOS::Anyi1RegClass)) {
    assert(SrcReg.isPhysical() && DestReg.isPhysical());
    Register SrcReg8 =
        TRI.getMatchingSuperReg(SrcReg, MOS::sublsb, &MOS::Anyi8RegClass);
    Register DestReg8 =
        TRI.getMatchingSuperReg(DestReg, MOS::sublsb, &MOS::Anyi8RegClass);

    if (SrcReg8) {
      SrcReg = SrcReg8;
      if (DestReg8) {
        DestReg = DestReg8;
        // MOS defines LSB writes to write the whole 8-bit register, not just
        // part of it.
        assert(!Builder.getInsertPt()->readsRegister(DestReg, /*TRI=*/nullptr));

        copyPhysRegImpl(Builder, DestReg, SrcReg);
      } else {
        if (DestReg == MOS::C) {
          // C = SrcReg >= 1
          Builder.buildInstr(
              MOS::CMPImm, {MOS::C},
              {getRegWithVal(Builder, SrcReg, MOS::GPRRegClass), INT64_C(1)});
        } else {
          assert(DestReg == MOS::V);

          if (MOS::AcRegClass.contains(SrcReg)) {
            // ORA #0 defines NZ without impacting other flags or the register.
            Builder.buildInstr(MOS::ORAImm, {SrcReg}, {SrcReg, INT64_C(0)})
                .addDef(MOS::NZ, RegState::Implicit);
            Builder.buildInstr(MOS::SelectImm, {MOS::V},
                               {Register(MOS::Z), INT64_C(0), INT64_C(-1)});
          } else if (MOS::XYRegClass.contains(SrcReg)) {
            // A DEC/INC pair defines NZ without impacting other flags or
            // the register.
            Builder.buildInstr(MOS::DEC, {SrcReg}, {SrcReg});
            Builder.buildInstr(MOS::INC, {SrcReg}, {SrcReg})
                .addDef(MOS::NZ, RegState::Implicit);
            Builder.buildInstr(MOS::SelectImm, {MOS::V},
                               {Register(MOS::Z), INT64_C(0), INT64_C(-1)});
          } else {
            Register Tmp = createVReg(Builder, MOS::GPRRegClass);
            copyPhysRegImpl(Builder, Tmp, SrcReg, /*Force=*/true);
            std::prev(Builder.getInsertPt())
                ->addOperand(MachineOperand::CreateReg(MOS::NZ,
                                                       /*isDef=*/true,
                                                       /*isImp=*/true));
            // Add an implicit use of the vreg; otherwise, the register
            // scavenger may try to insert a reload between the load and the
            // select.
            auto Select =
                Builder.buildInstr(MOS::SelectImm, {MOS::V},
                                   {Register(MOS::Z), INT64_C(0), INT64_C(-1)});
            Select.addUse(Tmp, RegState::Implicit);
          }
        }
      }
    } else {
      if (DestReg8) {
        DestReg = DestReg8;

        Register Tmp = DestReg;
        if (!MOS::GPRRegClass.contains(Tmp))
          Tmp = createVReg(Builder, MOS::GPRRegClass);
        Builder.buildInstr(MOS::SelectImm, {Tmp},
                           {SrcReg, INT64_C(1), INT64_C(0)});
        if (Tmp != DestReg)
          copyPhysRegImpl(Builder, DestReg, Tmp);
      } else {
        Builder.buildInstr(MOS::SelectImm, {DestReg},
                           {SrcReg, INT64_C(-1), INT64_C(0)});
      }
    }
  } else
    llvm_unreachable("Unexpected physical register copy.");
}

// Get the value of the given physical register into a location of the given
// register class. This will search for an ealier instance of this value to
// use, starting from the insertion point of the given builder. If none is
// found, creates a virtual register and copies in the value.
Register MOSInstrInfo::getRegWithVal(MachineIRBuilder &Builder, Register Val,
                                     const TargetRegisterClass &RC) const {
  if (RC.contains(Val))
    return Val;

  const TargetRegisterInfo *TRI = Builder.getMRI()->getTargetRegisterInfo();

  // Whether the register still holds the value it does at the Builder
  // instruction.
  DenseMap<Register, bool> Clobbered;

  for (MachineInstr &MI :
       make_range(MachineBasicBlock::reverse_iterator(Builder.getInsertPt()),
                  Builder.getMBB().rend())) {
    for (Register R : RC)
      if (MI.modifiesRegister(R, TRI))
        Clobbered[R] = true;

    // At this point, all previous copies will already have been lowered.
    if (!isTargetCopy(MI)) {
      if (MI.modifiesRegister(Val, TRI))
        goto none;
      continue;
    }

    Register Dst = MI.getOperand(0).getReg();
    Register Src = MI.getOperand(1).getReg();

    if (Dst == Val) {
      if (RC.contains(Src) && !Clobbered[Src])
        return Src;
      break;
    }
    if (Src == Val && RC.contains(Dst) && !Clobbered[Dst])
      return Dst;
  }

none:
  Register R = createVReg(Builder, RC);
  copyPhysRegImpl(Builder, R, Val);
  return R;
}

const TargetRegisterClass *MOSInstrInfo::canFoldCopy(const MachineInstr &MI,
                                                     const TargetInstrInfo &TII,
                                                     unsigned FoldIdx) const {
  const MachineFunction &MF = *MI.getMF();
  const MOSFrameLowering &TFL =
      *MF.getSubtarget<MOSSubtarget>().getFrameLowering();
  if (!TFL.usesStaticStack(MF))
    return TargetInstrInfo::canFoldCopy(MI, TII, FoldIdx);

  Register FoldReg = MI.getOperand(FoldIdx).getReg();
  if (MOS::GPRRegClass.contains(FoldReg) ||
      MOS::GPR_LSBRegClass.contains(FoldReg))
    return TargetInstrInfo::canFoldCopy(MI, TII, FoldIdx);
  if (FoldReg.isVirtual()) {
    const auto *RC = MI.getMF()->getRegInfo().getRegClass(FoldReg);
    if (RC == &MOS::GPRRegClass || RC == &MOS::GPR_LSBRegClass)
      return TargetInstrInfo::canFoldCopy(MI, TII, FoldIdx);
  }
  return nullptr;
}

void MOSInstrInfo::storeRegToStackSlot(
    MachineBasicBlock &MBB, MachineBasicBlock::iterator MI, Register SrcReg,
    bool isKill, int FrameIndex, const TargetRegisterClass *RC,
    const TargetRegisterInfo *TRI, Register VReg) const {
  loadStoreRegStackSlot(MBB, MI, SrcReg, isKill, FrameIndex, RC, TRI,
                        /*IsLoad=*/false);
}

void MOSInstrInfo::loadRegFromStackSlot(MachineBasicBlock &MBB,
                                        MachineBasicBlock::iterator MI,
                                        Register DestReg, int FrameIndex,
                                        const TargetRegisterClass *RC,
                                        const TargetRegisterInfo *TRI,
                                        Register VReg) const {
  loadStoreRegStackSlot(MBB, MI, DestReg, false, FrameIndex, RC, TRI,
                        /*IsLoad=*/true);
}

// Load or store one byte from/to a location on the static stack.
static void loadStoreByteStaticStackSlot(MachineIRBuilder &Builder,
                                         MachineOperand MO, int FrameIndex,
                                         int64_t Offset,
                                         MachineMemOperand *MMO) {
  const MachineRegisterInfo &MRI = *Builder.getMRI();
  const TargetRegisterInfo &TRI =
      *Builder.getMF().getSubtarget().getRegisterInfo();

  Register Reg = MO.getReg();

  // Convert bit to byte if directly possible.
  if (Reg.isPhysical() && MOS::GPR_LSBRegClass.contains(Reg)) {
    Reg = TRI.getMatchingSuperReg(Reg, MOS::sublsb, &MOS::GPRRegClass);
    MO.setReg(Reg);
  } else if (Reg.isVirtual() &&
             MRI.getRegClass(Reg)->hasSuperClassEq(&MOS::GPRRegClass) &&
             MO.getSubReg() == MOS::sublsb) {
    MO.setSubReg(0);
  }

  // Emit directly through GPR if possible.
  if ((Reg.isPhysical() && MOS::GPRRegClass.contains(Reg)) ||
      (Reg.isVirtual() &&
       MRI.getRegClass(Reg)->hasSuperClassEq(&MOS::GPRRegClass) &&
       !MO.getSubReg())) {
    Builder.buildInstr(MO.isDef() ? MOS::LDAbs : MOS::STAbs)
        .add(MO)
        .addFrameIndex(FrameIndex, Offset)
        .addMemOperand(MMO);
    return;
  }

  // Emit via copy through GPR.
  bool IsBit = (Reg.isPhysical() && MOS::Anyi1RegClass.contains(Reg)) ||
               (Reg.isVirtual() &&
                (MRI.getRegClass(Reg)->hasSuperClassEq(&MOS::Anyi1RegClass) ||
                 MO.getSubReg() == MOS::sublsb));
  MachineOperand Tmp = MachineOperand::CreateReg(
      Builder.getMRI()->createVirtualRegister(&MOS::GPRRegClass), MO.isDef());
  if (Tmp.isUse()) {
    // Define the temporary register via copy from the MO.
    MachineOperand TmpDef = Tmp;
    TmpDef.setIsDef();
    if (IsBit) {
      TmpDef.setSubReg(MOS::sublsb);
      TmpDef.setIsUndef();
    }
    Builder.buildInstr(MOS::COPY).add(TmpDef).add(MO);

    loadStoreByteStaticStackSlot(Builder, Tmp, FrameIndex, Offset, MMO);
  } else {
    assert(Tmp.isDef());

    loadStoreByteStaticStackSlot(Builder, Tmp, FrameIndex, Offset, MMO);

    // Define the MO via copy from the temporary register.
    MachineOperand TmpUse = Tmp;
    TmpUse.setIsUse();
    if (IsBit)
      TmpUse.setSubReg(MOS::sublsb);
    Builder.buildInstr(MOS::COPY).add(MO).add(TmpUse);
  }
}

void MOSInstrInfo::loadStoreRegStackSlot(
    MachineBasicBlock &MBB, MachineBasicBlock::iterator MI, Register Reg,
    bool IsKill, int FrameIndex, const TargetRegisterClass *RC,
    const TargetRegisterInfo *TRI, bool IsLoad) const {
  MachineFunction &MF = *MBB.getParent();
  MachineFrameInfo &MFI = MF.getFrameInfo();
  MachineRegisterInfo &MRI = MF.getRegInfo();
  const MOSFrameLowering &TFL =
      *MF.getSubtarget<MOSSubtarget>().getFrameLowering();

  MachinePointerInfo PtrInfo =
      MachinePointerInfo::getFixedStack(MF, FrameIndex);
  MachineMemOperand *MMO = MF.getMachineMemOperand(
      PtrInfo, IsLoad ? MachineMemOperand::MOLoad : MachineMemOperand::MOStore,
      MFI.getObjectSize(FrameIndex), MFI.getObjectAlign(FrameIndex));

  MachineIRBuilder Builder(MBB, MI);
  MachineInstrSpan MIS(MI, &MBB);

  // If we're using the soft stack, since the offset is not yet known, it may
  // be either 8 or 16 bits. Emit a 16-bit pseudo to be lowered during frame
  // index elimination.
  if (!TFL.usesStaticStack(MF)) {
    Register Ptr = MRI.createVirtualRegister(&MOS::Imag16RegClass);
    auto Instr = Builder.buildInstr(IsLoad ? MOS::LDStk : MOS::STStk);
    if (!IsLoad)
      Instr.addDef(Ptr, RegState::EarlyClobber);
    Instr.addReg(Reg, getDefRegState(IsLoad) | getKillRegState(IsKill));
    if (IsLoad)
      Instr.addDef(Ptr, RegState::EarlyClobber);
    Instr.addFrameIndex(FrameIndex).addImm(0).addMemOperand(MMO);
  } else {
    if ((Reg.isPhysical() && MOS::Imag16RegClass.contains(Reg)) ||
        (Reg.isVirtual() &&
         MRI.getRegClass(Reg)->hasSuperClassEq(&MOS::Imag16RegClass))) {
      MachineOperand Lo = MachineOperand::CreateReg(Reg, IsLoad);
      MachineOperand Hi = Lo;
      Register Tmp = Reg;
      if (Reg.isPhysical()) {
        Lo.setReg(TRI->getSubReg(Reg, MOS::sublo));
        Hi.setReg(TRI->getSubReg(Reg, MOS::subhi));
      } else {
        assert(Reg.isVirtual());
        // Live intervals for the original virtual register will already have
        // been computed by this point. Since this code introduces
        // subregisters, these must be using a new virtual register; otherwise
        // there would be no subregister live ranges for the new instructions.
        // This can cause VirtRegMap to fail.
        Tmp = MRI.createVirtualRegister(&MOS::Imag16RegClass);
        Lo.setReg(Tmp);
        Lo.setSubReg(MOS::sublo);
        if (Lo.isDef())
          Lo.setIsUndef();
        Hi.setReg(Tmp);
        Hi.setSubReg(MOS::subhi);
      }
      if (!IsLoad && Tmp != Reg)
        Builder.buildCopy(Tmp, Reg);
      loadStoreByteStaticStackSlot(Builder, Lo, FrameIndex, 0,
                                   MF.getMachineMemOperand(MMO, 0, 1));
      loadStoreByteStaticStackSlot(Builder, Hi, FrameIndex, 1,
                                   MF.getMachineMemOperand(MMO, 1, 1));
      if (IsLoad && Tmp != Reg)
        Builder.buildCopy(Reg, Tmp);
    } else {
      loadStoreByteStaticStackSlot(
          Builder, MachineOperand::CreateReg(Reg, IsLoad), FrameIndex, 0, MMO);
    }
  }

  LLVM_DEBUG({
    dbgs() << "Inserted stack slot load/store:\n";
    for (const auto &MI : make_range(MIS.begin(), MIS.getInitial()))
      dbgs() << MI;
  });
}

const TargetRegisterClass *
MOSInstrInfo::getRegClass(const MCInstrDesc &MCID, unsigned OpNum,
                          const TargetRegisterInfo *TRI,
                          const MachineFunction &MF) const {
  auto *RC = TargetInstrInfo::getRegClass(MCID, OpNum, TRI, MF);
  const MOSSubtarget &STI = MF.getSubtarget<MOSSubtarget>();

  // On SPC700, LDImm can be used for imaginary registers.
  if (STI.hasSPC700() && MCID.getOpcode() == MOS::LDImm && OpNum == 0) {
    return &MOS::Anyi8RegClass;
  }

  return RC;
}

bool MOSInstrInfo::expandPostRAPseudo(MachineInstr &MI) const {
  MachineIRBuilder Builder(MI);

  bool Changed = true;
  switch (MI.getOpcode()) {
  default:
    Changed = false;
    break;
  // Post RA
  case MOS::IncNMOS:
  case MOS::DecNMOS:
    expandIncDecNMOS(Builder);
    break;
  case MOS::IncPtr:
  case MOS::DecPtr:
  case MOS::DecDcpPtr:
    expandIncDecPtr(Builder);
    break;
  case MOS::LDZpIdx:
    expandLDIdx(Builder, true);
    break;
  case MOS::LDAbsIdx:
    expandLDIdx(Builder, false);
    break;
  case MOS::LDImm1:
    expandLDImm1(Builder);
    break;
  case MOS::LDImm16:
  case MOS::LDImm16SPC700:
    expandLDImm16(Builder);
    break;
  case MOS::LDImm16Remat:
    expandLDImm16Remat(Builder);
    break;
  case MOS::LDZ:
    expandLDZ(Builder);
    break;
  case MOS::CmpBrImm:
  case MOS::CmpBrImag8:
  case MOS::CmpBrZero:
  case MOS::CmpBrZpIdx:
  case MOS::CmpBrAbs:
  case MOS::CmpBrAbsIdx:
  case MOS::CmpBrIndir:
  case MOS::CmpBrIndirIdx:
    expandCmpBr(Builder);
    break;

  // Control flow
  case MOS::GBR:
    expandGBR(Builder);
    break;
  }

  return Changed;
}

//===---------------------------------------------------------------------===//
// Post RA pseudos
//===---------------------------------------------------------------------===//

void MOSInstrInfo::expandLDIdx(MachineIRBuilder &Builder, bool ZP) const {
  auto &MI = *Builder.getInsertPt();
  const MOSSubtarget &STI = Builder.getMF().getSubtarget<MOSSubtarget>();
  auto DestReg = MI.getOperand(0).getReg();
  auto IndexReg = MI.getOperand(2).getReg();

  if (DestReg == IndexReg ||
      (STI.hasSPC700() && (DestReg == MOS::X || DestReg == MOS::Y))) {
    // A direct load does not exist for when X or Y is both the destination and
    // index register. Since the 6502 has no instruction for this, use A as the
    // destination instead, then transfer to the real destination.
    // SPC700 does not support absolute indexed loads into X or Y at all.
    Register Tmp = createVReg(Builder, MOS::AcRegClass);
    Builder.buildInstr(ZP ? MOS::LDAZpIdx : MOS::LDAAbsIdx)
        .addDef(Tmp)
        .add(MI.getOperand(1))
        .add(MI.getOperand(2));
    Builder.buildInstr(MOS::TA).add(MI.getOperand(0)).addUse(Tmp);
    MI.eraseFromParent();
    return;
  }

  unsigned Opcode;
  switch (DestReg) {
  default:
    llvm_unreachable("Bad destination for LD*Idx.");
  case MOS::A:
    Opcode = ZP ? MOS::LDAZpIdx : MOS::LDAAbsIdx;
    break;
  case MOS::X:
    Opcode = MOS::LDXIdx;
    break;
  case MOS::Y:
    Opcode = MOS::LDYIdx;
    break;
  }

  MI.setDesc(Builder.getTII().get(Opcode));
}

void MOSInstrInfo::expandLDImm1(MachineIRBuilder &Builder) const {
  const MOSSubtarget &STI = Builder.getMF().getSubtarget<MOSSubtarget>();
  auto &MI = *Builder.getInsertPt();
  Register DestReg = MI.getOperand(0).getReg();
  int64_t Val = MI.getOperand(1).getImm();

  unsigned Opcode;
  switch (DestReg) {
  default: {
    DestReg = STI.getRegisterInfo()->getMatchingSuperReg(DestReg, MOS::sublsb,
                                                         &MOS::Anyi8RegClass);
    assert(DestReg && "Unexpected destination for LDImm1");
    assert(MOS::GPRRegClass.contains(DestReg));
    Opcode = MOS::LDImm;
    MI.getOperand(0).setReg(DestReg);
    MI.getOperand(1).setImm(!!Val);
    break;
  }
  case MOS::C:
    Opcode = MOS::LDCImm;
    break;
  case MOS::V:
    if (Val) {
      if (STI.hasSPC700()) {
        // SPC700 does not have BIT, so we use stack operations to specifically
        // set V.
        Register ACopy = createVReg(Builder, MOS::AcRegClass);
        Builder.buildInstr(MOS::PH, {}, {Register(MOS::P)});
        Builder.buildInstr(MOS::PL, {ACopy}, {});
        Builder.buildInstr(MOS::ORAImm, {ACopy}, {ACopy, INT64_C(0x40)});
        Builder.buildInstr(MOS::PH).addUse(ACopy, RegState::Kill);
        Builder.buildInstr(MOS::PL, {MOS::P}, {});
        MI.eraseFromParent();
        return;
      }

      auto Instr = STI.hasHUC6280()
                       ? Builder.buildInstr(MOS::BITImmHUC6280, {MOS::V}, {})
                             .addUse(MOS::A, RegState::Undef)
                             .addImm(0xFF)
                       : Builder.buildInstr(MOS::BITAbs, {MOS::V}, {})
                             .addUse(MOS::A, RegState::Undef)
                             .addExternalSymbol("__set_v");
      Instr->getOperand(1).setIsUndef();
      MI.eraseFromParent();
      return;
    }
    Opcode = MOS::CLV;
    // Remove imm.
    MI.removeOperand(1);
    break;
  }

  MI.setDesc(Builder.getTII().get(Opcode));
}

void MOSInstrInfo::expandLDImm16(MachineIRBuilder &Builder) const {
  auto &MI = *Builder.getInsertPt();
  const TargetRegisterInfo &TRI =
      *Builder.getMF().getSubtarget().getRegisterInfo();
  bool UseScratch = MI.getOpcode() != MOS::LDImm16SPC700;

  Register Dst = MI.getOperand(0).getReg();
  MachineOperand Src = MI.getOperand(UseScratch ? 2 : 1);
  // This value is only valid if UseScratch is true.
  Register Tmp = UseScratch ? MI.getOperand(1).getReg() : Register(0);

  Register LoReg = TRI.getSubReg(Dst, MOS::sublo);
  auto Lo = Builder.buildInstr(MOS::LDImm);
  Lo.addDef(UseScratch ? Tmp : LoReg);
  if (Src.isImm()) {
    Lo.addImm(Src.getImm() & 0xff);
  } else {
    Lo.add(Src);
    Lo->getOperand(1).setTargetFlags(MOS::MO_LO);
  }
  if (UseScratch)
    copyPhysRegImpl(Builder, LoReg, Tmp);

  Register HiReg = TRI.getSubReg(Dst, MOS::subhi);
  auto Hi = Builder.buildInstr(MOS::LDImm);
  Hi.addDef(UseScratch ? Tmp : HiReg);
  if (Src.isImm()) {
    Hi.addImm(Src.getImm() >> 8);
  } else {
    Hi.add(Src);
    Hi->getOperand(1).setTargetFlags(MOS::MO_HI);
  }
  if (UseScratch) {
    // Appease the register scavenger by making this appear to be a
    // redefinition.
    if (Tmp.isVirtual())
      Hi.addUse(Tmp, RegState::Implicit);
    copyPhysRegImpl(Builder, HiReg, Tmp);
  }

  MI.eraseFromParent();
}

void MOSInstrInfo::expandLDImm16Remat(MachineIRBuilder &Builder) const {
  MachineInstr &MI = *Builder.getInsertPt();
  Register Scratch = createVReg(Builder, MOS::GPRRegClass);
  auto Ld = Builder.buildInstr(MOS::LDImm16, {MI.getOperand(0), Scratch}, {})
                .add(MI.getOperand(1));
  MI.eraseFromParent();
  Builder.setInstrAndDebugLoc(*Ld);
  expandLDImm16(Builder);
}

void MOSInstrInfo::expandLDZ(MachineIRBuilder &Builder) const {
  auto &MI = *Builder.getInsertPt();
  Register DestReg = MI.getOperand(0).getReg();
  const MOSSubtarget &STI = Builder.getMF().getSubtarget<MOSSubtarget>();

  if (MOS::Imag8RegClass.contains(DestReg)) {
    MI.setDesc(Builder.getTII().get(MOS::STZImag8));
  } else if (MOS::GPRRegClass.contains(DestReg)) {
    if (STI.hasHUC6280()) {
      MI.setDesc(Builder.getTII().get(MOS::CL));
    } else {
      MI.setDesc(Builder.getTII().get(MOS::LDImm));
      MI.addOperand(MachineOperand::CreateImm(0));
    }
  } else {
    llvm_unreachable("Unexpected register class for LDZ.");
  }
}

void MOSInstrInfo::expandIncDecNMOS(MachineIRBuilder &Builder) const {
  const auto &TII = Builder.getTII();

  auto &MI = *Builder.getInsertPt();
  Register R = MI.getOperand(0).getReg();
  bool IsInc = MI.getOpcode() == MOS::IncNMOS;
  assert(IsInc || MI.getOpcode() == MOS::DecNMOS);

  if (R == MOS::A) {
    Builder.buildInstr(MOS::LDCImm).addDef(MOS::C).addImm(0);
    auto Instr = Builder.buildInstr(MOS::ADCImm)
                     .addDef(MOS::A)
                     .addDef(MOS::C)
                     .addDef(MOS::V)
                     .addUse(MOS::A, RegState::Kill)
                     .addImm(IsInc ? 1 : 255)
                     .addUse(MOS::C);
    if (MI.modifiesRegister(MOS::NZ, Builder.getMRI()->getTargetRegisterInfo()))
      Instr.addDef(MOS::NZ, RegState::Implicit);

    MI.eraseFromParent();
    return;
  }

  assert(R == MOS::X || R == MOS::Y || MOS::Imag8RegClass.contains(R));
  MI.setDesc(TII.get(IsInc ? MOS::INC : MOS::DEC));
}

void MOSInstrInfo::expandIncDecPtr(MachineIRBuilder &Builder) const {
  MachineInstr &MI = *Builder.getInsertPt();
  const TargetRegisterInfo &TRI = *Builder.getMRI()->getTargetRegisterInfo();
  Register Reg = MI.getOperand(MI.getOpcode() == MOS::IncPtr ? 0 : 1).getReg();
  Register Lo = TRI.getSubReg(Reg, MOS::sublo);
  Register Hi = TRI.getSubReg(Reg, MOS::subhi);
  auto Op = MI.getOpcode() == MOS::IncPtr
                ? MOS::IncMB
                : (MI.getOpcode() == MOS::DecPtr ? MOS::DecMB : MOS::DecDcpMB);
  auto Inst = Builder.buildInstr(Op);
  if (MI.getOpcode() != MOS::IncPtr)
    Inst.addDef(MI.getOperand(0).getReg());
  Inst.addDef(Lo).addDef(Hi).addUse(Lo).addUse(Hi);

  if (MI.getOpcode() == MOS::IncPtr) {
    Inst->tieOperands(0, 2);
    Inst->tieOperands(1, 3);
  } else {
    Inst->tieOperands(1, 3);
    Inst->tieOperands(2, 4);
  }
  MI.eraseFromParent();
}

//===---------------------------------------------------------------------===//
// Control flow pseudos
//===---------------------------------------------------------------------===//

void MOSInstrInfo::expandGBR(MachineIRBuilder &Builder) const {
  MachineInstr &MI = *Builder.getInsertPt();

  MI.setDesc(Builder.getTII().get(MOS::BR));

  Register Tst = MI.getOperand(1).getReg();
  switch (Tst) {
  case MOS::C:
  case MOS::V:
    return;
  default: {
    Register TstReg =
        Builder.getMF().getSubtarget().getRegisterInfo()->getMatchingSuperReg(
            Tst, MOS::sublsb, &MOS::Anyi8RegClass);
    Builder.buildInstr(MOS::CmpZero, {}, {TstReg})
        .addDef(MOS::Z, RegState::Implicit);
  }
  }
  // Branch on zero flag, which is now the inverse of the test.
  MI.getOperand(1).setReg(MOS::Z);
  MI.getOperand(1).setIsKill();
  MI.getOperand(2).setImm(MI.getOperand(2).getImm() ? 0 : 1);
}

void MOSInstrInfo::expandCmpBr(MachineIRBuilder &Builder) const {
  MachineInstr &MI = *Builder.getInsertPt();

  const Register Flag = MI.getOperand(1).getReg();

  unsigned CMPOpcode;
  switch (MI.getOpcode()) {
  case MOS::CmpBrImm:
    CMPOpcode = MOS::CMPImm;
    break;
  case MOS::CmpBrImag8:
    CMPOpcode = MOS::CMPImag8;
    break;
  case MOS::CmpBrZero:
    CMPOpcode = MOS::CmpZero;
    break;
  case MOS::CmpBrZpIdx:
    CMPOpcode = MOS::CMPZpIdx;
    break;
  case MOS::CmpBrAbs:
    CMPOpcode = MOS::CMPAbs;
    break;
  case MOS::CmpBrAbsIdx:
    CMPOpcode = MOS::CMPAbsIdx;
    break;
  case MOS::CmpBrIndir:
    CMPOpcode = MOS::CMPIndir;
    break;
  case MOS::CmpBrIndirIdx:
    CMPOpcode = MOS::CMPIndirIdx;
    break;
  }

  auto CMP = Builder.buildInstr(CMPOpcode);
  if (CMPOpcode != MOS::CmpZero)
    CMP.addDef(MOS::C, RegState::Dead);
  for (unsigned I = 3, E = MI.getNumOperands(); I != E; I++)
    CMP.add(MI.getOperand(I));
  CMP.cloneMemRefs(*CMP);
  CMP.addDef(Flag, RegState::Implicit);

  Builder.buildInstr(MOS::BR)
      .add(MI.getOperand(0))
      .addUse(Flag, RegState::Kill)
      .add(MI.getOperand(2));

  MI.eraseFromParent();
}

bool MOSInstrInfo::reverseBranchCondition(
    SmallVectorImpl<MachineOperand> &Cond) const {
  // Condition includes all arguments except the branch target.
  MachineOperand &ValMO =
      (Cond.front().getImm() == MOS::CmpBrZeroMultiByte) ? Cond[1] : Cond[2];
  ValMO.setImm(!ValMO.getImm());
  // Success.
  return false;
}

std::pair<unsigned, unsigned>
MOSInstrInfo::decomposeMachineOperandsTargetFlags(unsigned TF) const {
  return std::make_pair(TF, 0u);
}

ArrayRef<std::pair<int, const char *>>
MOSInstrInfo::getSerializableTargetIndices() const {
  static const std::pair<int, const char *> Flags[] = {
      {MOS::TI_STATIC_STACK, "mos-static-stack"}};
  return Flags;
}

ArrayRef<std::pair<unsigned, const char *>>
MOSInstrInfo::getSerializableDirectMachineOperandTargetFlags() const {
  static const std::pair<unsigned, const char *> Flags[] = {
      {MOS::MO_LO, "lo"},
      {MOS::MO_HI, "hi"},
      {MOS::MO_HI_JT, "hi-jt"},
      {MOS::MO_ZEROPAGE, "zeropage"}};
  return Flags;
}
