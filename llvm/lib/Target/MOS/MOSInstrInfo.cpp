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
#include "llvm/CodeGen/TargetInstrInfo.h"
#include "llvm/CodeGen/TargetRegisterInfo.h"
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

bool MOSInstrInfo::isReallyTriviallyReMaterializable(const MachineInstr &MI,
                                                     AAResults *AA) const {
  switch (MI.getOpcode()) {
  default:
    return false;
  case MOS::LDImm16:
    return true;
  }
}

unsigned MOSInstrInfo::isLoadFromStackSlot(const MachineInstr &MI,
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

unsigned MOSInstrInfo::isStoreToStackSlot(const MachineInstr &MI,
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

MachineInstr &MOSInstrInfo::duplicate(MachineBasicBlock &MBB,
                                      MachineBasicBlock::iterator InsertBefore,
                                      const MachineInstr &Orig) const {
  MachineInstr &MI = TargetInstrInfo::duplicate(MBB, InsertBefore, Orig);
  // These require tied variable operands, which aren't cloned by default.
  if (MI.getOpcode() == MOS::IncMB || MI.getOpcode() == MOS::DecMB)
    for (unsigned I = 0, E = MI.getNumExplicitDefs(); I != E; ++I)
      if (Orig.getOperand(I).isTied())
        MI.tieOperands(I, Orig.findTiedOperandIdx(I));
  return MI;
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
  // Overestimate the size of each instruction to guarantee that any necessary
  // branches are relaxed.
  return 3;
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
    return MI.getOperand(0).getMBB();
  case MOS::JMPIndir:
    return nullptr;
  }
}

bool MOSInstrInfo::analyzeBranch(MachineBasicBlock &MBB,
                                 MachineBasicBlock *&TBB,
                                 MachineBasicBlock *&FBB,
                                 SmallVectorImpl<MachineOperand> &Cond,
                                 bool AllowModify) const {
  auto I = MBB.getFirstTerminator();

  // Advance past any comparison terminators.
  while (I != MBB.end() && I->isCompare())
    ++I;

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
    Cond.push_back(FirstBR->getOperand(1));
    Cond.push_back(FirstBR->getOperand(2));
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
  // Since analyzeBranch succeeded, we know that the only terminators are
  // comparisons and branches.

  auto Begin = MBB.getFirstTerminator();
  auto End = MBB.end();

  // Advance to first branch.
  while (Begin != End && Begin->isCompare())
    ++Begin;

  // Erase all remaining terminators.
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
  // Since analyzeBranch succeeded and any existing branches were removed, the
  // only remaining terminators are comparisons.

  const MOSSubtarget &STI = MBB.getParent()->getSubtarget<MOSSubtarget>();

  MachineIRBuilder Builder(MBB, MBB.end());
  unsigned NumAdded = 0;
  if (BytesAdded)
    *BytesAdded = 0;

  // Unconditional branch target.
  auto *UBB = TBB;

  // Conditional branch.
  if (!Cond.empty()) {
    assert(TBB);
    // The condition stores the arguments for the BR instruction.
    assert(Cond.size() == 2);

    // The unconditional branch will be to the false branch (if any).
    UBB = FBB;

    // Add conditional branch.
    Register Reg = Cond[0].getReg();
    unsigned Opcode = Reg.isVirtual() || !MOS::FlagRegClass.contains(Reg)
                          ? MOS::GBR
                          : MOS::BR;
    auto BR = Builder.buildInstr(Opcode).addMBB(TBB);
    for (const MachineOperand &Op : Cond)
      BR.add(Op);
    ++NumAdded;
    if (BytesAdded)
      *BytesAdded += getInstSizeInBytes(*BR);
  }

  // Add unconditional branch if necessary.
  if (UBB) {
    // For 65C02, assume BRA and relax into JMP in insertIndirectBranch if
    // necessary.
    auto JMP =
        Builder.buildInstr(STI.has65C02() ? MOS::BRA : MOS::JMP).addMBB(UBB);
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
                               MCRegister SrcReg, bool KillSrc) const {
  MachineIRBuilder Builder(MBB, MI);
  copyPhysRegImpl(Builder, DestReg, SrcReg);
}

static Register createVReg(MachineIRBuilder &Builder,
                           const TargetRegisterClass &RC) {
  Builder.getMF().getProperties().reset(
      MachineFunctionProperties::Property::NoVRegs);
  return Builder.getMRI()->createVirtualRegister(&RC);
}

bool MOSInstrInfo::shouldOverlapInterval(const MachineInstr &MI) const {
  return MI.getOpcode() != MOS::CMPTermZ;
}

static bool isTargetCopy(MachineInstr &MI) {
  switch (MI.getOpcode()) {
  case MOS::LDImag8:
  case MOS::STImag8:
  case MOS::TA:
  case MOS::T_A:
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

void MOSInstrInfo::copyPhysRegImpl(MachineIRBuilder &Builder, Register DestReg,
                                   Register SrcReg, bool Force) const {
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
    } else {
      copyPhysRegImpl(Builder, DestReg,
                      getRegWithVal(Builder, SrcReg, MOS::AcRegClass));
    }
  } else if (AreClasses(MOS::Imag8RegClass, MOS::GPRRegClass)) {
    Builder.buildInstr(MOS::STImag8).addDef(DestReg).addUse(SrcReg);
  } else if (AreClasses(MOS::GPRRegClass, MOS::Imag8RegClass)) {
    Builder.buildInstr(MOS::LDImag8).addDef(DestReg).addUse(SrcReg);
  } else if (AreClasses(MOS::Imag8RegClass, MOS::Imag8RegClass)) {
    copyPhysRegImpl(Builder, DestReg,
                    getRegWithVal(Builder, SrcReg, MOS::GPRRegClass));
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
        assert(!Builder.getInsertPt()->readsRegister(DestReg));

        copyPhysRegImpl(Builder, DestReg, SrcReg);
      } else {
        if (DestReg == MOS::C) {
          // C = SrcReg >= 1
          Builder.buildInstr(
              MOS::CMPImm, {MOS::C},
              {getRegWithVal(Builder, SrcReg, MOS::GPRRegClass), INT64_C(1)});
        } else {
          assert(DestReg == MOS::V);
          const TargetRegisterClass &StackRegClass =
              STI.has65C02() ? MOS::GPRRegClass : MOS::AcRegClass;

          if (StackRegClass.contains(SrcReg)) {
            Builder.buildInstr(MOS::PH, {}, {SrcReg});
            Builder.buildInstr(MOS::PL, {SrcReg}, {})
                .addDef(MOS::NZ, RegState::Implicit);
            Builder.buildInstr(MOS::SelectImm, {MOS::V},
                               {Register(MOS::Z), INT64_C(0), INT64_C(-1)});
          } else {
            Register Tmp = createVReg(Builder, StackRegClass);
            copyPhysRegImpl(Builder, Tmp, SrcReg, /*Force=*/true);
            std::prev(Builder.getInsertPt())
                ->addOperand(MachineOperand::CreateReg(MOS::NZ,
                                                       /*isDef=*/true,
                                                       /*isImp=*/true));
            Builder.buildInstr(MOS::SelectImm, {MOS::V},
                               {Register(MOS::Z), INT64_C(0), INT64_C(-1)});
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
// register class. This will search for an ealier instance of this value to use,
// starting from the insertion point of the given builder. If none is found,
// creates a virtual register and copies in the value.
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
                                                     unsigned FoldIdx) const {
  const MachineFunction &MF = *MI.getMF();
  const MOSFrameLowering &TFL =
      *MF.getSubtarget<MOSSubtarget>().getFrameLowering();
  if (!TFL.usesStaticStack(MF))
    return TargetInstrInfo::canFoldCopy(MI, FoldIdx);

  Register FoldReg = MI.getOperand(FoldIdx).getReg();
  if (MOS::GPRRegClass.contains(FoldReg) ||
      MOS::GPR_LSBRegClass.contains(FoldReg))
    return TargetInstrInfo::canFoldCopy(MI, FoldIdx);
  if (FoldReg.isVirtual()) {
    const auto *RC = MI.getMF()->getRegInfo().getRegClass(FoldReg);
    if (RC == &MOS::GPRRegClass || RC == &MOS::GPR_LSBRegClass)
      return TargetInstrInfo::canFoldCopy(MI, FoldIdx);
  }
  return nullptr;
}

void MOSInstrInfo::storeRegToStackSlot(MachineBasicBlock &MBB,
                                       MachineBasicBlock::iterator MI,
                                       Register SrcReg, bool isKill,
                                       int FrameIndex,
                                       const TargetRegisterClass *RC,
                                       const TargetRegisterInfo *TRI) const {
  loadStoreRegStackSlot(MBB, MI, SrcReg, isKill, FrameIndex, RC, TRI,
                        /*IsLoad=*/false);
}

void MOSInstrInfo::loadRegFromStackSlot(MachineBasicBlock &MBB,
                                        MachineBasicBlock::iterator MI,
                                        Register DestReg, int FrameIndex,
                                        const TargetRegisterClass *RC,
                                        const TargetRegisterInfo *TRI) const {
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

  // If we're using the soft stack, since the offset is not yet known, it may be
  // either 8 or 16 bits. Emit a 16-bit pseudo to be lowered during frame index
  // elimination.
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
        // been computed by this point. Since this code introduces subregisters,
        // these must be using a new virtual register; otherwise there would be
        // no subregister live ranges for the new instructions. This can cause
        // VirtRegMap to fail.
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

bool MOSInstrInfo::expandPostRAPseudo(MachineInstr &MI) const {
  MachineIRBuilder Builder(MI);

  bool Changed = true;
  switch (MI.getOpcode()) {
  default:
    Changed = false;
    break;
  // Post RA
  case MOS::INC:
  case MOS::DEC:
    expandIncDec(Builder);
    break;
  case MOS::IncPtr:
  case MOS::DecPtr:
    expandIncDecPtr(Builder);
    break;
  case MOS::LDAbsIdx:
    expandLDIdx(Builder);
    break;
  case MOS::LDImm1:
    expandLDImm1(Builder);
    break;
  case MOS::LDImm16:
    expandLDImm16(Builder);
    break;
  case MOS::LDImm16Remat:
    expandLDImm16Remat(Builder);
    break;
  case MOS::LDZ:
    expandLDZ(Builder);
    break;
  case MOS::CMPNZImm:
  case MOS::CMPNZImag8:
  case MOS::CMPNZAbs:
  case MOS::CMPNZAbsIdx:
  case MOS::CMPNZIndirIdx:
  case MOS::SBCNZImm:
  case MOS::SBCNZImag8:
  case MOS::SBCNZAbs:
  case MOS::SBCNZAbsIdx:
  case MOS::SBCNZIndirIdx:
    expandNZ(Builder);
    break;
  case MOS::CMPTermImm:
  case MOS::CMPTermImag8:
  case MOS::CMPTermAbs:
  case MOS::CMPTermIndir:
  case MOS::CMPTermIdx:
    expandCMPTerm(Builder);
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

void MOSInstrInfo::expandLDIdx(MachineIRBuilder &Builder) const {
  auto &MI = *Builder.getInsertPt();

  // This occur when X or Y is both the destination and index register.
  // Since the 6502 has no instruction for this, use A as the destination
  // instead, then transfer to the real destination.
  if (MI.getOperand(0).getReg() == MI.getOperand(2).getReg()) {
    Register Tmp = createVReg(Builder, MOS::AcRegClass);
    Builder.buildInstr(MOS::LDAAbsIdx)
        .addDef(Tmp)
        .add(MI.getOperand(1))
        .add(MI.getOperand(2));
    Builder.buildInstr(MOS::TA).add(MI.getOperand(0)).addUse(Tmp);
    MI.eraseFromParent();
    return;
  }

  unsigned Opcode;
  switch (MI.getOperand(0).getReg()) {
  default:
    llvm_unreachable("Bad destination for LDAbsIdx.");
  case MOS::A:
    Opcode = MOS::LDAAbsIdx;
    break;
  case MOS::X:
    Opcode = MOS::LDXAbsIdx;
    break;
  case MOS::Y:
    Opcode = MOS::LDYAbsIdx;
    break;
  }

  MI.setDesc(Builder.getTII().get(Opcode));
}

void MOSInstrInfo::expandLDImm1(MachineIRBuilder &Builder) const {
  auto &MI = *Builder.getInsertPt();
  Register DestReg = MI.getOperand(0).getReg();
  int64_t Val = MI.getOperand(1).getImm();

  unsigned Opcode;
  switch (DestReg) {
  default: {
    DestReg =
        Builder.getMF().getSubtarget().getRegisterInfo()->getMatchingSuperReg(
            DestReg, MOS::sublsb, &MOS::Anyi8RegClass);
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
      auto Instr = Builder.buildInstr(MOS::BITAbs, {MOS::V}, {})
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

  Register Dst = MI.getOperand(0).getReg();
  Register Tmp = MI.getOperand(1).getReg();
  MachineOperand Src = MI.getOperand(2);

  auto Lo = Builder.buildInstr(MOS::LDImm, {Tmp}, {});
  if (Src.isImm()) {
    Lo.addImm(Src.getImm() & 0xff);
  } else {
    Lo.add(Src);
    Lo->getOperand(1).setTargetFlags(MOS::MO_LO);
  }
  copyPhysRegImpl(Builder, TRI.getSubReg(Dst, MOS::sublo), Tmp);

  auto Hi = Builder.buildInstr(MOS::LDImm, {Tmp}, {});
  if (Src.isImm()) {
    Hi.addImm(Src.getImm() >> 8);
  } else {
    Hi.add(Src);
    Hi->getOperand(1).setTargetFlags(MOS::MO_HI);
  }
  // Appease the register scavenger by making this appear to be a redefinition.
  if (Tmp.isVirtual())
    Hi.addUse(Tmp, RegState::Implicit);
  copyPhysRegImpl(Builder, TRI.getSubReg(Dst, MOS::subhi), Tmp);

  MI.eraseFromParent();
}

void MOSInstrInfo::expandLDImm16Remat(MachineIRBuilder &Builder) const {
  MachineInstr &MI = *Builder.getInsertPt();
  Register Scratch = createVReg(Builder, MOS::GPRRegClass);
  auto Ld = Builder.buildInstr(MOS::LDImm16, {MI.getOperand(0), Scratch}, {})
                .add(MI.getOperand(1));
  MI.eraseFromParent();
  Builder.setInsertPt(*Ld->getParent(), &*Ld);
  expandLDImm16(Builder);
}

void MOSInstrInfo::expandLDZ(MachineIRBuilder &Builder) const {
  auto &MI = *Builder.getInsertPt();
  Register DestReg = MI.getOperand(0).getReg();

  if (MOS::Imag8RegClass.contains(DestReg)) {
    MI.setDesc(Builder.getTII().get(MOS::STZImag8));
  } else if (MOS::GPRRegClass.contains(DestReg)) {
    MI.setDesc(Builder.getTII().get(MOS::LDImm));
    MI.addOperand(MachineOperand::CreateImm(0));
  } else {
    llvm_unreachable("Unexpected register class for LDZ.");
  }
}

void MOSInstrInfo::expandIncDec(MachineIRBuilder &Builder) const {
  const auto &TII = Builder.getTII();

  auto &MI = *Builder.getInsertPt();
  Register R = MI.getOperand(0).getReg();
  bool IsInc = MI.getOpcode() == MOS::INC;
  assert(IsInc || MI.getOpcode() == MOS::DEC);

  switch (R) {
  case MOS::A: {
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
    break;
  }
  case MOS::X:
  case MOS::Y:
    MI.setDesc(TII.get(IsInc ? MOS::IN : MOS::DE));
    break;
  default:
    assert(MOS::Imag8RegClass.contains(R));
    MI.setDesc(TII.get(IsInc ? MOS::INCImag8 : MOS::DECImag8));
    break;
  }
}

void MOSInstrInfo::expandIncDecPtr(MachineIRBuilder &Builder) const {
  MachineInstr &MI = *Builder.getInsertPt();
  const TargetRegisterInfo &TRI = *Builder.getMRI()->getTargetRegisterInfo();
  Register Reg = MI.getOperand(MI.getOpcode() == MOS::IncPtr ? 0 : 1).getReg();
  Register Lo = TRI.getSubReg(Reg, MOS::sublo);
  Register Hi = TRI.getSubReg(Reg, MOS::subhi);
  auto Op = Builder.buildInstr(MI.getOpcode() == MOS::IncPtr ? MOS::IncMB
                                                             : MOS::DecMB);
  if (MI.getOpcode() == MOS::DecPtr)
    Op.addDef(MI.getOperand(0).getReg());
  Op.addDef(Lo).addDef(Hi).addUse(Lo).addUse(Hi);

  if (MI.getOpcode() == MOS::IncPtr) {
    Op->tieOperands(0, 2);
    Op->tieOperands(1, 3);
  } else {
    Op->tieOperands(1, 3);
    Op->tieOperands(2, 4);
  }
  MI.eraseFromParent();
}

//===---------------------------------------------------------------------===//
// NZ pseudos
//===---------------------------------------------------------------------===//

void MOSInstrInfo::expandNZ(MachineIRBuilder &Builder) const {
  MachineInstr &MI = *Builder.getInsertPt();
  Register N;
  Register Z;
  switch (MI.getOpcode()) {
  case MOS::CMPNZImm:
  case MOS::CMPNZImag8:
  case MOS::CMPNZAbs:
  case MOS::CMPNZAbsIdx:
  case MOS::CMPNZIndirIdx:
    N = MI.getOperand(1).getReg();
    Z = MI.getOperand(2).getReg();
    break;
  case MOS::SBCNZImm:
  case MOS::SBCNZImag8:
  case MOS::SBCNZAbs:
  case MOS::SBCNZAbsIdx:
  case MOS::SBCNZIndirIdx:
    N = MI.getOperand(2).getReg();
    Z = MI.getOperand(4).getReg();
    break;
  }

  // The NZ location to which to copy.
  Register NZOut = MOS::NoRegister;
  // Which of N or Z to copy.
  Register NZIn = MOS::NoRegister;
  if (N) {
    assert(!Z);
    NZOut = N;
    NZIn = MOS::N;
  } else if (Z) {
    NZOut = Z;
    NZIn = MOS::Z;
  }

  MachineInstrBuilder Op;
  switch (MI.getOpcode()) {
  case MOS::CMPNZImm:
  case MOS::CMPNZImag8:
  case MOS::CMPNZAbs:
  case MOS::CMPNZAbsIdx:
  case MOS::CMPNZIndirIdx: {
    unsigned Opcode;
    switch (MI.getOpcode()) {
    case MOS::CMPNZImm:
      Opcode = MOS::CMPImm;
      break;
    case MOS::CMPNZImag8:
      Opcode = MOS::CMPImag8;
      break;
    case MOS::CMPNZAbs:
      Opcode = MOS::CMPAbs;
      break;
    case MOS::CMPNZAbsIdx:
      Opcode = MOS::CMPAbsIdx;
      break;
    case MOS::CMPNZIndirIdx:
      Opcode = MOS::CMPIndirIdx;
      break;
    }
    Op = Builder.buildInstr(Opcode, {MI.getOperand(0)}, {});
    for (int Idx : seq(3u, MI.getNumOperands()))
      Op.add(MI.getOperand(Idx));
    break;
  }
  case MOS::SBCNZImm:
  case MOS::SBCNZImag8:
  case MOS::SBCNZAbs:
  case MOS::SBCNZAbsIdx:
  case MOS::SBCNZIndirIdx: {
    unsigned Opcode;
    switch (MI.getOpcode()) {
    case MOS::SBCNZImm:
      Opcode = MOS::SBCImm;
      break;
    case MOS::SBCNZImag8:
      Opcode = MOS::SBCImag8;
      break;
    case MOS::SBCNZAbs:
      Opcode = MOS::SBCAbs;
      break;
    case MOS::SBCNZAbsIdx:
      Opcode = MOS::SBCAbsIdx;
      break;
    case MOS::SBCNZIndirIdx:
      Opcode = MOS::SBCIndirIdx;
      break;
    }
    Op = Builder.buildInstr(
        Opcode, {MI.getOperand(0), MI.getOperand(1), MI.getOperand(3)}, {});
    for (int Idx : seq(5u, MI.getNumOperands()))
      Op.add(MI.getOperand(Idx));
    break;
  }
  }

  // Copy out N or Z to a Anyi1 location if requested.
  if (NZOut) {
    assert(NZIn);
    Op.addDef(MOS::NZ, RegState::Implicit);
    Builder.buildInstr(MOS::SelectImm, {NZOut},
                       {NZIn, INT64_C(-1), INT64_C(0)});
  }
  MI.eraseFromParent();
}

void MOSInstrInfo::expandCMPTerm(MachineIRBuilder &Builder) const {
  MachineInstr &MI = *Builder.getInsertPt();
  switch (MI.getOpcode()) {
  case MOS::CMPTermImm:
    MI.setDesc(Builder.getTII().get(MOS::CMPImm));
    break;
  case MOS::CMPTermImag8:
    MI.setDesc(Builder.getTII().get(MOS::CMPImag8));
    break;
  case MOS::CMPTermAbs:
    MI.setDesc(Builder.getTII().get(MOS::CMPAbs));
    break;
  case MOS::CMPTermIdx:
    MI.setDesc(Builder.getTII().get(MOS::CMPAbsIdx));
    break;
  case MOS::CMPTermIndir:
    MI.setDesc(Builder.getTII().get(MOS::CMPIndirIdx));
    break;
  }
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
    Builder.buildInstr(MOS::CMPTermZ, {MOS::C}, {TstReg})
        ->getOperand(0)
        .setIsDead();
  }
  }
  // Branch on zero flag, which is now the inverse of the test.
  MI.getOperand(1).setReg(MOS::Z);
  MI.getOperand(2).setImm(MI.getOperand(2).getImm() ? 0 : 1);
}

bool MOSInstrInfo::reverseBranchCondition(
    SmallVectorImpl<MachineOperand> &Cond) const {
  assert(Cond.size() == 2);
  auto &Val = Cond[1];
  Val.setImm(!Val.getImm());
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
      {MOS::MO_LO, "lo"}, {MOS::MO_HI, "hi"}, {MOS::MO_HI_JT, "hi-jt"}};
  return Flags;
}
