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
#include "MOSRegisterInfo.h"

#include "llvm/ADT/BitVector.h"
#include "llvm/ADT/SparseBitVector.h"
#include "llvm/CodeGen/GlobalISel/MachineIRBuilder.h"
#include "llvm/CodeGen/GlobalISel/Utils.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineMemOperand.h"
#include "llvm/CodeGen/MachineOperand.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/PseudoSourceValue.h"
#include "llvm/CodeGen/TargetRegisterInfo.h"
#include "llvm/Support/Compiler.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Target/TargetMachine.h"

using namespace llvm;

#define DEBUG_TYPE "mos-instrinfo"

#define GET_INSTRINFO_CTOR_DTOR
#include "MOSGenInstrInfo.inc"

namespace {

// Returns a conservative analysis for whether Reg could be live at the current
// insertion point of Builder.
bool isMaybeLive(MachineIRBuilder &Builder, Register Reg) {
  const auto &MBB = Builder.getMBB();
  return MBB.computeRegisterLiveness(
             MBB.getParent()->getSubtarget().getRegisterInfo(), Reg,
             Builder.getInsertPt()) != MachineBasicBlock::LQR_Dead;
}

} // namespace

MOSInstrInfo::MOSInstrInfo()
    : MOSGenInstrInfo(/*CFSetupOpcode=*/MOS::ADJCALLSTACKDOWN,
                      /*CFDestroyOpcode=*/MOS::ADJCALLSTACKUP) {}

bool MOSInstrInfo::isReallyTriviallyReMaterializable(const MachineInstr &MI,
                                                     AAResults *AA) const {
  switch (MI.getOpcode()) {
  default:
    return TargetInstrInfo::isReallyTriviallyReMaterializable(MI, AA);
  // Note: Rematerializations cannot occur in terminators, so NZ cannot be live.
  case MOS::LDImm:
  case MOS::LDImm1:
    return true;
  }
}

unsigned MOSInstrInfo::isLoadFromStackSlot(const MachineInstr &MI,
                                           int &FrameIndex) const {
  switch (MI.getOpcode()) {
  default:
    return 0;
  case MOS::LDabs_offset:
  case MOS::LDstk:
    FrameIndex = MI.getOperand(1).getIndex();
    return MI.getOperand(0).getReg();
  }
}

unsigned MOSInstrInfo::isStoreToStackSlot(const MachineInstr &MI,
                                          int &FrameIndex) const {
  switch (MI.getOpcode()) {
  default:
    return 0;
  case MOS::STabs_offset:
  case MOS::STstk:
    FrameIndex = MI.getOperand(1).getIndex();
    return MI.getOperand(0).getReg();
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

  // Use the new register classes computed above, if any.
  if (Reg1Class)
    MRI.setRegClass(Reg1, Reg1Class);
  if (Reg2Class)
    MRI.setRegClass(Reg2, Reg2Class);
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
  case MOS::BR:
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
  case MOS::BR:
  case MOS::JMP:
    return MI.getOperand(0).getMBB();
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
  if (I == MBB.end())
    return false;

  // Non-branch terminators cannot be analyzed.
  if (!I->isBranch())
    return true;

  // Analyze first branch.
  auto FirstBR = I++;
  if (FirstBR->isPreISelOpcode())
    return true;
  // First branch always forms true edge, whether conditional or unconditional.
  TBB = getBranchDestBlock(*FirstBR);
  if (FirstBR->isConditionalBranch()) {
    Cond.push_back(FirstBR->getOperand(1));
    Cond.push_back(FirstBR->getOperand(2));
  }

  // If there's no second branch, done.
  if (I == MBB.end())
    return false;

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
    for (auto I = Begin; I != End; ++I)
      *BytesRemoved += getInstSizeInBytes(*I);
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
    auto BR = Builder.buildInstr(MOS::BR).addMBB(TBB);
    for (const MachineOperand &Op : Cond)
      BR.add(Op);
    ++NumAdded;
    if (BytesAdded)
      *BytesAdded += getInstSizeInBytes(*BR);
  }

  // Add unconditional branch if necessary.
  if (UBB) {
    auto JMP = Builder.buildInstr(MOS::JMP).addMBB(UBB);
    ++NumAdded;
    if (BytesAdded)
      *BytesAdded += getInstSizeInBytes(*JMP);
  }

  return NumAdded;
}

void MOSInstrInfo::copyPhysReg(MachineBasicBlock &MBB,
                               MachineBasicBlock::iterator MI,
                               const DebugLoc &DL, MCRegister DestReg,
                               MCRegister SrcReg, bool KillSrc) const {
  MachineIRBuilder Builder(MBB, MI);
  copyPhysRegImpl(Builder, DestReg, SrcReg);
}

void MOSInstrInfo::copyPhysRegImpl(MachineIRBuilder &Builder,
                                   MCRegister DestReg,
                                   MCRegister SrcReg) const {
  if (DestReg == SrcReg)
    return;

  const TargetRegisterInfo &TRI =
      *Builder.getMF().getSubtarget().getRegisterInfo();

  const auto &AreClasses = [&](const TargetRegisterClass &Dest,
                               const TargetRegisterClass &Src) {
    return Dest.contains(DestReg) && Src.contains(SrcReg);
  };

  // Reg is either A or ALSB.
  const auto &CopyThroughA = [&](Register Reg) {
    bool IsAMaybeLive = isMaybeLive(Builder, MOS::A);
    if (IsAMaybeLive)
      Builder.buildInstr(MOS::PH).addUse(MOS::A);
    copyPhysRegImpl(Builder, Reg, SrcReg);
    copyPhysRegImpl(Builder, DestReg, Reg);
    if (IsAMaybeLive)
      Builder.buildInstr(MOS::PL).addDef(MOS::A);
  };

  if (AreClasses(MOS::GPRRegClass, MOS::GPRRegClass)) {
    if (SrcReg == MOS::A) {
      assert(MOS::XYRegClass.contains(DestReg));
      Builder.buildInstr(MOS::TA).addDef(DestReg).addUse(SrcReg);
    } else if (DestReg == MOS::A) {
      assert(MOS::XYRegClass.contains(SrcReg));
      Builder.buildInstr(MOS::T_A).addDef(DestReg).addUse(SrcReg);
    } else
      CopyThroughA(MOS::A);
  } else if (AreClasses(MOS::Imag8RegClass, MOS::GPRRegClass)) {
    Builder.buildInstr(MOS::STImag8).addDef(DestReg).addUse(SrcReg);
  } else if (AreClasses(MOS::GPRRegClass, MOS::Imag8RegClass)) {
    Builder.buildInstr(MOS::LDImag8).addDef(DestReg).addUse(SrcReg);
  } else if (AreClasses(MOS::Imag8RegClass, MOS::Imag8RegClass)) {
    CopyThroughA(MOS::A);
  } else if (AreClasses(MOS::Imag16RegClass, MOS::Imag16RegClass)) {
    copyPhysRegImpl(Builder, TRI.getSubReg(DestReg, MOS::sublo),
                    TRI.getSubReg(SrcReg, MOS::sublo));
    copyPhysRegImpl(Builder, TRI.getSubReg(DestReg, MOS::subhi),
                    TRI.getSubReg(SrcReg, MOS::subhi));
  } else if (AreClasses(MOS::Anyi1RegClass, MOS::Anyi1RegClass)) {

    Register DestReg8 =
        TRI.getMatchingSuperReg(DestReg, MOS::sublsb, &MOS::Anyi8RegClass);
    if (!DestReg8) {
      LLVM_DEBUG(dbgs() << TRI.getName(DestReg) << " <- " << TRI.getName(SrcReg)
                        << "\n");
      report_fatal_error("Unsupported physical register copy.");
    }

    const MachineInstr &MI = *Builder.getInsertPt();
    // MOS defines LSB writes to clobber the rest of the register. This fact is
    // represented on the incoming COPY instruction by lack of an implicit use
    // of the full 8-bit destination register. Accordingly, we check for this to
    // make sure the code generator up until now accounted for the clobber.
    assert(!MI.readsRegister(DestReg8));

    Register SrcReg8 =
        TRI.getMatchingSuperReg(SrcReg, MOS::sublsb, &MOS::Anyi8RegClass);
    if (SrcReg8)
      copyPhysRegImpl(Builder, DestReg8, SrcReg8);
    else
      Builder.buildInstr(MOS::ZExt1, {DestReg8}, {Register(SrcReg)});
  } else {
    LLVM_DEBUG(dbgs() << TRI.getName(DestReg) << " <- " << TRI.getName(SrcReg)
                      << "\n");
    report_fatal_error("Unsupported physical register copy.");
  }
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
                                         Register Reg, int FrameIndex,
                                         int64_t Offset, MachineMemOperand *MMO,
                                         bool IsLoad) {
  Register Tmp = Builder.getMRI()->createVirtualRegister(&MOS::GPRRegClass);

  if (IsLoad) {
    Builder.buildInstr(MOS::LDabs_offset, {Tmp}, {})
        .addFrameIndex(FrameIndex)
        .addImm(Offset)
        .addMemOperand(MMO);
    Builder.buildCopy(Reg, Tmp);
  } else {
    Builder.buildCopy(Tmp, Reg);
    Builder.buildInstr(MOS::STabs_offset, {}, {Tmp})
        .addFrameIndex(FrameIndex)
        .addImm(Offset)
        .addMemOperand(MMO);
  }
}

void MOSInstrInfo::loadStoreRegStackSlot(
    MachineBasicBlock &MBB, MachineBasicBlock::iterator MI, Register Reg,
    bool IsKill, int FrameIndex, const TargetRegisterClass *RC,
    const TargetRegisterInfo *TRI, bool IsLoad) const {
  MachineFunction &MF = *MBB.getParent();
  MachineFrameInfo &MFI = MF.getFrameInfo();

  MachinePointerInfo PtrInfo =
      MachinePointerInfo::getFixedStack(MF, FrameIndex);
  MachineMemOperand *MMO = MF.getMachineMemOperand(
      PtrInfo, IsLoad ? MachineMemOperand::MOLoad : MachineMemOperand::MOStore,
      MFI.getObjectSize(FrameIndex), MFI.getObjectAlign(FrameIndex));

  MachineIRBuilder Builder(MBB, MI);

  // At this point, NZ cannot be live, since this will never occur inside a
  // terminator.

  // If we're using the soft stack, since the offset is not yet known, it may be
  // either 8 or 16 bits. Emit a 16-bit pseudo to be lowered during frame index
  // elimination.
  if (!MF.getFunction().doesNotRecurse()) {
    if (IsLoad) {
      Builder.buildInstr(MOS::LDstk)
          .addDef(Reg)
          .addFrameIndex(FrameIndex)
          .addImm(0)
          .addMemOperand(MMO);
    } else {
      Builder.buildInstr(MOS::STstk)
          .addReg(Reg, getKillRegState(IsKill))
          .addFrameIndex(FrameIndex)
          .addImm(0)
          .addMemOperand(MMO);
    }
    return;
  }

  MachineInstrSpan MIS(MI, &MBB);
  if (MOS::Imag16RegClass.contains(Reg)) {
    Register Lo = TRI->getSubReg(Reg, MOS::sublo);
    Register Hi = TRI->getSubReg(Reg, MOS::subhi);
    loadStoreByteStaticStackSlot(Builder, Lo, FrameIndex, 0,
                                 MF.getMachineMemOperand(MMO, 0, 1), IsLoad);
    loadStoreByteStaticStackSlot(Builder, Hi, FrameIndex, 1,
                                 MF.getMachineMemOperand(MMO, 1, 1), IsLoad);
    if (IsLoad) {
      // Record that DestReg as a whole was set; foldMemoryOperand needs this.
      Builder.buildInstr(MOS::KILL).addDef(Reg).addUse(Lo).addUse(Hi);
    }
  } else
    loadStoreByteStaticStackSlot(Builder, Reg, FrameIndex, 0, MMO, IsLoad);

  // Users of this function expect exactly one instruction to be added.
  // However, if we're in a NoVRegs region, the only way to satisfy vregs is
  // through the register scavenger, which doesn't handle bundles.
  if (std::next(MIS.begin()) != MI &&
      !MF.getProperties().hasProperty(
          MachineFunctionProperties::Property::NoVRegs))
    finalizeBundle(MBB, MIS.begin().getInstrIterator(), MI.getInstrIterator());
}

bool MOSInstrInfo::expandPostRAPseudo(MachineInstr &MI) const {
  MachineIRBuilder Builder(MI);

  bool Changed = true;
  switch (MI.getOpcode()) {
  default:
    Changed = false;
    break;
  case MOS::LDIdx:
    expandLDIdx(Builder);
    break;
  case MOS::LDImm1:
    expandLDImm1(Builder);
    break;
  }

  return Changed;
}

void MOSInstrInfo::expandLDIdx(MachineIRBuilder &Builder) const {
  auto &MI = *Builder.getInsertPt();

  // This occur when X or Y is both the destination and index register.
  // Since the 6502 has no instruction for this, use A as the destination
  // instead, then transfer to the real destination.
  if (MI.getOperand(0).getReg() == MI.getOperand(2).getReg()) {
    bool IsAMaybeLive = isMaybeLive(Builder, MOS::A);
    if (IsAMaybeLive)
      Builder.buildInstr(MOS::PH).addUse(MOS::A);
    Builder.buildInstr(MOS::LDAIdx)
        .addDef(MOS::A)
        .add(MI.getOperand(1))
        .add(MI.getOperand(2));
    Builder.buildInstr(MOS::TA).add(MI.getOperand(0)).addUse(MOS::A);
    if (IsAMaybeLive)
      Builder.buildInstr(MOS::PL).addDef(MOS::A);
    MI.eraseFromParent();
    return;
  }

  unsigned Opcode;
  switch (MI.getOperand(0).getReg()) {
  default:
    llvm_unreachable("Bad destination for LDIdx.");
  case MOS::A:
    Opcode = MOS::LDAIdx;
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
  auto &MI = *Builder.getInsertPt();
  Register DestReg = MI.getOperand(0).getReg();

  unsigned Opcode;
  switch (DestReg) {
  default: {
    Register DestReg8 =
        Builder.getMF().getSubtarget().getRegisterInfo()->getMatchingSuperReg(
            DestReg, MOS::sublsb, &MOS::Anyi8RegClass);
    assert(DestReg8 && "Unexpected destination for LDImm1");
    Opcode = MOS::LDImm;
    MI.getOperand(0).setReg(DestReg8);
    break;
  }
  case MOS::C:
    Opcode = MOS::LDCImm;
    // Remove NZ implicit def.
    MI.RemoveOperand(2);
    break;
  case MOS::V:
    LLVM_DEBUG(dbgs() << MI);
    report_fatal_error("Not yet implemented.");
  }

  MI.setDesc(Builder.getTII().get(Opcode));
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
  static const std::pair<unsigned, const char *> Flags[] = {{MOS::MO_LO, "lo"},
                                                            {MOS::MO_HI, "hi"}};
  return Flags;
}
