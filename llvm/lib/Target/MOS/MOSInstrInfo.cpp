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

bool isMaybeLive(MachineIRBuilder &Builder, Register Reg) {
  const auto &MBB = Builder.getMBB();
  return MBB.computeRegisterLiveness(
             MBB.getParent()->getSubtarget().getRegisterInfo(), Reg,
             Builder.getInsertPt()) != MachineBasicBlock::LQR_Dead;
}

// Retrieve the first free register of a given class. If none are free,
// returns the first register in the class. Should not be used on classes
// containing reserved registers or CSRs.
Register trivialScavenge(MachineIRBuilder &Builder,
                         const TargetRegisterClass &RegClass) {
  for (Register Reg : RegClass) {
    if (Builder.getMRI()->isReserved(Reg))
      continue;
    if (!isMaybeLive(Builder, Reg))
      return Reg;
  }
  return *RegClass.begin();
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
  // Thus, instructions that only clobber NZ are always trivially
  // rematerializable.
  case MOS::LDimm:
    return true;
  }
}

MachineInstr *MOSInstrInfo::commuteInstructionImpl(MachineInstr &MI, bool NewMI,
                                                   unsigned Idx1,
                                                   unsigned Idx2) const {
  // TODO: A version of this that doesn't modify register classes if NewMI.
  if (NewMI)
    report_fatal_error("Not yet implemented.");

  MachineFunction &MF = *MI.getMF();
  const TargetRegisterInfo &TRI = *MF.getSubtarget().getRegisterInfo();
  MachineRegisterInfo &MRI = MF.getRegInfo();

  switch (MI.getOpcode()) {
  default:
    LLVM_DEBUG(dbgs() << "Commute: " << MI);
    llvm_unreachable("Unexpected instruction commute.");
  case MOS::ADCimag8:
    break;
  }

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

bool MOSInstrInfo::findCommutedOpIndices(const MachineInstr &MI,
                                         unsigned &SrcOpIdx1,
                                         unsigned &SrcOpIdx2) const {
  assert(!MI.isBundle() &&
         "MOSInstrInfo::findCommutedOpIndices() can't handle bundles");

  const MCInstrDesc &MCID = MI.getDesc();
  if (!MCID.isCommutable())
    return false;

  assert(MI.getOpcode() == MOS::ADCimag8);

  if (!fixCommutedOpIndices(SrcOpIdx1, SrcOpIdx2, 2, 3))
    return false;

  if (!MI.getOperand(SrcOpIdx1).isReg() || !MI.getOperand(SrcOpIdx2).isReg())
    // No idea.
    return false;
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

  // No terminators, so falls through.
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

  // If more than two branches present, cannot analyze.
  if (I != MBB.end())
    return true;

  // Exactly two branches present.

  // Can only analyze conditional branch followed by unconditional branch.
  if (!SecondBR->isUnconditionalBranch())
    return true;

  // Second unconditional branch forms false edge.
  if (SecondBR->isPreISelOpcode())
    return true;
  FBB = getBranchDestBlock(*SecondBR);
  return false;
}

unsigned MOSInstrInfo::removeBranch(MachineBasicBlock &MBB,
                                    int *BytesRemoved) const {
  // Since analyzeBranch succeeded, we know that the only terminators are
  // comparisons and branches.

  auto Begin = MBB.getFirstTerminator();
  auto End = MBB.end();
  while (Begin != End && Begin->isCompare())
    ++Begin;

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
    assert(Cond.size() == 2);

    // The unconditional branch will be to the false branch (if any).
    UBB = FBB;

    auto BR = Builder.buildInstr(MOS::BR).addMBB(TBB);
    for (const MachineOperand &Op : Cond) {
      BR.add(Op);
    }
    ++NumAdded;
    if (BytesAdded)
      *BytesAdded += getInstSizeInBytes(*BR);
  }

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
  preserveAroundPseudoExpansion(
      Builder, [&]() { copyPhysRegNoPreserve(Builder, DestReg, SrcReg); });
}

void MOSInstrInfo::copyPhysRegNoPreserve(MachineIRBuilder &Builder,
                                         MCRegister DestReg,
                                         MCRegister SrcReg) const {
  if (DestReg == SrcReg)
    return;

  const TargetRegisterInfo &TRI =
      *Builder.getMF().getSubtarget().getRegisterInfo();

  const auto &areClasses = [&](const TargetRegisterClass &Dest,
                               const TargetRegisterClass &Src) {
    return Dest.contains(DestReg) && Src.contains(SrcReg);
  };

  if (areClasses(MOS::GPRRegClass, MOS::GPRRegClass)) {
    if (SrcReg == MOS::A) {
      assert(MOS::XYRegClass.contains(DestReg));
      Builder.buildInstr(MOS::TA_).addDef(DestReg);
    } else if (DestReg == MOS::A) {
      assert(MOS::XYRegClass.contains(SrcReg));
      Builder.buildInstr(MOS::T_A).addUse(SrcReg);
    } else {
      copyPhysRegNoPreserve(Builder, MOS::A, SrcReg);
      copyPhysRegNoPreserve(Builder, DestReg, MOS::A);
    }
  } else if (areClasses(MOS::Imag8RegClass, MOS::GPRRegClass)) {
    Builder.buildInstr(MOS::STimag8).addDef(DestReg).addUse(SrcReg);
  } else if (areClasses(MOS::GPRRegClass, MOS::Imag8RegClass)) {
    Builder.buildInstr(MOS::LDimag8).addDef(DestReg).addUse(SrcReg);
  } else if (areClasses(MOS::Imag16RegClass, MOS::Imag16RegClass)) {
    copyPhysRegNoPreserve(Builder, TRI.getSubReg(DestReg, MOS::sublo),
                          TRI.getSubReg(SrcReg, MOS::sublo));
    copyPhysRegNoPreserve(Builder, TRI.getSubReg(DestReg, MOS::subhi),
                          TRI.getSubReg(SrcReg, MOS::subhi));
  } else if (areClasses(MOS::Imag8RegClass, MOS::Imag8RegClass)) {
    Register Tmp = trivialScavenge(Builder, MOS::GPRRegClass);
    copyPhysRegNoPreserve(Builder, Tmp, SrcReg);
    copyPhysRegNoPreserve(Builder, DestReg, Tmp);
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

static void loadStoreByteStaticStackSlot(MachineIRBuilder &Builder,
                                         Register Reg, int FrameIndex,
                                         int64_t Offset, MachineMemOperand *MMO,
                                         bool IsLoad) {
  Register Tmp = Reg;
  if (!MOS::GPRRegClass.contains(Tmp))
    Tmp = Builder.getMRI()->createVirtualRegister(&MOS::GPRRegClass);

  // Get the value from wherever it's coming from to Tmp.
  if (IsLoad) {
    Builder.buildInstr(MOS::LDabs_offset, {Tmp}, {})
        .addFrameIndex(FrameIndex)
        .addImm(Offset)
        .addMemOperand(MMO);

    if (Reg == Tmp) {
    } else if (MOS::Imag8RegClass.contains(Reg))
      Builder.buildInstr(MOS::STimag8, {Reg}, {Tmp});
    else
      report_fatal_error("Not yet implemented.");
  } else {
    if (Reg == Tmp) {
    } else if (MOS::Imag8RegClass.contains(Reg))
      Builder.buildInstr(MOS::LDimag8, {Tmp}, {Reg});
    else
      report_fatal_error("Not yet implemented.");

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

  // Since the offset is not yet known, it may be either 8 or 16 bits. Emit a
  // 16-bit pseudo to be lowered during frame index elimination.
  if (!MI->getMF()->getFunction().doesNotRecurse()) {
    // Note: This can never occur in PEI, since PEI only loads/stores CSRs, and
    // those are custom-spilled to the hard stack in non-recursive functions.
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
      !MI->getMF()->getProperties().hasProperty(
          MachineFunctionProperties::Property::NoVRegs))
    finalizeBundle(MBB, MIS.begin().getInstrIterator(), MI.getInstrIterator());
}

bool MOSInstrInfo::expandPostRAPseudo(MachineInstr &MI) const {
  bool Changed;
  MachineIRBuilder Builder(MI);
  preserveAroundPseudoExpansion(
      Builder, [&]() { Changed = expandPostRAPseudoNoPreserve(Builder); });
  return Changed;
}

bool MOSInstrInfo::expandPostRAPseudoNoPreserve(
    MachineIRBuilder &Builder) const {
  MachineInstrSpan Span(Builder.getInsertPt(), &Builder.getMBB());
  auto &MI = *Builder.getInsertPt();

  bool Changed = true;
  switch (MI.getOpcode()) {
  default:
    Changed = false;
    break;
  case MOS::AddrLostk:
    expandAddrLostk(Builder);
    break;
  case MOS::AddrHistk:
    expandAddrHistk(Builder);
    break;
  case MOS::IncSP:
    expandIncSP(Builder);
    break;
  case MOS::LDstk:
  case MOS::STstk:
    expandLDSTstk(Builder);
    break;
  case MOS::LDidx:
    expandLDidx(Builder);
    break;
  }

  if (Changed) {
    Builder.setInsertPt(Builder.getMBB(), Span.begin());
    while (Builder.getInsertPt() != Span.getInitial()) {
      expandPostRAPseudoNoPreserve(Builder);
    }
    Builder.setInsertPt(Builder.getMBB(), std::next(Builder.getInsertPt()));
    MI.eraseFromParent();
  } else
    Builder.setInsertPt(Builder.getMBB(), std::next(Builder.getInsertPt()));
  return Changed;
}

void MOSInstrInfo::expandAddrLostk(MachineIRBuilder &Builder) const {
  auto &MI = *Builder.getInsertPt();
  assert(MI.getOpcode() == MOS::AddrLostk);
  const TargetRegisterInfo &TRI =
      *Builder.getMF().getSubtarget().getRegisterInfo();

  Register Dst = MI.getOperand(0).getReg();
  Register Base = MI.getOperand(2).getReg();

  int64_t OffsetImm = MI.getOperand(3).getImm();
  assert(0 <= OffsetImm && OffsetImm < 65536);
  auto Offset = static_cast<uint16_t>(OffsetImm);

  Register Src;
  switch (Base) {
  case MOS::Static: {
    Src = Dst;
    if (!MOS::GPRRegClass.contains(Src))
      Src = trivialScavenge(Builder, MOS::GPRRegClass);
    Builder.buildInstr(MOS::LDimm)
        .addDef(Src)
        .addTargetIndex(MOS::TI_STATIC_STACK, Offset, MOS::MO_LO);
    Offset = 0;
    break;
  }
  default:
    assert(MOS::Imag16RegClass.contains(Base));
    Src = TRI.getSubReg(Base, MOS::sublo);
    break;
  }

  Offset &= 0xFF;

  if (!Offset) {
    copyPhysRegNoPreserve(Builder, Dst, Src);
    return;
  }

  copyPhysRegNoPreserve(Builder, MOS::A, Src);
  Builder.buildInstr(MOS::LDCimm).addDef(MOS::C).addImm(0);
  Builder.buildInstr(MOS::ADCimm)
      .addDef(MOS::A)
      .addDef(MOS::C)
      .addUse(MOS::A)
      .addImm(Offset)
      .addUse(MOS::C);
  copyPhysRegNoPreserve(Builder, Dst, MOS::A);
}

void MOSInstrInfo::expandAddrHistk(MachineIRBuilder &Builder) const {
  auto &MI = *Builder.getInsertPt();
  assert(MI.getOpcode() == MOS::AddrHistk);
  const TargetRegisterInfo &TRI =
      *Builder.getMF().getSubtarget().getRegisterInfo();
  Register Dst = MI.getOperand(0).getReg();
  Register Base = MI.getOperand(1).getReg();

  int64_t OffsetImm = MI.getOperand(2).getImm();
  assert(0 <= OffsetImm && OffsetImm < 65536);
  auto Offset = static_cast<uint16_t>(OffsetImm);

  if (Base == MOS::Static) {
    Register Tmp = Dst;
    if (!MOS::GPRRegClass.contains(Tmp))
      Tmp = trivialScavenge(Builder, MOS::GPRRegClass);
    Builder.buildInstr(MOS::LDimm)
        .addDef(Tmp)
        .addTargetIndex(MOS::TI_STATIC_STACK, Offset, MOS::MO_HI);
    copyPhysRegNoPreserve(Builder, Dst, Tmp);
    return;
  }

  if (!Offset) {
    if (MOS::Imag16RegClass.contains(Base)) {
      copyPhysRegNoPreserve(Builder, Dst, TRI.getSubReg(Base, MOS::subhi));
    } else {
      Register Tmp = Dst;
      if (!MOS::GPRRegClass.contains(Tmp))
        Tmp = trivialScavenge(Builder, MOS::GPRRegClass);
      Builder.buildInstr(MOS::LDimm).addDef(Tmp).addImm(1);
      copyPhysRegNoPreserve(Builder, Dst, Tmp);
    }
    return;
  }

  if (MOS::Imag16RegClass.contains(Base)) {
    Builder.buildInstr(MOS::LDimag8)
        .addDef(MOS::A)
        .addUse(TRI.getSubReg(Base, MOS::subhi));
  } else {
    // The stack page begins at 0x0100
    Builder.buildInstr(MOS::LDimm).addDef(MOS::A).addImm(1);
  }

  // AddrLostk won't reset the carry if it has a zero offset.
  if (!(Offset & 0xFF))
    Builder.buildInstr(MOS::LDCimm).addDef(MOS::C).addImm(0);
  Builder.buildInstr(MOS::ADCimm)
      .addDef(MOS::A)
      .addDef(MOS::C, RegState::Dead)
      .addUse(MOS::A)
      .addImm(Offset >> 8)
      .addUse(MOS::C);
  copyPhysRegNoPreserve(Builder, Dst, MOS::A);
}

void MOSInstrInfo::expandLDSTstk(MachineIRBuilder &Builder) const {
  auto &MI = *Builder.getInsertPt();
  const TargetRegisterInfo &TRI =
      *Builder.getMF().getSubtarget().getRegisterInfo();

  Register Loc = MI.getOperand(0).getReg();
  Register Base = MI.getOperand(1).getReg();
  int64_t Offset = MI.getOperand(2).getImm();
  assert(0 <= Offset && Offset < 65536);

  bool IsLoad;
  switch (MI.getOpcode()) {
  default:
    llvm_unreachable("Unexpected opcode.");
  case MOS::LDstk:
    IsLoad = true;
    break;
  case MOS::STstk:
    IsLoad = false;
    break;
  }

  if (Offset >= 256) {
    // FIXME: Have this find a register other than Loc, if it's available.
    // Really feeling the pains of not using VRegs for this.
    Register Tmp = trivialScavenge(Builder, MOS::Imag16RegClass);

    // Guarantee that Tmp is different than Loc, even if it requires
    // save/restore. RS3 is used in lieu of RS1, since RS2 is callee-saved, and
    // this is called after PEI (when those registers are handled).
    if (TRI.isSubRegisterEq(Loc, Tmp)) {
      Tmp = Tmp == MOS::RS1 ? MOS::RS3 : MOS::RS0;
    }

    // Move the high byte of the offset into the base address.
    Builder.buildInstr(MOS::AddrLostk,
                       {Register(TRI.getSubReg(Tmp, MOS::sublo)), MOS::C},
                       {Base, Offset & 0xFF00});
    Builder.buildInstr(MOS::AddrHistk,
                       {Register(TRI.getSubReg(Tmp, MOS::subhi))},
                       {Base, Offset & 0xFF00, Register(MOS::C)});

    Base = Tmp;
    Offset &= 0xFF;
  }

  if (MOS::Imag16RegClass.contains(Loc)) {
    Builder.buildInstr(MI.getOpcode())
        .addReg(TRI.getSubReg(Loc, MOS::sublo), getDefRegState(IsLoad))
        .addUse(Base)
        .addImm(Offset);
    Builder.buildInstr(MI.getOpcode())
        .addReg(TRI.getSubReg(Loc, MOS::subhi), getDefRegState(IsLoad))
        .addUse(Base)
        .addImm(Offset + 1);
  } else {
    assert(MOS::Imag16RegClass.contains(Base));
    Builder.buildInstr(MOS::LDimm).addDef(MOS::Y).addImm(Offset);
    if (!IsLoad)
      copyPhysRegNoPreserve(Builder, MOS::A, Loc);
    Builder.buildInstr(IsLoad ? MOS::LDyindir : MOS::STyindir)
        .addReg(MOS::A, getDefRegState(IsLoad))
        .addUse(Base)
        .addUse(MOS::Y);
    if (IsLoad)
      copyPhysRegNoPreserve(Builder, Loc, MOS::A);
  }
}

void MOSInstrInfo::expandIncSP(MachineIRBuilder &Builder) const {
  auto &MI = *Builder.getInsertPt();
  assert(MI.getOpcode() == MOS::IncSP);

  int64_t BytesImm = MI.getOperand(0).getImm();
  assert(BytesImm);
  assert(-32768 <= BytesImm && BytesImm < 32768);
  auto Bytes = static_cast<uint16_t>(BytesImm);
  auto LoBytes = Bytes & 0xFF;
  auto HiBytes = Bytes >> 8;
  assert(LoBytes || HiBytes);

  Builder.buildInstr(MOS::LDCimm).addDef(MOS::C).addImm(0);
  if (LoBytes) {
    Builder.buildInstr(MOS::LDimm).addDef(MOS::A).addImm(LoBytes);
    Builder.buildInstr(MOS::ADCimag8)
        .addDef(MOS::A)
        .addDef(MOS::C)
        .addUse(MOS::A)
        .addUse(MOS::RC0)
        .addUse(MOS::C);
    Builder.buildInstr(MOS::STimag8).addDef(MOS::RC0).addUse(MOS::A);
  }
  Builder.buildInstr(MOS::LDimm).addDef(MOS::A).addImm(HiBytes);
  Builder.buildInstr(MOS::ADCimag8)
      .addDef(MOS::A)
      .addDef(MOS::C, RegState::Dead)
      .addUse(MOS::A)
      .addUse(MOS::RC1)
      .addUse(MOS::C);
  Builder.buildInstr(MOS::STimag8).addDef(MOS::RC1).addUse(MOS::A);
}

void MOSInstrInfo::expandLDidx(MachineIRBuilder &Builder) const {
  auto &MI = *Builder.getInsertPt();
  assert(MI.getOpcode() == MOS::LDidx);

  // This occur when X or Y is both the destination and index register.
  // Since the 6502 has no instruction for this, use A as the destination
  // instead, then transfer to the real destination.
  if (MI.getOperand(0).getReg() == MI.getOperand(2).getReg()) {
    Builder.buildInstr(MOS::LDAidx).add(MI.getOperand(1)).add(MI.getOperand(2));
    Builder.buildInstr(MOS::TA_).add(MI.getOperand(0));
    return;
  }

  switch (MI.getOperand(0).getReg()) {
  default:
    llvm_unreachable("Bad destination for LDidx.");
  case MOS::A:
    Builder.buildInstr(MOS::LDAidx).add(MI.getOperand(1)).add(MI.getOperand(2));
    break;
  case MOS::X:
    Builder.buildInstr(MOS::LDXidx).add(MI.getOperand(1));
    break;
  case MOS::Y:
    Builder.buildInstr(MOS::LDYidx).add(MI.getOperand(1));
    break;
  }
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

void MOSInstrInfo::preserveAroundPseudoExpansion(
    MachineIRBuilder &Builder, std::function<void()> ExpandFn) const {
  MachineBasicBlock &MBB = Builder.getMBB();
  const TargetRegisterInfo &TRI =
      *MBB.getParent()->getSubtarget().getRegisterInfo();
  const MachineRegisterInfo &MRI = *Builder.getMRI();

  // Returns the locations modified by the given instruction.
  const auto GetWrites = [&](MachineInstr &MI) {
    SparseBitVector<> Writes;
    for (unsigned Reg = MCRegister::FirstPhysicalReg; Reg < TRI.getNumRegs();
         ++Reg) {
      if (MRI.isReserved(Reg))
        continue;
      if (MI.definesRegister(Reg, &TRI))
        Writes.set(Reg);
    }
    return Writes;
  };

  SparseBitVector<> MaybeLive;
  for (unsigned Reg = MCRegister::FirstPhysicalReg; Reg < TRI.getNumRegs();
       ++Reg) {
    if (MRI.isReserved(Reg))
      continue;
    if (isMaybeLive(Builder, Reg))
      MaybeLive.set(Reg);
  }

  SparseBitVector<> ExpectedWrites = GetWrites(*Builder.getInsertPt());

  // If begin was the first instruction, it may no longer be the first once
  // ExpandFn is called, so make a note of it.
  auto Begin = Builder.getInsertPt();
  bool WasBegin = Begin == MBB.begin();
  // Have begin point at the instruction before the inserted range.
  if (!WasBegin)
    --Begin;

  ExpandFn();

  // If Begin was the first instruction, get the real first instruction now
  // that ExpandFn has been called. Otherwise, advance Begin to the first
  // instruction.
  if (WasBegin)
    Begin = MBB.begin();
  else
    ++Begin;
  auto End = Builder.getInsertPt();

  // Determine the writes of the expansion region.
  SparseBitVector<> Writes;
  for (auto I = Begin; I != End; ++I)
    Writes |= GetWrites(*I);

  SparseBitVector<> Save = MaybeLive;
  Save &= Writes;
  Save.intersectWithComplement(ExpectedWrites);

  const auto RecordSaved = [&](Register Reg) {
    for (MCSubRegIterator SubReg(Reg, &TRI, /*IncludeSelf=*/true);
         SubReg.isValid(); ++SubReg) {
      Save.reset(*SubReg);
    }
  };

  // This code is intentionally very simplistic: that way it's easy to verify
  // that it's complete. Eventually, more efficient PHA/PLA can be emitted in
  // situations that can be proven safe. Ideally, saving/restoring here should
  // be rare; especially if save/restores are elided as a post-processing
  // step. Thus, having a simple working version of this forms a good baseline
  // that the rest of the compiler can rely on.

  // After the save sequence, issued, all registers and flags are in the same
  // state as before the sequence. After the restore sequence, all registers
  // and flags are in the same state as before the sequence, except those that
  // were saved, which have their value at time of save. The only memory
  // locations affected by either are _Save<Reg>.

  // Only RS1 (RC2 and RC3) and RS3 (RC6 and RC7) are allowed to be used, being
  // the first two non-SP caller-saved registers.

  // FIXME: This won't work if there's only only 3 zero-page pointers, which is
  // currently allowed. A pseudo expansion may not be able to use RS1 and try to
  // use RS3, which the compiler cannot safely save/restore (it may be used by
  // an OS interrupt handler). All of this code smells awful though, so consider
  // finding a way to remove this whole system instead of fixing this specific
  // issue.

  Builder.setInsertPt(MBB, Begin);
  if (Save.test(MOS::C)) {
    Builder.buildInstr(MOS::PHA);
    Builder.buildInstr(MOS::PHP);
    Builder.buildInstr(MOS::PLA);
    Builder.buildInstr(MOS::STabs).addUse(MOS::A).addExternalSymbol("_SaveP");
    Builder.buildInstr(MOS::PLA);
  }
  if (Save.test(MOS::A))
    Builder.buildInstr(MOS::STabs).addUse(MOS::A).addExternalSymbol("_SaveA");
  if (Save.test(MOS::X))
    Builder.buildInstr(MOS::STabs).addUse(MOS::X).addExternalSymbol("_SaveX");
  if (Save.test(MOS::Y))
    Builder.buildInstr(MOS::STabs).addUse(MOS::Y).addExternalSymbol("_SaveY");
  if (Save.test(MOS::RC2)) {
    assert(!Save.test(MOS::RC6));
    Builder.buildInstr(MOS::PHP);
    Builder.buildInstr(MOS::PHA);
    Builder.buildInstr(MOS::LDimag8).addDef(MOS::A).addUse(MOS::RC2);
    Builder.buildInstr(MOS::STabs)
        .addUse(MOS::A)
        .addExternalSymbol("_SaveImagLo");
    Builder.buildInstr(MOS::PLA);
    Builder.buildInstr(MOS::PLP);
  }
  if (Save.test(MOS::RC3)) {
    assert(!Save.test(MOS::RC7));
    Builder.buildInstr(MOS::PHP);
    Builder.buildInstr(MOS::PHA);
    Builder.buildInstr(MOS::LDimag8).addDef(MOS::A).addUse(MOS::RC3);
    Builder.buildInstr(MOS::STabs)
        .addUse(MOS::A)
        .addExternalSymbol("_SaveImagHi");
    Builder.buildInstr(MOS::PLA);
    Builder.buildInstr(MOS::PLP);
  }
  if (Save.test(MOS::RC6)) {
    assert(!Save.test(MOS::RC2));
    Builder.buildInstr(MOS::PHP);
    Builder.buildInstr(MOS::PHA);
    Builder.buildInstr(MOS::LDimag8).addDef(MOS::A).addUse(MOS::RC6);
    Builder.buildInstr(MOS::STabs)
        .addUse(MOS::A)
        .addExternalSymbol("_SaveImagLo");
    Builder.buildInstr(MOS::PLA);
    Builder.buildInstr(MOS::PLP);
  }
  if (Save.test(MOS::RC7)) {
    assert(!Save.test(MOS::RC3));
    Builder.buildInstr(MOS::PHP);
    Builder.buildInstr(MOS::PHA);
    Builder.buildInstr(MOS::LDimag8).addDef(MOS::A).addUse(MOS::RC7);
    Builder.buildInstr(MOS::STabs)
        .addUse(MOS::A)
        .addExternalSymbol("_SaveImagHi");
    Builder.buildInstr(MOS::PLA);
    Builder.buildInstr(MOS::PLP);
  }

  Builder.setInsertPt(MBB, End);
  if (Save.test(MOS::C)) {
    // Note: This is particularly awful due to the requirement that the last
    // operation be a PLP. This means we have to get P onto the stack behind
    // the values of any registers that need to be saved to do so; hence the
    // indexed store behind the saves of X and A. That way, we can restore X
    // and A *before* P, preventing those restores from clobbering NZ.
    Builder.buildInstr(MOS::PHA);
    Builder.buildInstr(MOS::PHA);
    Builder.buildInstr(MOS::T_A).addUse(MOS::X, RegState::Undef);
    Builder.buildInstr(MOS::PHA);
    Builder.buildInstr(MOS::TSX);
    Builder.buildInstr(MOS::LDabs).addDef(MOS::A).addExternalSymbol("_SaveP");
    Builder.buildInstr(MOS::STidx)
        .addUse(MOS::A)
        .addImm(0x103) // Byte pushed by first PHA.
        .addUse(MOS::X);
    Builder.buildInstr(MOS::PLA);
    Builder.buildInstr(MOS::TA_).addDef(MOS::X);
    Builder.buildInstr(MOS::PLA);
    Builder.buildInstr(MOS::PLP);
    RecordSaved(MOS::P);
  }
  if (Save.test(MOS::A)) {
    Builder.buildInstr(MOS::PHP);
    Builder.buildInstr(MOS::LDabs).addDef(MOS::A).addExternalSymbol("_SaveA");
    Builder.buildInstr(MOS::PLP);
    RecordSaved(MOS::A);
  }
  if (Save.test(MOS::X)) {
    Builder.buildInstr(MOS::PHP);
    Builder.buildInstr(MOS::LDabs).addDef(MOS::X).addExternalSymbol("_SaveX");
    Builder.buildInstr(MOS::PLP);
    RecordSaved(MOS::X);
  }
  if (Save.test(MOS::Y)) {
    Builder.buildInstr(MOS::PHP);
    Builder.buildInstr(MOS::LDabs).addDef(MOS::Y).addExternalSymbol("_SaveY");
    Builder.buildInstr(MOS::PLP);
    RecordSaved(MOS::Y);
  }
  if (Save.test(MOS::RC2)) {
    Builder.buildInstr(MOS::PHP);
    Builder.buildInstr(MOS::PHA);
    Builder.buildInstr(MOS::LDabs)
        .addDef(MOS::A)
        .addExternalSymbol("_SaveImagLo");
    Builder.buildInstr(MOS::STimag8).addDef(MOS::RC2).addUse(MOS::A);
    Builder.buildInstr(MOS::PLA);
    Builder.buildInstr(MOS::PLP);
  }
  if (Save.test(MOS::RC3)) {
    Builder.buildInstr(MOS::PHP);
    Builder.buildInstr(MOS::PHA);
    Builder.buildInstr(MOS::LDabs)
        .addDef(MOS::A)
        .addExternalSymbol("_SaveImagHi");
    Builder.buildInstr(MOS::STimag8).addDef(MOS::RC3).addUse(MOS::A);
    Builder.buildInstr(MOS::PLA);
    Builder.buildInstr(MOS::PLP);
  }
  if (Save.test(MOS::RC6)) {
    Builder.buildInstr(MOS::PHP);
    Builder.buildInstr(MOS::PHA);
    Builder.buildInstr(MOS::LDabs)
        .addDef(MOS::A)
        .addExternalSymbol("_SaveImagLo");
    Builder.buildInstr(MOS::STimag8).addDef(MOS::RC6).addUse(MOS::A);
    Builder.buildInstr(MOS::PLA);
    Builder.buildInstr(MOS::PLP);
  }
  if (Save.test(MOS::RC7)) {
    Builder.buildInstr(MOS::PHP);
    Builder.buildInstr(MOS::PHA);
    Builder.buildInstr(MOS::LDabs)
        .addDef(MOS::A)
        .addExternalSymbol("_SaveImagHi");
    Builder.buildInstr(MOS::STimag8).addDef(MOS::RC7).addUse(MOS::A);
    Builder.buildInstr(MOS::PLA);
    Builder.buildInstr(MOS::PLP);
  }

  if (Save.count()) {
    for (Register Reg : Save)
      LLVM_DEBUG(dbgs() << "Unhandled saved register: " << TRI.getName(Reg)
                        << "\n");

    LLVM_DEBUG(dbgs() << "MaybeLive:\n");
    for (Register Reg : MaybeLive)
      LLVM_DEBUG(dbgs() << TRI.getName(Reg) << "\n");

    LLVM_DEBUG(dbgs() << "Writes:\n");
    for (Register Reg : Writes)
      LLVM_DEBUG(dbgs() << TRI.getName(Reg) << "\n");

    LLVM_DEBUG(dbgs() << "Expected Writes:\n");
    for (Register Reg : ExpectedWrites)
      LLVM_DEBUG(dbgs() << TRI.getName(Reg) << "\n");

    report_fatal_error("Cannot yet preserve register.");
  }
}
