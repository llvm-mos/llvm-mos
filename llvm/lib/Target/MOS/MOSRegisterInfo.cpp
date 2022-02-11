//===-- MOSRegisterInfo.cpp - MOS Register Information --------------------===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains the MOS implementation of the TargetRegisterInfo class.
//
//===----------------------------------------------------------------------===//

#include "MOSRegisterInfo.h"
#include "MCTargetDesc/MOSMCTargetDesc.h"
#include "MOS.h"
#include "MOSFrameLowering.h"
#include "MOSInstrInfo.h"
#include "MOSSubtarget.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineOperand.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/TargetFrameLowering.h"
#include "llvm/CodeGen/TargetInstrInfo.h"
#include "llvm/CodeGen/TargetRegisterInfo.h"
#include "llvm/IR/CallingConv.h"
#include "llvm/MC/MCRegisterInfo.h"
#include "llvm/Support/ErrorHandling.h"

#define DEBUG_TYPE "mos-reginfo"

#define GET_REGINFO_TARGET_DESC
#include "MOSGenRegisterInfo.inc"

using namespace llvm;

MOSRegisterInfo::MOSRegisterInfo()
    : MOSGenRegisterInfo(/*RA=*/0, /*DwarfFlavor=*/0, /*EHFlavor=*/0,
                         /*PC=*/0, /*HwMode=*/0),
      Imag8SymbolNames(new std::string[getNumRegs()]), Reserved(getNumRegs()) {
  for (unsigned Reg : seq(0u, getNumRegs())) {
    // Pointers are referred to by their low byte in the addressing modes that
    // use them.
    unsigned R = Reg;
    if (MOS::Imag16RegClass.contains(R))
      R = getSubReg(R, MOS::sublo);
    if (!MOS::Imag8RegClass.contains(R))
      continue;
    std::string &Str = Imag8SymbolNames[Reg];
    Str = "__";
    Str += getName(R);
    std::transform(Str.begin(), Str.end(), Str.begin(), ::tolower);
  }

  // Reserve all imaginary registers beyond the number allowed to the compiler.
  for (Register Ptr : enum_seq_inclusive(MOS::RS16, MOS::RS127))
    reserveAllSubregs(&Reserved, Ptr);

  // Reserve stack pointers.
  reserveAllSubregs(&Reserved, MOS::RS0);

  // Reserve one temporary register for use by register scavenger.
  reserveAllSubregs(&Reserved, MOS::RS8);
}

const MCPhysReg *
MOSRegisterInfo::getCalleeSavedRegs(const MachineFunction *MF) const {
  const MOSFrameLowering &TFI = *getFrameLowering(*MF);
  return TFI.isISR(*MF) ? MOS_Interrupt_CSR_SaveList : MOS_CSR_SaveList;
}

const uint32_t *
MOSRegisterInfo::getCallPreservedMask(const MachineFunction &MF,
                                      CallingConv::ID CallingConv) const {
  return MOS_CSR_RegMask;
}

BitVector MOSRegisterInfo::getReservedRegs(const MachineFunction &MF) const {
  const TargetFrameLowering *TFI = getFrameLowering(MF);
  BitVector Reserved = this->Reserved;
  if (TFI->hasFP(MF))
    reserveAllSubregs(&Reserved, getFrameRegister(MF));
  return Reserved;
}

const TargetRegisterClass *
MOSRegisterInfo::getLargestLegalSuperClass(const TargetRegisterClass *RC,
                                           const MachineFunction &) const {
  if (RC->hasSuperClass(&MOS::Anyi1RegClass))
    return &MOS::Anyi1RegClass;
  if (RC->hasSuperClass(&MOS::Anyi8RegClass))
    return &MOS::Anyi8RegClass;
  return RC;
}

const TargetRegisterClass *
MOSRegisterInfo::getCrossCopyRegClass(const TargetRegisterClass *RC) const {
  if (RC == &MOS::Imag8RegClass)
    return &MOS::GPRRegClass;
  if (RC == &MOS::YcRegClass || RC == &MOS::XYRegClass)
    return &MOS::AImag8RegClass;
  return RC;
}

// These values were chosen empirically based on the desired behavior of llc
// test cases. These values will likely need to be retuned as more examples come
// up.  Unfortunately, the way the register allocator actually uses this is very
// heuristic, and if tuning these params doesn't suffice, we'll need to build a
// more sophisticated analysis into the register allocator.
unsigned MOSRegisterInfo::getCSRFirstUseCost(const MachineFunction &MF) const {
  if (MF.getFunction().doesNotRecurse()) {
    return 15 * 16384 / 10;
  }
  return 5 * 16384 / 10;
}

static bool pushPullBalanced(MachineBasicBlock::iterator Begin,
                             MachineBasicBlock::iterator End) {
  int64_t PushCount = 0;
  for (const MachineInstr &MI : make_range(Begin, End)) {
    switch (MI.getOpcode()) {
    case MOS::PH:
      ++PushCount;
      break;
    case MOS::PL:
      if (!PushCount)
        return false;
      --PushCount;
      break;
    }
  }
  return !PushCount;
}

bool MOSRegisterInfo::saveScavengerRegister(MachineBasicBlock &MBB,
                                            MachineBasicBlock::iterator I,
                                            MachineBasicBlock::iterator &UseMI,
                                            const TargetRegisterClass *RC,
                                            Register Reg) const {

  // Note: NZ cannot be live at this point, since it's only live in terminators,
  // and virtual registers are never inserted into terminators.

  // Consider the regions in a basic block where a physical register is live.
  // The register scavenger will select one of these regions to spill and mark
  // the physical register as available within that region. Such a region cannot
  // contain any calls, since the physical registers are clobbered by calls.
  // This means that a save/restore pair for that physical register cannot
  // overlap with any other save/restore pair for the same physical register.

  MachineIRBuilder Builder(MBB, I);
  const MOSSubtarget &STI = Builder.getMF().getSubtarget<MOSSubtarget>();

  switch (Reg) {
  default:
    errs() << "Register: " << getName(Reg) << "\n";
    report_fatal_error("Scavenger spill for register not yet implemented.");
  case MOS::A:
  case MOS::Y: {
    // RS8 is reserved to save A and Y if necessary, but pushing is still
    // preferred.
    Register Save = Reg == MOS::A ? MOS::RC16 : MOS::RC17;
    bool UseHardStack =
        (Reg == MOS::A || STI.has65C02()) && pushPullBalanced(I, UseMI);

    if (UseHardStack)
      Builder.buildInstr(MOS::PH, {}, {Reg});
    else
      Builder.buildInstr(MOS::STImag8, {Save}, {Reg});

    Builder.setInsertPt(MBB, UseMI);

    if (UseHardStack)
      Builder.buildInstr(MOS::PL, {Reg}, {});
    else
      Builder.buildInstr(MOS::LDImag8, {Reg}, {Save});
    break;
  }
  }

  return true;
}

bool MOSRegisterInfo::canSaveScavengerRegister(Register Reg) const {
  return Reg != MOS::X;
}

void MOSRegisterInfo::eliminateFrameIndex(MachineBasicBlock::iterator MI,
                                          int SPAdj, unsigned FIOperandNum,
                                          RegScavenger *RS) const {
  MachineFunction &MF = *MI->getMF();
  const MachineFrameInfo &MFI = MF.getFrameInfo();

  assert(!SPAdj);

  int Idx = MI->getOperand(FIOperandNum).getIndex();
  int64_t Offset = MFI.getObjectOffset(Idx);
  if (FIOperandNum + 1 < MI->getNumOperands() &&
      MI->getOperand(FIOperandNum + 1).isImm())
    Offset += MI->getOperand(FIOperandNum + 1).getImm();
  else
    Offset += MI->getOperand(FIOperandNum).getOffset();

  if (MFI.getStackID(Idx) == TargetStackID::Default) {
    // All offsets are relative to the incoming SP
    // 1) Addr = Offset_SP + SP
    //
    // However, the incoming SP isn't available throughout the function; only
    // the frame pointer is. So we need to obtain the FP relative offset such
    // that:
    // 2) Addr = Offset_FP + FP
    //
    // Susbtituting (2) into (1) gives:
    // 3) Offset_FP = Offset_SP + SP - FP
    //
    // The frame pointer is:
    // 4) FP = SP - Stack_Size
    //
    // Substituting (4) into (3) gives:
    // 5) Offset_FP = Offset_SP + Stack_Size
    Offset += MFI.getStackSize();
  }

  switch (MI->getOpcode()) {
  default:
    MI->getOperand(FIOperandNum)
        .ChangeToTargetIndex(MOS::TI_STATIC_STACK, Offset,
                             MI->getOperand(FIOperandNum).getTargetFlags());
    break;
  case MOS::AddrLostk:
  case MOS::AddrHistk:
  case MOS::LDStk:
  case MOS::STStk:
    MI->getOperand(FIOperandNum)
        .ChangeToRegister(getFrameRegister(MF), /*isDef=*/false);
    MI->getOperand(FIOperandNum + 1).setImm(Offset);
    break;
  }

  switch (MI->getOpcode()) {
  default:
    break;
  case MOS::AddrLostk:
    expandAddrLostk(MI);
    break;
  case MOS::AddrHistk:
    expandAddrHistk(MI);
    break;
  case MOS::LDStk:
  case MOS::STStk:
    expandLDSTStk(MI);
    break;
  }
}

void MOSRegisterInfo::expandAddrLostk(MachineBasicBlock::iterator MI) const {
  MachineIRBuilder Builder(*MI->getParent(), MI);
  const TargetRegisterInfo &TRI =
      *Builder.getMF().getSubtarget().getRegisterInfo();

  const MachineOperand &Dst = MI->getOperand(0);
  Register Base = MI->getOperand(3).getReg();
  const MachineOperand &CDef = MI->getOperand(1);
  const MachineOperand &VDef = MI->getOperand(2);

  int64_t OffsetImm = MI->getOperand(4).getImm();
  assert(0 <= OffsetImm && OffsetImm < 65536);
  auto Offset = static_cast<uint16_t>(OffsetImm);
  Offset &= 0xFF;

  Register Src = TRI.getSubReg(Base, MOS::sublo);

  auto LDC = Builder.buildInstr(MOS::LDCImm).add(CDef).addImm(0);
  if (LDC->getOperand(0).getSubReg())
    LDC->getOperand(0).setIsUndef();

  if (!Offset)
    Builder.buildInstr(MOS::COPY).add(Dst).addUse(Src);
  else {
    Register A = Builder.buildCopy(&MOS::AcRegClass, Src).getReg(0);
    auto Instr = Builder.buildInstr(MOS::ADCImm)
                     .addDef(A)
                     .add(CDef)
                     .add(VDef)
                     .addUse(A)
                     .addImm(Offset)
                     .addUse(CDef.getReg(), 0, CDef.getSubReg());
    Instr->getOperand(2).setIsDead();
    Builder.buildInstr(MOS::COPY).add(Dst).addUse(A);
  }

  MI->eraseFromParent();
}

void MOSRegisterInfo::expandAddrHistk(MachineBasicBlock::iterator MI) const {
  MachineIRBuilder Builder(*MI->getParent(), MI);
  const TargetRegisterInfo &TRI =
      *Builder.getMF().getSubtarget().getRegisterInfo();

  MachineOperand Dst = MI->getOperand(0);
  MachineOperand CDef = MI->getOperand(1);
  MachineOperand VDef = MI->getOperand(2);
  Register Base = MI->getOperand(3).getReg();

  int64_t OffsetImm = MI->getOperand(4).getImm();
  assert(0 <= OffsetImm && OffsetImm < 65536);
  auto Offset = static_cast<uint16_t>(OffsetImm);

  MachineOperand CUse = MI->getOperand(5);

  Register Src = TRI.getSubReg(Base, MOS::subhi);

  // Note: We can only elide the high byte of the address into a copy if the
  // whole offset is zero. There may be a carry from the low byte sum if only
  // the high byte is zero.
  if (!Offset)
    Builder.buildInstr(MOS::COPY).add(Dst).addUse(Src);
  else {
    Register A = Builder.buildCopy(&MOS::AcRegClass, Src).getReg(0);
    auto Instr = Builder.buildInstr(MOS::ADCImm)
                     .addDef(A)
                     .add(CDef)
                     .add(VDef)
                     .addUse(A)
                     .addImm(Offset >> 8)
                     .add(CUse);
    Instr->getOperand(1).setIsDead();
    Instr->getOperand(2).setIsDead();
    Builder.buildInstr(MOS::COPY).add(Dst).addUse(A);
  }

  MI->eraseFromParent();
}

void MOSRegisterInfo::expandLDSTStk(MachineBasicBlock::iterator MI) const {
  MachineFunction &MF = *MI->getMF();
  MachineIRBuilder Builder(*MI->getParent(), MI);
  MachineRegisterInfo &MRI = *Builder.getMRI();
  const TargetRegisterInfo &TRI = *MRI.getTargetRegisterInfo();

  const bool IsLoad = MI->getOpcode() == MOS::LDStk;

  Register Loc =
      IsLoad ? MI->getOperand(0).getReg() : MI->getOperand(1).getReg();
  int64_t Offset = MI->getOperand(3).getImm();

  if (Offset >= 256) {
    Register P = MRI.createVirtualRegister(&MOS::PcRegClass);
    // Far stack accesses need a virtual base register, so materialize one here
    // using the pointer provided.
    Register NewBase =
        IsLoad ? MI->getOperand(1).getReg() : MI->getOperand(0).getReg();
    // We can't scavenge a 16-bit register, so this can't be virtual here (after
    // register allocation).
    assert(!NewBase.isVirtual() && "LDSTStk must not use a virtual base "
                                   "pointer after register allocation.");

    auto Lo = Builder.buildInstr(MOS::AddrLostk)
                  .addDef(TRI.getSubReg(NewBase, MOS::sublo))
                  .addDef(P, /*Flags=*/0, MOS::subcarry)
                  .addDef(P, RegState::Dead, MOS::subv)
                  .add(MI->getOperand(2))
                  .add(MI->getOperand(3));
    auto Hi = Builder.buildInstr(MOS::AddrHistk)
                  .addDef(TRI.getSubReg(NewBase, MOS::subhi))
                  .addDef(P, RegState::Dead, MOS::subcarry)
                  .addDef(P, RegState::Dead, MOS::subv)
                  .add(MI->getOperand(2))
                  .add(MI->getOperand(3))
                  .addUse(P, /*Flags=*/0, MOS::subcarry)
                  .addUse(NewBase, RegState::Implicit);
    MI->getOperand(2).setReg(NewBase);
    MI->getOperand(3).setImm(0);

    expandAddrLostk(Lo);
    expandAddrHistk(Hi);
    expandLDSTStk(MI);
    return;
  }

  if (MOS::Imag16RegClass.contains(Loc)) {
    if (!IsLoad) {
      // Loc may not be fully alive at this point, which would create uses of
      // undefined subregisters. Issuing a KILL here redefines the full 16-bit
      // register, making both halves alive, regardless of which parts of the
      // register were alive before.
      Builder.buildInstr(MOS::KILL, {Loc}, {Loc});
    }
    Register Lo = TRI.getSubReg(Loc, MOS::sublo);
    Register Hi = TRI.getSubReg(Loc, MOS::subhi);
    auto LoInstr = Builder.buildInstr(MI->getOpcode());
    if (!IsLoad)
      LoInstr.add(MI->getOperand(0));
    LoInstr.addReg(Lo, getDefRegState(IsLoad));
    if (IsLoad)
      LoInstr.add(MI->getOperand(1));
    LoInstr.add(MI->getOperand(2))
        .add(MI->getOperand(3))
        .addMemOperand(MF.getMachineMemOperand(*MI->memoperands_begin(), 0, 1));
    auto HiInstr = Builder.buildInstr(MI->getOpcode());
    if (!IsLoad)
      HiInstr.add(MI->getOperand(0));
    HiInstr.addReg(Hi, getDefRegState(IsLoad));
    if (IsLoad)
      HiInstr.add(MI->getOperand(1));
    HiInstr.add(MI->getOperand(2))
        .addImm(MI->getOperand(3).getImm() + 1)
        .addMemOperand(MF.getMachineMemOperand(*MI->memoperands_begin(), 1, 1));
    MI->eraseFromParent();
    expandLDSTStk(LoInstr);
    expandLDSTStk(HiInstr);
    return;
  }

  Register Loc8 =
      TRI.getMatchingSuperReg(Loc, MOS::sublsb, &MOS::Anyi8RegClass);
  if (Loc8)
    Loc = Loc8;

  assert(Loc == MOS::C || Loc == MOS::V || MOS::Anyi8RegClass.contains(Loc));

  Register A = Loc;
  if (A != MOS::A)
    A = MRI.createVirtualRegister(&MOS::AcRegClass);

  // Transfer the value to A to be stored (if applicable).
  if (!IsLoad && Loc != A) {
    if (Loc == MOS::C || Loc == MOS::V)
      Builder.buildInstr(MOS::COPY)
          .addDef(A, RegState::Undef, MOS::sublsb)
          .addUse(Loc);
    else {
      assert(MOS::Anyi8RegClass.contains(Loc));
      Builder.buildCopy(A, Loc);
    }
  }

  // This needs to occur after the above copy since the source may be Y.
  Register Y =
      Builder.buildInstr(MOS::LDImm, {&MOS::YcRegClass}, {Offset}).getReg(0);

  Builder.buildInstr(IsLoad ? MOS::LDIndirIdx : MOS::STIndirIdx)
      .addReg(A, getDefRegState(IsLoad))
      .add(MI->getOperand(2))
      .addUse(Y)
      .addMemOperand(*MI->memoperands_begin());

  // Transfer the loaded value out of A (if applicable).
  if (IsLoad && Loc != A) {
    if (Loc == MOS::C || Loc == MOS::V)
      Builder.buildInstr(MOS::COPY, {Loc}, {}).addUse(A, 0, MOS::sublsb);
    else {
      assert(MOS::Anyi8RegClass.contains(Loc));
      Builder.buildCopy(Loc, A);
    }
  }

  MI->eraseFromParent();
  return;
}

Register MOSRegisterInfo::getFrameRegister(const MachineFunction &MF) const {
  const TargetFrameLowering *TFI = getFrameLowering(MF);
  return TFI->hasFP(MF) ? MOS::RS15 : MOS::RS0;
}

bool referencedByShiftRotate(Register Reg, const MachineRegisterInfo &MRI) {
  for (MachineInstr &MI : MRI.reg_nodbg_instructions(Reg)) {
    switch (MI.getOpcode()) {
    default:
      break;
    case MOS::ASL:
    case MOS::LSR:
    case MOS::ROL:
    case MOS::ROR:
      return true;
    }
  }
  return false;
}

bool referencedByIncDec(Register Reg, const MachineRegisterInfo &MRI) {
  for (MachineInstr &MI : MRI.reg_nodbg_instructions(Reg)) {
    switch (MI.getOpcode()) {
    default:
      break;
    case MOS::INC:
    case MOS::DEC:
      return true;
    }
  }
  return false;
}

// Returns whether there's exactly one RMW operation, and all of the other
// references are to the poorer regclass. In that case, it's better to do the
// operation in the poorer regclass then to copy into a better one then copy
// back out.
bool isRmwPattern(Register Reg, const MachineRegisterInfo &MRI) {
  SmallVector<const MachineInstr *> RMW;
  const MachineInstr *Rmw = nullptr;
  for (MachineInstr &MI : MRI.reg_nodbg_instructions(Reg)) {
    switch (MI.getOpcode()) {
    default:
      break;
    case MOS::ASL:
    case MOS::LSR:
    case MOS::ROL:
    case MOS::ROR:
    case MOS::INC:
    case MOS::DEC:
      if (Rmw && Rmw != &MI)
        return false;
      Rmw = &MI;
      continue;
    }

    if (!MI.isCopy())
      return false;

    Register Dst = MI.getOperand(0).getReg();
    Register Src = MI.getOperand(1).getReg();

    Register Other = Reg == Dst ? Src : Dst;
    assert(Other != Reg);

    if (Other.isPhysical()) {
      if (!MOS::Imag8RegClass.contains(Other))
        return false;
      continue;
    }

    const auto *OtherRC = MRI.getRegClass(Other);
    if (OtherRC != &MOS::Imag8RegClass && OtherRC != &MOS::Imag16RegClass)
      return false;
  }
  assert(Rmw);
  return true;
}

bool MOSRegisterInfo::shouldCoalesce(
    MachineInstr *MI, const TargetRegisterClass *SrcRC, unsigned SubReg,
    const TargetRegisterClass *DstRC, unsigned DstSubReg,
    const TargetRegisterClass *NewRC, LiveIntervals &LIS) const {
  // Don't coalesce Imag8 and AImag8 registers together when used by shifts or
  // rotates.  This may cause expensive ASL zp's to be used when ASL A would
  // have sufficed. It's better to do arithmetic in A and then copy it out.
  // Same concerns apply to INC and DEC.
  if (NewRC == &MOS::Imag8RegClass || NewRC == &MOS::Imag16RegClass) {
    const auto &MRI = MI->getMF()->getRegInfo();
    if (DstRC == &MOS::AImag8RegClass &&
        referencedByShiftRotate(MI->getOperand(0).getReg(), MRI) &&
        !isRmwPattern(MI->getOperand(0).getReg(), MRI))
      return false;
    if (SrcRC == &MOS::AImag8RegClass &&
        referencedByShiftRotate(MI->getOperand(1).getReg(), MRI) &&
        !isRmwPattern(MI->getOperand(1).getReg(), MRI))
      return false;
    if (DstRC == &MOS::Anyi8RegClass &&
        referencedByIncDec(MI->getOperand(0).getReg(), MRI) &&
        !isRmwPattern(MI->getOperand(0).getReg(), MRI))
      return false;
    if (SrcRC == &MOS::Anyi8RegClass &&
        referencedByIncDec(MI->getOperand(1).getReg(), MRI) &&
        !isRmwPattern(MI->getOperand(1).getReg(), MRI))
      return false;
  }
  return true;
}

int copyCost(Register DestReg, Register SrcReg, const MOSSubtarget &STI) {
  const auto &TRI = *STI.getRegisterInfo();
  if (DestReg == SrcReg)
    return 0;

  const auto &AreClasses = [&](const TargetRegisterClass &Dest,
                               const TargetRegisterClass &Src) {
    return Dest.contains(DestReg) && Src.contains(SrcReg);
  };

  if (AreClasses(MOS::GPRRegClass, MOS::GPRRegClass)) {
    if (MOS::AcRegClass.contains(SrcReg)) {
      assert(MOS::XYRegClass.contains(DestReg));
      // TAX
      return 3;
    }
    if (MOS::AcRegClass.contains(DestReg)) {
      // TXA
      return 3;
    }
    // May need to pha/pla around; avg cost 4
    return 4 + copyCost(DestReg, MOS::A, STI) + copyCost(MOS::A, SrcReg, STI);
  }
  if (AreClasses(MOS::Imag8RegClass, MOS::GPRRegClass)) {
    // STImag8
    return 5;
  }
  if (AreClasses(MOS::GPRRegClass, MOS::Imag8RegClass)) {
    // LDImag8
    return 5;
  }
  if (AreClasses(MOS::Imag8RegClass, MOS::Imag8RegClass)) {
    // May need to pha/pla around; avg cost 4
    return 4 + copyCost(DestReg, MOS::A, STI) + copyCost(MOS::A, SrcReg, STI);
  }
  if (AreClasses(MOS::Imag16RegClass, MOS::Imag16RegClass)) {
    return 2 * copyCost(MOS::RC0, MOS::RC1, STI);
  }
  if (AreClasses(MOS::Anyi1RegClass, MOS::Anyi1RegClass)) {
    Register SrcReg8 =
        TRI.getMatchingSuperReg(SrcReg, MOS::sublsb, &MOS::Anyi8RegClass);
    Register DestReg8 =
        TRI.getMatchingSuperReg(DestReg, MOS::sublsb, &MOS::Anyi8RegClass);

    if (SrcReg8) {
      SrcReg = SrcReg8;
      if (DestReg8) {
        DestReg = DestReg8;
        return copyCost(DestReg, SrcReg, STI);
      }
      if (DestReg == MOS::C) {
        // Cmp #1
        int Cost = 4;
        if (!MOS::GPRRegClass.contains(SrcReg))
          Cost += copyCost(MOS::A, SrcReg, STI);
        return Cost;
      }
      assert(DestReg == MOS::V);
      const TargetRegisterClass &StackRegClass =
          STI.has65C02() ? MOS::GPRRegClass : MOS::AcRegClass;

      if (StackRegClass.contains(SrcReg)) {
        // PHA; PLA; BNE; BIT setv; JMP; CLV
        return 30;
      }
      // [PHA]; COPY; BNE; BIT setv; JMP; CLV; [PLA]
      return 23 + copyCost(MOS::A, SrcReg, STI);
    }
    if (DestReg8) {
      DestReg = DestReg8;

      Register Tmp = DestReg;
      if (!MOS::GPRRegClass.contains(Tmp))
        Tmp = MOS::A;
      // LDImm; BNE; LDImm;
      int Cost = 13;
      if (Tmp != DestReg)
        Cost += copyCost(DestReg, Tmp, STI);
      return Cost;
    }
    // BIT setv; BR; CLV;
    return 15;
  }

  llvm_unreachable("Unexpected physical register copy.");
}

bool MOSRegisterInfo::getRegAllocationHints(Register VirtReg,
                                            ArrayRef<MCPhysReg> Order,
                                            SmallVectorImpl<MCPhysReg> &Hints,
                                            const MachineFunction &MF,
                                            const VirtRegMap *VRM,
                                            const LiveRegMatrix *Matrix) const {
  const MOSSubtarget &STI = MF.getSubtarget<MOSSubtarget>();
  const auto &TRI = *STI.getRegisterInfo();
  const MachineRegisterInfo &MRI = MF.getRegInfo();
  DenseMap<Register, int> RegScores;

  DenseMap<Register, int> OriginalIndex;
  for (const auto &R : enumerate(Order))
    OriginalIndex[R.value()] = R.index();

  SmallSet<const MachineInstr *, 32> Visited;
  for (MachineInstr &MI : MRI.reg_nodbg_instructions(VirtReg)) {
    if (!Visited.insert(&MI).second)
      continue;
    switch (MI.getOpcode()) {
    default:
      continue;
    case MOS::COPY: {
      const MachineOperand &Self = MI.getOperand(0).getReg() == VirtReg
                                       ? MI.getOperand(0)
                                       : MI.getOperand(1);
      const MachineOperand &Other = MI.getOperand(0).getReg() == VirtReg
                                        ? MI.getOperand(1)
                                        : MI.getOperand(0);
      Register OtherReg = Other.getReg();
      if (OtherReg.isVirtual()) {
        if (!VRM->hasPhys(OtherReg))
          break;
        OtherReg = VRM->getPhys(OtherReg);
      }
      if (Other.getSubReg())
        OtherReg = TRI.getSubReg(OtherReg, Other.getSubReg());
      int WorstCost = 0;
      for (Register R : Order) {
        Register SelfReg = R;
        if (Self.getSubReg())
          SelfReg = TRI.getSubReg(SelfReg, Self.getSubReg());
        WorstCost = std::max(WorstCost, copyCost(SelfReg, OtherReg, STI));
      }
      for (Register R : Order) {
        Register SelfReg = R;
        if (Self.getSubReg())
          SelfReg = TRI.getSubReg(SelfReg, Self.getSubReg());
        int Cost = copyCost(SelfReg, OtherReg, STI);
        if (Cost < WorstCost)
          RegScores[R] += WorstCost - Cost;
      }
      break;
    }
    case MOS::ASL:
    case MOS::LSR:
    case MOS::ROR:
    case MOS::ROL:
      // ASL zp = 7
      // ASL a = 3
      if (is_contained(Order, MOS::A))
        RegScores[MOS::A] += 4;
      break;

    case MOS::CMPTermZ:
      // CMPTermZ GPR best case: 0 (TAX)
      // CMPTermZ GPR worst case: 4 (CMP #0)
      // Splitting the difference: 2
      // CMPTermZ ZP best case: 0 (elided)
      // CMPTermZ ZP worst case: 14 (INC DEC)
      // Splitting the difference: 7
      if (is_contained(Order, MOS::A))
        RegScores[MOS::A] += 5;
      if (is_contained(Order, MOS::X))
        RegScores[MOS::X] += 5;
      if (is_contained(Order, MOS::Y))
        RegScores[MOS::Y] += 5;
      break;

    case MOS::INC:
    case MOS::DEC:
    case MOS::IncMB:
    case MOS::DecMB:
      // The first operand to DecMB is scratch.
      if (MI.getOpcode() == MOS::DecMB && MI.getOperand(0).getReg() == VirtReg)
        break;

      // INC zp = (2 bytes + 5 cycles)
      // INXY = (1 bytes + 2 cycles)
      if (is_contained(Order, MOS::X))
        RegScores[MOS::X] += 4;
      if (is_contained(Order, MOS::Y))
        RegScores[MOS::Y] += 4;
      break;
    }
  }

  SmallVector<std::pair<Register, int>> RegsAndScores(RegScores.begin(),
                                                      RegScores.end());
  sort(RegsAndScores, [&](const std::pair<Register, int> &A,
                          const std::pair<Register, int> &B) {
    if (A.second > B.second)
      return true;
    if (A.second < B.second)
      return false;
    return OriginalIndex[A.first] < OriginalIndex[B.first];
  });
  append_range(Hints, make_first_range(RegsAndScores));
  return false;
}

void MOSRegisterInfo::reserveAllSubregs(BitVector *Reserved,
                                        Register Reg) const {
  for (Register R : subregs_inclusive(Reg))
    Reserved->set(R);
}
