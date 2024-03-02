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
#include "MOSInstrBuilder.h"
#include "MOSInstrCost.h"
#include "MOSInstrInfo.h"
#include "MOSMachineFunctionInfo.h"
#include "MOSSubtarget.h"
#include "llvm/ADT/SmallSet.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/CodeGen/LivePhysRegs.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineOperand.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/TargetFrameLowering.h"
#include "llvm/CodeGen/TargetInstrInfo.h"
#include "llvm/CodeGen/TargetRegisterInfo.h"
#include "llvm/CodeGen/VirtRegMap.h"
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
  const MOSFrameLowering &TFL =
      *MF.getSubtarget<MOSSubtarget>().getFrameLowering();
  return TFL.usesStaticStack(MF) ? 15 * 16384 / 10 : 5 * 16384 / 10;
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

static void assertNZDeadAt(MachineBasicBlock &MBB,
                           MachineBasicBlock::iterator Pos) {
#ifndef NDEBUG
  LivePhysRegs LiveRegs;
  LiveRegs.init(*MBB.getParent()->getSubtarget().getRegisterInfo());
  LiveRegs.addLiveOutsNoPristines(MBB);
  for (MachineBasicBlock::reverse_iterator
           I = MBB.rbegin(),
           E = MachineBasicBlock::reverse_iterator(Pos);
       I != E; ++I) {
    LiveRegs.stepBackward(*I);
  }
  assert(!LiveRegs.contains(MOS::N) &&
         "expected N to be free when saving scavenger register");
  assert(!LiveRegs.contains(MOS::Z) &&
         "expected Z to be free when saving scavenger register");
#endif
}

bool MOSRegisterInfo::saveScavengerRegister(MachineBasicBlock &MBB,
                                            MachineBasicBlock::iterator I,
                                            MachineBasicBlock::iterator &UseMI,
                                            const TargetRegisterClass *RC,
                                            Register Reg) const {

  // Note: NZ cannot be live at this point, since virtual registers are never
  // inserted into CmpBr instructions.
  assertNZDeadAt(MBB, I);
  assertNZDeadAt(MBB, UseMI);

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
  case MOS::Y:
  case MOS::P: {
    // RS8 is reserved to save A and Y if necessary, but pushing is still
    // preferred.
    Register Save = Reg == MOS::A ? MOS::RC16 : MOS::RC17;
    bool UseHardStack =
        (Reg == MOS::A || Reg == MOS::P || STI.hasGPRStackRegs()) &&
        pushPullBalanced(I, UseMI);

    // P can only be efficiently saved to the hard stack.
    assert(!(Reg == MOS::P && !UseHardStack) &&
           "expected P live range to fully contain all overlapping vreg live "
           "ranges");

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

bool MOSRegisterInfo::canSaveScavengerRegister(
    Register Reg, MachineBasicBlock::iterator I,
    MachineBasicBlock::iterator UseMI) const {
  // Easy cases
  switch (Reg) {
  case MOS::X:
    return false;
  case MOS::P:
    return pushPullBalanced(I, UseMI);
  default:
    break;
  }

  const MOSSubtarget &STI = I->getMF()->getSubtarget<MOSSubtarget>();

  bool UseHardStack =
      (Reg == MOS::A || STI.hasGPRStackRegs()) && pushPullBalanced(I, UseMI);
  if (UseHardStack)
    return true;

  // Because the scavenger may run more than once, the reserved register may
  // already be in use. In such cases, it's not safe to save it, and a
  // different register must be used.
  Register Save = Reg == MOS::A ? MOS::RC16 : MOS::RC17;
  LivePhysRegs LPR(*STI.getRegisterInfo());
  LPR.addLiveOuts(*I->getParent());
  for (MachineBasicBlock::iterator J = std::prev(I->getParent()->end()); J != I;
       --J) {
    LPR.stepBackward(*J);
    if (J == UseMI && LPR.contains(Save))
      return false;
  }
  LPR.stepBackward(*I);
  return !LPR.contains(Save);
}

bool MOSRegisterInfo::eliminateFrameIndex(MachineBasicBlock::iterator MI,
                                          int SPAdj, unsigned FIOperandNum,
                                          RegScavenger *RS) const {
  MachineFunction &MF = *MI->getMF();
  const MachineFrameInfo &MFI = MF.getFrameInfo();
  const auto &MOSFI = MF.getInfo<MOSFunctionInfo>();

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
    if (MFI.getStackID(Idx) == TargetStackID::MosZeroPage) {
      MI->getOperand(FIOperandNum)
          .ChangeToGA(MOSFI->ZeroPageStackValue, Offset,
                      MI->getOperand(FIOperandNum).getTargetFlags());
    } else {
      assert(MFI.getStackID(Idx) == TargetStackID::MosStatic);
      MI->getOperand(FIOperandNum)
          .ChangeToTargetIndex(MOS::TI_STATIC_STACK, Offset,
                               MI->getOperand(FIOperandNum).getTargetFlags());
    }
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
    return false;
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
  return true;
}

void MOSRegisterInfo::expandAddrLostk(MachineBasicBlock::iterator MI) const {
  MachineIRBuilder Builder(*MI);
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
  MachineIRBuilder Builder(*MI);
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
  MachineIRBuilder Builder(*MI);
  MachineRegisterInfo &MRI = *Builder.getMRI();
  const TargetRegisterInfo &TRI = *MRI.getTargetRegisterInfo();

  const bool IsLoad = MI->getOpcode() == MOS::LDStk;

  Register Loc =
      IsLoad ? MI->getOperand(0).getReg() : MI->getOperand(1).getReg();
  int64_t Offset = MI->getOperand(3).getImm();

  if (Offset >= 256) {
    Register P = MRI.createVirtualRegister(&MOS::PcRegClass);
    // Far stack accesses need a virtual base register, so materialize one
    // here using the pointer provided.
    Register NewBase =
        IsLoad ? MI->getOperand(1).getReg() : MI->getOperand(0).getReg();
    // We can't scavenge a 16-bit register, so this can't be virtual here
    // (after register allocation).
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
    case MOS::IncNMOS:
    case MOS::DecNMOS:
    case MOS::IncMB:
    case MOS::DecMB:
    case MOS::DecDcpMB:
      return true;
    }
  }
  return false;
}

bool referencedByIncDecMB(Register Reg, const MachineRegisterInfo &MRI) {
  for (MachineInstr &MI : MRI.reg_nodbg_instructions(Reg)) {
    switch (MI.getOpcode()) {
    default:
      break;
    case MOS::IncMB:
    case MOS::DecMB:
    case MOS::DecDcpMB:
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
    case MOS::IncMB:
    case MOS::DecMB:
    case MOS::DecDcpMB:
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
  const auto &MRI = MI->getMF()->getRegInfo();

  // Don't coalesce Imag8 and AImag8 registers together when used by shifts or
  // rotates.  This may cause expensive ASL zp's to be used when ASL A would
  // have sufficed. It's better to do arithmetic in A and then copy it out.
  // Same concerns apply to INC and DEC.
  if (NewRC == &MOS::Imag8RegClass || NewRC == &MOS::Imag16RegClass) {
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
  // Don't coalesce GPR and Anyi8 registers together when used by IncMB and
  // DecMB; this can make them impossible to allocate.
  if (NewRC == &MOS::GPRRegClass) {
    if (DstRC == &MOS::Anyi8RegClass &&
        referencedByIncDecMB(MI->getOperand(0).getReg(), MRI))
      return false;
    if (SrcRC == &MOS::Anyi8RegClass &&
        referencedByIncDecMB(MI->getOperand(1).getReg(), MRI))
      return false;
  }
  return true;
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
  DenseMap<Register, MOSInstrCost> RegScores;
  auto CostMode = MOSInstrCost::getModeFor(MF);

  DenseMap<Register, int> OriginalIndex;
  for (const auto &R : enumerate(Order))
    OriginalIndex[R.value()] = R.index();

  if (std::optional<Register> StrongHint =
          getStrongCopyHint(VirtReg, MF, VRM)) {
    if (*StrongHint)
      Hints.push_back(*StrongHint);
    return true;
  }

  MOSInstrCost INCzp = MOSInstrCost(2, 5);
  MOSInstrCost INCxy = MOSInstrCost(1, 2);
  MOSInstrCost ASLzp = MOSInstrCost(2, 5);
  MOSInstrCost ASLa = MOSInstrCost(1, 2);
  if (STI.hasHUC6280()) {
    INCzp = MOSInstrCost(2, 6);
    ASLzp = MOSInstrCost(2, 6);
  }
  if (STI.has65CE02() || STI.hasSPC700()) {
    INCzp = MOSInstrCost(2, 4);
    ASLzp = MOSInstrCost(2, 4);
  }
  if (STI.has65CE02()) {
    INCxy = MOSInstrCost(1, 1);
    ASLa = MOSInstrCost(1, 1);
  }

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
      MOSInstrCost WorstCost;
      for (Register R : Order) {
        Register SelfReg = R;
        if (Self.getSubReg())
          SelfReg = TRI.getSubReg(SelfReg, Self.getSubReg());
        MOSInstrCost Cost = copyCost(SelfReg, OtherReg, STI);
        if (Cost.value(CostMode) > WorstCost.value(CostMode))
          WorstCost = Cost;
      }
      for (Register R : Order) {
        Register SelfReg = R;
        if (Self.getSubReg())
          SelfReg = TRI.getSubReg(SelfReg, Self.getSubReg());
        MOSInstrCost Cost = copyCost(SelfReg, OtherReg, STI);
        if (Cost.value(CostMode) < WorstCost.value(CostMode))
          RegScores[R] += (WorstCost - Cost);
      }
      break;
    }
    case MOS::ASL:
    case MOS::LSR:
    case MOS::ROR:
    case MOS::ROL:
      if (is_contained(Order, MOS::A))
        RegScores[MOS::A] += ASLzp - ASLa;
      break;

    case MOS::CmpBrZero: {
      // Branch costs are uniform; factor them out.
      // CmpZero GPR best case: 0 (TAX)
      // CmpZero GPR worst case: 4 (CMP #0)
      // Splitting the difference: 2
      MOSInstrCost CmpZeroGPR = MOSInstrCost(2, 2) / 2;
      // CmpZero ZP best case: 0 (elided)
      // CmpZero ZP worst case: 14 (INC DEC)
      // Splitting the difference: 7
      MOSInstrCost CmpZeroZP = INCzp * 2 / 2;
      if (is_contained(Order, MOS::A))
        RegScores[MOS::A] += CmpZeroZP - CmpZeroGPR;
      if (is_contained(Order, MOS::X))
        RegScores[MOS::X] += CmpZeroZP - CmpZeroGPR;
      if (is_contained(Order, MOS::Y))
        RegScores[MOS::Y] += CmpZeroZP - CmpZeroGPR;
      break;
    }

    case MOS::INC:
    case MOS::DEC:
    case MOS::IncNMOS:
    case MOS::DecNMOS:
    case MOS::IncMB:
    case MOS::DecMB:
    case MOS::DecDcpMB: {
      // The first operand to DecMB is scratch.
      if ((MI.getOpcode() == MOS::DecMB || MI.getOpcode() == MOS::DecDcpMB) &&
          MI.getOperand(0).getReg() == VirtReg)
        break;

      if (STI.hasGPRIncDec() && is_contained(Order, MOS::A))
        RegScores[MOS::A] += INCzp - INCxy;
      if (is_contained(Order, MOS::X))
        RegScores[MOS::X] += INCzp - INCxy;
      if (is_contained(Order, MOS::Y))
        RegScores[MOS::Y] += INCzp - INCxy;
      break;
    }
    }
  }

  SmallVector<std::pair<Register, MOSInstrCost>> RegsAndScores(
      RegScores.begin(), RegScores.end());
  sort(RegsAndScores, [&](const std::pair<Register, MOSInstrCost> &A,
                          const std::pair<Register, MOSInstrCost> &B) {
    auto AVal = A.second.value(CostMode);
    auto BVal = B.second.value(CostMode);
    if (AVal > BVal)
      return true;
    if (AVal < BVal)
      return false;
    return OriginalIndex[A.first] < OriginalIndex[B.first];
  });
  append_range(Hints, make_first_range(RegsAndScores));
  return false;
}

// If the VirtReg is trivially rematerializable, and the only uses of VirtReg
// are copies with exactly one register, returns a hint containing that
// register. If there are more than one such register, returns Some(0).
// Otherwise, returns None. This prevents the register allocator from
// assigning a value to a useless register; it's always better to split or
// spill in such cases, since absolutely nothing can use the value in that
// register.
std::optional<Register>
MOSRegisterInfo::getStrongCopyHint(Register VirtReg, const MachineFunction &MF,
                                   const VirtRegMap *VRM) const {
  const MachineRegisterInfo &MRI = MF.getRegInfo();
  const MOSSubtarget &STI = MF.getSubtarget<MOSSubtarget>();
  const auto &TRI = *STI.getRegisterInfo();
  const auto &TII = *STI.getInstrInfo();

  if (!MRI.hasOneDef(VirtReg))
    return std::nullopt;
  if (!TII.isReallyTriviallyReMaterializable(
          *MRI.getOneDef(VirtReg)->getParent()))
    return std::nullopt;

  std::optional<Register> Hint;
  for (MachineInstr &MI : MRI.use_nodbg_instructions(VirtReg)) {
    if (MI.getOpcode() != MOS::COPY)
      return std::nullopt;
    const MachineOperand &Self = MI.getOperand(0).getReg() == VirtReg
                                     ? MI.getOperand(0)
                                     : MI.getOperand(1);
    const MachineOperand &Other = MI.getOperand(0).getReg() == VirtReg
                                      ? MI.getOperand(1)
                                      : MI.getOperand(0);
    Register OtherReg = Other.getReg();
    if (OtherReg.isVirtual()) {
      if (!VRM->hasPhys(OtherReg))
        return std::nullopt;
      OtherReg = VRM->getPhys(OtherReg);
    }
    if (Other.getSubReg())
      OtherReg = TRI.getSubReg(OtherReg, Other.getSubReg());

    Register Reg = OtherReg;
    if (Self.getSubReg())
      Reg = TRI.getMatchingSuperReg(Reg, Self.getSubReg(),
                                    MRI.getRegClass(Self.getReg()));
    if (!Reg || !MRI.getRegClass(Self.getReg())->contains(Reg))
      return std::nullopt;
    if (Hint && *Hint != Reg) {
      *Hint = MOS::NoRegister;
      break;
    }
    if (!Hint)
      Hint = Reg;
  }
  return Hint;
}

void MOSRegisterInfo::reserveAllSubregs(BitVector *Reserved,
                                        Register Reg) const {
  for (Register R : subregs_inclusive(Reg))
    Reserved->set(R);
}

MOSInstrCost MOSRegisterInfo::copyCost(Register DestReg, Register SrcReg,
                                       const MOSSubtarget &STI) const {
  if (DestReg == SrcReg)
    return MOSInstrCost();

  const auto &AreClasses = [&](const TargetRegisterClass &Dest,
                               const TargetRegisterClass &Src) {
    return Dest.contains(DestReg) && Src.contains(SrcReg);
  };

  auto TransferCost = MOSInstrCost(1, STI.has65CE02() ? 1 : 2);
  auto PushCost = MOSInstrCost(1, STI.hasSPC700() ? 4 : 3);
  auto PopCost = MOSInstrCost(1, STI.has65CE02() ? 3 : 4);
  auto ClvCost = MOSInstrCost(1, STI.has65CE02() ? 1 : 2);
  auto JumpCost = MOSInstrCost(3, 3);
  auto BranchCost = MOSInstrCost(2, 3);
  auto LoadImmCost = MOSInstrCost(2, 2);
  auto AluImmCost = MOSInstrCost(2, 2);

  if (AreClasses(MOS::GPRRegClass, MOS::GPRRegClass)) {
    if (MOS::AcRegClass.contains(SrcReg)) {
      assert(MOS::XYRegClass.contains(DestReg));
      // TAX
      return TransferCost;
    }
    if (MOS::AcRegClass.contains(DestReg)) {
      // TXA
      return TransferCost;
    }

    // X<->Y copies
    if (STI.hasW65816Or65EL02()) {
      // TXY, TYX
      return TransferCost;
    }
    MOSInstrCost XYCopyCost;
    if (STI.hasGPRStackRegs()) {
      // PHX/PLY, PHY/PLX
      XYCopyCost = PushCost + PopCost;
    } else {
      // May need to PHA/PLA around.
      XYCopyCost = (PushCost + PopCost) / 2 +
                   copyCost(DestReg, MOS::A, STI) +
                   copyCost(MOS::A, SrcReg, STI);
    }
    if (STI.hasHUC6280()) {
      // SXY can be used, but only if the source register is killed. As such,
      // average the cost.
      XYCopyCost = (XYCopyCost + MOSInstrCost(1, 3)) / 2;
    }
    return XYCopyCost;
  }
  if (AreClasses(MOS::Imag8RegClass, MOS::GPRRegClass)) {
    // STImag8
    return MOSInstrCost(2, (STI.hasHUC6280() || STI.hasSPC700()) ? 4 : 3);
  }
  if (AreClasses(MOS::GPRRegClass, MOS::Imag8RegClass)) {
    // LDImag8
    return MOSInstrCost(2, (STI.hasHUC6280() || STI.hasSPC700()) ? 4 : 3);
  }
  if (AreClasses(MOS::Imag8RegClass, MOS::Imag8RegClass)) {
    // MOV dp, dp
    if (STI.hasSPC700())
      return MOSInstrCost(3, 5);
    // May need to PHA/PLA around.
    return (PushCost + PopCost) / 2 + copyCost(DestReg, MOS::A, STI) +
           copyCost(MOS::A, SrcReg, STI);
  }
  if (AreClasses(MOS::Imag16RegClass, MOS::Imag16RegClass)) {
    return copyCost(MOS::RC0, MOS::RC1, STI) * 2;
  }
  if (AreClasses(MOS::Anyi1RegClass, MOS::Anyi1RegClass)) {
    Register SrcReg8 =
        getMatchingSuperReg(SrcReg, MOS::sublsb, &MOS::Anyi8RegClass);
    Register DestReg8 =
        getMatchingSuperReg(DestReg, MOS::sublsb, &MOS::Anyi8RegClass);
    // BIT imm (HUC6280), BIT abs
    auto BitCost = STI.hasHUC6280() ? MOSInstrCost(2, 2) :
                   STI.has65CE02() ? MOSInstrCost(3, 5) :
                   MOSInstrCost(3, 4);

    if (SrcReg8) {
      SrcReg = SrcReg8;
      if (DestReg8) {
        DestReg = DestReg8;
        return copyCost(DestReg, SrcReg, STI);
      }
      if (DestReg == MOS::C) {
        // Cmp #1
        MOSInstrCost Cost = AluImmCost;
        if (!MOS::GPRRegClass.contains(SrcReg))
          Cost += copyCost(MOS::A, SrcReg, STI);
        return Cost;
      }

      assert(DestReg == MOS::V);
      if (STI.hasSPC700()) {
        // PHP, PLA, ORA #imm, PHA, PLP; may PHA/PLA
        return ((PushCost + PopCost) * 5 / 2) + AluImmCost;
      }

      const TargetRegisterClass &StackRegClass =
          STI.hasGPRStackRegs() ? MOS::GPRRegClass : MOS::AcRegClass;

      if (StackRegClass.contains(SrcReg)) {
        // PHA; PLA; BNE; BIT setv; JMP; CLV
        return PushCost + PopCost + BranchCost + BitCost + JumpCost + ClvCost;
      }
      // [PHA]; COPY; BNE; BIT setv; JMP; CLV; [PLA]
      return copyCost(MOS::A, SrcReg, STI) + BranchCost + BitCost +
             JumpCost + ClvCost;
    }
    if (DestReg8) {
      DestReg = DestReg8;

      Register Tmp = DestReg;
      if (!MOS::GPRRegClass.contains(Tmp))
        Tmp = MOS::A;
      // LDImm; BNE; LDImm;
      MOSInstrCost Cost = LoadImmCost * 2 + BranchCost;
      if (Tmp != DestReg)
        Cost += copyCost(DestReg, Tmp, STI);
      return Cost;
    }
    if (STI.hasSPC700()) {
      // PHA, PHP, PLA, ORA #imm, PHA, PLP, PLA, BR, CLV
      return (PushCost + PopCost) * 3 + AluImmCost + BranchCost +
             ClvCost;
    }
    // BIT setv; BR; CLV;
    return BitCost + BranchCost + ClvCost;
  }

  llvm_unreachable("Unexpected physical register copy.");
}
