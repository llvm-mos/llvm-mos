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
#include "MOSFrameLowering.h"
#include "MOSInstrInfo.h"
#include "MOSSubtarget.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineOperand.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/TargetFrameLowering.h"
#include "llvm/CodeGen/TargetInstrInfo.h"
#include "llvm/CodeGen/TargetRegisterInfo.h"
#include "llvm/MC/MCRegisterInfo.h"
#include "llvm/Support/ErrorHandling.h"

#define DEBUG_TYPE "mos-reginfo"

#define GET_REGINFO_TARGET_DESC
#include "MOSGenRegisterInfo.inc"

using namespace llvm;

cl::opt<int>
    NumImagPtrs("num-imag-ptrs", cl::init(127), cl::ZeroOrMore,
                cl::desc("Number of imaginary (Imag8) pointer registers "
                         "available for compiler use."),
                cl::value_desc("imaginary pointer registers"));

MOSRegisterInfo::MOSRegisterInfo()
    : MOSGenRegisterInfo(/*RA=*/0, /*DwarfFlavor=*/0, /*EHFlavor=*/0,
                         /*PC=*/0, /*HwMode=*/0),
      Imag8SymbolNames(new std::string[getNumRegs()]), Reserved(getNumRegs()) {
  for (unsigned Reg = 0; Reg < getNumRegs(); ++Reg) {
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

  // One for the stack pointer, one for the frame pointer, and one to ensure
  // that zero-page indirect addressing modes can be used.
  if (NumImagPtrs < 3)
    report_fatal_error("At least three imaginary pointers must be available.");
  if (NumImagPtrs > 128)
    report_fatal_error("More than 128 imaginary pointers cannot be available: "
                       "only 128 exist.");

  // Reserve all imaginary registers beyond the number allowed to the compiler.
  for (Register Ptr = MOS::RS0 + NumImagPtrs; Ptr <= MOS::RS127;
       Ptr = Ptr + 1) {
    Reserved.set(Ptr);
    Register Lo = getSubReg(Ptr, MOS::sublo);
    Reserved.set(Lo);
    Reserved.set(getSubReg(Lo, MOS::sublsb));
    Register Hi = getSubReg(Ptr, MOS::subhi);
    Reserved.set(Hi);
    Reserved.set(getSubReg(Hi, MOS::sublsb));
  }

  // Reserve stack pointers.
  Reserved.set(MOS::RS0);
  Reserved.set(MOS::RC0);
  Reserved.set(MOS::RC0LSB);
  Reserved.set(MOS::RC1);
  Reserved.set(MOS::RC1LSB);
}

const MCPhysReg *
MOSRegisterInfo::getCalleeSavedRegs(const MachineFunction *MF) const {
  return MOS_CSR_SaveList;
}

const uint32_t *MOSRegisterInfo::getCallPreservedMask(const MachineFunction &MF,
                                                      CallingConv::ID) const {
  return MOS_CSR_RegMask;
}

BitVector MOSRegisterInfo::getReservedRegs(const MachineFunction &MF) const {
  const TargetFrameLowering *TFI = getFrameLowering(MF);
  BitVector Reserved = this->Reserved;
  if (TFI->hasFP(MF)) {
    Reserved.set(MOS::RS2);
    Reserved.set(MOS::RC4);
    Reserved.set(MOS::RC5);
  }
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

static bool pushPullBalanced(MachineBasicBlock::iterator Begin,
                             MachineBasicBlock::iterator End) {
  int64_t PushCount = 0;
  for (auto MI = Begin; MI != End; ++MI) {
    switch (MI->getOpcode()) {
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
  const TargetRegisterInfo &TRI =
      *Builder.getMF().getSubtarget().getRegisterInfo();

  switch (Reg) {
  default:
    errs() << "Register: " << getName(Reg) << "\n";
    report_fatal_error("Scavenger spill for register not yet implemented.");
  case MOS::A:
  case MOS::ALSB: {
    bool UseHardStack = pushPullBalanced(I, UseMI);

    if (UseHardStack)
      Builder.buildInstr(MOS::PH).addUse(MOS::A);
    else
      Builder.buildInstr(MOS::STAbs)
          .addUse(MOS::A)
          .addExternalSymbol("__save_a");

    Builder.setInsertPt(MBB, UseMI);

    if (UseHardStack)
      Builder.buildInstr(MOS::PL).addDef(MOS::A);
    else
      Builder.buildInstr(MOS::LDAbs)
          .addDef(MOS::A)
          .addExternalSymbol("__save_a");
    break;
  }
  case MOS::X:
  case MOS::XLSB:
  case MOS::Y:
  case MOS::YLSB: {
    const char *Save = Reg == MOS::X ? "__save_x" : "__save_y";
    Builder.buildInstr(MOS::STAbs).addUse(Reg).addExternalSymbol(Save);

    Builder.setInsertPt(MBB, UseMI);
    Builder.buildInstr(MOS::LDAbs).addDef(Reg).addExternalSymbol(Save);
    break;
  }
  case MOS::P: {
    LiveRegUnits LiveUnits(TRI);
    LiveUnits.addLiveOuts(MBB);
    if (UseMI != MBB.end()) {
      for (auto J = std::prev(MBB.end()); J != UseMI; --J)
        LiveUnits.stepBackward(*J);
      LiveUnits.stepBackward(*UseMI);
    }

    assert(pushPullBalanced(I, UseMI));
    const bool CLive = !LiveUnits.available(MOS::C);
    const bool VLive = !LiveUnits.available(MOS::V);
    assert(CLive || VLive);
    auto Save = Builder.buildInstr(MOS::PH, {}, {Register(MOS::P)});
    Builder.setInsertPt(MBB, UseMI);
    Builder.buildInstr(MOS::PL, {MOS::P}, {});
    if (CLive && !VLive) {
      Save->getOperand(0).setIsUndef();
      Save.addUse(MOS::C, RegState::Implicit);
    } else if (!CLive && VLive) {
      Save->getOperand(0).setIsUndef();
      Save.addUse(MOS::V, RegState::Implicit);
    }
    break;
  }
  }

  return true;
}

void MOSRegisterInfo::eliminateFrameIndex(MachineBasicBlock::iterator MI,
                                          int SPAdj, unsigned FIOperandNum,
                                          RegScavenger *RS) const {
  MachineFunction &MF = *MI->getMF();
  const MachineFrameInfo &MFI = MF.getFrameInfo();
  const TargetInstrInfo &TII = *MF.getSubtarget().getInstrInfo();

  assert(!SPAdj);

  int Idx = MI->getOperand(FIOperandNum).getIndex();
  int64_t Offset = MFI.getObjectOffset(Idx);
  if (FIOperandNum + 1 < MI->getNumOperands() &&
      MI->getOperand(FIOperandNum + 1).isImm())
    Offset += MI->getOperand(FIOperandNum + 1).getImm();

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
    llvm_unreachable("Unexpected opcode.");
  case MOS::AddrLostk:
  case MOS::AddrHistk:
  case MOS::LDStk:
  case MOS::STStk:
    MI->getOperand(FIOperandNum)
        .ChangeToRegister(getFrameRegister(MF), /*isDef=*/false);
    MI->getOperand(FIOperandNum + 1).setImm(Offset);
    break;
  case MOS::LDAbsOffset:
  case MOS::STAbsOffset:
    MI->getOperand(FIOperandNum)
        .ChangeToTargetIndex(MOS::TI_STATIC_STACK, Offset);
    MI->RemoveOperand(FIOperandNum + 1);
    break;
  case MOS::LDImm:
    MI->getOperand(FIOperandNum)
        .ChangeToTargetIndex(MOS::TI_STATIC_STACK, Offset,
                             MI->getOperand(FIOperandNum).getTargetFlags());
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
  case MOS::LDAbsOffset:
    MI->setDesc(TII.get(MOS::LDAbs));
    break;
  case MOS::STAbsOffset:
    MI->setDesc(TII.get(MOS::STAbs));
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

  Builder.buildInstr(IsLoad ? MOS::LDYIndir : MOS::STYIndir)
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
  return TFI->hasFP(MF) ? MOS::RS2 : MOS::RS0;
}

bool MOSRegisterInfo::shouldCoalesce(
    MachineInstr *MI, const TargetRegisterClass *SrcRC, unsigned SubReg,
    const TargetRegisterClass *DstRC, unsigned DstSubReg,
    const TargetRegisterClass *NewRC, LiveIntervals &LIS) const {
  // Don't coalesce Imag8 and AImag8 registers together, since this may cause
  // expensive ASL zp's to be used when ASL A would have sufficed. It's better
  // to do arithmetic in A and then copy it out.
  if (NewRC == &MOS::Imag8RegClass &&
      (SrcRC == &MOS::AImag8RegClass || DstRC == &MOS::AImag8RegClass))
    return false;
  return true;
}
