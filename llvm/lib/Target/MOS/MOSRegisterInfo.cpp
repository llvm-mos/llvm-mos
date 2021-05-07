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
#include "llvm/Support/ErrorHandling.h"

#define DEBUG_TYPE "mos-reginfo"

#define GET_REGINFO_TARGET_DESC
#include "MOSGenRegisterInfo.inc"

using namespace llvm;

cl::opt<int>
    NumImagPtrs("num-imag-ptrs", cl::init(127),
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
    Reserved.set(getSubReg(Ptr, MOS::sublo));
    Reserved.set(getSubReg(Ptr, MOS::subhi));
    Reserved.set(getSubReg(Ptr, MOS::sublsb));
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
  switch (Reg) {
  default:
    llvm_unreachable("Unexpected scavenger register.");
  case MOS::A:
    Builder.buildInstr(MOS::PH).addUse(MOS::A);

    Builder.setInsertPt(MBB, UseMI);
    Builder.buildInstr(MOS::PL).addDef(MOS::A);
    break;
  case MOS::X:
  case MOS::Y:
    const char *Save = Reg == MOS::X ? "__save_x" : "__save_y";
    Builder.buildInstr(MOS::STAbs).addUse(Reg).addExternalSymbol(Save);

    Builder.setInsertPt(MBB, UseMI);
    Builder.buildInstr(MOS::LDAbs).addDef(Reg).addExternalSymbol(Save);
    break;
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
  if (MI->getOperand(FIOperandNum + 1).isImm())
    Offset += MI->getOperand(FIOperandNum + 1).getImm();

  if (MFI.getStackID(Idx) == TargetStackID::Default) {
    // Real Address = Offset Relative to Incoming SP + Incoming SP
    // Frame Pointer = Incoming SP - Stack Size
    // Real Address = Frame Pointer + Offset
    // Substituting gives:
    // Offset Relative to Incoming SP + Incoming SP = Incoming SP - Stack Size +
    // Offset Rearranging gives: Offset = Offset Relative to Incoming SP + Stack
    // Size

    // Thus, effective offset relative to the frame pointer, we need to add in
    // the stack size.
    Offset += MFI.getStackSize();
  }

  switch (MI->getOpcode()) {
  default:
    llvm_unreachable("Unexpected opcode.");
  case MOS::AddrLostk:
  case MOS::AddrHistk:
  case MOS::LDstk:
  case MOS::STstk:
    MI->getOperand(FIOperandNum)
        .ChangeToRegister(getFrameRegister(MF), /*isDef=*/false);
    MI->getOperand(FIOperandNum + 1).setImm(Offset);
    break;
  case MOS::LDabs_offset:
  case MOS::STabs_offset:
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
  case MOS::LDabs_offset:
    MI->setDesc(TII.get(MOS::LDAbs));
    break;
  case MOS::STabs_offset:
    MI->setDesc(TII.get(MOS::STAbs));
    break;
  case MOS::LDstk:
  case MOS::STstk:
    expandLDSTstk(MI);
    break;
  }
}

void MOSRegisterInfo::expandAddrLostk(MachineBasicBlock::iterator MI) const {
  MachineIRBuilder Builder(*MI->getParent(), MI);
  const TargetRegisterInfo &TRI =
      *Builder.getMF().getSubtarget().getRegisterInfo();

  MachineOperand Dst = MI->getOperand(0);
  Register Base = MI->getOperand(3).getReg();

  int64_t OffsetImm = MI->getOperand(4).getImm();
  assert(0 <= OffsetImm && OffsetImm < 65536);
  auto Offset = static_cast<uint16_t>(OffsetImm);
  Offset &= 0xFF;

  Register Src = TRI.getSubReg(Base, MOS::sublo);

  Builder.buildInstr(MOS::LDCImm).addDef(MOS::C).addImm(0);

  if (!Offset)
    Builder.buildInstr(MOS::COPY).add(Dst).addUse(Src);
  else {
    Register A = Builder.buildCopy(&MOS::AcRegClass, Src).getReg(0);
    auto Instr = Builder.buildInstr(MOS::ADCImm, {A, MOS::C, MOS::V},
                                    {A, int64_t(Offset), Register(MOS::C)});
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
  Register Base = MI->getOperand(3).getReg();

  int64_t OffsetImm = MI->getOperand(4).getImm();
  assert(0 <= OffsetImm && OffsetImm < 65536);
  auto Offset = static_cast<uint16_t>(OffsetImm);

  Register Src = TRI.getSubReg(Base, MOS::subhi);

  // Note: We can only elide the high byte of the address into a copy if the
  // whole offset is zero. There may be a carry from the low byte sum if only
  // the high byte is zero.
  if (!Offset)
    Builder.buildInstr(MOS::COPY).add(Dst).addUse(Src);
  else {
    Register A = Builder.buildCopy(&MOS::AcRegClass, Src).getReg(0);
    auto Instr =
        Builder.buildInstr(MOS::ADCImm, {A, MOS::C, MOS::V},
                           {A, int64_t(Offset >> 8), Register(MOS::C)});
    Instr->getOperand(1).setIsDead();
    Instr->getOperand(2).setIsDead();
    Builder.buildInstr(MOS::COPY).add(Dst).addUse(A);
  }

  MI->eraseFromParent();
}

void MOSRegisterInfo::expandLDSTstk(MachineBasicBlock::iterator MI) const {
  MachineFunction &MF = *MI->getMF();
  MachineIRBuilder Builder(*MI->getParent(), MI);
  MachineRegisterInfo &MRI = *Builder.getMRI();
  const TargetRegisterInfo &TRI = *MRI.getTargetRegisterInfo();

  const bool IsLoad = MI->getOpcode() == MOS::LDstk;

  Register Loc = MI->getOperand(0).getReg();
  int64_t Offset = MI->getOperand(2).getImm();

  if (Offset >= 256) {
    // Far stack accesses need a virtual base register, so materialize one here.
    Register NewBase = MRI.createVirtualRegister(&MOS::Imag16RegClass);
    Register CLo = MRI.createVirtualRegister(&MOS::CcRegClass);
    Register CHi = MRI.createVirtualRegister(&MOS::CcRegClass);
    Register VLo = MRI.createVirtualRegister(&MOS::VcRegClass);
    Register VHi = MRI.createVirtualRegister(&MOS::VcRegClass);
    auto Lo = Builder.buildInstr(MOS::AddrLostk)
                  .addDef(NewBase, /*Flags=*/0, MOS::sublo)
                  .addDef(CLo)
                  .addDef(VLo, RegState::Dead)
                  .add(MI->getOperand(1))
                  .add(MI->getOperand(2));
    auto Hi = Builder.buildInstr(MOS::AddrHistk)
                  .addDef(NewBase, /*Flags=*/0, MOS::subhi)
                  .addDef(CHi, RegState::Dead)
                  .addDef(VHi, RegState::Dead)
                  .add(MI->getOperand(1))
                  .add(MI->getOperand(2))
                  .addUse(CLo)
                  .addUse(NewBase, RegState::Implicit);
    MI->getOperand(1).setReg(NewBase);
    MI->getOperand(2).setImm(0);

    expandAddrLostk(Lo);

    MachineInstrSpan MIS(Hi, Hi->getParent());
    expandAddrHistk(Hi);
    for (auto &MI : MIS) {
      if (MI.modifiesRegister(NewBase, &TRI)) {
        // Keep scavenger from complaining about multiple definitions. We
        // instead consider them all redefinitions of the original set by
        // AddrLostk.
        MI.addOperand(MachineOperand::CreateReg(NewBase, /*isDef=*/false,
                                                /*isImp=*/true));
      }
    }

    expandLDSTstk(MI);
    return;
  }

  if (MOS::Imag16RegClass.contains(Loc)) {
    Register Lo = TRI.getSubReg(Loc, MOS::sublo);
    Register Hi = TRI.getSubReg(Loc, MOS::subhi);
    auto LoInstr = Builder.buildInstr(MI->getOpcode())
                       .addReg(Lo, getDefRegState(IsLoad))
                       .add(MI->getOperand(1))
                       .add(MI->getOperand(2))
                       .addMemOperand(MF.getMachineMemOperand(
                           *MI->memoperands_begin(), 0, 1));
    auto HiInstr = Builder.buildInstr(MI->getOpcode())
                       .addReg(Hi, getDefRegState(IsLoad))
                       .add(MI->getOperand(1))
                       .addImm(MI->getOperand(2).getImm() + 1)
                       .addMemOperand(MF.getMachineMemOperand(
                           *MI->memoperands_begin(), 1, 1));
    MI->eraseFromParent();
    expandLDSTstk(LoInstr);
    expandLDSTstk(HiInstr);
    return;
  }

  if(!MOS::Anyi8RegClass.contains(Loc)) {
    errs() << *MI;
    report_fatal_error("LDSTstk not yet implemented.");
  }

  Register A = Loc;
  if (A != MOS::A)
    A = MRI.createVirtualRegister(&MOS::AcRegClass);

  // Transfer the value to A to be stored (if applicable).
  if (!IsLoad && Loc != A)
    Builder.buildCopy(A, Loc);

  // This needs to occur after the above copy since the source may be Y.
  Register Y =
      Builder.buildInstr(MOS::LDImm, {&MOS::YcRegClass}, {Offset}).getReg(0);

  Builder.buildInstr(IsLoad ? MOS::LDYIndir : MOS::STYIndir)
      .addReg(A, getDefRegState(IsLoad))
      .add(MI->getOperand(1))
      .addUse(Y)
      .addMemOperand(*MI->memoperands_begin());

  // Transfer the loaded value out of A (if applicable).
  if (IsLoad && Loc != A)
    Builder.buildCopy(Loc, A);

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
