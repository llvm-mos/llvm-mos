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
#include "llvm/CodeGen/TargetFrameLowering.h"
#include "llvm/CodeGen/TargetInstrInfo.h"
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
  }

  // Reserve stack pointers.
  Reserved.set(MOS::RS0);
  Reserved.set(MOS::RC0);
  Reserved.set(MOS::RC1);
  Reserved.set(MOS::S);
  Reserved.set(MOS::Static);
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
  if (RC->contains(MOS::C))
    return &MOS::Anyi1RegClass;
  if (RC == &MOS::Imag16RegClass)
    return RC;
  return &MOS::Anyi8RegClass;
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
    Builder.buildInstr(MOS::PHA);

    Builder.setInsertPt(MBB, UseMI);
    Builder.buildInstr(MOS::PLA);
    break;
  case MOS::X:
  case MOS::Y:
    const char *Save = Reg == MOS::X ? "_SaveX" : "_SaveY";
    Builder.buildInstr(MOS::STabs).addUse(Reg).addExternalSymbol(Save);

    Builder.setInsertPt(MBB, UseMI);
    Builder.buildInstr(MOS::LDabs).addDef(Reg).addExternalSymbol(Save);
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

  int64_t StackSize;
  Register Base;
  switch (MFI.getStackID(Idx)) {
  default:
    llvm_unreachable("Unexpected Stack ID");
  case TargetStackID::Default:
    StackSize = MFI.getStackSize();
    Base = getFrameRegister(MF);
    break;
  case TargetStackID::NoAlloc:
    Base = MOS::Static;
    // Static stack grows up, so its offsets are positive.
    // Zeroing this allows the offsets through unchanged.
    StackSize = 0;
    break;
  }

  // FIXME: Remove this hack once we've ported away from the old stack pseudos.
  if (MI->getOpcode() == MOS::LDabs_offset ||
      MI->getOpcode() == MOS::STabs_offset) {
    assert(Base == MOS::Static);
    assert(MI->getOperand(FIOperandNum + 1).isImm());
    MI->getOperand(FIOperandNum)
        .ChangeToTargetIndex(MOS::TI_STATIC_STACK,
                             MFI.getObjectOffset(Idx) +
                                 MI->getOperand(FIOperandNum + 1).getImm());
    MI->RemoveOperand(FIOperandNum + 1);
    MI->setDesc(TII.get(MI->getOpcode() == MOS::LDabs_offset ? MOS::LDabs
                                                             : MOS::STabs));
  } else {
    MI->getOperand(FIOperandNum).ChangeToRegister(Base, /*isDef=*/false);
    assert(MI->getOperand(FIOperandNum + 1).isImm());
    MI->getOperand(FIOperandNum + 1)
        .setImm(StackSize + MFI.getObjectOffset(Idx));
  }
}

Register MOSRegisterInfo::getFrameRegister(const MachineFunction &MF) const {
  const TargetFrameLowering *TFI = getFrameLowering(MF);
  return TFI->hasFP(MF) ? MOS::RS2 : MOS::RS0;
}

bool MOSRegisterInfo::shouldCoalesce(
    MachineInstr *MI, const TargetRegisterClass *SrcRC, unsigned SubReg,
    const TargetRegisterClass *DstRC, unsigned DstSubReg,
    const TargetRegisterClass *NewRC, LiveIntervals &LIS) const {
  if (NewRC == &MOS::Imag8RegClass &&
      (SrcRC == &MOS::AImag8RegClass || DstRC == &MOS::AImag8RegClass))
    return false;
  return true;
}
