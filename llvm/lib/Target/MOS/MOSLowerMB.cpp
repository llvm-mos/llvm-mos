//===-- MOSLowerMB.cpp - MOS Late Optimization -------------------===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file defines the MOS multi-byte operation lowering pass.
//
//===----------------------------------------------------------------------===//

#include "MOSLowerMB.h"

#include "MCTargetDesc/MOSMCTargetDesc.h"
#include "MOS.h"
#include "MOSRegisterInfo.h"

#include "llvm/ADT/STLExtras.h"
#include "llvm/CodeGen/GlobalISel/MachineIRBuilder.h"
#include "llvm/CodeGen/GlobalISel/Utils.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/TargetInstrInfo.h"
#include "llvm/CodeGen/TargetRegisterInfo.h"
#include "llvm/Support/ErrorHandling.h"

#define DEBUG_TYPE "mos-lower-mb"

using namespace llvm;

MOSLowerMB::MOSLowerMB() : MachineFunctionPass(ID) {
  llvm::initializeMOSLowerMBPass(*PassRegistry::getPassRegistry());
}

bool MOSLowerMB::runOnMachineFunction(MachineFunction &MF) {
  bool Changed = false;
  for (MachineBasicBlock &MBB : make_early_inc_range(MF)) {
    DeadIncs.clear();
    for (MachineInstr &MI : make_early_inc_range(reverse(MBB))) {
      switch (MI.getOpcode()) {
      case MOS::CMPTermZMB16:
      case MOS::CMPTermZMB32:
      case MOS::CMPTermZMB64:
        Changed = true;
        lowerCMPTermZ(MI);
        break;
      case MOS::IncMB16:
      case MOS::IncMB32:
      case MOS::IncMB64:
      case MOS::DecMB16:
      case MOS::DecMB32:
      case MOS::DecMB64:
        Changed = true;
        lowerIncDecMB(MI);
        break;
      }
    }
  }

  rewriteMBRegs(MF);
  return Changed;
}

static unsigned SubRegIndices[] = {MOS::sublo, MOS::subhi, MOS::sub2,
                                   MOS::sub3,  MOS::sub4,  MOS::sub5,
                                   MOS::sub6,  MOS::sub7};

void MOSLowerMB::lowerCMPTermZ(MachineInstr &MI) {
  MachineFunction &MF = *MI.getMF();
  MachineBasicBlock *MBB = MI.getParent();
  const BasicBlock *BB = MBB->getBasicBlock();
  const TargetInstrInfo &TII = *MI.getMF()->getSubtarget().getInstrInfo();

  bool HasInc = false;
  if (MachineInstr *Inc = cmpTermZInc(MI)) {
    HasInc = true;
    DeadIncs.insert(Inc);
  }

  unsigned NumBytes;
  switch (MI.getOpcode()) {
  default:
    llvm_unreachable("Unexpected opcode.");
  case MOS::CMPTermZMB16:
    NumBytes = 2;
    break;
  case MOS::CMPTermZMB32:
    NumBytes = 4;
    break;
  case MOS::CMPTermZMB64:
    NumBytes = 8;
    break;
  }

  Register Reg = MI.getOperand(1).getReg();

  MachineBasicBlock *TBB;
  MachineBasicBlock *FBB;
  SmallVector<MachineOperand> Cond;
  if (TII.analyzeBranch(*MBB, TBB, FBB, Cond))
    llvm_unreachable("Could not analyze branch.");
  assert(TBB);
  assert(FBB);
  assert(Cond.size() == 2);
  assert(Cond[0].getReg() == MOS::Z);

  if (Cond[1].getImm()) {
    std::swap(TBB, FBB);
    TII.reverseBranchCondition(Cond);
  }
  assert(!Cond[1].getImm());

  TII.removeBranch(*MBB);
  MBB->removeSuccessor(TBB);
  MBB->removeSuccessor(FBB);
  MI.eraseFromParent();

  auto AfterMBBIter = std::next(MBB->getIterator());

  // Compare bytes to zero from the high to low.
  int Begin = HasInc ? 0 : NumBytes - 1;
  int End = HasInc ? NumBytes : -1;
  int Last = HasInc ? End - 1 : End + 1;
  for (int I = Begin; I != End; HasInc ? ++I : --I) {
    unsigned SubRegIdx = SubRegIndices[I];
    MachineIRBuilder Builder(*MBB, MBB->end());

    if (HasInc)
      Builder.buildInstr(MOS::INC)
          .addDef(Reg, 0, SubRegIdx)
          .addUse(Reg, 0, SubRegIdx);

    Builder.buildInstr(MOS::CMPTermZ, {&MOS::CcRegClass}, {})
        .addUse(Reg, 0, SubRegIdx);

    // If any of the bytes are not zero, then the whole is not zero and the true
    // path can be taken.
    if (I != Last) {
      MachineBasicBlock *NextMBB = MF.CreateMachineBasicBlock(BB);
      MF.insert(AfterMBBIter, NextMBB);
      MBB->addSuccessor(TBB);
      MBB->addSuccessor(NextMBB);
      TII.insertBranch(*MBB, TBB, NextMBB, Cond, Builder.getDebugLoc());
      MBB = NextMBB;
    } else {
      MBB->addSuccessor(TBB);
      MBB->addSuccessor(FBB);
      TII.insertBranch(*MBB, TBB, FBB, Cond, Builder.getDebugLoc());
    }
  }
}

static bool isInc(unsigned Opcode) {
  switch (Opcode) {
  default:
    return false;
  case MOS::IncMB16:
  case MOS::IncMB32:
  case MOS::IncMB64:
    return true;
  }
}

MachineInstr *MOSLowerMB::cmpTermZInc(MachineInstr &MI) {
  Register Val = MI.getOperand(1).getReg();
  for (auto I = MachineBasicBlock::reverse_iterator(MI.getIterator()),
            E = MI.getParent()->rend();
       I != E; ++I) {
    if (I->readsRegister(Val) || I->definesRegister(Val))
      return isInc(I->getOpcode()) ? &*I : nullptr;
  }
  return nullptr;
}

void MOSLowerMB::lowerIncDecMB(MachineInstr &MI) {
  if (DeadIncs.contains(&MI)) {
    MI.eraseFromParent();
    return;
  }

  MachineFunction &MF = *MI.getMF();
  MachineBasicBlock *MBB = MI.getParent();
  const BasicBlock *BB = MBB->getBasicBlock();

  Register Reg = MI.getOperand(0).getReg();

  unsigned NumBytes;
  switch (MI.getOpcode()) {
  default:
    llvm_unreachable("Unexpected opcode.");
  case MOS::IncMB16:
  case MOS::DecMB16:
    NumBytes = 2;
    break;
  case MOS::IncMB32:
  case MOS::DecMB32:
    NumBytes = 4;
    break;
  case MOS::IncMB64:
  case MOS::DecMB64:
    NumBytes = 8;
    break;
  }

  bool IsInc;
  switch (MI.getOpcode()) {
  default:
    llvm_unreachable("Unexpected opcode.");
  case MOS::IncMB16:
  case MOS::IncMB32:
  case MOS::IncMB64:
    IsInc = true;
    break;
  case MOS::DecMB16:
  case MOS::DecMB32:
  case MOS::DecMB64:
    IsInc = false;
    break;
  }

  MachineBasicBlock *AfterMBB = MBB->splitAt(MI);
  if (AfterMBB == MBB)
    AfterMBB = &*std::next(MBB->getIterator());
  MBB->removeSuccessor(AfterMBB);
  MI.eraseFromParent();

  for (unsigned I = 0; I < NumBytes; ++I) {
    unsigned SubRegIdx = SubRegIndices[I];
    MachineIRBuilder Builder(*MBB, MBB->end());

    Register Copy;
    if (!IsInc)
      Copy = Builder.buildInstr(MOS::COPY, {&MOS::GPRRegClass}, {})
                 .addUse(Reg, 0, SubRegIdx)
                 .getReg(0);

    Builder.buildInstr(IsInc ? MOS::INC : MOS::DEC)
        .addDef(Reg, 0, SubRegIdx)
        .addUse(Reg, 0, SubRegIdx);

    if (I != NumBytes - 1) {
      MachineBasicBlock *NextMBB = MF.CreateMachineBasicBlock(BB);
      MF.insert(AfterMBB->getIterator(), NextMBB);
      auto Cmp = Builder.buildInstr(MOS::CMPTermZ, {&MOS::CcRegClass}, {});
      if (IsInc)
        Cmp.addUse(Reg, 0, SubRegIdx);
      else
        Cmp.addUse(Copy);
      Builder.buildInstr(MOS::BR).addMBB(AfterMBB).addUse(MOS::Z).addImm(0);
      Builder.buildInstr(MOS::JMP).addMBB(NextMBB);
      MBB->addSuccessor(AfterMBB);
      MBB->addSuccessor(NextMBB);
      MBB = NextMBB;
    } else {
      Builder.buildInstr(MOS::JMP).addMBB(AfterMBB);
      MBB->addSuccessor(AfterMBB);
    }
  }
}

void MOSLowerMB::rewriteMBRegs(MachineFunction &MF) {
  MachineRegisterInfo &MRI = MF.getRegInfo();

  for (unsigned I = 0, E = MRI.getNumVirtRegs(); I != E; ++I) {
    Register Reg = Register::index2VirtReg(I);
    if (MRI.use_empty(Reg))
      continue;

    const TargetRegisterClass *RC = MRI.getRegClass(Reg);
    unsigned Size;
    if (RC == &MOS::Anyi16RegClass)
      Size = 2;
    else if (RC == &MOS::MB32RegClass)
      Size = 4;
    else if (RC == &MOS::MB64RegClass)
      Size = 8;
    else
      continue;

    SmallVector<Register> VRegs;
    for (unsigned I = 0; I < Size; ++I)
      VRegs.push_back(MRI.createGenericVirtualRegister(LLT::scalar(8)));

    const TargetInstrInfo &TII = *MF.getSubtarget().getInstrInfo();
    const TargetRegisterInfo &TRI = *MF.getSubtarget().getRegisterInfo();
    const RegisterBankInfo &RBI = *MF.getSubtarget().getRegBankInfo();
    for (MachineOperand &MO :
         make_early_inc_range(MRI.reg_nodbg_operands(Reg))) {
      switch (MO.getSubReg()) {
      default:
        llvm_unreachable("Unexpected subregister index.");
      case MOS::sublo:
        MO.setReg(VRegs[0]);
        break;
      case MOS::subhi:
        MO.setReg(VRegs[1]);
        break;
      case MOS::sub2:
        MO.setReg(VRegs[2]);
        break;
      case MOS::sub3:
        MO.setReg(VRegs[3]);
        break;
      case MOS::sub4:
        MO.setReg(VRegs[4]);
        break;
      case MOS::sub5:
        MO.setReg(VRegs[5]);
        break;
      case MOS::sub6:
        MO.setReg(VRegs[6]);
        break;
      case MOS::sub7:
        MO.setReg(VRegs[7]);
        break;
      }
      MO.setSubReg(0);
      MO.setIsUndef(false);
      constrainOperandRegClass(MF, TRI, MRI, TII, RBI, *MO.getParent(),
                               MO.getParent()->getDesc(), MO,
                               &MO - &MO.getParent()->getOperand(0));
    }
  }
}

char MOSLowerMB::ID = 0;

INITIALIZE_PASS(MOSLowerMB, DEBUG_TYPE, "MOS Lower Multi-byte", false, false)
