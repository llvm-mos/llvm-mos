//===-- MOSLateOptimization.cpp - MOS Late Optimization -------------------===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file defines the MOS late optimization pass.
//
// This pass performs simple optimizations once pseudo-instructions have been
// fully lowered. These optimizations might otherwise increase register pressure
// and cause spills, so they're done opportunistically at the very end.
//
//===----------------------------------------------------------------------===//

#include "MOSLateOptimization.h"

#include "MCTargetDesc/MOSMCTargetDesc.h"
#include "MOS.h"
#include "MOSRegisterInfo.h"

#include "llvm/CodeGen/GlobalISel/MachineIRBuilder.h"
#include "llvm/CodeGen/LivePhysRegs.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/TargetInstrInfo.h"
#include "llvm/CodeGen/TargetSubtargetInfo.h"

#define DEBUG_TYPE "mos-late-opt"

using namespace llvm;

namespace {

class MOSLateOptimization : public MachineFunctionPass {
public:
  static char ID;

  MOSLateOptimization() : MachineFunctionPass(ID) {
    llvm::initializeMOSLateOptimizationPass(*PassRegistry::getPassRegistry());
  }

  bool runOnMachineFunction(MachineFunction &MF) override;
  bool lowerCMPZTerms(MachineBasicBlock &MBB) const;
  void lowerCMPZTerm(MachineInstr &MI) const;
  bool ldImmToInxyDexy(MachineBasicBlock &MBB) const;
};

bool MOSLateOptimization::runOnMachineFunction(MachineFunction &MF) {
  bool Changed = false;
  for (MachineBasicBlock &MBB : MF) {
    Changed |= lowerCMPZTerms(MBB);
    Changed |= ldImmToInxyDexy(MBB);
  }
  return Changed;
}

static bool definesNZ(const MachineInstr &MI, Register Val) {
  if (MI.getOpcode() == MOS::STImag8)
    return false;
  if (MI.definesRegister(Val))
    return true;
  switch (MI.getOpcode()) {
  default:
    return false;
  case MOS::TA:
  case MOS::T_A:
  case MOS::LDImag8:
    return MI.getOperand(1).getReg() == Val;
  }
}

bool MOSLateOptimization::lowerCMPZTerms(MachineBasicBlock &MBB) const {
  const auto &MRI = MBB.getParent()->getRegInfo();
  const auto *TRI = MRI.getTargetRegisterInfo();
  bool Changed = false;
  MachineBasicBlock::reverse_iterator Next;
  for (auto I = MBB.rbegin(), E = MBB.rend(); I != E; I = Next) {
    Next = std::next(I);

    if (I->getOpcode() != MOS::CMPZTerm)
      continue;
    assert(I->getOperand(0).isDead());

    Register Val = I->getOperand(1).getReg();

    for (auto J = Next; J != E; ++J) {
      if (J->isCall())
        break;
      if (definesNZ(*J, Val)) {
        Changed = true;
        J->addOperand(MachineOperand::CreateReg(MOS::NZ, /*isDef=*/true,
                                                /*isImp=*/true));
        I->eraseFromParent();
        break;
      }
      if (J->modifiesRegister(MOS::NZ, TRI))
        break;
      bool ClobbersNZ = true;
      if (J->isBranch() || J->mayStore())
        ClobbersNZ = false;
      else
        switch (J->getOpcode()) {
        case MOS::CLV:
        case MOS::LDCImm:
        case MOS::STImag8:
        case MOS::PH:
          ClobbersNZ = false;
          break;
        }
      if (ClobbersNZ)
        break;
    }
    if (Changed)
      continue;

    Changed = true;
    lowerCMPZTerm(*I);
  }
  return Changed;
}

void MOSLateOptimization::lowerCMPZTerm(MachineInstr &MI) const {
  auto &MBB = *MI.getParent();
  const auto &MRI = MBB.getParent()->getRegInfo();
  const auto *TRI = MRI.getTargetRegisterInfo();
  Register Val = MI.getOperand(1).getReg();

  LivePhysRegs PhysRegs;
  PhysRegs.init(*TRI);
  PhysRegs.addLiveOuts(MBB);
  for (auto J = MBB.rbegin(); J != MI; ++J)
    PhysRegs.stepBackward(*J);

  MachineIRBuilder Builder(MBB, MI);
  switch (Val) {
  default: {
    assert(MOS::Imag8RegClass.contains(Val));
    Register Tmp = 0;
    if (PhysRegs.available(MRI, MOS::A))
      Tmp = MOS::A;
    else if (PhysRegs.available(MRI, MOS::X))
      Tmp = MOS::X;
    else if (PhysRegs.available(MRI, MOS::Y))
      Tmp = MOS::Y;
    MachineInstrBuilder Access;
    if (Tmp) {
      Access = Builder.buildInstr(MOS::LDImag8, {Tmp}, {Val});
    } else {
      SmallSet<Register, 3> Defined;

      // At this point, unless we can find a GPR with the value, it'll take 4
      // bytes and 10 cycles for an INC ZP DEC ZP. So look really hard for the
      // value in a GPR.
      for (auto J = std::next(MachineBasicBlock::reverse_iterator(MI)),
                E = MBB.rend();
           J != E; ++J) {
        if (J->getOpcode() == MOS::LDImag8 &&
            J->getOperand(1).getReg() == Val &&
            !Defined.contains(J->getOperand(0).getReg())) {
          Register GPR = J->getOperand(0).getReg();
          MI.getOperand(1).setReg(GPR);
          lowerCMPZTerm(MI);
          return;
        }

        // The only way to set an imaginary register is by STImag8 and
        // operations that set NZ. The latter case is handled by wholly eliding
        // the comparison, so we only need to worry about the former. In the
        // former case, we're guaranteed that there are no modifications of Val
        // after the first STImag8 we find, so there's no need to do any sort of
        // liveness tracking on Val.
        if (J->getOpcode() == MOS::STImag8 &&
            J->getOperand(0).getReg() == Val &&
            !Defined.contains(J->getOperand(1).getReg())) {
          Register GPR = J->getOperand(1).getReg();
          MI.getOperand(1).setReg(GPR);
          lowerCMPZTerm(MI);
          return;
        }
        if (J->modifiesRegister(MOS::A, TRI))
          Defined.insert(MOS::A);
        if (J->modifiesRegister(MOS::X, TRI))
          Defined.insert(MOS::X);
        if (J->modifiesRegister(MOS::Y, TRI))
          Defined.insert(MOS::Y);
        if (Defined.size() == 3)
          break;
      }

      Builder.buildInstr(MOS::INCImag8, {Val}, {Val});
      Access = Builder.buildInstr(MOS::DECImag8, {Val}, {Val});
    }
    Access.addDef(MOS::NZ, RegState::Implicit);
    Access->getOperand(0).setIsDead();
    break;
  }
  case MOS::A: {
    MachineInstrBuilder Access;
    if (PhysRegs.available(MRI, MOS::X))
      Access = Builder.buildInstr(MOS::TA, {MOS::X}, {Register(MOS::A)});
    else if (PhysRegs.available(MRI, MOS::Y))
      Access = Builder.buildInstr(MOS::TA, {MOS::Y}, {Register(MOS::A)});
    else {
      Access = Builder.buildInstr(MOS::CMPImm, {MOS::C},
                                  {Register(MOS::A), INT64_C(0)});
    }
    Access.addDef(MOS::NZ, RegState::Implicit);
    Access->getOperand(0).setIsDead();
    break;
  }
  case MOS::X:
  case MOS::Y: {
    MachineInstrBuilder Access;
    if (PhysRegs.available(MRI, MOS::A))
      Access = Builder.buildInstr(MOS::T_A, {MOS::A}, {Val});
    else
      Access = Builder.buildInstr(MOS::CMPImm, {MOS::C}, {Val, INT64_C(0)});
    Access.addDef(MOS::NZ, RegState::Implicit);
    Access->getOperand(0).setIsDead();
    break;
  }
  }
  MI.eraseFromParent();
}

bool MOSLateOptimization::ldImmToInxyDexy(MachineBasicBlock &MBB) const {
  const auto &TII = *MBB.getParent()->getSubtarget().getInstrInfo();
  const auto *TRI = MBB.getParent()->getSubtarget().getRegisterInfo();

  bool Changed = false;

  struct ImmLoad {
    MachineInstr *MI = nullptr;
    int64_t Val;
  } ConstX, ConstY;

  for (MachineInstr &MI : MBB) {
    if (MI.getOpcode() != MOS::LDImm || !MI.getOperand(1).isImm()) {
      if (MI.modifiesRegister(MOS::X, TRI))
        ConstX.MI = nullptr;
      if (MI.modifiesRegister(MOS::Y, TRI))
        ConstY.MI = nullptr;
      continue;
    }

    Register Dst = MI.getOperand(0).getReg();
    int64_t Val = MI.getOperand(1).getImm();
    ImmLoad *Load = nullptr;
    switch (Dst) {
    default:
      continue;
    case MOS::X:
      Load = &ConstX;
      break;
    case MOS::Y:
      Load = &ConstY;
      break;
    }

    if (Load->MI) {
      bool Reduced = false;
      if (Val == Load->Val + 1) {
        MI.setDesc(TII.get(MOS::IN));
        Reduced = true;
      } else if (Val == Load->Val - 1) {
        MI.setDesc(TII.get(MOS::DE));
        Reduced = true;
      }
      if (Reduced) {
        Changed = true;
        Load->MI->getOperand(0).setIsDead(false);
        for (MachineBasicBlock::iterator J = Load->MI, JE = MI; J != JE; ++J)
          J->clearRegisterKills(Dst, TRI);
        MI.getOperand(1).ChangeToRegister(Dst, /*isDef=*/false, /*isImp=*/false,
                                          /*isKill=*/true);
        MI.tieOperands(0, 1);
      }
    }

    Load->MI = &MI;
    Load->Val = Val;
  }
  return Changed;
}

} // namespace

char MOSLateOptimization::ID = 0;

INITIALIZE_PASS(MOSLateOptimization, DEBUG_TYPE, "MOS Late Optimizations",
                false, false)

MachineFunctionPass *llvm::createMOSLateOptimizationPass() {
  return new MOSLateOptimization;
}
