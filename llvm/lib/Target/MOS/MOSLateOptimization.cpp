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

#include "llvm/CodeGen/MachineBasicBlock.h"
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
  bool elideCMPImm0(MachineBasicBlock &MBB) const;
  bool ldImmToInxyDexy(MachineBasicBlock &MBB) const;
};

bool MOSLateOptimization::runOnMachineFunction(MachineFunction &MF) {
  bool Changed = false;
  for (MachineBasicBlock &MBB : MF) {
    Changed |= elideCMPImm0(MBB);
    Changed |= ldImmToInxyDexy(MBB);
  }
  return Changed;
}

bool MOSLateOptimization::elideCMPImm0(MachineBasicBlock &MBB) const {
  const auto *TRI = MBB.getParent()->getSubtarget().getRegisterInfo();
  bool Changed = false;
  MachineBasicBlock::reverse_iterator Next;
  for (auto I = MBB.rbegin(), E = MBB.rend(); I != E; I = Next) {
    Next = std::next(I);

    if (I->getOpcode() != MOS::CMPImm)
      continue;
    if (I->getOperand(2).getImm() != 0)
      continue;
    if (!I->getOperand(0).isDead())
      continue;

    Register Val = I->getOperand(1).getReg();

    for (auto J = Next; J != E; ++J) {
      if (J->isCall())
        break;
      if (J->definesRegister(Val)) {
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
  }
  return Changed;
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
