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
};

bool MOSLateOptimization::runOnMachineFunction(MachineFunction &MF) {
  // Right now, the only optimization done is to elide CMPImm 0 whenever it's
  // obviously safe to do so.

  const auto *TRI = MF.getSubtarget().getRegisterInfo();
  bool Changed = false;
  for (MachineBasicBlock &MBB : MF) {
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
