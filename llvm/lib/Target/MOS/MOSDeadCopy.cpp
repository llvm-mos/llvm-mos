//===-- MOSDeadCopy.cpp - MOS Dead Copy Elimination -----------------------===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file defines the MOS pass to elminiate dead COPY operations before COPYs
// are lowered.
//
//===----------------------------------------------------------------------===//

#include "MOSDeadCopy.h"

#include "MOS.h"
#include "llvm/ADT/PostOrderIterator.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/CodeGen/LivePhysRegs.h"
#include "llvm/CodeGen/TargetRegisterInfo.h"
#include "llvm/CodeGen/TargetSubtargetInfo.h"

#define DEBUG_TYPE "mos-dead-copy"

using namespace llvm;

namespace {

class MOSDeadCopy : public MachineFunctionPass {
public:
  static char ID;

  MOSDeadCopy() : MachineFunctionPass(ID) {
    llvm::initializeMOSDeadCopyPass(*PassRegistry::getPassRegistry());
  }

  bool runOnMachineFunction(MachineFunction &MF) override;
};

} // namespace

bool MOSDeadCopy::runOnMachineFunction(MachineFunction &MF) {
  const TargetRegisterInfo &TRI = *MF.getSubtarget().getRegisterInfo();
  for (MachineBasicBlock *MBB : post_order(&MF)) {
    LivePhysRegs LPR(TRI);
    MBB->clearLiveIns();
    computeAndAddLiveIns(LPR, *MBB);
    recomputeLivenessFlags(*MBB);
    for (MachineInstr &MI : make_early_inc_range(*MBB)) {
      if (MI.isCopy() && MI.getOperand(0).isDead()) {
        LLVM_DEBUG(dbgs() << "Erasing dead copy: " << MI);
        MI.eraseFromParent();
      }
    }
  }
  return true;
}

char MOSDeadCopy::ID = 0;

INITIALIZE_PASS(MOSDeadCopy, DEBUG_TYPE, "Eliminate dead copies for MOS", false,
                false)

MachineFunctionPass *llvm::createMOSDeadCopyPass() { return new MOSDeadCopy(); }
