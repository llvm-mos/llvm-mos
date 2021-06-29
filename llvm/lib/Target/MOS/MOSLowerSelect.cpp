//===-- MOSLowerSelect.cpp - MOS Select Lowering --------------------------===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file defines the MOS select pseudo lowering pass.
//
//===----------------------------------------------------------------------===//

#include "MOSLowerSelect.h"

#include "MCTargetDesc/MOSMCTargetDesc.h"
#include "MOS.h"
#include "llvm/CodeGen/GlobalISel/MachineIRBuilder.h"

#define DEBUG_TYPE "mos-lower-select"

using namespace llvm;

namespace {

class MOSLowerSelect : public MachineFunctionPass {
public:
  static char ID;

  MOSLowerSelect() : MachineFunctionPass(ID) {
    llvm::initializeMOSLowerSelectPass(*PassRegistry::getPassRegistry());
  }

  MachineFunctionProperties getRequiredProperties() const override {
    return MachineFunctionProperties()
        .set(MachineFunctionProperties::Property::IsSSA)
        .set(MachineFunctionProperties::Property::Legalized);
  }

  MachineFunctionProperties getClearedProperties() const override {
    return MachineFunctionProperties().set(
        MachineFunctionProperties::Property::NoPHIs);
  }

  bool runOnMachineFunction(MachineFunction &MF) override;
  void moveAwayFromCalls(MachineFunction &MF);
};

bool MOSLowerSelect::runOnMachineFunction(MachineFunction &MF) {
  bool Changed = false;

  moveAwayFromCalls(MF);

  for (auto I = MF.begin(), E = MF.end(); I != E; ++I) {
    MachineBasicBlock &MBB = *I;
    for (MachineBasicBlock::iterator MBBI = MBB.begin(), MBBE = MBB.end();
         MBBI != MBBE;) {
      MachineInstr &MI = *MBBI;
      if (MI.getOpcode() != MOS::G_SELECT) {
        ++MBBI;
        continue;
      }

      Changed = true;

      Register Dst = MI.getOperand(0).getReg();
      Register Tst = MI.getOperand(1).getReg();
      Register TrueValue = MI.getOperand(2).getReg();
      Register FalseValue = MI.getOperand(3).getReg();

      MachineIRBuilder Builder(MI);

      // To lower a G_SELECT instruction, we actually have to insert the diamond
      // control-flow pattern. The incoming instruction knows the destination
      // vreg to set, the condition to branch on, and the true/false values to
      // select between.
      const BasicBlock *LLVM_BB = MBB.getBasicBlock();
      MachineFunction::iterator It = std::next(I);

      //  thisMBB:
      //   ...
      //   %FalseValue = ...
      //   %TrueValue = ...
      //   ...
      //   G_BRCOND_IMM %Tst, %sinkMBB, 1
      //   fallthrough --> %copy0MBB
      MachineBasicBlock *copy0MBB = MF.CreateMachineBasicBlock(LLVM_BB);
      MachineBasicBlock *sinkMBB = MF.CreateMachineBasicBlock(LLVM_BB);
      MF.insert(It, copy0MBB);
      MF.insert(It, sinkMBB);

      // Transfer the remainder of MBB and its successor edges to sinkMBB.
      sinkMBB->splice(sinkMBB->begin(), &MBB, std::next(MBBI), MBBE);
      sinkMBB->transferSuccessorsAndUpdatePHIs(&MBB);

      // Next, add the true and fallthrough blocks as its successors.
      MBB.addSuccessor(copy0MBB);
      MBB.addSuccessor(sinkMBB);

      Builder.buildInstr(MOS::G_BRCOND_IMM, {}, {Tst})
          .addMBB(sinkMBB)
          .addImm(1);

      //  copy0MBB:
      //   %FalseValue = ...
      //   # fallthrough to sinkMBB
      // Update machine-CFG edges
      copy0MBB->addSuccessor(sinkMBB);

      //  sinkMBB:
      //   %Result = phi [ %TrueValue, thisMBB ], [ %FalseValue, copy0MBB ]
      //  ...
      Builder.setInsertPt(*sinkMBB, sinkMBB->begin());
      Builder.buildInstr(MOS::G_PHI)
          .addDef(Dst)
          .addUse(TrueValue)
          .addMBB(&MBB)
          .addUse(FalseValue)
          .addMBB(copy0MBB);

      MI.eraseFromParent(); // The G_SELECT is gone now.

      // New MBBs were added, but they cannot have any G_SELECTs in them.
      // Continue processing the first instruction in sinkMBB.
      I = copy0MBB->getIterator();
      break;
    }
  }

  return Changed;
}

// Before lowering selects, they and all attached instructions need to be
// moved outside of call regions. Otherwise, they can create live physical
// registers in basic blocks that are not entries, which is illegal in SSA
// form.
void MOSLowerSelect::moveAwayFromCalls(MachineFunction &MF) {
  for (MachineBasicBlock &MBB : MF) {
    for (auto I = MBB.begin(), E = MBB.end(); I != E; ++I) {
      if (I->getOpcode() != MOS::JSR)
        continue;

      SmallVector<MachineInstr *> PushedMIs;
      SmallSet<Register,
               CalculateSmallVectorDefaultInlinedElements<Register>::value>
          UsedRegs;

      const auto DefinesUsedReg = [&](const MachineInstr &MI) {
        for (const MachineOperand &MO : MI.operands()) {
          if (!MO.isReg() || !MO.isDef())
            continue;
          if (UsedRegs.contains(MO.getReg()))
            return true;
        }
        return false;
      };

      const auto TrackUsedRegs = [&](const MachineInstr &MI) {
        for (const MachineOperand &MO : MI.operands()) {
          if (!MO.isReg() || !MO.isUse() || !MO.getReg().isVirtual())
            continue;
          UsedRegs.insert(MO.getReg());
        }
      };

      auto J = std::prev(I);
      for (; J->getOpcode() != MOS::ADJCALLSTACKDOWN; --J) {
        if (J->getOpcode() == MOS::G_SELECT || DefinesUsedReg(*J)) {
          TrackUsedRegs(*J);
          auto NewJ = std::next(J);
          PushedMIs.push_back(J->removeFromParent());
          J = NewJ;
        }
      }
      while (!PushedMIs.empty()) {
        MBB.insert(J, PushedMIs.back());
        PushedMIs.pop_back();
      }

// G_SELECTs should never appear in the return value part of calls, since
// they're used for extensions, not truncations.
#ifndef NDEBUG
     for (auto J = std::next(I); J->getOpcode() != MOS::ADJCALLSTACKUP; ++J)
       assert(J->getOpcode() != MOS::G_SELECT);
#endif
    }
  }
}

} // namespace

char MOSLowerSelect::ID = 0;

INITIALIZE_PASS(MOSLowerSelect, DEBUG_TYPE,
                "Lower MOS Select pseudo-instruction", false, false)

MachineFunctionPass *llvm::createMOSLowerSelectPass() {
  return new MOSLowerSelect();
}
