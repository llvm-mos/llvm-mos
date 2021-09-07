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
#include "llvm/CodeGen/MachineBasicBlock.h"

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
  void lowerSelect(MachineInstr &MI);
  void moveAwayFromCalls(MachineFunction &MF);
};

bool MOSLowerSelect::runOnMachineFunction(MachineFunction &MF) {

  moveAwayFromCalls(MF);

  SmallVector<MachineBasicBlock::iterator> SelectMBBIs;
  for (auto I = MF.begin(), E = MF.end(); I != E; ++I) {
    MachineBasicBlock &MBB = *I;
    for (MachineBasicBlock::iterator MBBI = MBB.begin(), MBBE = MBB.end();
         MBBI != MBBE; ++MBBI)
      if (MBBI->getOpcode() == MOS::G_SELECT)
        SelectMBBIs.push_back(MBBI);
  }

  bool Changed = !SelectMBBIs.empty();

  for (MachineBasicBlock::iterator MBBI : SelectMBBIs)
    lowerSelect(*MBBI);

  return Changed;
}

void MOSLowerSelect::lowerSelect(MachineInstr &MI) {
  assert(MI.getOpcode() == MOS::G_SELECT);
  Register Dst = MI.getOperand(0).getReg();
  Register Tst = MI.getOperand(1).getReg();
  Register TrueValue = MI.getOperand(2).getReg();
  Register FalseValue = MI.getOperand(3).getReg();

  MachineIRBuilder Builder(MI);
  MachineBasicBlock &MBB = Builder.getMBB();
  MachineFunction &MF = Builder.getMF();

  // To lower a G_SELECT instruction, we actually have to insert the diamond
  // control-flow pattern. The incoming instruction knows the destination
  // vreg to set, the condition to branch on, and the true/false values to
  // select between.
  const BasicBlock *LLVM_BB = MBB.getBasicBlock();
  MachineFunction::iterator It = std::next(MBB.getIterator());

  //  thisMBB:
  //   ...
  //   %TrueValue = ...
  //   %FalseValue = ...
  //   ...
  //   G_BRCOND_IMM %Tst, %TrueMBB, 1
  //   G_BR --> %FalseMBB
  MachineBasicBlock *TrueMBB = MF.CreateMachineBasicBlock(LLVM_BB);
  MachineBasicBlock *FalseMBB = MF.CreateMachineBasicBlock(LLVM_BB);
  MachineBasicBlock *SinkMBB = MF.CreateMachineBasicBlock(LLVM_BB);
  MF.insert(It, TrueMBB);
  MF.insert(It, FalseMBB);
  MF.insert(It, SinkMBB);

  // Transfer the remainder of MBB and its successor edges to SinkMBB.
  SinkMBB->splice(SinkMBB->begin(), &MBB, std::next(MI.getIterator()),
                  MBB.end());
  SinkMBB->transferSuccessorsAndUpdatePHIs(&MBB);

  // Next, add the True and False blocks as its successors.
  MBB.addSuccessor(TrueMBB);
  MBB.addSuccessor(FalseMBB);
  Builder.buildInstr(MOS::G_BRCOND_IMM, {}, {Tst}).addMBB(TrueMBB).addImm(1);
  Builder.buildInstr(MOS::G_BR).addMBB(FalseMBB);

  // The True and False blocks both jump through to the Sink block.
  TrueMBB->addSuccessor(SinkMBB);
  FalseMBB->addSuccessor(SinkMBB);
  Builder.setInsertPt(*TrueMBB, TrueMBB->begin());
  Builder.buildInstr(MOS::G_BR).addMBB(SinkMBB);
  Builder.setInsertPt(*FalseMBB, FalseMBB->begin());
  Builder.buildInstr(MOS::G_BR).addMBB(SinkMBB);

  //  SinkMBB:
  //   %Result = phi [ %TrueValue, TrueMBB ], [ %FalseValue, FalseMBB ]
  //  ...
  Builder.setInsertPt(*SinkMBB, SinkMBB->begin());
  Builder.buildInstr(MOS::G_PHI)
      .addDef(Dst)
      .addUse(TrueValue)
      .addMBB(TrueMBB)
      .addUse(FalseValue)
      .addMBB(FalseMBB);

  MI.eraseFromParent(); // The G_SELECT is gone now.
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
