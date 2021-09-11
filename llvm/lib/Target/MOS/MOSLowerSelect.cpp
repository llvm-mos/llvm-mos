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
#include "llvm/CodeGen/MachineOperand.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"

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

// Returns whether the only use of a G_SELECT's result is the then or else part
// of another G_SELECT. In that case, the former G_SELECT can be sunk into the
// latter G_SELECT so long as the latter is expanded first.
static bool onlyUseIsConditional(const MachineInstr &MI,
                                 const MachineRegisterInfo &MRI) {
  assert(MI.getOpcode() == MOS::G_SELECT);
  Register Dst = MI.getOperand(0).getReg();
  if (!MRI.hasOneNonDBGUse(Dst))
    return false;
  const MachineInstr &OnlyUseMI = *MRI.use_nodbg_instructions(Dst).begin();
  return OnlyUseMI.getOpcode() == MOS::G_SELECT &&
         OnlyUseMI.getOperand(1).getReg() != Dst;
}

bool MOSLowerSelect::runOnMachineFunction(MachineFunction &MF) {
  LLVM_DEBUG(dbgs() << "Handling G_SELECTs in: " << MF.getName() << "\n\n");
  moveAwayFromCalls(MF);

  SmallVector<MachineBasicBlock::iterator> SelectMBBIs;
  for (auto I = MF.begin(), E = MF.end(); I != E; ++I) {
    MachineBasicBlock &MBB = *I;
    for (MachineBasicBlock::iterator MBBI = MBB.begin(), MBBE = MBB.end();
         MBBI != MBBE; ++MBBI)
      if (MBBI->getOpcode() == MOS::G_SELECT) {
        LLVM_DEBUG(dbgs() << "Found: " << *MBBI);
        SelectMBBIs.push_back(MBBI);
      }
  }

  bool Changed = !SelectMBBIs.empty();

  LLVM_DEBUG(dbgs() << "\nIteratively lowering G_SELECTs.\n\n");

  SmallVector<MachineBasicBlock::iterator> NewSelectMBBIs;
  MachineRegisterInfo &MRI = MF.getRegInfo();
  // Lower G_SELECTs in multiple passes to ensure the ordering is optimal.
  while (!SelectMBBIs.empty()) {
    NewSelectMBBIs.clear();
    LLVM_DEBUG(dbgs() << "Begin iteration.\n");
    for (MachineBasicBlock::iterator MBBI : SelectMBBIs) {
      LLVM_DEBUG(dbgs() << *MBBI);

      // If the only use of this instruction is conditioned on another G_SELECT,
      // then this whole control flow section can be sunk into that conditional
      // region. This can only occur if the other G_SELECT is lowered first, so
      // skip this one for this pass. The actual sinking occurs when that
      // G_SELECT is lowered, so by the time the next expansion pass occurs, the
      // conditional G_SELECT is already in the correct basic block.
      if (onlyUseIsConditional(*MBBI, MRI)) {
        LLVM_DEBUG(
            dbgs() << "\tG_SELECT only used conditionally inside another "
                      "G_SELECT. Deferring.\n");
        NewSelectMBBIs.push_back(MBBI);
      } else {
        LLVM_DEBUG(dbgs() << "\tLowering.\n");
        lowerSelect(*MBBI);
      }
    }
    LLVM_DEBUG(dbgs() << "End iteration.\n\n");
    if (NewSelectMBBIs.size() == SelectMBBIs.size())
      report_fatal_error("Infinite loop encountered while lowering G_SELECT.");
    std::swap(SelectMBBIs, NewSelectMBBIs);
  }

  LLVM_DEBUG(dbgs() << "\n");

  return Changed;
}

Register getPhiValue(const MachineInstr &Phi, const MachineBasicBlock *MBB) {
  assert(Phi.getOpcode() == MOS::G_PHI);
  for (unsigned Idx = 1, End = Phi.getNumOperands(); Idx != End; Idx += 2)
    if (Phi.getOperand(Idx + 1).getMBB() == MBB)
      return Phi.getOperand(Idx).getReg();
  llvm_unreachable("Could not find MBB in G_PHI.");
}

bool referencesSuccessor(const MachineBasicBlock &MBB,
                         const MachineBasicBlock *Tgt) {
  for (const MachineInstr &MI : MBB.terminators())
    for (const MachineOperand &MO : MI.operands())
      if (MO.isMBB() && MO.getMBB() == Tgt)
        return true;
  return false;
}

void removePredecessorFromPhis(MachineBasicBlock *MBB,
                               const MachineBasicBlock *PredMBB) {
  for (MachineInstr &Phi : MBB->phis())
    for (unsigned Idx = 1; Idx < Phi.getNumOperands();)
      if (Phi.getOperand(Idx + 1).getMBB() == PredMBB) {
        Phi.RemoveOperand(Idx);
        Phi.RemoveOperand(Idx);
      } else
        Idx += 2;
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
  const MachineRegisterInfo &MRI = *Builder.getMRI();

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

  // Sink the True and False values if G_SELECTs. This causes all inserted
  // control flow for the inner G_SELECT to be conditionalized on the outer. A
  // global ordering constraint ensures the the outer G_SELECT is always
  // expanded first.
  const auto SinkValue = [&](MachineBasicBlock *MBB, Register Value) {
    if (MRI.hasOneNonDBGUse(Value)) {
      MachineInstr &DefMI = *MRI.def_instr_begin(Value);
      if (DefMI.getOpcode() == MOS::G_SELECT) {
        DefMI.removeFromParent();
        MBB->insert(MBB->end(), &DefMI);
      }
    }
  };
  SinkValue(TrueMBB, TrueValue);
  SinkValue(FalseMBB, FalseValue);

  bool FoldedUse = false;
  // A select is commonly used to branch on a complex condition. If the next
  // instruction is a branch, and this is the only use of the select, then
  // duplicate the conditional branch into the true and false basic blocks. This
  // saves the PHI.
  if (MRI.hasOneNonDBGUse(Dst)) {
    MachineInstr &UseMI = *MRI.use_instr_nodbg_begin(Dst);
    if (UseMI.getIterator() == SinkMBB->begin() &&
        UseMI.getOpcode() == MOS::G_BRCOND_IMM) {
      MachineBasicBlock *Tgt = UseMI.getOperand(1).getMBB();

      FoldedUse = true;
      MachineInstr *TrueUseMI = UseMI.removeFromParent();
      MachineInstr *FalseUseMI = MF.CloneMachineInstr(&UseMI);

      const auto InsertUseMI = [&](MachineBasicBlock *MBB, MachineInstr *UseMI, Register Value) {
        MBB->insert(MBB->end(), UseMI);
        if (!MBB->isSuccessor(Tgt)) {
          MBB->addSuccessor(Tgt);
          for (MachineInstr &Phi : Tgt->phis()) {
            Phi.addOperand(MachineOperand::CreateReg(getPhiValue(Phi, SinkMBB),
                                                    /*isDef=*/false));
            Phi.addOperand(MachineOperand::CreateMBB(MBB));
          }
        }
        MRI.use_nodbg_begin(Dst)->setReg(Value);
      };
      InsertUseMI(TrueMBB, TrueUseMI, TrueValue);
      InsertUseMI(FalseMBB, FalseUseMI, FalseValue);

      if (!referencesSuccessor(*SinkMBB, Tgt)) {
        SinkMBB->removeSuccessor(Tgt, /*NormalizeProbs=*/true);
        removePredecessorFromPhis(Tgt, SinkMBB);
      }
    }
  }

  TrueMBB->addSuccessor(SinkMBB);
  FalseMBB->addSuccessor(SinkMBB);
  // The True and False blocks both jump through to the Sink block.
  Builder.setInsertPt(*TrueMBB, TrueMBB->end());
  Builder.buildInstr(MOS::G_BR).addMBB(SinkMBB);
  Builder.setInsertPt(*FalseMBB, FalseMBB->end());
  Builder.buildInstr(MOS::G_BR).addMBB(SinkMBB);

  if (!FoldedUse) {
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
  }
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
          // Conservatively assume there was a store.
          bool SawStore = true;
          assert(J->isSafeToMove(nullptr, SawStore));
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
