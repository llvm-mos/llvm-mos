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
#include "llvm/CodeGen/GlobalISel/Utils.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineOperand.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/TargetInstrInfo.h"

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
  void sinkSelectsToBranchUses(MachineFunction &MF);
  MachineFunction::reverse_iterator lowerSelect(MachineInstr &MI);
  void moveAwayFromCalls(MachineFunction &MF);
};

bool MOSLowerSelect::runOnMachineFunction(MachineFunction &MF) {
  LLVM_DEBUG(dbgs() << "\n\nHandling G_SELECTs in: " << MF.getName() << "\n\n");
  moveAwayFromCalls(MF);
  sinkSelectsToBranchUses(MF);

  bool Changed = false;
  for (auto I = MF.rbegin(), E = MF.rend(); I != E; ++I) {
    MachineBasicBlock &MBB = *I;
    for (auto MBBI = MBB.rbegin(), MBBE = MBB.rend(); MBBI != MBBE; ++MBBI) {
      if (MBBI->getOpcode() == MOS::G_SELECT) {
        LLVM_DEBUG(dbgs() << "Lowering: " << *MBBI);
        Changed = true;
        I = lowerSelect(*MBBI);
        break;
      }
    }
  }
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

MachineFunction::reverse_iterator
MOSLowerSelect::lowerSelect(MachineInstr &MI) {
  assert(MI.getOpcode() == MOS::G_SELECT);
  Register Dst = MI.getOperand(0).getReg();
  Register Tst = MI.getOperand(1).getReg();
  Register TrueValue = MI.getOperand(2).getReg();
  Register FalseValue = MI.getOperand(3).getReg();

  MachineIRBuilder Builder(MI);
  MachineBasicBlock &MBB = Builder.getMBB();
  MachineFunction &MF = Builder.getMF();
  const MachineRegisterInfo &MRI = *Builder.getMRI();

  SmallVector<Register> Dsts = {Dst};
  SmallVector<Register> TrueValues = {TrueValue};
  SmallVector<Register> FalseValues = {FalseValue};
  SmallVector<MachineBasicBlock::iterator> SelectsToRemove = {MI};

  // Collect G_SELECT instructions in the same basic block with the same test
  // and merge them into this one.
  SmallSet<Register, 8> UsedRegs;
  for (const auto &MO : MI.operands())
    if (MO.isReg() && MO.isUse())
      UsedRegs.insert(MO.getReg());
  for (auto MBBI = std::next(MachineBasicBlock::reverse_iterator(MI)),
            MBBE = MBB.rend();
       MBBI != MBBE; ++MBBI) {
    for (const auto &MO : MBBI->operands())
      if (MO.isReg() && MO.isUse())
        UsedRegs.insert(MO.getReg());

    if (MBBI->getOpcode() == MOS::G_SELECT &&
        MBBI->getOperand(1).getReg() == Tst &&
        !UsedRegs.contains(MBBI->getOperand(0).getReg())) {
      LLVM_DEBUG(dbgs() << "Absorbing select with same test: " << *MBBI);
      Dsts.push_back(MBBI->getOperand(0).getReg());
      TrueValues.push_back(MBBI->getOperand(2).getReg());
      FalseValues.push_back(MBBI->getOperand(3).getReg());
      SelectsToRemove.push_back(*MBBI);
    }
  }
  assert(Dsts.size() == TrueValues.size());
  assert(TrueValues.size() == FalseValues.size());

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

  // Sink the True and False values if only used in the conditional part of the
  // G_SELECT.
  const auto SinkValue = [&](MachineBasicBlock *MBB, Register Value) {
    if (!MRI.hasOneNonDBGUse(Value))
      return;
    MachineInstr &DefMI = *MRI.def_instr_begin(Value);
    bool SawStore = true;
    if (!DefMI.isSafeToMove(nullptr, SawStore))
      return;

    for (const MachineOperand &MO : DefMI.operands()) {
      if (!MO.isReg())
        continue;
      if (MO.getReg().isPhysical())
        return;
      // Note: Value is already determined to have exactly one non-dbg use.
      if (!MO.isDef() || MO.getReg() == Value)
        continue;
      // Can't sink DefMI if it defines registers used outside the select.
      if (!MRI.use_nodbg_empty(MO.getReg()))
        return;
    }
    if (Tst == Value)
      return;
    LLVM_DEBUG(dbgs() << "Sinking value: " << DefMI);

    auto SrcMI =
        MachineBasicBlock::reverse_iterator(MachineBasicBlock::iterator(DefMI));
    auto SrcEnd = DefMI.getParent()->rend();
    DefMI.removeFromParent();
    MBB->insert(MBB->begin(), &DefMI);

    for (; SrcMI != SrcEnd; ++SrcMI) {
      SawStore = true;
      if (!SrcMI->isSafeToMove(nullptr, SawStore))
        continue;

      LLVM_DEBUG(dbgs() << "Considering sinking: " << *SrcMI);

      // Can't sink SrcMI if any defined operand is used outside of the
      // conditional MBB.
      for (const MachineOperand &MO : SrcMI->operands()) {
        if (!MO.isReg())
          continue;
        if (MO.getReg().isPhysical())
          goto cont;
        if (!MO.isDef())
          continue;
        for (const MachineInstr &UseMI :
             MRI.use_nodbg_instructions(MO.getReg()))
          if (UseMI.getParent() != MBB)
            goto cont;
      }
      {
        MachineInstr *MIToSink = &*SrcMI;
        LLVM_DEBUG(dbgs() << "Sinking value: " << *MIToSink);
        --SrcMI; // To be incremented on loop.
        MIToSink->removeFromParent();
        MBB->insert(MBB->begin(), MIToSink);
      }
    cont:;
    }
  };
  for (Register TrueValue : TrueValues)
    SinkValue(TrueMBB, TrueValue);
  for (Register FalseValue : FalseValues)
    SinkValue(FalseMBB, FalseValue);

  bool FoldedUse = false;
  // A select is commonly used to branch on a complex condition. If the next
  // instruction is a branch, and this is the only use of the select, then
  // duplicate the conditional branch into the true and false basic blocks. This
  // saves the PHI.
  if (Dsts.size() == 1 && MRI.hasOneNonDBGUse(Dst)) {
    MachineInstr &UseMI = *MRI.use_instr_nodbg_begin(Dst);
    if (UseMI.getIterator() == SinkMBB->begin() &&
        UseMI.getOpcode() == MOS::G_BRCOND_IMM) {
      LLVM_DEBUG(dbgs() << "Folding use MI: " << UseMI);
      MachineBasicBlock *Tgt = UseMI.getOperand(1).getMBB();

      FoldedUse = true;
      MachineInstr *TrueUseMI = UseMI.removeFromParent();
      MachineInstr *FalseUseMI = MF.CloneMachineInstr(&UseMI);

      const auto InsertUseMI = [&](MachineBasicBlock *MBB, MachineInstr *UseMI,
                                   Register Value) {
        auto ConstVal = getIConstantVRegValWithLookThrough(Value, MRI);
        if (ConstVal) {
          if (ConstVal->Value.getBoolValue() !=
              static_cast<bool>(UseMI->getOperand(2).getImm())) {
            LLVM_DEBUG(dbgs() << "User branch cannot be taken; eliding.\n");
            return;
          }
          LLVM_DEBUG(dbgs()
                     << "User branch is always taken. Making unconditional.\n");
          UseMI->setDesc(Builder.getTII().get(MOS::G_BR));
          UseMI->RemoveOperand(2);
          UseMI->RemoveOperand(0);
        } else
          UseMI->getOperand(0).setReg(Value);
        MBB->insert(MBB->end(), UseMI);
        if (!MBB->isSuccessor(Tgt)) {
          MBB->addSuccessor(Tgt);
          for (MachineInstr &Phi : Tgt->phis()) {
            Phi.addOperand(MachineOperand::CreateReg(getPhiValue(Phi, SinkMBB),
                                                     /*isDef=*/false));
            Phi.addOperand(MachineOperand::CreateMBB(MBB));
          }
        }
      };
      LLVM_DEBUG(dbgs() << "Creating True MBB:\n");
      InsertUseMI(TrueMBB, TrueUseMI, TrueValue);
      LLVM_DEBUG(dbgs() << "Creating False MBB:\n");
      InsertUseMI(FalseMBB, FalseUseMI, FalseValue);

      if (!referencesSuccessor(*SinkMBB, Tgt)) {
        SinkMBB->removeSuccessor(Tgt, /*NormalizeProbs=*/true);
        removePredecessorFromPhis(Tgt, SinkMBB);
      }
    }
  }

  // The True and False blocks both jump through to the Sink block.
  if (TrueMBB->empty() ||
      !TrueMBB->getLastNonDebugInstr()->isUnconditionalBranch()) {
    TrueMBB->addSuccessor(SinkMBB);
    Builder.setInsertPt(*TrueMBB, TrueMBB->end());
    Builder.buildInstr(MOS::G_BR).addMBB(SinkMBB);
  }
  if (FalseMBB->empty() ||
      !FalseMBB->getLastNonDebugInstr()->isUnconditionalBranch()) {
    FalseMBB->addSuccessor(SinkMBB);
    Builder.setInsertPt(*FalseMBB, FalseMBB->end());
    Builder.buildInstr(MOS::G_BR).addMBB(SinkMBB);
  }

  if (!FoldedUse) {
    //  SinkMBB:
    //   %Result = phi [ %TrueValue, TrueMBB ], [ %FalseValue, FalseMBB ]
    //  ...
    Builder.setInsertPt(*SinkMBB, SinkMBB->begin());
    for (unsigned Idx = 0, End = Dsts.size(); Idx != End; ++Idx) {
      Register Dst = Dsts[Idx];
      Register TrueValue = TrueValues[Idx];
      Register FalseValue = FalseValues[Idx];
      Builder.buildInstr(MOS::G_PHI)
          .addDef(Dst)
          .addUse(TrueValue)
          .addMBB(TrueMBB)
          .addUse(FalseValue)
          .addMBB(FalseMBB);
    }
  }
  for (const auto Select : SelectsToRemove)
    Select->eraseFromParent();
  return MachineFunction::reverse_iterator(*SinkMBB);
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

void MOSLowerSelect::sinkSelectsToBranchUses(MachineFunction &MF) {
  const auto &MRI = MF.getRegInfo();
  for (MachineBasicBlock &MBB : MF) {
    MachineBasicBlock::reverse_iterator NextI;
    for (auto I = MBB.rbegin(), E = MBB.rend(); I != E; I = NextI) {
      NextI = std::next(I);
      if (I->getOpcode() != MOS::G_SELECT)
        continue;
      Register Dst = I->getOperand(0).getReg();
      if (!MRI.hasOneNonDBGUse(Dst))
        continue;
      auto &UseMI = *MRI.use_instr_nodbg_begin(Dst);
      if (UseMI.getOpcode() != MOS::G_BRCOND_IMM)
        continue;
      if (UseMI.getParent() != &MBB)
        continue;
      I->removeFromParent();
      MBB.insert(UseMI, &*I);
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
