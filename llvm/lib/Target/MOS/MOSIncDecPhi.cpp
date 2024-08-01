//===-- MOSIncDecPhi.cpp - MOS Increment Decrement PHI --------------------===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file defines the MOS pass to separate an increment/decrement from an
// ADC of a PHI of -1 or 1.
//
//===----------------------------------------------------------------------===//

#include "MOSIncDecPhi.h"

#include "MCTargetDesc/MOSMCTargetDesc.h"
#include "MOS.h"

#include "llvm/CodeGen/GlobalISel/MachineIRBuilder.h"
#include "llvm/CodeGen/GlobalISel/Utils.h"
#include "llvm/CodeGen/MachineDominators.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/InitializePasses.h"

#define DEBUG_TYPE "mos-incdecphi"

using namespace llvm;

namespace {

class MOSIncDecPhi : public MachineFunctionPass {
public:
  static char ID;

  MOSIncDecPhi() : MachineFunctionPass(ID) {
    llvm::initializeMOSIncDecPhiPass(*PassRegistry::getPassRegistry());
  }

  void getAnalysisUsage(AnalysisUsage &AU) const override {
    AU.addRequired<MachineDominatorTreeWrapperPass>();
    MachineFunctionPass::getAnalysisUsage(AU);
  }

  bool runOnMachineFunction(MachineFunction &MF) override;
};

} // namespace

static bool matchPhiOneNegOne(const MachineInstr &MI,
                              const MachineRegisterInfo &MRI,
                              MachineBasicBlock *&IncMBB, Register &OneReg,
                              MachineBasicBlock *&DecMBB, Register &NegOneReg) {
  if (MI.getOpcode() != MOS::G_PHI)
    return false;
  if (MI.getNumOperands() != 5)
    return false;
  bool HasOneReg = false;
  bool HasNegOneReg = false;
  for (unsigned I = 1, E = 5; I != E; I += 2) {
    Register R = MI.getOperand(I).getReg();
    std::optional<ValueAndVReg> V = getIConstantVRegValWithLookThrough(R, MRI);
    if (!V || !V->Value.abs().isOne())
      return false;
    if (V->Value.isOne()) {
      HasOneReg = true;
      OneReg = MI.getOperand(I).getReg();
      IncMBB = MI.getOperand(I + 1).getMBB();
    } else {
      HasNegOneReg = true;
      NegOneReg = MI.getOperand(I).getReg();
      DecMBB = MI.getOperand(I + 1).getMBB();
    }
  }
  return HasOneReg && HasNegOneReg;
}

static MachineInstr *matchFoldableStore(Register Dst,
                                        const MachineInstr &ValDef,
                                        const MachineRegisterInfo &MRI) {
  if (!MRI.hasOneNonDBGUse(Dst))
    return nullptr;
  MachineInstr &MI = *MRI.use_instr_nodbg_begin(Dst);
  if (MI.getOpcode() != MOS::G_STORE)
    return nullptr;
  assert(MI.getOperand(0).getReg() == Dst);
  if (ValDef.getOpcode() != MOS::G_LOAD)
    return nullptr;
  if (MI.getOperand(1).getReg() != ValDef.getOperand(1).getReg())
    return nullptr;
  if (MI.hasOrderedMemoryRef())
    return nullptr;
  return &MI;
}

static MachineBasicBlock *splitCriticalEdge(MachineBasicBlock &MBB,
                                            MachineBasicBlock &Succ) {
  MachineFunction &MF = *MBB.getParent();

  if (std::prev(MBB.end())->getOpcode() != MOS::G_BR) {
    MachineIRBuilder MIB(MBB, MBB.end());
    MIB.buildBr(Succ);
  }

  MachineBasicBlock *Split = MF.CreateMachineBasicBlock(MBB.getBasicBlock());
  MachineIRBuilder MIB(*Split, Split->begin());
  MIB.buildBr(Succ);
  Split->addSuccessor(&Succ);
  MF.insert(std::next(MBB.getIterator()), Split);

  assert(std::prev(MBB.end())->getOpcode() == MOS::G_BR);
  MBB.ReplaceUsesOfBlockWith(&Succ, Split);
  Succ.replacePhiUsesWith(&MBB, Split);

  return Split;
}

bool MOSIncDecPhi::runOnMachineFunction(MachineFunction &MF) {
  MachineRegisterInfo &MRI = MF.getRegInfo();
  bool Changed = false;
  const auto &MDT = getAnalysis<MachineDominatorTreeWrapperPass>().getDomTree();
  for (MachineBasicBlock &MBB : MF) {
    for (auto I = MBB.begin(), E = MBB.end(); I != E; ++I) {
      MachineInstr &MI = *I;
      if (MI.getOpcode() != MOS::G_ADD)
        continue;

      Register Dst = MI.getOperand(0).getReg();
      LLT Ty = MRI.getType(Dst);

      MachineBasicBlock *IncMBB;
      Register OneReg;
      MachineBasicBlock *DecMBB;
      Register NegOneReg;
      Register Val = MI.getOperand(1).getReg();
      const MachineInstr *Phi = MRI.getUniqueVRegDef(MI.getOperand(2).getReg());
      if (!matchPhiOneNegOne(*Phi, MRI, IncMBB, OneReg, DecMBB, NegOneReg)) {
        Phi = MRI.getUniqueVRegDef(MI.getOperand(1).getReg());
        Val = MI.getOperand(2).getReg();
        if (!matchPhiOneNegOne(*Phi, MRI, IncMBB, OneReg, DecMBB, NegOneReg))
          continue;
      }

      // If the value's definition is in this basic block, then it is
      // unavailable in the predecessors. In that case, check that it can be
      // safely hoisted and duplicated.
      MachineInstr *ValDef = MRI.getOneDef(Val)->getParent();
      MachineBasicBlock *ValDefMBB = ValDef->getParent();
      if (ValDefMBB == &MBB) {
        bool SawStore = false;
        if (!ValDef->isSafeToMove(SawStore))
          continue;
        if (!MRI.hasOneNonDBGUse(Val))
          continue;
        bool CanHoist = true;
        for (const MachineOperand &MO : ValDef->uses()) {
          if (MRI.getOneDef(MO.getReg())->getParent()->getParent() == &MBB) {
            CanHoist = false;
            break;
          }
        }
        if (!CanHoist)
          continue;
      } else {
        // If the value is defined in some other basic block, we don't try to
        // rematerialize it. Instead, just check that it's available in both
        // predecessors.
        if (!MDT.dominates(ValDefMBB, IncMBB) ||
            !MDT.dominates(ValDefMBB, DecMBB))
          continue;
      }

      MachineInstr *FoldableStore = matchFoldableStore(Dst, *ValDef, MRI);

      // If one of the parent edges is critical, then lifting across it
      // deconditionalizes computation, since the parent may branch away from
      // the increment/decrement. Avoid this by splitting the edge.
      if (IncMBB->succ_size() != 1)
        IncMBB = splitCriticalEdge(*IncMBB, MBB);
      assert(IncMBB->succ_size() == 1);
      if (DecMBB->succ_size() != 1)
        DecMBB = splitCriticalEdge(*DecMBB, MBB);
      assert(DecMBB->succ_size() == 1);

      Register IncMBBVal = Val;
      Register DecMBBVal = Val;
      // Hoist and duplicate the definition of the value out of the final basic
      // block into the increment and decrement predecessors.
      if (ValDef->getParent() == &MBB) {
        IncMBBVal = MRI.createGenericVirtualRegister(MRI.getType(Val));
        MachineInstr *IncValDef = MBB.getParent()->CloneMachineInstr(ValDef);
        IncValDef->substituteRegister(Val, IncMBBVal, 0,
                                      *MRI.getTargetRegisterInfo());
        IncMBB->insert(IncMBB->getFirstTerminator(), IncValDef);
        DecMBB->insert(DecMBB->getFirstTerminator(),
                       ValDef->removeFromParent());
      }

      MachineIRBuilder MIB(*IncMBB, IncMBB->getFirstTerminator());
      Register Inc = MIB.buildAdd(Ty, IncMBBVal, OneReg).getReg(0);
      MIB.setInsertPt(*DecMBB, DecMBB->getFirstTerminator());
      Register Dec = MIB.buildAdd(Ty, DecMBBVal, NegOneReg).getReg(0);
      MIB.setInsertPt(MBB, MBB.begin());
      MIB.buildInstr(MOS::G_PHI)
          .addDef(Dst)
          .addUse(Inc)
          .addMBB(IncMBB)
          .addUse(Dec)
          .addMBB(DecMBB);
      --I;
      MI.eraseFromParent();

      if (FoldableStore) {
        MachineInstr *IncStore =
            MBB.getParent()->CloneMachineInstr(FoldableStore);
        IncStore->substituteRegister(Dst, Inc, 0, *MRI.getTargetRegisterInfo());
        IncMBB->insert(IncMBB->getFirstTerminator(), IncStore);
        FoldableStore->substituteRegister(Dst, Dec, 0,
                                          *MRI.getTargetRegisterInfo());
        DecMBB->insert(DecMBB->getFirstTerminator(),
                       FoldableStore->removeFromParent());
      }

      Changed = true;
    }
  }

  return Changed;
}

char MOSIncDecPhi::ID = 0;

INITIALIZE_PASS_BEGIN(MOSIncDecPhi, DEBUG_TYPE,
                      "Optimize MOS Increment/Decrement PHI pattern", false,
                      false)
INITIALIZE_PASS_DEPENDENCY(MachineDominatorTreeWrapperPass)
INITIALIZE_PASS_END(MOSIncDecPhi, DEBUG_TYPE,
                    "Optimize MOS Increment/Decrement PHI pattern", false,
                    false)

MachineFunctionPass *llvm::createMOSIncDecPhiPass() {
  return new MOSIncDecPhi();
}
