//===-- MOSCopyOpt.cpp - MOS Copy Optimization ---------------------------===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file defines the MOS pass to fully optimize COPY operations before
// lowering.
//
//===----------------------------------------------------------------------===//

#include "MOSCopyOpt.h"

#include "MCTargetDesc/MOSMCTargetDesc.h"
#include "MOS.h"
#include "MOSRegisterInfo.h"
#include "MOSSubtarget.h"

#include "llvm/ADT/DenseSet.h"
#include "llvm/ADT/PostOrderIterator.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/ADT/SmallPtrSet.h"
#include "llvm/CodeGen/LivePhysRegs.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineMemOperand.h"
#include "llvm/CodeGen/ReachingDefAnalysis.h"
#include "llvm/CodeGen/TargetRegisterInfo.h"
#include "llvm/CodeGen/TargetSubtargetInfo.h"

#define DEBUG_TYPE "mos-copy-opt"

using namespace llvm;

namespace {

class MOSCopyOpt : public MachineFunctionPass {
public:
  static char ID;

  MOSCopyOpt() : MachineFunctionPass(ID) {
    llvm::initializeMOSCopyOptPass(*PassRegistry::getPassRegistry());
  }

  void getAnalysisUsage(AnalysisUsage &AU) const override {
    AU.setPreservesCFG();
    AU.addRequired<ReachingDefAnalysis>();
    MachineFunctionPass::getAnalysisUsage(AU);
  }

  bool runOnMachineFunction(MachineFunction &MF) override;
};

} // namespace

static bool
isRegLiveInFunction(MachineBasicBlock &MBB, Register PhysReg,
                    SmallPtrSetImpl<MachineBasicBlock *> &VisitedMBBs,
                    const ReachingDefAnalysis &RDA) {
  if (llvm::is_contained(VisitedMBBs, &MBB))
    return false;
  VisitedMBBs.insert(&MBB);
  if (!MBB.isLiveIn(PhysReg))
    return false;
  if (RDA.getLocalLiveOutMIDef(&MBB, PhysReg))
    return false;
  if (MBB.isEntryBlock())
    return true;
  return llvm::any_of(MBB.predecessors(), [&](MachineBasicBlock *MBB) {
    return isRegLiveInFunction(*MBB, PhysReg, VisitedMBBs, RDA);
  });
}

// Returns whether the value of the physical register used in a given MI might
// be live in to the containing function.
static bool isRegLiveInFunction(MachineInstr &MI, Register PhysReg,
                                const ReachingDefAnalysis &RDA) {
  if (RDA.hasLocalDefBefore(&MI, PhysReg))
    return false;
  SmallPtrSet<MachineBasicBlock *, 4> VisitedMBBs;
  return llvm::any_of(
      MI.getParent()->predecessors(), [&](MachineBasicBlock *MBB) {
        return isRegLiveInFunction(*MBB, PhysReg, VisitedMBBs, RDA);
      });
}

static Register findForwardedCopy(MachineInstr &MI,
                                  SmallPtrSetImpl<MachineInstr *> &NewSrcMIs,
                                  const ReachingDefAnalysis &RDA) {
  assert(MI.isCopy());
  Register Src = MI.getOperand(1).getReg();

  RDA.getGlobalReachingDefs(&MI, Src, NewSrcMIs);

  Register NewSrc = 0;
  for (MachineInstr *MI : NewSrcMIs) {
    if (!MI->isCopy())
      return 0;
    if (MI->getOperand(0).getReg() != Src)
      return 0;
    Register NewSrcCand = MI->getOperand(1).getReg();
    if (!NewSrc)
      NewSrc = NewSrcCand;
    if (NewSrcCand != NewSrc)
      return 0;
  }

  if (isRegLiveInFunction(MI, Src, RDA))
    return 0;

  return NewSrc;
}

static bool findLdImm(MachineInstr &MI, SmallPtrSetImpl<MachineInstr *> &LdImms,
                      const ReachingDefAnalysis &RDA) {
  const TargetRegisterInfo &TRI = *MI.getMF()->getSubtarget().getRegisterInfo();
  const TargetInstrInfo &TII = *MI.getMF()->getSubtarget().getInstrInfo();

  assert(MI.isCopy());
  Register Dst = MI.getOperand(0).getReg();
  Register Src = MI.getOperand(1).getReg();

  RDA.getGlobalReachingDefs(&MI, Src, LdImms);
  if (LdImms.empty())
    return false;

  for (MachineInstr *MI : LdImms) {
    if (!MI->isMoveImmediate())
      return false;
    if (MI->getOperand(0).getReg() != Src)
      return false;
    const TargetRegisterClass *RC =
        TII.getRegClass(MI->getDesc(), 0, &TRI, *MI->getMF());
    if (!RC->contains(Dst))
      return false;
    if (!(*LdImms.begin())->isIdenticalTo(*MI))
      return false;
  }

  if (isRegLiveInFunction(MI, Src, RDA))
    return false;

  return true;
}

static bool isClobberedBetween(MachineBasicBlock &MBB, Register PhysReg,
                               const SmallPtrSetImpl<MachineInstr *> &EndMIs,
                               SmallPtrSetImpl<MachineBasicBlock *> &Visited) {
  if (Visited.contains(&MBB))
    return false;
  Visited.insert(&MBB);
  const TargetRegisterInfo &TRI =
      *MBB.getParent()->getSubtarget().getRegisterInfo();
  for (MachineInstr &MI : llvm::reverse(MBB)) {
    if (is_contained(EndMIs, &MI))
      return false;
    if (MI.modifiesRegister(PhysReg, &TRI))
      return true;
  }
  return llvm::any_of(MBB.predecessors(), [&](MachineBasicBlock *MBB) {
    return isClobberedBetween(*MBB, PhysReg, EndMIs, Visited);
  });
}

static bool isClobberedBetween(MachineInstr &MI, Register PhysReg,
                               const SmallPtrSetImpl<MachineInstr *> &EndMIs) {
  const TargetRegisterInfo &TRI = *MI.getMF()->getSubtarget().getRegisterInfo();
  for (MachineInstr &MI :
       llvm::make_range(MachineBasicBlock::reverse_iterator(MI.getIterator()),
                        MI.getParent()->rend())) {
    if (is_contained(EndMIs, &MI))
      return false;
    if (MI.modifiesRegister(PhysReg, &TRI))
      return true;
  }
  SmallPtrSet<MachineBasicBlock *, 4> Visited;
  return llvm::any_of(
      MI.getParent()->predecessors(), [&](MachineBasicBlock *MBB) {
        return isClobberedBetween(*MBB, PhysReg, EndMIs, Visited);
      });
}

bool MOSCopyOpt::runOnMachineFunction(MachineFunction &MF) {
  const MOSSubtarget &STI = MF.getSubtarget<MOSSubtarget>();
  const MOSRegisterInfo &TRI = *STI.getRegisterInfo();
  const TargetInstrInfo &TII = *STI.getInstrInfo();
  const ReachingDefAnalysis &RDA = getAnalysis<ReachingDefAnalysis>();

  LLVM_DEBUG(dbgs() << MF.getName() << "\n");

  // Capture all mutations to perform before performing any of them to avoid
  // recomputing ReachingDefAnalysis.
  SmallVector<std::function<void()>, 16> Effects;

  for (MachineBasicBlock &MBB : MF) {
    for (MachineInstr &MI : make_early_inc_range(MBB)) {
      if (!MI.isCopy())
        continue;

      Register Dst = MI.getOperand(0).getReg();
      Register Src = MI.getOperand(1).getReg();

      SmallPtrSet<MachineInstr *, 4> NewSrcMIs;
      Register NewSrc = findForwardedCopy(MI, NewSrcMIs, RDA);
      if (!NewSrc)
        continue;

      LLVM_DEBUG(dbgs() << MI);
      LLVM_DEBUG(dbgs() << "Found candidate: " << printReg(NewSrc, &TRI)
                        << '\n');

      if (TRI.copyCost(Dst, NewSrc, STI) > TRI.copyCost(Dst, Src, STI)) {
        LLVM_DEBUG(dbgs() << "New copy is more expensive.\n");
        continue;
      }

      if (isClobberedBetween(MI, NewSrc, NewSrcMIs)) {
        LLVM_DEBUG(dbgs() << "Clobbered.\n");
        continue;
      }

      Effects.push_back(
          [&MI, NewSrc, NewSrcMIs = std::move(NewSrcMIs), &TRI, Dst]() {
            LLVM_DEBUG(dbgs() << "Rewriting copy: " << MI);
            for (MachineInstr *NewSrcMI : NewSrcMIs)
              NewSrcMI->clearRegisterKills(NewSrc, &TRI);
            if (Dst == NewSrc) {
              LLVM_DEBUG(dbgs() << "Erased.\n");
              MI.eraseFromParent();
            } else {
              MI.getOperand(1).setReg(NewSrc);
              MI.getOperand(1).setIsKill(false);
              LLVM_DEBUG(dbgs() << "Rewrote to: " << MI);
            }
          });
    }

    for (MachineInstr &MI : make_early_inc_range(MBB)) {
      if (!MI.isCopy())
        continue;

      Register Dst = MI.getOperand(0).getReg();
      Register Src = MI.getOperand(1).getReg();

      if (!MOS::Imag16RegClass.contains(Dst) && Dst != MOS::C &&
          Dst != MOS::V && TRI.copyCost(Dst, Src, STI) <= 4)
        continue;

      SmallPtrSet<MachineInstr *, 4> LdImms;
      if (!findLdImm(MI, LdImms, RDA))
        continue;

      LLVM_DEBUG(dbgs() << MI);
      MachineInstr &Cand = **LdImms.begin();
      LLVM_DEBUG(dbgs() << "Found remat candidate: " << Cand);

      if (isClobberedBetween(MI, Cand.getOperand(0).getReg(), LdImms)) {
        LLVM_DEBUG(dbgs() << "Clobbered.\n");
        continue;
      }

      Effects.push_back([LdImms = std::move(LdImms), Src, &TRI, &Cand, &TII,
                         &MI, &MBB, Dst]() {
        for (MachineInstr *LdImm : LdImms)
          LdImm->clearRegisterKills(Src, &TRI);
        Cand.clearRegisterKills(Src, &TRI);
        TII.reMaterialize(MBB, MI, Dst, 0, Cand, TRI);
        MI.eraseFromParent();
      });
    }
  }

  for (const auto &E : Effects)
    E();

  for (MachineBasicBlock *MBB : post_order(&MF)) {
    LivePhysRegs LPR(TRI);

    recomputeLivenessFlags(*MBB);
    for (MachineInstr &MI : make_early_inc_range(*MBB)) {
      if (MI.isCopy() && MI.getOperand(0).isDead()) {
        LLVM_DEBUG(dbgs() << "Erasing dead copy: " << MI);
        MI.eraseFromParent();
      }
    }

    if (!MBB->isEntryBlock()) {
      recomputeLivenessFlags(*MBB);
      MBB->clearLiveIns();
      computeAndAddLiveIns(LPR, *MBB);
    }
  }
  return true;
}

char MOSCopyOpt::ID = 0;

INITIALIZE_PASS(MOSCopyOpt, DEBUG_TYPE, "Optimize copies for MOS", false, false)

MachineFunctionPass *llvm::createMOSCopyOptPass() { return new MOSCopyOpt(); }
