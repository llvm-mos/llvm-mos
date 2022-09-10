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

#include "MOS.h"
#include "MOSRegisterInfo.h"
#include "MOSSubtarget.h"

#include "llvm/ADT/DenseSet.h"
#include "llvm/ADT/PostOrderIterator.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/CodeGen/LivePhysRegs.h"
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

  bool runOnMachineFunction(MachineFunction &MF) override;
};

} // namespace

static Register findForwardedCopy(MachineInstr &MI,
                                  SmallVectorImpl<MachineInstr *> &NewSrcMIs) {
  assert(MI.isCopy());
  const TargetRegisterInfo &TRI = *MI.getMF()->getSubtarget().getRegisterInfo();

  Register Src = MI.getOperand(1).getReg();

  struct Entry {
    MachineBasicBlock &MBB;
    MachineBasicBlock::reverse_iterator I;
  };

  SmallVector<Entry> WorkList = {
      {*MI.getParent(), MachineBasicBlock::reverse_iterator(MI.getIterator())}};
  DenseSet<const MachineBasicBlock *> Seen;
  Register NewSrc = 0;
  while (!WorkList.empty()) {
    Entry E = WorkList.back();
    WorkList.pop_back();
    if (Seen.contains(&E.MBB))
      continue;

    // Don't count the start MBB as seen until it's been seen as a predecessor.
    if (E.I == E.MBB.rbegin())
      Seen.insert(&E.MBB);

    bool Found = false;
    for (MachineInstr &MI : make_range(E.I, E.MBB.rend())) {
      if (!MI.modifiesRegister(Src, &TRI))
        continue;
      if (!MI.isCopy())
        return 0;
      Register NewDst = MI.getOperand(0).getReg();
      if (NewDst != Src)
        return 0;
      Register NewSrcCand = MI.getOperand(1).getReg();
      if (NewSrc && NewSrc != NewSrcCand)
        return 0;

      NewSrc = NewSrcCand;
      Found = true;
      NewSrcMIs.push_back(&MI);
      break;
    }
    if (!Found) {
      // The register must have been live-in.
      if (E.MBB.isEntryBlock())
        return 0;
      for (MachineBasicBlock *MBB : E.MBB.predecessors())
        WorkList.push_back({*MBB, MBB->rbegin()});
    }
  }
  return NewSrc;
}

static bool isClobbered(MachineInstr &MI, Register NewSrc,
                        const SmallVectorImpl<MachineInstr *> &NewSrcMIs) {
  const TargetRegisterInfo &TRI = *MI.getMF()->getSubtarget().getRegisterInfo();

  struct Entry {
    MachineBasicBlock &MBB;
    MachineBasicBlock::reverse_iterator I;
  };

  SmallVector<Entry> WorkList = {
      {*MI.getParent(), MachineBasicBlock::reverse_iterator(MI.getIterator())}};
  DenseSet<const MachineBasicBlock *> Seen;
  while (!WorkList.empty()) {
    Entry E = WorkList.back();
    WorkList.pop_back();
    if (Seen.contains(&E.MBB))
      continue;

    // Don't count the start MBB as seen until it's been seen as a predecessor.
    if (E.I == E.MBB.rbegin())
      Seen.insert(&E.MBB);

    bool Found = false;
    for (MachineInstr &MI : make_range(E.I, E.MBB.rend())) {
      if (is_contained(NewSrcMIs, &MI)) {
        Found = true;
        break;
      }
      if (MI.modifiesRegister(NewSrc, &TRI))
        return true;
    }
    if (!Found)
      for (MachineBasicBlock *MBB : E.MBB.predecessors())
        WorkList.push_back({*MBB, MBB->rbegin()});
  }
  return false;
}

static Register findBetterCopySource(MachineInstr &MI,
                                     MachineInstr *&NewSrcMI) {
  assert(MI.isCopy());
  const MOSSubtarget &STI = MI.getMF()->getSubtarget<MOSSubtarget>();
  const MOSRegisterInfo &TRI = *STI.getRegisterInfo();

  Register Dst = MI.getOperand(0).getReg();
  Register Src = MI.getOperand(1).getReg();

  for (MachineInstr &MI :
       make_range(MachineBasicBlock::reverse_iterator(MI.getIterator()),
                  MI.getParent()->rend())) {
    if (MI.modifiesRegister(Src, &TRI))
      return 0;
    if (!MI.isCopy())
      continue;
    if (MI.getOperand(1).getReg() != Src)
      continue;
    Register NewSrc = MI.getOperand(0).getReg();
    if (TRI.copyCost(Dst, NewSrc, STI) < TRI.copyCost(Dst, Src, STI)) {
      NewSrcMI = &MI;
      return NewSrc;
    }
  }
  return 0;
}

bool MOSCopyOpt::runOnMachineFunction(MachineFunction &MF) {
  const MOSSubtarget &STI = MF.getSubtarget<MOSSubtarget>();
  const MOSRegisterInfo &TRI = *STI.getRegisterInfo();

  LLVM_DEBUG(dbgs() << MF.getName() << "\n");

  for (MachineBasicBlock &MBB : MF) {
    for (MachineInstr &MI : make_early_inc_range(MBB)) {
      if (!MI.isCopy())
        continue;

      Register Dst = MI.getOperand(0).getReg();
      Register Src = MI.getOperand(1).getReg();
      SmallVector<MachineInstr *> NewSrcMIs;
      Register NewSrc = findForwardedCopy(MI, NewSrcMIs);
      if (!NewSrc)
        continue;

      LLVM_DEBUG(dbgs() << MI);
      LLVM_DEBUG(dbgs() << "Found candidate: " << printReg(NewSrc, &TRI)
                        << '\n');

      if (TRI.copyCost(Dst, NewSrc, STI) > TRI.copyCost(Dst, Src, STI)) {
        LLVM_DEBUG(dbgs() << "New copy is more expensive.\n");
        continue;
      }

      if (isClobbered(MI, NewSrc, NewSrcMIs)) {
        LLVM_DEBUG(dbgs() << "Clobbered.\n");
        continue;
      }

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
    }
  }

  for (MachineBasicBlock *MBB : post_order(&MF)) {
    LivePhysRegs LPR(TRI);

    recomputeLivenessFlags(*MBB);
    for (MachineInstr &MI : make_early_inc_range(*MBB)) {
      if (MI.isCopy() && MI.getOperand(0).isDead()) {
        LLVM_DEBUG(dbgs() << "Erasing dead copy: " << MI);
        MI.eraseFromParent();
      }
    }

    for (MachineInstr &MI : make_early_inc_range(*MBB)) {
      if (!MI.isCopy())
        continue;

      Register Dst = MI.getOperand(0).getReg();

      MachineInstr *NewSrcMI;
      Register NewSrc = findBetterCopySource(MI, NewSrcMI);
      if (!NewSrc)
        continue;

      LLVM_DEBUG(dbgs() << MI);
      LLVM_DEBUG(dbgs() << "Found candidate: " << printReg(NewSrc, &TRI)
                        << '\n');

      SmallVector<MachineInstr *> NewSrcMIs = {NewSrcMI};
      if (isClobbered(MI, NewSrc, NewSrcMIs)) {
        LLVM_DEBUG(dbgs() << "Clobbered.\n");
        continue;
      }

      LLVM_DEBUG(dbgs() << "Rewriting copy: " << MI);
      NewSrcMI->clearRegisterDeads(NewSrc);
      if (Dst == NewSrc) {
        LLVM_DEBUG(dbgs() << "Erased.\n");
        MI.eraseFromParent();
      } else {
        MI.getOperand(1).setReg(NewSrc);
        MI.getOperand(1).setIsKill(false);
        LLVM_DEBUG(dbgs() << "Rewrote to: " << MI);
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
