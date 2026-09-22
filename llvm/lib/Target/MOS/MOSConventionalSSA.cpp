//===----------------------------------------------------------------------===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
///
/// \file
/// Normalize the SSA into "conventional" form before allocation.
///
/// PHIs require their incoming values and result to meet in the same storage.
/// This pass isolates those ranges with parallel copies before predecessor
/// terminators and after the successor's PHIs. The fresh names allow the
/// allocator to give each PHI's inputs and result one assignment without
/// forcing that assignment on the original sources or subsequent users.
///
/// Copies for all successors share one outgoing PCOPY in each predecessor.
/// This also handles critical edges, including indirect branches, without
/// changing the CFG. MOSImagRegAlloc accounts for the interference of these
/// ranges across the whole function; MOSRegAlloc later eliminates the PHIs
/// and lowers the parallel copies.
///
/// This pass further normalizes tied uses and defs to allow us to treat a tied
/// def as a mere continuation of its use's live range. To do so, this pass
/// inserts copies and rewrites to ensure that each tied use is a kill and that
/// it satisfies the constraints of its tied def.
///
//===----------------------------------------------------------------------===//

#include "MOSConventionalSSA.h"
#include "MCTargetDesc/MOSMCTargetDesc.h"
#include "MOS.h"
#include "llvm/ADT/IndexedMap.h"
#include "llvm/ADT/SetVector.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/CodeGen/LiveVariables.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/TargetInstrInfo.h"
#include "llvm/CodeGen/TargetRegisterInfo.h"
#include "llvm/InitializePasses.h"

#define DEBUG_TYPE "mos-conventional-ssa"

using namespace llvm;

namespace {

class MOSConventionalSSA : public MachineFunctionPass {
public:
  static char ID;

  MOSConventionalSSA() : MachineFunctionPass(ID) {
    initializeMOSConventionalSSAPass(*PassRegistry::getPassRegistry());
  }

  bool runOnMachineFunction(MachineFunction &MF) override;

  MachineFunctionProperties getRequiredProperties() const override {
    return MachineFunctionProperties().setIsSSA();
  }

  void getAnalysisUsage(AnalysisUsage &AU) const override {
    AU.addRequired<LiveVariablesWrapperPass>();
    AU.addPreserved<LiveVariablesWrapperPass>();
    AU.setPreservesCFG();
    MachineFunctionPass::getAnalysisUsage(AU);
  }

private:
  struct Copy {
    MachineOperand Def;
    MachineOperand Use;
  };

  void isolatePHIs(MachineBasicBlock &MBB);
  void insertExitCopies(MachineBasicBlock &MBB, ArrayRef<Copy> Copies);
  bool normalizeTies(MachineInstr &MI);
  void insertParallelCopy(MachineBasicBlock &MBB,
                          MachineBasicBlock::iterator InsertPt,
                          ArrayRef<Copy> Copies, const DebugLoc &DL);

  MachineRegisterInfo *MRI = nullptr;
  const TargetInstrInfo *TII = nullptr;
  const TargetRegisterInfo *TRI = nullptr;
  LiveVariables *LV = nullptr;
  // One exit PCOPY per predecessor, shared by all its successors.
  IndexedMap<SmallVector<Copy, 0>, MBB2NumberFunctor> ExitCopies;
  SmallSetVector<Register, 16> LivenessDirty;
};

bool MOSConventionalSSA::runOnMachineFunction(MachineFunction &MF) {
  MRI = &MF.getRegInfo();
  TII = MF.getSubtarget().getInstrInfo();
  TRI = MF.getSubtarget().getRegisterInfo();
  LV = &getAnalysis<LiveVariablesWrapperPass>().getLV();
  LivenessDirty.clear();

  bool Changed = false;

  ExitCopies.clear();
  ExitCopies.resize(MF.getNumBlockIDs());

  for (MachineBasicBlock &MBB : MF) {
    assert(llvm::none_of(MBB,
                         [](const MachineInstr &MI) {
                           return MI.isTerminator() &&
                                  llvm::any_of(
                                      MI.all_defs(),
                                      [](const MachineOperand &Def) {
                                        return Def.getReg().isVirtual();
                                      });
                         }) &&
           "virtual definitions in terminators are not yet supported");
    if (MBB.phis().empty())
      continue;
    isolatePHIs(MBB);
    Changed = true;
  }

  for (MachineBasicBlock &MBB : MF) {
    ArrayRef<Copy> Copies = ExitCopies[&MBB];
    if (!Copies.empty())
      insertExitCopies(MBB, Copies);
  }
  for (Register Reg : LivenessDirty)
    LV->recomputeForSingleDefVirtReg(Reg);
  for (MachineBasicBlock &MBB : MF)
    for (MachineInstr &MI : MBB)
      Changed |= normalizeTies(MI);
  return Changed;
}

void MOSConventionalSSA::isolatePHIs(MachineBasicBlock &MBB) {
  SmallVector<Copy> EntryCopies;
  for (MachineInstr &PHI : MBB.phis()) {
    MachineOperand &Def = PHI.getOperand(0);
    if (Def.isDead())
      continue;
    Register Result = MRI->createVirtualRegister(TRI->getLargestLegalSuperClass(
        MRI->getRegClass(Def.getReg()), *MBB.getParent()));
    LivenessDirty.insert(Def.getReg());
    LivenessDirty.insert(Result);
    EntryCopies.push_back(
        {Def, MachineOperand::CreateReg(Result, /*isDef=*/false)});
    Def.setReg(Result);
  }
  if (!EntryCopies.empty())
    insertParallelCopy(MBB, MBB.getFirstNonPHI(), EntryCopies,
                       MBB.front().getDebugLoc());

  for (MachineInstr &PHI : MBB.phis()) {
    assert(PHI.getNumOperands() == 1 + 2 * MBB.pred_size() &&
           "expected one PHI input per predecessor");
    for (unsigned I = 1, E = PHI.getNumOperands(); I != E; I += 2) {
      MachineOperand &Use = PHI.getOperand(I);
      // An undef input does not read its register, so there is no value to
      // isolate. It imposes no incoming assignment requirement on this edge.
      if (Use.isUndef())
        continue;
      MachineBasicBlock *Pred = PHI.getOperand(I + 1).getMBB();
      // Derive the class from the PHI result, since the input may be a
      // subregister of a larger register.
      Register Input =
          MRI->createVirtualRegister(TRI->getLargestLegalSuperClass(
              MRI->getRegClass(PHI.getOperand(0).getReg()), *MBB.getParent()));
      LivenessDirty.insert(Input);
      LivenessDirty.insert(Use.getReg());
      ExitCopies[Pred].push_back(
          {MachineOperand::CreateReg(Input, /*isDef=*/true), Use});
      Use.setReg(Input);
      Use.setSubReg(0);
      Use.setIsKill(false);
    }
  }
}

void MOSConventionalSSA::insertExitCopies(MachineBasicBlock &MBB,
                                          ArrayRef<Copy> Copies) {
  assert(llvm::none_of(MBB,
                       [](const MachineInstr &MI) {
                         return MI.getOpcode() == TargetOpcode::INLINEASM_BR;
                       }) &&
         "asm goto is not yet supported");
  insertParallelCopy(MBB, MBB.getFirstTerminator(), Copies, DebugLoc());
}

bool MOSConventionalSSA::normalizeTies(MachineInstr &MI) {
  bool Changed = false;
  for (MachineOperand &Use : MI.all_uses()) {
    if (!Use.isTied())
      continue;
    MachineOperand &Def =
        MI.getOperand(MI.findTiedOperandIdx(Use.getOperandNo()));
    if (Def.getReg().isPhysical()) {
      assert(Use.getReg() == Def.getReg() &&
             "expected physical ties to name the same register");
      continue;
    }
    assert(!Def.getSubReg() && "expected a whole virtual definition in SSA");

    const TargetRegisterClass *RC = MRI->getRegClass(Def.getReg());
    const TargetRegisterClass *UseRC =
        MI.getRegClassConstraint(Use.getOperandNo(), TII, TRI);
    if (UseRC)
      RC = TRI->getCommonSubClass(RC, UseRC);
    if (!RC) {
      // The result's users may constrain it to registers that cannot satisfy
      // the tied input. Isolate the result so only the instruction's own
      // constraints apply to the tie.
      Register Original = Def.getReg();
      RC = TRI->getLargestLegalSuperClass(MRI->getRegClass(Original),
                                          *MI.getMF());
      if (const TargetRegisterClass *DefRC =
              MI.getRegClassConstraint(Def.getOperandNo(), TII, TRI))
        RC = TRI->getCommonSubClass(RC, DefRC);
      if (RC)
        RC = TRI->getCommonSubClass(RC, UseRC);
      assert(RC && "incompatible tied operand constraints");
      Register Result = MRI->createVirtualRegister(RC);
      Def.setReg(Result);
      BuildMI(*MI.getParent(), std::next(MI.getIterator()), MI.getDebugLoc(),
              TII->get(TargetOpcode::COPY), Original)
          .addReg(Result);
      LV->recomputeForSingleDefVirtReg(Original);
      LV->recomputeForSingleDefVirtReg(Result);
      Changed = true;
    }
    if (MRI->getRegClass(Def.getReg()) != RC) {
      MRI->setRegClass(Def.getReg(), RC);
      Changed = true;
    }

    // A dying input can hand its storage to one result. Other tied uses need
    // their own storage, and an early-clobber result cannot overwrite an input
    // that another operand still needs to read. Require identical classes to
    // avoid narrowing the combined live range and increasing pressure.
    // Otherwise, leave coalescing to allocation.
    if (Use.getReg().isVirtual() && !Use.getSubReg() && !Use.isUndef() &&
        MRI->getRegClass(Use.getReg()) == RC &&
        MI.killsRegister(Use.getReg(), TRI) &&
        llvm::none_of(MI.all_uses(), [&](const MachineOperand &Other) {
          return &Other != &Use && Other.isReg() && Other.readsReg() &&
                 Other.getReg() == Use.getReg() &&
                 (Other.isTied() || Def.isEarlyClobber());
        })) {
      Changed |= !Use.isKill();
      Use.setIsKill();
      continue;
    }

    // Isolate the input in the result's class, leaving allocation to coalesce
    // the copy if possible.
    Register Source = Use.getReg();
    bool IsUndef = Use.isUndef();
    Register Input = MRI->createVirtualRegister(RC);
    if (Use.isUndef())
      BuildMI(*MI.getParent(), MI, MI.getDebugLoc(),
              TII->get(TargetOpcode::IMPLICIT_DEF), Input);
    else
      BuildMI(*MI.getParent(), MI, MI.getDebugLoc(),
              TII->get(TargetOpcode::COPY), Input)
          .addReg(Use.getReg(), {}, Use.getSubReg());
    Use.setReg(Input);
    Use.setSubReg(0);
    Use.setIsUndef(false);
    if (Source.isVirtual() && !IsUndef)
      LV->recomputeForSingleDefVirtReg(Source);
    LV->recomputeForSingleDefVirtReg(Input);
    Changed = true;
  }
  return Changed;
}

void MOSConventionalSSA::insertParallelCopy(
    MachineBasicBlock &MBB, MachineBasicBlock::iterator InsertPt,
    ArrayRef<Copy> Copies, const DebugLoc &DL) {
  MachineInstrBuilder MIB = BuildMI(MBB, InsertPt, DL, TII->get(MOS::PCOPY));
  for (const Copy &C : Copies)
    MIB.add(C.Def);
  for (const Copy &C : Copies)
    MIB.add(C.Use);
}

} // namespace

char MOSConventionalSSA::ID = 0;
INITIALIZE_PASS_BEGIN(MOSConventionalSSA, DEBUG_TYPE, "MOS Conventional SSA",
                      false, false)
INITIALIZE_PASS_DEPENDENCY(LiveVariablesWrapperPass)
INITIALIZE_PASS_END(MOSConventionalSSA, DEBUG_TYPE, "MOS Conventional SSA",
                    false, false)

MachineFunctionPass *llvm::createMOSConventionalSSAPass() {
  return new MOSConventionalSSA;
}
