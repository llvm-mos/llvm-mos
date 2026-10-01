//===----------------------------------------------------------------------===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
///
/// \file
/// Spill values to make imaginary register pressure feasible within each block,
/// including room for spill and reload temporaries.
///
/// The spill functions record transfers without changing MIR. Emission then
/// inserts spills and reloads and renames uses to the latest reload. Currently
/// only one block with ordinary virtual definitions and uses is supported;
/// subregister operands, ties, PHIs, parallel copies, and physical imaginary
/// register constraints are not yet handled.
///
//===----------------------------------------------------------------------===//

#include "MOSImagRegSpill.h"
#include "MCTargetDesc/MOSMCTargetDesc.h"
#include "MOS.h"
#include "MOSFrameLowering.h"
#include "MOSInterferenceGraph.h"
#include "MOSRegisterInfo.h"
#include "MOSSubtarget.h"
#include "llvm/ADT/DenseMap.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/CodeGen/GlobalISel/MachineIRBuilder.h"
#include "llvm/CodeGen/LiveVariables.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/RegisterClassInfo.h"
#include "llvm/CodeGen/TargetInstrInfo.h"
#include "llvm/CodeGen/TargetRegisterInfo.h"
#include "llvm/InitializePasses.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/ErrorHandling.h"
#include <optional>

#define DEBUG_TYPE "mos-imag-reg-spill"

using namespace llvm;

namespace {

// Transfers to emit before an original instruction. Stores precede reloads so
// that evicted values vacate storage before the reloaded values need it. The
// registers name original SSA values; reload names are created during emission.
struct SpillTransfers {
  SmallVector<Register, 2> Stores;
  SmallVector<Register, 2> Reloads;
};

class MOSImagRegSpill : public MachineFunctionPass {
public:
  static char ID;

  MOSImagRegSpill() : MachineFunctionPass(ID) {
    initializeMOSImagRegSpillPass(*PassRegistry::getPassRegistry());
  }

  bool runOnMachineFunction(MachineFunction &MF) override;

  MachineFunctionProperties getRequiredProperties() const override {
    return MachineFunctionProperties().setIsSSA();
  }

  void getAnalysisUsage(AnalysisUsage &AU) const override {
    AU.addRequired<LiveVariablesWrapperPass>();
    AU.addRequired<MachineRegisterClassInfoWrapperPass>();
    AU.addPreserved<LiveVariablesWrapperPass>();
    AU.setPreservesCFG();
    MachineFunctionPass::getAnalysisUsage(AU);
  }

private:
  void checkSupportedMIR(const MachineFunction &MF) const;
  void spillForMBB(MachineBasicBlock &MBB);

  // Plan spills and reloads before the first instruction of a nonempty,
  // contiguous range, making its register demands feasible without transfers
  // between its instructions. Only resident values unused and undefined by the
  // range may be evicted. The builder must be capturing Graph at the range's
  // beginning; on return it is past the range, retaining only its live-outs.
  // Currently, no virtual register defined within the range may be read within
  // it: all inputs are made resident before any instruction is processed.
  void spillBeforeMIs(iterator_range<MachineBasicBlock::iterator> Instructions);

  // Make all inputs resident before the range, accounting for reload scratch.
  void
  reloadForInputs(iterator_range<MachineBasicBlock::iterator> Instructions);
  void reloadForInput(Register Reg,
                      iterator_range<MachineBasicBlock::iterator> Instructions);

  // Spill live-through values until the current residents leave enough room
  // for transfer temporaries. All graph vertices must already be retained.
  // Stores are inserted before Instructions.
  void ensureScratch(iterator_range<MachineBasicBlock::iterator> Instructions);

  // Resident live-throughs eligible for eviction before Instructions. Query
  // immediately before or after stepping the builder through the range.
  SmallVector<Register> spillCandidates(
      iterator_range<MachineBasicBlock::iterator> Instructions) const;

  // Spill one of RemainingRegs that is live through the entire range.
  void
  spillLiveThrough(iterator_range<MachineBasicBlock::iterator> Instructions,
                   ArrayRef<Register> RemainingRegs);
  void evict(Register Reg, MachineInstr &Before);
  void insertSpillsAndReloads(MachineBasicBlock &MBB);
  int getOrCreateSpillSlot(Register Reg);

  // Mark MIR changed and trace the inserted spill instructions.
  void recordSpillInstructions(
      iterator_range<MachineBasicBlock::iterator> Instructions);

  // Stack transfers for whole virtual registers, preserving SSA definitions.
  void storeRegToStackSlot(MachineBasicBlock &MBB,
                           MachineBasicBlock::iterator InsertPt, Register Reg,
                           int FI);
  void loadRegFromStackSlot(MachineBasicBlock &MBB,
                            MachineBasicBlock::iterator InsertPt, Register Reg,
                            int FI);

  MachineRegisterInfo *MRI = nullptr;
  MachineFrameInfo *MFI = nullptr;
  const TargetInstrInfo *TII = nullptr;
  const MOSRegisterInfo *TRI = nullptr;
  const MOSFrameLowering *TFL = nullptr;
  LiveVariables *LV = nullptr;
  const RegisterClassInfo *RCI = nullptr;
  MOSInterferenceGraphBuilder Builder;
  // Active graph while spilling a block.
  std::optional<MOSInterferenceGraph> Graph;
  // Original MIR remains unchanged during planning; keys survive until
  // emission.
  DenseMap<MachineInstr *, SpillTransfers> PlannedTransfers;
  // Each original SSA value has its own slot, shared by all of its reloads.
  DenseMap<Register, int> SpillSlots;
  bool Changed = false;
};

bool MOSImagRegSpill::runOnMachineFunction(MachineFunction &MF) {
  MRI = &MF.getRegInfo();
  MFI = &MF.getFrameInfo();
  TII = MF.getSubtarget().getInstrInfo();
  TRI = MF.getSubtarget<MOSSubtarget>().getRegisterInfo();
  TFL = MF.getSubtarget<MOSSubtarget>().getFrameLowering();
  LV = &getAnalysis<LiveVariablesWrapperPass>().getLV();
  RCI = &getAnalysis<MachineRegisterClassInfoWrapperPass>().getRCI();
  Builder = MOSInterferenceGraphBuilder();
  PlannedTransfers.clear();
  SpillSlots.clear();
  Changed = false;

  checkSupportedMIR(MF);
  spillForMBB(MF.front());
  insertSpillsAndReloads(MF.front());
  // Spilling can change most ranges in the block. Recompute them together:
  // per-register updates repeatedly scan the block to find last uses.
  if (Changed)
    *LV = LiveVariables(MF);
  return Changed;
}

void MOSImagRegSpill::checkSupportedMIR(const MachineFunction &MF) const {
  if (MF.size() != 1 || !MF.front().succ_empty())
    report_fatal_error("MOSImagRegSpill currently requires a single block",
                       false);
  for (const MachineBasicBlock &MBB : MF) {
    for (const auto &LiveIn : MBB.liveins())
      if (MOS::Imag16RegClass.contains(LiveIn.PhysReg) ||
          MOS::Imag8RegClass.contains(LiveIn.PhysReg) ||
          MOS::ImagLSBRegClass.contains(LiveIn.PhysReg))
        report_fatal_error("MOSImagRegSpill does not yet support imaginary "
                           "physical live-ins",
                           false);
    for (const MachineInstr &MI : MBB) {
      if (MI.isDebugOrPseudoInstr())
        continue;
      if (MI.isPHI() || MI.getOpcode() == MOS::PCOPY || MI.isInlineAsm() ||
          MI.isBundled())
        report_fatal_error(
            "MOSImagRegSpill does not yet support this instruction", false);
      for (const MachineOperand &MO : MI.operands()) {
        if (MO.isRegMask())
          report_fatal_error("MOSImagRegSpill does not yet support regmasks",
                             false);
        if (!MO.isReg() || !MO.getReg())
          continue;
        Register Reg = MO.getReg();
        if (Reg.isPhysical()) {
          // Undef hardware operands do not read a physical value and impose no
          // storage requirement.
          if ((!MO.isImplicit() && !MO.isUndef()) ||
              MOS::Imag16RegClass.contains(Reg) ||
              MOS::Imag8RegClass.contains(Reg) ||
              MOS::ImagLSBRegClass.contains(Reg))
            report_fatal_error("MOSImagRegSpill does not yet support physical "
                               "register constraints",
                               false);
          continue;
        }
        if (MO.getSubReg() || MO.isTied() || MO.isEarlyClobber() ||
            MO.isUndef())
          report_fatal_error("MOSImagRegSpill requires whole-register, untied "
                             "virtual operands",
                             false);
        TRI->getImagRegClass(Reg, *MRI);
      }
    }
  }
}

void MOSImagRegSpill::spillForMBB(MachineBasicBlock &MBB) {
  Graph.emplace(*MBB.getParent(), *RCI);
  Builder.beginCapture(*Graph);
  for (auto I = MBB.begin(), E = MBB.end(); I != E;) {
    if (I->isDebugOrPseudoInstr()) {
      ++I;
      continue;
    }
    // Terminators must remain contiguous, so consider them together.
    auto End = I->isTerminator() ? E : std::next(I);
    spillBeforeMIs(make_range(I, End));
    I = End;
  }
  Builder.endCapture();
  Graph.reset();
}

void MOSImagRegSpill::spillBeforeMIs(
    iterator_range<MachineBasicBlock::iterator> Instructions) {
  reloadForInputs(Instructions);

  for (MachineInstr &MI : Instructions) {
    if (!MI.isDebugOrPseudoInstr())
      Builder.stepForward(MI);
  }
  SmallVector<Register> RemainingRegs;
  while (!Graph->canSimplify(&RemainingRegs))
    spillLiveThrough(Instructions, RemainingRegs);

  // The next range must have scratch available to start its transfers. Making
  // room may require spilling live-ins before this range; waiting until the
  // next range could be too late. Retain the live-outs so scratch must fit for
  // any assignment of the new definitions. Further evictions only remove
  // interference, preserving the successful instruction simplification above.
  Builder.trimToLive();
  Graph->retainAll();
  ensureScratch(Instructions);
}

void MOSImagRegSpill::reloadForInputs(
    iterator_range<MachineBasicBlock::iterator> Instructions) {
  // Make every input resident at the insertion point. Inputs of later
  // instructions must coexist with those consumed by earlier instructions,
  // since no reload can be inserted between them.
  //
  // All spill stores will precede this reload sequence, using the scratch
  // available on entry. A spill discovered while planning a later reload
  // therefore also frees storage for earlier reloads; their checks remain
  // valid.
  for (MachineInstr &MI : Instructions) {
    if (MI.isDebugOrPseudoInstr())
      continue;
    for (const MachineOperand &Use : MI.all_uses()) {
      Register Reg = Use.getReg();
      if (!Reg.isVirtual() || Use.isDebug() || Use.isUndef() ||
          Builder.spillState().ResidentValues.contains(Reg))
        continue;
      reloadForInput(Reg, Instructions);
    }
  }
}

void MOSImagRegSpill::reloadForInput(
    Register Reg, iterator_range<MachineBasicBlock::iterator> Instructions) {
  // Provisionally account for the destination as a new resident definition.
  // The reload is not yet known to be executable: it also needs scratch while
  // producing this result. Any required stores will precede the reload, using
  // the scratch guaranteed on entry.
  Builder.reload(Reg);
  SmallVector<Register> RemainingRegs;
  while (!Graph->canSimplify(&RemainingRegs))
    spillLiveThrough(Instructions, RemainingRegs);

  // Retain the destination so scratch must fit alongside any assignment of it.
  // This provides scratch for executing this reload and any following transfers.
  // Retention also prevents later reload checks from rearranging this result.
  Graph->retainAll();
  ensureScratch(Instructions);
  PlannedTransfers[&*Instructions.begin()].Reloads.push_back(Reg);
}

void MOSImagRegSpill::ensureScratch(
    iterator_range<MachineBasicBlock::iterator> Instructions) {
  // Use a conservative allowance of scratch to account for spills or reloads
  // here. The spilled/reloaded value is accounted for separately.
  bool StaticStack = TFL->usesStaticStack(*Instructions.begin()->getMF());
  SmallVector<MOSInterferenceGraph::VertexID, 3> Scratch;
  if (StaticStack) {
    // Byte temporaries used to assemble 16-bit values.
    Scratch.push_back(Builder.addTemporaryVertex(MOS::Imag8RegClass));
    Scratch.push_back(Builder.addTemporaryVertex(MOS::Imag8RegClass));
  }
  // Used as a pointer register or a temporary to cover over an internal issue
  // in stack emission (this can probably be tightened).
  Scratch.push_back(Builder.addTemporaryVertex(MOS::Imag16RegClass));
  SmallVector<Register> RemainingRegs;
  while (!Graph->canSimplify(&RemainingRegs))
    spillLiveThrough(Instructions, RemainingRegs);
  for (auto ID : Scratch) {
    Builder.killTemporaryVertex(ID);
    Graph->removeVertex(ID);
  }
}

SmallVector<Register> MOSImagRegSpill::spillCandidates(
    iterator_range<MachineBasicBlock::iterator> Instructions) const {
  SmallVector<Register> Regs;
  for (Register Reg : Builder.spillState().ResidentValues) {
    // Transfers precede the entire range, so only values unused and undefined
    // throughout it can be evicted here.
    if (llvm::any_of(Instructions, [&](const MachineInstr &MI) {
          return !MI.isDebugOrPseudoInstr() &&
                 (MI.readsRegister(Reg, nullptr) ||
                  MI.definesRegister(Reg, nullptr));
        }))
      continue;
    Regs.push_back(Reg);
  }
  return Regs;
}

void MOSImagRegSpill::spillLiveThrough(
    iterator_range<MachineBasicBlock::iterator> Instructions,
    ArrayRef<Register> RemainingRegs) {
  SmallVector<Register> Candidates = spillCandidates(Instructions);
  auto Victim = llvm::find_if(RemainingRegs, [&](Register Reg) {
    return llvm::is_contained(Candidates, Reg);
  });
  if (Victim == RemainingRegs.end())
    report_fatal_error("MOS instruction pressure cannot be reduced by "
                       "spilling live-through registers",
                       false);
  Register Reg = *Victim;
  evict(Reg, *Instructions.begin());
  Graph->remove(Reg);
}

void MOSImagRegSpill::evict(Register Reg, MachineInstr &Before) {
  if (!Builder.spillState().SavedResidentValues.contains(Reg))
    PlannedTransfers[&Before].Stores.push_back(Reg);

  // If the builder has already stepped past Before, the value remains
  // evicted afterwards too.
  Builder.evict(Reg);
}

void MOSImagRegSpill::insertSpillsAndReloads(MachineBasicBlock &MBB) {
  DenseMap<Register, Register> CurrentNames;
  for (MachineInstr &MI : make_early_inc_range(MBB)) {
    auto It = PlannedTransfers.find(&MI);
    if (It != PlannedTransfers.end()) {
      MachineInstrSpan Span(MI.getIterator(), &MBB);
      for (Register Reg : It->second.Stores)
        storeRegToStackSlot(MBB, MI, CurrentNames.lookup_or(Reg, Reg),
                            getOrCreateSpillSlot(Reg));
      for (Register Reg : It->second.Reloads) {
        Register Reload = MRI->cloneVirtualRegister(Reg);
        loadRegFromStackSlot(MBB, MI, Reload, getOrCreateSpillSlot(Reg));
        CurrentNames[Reg] = Reload;
        MRI->markUsesInDebugValueAsUndef(Reg);
      }
      recordSpillInstructions(
          make_range(Span.begin(), MachineBasicBlock::iterator(MI)));
    }

    // Each reload replaces the original name for all subsequent uses. New
    // stack instructions already use the current name and are skipped here.
    if (!MI.isDebugInstr())
      for (MachineOperand &Use : MI.all_uses())
        Use.setReg(CurrentNames.lookup_or(Use.getReg(), Use.getReg()));
  }
}

int MOSImagRegSpill::getOrCreateSpillSlot(Register Reg) {
  auto It = SpillSlots.find(Reg);
  if (It != SpillSlots.end())
    return It->second;
  const TargetRegisterClass *RC = MRI->getRegClass(Reg);
  int FI = MFI->CreateSpillStackObject(TRI->getSpillSize(*RC),
                                       TRI->getSpillAlign(*RC),
                                       TRI->getSpillStackID(*RC));
  SpillSlots[Reg] = FI;
  return FI;
}

void MOSImagRegSpill::recordSpillInstructions(
    iterator_range<MachineBasicBlock::iterator> Instructions) {
  Changed |= !Instructions.empty();
  LLVM_DEBUG(for (MachineInstr &MI : Instructions) dbgs()
                 << "Inserted spill instruction: " << MI;);
}

void MOSImagRegSpill::storeRegToStackSlot(MachineBasicBlock &MBB,
                                          MachineBasicBlock::iterator InsertPt,
                                          Register Reg, int FI) {
  const TargetRegisterClass *RC = MRI->getRegClass(Reg);
  if (TFL->usesStaticStack(*MBB.getParent()) &&
      RC->hasSuperClassEq(&MOS::Anyi1RegClass)) {
    // The target hook widens flags with a partial definition. Assemble a
    // whole byte instead; only its low bit matters when the value is reloaded.
    MachineIRBuilder Builder(MBB, InsertPt);
    Register Byte = MRI->createVirtualRegister(&MOS::GPRRegClass);
    Builder.buildInstr(MOS::REG_SEQUENCE)
        .addDef(Byte)
        .addUse(Reg)
        .addImm(MOS::sublsb);
    Reg = Byte;
    RC = &MOS::GPRRegClass;
  }
  TII->storeRegToStackSlot(MBB, InsertPt, Reg, false, FI, RC, Reg);
}

void MOSImagRegSpill::loadRegFromStackSlot(MachineBasicBlock &MBB,
                                           MachineBasicBlock::iterator InsertPt,
                                           Register Reg, int FI) {
  MachineFunction &MF = *MBB.getParent();
  const TargetRegisterClass *RC = MRI->getRegClass(Reg);
  if (!TFL->usesStaticStack(MF) || !RC->hasSuperClassEq(&MOS::Imag16RegClass)) {
    TII->loadRegFromStackSlot(MBB, InsertPt, Reg, FI, RC, Reg);
    return;
  }

  // The target hook defines the two halves separately. Load independent
  // bytes so the pair has exactly one definition.
  MachineFrameInfo &MFI = MF.getFrameInfo();
  MachineMemOperand *MMO = MF.getMachineMemOperand(
      MachinePointerInfo::getFixedStack(MF, FI), MachineMemOperand::MOLoad,
      MFI.getObjectSize(FI), MFI.getObjectAlign(FI));
  MachineIRBuilder Builder(MBB, InsertPt);
  Register Lo = MRI->createVirtualRegister(&MOS::GPRRegClass);
  Register Hi = MRI->createVirtualRegister(&MOS::GPRRegClass);
  Builder.buildInstr(MOS::LDAbs)
      .addDef(Lo)
      .addFrameIndex(FI)
      .addMemOperand(MF.getMachineMemOperand(MMO, 0, 1));
  Builder.buildInstr(MOS::LDAbs)
      .addDef(Hi)
      .addFrameIndex(FI, 1)
      .addMemOperand(MF.getMachineMemOperand(MMO, 1, 1));
  Builder.buildInstr(MOS::REG_SEQUENCE)
      .addDef(Reg)
      .addUse(Lo)
      .addImm(MOS::sublo)
      .addUse(Hi)
      .addImm(MOS::subhi);
}

} // namespace

char MOSImagRegSpill::ID = 0;
INITIALIZE_PASS_BEGIN(MOSImagRegSpill, DEBUG_TYPE,
                      "MOS Imaginary Register Spilling", false, false)
INITIALIZE_PASS_DEPENDENCY(LiveVariablesWrapperPass)
INITIALIZE_PASS_DEPENDENCY(MachineRegisterClassInfoWrapperPass)
INITIALIZE_PASS_END(MOSImagRegSpill, DEBUG_TYPE,
                    "MOS Imaginary Register Spilling", false, false)

MachineFunctionPass *llvm::createMOSImagRegSpillPass() {
  return new MOSImagRegSpill;
}
