//===----------------------------------------------------------------------===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
///
/// \file
/// Spill values to make imaginary register demands feasible, accounting for
/// both local pressure and global PHI constraints. MOSImagRegAssign chooses
/// imaginary registers. This pass preserves SSA and does not assign registers.
///
/// The walk records transfers without changing MIR or its liveness. Emission
/// then inserts stack accesses and renames uses to the latest reload. Currently
/// only one block with ordinary virtual definitions and uses is supported;
/// subregister operands, ties, PHIs, parallel copies, and physical imaginary
/// register constraints are not yet handled. Implicit hardware register effects
/// do not constrain imaginary storage.
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
  void spillForMI(MachineInstr &MI, MOSInterferenceGraph &Graph);
  void evict(Register Reg, MachineInstr &Before);
  void insertSpillsAndReloads(MachineBasicBlock &MBB);
  int getOrCreateSpillSlot(Register Reg);

  // Print inserted instructions and record that MIR changed.
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
          if (!MO.isImplicit() || MOS::Imag16RegClass.contains(Reg) ||
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
  MOSInterferenceGraph Graph(*MBB.getParent(), *RCI);
  Builder.beginCapture(Graph);
  for (MachineInstr &MI : MBB)
    if (!MI.isDebugOrPseudoInstr()) {
      spillForMI(MI, Graph);
      Builder.trimToLive();
    }
  Builder.endCapture();
}

void MOSImagRegSpill::spillForMI(MachineInstr &MI,
                                 MOSInterferenceGraph &Graph) {
  SmallVector<Register> Reloads = Builder.nonResidentUses(MI);
  if (!Reloads.empty()) {
    auto &Transfers = PlannedTransfers[&MI];
    Transfers.Reloads.append(Reloads);
    for (Register Reg : Reloads)
      Builder.reload(Reg);
  }

  // Soft-stack transfers need an address pair; static-stack transfers can
  // need a byte to extract a pair component or widen a flag. This space also
  // covers reload temporaries. Emitted transfers are not visited by this walk.
  const TargetRegisterClass &ScratchRC = TFL->usesStaticStack(*MI.getMF())
                                             ? MOS::Imag8RegClass
                                             : MOS::Imag16RegClass;

  // It must be possible to issue spills and reloads before the instruction and
  // for the next instruction to insert spills and reloads after this
  // instruction. This instruction must account for both, since ensuring that
  // property may require spilling its live ins, and if we leave the latter to
  // the next instruction, it may be too late.
  Builder.reserveHeadroom(ScratchRC);
  Builder.stepForward(MI);
  Builder.reserveHeadroom(ScratchRC);

  SmallVector<Register> Candidates = Builder.spillCandidates(MI);
  SmallVector<Register> RemainingRegs;
  while (!Graph.canSimplify(&RemainingRegs)) {
    auto Victim = llvm::find_if(RemainingRegs, [&](Register Reg) {
      return llvm::is_contained(Candidates, Reg);
    });
    if (Victim == RemainingRegs.end())
      report_fatal_error("MOS instruction pressure cannot be reduced by "
                         "spilling live-through registers",
                         false);
    Register Reg = *Victim;
    evict(Reg, MI);
    Graph.remove(Reg);
  }
}

void MOSImagRegSpill::evict(Register Reg, MachineInstr &Before) {
  if (!Builder.spillState().SavedResidentValues.contains(Reg))
    PlannedTransfers[&Before].Stores.push_back(Reg);

  // Note that the builder has already stepped past Before, but the value
  // remains evicted afterwards too.
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
      recordSpillInstructions(make_range(Span.begin(), Span.getInitial()));
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
  for (MachineInstr &MI : Instructions) {
    Changed = true;
    LLVM_DEBUG(dbgs() << "Inserted spill instruction: " << MI);
  }
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
