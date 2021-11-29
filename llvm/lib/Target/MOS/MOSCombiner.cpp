//===-- MOSCombiner.cpp - MOS GlobalIsel Combiner -------------------------===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file declares the MOS global machine instruction combiner.
//
// This runs between phases of the GlobalISel process to optimize away
// inefficient patterns discovered in the global machine instructions.
//
//===----------------------------------------------------------------------===//

#include "MOSCombiner.h"

#include "MCTargetDesc/MOSMCTargetDesc.h"
#include "MOS.h"
#include "MOSLegalizerInfo.h"

#include "llvm/CodeGen/GlobalISel/Combiner.h"
#include "llvm/CodeGen/GlobalISel/CombinerHelper.h"
#include "llvm/CodeGen/GlobalISel/CombinerInfo.h"
#include "llvm/CodeGen/GlobalISel/GISelChangeObserver.h"
#include "llvm/CodeGen/GlobalISel/GISelKnownBits.h"
#include "llvm/CodeGen/GlobalISel/LegalizerHelper.h"
#include "llvm/CodeGen/GlobalISel/MIPatternMatch.h"
#include "llvm/CodeGen/GlobalISel/Utils.h"
#include "llvm/CodeGen/MachineDominators.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/TargetInstrInfo.h"
#include "llvm/CodeGen/TargetOpcodes.h"
#include "llvm/CodeGen/TargetPassConfig.h"
#include "llvm/IR/Function.h"
#include "llvm/IR/PatternMatch.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Target/TargetMachine.h"

#define DEBUG_TYPE "mos-combiner"

using namespace llvm;

class MOSCombinerHelperState {
protected:
  CombinerHelper &Helper;

public:
  MOSCombinerHelperState(CombinerHelper &Helper) : Helper(Helper) {}

  // G_PTR_ADD (GLOBAL_VALUE @x + y_const), z_const =>
  // GLOBAL_VALUE @x + (y_const + z_const)
  bool matchFoldGlobalOffset(
      MachineInstr &MI, MachineRegisterInfo &MRI,
      std::pair<const MachineOperand *, int64_t> &MatchInfo) const;
  // G_PTR_ADD (GLOBAL_VALUE @x + y_const), z_const =>
  // GLOBAL_VALUE @x + (y_const + z_const)
  bool applyFoldGlobalOffset(
      MachineInstr &MI, MachineRegisterInfo &MRI, MachineIRBuilder &B,
      GISelChangeObserver &Observer,
      std::pair<const MachineOperand *, int64_t> &MatchInfo) const;

  bool matchSBCEqual(MachineInstr &MI, MachineRegisterInfo &MRI) const;
  bool applySBCEqual(MachineInstr &MI, MachineRegisterInfo &MRI,
                     MachineIRBuilder &B, GISelChangeObserver &Observer) const;

  bool matchExtractLowBit(MachineInstr &MI, MachineRegisterInfo &MRI,
                          MachineInstr *&Shift) const;
  bool applyExtractLowBit(MachineInstr &MI, MachineRegisterInfo &MRI,
                          MachineIRBuilder &B, GISelChangeObserver &Observer,
                          MachineInstr *&Shift) const;
};

// G_PTR_ADD (GLOBAL_VALUE @x + y_const), z_const =>
// GLOBAL_VALUE @x + (y_const + z_const)
bool MOSCombinerHelperState::matchFoldGlobalOffset(
    MachineInstr &MI, MachineRegisterInfo &MRI,
    std::pair<const MachineOperand *, int64_t> &MatchInfo) const {
  using namespace TargetOpcode;
  assert(MI.getOpcode() == G_PTR_ADD);

  Register Base = MI.getOperand(1).getReg();
  Register Offset = MI.getOperand(2).getReg();

  MachineInstr *GlobalBase = getOpcodeDef(G_GLOBAL_VALUE, Base, MRI);
  auto ConstOffset = getIConstantVRegValWithLookThrough(Offset, MRI);

  if (!GlobalBase || !ConstOffset)
    return false;
  const MachineOperand *BaseGV = &GlobalBase->getOperand(1);
  int64_t NewOffset = BaseGV->getOffset() + ConstOffset->Value.getSExtValue();
  MatchInfo = {BaseGV, NewOffset};
  return true;
}

// G_PTR_ADD (GLOBAL_VALUE @x + y_const), z_const =>
// GLOBAL_VALUE @x + (y_const + z_const)
bool MOSCombinerHelperState::applyFoldGlobalOffset(
    MachineInstr &MI, MachineRegisterInfo &MRI, MachineIRBuilder &B,
    GISelChangeObserver &Observer,
    std::pair<const MachineOperand *, int64_t> &MatchInfo) const {
  using namespace TargetOpcode;
  assert(MI.getOpcode() == G_PTR_ADD);
  const TargetInstrInfo &TII = B.getTII();
  Observer.changingInstr(MI);
  MI.setDesc(TII.get(TargetOpcode::G_GLOBAL_VALUE));
  MI.getOperand(1).ChangeToGA(MatchInfo.first->getGlobal(), MatchInfo.second,
                              MatchInfo.first->getTargetFlags());
  MI.RemoveOperand(2);
  Observer.changedInstr(MI);
  return true;
}

bool MOSCombinerHelperState::matchSBCEqual(MachineInstr &MI,
                                           MachineRegisterInfo &MRI) const {
  assert(MI.getOpcode() == MOS::G_SBC);
  Register LHS = MI.getOperand(5).getReg();
  Register RHS = MI.getOperand(6).getReg();
  Register CarryIn = MI.getOperand(7).getReg();

  auto ConstCarryIn = getIConstantVRegValWithLookThrough(CarryIn, MRI);
  if (!ConstCarryIn)
    return false;
  if (!ConstCarryIn->Value.isAllOnesValue())
    return false;

  if (LHS == RHS)
    return true;

  auto ConstLHS = getIConstantVRegValWithLookThrough(LHS, MRI);
  auto ConstRHS = getIConstantVRegValWithLookThrough(RHS, MRI);
  if (!ConstLHS || !ConstRHS)
    return false;

  return ConstLHS->Value == ConstRHS->Value;
}

bool MOSCombinerHelperState::applySBCEqual(
    MachineInstr &MI, MachineRegisterInfo &MRI, MachineIRBuilder &B,
    GISelChangeObserver &Observer) const {
  LLT S1 = LLT::scalar(1);

  B.setInsertPt(*MI.getParent(), MI);
  B.buildCopy(MI.getOperand(0), B.buildConstant(LLT::scalar(8), 0));

  auto S1Zero = B.buildConstant(S1, 0);
  // C
  B.buildCopy(MI.getOperand(1), S1Zero);
  // N
  B.buildCopy(MI.getOperand(2), S1Zero);
  // V
  B.buildCopy(MI.getOperand(3), S1Zero);
  // Z
  B.buildCopy(MI.getOperand(4), B.buildConstant(S1, -1));
  MI.eraseFromParent();
  return true;
}

bool MOSCombinerHelperState::matchExtractLowBit(MachineInstr &MI,
                                                MachineRegisterInfo &MRI,
                                                MachineInstr *&Shift) const {
  using namespace MIPatternMatch;
  Register Src;
  if (MI.getOpcode() == MOS::G_TRUNC) {
    if (MRI.getType(MI.getOperand(0).getReg()) != LLT::scalar(1))
      return false;
    Src = MI.getOperand(1).getReg();
    Register NewSrc;
    if (mi_match(Src, MRI,
                 m_GAnd(m_Reg(NewSrc), MIPatternMatch::m_SpecificICst(1))))
      Src = NewSrc;
  } else {
    assert(MI.getOpcode() == MOS::G_ICMP);
    ICmpInst::Predicate Pred;
    if (!mi_match(MI.getOperand(0).getReg(), MRI,
                  m_GICmp(m_Pred(Pred),
                          m_GAnd(m_Reg(Src), MIPatternMatch::m_SpecificICst(1)),
                          MIPatternMatch::m_SpecificICst(0))))
      return false;
    // The NE case handled automatically via an optimization that converts it to
    // a G_TRUNC.
    if (Pred != CmpInst::ICMP_EQ)
      return false;
  }

  for (MachineInstr &RefMI : MRI.reg_nodbg_instructions(Src)) {
    if (RefMI.getOpcode() != MOS::G_LSHR)
      continue;
    if (RefMI.getOperand(1).getReg() != Src)
      continue;
    auto ConstAmt =
        getIConstantVRegValWithLookThrough(RefMI.getOperand(2).getReg(), MRI);
    if (!ConstAmt || !ConstAmt->Value.isOne())
      continue;
    if (!Helper.dominates(RefMI, MI) && !Helper.dominates(MI, RefMI))
      continue;
    Shift = &RefMI;
    return true;
  }

  return false;
}

bool MOSCombinerHelperState::applyExtractLowBit(MachineInstr &MI,
                                                MachineRegisterInfo &MRI,
                                                MachineIRBuilder &B,
                                                GISelChangeObserver &Observer,
                                                MachineInstr *&Shift) const {
  assert(Shift->getOpcode() == MOS::G_LSHR);
  LLT S1 = LLT::scalar(1);

  bool Negate = MI.getOpcode() == MOS::G_ICMP &&
                MI.getOperand(1).getPredicate() == CmpInst::ICMP_EQ;

  if (Helper.dominates(*Shift, MI)) {
    B.setInsertPt(*Shift->getParent(), *Shift);
  } else {
    assert(Helper.dominates(MI, *Shift));
    B.setInsertPt(*MI.getParent(), MI);
  }

  auto EvenShift = B.buildInstr(MOS::G_LSHRE, {Shift->getOperand(0), S1},
                                {Shift->getOperand(1), B.buildConstant(S1, 0)});
  if (Negate)
    B.buildNot(MI.getOperand(0).getReg(), EvenShift.getReg(1));
  else
    B.buildCopy(MI.getOperand(0).getReg(), EvenShift.getReg(1));
  MOSLegalizerInfo Legalizer;
  LegalizerHelper LegalizerHelper(B.getMF(), Legalizer, Observer, B);
  if (!Legalizer.legalizeLshrEShlE(LegalizerHelper, MRI, *EvenShift))
    return false;
  Shift->eraseFromParent();
  MI.eraseFromParent();
  return true;
}

#define MOSCOMBINERHELPER_GENCOMBINERHELPER_DEPS
#include "MOSGenGICombiner.inc"
#undef MOSCOMBINERHELPER_GENCOMBINERHELPER_DEPS

namespace {
#define MOSCOMBINERHELPER_GENCOMBINERHELPER_H
#include "MOSGenGICombiner.inc"
#undef MOSCOMBINERHELPER_GENCOMBINERHELPER_H

class MOSCombinerInfo : public CombinerInfo {
  GISelKnownBits *KB;
  MachineDominatorTree *MDT;
  MOSGenCombinerHelperRuleConfig GeneratedRuleCfg;

public:
  MOSCombinerInfo(bool EnableOpt, bool OptSize, bool MinSize,
                  GISelKnownBits *KB, MachineDominatorTree *MDT)
      : CombinerInfo(/*AllowIllegalOps*/ true,
                     /*ShouldLegalizeIllegal*/ false,
                     /*LegalizerInfo*/ nullptr, EnableOpt, OptSize, MinSize),
        KB(KB), MDT(MDT) {
    if (!GeneratedRuleCfg.parseCommandLineOption())
      report_fatal_error("Invalid rule identifier");
  }

  virtual bool combine(GISelChangeObserver &Observer, MachineInstr &MI,
                       MachineIRBuilder &B) const override;
};

bool MOSCombinerInfo::combine(GISelChangeObserver &Observer, MachineInstr &MI,
                              MachineIRBuilder &B) const {
  const LegalizerInfo *LI = MI.getMF()->getSubtarget().getLegalizerInfo();
  if (!MI.getMF()->getProperties().hasProperty(
          MachineFunctionProperties::Property::Legalized))
    LI = nullptr;
  CombinerHelper Helper(Observer, B, KB, MDT, LI);
  MOSGenCombinerHelper Generated(GeneratedRuleCfg, Helper);
  return Generated.tryCombineAll(Observer, MI, B);
}

#define MOSCOMBINERHELPER_GENCOMBINERHELPER_CPP
#include "MOSGenGICombiner.inc"
#undef MOSCOMBINERHELPER_GENCOMBINERHELPER_CPP

// Pass boilerplate
// ================

class MOSCombiner : public MachineFunctionPass {
public:
  static char ID;

  MOSCombiner();

  StringRef getPassName() const override { return "MOSCombiner"; }

  bool runOnMachineFunction(MachineFunction &MF) override;

  void getAnalysisUsage(AnalysisUsage &AU) const override;
};
} // end anonymous namespace

void MOSCombiner::getAnalysisUsage(AnalysisUsage &AU) const {
  AU.addRequired<TargetPassConfig>();
  AU.setPreservesCFG();
  getSelectionDAGFallbackAnalysisUsage(AU);
  AU.addRequired<GISelKnownBitsAnalysis>();
  AU.addPreserved<GISelKnownBitsAnalysis>();
  AU.addRequired<MachineDominatorTree>();
  AU.addPreserved<MachineDominatorTree>();
  AU.addRequired<GISelCSEAnalysisWrapperPass>();
  AU.addPreserved<GISelCSEAnalysisWrapperPass>();
  MachineFunctionPass::getAnalysisUsage(AU);
}

MOSCombiner::MOSCombiner() : MachineFunctionPass(ID) {
  initializeMOSCombinerPass(*PassRegistry::getPassRegistry());
}

bool MOSCombiner::runOnMachineFunction(MachineFunction &MF) {
  if (MF.getProperties().hasProperty(
          MachineFunctionProperties::Property::FailedISel))
    return false;

  auto *TPC = &getAnalysis<TargetPassConfig>();

  // Enable CSE.
  GISelCSEAnalysisWrapper &Wrapper =
      getAnalysis<GISelCSEAnalysisWrapperPass>().getCSEWrapper();
  auto *CSEInfo = &Wrapper.get(TPC->getCSEConfig());

  const Function &F = MF.getFunction();
  bool EnableOpt =
      MF.getTarget().getOptLevel() != CodeGenOpt::None && !skipFunction(F);
  GISelKnownBits *KB = &getAnalysis<GISelKnownBitsAnalysis>().get(MF);
  MachineDominatorTree *MDT = &getAnalysis<MachineDominatorTree>();
  MOSCombinerInfo PCInfo(EnableOpt, F.hasOptSize(), F.hasMinSize(), KB, MDT);
  Combiner C(PCInfo, TPC);
  return C.combineMachineInstrs(MF, CSEInfo);
}

char MOSCombiner::ID = 0;
INITIALIZE_PASS_BEGIN(MOSCombiner, DEBUG_TYPE, "Combine MOS machine instrs",
                      false, false)
INITIALIZE_PASS_DEPENDENCY(TargetPassConfig)
INITIALIZE_PASS_DEPENDENCY(GISelKnownBitsAnalysis)
INITIALIZE_PASS_END(MOSCombiner, DEBUG_TYPE, "Combine MOS machine instrs",
                    false, false)

namespace llvm {
FunctionPass *createMOSCombiner() { return new MOSCombiner; }
} // namespace llvm
