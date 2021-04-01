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

#include "MOS.h"

#include "llvm/CodeGen/GlobalISel/Combiner.h"
#include "llvm/CodeGen/GlobalISel/CombinerHelper.h"
#include "llvm/CodeGen/GlobalISel/CombinerInfo.h"
#include "llvm/CodeGen/GlobalISel/GISelKnownBits.h"
#include "llvm/CodeGen/GlobalISel/Utils.h"
#include "llvm/CodeGen/MachineDominators.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/TargetOpcodes.h"
#include "llvm/CodeGen/TargetPassConfig.h"
#include "llvm/IR/Function.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Target/TargetMachine.h"

#define DEBUG_TYPE "mos-prelegalizer-combiner"

using namespace llvm;

class MOSCombinerHelperState {
protected:
  CombinerHelper &Helper;

public:
  MOSCombinerHelperState(CombinerHelper &Helper) : Helper(Helper) {}
};

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
      : CombinerInfo(/*AllowIllegalOps*/ true, /*ShouldLegalizeIllegal*/ false,
                     /*LegalizerInfo*/ nullptr, EnableOpt, OptSize, MinSize),
        KB(KB), MDT(MDT) {
    if (!GeneratedRuleCfg.parseCommandLineOption())
      report_fatal_error("Invalid rule identifier");
  }

  virtual bool combine(GISelChangeObserver &Observer, MachineInstr &MI,
                       MachineIRBuilder &B) const override;
};

bool MOSCombinerInfo::combine(GISelChangeObserver &Observer,
                                  MachineInstr &MI, MachineIRBuilder &B) const {
  CombinerHelper Helper(Observer, B, KB, MDT);
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
  const Function &F = MF.getFunction();
  bool EnableOpt =
      MF.getTarget().getOptLevel() != CodeGenOpt::None && !skipFunction(F);
  GISelKnownBits *KB = &getAnalysis<GISelKnownBitsAnalysis>().get(MF);
  MachineDominatorTree *MDT = &getAnalysis<MachineDominatorTree>();
  MOSCombinerInfo PCInfo(EnableOpt, F.hasOptSize(), F.hasMinSize(), KB,
                             MDT);
  Combiner C(PCInfo, TPC);
  return C.combineMachineInstrs(MF, /*CSEInfo*/ nullptr);
}

char MOSCombiner::ID = 0;
INITIALIZE_PASS_BEGIN(MOSCombiner, DEBUG_TYPE,
                      "Combine MOS machine instrs", false, false)
INITIALIZE_PASS_DEPENDENCY(TargetPassConfig)
INITIALIZE_PASS_DEPENDENCY(GISelKnownBitsAnalysis)
INITIALIZE_PASS_END(MOSCombiner, DEBUG_TYPE,
                    "Combine MOS machine instrs", false, false)

namespace llvm {
FunctionPass *createMOSCombiner() { return new MOSCombiner; }
} // namespace llvm
