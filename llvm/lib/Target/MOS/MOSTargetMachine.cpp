//===-- MOSTargetMachine.cpp - Define TargetMachine for MOS ---------------===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file defines the MOS specific subclass of TargetMachine.
//
//===----------------------------------------------------------------------===//

#include "MOSTargetMachine.h"

#include "llvm/CodeGen/GlobalISel/CSEInfo.h"
#include "llvm/CodeGen/GlobalISel/IRTranslator.h"
#include "llvm/CodeGen/GlobalISel/InstructionSelect.h"
#include "llvm/CodeGen/GlobalISel/Legalizer.h"
#include "llvm/CodeGen/GlobalISel/RegBankSelect.h"
#include "llvm/CodeGen/Passes.h"
#include "llvm/CodeGen/TargetPassConfig.h"
#include "llvm/IR/LegacyPassManager.h"
#include "llvm/IR/Module.h"
#include "llvm/InitializePasses.h"
#include "llvm/Passes/PassBuilder.h"
#include "llvm/Support/TargetRegistry.h"
#include "llvm/Transforms/Scalar/IndVarSimplify.h"
#include "llvm/Transforms/Utils.h"

#include "MCTargetDesc/MOSMCTargetDesc.h"
#include "MOS.h"
#include "MOSCombiner.h"
#include "MOSIndexIV.h"
#include "MOSLowerSelect.h"
#include "MOSMachineScheduler.h"
#include "MOSNoRecurse.h"
#include "MOSStaticStackAlloc.h"
#include "MOSTargetObjectFile.h"
#include "MOSTargetTransformInfo.h"

using namespace llvm;

extern "C" void LLVM_EXTERNAL_VISIBILITY LLVMInitializeMOSTarget() {
  // Register the target.
  RegisterTargetMachine<MOSTargetMachine> X(getTheMOSTarget());

  PassRegistry &PR = *PassRegistry::getPassRegistry();
  initializeGlobalISel(PR);
  initializeMOSCombinerPass(PR);
  initializeMOSLowerSelectPass(PR);
  initializeMOSNoRecursePass(PR);
  initializeMOSStaticStackAllocPass(PR);
}

static const char *MOSDataLayout =
    "e-p:16:8-i16:8-i32:8-i64:8-f32:8-f64:8-a:8-Fi8-n8";

/// Processes a CPU name.
static StringRef getCPU(StringRef CPU) {
  return (CPU.empty() || CPU == "generic") ? "mos6502" : CPU;
}

static Reloc::Model getEffectiveRelocModel(Optional<Reloc::Model> RM) {
  return RM.hasValue() ? *RM : Reloc::Static;
}

MOSTargetMachine::MOSTargetMachine(const Target &T, const Triple &TT,
                                   StringRef CPU, StringRef FS,
                                   const TargetOptions &Options,
                                   Optional<Reloc::Model> RM,
                                   Optional<CodeModel::Model> CM,
                                   CodeGenOpt::Level OL, bool JIT)
    : LLVMTargetMachine(T, MOSDataLayout, TT, getCPU(CPU), FS, Options,
                        getEffectiveRelocModel(RM),
                        getEffectiveCodeModel(CM, CodeModel::Small), OL),
      SubTarget(TT, getCPU(CPU).str(), FS.str(), *this) {
  this->TLOF = std::make_unique<MOSTargetObjectFile>();

  initAsmInfo();

  setGlobalISel(true);
  // Prevents fallback to SelectionDAG by allowing direct aborts.
  setGlobalISelAbort(GlobalISelAbortMode::Enable);
}

const MOSSubtarget *MOSTargetMachine::getSubtargetImpl() const {
  return &SubTarget;
}

const MOSSubtarget *MOSTargetMachine::getSubtargetImpl(const Function &) const {
  return &SubTarget;
}

TargetTransformInfo
MOSTargetMachine::getTargetTransformInfo(const Function &F) {
  return TargetTransformInfo(MOSTTIImpl(this, F));
}

void MOSTargetMachine::registerPassBuilderCallbacks(PassBuilder &PB,
                                                    bool DebugPassManager) {
  PB.registerPipelineParsingCallback(
      [](StringRef Name, LoopPassManager &PM,
         ArrayRef<PassBuilder::PipelineElement>) {
        if (Name == "mos-indexiv") {
          // Rewrite pointer artithmetic in loops to use 8-bit IV offsets.
          PM.addPass(MOSIndexIV());
          return true;
        }
        return false;
      });

  PB.registerLateLoopOptimizationsEPCallback(
      [](LoopPassManager &PM, PassBuilder::OptimizationLevel Level) {
        if (Level != PassBuilder::OptimizationLevel::O0) {
          PM.addPass(MOSIndexIV());

          // New induction variables may have been added.
          PM.addPass(IndVarSimplifyPass());
        }
      });
}

//===----------------------------------------------------------------------===//
// Pass Pipeline Configuration
//===----------------------------------------------------------------------===//

namespace {
/// MOS Code Generator Pass Configuration Options.
class MOSPassConfig : public TargetPassConfig {
public:
  MOSPassConfig(MOSTargetMachine &TM, PassManagerBase &PM)
      : TargetPassConfig(TM, PM) {}

  MOSTargetMachine &getMOSTargetMachine() const {
    return getTM<MOSTargetMachine>();
  }

  void addIRPasses() override;
  bool addPreISel() override;
  bool addIRTranslator() override;
  void addPreLegalizeMachineIR() override;
  bool addLegalizeMachineIR() override;
  void addPreRegBankSelect() override;
  bool addRegBankSelect() override;
  bool addGlobalInstructionSelect() override;
  void addMachineSSAOptimization() override;
  bool addRegAssignAndRewriteOptimized() override;
  void addPreSched2() override;
  void addPreEmitPass() override;

  ScheduleDAGInstrs *
  createMachineScheduler(MachineSchedContext *C) const override;

  std::unique_ptr<CSEConfigBase> getCSEConfig() const override;
};
} // namespace

TargetPassConfig *MOSTargetMachine::createPassConfig(PassManagerBase &PM) {
  return new MOSPassConfig(*this, PM);
}

void MOSPassConfig::addIRPasses() {
  // Aggressively find provably non-recursive functions.
  addPass(createMOSNoRecursePass());
  TargetPassConfig::addIRPasses();
}

bool MOSPassConfig::addPreISel() {
  addPass(createLowerSwitchPass());
  return false;
}

bool MOSPassConfig::addIRTranslator() {
  addPass(new IRTranslator(getOptLevel()));
  return false;
}

void MOSPassConfig::addPreLegalizeMachineIR() { addPass(createMOSCombiner()); }

bool MOSPassConfig::addLegalizeMachineIR() {
  addPass(new Legalizer());
  return false;
}

void MOSPassConfig::addPreRegBankSelect() {
  addPass(createMOSCombiner());
  addPass(createMOSLowerSelectPass());
}

bool MOSPassConfig::addRegBankSelect() {
  addPass(new RegBankSelect());
  return false;
}

bool MOSPassConfig::addGlobalInstructionSelect() {
  addPass(new InstructionSelect());
  return false;
}

void MOSPassConfig::addMachineSSAOptimization() {
  // Ensures that phsyreg defs are appropriately tagged with "dead", allowing
  // later SSA optimizations to ignore them. It's a little odd that these passes
  // use dead annotations; I think they're generated by SelectionDAG emission,
  // but there doesn't seem to be anything in GlobalISel that produces them in a
  // uniform fashion.
  addPass(&LiveVariablesID);
  TargetPassConfig::addMachineSSAOptimization();
}

bool MOSPassConfig::addRegAssignAndRewriteOptimized() {
  bool Result = TargetPassConfig::addRegAssignAndRewriteOptimized();

  // Clean up BUNDLE instructions emitted by spilling in the register allocator.
  addPass(createUnpackMachineBundles(
      [](const MachineFunction &MF) { return true; }));

  return Result;
}

void MOSPassConfig::addPreSched2() {
  // Lower control flow pseudos.
  addPass(&FinalizeISelID);
  // Lower pseudos produced by control flow pseudos.
  addPass(&ExpandPostRAPseudosID);

  addPass(createMOSStaticStackAllocPass());
}

void MOSPassConfig::addPreEmitPass() { addPass(&BranchRelaxationPassID); }

ScheduleDAGInstrs *
MOSPassConfig::createMachineScheduler(MachineSchedContext *C) const {
  return new ScheduleDAGMILive(C, std::make_unique<MOSSchedStrategy>(C));
}

std::unique_ptr<CSEConfigBase> MOSPassConfig::getCSEConfig() const {
  return getStandardCSEConfigForOpt(TM->getOptLevel());
}
